#include <AP_HAL/AP_HAL.h>

#if HAL_CPU_CLASS >= HAL_CPU_CLASS_150
#include "AP_NavEKF2.h"
#include "AP_NavEKF2_core.h"
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_RangeFinder/RangeFinder_Backend.h>

#include <stdio.h>

extern const AP_HAL::HAL& hal;


/********************************************************
*              OPT FLOW AND RANGE FINDER                *
********************************************************/

// Read the range finder and take new measurements if available
// Apply a median filter
void NavEKF2_core::readRangeFinder(void)
{
    uint8_t midIndex;
    uint8_t maxIndex;
    uint8_t minIndex;

    // get theoretical correct range when the vehicle is on the ground
    // don't allow range to go below 5cm because this can cause problems with optical flow processing
    rngOnGnd = MAX(frontend->_rng.ground_clearance_cm_orient(ROTATION_PITCH_270) * 0.01f, 0.05f);

    // read range finder at 20Hz
    // TODO better way of knowing if it has new data
    if ((imuSampleTime_ms - lastRngMeasTime_ms) > 50) {

        // reset the timer used to control the measurement rate
        lastRngMeasTime_ms =  imuSampleTime_ms;

        // store samples and sample time into a ring buffer if valid
        // use data from two range finders if available

        for (uint8_t sensorIndex = 0; sensorIndex <= 1; sensorIndex++) {
            AP_RangeFinder_Backend *sensor = frontend->_rng.get_backend(sensorIndex);
            if (sensor == nullptr) {
                continue;
            }
            if ((sensor->orientation() == ROTATION_PITCH_270) && (sensor->status() == RangeFinder::RangeFinder_Good)) {
                rngMeasIndex[sensorIndex] ++;
                if (rngMeasIndex[sensorIndex] > 2) {
                    rngMeasIndex[sensorIndex] = 0;
                }
                storedRngMeasTime_ms[sensorIndex][rngMeasIndex[sensorIndex]] = imuSampleTime_ms - 25;
                storedRngMeas[sensorIndex][rngMeasIndex[sensorIndex]] = sensor->distance_cm() * 0.01f;
            }

            // check for three fresh samples
            bool sampleFresh[2][3] = {};
            for (uint8_t index = 0; index <= 2; index++) {
                sampleFresh[sensorIndex][index] = (imuSampleTime_ms - storedRngMeasTime_ms[sensorIndex][index]) < 500;
            }

            // find the median value if we have three fresh samples
            if (sampleFresh[sensorIndex][0] && sampleFresh[sensorIndex][1] && sampleFresh[sensorIndex][2]) {
                if (storedRngMeas[sensorIndex][0] > storedRngMeas[sensorIndex][1]) {
                    minIndex = 1;
                    maxIndex = 0;
                } else {
                    minIndex = 0;
                    maxIndex = 1;
                }
                if (storedRngMeas[sensorIndex][2] > storedRngMeas[sensorIndex][maxIndex]) {
                    midIndex = maxIndex;
                } else if (storedRngMeas[sensorIndex][2] < storedRngMeas[sensorIndex][minIndex]) {
                    midIndex = minIndex;
                } else {
                    midIndex = 2;
                }

                // don't allow time to go backwards
                if (storedRngMeasTime_ms[sensorIndex][midIndex] > rangeDataNew.time_ms) {
                    rangeDataNew.time_ms = storedRngMeasTime_ms[sensorIndex][midIndex];
                }

                // limit the measured range to be no less than the on-ground range
                rangeDataNew.rng = MAX(storedRngMeas[sensorIndex][midIndex],rngOnGnd);

                // get position in body frame for the current sensor
                rangeDataNew.sensor_idx = sensorIndex;

                // write data to buffer with time stamp to be fused when the fusion time horizon catches up with it
                storedRange.push(rangeDataNew);

                // indicate we have updated the measurement
                rngValidMeaTime_ms = imuSampleTime_ms;

            } else if (!takeOffDetected && ((imuSampleTime_ms - rngValidMeaTime_ms) > 200)) {
                // before takeoff we assume on-ground range value if there is no data
                rangeDataNew.time_ms = imuSampleTime_ms;
                rangeDataNew.rng = rngOnGnd;
                rangeDataNew.time_ms = imuSampleTime_ms;

                // don't allow time to go backwards
                if (imuSampleTime_ms > rangeDataNew.time_ms) {
                    rangeDataNew.time_ms = imuSampleTime_ms;
                }

                // write data to buffer with time stamp to be fused when the fusion time horizon catches up with it
                storedRange.push(rangeDataNew);

                // indicate we have updated the measurement
                rngValidMeaTime_ms = imuSampleTime_ms;

            }
        }
    }
}

// write the raw optical flow measurements
// this needs to be called externally.
void NavEKF2_core::writeOptFlowMeas(const uint8_t rawFlowQuality, const Vector2f &rawFlowRates, const Vector2f &rawGyroRates, const uint32_t msecFlowMeas, const Vector3f &posOffset)
{
    // The raw measurements need to be optical flow rates in radians/second averaged across the time since the last update
    // The PX4Flow sensor outputs flow rates with the following axis and sign conventions:
    // A positive X rate is produced by a positive sensor rotation about the X axis
    // A positive Y rate is produced by a positive sensor rotation about the Y axis
    // This filter uses a different definition of optical flow rates to the sensor with a positive optical flow rate produced by a
    // negative rotation about that axis. For example a positive rotation of the flight vehicle about its X (roll) axis would produce a negative X flow rate
    flowMeaTime_ms = imuSampleTime_ms;
    // calculate bias errors on flow sensor gyro rates, but protect against spikes in data
    // reset the accumulated body delta angle and time
    // don't do the calculation if not enough time lapsed for a reliable body rate measurement
    if (delTimeOF > 0.01f) {
        flowGyroBias.x = 0.99f * flowGyroBias.x + 0.01f * constrain_float((rawGyroRates.x - delAngBodyOF.x/delTimeOF),-0.1f,0.1f);
        flowGyroBias.y = 0.99f * flowGyroBias.y + 0.01f * constrain_float((rawGyroRates.y - delAngBodyOF.y/delTimeOF),-0.1f,0.1f);
        delAngBodyOF.zero();
        delTimeOF = 0.0f;
    }
    // by definition if this function is called, then flow measurements have been provided so we
    // need to run the optical flow takeoff detection
    detectOptFlowTakeoff();

    // calculate rotation matrices at mid sample time for flow observations
    stateStruct.quat.rotation_matrix(Tbn_flow);
    // don't use data with a low quality indicator or extreme rates (helps catch corrupt sensor data)
    if ((rawFlowQuality > 0) && rawFlowRates.length() < 4.2f && rawGyroRates.length() < 4.2f) {
        // correct flow sensor body rates for bias and write
        ofDataNew.bodyRadXYZ.x = rawGyroRates.x - flowGyroBias.x;
        ofDataNew.bodyRadXYZ.y = rawGyroRates.y - flowGyroBias.y;
        // the sensor interface doesn't provide a z axis rate so use the rate from the nav sensor instead
        if (delTimeOF > 0.001f) {
            // first preference is to use the rate averaged over the same sampling period as the flow sensor
            ofDataNew.bodyRadXYZ.z = delAngBodyOF.z / delTimeOF;
        } else if (imuDataNew.delAngDT > 0.001f){
            // second preference is to use most recent IMU data
            ofDataNew.bodyRadXYZ.z = imuDataNew.delAng.z / imuDataNew.delAngDT;
        } else {
            // third preference is use zero
            ofDataNew.bodyRadXYZ.z =  0.0f;
        }
        // write uncorrected flow rate measurements
        // note correction for different axis and sign conventions used by the px4flow sensor
        ofDataNew.flowRadXY = - rawFlowRates; // raw (non motion compensated) optical flow angular rate about the X axis (rad/sec)
        // write the flow sensor position in body frame
        ofDataNew.body_offset = &posOffset;
        // write flow rate measurements corrected for body rates
        ofDataNew.flowRadXYcomp.x = ofDataNew.flowRadXY.x + ofDataNew.bodyRadXYZ.x;
        ofDataNew.flowRadXYcomp.y = ofDataNew.flowRadXY.y + ofDataNew.bodyRadXYZ.y;
        // record time last observation was received so we can detect loss of data elsewhere
        flowValidMeaTime_ms = imuSampleTime_ms;
        // estimate sample time of the measurement
        ofDataNew.time_ms = imuSampleTime_ms - frontend->_flowDelay_ms - frontend->flowTimeDeltaAvg_ms/2;
        // Correct for the average intersampling delay due to the filter updaterate
        ofDataNew.time_ms -= localFilterTimeStep_ms/2;
        // Prevent time delay exceeding age of oldest IMU data in the buffer
        ofDataNew.time_ms = MAX(ofDataNew.time_ms,imuDataDelayed.time_ms);
        // Save data to buffer
        storedOF.push(ofDataNew);
        // Check for data at the fusion time horizon
        flowDataToFuse = storedOF.recall(ofDataDelayed, imuDataDelayed.time_ms);
    }
}


/********************************************************
*                      MAGNETOMETER                     *
********************************************************/

// check for new magnetometer data and update store measurements if available
//¼ì²éĞÂµÄ´ÅÁ¦¼ÆÊı¾İ£¬Èç¹û¿ÉÓÃ£¬¸üĞÂ´æ´¢µÄ´ÅÁ¦¼ÆÁ¿²â
//ÎªÊ²Ã´ÕâÃ´Ğ´?Äã»áÔõÃ´Ğ´?²î¾à?ÕâÀïµÄÄãÄÜÓÃµ½ÄÄÀï?ÈçºÎÓÃÀ´ÓÅ»¯×Ô¼ºµÄ´úÂë?
void NavEKF2_core::readMagData()
{//Èç¹û¶Á²»µ½´ÅÊı¾İ£¬Ôò·µ»Ø
    if (!_ahrs->get_compass()) {
        allMagSensorsFailed = true;//×¢Òâ±äÁ¿ÆğÃûµÄ¼¼ÇÉ
        return;        
    }
    // If we are a vehicle with a sideslip constraint to aid yaw estimation and we have timed out on our last avialable
    // magnetometer, then declare the magnetometers as failed for this flight
    //Èç¹û³µÁ¾ÓĞ²à»¬Ô¼ÊøÈ¥¸¨Öúº½Ïò¹À¼ÆÇÒÉÏÊ±¿Ì¿ÉÓÃµÄ´ÅÊı¾İ³¬Ê±£¬ÄÇÃ´¿É¶Ï¶¨ÔÚ¸Ã´ÎÔË¶¯ÖĞ´ÅÊı¾İ²»¿ÉÓÃ
    uint8_t maxCount = _ahrs->get_compass()->get_count();//_ahrs_get_compass()·µ»ØÒ»¸öconst CompassÀàĞÍµÄÖ¸Õë£¬¸ÃÖ¸Õëµ÷ÓÃget_count()·½·¨
    if (allMagSensorsFailed || (magTimeout && assume_zero_sideslip() && magSelectIndex >= maxCount-1 && inFlight)) {
        allMagSensorsFailed = true;
        return;
    }

    // do not accept new compass data faster than 14Hz (nominal rate is 10Hz) to prevent high processor loading
    // because magnetometer fusion is an expensive step and we could overflow the FIFO buffer
    //¾Ü¾ø¸üĞÂÆµÂÊ´óÓÚ14HzµÄ´ÅÊı¾İ(±ê×¼¸üĞÂÆµÂÊÎª10Hz)£¬·ÀÖ¹´¦ÀíÆ÷¸ºµ£¹ıÖØ
    //ÒòÎª´ÅÊı¾İÈÚºÏ·ÑÄÚ´æ£¬·ÀÖ¹FIFO»º³åÇøÒç³ö
    //_ahrs->get_compass()·µ»ØÒ»¸öcompassÀàĞÍµÄÖ¸Õë£¬¸ÃÖ¸Õëµ÷ÓÃcompassÀàÖĞµÄº¯Êılast_update_usec()
    if (use_compass() && _ahrs->get_compass()->last_update_usec() - lastMagUpdate_us > 70000) {
        frontend->logging.log_compass = true;// ->µÄ×ó±ßÊÇÖ¸Õë£¬¾äµãµÄ×ó±ßÊÇ¶ÔÏó  frontnedÊÇNavEKF2ÀàĞÍµÄ¶ÔÏó loggingÊÇÒ»¸ö½á¹¹Ìå

        // If the magnetometer has timed out (been rejected too long) we find another magnetometer to use if available
        // Don't do this if we are on the ground because there can be magnetic interference and we need to know if there is a problem
        // before taking off. Don't do this within the first 30 seconds from startup because the yaw error could be due to large yaw gyro bias affsets
        //Èô´ÅÊı¾İ³¬Ê±»ò×Ô¼ìÒ»Ö±²»Í¨¹ı£¬´ËÊ±ÈôÓĞÁíÒ»¸ö´ÅÂŞÅÌÔòÇĞ»»
        //²»ÒªÔÚµØÃæÖ´ĞĞ´Ë²Ù×÷£¬ÒòÎªµØÃæ¿ÉÄÜ»áÓĞ´Å¸ÉÈÅ£»
        //²»ÒªÔÙÆğ·É30sÄÚÖ´ĞĞ´Ë²Ù×÷£¬ÒòÎª¸Õ¿ªÊ¼µÄº½ÏòÎó²î¿ÉÄÜÊÇÓÉ½Ï´óµÄÍÓÂİÆ«ÒÆÒıÆğµÄ
        if (magTimeout && (maxCount > 1) && !onGround && imuSampleTime_ms - ekfStartTime_ms > 30000) {

            // search through the list of magnetometers
            //Ñ°ÕÒ´ÅÂŞÅÌÁĞ±í
            for (uint8_t i=1; i<maxCount; i++) {
                uint8_t tempIndex = magSelectIndex + i;
                // loop back to the start index if we have exceeded the bounds
                //Èç¹û³¬³ö±ß½ç£¬Ôò»Øµ½ÆğÊ¼Ë÷Òı
                if (tempIndex >= maxCount) {
                    tempIndex -= maxCount;
                }
                // if the magnetometer is allowed to be used for yaw and has a different index, we start using it
                //Èç¹û´ÅÁ¦¼Æ¿ÉÓÃÓÚÆ«º½½Ç¼ÆËãÇÒÓĞÒ»¸ö²»Í¬µÄË÷Òı£¬Ôò¿ªÊ¼Ê¹ÓÃ´ÅÁ¦¼Æ
                if (_ahrs->get_compass()->use_for_yaw(tempIndex) && tempIndex != magSelectIndex) {
                    magSelectIndex = tempIndex;
                    gcs().send_text(MAV_SEVERITY_INFO, "EKF2 IMU%u switching to compass %u",(unsigned)imu_index,magSelectIndex);
                    // reset the timeout flag and timer ÖØÖÃ³¬Ê±±êÖ¾ºÍ¼ÆÊ±Æ÷
                    magTimeout = false;
                    lastHealthyMagTime_ms = imuSampleTime_ms;
                    // zero the learned magnetometer bias states
                    stateStruct.body_magfield.zero();
                    // clear the measurement buffer
                    storedMag.reset();
                    // clear the data waiting flag so that we do not use any data pending from the previous sensor
                    //Çå³ıÊı¾İµÈ´ı±êÖ¾£¬ÒÔ±ãÓÚÎÒÃÇ²»Ê¹ÓÃÏÈÇ°´«¸ĞÆ÷ÓĞ´ı´¦ÀíµÄÈÎºÎÊı¾İ
                    magDataToFuse = false;
                    // request a reset of the magnetic field states
                    magStateResetRequest = true;
                    // declare the field unlearned so that the reset request will be obeyed
                    magFieldLearned = false;
                }
            }
        }

        // detect changes to magnetometer offset parameters and reset states
        //¼ì²â´ÅÁ¦¼ÆÆ«ÒÆ²ÎÊıµÄ±ä»¯²¢ÖØÖÃ×´Ì¬ _ahrs->get_compass()·µ»ØµÄÊÇÒ»¸öCompassÀàĞÍµÄÖ¸Õë£¬¹Ê½ÓÏÂÀ´¿ÉÖ±½Óµ÷ÓÃCompassÀàÖĞµÄ³ÉÔ±º¯Êı
        Vector3f nowMagOffsets = _ahrs->get_compass()->get_offsets(magSelectIndex);
        bool changeDetected = lastMagOffsetsValid && (nowMagOffsets != lastMagOffsets);
        if (changeDetected) {
            // zero the learned magnetometer bias states
            stateStruct.body_magfield.zero();
            // clear the measurement buffer
            storedMag.reset();
        }
        lastMagOffsets = nowMagOffsets;
        lastMagOffsetsValid = true;

        // store time of last measurement update ´æ´¢ÉÏÒ»´ÎÁ¿²â¸üĞÂµÄÊ±¼ä
        lastMagUpdate_us = _ahrs->get_compass()->last_update_usec(magSelectIndex);

        // estimate of time magnetometer measurement was taken, allowing for delays
        //¹À¼Æ´ÅÁ¦¼Æ²âÁ¿µÄÊ±¼äÑÓ³Ù£¬ÔÊĞíÑÓ³Ù
        magDataNew.time_ms = imuSampleTime_ms - frontend->magDelay_ms;

        // Correct for the average intersampling delay due to the filter updaterate
        //ĞŞÕıÓÉÓÚÂË²¨Æ÷¸üĞÂµ¼ÖÂµÄÆ½¾ù²ÉÑùÊ±¼äÑÓ³Ù
        magDataNew.time_ms -= localFilterTimeStep_ms/2;

        // read compass data and scale to improve numerical conditioning
        //¶Á´ÅÁ¦¼ÆÊı¾İºÍ±ê¶ÈÒòÊı£¬ÓÃÀ´¸ÄÉÆÊıÖµ¾«¶È
        //return _state[i].field ÊÇget_fieldµÄº¯Êı¾ßÌåÊµÏÖ£¬_state[]ÊÇÒ»¸ö½á¹¹ÌåÊı×é£¬fieldµÄVector3fÀàĞÍ£¬±íÊ¾Ğ£ÕıºóµÄ´Å³¡Ç¿¶È
        //_ahrs->get_compass()·µ»ØÒ»¸öÖ¸ÏòCompassÀàĞÍµÄconstÖ¸Õë£¬È»ºó¸ÃÖ¸ÕëÔÙµ÷ÓÃCompassµÄ³ÉÔ±º¯Êıget_field
        magDataNew.mag = _ahrs->get_compass()->get_field(magSelectIndex) * 0.001f;

        // check for consistent data between magnetometers
        //_ahrs->get_compass·µ»ØÒ»¸öconstµÄCompassÀàĞÍµÄÖ¸Õë£¬Í¨¹ı¸ÃÖ¸Õëµ÷ÓÃCompassÀàµÄ³ÉÔ±·½·¨consistent
        consistentMagData = _ahrs->get_compass()->consistent();

        // save magnetometer measurement to buffer to be fused later
        //½«´ÅÁ¦¼ÆÁ¿²â´æ´¢µ½bufÖĞ£¬µÈ´ıÖ®ºóÈÚºÏ
        storedMag.push(magDataNew);
    }
}

/********************************************************
*                Inertial Measurements                  *
********************************************************/

/*
 *  Read IMU delta angle and delta velocity measurements and downsample to 100Hz
 *  for storage in the data buffers used by the EKF. If the IMU data arrives at
 *  lower rate than 100Hz, then no downsampling or upsampling will be performed.
 *  Downsampling is done using a method that does not introduce coning or sculling
 *  errors.
 */
 // ¶ÁÈ¡IMU½Ç¶È²âÁ¿ºÍËÙ¶È²âÁ¿µÄdeltaÖµ£¬²ÉÑùÆµÂÊ100Hz£¬´æ´¢ÔÚEKFÊ¹ÓÃµÄÊı¾İ»º³åÇøÖĞ£¬
 //ÈôIMUÊı¾İµ½´ïµÄÆµÂÊµÍÓÚ100Hz£¬²»½øĞĞ½µ²ÉÑù»ò²»²ÉÑù¡£
 //ÔÚ²»ÒıÈëÔ²×¶Îó²î»ò»®´¬Îó²îµÄÇ°ÌáÏÂÍê³É½µ²ÉÑù
 
 //ÎªÊ²Ã´×÷ÕßÕâÃ´Ğ´?ÎÒ»áÔõÃ´Ğ´?ÎÒÑ§µ½ÁËÊ²Ã´?Ñ§µ½ÁË¿ÉÒÔÓÃµ½ÄÄÀï?ÔõÃ´ÓÅ»¯?
void NavEKF2_core::readIMUData()
{   //AP::ins()·µ»ØÒ»¸öAP_InertialSensorÀàĞÍµÄÒıÓÃ£¬²¢¸Ã½«¸ÃÒıÓÃ¸³Öµ¸øins¡£
    //APÊÇÒ»¸önamespace ¾ßÌåÊµÏÖÎªreturn *AP_InertialSensor::get_instance();
    const AP_InertialSensor &ins = AP::ins();//µÃµ½IMUÊı¾İ  APÊÇÃû³Æ¿Õ¼ä£¬insÊÇÃû³Æ¿Õ¼äÖĞÒ»¸öº¯Êı

    // average IMU sampling rate
    //Æ½¾ùIMUµÄ²ÉÑùËÙÂÊ  ·µ»ØÖ÷Ñ­»·µÄÁ½´ÎÑ­»·¼ä¸ô£¬µ¥Î»:s
    dtIMUavg = ins.get_loop_delta_t();

    // the imu sample time is used as a common time reference throughout the filter
    //IMUµÄ²ÉÑùÊ±¼äÓÃ×÷Õû¸öÂË²¨Æ÷µÄ¹«¹²Ê±¼ä»ù×¼
	imuSampleTime_ms = AP_HAL::millis();

    // use the nominated imu or primary if not available
    //Èç¹û²»¿ÉÓÃ£¬ÔòÊ¹ÓÃÖ¸¶¨µÄ»îÄ¬ÈÏµÄIMU
    if (ins.use_accel(imu_index)) {
		//´Ó¼ÓËÙ¶È»ñµÃ»ı·ÖËÙ¶È
		//accelPosOffset ÔØÌåÏµÖĞIMUµÄ°²×°Î»ÖÃ
        readDeltaVelocity(imu_index, imuDataNew.delVel, imuDataNew.delVelDT);
        accelPosOffset = ins.get_imu_pos_offset(imu_index);
    } else {
        readDeltaVelocity(ins.get_primary_accel(), imuDataNew.delVel, imuDataNew.delVelDT);
        accelPosOffset = ins.get_imu_pos_offset(ins.get_primary_accel());
    }

    // Get delta angle data from primary gyro or primary if not available
    //´ÓÍÓÂİÒÇ»ñµÃ»ı·Ö½Ç¶È
	if (ins.use_gyro(imu_index)) {
        readDeltaAngle(imu_index, imuDataNew.delAng, imuDataNew.delAngDT);
    } else {
        readDeltaAngle(ins.get_primary_gyro(), imuDataNew.delAng, imuDataNew.delAngDT);
    }

    // Get current time stamp
    //»ñÈ¡ÔËĞĞµ½´Ë´¦µÄÊ±¼ä
    imuDataNew.time_ms = imuSampleTime_ms;

    // Accumulate the measurement time interval for the delta velocity and angle data
    //»ıÀÛµÄ½Ç¶ÈºÍ½ÇËÙ¶ÈÊı¾İµÄ²âÁ¿Ê±¼ä  ÀÛ»ı»ı·ÖÊ±¼ä
    //imuDataDownSampleNew µ±Ç°Ê±¿ÌµÄIMUÊı¾İ£¬ÒÑ¾­±»½µ²ÉÑùµ½100Hz
    //imuDataNewµ±Ç°Ê±¿ÌµÄIMUÊı¾İ 
    //Êı¾İÀàĞÍ¶¨Òåå£º struct imu_elements{Vector3f delAng;Vector3f delVel;float delAngDT;float delVecDT;uint32_t time_ms}
	imuDataDownSampledNew.delAngDT += imuDataNew.delAngDT;
    imuDataDownSampledNew.delVelDT += imuDataNew.delVelDT;

    // Rotate quaternon atitude from previous to new and normalise.
    // Accumulation using quaternions prevents introduction of coning errors due to downsampling
    //µÃµ½ĞÂµÄĞı×ªËÄÔªËØ½Ç¶È£¬²¢¹éÒ»»¯
    //Ê¹ÓÃËÄÔªËØÀÛ»ı£¬·ÀÖ¹ÓÉÓÚ½µ²ÉÑùÒıÈëÔ²×¶Îó²î
    //ÀÛ»ıµÄ»ı·Ö½Ç¶È
    //imuQuatDownSampleNew ´Ó½µ²ÉÑù¿ªÊ¼ÀûÓÃIMUµÄ½Ç¶ÈÔöÁ¿»ñµÃµÄËÄÔªËØ
	imuQuatDownSampleNew.rotate(imuDataNew.delAng);//ÓÉ½Ç¶ÈÔöÁ¿ÇóËÄÔªÊı
    imuQuatDownSampleNew.normalize();//ËÄÔªÊı¹éÒ»»¯

    // Rotate the latest delta velocity into body frame at the start of accumulation
    //½«×î½üµÄ½Ç¶È²âÁ¿×ª»»µ½¸Õ¿ªÊ¼Ê±µÄÔØÌåÏµÖĞ  ¸ù¾İËÄÔªËØ³õÊ¼»¯×ËÌ¬¾ØÕó
    //ÇóÍÓÂİÒÇ±ä»¯µÄĞı×ª¾ØÕó£¬Ä¿µÄÊÇ°ÑÊı¾İµÄ¸Ä±äÍ¶Ó°¹ıÀ´  ÀûÓÃËÄÔªËØÇó´ËÊ±µÄ×ËÌ¬¾ØÕó
    //ÀûÓÃËÄÔªËØÇó×ËÌ¬¾ØÕó deltaRotMatÊÇËÄÔªËØ¼ÆËã³öµÄ×ËÌ¬¾ØÕó
	Matrix3f deltaRotMat;
    imuQuatDownSampleNew.rotation_matrix(deltaRotMat);

    // Apply the delta velocity to the delta velocity accumulator
    //½«ËÙ¶ÈÔöÁ¿¼Óµ½ËÙ¶ÈÀÛ¼ÓÆ÷ÖĞ ½«bÏµµÄËÙ¶ÈÔöÁ¿×ª»»µ½nÏµ
    //ÀÛ¼ÆµÄ»ı·ÖËÙ¶È ½«¼¸´ÎµÄËÙ¶ÈÔöÁ¿×ª»»µ½nÏµºóÀÛ¼Ó
    //½«bÏµÏÂËÙ¶ÈÔöÁ¿×ª»»µ½µ¼º½ÏµÏÂ
    imuDataDownSampledNew.delVel += deltaRotMat*imuDataNew.delVel;

    // Keep track of the number of IMU frames since the last state prediction
    //¼ÇÂ¼IMUÔ¤²âµÄ¸öÊı  Ò»´ÎÂË²¨¹ı³ÌÖĞÔöÁ¿ÀÛ¼ÓµÄ´ÎÊı¡£
    //ÓÃÓÚ½µ²ÉÑù£¬²ÉÑù¶àÉÙ´Î½øĞĞÒ»´Î½µ²ÉÑù
    framesSincePredict++;

    /*
     * If the target EKF time step has been accumulated, and the frontend has allowed start of a new predict cycle,
     * then store the accumulated IMU data to be used by the state prediction, ignoring the frontend permission if more
     * than twice the target time has lapsed. Adjust the target EKF step time threshold to allow for timing jitter in the
     * IMU data.
     */
     //Èç¹ûÄ¿±êEKFµÄÊ±¼ä²½³¤ÒÑ¾­ÀÛ»ıÇÒÇ°¶ËÔÊĞí¿ªÆôÒ»¸öĞÂµÄÔ¤²âÖÜÆÚ£¬´æ´¢ÀÛ»ıµÄIMUÊı¾İÓÃÓÚ×´Ì¬Ô¤²â
     //Èç¹û³¬¹ıÄ¿±êÊ±¼äµÄÁ½±¶£¬ÔòºöÂÔÇ°¶ËÇëÇó¡£µ÷ÕûEKFÊ±¼ä²½³¤µÄãĞÖµ£¬ÒÔÔÊĞíÔÚIMUÊı¾İÖĞµÄÊ±¼ä¶¶¶¯
     //EKF_TARGET_DT ÊÇÄ¿±êEKF¸üĞÂµÄÊ±¼ä²½³¤ dtIMUavg IMUÁ½´Î²âÁ¿Ö®¼äµÄÆÚÍûÊ±¼ä
     //EKF´¦ÀíimuÊı¾İ¼ä¸ô£¬²»ÓÃÃ¿´Î¶¼´¦Àí£¬´¦ÀíÆµÂÊÔ¼50Hz£¬GPSÆµÂÊÎª5Hz£¬ÆøÑ¹¼Æ10Hz
     //ÎªºÎ×ö´ËÅĞ¶Ï?
     //´ËÊ±frameSinsePredict¿ªÊ¼×ö½µ²ÉÑù£¬¿É¸ù¾İÅĞ¶ÏÌõ¼şµÃµ½framSincePredict
     //ÎªºÎÕâÃ´¶àÊ±¼äÅĞ¶Ï?
    if ((dtIMUavg*(float)framesSincePredict >= (EKF_TARGET_DT-(dtIMUavg*0.5)) &&
         startPredictEnabled) || (dtIMUavg*(float)framesSincePredict >= 2.0f*EKF_TARGET_DT)) {

        // convert the accumulated quaternion to an equivalent delta angle
        //×ª»»ÀÛ»ıµÄËÄÔªËØ£¬µÃµ½Ò»¸öµÈ¼ÛµÄ½ÇÔöÁ¿  ½«½ÇÔöÁ¿×ª»»ÎªÖá½Ç
        //Í¨¹ıimuÔÚÕâ¶ËÊ±¼äµÄ±ä»¯Á¿Çó³öĞı×ªµÄ»¡¶È´óĞ¡
        //imuDataDownSampleNew.delAngÖĞ´æ·ÅµÄÊÇÇóµÃµÄÖá½Ç
        imuQuatDownSampleNew.to_axis_angle(imuDataDownSampledNew.delAng);

        // Time stamp the data  ¼ÇÂ¼²ÉÑùÊ±¼ä
        //»ñÈ¡Ê±¼ä ²»ÊÇÊ±¼ä¼ä¸ô£¬Ó¦¸ÃÊÇ´Ó¿ª»úÒÔÀ´µÄÊ±¼ä
        imuDataDownSampledNew.time_ms = imuSampleTime_ms;

        // Write data to the FIFO IMU buffer ½«ĞÂÊı¾İĞ´Èë»º³åÇø
        //¼ÇÂ¼±¾´Î²ÉÑùÊı¾İµ½»·ĞÎbuff
        //ÎªºÎ°ÑÊı¾İĞ´ÈëbufÖĞ¶ø²»ÊÇÖ±½Ó´«µİ¼ÆËã?
        //imuDataDownSampleNew  struct imu_elements{Vector3f delAng;Vector3f delVel;float delAngDT;float delVelDT;uint_32t time_ms}
        storedIMU.push_youngest_element(imuDataDownSampledNew);

        // calculate the achieved average time step rate for the EKF
        //¼ÆËãEKFÊı¾İµÄÆ½¾ù²½³¤  dtEkfAvg Á½´ÎEKF¸üĞÂµÄÆÚÍûÊ±¼ä
        float dtNow = constrain_float(0.5f*(imuDataDownSampledNew.delAngDT+imuDataDownSampledNew.delVelDT),0.0f,10.0f*EKF_TARGET_DT);
        dtEkfAvg = 0.98f * dtEkfAvg + 0.02f * dtNow;

        // zero the accumulated IMU data and quaternion
        //½«ÀÛ»ıµÄËÄÔªËØºÍIMUÊı¾İÖÃÁã  Çé¿ö²ÉÑùÀÛ¼ÓÆ÷
        //imuDataDownSampleNewÒÑ±»´æÈë»·ĞÎbuf£¬´ËÊ±½«¸Ã½á¹¹ÌåÖĞÔªËØÇåÁã£¬ÏÂ´ÎÓÃÊ±Ö±½Ó´ÓbufÖĞÈ¡¼´¿É
        imuDataDownSampledNew.delAng.zero();
        imuDataDownSampledNew.delVel.zero();
        imuDataDownSampledNew.delAngDT = 0.0f;
        imuDataDownSampledNew.delVelDT = 0.0f;
        imuQuatDownSampleNew[0] = 1.0f;
        imuQuatDownSampleNew[3] = imuQuatDownSampleNew[2] = imuQuatDownSampleNew[1] = 0.0f;

        // reset the counter used to let the frontend know how many frames have elapsed since we started a new update cycle
        //ÖØÖÃ¼ÆÊıÆ÷ ÂË²¨Æ÷µÄÆµÂÊµÍÓÚIMU¸üĞÂµÄÆµÂÊ£¬¹Ê¶ÔIMU½øĞĞ½µ²ÉÑù£¬ÇóÆ½¾ùºóÔÙÂË²¨
        //½«±êÖ¾Î»ÇåÁã£¬´¿½İÁª¶àÉÙ´Î½øĞĞÒ»´ÎEKF
        framesSincePredict = 0;

        // set the flag to let the filter know it has new IMU data and needs to run
        //ÉèÖÃ±êÖ¾Î»£¬ÈÃÂË²¨Æ÷ÖªµÀĞÂµÄIMUÊı¾İÒÑ¾­µ½À´£¬ĞèÒªÔËĞĞ
        //runUpdatesÎªtrue£¬±íÃ÷¿ÉÒÔ½øĞĞEKF£¬ÕâÊÂEKFÖ®Ç°µÄ×´Ì¬ÅĞ¶Ï±êÖ¾Î»
        runUpdates = true;

        // extract the oldest available data from the FIFO buffer
        //´Ó»º³åÇøÈ¡³ö½ÏÀÏµÄ¿ÉÓÃµÄÊı¾İ
        //¶ÁÈ¡×î¾ÃµÄÒ»´Î²ÉÑù£¬Õâ¸ö¸ú×îĞÂµÄÊı¾İÏà²îÁËsize(13»ò26)¸ö²ÉÑù
        //ÎªºÎÈ¡³ö×î¾ÉµÄÊı¾İ£¬²»Ó¦¸ÃÊÇ¸Õ¸Õ´æÈëµÄÊı¾İÂğ?
        imuDataDelayed = storedIMU.pop_oldest_element();

        // protect against delta time going to zero
        // TODO - check if calculations can tolerate 0
        //dtEkfAvg Á½´ÎEKF¸üĞÂµÄÊ±¼ä¼ä¸ô
        //ÕâÑù¿ÉÒÔ±£Ö¤delta_timeÓÀÔ¶²»»áÎªÁã
        //delAngDTºÍdelVelDTÓ¦¸ÃÊÇÒ»ÑùµÄ£¬ÎªºÎ¶¨ÒåÁ½¸ö?
        float minDT = 0.1f*dtEkfAvg;
        imuDataDelayed.delAngDT = MAX(imuDataDelayed.delAngDT,minDT);
        imuDataDelayed.delVelDT = MAX(imuDataDelayed.delVelDT,minDT);

        updateTimingStatistics();
            
        // correct the extracted IMU data for sensor errors
        //ĞŞÕıÌáÈ¡µÄIMUÊı¾İ  ³ËÒÔ±ê¶ÈÒòÊıÔÙ¼õÈ¥ÁãÆ«
        delAngCorrected = imuDataDelayed.delAng;
        delVelCorrected = imuDataDelayed.delVel;
        correctDeltaAngle(delAngCorrected, imuDataDelayed.delAngDT);
        correctDeltaVelocity(delVelCorrected, imuDataDelayed.delVelDT);

    } else {
        // we don't have new IMU data in the buffer so don't run filter updates on this time step
        //ÈôÎ´µ½EKFµÄÊ±¿Ì£¬ÔòEKFÂË²¨±êÖ¾Î»ÎªÁã£¬²»½øÈëEKFÑ­»·¡£
        runUpdates = false;
    }
}

// read the delta velocity and corresponding time interval from the IMU
// return false if data is not available
//Èç¹ûÊı¾İ²»¿ÉÓÃ£¬´ÓIMUÖĞ¶ÁÈ¡ËÙ¶ÈÔöÁ¿ºÍÏàÓ¦Ê±¼ä£¬²¢·µ»Øfalse
bool NavEKF2_core::readDeltaVelocity(uint8_t ins_index, Vector3f &dVel, float &dVel_dt) {
    const AP_InertialSensor &ins = AP::ins();//ÃüÃû¿Õ¼ä£¬AP::ins()·µ»ØÒ»¸öÖ¸ÏòAP_InertialSensorµÄÒıÓÃ£¬ÇÒÒıÓÃÊÇconstÀàĞÍ

    if (ins_index < ins.get_accel_count()) {
        ins.get_delta_velocity(ins_index,dVel);//get_accel(i) * get_delta_time()
        dVel_dt = MAX(ins.get_delta_velocity_dt(ins_index),1.0e-4f);//ÎªÊ²Ã´»á ÓĞÕâ¸öÅĞ¶Ï?
        dVel_dt = MIN(dVel_dt,1.0e-1f);
        return true;
    }
    return false;
}

/********************************************************
*             Global Position Measurement               *
********************************************************/

// check for new valid GPS data and update stored measurement if available
//¼ì²éĞÂµÄ¿ÉÓÃµÄGPSÊı¾İ£¬Èç¹ûGPS¿ÉÓÃ¸üĞÂ´æ´¢µÄÁ¿²âĞÅÏ¢
void NavEKF2_core::readGpsData()
{
    // check for new GPS data
    // do not accept data at a faster rate than 14Hz to avoid overflowing the FIFO buffer
    //½ÓÊÕGPSÊı¾İËÙÂÊĞ¡ÓÚµÈÓÚ14Hz£¬±ÜÃâFIFO»º³åÇøÒç³ö
    const AP_GPS &gps = AP::gps();//AP ÃüÃû¿Õ¼ä ¸Ãgpsº¯ÊıÎª:AP_GPS &gps(){return AP_GPS::gps();}
    if (gps.last_message_time_ms() - lastTimeGpsReceived_ms > 70) {
        if (gps.status() >= AP_GPS::GPS_OK_FIX_3D) {//gps.status() ÊÇÖ÷GPSµÄ×´Ì¬
            // report GPS fix status  GPS_OK_FIX_3DÊÇGPS_StatusÖĞµÚËÄ¸ö×´Ì¬±äÁ¿
            //gpsCheckStatusÊÇÒ»¸ö°üº¬ÖÚ¶àbool±êÖ¾Î»µÄ½á¹¹Ìå£¬±íÃ÷Î»ÖÃ¡¢ËÙ¶ÈµÈÊÇ·ñÕıÈ·
            gpsCheckStatus.bad_fix = false;

            // store fix time from previous read
            secondLastGpsTime_ms = lastTimeGpsReceived_ms;//ÉÏ´ÎÊÕµ½GPSÊı¾İµÄÊ±¼ä

            // get current fix time 
            lastTimeGpsReceived_ms = gps.last_message_time_ms();


            // estimate when the GPS fix was valid, allowing for GPS processing and other delays
            // ideally we should be using a timing signal from the GPS receiver to set this time
            //¹À¼ÆÊ²Ã´Ê±ºòGPS¶¨Î»ÊÇÓĞĞ§µÄ£¬¿¼ÂÇµ½GPS´¦ÀíºÍÆäËûÑÓ³Ù£¬ÀíÏëÇé¿öÏÂÎÒÃÇÓ¦¸ÃÓÃGPS½ÓÊÕ»ú
            //µÄÊ±¼äÀ´ÉèÖÃ¸ÃÊ±¼ä
            float gps_delay = 0.0;
            gps.get_lag(gps_delay); // ignore the return value
            gpsDataNew.time_ms = lastTimeGpsReceived_ms - (uint32_t)(1e3f * gps_delay);

            // Correct for the average intersampling delay due to the filter updaterate
            //ĞŞÕıÓÉÓÚÂË²¨Æ÷¸üĞÂÆµÂÊµ¼ÖÂµÄÆ½¾ù²ÉÑùÊ±¼äÑÓ³Ù
            //localFilterTimeStep Á½´ÎÂË²¨Æ÷¸üĞÂÖ®¼äµÄÆ½¾ùÊ±¼ä µ¥Î»:ms
            //gpsDataNew µ±Ç°Ê±¿ÌµÄgpsÊı¾İ  
            //½á¹¹Ìå struct gps_elements{vector2f pos;float hgt;vector3f vel;uint32_t time_ms;uint8_t sensor_idx;}
            gpsDataNew.time_ms -= localFilterTimeStep_ms/2;

            // Prevent time delay exceeding age of oldest IMU data in the buffer
            //·ÀÖ¹Ê±¼äÑÓ³Ù³¬¹ıbufferÖĞ×îÔçimuµÄ´æ·ÅÊ±¼ä
            gpsDataNew.time_ms = MAX(gpsDataNew.time_ms,imuDataDelayed.time_ms);

            // Get which GPS we are using for position information
            //µÃµ½ÎÒÃÇÊ¹ÓÃµÄGPSµÄÎ»ÖÃĞÅÏ¢  ·µ»ØÖ÷´«¸ĞÆ÷µÄË÷Òı
            gpsDataNew.sensor_idx = gps.primary_sensor();

            // read the NED velocity from the GPS
            //µÃµ½GPSµÄËÙ¶ÈĞÅÏ¢
            gpsDataNew.vel = gps.velocity();

            // Use the speed and position accuracy from the GPS if available, otherwise set it to zero.
            // Apply a decaying envelope filter with a 5 second time constant to the raw accuracy data
            //Èç¹ûGPSËÙ¶È¡¢Î»ÖÃ¾«¶È¿ÉÓÃ£¬ÔòÓÃÖ®£»·ñÔò£¬½«ÆäÖÃÎªÁã£»GPSµÄÎ»ÖÃ¡¢ËÙ¶ÈºÍ´¹Ö±·½ÏòÎ»ÖÃ¾«¶È
            //ÔÚÁ¬ĞøµÄ5sÄÚ½«Ë¥¼õ°üÂçÂË²¨Æ÷Ó¦ÓÃÓÚÔ­Ê¼¾«¶ÈµÄÊı¾İ
            //gpsSpdAccuracy GPS½ÓÊÕ»ú·µ»ØµÄËÙ¶È¾«¶È  µ¥Î»:m/s
            float alpha = constrain_float(0.0002f * (lastTimeGpsReceived_ms - secondLastGpsTime_ms),0.0f,1.0f);
            gpsSpdAccuracy *= (1.0f - alpha);
            float gpsSpdAccRaw;
            if (!gps.speed_accuracy(gpsSpdAccRaw)) {//speed_accuracyÊÇGPSÌá¹©µÄËÙ¶È¾«¶È
                gpsSpdAccuracy = 0.0f;
            } else {
                gpsSpdAccuracy = MAX(gpsSpdAccuracy,gpsSpdAccRaw);
                gpsSpdAccuracy = MIN(gpsSpdAccuracy,50.0f);
            }
			//gpsPosAccuracy GPS½ÓÊÕ»ú·µ»ØµÄÎ»ÖÃ¾«¶È µ¥Î»:m
            gpsPosAccuracy *= (1.0f - alpha);
            float gpsPosAccRaw;
            if (!gps.horizontal_accuracy(gpsPosAccRaw)) {
                gpsPosAccuracy = 0.0f;
            } else {
                gpsPosAccuracy = MAX(gpsPosAccuracy,gpsPosAccRaw);
                gpsPosAccuracy = MIN(gpsPosAccuracy,100.0f);
            }
			//gps½ÓÊÕ»ú·µ»ØµÄ¸ß¶È¾«¶È µ¥Î»:m
            gpsHgtAccuracy *= (1.0f - alpha);
            float gpsHgtAccRaw;
            if (!gps.vertical_accuracy(gpsHgtAccRaw)) {
                gpsHgtAccuracy = 0.0f;
            } else {
                gpsHgtAccuracy = MAX(gpsHgtAccuracy,gpsHgtAccRaw);
                gpsHgtAccuracy = MIN(gpsHgtAccuracy,100.0f);
            }

            // check if we have enough GPS satellites and increase the gps noise scaler if we don't
            //¼ì²éÊÇ·ñÓĞ×ã¹»µÄGPSÎÀĞÇ£¬Èç¹û²»¼ì²éÔòÔö¼ÓGPSµÄÔëÉùË®Æ½
            //¸ù¾İÎÀĞÇÊıÄ¿ÌôÕ½GPSÎ»ÖÃ¡¢ËÙ¶ÈÔëÉùË®Æ½
            //PV_AidingModeÊÇÒ»¸öÃ¶¾Ù±äÁ¿£¬ÓĞÈı¸öÖµ£¬AID_ABSOLUTE AID_NONE AID_RELATIVE£¬·Ö±ğÎª0 1 2 
            if (gps.num_sats() >= 6 && (PV_AidingMode == AID_ABSOLUTE)) {//num_sats() ÓÉGPSÖ±½ÓÌá¹©µÄ¿ÉÊÓµÄÎÀĞÇÊı
                gpsNoiseScaler = 1.0f;
            } else if (gps.num_sats() == 5 && (PV_AidingMode == AID_ABSOLUTE)) {
            //gpsNoiseScaler  ÔÚGPSÎÀĞÇ½ÏÉÙÊ±ÓÃÓÚËõ·ÅGPSÁ¿²âÔëÉù
                gpsNoiseScaler = 1.4f;
            } else { // <= 4 satellites or in constant position mode
                gpsNoiseScaler = 2.0f;
            }

            // Check if GPS can output vertical velocity, if it is allowed to be used, and set GPS fusion mode accordingly
            //¼ì²éGPSÊÇ·ñ¿ÉÊä³ö´¹Ö±ËÙ¶È£¬Èç¹ûÔÊĞíÊ¹ÓÃ´¹Ö±ËÙ¶È£¬ÏàÓ¦µØÉèÖÃGPSÈÚºÏÄ£Ê½
            //gps.have_vertical_velocity²é¿´GPSÊÇ·ñÌá¹©´¹Ö±ËÙ¶È£¬ÈçÓĞ£¬ÖÃ1
            //useGpsVertvel ÈôGPS´¹Ö±ËÙ¶È±»ÓÃµ½ÔòÖÃtrue
            if (gps.have_vertical_velocity() && frontend->_fusionModeGPS == 0 && !frontend->inhibitGpsVertVelUse) {
                useGpsVertVel = true;
            } else {
                useGpsVertVel = false;
            }

            // Monitor quality of the GPS velocity data before and after alignment using separate checks
            //µ¥¶À¼ì²éGPSĞ£×¼Ç°ºóµÄËÙ¶ÈÖÊÁ¿
            //PV_AidingMode ¶¨ÒåÀ´×Ô¹ßµ¼µÄÎ»ÖÃ¡¢ËÙ¶ÈÈÚºÏµÄÓÅÑ¡Ä£Ê½
            if (PV_AidingMode != AID_ABSOLUTE) {//AID_ABSOLUTE ÓĞ¾ø¶ÔµÄÎ»ÖÃËÙ¶È²Î¿¼
                // Pre-alignment checks
                //µ±GPSµÄÖÊÁ¿¿É±»ÓÃÀ´³õÊ¼»¯µ¼º½ÏµÍ³Ê±£¬ÖÃtrue
                gpsGoodToAlign = calcGpsGoodToAlign();
            } else {
                gpsGoodToAlign = false;
            }

            // Post-alignment checks
            //¸üĞÂ·ÉĞĞ¼ÆËã£¬È·¶¨GPSÊÇ·ñ¿ÉÓÃÀ´µ¼º½
            calcGpsGoodForFlight();

            // Read the GPS location in WGS-84 lat,long,height coordinates
            //¶ÁWGS-84×ø±êÏµÏÂµÄÎ³¶È¡¢¾­¶È¡¢¸ß¶È
            //struct Location{ union {Location_option_Flag flags;uint8_t options;};int32_t alt:24;int32_t lat;int32_t lng}
            //¸ß¶È*100 Î³¶È¡¢¾­¶È*10^7
			const struct Location &gpsloc = gps.location();

            // Set the EKF origin and magnetic field declination if not previously set  and GPS checks have passed
            //Èç¹ûÏÈÇ°Î´ÉèÖÃÇÒGPS×Ô¼ìÍ¨¹ı£¬ÔòÉèÖÃEKFÔ­µãºÍ´ÅÆ«½Ç
            //ÈôGPSµÄÖÊÁ¿¿ÉÓÃÀ´³õÊ¼»¯µ¼º½ÏµÍ³£¬ÔògpsGoodToAlignÎªtrue
            //ÈôEKFÔ­µãÊÇ¿ÉÓÃµÄ£¬ÔòvalidOriginÎªtrue
            if (gpsGoodToAlign && !validOrigin) {
                setOrigin();

                // set the NE earth magnetic field states using the published declination
                // and set the corresponding variances and covariances
                //Ê¹ÓÃ·¢²¼µÄÆ«½ÇÉèÖÃ±±Ïò¡¢¶«Î÷µÄµØÀíÏµÏÂ´Å³¡
                //ÉèÖÃÏàÓ¦µÄ·½²îºÍĞ­·½²î¾ØÕó
                alignMagStateDeclination();

                // Set the height of the NED origin ÉèÖÃµ¼º½ÏµÆğµãµÄ¸ß¶ÈÊı¾İ refence height
                ekfGpsRefHgt = (double)0.01 * (double)gpsloc.alt + (double)outputDataNew.position.z;

                // Set the uncertainty of the GPS origin height  ÉèÖÃGPSÆğµã¸ß¶ÈµÄ·½²î
                ekfOriginHgtVar = sq(gpsHgtAccuracy);//gpsHgtAccuracyÊÇGPS½ÓÊÕ»ú·µ»ØµÄGPS¸ß¶ÈÃ×¼¶µÄ¾«¶È

            }

            // convert GPS measurements to local NED and save to buffer to be fused later if we have a valid origin
            //½«GPSÁ¿²âĞÅÏ¢×ª»»µ½NEDµ¼º½ÏµÏÂ£¬ÈôÓĞ³õÊ¼¿ÉÓÃµÄÆğµãÔò±£´æµ½bufÖĞÒÔ±ãÉÔºó½øĞĞÈÚºÏ
            if (validOrigin) {//validOriginÈôekf³õÊ¼µÄÆğµãÊÇ¿ÉÓÃµÄ£¬ÔòvalidOrigin·µ»Øtrue
                gpsDataNew.pos = location_diff(EKF_origin, gpsloc);//µÃµ½Á½¸öµãÖ®¼äµÄ¶«Î÷¡¢±±ÏòÎ»ÖÃÎó²î
                gpsDataNew.hgt = (float)((double)0.01 * (double)gpsloc.alt - ekfGpsRefHgt);//¸ß¶ÈÎó²î
                storedGPS.push(gpsDataNew);
                // declare GPS available for use
                //gpsNotAvailable GPSÊı¾İ²»¿ÉÓÃÊ±ÖÃtrue
                gpsNotAvailable = false;
            }

            frontend->logging.log_gps = true;

        } else {
            // report GPS fix status
            gpsCheckStatus.bad_fix = true;
        }
    }
}

// read the delta angle and corresponding time interval from the IMU
// return false if data is not available
//¶Ádelta½ÇºÍÏàÓ¦µÄIMUÊ±¼ä¼ä¸ô£¬Èç¹ûÊı¾İ²»¿ÉÓÃ·µ»Øfalse 
//gyro * delta_time
//×î´ó×îĞ¡Öµº¯Êı£¬
bool NavEKF2_core::readDeltaAngle(uint8_t ins_index, Vector3f &dAng, float &dAng_dt) {
    const AP_InertialSensor &ins = AP::ins();

    if (ins_index < ins.get_gyro_count()) {
        ins.get_delta_angle(ins_index,dAng);//get_gyro(i) * get_delta_time();
        frontend->logging.log_imu = true;
        dAng_dt = MAX(ins.get_delta_angle_dt(imu_index),1.0e-4f);
        dAng_dt = MIN(dAng_dt,1.0e-1f);
        return true;
    }
    return false;
}


/********************************************************
*                  Height Measurements                  *
********************************************************/

// check for new pressure altitude measurement data and update stored measurement if available
void NavEKF2_core::readBaroData()
{
    // check to see if baro measurement has changed so we know if a new measurement has arrived
    // do not accept data at a faster rate than 14Hz to avoid overflowing the FIFO buffer
    const AP_Baro &baro = AP::baro();
    if (baro.get_last_update() - lastBaroReceived_ms > 70) {
        frontend->logging.log_baro = true;

        baroDataNew.hgt = baro.get_altitude();

        // If we are in takeoff mode, the height measurement is limited to be no less than the measurement at start of takeoff
        // This prevents negative baro disturbances due to copter downwash corrupting the EKF altitude during initial ascent
        if (getTakeoffExpected()) {
            baroDataNew.hgt = MAX(baroDataNew.hgt, meaHgtAtTakeOff);
        }

        // time stamp used to check for new measurement
        lastBaroReceived_ms = baro.get_last_update();

        // estimate of time height measurement was taken, allowing for delays
        baroDataNew.time_ms = lastBaroReceived_ms - frontend->_hgtDelay_ms;

        // Correct for the average intersampling delay due to the filter updaterate
        baroDataNew.time_ms -= localFilterTimeStep_ms/2;

        // Prevent time delay exceeding age of oldest IMU data in the buffer
        baroDataNew.time_ms = MAX(baroDataNew.time_ms,imuDataDelayed.time_ms);

        // save baro measurement to buffer to be fused later
        storedBaro.push(baroDataNew);
    }
}

// calculate filtered offset between baro height measurement and EKF height estimate
// offset should be subtracted from baro measurement to match filter estimate
// offset is used to enable reversion to baro from alternate height data source
void NavEKF2_core::calcFiltBaroOffset()
{
    // Apply a first order LPF with spike protection
    baroHgtOffset += 0.1f * constrain_float(baroDataDelayed.hgt + stateStruct.position.z - baroHgtOffset, -5.0f, 5.0f);
}

// correct the height of the EKF origin to be consistent with GPS Data using a Bayes filter.
void NavEKF2_core::correctEkfOriginHeight()
{
    // Estimate the WGS-84 height of the EKF's origin using a Bayes filter

    // calculate the variance of our a-priori estimate of the ekf origin height
    float deltaTime = constrain_float(0.001f * (imuDataDelayed.time_ms - lastOriginHgtTime_ms), 0.0f, 1.0f);
    if (activeHgtSource == HGT_SOURCE_BARO) {
        // Use the baro drift rate
        const float baroDriftRate = 0.05f;
        ekfOriginHgtVar += sq(baroDriftRate * deltaTime);
    } else if (activeHgtSource == HGT_SOURCE_RNG) {
        // use the worse case expected terrain gradient and vehicle horizontal speed
        const float maxTerrGrad = 0.25f;
        ekfOriginHgtVar += sq(maxTerrGrad * norm(stateStruct.velocity.x , stateStruct.velocity.y) * deltaTime);
    } else {
        // by definition our height source is absolute so cannot run this filter
        return;
    }
    lastOriginHgtTime_ms = imuDataDelayed.time_ms;

    // calculate the observation variance assuming EKF error relative to datum is independent of GPS observation error
    // when not using GPS as height source
    float originHgtObsVar = sq(gpsHgtAccuracy) + P[8][8];

    // calculate the correction gain
    float gain = ekfOriginHgtVar / (ekfOriginHgtVar + originHgtObsVar);

    // calculate the innovation
    float innovation = - stateStruct.position.z - gpsDataDelayed.hgt;

    // check the innovation variance ratio
    float ratio = sq(innovation) / (ekfOriginHgtVar + originHgtObsVar);

    // correct the EKF origin and variance estimate if the innovation is less than 5-sigma
    if (ratio < 25.0f && gpsAccuracyGood) {
        ekfGpsRefHgt -= (double)(gain * innovation);
        ekfOriginHgtVar -= MAX(gain * ekfOriginHgtVar , 0.0f);
    }
}

/********************************************************
*                Air Speed Measurements                 *
********************************************************/

// check for new airspeed data and update stored measurements if available
void NavEKF2_core::readAirSpdData()
{
    // if airspeed reading is valid and is set by the user to be used and has been updated then
    // we take a new reading, convert from EAS to TAS and set the flag letting other functions
    // know a new measurement is available
    const AP_Airspeed *aspeed = _ahrs->get_airspeed();
    if (aspeed &&
            aspeed->use() &&
            aspeed->last_update_ms() != timeTasReceived_ms) {
        tasDataNew.tas = aspeed->get_airspeed() * aspeed->get_EAS2TAS();
        timeTasReceived_ms = aspeed->last_update_ms();
        tasDataNew.time_ms = timeTasReceived_ms - frontend->tasDelay_ms;

        // Correct for the average intersampling delay due to the filter update rate
        tasDataNew.time_ms -= localFilterTimeStep_ms/2;

        // Save data into the buffer to be fused when the fusion time horizon catches up with it
        storedTAS.push(tasDataNew);
    }
    // Check the buffer for measurements that have been overtaken by the fusion time horizon and need to be fused
    tasDataToFuse = storedTAS.recall(tasDataDelayed,imuDataDelayed.time_ms);
}

/********************************************************
*              Range Beacon Measurements                *
********************************************************/

// check for new range beacon data and push to data buffer if available
void NavEKF2_core::readRngBcnData()
{
    // get the location of the beacon data
    const AP_Beacon *beacon = _ahrs->get_beacon();

    // exit immediately if no beacon object
    if (beacon == nullptr) {
        return;
    }

    // get the number of beacons in use
    N_beacons = beacon->count();

    // search through all the beacons for new data and if we find it stop searching and push the data into the observation buffer
    bool newDataToPush = false;
    uint8_t numRngBcnsChecked = 0;
    // start the search one index up from where we left it last time
    uint8_t index = lastRngBcnChecked;
    while (!newDataToPush && numRngBcnsChecked < N_beacons) {
        // track the number of beacons checked
        numRngBcnsChecked++;

        // move to next beacon, wrap index if necessary
        index++;
        if (index >= N_beacons) {
            index = 0;
        }

        // check that the beacon is healthy and has new data
        if (beacon->beacon_healthy(index) &&
                beacon->beacon_last_update_ms(index) != lastTimeRngBcn_ms[index])
        {
            // set the timestamp, correcting for measurement delay and average intersampling delay due to the filter update rate
            lastTimeRngBcn_ms[index] = beacon->beacon_last_update_ms(index);
            rngBcnDataNew.time_ms = lastTimeRngBcn_ms[index] - frontend->_rngBcnDelay_ms - localFilterTimeStep_ms/2;

            // set the range noise
            // TODO the range library should provide the noise/accuracy estimate for each beacon
            rngBcnDataNew.rngErr = frontend->_rngBcnNoise;

            // set the range measurement
            rngBcnDataNew.rng = beacon->beacon_distance(index);

            // set the beacon position
            rngBcnDataNew.beacon_posNED = beacon->beacon_position(index);

            // identify the beacon identifier
            rngBcnDataNew.beacon_ID = index;

            // indicate we have new data to push to the buffer
            newDataToPush = true;

            // update the last checked index
            lastRngBcnChecked = index;
        }
    }

    // Check if the beacon system has returned a 3D fix
    if (beacon->get_vehicle_position_ned(beaconVehiclePosNED, beaconVehiclePosErr)) {
        rngBcnLast3DmeasTime_ms = imuSampleTime_ms;
    }

    // Check if the range beacon data can be used to align the vehicle position
    if (imuSampleTime_ms - rngBcnLast3DmeasTime_ms < 250 && beaconVehiclePosErr < 1.0f && rngBcnAlignmentCompleted) {
        // check for consistency between the position reported by the beacon and the position from the 3-State alignment filter
        float posDiffSq = sq(receiverPos.x - beaconVehiclePosNED.x) + sq(receiverPos.y - beaconVehiclePosNED.y);
        float posDiffVar = sq(beaconVehiclePosErr) + receiverPosCov[0][0] + receiverPosCov[1][1];
        if (posDiffSq < 9.0f*posDiffVar) {
            rngBcnGoodToAlign = true;
            // Set the EKF origin and magnetic field declination if not previously set
            if (!validOrigin && PV_AidingMode != AID_ABSOLUTE) {
                // get origin from beacon system
                Location origin_loc;
                if (beacon->get_origin(origin_loc)) {
                    setOriginLLH(origin_loc);

                    // set the NE earth magnetic field states using the published declination
                    // and set the corresponding variances and covariances
                    alignMagStateDeclination();

                    // Set the uncertainty of the origin height
                    ekfOriginHgtVar = sq(beaconVehiclePosErr);
                }
            }
        } else {
            rngBcnGoodToAlign = false;
        }
    } else {
        rngBcnGoodToAlign = false;
    }

    // Save data into the buffer to be fused when the fusion time horizon catches up with it
    if (newDataToPush) {
        storedRangeBeacon.push(rngBcnDataNew);
    }

    // Check the buffer for measurements that have been overtaken by the fusion time horizon and need to be fused
    rngBcnDataToFuse = storedRangeBeacon.recall(rngBcnDataDelayed,imuDataDelayed.time_ms);

}

/*
  update timing statistics structure
 */
void NavEKF2_core::updateTimingStatistics(void)
{
    if (timing.count == 0) {
        timing.dtIMUavg_max = dtIMUavg;
        timing.dtIMUavg_min = dtIMUavg;
        timing.dtEKFavg_max = dtEkfAvg;
        timing.dtEKFavg_min = dtEkfAvg;
        timing.delAngDT_max = imuDataDelayed.delAngDT;
        timing.delAngDT_min = imuDataDelayed.delAngDT;
        timing.delVelDT_max = imuDataDelayed.delVelDT;
        timing.delVelDT_min = imuDataDelayed.delVelDT;
    } else {
        timing.dtIMUavg_max = MAX(timing.dtIMUavg_max, dtIMUavg);
        timing.dtIMUavg_min = MIN(timing.dtIMUavg_min, dtIMUavg);
        timing.dtEKFavg_max = MAX(timing.dtEKFavg_max, dtEkfAvg);
        timing.dtEKFavg_min = MIN(timing.dtEKFavg_min, dtEkfAvg);
        timing.delAngDT_max = MAX(timing.delAngDT_max, imuDataDelayed.delAngDT);
        timing.delAngDT_min = MIN(timing.delAngDT_min, imuDataDelayed.delAngDT);
        timing.delVelDT_max = MAX(timing.delVelDT_max, imuDataDelayed.delVelDT);
        timing.delVelDT_min = MIN(timing.delVelDT_min, imuDataDelayed.delVelDT);
    }
    timing.count++;
}

// get timing statistics structure
void NavEKF2_core::getTimingStatistics(struct ekf_timing &_timing)
{
    _timing = timing;
    memset(&timing, 0, sizeof(timing));
}

void NavEKF2_core::writeExtNavData(const Vector3f &sensOffset, const Vector3f &pos, const Quaternion &quat, float posErr, float angErr, uint32_t timeStamp_ms, uint32_t resetTime_ms)
{
    // limit update rate to maximum allowed by sensor buffers and fusion process
    // don't try to write to buffer until the filter has been initialised
    if ((timeStamp_ms - extNavMeasTime_ms) < 70) {
        return;
    } else {
        extNavMeasTime_ms = timeStamp_ms;
    }

    if (resetTime_ms > extNavLastPosResetTime_ms) {
        extNavDataNew.posReset = true;
        extNavLastPosResetTime_ms = resetTime_ms;
    } else {
        extNavDataNew.posReset = false;
    }

    extNavDataNew.pos = pos;
    extNavDataNew.quat = quat;
    extNavDataNew.posErr = posErr;
    extNavDataNew.angErr = angErr;
    extNavDataNew.body_offset = &sensOffset;
    extNavDataNew.time_ms = timeStamp_ms;

    storedExtNav.push(extNavDataNew);

}

#endif // HAL_CPU_CLASS
