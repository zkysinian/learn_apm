#include <AP_HAL/AP_HAL.h>

#if HAL_CPU_CLASS >= HAL_CPU_CLASS_150

#include "AP_NavEKF2.h"
#include "AP_NavEKF2_core.h"
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_RangeFinder/RangeFinder_Backend.h>

#include <stdio.h>

extern const AP_HAL::HAL& hal;

/********************************************************
*                   RESET FUNCTIONS                     *
********************************************************/

// Reset velocity states to last GPS measurement if available or to zero if in constant position mode or if PV aiding is not absolute
// Do not reset vertical velocity using GPS as there is baro alt available to constrain drift
//������ã���״̬���ٶ�����Ϊ��һʱ�̵�GPS���⣻������ں㶨λ��ģʽ��pv�������Ǿ��Եģ���������Ϊ��
//��ʹ��GPS���ô�ֱ�ٶȣ���Ϊ��ѹ�Ƶĸ߶��ǿ��õģ��ܹ���������Ư��
void NavEKF2_core::ResetVelocity(void)
{
    // Store the position before the reset so that we can record the reset delta
    //stateStruct�ṹ����28ά��״̬������velocity��Vector3���ͱ������ٶ�˳����3~5(ǰ��������̬���angErr)
    velResetNE.x = stateStruct.velocity.x;
    velResetNE.y = stateStruct.velocity.y;

    // reset the corresponding covariances
    //��Э�����������ˮƽ�ٶ���ص��С�������
    zeroRows(P,3,4);
    zeroCols(P,3,4);
    //PV_AidingMode �����ߵ������ٶȡ�λ���ںϵ�GPS��ѡģʽ
    //difines the preferred mode for aiding of velocity and position estimates from the INS
    if (PV_AidingMode != AID_ABSOLUTE) {//AID_ABSOLUTE ���Ե�λ�òο�������������Ǿ��Ե�λ�òο�������������
        stateStruct.velocity.zero();
        // set the variances using the measurement noise parameter
        P[4][4] = P[3][3] = sq(frontend->_gpsHorizVelNoise);//fronted ��ָ�����ָ�룬ͨ��->����
    } else {
        // reset horizontal velocity states to the GPS velocity if available
        //��״̬�е�ˮƽ�ٶ�����Ϊ��һʱ��GPS�ٶ�
        //imuSampleTime_ms�ϴ�imu���ݱ�ȡ�ߵ�ʱ�� lastTimeGpsReceived_ms �ϴ��յ�GPS���ݵ�ʱ��
        if (imuSampleTime_ms - lastTimeGpsReceived_ms < 250) {
            stateStruct.velocity.x  = gpsDataNew.vel.x;
            stateStruct.velocity.y  = gpsDataNew.vel.y;
            // set the variances using the reported GPS speed accuracy
            //��һ��GPS������������?
            //frontend ��ָ��NavEKF2�����ָ��
            P[4][4] = P[3][3] = sq(MAX(frontend->_gpsHorizVelNoise,gpsSpdAccuracy));//gpsSpdAccuracy �ǽ��ջ����ص��ٶȾ���
            // clear the timeout flags and counters
            //�����ʱ��־�ͼ�����
            //velTimeout ���ٶ���������Ϣһ���Լ��ʧ�ܻ�ʱ������true
            //lastVelPassTime_ms �ϴ�GPS�ٶ�����ͨ��һ���Լ���ʱ��
            velTimeout = false;
            lastVelPassTime_ms = imuSampleTime_ms;
        } else {
            stateStruct.velocity.x  = 0.0f;
            stateStruct.velocity.y  = 0.0f;
            // set the variances using the likely speed range 
            //���ÿ��ܵ��ٶȷ�Χ���÷���
            P[4][4] = P[3][3] = sq(25.0f);
            // clear the timeout flags and counters
            //�����ʱ��׼�ͼ�����
            velTimeout = false;
            lastVelPassTime_ms = imuSampleTime_ms;
        }
    }
	//���״̬�Ļ�����
    for (uint8_t i=0; i<imu_buffer_length; i++) {
        storedOutput[i].velocity.x = stateStruct.velocity.x;
        storedOutput[i].velocity.y = stateStruct.velocity.y;
    }
	//�����ǰʱ�̵�״̬����
	//outputDataNew outputDataDelayed ���������ǰʱ�̵�״̬���� output_element����
	//struct output_elements{Quaternion quat; Vector3f velocity; Vector3f position} 10������
    outputDataNew.velocity.x = stateStruct.velocity.x;
    outputDataNew.velocity.y = stateStruct.velocity.y;
    outputDataDelayed.velocity.x = stateStruct.velocity.x;
    outputDataDelayed.velocity.y = stateStruct.velocity.y;

    // Calculate the position jump due to the reset ��������������ɵ�λ������
    //struct state_elements{Vector3f angErr; Vector3f velocity; Vector3f position; Vector3f gyro_bias;
    //Vector3f gyro_scale; float accel_zbias; Vector3f earth_magfield; Vector3f body_magfield; 
    //Vector2f wind_vel; Quaternion quat;}
    velResetNE.x = stateStruct.velocity.x - velResetNE.x;
    velResetNE.y = stateStruct.velocity.y - velResetNE.y;

    // store the time of the reset
    //lastVelReset �ϴ��ٶ�����ʱ��ϵͳʱ��
    lastVelReset_ms = imuSampleTime_ms;


}

// resets position states to last GPS measurement or to zero if in constant position mode
//��״̬��λ������Ϊ��ʱ��GPS������λ�ã�������ں㶨λ��ģʽ��������
void NavEKF2_core::ResetPosition(void)
{
    // Store the position before the reset so that we can record the reset delta
    //posResetNE ���ڷ����������ڶ��򡢱����ϸı��λ�ã���λm
    posResetNE.x = stateStruct.position.x;
    posResetNE.y = stateStruct.position.y;

    // reset the corresponding covariances
    //���ö�Ӧ��Э�������
    zeroRows(P,6,7);//����6������7����Ϊ��
    zeroCols(P,6,7);//����6������7����Ϊ��
    //AID_ABSOLUTE ��GPS���������Ե�λ�òο�
    if (PV_AidingMode != AID_ABSOLUTE) {
        // reset all position state history to the last known position
        //��λ������Ϊ��һʱ��λ��
        stateStruct.position.x = lastKnownPositionNE.x;
        stateStruct.position.y = lastKnownPositionNE.y;
        // set the variances using the position measurement noise parameter
        //��λ����������������ӦЭ����
        P[6][6] = P[7][7] = sq(frontend->_gpsHorizPosNoise);
    } else  {//������ھ��Ե�λ�òο�����ģʽ
        // Use GPS data as first preference if fresh data is available
        //��GPS���ݿ��ã�������Ϊ��Ҫѡ��
        //imuSampleTime_ms�ϴ�ȡIMU���ݵ�ʱ��
        //lastTimeGpsReceived �ϴ��յ�GPS���ݵ�ʱ��
        if (imuSampleTime_ms - lastTimeGpsReceived_ms < 250) {
            // record the ID of the GPS for the data we are using for the reset
            //��¼�˴������õ�GPS��ID  gpsDataNew��ǰʱ�̵�GPS���� last_gps_idx �ϴ��ںϻ�����ʱGPS���ջ���ID
            last_gps_idx = gpsDataNew.sensor_idx;
            // write to state vector and compensate for offset  between last GPS measurement and the EKF time horizon
            //д��״̬����������EKF��ʱ����ϴ�GPS����ʱ��֮���ƫ�� gpsDataNew ��ǰʱ�̵�GPS����
            //imuDataDelayed �ں�ʱ�̵�imu����
            stateStruct.position.x = gpsDataNew.pos.x  + 0.001f*gpsDataNew.vel.x*(float(imuDataDelayed.time_ms) - float(gpsDataNew.time_ms));
            stateStruct.position.y = gpsDataNew.pos.y  + 0.001f*gpsDataNew.vel.y*(float(imuDataDelayed.time_ms) - float(gpsDataNew.time_ms));
            // set the variances using the position measurement noise parameter
            P[6][6] = P[7][7] = sq(MAX(gpsPosAccuracy,frontend->_gpsHorizPosNoise));
            // clear the timeout flags and counters
            posTimeout = false;
            lastPosPassTime_ms = imuSampleTime_ms;//rngBcnLast3DmeasTime_ms �ϴ��ű�ϵͳ����3D��λϵͳ��ʱ�� ms
        } else if (imuSampleTime_ms - rngBcnLast3DmeasTime_ms < 250) {
            // use the range beacon data as a second preference �ڶ�������ѡ��
            //receiverPos ���ջ�NED��λ��
            stateStruct.position.x = receiverPos.x;
            stateStruct.position.y = receiverPos.y;
            // set the variances from the beacon alignment filter
            P[6][6] = receiverPosCov[0][0];
            P[7][7] = receiverPosCov[1][1];
            // clear the timeout flags and counters
            rngBcnTimeout = false;
            lastRngBcnPassTime_ms = imuSampleTime_ms;
        } else if (imuSampleTime_ms - extNavDataDelayed.time_ms < 250) {
            //extNavDataDelayed �ں�ʱ�̵�����������Ϣ
			// use the range beacon data as a second preference
            stateStruct.position.x = extNavDataDelayed.pos.x;
            stateStruct.position.y = extNavDataDelayed.pos.y;
            // set the variances from the beacon alignment filter
            P[7][7] = P[6][6] = sq(extNavDataDelayed.posErr);
        }
    }
    for (uint8_t i=0; i<imu_buffer_length; i++) {//uint_8t unsigned char   typedef unsigned char uint8_t
        storedOutput[i].position.x = stateStruct.position.x;
        storedOutput[i].position.y = stateStruct.position.y;
    }
    outputDataNew.position.x = stateStruct.position.x;
    outputDataNew.position.y = stateStruct.position.y;
    outputDataDelayed.position.x = stateStruct.position.x;
    outputDataDelayed.position.y = stateStruct.position.y;

    // Calculate the position jump due to the reset
    //��������������ɵ�λ������  posResetNE�����ϴ�������ɵ�λ�øı�
    posResetNE.x = stateStruct.position.x - posResetNE.x;
    posResetNE.y = stateStruct.position.y - posResetNE.y;

    // store the time of the reset
    lastPosReset_ms = imuSampleTime_ms;

}

// reset the vertical position state using the last height measurement
void NavEKF2_core::ResetHeight(void)
{
    // Store the position before the reset so that we can record the reset delta
    posResetD = stateStruct.position.z;

    // write to the state vector
    stateStruct.position.z = -hgtMea;
    outputDataNew.position.z = stateStruct.position.z;
    outputDataDelayed.position.z = stateStruct.position.z;

    // reset the terrain state height
    if (onGround) {
        // assume vehicle is sitting on the ground
        terrainState = stateStruct.position.z + rngOnGnd;
    } else {
        // can make no assumption other than vehicle is not below ground level
        terrainState = MAX(stateStruct.position.z + rngOnGnd , terrainState);
    }
    for (uint8_t i=0; i<imu_buffer_length; i++) {
        storedOutput[i].position.z = stateStruct.position.z;
    }

    // Calculate the position jump due to the reset
    posResetD = stateStruct.position.z - posResetD;

    // store the time of the reset
    lastPosResetD_ms = imuSampleTime_ms;

    // clear the timeout flags and counters
    hgtTimeout = false;
    lastHgtPassTime_ms = imuSampleTime_ms;

    // reset the corresponding covariances
    zeroRows(P,8,8);
    zeroCols(P,8,8);

    // set the variances to the measurement variance
    P[8][8] = posDownObsNoise;

    // Reset the vertical velocity state using GPS vertical velocity if we are airborne
    // Check that GPS vertical velocity data is available and can be used
    if (inFlight && !gpsNotAvailable && frontend->_fusionModeGPS == 0 && !frontend->inhibitGpsVertVelUse) {
        stateStruct.velocity.z =  gpsDataNew.vel.z;
    } else if (onGround) {
        stateStruct.velocity.z = 0.0f;
    }
    for (uint8_t i=0; i<imu_buffer_length; i++) {
        storedOutput[i].velocity.z = stateStruct.velocity.z;
    }
    outputDataNew.velocity.z = stateStruct.velocity.z;
    outputDataDelayed.velocity.z = stateStruct.velocity.z;

    // reset the corresponding covariances
    zeroRows(P,5,5);
    zeroCols(P,5,5);

    // set the variances to the measurement variance
    P[5][5] = sq(frontend->_gpsVertVelNoise);

}

// Zero the EKF height datum
// Return true if the height datum reset has been performed
bool NavEKF2_core::resetHeightDatum(void)
{
    if (activeHgtSource == HGT_SOURCE_RNG) {
        // by definition the height datum is at ground level so cannot perform the reset
        return false;
    }
    // record the old height estimate
    float oldHgt = -stateStruct.position.z;
    // reset the barometer so that it reads zero at the current height
    AP::baro().update_calibration();
    // reset the height state
    stateStruct.position.z = 0.0f;
    // adjust the height of the EKF origin so that the origin plus baro height before and after the reset is the same
    if (validOrigin) {
        ekfGpsRefHgt += (double)oldHgt;
    }
    // adjust the terrain state
    terrainState += oldHgt;
    return true;
}

/********************************************************
*                   FUSE MEASURED_DATA                  *
********************************************************/
// select fusion of velocity, position and height measurements
//ѡ���ں��ٶȡ�λ�ú͸߶���Ϣ
//λ���ٶȡ��߶��ںϵ�ϵͳģ�ͺ�����ģ��?24ά��״̬����ֻ�ں��ٶȸ߶ȡ���ν����ں�?
void NavEKF2_core::SelectVelPosFusion()
{
    // Check if the magnetometer has been fused on that time step and the filter is running at faster than 200 Hz
    // If so, don't fuse measurements on this time step to reduce frame over-runs
    // Only allow one time slip to prevent high rate magnetometer data preventing fusion of other measurements
    //���������ڴ�ʱ���Ƿ��Ѿ��ںϣ�����˲�Ƶ���Ƿ����200Hz
    //���ǣ����ʱ�̲��ںϴ����ƣ��Է�ֹ֡����
    //������һ����Ծ����ֹ���ٵĴ�������ֹ���������ں�
    //dtIMUavg ����IMU���ݸ���֮�������ʱ��  magFusePerformed �����������ںϵı�־λ 
    //posVelFusionDelayed ��λ�á��ٶ��ں��ӳ�ʱ����Ϊtrue
    if (magFusePerformed && dtIMUavg < 0.005f && !posVelFusionDelayed) {
        posVelFusionDelayed = true;
        return;
    } else {
        posVelFusionDelayed = false;
    }

    // Check for data at the fusion time horizon
    extNavDataToFuse = storedExtNav.recall(extNavDataDelayed, imuDataDelayed.time_ms);

    // read GPS data from the sensor and check for new data in the buffer
    //��GPS���ݲ����buf�е�������
    //Ϊʲô��GPS������ô���ж�????
    //buf����μ������
    //��GPS���ݲ����ں� ����gpsDataToFuseΪtrue
    readGpsData();
    gpsDataToFuse = storedGPS.recall(gpsDataDelayed,imuDataDelayed.time_ms);
    // Determine if we need to fuse position and velocity data on this time step
    //�����ڵ�ǰʱ���Ƿ���Ҫ�ں�λ�á��ٶ�
    //AID_ABSOLUTE ���Ե�λ�òο�
    //gpsDataToFuse �ںϵı�־ �����ñ�־u8 ��bool
    //fuseVelData NED���ٶ���������ں�����true
    //fusePosData NE��λ����������ں�����true
    if (gpsDataToFuse && PV_AidingMode == AID_ABSOLUTE) {
        // set fusion request flags
        if (frontend->_fusionModeGPS <= 1) {
            fuseVelData = true;
        } else {
            fuseVelData = false;
        }
        fusePosData = true;
        extNavUsedForPos = false;

        // correct GPS data for position offset of antenna phase centre relative to the IMU
        //У��IMU�����������λ���ĵ�λ��ƫ��
        //accelPosOffset IMU������ϵ�е�λ��
        Vector3f posOffsetBody = AP::gps().get_antenna_offset(gpsDataDelayed.sensor_idx) - accelPosOffset;
        if (!posOffsetBody.is_zero()) {//���λ��ƫ�Ʒ���
            // Don't fuse velocity data if GPS doesn't support it
            //���GPS���ٶȣ����ں��ٶ�����
            //fuseVelData �������ٶ��ںϵı�־λ
            if (fuseVelData) {
                // TODO use a filtered angular rate with a group delay that matches the GPS delay
                //ʹ���˲���Ľ����ʣ���Ϊ������GPS�ӳ�ƥ���ʱ���ӳ�
                //angRate ת�������� ��z������
                Vector3f angRate = imuDataDelayed.delAng * (1.0f/imuDataDelayed.delAngDT);
				//%������Ϊ��� �����ʺ�λ��ƫ�ƵĲ��Ϊʲô���ٶ�ƫ��
                Vector3f velOffsetBody = angRate % posOffsetBody;
                Vector3f velOffsetEarth = prevTnb.mul_transpose(velOffsetBody);
                gpsDataDelayed.vel.x -= velOffsetEarth.x;//����ƫ��֮���GPS�ٶ�����
                gpsDataDelayed.vel.y -= velOffsetEarth.y;
                gpsDataDelayed.vel.z -= velOffsetEarth.z;
            }

            //gpsDataDelayed �ں�ʱ�̵�GPS����
            //����λ��ƫ��֮���GPS����
			Vector3f posOffsetEarth = prevTnb.mul_transpose(posOffsetBody);
            gpsDataDelayed.pos.x -= posOffsetEarth.x;
            gpsDataDelayed.pos.y -= posOffsetEarth.y;
            gpsDataDelayed.hgt += posOffsetEarth.z;
        }

        // copy corrected GPS data to observation vector
        //���������GPS���ݸ��Ƹ��������
        //��NED���ٶ����ⱻ�ںϣ�����fuseVelDataΪtrue
        //velPosObs �ٶȺ�λ�õ�������Ϣ��ע��������ʽ
        if (fuseVelData) {
            velPosObs[0] = gpsDataDelayed.vel.x;
            velPosObs[1] = gpsDataDelayed.vel.y;
            velPosObs[2] = gpsDataDelayed.vel.z;
        }
        velPosObs[3] = gpsDataDelayed.pos.x;
        velPosObs[4] = gpsDataDelayed.pos.y;

    } else if (extNavDataToFuse && PV_AidingMode == AID_ABSOLUTE) {
        // This is a special case that uses and external nav system for position
        //���������������ⲿ����ϵͳ��λ��
        //extNavDataToFuse �����µ��ⲿ���������ںϣ�����true
        //PV_AidingMode == AID_ABSOLUTE  ���Ե�λ���ٶȲο�ģʽ
        extNavUsedForPos = true;
        activeHgtSource = HGT_SOURCE_EV;
        fuseVelData = false;
        fuseHgtData = true;
        fusePosData = true;
		//velPosObs gps��������Ϣ����λ���ٶȷ���һ��������
        velPosObs[3] = extNavDataDelayed.pos.x;
        velPosObs[4] = extNavDataDelayed.pos.y;
        velPosObs[5] = extNavDataDelayed.pos.z;

        // if compass is disabled, also use it for yaw
        if (!use_compass()) {
            extNavUsedForYaw = true;
            if (!yawAlignComplete) {//�������׼��ɣ���true
                extNavYawResetRequest = true;//��Ҫ�����ⲿ�����������ú�������extNavYawResetRequestΪtrue
                magYawResetRequest = false;//�����庽��ʹų�״̬�������ôų��������ã�����magYawResetRequestΪtrue
                gpsYawResetRequest = false;//����GPS�������ú�������gpsYawResetRequestΪtrue
                controlMagYawReset();
                finalInflightYawInit = true;
            } else {
                fuseEulerYaw();
            }
        } else {//extNavUsedForYaw ������ĵ���������������������ʱ��true
            extNavUsedForYaw = false;
        }

    } else {
        //���ں�NED���ٶ����⣬���ں�NE��λ������
		fuseVelData = false;
        fusePosData = false;
    }

    // we have GPS data to fuse and a request to align the yaw using the GPS course
    //ʹ��GPS���������뺽��
    //ʹ��GPS�������ú����ԭ��???��ͼƥ��???
    if (gpsYawResetRequest) {//�յ���GPS�����������庽������� gpsYawResetRequestΪtrue
        realignYawGPS();
    }

    // Select height data to be fused from the available baro, range finder and GPS sources

    selectHeightForFusion();

    // if we are using GPS, check for a change in receiver and reset position and height
    //��ʹ��GPS������GPS���ջ��ĸ��Ĳ�����λ�ú͸߶�
    if (gpsDataToFuse && PV_AidingMode == AID_ABSOLUTE && gpsDataDelayed.sensor_idx != last_gps_idx) {
        // record the ID of the GPS that we are using for the reset ��¼GPS��ID
        last_gps_idx = gpsDataDelayed.sensor_idx;

        // Store the position before the reset so that we can record the reset delta
        //�洢����֮ǰ��λ��
        posResetNE.x = stateStruct.position.x;
        posResetNE.y = stateStruct.position.y;

        // Set the position states to the position from the new GPS
        //�������µ�GPS����λ��״̬
        stateStruct.position.x = gpsDataNew.pos.x;
        stateStruct.position.y = gpsDataNew.pos.y;

        // Calculate the position offset due to the reset �����������ô�����λ��ƫ��
        posResetNE.x = stateStruct.position.x - posResetNE.x;
        posResetNE.y = stateStruct.position.y - posResetNE.y;

        // Add the offset to the output observer states
        for (uint8_t i=0; i<imu_buffer_length; i++) {
            storedOutput[i].position.x += posResetNE.x;
            storedOutput[i].position.y += posResetNE.y;
        }
        outputDataNew.position.x += posResetNE.x;
        outputDataNew.position.y += posResetNE.y;
        outputDataDelayed.position.x += posResetNE.x;
        outputDataDelayed.position.y += posResetNE.y;

        // store the time of the reset
        lastPosReset_ms = imuSampleTime_ms;

        // If we are alseo using GPS as the height reference, reset the height
        //����GPS���ڸ߶Ȳο��������ø߶�
        if (activeHgtSource == HGT_SOURCE_GPS) {
            // Store the position before the reset so that we can record the reset delta
            posResetD = stateStruct.position.z;

            // write to the state vector
            stateStruct.position.z = -hgtMea;

            // Calculate the position jump due to the reset
            //�����������ô�����λ������
            posResetD = stateStruct.position.z - posResetD;

            // Add the offset to the output observer states
            outputDataNew.position.z += posResetD;
            outputDataDelayed.position.z += posResetD;
            for (uint8_t i=0; i<imu_buffer_length; i++) {
                storedOutput[i].position.z += posResetD;
            }

            // store the time of the reset
            lastPosResetD_ms = imuSampleTime_ms;
        }
    }

    // If we are operating without any aiding, fuse in the last known position
    // to constrain tilt drift. This assumes a non-manoeuvring vehicle
    // Do this to coincide with the height fusion
    if (fuseHgtData && PV_AidingMode == AID_NONE) {
        velPosObs[3] = lastKnownPositionNE.x;
        velPosObs[4] = lastKnownPositionNE.y;
        fusePosData = true;
        fuseVelData = false;
    }

    // perform fusion
    if (fuseVelData || fusePosData || fuseHgtData) {
        FuseVelPosNED();
        // clear the flags to prevent repeated fusion of the same data
        fuseVelData = false;
        fuseHgtData = false;
        fusePosData = false;
    }
}

// fuse selected position, velocity and height measurements
//�ں�λ�á��ٶȺ͸߶�����
void NavEKF2_core::FuseVelPosNED()
{
    // start performance timer
    hal.util->perf_begin(_perf_FuseVelPosNED);

    // health is set bad until test passed 
    //Ϊʲô������˶��־λ???
    velHealth = false;
    posHealth = false;
    hgtHealth = false;

    // declare variables used to check measurement errors
    //������������������������
    Vector3f velInnov;

    // declare variables used to control access to arrays
    //����������������������ӿ�
    bool fuseData[6] = {false,false,false,false,false,false};
    uint8_t stateIndex;
    uint8_t obsIndex;

    // declare variables used by state and covariance update calculations
    //��������������״̬��Э������¼���
    Vector6 R_OBS; // Measurement variances used for fusion �ں��õ����ⷽ��
    Vector6 R_OBS_DATA_CHECKS; // Measurement variances used for data checks only
    float SK;

    // perform sequential fusion of GPS measurements. This assumes that the
    // errors in the different velocity and position components are
    // uncorrelated which is not true, however in the absence of covariance
    // data from the GPS receiver it is the only assumption we can make
    // so we might as well take advantage of the computational efficiencies
    // associated with sequential fusion
    //ִ��GPS�����˳���ںϡ��ü���(��ͬ�ٶȺ�λ�÷����е�����ǲ���ص�)�Ǵ���ġ�
    //��������ȱ��GPS���ջ���Э�������ݣ���������������Ψһ�ļ��衣
    //���Բ���������˳���ں���صļ���Ч��
    if (fuseVelData || fusePosData || fuseHgtData) {//���������ٶȡ�λ�á��߶��ںϵı�־

        // calculate additional error in GPS position caused by manoeuvring
        //�������ڻ��������GPS���ӵ�λ�����  gpsPosVarAccScale = 0.05f   NAVEKF2 *frontend
        //gpsPosVarAccScale �����ڻ���������ɵ�ˮƽλ�õı���������
        //accNavMag �����ļ��ٶȷ��ȣ����ڵ���GPS���ⷽ��
        float posErr = frontend->gpsPosVarAccScale * accNavMag;

        // estimate the GPS Velocity, GPS horiz position and height measurement variances.
        // Use different errors if operating without external aiding using an assumed position or velocity of zero
        //����GPS�ٶȡ�GPSˮƽλ�ú͸߶ȵ����ⷽ��
        //������ⲿ��������£��ٶ�λ�ú��ٶ�Ϊ�㣬��ʹ�ò�ͬ�����
        if (PV_AidingMode == AID_NONE) {
            if (tiltAlignComplete && motorsArmed) {//tiltAlignComplete �����б��׼����򷵻�true
            // This is a compromise between corrections for gyro errors and reducing effect of manoeuvre accelerations on tilt estimate
            //������������������ͽ��ͻ������ٶ���б����Ӱ�������
                R_OBS[0] = sq(constrain_float(frontend->_noaidHorizNoise, 0.5f, 50.0f));//_noaidHorizNoise ˮƽλ����������
            } else {
                // Use a smaller value to give faster initial alignment
                //ʹ�ý�С��ֵ�Ծ����ʼ��׼
                R_OBS[0] = sq(0.5f);
            }
            R_OBS[1] = R_OBS[0];//R_OBS ���ⷽ��observation
            R_OBS[2] = R_OBS[0];
            R_OBS[3] = R_OBS[0];
            R_OBS[4] = R_OBS[0];
            for (uint8_t i=0; i<=2; i++) R_OBS_DATA_CHECKS[i] = R_OBS[i];
        } else {
            if (gpsSpdAccuracy > 0.0f) {//gpsSpdAccuracy GPS���ջ����ص��ٶȾ���
                // use GPS receivers reported speed accuracy if available and floor at value set by GPS velocity noise parameter
                //���������ʹ��GPS���ջ�������ٶȾ���
                R_OBS[0] = sq(constrain_float(gpsSpdAccuracy, frontend->_gpsHorizVelNoise, 50.0f));
                R_OBS[2] = sq(constrain_float(gpsSpdAccuracy, frontend->_gpsVertVelNoise, 50.0f));
            } else {
                // calculate additional error in GPS velocity caused by manoeuvring
                //�����ɻ�����ɵ�GPS�ٶȵĶ������
                R_OBS[0] = sq(constrain_float(frontend->_gpsHorizVelNoise, 0.05f, 5.0f)) + sq(frontend->gpsNEVelVarAccScale * accNavMag);
                R_OBS[2] = sq(constrain_float(frontend->_gpsVertVelNoise,  0.05f, 5.0f)) + sq(frontend->gpsDVelVarAccScale  * accNavMag);
            }
            R_OBS[1] = R_OBS[0];
            // Use GPS reported position accuracy if available and floor at value set by GPS position noise parameter
            //��������ʹ��GPS�����λ�þ���
            if (gpsPosAccuracy > 0.0f) {//gpsPosAccuracy ��GPS���ջ����ص�λ�ù��ƾ���
                R_OBS[3] = sq(constrain_float(gpsPosAccuracy, frontend->_gpsHorizPosNoise, 100.0f));
            } else {
                R_OBS[3] = sq(constrain_float(frontend->_gpsHorizPosNoise, 0.1f, 10.0f)) + sq(posErr);
            }
            R_OBS[4] = R_OBS[3];
            // For data integrity checks we use the same measurement variances as used to calculate the Kalman gains for all measurements except GPS horizontal velocity
            // For horizontal GPs velocity we don't want the acceptance radius to increase with reported GPS accuracy so we use a value based on best GPs perfomrance
            // plus a margin for manoeuvres. It is better to reject GPS horizontal velocity errors early
            //�����������Լ�飬ʹ����ͬ�����ⷽ���������GPSˮƽ�ٶ�������������˲�����
            //����ˮƽGPS�ٶȣ����ǲ�����հ뾶��GPS���ջ�����ľ������Ӷ����ӣ��ʲ�����õ�GPS���ջ���ֵ���ټ��ϻ���������ֵ
            //��GPSˮƽ�ٶȴ�������þ���ܾ�ʹ��ˮƽGPS�ٶ�
            for (uint8_t i=0; i<=2; i++) R_OBS_DATA_CHECKS[i] = sq(constrain_float(frontend->_gpsHorizVelNoise, 0.05f, 5.0f)) + sq(frontend->gpsNEVelVarAccScale * accNavMag);
        }
        R_OBS[5] = posDownObsNoise;//posDownObsNoise ״̬��Э���������ʹ�õĴ�ֱλ�õ���������
        for (uint8_t i=3; i<=5; i++) R_OBS_DATA_CHECKS[i] = R_OBS[i];

        // if vertical GPS velocity data and an independent height source is being used, check to see if the GPS vertical velocity and altimeter
        // innovations have the same sign and are outside limits. If so, then it is likely aliasing is affecting
        // the accelerometers and we should disable the GPS and barometer innovation consistency checks.
        //����ʹ��GPS��ֱ�ٶȺͶ����ĸ߶�Դ����ô���GPS��ֱ�ٶȺ͸߶ȸ����Ƿ������ͬ�ķ��ź��Ƿ񳬳�����
        //���ǣ��п���Ӱ��Ӽƣ���ʱӦ����GPS�͸߶ȼƸ��³����Լ��
        if (useGpsVertVel && fuseVelData && (frontend->_altSource != 2)) {
            // calculate innovations for height and vertical GPS vel measurements
            //����߶Ⱥ�GPS��ֱ�ٶȵ�������
            float hgtErr  = stateStruct.position.z - velPosObs[5];
            float velDErr = stateStruct.velocity.z - velPosObs[2];
            // check if they are the same sign and both more than 3-sigma out of bounds
            //��������Ƿ������ͬ�ķ��ţ��Ƿ񶼳���3-sigma�߽�
            if ((hgtErr*velDErr > 0.0f) && (sq(hgtErr) > 9.0f * (P[8][8] + R_OBS_DATA_CHECKS[5])) && (sq(velDErr) > 9.0f * (P[5][5] + R_OBS_DATA_CHECKS[2]))) {
                badIMUdata = true;
            } else {
                badIMUdata = false;
            }
        }
		//what fucking codes

        // calculate innovations and check GPS data validity using an innovation consistency check
        // test position measurements
        //��������������ʹ���������������Լ�������GPS���ݵĿ�����
        if (fusePosData) {
            // test horizontal position measurements �ں�ˮƽ�ٶ�����
            innovVelPos[3] = stateStruct.position.x - velPosObs[3];
            innovVelPos[4] = stateStruct.position.y - velPosObs[4];
            varInnovVelPos[3] = P[6][6] + R_OBS_DATA_CHECKS[3];
            varInnovVelPos[4] = P[7][7] + R_OBS_DATA_CHECKS[4];
            // apply an innovation consistency threshold test, but don't fail if bad IMU data
            float maxPosInnov2 = sq(MAX(0.01f * (float)frontend->_gpsPosInnovGate, 1.0f))*(varInnovVelPos[3] + varInnovVelPos[4]);
            posTestRatio = (sq(innovVelPos[3]) + sq(innovVelPos[4])) / maxPosInnov2;
            posHealth = ((posTestRatio < 1.0f) || badIMUdata);
            // use position data if healthy or timed out
            if (PV_AidingMode == AID_NONE) {
                posHealth = true;
                lastPosPassTime_ms = imuSampleTime_ms;
            } else if (posHealth || posTimeout) {
                posHealth = true;
                lastPosPassTime_ms = imuSampleTime_ms;
                // if timed out or outside the specified uncertainty radius, reset to the GPS
                //����ʱ�򳬹�ָ����ȷ���Եİ뾶��������GPS
                if (posTimeout || ((P[6][6] + P[7][7]) > sq(float(frontend->_gpsGlitchRadiusMax)))) {
                    // reset the position to the current GPS position
                    //��λ������Ϊ��ǰ��GPSλ��
                    ResetPosition();
                    // reset the velocity to the GPS velocity
                    //���ٶ�����ΪGPS���ٶ�
                    ResetVelocity();
                    // don't fuse GPS data on this time step
                    //��ʱ�̣����ں�GPS����
                    fusePosData = false;
                    fuseVelData = false;
                    // Reset the position variances and corresponding covariances to a value that will pass the checks
                    //����λ�÷����Ӧ��Э����
                    zeroRows(P,6,7);
                    zeroCols(P,6,7);
					//_gpsGlitchRadiusMax ��GPS�޹���ʱ ins��GPSˮƽλ�������������
                    P[6][6] = sq(float(0.5f*frontend->_gpsGlitchRadiusMax));
                    P[7][7] = P[6][6];
                    // Reset the normalised innovation to avoid failing the bad fusion tests
                    //���ù�һ��������������Ա������벡̬���ںϲ�����
                    posTestRatio = 0.0f;
                    velTestRatio = 0.0f;
                }
            } else {
                posHealth = false;
            }
        }

        // test velocity measurements
        if (fuseVelData) {
            // test velocity measurements
            uint8_t imax = 2;
            // Don't fuse vertical velocity observations if inhibited by the user or if we are using synthetic data
            //����û���ֹ����ʹ���ۺϵ����ݣ����ںϴ�ֱ�ٶ�����
            if (frontend->_fusionModeGPS > 0 || PV_AidingMode != AID_ABSOLUTE || frontend->inhibitGpsVertVelUse) {
                imax = 1;
            }
            float innovVelSumSq = 0; // sum of squares of velocity innovations
            float varVelSum = 0; // sum of velocity innovation variances
            for (uint8_t i = 0; i<=imax; i++) {
                // velocity states start at index 3 ״̬���ٶȴ��±�3��ʼ
                stateIndex   = i + 3;
                // calculate innovations using blended and single IMU predicted states
                //ʹ�û�Ϻ͵���IMUԤ��״̬
                velInnov[i]  = stateStruct.velocity[i] - velPosObs[i]; // blended
                // calculate innovation variance
                varInnovVelPos[i] = P[stateIndex][stateIndex] + R_OBS_DATA_CHECKS[i];
                // sum the innovation and innovation variances
                innovVelSumSq += sq(velInnov[i]);
                varVelSum += varInnovVelPos[i];
            }
            // apply an innovation consistency threshold test, but don't fail if bad IMU data
            // calculate the test ratio
            velTestRatio = innovVelSumSq / (varVelSum * sq(MAX(0.01f * (float)frontend->_gpsVelInnovGate, 1.0f)));
            // fail if the ratio is greater than 1
            velHealth = ((velTestRatio < 1.0f)  || badIMUdata);//����⵽IMU���ݴ������⣬����1
            // use velocity data if healthy, timed out, or in constant position mode
            if (velHealth || velTimeout) {
                velHealth = true;
                // restart the timeout count ������ʱ����
                lastVelPassTime_ms = imuSampleTime_ms;
                // If we are doing full aiding and velocity fusion times out, reset to the GPS velocity
                //����ڽ��о���λ�òο��������ٶ��ںϳ�ʱ��������GPS�ٶ�
                if (PV_AidingMode == AID_ABSOLUTE && velTimeout) {
                    // reset the velocity to the GPS velocity ���ٶ�����ΪGPS���ٶ�
                    ResetVelocity();
                    // don't fuse GPS velocity data on this time step ��ʱ�̲��ں�GPS�ٶ���Ϣ
                    fuseVelData = false;
                    // Reset the normalised innovation to avoid failing the bad fusion tests
                    //���ù�һ�������������������������ںϲ�����
                    velTestRatio = 0.0f;
                }
            } else {
                velHealth = false;
            }
        }

        // test height measurements
        if (fuseHgtData) {
            // calculate height innovations
            innovVelPos[5] = stateStruct.position.z - velPosObs[5];
            varInnovVelPos[5] = P[8][8] + R_OBS_DATA_CHECKS[5];
            // calculate the innovation consistency test ratio
            hgtTestRatio = sq(innovVelPos[5]) / (sq(MAX(0.01f * (float)frontend->_hgtInnovGate, 1.0f)) * varInnovVelPos[5]);
            // fail if the ratio is > 1, but don't fail if bad IMU data
            hgtHealth = ((hgtTestRatio < 1.0f) || badIMUdata);
            // Fuse height data if healthy or timed out or in constant position mode
            if (hgtHealth || hgtTimeout || (PV_AidingMode == AID_NONE && onGround)) {
                // Calculate a filtered value to be used by pre-flight health checks
                // We need to filter because wind gusts can generate significant baro noise and we want to be able to detect bias errors in the inertial solution
                if (onGround) {
                    float dtBaro = (imuSampleTime_ms - lastHgtPassTime_ms)*1.0e-3f;
                    const float hgtInnovFiltTC = 2.0f;
                    float alpha = constrain_float(dtBaro/(dtBaro+hgtInnovFiltTC),0.0f,1.0f);
                    hgtInnovFiltState += (innovVelPos[5]-hgtInnovFiltState)*alpha;
                } else {
                    hgtInnovFiltState = 0.0f;
                }

                // if timed out, reset the height
                if (hgtTimeout) {
                    ResetHeight();
                }

                // If we have got this far then declare the height data as healthy and reset the timeout counter
                hgtHealth = true;
                lastHgtPassTime_ms = imuSampleTime_ms;
            }
        }

        // set range for sequential fusion of velocity and position measurements depending on which data is available and its health
        //���ݿ��õ����ݼ��佡������������ٶȡ�λ��˳���ںϵķ�Χ
        if (fuseVelData && velHealth) {//fuseVelData �ٶ������ںϵı�־λ��velHearth ����ٶ�����ͨ��һ���Լ������true
            fuseData[0] = true;
            fuseData[1] = true;
            if (useGpsVertVel) {//��ʹ��GPS��ֱ�ٶ�����true
                fuseData[2] = true;
            }
			//tiltErrVec �����һ�������ٶȡ�λ���ںϵ���̬���������
            tiltErrVec.zero();//tiltErrVec ����Ĵ�λ�á��ٶ��ںϵõ�����̬���������
        }
		//posHealth ���λ������ͨ����Ϣ��⣬����posHealthΪtrue
        if (fusePosData && posHealth) {//fusePosData λ���ںϵı�־λ 
            fuseData[3] = true;
            fuseData[4] = true;
            tiltErrVec.zero();
        }
        if (fuseHgtData && hgtHealth) {
            fuseData[5] = true;
        }

        // fuse measurements sequentially   ˳����ں�������Ϣ λ���ٶ��ں�
        for (obsIndex=0; obsIndex<=5; obsIndex++) {
            if (fuseData[obsIndex]) {
                stateIndex = 3 + obsIndex;
                // calculate the measurement innovation, using states from a different time coordinate if fusing height data
                // adjust scaling on GPS measurement noise variances if not enough satellites
                //����������£�����ںϸ߶�������ʹ�ò�ͬʱ������ϵ��״̬
                //�����㹻���ǣ������GPS���������������ű���
                if (obsIndex <= 2)
                {
                    innovVelPos[obsIndex] = stateStruct.velocity[obsIndex] - velPosObs[obsIndex];
                    R_OBS[obsIndex] *= sq(gpsNoiseScaler);//��������R��
                }
                else if (obsIndex == 3 || obsIndex == 4) {
                    innovVelPos[obsIndex] = stateStruct.position[obsIndex-3] - velPosObs[obsIndex];
                    R_OBS[obsIndex] *= sq(gpsNoiseScaler);
                } else if (obsIndex == 5) {
                    innovVelPos[obsIndex] = stateStruct.position[obsIndex-3] - velPosObs[obsIndex];
                    const float gndMaxBaroErr = 4.0f;
                    const float gndBaroInnovFloor = -0.5f;

                    if(getTouchdownExpected() && activeHgtSource == HGT_SOURCE_BARO) {
                        // when a touchdown is expected, floor the barometer innovation at gndBaroInnovFloor
                        // constrain the correction between 0 and gndBaroInnovFloor+gndMaxBaroErr
                        // this function looks like this:
                        //         |/
                        //---------|---------
                        //    ____/|
                        //   /     |
                        //  /      |
                        innovVelPos[5] += constrain_float(-innovVelPos[5]+gndBaroInnovFloor, 0.0f, gndBaroInnovFloor+gndMaxBaroErr);
                    }
                }

                // calculate the Kalman gain and calculate innovation variances
                //�����˲������һ��Ԥ��Э������  ֻ�öԽ���Ԫ��
                //һ��Ԥ��P:P = P * H / (H * P * H' + R),����H������ӦԪ��Ϊ1���ʱ����P = P / (P + R)
                varInnovVelPos[obsIndex] = P[stateIndex][stateIndex] + R_OBS[obsIndex];
                SK = 1.0f/varInnovVelPos[obsIndex];//  /(P + R)
                for (uint8_t i= 0; i<=15; i++) {
                    Kfusion[i] = P[i][stateIndex]*SK;
                }

                // inhibit magnetic field state estimation by setting Kalman gains to zero
                //ͨ�����˲�������������ֹ�ų�״̬����
                if (!inhibitMagStates) {//����ų�״̬����Э����㶨���䣬��inhibitMagState��true
                    for (uint8_t i = 16; i<=21; i++) {
                        Kfusion[i] = P[i][stateIndex]*SK;
                    }
                } else {//����ų�״̬����Э����䣬�����˲���������
                    for (uint8_t i = 16; i<=21; i++) {
                        Kfusion[i] = 0.0f;
                    }
                }

                // inhibit wind state estimation by setting Kalman gains to zero
                //ͨ�����˲�������������ֹ����״̬����
                if (!inhibitWindStates) {//������ٵ�״̬����Э����ֲ�����inhibitWindState��true
                    Kfusion[22] = P[22][stateIndex]*SK;
                    Kfusion[23] = P[23][stateIndex]*SK;
                } else {//���ٵ�״̬����Э����ֲ���
                    Kfusion[22] = 0.0f;
                    Kfusion[23] = 0.0f;
                }

                // update the covariance - take advantage of direct observation of a single state at index = stateIndex to reduce computations
                // this is a numerically optimised implementation of standard equation P = (I - K*H)*P;
                //����״̬���Ƶ�Э������� - ���õ���״̬��index = stateIndex����ֱ�ӹ۲���ټ�����
                //���Ǳ�׼����P = (I - K * H) * P��ֵʵ�ֵļ�
                for (uint8_t i= 0; i<=stateIndexLim; i++) {
                    for (uint8_t j= 0; j<=stateIndexLim; j++)
                    {
                        KHP[i][j] = Kfusion[i] * P[stateIndex][j];
                    }
                }
                // Check that we are not going to drive any variances negative and skip the update if so
                //����Ƿ����P�����Խ���Ϊ����������������������˴θ���
                bool healthyFusion = true;
                for (uint8_t i= 0; i<=stateIndexLim; i++) {
                    if (KHP[i][i] > P[i][i]) {
                        healthyFusion = false;
                    }
                }
                if (healthyFusion) {
                    // update the covariance matrix
                    for (uint8_t i= 0; i<=stateIndexLim; i++) {
                        for (uint8_t j= 0; j<=stateIndexLim; j++) {
                            P[i][j] = P[i][j] - KHP[i][j];
                        }
                    }

                    // force the covariance matrix to be symmetrical and limit the variances to prevent ill-condiioning.
                    //ǿ��Э�������P�Գƣ�������Э�����С�Է�ֹ�˲�����̬
                    ForceSymmetry();
                    ConstrainVariances();

                    // update the states
                    // zero the attitude error state - by definition it is assumed to be zero before each observaton fusion
                    //���ݶ���ÿ�ν��������ں�ǰ����״̬��������̬�������
                    //��ʵÿ���˲��󶼽�״̬��������̬���ٶ���λ���������
                    stateStruct.angErr.zero();//ƴ�����:��Ϊ�������÷���һ����������ã�������stateStruct��һ���ṹ�壬angErr������һ������ �����ֱ�ӵ��ó�Ա����

                    // calculate state corrections and re-normalise the quaternions for states predicted using the blended IMU data
                    //����״̬�����������û��IMU���ݽ�״̬Ԥ���е���Ԫ�����¹�һ��
                    // X = x + k * (z - H * x)  ǰ����H * x - z
                    for (uint8_t i = 0; i<=stateIndexLim; i++) {
                        statesArray[i] = statesArray[i] - Kfusion[i] * innovVelPos[obsIndex];
                    }

                    // the first 3 states represent the angular misalignment vector. This is
                    // is used to correct the estimated quaternion
                    //ǰ����״̬����̬�������������Ƶ���Ԫ��
                    //rotate()��input:��̬������ʽ���ݵĵ�ǰ����Ԫ�أ�output:���������Ԫ��
                    stateStruct.quat.rotate(stateStruct.angErr);

                    // sum the attitude error from velocity and position fusion only
                    // used as a metric for convergence monitoring
                    //���ٶȺ�λ���ںϵõ�����̬�����ӣ����������������
                    if (obsIndex != 5) {
                        tiltErrVec += stateStruct.angErr;
                    }
                    // record good fusion status
                    if (obsIndex == 0) {
                        faultStatus.bad_nvel = false;
                    } else if (obsIndex == 1) {
                        faultStatus.bad_evel = false;
                    } else if (obsIndex == 2) {
                        faultStatus.bad_dvel = false;
                    } else if (obsIndex == 3) {
                        faultStatus.bad_npos = false;
                    } else if (obsIndex == 4) {
                        faultStatus.bad_epos = false;
                    } else if (obsIndex == 5) {
                        faultStatus.bad_dpos = false;
                    }
                } else {
                    // record bad fusion status
                    if (obsIndex == 0) {
                        faultStatus.bad_nvel = true;
                    } else if (obsIndex == 1) {
                        faultStatus.bad_evel = true;
                    } else if (obsIndex == 2) {
                        faultStatus.bad_dvel = true;
                    } else if (obsIndex == 3) {
                        faultStatus.bad_npos = true;
                    } else if (obsIndex == 4) {
                        faultStatus.bad_epos = true;
                    } else if (obsIndex == 5) {
                        faultStatus.bad_dpos = true;
                    }
                }
            }
        }
    }

    // stop performance timer
    hal.util->perf_end(_perf_FuseVelPosNED);
}

/********************************************************
*                   MISC FUNCTIONS                      *
********************************************************/

// select the height measurement to be fused from the available baro, range finder and GPS sources
void NavEKF2_core::selectHeightForFusion()
{
    // Read range finder data and check for new data in the buffer
    // This data is used by both height and optical flow fusion processing
    readRangeFinder();
    rangeDataToFuse = storedRange.recall(rangeDataDelayed,imuDataDelayed.time_ms);

    // correct range data for the body frame position offset relative to the IMU
    // the corrected reading is the reading that would have been taken if the sensor was
    // co-located with the IMU
    if (rangeDataToFuse) {
        AP_RangeFinder_Backend *sensor = frontend->_rng.get_backend(rangeDataDelayed.sensor_idx);
        if (sensor != nullptr) {
            Vector3f posOffsetBody = sensor->get_pos_offset() - accelPosOffset;
            if (!posOffsetBody.is_zero()) {
                Vector3f posOffsetEarth = prevTnb.mul_transpose(posOffsetBody);
                rangeDataDelayed.rng += posOffsetEarth.z / prevTnb.c.z;
            }
        }
    }

    // read baro height data from the sensor and check for new data in the buffer
    readBaroData();
    baroDataToFuse = storedBaro.recall(baroDataDelayed, imuDataDelayed.time_ms);

    // select height source
    if (extNavUsedForPos) {
        // always use external vision as the hight source if using for position.
        activeHgtSource = HGT_SOURCE_EV;
    } else if (((frontend->_useRngSwHgt > 0) || (frontend->_altSource == 1)) && (imuSampleTime_ms - rngValidMeaTime_ms < 500)) {
        if (frontend->_altSource == 1) {
            // always use range finder
            activeHgtSource = HGT_SOURCE_RNG;
        } else {
            // determine if we are above or below the height switch region
            float rangeMaxUse = 1e-4f * (float)frontend->_rng.max_distance_cm_orient(ROTATION_PITCH_270) * (float)frontend->_useRngSwHgt;
            bool aboveUpperSwHgt = (terrainState - stateStruct.position.z) > rangeMaxUse;
            bool belowLowerSwHgt = (terrainState - stateStruct.position.z) < 0.7f * rangeMaxUse;

            // If the terrain height is consistent and we are moving slowly, then it can be
            // used as a height reference in combination with a range finder
            // apply a hysteresis to the speed check to prevent rapid switching
            float horizSpeed = norm(stateStruct.velocity.x, stateStruct.velocity.y);
            bool dontTrustTerrain = ((horizSpeed > frontend->_useRngSwSpd) && filterStatus.flags.horiz_vel) || !terrainHgtStable;
            float trust_spd_trigger = MAX((frontend->_useRngSwSpd - 1.0f),(frontend->_useRngSwSpd * 0.5f));
            bool trustTerrain = (horizSpeed < trust_spd_trigger) && terrainHgtStable;

            /*
             * Switch between range finder and primary height source using height above ground and speed thresholds with
             * hysteresis to avoid rapid switching. Using range finder for height requires a consistent terrain height
             * which cannot be assumed if the vehicle is moving horizontally.
            */
            if ((aboveUpperSwHgt || dontTrustTerrain) && (activeHgtSource == HGT_SOURCE_RNG)) {
                // cannot trust terrain or range finder so stop using range finder height
                if (frontend->_altSource == 0) {
                    activeHgtSource = HGT_SOURCE_BARO;
                } else if (frontend->_altSource == 2) {
                    activeHgtSource = HGT_SOURCE_GPS;
                }
            } else if (belowLowerSwHgt && trustTerrain && (activeHgtSource != HGT_SOURCE_RNG)) {
                // reliable terrain and range finder so start using range finder height
                activeHgtSource = HGT_SOURCE_RNG;
            }
        }
    } else if ((frontend->_altSource == 2) && ((imuSampleTime_ms - lastTimeGpsReceived_ms) < 500) && validOrigin && gpsAccuracyGood) {
        activeHgtSource = HGT_SOURCE_GPS;
    } else if ((frontend->_altSource == 3) && validOrigin && rngBcnGoodToAlign) {
        activeHgtSource = HGT_SOURCE_BCN;
    } else {
        activeHgtSource = HGT_SOURCE_BARO;
    }

    // Use Baro alt as a fallback if we lose range finder, GPS or external nav
    bool lostRngHgt = ((activeHgtSource == HGT_SOURCE_RNG) && ((imuSampleTime_ms - rngValidMeaTime_ms) > 500));
    bool lostGpsHgt = ((activeHgtSource == HGT_SOURCE_GPS) && ((imuSampleTime_ms - lastTimeGpsReceived_ms) > 2000));
    bool lostExtNavHgt = ((activeHgtSource == HGT_SOURCE_EV) && ((imuSampleTime_ms - extNavMeasTime_ms) > 2000));
    if (lostRngHgt || lostGpsHgt || lostExtNavHgt) {
        activeHgtSource = HGT_SOURCE_BARO;
    }

    // if there is new baro data to fuse, calculate filtered baro data required by other processes
    if (baroDataToFuse) {
        // calculate offset to baro data that enables us to switch to Baro height use during operation
        if  (activeHgtSource != HGT_SOURCE_BARO) {
            calcFiltBaroOffset();
        }
        // filtered baro data used to provide a reference for takeoff
        // it is is reset to last height measurement on disarming in performArmingChecks()
        if (!getTakeoffExpected()) {
            const float gndHgtFiltTC = 0.5f;
            const float dtBaro = frontend->hgtAvg_ms*1.0e-3f;
            float alpha = constrain_float(dtBaro / (dtBaro+gndHgtFiltTC),0.0f,1.0f);
            meaHgtAtTakeOff += (baroDataDelayed.hgt-meaHgtAtTakeOff)*alpha;
        }
    }

    // If we are not using GPS as the primary height sensor, correct EKF origin height so that
    // combined local NED position height and origin height remains consistent with the GPS altitude
    // This also enables the GPS height to be used as a backup height source
    if (gpsDataToFuse &&
            (((frontend->_originHgtMode & (1 << 0)) && (activeHgtSource == HGT_SOURCE_BARO)) ||
            ((frontend->_originHgtMode & (1 << 1)) && (activeHgtSource == HGT_SOURCE_RNG)))
            ) {
        correctEkfOriginHeight();
    }

    // Select the height measurement source
    if (extNavDataToFuse && (activeHgtSource == HGT_SOURCE_EV)) {
        hgtMea = -extNavDataDelayed.pos.z;
        posDownObsNoise = sq(constrain_float(extNavDataDelayed.posErr, 0.1f, 10.0f));
    } else if (rangeDataToFuse && (activeHgtSource == HGT_SOURCE_RNG)) {
        // using range finder data
        // correct for tilt using a flat earth model
        if (prevTnb.c.z >= 0.7) {
            // calculate height above ground
            hgtMea  = MAX(rangeDataDelayed.rng * prevTnb.c.z, rngOnGnd);
            // correct for terrain position relative to datum
            hgtMea -= terrainState;
            // enable fusion
            fuseHgtData = true;
            velPosObs[5] = -hgtMea;
            // set the observation noise
            posDownObsNoise = sq(constrain_float(frontend->_rngNoise, 0.1f, 10.0f));
            // add uncertainty created by terrain gradient and vehicle tilt
            posDownObsNoise += sq(rangeDataDelayed.rng * frontend->_terrGradMax) * MAX(0.0f , (1.0f - sq(prevTnb.c.z)));
        } else {
            // disable fusion if tilted too far
            fuseHgtData = false;
        }
    } else if  (gpsDataToFuse && (activeHgtSource == HGT_SOURCE_GPS)) {
        // using GPS data
        hgtMea = gpsDataDelayed.hgt;
        // enable fusion
        velPosObs[5] = -hgtMea;
        fuseHgtData = true;
        // set the observation noise using receiver reported accuracy or the horizontal noise scaled for typical VDOP/HDOP ratio
        if (gpsHgtAccuracy > 0.0f) {
            posDownObsNoise = sq(constrain_float(gpsHgtAccuracy, 1.5f * frontend->_gpsHorizPosNoise, 100.0f));
        } else {
            posDownObsNoise = sq(constrain_float(1.5f * frontend->_gpsHorizPosNoise, 0.1f, 10.0f));
        }
    } else if (baroDataToFuse && (activeHgtSource == HGT_SOURCE_BARO)) {
        // using Baro data
        hgtMea = baroDataDelayed.hgt - baroHgtOffset;
        // enable fusion
        velPosObs[5] = -hgtMea;
        fuseHgtData = true;
        // set the observation noise
        posDownObsNoise = sq(constrain_float(frontend->_baroAltNoise, 0.1f, 10.0f));
        // reduce weighting (increase observation noise) on baro if we are likely to be in ground effect
        if (getTakeoffExpected() || getTouchdownExpected()) {
            posDownObsNoise *= frontend->gndEffectBaroScaler;
        }
        // If we are in takeoff mode, the height measurement is limited to be no less than the measurement at start of takeoff
        // This prevents negative baro disturbances due to copter downwash corrupting the EKF altitude during initial ascent
        if (motorsArmed && getTakeoffExpected()) {
            hgtMea = MAX(hgtMea, meaHgtAtTakeOff);
        }
    } else {
        fuseHgtData = false;
    }

    // If we haven't fused height data for a while, then declare the height data as being timed out
    // set timeout period based on whether we have vertical GPS velocity available to constrain drift
    hgtRetryTime_ms = (useGpsVertVel && !velTimeout) ? frontend->hgtRetryTimeMode0_ms : frontend->hgtRetryTimeMode12_ms;
    if (imuSampleTime_ms - lastHgtPassTime_ms > hgtRetryTime_ms) {
        hgtTimeout = true;
    } else {
        hgtTimeout = false;
    }
}

#endif // HAL_CPU_CLASS
