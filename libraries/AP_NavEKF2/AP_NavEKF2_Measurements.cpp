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
//检查新的磁力计数据，如果可用，更新存储的磁力计量测
//为什么这么写?你会怎么写?差距?这里的你能用到哪里?如何用来优化自己的代码?
void NavEKF2_core::readMagData()
{//如果读不到磁数据，则返回
    if (!_ahrs->get_compass()) {
        allMagSensorsFailed = true;//注意变量起名的技巧
        return;        
    }
    // If we are a vehicle with a sideslip constraint to aid yaw estimation and we have timed out on our last avialable
    // magnetometer, then declare the magnetometers as failed for this flight
    //如果车辆有侧滑约束去辅助航向估计且上时刻可用的磁数据超时，那么可断定在该次运动中磁数据不可用
    uint8_t maxCount = _ahrs->get_compass()->get_count();//_ahrs_get_compass()返回一个const Compass类型的指针，该指针调用get_count()方法
    if (allMagSensorsFailed || (magTimeout && assume_zero_sideslip() && magSelectIndex >= maxCount-1 && inFlight)) {
        allMagSensorsFailed = true;
        return;
    }

    // do not accept new compass data faster than 14Hz (nominal rate is 10Hz) to prevent high processor loading
    // because magnetometer fusion is an expensive step and we could overflow the FIFO buffer
    //拒绝更新频率大于14Hz的磁数据(标准更新频率为10Hz)，防止处理器负担过重
    //因为磁数据融合费内存，防止FIFO缓冲区溢出
    //_ahrs->get_compass()返回一个compass类型的指针，该指针调用compass类中的函数last_update_usec()
    if (use_compass() && _ahrs->get_compass()->last_update_usec() - lastMagUpdate_us > 70000) {
        frontend->logging.log_compass = true;// ->的左边是指针，句点的左边是对象  frontned是NavEKF2类型的对象 logging是一个结构体

        // If the magnetometer has timed out (been rejected too long) we find another magnetometer to use if available
        // Don't do this if we are on the ground because there can be magnetic interference and we need to know if there is a problem
        // before taking off. Don't do this within the first 30 seconds from startup because the yaw error could be due to large yaw gyro bias affsets
        //若磁数据超时或自检一直不通过，此时若有另一个磁罗盘则切换
        //不要在地面执行此操作，因为地面可能会有磁干扰；
        //不要再起飞30s内执行此操作，因为刚开始的航向误差可能是由较大的陀螺偏移引起的
        if (magTimeout && (maxCount > 1) && !onGround && imuSampleTime_ms - ekfStartTime_ms > 30000) {

            // search through the list of magnetometers
            //寻找磁罗盘列表
            for (uint8_t i=1; i<maxCount; i++) {
                uint8_t tempIndex = magSelectIndex + i;
                // loop back to the start index if we have exceeded the bounds
                //如果超出边界，则回到起始索引
                if (tempIndex >= maxCount) {
                    tempIndex -= maxCount;
                }
                // if the magnetometer is allowed to be used for yaw and has a different index, we start using it
                //如果磁力计可用于偏航角计算且有一个不同的索引，则开始使用磁力计
                if (_ahrs->get_compass()->use_for_yaw(tempIndex) && tempIndex != magSelectIndex) {
                    magSelectIndex = tempIndex;
                    gcs().send_text(MAV_SEVERITY_INFO, "EKF2 IMU%u switching to compass %u",(unsigned)imu_index,magSelectIndex);
                    // reset the timeout flag and timer 重置超时标志和计时器
                    magTimeout = false;
                    lastHealthyMagTime_ms = imuSampleTime_ms;
                    // zero the learned magnetometer bias states
                    stateStruct.body_magfield.zero();
                    // clear the measurement buffer
                    storedMag.reset();
                    // clear the data waiting flag so that we do not use any data pending from the previous sensor
                    //清除数据等待标志，以便于我们不使用先前传感器有待处理的任何数据
                    magDataToFuse = false;
                    // request a reset of the magnetic field states
                    magStateResetRequest = true;
                    // declare the field unlearned so that the reset request will be obeyed
                    magFieldLearned = false;
                }
            }
        }

        // detect changes to magnetometer offset parameters and reset states
        //检测磁力计偏移参数的变化并重置状态 _ahrs->get_compass()返回的是一个Compass类型的指针，故接下来可直接调用Compass类中的成员函数
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

        // store time of last measurement update 存储上一次量测更新的时间
        lastMagUpdate_us = _ahrs->get_compass()->last_update_usec(magSelectIndex);

        // estimate of time magnetometer measurement was taken, allowing for delays
        //估计磁力计测量的时间延迟，允许延迟
        magDataNew.time_ms = imuSampleTime_ms - frontend->magDelay_ms;

        // Correct for the average intersampling delay due to the filter updaterate
        //修正由于滤波器更新导致的平均采样时间延迟
        magDataNew.time_ms -= localFilterTimeStep_ms/2;

        // read compass data and scale to improve numerical conditioning
        //读磁力计数据和标度因数，用来改善数值精度
        //return _state[i].field 是get_field的函数具体实现，_state[]是一个结构体数组，field的Vector3f类型，表示校正后的磁场强度
        //_ahrs->get_compass()返回一个指向Compass类型的const指针，然后该指针再调用Compass的成员函数get_field
        magDataNew.mag = _ahrs->get_compass()->get_field(magSelectIndex) * 0.001f;

        // check for consistent data between magnetometers
        //_ahrs->get_compass返回一个const的Compass类型的指针，通过该指针调用Compass类的成员方法consistent
        consistentMagData = _ahrs->get_compass()->consistent();

        // save magnetometer measurement to buffer to be fused later
        //将磁力计量测存储到buf中，等待之后融合
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
 // 读取IMU角度测量和速度测量的delta值，采样频率100Hz，存储在EKF使用的数据缓冲区中，
 //若IMU数据到达的频率低于100Hz，不进行降采样或不采样。
 //在不引入圆锥误差或划船误差的前提下完成降采样
 
 //为什么作者这么写?我会怎么写?我学到了什么?学到了可以用到哪里?怎么优化?
void NavEKF2_core::readIMUData()
{   //AP::ins()返回一个AP_InertialSensor类型的引用，并该将该引用赋值给ins。
    //AP是一个namespace 具体实现为return *AP_InertialSensor::get_instance();
    const AP_InertialSensor &ins = AP::ins();//得到IMU数据  AP是名称空间，ins是名称空间中一个函数

    // average IMU sampling rate
    //平均IMU的采样速率  返回主循环的两次循环间隔，单位:s
    dtIMUavg = ins.get_loop_delta_t();

    // the imu sample time is used as a common time reference throughout the filter
    //IMU的采样时间用作整个滤波器的公共时间基准
	imuSampleTime_ms = AP_HAL::millis();

    // use the nominated imu or primary if not available
    //如果不可用，则使用指定的活默认的IMU
    if (ins.use_accel(imu_index)) {
		//从加速度获得积分速度
		//accelPosOffset 载体系中IMU的安装位置
        readDeltaVelocity(imu_index, imuDataNew.delVel, imuDataNew.delVelDT);
        accelPosOffset = ins.get_imu_pos_offset(imu_index);
    } else {
        readDeltaVelocity(ins.get_primary_accel(), imuDataNew.delVel, imuDataNew.delVelDT);
        accelPosOffset = ins.get_imu_pos_offset(ins.get_primary_accel());
    }

    // Get delta angle data from primary gyro or primary if not available
    //从陀螺仪获得积分角度
	if (ins.use_gyro(imu_index)) {
        readDeltaAngle(imu_index, imuDataNew.delAng, imuDataNew.delAngDT);
    } else {
        readDeltaAngle(ins.get_primary_gyro(), imuDataNew.delAng, imuDataNew.delAngDT);
    }

    // Get current time stamp
    //获取运行到此处的时间
    imuDataNew.time_ms = imuSampleTime_ms;

    // Accumulate the measurement time interval for the delta velocity and angle data
    //积累的角度和角速度数据的测量时间  累积积分时间
    //imuDataDownSampleNew 当前时刻的IMU数据，已经被降采样到100Hz
    //imuDataNew当前时刻的IMU数据 
    //数据类型定义澹� struct imu_elements{Vector3f delAng;Vector3f delVel;float delAngDT;float delVecDT;uint32_t time_ms}
	imuDataDownSampledNew.delAngDT += imuDataNew.delAngDT;
    imuDataDownSampledNew.delVelDT += imuDataNew.delVelDT;

    // Rotate quaternon atitude from previous to new and normalise.
    // Accumulation using quaternions prevents introduction of coning errors due to downsampling
    //得到新的旋转四元素角度，并归一化
    //使用四元素累积，防止由于降采样引入圆锥误差
    //累积的积分角度
    //imuQuatDownSampleNew 从降采样开始利用IMU的角度增量获得的四元素
	imuQuatDownSampleNew.rotate(imuDataNew.delAng);//由角度增量求四元数
    imuQuatDownSampleNew.normalize();//四元数归一化

    // Rotate the latest delta velocity into body frame at the start of accumulation
    //将最近的角度测量转换到刚开始时的载体系中  根据四元素初始化姿态矩阵
    //求陀螺仪变化的旋转矩阵，目的是把数据的改变投影过来  利用四元素求此时的姿态矩阵
    //利用四元素求姿态矩阵 deltaRotMat是四元素计算出的姿态矩阵
	Matrix3f deltaRotMat;
    imuQuatDownSampleNew.rotation_matrix(deltaRotMat);

    // Apply the delta velocity to the delta velocity accumulator
    //将速度增量加到速度累加器中 将b系的速度增量转换到n系
    //累计的积分速度 将几次的速度增量转换到n系后累加
    //将b系下速度增量转换到导航系下
    imuDataDownSampledNew.delVel += deltaRotMat*imuDataNew.delVel;

    // Keep track of the number of IMU frames since the last state prediction
    //记录IMU预测的个数  一次滤波过程中增量累加的次数。
    //用于降采样，采样多少次进行一次降采样
    framesSincePredict++;

    /*
     * If the target EKF time step has been accumulated, and the frontend has allowed start of a new predict cycle,
     * then store the accumulated IMU data to be used by the state prediction, ignoring the frontend permission if more
     * than twice the target time has lapsed. Adjust the target EKF step time threshold to allow for timing jitter in the
     * IMU data.
     */
     //如果目标EKF的时间步长已经累积且前端允许开启一个新的预测周期，存储累积的IMU数据用于状态预测
     //如果超过目标时间的两倍，则忽略前端请求。调整EKF时间步长的阈值，以允许在IMU数据中的时间抖动
     //EKF_TARGET_DT 是目标EKF更新的时间步长 dtIMUavg IMU两次测量之间的期望时间
     //EKF处理imu数据间隔，不用每次都处理，处理频率约50Hz，GPS频率为5Hz，气压计10Hz
     //为何做此判断?
     //此时frameSinsePredict开始做降采样，可根据判断条件得到framSincePredict
     //为何这么多时间判断?
    if ((dtIMUavg*(float)framesSincePredict >= (EKF_TARGET_DT-(dtIMUavg*0.5)) &&
         startPredictEnabled) || (dtIMUavg*(float)framesSincePredict >= 2.0f*EKF_TARGET_DT)) {

        // convert the accumulated quaternion to an equivalent delta angle
        //转换累积的四元素，得到一个等价的角增量  将角增量转换为轴角
        //通过imu在这端时间的变化量求出旋转的弧度大小
        //imuDataDownSampleNew.delAng中存放的是求得的轴角
        imuQuatDownSampleNew.to_axis_angle(imuDataDownSampledNew.delAng);

        // Time stamp the data  记录采样时间
        //获取时间 不是时间间隔，应该是从开机以来的时间
        imuDataDownSampledNew.time_ms = imuSampleTime_ms;

        // Write data to the FIFO IMU buffer 将新数据写入缓冲区
        //记录本次采样数据到环形buff
        //为何把数据写入buf中而不是直接传递计算?
        //imuDataDownSampleNew  struct imu_elements{Vector3f delAng;Vector3f delVel;float delAngDT;float delVelDT;uint_32t time_ms}
        storedIMU.push_youngest_element(imuDataDownSampledNew);

        // calculate the achieved average time step rate for the EKF
        //计算EKF数据的平均步长  dtEkfAvg 两次EKF更新的期望时间
        float dtNow = constrain_float(0.5f*(imuDataDownSampledNew.delAngDT+imuDataDownSampledNew.delVelDT),0.0f,10.0f*EKF_TARGET_DT);
        dtEkfAvg = 0.98f * dtEkfAvg + 0.02f * dtNow;

        // zero the accumulated IMU data and quaternion
        //将累积的四元素和IMU数据置零  情况采样累加器
        //imuDataDownSampleNew已被存入环形buf，此时将该结构体中元素清零，下次用时直接从buf中取即可
        imuDataDownSampledNew.delAng.zero();
        imuDataDownSampledNew.delVel.zero();
        imuDataDownSampledNew.delAngDT = 0.0f;
        imuDataDownSampledNew.delVelDT = 0.0f;
        imuQuatDownSampleNew[0] = 1.0f;
        imuQuatDownSampleNew[3] = imuQuatDownSampleNew[2] = imuQuatDownSampleNew[1] = 0.0f;

        // reset the counter used to let the frontend know how many frames have elapsed since we started a new update cycle
        //重置计数器 滤波器的频率低于IMU更新的频率，故对IMU进行降采样，求平均后再滤波
        //将标志位清零，纯捷联多少次进行一次EKF
        framesSincePredict = 0;

        // set the flag to let the filter know it has new IMU data and needs to run
        //设置标志位，让滤波器知道新的IMU数据已经到来，需要运行
        //runUpdates为true，表明可以进行EKF，这事EKF之前的状态判断标志位
        runUpdates = true;

        // extract the oldest available data from the FIFO buffer
        //从缓冲区取出较老的可用的数据
        //读取最久的一次采样，这个跟最新的数据相差了size(13或26)个采样
        //为何取出最旧的数据，不应该是刚刚存入的数据吗?
        imuDataDelayed = storedIMU.pop_oldest_element();

        // protect against delta time going to zero
        // TODO - check if calculations can tolerate 0
        //dtEkfAvg 两次EKF更新的时间间隔
        //这样可以保证delta_time永远不会为零
        //delAngDT和delVelDT应该是一样的，为何定义两个?
        float minDT = 0.1f*dtEkfAvg;
        imuDataDelayed.delAngDT = MAX(imuDataDelayed.delAngDT,minDT);
        imuDataDelayed.delVelDT = MAX(imuDataDelayed.delVelDT,minDT);

        updateTimingStatistics();
            
        // correct the extracted IMU data for sensor errors
        //修正提取的IMU数据  乘以标度因数再减去零偏
        delAngCorrected = imuDataDelayed.delAng;
        delVelCorrected = imuDataDelayed.delVel;
        correctDeltaAngle(delAngCorrected, imuDataDelayed.delAngDT);
        correctDeltaVelocity(delVelCorrected, imuDataDelayed.delVelDT);

    } else {
        // we don't have new IMU data in the buffer so don't run filter updates on this time step
        //若未到EKF的时刻，则EKF滤波标志位为零，不进入EKF循环。
        runUpdates = false;
    }
}

// read the delta velocity and corresponding time interval from the IMU
// return false if data is not available
//如果数据不可用，从IMU中读取速度增量和相应时间，并返回false
bool NavEKF2_core::readDeltaVelocity(uint8_t ins_index, Vector3f &dVel, float &dVel_dt) {
    const AP_InertialSensor &ins = AP::ins();//命名空间，AP::ins()返回一个指向AP_InertialSensor的引用，且引用是const类型

    if (ins_index < ins.get_accel_count()) {
        ins.get_delta_velocity(ins_index,dVel);//get_accel(i) * get_delta_time()
        dVel_dt = MAX(ins.get_delta_velocity_dt(ins_index),1.0e-4f);//为什么会 有这个判断?
        dVel_dt = MIN(dVel_dt,1.0e-1f);
        return true;
    }
    return false;
}

/********************************************************
*             Global Position Measurement               *
********************************************************/

// check for new valid GPS data and update stored measurement if available
//检查新的可用的GPS数据，如果GPS可用更新存储的量测信息
void NavEKF2_core::readGpsData()
{
    // check for new GPS data
    // do not accept data at a faster rate than 14Hz to avoid overflowing the FIFO buffer
    //接收GPS数据速率小于等于14Hz，避免FIFO缓冲区溢出
    const AP_GPS &gps = AP::gps();//AP 命名空间 该gps函数为:AP_GPS &gps(){return AP_GPS::gps();}
    if (gps.last_message_time_ms() - lastTimeGpsReceived_ms > 70) {
        if (gps.status() >= AP_GPS::GPS_OK_FIX_3D) {//gps.status() 是主GPS的状态
            // report GPS fix status  GPS_OK_FIX_3D是GPS_Status中第四个状态变量
            //gpsCheckStatus是一个包含众多bool标志位的结构体，表明位置、速度等是否正确
            gpsCheckStatus.bad_fix = false;

            // store fix time from previous read
            secondLastGpsTime_ms = lastTimeGpsReceived_ms;//上次收到GPS数据的时间

            // get current fix time 
            lastTimeGpsReceived_ms = gps.last_message_time_ms();


            // estimate when the GPS fix was valid, allowing for GPS processing and other delays
            // ideally we should be using a timing signal from the GPS receiver to set this time
            //估计什么时候GPS定位是有效的，考虑到GPS处理和其他延迟，理想情况下我们应该用GPS接收机
            //的时间来设置该时间
            float gps_delay = 0.0;
            gps.get_lag(gps_delay); // ignore the return value
            gpsDataNew.time_ms = lastTimeGpsReceived_ms - (uint32_t)(1e3f * gps_delay);

            // Correct for the average intersampling delay due to the filter updaterate
            //修正由于滤波器更新频率导致的平均采样时间延迟
            //localFilterTimeStep 两次滤波器更新之间的平均时间 单位:ms
            //gpsDataNew 当前时刻的gps数据  
            //结构体 struct gps_elements{vector2f pos;float hgt;vector3f vel;uint32_t time_ms;uint8_t sensor_idx;}
            gpsDataNew.time_ms -= localFilterTimeStep_ms/2;

            // Prevent time delay exceeding age of oldest IMU data in the buffer
            //防止时间延迟超过buffer中最早imu的存放时间
            gpsDataNew.time_ms = MAX(gpsDataNew.time_ms,imuDataDelayed.time_ms);

            // Get which GPS we are using for position information
            //得到我们使用的GPS的位置信息  返回主传感器的索引
            gpsDataNew.sensor_idx = gps.primary_sensor();

            // read the NED velocity from the GPS
            //得到GPS的速度信息
            gpsDataNew.vel = gps.velocity();

            // Use the speed and position accuracy from the GPS if available, otherwise set it to zero.
            // Apply a decaying envelope filter with a 5 second time constant to the raw accuracy data
            //如果GPS速度、位置精度可用，则用之；否则，将其置为零；GPS的位置、速度和垂直方向位置精度
            //在连续的5s内将衰减包络滤波器应用于原始精度的数据
            //gpsSpdAccuracy GPS接收机返回的速度精度  单位:m/s
            float alpha = constrain_float(0.0002f * (lastTimeGpsReceived_ms - secondLastGpsTime_ms),0.0f,1.0f);
            gpsSpdAccuracy *= (1.0f - alpha);
            float gpsSpdAccRaw;
            if (!gps.speed_accuracy(gpsSpdAccRaw)) {//speed_accuracy是GPS提供的速度精度
                gpsSpdAccuracy = 0.0f;
            } else {
                gpsSpdAccuracy = MAX(gpsSpdAccuracy,gpsSpdAccRaw);
                gpsSpdAccuracy = MIN(gpsSpdAccuracy,50.0f);
            }
			//gpsPosAccuracy GPS接收机返回的位置精度 单位:m
            gpsPosAccuracy *= (1.0f - alpha);
            float gpsPosAccRaw;
            if (!gps.horizontal_accuracy(gpsPosAccRaw)) {
                gpsPosAccuracy = 0.0f;
            } else {
                gpsPosAccuracy = MAX(gpsPosAccuracy,gpsPosAccRaw);
                gpsPosAccuracy = MIN(gpsPosAccuracy,100.0f);
            }
			//gps接收机返回的高度精度 单位:m
            gpsHgtAccuracy *= (1.0f - alpha);
            float gpsHgtAccRaw;
            if (!gps.vertical_accuracy(gpsHgtAccRaw)) {
                gpsHgtAccuracy = 0.0f;
            } else {
                gpsHgtAccuracy = MAX(gpsHgtAccuracy,gpsHgtAccRaw);
                gpsHgtAccuracy = MIN(gpsHgtAccuracy,100.0f);
            }

            // check if we have enough GPS satellites and increase the gps noise scaler if we don't
            //检查是否有足够的GPS卫星，如果不检查则增加GPS的噪声水平
            //根据卫星数目挑战GPS位置、速度噪声水平
            //PV_AidingMode是一个枚举变量，有三个值，AID_ABSOLUTE AID_NONE AID_RELATIVE，分别为0 1 2 
            if (gps.num_sats() >= 6 && (PV_AidingMode == AID_ABSOLUTE)) {//num_sats() 由GPS直接提供的可视的卫星数
                gpsNoiseScaler = 1.0f;
            } else if (gps.num_sats() == 5 && (PV_AidingMode == AID_ABSOLUTE)) {
            //gpsNoiseScaler  在GPS卫星较少时用于缩放GPS量测噪声
                gpsNoiseScaler = 1.4f;
            } else { // <= 4 satellites or in constant position mode
                gpsNoiseScaler = 2.0f;
            }

            // Check if GPS can output vertical velocity, if it is allowed to be used, and set GPS fusion mode accordingly
            //检查GPS是否可输出垂直速度，如果允许使用垂直速度，相应地设置GPS融合模式
            //gps.have_vertical_velocity查看GPS是否提供垂直速度，如有，置1
            //useGpsVertvel 若GPS垂直速度被用到则置true
            if (gps.have_vertical_velocity() && frontend->_fusionModeGPS == 0 && !frontend->inhibitGpsVertVelUse) {
                useGpsVertVel = true;
            } else {
                useGpsVertVel = false;
            }

            // Monitor quality of the GPS velocity data before and after alignment using separate checks
            //单独检查GPS校准前后的速度质量
            //PV_AidingMode 定义来自惯导的位置、速度融合的优选模式
            if (PV_AidingMode != AID_ABSOLUTE) {//AID_ABSOLUTE 有绝对的位置速度参考
                // Pre-alignment checks
                //当GPS的质量可被用来初始化导航系统时，置true
                gpsGoodToAlign = calcGpsGoodToAlign();
            } else {
                gpsGoodToAlign = false;
            }

            // Post-alignment checks
            //更新飞行计算，确定GPS是否可用来导航
            calcGpsGoodForFlight();

            // Read the GPS location in WGS-84 lat,long,height coordinates
            //读WGS-84坐标系下的纬度、经度、高度
            //struct Location{ union {Location_option_Flag flags;uint8_t options;};int32_t alt:24;int32_t lat;int32_t lng}
            //高度*100 纬度、经度*10^7
			const struct Location &gpsloc = gps.location();

            // Set the EKF origin and magnetic field declination if not previously set  and GPS checks have passed
            //如果先前未设置且GPS自检通过，则设置EKF原点和磁偏角
            //若GPS的质量可用来初始化导航系统，则gpsGoodToAlign为true
            //若EKF原点是可用的，则validOrigin为true
            if (gpsGoodToAlign && !validOrigin) {
                setOrigin();

                // set the NE earth magnetic field states using the published declination
                // and set the corresponding variances and covariances
                //使用发布的偏角设置北向、东西的地理系下磁场
                //设置相应的方差和协方差矩阵
                alignMagStateDeclination();

                // Set the height of the NED origin 设置导航系起点的高度数据 refence height
                ekfGpsRefHgt = (double)0.01 * (double)gpsloc.alt + (double)outputDataNew.position.z;

                // Set the uncertainty of the GPS origin height  设置GPS起点高度的方差
                ekfOriginHgtVar = sq(gpsHgtAccuracy);//gpsHgtAccuracy是GPS接收机返回的GPS高度米级的精度

            }

            // convert GPS measurements to local NED and save to buffer to be fused later if we have a valid origin
            //将GPS量测信息转换到NED导航系下，若有初始可用的起点则保存到buf中以便稍后进行融合
            if (validOrigin) {//validOrigin若ekf初始的起点是可用的，则validOrigin返回true
                gpsDataNew.pos = location_diff(EKF_origin, gpsloc);//得到两个点之间的东西、北向位置误差
                gpsDataNew.hgt = (float)((double)0.01 * (double)gpsloc.alt - ekfGpsRefHgt);//高度误差
                storedGPS.push(gpsDataNew);
                // declare GPS available for use
                //gpsNotAvailable GPS数据不可用时置true
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
//读delta角和相应的IMU时间间隔，如果数据不可用返回false 
//gyro * delta_time
//最大最小值函数，
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
