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
//如果可用，则将状态中速度重置为上一时刻的GPS量测；如果处于恒定位置模式或pv辅助不是绝对的，则将其重置为零
//勿使用GPS重置垂直速度，因为气压计的高度是可用的，能够用来限制漂移
void NavEKF2_core::ResetVelocity(void)
{
    // Store the position before the reset so that we can record the reset delta
    //stateStruct结构体是28维的状态向量，velocity是Vector3类型变量，速度顺序是3~5(前三个是姿态误差angErr)
    velResetNE.x = stateStruct.velocity.x;
    velResetNE.y = stateStruct.velocity.y;

    // reset the corresponding covariances
    //将协方差矩阵中与水平速度相关的行、列置零
    zeroRows(P,3,4);
    zeroCols(P,3,4);
    //PV_AidingMode 辅助惯导进行速度、位置融合的GPS优选模式
    //difines the preferred mode for aiding of velocity and position estimates from the INS
    if (PV_AidingMode != AID_ABSOLUTE) {//AID_ABSOLUTE 绝对的位置参考辅助，如果不是绝对的位置参考，将将其置零
        stateStruct.velocity.zero();
        // set the variances using the measurement noise parameter
        P[4][4] = P[3][3] = sq(frontend->_gpsHorizVelNoise);//fronted 是指向类的指针，通过->调用
    } else {
        // reset horizontal velocity states to the GPS velocity if available
        //将状态中的水平速度重置为上一时刻GPS速度
        //imuSampleTime_ms上次imu数据被取走的时间 lastTimeGpsReceived_ms 上次收到GPS数据的时间
        if (imuSampleTime_ms - lastTimeGpsReceived_ms < 250) {
            stateStruct.velocity.x  = gpsDataNew.vel.x;
            stateStruct.velocity.y  = gpsDataNew.vel.y;
            // set the variances using the reported GPS speed accuracy
            //第一个GPS噪声哪里来的?
            //frontend 是指向NavEKF2对象的指针
            P[4][4] = P[3][3] = sq(MAX(frontend->_gpsHorizVelNoise,gpsSpdAccuracy));//gpsSpdAccuracy 是接收机返回的速度精度
            // clear the timeout flags and counters
            //清除超时标志和计数器
            //velTimeout 若速度量测在新息一致性检查失败或超时，则置true
            //lastVelPassTime_ms 上次GPS速度量测通过一致性检查的时间
            velTimeout = false;
            lastVelPassTime_ms = imuSampleTime_ms;
        } else {
            stateStruct.velocity.x  = 0.0f;
            stateStruct.velocity.y  = 0.0f;
            // set the variances using the likely speed range 
            //采用可能的速度范围设置方差
            P[4][4] = P[3][3] = sq(25.0f);
            // clear the timeout flags and counters
            //清除超时标准和计数器
            velTimeout = false;
            lastVelPassTime_ms = imuSampleTime_ms;
        }
    }
	//输出状态的缓冲区
    for (uint8_t i=0; i<imu_buffer_length; i++) {
        storedOutput[i].velocity.x = stateStruct.velocity.x;
        storedOutput[i].velocity.y = stateStruct.velocity.y;
    }
	//输出当前时刻的状态数据
	//outputDataNew outputDataDelayed 都是输出当前时刻的状态数据 output_element类型
	//struct output_elements{Quaternion quat; Vector3f velocity; Vector3f position} 10个向量
    outputDataNew.velocity.x = stateStruct.velocity.x;
    outputDataNew.velocity.y = stateStruct.velocity.y;
    outputDataDelayed.velocity.x = stateStruct.velocity.x;
    outputDataDelayed.velocity.y = stateStruct.velocity.y;

    // Calculate the position jump due to the reset 计算由于重置造成的位置跳变
    //struct state_elements{Vector3f angErr; Vector3f velocity; Vector3f position; Vector3f gyro_bias;
    //Vector3f gyro_scale; float accel_zbias; Vector3f earth_magfield; Vector3f body_magfield; 
    //Vector2f wind_vel; Quaternion quat;}
    velResetNE.x = stateStruct.velocity.x - velResetNE.x;
    velResetNE.y = stateStruct.velocity.y - velResetNE.y;

    // store the time of the reset
    //lastVelReset 上次速度重置时的系统时间
    lastVelReset_ms = imuSampleTime_ms;


}

// resets position states to last GPS measurement or to zero if in constant position mode
//将状态中位置重置为上时刻GPS测量的位置，如果处于恒定位置模式则将其置零
void NavEKF2_core::ResetPosition(void)
{
    // Store the position before the reset so that we can record the reset delta
    //posResetNE 由于飞行中重置在东向、北向上改变的位置，单位m
    posResetNE.x = stateStruct.position.x;
    posResetNE.y = stateStruct.position.y;

    // reset the corresponding covariances
    //重置对应的协方差矩阵
    zeroRows(P,6,7);//将第6行至第7行置为零
    zeroCols(P,6,7);//将第6列至第7列置为零
    //AID_ABSOLUTE 有GPS或其他绝对的位置参考
    if (PV_AidingMode != AID_ABSOLUTE) {
        // reset all position state history to the last known position
        //将位置重置为上一时刻位置
        stateStruct.position.x = lastKnownPositionNE.x;
        stateStruct.position.y = lastKnownPositionNE.y;
        // set the variances using the position measurement noise parameter
        //用位置量测噪声重置相应协方差
        P[6][6] = P[7][7] = sq(frontend->_gpsHorizPosNoise);
    } else  {//如果处于绝对的位置参考辅助模式
        // Use GPS data as first preference if fresh data is available
        //若GPS数据可用，则将其作为首要选择
        //imuSampleTime_ms上次取IMU数据的时间
        //lastTimeGpsReceived 上次收到GPS数据的时间
        if (imuSampleTime_ms - lastTimeGpsReceived_ms < 250) {
            // record the ID of the GPS for the data we are using for the reset
            //记录此次重置用的GPS的ID  gpsDataNew当前时刻的GPS数据 last_gps_idx 上次融合或重置时GPS接收机的ID
            last_gps_idx = gpsDataNew.sensor_idx;
            // write to state vector and compensate for offset  between last GPS measurement and the EKF time horizon
            //写入状态向量并补偿EKF的时间和上次GPS量测时间之间的偏差 gpsDataNew 当前时刻的GPS数据
            //imuDataDelayed 融合时刻的imu数据
            stateStruct.position.x = gpsDataNew.pos.x  + 0.001f*gpsDataNew.vel.x*(float(imuDataDelayed.time_ms) - float(gpsDataNew.time_ms));
            stateStruct.position.y = gpsDataNew.pos.y  + 0.001f*gpsDataNew.vel.y*(float(imuDataDelayed.time_ms) - float(gpsDataNew.time_ms));
            // set the variances using the position measurement noise parameter
            P[6][6] = P[7][7] = sq(MAX(gpsPosAccuracy,frontend->_gpsHorizPosNoise));
            // clear the timeout flags and counters
            posTimeout = false;
            lastPosPassTime_ms = imuSampleTime_ms;//rngBcnLast3DmeasTime_ms 上次信标系统返回3D定位系统的时间 ms
        } else if (imuSampleTime_ms - rngBcnLast3DmeasTime_ms < 250) {
            // use the range beacon data as a second preference 第二个优先选择
            //receiverPos 接收机NED的位置
            stateStruct.position.x = receiverPos.x;
            stateStruct.position.y = receiverPos.y;
            // set the variances from the beacon alignment filter
            P[6][6] = receiverPosCov[0][0];
            P[7][7] = receiverPosCov[1][1];
            // clear the timeout flags and counters
            rngBcnTimeout = false;
            lastRngBcnPassTime_ms = imuSampleTime_ms;
        } else if (imuSampleTime_ms - extNavDataDelayed.time_ms < 250) {
            //extNavDataDelayed 融合时刻的其他导航信息
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
    //计算由于重置造成的位置跳变  posResetNE由于上次重置造成的位置改变
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
//选择融合速度、位置和高度信息
//位置速度、高度融合的系统模型和量测模型?24维的状态变量只融合速度高度、如何进行融合?
void NavEKF2_core::SelectVelPosFusion()
{
    // Check if the magnetometer has been fused on that time step and the filter is running at faster than 200 Hz
    // If so, don't fuse measurements on this time step to reduce frame over-runs
    // Only allow one time slip to prevent high rate magnetometer data preventing fusion of other measurements
    //检查磁力计在此时刻是否已经融合，检查滤波频率是否大于200Hz
    //若是，则此时刻不融合磁力计，以防止帧超调
    //仅允许一次跳跃，防止高速的磁力计阻止其他量测融合
    //dtIMUavg 两次IMU数据更新之间的期望时间  magFusePerformed 磁力计数据融合的标志位 
    //posVelFusionDelayed 当位置、速度融合延迟时被置为true
    if (magFusePerformed && dtIMUavg < 0.005f && !posVelFusionDelayed) {
        posVelFusionDelayed = true;
        return;
    } else {
        posVelFusionDelayed = false;
    }

    // Check for data at the fusion time horizon
    extNavDataToFuse = storedExtNav.recall(extNavDataDelayed, imuDataDelayed.time_ms);

    // read GPS data from the sensor and check for new data in the buffer
    //读GPS数据并检查buf中的新数据
    //为什么读GPS数据那么多判断????
    //buf中如何检测数据
    //有GPS数据参与融合 则置gpsDataToFuse为true
    readGpsData();
    gpsDataToFuse = storedGPS.recall(gpsDataDelayed,imuDataDelayed.time_ms);
    // Determine if we need to fuse position and velocity data on this time step
    //决定在当前时刻是否需要融合位置、速度
    //AID_ABSOLUTE 绝对的位置参考
    //gpsDataToFuse 融合的标志 多设置标志u8 或bool
    //fuseVelData NED的速度量测参与融合则置true
    //fusePosData NE的位置量测参与融合则置true
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
        //校正IMU相对于天线相位中心的位置偏移
        //accelPosOffset IMU在载体系中的位置
        Vector3f posOffsetBody = AP::gps().get_antenna_offset(gpsDataDelayed.sensor_idx) - accelPosOffset;
        if (!posOffsetBody.is_zero()) {//如果位置偏移非零
            // Don't fuse velocity data if GPS doesn't support it
            //如果GPS无速度，则不融合速度数据
            //fuseVelData 东北天速度融合的标志位
            if (fuseVelData) {
                // TODO use a filtered angular rate with a group delay that matches the GPS delay
                //使用滤波后的角速率，因为它有与GPS延迟匹配的时间延迟
                //angRate 转动角速率 即z轴陀螺
                Vector3f angRate = imuDataDelayed.delAng * (1.0f/imuDataDelayed.delAngDT);
				//%被重载为叉积 角速率和位置偏移的叉积为什么是速度偏移
                Vector3f velOffsetBody = angRate % posOffsetBody;
                Vector3f velOffsetEarth = prevTnb.mul_transpose(velOffsetBody);
                gpsDataDelayed.vel.x -= velOffsetEarth.x;//修正偏移之后的GPS速度数据
                gpsDataDelayed.vel.y -= velOffsetEarth.y;
                gpsDataDelayed.vel.z -= velOffsetEarth.z;
            }

            //gpsDataDelayed 融合时刻的GPS数据
            //修正位置偏移之后的GPS数据
			Vector3f posOffsetEarth = prevTnb.mul_transpose(posOffsetBody);
            gpsDataDelayed.pos.x -= posOffsetEarth.x;
            gpsDataDelayed.pos.y -= posOffsetEarth.y;
            gpsDataDelayed.hgt += posOffsetEarth.z;
        }

        // copy corrected GPS data to observation vector
        //将修正后的GPS数据复制给量测输出
        //若NED的速度量测被融合，则置fuseVelData为true
        //velPosObs 速度和位置的量测信息，注意命名方式
        if (fuseVelData) {
            velPosObs[0] = gpsDataDelayed.vel.x;
            velPosObs[1] = gpsDataDelayed.vel.y;
            velPosObs[2] = gpsDataDelayed.vel.z;
        }
        velPosObs[3] = gpsDataDelayed.pos.x;
        velPosObs[4] = gpsDataDelayed.pos.y;

    } else if (extNavDataToFuse && PV_AidingMode == AID_ABSOLUTE) {
        // This is a special case that uses and external nav system for position
        //这是特例，采用外部导航系统的位置
        //extNavDataToFuse 若有新的外部导航数据融合，则置true
        //PV_AidingMode == AID_ABSOLUTE  绝对的位置速度参考模式
        extNavUsedForPos = true;
        activeHgtSource = HGT_SOURCE_EV;
        fuseVelData = false;
        fuseHgtData = true;
        fusePosData = true;
		//velPosObs gps的量测信息，将位置速度放在一个数组中
        velPosObs[3] = extNavDataDelayed.pos.x;
        velPosObs[4] = extNavDataDelayed.pos.y;
        velPosObs[5] = extNavDataDelayed.pos.z;

        // if compass is disabled, also use it for yaw
        if (!use_compass()) {
            extNavUsedForYaw = true;
            if (!yawAlignComplete) {//当航向对准完成，置true
                extNavYawResetRequest = true;//若要求用外部导航数据重置航向，则置extNavYawResetRequest为true
                magYawResetRequest = false;//若载体航向和磁场状态变量需用磁场量测重置，则置magYawResetRequest为true
                gpsYawResetRequest = false;//若用GPS航线重置航向，则置gpsYawResetRequest为true
                controlMagYawReset();
                finalInflightYawInit = true;
            } else {
                fuseEulerYaw();
            }
        } else {//extNavUsedForYaw 当额外的导航数据用来作航向量测时置true
            extNavUsedForYaw = false;
        }

    } else {
        //不融合NED的速度量测，不融合NE的位置量测
		fuseVelData = false;
        fusePosData = false;
    }

    // we have GPS data to fuse and a request to align the yaw using the GPS course
    //使用GPS航线来对齐航向
    //使用GPS航线重置航向的原理???地图匹配???
    if (gpsYawResetRequest) {//收到用GPS航线重置载体航向的请求 gpsYawResetRequest为true
        realignYawGPS();
    }

    // Select height data to be fused from the available baro, range finder and GPS sources

    selectHeightForFusion();

    // if we are using GPS, check for a change in receiver and reset position and height
    //若使用GPS，则检查GPS接收机的更改并重置位置和高度
    if (gpsDataToFuse && PV_AidingMode == AID_ABSOLUTE && gpsDataDelayed.sensor_idx != last_gps_idx) {
        // record the ID of the GPS that we are using for the reset 记录GPS的ID
        last_gps_idx = gpsDataDelayed.sensor_idx;

        // Store the position before the reset so that we can record the reset delta
        //存储重置之前的位置
        posResetNE.x = stateStruct.position.x;
        posResetNE.y = stateStruct.position.y;

        // Set the position states to the position from the new GPS
        //根据最新的GPS设置位置状态
        stateStruct.position.x = gpsDataNew.pos.x;
        stateStruct.position.y = gpsDataNew.pos.y;

        // Calculate the position offset due to the reset 计算由于重置带来的位置偏移
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
        //若将GPS用于高度参考，则重置高度
        if (activeHgtSource == HGT_SOURCE_GPS) {
            // Store the position before the reset so that we can record the reset delta
            posResetD = stateStruct.position.z;

            // write to the state vector
            stateStruct.position.z = -hgtMea;

            // Calculate the position jump due to the reset
            //计算由于重置带来的位置跳变
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
//融合位置、速度和高度量测
void NavEKF2_core::FuseVelPosNED()
{
    // start performance timer
    hal.util->perf_begin(_perf_FuseVelPosNED);

    // health is set bad until test passed 
    //为什么设置如此多标志位???
    velHealth = false;
    posHealth = false;
    hgtHealth = false;

    // declare variables used to check measurement errors
    //声明变量，用来检测量测误差
    Vector3f velInnov;

    // declare variables used to control access to arrays
    //声明向量，用来控制数组接口
    bool fuseData[6] = {false,false,false,false,false,false};
    uint8_t stateIndex;
    uint8_t obsIndex;

    // declare variables used by state and covariance update calculations
    //声明变量，用来状态和协方差更新计算
    Vector6 R_OBS; // Measurement variances used for fusion 融合用的量测方差
    Vector6 R_OBS_DATA_CHECKS; // Measurement variances used for data checks only
    float SK;

    // perform sequential fusion of GPS measurements. This assumes that the
    // errors in the different velocity and position components are
    // uncorrelated which is not true, however in the absence of covariance
    // data from the GPS receiver it is the only assumption we can make
    // so we might as well take advantage of the computational efficiencies
    // associated with sequential fusion
    //执行GPS量测的顺序融合。该假设(不同速度和位置分量中的误差是不相关的)是错误的。
    //但是由于缺少GPS接收机的协方差数据，这是我们能做的唯一的假设。
    //所以不妨利用与顺序融合相关的计算效率
    if (fuseVelData || fusePosData || fuseHgtData) {//此三者是速度、位置、高度融合的标志

        // calculate additional error in GPS position caused by manoeuvring
        //计算由于机动引起的GPS附加的位置误差  gpsPosVarAccScale = 0.05f   NAVEKF2 *frontend
        //gpsPosVarAccScale 是由于机动加速造成的水平位置的标度因数误差
        //accNavMag 导航的加速度幅度，用于调整GPS量测方差
        float posErr = frontend->gpsPosVarAccScale * accNavMag;

        // estimate the GPS Velocity, GPS horiz position and height measurement variances.
        // Use different errors if operating without external aiding using an assumed position or velocity of zero
        //估计GPS速度、GPS水平位置和高度的量测方差
        //如果无外部辅助情况下，假定位置和速度为零，则使用不同的误差
        if (PV_AidingMode == AID_NONE) {
            if (tiltAlignComplete && motorsArmed) {//tiltAlignComplete 如果倾斜对准完成则返回true
            // This is a compromise between corrections for gyro errors and reducing effect of manoeuvre accelerations on tilt estimate
            //这是在陀螺误差修正和降低机动加速对倾斜估计影响的折中
                R_OBS[0] = sq(constrain_float(frontend->_noaidHorizNoise, 0.5f, 50.0f));//_noaidHorizNoise 水平位置量测噪声
            } else {
                // Use a smaller value to give faster initial alignment
                //使用较小的值以尽快初始对准
                R_OBS[0] = sq(0.5f);
            }
            R_OBS[1] = R_OBS[0];//R_OBS 量测方差observation
            R_OBS[2] = R_OBS[0];
            R_OBS[3] = R_OBS[0];
            R_OBS[4] = R_OBS[0];
            for (uint8_t i=0; i<=2; i++) R_OBS_DATA_CHECKS[i] = R_OBS[i];
        } else {
            if (gpsSpdAccuracy > 0.0f) {//gpsSpdAccuracy GPS接收机返回的速度精度
                // use GPS receivers reported speed accuracy if available and floor at value set by GPS velocity noise parameter
                //如果可用则使用GPS接收机报告的速度精度
                R_OBS[0] = sq(constrain_float(gpsSpdAccuracy, frontend->_gpsHorizVelNoise, 50.0f));
                R_OBS[2] = sq(constrain_float(gpsSpdAccuracy, frontend->_gpsVertVelNoise, 50.0f));
            } else {
                // calculate additional error in GPS velocity caused by manoeuvring
                //计算由机动造成的GPS速度的额外误差
                R_OBS[0] = sq(constrain_float(frontend->_gpsHorizVelNoise, 0.05f, 5.0f)) + sq(frontend->gpsNEVelVarAccScale * accNavMag);
                R_OBS[2] = sq(constrain_float(frontend->_gpsVertVelNoise,  0.05f, 5.0f)) + sq(frontend->gpsDVelVarAccScale  * accNavMag);
            }
            R_OBS[1] = R_OBS[0];
            // Use GPS reported position accuracy if available and floor at value set by GPS position noise parameter
            //若可用则使用GPS报告的位置精度
            if (gpsPosAccuracy > 0.0f) {//gpsPosAccuracy 由GPS接收机返回的位置估计精度
                R_OBS[3] = sq(constrain_float(gpsPosAccuracy, frontend->_gpsHorizPosNoise, 100.0f));
            } else {
                R_OBS[3] = sq(constrain_float(frontend->_gpsHorizPosNoise, 0.1f, 10.0f)) + sq(posErr);
            }
            R_OBS[4] = R_OBS[3];
            // For data integrity checks we use the same measurement variances as used to calculate the Kalman gains for all measurements except GPS horizontal velocity
            // For horizontal GPs velocity we don't want the acceptance radius to increase with reported GPS accuracy so we use a value based on best GPs perfomrance
            // plus a margin for manoeuvres. It is better to reject GPS horizontal velocity errors early
            //对数据完整性检查，使用相同的量测方程来计算除GPS水平速度外所有量测的滤波增益
            //对于水平GPS速度，我们不想接收半径随GPS接收机报告的精度增加而增加，故采用最好的GPS接收机的值，再加上机动余量的值
            //若GPS水平速度存在误差，最好尽早拒绝使用水平GPS速度
            for (uint8_t i=0; i<=2; i++) R_OBS_DATA_CHECKS[i] = sq(constrain_float(frontend->_gpsHorizVelNoise, 0.05f, 5.0f)) + sq(frontend->gpsNEVelVarAccScale * accNavMag);
        }
        R_OBS[5] = posDownObsNoise;//posDownObsNoise 状态和协方差更新所使用的垂直位置的量测噪声
        for (uint8_t i=3; i<=5; i++) R_OBS_DATA_CHECKS[i] = R_OBS[i];

        // if vertical GPS velocity data and an independent height source is being used, check to see if the GPS vertical velocity and altimeter
        // innovations have the same sign and are outside limits. If so, then it is likely aliasing is affecting
        // the accelerometers and we should disable the GPS and barometer innovation consistency checks.
        //若正使用GPS垂直速度和独立的高度源，那么检查GPS垂直速度和高度更新是否具有相同的符号和是否超出限制
        //若是，有可能影响加计，此时应禁用GPS和高度计更新持续性检查
        if (useGpsVertVel && fuseVelData && (frontend->_altSource != 2)) {
            // calculate innovations for height and vertical GPS vel measurements
            //计算高度和GPS垂直速度的修正量
            float hgtErr  = stateStruct.position.z - velPosObs[5];
            float velDErr = stateStruct.velocity.z - velPosObs[2];
            // check if they are the same sign and both more than 3-sigma out of bounds
            //检测它们是否具有相同的符号，是否都超过3-sigma边界
            if ((hgtErr*velDErr > 0.0f) && (sq(hgtErr) > 9.0f * (P[8][8] + R_OBS_DATA_CHECKS[5])) && (sq(velDErr) > 9.0f * (P[5][5] + R_OBS_DATA_CHECKS[2]))) {
                badIMUdata = true;
            } else {
                badIMUdata = false;
            }
        }
		//what fucking codes

        // calculate innovations and check GPS data validity using an innovation consistency check
        // test position measurements
        //计算修正量，并使用修正量的连续性检查来检测GPS数据的可用性
        if (fusePosData) {
            // test horizontal position measurements 融合水平速度量测
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
                //若超时或超过指定不确定性的半径，则重置GPS
                if (posTimeout || ((P[6][6] + P[7][7]) > sq(float(frontend->_gpsGlitchRadiusMax)))) {
                    // reset the position to the current GPS position
                    //将位置重置为当前的GPS位置
                    ResetPosition();
                    // reset the velocity to the GPS velocity
                    //将速度重置为GPS的速度
                    ResetVelocity();
                    // don't fuse GPS data on this time step
                    //该时刻，不融合GPS数据
                    fusePosData = false;
                    fuseVelData = false;
                    // Reset the position variances and corresponding covariances to a value that will pass the checks
                    //重置位置方差及相应的协方差
                    zeroRows(P,6,7);
                    zeroCols(P,6,7);
					//_gpsGlitchRadiusMax 当GPS无故障时 ins和GPS水平位置允许的最大差异
                    P[6][6] = sq(float(0.5f*frontend->_gpsGlitchRadiusMax));
                    P[7][7] = P[6][6];
                    // Reset the normalised innovation to avoid failing the bad fusion tests
                    //重置归一化后的修正量，以避免陷入病态的融合测试中
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
            //如果用户禁止或正使用综合的数据，则不融合垂直速度量测
            if (frontend->_fusionModeGPS > 0 || PV_AidingMode != AID_ABSOLUTE || frontend->inhibitGpsVertVelUse) {
                imax = 1;
            }
            float innovVelSumSq = 0; // sum of squares of velocity innovations
            float varVelSum = 0; // sum of velocity innovation variances
            for (uint8_t i = 0; i<=imax; i++) {
                // velocity states start at index 3 状态中速度从下标3开始
                stateIndex   = i + 3;
                // calculate innovations using blended and single IMU predicted states
                //使用混合和单个IMU预测状态
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
            velHealth = ((velTestRatio < 1.0f)  || badIMUdata);//若检测到IMU数据存在问题，则置1
            // use velocity data if healthy, timed out, or in constant position mode
            if (velHealth || velTimeout) {
                velHealth = true;
                // restart the timeout count 重启超时计数
                lastVelPassTime_ms = imuSampleTime_ms;
                // If we are doing full aiding and velocity fusion times out, reset to the GPS velocity
                //如果在进行绝对位置参考辅助和速度融合超时，则重置GPS速度
                if (PV_AidingMode == AID_ABSOLUTE && velTimeout) {
                    // reset the velocity to the GPS velocity 将速度重置为GPS的速度
                    ResetVelocity();
                    // don't fuse GPS velocity data on this time step 该时刻不融合GPS速度信息
                    fuseVelData = false;
                    // Reset the normalised innovation to avoid failing the bad fusion tests
                    //重置归一化的修正量，避免陷入错误的融合测试中
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
        //依据可用的数据及其健康情况，设置速度、位置顺序融合的范围
        if (fuseVelData && velHealth) {//fuseVelData 速度量测融合的标志位；velHearth 如果速度量测通过一致性检测则置true
            fuseData[0] = true;
            fuseData[1] = true;
            if (useGpsVertVel) {//若使用GPS垂直速度则置true
                fuseData[2] = true;
            }
			//tiltErrVec 最近的一次来自速度、位置融合的姿态误差修正量
            tiltErrVec.zero();//tiltErrVec 最近的从位置、速度融合得到的姿态误差修正量
        }
		//posHealth 如何位置量测通过新息检测，则置posHealth为true
        if (fusePosData && posHealth) {//fusePosData 位置融合的标志位 
            fuseData[3] = true;
            fuseData[4] = true;
            tiltErrVec.zero();
        }
        if (fuseHgtData && hgtHealth) {
            fuseData[5] = true;
        }

        // fuse measurements sequentially   顺序地融合量测信息 位置速度融合
        for (obsIndex=0; obsIndex<=5; obsIndex++) {
            if (fuseData[obsIndex]) {
                stateIndex = 3 + obsIndex;
                // calculate the measurement innovation, using states from a different time coordinate if fusing height data
                // adjust scaling on GPS measurement noise variances if not enough satellites
                //计算量测更新，如果融合高度数据则使用不同时间坐标系下状态
                //若无足够卫星，则调整GPS量测噪声方差缩放比例
                if (obsIndex <= 2)
                {
                    innovVelPos[obsIndex] = stateStruct.velocity[obsIndex] - velPosObs[obsIndex];
                    R_OBS[obsIndex] *= sq(gpsNoiseScaler);//缩放噪声R阵
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
                //计算滤波增益和一步预测协方差阵  只用对角线元素
                //一步预测P:P = P * H / (H * P * H' + R),由于H阵中相应元素为1，故变成了P = P / (P + R)
                varInnovVelPos[obsIndex] = P[stateIndex][stateIndex] + R_OBS[obsIndex];
                SK = 1.0f/varInnovVelPos[obsIndex];//  /(P + R)
                for (uint8_t i= 0; i<=15; i++) {
                    Kfusion[i] = P[i][stateIndex]*SK;
                }

                // inhibit magnetic field state estimation by setting Kalman gains to zero
                //通过将滤波增益置零来禁止磁场状态估计
                if (!inhibitMagStates) {//如果磁场状态及其协方差恒定不变，则inhibitMagState置true
                    for (uint8_t i = 16; i<=21; i++) {
                        Kfusion[i] = P[i][stateIndex]*SK;
                    }
                } else {//如果磁场状态及其协方差不变，则将其滤波增益置零
                    for (uint8_t i = 16; i<=21; i++) {
                        Kfusion[i] = 0.0f;
                    }
                }

                // inhibit wind state estimation by setting Kalman gains to zero
                //通过将滤波增益置零来禁止风速状态估计
                if (!inhibitWindStates) {//如果风速的状态及其协方差保持不变则将inhibitWindState置true
                    Kfusion[22] = P[22][stateIndex]*SK;
                    Kfusion[23] = P[23][stateIndex]*SK;
                } else {//风速的状态及其协方差保持不变
                    Kfusion[22] = 0.0f;
                    Kfusion[23] = 0.0f;
                }

                // update the covariance - take advantage of direct observation of a single state at index = stateIndex to reduce computations
                // this is a numerically optimised implementation of standard equation P = (I - K*H)*P;
                //更新状态估计的协方差矩阵 - 利用单个状态在index = stateIndex处的直接观察减少计算量
                //这是标准方程P = (I - K * H) * P数值实现的简化
                for (uint8_t i= 0; i<=stateIndexLim; i++) {
                    for (uint8_t j= 0; j<=stateIndexLim; j++)
                    {
                        KHP[i][j] = Kfusion[i] * P[stateIndex][j];
                    }
                }
                // Check that we are not going to drive any variances negative and skip the update if so
                //检查是否存在P阵主对角线为负的情况，若存在则跳过此次更新
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
                    //强制协方差矩阵P对称，并限制协方差大小以防止滤波器病态
                    ForceSymmetry();
                    ConstrainVariances();

                    // update the states
                    // zero the attitude error state - by definition it is assumed to be zero before each observaton fusion
                    //根据定义每次进行量测融合前，将状态向量中姿态误差置零
                    //其实每次滤波后都将状态向量总姿态误差、速度误差、位置误差置零
                    stateStruct.angErr.zero();//拼接输出:因为函数调用返回一个对象或引用，而这里stateStruct是一个结构体，angErr是其中一个对象 类对象直接调用成员函数

                    // calculate state corrections and re-normalise the quaternions for states predicted using the blended IMU data
                    //计算状态修正，并采用混合IMU数据将状态预测中的四元素重新归一化
                    // X = x + k * (z - H * x)  前面是H * x - z
                    for (uint8_t i = 0; i<=stateIndexLim; i++) {
                        statesArray[i] = statesArray[i] - Kfusion[i] * innovVelPos[obsIndex];
                    }

                    // the first 3 states represent the angular misalignment vector. This is
                    // is used to correct the estimated quaternion
                    //前三个状态是姿态误差，用来修正估计的四元素
                    //rotate()的input:姿态误差和隐式传递的当前的四元素；output:修正后的四元素
                    stateStruct.quat.rotate(stateStruct.angErr);

                    // sum the attitude error from velocity and position fusion only
                    // used as a metric for convergence monitoring
                    //将速度和位置融合得到的姿态误差相加，仅用来做收敛检测
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
