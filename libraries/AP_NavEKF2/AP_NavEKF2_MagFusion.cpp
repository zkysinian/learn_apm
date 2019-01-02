#include <AP_HAL/AP_HAL.h>

#if HAL_CPU_CLASS >= HAL_CPU_CLASS_150

#include "AP_NavEKF2.h"
#include "AP_NavEKF2_core.h"
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <GCS_MAVLink/GCS.h>

#include <stdio.h>

extern const AP_HAL::HAL& hal;

/********************************************************
*                   RESET FUNCTIONS                     *
********************************************************/

// Control reset of yaw and magnetic field states
//控制航向和磁场状态的重置
void NavEKF2_core::controlMagYawReset()
{

    // Vehicles that can use a zero sideslip assumption (Planes) are a special case
    // They can use the GPS velocity to recover from bad initial compass data
    // This allows recovery for heading alignment errors due to compass faults
    //车辆是一个特例，因器不采用侧滑假设；
    //可使用GPS速度从错误的磁力计初始数据中恢复
    //这允许从由磁力计错误造成的航向对准误差中恢复
    if (assume_zero_sideslip() && !finalInflightYawInit && inFlight ) {
        gpsYawResetRequest = true;
        return;
    } else {
        gpsYawResetRequest = false;
    }

    // Quaternion and delta rotation vector that are re-used for different calculations
    Vector3f deltaRotVecTemp;
    Quaternion deltaQuatTemp;

    bool flightResetAllowed = false;
    bool initialResetAllowed = false;
    if (!finalInflightYawInit) {
        // Use a quaternion division to calculate the delta quaternion between the rotation at the current and last time
        deltaQuatTemp = stateStruct.quat / prevQuatMagReset;
        prevQuatMagReset = stateStruct.quat;

        // convert the quaternion to a rotation vector and find its length
        deltaQuatTemp.to_axis_angle(deltaRotVecTemp);

        // check if the spin rate is OK - high spin rates can cause angular alignment errors
        bool angRateOK = deltaRotVecTemp.length() < 0.1745f;

        initialResetAllowed = angRateOK;
        flightResetAllowed = angRateOK && !onGround;

    }

    // Check if conditions for a interim or final yaw/mag reset are met
    bool finalResetRequest = false;
    bool interimResetRequest = false;
    if (flightResetAllowed && !assume_zero_sideslip()) {
        // check that we have reached a height where ground magnetic interference effects are insignificant
        // and can perform a final reset of the yaw and field states
        finalResetRequest = (stateStruct.position.z  - posDownAtTakeoff) < -EKF2_MAG_FINAL_RESET_ALT;

        // check for increasing height
        bool hgtIncreasing = (posDownAtLastMagReset-stateStruct.position.z) > 0.5f;
        float yawInnovIncrease = fabsf(innovYaw) - fabsf(yawInnovAtLastMagReset);

        // check for increasing yaw innovations
        bool yawInnovIncreasing = yawInnovIncrease > 0.25f;

        // check that the yaw innovations haven't been caused by a large change in attitude
        deltaQuatTemp = quatAtLastMagReset / stateStruct.quat;
        deltaQuatTemp.to_axis_angle(deltaRotVecTemp);
        bool largeAngleChange = deltaRotVecTemp.length() > yawInnovIncrease;

        // if yaw innovations and height have increased and we haven't rotated much
        // then we are climbing away from a ground based magnetic anomaly and need to reset
        interimResetRequest = hgtIncreasing && yawInnovIncreasing && !largeAngleChange;
    }

    // an initial reset is required if we have not yet aligned the yaw angle
    bool initialResetRequest = initialResetAllowed && !yawAlignComplete;

    // a combined yaw angle and magnetic field reset can be initiated by:
    magYawResetRequest = magYawResetRequest || // an external request
            initialResetRequest || // an initial alignment performed by all vehicle types using magnetometer
            interimResetRequest || // an interim alignment required to recover from ground based magnetic anomaly
            finalResetRequest; // the final reset when we have acheived enough height to be in stable magnetic field environment

    // Perform a reset of magnetic field states and reset yaw to corrected magnetic heading
    if (magYawResetRequest || magStateResetRequest || extNavYawResetRequest) {

        // if a yaw reset has been requested, apply the updated quaternion to the current state
        if (extNavYawResetRequest) {
            // get the euler angles from the current state estimate
            Vector3f eulerAnglesOld;
            stateStruct.quat.to_euler(eulerAnglesOld.x, eulerAnglesOld.y, eulerAnglesOld.z);

            // previous value used to calculate a reset delta
            Quaternion prevQuat = stateStruct.quat;

            // Get the Euler angles from the external vision data
            Vector3f eulerAnglesNew;
            extNavDataDelayed.quat.to_euler(eulerAnglesNew.x, eulerAnglesNew.y, eulerAnglesNew.z);

            // the new quaternion uses the old roll/pitch and new yaw angle
            stateStruct.quat.from_euler(eulerAnglesOld.x, eulerAnglesOld.y, eulerAnglesNew.z);

            // calculate the change in the quaternion state and apply it to the ouput history buffer
            prevQuat = stateStruct.quat/prevQuat;
            StoreQuatRotate(prevQuat);

            // send initial alignment status to console
            if (!yawAlignComplete) {
                gcs().send_text(MAV_SEVERITY_INFO, "EKF2 IMU%u ext nav yaw alignment complete",(unsigned)imu_index);
            }

            // record the reset as complete and also record the in-flight reset as complete to stop further resets when hight is gained
            // in-flight reset is unnecessary because we do not need to consider groudn based magnetic anomaly effects
            yawAlignComplete = true;
            finalInflightYawInit = true;

            // clear the yaw reset request flag
            extNavYawResetRequest = false;

        } else if (magYawResetRequest || magStateResetRequest) {
            // get the euler angles from the current state estimate
            Vector3f eulerAngles;
            stateStruct.quat.to_euler(eulerAngles.x, eulerAngles.y, eulerAngles.z);

            // Use the Euler angles and magnetometer measurement to update the magnetic field states
            // and get an updated quaternion
            Quaternion newQuat = calcQuatAndFieldStates(eulerAngles.x, eulerAngles.y);

            if (magYawResetRequest) {
                // previous value used to calculate a reset delta
                Quaternion prevQuat = stateStruct.quat;

                // update the quaternion states using the new yaw angle
                stateStruct.quat = newQuat;

                // calculate the change in the quaternion state and apply it to the ouput history buffer
                prevQuat = stateStruct.quat/prevQuat;
                StoreQuatRotate(prevQuat);

                // send initial alignment status to console
                if (!yawAlignComplete) {
                    gcs().send_text(MAV_SEVERITY_INFO, "EKF2 IMU%u initial yaw alignment complete",(unsigned)imu_index);
                }

                // send in-flight yaw alignment status to console
                if (finalResetRequest) {
                    gcs().send_text(MAV_SEVERITY_INFO, "EKF2 IMU%u in-flight yaw alignment complete",(unsigned)imu_index);
                } else if (interimResetRequest) {
                    gcs().send_text(MAV_SEVERITY_WARNING, "EKF2 IMU%u ground mag anomaly, yaw re-aligned",(unsigned)imu_index);
                }

                // update the yaw reset completed status
                recordYawReset();

                // clear the yaw reset request flag
                magYawResetRequest = false;

                // clear the complete flags if an interim reset has been performed to allow subsequent
                // and final reset to occur
                if (interimResetRequest) {
                    finalInflightYawInit = false;
                    finalInflightMagInit = false;
                }
            }
        }
    }
}

// this function is used to do a forced re-alignment of the yaw angle to align with the horizontal velocity
// vector from GPS. It is used to align the yaw angle after launch or takeoff.
//该函数用GPS航线重置航向
void NavEKF2_core::realignYawGPS()
{
    if ((sq(gpsDataDelayed.vel.x) + sq(gpsDataDelayed.vel.y)) > 25.0f) {
        // get quaternion from existing filter states and calculate roll, pitch and yaw angles
        //quat.to_euler 利用四元素得到欧拉角，实参列表为函数计算的欧拉角传递出来，quat是隐式传递
        Vector3f eulerAngles;
        stateStruct.quat.to_euler(eulerAngles.x, eulerAngles.y, eulerAngles.z);

        // calculate course yaw angle    course angle 航向角、行进方向角
        //atan2f范围[-pi pi]，用atan2f代替atan
        float velYaw = atan2f(stateStruct.velocity.y,stateStruct.velocity.x);

        // calculate course yaw angle from GPS velocity 计算由GPS速度得到的航向角
        float gpsYaw = atan2f(gpsDataDelayed.vel.y,gpsDataDelayed.vel.x);

        // Check the yaw angles for consistency
        float yawErr = MAX(fabsf(wrap_PI(gpsYaw - velYaw)),fabsf(wrap_PI(gpsYaw - eulerAngles.z)));

        // If the angles disagree by more than 45 degrees and GPS innovations are large or no previous yaw alignment, we declare the magnetic yaw as bad
        //如果角度偏移超过45°且GPS航向新息很大或先前无磁航向对齐，则断定磁航向质量不好
        //velTestRatio GPS速度平方和除以失效阈值
		badMagYaw = ((yawErr > 0.7854f) && (velTestRatio > 1.0f) && (PV_AidingMode == AID_ABSOLUTE)) || !yawAlignComplete;

        // correct yaw angle using GPS ground course if compass yaw bad
        //若磁航向错误，则用GPS水平速度计算的航向修正航向角
        if (badMagYaw) {//badMagYaw 若磁力计不可用，则置true

            // calculate new filter quaternion states from Euler angles
            stateStruct.quat.from_euler(eulerAngles.x, eulerAngles.y, gpsYaw);
            // reset the velocity and position states as they will be inaccurate due to bad yaw
            //若磁航向出错，位置速度将会不准确，此时进行重置速度、位置状态
            ResetVelocity();
            ResetPosition();

            // send yaw alignment information to console
            gcs().send_text(MAV_SEVERITY_INFO, "EKF2 IMU%u yaw aligned to GPS velocity",(unsigned)imu_index);

            // zero the attitude covariances becasue the corelations will now be invalid
            //将姿态的方差置零，因为它们之间关联是不可用的
            //未理解?
            zeroAttCovOnly();

            // record the yaw reset event
            recordYawReset();

            // clear all pending yaw reset requests
            //gpsYawResetRequest 若载体航向角需用GPS航线重置，则置true
            //magYawResetRequest 若载体航向和磁场状态需用mag量测重置，则置true
            gpsYawResetRequest = false;
            magYawResetRequest = false;

            if (use_compass()) {
                // request a mag field reset which may enable us to use the magnetoemter if the previous fault was due to bad initialisation
                //magStateResetRequest 若磁场状态需用磁力计量测重置，则置true
                magStateResetRequest = true;
                // clear the all sensors failed status so that the magnetometers sensors get a second chance now that we are flying
                //allMagSensorSFailed 若磁力计飞行中超时且不需再用磁力计，则置true
                allMagSensorsFailed = false;
            }
        }
    }
}

/********************************************************
*                   FUSE MEASURED_DATA                  *
********************************************************/

// select fusion of magnetometer data
//别人为什么这么写?你会怎么写?你从这学到什么?怎么应用到你之前或之后的代码中?
//进行mag融合、状态模型和量测模型各是什么?有什么优缺点?我学到什么?我会怎么写?差距?如何优化?
void NavEKF2_core::SelectMagFusion()
{
    // start performance timer
    hal.util->perf_begin(_perf_FuseMagnetometer);

    // clear the flag that lets other processes know that the expensive magnetometer fusion operation has been perfomred on that time step
    // used for load levelling
    //清除标志位，让其他进程知道磁融合开始
    magFusePerformed = false;

    // check for and read new magnetometer measurements
    readMagData();

    // If we are using the compass and the magnetometer has been unhealthy for too long we declare a timeout
    //若在用磁罗盘且磁罗盘长时间自检不过，则断定磁罗盘超时
    if (magHealth) {//若磁力计一致性检查通过，则magHealth为1
        magTimeout = false;
        lastHealthyMagTime_ms = imuSampleTime_ms;
    } else if ((imuSampleTime_ms - lastHealthyMagTime_ms) > frontend->magFailTimeLimit_ms && use_compass()) {
        magTimeout = true;
    }

    // check for availability of magnetometer data to fuse
    //融合之前检查磁数据是否可用 
    //magDataToFuse 若磁数据可用且来到，置为true
    magDataToFuse = storedMag.recall(magDataDelayed,imuDataDelayed.time_ms);

    // Control reset of yaw and magnetic field states if we are using compass data
    //航向和磁状态的重置
    if (magDataToFuse && use_compass()) {
        controlMagYawReset();
    }

    // determine if conditions are right to start a new fusion cycle
    // wait until the EKF time horizon catches up with the measurement
    //确定条件是否合适开始新的融合周期
    //等待，直到EKF时间赶上量测更新的时间
    //stateInitialised 若滤波器初始化完成则返回true；yawAlignComplete 当yaw alignment 完成，返回true
    bool dataReady = (magDataToFuse && statesInitialised && use_compass() && yawAlignComplete);
    if (dataReady) {
        // use the simple method of declination to maintain heading if we cannot use the magnetic field states
        //如果不使用磁场状态，使用简单的方法去维持航向
        if(inhibitMagStates || magStateResetRequest || !magStateInitComplete) {
            fuseEulerYaw();
            // zero the test ratio output from the inactive 3-axis magnetometer fusion
            magTestRatio.zero();
        } else {
            // if we are not doing aiding with earth relative observations (eg GPS) then the declination is
            // maintained by fusing declination as a synthesised observation
            //如果没有像GPS之类的地球相对观测量，则偏差通过融合偏差作为一个综合的量测来维持
            if (PV_AidingMode != AID_ABSOLUTE) {
                FuseDeclination(0.34f);
            }
            // fuse the three magnetometer componenents sequentially
            //按顺序融合三轴磁力计数据
            for (mag_state.obsIndex = 0; mag_state.obsIndex <= 2; mag_state.obsIndex++) {
                hal.util->perf_begin(_perf_test[0]);
                FuseMagnetometer();
                hal.util->perf_end(_perf_test[0]);
                // don't continue fusion if unhealthy 若自检未通过，则不进行融合
                if (!magHealth) {
                    break;
                }
            }
            // zero the test ratio output from the inactive simple magnetometer yaw fusion
            yawTestRatio = 0.0f;
        }
    }

    // If we have no magnetometer and are on the ground, fuse in a synthetic heading measurement to prevent the
    // filter covariances from becoming badly conditioned
    if (!use_compass()) {
        if (onGround && (imuSampleTime_ms - lastYawTime_ms > 1000)) {
            fuseEulerYaw();
            magTestRatio.zero();
            yawTestRatio = 0.0f;
        }
    }

    // If the final yaw reset has been performed and the state variances are sufficiently low
    // record that the earth field has been learned.
    if (!magFieldLearned && finalInflightMagInit) {
        magFieldLearned = (P[16][16] < sq(0.01f)) && (P[17][17] < sq(0.01f)) && (P[18][18] < sq(0.01f));
    }

    // record the last learned field variances
    if (magFieldLearned && !inhibitMagStates) {
        earthMagFieldVar.x = P[16][16];
        earthMagFieldVar.y = P[17][17];
        earthMagFieldVar.z = P[18][18];
        bodyMagFieldVar.x = P[19][19];
        bodyMagFieldVar.y = P[20][20];
        bodyMagFieldVar.z = P[21][21];
    }

    // stop performance timer
    hal.util->perf_end(_perf_FuseMagnetometer);
}

/*
 * Fuse magnetometer measurements using explicit algebraic equations generated with Matlab symbolic toolbox.
 * The script file used to generate these and other equations in this filter can be found here:
 * https://github.com/priseborough/InertialNav/blob/master/derivations/RotationVectorAttitudeParameterisation/GenerateNavFilterEquations.m
*/
void NavEKF2_core::FuseMagnetometer()
{
    hal.util->perf_begin(_perf_test[1]);
    
    // declarations 
    //引用，将全局的变量赋值给本地的变量，直接采用引用的形式，减少了赋值操作
    ftype &q0 = mag_state.q0;//typedef float ftype
    ftype &q1 = mag_state.q1;
    ftype &q2 = mag_state.q2;
    ftype &q3 = mag_state.q3;
    ftype &magN = mag_state.magN;
    ftype &magE = mag_state.magE;
    ftype &magD = mag_state.magD;
    ftype &magXbias = mag_state.magXbias;
    ftype &magYbias = mag_state.magYbias;
    ftype &magZbias = mag_state.magZbias;
    uint8_t &obsIndex = mag_state.obsIndex;
    Matrix3f &DCM = mag_state.DCM;
    Vector3f &MagPred = mag_state.MagPred;
    ftype &R_MAG = mag_state.R_MAG;
    ftype *SH_MAG = &mag_state.SH_MAG[0];
    Vector24 H_MAG;
    Vector6 SK_MX;
    Vector6 SK_MY;
    Vector6 SK_MZ;

    hal.util->perf_end(_perf_test[1]);
    
    // perform sequential fusion of magnetometer measurements.
    // this assumes that the errors in the different components are
    // uncorrelated which is not true, however in the absence of covariance
    // data fit is the only assumption we can make
    // so we might as well take advantage of the computational efficiencies
    // associated with sequential fusion
    // calculate observation jacobians and Kalman gains
    //执行磁力计测量信息的顺序融合
    //假设不同传感器中的误差是不相关的，这中假设不正确。但缺少协方差时，数据拟合是我们可以做的唯一假设
    //因此我们也可以利用与顺序拟合相关的计算效率，计算量测的Jacobian矩阵和滤波增益
    if (obsIndex == 0)
    {

        hal.util->perf_begin(_perf_test[2]);

        // copy required states to local variable names
        //将全局变量赋值给本地变量 为什么这么做?
        q0       = stateStruct.quat[0];
        q1       = stateStruct.quat[1];
        q2       = stateStruct.quat[2];
        q3       = stateStruct.quat[3];
        magN     = stateStruct.earth_magfield[0];
        magE     = stateStruct.earth_magfield[1];
        magD     = stateStruct.earth_magfield[2];
        magXbias = stateStruct.body_magfield[0];
        magYbias = stateStruct.body_magfield[1];
        magZbias = stateStruct.body_magfield[2];

        // rotate predicted earth components into body axes and calculate
        // predicted measurements
        //将地理系下磁数据测量转换到本体系下，并计算预测的量测量
        DCM[0][0] = q0*q0 + q1*q1 - q2*q2 - q3*q3;
        DCM[0][1] = 2.0f*(q1*q2 + q0*q3);
        DCM[0][2] = 2.0f*(q1*q3-q0*q2);
        DCM[1][0] = 2.0f*(q1*q2 - q0*q3);
        DCM[1][1] = q0*q0 - q1*q1 + q2*q2 - q3*q3;
        DCM[1][2] = 2.0f*(q2*q3 + q0*q1);
        DCM[2][0] = 2.0f*(q1*q3 + q0*q2);
        DCM[2][1] = 2.0f*(q2*q3 - q0*q1);
        DCM[2][2] = q0*q0 - q1*q1 - q2*q2 + q3*q3;
        MagPred[0] = DCM[0][0]*magN + DCM[0][1]*magE  + DCM[0][2]*magD + magXbias;
        MagPred[1] = DCM[1][0]*magN + DCM[1][1]*magE  + DCM[1][2]*magD + magYbias;
        MagPred[2] = DCM[2][0]*magN + DCM[2][1]*magE  + DCM[2][2]*magD + magZbias;

        // calculate the measurement innovation for each axis
        //计算三轴的量测更新 Z-HX
        for (uint8_t i = 0; i<=2; i++) {
            innovMag[i] = MagPred[i] - magDataDelayed.mag[i];
        }

        // scale magnetometer observation error with total angular rate to allow for timing errors
        //与总的角速率相比，衡量磁力计的观测误差，以允许时间误差
        R_MAG = sq(constrain_float(frontend->_magNoise, 0.01f, 0.5f)) + sq(frontend->magVarRateScale*delAngCorrected.length() / imuDataDelayed.delAngDT);

        // calculate common expressions used to calculate observation jacobians an innovation variance for each component
        //用于计算量测的Jacobian矩阵的常用表达式
        SH_MAG[0] = sq(q0) - sq(q1) + sq(q2) - sq(q3);
        SH_MAG[1] = sq(q0) + sq(q1) - sq(q2) - sq(q3);
        SH_MAG[2] = sq(q0) - sq(q1) - sq(q2) + sq(q3);
        SH_MAG[3] = 2.0f*q0*q1 + 2.0f*q2*q3;
        SH_MAG[4] = 2.0f*q0*q3 + 2.0f*q1*q2;
        SH_MAG[5] = 2.0f*q0*q2 + 2.0f*q1*q3;
        SH_MAG[6] = magE*(2.0f*q0*q1 - 2.0f*q2*q3);
        SH_MAG[7] = 2.0f*q1*q3 - 2.0f*q0*q2;
        SH_MAG[8] = 2.0f*q0*q3;

        // Calculate the innovation variance for each axis 计算每个轴的一步预测的协方差
        //为什么三个轴分开计算
        // X axis
        varInnovMag[0] = (P[19][19] + R_MAG - P[1][19]*(magD*SH_MAG[2] - SH_MAG[6] + magN*SH_MAG[5]) + P[16][19]*SH_MAG[1] + P[17][19]*SH_MAG[4] + P[18][19]*SH_MAG[7] + P[2][19]*(magE*SH_MAG[0] + magD*SH_MAG[3] - magN*(SH_MAG[8] - 2.0f*q1*q2)) - (magD*SH_MAG[2] - SH_MAG[6] + magN*SH_MAG[5])*(P[19][1] - P[1][1]*(magD*SH_MAG[2] - SH_MAG[6] + magN*SH_MAG[5]) + P[16][1]*SH_MAG[1] + P[17][1]*SH_MAG[4] + P[18][1]*SH_MAG[7] + P[2][1]*(magE*SH_MAG[0] + magD*SH_MAG[3] - magN*(SH_MAG[8] - 2.0f*q1*q2))) + SH_MAG[1]*(P[19][16] - P[1][16]*(magD*SH_MAG[2] - SH_MAG[6] + magN*SH_MAG[5]) + P[16][16]*SH_MAG[1] + P[17][16]*SH_MAG[4] + P[18][16]*SH_MAG[7] + P[2][16]*(magE*SH_MAG[0] + magD*SH_MAG[3] - magN*(SH_MAG[8] - 2.0f*q1*q2))) + SH_MAG[4]*(P[19][17] - P[1][17]*(magD*SH_MAG[2] - SH_MAG[6] + magN*SH_MAG[5]) + P[16][17]*SH_MAG[1] + P[17][17]*SH_MAG[4] + P[18][17]*SH_MAG[7] + P[2][17]*(magE*SH_MAG[0] + magD*SH_MAG[3] - magN*(SH_MAG[8] - 2.0f*q1*q2))) + SH_MAG[7]*(P[19][18] - P[1][18]*(magD*SH_MAG[2] - SH_MAG[6] + magN*SH_MAG[5]) + P[16][18]*SH_MAG[1] + P[17][18]*SH_MAG[4] + P[18][18]*SH_MAG[7] + P[2][18]*(magE*SH_MAG[0] + magD*SH_MAG[3] - magN*(SH_MAG[8] - 2.0f*q1*q2))) + (magE*SH_MAG[0] + magD*SH_MAG[3] - magN*(SH_MAG[8] - 2.0f*q1*q2))*(P[19][2] - P[1][2]*(magD*SH_MAG[2] - SH_MAG[6] + magN*SH_MAG[5]) + P[16][2]*SH_MAG[1] + P[17][2]*SH_MAG[4] + P[18][2]*SH_MAG[7] + P[2][2]*(magE*SH_MAG[0] + magD*SH_MAG[3] - magN*(SH_MAG[8] - 2.0f*q1*q2))));
        if (varInnovMag[0] >= R_MAG) {
            faultStatus.bad_xmag = false;
        } else {
            // the calculation is badly conditioned, so we cannot perform fusion on this step
            // we reset the covariance matrix and try again next measurement
            //如果计算出现病态，则不进行该次融合。重置协方差阵并尝试使用下次量测进行融合
            CovarianceInit();
            obsIndex = 1;
            faultStatus.bad_xmag = true;

            hal.util->perf_end(_perf_test[2]);

            return;
        }

        // Y axis
        varInnovMag[1] = (P[20][20] + R_MAG + P[0][20]*(magD*SH_MAG[2] - SH_MAG[6] + magN*SH_MAG[5]) + P[17][20]*SH_MAG[0] + P[18][20]*SH_MAG[3] - (SH_MAG[8] - 2.0f*q1*q2)*(P[20][16] + P[0][16]*(magD*SH_MAG[2] - SH_MAG[6] + magN*SH_MAG[5]) + P[17][16]*SH_MAG[0] + P[18][16]*SH_MAG[3] - P[2][16]*(magE*SH_MAG[4] + magD*SH_MAG[7] + magN*SH_MAG[1]) - P[16][16]*(SH_MAG[8] - 2.0f*q1*q2)) - P[2][20]*(magE*SH_MAG[4] + magD*SH_MAG[7] + magN*SH_MAG[1]) + (magD*SH_MAG[2] - SH_MAG[6] + magN*SH_MAG[5])*(P[20][0] + P[0][0]*(magD*SH_MAG[2] - SH_MAG[6] + magN*SH_MAG[5]) + P[17][0]*SH_MAG[0] + P[18][0]*SH_MAG[3] - P[2][0]*(magE*SH_MAG[4] + magD*SH_MAG[7] + magN*SH_MAG[1]) - P[16][0]*(SH_MAG[8] - 2.0f*q1*q2)) + SH_MAG[0]*(P[20][17] + P[0][17]*(magD*SH_MAG[2] - SH_MAG[6] + magN*SH_MAG[5]) + P[17][17]*SH_MAG[0] + P[18][17]*SH_MAG[3] - P[2][17]*(magE*SH_MAG[4] + magD*SH_MAG[7] + magN*SH_MAG[1]) - P[16][17]*(SH_MAG[8] - 2.0f*q1*q2)) + SH_MAG[3]*(P[20][18] + P[0][18]*(magD*SH_MAG[2] - SH_MAG[6] + magN*SH_MAG[5]) + P[17][18]*SH_MAG[0] + P[18][18]*SH_MAG[3] - P[2][18]*(magE*SH_MAG[4] + magD*SH_MAG[7] + magN*SH_MAG[1]) - P[16][18]*(SH_MAG[8] - 2.0f*q1*q2)) - P[16][20]*(SH_MAG[8] - 2.0f*q1*q2) - (magE*SH_MAG[4] + magD*SH_MAG[7] + magN*SH_MAG[1])*(P[20][2] + P[0][2]*(magD*SH_MAG[2] - SH_MAG[6] + magN*SH_MAG[5]) + P[17][2]*SH_MAG[0] + P[18][2]*SH_MAG[3] - P[2][2]*(magE*SH_MAG[4] + magD*SH_MAG[7] + magN*SH_MAG[1]) - P[16][2]*(SH_MAG[8] - 2.0f*q1*q2)));
        if (varInnovMag[1] >= R_MAG) {
            faultStatus.bad_ymag = false;
        } else {
            // the calculation is badly conditioned, so we cannot perform fusion on this step
            // we reset the covariance matrix and try again next measurement
            CovarianceInit();
            obsIndex = 2;
            faultStatus.bad_ymag = true;

            hal.util->perf_end(_perf_test[2]);

            return;
        }

        // Z axis
        varInnovMag[2] = (P[21][21] + R_MAG + P[16][21]*SH_MAG[5] + P[18][21]*SH_MAG[2] - (2.0f*q0*q1 - 2.0f*q2*q3)*(P[21][17] + P[16][17]*SH_MAG[5] + P[18][17]*SH_MAG[2] - P[0][17]*(magE*SH_MAG[0] + magD*SH_MAG[3] - magN*(SH_MAG[8] - 2.0f*q1*q2)) + P[1][17]*(magE*SH_MAG[4] + magD*SH_MAG[7] + magN*SH_MAG[1]) - P[17][17]*(2.0f*q0*q1 - 2.0f*q2*q3)) - P[0][21]*(magE*SH_MAG[0] + magD*SH_MAG[3] - magN*(SH_MAG[8] - 2.0f*q1*q2)) + P[1][21]*(magE*SH_MAG[4] + magD*SH_MAG[7] + magN*SH_MAG[1]) + SH_MAG[5]*(P[21][16] + P[16][16]*SH_MAG[5] + P[18][16]*SH_MAG[2] - P[0][16]*(magE*SH_MAG[0] + magD*SH_MAG[3] - magN*(SH_MAG[8] - 2.0f*q1*q2)) + P[1][16]*(magE*SH_MAG[4] + magD*SH_MAG[7] + magN*SH_MAG[1]) - P[17][16]*(2.0f*q0*q1 - 2.0f*q2*q3)) + SH_MAG[2]*(P[21][18] + P[16][18]*SH_MAG[5] + P[18][18]*SH_MAG[2] - P[0][18]*(magE*SH_MAG[0] + magD*SH_MAG[3] - magN*(SH_MAG[8] - 2.0f*q1*q2)) + P[1][18]*(magE*SH_MAG[4] + magD*SH_MAG[7] + magN*SH_MAG[1]) - P[17][18]*(2.0f*q0*q1 - 2.0f*q2*q3)) - (magE*SH_MAG[0] + magD*SH_MAG[3] - magN*(SH_MAG[8] - 2.0f*q1*q2))*(P[21][0] + P[16][0]*SH_MAG[5] + P[18][0]*SH_MAG[2] - P[0][0]*(magE*SH_MAG[0] + magD*SH_MAG[3] - magN*(SH_MAG[8] - 2.0f*q1*q2)) + P[1][0]*(magE*SH_MAG[4] + magD*SH_MAG[7] + magN*SH_MAG[1]) - P[17][0]*(2.0f*q0*q1 - 2.0f*q2*q3)) - P[17][21]*(2.0f*q0*q1 - 2.0f*q2*q3) + (magE*SH_MAG[4] + magD*SH_MAG[7] + magN*SH_MAG[1])*(P[21][1] + P[16][1]*SH_MAG[5] + P[18][1]*SH_MAG[2] - P[0][1]*(magE*SH_MAG[0] + magD*SH_MAG[3] - magN*(SH_MAG[8] - 2.0f*q1*q2)) + P[1][1]*(magE*SH_MAG[4] + magD*SH_MAG[7] + magN*SH_MAG[1]) - P[17][1]*(2.0f*q0*q1 - 2.0f*q2*q3)));
        if (varInnovMag[2] >= R_MAG) {
            faultStatus.bad_zmag = false;
        } else {
            // the calculation is badly conditioned, so we cannot perform fusion on this step
            // we reset the covariance matrix and try again next measurement
            CovarianceInit();
            obsIndex = 3;
            faultStatus.bad_zmag = true;

            hal.util->perf_end(_perf_test[2]);

            return;
        }

        // calculate the innovation test ratios
        for (uint8_t i = 0; i<=2; i++) {
            magTestRatio[i] = sq(innovMag[i]) / (sq(MAX(0.01f * (float)frontend->_magInnovGate, 1.0f)) * varInnovMag[i]);
        }

        // check the last values from all components and set magnetometer health accordingly
        magHealth = (magTestRatio[0] < 1.0f && magTestRatio[1] < 1.0f && magTestRatio[2] < 1.0f);

        // if the magnetometer is unhealthy, do not proceed further
        if (!magHealth) {
            hal.util->perf_end(_perf_test[2]);
            return;
        }

        for (uint8_t i = 0; i<=stateIndexLim; i++) H_MAG[i] = 0.0f;
        H_MAG[1] = SH_MAG[6] - magD*SH_MAG[2] - magN*SH_MAG[5];
        H_MAG[2] = magE*SH_MAG[0] + magD*SH_MAG[3] - magN*(SH_MAG[8] - 2.0f*q1*q2);
        H_MAG[16] = SH_MAG[1];
        H_MAG[17] = SH_MAG[4];
        H_MAG[18] = SH_MAG[7];
        H_MAG[19] = 1.0f;

        // calculate Kalman gain
        //计算滤波增益，为什么就16维状态变量???
        SK_MX[0] = 1.0f / varInnovMag[0];
        SK_MX[1] = magE*SH_MAG[0] + magD*SH_MAG[3] - magN*(SH_MAG[8] - 2.0f*q1*q2);
        SK_MX[2] = magD*SH_MAG[2] - SH_MAG[6] + magN*SH_MAG[5];
        SK_MX[3] = SH_MAG[7];
        Kfusion[0] = SK_MX[0]*(P[0][19] + P[0][16]*SH_MAG[1] + P[0][17]*SH_MAG[4] - P[0][1]*SK_MX[2] + P[0][2]*SK_MX[1] + P[0][18]*SK_MX[3]);
        Kfusion[1] = SK_MX[0]*(P[1][19] + P[1][16]*SH_MAG[1] + P[1][17]*SH_MAG[4] - P[1][1]*SK_MX[2] + P[1][2]*SK_MX[1] + P[1][18]*SK_MX[3]);
        Kfusion[2] = SK_MX[0]*(P[2][19] + P[2][16]*SH_MAG[1] + P[2][17]*SH_MAG[4] - P[2][1]*SK_MX[2] + P[2][2]*SK_MX[1] + P[2][18]*SK_MX[3]);
        Kfusion[3] = SK_MX[0]*(P[3][19] + P[3][16]*SH_MAG[1] + P[3][17]*SH_MAG[4] - P[3][1]*SK_MX[2] + P[3][2]*SK_MX[1] + P[3][18]*SK_MX[3]);
        Kfusion[4] = SK_MX[0]*(P[4][19] + P[4][16]*SH_MAG[1] + P[4][17]*SH_MAG[4] - P[4][1]*SK_MX[2] + P[4][2]*SK_MX[1] + P[4][18]*SK_MX[3]);
        Kfusion[5] = SK_MX[0]*(P[5][19] + P[5][16]*SH_MAG[1] + P[5][17]*SH_MAG[4] - P[5][1]*SK_MX[2] + P[5][2]*SK_MX[1] + P[5][18]*SK_MX[3]);
        Kfusion[6] = SK_MX[0]*(P[6][19] + P[6][16]*SH_MAG[1] + P[6][17]*SH_MAG[4] - P[6][1]*SK_MX[2] + P[6][2]*SK_MX[1] + P[6][18]*SK_MX[3]);
        Kfusion[7] = SK_MX[0]*(P[7][19] + P[7][16]*SH_MAG[1] + P[7][17]*SH_MAG[4] - P[7][1]*SK_MX[2] + P[7][2]*SK_MX[1] + P[7][18]*SK_MX[3]);
        Kfusion[8] = SK_MX[0]*(P[8][19] + P[8][16]*SH_MAG[1] + P[8][17]*SH_MAG[4] - P[8][1]*SK_MX[2] + P[8][2]*SK_MX[1] + P[8][18]*SK_MX[3]);
        Kfusion[9] = SK_MX[0]*(P[9][19] + P[9][16]*SH_MAG[1] + P[9][17]*SH_MAG[4] - P[9][1]*SK_MX[2] + P[9][2]*SK_MX[1] + P[9][18]*SK_MX[3]);
        Kfusion[10] = SK_MX[0]*(P[10][19] + P[10][16]*SH_MAG[1] + P[10][17]*SH_MAG[4] - P[10][1]*SK_MX[2] + P[10][2]*SK_MX[1] + P[10][18]*SK_MX[3]);
        Kfusion[11] = SK_MX[0]*(P[11][19] + P[11][16]*SH_MAG[1] + P[11][17]*SH_MAG[4] - P[11][1]*SK_MX[2] + P[11][2]*SK_MX[1] + P[11][18]*SK_MX[3]);
        Kfusion[12] = SK_MX[0]*(P[12][19] + P[12][16]*SH_MAG[1] + P[12][17]*SH_MAG[4] - P[12][1]*SK_MX[2] + P[12][2]*SK_MX[1] + P[12][18]*SK_MX[3]);
        Kfusion[13] = SK_MX[0]*(P[13][19] + P[13][16]*SH_MAG[1] + P[13][17]*SH_MAG[4] - P[13][1]*SK_MX[2] + P[13][2]*SK_MX[1] + P[13][18]*SK_MX[3]);
        Kfusion[14] = SK_MX[0]*(P[14][19] + P[14][16]*SH_MAG[1] + P[14][17]*SH_MAG[4] - P[14][1]*SK_MX[2] + P[14][2]*SK_MX[1] + P[14][18]*SK_MX[3]);
        Kfusion[15] = SK_MX[0]*(P[15][19] + P[15][16]*SH_MAG[1] + P[15][17]*SH_MAG[4] - P[15][1]*SK_MX[2] + P[15][2]*SK_MX[1] + P[15][18]*SK_MX[3]);
        // end perf block

        // zero Kalman gains to inhibit wind state estimation
        //为禁止风速状态估计，将滤波增益置零
        if (!inhibitWindStates) {
            Kfusion[22] = SK_MX[0]*(P[22][19] + P[22][16]*SH_MAG[1] + P[22][17]*SH_MAG[4] - P[22][1]*SK_MX[2] + P[22][2]*SK_MX[1] + P[22][18]*SK_MX[3]);
            Kfusion[23] = SK_MX[0]*(P[23][19] + P[23][16]*SH_MAG[1] + P[23][17]*SH_MAG[4] - P[23][1]*SK_MX[2] + P[23][2]*SK_MX[1] + P[23][18]*SK_MX[3]);
        } else {
            Kfusion[22] = 0.0f;
            Kfusion[23] = 0.0f;
        }
        // zero Kalman gains to inhibit magnetic field state estimation
        //为禁止磁强度状态估计，将滤波增益置零
        if (!inhibitMagStates) {
            Kfusion[16] = SK_MX[0]*(P[16][19] + P[16][16]*SH_MAG[1] + P[16][17]*SH_MAG[4] - P[16][1]*SK_MX[2] + P[16][2]*SK_MX[1] + P[16][18]*SK_MX[3]);
            Kfusion[17] = SK_MX[0]*(P[17][19] + P[17][16]*SH_MAG[1] + P[17][17]*SH_MAG[4] - P[17][1]*SK_MX[2] + P[17][2]*SK_MX[1] + P[17][18]*SK_MX[3]);
            Kfusion[18] = SK_MX[0]*(P[18][19] + P[18][16]*SH_MAG[1] + P[18][17]*SH_MAG[4] - P[18][1]*SK_MX[2] + P[18][2]*SK_MX[1] + P[18][18]*SK_MX[3]);
            Kfusion[19] = SK_MX[0]*(P[19][19] + P[19][16]*SH_MAG[1] + P[19][17]*SH_MAG[4] - P[19][1]*SK_MX[2] + P[19][2]*SK_MX[1] + P[19][18]*SK_MX[3]);
            Kfusion[20] = SK_MX[0]*(P[20][19] + P[20][16]*SH_MAG[1] + P[20][17]*SH_MAG[4] - P[20][1]*SK_MX[2] + P[20][2]*SK_MX[1] + P[20][18]*SK_MX[3]);
            Kfusion[21] = SK_MX[0]*(P[21][19] + P[21][16]*SH_MAG[1] + P[21][17]*SH_MAG[4] - P[21][1]*SK_MX[2] + P[21][2]*SK_MX[1] + P[21][18]*SK_MX[3]);
        } else {
            for (uint8_t i=16; i<=21; i++) {
                Kfusion[i] = 0.0f;
            }
        }

        // reset the observation index to 0 (we start by fusing the X measurement)
        obsIndex = 0;

        // set flags to indicate to other processes that fusion has been performed and is required on the next frame
        // this can be used by other fusion processes to avoid fusing on the same frame as this expensive step
        magFusePerformed = true;
        magFuseRequired = true;

        hal.util->perf_end(_perf_test[2]);

    }
    else if (obsIndex == 1) // we are now fusing the Y measurement
    {

        hal.util->perf_begin(_perf_test[3]);

        // calculate observation jacobians
        for (uint8_t i = 0; i<=stateIndexLim; i++) H_MAG[i] = 0.0f;
        H_MAG[0] = magD*SH_MAG[2] - SH_MAG[6] + magN*SH_MAG[5];
        H_MAG[2] = - magE*SH_MAG[4] - magD*SH_MAG[7] - magN*SH_MAG[1];
        H_MAG[16] = 2.0f*q1*q2 - SH_MAG[8];
        H_MAG[17] = SH_MAG[0];
        H_MAG[18] = SH_MAG[3];
        H_MAG[20] = 1.0f;

        // calculate Kalman gain
        SK_MY[0] = 1.0f / varInnovMag[1];
        SK_MY[1] = magE*SH_MAG[4] + magD*SH_MAG[7] + magN*SH_MAG[1];
        SK_MY[2] = magD*SH_MAG[2] - SH_MAG[6] + magN*SH_MAG[5];
        SK_MY[3] = SH_MAG[8] - 2.0f*q1*q2;
        Kfusion[0] = SK_MY[0]*(P[0][20] + P[0][17]*SH_MAG[0] + P[0][18]*SH_MAG[3] + P[0][0]*SK_MY[2] - P[0][2]*SK_MY[1] - P[0][16]*SK_MY[3]);
        Kfusion[1] = SK_MY[0]*(P[1][20] + P[1][17]*SH_MAG[0] + P[1][18]*SH_MAG[3] + P[1][0]*SK_MY[2] - P[1][2]*SK_MY[1] - P[1][16]*SK_MY[3]);
        Kfusion[2] = SK_MY[0]*(P[2][20] + P[2][17]*SH_MAG[0] + P[2][18]*SH_MAG[3] + P[2][0]*SK_MY[2] - P[2][2]*SK_MY[1] - P[2][16]*SK_MY[3]);
        Kfusion[3] = SK_MY[0]*(P[3][20] + P[3][17]*SH_MAG[0] + P[3][18]*SH_MAG[3] + P[3][0]*SK_MY[2] - P[3][2]*SK_MY[1] - P[3][16]*SK_MY[3]);
        Kfusion[4] = SK_MY[0]*(P[4][20] + P[4][17]*SH_MAG[0] + P[4][18]*SH_MAG[3] + P[4][0]*SK_MY[2] - P[4][2]*SK_MY[1] - P[4][16]*SK_MY[3]);
        Kfusion[5] = SK_MY[0]*(P[5][20] + P[5][17]*SH_MAG[0] + P[5][18]*SH_MAG[3] + P[5][0]*SK_MY[2] - P[5][2]*SK_MY[1] - P[5][16]*SK_MY[3]);
        Kfusion[6] = SK_MY[0]*(P[6][20] + P[6][17]*SH_MAG[0] + P[6][18]*SH_MAG[3] + P[6][0]*SK_MY[2] - P[6][2]*SK_MY[1] - P[6][16]*SK_MY[3]);
        Kfusion[7] = SK_MY[0]*(P[7][20] + P[7][17]*SH_MAG[0] + P[7][18]*SH_MAG[3] + P[7][0]*SK_MY[2] - P[7][2]*SK_MY[1] - P[7][16]*SK_MY[3]);
        Kfusion[8] = SK_MY[0]*(P[8][20] + P[8][17]*SH_MAG[0] + P[8][18]*SH_MAG[3] + P[8][0]*SK_MY[2] - P[8][2]*SK_MY[1] - P[8][16]*SK_MY[3]);
        Kfusion[9] = SK_MY[0]*(P[9][20] + P[9][17]*SH_MAG[0] + P[9][18]*SH_MAG[3] + P[9][0]*SK_MY[2] - P[9][2]*SK_MY[1] - P[9][16]*SK_MY[3]);
        Kfusion[10] = SK_MY[0]*(P[10][20] + P[10][17]*SH_MAG[0] + P[10][18]*SH_MAG[3] + P[10][0]*SK_MY[2] - P[10][2]*SK_MY[1] - P[10][16]*SK_MY[3]);
        Kfusion[11] = SK_MY[0]*(P[11][20] + P[11][17]*SH_MAG[0] + P[11][18]*SH_MAG[3] + P[11][0]*SK_MY[2] - P[11][2]*SK_MY[1] - P[11][16]*SK_MY[3]);
        Kfusion[12] = SK_MY[0]*(P[12][20] + P[12][17]*SH_MAG[0] + P[12][18]*SH_MAG[3] + P[12][0]*SK_MY[2] - P[12][2]*SK_MY[1] - P[12][16]*SK_MY[3]);
        Kfusion[13] = SK_MY[0]*(P[13][20] + P[13][17]*SH_MAG[0] + P[13][18]*SH_MAG[3] + P[13][0]*SK_MY[2] - P[13][2]*SK_MY[1] - P[13][16]*SK_MY[3]);
        Kfusion[14] = SK_MY[0]*(P[14][20] + P[14][17]*SH_MAG[0] + P[14][18]*SH_MAG[3] + P[14][0]*SK_MY[2] - P[14][2]*SK_MY[1] - P[14][16]*SK_MY[3]);
        Kfusion[15] = SK_MY[0]*(P[15][20] + P[15][17]*SH_MAG[0] + P[15][18]*SH_MAG[3] + P[15][0]*SK_MY[2] - P[15][2]*SK_MY[1] - P[15][16]*SK_MY[3]);
        // zero Kalman gains to inhibit wind state estimation
        if (!inhibitWindStates) {
            Kfusion[22] = SK_MY[0]*(P[22][20] + P[22][17]*SH_MAG[0] + P[22][18]*SH_MAG[3] + P[22][0]*SK_MY[2] - P[22][2]*SK_MY[1] - P[22][16]*SK_MY[3]);
            Kfusion[23] = SK_MY[0]*(P[23][20] + P[23][17]*SH_MAG[0] + P[23][18]*SH_MAG[3] + P[23][0]*SK_MY[2] - P[23][2]*SK_MY[1] - P[23][16]*SK_MY[3]);
        } else {
            Kfusion[22] = 0.0f;
            Kfusion[23] = 0.0f;
        }
        // zero Kalman gains to inhibit magnetic field state estimation
        if (!inhibitMagStates) {
            Kfusion[16] = SK_MY[0]*(P[16][20] + P[16][17]*SH_MAG[0] + P[16][18]*SH_MAG[3] + P[16][0]*SK_MY[2] - P[16][2]*SK_MY[1] - P[16][16]*SK_MY[3]);
            Kfusion[17] = SK_MY[0]*(P[17][20] + P[17][17]*SH_MAG[0] + P[17][18]*SH_MAG[3] + P[17][0]*SK_MY[2] - P[17][2]*SK_MY[1] - P[17][16]*SK_MY[3]);
            Kfusion[18] = SK_MY[0]*(P[18][20] + P[18][17]*SH_MAG[0] + P[18][18]*SH_MAG[3] + P[18][0]*SK_MY[2] - P[18][2]*SK_MY[1] - P[18][16]*SK_MY[3]);
            Kfusion[19] = SK_MY[0]*(P[19][20] + P[19][17]*SH_MAG[0] + P[19][18]*SH_MAG[3] + P[19][0]*SK_MY[2] - P[19][2]*SK_MY[1] - P[19][16]*SK_MY[3]);
            Kfusion[20] = SK_MY[0]*(P[20][20] + P[20][17]*SH_MAG[0] + P[20][18]*SH_MAG[3] + P[20][0]*SK_MY[2] - P[20][2]*SK_MY[1] - P[20][16]*SK_MY[3]);
            Kfusion[21] = SK_MY[0]*(P[21][20] + P[21][17]*SH_MAG[0] + P[21][18]*SH_MAG[3] + P[21][0]*SK_MY[2] - P[21][2]*SK_MY[1] - P[21][16]*SK_MY[3]);
        } else {
            for (uint8_t i=16; i<=21; i++) {
                Kfusion[i] = 0.0f;
            }
        }

        // set flags to indicate to other processes that fusion has been performede and is required on the next frame
        // this can be used by other fusion processes to avoid fusing on the same frame as this expensive step
        magFusePerformed = true;
        magFuseRequired = true;

        hal.util->perf_end(_perf_test[3]);

    }
    else if (obsIndex == 2) // we are now fusing the Z measurement
    {

        hal.util->perf_begin(_perf_test[4]);

        // calculate observation jacobians
        for (uint8_t i = 0; i<=stateIndexLim; i++) H_MAG[i] = 0.0f;
        H_MAG[0] = magN*(SH_MAG[8] - 2.0f*q1*q2) - magD*SH_MAG[3] - magE*SH_MAG[0];
        H_MAG[1] = magE*SH_MAG[4] + magD*SH_MAG[7] + magN*SH_MAG[1];
        H_MAG[16] = SH_MAG[5];
        H_MAG[17] = 2.0f*q2*q3 - 2.0f*q0*q1;
        H_MAG[18] = SH_MAG[2];
        H_MAG[21] = 1.0f;

        // calculate Kalman gain
        SK_MZ[0] = 1.0f / varInnovMag[2];
        SK_MZ[1] = magE*SH_MAG[0] + magD*SH_MAG[3] - magN*(SH_MAG[8] - 2.0f*q1*q2);
        SK_MZ[2] = magE*SH_MAG[4] + magD*SH_MAG[7] + magN*SH_MAG[1];
        SK_MZ[3] = 2.0f*q0*q1 - 2.0f*q2*q3;
        Kfusion[0] = SK_MZ[0]*(P[0][21] + P[0][18]*SH_MAG[2] + P[0][16]*SH_MAG[5] - P[0][0]*SK_MZ[1] + P[0][1]*SK_MZ[2] - P[0][17]*SK_MZ[3]);
        Kfusion[1] = SK_MZ[0]*(P[1][21] + P[1][18]*SH_MAG[2] + P[1][16]*SH_MAG[5] - P[1][0]*SK_MZ[1] + P[1][1]*SK_MZ[2] - P[1][17]*SK_MZ[3]);
        Kfusion[2] = SK_MZ[0]*(P[2][21] + P[2][18]*SH_MAG[2] + P[2][16]*SH_MAG[5] - P[2][0]*SK_MZ[1] + P[2][1]*SK_MZ[2] - P[2][17]*SK_MZ[3]);
        Kfusion[3] = SK_MZ[0]*(P[3][21] + P[3][18]*SH_MAG[2] + P[3][16]*SH_MAG[5] - P[3][0]*SK_MZ[1] + P[3][1]*SK_MZ[2] - P[3][17]*SK_MZ[3]);
        Kfusion[4] = SK_MZ[0]*(P[4][21] + P[4][18]*SH_MAG[2] + P[4][16]*SH_MAG[5] - P[4][0]*SK_MZ[1] + P[4][1]*SK_MZ[2] - P[4][17]*SK_MZ[3]);
        Kfusion[5] = SK_MZ[0]*(P[5][21] + P[5][18]*SH_MAG[2] + P[5][16]*SH_MAG[5] - P[5][0]*SK_MZ[1] + P[5][1]*SK_MZ[2] - P[5][17]*SK_MZ[3]);
        Kfusion[6] = SK_MZ[0]*(P[6][21] + P[6][18]*SH_MAG[2] + P[6][16]*SH_MAG[5] - P[6][0]*SK_MZ[1] + P[6][1]*SK_MZ[2] - P[6][17]*SK_MZ[3]);
        Kfusion[7] = SK_MZ[0]*(P[7][21] + P[7][18]*SH_MAG[2] + P[7][16]*SH_MAG[5] - P[7][0]*SK_MZ[1] + P[7][1]*SK_MZ[2] - P[7][17]*SK_MZ[3]);
        Kfusion[8] = SK_MZ[0]*(P[8][21] + P[8][18]*SH_MAG[2] + P[8][16]*SH_MAG[5] - P[8][0]*SK_MZ[1] + P[8][1]*SK_MZ[2] - P[8][17]*SK_MZ[3]);
        Kfusion[9] = SK_MZ[0]*(P[9][21] + P[9][18]*SH_MAG[2] + P[9][16]*SH_MAG[5] - P[9][0]*SK_MZ[1] + P[9][1]*SK_MZ[2] - P[9][17]*SK_MZ[3]);
        Kfusion[10] = SK_MZ[0]*(P[10][21] + P[10][18]*SH_MAG[2] + P[10][16]*SH_MAG[5] - P[10][0]*SK_MZ[1] + P[10][1]*SK_MZ[2] - P[10][17]*SK_MZ[3]);
        Kfusion[11] = SK_MZ[0]*(P[11][21] + P[11][18]*SH_MAG[2] + P[11][16]*SH_MAG[5] - P[11][0]*SK_MZ[1] + P[11][1]*SK_MZ[2] - P[11][17]*SK_MZ[3]);
        Kfusion[12] = SK_MZ[0]*(P[12][21] + P[12][18]*SH_MAG[2] + P[12][16]*SH_MAG[5] - P[12][0]*SK_MZ[1] + P[12][1]*SK_MZ[2] - P[12][17]*SK_MZ[3]);
        Kfusion[13] = SK_MZ[0]*(P[13][21] + P[13][18]*SH_MAG[2] + P[13][16]*SH_MAG[5] - P[13][0]*SK_MZ[1] + P[13][1]*SK_MZ[2] - P[13][17]*SK_MZ[3]);
        Kfusion[14] = SK_MZ[0]*(P[14][21] + P[14][18]*SH_MAG[2] + P[14][16]*SH_MAG[5] - P[14][0]*SK_MZ[1] + P[14][1]*SK_MZ[2] - P[14][17]*SK_MZ[3]);
        Kfusion[15] = SK_MZ[0]*(P[15][21] + P[15][18]*SH_MAG[2] + P[15][16]*SH_MAG[5] - P[15][0]*SK_MZ[1] + P[15][1]*SK_MZ[2] - P[15][17]*SK_MZ[3]);
        // zero Kalman gains to inhibit wind state estimation
        if (!inhibitWindStates) {
            Kfusion[22] = SK_MZ[0]*(P[22][21] + P[22][18]*SH_MAG[2] + P[22][16]*SH_MAG[5] - P[22][0]*SK_MZ[1] + P[22][1]*SK_MZ[2] - P[22][17]*SK_MZ[3]);
            Kfusion[23] = SK_MZ[0]*(P[23][21] + P[23][18]*SH_MAG[2] + P[23][16]*SH_MAG[5] - P[23][0]*SK_MZ[1] + P[23][1]*SK_MZ[2] - P[23][17]*SK_MZ[3]);
        } else {
            Kfusion[22] = 0.0f;
            Kfusion[23] = 0.0f;
        }
        // zero Kalman gains to inhibit magnetic field state estimation
        if (!inhibitMagStates) {
            Kfusion[16] = SK_MZ[0]*(P[16][21] + P[16][18]*SH_MAG[2] + P[16][16]*SH_MAG[5] - P[16][0]*SK_MZ[1] + P[16][1]*SK_MZ[2] - P[16][17]*SK_MZ[3]);
            Kfusion[17] = SK_MZ[0]*(P[17][21] + P[17][18]*SH_MAG[2] + P[17][16]*SH_MAG[5] - P[17][0]*SK_MZ[1] + P[17][1]*SK_MZ[2] - P[17][17]*SK_MZ[3]);
            Kfusion[18] = SK_MZ[0]*(P[18][21] + P[18][18]*SH_MAG[2] + P[18][16]*SH_MAG[5] - P[18][0]*SK_MZ[1] + P[18][1]*SK_MZ[2] - P[18][17]*SK_MZ[3]);
            Kfusion[19] = SK_MZ[0]*(P[19][21] + P[19][18]*SH_MAG[2] + P[19][16]*SH_MAG[5] - P[19][0]*SK_MZ[1] + P[19][1]*SK_MZ[2] - P[19][17]*SK_MZ[3]);
            Kfusion[20] = SK_MZ[0]*(P[20][21] + P[20][18]*SH_MAG[2] + P[20][16]*SH_MAG[5] - P[20][0]*SK_MZ[1] + P[20][1]*SK_MZ[2] - P[20][17]*SK_MZ[3]);
            Kfusion[21] = SK_MZ[0]*(P[21][21] + P[21][18]*SH_MAG[2] + P[21][16]*SH_MAG[5] - P[21][0]*SK_MZ[1] + P[21][1]*SK_MZ[2] - P[21][17]*SK_MZ[3]);
        } else {
            for (uint8_t i=16; i<=21; i++) {
                Kfusion[i] = 0.0f;
            }
        }

        // set flags to indicate to other processes that fusion has been performede and is required on the next frame
        // this can be used by other fusion processes to avoid fusing on the same frame as this expensive step
        magFusePerformed = true;
        magFuseRequired = false;

        hal.util->perf_end(_perf_test[4]);

    }

    hal.util->perf_begin(_perf_test[5]);

    // correct the covariance P = (I - K*H)*P
    // take advantage of the empty columns in KH to reduce the
    // number of operations
    for (unsigned i = 0; i<=stateIndexLim; i++) {
        for (unsigned j = 0; j<=2; j++) {
            KH[i][j] = Kfusion[i] * H_MAG[j];
        }
        for (unsigned j = 3; j<=15; j++) {
            KH[i][j] = 0.0f;
        }
        for (unsigned j = 16; j<=21; j++) {
            KH[i][j] = Kfusion[i] * H_MAG[j];
        }
        for (unsigned j = 22; j<=23; j++) {
            KH[i][j] = 0.0f;
        }
    }
    for (unsigned j = 0; j<=stateIndexLim; j++) {
        for (unsigned i = 0; i<=stateIndexLim; i++) {
            ftype res = 0;
            res += KH[i][0] * P[0][j];
            res += KH[i][1] * P[1][j];
            res += KH[i][2] * P[2][j];
            res += KH[i][16] * P[16][j];
            res += KH[i][17] * P[17][j];
            res += KH[i][18] * P[18][j];
            res += KH[i][19] * P[19][j];
            res += KH[i][20] * P[20][j];
            res += KH[i][21] * P[21][j];
            KHP[i][j] = res;
        }
    }
    // Check that we are not going to drive any variances negative and skip the update if so
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
        ForceSymmetry();
        ConstrainVariances();

        // update the states
        // zero the attitude error state - by definition it is assumed to be zero before each observaton fusion
        stateStruct.angErr.zero();

        // correct the state vector
        for (uint8_t j= 0; j<=stateIndexLim; j++) {
            statesArray[j] = statesArray[j] - Kfusion[j] * innovMag[obsIndex];
        }

        // the first 3 states represent the angular misalignment vector. This is
        // is used to correct the estimated quaternion on the current time step
        stateStruct.quat.rotate(stateStruct.angErr);

    } else {
        // record bad axis
        if (obsIndex == 0) {
            faultStatus.bad_xmag = true;
        } else if (obsIndex == 1) {
            faultStatus.bad_ymag = true;
        } else if (obsIndex == 2) {
            faultStatus.bad_zmag = true;
        }
        CovarianceInit();
        hal.util->perf_end(_perf_test[5]);
        return;
    }

    hal.util->perf_end(_perf_test[5]);

}


/*
 * Fuse magnetic heading measurement using explicit algebraic equations generated with Matlab symbolic toolbox.
 * The script file used to generate these and other equations in this filter can be found here:
 * https://github.com/priseborough/InertialNav/blob/master/derivations/RotationVectorAttitudeParameterisation/GenerateNavFilterEquations.m
 * This fusion method only modifies the orientation, does not require use of the magnetic field states and is computationally cheaper.
 * It is suitable for use when the external magnetic field environment is disturbed (eg close to metal structures, on ground).
 * It is not as robust to magnetometer failures.
 * It is not suitable for operation where the horizontal magnetic field strength is weak (within 30 degrees latitude of the the magnetic poles)
*/
//使用MATLAB符号工具箱生成的代数方程进行磁航向融合
//该融合方法只修正航向，不需要磁场状态同时计算量更小
//适用于外部磁场环境受到干扰时(靠近金属物体或者地面)
//不适合水平磁场强度较弱的场合(纬度离磁极30度范围内)
void NavEKF2_core::fuseEulerYaw()
{
    float q0 = stateStruct.quat[0];
    float q1 = stateStruct.quat[1];
    float q2 = stateStruct.quat[2];
    float q3 = stateStruct.quat[3];

    // compass measurement error variance (rad^2) 磁罗盘量测方差
    const float R_YAW = sq(frontend->_yawNoise);

    // calculate observation jacobian, predicted yaw and zero yaw body to earth rotation matrix
    // determine if a 321 or 312 Euler sequence is best
    //计算量测的Jacobian矩阵、航向一步预测、
    float predicted_yaw;
    float measured_yaw;
    float H_YAW[3];
    Matrix3f Tbn_zeroYaw;
    if (fabsf(prevTnb[0][2]) < fabsf(prevTnb[1][2])) {
        // calculate observation jacobian when we are observing the first rotation in a 321 sequence
        //当按照321旋转顺序时 计算量测的Jacobian矩阵
        float t2 = q0*q0;
        float t3 = q1*q1;
        float t4 = q2*q2;
        float t5 = q3*q3;
        float t6 = t2+t3-t4-t5;
        float t7 = q0*q3*2.0f;
        float t8 = q1*q2*2.0f;
        float t9 = t7+t8;
        float t10 = sq(t6);
        if (t10 > 1e-6f) {
            t10 = 1.0f / t10;
        } else {
            return;
        }
        float t11 = t9*t9;
        float t12 = t10*t11;
        float t13 = t12+1.0f;
        float t14;
        if (fabsf(t13) > 1e-3f) {
            t14 = 1.0f/t13;
        } else {
            return;
        }
        float t15 = 1.0f/t6;
        H_YAW[0] = 0.0f;
        H_YAW[1] = t14*(t15*(q0*q1*2.0f-q2*q3*2.0f)+t9*t10*(q0*q2*2.0f+q1*q3*2.0f));
        H_YAW[2] = t14*(t15*(t2-t3+t4-t5)+t9*t10*(t7-t8));

        // calculate predicted and measured yaw angle
        //计算一步预测和量测的磁航向
        Vector3f euler321;
        stateStruct.quat.to_euler(euler321.x, euler321.y, euler321.z);//由四元素计算欧拉角
        predicted_yaw = euler321.z;
        if (use_compass() && yawAlignComplete && magStateInitComplete) {
            // Use measured mag components rotated into earth frame to measure yaw
            //使用旋转到地理系下的磁测量组件测量航向角
            Tbn_zeroYaw.from_euler(euler321.x, euler321.y, 0.0f);//由欧拉角计算姿态矩阵Cbn
            Vector3f magMeasNED = Tbn_zeroYaw*magDataDelayed.mag;//将三轴磁数据转换到北东地坐标系下
            //得到航向角的同时限制航向角的范围 少用一个变量
            measured_yaw = wrap_PI(-atan2f(magMeasNED.y, magMeasNED.x) + _ahrs->get_compass()->get_declination());
        } else if (extNavUsedForYaw) {
            // Get the yaw angle  from the external vision data
            //从外部视觉得到航向角
            extNavDataDelayed.quat.to_euler(euler321.x, euler321.y, euler321.z);
            measured_yaw =  euler321.z;
        } else {
            // no data so use predicted to prevent unconstrained variance growth
            //没有数据，故使用预测来防止未限制的方差增长
            measured_yaw = predicted_yaw;
        }
    } else {
        // calculate observaton jacobian when we are observing a rotation in a 312 sequence
        float t2 = q0*q0;
        float t3 = q1*q1;
        float t4 = q2*q2;
        float t5 = q3*q3;
        float t6 = t2-t3+t4-t5;
        float t7 = q0*q3*2.0f;
        float t10 = q1*q2*2.0f;
        float t8 = t7-t10;
        float t9 = sq(t6);
        if (t9 > 1e-6f) {
            t9 = 1.0f/t9;
        } else {
            return;
        }
        float t11 = t8*t8;
        float t12 = t9*t11;
        float t13 = t12+1.0f;
        float t14;
        if (fabsf(t13) > 1e-3f) {
            t14 = 1.0f/t13;
        } else {
            return;
        }
        float t15 = 1.0f/t6;
        H_YAW[0] = -t14*(t15*(q0*q2*2.0+q1*q3*2.0)-t8*t9*(q0*q1*2.0-q2*q3*2.0));
        H_YAW[1] = 0.0f;
        H_YAW[2] = t14*(t15*(t2+t3-t4-t5)+t8*t9*(t7+t10));

        // calculate predicted and measured yaw angle
        Vector3f euler312 = stateStruct.quat.to_vector312();
        predicted_yaw = euler312.z;
        if (use_compass() && yawAlignComplete && magStateInitComplete) {
            // Use measured mag components rotated into earth frame to measure yaw
            Tbn_zeroYaw.from_euler312(euler312.x, euler312.y, 0.0f);
            Vector3f magMeasNED = Tbn_zeroYaw*magDataDelayed.mag;
            measured_yaw = wrap_PI(-atan2f(magMeasNED.y, magMeasNED.x) + _ahrs->get_compass()->get_declination());
        } else if (extNavUsedForYaw) {
            // Get the yaw angle  from the external vision data
            euler312 = extNavDataDelayed.quat.to_vector312();
            measured_yaw =  euler312.z;
        } else {
            // no data so use predicted to prevent unconstrained variance growth
            measured_yaw = predicted_yaw;
        }
    }

    // Calculate the innovation
    //计算量测更新，涉及到姿态角的均需限制角度范围
    float innovation = wrap_PI(predicted_yaw - measured_yaw);

    // Copy raw value to output variable used for data logging
    //将原始的值赋值给输出变量，用于记录log信息
    innovYaw = innovation;

    // Calculate innovation variance and Kalman gains, taking advantage of the fact that only the first 3 elements in H are non zero
    //利用H中前三个元素非零计算更新方差、卡尔曼滤波增益
    float PH[3];
    float varInnov = R_YAW;
    for (uint8_t rowIndex=0; rowIndex<=2; rowIndex++) {
        PH[rowIndex] = 0.0f;
        for (uint8_t colIndex=0; colIndex<=2; colIndex++) {
            PH[rowIndex] += P[rowIndex][colIndex]*H_YAW[colIndex];
        }
        varInnov += H_YAW[rowIndex]*PH[rowIndex];
    }
    float varInnovInv;
    if (varInnov >= R_YAW) {
        varInnovInv = 1.0f / varInnov;
        // output numerical health status
        faultStatus.bad_yaw = false;//将所有的标志位放在一个结构体中
    } else {
        // the calculation is badly conditioned, so we cannot perform fusion on this step
        // we reset the covariance matrix and try again next measurement
        //如果计算出错，则放弃此次融合。重置方差矩阵并尝试利用下次量测进行融合
        CovarianceInit();
        // output numerical health status
        faultStatus.bad_yaw = true;
        return;
    }

    // calculate Kalman gain
    //计算滤波增益，由于H阵阵很多零，故不用全部循环计算矩阵乘法
    for (uint8_t rowIndex=0; rowIndex<=stateIndexLim; rowIndex++) {
        Kfusion[rowIndex] = 0.0f;
        for (uint8_t colIndex=0; colIndex<=2; colIndex++) {
            Kfusion[rowIndex] += P[rowIndex][colIndex]*H_YAW[colIndex];
        }
        Kfusion[rowIndex] *= varInnovInv;
    }

    // calculate the innovation test ratio
    //计算修正量测试比率
    yawTestRatio = sq(innovation) / (sq(MAX(0.01f * (float)frontend->_yawInnovGate, 1.0f)) * varInnov);

    // Declare the magnetometer unhealthy if the innovation test fails
    if (yawTestRatio > 1.0f) {
        magHealth = false;
        // On the ground a large innovation could be due to large initial gyro bias or magnetic interference from nearby objects
        // If we are flying, then it is more likely due to a magnetometer fault and we should not fuse the data
        //在地面上，一个较大的修正量可能由于初始陀螺偏置较大或者在地面附近有磁场干扰
        //如果在飞行中，一个较大的修正量更有可能是磁传感器故障，此时不融合
        if (inFlight) {
            return;
        }
    } else {
        magHealth = true;
    }

    // limit the innovation so that initial corrections are not too large
    //限制修正量，以便初始修正不致于过大
    if (innovation > 0.5f) {
        innovation = 0.5f;
    } else if (innovation < -0.5f) {
        innovation = -0.5f;
    }

    // correct the covariance using P = P - K*H*P taking advantage of the fact that only the first 3 elements in H are non zero
    // calculate K*H*P
    //利用H阵中仅有前三个元素非零，计算状态估计协方差阵
    for (uint8_t row = 0; row <= stateIndexLim; row++) {
        for (uint8_t column = 0; column <= 2; column++) {
            KH[row][column] = Kfusion[row] * H_YAW[column];
        }
    }
    for (uint8_t row = 0; row <= stateIndexLim; row++) {
        for (uint8_t column = 0; column <= stateIndexLim; column++) {
            float tmp = KH[row][0] * P[0][column];
            tmp += KH[row][1] * P[1][column];
            tmp += KH[row][2] * P[2][column];
            KHP[row][column] = tmp;
        }
    }

    // Check that we are not going to drive any variances negative and skip the update if so
    //防止P阵对角线元素出现负的，如果出现则跳过此次更新
    bool healthyFusion = true;
    for (uint8_t i= 0; i<=stateIndexLim; i++) {
        if (KHP[i][i] > P[i][i]) {
            healthyFusion = false;
        }
    }
    if (healthyFusion) {
        // update the covariance matrix 更新状态估计的协方差矩阵
        for (uint8_t i= 0; i<=stateIndexLim; i++) {
            for (uint8_t j= 0; j<=stateIndexLim; j++) {
                P[i][j] = P[i][j] - KHP[i][j];
            }
        }

        // force the covariance matrix to be symmetrical and limit the variances to prevent ill-condiioning.
        //强制P阵对称并根据不同的状态量限制P阵中相应对角线元素的大小
        ForceSymmetry();
        ConstrainVariances();

        // zero the attitude error state - by definition it is assumed to be zero before each observaton fusion
        stateStruct.angErr.zero();

        // correct the state vector  状态估计
        for (uint8_t i=0; i<=stateIndexLim; i++) {
            statesArray[i] -= Kfusion[i] * innovation;
        }

        // the first 3 states represent the angular misalignment vector. This is
        // is used to correct the estimated quaternion on the current time step
        stateStruct.quat.rotate(stateStruct.angErr);

        // record fusion event
        faultStatus.bad_yaw = false;
        lastYawTime_ms = imuSampleTime_ms;


    } else {
        // record fusion numerical health status
        faultStatus.bad_yaw = true;
    }
}

/*
 * Fuse declination angle using explicit algebraic equations generated with Matlab symbolic toolbox.
 * The script file used to generate these and other equations in this filter can be found here:
 * https://github.com/priseborough/InertialNav/blob/master/derivations/RotationVectorAttitudeParameterisation/GenerateNavFilterEquations.m
 * This is used to prevent the declination of the EKF earth field states from drifting during operation without GPS
 * or some other absolute position or velocity reference
*/
//使用matlab符号工具箱生成的显式代数方程来融合磁偏角
//在没有GPS或者没有绝对的位置速度作为参考时，防止EKF地理系下状态发散
void NavEKF2_core::FuseDeclination(float declErr)
{
    // declination error variance (rad^2)
    const float R_DECL = sq(declErr);//sq return powf(static_cast<float>(val),2)

    // copy required states to local variables 将需要的状态赋值给本地变量
    float magN = stateStruct.earth_magfield.x;//stateStruct 28维的结构体  earth_magfield是Vector3f类型的变量
    float magE = stateStruct.earth_magfield.y;

    // prevent bad earth field states from causing numerical errors or exceptions
    //防止地理系下磁场状态变量错误造成数据计算误差或者异常
    if (magN < 1e-3f) {
        return;
    }

    // Calculate observation Jacobian and Kalman gains
    //计算量测的Jacobian矩阵和卡尔曼滤波增益
    float t2 = magE*magE;
    float t3 = magN*magN;
    float t4 = t2+t3;
    float t5 = 1.0f/t4;
    float t22 = magE*t5;
    float t23 = magN*t5;
    float t6 = P[16][16]*t22;
    float t13 = P[17][16]*t23;
    float t7 = t6-t13;
    float t8 = t22*t7;
    float t9 = P[16][17]*t22;
    float t14 = P[17][17]*t23;
    float t10 = t9-t14;
    float t15 = t23*t10;
    float t11 = R_DECL+t8-t15; // innovation variance
    if (t11 < R_DECL) {
        return;
    }
    float t12 = 1.0f/t11;

    float H_MAG[24];
    H_MAG[16] = -magE*t5;
    H_MAG[17] = magN*t5;

    for (uint8_t i=0; i<=15; i++) {
        Kfusion[i] = -t12*(P[i][16]*t22-P[i][17]*t23);
    }
    Kfusion[16] = -t12*(t6-P[16][17]*t23);
    Kfusion[17] = t12*(t14-P[17][16]*t22);
    for (uint8_t i=17; i<=23; i++) {
        Kfusion[i] = -t12*(P[i][16]*t22-P[i][17]*t23);
    }

    // get the magnetic declination  得到磁偏角 如何得到?
    //use_compass()如果用到磁力计则返回true
    //_arhs->get_compass()返回一个指向const Compass类型的指针，该指针调用Compass类中的成员函数get_declination 通过指针调用
    float magDecAng = use_compass() ? _ahrs->get_compass()->get_declination() : 0;

    // Calculate the innovation  计算修正量  计算磁航向
    float innovation = atan2f(magE , magN) - magDecAng;

    // limit the innovation to protect against data errors
    //为了防止数据错误，限制修正量的大小
    if (innovation > 0.5f) {
        innovation = 0.5f;
    } else if (innovation < -0.5f) {
        innovation = -0.5f;
    }

    // correct the covariance P = (I - K*H)*P
    // take advantage of the empty columns in KH to reduce the
    // number of operations
    for (unsigned i = 0; i<=stateIndexLim; i++) {
        for (unsigned j = 0; j<=15; j++) {
            KH[i][j] = 0.0f;
        }
        KH[i][16] = Kfusion[i] * H_MAG[16];
        KH[i][17] = Kfusion[i] * H_MAG[17];
        for (unsigned j = 18; j<=23; j++) {
            KH[i][j] = 0.0f;
        }
    }
    for (unsigned j = 0; j<=stateIndexLim; j++) {
        for (unsigned i = 0; i<=stateIndexLim; i++) {
            KHP[i][j] = KH[i][16] * P[16][j] + KH[i][17] * P[17][j];
        }
    }

    // Check that we are not going to drive any variances negative and skip the update if so
    //如果协方差阵主对角线元素出现负值，则跳过此次更新
    bool healthyFusion = true;
    for (uint8_t i= 0; i<=stateIndexLim; i++) {
        if (KHP[i][i] > P[i][i]) {
            healthyFusion = false;
        }
    }
    //如果P阵主对角线元非负，则见融合，更新状态估计协方差阵P
    if (healthyFusion) {
        // update the covariance matrix
        for (uint8_t i= 0; i<=stateIndexLim; i++) {
            for (uint8_t j= 0; j<=stateIndexLim; j++) {
                P[i][j] = P[i][j] - KHP[i][j];
            }
        }

        // force the covariance matrix to be symmetrical and limit the variances to prevent ill-condiioning.
        ForceSymmetry();
        ConstrainVariances();

        // zero the attitude error state - by definition it is assumed to be zero before each observaton fusion
        //将状态中姿态误差置零，根据定义每次量测融合之前姿态误差都被置零
        stateStruct.angErr.zero();

        // correct the state vector
        for (uint8_t j= 0; j<=stateIndexLim; j++) {
            statesArray[j] = statesArray[j] - Kfusion[j] * innovation;
        }

        // the first 3 states represent the angular misalignment vector. This is
        // is used to correct the estimated quaternion on the current time step
        //前三个元素代表姿态误差，被用来修正当前时刻估计的四元素
        stateStruct.quat.rotate(stateStruct.angErr);

        // record fusion health status
        faultStatus.bad_decl = false;
    } else {
        // record fusion health status
        faultStatus.bad_decl = true;
    }
}

/********************************************************
*                   MISC FUNCTIONS                      *
********************************************************/
//你能优化吗?能用到你自己编程里面吗?
// align the NE earth magnetic field states with the published declination
//将北向、东向的地理系下磁场状态和发布的磁偏角
void NavEKF2_core::alignMagStateDeclination()
{
    // don't do this if we already have a learned magnetic field
    if (magFieldLearned) {
        return;
    }

    // get the magnetic declination  
    float magDecAng = use_compass() ? _ahrs->get_compass()->get_declination() : 0;

    // rotate the NE values so that the declination matches the published value
    Vector3f initMagNED = stateStruct.earth_magfield;
    float magLengthNE = norm(initMagNED.x,initMagNED.y);
    stateStruct.earth_magfield.x = magLengthNE * cosf(magDecAng);//magDecAng 为磁偏角
    stateStruct.earth_magfield.y = magLengthNE * sinf(magDecAng);

    if (!inhibitMagStates) {//当磁场状态和协方差保持不变时，inhibitMagStates保持不变时，为真
        // zero the corresponding state covariances if magnetic field state learning is active
        float var_16 = P[16][16];
        float var_17 = P[17][17];
        zeroRows(P,16,17);
        zeroCols(P,16,17);
        P[16][16] = var_16;
        P[17][17] = var_17;

        // fuse the declination angle to establish covariances and prevent large swings in declination
        // during initial fusion
        //融合磁偏角以建立协方差阵，同时在融合初期防止磁偏角的大的摆动
        FuseDeclination(0.1f);

    }
}

// record a magentic field state reset event
void NavEKF2_core::recordMagReset()
{
    magStateInitComplete = true;
    if (inFlight) {
        finalInflightMagInit = true;
    }
    // take a snap-shot of the vertical position, quaternion  and yaw innovation to use as a reference
    // for post alignment checks
    posDownAtLastMagReset = stateStruct.position.z;
    quatAtLastMagReset = stateStruct.quat;
    yawInnovAtLastMagReset = innovYaw;
}


#endif // HAL_CPU_CLASS
