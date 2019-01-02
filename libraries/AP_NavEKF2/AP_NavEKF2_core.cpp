#include <AP_HAL/AP_HAL.h>

#if HAL_CPU_CLASS >= HAL_CPU_CLASS_150

#include "AP_NavEKF2.h"
#include "AP_NavEKF2_core.h"
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Vehicle/AP_Vehicle.h>

#include <stdio.h>

extern const AP_HAL::HAL& hal;

#define earthRate 0.000072921f // earth rotation rate (rad/sec)

// initial imu bias uncertainty (deg/sec)
#define INIT_ACCEL_BIAS_UNCERTAINTY 0.5f

// maximum allowed gyro bias (rad/sec)
#define GYRO_BIAS_LIMIT 0.5f

// constructor
NavEKF2_core::NavEKF2_core(void) :
    stateStruct(*reinterpret_cast<struct state_elements *>(&statesArray)),

    _perf_UpdateFilter(hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "EK2_UpdateFilter")),
    _perf_CovariancePrediction(hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "EK2_CovariancePrediction")),
    _perf_FuseVelPosNED(hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "EK2_FuseVelPosNED")),
    _perf_FuseMagnetometer(hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "EK2_FuseMagnetometer")),
    _perf_FuseAirspeed(hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "EK2_FuseAirspeed")),
    _perf_FuseSideslip(hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "EK2_FuseSideslip")),
    _perf_TerrainOffset(hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "EK2_TerrainOffset")),
    _perf_FuseOptFlow(hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "EK2_FuseOptFlow"))
{
    _perf_test[0] = hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "EK2_Test0");
    _perf_test[1] = hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "EK2_Test1");
    _perf_test[2] = hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "EK2_Test2");
    _perf_test[3] = hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "EK2_Test3");
    _perf_test[4] = hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "EK2_Test4");
    _perf_test[5] = hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "EK2_Test5");
    _perf_test[6] = hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "EK2_Test6");
    _perf_test[7] = hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "EK2_Test7");
    _perf_test[8] = hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "EK2_Test8");
    _perf_test[9] = hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "EK2_Test9");
}

// setup this core backend
bool NavEKF2_core::setup_core(NavEKF2 *_frontend, uint8_t _imu_index, uint8_t _core_index)
{
    frontend = _frontend;
    imu_index = _imu_index;
    core_index = _core_index;
    _ahrs = frontend->_ahrs;

    /*
      the imu_buffer_length needs to cope with a 260ms delay at a
      maximum fusion rate of 100Hz. Non-imu data coming in at faster
      than 100Hz is downsampled. For 50Hz main loop rate we need a
      shorter buffer.
     */
    if (AP::ins().get_sample_rate() < 100) {
        imu_buffer_length = 13;
    } else {
        // maximum 260 msec delay at 100 Hz fusion rate
        imu_buffer_length = 26;
    }
    if(!storedGPS.init(OBS_BUFFER_LENGTH)) {
        return false;
    }
    if(!storedMag.init(OBS_BUFFER_LENGTH)) {
        return false;
    }
    if(!storedBaro.init(OBS_BUFFER_LENGTH)) {
        return false;
    } 
    if(!storedTAS.init(OBS_BUFFER_LENGTH)) {
        return false;
    }
    if(!storedOF.init(OBS_BUFFER_LENGTH)) {
        return false;
    }
    // Note: the use of dual range finders potentially doubles the amount of to be stored
    if(!storedRange.init(2*OBS_BUFFER_LENGTH)) {
        return false;
    }
    // Note: range beacon data is read one beacon at a time and can arrive at a high rate
    if(!storedRangeBeacon.init(imu_buffer_length)) {
        return false;
    }
    if(!storedExtNav.init(OBS_BUFFER_LENGTH)) {
        return false;
    }
    if(!storedIMU.init(imu_buffer_length)) {
        return false;
    }
    if(!storedOutput.init(imu_buffer_length)) {
        return false;
    }

    return true;
}
    

/********************************************************
*                   INIT FUNCTIONS                      *
********************************************************/

// Use a function call rather than a constructor to initialise variables because it enables the filter to be re-started in flight if necessary.
void NavEKF2_core::InitialiseVariables()
{
    // calculate the nominal filter update rate
    const AP_InertialSensor &ins = AP::ins();
    localFilterTimeStep_ms = (uint8_t)(1000*ins.get_loop_delta_t());
    localFilterTimeStep_ms = MAX(localFilterTimeStep_ms,10);

    // initialise time stamps
    imuSampleTime_ms = frontend->imuSampleTime_us / 1000;
    lastHealthyMagTime_ms = imuSampleTime_ms;
    prevTasStep_ms = imuSampleTime_ms;
    prevBetaStep_ms = imuSampleTime_ms;
    lastMagUpdate_us = 0;
    lastBaroReceived_ms = imuSampleTime_ms;
    lastVelPassTime_ms = 0;
    lastPosPassTime_ms = 0;
    lastHgtPassTime_ms = 0;
    lastTasPassTime_ms = 0;
    lastYawTime_ms = imuSampleTime_ms;
    lastTimeGpsReceived_ms = 0;
    secondLastGpsTime_ms = 0;
    lastDecayTime_ms = imuSampleTime_ms;
    timeAtLastAuxEKF_ms = imuSampleTime_ms;
    flowValidMeaTime_ms = imuSampleTime_ms;
    rngValidMeaTime_ms = imuSampleTime_ms;
    flowMeaTime_ms = 0;
    prevFlowFuseTime_ms = 0;
    gndHgtValidTime_ms = 0;
    ekfStartTime_ms = imuSampleTime_ms;
    lastGpsVelFail_ms = 0;
    lastGpsAidBadTime_ms = 0;
    timeTasReceived_ms = 0;
    magYawResetTimer_ms = imuSampleTime_ms;
    lastPreAlignGpsCheckTime_ms = imuSampleTime_ms;
    lastPosReset_ms = 0;
    lastVelReset_ms = 0;
    lastPosResetD_ms = 0;
    lastRngMeasTime_ms = 0;
    terrainHgtStableSet_ms = 0;

    // initialise other variables
    gpsNoiseScaler = 1.0f;
    hgtTimeout = true;
    magTimeout = false;
    allMagSensorsFailed = false;
    tasTimeout = true;
    badMagYaw = false;
    badIMUdata = false;
    finalInflightYawInit = false;
    finalInflightMagInit = false;
    dtIMUavg = 0.0025f;
    dtEkfAvg = EKF_TARGET_DT;
    dt = 0;
    velDotNEDfilt.zero();
    lastKnownPositionNE.zero();
    prevTnb.zero();
    memset(&P[0][0], 0, sizeof(P));
    memset(&nextP[0][0], 0, sizeof(nextP));
    memset(&processNoise[0], 0, sizeof(processNoise));
    flowDataValid = false;
    rangeDataToFuse  = false;
    fuseOptFlowData = false;
    Popt = 0.0f;
    terrainState = 0.0f;
    prevPosN = stateStruct.position.x;
    prevPosE = stateStruct.position.y;
    inhibitGndState = false;
    flowGyroBias.x = 0;
    flowGyroBias.y = 0;
    heldVelNE.zero();
    PV_AidingMode = AID_NONE;
    PV_AidingModePrev = AID_NONE;
    posTimeout = true;
    velTimeout = true;
    memset(&faultStatus, 0, sizeof(faultStatus));
    hgtRate = 0.0f;
    mag_state.q0 = 1;
    mag_state.DCM.identity();
    onGround = true;
    prevOnGround = true;
    inFlight = false;
    prevInFlight = false;
    manoeuvring = false;
    inhibitWindStates = true;
    inhibitMagStates = true;
    gndOffsetValid =  false;
    validOrigin = false;
    takeoffExpectedSet_ms = 0;
    expectGndEffectTakeoff = false;
    touchdownExpectedSet_ms = 0;
    expectGndEffectTouchdown = false;
    gpsSpdAccuracy = 0.0f;
    gpsPosAccuracy = 0.0f;
    gpsHgtAccuracy = 0.0f;
    baroHgtOffset = 0.0f;
    yawResetAngle = 0.0f;
    lastYawReset_ms = 0;
    tiltErrFilt = 1.0f;
    tiltAlignComplete = false;
    yawAlignComplete = false;
    stateIndexLim = 23;
    baroStoreIndex = 0;
    rangeStoreIndex = 0;
    magStoreIndex = 0;
    gpsStoreIndex = 0;
    tasStoreIndex = 0;
    ofStoreIndex = 0;
    delAngCorrection.zero();
    velErrintegral.zero();
    posErrintegral.zero();
    gpsGoodToAlign = false;
    gpsNotAvailable = true;
    motorsArmed = false;
    prevMotorsArmed = false;
    innovationIncrement = 0;
    lastInnovation = 0;
    memset(&gpsCheckStatus, 0, sizeof(gpsCheckStatus));
    gpsSpdAccPass = false;
    ekfInnovationsPass = false;
    sAccFilterState1 = 0.0f;
    sAccFilterState2 = 0.0f;
    lastGpsCheckTime_ms = 0;
    lastInnovPassTime_ms = 0;
    lastInnovFailTime_ms = 0;
    gpsAccuracyGood = false;
    memset(&gpsloc_prev, 0, sizeof(gpsloc_prev));
    gpsDriftNE = 0.0f;
    gpsVertVelFilt = 0.0f;
    gpsHorizVelFilt = 0.0f;
    memset(&statesArray, 0, sizeof(statesArray));
    posDownDerivative = 0.0f;
    posDown = 0.0f;
    posVelFusionDelayed = false;
    optFlowFusionDelayed = false;
    airSpdFusionDelayed = false;
    sideSlipFusionDelayed = false;
    posResetNE.zero();
    velResetNE.zero();
    posResetD = 0.0f;
    hgtInnovFiltState = 0.0f;
    if (_ahrs->get_compass()) {
        magSelectIndex = _ahrs->get_compass()->get_primary();
    }
    imuDataDownSampledNew.delAng.zero();
    imuDataDownSampledNew.delVel.zero();
    imuDataDownSampledNew.delAngDT = 0.0f;
    imuDataDownSampledNew.delVelDT = 0.0f;
    runUpdates = false;
    framesSincePredict = 0;
    lastMagOffsetsValid = false;
    magStateResetRequest = false;
    magStateInitComplete = false;
    magYawResetRequest = false;
    gpsYawResetRequest = false;
    posDownAtLastMagReset = stateStruct.position.z;
    yawInnovAtLastMagReset = 0.0f;
    quatAtLastMagReset = stateStruct.quat;
    magFieldLearned = false;
    delAngBiasLearned = false;
    memset(&filterStatus, 0, sizeof(filterStatus));
    gpsInhibit = false;
    activeHgtSource = 0;
    memset(&rngMeasIndex, 0, sizeof(rngMeasIndex));
    memset(&storedRngMeasTime_ms, 0, sizeof(storedRngMeasTime_ms));
    memset(&storedRngMeas, 0, sizeof(storedRngMeas));
    terrainHgtStable = true;
    ekfOriginHgtVar = 0.0f;
    ekfGpsRefHgt = 0.0;
    velOffsetNED.zero();
    posOffsetNED.zero();
    memset(&velPosObs, 0, sizeof(velPosObs));

    // range beacon fusion variables
    memset(&rngBcnDataNew, 0, sizeof(rngBcnDataNew));
    memset(&rngBcnDataDelayed, 0, sizeof(rngBcnDataDelayed));
    rngBcnStoreIndex = 0;
    lastRngBcnPassTime_ms = 0;
    rngBcnTestRatio = 0.0f;
    rngBcnHealth = false;
    rngBcnTimeout = true;
    varInnovRngBcn = 0.0f;
    innovRngBcn = 0.0f;
    memset(&lastTimeRngBcn_ms, 0, sizeof(lastTimeRngBcn_ms));
    rngBcnDataToFuse = false;
    beaconVehiclePosNED.zero();
    beaconVehiclePosErr = 1.0f;
    rngBcnLast3DmeasTime_ms = 0;
    rngBcnGoodToAlign = false;
    lastRngBcnChecked = 0;
    receiverPos.zero();
    memset(&receiverPosCov, 0, sizeof(receiverPosCov));
    rngBcnAlignmentStarted =  false;
    rngBcnAlignmentCompleted = false;
    lastBeaconIndex = 0;
    rngBcnPosSum.zero();
    numBcnMeas = 0;
    rngSum = 0.0f;
    N_beacons = 0;
    maxBcnPosD = 0.0f;
    minBcnPosD = 0.0f;
    bcnPosOffset = 0.0f;
    bcnPosOffsetMax = 0.0f;
    bcnPosOffsetMaxVar = 0.0f;
    OffsetMaxInnovFilt = 0.0f;
    bcnPosOffsetMin = 0.0f;
    bcnPosOffsetMinVar = 0.0f;
    OffsetMinInnovFilt = 0.0f;
    rngBcnFuseDataReportIndex = 0;
    memset(&rngBcnFusionReport, 0, sizeof(rngBcnFusionReport));
    last_gps_idx = 0;

    // external nav data fusion
    memset(&extNavDataNew, 0, sizeof(extNavDataNew));
    memset(&extNavDataDelayed, 0, sizeof(extNavDataDelayed));
    extNavDataToFuse = false;
    extNavMeasTime_ms = 0;
    extNavLastPosResetTime_ms = 0;
    extNavUsedForYaw = false;
    extNavUsedForPos = false;
    extNavYawResetRequest = false;

    // zero data buffers
    storedIMU.reset();
    storedGPS.reset();
    storedMag.reset();
    storedBaro.reset();
    storedTAS.reset();
    storedRange.reset();
    storedOutput.reset();
    storedRangeBeacon.reset();
    storedExtNav.reset();
}

// Initialise the states from accelerometer and magnetometer data (if present)
// This method can only be used when the vehicle is static
//¸ù¾Ý¼Ó¼ÆºÍ´ÅÂÞÅÌÊý¾Ý³õÊ¼»¯×´Ì¬£¬±ØÐëÔÚ³µÁ¾¾²Ö¹Ê±²ÅÄÜ½øÐÐ´ËÏî³õÊ¼»¯
bool NavEKF2_core::InitialiseFilterBootstrap(void)
{    //Èç¹ûÃ»ÓÐGPSËø¶¨¾Í²»ÓÃ³õÊ¼»¯
    // If we are a plane and don't have GPS lock then don't initialise
    if (assume_zero_sideslip() && AP::gps().status() < AP_GPS::GPS_OK_FIX_3D) {
        hal.util->snprintf(prearm_fail_string,
                           sizeof(prearm_fail_string),
                           "EKF2 init failure: No GPS lock");
        statesInitialised = false;
        return false;
    }
    //Èç¹û³õÊ¼»¯ÒÑ¾­Íê³É
    if (statesInitialised) {
        // we are initialised, but we don't return true until the IMU
        // buffer has been filled. This prevents a timing
        // vulnerability with a pause in IMU data during filter startup
        //½øÐÐ³õÊ¼»¯,µ«Ö±µ½IMU»º³åÇø±»ÌîÂú²Å·µ»ØÕæ£¬ÕâÎªÁË·ÀÖ¹ÂË²¨Æ÷Æô¶¯
        //ÆÚ¼äÔÝÍ£IMUÊý¾ÝµÄÊ±¼äÂ©¶´
        readIMUData();
        readMagData();
        readGpsData();
        readBaroData();
        return storedIMU.is_filled();
    }
    //½«ÖØÐÂÊ¹ÓÃµÄ±äÁ¿³õÊ¼»¯Îª0
    // set re-used variables to zero
    InitialiseVariables();
   //»ñµÃIMUÊý¾Ý
    const AP_InertialSensor &ins = AP::ins();

    // Initialise IMU data
    dtIMUavg = ins.get_loop_delta_t();//¼ÆËã»ñÈ¡IMUÊý¾ÝµÄÊ±¼ä
    readIMUData();
    storedIMU.reset_history(imuDataNew);//½«imuDataNewÐ´Èë»·ÐÎbufÖÐ
    imuDataDelayed = imuDataNew;//½á¹¹Ìå¿ÉÒÔÖ±½Ó¸³Öµ£¬imuDataNewÖÐ±äÁ¿delAng delVel delAngDT delVelDT time_ms
    //IMU²âÁ¿µÄ¼Ó¼ÆÔÚxyz»úÌå×ø±êÏµ
    // acceleration vector in XYZ body axes measured by the IMU (m/s^2)
    Vector3f initAccVec;
    //¶Ô¼¸¸öÖÜÆÚÄÚµÄ¼Ó¼ÆÊý¾ÝÇóÆ½¾ù
    // TODO we should average accel readings over several cycles
    initAccVec = ins.get_accel(imu_index);//·µ»Ø¼Ó¼ÆÄ³Ò»ÖáµÄÊý¾Ý
    //»ñµÃ´ÅÊý¾Ý
    // read the magnetometer data
    readMagData();
    //¹éÒ»»¯¼Ó¼ÆÊý¾Ý ¼ÆËã³õÊ¼¸©Ñö½ÇºÍºá¹ö½Ç
    // normalise the acceleration vector
    float pitch=0, roll=0;
    if (initAccVec.length() > 0.001f) {//length() norm(x,y,z)
        initAccVec.normalize();//¼Ó¼Æ¹éÒ»»¯ *this /= length ÖØÔØÁË²Ù×÷·û /=

        // calculate initial pitch angle Ê¹ÓÃ¼Ó¼Æ¼ÆËã³õÊ¼¸©Ñö½Ç
        pitch = asinf(initAccVec.x);

        // calculate initial roll angle Ê¹ÓÃ¼Ó¼Æ¼ÆËã³õÊ¼ºá¹ö½Ç
        roll = atan2f(-initAccVec.y , -initAccVec.z);
    }

    // calculate initial roll and pitch orientation Å·À­½Çµ½ËÄÔªÊý×ª»»
    //quatÊÇ½á¹¹ÌåstateStruct½á¹¹ÌåÖÐ³ÉÔ±£¬¶øquatµÄÀàÐÍÎªQuaternionµÄÀà¶ÔÏó£¬¹Êquatµ÷ÓÃQuaternionÀàµÄ·½·¨
    stateStruct.quat.from_euler(roll, pitch, 0.0f);//µÃµ½³õÊ¼µÄËÄÔªÊý£¬º½ÏòÎªÁã

    // initialise dynamic states  ³õÊ¼»¯ÔË¶¯×´Ì¬ ½«Î»ÖÃËÙ¶È½Ç¶ÈÎó²î³õÊ¼»¯ÎªÁã
    //stateStructÊÇEKF2µÄ28Î¬×´Ì¬±äÁ¿£¬velocity/position/angErr¶¼ÊÇVector3ÐÍµÄ±äÁ¿
    stateStruct.velocity.zero();
    stateStruct.position.zero();
    stateStruct.angErr.zero();

    // initialise static process model states ³õÊ¼»¯¾²Ì¬Ä£ÐÍµÄ×´Ì¬±äÁ¿
    //EKFÒª´¦ÀíµÄ×´Ì¬±äÁ¿ÓÐÄÄÐ©£¬»¹ÓÐÊä³öµÄ×´Ì¬±äÁ¿
    //stateStruct´æ´¢µÄÊÇEKF2µÄ×´Ì¬±äÁ¿£¬¹²28Î¬ ½«×´Ì¬±äÁ¿³õÊ¼»¯£¬±ê¶ÈÒòÊý³õÊ¼»¯Îª1
    stateStruct.gyro_bias.zero();
    stateStruct.gyro_scale.x = 1.0f;
    stateStruct.gyro_scale.y = 1.0f;
    stateStruct.gyro_scale.z = 1.0f;
    stateStruct.accel_zbias = 0.0f;
    stateStruct.wind_vel.zero();
    stateStruct.earth_magfield.zero();
    stateStruct.body_magfield.zero();
    //¶ÁÈ¡GPSÊý¾Ý£¬²¢ÉèÖÃÎ»ÖÃºÍËÙ¶È×´Ì¬ÐÅÏ¢
    // read the GPS and set the position and velocity states
    readGpsData();
    ResetVelocity();
    ResetPosition();

    // read the barometer and set the height state ¶ÁÈ¡ÆøÑ¹¼ÆÊý¾Ý
    readBaroData();
    ResetHeight();//¸´Î»¸ß¶ÈÊý¾Ý
   //ÔÚNEDµ¼º½ÏµÏÂ¶¨ÒåµØÇòÐý×ªÊ¸Á¿
    // define Earth rotation vector in the NED navigation frame
    //const AP_ARHR *_ahrs  _ahrs->get_home() ·µ»ØµÄLocationµÄ½á¹¹Ìå£¬ÀïÃæÖ÷ÒªÊÇintÐÍµÄlatitude longitude height
    //earthRateNED NED×ø±êÏµÏÂµÄµØÇò½ÇËÙÂÊ
    //µØÇò×Ô×ª½ÇËÙÂÊwieÔÚNEDÏÂµÄÍ¶Ó°
    calcEarthRateNED(earthRateNED, _ahrs->get_home().lat);

    // initialise the covariance matrix ³õÊ¼»¯Ð­·½²î¾ØÕó
    CovarianceInit();//Ð­·½²îÕóP³õÊ¼»¯£¬²»Í¬µÄÔªËØÉèÖÃ²»Í¬µÄ³õÖµ

    // reset output states ¸´Î»Êä³ö×´Ì¬
    StoreOutputReset();

    // set to true now that states have be initialised ÏÖÔÚµÄ×´Ì¬ÒÑ³õÊ¼»¯ÎªÕæ
    statesInitialised = true;
    //·µ»ØfalseµÈ´ýIMU buffµÄÌî³ä
    // we initially return false to wait for the IMU buffer to fill
    return false;
}

//ÎªÊ²Ã´ÕâÃ´Ð´?Äã»áÔõÃ´Ð´?Çø±ðÔÚÄÄÀï?ÈçºÎÓÅ»¯¸Ã¶Î´úÂë?ÈçºÎ½«´ËÓÃµ½ÓÅ»¯ÄãÖ®Ç°ºÍÖ®ºóµÄ´úÂë?
// initialise the covariance matrix  ³õÊ¼»¯Ð­·½²î¾ØÕóP
void NavEKF2_core::CovarianceInit()
{
    // zero the matrix ¸´Î»Ð­·½²îÕó Matrix24 P
    //×´Ì¬ÏòÁ¿28Î¬£¬ÎªºÎÖ»³õÊ¼»¯24¸ö×´Ì¬ÏòÁ¿
    memset(P, 0, sizeof(P));

    // attitude error ×ËÌ¬Îó²îÐ­·½²îÕó
    P[0][0]   = 0.1f;
    P[1][1]   = 0.1f;
    P[2][2]   = 0.1f;
    // velocities ËÙ¶ÈÐ­·½²î
    // NAV_EKF2 *fronted Ö¸ÏòÀà¶ÔÏóµÄÖ¸Õë »òÕß(*fronted)._gpsHorizVelNoise
    P[3][3]   = sq(frontend->_gpsHorizVelNoise);
    P[4][4]   = P[3][3];
    P[5][5]   = sq(frontend->_gpsVertVelNoise);
    // positions Î»ÖÃÐ­·½²î
    P[6][6]   = sq(frontend->_gpsHorizPosNoise);
    P[7][7]   = P[6][6];
    P[8][8]   = sq(frontend->_baroAltNoise);
    // gyro delta angle biases ÍÓÂÝ½Ç¶ÈÆ«ÒÆ
    //dtEkfAvg Á½´ÎEKF¸üÐÂµÄÆÚÍûÊ±¼ä¼ä¸ô InitialGyroBiasUncertainty()·µ»Ø2.5f
    P[9][9] = sq(radians(InitialGyroBiasUncertainty() * dtEkfAvg));
    P[10][10] = P[9][9];
    P[11][11] = P[9][9];
    // gyro scale factor biases ÍÓÂÝ±ê¶ÈÒòÊýÆ«ÒÆ
    P[12][12] = sq(1e-3);
    P[13][13] = P[12][12];
    P[14][14] = P[12][12];
    // Z delta velocity bias  ´¹Ö±ËÙ¶ÈÆ«ÒÆ
    //³õÊ¼imuÆ«²îµÄ²»È·¶¨ÐÔ deg/sec  INIT_ACCEL_BIAS_UNCERTAINTY 0.5f
    P[15][15] = sq(INIT_ACCEL_BIAS_UNCERTAINTY * dtEkfAvg);
    // earth magnetic field  µØ´Å³¡µØÀíÏµÏÂÐ­·½²î
    P[16][16] = 0.0f;
    P[17][17] = P[16][16];
    P[18][18] = P[16][16];
    // body magnetic field  ÔØÌåÏµÏÂµØ´Å³¡Ð­·½²î
    P[19][19] = 0.0f;
    P[20][20] = P[19][19];
    P[21][21] = P[19][19];
    // wind velocities   ·çËÙÐ­·½²î
    P[22][22] = 0.0f;
    P[23][23]  = P[22][22];

    // optical flow ground height covariance ¹âÁ÷¸ß¶ÈÐ­·½²î
    Popt = 0.25f;
}

/********************************************************
*                 UPDATE FUNCTIONS                      *
********************************************************/
// Update Filter States - this should be called whenever new IMU data is available
//¸üÐÂÂË²¨Æ÷×´Ì¬£¬Ö»ÒªÓÐÐÂµÄIMUÊý¾Ý¿ÉÓÃ
void NavEKF2_core::UpdateFilter(bool predict)
{
    // Set the flag to indicate to the filter that the front-end has given permission for a new state prediction cycle to be started
    //ÉèÖÃ±êÖ¾Î»ÓÃÀ´ÏòEKF2ÂË²¨Æ÷£¬ÈôÇ°¶ËÒÑ¾­ÔÊÐíÆô¶¯ÐÂµÄ×´Ì¬Ô¤²âÖÜÆÚ£¬Ôò¸ÃÖµÎªtrue
    //×¢Òâ±äÁ¿ÃüÁî 
	startPredictEnabled = predict;

    // don't run filter updates if states have not been initialised
    //Èç¹û³õÊ¼»¯×´Ì¬Î´Íê³É£¬²»ÔËÐÐÂË²¨Æ÷
	if (!statesInitialised) {
        return;
    }

    // start the timer used for load measurement
    //Æô¶¯ÓÃÓÚ¸ºÔØ²âÁ¿µÄ¶¨Ê±Æ÷
#if EK2_DISABLE_INTERRUPTS
    irqstate_t istate = irqsave();//¸ºÔØ¼ÆËã
#endif
    hal.util->perf_begin(_perf_UpdateFilter);

    // TODO - in-flight restart method ·ÉÐÐÆ÷ÖØÆô²ßÂÔ

    //get starting time for update step
    //»ñÈ¡¸üÐÂ²½ÖèµÄ¿ªÊ¼Ê±¼ä   ÔËÐÐ×¼±¸  const NAVEKF2 *frontend
    imuSampleTime_ms = frontend->imuSampleTime_us / 1000;

    // Check arm status and perform required checks and mode changes
    //¼ì²éÒ£¿ØÆ÷ÊÇ·ñ½âËøµç»úºÍÖ´ÐÐ±ØÒªµÄ¼ì²éºÍÄ£Ê½
    controlFilterModes();

    // read IMU data as delta angles and velocities
    //¶ÁÈ¡IMUÊý¾Ý×÷Îª½Ç¶ÈºÍËÙ¶È
    readIMUData();

    // Run the EKF equations to estimate at the fusion time horizon if new IMU data is available in the buffer
    //Èç¹ûÔÚ»º³åÇø´æÔÚÐÂµÄIMUÊý¾Ý£¬ÔÚÂú×ãµÄÈÚºÏÊ±¼äÄÚ£¬ÔËÐÐEKF
    //²»Í¬ÈÚºÏÄ£Ê½ÏÂ×´Ì¬·½²îºÍÁ¿²â·½³Ì¡¢×´Ì¬Ä£ÐÍ¸÷ÊÇÊ²Ã´?
    //¶àÖÖ´«¸ÐÆ÷½øÐÐÈÚºÏ£¬Èç¹û·Ç·ÖÉ¢ÂË²¨»òÁª°îÂË²¨£¬Ôò´¦Àí·½Ê½?
	if (runUpdates) {//boolen true when EKF can run

		// Predict states using IMU data from the delayed time horizon
        //Ô¤²â·½³Ì:´ÓÑÓ³Ù³ß¶ÈÊ±¼ä£¬Ê¹ÓÃIMUÊý¾Ý  EKFµÚÒ»¸ö·½³Ì
        //¸üÐÂµ¼º½ÏÂµÄÎ»ÖÃ¡¢ËÙ¶È  ´¿¹ßµ¼¼ÆËã
        //×´Ì¬Ò»²½Ô¤²âÓ¦¸üÐÂËùÓÐ×´Ì¬±äÁ¿?ÕâÀïÎªºÎÖ»¸üÐÂÁËÎ»ÖÃ¡¢ËÙ¶È?
		UpdateStrapdownEquationsNED();

        // Predict the covariance growth    EKFµÄµÚ¶þ¸ö·½³Ì
        //Ô¤²âÐ­·½²îÔö³¤ Ö»ÊÇÒ»²½Ô¤²âµÄ¾ù·½Îó²î
        CovariancePrediction();

        // Update states using  magnetometer data
        //Ê¹ÓÃµØ´Å¸üÐÂÂË²¨Æ÷×´Ì¬
        SelectMagFusion();

        // Update states using GPS and altimeter data

		//Ê¹ÓÃGPSºÍ¼ÓËÙ¶È¼Æ½øÐÐ×´Ì¬¸üÐÂ
        SelectVelPosFusion();

        // Update states using range beacon data
        //Ê¹ÓÃ²â¾àÒÇ½øÐÐÂË²¨Æ÷×´Ì¬¸üÐÂ
        SelectRngBcnFusion();

        // Update states using optical flow data
        //Ê¹ÓÃ¹âÁ÷¸üÐÂÊý¾Ý
        SelectFlowFusion();

        // Update states using airspeed data
        //Ê¹ÓÃ¿ÕËÙ¼ÆÊý¾Ý¸üÐÂÊý¾Ý
        SelectTasFusion();

        // Update states using sideslip constraint assumption for fly-forward vehicles
        //¸üÐÂÓ¦ÓÃ²à»¬Ô¼Êø¼ÓÉÏµÄ·ÉÔ½·ÉÐÐÆ÷×´Ì¬
        SelectBetaFusion();

        // Update the filter status¡¢
        //¸üÐÂÂË²¨Æ÷×´Ì¬     ÒÔÉÏÎªEKFµÄµÚÈý¸ö·½³Ì£¬¸üÐÂ×´Ì¬
        updateFilterStatus();
    }

    // Wind output forward from the fusion to output time horizon
    //´ÓÈÚºÏÊý¾Ýµ½Êý¾ÝÊä³ö EKFµÄµÚ4¡/5¸ö·½³Ì ¼ÆËãÎó²î¡¢ÔöÒæ¡¢×´Ì¬Êä³ö¡¢¸üÐÂÐ­·½²îÕó
    calcOutputStates();

    // stop the timer used for load measurement
    //Í£Ö¹ÓÃÓÚ¸ºÔØ²âÁ¿µÄ¼ÆÊ±Æ÷
    hal.util->perf_end(_perf_UpdateFilter);
#if EK2_DISABLE_INTERRUPTS
    irqrestore(istate);
#endif
}
//½«½Ç¶ÈÔöÁ¿*±ê¶ÈÒòÊý-ÁãÆ«
//ÐÎ²ÎÎªÒýÓÃ£¬½«ÐÞÕýºóµÄÊý¾ÝÍ¨¹ýdelAng´«³öÈ¥£¬·µ»ØÀàÐÍÎªvoid
void NavEKF2_core::correctDeltaAngle(Vector3f &delAng, float delAngDT)
{
    delAng.x = delAng.x * stateStruct.gyro_scale.x;
    delAng.y = delAng.y * stateStruct.gyro_scale.y;
    delAng.z = delAng.z * stateStruct.gyro_scale.z;
	//ÏòÁ¿Ö®¼äµÄÔËËãÒÑ¾­Í¨¹ýÖØÔØ²Ù×÷·ûÍê³ÉÁË£¬¹Ê¿ÉÖ±½ÓÏà³Ë¡¢Êý³ý
	//delAngDTÊÇ¾Ö²¿±äÁ¿£¬ÆäÊµ¾ÍÊÇimuDataDownSampleNEw.delAngDT
	//ÍÓÂÝÊý¾Ý - Æ«ÒÆ
    delAng -= stateStruct.gyro_bias * (delAngDT / dtEkfAvg);//dtEkfAvg EKFÁ½´Î¸üÐÂÖ®¼äµÄÆÚÍûÊ±¼ä¼ä¸ô
}
//ÐÞÕýzÖáÆ¯ÒÆÓ°Ïì ÎªºÎÖ»ÐÞÕýzÖá
void NavEKF2_core::correctDeltaVelocity(Vector3f &delVel, float delVelDT)
{
    delVel.z -= stateStruct.accel_zbias * (delVelDT / dtEkfAvg);
}

/*
 * Update the quaternion, velocity and position states using delayed IMU measurements
 * because the EKF is running on a delayed time horizon. Note that the quaternion is
 * not used by the EKF equations, which instead estimate the error in the attitude of
 * the vehicle when each observtion is fused. This attitude error is then used to correct
 * the quaternion.
*/
//¸üÐÂËÄÔªËØ¡¢ËÙ¶È¡¢Î»ÖÃµÄ×´Ì¬ with delayed IMU measurement 
//ÒòÎªEKFÔÚÒ»¸öÑÓ³ÙµÄÊ±¼ä³ß¶ÈÉÏÔËÐÐ¡£ËÄÔªËØ²»ÊÇÓÃÀ´ÔÚEKF·½³ÌÖÐ£¬¶øÊÇµ±Íâ²¿Á¿²âÈÚºÏÊ±¹À¼ÆÔØÌå×ËÌ¬Îó²î
//×ËÌ¬Îó²îÖ®ºó±»ÓÃÀ´ÐÞÕýËÄÔªËØ
//¸üÐÂÓÉ¹ßµ¼±¾Éí¼ÆËãµÄÎ»ÖÃ¡¢ËÙ¶È£¬¿ÉÒÔ¿´×öÒ»²½Ô¤²â
//Õý³£×´Ì¬Ò»²½Ô¤²âÓ¦¸üÐÂËùÓÐµÄ×´Ì¬±äÁ¿£¬ÕâÀïÖ»ÊÇ¸üÐÂÁË¹ßµ¼½âËãµÄÎ»ÖÃ¡¢ËÙ¶È
void NavEKF2_core::UpdateStrapdownEquationsNED()
{
    // update the quaternion states by rotating from the previous attitude through
    // the delta angle rotation quaternion and normalise
    // apply correction for earth's rotation rate
    // % * - and + operators have been overloaded ²Ù×÷ÔËËã·ûÖ®Ç°ÒÑ±»ÖØÔØ
    //Í¨¹ýÖ®Ç°µÄ×ËÌ¬Ðý×ª¸üÐÂËÄÔªËØ£¬²¢¹éÒ»»¯ÓÃÓÚÐÞÕýµØÇòÐý×ªËÙ¶ÈÊ¸Á¿£
    //prevTnbÉÏÒ»Ê±¿ÌµÄCnb  earthRateNED µØÇòÐý×ªµÄ½ÇËÙÂÊÊ¸Á¿£¬¼´wieÔÚnÏµµÄ·Ö½â
    //wien = [wie*cosf(lat) 0  -wie * sinf(lat)] ÔÚNED×ø±êÏµºÍÊéÉÏENUÓÐÇø±ð
    //¶ÔÓÚmems£¬¼¸ºõÃô¸Ð²»µ½wie£¬¹Ê¿ÉÒÔºöÂÔ
    //stateStructÎª28Î¬µÄ½á¹¹Ìå£¬quatµÄÀàÐÍÊÇQuaternion£¬rotate()¸ù¾Ý½ÇÔöÁ¿¸üÐÂËÄÔªÊý
    //delAngCorrected ÐÞÕýºóµÄimuÔöÁ¿ Wibb * dt - Cnb * Wien * dt£¬µÃµ½Wnbb
    //ÎªÊ²Ã´Ã»ÓÃµ½ÔØÌåµÄËÙ¶È???memsÄÜÃô¸Ðµ½WieÂð
    stateStruct.quat.rotate(delAngCorrected - prevTnb * earthRateNED*imuDataDelayed.delAngDT);
    stateStruct.quat.normalize();//ËÄÔªËØ¹éÒ»»¯

    // transform body delta velocities to delta velocities in the nav frame
    // use the nav frame from previous time step as the delta velocities
    // have been rotated into that frame
    // * and + operators have been overloaded
    //½«ËÙ¶ÈÊ¸Á¿µÄdeltaÖµ´Ó±¾ÌåÏµ×ª»»µ½µ¼º½Ïµ
    //prevTnb ÉÏÒ»Ê±¿ÌCnb
    //ºöÂÔÁË²æ³ËµÄÓ°Ïì£¬ÓÉÓÚ×ø±êÏµÊÇNED£¬¹Ê+9.8,ÈôÑ¡ÔñNEU£¬Ôò-9.8
    Vector3f delVelNav;  // delta velocity vector in earth axes µØÀíÏµÏÂËÙ¶ÈÊ¸Á¿µÄdeltaÖµ
    delVelNav  = prevTnb.mul_transpose(delVelCorrected);//¼´CnbµÄ×ªÖÃ * delVelCorrected,½«ÐÞÕýºóacc * dt ×ª»»µ½nÏµ
    delVelNav.z += GRAVITY_MSS*imuDataDelayed.delVelDT;//È¥µôÖØÁ¦Ó°Ïì GRAVITY_MSS = 9.8

    // calculate the body to nav cosine matrix ¼ÆËã±¾ÌåÏµµ½µ¼º½ÏµµÄÓàÏÒ¾ØÕó
    //Ê×ÏÈÓÉ×ËÌ¬¾ØÕó·µ»ØËÄÔªÊý£¬ËÄÔªÊýÇóÄæÔÙ·µ»Ø
    //inverse()/rotation_matrix(prevTnb)¶¼ÊÇÀàQuaternionÖÐ³ÉÔ±º¯Êý,inverse()ÊÇËÄÔªËØÇóÄæ²¢·µ»ØÒ»¸öQuaternionÀà
    //raotation_matrix(prevTnb),ÀûÓÃËÄÔªËØÇó×ËÌ¬¾ØÕóCnb£¬ÆäÖÐËÄÔªËØÍ¨¹ýthisÖ¸ÕëÒþÊ½´«µÝ
    //stateStruct.quatÊÇÒ»¸öQuaternionÀà£¬ÎªÊ²Ã´ÏÈ¶ÔËÄÔªÊý½øÐÐÇóÄæÖ®ºóÔÙÇóCnb
    //²»¶ÔËÄÔªËØÇóÄæ£¬Ö±½ÓÇó³öÀ´ÊÇCbn,ÇóÄæÖ®ºó£¬Ðý×ª·½Ïò±äÁË£¬´ËÊ±ÎªCnb
    //stateStruct.quatÊÇÒ»¸öÀà¶ÔÏó£¬stateStruct.quat.inverse()·µ»ØÒ»¸öËÄÔªËØµÄÄæ£¬
    //stateStruct.quat.inverse.rotation_matrixÊÇ½«ÇóÄæºóµÄËÄÔªËØ×ª»»Îª×ËÌ¬¾ØÕóCnb
    //Ö±½Ó¶ÔCbnÇó×ªÖÃÆäÊµ¾ÍÊÇCnb£¬ÊµÏÖ±ØÐë¶Ô×ËÌ¬¾ØÕó¹éÒ»»¯
    stateStruct.quat.inverse().rotation_matrix(prevTnb);

    // calculate the rate of change of velocity (used for launch detect and other functions)
    //¼ÆËãËÙ¶ÈµÄ±ä»¯ÂÊ ÆäÊµµÈÐ§ÓÚµ¼º½ÏµÏÂ¼Ó¼ÆÊý¾Ý
	velDotNED = delVelNav / imuDataDelayed.delVelDT;

    // apply a first order lowpass filter Ò»½×µÍÍ¨ÂË²¨´¦Àí
    velDotNEDfilt = velDotNED * 0.05f + velDotNEDfilt * 0.95f;

    // calculate a magnitude of the filtered nav acceleration (required for GPS
    // variance estimation)
    //¼ÆËãÂË²¨ºóµÄµ¼º½ÏµÏÂ¼ÓËÙ¶È·ùÖµ(GPS·½²î¹À¼Æ»áÓÃµ½¸ÃÖµ)
    //nÏµÖÐË®Æ½¼Ó¼Æ½øÐÐ¹éÒ»»¯´¦Àí
    accNavMag = velDotNEDfilt.length();//µ¼º½ÏµÏÂÈýÖá¼Ó¼ÆµÄÄ£
    accNavMagHoriz = norm(velDotNEDfilt.x , velDotNEDfilt.y);//¶Ôµ¼º½ÏµÏÂË®Æ½¼Ó¼Æ½øÐÐÆ½·½ÔÙ¿ª·½

    // if we are not aiding, then limit the horizontal magnitude of acceleration
    // to prevent large manoeuvre transients disturbing the attitude
    // ÏÞÖÆ¼ÓËÙ¶ÈµÄË®Æ½´óÐ¡£¬·ÀÖ¹´ó»ú¶¯Ë³±ã¸ÉÈÅ×ËÌ¬
    //PV_AidingMode ÈÚºÏins¹À¼ÆÎ»ÖÃ¡¢ËÙ¶ÈµÄÊ×Ñ¡Ä£Ê½ AID_NONE Ã»ÓÐ¸¨Öú£¬½ö½ö×ËÌ¬ºÍ¸ß¶È¹À¼ÆÊÇ¿ÉÓÃµÄ
    if ((PV_AidingMode == AID_NONE) && (accNavMagHoriz > 5.0f)) {
        float gain = 5.0f/accNavMagHoriz;
        delVelNav.x *= gain;
        delVelNav.y *= gain;
    }

    // save velocity for use in trapezoidal integration for position calcuation
    //±£´æËÙ¶ÈÓÃÓÚÌÝÐÎ»ý·ÖµÄÎ»ÖÃ¼ÆËã
    Vector3f lastVelocity = stateStruct.velocity;

    // sum delta velocities to get velocity ¼ÆËãËÙ¶È
    stateStruct.velocity += delVelNav;

    // apply a trapezoidal integration to velocities to calculate position
    //ÀûÓÃËÙ¶ÈµÄÌÝÐÎ»ý·Ö¼ÆËãÎ»ÖÃ
    stateStruct.position += (stateStruct.velocity + lastVelocity) * (imuDataDelayed.delVelDT*0.5f);

    // accumulate the bias delta angle and time since last reset by an OF measurement arrival
    //´ÓÉÏ´Î¸´Î»ºó£¬ÀÛ»ý½Ç¶ÈÆ«ÒÆµÄdeltaÖµºÍÊ±¼ä 
	delAngBodyOF += delAngCorrected;
    delTimeOF += imuDataDelayed.delAngDT;

    // limit states to protect against divergence
    //ÏÞÖÆ×´Ì¬±äÁ¿£¬·ÀÖ¹·¢É¢£¬ÀàËÆÏÞÖÆÐ­·½²îÕóPµÄ¶Ô½ÇÏßÔªËØ
    //¶Ô×´Ì¬±äÁ¿µÄ·¶Î§½øÐÐÏÞÖÆ
    ConstrainStates();
}

/*
 * Propagate PVA solution forward from the fusion time horizon to the current time horizon
 * using simple observer which performs two functions:
 * 1) Corrects for the delayed time horizon used by the EKF.
 * 2) Applies a LPF to state corrections to prevent 'stepping' in states due to measurement
 * fusion introducing unwanted noise into the control loops.
 * The inspiration for using a complementary filter to correct for time delays in the EKF
 * is based on the work by A Khosravian.
 *
 * "Recursive Attitude Estimation in the Presence of Multi-rate and Multi-delay Vector Measurements"
 * A Khosravian, J Trumpf, R Mahony, T Hamel, Australian National University
*/
//¼ÆËãÔöÒæ¡¢Êä³öÊý¾Ý¡¢¸üÐÂÐ­·½²î
//ÐÞÕýEKFËùÊ¹ÓÃµÄÊ±¼äÑÓ³Ù·¶Î§
//Ó¦ÓÃµÍÍ¨ÂË²¨½øÐÐ×´Ì¬ÐÞÕý£¬·ÀÖ¹ÓÉÓÚÁ¿²âÈÚºÏ´øÀ´µÄ²½½øÏÖÏó£¬½ø¶ø½«²»ÐèÒªµÄÔëÉùÒýÈë¿ØÖÆ»ØÂ·
//ÔÚEKFÖÐÓ¦ÓÃ»¥²¹ÂË²¨À´ÐÞÕýÊ±¼äÑÓ³Ù
//ÊµÏÖÁË»¥²¹ÂË²¨
void NavEKF2_core::calcOutputStates()
{//ÔËÐÐÆµÂÊÎª400Hz£¬Ê¹ÓÃimuÊý¾ÝÔ¤²â×ËÌ¬¡¢Î»ÖÃ¡¢ËÙ¶È
    // apply corrections to the IMU data
    //¶ÔIMUÊý¾Ý½øÐÐÐÞÕý
    Vector3f delAngNewCorrected = imuDataNew.delAng;//»ñµÃÍÓÂÝ»ý·Ö½Ç¶È
    Vector3f delVelNewCorrected = imuDataNew.delVel;//»ñµÃ¼Ó¼Æ»ý·ÖËÙ¶È
    correctDeltaAngle(delAngNewCorrected, imuDataNew.delAngDT);//Ïû³ý±ê¶ÈÒòÊýÎó²îºÍÁãÆ«Ó°Ïì
    correctDeltaVelocity(delVelNewCorrected, imuDataNew.delVelDT);//Ïû³ýzÖá¼Ó¼ÆÆ¯ÒÆ¶ÔÌìÏòËÙ¶ÈÓ°Ïì

    // apply corections to track EKF solution
    //IMU»ý·ÖµÄ½Ç¶È¼ÓÉÏekfÏÞÖÆ½Ç¶ÈµÄÐÞÕýÖµ
    //imu»ý·ÖµÄ½Ç¶È¼ÓÉÏekfÏÖÔÚ¶Ô½Ç¶ÈµÄÐÞÕýÁ¿  ÕâÀïÊÇ×ËÌ¬µÄ»¥²¹ÂË²¨
    Vector3f delAng = delAngNewCorrected + delAngCorrection;

    // convert the rotation vector to its equivalent quaternion
    //½«»ñÈ¡µÄÐý×ªÊ¸Á¿×ª»»ÎªµÈ¼ÛµÄËÄÔªËØ
    Quaternion deltaQuat;
    deltaQuat.from_axis_angle(delAng);//ÓÉ½ÇÔöÁ¿Çó³öËÄÔªËØµÄÐÞÕýÁ¿

    // update the quaternion states by rotating from the previous attitude through
    // the delta angle rotation quaternion and normalise
    //¸üÐÂËÄÔªËØ²¢¹éÒ»»¯  ÔÚÇ°Ò»´ÎÊä³öÉÏ½øÐÐÐý×ª
    //outpuDataNewÊÇoutput_elementsÀàÐÍµÄ½á¹¹Ìå£¬½á¹¹ÌåÖÐ°üº¬ËÄÔªËØ¡¢ÈýÖáËÙ¶È¡¢ÈýÖáÎ»ÖÃ
    outputDataNew.quat *= deltaQuat;//¸üÐÂËÄÔªËØ£¬ËÄÔªËØÏà³Ë£¬µÃµ½µ±Ç°Ê±¿ÌµÄËÄÔªËØ
    outputDataNew.quat.normalize();//ËÄÔªËØ¹éÒ»»¯

    // calculate the body to nav cosine matrix
    //¼ÆËãCbn ÔØÌåÏµµ½µ¼º½Ïµ×ª»»¾ØÕó 
    Matrix3f Tbn_temp;
    outputDataNew.quat.rotation_matrix(Tbn_temp);//ÓÉÐÞÕý¹ýµÄËÄÔªËØ¸üÐÂ×ËÌ¬¾ØÕóCbn

    // transform body delta velocities to delta velocities in the nav frame
    //½«ÔØÌåÏµµÄËÙ¶ÈÀÛ»ý×ª»»µ½µ¼º½Ïµ
    Vector3f delVelNav  = Tbn_temp*delVelNewCorrected;//½«ËÙ¶ÈÔöÁ¿ÐÞÕýÁ¿×ª»»µ½nÏµ
    delVelNav.z += GRAVITY_MSS*imuDataNew.delVelDT;

    // save velocity for use in trapezoidal integration for position calcuation
    //±£´æËÙ¶ÈÒÔÓÃÔÚÎ»ÖÃ¼ÆËãµÄÌÝÐÎ»ý·ÖÖÐ   ±£´æÉÏÒ»´ÎËÙ¶È
    Vector3f lastVelocity = outputDataNew.velocity;

    // sum delta velocities to get velocity
    //½«ÀÛ»ýµÄËÙ¶ÈÔöÁ¿ÇóºÍÒÔµÃµ½µ±Ç°ËÙ¶È
    outputDataNew.velocity += delVelNav;

    // apply a trapezoidal integration to velocities to calculate position
    //¶ÔËÙ¶ÈÓ¦ÓÃÌÝÐÎ»ý·Ö¼ÆËãµ±Ç°Î»ÖÃ ÌÝÐÎ»ý·Ö:ÉÏÊ±¿ÌËÙ¶È+¸ÃÊ±¿ÌËÙ¶ÈµÄºÍµÄÆ½¾ù
    //ÂË²¨ºó¶ÔÎ»ÖÃ½øÐÐÐ£Õý
    outputDataNew.position += (outputDataNew.velocity + lastVelocity) * (imuDataNew.delVelDT*0.5f);

    // If the IMU accelerometer is offset from the body frame origin, then calculate corrections
    // that can be added to the EKF velocity and position outputs so that they represent the velocity
    // and position of the body frame origin.
    // Note the * operator has been overloaded to operate as a dot product
    //Èç¹ûIMUµÄ¼Ó¼Æ°²×°²½Öè³µÌåÔ­µã£¬Ôò¼ÆËãÕâ¸öÐÞÕýÁ¿£¬È»ºóÌí¼Óµ½Î»ÖÃ¡¢ËÙ¶ÈÊä³öÉÏ£¬ÒÔ±ãÓÚIMU¼ÆËãµÄÎ»ÖÃ
    //ºÍËÙ¶È¿ÉÒÔ´ú±í³µÌåÔ­µãµÄÎ»ÖÃºÍËÙ¶È¡£
    //*ÒÑ¾­±»ÖØÔØÎªµã»ý(ÏòÁ¿¶ÔÓ¦ÔªËØÏà³Ë) ²æ»ý:ÔËËã½á¹ûÊÇÒ»¸öÏòÁ¿£¬¸ÃÏòÁ¿´¹Ö±ÓÚÁ½¸öÏòÁ¿µÄºÍ
    //´¦ÀíimuÊÇ·ñÓÐÎ»ÖÃÆ«ÒÆ£¬3.5°æ±¾¿ÉÒÔ°ÑimuÎ»ÖÃÆ«ÒÆµ½ÖØÐÄÉÏ
    if (!accelPosOffset.is_zero()) {//caaelPosOffset ÊÇimuÖÐ¼ÓËÙ¶È¼ÆÔÚÔØÌåÏµÖÐÎ»ÖÃ is_zero()ÅÐ¶ÏÈý¸öÔªËØÊÇ·ñ¶¼ÎªÁã
        // calculate the average angular rate across the last IMU update
        // note delAngDT is prevented from being zero in readIMUData()
        //¼ÆËãÉÏ´ÎIMU¸üÐÂÆÚ¼äÆ½¾ùµÄ½ÇËÙÂÊ£¬ÆäÊµ¾ÍÊÇÉÏÒ»ÂË²¨ÖÜÆÚÖÐÍÓÂÝÊý¾ÝµÄÆ½¾ù
        //noet:delAngDt±»×èÖ¹ÎªÁãÔÚreadIMUData()ÖÐ
        Vector3f angRate = imuDataNew.delAng * (1.0f/imuDataNew.delAngDT);

        // Calculate the velocity of the body frame origin relative to the IMU in body frame
        // and rotate into earth frame. Note % operator has been overloaded to perform a cross product
        //¼ÆËãÔØÌåÏÂÖÐÔØÌåÔ­µãÓëIMUÖ®¼äµÄÏà¶ÔËÙ¶È£¬²¢Ðý×ªµ½µØÀíÏµÏÂ
        //%ÒÑ±»ÖØÔØÎª²æ»ý  angRateÆ½¾ù×ª¶¯½ÇËÙÂÊ
        Vector3f velBodyRelIMU = angRate % (- accelPosOffset);//accelPosOffset ¼Ó¼ÆÔÚ±¾ÌåÏÂÖÐµÄÎ»ÖÃ
        velOffsetNED = Tbn_temp * velBodyRelIMU;//velOffsetNED ±»¼Óµ½µ¼º½ÏµÏÂimuµÄËÙ¶È¹À¼ÆÉÏ£¬ÒÔ±ãµÃµ½ÔØÌåÔ­µãµÄËÙ¶È

        // calculate the earth frame position of the body frame origin relative to the IMU
        //¼ÆËãÔØÌåÖÐimuÎ»ÖÃÓëÔØÌåÔ­µãÖ®¼äµÄ¾àÀë
        posOffsetNED = Tbn_temp * (- accelPosOffset);
    } else {
        velOffsetNED.zero();
        posOffsetNED.zero();
    }
    //²»Ê¹ÓÃEKF£¬»¥²¹ÂË²¨£¬µ½ÕâÀï×ËÌ¬¡¢ËÙ¶È¡¢Î»ÖÃµÄÔ¤²âÒÑ¾­Íê³É
    // store INS states in a ring buffer that with the same length and time coordinates as the IMU data buffer
    //°ÑÐÂµÄÊý¾Ý´æ´¢µ½»·ÐÎ»º³åÇøbuf£¬Ã¿ÔËÐÐÒ»´Î£¬±£´æÒ»´Î¡£
	if (runUpdates) {
        // store the states at the output time horizon
        //´æ´¢µ±Ç°×´Ì¬µ½»·ÐÎbuffÖÐ£¬Ö»ÓÐÔËÐÐÒ»´Î²Å»á±£´æÒ»´Î
        storedOutput[storedIMU.get_youngest_index()] = outputDataNew;

        // recall the states from the fusion time horizon
        //»ñÈ¡×îÔçµÄÒ»´Î±£´æ£¬Ê±¼ä²îÔÚ0.02sÒÔÄÚ
        outputDataDelayed = storedOutput[storedIMU.get_oldest_index()];
        
        // compare quaternion data with EKF quaternion at the fusion time horizon and calculate correction
        //½«ËÄÔªÊýÊý¾ÝºÍEKFËÄÔªÊýÔÚÈÚºÏÊ±¼äÓòÉÏ½øÐÐ±È½Ï£¬²¢¼ÆËãÐÞÕýËùÐèËÄÔªËØµÄ¹À¼ÆÖµ£¬µÃµ½Îó²î
        // divide the demanded quaternion by the estimated to get the error
        //Çó³öekf×ËÌ¬ÓëÊµ¼ù×î¾ÃµÄ×ËÌ¬²îËÄÔªÊý£¬¾ÍÊÇÐý×ªµÄ½Ç¶ÈÓÃËÄÔªËØ±íÊ¾
        //ËÄÔªËØÏà³ýÆäÊµ¾ÍÊÇ±íÊ¾ÕâÁ½¸öËÄÔªËØÖ®¼äµÄ½Ç¶ÈÎó²î£¬ËÄÔªËØ´ú±íÐý×ª£¬Ïàµ±ÓÚÔÚµ±Ç°»ù´¡ÉÏ
        //ÑØÏà·´·½ÏòÐý×ªÒ»¸ö½Ç¶È£¬´ËÊ±µÃµ½µÄ¾ÍÊÇÁ½¸öËÄÔªËØÖ®¼äµÄ²îÖµ
        Quaternion quatErr = stateStruct.quat / outputDataDelayed.quat;

        // Convert to a delta rotation using a small angle approximation
        //Ê¹ÓÃÐ¡½Ç¶ÈÐý×ª½üËÆ£¬ÇóËÄÔªËØµÄÐý×ªÓÃÒ»¸öÐ¡½Ç¶È½üËÆ£¬µ±theta½ÇºÜÐ¡Ê±£¬
        //theta = sin(theta) 
        quatErr.normalize();
        Vector3f deltaAngErr;
        float scaler;
        if (quatErr[0] >= 0.0f) {
            scaler = 2.0f;
        } else {
            scaler = -2.0f;
        }
        deltaAngErr.x = scaler * quatErr[1];//Ð¡½Ç¶È½üËÆ£¬½«ËÄÔªÊý×ª»»Îª×ËÌ¬½ÇÎó²î ÔÝÊ±²»Ã÷°×ÈçºÎ×ª»»?
        deltaAngErr.y = scaler * quatErr[2];
        deltaAngErr.z = scaler * quatErr[3];

        // calculate a gain that provides tight tracking of the estimator states and
        // adjust for changes in time delay to maintain consistent damping ratio of ~0.7
        //¼ÆËã×èÄá±È 1/2 * (dtIMUavg/timeDelay)
        float timeDelay = 1e-3f * (float)(imuDataNew.time_ms - imuDataDelayed.time_ms);
        timeDelay = fmaxf(timeDelay, dtIMUavg);
        float errorGain = 0.5f / timeDelay;

        // calculate a corrrection to the delta angle
        // that will cause the INS to track the EKF quaternions
        //¼ÆËã×ËÌ¬µÄÐÞÕýÖµ£¬ ÔÚ 400hz µÄÑ­»·ÀïÓÃÕâ¸öÖµ
        delAngCorrection = deltaAngErr * errorGain * dtIMUavg;

        // calculate velocity and position tracking errors
        //¼ÆËãÎ»ÖÃºÍËÙ¶ÈµÄ²î
        Vector3f velErr = (stateStruct.velocity - outputDataDelayed.velocity);
        Vector3f posErr = (stateStruct.position - outputDataDelayed.position);

        // collect magnitude tracking error for diagnostics Îó²îÊÕ¼¯
        //ÕâÀïÍ¨¹ýÉèÖÃÊ±¼ä³£ÊýÓëEKFÆ½¾ùÔËÐÐµÄÊ±¼äÏà±È£¬Çó³öÒ»¸öÔöÒæ£¬ÓÃÕâ¸öÔöÒæµÄ¶þ´Î
        //º¯Êý¶ÔËÙ¶ÈºÍÎ»ÖÃ²îµÄ»ý·Ö½øÐÐ´¦Àí£¬µÃµ½ËÙ¶ÈÎ»ÖÃÐÞÕýÖµ¡£ÕâÀïÓ¦¸ÃÓÐÓÃÔ­Ê¼Êý¾Ý
        //·Å×ÅÄãÄÇ¹ý 
        outputTrackError.x = deltaAngErr.length();
        outputTrackError.y = velErr.length();
        outputTrackError.z = posErr.length();

        // convert user specified time constant from centi-seconds to seconds
        //½«ÓÃ»§Ö¸¶¨µÄÊ±¼ä³£Êý´ÓÃëÃë×ª»»ÎªÃë
        float tauPosVel = constrain_float(0.01f*(float)frontend->_tauVelPosOutput, 0.1f, 0.5f);

        // calculate a gain to track the EKF position states with the specified time constant
        //¼ÆËãÔöÒæÒÔÖ¸¶¨µÄÊ±¼ä³£Êý¸ú×ÙEKFÎ»ÖÃ×´Ì¬
        float velPosGain = dtEkfAvg / constrain_float(tauPosVel, dtEkfAvg, 10.0f);

        // use a PI feedback to calculate a correction that will be applied to the output state history
        //ÓÃPI·´À¡¼ÆËãÒ»¸öÐÞÕý£¬¸ÃÐÞÕýÁ¿±»ÓÃÓÚÊä³öÀúÊ·µÄÐ£Õý
        posErrintegral += posErr;
        velErrintegral += velErr;
        Vector3f velCorrection = velErr * velPosGain + velErrintegral * sq(velPosGain) * 0.1f;
        Vector3f posCorrection = posErr * velPosGain + posErrintegral * sq(velPosGain) * 0.1f;

        // loop through the output filter state history and apply the corrections to the velocity and position states
        // this method is too expensive to use for the attitude states due to the quaternion operations required
        // but does not introduce a time delay in the 'correction loop' and allows smaller tracking time constants
        // to be used
        //Ñ­»·Êä³öÂË²¨Æ÷×´Ì¬ÀúÊ·¼ÇÂ¼£¬²¢½«ÐÞÕýÁ¿ÓÃÓÚÎ»ÖÃ¡¢ËÙ¶È£»ÕâÖÖ·½·¨¶ÔËÄÔªËØÄÑÒÔÓ¦ÓÃ£¬ÒòÎªËÄÔªËØ²»½öÐèÒªÔÚÐÞÕýÑ­»·
        //ÖÐÓÐ¸ötime delay£¬¶øÇÒÔÊÐí½ÏÐ¡µÄ¸ú×ÙÊ±¼ä³£Êý
        output_elements outputStates;
        for (unsigned index=0; index < imu_buffer_length; index++) {
            outputStates = storedOutput[index];

            // a constant  velocity correction is applied
            //³ÖÐøµÄËÙ¶ÈÐÞÕý
            outputStates.velocity += velCorrection;

            // a constant position correction is applied
            //³ÖÐøµÄÎ»ÖÃÐÞÕý
            outputStates.position += posCorrection;

            // push the updated data to the buffer
            //½«ÐÞÕýºóµÄÊý¾ÝÍÆµ½bufÖÐ
            storedOutput[index] = outputStates;
        }

        // update output state to corrected values  ½«Êä³ö×´Ì¬¸üÐÂÎªÐ£ÕýºóµÄÖµ
        outputDataNew = storedOutput[storedIMU.get_youngest_index()];

    }
}

/*
 * Calculate the predicted state covariance matrix using algebraic equations generated with Matlab symbolic toolbox.
 * The script file used to generate these and other equations in this filter can be found here:
 * https://github.com/priseborough/InertialNav/blob/master/derivations/RotationVectorAttitudeParameterisation/GenerateNavFilterEquations.m
*/
//ÀûÓÃMATLAB·ûºÅ¹¤¾ßÏä²úÉúµÄ´úÊý·½³ÌÀ´¼ÆËãÒ»²½Ô¤²âµÄ·½²îÕó
void NavEKF2_core::CovariancePrediction()
{
    hal.util->perf_begin(_perf_CovariancePrediction);
    float windVelSigma; // wind velocity 1-sigma process noise - m/s
    float dAngBiasSigma;// delta angle bias 1-sigma process noise - rad/s
    float dVelBiasSigma;// delta velocity bias 1-sigma process noise - m/s
    float dAngScaleSigma;// delta angle scale factor 1-Sigma process noise
    float magEarthSigma;// earth magnetic field 1-sigma process noise
    float magBodySigma; // body magnetic field 1-sigma process noise
    float daxNoise;     // X axis delta angle noise variance rad^2
    float dayNoise;     // Y axis delta angle noise variance rad^2
    float dazNoise;     // Z axis delta angle noise variance rad^2
    float dvxNoise;     // X axis delta velocity variance noise (m/s)^2
    float dvyNoise;     // Y axis delta velocity variance noise (m/s)^2
    float dvzNoise;     // Z axis delta velocity variance noise (m/s)^2
    float dvx;          // X axis delta velocity (m/s)
    float dvy;          // Y axis delta velocity (m/s)
    float dvz;          // Z axis delta velocity (m/s)
    float dax;          // X axis delta angle (rad)
    float day;          // Y axis delta angle (rad)
    float daz;          // Z axis delta angle (rad)
    float q0;           // attitude quaternion
    float q1;           // attitude quaternion
    float q2;           // attitude quaternion
    float q3;           // attitude quaternion
    float dax_b;        // X axis delta angle measurement bias (rad)
    float day_b;        // Y axis delta angle measurement bias (rad)
    float daz_b;        // Z axis delta angle measurement bias (rad)
    float dax_s;        // X axis delta angle measurement scale factor
    float day_s;        // Y axis delta angle measurement scale factor
    float daz_s;        // Z axis delta angle measurement scale factor
    float dvz_b;        // Z axis delta velocity measurement bias (rad)

    // calculate covariance prediction process noise
    // use filtered height rate to increase wind process noise when climbing or descending
    // this allows for wind gradient effects.
    // filter height rate using a 10 second time constant filter Ê²Ã´ÊÇ³£Á¿ÂË²¨Æ÷?????????
    //¼ÆËãÐ­·½²îÔ¤²â¹ý³ÌÔëÉù
    //Ê¹ÓÃ¹ýÂË¸ß¶ÈÂÊÀ´Ôö¼ÓÅÊÅÀ»òÏÂ½µÊ±µÄ·ç¹ý³ÌÔëÒô,ÕâÔÊÐí·çÌÝ¶ÈÐ§Ó¦
    //Ê¹ÓÃ10ÃëÊ±¼ä³£ÊýÂË²¨Æ÷¹ýÂË¸ß¶È
    dt = imuDataDelayed.delAngDT;
    float alpha = 0.1f * dt;
    hgtRate = hgtRate * (1.0f - alpha) - stateStruct.velocity.z * alpha;

    // use filtered height rate to increase wind process noise when climbing or descending
    // this allows for wind gradient effects.
    windVelSigma  = dt * constrain_float(frontend->_windVelProcessNoise, 0.0f, 1.0f) * (1.0f + constrain_float(frontend->_wndVarHgtRateScale, 0.0f, 1.0f) * fabsf(hgtRate));
    dAngBiasSigma = sq(dt) * constrain_float(frontend->_gyroBiasProcessNoise, 0.0f, 1.0f);
    dVelBiasSigma = sq(dt) * constrain_float(frontend->_accelBiasProcessNoise, 0.0f, 1.0f);
    dAngScaleSigma = dt * constrain_float(frontend->_gyroScaleProcessNoise, 0.0f, 1.0f);
    magEarthSigma = dt * constrain_float(frontend->_magEarthProcessNoise, 0.0f, 1.0f);
    magBodySigma  = dt * constrain_float(frontend->_magBodyProcessNoise, 0.0f, 1.0f);
    for (uint8_t i= 0; i<=8;  i++) processNoise[i] = 0.0f;
    for (uint8_t i=9; i<=11; i++) processNoise[i] = dAngBiasSigma;
    for (uint8_t i=12; i<=14; i++) processNoise[i] = dAngScaleSigma;
    if (expectGndEffectTakeoff) {
        processNoise[15] = 0.0f;
    } else {
        processNoise[15] = dVelBiasSigma;
    }
    for (uint8_t i=16; i<=18; i++) processNoise[i] = magEarthSigma;
    for (uint8_t i=19; i<=21; i++) processNoise[i] = magBodySigma;
    for (uint8_t i=22; i<=23; i++) processNoise[i] = windVelSigma;

    for (uint8_t i= 0; i<=stateIndexLim; i++) processNoise[i] = sq(processNoise[i]);

    // set variables used to calculate covariance growth
    dvx = imuDataDelayed.delVel.x;
    dvy = imuDataDelayed.delVel.y;
    dvz = imuDataDelayed.delVel.z;
    dax = imuDataDelayed.delAng.x;
    day = imuDataDelayed.delAng.y;
    daz = imuDataDelayed.delAng.z;
    q0 = stateStruct.quat[0];
    q1 = stateStruct.quat[1];
    q2 = stateStruct.quat[2];
    q3 = stateStruct.quat[3];
    dax_b = stateStruct.gyro_bias.x;
    day_b = stateStruct.gyro_bias.y;
    daz_b = stateStruct.gyro_bias.z;
    dax_s = stateStruct.gyro_scale.x;
    day_s = stateStruct.gyro_scale.y;
    daz_s = stateStruct.gyro_scale.z;
    dvz_b = stateStruct.accel_zbias;
    float _gyrNoise = constrain_float(frontend->_gyrNoise, 0.0f, 1.0f);
    daxNoise = dayNoise = dazNoise = sq(dt*_gyrNoise);
    float _accNoise = constrain_float(frontend->_accNoise, 0.0f, 10.0f);
    dvxNoise = dvyNoise = dvzNoise = sq(dt*_accNoise);

    // calculate the predicted covariance due to inertial sensor error propagation
    // we calculate the upper diagonal and copy to take advantage of symmetry
    //¼ÆËãÓÉ´«¸ÐÆ÷Îó²î´«²¥ÒýÆðµÄÔ¤²âÐ­·½²î
    //¼ÆËãÖ÷¶Ô½ÇÏß²¢¸´ÖÆÀûÓÃ¶Ô³ÆÐÔ
    SF[0] = daz_b/2 - (daz*daz_s)/2;
    SF[1] = day_b/2 - (day*day_s)/2;
    SF[2] = dax_b/2 - (dax*dax_s)/2;
    SF[3] = q3/2 - (q0*SF[0])/2 + (q1*SF[1])/2 - (q2*SF[2])/2;
    SF[4] = q0/2 - (q1*SF[2])/2 - (q2*SF[1])/2 + (q3*SF[0])/2;
    SF[5] = q1/2 + (q0*SF[2])/2 - (q2*SF[0])/2 - (q3*SF[1])/2;
    SF[6] = q3/2 + (q0*SF[0])/2 - (q1*SF[1])/2 - (q2*SF[2])/2;
    SF[7] = q0/2 - (q1*SF[2])/2 + (q2*SF[1])/2 - (q3*SF[0])/2;
    SF[8] = q0/2 + (q1*SF[2])/2 - (q2*SF[1])/2 - (q3*SF[0])/2;
    SF[9] = q2/2 + (q0*SF[1])/2 + (q1*SF[0])/2 + (q3*SF[2])/2;
    SF[10] = q2/2 - (q0*SF[1])/2 - (q1*SF[0])/2 + (q3*SF[2])/2;
    SF[11] = q2/2 + (q0*SF[1])/2 - (q1*SF[0])/2 - (q3*SF[2])/2;
    SF[12] = q1/2 + (q0*SF[2])/2 + (q2*SF[0])/2 + (q3*SF[1])/2;
    SF[13] = q1/2 - (q0*SF[2])/2 + (q2*SF[0])/2 - (q3*SF[1])/2;
    SF[14] = q3/2 + (q0*SF[0])/2 + (q1*SF[1])/2 + (q2*SF[2])/2;
    SF[15] = - sq(q0) - sq(q1) - sq(q2) - sq(q3);
    SF[16] = dvz_b - dvz;
    SF[17] = dvx;
    SF[18] = dvy;
    SF[19] = sq(q2);
    SF[20] = SF[19] - sq(q0) + sq(q1) - sq(q3);
    SF[21] = SF[19] + sq(q0) - sq(q1) - sq(q3);
    SF[22] = 2*q0*q1 - 2*q2*q3;
    SF[23] = SF[19] - sq(q0) - sq(q1) + sq(q3);
    SF[24] = 2*q1*q2;

    SG[0] = - sq(q0) - sq(q1) - sq(q2) - sq(q3);
    SG[1] = sq(q3);
    SG[2] = sq(q2);
    SG[3] = sq(q1);
    SG[4] = sq(q0);

    SQ[0] = - dvyNoise*(2*q0*q1 + 2*q2*q3)*(SG[1] - SG[2] + SG[3] - SG[4]) - dvzNoise*(2*q0*q1 - 2*q2*q3)*(SG[1] - SG[2] - SG[3] + SG[4]) - dvxNoise*(2*q0*q2 - 2*q1*q3)*(2*q0*q3 + 2*q1*q2);
    SQ[1] = dvxNoise*(2*q0*q2 - 2*q1*q3)*(SG[1] + SG[2] - SG[3] - SG[4]) + dvzNoise*(2*q0*q2 + 2*q1*q3)*(SG[1] - SG[2] - SG[3] + SG[4]) - dvyNoise*(2*q0*q1 + 2*q2*q3)*(2*q0*q3 - 2*q1*q2);
    SQ[2] = dvyNoise*(2*q0*q3 - 2*q1*q2)*(SG[1] - SG[2] + SG[3] - SG[4]) - dvxNoise*(2*q0*q3 + 2*q1*q2)*(SG[1] + SG[2] - SG[3] - SG[4]) - dvzNoise*(2*q0*q1 - 2*q2*q3)*(2*q0*q2 + 2*q1*q3);
    SQ[3] = sq(SG[0]);
    SQ[4] = 2*q2*q3;
    SQ[5] = 2*q1*q3;
    SQ[6] = 2*q1*q2;
    SQ[7] = SG[4];

    SPP[0] = SF[17]*(2*q0*q1 + 2*q2*q3) + SF[18]*(2*q0*q2 - 2*q1*q3);
    SPP[1] = SF[18]*(2*q0*q2 + 2*q1*q3) + SF[16]*(SF[24] - 2*q0*q3);
    SPP[2] = 2*q3*SF[8] + 2*q1*SF[11] - 2*q0*SF[14] - 2*q2*SF[13];
    SPP[3] = 2*q1*SF[7] + 2*q2*SF[6] - 2*q0*SF[12] - 2*q3*SF[10];
    SPP[4] = 2*q0*SF[6] - 2*q3*SF[7] - 2*q1*SF[10] + 2*q2*SF[12];
    SPP[5] = 2*q0*SF[8] + 2*q2*SF[11] + 2*q1*SF[13] + 2*q3*SF[14];
    SPP[6] = 2*q0*SF[7] + 2*q3*SF[6] + 2*q2*SF[10] + 2*q1*SF[12];
    SPP[7] = SF[18]*SF[20] - SF[16]*(2*q0*q1 + 2*q2*q3);
    SPP[8] = 2*q1*SF[3] - 2*q2*SF[4] - 2*q3*SF[5] + 2*q0*SF[9];
    SPP[9] = 2*q0*SF[5] - 2*q1*SF[4] - 2*q2*SF[3] + 2*q3*SF[9];
    SPP[10] = SF[17]*SF[20] + SF[16]*(2*q0*q2 - 2*q1*q3);
    SPP[11] = SF[17]*SF[21] - SF[18]*(SF[24] + 2*q0*q3);
    SPP[12] = SF[17]*SF[22] - SF[16]*(SF[24] + 2*q0*q3);
    SPP[13] = 2*q0*SF[4] + 2*q1*SF[5] + 2*q3*SF[3] + 2*q2*SF[9];
    SPP[14] = 2*q2*SF[8] - 2*q0*SF[11] - 2*q1*SF[14] + 2*q3*SF[13];
    SPP[15] = SF[18]*SF[23] + SF[17]*(SF[24] - 2*q0*q3);
    SPP[16] = daz*SF[19] + daz*sq(q0) + daz*sq(q1) + daz*sq(q3);
    SPP[17] = day*SF[19] + day*sq(q0) + day*sq(q1) + day*sq(q3);
    SPP[18] = dax*SF[19] + dax*sq(q0) + dax*sq(q1) + dax*sq(q3);
    SPP[19] = SF[16]*SF[23] - SF[17]*(2*q0*q2 + 2*q1*q3);
    SPP[20] = SF[16]*SF[21] - SF[18]*SF[22];
    SPP[21] = 2*q0*q2 + 2*q1*q3;
    SPP[22] = SF[15];

    if (inhibitMagStates) {
        zeroRows(P,16,21);
        zeroCols(P,16,21);
    } else if (inhibitWindStates) {
        zeroRows(P,22,23);
        zeroCols(P,22,23);
    }

    nextP[0][0] = daxNoise*SQ[3] + SPP[5]*(P[0][0]*SPP[5] - P[1][0]*SPP[4] + P[9][0]*SPP[22] + P[12][0]*SPP[18] + P[2][0]*(2*q1*SF[3] - 2*q2*SF[4] - 2*q3*SF[5] + 2*q0*SF[9])) - SPP[4]*(P[0][1]*SPP[5] - P[1][1]*SPP[4] + P[9][1]*SPP[22] + P[12][1]*SPP[18] + P[2][1]*(2*q1*SF[3] - 2*q2*SF[4] - 2*q3*SF[5] + 2*q0*SF[9])) + SPP[8]*(P[0][2]*SPP[5] + P[2][2]*SPP[8] + P[9][2]*SPP[22] + P[12][2]*SPP[18] - P[1][2]*(2*q0*SF[6] - 2*q3*SF[7] - 2*q1*SF[10] + 2*q2*SF[12])) + SPP[22]*(P[0][9]*SPP[5] - P[1][9]*SPP[4] + P[9][9]*SPP[22] + P[12][9]*SPP[18] + P[2][9]*(2*q1*SF[3] - 2*q2*SF[4] - 2*q3*SF[5] + 2*q0*SF[9])) + SPP[18]*(P[0][12]*SPP[5] - P[1][12]*SPP[4] + P[9][12]*SPP[22] + P[12][12]*SPP[18] + P[2][12]*(2*q1*SF[3] - 2*q2*SF[4] - 2*q3*SF[5] + 2*q0*SF[9]));
    nextP[0][1] = SPP[6]*(P[0][1]*SPP[5] - P[1][1]*SPP[4] + P[2][1]*SPP[8] + P[9][1]*SPP[22] + P[12][1]*SPP[18]) - SPP[2]*(P[0][0]*SPP[5] - P[1][0]*SPP[4] + P[2][0]*SPP[8] + P[9][0]*SPP[22] + P[12][0]*SPP[18]) + SPP[22]*(P[0][10]*SPP[5] - P[1][10]*SPP[4] + P[2][10]*SPP[8] + P[9][10]*SPP[22] + P[12][10]*SPP[18]) + SPP[17]*(P[0][13]*SPP[5] - P[1][13]*SPP[4] + P[2][13]*SPP[8] + P[9][13]*SPP[22] + P[12][13]*SPP[18]) - (2*q0*SF[5] - 2*q1*SF[4] - 2*q2*SF[3] + 2*q3*SF[9])*(P[0][2]*SPP[5] - P[1][2]*SPP[4] + P[2][2]*SPP[8] + P[9][2]*SPP[22] + P[12][2]*SPP[18]);
    nextP[1][1] = dayNoise*SQ[3] - SPP[2]*(P[1][0]*SPP[6] - P[0][0]*SPP[2] - P[2][0]*SPP[9] + P[10][0]*SPP[22] + P[13][0]*SPP[17]) + SPP[6]*(P[1][1]*SPP[6] - P[0][1]*SPP[2] - P[2][1]*SPP[9] + P[10][1]*SPP[22] + P[13][1]*SPP[17]) - SPP[9]*(P[1][2]*SPP[6] - P[0][2]*SPP[2] - P[2][2]*SPP[9] + P[10][2]*SPP[22] + P[13][2]*SPP[17]) + SPP[22]*(P[1][10]*SPP[6] - P[0][10]*SPP[2] - P[2][10]*SPP[9] + P[10][10]*SPP[22] + P[13][10]*SPP[17]) + SPP[17]*(P[1][13]*SPP[6] - P[0][13]*SPP[2] - P[2][13]*SPP[9] + P[10][13]*SPP[22] + P[13][13]*SPP[17]);
    nextP[0][2] = SPP[13]*(P[0][2]*SPP[5] - P[1][2]*SPP[4] + P[2][2]*SPP[8] + P[9][2]*SPP[22] + P[12][2]*SPP[18]) - SPP[3]*(P[0][1]*SPP[5] - P[1][1]*SPP[4] + P[2][1]*SPP[8] + P[9][1]*SPP[22] + P[12][1]*SPP[18]) + SPP[22]*(P[0][11]*SPP[5] - P[1][11]*SPP[4] + P[2][11]*SPP[8] + P[9][11]*SPP[22] + P[12][11]*SPP[18]) + SPP[16]*(P[0][14]*SPP[5] - P[1][14]*SPP[4] + P[2][14]*SPP[8] + P[9][14]*SPP[22] + P[12][14]*SPP[18]) + (2*q2*SF[8] - 2*q0*SF[11] - 2*q1*SF[14] + 2*q3*SF[13])*(P[0][0]*SPP[5] - P[1][0]*SPP[4] + P[2][0]*SPP[8] + P[9][0]*SPP[22] + P[12][0]*SPP[18]);
    nextP[1][2] = SPP[13]*(P[1][2]*SPP[6] - P[0][2]*SPP[2] - P[2][2]*SPP[9] + P[10][2]*SPP[22] + P[13][2]*SPP[17]) - SPP[3]*(P[1][1]*SPP[6] - P[0][1]*SPP[2] - P[2][1]*SPP[9] + P[10][1]*SPP[22] + P[13][1]*SPP[17]) + SPP[22]*(P[1][11]*SPP[6] - P[0][11]*SPP[2] - P[2][11]*SPP[9] + P[10][11]*SPP[22] + P[13][11]*SPP[17]) + SPP[16]*(P[1][14]*SPP[6] - P[0][14]*SPP[2] - P[2][14]*SPP[9] + P[10][14]*SPP[22] + P[13][14]*SPP[17]) + (2*q2*SF[8] - 2*q0*SF[11] - 2*q1*SF[14] + 2*q3*SF[13])*(P[1][0]*SPP[6] - P[0][0]*SPP[2] - P[2][0]*SPP[9] + P[10][0]*SPP[22] + P[13][0]*SPP[17]);
    nextP[2][2] = dazNoise*SQ[3] - SPP[3]*(P[0][1]*SPP[14] - P[1][1]*SPP[3] + P[2][1]*SPP[13] + P[11][1]*SPP[22] + P[14][1]*SPP[16]) + SPP[14]*(P[0][0]*SPP[14] - P[1][0]*SPP[3] + P[2][0]*SPP[13] + P[11][0]*SPP[22] + P[14][0]*SPP[16]) + SPP[13]*(P[0][2]*SPP[14] - P[1][2]*SPP[3] + P[2][2]*SPP[13] + P[11][2]*SPP[22] + P[14][2]*SPP[16]) + SPP[22]*(P[0][11]*SPP[14] - P[1][11]*SPP[3] + P[2][11]*SPP[13] + P[11][11]*SPP[22] + P[14][11]*SPP[16]) + SPP[16]*(P[0][14]*SPP[14] - P[1][14]*SPP[3] + P[2][14]*SPP[13] + P[11][14]*SPP[22] + P[14][14]*SPP[16]);
    nextP[0][3] = P[0][3]*SPP[5] - P[1][3]*SPP[4] + P[2][3]*SPP[8] + P[9][3]*SPP[22] + P[12][3]*SPP[18] + SPP[1]*(P[0][0]*SPP[5] - P[1][0]*SPP[4] + P[2][0]*SPP[8] + P[9][0]*SPP[22] + P[12][0]*SPP[18]) + SPP[15]*(P[0][2]*SPP[5] - P[1][2]*SPP[4] + P[2][2]*SPP[8] + P[9][2]*SPP[22] + P[12][2]*SPP[18]) - SPP[21]*(P[0][15]*SPP[5] - P[1][15]*SPP[4] + P[2][15]*SPP[8] + P[9][15]*SPP[22] + P[12][15]*SPP[18]) + (SF[16]*SF[23] - SF[17]*SPP[21])*(P[0][1]*SPP[5] - P[1][1]*SPP[4] + P[2][1]*SPP[8] + P[9][1]*SPP[22] + P[12][1]*SPP[18]);
    nextP[1][3] = P[1][3]*SPP[6] - P[0][3]*SPP[2] - P[2][3]*SPP[9] + P[10][3]*SPP[22] + P[13][3]*SPP[17] + SPP[1]*(P[1][0]*SPP[6] - P[0][0]*SPP[2] - P[2][0]*SPP[9] + P[10][0]*SPP[22] + P[13][0]*SPP[17]) + SPP[15]*(P[1][2]*SPP[6] - P[0][2]*SPP[2] - P[2][2]*SPP[9] + P[10][2]*SPP[22] + P[13][2]*SPP[17]) - SPP[21]*(P[1][15]*SPP[6] - P[0][15]*SPP[2] - P[2][15]*SPP[9] + P[10][15]*SPP[22] + P[13][15]*SPP[17]) + (SF[16]*SF[23] - SF[17]*SPP[21])*(P[1][1]*SPP[6] - P[0][1]*SPP[2] - P[2][1]*SPP[9] + P[10][1]*SPP[22] + P[13][1]*SPP[17]);
    nextP[2][3] = P[0][3]*SPP[14] - P[1][3]*SPP[3] + P[2][3]*SPP[13] + P[11][3]*SPP[22] + P[14][3]*SPP[16] + SPP[1]*(P[0][0]*SPP[14] - P[1][0]*SPP[3] + P[2][0]*SPP[13] + P[11][0]*SPP[22] + P[14][0]*SPP[16]) + SPP[15]*(P[0][2]*SPP[14] - P[1][2]*SPP[3] + P[2][2]*SPP[13] + P[11][2]*SPP[22] + P[14][2]*SPP[16]) - SPP[21]*(P[0][15]*SPP[14] - P[1][15]*SPP[3] + P[2][15]*SPP[13] + P[11][15]*SPP[22] + P[14][15]*SPP[16]) + (SF[16]*SF[23] - SF[17]*SPP[21])*(P[0][1]*SPP[14] - P[1][1]*SPP[3] + P[2][1]*SPP[13] + P[11][1]*SPP[22] + P[14][1]*SPP[16]);
    nextP[3][3] = P[3][3] + P[0][3]*SPP[1] + P[1][3]*SPP[19] + P[2][3]*SPP[15] - P[15][3]*SPP[21] + dvyNoise*sq(SQ[6] - 2*q0*q3) + dvzNoise*sq(SQ[5] + 2*q0*q2) + SPP[1]*(P[3][0] + P[0][0]*SPP[1] + P[1][0]*SPP[19] + P[2][0]*SPP[15] - P[15][0]*SPP[21]) + SPP[19]*(P[3][1] + P[0][1]*SPP[1] + P[1][1]*SPP[19] + P[2][1]*SPP[15] - P[15][1]*SPP[21]) + SPP[15]*(P[3][2] + P[0][2]*SPP[1] + P[1][2]*SPP[19] + P[2][2]*SPP[15] - P[15][2]*SPP[21]) - SPP[21]*(P[3][15] + P[0][15]*SPP[1] + P[2][15]*SPP[15] - P[15][15]*SPP[21] + P[1][15]*(SF[16]*SF[23] - SF[17]*SPP[21])) + dvxNoise*sq(SG[1] + SG[2] - SG[3] - SQ[7]);
    nextP[0][4] = P[0][4]*SPP[5] - P[1][4]*SPP[4] + P[2][4]*SPP[8] + P[9][4]*SPP[22] + P[12][4]*SPP[18] + SF[22]*(P[0][15]*SPP[5] - P[1][15]*SPP[4] + P[2][15]*SPP[8] + P[9][15]*SPP[22] + P[12][15]*SPP[18]) + SPP[12]*(P[0][1]*SPP[5] - P[1][1]*SPP[4] + P[2][1]*SPP[8] + P[9][1]*SPP[22] + P[12][1]*SPP[18]) + SPP[20]*(P[0][0]*SPP[5] - P[1][0]*SPP[4] + P[2][0]*SPP[8] + P[9][0]*SPP[22] + P[12][0]*SPP[18]) + SPP[11]*(P[0][2]*SPP[5] - P[1][2]*SPP[4] + P[2][2]*SPP[8] + P[9][2]*SPP[22] + P[12][2]*SPP[18]);
    nextP[1][4] = P[1][4]*SPP[6] - P[0][4]*SPP[2] - P[2][4]*SPP[9] + P[10][4]*SPP[22] + P[13][4]*SPP[17] + SF[22]*(P[1][15]*SPP[6] - P[0][15]*SPP[2] - P[2][15]*SPP[9] + P[10][15]*SPP[22] + P[13][15]*SPP[17]) + SPP[12]*(P[1][1]*SPP[6] - P[0][1]*SPP[2] - P[2][1]*SPP[9] + P[10][1]*SPP[22] + P[13][1]*SPP[17]) + SPP[20]*(P[1][0]*SPP[6] - P[0][0]*SPP[2] - P[2][0]*SPP[9] + P[10][0]*SPP[22] + P[13][0]*SPP[17]) + SPP[11]*(P[1][2]*SPP[6] - P[0][2]*SPP[2] - P[2][2]*SPP[9] + P[10][2]*SPP[22] + P[13][2]*SPP[17]);
    nextP[2][4] = P[0][4]*SPP[14] - P[1][4]*SPP[3] + P[2][4]*SPP[13] + P[11][4]*SPP[22] + P[14][4]*SPP[16] + SF[22]*(P[0][15]*SPP[14] - P[1][15]*SPP[3] + P[2][15]*SPP[13] + P[11][15]*SPP[22] + P[14][15]*SPP[16]) + SPP[12]*(P[0][1]*SPP[14] - P[1][1]*SPP[3] + P[2][1]*SPP[13] + P[11][1]*SPP[22] + P[14][1]*SPP[16]) + SPP[20]*(P[0][0]*SPP[14] - P[1][0]*SPP[3] + P[2][0]*SPP[13] + P[11][0]*SPP[22] + P[14][0]*SPP[16]) + SPP[11]*(P[0][2]*SPP[14] - P[1][2]*SPP[3] + P[2][2]*SPP[13] + P[11][2]*SPP[22] + P[14][2]*SPP[16]);
    nextP[3][4] = P[3][4] + SQ[2] + P[0][4]*SPP[1] + P[1][4]*SPP[19] + P[2][4]*SPP[15] - P[15][4]*SPP[21] + SF[22]*(P[3][15] + P[0][15]*SPP[1] + P[1][15]*SPP[19] + P[2][15]*SPP[15] - P[15][15]*SPP[21]) + SPP[12]*(P[3][1] + P[0][1]*SPP[1] + P[1][1]*SPP[19] + P[2][1]*SPP[15] - P[15][1]*SPP[21]) + SPP[20]*(P[3][0] + P[0][0]*SPP[1] + P[1][0]*SPP[19] + P[2][0]*SPP[15] - P[15][0]*SPP[21]) + SPP[11]*(P[3][2] + P[0][2]*SPP[1] + P[1][2]*SPP[19] + P[2][2]*SPP[15] - P[15][2]*SPP[21]);
    nextP[4][4] = P[4][4] + P[15][4]*SF[22] + P[0][4]*SPP[20] + P[1][4]*SPP[12] + P[2][4]*SPP[11] + dvxNoise*sq(SQ[6] + 2*q0*q3) + dvzNoise*sq(SQ[4] - 2*q0*q1) + SF[22]*(P[4][15] + P[15][15]*SF[22] + P[0][15]*SPP[20] + P[1][15]*SPP[12] + P[2][15]*SPP[11]) + SPP[12]*(P[4][1] + P[15][1]*SF[22] + P[0][1]*SPP[20] + P[1][1]*SPP[12] + P[2][1]*SPP[11]) + SPP[20]*(P[4][0] + P[15][0]*SF[22] + P[0][0]*SPP[20] + P[1][0]*SPP[12] + P[2][0]*SPP[11]) + SPP[11]*(P[4][2] + P[15][2]*SF[22] + P[0][2]*SPP[20] + P[1][2]*SPP[12] + P[2][2]*SPP[11]) + dvyNoise*sq(SG[1] - SG[2] + SG[3] - SQ[7]);
    nextP[0][5] = P[0][5]*SPP[5] - P[1][5]*SPP[4] + P[2][5]*SPP[8] + P[9][5]*SPP[22] + P[12][5]*SPP[18] + SF[20]*(P[0][15]*SPP[5] - P[1][15]*SPP[4] + P[2][15]*SPP[8] + P[9][15]*SPP[22] + P[12][15]*SPP[18]) - SPP[7]*(P[0][0]*SPP[5] - P[1][0]*SPP[4] + P[2][0]*SPP[8] + P[9][0]*SPP[22] + P[12][0]*SPP[18]) + SPP[0]*(P[0][2]*SPP[5] - P[1][2]*SPP[4] + P[2][2]*SPP[8] + P[9][2]*SPP[22] + P[12][2]*SPP[18]) + SPP[10]*(P[0][1]*SPP[5] - P[1][1]*SPP[4] + P[2][1]*SPP[8] + P[9][1]*SPP[22] + P[12][1]*SPP[18]);
    nextP[1][5] = P[1][5]*SPP[6] - P[0][5]*SPP[2] - P[2][5]*SPP[9] + P[10][5]*SPP[22] + P[13][5]*SPP[17] + SF[20]*(P[1][15]*SPP[6] - P[0][15]*SPP[2] - P[2][15]*SPP[9] + P[10][15]*SPP[22] + P[13][15]*SPP[17]) - SPP[7]*(P[1][0]*SPP[6] - P[0][0]*SPP[2] - P[2][0]*SPP[9] + P[10][0]*SPP[22] + P[13][0]*SPP[17]) + SPP[0]*(P[1][2]*SPP[6] - P[0][2]*SPP[2] - P[2][2]*SPP[9] + P[10][2]*SPP[22] + P[13][2]*SPP[17]) + SPP[10]*(P[1][1]*SPP[6] - P[0][1]*SPP[2] - P[2][1]*SPP[9] + P[10][1]*SPP[22] + P[13][1]*SPP[17]);
    nextP[2][5] = P[0][5]*SPP[14] - P[1][5]*SPP[3] + P[2][5]*SPP[13] + P[11][5]*SPP[22] + P[14][5]*SPP[16] + SF[20]*(P[0][15]*SPP[14] - P[1][15]*SPP[3] + P[2][15]*SPP[13] + P[11][15]*SPP[22] + P[14][15]*SPP[16]) - SPP[7]*(P[0][0]*SPP[14] - P[1][0]*SPP[3] + P[2][0]*SPP[13] + P[11][0]*SPP[22] + P[14][0]*SPP[16]) + SPP[0]*(P[0][2]*SPP[14] - P[1][2]*SPP[3] + P[2][2]*SPP[13] + P[11][2]*SPP[22] + P[14][2]*SPP[16]) + SPP[10]*(P[0][1]*SPP[14] - P[1][1]*SPP[3] + P[2][1]*SPP[13] + P[11][1]*SPP[22] + P[14][1]*SPP[16]);
    nextP[3][5] = P[3][5] + SQ[1] + P[0][5]*SPP[1] + P[1][5]*SPP[19] + P[2][5]*SPP[15] - P[15][5]*SPP[21] + SF[20]*(P[3][15] + P[0][15]*SPP[1] + P[1][15]*SPP[19] + P[2][15]*SPP[15] - P[15][15]*SPP[21]) - SPP[7]*(P[3][0] + P[0][0]*SPP[1] + P[1][0]*SPP[19] + P[2][0]*SPP[15] - P[15][0]*SPP[21]) + SPP[0]*(P[3][2] + P[0][2]*SPP[1] + P[1][2]*SPP[19] + P[2][2]*SPP[15] - P[15][2]*SPP[21]) + SPP[10]*(P[3][1] + P[0][1]*SPP[1] + P[1][1]*SPP[19] + P[2][1]*SPP[15] - P[15][1]*SPP[21]);
    nextP[4][5] = P[4][5] + SQ[0] + P[15][5]*SF[22] + P[0][5]*SPP[20] + P[1][5]*SPP[12] + P[2][5]*SPP[11] + SF[20]*(P[4][15] + P[15][15]*SF[22] + P[0][15]*SPP[20] + P[1][15]*SPP[12] + P[2][15]*SPP[11]) - SPP[7]*(P[4][0] + P[15][0]*SF[22] + P[0][0]*SPP[20] + P[1][0]*SPP[12] + P[2][0]*SPP[11]) + SPP[0]*(P[4][2] + P[15][2]*SF[22] + P[0][2]*SPP[20] + P[1][2]*SPP[12] + P[2][2]*SPP[11]) + SPP[10]*(P[4][1] + P[15][1]*SF[22] + P[0][1]*SPP[20] + P[1][1]*SPP[12] + P[2][1]*SPP[11]);
    nextP[5][5] = P[5][5] + P[15][5]*SF[20] - P[0][5]*SPP[7] + P[1][5]*SPP[10] + P[2][5]*SPP[0] + dvxNoise*sq(SQ[5] - 2*q0*q2) + dvyNoise*sq(SQ[4] + 2*q0*q1) + SF[20]*(P[5][15] + P[15][15]*SF[20] - P[0][15]*SPP[7] + P[1][15]*SPP[10] + P[2][15]*SPP[0]) - SPP[7]*(P[5][0] + P[15][0]*SF[20] - P[0][0]*SPP[7] + P[1][0]*SPP[10] + P[2][0]*SPP[0]) + SPP[0]*(P[5][2] + P[15][2]*SF[20] - P[0][2]*SPP[7] + P[1][2]*SPP[10] + P[2][2]*SPP[0]) + SPP[10]*(P[5][1] + P[15][1]*SF[20] - P[0][1]*SPP[7] + P[1][1]*SPP[10] + P[2][1]*SPP[0]) + dvzNoise*sq(SG[1] - SG[2] - SG[3] + SQ[7]);
    nextP[0][6] = P[0][6]*SPP[5] - P[1][6]*SPP[4] + P[2][6]*SPP[8] + P[9][6]*SPP[22] + P[12][6]*SPP[18] + dt*(P[0][3]*SPP[5] - P[1][3]*SPP[4] + P[2][3]*SPP[8] + P[9][3]*SPP[22] + P[12][3]*SPP[18]);
    nextP[1][6] = P[1][6]*SPP[6] - P[0][6]*SPP[2] - P[2][6]*SPP[9] + P[10][6]*SPP[22] + P[13][6]*SPP[17] + dt*(P[1][3]*SPP[6] - P[0][3]*SPP[2] - P[2][3]*SPP[9] + P[10][3]*SPP[22] + P[13][3]*SPP[17]);
    nextP[2][6] = P[0][6]*SPP[14] - P[1][6]*SPP[3] + P[2][6]*SPP[13] + P[11][6]*SPP[22] + P[14][6]*SPP[16] + dt*(P[0][3]*SPP[14] - P[1][3]*SPP[3] + P[2][3]*SPP[13] + P[11][3]*SPP[22] + P[14][3]*SPP[16]);
    nextP[3][6] = P[3][6] + P[0][6]*SPP[1] + P[1][6]*SPP[19] + P[2][6]*SPP[15] - P[15][6]*SPP[21] + dt*(P[3][3] + P[0][3]*SPP[1] + P[1][3]*SPP[19] + P[2][3]*SPP[15] - P[15][3]*SPP[21]);
    nextP[4][6] = P[4][6] + P[15][6]*SF[22] + P[0][6]*SPP[20] + P[1][6]*SPP[12] + P[2][6]*SPP[11] + dt*(P[4][3] + P[15][3]*SF[22] + P[0][3]*SPP[20] + P[1][3]*SPP[12] + P[2][3]*SPP[11]);
    nextP[5][6] = P[5][6] + P[15][6]*SF[20] - P[0][6]*SPP[7] + P[1][6]*SPP[10] + P[2][6]*SPP[0] + dt*(P[5][3] + P[15][3]*SF[20] - P[0][3]*SPP[7] + P[1][3]*SPP[10] + P[2][3]*SPP[0]);
    nextP[6][6] = P[6][6] + P[3][6]*dt + dt*(P[6][3] + P[3][3]*dt);
    nextP[0][7] = P[0][7]*SPP[5] - P[1][7]*SPP[4] + P[2][7]*SPP[8] + P[9][7]*SPP[22] + P[12][7]*SPP[18] + dt*(P[0][4]*SPP[5] - P[1][4]*SPP[4] + P[2][4]*SPP[8] + P[9][4]*SPP[22] + P[12][4]*SPP[18]);
    nextP[1][7] = P[1][7]*SPP[6] - P[0][7]*SPP[2] - P[2][7]*SPP[9] + P[10][7]*SPP[22] + P[13][7]*SPP[17] + dt*(P[1][4]*SPP[6] - P[0][4]*SPP[2] - P[2][4]*SPP[9] + P[10][4]*SPP[22] + P[13][4]*SPP[17]);
    nextP[2][7] = P[0][7]*SPP[14] - P[1][7]*SPP[3] + P[2][7]*SPP[13] + P[11][7]*SPP[22] + P[14][7]*SPP[16] + dt*(P[0][4]*SPP[14] - P[1][4]*SPP[3] + P[2][4]*SPP[13] + P[11][4]*SPP[22] + P[14][4]*SPP[16]);
    nextP[3][7] = P[3][7] + P[0][7]*SPP[1] + P[1][7]*SPP[19] + P[2][7]*SPP[15] - P[15][7]*SPP[21] + dt*(P[3][4] + P[0][4]*SPP[1] + P[1][4]*SPP[19] + P[2][4]*SPP[15] - P[15][4]*SPP[21]);
    nextP[4][7] = P[4][7] + P[15][7]*SF[22] + P[0][7]*SPP[20] + P[1][7]*SPP[12] + P[2][7]*SPP[11] + dt*(P[4][4] + P[15][4]*SF[22] + P[0][4]*SPP[20] + P[1][4]*SPP[12] + P[2][4]*SPP[11]);
    nextP[5][7] = P[5][7] + P[15][7]*SF[20] - P[0][7]*SPP[7] + P[1][7]*SPP[10] + P[2][7]*SPP[0] + dt*(P[5][4] + P[15][4]*SF[20] - P[0][4]*SPP[7] + P[1][4]*SPP[10] + P[2][4]*SPP[0]);
    nextP[6][7] = P[6][7] + P[3][7]*dt + dt*(P[6][4] + P[3][4]*dt);
    nextP[7][7] = P[7][7] + P[4][7]*dt + dt*(P[7][4] + P[4][4]*dt);
    nextP[0][8] = P[0][8]*SPP[5] - P[1][8]*SPP[4] + P[2][8]*SPP[8] + P[9][8]*SPP[22] + P[12][8]*SPP[18] + dt*(P[0][5]*SPP[5] - P[1][5]*SPP[4] + P[2][5]*SPP[8] + P[9][5]*SPP[22] + P[12][5]*SPP[18]);
    nextP[1][8] = P[1][8]*SPP[6] - P[0][8]*SPP[2] - P[2][8]*SPP[9] + P[10][8]*SPP[22] + P[13][8]*SPP[17] + dt*(P[1][5]*SPP[6] - P[0][5]*SPP[2] - P[2][5]*SPP[9] + P[10][5]*SPP[22] + P[13][5]*SPP[17]);
    nextP[2][8] = P[0][8]*SPP[14] - P[1][8]*SPP[3] + P[2][8]*SPP[13] + P[11][8]*SPP[22] + P[14][8]*SPP[16] + dt*(P[0][5]*SPP[14] - P[1][5]*SPP[3] + P[2][5]*SPP[13] + P[11][5]*SPP[22] + P[14][5]*SPP[16]);
    nextP[3][8] = P[3][8] + P[0][8]*SPP[1] + P[1][8]*SPP[19] + P[2][8]*SPP[15] - P[15][8]*SPP[21] + dt*(P[3][5] + P[0][5]*SPP[1] + P[1][5]*SPP[19] + P[2][5]*SPP[15] - P[15][5]*SPP[21]);
    nextP[4][8] = P[4][8] + P[15][8]*SF[22] + P[0][8]*SPP[20] + P[1][8]*SPP[12] + P[2][8]*SPP[11] + dt*(P[4][5] + P[15][5]*SF[22] + P[0][5]*SPP[20] + P[1][5]*SPP[12] + P[2][5]*SPP[11]);
    nextP[5][8] = P[5][8] + P[15][8]*SF[20] - P[0][8]*SPP[7] + P[1][8]*SPP[10] + P[2][8]*SPP[0] + dt*(P[5][5] + P[15][5]*SF[20] - P[0][5]*SPP[7] + P[1][5]*SPP[10] + P[2][5]*SPP[0]);
    nextP[6][8] = P[6][8] + P[3][8]*dt + dt*(P[6][5] + P[3][5]*dt);
    nextP[7][8] = P[7][8] + P[4][8]*dt + dt*(P[7][5] + P[4][5]*dt);
    nextP[8][8] = P[8][8] + P[5][8]*dt + dt*(P[8][5] + P[5][5]*dt);
    nextP[0][9] = P[0][9]*SPP[5] - P[1][9]*SPP[4] + P[2][9]*SPP[8] + P[9][9]*SPP[22] + P[12][9]*SPP[18];
    nextP[1][9] = P[1][9]*SPP[6] - P[0][9]*SPP[2] - P[2][9]*SPP[9] + P[10][9]*SPP[22] + P[13][9]*SPP[17];
    nextP[2][9] = P[0][9]*SPP[14] - P[1][9]*SPP[3] + P[2][9]*SPP[13] + P[11][9]*SPP[22] + P[14][9]*SPP[16];
    nextP[3][9] = P[3][9] + P[0][9]*SPP[1] + P[1][9]*SPP[19] + P[2][9]*SPP[15] - P[15][9]*SPP[21];
    nextP[4][9] = P[4][9] + P[15][9]*SF[22] + P[0][9]*SPP[20] + P[1][9]*SPP[12] + P[2][9]*SPP[11];
    nextP[5][9] = P[5][9] + P[15][9]*SF[20] - P[0][9]*SPP[7] + P[1][9]*SPP[10] + P[2][9]*SPP[0];
    nextP[6][9] = P[6][9] + P[3][9]*dt;
    nextP[7][9] = P[7][9] + P[4][9]*dt;
    nextP[8][9] = P[8][9] + P[5][9]*dt;
    nextP[9][9] = P[9][9];
    nextP[0][10] = P[0][10]*SPP[5] - P[1][10]*SPP[4] + P[2][10]*SPP[8] + P[9][10]*SPP[22] + P[12][10]*SPP[18];
    nextP[1][10] = P[1][10]*SPP[6] - P[0][10]*SPP[2] - P[2][10]*SPP[9] + P[10][10]*SPP[22] + P[13][10]*SPP[17];
    nextP[2][10] = P[0][10]*SPP[14] - P[1][10]*SPP[3] + P[2][10]*SPP[13] + P[11][10]*SPP[22] + P[14][10]*SPP[16];
    nextP[3][10] = P[3][10] + P[0][10]*SPP[1] + P[1][10]*SPP[19] + P[2][10]*SPP[15] - P[15][10]*SPP[21];
    nextP[4][10] = P[4][10] + P[15][10]*SF[22] + P[0][10]*SPP[20] + P[1][10]*SPP[12] + P[2][10]*SPP[11];
    nextP[5][10] = P[5][10] + P[15][10]*SF[20] - P[0][10]*SPP[7] + P[1][10]*SPP[10] + P[2][10]*SPP[0];
    nextP[6][10] = P[6][10] + P[3][10]*dt;
    nextP[7][10] = P[7][10] + P[4][10]*dt;
    nextP[8][10] = P[8][10] + P[5][10]*dt;
    nextP[9][10] = P[9][10];
    nextP[10][10] = P[10][10];
    nextP[0][11] = P[0][11]*SPP[5] - P[1][11]*SPP[4] + P[2][11]*SPP[8] + P[9][11]*SPP[22] + P[12][11]*SPP[18];
    nextP[1][11] = P[1][11]*SPP[6] - P[0][11]*SPP[2] - P[2][11]*SPP[9] + P[10][11]*SPP[22] + P[13][11]*SPP[17];
    nextP[2][11] = P[0][11]*SPP[14] - P[1][11]*SPP[3] + P[2][11]*SPP[13] + P[11][11]*SPP[22] + P[14][11]*SPP[16];
    nextP[3][11] = P[3][11] + P[0][11]*SPP[1] + P[1][11]*SPP[19] + P[2][11]*SPP[15] - P[15][11]*SPP[21];
    nextP[4][11] = P[4][11] + P[15][11]*SF[22] + P[0][11]*SPP[20] + P[1][11]*SPP[12] + P[2][11]*SPP[11];
    nextP[5][11] = P[5][11] + P[15][11]*SF[20] - P[0][11]*SPP[7] + P[1][11]*SPP[10] + P[2][11]*SPP[0];
    nextP[6][11] = P[6][11] + P[3][11]*dt;
    nextP[7][11] = P[7][11] + P[4][11]*dt;
    nextP[8][11] = P[8][11] + P[5][11]*dt;
    nextP[9][11] = P[9][11];
    nextP[10][11] = P[10][11];
    nextP[11][11] = P[11][11];
    nextP[0][12] = P[0][12]*SPP[5] - P[1][12]*SPP[4] + P[2][12]*SPP[8] + P[9][12]*SPP[22] + P[12][12]*SPP[18];
    nextP[1][12] = P[1][12]*SPP[6] - P[0][12]*SPP[2] - P[2][12]*SPP[9] + P[10][12]*SPP[22] + P[13][12]*SPP[17];
    nextP[2][12] = P[0][12]*SPP[14] - P[1][12]*SPP[3] + P[2][12]*SPP[13] + P[11][12]*SPP[22] + P[14][12]*SPP[16];
    nextP[3][12] = P[3][12] + P[0][12]*SPP[1] + P[1][12]*SPP[19] + P[2][12]*SPP[15] - P[15][12]*SPP[21];
    nextP[4][12] = P[4][12] + P[15][12]*SF[22] + P[0][12]*SPP[20] + P[1][12]*SPP[12] + P[2][12]*SPP[11];
    nextP[5][12] = P[5][12] + P[15][12]*SF[20] - P[0][12]*SPP[7] + P[1][12]*SPP[10] + P[2][12]*SPP[0];
    nextP[6][12] = P[6][12] + P[3][12]*dt;
    nextP[7][12] = P[7][12] + P[4][12]*dt;
    nextP[8][12] = P[8][12] + P[5][12]*dt;
    nextP[9][12] = P[9][12];
    nextP[10][12] = P[10][12];
    nextP[11][12] = P[11][12];
    nextP[12][12] = P[12][12];
    nextP[0][13] = P[0][13]*SPP[5] - P[1][13]*SPP[4] + P[2][13]*SPP[8] + P[9][13]*SPP[22] + P[12][13]*SPP[18];
    nextP[1][13] = P[1][13]*SPP[6] - P[0][13]*SPP[2] - P[2][13]*SPP[9] + P[10][13]*SPP[22] + P[13][13]*SPP[17];
    nextP[2][13] = P[0][13]*SPP[14] - P[1][13]*SPP[3] + P[2][13]*SPP[13] + P[11][13]*SPP[22] + P[14][13]*SPP[16];
    nextP[3][13] = P[3][13] + P[0][13]*SPP[1] + P[1][13]*SPP[19] + P[2][13]*SPP[15] - P[15][13]*SPP[21];
    nextP[4][13] = P[4][13] + P[15][13]*SF[22] + P[0][13]*SPP[20] + P[1][13]*SPP[12] + P[2][13]*SPP[11];
    nextP[5][13] = P[5][13] + P[15][13]*SF[20] - P[0][13]*SPP[7] + P[1][13]*SPP[10] + P[2][13]*SPP[0];
    nextP[6][13] = P[6][13] + P[3][13]*dt;
    nextP[7][13] = P[7][13] + P[4][13]*dt;
    nextP[8][13] = P[8][13] + P[5][13]*dt;
    nextP[9][13] = P[9][13];
    nextP[10][13] = P[10][13];
    nextP[11][13] = P[11][13];
    nextP[12][13] = P[12][13];
    nextP[13][13] = P[13][13];
    nextP[0][14] = P[0][14]*SPP[5] - P[1][14]*SPP[4] + P[2][14]*SPP[8] + P[9][14]*SPP[22] + P[12][14]*SPP[18];
    nextP[1][14] = P[1][14]*SPP[6] - P[0][14]*SPP[2] - P[2][14]*SPP[9] + P[10][14]*SPP[22] + P[13][14]*SPP[17];
    nextP[2][14] = P[0][14]*SPP[14] - P[1][14]*SPP[3] + P[2][14]*SPP[13] + P[11][14]*SPP[22] + P[14][14]*SPP[16];
    nextP[3][14] = P[3][14] + P[0][14]*SPP[1] + P[1][14]*SPP[19] + P[2][14]*SPP[15] - P[15][14]*SPP[21];
    nextP[4][14] = P[4][14] + P[15][14]*SF[22] + P[0][14]*SPP[20] + P[1][14]*SPP[12] + P[2][14]*SPP[11];
    nextP[5][14] = P[5][14] + P[15][14]*SF[20] - P[0][14]*SPP[7] + P[1][14]*SPP[10] + P[2][14]*SPP[0];
    nextP[6][14] = P[6][14] + P[3][14]*dt;
    nextP[7][14] = P[7][14] + P[4][14]*dt;
    nextP[8][14] = P[8][14] + P[5][14]*dt;
    nextP[9][14] = P[9][14];
    nextP[10][14] = P[10][14];
    nextP[11][14] = P[11][14];
    nextP[12][14] = P[12][14];
    nextP[13][14] = P[13][14];
    nextP[14][14] = P[14][14];
    nextP[0][15] = P[0][15]*SPP[5] - P[1][15]*SPP[4] + P[2][15]*SPP[8] + P[9][15]*SPP[22] + P[12][15]*SPP[18];
    nextP[1][15] = P[1][15]*SPP[6] - P[0][15]*SPP[2] - P[2][15]*SPP[9] + P[10][15]*SPP[22] + P[13][15]*SPP[17];
    nextP[2][15] = P[0][15]*SPP[14] - P[1][15]*SPP[3] + P[2][15]*SPP[13] + P[11][15]*SPP[22] + P[14][15]*SPP[16];
    nextP[3][15] = P[3][15] + P[0][15]*SPP[1] + P[1][15]*SPP[19] + P[2][15]*SPP[15] - P[15][15]*SPP[21];
    nextP[4][15] = P[4][15] + P[15][15]*SF[22] + P[0][15]*SPP[20] + P[1][15]*SPP[12] + P[2][15]*SPP[11];
    nextP[5][15] = P[5][15] + P[15][15]*SF[20] - P[0][15]*SPP[7] + P[1][15]*SPP[10] + P[2][15]*SPP[0];
    nextP[6][15] = P[6][15] + P[3][15]*dt;
    nextP[7][15] = P[7][15] + P[4][15]*dt;
    nextP[8][15] = P[8][15] + P[5][15]*dt;
    nextP[9][15] = P[9][15];
    nextP[10][15] = P[10][15];
    nextP[11][15] = P[11][15];
    nextP[12][15] = P[12][15];
    nextP[13][15] = P[13][15];
    nextP[14][15] = P[14][15];
    nextP[15][15] = P[15][15];

    if (stateIndexLim > 15) {
        nextP[0][16] = P[0][16]*SPP[5] - P[1][16]*SPP[4] + P[2][16]*SPP[8] + P[9][16]*SPP[22] + P[12][16]*SPP[18];
        nextP[1][16] = P[1][16]*SPP[6] - P[0][16]*SPP[2] - P[2][16]*SPP[9] + P[10][16]*SPP[22] + P[13][16]*SPP[17];
        nextP[2][16] = P[0][16]*SPP[14] - P[1][16]*SPP[3] + P[2][16]*SPP[13] + P[11][16]*SPP[22] + P[14][16]*SPP[16];
        nextP[3][16] = P[3][16] + P[0][16]*SPP[1] + P[1][16]*SPP[19] + P[2][16]*SPP[15] - P[15][16]*SPP[21];
        nextP[4][16] = P[4][16] + P[15][16]*SF[22] + P[0][16]*SPP[20] + P[1][16]*SPP[12] + P[2][16]*SPP[11];
        nextP[5][16] = P[5][16] + P[15][16]*SF[20] - P[0][16]*SPP[7] + P[1][16]*SPP[10] + P[2][16]*SPP[0];
        nextP[6][16] = P[6][16] + P[3][16]*dt;
        nextP[7][16] = P[7][16] + P[4][16]*dt;
        nextP[8][16] = P[8][16] + P[5][16]*dt;
        nextP[9][16] = P[9][16];
        nextP[10][16] = P[10][16];
        nextP[11][16] = P[11][16];
        nextP[12][16] = P[12][16];
        nextP[13][16] = P[13][16];
        nextP[14][16] = P[14][16];
        nextP[15][16] = P[15][16];
        nextP[16][16] = P[16][16];
        nextP[0][17] = P[0][17]*SPP[5] - P[1][17]*SPP[4] + P[2][17]*SPP[8] + P[9][17]*SPP[22] + P[12][17]*SPP[18];
        nextP[1][17] = P[1][17]*SPP[6] - P[0][17]*SPP[2] - P[2][17]*SPP[9] + P[10][17]*SPP[22] + P[13][17]*SPP[17];
        nextP[2][17] = P[0][17]*SPP[14] - P[1][17]*SPP[3] + P[2][17]*SPP[13] + P[11][17]*SPP[22] + P[14][17]*SPP[16];
        nextP[3][17] = P[3][17] + P[0][17]*SPP[1] + P[1][17]*SPP[19] + P[2][17]*SPP[15] - P[15][17]*SPP[21];
        nextP[4][17] = P[4][17] + P[15][17]*SF[22] + P[0][17]*SPP[20] + P[1][17]*SPP[12] + P[2][17]*SPP[11];
        nextP[5][17] = P[5][17] + P[15][17]*SF[20] - P[0][17]*SPP[7] + P[1][17]*SPP[10] + P[2][17]*SPP[0];
        nextP[6][17] = P[6][17] + P[3][17]*dt;
        nextP[7][17] = P[7][17] + P[4][17]*dt;
        nextP[8][17] = P[8][17] + P[5][17]*dt;
        nextP[9][17] = P[9][17];
        nextP[10][17] = P[10][17];
        nextP[11][17] = P[11][17];
        nextP[12][17] = P[12][17];
        nextP[13][17] = P[13][17];
        nextP[14][17] = P[14][17];
        nextP[15][17] = P[15][17];
        nextP[16][17] = P[16][17];
        nextP[17][17] = P[17][17];
        nextP[0][18] = P[0][18]*SPP[5] - P[1][18]*SPP[4] + P[2][18]*SPP[8] + P[9][18]*SPP[22] + P[12][18]*SPP[18];
        nextP[1][18] = P[1][18]*SPP[6] - P[0][18]*SPP[2] - P[2][18]*SPP[9] + P[10][18]*SPP[22] + P[13][18]*SPP[17];
        nextP[2][18] = P[0][18]*SPP[14] - P[1][18]*SPP[3] + P[2][18]*SPP[13] + P[11][18]*SPP[22] + P[14][18]*SPP[16];
        nextP[3][18] = P[3][18] + P[0][18]*SPP[1] + P[1][18]*SPP[19] + P[2][18]*SPP[15] - P[15][18]*SPP[21];
        nextP[4][18] = P[4][18] + P[15][18]*SF[22] + P[0][18]*SPP[20] + P[1][18]*SPP[12] + P[2][18]*SPP[11];
        nextP[5][18] = P[5][18] + P[15][18]*SF[20] - P[0][18]*SPP[7] + P[1][18]*SPP[10] + P[2][18]*SPP[0];
        nextP[6][18] = P[6][18] + P[3][18]*dt;
        nextP[7][18] = P[7][18] + P[4][18]*dt;
        nextP[8][18] = P[8][18] + P[5][18]*dt;
        nextP[9][18] = P[9][18];
        nextP[10][18] = P[10][18];
        nextP[11][18] = P[11][18];
        nextP[12][18] = P[12][18];
        nextP[13][18] = P[13][18];
        nextP[14][18] = P[14][18];
        nextP[15][18] = P[15][18];
        nextP[16][18] = P[16][18];
        nextP[17][18] = P[17][18];
        nextP[18][18] = P[18][18];
        nextP[0][19] = P[0][19]*SPP[5] - P[1][19]*SPP[4] + P[2][19]*SPP[8] + P[9][19]*SPP[22] + P[12][19]*SPP[18];
        nextP[1][19] = P[1][19]*SPP[6] - P[0][19]*SPP[2] - P[2][19]*SPP[9] + P[10][19]*SPP[22] + P[13][19]*SPP[17];
        nextP[2][19] = P[0][19]*SPP[14] - P[1][19]*SPP[3] + P[2][19]*SPP[13] + P[11][19]*SPP[22] + P[14][19]*SPP[16];
        nextP[3][19] = P[3][19] + P[0][19]*SPP[1] + P[1][19]*SPP[19] + P[2][19]*SPP[15] - P[15][19]*SPP[21];
        nextP[4][19] = P[4][19] + P[15][19]*SF[22] + P[0][19]*SPP[20] + P[1][19]*SPP[12] + P[2][19]*SPP[11];
        nextP[5][19] = P[5][19] + P[15][19]*SF[20] - P[0][19]*SPP[7] + P[1][19]*SPP[10] + P[2][19]*SPP[0];
        nextP[6][19] = P[6][19] + P[3][19]*dt;
        nextP[7][19] = P[7][19] + P[4][19]*dt;
        nextP[8][19] = P[8][19] + P[5][19]*dt;
        nextP[9][19] = P[9][19];
        nextP[10][19] = P[10][19];
        nextP[11][19] = P[11][19];
        nextP[12][19] = P[12][19];
        nextP[13][19] = P[13][19];
        nextP[14][19] = P[14][19];
        nextP[15][19] = P[15][19];
        nextP[16][19] = P[16][19];
        nextP[17][19] = P[17][19];
        nextP[18][19] = P[18][19];
        nextP[19][19] = P[19][19];
        nextP[0][20] = P[0][20]*SPP[5] - P[1][20]*SPP[4] + P[2][20]*SPP[8] + P[9][20]*SPP[22] + P[12][20]*SPP[18];
        nextP[1][20] = P[1][20]*SPP[6] - P[0][20]*SPP[2] - P[2][20]*SPP[9] + P[10][20]*SPP[22] + P[13][20]*SPP[17];
        nextP[2][20] = P[0][20]*SPP[14] - P[1][20]*SPP[3] + P[2][20]*SPP[13] + P[11][20]*SPP[22] + P[14][20]*SPP[16];
        nextP[3][20] = P[3][20] + P[0][20]*SPP[1] + P[1][20]*SPP[19] + P[2][20]*SPP[15] - P[15][20]*SPP[21];
        nextP[4][20] = P[4][20] + P[15][20]*SF[22] + P[0][20]*SPP[20] + P[1][20]*SPP[12] + P[2][20]*SPP[11];
        nextP[5][20] = P[5][20] + P[15][20]*SF[20] - P[0][20]*SPP[7] + P[1][20]*SPP[10] + P[2][20]*SPP[0];
        nextP[6][20] = P[6][20] + P[3][20]*dt;
        nextP[7][20] = P[7][20] + P[4][20]*dt;
        nextP[8][20] = P[8][20] + P[5][20]*dt;
        nextP[9][20] = P[9][20];
        nextP[10][20] = P[10][20];
        nextP[11][20] = P[11][20];
        nextP[12][20] = P[12][20];
        nextP[13][20] = P[13][20];
        nextP[14][20] = P[14][20];
        nextP[15][20] = P[15][20];
        nextP[16][20] = P[16][20];
        nextP[17][20] = P[17][20];
        nextP[18][20] = P[18][20];
        nextP[19][20] = P[19][20];
        nextP[20][20] = P[20][20];
        nextP[0][21] = P[0][21]*SPP[5] - P[1][21]*SPP[4] + P[2][21]*SPP[8] + P[9][21]*SPP[22] + P[12][21]*SPP[18];
        nextP[1][21] = P[1][21]*SPP[6] - P[0][21]*SPP[2] - P[2][21]*SPP[9] + P[10][21]*SPP[22] + P[13][21]*SPP[17];
        nextP[2][21] = P[0][21]*SPP[14] - P[1][21]*SPP[3] + P[2][21]*SPP[13] + P[11][21]*SPP[22] + P[14][21]*SPP[16];
        nextP[3][21] = P[3][21] + P[0][21]*SPP[1] + P[1][21]*SPP[19] + P[2][21]*SPP[15] - P[15][21]*SPP[21];
        nextP[4][21] = P[4][21] + P[15][21]*SF[22] + P[0][21]*SPP[20] + P[1][21]*SPP[12] + P[2][21]*SPP[11];
        nextP[5][21] = P[5][21] + P[15][21]*SF[20] - P[0][21]*SPP[7] + P[1][21]*SPP[10] + P[2][21]*SPP[0];
        nextP[6][21] = P[6][21] + P[3][21]*dt;
        nextP[7][21] = P[7][21] + P[4][21]*dt;
        nextP[8][21] = P[8][21] + P[5][21]*dt;
        nextP[9][21] = P[9][21];
        nextP[10][21] = P[10][21];
        nextP[11][21] = P[11][21];
        nextP[12][21] = P[12][21];
        nextP[13][21] = P[13][21];
        nextP[14][21] = P[14][21];
        nextP[15][21] = P[15][21];
        nextP[16][21] = P[16][21];
        nextP[17][21] = P[17][21];
        nextP[18][21] = P[18][21];
        nextP[19][21] = P[19][21];
        nextP[20][21] = P[20][21];
        nextP[21][21] = P[21][21];

        if (stateIndexLim > 21) {
            nextP[0][22] = P[0][22]*SPP[5] - P[1][22]*SPP[4] + P[2][22]*SPP[8] + P[9][22]*SPP[22] + P[12][22]*SPP[18];
            nextP[1][22] = P[1][22]*SPP[6] - P[0][22]*SPP[2] - P[2][22]*SPP[9] + P[10][22]*SPP[22] + P[13][22]*SPP[17];
            nextP[2][22] = P[0][22]*SPP[14] - P[1][22]*SPP[3] + P[2][22]*SPP[13] + P[11][22]*SPP[22] + P[14][22]*SPP[16];
            nextP[3][22] = P[3][22] + P[0][22]*SPP[1] + P[1][22]*SPP[19] + P[2][22]*SPP[15] - P[15][22]*SPP[21];
            nextP[4][22] = P[4][22] + P[15][22]*SF[22] + P[0][22]*SPP[20] + P[1][22]*SPP[12] + P[2][22]*SPP[11];
            nextP[5][22] = P[5][22] + P[15][22]*SF[20] - P[0][22]*SPP[7] + P[1][22]*SPP[10] + P[2][22]*SPP[0];
            nextP[6][22] = P[6][22] + P[3][22]*dt;
            nextP[7][22] = P[7][22] + P[4][22]*dt;
            nextP[8][22] = P[8][22] + P[5][22]*dt;
            nextP[9][22] = P[9][22];
            nextP[10][22] = P[10][22];
            nextP[11][22] = P[11][22];
            nextP[12][22] = P[12][22];
            nextP[13][22] = P[13][22];
            nextP[14][22] = P[14][22];
            nextP[15][22] = P[15][22];
            nextP[16][22] = P[16][22];
            nextP[17][22] = P[17][22];
            nextP[18][22] = P[18][22];
            nextP[19][22] = P[19][22];
            nextP[20][22] = P[20][22];
            nextP[21][22] = P[21][22];
            nextP[22][22] = P[22][22];
            nextP[0][23] = P[0][23]*SPP[5] - P[1][23]*SPP[4] + P[2][23]*SPP[8] + P[9][23]*SPP[22] + P[12][23]*SPP[18];
            nextP[1][23] = P[1][23]*SPP[6] - P[0][23]*SPP[2] - P[2][23]*SPP[9] + P[10][23]*SPP[22] + P[13][23]*SPP[17];
            nextP[2][23] = P[0][23]*SPP[14] - P[1][23]*SPP[3] + P[2][23]*SPP[13] + P[11][23]*SPP[22] + P[14][23]*SPP[16];
            nextP[3][23] = P[3][23] + P[0][23]*SPP[1] + P[1][23]*SPP[19] + P[2][23]*SPP[15] - P[15][23]*SPP[21];
            nextP[4][23] = P[4][23] + P[15][23]*SF[22] + P[0][23]*SPP[20] + P[1][23]*SPP[12] + P[2][23]*SPP[11];
            nextP[5][23] = P[5][23] + P[15][23]*SF[20] - P[0][23]*SPP[7] + P[1][23]*SPP[10] + P[2][23]*SPP[0];
            nextP[6][23] = P[6][23] + P[3][23]*dt;
            nextP[7][23] = P[7][23] + P[4][23]*dt;
            nextP[8][23] = P[8][23] + P[5][23]*dt;
            nextP[9][23] = P[9][23];
            nextP[10][23] = P[10][23];
            nextP[11][23] = P[11][23];
            nextP[12][23] = P[12][23];
            nextP[13][23] = P[13][23];
            nextP[14][23] = P[14][23];
            nextP[15][23] = P[15][23];
            nextP[16][23] = P[16][23];
            nextP[17][23] = P[17][23];
            nextP[18][23] = P[18][23];
            nextP[19][23] = P[19][23];
            nextP[20][23] = P[20][23];
            nextP[21][23] = P[21][23];
            nextP[22][23] = P[22][23];
            nextP[23][23] = P[23][23];
        }
    }

    // Copy upper diagonal to lower diagonal taking advantage of symmetry
    //ÀûÓÃ¶Ô³ÆÐÔ£¬½«ÉÏ¶Ô½ÇÔªËØ¸´ÖÆµ½ÏÂ¶Ô½Ç£¬±£Ö¤PÕóÊÇ¶Ô³ÆµÄ
    for (uint8_t colIndex=0; colIndex<=stateIndexLim; colIndex++)
    {
        for (uint8_t rowIndex=0; rowIndex<colIndex; rowIndex++)
        {
            nextP[colIndex][rowIndex] = nextP[rowIndex][colIndex];
        }
    }

    // add the general state process noise variances
    //Ìí¼ÓÒ»°ã¹ý³ÌµÄÔëÉù·½²î
    for (uint8_t i=0; i<=stateIndexLim; i++)
    {
        nextP[i][i] = nextP[i][i] + processNoise[i];
    }

    // if the total position variance exceeds 1e4 (100m), then stop covariance
    // growth by setting the predicted to the previous values
    // This prevent an ill conditioned matrix from occurring for long periods
    // without GPS
    //Èç¹û×ÜµÄÎ»ÖÃ·½²î³¬¹ý100m£¬Ôò½«Ò»²½Ô¤²âµÄ·½²îÖÃÎªÉÏ´Î×´Ì¬¹À¼ÆµÄ·½²î
    //Õâ¿ÉÒÔÔÚ³¤Ê±¼äÈ±ÉÙGPSÐÅºÅÊ±·ÀÖ¹²¡Ì¬¾ØÕó
    if ((P[6][6] + P[7][7]) > 1e4f)
    {
        for (uint8_t i=6; i<=7; i++)
        {
            for (uint8_t j=0; j<=stateIndexLim; j++)
            {
                nextP[i][j] = P[i][j];
                nextP[j][i] = P[j][i];
            }
        }
    }

    // copy covariances to output
    CopyCovariances();

    // constrain diagonals to prevent ill-conditioning
    //ÏÞÖÆÐ­·½²î¶Ô½ÇÏßµÄ´óÐ¡£¬Õë¶Ô²»Í¬µÄÔªËØÉèÖÃ²»Í¬µÄãÐÖµ
    ConstrainVariances();

    hal.util->perf_end(_perf_CovariancePrediction);
}

// zero specified range of rows in the state covariance matrix
//½«×´Ì¬Ð­·½²îÕóÖÐÖ¸¶¨µÄÐÐ³õÊ¼»¯ÎªÁã  why¡¢how¡¢what¡¢result£¬ÄãÄÜÑ§µ½Ê²Ã´ ÓÃµ½ÄÄÀï
//firstºÍlastµÄÐÐµÄ·¶Î§
void NavEKF2_core::zeroRows(Matrix24 &covMat, uint8_t first, uint8_t last)
{//Matrix24ÊÇ24*24µÄ¾ØÕó
    uint8_t row;
    for (row=first; row<=last; row++)
    {//Ò»´ÎÖ±½ÓÓÃmemset½«Ò»ÐÐµÄÊý¾Ý¸³ÖµwieÁã
        memset(&covMat[row][0], 0, sizeof(covMat[0][0])*24);//½«´ÓµØÖ·&covMat[row][0]¿ªÊ¼µÄ24µÄÔªËØµÄ¿Õ¼ä¸³ÖµÎªÁã ÎÊÌâÔÚÄÄ
    }
}

// zero specified range of columns in the state covariance matrix
//½«Ð­·½²î¾ØÕóÖÐÖ¸¶¨·¶Î§µÄÁÐ¸³ÖµÎªÁã
//Õý³£ÎÒÏëµ½µÄÊÇË«ÖØÑ­»·
void NavEKF2_core::zeroCols(Matrix24 &covMat, uint8_t first, uint8_t last)
{
    uint8_t row;
    for (row=0; row<=23; row++)//µ¥´ÎÑ­»·£¬ÓÉÓÚÓû¸³ÖµÎªÁãµÄÁÐÊÇÏàÁÚµÄ£¬¹ÊÃ¿´ÎÑ­»·¶¼¶ÔÏàÁÚµÄ¼¸¸öµØÖ·Í¬Ê±¸³ÖµÎªÁã
    {
        memset(&covMat[row][first], 0, sizeof(covMat[0][0])*(1+last-first));//¶ÔÒ»ÐÐÖÐµÄfirstÐÐµ½lastÐÐÍ¬Ê±¸³ÖµÎªÁã
    }
}

// reset the output data to the current EKF state
//ÒÔºó×Ô¼ºÐ´³ÌÐò£¬±ØÐëÑÏ¸ñ°´ÕÕ¸ÃÄ£°å½øÐÐ
/*********************************
* º¯ÊýÔ­ÐÍ:void NavEKF2_core::StoreOutputReset()
* ¹¦ÄÜËµÃ÷:¸´Î»Êä³ö
* ÐÞ¸ÄÈÕÆÚ:
* ÐÞ¸Ä×÷Õß:
* ´úÂë±¸×¢:reset the output data to the current EKF state
************************************/
//Êä³ö×´Ì¬µÄÖØÖÃ
void NavEKF2_core::StoreOutputReset()
{
    //Êä³ö±äÁ¿ÉèÖÃ outputDataNewÊÇµ±Ç°Ê±¿ÌµÄÊä³ö×´Ì¬(output:quaternion velocity position)
	outputDataNew.quat = stateStruct.quat;
    outputDataNew.velocity = stateStruct.velocity;
    outputDataNew.position = stateStruct.position;
    // write current measurement to entire table
    //½«µ±Ç°Á¿²âÐ´Èë±íÖÐ storeOutputÊä³ö×´Ì¬µÄ»º³åÇø
    for (uint8_t i=0; i<imu_buffer_length; i++) {
        storedOutput[i] = outputDataNew;
    }
    //ouputDataDelayed ºÍ outputDataNew¶¼ÊÇµ±Ç°Ê±¿ÌµÄÊä³öÊý¾Ý
	outputDataDelayed = outputDataNew;
    // reset the states for the complementary filter used to provide a vertical position dervative output
    //ÖØÖÃÓÃÓÚÌá¹©´¹Ö±Î»ÖÃÊä³öµÄ»¥²¹ÂË²¨Æ÷×´Ì¬
    //posDownDerivative ´¹Ö±Î»ÖÃ¸Ä±äµÄËÙÂÊ
	posDown = stateStruct.position.z;
    posDownDerivative = stateStruct.velocity.z;
}

// Reset the stored output quaternion history to current EKF state
void NavEKF2_core::StoreQuatReset()
{
    outputDataNew.quat = stateStruct.quat;
    // write current measurement to entire table
    for (uint8_t i=0; i<imu_buffer_length; i++) {
        storedOutput[i].quat = outputDataNew.quat;
    }
    outputDataDelayed.quat = outputDataNew.quat;
}

// Rotate the stored output quaternion history through a quaternion rotation
void NavEKF2_core::StoreQuatRotate(Quaternion deltaQuat)
{
    outputDataNew.quat = outputDataNew.quat*deltaQuat;
    // write current measurement to entire table
    for (uint8_t i=0; i<imu_buffer_length; i++) {
        storedOutput[i].quat = storedOutput[i].quat*deltaQuat;
    }
    outputDataDelayed.quat = outputDataDelayed.quat*deltaQuat;
}

// calculate nav to body quaternions from body to nav rotation matrix
//ÀûÓÃËÄÔªËØÇó×ËÌ¬¾ØÕó£¬ÇóµÃ×ËÌ¬¾ØÕó´æÔÚÔÚCbnÖÐ
void NavEKF2_core::quat2Tbn(Matrix3f &Tbn, const Quaternion &quat) const
{
    // Calculate the body to nav cosine matrix
    //void Quaternion::rotation_matrix(Matrix3f &m)const
    quat.rotation_matrix(Tbn);
}

// force symmetry on the covariance matrix to prevent ill-conditioning
//Ç¿ÖÆPÕó¶Ô³Æ£¬·ÀÖ¹ÂË²¨Æ÷³öÏÖ²¡Ì¬
//Ç¿ÖÆ¶Ô³Æ£¬ÕâÀïÎ´¿¼ÂÇÖ÷¶Ô½ÇÏßÉÏÔªËØ£¬ÒòÎªÖ÷¶Ô½ÇÉÏÔªËØÈô¸º£¬ÔòÖ±½ÓÌø¹ý±¾´ÎPÕó¸üÐÂ
void NavEKF2_core::ForceSymmetry()
{
    for (uint8_t i=1; i<=stateIndexLim; i++)//stateIndexLimÊÇ¾ØÕó»òÏòÁ¿ÔËËãÖÐ×î´óµÄÐÐ»òÁÐ
    {
        for (uint8_t j=0; j<=i-1; j++)
        {
            float temp = 0.5f*(P[i][j] + P[j][i]);
            P[i][j] = temp;
            P[j][i] = temp;
        }
    }
}

// copy covariances across from covariance prediction calculation
//¸´ÖÆ·½²î¸³¸ø¸øÊä³ö
void NavEKF2_core::CopyCovariances()
{
    // copy predicted covariances
    for (uint8_t i=0; i<=stateIndexLim; i++) {
        for (uint8_t j=0; j<=stateIndexLim; j++)
        {
            P[i][j] = nextP[i][j];
        }
    }
}

// constrain variances (diagonal terms) in the state covariance matrix to  prevent ill-conditioning
//ÏÞÖÆPÕó¶Ô½ÇÏßÔªËØ£¬·ÀÖ¹³öÏÖ²¡Ì¬Ôì³ÉÂË²¨Æ÷·¢É¢
//Ñ­»·´ÎÊý½ÏÉÙµÄ»°£¬²ÉÓÃuint8_tÀàÐÍ ¼´unsigned char
void NavEKF2_core::ConstrainVariances()
{
    for (uint8_t i=0; i<=2; i++) P[i][i] = constrain_float(P[i][i],0.0f,1.0f); // attitude error
    for (uint8_t i=3; i<=5; i++) P[i][i] = constrain_float(P[i][i],0.0f,1.0e3f); // velocities
    for (uint8_t i=6; i<=7; i++) P[i][i] = constrain_float(P[i][i],0.0f,1.0e6f);
    P[8][8] = constrain_float(P[8][8],0.0f,1.0e6f); // vertical position
    for (uint8_t i=9; i<=11; i++) P[i][i] = constrain_float(P[i][i],0.0f,sq(0.175f * dtEkfAvg)); // delta angle biases
    if (PV_AidingMode != AID_NONE) {
        for (uint8_t i=12; i<=14; i++) P[i][i] = constrain_float(P[i][i],0.0f,0.01f); // delta angle scale factors
    } else {
        // we can't reliably estimate scale factors when there is no aiding data due to transient manoeuvre induced innovations
        // so inhibit estimation by keeping covariance elements at zero
        zeroRows(P,12,14);
        zeroCols(P,12,14);
    }
    P[15][15] = constrain_float(P[15][15],0.0f,sq(10.0f * dtEkfAvg)); // delta velocity bias
    for (uint8_t i=16; i<=18; i++) P[i][i] = constrain_float(P[i][i],0.0f,0.01f); // earth magnetic field
    for (uint8_t i=19; i<=21; i++) P[i][i] = constrain_float(P[i][i],0.0f,0.01f); // body magnetic field
    for (uint8_t i=22; i<=23; i++) P[i][i] = constrain_float(P[i][i],0.0f,1.0e3f); // wind velocity
}

// constrain states to prevent ill-conditioning
//ÏÞÖÆ×´Ì¬±äÁ¿ ·ÀÖ¹ÂË²¨·¢É¢   ×´Ì¬µÄÏÞÖÆ·¶Î§ÊÇÈçºÎÈ·¶¨µÄ?
void NavEKF2_core::ConstrainStates()
{
    // attitude errors are limited between +-1
    //×ËÌ¬Îó²îÏÞÖÆ[-1,1]
    for (uint8_t i=0; i<=2; i++) statesArray[i] = constrain_float(statesArray[i],-1.0f,1.0f);
    // velocity limit 500 m/sec (could set this based on some multiple of max airspeed * EAS2TAS)
    //ËÙ¶ÈÏÞÖÆÔÚ500m/s
    for (uint8_t i=3; i<=5; i++) statesArray[i] = constrain_float(statesArray[i],-5.0e2f,5.0e2f);
    // position limit 1000 km - TODO apply circular limit
    //Î»ÖÃÏÞÖÆ1000km
    for (uint8_t i=6; i<=7; i++) statesArray[i] = constrain_float(statesArray[i],-1.0e6f,1.0e6f);
    // height limit covers home alt on everest through to home alt at SL and ballon drop
    //¸ß¶Èº­¸ÇÁË´Óº£Æ½Ãæµ½ÖéÄÂÀÊÂê·å
    stateStruct.position.z = constrain_float(stateStruct.position.z,-4.0e4f,1.0e4f);
    // gyro bias limit (this needs to be set based on manufacturers specs)
    //ÍÓÂÝÆ¯ÒÆÏÞÖÆ£¬ÐèÒª¸ù¾ÝÖÆÔì¹æ¸ñ½øÐÐÉèÖÃ
    for (uint8_t i=9; i<=11; i++) statesArray[i] = constrain_float(statesArray[i],-GYRO_BIAS_LIMIT*dtEkfAvg,GYRO_BIAS_LIMIT*dtEkfAvg);
    // gyro scale factor limit of +-5% (this needs to be set based on manufacturers specs)
    //ÍÓÂÝ±ê¶ÈÒòÊýÎó²îÎª[-0.05,0.05],¼´±ê¶ÈÒòÊý·¶Î§Îª[0.95,1.05]Ö®¼ä
    for (uint8_t i=12; i<=14; i++) statesArray[i] = constrain_float(statesArray[i],0.95f,1.05f);
    // Z accel bias limit 1.0 m/s^2	(this needs to be finalised from test data)
    //zÖá¼Ó¼ÆÆ«ÒÆÎª1.0m/s^2,ÕâÐèÒª¸ù¾Ý²âÊÔÊý¾ÝÈ·¶¨
    stateStruct.accel_zbias = constrain_float(stateStruct.accel_zbias,-1.0f*dtEkfAvg,1.0f*dtEkfAvg);
    // earth magnetic field limit µØÀíÏµÏÂ´Å³¡Ç¿¶ÈÏÞÖÆ
    for (uint8_t i=16; i<=18; i++) statesArray[i] = constrain_float(statesArray[i],-1.0f,1.0f);
    // body magnetic field limit  ÔØÌåÏµÏÂÈýÖá´Å³¡Ç¿¶ÈÏÞÖÆ
    for (uint8_t i=19; i<=21; i++) statesArray[i] = constrain_float(statesArray[i],-0.5f,0.5f);
    // wind velocity limit 100 m/s (could be based on some multiple of max airspeed * EAS2TAS) - TODO apply circular limit
    for (uint8_t i=22; i<=23; i++) statesArray[i] = constrain_float(statesArray[i],-100.0f,100.0f);
    // constrain the terrain state to be below the vehicle height unless we are using terrain as the height datum
    if (!inhibitGndState) {
        terrainState = MAX(terrainState, stateStruct.position.z + rngOnGnd);
    }
}

// calculate the NED earth spin vector in rad/sec
//¼ÆËãNEDÏÂµÄµØÇò×Ô×ª½ÇËÙÂÊ
//#define earthEate 0.000072921f rad/sec
void NavEKF2_core::calcEarthRateNED(Vector3f &omega, int32_t latitude) const
{
    float lat_rad = radians(latitude*1.0e-7f);//radians()½«¶È×ª»»Îª»¡¶È£¬latitudeÊÇintÐÍ£¬*1e-7×ª»»Îª³£¹æµÄ
    omega.x  = earthRate*cosf(lat_rad);
    omega.y  = 0;
    omega.z  = -earthRate*sinf(lat_rad);
}

// initialise the earth magnetic field states using declination, suppled roll/pitch
// and magnetometer measurements and return initial attitude quaternion
Quaternion NavEKF2_core::calcQuatAndFieldStates(float roll, float pitch)
{
    // declare local variables required to calculate initial orientation and magnetic field
    float yaw;
    Matrix3f Tbn;
    Vector3f initMagNED;
    Quaternion initQuat;

    if (use_compass()) {
        // calculate rotation matrix from body to NED frame
        Tbn.from_euler(roll, pitch, 0.0f);

        // read the magnetometer data
        readMagData();

        // rotate the magnetic field into NED axes
        initMagNED = Tbn * magDataDelayed.mag;

        // calculate heading of mag field rel to body heading
        float magHeading = atan2f(initMagNED.y, initMagNED.x);

        // get the magnetic declination
        float magDecAng = use_compass() ? _ahrs->get_compass()->get_declination() : 0;

        // calculate yaw angle rel to true north
        yaw = magDecAng - magHeading;

        // calculate initial filter quaternion states using yaw from magnetometer
        // store the yaw change so that it can be retrieved externally for use by the control loops to prevent yaw disturbances following a reset
        Vector3f tempEuler;
        stateStruct.quat.to_euler(tempEuler.x, tempEuler.y, tempEuler.z);
        // this check ensures we accumulate the resets that occur within a single iteration of the EKF
        if (imuSampleTime_ms != lastYawReset_ms) {
            yawResetAngle = 0.0f;
        }
        yawResetAngle += wrap_PI(yaw - tempEuler.z);
        lastYawReset_ms = imuSampleTime_ms;
        // calculate an initial quaternion using the new yaw value
        initQuat.from_euler(roll, pitch, yaw);
        // zero the attitude covariances becasue the corelations will now be invalid
        zeroAttCovOnly();

        // calculate initial Tbn matrix and rotate Mag measurements into NED
        // to set initial NED magnetic field states
        // don't do this if the earth field has already been learned
        if (!magFieldLearned) {
            initQuat.rotation_matrix(Tbn);
            stateStruct.earth_magfield = Tbn * magDataDelayed.mag;

            // set the NE earth magnetic field states using the published declination
            // and set the corresponding variances and covariances
            alignMagStateDeclination();

            // set the remaining variances and covariances
            zeroRows(P,18,21);
            zeroCols(P,18,21);
            P[18][18] = sq(frontend->_magNoise);
            P[19][19] = P[18][18];
            P[20][20] = P[18][18];
            P[21][21] = P[18][18];

        }

        // record the fact we have initialised the magnetic field states
        recordMagReset();

        // clear mag state reset request
        magStateResetRequest = false;

    } else {
        // this function should not be called if there is no compass data but if is is, return the
        // current attitude
        initQuat = stateStruct.quat;
    }

    // return attitude quaternion
    return initQuat;
}

// zero the attitude covariances, but preserve the variances
//½«×ËÌ¬µÄ·½²îÖÃÁã£¬µ«±£´æÕâÐ©·½²î
//why/how/what/result/question 
//Õâ¸öº¯ÊýÓÐºÎÒâÒå?½«×ËÌ¬µÄÐ­·½²î¸³Öµ¸øÖÐ¼ä±äÁ¿£¬È»ºó
void NavEKF2_core::zeroAttCovOnly()
{
    float varTemp[3];
    for (uint8_t index=0; index<=2; index++) {
        varTemp[index] = P[index][index];
    }
    zeroCols(P,0,2);
    zeroRows(P,0,2);
    for (uint8_t index=0; index<=2; index++) {
        P[index][index] = varTemp[index];
    }
}

#endif // HAL_CPU_CLASS
