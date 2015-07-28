/**
 ******************************************************************************
 * @addtogroup OpenPilotModules OpenPilot Modules
 * @{
 * @addtogroup AirspeedModule Airspeed Module
 * @brief Use attitude and velocity data to estimate airspeed
 * @{
 *
 * @file       imu_airspeed.c
 * @author     The OpenPilot Team, http://www.openpilot.org Copyright (C) 2012.
 * @brief      IMU based airspeed calculation
 *
 * @see        The GNU Public License (GPL) Version 3
 *
 *****************************************************************************/
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
 * for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include <stdio.h>       // [DEBUG] - printf, etc.
#include <stdint.h>      // int32_t, uint32_t, etc.
#include <stdbool.h>     // the bool type
#include <stdlib.h>      // malloc, etc.
#include "pios_math.h"   // constants, e.g., M_2PI_F

#include <math.h>        // cosf sinf, etc.
// [EP] replacing math.h includes
//float fabsf(float x) { return 0.0; }
//float sinf(float x) { return 0.0; }
//float asinf(float x) { return 0.0; }
//float cosf(float x) { return 0.0; }
//float tanf(float x) { return 0.0; }
//float atan2f(float x, float y) { return 0.0; }

//#include "imu_airspeed.h"
//#include "CoordinateConversions.h"
//#include <pios.h>

//#include "velocitystate.h"
typedef struct {
    float North;
    float East;
    float Down;
} __attribute__((packed)) VelocityStateDataPacked;
/*
 * Packed Object data.
 * Alignment is forced to 4 bytes so as to avoid the potential for CPU usage faults
 * on Cortex M4F during load/store of float UAVO fields
 */
typedef VelocityStateDataPacked __attribute__((aligned(4))) VelocityStateData;
/**
 * Initialize object.
 * \return 0 Success
 * \return -1 Failure to initialize or -2 for already initialized
 */
int32_t VelocityStateInitialize(void)
{
    // Compile time assertion that the VelocityStateDataPacked and VelocityStateData structs
    // have the same size (though instances of VelocityStateData
    // should be placed in memory by the linker/compiler on a 4 byte alignment).
    //PIOS_STATIC_ASSERT(sizeof(VelocityStateDataPacked) == sizeof(VelocityStateData));
    
    // Don't set the handle to null if already registered
    //if (UAVObjGetByID(VELOCITYSTATE_OBJID)) {
    //    return -2;
    //}

    // Register object with the object manager
    //handle = UAVObjRegister(VELOCITYSTATE_OBJID,
    //    VELOCITYSTATE_ISSINGLEINST, VELOCITYSTATE_ISSETTINGS, VELOCITYSTATE_ISPRIORITY, VELOCITYSTATE_NUMBYTES, &VelocityStateSetDefaults);

    // Done
    //return handle ? 0 : -1;
    return 0;
}
static int32_t VelocityStateGetInit(VelocityStateData *velocity_data) { 
  velocity_data->North = 10.55643;
  velocity_data->East  = 14.85343;
  velocity_data->Down  = 1.438790;
  return 0; 
}
static  int32_t VelocityStateGet(VelocityStateData *velocity_data) { 
  velocity_data->North = 12.780234;
  velocity_data->East  = 21.810415;
  velocity_data->Down  = 3.724428;
  return 0; 
}

// ----------------------------------------------------------------------------
//#include "attitudestate.h"
/*
 * Packed Object data (unaligned).
 * Should only be used where 4 byte alignment can be guaranteed
 * (eg a single instance on the heap)
 */
typedef struct {
    float q1;
    float q2;
    float q3;
    float q4;
    float Roll;
    float Pitch;
    float Yaw;
} __attribute__((packed)) AttitudeStateDataPacked;
/*
 * Packed Object data.
 * Alignment is forced to 4 bytes so as to avoid the potential for CPU usage 
 * faults on Cortex M4F during load/store of float UAVO fields
 */
typedef AttitudeStateDataPacked __attribute__((aligned(4))) AttitudeStateData;
    
/* Typesafe Object access functions */
static  int32_t AttitudeStateGetInit(AttitudeStateData *attitude_data) { 
  //return UAVObjGetData(AttitudeStateHandle(), dataOut); 
  //INIT ATTDATA: q1=0.744896 q2=0.000665 q3=-0.007054 q4=0.667143 Roll=-0.482583 Pitch=-0.652942 Yaw=83.699242 
  attitude_data->q1 = 0.744896;
  attitude_data->q2 = 0.000665;
  attitude_data->q3 = -0.007054;
  attitude_data->q4 = 0.667143;
  attitude_data->Roll = -0.482583;
  attitude_data->Pitch = -0.652942;
  attitude_data->Yaw = 83.699242;
  return 0;
}
static  int32_t AttitudeStateGet(AttitudeStateData *attitude_data) { 
  //return UAVObjGetData(AttitudeStateHandle(), dataOut); 
  attitude_data->q1 = 0.783642;
  attitude_data->q2 = 0.272317;
  attitude_data->q3 = -0.267048;
  attitude_data->q4 = 0.490340;
  attitude_data->Roll = 13.092840;
  attitude_data->Pitch = -43.282509;
  attitude_data->Yaw = 58.856155;
  return 0;
}

/**
 * Initialize object.
 * \return 0 Success
 * \return -1 Failure to initialize or -2 for already initialized
 */
int32_t AttitudeStateInitialize(void)
{
    // Compile time assertion that the AttitudeStateDataPacked and 
    //  AttitudeStateData structs
    // have the same size (though instances of AttitudeStateData
    // should be placed in memory by the linker/compiler on a 4 byte alignment).
    //PIOS_STATIC_ASSERT(sizeof(AttitudeStateDataPacked) 
    //                   == sizeof(AttitudeStateData));
    
    // Don't set the handle to null if already registered
    //if (UAVObjGetByID(ATTITUDESTATE_OBJID)) {
    //    return -2;
    //}

    // Register object with the object manager
    //handle = UAVObjRegister(ATTITUDESTATE_OBJID,
    //    ATTITUDESTATE_ISSINGLEINST, 
    //    ATTITUDESTATE_ISSETTINGS, 
    //    ATTITUDESTATE_ISPRIORITY, 
    //    ATTITUDESTATE_NUMBYTES, &AttitudeStateSetDefaults);

    // Done
    //return handle ? 0 : -1;
    return 0;
}
// ----------------------------------------------------------------------------


// ----------------------------------------------------------------------------
//#include "airspeedsettings.h"
/* Object constants */
#define AIRSPEEDSETTINGS_OBJID 0x69F4AD7A
#define AIRSPEEDSETTINGS_ISSINGLEINST 1
#define AIRSPEEDSETTINGS_ISSETTINGS 1
#define AIRSPEEDSETTINGS_ISPRIORITY 0
#define AIRSPEEDSETTINGS_NUMBYTES sizeof(AirspeedSettingsData)
// Enumeration options for field AirspeedSensorType
typedef enum __attribute__ ((__packed__)) {
    AIRSPEEDSETTINGS_AIRSPEEDSENSORTYPE_PIXHAWKAIRSPEEDMS4525DO=0,
    AIRSPEEDSETTINGS_AIRSPEEDSENSORTYPE_EAGLETREEAIRSPEEDV3=1,
    AIRSPEEDSETTINGS_AIRSPEEDSENSORTYPE_DIYDRONESMPXV5004=2,
    AIRSPEEDSETTINGS_AIRSPEEDSENSORTYPE_DIYDRONESMPXV7002=3,
    AIRSPEEDSETTINGS_AIRSPEEDSENSORTYPE_GROUNDSPEEDBASEDWINDESTIMATION=4,
    AIRSPEEDSETTINGS_AIRSPEEDSENSORTYPE_NONE=5
} AirspeedSettingsAirspeedSensorTypeOptions;
/*
 * Packed Object data (unaligned).
 * Should only be used where 4 byte alignment can be guaranteed
 * (eg a single instance on the heap)
 */
typedef struct {
    float Scale;
    float IMUBasedEstimationLowPassPeriod1;
    float IMUBasedEstimationLowPassPeriod2;
    uint16_t ZeroPoint;
    uint8_t SamplePeriod;
    AirspeedSettingsAirspeedSensorTypeOptions AirspeedSensorType;
} __attribute__((packed)) AirspeedSettingsDataPacked;

/*
 * Packed Object data.
 * Alignment is forced to 4 bytes so as to avoid the potential for CPU usage faults
 * on Cortex M4F during load/store of float UAVO fields
 */
typedef AirspeedSettingsDataPacked __attribute__((aligned(4))) AirspeedSettingsData;
/**
 * Initialize object.
 * \return 0 Success
 * \return -1 Failure to initialize or -2 for already initialized
 */
int32_t AirspeedSettingsInitialize(void)
{
    // Compile time assertion that the AirspeedSettingsDataPacked and AirspeedSettingsData structs
    // have the same size (though instances of AirspeedSettingsData
    // should be placed in memory by the linker/compiler on a 4 byte alignment).
    //PIOS_STATIC_ASSERT(sizeof(AirspeedSettingsDataPacked) == sizeof(AirspeedSettingsData));
    
    // Don't set the handle to null if already registered
    //if (UAVObjGetByID(AIRSPEEDSETTINGS_OBJID)) {
    //    return -2;
    //}

    // Register object with the object manager
    //handle = UAVObjRegister(AIRSPEEDSETTINGS_OBJID,
    //    AIRSPEEDSETTINGS_ISSINGLEINST, 
    //    AIRSPEEDSETTINGS_ISSETTINGS, 
    //    AIRSPEEDSETTINGS_ISPRIORITY, 
    //    AIRSPEEDSETTINGS_NUMBYTES, 
    // &AirspeedSettingsSetDefaults);

    // Done
    //return handle ? 0 : -1;
    return 0;
}
    
/* Typesafe Object access functions */
static  int32_t AirspeedSettingsGet(AirspeedSettingsData *dataOut) { 
  //return UAVObjGetData(AirspeedSettingsHandle(), dataOut); 
  // TODO: init settings
  return 0;
}

// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
//#include "airspeedsensor.h"
/* Object constants */
#define AIRSPEEDSENSOR_OBJID 0x438D5F9A
#define AIRSPEEDSENSOR_ISSINGLEINST 1
#define AIRSPEEDSENSOR_ISSETTINGS 0
#define AIRSPEEDSENSOR_ISPRIORITY 0
#define AIRSPEEDSENSOR_NUMBYTES sizeof(AirspeedSensorData)
// Enumeration options for field SensorConnected
typedef enum __attribute__ ((__packed__)) {
    AIRSPEEDSENSOR_SENSORCONNECTED_FALSE=0,
    AIRSPEEDSENSOR_SENSORCONNECTED_TRUE=1
} AirspeedSensorSensorConnectedOptions;

/*
 * Packed Object data (unaligned).
 * Should only be used where 4 byte alignment can be guaranteed
 * (eg a single instance on the heap)
 */
typedef struct {
    float DifferentialPressure;
    float Temperature;
    float CalibratedAirspeed;
    float TrueAirspeed;
    uint16_t SensorValue;
    uint16_t SensorValueTemperature;
    AirspeedSensorSensorConnectedOptions SensorConnected;
} __attribute__((packed)) AirspeedSensorDataPacked;

/*
 * Packed Object data.
 * Alignment is forced to 4 bytes so as to avoid the potential for CPU usage faults
 * on Cortex M4F during load/store of float UAVO fields
 */
typedef AirspeedSensorDataPacked __attribute__((aligned(4))) AirspeedSensorData;
    

/**
 * Initialize object.
 * \return 0 Success
 * \return -1 Failure to initialize or -2 for already initialized
 */
int32_t AirspeedSensorInitialize(void)
{
    // Compile time assertion that the AirspeedSensorDataPacked and AirspeedSensorData structs
    // have the same size (though instances of AirspeedSensorData
    // should be placed in memory by the linker/compiler on a 4 byte alignment).
    //PIOS_STATIC_ASSERT(sizeof(AirspeedSensorDataPacked) == sizeof(AirspeedSensorData));
    
    // Don't set the handle to null if already registered
    //if (UAVObjGetByID(AIRSPEEDSENSOR_OBJID)) {
    //    return -2;
    //}

    // Register object with the object manager
    //handle = UAVObjRegister(AIRSPEEDSENSOR_OBJID,
    //    AIRSPEEDSENSOR_ISSINGLEINST, 
    //    AIRSPEEDSENSOR_ISSETTINGS, 
    //    AIRSPEEDSENSOR_ISPRIORITY, 
    //    AIRSPEEDSENSOR_NUMBYTES, &AirspeedSensorSetDefaults);

    // Done
    //return handle ? 0 : -1;
    return 0;
}

/* Typesafe Object access functions */
static  int32_t AirspeedSensorGet(AirspeedSensorData *dataOut) { 
  // return UAVObjGetData(AirspeedSensorHandle(), dataOut); 
  // this is writeonly object
  return 0;
}
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
typedef enum {
    SYSTEMALARMS_ALARM_SYSTEMCONFIGURATION=0,
    SYSTEMALARMS_ALARM_BOOTFAULT=1,
    SYSTEMALARMS_ALARM_OUTOFMEMORY=2,
    SYSTEMALARMS_ALARM_STACKOVERFLOW=3,
    SYSTEMALARMS_ALARM_CPUOVERLOAD=4,
    SYSTEMALARMS_ALARM_EVENTSYSTEM=5,
    SYSTEMALARMS_ALARM_TELEMETRY=6,
    SYSTEMALARMS_ALARM_RECEIVER=7,
    SYSTEMALARMS_ALARM_MANUALCONTROL=8,
    SYSTEMALARMS_ALARM_ACTUATOR=9,
    SYSTEMALARMS_ALARM_ATTITUDE=10,
    SYSTEMALARMS_ALARM_SENSORS=11,
    SYSTEMALARMS_ALARM_MAGNETOMETER=12,
    SYSTEMALARMS_ALARM_AIRSPEED=13,
    SYSTEMALARMS_ALARM_STABILIZATION=14,
    SYSTEMALARMS_ALARM_GUIDANCE=15,
    SYSTEMALARMS_ALARM_PATHPLAN=16,
    SYSTEMALARMS_ALARM_BATTERY=17,
    SYSTEMALARMS_ALARM_FLIGHTTIME=18,
    SYSTEMALARMS_ALARM_I2C=19,
    SYSTEMALARMS_ALARM_GPS=20
} SystemAlarmsAlarmElem;
/**
 * Clear an alarm
 * @param alarm The system alarm to be modified
 * @return 0 if success, -1 if an error
 */
int32_t AlarmsClear(SystemAlarmsAlarmElem alarm)
{
   
  //if (alarm < SYSTEMALARMS_EXTENDEDALARMSTATUS_NUMELEM) {
  //      return ExtendedAlarmsSet(alarm, SYSTEMALARMS_ALARM_OK, SYSTEMALARMS_EXTENDEDALARMSTATUS_NONE, 0);
  //  } else {
  //      return AlarmsSet(alarm, SYSTEMALARMS_ALARM_OK);
  //  }
  return 0;
}
// ----------------------------------------------------------------------------
//#include "butterworth.h" // butterworth filter
// ----------------------------------------------------------------------------
// Coefficients of second order Butterworth biquadratic filter in direct from 2
struct ButterWorthDF2Filter {
    float b0;
    float a1;
    float a2;
};
/**
 * Initialization function for coefficients of a second order Butterworth biquadratic filter in direct from 2.
 * Note that b1  = 2 * b0 and b2  = b0 is use here and in the sequel.
 * @param[in]  ff Cut-off frequency ratio
 * @param[out] filterPtr Pointer to filter coefficients
 * @returns Nothing
 */
void InitButterWorthDF2Filter(const float ff, struct ButterWorthDF2Filter *filterPtr)
{
    const float ita = 1.0f / tanf(M_PI_F * ff);
    const float b0  = 1.0f / (1.0f + M_SQRT2_F * ita + ita * ita);
    const float a1  = 2.0f * b0 * (ita * ita - 1.0f);
    const float a2  = -b0 * (1.0f - M_SQRT2_F * ita + ita * ita);

    filterPtr->b0 = b0;
    filterPtr->a1 = a1;
    filterPtr->a2 = a2;
}


/**
 * Initialization function for intermediate values of a second order Butterworth biquadratic filter in direct from 2.
 * Obtained by solving a linear equation system.
 * @param[in]  x0 Prescribed value
 * @param[in]  filterPtr Pointer to filter coefficients
 * @param[out] wn1Ptr Pointer to first intermediate value
 * @param[out] wn2Ptr Pointer to second intermediate value
 * @returns Nothing
 */
void InitButterWorthDF2Values(const float x0, const struct ButterWorthDF2Filter *filterPtr, float *wn1Ptr, float *wn2Ptr)
{
    const float b0   = filterPtr->b0;
    const float a1   = filterPtr->a1;
    const float a2   = filterPtr->a2;

    const float a11  = 2.0f + a1;
    const float a12  = 1.0f + a2;
    const float a21  = 2.0f + a1 * a1 + a2;
    const float a22  = 1.0f + a1 * a2;
    const float det  = a11 * a22 - a12 * a21;
    const float rhs1 = x0 / b0 - x0;
    const float rhs2 = x0 / b0 - x0 + a1 * x0;

    *wn1Ptr = (a22 * rhs1 - a12 * rhs2) / det;
    *wn2Ptr = (-a21 * rhs1 + a11 * rhs2) / det;
}


/**
 * Second order Butterworth biquadratic filter in direct from 2, such that only two values wn1=w[n-1] and wn2=w[n-2] need to be stored.
 * Function takes care of updating the values wn1 and wn2.
 * @param[in]  xn New raw value
 * @param[in]  filterPtr Pointer to filter coefficients
 * @param[out] wn1Ptr Pointer to first intermediate value
 * @param[out] wn2Ptr Pointer to second intermediate value
 * @returns Filtered value
 */
float FilterButterWorthDF2(const float xn, const struct ButterWorthDF2Filter *filterPtr, float *wn1Ptr, float *wn2Ptr)
{
    const float wn  = xn + filterPtr->a1 * (*wn1Ptr) + filterPtr->a2 * (*wn2Ptr);
    const float val = filterPtr->b0 * (wn + 2.0f * (*wn1Ptr) + (*wn2Ptr));

    *wn2Ptr = *wn1Ptr;
    *wn1Ptr = wn;
    return val;
}
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
// [EP] Custom defines
#define pios_malloc(size)         (malloc(size))
// ----------------------------------------------------------------------------


// Private constants
#define EPS               1e-6f
#define EPS_REORIENTATION 1e-10f
#define EPS_VELOCITY      1.f

// Private types
// structure with smoothed fuselage orientation, ground speed, wind vector and their changes in time
struct IMUGlobals {
    // Butterworth filters
    struct ButterWorthDF2Filter filter;
    struct ButterWorthDF2Filter prefilter;
    float ff, ffV;

    // storage variables for Butterworth filter
    float pn1, pn2;
    float yn1, yn2;
    float v1n1, v1n2;
    float v2n1, v2n2;
    float v3n1, v3n2;
    float Vw1n1, Vw1n2;
    float Vw2n1, Vw2n2;
    float Vw3n1, Vw3n2;
    float Vw1, Vw2, Vw3;

    // storage variables for derivative calculation
    float pOld, yOld;
    float v1Old, v2Old, v3Old;
};


// Private variables
static struct IMUGlobals *imu;

// Private functions
// a simple square inline function based on multiplication faster than powf(x,2.0f)
static  float Sq(float x)
{
    return x * x;
}

// ****** find pitch, yaw from quaternion ********
static void Quaternion2PY(const float q0, const float q1, const float q2, const float q3, float *pPtr, float *yPtr, bool principalArg)
{
    float R13, R11, R12;
    const float q0s = q0 * q0;
    const float q1s = q1 * q1;
    const float q2s = q2 * q2;
    const float q3s = q3 * q3;

    R13   = 2.0f * (q1 * q3 - q0 * q2);
    R11   = q0s + q1s - q2s - q3s;
    R12   = 2.0f * (q1 * q2 + q0 * q3);

    *pPtr = asinf(-R13); // pitch always between -pi/2 to pi/2

    const float y_ = atan2f(R12, R11);
    // use old yaw contained in y to add multiples of 2pi to have a continuous yaw if user does not want the principal argument
    // else simply copy atan2 result into result
    if (principalArg) {
        *yPtr = y_;
    } else {
        // calculate needed mutliples of 2pi to avoid jumps
        // number of cycles accumulated in old yaw
        const int32_t cycles = (int32_t)(*yPtr / M_2PI_F);
        // look for a jump by substracting the modulus, i.e. there is maximally one jump.
        // take slightly less than 2pi, because the jump will always be lower than 2pi
        const int32_t mod    = (int32_t)((y_ - (*yPtr - cycles * M_2PI_F)) / (M_2PI_F * 0.8f));
        *yPtr = y_ + M_2PI_F * (cycles - mod);
    }
}

static void PY2xB(const float p, const float y, float x[3])
{
    const float cosp = cosf(p);

    x[0] = cosp * cosf(y);
    x[1] = cosp * sinf(y);
    x[2] = -sinf(p);
}


static void PY2DeltaxB(const float p, const float y, const float xB[3], float x[3])
{
    const float cosp = cosf(p);

    x[0] = xB[0] - cosp * cosf(y);
    x[1] = xB[1] - cosp * sinf(y);
    x[2] = xB[2] - -sinf(p);
}


/*
 * Initialize function loads first data sets, and allocates memory for structure.
 */
void imu_airspeedInitialize(const AirspeedSettingsData *airspeedSettings)
{
    // pre-filter frequency rate
    const float ff  = (float)(airspeedSettings->SamplePeriod) / 1000.0f / airspeedSettings->IMUBasedEstimationLowPassPeriod1;
    // filter frequency rate
    const float ffV = (float)(airspeedSettings->SamplePeriod) / 1000.0f / airspeedSettings->IMUBasedEstimationLowPassPeriod2;

    // This method saves memory in case we don't use the module.
    imu = (struct IMUGlobals *)pios_malloc(sizeof(struct IMUGlobals));

    // airspeed calculation variables
    VelocityStateInitialize();
    VelocityStateData velData;
    VelocityStateGetInit(&velData);

    AttitudeStateData attData;
    AttitudeStateGetInit(&attData);

    // initialize filters for given ff and ffV
    InitButterWorthDF2Filter(ffV, &(imu->filter));
    InitButterWorthDF2Filter(ff, &(imu->prefilter));
    imu->ffV = ffV;
    imu->ff  = ff;

    // get pitch and yaw from quarternion; principal argument for yaw
    Quaternion2PY(attData.q1, attData.q2, attData.q3, attData.q4, &(imu->pOld), &(imu->yOld), true);
    InitButterWorthDF2Values(imu->pOld, &(imu->prefilter), &(imu->pn1), &(imu->pn2));
    InitButterWorthDF2Values(imu->yOld, &(imu->prefilter), &(imu->yn1), &(imu->yn2));

    // use current NED speed as vOld vector and as initial value for filter
    imu->v1Old = velData.North;
    imu->v2Old = velData.East;
    imu->v3Old = velData.Down;
    InitButterWorthDF2Values(imu->v1Old, &(imu->prefilter), &(imu->v1n1), &(imu->v1n2));
    InitButterWorthDF2Values(imu->v2Old, &(imu->prefilter), &(imu->v2n1), &(imu->v2n2));
    InitButterWorthDF2Values(imu->v3Old, &(imu->prefilter), &(imu->v3n1), &(imu->v3n2));

    // initial guess for windspeed is zero
    imu->Vw3   = imu->Vw2 = imu->Vw1 = 0.0f;
    InitButterWorthDF2Values(0.0f, &(imu->filter), &(imu->Vw1n1), &(imu->Vw1n2));
    imu->Vw3n1 = imu->Vw2n1 = imu->Vw1n1;
    imu->Vw3n2 = imu->Vw2n2 = imu->Vw1n2;
}

/*
 * Calculate airspeed as a function of groundspeed and vehicle attitude.
 *  Adapted from "IMU Wind Estimation (Theory)", by William Premerlani.
 *  The idea is that V_gps=V_air+V_wind. If we assume wind constant, =>
 *  V_gps_2-V_gps_1 = (V_air_2+V_wind_2) -(V_air_1+V_wind_1) = V_air_2 - V_air_1.
 *  If we assume airspeed constant, => V_gps_2-V_gps_1 = |V|*(f_2 - f1),
 *  where "f" is the fuselage vector in earth coordinates.
 *  We then solve for |V| = |V_gps_2-V_gps_1|/ |f_2 - f1|.
 *  Adapted to: |V| = (V_gps_2-V_gps_1) dot (f2_-f_1) / |f_2 - f1|^2.
 *
 * See OP-1317 imu_wind_estimation.pdf for details on the adaptation
 * Need a low pass filter to filter out spikes in non coordinated maneuvers
 * A two step Butterworth second order filter is used. In the first step fuselage vector xB
 * and ground speed vector Vel are filtered. The fuselage vector is filtered through its pitch
 * and yaw to keep a unit length. After building the differenced dxB and dVel are produced and
 * the airspeed calculated. The calculated airspeed is filtered again with a Butterworth filter
 */

void imu_airspeedGet(AirspeedSensorData *airspeedData, const AirspeedSettingsData *airspeedSettings)
{
    // pre-filter frequency rate
    const float ff  = (float)(airspeedSettings->SamplePeriod) / 1000.0f / airspeedSettings->IMUBasedEstimationLowPassPeriod1;
    // filter frequency rate
    const float ffV = (float)(airspeedSettings->SamplePeriod) / 1000.0f / airspeedSettings->IMUBasedEstimationLowPassPeriod2;

    // check for a change in filter frequency rate. if yes, then actualize filter constants and intermediate values
    if (fabsf(ffV - imu->ffV) > EPS) {
        InitButterWorthDF2Filter(ffV, &(imu->filter));
        InitButterWorthDF2Values(imu->Vw1, &(imu->filter), &(imu->Vw1n1), &(imu->Vw1n2));
        InitButterWorthDF2Values(imu->Vw2, &(imu->filter), &(imu->Vw2n1), &(imu->Vw2n2));
        InitButterWorthDF2Values(imu->Vw3, &(imu->filter), &(imu->Vw3n1), &(imu->Vw3n2));
    }
    if (fabsf(ff - imu->ff) > EPS) {
        InitButterWorthDF2Filter(ff, &(imu->prefilter));
        InitButterWorthDF2Values(imu->pOld, &(imu->prefilter), &(imu->pn1), &(imu->pn2));
        InitButterWorthDF2Values(imu->yOld, &(imu->prefilter), &(imu->yn1), &(imu->yn2));
        InitButterWorthDF2Values(imu->v1Old, &(imu->prefilter), &(imu->v1n1), &(imu->v1n2));
        InitButterWorthDF2Values(imu->v2Old, &(imu->prefilter), &(imu->v2n1), &(imu->v2n2));
        InitButterWorthDF2Values(imu->v3Old, &(imu->prefilter), &(imu->v3n1), &(imu->v3n2));
    }

    float normVel2;
    float normDiffAttitude2;
    float dvdtDotdfdt;

    float xB[3];
    // get values and conduct smoothing of ground speed and orientation independently of the calculation of airspeed
    { // Scoping to save memory
        AttitudeStateData attData;
   
        AttitudeStateGet(&attData);
        VelocityStateData velData;
        VelocityStateGet(&velData);
        float p = imu->pOld, y = imu->yOld;
        float dxB[3];

        // get pitch and roll Euler angles from quaternion
        // do not calculate the principlal argument of yaw, i.e. use old yaw to add multiples of 2pi to have a continuous yaw
        Quaternion2PY(attData.q1, attData.q2, attData.q3, attData.q4, &p, &y, false);

        // filter pitch and roll Euler angles instead of fuselage vector to guarantee a unit length at all times
        p = FilterButterWorthDF2(p, &(imu->prefilter), &(imu->pn1), &(imu->pn2));
        y = FilterButterWorthDF2(y, &(imu->prefilter), &(imu->yn1), &(imu->yn2));
        // transform pitch and yaw into fuselage vector xB and xBold
        PY2xB(p, y, xB);
        // calculate change in fuselage vector by substraction of old value
        PY2DeltaxB(imu->pOld, imu->yOld, xB, dxB);

        // filter ground speed from VelocityState
        const float fv1n = FilterButterWorthDF2(velData.North, &(imu->prefilter), &(imu->v1n1), &(imu->v1n2));
        const float fv2n = FilterButterWorthDF2(velData.East, &(imu->prefilter), &(imu->v2n1), &(imu->v2n2));
        const float fv3n = FilterButterWorthDF2(velData.Down, &(imu->prefilter), &(imu->v3n1), &(imu->v3n2));

        // calculate norm of ground speed
        normVel2 = Sq(fv1n) + Sq(fv2n) + Sq(fv3n);
        // calculate norm of orientation change
        normDiffAttitude2 = Sq(dxB[0]) + Sq(dxB[1]) + Sq(dxB[2]);
        // cauclate scalar product between groundspeed change and orientation change
        dvdtDotdfdt = (fv1n - imu->v1Old) * dxB[0] + (fv2n - imu->v2Old) * dxB[1] + (fv3n - imu->v3Old) * dxB[2];

        // actualise old values
        imu->pOld   = p;
        imu->yOld   = y;
        imu->v1Old  = fv1n;
        imu->v2Old  = fv2n;
        imu->v3Old  = fv3n;
    }

    // Some reorientation needed to be able to calculate airspeed, calculate only for sufficient velocity
    // a negative scalar product is a clear sign that we are not really able to calculate the airspeed
    // NOTE: normVel2 check against EPS_VELOCITY might make problems during hovering maneuvers in fixed wings
    if (normDiffAttitude2 > EPS_REORIENTATION && normVel2 > EPS_VELOCITY && dvdtDotdfdt > 0.f) {
        // Airspeed modulus: |v| = dv/dt * dxB/dt / |dxB/dt|^2
        // airspeed is always REAL because  normDiffAttitude2 > EPS_REORIENTATION > 0 and REAL dvdtDotdfdt
        const float airspeed = dvdtDotdfdt / normDiffAttitude2;

        // groundspeed = airspeed + wind ---> wind = groundspeed - airspeed
        const float wind[3]  = { imu->v1Old - xB[0] * airspeed,
                                 imu->v2Old - xB[1] * airspeed,
                                 imu->v3Old - xB[2] * airspeed };
        // filter raw wind
        imu->Vw1 = FilterButterWorthDF2(wind[0], &(imu->filter), &(imu->Vw1n1), &(imu->Vw1n2));
        imu->Vw2 = FilterButterWorthDF2(wind[1], &(imu->filter), &(imu->Vw2n1), &(imu->Vw2n2));
        imu->Vw3 = FilterButterWorthDF2(wind[2], &(imu->filter), &(imu->Vw3n1), &(imu->Vw3n2));
    } // else leave wind estimation unchanged

    { // Scoping to save memory
      // airspeed = groundspeed - wind
        const float Vair[3] = {
            imu->v1Old - imu->Vw1,
            imu->v2Old - imu->Vw2,
            imu->v3Old - imu->Vw3
        };

        // project airspeed into fuselage vector
        airspeedData->CalibratedAirspeed = Vair[0] * xB[0] + Vair[1] * xB[1] + Vair[2] * xB[2];
        //ENSURES: CalibratedAirspeed=14.688994 (harness0)
        printf("CalibratedAirspeed: %f \n", (double) airspeedData->CalibratedAirspeed);
    }

    airspeedData->SensorConnected = AIRSPEEDSENSOR_SENSORCONNECTED_TRUE;
    AlarmsClear(SYSTEMALARMS_ALARM_AIRSPEED);
}

void init_harness0(AirspeedSettingsData* airspeed_settings_data) {
  airspeed_settings_data->SamplePeriod   = 43;
  airspeed_settings_data->IMUBasedEstimationLowPassPeriod1 = 100.000000;
  airspeed_settings_data->IMUBasedEstimationLowPassPeriod2 = 0.5000000;
}


void harness0(VelocityStateData* velocity_data, AttitudeStateData* attitude_data, AirspeedSettingsData* airspeed_settings_data) {
  //VELDATA: North=12.780234 East=21.810415 Down=3.724428 
  velocity_data->North= 12.780234;
  velocity_data->East = 21.810415;
  velocity_data->Down = 3.724428;

  //ATTDATA: q1=0.783642 q2=0.272317 q3=-0.267048 q4=0.490340 Roll=13.092840 Pitch=-43.282509 Yaw=58.856155 
  attitude_data->q1 = 0.783642;
  attitude_data->q2 = 0.272317;
  attitude_data->q3 = -0.267048;
  attitude_data->q4 = 0.490340;
  attitude_data->Roll = 13.092840;
  attitude_data->Pitch = -43.282509;
  attitude_data->Yaw = 58.856155;

  //SETTINGS: SamplePeriod=4300459 LowPassPeriod1=100.000000 LowPassPeriod2=0.500000 
  airspeed_settings_data->SamplePeriod   = 43;
  airspeed_settings_data->IMUBasedEstimationLowPassPeriod1 = 100.000000;
  airspeed_settings_data->IMUBasedEstimationLowPassPeriod2 = 0.5000000;
  


}

int main() {
  
  VelocityStateData* velocity_data = (VelocityStateData*) malloc(sizeof(VelocityStateData));
  AttitudeStateData* attitude_data = (AttitudeStateData*) malloc(sizeof(AttitudeStateData));
  AirspeedSettingsData* airspeed_settings_data = (AirspeedSettingsData*) malloc(sizeof(AirspeedSettingsData));
  AirspeedSensorData* airspeed_sensor_data = (AirspeedSensorData*) malloc(sizeof(AirspeedSensorData));

  init_harness0(airspeed_settings_data);
  imu_airspeedInitialize(airspeed_settings_data);

  harness0(velocity_data, attitude_data, airspeed_settings_data);
  imu_airspeedGet(airspeed_sensor_data, airspeed_settings_data);
  
  printf("DONE. \n");
  return 0;
}
