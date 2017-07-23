/*================================================================================*
 *                                                                                *
 *            _    _ _____   _____   ______         _                             *
 *      /\   | |  | |  __ \ / ____| |  ____|       (_)                            *
 *     /  \  | |__| | |__) | (___   | |__ _   _ ___ _  ___  _ __                  *
 *    / /\ \ |  __  |  _  / \___ \  |  __| | | / __| |/ _ \| '_ \                 *
 *   / ____ \| |  | | | \ \ ____) | | |  | |_| \__ \ | (_) | | | |                *
 *  /_/    \_\_|  |_|_|  \_\_____/  |_|   \__,_|___/_|\___/|_| |_|                *
 *                                                                                *
 *                                                                                *
 * Nuvoton A.H.R.S Library for Cortex M4 Series                                   *
 *                                                                                *
 * Written by by T.L. Shen for Nuvoton Technology.                                *
 * tlshen@nuvoton.com/tzulan611126@gmail.com                                      *
 *                                                                                *
 *================================================================================*
 */
#ifndef __AHRS_LIB_H__
#define __AHRS_LIB_H__
#include <stdint.h>
#include <stdbool.h>
#define ROLL       0
#define PITCH      1
#define YAW      	 2

#define STATUS_NORMAL             0
#define STATUS_ERROR             -1
#define STATUS_GYRO_STEADY        0
#define STATUS_GYRO_NOT_STEADY   -1
#define STATUS_GYRO_CAL_BEGINE   -1
#define STATUS_GYRO_CAL_RUNNING   0
#define STATUS_GYRO_CAL_DONE      1
#define STATUS_GYRO_AXIS_CAL_DONE 2

#define STATUS_BUFFER_FILLED      0
#define STATUS_BUFFER_NOT_FILLED -1
#define STATUS_CAL_DONE           1

#define AXIS_X             0
#define AXIS_Y             1
#define AXIS_Z             2

#define CAL_X_UP           0
#define CAL_X_DOWN         1
#define CAL_Y_UP           2
#define CAL_Y_DOWN         3
#define CAL_Z_UP           4
#define CAL_Z_DOWN         5

#define ACC  0
#define GYRO 1
#define MAG  2
#define BARO 3
#define HALL 4
#define SENSOR_ACC        (1<<ACC)
#define SENSOR_GYRO       (1<<GYRO)
#define SENSOR_MAG        (1<<MAG)
#define SENSOR_BARO       (1<<BARO)
#define SENSOR_HALL       (1<<HALL)
#define GYRO_CAL_DATA_SIZE    6
#define ACC_CAL_DATA_SIZE     15
#define MAG_CAL_DATA_SIZE    10
#define DRIFT_TYPE_MEAN       0
#define DRIFT_TYPE_MODE       1
#define DRIFT_TYPE_MEDIAN     2
typedef struct {
  int16_t actuatorThrust;
  int16_t  actuatorRoll;
  int16_t  actuatorPitch;
  int16_t  actuatorYaw;
  int16_t  actuatorSpeed;
} ACTUATOR_T;
typedef struct  {
  float MotorOffset[2];
  float MotorScale[2];
}MotorCal_t;
void nvtGetEulerRPY(float*);
/* w x y z */
void nvtGetQuaternion(float*);
void nvtGetNormAttitude(float*);
void nvtGetNormACC(float*);
void nvtGetMAGHeading(float*);
void nvtGetEulerNormMAG(float*);
void nvtGetNormMAG(float*);
void nvtGetGYRODegree(float*);
void nvtGetVelocity(float* Velocity);
void nvtGetMove(float* Move);
void nvtSetMove(float* Move);
void nvtGetFusionSpeed(float* Speed);
void nvtResetMove(void);

void nvtGetCalibratedGYRO(float*);
void nvtGetCalibratedACC(float*);
void nvtGetCalibratedMAG(float*);
void nvtGetCalibratedHALL(float*);

void nvtAHRSInit(void);
void nvtUpdateAHRS(uint8_t UPDATE);
void nvtMillisecondTick(void);
void nvt100usecondTick(void);
void nvtInputSensorRawACC(int16_t *raw);
void nvtInputSensorRawGYRO(int16_t *raw);
void nvtInputSensorRawMAG(int16_t *raw);
void nvtInputSensorRawBARO(int16_t *raw);
void nvtInputSensorRawHALL(int16_t *raw);
void nvtInputSensorRaw9D(int16_t *RawACC, int16_t *RawGYRO, int16_t *RawMAG);

void nvtGetAccZWithoutGravity(float *ZWithoutGravity, float *AccZMag);
void nvtGetAccOffset(float*);
void nvtGetAccScale(float*);
void nvtGetAccRotate(float*);
void nvtGetGyroOffset(float* );
void nvtGetGyroScale(float*);
void nvtGetMagCalMatrix(float*);
char nvtGetAHRSID(void);
void nvtSetAccOffset(float* AccMean);
void nvtSetAccScale(float* AccScale);
void nvtSetAccRotate(float* AccRotate);
void nvtSetGyroOffset(float* GyroMean);
void nvtSetGyroScale(float* GyroScale);
void nvtSetGYRODegPLSB(float DPLSB);
void nvtSetAccG_PER_LSB(float G_PER_LSB);
void nvtSetMagCalMatrix(float* MagCalMatrix);

void nvtSetAHRSID(char id);

void nvtGetSensorRawACC(int16_t *raw);
void nvtGetSensorRawGYRO(int16_t *raw);
void nvtGetSensorRawMAG(int16_t *raw);
void nvtGetSensorRawBARO(uint16_t *raw);
void nvtGetSensorRawHALL(int16_t *raw);
void nvtGetSensorRaw9D(int16_t *RawACC, int16_t *RawGYRO, int16_t *RawMAG);
//void nvtSetSensorEnable(char SensorType, char enable);

void nvtCalACCInit(void);
signed char nvtCalACCBufferFill(int8_t Dir);

signed char nvtGyroScaleCalibrate(int8_t axis);
signed char nvtGyroCenterCalibrate(void);
signed char nvtGyroIsSteady(void);
void nvtCalGyroInit(char axis);

signed char nvtCalMAGBufferFill(void);
void nvtCalMAGInit(void);
void nvtSetMagGaussPLSB(float);
uint8_t nvtGetMagCalQFactor(void);
void nvtSmoothSensorRawData(bool enable, char sensor);
void nvtActuatorFusionFilter(ACTUATOR_T* pActuator);
void nvtGetActuatorSmooth(int16_t* actuatorSmooth);
void nvtSetMotorSmooth(MotorCal_t* MotorCal);
void nvtSetGyroDeviationTH(float TH);
float nvtGetGyroDeviationTH(void);
float nvtGetGyroDeviation(void);
void nvtSetGyroDriftType(char drifttype);
void nvtPerformanceOverAccuracy(bool Trade);
bool nvtGetPerformanceOverAccuracy(void);
void nvtResetDirection(void);
void nvtSetCalDataDefault(uint8_t);
/**
  * @brief  This function provide the sensor fusion parameter adjustment.
  *         Fine tune between the response time and stability by parameter proportional and integral. 
  * @param[proportional] Default by 0.5. "proportional" is used to control how much the new sensor data to be referenced by current oriental.
            Higher proportional means a faster response time and vise versa.  
  * @param[integral] Default by 0.03. "integral" is used to control how much the new sensor data to be referenced by current oriental for the accumulated oriental info.
            Higher integral means a faster update to the accumulated info and also result in a faster response time
  * @return None
  *
  * @details nvtSetFusionParam(float Proportional, float Integral) can update both proportional and integral to AHRSLib and 
            nvtGetFusionParam(float *Proportional, float *Integral) can get current proportional and integral from AHRSLib
  */
void nvtSetFusionParam(float Proportional, float Integral);
void nvtGetFusionParam(float *Proportional, float *Integral);
#endif	//__AHRS_LIB_H__


