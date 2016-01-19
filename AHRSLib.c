/*================================================================================*
 * O     O          __             ______  __   __  ____     __  ___          __  *
 *  \   /      /\  / /_      _    / /___/ / /  / / / __ \   / / /   \    /\  / /  *
 *   [+]      /  \/ / \\    //   / /____ / /  / /  \ \_    / / | | | |  /  \/ /   *
 *  /   \    / /\  /   \\__//   / /----// /__/ /  \ \__ \ / /  | | | | / /\  /    *
 * O     O  /_/  \/     \__/   /_/      \_ ___/    \___ //_/    \___/ /_/  \/     *
 *                                                                                *
 *                                                                                *
 * Nuvoton A.H.R.S Library for Cortex M4 Series                                   *
 *                                                                                *
 * Written by by T.L. Shen for Nuvoton Technology.                                *
 * tlshen@nuvoton.com/tzulan611126@gmail.com                                      *
 *                                                                                *
 *================================================================================*
 */
#include <stdio.h>
#include "SensorFusion6.h"
#include "AHRSLib.h"
#include "Common.h"
#include "Security.h"
#include <math.h>
CalibrateInfo CalInfo;
SensorStateInfo SensorState;
TimeFrameInfo TimeInfo;
AttitudeInfo_T AttitudeInfo;
uint32_t msTick;
char DIM[MAX_SENSOR] = {ACC_DIM, GYRO_DIM, MAG_DIM, BARO_DIM};
float period;
float ZWithoutG_Base = 0;
static int16_t    accelLPF[3];
void SetZWithoutG_Base(float base)
{
	ZWithoutG_Base = base;
}
float GetAHRSPeriod()
{
	return period;
}
void TimerSet()
{
	TimeInfo.currentTime = TimeInfo.tick_counter;
  TimeInfo.lastTime = TimeInfo.currentTime;
}
void TimerStart()
{
	TimeInfo.currentTime = TimeInfo.lastTime = TimeInfo.tick_counter;
}
float TimerRead()
{
	return (TimeInfo.tick_counter-TimeInfo.currentTime)/10000.0f;
}


void SensorStateInit() 
{
	SensorState.beCalibrated[ACC]  = false;
	SensorState.beCalibrated[GYRO] = false;
	SensorState.beCalibrated[MAG]  = false;
	
	SensorState.beSensorEnable[ACC]  = false;
	SensorState.beSensorEnable[GYRO] = false;
	SensorState.beSensorEnable[MAG]  = false;
	
	SensorState.beGyroSteady       = false;
	SensorState.beGyroSteadyPrev   = false;
	SensorState.GyroDynamicCenter[AXIS_X] = 0;
	SensorState.GyroDynamicCenter[AXIS_Y] = 0;
	SensorState.GyroDynamicCenter[AXIS_Z] = 0;
	SensorState.RawSmooth.Index[ACC] = 0;
	SensorState.RawSmooth.enable[ACC] = false;
	SensorState.RawSmooth.Index[GYRO] = 0;
	SensorState.RawSmooth.enable[GYRO] = false;
	SensorState.RawSmooth.Index[MAG] = 0;
	SensorState.RawSmooth.enable[MAG] = true;
	SensorState.RawSmooth.Index[BARO] = 0;
	SensorState.RawSmooth.enable[BARO] = true;
}
void CalInfoInit()
{
	CalInfo.AccMean[0] = 0;
	CalInfo.AccMean[1] = 0;
	CalInfo.AccMean[2] = 0;
	CalInfo.AccScale[0] = 1;
	CalInfo.AccScale[1] = 1;
	CalInfo.AccScale[2] = 1;
	
	CalInfo.GyroMean[0] = 0;
	CalInfo.GyroMean[1] = 0;
	CalInfo.GyroMean[2] = 0;
	CalInfo.GyroScale[0] = 1;
	CalInfo.GyroScale[1] = 1;
	CalInfo.GyroScale[2] = 1;
	
	CalInfo.MagInvW[0] = 0;
	CalInfo.MagInvW[1] = 0;
	CalInfo.MagInvW[2] = 0;
  CalInfo.MagInvW[3] = 0.0002;
	CalInfo.MagInvW[4] = 0.0002;
	CalInfo.MagInvW[5] = 0.0002;
  CalInfo.MagInvW[6] = 0.000f;
	CalInfo.MagInvW[7] = 0.000f;
	CalInfo.MagInvW[8] = 0.000f;
	CalInfo.MagInvW[9] = 1.0;
}
void TimeInfoInit()
{
	TimeInfo.lastTime = 0;
	TimeInfo.interval = 0;
	TimeInfo.currentTime = 0;
	TimeInfo.tick_counter = 0;
}
void nvtAHRSInit()
{
	SetupSecurityID();
	GUARD
	SensorStateInit();
	CalInfoInit();
	TimeInfoInit();
	TimerStart();
	AccFilterInit();
}
void nvtGetEulerRPY(float* rpy)
{
	sensfusion6GetEulerRPY(&AttitudeInfo.Euler[ROLL], &AttitudeInfo.Euler[PITCH], &AttitudeInfo.Euler[YAW]);
	rpy[0] = AttitudeInfo.Euler[ROLL];
	rpy[1] = AttitudeInfo.Euler[PITCH];
	rpy[2] = AttitudeInfo.Euler[YAW];
}
void nvtGetQuaternion(float *Qua)
{
	Axis4f Quaternion;
	sensfusion6Getquaternion(&Quaternion);
	Qua[0] = Quaternion.w;
	Qua[1] = Quaternion.x;
	Qua[2] = Quaternion.y;
	Qua[3] = Quaternion.z;
}
void nvtGetNormAttitude(float* Att)
{
	sensfusion6GetGyro(&AttitudeInfo.NormAttitude[0]);
	Att[0] = AttitudeInfo.NormAttitude[0];
	Att[1] = AttitudeInfo.NormAttitude[1];
	Att[2] = AttitudeInfo.NormAttitude[2];
}
void nvtGetNormACC(float* NormACC)
{
	sensfusion6GetAcc(&AttitudeInfo.NormACC[0]);
	NormACC[0] = AttitudeInfo.NormACC[0];
	NormACC[1] = AttitudeInfo.NormACC[1];
	NormACC[2] = AttitudeInfo.NormACC[2];
}
void nvtGetMAGHeading(float* Heading)
{
	sensfusion6GetMagDegree(&AttitudeInfo.MAGHead[0]);
	Heading[0] = AttitudeInfo.MAGHead[0];
	Heading[1] = AttitudeInfo.MAGHead[1];
	Heading[2] = AttitudeInfo.MAGHead[2];
}
void nvtGetEulerNormMAG(float* NormMAG)
{
	sensfusion6GetMagEuler(&AttitudeInfo.EulerNormMAG[0]);
	NormMAG[0] = AttitudeInfo.EulerNormMAG[0];
	NormMAG[1] = AttitudeInfo.EulerNormMAG[1];
	NormMAG[2] = AttitudeInfo.EulerNormMAG[2];
}
void nvtGetNormMAG(float* NormMAG)
{
	sensfusion6GetNormMag(&AttitudeInfo.NormMAG[0]);
	NormMAG[0] = AttitudeInfo.NormMAG[0];
	NormMAG[1] = AttitudeInfo.NormMAG[1];
	NormMAG[2] = AttitudeInfo.NormMAG[2];
}
void nvtGetGYRODegree(float* Deg)
{
	sensfusion6GetGyroDeg(&AttitudeInfo.GYRODeg[0]);
	Deg[0] = AttitudeInfo.GYRODeg[0];
	Deg[1] = AttitudeInfo.GYRODeg[1];
	Deg[2] = AttitudeInfo.GYRODeg[2];
}
void nvtGetVelocity(float* Velocity)
{
	sensfusion6GetVelocity(Velocity);
}
void nvtGetMove(float* Move)
{
	sensfusion6GetMove(Move);
}
void nvtSetMove(float* Move)
{
	sensfusion6SetMove(Move);
}
void nvtResetMove()
{
	sensfusion6ResetMove();
}
void nvtUpdateAHRS(uint8_t UPDATE)
{
	float CACC[3]={0,0,0}, CGYRO[3]={0,0,0}, CMAG[3]={0,0,0};
	GUARD
	if(UPDATE&SENSOR_ACC){
		CACC[0] = AttitudeInfo.CalACC[0];
		CACC[1] = AttitudeInfo.CalACC[1];
		CACC[2] = AttitudeInfo.CalACC[2];
	}
	if(UPDATE&SENSOR_GYRO){
		CGYRO[0] = AttitudeInfo.CalGYRO[0];
		CGYRO[1] = AttitudeInfo.CalGYRO[1];
		CGYRO[2] = AttitudeInfo.CalGYRO[2];
	}
	if(UPDATE&SENSOR_MAG){
		CMAG[0] = AttitudeInfo.CalMAG[0];
		CMAG[1] = AttitudeInfo.CalMAG[1];
		CMAG[2] = AttitudeInfo.CalMAG[2];
	}
	sensfusion9UpdateQ(CGYRO[0], CGYRO[1], CGYRO[2], 
										 CACC[0], CACC[1], CACC[2], 
										 CMAG[0], CMAG[1], CMAG[2], period);

	period = TimerRead();
	TimerSet();
	sensfusion6UpdateMagByEuler();
}
void nvtMillisecondTick()
{
	GUARD
	TimeInfo.tick_counter+=10;
}
void nvt100usecondTick()
{
	GUARD
	TimeInfo.tick_counter++;
}
uint32_t GetTickCounter()
{
	return TimeInfo.tick_counter;
}
void RawSmoothInput(char Sensor, int16_t *raw)
{
	char i, RingOffset = 0;
	for(i=0; i<Sensor;i++) {
		RingOffset+=DIM[i];
	}
	for(i=0; i<DIM[Sensor];i++) {
		SensorState.RawSmooth.RingBuffer[RingOffset+i][SensorState.RawSmooth.Index[Sensor]] = raw[i];
	}

	SensorState.RawSmooth.Index[Sensor]++;
	SensorState.RawSmooth.Index[Sensor] = SensorState.RawSmooth.Index[Sensor]%RAW_SMOOTH_BUFFER_SIZE;
	//printf("inp: %d   %d   %d\n", raw[0], raw[1], raw[2]);
}
void RawSmoothOutput(char Sensor, float *raw)
{
	char i, j, RingOffset = 0;
	int32_t average=0;

	for(i=0; i<Sensor;i++) {
		RingOffset+=DIM[i];
		}
	for(i=0; i<DIM[Sensor];i++) {
		for(j=0; j<RAW_SMOOTH_BUFFER_SIZE; j++)
			average+=SensorState.RawSmooth.RingBuffer[RingOffset+i][j];
		raw[i] = average/RAW_SMOOTH_BUFFER_SIZE;
		average = 0;
	}
	//printf("out: %d   %d   %d\n", (int16_t)raw[0], (int16_t)raw[1], (int16_t)raw[2]);
}

void nvtInputSensorRawACC(int16_t *raw)
{
	GUARD
	//RawSmoothInput(ACC, raw);
	if(SensorState.RawSmooth.enable[ACC]) {
		//RawSmoothOutput(ACC, SensorState.RawACC);
		//imuAccIIRLPFilter(raw, accelLPF);
		MultiwiiAccFilter(raw, accelLPF);
		SensorState.RawACC[0] = accelLPF[0];
		SensorState.RawACC[1] = accelLPF[1];
		SensorState.RawACC[2] = accelLPF[2];
	}	
	else {
	SensorState.RawACC[0] = raw[0];
	SensorState.RawACC[1] = raw[1];
	SensorState.RawACC[2] = raw[2];
	}
	
	SetCalibrateACC();
}
void nvtInputSensorRawGYRO(int16_t *raw)
{
	GUARD
	RawSmoothInput(GYRO, raw);
	if(SensorState.RawSmooth.enable[GYRO])
		RawSmoothOutput(GYRO, SensorState.RawGYRO);
	else {
	SensorState.RawGYRO[0] = raw[0];
	SensorState.RawGYRO[1] = raw[1];
	SensorState.RawGYRO[2] = raw[2];
	}
	SetCalibrateGYRO();
}
void nvtInputSensorRawMAG(int16_t *raw)
{
	GUARD
	RawSmoothInput(MAG, raw);
	if(SensorState.RawSmooth.enable[MAG])
		RawSmoothOutput(MAG, SensorState.RawMAG);
	else {
		SensorState.RawMAG[0] = raw[0];
		SensorState.RawMAG[1] = raw[1];
		SensorState.RawMAG[2] = raw[2];
	}
	SetCalibrateMAG();
}
void nvtInputSensorRawBARO(int16_t *raw)
{
	int16_t RawBuffer[2];
	GUARD
	RawBuffer[0] = raw[0];//kalmanFilter1(raw[0]);
	RawBuffer[1] = raw[1];//kalmanFilter2(raw[1]);
	//BaroFilter(RawBuffer);
	RawSmoothInput(BARO, RawBuffer);
	if(SensorState.RawSmooth.enable[BARO]) {
		RawSmoothOutput(BARO, SensorState.RawBARO);	
		//SensorState.RawBARO[0] = RawBuffer[0];
		//SensorState.RawBARO[1] = RawBuffer[1];
	}
	else {
		SensorState.RawBARO[0] = raw[0];
		SensorState.RawBARO[1] = raw[1];
	}
}
void nvtInputSensorRaw9D(int16_t *RawACC, int16_t *RawGYRO, int16_t *RawMAG)
{
	GUARD
	if(RawACC)
		nvtInputSensorRawACC(RawACC);
	
	if(RawGYRO)
		nvtInputSensorRawGYRO(RawGYRO);
	
	if(RawMAG)
		nvtInputSensorRawMAG(RawMAG);
}
void nvtGetAccZWithoutGravity(float *ZWithoutGravity, float *AccZMag)
{
	if(ZWithoutGravity)
		*ZWithoutGravity = sensfusion6GetAccZWithoutGravity(AttitudeInfo.CalACC[0], AttitudeInfo.CalACC[1], AttitudeInfo.CalACC[2]) - ZWithoutG_Base;
	
	if(AccZMag)
	*AccZMag = AttitudeInfo.CalACC[0]*AttitudeInfo.CalACC[0]+
							AttitudeInfo.CalACC[1]*AttitudeInfo.CalACC[1]+
							AttitudeInfo.CalACC[2]*AttitudeInfo.CalACC[2];
}
void nvtGetSensorRawACC(int16_t *raw)
{
	GUARD
	raw[0] = SensorState.RawACC[0];
	raw[1] = SensorState.RawACC[1];
	raw[2] = SensorState.RawACC[2];
}
void nvtGetSensorRawGYRO(int16_t *raw)
{
	GUARD
	raw[0] = SensorState.RawGYRO[0];
	raw[1] = SensorState.RawGYRO[1];
	raw[2] = SensorState.RawGYRO[2];
}
void nvtGetSensorRawMAG(int16_t *raw)
{
	GUARD
	raw[0] = SensorState.RawMAG[0];
	raw[1] = SensorState.RawMAG[1];
	raw[2] = SensorState.RawMAG[2];
}
void nvtGetSensorRawBARO(uint16_t *raw)
{
	int16_t temp;
	GUARD
	temp = SensorState.RawBARO[0];
	raw[0] = (uint16_t)temp;
	temp = SensorState.RawBARO[1];
	raw[1] = (uint16_t)temp;
}
void nvtSmoothSensorRawData(bool enable,char sensor)
{
	SensorState.RawSmooth.enable[sensor] = enable;
}
void nvtGetSensorRaw9D(int16_t *RawACC, int16_t *RawGYRO, int16_t *RawMAG)
{
	GUARD
	nvtGetSensorRawACC(RawACC);
	nvtGetSensorRawGYRO(RawGYRO);
	nvtGetSensorRawMAG(RawMAG);
}
/* Get Calibration Information */
void nvtGetAccOffset(float* offset)
{
	offset[0] = CalInfo.AccMean[0];
	offset[1] = CalInfo.AccMean[1];
	offset[2] = CalInfo.AccMean[2];
}
void nvtGetAccScale(float* scale)
{
	scale[0] = CalInfo.AccScale[0];
	scale[1] = CalInfo.AccScale[1];
	scale[2] = CalInfo.AccScale[2];
}
void nvtGetGyroOffset(float* offset)
{
	offset[0] = CalInfo.GyroMean[0];
	offset[1] = CalInfo.GyroMean[1];
	offset[2] = CalInfo.GyroMean[2];
}
void nvtGetGyroScale(float* scale)
{
	scale[0] = CalInfo.GyroScale[0];
	scale[1] = CalInfo.GyroScale[1];
	scale[2] = CalInfo.GyroScale[2];
}
void nvtGetMagCalMatrix(float* calMatrix)
{
	int i;
	for(i=0; i<MAG_CAL_DATA_SIZE; i++)
		calMatrix[i] = CalInfo.MagInvW[i];
}
void nvtSetSensorEnable(char SensorID, char enable)
{
	SensorState.beSensorEnable[SensorID]  = enable;
}
/* Set Calibration Information */
void nvtSetAccOffset(float* AccMean)
{
	CalInfo.AccMean[AXIS_X] = AccMean[AXIS_X];
	CalInfo.AccMean[AXIS_Y] = AccMean[AXIS_Y];
	CalInfo.AccMean[AXIS_Z] = AccMean[AXIS_Z];
}
void nvtSetAccScale(float* AccScale)
{
	CalInfo.AccScale[AXIS_X] = AccScale[AXIS_X];
	CalInfo.AccScale[AXIS_Y] = AccScale[AXIS_Y];
	CalInfo.AccScale[AXIS_Z] = AccScale[AXIS_Z];
}
void nvtSetGyroOffset(float* GyroMean)
{
	CalInfo.GyroMean[AXIS_X] = GyroMean[AXIS_X];
	CalInfo.GyroMean[AXIS_Y] = GyroMean[AXIS_Y];
	CalInfo.GyroMean[AXIS_Z] = GyroMean[AXIS_Z];
}
void nvtSetGyroScale(float* GyroScale)
{
	CalInfo.GyroScale[AXIS_X] = GyroScale[AXIS_X];
	CalInfo.GyroScale[AXIS_Y] = GyroScale[AXIS_Y];
	CalInfo.GyroScale[AXIS_Z] = GyroScale[AXIS_Z];
}
void nvtSetMagCalMatrix(float* MagCalMatrix)
{
	int i;
	for(i=0;i<MAG_CAL_DATA_SIZE;i++)
		CalInfo.MagInvW[i] = MagCalMatrix[i];
	UpdateMagMasterTime();
	SetMagGuass();
}
