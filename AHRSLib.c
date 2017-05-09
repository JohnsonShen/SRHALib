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
CalibrateInfo CalInfo[MAX_AHRS];
SensorStateInfo SensorState[MAX_AHRS];
TimeFrameInfo TimeInfo;
AttitudeInfo_T AttitudeInfo[MAX_AHRS];
uint32_t msTick;
char DIM[MAX_SENSOR] = {ACC_DIM, GYRO_DIM, MAG_DIM, BARO_DIM, HALL_DIM};
float period;
float ZWithoutG_Base[MAX_AHRS];
static int16_t accelLPF[MAX_AHRS][3];
int16_t actuator[2];
MotorCal_t MotorCalL;
char AHRSID = AHRDID_0;
char Performance = 0;
void nvtAccurtcyTradePerformance(bool Trade)
{
	if(Trade)
    Performance = 1;
  else
    Performance = 0;
}
bool nvtGetAccurtcyTradePerformance()
{
  return Performance;
}
void SetZWithoutG_Base(float base)
{
	ZWithoutG_Base[AHRSID] = base;
}
float GetAHRSPeriod()
{
	return period;
}
void TimerSet()
{
	TimeInfo.currentTime[AHRSID] = TimeInfo.tick_counter;
  TimeInfo.lastTime[AHRSID] = TimeInfo.currentTime[AHRSID];
}
void TimerStart()
{
	TimeInfo.currentTime[AHRSID] = TimeInfo.lastTime[AHRSID] = TimeInfo.tick_counter;
}
float TimerRead()
{
	return (TimeInfo.tick_counter-TimeInfo.currentTime[AHRSID])/10000.0f;
}


void SensorStateInit() 
{
  uint8_t i;
  for(i = 0;i< MAX_AHRS;i++) {
    SensorState[i].beCalibrated[ACC]  = false;
    SensorState[i].beCalibrated[GYRO] = false;
    SensorState[i].beCalibrated[MAG]  = false;
    
    SensorState[i].beSensorEnable[ACC]  = false;
    SensorState[i].beSensorEnable[GYRO] = false;
    SensorState[i].beSensorEnable[MAG]  = false;
    
    SensorState[i].beGyroSteady       = false;
    SensorState[i].beGyroSteadyPrev   = false;
    SensorState[i].GyroDynamicCenter[AXIS_X] = 0;
    SensorState[i].GyroDynamicCenter[AXIS_Y] = 0;
    SensorState[i].GyroDynamicCenter[AXIS_Z] = 0;
    SensorState[i].RawSmooth.Index[ACC] = 0;
    SensorState[i].RawSmooth.enable[ACC] = false;
    SensorState[i].RawSmooth.Index[GYRO] = 0;
    SensorState[i].RawSmooth.enable[GYRO] = false;
    SensorState[i].RawSmooth.Index[MAG] = 0;
    SensorState[i].RawSmooth.enable[MAG] = true;
    SensorState[i].RawSmooth.Index[BARO] = 0;
    SensorState[i].RawSmooth.enable[BARO] = true;
    SensorState[i].RawSmooth.Index[HALL] = 0;
    SensorState[i].RawSmooth.enable[HALL] = false;
  }
}
void CalInfoInit()
{
  uint8_t i;
  for(i = 0;i< MAX_AHRS;i++) {
    CalInfo[i].AccMean[0] = 0;
    CalInfo[i].AccMean[1] = 0;
    CalInfo[i].AccMean[2] = 0;
    CalInfo[i].AccScale[0] = 1;
    CalInfo[i].AccScale[1] = 1;
    CalInfo[i].AccScale[2] = 1;
    
    CalInfo[i].GyroMean[0] = 0;
    CalInfo[i].GyroMean[1] = 0;
    CalInfo[i].GyroMean[2] = 0;
    CalInfo[i].GyroScale[0] = 1;
    CalInfo[i].GyroScale[1] = 1;
    CalInfo[i].GyroScale[2] = 1;
    
    CalInfo[i].MagInvW[0] = 0;
    CalInfo[i].MagInvW[1] = 0;
    CalInfo[i].MagInvW[2] = 0;
    CalInfo[i].MagInvW[3] = 0.0002;
    CalInfo[i].MagInvW[4] = 0.0002;
    CalInfo[i].MagInvW[5] = 0.0002;
    CalInfo[i].MagInvW[6] = 0.000f;
    CalInfo[i].MagInvW[7] = 0.000f;
    CalInfo[i].MagInvW[8] = 0.000f;
    CalInfo[i].MagInvW[9] = 1.0;
  }
}
void TimeInfoInit()
{
  int i;
  for(i=0;i<MAX_AHRS;i++) {
    TimeInfo.lastTime[i] = 0;
    TimeInfo.interval[i] = 0;
    TimeInfo.currentTime[i] = 0;
   }
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
	sensfusion6GetEulerRPY(&AttitudeInfo[AHRSID].Euler[ROLL], &AttitudeInfo[AHRSID].Euler[PITCH], &AttitudeInfo[AHRSID].Euler[YAW]);
	rpy[0] = AttitudeInfo[AHRSID].Euler[ROLL];
	rpy[1] = AttitudeInfo[AHRSID].Euler[PITCH];
	rpy[2] = AttitudeInfo[AHRSID].Euler[YAW];
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
	sensfusion6GetGyro(&AttitudeInfo[AHRSID].NormAttitude[0]);
	Att[0] = AttitudeInfo[AHRSID].NormAttitude[0];
	Att[1] = AttitudeInfo[AHRSID].NormAttitude[1];
	Att[2] = AttitudeInfo[AHRSID].NormAttitude[2];
}
void nvtGetNormACC(float* NormACC)
{
	sensfusion6GetAcc(&AttitudeInfo[AHRSID].NormACC[0]);
	NormACC[0] = AttitudeInfo[AHRSID].NormACC[0];
	NormACC[1] = AttitudeInfo[AHRSID].NormACC[1];
	NormACC[2] = AttitudeInfo[AHRSID].NormACC[2];
}
void nvtGetMAGHeading(float* Heading)
{
	sensfusion6GetMagDegree(&AttitudeInfo[AHRSID].MAGHead[0]);
	Heading[0] = AttitudeInfo[AHRSID].MAGHead[0];
	Heading[1] = AttitudeInfo[AHRSID].MAGHead[1];
	Heading[2] = AttitudeInfo[AHRSID].MAGHead[2];
}
void nvtGetEulerNormMAG(float* NormMAG)
{
	sensfusion6GetMagEuler(&AttitudeInfo[AHRSID].EulerNormMAG[0]);
	NormMAG[0] = AttitudeInfo[AHRSID].EulerNormMAG[0];
	NormMAG[1] = AttitudeInfo[AHRSID].EulerNormMAG[1];
	NormMAG[2] = AttitudeInfo[AHRSID].EulerNormMAG[2];
}
void nvtGetNormMAG(float* NormMAG)
{
	sensfusion6GetNormMag(&AttitudeInfo[AHRSID].NormMAG[0]);
	NormMAG[0] = AttitudeInfo[AHRSID].NormMAG[0];
	NormMAG[1] = AttitudeInfo[AHRSID].NormMAG[1];
	NormMAG[2] = AttitudeInfo[AHRSID].NormMAG[2];
}
void nvtGetGYRODegree(float* Deg)
{
	sensfusion6GetGyroDeg(&AttitudeInfo[AHRSID].GYRODeg[0]);
	Deg[0] = AttitudeInfo[AHRSID].GYRODeg[0];
	Deg[1] = AttitudeInfo[AHRSID].GYRODeg[1];
	Deg[2] = AttitudeInfo[AHRSID].GYRODeg[2];
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
		CACC[0] = AttitudeInfo[AHRSID].CalACC[0];
		CACC[1] = AttitudeInfo[AHRSID].CalACC[1];
		CACC[2] = AttitudeInfo[AHRSID].CalACC[2];
	}
	if(UPDATE&SENSOR_GYRO){
		CGYRO[0] = AttitudeInfo[AHRSID].CalGYRO[0];
		CGYRO[1] = AttitudeInfo[AHRSID].CalGYRO[1];
		CGYRO[2] = AttitudeInfo[AHRSID].CalGYRO[2];
	}
	if(UPDATE&SENSOR_MAG){
		CMAG[0] = AttitudeInfo[AHRSID].CalMAG[0];
		CMAG[1] = AttitudeInfo[AHRSID].CalMAG[1];
		CMAG[2] = AttitudeInfo[AHRSID].CalMAG[2];
	}
        if(UPDATE&SENSOR_HALL){
	}
	sensfusion9UpdateQ(CGYRO[0], CGYRO[1], CGYRO[2], 
										 CACC[0], CACC[1], CACC[2], 
										 CMAG[0], CMAG[1], CMAG[2], period);

	period = TimerRead();
	TimerSet();
	//sensfusion6UpdateMagByEuler();
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
		SensorState[AHRSID].RawSmooth.RingBuffer[RingOffset+i][SensorState[AHRSID].RawSmooth.Index[Sensor]] = raw[i];
	}

	SensorState[AHRSID].RawSmooth.Index[Sensor]++;
	SensorState[AHRSID].RawSmooth.Index[Sensor] = SensorState[AHRSID].RawSmooth.Index[Sensor]%RAW_SMOOTH_BUFFER_SIZE;
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
			average+=SensorState[AHRSID].RawSmooth.RingBuffer[RingOffset+i][j];
		raw[i] = average/RAW_SMOOTH_BUFFER_SIZE;
		average = 0;
	}
	//printf("out: %d   %d   %d\n", (int16_t)raw[0], (int16_t)raw[1], (int16_t)raw[2]);
}

void nvtInputSensorRawACC(int16_t *raw)
{
	GUARD
	//RawSmoothInput(ACC, raw);
	if(SensorState[AHRSID].RawSmooth.enable[ACC]) {
		//RawSmoothOutput(ACC, SensorState.RawACC);
		//imuAccIIRLPFilter(raw, accelLPF);
		MultiwiiAccFilter(raw, accelLPF[AHRSID]);
		SensorState[AHRSID].RawACC[0] = accelLPF[AHRSID][0];
		SensorState[AHRSID].RawACC[1] = accelLPF[AHRSID][1];
		SensorState[AHRSID].RawACC[2] = accelLPF[AHRSID][2];
	}	
	else {
	SensorState[AHRSID].RawACC[0] = raw[0];
	SensorState[AHRSID].RawACC[1] = raw[1];
	SensorState[AHRSID].RawACC[2] = raw[2];
	}
	
	SetCalibrateACC();
}
void nvtInputSensorRawGYRO(int16_t *raw)
{
	GUARD
	RawSmoothInput(GYRO, raw);
	if(SensorState[AHRSID].RawSmooth.enable[GYRO])
		RawSmoothOutput(GYRO, SensorState[AHRSID].RawGYRO);
	else {
	SensorState[AHRSID].RawGYRO[0] = raw[0];
	SensorState[AHRSID].RawGYRO[1] = raw[1];
	SensorState[AHRSID].RawGYRO[2] = raw[2];
	}
	SetCalibrateGYRO();
}
void nvtInputSensorRawMAG(int16_t *raw)
{
	GUARD
	RawSmoothInput(MAG, raw);
	if(SensorState[AHRSID].RawSmooth.enable[MAG])
		RawSmoothOutput(MAG, SensorState[AHRSID].RawMAG);
	else {
		SensorState[AHRSID].RawMAG[0] = raw[0];
		SensorState[AHRSID].RawMAG[1] = raw[1];
		SensorState[AHRSID].RawMAG[2] = raw[2];
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
	if(SensorState[AHRSID].RawSmooth.enable[BARO]) {
		RawSmoothOutput(BARO, SensorState[AHRSID].RawBARO);	
		//SensorState.RawBARO[0] = RawBuffer[0];
		//SensorState.RawBARO[1] = RawBuffer[1];
	}
	else {
		SensorState[AHRSID].RawBARO[0] = raw[0];
		SensorState[AHRSID].RawBARO[1] = raw[1];
	}
}
void nvtInputSensorRawHALL(int16_t *raw)
{
	GUARD
	RawSmoothInput(HALL, raw);
	if(SensorState[0].RawSmooth.enable[HALL])
		RawSmoothOutput(HALL, SensorState[0].RawHALL);
	else {
		SensorState[0].RawHALL[0] = raw[0];
		SensorState[0].RawHALL[1] = raw[1];
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
		*ZWithoutGravity = sensfusion6GetAccZWithoutGravity(AttitudeInfo[AHRSID].CalACC[0], AttitudeInfo[AHRSID].CalACC[1], AttitudeInfo[AHRSID].CalACC[2]) - ZWithoutG_Base[AHRSID];
	
	if(AccZMag)
	*AccZMag = AttitudeInfo[AHRSID].CalACC[0]*AttitudeInfo[AHRSID].CalACC[0]+
							AttitudeInfo[AHRSID].CalACC[1]*AttitudeInfo[AHRSID].CalACC[1]+
							AttitudeInfo[AHRSID].CalACC[2]*AttitudeInfo[AHRSID].CalACC[2];
}
void nvtGetSensorRawACC(int16_t *raw)
{
	GUARD
	raw[0] = SensorState[AHRSID].RawACC[0];
	raw[1] = SensorState[AHRSID].RawACC[1];
	raw[2] = SensorState[AHRSID].RawACC[2];
}
void nvtGetSensorRawGYRO(int16_t *raw)
{
	GUARD
	raw[0] = SensorState[AHRSID].RawGYRO[0];
	raw[1] = SensorState[AHRSID].RawGYRO[1];
	raw[2] = SensorState[AHRSID].RawGYRO[2];
}
void nvtGetSensorRawMAG(int16_t *raw)
{
	GUARD
	raw[0] = SensorState[AHRSID].RawMAG[0];
	raw[1] = SensorState[AHRSID].RawMAG[1];
	raw[2] = SensorState[AHRSID].RawMAG[2];
}
void nvtGetSensorRawBARO(uint16_t *raw)
{
	int16_t temp;
	GUARD
	temp = SensorState[AHRSID].RawBARO[0];
	raw[0] = (uint16_t)temp;
	temp = SensorState[AHRSID].RawBARO[1];
	raw[1] = (uint16_t)temp;
}
void nvtGetSensorRawHALL(int16_t *raw)
{
	int16_t temp;
	GUARD
	temp = SensorState[0].RawHALL[0];
	raw[0] = (uint16_t)temp;
	temp = SensorState[0].RawHALL[1];
	raw[1] = (uint16_t)temp;
}
void nvtGetFusionSpeed(float* speed)
{
	speed[0] = SensorState[0].RawHALL[0];
	speed[1] = SensorState[0].RawHALL[1];
	speed[2] = (SensorState[0].RawHALL[0]+SensorState[0].RawHALL[1])/2;
}
void nvtSmoothSensorRawData(bool enable,char sensor)
{
	SensorState[AHRSID].RawSmooth.enable[sensor] = enable;
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
	offset[0] = CalInfo[AHRSID].AccMean[0];
	offset[1] = CalInfo[AHRSID].AccMean[1];
	offset[2] = CalInfo[AHRSID].AccMean[2];
}
void nvtGetAccScale(float* scale)
{
	scale[0] = CalInfo[AHRSID].AccScale[0];
	scale[1] = CalInfo[AHRSID].AccScale[1];
	scale[2] = CalInfo[AHRSID].AccScale[2];
}
void nvtGetGyroOffset(float* offset)
{
	offset[0] = CalInfo[AHRSID].GyroMean[0];
	offset[1] = CalInfo[AHRSID].GyroMean[1];
	offset[2] = CalInfo[AHRSID].GyroMean[2];
}
void nvtGetGyroScale(float* scale)
{
	scale[0] = CalInfo[AHRSID].GyroScale[0];
	scale[1] = CalInfo[AHRSID].GyroScale[1];
	scale[2] = CalInfo[AHRSID].GyroScale[2];
}
void nvtGetMagCalMatrix(float* calMatrix)
{
	int i;
	for(i=0; i<MAG_CAL_DATA_SIZE; i++)
		calMatrix[i] = CalInfo[AHRSID].MagInvW[i];
}
void nvtSetSensorEnable(char SensorID, char enable)
{
	SensorState[AHRSID].beSensorEnable[SensorID]  = enable;
}
/* Set Calibration Information */
void nvtSetAccOffset(float* AccMean)
{
	CalInfo[AHRSID].AccMean[AXIS_X] = AccMean[AXIS_X];
	CalInfo[AHRSID].AccMean[AXIS_Y] = AccMean[AXIS_Y];
	CalInfo[AHRSID].AccMean[AXIS_Z] = AccMean[AXIS_Z];
}
void nvtSetAccScale(float* AccScale)
{
	CalInfo[AHRSID].AccScale[AXIS_X] = AccScale[AXIS_X];
	CalInfo[AHRSID].AccScale[AXIS_Y] = AccScale[AXIS_Y];
	CalInfo[AHRSID].AccScale[AXIS_Z] = AccScale[AXIS_Z];
}
void nvtSetGyroOffset(float* GyroMean)
{
	CalInfo[AHRSID].GyroMean[AXIS_X] = GyroMean[AXIS_X];
	CalInfo[AHRSID].GyroMean[AXIS_Y] = GyroMean[AXIS_Y];
	CalInfo[AHRSID].GyroMean[AXIS_Z] = GyroMean[AXIS_Z];
}
void nvtSetGyroScale(float* GyroScale)
{
	CalInfo[AHRSID].GyroScale[AXIS_X] = GyroScale[AXIS_X];
	CalInfo[AHRSID].GyroScale[AXIS_Y] = GyroScale[AXIS_Y];
	CalInfo[AHRSID].GyroScale[AXIS_Z] = GyroScale[AXIS_Z];
}
void nvtSetMagCalMatrix(float* MagCalMatrix)
{
	int i;
	for(i=0;i<MAG_CAL_DATA_SIZE;i++)
		CalInfo[AHRSID].MagInvW[i] = MagCalMatrix[i];
	UpdateMagMasterTime();
	SetMagGuass();
}
void nvtActuatorFusionFilter(ACTUATOR_T* pActuator)
{
	actuator[RIGHT] = (/*pActuator->actuatorThrust*5*/  +pActuator->actuatorSpeed - pActuator->actuatorPitch + pActuator->actuatorRoll - pActuator->actuatorYaw)*1.0417f;
  actuator[LEFT] = (/*-pActuator->actuatorThrust*5*/ -pActuator->actuatorSpeed + pActuator->actuatorPitch + pActuator->actuatorRoll - pActuator->actuatorYaw)*1.0417f;
}
void nvtGetActuatorSmooth(int16_t* actuatorSmooth)
{
	actuatorSmooth[RIGHT] = (actuator[RIGHT] + MotorCalL.MotorOffset[RIGHT])*MotorCalL.MotorScale[RIGHT];
  actuatorSmooth[LEFT] = (actuator[LEFT] + MotorCalL.MotorOffset[LEFT])*MotorCalL.MotorScale[LEFT];
}
void nvtSetMotorSmooth(MotorCal_t* MotorCal)
{
	MotorCalL.MotorOffset[RIGHT] = MotorCal->MotorOffset[RIGHT];
	MotorCalL.MotorOffset[LEFT] = MotorCal->MotorOffset[LEFT];
	MotorCalL.MotorScale[RIGHT] = MotorCal->MotorScale[RIGHT];
	MotorCalL.MotorScale[LEFT] = MotorCal->MotorScale[LEFT];
}
void nvtSetAHRSID(char id)
{
	AHRSID = id;
}
char nvtGetAHRSID()
{
	return AHRSID;
}
