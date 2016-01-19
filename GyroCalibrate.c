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
#include <math.h>
#include <stdint.h>
#include "Common.h"
#include "AHRSLib.h"
#include "GyroDriftCalibrate.h"
static int8_t GyroTestStatus[MAX_AHRS][3];
static float GyroDeg[MAX_AHRS][3];
uint8_t	AxisDoneFlag[MAX_AHRS] = {0};
void nvtCalGyroInit(char axis)
{
	GyroDeg[AHRSID][axis] = 0;GyroTestStatus[AHRSID][axis] = STATUS_GYRO_CAL_BEGINE;
	SensorState[AHRSID].beGyroSteadyPrev = true;
	AxisDoneFlag[AHRSID]&=~(1<<axis);
	while(nvtGyroCenterCalibrate()==STATUS_GYRO_CAL_RUNNING);
}
int8_t nvtGyroIsSteady()
{
	GyroDynamicCalibrate(SensorState[AHRSID].RawGYRO[0], SensorState[AHRSID].RawGYRO[1], SensorState[AHRSID].RawGYRO[2]);
	
	if(!SensorState[AHRSID].beGyroSteady) {
		//DBG_PRINTF("GYRO not steady\n");
		return STATUS_GYRO_NOT_STEADY;
	}
	else {
		//DBG_PRINTF("GYRO steady\n");
		return STATUS_GYRO_STEADY;
	}
}
int8_t nvtGyroCenterCalibrate()
{
	float AccZWithoutG_Base;
	SensorState[AHRSID].beGyroSteady = false;
	GyroDynamicCalibrate(SensorState[AHRSID].RawGYRO[0], SensorState[AHRSID].RawGYRO[1], SensorState[AHRSID].RawGYRO[2]);
	
	if(!SensorState[AHRSID].beGyroSteady) {
		return STATUS_GYRO_CAL_RUNNING;
	}
	else {
		GetGyroDynamicCenter(&CalInfo[AHRSID].GyroMean[0], &CalInfo[AHRSID].GyroMean[1], &CalInfo[AHRSID].GyroMean[2]);
		nvtGetAccZWithoutGravity(&AccZWithoutG_Base, null);
		SetZWithoutG_Base(AccZWithoutG_Base);
		SetMagGuass();
		return STATUS_GYRO_CAL_DONE;
	}
}

int8_t GyroScaleCalibrate(int8_t axis, float intervalSec)
{
	float GyroCorrected;
	GyroDynamicCalibrate(SensorState[AHRSID].RawGYRO[0], SensorState[AHRSID].RawGYRO[1], SensorState[AHRSID].RawGYRO[2]);

	GyroCorrected = (float)SensorState[AHRSID].RawGYRO[axis]*CalInfo[AHRSID].GyroDegPLSB - CalInfo[AHRSID].GyroMean[axis]*CalInfo[AHRSID].GyroDegPLSB;
	GyroDeg[AHRSID][axis]+=GyroCorrected*intervalSec;
	
	if((SensorState[AHRSID].beGyroSteadyPrev==false)&&(SensorState[AHRSID].beGyroSteady==true)) {
		GyroTestStatus[AHRSID][axis]=true;
	}
	else if((SensorState[AHRSID].beGyroSteadyPrev==true)&&(SensorState[AHRSID].beGyroSteady==true)) {
		GyroTestStatus[AHRSID][axis]=false;
	}
	else if((SensorState[AHRSID].beGyroSteadyPrev==false)&&(SensorState[AHRSID].beGyroSteady==false)) {
		GyroTestStatus[AHRSID][axis]=false;
	}
	else if((SensorState[AHRSID].beGyroSteadyPrev==true)&&(SensorState[AHRSID].beGyroSteady==false)) {
		GyroTestStatus[AHRSID][axis]=false;
	}
	SensorState[AHRSID].beGyroSteadyPrev = SensorState[AHRSID].beGyroSteady;
	
	if(GyroTestStatus[AHRSID][axis]) {	
		if(GyroDeg[AHRSID][axis]<0) GyroDeg[AHRSID][axis]=-GyroDeg[AHRSID][axis];
		CalInfo[AHRSID].GyroScale[axis]=(float)(360.0f*3/GyroDeg[AHRSID][axis]);
		printf("%f:",GyroDeg[AHRSID][axis]);
		
		AxisDoneFlag[AHRSID]|=(1<<axis);
		if(AxisDoneFlag[AHRSID]==0x7) {
			SensorState[AHRSID].beCalibrated[GYRO]  = true;
		return STATUS_GYRO_CAL_DONE;
		}
		else
			return STATUS_GYRO_AXIS_CAL_DONE;		
	}
	else
		return STATUS_GYRO_CAL_RUNNING;
}
int8_t nvtGyroScaleCalibrate(int8_t axis)
{
	if(GyroTestStatus[AHRSID][axis]==STATUS_GYRO_CAL_BEGINE) {
		TimerStart();
		GyroTestStatus[AHRSID][axis]=STATUS_GYRO_CAL_RUNNING;
	}
	else if(GyroTestStatus[AHRSID][axis]==STATUS_GYRO_CAL_RUNNING) {
		GyroTestStatus[AHRSID][axis]=GyroScaleCalibrate(axis, TimerRead());
		//DBG_PRINTF("msec:%d\n",TimerRead());
	}
	
	if(GyroTestStatus[AHRSID][axis]==STATUS_GYRO_AXIS_CAL_DONE)
		DBG_PRINT("%f\n",	GyroDeg[AHRSID][axis]);		
	
	TimerSet();
	return GyroTestStatus[AHRSID][axis];
}
void SetCalibrateGYRO()
{
	AttitudeInfo[AHRSID].CalGYRO[0] = (SensorState[AHRSID].RawGYRO[0] -  CalInfo[AHRSID].GyroMean[0]) * CalInfo[AHRSID].GyroScale[0];
	AttitudeInfo[AHRSID].CalGYRO[1] = (SensorState[AHRSID].RawGYRO[1] -  CalInfo[AHRSID].GyroMean[1]) * CalInfo[AHRSID].GyroScale[1];
	AttitudeInfo[AHRSID].CalGYRO[2] = (SensorState[AHRSID].RawGYRO[2] -  CalInfo[AHRSID].GyroMean[2]) * CalInfo[AHRSID].GyroScale[2];
}
void nvtGetCalibratedGYRO(float* CGYRO)
{
	CGYRO[0] = AttitudeInfo[AHRSID].CalGYRO[0];
	CGYRO[1] = AttitudeInfo[AHRSID].CalGYRO[1];
	CGYRO[2] = AttitudeInfo[AHRSID].CalGYRO[2];
}
void nvtSetGYRODegPLSB(float DPLSB)
{
	CalInfo[AHRSID].GyroDegPLSB = DPLSB;
}

