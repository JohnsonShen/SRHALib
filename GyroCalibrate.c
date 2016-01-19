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
static int8_t GyroTestStatus[3];
static float GyroDeg[3];
uint8_t	AxisDoneFlag = 0;
void nvtCalGyroInit(char axis)
{
	GyroDeg[axis] = 0;GyroTestStatus[axis] = STATUS_GYRO_CAL_BEGINE;
	SensorState.beGyroSteadyPrev = true;
	AxisDoneFlag&=~(1<<axis);
	while(nvtGyroCenterCalibrate()==STATUS_GYRO_CAL_RUNNING);
}
int8_t nvtGyroIsSteady()
{
	GyroDynamicCalibrate(SensorState.RawGYRO[0], SensorState.RawGYRO[1], SensorState.RawGYRO[2]);
	
	if(!SensorState.beGyroSteady) {
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
	SensorState.beGyroSteady = false;
	GyroDynamicCalibrate(SensorState.RawGYRO[0], SensorState.RawGYRO[1], SensorState.RawGYRO[2]);
	
	if(!SensorState.beGyroSteady) {
		return STATUS_GYRO_CAL_RUNNING;
	}
	else {
		GetGyroDynamicCenter(&CalInfo.GyroMean[0], &CalInfo.GyroMean[1], &CalInfo.GyroMean[2]);
		nvtGetAccZWithoutGravity(&AccZWithoutG_Base, null);
		SetZWithoutG_Base(AccZWithoutG_Base);
		SetMagGuass();
		return STATUS_GYRO_CAL_DONE;
	}
}

int8_t GyroScaleCalibrate(int8_t axis, float intervalSec)
{
	float GyroCorrected;
	GyroDynamicCalibrate(SensorState.RawGYRO[0], SensorState.RawGYRO[1], SensorState.RawGYRO[2]);

	GyroCorrected = (float)SensorState.RawGYRO[axis]*CalInfo.GyroDegPLSB - CalInfo.GyroMean[axis]*CalInfo.GyroDegPLSB;
	GyroDeg[axis]+=GyroCorrected*intervalSec;
	
	if((SensorState.beGyroSteadyPrev==false)&&(SensorState.beGyroSteady==true)) {
		GyroTestStatus[axis]=true;
	}
	else if((SensorState.beGyroSteadyPrev==true)&&(SensorState.beGyroSteady==true)) {
		GyroTestStatus[axis]=false;
	}
	else if((SensorState.beGyroSteadyPrev==false)&&(SensorState.beGyroSteady==false)) {
		GyroTestStatus[axis]=false;
	}
	else if((SensorState.beGyroSteadyPrev==true)&&(SensorState.beGyroSteady==false)) {
		GyroTestStatus[axis]=false;
	}
	SensorState.beGyroSteadyPrev = SensorState.beGyroSteady;
	
	if(GyroTestStatus[axis]) {	
		if(GyroDeg[axis]<0) GyroDeg[axis]=-GyroDeg[axis];
		CalInfo.GyroScale[axis]=(float)(360.0f*3/GyroDeg[axis]);
		printf("%f:",GyroDeg[axis]);
		
		AxisDoneFlag|=(1<<axis);
		if(AxisDoneFlag==0x7) {
			SensorState.beCalibrated[GYRO]  = true;
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
	if(GyroTestStatus[axis]==STATUS_GYRO_CAL_BEGINE) {
		TimerStart();
		GyroTestStatus[axis]=STATUS_GYRO_CAL_RUNNING;
	}
	else if(GyroTestStatus[axis]==STATUS_GYRO_CAL_RUNNING) {
		GyroTestStatus[axis]=GyroScaleCalibrate(axis, TimerRead());
		//DBG_PRINTF("msec:%d\n",TimerRead());
	}
	
	if(GyroTestStatus[axis]==STATUS_GYRO_AXIS_CAL_DONE)
		DBG_PRINT("%f\n",	GyroDeg[axis]);		
	
	TimerSet();
	return GyroTestStatus[axis];
}
void SetCalibrateGYRO()
{
	AttitudeInfo.CalGYRO[0] = (SensorState.RawGYRO[0] -  CalInfo.GyroMean[0]) * CalInfo.GyroScale[0];
	AttitudeInfo.CalGYRO[1] = (SensorState.RawGYRO[1] -  CalInfo.GyroMean[1]) * CalInfo.GyroScale[1];
	AttitudeInfo.CalGYRO[2] = (SensorState.RawGYRO[2] -  CalInfo.GyroMean[2]) * CalInfo.GyroScale[2];
}
void nvtGetCalibratedGYRO(float* CGYRO)
{
	CGYRO[0] = AttitudeInfo.CalGYRO[0];
	CGYRO[1] = AttitudeInfo.CalGYRO[1];
	CGYRO[2] = AttitudeInfo.CalGYRO[2];
}
void nvtSetGYRODegPLSB(float DPLSB)
{
	CalInfo.GyroDegPLSB = DPLSB;
}

