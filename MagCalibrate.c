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
#include "AHRSLib.h"
#include "Common.h"
int16_t BufferRawMAG[MAX_AHRS][SAMPLE_SIZE_MAG*3];
int16_t BufferIndexMag[MAX_AHRS];
float MagGuass[MAX_AHRS];
void nvtCalMAGInit()
{
  int i;
  for(i=0;i<MAX_AHRS;i++) {
	BufferIndexMag[i] = 0;
	SensorState[i].beCalibrated[MAG] = false;
	CalInfo[i].MagCalQuality = 0;
	CalInfo[i].RestoreRawSmooth[MAG] = SensorState[AHRSID].RawSmooth.enable[MAG];
	SensorState[i].RawSmooth.enable[MAG] = false;
  }
}
void UpdateCalibrateInfoMAG()
{
	float *BetaMAG;
	int i;
	BetaMAG = GetCalibrateParams(SENSOR_MAG);
	for(i=0;i<MAG_BETA_SIZE;i++)
		CalInfo[AHRSID].MagInvW[i] = BetaMAG[i];

	SensorState[AHRSID].beCalibrated[MAG]  = true;
}
void SetCalibrateMAG()
{
	int i;
	float X_B0, Y_B1, Z_B2, B[MAG_BETA_SIZE];

	if((SensorState[AHRSID].RawMAG[0]!=0)||(SensorState[AHRSID].RawMAG[1]!=0)||(SensorState[AHRSID].RawMAG[2]!=0)) {	
		for(i=0;i<MAG_BETA_SIZE;i++)
			B[i]=CalInfo[AHRSID].MagInvW[i];
		
		X_B0 = (SensorState[AHRSID].RawMAG[0] -  B[0]);
		Y_B1 = (SensorState[AHRSID].RawMAG[1] -  B[1]);
		Z_B2 = (SensorState[AHRSID].RawMAG[2] -  B[2]);
		
		AttitudeInfo[AHRSID].CalMAG[0] = B[3]*X_B0 + B[6]*Y_B1 + B[8]*Z_B2;
		AttitudeInfo[AHRSID].CalMAG[1] = B[6]*X_B0 + B[4]*Y_B1 + B[7]*Z_B2;
		AttitudeInfo[AHRSID].CalMAG[2] = B[8]*X_B0 + B[7]*Y_B1 + B[5]*Z_B2;
	}
	else {
		AttitudeInfo[AHRSID].CalMAG[0] = 0;
		AttitudeInfo[AHRSID].CalMAG[1] = 0;
		AttitudeInfo[AHRSID].CalMAG[2] = 0;
	}
}
void nvtGetCalibratedMAG(float* CalMAG)
{
	CalMAG[0] = AttitudeInfo[AHRSID].CalMAG[0];
	CalMAG[1] = AttitudeInfo[AHRSID].CalMAG[1];
	CalMAG[2] = AttitudeInfo[AHRSID].CalMAG[2];
}
void nvtGetCalibratedHALL(float* CalHALL)
{
	CalHALL[0] = SensorState[0].RawHALL[0];
	CalHALL[1] = SensorState[0].RawHALL[1];
}
int8_t nvtCalMAGBufferFill()
{	
	int16_t byteIndex;
	
	byteIndex = BufferIndexMag[AHRSID]*3;
	BufferRawMAG[AHRSID][byteIndex] = SensorState[AHRSID].RawMAG[0];
	BufferRawMAG[AHRSID][byteIndex+1] = SensorState[AHRSID].RawMAG[1];
	BufferRawMAG[AHRSID][byteIndex+2] = SensorState[AHRSID].RawMAG[2];
	//DBG_PRINT("%f  %f  %f\n", SensorState.RawMAG[0], SensorState.RawMAG[1], SensorState.RawMAG[2]);
	BufferIndexMag[AHRSID]++;

	if(BufferIndexMag[AHRSID]>=SAMPLE_SIZE_MAG) {
			MagCalibrate(&BufferRawMAG[AHRSID][0], SAMPLE_SIZE_MAG);
			UpdateCalibrateInfoMAG();
			SensorState[AHRSID].RawSmooth.enable[MAG] = CalInfo[AHRSID].RestoreRawSmooth[MAG];
			return STATUS_CAL_DONE;
	}
	else
		return STATUS_BUFFER_NOT_FILLED;
}
void nvtSetMagGaussPLSB(float GaussPLSB)
{
	CalInfo[AHRSID].MagGaussPLSB = GaussPLSB;
}
uint8_t nvtGetMagCalQFactor()
{
	uint8_t QF;

	if((CalInfo[AHRSID].MagCalQuality>255)||(CalInfo[AHRSID].MagCalQuality<-255)||(CalInfo[AHRSID].MagCalQuality==0))
		QF=255;
	else
		QF = (uint8_t)fabs(CalInfo[AHRSID].MagCalQuality);

	return QF;
}
void SetMagGuass()
{
	MagGuass[AHRSID] = sqrt(AttitudeInfo[AHRSID].CalMAG[0]*AttitudeInfo[AHRSID].CalMAG[0]+ AttitudeInfo[AHRSID].CalMAG[1]*AttitudeInfo[AHRSID].CalMAG[1]+
	AttitudeInfo[AHRSID].CalMAG[2]*AttitudeInfo[AHRSID].CalMAG[2]);
}
float GetMagGuass()
{
	return MagGuass[AHRSID];
}
