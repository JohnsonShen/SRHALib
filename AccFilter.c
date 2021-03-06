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
#include "AccFilter.h"

static int32_t    accelStoredFilterValues[MAX_AHRS][3];
static uint8_t    imuAccLpfAttFactor[MAX_AHRS];
static int32_t 		accLPF32[MAX_AHRS][3] = {0, 0, 1},accSmooth[MAX_AHRS][3];
void AccFilterInit()
{
  int j;
  for(j=0;j<MAX_AHRS;j++)
    imuAccLpfAttFactor[j] = IMU_ACC_IIR_LPF_ATT_FACTOR;
}
int16_t iirLPFilterSingle(int32_t in, int32_t attenuation,  int32_t* filt)
{
  int32_t inScaled;
  int32_t filttmp = *filt;
  int16_t out;

  if (attenuation > (1<<IIR_SHIFT))
  {
    attenuation = (1<<IIR_SHIFT);
  }
  else if (attenuation < 1)
  {
    attenuation = 1;
  }

  // Shift to keep accuracy
  inScaled = in << IIR_SHIFT;
  // Calculate IIR filter
  filttmp = filttmp + (((inScaled-filttmp) >> IIR_SHIFT) * attenuation);
  // Scale and round
  out = (filttmp >> 8) + ((filttmp & (1 << (IIR_SHIFT - 1))) >> (IIR_SHIFT - 1));
  *filt = filttmp;

  return out;
}
void imuAccIIRLPFilter(int16_t* in, int16_t* out)
{
	imuAccLpfAttFactor[AHRSID] = IMU_ACC_IIR_LPF_ATT_FACTOR;
  out[0] = iirLPFilterSingle(in[0], imuAccLpfAttFactor[AHRSID], &accelStoredFilterValues[AHRSID][0]);
  out[1] = iirLPFilterSingle(in[1], imuAccLpfAttFactor[AHRSID], &accelStoredFilterValues[AHRSID][1]);
  out[2] = iirLPFilterSingle(in[2], imuAccLpfAttFactor[AHRSID], &accelStoredFilterValues[AHRSID][2]);
}
/* Multiwii Filter*/
void FilterCalibrateACC(float* CalACC, int32_t* RawACC)
{
	CalACC[0] = (RawACC[0] -  CalInfo[AHRSID].AccMean[0]) * CalInfo[AHRSID].AccScale[0];
	CalACC[1] = (RawACC[1] -  CalInfo[AHRSID].AccMean[1]) * CalInfo[AHRSID].AccScale[1];
	CalACC[2] = (RawACC[2] -  CalInfo[AHRSID].AccMean[2]) * CalInfo[AHRSID].AccScale[2];
}
void MultiwiiAccFilter(int16_t* in, int16_t* out)
{
	uint8_t axis;
	float OutACC[3],validAcc, NonZ_Norm, NonZ, OutZSqr,Norm;
	
	 // Initialization
  for (axis = 0; axis < 3; axis++) {
    accLPF32[AHRSID][axis]    -= accLPF32[AHRSID][axis]>>ACC_LPF_FACTOR;
    accLPF32[AHRSID][axis]    += in[axis];
    accSmooth[AHRSID][axis]    = accLPF32[AHRSID][axis]>>ACC_LPF_FACTOR;
  }
	FilterCalibrateACC(OutACC, accSmooth[AHRSID]);
	OutZSqr = OutACC[2]*OutACC[2];
	validAcc = OutACC[0]*OutACC[0]+OutACC[1]*OutACC[1]+OutZSqr;
	
	//printf("ValidAcc:%f\n",validAcc);
	if((validAcc<1.33f)&&(validAcc>0.72f)) {
		out[0] = accSmooth[AHRSID][0];
		out[1] = accSmooth[AHRSID][1];
		out[2] = accSmooth[AHRSID][2];
	}
	else {
		NonZ_Norm = 1 - OutZSqr;
		NonZ = validAcc - OutZSqr;
		Norm = sqrt(NonZ_Norm/NonZ);
		out[0] = accSmooth[AHRSID][0]*Norm;
		out[1] = accSmooth[AHRSID][1]*Norm;
		out[2] = accSmooth[AHRSID][2];
	}
	
}

