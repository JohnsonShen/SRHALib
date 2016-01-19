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
#ifndef _GYRO_DRIFT_CAL_H
#define _GYRO_DRIFT_CAL_H
#ifdef NUC122
#define GYRO_SAMPLE_NUMBER 100
#else
#define GYRO_SAMPLE_NUMBER 256
#endif
#include "Common.h"
typedef struct   
{ 
	int16_t buffer[GYRO_SAMPLE_NUMBER];
  float mean;
  int16_t median;
  int16_t mode;
  float std_dev;
  int16_t empirical[3];
	int16_t  buffercount;
} GyroDriftType; 
void GyroDynamicCalibrate(int16_t gx, int16_t gy, int16_t gz);
void GyroDynamicInit(void);
void GetGyroDynamicCenter(float* gx, float* gy, float* gz);
bool GyroDynamicGetSteady(void);
#endif
