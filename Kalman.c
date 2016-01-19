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
#include "Kalman.h"
float Zk[FILTER_NUM];
float Xk[FILTER_NUM], Xk_1[FILTER_NUM],Pk[FILTER_NUM], Pk_1[FILTER_NUM], Kk[FILTER_NUM], R[FILTER_NUM],Q[FILTER_NUM];
void KalmanInit(char index, float noise)
{
        if(noise==0)
		noise = 0.0000001f;
	Xk_1[index] = 0;
	/*Pk_1[index] = 10;
	R[index] = 0.05;
	Q[index] = 0.0001;*/
	
	Pk_1[index] = noise;
	R[index] = noise/50;
	Q[index] = noise/10000;
}
void TimeUpdate(char index)
{
	Xk[index] = Xk_1[index];
	Pk[index] = Pk_1[index] + Q[index];
}
float MeasurementUpdate(char index, float measure)
{
	Kk[index] = Pk[index]/(Pk[index] + R[index]);
	Xk[index] = Xk[index] + Kk[index]*(measure - Xk[index]);
	Pk[index] = (1 - Kk[index])*Pk[index];
	Xk_1[index] = Xk[index];
	Pk_1[index] = Pk[index];
	//if(index==0)
	//	printf("Xk:%f  KK:%f    Pk:%f\n",Xk[index],Kk[index],Pk[index]);
	return Xk[index];
}
