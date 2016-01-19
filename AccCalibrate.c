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
#define FILTER_STATE_FIRST 		0
#define FILTER_STATE_SECOND  	1
#define FILTER_STATE_DONE  		2
#define FILTER_FIRST_PASS_COUNT  			32
#define FILTER_SECOND_PASS_COUNT  		32
int16_t BufferRawACC[SAMPLE_SIZE_ACC*3];
int16_t BufferIndexACC[SIDE_ACC];
int8_t	bufferFillFlag;

typedef struct {
	char		state;
	char		first_pass_count;
	char		second_pass_count;
	int32_t sum[3];
  unsigned long sum_squares[3];
  unsigned long variance[3];
	int32_t sum_acc[3];
	int16_t out_acc[3];
} FilterACC_T;
FilterACC_T FilterACC;
void FilterACCInit()
{
	int i;
	FilterACC.state = FILTER_STATE_FIRST;
	FilterACC.first_pass_count = 0;
	FilterACC.second_pass_count = 0;
	for(i=0;i<3;i++) {
		FilterACC.sum[i] = 0;
		FilterACC.sum_squares[i] = 0;
		FilterACC.variance[i] = 0;
		FilterACC.sum_acc[i] = 0;
		FilterACC.out_acc[i] = 0;
	}
}
void nvtCalACCInit()
{
	int i;
	for(i=0;i<SIDE_ACC;i++)
		BufferIndexACC[i] = 0;
	
	SensorState.beCalibrated[ACC] = false;
	bufferFillFlag = 0;
}
void UpdateCalibrateInfoACC()
{
	float *BetaACC;
	BetaACC = GetCalibrateParams(SENSOR_ACC);
	CalInfo.AccMean[0] = BetaACC[0];
	CalInfo.AccMean[1] = BetaACC[1];
	CalInfo.AccMean[2] = BetaACC[2];
	CalInfo.AccScale[0] = BetaACC[3];
	CalInfo.AccScale[1] = BetaACC[4];
	CalInfo.AccScale[2]= BetaACC[5];
	SensorState.beCalibrated[ACC]  = true;
}
void SetCalibrateACC()
{
	AttitudeInfo.CalACC[0] = (SensorState.RawACC[0] -  CalInfo.AccMean[0]) * CalInfo.AccScale[0];
	AttitudeInfo.CalACC[1] = (SensorState.RawACC[1] -  CalInfo.AccMean[1]) * CalInfo.AccScale[1];
	AttitudeInfo.CalACC[2] = (SensorState.RawACC[2] -  CalInfo.AccMean[2]) * CalInfo.AccScale[2];
}
void nvtGetCalibratedACC(float* CalACC)
{
	CalACC[0] = AttitudeInfo.CalACC[0];
	CalACC[1] = AttitudeInfo.CalACC[1];
	CalACC[2] = AttitudeInfo.CalACC[2];
}
//ACC Samples Varience Check
bool SampleFilter() 
{
	if(FilterACC.state==FILTER_STATE_FIRST) {
		int i;
		for(i = 0;i < 3;i++) {
			FilterACC.sum[i]+=SensorState.RawACC[i];
			FilterACC.sum_squares[i]+=SensorState.RawACC[i]*SensorState.RawACC[i];
		}
		FilterACC.first_pass_count++;
		if(FilterACC.first_pass_count>=FILTER_FIRST_PASS_COUNT) {
			for(i=0;i<3; i++) {
				FilterACC.variance[i] = 1 + FilterACC.sum_squares[i] - (FilterACC.sum[i]*FilterACC.sum[i])/FILTER_FIRST_PASS_COUNT;
			}
			FilterACC.state=FILTER_STATE_SECOND;
		}
		return false;
	}
	else if(FilterACC.state==FILTER_STATE_SECOND) {
		unsigned long dx;
    unsigned long dy;
    unsigned long dz;
		dx = SensorState.RawACC[0] - FilterACC.sum[0]/FILTER_SECOND_PASS_COUNT;
    dy = SensorState.RawACC[1] - FilterACC.sum[1]/FILTER_SECOND_PASS_COUNT;
    dz = SensorState.RawACC[2] - FilterACC.sum[2]/FILTER_SECOND_PASS_COUNT;
		
		if((dx*dx)<9*FilterACC.variance[0]&&(dy*dy)<9*FilterACC.variance[1]&&(dz*dz)<9*FilterACC.variance[2]) {
        FilterACC.second_pass_count++;
        FilterACC.sum_acc[0] += SensorState.RawACC[0];
        FilterACC.sum_acc[1] += SensorState.RawACC[1];
        FilterACC.sum_acc[2] += SensorState.RawACC[2];
		}
		
		if(FilterACC.second_pass_count>=FILTER_SECOND_PASS_COUNT) {
			FilterACC.out_acc[0] = FilterACC.sum_acc[0]/FILTER_SECOND_PASS_COUNT;
      FilterACC.out_acc[1] = FilterACC.sum_acc[1]/FILTER_SECOND_PASS_COUNT;
      FilterACC.out_acc[2] = FilterACC.sum_acc[2]/FILTER_SECOND_PASS_COUNT;
			FilterACC.state=FILTER_STATE_DONE;
			return true;
		}
		else
			return false;
	}
	else if(FilterACC.state==FILTER_STATE_DONE) {
		FilterACCInit();
		return false;
	}
	return false;
}

int8_t nvtCalACCBufferFill(int8_t Dir)
{	
	int16_t Index;
	
	if(SampleFilter()) {
		Index = (Dir*SAMPLE_ACC_PER_SIDE+BufferIndexACC[Dir])*3;
		*(BufferRawACC+Index) = FilterACC.out_acc[0];
		*(BufferRawACC+Index+1) = FilterACC.out_acc[1];
		*(BufferRawACC+Index+2) = FilterACC.out_acc[2];
		BufferIndexACC[Dir]++;
	}
	if(BufferIndexACC[Dir]>=SAMPLE_ACC_PER_SIDE) {
		bufferFillFlag|=(1<<Dir);
		if(bufferFillFlag==0x3F) {
			AccCalibrate(BufferRawACC, SAMPLE_SIZE_ACC);
			UpdateCalibrateInfoACC();
			return STATUS_CAL_DONE;
		}
		else if(Dir==0){
			AccZCalibrate(BufferRawACC, SAMPLE_ACC_PER_SIDE);
			UpdateCalibrateInfoACC();
			return STATUS_BUFFER_FILLED;
		}
		else {
			return STATUS_BUFFER_FILLED;
	}
	}
	else
		return STATUS_BUFFER_NOT_FILLED;
}
void nvtSetAccG_PER_LSB(float G_PER_LSB)
{
	CalInfo.AccG_PER_LSB = G_PER_LSB;
}
