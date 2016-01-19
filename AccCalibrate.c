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
int16_t BufferRawACC[MAX_AHRS][SAMPLE_SIZE_ACC*3];
int16_t BufferIndexACC[MAX_AHRS][SIDE_ACC];
int8_t	bufferFillFlag[MAX_AHRS];

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
FilterACC_T FilterACC[MAX_AHRS];
void FilterACCInit()
{
	int i,j;
  for(j=0;j<MAX_AHRS;j++) {
    FilterACC[j].state = FILTER_STATE_FIRST;
    FilterACC[j].first_pass_count = 0;
    FilterACC[j].second_pass_count = 0;
	for(i=0;i<3;i++) {
      FilterACC[j].sum[i] = 0;
      FilterACC[j].sum_squares[i] = 0;
      FilterACC[j].variance[i] = 0;
      FilterACC[j].sum_acc[i] = 0;
      FilterACC[j].out_acc[i] = 0;
    }
	}
}
void nvtCalACCInit()
{
	int i,j;
  for(j=0;j<MAX_AHRS;j++) {
	for(i=0;i<SIDE_ACC;i++)
      BufferIndexACC[j][i] = 0;
	
    SensorState[j].beCalibrated[ACC] = false;
    bufferFillFlag[j] = 0;
  }
}
void UpdateCalibrateInfoACC()
{
	float *BetaACC;
	BetaACC = GetCalibrateParams(SENSOR_ACC);
	CalInfo[AHRSID].AccMean[0] = BetaACC[0];
	CalInfo[AHRSID].AccMean[1] = BetaACC[1];
	CalInfo[AHRSID].AccMean[2] = BetaACC[2];
	CalInfo[AHRSID].AccScale[0] = BetaACC[3];
	CalInfo[AHRSID].AccScale[1] = BetaACC[4];
	CalInfo[AHRSID].AccScale[2]= BetaACC[5];
	SensorState[AHRSID].beCalibrated[ACC]  = true;
}
void SetCalibrateACC()
{
	AttitudeInfo[AHRSID].CalACC[0] = (SensorState[AHRSID].RawACC[0] -  CalInfo[AHRSID].AccMean[0]) * CalInfo[AHRSID].AccScale[0];
	AttitudeInfo[AHRSID].CalACC[1] = (SensorState[AHRSID].RawACC[1] -  CalInfo[AHRSID].AccMean[1]) * CalInfo[AHRSID].AccScale[1];
	AttitudeInfo[AHRSID].CalACC[2] = (SensorState[AHRSID].RawACC[2] -  CalInfo[AHRSID].AccMean[2]) * CalInfo[AHRSID].AccScale[2];
}
void nvtGetCalibratedACC(float* CalACC)
{
	CalACC[0] = AttitudeInfo[AHRSID].CalACC[0];
	CalACC[1] = AttitudeInfo[AHRSID].CalACC[1];
	CalACC[2] = AttitudeInfo[AHRSID].CalACC[2];
}
//ACC Samples Varience Check
bool SampleFilter() 
{
	if(FilterACC[AHRSID].state==FILTER_STATE_FIRST) {
		int i;
		for(i = 0;i < 3;i++) {
			FilterACC[AHRSID].sum[i]+=SensorState[AHRSID].RawACC[i];
			FilterACC[AHRSID].sum_squares[i]+=SensorState[AHRSID].RawACC[i]*SensorState[AHRSID].RawACC[i];
		}
		FilterACC[AHRSID].first_pass_count++;
		if(FilterACC[AHRSID].first_pass_count>=FILTER_FIRST_PASS_COUNT) {
			for(i=0;i<3; i++) {
				FilterACC[AHRSID].variance[i] = 1 + FilterACC[AHRSID].sum_squares[i] - (FilterACC[AHRSID].sum[i]*FilterACC[AHRSID].sum[i])/FILTER_FIRST_PASS_COUNT;
			}
			FilterACC[AHRSID].state=FILTER_STATE_SECOND;
		}
		return false;
	}
	else if(FilterACC[AHRSID].state==FILTER_STATE_SECOND) {
		unsigned long dx;
    unsigned long dy;
    unsigned long dz;
		dx = SensorState[AHRSID].RawACC[0] - FilterACC[AHRSID].sum[0]/FILTER_SECOND_PASS_COUNT;
    dy = SensorState[AHRSID].RawACC[1] - FilterACC[AHRSID].sum[1]/FILTER_SECOND_PASS_COUNT;
    dz = SensorState[AHRSID].RawACC[2] - FilterACC[AHRSID].sum[2]/FILTER_SECOND_PASS_COUNT;
		
		if((dx*dx)<9*FilterACC[AHRSID].variance[0]&&(dy*dy)<9*FilterACC[AHRSID].variance[1]&&(dz*dz)<9*FilterACC[AHRSID].variance[2]) {
        FilterACC[AHRSID].second_pass_count++;
        FilterACC[AHRSID].sum_acc[0] += SensorState[AHRSID].RawACC[0];
        FilterACC[AHRSID].sum_acc[1] += SensorState[AHRSID].RawACC[1];
        FilterACC[AHRSID].sum_acc[2] += SensorState[AHRSID].RawACC[2];
		}
		
		if(FilterACC[AHRSID].second_pass_count>=FILTER_SECOND_PASS_COUNT) {
			FilterACC[AHRSID].out_acc[0] = FilterACC[AHRSID].sum_acc[0]/FILTER_SECOND_PASS_COUNT;
      FilterACC[AHRSID].out_acc[1] = FilterACC[AHRSID].sum_acc[1]/FILTER_SECOND_PASS_COUNT;
      FilterACC[AHRSID].out_acc[2] = FilterACC[AHRSID].sum_acc[2]/FILTER_SECOND_PASS_COUNT;
			FilterACC[AHRSID].state=FILTER_STATE_DONE;
			return true;
		}
		else
			return false;
	}
	else if(FilterACC[AHRSID].state==FILTER_STATE_DONE) {
		FilterACCInit();
		return false;
	}
	return false;
}

int8_t nvtCalACCBufferFill(int8_t Dir)
{	
	int16_t Index;
	
	if(SampleFilter()) {
		Index = (Dir*SAMPLE_ACC_PER_SIDE+BufferIndexACC[AHRSID][Dir])*3;
		*(BufferRawACC[AHRSID]+Index) = FilterACC[AHRSID].out_acc[0];
		*(BufferRawACC[AHRSID]+Index+1) = FilterACC[AHRSID].out_acc[1];
		*(BufferRawACC[AHRSID]+Index+2) = FilterACC[AHRSID].out_acc[2];
		BufferIndexACC[AHRSID][Dir]++;
	}
	if(BufferIndexACC[AHRSID][Dir]>=SAMPLE_ACC_PER_SIDE) {
		bufferFillFlag[AHRSID]|=(1<<Dir);
		if(bufferFillFlag[AHRSID]==0x3F) {
			AccCalibrate(BufferRawACC[AHRSID], SAMPLE_SIZE_ACC);
			UpdateCalibrateInfoACC();
			return STATUS_CAL_DONE;
		}
		else if(Dir==0){
			AccZCalibrate(BufferRawACC[AHRSID], SAMPLE_ACC_PER_SIDE);
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
	CalInfo[AHRSID].AccG_PER_LSB = G_PER_LSB;
}
