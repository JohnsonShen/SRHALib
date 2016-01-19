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
#include "GyroDriftCalibrate.h"
#define STD_DEV_SUM_TH 30
#define STD_DEV_TH  2
static GyroDriftType gyroDrift[3];
int16_t HistogramValue[GYRO_SAMPLE_NUMBER],HistogramCount[GYRO_SAMPLE_NUMBER];
void shiftBufferR(int16_t start,int16_t* buffer)
{
	int i;
	for(i=(GYRO_SAMPLE_NUMBER-1); i>start; i--) {
		buffer[i]=buffer[i-1];
	}
}
//qsort (x, sizeof(x)/sizeof(*x), sizeof(*x), comp);
bool pushBuffer(GyroDriftType* gyroDrift, int16_t data)
{
	gyroDrift->buffer[(gyroDrift->buffercount++)%GYRO_SAMPLE_NUMBER]=data;
	if(gyroDrift->buffercount<GYRO_SAMPLE_NUMBER){
		return false;
	}
	else {
		return true;
	}
}
void CheckMean(GyroDriftType* gyroDrift)
{
	int i,sum=0;
	for(i=0;i<GYRO_SAMPLE_NUMBER;i++)
		sum+=gyroDrift->buffer[i];
	
	gyroDrift->mean=(float)sum/GYRO_SAMPLE_NUMBER;
}
void CheckStandardDV(GyroDriftType* gyroDrift)
{
	int i;
	float sum=0,err;

	for(i=0;i<GYRO_SAMPLE_NUMBER;i++) {
		err = (gyroDrift->buffer[i]-gyroDrift->mean);
		sum+=err*err;
	}
	sum/=(GYRO_SAMPLE_NUMBER-1);
	gyroDrift->std_dev= sqrt(sum);
}
void CheckMedian(GyroDriftType* gyroDrift)
{
	gyroDrift->median=gyroDrift->buffer[GYRO_SAMPLE_NUMBER/2];
}
void CheckMode(GyroDriftType* gyroDrift)
{
	int16_t i,j,HistogramNum=0,hit,MaxHistogram=0,MaxHistogramIndex=0;
	for(i=0;i<GYRO_SAMPLE_NUMBER;i++) {
		hit=0;
		for(j=0;j<HistogramNum;j++) {
			if(gyroDrift->buffer[i]==HistogramValue[j]) {
				HistogramCount[j]++;
				hit=1;
			}
		}
		if(!hit) {
			HistogramValue[HistogramNum]=gyroDrift->buffer[i];
			HistogramCount[HistogramNum++]++;
		}	
	}
	for(j=0;j<HistogramNum;j++) {
		if(MaxHistogram<HistogramCount[j]) {
			MaxHistogram=HistogramCount[j];
			MaxHistogramIndex=j;
		}
	}
	gyroDrift->mode=HistogramValue[MaxHistogramIndex];
}
void CheckEmpirical(GyroDriftType* gyroDrift)
{
	int16_t i, empirical_point[6]={GYRO_SAMPLE_NUMBER, GYRO_SAMPLE_NUMBER, GYRO_SAMPLE_NUMBER,
		GYRO_SAMPLE_NUMBER, GYRO_SAMPLE_NUMBER, GYRO_SAMPLE_NUMBER};
	for(i=0;i<GYRO_SAMPLE_NUMBER;i++) {
		if(gyroDrift->buffer[i]<(gyroDrift->median-gyroDrift->std_dev*3)) {
			empirical_point[0]=i;
		}
		else if(gyroDrift->buffer[i]<(gyroDrift->median-gyroDrift->std_dev*2)) {
			empirical_point[1]=i;
		}
		else if(gyroDrift->buffer[i]<(gyroDrift->median-gyroDrift->std_dev)) {
			empirical_point[2]=i;
		}
		else if(gyroDrift->buffer[i]<(gyroDrift->median+gyroDrift->std_dev)) {
			empirical_point[3]=i;
		}
		else if(gyroDrift->buffer[i]<(gyroDrift->median+gyroDrift->std_dev*2)) {
			empirical_point[4]=i;
		}
		else if(gyroDrift->buffer[i]<(gyroDrift->median+gyroDrift->std_dev*3)) {
			empirical_point[5]=i;
		}
	}
	gyroDrift->empirical[2]=((empirical_point[5]-empirical_point[0] +1)*100)/GYRO_SAMPLE_NUMBER;
	gyroDrift->empirical[1]=((empirical_point[4]-empirical_point[1]+1)*100)/GYRO_SAMPLE_NUMBER;
	gyroDrift->empirical[0]=((empirical_point[3]-empirical_point[2]+1)*100)/GYRO_SAMPLE_NUMBER;
}
int8_t comp (const void * elem1, const void * elem2) 
{
	
		int f = *((int*)elem1);
    int s = *((int*)elem2);
    if (f > s) return  1;
    if (f < s) return -1;
    return 0;
}
// A utility function to swap two elements
void swap ( int16_t* a, int16_t* b )
{
    int16_t t = *a;
    *a = *b;
    *b = t;
}
 
/* This function is same in both iterative and recursive*/
int16_t partition (int16_t arr[], int16_t l, int16_t h)
{
    int16_t x = arr[h];
    int16_t i = (l - 1), j;
 
    for (j = l; j <= h- 1; j++)
    {
        if (arr[j] <= x)
        {
            i++;
            swap (&arr[i], &arr[j]);
        }
    }
    swap (&arr[i + 1], &arr[h]);
    return (i + 1);
}
 
/* A[] --> Array to be sorted, l  --> Starting index, h  --> Ending index */
void quickSortIterative (int16_t arr[], int16_t l, int16_t h)
{
    // Create an auxiliary stack
    int16_t stack[GYRO_SAMPLE_NUMBER];
 
    // initialize top of stack
    int16_t top = -1;
		int16_t p;
    // push initial values of l and h to stack
    stack[ ++top ] = l;
    stack[ ++top ] = h;
 
    // Keep popping from stack while is not empty
    while ( top >= 0 )
    {
        // Pop h and l
        h = stack[ top-- ];
        l = stack[ top-- ];
 
        // Set pivot element at its correct position in sorted array
        p = partition( arr, l, h );
 
        // If there are elements on left side of pivot, then push left
        // side to stack
        if ( p-1 > l )
        {
            stack[ ++top ] = l;
            stack[ ++top ] = p - 1;
        }
 
        // If there are elements on right side of pivot, then push right
        // side to stack
        if ( p+1 < h )
        {
            stack[ ++top ] = p + 1;
            stack[ ++top ] = h;
        }
    }
}
void GetGyroDynamicCenter(float* gx, float* gy, float* gz)
{
	*gx=SensorState.GyroDynamicCenter[0];
	*gy=SensorState.GyroDynamicCenter[1];
	*gz=SensorState.GyroDynamicCenter[2];
}
void CheckNormalDistribution()
{
	float std_dev_sum;
	std_dev_sum = gyroDrift[0].std_dev + gyroDrift[1].std_dev + gyroDrift[2].std_dev;
	//if((gyroDrift[0].std_dev<=STD_DEV_TH)&&(gyroDrift[1].std_dev<=STD_DEV_TH)&&(gyroDrift[2].std_dev<=STD_DEV_TH)) 
	if(std_dev_sum<STD_DEV_SUM_TH)
	{
		SensorState.GyroDynamicCenter[0]=gyroDrift[0].mean;
		SensorState.GyroDynamicCenter[1]=gyroDrift[1].mean;
		SensorState.GyroDynamicCenter[2]=gyroDrift[2].mean;
		SensorState.beGyroSteady = true;
	}
	else {
		SensorState.beGyroSteady = false;
	}
}
int8_t CheckGyroNormalParam(GyroDriftType* gyroDrift)
{
	quickSortIterative (&gyroDrift->buffer[0],0,GYRO_SAMPLE_NUMBER-1);
	CheckMean(gyroDrift);
	CheckStandardDV(gyroDrift);
	CheckMedian(gyroDrift);
	CheckMode(gyroDrift);
	CheckEmpirical(gyroDrift);
	gyroDrift->buffercount=0;
	return STATUS_NORMAL;
#if 0
	DBG_PRINTF("mean:%f\n",gyroDrift->mean);
	DBG_PRINTF("median:%d\n",gyroDrift->median);
	DBG_PRINTF("mode:%d\n",gyroDrift->mode);
	DBG_PRINTF("std_dev:%f\n",gyroDrift->std_dev);
	DBG_PRINTF("empirical:%d %d %d\n",gyroDrift->empirical[0],gyroDrift->empirical[1],gyroDrift->empirical[2]);
#endif
}
void GyroDynamicCalibrate(int16_t gx, int16_t gy, int16_t gz)
{
	bool dataReady;
	dataReady=pushBuffer(&gyroDrift[0], gx);
	dataReady=pushBuffer(&gyroDrift[1], gy);
	dataReady=pushBuffer(&gyroDrift[2], gz);
	
	if(dataReady) {
		CheckGyroNormalParam(&gyroDrift[0]);
		CheckGyroNormalParam(&gyroDrift[1]);
		CheckGyroNormalParam(&gyroDrift[2]);
		CheckNormalDistribution();
	}
}
GyroDriftType GyroDynamicGetDrift(int8_t axis)
{
	return gyroDrift[axis];
}
bool GyroDynamicGetSteady()
{
	return SensorState.beGyroSteady;
}
void GyroDynamicInit()
{
	/*int16_t i;
	for(i=0; i<GYRO_SAMPLE_NUMBER; i++) {
		gyroDrift[0].buffer[i]=65535;
		gyroDrift[1].buffer[i]=65535;
		gyroDrift[2].buffer[i]=65535;
	}*/
}

