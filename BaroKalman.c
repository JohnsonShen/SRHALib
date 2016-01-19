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
#include "Kalman.h"
#define BARO_SAMPLE_NUMBER 20
#define BARO_INIT_COUNT 100
bool beInitialize = false;
bool beSTDDV = false;
int  start_counter = 0;
typedef struct   
{ 
	int16_t buffer[BARO_SAMPLE_NUMBER];
  float mean;
  float std_dev;
	int16_t  buffercount;
} StandardDVType; 
StandardDVType Pressure;
StandardDVType Temperature;
bool PutBuffer(StandardDVType* baro, int16_t data)
{
	baro->buffer[(baro->buffercount++)%BARO_SAMPLE_NUMBER]=data;
	if(baro->buffercount<(BARO_SAMPLE_NUMBER-1)){
		return false;
	}
	else {
		return true;
	}
}
void CalMean(StandardDVType* baro)
{
	int i,sum=0;
	for(i=0;i<BARO_SAMPLE_NUMBER;i++)
		sum+=baro->buffer[i];
	
	baro->mean=(float)sum/BARO_SAMPLE_NUMBER;
}
void CalStandardDV(StandardDVType* baro)
{
	int i;
	float sum=0,err;

	for(i=0;i<BARO_SAMPLE_NUMBER;i++) {
		err = (baro->buffer[i]-baro->mean);
		sum+=err*err;
	}
	sum/=(BARO_SAMPLE_NUMBER-1);
	
	baro->std_dev = sqrt(sum);
}
void BaroFilter(int16_t *raw)
{
	if(beInitialize==false) {
		Pressure.buffercount = 0;
		Temperature.buffercount = 0;
		if(start_counter++>BARO_INIT_COUNT)
			beInitialize = true;
		
		return;
	}
	if(beSTDDV==true) {
		float temp;
		TimeUpdate(0);
		temp = MeasurementUpdate(0, raw[0]);
		//temp = kalmanFilter1(raw[0]);
		raw[0] = temp;
		TimeUpdate(1);
		temp = MeasurementUpdate(1, raw[1]);
		//temp = kalmanFilter2(raw[1]);
		raw[1] = temp;
	}
	else {
		bool BufferFull[FILTER_NUM];
		BufferFull[0] = PutBuffer(&Pressure, raw[0]);
		BufferFull[1] = PutBuffer(&Temperature, raw[1]);
		if((BufferFull[0]==true)&&(BufferFull[1]==true)) {
			CalMean(&Pressure);
			CalMean(&Temperature);
			CalStandardDV(&Pressure);
			CalStandardDV(&Temperature);
			KalmanInit(0, Pressure.std_dev);
			KalmanInit(1, Temperature.std_dev);
			//printf("P:%f    T:%f\n",Pressure.std_dev,Temperature.std_dev);
			beSTDDV = true;
		}
	}
}
int16_t kalmanFilter1(int16_t inData)
{                        
    static float prevData=0;
    static float p=10, q=0.0001, r=0.05, kGain=0;
 
    //Kalman filter function start*******************************
    p = p+q;
    kGain = p/(p+r);
 
    inData = prevData+(kGain*(inData-prevData));
 
    p = (1-kGain)*p;
 
	prevData = inData;
    //Kalman filter function stop********************************
	//printf("Xk:%d  KK:%f    Pk:%f\n",inData,kGain,p);
    return inData;
}
int16_t kalmanFilter2(int16_t inData)
{                        
    static float prevData=0;
    static float p=10, q=0.0001, r=0.05, kGain=0;
 
    //Kalman filter function start*******************************
    p = p+q;
    kGain = p/(p+r);
 
    inData = prevData+(kGain*(inData-prevData));
 
    p = (1-kGain)*p;
 
	prevData = inData;
    //Kalman filter function stop********************************
 
    return inData;
}
