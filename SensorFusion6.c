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
#include "M451Series.h"
#include <stdio.h>
#include <math.h>
#include <arm_math.h>
#include "Common.h"
#include "AHRSLib.h"
#include "GyroDriftCalibrate.h"
extern TimeFrameInfo TimeInfo;
#define TWO_KP_DEF	(2.0f * 0.5f)	// 2 * proportional gain
#define TWO_KI_DEF	(2.0f * 0.003f)	// 2 * integral gain
//#define TWO_KP_DEF	(2.0f * 0.4f)	// 2 * proportional gain
//#define TWO_KI_DEF	(2.0f * 0.001f)	// 2 * integral gain
#define GUASS_ERROR_TH 0.15f
#define ACC_TH 0.1f
#define VG_TH 0.01f
#define SPEED_DEC_RATIO 0.99f
#define MAG_MASTER_TIME 1000 /* ms */

float twoKp = TWO_KP_DEF;    // 2 * proportional gain (Kp)
float twoKi = TWO_KI_DEF;    // 2 * integral gain (Ki)
float integralFBx[MAX_AHRS] = {0.0f,0.0f};
float integralFBy[MAX_AHRS] = {0.0f,0.0f};
float integralFBz[MAX_AHRS] = {0.0f,0.0f}; // integral error terms scaled by Ki

float gx[MAX_AHRS] = {0.0f,0.0f};
float gy[MAX_AHRS] = {0.0f,0.0f};
float gz[MAX_AHRS] = {0.0f,0.0f};

float qx[MAX_AHRS] = {0.0f,0.0f};
float qy[MAX_AHRS] = {0.0f,0.0f};
float qz[MAX_AHRS] = {0.0f,0.0f};

float gx_deg[MAX_AHRS] = {0.0f,0.0f};
float gy_deg[MAX_AHRS] = {0.0f,0.0f};
float gz_deg[MAX_AHRS] = {0.0f,0.0f};

float ax[MAX_AHRS] = {0.0f,0.0f};
float ay[MAX_AHRS] = {0.0f,0.0f};
float az[MAX_AHRS] = {0.0f,0.0f};

float EulerR[MAX_AHRS] = {0.0f,0.0f};
float EulerP[MAX_AHRS] = {0.0f,0.0f};
float EulerY[MAX_AHRS] = {0.0f,0.0f};

float EulerRd[MAX_AHRS] = {0.0f,0.0f};
float EulerPd[MAX_AHRS] = {0.0f,0.0f};
float EulerYd[MAX_AHRS] = {0.0f,0.0f};

float mx[MAX_AHRS] = {0.0f,0.0f};
float my[MAX_AHRS] = {0.0f,0.0f};
float mz[MAX_AHRS] = {0.0f,0.0f};
float mag_norm[MAX_AHRS][3];
float mdx[MAX_AHRS] = {0.0f,0.0f};
float mdy[MAX_AHRS] = {0.0f,0.0f};
float mdz[MAX_AHRS] = {0.0f,0.0f};
float mdfx[MAX_AHRS] = {0.0f,0.0f};
float mdfy[MAX_AHRS] = {0.0f,0.0f};
float mdfz[MAX_AHRS] = {0.0f,0.0f};

float q0[MAX_AHRS] = {1.0f,1.0f};
float q1[MAX_AHRS] = {0.0f,0.0f};
float q2[MAX_AHRS] = {0.0f,0.0f};
float q3[MAX_AHRS] = {0.0f,0.0f}; // quaternion of sensor frame relative to auxiliary frame

static float motion[MAX_AHRS][3] = {0};
static float motionlast[MAX_AHRS][3] = {0, 0, 0};
float vg[MAX_AHRS][3] = {0, 0, 0};
float Ve[MAX_AHRS][3] = {0, 0, 0};
float V0[MAX_AHRS][3] = {0, 0, 0};
float Vdiff[MAX_AHRS][3] = {0, 0, 0};
float Alast[MAX_AHRS][3] = {0, 0, 0};
float diam_diff[MAX_AHRS];

float acc_steady[MAX_AHRS]={0};
int MagMasterTime[MAX_AHRS] = {MAG_MASTER_TIME,MAG_MASTER_TIME};

// TODO: Make math util file
static float invSqrt(float x);
void nvtSetFusionParam(float Proportional, float Integral)
{
  twoKp = Proportional*2;
  twoKi = Integral*2;
}
void nvtGetFusionParam(float *Proportional, float *Integral)
{
  *Proportional = twoKp/2.0f;
  *Integral = twoKi/2.0f;
}
void nvtResetDirection()
{
  q0[AHRSID] = 1.0f;
  q1[AHRSID] = 0.0f;
  q2[AHRSID] = 0.0f;
  q3[AHRSID] = 0.0f;
}
void UpdateMagMasterTime()
{
	MagMasterTime[AHRSID] = GetTickCounter() + MAG_MASTER_TIME;
	q0[AHRSID] = 1.0f;
	q1[AHRSID] = 0.0f;
	q2[AHRSID] = 0.0f;
	q3[AHRSID] = 0.0f;
}
void ComputeEuler()
{
	//Compute Euler
  qx[AHRSID] = 2 * (q1[AHRSID]*q3[AHRSID] - q0[AHRSID]*q2[AHRSID]);
  qy[AHRSID] = 2 * (q0[AHRSID]*q1[AHRSID] + q2[AHRSID]*q3[AHRSID]);
  qz[AHRSID] = q0[AHRSID]*q0[AHRSID] - q1[AHRSID]*q1[AHRSID] - q2[AHRSID]*q2[AHRSID] + q3[AHRSID]*q3[AHRSID];

  if (qx[AHRSID]>1.0f) qx[AHRSID]=1.0f;
  if (qx[AHRSID]<-1.0f) qx[AHRSID]=-1.0f;

  EulerY[AHRSID] = atan2f(2*(q0[AHRSID]*q3[AHRSID] + q1[AHRSID]*q2[AHRSID]), q0[AHRSID]*q0[AHRSID] + q1[AHRSID]*q1[AHRSID] - q2[AHRSID]*q2[AHRSID] - q3[AHRSID]*q3[AHRSID]);
	
	EulerR[AHRSID] = atan2f(qy[AHRSID], qz[AHRSID]);
  //EulerP =atan2(-gx, gy*sin(EulerR)+gz*cos(EulerR));
	
	/* Since gy and gz are both very mall the atan2(gy,gz) has a great error, 
	and EullerP should not reference EulerR(let EulerR=0)*/
	//if((qy*qy+qz*qz)<0.15f)
		EulerP[AHRSID] =atan2f(-qx[AHRSID],qz[AHRSID]);//atan2(gx, gz);//asin(gx); //Pitch seems to be inverted
	//else
	//	EulerP =atan(-qx/(qy*sin(EulerR)+qz*cos(EulerR)));//atan2(gx, gz);//asin(gx); //Pitch seems to be inverted
	
  EulerYd[AHRSID] = EulerY[AHRSID] * 180 / M_PI;
  EulerPd[AHRSID] = EulerP[AHRSID] * 180 / M_PI; //Pitch seems to be inverted
  EulerRd[AHRSID] = EulerR[AHRSID] * 180 / M_PI;
}

void filterA(int axis)
{
	if((vg[AHRSID][axis]<=VG_TH)&&(vg[AHRSID][axis]>=-VG_TH)){
		vg[AHRSID][axis] = 0.0;
	}
}
bool CheckAccSteady(float innerZ)
{ 
	if(innerZ<0)
		innerZ = -innerZ;
	if(innerZ<ACC_TH) {
		acc_steady[AHRSID] = 1;
		return true;
	}
	else {
		acc_steady[AHRSID] = 0;
		return false;
	}
}
void ComputeMotion( float axf, float ayf, float azf, float recipNorm, float dt)
{
	float A[3], innerZ;

	vg[AHRSID][0] = axf - qx[AHRSID];
	vg[AHRSID][1] = ayf - qy[AHRSID];
	vg[AHRSID][2] = azf - qz[AHRSID];
	//printf("%f  %f  %f\n",vg[2], azf, qz);
	
	//filterA(0);
	//filterA(1);
	//filterA(2);
	
	innerZ = qx[AHRSID]*vg[AHRSID][0] + qy[AHRSID]*vg[AHRSID][1] + qz[AHRSID]*vg[AHRSID][2];
	//vg[2] = innerZ;
	
	if((mx[AHRSID]!=0.0f)||(my[AHRSID]!=0.0f)||(mz[AHRSID]!=0.0f)) {
		float innerX;
		float innerY;
		float cross[3];
		
		innerX = mx[AHRSID]*vg[AHRSID][0] + my[AHRSID]*vg[AHRSID][1] + mz[AHRSID]*vg[AHRSID][2];
		cross[0] = my[AHRSID]*qz[AHRSID]-mz[AHRSID]*qy[AHRSID];
		cross[1] = mz[AHRSID]*qx[AHRSID]-mx[AHRSID]*qz[AHRSID];
		cross[2] = mx[AHRSID]*qy[AHRSID]-my[AHRSID]*qx[AHRSID];
		innerY = cross[0]*vg[AHRSID][0] + cross[1]*vg[AHRSID][1] + cross[2]*vg[AHRSID][2];
		
		vg[AHRSID][1] = innerY;
		vg[AHRSID][0] = innerX;
	}
	
	/*if(CheckAccSteady(innerZ)) {
		vg[0] = 0;
		vg[1] = 0;
		vg[2] = 0;
	}*/
	
	A[0] = (vg[AHRSID][0])*9.8f;
	A[1] = (vg[AHRSID][1])*9.8f;
	A[2] = (vg[AHRSID][2])*9.8f;
	
	Vdiff[AHRSID][0] = (A[0]+Alast[AHRSID][0])/2*dt;
	Vdiff[AHRSID][1] = (A[1]+Alast[AHRSID][1])/2*dt;
	Vdiff[AHRSID][2] = (A[2]+Alast[AHRSID][2])/2*dt;

	Ve[AHRSID][0] = V0[AHRSID][0] + Vdiff[AHRSID][0];
	Ve[AHRSID][1] = V0[AHRSID][1] + Vdiff[AHRSID][1];
	Ve[AHRSID][2] = V0[AHRSID][2] + Vdiff[AHRSID][2];
	
	motion[AHRSID][0] = motionlast[AHRSID][0] + Alast[AHRSID][0]*dt*dt/2*100 + V0[AHRSID][0]*dt*100;
	motion[AHRSID][1] = motionlast[AHRSID][1] + Alast[AHRSID][1]*dt*dt/2*100 + V0[AHRSID][1]*dt*100;
	motion[AHRSID][2] = motionlast[AHRSID][2] + Alast[AHRSID][2]*dt*dt/2*100 + V0[AHRSID][2]*dt*100;
	
	Alast[AHRSID][0] = A[0];
	Alast[AHRSID][1] = A[1];
	Alast[AHRSID][2] = A[2];
	
	V0[AHRSID][0] = Ve[AHRSID][0];
	V0[AHRSID][1] = Ve[AHRSID][1];
	V0[AHRSID][2] = Ve[AHRSID][2];
		
	motionlast[AHRSID][0] = motion[AHRSID][0];
	motionlast[AHRSID][1] = motion[AHRSID][1];
	motionlast[AHRSID][2] = motion[AHRSID][2];

	if(CheckAccSteady(innerZ)) {
		V0[AHRSID][0]*=SPEED_DEC_RATIO;
		V0[AHRSID][1]*=SPEED_DEC_RATIO;
		V0[AHRSID][2]*=SPEED_DEC_RATIO;
	}
#if 0
		Serial_write((char*)&dt, 4);
		Serial_write((char*)&motion[1], 4);
		Serial_write((char*)&acc_steady, 4);
		Serial_write((char*)&Ve[1], 4);
#endif	
#if 0
		Serial_write((char*)&dt, 4);
		Serial_write((char*)&motion[0], 4);
		Serial_write((char*)&acc_steady, 4);
		Serial_write((char*)&Ve[0], 4);
#endif	
#if 0
		Serial_write((char*)&dt, 4);
		Serial_write((char*)&motion[2], 4);
		Serial_write((char*)&acc_steady, 4);
		Serial_write((char*)&Ve[2], 4);
#endif	
#if 0
		Serial_write((char*)&dt, 4);
		Serial_write((char*)&vg[0], 4);
		Serial_write((char*)&vg[1], 4);
		Serial_write((char*)&vg[2], 4);
#endif	
#if 0
		Serial_write((char*)&dt, 4);
		Serial_write((char*)&vg_b[0], 4);
		Serial_write((char*)&vg_b[1], 4);
		Serial_write((char*)&vg_b[2], 4);
#endif
#if 0
		Serial_write((char*)&dt, 4);
		Serial_write((char*)&Ve[0], 4);
		Serial_write((char*)&Ve[1], 4);
		Serial_write((char*)&Ve[2], 4);
#endif
#if 0
		Serial_write((char*)&dt, 4);
		Serial_write((char*)&Vdiff[0], 4);
		Serial_write((char*)&Vdiff[1], 4);
		Serial_write((char*)&Vdiff[2], 4);
#endif
#if 0
		Serial_write((char*)&dt, 4);
		Serial_write((char*)&axf, 4);
		Serial_write((char*)&ayf, 4);
		Serial_write((char*)&azf, 4);
#endif
#if 0
		Serial_write((char*)&dt, 4);
		Serial_write((char*)&motion[0], 4);
		Serial_write((char*)&motion[1], 4);
		Serial_write((char*)&motion[2], 4);
#endif
	//printf("%f\n",diam_diff);
	//printf("%1.3f %1.3f %1.3f\n",Vdiff[0], Vdiff[1], Vdiff[2]);
	//printf("a:%1.3f %1.3f %1.3f\n",axf, ayf, azf);
	//printf("%1.5f %1.5f %1.5f\n",vg[0], vg[1], vg[2]);
	//printf("A:%1.3f %1.3f %1.3f\n",A[0], A[1], A[2]);
	//printf("motion:%1.3f %1.3f %1.3f\n",motion[0], motion[1], motion[2]);
	//printf("%1.3f %1.3f %1.3f\n",vg[2], Vdiff[2], Ve[2]);
}
void sensfusion6UpdateQ(float gxf, float gyf, float gzf, float axf, float ayf, float azf, float dt)
{
  float recipNorm;
  float halfvx, halfvy, halfvz;
  float halfex, halfey, halfez;
  float qa, qb, qc;
	float no_gyro_factor = 0.0f;

  gx[AHRSID] = gxf * M_PI / 180.0f;
  gy[AHRSID] = gyf * M_PI / 180.0f;
  gz[AHRSID] = gzf * M_PI / 180.0f;
	

	gx_deg[AHRSID] += gxf*dt;
	gy_deg[AHRSID] += gyf*dt;
	gz_deg[AHRSID] += gzf*dt;
	
  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((axf == 0.0f) && (ayf == 0.0f) && (azf == 0.0f)))
  {
		if(((gxf == 0.0f) && (gyf == 0.0f) && (gzf == 0.0f)))
			no_gyro_factor = 8.0f;
    // Normalise accelerometer measurement
    recipNorm = invSqrt(axf * axf + ayf * ayf + azf * azf);
    ax[AHRSID] = axf*recipNorm;
    ay[AHRSID] = ayf*recipNorm;
    az[AHRSID] = azf*recipNorm;
		
    // Estimated direction of gravity and vector perpendicular to magnetic flux
    halfvx = q1[AHRSID] * q3[AHRSID] - q0[AHRSID] * q2[AHRSID];
    halfvy = q0[AHRSID] * q1[AHRSID] + q2[AHRSID] * q3[AHRSID];
    halfvz = q0[AHRSID] * q0[AHRSID] - 0.5f + q3[AHRSID] * q3[AHRSID];

    // Error is sum of cross product between estimated and measured direction of gravity
    halfex = (ay[AHRSID] * halfvz - az[AHRSID] * halfvy);
    halfey = (az[AHRSID] * halfvx - ax[AHRSID] * halfvz);
    halfez = (ax[AHRSID] * halfvy - ay[AHRSID] * halfvx);

    // Compute and apply integral feedback if enabled
    if((twoKi > 0.0f)&&(GyroDynamicGetSteady()==false))
    { 
      integralFBx[AHRSID] += twoKi * halfex * dt;  // integral error scaled by Ki
      integralFBy[AHRSID] += twoKi * halfey * dt;
      integralFBz[AHRSID] += twoKi * halfez * dt;

      gx[AHRSID] += integralFBx[AHRSID];  // apply integral feedback
      gy[AHRSID] += integralFBy[AHRSID];
      gz[AHRSID] += integralFBz[AHRSID];
      //if((TimeInfo.tick_counter%10000)==0)
      //  printf("%f  %f  %f - %f\n",halfex,halfey,halfez,integralFBz[AHRSID]);
    }
    else
    {
      integralFBx[AHRSID] = 0.0f; // prevent integral windup
      integralFBy[AHRSID] = 0.0f;
      integralFBz[AHRSID] = 0.0f;
    }

    // Apply proportional feedback
    gx[AHRSID] += twoKp * halfex;
    gy[AHRSID] += twoKp * halfey;
    gz[AHRSID] += twoKp * halfez;
  }

  // Integrate rate of change of quaternion
  gx[AHRSID] *= ((0.5f + no_gyro_factor) * dt);   // pre-multiply common factors
  gy[AHRSID] *= ((0.5f + no_gyro_factor) * dt);
  gz[AHRSID] *= ((0.5f + no_gyro_factor) * dt);
  /*gx *= (0.9f * dt);   // pre-multiply common factors
  gy *= (0.9f * dt);
  gz *= (0.9f * dt);*/
  qa = q0[AHRSID];
  qb = q1[AHRSID];
  qc = q2[AHRSID];
  q0[AHRSID] += (-qb * gx[AHRSID] - qc * gy[AHRSID] - q3[AHRSID] * gz[AHRSID]);
  q1[AHRSID] += (qa * gx[AHRSID] + qc * gz[AHRSID] - q3[AHRSID] * gy[AHRSID]);
  q2[AHRSID] += (qa * gy[AHRSID] - qb * gz[AHRSID] + q3[AHRSID] * gx[AHRSID]);
  q3[AHRSID] += (qa * gz[AHRSID] + qb * gy[AHRSID] - qc * gx[AHRSID]);

  // Normalise quaternion
  if(nvtGetPerformanceOverAccuracy()) {
    recipNorm = invSqrt(q0[AHRSID] * q0[AHRSID] + q1[AHRSID] * q1[AHRSID] + q2[AHRSID] * q2[AHRSID] + q3[AHRSID] * q3[AHRSID]);
    q0[AHRSID] *= recipNorm;
    q1[AHRSID] *= recipNorm;
    q2[AHRSID] *= recipNorm;
    q3[AHRSID] *= recipNorm;
  }
  else {
    recipNorm = sqrt(q0[AHRSID] * q0[AHRSID] + q1[AHRSID] * q1[AHRSID] + q2[AHRSID] * q2[AHRSID] + q3[AHRSID] * q3[AHRSID]);
    q0[AHRSID] /= recipNorm;
    q1[AHRSID] /= recipNorm;
    q2[AHRSID] /= recipNorm;
    q3[AHRSID] /= recipNorm;
  }
  ComputeEuler();
	ComputeMotion(axf, ayf, azf, recipNorm, dt);
}
bool checkMagFail(float mxf, float myf, float mzf)
{
	//float CurGuass;

	if((mxf == 0.0f) && (myf == 0.0f) && (mzf == 0.0f)) {
		//printf("checkMagFail 0\n");
		return true;
	}
	
	/*CurGuass = sqrt(mxf*mxf+myf*myf+mzf*mzf);
	if (fabs(GetMagGuass()- CurGuass)/GetMagGuass()>GUASS_ERROR_TH) {
		printf("checkMagFail 1\n");
		return true;
	}
	
	if((mzf>mxf)&&(mzf>myf)) {
		printf("checkMagFail 2\n");
		return true;
	}*/
	
	return false;
	
}
bool checkAccFail(float axf, float ayf, float azf)
{
	if((axf == 0.0f) && (ayf == 0.0f) && (azf == 0.0f)) {
		return true;
	}
	
	return false;
	
}
void sensfusion9UpdateQ(float gxf, float gyf, float gzf, float axf, float ayf, float azf, float mxf, float myf, float mzf, float dt) 
{
	float recipNorm;
  float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;  
	float hx, hy, bx, bz;
	float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
	float halfex, halfey, halfez;
	float qa, qb, qc;
	float no_gyro_factor = 0.0f;

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if(checkMagFail(mxf, myf, mzf)) {
		sensfusion6UpdateQ(gxf, gyf, gzf, axf, ayf, azf, dt);
		return;
	}
  else if(checkAccFail(axf, ayf, azf)) {
		sensfusionMagUpdateQ(0.0f,0.0f,0.0f,0.0f,0.0f,1.0f,mxf,myf,mzf,dt);
		return;
	}

	if(((gxf == 0.0f) && (gyf == 0.0f) && (gzf == 0.0f)))
		no_gyro_factor = 8.0f;

	gx[AHRSID] = gxf * M_PI / 180.0f;
  gy[AHRSID] = gyf * M_PI / 180.0f;
  gz[AHRSID] = gzf * M_PI / 180.0f;
	
	gx_deg[AHRSID] += gxf*dt;
	gy_deg[AHRSID] += gyf*dt;
	gz_deg[AHRSID] += gzf*dt;
	
	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((axf == 0.0f) && (ayf == 0.0f) && (azf == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(axf * axf + ayf * ayf + azf * azf);
		ax[AHRSID] = axf*recipNorm;
		ay[AHRSID] = ayf*recipNorm;
		az[AHRSID] = azf*recipNorm;     
		
		// Normalise magnetometer measurement
		recipNorm = invSqrt(mxf * mxf + myf * myf + mzf * mzf);
		mx[AHRSID] = mag_norm[AHRSID][0] = mxf*recipNorm;
		my[AHRSID] = mag_norm[AHRSID][1] = myf*recipNorm;
		mz[AHRSID] = mag_norm[AHRSID][2] = mzf*recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    q0q0 = q0[AHRSID] * q0[AHRSID];
    q0q1 = q0[AHRSID] * q1[AHRSID];
    q0q2 = q0[AHRSID] * q2[AHRSID];
    q0q3 = q0[AHRSID] * q3[AHRSID];
    q1q1 = q1[AHRSID] * q1[AHRSID];
    q1q2 = q1[AHRSID] * q2[AHRSID];
    q1q3 = q1[AHRSID] * q3[AHRSID];
    q2q2 = q2[AHRSID] * q2[AHRSID];
    q2q3 = q2[AHRSID] * q3[AHRSID];
    q3q3 = q3[AHRSID] * q3[AHRSID];   

    // Reference direction of Earth's magnetic field
    hx = 2.0f * (mx[AHRSID] * (0.5f - q2q2 - q3q3) + my[AHRSID] * (q1q2 - q0q3) + mz[AHRSID] * (q1q3 + q0q2));
    hy = 2.0f * (mx[AHRSID] * (q1q2 + q0q3) + my[AHRSID] * (0.5f - q1q1 - q3q3) + mz[AHRSID] * (q2q3 - q0q1));
    bx = sqrt(hx * hx + hy * hy);
    bz = 2.0f * (mx[AHRSID] * (q1q3 - q0q2) + my[AHRSID] * (q2q3 + q0q1) + mz[AHRSID] * (0.5f - q1q1 - q2q2));

		// Estimated direction of gravity and magnetic field
		halfvx = q1q3 - q0q2;
		halfvy = q0q1 + q2q3;
		halfvz = q0q0 - 0.5f + q3q3;
    halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
    halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
    halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);  
	
		// Error is sum of cross product between estimated direction and measured direction of field vectors
		halfex = ((ay[AHRSID] * halfvz - az[AHRSID] * halfvy)*2 + (my[AHRSID] * halfwz - mz[AHRSID] * halfwy)*0);
		halfey = ((az[AHRSID] * halfvx - ax[AHRSID] * halfvz)*2 + (mz[AHRSID] * halfwx - mx[AHRSID] * halfwz)*0);
		halfez = (ax[AHRSID] * halfvy - ay[AHRSID] * halfvx) + (mx[AHRSID] * halfwy - my[AHRSID] * halfwx);
		// Compute and apply integral feedback if enabled
		if((twoKi > 0.0f)&&(GyroDynamicGetSteady()==false)) {
      
			integralFBx[AHRSID] += twoKi * halfex * dt;	// integral error scaled by Ki
			integralFBy[AHRSID] += twoKi * halfey * dt;
			integralFBz[AHRSID] += twoKi * halfez * dt;
		
			gx[AHRSID] += integralFBx[AHRSID];	// apply integral feedback
			gy[AHRSID] += integralFBy[AHRSID];
			gz[AHRSID] += integralFBz[AHRSID];
		}
		else {
			integralFBx[AHRSID] = 0.0f;	// prevent integral windup
			integralFBy[AHRSID] = 0.0f;
			integralFBz[AHRSID] = 0.0f;
		}

		// Apply proportional feedback
		gx[AHRSID] += twoKp * halfex;
		gy[AHRSID] += twoKp * halfey;
		gz[AHRSID] += twoKp * halfez;
	}
	
	// Integrate rate of change of quaternion
	
	if(GetTickCounter()>MagMasterTime[AHRSID]) {
		gx[AHRSID] *= ((0.5f + no_gyro_factor) * dt);		// pre-multiply common factors
		gy[AHRSID] *= ((0.5f + no_gyro_factor) * dt);
		gz[AHRSID] *= ((0.5f + no_gyro_factor) * dt);
	}
	qa = q0[AHRSID];
	qb = q1[AHRSID];
	qc = q2[AHRSID];
	q0[AHRSID] += (-qb * gx[AHRSID] - qc * gy[AHRSID] - q3[AHRSID] * gz[AHRSID]);
	q1[AHRSID] += (qa * gx[AHRSID] + qc * gz[AHRSID] - q3[AHRSID] * gy[AHRSID]);
	q2[AHRSID] += (qa * gy[AHRSID] - qb * gz[AHRSID] + q3[AHRSID] * gx[AHRSID]);
	q3[AHRSID] += (qa * gz[AHRSID] + qb * gy[AHRSID] - qc * gx[AHRSID]); 
	
	// Normalise quaternion
  if(nvtGetPerformanceOverAccuracy()) {
    recipNorm = invSqrt(q0[AHRSID] * q0[AHRSID] + q1[AHRSID] * q1[AHRSID] + q2[AHRSID] * q2[AHRSID] + q3[AHRSID] * q3[AHRSID]);
    q0[AHRSID] *= recipNorm;
    q1[AHRSID] *= recipNorm;
    q2[AHRSID] *= recipNorm;
    q3[AHRSID] *= recipNorm;
  }
  else {
    recipNorm = sqrt(q0[AHRSID] * q0[AHRSID] + q1[AHRSID] * q1[AHRSID] + q2[AHRSID] * q2[AHRSID] + q3[AHRSID] * q3[AHRSID]);
    q0[AHRSID] /= recipNorm;
    q1[AHRSID] /= recipNorm;
    q2[AHRSID] /= recipNorm;
    q3[AHRSID] /= recipNorm;
  }
	ComputeEuler();
  ComputeMotion(axf, ayf, azf, recipNorm, dt);
}
void sensfusionMagUpdateQ(float gxf, float gyf, float gzf, float axf, float ayf, float azf, float mxf, float myf, float mzf, float dt) 
{
	float recipNorm;
  float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;  
	float hx, hy, bx, bz;
	float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
	float halfex, halfey, halfez;
	float qa, qb, qc;
	float no_gyro_factor = 0.0f;

	if(((gxf == 0.0f) && (gyf == 0.0f) && (gzf == 0.0f)))
		no_gyro_factor = 8.0f;

	gx[AHRSID] = gxf * M_PI / 180.0f;
  gy[AHRSID] = gyf * M_PI / 180.0f;
  gz[AHRSID] = gzf * M_PI / 180.0f;
	
	gx_deg[AHRSID] += gxf*dt;
	gy_deg[AHRSID] += gyf*dt;
	gz_deg[AHRSID] += gzf*dt;
	
  azf = 1.0f;
	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((axf == 0.0f) && (ayf == 0.0f) && (azf == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(axf * axf + ayf * ayf + azf * azf);
		ax[AHRSID] = axf*recipNorm;
		ay[AHRSID] = ayf*recipNorm;
		az[AHRSID] = azf*recipNorm;     
		
		// Normalise magnetometer measurement
		recipNorm = invSqrt(mxf * mxf + myf * myf + mzf * mzf);
		mx[AHRSID] = mag_norm[AHRSID][0] = mxf*recipNorm;
		my[AHRSID] = mag_norm[AHRSID][1] = myf*recipNorm;
		mz[AHRSID] = mag_norm[AHRSID][2] = mzf*recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    q0q0 = q0[AHRSID] * q0[AHRSID];
    q0q1 = q0[AHRSID] * q1[AHRSID];
    q0q2 = q0[AHRSID] * q2[AHRSID];
    q0q3 = q0[AHRSID] * q3[AHRSID];
    q1q1 = q1[AHRSID] * q1[AHRSID];
    q1q2 = q1[AHRSID] * q2[AHRSID];
    q1q3 = q1[AHRSID] * q3[AHRSID];
    q2q2 = q2[AHRSID] * q2[AHRSID];
    q2q3 = q2[AHRSID] * q3[AHRSID];
    q3q3 = q3[AHRSID] * q3[AHRSID];   

    // Reference direction of Earth's magnetic field
    hx = 2.0f * (mx[AHRSID] * (0.5f - q2q2 - q3q3) + my[AHRSID] * (q1q2 - q0q3) + mz[AHRSID] * (q1q3 + q0q2));
    hy = 2.0f * (mx[AHRSID] * (q1q2 + q0q3) + my[AHRSID] * (0.5f - q1q1 - q3q3) + mz[AHRSID] * (q2q3 - q0q1));
    bx = sqrt(hx * hx + hy * hy);
    bz = 2.0f * (mx[AHRSID] * (q1q3 - q0q2) + my[AHRSID] * (q2q3 + q0q1) + mz[AHRSID] * (0.5f - q1q1 - q2q2));

		// Estimated direction of gravity and magnetic field
		halfvx = q1q3 - q0q2;
		halfvy = q0q1 + q2q3;
		halfvz = q0q0 - 0.5f + q3q3;
    halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
    halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
    halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);  
	
		// Error is sum of cross product between estimated direction and measured direction of field vectors
		halfex = ((ay[AHRSID] * halfvz - az[AHRSID] * halfvy)*2 + (my[AHRSID] * halfwz - mz[AHRSID] * halfwy)*2);
		halfey = ((az[AHRSID] * halfvx - ax[AHRSID] * halfvz)*2 + (mz[AHRSID] * halfwx - mx[AHRSID] * halfwz)*2);
		halfez = (ax[AHRSID] * halfvy - ay[AHRSID] * halfvx) + (mx[AHRSID] * halfwy - my[AHRSID] * halfwx);
		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx[AHRSID] += twoKi * halfex * dt;	// integral error scaled by Ki
			integralFBy[AHRSID] += twoKi * halfey * dt;
			integralFBz[AHRSID] += twoKi * halfez * dt;
			gx[AHRSID] += integralFBx[AHRSID];	// apply integral feedback
			gy[AHRSID] += integralFBy[AHRSID];
			gz[AHRSID] += integralFBz[AHRSID];
		}
		else {
			integralFBx[AHRSID] = 0.0f;	// prevent integral windup
			integralFBy[AHRSID] = 0.0f;
			integralFBz[AHRSID] = 0.0f;
		}

		// Apply proportional feedback
		gx[AHRSID] += twoKp * halfex;
		gy[AHRSID] += twoKp * halfey;
		gz[AHRSID] += twoKp * halfez;
	}
	
	// Integrate rate of change of quaternion
	
	if(GetTickCounter()>MagMasterTime[AHRSID]) {
		gx[AHRSID] *= ((0.5f + no_gyro_factor) * dt);		// pre-multiply common factors
		gy[AHRSID] *= ((0.5f + no_gyro_factor) * dt);
		gz[AHRSID] *= ((0.5f + no_gyro_factor) * dt);
	}
	qa = q0[AHRSID];
	qb = q1[AHRSID];
	qc = q2[AHRSID];
	q0[AHRSID] += (-qb * gx[AHRSID] - qc * gy[AHRSID] - q3[AHRSID] * gz[AHRSID]);
	q1[AHRSID] += (qa * gx[AHRSID] + qc * gz[AHRSID] - q3[AHRSID] * gy[AHRSID]);
	q2[AHRSID] += (qa * gy[AHRSID] - qb * gz[AHRSID] + q3[AHRSID] * gx[AHRSID]);
	q3[AHRSID] += (qa * gz[AHRSID] + qb * gy[AHRSID] - qc * gx[AHRSID]); 
	
	// Normalise quaternion
	if(nvtGetPerformanceOverAccuracy()) {
    recipNorm = invSqrt(q0[AHRSID] * q0[AHRSID] + q1[AHRSID] * q1[AHRSID] + q2[AHRSID] * q2[AHRSID] + q3[AHRSID] * q3[AHRSID]);
    q0[AHRSID] *= recipNorm;
    q1[AHRSID] *= recipNorm;
    q2[AHRSID] *= recipNorm;
    q3[AHRSID] *= recipNorm;
  }
  else {
    recipNorm = sqrt(q0[AHRSID] * q0[AHRSID] + q1[AHRSID] * q1[AHRSID] + q2[AHRSID] * q2[AHRSID] + q3[AHRSID] * q3[AHRSID]);
    q0[AHRSID] /= recipNorm;
    q1[AHRSID] /= recipNorm;
    q2[AHRSID] /= recipNorm;
    q3[AHRSID] /= recipNorm;
  }
	
	ComputeEuler();
  ComputeMotion(axf, ayf, azf, recipNorm, dt);
}
#if 0
void sensfusionMagUpdateQ(float mxf, float myf, float mzf, float dt) 
{
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;
	float no_gyro_factor = 0.0f;

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	sensfusion6UpdateQ(0,0,0,mxf, myf, mzf, dt);

	no_gyro_factor = 8.0f;

	gx[AHRSID] = 0;
  gy[AHRSID] = 0;
  gz[AHRSID] = 0;
	
	gx_deg[AHRSID] += 0;
	gy_deg[AHRSID] += 0;
	gz_deg[AHRSID] += 0;
	

  // Normalise magnetometer measurement
  recipNorm = invSqrt(mxf * mxf + myf * myf + mzf * mzf);
  mx[AHRSID] = mag_norm[AHRSID][0] = mxf*recipNorm;
  my[AHRSID] = mag_norm[AHRSID][1] = myf*recipNorm;
  mz[AHRSID] = mag_norm[AHRSID][2] = mzf*recipNorm;

  // Estimated direction of gravity and vector perpendicular to magnetic flux
  halfvx = q1[AHRSID] * q3[AHRSID] - q0[AHRSID] * q2[AHRSID];
  halfvy = q0[AHRSID] * q1[AHRSID] + q2[AHRSID] * q3[AHRSID];
  halfvz = q0[AHRSID] * q0[AHRSID] - 0.5f + q3[AHRSID] * q3[AHRSID];

  // Error is sum of cross product between estimated and measured direction of gravity
  halfex = (my[AHRSID] * halfvz - mz[AHRSID] * halfvy);
  halfey = (mz[AHRSID] * halfvx - mx[AHRSID] * halfvz);
  halfez = (mx[AHRSID] * halfvy - my[AHRSID] * halfvx);
    
		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx[AHRSID] += twoKi * halfex * dt;	// integral error scaled by Ki
			integralFBy[AHRSID] += twoKi * halfey * dt;
			integralFBz[AHRSID] += twoKi * halfez * dt;
			gx[AHRSID] += integralFBx[AHRSID];	// apply integral feedback
			gy[AHRSID] += integralFBy[AHRSID];
			gz[AHRSID] += integralFBz[AHRSID];
		}
		else {
			integralFBx[AHRSID] = 0.0f;	// prevent integral windup
			integralFBy[AHRSID] = 0.0f;
			integralFBz[AHRSID] = 0.0f;
		}

		// Apply proportional feedback
		gx[AHRSID] += twoKp * halfex;
		gy[AHRSID] += twoKp * halfey;
		gz[AHRSID] += twoKp * halfez;

	
	// Integrate rate of change of quaternion
	
	if(GetTickCounter()>MagMasterTime[AHRSID]) {
		gx[AHRSID] *= ((0.5f + no_gyro_factor) * dt);		// pre-multiply common factors
		gy[AHRSID] *= ((0.5f + no_gyro_factor) * dt);
		gz[AHRSID] *= ((0.5f + no_gyro_factor) * dt);
	}
	qa = q0[AHRSID];
	qb = q1[AHRSID];
	qc = q2[AHRSID];
	q0[AHRSID] += (-qb * gx[AHRSID] - qc * gy[AHRSID] - q3[AHRSID] * gz[AHRSID]);
	q1[AHRSID] += (qa * gx[AHRSID] + qc * gz[AHRSID] - q3[AHRSID] * gy[AHRSID]);
	q2[AHRSID] += (qa * gy[AHRSID] - qb * gz[AHRSID] + q3[AHRSID] * gx[AHRSID]);
	q3[AHRSID] += (qa * gz[AHRSID] + qb * gy[AHRSID] - qc * gx[AHRSID]); 
	
	// Normalise quaternion
	recipNorm = invSqrt(q0[AHRSID] * q0[AHRSID] + q1[AHRSID] * q1[AHRSID] + q2[AHRSID] * q2[AHRSID] + q3[AHRSID] * q3[AHRSID]);
	q0[AHRSID] *= recipNorm;
	q1[AHRSID] *= recipNorm;
	q2[AHRSID] *= recipNorm;
	q3[AHRSID] *= recipNorm;
	
	ComputeEuler();
  ComputeMotion(mxf, myf, mzf, recipNorm, dt);
}
#endif
void sensfusion6GetEulerRPY(float* roll, float* pitch, float* yaw)
{
  *yaw = EulerYd[AHRSID];
  *pitch = EulerPd[AHRSID]; //Pitch seems to be inverted
  *roll = EulerRd[AHRSID];
}
float sensfusion6GetAccZWithoutGravity(const float ax, const float ay, const float az)
{
  // return vertical acceleration without gravity
  // (A dot G) / |G| - 1G (|G| = 1) -> (A dot G) - 1G
  return ((ax*qx[AHRSID] + ay*qy[AHRSID] + az*qz[AHRSID]) - 1.0f);
}
void sensfusion6GetGyro(float* gyroOut)
{
	gyroOut[0] = gx[AHRSID];
	gyroOut[1] = gy[AHRSID];
	gyroOut[2] = gz[AHRSID];
}
void sensfusion6GetAcc(float* accOut)
{
	accOut[0] = ax[AHRSID];
	accOut[1] = ay[AHRSID];
	accOut[2] = az[AHRSID];
}
void sensfusion6Getquaternion(Axis4f* Quaternion)
{
	Quaternion->w = q0[AHRSID];
	Quaternion->x = q1[AHRSID];
	Quaternion->y = q2[AHRSID];
	Quaternion->z = q3[AHRSID];
}
void sensfusion6UpdateMagByEuler()
{
	float Inclination;
	// Normalise mag

	Inclination = 0;//18.5*M_PI/180;
	//mx = mag_norm.x*cos(EulerP) + mag_norm.z*sin(EulerP);
	//my = mag_norm.y*cos(EulerR) + mag_norm.x*sin(EulerP+Inclination)*sin(EulerR) - mag_norm.z*cos(EulerP+Inclination)*sin(EulerR);
	my[AHRSID] = mag_norm[AHRSID][1]*cos(EulerR[AHRSID]) - mag_norm[AHRSID][2]*sin(EulerR[AHRSID]);
	mx[AHRSID] = mag_norm[AHRSID][0]*cos(EulerP[AHRSID]+Inclination) + mag_norm[AHRSID][1]*sin(EulerP[AHRSID])*sin(EulerR[AHRSID]) + mag_norm[AHRSID][2]*cos(EulerR[AHRSID])*sin(EulerP[AHRSID]+Inclination);
	//mz = mag_norm.z;
	mz[AHRSID] = -mag_norm[AHRSID][0]*sin(EulerP[AHRSID]+Inclination) + mag_norm[AHRSID][1]*cos(EulerP[AHRSID]+Inclination)*sin(EulerR[AHRSID]) + mag_norm[AHRSID][2]*cos(EulerP[AHRSID]+Inclination)*cos(EulerR[AHRSID]);
}
void caculate_mag_degree()
{
  float xHeading; 
  float yHeading;
  float zHeading;
	xHeading = atan2(-my[AHRSID], mx[AHRSID]); 
  yHeading = atan2(mz[AHRSID], mx[AHRSID]); 
  zHeading = atan2(-my[AHRSID], mz[AHRSID]);//atan2(mz, my); 
  if(xHeading < 0) xHeading += 2*M_PI; 
  if(xHeading > 2*M_PI) xHeading -= 2*M_PI; 
  if(yHeading < 0) yHeading += 2*M_PI; 
  if(yHeading > 2*M_PI) yHeading -= 2*M_PI; 
  if(zHeading < 0) zHeading += 2*M_PI; 
  if(zHeading > 2*M_PI) zHeading -= 2*M_PI; 

#ifdef MG_LPF_FACTOR
	mdfx[AHRSID] = (mdfx[AHRSID]*(MG_LPF_FACTOR-1) + (int)((xHeading * 180/M_PI) - mdx[AHRSID]))/MG_LPF_FACTOR; 
	mdfy[AHRSID] = (mdfy[AHRSID]*(MG_LPF_FACTOR-1) + (int)((yHeading * 180/M_PI) - mdy[AHRSID]))/MG_LPF_FACTOR; 
	mdfz[AHRSID] = (mdfz[AHRSID]*(MG_LPF_FACTOR-1) + (int)((zHeading * 180/M_PI) - mdz[AHRSID]))/MG_LPF_FACTOR; 
	mdx[AHRSID] = mdx[AHRSID] + mdfx[AHRSID];
	mdy[AHRSID] = mdy[AHRSID] + mdfy[AHRSID];
	mdz[AHRSID] = mdz[AHRSID] + mdfz[AHRSID];
#else
	mdx[AHRSID] = xHeading * 180/M_PI; 
  mdy[AHRSID] = yHeading * 180/M_PI; 
  mdz[AHRSID] = zHeading * 180/M_PI; 		
#endif
	
	
}
void sensfusion6GetMagEuler(float* magOut)
{
	magOut[0] = mx[AHRSID];
	magOut[1] = my[AHRSID];
	magOut[2] = mz[AHRSID];
}
void sensfusion6GetNormMag(float* magOut)
{
	magOut[0] = mag_norm[AHRSID][0];
	magOut[1] = mag_norm[AHRSID][1];
	magOut[2] = mag_norm[AHRSID][2];
}
void sensfusion6GetMagDegree(float* magdOut)
{
	caculate_mag_degree();
	magdOut[0] = mdx[AHRSID];
	magdOut[1] = mdy[AHRSID];
	magdOut[2] = mdz[AHRSID];
}
void sensfusion6GetGyroDeg(float* gyroDegout)
{
	gyroDegout[0] = gx_deg[AHRSID];
	gyroDegout[1] = gy_deg[AHRSID];
	gyroDegout[2] = gz_deg[AHRSID];
}
void sensfusion6GetVelocity(float* Velocity)
{
	Velocity[0] = Ve[0][AHRSID];
	Velocity[1] = Ve[1][AHRSID];
	Velocity[2] = Ve[2][AHRSID];
}
void sensfusion6GetMove(float* Move)
{
	Move[0] = motion[0][AHRSID];
	Move[1] = motion[1][AHRSID];
	Move[2] = motion[2][AHRSID];
}
void sensfusion6SetMove(float* Move)
{
	motion[AHRSID][0] = Move[0];
	motion[AHRSID][1] = Move[1]; 
	motion[AHRSID][2] = Move[2];
}
void sensfusion6ResetMove()
{
	motion[AHRSID][0] = 0;
	motion[AHRSID][1] = 0;
	motion[AHRSID][2] = 0;
}
//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
float invSqrt(float x)
{
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}
