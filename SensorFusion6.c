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
#include "Common.h"
#define TWO_KP_DEF  (2.0f * 0.4f) // 2 * proportional gain
#define TWO_KI_DEF  (2.0f * 0.001f) // 2 * integral gain
//#define TWO_KP_DEF	(2.0f * 0.4f)	// 2 * proportional gain
//#define TWO_KI_DEF	(2.0f * 0.001f)	// 2 * integral gain
#define GUASS_ERROR_TH 0.15f
#define ACC_TH 0.1f
#define VG_TH 0.01f
#define SPEED_DEC_RATIO 0.99f
#define MAG_MASTER_TIME 1000 /* ms */

float twoKp = TWO_KP_DEF;    // 2 * proportional gain (Kp)
float twoKi = TWO_KI_DEF;    // 2 * integral gain (Ki)
float integralFBx = 0.0f;
float integralFBy = 0.0f;
float integralFBz = 0.0f;  // integral error terms scaled by Ki

float gx = 0.0f;
float gy = 0.0f;
float gz = 0.0f;

float qx = 0.0f;
float qy = 0.0f;
float qz = 0.0f;

float gx_deg = 0.0f;
float gy_deg = 0.0f;
float gz_deg = 0.0f;

float ax = 0.0f;
float ay = 0.0f;
float az = 0.0f;

float EulerR = 0.0f;
float EulerP = 0.0f;
float EulerY = 0.0f;

float EulerRd = 0.0f;
float EulerPd = 0.0f;
float EulerYd = 0.0f;

float mx = 0.0f;
float my = 0.0f;
float mz = 0.0f;
float mag_norm[3];
float mdx = 0.0f;
float mdy = 0.0f;
float mdz = 0.0f;
float mdfx = 0.0f;
float mdfy = 0.0f;
float mdfz = 0.0f;

float q0 = 1.0f;
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;  // quaternion of sensor frame relative to auxiliary frame

static float motion[3] = {0, 0, 0};
static float motionlast[3] = {0, 0, 0};
float vg[3] = {0, 0, 0};
float Ve[3] = {0, 0, 0};
float V0[3] = {0, 0, 0};
float Vdiff[3] = {0, 0, 0};
float Alast[3] = {0, 0, 0};
float diam_diff;

float acc_steady=0;
int MagMasterTime = MAG_MASTER_TIME;

// TODO: Make math util file
static float invSqrt(float x);
void UpdateMagMasterTime()
{
	MagMasterTime = GetTickCounter() + MAG_MASTER_TIME;
	q0 = 1.0f;
	q1 = 0.0f;
	q2 = 0.0f;
	q3 = 0.0f;
}
void ComputeEuler()
{
	//Compute Euler
  qx = 2 * (q1*q3 - q0*q2);
  qy = 2 * (q0*q1 + q2*q3);
  qz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

  if (qx>1.0f) qx=1.0f;
  if (qx<-1.0f) qx=-1.0f;

  EulerY = atan2(2*(q0*q3 + q1*q2), q0*q0 + q1*q1 - q2*q2 - q3*q3);
  
	EulerR = atan2(qy, qz);
        //EulerP =atan2(-gx, gy*sin(EulerR)+gz*cos(EulerR));
	
	/* Since gy and gz are both very mall the atan2(gy,gz) has a great error, 
	and EullerP should not reference EulerR(let EulerR=0)*/
	//if((qy*qy+qz*qz)<0.15f)
		EulerP =atan(-qx/qz);//atan2(gx, gz);//asin(gx); //Pitch seems to be inverted
	//else
	//	EulerP =atan(-qx/(qy*sin(EulerR)+qz*cos(EulerR)));//atan2(gx, gz);//asin(gx); //Pitch seems to be inverted
	
  EulerYd = EulerY * 180 / M_PI;
  EulerPd = EulerP * 180 / M_PI; //Pitch seems to be inverted
  EulerRd = EulerR * 180 / M_PI;
}
	
void filterA(int axis)
{
	if((vg[axis]<=VG_TH)&&(vg[axis]>=-VG_TH)){
		vg[axis] = 0.0;
	}
}
bool CheckAccSteady(float innerZ)
{ 
	if(innerZ<0)
		innerZ = -innerZ;
	if(innerZ<ACC_TH) {
		acc_steady = 1;
		return true;
	}
	else {
		acc_steady = 0;
		return false;
	}
}
void ComputeMotion( float axf, float ayf, float azf, float recipNorm, float dt)
{
	float A[3], innerZ;

	vg[0] = axf - qx;
	vg[1] = ayf - qy;
	vg[2] = azf - qz;
	//printf("%f  %f  %f\n",vg[2], azf, qz);
	
	//filterA(0);
	//filterA(1);
	//filterA(2);
	
	innerZ = qx*vg[0] + qy*vg[1] + qz*vg[2];
	//vg[2] = innerZ;
	
	if((mx!=0.0f)||(my!=0.0f)||(mz!=0.0f)) {
		float innerX;
		float innerY;
		float cross[3];
		
	innerX = mx*vg[0] + my*vg[1] + mz*vg[2];
		cross[0] = my*qz-mz*qy;
		cross[1] = mz*qx-mx*qz;
		cross[2] = mx*qy-my*qx;
	innerY = cross[0]*vg[0] + cross[1]*vg[1] + cross[2]*vg[2];
	
	vg[1] = innerY;
	vg[0] = innerX;
	}
	
	/*if(CheckAccSteady(innerZ)) {
		vg[0] = 0;
		vg[1] = 0;
		vg[2] = 0;
	}*/
	
	A[0] = (vg[0])*9.8f;
	A[1] = (vg[1])*9.8f;
	A[2] = (vg[2])*9.8f;
	
	Vdiff[0] = (A[0]+Alast[0])/2*dt;
	Vdiff[1] = (A[1]+Alast[1])/2*dt;
	Vdiff[2] = (A[2]+Alast[2])/2*dt;

	Ve[0] = V0[0] + Vdiff[0];
	Ve[1] = V0[1] + Vdiff[1];
	Ve[2] = V0[2] + Vdiff[2];
	
	motion[0] = motionlast[0] + Alast[0]*dt*dt/2*100 + V0[0]*dt*100;
	motion[1] = motionlast[1] + Alast[1]*dt*dt/2*100 + V0[1]*dt*100;
	motion[2] = motionlast[2] + Alast[2]*dt*dt/2*100 + V0[2]*dt*100;
	
	Alast[0] = A[0];
	Alast[1] = A[1];
	Alast[2] = A[2];
	
	V0[0] = Ve[0];
	V0[1] = Ve[1];
	V0[2] = Ve[2];
		
	motionlast[0] = motion[0];
	motionlast[1] = motion[1];
	motionlast[2] = motion[2];

	if(CheckAccSteady(innerZ)) {
		V0[0]*=SPEED_DEC_RATIO;
		V0[1]*=SPEED_DEC_RATIO;
		V0[2]*=SPEED_DEC_RATIO;
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

  gx = gxf * M_PI / 180.0f;
  gy = gyf * M_PI / 180.0f;
  gz = gzf * M_PI / 180.0f;
	

	gx_deg += gxf*dt;
	gy_deg += gyf*dt;
	gz_deg += gzf*dt;
	
  // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
  if(!((axf == 0.0f) && (ayf == 0.0f) && (azf == 0.0f)))
  {
		if(((gxf == 0.0f) && (gyf == 0.0f) && (gzf == 0.0f)))
			no_gyro_factor = 8.0f;
    // Normalise accelerometer measurement
    recipNorm = invSqrt(axf * axf + ayf * ayf + azf * azf);
    ax = axf*recipNorm;
    ay = ayf*recipNorm;
    az = azf*recipNorm;

    // Estimated direction of gravity and vector perpendicular to magnetic flux
    halfvx = q1 * q3 - q0 * q2;
    halfvy = q0 * q1 + q2 * q3;
    halfvz = q0 * q0 - 0.5f + q3 * q3;

    // Error is sum of cross product between estimated and measured direction of gravity
    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);

    // Compute and apply integral feedback if enabled
    if(twoKi > 0.0f)
    {
      integralFBx += twoKi * halfex * dt;  // integral error scaled by Ki
      integralFBy += twoKi * halfey * dt;
      integralFBz += twoKi * halfez * dt;
      gx += integralFBx;  // apply integral feedback
      gy += integralFBy;
      gz += integralFBz;
    }
    else
    {
      integralFBx = 0.0f; // prevent integral windup
      integralFBy = 0.0f;
      integralFBz = 0.0f;
    }

    // Apply proportional feedback
    gx += twoKp * halfex;
    gy += twoKp * halfey;
    gz += twoKp * halfez;
  }

  // Integrate rate of change of quaternion
  gx *= ((0.5f + no_gyro_factor) * dt);   // pre-multiply common factors
  gy *= ((0.5f + no_gyro_factor) * dt);
  gz *= ((0.5f + no_gyro_factor) * dt);
  /*gx *= (0.9f * dt);   // pre-multiply common factors
  gy *= (0.9f * dt);
  gz *= (0.9f * dt);*/
  qa = q0;
  qb = q1;
  qc = q2;
  q0 += (-qb * gx - qc * gy - q3 * gz);
  q1 += (qa * gx + qc * gz - q3 * gy);
  q2 += (qa * gy - qb * gz + q3 * gx);
  q3 += (qa * gz + qb * gy - qc * gx);

  // Normalise quaternion
  recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
	
  
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
	if(((gxf == 0.0f) && (gyf == 0.0f) && (gzf == 0.0f)))
		no_gyro_factor = 8.0f;

	gx = gxf * M_PI / 180.0f;
  gy = gyf * M_PI / 180.0f;
  gz = gzf * M_PI / 180.0f;
	
	gx_deg += gxf*dt;
	gy_deg += gyf*dt;
	gz_deg += gzf*dt;
	
	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((axf == 0.0f) && (ayf == 0.0f) && (azf == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(axf * axf + ayf * ayf + azf * azf);
		ax = axf*recipNorm;
		ay = ayf*recipNorm;
		az = azf*recipNorm;     

		// Normalise magnetometer measurement
		recipNorm = invSqrt(mxf * mxf + myf * myf + mzf * mzf);
		mx = mag_norm[0] = mxf*recipNorm;
		my = mag_norm[1] = myf*recipNorm;
		mz = mag_norm[2] = mzf*recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    q0q0 = q0 * q0;
    q0q1 = q0 * q1;
    q0q2 = q0 * q2;
    q0q3 = q0 * q3;
    q1q1 = q1 * q1;
    q1q2 = q1 * q2;
    q1q3 = q1 * q3;
    q2q2 = q2 * q2;
    q2q3 = q2 * q3;
    q3q3 = q3 * q3;   

    // Reference direction of Earth's magnetic field
    hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
    hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
    bx = sqrt(hx * hx + hy * hy);
    bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

		// Estimated direction of gravity and magnetic field
		halfvx = q1q3 - q0q2;
		halfvy = q0q1 + q2q3;
		halfvz = q0q0 - 0.5f + q3q3;
    halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
    halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
    halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);  
	
		// Error is sum of cross product between estimated direction and measured direction of field vectors
		halfex = ((ay * halfvz - az * halfvy)*2 + (my * halfwz - mz * halfwy)*0);
		halfey = ((az * halfvx - ax * halfvz)*2 + (mz * halfwx - mx * halfwz)*0);
		halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);
		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * dt;	// integral error scaled by Ki
			integralFBy += twoKi * halfey * dt;
			integralFBz += twoKi * halfez * dt;
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}
	
	// Integrate rate of change of quaternion
	
	if(GetTickCounter()>MagMasterTime) {
	gx *= ((0.5f + no_gyro_factor) * dt);		// pre-multiply common factors
	gy *= ((0.5f + no_gyro_factor) * dt);
	gz *= ((0.5f + no_gyro_factor) * dt);
	}
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx); 
	
	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
	
	ComputeEuler();
  ComputeMotion(axf, ayf, azf, recipNorm, dt);
}

void sensfusion6GetEulerRPY(float* roll, float* pitch, float* yaw)
{
  *yaw = EulerYd;
  *pitch = EulerPd; //Pitch seems to be inverted
  *roll = EulerRd;
}
float sensfusion6GetAccZWithoutGravity(const float ax, const float ay, const float az)
{
  // return vertical acceleration without gravity
  // (A dot G) / |G| - 1G (|G| = 1) -> (A dot G) - 1G
  return ((ax*qx + ay*qy + az*qz) - 1.0f);
}
void sensfusion6GetGyro(float* gyroOut)
{
	gyroOut[0] = gx;
	gyroOut[1] = gy;
	gyroOut[2] = gz;
}
void sensfusion6GetAcc(float* accOut)
{
	accOut[0] = ax;
	accOut[1] = ay;
	accOut[2] = az;
}
void sensfusion6Getquaternion(Axis4f* Quaternion)
{
	Quaternion->w = q0;
	Quaternion->x = q1;
	Quaternion->y = q2;
	Quaternion->z = q3;
}
void sensfusion6UpdateMagByEuler()
{
	float Inclination;
	// Normalise mag

	Inclination = 0;//18.5*M_PI/180;
	//mx = mag_norm.x*cos(EulerP) + mag_norm.z*sin(EulerP);
	//my = mag_norm.y*cos(EulerR) + mag_norm.x*sin(EulerP+Inclination)*sin(EulerR) - mag_norm.z*cos(EulerP+Inclination)*sin(EulerR);
	my = mag_norm[1]*cos(EulerR) - mag_norm[2]*sin(EulerR);
	mx = mag_norm[0]*cos(EulerP+Inclination) + mag_norm[1]*sin(EulerP)*sin(EulerR) + mag_norm[2]*cos(EulerR)*sin(EulerP+Inclination);
	//mz = mag_norm.z;
	mz = -mag_norm[0]*sin(EulerP+Inclination) + mag_norm[1]*cos(EulerP+Inclination)*sin(EulerR) + mag_norm[2]*cos(EulerP+Inclination)*cos(EulerR);
}
void caculate_mag_degree()
{
  float xHeading; 
  float yHeading;
  float zHeading;
	xHeading = atan2(-my, mx); 
  yHeading = atan2(mz, mx); 
  zHeading = atan2(-my, mz);//atan2(mz, my); 
  if(xHeading < 0) xHeading += 2*M_PI; 
  if(xHeading > 2*M_PI) xHeading -= 2*M_PI; 
  if(yHeading < 0) yHeading += 2*M_PI; 
  if(yHeading > 2*M_PI) yHeading -= 2*M_PI; 
  if(zHeading < 0) zHeading += 2*M_PI; 
  if(zHeading > 2*M_PI) zHeading -= 2*M_PI; 

#ifdef MG_LPF_FACTOR
	mdfx = (mdfx*(MG_LPF_FACTOR-1) + (int)((xHeading * 180/M_PI) - mdx))/MG_LPF_FACTOR; 
	mdfy = (mdfy*(MG_LPF_FACTOR-1) + (int)((yHeading * 180/M_PI) - mdy))/MG_LPF_FACTOR; 
	mdfz = (mdfz*(MG_LPF_FACTOR-1) + (int)((zHeading * 180/M_PI) - mdz))/MG_LPF_FACTOR; 
	mdx = mdx + mdfx;
	mdy = mdy + mdfy;
	mdz = mdz + mdfz;
#else
	mdx = xHeading * 180/M_PI; 
  mdy = yHeading * 180/M_PI; 
  mdz = zHeading * 180/M_PI; 		
#endif
	
	
}
void sensfusion6GetMagEuler(float* magOut)
{
	magOut[0] = mx;
	magOut[1] = my;
	magOut[2] = mz;
}
void sensfusion6GetNormMag(float* magOut)
{
	magOut[0] = mag_norm[0];
	magOut[1] = mag_norm[1];
	magOut[2] = mag_norm[2];
}
void sensfusion6GetMagDegree(float* magdOut)
{
	caculate_mag_degree();
	magdOut[0] = mdx;
	magdOut[1] = mdy;
	magdOut[2] = mdz;
}
void sensfusion6GetGyroDeg(float* gyroDegout)
{
	gyroDegout[0] = gx_deg;
	gyroDegout[1] = gy_deg;
	gyroDegout[2] = gz_deg;
}
void sensfusion6GetVelocity(float* Velocity)
{
	Velocity[0] = Ve[0];
	Velocity[1] = Ve[1];
	Velocity[2] = Ve[2];
}
void sensfusion6GetMove(float* Move)
{
	Move[0] = motion[0];
	Move[1] = motion[1];
	Move[2] = motion[2];
}
void sensfusion6SetMove(float* Move)
{
	motion[0] = Move[0];
	motion[1] = Move[1]; 
	motion[2] = Move[2];
}
void sensfusion6ResetMove()
{
	motion[0] = 0;
	motion[1] = 0;
	motion[2] = 0;
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
