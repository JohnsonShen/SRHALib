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
#include <stdint.h>
#include <math.h>
#include "Common.h"
#include "Gauss_Newton_Calibrate.h"
#include "AHRSLib.h"
#define GUASS_ITERATIONS 60

static Ellipsoid_T Ellipsoid;

static float beta[MAX_BETA_SIZE];
static float beta_min[MAX_BETA_SIZE];																			
static float beta_mag[MAG_BETA_SIZE];  
static float beta_acc[ACC_BETA_SIZE]; 
//matrices for Gauss-Newton computations
static float JS[MAX_BETA_SIZE][MAX_BETA_SIZE];
static float dS[MAX_BETA_SIZE];
static float delta[MAX_BETA_SIZE];

void find_delta(int DeltaSize);
int8_t calibrate_model_acc(int16_t* data_acc, int sample_count);
void reset_calibration_matrices(void);

void setup_acc_beta()
{
  reset_calibration_matrices();
  
  //initialize beta to something reasonable
  beta[0] = beta[1] = beta[2] = 0.0f;
  beta[3] = beta[4] = beta[5] = 0.000244f; 
}

void setup_mag_beta()
{
  reset_calibration_matrices();
  
  //initialize beta to something reasonable
	/*beta[0] = Ellipsoid.mean[0];
	beta[1] = Ellipsoid.mean[1];
	beta[2] = Ellipsoid.mean[2];
  beta[3] = 0.0002;
	beta[4] = 0.0002;
	beta[5] = 0.0002;
  beta[6] = 0.000f;
	beta[7] = 0.000f;
	beta[8] = 0.000f;
	beta[9] = 1.0;*/
	
	beta[0] = Ellipsoid.mean[0];
	beta[1] = Ellipsoid.mean[1];
	beta[2] = Ellipsoid.mean[2];
  beta[3] = CalInfo[AHRSID].MagGaussPLSB;
	beta[4] = CalInfo[AHRSID].MagGaussPLSB;
	beta[5] = CalInfo[AHRSID].MagGaussPLSB;
  beta[6] = 0.000f;
	beta[7] = 0.000f;
	beta[8] = 0.000f;
	beta[9] = 1.0;

}

//Gauss-Newton functions
void reset_calibration_matrices() {
  int j,k;
    for(j=0;j<MAX_BETA_SIZE;++j) {
       dS[j] = 0.0;
       for(k=0;k<MAX_BETA_SIZE;++k) {
         JS[j][k] = 0.0;
       }
    }     
}

void update_calibration_matrices(signed short* data, float residual) {
    int j, k;
    float dx, b;
    float jacobian[6];
    
    for(j=0;j<3;++j) {
      b = beta[3+j];
      dx = ((float)data[j]) - beta[j];
      residual -= b*b*dx*dx;
      jacobian[j] = 2.0f*b*b*dx;
      jacobian[3+j] = -2.0f*b*dx*dx;
    }
    
    for(j=0;j<6;++j) {
       dS[j] += jacobian[j]*residual;
       for(k=0;k<6;++k) {
         JS[j][k] += jacobian[j]*jacobian[k];
       }
    }
}

void update_calibration_matrices_mag(signed short* data, float residual) 
{
    int i, j, k;
    float x[3],x_m_beta[MAG_BETA_SIZE], x_m_beta_2[MAG_BETA_SIZE],F_x;
    float jacobian[MAG_BETA_SIZE];
    
		x[0] = (float)data[0];
    x[1] = (float)data[1];
		x[2] = (float)data[2];
		
	for(i=0;i<3;i++) {
			x_m_beta[i] = x[i]-beta[i];
			x_m_beta_2[i] = x_m_beta[i]*x_m_beta[i];
		}
	
		F_x = x_m_beta_2[0]*beta[3] + x_m_beta_2[1]*beta[4] + x_m_beta_2[2]*beta[5] +
					2.0f*(x_m_beta[0]*x_m_beta[1]*beta[6] + x_m_beta[1]*x_m_beta[2]*beta[7] + x_m_beta[0]*x_m_beta[2]*beta[8]);
		F_x = F_x/(beta[9]*beta[9]);
		residual -= F_x;
		/* J0 */
		jacobian[0] = 2.0f*(x_m_beta[0]*beta[3] + x_m_beta[1]*beta[6] + x_m_beta[2]*beta[8])/(beta[9]*beta[9]);
		/* J1 */
		jacobian[1] = 2.0f*(x_m_beta[1]*beta[4] + x_m_beta[0]*beta[6] + x_m_beta[2]*beta[7])/(beta[9]*beta[9]);
		/* J2 */
		jacobian[2] = 2.0f*(x_m_beta[2]*beta[5] + x_m_beta[1]*beta[7] + x_m_beta[0]*beta[8])/(beta[9]*beta[9]);
		/* J3 */
		jacobian[3] = -x_m_beta_2[0]/(beta[9]*beta[9]);
		/* J4 */
		jacobian[4] = -x_m_beta_2[1]/(beta[9]*beta[9]);
		/* J5 */
		jacobian[5] = -x_m_beta_2[2]/(beta[9]*beta[9]);
		/* J6 */
		jacobian[6] = -2*x_m_beta[0]*x_m_beta[1]/(beta[9]*beta[9]);
		/* J7 */
		jacobian[7] = -2*x_m_beta[1]*x_m_beta[2]/(beta[9]*beta[9]);
		/* J8 */
		jacobian[8] = -2*x_m_beta[0]*x_m_beta[2]/(beta[9]*beta[9]);
		/* J9 */
		jacobian[9] = 2*F_x/beta[9];
	
    for(j=0;j<MAG_BETA_SIZE;++j) {
       dS[j] += jacobian[j]*residual;
       for(k=0;k<MAG_BETA_SIZE;++k) {
         JS[j][k] += jacobian[j]*jacobian[k];
       }
    }   
}

void compute_calibration_matrices_acc(signed short* data, int sample_count) {
    int i;
    reset_calibration_matrices();
    for(i=0;i<sample_count;i++) {    
      update_calibration_matrices(data+3*i, 1.0f);
    }
}
void compute_calibration_matrices_mag(signed short* data, int sample_count) {
    int i;

    reset_calibration_matrices();

    for(i=0;i<sample_count;i++) {    
      update_calibration_matrices_mag(data+3*i, 1.0f);
    }
}
float Det(float* Matrix)
{
	int i,j;
	float DeltaMx[3][3],Det;
	for(i=0;i<3;i++)
		for(j=0;j<3;j++)
			DeltaMx[i][j]=Matrix[i*3+j];
	
	
	// [00 01 02] 
	// [10 11 12] 
	// [20 21 22]  
	
	Det = DeltaMx[0][0]*(DeltaMx[1][1]*DeltaMx[2][2]- DeltaMx[1][2]*DeltaMx[2][1])
			- DeltaMx[0][1]*(DeltaMx[1][0]*DeltaMx[2][2]- DeltaMx[1][2]*DeltaMx[2][0])
			+ DeltaMx[0][2]*(DeltaMx[1][0]*DeltaMx[2][1]- DeltaMx[1][1]*DeltaMx[2][0]);
	return Det;
}
void Jacobi_Cyclic_Method(float eigenvalues[], float *eigenvectors,
                                                              float *A, int n)
{
   int i, j, k, m;
   float *pAk, *pAm, *p_r, *p_e;
   float threshold_norm;
   float threshold;
   float tan_phi, sin_phi, cos_phi, tan2_phi, sin2_phi, cos2_phi;
   float sin_2phi, cot_2phi;
   float dum1;
   float dum2;
   float dum3;
   float max;

   // Take care of trivial cases
   if ( n < 1) return;
   if ( n == 1) {
      eigenvalues[0] = *A;
      *eigenvectors = 1.0;
      return;
   }

   // Initialize the eigenvalues to the identity matrix.
   for (p_e = eigenvectors, i = 0; i < n; i++)
      for (j = 0; j < n; p_e++, j++)
         if (i == j) *p_e = 1.0; else *p_e = 0.0;
  
   // Calculate the threshold and threshold_norm.
   for (threshold = 0.0, pAk = A, i = 0; i < ( n - 1 ); pAk += n, i++) 
      for (j = i + 1; j < n; j++) threshold += *(pAk + j) * *(pAk + j);
   threshold = sqrt(threshold + threshold);
   threshold_norm = threshold * 2.2204460492503131E-16f;//DBL_EPSILON;
   max = threshold + 1.0f;
   while (threshold > threshold_norm) {
      threshold /= 10.0f;
      if (max < threshold) continue;
      max = 0.0;
      for (pAk = A, k = 0; k < (n-1); pAk += n, k++) {
         for (pAm = pAk + n, m = k + 1; m < n; pAm += n, m++) {
            if ( fabs(*(pAk + m)) < threshold ) continue;

            // Calculate the sin and cos of the rotation angle which
            // annihilates A[k][m].
            cot_2phi = 0.5f * ( *(pAk + k) - *(pAm + m) ) / *(pAk + m);
            dum1 = sqrt( cot_2phi * cot_2phi + 1.0f);
            if (cot_2phi < 0.0f) dum1 = -dum1;
            tan_phi = -cot_2phi + dum1;
            tan2_phi = tan_phi * tan_phi;
            sin2_phi = tan2_phi / (1.0f + tan2_phi);
            cos2_phi = 1.0f - sin2_phi;
            sin_phi = sqrt(sin2_phi);
            if (tan_phi < 0.0f) sin_phi = - sin_phi;
            cos_phi = sqrt(cos2_phi); 
            sin_2phi = 2.0f * sin_phi * cos_phi;
            //cos_2phi = cos2_phi - sin2_phi;

           // Rotate columns k and m for both the matrix A 
           //     and the matrix of eigenvectors.
            p_r = A;
            dum1 = *(pAk + k);
            dum2 = *(pAm + m);
            dum3 = *(pAk + m);
            *(pAk + k) = dum1 * cos2_phi + dum2 * sin2_phi + dum3 * sin_2phi;
            *(pAm + m) = dum1 * sin2_phi + dum2 * cos2_phi - dum3 * sin_2phi;
            *(pAk + m) = 0.0;
            *(pAm + k) = 0.0;
            for (i = 0; i < n; p_r += n, i++) {
               if ( (i == k) || (i == m) ) continue;
               if ( i < k ) dum1 = *(p_r + k); else dum1 = *(pAk + i);
               if ( i < m ) dum2 = *(p_r + m); else dum2 = *(pAm + i);
               dum3 = dum1 * cos_phi + dum2 * sin_phi;
               if ( i < k ) *(p_r + k) = dum3; else *(pAk + i) = dum3;
               dum3 = - dum1 * sin_phi + dum2 * cos_phi;
               if ( i < m ) *(p_r + m) = dum3; else *(pAm + i) = dum3;
            }
            for (p_e = eigenvectors, i = 0; i < n; p_e += n, i++) {
               dum1 = *(p_e + k);
               dum2 = *(p_e + m);
               *(p_e + k) = dum1 * cos_phi + dum2 * sin_phi;
               *(p_e + m) = - dum1 * sin_phi + dum2 * cos_phi;
            }
         }
         for (i = 0; i < n; i++)
            if ( i == k ) continue;
            else if ( max < fabs(*(pAk + i))) max = fabs(*(pAk + i));
      }
   }
   for (pAk = A, k = 0; k < n; pAk += n, k++) eigenvalues[k] = *(pAk + k); 
}
void compute_eigenvalue(float* data) 
{
	int i,j,k;

	float /*F_B[3],Delta,DeltaX[3]*/Root[3],V[3][3],VT[3][3],InvW[3][3],VTA[3][3];
	//F(B) = c3*B^3 + c2*B^2 + c1*B +c0 = 0 ,c3=1
	InvW[0][0] = data[3];//*100;
	InvW[0][1] = data[6];//*100;
	InvW[0][2] = data[8];//*100;
	InvW[1][0] = data[6];//*100;
	InvW[1][1] = data[4];//*100;
	InvW[1][2] = data[7];//*100;
	InvW[2][0] = data[8];//*100;
	InvW[2][1] = data[7];//*100;
	InvW[2][2] = data[5];//*100;
	for(i=0;i<3;i++) {
		DBG_PRINT(" A[ %f %f %f ]\n", InvW[i][0],InvW[i][1],InvW[i][2]);
	}
	/*c0 = (a00*a11*a22 + 2*a01*a02*a12 -a00*a12*a12 -a11*a02*a02 -a22*a01*a01);
	c1 = (a00*a11 - a01*a01 + a00*a22 -a02*a02 + a11*a22 -a12*a12);
	c2 = (a00 + a11 + a22);
	DBG_PRINT("c0=%f,c1=%f,c2=%f\n",c0,c1,c2);
	A = (c2*c1/6/c3-c2*c2*c2/27/c3/c3/c3 - c0/2/c3);
	AA=A*A;
	B = (c1/3/c3 - c2*c2/9/c3/c3);
	BBB=B*B*B;
	Judge = AA + BBB;
	
	DBG_PRINT("AA=%f,BBB=%f,Judge=%f\n",AA,BBB,Judge);
	
	Root[0] = c2/3/c3 + 2*sqrt(-B)*cos(acos(A/pow(-B,3/2))/3);
	Root[1] = c2/3/c3 + 2*sqrt(-B)*cos((acos(A/pow(-B,3/2))+2*M_PI)/3);
	Root[2] = c2/3/c3 + 2*sqrt(-B)*cos((acos(A/pow(-B,3/2))-2*M_PI)/3);

	DBG_PRINT("Root=%f %f %f\n", Root[0], Root[1], Root[2]);*/

	Jacobi_Cyclic_Method(&Root[0], &V[0][0], &InvW[0][0], 3);
#ifdef DBG
	{
		float RZ,RY;
		RZ=atan(V[0][1]/V[0][0])/M_PI*180;
		RY=atan(V[0][2]/sqrt(V[1][0]*V[1][0]+V[0][0]*V[0][0]))/M_PI*180;;
		for(i=0;i<3;i++) {
			DBG_PRINT(" Root:%f,V[ %f %f %f ],Deg[%f %f]\n", Root[i], V[i][0],V[i][1],V[i][2],RZ,RY);
		}
	}
#endif
	for(i=0;i<3;i++) {
		if(Root[i]<0)
			InvW[i][i]=-InvW[i][i];
	}

	//Root[0]=A/pow(-B,3/2);
	/*for(i=0;i<3;i++) {
		F_B[i] = Root[i]*Root[i]*Root[i] + c2*Root[i]*Root[i] + c1*Root[i] + c0;
		DBG_PRINT("Root[%d]=%f, F_B[%d]=%f\n", i, Root[i], i, F_B[i]);
	

		// [a3 a6 a8]   [x0]          [x0]
		// [a6 a4 a7] * [x1] = Root * [x1]
		// [a8 a7 a5]   [x2]          [x2]
		// -------------------------------
		// [(a3-Root)   a6       a8   ] [x0] = [0]
		// [   a6   (a4-Root)	   a7   ]*[x1] = [0]
		// [   a8       a7   (a5-Root)] [x2] = [0]
		// --------------------------------
		//             [(a3-Root)   a6       a8   ] 
		//Delta = det( [   a6   (a4-Root)	   a7   ])
		//             [   a8       a7   (a5-Root)] 
		
		DeltaMx[0][0] = data[3] - Root[i];
		DeltaMx[0][1] = data[6];
		DeltaMx[0][2] = data[8];
		DeltaMx[1][0] = data[6];
		DeltaMx[1][1] = data[4] - Root[i];
		DeltaMx[1][2] = data[7];
		DeltaMx[2][0] = data[8];
		DeltaMx[2][1] = data[7];
		DeltaMx[2][2] = data[5] - Root[i];
		Delta = Det(&DeltaMx[0][0]);
		//                [ 0    a6      a8    ] 
		//DeltaX[0] = det([ 0 (a4-Root)	 a7    ])
		//                [ 0    a7   (a5-Root)] 
		DeltaMx[0][0] = 0;
		DeltaMx[0][1] = data[6];
		DeltaMx[0][2] = data[8];
		DeltaMx[1][0] = 0;
		DeltaMx[1][1] = data[4] - Root[i];
		DeltaMx[1][2] = data[7];
		DeltaMx[2][0] = 0;
		DeltaMx[2][1] = data[7];
		DeltaMx[2][2] = data[5] - Root[i];
		DeltaX[0] = Det(&DeltaMx[0][0]);
		//                [(a3-Root)  0       a8   ] 
		//DeltaX[1] = det([   a6      0	      a7   ])
		//                [   a8      0   (a5-Root)] 
		DeltaMx[0][0] = data[3] - Root[i];
		DeltaMx[0][1] = 0;
		DeltaMx[0][2] = data[8];
		DeltaMx[1][0] = data[6];
		DeltaMx[1][1] = 0;
		DeltaMx[1][2] = data[7];
		DeltaMx[2][0] = data[8];
		DeltaMx[2][1] = 0;
		DeltaMx[2][2] = data[5] - Root[i];
		DeltaX[1] = Det(&DeltaMx[0][0]);
		//                 [(a3-Root)   a6       0 ] 
		//DeltaX[2] = det( [   a6   (a4-Root)  	 0 ])
		//                 [   a8       a7       0 ] 
		DeltaMx[0][0] = data[3] - Root[i];
		DeltaMx[0][1] = data[6];
		DeltaMx[0][2] = 0;
		DeltaMx[1][0] = data[6];
		DeltaMx[1][1] = data[4] - Root[i];
		DeltaMx[1][2] = 0;
		DeltaMx[2][0] = data[8];
		DeltaMx[2][1] = data[7];
		DeltaMx[2][2] = 0;
		DeltaX[2]= Det(&DeltaMx[0][0]);
		
		if(Delta!=0) {
			V[i][0] = DeltaX[0]/Delta;
			V[i][1] = DeltaX[1]/Delta;
			V[i][2] = DeltaX[2]/Delta;
			VT[0][i] = DeltaX[0]/Delta;
			VT[1][i] = DeltaX[1]/Delta;
			VT[2][i] = DeltaX[2]/Delta;
			DBG_PRINT("Delta:%f\n",Delta);
			DBG_PRINT("V(%f,%f,%f)\n",V[i][0],V[i][1],V[i][2]);
			DBG_PRINT("VT(%f,%f,%f)\n",VT[0][i],VT[1][i],VT[2][i]);
		}
		else {
			DBG_PRINT("Delta =0, Error\n");
		}
	}*/
	//A=V*D*VT
	//D=VT*A*V
	//SquareRoot(A) = V*SquareRoot(D)*VT;
	for(i=0;i<3;i++)
		for(j=0;j<3;j++)
			VT[i][j]=V[j][i];
	
	for(i=0;i<3;i++) {
		DBG_PRINT(" VT[ %f %f %f ]\n", VT[i][0],VT[i][1],VT[i][2]);
	}		
	
	for(i=0;i<3;i++)
		for(j=0;j<3;j++)
			VTA[i][j]=0;
	for(i=0;i<3;i++) {
		DBG_PRINT(" V[ %f %f %f ]\n", V[i][0],V[i][1],V[i][2]);
	}	
	InvW[0][0]=sqrt(InvW[0][0]);
	InvW[1][1]=sqrt(InvW[1][1]);
	InvW[2][2]=sqrt(InvW[2][2]);
	for(i=0;i<3;i++) {
		DBG_PRINT(" A[ %f %f %f ]\n", InvW[i][0],InvW[i][1],InvW[i][2]);
	}
	for(i=0;i<3;i++)
		for(j=0;j<3;j++)
			for(k=0;k<3;k++)
				VTA[i][j]+=V[i][k]*InvW[k][j];
	
	for(i=0;i<3;i++) {
		DBG_PRINT(" V*A[ %f %f %f ]\n", VTA[i][0],VTA[i][1],VTA[i][2]);
	}	
	for(i=0;i<3;i++)
		for(j=0;j<3;j++)
			InvW[i][j]=0;
	
	for(i=0;i<3;i++)
		for(j=0;j<3;j++)
			for(k=0;k<3;k++)
				InvW[i][j]+=VTA[i][k]*VT[k][j];
	
	for(i=0;i<3;i++) {
		DBG_PRINT(" InvW[ %f %f %f ]\n", InvW[i][0],InvW[i][1],InvW[i][2]);
	}
	
	 data[3]=InvW[0][0];
	 data[6]=InvW[0][1];
	 data[8]=InvW[0][2];
	 data[4]=InvW[1][1];
	 data[7]=InvW[1][2];
	 data[5]=InvW[2][2];
	
}
void find_delta(int DeltaSize) {
  //Solve 6-d matrix equation JS*x = dS
  //first put in upper triangular form
  int i,j,k;
  float mu;
  
  //make upper triangular
  for(i=0;i<DeltaSize;++i) {
    //eliminate all nonzero entries below JS[i][i]
    for(j=i+1;j<DeltaSize;++j) {
      mu = JS[i][j]/JS[i][i];
      if(mu != 0.0f) {
        dS[j] -= mu*dS[i];
        for(k=j;k<DeltaSize;++k) {
         JS[k][j] -= mu*JS[k][i];
        } 
      }
    }
  }
  
  //back-substitute
  for(i=DeltaSize-1;i>=0;--i) {
    dS[i] /= JS[i][i];
    JS[i][i] = 1.0;
    for(j=0;j<i;++j) {
      mu = JS[i][j];
      dS[j] -= mu*dS[i];
      JS[i][j] = 0.0;
    }
  }
  
  for(i=0;i<DeltaSize;++i) {
    delta[i] = dS[i];
  }
}
void VerifyBetaAcc(signed short* data,int sample_count)
{
    int j;
    float dx, b, F_x=0;
    
    for(j=0;j<3;++j) {
      b = beta[3+j];
      dx = ((float)data[j]) - beta[j];
      F_x+=b*b*dx*dx;
		}
		DBG_PRINT("sample %d:%f\n",sample_count,F_x);
}
void VerifyBetaMagCircle(signed short* data)
{
	/*float X_B0, Y_B1, Z_B2, B[MAG_BETA_SIZE];
	float p0[3],p1[3];
	int i;

	p0[0] = (float)data[0];
  p0[1] = (float)data[1];
	p0[2] = (float)data[2];
	
	for(i=0;i<MAG_BETA_SIZE;i++)
		B[i]=beta_mag[i];
	
	X_B0 = (p0[0] -  B[0]);
	Y_B1 = (p0[1] -  B[1]);
	Z_B2 = (p0[2] -  B[2]);
	
	p1[0] = B[3]*X_B0 + B[6]*Y_B1 + B[8]*Z_B2;
  p1[1] = B[6]*X_B0 + B[4]*Y_B1 + B[7]*Z_B2;
  p1[2] = B[8]*X_B0 + B[7]*Y_B1 + B[5]*Z_B2;
	
	DBG_PRINT("%f,%f,%f,%f,%f,%f\n", p0[0], p0[1], p0[2], p1[0],p1[1],p1[2]);*/
}
void VerifyBetaMag(signed short* data, int sample_count)
{
    int i;
    float x[3],x_m_beta[MAG_BETA_SIZE], x_m_beta_2[MAG_BETA_SIZE],F_x;
    
		x[0] = (float)data[0];
    x[1] = (float)data[1];
		x[2] = (float)data[2];
		
		for(i=0;i<3;i++) {
			x_m_beta[i] = x[i]-beta[i];
			x_m_beta_2[i] = x_m_beta[i]*x_m_beta[i];
		}
	
		F_x = (x_m_beta_2[0]*beta[3] + x_m_beta_2[1]*beta[4] + x_m_beta_2[2]*beta[5] +
					2.0f*(x_m_beta[0]*x_m_beta[1]*beta[6] + x_m_beta[1]*x_m_beta[2]*beta[7] + x_m_beta[0]*x_m_beta[2]*beta[8]));
		
		CalInfo[AHRSID].MagCalQuality+=(F_x- beta[9]*beta[9]);
		F_x = F_x/(beta[9]*beta[9]);
		DBG_PRINT("sample %d:%f\n",sample_count,F_x);
}
float VerifyBetaMagAvg(int16_t* data_mag, int sample_count)
{
    int i,k;
		signed short* data;
    float x[3],x_m_beta[MAG_BETA_SIZE], x_m_beta_2[MAG_BETA_SIZE],F_x,Sum_Fx=0;
    for(k=0;k<sample_count;++k) {
			data=data_mag+k*3;
			x[0] = (float)data[0];
			x[1] = (float)data[1];
			x[2] = (float)data[2];
			
			for(i=0;i<3;i++) {
				x_m_beta[i] = x[i]-beta[i];
				x_m_beta_2[i] = x_m_beta[i]*x_m_beta[i];
			}
		
			F_x = (x_m_beta_2[0]*beta[3] + x_m_beta_2[1]*beta[4] + x_m_beta_2[2]*beta[5] +
						2.0f*(x_m_beta[0]*x_m_beta[1]*beta[6] + x_m_beta[1]*x_m_beta[2]*beta[7] + x_m_beta[0]*x_m_beta[2]*beta[8]));
			F_x = F_x/(beta[9]*beta[9]);
			Sum_Fx+=F_x;
		}
		return Sum_Fx/sample_count;
}
int8_t calibrate_model_acc(int16_t* data_acc, int sample_count)
{
  int i,j;
	int8_t cal_status=STATUS_NORMAL;
  float eps = 0.000000001;
  int num_iterations = 20;
  float change = 100.0;
  while (--num_iterations >=0 && change > eps) {
    compute_calibration_matrices_acc(data_acc, sample_count);
    find_delta(ACC_BETA_SIZE);
    change = delta[0]*delta[0] + delta[1]*delta[1] + delta[2]*delta[2] + delta[3]*delta[3]/(beta_acc[3]*beta_acc[3]) + delta[4]*delta[4]/(beta_acc[4]*beta_acc[4]) + delta[5]*delta[5]/(beta_acc[5]*beta_acc[5]); 
		for(j=0;j<ACC_BETA_SIZE;j++) 
			DBG_PRINT("delta[%d]=%f ",j,delta[j]);
		DBG_PRINT("\n");
		
    for(i=0;i<ACC_BETA_SIZE;++i) {
      beta[i] -= delta[i];
    }
    
    reset_calibration_matrices();

  }
  for(i=0;i<ACC_BETA_SIZE;++i) {
		beta_acc[i] = beta[i];
   }
  DBG_PRINT("\n");
  for(i=0;i<ACC_BETA_SIZE;++i) {
    DBG_PRINT("beta_acc[%d]:%f",i,beta_acc[i]);
    DBG_PRINT(" ");
  }
	DBG_PRINT("\n");
	DBG_PRINT("Verify Beta Acc,\n");
	for(i=0;i<SAMPLE_SIZE_ACC;++i) {
		VerifyBetaAcc(data_acc+i*3, i);
	}
	
	return cal_status;
}
int8_t calibrate_model_mag(int16_t* data_mag, int sample_count) {
  int i,j;
  int num_iterations = GUASS_ITERATIONS;
  float change = 0.0,eclipse=0.0f,change_min=1000, eclipse_min=1000;
  int8_t cal_status=STATUS_ERROR;
  while (--num_iterations >=0) {
		change=0;
    compute_calibration_matrices_mag(data_mag, sample_count);
    find_delta(MAG_BETA_SIZE);
    //change = delta[0]*delta[0] + delta[1]*delta[1] + delta[2]*delta[2] + delta[3]*delta[3]/(beta_mag[3]*beta_mag[3]) + delta[4]*delta[4]/(beta_mag[4]*beta_mag[4]) + delta[5]*delta[5]/(beta_mag[5]*beta_mag[5]); 
    
    beta[0] -= delta[0];
		beta[1] -= delta[1];
		beta[2] -= delta[2];
		for(i=3;i<MAG_BETA_SIZE-1;i++)
			beta[i] -= delta[i];

		beta[9] -= delta[9];
		DBG_PRINT("[iteration:%d]  ", num_iterations);
		for(j=0;j<MAG_BETA_SIZE;j++) {
			DBG_PRINT("beta[%d]=%f ",j,beta[j]);
			change+=delta[j]*delta[j];
		}
		eclipse = VerifyBetaMagAvg(data_mag, sample_count);
		DBG_PRINT("change=%f, Fx_Avg=%f\n", change,eclipse);

		if(change_min>change) {
			if((eclipse<2) && (eclipse>-2)) {
				for(i=0;i<MAG_BETA_SIZE;i++)
				beta_min[i]=beta[i];
				change_min = change;
				eclipse_min = eclipse;
			}
		}

		if(change<0.01f)
				break;
    
    reset_calibration_matrices();
  }
	if(num_iterations<=0) {
		for(i=0;i<MAG_BETA_SIZE;i++)
			beta[i]=beta_min[i];
		
		change=change_min;
		eclipse=eclipse_min;
	}
	else
		cal_status=STATUS_NORMAL;
		
  for(i=0;i<MAG_BETA_SIZE;++i) {
    if((beta[i]==INFINITY)||(beta[i]==NAN)) {
      cal_status = STATUS_ERROR;
    }
  }
  DBG_PRINT("\n");
  for(i=0;i<MAG_BETA_SIZE;++i) {
    DBG_PRINT("beta_mag_eclipse[%d]:%f",i,beta[i]);
    DBG_PRINT(" ");
  }
	DBG_PRINT("change:%f,eclipse:%f",change,eclipse);
	DBG_PRINT("\n");
	DBG_PRINT("Verify Beta Mag Eclipse,\n");
	for(i=0;i<SAMPLE_SIZE_MAG;++i) {
		VerifyBetaMag(data_mag+i*3, i);
	}
	//Compute wInv
  compute_eigenvalue(beta);
	
	beta_mag[0] = beta[0];
	beta_mag[1] = beta[1];
	beta_mag[2] = beta[2];
	
	for(i=3;i<MAG_BETA_SIZE-1;i++) {
		beta_mag[i] = beta[i];
	}

	beta_mag[9] = beta[9];
	
	for(i=0;i<MAG_BETA_SIZE-1;++i) {
    DBG_PRINT("InvW[%d]:%f",i,beta_mag[i]);
    DBG_PRINT(" ");
  }
	
	DBG_PRINT("Mag Calibration Quality:%f\n", CalInfo.MagCalQuality);
	for(i=0;i<SAMPLE_SIZE_MAG;++i) {
		VerifyBetaMagCircle(data_mag+i*3);
	}
	return cal_status;
}
void EllipsoidInit()
{
	Ellipsoid.xrange[0]=1000;
	Ellipsoid.xrange[1]=-1000;
	Ellipsoid.yrange[0]=1000;
	Ellipsoid.yrange[1]=-1000;
	Ellipsoid.zrange[0]=1000;
	Ellipsoid.zrange[1]=-1000;
}
void UpdateEllipsoidRange(int16_t* sample_mag, int sample_count)
{
	int i;
	for(i=0; i<sample_count;i++) {
	//DBG_PRINT("sample_mag: %d %d %d\n",sample_mag[0],sample_mag[1],sample_mag[2]);
		if(Ellipsoid.xrange[1]<sample_mag[0+i*3])
			Ellipsoid.xrange[1]=(int)sample_mag[0+i*3];
		if(Ellipsoid.xrange[0]>sample_mag[0+i*3])
			Ellipsoid.xrange[0]=(int)sample_mag[0+i*3];
		if(Ellipsoid.yrange[1]<sample_mag[1+i*3])
			Ellipsoid.yrange[1]=(int)sample_mag[1+i*3];
		if(Ellipsoid.yrange[0]>sample_mag[1+i*3])
			Ellipsoid.yrange[0]=(int)sample_mag[1+i*3];
		if(Ellipsoid.zrange[1]<sample_mag[2+i*3])
			Ellipsoid.zrange[1]=(int)sample_mag[2+i*3];
		if(Ellipsoid.zrange[0]>sample_mag[2+i*3])
			Ellipsoid.zrange[0]=(int)sample_mag[2+i*3];
	}
}
void PreCaculateEllipsoidCenter()
{
	Ellipsoid.mean[0]=(Ellipsoid.xrange[0]+Ellipsoid.xrange[1])/2;
	Ellipsoid.mean[1]=(Ellipsoid.yrange[0]+Ellipsoid.yrange[1])/2;
	Ellipsoid.mean[2]=(Ellipsoid.zrange[0]+Ellipsoid.zrange[1])/2;
	DBG_PRINT("Ellipsoid Xrange: %d %d\n", Ellipsoid.xrange[0], Ellipsoid.xrange[1]);
	DBG_PRINT("Ellipsoid Yrange: %d %d\n", Ellipsoid.yrange[0], Ellipsoid.yrange[1]);
	DBG_PRINT("Ellipsoid Zrange: %d %d\n", Ellipsoid.zrange[0], Ellipsoid.zrange[1]);
	DBG_PRINT("Ellipsoid Mean: %d %d %d\n", Ellipsoid.mean[0], Ellipsoid.mean[1], Ellipsoid.mean[2]);
}
int8_t MagCalibrate(int16_t* data_mag, int sample_count)
{
	//DBG_PRINT("Calibrate MAG\n");
	EllipsoidInit();
	UpdateEllipsoidRange(data_mag, sample_count);
	PreCaculateEllipsoidCenter();
	setup_mag_beta();
	return calibrate_model_mag(data_mag, sample_count);
}
int8_t AccCalibrate(int16_t* data_acc, int sample_count)
{
	//DBG_PRINT("Calibrate ACC\n");
	setup_acc_beta();
	return calibrate_model_acc(data_acc, sample_count);
}
float* GetCalibrateParams(int8_t sensorType)
{
	if(sensorType==SENSOR_ACC)
		return beta_acc;
	else if(sensorType==SENSOR_MAG)
		return beta_mag;
	else
		return null;
}

void AccZCalibrate(int16_t* data_acc, int sample_count)
{
	int32_t sum_acc[3] = {0, 0, 0};
	uint8_t i;
	for(i = 0; i< sample_count; i++) {
		sum_acc[0]+=data_acc[i*3+0];
		sum_acc[1]+=data_acc[i*3+1];
		sum_acc[2]+=data_acc[i*3+2];
		//printf("%d  %d  %d\n",data_acc[i*3+0], data_acc[i*3+1],data_acc[i*3+2]);
	}
	beta_acc[0] = sum_acc[0]/sample_count;
	beta_acc[1] = sum_acc[1]/sample_count;
	beta_acc[2] = sum_acc[2]/sample_count - (1/CalInfo[AHRSID].AccG_PER_LSB);
	beta_acc[3] = CalInfo[AHRSID].AccG_PER_LSB;
	beta_acc[4] = CalInfo[AHRSID].AccG_PER_LSB;
	beta_acc[5] = CalInfo[AHRSID].AccG_PER_LSB;
}
