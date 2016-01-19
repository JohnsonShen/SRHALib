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
#if 0
void take_sample_test_equ_acc(int16_t* sample_acc) 
{
	int G=4096,i;
	for(i=0;i<SAMPLE_SIZE_ACC;i++) 
	{
		int s[23]={0,3,6,9,12,15,18,
			         21,24,27,30,33,36,39,42,
			         44,46,48,50,52,54,56,60
		};
		if(i<s[1]) {//Y
			sample_acc[i*3]=0+1;
			sample_acc[i*3+1]=G+20;
			sample_acc[i*3+2]=0+1;
		}
		else if(i<s[2]) {//-Y
			sample_acc[i*3]=0+2;
			sample_acc[i*3+1]=-G+20;
			sample_acc[i*3+2]=0+2;
		}
		else if(i<s[3]) {//-Z
			sample_acc[i*3]=0+3;
			sample_acc[i*3+1]=0+3;
			sample_acc[i*3+2]=-G+3;
		}
		else if(i<s[4]) {//-X
			sample_acc[i*3]=-G+4;
			sample_acc[i*3+1]=0+4;
			sample_acc[i*3+2]=0+4;
		}
		else if(i<s[5]) {//X
			sample_acc[i*3]=G+5;
			sample_acc[i*3+1]=0+5;
			sample_acc[i*3+2]=0+5;
		}
		else if(i<s[6]) {//Z
			sample_acc[i*3]=0+6;
			sample_acc[i*3+1]=0+6;
			sample_acc[i*3+2]=G+6;
		}
		else if(i<s[7]) {//Y,-Z
			sample_acc[i*3]=0;
			sample_acc[i*3+1]=(G+10)/1.414f;
			sample_acc[i*3+2]=-(G+10)/1.414f;
		}
		else if(i<s[8]) {//-Y,-Z
			sample_acc[i*3]=0;
			sample_acc[i*3+1]=-(G+5)/1.414f;
			sample_acc[i*3+2]=-(G-7)/1.414f;
		}
		else if(i<s[9]) {//Y,Z
			sample_acc[i*3]=4;
			sample_acc[i*3+1]=(G-4)/1.414f;
			sample_acc[i*3+2]=(G-2)/1.414f;
		}
		else if(i<s[10]) {//-Y,Z
			sample_acc[i*3]=0;
			sample_acc[i*3+1]=-(G+3)/1.414f;
			sample_acc[i*3+2]=(G-1)/1.414f;
		}
		else if(i<s[11]) {//X,-Y
			sample_acc[i*3]=(G+8)/1.414f;
			sample_acc[i*3+1]=-(G-3)/1.414f;
			sample_acc[i*3+2]=0;
		}
		else if(i<s[12]) {//X,Y
			sample_acc[i*3]=(G-9)/1.414f;
			sample_acc[i*3+1]=(G+7)/1.414f;
			sample_acc[i*3+2]=0;
		}
		else if(i<s[13]) {//-X,Y
			sample_acc[i*3]=-(G-10)/1.414f;
			sample_acc[i*3+1]=(G+6)/1.414f;
			sample_acc[i*3+2]=0;
		}
		else if(i<s[14]) {//-X,-Y
			sample_acc[i*3]=-(G-1)/1.414f;
			sample_acc[i*3+1]=-(G+7)/1.414f;
			sample_acc[i*3+2]=0;
		}
		//
		else if(i<s[15]) {//X,Y,-Z
			sample_acc[i*3]=(G+10)/1.414f;;
			sample_acc[i*3+1]=(G+10)/1.414f;
			sample_acc[i*3+2]=-(G+10)/1.414f;
		}
		else if(i<s[16]) {//X,Y,Z
			sample_acc[i*3]=(G+1)/1.414f;;
			sample_acc[i*3+1]=(G+2)/1.414f;
			sample_acc[i*3+2]=(G+6)/1.414f;
		}
		else if(i<s[17]) {//X,-Y,-Z
			sample_acc[i*3]=(G+1)/1.414f;;
			sample_acc[i*3+1]=-(G+2)/1.414f;
			sample_acc[i*3+2]=-(G+6)/1.414f;
		}
		else if(i<s[18]) {//X,-Y,Z
			sample_acc[i*3]=(G+1)/1.414f;;
			sample_acc[i*3+1]=-(G+2)/1.414f;
			sample_acc[i*3+2]=(G+6)/1.414f;
		}
		else if(i<s[19]) {//-X,Y,-Z
			sample_acc[i*3]=-(G+1)/1.414f;;
			sample_acc[i*3+1]=(G+2)/1.414f;
			sample_acc[i*3+2]=-(G+6)/1.414f;
		}
		else if(i<s[20]) {//-X,Y,-Z
			sample_acc[i*3]=-(G+7)/1.414f;;
			sample_acc[i*3+1]=(G+3)/1.414f;
			sample_acc[i*3+2]=-(G+9)/1.414f;
		}
		else if(i<s[21]) {//-X,-Y,Z
			sample_acc[i*3]=-(G+4)/1.414f;;
			sample_acc[i*3+1]=-(G+7)/1.414f;
			sample_acc[i*3+2]=(G+2)/1.414f;
		}
		else if(i<s[22]) {//-X,Y,Z
			sample_acc[i*3]=-(G+6)/1.414f;;
			sample_acc[i*3+1]=(G+12)/1.414f;
			sample_acc[i*3+2]=-(G+6)/1.414f;
		}
	}
}
void take_sample_test_equ_mag(int16_t* sample_mag) 
{
	int a,b,c,guass=100,i,j,x,y,z;
	float Betas=-89*M_PI/180.0f,BetaStep=178/5*M_PI/180.0f,Ld=-180*M_PI/180.0f,LdStep=360/10*M_PI/180.0f,rotateZ=0.0f*M_PI/180.0f;
	a=guass*1.5f;b=guass*1.5f;c=guass*1.5f;
	
	
	for(i=0;i<6;i++)  {
		for(j=0;j<10;j++)  {
			x=sample_mag[(i*10+j)*3]=a*cos(Betas)*cos(Ld);
			y=sample_mag[(i*10+j)*3+1]=b*cos(Betas)*sin(Ld);
			z=sample_mag[(i*10+j)*3+2]=c*sin(Betas);
			
			
			//Rotation By X-Axis
			// 1          0     0
			// 0      Cos(Th) -Sin(Th)
			// 0      Sin(Th)  Cos(Th)
			//sample_mag[(i*10+j)*3]=x;//x*cos(rotateZ) - y*sin(rotateZ);
			//sample_mag[(i*10+j)*3+1]=y*cos(rotateZ) - z*sin(rotateZ);
			//sample_mag[(i*10+j)*3+2]=y*sin(rotateZ) + z*cos(rotateZ);
			//Rotation By Y-Axis
			// Cos(Th)   0     Sin(Th)
			//    0      1      0
			// -Sin(Th)  0     Cos(Th)
			/*sample_mag[(i*10+j)*3]=x*cos(rotateZ) + z*sin(rotateZ);
			sample_mag[(i*10+j)*3+1]=y;
			sample_mag[(i*10+j)*3+2]=-x*sin(rotateZ) + z*cos(rotateZ);*/
			
			//Rotation By Z-Axis
			//Cos(Th) -Sin(Th) 0
			//Sin(Th)  Cos(Th) 0
			// 0          0    1
			//x = sample_mag[(i*10+j)*3];
			//y = sample_mag[(i*10+j)*3+1];
			//z = sample_mag[(i*10+j)*3+2];
			sample_mag[(i*10+j)*3]=x*cos(rotateZ) - y*sin(rotateZ);
			sample_mag[(i*10+j)*3+1]=x*sin(rotateZ) + y*cos(rotateZ);
			sample_mag[(i*10+j)*3+2]=z;
			
			Ld+=LdStep;//+(((i+j+1)*j)%20)*M_PI/180.0f;
		}
		Betas+=BetaStep;//+(((i+1)*i)%20)*M_PI/180.0f;;
		Ld=-180*M_PI/180.0f;
	}
}
//pass in a length-3 array where the data will be written
void take_sample_test(int16_t* sample_acc, int16_t* sample_mag) {
  int i=0,guass=660*0.4,G=4096;
	for(i=0;i<SAMPLE_SIZE_ACC;i++) 
	{
		int x[23]={0,3,6,9,12,15,18,
			         21,24,27,30,33,36,39,42,
			         44,46,48,50,52,54,56,60
		};
		/*int x[23]={0,10,20,30,40,50,60,
			         21,24,27,30,33,36,39,42,
			         44,46,48,50,52,54,56,60
		};*/
		if(i<x[1]) {//Y
			sample_acc[i*3]=0+1;
			sample_acc[i*3+1]=G+20;
			sample_acc[i*3+2]=0+1;
		}
		else if(i<x[2]) {//-Y
			sample_acc[i*3]=0+2;
			sample_acc[i*3+1]=-G+20;
			sample_acc[i*3+2]=0+2;
		}
		else if(i<x[3]) {//-Z
			sample_acc[i*3]=0+3;
			sample_acc[i*3+1]=0+3;
			sample_acc[i*3+2]=-G+3;
		}
		else if(i<x[4]) {//-X
			sample_acc[i*3]=-G+4;
			sample_acc[i*3+1]=0+4;
			sample_acc[i*3+2]=0+4;
		}
		else if(i<x[5]) {//X
			sample_acc[i*3]=G+5;
			sample_acc[i*3+1]=0+5;
			sample_acc[i*3+2]=0+5;
		}
		else if(i<x[6]) {//Z
			sample_acc[i*3]=0+6;
			sample_acc[i*3+1]=0+6;
			sample_acc[i*3+2]=G+6;
		}
		else if(i<x[7]) {//Y,-Z
			sample_acc[i*3]=0;
			sample_acc[i*3+1]=(G+10)/1.414f;
			sample_acc[i*3+2]=-(G+10)/1.414f;
		}
		else if(i<x[8]) {//-Y,-Z
			sample_acc[i*3]=0;
			sample_acc[i*3+1]=-(G+5)/1.414f;
			sample_acc[i*3+2]=-(G-7)/1.414f;
		}
		else if(i<x[9]) {//Y,Z
			sample_acc[i*3]=4;
			sample_acc[i*3+1]=(G-4)/1.414f;
			sample_acc[i*3+2]=(G-2)/1.414f;
		}
		else if(i<x[10]) {//-Y,Z
			sample_acc[i*3]=0;
			sample_acc[i*3+1]=-(G+3)/1.414f;
			sample_acc[i*3+2]=(G-1)/1.414f;
		}
		else if(i<x[11]) {//X,-Y
			sample_acc[i*3]=(G+8)/1.414f;
			sample_acc[i*3+1]=-(G-3)/1.414f;
			sample_acc[i*3+2]=0;
		}
		else if(i<x[12]) {//X,Y
			sample_acc[i*3]=(G-9)/1.414f;
			sample_acc[i*3+1]=(G+7)/1.414f;
			sample_acc[i*3+2]=0;
		}
		else if(i<x[13]) {//-X,Y
			sample_acc[i*3]=-(G-10)/1.414f;
			sample_acc[i*3+1]=(G+6)/1.414f;
			sample_acc[i*3+2]=0;
		}
		else if(i<x[14]) {//-X,-Y
			sample_acc[i*3]=-(G-1)/1.414f;
			sample_acc[i*3+1]=-(G+7)/1.414f;
			sample_acc[i*3+2]=0;
		}
		//
		else if(i<x[15]) {//X,Y,-Z
			sample_acc[i*3]=(G+10)/1.414f;;
			sample_acc[i*3+1]=(G+10)/1.414f;
			sample_acc[i*3+2]=-(G+10)/1.414f;
		}
		else if(i<x[16]) {//X,Y,Z
			sample_acc[i*3]=(G+1)/1.414f;;
			sample_acc[i*3+1]=(G+2)/1.414f;
			sample_acc[i*3+2]=(G+6)/1.414f;
		}
		else if(i<x[17]) {//X,-Y,-Z
			sample_acc[i*3]=(G+1)/1.414f;;
			sample_acc[i*3+1]=-(G+2)/1.414f;
			sample_acc[i*3+2]=-(G+6)/1.414f;
		}
		else if(i<x[18]) {//X,-Y,Z
			sample_acc[i*3]=(G+1)/1.414f;;
			sample_acc[i*3+1]=-(G+2)/1.414f;
			sample_acc[i*3+2]=(G+6)/1.414f;
		}
		else if(i<x[19]) {//-X,Y,-Z
			sample_acc[i*3]=-(G+1)/1.414f;;
			sample_acc[i*3+1]=(G+2)/1.414f;
			sample_acc[i*3+2]=-(G+6)/1.414f;
		}
		else if(i<x[20]) {//-X,Y,-Z
			sample_acc[i*3]=-(G+7)/1.414f;;
			sample_acc[i*3+1]=(G+3)/1.414f;
			sample_acc[i*3+2]=-(G+9)/1.414f;
		}
		else if(i<x[21]) {//-X,-Y,Z
			sample_acc[i*3]=-(G+4)/1.414f;;
			sample_acc[i*3+1]=-(G+7)/1.414f;
			sample_acc[i*3+2]=(G+2)/1.414f;
		}
		else if(i<x[22]) {//-X,Y,Z
			sample_acc[i*3]=-(G+6)/1.414f;;
			sample_acc[i*3+1]=(G+12)/1.414f;
			sample_acc[i*3+2]=-(G+6)/1.414f;
		}
	}
  /*for(i=0;i<SAMPLE_SIZE;i++) 
	{
		if(i<10) {
			sample_acc[i*3]=G+1;
			sample_acc[i*3+1]=0+1;
			sample_acc[i*3+2]=0+1;
		}
		else if(i<20) {
			sample_acc[i*3]=0+2;
			sample_acc[i*3+1]=0+2;
			sample_acc[i*3+2]=-G+2;
		}
		else if(i<30) {
			sample_acc[i*3]=0+3;
			sample_acc[i*3+1]=-G+3;
			sample_acc[i*3+2]=0+3;
		}
		else if(i<40) {
			sample_acc[i*3]=0+4;
			sample_acc[i*3+1]=0+4;
			sample_acc[i*3+2]=G+4;
		}
		else if(i<50) {
			sample_acc[i*3]=0+5;
			sample_acc[i*3+1]=G+5;
			sample_acc[i*3+2]=0+5;
		}
		else if(i<60) {
			sample_acc[i*3]=-G+6;
			sample_acc[i*3+1]=0+6;
			sample_acc[i*3+2]=0+6;
		}
	}*/
	
	for(i=0;i<SAMPLE_SIZE_MAG;i++) 
	{
		int x[23]={0,3,6,9,12,15,18,
			         21,24,27,30,33,36,39,42,
			         44,46,48,50,52,54,56,60
		};
		if(i<x[1]) {//Y
			sample_mag[i*3]=0+1;
			sample_mag[i*3+1]=guass-20;
			sample_mag[i*3+2]=0+1;
		}
		else if(i<x[2]) {//-Y
			sample_mag[i*3]=0+2;
			sample_mag[i*3+1]=-guass-20;
			sample_mag[i*3+2]=0+2;
		}
		else if(i<x[3]) {//-Z
			sample_mag[i*3]=0+3;
			sample_mag[i*3+1]=0+3;
			sample_mag[i*3+2]=-guass+50;
		}
		else if(i<x[4]) {//-X
			sample_mag[i*3]=-guass+4;
			sample_mag[i*3+1]=0+4;
			sample_mag[i*3+2]=0+4;
		}
		else if(i<x[5]) {//X
			sample_mag[i*3]=guass+5;
			sample_mag[i*3+1]=0+5;
			sample_mag[i*3+2]=0+5;
		}
		else if(i<x[6]) {//Z
			sample_mag[i*3]=0+6;
			sample_mag[i*3+1]=0+6;
			sample_mag[i*3+2]=guass+6;
		}
		else if(i<x[7]) {//Y,-Z
			sample_mag[i*3]=1;
			sample_mag[i*3+1]=(guass+10)/1.414f;
			sample_mag[i*3+2]=-(guass+10)/1.414f;
		}
		else if(i<x[8]) {//-Y,-Z
			sample_mag[i*3]=2;
			sample_mag[i*3+1]=-(guass+5)/1.414f;
			sample_mag[i*3+2]=-(guass-7)/1.414f;
		}
		else if(i<x[9]) {//Y,Z
			sample_mag[i*3]=4;
			sample_mag[i*3+1]=(guass-4)/1.414f;
			sample_mag[i*3+2]=(guass-2)/1.414f;
		}
		else if(i<x[10]) {//-Y,Z
			sample_mag[i*3]=3;
			sample_mag[i*3+1]=-(guass+3)/1.414f;
			sample_mag[i*3+2]=(guass-1)/1.414f;
		}
		else if(i<x[11]) {//X,-Y
			sample_mag[i*3]=(guass+8)/1.414f;
			sample_mag[i*3+1]=-(guass-3)/1.414f;
			sample_mag[i*3+2]=2;
		}
		else if(i<x[12]) {//X,Y
			sample_mag[i*3]=(guass-9)/1.414f;
			sample_mag[i*3+1]=(guass+7)/1.414f;
			sample_mag[i*3+2]=1;
		}
		else if(i<x[13]) {//-X,Y
			sample_mag[i*3]=-(guass-10)/1.414f;
			sample_mag[i*3+1]=(guass+6)/1.414f;
			sample_mag[i*3+2]=2;
		}
		else if(i<x[14]) {//-X,-Y
			sample_mag[i*3]=-(guass-1)/1.414f;
			sample_mag[i*3+1]=-(guass+7)/1.414f;
			sample_mag[i*3+2]=3;
		}
		//
		else if(i<x[15]) {//X,Y,-Z
			sample_mag[i*3]=(guass+10)/1.732f;
			sample_mag[i*3+1]=(guass+10)/1.732f;
			sample_mag[i*3+2]=-(guass+10)/1.732f;
		}
		else if(i<x[16]) {//X,Y,Z
			sample_mag[i*3]=(guass+1)/1.732f;
			sample_mag[i*3+1]=(guass+2)/1.732f;
			sample_mag[i*3+2]=(guass+6)/1.732f;
		}
		else if(i<x[17]) {//X,-Y,-Z
			sample_mag[i*3]=(guass+1)/1.732f;
			sample_mag[i*3+1]=-(guass+2)/1.732f;
			sample_mag[i*3+2]=-(guass+6)/1.732f;
		}
		else if(i<x[18]) {//X,-Y,Z
			sample_mag[i*3]=(guass+1)/1.732f;
			sample_mag[i*3+1]=-(guass+2)/1.732f;
			sample_mag[i*3+2]=(guass+6)/1.732f;
		}
		else if(i<x[19]) {//-X,Y,-Z
			sample_mag[i*3]=-(guass+1)/1.732f;
			sample_mag[i*3+1]=(guass+2)/1.732f;
			sample_mag[i*3+2]=-(guass+6)/1.732f;
		}
		else if(i<x[20]) {//-X,-Y,-Z
			sample_mag[i*3]=-(guass+7)/1.732f;
			sample_mag[i*3+1]=-(guass+3)/1.732f;
			sample_mag[i*3+2]=-(guass+9)/1.732f;
		}
		else if(i<x[21]) {//-X,-Y,Z
			sample_mag[i*3]=-(guass+4)/1.732f;
			sample_mag[i*3+1]=-(guass+7)/1.732f;
			sample_mag[i*3+2]=(guass+2)/1.732f;
		}
		else if(i<x[22]) {//-X,Y,Z
			sample_mag[i*3]=-(guass+6)/1.732f;
			sample_mag[i*3+1]=(guass+12)/1.732f;
			sample_mag[i*3+2]=-(guass+6)/1.732f;
		}
	}
	/*for(i=0;i<SAMPLE_SIZE;i++) 
	{
		if(i<5) {
			sample_mag[i*3]=0.07f*MAG_GAUSS_PER_LSB;
			sample_mag[i*3+1]=0.18f*MAG_GAUSS_PER_LSB;
			sample_mag[i*3+2]=0.13f*MAG_GAUSS_PER_LSB;
		}
		else if(i<10) {
			sample_mag[i*3]=0.13f*MAG_GAUSS_PER_LSB;
			sample_mag[i*3+1]=-0.29f*MAG_GAUSS_PER_LSB;
			sample_mag[i*3+2]=0.09f*MAG_GAUSS_PER_LSB;
		}
		else if(i<15) {
			sample_mag[i*3]=-0.17f*MAG_GAUSS_PER_LSB;
			sample_mag[i*3+1]=-0.14f*MAG_GAUSS_PER_LSB;
			sample_mag[i*3+2]=0.22f*MAG_GAUSS_PER_LSB;
		}
		else if(i<20) {
			sample_mag[i*3]=-0.26f*MAG_GAUSS_PER_LSB;
			sample_mag[i*3+1]=0.1f*MAG_GAUSS_PER_LSB;
			sample_mag[i*3+2]=-0.05f*MAG_GAUSS_PER_LSB;
		}
		else if(i<25) {
			sample_mag[i*3]=0.23f*MAG_GAUSS_PER_LSB;
			sample_mag[i*3+1]=0.05f*MAG_GAUSS_PER_LSB;
			sample_mag[i*3+2]=-0.14f*MAG_GAUSS_PER_LSB;
		}
		else if(i<30) {
			sample_mag[i*3]=-0.19f*MAG_GAUSS_PER_LSB;
			sample_mag[i*3+1]=-0.21f*MAG_GAUSS_PER_LSB;
			sample_mag[i*3+2]=-0.18f*MAG_GAUSS_PER_LSB;
		}
		else if(i<33) {
			sample_mag[i*3]=0.14f*MAG_GAUSS_PER_LSB;
			sample_mag[i*3+1]=-0.12f*MAG_GAUSS_PER_LSB;
			sample_mag[i*3+2]=0.23f*MAG_GAUSS_PER_LSB;
		}
		else if(i<36) {
			sample_mag[i*3]=0.02f*MAG_GAUSS_PER_LSB;
			sample_mag[i*3+1]=0.11f*MAG_GAUSS_PER_LSB;
			sample_mag[i*3+2]=0.21f*MAG_GAUSS_PER_LSB;
		}
		else if(i<39) {
			sample_mag[i*3]=-0.19f*MAG_GAUSS_PER_LSB;
			sample_mag[i*3+1]=0.0f*MAG_GAUSS_PER_LSB;
			sample_mag[i*3+2]=0.21f*MAG_GAUSS_PER_LSB;
		}
		else if(i<42) {
			sample_mag[i*3]=-0.03f*MAG_GAUSS_PER_LSB;
			sample_mag[i*3+1]=-0.24f*MAG_GAUSS_PER_LSB;
			sample_mag[i*3+2]=0.22f*MAG_GAUSS_PER_LSB;
		}
		else if(i<45) {
			sample_mag[i*3]=0.2f*MAG_GAUSS_PER_LSB;
			sample_mag[i*3+1]=-0.23f*MAG_GAUSS_PER_LSB;
			sample_mag[i*3+2]=0.12f*MAG_GAUSS_PER_LSB;
		}
		else if(i<48) {
			sample_mag[i*3]=0.06f*MAG_GAUSS_PER_LSB;
			sample_mag[i*3+1]=-0.21f*MAG_GAUSS_PER_LSB;
			sample_mag[i*3+2]=-0.18f*MAG_GAUSS_PER_LSB;
		}
		else if(i<51) {
			sample_mag[i*3]=-0.24f*MAG_GAUSS_PER_LSB;
			sample_mag[i*3+1]=-0.2f*MAG_GAUSS_PER_LSB;
			sample_mag[i*3+2]=-0.08f*MAG_GAUSS_PER_LSB;
		}
		else if(i<60) {
			sample_mag[i*3]=-0.15f*MAG_GAUSS_PER_LSB;
			sample_mag[i*3+1]=-0.2f*MAG_GAUSS_PER_LSB;
			sample_mag[i*3+2]=0.21f*MAG_GAUSS_PER_LSB;
		}
	}*/
  
}
#endif
