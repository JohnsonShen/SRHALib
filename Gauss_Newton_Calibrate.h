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
#ifndef _GAUSS_NEWTON_CALIBRATE_H
#define _GAUSS_NEWTON_CALIBRATE_H
//#define SIDE 	14
//#define SAMPLE_PER_SIDE 5
//#define SIDE_ACC 6
#define SAMPLE_ACC_PER_SIDE 10
#define SAMPLE_MAG_PER_SIDE 120
#define SIDE_ACC 6
#define SIDE_MAG 1
#define SAMPLE_SIZE_ACC (SIDE_ACC*SAMPLE_ACC_PER_SIDE)
#define SAMPLE_SIZE_MAG (SIDE_MAG*SAMPLE_MAG_PER_SIDE)
#define MAG_BETA_SIZE 10
#define ACC_BETA_SIZE 6
#define ACC_ROTATE_SIZE 9
#define EIGEN_BETA_SIZE 4
#define MAX_BETA_SIZE MAG_BETA_SIZE
#define GUESS_R 0.6f
float* get_beta_acc(void);
float* get_beta_mag(void);
void setup(void);
void loop(int8_t sensorType);
typedef struct {
	int xrange[2];
	int yrange[2];
	int zrange[2];
	int mean[3];
} Ellipsoid_T;
#endif
