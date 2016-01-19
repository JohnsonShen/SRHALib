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
#ifndef _SENSORFUSION6_H
#define _SENSORFUSION6_H
#include "Common.h"
void sensfusion6UpdateQ(float gxf, float gyf, float gzf, float axf, float ayf, float azf, float dt);
void sensfusion9UpdateQ(float gxf, float gyf, float gzf, float axf, float ayf, float azf, float mxf, float myf, float mzf, float dt);
void sensfusion6Getquaternion(Axis4f* Quaternion);
void sensfusion6GetAcc(float* accOut);
void UpdateMagMasterTime(void);
#endif
