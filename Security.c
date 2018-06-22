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
#include <stdint.h>
#include "common.h"
#ifndef M451
#include "NUC1xx.h"
#include "DrvFMC.h"
#else
#include "fmc.h"
#endif
static uint32_t UID[3],UCID[4],PID;
#define NUC123 0x00012315
#define NANO105 0x00002800
#define NUC140 0x02508c0a
#define M452   0x00840000,0x00940000
#define M452LG6AE   0x00845200
uint32_t IC_LIST[] = {NUC123,M452,NANO105};
#ifndef M451
uint32_t FMCReadPID(void)
{
	return (uint32_t)SYS->PDID;
}
int32_t FMCReadUID(uint32_t * u32UID)
{ 
	uint8_t i;

	for(i=0;i<3;i++) {
		FMC->ISPCMD.FCTRL = 4;
		FMC->ISPCMD.FCEN = 0;
		FMC->ISPCMD.FOEN = 0;
		FMC->ISPADR	= i*4;
		FMC->ISPDAT	= 0;
		FMC->ISPTRG.ISPGO = 1;    
		__ISB();
		while (FMC->ISPTRG.ISPGO);

		if (FMC->ISPCON.ISPFF == 1)
		{
			FMC->ISPCON.ISPFF = 1;
			return E_DRVFMC_ERR_ISP_FAIL;
		}
		
		*u32UID++=FMC->ISPDAT;
	}

	return 0;
}
int32_t FMCReadUCID(uint32_t * u32UCID)
{ 
	uint8_t i;

	for(i=0;i<4;i++) {
		FMC->ISPCMD.FCTRL = 4;
		FMC->ISPCMD.FCEN = 0;
		FMC->ISPCMD.FOEN = 0;
		FMC->ISPADR	= i*4+0x100;
		FMC->ISPDAT	= 0;
		FMC->ISPTRG.ISPGO = 1;    
		__ISB();
		while (FMC->ISPTRG.ISPGO);

		if (FMC->ISPCON.ISPFF == 1)
		{
			FMC->ISPCON.ISPFF = 1;
			return E_DRVFMC_ERR_ISP_FAIL;
		}
		
		*u32UCID++=FMC->ISPDAT;
	}

	return 0;
}
#else //M451

uint32_t FMCReadPID(void)
{
    FMC->ISPCMD = FMC_ISPCMD_READ_DID;          /* Set ISP Command Code */
    FMC->ISPADDR = 0x04;                         /* Must keep 0x4 when read PID */
    FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;         /* Trigger to start ISP procedure */
#if ISBEN
    __ISB();
#endif                                     /* To make sure ISP/CPU be Synchronized */
    while(FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk);  /* Waiting for ISP Done */

    return FMC->ISPDAT;
}
int32_t FMCReadUID(uint32_t * u32UID)
{ 
	uint8_t i;

	for(i=0;i<3;i++) {
		FMC->ISPCMD = 4;
		FMC->ISPADDR	= i*4;
		FMC->ISPDAT	= 0;
		FMC->ISPTRG = 1;    
		while (FMC->ISPTRG);
		*u32UID++=FMC->ISPDAT;
	}

	return 0;
}
int32_t FMCReadUCID(uint32_t * u32UCID)
{ 
	uint8_t i;

	for(i=0;i<4;i++) {
		FMC->ISPCMD = 4;
		FMC->ISPADDR	= i*4+0x100;
		FMC->ISPDAT	= 0;
		FMC->ISPTRG = 1;    
		__ISB();
		while (FMC->ISPTRG);

		if (FMC->ISPCTL&0x40)
		{
			FMC->ISPCTL = (FMC->ISPCTL|0x40);
			return -1;
		}
		
		*u32UCID++=FMC->ISPDAT;
	}

	return 0;
}
#endif
uint8_t CheckSecurityID(void)
{
	char IcNum = sizeof(IC_LIST)/4, i;
	for(i = 0; i<IcNum; i++) {
#ifndef M451
		if((PID&0x0000ff00)==(IC_LIST[i]&0x0000ff00))
#else
		if((PID&0xffff0000)==(IC_LIST[i]&0xffff0000))
#endif
		return 0;
	}
	
		return 1;
}

uint8_t SetupSecurityID(void)
{
	__set_PRIMASK(1);
#ifndef M451
	UNLOCKREG();
	DrvFMC_EnableISP();
	FMCReadUID(&UID[0]);
	FMCReadUCID(&UCID[0]);
	PID	= FMCReadPID();
	DrvFMC_DisableISP(); 
	LOCKREG();
#else
	SYS_UnlockReg();
	FMC_Open();
	FMCReadUID(&UID[0]);
	FMCReadUCID(&UCID[0]);
	PID	= FMCReadPID();
	FMC_Close(); 
	SYS_LockReg();	
#endif
	__set_PRIMASK(0);
	return 0;
}
