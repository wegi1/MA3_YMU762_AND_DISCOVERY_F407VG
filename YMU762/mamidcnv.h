/*==============================================================================
//	Copyright (C) 2001-2003 YAMAHA CORPORATION
//
//	Title		: MAMIDCNV.H
//
//	Description	: MA-3 SMF/GML Stream Converter Module.
//
//	Version		: 1.10.2.2
//
//	History		:
//					May 8, 2001		1st try.
//					June 24, 2002	Add 4op Voice&Drum parameter.
//============================================================================*/
#ifndef	_MAMIDCNV_H_
#define	_MAMIDCNV_H_

#include "mamachdep.h"
#include "masnddrv.h"		/*	MA-X Sound Driver		*/
#include "maresmgr.h"		/*	MA-X Resource Manager	*/
#include "madebug.h"

/*==============================================================================
//	prototype functions
//============================================================================*/
SINT32	MaMidCnv_Initialize	(void);
SINT32	MaMidCnv_End		(void);
SINT32	MaMidCnv_Load		(UINT8* file_ptr, UINT32 file_size, UINT8 mode, SINT32 (*func)(UINT8 id), void* ext_args);
SINT32	MaMidCnv_Unload		(SINT32 file_id, void* ext_args);
SINT32	MaMidCnv_Open		(SINT32 file_id, UINT16 mode, void* ext_args);
SINT32	MaMidCnv_Close		(SINT32 file_id, void* ext_args);
SINT32	MaMidCnv_Standby	(SINT32 file_id, void* ext_args);
SINT32	MaMidCnv_Seek		(SINT32 file_id, UINT32 pos, UINT8 flag, void* ext_args);
SINT32	MaMidCnv_Start		(SINT32 file_id, void* ext_args);
SINT32	MaMidCnv_Stop		(SINT32 file_id, void* ext_args);
SINT32	MaMidCnv_Control	(SINT32 file_id, UINT8 ctrl_num, void* prm, void* ext_args);
SINT32	MaMidCnv_Convert	(void);

#endif	/*	_MAMIDCNV_H_	*/

