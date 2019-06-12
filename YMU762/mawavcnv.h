/****************************************************************************
 *
 *		Copyright (C) 2002-2003	YAMAHA CORPORATION. All rights reserved.
 *
 *		Module		: mawavcnv.h
 *
 *		Description	: Header of WAV Stream Converter Module.
 *
 *		Version		: 1.1.0.1		2003.04.09
 *
 *		History		:
 *			2002/09/09 1st try.
 ****************************************************************************/
#ifndef	_MAWAVCNV_H_
#define	_MAWAVCNV_H_

#include "mamachdep.h"
#include "masnddrv.h"		/*	MA-X Sound Driver		*/
#include "maresmgr.h"		/*	MA-X Resource Manager	*/
#include "madebug.h"

/*==============================================================================
//	prototype functions
//============================================================================*/
SINT32	MaWavCnv_Initialize	( void );
SINT32	MaWavCnv_End		( void );
SINT32	MaWavCnv_Load		(UINT8* pbFile, UINT32 dFSize, UINT8 bMode, SINT32	(*pFunc)(UINT8 bId), void* pvExt_args);
SINT32	MaWavCnv_Unload		(SINT32 sdFile_id, void* pvExt_args);
SINT32	MaWavCnv_Open		(SINT32 sdFile_id, UINT16 wMode, void* pvExt_args);
SINT32	MaWavCnv_Standby	(SINT32 sdFile_id, void* pvExt_args);
SINT32	MaWavCnv_Close		(SINT32 sdFile_id, void* pvExt_args);
SINT32	MaWavCnv_Seek		(SINT32 sdFile_id, UINT32 dPos, UINT8 bFlag, void* pvExt_args);
SINT32	MaWavCnv_Start		(SINT32 sdFile_id, void* pvExt_args);
SINT32	MaWavCnv_Stop		(SINT32 sdFile_id, void* pvExt_args);
SINT32	MaWavCnv_Control	(SINT32 sdFile_id, UINT8 bCtrl_num, void* pvPrm, void* pvExt_args);

#endif	/*	_MAWAVCNV_H_	*/

