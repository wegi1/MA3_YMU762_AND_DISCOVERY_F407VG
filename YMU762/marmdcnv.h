/*==============================================================================
//	Copyright (C) 2001-2002 YAMAHA CORPORATION
//
//	Title		: MARMDCNV.H
//
//	Description	: MA-3 Realtime MIDI/GML Stream Converter Module.
//
//	Version		: 1.5.1.0
//
//	History		:
//					May 8, 2001		1st try.
//============================================================================*/
#ifndef	_MARMDCNV_H_
#define	_MARMDCNV_H_

#include "mamachdep.h"
#include "masnddrv.h"		/*	MA-X Sound Driver		*/
#include "maresmgr.h"		/*	MA-X Resource Manager	*/
#include "madebug.h"

/*==============================================================================
//	define ERROR Code
//============================================================================*/
/*	convert error code	*/
#define	MA3RMDERR_NOERROR			MASMW_SUCCESS
#define	MA3RMDERR_GENERAL			MASMW_ERROR
#define	MA3RMDERR_ARGUMENT			MASMW_ERROR_ARGUMENT
#define	MA3RMDERR_ID				MASMW_ERROR_ID
#define	MA3RMDERR_NO_RESOURCE		MASMW_ERROR_RESOURCE_OVER

typedef struct _MASMW_MIDIMSG
{
	UINT8*	pbMsg;						/* pointer to MIDI message */
	UINT32	dSize;						/* size of MIDI message */
} MASMW_MIDIMSG, *PMASMW_MIDIMSG;

/*==============================================================================
//	prototype functions
//============================================================================*/
SINT32	MaRmdCnv_Initialize	(void);
SINT32	MaRmdCnv_End		(void);
SINT32	MaRmdCnv_Load		(UINT8* file_ptr, UINT32 file_size, UINT8 mode, SINT32 (*func)(UINT8 id), void* ext_args);
SINT32	MaRmdCnv_Unload		(SINT32 file_id, void* ext_args);
SINT32	MaRmdCnv_Open		(SINT32 file_id, UINT16 mode, void* ext_args);
SINT32	MaRmdCnv_Close		(SINT32 file_id, void* ext_args);
SINT32	MaRmdCnv_Standby	(SINT32 file_id, void* ext_args);
SINT32	MaRmdCnv_Seek		(SINT32 file_id, UINT32 pos, UINT8 flag, void* ext_args);
SINT32	MaRmdCnv_Start		(SINT32 file_id, void* ext_args);
SINT32	MaRmdCnv_Stop		(SINT32 file_id, void* ext_args);
SINT32	MaRmdCnv_Control	(SINT32 file_id, UINT8 ctrl_num, void* prm, void* ext_args);
SINT32	MaRmdCnv_Convert	(void);

#endif	/*	_MARMDCNV_H_	*/

