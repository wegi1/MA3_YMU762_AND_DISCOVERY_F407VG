/*==============================================================================
//	Copyright(c) 2002 YAMAHA CORPORATION
//
//	Title		: MAMIL.H
//
//	Description	: MA-3 SMAF/Phrase/Audio Error Check Module.
//
//	Version		: 0.8.0.1		2002.11.15
//
//============================================================================*/
#include "madefs.h"
#include "mamachdep.h"

/*	Contents class		*/
#define	MALIB_CNTI_CLASS_YAMAHA		0x00

SINT32	malib_smafphrase_checker(UINT8* pbBuf, UINT32 dwSize, void* pvPCA);
SINT32	malib_smafaudio_checker	(UINT8* pbBuf, UINT32 dwSize, UINT8 bMode, void* pvI);
UINT16	malib_MakeCRC			(UINT32 dwSize, UINT8* pbBuf);
UINT16	makeCRC					(UINT32 dwSize, UINT8* pbBuf);
