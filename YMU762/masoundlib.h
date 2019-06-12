/****************************************************************************
 *
 *		Copyright (C) 2002	YAMAHA CORPORATION. All rights reserved.
 *
 *		Module		: masoundlib.h
 *
 *		Description	: MA Sound Player LIB
 *
 *		Version		: 1.0.0.0		2002.11.19
 *
 *		History		:
 *			2002.11.19			1st try.
 ****************************************************************************/

#ifndef __MASOUNDLIB_H__
#define __MASOUNDLIB_H__

#if defined(__cplusplus)
extern "C" {
#endif

unsigned short	makeCRC(unsigned long dSize, unsigned char* pbData);
unsigned short	malib_MakeCRC(unsigned long dSize, unsigned char* pbData);
long			malib_smafphrase_checker(unsigned char* pbData, unsigned long dSize, void* pvInfo);
long			malib_smafaudio_checker(unsigned char* pbData, unsigned long dSize, unsigned char bMode, void* pvInfo);

typedef unsigned short	(*MASMWLIB_MAKECRC)(unsigned long dSize, unsigned char* pbData);
typedef long			(*MASMWLIB_PHRASECHECK)(unsigned char* pbData, unsigned long dSize, void* pvInfo);
typedef long			(*MASMWLIB_AUDIOCHECK)(unsigned char* pbData, unsigned long dSize, unsigned char bMode, void* pvInfo);

#if defined(__cplusplus)
}
#endif

#endif /*__MASOUNDLIB_H__*/
