/****************************************************************************
 *																			*
 *		Copyright (C) 2001-2002	YAMAHA CORPORATION							*
 *																			*
 *		Module		: madebug.h												*
 *																			*
 *		Description	: for MA-3 Sound Middleware debug						*
 *																			*
 * 		Version		: 1.3.12.3	2002.11.15									*
 *																			*
 ****************************************************************************/

#ifndef __MADEBUG_H__
#define __MADEBUG_H__

#define DEBUG		(0)
#define	MASMW_DEBUG	(0)

#if DEBUG
 #include <stdio.h>
 #include <assert.h>
#endif

#if DEBUG
#define DEBUG_MA_SMF_STREAM_CONVERTER
#define DEBUG_MA_SMAF_AUDIO_STREAM_CONVERTER
#define DEBUG_MA_REALTIME_MIDI_STREAM_CONVERTER
#define DEBUG_MA_SMAF_PHRASE_STREAM_CONVERTER
#define DEBUG_MA_SMAF_CONVERTER
#define DEBUG_MA_SOUND_SEQUENCER
#define DEBUG_MA_SOUND_DRIVER
#define DEBUG_MA_RESOURCE_MANAGER
#define DEBUG_MA_DEVICE_DRIVER

#define MASMW_ASSERT(f)	assert(f)
#else
#define MASMW_ASSERT(f)	((void)0)
#endif


/* */


#ifdef DEBUG_MA_SMF_STREAM_CONVERTER			/* SMF Converter */
 #define MASMFCNV_DBGMSG(a)	printf a
#else
 #define MASMFCNV_DBGMSG(a)
#endif

#ifdef DEBUG_MA_WAV_STREAM_CONVERTER			/* WAV Converter */
 #define MAWAVCNV_DBGMSG(a)	printf a
#else
 #define MAWAVCNV_DBGMSG(a)
#endif

#ifdef DEBUG_MA_SMAF_AUDIO_STREAM_CONVERTER		/* SMAF/Audio Converter */
 #define MAAUDCNV_DBGMSG(a)	printf a
#else
 #define MAAUDCNV_DBGMSG(a)
#endif

#ifdef DEBUG_MA_REALTIME_MIDI_STREAM_CONVERTER	/* Realtime MIDI Converter */
 #define MARMDCNV_DBGMSG(a)	printf a
#else
 #define MARMDCNV_DBGMSG(a)
#endif

#ifdef DEBUG_MA_SMAF_PHRASE_STREAM_CONVERTER	/* SMAF/Phrase Converter */
 #define MAPHRCNV_DBGMSG(a)	printf a
#else
 #define MAPHRCNV_DBGMSG(a)
#endif

#ifdef DEBUG_MA_SMAF_CONVERTER					/* MA SMAF Converter */
 #define MAMMFCNV_DBGMSG(a)	printf a
#else
 #define MAMMFCNV_DBGMSG(a)
#endif

#ifdef DEBUG_MA_SOUND_SEQUENCER					/* MA Sound Sequencer */
 #define MASNDSEQ_DBGMSG(a)	printf a
#else
 #define MASNDSEQ_DBGMSG(a)
#endif

#ifdef DEBUG_MA_SOUND_DRIVER					/* MA Sound Driver */
 #define MASNDDRV_DBGMSG(a)	printf a
#else
 #define MASNDDRV_DBGMSG(a)
#endif

#ifdef DEBUG_MA_RESOURCE_MANAGER				/* MA Resource Manager */
 #define MARESMGR_DBGMSG(a)	printf a
#else
 #define MARESMGR_DBGMSG(a)
#endif

#ifdef DEBUG_MA_DEVICE_DRIVER					/* MA Device Driver */
 #define MADEVDRV_DBGMSG(a)	printf a
#else
 #define MADEVDRV_DBGMSG(a)
#endif

#if MASMW_DEBUG
 void			madebug_Open				( unsigned char * ptr,
 											  unsigned long size );
 unsigned long	madebug_Close				( void );
 void			madebug_SetMode				( unsigned char mode );
 signed long 	madebug_SendDelayedPacket	( const unsigned char *ptr,
 											  unsigned short size );
 signed long 	madebug_SendDirectPacket	( const unsigned char *ptr,
 											  unsigned short size );
 signed long 	madebug_SendDirectRamData	( unsigned long address,
 											  unsigned char data_type,
 											  const unsigned char * data_ptr,
 											  unsigned long data_size );
 signed long 	madebug_SendDirectRamVal	( unsigned long address,
 											  unsigned long data_size,
 											  unsigned char val );
 signed long 	madebug_SetStream			( signed long seq_id,
 											  unsigned char wave_id,
 											  unsigned char format,
 											  unsigned long frequency,
 											  unsigned char * wave_ptr,
 											  unsigned long wave_size );
#endif

#endif /*__MA_DEBUG_H__*/
