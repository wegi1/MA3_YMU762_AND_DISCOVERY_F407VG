/****************************************************************************
 *																			*
 *		Copyright (C) 2001-2003 YAMAHA CORPORATION. All rights reserved.	*
 *																			*
 *		Module		: maresmgr.h											*
 *																			*
 *		Description	: MA Resource Manager									*
 *																			*
 * 		Version		: 1.3.15.3	2003.03.17									*
 *																			*
 ****************************************************************************/

#ifndef __MARESMGR_H__
#define __MARESMGR_H__


#include "mamachdep.h"
#include "madebug.h"
#include "madevdrv.h"
#include "madefs.h"


#define MA_CH_MAP_MASK		(0x0000FFFF)	/* Channel */
#define MA_FM_MAP_MASK		(0xFFFFFFFF)	/* FM voice */
#define MA_WT_MAP_MASK		(0x000000FF)	/* WT voice */
#define MA_RB_MAP_MASK		(0x000000FF)	/* RAM block */
#define MA_TM_MAP_MASK		(0x00000007)	/* Timer */
#define MA_SI_MAP_MASK		(0x0000000F)	/* Software Interrupt */
#define MA_SA_MAP_MASK		(0x00000003)	/* Stream Audio */
#define MA_LD_MAP_MASK		(0x00000001)	/* LED */
#define MA_MT_MAP_MASK		(0x00000001)	/* Motor */
#define MA_SQ_MAP_MASK		(0x00000003)	/* Sequencer */

#define MASMW_MAX_WAVEID	(31)			/* maximum value of wave id */


typedef struct _MA_STREAM
{
	UINT8	format;							/* wave format */
	UINT8 *	wave_ptr;						/* pointer to the wave data */
	UINT32	wave_size;						/* size of the wave data */
	UINT32	seek_pos;						/* seek point */
} MA_STREAM, * PMA_STREAM;

typedef struct _MA_RESOURCE_INFO
{
	UINT32	ch_map;							/* Channle */
	UINT32	fm_map;							/* FM voice */
	UINT32	wt_map;							/* WT voice */
	UINT32	tm_map;							/* Timer */
	UINT32	si_map;							/* Software interrupt */
	UINT32	sa_map;							/* Stream audio */
	UINT32	rb_map;							/* RAM block */
	UINT32	ld_map;							/* Led */
	UINT32	mt_map;							/* Motor */
	UINT32	sq_map;							/* Sequencer */
	MA_STREAM	stream_audio[MA_MAX_REG_STREAM_AUDIO];

} MA_RESOURCE_INFO, * PMA_RESOURCE_INFO;


PMA_RESOURCE_INFO MaResMgr_GetResourceInfo( void );

SINT32	MaResMgr_Initialize			( void );

SINT32	MaResMgr_GetDefWaveAddress	( UINT8 wave_id );
SINT32	MaResMgr_GetDefVoiceAddress	( UINT8 prog );
SINT32	MaResMgr_GetDefVoiceSynth	( UINT8 prog );
SINT32	MaResMgr_GetDefVoiceKey		( UINT8 prog );
SINT32	MaResMgr_GetStreamAudioInfo	( UINT8 wave_id, UINT8 * format, UINT8 ** wave_ptr, UINT32 * wave_size, UINT32 * seek_pos );
SINT32	MaResMgr_RegStreamAudio		( UINT8 wave_id, UINT8 format, UINT8 * wave_ptr, UINT32 wave_size );
SINT32	MaResMgr_DelStreamAudio		( UINT8 wave_id );
SINT32	MaResMgr_SetStreamSeekPos	( UINT8 wave_id, UINT32 seek_pos );

SINT32	MaResMgr_AllocStreamAudio	( UINT32 sa_map );
SINT32	MaResMgr_FreeStreamAudio	( UINT32 sa_map );
SINT32	MaResMgr_AllocCh			( UINT32 ch_map );
SINT32 	MaResMgr_FreeCh				( UINT32 ch_map );
SINT32	MaResMgr_AllocRam			( UINT32 rb_map );
SINT32 	MaResMgr_FreeRam			( UINT32 rb_map );
SINT32 	MaResMgr_AllocFmVoice		( UINT32 fm_map );
SINT32	MaResMgr_FreeFmVoice		( UINT32 fm_map );
SINT32	MaResMgr_AllocWtVoice		( UINT32 wt_map );
SINT32 	MaResMgr_FreeWtVoice		( UINT32 wt_map );
SINT32 	MaResMgr_AllocSoftInt		( UINT32 si_map );
SINT32 	MaResMgr_FreeSoftInt		( UINT32 si_map );
SINT32 	MaResMgr_AllocLed			( UINT32 ld_map );
SINT32 	MaResMgr_FreeLed			( UINT32 ld_map );
SINT32 	MaResMgr_AllocMotor			( UINT32 mt_map );
SINT32 	MaResMgr_FreeMotor			( UINT32 mt_map );
SINT32 	MaResMgr_AllocTimer			( UINT8 timer_id, UINT8 base_time, UINT8 time_count, UINT8 mode, UINT8 one_shot );
SINT32 	MaResMgr_FreeTimer			( UINT8 timer_id );
SINT32	MaResMgr_AllocSequencer		( UINT8 seq_id, UINT16 base_time );
SINT32	MaResMgr_FreeSequencer		( UINT8 seq_id );


#endif /*__MARESMGR_H__*/
