/****************************************************************************
 *																			*
 *		Copyright (C) 2001-2002	YAMAHA CORPORATION. All rights reserved.	*
 *																			*
 *		Module		: macnvprf.h											*
 *																			*
 *		Description	: MA Stream Converter profile							*
 *																			*
 * 		Version		: 1.3.12.3	2002.11.15									*
 *																			*
 ****************************************************************************/

#ifndef __MACNVPRF_H__
#define __MACNVPRF_H__

#if MASMW_SRMCNV_SMAF
 #include "mammfcnv.h"
#endif
#if MASMW_SRMCNV_SMAF_PHRASE
 #include "maphrcnv.h"
#endif
#if MASMW_SRMCNV_REALTIME_MIDI
 #include "marmdcnv.h"
#endif
#if MASMW_SRMCNV_SMF
 #include "mamidcnv.h"
#endif
#if MASMW_SRMCNV_WAV
 #include "mawavcnv.h"
#endif

#define MASMW_CNVID_MMF	1				/* SMAF/MA-1/MA-2/MA-3 */
#define MASMW_CNVID_PHR	2				/* SMAF/Phrase L1 */
#define	MASMW_CNVID_RMD	3				/* Realtime MIDI */
#define	MASMW_CNVID_AUD	4				/* SMAF/Audio */
#define	MASMW_CNVID_MID	5				/* SMF format 0/GM Lite or Level 1 */
#define	MASMW_CNVID_WAV	11				/* Wav */

#define DUMMY_FUNC		\
	{					\
		dummy_Init,		\
		dummy_Load,		\
		dummy_Open,		\
		dummy_Control,	\
		dummy_Standby,	\
		dummy_Seek,		\
		dummy_Start,	\
		dummy_Stop,		\
		dummy_Close,	\
		dummy_Unload,	\
		dummy_End		\
	}

static SINT32 dummy_Init   ( void );
static SINT32 dummy_Load   ( UINT8 * file_ptr, UINT32 file_size, UINT8 mode, SINT32 (*func)(UINT8 id), void * ext_args );
static SINT32 dummy_Open   ( SINT32 file_id, UINT16 open_mode, void * ext_args );
static SINT32 dummy_Control( SINT32 file_id, UINT8 ctrl_num, void * prm, void * ext_args );
static SINT32 dummy_Standby( SINT32 file_id, void * ext_args );
static SINT32 dummy_Seek   ( SINT32 file_id, UINT32 pos, UINT8 flag, void * ext_args );
static SINT32 dummy_Start  ( SINT32 file_id, void * ext_args );
static SINT32 dummy_Stop   ( SINT32 file_id, void * ext_args );
static SINT32 dummy_Close  ( SINT32 file_id, void * ext_args );
static SINT32 dummy_Unload ( SINT32 file_id, void * ext_args );
static SINT32 dummy_End    ( void );

static MASRMCNVFUNC ma_srmcnv_func[MASMW_NUM_SRMCNV] =
{
	DUMMY_FUNC,							/* 0 */
#if MASMW_SRMCNV_SMAF
	{									/* 1 */
		MaMmfCnv_Initialize,
		MaMmfCnv_Load,
		MaMmfCnv_Open,
		MaMmfCnv_Control,
		MaMmfCnv_Standby,
		MaMmfCnv_Seek,
		MaMmfCnv_Start,
		MaMmfCnv_Stop,
		MaMmfCnv_Close,
		MaMmfCnv_Unload,
		MaMmfCnv_End
	},
#else
	DUMMY_FUNC,
#endif
	DUMMY_FUNC,							/* 2 */
#if MASMW_SRMCNV_REALTIME_MIDI
	{									/* 3 */
		MaRmdCnv_Initialize,
		MaRmdCnv_Load,
		MaRmdCnv_Open,
		MaRmdCnv_Control,
		MaRmdCnv_Standby,
		MaRmdCnv_Seek,
		MaRmdCnv_Start,
		MaRmdCnv_Stop,
		MaRmdCnv_Close,
		MaRmdCnv_Unload,
		MaRmdCnv_End
	},
#else
	DUMMY_FUNC,
#endif
#if MASMW_SRMCNV_SMAF_AUDIO
	{									/* 4 */
		MaAudCnv_Initialize,
		MaAudCnv_Load,
		MaAudCnv_Open,
		MaAudCnv_Control,
		MaAudCnv_Standby,
		MaAudCnv_Seek,
		MaAudCnv_Start,
		MaAudCnv_Stop,
		MaAudCnv_Close,
		MaAudCnv_Unload,
		MaAudCnv_End
	},
#else
	DUMMY_FUNC,
#endif
#if MASMW_SRMCNV_SMF
	{									/* 5 */
		MaMidCnv_Initialize,
		MaMidCnv_Load,
		MaMidCnv_Open,
		MaMidCnv_Control,
		MaMidCnv_Standby,
		MaMidCnv_Seek,
		MaMidCnv_Start,
		MaMidCnv_Stop,
		MaMidCnv_Close,
		MaMidCnv_Unload,
		MaMidCnv_End
	},
#else
	DUMMY_FUNC,
#endif
	DUMMY_FUNC,							/* 6 */
	DUMMY_FUNC,							/* 7 */
	DUMMY_FUNC,							/* 8 */
	DUMMY_FUNC,							/* 9 */
	DUMMY_FUNC,							/* 10 */
#if MASMW_SRMCNV_WAV
	{									/* 11 */
		MaWavCnv_Initialize,
		MaWavCnv_Load,
		MaWavCnv_Open,
		MaWavCnv_Control,
		MaWavCnv_Standby,
		MaWavCnv_Seek,
		MaWavCnv_Start,
		MaWavCnv_Stop,
		MaWavCnv_Close,
		MaWavCnv_Unload,
		MaWavCnv_End
	},
#else
	DUMMY_FUNC,
#endif
};

#endif /*__MACNVPRF_H__*/
