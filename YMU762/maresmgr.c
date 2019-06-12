/****************************************************************************
 *																			*
 *		Copyright (C) 2001-2003 YAMAHA CORPORATION. All rights reserved.	*
 *																			*
 *		Module		: maresmgr.c											*
 *																			*
 *		Description	: MA Resource Manager									*
 *																			*
 * 		Version		: 1.3.15.3	2003.03.17									*
 *																			*
 ****************************************************************************/

#include "maresmgr.h"					/* MA Resource Manager */

/* Resource Information */
static MA_RESOURCE_INFO ma_resource_info;

static const UINT16 ma_rom_drum_type[128/16] =
{
					/*           F E D C  B A 9 8  7 6 5 4  3 2 1 0  */
					/* --------+------------------------------------ */
	0x0000,			/*   0- 15 | 0 0 0 0  0 0 0 0  0 0 0 0  0 0 0 0  */
	0x8000,			/*  16- 31 | 1 0 0 0  0 0 0 0  0 0 0 0  0 0 0 0  */
	0xFF5A,			/*  32- 47 | 1 1 1 1  1 1 1 1  0 1 0 1  1 0 1 0  */
	0x0A9F,			/*  48- 63 | 0 0 0 0  1 0 1 0  1 0 0 1  1 1 1 1  */
	0x0000,			/*  64- 79 | 0 0 0 0  0 0 0 0  0 0 0 0  0 0 0 0  */
	0x0000,			/*  80- 95 | 0 0 0 0  0 0 0 0  0 0 0 0  0 0 0 0  */
	0x0000,			/*  96-111 | 0 0 0 0  0 0 0 0  0 0 0 0  0 0 0 0  */
	0x0000,			/* 112-127 | 0 0 0 0  0 0 0 0  0 0 0 0  0 0 0 0  */
};

static const UINT16 ma_rom_wave_address[MA_MAX_ROM_WAVE] =
	{ 0x1400, 0x15D6, 0x1BA4, 0x2288, 0x24F4, 0x2B72, 0x34D2 };

static const UINT16 ma_rom_drum_key[61] =
{
	0x0058, 0x001D, 0x003C, 0x0024, 0x0018, 0x0012, 0x0057, 0x0155, 
	0x005E, 0x00C0, 0x0027, 0x00D5, 0x0100, 0x004A, 0x0195, 0x0063, 
	0x01EA, 0x00AA, 0x0140, 0x00D5, 0x012A, 0x010A, 0x014A, 0x0140, 
	0x0180, 0x0119, 0x01C0, 0x0155, 0x00C0, 0x006F, 0x006D, 0x01C0, 
	0x0054, 0x012A, 0x001A, 0x0180, 0x003E, 0x003D, 0x004C, 0x0047, 
	0x0039, 0x0040, 0x003B, 0x004D, 0x0048, 0x0064, 0x006D, 0x0030, 
	0x002C, 0x0031, 0x0030, 0x0045, 0x0049, 0x0044, 0x0050, 0x0010, 
	0x0058, 0x0058, 0x005E, 0x004C, 0x003E
};

/****************************************************************************
 *	MaResMgr_GetResourceInfo
 *
 *	Function:
 *			Gets the currenet mapping of resources.
 *	Argument:
 *			None
 *	Return:
 *			Pointer to the resource information structure.
 *
 ****************************************************************************/
PMA_RESOURCE_INFO MaResMgr_GetResourceInfo( void )
{
	MARESMGR_DBGMSG(("   MaResMgr_GetResourceInfo:\n"));
	
	return &ma_resource_info;
}
/****************************************************************************
 *	MaResMgr_GetDefWaveAddress
 *
 *	Function:
 *			Get internal memory address of specified wave data.
 *	Argument:
 *			wave_id	wave id number (0..6)
 *	Return:
 *			> 0		internal memory address of specified wave data.
 *			0		no entry
 *			< 0		error code
 *
 ****************************************************************************/
SINT32 MaResMgr_GetDefWaveAddress
(
	UINT8	wave_id						/* wave id */
)
{
	SINT32	result;
	
	MARESMGR_DBGMSG(("   MaResMgr_GetDefWaveAddress: wave_id=%d ->", wave_id));

	if ( wave_id >= MA_MAX_ROM_WAVE ) return MASMW_ERROR_ARGUMENT;
	
	result = (SINT32)ma_rom_wave_address[wave_id];

	MARESMGR_DBGMSG((" %04lX\n", result));

	return result;
}
/****************************************************************************
 *	MaResMgr_GetDefVoiceAddress
 *
 *	Function:
 *			Get internal memory address of specified timber.
 *	Argument:
 *			prog	program number(0..127: melody timbers, 128..255: drum timbers)
 *	Return:
 *			> 0		internal memory address of specified timber.
 *			0		no entry
 *			< 0		error code
 *
 ****************************************************************************/
SINT32 MaResMgr_GetDefVoiceAddress
(
	UINT8	prog						/* program number */
)
{
	SINT32	result;
	
	MARESMGR_DBGMSG(("   MaResMgr_GetDefVoiceAddress: prog=%d ->", prog));
	
	if ( prog < 128 )
	{
		/* melody */
		result = (SINT32)( MA_NORMAL_ROM_ADDRESS + ( prog * 16 ) );
	}
	else
	{
		prog -= 128;

		/* drum */
		if ( ( prog < MA_MIN_ROM_DRUM ) || ( MA_MAX_ROM_DRUM < prog ) )
		{
			result = (SINT32)(0);
		}
		else
		{
			result = (SINT32)( MA_DRUM_ROM_ADDRESS + ( ( prog - MA_MIN_ROM_DRUM ) * 16 ) );
		}
	}
	
	MARESMGR_DBGMSG((" %04lX\n", result));

	return result;
}
/****************************************************************************
 *	MaResMgr_GetDefVoiceSynth
 *
 *	Function:
 *			Get the voice type of specified timber.
 *	Argument:
 *			prog	program number(0..127: melody timbers, 128..255: drum timbers)
 *	Return:
 *			> 0		voice type of specified timber (1:FM, 2:WT).
 *			0		no entry
 *			< 0		error code
 *
 ****************************************************************************/
SINT32 MaResMgr_GetDefVoiceSynth
(
	UINT8	prog						/* program number */
)
{
	SINT32	result;
	
	MARESMGR_DBGMSG(("   MaResMgr_GetDefVoiceSynth: prog=%d ->", prog));
	
	if ( prog < 128 )
	{
		result = (SINT32)(1);
	}
	else
	{
		prog -= 128;

		if ( ( prog < MA_MIN_ROM_DRUM ) || ( MA_MAX_ROM_DRUM < prog ) )
		{
			result = (SINT32)(0);
		}
		else
		{
			result = (SINT32)( ( ( ma_rom_drum_type[prog/16] >> (prog%16) ) & 0x1 ) + 1 );
		}
	}
	
	MARESMGR_DBGMSG((" %lu\n", result));
	
	return result;
}
/****************************************************************************
 *	MaResMgr_GetDefVoiceKey
 *
 *	Function:
 *			Get the key(block:f-number) of specified timber.
 *	Argument:
 *			prog	program number(0..127: melody timbers, 128..255: drum timbers)
 *	Return:
 *			> 0		internal memory address of specified timber.
 *			0		no entry
 *			< 0		error code
 *
 ****************************************************************************/
SINT32 MaResMgr_GetDefVoiceKey
(
	UINT8	prog						/* program number */
)
{
	SINT32	result;
	
	MARESMGR_DBGMSG(("   MaResMgr_GetDefVoiceKey: prog=%d ->", prog));

	if ( prog < 128 )
	{
		result = (SINT32)MASMW_ERROR;
	}
	else
	{
		prog -= 128;

		if ( ( prog < MA_MIN_ROM_DRUM ) || ( MA_MAX_ROM_DRUM < prog ) )
		{
			result = (SINT32)(0);
		}
		else
		{
			result = (SINT32)ma_rom_drum_key[prog - MA_MIN_ROM_DRUM];
		}
	}

	MARESMGR_DBGMSG((" %ld\n", result));
	
	return result;
}
/****************************************************************************
 *	MaResMgr_RegStreamAudio
 *
 *	Function:
 *			Regist the wave data for stream audio.
 *	Argument:
 *			wave_id		wave id
 *			format		wave format
 *			wave_ptr	pointer to the wave data for stream audio
 *			wave_size	size of the wave data for stream audio
 *	Return:
 *			0			success
 *			< 0			error codes
 *
 ****************************************************************************/
SINT32 MaResMgr_RegStreamAudio
(
	UINT8	wave_id,					/* wave id */
	UINT8	format,						/* wave format */
	UINT8 *	wave_ptr,					/* pointer to the wave data */
	UINT32	wave_size					/* size of the wave data */
)
{
	MARESMGR_DBGMSG(("   MaResMgr_RegStreamAudio: id=%d ptr=%p sz=%ld\n", wave_id, wave_ptr, wave_size));

	/* check arugment */

	if ( wave_id > MASMW_MAX_WAVEID )	return MASMW_ERROR_ARGUMENT;
	if ( wave_ptr == NULL ) 			return MASMW_ERROR_ARGUMENT;
	if ( wave_size == 0 )				return MASMW_ERROR_ARGUMENT;
	
	ma_resource_info.stream_audio[wave_id].format	 = format;
	ma_resource_info.stream_audio[wave_id].wave_ptr	 = wave_ptr;
	ma_resource_info.stream_audio[wave_id].wave_size = wave_size;
	ma_resource_info.stream_audio[wave_id].seek_pos  = (UINT32)0;

	return MASMW_SUCCESS;
}
/****************************************************************************
 *	MaResMgr_DelStreamAudio
 *
 *	Function:
 *			Delete the wave data for stream audio.
 *	Argument:
 *			wave_id		wave id
 *	Return:
 *			0		success
 *			< 0		error codes
 *
 ****************************************************************************/
SINT32 MaResMgr_DelStreamAudio
(
	UINT8	wave_id						/* wave id */
)
{
	MARESMGR_DBGMSG(("   MaResMgr_DelStreamAudio: id=%d\n", wave_id));

	/* check arugment */
	if ( wave_id > MASMW_MAX_WAVEID )	return MASMW_ERROR_ARGUMENT;

	ma_resource_info.stream_audio[wave_id].wave_ptr = NULL;
	ma_resource_info.stream_audio[wave_id].wave_size = 0;

	return MASMW_SUCCESS;
}
/****************************************************************************
 *	MaResMgr_SetStreamSeekPos
 *
 *	Function:
 *			Set seek point of stream.
 *	Argument:
 *			wave_id		wave id (0..31)
 *			seek_pos	seek point (byte)
 *	Return:
 *			0			success
 *			< 0			error codes
 *
 ****************************************************************************/
SINT32 MaResMgr_SetStreamSeekPos
(
	UINT8	wave_id,					/* wave id */
	UINT32	seek_pos					/* seek point */
)
{
	MARESMGR_DBGMSG(("   MaResMgr_SetStreamSeekPos: id=%d pos=%ld\n", wave_id, seek_pos));

	/* check arguments */

	if ( wave_id > MASMW_MAX_WAVEID )	return MASMW_ERROR_ARGUMENT;

	if ( seek_pos > ma_resource_info.stream_audio[wave_id].wave_size )
	{
		return MASMW_ERROR_ARGUMENT;
	}
	
	/* regist seek point */
	ma_resource_info.stream_audio[wave_id].seek_pos = seek_pos;

	return MASMW_SUCCESS;
}
/****************************************************************************
 *	MaResMgr_GetStreamAudioInfo
 *
 *	Function:
 *			return speciefied stream audio information
 *	Argument:
 *			wave_id		wave id 
 *			format		wave format
 *			wave_ptr	pointer to the wave data for stream audio
 *			wave_size	size of the wave data for stream audio
 *			seek_pos	seek point of stream
 *	Return:
 *			0		success
 *			< 0		error codes
 *
 ****************************************************************************/
SINT32 MaResMgr_GetStreamAudioInfo
(
	UINT8		wave_id,				/* wave id */
	UINT8 *		format,					/* format */
	UINT8 **	wave_ptr,				/* pointer to the wave data */
	UINT32 *	wave_size,				/* size of the wave data */
	UINT32 *	seek_pos				/* seek point */
)
{
	MARESMGR_DBGMSG(("   MaResMgr_GetStreamAudioInfo: wave_id=%d\n", wave_id));
	
	if ( wave_id > MASMW_MAX_WAVEID )	return MASMW_ERROR_ARGUMENT;
	
	*format	   = ma_resource_info.stream_audio[wave_id].format;
	*wave_ptr  = ma_resource_info.stream_audio[wave_id].wave_ptr;
	*wave_size = ma_resource_info.stream_audio[wave_id].wave_size;
	*seek_pos  = ma_resource_info.stream_audio[wave_id].seek_pos;
	
	/* reset seek point */
	ma_resource_info.stream_audio[wave_id].seek_pos = 0;

	return MASMW_SUCCESS;
}
/****************************************************************************
 *	MaResMgr_AllocStreamAudio
 *
 *	Function:
 *			Allocate the specified stream audio resources.
 *	Argument:
 *			sa_map	bit mapping of Stream Audio resources to allocate.
 *	Return:
 *			0		success
 *			< 0		error code
 *
 ****************************************************************************/
SINT32 MaResMgr_AllocStreamAudio
(
	UINT32	sa_map						/* stream audio id */
)
{
	SINT32	result;
	static UINT32 res_map[4] = { 0x00000000, 0x00000080, 0x00000060, 0x000000C0 };
	
	MARESMGR_DBGMSG(("    MaResMgr_AllocStreamAudio: sa_map=%08lX\n", sa_map));

	if ( sa_map == 0x00000000 ) return MASMW_SUCCESS;

	if ( ( sa_map & MA_SA_MAP_MASK ) != sa_map )
	{
		MARESMGR_DBGMSG(("Illegal sa_map value.\n"));
		return MASMW_ERROR_ARGUMENT;
	}

	if ( ( ma_resource_info.sa_map & sa_map ) != 0 )
	{
		MARESMGR_DBGMSG(("can't alloc stream audio\n"));
		return MASMW_ERROR_RESOURCE_OVER;
	}

	result = MaResMgr_AllocWtVoice( res_map[sa_map] );
	if ( result != MASMW_SUCCESS ) return result;

	result = MaResMgr_AllocRam( res_map[sa_map] );
	if ( result != MASMW_SUCCESS )
	{
		MaResMgr_FreeWtVoice( res_map[sa_map] );
		return result;
	}

	result = MaResMgr_AllocSoftInt( sa_map );
	if ( result != MASMW_SUCCESS )
	{
		MaResMgr_FreeWtVoice( res_map[sa_map] );
		MaResMgr_FreeRam( res_map[sa_map] );
		return result;
	}

	result = MaResMgr_AllocTimer( 1, 0x1B, 20, 0, 0 );	/* 0.999 * 20 [ms] */
	if ( result != MASMW_SUCCESS )
	{
		MaResMgr_FreeWtVoice( res_map[sa_map] );
		MaResMgr_FreeRam( res_map[sa_map] );
		MaResMgr_FreeSoftInt( sa_map );
		return result;
	}

	ma_resource_info.sa_map |= sa_map;

	MARESMGR_DBGMSG((" -> sa_map=%08lX\n", ma_resource_info.sa_map));

	return MASMW_SUCCESS;
}
/****************************************************************************
 *	MaResMgr_FreeStreamAudio
 *
 *	Function:
 *			Free the specified stream audio resources.
 *	Argument:
 *			sa_id	stream audio id number
 *	Return:
 *			0		success
 *			< 0		error codes
 *
 ****************************************************************************/
SINT32 MaResMgr_FreeStreamAudio
(
	UINT32	sa_map						/* stream audio map */
)
{
	SINT32	result;
	static UINT32 res_map[4] = { 0x00000000, 0x00000080, 0x00000060, 0x000000C0 };

	MARESMGR_DBGMSG(("    MaResMgr_FreeStreamAudio : sa_map=%08lX\n", sa_map));

	if ( sa_map == 0x00000000 ) return MASMW_SUCCESS;

	if ( ( sa_map & MA_SA_MAP_MASK ) != sa_map )
	{
		MARESMGR_DBGMSG(("illegal stream audio id\n"));
		return MASMW_ERROR_ARGUMENT;
	}

	if ( ( ma_resource_info.sa_map & sa_map ) != sa_map )
	{
		MARESMGR_DBGMSG(("can't free sa\n"));
		return MASMW_ERROR;
	}

	result = MaResMgr_FreeWtVoice( res_map[sa_map] );
	if ( result != MASMW_SUCCESS ) return result;
	
	result = MaResMgr_FreeRam( res_map[sa_map] );
	if ( result != MASMW_SUCCESS ) return result;

	result = MaResMgr_FreeSoftInt( sa_map );
	if ( result != MASMW_SUCCESS ) return result;

	result = MaResMgr_FreeTimer( 1 );
	if ( result != MASMW_SUCCESS ) return result;

	ma_resource_info.sa_map ^= sa_map;

	MARESMGR_DBGMSG((" -> sa_map=%08lX\n", ma_resource_info.sa_map));

	return MASMW_SUCCESS;
}
/****************************************************************************
 *	MaResMgr_AllocRam
 *
 *	Function:
 *			Allocate the specified RAM block resources.
 *	Argument:
 *			rb_map		bit mapping of RAM block resources to allocate.
 *			ram_adrs	return the RAM address of each blocks.
 *	Return:
 *			0		success
 *			< 0		error codes
 *
 ****************************************************************************/
SINT32 MaResMgr_AllocRam
(
	UINT32		rb_map					/* ram block map */
)
{
	MARESMGR_DBGMSG(("    MaResMgr_AllocRam        : rb_map=%08lX", rb_map));

	if ( ( rb_map & MA_RB_MAP_MASK ) != rb_map )
	{
		MARESMGR_DBGMSG(("illegal rb value\n"));
		return MASMW_ERROR_ARGUMENT;
	}
	
	if ( ( ma_resource_info.rb_map & rb_map ) != 0 )
	{
		MARESMGR_DBGMSG(("can't alloc rb\n"));
		return MASMW_ERROR_RESOURCE_OVER;
	}

	ma_resource_info.rb_map |= rb_map;

	MARESMGR_DBGMSG((" -> rb_map=%08lX\n", ma_resource_info.rb_map));

	return MASMW_SUCCESS;
}
/****************************************************************************
 *	MaResMgr_FreeRam
 *
 *	Function:
 *			Free RAM block resources.
 *	Argument:
 *			rb_map	bit mapping of RAM block resources to free
 *	Return:
 *			0		success
 *			< 0		error codes
 *
 ****************************************************************************/
SINT32 MaResMgr_FreeRam
(
	UINT32	rb_map						/* ram block map */
)
{
	MARESMGR_DBGMSG(("    MaResMgr_FreeRam         : rb_map=%08lX", rb_map));

	if ( ( rb_map & MA_RB_MAP_MASK ) != rb_map )
	{
		MARESMGR_DBGMSG(("illegal rb value\n"));
		return MASMW_ERROR_ARGUMENT;
	}
	
	if ( ( ma_resource_info.rb_map & rb_map ) != rb_map )
	{
		MARESMGR_DBGMSG(("can't free rb\n"));
		return MASMW_ERROR;
	}

	ma_resource_info.rb_map ^= rb_map;

	MARESMGR_DBGMSG((" -> rb_map=%08lX\n", ma_resource_info.rb_map));

	return MASMW_SUCCESS;
}
/****************************************************************************
 *	MaResMgr_AllocCh
 *
 *	Function:
 *			Allocate the channel resources.
 *	Argument:
 *			ch_map	bit mapping of channel resources to allocate.
 *	Return:
 *			0		success
 *			< 0		error codes
 *
 ****************************************************************************/
SINT32 MaResMgr_AllocCh
(
	UINT32	ch_map						/* channel map */
)
{
	MARESMGR_DBGMSG(("    MaResMgr_AllocCh         : ch_map=%08lX", ch_map));

	if ( ( ch_map & MA_CH_MAP_MASK ) != ch_map )
	{
		MARESMGR_DBGMSG((" illegal ch value\n"));
		return MASMW_ERROR_ARGUMENT;
	}
	
	if ( ( ma_resource_info.ch_map & ch_map ) != 0 )
	{
		MARESMGR_DBGMSG((" can't alloc ch\n"));
		return MASMW_ERROR_RESOURCE_OVER;
	}

	ma_resource_info.ch_map |= ch_map;

	MARESMGR_DBGMSG((" -> ch_map=%08lX\n", ma_resource_info.ch_map));

	return MASMW_SUCCESS;
}
/****************************************************************************
 *	MaResMgr_FreeCh
 *
 *	Function:
 *			Free the specified channel resources.
 *	Argument:
 *			ch_map	bit mapping of channel resources to free.
 *	Return:
 *			0		success
 *			< 0		error codes
 *
 ****************************************************************************/
SINT32 MaResMgr_FreeCh
(
	UINT32	ch_map						/* channel map */
)
{
	MARESMGR_DBGMSG(("    MaResMgr_FreeCh          : ch_map=%08lX", ch_map));

	if ( ( ch_map & MA_CH_MAP_MASK ) != ch_map )
	{
		MARESMGR_DBGMSG(("illegal ch value\n"));
		return MASMW_ERROR_ARGUMENT;
	}
	
	if ( ( ma_resource_info.ch_map & ch_map ) != ch_map )
	{
		MARESMGR_DBGMSG(("can't free ch\n"));
		return MASMW_ERROR;
	}

	ma_resource_info.ch_map ^= ch_map;

	MARESMGR_DBGMSG((" -> ch_map=%08lX\n", ma_resource_info.ch_map));

	return MASMW_SUCCESS;
}
/****************************************************************************
 *	MaResMgr_AllocFmVoice
 *
 *	Function:
 *			Allocate the FM voice resources.
 *	Argument:
 *			fm_map	bit mapping of FM voice resources to allocate
 *	Return:
 *			0		success
 *			< 0		error codes
 *
 ****************************************************************************/
SINT32 MaResMgr_AllocFmVoice
(
	UINT32	fm_map						/* FM voice map */
)
{
	MARESMGR_DBGMSG(("    MaResMgr_AllocFmVoice    : fm_map=%08lX", fm_map));

	if ( ( fm_map & MA_FM_MAP_MASK ) != fm_map )
	{
		MARESMGR_DBGMSG(("illegal fm voice value\n"));
		return MASMW_ERROR_ARGUMENT;
	}
	
	if ( ( ma_resource_info.fm_map & fm_map ) != 0 )
	{
		MARESMGR_DBGMSG(("can't alloc fm voice\n"));
		return MASMW_ERROR_RESOURCE_OVER;
	}

	ma_resource_info.fm_map |= fm_map;

	MARESMGR_DBGMSG((" -> fm_map=%08lX\n", ma_resource_info.fm_map));

	return MASMW_SUCCESS;
}
/****************************************************************************
 *	MaResMgr_FreeFmVoice
 *
 *	Function:
 *			Free the specified FM voice resources.
 *	Argument:
 *			fm_map	bit mapping of FM voice resources to free
 *	Return:
 *			0		success
 *			< 0		error codes
 *
 ****************************************************************************/
SINT32 MaResMgr_FreeFmVoice
(
	UINT32	fm_map						/* FM voice map */
)
{
	MARESMGR_DBGMSG(("    MaResMgr_FreeFmVoice     : fm_map=%08lX", fm_map));

	if ( ( fm_map & MA_FM_MAP_MASK ) != fm_map )
	{
		MARESMGR_DBGMSG(("illegal fm voice value\n"));
		return MASMW_ERROR_ARGUMENT;
	}
	
	if ( ( ma_resource_info.fm_map & fm_map ) != fm_map )
	{
		MARESMGR_DBGMSG(("can't free fm map\n"));
		return MASMW_ERROR;
	}

	ma_resource_info.fm_map ^= fm_map;

	MARESMGR_DBGMSG((" -> fm_map=%08lX\n", ma_resource_info.fm_map));

	return MASMW_SUCCESS;
}
/****************************************************************************
 *	MaResMgr_AllocWtVoice
 *
 *	Function:
 *			Allocate the specified WT voice resources.
 *	Argument:
 *			wt_map	bit mapping of WT voice resources to allocate.
 *	Return:
 *			0		success
 *			< 0		error codes
 *
 ****************************************************************************/
SINT32 MaResMgr_AllocWtVoice
(
	UINT32	wt_map						/* WT voice map */
)
{
	MARESMGR_DBGMSG(("    MaResMgr_AllocWtVoice    : wt_map=%08lX", wt_map));

	if ( ( wt_map & MA_WT_MAP_MASK ) != wt_map )
	{
		MARESMGR_DBGMSG(("illegal wt voice value\n"));
		return MASMW_ERROR_ARGUMENT;
	}
	
	if ( ( ma_resource_info.wt_map & wt_map ) != 0 )
	{
		MARESMGR_DBGMSG(("can't alloc wt voice\n"));
		return MASMW_ERROR_RESOURCE_OVER;
	}

	ma_resource_info.wt_map |= wt_map;

	MARESMGR_DBGMSG((" -> wt_map=%08lX\n", ma_resource_info.wt_map));

	return MASMW_SUCCESS;
}
/****************************************************************************
 *	MaResMgr_FreeWtVoice
 *
 *	Function:
 *			Free the specified WT voice resources.
 *	Argument:
 *			wt_map	bit mapping of WT voice resources to free.
 *	Return:
 *			0		success
 *			< 0		error codes
 *
 ****************************************************************************/
SINT32 MaResMgr_FreeWtVoice
(
	UINT32	wt_map						/* WT voice map */
)
{
	MARESMGR_DBGMSG(("    MaResMgr_FreeWtVoice     : wt_map=%08lX", wt_map));

	if ( ( wt_map & MA_WT_MAP_MASK ) != wt_map )
	{
		MARESMGR_DBGMSG(("illegal wt voice value\n"));
		return MASMW_ERROR_ARGUMENT;
	}
	
	if ( ( ma_resource_info.wt_map & wt_map ) != wt_map )
	{
		MARESMGR_DBGMSG(("can't free wt map\n"));
		return MASMW_ERROR;
	}

	ma_resource_info.wt_map ^= wt_map;

	MARESMGR_DBGMSG((" -> wt_map=%08lX\n", ma_resource_info.wt_map));

	return MASMW_SUCCESS;
}
/****************************************************************************
 *	MaResMgr_AllocSoftInt
 *
 *	Function:
 *			Allocate the specified software interrupt.
 *	Argument:
 *			si_map	bit mapping of software interrupt resources to allocate.
 *	Return:
 *			0		success
 *			< 0		error code
 *
 ****************************************************************************/
SINT32 MaResMgr_AllocSoftInt
(
	UINT32	si_map						/* software interrupt map */
)
{
	MARESMGR_DBGMSG(("    MaResMgr_AllocSoftInt    : si_map=%08lX", si_map));

	if ( ( si_map & MA_SI_MAP_MASK ) != si_map )
	{
		MARESMGR_DBGMSG(("illegal soft int value\n"));
		return MASMW_ERROR_ARGUMENT;
	}
	
	if ( ( ma_resource_info.si_map & si_map ) != 0 )
	{
		MARESMGR_DBGMSG(("can't alloc soft int\n"));
		return MASMW_ERROR_RESOURCE_OVER;
	}

	ma_resource_info.si_map |= si_map;

	MARESMGR_DBGMSG((" -> si_map=%08lX\n", ma_resource_info.si_map));

	return MASMW_SUCCESS;
}
/****************************************************************************
 *	MaResMgr_FreeSoftInt
 *
 *	Function:
 *			Free the specified software interrupt.
 *			It is possible to specify some software interrupts at once.
 *	Argument:
 *			si_map	bit mapping of software interrupt resources to free.
 *	Return:
 *			0		success
 *			< 0		error code
 *
 ****************************************************************************/
SINT32 MaResMgr_FreeSoftInt
(
	UINT32	si_map						/* software interrupt map */
)
{
	MARESMGR_DBGMSG(("    MaResMgr_FreeSoftInt     : si_map=%08lX", si_map));

	if ( ( si_map & MA_SI_MAP_MASK ) != si_map )
	{
		MARESMGR_DBGMSG(("illegal soft int value\n"));
		return MASMW_ERROR_ARGUMENT;
	}

	if ( ( ma_resource_info.si_map & si_map ) != si_map )
	{
		MARESMGR_DBGMSG(("can't free soft int map\n"));
		return MASMW_ERROR;
	}

	ma_resource_info.si_map ^= si_map;

	MARESMGR_DBGMSG((" -> si_map=%08lX\n", ma_resource_info.si_map));

	return MASMW_SUCCESS;
}
/****************************************************************************
 *	MaResMgr_AllocLed
 *
 *	Function:
 *			Allocate the specified LED.
 *	Argument:
 *			ld_map	bit mapping of LED resources to allocate.
 *	Return:
 *			0		success
 *			< 0		error codes
 *
 ****************************************************************************/
SINT32 MaResMgr_AllocLed
(
	UINT32	ld_map						/* led map */
)
{
	MARESMGR_DBGMSG(("    MaResMgr_AllocLed        : ld_map=%08lX", ld_map));

	if ( ( ld_map & MA_LD_MAP_MASK ) != ld_map )
	{
		MARESMGR_DBGMSG(("illegal led number\n"));
		return MASMW_ERROR_ARGUMENT;
	}

	if ( ( ma_resource_info.ld_map & ld_map ) != 0 )
	{
		MARESMGR_DBGMSG(("can't alloc led\n"));
		return MASMW_ERROR_RESOURCE_OVER;
	}

	ma_resource_info.ld_map |= ld_map;

	MARESMGR_DBGMSG((" -> ld_map=%08lX\n", ma_resource_info.ld_map));

	return MASMW_SUCCESS;
}
/****************************************************************************
 *	MaResMgr_FreeLed
 *
 *	Function:
 *			Free the specified LED.
 *	Argument:
 *			ld_map	bit mapping of LED resources to free.
 *	Return:
 *			0		success
 *			< 0		error codes
 *
 ****************************************************************************/
SINT32 MaResMgr_FreeLed
(
	UINT32	ld_map						/* led map */
)
{
	MARESMGR_DBGMSG(("    MaResMgr_FreeLed         : ld_map=%08lX", ld_map));

	if ( ( ld_map & MA_LD_MAP_MASK ) != ld_map )
	{
		MARESMGR_DBGMSG(("illegal led number\n"));
		return MASMW_ERROR_ARGUMENT;
	}
	
	if ( ( ma_resource_info.ld_map & ld_map ) != ld_map )
	{
		MARESMGR_DBGMSG(("can't free led\n"));
		return MASMW_ERROR;
	}

	ma_resource_info.ld_map ^= ld_map;

	MARESMGR_DBGMSG((" -> ld_map=%08lX\n", ma_resource_info.ld_map));

	return MASMW_SUCCESS;
}
/****************************************************************************
 *	MaResMgr_AllocMotor
 *
 *	Function:
 *			Allocate the specified motor.
 *	Argument:
 *			mt_map	bit mapping of motor resources to allocate.
 *	Return:
 *			0		success
 *			< 0		error codes
 *
 ****************************************************************************/
SINT32 MaResMgr_AllocMotor
(
	UINT32	mt_map						/* mtr map */
)
{
	MARESMGR_DBGMSG(("    MaResMgr_AllocMotor      : mt_map=%08lX", mt_map));

	if ( ( mt_map & MA_MT_MAP_MASK ) != mt_map )
	{
		MARESMGR_DBGMSG(("illegal motor number\n"));
		return MASMW_ERROR_ARGUMENT;
	}
	
	if ( ( ma_resource_info.mt_map & mt_map ) != 0 )
	{
		MARESMGR_DBGMSG(("can't alloc motor\n"));
		return MASMW_ERROR_RESOURCE_OVER;
	}

	ma_resource_info.mt_map |= mt_map;

	MARESMGR_DBGMSG((" -> mt_map=%08lX\n", ma_resource_info.mt_map));

	return MASMW_SUCCESS;
}
/****************************************************************************
 *	MaResMgr_FreeMotor
 *
 *	Function:
 *			Free the specified motor.
 *	Argument:
 *			mt_map	bit mapping of motor resources to free.
 *	Return:
 *			0		success
 *			< 0		error codes
 *
 ****************************************************************************/
SINT32 MaResMgr_FreeMotor
(
	UINT32	mt_map						/* mtr map */
)
{
	MARESMGR_DBGMSG(("    MaResMgr_FreeMotor       : mt_map=%08lX", mt_map));

	if ( ( mt_map & MA_MT_MAP_MASK ) != mt_map )
	{
		MARESMGR_DBGMSG(("illegal motor number\n"));
		return MASMW_ERROR_ARGUMENT;
	}
	
	if ( ( ma_resource_info.mt_map & mt_map ) != mt_map )
	{
		MARESMGR_DBGMSG(("can't free mtr\n"));
		return MASMW_ERROR;
	}

	ma_resource_info.mt_map ^= mt_map;

	MARESMGR_DBGMSG((" -> mt_map=%08lX\n", ma_resource_info.mt_map));

	return MASMW_SUCCESS;
}
/****************************************************************************
 *	MaResMgr_AllocTimer
 *
 *	Function:
 *			Allocate the specified timer of.
 *	Arguments:
 *			timer_id	timer id (0..2)
 *			base_time	base time value (0..127)
 *			time		timer count value (0..127)
 *			mode		
 *			one_shot	0: continuous, 1: one shot
 *	Return:
 *			0		success
 *			< 0		error codes
 *
 ****************************************************************************/
SINT32 MaResMgr_AllocTimer
(
	UINT8	timer_id,					/* timer id */
	UINT8	base_time,					/* base time */
	UINT8	time_count,					/* timer count value */
	UINT8	mode,						/* mode */
	UINT8	one_shot					/* one shot mode */
)
{
	UINT8	packet[7];
	UINT8	num = 0;
	UINT8	sw;
	UINT32	tm_map;
	SINT32	result;
	
	MARESMGR_DBGMSG(("    MaResMgr_AllocTimer      : id=%d bt=%d tm=%d md=%d os=%d", timer_id, base_time, time_count, mode, one_shot));

	if ( timer_id > 3 ) return MASMW_ERROR_ARGUMENT;
	if ( one_shot > 1 ) return MASMW_ERROR_ARGUMENT;
	if ( mode > 1 ) 	return MASMW_ERROR_ARGUMENT;

	tm_map = (UINT32)( 1 << timer_id );

	if ( ( tm_map & MA_TM_MAP_MASK ) != tm_map )
	{
		MARESMGR_DBGMSG(("illegal timer value\n"));
		return MASMW_ERROR_ARGUMENT;
	}
	
	if ( ( ma_resource_info.tm_map & tm_map ) != 0 )
	{
		MARESMGR_DBGMSG(("can't alloc timer\n"));
		return MASMW_ERROR_RESOURCE_OVER;
	}

	if ( timer_id < 2 )						/* Timer #0 */
	{
		if ( ( ma_resource_info.tm_map & 0x03 ) == 0 )
		{
			packet[num++] = (UINT8)(  MA_TIMER_MS           & 0x7F );	/* address */
			packet[num++] = (UINT8)( (MA_TIMER_MS >> 7)     | 0x80 );
			packet[num++] = (UINT8)(  base_time             | 0x80 );	/* data */

			packet[num++] = (UINT8)(  MA_TIMER_0_TIME       & 0x7F );	/* address */
			packet[num++] = (UINT8)( (MA_TIMER_0_TIME >> 7) | 0x80 );
			packet[num++] = (UINT8)(  time_count            & 0x7F );	/* data */
			packet[num++] = (UINT8)( (one_shot << 1) | (mode << 2) | 0x80 );

			result = MaDevDrv_SendDirectPacket( packet, num );
		}
		else
		{
			result = MASMW_SUCCESS;
		}
	}
	else									/* Timer #1 */
	{
		sw = (UINT8)( ( mode == 1 ) ? 1 : 0 );

		packet[num++] = (UINT8)(  MA_TIMER_1_TIME		& 0x7F );	/* address */
		packet[num++] = (UINT8)( (MA_TIMER_1_TIME >> 7) | 0x80 );
		packet[num++] = (UINT8)(  time_count            & 0x7F );	/* data */
		packet[num++] = (UINT8)( (one_shot << 1) | (mode << 2) | sw | 0x80 );

		result = MaDevDrv_SendDirectPacket( packet, num );
	}

	if (result >= 0)
		ma_resource_info.tm_map |= tm_map;

	MARESMGR_DBGMSG((" -> tm_map=%08lX\n", ma_resource_info.tm_map));

	return result;
}
/****************************************************************************
 *	MaResMgr_FreeTimer
 *
 *	Function:
 *			Free the specified timer of.
 *	Argument:
 *			timer_id	timer id
 *	Return:
 *			0		success
 *			< 0		error code
 *
 ****************************************************************************/
SINT32 MaResMgr_FreeTimer
(
	UINT8	timer_id					/* timer id */
)
{
	UINT8	packet[3];
	UINT32	tm_map;
	SINT32	result = MASMW_SUCCESS;
	
	MARESMGR_DBGMSG(("    MaResMgr_FreeTimer       : id=%d", timer_id));

	if ( timer_id > 2 ) return MASMW_ERROR_ARGUMENT;

	tm_map = (UINT32)( 1 << timer_id );

	if ( ( tm_map & MA_TM_MAP_MASK ) != tm_map )
	{
		MARESMGR_DBGMSG(("illegal timer value\n"));
		return MASMW_ERROR_ARGUMENT;
	}
	
	if ( ( ma_resource_info.tm_map & tm_map ) != tm_map )
	{
		MARESMGR_DBGMSG(("can't free timer map\n"));
		return MASMW_ERROR;
	}

	ma_resource_info.tm_map ^= tm_map;

	MARESMGR_DBGMSG((" -> tm_map=%08lX\n", ma_resource_info.tm_map));

	if ( timer_id == 2 )					/* Timer #1 */
	{
		packet[0] = (UINT8)(  MA_TIMER_1_CTRL		& 0x7F );	/* address */
		packet[1] = (UINT8)( (MA_TIMER_1_CTRL >> 7) | 0x80 );
		packet[2] = (UINT8)(                          0x80 );	/* data */

		result = MaDevDrv_SendDirectPacket( packet, 3 );
	}

	return result;
}
/****************************************************************************
 *	MaResMgr_AllocSequencer
 *
 *	Function:
 *			Allocate the specified sequencer and setting it.
 *	Argument:
 *			seq_id		sequencer id
 *			base_time	minimum time base of sequencer
 *	Return:
 *			0		success
 *			< 0		error codes
 *
 ****************************************************************************/
SINT32 MaResMgr_AllocSequencer
(
	UINT8	seq_id,						/* sequencer id */
	UINT16	base_time					/* base time */
)
{
	UINT8	packet[5];
	UINT32	sq_map;
	SINT32	result;

	MARESMGR_DBGMSG(("    MaResMgr_AllocSequencer  : id=%d ", seq_id));

	sq_map = (UINT32)( 1 << seq_id );

	if ( ( sq_map & MA_SQ_MAP_MASK ) != sq_map )
	{
		MARESMGR_DBGMSG(("illegal sequencer number\n"));
		return MASMW_ERROR_ARGUMENT;
	}
	
	if ( ( ma_resource_info.sq_map & sq_map ) != 0 )
	{
		MARESMGR_DBGMSG(("can't alloc sequencer\n"));
		return MASMW_ERROR_RESOURCE_OVER;
	}

	ma_resource_info.sq_map |= sq_map;

	MARESMGR_DBGMSG(("-> sq_map=%08lX\n", ma_resource_info.sq_map));

	if ( seq_id == 0 )
	{
		packet[0] = (UINT8)(  MA_SEQUENCE       & 0x7F );	/* address */
		packet[1] = (UINT8)( (MA_SEQUENCE >> 7) | 0x80 );
		packet[2] = (UINT8)( (base_time   >> 7) & 0x7F );	/* data */
		packet[3] = (UINT8)(  base_time         & 0x7F );
		packet[4] = (UINT8)(  MA_INT_POINT      | 0x80 );
		result = MaDevDrv_SendDirectPacket( packet, 5 );
	}
	else
	{
		result = MASMW_SUCCESS;
	}

	if (result < 0)
		ma_resource_info.sq_map ^= sq_map;

	return result;
}
/****************************************************************************
 *	MaResMgr_FreeSequencer
 *
 *	Function:
 *			Free the specified sequencer.
 *	Argument:
 *			seq_id	sequencer id
 *	Return:
 *			0		success
 *			< 0		error codes
 *
 ****************************************************************************/
SINT32 MaResMgr_FreeSequencer
(
	UINT8	seq_id						/* sequencer id */
)
{
	UINT32	sq_map;
	
	MARESMGR_DBGMSG(("    MaResMgr_FreeSequencer   : id=%d", seq_id));

	sq_map = (UINT32)( 1 << seq_id );

	if ( ( sq_map & MA_SQ_MAP_MASK ) != sq_map )
	{
		MARESMGR_DBGMSG(("illegal sequencer number\n"));
		return MASMW_ERROR_ARGUMENT;
	}
	
	if ( ( ma_resource_info.sq_map & sq_map ) != sq_map )
	{
		MARESMGR_DBGMSG(("can't free seq\n"));
		return MASMW_ERROR;
	}

	ma_resource_info.sq_map ^= sq_map;

	MARESMGR_DBGMSG((" -> sq_map=%08lX\n", ma_resource_info.sq_map));

	return MASMW_SUCCESS;
}
/****************************************************************************
 *	MaResMgr_Initialize
 *
 *	Function:
 *			Initialize the MA Resource Manager.
 *	argument:
 *			None
 *	return:
 *			0		success
 *
 ****************************************************************************/
SINT32 MaResMgr_Initialize( void )
{
	UINT8	wave_id;
	
	MARESMGR_DBGMSG(("MaResMgr_Initialize\n"));

	ma_resource_info.ch_map = 0x00000000;		/* channel */
	ma_resource_info.fm_map = 0x00000000;		/* FM voice */
	ma_resource_info.wt_map = 0x00000000;		/* WT voice */
	ma_resource_info.tm_map = 0x00000000;		/* timer */
	ma_resource_info.si_map = 0x00000000;		/* software interrupt */
	ma_resource_info.rb_map = 0x00000000;		/* RAM block */
	ma_resource_info.sa_map = 0x00000000;		/* stream audio */
	ma_resource_info.ld_map = 0x00000000;		/* LED */
	ma_resource_info.mt_map = 0x00000000;		/* motor */
	ma_resource_info.sq_map = 0x00000000;		/* sequencer */

	/* reset stream audio info */
	for ( wave_id = 0; wave_id < MA_MAX_REG_STREAM_AUDIO; wave_id++ )
	{
		ma_resource_info.stream_audio[wave_id].format    = (UINT8)0;
		ma_resource_info.stream_audio[wave_id].wave_ptr  = NULL;
		ma_resource_info.stream_audio[wave_id].wave_size = (UINT32)0;
		ma_resource_info.stream_audio[wave_id].seek_pos  = (UINT32)0;
	}
	
	return MASMW_SUCCESS;
}
