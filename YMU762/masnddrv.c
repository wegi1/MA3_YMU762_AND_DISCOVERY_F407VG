/****************************************************************************
 *																			*
 *		Copyright (C) 2001-2003	YAMAHA CORPORATION. All rights reserved.	*
 *																			*
 *		Module		: masnddrv.c											*
 *																			*
 *		Description	: MA Sound Driver										*
 *																			*
 * 		Version		: 1.3.15.4	2003.04.04									*
 *																			*
 ****************************************************************************/

#include "masnddrv.h"
#include "matable.h"

static MASNDDRVINFO		ma_snddrv_info[MASMW_NUM_SEQTYPE];
static MACHINFO 		ma_channel_info[MASMW_NUM_CHANNEL];
static MAVOICEINFO		ma_voice_info[MASMW_NUM_VOICEINFO][MA_MAX_REG_VOICE*2];
static UINT8			ma_voice_index[MA_MAX_RAM_BANK*2*128];	/* (Bank[16+16]) * Prog   */
																/* index of ma_voice_info */
																/* NULL if index = 0xff   */
static MASTREAMINFO		ma_stream_info[MASMW_NUM_SEQTYPE][MA_MAX_REG_STREAM];
static MASLOTINFO		ma_slot_info[MASMW_MAX_SLOT];
static MASLOTLIST		ma_slot_list[MASMW_NUM_SEQTYPE];

static MALEDINFO		ma_led_info;
static MAMTRINFO		ma_mtr_info;

static MA_SEQBUFINFO	ma_seqbuf_info;

SINT32 (* ma_srm_cnv[MASMW_NUM_SEQTYPE])( void );

static UINT32			ma_pos_count[MASMW_NUM_SEQTYPE];

static UINT8			ma_control_sequencer[MASMW_NUM_SEQTYPE];

static SINT32 MaSndDrv_Nop					( SINT32 sid, SINT32 dt, UINT32 tp, UINT32 p2,  UINT32 p3 );
static SINT32 MaSndDrv_UserEvent			( SINT32 sid, SINT32 dt, UINT32 vl, UINT32 p2,  UINT32 p3 );
static SINT32 MaSndDrv_NoteOn				( SINT32 sid, SINT32 dt, UINT32 ch, UINT32 key, UINT32 vel );
static SINT32 MaSndDrv_NoteOnMa2			( SINT32 sid, SINT32 dt, UINT32 ch, UINT32 key, UINT32 vel );
static SINT32 MaSndDrv_NoteOnMa2Ex			( SINT32 sid, SINT32 dt, UINT32 ch, UINT32 key, UINT32 vel );
static SINT32 MaSndDrv_NoteOff				( SINT32 sid, SINT32 dt, UINT32 ch, UINT32 key, UINT32 p3 );
static SINT32 MaSndDrv_NoteOffMa2			( SINT32 sid, SINT32 dt, UINT32 ch, UINT32 key, UINT32 p3 );
static SINT32 MaSndDrv_NoteOffMa2Ex			( SINT32 sid, SINT32 dt, UINT32 ch, UINT32 key, UINT32 p3 );
static SINT32 MaSndDrv_ProgramChange		( SINT32 sid, SINT32 dt, UINT32 ch, UINT32 bkn, UINT32 prg );
static SINT32 MaSndDrv_ModulationDepth		( SINT32 sid, SINT32 dt, UINT32 ch, UINT32 dph, UINT32 p3 );
static SINT32 MaSndDrv_ChannelVolume		( SINT32 sid, SINT32 dt, UINT32 ch, UINT32 vol, UINT32 p3 );
static SINT32 MaSndDrv_Panpot				( SINT32 sid, SINT32 dt, UINT32 ch, UINT32 pan, UINT32 p3 );
static SINT32 MaSndDrv_Expression			( SINT32 sid, SINT32 dt, UINT32 ch, UINT32 vol, UINT32 p3 );
static SINT32 MaSndDrv_Hold1				( SINT32 sid, SINT32 dt, UINT32 ch, UINT32 val, UINT32 p3 );
static SINT32 MaSndDrv_AllSoundOff			( SINT32 sid, SINT32 dt, UINT32 ch, UINT32 p2,  UINT32 p3 );
static SINT32 MaSndDrv_ResetAllControllers	( SINT32 sid, SINT32 dt, UINT32 ch, UINT32 p2,  UINT32 p3 );
static SINT32 MaSndDrv_AllNoteOff			( SINT32 sid, SINT32 dt, UINT32 ch, UINT32 p2,  UINT32 p3 );
static SINT32 MaSndDrv_MonoModeOn			( SINT32 sid, SINT32 dt, UINT32 ch, UINT32 p2,  UINT32 p3 );
static SINT32 MaSndDrv_PolyModeOn			( SINT32 sid, SINT32 dt, UINT32 ch, UINT32 p2,  UINT32 p3 );
static SINT32 MaSndDrv_PitchBend			( SINT32 sid, SINT32 dt, UINT32 ch, UINT32 val, UINT32 p3 );
static SINT32 MaSndDrv_BendRange			( SINT32 sid, SINT32 dt, UINT32 ch, UINT32 rng, UINT32 p3 );
static SINT32 MaSndDrv_StreamOn				( SINT32 sid, SINT32 dt, UINT32 ch, UINT32 wid, UINT32 vel );
static SINT32 MaSndDrv_StreamOff			( SINT32 sid, SINT32 dt, UINT32 ch, UINT32 wid, UINT32 p3 );
static SINT32 MaSndDrv_StreamSlave			( SINT32 sid, SINT32 dt, UINT32 ch, UINT32 wid, UINT32 vel );
static SINT32 MaSndDrv_StreamPanpot			( SINT32 sid, SINT32 dt, UINT32 wv, UINT32 pan, UINT32 p3 );
static SINT32 MaSndDrv_MasterVolume			( SINT32 sid, SINT32 dt, UINT32 vl, UINT32 p2,  UINT32 p3 );
static SINT32 MaSndDrv_SystemOn				( SINT32 sid, SINT32 dt, UINT32 p1, UINT32 p2,  UINT32 p3 );
static SINT32 MaSndDrv_LedOn				( SINT32 sid, SINT32 dt, UINT32 p1, UINT32 p2,  UINT32 p3 );
static SINT32 MaSndDrv_LedOff				( SINT32 sid, SINT32 dt, UINT32 p1, UINT32 p2,  UINT32 p3 );
static SINT32 MaSndDrv_MotorOn				( SINT32 sid, SINT32 dt, UINT32 p1, UINT32 p2,  UINT32 p3 );
static SINT32 MaSndDrv_MotorOff				( SINT32 sid, SINT32 dt, UINT32 p1, UINT32 p2,  UINT32 p3 );
static SINT32 MaSndDrv_StreamSeek			( SINT32 sid, SINT32 dt, UINT32 p1, UINT32 p2,  UINT32 p3 );

static SINT32 (* ma_snddrv_command[])( SINT32 sid, SINT32 dt, UINT32 p1, UINT32 p2, UINT32 p3 ) =
{
	MaSndDrv_NoteOn,						/*  0 */
	MaSndDrv_NoteOnMa2,						/*  1 */
	MaSndDrv_NoteOnMa2Ex,					/*  2 */
	MaSndDrv_NoteOff,						/*  3 */
	MaSndDrv_NoteOffMa2,					/*  4 */
	MaSndDrv_NoteOffMa2Ex,					/*  5 */
	MaSndDrv_ProgramChange,					/*  6 */
	MaSndDrv_ModulationDepth,				/*  7 */
	MaSndDrv_ChannelVolume,					/*  8 */
	MaSndDrv_Panpot,						/*  9 */
	MaSndDrv_Expression,					/* 10 */
	MaSndDrv_Hold1,							/* 11 */
	MaSndDrv_AllSoundOff,					/* 12 */
	MaSndDrv_ResetAllControllers,			/* 13 */
	MaSndDrv_AllNoteOff,					/* 14 */
	MaSndDrv_MonoModeOn,					/* 15 */
	MaSndDrv_PolyModeOn,					/* 16 */
	MaSndDrv_PitchBend,						/* 17 */
	MaSndDrv_BendRange,						/* 18 */
	MaSndDrv_StreamOn,						/* 19 */
	MaSndDrv_StreamOff,						/* 20 */
	MaSndDrv_StreamSlave,					/* 21 */
	MaSndDrv_StreamPanpot,					/* 22 */
	MaSndDrv_MasterVolume,					/* 23 */
	MaSndDrv_SystemOn,						/* 24 */
	MaSndDrv_LedOn,							/* 25 */
	MaSndDrv_LedOff,						/* 26 */
	MaSndDrv_MotorOn,						/* 27 */
	MaSndDrv_MotorOff,						/* 28 */
	MaSndDrv_UserEvent,						/* 29 */
	MaSndDrv_Nop,							/* 30 */
	MaSndDrv_StreamSeek						/* 31 */
};

static UINT8 packet_buf[MASMW_NUM_SEQTYPE][256];

extern void machdep_memcpy( UINT8 *d, UINT8 *s, UINT32 size );

/*--------------------------------------------------------------------------*/
/****************************************************************************
 *	SendDelayedPacket
 *
 *	Description:
 *			Send the delayed packet.
 *	Argument:
 *			packet_ptr	pointer to the packet data
 *			packet_size	size of the packet data
 *	Return:
 *			0		success
 *			1		save to queue
 *
 ****************************************************************************/
static SINT32 SendDelayedPacket
(
	UINT8 *	packet_ptr,					/* pointer to the packet */
	UINT32	packet_size					/* size of the packet */
)
{
	MASNDDRV_DBGMSG(("SendDelayedPacket: ptr=%p sz=%ld\n", packet_ptr, packet_size));

	if ( ( ma_seqbuf_info.buf_size < packet_size ) || ( ma_seqbuf_info.queue_flag == 1 ) )
	{
		machdep_memcpy( &ma_seqbuf_info.queue[ma_seqbuf_info.queue_wptr], packet_ptr, packet_size );

		ma_seqbuf_info.queue_flag = 1;
		ma_seqbuf_info.queue_size += packet_size;
		ma_seqbuf_info.queue_wptr += packet_size;

		return 1;
	}

	if ( ma_seqbuf_info.buf_ptr == NULL ) return MASMW_ERROR;

	machdep_memcpy( ma_seqbuf_info.buf_ptr, packet_ptr, packet_size );
	ma_seqbuf_info.buf_ptr    += packet_size;
	ma_seqbuf_info.buf_size   -= packet_size;
	ma_seqbuf_info.wrote_size += packet_size;

	return MASMW_SUCCESS;
}
/****************************************************************************
 *	GetVoiceInfo
 *
 *	Description:
 *			Return the voice information.
 *	Arguments:
 *			seq_id			sequence id (0..1)
 *			ch				channel number (0..15)
 *			key				key number (0..127)
 *			voice_info_ptr	pointer to the voice information
 *	Return:
 *			voice information:
 *			  key number or base frequency (48[KHz] = 1.0[0x0400])
 *			  internal RAM address
 *			  type of synthesize (1:FM, 2:WT)
 *
 ****************************************************************************/
static SINT32 GetVoiceInfo
(
	SINT32		seq_id,					/* sequence id */
	UINT32 		ch,						/* channel number */
	UINT32 		key,					/* key number */
	PMAVINFO	voice_info_ptr			/* voice information */
)
{
	UINT8	bank_no;					/* bank number */
	UINT8	prog_no;					/* program number */
	UINT16	bank_prog;					/* bank & program number */
	UINT32	type;						/* type */
	UINT32	i;							/* loop counter */
	SINT32	result;						/* result of function */

	MASMW_ASSERT( seq_id <= MASMW_SEQTYPE_DIRECT );

	bank_no = ma_channel_info[ch].bank_no;
	prog_no = (UINT8)( ( bank_no < 128 ) ? ma_channel_info[ch].prog_no : key ) & 0x7F;

	if ( bank_no < 128 )
	{
		type = 0;
		bank_prog = (UINT16)(((UINT16)bank_no << 7) + (UINT16)prog_no);
	}
	else
	{
		type = MA_MAX_REG_VOICE;
		bank_prog = (UINT16)(((UINT16)(bank_no - 128 + MA_MAX_RAM_BANK) << 7) + (UINT16)prog_no);
	}

	if ( ( bank_no & 0x7f ) < MA_MAX_RAM_BANK )
	{
		if ( ma_voice_index[bank_prog] != 0xff )
		{
			/*--- Yes, Voice Info in RAM ---------*/
			i = ma_voice_index[bank_prog];
			voice_info_ptr->synth	= ma_voice_info[seq_id][i].synth;
			voice_info_ptr->key		= ma_voice_info[seq_id][i].key;
			voice_info_ptr->address = (UINT16)(ma_voice_info[seq_id][i].ram_adrs >> 1);
			
			return MASMW_SUCCESS;
		}
	}

	if ( ma_snddrv_info[seq_id].drum_type != 0 )
	{
		if ( bank_no >= 128 )
		{
			if ( ( prog_no < MA_MIN_GML1_DRUM ) || ( MA_MAX_GML1_DRUM < prog_no ) )
			{
				return MASMW_ERROR;
			}
		}
	}

	result = MaResMgr_GetDefVoiceSynth( (UINT8)(type + prog_no) );
	if ( result <= 0 )
		return MASMW_ERROR;
	else
		voice_info_ptr->synth = (UINT8)result;

	if ( type == MA_MAX_REG_VOICE )
	{
		result = MaResMgr_GetDefVoiceKey( (UINT8)(type + prog_no) );
		if ( result <= 0 )
			return MASMW_ERROR;
		else
			voice_info_ptr->key = (UINT16)result;
	}

	result = MaResMgr_GetDefVoiceAddress( (UINT8)(type + prog_no) );
	if ( result <= 0 )
		return MASMW_ERROR;
	else
		voice_info_ptr->address = (UINT16)(result >> 1);

	return MASMW_SUCCESS;
}
/****************************************************************************
 *	CalcChVolume
 *
 *	Description:
 *			Calculate the channel volume.
 *	Arguments:
 *			ch			channnel number (0..15)
 *	Return:
 *			Channel volume value.
 *
 ****************************************************************************/
static UINT8 CalcChVolume
(
	UINT32	ch							/* channel number */
)
{
	UINT32	db;
	UINT8	volume;

	db = (UINT32)db_table[ma_channel_info[ch].volume];
	db = (UINT32)( db + db_table[ma_channel_info[ch].expression] );
	if ( db > (UINT32)MASMW_MAX_DB ) db = (UINT32)MASMW_MAX_DB;

	volume = vol_table[db];

	return volume;
}
/****************************************************************************
 *	CalcVoiceVolume
 *
 *	Description:
 *			Calculate the channel volume.
 *	Arguments:
 *			seq_id		sequence id (0..2)
 *			vol			volume (0..127)
 *			offset		offset value
 *	Return:
 *			volume value of slot (0..127).
 *
 ****************************************************************************/
static UINT8 CalcVoiceVolume
(
	SINT32	seq_id,						/* sequence id */
	UINT32	vol,						/* volume value */
	UINT32	offset						/* offset value */
)
{
	UINT32	db;
	UINT8	volume;

	db = db_table[vol];
	db = (UINT32)( db + db_table[ma_snddrv_info[seq_id].master_volume] );
	db = (UINT32)( db + db_table[ma_snddrv_info[seq_id].ctrl_volume] );
	db = (UINT32)(( db >= offset) ? (db - offset) : 0);
	if ( db > (UINT32)MASMW_MAX_DB ) db = (UINT32)MASMW_MAX_DB;

	volume = vol_table[db];

	return volume;
}
/****************************************************************************
 *	MakeDeltaTime
 *
 *	Description:
 *			
 *	Arguments:
 *			packet_ptr
 *			delta_time
 *	Return:
 *			
 *
 ****************************************************************************/
static UINT32 MakeDeltaTime( UINT8 * packet_ptr, SINT32 delta_time )
{
	UINT32	num = 0;

	if ( delta_time < 0 ) return num;
	
	while ( ( delta_time > 127 ) && ( num < 2 ) )
	{
		*(packet_ptr+num) = (UINT8)(delta_time & 0x7F);
		delta_time >>= 7;
		num++;
	}
	*(packet_ptr+num) = (UINT8)(delta_time | 0x80 );
	
	return (UINT32)(num+1);
}
/****************************************************************************
 *	GetSlotList
 *
 *	Description:
 *			Return slot number from slot list for note on.
 *	Arguments:
 *			seq_id		sequence id (0..2)
 *			synth_type	type of synthesize (0..2)
 *	Return:
 *			slot number (0..15/31, 32..39, 40..41)
 *
 ****************************************************************************/
static UINT32 GetSlotList
(
	SINT32	seq_id,						/* sequence id */
	UINT32	synth_type					/* synth type */
)
{
	UINT8	top_slot;
	UINT8	end_slot;

	top_slot = ma_slot_list[seq_id].top[synth_type];
	end_slot = ma_slot_list[seq_id].end[synth_type];

	ma_slot_list[seq_id].next[end_slot]  = top_slot;
	ma_slot_list[seq_id].top[synth_type] = ma_slot_list[seq_id].next[top_slot];
	ma_slot_list[seq_id].end[synth_type] = top_slot;
	ma_slot_list[seq_id].next[top_slot]  = (UINT8)(ma_snddrv_info[seq_id].max_slot[synth_type] + 1);

	return top_slot;
}
/****************************************************************************
 *	SearchSlotList
 *
 *	Description:
 *			Return slot number from slot list for note off.
 *	Arguments:
 *			seq_id		sequence id (0..2)
 *			synth_type	type of synthesize (0..2)
 *			ch			channel number (0..15)
 *			key			key number (0..127)
 *	Return:
 *			slot number (0..15/31, 32..39, 40..41, 255)
 *
 ****************************************************************************/
static UINT32 SearchSlotList
(
	SINT32	seq_id,						/* sequence id */
	UINT32	synth_type,					/* synth type */
	UINT32	ch,							/* channel number */
	UINT32	key							/* key number */
)
{
	UINT32	slot_no;					/* slot number */
	UINT32	min_slot;					/* minimum slot number */
	UINT32	max_slot;					/* maximum slot number */
	UINT32	i;							/* loop counter */

	min_slot = ma_snddrv_info[seq_id].min_slot[synth_type];
	max_slot = ma_snddrv_info[seq_id].max_slot[synth_type];
	
	slot_no = ma_slot_list[seq_id].top[synth_type];
	for ( i = min_slot; i <= max_slot; i++ )
	{
		if ( ma_slot_info[slot_no].ch == (ch + 0x81) )
		{
			if ( ma_slot_info[slot_no].key == key )
			{
				return slot_no;
			}
		}

		slot_no = ma_slot_list[seq_id].next[slot_no];
	}

	return 0xFF;
}
/****************************************************************************
 *	RemakeSlotList
 *
 *	Description:
 *			Return slot number from slot list for note off.
 *	Arguments:
 *			synth_type	type of synthesize (0..2)
 *	Return:
 *			None
 *
 ****************************************************************************/
static void RemakeSlotList
(
	SINT32	seq_id,						/* sequence id */
	UINT32	synth_type,					/* synth type */
	UINT32	slot_no,					/* slot number */
	UINT32	type						/* type */
)
{
	UINT32	prv_slot;
	UINT32	nxt_slot;
	UINT32	temp = 0;
	UINT32	flag = 0;
	UINT32	min_slot;
	UINT32	max_slot;
	UINT32	i;
	
	min_slot = ma_snddrv_info[seq_id].min_slot[synth_type];
	max_slot = ma_snddrv_info[seq_id].max_slot[synth_type];

	prv_slot = nxt_slot = ma_slot_list[seq_id].top[synth_type];
	for ( i = min_slot; i <= max_slot; i++ )
	{
		if ( flag == 0 )
		{
			if ( ma_slot_info[nxt_slot].ch > type )
			{
				if ( nxt_slot == ma_slot_list[seq_id].top[synth_type] )
				{
					ma_slot_list[seq_id].top[synth_type] = (UINT8)slot_no;
				}
				else
				{
					ma_slot_list[seq_id].next[prv_slot] = (UINT8)slot_no;
				}

				temp = ma_slot_list[seq_id].next[slot_no];
				ma_slot_list[seq_id].next[slot_no] = (UINT8)nxt_slot;

				flag = 1;
			}
		}

		if ( flag == 1 )
		{
			if ( ma_slot_list[seq_id].next[nxt_slot] == slot_no )
			{
				if ( slot_no == ma_slot_list[seq_id].end[synth_type] )
				{
					ma_slot_list[seq_id].end[synth_type] = (UINT8)nxt_slot;
				}
				
				ma_slot_list[seq_id].next[nxt_slot] = (UINT8)temp;
				break;
			}
		}

		prv_slot = nxt_slot;
		nxt_slot = ma_slot_list[seq_id].next[nxt_slot];
	}
}
/****************************************************************************
 *	RemakeSlotListE
 *
 *	Description:
 *			Return slot number from slot list for note off.
 *	Arguments:
 *			synth_type	type of synthesize (0..2)
 *	Return:
 *			None
 *
 ****************************************************************************/
static void RemakeSlotListE
(
	SINT32	seq_id,						/* sequence id */
	UINT32	synth_type,					/* synth type */
	UINT32	slot_no						/* slot number */
)
{
	UINT32	prv_slot;
	UINT32	nxt_slot;
	UINT32	min_slot;
	UINT32	max_slot;
	UINT32	i;
	UINT8	end_slot;

	min_slot = ma_snddrv_info[seq_id].min_slot[synth_type];
	max_slot = ma_snddrv_info[seq_id].max_slot[synth_type];

	prv_slot = nxt_slot = ma_slot_list[seq_id].top[synth_type];
	for ( i = min_slot; i <= max_slot; i++ )
	{
		if ( nxt_slot == slot_no )
		{
			if		( i == min_slot )
			{
				ma_slot_list[seq_id].top[synth_type] = ma_slot_list[seq_id].next[nxt_slot];
			}
			else if	( i == max_slot )
			{
				break;
			}
			else
			{
				ma_slot_list[seq_id].next[prv_slot]  = ma_slot_list[seq_id].next[nxt_slot];
			}
			break;
		}
		prv_slot = nxt_slot;
		nxt_slot = ma_slot_list[seq_id].next[nxt_slot];
	}

	end_slot = ma_slot_list[seq_id].end[synth_type];
	ma_slot_list[seq_id].next[end_slot]  = (UINT8)slot_no;
	ma_slot_list[seq_id].end[synth_type] = (UINT8)slot_no;
	ma_slot_list[seq_id].next[slot_no]   = (UINT8)(ma_snddrv_info[seq_id].max_slot[synth_type] + 1);
}
/****************************************************************************
 *	GetSlot
 *
 *	Description:
 *			Return slot number for NoteOn
 *	Arguments:
 *			seq_id		sequence id (0..2)
 *			synth_type	type of synthesize (1..3)
 *			ch			channel number (0..15)
 *	Return:
 *			slot number (0..15/31, 32..39, 40..41)
 *
 ****************************************************************************/
static UINT32 GetSlot
(
	SINT32	seq_id,						/* sequence id */
	UINT32	synth_type,					/* synth type */
	UINT32	ch							/* channel number */
)
{
	UINT32	slot_no;					/* slot number */

	MASMW_ASSERT( ma_snddrv_info[seq_id].dva_mode == 2 );

	if ( synth_type != 2 )
	{
		slot_no = ch;
	}
	else
	{
		slot_no = GetSlotList( seq_id, (UINT8)(synth_type - 1) );
	}

	return slot_no;
}
/****************************************************************************
 *	SearchSlot
 *
 *	Description:
 *			Return slot number for NoteOff
 *	Arguments:
 *			seq_id		sequence id (0..2)
 *			ch			channel number (0..15)
 *			key			key number (0..127)
 *	Return:
 *			slot number (0..15/31, 32..39, 40..41)
 *
 ****************************************************************************/
static UINT32 SearchSlot
(
	SINT32	seq_id,						/* sequence id */
	UINT32	synth_type,					/* synth type */
	UINT32	ch,							/* channel number */
	UINT32	key							/* key number */
)
{
	UINT32	i;
	UINT32	slot_no;
	UINT32	synth;
	static const UINT8 search_slot[3][3] = { { 0, 1, 2 },
											 { 1, 0, 2 },
											 { 2, 0, 1 } };

	MASMW_ASSERT( ma_snddrv_info[seq_id].dva_mode != 2 );

	slot_no = 0xFF;

	for ( i = 0; i < 3; i++ )
	{
		synth = search_slot[synth_type-1][i];
		slot_no = SearchSlotList( seq_id, synth, ch, key );
		if ( slot_no != 0xFF )
		{
			RemakeSlotList( seq_id, synth, slot_no, 0x80 );
			break;
		}
	}

	return slot_no;
}
/****************************************************************************
 *	SearchSlotMa2
 *
 *	Description:
 *			Return slot number for NoteOff
 *	Arguments:
 *			seq_id		sequence id (0..2)
 *			ch			channel number (0..15)
 *			key			key number (0..127)
 *	Return:
 *			slot number (0..15/31, 32..39, 40..41)
 *
 ****************************************************************************/
static UINT32 SearchSlotMa2
(
	SINT32	seq_id,						/* sequence id */
	UINT32	synth_type,					/* synth type */
	UINT32	ch,							/* channel number */
	UINT32	key							/* key number */
)
{
	UINT32	i;
	UINT32	slot_no;
	UINT32	synth;
	static const UINT8 search_slot[3][3] = { { 0, 1, 2 },
											 { 1, 0, 2 },
											 { 2, 0, 1 } };

	MASMW_ASSERT( ma_snddrv_info[seq_id].dva_mode == 2 );

	if ( synth_type != 2 )	return ch;

	slot_no = 0xFF;

	for ( i = 0; i < 3; i++ )
	{
		synth = search_slot[synth_type-1][i];
		slot_no = SearchSlotList( seq_id, synth, ch, key );
		if ( slot_no != 0xFF )
		{
			RemakeSlotList( seq_id, synth, slot_no, 0x80 );
			break;
		}
	}

	return slot_no;
}
/****************************************************************************
 *	GetStreamBlockFnum
 *
 *	Description:
 *			
 *	Arguments:
 *			seq_id		sequence id
 *			wave_id		wave id
 *	Return:
 *			0			success
 *			< 0			error code
 ****************************************************************************/
static UINT16 GetStreamBlockFnum
(
	SINT32	seq_id,
	UINT8	wave_id
)
{
	UINT32	block;
	UINT32	f_num = 0;
	UINT32	base_fnum;
	UINT32	fs;
	UINT16	fix_pitch;

	fs = ma_stream_info[seq_id][wave_id].frequency;
	base_fnum = 0x00001000;

	fs <<= 12;
	fs /= MA_FS;
	fs *= base_fnum;
	fs >>= 8;
	if ( fs > 0x10000 )		fs = 0x10000;		/* 1.0 à»è„ */
	else if ( fs < 0x800 )	fs = 0x800;

	for ( block = 0; block < 8; block++ )
	{
		f_num = (fs >> (block + 1)) - 1024;
		if ( f_num < 1024 ) break;
	}

	fix_pitch = (UINT16)( ((f_num << 3) & 0x1C00)
	                    | ((block << 7) & 0x0380)
			            | ( f_num       & 0x007F) );

	MASNDDRV_DBGMSG(("  GetStreamPitchFnum[%d] = %04X\n", wave_id, fix_pitch));
	
	return fix_pitch;
}
/****************************************************************************
 *	GetRealKeyMelody
 *
 *	Description:
 *			Get real key number of melody with key control.
 *	Arguments:
 *			seq_id		sequence id (0..1)
 *			ch			channel number (0..15)
 *			key			key number (0..127)
 *	Return:
 *			real key number (0..127)
 *
 ****************************************************************************/
static UINT8 GetRealKeyMelody
(
	SINT32	seq_id,						/* sequence id */
	UINT8	ch,							/* channel number */
	UINT8	key							/* key number */
)
{
	SINT16	rkey;						/* real key value */
	SINT16	key_offset;					/* key offset */
	UINT32	prog_no;
	
	rkey = (SINT16)key;
	key_offset = (SINT16)ma_snddrv_info[seq_id].key_offset;
	
	switch ( ma_snddrv_info[seq_id].melody_mode )
	{
	case 0:

		prog_no = ma_channel_info[ch].prog_no;
		if ( prog_no <= 114 )									/* 0..114 */
		{
			if ( ma_channel_info[ch].key_control != MA_KEYCTRL_OFF )
			{
				rkey = (SINT16)( rkey + key_offset );
			}
		}
		else if ( prog_no <= 119 )								/* 115..119 */
		{
			if ( ma_channel_info[ch].key_control == MA_KEYCTRL_ON )
			{
				rkey = (SINT16)( rkey + key_offset * 2 );
			}
		}
		else if ( prog_no <= 121 )								/* 119..121 */
		{
			if ( ma_channel_info[ch].key_control != MA_KEYCTRL_OFF )
			{
				rkey = (SINT16)( rkey + key_offset );
			}
		}
		else if ( ( prog_no == 122 ) || ( prog_no == 127 ) )	/* 122, 127 */
		{
			if ( ma_channel_info[ch].key_control == MA_KEYCTRL_ON )
			{
				rkey = (SINT16)( rkey + key_offset * 5 );
			}
		}
		else if ( ( prog_no == 123 ) || ( prog_no == 126 ) )	/* 123, 126 */
		{
			if ( ma_channel_info[ch].key_control == MA_KEYCTRL_ON )
			{
				rkey = (SINT16)( rkey + key_offset * 20 );
			}
		}
		else if ( ( prog_no == 124 ) || ( prog_no == 125 ) )	/* 124, 125 */
		{
			if ( ma_channel_info[ch].key_control == MA_KEYCTRL_ON )
			{
				rkey = (SINT16)( rkey + key_offset * 10 );
			}
		}

		if ( rkey < 0   ) rkey = 0;
		if ( rkey > 127 ) rkey = 127;

		break;

	case 1:

		if ( ma_channel_info[ch].key_control != MA_KEYCTRL_OFF )
		{
			rkey = (SINT16)( rkey + key_offset );
			if ( rkey < 0   ) rkey = 0;
			if ( rkey > 127 ) rkey = 127;
		}
		break;

	default:
		break;
	}

	return (UINT8)rkey;
}
/****************************************************************************
 *	GetRealKeyDrum
 *
 *	Description:
 *			Get real key number of drum with key control.
 *	Arguments:
 *			seq_id		sequence id (0..1)
 *			ch			channel number (0..15)
 *			key			key number (0..127)
 *	Return:
 *			real key number (0..127)
 *
 ****************************************************************************/
static UINT8 GetRealKeyDrum
(
	SINT32	seq_id,						/* sequence id */
	UINT8	ch,							/* channel number */
	UINT8	key							/* key number */
)
{
	SINT16	rkey;						/* real key value */
	SINT16	key_offset;					/* key offset */
	
	rkey = (SINT16)key;
	key_offset = (SINT16)ma_snddrv_info[seq_id].key_offset;
	
	if ( ma_channel_info[ch].key_control == MA_KEYCTRL_ON )
	{
		rkey = (SINT16)( rkey + key_offset );
		if ( rkey < 0   ) rkey = 0;
		if ( rkey > 127 ) rkey = 127;
	}

	return (UINT8)rkey;
}
/****************************************************************************
 *	GetFmBlockFnum
 *
 *	Description:
 *			Return Block:Fnum value for FM slot.
 *	Arguments:
 *			seq_id		sequence id (0..1)
 *			ch			channel number (0..15)
 *			key			key number (0..127)
 *			drum_key	drum key number (0..127)
 *	Return:
 *			0			success
 *			< 0			error code
 *
 ****************************************************************************/
static UINT16 GetFmBlockFnum
(
	SINT32	seq_id,						/* sequence id */
	UINT8	ch,							/* channel number */
	UINT32	key,						/* key number */
	UINT16	drum_key					/* key number for drum */
)
{
	UINT8	rkey;						/* real key number */
	UINT8	bank_no;					/* bank number */
	UINT8	prog_no;					/* program number */
	UINT16	pitch;						/* pitch = block&fnum */

	MASMW_ASSERT( seq_id <= MASMW_SEQTYPE_DIRECT );
	MASMW_ASSERT( key <= 0x80000000 );

	bank_no = ma_channel_info[ch].bank_no;
	prog_no = ma_channel_info[ch].prog_no;

	if ( bank_no < 128 )
	{
		rkey = GetRealKeyMelody( seq_id, ch, (UINT8)key & 0x7F );
		switch ( ma_snddrv_info[seq_id].melody_mode )
		{
		case 0:
			
			if ( prog_no <= 114 )									/* 1..114 */
			{
				pitch = ma_fm_block_100[rkey];
			}
			else if ( prog_no == 115 )								/* 115 */
			{
				pitch = ma_fm_block_50a[rkey];
			}
			else if ( prog_no == 116 )								/* 116 */
			{
				pitch = ma_fm_block_50b[rkey];
			}
			else if ( prog_no == 117 )								/* 117 */
			{
				pitch = ma_fm_block_50c[rkey];
			}
			else if ( prog_no <= 119 )								/* 118, 119 */
			{
				pitch = ma_fm_block_50[rkey];
			}
			else if ( prog_no <= 121 )								/* 120, 121 */
			{
				pitch = ma_fm_block_100[rkey];
			}
			else if ( ( prog_no == 122 ) || ( prog_no == 127 ) )	/* 122, 127 */
			{
				pitch = ma_fm_block_20[rkey];
			}
			else if ( ( prog_no == 123 ) || ( prog_no == 126 ) )	/* 123, 126 */
			{
				pitch = ma_fm_block_5[rkey];
			}
			else if ( ( prog_no == 124 ) || ( prog_no == 125 ) )	/* 124, 125 */
			{
				pitch = ma_fm_block_10[rkey];
			}
			else
			{
				pitch = ma_fm_block_100[rkey];
			}
			break;
			
		case 1:
		default:
			pitch = ma_fm_block_100[rkey];
			break;
		}
	}
	else
	{
		rkey = GetRealKeyDrum( seq_id, ch, (UINT8)drum_key & 0x7F );
		pitch = ma_fm_block_100[rkey];
	}

	return pitch;
}
/****************************************************************************
 *	GetFmBlockFnumMa2
 *
 *	Description:
 *			Return MA-2 Block:Fnum value for FM slot.
 *	Arguments:
 *			seq_id		sequence id (0..1)
 *			ch			channel number (0..15)
 *			key			key number (0x8000xxxx)
 *			drum_key	drum key number (0..127)
 *	Return:
 *			0			success
 *			< 0			error code
 *
 ****************************************************************************/
static UINT16 GetFmBlockFnumMa2
(
	SINT32	seq_id,						/* sequence id */
	UINT8	ch,							/* channel number */
	UINT32	key,						/* key number */
	UINT16	drum_key					/* key number for drum */
)
{
	UINT16	pitch;						/* pitch = block&fnum */
	UINT32	block;						/* block number */
	UINT32	fnum;						/* f-number */
	SINT32	keycon;

	(void)drum_key;

	MASMW_ASSERT( seq_id <= MASMW_SEQTYPE_DIRECT );
	MASMW_ASSERT( key >= 0x80000000 );

	/* compatible mode */
		
	block = (key >> 10) & 0x0007;
	fnum  =  key        & 0x03FF;

	if (ma_channel_info[ch].key_control == 2) {
		keycon = ma_snddrv_info[seq_id].key_offset + 12;
		fnum *= ma_ma2ex_fnum[keycon];
	}
	else
		fnum *= 0x10911;	/* 49700/48000 */
	fnum >>= 16;
	while ( fnum > 1023 )
	{
		fnum >>= 1;
		if ( block < 7 ) block++;
	}
		
	pitch = (UINT16)( ((fnum  << 3) & 0x1C00)
		            | ((block << 7) & 0x0380)
		            | ( fnum        & 0x007F) );

	return pitch;
}
/****************************************************************************
 *	GetWtBlockFnum
 *
 *	Description:
 *			Return Block:Fnum value for WT slot.
 *	Arguments:
 *			seq_id		sequence id number
 *			ch			channel number (0..15)
 *			key			key number (0..127)
 *			base_fs		base fequency
 *	Return:
 *			0			success
 *			< 0			error code
 *
 ****************************************************************************/
static UINT16 GetWtBlockFnum
(
	SINT32	seq_id,						/* sequence id number */
	UINT8	ch,							/* channel number */
	UINT32	key,						/* key number */
	UINT16	base_fs						/* base frequency */
)
{
	UINT8	rkey;						/* real key number */
	UINT16	pitch;						/* pitch = block&fnum */
	UINT16	rate;						/* rate */
	UINT32	scale;						/* scale */
	UINT32	block;						/* block */
	UINT32	fnum = 0;					/* f number */

	MASMW_ASSERT( seq_id <= MASMW_SEQTYPE_DIRECT );

	if ( key >= 0x80000000 )
	{
		return 0;
	}

	if ( ma_channel_info[ch].bank_no < 128 )
	{

		rkey = GetRealKeyMelody( seq_id, ch, (UINT8)key & 0x7F );

		switch ( ma_snddrv_info[seq_id].melody_mode )
		{
		case 0:
			
			switch ( ma_channel_info[ch].prog_no )
			{
			case 115:	rate = ma_wt_block_50a[rkey];		break;
			case 116:	rate = ma_wt_block_50b[rkey];		break;
			case 117:	rate = ma_wt_block_50c[rkey];		break;
			case 118:
			case 119:	rate = ma_wt_block_50[rkey];		break;
			case 122:
			case 127:	rate = ma_wt_block_20[rkey];		break;
			case 123:
			case 126:	rate = ma_wt_block_5[rkey];			break;
			case 124:
			case 125:	rate = ma_wt_block_10[rkey];		break;
			default:	rate = ma_wt_block_100[rkey];		break;
			}
			break;
			
		case 1:
		default:
			rate = ma_wt_block_100[rkey];
			break;
		}
	}
	else
	{
		rkey = GetRealKeyDrum( seq_id, ch, (UINT8)60 );

		rate = ma_wt_block_100[rkey];
	}

	scale = base_fs;								/* base frequency [Hz] */
													/* 1/48[kHz] = 1.0 (0x0400) */
	scale *= rate;									/* [12.20] = [6.10] * [6.10] */
	scale >>= 10;									/* [22.10] */
	if ( scale > 0x0400 ) scale = 0x0400;
	if ( scale < 0x0020 ) scale = 0x0020;

	for ( block = 0; block < 6; block++ )
	{
		fnum = (SINT32)( scale << (5 - block) ) - 1024;
		if ( fnum < 1024 )
			break;
	}

	pitch = (UINT16)( ((fnum  << 3) & 0x1C00)
	                | ((block << 7) & 0x0380)
		            | ( fnum        & 0x007F) );

	return pitch;
}
/****************************************************************************
 *	MakeStreamAudioParam
 *
 *	Description:
 *			Make parameter of specified stream audio.
 *	Arguments:
 *			seq_id			sequence id (0..2)
 *			ch				channel number (0..15)
 *			wave_id			wave id number (0..63)
 *			velocity		velocity value (0..127)
 *			stream_param	stream parameter of stream audio
 *	Return:
 *			0				success
 *			< 0				error
 *
 ****************************************************************************/
static SINT32 MakeStreamAudioParam
(
	SINT32	seq_id,						/* sequence id */
	UINT32	ch,							/* channel number */
	UINT32	wave_id,					/* wave id */
	UINT32	velocity,					/* velocity */
	PMASTREAMPARAM	stream_param		/* stream audio parameter */
)
{
	UINT8	format;
	UINT8	panpot;
	UINT16	db;
	UINT8	sa_id = 0;
	UINT8	vo_volume = 0;
	static UINT8 wave_mode[4] = { 0, 0, 2, 3 };
	
	(void)ch;							/* for unused warning message */
	
	if ( ma_stream_info[seq_id][wave_id].frequency == 0 )
	{
		return MASMW_ERROR;
	}
	
	format = ma_stream_info[seq_id][wave_id].format;
	panpot = ma_stream_info[seq_id][wave_id].pan;
	
	ma_snddrv_info[seq_id].wave_velocity = (UINT8)velocity;
	
	db = db_table[velocity];
	velocity = vol_table[db];

	if ( seq_id == MASMW_SEQTYPE_AUDIO )
	{
		sa_id = 0;
		vo_volume = CalcVoiceVolume( seq_id, velocity, (UINT8)0 );
	}
	else
	{
		switch ( ma_snddrv_info[seq_id].dva_mode )
		{
		case 0:
		case 1:
			sa_id  = (UINT8)( GetSlotList( seq_id, 2 ) - 40 );
			if ( sa_id == 0xFF )
			{
				return MASMW_ERROR;
			}
			vo_volume = CalcVoiceVolume( seq_id, velocity, (UINT8)(12 << 1) );	/* +12dB */
			break;
			
		case 2:
			sa_id = 0;
			vo_volume = CalcVoiceVolume( seq_id, velocity, (UINT8)0 );
			break;
		}
	}

	stream_param->panpot	= (UINT8)( (panpot & 0x7C) << 1 );
	stream_param->stm		= (UINT8)( 1 << 1 );
	stream_param->pe		= (UINT8)( ( panpot == 255 ) ? 0 : 1 );
	if ( ( panpot == 128 ) || ( format == 0 ) )
		stream_param->panoff = (UINT8)(1 << 2);
	else
		stream_param->panoff = 0;
	stream_param->mode		= wave_mode[format];
	stream_param->end_point	= (UINT16)( ( format < 2 ) ? MASMW_STMADPCM_ENDPOINT :
											             MASMW_STMPCM_ENDPOINT );
	stream_param->sa_id		= sa_id;
	stream_param->vo_volume	= vo_volume;
	stream_param->pitch		= GetStreamBlockFnum( seq_id, (UINT8)wave_id );

	return MASMW_SUCCESS;
}
/****************************************************************************
 *	AlternateAssign
 *
 *	Description:
 *			Alternate assign.
 *	Arguments:
 *			seq_id		sequence id (0..2)
 *			delta_time	delta time [tick]
 *			key			key number (0..127)
 *	Return:
 *			channged delta time
 *
 ****************************************************************************/
static SINT32 AlternateAssign
(
	SINT32	seq_id,						/* sequence id */
	SINT32	delta_time,					/* delta time */
	UINT32	key							/* key number */
)
{
	UINT8	ch;							/* channel number */
	UINT8	alt_assign;					/* alternate assign flag */
	SINT32	dt0;						/* delta time 0 */
	SINT32	dt1;						/* delta time 1 */
	

	alt_assign = 0;

	dt1 = ( delta_time >= 0 ) ? (SINT32)0 : (SINT32)(-1);

	for ( ch = 0; ch < MASMW_NUM_CHANNEL; ch++ )
	{
		if ( ma_channel_info[ch].bank_no >= 128 )
		{
			if ( alt_assign == 0 )
			{
				dt0 = delta_time;
			}
			else
			{
				dt0 = ( delta_time >= 0 ) ? (SINT32)0 : (SINT32)(-1);
			}

			alt_assign++;
			
			switch ( key )
			{
			case 42:	MaSndDrv_NoteOff( seq_id, dt0, ch, 44, 0 );
						MaSndDrv_NoteOff( seq_id, dt1, ch, 46, 0 );	break;
			case 44:	MaSndDrv_NoteOff( seq_id, dt0, ch, 42, 0 );
						MaSndDrv_NoteOff( seq_id, dt1, ch, 46, 0 );	break;
			case 46:	MaSndDrv_NoteOff( seq_id, dt0, ch, 42, 0 );
						MaSndDrv_NoteOff( seq_id, dt1, ch, 44, 0 );	break;
			case 71:	MaSndDrv_NoteOff( seq_id, dt0, ch, 72, 0 );	break;
			case 72:	MaSndDrv_NoteOff( seq_id, dt0, ch, 71, 0 );	break;
			case 73:	MaSndDrv_NoteOff( seq_id, dt0, ch, 74, 0 );	break;
			case 74:	MaSndDrv_NoteOff( seq_id, dt0, ch, 73, 0 );	break;
			case 78:	MaSndDrv_NoteOff( seq_id, dt0, ch, 79, 0 );	break;
			case 79:	MaSndDrv_NoteOff( seq_id, dt0, ch, 78, 0 );	break;
			case 80:	MaSndDrv_NoteOff( seq_id, dt0, ch, 81, 0 );	break;
			case 81:	MaSndDrv_NoteOff( seq_id, dt0, ch, 80, 0 );	break;
			default:	if ( alt_assign != 0 ) alt_assign--;		break;
			}
		}
	}

	if ( alt_assign != 0 )
	{
		delta_time = ( delta_time >= 0 ) ? (SINT32)0 : (SINT32)(-1);
	}

	return delta_time;
}

/*--------------------------------------------------------------------------*/

/****************************************************************************
 *	MaSndDrv_UpdateSequence
 *
 *	Description:
 *			Update sequence data.
 *	Arguments:
 *			seq_id		sequence id number
 *			buf_ptr		pointer to the buffer
 *			buf_size	size of the buffer
 *	Return:
 *			0			success
 *			< 0			error code
 *
 ****************************************************************************/
SINT32 MaSndDrv_UpdateSequence
(
	SINT32	seq_id,						/* sequence id */
	UINT8 *	buf_ptr,					/* pointer to the buffer */
	UINT32	buf_size					/* size of the buffer */
)
{
	UINT16	time_out;					/* time out count */
	UINT32	read_ptr;
	SINT32	message;					/* message */
	SINT32	result;						/* result of function */

	static UINT8 reset[8] = { (  0x00					 | 0x80 ),
							  (  MA_TIMER_1_CTRL		 & 0x7F ),
							  ( (MA_TIMER_1_CTRL >> 7)   | 0x80 ),
							  (  0x04					 | 0x80 ),
							  (  0x00					 | 0x80 ),
							  (  MA_TIMER_1_CTRL		 & 0x7F ),
							  ( (MA_TIMER_1_CTRL >> 7)   | 0x80 ),
							  (  0x05                    | 0x80 ) };

	MASNDDRV_DBGMSG(("MaSndDrv_UpdateSequence\n"));

	if ( ma_snddrv_info[seq_id].status == 0 )
	{
		return 0;
	}
	
	switch ( seq_id )
	{
	case MASMW_SEQTYPE_DELAYED:

		ma_seqbuf_info.buf_ptr  = buf_ptr;
		ma_seqbuf_info.buf_size = buf_size;

		if ( ma_seqbuf_info.queue_flag == 1 )
		{
			read_ptr = ma_seqbuf_info.queue_wptr - ma_seqbuf_info.queue_size;

			if ( buf_size < ma_seqbuf_info.queue_size )
			{
				machdep_memcpy( ma_seqbuf_info.buf_ptr, &ma_seqbuf_info.queue[read_ptr], buf_size );

				ma_seqbuf_info.queue_size -= buf_size;
				
				return (SINT32)buf_size;
			}
			else
			{
				machdep_memcpy( ma_seqbuf_info.buf_ptr, &ma_seqbuf_info.queue[read_ptr], ma_seqbuf_info.queue_size );

				ma_seqbuf_info.buf_ptr   += ma_seqbuf_info.queue_size;
				ma_seqbuf_info.buf_size  -= ma_seqbuf_info.queue_size;
				ma_seqbuf_info.wrote_size = ma_seqbuf_info.queue_size;

				ma_seqbuf_info.queue_size = 0;
				ma_seqbuf_info.queue_flag = 0;
				ma_seqbuf_info.queue_wptr = 0;
			}
		}
		else
		{
			ma_seqbuf_info.wrote_size = 0;
		}

		time_out = 1024;
		do
		{
			result = ma_srm_cnv[seq_id]();
		}
		while ( ( result > 0 ) && ( ma_seqbuf_info.queue_flag == 0 ) && ( --time_out !=0 ) );

		if ( ( result == 0 ) || ( time_out == 0 ) )
		{
			message = MaSound_ReceiveMessage( seq_id, 0, MASMW_END_OF_DATA );
			if ( message == MASMW_END_OF_SEQUENCE )
			{
				MaDevDrv_EndOfSequence();
			}
			else
			{
				SendDelayedPacket( reset, 8 );
			}
		}

		break;

	case MASMW_SEQTYPE_DIRECT:

		result = ma_srm_cnv[seq_id]();
		
		break;
		
	default:
	
		result = 1;
		break;
	}

	if ( result >= 0 )
	{
		result = (SINT32)ma_seqbuf_info.wrote_size;
	}

	return result;
}
/****************************************************************************
 *	MaSndDrv_Nop
 *
 *	Description:
 *			Nop operation.
 *	Arguments:
 *			seq_id		sequence id (0..2)
 *			dalta_time	delta time [tick]
 *			type		NOP type (0..1)
 *			p2			unused
 *			p3			unused
 *	Return:
 *			0			success
 *			< 0			error code
 *
 ****************************************************************************/
static SINT32 MaSndDrv_Nop
(
	SINT32	seq_id,						/* sequence id */
	SINT32	delta_time,					/* delta time */
	UINT32	type,						/* NOP type */
	UINT32	p2,							/* unused */
	UINT32	p3							/* unused */
)
{
	UINT32	num;						/* number of packet data */
	UINT32	reg_index;					/* register index number */
	SINT32	result;						/* result of function */
	
	(void)p2;							/* for unused warning message */
	(void)p3;
	
	MASNDDRV_DBGMSG((" MaSndDrv_Nop: id=%ld dt=%ld tp=%ld\n", seq_id, delta_time, type));

	/* check arguments */

	MASMW_ASSERT( type <= MASMW_MAX_NOPTYPE );

	/* Make packet data */

	num = MakeDeltaTime( &packet_buf[seq_id][0], delta_time );			/* timer part */

	reg_index = (UINT32)( ( type == 0 ) ? MA_NOP_1 : MA_NOP_2 );

	packet_buf[seq_id][num++] = (UINT8)(  reg_index       & 0x7F );		/* address part */
	packet_buf[seq_id][num++] = (UINT8)( (reg_index >> 7) | 0x80 );
	packet_buf[seq_id][num++] = (UINT8)0x80;							/* data part */
	
	if ( delta_time < 0 )				/* direct command */
	{
		result = MaDevDrv_SendDirectPacket( &packet_buf[seq_id][0], num );
	}
	else								/* delayed command */
	{
		result = SendDelayedPacket( &packet_buf[seq_id][0], num );
	}

	return result;
}
/****************************************************************************
 *	MaSndDrv_UserEvent
 *
 *	Description:
 *			User Event operation
 *	Arguments:
 *			seq_id		sequence id (0..2)
 *			delta_time	delta time [tick]
 *			val			event value (0..127)
 *			p2			unused
 *			p3			unused
 *	Return:
 *			Number of byte data of created packet
 *
 ****************************************************************************/
static SINT32 MaSndDrv_UserEvent
(
	SINT32	seq_id,						/* sequence id */
	SINT32	delta_time,					/* delta time */
	UINT32	val,						/* value */
	UINT32	p2,							/* unused */
	UINT32	p3							/* unused */
)
{
	UINT8	sint_num;					/* software interrupt number */
	UINT32	num;						/* number of packet data */
	UINT32	reg_index;					/* register index number */
	SINT32	result;						/* result of function */

	(void)p2;							/* for unused warning message */
	(void)p3;

	MASNDDRV_DBGMSG((" MaSndDrv_UserEvent: id=%ld dt=%ld val=%ld\n", seq_id, delta_time, val));

	/* check arguments */

	MASMW_ASSERT( val <= MASMW_MAX_USEREVENT );

	/* Make packet data */

	num = MakeDeltaTime( &packet_buf[seq_id][0], delta_time );			/* timer part */

	if ( val == MASMW_END_OF_SEQUENCE )
	{
		reg_index = (UINT32)(MA_SOFTINT_RAM + MA_SEQEVENT_INT);
		sint_num = (UINT8)(1 << MA_SEQEVENT_INT);
	}
	else
	{
		reg_index = (UINT32)(MA_SOFTINT_RAM + MA_EVENT_INT);
		sint_num = (UINT8)(1 << MA_EVENT_INT);
	}

	packet_buf[seq_id][num++] = (UINT8)(  reg_index       & 0x7F );		/* address part */
	packet_buf[seq_id][num++] = (UINT8)( (reg_index >> 7) | 0x80 );
	packet_buf[seq_id][num++] = (UINT8)(  val             | 0x80 );		/* data part */

	if ( delta_time >= 0 ) packet_buf[seq_id][num++] = (UINT8)0x80;		/* timer part */

	reg_index = (UINT32)MA_SOFTINT_CTRL;

	packet_buf[seq_id][num++] = (UINT8)(  reg_index       & 0x7F );		/* address part */
	packet_buf[seq_id][num++] = (UINT8)( (reg_index >> 7) | 0x80 );
	packet_buf[seq_id][num++] = (UINT8)(  sint_num        | 0x80 );		/* data part */

	if ( delta_time < 0 )				/* direct command */
	{
		result = MaDevDrv_SendDirectPacket( &packet_buf[seq_id][0], num );
	}
	else								/* delayed command */
	{
		result = SendDelayedPacket( &packet_buf[seq_id][0], num );
	}

	return result;
}
/****************************************************************************
 *	MaSndDrv_NoteOff
 *
 *	Description:
 *			Note Off processing
 *	Arguments:
 *			seq_id		sequence id (0..2)
 *			delta_time	delta time [tick]
 *			ch			channel number (0..15)
 *			key			key number (0..127)
 *			p3			unused
 *	Return:
 *			0			success
 *			< 0			error code
 *
 ****************************************************************************/
static SINT32 MaSndDrv_NoteOff
(
	SINT32	seq_id,						/* sequence id */
	SINT32	delta_time,					/* delta time */
	UINT32	ch,							/* channel number */
	UINT32	key,						/* key number */
	UINT32	p3							/* unused */
)
{
	UINT32	slot_no = 0xFF;				/* slot number */
	UINT32	num = 0;					/* number of packet data */
	UINT32	reg_index;					/* register index number */
	UINT16	pitch;						/* pitch */
	SINT32	result;						/* result of function */
	MAVINFO	voice_info;					/* voice information */

	(void)p3;							/* for unused warning message */
	
	MASNDDRV_DBGMSG((" MaSndDrv_NoteOff: id=%ld dt=%ld ch=%ld key=%ld\n", seq_id, delta_time, ch, key));

	/* check arguments */

	MASMW_ASSERT( ch <= MASMW_MAX_CHANNEL );
	MASMW_ASSERT( key <= MASMW_MAX_KEY );
	MASMW_ASSERT( ma_snddrv_info[seq_id].dva_mode != 2 );

	/* get voice information */

	result = GetVoiceInfo( seq_id, ch, key, &voice_info );
	if ( result < MASMW_SUCCESS )
	{
		result = MaSndDrv_Nop( seq_id, delta_time, 0, 0, 0 );
		return result;
	}

	if ( ma_channel_info[ch].poly == MA_MODE_POLY )
	{
		slot_no = SearchSlot( seq_id, voice_info.synth, (UINT8)ch, key );
	}
	else
	{
		if ( ma_channel_info[ch].note_on != 0 )
		{
			ma_channel_info[ch].note_on--;
			if ( ma_channel_info[ch].note_on != 0 )
			{
				result = MaSndDrv_Nop( seq_id, delta_time, 0, 0, 0 );
				return result;
			}

			slot_no = ma_channel_info[ch].slot_no;
			RemakeSlotList( seq_id, (UINT8)(voice_info.synth - 1), slot_no, 0x80 );
		}
	}

	if ( slot_no < 16 )
	{
		reg_index = (UINT32)(((slot_no -  0) * 6) + MA_FM_VOICE_ADDRESS);
		pitch = GetFmBlockFnum( seq_id, (UINT8)ch, key, voice_info.key );
	}
	else if ( slot_no < 32 )
	{
		reg_index = (UINT32)(((slot_no - 16) * 6) + MA_EXT_FM_VOICE_ADDRESS);
		pitch = GetFmBlockFnum( seq_id, (UINT8)ch, key, voice_info.key );
	}
	else if ( slot_no < 40 )
	{
		reg_index = (UINT32)(((slot_no - 32) * 6) + MA_WT_VOICE_ADDRESS);
		pitch = GetWtBlockFnum( seq_id, (UINT8)ch, key, voice_info.key );
	}
	else
	{
		if ( delta_time > 0 )
			result = MaSndDrv_Nop( seq_id, delta_time, 0, 0, 0 );
		else
			result = MASMW_SUCCESS;

		return result;
	}

	/* Create packet data */

	num = MakeDeltaTime( &packet_buf[seq_id][0], delta_time );

	if ( delta_time <= 0 )
	{
		/* NOP2 */
		MAKE_NOP( seq_id, num, MA_NOP_2 )
		MAKE_TIMER_PART( seq_id, num, delta_time )
	}

	/* KeyOff */
	reg_index += 5;
	MAKE_ADDRESS_PART( seq_id, num, reg_index )
	MAKE_KEY_OFF( seq_id, num, ch )

	/* Send packet data */

	if ( delta_time < 0 )				/* direct command */
	{
		result = MaDevDrv_SendDirectPacket( &packet_buf[seq_id][0], num );
	}
	else								/* delayed command */
	{
		result = SendDelayedPacket( &packet_buf[seq_id][0], num );
	}

	/* update slot information */

	ma_slot_info[slot_no].ch	= (UINT8)(ch + 1);
	ma_slot_info[slot_no].key	= key;

	return result;
}
/****************************************************************************
 *	MaSndDrv_NoteOffMa2
 *
 *	Description:
 *			Note Off processing
 *	Arguments:
 *			seq_id		sequence id (0..2)
 *			delta_time	delta time [tick]
 *			ch			channel number (0..15)
 *			key			key number
 *			p3			unused
 *	Return:
 *			Number of byte data of created packet
 *
 ****************************************************************************/
static SINT32 MaSndDrv_NoteOffMa2
(
	SINT32	seq_id,						/* sequence id */
	SINT32	delta_time,					/* delta time */
	UINT32	ch,							/* channel number */
	UINT32	key,						/* key number */
	UINT32	p3							/* unused */
)
{
	UINT32	slot_no = 0xFF;				/* slot number */
	UINT32	num = 0;					/* number of packet data */
	UINT32	reg_index;					/* register index number */
	UINT16	pitch;						/* pitch */
	SINT32	result;						/* result of function */
	MAVINFO	voice_info;					/* voice information */

	(void)p3;							/* for unused warning message */
	
	MASNDDRV_DBGMSG((" MaSndDrv_NoteOffMa2: id=%ld dt=%ld ch=%ld key=%ld\n", seq_id, delta_time, ch, key));

	/* check arguments */

	MASMW_ASSERT( ch <= MASMW_MAX_CHANNEL );
	MASMW_ASSERT( key <= MASMW_MAX_KEY );
	MASMW_ASSERT( ma_snddrv_info[seq_id].dva_mode == 2 );

	/* get voice information */

	result = GetVoiceInfo( seq_id, ch, key, &voice_info );
	if ( result < MASMW_SUCCESS )
	{
		result = MaSndDrv_Nop( seq_id, delta_time, 0, 0, 0 );
		return result;
	}


	slot_no = SearchSlotMa2( seq_id, voice_info.synth, (UINT8)ch, key );
	ma_channel_info[ch].note_on = 0;

	if ( slot_no < 16 )
	{
		reg_index = (UINT32)(((slot_no -  0) * 6) + MA_FM_VOICE_ADDRESS);
		pitch = GetFmBlockFnum( seq_id, (UINT8)ch, key, voice_info.key );
	}
	else if ( slot_no < 32 )
	{
		reg_index = (UINT32)(((slot_no - 16) * 6) + MA_EXT_FM_VOICE_ADDRESS);
		pitch = GetFmBlockFnum( seq_id, (UINT8)ch, key, voice_info.key );
	}
	else if ( slot_no < 40 )
	{
		reg_index = (UINT32)(((slot_no - 32) * 6) + MA_WT_VOICE_ADDRESS);
		pitch = GetWtBlockFnum( seq_id, (UINT8)ch, key, voice_info.key );
	}
	else
	{
		result = MaSndDrv_Nop( seq_id, delta_time, 0, 0, 0 );
		return result;
	}

	/* Create packet data */

	num = MakeDeltaTime( &packet_buf[seq_id][0], delta_time );

	if ( delta_time <= 0 )
	{
		/* NOP2 */
		MAKE_NOP( seq_id, num, MA_NOP_2 )
		MAKE_TIMER_PART( seq_id, num, delta_time )
	}

	/* KeyOff */
	reg_index += 5;
	MAKE_ADDRESS_PART( seq_id, num, reg_index )
	MAKE_KEY_OFF( seq_id, num, ch )

	/* Send packet data */

	if ( delta_time < 0 )				/* direct command */
	{
		result = MaDevDrv_SendDirectPacket( &packet_buf[seq_id][0], num );
	}
	else								/* delayed command */
	{
		result = SendDelayedPacket( &packet_buf[seq_id][0], num );
	}

	/* update slot information */

	ma_slot_info[slot_no].ch	= (UINT8)(ch + 1);
	ma_slot_info[slot_no].key	= key;

	return result;
}
/****************************************************************************
 *	MaSndDrv_NoteOffMa2Ex
 *
 *	Description:
 *			Note Off processing
 *	Arguments:
 *			seq_id		sequence id (0..2)
 *			delta_time	delta time [tick]
 *			ch			channel number (0..15)
 *			key			key number (0x8000xxxx)
 *			p3			unused
 *	Return:
 *			Number of byte data of created packet
 *
 ****************************************************************************/
static SINT32 MaSndDrv_NoteOffMa2Ex
(
	SINT32	seq_id,						/* sequence id */
	SINT32	delta_time,					/* delta time */
	UINT32	ch,							/* channel number */
	UINT32	key,						/* key number */
	UINT32	p3							/* unused */
)
{
	UINT32	slot_no = 0xFF;				/* slot number */
	UINT32	num = 0;					/* number of packet data */
	UINT32	reg_index;					/* register index number */
	UINT16	pitch;						/* pitch */
	SINT32	result;						/* result of function */
	MAVINFO	voice_info;					/* voice information */

	(void)p3;							/* for unused warning message */
	
	MASNDDRV_DBGMSG((" MaSndDrv_NoteOffMa2Ex: id=%ld dt=%ld ch=%ld key=%ld\n", seq_id, delta_time, ch, key));

	/* check arguments */

	MASMW_ASSERT( ch <= MASMW_MAX_CHANNEL );
	MASMW_ASSERT( ma_snddrv_info[seq_id].dva_mode == 2 );

	/* get voice information */

	result = GetVoiceInfo( seq_id, ch, key, &voice_info );
	if ( result < MASMW_SUCCESS )
	{
		result = MaSndDrv_Nop( seq_id, delta_time, 0, 0, 0 );
		return result;
	}


	slot_no = SearchSlotMa2( seq_id, voice_info.synth, (UINT8)ch, key );
	ma_channel_info[ch].note_on = 0;


	if ( slot_no < 16 )
	{
		reg_index = (UINT32)(((slot_no -  0) * 6) + MA_FM_VOICE_ADDRESS);
		pitch = GetFmBlockFnumMa2( seq_id, (UINT8)ch, key, voice_info.key );
	}
	else if ( slot_no < 32 )
	{
		reg_index = (UINT32)(((slot_no - 16) * 6) + MA_EXT_FM_VOICE_ADDRESS);
		pitch = GetFmBlockFnumMa2( seq_id, (UINT8)ch, key, voice_info.key );
	}
	else if ( slot_no < 40 )
	{
		reg_index = (UINT32)(((slot_no - 32) * 6) + MA_WT_VOICE_ADDRESS);
		pitch = GetWtBlockFnum( seq_id, (UINT8)ch, key, voice_info.key );
	}
	else
	{
		result = MaSndDrv_Nop( seq_id, delta_time, 0, 0, 0 );
		return result;
	}

	/* Create packet data */

	num = MakeDeltaTime( &packet_buf[seq_id][0], delta_time );

	if ( delta_time <= 0 )
	{
		/* NOP2 */
		MAKE_NOP( seq_id, num, MA_NOP_2 )
		MAKE_TIMER_PART( seq_id, num, delta_time )
	}

	/* KeyOff */
	reg_index += 5;
	MAKE_ADDRESS_PART( seq_id, num, reg_index )
	MAKE_KEY_OFF( seq_id, num, ch )

	/* pitch */
	reg_index -= 2;
	MAKE_TIMER_PART( seq_id, num, delta_time )
	MAKE_ADDRESS_PART( seq_id, num, reg_index )
	MAKE_PITCH( seq_id, num, pitch )

	/* Send packet data */

	if ( delta_time < 0 )				/* direct command */
	{
		result = MaDevDrv_SendDirectPacket( &packet_buf[seq_id][0], num );
	}
	else								/* delayed command */
	{
		result = SendDelayedPacket( &packet_buf[seq_id][0], num );
	}

	/* update slot information */

	ma_slot_info[slot_no].ch	= (UINT8)(ch + 1);
	ma_slot_info[slot_no].key	= key;

	return result;
}
/****************************************************************************
 *	MaSndDrv_NoteOn
 *
 *	Description:
 *			Note On prcessing
 *	Argument:
 *			seq_id		sequence id (0..2)
 *			delta_time	delta time [tick]
 *			ch			channel number (0..15)
 *			key			key number (0..127, 0x8xxxxxxx)
 *			velocity	velocity (0..127)
 *	Return:
 *			Number of byte data of created packet
 *
 ****************************************************************************/
static SINT32 MaSndDrv_NoteOn
(
	SINT32	seq_id,						/* sequnce id */
	SINT32	delta_time,					/* delta time */
	UINT32	ch,							/* channel number */
	UINT32	key,						/* key number */
	UINT32	velocity					/* velocity */
)
{
	UINT32	slot_no;					/* slot number */
	UINT8	old_ch;						/* old channel number */
	UINT8	vo_volume;					/* voice volume */
	UINT32	num;						/* number of packet data */
	UINT32	reg_index;					/* register index */
	UINT32	pitch;						/* pitch */
	SINT32	result;						/* result of function */
	MAVINFO	voice_info;					/* voice information */
	
	MASNDDRV_DBGMSG((" MaSndDrv_NoteOn: id=%ld dt=%ld ch=%ld key=%ld vel=%ld\n", seq_id, delta_time, ch, key, velocity));

	/* check arguments */

	MASMW_ASSERT( ch <= MASMW_MAX_CHANNEL );
	MASMW_ASSERT( key <= MASMW_MAX_KEY );
	MASMW_ASSERT( velocity <= MASMW_MAX_VELOCITY );

	/* get voice information */

	result = GetVoiceInfo( seq_id, ch, key, &voice_info );
	if ( result < MASMW_SUCCESS )
	{
		result = MaSndDrv_Nop( seq_id, delta_time, 0, 0, 0 );
		return result;
	}

	/* get slot number */

	if ( ma_channel_info[ch].poly == MA_MODE_POLY )
	{
		slot_no = GetSlotList( seq_id, voice_info.synth-1 );

		if ( ( ma_slot_info[slot_no].ch & 0x80 ) != 0 )
		{
			old_ch = (UINT8)( ( ma_slot_info[slot_no].ch - 1 ) & 0x7F );
			ma_channel_info[old_ch].note_on = 0;
		}
	}
	else
	{
		if ( ma_channel_info[ch].note_on == 0 )
		{
			slot_no = GetSlotList( seq_id, voice_info.synth-1 );
			ma_channel_info[ch].slot_no = (UINT8)slot_no;

			if ( ( ma_slot_info[slot_no].ch & 0x80 ) != 0 )
			{
				old_ch = (UINT8)( ( ma_slot_info[slot_no].ch - 1 ) & 0x7F );
				ma_channel_info[old_ch].note_on = 0;
			}
		}
		else
		{
			slot_no = ma_channel_info[ch].slot_no;
			RemakeSlotListE( seq_id, (UINT8)(voice_info.synth - 1), slot_no );
		}
	
		ma_channel_info[ch].note_on++;
	}

	/* set register index value and pitch of slot */

	if ( slot_no < 16 )
	{
		reg_index = (UINT32)(((slot_no -  0) * 6) + MA_FM_VOICE_ADDRESS);
		pitch = GetFmBlockFnum( seq_id, (UINT8)ch, key, voice_info.key );
		vo_volume = CalcVoiceVolume( seq_id, velocity, 0 );
	}
	else if ( slot_no < 32 )
	{
		reg_index = (UINT32)(((slot_no - 16) * 6) + MA_EXT_FM_VOICE_ADDRESS);
		pitch = GetFmBlockFnum( seq_id, (UINT8)ch, key, voice_info.key );
		vo_volume = CalcVoiceVolume( seq_id, velocity, 0 );
	}
	else if ( slot_no < 40 )
	{
		reg_index = (UINT32)(((slot_no - 32) * 6) + MA_WT_VOICE_ADDRESS);
		pitch = GetWtBlockFnum( seq_id, (UINT8)ch, key, voice_info.key );
		vo_volume = CalcVoiceVolume( seq_id, velocity, (12 << 1) );
	}
	else
	{
		result = MaSndDrv_Nop( seq_id, delta_time, 0, 0, 0 );
		return result;
	}

	/* alternate assign */

	if ( ma_channel_info[ch].bank_no >= 128 )
	{
		if ( ma_snddrv_info[seq_id].drum_mode == 0 )
		{
			delta_time = AlternateAssign( seq_id, delta_time, key );
		}
	}


	num = MakeDeltaTime( &packet_buf[seq_id][0], delta_time );


	if ( ma_channel_info[ch].poly == MA_MODE_POLY )
	{
		if ( ( ma_slot_info[slot_no].ch & 0x80 ) == 0 )	/* new note */
		{
			old_ch = (UINT8)ch;
		}
		else
		{
			old_ch = (UINT8)( ma_slot_info[slot_no].ch - 1 );
		}

		if ( delta_time <= 0 )
		{
			/* NOP2 */
			MAKE_NOP( seq_id, num, (UINT16)MA_NOP_2 )
			MAKE_TIMER_PART( seq_id, num, delta_time )
		}

		/* KeyOn */
		reg_index += 5;
		MAKE_ADDRESS_PART( seq_id, num, reg_index )
		MAKE_RST( seq_id, num, old_ch )
		MAKE_TIMER_PART( seq_id, num, delta_time )

		MAKE_NOP( seq_id, num, (UINT16)MA_NOP_2 )
		MAKE_TIMER_PART( seq_id, num, delta_time )

		reg_index -= 5;
		MAKE_ADDRESS_PART( seq_id, num, reg_index )
		MAKE_KEY_ON( seq_id, num, voice_info.address, vo_volume, pitch, ch )
	}
	else
	{
		if ( ma_channel_info[ch].bank_no >= 128 ) return MASMW_SUCCESS;

		if ( ma_channel_info[ch].note_on == 1 )	/* new note */
		{
			if ( delta_time <= 0 )
			{
				/* NOP2 */
				MAKE_NOP( seq_id, num, (UINT16)MA_NOP_2 )
				MAKE_TIMER_PART( seq_id, num, delta_time )
			}

			/* KeyOn */
			reg_index += 5;
			MAKE_ADDRESS_PART( seq_id, num, reg_index )
			MAKE_RST( seq_id, num, ch )
			MAKE_TIMER_PART( seq_id, num, delta_time )

			MAKE_NOP( seq_id, num, (UINT16)MA_NOP_2 )
			MAKE_TIMER_PART( seq_id, num, delta_time )

			reg_index -= 5;
			MAKE_ADDRESS_PART( seq_id, num, reg_index )
			MAKE_KEY_ON( seq_id, num, voice_info.address, vo_volume, pitch, ch )
		}
		else
		{
			/* velocity & pitch */
			reg_index += 2;
			MAKE_ADDRESS_PART( seq_id, num, reg_index )
			MAKE_VEL_PITCH( seq_id, num, vo_volume, pitch )
		}
	}

	/* Send packet data */

	if ( delta_time < 0 )				/* direct command */
	{
		result = MaDevDrv_SendDirectPacket( &packet_buf[seq_id][0], num );
	}
	else								/* delayed command */
	{
		result = SendDelayedPacket( &packet_buf[seq_id][0], num );
	}

	/* update the slot information */

	ma_slot_info[slot_no].ch	= (UINT8)( (ch + 1) | 0x80 );
	ma_slot_info[slot_no].key	= key;

	return result;
}
/****************************************************************************
 *	MaSndDrv_NoteOnMa2
 *
 *	Description:
 *			MA-2 compatible Note On prcessing
 *	Argument:
 *			seq_id		sequence id (0..2)
 *			delta_time	delta time [tick]
 *			ch			channel number (0..15)
 *			key			key number (0..127)
 *			velocity	velocity (0..127)
 *	Return:
 *			Number of byte data of created packet
 *
 ****************************************************************************/
static SINT32 MaSndDrv_NoteOnMa2
(
	SINT32	seq_id,						/* sequnce id */
	SINT32	delta_time,					/* delta time */
	UINT32	ch,							/* channel number */
	UINT32	key,						/* key number */
	UINT32	velocity					/* velocity */
)
{
	UINT32	slot_no;					/* slot number */
	UINT8	old_ch;						/* old channel number */
	UINT8	vo_volume;					/* voice volume */
	UINT32	num;						/* number of packet data */
	UINT32	reg_index;					/* register index */
	UINT32	pitch;						/* pitch */
	SINT32	result;						/* result of function */
	MAVINFO	voice_info;					/* voice information */
	
	MASNDDRV_DBGMSG((" MaSndDrv_NoteOnMa2: id=%ld dt=%ld ch=%ld key=%ld vel=%ld\n", seq_id, delta_time, ch, key, velocity));

	/* check arguments */

	MASMW_ASSERT( ch <= MASMW_MAX_CHANNEL );
	MASMW_ASSERT( key <= MASMW_MAX_KEY );
	MASMW_ASSERT( velocity <= MASMW_MAX_VELOCITY );

	/* get voice information */

	result = GetVoiceInfo( seq_id, ch, key, &voice_info );
	if ( result < MASMW_SUCCESS )
	{
		result = MaSndDrv_Nop( seq_id, delta_time, 0, 0, 0 );
		return result;
	}

	/* get slot number */

	slot_no = GetSlot( seq_id, voice_info.synth, (UINT8)ch );
	ma_channel_info[ch].note_on++;

	/* set register index value and pitch of slot */

	if ( slot_no < 16 )
	{
		reg_index = (UINT32)(((slot_no -  0) * 6) + MA_FM_VOICE_ADDRESS);
		pitch = GetFmBlockFnum( seq_id, (UINT8)ch, key, voice_info.key );
		vo_volume = CalcVoiceVolume( seq_id, velocity, 0 );
	}
	else if ( slot_no < 32 )
	{
		reg_index = (UINT32)(((slot_no - 16) * 6) + MA_EXT_FM_VOICE_ADDRESS);
		pitch = GetFmBlockFnum( seq_id, (UINT8)ch, key, voice_info.key );
		vo_volume = CalcVoiceVolume( seq_id, velocity, 0 );
	}
	else if ( slot_no < 40 )
	{
		reg_index = (UINT32)(((slot_no - 32) * 6) + MA_WT_VOICE_ADDRESS);
		pitch = GetWtBlockFnum( seq_id, (UINT8)ch, key, voice_info.key );
		vo_volume = CalcVoiceVolume( seq_id, velocity, (12 << 1) );
	}
	else
	{
		result = MaSndDrv_Nop( seq_id, delta_time, 0, 0, 0 );
		return result;
	}

	/* update channel information */

	old_ch = (UINT8)( ma_slot_info[slot_no].ch - 1 );


	num = MakeDeltaTime( &packet_buf[seq_id][0], delta_time );


	if ( delta_time <= 0 )
	{
		/* NOP2 */
		MAKE_NOP( seq_id, num, (UINT16)MA_NOP_2 )
		MAKE_TIMER_PART( seq_id, num, delta_time )
	}

	if ( ma_channel_info[ch].bank_no >= 128 )
	{
		/* KeyOff & RST */
		reg_index += 5;
		MAKE_ADDRESS_PART( seq_id, num, reg_index )
		MAKE_RST( seq_id, num, ch )
		MAKE_TIMER_PART( seq_id, num, delta_time )
		reg_index -= 5;

		/* NOP2 */
		MAKE_NOP( seq_id, num, (UINT16)MA_NOP_2 )
		MAKE_TIMER_PART( seq_id, num, delta_time )
	}

	/* KeyOn */

	if ( ma_channel_info[ch].note_on == 1 )		/* new note */
	{
		MAKE_ADDRESS_PART( seq_id, num, reg_index )
		MAKE_KEY_ON( seq_id, num, voice_info.address, vo_volume, pitch, ch )
	}
	else										/* continuouse note */
	{
		reg_index += 2;
		MAKE_ADDRESS_PART( seq_id, num, reg_index )
		MAKE_VEL_PITCH( seq_id, num, vo_volume, pitch )
	}

	/* Send packet data */

	if ( delta_time < 0 )				/* direct command */
	{
		result = MaDevDrv_SendDirectPacket( &packet_buf[seq_id][0], num );
	}
	else								/* delayed command */
	{
		result = SendDelayedPacket( &packet_buf[seq_id][0], num );
	}

	/* update the slot information */

	ma_slot_info[slot_no].ch	= (UINT8)( (ch + 1) | 0x80 );
	ma_slot_info[slot_no].key	= key;

	return result;
}
/****************************************************************************
 *	MaSndDrv_NoteOnMa2Ex
 *
 *	Description:
 *			MA-2 compatible Note On prcessing
 *	Argument:
 *			seq_id		sequence id (0..2)
 *			delta_time	delta time [tick]
 *			ch			channel number (0..15)
 *			key			key number (0..127)
 *			velocity	velocity (0..127)
 *	Return:
 *			Number of byte data of created packet
 *
 ****************************************************************************/
static SINT32 MaSndDrv_NoteOnMa2Ex
(
	SINT32	seq_id,						/* sequnce id */
	SINT32	delta_time,					/* delta time */
	UINT32	ch,							/* channel number */
	UINT32	key,						/* key number */
	UINT32	velocity					/* velocity */
)
{
	UINT32	slot_no;					/* slot number */
	UINT8	old_ch;						/* old channel number */
	UINT8	vo_volume;					/* voice volume */
	UINT32	num;						/* number of packet data */
	UINT32	reg_index;					/* register index */
	UINT32	pitch;						/* pitch */
	SINT32	result;						/* result of function */
	MAVINFO	voice_info;					/* voice information */
	
	MASNDDRV_DBGMSG((" MaSndDrv_NoteOnMa2Ex: id=%ld dt=%ld ch=%ld key=%ld vel=%ld\n", seq_id, delta_time, ch, key, velocity));

	/* check arguments */

	MASMW_ASSERT( ch <= MASMW_MAX_CHANNEL );
	MASMW_ASSERT( velocity <= MASMW_MAX_VELOCITY );

	/* get voice information */

	result = GetVoiceInfo( seq_id, ch, key, &voice_info );
	if ( result < MASMW_SUCCESS )
	{
		result = MaSndDrv_Nop( seq_id, delta_time, 0, 0, 0 );
		return result;
	}

	/* get slot number */

	slot_no = GetSlot( seq_id, voice_info.synth, (UINT8)ch );
	ma_channel_info[ch].note_on++;

	/* set register index value and pitch of slot */

	if ( slot_no < 16 )
	{
		reg_index = (UINT32)(((slot_no -  0) * 6) + MA_FM_VOICE_ADDRESS);
		pitch = GetFmBlockFnumMa2( seq_id, (UINT8)ch, key, voice_info.key );
		vo_volume = CalcVoiceVolume( seq_id, velocity, 0 );
	}
	else if ( slot_no < 32 )
	{
		reg_index = (UINT32)(((slot_no - 16) * 6) + MA_EXT_FM_VOICE_ADDRESS);
		pitch = GetFmBlockFnumMa2( seq_id, (UINT8)ch, key, voice_info.key );
		vo_volume = CalcVoiceVolume( seq_id, velocity, 0 );
	}
	else if ( slot_no < 40 )
	{
		reg_index = (UINT32)(((slot_no - 32) * 6) + MA_WT_VOICE_ADDRESS);
		pitch = GetWtBlockFnum( seq_id, (UINT8)ch, key, voice_info.key );
		vo_volume = CalcVoiceVolume( seq_id, velocity, (12 << 1) );
	}
	else
	{
		result = MaSndDrv_Nop( seq_id, delta_time, 0, 0, 0 );
		return result;
	}

	/* update channel information */

	old_ch = (UINT8)( ma_slot_info[slot_no].ch - 1 );


	num = MakeDeltaTime( &packet_buf[seq_id][0], delta_time );


	if ( delta_time <= 0 )
	{
		/* NOP2 */
		MAKE_NOP( seq_id, num, (UINT16)MA_NOP_2 )
		MAKE_TIMER_PART( seq_id, num, delta_time )
	}

	if ( ma_channel_info[ch].bank_no >= 128 )
	{
		/* KeyOff & RST */
		reg_index += 5;
		MAKE_ADDRESS_PART( seq_id, num, reg_index )
		MAKE_RST( seq_id, num, ch )
		MAKE_TIMER_PART( seq_id, num, delta_time )
		reg_index -= 5;

		/* NOP2 */
		MAKE_NOP( seq_id, num, (UINT32)MA_NOP_2 )
		MAKE_TIMER_PART( seq_id, num, delta_time )
	}

	/* KeyOn */

	if ( ma_channel_info[ch].note_on == 1 )		/* new note */
	{
		MAKE_ADDRESS_PART( seq_id, num, reg_index )
		MAKE_KEY_ON( seq_id, num, voice_info.address, vo_volume, pitch, ch )
	}
	else										/* continuouse note */
	{
		reg_index += 2;
		MAKE_ADDRESS_PART( seq_id, num, reg_index )
		MAKE_VEL_PITCH( seq_id, num, vo_volume, pitch )
	}

	/* Send packet data */

	if ( delta_time < 0 )				/* direct command */
	{
		result = MaDevDrv_SendDirectPacket( &packet_buf[seq_id][0], num );
	}
	else								/* delayed command */
	{
		result = SendDelayedPacket( &packet_buf[seq_id][0], num );
	}

	/* update the slot information */

	ma_slot_info[slot_no].ch	= (UINT8)( (ch + 1) | 0x80 );
	ma_slot_info[slot_no].key	= key;

	return result;
}
/****************************************************************************
 *	MaSndDrv_ProgramChange
 *
 *	Description:
 *			Sets the bank number and program number of specified channel.
 *	Arguments:
 *			seq_id			sequence id (0..2)
 *			delta_time		delta time [tick]
 *			ch				channel number (0..15)
 *			bank_no			bank number (0..255)
 *			program_no		program number (0..127)
 *	Return:
 *			Number of byte data of created packet
 *
 ****************************************************************************/
static SINT32 MaSndDrv_ProgramChange
(
	SINT32	seq_id,						/* sequence id */
	SINT32	delta_time,					/* delta time */
	UINT32	ch,							/* channel number */
	UINT32	bank_no,					/* bank number */
	UINT32	program_no					/* program number */
)
{
	SINT32	result;						/* result of function */
	
	(void)seq_id;						/* for unused warning message */
	(void)delta_time;
	
	MASNDDRV_DBGMSG((" MaSndDrv_ProgramChange: id=%ld dt=%ld ch=%ld bn=%ld pn=%ld\n", seq_id, delta_time, ch, bank_no, program_no ));

	/* check arguments */

	MASMW_ASSERT( ch <= MASMW_MAX_CHANNEL );
	MASMW_ASSERT( bank_no <= MASMW_MAX_BANK );
	MASMW_ASSERT( program_no <= MASMW_MAX_PROGRAM );

	/* update channel information */

	ma_channel_info[ch].bank_no = (UINT8)( bank_no & MASMW_MASK_BANK );
	ma_channel_info[ch].prog_no = (UINT8)( program_no & MASMW_MASK_PROGRAM );

	if ( bank_no >= 128 )
	{
		ma_channel_info[ch].poly = MA_MODE_POLY;
	}

	if ( delta_time > 0 )
		result = MaSndDrv_Nop( seq_id, delta_time, 0, 0, 0 );
	else
		result = MASMW_SUCCESS;

	return result;
}
/****************************************************************************
 *	MaSndDrv_ModulationDepth
 *
 *	Description:
 *			Sets the depth of vibrato of specified channel.
 *	Arguments:
 *			seq_id		sequence id (0..2)
 *			delta_time	delta time [tick]
 *			ch			channel number (0..15)
 *			depth		depth of vibrato (0: OFF, 1: x1, 2: x2, 3: x4, 4: x8)
 *			p3			unused
 *	Return:
 *			Number of byte data of created packet
 *
 ****************************************************************************/
static SINT32 MaSndDrv_ModulationDepth
(
	SINT32	seq_id,						/* sequence id */
	SINT32	delta_time,					/* delta time */
	UINT32	ch,							/* channel number */
	UINT32	depth,						/* modulation depth */
	UINT32	p3
)
{
	UINT8	sfx;						/* sfx value */
	UINT32	num;						/* number of packet data */
	UINT32	reg_index;					/* register index number */
	SINT32	result;						/* result of function */
	static const UINT8 xvb[8] = { 0, 1, 2, 4, 6, 0, 0, 0 };
	
	(void)p3;							/* for unused warning message */
	
	MASNDDRV_DBGMSG((" MaSndDrv_ModulationDepth: id=%ld dt=%ld ch=%ld dp=%ld\n", seq_id, delta_time, ch, depth ));

	/* check arguments */

	MASMW_ASSERT( ch <= MASMW_MAX_CHANNEL );
	MASMW_ASSERT( depth <= MASMW_MAX_DEPTH );

	/* update channel information */

	depth &= MASMW_MASK_DEPTH;

	sfx = (UINT8)( (ma_channel_info[ch].sfx & MASMW_MASK_XVB) | xvb[depth] );
	ma_channel_info[ch].sfx = sfx;

	/* Make packet data and send it */

	num = MakeDeltaTime( &packet_buf[seq_id][0], delta_time );

	reg_index = (UINT32)(MA_CHANNEL_VIBRATO + ch);

	MAKE_ADDRESS_PART( seq_id, num, reg_index )
	MAKE_SFX( seq_id, num, sfx )
	
	if ( delta_time < 0 )				/* direct command */
	{
		result = MaDevDrv_SendDirectPacket( &packet_buf[seq_id][0], num );
	}
	else								/* delayed command */
	{
		result = SendDelayedPacket( &packet_buf[seq_id][0], num );
	}

	return result;
}
/****************************************************************************
 *	MaSndDrv_ChannelVolume
 *
 *	Description:
 *			Sets the volume of the specified channel.
 *	Arguments:
 *			seq_id		sequence id (0..2)
 *			delta_time	delta time [tick]
 *			ch			channel number (0..15)
 *			vol			volume value (0..127) default=100
 *			p3			unused
 *	Return:
 *			number of byte data of created packet
 *
 ****************************************************************************/
static SINT32 MaSndDrv_ChannelVolume
(
	SINT32	seq_id,						/* sequence id */
	SINT32	delta_time,					/* delta time */
	UINT32	ch,							/* channel number */
	UINT32	vol,						/* volume value */
	UINT32	p3							/* unused */
)
{
	UINT8	volume;						/* volume */
	UINT32	num;						/* number of packet data */
	UINT32	reg_index;					/* register index number */
	SINT32	result;						/* result of function */
	
	(void)p3;							/* for unused warning message */
	
	MASNDDRV_DBGMSG((" MaSndDrv_ChannelVolume: id=%ld dt=%ld ch=%ld vol=%ld\n", seq_id, delta_time, ch, vol ));

	/* check arguments */

	MASMW_ASSERT( ch <= MASMW_MAX_CHANNEL );
	MASMW_ASSERT( vol <= MASMW_MAX_VOLUME );

	/* updata channel information */

	ma_channel_info[ch].volume = (UINT8)(vol & MASMW_MASK_VOLUME);
	volume = CalcChVolume( ch );

	/* Make packet data and send it */

	num = MakeDeltaTime( &packet_buf[seq_id][0], delta_time );

	reg_index = (UINT32)(MA_CHANNEL_VOLUME + ch);

	MAKE_ADDRESS_PART( seq_id, num, reg_index )
	MAKE_CHANNEL_VOLUME( seq_id, num, volume )
	
	if ( delta_time < 0 )				/* direct command */
	{
		result = MaDevDrv_SendDirectPacket( &packet_buf[seq_id][0], num );
	}
	else								/* delayed command */
	{
		result = SendDelayedPacket( &packet_buf[seq_id][0], num );
	}

	return result;
}
/****************************************************************************
 *	MaSndDrv_Panpot
 *
 *	Description:
 *			Sets panpot of the specified channel.
 *	Arguments:
 *			seq_id		sequence id number (0..2)
 *			delta_time	delta time [tick]
 *			ch			channel number (0..15)
 *			pan			panpot value (0..127) default=64
 *			p3
 *	Return:
 *			number of byte data of created packet
 *
 ****************************************************************************/
static SINT32 MaSndDrv_Panpot
(
	SINT32	seq_id,						/* sequence id */
	SINT32	delta_time,					/* delta time */
	UINT32	ch,							/* channel number */
	UINT32	pan,						/* pan */
	UINT32	p3							/* unused */
)
{
	UINT32	num;						/* number of packet data */
	UINT32	reg_index;					/* register index number */
	SINT32	result;						/* result of function */
	
	(void)p3;							/* for unused warning message */
	
	MASNDDRV_DBGMSG((" MaSndDrv_Panpot: id=%ld dt=%ld ch=%ld pan=%ld\n", seq_id, delta_time, ch, pan ));

	/* check arguments */

	MASMW_ASSERT( ch <= MASMW_MAX_CHANNEL );
	MASMW_ASSERT( pan <= MASMW_MAX_PAN );

	/* Make packet data and send it */

	num = MakeDeltaTime( &packet_buf[seq_id][0], delta_time );

	reg_index = (UINT32)(MA_CHANNEL_PANPOT + ch);

	MAKE_ADDRESS_PART( seq_id, num, reg_index )
	MAKE_PANPOT( seq_id, num, pan )
	
	if ( delta_time < 0 )				/* direct command */
	{
		result = MaDevDrv_SendDirectPacket( &packet_buf[seq_id][0], num );
	}
	else								/* delayed command */
	{
		result = SendDelayedPacket( &packet_buf[seq_id][0], num );
	}

	return result;
}
/****************************************************************************
 *	MaSndDrv_Expression
 *
 *	Description:
 *			Sets the expression of specified channel.
 *	Arguments:
 *			seq_id		sequence id (0..2)
 *			delta_time	delta time [tick]
 *			ch			channel number (0..15)
 *			val			expression value (0..127) default=127
 *			p3			unused
 *	Return:
 *			number of byte data of created packet
 *
 ****************************************************************************/
static SINT32 MaSndDrv_Expression
(
	SINT32	seq_id,						/* sequence id */
	SINT32	delta_time,					/* delta time */
	UINT32	ch,							/* channel number */
	UINT32	vol,						/* volume value */
	UINT32	p3							/* unused */
)
{
	UINT8	volume;						/* volume value */
	UINT32	num;						/* number of packet data */
	UINT32	reg_index;					/* register index number */
	SINT32	result;						/* result of function */
	
	(void)p3;							/* for unused warning message */
	
	MASNDDRV_DBGMSG((" MaSndDrv_Expression: id=%ld dt=%ld ch=%ld vol=%ld\n", seq_id, delta_time, ch, vol ));

	/* check arguments */
	
	MASMW_ASSERT( ch <= MASMW_MAX_CHANNEL );
	MASMW_ASSERT( vol <= MASMW_MAX_VOLUME );

	/* update channel information */

	ma_channel_info[ch].expression = (UINT8)(vol & MASMW_MASK_VOLUME);
	volume = CalcChVolume( ch );

	/* Make packet data and send it */

	num = MakeDeltaTime( &packet_buf[seq_id][0], delta_time );

	reg_index = (UINT32)(MA_CHANNEL_VOLUME + ch);

	MAKE_ADDRESS_PART( seq_id, num, reg_index )
	MAKE_CHANNEL_VOLUME( seq_id, num, volume )

	if ( delta_time < 0 )				/* direct command */
	{
		result = MaDevDrv_SendDirectPacket( &packet_buf[seq_id][0], num );
	}
	else								/* delayed command */
	{
		result = SendDelayedPacket( &packet_buf[seq_id][0], num );
	}

	return result;
}
/****************************************************************************
 *	MaSndDrv_Hold1
 *
 *	Description:
 *			Sets the dumpper setting of specified channel.
 *	Arguments:
 *			seq_id		sequence id (0..2)
 *			delta_time	delta time [tick]
 *			ch			channel number (0..15)
 *			val			dumper value (0:OFF, 1:ON) default=0(OFF)
 *			p3			unused
 *	Return:
 *			Number of byte data of created packet
 *
 ****************************************************************************/
static SINT32 MaSndDrv_Hold1
(
	SINT32	seq_id,						/* sequence id */
	SINT32	delta_time,					/* delta time */
	UINT32	ch,							/* channle number */
	UINT32	val,						/* dumper setting value */
	UINT32	p3							/* unused */
)
{
	UINT8	sfx;						/* sfx value */
	UINT32	num;						/* number of packet data */
	UINT32	reg_index;					/* register index number */
	SINT32	result;						/* result of function */
	
	(void)p3;							/* for unused warning message */
	
	MASNDDRV_DBGMSG((" MaSndDrv_Hold1: id=%ld dt=%ld ch=%ld val=%ld\n", seq_id, delta_time, ch, val ));

	/* check arguments */

	MASMW_ASSERT( ch <= MASMW_MAX_CHANNEL );
	MASMW_ASSERT( val <= MASMW_MAX_HOLD1 );

	/* updata channel information */
	
	sfx = (UINT8)( (ma_channel_info[ch].sfx & MASMW_MASK_SUS) | ((val & 0x01) << 4) );
	ma_channel_info[ch].sfx = sfx;

	/* Make packet data and send it */

	num = MakeDeltaTime( &packet_buf[seq_id][0], delta_time );

	reg_index = (UINT32)(MA_CHANNEL_SUSTAIN + ch);

	MAKE_ADDRESS_PART( seq_id, num, reg_index )
	MAKE_SFX( seq_id, num, sfx )
	
	if ( delta_time < 0 )				/* direct command */
	{
		result = MaDevDrv_SendDirectPacket( &packet_buf[seq_id][0], num );
	}
	else								/* delayed command */
	{
		result = SendDelayedPacket( &packet_buf[seq_id][0], num );
	}

	return result;
}
/****************************************************************************
 *	MaSndDrv_AllSoundOff
 *
 *	Description:
 *			Turn off the all sound of specified channel.
 *	Arguments:
 *			seq_id		sequence id (0..2)
 *			delta_time	delta time [tick]
 *			ch			channel number (0..15)
 *			p2			unused
 *			p3			unused
 *	Return:
 *			Number of byte data of created packet
 *
 ****************************************************************************/
static SINT32 MaSndDrv_AllSoundOff
(
	SINT32	seq_id,						/* sequence id */
	SINT32	delta_time,					/* delta time */
	UINT32	ch,							/* channel number */
	UINT32	p2,							/* unused */
	UINT32	p3							/* unused */
)
{
	UINT32	slot_no;					/* slot number */
	UINT32	min_slot;					/* minimum slot number */
	UINT32	max_slot;					/* maximum slot number */
	UINT32	synth_type;					/* type of synth */
	UINT32	first = 0;					/* first flag */
	UINT32	tnum = 0;					/* total number of packet data */
	UINT32	num = 0;					/* number of packet data */
	UINT32	reg_index;					/* register index number */
	UINT32	res_map;					/* bit mapping of resource */
	SINT32	result = MASMW_SUCCESS;		/* result of packet sending */
	SINT32	dt;
	
	(void)p2;							/* for unused warning message */
	(void)p3;
	
	MASNDDRV_DBGMSG((" MaSndDrv_AllSoundOff: id=%ld dt=%ld ch=%ld\n", seq_id, delta_time, ch ));

	MASMW_ASSERT( ch <= MASMW_MAX_CHANNEL );
	MASMW_ASSERT( ma_snddrv_info[seq_id].dva_mode < 3 );

	ma_channel_info[ch].note_on = 0;

	/* FM & WT */

	tnum = 0;
	
	for ( synth_type = 0; synth_type < 2; synth_type++ )
	{
		min_slot = ma_snddrv_info[seq_id].min_slot[synth_type];
		max_slot = ma_snddrv_info[seq_id].max_slot[synth_type];

		for ( slot_no = min_slot; slot_no <= max_slot; slot_no++ )
		{
			if ( (ma_slot_info[slot_no].ch & 0x7F) == (UINT8)(ch + 1) )
			{
				if ( ( ma_slot_info[slot_no].ch & 0x80 ) != 0 )
				{
					RemakeSlotList( seq_id, synth_type, slot_no, 0x80 );
					ma_slot_info[slot_no].ch = (UINT8)(ch + 1);
				}

				if ( slot_no < 16 )
				{
					reg_index = (UINT32)(((slot_no -  0) * 6) + MA_FM_VOICE_ADDRESS);
				}
				else if ( slot_no < 32 )
				{
					reg_index = (UINT32)(((slot_no - 16) * 6) + MA_EXT_FM_VOICE_ADDRESS);
				}
				else if ( slot_no < 40 )
				{
					reg_index = (UINT32)(((slot_no - 32) * 6) + MA_WT_VOICE_ADDRESS);
				}
				else
				{
					continue;
				}

				if ( first == 0 )
				{
					num = MakeDeltaTime( &packet_buf[seq_id][0], delta_time );

					if ( delta_time <= 0 )
					{
						/* NOP2 */
						MAKE_NOP( seq_id, num, (UINT16)MA_NOP_2 )
						MAKE_TIMER_PART( seq_id, num, delta_time )
					}

					first = 1;
				}
				else
				{
					num = 0;
					MAKE_TIMER_PART( seq_id, num, delta_time )
				}

				/* KeyOff & MUTE & RST */
				reg_index += 5;
				MAKE_ADDRESS_PART( seq_id, num, reg_index )
				MAKE_RST_MUTE( seq_id, num, ch )

				if ( delta_time < 0 )			/* direct command */
				{
					result = MaDevDrv_SendDirectPacket( &packet_buf[seq_id][0], num );
				}
				else							/* delayed command */
				{
					result = SendDelayedPacket( &packet_buf[seq_id][0], num );
				}

				tnum += num;
			}
		}
	}

	if ( tnum == 0 )
	{
		if ( delta_time > 0 )
			result = MaSndDrv_Nop( seq_id, delta_time, 0, 0, 0 );
		else
			result = MASMW_SUCCESS;
	}

	res_map = ma_snddrv_info[seq_id].sa_map;
	for ( slot_no = 40; slot_no < 42; slot_no++ )
	{
		if ( ( ( res_map & (UINT32)(1 << (slot_no-40)) ) != 0 )
		  && ( ma_slot_info[slot_no].ch == (UINT8)( (ch+1) | 0x80) ) )
		{
			dt = (SINT32)( ( delta_time >= 0 ) ? 0 : -1 );
			MaSndDrv_StreamOff( seq_id, dt, ch, ma_slot_info[slot_no].key, 0 );
		}
	}

	return result;
}
/****************************************************************************
 *	MaSndDrv_ResetAllControllers
 *
 *	Description:
 *			Resets the control values of specified channel.
 *	Arguments:
 *			seq_id		sequence id (0..2)
 *			delta_time	delta time [tick]
 *			ch			channel number (0..15)
 *			p2			unused
 *			p3			unused
 *	Return:
 *			Number of byte data of created packet
 *
 ****************************************************************************/
static SINT32 MaSndDrv_ResetAllControllers
(
	SINT32	seq_id,						/* sequence id */
	SINT32	delta_time,					/* delta time */
	UINT32	ch,							/* channel number */
	UINT32	p2,							/* unused */
	UINT32	p3							/* unused */
)
{
	UINT8	volume;						/* volume value */
	UINT8	bend_range;					/* bend range */
	UINT16	bend;						/* bend value */
	UINT32	num;						/* number of packet data */
	UINT32	reg_index;					/* register index number */
	SINT32	result;						/* result of function */
	
	(void)p2;							/* for unused warning message */
	(void)p3;
	
	MASNDDRV_DBGMSG((" MaSndDrv_ResetAllControllers: id=%ld dt=%ld ch=%ld\n", seq_id, delta_time, ch ));

	/* check arguments */
	
	MASMW_ASSERT( ch <= MASMW_MAX_CHANNEL );

	/* updata channel information */

	ma_channel_info[ch].sfx = 0x00;
	ma_channel_info[ch].expression = 127;

	/* Make packet data */

	num = MakeDeltaTime( &packet_buf[seq_id][0], delta_time );

	/* channel volume */
	volume = CalcChVolume( ch );
	reg_index = (UINT32)(MA_CHANNEL_VOLUME + ch);
	MAKE_ADDRESS_PART( seq_id, num, reg_index )
	MAKE_CHANNEL_VOLUME( seq_id, num, volume )

	/* sfx */
	reg_index = (UINT32)(MA_CHANNEL_SUSTAIN + ch);
	MAKE_TIMER_PART( seq_id, num, delta_time )
	MAKE_ADDRESS_PART( seq_id, num, reg_index )
	MAKE_SFX( seq_id, num, ma_channel_info[ch].sfx )

	/* pitch bend */
	bend_range = ma_channel_info[ch].bend_range;
	bend = ma_pitchbend[bend_range][64];
	reg_index = (UINT32)( MA_CHANNEL_BEND + ch*2 );
	MAKE_TIMER_PART( seq_id, num, delta_time )
	MAKE_ADDRESS_PART( seq_id, num, reg_index )
	MAKE_PITCH_BEND( seq_id, num, bend )

	if ( delta_time < 0 )				/* direct command */
	{
		result = MaDevDrv_SendDirectPacket( &packet_buf[seq_id][0], num );
	}
	else								/* delayed command */
	{
		result = SendDelayedPacket( &packet_buf[seq_id][0], num );
	}

	return result;
}
/****************************************************************************
 *	MaSndDrv_AllNoteOff
 *
 *	Description:
 *			Off the all notes of specified channel.
 *	Arguments:
 *			seq_id		sequence id (0..2)
 *			delta_time	delta time [tick]
 *			ch			channel number (0..15)
 *			p2			unused
 *			p3			unused
 *	Return:
 *			Number of byte data of created packet
 *
 ****************************************************************************/
static SINT32 MaSndDrv_AllNoteOff
(
	SINT32	seq_id,						/* sequence id */
	SINT32	delta_time,					/* delta time */
	UINT32	ch,							/* channel number */
	UINT32	p2,							/* unused */
	UINT32	p3							/* unused */
)
{
	UINT32	slot_no;					/* slot number */
	UINT32	min_slot;					/* minimum slot number */
	UINT32	max_slot;					/* maximum slot number */
	UINT32	synth_type;					/* type of synth */
	UINT32	first = 0;					/* first flag */
	UINT32	tnum = 0;					/* total number of packet data */
	UINT32	num = 0;					/* number of packet data */
	UINT32	reg_index;					/* register index number */
	UINT32	res_map;					/* bit mapping of resource */
	SINT32	result = MASMW_SUCCESS;		/* result of packet sending */
	SINT32	dt;
	
	(void)p2;							/* for unused warning message */
	(void)p3;
	
	MASNDDRV_DBGMSG((" MaSndDrv_AllNoteOff: id=%ld dt=%ld ch=%ld\n", seq_id, delta_time, ch ));

	MASMW_ASSERT( ch <= MASMW_MAX_CHANNEL );
	MASMW_ASSERT( ma_snddrv_info[seq_id].dva_mode < 3 );

	ma_channel_info[ch].note_on = 0;

	/* FM & WT */

	tnum = 0;
	
	for ( synth_type = 0; synth_type < 2; synth_type++ )
	{
		min_slot = ma_snddrv_info[seq_id].min_slot[synth_type];
		max_slot = ma_snddrv_info[seq_id].max_slot[synth_type];

		for ( slot_no = min_slot; slot_no <= max_slot; slot_no++ )
		{
			if ( ma_slot_info[slot_no].ch == (UINT8)( (ch+1) | 0x80 ) )
			{
				RemakeSlotList( seq_id, synth_type, slot_no, 0x80 );
				ma_slot_info[slot_no].ch = (UINT8)(ch + 1);

				if ( slot_no < 16 )
				{
					reg_index = (UINT32)(((slot_no -  0) * 6) + MA_FM_VOICE_ADDRESS);
				}
				else if ( slot_no < 32 )
				{
					reg_index = (UINT32)(((slot_no - 16) * 6) + MA_EXT_FM_VOICE_ADDRESS);
				}
				else if ( slot_no < 40 )
				{
					reg_index = (UINT32)(((slot_no - 32) * 6) + MA_WT_VOICE_ADDRESS);
				}
				else
				{
					continue;
				}

				if ( first == 0 )
				{
					num = MakeDeltaTime( &packet_buf[seq_id][0], delta_time );

					if ( delta_time <= 0 )
					{
						/* NOP2 */
						MAKE_NOP( seq_id, num, (UINT16)MA_NOP_2 )
						MAKE_TIMER_PART( seq_id, num, delta_time )
					}

					first = 1;
				}
				else
				{
					num = 0;
					MAKE_TIMER_PART( seq_id, num, delta_time )
				}

				/* KeyOff */
				reg_index += 5;
				MAKE_ADDRESS_PART( seq_id, num, reg_index )
				MAKE_KEY_OFF( seq_id, num, ch )

				if ( delta_time < 0 )			/* direct command */
				{
					result = MaDevDrv_SendDirectPacket( &packet_buf[seq_id][0], num );
				}
				else							/* delayed command */
				{
					result = SendDelayedPacket( &packet_buf[seq_id][0], num );
				}

				tnum += num;
			}
		}
	}

	if ( tnum == 0 )
	{
		if ( delta_time > 0 )
			result = MaSndDrv_Nop( seq_id, delta_time, 0, 0, 0 );
		else
			result = MASMW_SUCCESS;
	}

	res_map = ma_snddrv_info[seq_id].sa_map;
	for ( slot_no = 40; slot_no < 42; slot_no++ )
	{
		if ( ( ( res_map & (UINT32)(1 << (slot_no-40)) ) != 0 )
		  && ( ma_slot_info[slot_no].ch == (UINT8)( (ch+1) | 0x80) ) )
		{
			dt = (SINT32)( ( delta_time >= 0 ) ? 0 : -1 );
			MaSndDrv_StreamOff( seq_id, dt, ch, ma_slot_info[slot_no].key, 0 );
		}
	}

	return result;
}
/****************************************************************************
 *	MaSndDrv_MonoModeOn
 *
 *	Description:
 *			Sets the monophonic mode to specified channel.
 *	Arguments:
 *			seq_id		sequence id (0..2)
 *			delta_time	delta time [tick]
 *			ch			channel number (0..15)
 *			p2			unused
 *			p3			unused
 *	Return:
 *			Number of byte data of created packet
 *
 ****************************************************************************/
static SINT32 MaSndDrv_MonoModeOn
(
	SINT32	seq_id,						/* sequence id */
	SINT32	delta_time,					/* delta time */
	UINT32	ch,							/* channel number */
	UINT32	p2,							/* unused */
	UINT32	p3							/* unused */
)
{
	SINT32	result;						/* result of function */

	(void)seq_id;						/*for unused warning message */
	(void)delta_time;
	(void)p2;
	(void)p3;
	
	MASNDDRV_DBGMSG((" MaSndDrv_MonoModeOn: id=%ld dt=%ld ch=%ld\n", seq_id, delta_time, ch ));

	/* check argument */

	MASMW_ASSERT( ch <= MASMW_MAX_CHANNEL );

	/* updata channel information */

	if ( ma_channel_info[ch].bank_no < 128 )
	{
		ma_channel_info[ch].poly = MA_MODE_MONO;
	}

	/* Make packet data and send it */

	if ( delta_time > 0 )
		result = MaSndDrv_Nop( seq_id, delta_time, 0, 0, 0 );
	else
		result = MASMW_SUCCESS;

	return result;
}
/****************************************************************************
 *	MaSndDrv_PolyModeOn
 *
 *	Description:
 *			Sets the polyphonic mode to specified channel.
 *	Arguments:
 *			seq_id		sequence id (0..2)
 *			delta_time	delta time [tick]
 *			ch			channel number (0..15)
 *			p2			unused
 *			p3			unused
 *	Return:
 *			Number of byte data of created packet
 *
 ****************************************************************************/
static SINT32 MaSndDrv_PolyModeOn
(
	SINT32	seq_id,						/* sequence id */
	SINT32	delta_time,					/* delta time */
	UINT32	ch,							/* channel number */
	UINT32	p2,							/* unused */
	UINT32	p3							/* unused */
)
{
	SINT32	result;						/* result of function */
	
	(void)seq_id;						/*for unused warning message */
	(void)delta_time;
	(void)p2;
	(void)p3;
	
	MASNDDRV_DBGMSG((" MaSndDrv_PolyModeOn: id=%ld dt=%ld ch=%ld\n", seq_id, delta_time, ch ));

	/* check argument */

	MASMW_ASSERT( ch <= MASMW_MAX_CHANNEL );

	/* updata channel information */

	ma_channel_info[ch].poly = MA_MODE_POLY;

	/* Make packet data and send it */

	if ( delta_time > 0 )
		result = MaSndDrv_Nop( seq_id, delta_time, 0, 0, 0 );
	else
		result = MASMW_SUCCESS;

	return result;
}
/****************************************************************************
 *	MaSndDrv_PitchBend
 *
 *	Description:
 *			Sets the pitch bend of the specified channel.
 *	Arguments:
 *			seq_id		sequence id (0..2)
 *			delta_time	delta time [tick]
 *			ch			channel nubmer (0..15)
 *			bend		pitch bend value (0..2000h..3FFFh)
 *			p3			unused
 *	Return:
 *			Number of byte data of created packet
 *
 ****************************************************************************/
static SINT32 MaSndDrv_PitchBend
(
	SINT32	seq_id,						/* sequence id */
	SINT32	delta_time,					/* delta time */
	UINT32	ch,							/* channel number */
	UINT32	bend,						/* bend */
	UINT32	p3							/* unused */
)
{
	UINT8	bend_range;					/* bend range */
	UINT16	pitch_bend;					/* pitch bend */
	UINT32	reg_index;					/* register index number */
	SINT32	num;						/* number of packet data */
	SINT32	result;						/* result of function */

	(void)p3;							/* for unused warning message */

	MASNDDRV_DBGMSG((" MaSndDrv_PitchBend: id=%ld dt=%ld ch=%ld bend=%ld\n", seq_id, delta_time, ch, bend ));

	/* check arguments */

	MASMW_ASSERT( ch <= MASMW_MAX_CHANNEL );
	MASMW_ASSERT( bend <= MASMW_MAX_BEND );

	/* setting the bend value */

	bend &= MASMW_MASK_BEND;

	bend_range = ma_channel_info[ch].bend_range;
	pitch_bend = ma_pitchbend[bend_range][bend >> 7];

	/* Make packet data and send it */

	num = MakeDeltaTime( &packet_buf[seq_id][0], delta_time );
	
	reg_index = (UINT32)(MA_CHANNEL_BEND + ch * 2);

	MAKE_ADDRESS_PART( seq_id, num, reg_index )
	MAKE_PITCH_BEND( seq_id, num, pitch_bend )
	
	if ( delta_time < 0 )				/* direct command */
	{
		result = MaDevDrv_SendDirectPacket( &packet_buf[seq_id][0], num );
	}
	else								/* delayed command */
	{
		result = SendDelayedPacket( &packet_buf[seq_id][0], num );
	}

	return result;
}
/****************************************************************************
 *	MaSndDrv_BendRange
 *
 *	Description:
 *			
 *	Arguments:
 *			seq_id		sequence id (0..2)
 *			delta_time	delta time [tick]
 *			ch			channel number (0..15)
 *			bend_range	bend range (0..24)
 *			p3			unused
 *	Return:
 *			Number of byte data of created packet
 *
 ****************************************************************************/
static SINT32 MaSndDrv_BendRange
(
	SINT32	seq_id,						/* sequence id */
	SINT32	delta_time,					/* delta time */
	UINT32	ch,							/* channel number */
	UINT32	bend_range,					/* bend range */
	UINT32	p3							/* unused */
)
{
	SINT32	result;						/* result of function */
	
	(void)p3;							/* for unused warning message */

	MASNDDRV_DBGMSG((" MaSndDrv_BendRange: id=%ld dt=%ld ch=%ld br=%ld\n", seq_id, delta_time, ch, bend_range ));

	/* check argument */

	MASMW_ASSERT( ch <= MASMW_MAX_CHANNEL );

	if ( bend_range > MASMW_MAX_BENDRANGE )	return MASMW_ERROR_ARGUMENT;

	/* updata channel information */

	ma_channel_info[ch].bend_range = (UINT8)( bend_range );

	/* Make packet data and send it */

	if ( delta_time > 0 )
		result = MaSndDrv_Nop( seq_id, delta_time, 0, 0, 0 );
	else
		result = MASMW_SUCCESS;

	return result;
}
/****************************************************************************
 *	MaSndDrv_StreamPanpot
 *
 *	Description:
 *			Set the panpot value for stream audio data.
 * Arguments:
 *			seq_id		sequence id
 *			delta_time	delta time [tick]
 *			wav_id		wave id number(0..31)
 *			pan			panpot setting(0..127, 128: center 0dB, 255: use ChPan)
 *			p3
 *	Return:
 *			0			success
 *			< 0 		error code
 *
 ****************************************************************************/
static SINT32 MaSndDrv_StreamPanpot
(
	SINT32	seq_id,						/* sequence id */
	SINT32	delta_time,					/* delta time */
	UINT32	wav_id,						/* wave id number */
	UINT32	pan,						/* pan setting */
	UINT32	p3
)
{
	SINT32	result;						/* result of function */

	(void)p3;

	MASNDDRV_DBGMSG((" MaSndDrv_StreamPanpot: id=%ld dt=%ld id=%ld pan=%ld\n", seq_id, delta_time, wav_id, pan ));

	if ( wav_id > MASMW_MAX_WAVEID )	return MASMW_ERROR_ARGUMENT;
	if ( pan > MASMW_MAX_STREAMPAN )	return MASMW_ERROR_ARGUMENT;

	if ( ma_snddrv_info[seq_id].sa_num == 0 )
	{
		if ( delta_time > 0 )
			result = MaSndDrv_Nop( seq_id, delta_time, 0, 0, 0 );

		return MASMW_SUCCESS;
	}

	ma_stream_info[seq_id][wav_id].pan = (UINT8)pan;

	if ( delta_time > 0 )
		result = MaSndDrv_Nop( seq_id, delta_time, 0, 0, 0 );
	else
		result = MASMW_SUCCESS;

	return result;
}
/****************************************************************************
 *	MaSndDrv_StreamOn
 *
 *	Description:
 *			Play the stream audio.
 *	Arguments:
 *			seq_id		sequence id (0..2)
 *			delta_time	delta time [tick]
 *			ch			channel number (0..15)
 *			wave_id		wave id number (0..31)
 *			velocity	velocity value (0..127)
 *	Return:
 *			0			success
 *			< 0			error code
 *
 ****************************************************************************/
static SINT32 MaSndDrv_StreamOn
(
	SINT32	seq_id,						/* sequence id */
	SINT32	delta_time,					/* delta time */
	UINT32	ch,							/* channel number */
	UINT32	wave_id,					/* wave id */
	UINT32	velocity					/* velocity */
)
{
	UINT32	slave_id;					/* slave wave id */
	UINT8	slave_sa_id;				/* slave sa id */
	UINT32	num;						/* number of packet data */
	UINT32	reg_index;					/* register index number */
	UINT16	start_adrs;					/* start address */
	UINT32	ram_adrs;					/* ram address */
	SINT32	result;						/* result of function */
	MASTREAMPARAM	stream_param;		/* parameter of stream audio */
	
	MASNDDRV_DBGMSG((" MaSndDrv_StreamOn: id=%ld dt=%ld ch=%ld id=%ld vol=%ld\n", seq_id, delta_time, ch, wave_id, velocity ));

	/* check arguments */

	if ( ch > MASMW_MAX_CHANNEL )			return MASMW_ERROR_ARGUMENT;
	if ( wave_id > MASMW_MAX_WAVEID )		return MASMW_ERROR_ARGUMENT;
	if ( velocity > MASMW_MAX_VELOCITY )	return MASMW_ERROR_ARGUMENT;

	if ( ma_snddrv_info[seq_id].sa_num == 0 )
	{
		result = MaSndDrv_Nop( seq_id, delta_time, 0, 0, 0 );
		return result;
	}

	/* Make WT parameters */

	result = MakeStreamAudioParam( seq_id, ch, wave_id, velocity, &stream_param );
	if ( result < MASMW_SUCCESS )
	{
		result = MaSndDrv_Nop( seq_id, delta_time, 0, 0, 0 );
		return result;
	}


	num = MakeDeltaTime( &packet_buf[seq_id][0], delta_time );
	
	ram_adrs = (UINT16)(MA_RAM_START_ADDRESS + (7-stream_param.sa_id) * MA_RAM_BLOCK_SIZE);
	start_adrs = (UINT16)( ram_adrs + 0x20 );

	packet_buf[seq_id][num++] = (UINT8)(  ram_adrs        & 0x7F );
	packet_buf[seq_id][num++] = (UINT8)( (ram_adrs >>  7) & 0x7F );
	packet_buf[seq_id][num++] = (UINT8)( (ram_adrs >> 14) | 0x80 );

	packet_buf[seq_id][num++] = (UINT8)( 13 | 0x80 );
	packet_buf[seq_id][num++] = (UINT8)( stream_param.panpot | stream_param.stm | stream_param.pe );
	packet_buf[seq_id][num++] = (UINT8)( stream_param.panoff | stream_param.mode );
	packet_buf[seq_id][num++] = (UINT8)0x00;
	packet_buf[seq_id][num++] = (UINT8)0xF0;
	packet_buf[seq_id][num++] = (UINT8)0xF0;
	packet_buf[seq_id][num++] = (UINT8)0x00;
	packet_buf[seq_id][num++] = (UINT8)0x00;
	packet_buf[seq_id][num++] = (UINT8)( start_adrs >> 8 );
	packet_buf[seq_id][num++] = (UINT8)( start_adrs      );
	packet_buf[seq_id][num++] = (UINT8)0x00;
	packet_buf[seq_id][num++] = (UINT8)0x00;
	packet_buf[seq_id][num++] = (UINT8)( stream_param.end_point >> 8 );
	packet_buf[seq_id][num++] = (UINT8)( stream_param.end_point );


	ram_adrs >>= 1;

	reg_index = (UINT32)(MA_WT_VOICE_ADDRESS + (7-stream_param.sa_id) * 6);
	MAKE_TIMER_PART( seq_id, num, delta_time )
	MAKE_ADDRESS_PART( seq_id, num, reg_index )
	MAKE_STREAM_ON( seq_id, num, ram_adrs, stream_param.vo_volume, stream_param.pitch, ch )


	if ( ma_snddrv_info[seq_id].wave_slave != 0x80 )
	{
		slave_id = ma_snddrv_info[seq_id].wave_slave;
		ma_snddrv_info[seq_id].wave_slave = 0x80;

		slave_sa_id = (UINT8)( ( stream_param.sa_id == 0 ) ? 1 : 0 );

		reg_index = (UINT32)(MA_SOFTINT_RAM + slave_sa_id);
		MAKE_TIMER_PART( seq_id, num, delta_time )
		MAKE_ADDRESS_PART( seq_id, num, reg_index )
		MAKE_SOFTINT_RAM( seq_id, num, ( slave_id | 0x40) )

		wave_id |= 0x20;
	}

	reg_index = (UINT32)(MA_SOFTINT_RAM + stream_param.sa_id);
	MAKE_TIMER_PART( seq_id, num, delta_time )
	MAKE_ADDRESS_PART( seq_id, num, reg_index )
	MAKE_SOFTINT_RAM( seq_id, num, (wave_id | 0x40) )


	reg_index = (UINT32)MA_SOFTINT_CTRL;
	MAKE_TIMER_PART( seq_id, num, delta_time )
	MAKE_ADDRESS_PART( seq_id, num, reg_index )
	MAKE_SOFTINT( seq_id, num, (1 << stream_param.sa_id) )

	if ( delta_time < 0 )				/* direct command */
	{
		result = MaDevDrv_SendDirectPacket( &packet_buf[seq_id][0], num );
	}
	else								/* delayed command */
	{
		result = SendDelayedPacket( &packet_buf[seq_id][0], num );
	}


	ma_slot_info[40+stream_param.sa_id].ch   = (UINT8)( (ch + 1) | 0x80 );
	ma_slot_info[40+stream_param.sa_id].key  = (UINT8)(wave_id & 0x1F);

	return result;
}
/****************************************************************************
 *	MaSndDrv_StreamOff
 *
 *	Description:
 *			
 *	Arguments:
 *			seq_id		sequence id (0..2)
 *			delta_time	delta time [tick]
 *			ch			channel number (0..15)
 *			wave_id		wave id number (0..31)
 *			p3
 *	Return:
 *			
 *
 ****************************************************************************/
static SINT32 MaSndDrv_StreamOff
(
	SINT32	seq_id,						/* sequence id */
	SINT32	delta_time,					/* delta time */
	UINT32	ch,							/* channel number */
	UINT32	wave_id,					/* wave id */
	UINT32	p3							/* unused */
)
{
	UINT32	sa_id = 0;					/* stream audio id */
	UINT32	num;						/* number of packet data */
	UINT32	reg_index;					/* register index number */
	SINT32	result;						/* result of function */

	(void)p3;							/* for unused warning message */

	MASNDDRV_DBGMSG((" MaSndDrv_StreamOff: id=%ld dt=%ld ch=%ld id=%ld\n", seq_id, delta_time, ch, wave_id ));

	/* check arguments */

	if ( ch > MASMW_MAX_CHANNEL )		return MASMW_ERROR_ARGUMENT;
	if ( wave_id > MASMW_MAX_WAVEID )	return MASMW_ERROR_ARGUMENT;

	if ( ma_snddrv_info[seq_id].sa_num == 0 )
	{
		if ( delta_time > 0 )
			result = MaSndDrv_Nop( seq_id, delta_time, 0, 0, 0 );

		return MASMW_SUCCESS;
	}
	
	switch ( ma_snddrv_info[seq_id].dva_mode )
	{
	case 0:
	case 1:
		sa_id = SearchSlot( seq_id, 3, (UINT8)ch, (UINT8)wave_id );
		if ( sa_id == 0xFF )
		{
			/* send delta_time value if needed */
			if ( delta_time > 0 )
				result = MaSndDrv_Nop( seq_id, delta_time, 0, 0, 0 );
			else
				result = MASMW_SUCCESS;

			return result;
		}
		sa_id -= 40;
		break;
			
	case 2:
		sa_id = 0;
		break;
	}

	if ( delta_time < 0 )				/* direct command */
	{
		result = MaDevDrv_StreamHandler( (UINT8)sa_id, (UINT8)0, (UINT8)wave_id );
	}
	else								/* delayed command */
	{
		num = MakeDeltaTime( &packet_buf[seq_id][0], delta_time );

		/* send software interrupt command to stop wave data update */

		reg_index = (UINT32)(MA_SOFTINT_RAM + sa_id);
		MAKE_ADDRESS_PART( seq_id, num, reg_index )
		MAKE_SOFTINT_RAM( seq_id, num, wave_id )

		reg_index = (UINT32)MA_SOFTINT_CTRL;
		MAKE_TIMER_PART( seq_id, num, delta_time )
		MAKE_ADDRESS_PART( seq_id, num, reg_index )
		MAKE_SOFTINT( seq_id, num, (1 << sa_id) )

		result = SendDelayedPacket( &packet_buf[seq_id][0], num );
	}

	/* update slot information */

	ma_slot_info[40+sa_id].ch   = (UINT8)(ch + 1);
	ma_slot_info[40+sa_id].key	= wave_id;

	return result;
}
/****************************************************************************
 *	MaSndDrv_StreamSlave
 *
 *	Description:
 *			Set slave of the stream audio.
 *	Arguments:
 *			seq_id		sequence id (0..2)
 *			delta_time	delta time [tick]
 *			ch			channel number (0..15)
 *			wave_id		wave id number (0..31)
 *			velocity	velocity value (0..127)
 *	Return:
 *			0			success
 *			< 0			error code
 *
 ****************************************************************************/
static SINT32 MaSndDrv_StreamSlave
(
	SINT32	seq_id,						/* sequence id */
	SINT32	delta_time,					/* delta time */
	UINT32	ch,							/* channel number */
	UINT32	wave_id,					/* wave id */
	UINT32	velocity					/* velocity */
)
{
	UINT32	num;						/* number of packet data */
	UINT32	reg_index;					/* register index number */
	UINT16	start_adrs;					/* start address */
	UINT32	ram_adrs;					/* ram address */
	SINT32	result;						/* result of function */
	MASTREAMPARAM	stream_param;		/* stream audio parameter */

	MASNDDRV_DBGMSG((" MaSndDrv_StreamSlave: id=%ld dt=%ld ch=%ld id=%ld vol=%ld\n", seq_id, delta_time, ch, wave_id, velocity ));

	/* check arguments */

	if ( ch > MASMW_MAX_CHANNEL )			return MASMW_ERROR_ARGUMENT;
	if ( wave_id > MASMW_MAX_WAVEID )		return MASMW_ERROR_ARGUMENT;
	if ( velocity > MASMW_MAX_VELOCITY )	return MASMW_ERROR_ARGUMENT;

	if ( ma_snddrv_info[seq_id].sa_num < 2 )
	{
		if ( delta_time > 0 )
			result = MaSndDrv_Nop( seq_id, delta_time, 0, 0, 0 );

		return MASMW_SUCCESS;
	}

	/* Make WT parameters */

	result = MakeStreamAudioParam( seq_id, ch, wave_id, velocity, &stream_param );
	if ( result < MASMW_SUCCESS )
	{
		result = MaSndDrv_Nop( seq_id, delta_time, 0, 0, 0 );
		return result;
	}


	num = MakeDeltaTime( &packet_buf[seq_id][0], delta_time );
	
	ram_adrs = (UINT16)(MA_RAM_START_ADDRESS + (7-stream_param.sa_id) * MA_RAM_BLOCK_SIZE);
	start_adrs = (UINT16)( ram_adrs + 0x20 );

	packet_buf[seq_id][num++] = (UINT8)(  ram_adrs        & 0x7F );
	packet_buf[seq_id][num++] = (UINT8)( (ram_adrs >>  7) & 0x7F );
	packet_buf[seq_id][num++] = (UINT8)( (ram_adrs >> 14) | 0x80 );

	packet_buf[seq_id][num++] = (UINT8)( 13 | 0x80 );
	packet_buf[seq_id][num++] = (UINT8)( stream_param.panpot | stream_param.stm | stream_param.pe );
	packet_buf[seq_id][num++] = (UINT8)( stream_param.panoff | stream_param.mode );
	packet_buf[seq_id][num++] = (UINT8)0x00;
	packet_buf[seq_id][num++] = (UINT8)0xF0;
	packet_buf[seq_id][num++] = (UINT8)0xF0;
	packet_buf[seq_id][num++] = (UINT8)0x00;
	packet_buf[seq_id][num++] = (UINT8)0x00;
	packet_buf[seq_id][num++] = (UINT8)( start_adrs >> 8 );
	packet_buf[seq_id][num++] = (UINT8)( start_adrs      );
	packet_buf[seq_id][num++] = (UINT8)0x00;
	packet_buf[seq_id][num++] = (UINT8)0x00;
	packet_buf[seq_id][num++] = (UINT8)( stream_param.end_point >> 8 );
	packet_buf[seq_id][num++] = (UINT8)( stream_param.end_point );


	ram_adrs >>= 1;

	reg_index = (UINT32)(MA_WT_VOICE_ADDRESS + (7-(UINT32)stream_param.sa_id) * 6);
	MAKE_TIMER_PART( seq_id, num, delta_time )
	MAKE_ADDRESS_PART( seq_id, num, reg_index )
	MAKE_STREAM_ON( seq_id, num, ram_adrs, stream_param.vo_volume, stream_param.pitch, ch )

	if ( delta_time < 0 )				/* direct command */
	{
		result = MaDevDrv_SendDirectPacket( &packet_buf[seq_id][0], num );
	}
	else								/* delayed command */
	{
		result = SendDelayedPacket( &packet_buf[seq_id][0], num );
	}


	ma_slot_info[40+stream_param.sa_id].ch   = (UINT8)( (ch + 1) | 0x80 );
	ma_slot_info[40+stream_param.sa_id].key  = (UINT8)wave_id;

	ma_snddrv_info[seq_id].wave_slave = (UINT8)wave_id;

	return result;
}
/****************************************************************************
 *	MaSndDrv_MasterVolume
 *
 *	Description:
 *			Sets the master volume of the specified sequence.
 *	Argument:
 *			seq_id		sequence id (0..2)
 *			delta_time	delta time [tick]
 *			vol			volume value (0..127)
 *			p2			unused
 *			p3			unused
 *	Return:
 *			0			success
 *			< 0			error code
 *
 ****************************************************************************/
static SINT32 MaSndDrv_MasterVolume
(
	SINT32	seq_id,						/* sequence id */
	SINT32	delta_time,					/* delta time */
	UINT32	vol,						/* volume value */
	UINT32	p2,							/* unused */
	UINT32	p3							/* unused */
)
{
	UINT32	db;							/* db value */
	UINT8	velocity;					/* velocity */
	UINT32	num = 0;					/* number of packet data */
	UINT32	reg_index;					/* register index number */
	SINT32	result;						/* result of function */

	(void)p2;							/* for unused warning message */
	(void)p3;

	MASNDDRV_DBGMSG((" MaSndDrv_MasterVolume: id=%ld dt=%ld vol=%ld\n", seq_id, delta_time, vol ));

	/* check argument */
	if ( vol > MASMW_MAX_VOLUME )	return MASMW_ERROR_ARGUMENT;

	ma_snddrv_info[seq_id].master_volume = (UINT8)vol;

	switch ( seq_id )
	{
	case MASMW_SEQTYPE_DELAYED:

		result = MaSndDrv_Nop( seq_id, delta_time, 0, 0, 0 );
		return result;
	 /*	break; */

	case MASMW_SEQTYPE_DIRECT:

		return MASMW_SUCCESS;
	 /*	break; */

	case MASMW_SEQTYPE_AUDIO:

		db = db_table[vol];
		db = (UINT32)( db + db_table[ma_snddrv_info[seq_id].wave_velocity] );
		db = (UINT32)( db + db_table[ma_snddrv_info[seq_id].ctrl_volume] );
		if ( db > MASMW_MAX_DB ) db = (UINT8)MASMW_MAX_DB;

		velocity = vol_table[db];

		num = MakeDeltaTime( &packet_buf[seq_id][0], delta_time );
		
		reg_index = (UINT32)(MA_WT_VOICE_ADDRESS + (7 * 6) + 2);
		MAKE_ADDRESS_PART( seq_id, num, reg_index )
		MAKE_CHANNEL_VOLUME( seq_id, num, velocity );

		break;

	default:
		return MASMW_ERROR;
	 /* break; */
	}

	if ( delta_time < 0 )				/* direct command */
	{
		result = MaDevDrv_SendDirectPacket( &packet_buf[seq_id][0], num );
	}
	else								/* delayed command */
	{
		result = SendDelayedPacket( &packet_buf[seq_id][0], num );
	}

	return result;
}
/****************************************************************************
 *	MaSndDrv_SystemOn
 *
 *	Description:
 *			Initialize the specified sequence.
 *	Argument:
 *			seq_id		sequence id (0..2)
 *			delta_time	delta time [tick]
 *			p1			unused
 *			p2			unused
 *			p3			unused
 *	Return:
 *			0			success
 *			< 0			error code
 *
 ****************************************************************************/
static SINT32 MaSndDrv_SystemOn
(
	SINT32	seq_id,						/* sequence id */
	SINT32	delta_time,					/* delta time */
	UINT32	p1,							/* unused */
	UINT32	p2,							/* unused */
	UINT32	p3							/* unused */
)
{
	UINT32	ch;							/* channel number */
	UINT32	min_ch;						/* minimum channel number */
	UINT32	max_ch;						/* maximum channel number */
	UINT8	volume;						/* volume value */
	UINT32 	num;						/* number of packet */
	UINT8	wave_id;					/* wave id */
	UINT32	reg_index;					/* register index number */
	UINT32	res_map;					/* bit mapping of resource */
	SINT32	result;						/* result of function */

	(void)p1;							/* for unused warning message */
	(void)p2;
	(void)p3;
	
	MASNDDRV_DBGMSG((" MaSndDrv_SystemOn: id=%ld dt=%ld\n", seq_id, delta_time ));


	min_ch = MASMW_NUM_CHANNEL;
	max_ch = 0;
	res_map = ma_snddrv_info[seq_id].ch_map;
	for ( ch = 0; ch < MASMW_NUM_CHANNEL; ch++ )
	{
		if ( ( res_map & (UINT32)(1 << ch) ) != 0 )
		{
			if ( min_ch > ch ) min_ch = ch;
			if ( max_ch < ch ) max_ch = ch;
		}
	}
	if ( min_ch == MASMW_NUM_CHANNEL ) return MASMW_SUCCESS;

	for ( ch = min_ch; ch <= max_ch; ch++ )
	{
		if ( seq_id == MASMW_SEQTYPE_DIRECT )
			ma_channel_info[ch].bank_no = (UINT8)( ( ch == 6 ) ? 128 : 0 );
		else
			ma_channel_info[ch].bank_no = (UINT8)( ( ch == 9 ) ? 128 : 0 );

		ma_channel_info[ch].prog_no		= 0;
		ma_channel_info[ch].poly 		= MA_MODE_POLY;
		ma_channel_info[ch].sfx			= 0;
		ma_channel_info[ch].volume 		= 100;
		ma_channel_info[ch].expression	= 127;
		ma_channel_info[ch].bend_range 	= 2;
		ma_channel_info[ch].note_on		= 0;
	}

	for ( wave_id = 0; wave_id < MA_MAX_REG_STREAM; wave_id++ )
	{
		ma_stream_info[seq_id][wave_id].pan = 255;
	}


	num = MakeDeltaTime( &packet_buf[seq_id][0], delta_time );

	/* channel volume */
	
	reg_index = (UINT32)( MA_CHANNEL_VOLUME + min_ch );
	MAKE_ADDRESS_PART( seq_id, num, reg_index )

	for ( ch = min_ch; ch < max_ch; ch++ )
	{
		volume = CalcChVolume( ch );
		packet_buf[seq_id][num++] = (UINT8)( volume & 0x7C );
	}
	volume = CalcChVolume( ch );
	packet_buf[seq_id][num++] = (UINT8)( (volume & 0x7C) | 0x80 );
	
	/* panpot - 64(center) */

	reg_index = (UINT32)( MA_CHANNEL_PANPOT + min_ch );
	MAKE_TIMER_PART( seq_id, num, delta_time )
	MAKE_ADDRESS_PART( seq_id, num, reg_index )

	for ( ch = min_ch; ch < max_ch; ch++ )
	{
		packet_buf[seq_id][num++] = (UINT8)( 64 & 0x7C );
	}
	packet_buf[seq_id][num++] = (UINT8)( (64  & 0x7C) | 0x80);

	/* hold1            - 0(off) */
	/* modulation depth - 0(off) */

	reg_index = (UINT32)( MA_CHANNEL_SUSTAIN + min_ch );
	MAKE_TIMER_PART( seq_id, num, delta_time )
	MAKE_ADDRESS_PART( seq_id, num, reg_index )

	for ( ch = min_ch; ch < max_ch; ch++ )
	{
		packet_buf[seq_id][num++] = (UINT8)(0 & 0x7F);
	}
	packet_buf[seq_id][num++] = (UINT8)(0 | 0x80);

	/* pitch bend - (1.0) */

	reg_index = (UINT32)( MA_CHANNEL_BEND + min_ch*2 );
	MAKE_TIMER_PART( seq_id, num, delta_time )
	MAKE_ADDRESS_PART( seq_id, num, reg_index )

	for ( ch = min_ch; ch < max_ch; ch++ )
	{
		packet_buf[seq_id][num++] = (UINT8)0x08;
		packet_buf[seq_id][num++] = (UINT8)0x00;
	}
	packet_buf[seq_id][num++] = (UINT8)(0x08);
	packet_buf[seq_id][num++] = (UINT8)(0x00 | 0x80);

	if ( seq_id == MASMW_SEQTYPE_DELAYED )
	{
		/* LED */
		if ( ma_led_info.ctrl == 2 )
		{
			reg_index = (UINT32)MA_LED_CTRL;
			MAKE_TIMER_PART( seq_id, num, delta_time )
			MAKE_ADDRESS_PART( seq_id, num, reg_index )
			MAKE_LED( seq_id, num, 0 )
		}
		
		/* Motor */
		if ( ma_mtr_info.ctrl == 2 )
		{
			reg_index = (UINT32)MA_MOTOR_CTRL;
			MAKE_TIMER_PART( seq_id, num, delta_time )
			MAKE_ADDRESS_PART( seq_id, num, reg_index )
			MAKE_MOTOR( seq_id, num, 0 )
		}
	}

	if ( delta_time < 0 )				/* direct command */
	{
		result = MaDevDrv_SendDirectPacket( &packet_buf[seq_id][0], num );
	}
	else								/* delayed command */
	{
		result = SendDelayedPacket( &packet_buf[seq_id][0], num );
	}

	return result;
}
/****************************************************************************
 *	MaSndDrv_LedOn
 *
 *	Description:
 *			Turn on the LED.
 *	Arguments:
 *			seq_id		sequence id (0..2)
 *			delta_time	delta time [tick]
 *			p1			unused
 *			p2			unused
 *			p3			unused
 *	Return:
 *			0			success
 *			< 0			error code
 *
 ****************************************************************************/
static SINT32 MaSndDrv_LedOn
(
	SINT32	seq_id,						/* sequence id */
	SINT32	delta_time,					/* delta time */
	UINT32	p1,							/* unused */
	UINT32	p2,							/* unused */
	UINT32	p3							/* unused */
)
{
	UINT32	num;						/* number of packet data */
	UINT32	reg_index;					/* register index number */
	SINT32	result;						/* result of function */
	
	(void)p1;							/* for unused warning message */
	(void)p2;
	(void)p3;

	MASNDDRV_DBGMSG((" MaSndDrv_LedOn: id=%ld dt=%ld\n", seq_id, delta_time ));

	/* Only allowed to delayed sequence. */

	MASMW_ASSERT( seq_id == MASMW_SEQTYPE_DELAYED );

	if ( ma_led_info.ctrl == 2 )
	{

		num = MakeDeltaTime( &packet_buf[seq_id][0], delta_time );

		reg_index = (UINT32)MA_LED_CTRL;
		MAKE_ADDRESS_PART( seq_id, num, reg_index )
		MAKE_LED( seq_id, num, 1 )
	
		if ( delta_time < 0 )			/* direct command */
		{
			result = MaDevDrv_SendDirectPacket( &packet_buf[seq_id][0], num );
		}
		else							/* delayed command */
		{
			result = SendDelayedPacket( &packet_buf[seq_id][0], num );
		}
	}
	else
	{
		if ( delta_time > 0 )
			result = MaSndDrv_Nop( seq_id, delta_time, 0, 0, 0 );
		else
			result = MASMW_SUCCESS;
	}

	return result;
}
/****************************************************************************
 *	MaSndDrv_LedOff
 *
 *	Description:
 *			Turn off the LED.
 *	Arguments:
 *			seq_id		sequence id (0..2)
 *			delta_time	delta time [tick]
 *			p1			unused
 *			p2			unused
 *			p3			unused
 *	Return:
 *			number of byte data of created packet
 *
 ****************************************************************************/
static SINT32 MaSndDrv_LedOff
(
	SINT32	seq_id,						/* sequence id */
	SINT32	delta_time,					/* delta time */
	UINT32	p1,							/* unused */
	UINT32	p2,							/* unused */
	UINT32	p3							/* unused */
)
{
	UINT32	num;						/* number of packet data */
	UINT32	reg_index;					/* register index number */
	SINT32	result;						/* result of function */

	(void)p1;							/* for unused warning message */
	(void)p2;
	(void)p3;
	
	MASNDDRV_DBGMSG((" MaSndDrv_LedOff: id=%ld dt=%ld\n", seq_id, delta_time ));

	/* Only allowed to delayed sequence. */

	MASMW_ASSERT( seq_id == MASMW_SEQTYPE_DELAYED );

	if ( ma_led_info.ctrl == 2 )
	{

		num = MakeDeltaTime( &packet_buf[seq_id][0], delta_time );

		reg_index = (UINT32)MA_LED_CTRL;
		MAKE_ADDRESS_PART( seq_id, num, reg_index )
		MAKE_LED( seq_id, num, 0 )

		if ( delta_time < 0 )			/* direct command */
		{
			result = MaDevDrv_SendDirectPacket( &packet_buf[seq_id][0], num );
		}
		else							/* delayed command */
		{
			result = SendDelayedPacket( &packet_buf[seq_id][0], num );
		}
	}
	else
	{
		if ( delta_time > 0 )
			result = MaSndDrv_Nop( seq_id, delta_time, 0, 0, 0 );
		else
			result = MASMW_SUCCESS;
	}

	return result;
}
/****************************************************************************
 *	MaSndDrv_MotorOn
 *
 *	Description:
 *			Turn on the motor.
 *	Arguments:
 *			seq_id		sequence id (0..2)
 *			delta_time	delta time [tick]
 *			p1			unused
 *			p2			unused
 *			p3			unused
 *	Return:
 *			number of byte data of created packet
 *
 ****************************************************************************/
static SINT32 MaSndDrv_MotorOn
(
	SINT32	seq_id,						/* sequnece id */
	SINT32	delta_time,					/* delta time */
	UINT32	p1,							/* unused */
	UINT32	p2,							/* unused */
	UINT32	p3							/* unused */
)
{
	UINT32	num;						/* number of packet data */
	UINT32	reg_index;					/* register index number */
	SINT32	result;						/* result of function */

	(void)p1;							/* for unused warning message */
	(void)p2;
	(void)p3;

	MASNDDRV_DBGMSG((" MaSndDrv_MotorOn: id=%ld dt=%ld\n", seq_id, delta_time ));

	/* Only allowed to delayed sequence. */

	MASMW_ASSERT( seq_id == MASMW_SEQTYPE_DELAYED );

	if ( ma_mtr_info.ctrl == 2 )
	{

		num = MakeDeltaTime( &packet_buf[seq_id][0], delta_time );
		
		reg_index = (UINT32)MA_MOTOR_CTRL;
		MAKE_ADDRESS_PART( seq_id, num, reg_index )
		MAKE_MOTOR( seq_id, num, 1 )
	
		if ( delta_time < 0 )			/* direct command */
		{
			result = MaDevDrv_SendDirectPacket( &packet_buf[seq_id][0], num );
		}
		else							/* delayed command */
		{
			result = SendDelayedPacket( &packet_buf[seq_id][0], num );
		}
	}
	else
	{
		if ( delta_time > 0 )
			result = MaSndDrv_Nop( seq_id, delta_time, 0, 0, 0 );
		else
			result = MASMW_SUCCESS;
	}
	
	return result;
}
/****************************************************************************
 *	MaSndDrv_MotorOff
 *
 *	Description:
 *			Turn off the motor.
 *	Arguments:
 *			seq_id		sequence id (0..2)
 *			delta_time	delta time [tick]
 *			p1			unused
 *			p2			unused
 *			p3			unused
 *	Return:
 *			number of byte data of created packet
 *
 ****************************************************************************/
static SINT32 MaSndDrv_MotorOff
(
	SINT32	seq_id,						/* sequence id */
	SINT32	delta_time,					/* delta time */
	UINT32	p1,							/* unused */
	UINT32	p2,							/* unused */
	UINT32	p3							/* unused */
)
{
	UINT32	num;						/* number of packet data */
	UINT32	reg_index;					/* register index number */
	SINT32	result;						/* result of function */

	(void)p1;							/* for unused warning message */
	(void)p2;
	(void)p3;

	MASNDDRV_DBGMSG((" MaSndDrv_MotorOff: id=%ld dt=%ld\n", seq_id, delta_time ));

	/* Only allowed to delayed sequence. */

	MASMW_ASSERT( seq_id == MASMW_SEQTYPE_DELAYED );

	if ( ma_mtr_info.ctrl == 2 )
	{

		num = MakeDeltaTime( &packet_buf[seq_id][0], delta_time );

		reg_index = (UINT32)MA_MOTOR_CTRL;
		MAKE_ADDRESS_PART( seq_id, num, reg_index )
		MAKE_MOTOR( seq_id, num, 0 )
	
		if ( delta_time < 0 )			/* direct command */
		{
			result = MaDevDrv_SendDirectPacket( &packet_buf[seq_id][0], num );
		}
		else							/* delayed command */
		{
			result = SendDelayedPacket( &packet_buf[seq_id][0], num );
		}
	}
	else
	{
		if ( delta_time > 0 )
			result = MaSndDrv_Nop( seq_id, delta_time, 0, 0, 0 );
		else
			result = MASMW_SUCCESS;
	}

	return result;
}
/****************************************************************************
 *	MaSndDrv_StreamSeek
 *
 *	Description:
 *			Set seek point of stream
 *	Argument:
 *			seq_id		sequence id (0..2)
 *			delta_time	delta time [tick]
 *			ch			channel number (0..15)
 *			wave_id		wave id number (0..31)
 *			seek_pos	seek point (byte)
 *	Return:
 *			0			success
 *			< 0			error code
 *
 ****************************************************************************/
static SINT32 MaSndDrv_StreamSeek
(
	SINT32	seq_id,						/* sequence id */
	SINT32	delta_time,					/* delta time */
	UINT32	ch,							/* unused */
	UINT32	wave_id,					/* wave id */
	UINT32	seek_pos					/* seek point */
)
{
	SINT32	result;						/* result of function */

	(void)ch;							/* for unused warning message */

	MASNDDRV_DBGMSG((" MaSndDrv_StreamSeek: id=%ld dt=%ld ch=%ld id=%ld sp=%ld\n", seq_id, delta_time, ch, wave_id, seek_pos ));

	/* check arugment */
	if ( wave_id > MASMW_MAX_WAVEID )	return MASMW_ERROR_ARGUMENT;

	/* already allocated stream audio resources, setting seek point */
	if ( ma_snddrv_info[seq_id].sa_num != 0 )
	{
		/* registered stream audio at specified wave id, setting it */
		if ( ma_stream_info[seq_id][wave_id].frequency != 0 )
		{
			/* send seek point to Resource Manager */
			MaResMgr_SetStreamSeekPos( (UINT8)wave_id, seek_pos );
		}
	}

	/* send delta_time, if needed */
	if ( delta_time > 0 )
		result = MaSndDrv_Nop( seq_id, delta_time, 0, 0, 0 );
	else
		result = MASMW_SUCCESS;

	return result;
}
/****************************************************************************
 *	MaSndDrv_SetCommand
 *
 *	Description:
 *			
 *	Arguments:
 *			seq_id		sequence id (0..2)
 *			delta_time	delta time [tick]
 *			cmd			command number
 *			param1		parameter 1
 *			param2		parameter 2
 *			param3		parameter 3
 *	Return:
 *			0			success
 *			< 0			error code
 *
 ****************************************************************************/
SINT32 MaSndDrv_SetCommand
(
	SINT32	seq_id,						/* sequence id */
	SINT32	delta_time,					/* delta time */
	UINT32	cmd,						/* command number */
	UINT32	param1,						/* parameter 1 */
	UINT32	param2,						/* parameter 2 */
	UINT32	param3						/* parameter 3 */
)
{
	SINT32	result;						/* result of function */
	
	MASNDDRV_DBGMSG(("MaSndDrv_SetCommand: id=%ld dt=%ld cmd=%ld p1=%ld p2=%ld p3=%ld\n", seq_id, delta_time, cmd, param1, param2, param3 ));

	MASMW_ASSERT( seq_id <= MASMW_MAX_SEQTYPE );
	MASMW_ASSERT( ma_snddrv_info[seq_id].status == 1 );
	MASMW_ASSERT( cmd <= MASMW_MAX_COMMAND );

	if ( seq_id != MASMW_SEQTYPE_AUDIO )
	{
		if ( cmd <= MASNDDRV_CMD_STREAM_SLAVE )
		{
			param1 &= MASMW_MASK_CHANNEL;

			if ( ( ma_snddrv_info[seq_id].ch_map & (UINT32)(1 << param1) ) == 0 )
			{
				cmd = MASNDDRV_CMD_NOP;
				param1 = 0;						/* NOP_1 */
			}
			else if ( seq_id == MASMW_SEQTYPE_DIRECT )
			{
				param1 = (SINT32)(15 - param1);
			}
		}
	}

	cmd &= MASMW_MASK_COMMAND;
	result = ma_snddrv_command[cmd]( seq_id, delta_time, param1, param2, param3 );
	
	return result;
}
/****************************************************************************
 *	MaSndDrv_SeekControl
 *
 *	Description:
 *			seek.
 *	Arguments:
 *			seq_id		sequence id
 *	Return:
 *			0			success
 *			< 0			error code
 *
 ****************************************************************************/
static SINT32 MaSndDrv_SeekControl
(
	SINT32	seq_id						/* sequence id */
)
{
	UINT16	time_out;					/* time out count */
	UINT16	buf_size;					/* size of buffer */
	SINT32	result;						/* result of function */

	MASNDDRV_DBGMSG(("MaSndDrv_SeekControl: id=%ld\n", seq_id ));

	/* get buffer pointer */
	ma_seqbuf_info.buf_ptr  = MaDevDrv_GetSeekBuffer( &buf_size );
	ma_seqbuf_info.buf_size = buf_size;

	/* clear FIFO */
	MaDevDrv_ClearFifo();

	/* call converter to make seek control data */
	result = 1;
	time_out = 1024;
	while ( ( result > 0 ) && ( result != 0xFFFF ) && ( --time_out !=0 ) )
	{
		result = ma_srm_cnv[seq_id]();
	}

	/* send control data created by converter */
	result = MaDevDrv_SeekControl( seq_id, ma_seqbuf_info.wrote_size );

	/* clear FIFO */
	MaDevDrv_ClearFifo();

	return MASMW_SUCCESS;
}
/****************************************************************************
 *	MaSndDrv_ControlSequencer
 *
 *	Description:
 *			
 *	Arguments:
 *			seq_id		sequence id
 *			ctrl		control value
 *	Return:
 *			0			success
 *			< 0			error code
 ****************************************************************************/
SINT32 MaSndDrv_ControlSequencer
(
	SINT32	seq_id,						/* sequence id */
	UINT8	ctrl						/* control value */
)
{
	UINT8	packet0[3] = { 0x58, 0x82, 0x80 };	/* timer 1 stop */
	UINT8	packet1[3] = { 0x58, 0x82, 0x85 };	/* timer 1 start */
	UINT32	slot_no;
	UINT8	ch;
	UINT32	sa_id;
	UINT32	num;
	UINT32	reg_index;
	UINT32	res_map;
	SINT32	result;

	MASNDDRV_DBGMSG(("MaSndDrv_ControlSequencer: id=%ld ctrl=%d\n", seq_id, ctrl ));

	switch ( ctrl )
	{
	case 0:							/* stop */

		if ( ma_control_sequencer[seq_id] == 0 )
		{
			MaDevDrv_StopSequencer( seq_id, 1 );

			num = 0;

			/* FM all slot sound off */
			res_map = ma_snddrv_info[seq_id].fm_map;
			for ( slot_no = 0; slot_no < MA_MAX_FM_SLOT; slot_no++ )
			{
				if ( ( res_map & (UINT32)(1 << slot_no) ) != 0 )
				{
					if ( slot_no < 16 )
						reg_index = (UINT32)( MA_FM_VOICE_ADDRESS + slot_no * 6 + 5 );
					else
						reg_index = (UINT32)( MA_EXT_FM_VOICE_ADDRESS + (slot_no - 16) * 6 + 5 );

					ch = (UINT8)( MaDevDrv_ReceiveData( reg_index, 0 ) & 0x0F );

					/* KeyOff & MUTE & RST */
					MAKE_ADDRESS_PART( seq_id, num, reg_index )
					MAKE_RST_MUTE( seq_id, num, ch )
				}
			}

			/* WT all slot sound off */
			res_map = ma_snddrv_info[seq_id].wt_map;
			for ( slot_no = 0; slot_no < MA_MAX_WT_SLOT; slot_no++ )
			{
				if ( ( res_map & (UINT32)(1 << slot_no) ) != 0 )
				{
					reg_index = (UINT32)( MA_WT_VOICE_ADDRESS + slot_no * 6 + 5 );
					ch = (UINT8)( MaDevDrv_ReceiveData( reg_index, 0 ) & 0x0F );

					/* KeyOff & MUTE & RST */
					MAKE_ADDRESS_PART( seq_id, num, reg_index )
					MAKE_RST_MUTE( seq_id, num, ch )
				}
			}

			/* SA all slot sound off */
			for ( sa_id = 0; sa_id < MA_MAX_STREAM_AUDIO; sa_id++ )
			{
				if ( ( ma_snddrv_info[seq_id].sa_map & (UINT32)(1 << sa_id) ) != 0 )
				{
					/* Stream Off, setup */
					reg_index = (UINT32)(MA_SOFTINT_RAM + sa_id);
					MAKE_ADDRESS_PART( seq_id, num, reg_index );
					MAKE_SOFTINT_RAM( seq_id, num, 0x00 );
				}
			}

			if ( num != 0 )
			{
				MaDevDrv_SendDirectPacket( &packet_buf[seq_id][0], num );
			}

			for ( sa_id = 0; sa_id < MA_MAX_STREAM_AUDIO; sa_id++ )
			{
				if ( ( ma_snddrv_info[seq_id].sa_map & (UINT32)(1 << sa_id) ) != 0 )
				{
					/* Stream Off */
					MaDevDrv_StreamHandler( (UINT8)sa_id, 0, 0x00 );
				}
			}

			if ( seq_id == MASMW_SEQTYPE_DELAYED )
			{
				MaSndDrv_LedOff( seq_id, -1, 0, 0, 0 );
				MaSndDrv_MotorOff( seq_id, -1, 0, 0, 0 );
			}
		}

		result = MASMW_SUCCESS;
		break;

	case 1:							/* start */

		if ( ma_control_sequencer[seq_id] == 0 )
		{
			if ( ( seq_id == 0 )
			  && ( ma_seqbuf_info.sequence_flag == (UINT8)(MA_INT_POINT - 2) ) )
			{
				MaDevDrv_Fifo( ma_seqbuf_info.sequence_flag );
			}
			MaDevDrv_StartSequencer( seq_id, 1 );
		}
		ma_seqbuf_info.sequence_flag = 0;

		result = MASMW_SUCCESS;
		break;

	case 2:
	
		ma_control_sequencer[seq_id] = 1;
		result = MASMW_SUCCESS;
		break;

	case 3:
	
		ma_control_sequencer[seq_id] = 0;
		result = MASMW_SUCCESS;
		break;

	case 4:		/* seek */

		if ( ma_control_sequencer[seq_id] != 1 )
		{
			ma_pos_count[seq_id] = 0;
		}

		for ( slot_no = 0; slot_no < ma_snddrv_info[seq_id].fm_num; slot_no++ )
		{
			if ( ( ma_snddrv_info[seq_id].fm_map & (UINT32)(1 << slot_no) ) != 0 )
			{
				ma_slot_info[slot_no].ch   = 0;
			}
		}
		for ( slot_no = 0; slot_no < ma_snddrv_info[seq_id].wt_num; slot_no++ )
		{
			if ( ( ma_snddrv_info[seq_id].wt_map & (UINT32)(1 << slot_no) ) != 0 )
			{
				ma_slot_info[slot_no+32].ch   = 0;
			}
		}
		for ( slot_no = 0; slot_no < ma_snddrv_info[seq_id].sa_num; slot_no++ )
		{
			if ( ( ma_snddrv_info[seq_id].sa_map & (UINT32)(1 << slot_no) ) != 0 )
			{
				ma_slot_info[slot_no+40].ch   = 0;
			}
		}

		for ( ch = 0; ch < ma_snddrv_info[seq_id].ch_num; ch++ )
		{
			if ( ( ma_snddrv_info[seq_id].ch_map & (UINT32)(1 << ch) ) != 0 )
			{
				ma_channel_info[ch].note_on = 0;
			}
		}

		if ( seq_id == MASMW_SEQTYPE_DELAYED )
		{
			if ( ma_control_sequencer[seq_id] == 0 )
			{
				ma_seqbuf_info.buf_ptr		= 0;
				ma_seqbuf_info.buf_size		= 0;
				ma_seqbuf_info.queue_flag 	= 0;
				ma_seqbuf_info.queue_size	= 0;
				ma_seqbuf_info.queue_wptr	= 0;
				ma_seqbuf_info.wrote_size	= 0;

				MaDevDrv_SendDirectPacket( packet0, 3 );

				MaSndDrv_SeekControl( seq_id );

				ma_seqbuf_info.buf_ptr		= 0;
				ma_seqbuf_info.buf_size		= 0;
				ma_seqbuf_info.queue_flag 	= 0;
				ma_seqbuf_info.queue_size	= 0;
				ma_seqbuf_info.queue_wptr	= 0;
				ma_seqbuf_info.wrote_size	= 0;

				ma_seqbuf_info.sequence_flag = (UINT8)(MA_INT_POINT - 2);

				MaDevDrv_SendDirectPacket( packet1, 3 );
			}
		}
		else if ( seq_id == MASMW_SEQTYPE_AUDIO )
		{
			MaDevDrv_GetStreamPos( 1 );
		}
		result = MASMW_SUCCESS;
		break;

	default:
		result = MASMW_ERROR;
		break;
	}

	return result;
}
/****************************************************************************
 *	MaSndDrv_SetFmExtWave
 *
 *	Description:
 *			
 *	Arguments:
 *			seq_id
 *			wave_no
 *			ram_adrs
 *	Return:
 *			0			success
 *			< 0			error code
 *
 ****************************************************************************/
SINT32 MaSndDrv_SetFmExtWave
(
	SINT32	seq_id,						/* sequence id */
	UINT8	wave_no,					/* wave number */
	UINT32	ram_adrs					/* ram address */
)
{
	UINT8	packet[4];
	UINT32	reg_index;
	SINT32	result;
	
	MASNDDRV_DBGMSG(("MaSndDrv_SetFmExtVoice: id=%ld wn=%d ad=%08lx\n", seq_id, wave_no, ram_adrs));

	if ( seq_id > MASMW_MAX_SEQTYPE )			return MASMW_ERROR_ID;
	if ( ma_snddrv_info[seq_id].status != 1 )	return MASMW_ERROR_ID;

	if ( ram_adrs == 0xFFFFFFFF ) return MASMW_SUCCESS;

	switch ( wave_no )
	{
	case 15:	reg_index = (UINT32)MA_FM_EXTWAVE_15;	break;
	case 23:	reg_index = (UINT32)MA_FM_EXTWAVE_23;	break;
	case 31:	reg_index = (UINT32)MA_FM_EXTWAVE_31;	break;
	default:	return MASMW_ERROR_ARGUMENT;
	}

	ram_adrs >>= 1;

	packet[0] = (UINT8)(  reg_index       & 0x7F );		/* address part */
	packet[1] = (UINT8)( (reg_index >> 7) | 0x80 );
	packet[2] = (UINT8)( (ram_adrs >> 7)  & 0x7F );		/* data part */
	packet[3] = (UINT8)(  ram_adrs        | 0x80 );

	result = MaDevDrv_SendDirectPacket( packet, 4 );
	
	return result;
}
/****************************************************************************
 *	MaSndDrv_SetVolume
 *
 *	Description:
 *			Set volume of specified sequence.
 *	Argument:
 *			seq_id		sequence id
 *			volume		volume (0..127)
 *	Return:
 *			0			success
 *			< 0			error code
 *
 ****************************************************************************/
SINT32 MaSndDrv_SetVolume
(
	SINT32	seq_id,						/* sequence id */
	UINT8	volume						/* volume */
)
{
	MASNDDRV_DBGMSG(("MaSndDrv_SetVolume: id=%ld vl=%d\n", seq_id, volume));

	if ( seq_id > MASMW_MAX_SEQTYPE ) return MASMW_ERROR_ID;
	if ( volume > MASMW_MAX_VOLUME ) return MASMW_ERROR_ARGUMENT;
	
	ma_snddrv_info[seq_id].ctrl_volume = volume;
	
	return MASMW_SUCCESS;
}
/****************************************************************************
 *	MaSndDrv_SetVoice
 *
 *	Description:
 *			Set timber of voice
 *	Argument:
 *			seq_id		sequence id
 *			bank_no		bank number (0,1..127: melody, 128,129..255: drum)
 *			program_no	program (key) number (0..127)
 *			synth		synth mode (1:FM, 2:WT)
 *			key			key number of drum (0..127)
 *			ram_adrs	internal RAM address
 *	Return:
 *			0			success
 *			< 0			error code
 *
 ****************************************************************************/
SINT32 MaSndDrv_SetVoice
(
	SINT32	seq_id,						/* sequence id */
	UINT8	bank_no,					/* bank number */
	UINT8	program_no,					/* program number */
	UINT8	synth,						/* synth type */
	UINT32	key,						/* key number */
	UINT32	ram_adrs					/* ram address */
)
{
	UINT32	state = 0;
	UINT16	bank_prog;					/* bank & program number */
	UINT32	i;
	SINT32	result = MASMW_SUCCESS;
	UINT32	real_key;

	MASNDDRV_DBGMSG(("MaSndDrv_SetVoice: id=%ld bn=%d pb=%d sy=%d key=%ld ad=%08lx\n", seq_id, bank_no, program_no, synth, key, ram_adrs));

	/* check arguments */

	if ( seq_id > MASMW_SEQTYPE_DIRECT )		return MASMW_ERROR_ID;
	if ( ma_snddrv_info[seq_id].status != 1 )	return MASMW_ERROR_ID;
	if ( program_no > MASMW_MAX_PROGRAM )		return MASMW_ERROR_ARGUMENT;
	if ( ( ram_adrs != 0xFFFFFFFF ) && ( ( ram_adrs < MASMW_MIN_RAMADRS ) || ( MASMW_MAX_RAMADRS < ram_adrs ) ) )
		return MASMW_ERROR_ARGUMENT;

	if ((bank_no & 0x7f) >= MA_MAX_RAM_BANK) return MASMW_ERROR_ARGUMENT;
	if ( bank_no < 128 )
	{
		bank_prog = (UINT16)(((UINT16)bank_no << 7) + (UINT16)program_no);
	}
	else
	{
		bank_prog = (UINT16)(((UINT16)(bank_no - 128 + MA_MAX_RAM_BANK) << 7) + (UINT16)program_no);
	}

	if ( ram_adrs == (UINT32)0xFFFFFFFF )
	{
		if ( ma_voice_index[bank_prog] != 0xff )
		{
			state = 1;
			i = ma_voice_index[bank_prog];
			ma_voice_info[seq_id][i].ram_adrs = 0xFFFF;
			ma_voice_index[bank_prog] = 0xff;
		}
	}
	else
	{
		if (ma_voice_index[bank_prog] == 0xff)
		{
			state = 1;

			if ( synth == 2 )	real_key = ( ( ( (key * 0x800L) / 48000L) + 1 ) >> 1 );
			else				real_key = key;

			/*--- 1st Try ----*/
			i = (bank_no & 0x80) + program_no;
			if ( ma_voice_info[seq_id][i].ram_adrs == 0xFFFF )
			{
				state = 1;
				ma_voice_info[seq_id][i].synth 		= synth;
				ma_voice_info[seq_id][i].key	  	= (UINT16)real_key;
				ma_voice_info[seq_id][i].ram_adrs	= (UINT16)ram_adrs;
				ma_voice_index[bank_prog] = (UINT8)i;
			} 
			else
			{
				for ( i = 0; i < (MA_MAX_REG_VOICE * 2 - 1); i++ )
				{
					if ( ma_voice_info[seq_id][i].ram_adrs == 0xFFFF )
					{
						state = 1;
						ma_voice_info[seq_id][i].synth 		= synth;
						ma_voice_info[seq_id][i].key	  	= (UINT16)real_key;
						ma_voice_info[seq_id][i].ram_adrs	= (UINT16)ram_adrs;
						ma_voice_index[bank_prog] = (UINT8)i;
						break;
					}
				}
			}
		}
	}

	if ( state == 0 ) result = MASMW_ERROR;

	return result;
}
/****************************************************************************
 *	MaSndDrv_GetVoice
 *
 *	Description:
 *			Return entry of specified timber.
 *	Argument:
 *			seq_id		sequence id
 *			bank_no		bank number (0,1..127: melody, 127,128..255: drum)
 *			program_no	program (key) number (0..127)
 *	Return:
 *			1			entried
 *			0			not entry
 *			< 0			error code
 *
 ****************************************************************************/
SINT32 MaSndDrv_GetVoice
(
	SINT32	seq_id,						/* sequence id */
	UINT8	bank_no,					/* bank number */
	UINT8	program_no					/* program number */
)
{
	UINT16	bank_prog;					/* bank & program number */

	MASNDDRV_DBGMSG(("MaSndDrv_GetVoice: id=%ld bn=%d pb=%d\n", seq_id, bank_no, program_no));

	/* check arguments */

	if ( seq_id > MASMW_SEQTYPE_DIRECT )		return MASMW_ERROR_ID;
	if ( ma_snddrv_info[seq_id].status != 1 )	return MASMW_ERROR_ID;
	if ( program_no > MASMW_MAX_PROGRAM )		return MASMW_ERROR_ARGUMENT;

	if ( bank_no < 128 )
	{
		bank_prog = (UINT16)(((UINT16)bank_no << 7) + (UINT16)program_no);
	}
	else
	{
		bank_prog = (UINT16)(((UINT16)(bank_no - 128 + MA_MAX_RAM_BANK) << 7) + (UINT16)program_no);
	}

	if ( (bank_no & 0x7f) >= MA_MAX_RAM_BANK ) return (0);
	if ( ma_voice_index[bank_prog] != 0xff ) return (1);

	return (0);
}
/****************************************************************************
 *	MaSndDrv_SetStream
 *
 *	Description:
 *			Set stream audio data.
 *	Arguments:
 *			seq_id		sequence id
 *			wave_id		wave id (0..31)
 *			format		wave format		0: MA-2 ADPCM
 *										1: ADPCM
 *										2: 8bit offset binary PCM
 *										3: 8bit 2's comp binary PCM
 *			frequency	sampling frequency (0, 4000..48000[Hz])
 *			wave_ptr	pointer to the wave data
 *			wave_size	byte size of the wave data
 *	Return:
 *			0			success
 *			< 0			error code
 *
 ****************************************************************************/
SINT32 MaSndDrv_SetStream
(
	SINT32	seq_id,						/* sequence id */
	UINT8	wave_id,					/* wave id */
	UINT8	format,						/* wave format */
	UINT32	frequency,					/* sampling frequency */
	UINT8 *	wave_ptr,					/* pointer to the wave data */
	UINT32	wave_size					/* byte size of the wave data */
)
{
	SINT32	result;

	MASNDDRV_DBGMSG(("MaSndDrv_SetStream: id=%ld wv=%d ft=%d fs=%ld\n", seq_id, wave_id, format, frequency));

	/* check arguments */

	if ( seq_id > MASMW_MAX_SEQTYPE )			return MASMW_ERROR_ID;
	if ( ma_snddrv_info[seq_id].status != 1 )	return MASMW_ERROR_ID;
	if ( wave_id > MASMW_MAX_WAVEID )			return MASMW_ERROR_ARGUMENT;
	if ( format > MASMW_MAX_FORMAT )			return MASMW_ERROR_ARGUMENT;
	if ( ( frequency != 0 )
	  && ( ( frequency < MASMW_MIN_STREAMFS ) || ( MASMW_MAX_STREAMFS < frequency ) ) )
		return MASMW_ERROR_ARGUMENT;

	if ( ma_snddrv_info[seq_id].sa_num == 0 )
	{
		return MASMW_SUCCESS;
	}

#if MASMW_DEBUG
	result = madebug_SetStream( seq_id, wave_id, format, frequency, wave_ptr, wave_size );
	if ( result < MASMW_SUCCESS ) return result;
#endif

	if ( frequency == 0 )
	{

		result = MaResMgr_DelStreamAudio( wave_id );
		if ( result != MASMW_SUCCESS ) return result;

		ma_stream_info[seq_id][wave_id].frequency = 0;
	}
	else
	{

		result = MaResMgr_RegStreamAudio( wave_id, format, wave_ptr, wave_size );
		if ( result != MASMW_SUCCESS ) return result;

		ma_stream_info[seq_id][wave_id].format 	  = format;
		ma_stream_info[seq_id][wave_id].pan		  = (UINT8)MASMW_MAX_STREAMPAN;
		ma_stream_info[seq_id][wave_id].frequency = frequency;
	}

	return MASMW_SUCCESS;
}
/****************************************************************************
 *	MaSndDrv_GetTimeError
 *
 *	Description:
 *			get time error.
 *	Argument:
 *			seq_id		sequence id
 *	Return:
 *			0			success
 *			< 0			error code
 *
 ****************************************************************************/
SINT32 MaSndDrv_GetTimeError
(
	SINT32	seq_id						/* sequence id */
)
{
	SINT32	result;

	MASNDDRV_DBGMSG(("MaSndDrv_GetTimeError: id=%ld\n", seq_id));

	if ( seq_id == 0 )
	{
		result = 0;
	}
	else
	{
		result = 0;
	}

	return result;
}
/****************************************************************************
 *	MaSndDrv_GetPos
 *
 *	Description:
 *			Get current position.
 *	Arguments:
 *			seq_id		sequence id
 *	Return:
 *			>= 0		current play position [ms]
 *			< 0			error code
 *
 ****************************************************************************/
SINT32 MaSndDrv_GetPos
(
	SINT32	seq_id						/* sequence id */
)
{
	UINT32	position;
	SINT32	result;

	MASNDDRV_DBGMSG(("MaSndDrv_GetPos: id=%ld\n", seq_id));

	switch ( seq_id )
	{
	case MASMW_SEQTYPE_DELAYED:

		position = ma_pos_count[0] * 100;
		position += (UINT32)( 100 - MaDevDrv_ReceiveData( MA_TIMER_1_COUNT, 0 ) );
		result = (SINT32)(position * (UINT32)ma_snddrv_info[seq_id].time_base);
		break;

	case MASMW_SEQTYPE_DIRECT:

		result = MASMW_ERROR;
		break;

	case MASMW_SEQTYPE_AUDIO:

		result = (SINT32)MaDevDrv_GetStreamPos(0);
		break;

	default:
		result = MASMW_ERROR;
		break;
	}

	return result;
}
/****************************************************************************
 *	MaSndDrv_SetSpeedWide
 *
 *	Description:
 *			Sets speed of play.
 *	Arguments:
 *			seq_id		sequence id
 *			val			play speed (25..100..400)
 *	Return:
 *			0			success
 *			< 0			error code
 *
 ****************************************************************************/
SINT32 MaSndDrv_SetSpeedWide
(
	SINT32	seq_id,						/* sequence id */
	UINT32	val							/* speed value */
)
{
	UINT8	packet[4];
	UINT32	base_time;
	SINT32	result;

	MASNDDRV_DBGMSG(("MaSndDrv_SetSpeedWide: id=%ld val=%ld\n", seq_id, val));

	if ( ( 25 > val ) || ( val > 400 ) ) return MASMW_ERROR_ARGUMENT;

	switch ( seq_id )
	{
	case MASMW_SEQTYPE_DELAYED:

		result = MASMW_SUCCESS;

		if ( ma_snddrv_info[seq_id].speed != val )
		{
			ma_snddrv_info[seq_id].speed = val;

			base_time = ( (UINT32)ma_snddrv_info[seq_id].time_base * (UINT32)100000000 )
			            / (1157 * (UINT32)val);

			packet[0] = (UINT8)(  MA_SEQUENCE       & 0x7F );	/* address */
			packet[1] = (UINT8)( (MA_SEQUENCE >> 7) | 0x80 );
			packet[2] = (UINT8)( (base_time   >> 7) & 0x7F );	/* data */
			packet[3] = (UINT8)(  base_time         | 0x80 );
			result = MaDevDrv_SendDirectPacket( packet, 4 );
		}

		break;

	case MASMW_SEQTYPE_DIRECT:
	case MASMW_SEQTYPE_AUDIO:

		result = MASMW_ERROR;
		break;

	default:
		result = MASMW_ERROR;
		break;
	}

	return result;
}
/****************************************************************************
 *	MaSndDrv_SetSpeed
 *
 *	Description:
 *			Sets speed of play.
 *	Arguments:
 *			seq_id		sequence id
 *			val			play speed (70..100..130)
 *	Return:
 *			0			success
 *			< 0			error code
 *
 ****************************************************************************/
SINT32 MaSndDrv_SetSpeed
(
	SINT32	seq_id,						/* sequence id */
	UINT8	val							/* speed value */
)
{
	UINT8	packet[4];
	UINT32	base_time;
	SINT32	result;

	MASNDDRV_DBGMSG(("MaSndDrv_SetSpeed: id=%ld val=%d\n", seq_id, val));

	if ( ( 70 > val ) || ( val > 130 ) ) return MASMW_ERROR_ARGUMENT;

	switch ( seq_id )
	{
	case MASMW_SEQTYPE_DELAYED:

		result = MASMW_SUCCESS;

		if ( ma_snddrv_info[seq_id].speed != (UINT32)val )
		{
			ma_snddrv_info[seq_id].speed = (UINT32)val;

			base_time = ( (UINT32)ma_snddrv_info[seq_id].time_base * (UINT32)100000000 )
			            / (1157 * (UINT32)val);

			packet[0] = (UINT8)(  MA_SEQUENCE       & 0x7F );	/* address */
			packet[1] = (UINT8)( (MA_SEQUENCE >> 7) | 0x80 );
			packet[2] = (UINT8)( (base_time   >> 7) & 0x7F );	/* data */
			packet[3] = (UINT8)(  base_time         | 0x80 );
			result = MaDevDrv_SendDirectPacket( packet, 4 );
		}

		break;

	case MASMW_SEQTYPE_DIRECT:
	case MASMW_SEQTYPE_AUDIO:

		result = MASMW_ERROR;
		break;

	default:
		result = MASMW_ERROR;
		break;
	}

	return result;
}
/****************************************************************************
 *	MaSndDrv_SetKey
 *
 *	Description:
 *			Sets key offset value.
 *	Arguments:
 *			seq_id		sequence id (0..2)
 *			val			key offset value (-12..0..+12)
 *	Return:
 *			0			success
 *			< 0			error code
 *
 ****************************************************************************/
SINT32 MaSndDrv_SetKey
(
	SINT32	seq_id,						/* sequence id */
	SINT8	val							/* key offset value */
)
{
	MASNDDRV_DBGMSG(("MaSndDrv_SetKey: id=%ld val=%d\n", seq_id, val));

	if ( ( -12 > val ) || ( val > 12 ) ) return MASMW_ERROR_ARGUMENT;

	switch ( seq_id )
	{
	case MASMW_SEQTYPE_DELAYED:

		ma_snddrv_info[seq_id].key_offset = val;
		break;

	case MASMW_SEQTYPE_DIRECT:
	case MASMW_SEQTYPE_AUDIO:
		break;

	default:
		return MASMW_ERROR;
	}

	return MASMW_SUCCESS;
}
/****************************************************************************
 *	MaSndDrv_SetKeyControl
 *
 *	Description:
 *			Sets key control.
 *	Arguments:
 *			seq_id		sequence id (0..2)
 *			ch			channel number (0..15)
 *			val			contorol value (0: None, 1: OFF, 2: ON)
 *	Return:
 *			0			success
 *			< 0			error code
 *
 ****************************************************************************/
SINT32 MaSndDrv_SetKeyControl
(
	SINT32	seq_id,						/* sequence id */
	UINT8	ch,							/* channel number */
	UINT8	val							/* control value */
)
{
	MASNDDRV_DBGMSG(("MaSndDrv_SetKeyControl: id=%ld ch=%d val=%d\n", seq_id, ch, val));

	if ( ch > 15 ) return MASMW_ERROR_ARGUMENT;
	if ( val > 2 ) return MASMW_ERROR_ARGUMENT;

	switch ( seq_id )
	{
	case MASMW_SEQTYPE_DELAYED:

		if ( ( ma_snddrv_info[seq_id].ch_map & (UINT32)(1 << ch) ) != 0 )
		{
			ma_channel_info[ch].key_control = val;
		}
		break;

	case MASMW_SEQTYPE_DIRECT:
	case MASMW_SEQTYPE_AUDIO:
		break;

	default:
		return MASMW_ERROR;
	}

	return MASMW_SUCCESS;
}
/****************************************************************************
 *	MaSndDrv_Free
 *
 *	Description:
 *			Free the resources.
 *	Argument:
 *			seq_id		sequence id number
 *	Return:
 *			0			success
 *			< 0			error code
 *
 ****************************************************************************/
SINT32 MaSndDrv_Free
(
	SINT32	seq_id						/* sequence id */
)
{
	MASNDDRV_DBGMSG(("MaSndDrv_Free: id=%ld\n", seq_id));

	switch ( seq_id )
	{
	case MASMW_SEQTYPE_DELAYED:

		MaResMgr_FreeSequencer( 0 );
		MaResMgr_FreeTimer( 2 );
		break;
		
	case MASMW_SEQTYPE_DIRECT:

		MaResMgr_FreeSequencer( 1 );
		if ( ma_snddrv_info[seq_id].tm_map == 1 ) MaResMgr_FreeTimer( 0 );
		break;

	case MASMW_SEQTYPE_AUDIO:

		MaDevDrv_SetAudioMode( 0 );
		MaDevDrv_StopSequencer( 2, 1 );
		break;
		
	default:
		return MASMW_ERROR_ARGUMENT;
	}

	MaResMgr_FreeCh( ma_snddrv_info[seq_id].ch_map );
	MaResMgr_FreeFmVoice( ma_snddrv_info[seq_id].fm_map );
	MaResMgr_FreeWtVoice( ma_snddrv_info[seq_id].wt_map );
	MaResMgr_FreeRam( ma_snddrv_info[seq_id].rb_map );
	MaResMgr_FreeStreamAudio( ma_snddrv_info[seq_id].sa_map );
	MaResMgr_FreeLed( ma_snddrv_info[seq_id].ld_map );
	MaResMgr_FreeMotor( ma_snddrv_info[seq_id].mt_map );

	if ( ma_snddrv_info[seq_id].fm_num == 32 )
	{
		MaDevDrv_DeviceControl( 10, 0, 0, 0 );
	}

	ma_snddrv_info[seq_id].status = 0;

	return MASMW_SUCCESS;
}
/****************************************************************************
 *	MaSndDrv_Create
 *
 *	Description:
 *			Allocates the resouces.
 *	Arguments:
 *			seq_type	sequence type
 *			time_base	time base (0, 4..10, 20 [ms])
 *			cnv_mode	convert mode
 *						  bit   0: drum mode(0: normal, 1: compatible)
 *						  bit 2-1: DVA mode(0: normal, 1: simple, 2: compatible)
 *						  bit   3: melody mode(0: normal, 1: compatible)
 *						  bit   4: FM mode(0: 2-OP, 1: 4-OP)
 *			res_mode	resource mode (0, 1..16)
 *			sa_num		number of stream audio (0..2)
 *			srm_cnv		pointer to the converter function
 *			ram_adrs	start address of the internal RAM
 *			ram_size	byte size of the internal RAM
 *	Return:
 *			>= 0		success. sequence id number
 *			< 0			error code
 *
 ****************************************************************************/
SINT32 MaSndDrv_Create
(
	UINT8		seq_type,				/* sequence type */
	UINT8		time_base,				/* time base */
	UINT8		cnv_mode,				/* convert mode */
	UINT8		res_mode,				/* resource mode */
	UINT8		sa_num,					/* number of stream audio */
	SINT32 (*srm_cnv)(void),			/* pointer to converter function */
	UINT32 *	ram_adrs,				/* ram address */
	UINT32 *	ram_size				/* byte size of ram */
)
{
	UINT16	i;
	UINT8	rmode;						/* index of resource mode */
	UINT8	ch;							/* channel number */
	UINT8	slot_no;					/* slot number */
	UINT8	fm_ext_num;					/* number of FM EXT voices */
	UINT32	ch_map;						/* bit mapping for channel */
	UINT32	fm_map;						/* bit mapping for FM voices */
	UINT32	wt_map;						/* bit mapping for WT voices */
	UINT32	rb_map;						/* bit mapping for RAM blocks */
	UINT32	sa_map;						/* bit mapping for Stream Audio */
	UINT32	ld_map;						/* bit mapping for LED */
	UINT32	mt_map;						/* bit mapping for Motor */
	UINT32	tm_map;						/* bit mapping for Timer */
	UINT32	fm_ext_map;					/* bit mapping for FM EXT voices */
	UINT32	err_flag;					/* error flag */
	UINT32	res_flag;					/* flag of resources */
	SINT32	result;						/* result of allocate resources */
	UINT32	ram_address;

	static UINT32 res_map[3] = { 0x00000000, 0x00000001, 0x00000003 };
	static UINT32 msk_map[3] = { 0xFFFFFFFF, 0xFFFFFF7F, 0xFFFFFF3F };

	MASNDDRV_DBGMSG(("MaSndDrv_Create: st=%d tm=%d cm=%02x rm=%d sa=%d\n", seq_type, time_base, cnv_mode, res_mode, sa_num));
	
	if ( seq_type > MASMW_MAX_SEQTYPE ) return MASMW_ERROR_ARGUMENT;
	if ( cnv_mode > MASMW_MAX_CNVMODE ) return MASMW_ERROR_ARGUMENT;
	if ( res_mode > MASMW_MAX_RESMODE ) return MASMW_ERROR_ARGUMENT;
	if ( sa_num > MASMW_NUM_STREAM )    return MASMW_ERROR_ARGUMENT;

	tm_map = 0;

	switch ( seq_type )
	{
	case MASMW_SEQTYPE_DELAYED:


		if ( ( MASMW_MIN_DLY_BASETIME > time_base ) || ( time_base > MASMW_MAX_DLY_BASETIME ) )
			return MASMW_ERROR_ARGUMENT;

		result = MaResMgr_AllocSequencer( 0, ma_timebase[time_base-1] );
		if ( result != MASMW_SUCCESS )
		{
			return result;
		}

		result = MaResMgr_AllocTimer( 2, 0, 100, 1, 0 );
		if ( result != MASMW_SUCCESS )
		{
			MaResMgr_FreeSequencer( 0 );
			return result;
		}

		ld_map = 1;
		mt_map = 1;

		break;

	case MASMW_SEQTYPE_DIRECT:


		result = MaResMgr_AllocSequencer( 1, 0 );
		if ( result != MASMW_SUCCESS )
		{
			return result;
		}

		if ( time_base == MASMW_DIRECT_BASETIME ) 
		{
			result = MaResMgr_AllocTimer( 0, 0x1B, 20, 0, 0 );	/* 0.999 * 20 [ms] */
			if ( result != MASMW_SUCCESS )
			{
				return result;
			}

			tm_map = 1;
		}
		else if ( time_base != 0 )
		{
			return MASMW_ERROR;
		}

		ld_map = 0;
		mt_map = 0;

		break;

	case MASMW_SEQTYPE_AUDIO:


		if ( time_base != MASMW_AUDIO_BASETIME )
		{
			return MASMW_ERROR_ARGUMENT;
		}

		ld_map = 0;
		mt_map = 0;

		break;

	default:
		return MASMW_ERROR_ARGUMENT;
	}

	fm_ext_num = 0;
	fm_ext_map = 0;
	if ( ( res_mode == 0 ) && ( seq_type != MASMW_SEQTYPE_AUDIO ) )
	{
		if ( ( cnv_mode & 0x10 ) == 0 )
		{
			/* 2OP - 32 voices */
			fm_ext_num = 16;
			fm_ext_map = 0xFFFF0000;
		}
	}


	rmode = ma_resmode_index[res_mode];

	ch_map = ma_resmode_chmap[seq_type][rmode];
	fm_map = ma_resmode_fmmap[seq_type][rmode];
	wt_map = ma_resmode_wtmap[seq_type][rmode] & msk_map[sa_num];
	rb_map = ma_resmode_rbmap[seq_type][rmode] & msk_map[sa_num];

	sa_map = res_map[sa_num];

	err_flag = 0;
	res_flag = 0;

	result = MaResMgr_AllocCh( ch_map );
	if ( result != MASMW_SUCCESS )	err_flag |= (UINT32)(1 << 0);
	else							res_flag |= (UINT32)(1 << 0);

	result = MaResMgr_AllocFmVoice( fm_map );
	if ( result != MASMW_SUCCESS )	err_flag |= (UINT32)(1 << 1);
	else							res_flag |= (UINT32)(1 << 1);

	result = MaResMgr_AllocFmVoice( fm_ext_map );
	if ( result != MASMW_SUCCESS ) 	err_flag |= (UINT32)(1 << 2);
	else							res_flag |= (UINT32)(1 << 2);

	result = MaResMgr_AllocWtVoice( wt_map );
	if ( result != MASMW_SUCCESS )	err_flag |= (UINT32)(1 << 3);
	else							res_flag |= (UINT32)(1 << 3);

	result = MaResMgr_AllocRam( rb_map );
	if ( result != MASMW_SUCCESS )	err_flag |= (UINT32)(1 << 4);
	else							res_flag |= (UINT32)(1 << 4);

	result = MaResMgr_AllocStreamAudio( sa_map );
	if ( result != MASMW_SUCCESS )	err_flag |= (UINT32)(1 << 5);
	else							res_flag |= (UINT32)(1 << 5);

	result = MaResMgr_AllocLed( ld_map );
	if ( result != MASMW_SUCCESS )	err_flag |= (UINT32)(1 << 6);
	else							res_flag |= (UINT32)(1 << 6);

	result = MaResMgr_AllocMotor( mt_map );
	if ( result != MASMW_SUCCESS )	err_flag |= (UINT32)(1 << 7);
	else							res_flag |= (UINT32)(1 << 7);

	if ( err_flag != 0 )
	{
		switch ( seq_type )
		{
		case MASMW_SEQTYPE_DELAYED:
			MaResMgr_FreeSequencer( 0 );
			MaResMgr_FreeTimer( 2 );
			break;
		case MASMW_SEQTYPE_DIRECT:
			MaResMgr_FreeSequencer( 1 );
			if ( tm_map == 1 )	MaResMgr_FreeTimer( 0 );
			break;
		case MASMW_SEQTYPE_AUDIO:
			break;
		default:
			break;
		}
		
		if ( ( res_flag & (1 << 0) ) != 0 )	MaResMgr_FreeCh( ch_map );
		if ( ( res_flag & (1 << 1) ) != 0 )	MaResMgr_FreeFmVoice( fm_map );
		if ( ( res_flag & (1 << 2) ) != 0 )	MaResMgr_FreeFmVoice( fm_ext_map );
		if ( ( res_flag & (1 << 3) ) != 0 )	MaResMgr_FreeWtVoice( wt_map );
		if ( ( res_flag & (1 << 4) ) != 0 )	MaResMgr_FreeRam( rb_map );
		if ( ( res_flag & (1 << 5) ) != 0 )	MaResMgr_FreeStreamAudio( sa_map );
		if ( ( res_flag & (1 << 6) ) != 0 )	MaResMgr_FreeLed( ld_map );
		if ( ( res_flag & (1 << 7) ) != 0 )	MaResMgr_FreeMotor( mt_map );
		
		return MASMW_ERROR_RESOURCE_OVER;
	}

	if ( fm_ext_num == 16 )
	{
		MaDevDrv_DeviceControl( 10, 1, 0, 0 );
#if MASMW_DEBUG
		madebug_SetMode( 1 );
#endif
	}

	if ( seq_type == MASMW_SEQTYPE_DELAYED )
	{
		MaDevDrv_ClearFifo();
	}

	ma_pos_count[seq_type] = 0;

	ram_address = 0;
	for ( i = 0; i < 8; i++ )
	{
		if ( ( rb_map & (1 << i) ) != 0 )
		{
			ram_address = i * MA_RAM_BLOCK_SIZE + MA_RAM_START_ADDRESS + 16;
			break;
		}
	}

	if ( ( ram_adrs != NULL ) && ( ram_size != NULL ) && ( ram_address != 0 ) )
	{
		*ram_adrs = ram_address;
		*ram_size = ma_resmode_rbnum[rmode] * MA_RAM_BLOCK_SIZE - 16;
	}

	ma_snddrv_info[seq_type].ch_num = ma_resmode_chnum[rmode];
	ma_snddrv_info[seq_type].fm_num = (UINT8)( ma_resmode_fmnum[rmode] + fm_ext_num );
	ma_snddrv_info[seq_type].wt_num = (UINT8)( ma_resmode_wtnum[rmode] - sa_num );
	ma_snddrv_info[seq_type].rb_num = ma_resmode_rbnum[rmode];
	ma_snddrv_info[seq_type].sa_num = sa_num;

	ma_snddrv_info[seq_type].ch_map = ch_map;
	ma_snddrv_info[seq_type].fm_map = (fm_map | fm_ext_map);
	ma_snddrv_info[seq_type].wt_map = wt_map;
	ma_snddrv_info[seq_type].rb_map = rb_map;
	ma_snddrv_info[seq_type].sa_map = sa_map;
	ma_snddrv_info[seq_type].ld_map = ld_map;
	ma_snddrv_info[seq_type].mt_map = mt_map;
	ma_snddrv_info[seq_type].tm_map = tm_map;

	ma_snddrv_info[seq_type].drum_mode   = (UINT8)( cnv_mode       & 0x01);
	ma_snddrv_info[seq_type].dva_mode    = (UINT8)((cnv_mode >> 1) & 0x03);
	ma_snddrv_info[seq_type].melody_mode = (UINT8)((cnv_mode >> 3) & 0x01);
	ma_snddrv_info[seq_type].drum_type   = (UINT8)((cnv_mode >> 5) & 0x01);

	ma_snddrv_info[seq_type].time_base   = time_base;

	ma_snddrv_info[seq_type].key_offset		= (SINT8)0;
	ma_snddrv_info[seq_type].speed			= (SINT8)100;
	ma_snddrv_info[seq_type].master_volume	= (UINT8)100;
	ma_snddrv_info[seq_type].ctrl_volume	= (UINT8)100;
	ma_snddrv_info[seq_type].wave_slave		= (UINT8)0x80;

	for ( ch = 0; ch < MASMW_NUM_CHANNEL; ch++ )
	{
		if ( ( ch_map & (UINT32)(1 << ch) ) != 0 )
		{
			ma_channel_info[ch].key_control	 = (UINT8)0;
		}
	}

	for ( slot_no = 0; slot_no < ma_snddrv_info[seq_type].fm_num; slot_no++ )
	{
		if ( ( fm_map & (UINT32)(1 << slot_no) ) != 0 )
		{
			ma_slot_info[slot_no].ch   = 0;
		}
	}
	for ( slot_no = 0; slot_no < ma_snddrv_info[seq_type].wt_num; slot_no++ )
	{
		if ( ( wt_map & (UINT32)(1 << slot_no) ) != 0 )
		{
			ma_slot_info[slot_no+32].ch   = 0;
		}
	}
	for ( slot_no = 0; slot_no < ma_snddrv_info[seq_type].sa_num; slot_no++ )
	{
		if ( ( sa_map & (UINT32)(1 << slot_no) ) != 0 )
		{
			ma_slot_info[slot_no+40].ch   = 0;
		}
	}

	if ( seq_type != MASMW_SEQTYPE_AUDIO )
	{
		if ( rmode == 0 )
		{
			ma_snddrv_info[seq_type].min_slot[0] = (UINT8)0;
			ma_snddrv_info[seq_type].max_slot[0] = (UINT8)(0  + ma_snddrv_info[seq_type].fm_num - 1);
			ma_snddrv_info[seq_type].min_slot[1] = (UINT8)32;
			ma_snddrv_info[seq_type].max_slot[1] = (UINT8)(32 + ma_snddrv_info[seq_type].wt_num - 1);
		}
		else
		{
			ma_snddrv_info[seq_type].min_slot[0] = ma_resmode_minslot[seq_type][0][rmode];
			ma_snddrv_info[seq_type].max_slot[0] = ma_resmode_maxslot[seq_type][0][rmode];
			ma_snddrv_info[seq_type].min_slot[1] = ma_resmode_minslot[seq_type][1][rmode];
			ma_snddrv_info[seq_type].max_slot[1] = ma_resmode_maxslot[seq_type][1][rmode];
		}
	}

	if ( ma_snddrv_info[seq_type].sa_num != 0 )
	{
		ma_snddrv_info[seq_type].min_slot[2] = (UINT8)40;
		ma_snddrv_info[seq_type].max_slot[2] = (UINT8)(40 + ma_snddrv_info[seq_type].sa_num - 1);
	}

	for ( i = 0; i < 3; i++ )
	{
		ma_slot_list[seq_type].top[i] = ma_snddrv_info[seq_type].min_slot[i];
		ma_slot_list[seq_type].end[i] = ma_snddrv_info[seq_type].max_slot[i];
	}

	for ( i = 0; i < 42; i++ )
	{
		ma_slot_list[seq_type].next[i] = (UINT8)(i + 1);
	}

	if ( seq_type < MASMW_SEQTYPE_AUDIO )
	{
		for ( i = 0; i < MA_MAX_REG_VOICE*2; i++ )
		{
			 ma_voice_info[seq_type][i].ram_adrs = (UINT16)0xFFFF;
		}
	}

	for ( i = 0; i < MA_MAX_REG_STREAM; i++ )
	{
		ma_stream_info[seq_type][i].frequency = (UINT16)0;
	}
	
	ma_srm_cnv[seq_type] 	= srm_cnv;

	ma_control_sequencer[seq_type]	= 0;

	ma_snddrv_info[seq_type].status = 1;

	switch ( seq_type )
	{
	case MASMW_SEQTYPE_DELAYED:

		ma_seqbuf_info.sequence_flag = (UINT8)(MA_INT_POINT - 2);
		ma_seqbuf_info.buf_ptr		 = NULL;
		ma_seqbuf_info.buf_size 	 = 0;
		ma_seqbuf_info.queue_flag	 = 0;
		ma_seqbuf_info.queue_size	 = 0;
		ma_seqbuf_info.queue_wptr	 = 0;


		break;

	case MASMW_SEQTYPE_AUDIO:
	
		MaDevDrv_SetAudioMode( 1 );
		break;

	default:
		break;
	}

	if ( seq_type != MASMW_SEQTYPE_AUDIO )
	{
		for (ch_map = 0; ch_map < (MA_MAX_RAM_BANK * 2 * 128); ch_map++)
		{
			ma_voice_index[ch_map] = 0xff;
		}

		result = MaDevDrv_SendDirectRamData( (ram_address - 16), 0, &ma_woodblk[0], 16 );
		if ( result != MASMW_SUCCESS ) return result;

		result = MaSndDrv_SetVoice( seq_type, 0, 115, 1, 0, (ram_address - 16) );
		if ( result != MASMW_SUCCESS ) return result;
	}

	return (SINT32)seq_type;
}
/****************************************************************************
 *	MaSndDrv_UpdatePos
 *
 *	Description:
 *			Update the current play position.
 *	Argument:
 *			seq_id		sequence id
 *			ctrl		control flag
 *	Return:
 *			0			success
 *			< 0			error code
 *
 ****************************************************************************/
void MaSndDrv_UpdatePos
(
	SINT32	seq_id,						/* sequence id */
	UINT8	ctrl						/* control flag */
)
{
	if ( ctrl == 0 )
	{
		ma_pos_count[seq_id] = 0;
	}
	else
	{
		ma_pos_count[seq_id]++;
	}
}
/****************************************************************************
 *	MaSndDrv_DeviceControl
 *
 *	Description:
 *			Control the MA hardware.
 *	Argument:
 *			cmd		command number
 *			param1	parameter 1
 *			param2	parameter 2
 *			param3	parameter 3
 *	Return:
 *			0		success
 *			< 0		error code
 *
 ****************************************************************************/
SINT32 MaSndDrv_DeviceControl
(
	UINT8	cmd,						/* command number */
	UINT8	param1,						/* parameter 1 */
	UINT8	param2,						/* parameter 2 */
	UINT8	param3						/* parameter 3 */
)
{
	SINT32	result;
	UINT8	mode = 0;
	static UINT8 led_on[3]  = { 0x46, 0x82, 0x81 };
	static UINT8 led_off[3] = { 0x46, 0x82, 0x80 };
	static UINT8 mtr_on[3]  = { 0x47, 0x82, 0x81 };
	static UINT8 mtr_off[3] = { 0x47, 0x82, 0x80 };
	
	MASNDSEQ_DBGMSG(("MaSndDrv_DeviceControl: cmd=%d p1=%d p2=%d p3=%d\n", cmd, param1, param2, param3));

	if ( cmd <= MASMW_SP_VOLUME )
	{
		result = MaDevDrv_DeviceControl( cmd, param1, param2, param3 );
	}
	else if ( cmd <= MASMW_LED_DIRECT )
	{
		switch ( cmd )
		{
		case MASMW_LED_MASTER:
		
			if ( param1 > MASMW_MAX_LED_MASTER ) return MASMW_ERROR_ARGUMENT;

			ma_led_info.ctrl = param1;

			switch ( param1 )
			{
			case 0:		ma_led_info.dsw = 0;	break;		/* OFF */
			case 1:		ma_led_info.dsw = 0;	break;		/* direct */
			case 2:		
				ma_led_info.dsw = 1;						/* sequence */
				result = MaDevDrv_SendDirectPacket( led_off, 3 );
				break;
			}

			break;
	
		case MASMW_LED_BLINK:

			if ( param1 > MASMW_MAX_LED_BLINK ) return MASMW_ERROR_ARGUMENT;
	
			if ( param1 == 0 )
			{
				ma_led_info.blink = 4;
			}
			else
			{
				ma_led_info.blink = 5;
				ma_led_info.freq = (UINT8)( param1 - 1 );
			}
	
			break;
	
		case MASMW_LED_DIRECT:
	
			if ( param1 > MASMW_MAX_LED_DIRECT ) return MASMW_ERROR_ARGUMENT;
	
			if ( ma_led_info.ctrl == 1 )
			{
				ma_led_info.dsw = param1;
			}
	
			break;
		}
	
		switch ( ma_led_info.ctrl )
		{
		case 0:		mode = ma_led_info.ctrl;	break;		/* OFF */
		case 1:												/* direct */
		case 2:		mode = ma_led_info.blink;	break;		/* sequence */
		}

		result = MaDevDrv_DeviceControl( MADEVDRV_DCTRL_LED,
										  ma_led_info.dsw,			/* led */
										  ma_led_info.freq,		/* freq */
										  mode );					/* mode */
	
		if ( ma_led_info.ctrl == 1 )
		{
			result = MaDevDrv_SendDirectPacket( led_on, 3 );
		}
	}
	else if ( cmd <= MASMW_MTR_DIRECT )
	{
		switch ( cmd )
		{
		case MASMW_MTR_MASTER:
		
			if ( param1 > MASMW_MAX_MTR_MASTER ) return MASMW_ERROR_ARGUMENT;

			ma_mtr_info.ctrl = param1;

			switch ( param1 )
			{
			case 0:		ma_mtr_info.dsw = 0;	break;		/* OFF */
			case 1:		ma_mtr_info.dsw = 0;	break;		/* direct */
			case 2:		
				ma_mtr_info.dsw = 1;						/* sequence */
				result = MaDevDrv_SendDirectPacket( mtr_off, 3 );
				break;
			}

			break;
	
		case MASMW_MTR_BLINK:

			if ( param1 > MASMW_MAX_MTR_BLINK ) return MASMW_ERROR_ARGUMENT;
	
			if ( param1 == 0 )
			{
				ma_mtr_info.blink = 4;
			}
			else
			{
				ma_mtr_info.blink = 5;
				ma_mtr_info.freq = (UINT8)( param1 - 1 );
			}
	
			break;
	
		case MASMW_MTR_DIRECT:
	
			if ( param1 > MASMW_MAX_MTR_DIRECT ) return MASMW_ERROR_ARGUMENT;
	
			if ( ma_mtr_info.ctrl == 1 )
			{
				ma_mtr_info.dsw = param1;
			}

			break;
		}
	
		switch ( ma_mtr_info.ctrl )
		{
		case 0:		mode = ma_mtr_info.ctrl;	break;		/* OFF */
		case 1:												/* direct */
		case 2:		mode = ma_mtr_info.blink;	break;		/* sequence */
		}

		result = MaDevDrv_DeviceControl( MADEVDRV_DCTRL_MTR,
										  ma_mtr_info.dsw,
										  ma_mtr_info.freq,
										  mode );
	
		if ( ma_mtr_info.ctrl == 1 )
		{
			result = MaDevDrv_SendDirectPacket( mtr_on, 3 );
		}
	}
	else if ( cmd == MASMW_GET_PLLOUT )
	{
		result = (SINT32)MA_PLL_OUT;
	}
	else if ( cmd == MASMW_GET_SEQSTATUS )
	{
		result = MaDevDrv_DeviceControl( MADEVDRV_GET_SEQSTATUS, 0, 0, 0 );
	}
	else if ( cmd == MASMW_GET_LEDSTATUS )
	{
		result = ( ma_led_info.ctrl == 2 ) ? 1 : 0;
	}
	else if ( cmd == MASMW_GET_VIBSTATUS )
	{
		result = ( ma_mtr_info.ctrl == 2 ) ? 1 : 0;
	}
	else
	{
		result = MASMW_ERROR;
	}

	return result;
}
/****************************************************************************
 *	MaSndDrv_End
 *
 *	Description:
 *			End processing of the MA Sound Driver.
 *	Argument:
 *			None
 *	Return:
 *			0			success
 *			< 0			error code
 *
 ****************************************************************************/
SINT32 MaSndDrv_End(void)
{
	MASNDDRV_DBGMSG(("MaSndDrv_End:\n"));

	return MASMW_SUCCESS;
}
/****************************************************************************
 *	MaSndDrv_Initialize
 *
 *	Description:
 *			Initialize the MA Sound Driver
 *	Argument:
 *			None
 *	Return:
 *			0			success
 *			< 0			error code
 *
 ****************************************************************************/
SINT32 MaSndDrv_Initialize(void)
{
	UINT8	seq_id;						/* sequence id */
	UINT8	ch;							/* channel number */
	UINT16	i;							/* loop counter */

	MASNDDRV_DBGMSG(("MaSndDrv_Initialize:\n"));

	for ( seq_id = 0; seq_id < MASMW_NUM_SEQTYPE; seq_id++ )
	{
		ma_snddrv_info[seq_id].status			= (UINT8)0;
		ma_snddrv_info[seq_id].drum_mode		= (UINT8)0;
		ma_snddrv_info[seq_id].dva_mode			= (UINT8)0;
		ma_snddrv_info[seq_id].melody_mode		= (UINT8)0;
		ma_snddrv_info[seq_id].drum_type		= (UINT8)0;
		ma_snddrv_info[seq_id].ch_num			= (UINT8)0;
		ma_snddrv_info[seq_id].rb_num			= (UINT8)0;
		ma_snddrv_info[seq_id].speed			= (UINT8)100;
		ma_snddrv_info[seq_id].key_offset		= (SINT8)0;
		ma_snddrv_info[seq_id].master_volume 	= (UINT8)100;
		ma_snddrv_info[seq_id].ctrl_volume		= (UINT8)100;
		ma_snddrv_info[seq_id].wave_velocity	= (UINT8)0;
		ma_snddrv_info[seq_id].wave_slave		= (UINT8)0;
		ma_snddrv_info[seq_id].time_base		= (UINT8)0;
		ma_snddrv_info[seq_id].ch_map			= (UINT32)0x00000000;
		ma_snddrv_info[seq_id].fm_map			= (UINT32)0x00000000;
		ma_snddrv_info[seq_id].wt_map			= (UINT32)0x00000000;
		ma_snddrv_info[seq_id].sa_map			= (UINT32)0x00000000;
		ma_snddrv_info[seq_id].rb_map			= (UINT32)0x00000000;
		ma_snddrv_info[seq_id].ld_map			= (UINT32)0x00000000;
		ma_snddrv_info[seq_id].mt_map			= (UINT32)0x00000000;
		ma_snddrv_info[seq_id].tm_map			= (UINT32)0x00000000;

		for ( i = 0; i < 3; i++ )
		{
			ma_snddrv_info[seq_id].min_slot[i]  = (UINT8)0;
			ma_snddrv_info[seq_id].max_slot[i]  = (UINT8)0;
		}
	}

	for ( seq_id = 0; seq_id < MASMW_SEQTYPE_AUDIO; seq_id++ )
	{
		for ( i = 0; i < MA_MAX_REG_VOICE*2; i++ )
		{
			ma_voice_info[seq_id][i].ram_adrs = (UINT16)0xFFFF;
		}
	}

	for ( seq_id = 0; seq_id <= MASMW_SEQTYPE_AUDIO; seq_id++ )
	{
		for ( i = 0; i < MA_MAX_REG_STREAM; i++ )
		{
			ma_stream_info[seq_id][i].frequency = (UINT32)0;
		}
	}
	
	for ( ch = 0; ch < MASMW_NUM_CHANNEL; ch++ )
	{
		if ( seq_id == MASMW_SEQTYPE_DIRECT )
			ma_channel_info[ch].bank_no	 = (UINT8)( ( ch == 6 ) ? 128 : 0 );
		else
			ma_channel_info[ch].bank_no	 = (UINT8)( ( ch == 9 ) ? 128 : 0 );
		ma_channel_info[ch].prog_no 	 = (UINT8)0;
		ma_channel_info[ch].poly 		 = (UINT8)MA_MODE_POLY;		/* poly mode */
		ma_channel_info[ch].sfx 		 = (UINT8)0;				/* all off */
		ma_channel_info[ch].volume 		 = (UINT8)100;
		ma_channel_info[ch].expression 	 = (UINT8)127;
		ma_channel_info[ch].bend_range 	 = (UINT8)2;
		ma_channel_info[ch].key_control	 = (UINT8)0;
		ma_channel_info[ch].note_on 	 = (UINT8)0;
	}

	ma_led_info.ctrl  = 0;
	ma_led_info.dsw   = 0;
	ma_led_info.blink = 4;
	ma_led_info.freq  = 0;

	ma_mtr_info.ctrl  = 0;
	ma_mtr_info.dsw   = 0;
	ma_mtr_info.blink = 4;
	ma_mtr_info.freq  = 0;

	return MASMW_SUCCESS;
}
