/****************************************************************************
 *																			*
 *		Copyright (C) 2001-2003	YAMAHA CORPORATION. All rights reserved.	*
 *																			*
 *		Module		: masnddrv.h											*
 *																			*
 *		Description	: MA Sound Driver										*
 *																			*
 * 		Version		: 1.3.15.3	2003.03.17									*
 *																			*
 ****************************************************************************/


#ifndef __MASNDDRV_H__
#define __MASNDDRV_H__

#include "mamachdep.h"
#include "madebug.h"
#include "maresmgr.h"
#include "masndseq.h"
#include "madevdrv.h"

#define MASNDDRV_MAX_ID						(1)

#define MASMW_MAX_REG_SEQ					(2)
#define	MA_MAX_REG_VOICE					(128)
#define	MA_MAX_REG_STREAM					(32)
#define MASMW_MAX_SLOT						(32+8+2)
#define MA_MAX_RAM_BANK						(16)

#define MASNDDRV_CMD_NOTE_ON				(0)
#define MASNDDRV_CMD_NOTE_ON_MA2			(1)
#define MASNDDRV_CMD_NOTE_ON_MA2EX			(2)
#define MASNDDRV_CMD_NOTE_OFF				(3)
#define MASNDDRV_CMD_NOTE_OFF_MA2			(4)
#define MASNDDRV_CMD_NOTE_OFF_MA2EX			(5)
#define MASNDDRV_CMD_PROGRAM_CHANGE			(6)
#define MASNDDRV_CMD_MODULATION_DEPTH		(7)
#define MASNDDRV_CMD_CHANNEL_VOLUME			(8)
#define MASNDDRV_CMD_PANPOT					(9)
#define MASNDDRV_CMD_EXPRESSION				(10)
#define MASNDDRV_CMD_HOLD1					(11)
#define MASNDDRV_CMD_ALL_SOUND_OFF			(12)
#define MASNDDRV_CMD_RESET_ALL_CONTROLLERS	(13)
#define MASNDDRV_CMD_ALL_NOTE_OFF			(14)
#define MASNDDRV_CMD_MONO_MODE_ON			(15)
#define MASNDDRV_CMD_POLY_MODE_ON			(16)
#define MASNDDRV_CMD_PITCH_BEND				(17)
#define MASNDDRV_CMD_BEND_RANGE				(18)
#define MASNDDRV_CMD_STREAM_ON				(19)
#define MASNDDRV_CMD_STREAM_OFF				(20)
#define MASNDDRV_CMD_STREAM_SLAVE			(21)
#define MASNDDRV_CMD_STREAM_PANPOT			(22)
#define MASNDDRV_CMD_MASTER_VOLUME			(23)
#define MASNDDRV_CMD_SYSTEM_ON				(24)
#define MASNDDRV_CMD_LED_ON					(25)
#define MASNDDRV_CMD_LED_OFF				(26)
#define MASNDDRV_CMD_MOTOR_ON				(27)
#define MASNDDRV_CMD_MOTOR_OFF				(28)
#define MASNDDRV_CMD_USER_EVENT				(29)
#define MASNDDRV_CMD_NOP					(30)
#define MASNDDRV_CMD_STREAM_SEEK			(31)

#define MASMW_MAX_COMMAND					MASNDDRV_CMD_STREAM_SEEK

#define MASMW_NUM_CHANNEL					(16)		/* number of channel */
#define MASMW_MAX_CHANNEL					(15)		/* maximum value of channle */

#define MASMW_MAX_SEQTYPE					(2)			/* maximum value of sequence type */

#define MASMW_MAX_CNVMODE					(0x3F)		/* maximum value of cnv_mode */
#define MASMW_MAX_RESMODE					(16)		/* maximum value of res_mode */

#define MASMW_NUM_STREAM					(2)			/* number of stream audio */
#define MASMW_MAX_STREAM					(1)			/* maximum value of stream audio */

#define MASMW_NUM_VOICEINFO					(2)			/* number of voice information */

#define MASMW_SEQTYPE_DELAYED				(0)
#define MASMW_SEQTYPE_DIRECT				(1)
#define MASMW_SEQTYPE_AUDIO					(2)

#define MASMW_MIN_DLY_BASETIME				(1)
#define MASMW_MAX_DLY_BASETIME				(10)
#define MASMW_DIRECT_BASETIME				(20)
#define MASMW_AUDIO_BASETIME				(0)

#define MASMW_MAX_NOPTYPE					(1)			/* maximum value of nop type */
#define MASMW_MAX_USEREVENT					(127)		/* maximum value of user event */
#define MASMW_MAX_KEY						(127)		/* maximum value of key */
#define MASMW_MAX_VELOCITY					(127)		/* maximum value of velocity */
#define MASMW_MAX_BANK						(255)		/* maximum value of bank number */
#define MASMW_MAX_PROGRAM					(127)		/* maximum value of program number */
#define MASMW_MAX_DEPTH						(4)			/* maximum value of modulation depth */
#define MASMW_MAX_VOLUME					(127)		/* maximum value of volume */
#define MASMW_MAX_PAN						(127)		/* maximum value of panpot */
#define MASMW_MAX_HOLD1						(1)			/* maximum value of hold1 */
#define MASMW_MAX_BEND						(0x3FFF)	/* maximum value of pitch bend */
#define MASMW_MAX_BENDRANGE					(24)		/* maximum value of bend range */
#define MASMW_MAX_FORMAT					(3)			/* maximum value of format */
#define MASMW_MAX_STREAMPAN					(255)		/* maximum value of stream panpot */
#define MASMW_MIN_STREAMFS					(4000)		/* minimum value of stream frequency */
#define MASMW_MAX_STREAMFS					(48000)		/* maximum value of stream frequency */
#define MASMW_MIN_RAMADRS					(0x4000)	/* minimum value of ram address */
#define MASMW_MAX_RAMADRS					(0x5FFF)	/* maximum value of ram address */

#define MASMW_STMADPCM_ENDPOINT				(0x07C0)	/* steram audio ADPCM end point */
#define MASMW_STMPCM_ENDPOINT				(0x03DF)	/* stream audio PCM end point */

#define MASMW_MASK_CHANNEL					(0x0F)
#define MASMW_MASK_COMMAND					(0x1F)
#define MASMW_MASK_BANK						(0xFF)
#define MASMW_MASK_PROGRAM					(0x7F)
#define MASMW_MASK_VOLUME					(0x7F)
#define MASMW_MASK_XVB						(0xF8)
#define MASMW_MASK_SUS						(0xEF)
#define MASMW_MASK_BEND						(0x3FFF)
#define MASMW_MASK_DEPTH					(0x07)

#define MASMW_MAX_DB						(192)

#define MA_FS 								(48000)

#define MA_MIN_GML1_DRUM					(35)		/* minimum number of GM Level1 drum */
#define MA_MAX_GML1_DRUM					(81)		/* maximum number of GM Level1 drum */

/* macro definitions */

#define MAKE_TIMER_PART(seq_id, num, delta_time)							\
{																			\
	if ( delta_time >= 0 )													\
	{																		\
		packet_buf[seq_id][num++] = (UINT8)( 0x80 );						\
	}																		\
}

#define MAKE_ADDRESS_PART(seq_id, num, reg_index)							\
{																			\
	packet_buf[seq_id][num++] = (UINT8)(  reg_index       & 0x7F );			\
	packet_buf[seq_id][num++] = (UINT8)( (reg_index >> 7) | 0x80 );			\
}

#define MAKE_KEY_ON(seq_id, num, voice_id, vo_volume, pitch, ch)			\
{																			\
	packet_buf[seq_id][num++] = (UINT8)( (voice_id >> 7)  & 0x7F );			\
	packet_buf[seq_id][num++] = (UINT8)(  voice_id        & 0x7F );			\
	packet_buf[seq_id][num++] = (UINT8)(  vo_volume       & 0x7C );			\
	packet_buf[seq_id][num++] = (UINT8)( (pitch >> 7)     & 0x7F );			\
	packet_buf[seq_id][num++] = (UINT8)(  pitch           & 0x7F );			\
	packet_buf[seq_id][num++] = (UINT8)( (0x40 + ch)      | 0x80 );			\
}

#define MAKE_KEY_OFF(seq_id, num, ch)										\
{																			\
	packet_buf[seq_id][num++] = (UINT8)(  ch              | 0x80 );			\
}

#define MAKE_VEL_PITCH(seq_id, num, vo_volume, pitch)						\
{																			\
	packet_buf[seq_id][num++] = (UINT8)(  vo_volume       & 0x7C );			\
	packet_buf[seq_id][num++] = (UINT8)( (pitch >> 7)     & 0x7F );			\
	packet_buf[seq_id][num++] = (UINT8)(  pitch           | 0x80 );			\
}

#define MAKE_PITCH(seq_id, num, pitch)										\
{																			\
	packet_buf[seq_id][num++] = (UINT8)( (pitch >> 7)     & 0x7F );			\
	packet_buf[seq_id][num++] = (UINT8)(  pitch           | 0x80 );			\
}

#define MAKE_MUTE(seq_id, num, ch)											\
{																			\
	packet_buf[seq_id][num++] = (UINT8)( (0x20 + ch)  	  | 0x80 );			\
}

#define MAKE_RST(seq_id, num, ch)											\
{																			\
	packet_buf[seq_id][num++] = (UINT8)( (0x10 + ch)  	  | 0x80 );			\
}

#define MAKE_RST_MUTE(seq_id, num, ch)										\
{																			\
	packet_buf[seq_id][num++] = (UINT8)( (0x30 + ch)  	  | 0x80 );			\
}

#define MAKE_CHANNEL_VOLUME(seq_id, num, volume)							\
{																			\
	packet_buf[seq_id][num++] = (UINT8)( (volume & 0x7C)  | 0x80 );			\
}

#define MAKE_PANPOT(seq_id, num, pan)										\
{																			\
	packet_buf[seq_id][num++] = (UINT8)( (pan & 0x7C)     | 0x80 );			\
}

#define MAKE_SFX(seq_id, num, sfx)											\
{																			\
	packet_buf[seq_id][num++] = (UINT8)(  sfx             | 0x80 );			\
}

#define MAKE_STREAM_ON(seq_id, num, ram_adrs, vo_volume, pitch, ch)			\
{																			\
	packet_buf[seq_id][num++] = (UINT8)( (ram_adrs >> 7)  & 0x7F );			\
	packet_buf[seq_id][num++] = (UINT8)(  ram_adrs        & 0x7F );			\
	packet_buf[seq_id][num++] = (UINT8)(  vo_volume       & 0x7C );			\
	packet_buf[seq_id][num++] = (UINT8)( (pitch >> 7)     & 0x7F );			\
	packet_buf[seq_id][num++] = (UINT8)(  pitch           & 0x7F );			\
	packet_buf[seq_id][num++] = (UINT8)(  ch              | 0x80 );			\
}

#define MAKE_SOFTINT_RAM(seq_id, num, ram_val)								\
{																			\
	packet_buf[seq_id][num++] = (UINT8)(  ram_val         | 0x80 );			\
}

#define MAKE_SOFTINT(seq_id, num, int_val)									\
{																			\
	packet_buf[seq_id][num++] = (UINT8)(  int_val         | 0x80 );			\
}

#define MAKE_LED(seq_id, num, led)											\
{																			\
	packet_buf[seq_id][num++] = (UINT8)(  led             | 0x80 );			\
}

#define MAKE_MOTOR(seq_id, num, mtr)										\
{																			\
	packet_buf[seq_id][num++] = (UINT8)(  mtr             | 0x80 );			\
}

#define MAKE_PITCH_BEND(seq_id, num, bend)									\
{																			\
	packet_buf[seq_id][num++] = (UINT8)( (bend >> 7)      & 0x7F );			\
	packet_buf[seq_id][num++] = (UINT8)(  bend            | 0x80 );			\
}

#define MAKE_NOP(seq_id, num, reg_index)									\
{																			\
	packet_buf[seq_id][num++] = (UINT8)(  reg_index       & 0x7F );			\
	packet_buf[seq_id][num++] = (UINT8)( (reg_index >> 7) | 0x80 );			\
	packet_buf[seq_id][num++] = (UINT8)(  0x00            | 0x80 );			\
}

/* struct definitions */

typedef struct _MACHINFO
{
	UINT8	bank_no;					/* bank number */
	UINT8	prog_no;					/* program number */
	UINT8	poly;						/* poly/mono */
	UINT8	sfx;						/* sfx */
	UINT8	volume;						/* volume */
	UINT8	expression;					/* expression */
	UINT8	bend_range;					/* bend range */
	UINT8	key_control;				/* key control */
	UINT8	note_on;					/* note on */
	UINT8	slot_no;					/* slot number */
} MACHINFO;

typedef struct _MASLOTINFO
{
	UINT8	ch;							/* channel number */
	UINT32	key;						/* key number */
	UINT32	time;						/* time */
	UINT32	tick;						/* tick */
} MASLOTINFO;

typedef struct _MAVOICEINFO
{
	UINT16	bank_prog;					/* bank, program number */
	UINT8	synth;						/* synth type */
	UINT16	key;						/* key number */
	UINT16	ram_adrs;					/* ram address */
} MAVOICEINFO, * PMAVOICEINFO;

typedef struct _MAVINFO
{
	UINT8	synth;						/* synth type */
	UINT16	key;						/* key number */
	UINT16	address;					/* address */
} MAVINFO, * PMAVINFO;

typedef struct _MASTREAMINFO
{
	UINT8	format;						/* format */
	UINT8	pan;						/* pan */
	UINT32	frequency;					/* frequency */
} MASTREAMINFO, * PMASTREAMINFO;

typedef struct _MALEDINFO
{
	UINT8	ctrl;						/* control */
	UINT8	dsw;						/* direct sw */
	UINT8	blink;						/* blink */
	UINT8	freq;						/* frequency */
} MALEDINFO, *PMALEDINFO;

typedef struct _MAMTRINFO
{
	UINT8	ctrl;						/* control */
	UINT8	dsw;						/* direct sw */
	UINT8	blink;						/* blink */
	UINT8	freq;						/* frequency */
} MAMTRINFO, *PMAMTRINFO;

typedef struct _MA_SEQBUFINFO
{
	UINT8	sequence_flag;				/* sequence flag */
	UINT8 *	buf_ptr;					/* pointer to the buffer */
	UINT32	buf_size;					/* size of the buffer */
	UINT8	queue_flag;					/* queue flag */
	UINT32	queue_size;					/* size of data in queue */
	UINT32	queue_wptr;					/* write pointer */
	UINT32	wrote_size;					/* size of wrote data */
	UINT8	queue[2048];				/* queue */
} MA_SEQBUFINFO, *PMA_SEQBUFINFO;


typedef struct _MASNDDRVINFO
{
	UINT8	status;						/* status */
	UINT8	drum_mode;					/* drum mode */
	UINT8	dva_mode;					/* dva mode */
	UINT8	melody_mode;				/* melody mode */
	UINT8	drum_type;					/* drum type */
	UINT8	ch_num;						/* number of channel */
	UINT8	fm_num;						/* number of FM slot */
	UINT8	wt_num;						/* number of WT slot */
	UINT8	sa_num;						/* number of stream audio slot */
	UINT8	rb_num;						/* number of ram block */
	UINT32	speed;						/* speed */
	SINT8	key_offset;					/* key offset for key control */
	UINT8	master_volume;				/* master volume */
	UINT8	ctrl_volume;				/* control volume */
	UINT8	wave_velocity;				/* wave velocity */
	UINT8	wave_slave;					/* wave slave number */
	UINT8	time_base;					/* time base */
	UINT32	ch_map;						/* bit mapping of channel */
	UINT32	fm_map;						/* bit mapping of FM slot */
	UINT32	wt_map;						/* bit mapping of WT slot */
	UINT32	sa_map;						/* bit mapping of stream audio slot */
	UINT32	rb_map;						/* bit mapping of ram block */
	UINT32	ld_map;						/* bit mapping of led */
	UINT32	mt_map;						/* bit mapping of motor */
	UINT32	tm_map;						/* bit mapping of timer */
	UINT8	min_slot[3];				/* minimum slot number */
	UINT8	max_slot[3];				/* maximum slot number */
} MASNDDRVINFO;

typedef struct _MASLOTLIST
{
	UINT8	top[3];						/* top */
	UINT8	end[3];						/* end */
	UINT8	next[42];					/* next */
} MASLOTLIST, *PMASLOTLIST;

typedef struct _MASTREAMPARAM
{
	UINT8	panpot;						/* panpot */
	UINT8	stm;						/* stream */
	UINT8	pe;							/* pan enable */
	UINT8	panoff;						/* pan off */
	UINT8	mode;						/* mode */
	UINT16	end_point;					/* end point */
	UINT8	sa_id;						/* stream audio id */
	UINT8	vo_volume;					/* voice volume */
	UINT16	pitch;						/* pitch */
} MASTREAMPARAM, *PMASTREAMPARAM;

typedef struct _MASMW_CONTENTSINFO
{
	UINT16	code;						/* code type */
	UINT8	tag[2];						/* tag name */
	UINT8*	buf;						/* pointer to read buffer */
	UINT32	size;						/* size of read buffer */
} MASMW_CONTENTSINFO, *PMASMW_CONTENTSINFO;

typedef struct _MASMW_PHRASELIST
{
	UINT8	tag[2];						/* tag name */
	UINT32	start;						/* start point */
	UINT32	stop;						/* stop point */
} MASMW_PHRASELIST, *PMASMW_PHRASELIST;

typedef struct _MASMW_EVENTNOTE
{
	UINT8	ch;							/* channel number */
	UINT8	note;						/* note number */
} MASMW_EVENTNOTE, *PMASMW_EVENTNOTE;

SINT32 MaSndDrv_Initialize		( void );
SINT32 MaSndDrv_End				( void );
SINT32 MaSndDrv_Create			( UINT8 seq_type, UINT8 time_base, UINT8 cnv_mode, UINT8 res_mode, UINT8 sa_num, SINT32 (*func)(void), UINT32 * ram_adrs, UINT32 * ram_size );
SINT32 MaSndDrv_Free			( SINT32 seq_id );
SINT32 MaSndDrv_GetTimeError	( SINT32 seq_id );
SINT32 MaSndDrv_GetPos			( SINT32 seq_id );
SINT32 MaSndDrv_SetSpeed		( SINT32 seq_id, UINT8 val );
SINT32 MaSndDrv_SetSpeedWide	( SINT32 seq_id, UINT32 val );
SINT32 MaSndDrv_SetKey			( SINT32 seq_id, SINT8 val );
SINT32 MaSndDrv_SetKeyControl	( SINT32 seq_id, UINT8 ch, UINT8 val );
SINT32 MaSndDrv_SetFmExtWave	( SINT32 seq_id, UINT8 wave_no, UINT32 ram_adrs );
SINT32 MaSndDrv_SetVolume		( SINT32 seq_id, UINT8 volume );
SINT32 MaSndDrv_SetVoice		( SINT32 seq_id, UINT8 bank, UINT8 prog, UINT8 synth, UINT32 key, UINT32 address );
SINT32 MaSndDrv_GetVoice		( SINT32 seq_id, UINT8 bank, UINT8 prog );
SINT32 MaSndDrv_SetStream		( SINT32 seq_id, UINT8 wave_id, UINT8 format, UINT32 frequency, UINT8 * wave_ptr, UINT32 wave_size );
SINT32 MaSndDrv_SetCommand		( SINT32 seq_id, SINT32 delta_time, UINT32 cmd, UINT32 param1, UINT32 param2, UINT32 param3 );

SINT32 MaSndDrv_ControlSequencer( SINT32 id, UINT8 ctrl );

SINT32 MaSndDrv_DeviceControl	( UINT8 cmd, UINT8 param1, UINT8 param2, UINT8 param3 );

SINT32 MaSndDrv_UpdateSequence	( SINT32 id, UINT8 * buf_ptr, UINT32 buf_size );
void   MaSndDrv_UpdatePos		( SINT32 id, UINT8 ctrl );


#endif /*__MASNDDRV_H__*/
