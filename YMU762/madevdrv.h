/****************************************************************************
 *																			*
 *		Copyright (C) 2001-2003	YAMAHA CORPORATION. All rights reserved.	*
 *																			*
 *		Module		: madevdrv.h											*
 *																			*
 *		Description	: MA Device Driver										*
 *																			*
 * 		Version		: 1.3.15.2	2003.03.13									*
 *																			*
 ****************************************************************************/

#ifndef __MADEVDRV_H__
#define __MADEVDRV_H__

#include "mamachdep.h"
#include "madefs.h"
#include "madebug.h"
#include "masndseq.h"
#include "masnddrv.h"

/* Struct Definitions */

typedef struct _MA_SBUF_INFO
{
	UINT8	write_num;
	UINT8	read_num;
	UINT16	buf_total;
	UINT16	buf_ptr;
	UINT16	buf_size[MA_SBUF_NUM];
} MA_SBUF_INFO;

typedef struct _MA_STREAM_AUDIO_INFO
{
	UINT8	state[MA_MAX_STREAM_AUDIO];
	UINT16	write_block[MA_MAX_STREAM_AUDIO];
	UINT16	read_block[MA_MAX_STREAM_AUDIO];
	UINT8	format[MA_MAX_STREAM_AUDIO];
	UINT8 *	wave_ptr[MA_MAX_STREAM_AUDIO];
	UINT32	wave_size[MA_MAX_STREAM_AUDIO];
	UINT32	position[MA_MAX_STREAM_AUDIO];
	UINT16	end_point[MA_MAX_STREAM_AUDIO];
	UINT16	prv_point[MA_MAX_STREAM_AUDIO];
} MA_STREAM_AUDIO_INFO, *PMA_STREAM_AUDIO_INFO;

typedef struct _MADEVDRVINFO
{
	void (* int_func[8])(UINT8 ctrl);
	UINT8	seq_flag;
	UINT8	stop_reg;
	UINT8	stop_flag;
	UINT8	ctrl_seq;
	UINT8	timer0;
	UINT8	mask_interrupt;
	UINT8	int_func_map;
	UINT8	audio_mode;
	UINT8	end_of_sequence[3];
	UINT8	sbuf_buffer[MA_SBUF_NUM][MA_FIFO_SIZE];
	MA_SBUF_INFO			sbuf_info;
	MA_STREAM_AUDIO_INFO	streaminfo;
} MADEVDRVINFO, *PMADEVDRVINFO;

/* Function Definitions */

SINT32	MaDevDrv_Initialize			( void );

UINT8 * MaDevDrv_GetSeekBuffer		( UINT16 * size );
SINT32	MaDevDrv_SeekControl		( SINT32 seq_id, UINT32 size );

void	MaDevDrv_SetAudioMode		( UINT8 mode );

UINT32	MaDevDrv_GetStreamPos		( UINT8 ctrl );

void	MaDevDrv_InitRegisters		( void );
SINT32	MaDevDrv_VerifyRegisters	( void );
SINT32	MaDevDrv_PowerManagement	( UINT8 sw );

SINT32	MaDevDrv_DeviceControl		( UINT8 cmd, UINT8 prm1, UINT8 prm2, UINT8 prm3 );

SINT32	MaDevDrv_StartSequencer		( SINT32 seq_id, UINT8 ctrl );
SINT32	MaDevDrv_StopSequencer		( SINT32 seq_id, UINT8 ctrl );

SINT32	MaDevDrv_EndOfSequence		( void );

SINT32	MaDevDrv_ClearFifo			( void );

void	MaDevDrv_SoftInt0			( UINT8	ctrl );
void	MaDevDrv_SoftInt1			( UINT8 ctrl );
void	MaDevDrv_SoftInt2			( UINT8 ctrl );
void	MaDevDrv_Timer0				( UINT8 ctrl );
void	MaDevDrv_Timer1				( UINT8 ctrl );
void	MaDevDrv_Fifo				( UINT8 ctrl );

SINT32	MaDevDrv_StreamHandler		( UINT8 sa_id, UINT8 ctrl, UINT8 ram_val );

SINT32	MaDevDrv_AddIntFunc			( UINT8 number, void (*func)( UINT8 ctrl ) );
SINT32	MaDevDrv_RemoveIntFunc			( UINT8 number );
void	MaDevDrv_IntHandler			( void );

SINT32	MaDevDrv_SendDirectPacket	( const UINT8 * ptr, UINT32 size );
SINT32 	MaDevDrv_ReceiveData		( UINT32 adrs, UINT8 buf_adrs );
SINT32	MaDevDrv_SendDelayedPacket	( const UINT8 * ptr, UINT32 size );
SINT32	MaDevDrv_SendDirectRamData	( UINT32 adrs, UINT8 type, const UINT8 * ptr, UINT32 size );
SINT32	MaDevDrv_SendDirectRamVal	( UINT32 adrs, UINT32 size, UINT8 val );

void	MaDevDrv_WriteStatusFlagReg	( UINT8 data );
UINT8 	MaDevDrv_ReadStatusFlagReg	( void );
void	MaDevDrv_WriteDataReg		( UINT8 data );
UINT8	MaDevDrv_ReadDataReg		( void );

#endif /*__MADEVDRV_H__*/
