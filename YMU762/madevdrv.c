/****************************************************************************
 *																			*
 *		Copyright (C) 2001-2003	YAMAHA CORPORATION. All rights reserved.	*
 *																			*
 *		Module		: madevdrv.c											*
 *																			*
 *		Description	: MA Device Driver										*
 *																			*
 * 		Version		: 1.3.15.2	2003.03.13									*
 *																			*
 ****************************************************************************/

#include "madevdrv.h"

UINT8 _ma_intstate;

MADEVDRVINFO ma_devdrv_info;
static PMADEVDRVINFO cinfo_ptr = &ma_devdrv_info;

// Not in the M5_EmuHw.dll
extern void 	machdep_Wait					( UINT32 wait_time );       // delay [ms]
extern SINT32	machdep_CheckStatus				( UINT8 flag );             // repackaged machdep_ReadStatusFlagReg()
extern SINT32	machdep_CheckDelayedFifoEmpty	( void );                   // calls machdep_ReadStatusFlagReg() few times (and no other machdep func)

// M5_EmuHw.dll has "Hw_" prefix instead of "machdep_"... :)
extern void 	machdep_WriteStatusFlagReg		( UINT8 data );
extern UINT8	machdep_ReadStatusFlagReg		( void );
extern void		machdep_WriteDataReg			( UINT8 data );
extern UINT8	machdep_ReadDataReg				( void );
extern SINT32	machdep_WaitValidData			( UINT8 flag );
extern SINT32	machdep_WaitDelayedFifoEmpty 	( void );
extern SINT32	machdep_WaitImmediateFifoEmpty	( void );





/****************************************************************************
	 dummy Description
 ****************************************************************************/
static void dummy_IntFunc( UINT8 ctrl )
{
	(void)ctrl;

	MADEVDRV_DBGMSG((" dummy_IntFunc: ctrl=%d\n", ctrl));
}
/****************************************************************************
 *	MaDevDrv_GetSeekBuffer
 *
 *	Description:
 *			Return pointer to buffer and size of buffer for setting control
 *			values when seek.
 *	Arguments:
 *			*size		pointer to buffer size
 *	Return:
 *			pointer to buffer
 *
 ****************************************************************************/
UINT8 * MaDevDrv_GetSeekBuffer( UINT16 * size )
{
	*size = MA_FIFO_SIZE * MA_SBUF_NUM;

	return cinfo_ptr->sbuf_buffer[0];
}
/****************************************************************************
 *	MaDevDrv_SeekControl
 *
 *	Description:
 *			Send control values when seek.
 *	Arguments:
 *			seq_id		sequence id
 *			buf_size	buffer byte size
 *	Return:
 *			0			success
 *			< 0			error code
 *
 ****************************************************************************/
SINT32 MaDevDrv_SeekControl
(
	SINT32	seq_id,						/* sequence id */
	UINT32	buf_size					/* byte size of buffer */
)
{
	UINT8 *	buf_ptr;					/* pointer to buffer */
	SINT32	result;						/* result of function */
	static UINT8	packet0[3] = { 0x61, 0x82, 0x81 };	/* start sequencer */
	static UINT8	packet1[3] = { 0x61, 0x82, 0x80 };	/* stop sequencer */

	/* delayed sequence only */
	if ( seq_id != MASMW_SEQTYPE_DELAYED )
	{
		return MASMW_ERROR;
	}

	/* write control data to H/W FIFO for seek */
	buf_ptr = cinfo_ptr->sbuf_buffer[0];
	if ( buf_size != 0 )
	{
		MaDevDrv_SendDelayedPacket( buf_ptr, buf_size );
	}

	/* disable smooth volume setting */
	MaDevDrv_DeviceControl( 8, 0x00, 0x00, 0x00 );

	/* start the MA-3 H/W sequencer */
	result = MaDevDrv_SendDirectPacket( packet0, 3 );

	/* waiting for FIFO empty */
	result = machdep_WaitDelayedFifoEmpty();

	/* stop the MA-3 H/W sequencer */
	MaDevDrv_SendDirectPacket( packet1, 3 );

	/* enable smooth volume setting */
	MaDevDrv_DeviceControl( 8, 0x03, 0x03, 0x03 );

	return result;
}
/****************************************************************************
 *	MaDevDrv_GetStreamPos
 *
 *	Description:
 *			Get position of stream audio.
 *	Arguments:
 *			ctrl	0: get current position
 *					1: reset position count
 *	Return:
 *			position of stream audio [ms]
 *
 ****************************************************************************/
UINT32 MaDevDrv_GetStreamPos
(
	UINT8 ctrl
)
{
	UINT32	result = 0;					/* result of function */

	switch ( ctrl )
	{
	case 0:
		result = (UINT32)( cinfo_ptr->streaminfo.position[0] * 20 );
		break;

	case 1:
		cinfo_ptr->streaminfo.position[0] = 0;
		break;

	default:
		break;
	}

	return result;
}
/****************************************************************************
 *	MaDevDrv_SetAudioMode
 *
 *	Description:
 *			Set the flag of audio sequence.
 *	Arguments:
 *			mode	0: none audio sequence
 *					1: audio sequence
 *	Return:
 *			None
 *
 ****************************************************************************/
void MaDevDrv_SetAudioMode( UINT8 mode )
{
	cinfo_ptr->audio_mode = mode;
}
/****************************************************************************
 *	MaDevDrv_ControlInterrupt
 *
 *	Description:
 *			Enable or disable the interrupt.
 *	Arguments:
 *			ctrl		control flag (0: enable, 1: disable)
 *			int_ctrl	interrupt number
 *	Return:
 *			None
 *
 ****************************************************************************/
void MaDevDrv_ControlInterrupt
(
	UINT8	ctrl,
	UINT8	int_ctrl
)
{
	UINT8	data;
	UINT8	packet[3];

	/* interrupt disable */

	data = (UINT8)MaDevDrv_ReceiveData( MA_INT_SETTING, 1 );

	if ( ctrl == 0 )		/* enable */
	{
		data |= int_ctrl;
	}
	else					/* diable */
	{
		data &= (UINT8)( ~int_ctrl );
	}

	packet[0] = (UINT8)(  MA_INT_SETTING       & 0x7F );
	packet[1] = (UINT8)( (MA_INT_SETTING >> 7) | 0x80 );
	packet[2] = (UINT8)(  data                  | 0x80 );
	MaDevDrv_SendDirectPacket( packet, 3 );

	/* interrupt enable */
}
/****************************************************************************
 *	MaDevDrv_AddIntFunc
 *
 *	Description:
 *			Add the interrupt function of hardware interrupt.
 *	Arguments:
 *			number		interrupt number
 *						  0 SIRQ#0	software interrupt #0
 *						  1 SIRQ#1	software interrupt #1
 *						  2 SIRQ#2	software interrupt #2
 *						  3 SIRQ#3	software interrupt #3
 *						  4
 *						  5 TM#0	timer #0
 *						  6 TM#1	tiemr #1
 *						  7 FIFO	FIFO
 *			int_func	pointer to the interrupt function
 *	Return:
 *
 ****************************************************************************/
SINT32 MaDevDrv_AddIntFunc
(
	UINT8	number,
	void	(* int_func)(UINT8 ctrl)
)
{
	MADEVDRV_DBGMSG(("    MaDevDrv_AddIntFunc: %d\n", number));

	cinfo_ptr->int_func[number] = int_func;
	cinfo_ptr->int_func_map |= (UINT8)1 << number;

	return MASMW_SUCCESS;
}
/****************************************************************************
 *	MaDevDrv_RemoveIntFunc
 *
 *	Description:
 *			Remove the specified interrupt function.
 *	Argument:
 *			number		function number
 *	Return:
 *			0			success
 *			< 0			error
 *
 ****************************************************************************/
SINT32 MaDevDrv_RemoveIntFunc
(
	UINT8		number
)
{
	MADEVDRV_DBGMSG(("    MaDevDrv_RemoveIntFunc: %d\n", number));

	cinfo_ptr->int_func[number] = dummy_IntFunc;
	cinfo_ptr->int_func_map &= ~(UINT8)1 << number;

	return MASMW_SUCCESS;
}
/****************************************************************************
 *	MaDevDrv_IntHandler
 *
 *	Description:
 *			Interrupt handler for hardware interrupts.
 *	Argument:
 *			None
 *	Return:
 *			None
 *
 ****************************************************************************/
void MaDevDrv_IntHandler( void )
{
	UINT32	i;
	UINT8	read_data;
	UINT8	int_num;

	static UINT8	ma_int_priority[8] = { 0, 1, 6, 7, 2, 3, 5, 4 };

	MADEVDRV_DBGMSG(("    MaDevDrv_IntHandler\n"));

	machdep_WriteStatusFlagReg( (UINT8)MA_INTERRUPT_FLAG_REG );

	read_data = machdep_ReadDataReg();

	cinfo_ptr->mask_interrupt = 0x00;

	for ( i = 0; i < 8; i++ )
	{
		int_num = ma_int_priority[i];

		if ( ( read_data & (1 << int_num) ) != 0 )
		{
			if ( ( cinfo_ptr->int_func_map & (1 << int_num) ) != 0 )
			{
				if ( int_num != 7 )
				{
					machdep_WriteStatusFlagReg( (UINT8)MA_INTERRUPT_FLAG_REG );

					machdep_WriteDataReg( (UINT8)( 1 << int_num ) );
				}

				cinfo_ptr->int_func[int_num](0);
				if ( int_num == 7 )
				{
					machdep_WriteStatusFlagReg( (UINT8)MA_INTERRUPT_FLAG_REG );

					machdep_WriteDataReg( (UINT8)( 1 << int_num ) );
				}
			}
		}
	}

	cinfo_ptr->mask_interrupt = 0x80;
	machdep_WriteStatusFlagReg( cinfo_ptr->mask_interrupt );
}
/****************************************************************************
 *	MaDevDrv_SendDelayedPacket
 *
 *	Description:
 *			Write delayed packets to the REG_ID #1 delayed write register.
 *	Argument:
 *			ptr			pointer to the delayed packets
 *			size		size of the delayed packets
 *	Return:
 *			MASMW_SUCCESS(0)
 *
 ****************************************************************************/
SINT32 MaDevDrv_SendDelayedPacket
(
	const UINT8 * ptr,					/* pointer to packets */
	UINT32	      size					/* byte size of packets */
)
{
#if MASMW_DEBUG
	SINT32	result;

	result = madebug_SendDelayedPacket( ptr, (UINT16)size );
	if ( result < MASMW_SUCCESS ) return result;
#endif

	MADEVDRV_DBGMSG(("    MaDevDrv_SendDelayedPacket: pt=%p, sz=%d\n", ptr, size));

	machdep_WriteStatusFlagReg( MA_DELAYED_WRITE_REG );

#if 1
	while ( (size--) != 0 )
#else
	for ( i = 0; i < size; i++ )
#endif
	{
		machdep_WriteDataReg( *ptr++ );
	}

	machdep_WriteStatusFlagReg( cinfo_ptr->mask_interrupt );

	return MASMW_SUCCESS;
}
/****************************************************************************
 *	MaDevDrv_SendDirectPacket
 *
 *	Description:
 *			Write direct packets to the REG_ID #2 direct write register.
 *	Argument:
 *			ptr			pointer to the direct packets
 *			size		size of the direct packets
 *	Return:
 *			MASMW_SUCCESS(0)
 *
 ****************************************************************************/
SINT32 MaDevDrv_SendDirectPacket
(
	const UINT8 * ptr,					/* pointer to packets */
	UINT32 size							/* byte size of packets */
)
{
	UINT32	i, j;
	SINT32	result;

#if MASMW_DEBUG
	result = madebug_SendDirectPacket( ptr, (UINT16)size );
	if ( result < MASMW_SUCCESS ) return result;
#endif

	MADEVDRV_DBGMSG(("    MaDevDrv_SendDirectPacket: pt=%p, sz=%d\n", ptr, size));

	machdep_WriteStatusFlagReg( MA_IMMEDIATE_WRITE_REG );

#if 1
	i = size/64;
	while ( (i--) != 0 )
#else
	for ( i = 0; i < (size/64); i++ )
#endif
	{
		result = machdep_WaitImmediateFifoEmpty();
		if ( result < MASMW_SUCCESS )
		{
			machdep_WriteStatusFlagReg( cinfo_ptr->mask_interrupt );
			return result;
		}
#if 1
		j = 64;
		while ( (j--) != 0 )
#else
		for ( j = 0; j < 64; j++ )
#endif
		{
			machdep_WriteDataReg( *ptr++ );
		}
	}

	if ( (size %64) != 0 )
	{
		result = machdep_WaitImmediateFifoEmpty();
		if ( result < MASMW_SUCCESS )
		{
			machdep_WriteStatusFlagReg( cinfo_ptr->mask_interrupt );
			return result;
		}

#if 1
		j = size%64;
		while ( (j--) != 0 )
#else
		for ( j = 0; j < (size%64); j++ )
#endif
		{
			machdep_WriteDataReg( *ptr++ );
		}
	}

	machdep_WriteStatusFlagReg( cinfo_ptr->mask_interrupt );

	return MASMW_SUCCESS;
}
/****************************************************************************
 *	MaDevDrv_ReceiveData
 *
 *	Description:
 *			Read byte data of register or memory.
 *	Argument:
 *			address			address of the register for read
 *			buffer_address	address of the return buffer
 *	Return:
 *			MASMW_SUCCESS(0)
 *
 ****************************************************************************/
SINT32 MaDevDrv_ReceiveData
(
	UINT32	address,			/* address of register */
	UINT8	buffer_address		/* address of read buffer */
)
{
	UINT8	i;
	UINT8	packet_buffer[4];
	UINT8	valid_rx;
	UINT8	read_data;
	UINT8	count;
	SINT32	result;

	MADEVDRV_DBGMSG(("    MaDevDrv_ReceiveData: adrs=%ld, bufadrs=%d\n", address, buffer_address));

	machdep_WriteStatusFlagReg( MA_IMMEDIATE_READ_REG );


	if		( address < 0x0080 )	count = 0;
	else if ( address < 0x4000 )	count = 1;
	else							count = 2;

	for ( i = 0; i < count; i++ )
	{
		packet_buffer[i] = (UINT8)( (address >> (7*i)) & 0x7F );
	}
	packet_buffer[i] = (UINT8)( (address >> (7*i)) | 0x80 );

	count++;
	packet_buffer[count] = (UINT8)( 0x80 | buffer_address );

	result = machdep_WaitImmediateFifoEmpty();
	if ( result < MASMW_SUCCESS )
	{
		machdep_WriteStatusFlagReg( cinfo_ptr->mask_interrupt );
		return result;
	}

	for ( i = 0; i <= count; i++ )
	{
		machdep_WriteDataReg( packet_buffer[i] );
	}

	valid_rx = (UINT8)(1 << (MA_VALID_RX + buffer_address));

	result = machdep_WaitValidData( valid_rx );
	if ( result < MASMW_SUCCESS )
	{
		machdep_WriteStatusFlagReg( cinfo_ptr->mask_interrupt );
		return result;
	}

	machdep_WriteStatusFlagReg( (UINT8)(valid_rx | (buffer_address + 1)) );

	read_data = machdep_ReadDataReg();

	machdep_WriteStatusFlagReg( cinfo_ptr->mask_interrupt );

	return (SINT32)read_data;
}
/****************************************************************************
 *	MaDevDrv_SendDirectRamData
 *
 *	Description:
 *			write data to internal RAM.
 *	Arguments:
 *			address			internal RAM address
 *			data_type		type of data
 *			data_ptr		pointer ot the write byte data to internal RAM
 *			data_size		size of write byte data to internal RAM
 *	Return:
 *			MASMW_SUCCESS(0)
 *
 ****************************************************************************/
SINT32 MaDevDrv_SendDirectRamData
(
	UINT32	address,					/* address of internal ram */
	UINT8	data_type,					/* type of data */
	const UINT8 * data_ptr,				/* pointer to data */
	UINT32	data_size					/* size of data */
)
{
	UINT32	i, j;
	UINT8	packet_buffer[3+2];
	UINT32	count;
	UINT32	write_size;
	UINT8	write_data;
	UINT8	temp = 0;
	SINT32	result;

#if MASMW_DEBUG
	result = madebug_SendDirectRamData( address, data_type, data_ptr, data_size );
	if ( result < MASMW_SUCCESS ) return result;
#endif

	MADEVDRV_DBGMSG(("    MaDevDrv_SendDirectRamData: adrs=%04lX type=%d ptr=%p size=%ld\n", address, data_type, data_ptr, data_size));

	if ( data_size == 0 ) return MASMW_SUCCESS;

	machdep_WriteStatusFlagReg( MA_IMMEDIATE_WRITE_REG );


	packet_buffer[0] = (UINT8)( (address >>  0 ) & 0x7F );
	packet_buffer[1] = (UINT8)( (address >>  7 ) & 0x7F );
	packet_buffer[2] = (UINT8)( (address >> 14 ) | 0x80 );

	if ( data_type == 0 )
	{
		write_size = data_size;
	}
	else
	{
		write_size = (data_size / 8) * 7;
		if ( data_size % 8 != 0 )
			write_size = (UINT32)( write_size + (data_size % 8 - 1) );
	}

	if	( write_size < 0x0080 )
	{
		packet_buffer[3] = (UINT8)( (write_size >> 0) | 0x80 );
		count = 4;
	}
	else
	{
		packet_buffer[3] = (UINT8)( (write_size >> 0) & 0x7F );
		packet_buffer[4] = (UINT8)( (write_size >> 7) | 0x80 );
		count = 5;
	}

	result = machdep_WaitImmediateFifoEmpty();
	if ( result < MASMW_SUCCESS )
	{
		machdep_WriteStatusFlagReg( cinfo_ptr->mask_interrupt );
		return result;
	}

	for ( i = 0; i < count; i++ )
	{
		machdep_WriteDataReg( packet_buffer[i] );
	}

	if ( data_type == 0 )
	{
#if 1
		i = data_size/64;
		while ( (i--) != 0 )
#else
		for ( i = 0; i < data_size/64; i++ )
#endif
		{
			result = machdep_WaitImmediateFifoEmpty();
			if ( result < MASMW_SUCCESS )
			{
				machdep_WriteStatusFlagReg( cinfo_ptr->mask_interrupt );
				return result;
			}
#if 1
			j = 64;
			while ( (j--) != 0 )
#else
			for ( j = 0; j < 64; j++ )
#endif
			{
				machdep_WriteDataReg( *data_ptr++ );
			}
		}

		if ( (data_size%64) != 0 )
		{
			result = machdep_WaitImmediateFifoEmpty();
			if ( result < MASMW_SUCCESS )
			{
				machdep_WriteStatusFlagReg( cinfo_ptr->mask_interrupt );
				return result;
			}
#if 1
			j = data_size%64;
			while ( (j--) != 0 )
#else
			for ( j = 0; j < (data_size%64); j++ )
#endif
			{
				machdep_WriteDataReg( *data_ptr++ );
			}
		}
	}
	else
	{
		for ( i = 0; i < data_size; i++ )
		{
			if ( (i & 0xFFFFFFC0) == 0 )
			{
				result = machdep_WaitImmediateFifoEmpty();
				if ( result < MASMW_SUCCESS )
				{
					machdep_WriteStatusFlagReg( cinfo_ptr->mask_interrupt );
					return result;
				}
			}

			if ( ( i % 8 ) == 0 )
			{
				temp = *data_ptr++;
			}
			else
			{
				write_data = *data_ptr++;
				write_data |= (UINT8)( ( temp << (i % 8) ) & 0x80 );

				machdep_WriteDataReg( write_data );
			}
		}
	}

	machdep_WriteStatusFlagReg( cinfo_ptr->mask_interrupt );

	return MASMW_SUCCESS;
}
/****************************************************************************
 *	MaDevDrv_SendDirectRamVal
 *
 *	Description:
 *
 *	Arguments:
 *			address		internal RAM address
 *			size		size of write byte data to internal RAM
 *			val			write byte data
 *	Return:
 *			0			success
 *
 ****************************************************************************/
SINT32 MaDevDrv_SendDirectRamVal
(
	UINT32	address,					/* address of internal ram */
	UINT32	data_size,					/* size of data */
	UINT8	val							/* write value */
)
{
	UINT32	i, j;						/* loop counter */
	UINT32	count;						/* count of packet */
	SINT32	result;						/* result of function */
	UINT8	packet_buffer[3+2];			/* packet buffer */

#if MASMW_DEBUG
	result = madebug_SendDirectRamVal( address, data_size, val );
	if ( result < MASMW_SUCCESS ) return result;
#endif

	MADEVDRV_DBGMSG(("    MaDevDrv_SendDirectRamVal: adrs=%04lX size=%ld val=%02X\n", address, data_size, val));

	if ( data_size == 0 ) return MASMW_SUCCESS;

	machdep_WriteStatusFlagReg( MA_IMMEDIATE_WRITE_REG );


	packet_buffer[0] = (UINT8)( (address >>  0 ) & 0x7F );
	packet_buffer[1] = (UINT8)( (address >>  7 ) & 0x7F );
	packet_buffer[2] = (UINT8)( (address >> 14 ) | 0x80 );

	if	( data_size < 0x0080 )
	{
		packet_buffer[3] = (UINT8)( (data_size >> 0) | 0x80 );
		count = 4;
	}
	else
	{
		packet_buffer[3] = (UINT8)( (data_size >> 0) & 0x7F );
		packet_buffer[4] = (UINT8)( (data_size >> 7) | 0x80 );
		count = 5;
	}


	result = machdep_WaitImmediateFifoEmpty();
	if ( result < MASMW_SUCCESS )
	{
		machdep_WriteStatusFlagReg( cinfo_ptr->mask_interrupt );
		return result;
	}

	for ( i = 0; i < count; i++ )
	{
		machdep_WriteDataReg( packet_buffer[i] );
	}

#if 1
	i = data_size/64;
	while ( (i--) != 0 )
#else
	for ( i = 0; i < (data_size/64); i++ )
#endif
	{
		result = machdep_WaitImmediateFifoEmpty();
		if ( result < MASMW_SUCCESS )
		{
			machdep_WriteStatusFlagReg( cinfo_ptr->mask_interrupt );
			return result;
		}

#if 1
		j = 64;
		while ( (j--) != 0 )
#else
		for ( j = 0; j < 64; j++ )
#endif
		{
			machdep_WriteDataReg( val );
		}
	}

	if ( (data_size%64) != 0 )
	{
		result = machdep_WaitImmediateFifoEmpty();
		if ( result < MASMW_SUCCESS )
		{
			machdep_WriteStatusFlagReg( cinfo_ptr->mask_interrupt );
			return result;
		}
#if 1
		j = data_size%64;
		while ( (j--) != 0 )
#else
		for ( j = 0; j < (data_size%64); j++ )
#endif
		{
			machdep_WriteDataReg( val );
		}
	}

	machdep_WriteStatusFlagReg( cinfo_ptr->mask_interrupt );

	return MASMW_SUCCESS;
}
/****************************************************************************
 *	MaDevDrv_StreamSetup
 *
 *	Description:
 *			Setup wave data to the stream audio.
 *	Arguments:
 *			sa_id	stream audio slot number
 *	Return:
 *			0		success
 *			< 0		error code
 *
 ****************************************************************************/
static SINT32 MaDevDrv_StreamSetup
(
	UINT8	sa_id						/* stream audio number */
)
{
	UINT8	zero_val;
	UINT8	format;
	UINT8 *	wave_ptr;
	UINT32	wave_size;
	UINT32	ram_adrs;
	SINT32	result;

	format    = cinfo_ptr->streaminfo.format[sa_id];
	wave_ptr  = cinfo_ptr->streaminfo.wave_ptr[sa_id];
	wave_size = cinfo_ptr->streaminfo.wave_size[sa_id];

	ram_adrs = (UINT32)(MA_RAM_START_ADDRESS + (MA_RAM_BLOCK_SIZE * (7-sa_id)) + 0x20);

	if ( wave_size == MA_WAVE_SIZE )
		cinfo_ptr->streaminfo.end_point[sa_id] = (UINT16)wave_size;
	else
		if ( format < 2 )
			cinfo_ptr->streaminfo.end_point[sa_id] = (UINT16)(wave_size % MA_WAVE_SIZE);
		else
			cinfo_ptr->streaminfo.end_point[sa_id] =
				(UINT16)( (wave_size + (wave_size / MA_WAVE_SIZE) ) % MA_WAVE_SIZE );

	if ( wave_size > MA_WAVE_SIZE )
	{
		result = MaDevDrv_SendDirectRamData( ram_adrs, 0, wave_ptr, MA_WAVE_SIZE );
		if ( result != MASMW_SUCCESS ) return result;

		if ( format < 2 )					/* ADPCM */
		{
			wave_size -= MA_WAVE_SIZE;
			wave_ptr  += MA_WAVE_SIZE;
		}
		else								/* PCM */
		{
			wave_size -= (MA_WAVE_SIZE - 1);
			wave_ptr  += (MA_WAVE_SIZE - 1);
		}
	}
	else
	{
		result = MaDevDrv_SendDirectRamData( ram_adrs, 0, wave_ptr, wave_size );
		if ( result != MASMW_SUCCESS ) return result;

		if ( format < 2 )					/* ADPCM */
		{
			zero_val = (UINT8)0x80;
		}
		else								/* PCM */
		{
			zero_val = (UINT8)( ( format == 2 ) ? 0x80 : 0x00 );
		}

		MaDevDrv_SendDirectRamVal( ram_adrs + wave_size, MA_WAVE_SIZE - wave_size, zero_val );
		if ( result != MASMW_SUCCESS) return result;

		wave_ptr  += wave_size;
		wave_size -= wave_size;
	}

	cinfo_ptr->streaminfo.state[sa_id]	   = 2;
	cinfo_ptr->streaminfo.wave_ptr[sa_id]  = wave_ptr;
	cinfo_ptr->streaminfo.wave_size[sa_id] = wave_size;

	return MASMW_SUCCESS;
}
/****************************************************************************
 *	MaDevDrv_StreamUpdate
 *
 *	Description:
 *			Update wave data to the stream audio.
 *	Arguments:
 *			sa_id		stream audio slot number
 *	Return:
 *			0		success
 *			< 0		error code
 *
 ****************************************************************************/
static SINT32 MaDevDrv_StreamUpdate
(
	UINT8	sa_id						/* stream audio number */
)
{
	UINT8	packet[6];
	UINT16	write_block;
	UINT16	read_block;
	UINT32	ram_size;
	UINT8	format;
	UINT8 *	wave_ptr;
	UINT32	wave_size;
	SINT32	result = MASMW_SUCCESS;
	UINT32	ram_adrs;
	UINT16	pg_point;
	UINT16	end_point;
	UINT16	prv_point;
	UINT8	zero_val = 0x00;
	UINT32	reg_index;
	UINT8	ch;

	write_block = cinfo_ptr->streaminfo.write_block[sa_id];
	read_block  = cinfo_ptr->streaminfo.read_block[sa_id];
	format 		= cinfo_ptr->streaminfo.format[sa_id];

	reg_index = MA_NOP_2;
	packet[0] = (UINT8)(  reg_index       & 0x7F );		/* address part */
	packet[1] = (UINT8)( (reg_index >> 7) | 0x80 );
	packet[2] = (UINT8)(  0x00            | 0x80 );		/* data part */
	MaDevDrv_SendDirectPacket( packet, 3 );

	pg_point = (UINT8)MaDevDrv_ReceiveData( MA_WT_PG + (7 - sa_id), 0 );

	if ( format < 2 )					/* ADPCM */
	{
		pg_point = (UINT16)((pg_point & 0x3E) << 4);
		zero_val = (UINT8)0x80;
	}
	else								/* PCM */
	{
		pg_point = (UINT16)((pg_point & 0x1F) << 5);
		zero_val = (UINT8)( ( format == 2 ) ? 0x80 : 0x00 );
	}

	cinfo_ptr->streaminfo.position[sa_id]++;

	if ( pg_point == read_block )
	{
		return result;
	}

	ram_adrs = write_block + (MA_RAM_START_ADDRESS + (MA_RAM_BLOCK_SIZE * (7-sa_id)) + 0x20);

	if ( pg_point == MA_MIN_PG )
		read_block = (UINT16)(MA_MAX_PG - (1 << 5));
	else
		read_block = (UINT16)(pg_point  - (1 << 5));

	if ( read_block == write_block ) return result;

	if ( write_block < read_block )
	{
		ram_size = read_block - write_block;

		wave_ptr  = cinfo_ptr->streaminfo.wave_ptr[sa_id];
		wave_size = cinfo_ptr->streaminfo.wave_size[sa_id];

		if ( ram_size > wave_size )
		{
			MaDevDrv_SendDirectRamData( ram_adrs, 0, wave_ptr, wave_size );

			cinfo_ptr->streaminfo.wave_ptr[sa_id]  += wave_size;
			cinfo_ptr->streaminfo.wave_size[sa_id] -= wave_size;

			ram_adrs += wave_size;
			ram_size -= wave_size;
			MaDevDrv_SendDirectRamVal( ram_adrs, ram_size, zero_val );
		}
		else
		{
			MaDevDrv_SendDirectRamData( ram_adrs, 0, wave_ptr, ram_size );

			cinfo_ptr->streaminfo.wave_ptr[sa_id]  += ram_size;
			cinfo_ptr->streaminfo.wave_size[sa_id] -= ram_size;
		}

		cinfo_ptr->streaminfo.write_block[sa_id] = read_block;
		cinfo_ptr->streaminfo.read_block[sa_id]  = pg_point;
	}
	else
	{
		/* 1 */
		ram_size = (UINT32)(MA_MAX_PG - write_block);

		wave_ptr  = cinfo_ptr->streaminfo.wave_ptr[sa_id];
		wave_size = cinfo_ptr->streaminfo.wave_size[sa_id];

		if ( ram_size > wave_size )
		{
			MaDevDrv_SendDirectRamData( ram_adrs, 0, wave_ptr, wave_size );

			ram_adrs += wave_size;
			ram_size -= wave_size;
			MaDevDrv_SendDirectRamVal( ram_adrs, ram_size, zero_val );

			cinfo_ptr->streaminfo.wave_ptr[sa_id]  += wave_size;
			cinfo_ptr->streaminfo.wave_size[sa_id] -= wave_size;
		}
		else
		{
			MaDevDrv_SendDirectRamData( ram_adrs, 0, wave_ptr, ram_size );

			if ( ( format > 1 ) && ( ram_size != 0 ) ) ram_size--;

			cinfo_ptr->streaminfo.wave_ptr[sa_id]  += ram_size;
			cinfo_ptr->streaminfo.wave_size[sa_id] -= ram_size;
		}

		cinfo_ptr->streaminfo.write_block[sa_id] = read_block;
		cinfo_ptr->streaminfo.read_block[sa_id]  = pg_point;

		/* 2 */
		ram_adrs = (UINT32)(MA_MIN_PG + MA_RAM_START_ADDRESS + (MA_RAM_BLOCK_SIZE * (7-sa_id)) + 0x20);
		ram_size = (UINT32)(read_block - MA_MIN_PG);

		if ( ram_size != 0 )
		{
			wave_ptr  = cinfo_ptr->streaminfo.wave_ptr[sa_id];
			wave_size = cinfo_ptr->streaminfo.wave_size[sa_id];

			if ( ram_size > wave_size )
			{
				MaDevDrv_SendDirectRamData( ram_adrs, 0, wave_ptr, wave_size );

				ram_adrs += wave_size;
				ram_size -= wave_size;
				MaDevDrv_SendDirectRamVal( ram_adrs, ram_size, zero_val );

				cinfo_ptr->streaminfo.wave_ptr[sa_id]  += wave_size;
				cinfo_ptr->streaminfo.wave_size[sa_id] -= wave_size;
			}
			else
			{
				MaDevDrv_SendDirectRamData( ram_adrs, 0, wave_ptr, ram_size );

				cinfo_ptr->streaminfo.wave_ptr[sa_id]  += ram_size;
				cinfo_ptr->streaminfo.wave_size[sa_id] -= ram_size;
			}

			cinfo_ptr->streaminfo.write_block[sa_id] = read_block;
			cinfo_ptr->streaminfo.read_block[sa_id]  = pg_point;
		}
	}


	switch ( cinfo_ptr->streaminfo.state[sa_id] )
	{
	case 2:

		if ( cinfo_ptr->streaminfo.wave_size[sa_id] == 0 )
		{
			cinfo_ptr->streaminfo.prv_point[sa_id] = pg_point;
			cinfo_ptr->streaminfo.state[sa_id] = 3;
		}
		break;

	case 3:

		end_point = cinfo_ptr->streaminfo.end_point[sa_id];
		prv_point = cinfo_ptr->streaminfo.prv_point[sa_id];

		if ( ( ( end_point <  pg_point  ) && ( pg_point  <  prv_point ) )
		  || ( ( pg_point  <  prv_point ) && ( prv_point <= end_point ) )
		  || ( ( prv_point <= end_point ) && ( end_point <  pg_point  ) ) )
		{

			cinfo_ptr->streaminfo.state[sa_id] = 0;

			cinfo_ptr->timer0 &= (UINT8)~(1 << sa_id);

			if ( cinfo_ptr->timer0 == 0 )
			{
				packet[0] = (UINT8)(  MA_TIMER_0_CTRL     & 0x7F );
				packet[1] = (UINT8)( (MA_TIMER_0_CTRL>>7) | 0x80 );
				packet[2] = (UINT8)(                        0x80 );
				MaDevDrv_SendDirectPacket( packet, 3 );
			}

			/* KeyOff */
			reg_index = (UINT32)( MA_WT_VOICE_ADDRESS + (7-(UINT32)sa_id) * 6 + 5 );
			ch = (UINT8)( MaDevDrv_ReceiveData( reg_index, 0 ) & 0x0F );

			packet[0] = (UINT8)(  reg_index       & 0x7F );		/* address part */
			packet[1] = (UINT8)( (reg_index >> 7) | 0x80 );
			packet[2] = (UINT8)(  0x00 | ch       | 0x80 );		/* data part */
			MaDevDrv_SendDirectPacket( packet, 3 );

			if ( cinfo_ptr->audio_mode == 1 )
			{
				result = MaSound_ReceiveMessage( 2, 0, 128 );
				result = MaSound_ReceiveMessage( 2, 0, 127 );
			}
		}
		else
		{
			cinfo_ptr->streaminfo.prv_point[sa_id] = pg_point;
		}

		break;

	default:
		break;
	}

	return result;
}
/****************************************************************************
 *	MaDevDrv_StreamHandler
 *
 *	Description:
 *			Control the stream audio.
 *	Arguments:
 *			sa_id		stream audio slot number
 *			ctrl		0: setup
 *						1: update
 *			ram			SINT RAM register value
 *	Return:
 *			None
 *
 ****************************************************************************/
SINT32 MaDevDrv_StreamHandler
(
	UINT8	sa_id,						/* stream audio number */
	UINT8	ctrl,						/* control */
	UINT8	ram_val
)
{
	UINT8	packet[6];
	UINT8	format;
	UINT8 *	wave_ptr;
	UINT32	wave_size;
	UINT8	wave_id[2];
	UINT8	sa_slot[2];
	SINT32	result;
	UINT32	reg_index;
	UINT8	ch;
	UINT8	num_saon;
	UINT8	i;
	UINT8	num;
	UINT32	seek_pos;

	MADEVDRV_DBGMSG(("MaDevDrv_StreamHandler: sa_id=%d ctrl=%d ram_val=%d\n", sa_id, ctrl, ram_val));

	if ( sa_id >= MA_MAX_STREAM_AUDIO ) return MASMW_ERROR;

	_ma_intstate |= (UINT8)(1 << 1);

	if ( ctrl == 0 )						/* setup */
	{
		if ( ( ram_val & 0x40 ) != 0 )
		{

			wave_id[0] = (UINT8)( ram_val & 0x1F );
			sa_slot[0] = sa_id;
			num_saon = 1;

			cinfo_ptr->streaminfo.position[sa_id] = 0;

			if ( ( ram_val & 0x20 ) != 0 )
			{
				sa_slot[1] = (UINT8)( ( sa_id == 0 ) ? 1 : 0 );
				wave_id[1] = (UINT8)( MaDevDrv_ReceiveData( MA_SOFTINT_RAM + sa_slot[1], 0 ) & 0x1F );
				num_saon++;
			}

			for ( i = 0; i < num_saon; i++ )
			{
				if ( cinfo_ptr->streaminfo.state[sa_slot[i]] != 0 )
				{
					reg_index = (UINT32)( MA_WT_VOICE_ADDRESS + (7-(UINT32)sa_slot[i]) * 6 + 5 );
					ch = (UINT8)( MaDevDrv_ReceiveData( reg_index, 0 ) & 0x0F );

					packet[0] = (UINT8)(  reg_index       & 0x7F );		/* address part */
					packet[1] = (UINT8)( (reg_index >> 7) | 0x80 );
					packet[2] = (UINT8)(  0x00 | ch       | 0x80 );		/* data part */
					MaDevDrv_SendDirectPacket( packet, 3 );
				}

				result = MaResMgr_GetStreamAudioInfo( wave_id[i], &format, &wave_ptr, &wave_size, &seek_pos );
				if ( result != MASMW_SUCCESS )
				{
					_ma_intstate &= (UINT8)~(1 << 1);
					return result;
				}

				cinfo_ptr->streaminfo.write_block[sa_slot[i]] = (UINT16)MA_MIN_PG;
				cinfo_ptr->streaminfo.read_block[sa_slot[i]]  = (UINT16)MA_MIN_PG;
				cinfo_ptr->streaminfo.state[sa_slot[i]]		  = 1;
				cinfo_ptr->streaminfo.format[sa_slot[i]]	  = format;
				cinfo_ptr->streaminfo.wave_ptr[sa_slot[i]] 	  = wave_ptr  + seek_pos;
				cinfo_ptr->streaminfo.wave_size[sa_slot[i]]   = wave_size - seek_pos;

				MaDevDrv_StreamSetup( sa_slot[i] );

				if ( cinfo_ptr->seq_flag == 1 )
				{
					if ( cinfo_ptr->timer0 == 0 )
					{
						packet[0] = (UINT8)(  MA_TIMER_0_CTRL     & 0x7F );
						packet[1] = (UINT8)( (MA_TIMER_0_CTRL>>7) | 0x80 );
						packet[2] = (UINT8)( 0x81 );
						MaDevDrv_SendDirectPacket( packet, 3 );
					}

					cinfo_ptr->timer0 |= (1 << sa_slot[i]);
				}
			}

			num = 0;
			for ( i = 0; i < num_saon; i++ )
			{
				reg_index = (UINT32)( MA_WT_VOICE_ADDRESS + (7-(UINT32)sa_slot[i]) * 6 + 5 );
				ch = (UINT8)( MaDevDrv_ReceiveData( reg_index, 0 ) & 0x0F );

				packet[num++] = (UINT8)(  reg_index       & 0x7F );		/* address part */
				packet[num++] = (UINT8)( (reg_index >> 7) | 0x80 );
				packet[num++] = (UINT8)(  0x40 | ch       | 0x80 );		/* data part */
			}

			if ( num != 0 )
				MaDevDrv_SendDirectPacket( packet, num );
		}
		else
		{

			cinfo_ptr->streaminfo.state[sa_id] = 0;

			cinfo_ptr->timer0 &= (UINT8)~(1 << sa_id);


			if ( cinfo_ptr->timer0 == 0 )
			{
				packet[0] = (UINT8)(  MA_TIMER_0_CTRL     & 0x7F );
				packet[1] = (UINT8)( (MA_TIMER_0_CTRL>>7) | 0x80 );
				packet[2] = (UINT8)(                        0x80 );
				MaDevDrv_SendDirectPacket( packet, 3 );
			}

			reg_index = (UINT32)( MA_WT_VOICE_ADDRESS + (7-(UINT32)sa_id) * 6 + 5 );
			ch = (UINT8)( MaDevDrv_ReceiveData( reg_index, 0 ) & 0x0F );

			packet[0] = (UINT8)(  reg_index       & 0x7F );		/* address part */
			packet[1] = (UINT8)( (reg_index >> 7) | 0x80 );
			packet[2] = (UINT8)(  0x00 | ch       | 0x80 );		/* data part */
			MaDevDrv_SendDirectPacket( packet, 3 );
		}
	}
	else									/* update */
	{
		switch ( cinfo_ptr->streaminfo.state[sa_id] )
		{
		case 0:
			break;

		case 1:
			break;

		case 2:
		case 3:

			MaDevDrv_StreamUpdate( sa_id );
			break;
		}
	}

	_ma_intstate &= (UINT8)~(1 << 1);

	return MASMW_SUCCESS;
}
/****************************************************************************
 *	MaDevDrv_SoftInt0
 *
 *	Description:
 *			Software interrupt #0 interrupt handler.
 *	Argument:
 *			ctrl	unused
 *	Return:
 *			None
 *
 ****************************************************************************/
void MaDevDrv_SoftInt0
(
	UINT8	ctrl
)
{
	UINT8	ram_val1;					/* ram value */

	(void)ctrl;							/* for unused warning message */

	MADEVDRV_DBGMSG(("MaDevDrv_SoftInt0: ctrl=%d\n", ctrl));

	ram_val1 = (UINT8)MaDevDrv_ReceiveData( MA_SOFTINT_RAM + 0, 0 );

	MaDevDrv_StreamHandler( 0, 0, ram_val1 );	/* SA #0, setup */

}
/****************************************************************************
 *	MaDevDrv_SoftInt1
 *
 *	Description:
 *			Software interrupt #1 interrupt handler.
 *	Argument:
 *			ctrl	unused
 *	Return:
 *			None
 *
 ****************************************************************************/
void MaDevDrv_SoftInt1
(
	UINT8	ctrl
)
{
	UINT8	ram_val1;					/* ram value */

	(void)ctrl;							/* for unused warning message */

	MADEVDRV_DBGMSG(("MaDevDrv_SoftInt1: ctrl=%d\n", ctrl));

	ram_val1 = (UINT8)MaDevDrv_ReceiveData( MA_SOFTINT_RAM + 1, 0 );

	MaDevDrv_StreamHandler( 1, 0, ram_val1 );	/* SA #1, setup */

}
/****************************************************************************
 *	MaDevDrv_SoftInt2
 *
 *	Description:
 *			Software interrupt #2 interrupt handler.
 *	Argument:
 *			ctrl	unused
 *	Return:
 *			None
 *
 ****************************************************************************/
void MaDevDrv_SoftInt2
(
	UINT8	ctrl
)
{
	UINT8	event;					/* event value */

	(void)ctrl;						/* for unused warning message */

	MADEVDRV_DBGMSG(("MaDevDrv_SoftInt2: ctrl=%d\n", ctrl));

	event = (UINT8)MaDevDrv_ReceiveData( MA_SOFTINT_RAM + 2, 0 );

	MaSound_ReceiveMessage( 0, 0, event );
}
/****************************************************************************
 *	MaDevDrv_SoftInt3
 *
 *	Description:
 *			Software interrupt #3 interrupt handler.
 *	Argument:
 *			ctrl	unused
 *	Return:
 *			None
 *
 ****************************************************************************/
void MaDevDrv_SoftInt3
(
	UINT8	ctrl
)
{
	UINT8	event;					/* event value */

	(void)ctrl;						/* for unused warning message */

	MADEVDRV_DBGMSG(("MaDevDrv_SoftInt3: ctrl=%d\n", ctrl));

	event = (UINT8)MaDevDrv_ReceiveData( MA_SOFTINT_RAM + 3, 0 );

	MaSound_ReceiveMessage( 0, 0, event );
}
/****************************************************************************
 *	MaDevDrv_Timer0
 *
 *	Description:
 *			Timer #0 interrupt handler.
 *	Argument:
 *			ctrl	unused
 *	Return:
 *			None
 *
 ****************************************************************************/
void MaDevDrv_Timer0
(
	UINT8	ctrl
)
{
	SINT32 	result;				/* result of function */
	UINT8	sa_id;				/* stream audio number */

	(void)ctrl;					/* for unused warning message */

	MADEVDRV_DBGMSG(("MaDevDrv_Timer0: ctrl=%d\n", ctrl));

	for ( sa_id = 0; sa_id < MA_MAX_STREAM_AUDIO; sa_id++ )
	{
		result = MaDevDrv_StreamHandler( sa_id, 1, 0 );	/* update */
		if ( result != MASMW_SUCCESS ) return;
	}

	if ( ( cinfo_ptr->timer0 & (UINT8)0x0C ) != 0 )
	{
		result = MaSndDrv_UpdateSequence( 1, NULL, 0 );
	}
}
/****************************************************************************
 *	MaDevDrv_Timer1
 *
 *	Description:
 *			Timer #1 interrupt handler.
 *	Argument:
 *			ctrl
 *	Return:
 *			None
 *
 ****************************************************************************/
void MaDevDrv_Timer1
(
	UINT8	ctrl
)
{
	MADEVDRV_DBGMSG(("MaDevDrv_Timer1: ctrl=%d\n", ctrl));

	(void)ctrl;

	MaSndDrv_UpdatePos( 0, 1 );
}
/****************************************************************************
 *	MaDevDrv_Fifo
 *
 *	Description:
 *			Update sequence data to sequence FIFO.
 *	Argument:
 *			ctrl
 *	Return:
 *			None
 *
 ****************************************************************************/
void MaDevDrv_Fifo
(
	UINT8	ctrl
)
{
	UINT8 *	buf_ptr;
	UINT32	buf_size;
	SINT32	wrote_size;
	UINT32	send_buf_size;
	UINT8	i;
	UINT8	packet[3] = { (   ( MA_SEQUENCE + 2 )        & 0x7F ),
						  ( ( ( MA_SEQUENCE + 2 ) >> 7 ) | 0x80 ),
						  0x80 };

	MADEVDRV_DBGMSG(("MaDevDrv_Fifo: ctrl=%d\n", ctrl));

	_ma_intstate |= (UINT8)(1 << 0);

	if ( cinfo_ptr->end_of_sequence[0] == 2 )
	{
		cinfo_ptr->end_of_sequence[0] = 0;
		MaDevDrv_StopSequencer( 0, 1 );

		packet[2] = (0x80 | MA_INT_POINT);
		MaDevDrv_SendDirectPacket( packet, 3 );

		_ma_intstate &= (UINT8)~(1 << 0);

		return;
	}

	wrote_size = 0;
	send_buf_size = 0;
	for ( i = 0; i <= ctrl; i++ )
	{
		buf_ptr  = cinfo_ptr->sbuf_buffer[cinfo_ptr->sbuf_info.read_num];
		buf_size = cinfo_ptr->sbuf_info.buf_size[cinfo_ptr->sbuf_info.read_num];
		if ( buf_size != 0 )
		{
			if ( cinfo_ptr->ctrl_seq != 0 )
			{
				if ( machdep_CheckDelayedFifoEmpty()!=0 )
				{
					MaDevDrv_StopSequencer( 0, 0 );
					cinfo_ptr->stop_flag = 1;
					cinfo_ptr->stop_reg  = 1;

					machdep_WriteStatusFlagReg( MA_BASIC_SETTING_REG );
					machdep_WriteDataReg( 0x20 );
					machdep_Wait( 300 );
					machdep_WriteDataReg( 0x00 );
					machdep_WriteStatusFlagReg( cinfo_ptr->mask_interrupt );
				}
			}
			MaDevDrv_SendDelayedPacket( buf_ptr, (UINT16)buf_size );

			cinfo_ptr->sbuf_info.buf_size[cinfo_ptr->sbuf_info.read_num] = 0;

			cinfo_ptr->sbuf_info.read_num++;
			if ( cinfo_ptr->sbuf_info.read_num == MA_SBUF_NUM )
			{
				cinfo_ptr->sbuf_info.read_num = 0;
			}
		}
		send_buf_size += buf_size;

		if ( cinfo_ptr->end_of_sequence[0] == 1 ) continue;

		buf_ptr	 = cinfo_ptr->sbuf_buffer[cinfo_ptr->sbuf_info.write_num];
		buf_size = MA_FIFO_SIZE;
		if ( buf_size != 0 )
		{
			wrote_size = MaSndDrv_UpdateSequence( 0, buf_ptr, buf_size  );
			if ( wrote_size >= 0 )
				cinfo_ptr->sbuf_info.buf_size[cinfo_ptr->sbuf_info.write_num] = (UINT16)wrote_size;

			cinfo_ptr->sbuf_info.write_num++;
			if ( cinfo_ptr->sbuf_info.write_num == MA_SBUF_NUM )
			{
				cinfo_ptr->sbuf_info.write_num = 0;
			}
		}
	}

	if ( ( cinfo_ptr->stop_flag == 1 ) && ( cinfo_ptr->ctrl_seq != 0 ) )
	{
		machdep_WriteStatusFlagReg( MA_BASIC_SETTING_REG );
		machdep_WriteDataReg( 0x08 );
		machdep_Wait( 300 );
		machdep_WriteDataReg( 0x00 );
		machdep_WriteStatusFlagReg( cinfo_ptr->mask_interrupt );

		cinfo_ptr->stop_flag = 0;
		MaDevDrv_StartSequencer( 0, 0 );
	}

	if ( ( ctrl == 0 ) && ( send_buf_size == 0 ) && ( wrote_size == 0 ) && ( cinfo_ptr->end_of_sequence[0] == 1 ) )
	{
		MaDevDrv_SendDirectPacket( packet, 3 );
		cinfo_ptr->end_of_sequence[0]++;
	}

	_ma_intstate &= (UINT8)~(1 << 0);
}
/****************************************************************************
 *	MaDevDrv_ClearFifo
 *
 *	Description:
 *			Clear FIFO.
 *	Argument:
 *			None
 *	Return:
 *			0		success
 *			< 0		error code
 *
 ****************************************************************************/
SINT32 MaDevDrv_ClearFifo
(void)
{
	SINT32	i;
	static UINT8 packet[3] = { (   ( MA_SEQUENCE + 2 )        & 0x7F ),
							   ( ( ( MA_SEQUENCE + 2 ) >> 7 ) | 0x80 ),
							   (     MA_INT_POINT             | 0x80 ) };

	MADEVDRV_DBGMSG(("MaDevDrv_ClearFifo\n"));

	machdep_WriteStatusFlagReg( MA_BASIC_SETTING_REG );
	machdep_WriteDataReg( 0x28 );
	machdep_Wait( 300 );						/* wait 300ns */
	machdep_WriteDataReg( 0x00 );

	MaDevDrv_SendDirectPacket( packet, 3 );

	cinfo_ptr->sbuf_info.write_num = 0;
	cinfo_ptr->sbuf_info.read_num  = 0;
	for ( i = 0; i < MA_SBUF_NUM; i++ )
	{
		cinfo_ptr->sbuf_info.buf_size[i] = 0;
	}

	machdep_WriteStatusFlagReg( cinfo_ptr->mask_interrupt );

	cinfo_ptr->end_of_sequence[0] = 0;

	cinfo_ptr->stop_flag = 0;
	cinfo_ptr->stop_reg = 0;

	return MASMW_SUCCESS;
}
/****************************************************************************
 *	MaDevDrv_StartSequencer
 *
 *	Description:
 *			Start sequencer.
 *	Argument:
 *			seq_id	sequence id
 *			ctrl
 *	Return:
 *			0		success
 *			< 0		error code
 *
 ****************************************************************************/
SINT32 MaDevDrv_StartSequencer
(
	SINT32	seq_id,						/* sequence id */
	UINT8	ctrl						/* control flag */
)
{
	SINT32	result;						/* result of function */
	static UINT8	packet1[3] = { 0x61, 0x82, 0x81 };	/* start sequencer */
	static UINT8	packet2[3] = { 0x55, 0x82, 0x81 };	/* timer 0 start */
	UINT8	save_mask;

	MADEVDRV_DBGMSG(("MaDevDrv_StartSequencer: seq_id=%ld\n", seq_id));

	result = MASMW_SUCCESS;

	/* mask interrupt */
	machdep_WriteStatusFlagReg( (UINT8)MA_INTERRUPT_FLAG_REG );
	save_mask = cinfo_ptr->mask_interrupt;
	cinfo_ptr->mask_interrupt = 0x00;

	switch ( seq_id )
	{
	case 0:						/* delayed sequence */

		if ( ctrl == 1 ) cinfo_ptr->ctrl_seq = 1;

		MaDevDrv_ControlInterrupt( 0, 0x7F );

		result = MaDevDrv_SendDirectPacket( packet1, 3 );

		break;

	case 1:

		if ( cinfo_ptr->timer0 == 0 )
		{
			MaDevDrv_ControlInterrupt( 0, 0x1F );

			result = MaDevDrv_SendDirectPacket( packet2, 3 );
		}
		cinfo_ptr->timer0 |= (UINT8)(1 << 2);

		break;

	case 2:

		if ( cinfo_ptr->timer0 == 0 )
		{
			MaDevDrv_ControlInterrupt( 0, 0x1F );

			result = MaDevDrv_SendDirectPacket( packet2, 3 );
		}
		cinfo_ptr->timer0 |= (UINT8)(1 << 3);
		break;

	default:

		result = MASMW_ERROR;
		break;
	}

	cinfo_ptr->seq_flag = 1;

	/* unmask interrupt */
	cinfo_ptr->mask_interrupt = save_mask;
	if ( save_mask == 0x80 )
	{
		machdep_WriteStatusFlagReg( cinfo_ptr->mask_interrupt );
	}

	return result;
}
/****************************************************************************
 *	MaDevDrv_StopSequencer
 *
 *	Description:
 *			Stop sequencer.
 *	Argument:
 *			seq_id
 *			ctrl
 *	Return:
 *			0		success
 *			< 0		error code
 *
 ****************************************************************************/
SINT32 MaDevDrv_StopSequencer
(
	SINT32	seq_id,						/* sequence id */
	UINT8	ctrl						/* control flag */
)
{
	SINT32	result = MASMW_SUCCESS;
	static UINT8	packet1[3] = { 0x61, 0x82, 0x80 };	/* sequencer stop */
	static UINT8	packet2[3] = { 0x55, 0x82, 0x80 };	/* timer 0 stop */

	MADEVDRV_DBGMSG(("MaDevDrv_StopSequencer: seq_id=%ld\n", seq_id));

	switch ( seq_id )
	{
	case 0:						/* delayed sequence */

		if ( ctrl == 1 ) cinfo_ptr->ctrl_seq = 0;

		MaDevDrv_ControlInterrupt( 1, 0x60 );

		result = MaDevDrv_SendDirectPacket( packet1, 3 );

		break;

	case 1:						/* direct sequence */

		cinfo_ptr->timer0 &= (UINT8)~(1 << 2);

		if ( cinfo_ptr->timer0 == 0 )
		{
			MaDevDrv_ControlInterrupt( 1, 0x00 );

			result = MaDevDrv_SendDirectPacket( packet2, 3 );
		}

		break;

	case 2:

		cinfo_ptr->timer0 &= (UINT8)~(1 << 3);

		if ( cinfo_ptr->timer0 == 0 )
		{
			MaDevDrv_ControlInterrupt( 1, 0x00 );

			result = MaDevDrv_SendDirectPacket( packet2, 3 );
		}
		break;

	default:

		result = MASMW_ERROR;
		break;
	}

	cinfo_ptr->seq_flag = 0;

	return result;
}
/****************************************************************************
 *	MaDevDrv_EndOfSequence
 *
 *	Description:
 *			Receive the end of sequence.
 *	Argument:
 *			None
 *	Return:
 *			0		success
 *			< 0		error code
 *
 ****************************************************************************/
SINT32 MaDevDrv_EndOfSequence
(
	void
)
{
	MADEVDRV_DBGMSG(("MaDevDrv_EndOfSequence:\n"));

	cinfo_ptr->end_of_sequence[0] = 1;

	return MASMW_SUCCESS;
}
/****************************************************************************
 *	MaDevDrv_InitRegisters
 *
 *	Description:
 *			Initialize the uninitialized registers by software reset
 *	Argument:
 *			None
 *	Return:
 *			None
 *
 ****************************************************************************/
void MaDevDrv_InitRegisters
(
	void
)
{
	UINT8	count;

	static UINT8 index[15] = { 0x00, 0x04, 0x07, 0x08, 0x09, 0x0A, 0x0F, 0x04,
							   0x0B, 0x0C, 0x04, 0x05, 0x06, 0x0A, 0x0D, };

	static UINT8 data[15]  = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01,
							   0x03, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00 };

	MADEVDRV_DBGMSG(("MaDevDrv_InitRegisters:\n"));

	for ( count = 0; count < 15; count++ )
	{
		/* set index of interrupt flag register to REG_ID */
		machdep_WriteStatusFlagReg( index[count] );

		/* write data to the target register */
		machdep_WriteDataReg( data[count] );
	}
}
/****************************************************************************
 *	MaDevDrv_VerifyRegisters
 *
 *	Description:
 *			Verify the initialized registers by software reset
 *	Argument:
 *			None
 *	Return:
 *			0		success
 *			-1		error
 *
 ****************************************************************************/
SINT32 MaDevDrv_VerifyRegisters
(
	void
)
{
	UINT32	reg_adrs;					/* register address */
	UINT32	count;						/* loop count */
	UINT8	data;						/* read data */

	MADEVDRV_DBGMSG(("MaDevDrv_VerifyRegisters:\n"));

	/* Verify the channel volume value to 0x60 */
	reg_adrs = (UINT32)MA_CHANNEL_VOLUME;
	for ( count = 0; count < 16; count++ )
	{
		data = (UINT8)MaDevDrv_ReceiveData( (UINT32)(reg_adrs + count), 2 );
		if ( data != 0x60 )
		{
			return MASMW_ERROR;
		}
	}

	/* Verify the panpot value to 0x3C */
	reg_adrs = (UINT32)MA_CHANNEL_PANPOT;
	for ( count = 0; count < 16; count++ )
	{
		data = (UINT8)MaDevDrv_ReceiveData( (UINT32)(reg_adrs + count), 2 );
		if ( data != 0x3C )
		{
			return MASMW_ERROR;
		}
	}

	/* Verify the WT position value to 0x40 */
	reg_adrs = (UINT32)MA_WT_PG;
	for ( count = 0; count < 8; count++ )
	{
		data = (UINT8)MaDevDrv_ReceiveData( (UINT32)(reg_adrs + count), 2 );
		if ( data != 0x40 )
		{
			return MASMW_ERROR;
		}
	}

	return MASMW_SUCCESS;
}
/****************************************************************************
 *	MaDevDrv_PowerManagement
 *
 *	Description:
 *			Power management.
 *	Argument:
 *			mode	0: Hardware initialize sequence (power down)
 *					1: Hardware initialize sequence (normal)
 *					2: Power down change sequence
 *					3: Power down release sequence
 *	Return:
 *			0		success
 *			< 0		error code
 *
 ****************************************************************************/
SINT32 MaDevDrv_PowerManagement
(
	UINT8	mode						/* operation mode */
)
{
	UINT8	count;						/* loop counter */
	SINT32	result = MASMW_SUCCESS;		/* result of function */

	MADEVDRV_DBGMSG(("MaDevDrv_PowerManagement: mode=%d\n", mode));

	switch ( mode )
	{
	case 0:

		/* sequence of hardware initialize when power downed */

		/* set BANK bits of REG_ID #4 basic setting register to '0' */
		machdep_WriteStatusFlagReg( MA_BASIC_SETTING_REG );
		machdep_WriteDataReg( 0x00 );

		/* set DP0 bit of REG_ID #5 power management (D) setting register to '0' */
		machdep_WriteStatusFlagReg( MA_POWER_MANAGEMENT_DIGITAL_REG );
		machdep_WriteDataReg( MA_DP3 | MA_DP2 | MA_DP1 );

		/* set PLLPD bit of power management (A) setting register to '0' */
		machdep_WriteStatusFlagReg( MA_POWER_MANAGEMENT_ANALOG_REG );
		machdep_WriteDataReg( MA_AP4R | MA_AP4L | MA_AP3 | MA_AP2 | MA_AP1 | MA_AP0 );

		/* wait 10ms */
		machdep_Wait( 10 * 1000 * 1000 );

		/* set DP1 bit of REG_ID #5 power management (D) setting register to '0' */
		machdep_WriteStatusFlagReg( MA_POWER_MANAGEMENT_DIGITAL_REG );
		machdep_WriteDataReg( MA_DP3 | MA_DP2 );

		/* set DP2 bit of REG_ID #5 power management (D) setting register to '0' */
		machdep_WriteDataReg( MA_DP3 );

		for ( count = 0; count < MA_RESET_RETRYCOUNT; count++ )
		{
			/* set RST bit of REG_ID #4 basic setting register to '1' */
			machdep_WriteStatusFlagReg( MA_BASIC_SETTING_REG );
			machdep_WriteDataReg( 0x80 );

			/* set RST bit of REG_ID #4 basic setting register to '0' */
			machdep_WriteDataReg( 0x00 );

			/* verify the initialized registers by software reset */
			result = MaDevDrv_VerifyRegisters();
			if ( result == MASMW_SUCCESS ) break;
		}

		/* wait 41.7us */
		machdep_Wait( 41700 );

		/* set DP2 bit of REG_ID #5 power management (D) setting register to '1' */
		machdep_WriteStatusFlagReg( MA_POWER_MANAGEMENT_DIGITAL_REG );
		machdep_WriteDataReg( MA_DP3 | MA_DP2 );

		/* set DP1 bit of REG_ID #5 power management (D) setting register to '1' */
		machdep_WriteDataReg( MA_DP3 | MA_DP2 | MA_DP1 );

		/* set PLLPD bit of REG_ID #6 power management (A) setting register to '1' */
		machdep_WriteStatusFlagReg( MA_POWER_MANAGEMENT_ANALOG_REG );
		machdep_WriteDataReg( MA_PLLPD | MA_AP4R | MA_AP4L | MA_AP3 | MA_AP2 | MA_AP1 | MA_AP0 );

		/* set DP0 bit of REG_ID #5 power management (D) setting register to '1' */
		machdep_WriteStatusFlagReg( MA_POWER_MANAGEMENT_DIGITAL_REG );
		machdep_WriteDataReg( MA_DP3 | MA_DP2 | MA_DP1 | MA_DP0 );

		/* enable interrupt if needed */
		machdep_WriteStatusFlagReg( cinfo_ptr->mask_interrupt );

		break;

	case 1:

		/* sequence of hardware initialize when normal */

		/* set BANK bits of REG_ID #4 basic setting register to '0' */
		machdep_WriteStatusFlagReg( MA_BASIC_SETTING_REG );
		machdep_WriteDataReg( 0x00 );

		/* set DP0 bit of REG_ID #5 power management (D) setting register to '0' */
		machdep_WriteStatusFlagReg( MA_POWER_MANAGEMENT_DIGITAL_REG );
		machdep_WriteDataReg( MA_DP3 | MA_DP2 | MA_DP1 );

		/* set PLLPD and AP0 bits of REG_ID #6 power management (A) setting register to '0' */
		machdep_WriteStatusFlagReg( MA_POWER_MANAGEMENT_ANALOG_REG );
		machdep_WriteDataReg( MA_AP4R | MA_AP4L | MA_AP3 | MA_AP2 | MA_AP1 );

		machdep_Wait( 10 * 1000 * 1000 );

		/* set DP1 bit of REG_ID #5 power management (D) setting register to '0' */
		machdep_WriteStatusFlagReg( MA_POWER_MANAGEMENT_DIGITAL_REG );
		machdep_WriteDataReg( MA_DP3 | MA_DP2 );

		/* set DP2 bit of REG_ID #5 power management (D) setting register to '0' */
		machdep_WriteDataReg( MA_DP3 );

		for ( count = 0; count < 10; count++ )
		{
			/* set RST bit of REG_ID #4 basic setting register to '1' */
			machdep_WriteStatusFlagReg( MA_BASIC_SETTING_REG );
			machdep_WriteDataReg( 0x80 );

			/* set RST bit of REG_ID #4 basic setting register to '0' */
			machdep_WriteDataReg( 0x00 );

			/* verify the initialized registers by software reset */
			result = MaDevDrv_VerifyRegisters();
			if ( result == MASMW_SUCCESS ) break;
		}

		/* wait 41.7us */
		machdep_Wait( 41700 );

		/* set DP3 bit of REG_ID #5 power management (D) setting register to '0' */
		machdep_WriteStatusFlagReg( MA_POWER_MANAGEMENT_DIGITAL_REG );
		machdep_WriteDataReg( 0x00 );

		/* set AP1, AP3 and AP4 bits of REG_ID #6 power management (A) setting register to '0' */
		machdep_WriteStatusFlagReg( MA_POWER_MANAGEMENT_ANALOG_REG );
		machdep_WriteDataReg( MA_AP2 );

		/* wait 10us */
		machdep_Wait( 10 * 1000 );

		/* set AP2 bit of REG_ID #6 power management (A) setting register to '0' */
		machdep_WriteDataReg( 0x00 );

		/* enable interrupt */
		machdep_WriteStatusFlagReg( cinfo_ptr->mask_interrupt );

		/* normal */

		break;

	case 2:

		/* sequence of power down */

		/* set BANK bits of REG_ID #4 basic setting register to '0' */
		machdep_WriteStatusFlagReg( MA_BASIC_SETTING_REG );
		machdep_WriteDataReg( 0x00 );


		/* set DP3 bit of REG_ID #5 power management (D) setting register to '1' */
		machdep_WriteStatusFlagReg( MA_POWER_MANAGEMENT_DIGITAL_REG );
		machdep_WriteDataReg( MA_DP3 );

		/* set DP2 bit of REG_ID #5 power management (D) setting register to '1' */
		machdep_WriteDataReg( MA_DP3 | MA_DP2 );

		/* set DP1 bit of REG_ID #5 power management (D) setting register to '1' */
		machdep_WriteDataReg( MA_DP3 | MA_DP2 | MA_DP1 );

		/* set AP1, AP2, AP3, AP4 and PLLPD bits of REG_ID #6 power management (A) setting register to '1' */
		machdep_WriteStatusFlagReg( MA_POWER_MANAGEMENT_ANALOG_REG );
		machdep_WriteDataReg( MA_PLLPD | MA_AP4R | MA_AP4L | MA_AP3 | MA_AP2 | MA_AP1 | MA_AP0 );

		/* set DP0 bit of REG_ID #5 power management (D) setting register to '1' */
		machdep_WriteStatusFlagReg( MA_POWER_MANAGEMENT_DIGITAL_REG );
		machdep_WriteDataReg( MA_DP3 | MA_DP2 | MA_DP1 | MA_DP0 );

		/* enable interrupt */
		machdep_WriteStatusFlagReg( cinfo_ptr->mask_interrupt );

		break;

	case 3:


		/* set BANK bits of REG_ID #4 basic setting register to '0' */
		machdep_WriteStatusFlagReg( MA_BASIC_SETTING_REG );
		machdep_WriteDataReg( 0x00 );

		/* set DP0 bit of REG_ID #5 power management (D) setting register to '0' */
		machdep_WriteStatusFlagReg( MA_POWER_MANAGEMENT_DIGITAL_REG );
		machdep_WriteDataReg( MA_DP3 | MA_DP2 | MA_DP1 );

		/* set AP0 and PLLPD bits of REG_ID #6 power management (A) setting register to '0' */
		machdep_WriteStatusFlagReg( MA_POWER_MANAGEMENT_ANALOG_REG );
		machdep_WriteDataReg( MA_AP4R | MA_AP4L | MA_AP3 | MA_AP2 | MA_AP1 );

		/* wait 10ms */
		machdep_Wait( 10 * 1000 * 1000 );

		/* set DP1 bit of REG_ID #5 power management (D) setting register to '0' */
		machdep_WriteStatusFlagReg( MA_POWER_MANAGEMENT_DIGITAL_REG );
		machdep_WriteDataReg( MA_DP3 | MA_DP2 );

		/* set DP2 bit of REG_ID #5 power management (D) setting register to '0' */
		machdep_WriteDataReg( MA_DP3 );

		/* set DP3 bit of REG_ID #5 power management (D) setting register to '0' */
		machdep_WriteDataReg( 0x00 );

		/* set AP1, AP3 and AP4 bits of REG_ID #6 power management (A) setting register to '0' */
		machdep_WriteStatusFlagReg( MA_POWER_MANAGEMENT_ANALOG_REG );
		machdep_WriteDataReg( MA_AP2 );

		/* wait 10us */
		machdep_Wait( 10 * 1000 );

		/* set AP2 bit of REG_ID #6 power management (A) setting register to '0' */
		machdep_WriteDataReg( 0x00 );

		/* enable interrupt */
		machdep_WriteStatusFlagReg( cinfo_ptr->mask_interrupt );

		break;

	default:
		break;
	}

	return result;
}
/****************************************************************************
 *	MaDevDrv_DeviceControl
 *
 *	Description:
 *			Control the device.
 *	Arguments:
 *			cmd			command number (0..10)
 *			param1		1st parameter
 *			param2		2nd parameter
 *			param3		3rd parameter
 *	Return:
 *			None
 *
 *				message			cmd		param1		param2		param3
 *				-------------------------------------------------------
 *				DIGITAL PWR		0		val
 *				ANALOG PWR		1		val
 *				EQVOL			2		val
 *				HPVOL			3		mono		vol_l		vol_r
 *				SPVOL			4		vol
 *				LED				5		led			freq		mode
 *				MOTOR			6		mtr			freq		mode
 *				PLL				7		adjust1		adjust2
 *				VOL MODE		8		mute		chvol		panpot
 *				EFFECT			9		prb
 *				FM MODE			10		mode
 *				GET SEQ STATUS	11
 *
 ****************************************************************************/
SINT32 MaDevDrv_DeviceControl
(
	UINT8	cmd,						/* command number */
	UINT8	param1,						/* parameter 1 */
	UINT8	param2,						/* parameter 2 */
	UINT8	param3						/* parameter 3 */
)
{
	SINT32	seq_flag;
	UINT8	packet[3];

	MADEVDRV_DBGMSG(("  MaDevDrv_DeviceControl: cmd=%d p1=%d p2=%d p3=%d\n", cmd, param1, param2, param3));

	if ( cmd <= 6 )
	{
		machdep_WriteStatusFlagReg( MA_BASIC_SETTING_REG );
		machdep_WriteDataReg( 0x00 );

		switch ( cmd )
		{
		case 0:
			machdep_WriteStatusFlagReg( MA_POWER_MANAGEMENT_DIGITAL_REG );
			machdep_WriteDataReg( param1 );
			break;
		case 1:
			machdep_WriteStatusFlagReg( MA_POWER_MANAGEMENT_ANALOG_REG );
			machdep_WriteDataReg( (UINT8)(param1 & 0xBF) );
			break;
		case 2:
			machdep_WriteStatusFlagReg( MA_ANALOG_EQVOL_REG );
			machdep_WriteDataReg( (UINT8)(param1 & 0x1F) );
			break;
		case 3:
			machdep_WriteStatusFlagReg( MA_ANALOG_HPVOL_L_REG );
			machdep_WriteDataReg( (UINT8)((param1<<7) | (param2&0x1F)) );
			machdep_WriteStatusFlagReg( MA_ANALOG_HPVOL_R_REG );
			machdep_WriteDataReg( param3 );
			break;
		case 4:
			machdep_WriteStatusFlagReg( MA_ANALOG_SPVOL_REG );
			machdep_WriteDataReg( (UINT8)((MA_VSEL<<6) | (param1&0x1F)) );
			break;
		case 5:
			machdep_WriteStatusFlagReg( MA_LED_SETTING_1_REG );
			machdep_WriteDataReg( (UINT8)(param1 & 0x3F) );
			machdep_WriteStatusFlagReg( MA_LED_SETTING_2_REG );
			machdep_WriteDataReg( (UINT8)((param2<<4) | (param3 & 0x07)) );
			break;
		case 6:
			machdep_WriteStatusFlagReg( MA_MOTOR_SETTING_1_REG );
			machdep_WriteDataReg( (UINT8)(param1 & 0x3F) );
			machdep_WriteStatusFlagReg( MA_MOTOR_SETTING_2_REG );
			machdep_WriteDataReg( (UINT8)((param2<<4) | (param3 & 0x07)) );
			break;
		}

		machdep_WriteStatusFlagReg( cinfo_ptr->mask_interrupt );
	}
	else if ( cmd <= 7 )
	{
		machdep_WriteStatusFlagReg( MA_BASIC_SETTING_REG );
		machdep_WriteDataReg( 0x01 );

		machdep_WriteStatusFlagReg( MA_PLL_SETTING_1_REG );
		machdep_WriteDataReg( (UINT8)(param1 & 0x1F) );
		machdep_WriteStatusFlagReg( MA_PLL_SETTING_2_REG );
		machdep_WriteDataReg( (UINT8)(param2 & 0x7F) );

		machdep_WriteStatusFlagReg( cinfo_ptr->mask_interrupt );
	}
	else if ( cmd <= 10 )
	{
		switch ( cmd )
		{
		case 8:						/* #355 smooth vol */
			packet[0] = 0x63;
			packet[1] = 0x82;
			packet[2] = (UINT8)( 0x80 | (param1<<4) | (param2<<2) | param3 );
			break;
		case 9:						/* #357 effects */
			packet[0] = 0x65;
			packet[1] = 0x82;
			packet[2] = (UINT8)( 0x80 | param1 );
			break;
		case 10:					/* #359 FM mode */
			packet[0] = 0x67;
			packet[1] = 0x82;
			packet[2] = (UINT8)( 0x80 | param1 );
			break;
		}

		MaDevDrv_SendDirectPacket( packet, 3 );
	}
	else if ( cmd == 11 )
	{
		machdep_WriteStatusFlagReg( 0x00 );

		seq_flag = (SINT32)cinfo_ptr->stop_reg;
		cinfo_ptr->stop_reg = 0;

		machdep_WriteStatusFlagReg( cinfo_ptr->mask_interrupt );

		return seq_flag;
	}
	else
	{
		return MASMW_ERROR;
	}

	return MASMW_SUCCESS;
}
/****************************************************************************
 *	MaDevDrv_Initialize
 *
 *	Description:
 *			Initialize the MA Device Driver module.
 *	Arguments:
 *			None
 *	Return:
 *			0		success
 *			-1		error
 *
 ****************************************************************************/
SINT32 MaDevDrv_Initialize( void )
{
	UINT32	i;							/* loop counter */
	SINT32	result;						/* result of function */

	MADEVDRV_DBGMSG(("MaDevDrv_Initialize\n"));

	cinfo_ptr->seq_flag			= 0;
	cinfo_ptr->stop_reg			= 0;
	cinfo_ptr->stop_flag		= 0;
	cinfo_ptr->ctrl_seq			= 0;
	cinfo_ptr->timer0 			= 0;
	cinfo_ptr->mask_interrupt	= 0x80;
	cinfo_ptr->int_func_map 	= 0x00;
	cinfo_ptr->audio_mode		= 0;

	for ( i = 0; i < 3; i++ )
	{
		cinfo_ptr->end_of_sequence[i] = 0;		/* !! */
	}

	cinfo_ptr->sbuf_info.write_num = 0;
	cinfo_ptr->sbuf_info.read_num  = 0;
	cinfo_ptr->sbuf_info.buf_total = 0;
	cinfo_ptr->sbuf_info.buf_ptr   = 0;
	for ( i = 0; i < MA_SBUF_NUM; i++ )
	{
		cinfo_ptr->sbuf_info.buf_size[i] = 0;
	}

	for ( i = 0; i < MA_MAX_STREAM_AUDIO; i++ )
	{
		cinfo_ptr->streaminfo.state[i] 		 = 0;
		cinfo_ptr->streaminfo.write_block[i] = 0;
		cinfo_ptr->streaminfo.read_block[i]  = 0;
		cinfo_ptr->streaminfo.format[i]		 = 0;
		cinfo_ptr->streaminfo.wave_ptr[i]	 = NULL;
		cinfo_ptr->streaminfo.wave_size[i]	 = 0;
		cinfo_ptr->streaminfo.position[i]	 = 0;
		cinfo_ptr->streaminfo.end_point[i] 	 = 0;
		cinfo_ptr->streaminfo.prv_point[i]	 = 0;
	}

	MaDevDrv_AddIntFunc( 0, MaDevDrv_SoftInt0 );		/* Soft Int #0 */
	MaDevDrv_AddIntFunc( 1, MaDevDrv_SoftInt1 );		/* Soft Int #1 */
	MaDevDrv_AddIntFunc( 2, MaDevDrv_SoftInt2 );		/* Soft Int #2 */
	MaDevDrv_AddIntFunc( 3, MaDevDrv_SoftInt3 );		/* Soft Int #3 */
	MaDevDrv_AddIntFunc( 4, dummy_IntFunc );			/* */
	MaDevDrv_AddIntFunc( 5, MaDevDrv_Timer0 );			/* Timer #0 */
	MaDevDrv_AddIntFunc( 6, MaDevDrv_Timer1 );			/* Timer #1 */
	MaDevDrv_AddIntFunc( 7, MaDevDrv_Fifo );			/* FIFO */

	_ma_intstate = 0;


	/* Initialize the uninitialized registers by software reset */
	MaDevDrv_InitRegisters();

	/* Set the PLL. */
	MaDevDrv_DeviceControl( 7, MA_ADJUST1_VALUE, MA_ADJUST2_VALUE, 0 );

	/* Disable power down mode. */
	result = MaDevDrv_PowerManagement( 1 );
	if ( result != MASMW_SUCCESS )
	{
		return result;
	}

	/* Set volume mode */
	MaDevDrv_DeviceControl( 8, 0x03, 0x03, 0x03 );

	return MASMW_SUCCESS;
}
