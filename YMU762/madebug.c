/****************************************************************************
 *																			*
 *		Copyright (C) 2001-2002	YAMAHA CORPORATION							*
 *																			*
 *		Module		: madebug.c												*
 *																			*
 *		Description	: for MA Sound Middleware debug							*
 *																			*
 * 		Version		: 1.3.11.0	2002.08.27									*
 *																			*
 ****************************************************************************/

#include "mamachdep.h"
#include "madebug.h"
#include "madefs.h"

#if MASMW_DEBUG

#define MADBG_INIT	0
#define MADBG_SEQU 1

typedef struct _MADEBUGINFO
{
	UINT8 * ptr;					/* pointer to the current position */
	UINT32	buf_size;
	UINT32	size;
	UINT8	mode;
	UINT8 *	size_ptr;
	UINT8	init_status;			/* status of the 'init' chunk 		*/
	UINT8 * init_ptr;				/* pointer to the 'init' chunk		*/
	UINT32	init_size;				/* size of the 'init' chunk 		*/
	UINT32	strm_num;				/* number of stream 				*/
	UINT8 * strm_ptr;				/* pointer to the 'strm' chunk 		*/
	UINT32	strm_size;				/* size of the 'strm' chunk			*/
	UINT8 * sequ_ptr;				/* pointer to the 'sequ' chunk		*/
	UINT32	sequ_size;				/* size of the 'sequ' chunk			*/
} MADEBUGINFO;

static MADEBUGINFO madbg_info;

/****************************************************************************
 *	madebug_Open
 *
 ****************************************************************************/
void madebug_Open( UINT8 * ptr, UINT32 size )
{
	madbg_info.ptr		 	= ptr;
	madbg_info.buf_size	 	= size;
	madbg_info.size		 	= 0;
	madbg_info.mode         = 0;
	madbg_info.size_ptr	 	= NULL;
	madbg_info.init_status	= 0;
	madbg_info.init_ptr	 	= NULL;
	madbg_info.init_size 	= 0;
	madbg_info.strm_num		= 0;
	madbg_info.strm_ptr	 	= NULL;
	madbg_info.strm_size 	= 0;
	madbg_info.sequ_ptr	 	= NULL;
	madbg_info.sequ_size 	= 0;
	
	*madbg_info.ptr++ = 'm';
	*madbg_info.ptr++ = 'a';
	*madbg_info.ptr++ = '3';
	*madbg_info.ptr++ = 'a';

	madbg_info.size_ptr = madbg_info.ptr;

	madbg_info.ptr += 4;

	/* */
	*madbg_info.ptr++ = 'i';
	*madbg_info.ptr++ = 'n';
	*madbg_info.ptr++ = 'f';
	*madbg_info.ptr++ = 'o';
	*madbg_info.ptr++ = 0x00;
	*madbg_info.ptr++ = 0x00;
	*madbg_info.ptr++ = 0x00;
	*madbg_info.ptr++ = 0x09;
	*madbg_info.ptr++ = 't';
	*madbg_info.ptr++ = 'm';
	*madbg_info.ptr++ = 'p';
	*madbg_info.ptr++ = 'o';
	*madbg_info.ptr++ = 0x00;
	*madbg_info.ptr++ = 0x00;
	*madbg_info.ptr++ = 0x00;
	*madbg_info.ptr++ = 0x01;
	*madbg_info.ptr++ = 0x04;
	
	madbg_info.size += 17;
}
/****************************************************************************
 *	madebug_Close
 *
 ****************************************************************************/
UINT32 madebug_Close()
{
	UINT32	i;
	
	for ( i = 0; i < 4; i++ )
		*(madbg_info.size_ptr+i) = (UINT8)( madbg_info.size >> ((3-i)*8) );

	if ( madbg_info.init_size != 0 )
	{
		for ( i = 0; i < 4; i++ )
			*(madbg_info.init_ptr+i) = (UINT8)( madbg_info.init_size >> ((3-i)*8) );
	}

	if ( madbg_info.strm_size != 0 )
	{
		for ( i = 0; i < 4; i++ )
			*(madbg_info.strm_ptr+i) = (UINT8)( madbg_info.strm_size >> ((3-i)*8) );
		for ( i = 0; i < 4; i++ )
			*(madbg_info.strm_ptr+i+4) = (UINT8)( madbg_info.strm_num >> ((3-i)*8) );
	}

	if ( madbg_info.sequ_size != 0 )
	{
		for ( i = 0; i < 4; i++ )
			*(madbg_info.sequ_ptr+i) = (UINT8)( madbg_info.sequ_size >> ((3-i)*8) );
	}

	return (UINT32)(madbg_info.size + 8);
}
/****************************************************************************
 *	madebug_SetMode
 *
 ****************************************************************************/
void madebug_SetMode( unsigned char mode )
{
	madbg_info.mode = mode;
}
/****************************************************************************
 *	madebug_Write
 *
 ****************************************************************************/
static SINT32 madebug_Write
(
	UINT8	val
)
{
	if ( madbg_info.buf_size > madbg_info.size )
	{
		*(madbg_info.ptr) = val;

		madbg_info.ptr++;
		madbg_info.size++;
	}

	return MASMW_SUCCESS;
}
/****************************************************************************
 *	madebug_SendDelayedPacket
 *
 ****************************************************************************/
SINT32 madebug_SendDelayedPacket
(
	const UINT8 * ptr,		/* pointer to packets */
	UINT16	size			/* byte size of packets */
)
{
	UINT16	i;
	UINT8	write_data;

	if ( madbg_info.sequ_ptr == NULL )
	{
		madebug_Write( 's' );
		madebug_Write( 'e' );
		madebug_Write( 'q' );
		madebug_Write( 'u' );
		madbg_info.sequ_ptr = madbg_info.ptr;
		madbg_info.ptr += 4;
		madbg_info.size += 4;

		madbg_info.init_status = 3;
	}

	for ( i = 0; i < size; i++ )
	{
		write_data = *ptr++;
		madebug_Write( write_data );
		madbg_info.sequ_size++;
	}

	return MASMW_SUCCESS;
}
/****************************************************************************
 *	madebug_SendDirectPacket
 *
 ****************************************************************************/
SINT32 madebug_SendDirectPacket
(
	const UINT8 * ptr,		/* pointer to packets */
	UINT16	size			/* byte size of packets */
)
{
	UINT16	i;
	UINT8	write_data;
	UINT32	address;
	UINT8	st = 0;
	UINT8	set_timer = 0;

	if ( madbg_info.init_status == 3 )
	{
		return MASMW_SUCCESS;
	}

	address = 0;
	for ( i = 0; i < 3; i++ )
	{
		address |= (UINT32)( ( (*(ptr+i)) & 0x7F ) << (i*7) );
		if ( ( *(ptr+i) & 0x80 ) != 0 )
		{
			break;
		}
	}
	if ( address > 328 ) return MASMW_SUCCESS;

	if ( madbg_info.init_status != 3 )
	{
		if ( madbg_info.init_ptr == NULL )
		{
			madebug_Write( 'i' );
			madebug_Write( 'n' );
			madebug_Write( 'i' );
			madebug_Write( 't' );
			madbg_info.init_ptr = madbg_info.ptr;
			madbg_info.ptr += 4;
			madbg_info.size += 4;

			if ( madbg_info.mode != 0 )
			{
				madebug_Write( 0x67 );
				madebug_Write( 0x82 );
				madebug_Write( 0x81 );
				madbg_info.init_size += 3;
			}

			madbg_info.init_status = 2;
		}
	}
	else
	{
		if ( madbg_info.sequ_ptr == NULL )
		{
			madebug_Write( 's' );
			madebug_Write( 'e' );
			madebug_Write( 'q' );
			madebug_Write( 'u' );
			madbg_info.sequ_ptr = madbg_info.ptr;
			madbg_info.ptr += 4;
			madbg_info.size += 4;

			madbg_info.init_status = 3;
		}
	}

	set_timer = 1;
	for ( i = 0; i < size; i++ )
	{
		if ( set_timer == 1 )
		{
			if ( madbg_info.init_status == 3 )
			{
				madebug_Write( 0x80 );
				madbg_info.sequ_size++;
			}
			set_timer = 0;
		}
		
		write_data = *ptr++;
		madebug_Write( write_data );
		if ( madbg_info.init_status == 2 )
			madbg_info.init_size++;
		else
			madbg_info.sequ_size++;

		if ( ( write_data & 0x80 ) != 0 )
		{
			st++;
			if ( st == 2 )
			{
				st = 0;
				set_timer = 1;
			}
		}
	}

	return MASMW_SUCCESS;
}
/****************************************************************************
 *	madebug_SendDirectRamData
 *
 ****************************************************************************/
SINT32 madebug_SendDirectRamData
(
	UINT32	address,
	UINT8	data_type,
	const UINT8 * data_ptr,
	UINT32	data_size
)
{
	UINT32	i;
	UINT8	packet_buffer[3+2];
	UINT32	count;
	UINT32	write_size;
	UINT8	write_data;
	UINT8	temp = 0;

	if ( madbg_info.init_status == 3 )
	{
		return MASMW_SUCCESS;
	}

	if ( address == MA_RAM_START_ADDRESS )
	{
		return MASMW_SUCCESS;
	}

	if ( madbg_info.init_ptr == NULL )
	{
		madebug_Write( 'i' );
		madebug_Write( 'n' );
		madebug_Write( 'i' );
		madebug_Write( 't' );
		madbg_info.init_ptr = madbg_info.ptr;
		madbg_info.ptr += 4;
		madbg_info.size += 4;

		if ( madbg_info.mode != 0 )
		{
			madebug_Write( 0x67 );
			madebug_Write( 0x82 );
			madebug_Write( 0x81 );
			madbg_info.init_size += 3;
		}

		madbg_info.init_status = 2;
	}

	if ( data_size == 0 ) return MASMW_SUCCESS;

	
	for ( i = 0; i < 2; i++ )
	{
		packet_buffer[i] = (UINT8)( (address >> (7*i)) & 0x7F );
	}
	packet_buffer[i] = (UINT8)( (address >> (7*i)) | 0x80 );

	if ( data_type == 0 )
	{
		write_size = data_size;
	}
	else
	{
		write_size = (data_size / 8) * 7 + (data_size % 8 - 1);
	}

	if		( write_size < 0x0080 )	count = 0;
	else							count = 1;

	for ( i = 0; i < count; i++ )
	{
		packet_buffer[3+i] = (UINT8)( (write_size >> (7*i)) & 0x7F );
	}
	packet_buffer[3+i] = (UINT8)( (write_size >> (7*i)) | 0x80 );
	
	for ( i = 0; i < (count+1+3); i++ )
	{
		madebug_Write( packet_buffer[i] );
		madbg_info.init_size++;
	}

	if ( data_type == 0 )
	{
		for ( i = 0; i < data_size; i++ )
		{
			madebug_Write( *data_ptr++ );
			madbg_info.init_size++;
		}
	}
	else
	{
		for ( i = 0; i < data_size; i++ )
		{
			if ( ( i % 8 ) == 0 )
			{
				temp = *data_ptr++;
			}
			else
			{
				write_data = *data_ptr++;
				write_data |= (UINT8)( ( temp << (i % 8) ) & 0x80 );

				madebug_Write( write_data );
				madbg_info.init_size++;
			}
		}
	}

	return MASMW_SUCCESS;
}
/****************************************************************************
 *	madebug_SendDirectRamVal
 *
 ****************************************************************************/
SINT32 madebug_SendDirectRamVal
(
	UINT32	address,
	UINT32	data_size,
	UINT8	val
)
{
	UINT32	i;
	UINT8	packet_buffer[3+2];
	UINT32	count;

	if ( madbg_info.init_status == 3 )
	{
		return MASMW_SUCCESS;
	}

	if ( madbg_info.init_ptr == NULL )
	{
		madebug_Write( 'i' );
		madebug_Write( 'n' );
		madebug_Write( 'i' );
		madebug_Write( 't' );
		madbg_info.init_ptr = madbg_info.ptr;
		madbg_info.ptr += 4;
		madbg_info.size += 4;

		if ( madbg_info.mode != 0 )
		{
			madebug_Write( 0x67 );
			madebug_Write( 0x82 );
			madebug_Write( 0x81 );
			madbg_info.init_size += 3;
		}

		madbg_info.init_status = 2;
	}

	
	for ( i = 0; i < 2; i++ )
	{
		packet_buffer[i] = (UINT8)( (address >> (7*i)) & 0x7F );
	}
	packet_buffer[i] = (UINT8)( (address >> (7*i)) | 0x80 );

	if		( data_size < 0x0080 )	count = 0;
	else							count = 1;

	for ( i = 0; i < count; i++ )
	{
		packet_buffer[3+i] = (UINT8)( (data_size >> (7*i)) & 0x7F );
	}
	packet_buffer[3+i] = (UINT8)( (data_size >> (7*i)) | 0x80 );

	for ( i = 0; i < (count+1+3); i++ )
	{
		madebug_Write( packet_buffer[i] );
		madbg_info.init_size++;
	}

	for ( i = 0; i < data_size; i++ )
	{
		madebug_Write( val );
		madbg_info.init_size++;
	}

	return MASMW_SUCCESS;
}
/****************************************************************************
 *	madebug_SetStream
 *
 ****************************************************************************/
SINT32 madebug_SetStream
(
	SINT32	seq_id,		/* sequence id number */
	UINT8	wave_id,	/* wave id number */
	UINT8	format,		/* wave format */
	UINT32	frequency,	/* sampling frequency */
	UINT8 *	wave_ptr,	/* pointer to the wave data */
	UINT32	wave_size	/* byte size of the wave data */
)
{
	(void)seq_id;

	if ( madbg_info.init_status > 1 )
	{
		return MASMW_SUCCESS;
	}

	if ( frequency != 0 )
	{
		if ( madbg_info.strm_ptr == NULL )
		{
			madebug_Write( 's' );
			madebug_Write( 't' );
			madebug_Write( 'r' );
			madebug_Write( 'm' );
			madbg_info.strm_ptr = madbg_info.ptr;
			madbg_info.strm_size += 4;
			madbg_info.ptr += 8;
			madbg_info.size += 8;

			madbg_info.init_status = 1;
		}

		madebug_Write( (UINT8)(format << 6) );
		madebug_Write( 0x00 );
		madebug_Write( 0x00 );
		madebug_Write( wave_id );
		madebug_Write( (UINT8)(wave_size >> 24) );
		madebug_Write( (UINT8)(wave_size >> 16) );
		madebug_Write( (UINT8)(wave_size >>  8) );
		madebug_Write( (UINT8)(wave_size >>  0) );

		memcpy( madbg_info.ptr, wave_ptr, wave_size );

		madbg_info.strm_num++;
		madbg_info.strm_size += (UINT32)(wave_size + 8);

		madbg_info.ptr  += wave_size;
		madbg_info.size += wave_size;
	}

	return MASMW_SUCCESS;
}
#else
static void madebug_dummy( void ) { }
#endif
