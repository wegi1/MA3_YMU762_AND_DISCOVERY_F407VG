/****************************************************************************
 *																			*
 *		Copyright (C) 2001	YAMAHA CORPORATION. All rights reserved.		*
 *																			*
 *		Module		: mamachdep.c											*
 *																			*
 *		Description	: machine dependent part for MA SMW						*
 *																			*
 * 		Version		: 1.3.4.0	2001.12.14									*
 *																			*

Modefied by Kenny to support both MA-3 and  MA-3L2

 ****************************************************************************/

#include "mamachdep.h"
#include "madefs.h"

extern void delay_ms(volatile unsigned long int d);

/****************************************************************************
 *	machdep_memcpy
 *
 *
 ****************************************************************************/
void machdep_memcpy( UINT8 *dst_ptr, UINT8 *src_ptr, UINT32 size )
{
	while ( size-- != 0 )
	{
		*dst_ptr++ = *src_ptr++;
	}
}
/****************************************************************************
 *	machdep_Wait
 *
 *
 ****************************************************************************/
void machdep_Wait( UINT32 wait_time )
{
#if 1
  volatile UINT32 d=(wait_time*100)/1000; //ns
  while(d--);
#else
    HAL_Delay(1);
#endif
}
/****************************************************************************
 *	machdep_WriteStatusFlagReg
 *
 *	Function:
 *			Write a byte data to the status flag register.
 *	Arguments:
 *			data	byte data for write
 *	Return:
 *			None
 *
 ****************************************************************************/

UINT32 ulPos = 0;
UINT32 ulCount = 0;

#if 1
YMU262_LOG_ENTRY logbuf[LOG_ENTRIES_MAX];
#endif

#if 0
// initial implementation, needs some improvements
void AddToBuffer(int isRead, int isData, UINT8 bVal )
{
    UINT8 flag = (isRead ? 1 : 0) | (isData ? 2 : 0);

    // if repeat (e.g. read status - with the same data returned)
    if(((logbuf[ulPos].flag & 0x7F) == flag) && (logbuf[ulPos].data == bVal))
    {
        logbuf[ulPos].ushCnt = (UINT16) (ulCount & 0xFFFF);
        logbuf[ulPos].flag = flag | 128;    // mark repeat
    }
    else
    {
        logbuf[ulPos].ushCnt = (UINT16) (ulCount & 0xFFFF);
        logbuf[ulPos].flag = flag;
        logbuf[ulPos].data = bVal;
        ulPos++;
    }

    ulCount++;

    if(ulPos >= LOG_ENTRIES_MAX)
    {
        ulPos = 0;
    }
}
#else
void AddToBuffer(int isRead, int isData, UINT8 bVal )
{
    UINT8 flag = (isRead ? 1 : 0) | (isData ? 2 : 0);

    // if repeat (e.g. read status - with the same data returned)
    if(((logbuf[ulPos].flag & 0x7F) == flag) && (logbuf[ulPos].data == bVal))
    {
        logbuf[ulPos].ushCnt = (UINT16) (ulCount & 0xFFFF);
        logbuf[ulPos].flag = flag | 128;    // mark repeat
    }
    else
    {
        logbuf[ulPos].ushCnt = (UINT16) (ulCount & 0xFFFF);
        logbuf[ulPos].flag = flag;
        logbuf[ulPos].data = bVal;
        ulPos++;
    }

    ulCount++;

    if(ulPos >= LOG_ENTRIES_MAX)
    {
        ulPos = 0;
    }
}
#endif


void machdep_WriteStatusFlagReg( UINT8	data )
{
	/*----------------------------------------------------------------------*
	 *----------------------------------------------------------------------*/

	MA_STATUS_REG = data;

    AddToBuffer(0, 0, data);

	// YMU762C_MA-3L2
	machdep_Wait( 600 );
}
/****************************************************************************
 *	machdep_ReadStausFlagReg
 *
 *	Function:
 *			Read a byte data from the status flag register.
 *	Arguments:
 *			data	byte data for write
 *	Return:
 *			status register value
 *
 ****************************************************************************/
UINT8 machdep_ReadStatusFlagReg( void )
{
	UINT8	data;

	/*----------------------------------------------------------------------*
	 *----------------------------------------------------------------------*/

	data = MA_STATUS_REG;

    AddToBuffer(1, 0, data);

	machdep_Wait( 70 );

	return data;
}
/****************************************************************************
 *	machdep_WriteDataReg
 *
 *	Function:
 *			Write a byte data to the MA-3 write data register.
 *	Arguments:
			data	byte data for write
 *	Return:
 *			none
 *
 ****************************************************************************/
void machdep_WriteDataReg( UINT8 data )
{
	/*----------------------------------------------------------------------*
	 *----------------------------------------------------------------------*/

	MA_DATA_REG = data;

    AddToBuffer(0, 1, data);

	// YMU762C_MA-3L2
	machdep_Wait( 600 );

}
/****************************************************************************
 * machdep_ReadDataReg
 *
 *	Function:
 *			Read a byte data from the MA-3 read data register.
 *	Arguments:
			None
 *	Return:
 *			read data
 *
 ****************************************************************************/
UINT8 machdep_ReadDataReg( void )
{
	UINT8	data;

	/*----------------------------------------------------------------------*
	 *----------------------------------------------------------------------*/

	data = MA_DATA_REG;

    AddToBuffer(1, 1, data);

	machdep_Wait( 70 );

	return data;
}
/****************************************************************************
 *	machdep_CheckStauts
 *
 *	Function:
 *			Check status of the MA-3.
 *	Arguments:
			flag	check flag
 *	Return:
 *			0		success
 *			-1		time out
 *
 ****************************************************************************/
SINT32 machdep_CheckStatus( UINT8 flag )
{
	UINT8	read_data;

	do
	{
		read_data = machdep_ReadStatusFlagReg();

#if 0
		if ( /* time */ > MA_STATUS_TIMEOUT )
		{
			/* timeout: stop timer */
			return MASMW_ERROR;
		}
#endif

	}
	while ( ( read_data & flag ) != 0 );

	return MASMW_SUCCESS;
}
/****************************************************************************
 *	machdep_WaitValidData
 *
 *	Function:
 *			Check status of the MA-3.
 *	Arguments:
			flag	check flag
 *	Return:
 *			0		success
 *			-1		time out
 *
 ****************************************************************************/
SINT32 machdep_WaitValidData( UINT8 flag )
{
	UINT8	read_data;

	do
	{
		read_data = machdep_ReadStatusFlagReg();

#if 0
		if ( /* time */ > MA_STATUS_TIMEOUT )
		{
			/* timeout: stop timer */
			return MASMW_ERROR;
		}
#endif

	}
	while ( ( read_data & flag ) == 0 );

	return MASMW_SUCCESS;
}
/****************************************************************************
 *	machdep_Sleep
 *
 *
 ****************************************************************************/
void machdep_Sleep( UINT32 sleep_time )
{
 delay_ms(sleep_time); //ms
}
/****************************************************************************
 *	machdep_CheckDelayedFifoEmpty
 *
 *	Function:
 *			Check Delayed FIFO Empty Flag
 *	Arguments:

 *	Return:
 *			0		not Empty
 *			1		Empty
 *
 ****************************************************************************/
SINT32 machdep_CheckDelayedFifoEmpty( void )
{
	UINT32 flag;

	flag = 0;
	if( machdep_ReadStatusFlagReg()&MA_EMP_DW ) flag |= 0x01;
	if( machdep_ReadStatusFlagReg()&MA_EMP_DW ) flag |= 0x02;

	if( flag==0 ) return 0;
	else if( flag==0x03 ) return 1;

	flag = 0;
	if( machdep_ReadStatusFlagReg()&MA_EMP_DW ) flag |= 0x01;
	if( machdep_ReadStatusFlagReg()&MA_EMP_DW ) flag |= 0x02;
	if( flag==0 ) return 0;
	else return 1;
}

/****************************************************************************
 *	machdep_WaitDelayedFifoEmpty
 *
 *	Function:
 *			Wait untill Delayed FIFO Empty
 *	Arguments:

 *	Return:
 *			0		success
 *			-1		time out
 *
 ****************************************************************************/
SINT32 machdep_WaitDelayedFifoEmpty( void )
{
	UINT32 read_data;

	do
	{
		read_data = machdep_CheckDelayedFifoEmpty();

#if 0
		if ( /* time */ > MA_STATUS_TIMEOUT )
		{
			/* timeout: stop timer */
			return MASMW_ERROR;
		}
#endif

	}
	while ( read_data==0 );

	return MASMW_SUCCESS;
}

/****************************************************************************
 *	machdep_WaitImmediateFifoEmpty
 *
 *	Function:
 *			Wait untill Immediate FIFO Empty
 *	Arguments:

 *	Return:
 *			0		success
 *			-1		time out
 *
 ****************************************************************************/
SINT32 machdep_WaitImmediateFifoEmpty( void )
{
	UINT8 read_data;

	do
	{
		read_data = machdep_ReadStatusFlagReg();

#if 0
		if ( /* time */ > MA_STATUS_TIMEOUT )
		{
			/* timeout: stop timer */
			return MASMW_ERROR;
		}
#endif

	} while( (read_data&MA_EMP_W)==0 );

	return MASMW_SUCCESS;
}
