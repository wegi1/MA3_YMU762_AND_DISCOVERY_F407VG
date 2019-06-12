/****************************************************************************
 *																			*
 *		Copyright (C) 2001-2002	YAMAHA CORPORATION. All rights reserved.	*
 *																			*
 *		Module		: masmwmain.c											*
 *																			*
 *		Description	: MA Sound Middleware sample main.c						*
 *																			*
 * 		Version		: 1.3.8.0	2002.05.16									*
 *																			*
 ****************************************************************************/

#include "masound.h"					/* MA Sound Player API */
#include "maphrwrp.h"					/* Phrase Wrapper API */
#include "mamachdep.h"

#define USE_LOADINFO		(0)

volatile unsigned char _flag;

#if USE_LOADINFO
static unsigned char prof[128];
#endif

/****************************************************************************
 *	masmw_cbfunc
 *
 *	Description:
 *			callback function
 *	Argument:
 *			id			id (0..127)
 *	Return:
 *			0			success
 *
 ****************************************************************************/
signed long masmw_cbfunc( unsigned char id )
{
	switch ( id )
	{
	case MASMW_REPEAT:
		/* Repeat */
		_flag = 1;
		break;

	case MASMW_END_OF_SEQUENCE:
		/* End of sequence */
		_flag = 2;
		break;

	default:
		/* User event */
		break;
	}

	return MASMW_SUCCESS;
}
/****************************************************************************
 *	masmw_error_handler
 *
 *	Description:
 *			error handler
 *	Argument:
 *			func_id		function id
 *			file_id		file id 
 *	Return:
 *			None
 *
 ****************************************************************************/
static void masmw_error_handler
(
	signed long func_id,			/* function id */
	signed long	file_id				/* file id */
)
{
	signed long state;

	state = MaSound_Control( func_id, file_id, MASMW_GET_STATE, NULL, NULL );

	switch ( state )
	{
	case MASMW_STATE_PLAYING:
	case MASMW_STATE_PAUSE:
		MaSound_Stop( func_id, file_id, NULL );
		/* cotinue to below */

	case MASMW_STATE_READY:
	case MASMW_STATE_OPENED:
		MaSound_Close( func_id, file_id, NULL );
		/* cotinue to below */

	case MASMW_STATE_LOADED:
		MaSound_Unload( func_id, file_id, NULL );
		/* cotinue to below */

	case MASMW_STATE_IDLE:
		MaSound_Delete( func_id );
		/* cotinue to below */

	default:
		break;
	}
}
/****************************************************************************
 *	masmw_main
 *
 *	Description:
 *			Sample of playing the SMAF file using the MA Sound Middleware.
 *	Argument:
 *			file_ptr	pointer to the SMAF file
 *			file_size	size of the SMAF file
 *	Return:
 *			0		success
 *			-1		error
 *
 ****************************************************************************/
signed long masmw_main
(
	unsigned char *	file_ptr,			/* pointer to the SMAF file */
	unsigned long	file_size			/* size of the SMAF file */
)
{
	unsigned char volume;				/* volume */
	signed long	func_id;				/* function id */
	signed long	file_id;				/* file id */
	signed long	result;					/* result of function */

	_flag = 0;

	/* Initialize the MA Sound Middleware */
	result = MaSound_Initialize();
	if ( result < MASMW_SUCCESS )
	{
		return MASMW_ERROR;
	}

	/* Device Control */
	result = MaSound_DeviceControl( MASMW_EQ_VOLUME, 31, 0, 0 );
	if ( result < MASMW_SUCCESS )
	{
		return MASMW_ERROR;
	}

	result = MaSound_DeviceControl( MASMW_SP_VOLUME, 31, 0, 0 );
	if ( result < MASMW_SUCCESS )
	{
		return MASMW_ERROR;
	}

	result = MaSound_DeviceControl( MASMW_HP_VOLUME, 0, 31, 31 );
	if ( result < MASMW_SUCCESS )
	{
		return MASMW_ERROR;
	}

	/* Create SMAF Stream Converter */
	func_id = MaSound_Create( MASMW_CNVID_MMF );
	if ( func_id < MASMW_SUCCESS )
	{
		return MASMW_ERROR;
	}

#if USE_LOADINFO
	/* Check the SMAF file */
	file_id = MaSound_Load( func_id, file_ptr, file_size, 2, masmw_cbfunc, NULL );
	if ( file_id < MASMW_SUCCESS )
	{
		masmw_error_handler( func_id, file_id );
		return MASMW_ERROR;
	}

	/* get information of the SMAF file */
	result = MaSound_Control( func_id, file_id, MASMW_GET_LOADINFO, (void *)&prof[0], NULL );

	/* set information of the SMAF file */
	result = MaSound_Control( func_id, 0, MASMW_SET_LOADINFO, (void *)&prof[0], NULL );

	/* Load the SMAF file without error checking */
	file_id = MaSound_Load( func_id, file_ptr, file_size, 0, masmw_cbfunc, NULL );
	if ( file_id < MASMW_SUCCESS )
	{
		masmw_error_handler( func_id, file_id );
		return MASMW_ERROR;
	}
#else
	/* Load the SMAF file */
	file_id = MaSound_Load( func_id, file_ptr, file_size, 1, masmw_cbfunc, NULL );
	if ( file_id < MASMW_SUCCESS )
	{
		masmw_error_handler( func_id, file_id );
		return MASMW_ERROR;
	}
#endif

	/* Open the SMAF file */
	result = MaSound_Open( func_id, file_id, 0, NULL );
	if ( result < MASMW_SUCCESS )
	{
		masmw_error_handler( func_id, file_id );
		return MASMW_ERROR;
	}
	
	/* Set volume */
	volume = 127;
	result = MaSound_Control( func_id, file_id, MASMW_SET_VOLUME, &volume, NULL );
	if ( result < MASMW_SUCCESS )
	{
		masmw_error_handler( func_id, file_id );
		return MASMW_ERROR;
	}

	/* Standby for playing */
	result = MaSound_Standby( func_id, file_id, NULL );
	if ( result < MASMW_SUCCESS )
	{
		masmw_error_handler( func_id, file_id );
		return MASMW_ERROR;
	}

	/* Play */
	result = MaSound_Start( func_id, file_id, 1, NULL );
	if ( result < MASMW_SUCCESS )
	{
		masmw_error_handler( func_id, file_id );
		return MASMW_ERROR;
	}

	while ( _flag == 0 )
		;

	/* Stop */
	result = MaSound_Stop( func_id, file_id, NULL );
	if ( result < MASMW_SUCCESS )
	{
		masmw_error_handler( func_id, file_id );
		return MASMW_ERROR;
	}

	/* Close the SMAF file */
	result = MaSound_Close( func_id, file_id, NULL );
	if ( result < MASMW_SUCCESS )
	{
		masmw_error_handler( func_id, file_id );
		return MASMW_ERROR;
	}
	
	/* Unload the SMAF file */
	result = MaSound_Unload( func_id, file_id, NULL );
	if ( result < MASMW_SUCCESS )
	{
		masmw_error_handler( func_id, file_id );
		return MASMW_ERROR;
	}

	/* Delete the SMAF stream converter */
	result = MaSound_Delete( func_id );
	if ( result < MASMW_SUCCESS )
	{
		return MASMW_ERROR;
	}

	return MASMW_SUCCESS;
}
/****************************************************************************
 *	phrase_cbfunc
 *
 *	Description:
 *			callback function
 *	Argument:
 *			id			id (0..127)
 *	Return:
 *			0			success
 *
 ****************************************************************************/
static void phrase_cbfunc( struct event * eve )
{
	switch ( eve->mode )
	{
	case -1:
		/* End of play */
		_flag = 1;
		break;

	case -2:
		/* End of loop play */
		_flag = 2;
		break;

	default:
		/* User event */
		break;
	}
}
/****************************************************************************
 *	phrase_error_handler
 *
 *	Description:
 *			error handler
 *	Argument:
 *			ch			sequence number
 *	Return:
 *			None
 *
 ****************************************************************************/
static void phrase_error_handler
(
	signed long ch					/* channel number */
)
{
	int state;

	state = Phrase_GetStatus( ch );

	switch ( state & 0x0F )
	{
	case ID_STATUS_PAUSE:			/* pause */
	case ID_STATUS_PLAY:			/* play */
		Phrase_Stop( ch );
		/* cotinue to below */

	case ID_STATUS_READY:			/* ready */
		Phrase_RemoveData( ch );
		/* cotinue to below */

	case ID_STATUS_NODATA:			/* no data */
		Phrase_Terminate();
		/* cotinue to below */

	default:
		break;
	}
}
/****************************************************************************
 *	phrase_main
 *
 *	Description:
 *			Sample of playing the SMAF/Phrase files using Phrase Wrapper API.
 *	Argument:
 *			file_ptr	pointer to the SMAF/Phrase file
 *			file_size	size of the SMAF/Phrase file
 *	Return:
 *			0		success
 *			-1		error
 *
 ****************************************************************************/
signed long phrase_main
(
	unsigned char * file_ptr,			/* pointer to the SMAF/Phrase file */
	unsigned long 	file_size			/* size of the SMAF/Phrase file */
)
{
	int	ch;								/* channel number */
	signed long	result;					/* result of function */

	_flag = 0;

	/* Initialize the MA Sound Middleware */
	result = MaSound_Initialize();
	if ( result < MASMW_SUCCESS )
	{
		return MASMW_ERROR;
	}

	/* Device Control */
	result = MaSound_DeviceControl( MASMW_EQ_VOLUME, 31, 0, 0 );
	if ( result < MASMW_SUCCESS )
	{
		return MASMW_ERROR;
	}

	result = MaSound_DeviceControl( MASMW_SP_VOLUME, 31, 0, 0 );
	if ( result < MASMW_SUCCESS )
	{
		return MASMW_ERROR;
	}

	result = MaSound_DeviceControl( MASMW_HP_VOLUME, 0, 31, 31 );
	if ( result < MASMW_SUCCESS )
	{
		return MASMW_ERROR;
	}

	/* Initialize the SMAF/Phrase I/F */
	result = Phrase_Initialize();
	if ( result < MASMW_SUCCESS )
	{
		return MASMW_ERROR;
	}

	/* Set the callback */
	result = Phrase_SetEvHandler( phrase_cbfunc );
	if ( result < MASMW_SUCCESS )
	{
		phrase_error_handler( 0 );
		return MASMW_ERROR;
	}

	/* Use sequence number is 0 */
	ch = 0;

	/* Specify the SMAF/Phrase file for play */
	result = Phrase_SetData( ch, file_ptr, file_size, 1 );
	if ( result < MASMW_SUCCESS )
	{
		phrase_error_handler( ch );
		return MASMW_ERROR;
	}

	/* Play the SMAF/Phrase file */
	result = Phrase_Play( ch, 1 );
	if ( result < MASMW_SUCCESS )
	{
		phrase_error_handler( ch );
		return MASMW_ERROR;
	}

	while ( _flag == 0 )
		;

	/* Remove the SMAF/Phrase file */
	result = Phrase_RemoveData( ch );
	if ( result < MASMW_SUCCESS )
	{
		phrase_error_handler( ch );
		return MASMW_ERROR;
	}

	/* Terminate the SMAF/Phrase I/F */
	result = Phrase_Terminate();
	if ( result < MASMW_SUCCESS )
	{
		return MASMW_ERROR;
	}

	return MASMW_SUCCESS;
}
