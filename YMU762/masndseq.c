/****************************************************************************
 *																			*
 *		Copyright (C) 2001-2003	YAMAHA CORPORATION. All rights reserved.	*
 *																			*
 *		Module		: masndseq.c											*
 *																			*
 *		Description	: MA Sound Sequencer 									*
 *																			*
 * 		Version		: 1.3.14.0	2003.01.17									*
 *																			*
 ****************************************************************************/

#include "masndseq.h"
#include "macnvprf.h"

extern UINT8 _ma_intstate;

extern void machdep_Sleep( UINT32 sleep_time );

/*--------------------------------------------------------------------------*/

static MASNDSEQINFO ma_sndseq_info;

static const UINT8 ma_seqtype[MASMW_NUM_SRMCNV] = { 0, 0, 1, 1, 2, 0, 0, 1, 2, 0, 1, 2 };

/*--------------------------------------------------------------------------*/

/* dummy functions */

static SINT32 dummy_Init   ( void )
{
	return MASMW_SUCCESS;
}
static SINT32 dummy_Load   ( UINT8 * file_ptr, UINT32 file_size, UINT8 mode, SINT32 (*func)(UINT8 id), void * ext_args )
{
	(void)file_ptr;
	(void)file_size;
	(void)mode;
	(void)func;
	(void)ext_args;
	return MASMW_SUCCESS;
}
static SINT32 dummy_Open   ( SINT32 file_id, UINT16 open_mode, void * ext_args )
{
	(void)file_id;
	(void)open_mode;
	(void)ext_args;
	return 0;
}
static SINT32 dummy_Control( SINT32 file_id, UINT8 ctrl_num, void * prm, void * ext_args )
{
	(void)file_id;
	(void)ctrl_num;
	(void)prm;
	(void)ext_args;
	return MASMW_SUCCESS;
}
static SINT32 dummy_Standby( SINT32 file_id, void * ext_args )
{
	(void)file_id;
	(void)ext_args;
	return MASMW_SUCCESS;
}
static SINT32 dummy_Seek   ( SINT32 file_id, UINT32 pos, UINT8 flag, void * ext_args )
{
	(void)file_id;
	(void)pos;
	(void)flag;
	(void)ext_args;
	return MASMW_SUCCESS;
}
static SINT32 dummy_Start  ( SINT32 file_id, void * ext_args )
{
	(void)file_id;
	(void)ext_args;
	return MASMW_SUCCESS;
}
static SINT32 dummy_Stop   ( SINT32 file_id, void * ext_args )
{
	(void)file_id;
	(void)ext_args;
	return MASMW_SUCCESS;
}
static SINT32 dummy_Close  ( SINT32 file_id, void * ext_args )
{
	(void)file_id;
	(void)ext_args;
	return MASMW_SUCCESS;
}
static SINT32 dummy_Unload ( SINT32 file_id, void * ext_args )
{
	(void)file_id;
	(void)ext_args;
	return MASMW_SUCCESS;
}
static SINT32 dummy_End    ( void )
{
	return MASMW_SUCCESS;
}

/*--------------------------------------------------------------------------*/

/****************************************************************************
 *	MaSound_ReceiveMessage
 *
 *	Function:
 *			Receive the message from MA Sound Driver.
 *	Argument:
 *			id		id number
 *			file_id	file id 
 *			message	message type
 *	Return:
 *			0		success
 *			< 0		error code
 *
 ****************************************************************************/
SINT32 MaSound_ReceiveMessage
(
	SINT32	seq_id,						/* sequence id */
	SINT32	file_id,					/* file id */
	UINT8	message						/* message number */
)
{
	SINT32	func_id;					/* function id */
	SINT32	result;						/* result of function */

	MASNDSEQ_DBGMSG(("MaSndSeq_ReceiveMessage: id=%ld fid=%ld msg=%d\n", seq_id, file_id, message));

	func_id = ma_sndseq_info.func_id[seq_id];
	if ( seq_id != 1 )
		file_id = ma_sndseq_info.file_id[seq_id];

	switch ( message )
	{
	case MASMW_END_OF_DATA:


		if ( ma_sndseq_info.play_mode[seq_id][file_id] == 0xFFFF )
		{
			return MASMW_END_OF_SEQUENCE;
		}

		if ( ma_sndseq_info.play_mode[seq_id][file_id] != 0 )
		{
			ma_sndseq_info.play_mode[seq_id][file_id]--;
			if ( ma_sndseq_info.play_mode[seq_id][file_id] == 0 )
			{
				ma_sndseq_info.play_mode[seq_id][file_id] = 0xFFFF;

				return MASMW_END_OF_SEQUENCE;
			}
		}

		MaSndDrv_ControlSequencer( seq_id, 2 );

		if ( seq_id == 2 )
		{
			result = ma_srmcnv_func[func_id].Stop( file_id, (void *)(-1) );
		}
		else
		{
			result = ma_srmcnv_func[func_id].Stop( file_id, NULL );
		}
		if ( result < MASMW_SUCCESS ) return result;
		
		result = ma_srmcnv_func[func_id].Seek( file_id, ma_sndseq_info.start_point[func_id][file_id], 0, NULL );
		if ( result < MASMW_SUCCESS ) return result;

		result = ma_srmcnv_func[func_id].Start( file_id, ma_sndseq_info.start_extargs[func_id][file_id] );
		if ( result < MASMW_SUCCESS ) return result;

		MaSndDrv_ControlSequencer( seq_id, 3 );

		break;

	case MASMW_END_OF_SEQUENCE:


		if ( ma_sndseq_info.loop_count[seq_id][file_id] == 0xFFFF )
		{
			message = MASMW_END_OF_SEQUENCE;
		}
		else if ( ma_sndseq_info.loop_count[seq_id][file_id] != 0 )
		{
			ma_sndseq_info.loop_count[seq_id][file_id]--;
			if ( ma_sndseq_info.loop_count[seq_id][file_id] == 0 )
			{
				ma_sndseq_info.loop_count[seq_id][file_id] = 0xFFFF;
				ma_sndseq_info.save_mode[seq_id][file_id] = 0xFFFF;

				if ( seq_id == 0 )
				{
					result = ma_srmcnv_func[func_id].Stop( file_id, NULL );
					if ( result < MASMW_SUCCESS ) return result;
				}
				else if ( seq_id == 2 )
				{
					result = ma_srmcnv_func[func_id].Stop( file_id, (void *)(-1) );
					if ( result < MASMW_SUCCESS ) return result;
				}

				ma_sndseq_info.state[func_id][file_id] = MASMW_STATE_READY;

				message = MASMW_END_OF_SEQUENCE;
			}
			else
			{
				ma_sndseq_info.seek_point[func_id][file_id] =
					ma_sndseq_info.start_point[func_id][file_id]
				  + ma_sndseq_info.seek_pos0[func_id][file_id];
				MaSndDrv_UpdatePos( seq_id, 0 );
				message = MASMW_REPEAT;
			}
		}
		else
		{
			ma_sndseq_info.seek_point[func_id][file_id] =
				ma_sndseq_info.start_point[func_id][file_id]
			  + ma_sndseq_info.seek_pos0[func_id][file_id];
			MaSndDrv_UpdatePos( seq_id, 0 );
			message = MASMW_REPEAT;
		}
	
	
	default:

		if ( ma_sndseq_info.clback_func[func_id][file_id] != NULL )
		{
			ma_sndseq_info.clback_func[func_id][file_id]( message );
		}
		break;
	}
	
	return MASMW_SUCCESS;
}
/****************************************************************************
 *	MaSound_Create
 *
 *	Function:
 *			Setting the MA Stream Converter and callback function.
 *	Argument:
 *			cnv_id		ID number of MA Stream Converter
 *	Return:
 *			>= 0		function id of registered MA Stream Converter
 *			< 0			error code
 *
 ****************************************************************************/
SINT32 MaSound_Create
(
	UINT8	cnv_id						/* stream converter id */
)
{
	UINT32	i;							/* loop counter */
	SINT32	result;						/* result of function */

	MASNDSEQ_DBGMSG(("MaSound_Create: cnv_id=%d\n", cnv_id));

	if ( cnv_id >= MASMW_NUM_SRMCNV )
	{
		return MASMW_ERROR;
	}

	if ( ( ma_sndseq_info.srmcnv_map & (UINT32)(1 << cnv_id) ) != 0 )
	{
		return MASMW_ERROR;
	}

	result = ma_srmcnv_func[cnv_id].Init();

	if ( result < MASMW_SUCCESS )
	{
		MASNDSEQ_DBGMSG(("Can't initialize the stream converter\n"));
		return result;
	}

	for ( i = 0; i < MASMW_NUM_FILE; i++ )
	{
		ma_sndseq_info.state[cnv_id][i] = MASMW_STATE_IDLE;
	}

	ma_sndseq_info.srmcnv_map |= (UINT32)(1 << cnv_id);

	return (SINT32)cnv_id;
}
/****************************************************************************
 *	MaSound_Load
 *
 *	Function:
 *			Performs loading processing of Stream Converter.
 *	Arguments:
 *			func_id		function id
 *			file_ptr	pointer to file
 *			file_size	byte size of file
 *			mode		error check mode	0: no check
 *											1: check
 *											2: check only
 *											3: get contents information
 *			func		callback function
 *			ext_args	for future extension
 *	Return:
 *			>= 0		success. file id.
 *			< 0			error code
 *
 ****************************************************************************/
SINT32 MaSound_Load
(
	SINT32	func_id, 					/* function id */
	UINT8 *	file_ptr,					/* pointer to file */
	UINT32	file_size,					/* size of file */
	UINT8	mode,						/* error check mode */
	SINT32	(*func)(UINT8 id),			/* callback function */
	void *	ext_args					/* for future extension */
)
{
	SINT32	result;						/* result of function */
	
	MASNDSEQ_DBGMSG(("MaSound_Load: id=%ld\n", func_id));

	if ( ( ma_sndseq_info.srmcnv_map & (UINT32)(1 << func_id) ) == 0 )
	{
		return MASMW_ERROR;
	}

	result = ma_srmcnv_func[func_id].Load( file_ptr, file_size, mode, func, ext_args );

	if ( result >= MASMW_SUCCESS )
	{
		if ( result == 0 )
		{
			return result;
		}

		ma_sndseq_info.state[func_id][result] = MASMW_STATE_LOADED;

		ma_sndseq_info.clback_func[func_id][result] = func;
	}
	
	return result;
}
/****************************************************************************
 *	MaSound_Open
 *
 *	Function:
 *			Performs opening processing of Stream Converter.
 *	Arguments:
 *			func_id		function id
 *			file_id		file id
 *			open_mode	file open mode
 *			ext_args	for future extension
 *	Return:
 *			0			success
 *			< 0 		error code
 *
 ****************************************************************************/
SINT32 MaSound_Open
(
	SINT32	func_id,					/* function id */
	SINT32	file_id,					/* file id */
	UINT16	open_mode,					/* open mode */
	void *	ext_args					/* for future extension */
)
{
	SINT32	result;						/* result of function */
	SINT32	result2;					/* result of function */
	
	MASNDSEQ_DBGMSG(("MaSound_Open: id=%ld hd=%ld md=%hd\n", func_id, file_id, open_mode));

	if ( ( ma_sndseq_info.srmcnv_map & (UINT32)(1 << func_id) ) == 0 )
	{
		return MASMW_ERROR;
	}

	if ( ma_sndseq_info.state[func_id][file_id] != MASMW_STATE_LOADED )
	{
		return MASMW_ERROR;
	}

	result = ma_srmcnv_func[func_id].Open( file_id, open_mode, ext_args );

	if ( result >= MASMW_SUCCESS )
	{
		result = ma_srmcnv_func[func_id].Control( file_id, MASMW_GET_LENGTH, NULL, NULL );
		if ( result < MASMW_SUCCESS )
		{
			return result;
		}

		result2 = ma_srmcnv_func[func_id].Control( file_id, MASMW_GET_LENGTH, NULL, (void *)(-1) );
		if ( result2 < MASMW_SUCCESS )
		{
			return result2;
		}

		ma_sndseq_info.start_point[func_id][file_id] = 0;
		ma_sndseq_info.end_point[func_id][file_id]   = (UINT32)result;

		ma_sndseq_info.seek_point[func_id][file_id]  = 0;

		ma_sndseq_info.play_length[func_id][file_id] = (UINT32)result;
		ma_sndseq_info.loop_length[func_id][file_id] = (UINT32)result2;

		ma_sndseq_info.state[func_id][file_id] = MASMW_STATE_OPENED;

		result = MASMW_SUCCESS;
	}

	return result;
}
/****************************************************************************
 *	MaSound_Control
 *
 *	Function:
 *			Performs controlling processing of Stream Conveter.
 *	Argument:
 *			func_id		function id
 *			file_id		file id
 *			ctrl_num	control number
 *			prm			control parameter
 *			ext_args	for future extension
 *	Return:
 *			0			success
 *			< 0			error code
 *
 ****************************************************************************/
SINT32 MaSound_Control
(
	SINT32	func_id,					/* function id */
	SINT32	file_id,					/* file id */
	UINT8	ctrl_num,					/* control number */
	void *	prm,						/* parameter */
	void *	ext_args					/* for future extension */
)
{
	UINT32	start_point;				/* start point */
	UINT32	end_point;					/* end point */
	UINT32	position;					/* position */
	UINT32	play_length;				/* play length */
	SINT32	result;						/* result of function */
	UINT8	seq_type;
	UINT32	dPos;

	MASNDSEQ_DBGMSG(("MaSound_Control: id=%ld hd=%ld cn=%d\n", func_id, file_id, ctrl_num));

	if ( ( ma_sndseq_info.srmcnv_map & (UINT32)(1 << func_id) ) == 0 )
	{
		return MASMW_ERROR;
	}

	switch ( ctrl_num )
	{
	case MASMW_GET_STATE:
	
		return (UINT32)ma_sndseq_info.state[func_id][file_id];
	 /*	break; */

	case MASMW_GET_CONTENTSDATA:
	
		if ( ( ma_sndseq_info.state[func_id][file_id] < MASMW_STATE_LOADED )
		  && ( file_id != 0 ) )
		{
			return MASMW_ERROR;
		}
		if ( ((PMASMW_CONTENTSINFO)prm)->size == 0 )
		{
			return MASMW_ERROR_ARGUMENT;
		}
		break;
	
	case MASMW_GET_PHRASELIST:

		if ( ( ma_sndseq_info.state[func_id][file_id] < MASMW_STATE_LOADED )
		  && ( file_id != 0 ) )
		{
			return MASMW_ERROR;
		}
		break;

	case MASMW_SET_STARTPOINT:

		if ( ma_sndseq_info.state[func_id][file_id] != MASMW_STATE_OPENED )
		{
			return MASMW_ERROR;
		}

		start_point = *((UINT32 *)prm);
		play_length = ma_sndseq_info.play_length[func_id][file_id];
		if ( start_point > play_length )
		{
			return MASMW_ERROR;
		}
		else
		{
			ma_sndseq_info.start_point[func_id][file_id] = start_point;
			ma_sndseq_info.seek_point[func_id][file_id]  = start_point;
#if MASMW_SRMCNV_MUL
			if (func_id == MASMW_CNVID_MUL)
				break;
			else
				return MASMW_SUCCESS;
#else
			return MASMW_SUCCESS;
#endif
		}

	 /*	break; */

	case MASMW_SET_ENDPOINT:

		if ( ma_sndseq_info.state[func_id][file_id] != MASMW_STATE_OPENED )
		{
			return MASMW_ERROR;
		}

		end_point = *((UINT32 *)prm);
		play_length = ma_sndseq_info.play_length[func_id][file_id];
		if ( end_point > play_length )
		{
			return MASMW_ERROR;
		}
		else
		{
			ma_sndseq_info.end_point[func_id][file_id] = end_point;
		}

		break;

	case MASMW_GET_POSITION:

		if ( ma_sndseq_info.state[func_id][file_id] < MASMW_STATE_OPENED )
		{
			return MASMW_ERROR;
		}

		if ( ma_seqtype[func_id] == 0 )
		{
			result = ma_srmcnv_func[func_id].Control( file_id, ctrl_num, prm, ext_args );
			if ( result >= 0 )
			{
				position = ma_sndseq_info.seek_point[func_id][file_id] + result;
				play_length = ma_sndseq_info.loop_length[func_id][file_id];

				if ( (play_length != 0 ) && ( position > play_length ) )
					return play_length;
				else
					return position;
			}
			else
			{
				return result;
			}
		}

		break;

	case MASMW_SET_SPEEDWIDE:
	case MASMW_SET_VOLUME:
	case MASMW_SET_SPEED:
	case MASMW_SET_KEYCONTROL:
	case MASMW_GET_TIMEERROR:
	case MASMW_SEND_MIDIMSG:
	case MASMW_SEND_SYSEXMIDIMSG:
	case MASMW_SET_BIND:
	case MASMW_SET_PANPOT:
	case MASMW_GET_LEDSTATUS:
	case MASMW_GET_VIBSTATUS:
	case MASMW_SET_EVENTNOTE:
	case MASMW_GET_CONVERTTIME:
		if ( ma_sndseq_info.state[func_id][file_id] < MASMW_STATE_OPENED )
		{
			return MASMW_ERROR;
		}
		break;

	case MASMW_GET_LOADINFO:
		/* get file information */
		break;

	case MASMW_SET_LOADINFO:
		/* set file information */
		break;

	case MASMW_GET_LENGTH:
	
		if ( ( ma_sndseq_info.state[func_id][file_id] < MASMW_STATE_LOADED )
		  && ( file_id != 0 ) )
		{
			return MASMW_ERROR;
		}
		break;

	case MASMW_SET_REPEAT:
	
		seq_type = ma_seqtype[func_id];
		if ((( ma_sndseq_info.state[func_id][file_id] != MASMW_STATE_READY ) &&
			 ( ma_sndseq_info.state[func_id][file_id] != MASMW_STATE_PAUSE ))
		  || ( seq_type == 1 ) )
		{
			return MASMW_ERROR;
		}
		ma_sndseq_info.repeat_mode[seq_type][file_id] = (UINT16)*((UINT8*)prm);
		ma_sndseq_info.play_mode[seq_type][file_id] = (UINT16)*((UINT8*)prm);
		ma_sndseq_info.loop_count[seq_type][file_id] = (UINT16)*((UINT8*)prm);
		dPos = (seq_type == 2) ? 0 : MaSound_Control(func_id, file_id, MASMW_GET_POSITION, NULL, NULL);
		return (MaSound_Seek(func_id, file_id, dPos, 0, NULL));

	case MASMW_GET_CONTROL_VAL:
		if ( ma_sndseq_info.state[func_id][file_id] < MASMW_STATE_READY )
		{
			return MASMW_ERROR;
		}
		break;

	case MASMW_SET_CB_INTERVAL:
		if ( ma_sndseq_info.state[func_id][file_id] != MASMW_STATE_LOADED )
		{
			return MASMW_ERROR;
		}
		break;

	default:
		return MASMW_ERROR;
	}

	result = ma_srmcnv_func[func_id].Control( file_id, ctrl_num, prm, ext_args );


	return result;
}
/****************************************************************************
 *	MaSound_Standby
 *
 *	Function:
 *			Performs standby processing of Stream Converter.
 *	Argument:
 *			func_id		function id
 *			file_id		file id
 *			ext_args	for future extension
 *	Return:
 *			0			success
 *			< 0			error code
 *
 ****************************************************************************/
SINT32 MaSound_Standby
(
	SINT32	func_id,					/* function id */
	SINT32	file_id,					/* file id */
	void *	ext_args					/* for future extension */
)
{
	UINT8	seq_type;					/* type of sequence */
	UINT32	pos;						/* position */
	SINT32	result;						/* result of function */
	SINT32	result2;

	MASNDSEQ_DBGMSG(("MaSound_Standby: id=%ld hd=%ld\n", func_id, file_id));

	if ( ( ma_sndseq_info.srmcnv_map & (UINT32)(1 << func_id) ) == 0 )
	{
		return MASMW_ERROR;
	}

	if ( ma_sndseq_info.state[func_id][file_id] != MASMW_STATE_OPENED )
	{
		return MASMW_ERROR;
	}

	result = ma_srmcnv_func[func_id].Standby( file_id, ext_args );

	if ( result >= MASMW_SUCCESS )
	{
		seq_type = ma_seqtype[func_id];
		ma_sndseq_info.repeat_mode[seq_type][file_id] = 0xFFFF ;
		ma_sndseq_info.play_mode[seq_type][file_id]  = 0xFFFF;
		ma_sndseq_info.save_mode[seq_type][file_id]  = 0xFFFF;
		ma_sndseq_info.loop_count[seq_type][file_id] = 0xFFFF;

		pos = ma_sndseq_info.start_point[func_id][file_id];
		result2 = ma_srmcnv_func[func_id].Seek( file_id, pos, 0, ext_args );
		if ( result2 < MASMW_SUCCESS )
		{
			return result2;
		}

		ma_sndseq_info.seek_pos0[func_id][file_id]  = (UINT32)result2;
		ma_sndseq_info.seek_point[func_id][file_id] = (UINT32)(pos + (UINT32)result2);

		ma_sndseq_info.state[func_id][file_id] = MASMW_STATE_READY;
	}

	return result;
}
/****************************************************************************
 *	MaSound_Seek
 *
 *	Function:
 *			Performs seeking processing of Stream Converter.
 *	Argument:
 *			func_id		function id
 *			file_id		file id
 *			pos			playing position from top or start point [ms]
 *			flag		wait(0: wait without before play, 1: wait with before play)
 *			ext_args	for future extension
 *	Return:
 *			0			success
 *			< 0 		error code
 *
 ****************************************************************************/
SINT32 MaSound_Seek
(
	SINT32	func_id,					/* function id */
	SINT32	file_id,					/* file id */
	UINT32	pos,						/* start position (msec) */
	UINT8	flag,						/* flag */
	void *	ext_args					/* for future extension */
)
{
	UINT8	seq_type;					/* type of sequence */
	UINT16	loop_count;					/* loop count */
	UINT32	start_point;				/* start point */
	UINT32	end_point;					/* end point */
	UINT32	play_length;				/* play length */
	SINT32	result;						/* result of function */
	
	MASNDSEQ_DBGMSG(("MaSound_Seek: id=%ld hd=%ld pos=%ld fg=%d\n", func_id, file_id, pos, flag));

	if ( ( ma_sndseq_info.srmcnv_map & (UINT32)(1 << func_id) ) == 0 )
	{
		return MASMW_ERROR;
	}

	if ( ( ma_sndseq_info.state[func_id][file_id] != MASMW_STATE_READY )
	  && ( ma_sndseq_info.state[func_id][file_id] != MASMW_STATE_PAUSE ) )
	{
		return MASMW_ERROR;
	}

	start_point = ma_sndseq_info.start_point[func_id][file_id];
	end_point   = ma_sndseq_info.end_point[func_id][file_id];
	play_length = ma_sndseq_info.play_length[func_id][file_id];

	pos += start_point;

	if ( ( pos > end_point ) || ( pos > play_length ) )
	{
		return MASMW_ERROR;
	}

	result = ma_srmcnv_func[func_id].Seek( file_id, pos, flag, ext_args );

	if ( result >= MASMW_SUCCESS )
	{
		ma_sndseq_info.seek_point[func_id][file_id] = (UINT32)(pos + (UINT32)result);

		seq_type = ma_seqtype[func_id];
		
		if ( seq_type != 1 )
		{
			loop_count = ma_sndseq_info.loop_count[seq_type][file_id];
			ma_sndseq_info.play_mode[seq_type][file_id] = loop_count;
		}
		
		result = MASMW_SUCCESS;
	}


	return result;
}
/****************************************************************************
 *	MaSound_Start
 *
 *	Function:
 *			Performs starting process of Stream Converter.
 *	Argument:
 *			func_id		function id
 *			file_id		file id
 *			play_mode	playing mode
 *			ext_args	for future extension
 *	Return:
 *			0			success
 *			< 0 		error code
 *
 ****************************************************************************/
SINT32 MaSound_Start
(
	SINT32	func_id,					/* function id */
	SINT32	file_id,					/* file id */
	UINT16	play_mode,					/* playing mode */
	void * 	ext_args					/* for future extension */
)
{
	UINT8	seq_type;					/* type of sequence */
	UINT16	save_play_mode = 0;
	SINT32	save_func_id = 0;
	SINT32	save_file_id = 0;
	UINT32	start_point;				/* start point */
	UINT32	end_point;					/* end point */
	SINT32	result;						/* result of function */
	
	MASNDSEQ_DBGMSG(("MaSound_Start: id=%ld hd=%ld pm=%d\n", func_id, file_id, play_mode));

	if ( ( ma_sndseq_info.srmcnv_map & (UINT32)(1 << func_id) ) == 0 )
	{
		return MASMW_ERROR;
	}

	if ( ma_sndseq_info.state[func_id][file_id] != MASMW_STATE_READY )
	{
		return MASMW_ERROR;
	}

	start_point = ma_sndseq_info.start_point[func_id][file_id];
	end_point   = ma_sndseq_info.end_point[func_id][file_id];
	if ( start_point >= end_point )
	{
		return MASMW_ERROR;
	}

	if ( (end_point - start_point) <= MA_MIN_LENGTH )
	{
		return MASMW_ERROR;
	}

	seq_type = ma_seqtype[func_id];

#if MA_PLAYMODE_CHECK
	if ( ( ma_sndseq_info.save_mode[seq_type][file_id] != 0xFFFF ) &&
		 ( ma_sndseq_info.save_mode[seq_type][file_id] != play_mode ) &&
		 ( ma_sndseq_info.repeat_mode[seq_type][file_id] == 0xFFFF ) )
	{
		return MASMW_ERROR;
	}
#endif

	save_func_id = ma_sndseq_info.func_id[seq_type];
	save_file_id = ma_sndseq_info.file_id[seq_type];
	save_play_mode = play_mode;

	ma_sndseq_info.func_id[seq_type] = func_id;
	ma_sndseq_info.file_id[seq_type] = file_id;

	ma_sndseq_info.state[func_id][file_id] = MASMW_STATE_PLAYING;

	ma_sndseq_info.start_extargs[func_id][file_id] = ext_args;

	if ( seq_type != 1 )
	{
		if ( ( ma_sndseq_info.play_mode[seq_type][file_id] == 0xFFFF )
		  && ( ma_sndseq_info.loop_count[seq_type][file_id] == 0xFFFF ) )
		{
			if (ma_sndseq_info.repeat_mode[seq_type][file_id] != 0xFFFF)
				play_mode = ma_sndseq_info.repeat_mode[seq_type][file_id];
			ma_sndseq_info.play_mode[seq_type][file_id] = play_mode;
			ma_sndseq_info.save_mode[seq_type][file_id] = play_mode;
			ma_sndseq_info.loop_count[seq_type][file_id] = play_mode;
		}
		result = ma_srmcnv_func[func_id].Start( file_id, ext_args );
	}
	else
	{
		ma_sndseq_info.play_mode[seq_type][file_id] = play_mode;
		ma_sndseq_info.save_mode[seq_type][file_id] = play_mode;
		ma_sndseq_info.loop_count[seq_type][file_id] = 1;
		result = ma_srmcnv_func[func_id].Start( file_id, &save_play_mode );
	}

	if ( result < MASMW_SUCCESS )
	{
		ma_sndseq_info.state[func_id][file_id] = MASMW_STATE_READY;

		ma_sndseq_info.func_id[seq_type] = save_func_id;
		ma_sndseq_info.file_id[seq_type] = save_file_id;
	}

	return result;
}
/****************************************************************************
 *	MaSound_Pause
 *
 *	Function:
 *			Stops sequence data reproduction temporarily.
 *			For audio systems, this operation is same as Stop.
 *	Argument:
 *			func_id		function id
 *			file_id		file id
 *			ext_args	for future extenstion
 *	Return:
 *			0			success
 *			< 0			error code
 *
 ****************************************************************************/
SINT32 MaSound_Pause
(
	SINT32	func_id,					/* function id */
	SINT32	file_id,					/* file id */
	void * 	ext_args					/* for future extension */
)
{
	SINT32	result;						/* result of function */
	
	MASNDSEQ_DBGMSG(("MaSound_Pause: id=%ld hd=%ld\n", func_id, file_id));

	if ( ( ma_sndseq_info.srmcnv_map & (UINT32)(1 << func_id) ) == 0 )
	{
		return MASMW_ERROR;
	}

	if ( ma_sndseq_info.state[func_id][file_id] != MASMW_STATE_PLAYING )
	{
		return MASMW_ERROR;
	}

	result = ma_srmcnv_func[func_id].Stop( file_id, ext_args );

	if ( result >= MASMW_SUCCESS )
	{
		ma_sndseq_info.state[func_id][file_id] = MASMW_STATE_PAUSE;
	}

	return result;
}
/****************************************************************************
 *	MaSound_Restart
 *
 *	Function:
 *			Cancels pause of sequence data reproduction.
 *			For audio systems, this operation is the same as Start.
 *	Argument:
 *			func_id		function id
 *			file_id		file id
 *			ext_args	for future extension
 *	Return:
 *			0			success
 *			< 0			error code
 *
 ****************************************************************************/
SINT32 MaSound_Restart
(
	SINT32	func_id,					/* function id */
	SINT32	file_id,					/* file id */
	void * 	ext_args					/* for future extension */
)
{
	UINT8	seq_type;					/* type of sequence */
	UINT16	mode;						/* mode */
	SINT32	result;						/* result of function */

	MASNDSEQ_DBGMSG(("MaSound_Restart: id=%ld, hd=%ld\n", func_id, file_id));

	if ( ( ma_sndseq_info.srmcnv_map & (UINT32)(1 << func_id) ) == 0 )
	{
		return MASMW_ERROR;
	}

	if ( ma_sndseq_info.state[func_id][file_id] != MASMW_STATE_PAUSE )
	{
		return MASMW_ERROR;
	}

	ma_sndseq_info.state[func_id][file_id] = MASMW_STATE_PLAYING;


	seq_type = ma_seqtype[func_id];
	if ( seq_type != 1 )
	{
		ma_sndseq_info.start_extargs[func_id][file_id] = ext_args;

		result = ma_srmcnv_func[func_id].Start( file_id, ext_args );
	}
	else
	{
		mode = 0xFFFF;
		result = ma_srmcnv_func[func_id].Start( file_id, &mode );
	}

	if ( result < MASMW_SUCCESS )
	{
		ma_sndseq_info.state[func_id][file_id] = MASMW_STATE_PAUSE;
	}

	return result;
}
/****************************************************************************
 *	MaSound_Stop
 *
 *	Function:
 *			Performs stopping processing of Stream Converter.
 *	Argument:
 *			func_id		function id
 *			file_id		file id
 *			ext_args	for future extension
 *	Return:
 *			0			success
 *			< 0 		error code
 *
 ****************************************************************************/
SINT32 MaSound_Stop
(
	SINT32	func_id,					/* function id */
	SINT32	file_id,					/* file id */
	void *	ext_args					/* for future extension */
)
{
#if MA_STOPWAIT
	SINT32	time_out;					/* time out value */
#endif
	SINT32	result;						/* result of function */
	
	MASNDSEQ_DBGMSG(("MaSound_Stop: id=%ld hd=%ld\n", func_id, file_id));
	if ( ( ma_sndseq_info.srmcnv_map & (UINT32)(1 << func_id) ) == 0 )
	{
		return MASMW_ERROR;
	}

	if ( ma_sndseq_info.state[func_id][file_id] == MASMW_STATE_READY )
	{
		return MASMW_SUCCESS;
	}

	if ( ( ma_sndseq_info.state[func_id][file_id] != MASMW_STATE_PLAYING )
	  && ( ma_sndseq_info.state[func_id][file_id] != MASMW_STATE_PAUSE ) )
	{
		return MASMW_ERROR;
	}

	if ( ma_sndseq_info.state[func_id][file_id] == MASMW_STATE_PLAYING )
	{
		result = ma_srmcnv_func[func_id].Stop( file_id, ext_args );
	}
	else
	{
		result = MASMW_SUCCESS;
	}

	if ( result >= MASMW_SUCCESS )
	{
		ma_sndseq_info.state[func_id][file_id] = MASMW_STATE_READY;
	}

#if MA_STOPWAIT
	time_out = MA_STOPWAIT_TIMEOUT;
	while ( ( _ma_intstate != 0 ) && ( time_out > 0 ) )
	{
		machdep_Sleep(5);
		time_out -= 5;
	}
#endif

	return result;
}
/****************************************************************************
 *	MaSound_Close
 *
 *	Function:
 *			Performs closing processing of Stream Converter.
 *	Argument:
 *			func_id		function id
 *			file_id		file id
 *			ext_args	for future extension
 *	Return:
 *			0			success
 *			< 0			error code
 *
 ****************************************************************************/
SINT32 MaSound_Close
(
	SINT32	func_id,					/* function id */
	SINT32	file_id,					/* file id */
	void *	ext_args					/* for future extension */
)
{
	SINT32	result;						/* result of function */

	MASNDSEQ_DBGMSG(("MaSound_Close: id=%ld hd=%ld\n", func_id, file_id));

	if ( ( ma_sndseq_info.srmcnv_map & (UINT32)(1 << func_id) ) == 0 )
	{
		return MASMW_ERROR;
	}

	if ( ( ma_sndseq_info.state[func_id][file_id] != MASMW_STATE_READY )
	  && ( ma_sndseq_info.state[func_id][file_id] != MASMW_STATE_OPENED ) )
	{
		return MASMW_ERROR;
	}

	result = ma_srmcnv_func[func_id].Close( file_id, ext_args );

	if ( result >= MASMW_SUCCESS )
	{
		ma_sndseq_info.state[func_id][file_id] = MASMW_STATE_LOADED;
	}

	return result;
}
/****************************************************************************
 *	MaSound_Unload
 *
 *	Function:
 *			Performs unloading prcessing of Stream Converter.
 *	Argument:
 *			func_id		function id
 *			file_id		file id
 *			ext_args	for future extension
 *	Return:
 *			0			success
 *			< 0			error code
 *
 ****************************************************************************/
SINT32 MaSound_Unload
(
	SINT32	func_id,					/* function id */
	SINT32	file_id,					/* file id */
	void *	ext_args					/* for future extension */
)
{
	SINT32 result;						/* result of function */

	MASNDSEQ_DBGMSG(("MaSound_Unload: id=%ld hd=%ld\n", func_id, file_id));

	if ( ( ma_sndseq_info.srmcnv_map & (UINT32)(1 << func_id) ) == 0 )
	{
		return MASMW_ERROR;
	}

	if ( file_id == 0 )
	{
		return MASMW_SUCCESS;
	}

	if ( ma_sndseq_info.state[func_id][file_id] != MASMW_STATE_LOADED )
	{
		return MASMW_ERROR;
	}

	result = ma_srmcnv_func[func_id].Unload( file_id, ext_args );

	if ( result >= MASMW_SUCCESS )
	{
		ma_sndseq_info.state[func_id][file_id] = MASMW_STATE_IDLE;
	}

	return result;
}
/****************************************************************************
 *	MaSound_Delete
 *
 *	Function:
 *			Deletes Stream Converter that has been registered with func_id.
 *	Argument:
 *			func_id		function id
 *	Return:
 *			0			success
 *			< 0			error code
 *
 ****************************************************************************/
SINT32 MaSound_Delete
(
	SINT32	func_id						/* function id */
)
{
	UINT32	i;							/* loop counter */
	SINT32	result;						/* result of function */
	
	MASNDSEQ_DBGMSG(("MaSound_Delete: id=%ld\n", func_id));

	if ( ( ma_sndseq_info.srmcnv_map & (UINT32)(1 << func_id) ) == 0 )
	{
		return MASMW_ERROR;
	}

	for ( i = 0; i < MASMW_NUM_FILE; i++ )
	{
		if ( ma_sndseq_info.state[func_id][i] != MASMW_STATE_IDLE )
		{
			return MASMW_ERROR;
		}
	}

	result = ma_srmcnv_func[func_id].End();

	ma_sndseq_info.srmcnv_map ^= (UINT32)(1 << func_id);

	return result;
}
/****************************************************************************
 *	MaSound_DeviceControl
 *
 *	Function:
 *			Control the hardware.
 *	Argument:
 *			cmd		command number
 *			param1	parameter 1
 *			param2	parameter 2
 *			param3	parameter 3
 *	Return:
 *			>= 0	success
 *			< 0		error code
 *
 ****************************************************************************/
SINT32 MaSound_DeviceControl
(
	UINT8	cmd,						/* command number */
	UINT8	param1,						/* parameter 1 */
	UINT8	param2,						/* parameter 2 */
	UINT8	param3						/* parameter 3 */
)
{
	SINT32	result;						/* result of function */

	MASNDSEQ_DBGMSG(("MaSound_DeviceControl: cmd=%d p1=%d p2=%d p3=%d\n", cmd, param1, param2, param3));

	result = MaSndDrv_DeviceControl( cmd, param1, param2, param3 );

	return result;
}
/****************************************************************************
 *	MaSound_Initialize
 *
 *	Function:
 *			Initialize the MA Sound Sequencer.
 *	Argument:
 *			None
 *	Return:
 *			0		success
 *			< 0 	error code
 *
 ****************************************************************************/
SINT32 MaSound_Initialize
(
	void
)
{
	UINT32	i, j;						/* loop counter */
	SINT32	result;						/* result of function */

	MASNDSEQ_DBGMSG(("MaSound_Initialize\n"));

	/* Initialzie stream converter mapping flag */
	ma_sndseq_info.srmcnv_map = 0;

	/* Initillize the all internal status */
	for ( i = 0; i < MASMW_NUM_SRMCNV; i++ )
	{
		for ( j = 0; j < MASMW_NUM_FILE; j++ )
		{
			ma_sndseq_info.state[i][j] = MASMW_STATE_IDLE;
			ma_sndseq_info.clback_func[i][j] = NULL;
		}
	}

	/* Initialize MA Device Driver */
	result = MaDevDrv_Initialize();
	if ( result != MASMW_SUCCESS )
	{
		return result;
	}

	/* Initialize MA Resource Manager */
	MaResMgr_Initialize();

	/* Initialize MA Sound Driver */
	MaSndDrv_Initialize();

#if MASMW_SRMCNV_SMAF_PHRASE
	PhrAudCnv_Initialize();
#endif

	return MASMW_SUCCESS;
}
