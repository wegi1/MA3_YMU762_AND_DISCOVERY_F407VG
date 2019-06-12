/****************************************************************************
 *																			*
 *		Copyright (C) 2001-2002	YAMAHA CORPORATION. All rights reserved.	*
 *																			*
 *		Module		: masndseq.h											*
 *																			*
 *		Description	: MA Sound Sequencer									*
 *																			*
 * 		Version		: 1.3.9.1	2002.11.27									*
 *																			*
 ****************************************************************************/

#ifndef __MASNDSEQ_H__
#define __MASNDSEQ_H__

#include "mamachdep.h"
#include "madefs.h"
#include "madebug.h"
#include "masnddrv.h"

#define MA_STOPWAIT_TIMEOUT	(1000)	/* ms */


typedef struct _MASRMCNVFUNC
{
	SINT32	(* Init)	( void );
	SINT32	(* Load)	( UINT8 * file_ptr, UINT32 file_size, UINT8 mode, SINT32 (*func)(UINT8 id), void * ext_args );
	SINT32 	(* Open)	( SINT32 file_id, UINT16 open_mode, void * ext_args );
	SINT32	(* Control)	( SINT32 file_id, UINT8 ctrl_num, void * prm, void * ext_args );
	SINT32 	(* Standby)	( SINT32 file_id, void * ext_args );
	SINT32 	(* Seek)	( SINT32 file_id, UINT32 pos, UINT8 flag, void * ext_args );
	SINT32 	(* Start)	( SINT32 file_id, void * ext_args );
	SINT32 	(* Stop)	( SINT32 file_id, void * ext_args );
	SINT32 	(* Close)	( SINT32 file_id, void * ext_args );
	SINT32	(* Unload)	( SINT32 file_id, void * ext_args );
	SINT32	(* End)		( void );
} MASRMCNVFUNC, *PMASRMCNVFUNC;

typedef struct _MASNDSEQINFO
{
	UINT16	repeat_mode[MASMW_NUM_SEQTYPE][MASMW_NUM_FILE];
	UINT16	play_mode[MASMW_NUM_SEQTYPE][MASMW_NUM_FILE];
	UINT16	save_mode[MASMW_NUM_SEQTYPE][MASMW_NUM_FILE];
	UINT16	loop_count[MASMW_NUM_SEQTYPE][MASMW_NUM_FILE];
	SINT32	func_id[MASMW_NUM_SEQTYPE];
	SINT32	file_id[MASMW_NUM_SEQTYPE];
	UINT32	srmcnv_map;
	UINT32	start_point[MASMW_NUM_SRMCNV][MASMW_NUM_FILE];
	UINT32	end_point[MASMW_NUM_SRMCNV][MASMW_NUM_FILE];
	UINT32	seek_point[MASMW_NUM_SRMCNV][MASMW_NUM_FILE];
	UINT32	seek_pos0[MASMW_NUM_SRMCNV][MASMW_NUM_FILE];
	UINT32	play_length[MASMW_NUM_SRMCNV][MASMW_NUM_FILE];
	UINT32	loop_length[MASMW_NUM_SRMCNV][MASMW_NUM_FILE];
	UINT8	state[MASMW_NUM_SRMCNV][MASMW_NUM_FILE];
	SINT32 (*clback_func[MASMW_NUM_SRMCNV][MASMW_NUM_FILE])(UINT8 id);
	void *	start_extargs[MASMW_NUM_SRMCNV][MASMW_NUM_FILE];
} MASNDSEQINFO, *PMASNSEQINFO;


SINT32	MaSound_Initialize		( void );
SINT32	MaSound_Create			( UINT8 srm_id );
SINT32	MaSound_Load			( SINT32 func_id, UINT8 * file_ptr, UINT32 file_size, UINT8 mode, SINT32 (* func)(UINT8 id), void * ext_args );
SINT32	MaSound_Open			( SINT32 func_id, SINT32 file_id, UINT16 open_mode, void * ext_args );
SINT32	MaSound_Control			( SINT32 func_id, SINT32 file_id, UINT8 ctrl_num, void * prm, void * ext_args );
SINT32	MaSound_Standby			( SINT32 func_id, SINT32 file_id, void * ext_args );
SINT32	MaSound_Seek			( SINT32 func_id, SINT32 file_id, UINT32 pos, UINT8 flag, void * ext_args );
SINT32	MaSound_Start			( SINT32 func_id, SINT32 file_id, UINT16 play_mode, void * ext_args );
SINT32	MaSound_Pause			( SINT32 func_id, SINT32 file_id, void * ext_args );
SINT32	MaSound_Restart			( SINT32 func_id, SINT32 file_id, void * ext_args );
SINT32	MaSound_Stop			( SINT32 func_id, SINT32 file_id, void * ext_args );
SINT32	MaSound_Close			( SINT32 func_id, SINT32 file_id, void * ext_args );
SINT32	MaSound_Unload			( SINT32 func_id, SINT32 file_id, void * ext_args );
SINT32	MaSound_Delete			( SINT32 func_id );

SINT32	MaSound_DeviceControl	( UINT8 p1, UINT8 p2, UINT8 p3, UINT8 p4 );

SINT32	MaSound_ReceiveMessage	( SINT32 seq_id, SINT32 file_id, UINT8 event );

#endif /*__MASNDSEQ_H__*/
