/****************************************************************************
 *																			*
 *		Copyright (C) 2001-2002	YAMAHA CORPORATION. All rights reserved.	*
 *																			*
 *		Module		: ma3sound.h											*
 *																			*
 *		Description	: MA Sound Player API									*
 *																			*
 * 		Version		: 1.3.6.4	2002.06.17									*
 *																			*
 ****************************************************************************/

#ifndef __MA3SOUND_H__
#define __MA3SOUND_H__

#define MA3SMW_CNVID_MMF			(1)		/* SMAF/MA-1/MA-2/MA-3 */
#define MA3SMW_CNVID_PHR			(2)		/* SMAF/Phrase L1 */
#define	MA3SMW_CNVID_RMD			(3)		/* Realtime MIDI */
#define	MA3SMW_CNVID_AUD			(4)		/* Audio */
#define	MA3SMW_CNVID_MID			(5)		/* SMF format 0/GM Lite or Level 1 */

#define MA3SMW_SUCCESS				(0)		/* success 								*/
#define MA3SMW_ERROR				(-1)	/* error								*/
#define MA3SMW_ERROR_ARGUMENT		(-2)	/* error of arguments					*/
#define MA3SMW_ERROR_RESOURCE_OVER	(-3)	/* over specified resources				*/
#define MA3SMW_ERROR_ID				(-4)	/* error id number 						*/

#define MA3SMW_ERROR_FILE			(-16)	/* file error							*/
#define MA3SMW_ERROR_CONTENTS_CLASS	(-17)	/* SMAF Contents Class shows can't play */
#define MA3SMW_ERROR_CONTENTS_TYPE	(-18)	/* SMAF Contents Type shows can't play	*/
#define MA3SMW_ERROR_CHUNK_SIZE		(-19)	/* illegal SMAF Chunk Size value		*/
#define MA3SMW_ERROR_CHUNK			(-20)	/* illegal SMAF Track Chunk value		*/
#define MA3SMW_ERROR_UNMATCHED_TAG	(-21)	/* unmathced specified TAG 				*/
#define MA3SMW_ERROR_SHORT_LENGTH	(-22)	/* short sequence 						*/

#define MASMW_STATE_IDLE			(0)		/* idle state */
#define MASMW_STATE_LOADED			(1)		/* loaded state */
#define MASMW_STATE_OPENED			(2)		/* opened state */
#define MASMW_STATE_READY			(3)		/* ready state */
#define MASMW_STATE_PLAYING			(4)		/* playing state */
#define	MASMW_STATE_PAUSE			(5)		/* pause state */

#define MA3SMW_SET_VOLUME			(0)		/* set volume */
#define MA3SMW_SET_SPEED			(1)		/* set speed */
#define MA3SMW_SET_KEYCONTROL		(2)		/* set key control */
#define MA3SMW_GET_TIMEERROR		(3)		/* get time error */
#define MA3SMW_GET_POSITION			(4)		/* get position */
#define MA3SMW_GET_LENGTH			(5)		/* get length */
#define MA3SMW_GET_STATE			(6)		/* get state */
#define MA3SMW_SEND_MIDIMSG			(7)		/* send midi message */
#define MA3SMW_SEND_SYSEXMIDIMSG	(8)		/* send sys.ex. midi message */
#define MA3SMW_SET_BIND				(9)		/* set bind */
#define MA3SMW_GET_CONTENTSDATA		(10)	/* get contents data */
#define MA3SMW_GET_PHRASELIST		(11)	/* get phrase list */
#define MA3SMW_SET_STARTPOINT		(12)	/* set start point */
#define MA3SMW_SET_ENDPOINT			(13)	/* set end point */
#define MA3SMW_SET_PANPOT			(14)	/* set Panpot */
#define MA3SMW_GET_LEDSTATUS		(15)	/* get LED status */
#define MA3SMW_GET_VIBSTATUS		(16)	/* get VIB status */
#define MA3SMW_SET_EVENTNOTE		(17)	/* set event note */
#define MA3SMW_GET_CONVERTTIME		(18)	/* get convert time */
#define MA3SMW_GET_LOADINFO			(19)	/* get load information */
#define MA3SMW_SET_LOADINFO			(20)	/* set load information */

#define MA3SMW_PWM_DIGITAL			(0)		/* power management (digital) */
#define MA3SMW_PWM_ANALOG			(1)		/* power management (analog) */
#define MA3SMW_EQ_VOLUME			(2)		/* eq volume */
#define MA3SMW_HP_VOLUME			(3)		/* hp volume */
#define MA3SMW_SP_VOLUME			(4)		/* sp volume */
#define MA3SMW_LED_MASTER			(5)		/* LED master select */
#define MA3SMW_LED_BLINK			(6)		/* LED blink setting */
#define MA3SMW_LED_DIRECT			(7)		/* LED direct control */
#define MA3SMW_MTR_MASTER			(8)		/* MOTOR master select */
#define MA3SMW_MTR_BLINK			(9)		/* MOTOR blink setting */
#define MA3SMW_MTR_DIRECT			(10)	/* MOTOR direct control */
#define MA3SMW_GET_PLLOUT			(11)	/* get PLL out */
#define MA3SMW_GET_SEQSTATUS		(12)	/* get status of HW sequencer */

#define MASMW_REPEAT				(126)	/* repeat */
#define MASMW_END_OF_SEQUENCE		(127)	/* end of sequence */

typedef struct _MASMW_MIDIMSG
{
	unsigned char *	msg;
	unsigned long	size;
} MASMW_MIDIMSG, *PMASMW_MIDIMSG;

typedef struct _MASMW_CONTENTSINFO
{
	unsigned short	code;
	unsigned char	tag[2];
	unsigned char *	buf;
	unsigned long	size;
} MASMW_CONTENTSINFO, *PMASMW_CONTENTSINFO;

typedef struct _MASMW_PHRASELIST
{
	unsigned char	tag[2];
	unsigned long	start;
	unsigned long	stop;
} MASMW_PHRASELIST, *PMASMW_PHRASELIST;

typedef struct _MASMW_EVENTNOTE
{
	unsigned char 	ch;						/* channel number */
	unsigned char	note;					/* note number */
} MASMW_EVENTNOTE, *PMASMW_EVENTNOTE;

#define Ma3Sound_DeviceControl MaSound_DeviceControl
#define Ma3Sound_Initialize MaSound_Initialize
#define Ma3Sound_Create MaSound_Create
#define Ma3Sound_Load MaSound_Load
#define Ma3Sound_Open MaSound_Open
#define Ma3Sound_Control MaSound_Control
#define Ma3Sound_Standby MaSound_Standby
#define Ma3Sound_Seek MaSound_Seek
#define Ma3Sound_Start MaSound_Start
#define Ma3Sound_Pause MaSound_Pause
#define Ma3Sound_Restart MaSound_Restart
#define Ma3Sound_Stop MaSound_Stop
#define Ma3Sound_Close MaSound_Close
#define Ma3Sound_Unload MaSound_Unload
#define Ma3Sound_Delete MaSound_Delete

#endif /*__MA3SOUND_H__*/
