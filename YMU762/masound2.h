/****************************************************************************
 *																			*
 *		Copyright (C) 2001-2002	YAMAHA CORPORATION. All rights reserved.	*
 *																			*
 *		Module		: masound.h												*
 *																			*
 *		Description	: MA Sound Player Extended API							*
 *																			*
 * 		Version		: 1.3.12.4	2002.12.13									*
 *																			*
 ****************************************************************************/

#ifndef __MASOUND_H__
#define __MASOUND_H__

#define MASMW_CNVID_MMF				(1)		/* SMAF/MA-1/MA-2/MA-3 */
#define MASMW_CNVID_PHR				(2)		/* SMAF/Phrase L1 */
#define	MASMW_CNVID_RMD				(3)		/* Realtime MIDI */
#define	MASMW_CNVID_AUD				(4)		/* Audio */
#define	MASMW_CNVID_MID				(5)		/* SMF format 0/GM Lite or Level 1 */
#define	MASMW_CNVID_WAV				(11)	/* WAV */

#define MASMW_SUCCESS				(0)		/* success 								*/
#define MASMW_ERROR					(-1)	/* error								*/
#define MASMW_ERROR_ARGUMENT		(-2)	/* error of arguments					*/
#define MASMW_ERROR_RESOURCE_OVER	(-3)	/* over specified resources				*/
#define MASMW_ERROR_ID				(-4)	/* error id number 						*/

#define MASMW_ERROR_FILE			(-16)	/* file error							*/
#define MASMW_ERROR_CONTENTS_CLASS	(-17)	/* SMAF Contents Class shows can't play */
#define MASMW_ERROR_CONTENTS_TYPE	(-18)	/* SMAF Contents Type shows can't play	*/
#define MASMW_ERROR_CHUNK_SIZE		(-19)	/* illegal SMAF Chunk Size value		*/
#define MASMW_ERROR_CHUNK			(-20)	/* illegal SMAF Track Chunk value		*/
#define MASMW_ERROR_UNMATCHED_TAG	(-21)	/* unmathced specified TAG 				*/
#define MASMW_ERROR_SHORT_LENGTH	(-22)	/* short sequence 						*/
#define MASMW_ERROR_LONG_LENGTH		(-23)	/* long sequence 						*/

#define MASMW_ERROR_MA3BOARD_NORESPONSE	(-32)

#define MASMW_ERROR_SMF_FORMAT		(-50)	/* invalid format type != 0/1			*/
#define MASMW_ERROR_SMF_TRACKNUM	(-51)	/* invalid number of tracks				*/
#define MASMW_ERROR_SMF_TIMEUNIT	(-52)	/* invalid time unit					*/
#define MASMW_ERROR_SMF_CMD			(-53)	/* invalid command byte					*/

#define MASMW_STATE_IDLE			(0)		/* idle state */
#define MASMW_STATE_LOADED			(1)		/* loaded state */
#define MASMW_STATE_OPENED			(2)		/* opened state */
#define MASMW_STATE_READY			(3)		/* ready state */
#define MASMW_STATE_PLAYING			(4)		/* playing state */
#define	MASMW_STATE_PAUSE			(5)		/* pause state */

#define MASMW_SET_VOLUME			(0)		/* set volume */
#define MASMW_SET_SPEED				(1)		/* set speed */
#define MASMW_SET_KEYCONTROL		(2)		/* set key control */
#define MASMW_GET_TIMEERROR			(3)		/* get time error */
#define MASMW_GET_POSITION			(4)		/* get position */
#define MASMW_GET_LENGTH			(5)		/* get length */
#define MASMW_GET_STATE				(6)		/* get state */
#define MASMW_SEND_MIDIMSG			(7)		/* send midi message */
#define MASMW_SEND_SYSEXMIDIMSG		(8)		/* send sys.ex. midi message */
#define MASMW_SET_BIND				(9)		/* set bind */
#define MASMW_GET_CONTENTSDATA		(10)	/* get contents data */
#define MASMW_GET_PHRASELIST		(11)	/* get phrase list */
#define MASMW_SET_STARTPOINT		(12)	/* set start point */
#define MASMW_SET_ENDPOINT			(13)	/* set end point */
#define MASMW_SET_PANPOT			(14)	/* set Panpot */
#define MASMW_GET_LEDSTATUS			(15)	/* get LED status */
#define MASMW_GET_VIBSTATUS			(16)	/* get VIB status */
#define MASMW_SET_EVENTNOTE			(17)	/* set event note */
#define MASMW_GET_CONVERTTIME		(18)	/* get convert time */
#define MASMW_GET_LOADINFO			(19)	/* get load information */
#define MASMW_SET_LOADINFO			(20)	/* set load information */
#define MASMW_SET_SPEEDWIDE			(26)	/* set speedwide */
#define MASMW_SET_REPEAT			(27)	/* set number of repeat */
#define MASMW_GET_CONTROL_VAL		(29)	/* Get control value  */
#define MASMW_SET_CB_INTERVAL		(30)	/* Set calback function interval */

#define MASMW_PWM_DIGITAL			(0)		/* power management (digital) */
#define MASMW_PWM_ANALOG			(1)		/* power management (analog) */
#define MASMW_EQ_VOLUME				(2)		/* eq volume */
#define MASMW_HP_VOLUME				(3)		/* hp volume */
#define MASMW_SP_VOLUME				(4)		/* sp volume */
#define MASMW_LED_MASTER			(5)		/* LED master select */
#define MASMW_LED_BLINK				(6)		/* LED blink setting */
#define MASMW_LED_DIRECT			(7)		/* LED direct control */
#define MASMW_MTR_MASTER			(8)		/* MOTOR master select */
#define MASMW_MTR_BLINK				(9)		/* MOTOR blink setting */
#define MASMW_MTR_DIRECT			(10)	/* MOTOR direct control */
#define MASMW_GET_PLLOUT			(11)	/* get PLL out */
#define MASMW_GET_SEQSTATUS			(12)	/* get status of HW sequencer */

#define MASMW_REPEAT				(126)	/* repeat */
#define MASMW_END_OF_SEQUENCE		(127)	/* end of sequence */

#define MASMW_RMD_INTERVAL_CB		(17)	/* interval timer */

typedef struct _MASMW_MIDIMSG
{
	unsigned char *	msg;					/* pointer to MIDI message */
	unsigned long	size;					/* size of MIDI message */
} MASMW_MIDIMSG, *PMASMW_MIDIMSG;

typedef struct _MASMW_CONTENTSINFO
{
	unsigned short	code;					/* code type */
	unsigned char	tag[2];					/* tag name */
	unsigned char *	buf;					/* pointer to read buffer */
	unsigned long	size;					/* size of read buffer */
} MASMW_CONTENTSINFO, *PMASMW_CONTENTSINFO;

typedef struct _MASMW_PHRASELIST
{
	unsigned char	tag[2];					/* tag name */
	unsigned long	start;					/* start point */
	unsigned long	stop;					/* stop point */
} MASMW_PHRASELIST, *PMASMW_PHRASELIST;

typedef struct _MASMW_EVENTNOTE
{
	unsigned char 	ch;						/* channel number */
	unsigned char	note;					/* note number */
} MASMW_EVENTNOTE, *PMASMW_EVENTNOTE;

typedef	struct _MASMW_GETCTL
{
	unsigned char	bControl;				/* contorl number */
	unsigned char 	bCh;					/* channel number */
} MASMW_GETCTL, *PMASMW_GETCTL;

#if defined(__cplusplus)
extern "C" {
#endif

#ifdef MASMW_USEMA3BOARD
signed long	MaSound_Initialize		( char *pInDev, char *pOutDev );
signed long	MaSound_Terminate		( void );
#else
signed long	MaSound_Initialize		( void );
#endif
signed long	MaSound_DeviceControl	( unsigned char 	p1,
									  unsigned char 	p2,
									  unsigned char 	p3,
									  unsigned char 	p4 );
signed long	MaSound_Create			( unsigned char		srm_id );
signed long	MaSound_Load			( signed long 		func_id,
									  unsigned char *	file_ptr,
									  unsigned long		file_size,
									  unsigned char		mode,
									  signed long (* func)(unsigned char id),
									  void * 			ext_args );
signed long	MaSound_Open			( signed long 		func_id,
									  signed long		file_id,
									  unsigned short 	open_mode,
									  void * 			ext_args );
signed long	MaSound_Control			( signed long 		func_id,
									  signed long		file_id,
									  unsigned char		ctrl_num,
									  void * 			prm,
									  void * 			ext_args );
signed long	MaSound_Standby			( signed long 		func_id,
									  signed long		file_id,
									  void *			ext_args );
signed long	MaSound_Seek			( signed long 		func_id,
									  signed long		file_id,
									  unsigned long		pos,
									  unsigned char		flag,
									  void * 			ext_args );
signed long	MaSound_Start			( signed long 		func_id,
									  signed long		file_id,
									  unsigned short	play_mode,
									  void * 			ext_args );
signed long	MaSound_Pause			( signed long		func_id,
									  signed long		file_id,
									  void * 			ext_args );
signed long	MaSound_Restart			( signed long		func_id,
									  signed long		file_id,
									  void * 			ext_args );
signed long	MaSound_Stop			( signed long		func_id,
									  signed long		file_id,
									  void * 			ext_args );
signed long	MaSound_Close			( signed long		func_id,
									  signed long		file_id,
									  void *			ext_args );
signed long	MaSound_Unload			( signed long		func_id,
									  signed long		file_id,
									  void *			ext_args );
signed long	MaSound_Delete			( signed long 		func_id );

#if defined(__cplusplus)
}
#endif


#ifndef MAX_PHRASE_CHANNEL
#define	MAX_PHRASE_CHANNEL	(4)
#endif	/*	!MAX_PHRASE_CHANNEL	*/

#define	BIT_STATUS_SLAVE	(0x10)

typedef enum	_tagIdStatus {
	ID_STATUS_UNKNOWN	= 0,
	ID_STATUS_NODATA,
	ID_STATUS_READY,
	ID_STATUS_PLAY,
	ID_STATUS_ENDING,
	ID_STATUS_PAUSE,
	ID_STATUS_LINKSLAVE
} IDSTATUS;

struct	event {
	int	ch;
	int	mode;
};

#if 1
struct info {
	long	MakerID;
	int		DeviceID;
	int		VersionID;
	int		MaxVoice;
	int		MaxChannel;
	int		SupportSMAF;
	long	Latency;
};
#endif

#if defined(__cplusplus)
extern "C" {
#endif

int		Phrase_Initialize	(void);
int		Phrase_Terminate	(void);
int		Phrase_GetInfo		(struct info* dat);
int		Phrase_CheckData	(unsigned char* data, unsigned long len);
int		Phrase_SetData		(int ch, unsigned char* data, unsigned long len, int check);
int		Phrase_Seek			(int ch, long pos);
int		Phrase_Play			(int ch, int loop);
int		Phrase_Stop			(int ch);
int		Phrase_Pause		(int ch);
int		Phrase_Restart		(int ch);
int		Phrase_Kill			(void);
void	Phrase_SetVolume	(int ch, int vol);
int		Phrase_GetVolume	(int ch);
void	Phrase_SetPanpot	(int ch, int vol);
int		Phrase_GetPanpot	(int ch);
int		Phrase_GetStatus	(int ch);
long	Phrase_GetPosition	(int ch);
long	Phrase_GetLength	(int ch);
int		Phrase_RemoveData	(int ch);
int		Phrase_SetEvHandler(void (* func)(struct event*));
int		Phrase_SetLink		(int ch, unsigned long slave);
unsigned long	Phrase_GetLink(int ch);


#if defined(__cplusplus)
}
#endif

#endif /*__MASOUND_H__*/
