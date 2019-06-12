/*==============================================================================
//	Copyright(c) 2001-2002 YAMAHA CORPORATION
//
//	Title		: MAPHRCNV.H
//
//	Description	: MA-3 SMAF/Phrase Stream Converter Module.
//
//	Version		: 1.6.0.0	2002.11.15
//
//============================================================================*/
#ifndef	_MAPHRCNV_H_
#define	_MAPHRCNV_H_

#include "mamachdep.h"
#include "madebug.h"
#include "masnddrv.h"		/*	MA-X Sound Driver		*/
#include "maresmgr.h"		/*	MA-X Resource Manager	*/
#include "mammfcnv.h"
#include "maphrwrp.h"


/*== SMAF/Phrase ============================================================*/
#define PHRASE_WAIT_TIMEOUT		(1000)	/* ms */
#define AUDIO_WAIT_TIMEOUT		(1000)  /* ms */

/*==============================================================================
//	define ERROR Code
//============================================================================*/
/*	format error code */
#define	PHRASE_DATA_NOERROR		(0x0000)

#define	MAX_PHRASE_DATA			(5)
#define	MAX_PHRASE_VOICES		(4)
#define	MAX_PHRASE_SLOT		(4)
#define	MIN_PHRASE_DATA_LENGTH	(1)

#define	MA2VOICE_4OP_SIZE		(25)
#define	MA3VOICE_PARAM_SIZE		(30)

/*	Check Mode			*/
#define	CHKMODE_ENABLE			(0x01)
#define	CHKMODE_CNTIONLY		(0x02)

/* Time Base	*/
#define	PHRASE_TIMEBASE			(0x14)	/* 20 msec	*/


/*	Extend voice type	*/
#define	VOICE_TYPE_UNKNOWN		(0)
#define	VOICE_TYPE_MA2			(1)
#define	VOICE_TYPE_MA3			(2)

/*	Control flag		*/
#define	CTRL_CLEAR_ALL			(0x00000000)
#define	CTRL_MASTER_VOLUME		(0x40000000)
#define	CTRL_SYSTEM_ON			(0x80000000)

#define	CTRL_INIT_PHRASE		(0x00000001)
#define	CTRL_VOLUME_CHANGE		(0x00000002)
#define	CTRL_PANPOT_CHANGE		(0x00000004)
#define	CTRL_WAIT				(0x00000008)


/*==============================================================================
//	typedef struct
//============================================================================*/
/*	SMAF/Phrase Voice Information	*/
typedef struct _tagVoiceInfo
{
	UINT8	bBankNo;
	UINT8	bProgNo;
	UINT8	bType;
	UINT8	bSize;
	UINT8*	pbVoice;
} VOCINFO, *PVOCINFO;

/*	SMAF/Phrase Information */
typedef struct _tagPhrInfo
{
	UINT8	bPhrNum;					/*	Phrase Number(ID)	(0..3)			*/
	UINT16	wLoopNum;					/*	Loop Count							*/
	UINT8	bCntiClass;					/*	Contents Class						*/
	UINT8	bCntiType;					/*	Contents Type						*/
	UINT8	bCodeType;					/*	Contents Code Type					*/
	UINT8	bCopyStatus;				/*	Copy Status							*/
	UINT8	bCopyCount;					/*	Copy Count							*/
	UINT8*	pbCntiOption;				/*	Pointer to Contents info Chunk		*/
	UINT8*	pbOptionChunk;				/*	Pointer to Option Data Chunk		*/
	UINT8*	pbInfoChunk;				/*	Pointer to Info Chunk body			*/
	UINT8*	pbVoiceChunk;				/*	Pointer to Voice Chunk body			*/
	UINT8*	pbSequenceChunk;			/*	Pointer to Sequence Chunk body		*/
	UINT32	dwCntiOptionSize;			/*	The size of Contents info Chunk		*/
	UINT32	dwOptionDataSize;			/*	The size of Option Data Chunk		*/
	UINT32	dwInfoChunkSize;			/*	The size of Info Chunk body			*/
	UINT32	dwVoiceChunkSize;			/*	The size of Voice Chunk body		*/
	UINT32	dwSequenceChunkSize;		/*	The size of Sequence Chunk body		*/
	UINT32	dwPlayTime;					/*	total Play back time (tick)				*/
	UINT32	dwTimer;
	UINT32	dwCurrentTime;
	UINT32	dwDataPosition;
	SINT32	(*CallbackFunc)(UINT8 id);	/* pointer to the callback function */
	VOCINFO	VocInfo[MAX_PHRASE_VOICES];
} PHRINFO, *PPHRINFO;

/*	work area for SMAF/Phrase Format Check	*/
typedef struct _tagPhrCheck
{
	UINT8		bMode;				/*	Check Mode								*/
	UINT16		wErrStatus;			/**/
	UINT8*		pbBuffer;			/*	Pointer to Data							*/
	UINT32		dwSize;				/*	The size of Data (in Byte)				*/
	UINT8*		pbPhrase;			/*	Pointer to Phrase Header				*/
	UINT32		dwPhraseSize;		/*	The size of Phrase Data					*/
	UINT8*		pbVoice;			/*	Pointer to Voice Chunk Header			*/
	UINT32		dwVoiceSize;		/*	The size of Voice Chunk Data			*/
	PPHRINFO	pPhrInfo;			/**/
} PHRCHECK, *PPHRCHECK;

/*	Channel Information */
typedef struct _tagChInfo
{
	UINT8	bNew;					/*	*/
	UINT8	bKey;					/*	Key Number	(0..127)						*/
	UINT8	bVoiceNo;				/*	Voicem Number	(0..3)						*/
	UINT8	bVolume;				/*	Channel Volume	(0..127)					*/
	UINT8	bExp;					/*	Expression	(0..127)						*/
	UINT8	bPan;					/*	Panpot		(0..127)						*/
	UINT8	bMod;					/*	Modulation	(0..?)							*/
	UINT8	bPitch;					/*	PitchBend	(0..127							*/
	UINT8	bRange;					/*	PitchBend Range	(100cent unit)				*/
	UINT8	bExNoteFlag;			/*	bit0 : Low	bit1: High						*/
	UINT8	bRegL;					/*	Extended Note Low-Byte value				*/
	UINT8	bRegH;					/*	Extended Note High-Byte value				*/
	SINT16	nOctShift;				/*	Octave Shift (-4..0..4)						*/
	UINT32	dwKeyOffTime;			/*	KeyOff Time									*/
} CHINFO, *PCHINFO;

/* Sequencer Information */
typedef struct	_tagApiInfo {
	IDSTATUS	status;
	SINT32		file_id;
	UINT8		vol;
	UINT8		pan;
	UINT16		loop;
	int			master_channel;
	UINT32		slave;
	UINT32		length;
	SINT32		pos;
	PPHRINFO	gpPhrase;
	PHRINFO		gPhraseInfo;
	UINT32		gdwRamAddr;
} APIINFO, *PAPIINFO;


/*== SMAF/Audio =============================================================*/


/*==============================================================================
//	define ERROR Code
//============================================================================*/
/*	convert error code	*/
#define	MA3AUDERR_NOERROR			MASMW_SUCCESS
#define	MA3AUDERR_GENERAL			MASMW_ERROR
#define	MA3AUDERR_ARGUMENT			MASMW_ERROR_ARGUMENT
#define	MA3AUDERR_UNMATCHED_TAG		MASMW_ERROR_UNMATCHED_TAG

/*==============================================================================
//	prototype functions
//============================================================================*/
SINT32	PhrAudCnv_Initialize(void);

SINT32	MaAudCnv_Initialize	(void);
SINT32	MaAudCnv_End		(void);
SINT32	MaAudCnv_Load		(UINT8* file_ptr, UINT32 file_size, UINT8 mode, SINT32 (*func)(UINT8 id), void* ext_args);
SINT32	MaAudCnv_Unload		(SINT32 file_id, void* ext_args);
SINT32	MaAudCnv_Open		(SINT32 file_id, UINT16 mode, void* ext_args);
SINT32	MaAudCnv_Close		(SINT32 file_id, void* ext_args);
SINT32	MaAudCnv_Standby	(SINT32 file_id, void* ext_args);
SINT32	MaAudCnv_Seek		(SINT32 file_id, UINT32 pos, UINT8 flag, void* ext_args);
SINT32	MaAudCnv_Start		(SINT32 file_id, void* ext_args);
SINT32	MaAudCnv_Stop		(SINT32 file_id, void* ext_args);
SINT32	MaAudCnv_Control	(SINT32 file_id, UINT8 ctrl_num, void* prm, void* ext_args);
SINT32	MaAudCnv_Convert	(void);


#endif	/*	_MAPHRCNV_H_	*/

