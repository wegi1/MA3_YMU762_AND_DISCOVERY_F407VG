/*==============================================================================
//	Copyright(c) 2001-2003 YAMAHA CORPORATION
//
//	Title		: MAPHRCNV.C
//
//	Description	: MA-3 SMAF/Phrase Stream Converter Module.
//
//	Version		: 1.6.1.0		2003.04.04
//
//============================================================================*/
#include "maphrcnv.h"
#include "malib.h"

/*== common ============================================================*/
/*=============================================================================
//	gloval values
=============================================================================*/
static UINT8		gbCreateStatus = 0; /* bit0:SMAF/Phrase bit1:SMAF/Audio  */
static UINT8		gbSeqStatus = 0;    /* bit0:SMAF/Phrase bit1:SMAF/Audio  */
								   /* 1:sequence playing 0:sequence stop */
#define MASK_STATUS_PHRASE	(0x01)
#define MASK_STATUS_AUDIO	(0x02)


/*== SMAF/Phrase ============================================================*/
/*=============================================================================
//	gloval values
=============================================================================*/
static UINT8		gbPlayStatus;  /* bit0:ch0 - bit3:ch3 ... 1:Playing 0:not playing */
static UINT8		gbPauseStatus; /* bit0:ch0 - bit3:ch3 ... 1:Pause   0:not pause   */
static const UINT8	gMaskPlay[MAX_PHRASE_CHANNEL] = {0x1, 0x2, 0x4, 0x8};
static UINT32		gdwCtrlStatus; /* bit0..3:ch0 - bit12..15:ch3 */
								   /* bit4n:set init param        */
								   /* bit4n+1:change volume       */
								   /* bit4n+2:change panpot       */

static UINT32		gdwPhrStatus;  /* bit0..3:ch0 - bit12..15:ch3 */
								   /* bit4n:start flag            */
								   /* bit4n+1:stop flag           */
								   /* bit4n+2:pause flag          */
								   /* bit4n+3:restart flag        */
#define MASK_PHRASE_STATUS_START	(0x00001111)
#define MASK_PHRASE_STATUS_STOP		(0x00002222)
#define MASK_PHRASE_STATUS_PAUSE	(0x00004444)
#define MASK_PHRASE_STATUS_RESTART	(0x00008888)
static const UINT32 gMaskStart[MAX_PHRASE_CHANNEL] =	{0x1, 0x10, 0x100, 0x1000};
#if MA_STOPWAIT
static const UINT32 gMaskStop[MAX_PHRASE_CHANNEL] =		{0x2, 0x20, 0x200, 0x2000};
#endif
static const UINT32 gMaskPause[MAX_PHRASE_CHANNEL] =	{0x4, 0x40, 0x400, 0x4000};
static const UINT32 gMaskRestart[MAX_PHRASE_CHANNEL] =	{0x8, 0x80, 0x800, 0x8000};

static CHINFO		gChInfo[MASMW_NUM_CHANNEL];


/* Phrase Wrapper */
typedef	SINT32	(*PCALLBACK)(UINT8 id);
static SINT32	CallBack_Service1(UINT8 id);
static SINT32	CallBack_Service2(UINT8 id);
static SINT32	CallBack_Service3(UINT8 id);
static SINT32	CallBack_Service4(UINT8 id);

static SINT32		gdwPhrId = -1L;			/*	MA-3 SMAF/Phrase Stream Converter ID */
static SINT32		gdwSlave = 0L;
static APIINFO		gApiInfo[MAX_PHRASE_DATA];
static PCALLBACK	gCallBack[4] = {CallBack_Service1, CallBack_Service2, CallBack_Service3, CallBack_Service4};

static void		(* gEvHandler)(struct event* eve);

static UINT32	gPhrRamAdrs;		/* RAM address for SMAF/Phrase converter */
static UINT32	gPhrRamSize;		/* RAM size for SMAF/Phrase converter */


/*	Short type to Standard type Volume converting table	*/
static const UINT8	gbVolTbl[16] = {0, 0, 31, 39, 47, 55, 63, 71, 79, 87, 95, 103, 111, 119, 127, 127};

/*	Short type to Standard type Modulation converting table	*/
static const UINT8	gbModTbl[16] = {0, 0, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4};



/*== SMAF/Audio =============================================================*/
typedef struct _MAAUDCNV_INFO
{
	UINT8		bSMAFType;						/* SMAF Type    */
	UINT8		bCodeType;						/* Code Type	*/
	UINT8		bOptionType;					/* Option Type	*/
	UINT8*		pConti;							/* Top of CNTI  */
	UINT8*		pInfo;							/* Top of OPDA  */
	UINT32		dwSize;							/* Size of OPDA */

	UINT32		dFs;							/* [Hz]         */
	UINT32		dFormat;						/*              */
	UINT8*		pSource;						/* Top of Data  */
	UINT32		dSizeOfSource;					/* Size of Data */
} MAAUDCNV_INFO, *PMAAUDCNV_INFO;


/*=============================================================================
//	gloval values
=============================================================================*/
static UINT8		gbAudCtrlStatus;	/* bit0:change master volume  1:change 0:not change */
#define CTRL_AUDIO_MASTERVOLUME_CHANGE	(0x01)

static UINT8		gbAudStatus;		/* bit0:start flag */
										/* bit1:stop flag  */
										/* bit2:stop and start from top */
#define MASK_AUDIO_STATUS_START			(0x01)
#define MASK_AUDIO_STATUS_STOP			(0x02)
#define MASK_AUDIO_STATUS_REPLAY		(0x04)

static SINT32		gSeqID = -1;							/* Sequence ID  */
static SINT32		gFileID;						/* File ID      */
static UINT8		gNumOfLoaded;					/*              */

static UINT8		gbAudEnding;					/* Flag of stop proccess */

static MAAUDCNV_INFO	gInfo[2];					/* Top of OPDA  */
static UINT8		gFormat;						/*              */
static UINT8		gPanpot;						/* Value of Panpot        */
static UINT8		gMasterVol;						/* Value of Master volume */
static UINT32		gFs;							/* [Hz]         */
static UINT8*		gpSource;						/* Top of data  */
static UINT32		gSizeOfSource;					/* Size of Data */

/*===========================================================================*/

SINT32 MaPhrCnv_Convert( void );
extern void machdep_memcpy( UINT8 *d, UINT8 *s, UINT32 size );
extern void machdep_Sleep( UINT32 time );






/*=============================================================================
//	Function Name	:	UINT32	ConvertMA3Voice(UINT8* pM2V, UINT8 bM2Size, UINT8* pM3V)
//
//	Description		:	Convert MA-2 Voice Parameter to MA-3 format
//
//	Argument		:	pM2V	...	Pointer to MA-2 voice parameter
//						bM2Size	...	Size of MA-2 voice parameter
//						pM3V	...	Pointer to MA-3 voice parameter
//
//	Return			:	converted size
//
=============================================================================*/
static UINT32	ConvertMA3Voice(UINT8* pM2V, UINT8 bM2Size, UINT8* pM3V)
{
	UINT8	bFb;
	UINT8	bAlg;
	UINT8	bNumofOp;
	UINT8	bOp;
	UINT8	bVib;
	UINT8	bEgt;
	UINT8	bSus;
	UINT8	bKsr;
	UINT8	bRR;
	UINT8	bDR;
	UINT8	bDvb;
	UINT8	bDam;
	UINT8	bAm;
	UINT8	bWs;
	UINT32	dwSize = 0L;

	bNumofOp = 4;
	bFb	 = (UINT8)((pM2V[3] & 0x38) >> 3);
	bAlg = (UINT8)(pM2V[3] & 0x07);
	if((bAlg & 0x06) == 0)			bNumofOp = 2;
	if(bM2Size < MA2VOICE_4OP_SIZE)	bNumofOp = 2;

	pM3V[0] = (UINT8)(0x80 | (pM2V[4] & 0x03));	/*	panpot = 0x10 center	*/
	pM3V[1] = (UINT8)((pM2V[3] & 0xC0) | bAlg);

	pM2V += 5;
	pM3V += 2;
	dwSize = 2;
	for(bOp = 0; bOp < bNumofOp; bOp++)
	{
		bVib = (UINT8)((pM2V[0] & 0x08) >> 3);
		bEgt = (UINT8)((pM2V[0] & 0x04) >> 2);
		bSus = (UINT8)((pM2V[0] & 0x02) >> 1);
		bKsr = (UINT8)((pM2V[0] & 0x01) >> 0);

		bRR	 = (UINT8)((pM2V[1] & 0xF0) >> 4);
		bDR	 = (UINT8)(pM2V[1] & 0x0F);

		bDvb = (UINT8)((pM2V[4] & 0xC0) >> 6);
		bDam = (UINT8)((pM2V[4] & 0x30) >> 4);
		bAm	 = (UINT8)((pM2V[4] & 0x08) >> 3);
		bWs	 = (UINT8)((pM2V[4] & 0x07));

		pM3V[0] = (UINT8)((((bEgt) ? 0x00 : bRR) << 4)| 0x04 | bKsr);	/*	SR | DR					*/
		pM3V[1] = (UINT8)(((bSus) ? 0x04 : bRR) << 4| bDR);				/*	RR | DR					*/
		pM3V[2] = pM2V[2];												/*	AR | SL					*/
		pM3V[3] = pM2V[3];												/*	TL | KSL				*/
		pM3V[4] = (UINT8)(((bDam) << 5) | (bAm << 4) | (bDvb << 1) | (bVib));
																		/*	DAM | EAM | DVB | EVB	*/
		pM3V[5] = (UINT8)(pM2V[0] & 0xF0);								/*	MULTI | DT=0			*/
		pM3V[6] = (UINT8)(((bWs) << 3) | ((bOp == 0) ? bFb : 0x00));	/*	WS | FB					*/

		pM2V += 5;
		pM3V += 7;
		dwSize += 7;
	}

	return(dwSize);
}

/*=============================================================================
//	Function Name	:	void	PhrChk_InitPhraseInfo(PPHRINFO pPhrInfo)
//
//	Description		:	Initialize PHRINFO structure
//
//	Argument		:	pPhrInfo		...	Pointer to PHRINFO structure
//
//	Return			:
//
=============================================================================*/
static void	PhrChk_InitPhraseInfo(PPHRINFO pPhrInfo)
{
	UINT8	i;

	pPhrInfo->bPhrNum				= 0;
	pPhrInfo->bCntiClass			= 0;
	pPhrInfo->bCntiType				= 0;
	pPhrInfo->bCodeType				= 0;
	pPhrInfo->bCopyStatus			= 0;
	pPhrInfo->bCopyCount			= 0;
	pPhrInfo->pbCntiOption			= NULL;
	pPhrInfo->pbOptionChunk			= NULL;
	pPhrInfo->pbInfoChunk			= NULL;
	pPhrInfo->pbVoiceChunk			= NULL;
	pPhrInfo->pbSequenceChunk		= NULL;
	pPhrInfo->dwCntiOptionSize		= 0L;
	pPhrInfo->dwOptionDataSize		= 0L;
	pPhrInfo->dwInfoChunkSize		= 0L;
	pPhrInfo->dwVoiceChunkSize		= 0L;
	pPhrInfo->dwSequenceChunkSize	= 0L;
	pPhrInfo->dwPlayTime			= 0L;
	pPhrInfo->dwTimer				= 0L;
	pPhrInfo->dwCurrentTime			= 0L;
	pPhrInfo->dwDataPosition		= 0L;
	for(i = 0; i < MAX_PHRASE_VOICES; i++)
	{
		pPhrInfo->VocInfo[i].bBankNo = 0;
		pPhrInfo->VocInfo[i].bProgNo = 0;
	}
}

/*=============================================================================
//	Function Name	:	void	PhrChk_Initialize(PPHRCHECK pPhrChk, UINT8 bPhrNum,
//										 UINT8 bMode, UINT8* pBuffer, UINT32 dwSize)
//
//	Description		:	Initialize PHRCHECK structure
//
//	Argument		:	pPhrChk		...	Pointer to PHRCHECK structure
//						bPhrNum		...	Phrase Number (0..4).
//						bMode		...	Check mode.
//						pBuffer		...	Pointer to data.
//						dwSize		...	The Size of data(in Byte).
//	Return			:
//
=============================================================================*/
static void	PhrChk_Initialize(PPHRCHECK pPhrChk, UINT8 bPhrNum, UINT8 bMode, UINT8* pBuffer, UINT32 dwSize)
{
	pPhrChk->bMode				= bMode;
	pPhrChk->wErrStatus			= PHRASE_DATA_NOERROR;
	pPhrChk->pbBuffer			= pBuffer;
	pPhrChk->dwSize				= dwSize;
	pPhrChk->pbPhrase			= pBuffer;
	pPhrChk->dwPhraseSize		= dwSize;
	pPhrChk->pbVoice			= pBuffer;
	pPhrChk->dwVoiceSize		= dwSize;
	pPhrChk->pbPhrase			= NULL;
	pPhrChk->dwPhraseSize		= 0L;
	pPhrChk->pPhrInfo			= NULL;
	if(bPhrNum < MAX_PHRASE_DATA) {
		pPhrChk->pPhrInfo = &(gApiInfo[bPhrNum].gPhraseInfo);
		PhrChk_InitPhraseInfo(pPhrChk->pPhrInfo);
		pPhrChk->pPhrInfo->bPhrNum	= bPhrNum;
	}
}

/*=============================================================================
//	Function Name	:	PPHRINFO	SmafPhrChecker(UINT8 bPhrNum, UINT8 bMode,
//										UINT8* pBuffer, UINT32 dwSize)
//
//	Description		:	Check SMAF/Phrase
//
//	Argument		:	bPhrNum		...	#Phrase(0..4  4:only check)
//						bMode		...	check mode
//											bit0 : error check	0:disable 1:enable
//											bit1 : contents info check 0:not only 1:only
//						pBuffer		...	Pointer to the data
//						dwSize		...	Data size
//
//	Return			:	data length(msec) or error code
//
=============================================================================*/
static SINT32	SmafPhrChecker(UINT8 bPhrNum, UINT8 bMode, UINT8* pBuffer, UINT32 dwSize)
{
	PHRCHECK	PhrChkArea;				/*	work area for SMAF/Phrase format Check	*/
	PPHRCHECK	pPCA = (PPHRCHECK)(&PhrChkArea);

	if(bPhrNum >= MAX_PHRASE_DATA)		return 0;

	/*	Initialize PHRCHECK structure		*/
	PhrChk_Initialize(pPCA, bPhrNum, bMode, pBuffer, dwSize);
/*
	return ((PPHRINFO)pPCA->pPhrInfo);
*/
	return malib_smafphrase_checker(pBuffer, dwSize, pPCA);
}

/*==============================================================================
//	Function Name	:	UINT32	ProgramChange(UINT8 bCh, UINT8 bVoice)
//
//	Description		:	Convert to ProgramChange(Native format)
//
//	Argument		:	bCh			...	#Channel(0..15)
//						bVoiceNo	...	#Voice(0..3)
//
//	Return			:	0:NoError  <0:Error Code
//
==============================================================================*/
static UINT32	ProgramChange(UINT8 bCh, UINT8 bVoice)
{
	MAPHRCNV_DBGMSG(("ProgramChange[%d %02X] \n", bCh, bVoice));

	if(gChInfo[bCh].bVoiceNo != bVoice) {
		gChInfo[bCh].bVoiceNo = bVoice;
		gChInfo[bCh].bNew	= 1;
	}
	return (0L);
}

/*==============================================================================
//	Function Name	:	SINT32	CnvOctaveShift(UINT8 bCh, UINT8 bOct)
//
//	Description		:	Convert to OctaveShift(Native format)
//
//	Argument		:	bCh			...	#Channel(0..15)
//						bOct		...	OctaveShift
//
//	Return			:	0:NoError  <0:Error Code
//
==============================================================================*/
static UINT32 OctaveShift(UINT8 bCh, UINT8 bOct)
{
	MAPHRCNV_DBGMSG(("OctaveShift[%d %02X] \n", bCh, bOct));

	switch(bOct) {
	case 0x00:
	case 0x01:
	case 0x02:
	case 0x03:
	case 0x04:
		gChInfo[bCh].nOctShift = (SINT16)bOct;
		break;
	case 0x81:
		gChInfo[bCh].nOctShift = -1;
		break;
	case 0x82:
		gChInfo[bCh].nOctShift = -2;
		break;
	case 0x83:
		gChInfo[bCh].nOctShift = -3;
		break;
	case 0x84:
		gChInfo[bCh].nOctShift = -4;
		break;
	default:   /* reserved */
		gChInfo[bCh].nOctShift = 0;
		break;
	}

	return (0L);
}

/*==============================================================================
//	Function Name	:	UINT32	CnvChannelVolume(UINT8 bCh, UINT8 bVol)
//
//	Description		:	Convert to Channel Volume(Native format)
//
//	Argument		:	bCh			...	#Channel(0..15)
//						bVol		...	Volume(0..127)
//
//	Return			:	>0:Number of converted data   <0:Error Code
//
==============================================================================*/
static UINT32	CnvChannelVolume(UINT8 bCh, UINT8 bVol)
{
	MAPHRCNV_DBGMSG(("CnvChannelVolume[%d %02X] \n", bCh, bVol));

	if(gChInfo[bCh].bVolume != bVol)
	{
		MaSndDrv_SetCommand(gdwPhrId, -1L, MASNDDRV_CMD_CHANNEL_VOLUME, (UINT32)bCh, (UINT32)bVol, 0L);
		gChInfo[bCh].bVolume = bVol;
		return (1L);
	}
	return (0L);
}

/*==============================================================================
//	Function Name	:	UINT32	CnvModulation(UINT8 bCh, UINT8 bMod)
//
//	Description		:	Convert to Modulation(Native format)
//
//	Argument		:	bCh			...	#Channel(0..15)
//						bMod		...	Modulation(0..4)
//
//	Return			:	>0:Number of converted data  <0:Error Code
//
==============================================================================*/
static UINT32	CnvModulation(UINT8 bCh, UINT8 bMod)
{
	MAPHRCNV_DBGMSG(("CnvModulation[%d %02X] \n", bCh, bMod));

	if(gChInfo[bCh].bMod != bMod)
	{
		MaSndDrv_SetCommand(gdwPhrId, -1L, MASNDDRV_CMD_MODULATION_DEPTH, (UINT32)bCh, (UINT32)bMod, 0L);
		gChInfo[bCh].bMod = bMod;
		return (1L);
	}
	return (0L);
}

/*==============================================================================
//	Function Name	:	UINT32	CnvExpression(UINT8 bCh, UINT8 bExp)
//
//	Description		:	Convert to Expression(Native format)
//
//	Argument		:	bCh			...	#Channel(0..15)
//						bExp		...	Expression(0..127)
//
//	Return			:	>0:Number of converted data  <0:Error Code
//
==============================================================================*/
static UINT32	CnvExpression(UINT8 bCh, UINT8 bExp)
{
	MAPHRCNV_DBGMSG(("CnvExpression[%d %02X] \n", bCh, bExp));

	if(gChInfo[bCh].bExp != bExp)
	{
		MaSndDrv_SetCommand(gdwPhrId, -1L, MASNDDRV_CMD_EXPRESSION, (UINT32)bCh, (UINT32)bExp, 0L);
		gChInfo[bCh].bExp = bExp;
		return (1L);
	}
	return (0L);
}

/*==============================================================================
//	Function Name	:	UINT32	CnvPanpot(UINT8 bCh, UINT8 bPan)
//
//	Description		:	Convert to Panpot(Native format)
//
//	Argument		:	bCh			...	#Channel(0..15)
//						bPan		...	Panpote(0..127)
//
//	Return			:	>0:Number of converted data  <0:Error Code
//
==============================================================================*/
static UINT32	CnvPanpot(UINT8 bCh, UINT8 bPan)
{
	MAPHRCNV_DBGMSG(("CnvPanpot[%d %02X] \n", bCh, bPan));

	if(gChInfo[bCh].bPan != bPan)
	{
		MaSndDrv_SetCommand(gdwPhrId, -1L, MASNDDRV_CMD_PANPOT, (UINT32)bCh, (UINT32)bPan, 0L);
		gChInfo[bCh].bPan = bPan;
		return (1L);
	}
	return (0L);
}

/*==============================================================================
//	Function Name	:	UINT32	CnvPitchBend(UINT8 bCh, UINT8 bPitch)
//
//	Description		:	Convert to PitchBend(Native format)
//
//	Argument		:	bCh			...	#Channel(0..15)
//						bPitch		...	Pitch(0..127)
//
//	Return			:	>0:Number of converted data  <0:Error Code
//
==============================================================================*/
static UINT32	CnvPitchBend(UINT8 bCh, UINT8 bPitch)
{
	UINT16	wPitch;

	MAPHRCNV_DBGMSG(("CnvPitchBend[%d %02X] \n", bCh, bPitch));

	gChInfo[bCh].bPitch = bPitch;
	wPitch = (UINT16)((UINT16)bPitch << 7);
	MaSndDrv_SetCommand(gdwPhrId, -1L, MASNDDRV_CMD_PITCH_BEND, (UINT32)bCh, (UINT32)wPitch, 0L);
	return (1L);
}

/*==============================================================================
//	Function Name	:	UINT32	CnvNoteOn(UINT8 bCh, UINT8 bOct, UINT8 bKey, UINT32 dwNoteOffTime)
//
//	Description		:	Convert to NoteOn(Native format)
//
//	Argument		:	bCh				...	#Channel(0..15)
//						bOct			...	Octave Shift(0..3)
//						bKey			...	#Key(1..12)
//						dwNoteOffTime	...	NoteOff Time
//
//	Return			:	0:NoError  <0:Error Code
//
==============================================================================*/
static UINT32	CnvNoteOn(UINT8 bCh, UINT8 bOct, UINT8 bKey, UINT32 dwNoteOffTime)
{
	UINT8	bNote;
	SINT16	nOct;
	SINT16	nNote;

	nOct	= (SINT16)(bOct + gChInfo[bCh].nOctShift + 3);
	nNote	= (SINT16)((nOct * 12) + bKey);

	/*	Process for when nNote is out of the range (Note[0..127])	*/
	if(nNote < 0)			bNote = 0;
	else if(nNote >= 127)	bNote = 127;
	else					bNote = (UINT8)nNote;

	gChInfo[bCh].bKey			= bNote;
	gChInfo[bCh].dwKeyOffTime	= dwNoteOffTime;

	/*	Velocity 127 fixed */
	/*
	MAPHRCNV_DBGMSG(("CnvNoteOn[%d %d %02X %08lX] -> [%02X]\n", bCh, bOct, bKey, dwNoteOffTime, bNote));
	*/

	MaSndDrv_SetCommand(gdwPhrId, -1L, MASNDDRV_CMD_NOTE_ON_MA2, (UINT32)bCh, (UINT32)bNote, 0x7FL);
	return (1L);
}

/*==============================================================================
//	Function Name	:	UINT32	CnvNoteOff(UINT8 bCh)
//
//	Description		:	Convert to NoteOff(Native format)
//
//	Argument		:	bCh			...	#Channel(0..15)
//
//	Return			:	0:NoError  <0:Error Code
//
==============================================================================*/
static UINT32	CnvNoteOff(UINT8 bCh)
{
	/*
	MAPHRCNV_DBGMSG(("CnvNoteOff[%d] \n", bCh));
	*/

	if(gChInfo[bCh].dwKeyOffTime != 0xFFFFFFFF) {
		MaSndDrv_SetCommand(gdwPhrId, -1L, MASNDDRV_CMD_NOTE_OFF_MA2, (UINT32)bCh, (UINT32)gChInfo[bCh].bKey, 0L);
		gChInfo[bCh].dwKeyOffTime = 0xFFFFFFFF;
		return (1L);
	}

	return (0L);
}

/*==============================================================================
//	Function Name	:	UINT32	CnvExclusiveMessage(UINT8 bPhrNum, UINT8* pbExMsg, UINT32 dwSize)
//
//	Description		:	Convert to ExclusiveMessage(Native format)
//
//	Argument		:	bPhrNum	...	#Phrase(0..3)
//						pbExMsg	...	pointer to exclusive message.
//						dwSize	...	size of exclusive message.
//
//	Return			:	0:NoError  <0:Error Code
//
==============================================================================*/
static UINT32	CnvExclusiveMessage(UINT8 bPhrNum, UINT8* pbExMsg, UINT32 dwSize)
{
	UINT8	bCh;
	UINT8	bRange;
	UINT32	dwBlkFnum;
	UINT32	dwRet;
	PCHINFO	pCi;
	PVOCINFO pVi;

	MAPHRCNV_DBGMSG(("CnvExclusiveMessage[%d %08lX %08lX] \n", bPhrNum, pbExMsg, dwSize));

	dwRet = 0L;
	if(dwSize == 6L)
	{
		if(	(pbExMsg[0] == 0x43)	&&
			(pbExMsg[1] == 0x03)	&&
			(pbExMsg[2] == 0x90)	&&
			(pbExMsg[5] == 0xF7))			/*	Extend NoteOn/Off	*/
		{
			bCh		= (UINT8)((pbExMsg[3] & 0x03) + (bPhrNum << 2));
			pCi		= &(gChInfo[bCh]);

			if((pbExMsg[3] & 0xF0) == 0xC0) {
				pCi->bExNoteFlag |= 0x02;
				pCi->bRegH = pbExMsg[4];
			}
			else if((pbExMsg[3] & 0xF0) == 0xB0) {
				pCi->bExNoteFlag |= 0x01;
				pCi->bRegL = pbExMsg[4];
			}
			else {
				return (0L);
			}

			if((pCi->bExNoteFlag & 0x03) == 0x03)
			{
				dwBlkFnum	= (UINT32)(((UINT32)(pCi->bRegH & ~0x20) << 8) | (UINT32)pCi->bRegL);
				dwBlkFnum	|= 0x80000000;
				if((pCi->bRegH & 0x20) == 0x20) {
					if (pCi->bNew) {
						pVi = &(gApiInfo[bPhrNum].gPhraseInfo.VocInfo[pCi->bVoiceNo]);
						MAPHRCNV_DBGMSG(("ProgramChange[%d %d %d %d] \n", bCh, pCi->bVoiceNo, pVi->bBankNo, pVi->bProgNo));
						MaSndDrv_SetCommand(gdwPhrId, -1L, MASNDDRV_CMD_PROGRAM_CHANGE,	\
											 (UINT32)bCh, (UINT32)pVi->bBankNo, (UINT32)pVi->bProgNo);
						pCi->bNew = 0x00;
						dwRet++;
					}
					MaSndDrv_SetCommand(gdwPhrId, -1L, MASNDDRV_CMD_NOTE_ON_MA2EX,	 (UINT32)bCh, dwBlkFnum, 0x7FL);
				} else {
					MaSndDrv_SetCommand(gdwPhrId, -1L, MASNDDRV_CMD_NOTE_OFF_MA2EX, (UINT32)bCh, dwBlkFnum, 0L);
				}
				dwRet++;
			}
		}
		else if((pbExMsg[0] == 0x43)	&&
				(pbExMsg[1] == 0x04)	&&
				(pbExMsg[2] == 0x02)	&&
				(pbExMsg[5] == 0xF7))		/*	Pitch Bend Range	*/
		{
			bCh		= (UINT8)((pbExMsg[3] & 0x03) + (bPhrNum << 2));
			pCi		= &(gChInfo[bCh]);
			bRange	= pbExMsg[4];

			if((pCi->bRange != bRange) && (bRange <= 24)) {
				MaSndDrv_SetCommand(gdwPhrId, -1L, MASNDDRV_CMD_BEND_RANGE, (UINT32)bCh, (UINT32)bRange, 0L);
				pCi->bRange = bRange;
				dwRet++;
			}
		}
	}

	return (dwRet);
}

/*==============================================================================
//	Function Name	:	UINT32	CnvAllSoundOff(UINT8 bCh)
//
//	Description		:	Convert to AllSoundOff(Native format)
//
//	Argument		:	bCh			...	#Channel(0..15)
//
//	Return			:	>=0:Number of converted data  <0:Error Code
//
==============================================================================*/
static UINT32	CnvAllSoundOff(UINT8 bCh)
{
	MAPHRCNV_DBGMSG(("CnvAllSoundOff[%d] \n", bCh));

	MaSndDrv_SetCommand(gdwPhrId, -1L, MASNDDRV_CMD_ALL_SOUND_OFF, (UINT32)bCh, 0L, 0L);
	return (1L);
}

/*=============================================================================
//	Function Name	:	UINT32	PutInitPhraseMessage(UINT8 phr_id)
//
//	Description		:	Initialize Phrase commands message
//
//	Argument		:	phr_id	.... Phrase ID (0..3)
//
//	Return			:	0:NoError <0:Error Code
//
=============================================================================*/
static SINT32	PutInitPhraseMessage(UINT8 phr_id)
{
	UINT8		i;
	UINT8		bCh;
	UINT16		wPitch;
	PCHINFO		pCi;

	MAPHRCNV_DBGMSG(("PutInitPhraseMessage[%ld] \n", phr_id));

	for(i = 0; i < MAX_PHRASE_SLOT; i++)
	{
		bCh = (UINT8)(i + (phr_id << 2));
		pCi = &(gChInfo[bCh]);

		/*	All Sound Off	*/
		MaSndDrv_SetCommand(gdwPhrId, -1L, MASNDDRV_CMD_ALL_SOUND_OFF, (UINT32)bCh, 0L, 0L);

		/*	Program Change	*/
		/*	Send command with next NoteOn message	*/
		pCi->bNew = 1;

		/*	Channel Volume	*/
		MaSndDrv_SetCommand(gdwPhrId, -1L, MASNDDRV_CMD_CHANNEL_VOLUME, (UINT32)bCh, (UINT32)pCi->bVolume, 0L);

		/*	Expression		*/
		MaSndDrv_SetCommand(gdwPhrId, -1L, MASNDDRV_CMD_EXPRESSION, (UINT32)bCh, (UINT32)pCi->bExp, 0L);

		/*	Panpot			*/
		MaSndDrv_SetCommand(gdwPhrId, -1L, MASNDDRV_CMD_PANPOT, (UINT32)bCh, (UINT32)pCi->bPan, 0L);

		/*	Modulation		*/
		MaSndDrv_SetCommand(gdwPhrId, -1L, MASNDDRV_CMD_MODULATION_DEPTH, (UINT32)bCh, (UINT32)pCi->bMod, 0L);

		/*	BendRange		*/
		MaSndDrv_SetCommand(gdwPhrId, -1L, MASNDDRV_CMD_BEND_RANGE, (UINT32)bCh, (UINT32)pCi->bRange, 0L);

		/*	PitchBend		*/
		wPitch = (UINT16)((UINT16)pCi->bPitch << 7);
		MaSndDrv_SetCommand(gdwPhrId, -1L, MASNDDRV_CMD_PITCH_BEND, (UINT32)bCh, (UINT32)wPitch, 0L);
	}

	return (MASMW_SUCCESS);
}

/*=============================================================================
//	Function Name	:	void	InitChInfo(UINT8 bCh)
//
//	Description		:	Initialize Channel Information Struct
//
//	Argument		:	bCh	...	#Channel (0..15)
//
//	Return			:	void
//
=============================================================================*/
static void	InitChInfo(UINT8 bCh)
{
	UINT8	bPhrNum;
	PCHINFO	pCi;

	if(bCh >= MASMW_NUM_CHANNEL)	return;

	pCi = &(gChInfo[bCh]);
	pCi->bNew			= 0x00;
	pCi->bKey			= 0x00;
	pCi->bVoiceNo		= 0x00;			/*	VoiceNo			(0..3)				*/
	pCi->bVolume		= 0x64;			/*	Channel Volume	(0..127)			*/
	pCi->bMod			= 0;			/*	Modulation	(0..4)					*/
	pCi->bPitch			= 0x40;			/*	PitchBend	(0..127)				*/
	pCi->bRange			= 2;			/*	PitchBend Range	(100cent unit)		*/
	pCi->bExNoteFlag	= 0x00;			/*	bit0 : Low	bit1: High				*/
	pCi->bRegL			= 0x00;			/*	Extended Note High-Byte value		*/
	pCi->bRegH			= 0x00;			/*	Extended Note Low-Byte value		*/
	pCi->nOctShift		= 0;			/*	Octave Shift (-4..0..4)				*/
	pCi->dwKeyOffTime	= 0xFFFFFFFF;	/*	KeyOff Time							*/

	bPhrNum		= (UINT8)(bCh >> 2);
	pCi->bExp	= gApiInfo[bPhrNum].vol;
	pCi->bPan	= gApiInfo[bPhrNum].pan;
}


/*=============================================================================
//	Function Name	:	SINT32	Seek(UINT8 phr_id, UINT32 pos)
//
//	Description		:	Seek process of converter data
//
//	Argument		:	phr_id		...	Phrase ID (0..3)
//						pos			...	Starting position of play back (msec)
//
//	Return			:	0:NoError  <0:Error Code
//
=============================================================================*/
static SINT32	Seek(UINT8 phr_id, UINT32 pos)
{
	UINT8		i;
	UINT8		bCh;
	UINT8		bData0;
	UINT8		bData1;
	UINT8*		pbExMsg;
	UINT32		dwSize;
	UINT32		dwSeekTime;
	UINT32		dwCurrent;
	UINT32		dwDuration;
	PCHINFO		pCi;
	PPHRINFO	pPi;

	MAPHRCNV_DBGMSG(("Seek[%08lX %08lX %02X] \n", phr_id, pos, gbPlayStatus));

	dwSeekTime = (UINT32)(pos / PHRASE_TIMEBASE);
	pPi = &(gApiInfo[phr_id].gPhraseInfo);
	if(dwSeekTime >= pPi->dwPlayTime)					return (MASMW_ERROR);

	pPi->dwTimer		= 0L;
	pPi->dwCurrentTime	= 0L;
	pPi->dwDataPosition	= 0L;

	/*	Initialize each channel info	*/
	for(i = 0; i < MAX_PHRASE_SLOT; i++) {
		bCh = (UINT8)(i + (phr_id << 2));
		InitChInfo(bCh);
	}

	/*	Seek the position	*/
	while(pPi->dwSequenceChunkSize > pPi->dwDataPosition)
	{
		/*	Get Current Time	*/
		dwCurrent = pPi->dwDataPosition;
		dwDuration = (UINT32)pPi->pbSequenceChunk[dwCurrent++];
		if(dwDuration & 0x80) {
			dwDuration = (UINT32)(((dwDuration & 0x7F) << 7) + (UINT32)pPi->pbSequenceChunk[dwCurrent++] + 128L);
		}

		if((pPi->dwCurrentTime + dwDuration) >= dwSeekTime) {
			pPi->dwTimer = dwSeekTime;
			break;
		}

		/*	Update position & time	*/
		pPi->dwDataPosition	= dwCurrent;
		pPi->dwCurrentTime	+= dwDuration;

		/**/
		bData0 = pPi->pbSequenceChunk[pPi->dwDataPosition++];
		switch(bData0)
		{
		case 0x00 :
			bData1	= pPi->pbSequenceChunk[pPi->dwDataPosition++];
			bCh		= (UINT8)(((bData1 & 0xC0) >> 6) + (phr_id << 2));
			switch(bData1 & 0x30)
			{
			case 0x00 :	/*	Short type Volume	*/
				gChInfo[bCh].bVolume = gbVolTbl[bData1 & 0x0F];
				break;
			case 0x10 : /*	*/
				break;
			case 0x20 :	/*	Short type Modulation	*/
				gChInfo[bCh].bMod = gbModTbl[bData1 & 0x0F];
				break;
			case 0x30 :
				switch(bData1 & 0x0F)
				{
				case 0x00 :
					gChInfo[bCh].bVoiceNo = (UINT8)(pPi->pbSequenceChunk[pPi->dwDataPosition++] & 0x03);
					break;
				case 0x02 :
					OctaveShift(bCh, pPi->pbSequenceChunk[pPi->dwDataPosition++]);
					break;
				case 0x04 :
					gChInfo[bCh].bPitch = (UINT8)(pPi->pbSequenceChunk[pPi->dwDataPosition++] & 0x7F);
					break;
				case 0x0A :
					gChInfo[bCh].bPan = (UINT8)(pPi->pbSequenceChunk[pPi->dwDataPosition++] & 0x7F);
					break;
				case 0x0B :
					gChInfo[bCh].bVolume = (UINT8)(pPi->pbSequenceChunk[pPi->dwDataPosition++] & 0x7F);
					break;
				default	  :
					pPi->dwDataPosition++;
					break;
				}
				break;
			default	 :
				break;
			}
			break;
		case 0xFF :
			bData1 = pPi->pbSequenceChunk[pPi->dwDataPosition++];
			if(bData1 == 0xF0) {				/*	Exclusive Message	*/
				dwSize	= pPi->pbSequenceChunk[pPi->dwDataPosition++];
				pbExMsg	= &(pPi->pbSequenceChunk[pPi->dwDataPosition]);

				if(dwSize == 6L)
				{
					if(	(pbExMsg[0] == 0x43)	&&
						(pbExMsg[1] == 0x03)	&&
						(pbExMsg[2] == 0x90)	&&
						(pbExMsg[5] == 0xF7))			/*	Extend NoteOn/Off	*/
					{
						bCh		= (UINT8)((pbExMsg[3] & 0x03) + (phr_id << 2));
						pCi		= &(gChInfo[bCh]);
						if((pbExMsg[3] & 0xF0) == 0xC0) {
							pCi->bExNoteFlag |= 0x02;
							pCi->bRegH = pbExMsg[4];
						}
						else if((pbExMsg[3] & 0xF0) == 0xB0) {
							pCi->bExNoteFlag |= 0x01;
							pCi->bRegL = pbExMsg[4];
						}
					}
					else if((pbExMsg[0] == 0x43)	&&
							(pbExMsg[1] == 0x04)	&&
							(pbExMsg[2] == 0x02)	&&
							(pbExMsg[5] == 0xF7))		/*	Pitch Bend Range	*/
					{
						bCh	= (UINT8)((pbExMsg[3] & 0x03) + (phr_id << 2));
						pCi	= &(gChInfo[bCh]);
						pCi->bRange	= (UINT8)((pbExMsg[4] <= 24) ? pbExMsg[4] : 24);
					}
				}

				/*	skip exclusive message	*/
				pPi->dwDataPosition += dwSize;
			}
			break;
		default	  :
			dwDuration = (UINT32)pPi->pbSequenceChunk[pPi->dwDataPosition++];
			if(dwDuration & 0x80) {
				pPi->dwDataPosition++;
			}
			break;
		}
	}

	return (MASMW_SUCCESS);
}


/*=============================================================================
//	Function Name	:	SINT32	MaPhr_Load(UINT8 phr_id, UINT8* file_ptr, UINT32 file_size, UINT8 mode,
//												SINT32 (*func)(UINT8 id), void* ext_args)
//
//	Description		:	Load data
//
//	Argument		:	phr_id		... Phrase ID (0..4)
//						file_ptr	...	Pointer to the data
//						file_size	...	Data size (Byte)
//						mode		...	Error check (0:No  1:Yes  2:Check Only)
//						func		...	Pointer to the call back function
//						ext_args	...	NULL
//
//	Return			:	0:NoError  <0:Error Code
//
=============================================================================*/
static SINT32	MaPhr_Load(UINT8 phr_id, UINT8* file_ptr, UINT32 file_size, UINT8 mode, SINT32 (*func)(UINT8 id), void* ext_args)
{
	UINT8	bMode;
/*	UINT16	wErrStatus; */
	SINT32	sdResult;
	(void)ext_args;

	MAPHRCNV_DBGMSG(("MaPhr_Load\n"));

	/*	set register number	*/
	bMode = 0x00;
	if(mode >= 1) {
		bMode |= CHKMODE_ENABLE;
		if(mode == 3) bMode |= CHKMODE_CNTIONLY;
	}

	/*	Check SMAF/Phrase data	*/
/*
	wErrStatus = PHRASE_DATA_NOERROR;
	SmafPhrChecker(phr_id, bMode, file_ptr, file_size, &wErrStatus);
	if(wErrStatus) {
		MAPHRCNV_DBGMSG(("Error : Invalid SMAF/Phrase [%04X] \n", wErrStatus));

		if(wErrStatus & PHRASE_DATA_SIZE)		return (MASMW_ERROR_CHUNK_SIZE);
		if(wErrStatus & PHRASE_DATA_CLASS)		return (MASMW_ERROR_CONTENTS_CLASS);
		if(wErrStatus & PHRASE_DATA_TYPE)		return (MASMW_ERROR_CONTENTS_TYPE);
		if(wErrStatus & PHRASE_DATA_CHUNK)		return (MASMW_ERROR_CHUNK);
		if(wErrStatus & PHRASE_DATA_SHORTLEN)	return (MASMW_ERROR_SHORT_LENGTH);

		return (MASMW_ERROR_FILE);
	}
*/
	sdResult = SmafPhrChecker(phr_id, bMode, file_ptr, file_size);
	if (sdResult < 0)			return sdResult;

	gApiInfo[phr_id].gPhraseInfo.CallbackFunc = func;

	return (MASMW_SUCCESS);
}

/*=============================================================================
//	Function Name	:	SINT32	MaPhr_Standby(UINT8 phr_id, void* ext_args)
//
//	Description		:	Standby process of converter data (register voices)
//
//	Argument		:	phr_id		...	Phrase ID (0..3)
//						ext_args	...	NULL
//
//	Return			:	0:NoError <0:Error Code
//
=============================================================================*/
static SINT32	MaPhr_Standby(UINT8 phr_id, void* ext_args)
{
	UINT8		i;
	UINT8		bCh;
	UINT8		bVoiceParam[MA3VOICE_PARAM_SIZE];
	UINT32		dwVocAdr;
	UINT32		dwSize;
	PPHRINFO	pPi;
	PVOCINFO	pVi;
	(void)ext_args;

	MAPHRCNV_DBGMSG(("MaPhr_Standby [%X]\n", phr_id));

	pPi = &(gApiInfo[phr_id].gPhraseInfo);

	pPi->dwTimer		= 0L;
	pPi->dwCurrentTime	= 0L;
	pPi->dwDataPosition	= 0L;

	/*	Initialize each channel info	*/
	for(i = 0; i < MAX_PHRASE_SLOT; i++) {
		bCh = (UINT8)(i + (phr_id << 2));
		InitChInfo(bCh);
	}

	dwVocAdr = gApiInfo[phr_id].gdwRamAddr;
	MAPHRCNV_DBGMSG((" dwVocAdr [%X]\n", dwVocAdr));
	if (dwVocAdr != 0xFFFFFFFF) {
	for(i = 0; i < MAX_PHRASE_VOICES; i++)
	{
		/*	extend voice	*/
		pVi = &(pPi->VocInfo[i]);
		MAPHRCNV_DBGMSG((" bBankNo [%X]\n", pVi->bBankNo));
		if(pVi->bBankNo != 0x00) {
			MAPHRCNV_DBGMSG((" bType [%X]\n", pVi->bType));
			if(pVi->bType == VOICE_TYPE_MA2) {	/*	MA-2 voice type	*/
				dwSize = ConvertMA3Voice(pVi->pbVoice, pVi->bSize, &(bVoiceParam[0]));
				MaDevDrv_SendDirectRamData(dwVocAdr, 0, &(bVoiceParam[0]), dwSize);
			}
			else {								/*	MA-3 voice type	*/
				machdep_memcpy(&(bVoiceParam[0]), pVi->pbVoice, pVi->bSize);
				/* Prohibit XOF=1                                               */
				bVoiceParam[2] = (UINT8)(bVoiceParam[2] & 0xF7);
				bVoiceParam[9] = (UINT8)(bVoiceParam[9] & 0xF7);
				if (pVi->bSize > 16) {
					bVoiceParam[16] = (UINT8)(bVoiceParam[16] & 0xF7);
					bVoiceParam[23] = (UINT8)(bVoiceParam[23] & 0xF7);
				}
				MaDevDrv_SendDirectRamData(dwVocAdr, 0, &(bVoiceParam[0]), (UINT32)pVi->bSize);
			}

			/*	Remove	*/
			MaSndDrv_SetVoice(	gdwPhrId,
								pVi->bBankNo,	/*	Bank Number					*/
								pVi->bProgNo,	/*	Program Number				*/
								(UINT8)1,		/*	Synth Mode					*/
								(UINT8)0,		/*	Durum Key					*/
								0xFFFFFFFF);	/*	Registered voice address	*/
			/*	Set		*/
			MaSndDrv_SetVoice(	gdwPhrId,
								pVi->bBankNo,	/*	Bank Number					*/
								pVi->bProgNo,	/*	Program Number				*/
								(UINT8)1,		/*	Synth Mode					*/
								(UINT8)0,		/*	Durum Key					*/
								dwVocAdr);		/*	Registered voice address	*/

			dwVocAdr += MA3VOICE_PARAM_SIZE;
		}
	}
	}

	return (MASMW_SUCCESS);
}

/*=============================================================================
//	Function Name	:	SINT32	MaPhr_Seek(UINT8 phr_id, UINT32 pos, UINT8 flag, void* ext_args)
//
//	Description		:	Seek process of converter data
//
//	Argument		:	phr_id		...	Phrase ID (0..3)
//						pos			...	Starting position of play back (msec)
//						flag		...	Wait mode 0:No wait before start 1:Wait before start
//						ext_args	...	NULL
//
//	Return			:	0:NoError  <0:Error Code
//
=============================================================================*/
SINT32	MaPhr_Seek(UINT8 phr_id, UINT32 pos, UINT8 flag, void* ext_args)
{
	UINT8	i;
	UINT32	dwSlave;
	SINT32	lRet = MASMW_SUCCESS;
	(void)flag;
	(void)ext_args;

	MAPHRCNV_DBGMSG(("MaPhr_Seek\n"));

	/*	Master Seek	*/
	lRet = Seek(phr_id, pos);
	if(lRet != MASMW_SUCCESS) {
		return (lRet);
	}

	/*	Synchronization control	*/
	dwSlave = gApiInfo[phr_id].slave;
	for(i = 0; i < MAX_PHRASE_CHANNEL; i++) {
		if(dwSlave & (UINT32)(0x01L << i)) {	/*	synchronization	*/
			if (gApiInfo[i].status != ID_STATUS_READY)
				continue;
			lRet = Seek(i, pos);
			if (lRet != MASMW_SUCCESS) continue;
		}
	}


	return (MASMW_SUCCESS);
}

/*=============================================================================
//	Function Name	:	SINT32	MaPhr_Start(UINT8 phr_id, UINT16 loop)
//
//	Description		:	Start process of converter data
//
//	Argument		:	phr_id		...	Phrase ID (0..3)
//						loop		...	loop count
//
//	Return			:	0:NoError  <0:Error Code
//
=============================================================================*/
static SINT32	MaPhr_Start(UINT8 phr_id, UINT16 loop)
{
	UINT8	i;
	UINT32	dwSlave;

	MAPHRCNV_DBGMSG(("MaPhr_Start[Loop:%04X] \n", loop));

	/*	Master Start	*/
	if (gApiInfo[phr_id].gPhraseInfo.dwPlayTime <= MIN_PHRASE_DATA_LENGTH)
		return (MASMW_ERROR);
	gApiInfo[phr_id].loop = loop;
	gApiInfo[phr_id].status = ID_STATUS_PLAY;
	gdwPhrStatus |= gMaskStart[phr_id];
	
	/*	Synchronization control	*/
	dwSlave = gApiInfo[phr_id].slave;
	for(i = 0; i < MAX_PHRASE_CHANNEL; i++) {
		if(dwSlave & (UINT32)(0x01L << i)) {	/*	synchronization	*/
			if (gApiInfo[i].status != ID_STATUS_READY) continue;
			if (gApiInfo[i].gPhraseInfo.dwPlayTime <= MIN_PHRASE_DATA_LENGTH)
				continue;
			gApiInfo[i].loop = loop;
			gApiInfo[i].status = ID_STATUS_PLAY;
			gdwPhrStatus |= gMaskStart[i];
		}
	}

	/* control sequencer */
	if (!(gbSeqStatus & MASK_STATUS_PHRASE)) {
		gbSeqStatus |= MASK_STATUS_PHRASE;
		MaSndDrv_ControlSequencer(gdwPhrId, 1);
	}

	return (MASMW_SUCCESS);
}

/*=============================================================================
//	Function Name	:	SINT32	MaPhr_Stop(UINT8 phr_id)
//
//	Description		:	Stop process of converter data
//
//	Argument		:	phr_id		...	Phrase ID (0..3)
//
//	Return			:	0:NoError  <0:Error Code
//
=============================================================================*/
static SINT32	MaPhr_Stop(UINT8 phr_id)
{
#if !MA_STOPWAIT
	UINT8		bCh;
#endif
	UINT8		i;
	UINT32		dwSlave;
	UINT32		dwMask;
#if MA_STOPWAIT
	SINT32		lTimeOut;
#endif
	IDSTATUS	status;

	MAPHRCNV_DBGMSG(("MaPhr_Stop[%d %02X] \n", phr_id, gbPlayStatus));

	dwMask = 0;

	/*	Master Stop	*/
#if MA_STOPWAIT
	gApiInfo[phr_id].status = ID_STATUS_ENDING;
	dwMask |= gMaskStop[phr_id];
	gdwPhrStatus |= gMaskStop[phr_id];
#else
	gdwPhrStatus &= ~(gMaskStart[phr_id]|gMaskPause[phr_id]|gMaskRestart[phr_id]);
	/* Mute */
	bCh = (UINT8)(phr_id << 2);
	CnvAllSoundOff(bCh);
	CnvAllSoundOff((UINT8)(bCh+1));
	CnvAllSoundOff((UINT8)(bCh+2));
	CnvAllSoundOff((UINT8)(bCh+3));

	gbPauseStatus &= ~gMaskPlay[phr_id];
	gbPlayStatus &= ~gMaskPlay[phr_id];
	gApiInfo[phr_id].status = ID_STATUS_READY;

	/* seek to top */
	Seek(phr_id, 0L);
#endif

	/*	Synchronization control	*/
	dwSlave = gApiInfo[phr_id].slave;
	for(i = 0; i < MAX_PHRASE_CHANNEL; i++) {
		if(dwSlave & (UINT32)(0x01L << i)) {	/*	synchronization	*/
			status = gApiInfo[i].status;
#if 0
			if(status == ID_STATUS_READY) continue;
#endif
			if((status != ID_STATUS_PLAY) && (status != ID_STATUS_PAUSE))
				continue;
#if MA_STOPWAIT
			gApiInfo[i].status = ID_STATUS_ENDING;
			dwMask |= gMaskStop[i];
			gdwPhrStatus |= gMaskStop[i];
#else
			gdwPhrStatus &= ~(gMaskStart[i] | gMaskPause[i] | gMaskRestart[i]);
			/* Mute */
			bCh = (UINT8)(i << 2);
			CnvAllSoundOff(bCh);
			CnvAllSoundOff((UINT8)(bCh+1));
			CnvAllSoundOff((UINT8)(bCh+2));
			CnvAllSoundOff((UINT8)(bCh+3));

			gbPauseStatus &= ~gMaskPlay[i];
			gbPlayStatus &= ~gMaskPlay[i];
			gApiInfo[i].status = ID_STATUS_READY;

			/* seek to top */
			Seek(i, 0L);
#endif
		}
	}

#if MA_STOPWAIT
	lTimeOut = PHRASE_WAIT_TIMEOUT;
	while ((gdwPhrStatus & dwMask) && lTimeOut > 0) {
		machdep_Sleep(5);
		lTimeOut -= 5;
	}
#endif

	/*	No played Phrase	*/
	if((gbPlayStatus | gbPauseStatus | gdwPhrStatus) == 0x00) {
		if (gbSeqStatus  & MASK_STATUS_PHRASE) {
			gbSeqStatus &= ~ MASK_STATUS_PHRASE;
			MaSndDrv_ControlSequencer(gdwPhrId, 0);
		}
	}

	return (MASMW_SUCCESS);
}

/*=============================================================================
//	Function Name	:	SINT32	MaPhr_Restart(UINT8 phr_id)
//
//	Description		:	Restart process of converter data
//
//	Argument		:	phr_id		...	Phrase ID (0..3)
//
//	Return			:	0:NoError  <0:Error Code
//
=============================================================================*/
static SINT32	MaPhr_Restart(UINT8 phr_id)
{
	UINT8	i;
	UINT32	dwSlave;

	MAPHRCNV_DBGMSG(("MaPhr_Restart[%d] \n", phr_id));

	/*	Master Restart	*/
	gApiInfo[phr_id].status = ID_STATUS_PLAY;
	if (gdwPhrStatus & gMaskPause[phr_id])
		gdwPhrStatus &= ~gMaskPause[phr_id];
	else
		gdwPhrStatus |= gMaskRestart[phr_id];
	
	/*	Synchronization control	*/
	dwSlave = gApiInfo[phr_id].slave;
	for(i = 0; i < MAX_PHRASE_CHANNEL; i++) {
		if(dwSlave & (UINT32)(0x01L << i)) {	/*	synchronization	*/
			if (gApiInfo[i].status != ID_STATUS_PAUSE)
				continue;
			gApiInfo[i].status = ID_STATUS_PLAY;
			if (gdwPhrStatus & gMaskPause[i])
				gdwPhrStatus &= ~gMaskPause[i];
			else
				gdwPhrStatus |= gMaskRestart[i];
		}
	}

	/* control sequencer */
	if (!(gbSeqStatus & MASK_STATUS_PHRASE)) {
		gbSeqStatus |= MASK_STATUS_PHRASE;
		MaSndDrv_ControlSequencer(gdwPhrId, 1);
	}

	return (MASMW_SUCCESS);
}

/*=============================================================================
//	Function Name	:	SINT32	MaPhr_Pause(UINT8 phr_id)
//
//	Description		:	Stop process of converter data
//
//	Argument		:	phr_id		...	Phrase ID (0..3)
//
//	Return			:	0:NoError  <0:Error Code
//
=============================================================================*/
static SINT32	MaPhr_Pause(UINT8 phr_id)
{
	UINT8		i;
	UINT32		dwSlave;
	IDSTATUS	status;

	MAPHRCNV_DBGMSG(("MaPhr_Pause[%d %02X] \n", phr_id, gbPlayStatus));

	/*	Master Pause	*/
	gApiInfo[phr_id].status = ID_STATUS_PAUSE;
	if (gdwPhrStatus & gMaskRestart[phr_id])
		gdwPhrStatus &= ~gMaskRestart[phr_id];
	else
		gdwPhrStatus |= gMaskPause[phr_id];

	/*	Synchronization control	*/
	dwSlave = gApiInfo[phr_id].slave;
	for(i = 0; i < MAX_PHRASE_CHANNEL; i++) {
		if(dwSlave & (UINT32)(0x01L << i)) {	/*	synchronization	*/
			status = gApiInfo[i].status;
			if(gApiInfo[i].status != ID_STATUS_PLAY)
				continue;
			gApiInfo[i].status = ID_STATUS_PAUSE;
			if (gdwPhrStatus & gMaskRestart[i])
				gdwPhrStatus &= ~gMaskRestart[i];
			else
				gdwPhrStatus |= gMaskPause[i];
		}
	}

	return (MASMW_SUCCESS);
}


/*=============================================================================
//	Function Name	:	SINT32	MaPhr_Convert(void)
//
//	Description		:	Convert SMAF/Phraes to Native format
//
//	Argument		:	N/A
//
//	Return			:	Number of convertion events (0:NoEvent)
//
=============================================================================*/
SINT32	MaPhr_Convert(void)
{
	UINT8		i;
	UINT8		j;
	UINT8		bCh;
	UINT8		bContinue;
	UINT8		bData0;
	UINT8		bData1;
	UINT8*		pbExMsg;
	UINT32		dwMask;
	UINT32		dwSize;
	UINT32		dwCurrent;
	UINT32		dwDuration;
	UINT32		dwNoteOffTime;
	UINT32		dwPhrStatusB;
	SINT32		lCnvData;
	PPHRINFO	pPi;

	dwPhrStatusB = gdwPhrStatus;
	gdwPhrStatus = 0;

	lCnvData = 0L;

	/*	set Control parameter */
	if (gdwCtrlStatus) {
		/*	System-On	*/
		if(gdwCtrlStatus & CTRL_SYSTEM_ON) {
			MaSndDrv_SetCommand(gdwPhrId, -1L, MASNDDRV_CMD_SYSTEM_ON, 0L, 0L, 0L);
			gdwCtrlStatus &= ~CTRL_SYSTEM_ON;
			lCnvData++;
		}
		/*	Master Volume	*/
		if(gdwCtrlStatus & CTRL_MASTER_VOLUME) {
			MaSndDrv_SetCommand(gdwPhrId, -1L, MASNDDRV_CMD_MASTER_VOLUME,
			                    (UINT32)0x7F, 0L, 0L);
			gdwCtrlStatus &= ~CTRL_MASTER_VOLUME;
			lCnvData++;
		}
	}

	/* control start/stop/pause/restart */
	if (dwPhrStatusB) {
		/* Start */
		if (dwPhrStatusB & MASK_PHRASE_STATUS_START) {
			for (i = 0; i < MAX_PHRASE_CHANNEL; i++) {
				if (dwPhrStatusB & gMaskStart[i]) {
					gbPlayStatus |= gMaskPlay[i];
					lCnvData += PutInitPhraseMessage(i);
				}
			}
		}
		/* Restart */
		if (dwPhrStatusB & MASK_PHRASE_STATUS_RESTART) {
			for (i = 0; i < MAX_PHRASE_CHANNEL; i++) {
				if (dwPhrStatusB & gMaskRestart[i]) {
					gbPauseStatus &= ~gMaskPlay[i];
					gbPlayStatus |= gMaskPlay[i];
				}
			}
		}
		/* Pause */
		if (dwPhrStatusB & MASK_PHRASE_STATUS_PAUSE) {
			for (i = 0; i < MAX_PHRASE_CHANNEL; i++) {
				if (dwPhrStatusB & gMaskPause[i]) {
					/* Mute */
					bCh = (UINT8)(i << 2);
					lCnvData += CnvAllSoundOff(bCh);
					lCnvData += CnvAllSoundOff((UINT8)(bCh+1));
					lCnvData += CnvAllSoundOff((UINT8)(bCh+2));
					lCnvData += CnvAllSoundOff((UINT8)(bCh+3));

					gbPauseStatus |= gMaskPlay[i];
					gbPlayStatus &= ~gMaskPlay[i];
				}
			}
		}
		/* Stop */
		if (dwPhrStatusB & MASK_PHRASE_STATUS_STOP) {
#if MA_STOPWAIT
			for (i = 0; i < MAX_PHRASE_CHANNEL; i++) {
				if (dwPhrStatusB & gMaskStop[i]) {
					/* Mute */
					bCh = (UINT8)(i << 2);
					lCnvData += CnvAllSoundOff(bCh);
					lCnvData += CnvAllSoundOff((UINT8)(bCh+1));
					lCnvData += CnvAllSoundOff((UINT8)(bCh+2));
					lCnvData += CnvAllSoundOff((UINT8)(bCh+3));

					gbPauseStatus &= ~gMaskPlay[i];
					gbPlayStatus &= ~gMaskPlay[i];
					gApiInfo[i].status = ID_STATUS_READY;

					/* seek to top */
					Seek(i, 0L);
				}
			}
#endif
		}
	}

	/*	No played Phrase	*/
	if((gbPlayStatus | gbPauseStatus) == 0x00) {
		if (gbSeqStatus & MASK_STATUS_PHRASE) {
			gbSeqStatus &= ~ MASK_STATUS_PHRASE;
			MaSndDrv_ControlSequencer(gdwPhrId, 0);
		}
		return lCnvData;
	}

	bContinue = gbPlayStatus;

	/*	Convert	*/
	for(i = 0; i < MAX_PHRASE_CHANNEL; i++) {
		pPi = &(gApiInfo[i].gPhraseInfo);
		if(!(gbPlayStatus & (UINT8)(0x01 << i))) {
			continue;
		}

		if (gdwCtrlStatus) {
#if 0
			/*	Send initialization message	*/
			dwMask = (UINT32)(CTRL_INIT_PHRASE << (i << 2));
			if(gdwCtrlStatus & dwMask) {
				lCnvData += PutInitPhraseMessage(i);
				gdwCtrlStatus &= ~dwMask;
			}
#endif
			/*	Expression	*/
			dwMask = (UINT32)(CTRL_VOLUME_CHANGE << (i << 2));
			if(gdwCtrlStatus & dwMask) {
				for(j = 0; j < MAX_PHRASE_SLOT; j++) {
					bCh = (UINT8)(j + (i << 2));
					lCnvData += CnvExpression(bCh, gApiInfo[i].vol);
				}
				gdwCtrlStatus &= ~dwMask;
			}

			/*	Panpot	*/
			dwMask = (UINT32)(CTRL_PANPOT_CHANGE << (i << 2));
			if(gdwCtrlStatus & dwMask) {
				for(j = 0; j < MAX_PHRASE_SLOT; j++) {
					bCh = (UINT8)(j + (i << 2));
					MaSndDrv_SetCommand(gdwPhrId, -1L, MASNDDRV_CMD_PANPOT, (UINT32)bCh, gChInfo[bCh].bPan, 0L);
					lCnvData ++;
				}
				gdwCtrlStatus &= ~dwMask;
			}
		}

		/*	Send Note Off	*/
		for(j = 0; j < MAX_PHRASE_SLOT; j++) {
			bCh = (UINT8)(j + (i << 2));
			if(gChInfo[bCh].dwKeyOffTime != 0xFFFFFFFF) {
				if(gChInfo[bCh].dwKeyOffTime <= pPi->dwTimer) {
					lCnvData += CnvNoteOff(bCh);
				}
			}
		}

		while(pPi->dwSequenceChunkSize > pPi->dwDataPosition)
		{
			/*	Get Current Time	*/
			dwCurrent = pPi->dwDataPosition;
			dwDuration = (UINT32)pPi->pbSequenceChunk[dwCurrent++];
			if(dwDuration & 0x80) {
				dwDuration = (UINT32)(((dwDuration & 0x7F) << 7) + (UINT32)pPi->pbSequenceChunk[dwCurrent++] + 128L);
			}

			if((pPi->dwCurrentTime + dwDuration) > pPi->dwTimer) {
				break;
			}

			/*	Update position & time	*/
			pPi->dwDataPosition	= dwCurrent;
			pPi->dwCurrentTime	+= dwDuration;

			/**/
			bData0 = pPi->pbSequenceChunk[pPi->dwDataPosition++];
			switch(bData0)
			{
			case 0x00 :
				bData1	= pPi->pbSequenceChunk[pPi->dwDataPosition++];
				bCh		= (UINT8)(((bData1 & 0xC0) >> 6) + (i << 2));
				switch(bData1 & 0x30)
				{
				case 0x00 :	/*	Short type Volume	*/
					lCnvData += CnvChannelVolume(bCh, gbVolTbl[bData1 & 0x0F]);
					break;
				case 0x10 : /*	*/
					break;
				case 0x20 :	/*	Short type Modulation	*/
					lCnvData += CnvModulation(bCh, gbModTbl[bData1 & 0x0F]);
					break;
				case 0x30 :
					switch(bData1 & 0x0F)
					{
					case 0x00 :
						ProgramChange(bCh, (UINT8)(pPi->pbSequenceChunk[pPi->dwDataPosition++] & 0x03));
						break;
					case 0x02 :
						OctaveShift(bCh, pPi->pbSequenceChunk[pPi->dwDataPosition++]);
						break;
					case 0x04 :
						lCnvData += CnvPitchBend(bCh, (UINT8)(pPi->pbSequenceChunk[pPi->dwDataPosition++] & 0x7F));
						break;
					case 0x0A :
						lCnvData += CnvPanpot(bCh, (UINT8)(pPi->pbSequenceChunk[pPi->dwDataPosition++] & 0x7F));
						break;
					case 0x0B :
						lCnvData += CnvChannelVolume(bCh, (UINT8)(pPi->pbSequenceChunk[pPi->dwDataPosition++] & 0x7F));
						break;
					default	  :
						pPi->dwDataPosition++;
						break;
					}
					break;
				default	 :
					break;
				}
				break;
			case 0xFF :
				bData1 = pPi->pbSequenceChunk[pPi->dwDataPosition++];
				if(bData1 == 0xF0) {				/*	Exclusive Message	*/
					dwSize	= (UINT32)(pPi->pbSequenceChunk[pPi->dwDataPosition++]);
					pbExMsg	= &(pPi->pbSequenceChunk[pPi->dwDataPosition]);

					lCnvData += CnvExclusiveMessage(pPi->bPhrNum, pbExMsg, dwSize);

					/*	skip exclusive message	*/
					pPi->dwDataPosition += dwSize;
				}
				else if((bData1 & 0xF0) == 0x10) {	/*	User Event	*/
					if(pPi->CallbackFunc != NULL) {
						pPi->CallbackFunc((UINT8)(bData1 & 0x0F));
					}
				}
				break;
			default	  :
				bCh = (UINT8)(((bData0 & 0xC0) >> 6) + (i << 2));

				/*	Send Program Chnage	*/
				if(gChInfo[bCh].bNew) {
					PVOCINFO	pVi = &(pPi->VocInfo[gChInfo[bCh].bVoiceNo]);

					MAPHRCNV_DBGMSG(("ProgramChange[%d %d %d %d] \n", bCh, gChInfo[bCh].bVoiceNo, pVi->bBankNo, pVi->bProgNo));

					MaSndDrv_SetCommand(gdwPhrId, -1L, MASNDDRV_CMD_PROGRAM_CHANGE,	\
										 (UINT32)bCh, (UINT32)pVi->bBankNo, (UINT32)pVi->bProgNo);
					gChInfo[bCh].bNew = 0x00;
					lCnvData++;
				}

				dwDuration = (UINT32)pPi->pbSequenceChunk[pPi->dwDataPosition++];
				if(dwDuration & 0x80) {
					dwDuration = (UINT32)(((dwDuration & 0x7F) << 7) +	\
											(UINT32)pPi->pbSequenceChunk[pPi->dwDataPosition++] + 128L);
				}
				if (dwDuration != 0) {
					dwNoteOffTime = (UINT32)(pPi->dwCurrentTime + dwDuration);
					lCnvData += CnvNoteOn(	bCh,
											(UINT8)((bData0 >> 4) & 0x03),
											(UINT8)(bData0 & 0x0F),
											dwNoteOffTime);
				}
				break;
			}
		}

		/*	End of Data	*/
		if(pPi->dwSequenceChunkSize <= pPi->dwDataPosition) {
			bContinue &= ~(UINT8)(0x01 << i);
		}

		/*	increment timer	*/
		pPi->dwTimer++;
	}

	/*	End of Phrase	*/
	if (bContinue != gbPlayStatus) { /* exist end phrase */
		for(i = 0; i < MAX_PHRASE_CHANNEL; i++) {
			if(!(gbPlayStatus & gMaskPlay[i])) {
				continue;
			}

			pPi = &(gApiInfo[i].gPhraseInfo);
			if(!(bContinue & gMaskPlay[i]))
			{
				MAPHRCNV_DBGMSG(("Phrase%d data end!! \n", i));

				/* Stop ***********/
				/* Note Off */
				bCh = (UINT8)(i << 2);
				lCnvData += CnvNoteOff(bCh);
				lCnvData += CnvNoteOff((UINT8)(bCh+1));
				lCnvData += CnvNoteOff((UINT8)(bCh+2));
				lCnvData += CnvNoteOff((UINT8)(bCh+3));

				gbPlayStatus &= ~gMaskPlay[i];

				/* Seek to top ****/
				Seek(i, 0L);

				/* Repeat? */
				if(gApiInfo[i].loop != 1) {	/*	repeat	*/
					if(gApiInfo[i].loop != 0) {
						gApiInfo[i].loop--;
					}

					/*	Callback	*/
					if(pPi->CallbackFunc != NULL) {
						pPi->CallbackFunc((UINT8)MASMW_REPEAT);
					}

					/* Start ******/
					gbPlayStatus |= gMaskPlay[i];
					lCnvData += PutInitPhraseMessage(i);

				} else {					/*	one short	*/
					if ((gbPlayStatus | gbPauseStatus) == 0) {
						if (gbSeqStatus & MASK_STATUS_PHRASE) {
							gbSeqStatus &= ~MASK_STATUS_PHRASE;
							MaSndDrv_ControlSequencer(gdwPhrId, 0);
						}
					}
					gApiInfo[i].status = ID_STATUS_READY;
					if(pPi->CallbackFunc != NULL) {
						pPi->CallbackFunc((UINT8)MASMW_END_OF_SEQUENCE);
					}
				}
			}
		}
	}
	if ((gbPlayStatus | gbPauseStatus) == 0) {
		if (gbSeqStatus & MASK_STATUS_PHRASE) {
			gbSeqStatus &= ~MASK_STATUS_PHRASE;
			MaSndDrv_ControlSequencer(gdwPhrId, 0);
		}
	}


	return (lCnvData);
}

/*=============================================================================
//	Function Name	:	SINT32	MaPhr_GetLength(UINT8 phr_id)
//
//	Description		:	Get Play back time
//
//	Argument		:	phr_id		...	Phrase ID (0..3)
//
//	Return			:	Play back time (ms)
//
=============================================================================*/
static SINT32	MaPhr_GetLength(UINT8 phr_id)
{
	MAPHRCNV_DBGMSG(("MaPhr_GetLength[%d] \n",phr_id));

	return (gApiInfo[phr_id].gPhraseInfo.dwPlayTime * PHRASE_TIMEBASE);
}

/*=============================================================================
//	Function Name	:	SINT32	MaPhr_GetPos(UINT8 phr_id)
//
//	Description		:	Get playing position
//
//	Argument		:	phr_id		...	Phrase ID (0..3)
//
//	Return			:	Playing position [ms]
//
=============================================================================*/
static SINT32	MaPhr_GetPos(UINT8 phr_id)
{
	SINT32	lPos;

	/*
	MAPHRCNV_DBGMSG(("MaPhr_GetPos[%d] \n", phr_id));
	*/

	lPos = (SINT32)gApiInfo[phr_id].gPhraseInfo.dwTimer;
	if(lPos > (SINT32)gApiInfo[phr_id].gPhraseInfo.dwPlayTime) {
		lPos = (SINT32)gApiInfo[phr_id].gPhraseInfo.dwPlayTime;
	}

	return (SINT32)(lPos * PHRASE_TIMEBASE);
}

/*=============================================================================
//	Function Name	:	SINT32	MaPhr_SetBind(UINT8 phr_id, UINT32 dwSlave)
//
//	Description		:	Set
//
//	Argument		:	phr_id		...	Phrase ID (0..3)
//					:	bSlave		...	Slave Phrase ID
//
//	Return			:	0:NoError  <0:Error Code
//
=============================================================================*/
static SINT32	MaPhr_SetBind(UINT8 phr_id, UINT8 bSlave)
{
	UINT8		i;
	PPHRINFO	pMaster;
	PPHRINFO	pSlave;

	MAPHRCNV_DBGMSG(("MaPhr_SetBind[%d, %08lX] \n", phr_id, bSlave));

	pMaster = &(gApiInfo[phr_id].gPhraseInfo);
	if(gApiInfo[phr_id].master_channel != -1)		return (MASMW_ERROR);
	if(gApiInfo[phr_id].status != ID_STATUS_READY)	return (MASMW_ERROR);

	/*	Register bind	*/
	if(phr_id != bSlave) {
		pSlave = &(gApiInfo[bSlave].gPhraseInfo);
		if(gApiInfo[bSlave].status != ID_STATUS_READY)		return (MASMW_ERROR);

		if(	(gApiInfo[bSlave].master_channel == -1)	&&		/*	Master			*/
			(gApiInfo[bSlave].slave == 0)	&&				/*	Not have slaves	*/
			(pMaster->dwPlayTime == pSlave->dwPlayTime))	/*	same playtime	*/
		{
			gApiInfo[bSlave].master_channel = phr_id;
			gApiInfo[phr_id].slave |= (UINT32)(1L << bSlave);
			MAPHRCNV_DBGMSG(("Successful : Register bind[%02X %08lX] \n", 
				gApiInfo[bSlave].master_channel,gApiInfo[phr_id].slave));
		} else if(gApiInfo[bSlave].master_channel == phr_id) {
			MAPHRCNV_DBGMSG(("Successful : Already bind[%08lX] \n", bSlave));
		} else {
			MAPHRCNV_DBGMSG(("Error : Register bind \n"));
			return (MASMW_ERROR);
		}
	}
	/*	Cancel bind	*/
	else {
		MAPHRCNV_DBGMSG(("Cancel bind [%08lX] \n", gApiInfo[phr_id].slave));
		for(i = 0; i < MAX_PHRASE_CHANNEL; i++) {
			if(gApiInfo[phr_id].slave & (UINT32)(0x01 << i)) {
				if(gApiInfo[i].status != ID_STATUS_READY)	return (MASMW_ERROR);
			}
		}
		for(i = 0; i < MAX_PHRASE_CHANNEL; i++) {
			if(gApiInfo[phr_id].slave & (UINT32)(0x01 << i)) {
				gApiInfo[i].master_channel = -1;
			}
		}

		gApiInfo[phr_id].slave = 0L;
	}

	return (MASMW_SUCCESS);
}

/***** Phrase Wrapper - static ***********************************************/

/*==============================================================================
//	void	InitApiInfo(int ch)
//
//	description	:	Initialize APIINFO
//
//	argument	:	ch ...	channel(0..3)
//
//	return		:
//
//============================================================================*/
static void	InitApiInfo(int ch)
{
	PAPIINFO	pAi;

	if((ch < 0) || (MAX_PHRASE_DATA <= ch)) return;

	pAi = &(gApiInfo[ch]);
	pAi->status			= ID_STATUS_NODATA;
	pAi->file_id		= -1L;
	pAi->vol			= 100;
	pAi->pan			= 64;
	pAi->loop			= 0;
	pAi->master_channel	= -1;
	pAi->slave			= 0L;
	pAi->length			= 0L;
	pAi->pos			= 0L;
	if (ch == 4)
		pAi->gdwRamAddr = 0xFFFFFFFF;
	else {
		if (gPhrRamSize != 0)
			pAi->gdwRamAddr = (UINT32)(gPhrRamAdrs + (0x400 * ch));
		else
			pAi->gdwRamAddr = 0xFFFFFFFF;
	}

}

/*==============================================================================
//	UINT32	SetMasterChannel(int ch, int master_channel)
//
//	description	:	set master channel
//
//	argument	:	ch				...	channel(0..3)
//					master_channel	... channel(0..3)
//
//	return		:
//============================================================================*/
static UINT32	SetMasterChannel(int ch, int master_channel)
{
	if(gdwSlave & (0x01 << ch))					return(1L);
	if(gApiInfo[ch].slave != 0L)				return(1L);

	gApiInfo[ch].master_channel = master_channel;
	gdwSlave |= (UINT32)(1L << ch);

	return (0L);
}

/*==============================================================================
//	SINT32 CallBack(int ch, UINT8 id)
//
//	description	:
//
//	argument	:	ch				...	channel(0..3)
//					id				...	event
//
//	return		:
//					0 meanes no error.
//============================================================================*/
static SINT32	CallBack(int ch, UINT8 id)
{
	struct event	eve;

	/*---	check channel ----------------------------------------------------*/
	if((ch < 0) || (MAX_PHRASE_CHANNEL <= ch))	return (SINT32)(-1L);

	eve.ch		= ch;
	eve.mode	= 0x80;

	switch(id)
	{
	case 0x7F :
		gApiInfo[ch].status = ID_STATUS_READY;
		eve.mode = -0x01;
		break;
	case 0x7E :
		eve.mode = -0x02;
		break;
	case 0x00 :
	case 0x01 :
	case 0x02 :
	case 0x03 :
	case 0x04 :
	case 0x05 :
	case 0x06 :
	case 0x07 :
	case 0x08 :
	case 0x09 :
	case 0x0A :
	case 0x0B :
	case 0x0C :
	case 0x0D :
	case 0x0E :
	case 0x0F :
		eve.mode = (int)id;
		break;
	}

	if((eve.mode != 0x80) && (gEvHandler != NULL)) {
		gEvHandler(&eve);
	}

	return (0L);
}

/*==============================================================================
//	SINT32 CallBack_Service1(UINT8 id)
//
//	description	:
//
//	argument	:	id				... event
//
//	return		:
//					0 meanes no error.
//============================================================================*/
static SINT32 CallBack_Service1(UINT8 id)
{
	return (CallBack(0, id));
}

/*==============================================================================
//	SINT32 CallBack_Service2(UINT8 id)
//
//	description	:
//
//	argument	:	id				... event
//
//	return		:
//					0 meanes no error.
//============================================================================*/
static SINT32 CallBack_Service2(UINT8 id)
{
	return (CallBack(1, id));
}

/*==============================================================================
//	SINT32 CallBack_Service3(UINT8 id)
//
//	description	:
//
//	argument	:	id				... event
//
//	return		:
//					0 meanes no error.
//============================================================================*/
static SINT32 CallBack_Service3(UINT8 id)
{
	return (CallBack(2, id));
}

/*==============================================================================
//	SINT32 CallBack_Service4(UINT8 id)
//
//	description	:
//
//	argument	:	id				... event
//
//	return		:
//					0 meanes no error.
//============================================================================*/
static SINT32 CallBack_Service4(UINT8 id)
{
	return (CallBack(3, id));
}


/***** Common Function *******************************************************/

/*==============================================================================
//	int	ResourceCreate(void)
//
//	description	:	call MaSndDrv_Create().
//
//	argument	:
//
//	return		:
//					0 meanes no error.
//============================================================================*/
static SINT32 ResourceCreate(void)
{
	UINT32	Ram;
	UINT32	Size;

	gdwPhrId = MaSndDrv_Create(	(UINT8)1,
	            (UINT8)PHRASE_TIMEBASE, /* 20msec */
	            (UINT8)0x15,  /* Durum setting ... MA-1/2 Compatible mode */
	                          /* DVA setting ... Compulsion mono mode */
	                          /* Melody setting ... Standard mode */
	            (UINT8)16,    /* Resource mode */
	            (UINT8)0,     /* Number of Streams */
	                   MaPhrCnv_Convert,  /* Call back function */
	                   &gPhrRamAdrs,      /* Starting sddress of RAM */
	                   &gPhrRamSize);     /* Size of RAM */

	if (gdwPhrId < 0L) {
		gdwPhrId = -1L;
		gPhrRamAdrs = 0xFFFFFFFF;
		gPhrRamSize = 0;
		return (-1);
	}

	gSeqID = MaSndDrv_Create(
				(UINT8)MASMW_SEQTYPE_AUDIO,		/* AudioMode                     */
				(UINT8)MASMW_AUDIO_BASETIME,	/* Timer Setting                 */
				(UINT8)0,						/* Option : ---                  */
				(UINT8)1,						/* ResMode                       */
				(UINT8)1,						/* Num of Stream                 */
				MaAudCnv_Convert,				/* Callback Fn                   */
				&Ram,							/* Top Address of Reseved RAM    */
				&Size);							/* Size of Reserved RAM          */

	if (gSeqID < 0L) {
		MaSndDrv_Free(gdwPhrId);
		gdwPhrId = -1L;
		gSeqID = -1L;
		gPhrRamAdrs = 0xFFFFFFFF;
		gPhrRamSize = 0;
		return (-1);
	}

	MAPHRCNV_DBGMSG(("  ResourceCreate [%d %d]\n", gdwPhrId, gSeqID));

	return (0);
}

/*==============================================================================
//	int	ResourceFree(void)
//
//	description	:	call MaSndDrv_Free().
//
//	argument	:
//
//	return		:
//					0 meanes no error.
//============================================================================*/
static SINT32 ResourceFree(void)
{

	MaSndDrv_Free(gdwPhrId);
	gdwPhrId = -1L;
	MaSndDrv_Free(gSeqID);
	gSeqID = -1L;

	gPhrRamAdrs = 0xFFFFFFFF;
	gPhrRamSize = 0;

	return (0);
}


/***** Phrase Wrapper - API **************************************************/

/*==============================================================================
//	int	Phrase_Initialize(void)
//
//	description	:	Initialize SMAF/Phrase Sequencer.
//
//	argument	:
//
//	return		:
//					0 meanes no error.
//============================================================================*/
int	Phrase_Initialize(void)
{
	int	ch;
	UINT8 i;

	MAPHRCNV_DBGMSG(("Phrase_Initialize\n"));

	/*---	check SMAF/Phrase stream converter ID ----------------------------*/
	if (gbCreateStatus & MASK_STATUS_PHRASE)	return (-1);

	/*---	MA-3 Sequence Create ---------------------------------------------*/
	if (gbCreateStatus == 0) {
		if (ResourceCreate() < 0L) {
			return (-1);
		}
	}
	gbCreateStatus |= MASK_STATUS_PHRASE;

	/*---	Initialize global values -----------------------------------------*/
	gbPlayStatus = 0x00;
	gbPauseStatus = 0x00;
	gdwCtrlStatus = CTRL_CLEAR_ALL;
	gdwPhrStatus = 0;

	for(ch = 0; ch < MAX_PHRASE_DATA; ch++) {
		InitApiInfo(ch);
	}

	for(i = 0; i < MASMW_NUM_CHANNEL; i++) {
		InitChInfo(i);
	}

	gEvHandler = NULL;
	gdwSlave = 0L;


	/*	set System On & Master Volume	*/
	gdwCtrlStatus |= (CTRL_SYSTEM_ON | CTRL_MASTER_VOLUME);

	MaSndDrv_SetVolume(gdwPhrId, 0x7F);

	return (0);
}

/*==============================================================================
//	int	Phrase_GetInfo(struct info* dat)
//
//	description	:	Get device information.
//
//	argument	:	dat		...	pointer to device information
//
//	return		:
//					0 meanes no error.
//============================================================================*/
int	Phrase_GetInfo(struct info* dat)
{
	MAPHRCNV_DBGMSG(("Phrase_GetInfo[0x%X] \n", dat));

	if(dat == NULL)	return (-1);

	dat->MakerID		= 0x00430000;			/*	YAMAHA	*/
	dat->DeviceID		= 0x06;					/*	MA-3	*/
	dat->VersionID		= 1;
	dat->MaxVoice		= 4;
	dat->MaxChannel		= 4;
	dat->SupportSMAF	= 0x01;					/*	SMAF/Phrase ver1	*/
	dat->Latency		= (long)(20 * 1000);	/*	20msec	*/

	return (0);
}

/*==============================================================================
//	int	Phrase_CheckData(unsigned char* data, unsigned long len)
//
//	description	:	Check SMAF/Phrase data.
//
//	argument	:	data	...	pointer to data.
//					len		...	the size of data(in byte).
//
//	return		:
//					0 meanes no error.
//============================================================================*/
int	Phrase_CheckData(UINT8* data, UINT32 len)
{
	MAPHRCNV_DBGMSG(("Phrase_CheckData [0x%X 0x%X] \n", data, len));

	/*---	check SMAF/Phrase stream converter ID ----------------------------*/
	if(!(gbCreateStatus & MASK_STATUS_PHRASE) || (gdwPhrId == -1L))	return (-1);

	/*---	check arugment ---------------------------------------------------*/
	if((data == NULL) || (len == 0L)) return (-1);

	/*---	check data -------------------------------------------------------*/
	if(MaPhr_Load(MAX_PHRASE_CHANNEL, (UINT8*)data, (UINT32)len, 2, NULL, NULL) < 0L)
		return (-1);

	return (0);
}

/*==============================================================================
//	int	Phrase_SetData(int ch, unsigned char* data, unsigned long len, int check)
//
//	description	:	Check SMAF/Phrase data.
//
//	argument	:	ch		...	channel(0..3)
//					data	...	pointer to data.
//					len		...	the size of data(in byte).
//					check	...	error check enable(1) or disable(0)
//
//	return		:
//					0 meanes no error.
//============================================================================*/
int	Phrase_SetData(int ch, unsigned char* data, unsigned long len, int check)
{
	MAPHRCNV_DBGMSG(("Phrase_SetData [%d 0x%X 0x%X %d] \n", ch, data, len, check));


	/*---	check SMAF/Phrase stream converter ID ----------------------------*/
	if(!(gbCreateStatus & MASK_STATUS_PHRASE) || (gdwPhrId == -1L))	return (-1);

	/*---	check channel ----------------------------------------------------*/
	if((ch < 0) || (MAX_PHRASE_CHANNEL <= ch))	return (-1);

	/*---	check arugment ---------------------------------------------------*/
	if((data == NULL) || (len == 0L)) return (-1);

	/*---	check status (Is status NODATA?) ---------------------------------*/
	if(gApiInfo[ch].status != ID_STATUS_NODATA) return (-1);

	/*---	check data -------------------------------------------------------*/
	if (MaPhr_Load((UINT8)ch, (UINT8*)data, (UINT32)len, (UINT8)((check) ? 1 : 0), gCallBack[ch], NULL) < 0L)
		return (-1);		/*	fail load data	*/

	/*	Standby	*/
	if(MaPhr_Standby((UINT8)ch, NULL) < 0L) {
		return (-1);
	}

	/*---	successful load data ---------------------------------------------*/
	gApiInfo[ch].status = ID_STATUS_READY;
	gApiInfo[ch].length	= (UINT32)MaPhr_GetLength((UINT8)ch);

	/*---	setup volumes ----------------------------------------------------*/
	gdwCtrlStatus |= (UINT32)(CTRL_VOLUME_CHANGE << (ch << 2));


	return (0);
}

/*==============================================================================
//	long	Phrase_GetLength(int ch)
//
//	description	:	get length
//
//	argument	:	ch		...	channel(0..3).
//
//	return		:
//					position in msec.
//============================================================================*/
long	Phrase_GetLength(int ch)
{
	SINT32		sdLen;

	MAPHRCNV_DBGMSG(("Phrase_GetLength [%d] \n", ch));

	/*---	check SMAF/Phrase stream converter ID ----------------------------*/
	if(!(gbCreateStatus & MASK_STATUS_PHRASE) || (gdwPhrId == -1L))	return (-1);

	/*---	check channel ----------------------------------------------------*/
	if((ch < 0) || (MAX_PHRASE_CHANNEL <= ch))	return (-1);

	/*---	check channel status ---------------------------------------------*/
	if(gApiInfo[ch].status == ID_STATUS_NODATA)	return (-1);

	/*	Get Length	*/
	sdLen = MaPhr_GetLength((UINT8)ch);

	return (long)(sdLen);
}

/*==============================================================================
//	long	Phrase_GetPosition(int ch)
//
//	description	:	get position
//
//	argument	:	ch		...	channel(0..3).
//
//	return		:
//					position in msec.
//============================================================================*/
long	Phrase_GetPosition(int ch)
{
	SINT32		pos;
	IDSTATUS	status;

	MAPHRCNV_DBGMSG(("Phrase_GetPosition [%d] \n", ch));

	/*---	check SMAF/Phrase stream converter ID ----------------------------*/
	if(!(gbCreateStatus & MASK_STATUS_PHRASE) || (gdwPhrId == -1L))	return (-1);

	/*---	check channel ----------------------------------------------------*/
	if((ch < 0) || (MAX_PHRASE_CHANNEL <= ch))	return (-1);

	/*---	check status(Is status PLAY or PAUSE?) ---------------------------*/
	status = gApiInfo[ch].status;
	if((status != ID_STATUS_PLAY) && (status != ID_STATUS_PAUSE))	return (-1);

	/*	Get Position	*/
	pos = MaPhr_GetPos((UINT8)ch);

	return (long)(pos);
}

/*==============================================================================
//	int	Phrase_Seek(int ch, long pos)
//
//	description	:	seek position.
//
//	argument	:	ch		...	channel(0..3).
//					pos		... position
//
//	return		:
//					0 meanes no error.
//============================================================================*/
int	Phrase_Seek(int ch, long pos)
{
	MAPHRCNV_DBGMSG(("Phrase_Seek [%d 0x%X] \n", ch, pos));

	/*---	check SMAF/Phrase stream converter ID ----------------------------*/
	if(!(gbCreateStatus & MASK_STATUS_PHRASE) || (gdwPhrId == -1L))	return (-1);

	/*---	check argument ---------------------------------------------------*/
	if((ch < 0) || (MAX_PHRASE_CHANNEL <= ch))	return (-1);
	if((pos < 0L) || (pos >= (long)(gApiInfo[ch].length)))				return (-1);

	/*---	check status(Is status READY?) -----------------------------------*/
	if(gApiInfo[ch].status != ID_STATUS_READY)	return (-1);

	/*---	is slave ? -------------------------------------------------------*/
	if(gdwSlave & (0x01 << ch))	return (-1);

	/*---	seek ----------------------------------------------------*/
	if(MaPhr_Seek((UINT8)ch, (UINT32)pos, 0, NULL) < 0L)
		return (-1);

	return (0);
}

/*==============================================================================
//	int	Phrase_Play(int ch, int loop)
//
//	description	:	start playback!!
//
//	argument	:	ch		...	channel(0..3).
//					loop	...	0 means infinite loop, else number of loop.
//
//	return		:
//					0 meanes no error.
//============================================================================*/
int	Phrase_Play(int ch, int loop)
{

	MAPHRCNV_DBGMSG(("Phrase_Play [%d %d] \n", ch, loop));


	/*---	check SMAF/Phrase stream converter ID ----------------------------*/
	if(!(gbCreateStatus & MASK_STATUS_PHRASE) || (gdwPhrId == -1L))	return (-1);

	/*---	check channel ----------------------------------------------------*/
	if((ch < 0) || (MAX_PHRASE_CHANNEL <= ch))	return (-1);

	/*---	check status(Is status READY?) -----------------------------------*/
	if(gApiInfo[ch].status != ID_STATUS_READY) return (-1);

	/*---	is slave ? -------------------------------------------------------*/
	if(gdwSlave & (0x01 << ch))	return (-1);

	/*---	start playback ---------------------------------------------------*/
	/*	Start	*/
	if(MaPhr_Start((UINT8)ch, (UINT16)loop) < 0L) return (-1);


	return (0);
}

/*==============================================================================
//	int	Phrase_Stop(int ch)
//
//	description	:	stop playback!!
//
//	argument	:	ch		...	channel(0..3).
//
//	return		:
//					0 meanes no error.
//============================================================================*/
int	Phrase_Stop(int ch)
{
	IDSTATUS	status;

	MAPHRCNV_DBGMSG(("Phrase_Stop [%d] \n", ch));

	/*---	check SMAF/Phrase stream converter ID ----------------------------*/
	if(!(gbCreateStatus & MASK_STATUS_PHRASE) || (gdwPhrId == -1L))	return (-1);

	/*---	check channel ----------------------------------------------------*/
	if((ch < 0) || (MAX_PHRASE_CHANNEL <= ch))	return (-1);

	/*---	check status(Is status READY or PLAY, PAUSE?) --------------------*/
	status = gApiInfo[ch].status;
#if 0
	if (status == ID_STATUS_READY) return (0);
#endif
	if((status != ID_STATUS_PLAY) && (status != ID_STATUS_PAUSE))	return (-1);

	/*---	is slave ? -------------------------------------------------------*/
	if(gdwSlave & (0x01 << ch))	return (-1);

	/*---	stop playback ----------------------------------------------------*/
	/*	Stop	*/
	if(MaPhr_Stop((UINT8)ch) < 0L) return (-1);

	/*---	seek top ----------------------------------------------------*/
	if(MaPhr_Seek((UINT8)ch, 0, 0, NULL) <0L) return (-1);

	return (0);
}

/*==============================================================================
//	int	Phrase_Pause(int ch)
//
//	description	:	pause playback!!
//
//	argument	:	ch		...	channel(0..3).
//
//	return		:
//					0 meanes no error.
//============================================================================*/
int	Phrase_Pause(int ch)
{

	MAPHRCNV_DBGMSG(("Phrase_Pause [%d] \n", ch));

	/*---	check SMAF/Phrase stream converter ID ----------------------------*/
	if(!(gbCreateStatus & MASK_STATUS_PHRASE) || (gdwPhrId == -1L))	return (-1);

	/*---	check channel ----------------------------------------------------*/
	if((ch < 0) || (MAX_PHRASE_CHANNEL <= ch))	return (-1);

	/*---	check status(Is status PLAY?) ------------------------------------*/
	if(gApiInfo[ch].status != ID_STATUS_PLAY) return (-1);

	/*---	is slave ? -------------------------------------------------------*/
	if(gdwSlave & (0x01 << ch))	return (-1);

	/*---	pause playback ---------------------------------------------------*/
	if(MaPhr_Pause((UINT8)ch) < 0L) return (-1);

	return (0);
}

/*==============================================================================
//	int	Phrase_Restart(int ch)
//
//	description	:	restart playback!!
//
//	argument	:	ch		...	channel(0..3).
//
//	return		:
//					0 meanes no error.
//============================================================================*/
int	Phrase_Restart(int ch)
{

	MAPHRCNV_DBGMSG(("Phrase_Restert [%d] \n", ch));

	/*---	check SMAF/Phrase stream converter ID ----------------------------*/
	if(!(gbCreateStatus & MASK_STATUS_PHRASE) || (gdwPhrId == -1L))	return (-1);

	/*---	check channel ----------------------------------------------------*/
	if((ch < 0) || (MAX_PHRASE_CHANNEL <= ch))	return (-1);

	/*---	check status(Is status PAUSE?) ------------------------------------*/
	if(gApiInfo[ch].status != ID_STATUS_PAUSE) return (-1);

	/*---	is slave ? -------------------------------------------------------*/
	if(gdwSlave & (0x01 << ch))	return (-1);

	/*---	restart playback -------------------------------------------------*/
	/*	Pause	*/
	if(MaPhr_Restart((UINT8)ch) < 0L) return (-1);

	return (0);
}

/*==============================================================================
//	int	Phrase_RemoveData(int ch)
//
//	description	:	remove data
//
//	argument	:	ch		...	channel(0..3).
//
//	return		:
//					0 meanes no error.
//============================================================================*/
int	Phrase_RemoveData(int ch)
{
	int		i;
	UINT32	dwSlave;

	MAPHRCNV_DBGMSG(("Phrase_RemoveData[%d] \n", ch));

	/*---	check SMAF/Phrase stream converter ID ----------------------------*/
	if(!(gbCreateStatus & MASK_STATUS_PHRASE) || (gdwPhrId == -1L))	return (-1);

	/*---	check channel ----------------------------------------------------*/
	if((ch < 0) || (MAX_PHRASE_CHANNEL <= ch))	return (-1);

	/*---	check status(Is status READY?) -----------------------------------*/
	if(gApiInfo[ch].status != ID_STATUS_READY) return (-1);

	/*---	remove data ------------------------------------------------------*/
	/*	unload	*/
	/*---	free slave setting. ----------------------------------------------*/
	if(gdwSlave & (0x01 << ch)) {	/*	slave	*/
		i = gApiInfo[ch].master_channel;
		dwSlave = gApiInfo[i].slave;
		dwSlave &= ~(1L << ch);
		gApiInfo[ch].master_channel = -1;
		gdwSlave &= ~((UINT32)(1L << ch));
		gApiInfo[i].slave	= dwSlave;
	} else {			/*	master	*/
		dwSlave = gApiInfo[ch].slave;
		for(i = 0; i < MAX_PHRASE_CHANNEL; i++) {
			if(dwSlave & (1 << i)) {
				gApiInfo[i].master_channel = -1;
				gdwSlave &= ~((UINT32)(1L << i));
			}
		}
		gApiInfo[ch].slave	= 0;
	}

	/*---	successful remove data!! -----------------------------------------*/
	gApiInfo[ch].file_id = -1L;
	gApiInfo[ch].status = ID_STATUS_NODATA;

	return (0);
}

/*==============================================================================
//	int	Phrase_Kill(void)
//
//	description	:	force reset!!
//
//	argument	:
//
//	return		:
//					0 meanes no error.
//============================================================================*/
int	Phrase_Kill(void)
{
	int			ch;

	MAPHRCNV_DBGMSG(("Phrase_Kill \n"));

	/*---	check SMAF/Phrase stream converter ID ----------------------------*/
	if(!(gbCreateStatus & MASK_STATUS_PHRASE) || (gdwPhrId < 0L))	return (-1);

	for(ch = 0; ch < MAX_PHRASE_CHANNEL; ch++) {
		Phrase_Stop(ch);
	}
	for(ch = 0; ch < MAX_PHRASE_CHANNEL; ch++) {
		Phrase_RemoveData(ch);
	}

	return (0);
}

/*==============================================================================
//	int	Phrase_Terminate(void)
//
//	description	:	Terminate SMAF/Phrase Sequencer.
//
//	argument	:
//
//	return		:
//					0 meanes no error.
//============================================================================*/
int	Phrase_Terminate(void)
{
	int	ch;

	MAPHRCNV_DBGMSG(("Phrase_Terminate\n"));

	/*---	check SMAF/Phrase stream converter ID ----------------------------*/
	if(!(gbCreateStatus & MASK_STATUS_PHRASE) || (gdwPhrId == -1L))	return (-1);

		Phrase_Kill();

	gbCreateStatus &= ~MASK_STATUS_PHRASE;
	if (gbCreateStatus == 0)
		ResourceFree();

	/*---	Initialize global values -----------------------------------------*/
	for(ch = 0; ch < MAX_PHRASE_DATA; ch++) {
		InitApiInfo(ch);
	}

	gdwSlave = 0L;

	return (0);
}

/*==============================================================================
//	void	Phrase_SetVolume(int ch, int vol)
//
//	description	:	set volume.
//
//	argument	:	ch	...	channel(0..3)
//					vol	...	volume(-1..127)
//
//	return		:
//============================================================================*/
void	Phrase_SetVolume(int ch, int vol)
{

	MAPHRCNV_DBGMSG(("Phrase_SetVolume [%d %d] \n", ch, vol));

	/*---	check SMAF/Phrase stream converter ID ----------------------------*/
	if(!(gbCreateStatus & MASK_STATUS_PHRASE) || (gdwPhrId == -1L))	return ;

	/*---	check channel ----------------------------------------------------*/
	if((ch < 0) || (MAX_PHRASE_CHANNEL <= ch))	return;

	if(vol == -1)	gApiInfo[ch].vol = 100;	/*	default volume	*/
	else if((0 <= vol) && (vol < 128))	gApiInfo[ch].vol = (UINT8)vol;

	gdwCtrlStatus |= (UINT32)(CTRL_VOLUME_CHANGE << (ch << 2));

	return;
}

/*==============================================================================
//	int	Phrase_GetVolume(int ch)
//
//	description	:	get volume.
//
//	argument	:	ch	...	channel(0..3)
//
//	return		:	volume
//============================================================================*/
int		Phrase_GetVolume(int ch)
{
	MAPHRCNV_DBGMSG(("Phrase_GetVolume [%d] \n", ch));

	/*---	check SMAF/Phrase stream converter ID ----------------------------*/
	if(!(gbCreateStatus & MASK_STATUS_PHRASE) || (gdwPhrId == -1L))	return (-1);

	/*---	check channel ----------------------------------------------------*/
	if((ch < 0) || (MAX_PHRASE_CHANNEL <= ch))	return (-1);

	return (int)(gApiInfo[ch].vol);
}

/*==============================================================================
//	void	Phrase_SetPanpot(int ch, int pan)
//
//	description	:	set panpot.
//
//	argument	:	ch	...	channel(0..3)
//					pan	...	panpot(-1..127)
//
//	return		:
//============================================================================*/
void	Phrase_SetPanpot(int ch, int pan)
{
	UINT8	i;
	UINT8	bChOffset;

	MAPHRCNV_DBGMSG(("Phrase_SetPanpot [%d %d] \n", ch, pan));

	/*---	check SMAF/Phrase stream converter ID ----------------------------*/
	if(!(gbCreateStatus & MASK_STATUS_PHRASE) || (gdwPhrId == -1L))	return ;

	/*---	check channel ----------------------------------------------------*/
	if((ch < 0) || (MAX_PHRASE_CHANNEL <= ch))	return;

	if(pan == -1)	gApiInfo[ch].pan = 64;	/*	default panpot	*/
	else if((0 <= pan) && (pan < 128))	gApiInfo[ch].pan = (UINT8)pan;

	bChOffset = (UINT8)(ch << 2);
	for (i = 0; i < MAX_PHRASE_SLOT; i++) {
		gChInfo[bChOffset + i].bPan = gApiInfo[ch].pan;
	}

	gdwCtrlStatus |= (UINT32)(CTRL_PANPOT_CHANGE << (ch << 2));

	return;
}

/*==============================================================================
//	int	Phrase_GetPanpot(int ch)
//
//	description	:	get panpot.
//
//	argument	:	ch	...	channel(0..3)
//
//	return		:	panpot
//============================================================================*/
int		Phrase_GetPanpot(int ch)
{
	MAPHRCNV_DBGMSG(("Phrase_GetPanpot [%d] \n", ch));

	/*---	check SMAF/Phrase stream converter ID ----------------------------*/
	if(!(gbCreateStatus & MASK_STATUS_PHRASE) || (gdwPhrId == -1L))	return (-1);

	/*---	check channel ----------------------------------------------------*/
	if((ch < 0) || (MAX_PHRASE_CHANNEL <= ch))	return (-1);

	return (int)(gApiInfo[ch].pan);
}

/*==============================================================================
//	int	Phrase_GetStatus(int ch)
//
//	description	:	get status.
//
//	argument	:	ch		...	channel(0..3)
//
//	return		:	status
//============================================================================*/
int		Phrase_GetStatus(int ch)
{
	int	status = 0;

	MAPHRCNV_DBGMSG(("Phrase_GetStatus [%d] \n", ch));

	/*---	check SMAF/Phrase stream converter ID ----------------------------*/
	if(!(gbCreateStatus & MASK_STATUS_PHRASE) || (gdwPhrId == -1L))	return (-1);

	/*---	check channel ----------------------------------------------------*/
	if((ch < 0) || (MAX_PHRASE_CHANNEL <= ch))	return (-1);

	/*---	is slave ? -------------------------------------------------------*/
	if(gdwSlave & (0x01 << ch)) {
		status |= BIT_STATUS_SLAVE;
	}

	switch(gApiInfo[ch].status)
	{
	case ID_STATUS_NODATA :
		status |= 1;
		break;
	case ID_STATUS_READY :
		status |= 2;
		break;
	case ID_STATUS_PLAY :
		status |= 3;
		break;
	case ID_STATUS_ENDING :
		status |= 4;
		break;
	case ID_STATUS_PAUSE :
		status |= 5;
		break;
	default:
		break;
	}
	return (status);
}

/*==============================================================================
//	int	Phrase_SetEvHandler(void (* func(struct event* eve))
//
//	description	:	set event handler.
//
//	argument	:	func	...	pointer the struct event.
//
//	return		:	0 means no error
//============================================================================*/
int		Phrase_SetEvHandler(void (* func)(struct event* eve))
{
	/*---	check SMAF/Phrase stream converter ID ----------------------------*/
	if(!(gbCreateStatus & MASK_STATUS_PHRASE) || (gdwPhrId == -1L))	return (-1);

	/*--- check parameter ----------------------------------------------------*/
	if(func == NULL)	return (-1);

	/*--- regist function ----------------------------------------------------*/
	gEvHandler = func;

	return (int)(0);
}

/*==============================================================================
//	int	Phrase_SetLink(int ch, unsigned long slave)
//
//	description	:	set link.
//
//	argument	:	ch		...	channel(0..3)
//					slave	...	slave(bit0:Ch1 .. bit31:Ch32)
//
//	return		:	error code
//============================================================================*/
int		Phrase_SetLink(int ch, unsigned long slave)
{
	UINT8	i;
	UINT32	dwSlave		= 0L;
	UINT32	dwLength	= 0L;

	/*---	check SMAF/Phrase stream converter ID ----------------------------*/
	if(!(gbCreateStatus & MASK_STATUS_PHRASE) || (gdwPhrId == -1L))	return (-1);

	/*---	check channel ----------------------------------------------------*/
	if((ch < 0) || (MAX_PHRASE_CHANNEL <= ch))	return (-1);

	/*---	is slave ? -----------------------------------------*/
	if(gdwSlave & (0x01 << ch))	return (-1);

	/*---	check status(Is status READY?) -----------------------------------*/
	if(gApiInfo[ch].status != ID_STATUS_READY) return (-1);

	slave		&= ~(1L << ch);
	dwLength	= gApiInfo[ch].length;
	for(i = 0; i < MAX_PHRASE_CHANNEL; i++) {
		if(slave & (1L << i)) {
			if(gApiInfo[i].slave != 0)			return (-1);	/*	already master	*/
			if((gApiInfo[i].master_channel != -1)
				 && (gApiInfo[i].master_channel != ch))
				return (-1);	/* already slave of other channel */
			if(gApiInfo[i].status != ID_STATUS_READY)	return (-1);	/*	not ready	*/
			if(dwLength != gApiInfo[i].length)	return (-1);	/*	not match length	*/		}
	}

	/*---	free slave -------------------------------------------------------*/
	dwSlave = gApiInfo[ch].slave;
	for(i = 0; i < MAX_PHRASE_CHANNEL; i++) {
		if(dwSlave & (1L << i)) {
			gApiInfo[i].master_channel = -1;
			gdwSlave &= ~((UINT32)(1L << i));
		}
	}
	gApiInfo[ch].slave = 0;
	MaPhr_SetBind((UINT8)ch, (UINT8)ch);

	/*---	set slave --------------------------------------------------------*/
	for(i = 0; i < MAX_PHRASE_CHANNEL; i++) {
		if(slave & (1L << i)) {
			SetMasterChannel(i, ch);
			MaPhr_SetBind((UINT8)ch, i);
		}
	}
	gApiInfo[ch].slave	= slave;

	return (0);
}

/*==============================================================================
//	unsigned long	Phrase_GetLink(int ch)
//
//	description	:	get link.
//
//	argument	:	ch		...	channel(0..3)
//
//	return		:	link info
//============================================================================*/
unsigned long		Phrase_GetLink(int ch)
{
	/*---	check SMAF/Phrase stream converter ID ----------------------------*/
	if(!(gbCreateStatus & MASK_STATUS_PHRASE) || (gdwPhrId == -1L))	return (0L);

	/*---	check channel ----------------------------------------------------*/
	if((ch < 0) || (MAX_PHRASE_CHANNEL <= ch))	return (0L);

	return (gApiInfo[ch].slave);
}



/*============================================================================
	  SMAF/Audio
=============================================================================*/

/*=============================================================================
//	Function Name	:	SINT32	NextChunk(UINT8 state, UINT8* pBuff,
//                                        UINT32 remain, UINT16* chunk)
//
//	Description		:	Get Chunk Name and Size
//
//	Argument		:	state		: check status
//						pBuff		: pointer to the data
//						remain		: size of the data
//						chunk		: chunk id
//
//	Return			: >= 0 : NoError(Chunk Size),
//					  <  0 : Error
=============================================================================*/
static SINT32	NextChunk(UINT8  state, UINT8  *pBuff, UINT32 remain,
					UINT16 *chunk)
{
	UINT32 dwChunkID;
	UINT32 dwChunkSize;

	if(remain <= 8)
		return -1;

	dwChunkID = ((UINT32)(*pBuff     << 24) | (UINT32)(*(pBuff+1) << 16) |
				 (UINT32)(*(pBuff+2) << 8 ) | (UINT32)(*(pBuff+3)));
	remain -= 4;
	switch(dwChunkID){
		case 0x4D4D4D44:						/* File Chunk				  */
			if(state != 0)						/* status Check				  */
				return -1;
			*chunk = 0x0001;
			break;
		case 0x434E5449:						/* Contents Info Chunk		  */
			if(state != 1)						/* status Check				  */
				return -1;
			*chunk = 0x0002;
			break;
		case 0x4F504441:						/* Optional Data Chunk		  */
			if(state != 2)						/* status Check				  */
				return -1;
			*chunk = 0x0003;
			break;
		case 0x4D535452:						/* Master Track Chunk		  */
			if(state != 2)						/* status Check				  */
				return -1;
			*chunk = 0x0007;
			break;
		case 0x4D737049:						/* Seek&Phrase Info Chunk(MTR)*/
			if(state != 3)						/* status Check				  */
				return -1;
			*chunk = 0x0009;
			break;
		case 0x4D747375:						/* Setup Data Chunk(MTR)	  */
			if(state != 3)						/* status Check				  */
				return -1;
			*chunk = 0x000A;
			break;
		case 0x4D747371:						/* Sequense Data Chunk(MTR)	  */
			if(state != 3)						/* status Check				  */
				return -1;
			*chunk = 0x000B;
			break;
		case 0x4D747370:						/* Stream PCM Wave Data Chunk */
			if(state != 3)						/* status Check				  */
				return -1;
			*chunk = 0x000C;
			break;
		case 0x41737049:						/* Seek&Phrase Info Chunk(ATR)*/
			if(state != 6)						/* status Check				  */
				return -1;
			*chunk = 0x000E;
			break;
		case 0x41747375:						/* Setup Data Chunk(ATR)	  */
			if(state != 6)						/* status Check				  */
				return -1;
			*chunk = 0x000F;
			break;
		case 0x41747371:						/* Sequense Data Chunk(ATR)	  */
			if(state != 6)						/* status Check				  */
				return -1;
			*chunk = 0x0010;
			break;
		default:
			*chunk = (UINT16)((dwChunkID & 0x000000FF) << 8);
			switch(dwChunkID & 0xFFFFFF00){
				case 0x4D545200:				/* Score Track Chunk		  */
					if(state != 2)				/* status Check				  */
						return -1;
					*chunk |= 0x0004;
					break;
				case 0x41545200:				/* Audio Track Chunk		  */
					if(state != 2)				/* status Check				  */
						return -1;
					*chunk |= 0x0005;
					break;
				case 0x47545200:				/* Graphics Data Chunk		  */
					if(state != 2)				/* status Check				  */
						return -1;
					*chunk |= 0x0006;
					break;
				case 0x44636800:				/* Data Chunk(OPDA)			  */
					if(state != 5)				/* status Check				  */
						return -1;
					*chunk |= 0x0008;
					break;
				case 0x4D776100:				/* Wave Data Chunk(MTR)		  */
					if(state != 4)				/* status Check				  */
						return -1;
					*chunk |= 0x000D;
					break;
				case 0x41776100:				/* Wave Data Chunk(ATR)		  */
					if(state != 6)				/* status Check				  */
						return -1;
					*chunk |= 0x0011;
					break;
				default:						/* Unknown Chunk			  */
					*chunk = 0xFFFF;
					break;
			}
	}
	dwChunkSize = ((UINT32)(*(pBuff+4) << 24) | (UINT32)(*(pBuff+5) << 16) |
				   (UINT32)(*(pBuff+6) << 8 ) | (UINT32)(*(pBuff+7)));
	remain -= 4;
	if(dwChunkSize > remain)					/* Check Chunk Size			  */
		return -1;
	return (SINT32)dwChunkSize;
}


/*=============================================================================
//	Function Name	:	SINT32	GetDataInfo(UINT8* fp, UINT32 fsize, UINT8 mode)
//
//	Description		:	Get data Info from the file
//
//	Argument		:	fp			: pointer to the data
//						fsize		: size fo the data
//						mode		: error check (0:No, 1:Yes, 2:CheckOnly, 3:OPDA)
//
//	Return			: 0 : NoError, < 0 : Error
=============================================================================*/
static SINT32	GetDataInfo(UINT8* fp, UINT32 fsize, UINT8 mode)
{
	SINT32			sdResult;
	PMAAUDCNV_INFO	pI;

	MAAUDCNV_DBGMSG(("GetDataInfo \n"));

	if (mode < 2)			pI = &gInfo[1];
	else					pI = &gInfo[0];

	sdResult	= malib_smafaudio_checker(fp, fsize, mode, pI);

	if (mode < 2) {
		gFs				= pI->dFs;
		gFormat			= (UINT8)pI->dFormat;
		gpSource		= pI->pSource;
		gSizeOfSource	= pI->dSizeOfSource;
	}

	if (sdResult < 0)		return sdResult;
	else					return 0;
}


/*=============================================================================
//	Function Name	:	SINT32	MaAudCnv_Initialize(void)
//
//	Description		:	Initialize()
//
//	Argument		:	none
//
//	Return			: 0 : NoError, < 0 : Error
=============================================================================*/
SINT32	MaAudCnv_Initialize(void)
{
	MAAUDCNV_DBGMSG(("MaAudCnv_Initialize \n"));

	gbAudCtrlStatus = 0;
	gbAudStatus = 0;

#if 0
	gSeqID = -1;
#endif
	gFileID = -1;
	gNumOfLoaded = 0;
	gbAudEnding = 0;
	gSizeOfSource = 0;
	gFs = 8000;
	gInfo[0].dwSize = 0;
	gInfo[0].pConti = 0;
	gInfo[0].dFs = 0;
	gInfo[1].dwSize = 0;
	gInfo[1].pConti = 0;
	gInfo[1].dFs = 0;
	return (MA3AUDERR_NOERROR);
}


/*=============================================================================
//	Function Name	:	SINT32	MaAudCnv_End(void)
//
//	Description		:	Ending()
//
//	Argument		:	none
//
//	Return			: 0 : NoError, < 0 : Error
=============================================================================*/
SINT32	MaAudCnv_End(void)
{
	MAAUDCNV_DBGMSG(("MaAudCnv_End \n"));

	return (MA3AUDERR_NOERROR);
}


/*=============================================================================
//	Function Name	:	SINT32	MaAudCnv_Load(UINT8* file_ptr, UINT32 file_size, UINT8 mode,
//												SINT32 (*func)(UINT8 id), void* ext_args)
//
//	Description		:	Load()
//
//	Argument		:	file_ptr	: pointer to the data
//						file_size	: size fo the data
//						mode		: error check (0:No, 1:Yes, 2:Check)
//						func		: pointer of rhe callback function
//						ext_args	: NULL
//
//	Return			: >= 0 : File ID, < 0 : Error
=============================================================================*/
SINT32	MaAudCnv_Load(UINT8* file_ptr, UINT32 file_size, UINT8 mode, SINT32 (*func)(UINT8 id), void* ext_args)
{
	SINT32	dwRet;

	(void)func;
	(void)ext_args;

	MAAUDCNV_DBGMSG(("MaAudCnv_Load[%08lX %08lX %02X] \n", file_ptr,
		file_size, mode));
	/*--- check parameter ---*/
	if (file_ptr == NULL)					return (MA3AUDERR_GENERAL);
	if (file_size == 0)						return (MA3AUDERR_GENERAL);
	if ((gNumOfLoaded > 0) && (mode < 2))	return (MA3AUDERR_GENERAL);

	dwRet = GetDataInfo(file_ptr, file_size, mode);

	MAAUDCNV_DBGMSG((" Return Value 'GetDataInfo'[%d] \n", dwRet));

	if ((dwRet < 0) || (mode > 1)) return (dwRet);

	gNumOfLoaded++;
	gFileID = gNumOfLoaded;

	return ((UINT32)(gNumOfLoaded));
}


/*=============================================================================
//	Function Name	:	SINT32	MaAudCnv_Unload(SINT32 file_id, void* ext_args)
//
//	Description		:	Unload()
//
//	Argument		:	file_id		: File ID
//						ext_args	: NULL
//
//	Return			: 0 : NoError, < 0 : Error
=============================================================================*/
SINT32	MaAudCnv_Unload(SINT32 file_id, void* ext_args)
{
	(void)ext_args;

	MAAUDCNV_DBGMSG(("MaAudCnv_Unload[%d] \n", file_id));

	if ((file_id < 0) || (file_id != gFileID))
	{
		return (MA3AUDERR_GENERAL);
	}

	gInfo[file_id].dwSize = 0;
	gInfo[file_id].pConti = 0;
	gInfo[file_id].dFs = 0;
	gNumOfLoaded = 0;
	gSizeOfSource = 0;
	gFileID = -1;

	return (MA3AUDERR_NOERROR);
}


/*=============================================================================
//	Function Name	:	MaAudCnv_Open(SINT32 file_id, UINT16 mode, void* ext_args)
//
//	Description		:	Open()
//
//	Argument		:	file_id		: File ID
//						mode		: ResType (0..1)
//						ext_args	: NULL
//
//	Return			: 0 : NoError, < 0 : Error
=============================================================================*/
SINT32	MaAudCnv_Open(SINT32 file_id, UINT16 mode, void* ext_args)
{
	(void)ext_args;
	(void)mode;

	MAAUDCNV_DBGMSG(("MaAudCnv_Open[%d] \n", file_id));

	if ((file_id < 0) || (file_id != gFileID))
	{
		return (MA3AUDERR_GENERAL);
	}

	if (gbCreateStatus == 0) {
		if (ResourceCreate() < 0L) {
			return (MA3AUDERR_GENERAL);
		}
	}
	gbCreateStatus |= MASK_STATUS_AUDIO;

	gbAudCtrlStatus = 0;
	gbAudStatus = 0;
	gPanpot = 64;
	gMasterVol = 100;

	gbAudCtrlStatus |= CTRL_AUDIO_MASTERVOLUME_CHANGE;
	MaSndDrv_SetVolume(gSeqID, 0x7F);

	return (MA3AUDERR_NOERROR);
}


/*=============================================================================
//	Function Name	:	SINT32	MaAudCnv_Close(SINT32 file_id, void* ext_args)
//
//	Description		:	Close()
//
//	Argument		:	file_id		: File ID
//						ext_args	: NULL
//
//	Return			: 0 : NoError, < 0 : Error
=============================================================================*/
SINT32	MaAudCnv_Close(SINT32 file_id, void* ext_args)
{
	(void)ext_args;

	MAAUDCNV_DBGMSG(("MaAudCnv_Close[%d] \n", file_id));

	if ((file_id < 0) || (file_id != gFileID))
	{
		return (MA3AUDERR_GENERAL);
	}

	gbCreateStatus &= ~MASK_STATUS_AUDIO;
	if (gbCreateStatus == 0)
		ResourceFree();

	return (MA3AUDERR_NOERROR);
}


/*=============================================================================
//	Function Name	:	SINT32	MaAudCnv_Standby(SINT32 file_id, void* ext_args)
//
//	Description		:	Standby()
//
//	Argument		:	file_id		: File ID
//						ext_args	: NULL
//
//	Return			: 0 : NoError, < 0 : Error
=============================================================================*/
SINT32	MaAudCnv_Standby(SINT32 file_id, void* ext_args)
{
	(void)ext_args;

	MAAUDCNV_DBGMSG(("MaAudCnv_Standby[%08lX]\n", file_id));

	if ((file_id < 0) || (file_id != gFileID) || (gSizeOfSource == 0))
	{
		return (MA3AUDERR_GENERAL);
	}

	return (MaSndDrv_SetStream(gSeqID, 0, gFormat, gFs, gpSource, gSizeOfSource));
}


/*=============================================================================
//	Function Name	: SINT32	MaAudCnv_Seek(SINT32 file_id, UINT32 pos, UINT8 flag, void* ext_args)
//
//	Description		: Seek() , NOt Supported.
//
//	Argument		: file_id		: File ID
//					  pos			: Start position(msec)
//					  flag			: Wait 0:No wait, 1:20[ms] wait
//					  ext_args		: NULL
//
//	Return			: 0 : NoError, < 0 : Error
=============================================================================*/
SINT32	MaAudCnv_Seek(SINT32 file_id, UINT32 pos, UINT8 flag, void* ext_args)
{
	(void)flag;
	(void)ext_args;

	MAAUDCNV_DBGMSG(("MaAudCnv_Seek[%08lX] = %ld \n", file_id, pos));

	if (gbAudEnding != 0) return (MA3AUDERR_NOERROR);

	if ((file_id < 0) || (file_id != gFileID))
	{
		return (MA3AUDERR_GENERAL);
	}

	/*--- No seek on MA-3 ------------------------*/
	if (pos != 0) return (MA3AUDERR_GENERAL);

	MaSndDrv_ControlSequencer(gSeqID, 4);

	return (MA3AUDERR_NOERROR);
}


/*=============================================================================
//	Function Name	:	SINT32	MaAudCnv_Start(SINT32 file_id, void* ext_args)
//
//	Description		:	Start()
//
//	Argument		:	file_id		: File ID
//						ext_args	: NULL
//
//	Return			: 0 : NoError, < 0 : Error
=============================================================================*/
SINT32	MaAudCnv_Start(SINT32 file_id, void* ext_args)
{
	(void)ext_args;

	MAAUDCNV_DBGMSG(("MaAudCnv_Start[%d] \n", file_id));

	if (gbAudEnding != 0) return (MA3AUDERR_NOERROR);

	if ((file_id < 0) || (file_id != gFileID))
	{
		return (MA3AUDERR_GENERAL);
	}
	
	if (gbAudStatus & MASK_AUDIO_STATUS_STOP) {
		gbAudStatus &= ~MASK_AUDIO_STATUS_STOP;
		gbAudStatus |= MASK_AUDIO_STATUS_REPLAY;
	}
	else if (gbAudStatus & MASK_AUDIO_STATUS_REPLAY) {
	}
	else {
		gbAudStatus |= MASK_AUDIO_STATUS_START;

		/* control sequencer */
		gbSeqStatus |= MASK_STATUS_AUDIO;
		MaSndDrv_ControlSequencer(gSeqID, 1);
	}

	return (MA3AUDERR_NOERROR);
}


/*=============================================================================
//	Function Name	:	SINT32	MaAudCnv_Stop(SINT32 file_id, void* ext_args)
//
//	Description		:	Stop()
//
//	Argument		:	file_id		: File ID
//						ext_args	: Control flag  
//										NULL   call from normal API
//										!=NULL call from interrupt process
//
//	Return			: 0 : NoError, < 0 : Error
=============================================================================*/
SINT32	MaAudCnv_Stop(SINT32 file_id, void* ext_args)
{
#if MA_STOPWAIT
	SINT32 lTimeOut;
#else
	(void)ext_args;
#endif

	MAAUDCNV_DBGMSG(("MaAudCnv_Stop[%d] \n", file_id));

	if (gbAudEnding != 0) return (MA3AUDERR_NOERROR);

	if ((file_id < 0) || (file_id != gFileID))
	{
		return (MA3AUDERR_GENERAL);
	}

	gbAudEnding = 1;

#if MA_STOPWAIT
	if (gbAudStatus & MASK_AUDIO_STATUS_REPLAY) {
		gbAudStatus &= ~MASK_AUDIO_STATUS_REPLAY;
		gbAudStatus |= MASK_AUDIO_STATUS_STOP;
	}
	else if (gbAudStatus & MASK_AUDIO_STATUS_START) {
		gbAudStatus &= ~MASK_AUDIO_STATUS_START;
		gbAudStatus |= MASK_AUDIO_STATUS_STOP;
	}
	else {
		gbAudStatus |= MASK_AUDIO_STATUS_STOP;
	}

	if (ext_args == NULL) {
		lTimeOut = AUDIO_WAIT_TIMEOUT;
		while ((gbAudStatus & MASK_AUDIO_STATUS_STOP) && lTimeOut > 0) {
			machdep_Sleep(5);
			lTimeOut -= 5;
		}
	}
	else {
		MaSndDrv_SetCommand(gSeqID, -1, MASNDDRV_CMD_STREAM_OFF, 0, 0, 0);

		gbAudStatus = 0;
		if (gbSeqStatus & MASK_STATUS_AUDIO) {
			/* sequencer stop */
			gbSeqStatus &= ~MASK_STATUS_AUDIO;
			MaSndDrv_ControlSequencer(gSeqID, 0);
		}
	}

#else
	gbAudStatus = 0;
	if (gbSeqStatus & MASK_STATUS_AUDIO) {
		MaSndDrv_SetCommand(gSeqID, -1, MASNDDRV_CMD_STREAM_OFF, 0, 0, 0);
		
		/* sequencer stop */
		gbSeqStatus &= ~MASK_STATUS_AUDIO;
		MaSndDrv_ControlSequencer(gSeqID, 0);
	}
#endif

	gbAudEnding = 0;

	return (MA3AUDERR_NOERROR);
}


/*=============================================================================
//	Function Name	: SINT32	MaAudCnv_GetContentsInfo_3(SINT32 file_id, PMASMW_CONTENTSINFO pinf)
//
//	Description		: Get position of the data
//
//	Argument		: file_id 		: File ID
//					  pinf :		: pointer to the info
//
//	Return			: >= 0 : Length , < 0 : Error
=============================================================================*/
static SINT32	MaAudCnv_GetContentsInfo_3(SINT32 file_id, PMASMW_CONTENTSINFO pinf)
{
	UINT32	i;
	UINT32	dwTagSize;
	UINT8	tag1, tag2;
	UINT8	*pOpda, *pDch, *pTag;
	UINT32	dwOpda, dwDch, dwTag;
	SINT32	slNextSize;
	UINT16	NextID;

    pOpda  = gInfo[file_id].pInfo;
    dwOpda = gInfo[file_id].dwSize;
    pDch   = 0;
    dwDch  = 0;
    pTag   = 0;
    dwTag  = 0;

    while(dwOpda > 8){                          /* OPDA                       */
        slNextSize = NextChunk(5, pOpda, dwOpda, &NextID);
        if (slNextSize < 0)
            return MASMW_ERROR;
        pOpda += 8;
        dwOpda -= (slNextSize + 8);
        if ( ((NextID & 0x00FF) != 8) ||
            (((NextID >> 8) & 0x00FF) != pinf->code) ) {
            pOpda += slNextSize;
            continue;
        }
        else {
            pDch  = pOpda;
            dwDch = (UINT32)slNextSize;
            pOpda += slNextSize;
        }
    }

    while(dwDch >= 4) {                         /* Dch* (Code type Much)      */
        tag1 = *pDch; pDch++;                   /* TAG 1st Byte               */
        tag2 = *pDch; pDch++;                   /* TAG 2nd Byte               */
        dwTagSize = (UINT32)((*pDch << 8) + *(pDch + 1));
        pDch += 2;
        if ((UINT32)(dwTagSize + 4) > dwDch)
            return MASMW_ERROR;                 /* TAG Size Error             */
        dwDch = dwDch - (dwTagSize + 4);
        if ((tag1 == pinf->tag[0]) && (tag2 == pinf->tag[1])) {
            pTag  = pDch;
            dwTag = dwTagSize;
        }
        pDch += dwTagSize;                      /* Next Information           */
    }

    if (pTag == 0)
        return MA3AUDERR_UNMATCHED_TAG;
    if (dwTag > pinf->size)                     /* Inforamtion Size Check     */
        dwTag = pinf->size;
    for(i = 0; i < dwTag; i++)
        *(pinf->buf + i) = *(pTag + i);
    return (SINT32)dwTag;                       /* Return Information Size    */
}


/*=============================================================================
//	Function Name	: SINT32	MaAudCnv_GetContentsInfo_2(SINT32 file_id, PMASMW_CONTENTSINFO pinf)
//
//	Description		: Get position of the data
//
//	Argument		: file_id 		: File ID
//					  pinf :		: pointer to the info
//
//	Return			: >= 0 : Length , < 0 : Error
=============================================================================*/
static SINT32	MaAudCnv_GetContentsInfo_2(SINT32 file_id, PMASMW_CONTENTSINFO pinf)
{
    UINT8  *pBuf;
    UINT32 dwSize;
    SINT32 slTemp;
    UINT32 i;
    UINT8  flag1, flag2, bEscape;
    UINT8  bData;
    UINT8  bCode;

    pBuf   = gInfo[file_id].pInfo;
    dwSize = gInfo[file_id].dwSize;
    slTemp = 0;
    i      = 0;

    if ((dwSize < 4) || ((pinf->code != CONTENTS_CODE_BINARY) &&
        (gInfo[file_id].bCodeType != pinf->code)))
        return MA3AUDERR_UNMATCHED_TAG;

	bCode = gInfo[file_id].bCodeType;

    while ((dwSize - i) >= 4) {
        if ((pBuf[i] == pinf->tag[0]) && (pBuf[i+1] == pinf->tag[1])) {
            if (((pBuf[i] == 'M') && (pBuf[i+1] == '2')) ||
                ((pBuf[i] == 'L') && (pBuf[i+1] == '2'))) {
                pinf->buf[0] = pBuf[i+3];       /* M2 or L2 TAG               */
                return 1;                       /* return 1 Byte              */
            }
            flag1 = 1;                          /* TAG Match !                */
        }
        else
            flag1 = 0;                          /* UN Match                   */
        flag2   = 0;
        bEscape = 0;
        i += 3;
        while ((flag2 == 0) && ((dwSize - i) >= 1)) {
            bData = pBuf[i];                    /* 1 Byte Get                 */
            i ++;                               /* Size Adjust                */
            if (bEscape == 0) {
                if (bData == ',') {
                    flag2 = 1;                  /* Data End                   */
                    if (flag1 == 1)
                        return slTemp;
                    continue;
                }
                if (bData == '\\') {
                    if ((pBuf[i] == '\\') || (pBuf[i] == ','))
	                    bEscape = 1;            /* Escape Character           */
                    continue;
                }
                if (bCode == CONTENTS_CODE_SJIS) {
                    if (((0x81 <= bData) && (bData <= 0x9F)) ||
                        ((0xE0 <= bData) && (bData <= 0xFC)))
                        bEscape = 2;            /* 2 Byte Character           */
                }
                if (bCode == CONTENTS_CODE_UTF8) {
                    if ((0xC0 <= bData) && (bData <= 0xDF))
                        bEscape = 2;            /* 2 Byte Character           */
                    if ((0xE0 <= bData) && (bData <= 0xEF))
                        bEscape = 3;            /* 3 Byte Character           */
                }
            }
            if (flag1 == 1) {                   /* TAG Match -> Data Copy     */
                pinf->buf[slTemp] = bData;
                slTemp++;
                if (slTemp ==(SINT32)pinf->size)/* Buffer FULL ?              */
                    return slTemp;              /* Return Buffer Size         */
            }
            if (bEscape != 0)
                bEscape --;
        }
    }
    return MA3AUDERR_UNMATCHED_TAG;
}


/*=============================================================================
//	Function Name	: SINT32	MaAudCnv_GetContentsInfo(SINT32 file_id, PMASMW_CONTENTSINFO pinf)
//
//	Description		: Get position of the data
//
//	Argument		: file_id 		: File ID
//					  pinf :		: pointer to the info
//
//	Return			: >= 0 : Length , < 0 : Error
=============================================================================*/
static SINT32	MaAudCnv_GetContentsInfo(SINT32 file_id, PMASMW_CONTENTSINFO pinf)
{
	UINT32 dwret;

	MAAUDCNV_DBGMSG(("MaAudCnv_GetContentsInfo[%08lX] \n", file_id));

	if ((file_id < 0) || (1 < file_id))			/* File ID Error			  */
		return (MA3AUDERR_GENERAL);
    if ((pinf->size == 0) || (gInfo[file_id].pConti == NULL))
        return MASMW_ERROR;

    if ((pinf->tag[0] == 'C') &&
        (0x30 <= pinf->tag[1]) && (pinf->tag[1] <= 0x33)){
        switch(pinf->tag[1]) {
            case 0x30 :                         /* Contents Class             */
                *(pinf->buf) = gInfo[file_id].pConti[0];
                break;
            case 0x31 :                         /* Contents Type              */
                *(pinf->buf) = gInfo[file_id].pConti[1];
                break;
            case 0x32 :                         /* Copy Status                */
                *(pinf->buf) = gInfo[file_id].pConti[3];
                break;
            case 0x33 :                         /* Copy Counts                */
                *(pinf->buf) = gInfo[file_id].pConti[4];
                break;
        }
        return 1;                               /* Data Size(1Byte)           */
    }

	switch (gInfo[file_id].bOptionType){
		case 0:									/* Get from CNTI              */
			dwret = MaAudCnv_GetContentsInfo_2(file_id, pinf);
            return dwret;
		case 1:									/* Get from OPDA              */
			dwret = MaAudCnv_GetContentsInfo_3(file_id, pinf);
            return dwret;
		default :
			return MA3AUDERR_GENERAL;
	}
}


/*=============================================================================
//	Function Name	:	SINT32	MaAudCnv_GetLength(SINT32 file_id)
//
//	Description		:	Get length of the data
//
//	Argument		:	file_id 		: File ID
//
//	Return			: >= 0 : Length , < 0 : Error
=============================================================================*/
static SINT32	MaAudCnv_GetLength(SINT32 file_id)
{
	SINT32	dwLen;

	MAAUDCNV_DBGMSG(("MaAudCnv_GetLength[%08lX] \n", file_id));

	if ((file_id < 0) || (file_id > 1))
	{
		return (MA3AUDERR_GENERAL);
	}

	if (gInfo[file_id].dFs == 0) return (MA3AUDERR_GENERAL);

    switch (gInfo[file_id].dFormat)
    {
    case 0x01: /* 4bit ADPCM */
		dwLen = (gInfo[file_id].dSizeOfSource * 2000 + (gInfo[file_id].dFs - 1)) / gInfo[file_id].dFs;
    	break;
    case 0x02:
    case 0x03:
    	dwLen = (gInfo[file_id].dSizeOfSource * 1000 + (gInfo[file_id].dFs - 1)) / gInfo[file_id].dFs;
    	break;
    default:
    	return (MA3AUDERR_GENERAL);
    }

	return (dwLen);
}


/*=============================================================================
//	Function Name	:	SINT32	MaAudCnv_GetPos(SINT32 file_id)
//
//	Description		:	Get position of the data
//
//	Argument		:	file_id 		: File ID
//
//	Return			: >= 0 : Length , < 0 : Error
=============================================================================*/
static SINT32	MaAudCnv_GetPos(SINT32 file_id)
{
	SINT32 dwLen;
	SINT32 dwPos;

	MAAUDCNV_DBGMSG(("MaAudCnv_GetPos[%08lX] \n", file_id));

	if ((file_id < 0) || (file_id != gFileID))
	{
		return (MA3AUDERR_GENERAL);
	}

	/* get length of the data */
	dwLen = MaAudCnv_GetLength(file_id);
	if (dwLen < 0) return (MA3AUDERR_GENERAL);

	/* get play position of the data */
	dwPos = MaSndDrv_GetPos(gSeqID);

	/* check position */
	if (dwPos > dwLen)
		dwPos = dwLen;

	return (dwPos);
}


/*=============================================================================
//	Function Name	:	SINT32	MaAudCnv_SetVolume(SINT32 file_id, UINT8 bVol)
//
//	Description		:	Set Master volume
//
//	Argument		:	file_id 		: File ID
//						bVol			: Volume
//
//	Return			: 0 : NoError, < 0 : Error
=============================================================================*/
static SINT32	MaAudCnv_SetVolume(SINT32 file_id, UINT8 bVol)
{
	(void)file_id;

	MAAUDCNV_DBGMSG(("MaAudCnv_SetVolume[%08lX, %d] \n", file_id, bVol));

	gMasterVol = (UINT8)(bVol & 0x7f);
	gbAudCtrlStatus |= CTRL_AUDIO_MASTERVOLUME_CHANGE;

	return (MA3AUDERR_NOERROR);
}


/*=============================================================================
//	Function Name	:	SINT32	MaAudCnv_SetPanpot(SINT32 file_id, UINT8 bPan)
//
//	Description		:	Set Master panpot
//
//	Argument		:	file_id 		: File ID
//						bPan			: Panpot
//
//	Return			: 0 : NoError, < 0 : Error
=============================================================================*/
static SINT32	MaAudCnv_SetPanpot(SINT32 file_id, UINT8 bPan)
{
	(void)file_id;

	MAAUDCNV_DBGMSG(("MaAudCnv_SetPanpot[%08lX, %d] \n", file_id, bPan));

	gPanpot = (UINT8)(bPan & 0x7f);

	return (MA3AUDERR_NOERROR);
}


/*=============================================================================
//	Function Name	:	SINT32	MaAudCnv_Control(SINT32 file_id, UINT8 ctrl_num, void* prm, void* ext_args)
//
//	Description		:	Control
//
//	Argument		:	file_id 		: File ID
//						ctrl_num		: Control command ID
//						prm				: Parameters
//						ext_args		: NULL
//
//	Return			: 0 : NoError, < 0 : Error
=============================================================================*/
SINT32	MaAudCnv_Control(SINT32 file_id, UINT8 ctrl_num, void* prm, void* ext_args)
{
	(void)ext_args;
	
	MAAUDCNV_DBGMSG(("MaAudCnv_Control[%d, %d] \n", file_id, ctrl_num));

	if ((file_id < 0) || (file_id > 1))
	{
		MAAUDCNV_DBGMSG(("MaAudCnv_Control/ FilID Error\n"));
		return (MA3AUDERR_ARGUMENT);
	}

	switch(ctrl_num)
	{
	case MASMW_SET_VOLUME :
		if (file_id != gFileID)
		{
			MAAUDCNV_DBGMSG(("MaAudCnv_Control/ FilID Error\n"));
			return (MA3AUDERR_ARGUMENT);
		}
		return (MaAudCnv_SetVolume(file_id, *((UINT8*)prm)));

	case MASMW_GET_LENGTH :
		if (file_id == 0) return (MaAudCnv_GetLength(file_id));
		if (file_id != gFileID)
		{
			MAAUDCNV_DBGMSG(("MaAudCnv_Control/ FilID Error\n"));
			return (MA3AUDERR_ARGUMENT);
		}
		return (MaAudCnv_GetLength(file_id));

	case MASMW_GET_POSITION :
		if (file_id != gFileID)
		{
			MAAUDCNV_DBGMSG(("MaAudCnv_Control/ FilID Error\n"));
			return (MA3AUDERR_ARGUMENT);
		}
		return (MaAudCnv_GetPos(file_id));

	case MASMW_GET_CONTENTSDATA :
		return (MaAudCnv_GetContentsInfo(file_id, (PMASMW_CONTENTSINFO)prm));

	case MASMW_SET_PANPOT :
		if (file_id != gFileID)
		{
			MAAUDCNV_DBGMSG(("MaAudCnv_Control/ FilID Error\n"));
			return (MA3AUDERR_ARGUMENT);
		}
		return (MaAudCnv_SetPanpot(file_id, *((UINT8*)prm)));
	}

	return (MA3AUDERR_NOERROR);
}


/*=============================================================================
//	Function Name	: SINT32	MaAud_Convert(void)
//
//	Description		: No effect.
//
//	Argument		: none
//
//	Return			: 0 : NoMoreData
=============================================================================*/
SINT32	MaAud_Convert(void)
{
	UINT8	bAudStatusB;
	SINT32 lCnvData;

	bAudStatusB = gbAudStatus;
	gbAudStatus = 0;

	MAAUDCNV_DBGMSG((" MaAudCnv_Convert[%02X]\n", bAudStatusB));

	lCnvData = 0L;

	if (gbAudCtrlStatus & CTRL_AUDIO_MASTERVOLUME_CHANGE) {
		MaSndDrv_SetCommand(gSeqID, -1, MASNDDRV_CMD_MASTER_VOLUME, (UINT32)gMasterVol, 0, 0);
		gbAudCtrlStatus &= ~CTRL_AUDIO_MASTERVOLUME_CHANGE;
		lCnvData++;
	}

	if (bAudStatusB & MASK_AUDIO_STATUS_START) {
		MaSndDrv_SetCommand(gSeqID, -1, MASNDDRV_CMD_STREAM_PANPOT, 0, gPanpot, 0);
		MaSndDrv_SetCommand(gSeqID, -1, MASNDDRV_CMD_STREAM_ON, 0, 0, 127);
		lCnvData += 2;
	}

	if (bAudStatusB & MASK_AUDIO_STATUS_STOP) {
#if MA_STOPWAIT
		MaSndDrv_SetCommand(gSeqID, -1, MASNDDRV_CMD_STREAM_OFF, 0, 0, 0);
		lCnvData++;
		
		/* sequencer stop */
		gbSeqStatus &= ~MASK_STATUS_AUDIO;
		MaSndDrv_ControlSequencer(gSeqID, 0);
#endif
	}
	else if (bAudStatusB & MASK_AUDIO_STATUS_REPLAY) {
		MaSndDrv_SetCommand(gSeqID, -1, MASNDDRV_CMD_STREAM_OFF, 0, 0, 0);
		MaSndDrv_SetCommand(gSeqID, -1, MASNDDRV_CMD_STREAM_PANPOT, 0, gPanpot, 0);
		MaSndDrv_SetCommand(gSeqID, -1, MASNDDRV_CMD_STREAM_ON, 0, 0, 127);
		lCnvData += 3;
	}

	return (lCnvData);
}


/*=============================================================================
//	Function Name	: SINT32	MaAudCnv_Convert(void)
//
//	Description		: No effect.
//
//	Argument		: none
//
//	Return			: 0 : NoMoreData
=============================================================================*/
SINT32	MaAudCnv_Convert(void)
{

	return (0);
}


/***** Common I/F *************************************************************/

/*==============================================================================
//	SINT32	PhrAud_Initialize(void)
//
//	description	:	Initialize SMAF/Phrase, SMAF/Audio Sequencer.
//	                called by masndseq.c
//
//	argument	:
//
//	return		:
//					0 meanes no error.
//============================================================================*/
SINT32	PhrAudCnv_Initialize(void)
{
	MAPHRCNV_DBGMSG((" PhrAudCnv_Initialize\n"));

	/*--- Initialize global variables ----------------------------------------*/
	gbCreateStatus = 0;
	gbSeqStatus = 0;

	gdwPhrId = -1L;
	gSeqID = -1L;

	gPhrRamAdrs = 0xFFFFFFFF;
	gPhrRamSize = 0;

	return (0);
}


/*=============================================================================
//	Function Name	:	SINT32	MaPhrCnv_Convert(void)
//
//	Description		:	Convert SMAF/Phraes and SMAF/Audio to Native format
//
//	Argument		:	N/A
//
//	Return			:	Number of convertion events (0:NoEvent)
//
=============================================================================*/
SINT32	MaPhrCnv_Convert(void)
{
	SINT32	lCnvData;

	MAPHRCNV_DBGMSG((" MaPhrCnv_Convert[%02X]\n", gbSeqStatus));

	lCnvData = 0L;

	if (gbSeqStatus & MASK_STATUS_PHRASE) {
		lCnvData += MaPhr_Convert();
	}

	if (gbSeqStatus & MASK_STATUS_AUDIO) {
		lCnvData += MaAud_Convert();
	}

	if (gbSeqStatus) {
		/*	When no converted data, send NOP as dummy data	*/
		/*	Return 0 makes sequencer to not call Convert	*/
		if (lCnvData == 0L) {
			MaSndDrv_SetCommand(gdwPhrId, -1L, MASNDDRV_CMD_NOP, 0, 0, 0);
			lCnvData++;
		}
	}

	return (lCnvData);
}




