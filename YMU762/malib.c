/*==============================================================================
//	Copyright(c) 2002 YAMAHA CORPORATION
//
//	Title		: MAMIL.C
//
//	Description	: MA-3 SMAF/Phrase/Audio Error Check Module.
//
//	Version		: 0.8.0.3		2002.12.12
//
//============================================================================*/
#include "malib.h"
#include "mammfcnv.h"

#define	MALIB_PHRASE_DATA_NOERROR		0x0000
#define	MALIB_PHRASE_DATA_SIZE			0x0001
#define	MALIB_PHRASE_DATA_CLASS			0x0002
#define	MALIB_PHRASE_DATA_TYPE			0x0004
#define	MALIB_PHRASE_DATA_VERSION		0x0008
#define	MALIB_PHRASE_DATA_TIMEBASE		0x0010
#define	MALIB_PHRASE_DATA_CHUNK			0x0020
#define	MALIB_PHRASE_DATA_CRC			0x0040
#define	MALIB_PHRASE_DATA_SHORTLEN		0x0080

#define	MALIB_MA3AUDERR_ARGUMENT		MASMW_ERROR_ARGUMENT
#define	MALIB_MA3AUDERR_CHUNK_SIZE		MASMW_ERROR_CHUNK_SIZE
#define	MALIB_MA3AUDERR_CHUNK			MASMW_ERROR_CHUNK
#define	MALIB_MA3AUDERR_C_CLASS			MASMW_ERROR_CONTENTS_CLASS
#define	MALIB_MA3AUDERR_C_TYPE			MASMW_ERROR_CONTENTS_TYPE
#define	MALIB_MA3AUDERR_FILE			MASMW_ERROR_FILE
#define	MALIB_MA3AUDERR_SHORT_LENGTH	MASMW_ERROR_SHORT_LENGTH

#define	MALIB_MAX_PHRASE_VOICES			4
#define	MALIB_MIN_PHRASE_DATA_LENGTH	1

#define	MALIB_SIZE_OF_CHUNKHEADER		8
#define	MALIB_SIZE_OF_CRC				2
#define	MALIB_SIZE_OF_MIN_CNTI			5

#define	MALIB_CNTI_TYPE_PHRASE			0xF0

#define	MALIB_PHRASE_VERSION			1
#define	MALIB_PHRASE_TIMEBASE			20

#define	MALIB_VOICE_TYPE_UNKNOWN		0
#define	MALIB_VOICE_TYPE_MA2			1
#define	MALIB_VOICE_TYPE_MA3			2

#define	MALIB_CHKMODE_ENABLE			1
#define	MALIB_CHKMODE_CNTIONLY			2

#define	MALIB_MAX_STREAM_FS_4BIT		24000
#define	MALIB_MAX_STREAM_FS_8BIT		12000


/*	CRC converting table	*/
static const UINT16 gwCRCTbl[256] = {
  0x0000U,0x1021U,0x2042U,0x3063U,0x4084U,0x50A5U,0x60C6U,0x70E7U,
  0x8108U,0x9129U,0xA14AU,0xB16BU,0xC18CU,0xD1ADU,0xE1CEU,0xF1EFU,
  0x1231U,0x0210U,0x3273U,0x2252U,0x52B5U,0x4294U,0x72F7U,0x62D6U,
  0x9339U,0x8318U,0xB37BU,0xA35AU,0xD3BDU,0xC39CU,0xF3FFU,0xE3DEU,
  0x2462U,0x3443U,0x0420U,0x1401U,0x64E6U,0x74C7U,0x44A4U,0x5485U,
  0xA56AU,0xB54BU,0x8528U,0x9509U,0xE5EEU,0xF5CFU,0xC5ACU,0xD58DU,
  0x3653U,0x2672U,0x1611U,0x0630U,0x76D7U,0x66F6U,0x5695U,0x46B4U,
  0xB75BU,0xA77AU,0x9719U,0x8738U,0xF7DFU,0xE7FEU,0xD79DU,0xC7BCU,
  0x48C4U,0x58E5U,0x6886U,0x78A7U,0x0840U,0x1861U,0x2802U,0x3823U,
  0xC9CCU,0xD9EDU,0xE98EU,0xF9AFU,0x8948U,0x9969U,0xA90AU,0xB92BU,
  0x5AF5U,0x4AD4U,0x7AB7U,0x6A96U,0x1A71U,0x0A50U,0x3A33U,0x2A12U,
  0xDBFDU,0xCBDCU,0xFBBFU,0xEB9EU,0x9B79U,0x8B58U,0xBB3BU,0xAB1AU,
  0x6CA6U,0x7C87U,0x4CE4U,0x5CC5U,0x2C22U,0x3C03U,0x0C60U,0x1C41U,
  0xEDAEU,0xFD8FU,0xCDECU,0xDDCDU,0xAD2AU,0xBD0BU,0x8D68U,0x9D49U,
  0x7E97U,0x6EB6U,0x5ED5U,0x4EF4U,0x3E13U,0x2E32U,0x1E51U,0x0E70U,
  0xFF9FU,0xEFBEU,0xDFDDU,0xCFFCU,0xBF1BU,0xAF3AU,0x9F59U,0x8F78U,
  0x9188U,0x81A9U,0xB1CAU,0xA1EBU,0xD10CU,0xC12DU,0xF14EU,0xE16FU,
  0x1080U,0x00A1U,0x30C2U,0x20E3U,0x5004U,0x4025U,0x7046U,0x6067U,
  0x83B9U,0x9398U,0xA3FBU,0xB3DAU,0xC33DU,0xD31CU,0xE37FU,0xF35EU,
  0x02B1U,0x1290U,0x22F3U,0x32D2U,0x4235U,0x5214U,0x6277U,0x7256U,
  0xB5EAU,0xA5CBU,0x95A8U,0x8589U,0xF56EU,0xE54FU,0xD52CU,0xC50DU,
  0x34E2U,0x24C3U,0x14A0U,0x0481U,0x7466U,0x6447U,0x5424U,0x4405U,
  0xA7DBU,0xB7FAU,0x8799U,0x97B8U,0xE75FU,0xF77EU,0xC71DU,0xD73CU,
  0x26D3U,0x36F2U,0x0691U,0x16B0U,0x6657U,0x7676U,0x4615U,0x5634U,
  0xD94CU,0xC96DU,0xF90EU,0xE92FU,0x99C8U,0x89E9U,0xB98AU,0xA9ABU,
  0x5844U,0x4865U,0x7806U,0x6827U,0x18C0U,0x08E1U,0x3882U,0x28A3U,
  0xCB7DU,0xDB5CU,0xEB3FU,0xFB1EU,0x8BF9U,0x9BD8U,0xABBBU,0xBB9AU,
  0x4A75U,0x5A54U,0x6A37U,0x7A16U,0x0AF1U,0x1AD0U,0x2AB3U,0x3A92U,
  0xFD2EU,0xED0FU,0xDD6CU,0xCD4DU,0xBDAAU,0xAD8BU,0x9DE8U,0x8DC9U,
  0x7C26U,0x6C07U,0x5C64U,0x4C45U,0x3CA2U,0x2C83U,0x1CE0U,0x0CC1U,
  0xEF1FU,0xFF3EU,0xCF5DU,0xDF7CU,0xAF9BU,0xBFBAU,0x8FD9U,0x9FF8U,
  0x6E17U,0x7E36U,0x4E55U,0x5E74U,0x2E93U,0x3EB2U,0x0ED1U,0x1EF0U
};


typedef struct _tagMalibVoiceInfo
{
	UINT8	bBankNo;
	UINT8	bProgNo;
	UINT8	bType;
	UINT8	bSize;
	UINT8*	pbVoice;
} MALIB_VOCINFO, *MALIB_PVOCINFO;

/*	SMAF/Phrase Information */
typedef struct _tagMalibPhrInfo
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
	MALIB_VOCINFO	VocInfo[MALIB_MAX_PHRASE_VOICES];
} MALIB_PHRINFO, *MALIB_PPHRINFO;

/*	work area for SMAF/Phrase Format Check	*/
typedef struct _tagMalibPhrCheck
{
	UINT8		bMode;				/*	Check Mode								*/
	UINT16		wErrStatus;			/**/
	UINT8*		pbBuffer;			/*	Pointer to Data							*/
	UINT32		dwSize;				/*	The size of Data (in Byte)				*/
	UINT8*		pbPhrase;			/*	Pointer to Phrase Header				*/
	UINT32		dwPhraseSize;		/*	The size of Phrase Data					*/
	UINT8*		pbVoice;			/*	Pointer to Voice Chunk Header			*/
	UINT32		dwVoiceSize;		/*	The size of Voice Chunk Data			*/
	MALIB_PPHRINFO	pPhrInfo;
} MALIB_PHRCHECK, *MALIB_PPHRCHECK;


typedef struct _MALIB_MAAUDCNV_INFO
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
} MALIB_MAAUD_INFO, *MALIB_PMAAUD_INFO;

static MALIB_PHRCHECK		sPhrase_Check;
static MALIB_PHRINFO		sPhrase_Info;

static MALIB_MAAUD_INFO		sAudio_Check;

/*=============================================================================
//	Function Name	:	UINT8	_get1b(UINT8** ppBuffer)
//
//	Description		:	Get 1Byte
//
//	Argument		:	ppBuffer	...	Pointer which contains the address of data
//
//	Return			:	get value
//
=============================================================================*/
static UINT8			_get1b(
	UINT8**				ppBuffer
)
{
	UINT8	b8;

	b8 = **ppBuffer;
	(*ppBuffer)++;

	return b8;
}


/*=============================================================================
//	Function Name	:	UINT32	_get4b(UINT8** ppBuffer)
//
//	Description		:	Get 4Bytes
//
//	Argument		:	ppBuffer	...	Pointer which contains the address of data
//
//	Return			:	get value
//
=============================================================================*/
static UINT32			_get4b(
	UINT8**				ppBuffer
)
{
	UINT32	dw32;

	dw32 =	(UINT32)(((UINT32)*(*ppBuffer + 0) << 24)	|	\
					 ((UINT32)*(*ppBuffer + 1) << 16)	|	\
					 ((UINT32)*(*ppBuffer + 2) << 8)	|	\
					 ((UINT32)*(*ppBuffer + 3) << 0));
	(*ppBuffer) += 4;

	return dw32;
}


/*=============================================================================
//	Function Name	:	void	malib_PhrChk_InitPhraseInfo(PPHRINFO pPhrInfo)
//
//	Description		:	Initialize PHRINFO structure
//
//	Argument		:	pPhrInfo		...	Pointer to PHRINFO structure
//
//	Return			:
//
=============================================================================*/
static void				malib_PhrChk_InitPhraseInfo(
	MALIB_PPHRINFO		pPhrInfo
)
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
	for(i = 0; i < MALIB_MAX_PHRASE_VOICES; i++)
	{
		pPhrInfo->VocInfo[i].bBankNo = 0;
		pPhrInfo->VocInfo[i].bProgNo = 0;
	}
}


/*=============================================================================
//	Function Name	:	void	malib_PhrChk_Initialize(UINT8* pBuffer, UINT32 dwSize)
//
//	Description		:	Initialize PHRCHECK structure
//
//	Argument		:	pBuffer		...	Pointer to data.
//						dwSize		...	The Size of data(in Byte).
//	Return			:	pointer to work space
//
=============================================================================*/
static MALIB_PPHRCHECK	malib_PhrChk_Initialize(
	UINT8*				pBuffer,
	UINT32				dwSize
)
{
	sPhrase_Check.bMode			= MALIB_CHKMODE_ENABLE;
	sPhrase_Check.wErrStatus	= MALIB_PHRASE_DATA_NOERROR;
	sPhrase_Check.pbBuffer		= pBuffer;
	sPhrase_Check.dwSize		= dwSize;
	sPhrase_Check.pbPhrase		= pBuffer;
	sPhrase_Check.dwPhraseSize	= dwSize;
	sPhrase_Check.pbVoice		= pBuffer;
	sPhrase_Check.dwVoiceSize	= dwSize;
	sPhrase_Check.pbPhrase		= NULL;
	sPhrase_Check.dwPhraseSize	= 0L;
	sPhrase_Check.pPhrInfo		= &sPhrase_Info;

	malib_PhrChk_InitPhraseInfo(sPhrase_Check.pPhrInfo);

	return (&sPhrase_Check);
}


/*=============================================================================
//	Function Name	:	UINT32	PhrChk_CompareID(UINT8* pId, UINT8* pBuffer, UINT32 dwMask)
//
//	Description		:	Compare ID
//
//	Argument		:	pId		...	ID
//						pBuffer	...	Pointer to the data
//						dwMask	...	Mask
//
//	Return			:	0	Match ID
//						!0	Not Match ID
//
=============================================================================*/
static UINT32			PhrChk_CompareID(
	UINT8*				pId,
	UINT8*				pBuffer,
	UINT32				dwMask
)
{
	UINT32	dw32;
	UINT32	dwId;

	dwId = (UINT32)(((UINT32)pId[0] << 24)		|	\
					((UINT32)pId[1] << 16)		|	\
					((UINT32)pId[2] << 8)		|	\
					((UINT32)pId[3] << 0));

	dw32 = (UINT32)(((UINT32)pBuffer[0] << 24)		|	\
					((UINT32)pBuffer[1] << 16)		|	\
					((UINT32)pBuffer[2] << 8)		|	\
					((UINT32)pBuffer[3] << 0));

	dwId &= dwMask;
	dw32 &= dwMask;

	if(dwId != dw32) {
		return (1L);
	}

	return (0L);
}


/*=============================================================================
//	Function Name	:	UINT16	makeCRC(UINT32 dwSize, UINT8* pBuffer)
//
//	Description		:	Caluclate CRC
//
//	Argument		:	dwSize		...	Data size (in Byte)
//						pBuffer		...	Pointer to the data
//
//	Return			:	CRC
//
=============================================================================*/
UINT16					makeCRC(
	UINT32				dwSize,
	UINT8*				pBuffer
)
{
	UINT16	res;
	UINT8	data0;

	res		= 0xFFFFU;

	while(--dwSize >= 2L) {
		data0	= *pBuffer++;
		res		= (UINT16)((res << 8) ^ gwCRCTbl[(UINT8)(res >> 8) ^ data0]);
	}

	return (UINT16)(~res & 0xFFFFU);
}


/*=============================================================================
//	Function Name	:	UINT16	malib_MakeCRC(UINT32 dwSize, UINT8* pBuffer)
//
//	Description		:	Caluclate CRC
//
//	Argument		:	dwSize		...	Data size (in Byte)
//						pBuffer		...	Pointer to the data
//
//	Return			:	CRC
//
=============================================================================*/
UINT16					malib_MakeCRC(
	UINT32				dwSize,
	UINT8*				pBuffer
)
{
	return (makeCRC(dwSize, pBuffer));
}


/*=============================================================================
//	Function Name	:	void UINT32	PhrChk_OverallChunkCheck
//
//	Description		:	Check SMAF/Phrase data
//
//	Argument		:	pPhrChk	...	Pointer to the checking result
//
//	Return			:	0 NoError
//						!0 Error
//
=============================================================================*/
static UINT32			PhrChk_OverallChunkCheck(
	MALIB_PPHRCHECK		pPhrChk
)
{
	UINT16	wGetCRC;
	UINT16	wCalCRC;
	UINT32	dwSize;
	UINT8*	pHead;
	static UINT8 bChunkName[5] = "MMMD";

	/*	Check Size							*/
	if(pPhrChk->dwSize < MALIB_SIZE_OF_CHUNKHEADER + MALIB_SIZE_OF_CRC) {
		pPhrChk->wErrStatus |= MALIB_PHRASE_DATA_SIZE;
		return (1L);
	}

	/*	Check Header ID						*/
	pHead = pPhrChk->pbBuffer;
	if(PhrChk_CompareID(bChunkName, pPhrChk->pbBuffer, 0xFFFFFFFF) != 0L) {
		pPhrChk->wErrStatus |= MALIB_PHRASE_DATA_CHUNK;
		return (1L);
	}
	pPhrChk->pbBuffer += 4L;

	/*	Get Chunk Size						*/
	dwSize	= _get4b(&(pPhrChk->pbBuffer));

	/* Compare Chunk Size and SMAF size		*/
	if((UINT32)(dwSize + MALIB_SIZE_OF_CHUNKHEADER) != pPhrChk->dwSize) {
		pPhrChk->wErrStatus |= MALIB_PHRASE_DATA_SIZE;
		return (1L);
	}

	/*	Check CRC			*/
	if(pPhrChk->bMode & MALIB_CHKMODE_ENABLE) {
		wCalCRC	= makeCRC((UINT32)(pPhrChk->dwSize), pHead);
		wGetCRC	= (UINT16)(((UINT16)pHead[pPhrChk->dwSize - 2L] << 8) |	\
						   ((UINT16)pHead[pPhrChk->dwSize - 1L] << 0));
		if(wCalCRC != wGetCRC) {
			pPhrChk->wErrStatus |= MALIB_PHRASE_DATA_CRC;
			return (1L);
		}
	}

	/*	Update				*/
	pPhrChk->dwSize = dwSize;

	return (0L);
}


/*=============================================================================
//	Function Name	:	UINT32	PhrChk_GetCntiInfoChunkBody(PPHRCHECK pPhrChk)
//
//	Description		:	Check SMAF/Phrase Contents info chunk body
//
//	Argument		:	pPhrChk	...	Pointer to the checking result
//
//	Return			:	0 NoError
//						!0 Error
//
=============================================================================*/
static UINT32			PhrChk_GetCntiInfoChunkBody(
	MALIB_PPHRCHECK		pPhrChk
)
{
	UINT32			dwSize;
	MALIB_PPHRINFO	pPi;

	pPi = pPhrChk->pPhrInfo;
	pPi->bCntiClass		= _get1b(&(pPhrChk->pbBuffer));		/*	Contents Class		*/
	pPi->bCntiType		= _get1b(&(pPhrChk->pbBuffer));		/*	Contents Type		*/
	pPi->bCodeType		= _get1b(&(pPhrChk->pbBuffer));		/*	Contents Code Type	*/
	pPi->bCopyStatus	= _get1b(&(pPhrChk->pbBuffer));		/*	Copy Status			*/
	pPi->bCopyCount		= _get1b(&(pPhrChk->pbBuffer));		/*	Copy Count			*/

	/*	Contents Class		*/
	if(pPhrChk->bMode & MALIB_CHKMODE_ENABLE)
	{
		if  (pPi->bCntiClass != MALIB_CNTI_CLASS_YAMAHA) {
			pPhrChk->wErrStatus |= MALIB_PHRASE_DATA_CLASS;
			return (1L);
		}

		/*	Contents Type		*/
		if  (pPi->bCntiType != MALIB_CNTI_TYPE_PHRASE) {
			pPhrChk->wErrStatus |= MALIB_PHRASE_DATA_TYPE;
			return (1L);
		}
	}

	/*	Option				*/
	pPhrChk->pPhrInfo->pbCntiOption = pPhrChk->pbBuffer;

	/*	skip	*/
	dwSize = pPhrChk->pPhrInfo->dwCntiOptionSize;
	pPhrChk->pbBuffer += dwSize;

	/*	update	*/
	pPhrChk->dwSize	 -= (UINT32)(dwSize + MALIB_SIZE_OF_MIN_CNTI + MALIB_SIZE_OF_CHUNKHEADER);
	return	(0L);
}


/*=============================================================================
//	Function Name	:	UINT32	PhrChk_CntiInfoChunkCheck(PPHRCHECK pPhrChk)
//
//	Description		:	Check SMAF/Phrase Contents info chunk
//
//	Argument		:	pPhrChk	...	Pointer to the checking result
//
//	Return			:	0 NoError
//						!0 Error
//
=============================================================================*/
static UINT32			PhrChk_CntiInfoChunkCheck(
	MALIB_PPHRCHECK		pPhrChk
)
{
	UINT32		dwSize;
	static UINT8 bChunkName[5] = "CNTI";

	/*	Check Size							*/
	if(pPhrChk->dwSize < (UINT32)(MALIB_SIZE_OF_CHUNKHEADER + MALIB_SIZE_OF_MIN_CNTI + MALIB_SIZE_OF_CRC)) {
		pPhrChk->wErrStatus |= MALIB_PHRASE_DATA_SIZE;
		return (1L);
	}

	/*	Check Contents Info ID				*/
	if(PhrChk_CompareID(bChunkName, pPhrChk->pbBuffer, 0xFFFFFFFF) != 0L) {
		pPhrChk->wErrStatus |= MALIB_PHRASE_DATA_CHUNK;
		return (1L);
	}
	pPhrChk->pbBuffer += 4L;

	/*	Get Chunk and  Size					*/
	dwSize	= _get4b(&(pPhrChk->pbBuffer));

	/*	Check Size							*/
	if(dwSize < (UINT32)MALIB_SIZE_OF_MIN_CNTI) {
		pPhrChk->wErrStatus |= MALIB_PHRASE_DATA_SIZE;
		return (1L);
	}

	/*	Check Size							*/
	if(pPhrChk->dwSize < (UINT32)(MALIB_SIZE_OF_CHUNKHEADER + dwSize + MALIB_SIZE_OF_CRC)) {
		pPhrChk->wErrStatus |= MALIB_PHRASE_DATA_SIZE;
		return (1L);
	}

	/*	Contents Info Chunk body			*/
	pPhrChk->pPhrInfo->dwCntiOptionSize = (UINT16)(dwSize - MALIB_SIZE_OF_MIN_CNTI);
	if(PhrChk_GetCntiInfoChunkBody(pPhrChk) != 0L) {
		return (1L);
	}

	return (0L);
}


/*=============================================================================
//	Function Name	:	UINT32	PhrChk_OptionDataChunkCheck(PPHRCHECK pPhrChk)
//
//	Description		:	Check SMAF/Phrase Option data chunk
//
//	Argument		:	pPhrChk	...	Pointer to the checking result
//
//	Return			:	0 NoError
//						!0 Error
//
=============================================================================*/
static UINT32			PhrChk_OptionDataChunkCheck(
	MALIB_PPHRCHECK		pPhrChk
)
{
	UINT32		dwSize;
	static UINT8 bChunkName[5] = "OPDA";

	/*	Check Size							*/
	if(pPhrChk->dwSize < (UINT32)(MALIB_SIZE_OF_CHUNKHEADER + MALIB_SIZE_OF_CRC)) {
		pPhrChk->wErrStatus |= MALIB_PHRASE_DATA_SIZE;
		return (1L);
	}

	/*	Check Option Data Chunk				*/
	if(PhrChk_CompareID(bChunkName, pPhrChk->pbBuffer, 0xFFFFFFFF) != 0L) {
		return (0L);
	}

	/*	Get Chunk and Size					*/
	pPhrChk->pbBuffer += 4L;
	dwSize	= _get4b(&(pPhrChk->pbBuffer));

	/*	Check Size							*/
	if(pPhrChk->dwSize < (UINT32)(MALIB_SIZE_OF_CHUNKHEADER + dwSize + MALIB_SIZE_OF_CRC)) {
		pPhrChk->wErrStatus |= MALIB_PHRASE_DATA_SIZE;
		return (1L);
	}

	pPhrChk->pPhrInfo->pbOptionChunk	= pPhrChk->pbBuffer;
	pPhrChk->pPhrInfo->dwOptionDataSize	= dwSize;

	/*	update	*/
	pPhrChk->pbBuffer += dwSize;
	pPhrChk->dwSize	 -= (UINT32)(dwSize + MALIB_SIZE_OF_CHUNKHEADER);
	return	(0L);
}


/*=============================================================================
//	Function Name	:	UINT32	PhrChk_InfoChunkBody(PPHRCHECK pPhrChk)
//
//	Description		:	Check SMAF/Phrase Info chunk body
//
//	Argument		:	pPhrChk	...	Pointer to the checking result
//
//	Return			:	0 NoError
//						!0 Error
//
=============================================================================*/
static UINT32			PhrChk_InfoChunkBody(
	MALIB_PPHRCHECK		pPhrChk
)
{
	UINT32			dwSize;
	MALIB_PPHRINFO	pPi = pPhrChk->pPhrInfo;

	/*	Get Size								*/
	dwSize = _get4b(&(pPhrChk->pbPhrase));

	/*	Check Size								*/
	if(pPhrChk->dwPhraseSize < (UINT32)(MALIB_SIZE_OF_CHUNKHEADER + dwSize))
	{
		pPhrChk->wErrStatus |= MALIB_PHRASE_DATA_SIZE;
		return (1L);
	}

	/*	Remember position and size				*/
	pPi->pbInfoChunk		= pPhrChk->pbPhrase;
	pPi->dwInfoChunkSize	= dwSize;

	/*	Update	*/
	pPhrChk->pbPhrase		+= dwSize;
	pPhrChk->dwPhraseSize	-= (UINT32)(dwSize + MALIB_SIZE_OF_CHUNKHEADER);
	return (0L);
}


/*=============================================================================
//	Function Name	:	UINT32	PhrChk_DefaultVoiceChunkBody(PPHRCHECK pPhrChk)
//
//	Description		:	Check SMAF/Phrase Voice chunk
//
//	Argument		:	dwNumofVoice	...	Registering Voice number
//						pPhrChk			...	Pointer to the checking result
//
//	Return			:	0 NoError
//						!0 Error
//
=============================================================================*/
static UINT32			PhrChk_DefaultVoiceChunkBody(
	UINT32				dwNumofVoice,
	MALIB_PPHRCHECK		pPhrChk
)
{
	UINT8			bProgNo;
	UINT8*			pParam;
	UINT32			dwSize;
	MALIB_PVOCINFO	pVi;

	if(dwNumofVoice >= MALIB_MAX_PHRASE_VOICES)		return(1L);

	pVi = &(pPhrChk->pPhrInfo->VocInfo[dwNumofVoice]);

	/*	Get Size								*/
	dwSize = _get4b(&(pPhrChk->pbVoice));

	/*	Check Size								*/
	if(pPhrChk->dwVoiceSize < (UINT32)(MALIB_SIZE_OF_CHUNKHEADER + dwSize))
	{
		pPhrChk->wErrStatus |= MALIB_PHRASE_DATA_SIZE;
		return (1L);
	}

	pParam = pPhrChk->pbVoice;

	if (dwSize >= 1L) {
		/*	set default voice						*/
		bProgNo = pParam[0];
		pVi->bBankNo = 0;		/*	means default voice	*/
		pVi->bProgNo = (UINT8)(bProgNo & 0x7F);
		pVi->bType	 = MALIB_VOICE_TYPE_MA3;
		pVi->bSize	 = 0;
		pVi->pbVoice = NULL;
	}
	else {
		pVi->bBankNo = 0;
		pVi->bProgNo = 0;
	}

	/*	Update	*/
	pPhrChk->pbVoice += dwSize;
	pPhrChk->dwVoiceSize -= (UINT32)(dwSize + MALIB_SIZE_OF_CHUNKHEADER);
	return (0L);
}


/*=============================================================================
//	Function Name	:	UINT32	PhrChk_ExtendVoiceChunkBody(PPHRCHECK pPhrChk)
//
//	Description		:	Check SMAF/Phrase Extend Voice chunk body
//
//	Argument		:	dwNumofVoice	...	Registering Voice number
//						pPhrChk			...	Pointer to the checking result
//
//	Return			:	0 NoError
//						!0 Error
//
=============================================================================*/
static UINT32			PhrChk_ExtendVoiceChunkBody(
	UINT32				dwNumofVoice,
	MALIB_PPHRCHECK		pPhrChk
)
{
	UINT8			bSize;
	UINT8			bVoiceType;
	UINT8*			pParam;
	UINT32			dwSize;
	MALIB_PVOCINFO	pVi;

	if(dwNumofVoice >= MALIB_MAX_PHRASE_VOICES)		return(1L);

	pVi = &(pPhrChk->pPhrInfo->VocInfo[dwNumofVoice]);

	/*	Get Size								*/
	dwSize = _get4b(&(pPhrChk->pbVoice));

	/*	Check Size								*/
	if(pPhrChk->dwVoiceSize < (UINT32)(MALIB_SIZE_OF_CHUNKHEADER + dwSize))
	{
		pPhrChk->wErrStatus |= MALIB_PHRASE_DATA_SIZE;
		return (1L);
	}

	bVoiceType	= MALIB_VOICE_TYPE_UNKNOWN;
	pParam		= pPhrChk->pbVoice;
	bSize = 0;

	if (dwSize >= 21L) {	/* MA-2 2op Voice(Shortest voice parameter) */
		/*	-------------------------- Get voice ----------------------------	*/
		bSize		= pParam[2];

		/*	Check Size								*/
		if(dwSize < (UINT32)(bSize + 3))
		{
			pPhrChk->wErrStatus |= MALIB_PHRASE_DATA_SIZE;
			return (1L);
		}

		/*	MA-2 FM wave format setting */
		if(	(pParam[0] == 0xFF)		&&
			(pParam[1] == 0xF0)		&&
			(pParam[3] == 0x43)		&&
			(pParam[4] == 0x03))
		{
			if(	((bSize == 18) || (bSize == 28)) && (pParam[bSize + 2] == 0xF7)) {
				bSize		-= 3;
				pParam		+= 5;
				bVoiceType = MALIB_VOICE_TYPE_MA2;
			}
		}
		/*	MA-3 FM wave format setting	*/
		else if((pParam[0] == 0xFF)		&&
				(pParam[1] == 0xF0)		&&
				(pParam[3] == 0x43)		&&
				(pParam[4] == 0x04)		&&
				(pParam[5] == 0x01))
		{
			if(	((bSize == 20) || (bSize == 34)) && (pParam[bSize + 2] == 0xF7)) {
				bSize		-= 4;
				pParam		+= 6;
				bVoiceType	= MALIB_VOICE_TYPE_MA3;
			}
		}
	}

	if(bVoiceType == MALIB_VOICE_TYPE_UNKNOWN) {
		/*	Set ProgNo = 0 as default vioce		*/
		pVi->bBankNo = 0;
		pVi->bProgNo = 0;
	} else {
		/*	extend voice setting				*/
		pVi->bBankNo = (UINT8)(pPhrChk->pPhrInfo->bPhrNum+1); /* means extend voice */
		pVi->bProgNo = (UINT8)(dwNumofVoice);
		pVi->bType	 = bVoiceType;
		pVi->bSize	 = bSize;
		pVi->pbVoice = pParam;
	}

	/*	Update	*/
	pPhrChk->pbVoice	 += dwSize;
	pPhrChk->dwVoiceSize -= (UINT32)(dwSize + MALIB_SIZE_OF_CHUNKHEADER);
	return (0L);
}


/*=============================================================================
//	Function Name	:	UINT32	PhrChk_VoiceChunkCheck(PPHRCHECK pPhrChk)
//
//	Description		:	Check SMAF/Phrase voice chunk
//
//	Argument		:	pPhrChk	...	Pointer to the checking result
//
//	Return			:	0 NoError
//						!0 Error
//
=============================================================================*/
static UINT32			PhrChk_VoiceChunkCheck(
	MALIB_PPHRCHECK		pPhrChk
)
{
	UINT32			i;
	UINT32			dwSize;
	UINT32			dwSkip;
	UINT32			dwNumofVoice;
	MALIB_PPHRINFO	pPi = pPhrChk->pPhrInfo;
	MALIB_PVOCINFO	pVi;
	static UINT8	bChunkName0[5] = "DEVO";
	static UINT8	bChunkName1[5] = "EXVO";

	/*	Get Size								*/
	dwSize = _get4b(&(pPhrChk->pbPhrase));

	/*	Check Size								*/
	if(pPhrChk->dwPhraseSize < (UINT32)(MALIB_SIZE_OF_CHUNKHEADER + dwSize))
	{
		pPhrChk->wErrStatus |= MALIB_PHRASE_DATA_SIZE;
		return (1L);
	}

	/*	Remember position and size				*/
	pPhrChk->pbVoice		= pPhrChk->pbPhrase;
	pPhrChk->dwVoiceSize	= dwSize;
	pPi->pbVoiceChunk		= pPhrChk->pbPhrase;
	pPi->dwVoiceChunkSize	= dwSize;

	/*	Get Voice Parameter						*/
	dwNumofVoice = 0L;
	while(pPhrChk->dwVoiceSize > (UINT32)(MALIB_SIZE_OF_CHUNKHEADER))
	{
		/**/
		if(	(dwNumofVoice < MALIB_MAX_PHRASE_VOICES)	&&
			(PhrChk_CompareID(bChunkName0, pPhrChk->pbVoice, 0xFFFFFFFF) == 0L))
		{
			pPhrChk->pbVoice += 4L;
			if(PhrChk_DefaultVoiceChunkBody(dwNumofVoice, pPhrChk) != 0L) {
				return (1L);
			}
			dwNumofVoice++;
		}
		/**/
		else if((dwNumofVoice < MALIB_MAX_PHRASE_VOICES)	&&
				(PhrChk_CompareID(bChunkName1, pPhrChk->pbVoice, 0xFFFFFFFF) == 0L))
		{
			pPhrChk->pbVoice += 4L;
			if(PhrChk_ExtendVoiceChunkBody(dwNumofVoice, pPhrChk) != 0L) {
				return (1L);
			}
			dwNumofVoice++;
		}
		/*	other chunk!!						*/
		else
		{
			/*	Skip Chunk						*/
			pPhrChk->pbVoice += 4L;
			dwSkip = _get4b(&(pPhrChk->pbVoice));

			/*	Check Size			*/
			if(pPhrChk->dwVoiceSize < (UINT32)(MALIB_SIZE_OF_CHUNKHEADER + dwSkip))
			{
				pPhrChk->wErrStatus |= MALIB_PHRASE_DATA_SIZE;
				return (1L);
			}

			/*	Update	*/
			pPhrChk->pbVoice		+= dwSkip;
			pPhrChk->dwVoiceSize	-= (UINT32)(MALIB_SIZE_OF_CHUNKHEADER + dwSkip);
		}
	}

	for(i = dwNumofVoice; i < MALIB_MAX_PHRASE_VOICES; i++)
	{
		/*	Set ProgNo=0 as default voice to un-set voices	*/
		pVi = &(pPi->VocInfo[i]);
		pVi->bBankNo = 0;
		pVi->bProgNo = 0;
	}

	/*	Update	*/
	pPhrChk->pbPhrase	  += dwSize;
	pPhrChk->dwPhraseSize -= (UINT32)(dwSize + MALIB_SIZE_OF_CHUNKHEADER);
	return (0L);
}


/*=============================================================================
//	Function Name	:	UINT32	PhrChk_SequenceChunkCheck(PPHRCHECK pPhrChk)
//
//	Description		:	Check SMAF/Phrase Sequence chunk
//
//	Argument		:	pPhrChk	...	Pointer to the checking result
//
//	Return			:	0 NoError
//						!0 Error
//
=============================================================================*/
static UINT32			PhrChk_SequenceChunkCheck(
	MALIB_PPHRCHECK		pPhrChk
)
{
	UINT8*			pbSeq;
	UINT8			bData0;
	UINT8			bData1;
	UINT8			bSize;
	UINT32			dwRemain;
	UINT32			dwSize;
	UINT32			dwDuration;
	UINT32			dwPlayTime;
	MALIB_PPHRINFO	pPi = pPhrChk->pPhrInfo;

	/*	Get Size								*/
	dwSize = _get4b(&(pPhrChk->pbPhrase));

	/*	Check Size								*/
	if(pPhrChk->dwPhraseSize < (UINT32)(MALIB_SIZE_OF_CHUNKHEADER + dwSize))
	{
		pPhrChk->wErrStatus |= MALIB_PHRASE_DATA_SIZE;
		return (1L);
	}

	/*	Remember position and size				*/
	pPi->pbSequenceChunk		= pPhrChk->pbPhrase;
	pPi->dwSequenceChunkSize	= dwSize;

	/*	Get play back time						*/
	pbSeq		= pPi->pbSequenceChunk;
	dwRemain	= pPi->dwSequenceChunkSize;
	dwPlayTime	= 0L;
	while(dwRemain)
	{
		/*	Check Size!!		*/
		if(dwRemain < 3L) {		/*	minimam command	*/
			pPhrChk->wErrStatus |= MALIB_PHRASE_DATA_SIZE;
			return (1L);
		}

		/*	Duration 1 or 2byte	*/
		dwDuration = (UINT32)*(pbSeq++);
		dwRemain--;
		if(dwDuration & 0x80) {
			dwDuration = (UINT32)((((dwDuration & 0x7F) << 7) + (UINT32)*(pbSeq++)) + 128);
			dwRemain--;
		}

		/*	Update play back time	*/
		dwPlayTime += dwDuration;

		/* Event */
		bData0 = *(pbSeq++);
		dwRemain--;
		switch (bData0)
		{
		case 0x00:
			/* Control Message */
			if(dwRemain < 1L) {
				pPhrChk->wErrStatus |= MALIB_PHRASE_DATA_SIZE;
				return (1L);
			}
			bData1 = *(pbSeq++);
			dwRemain--;

			if((bData1 & 0x30) == 0x30) {	/*	3byte	*/
				if(dwRemain < 1L) {
					pPhrChk->wErrStatus |= MALIB_PHRASE_DATA_SIZE;
					return (1L);
				}
				pbSeq++;
				dwRemain--;
			}
			else {						/*	2byte	*/
			}
			break;

		case 0xFF:
			/* Exclusive Message or NOP */
			if(dwRemain < 1L) {
				pPhrChk->wErrStatus |= MALIB_PHRASE_DATA_SIZE;
				return (1L);
			}
			bData1 = *(pbSeq++);
			dwRemain--;

			if(bData1 == 0xF0) {
				if(dwRemain < 1L) {
					pPhrChk->wErrStatus |= MALIB_PHRASE_DATA_SIZE;
					return (1L);
				}
				bSize = *(pbSeq++);
				dwRemain--;

				if(dwRemain < (UINT32)bSize) {
					pPhrChk->wErrStatus |= MALIB_PHRASE_DATA_SIZE;
					return (1L);
				}
				else {
					pbSeq += (UINT32)bSize;
					dwRemain -= (UINT32)bSize;
				}
			}
			else {				/*	NOP	*/
			}
			break;

		default:
			/* Note Message */
			/* Gatetime 1 or 2byte */
			dwDuration = (UINT32)*(pbSeq++);
			dwRemain--;
			if(dwDuration & 0x80) {
				if(dwRemain < 1) {
					pPhrChk->wErrStatus |= MALIB_PHRASE_DATA_SIZE;
					return (1L);
				}
				dwDuration = (UINT32)((((dwDuration & 0x7F) << 7) + (UINT32)*(pbSeq++)) + 128);
				dwRemain--;
			}
			break;
		}
	}

	pPi->dwPlayTime = dwPlayTime;
	if(pPi->dwPlayTime <= MALIB_MIN_PHRASE_DATA_LENGTH) {
		pPhrChk->wErrStatus |= MALIB_PHRASE_DATA_SHORTLEN;
		return (1L);
	}

	/*	Update	*/
	pPhrChk->pbPhrase	  += dwSize;
	pPhrChk->dwPhraseSize -= (UINT32)(dwSize + MALIB_SIZE_OF_CHUNKHEADER);
	return (0L);
}


/*=============================================================================
//	Function Name	:	UINT32	PhrChk_PhraseBody(PPHRCHECK pPhrChk)
//
//	Description		:	Check SMAF/Phrase "MMMG" chunk body
//
//	Argument		:	pPhrChk	...	Pointer to the checking result
//
//	Return			:	0 NoError
//						!0 Error
//
=============================================================================*/
static UINT32			PhrChk_PhraseBody(
	MALIB_PPHRCHECK		pPhrChk
)
{
	UINT8			bFound;
	UINT8			bTmp;
	UINT32			dwSize;
	UINT32			dwSkip;
	static UINT8	bChunkName0[5] = "INFO";
	static UINT8	bChunkName1[5] = "VOIC";
	static UINT8	bChunkName2[5] = "SEQU";

	/*	Get Size								*/
	dwSize = _get4b(&(pPhrChk->pbBuffer));

	/*	Check Size								*/
	if(pPhrChk->dwSize < (UINT32)(MALIB_SIZE_OF_CHUNKHEADER + dwSize + MALIB_SIZE_OF_CRC))
	{
		pPhrChk->wErrStatus |= MALIB_PHRASE_DATA_SIZE;
		return (1L);
	}

	pPhrChk->pbPhrase	  = pPhrChk->pbBuffer;
	pPhrChk->dwPhraseSize = dwSize;

	if(pPhrChk->bMode & MALIB_CHKMODE_ENABLE)
	{
		/*	Check Version							*/
		bTmp = _get1b(&(pPhrChk->pbPhrase));
		if(bTmp != MALIB_PHRASE_VERSION) {
			pPhrChk->wErrStatus |= MALIB_PHRASE_DATA_VERSION;
			return (1L);
		}

		/*	Check Timer base						*/
		bTmp = _get1b(&(pPhrChk->pbPhrase));
		if(bTmp != MALIB_PHRASE_TIMEBASE) {
			pPhrChk->wErrStatus |= MALIB_PHRASE_DATA_TIMEBASE;
			return (1L);
		}
	} else {
		pPhrChk->pbPhrase += 2L;	/*	Skip	*/
	}
	pPhrChk->dwPhraseSize -= 2L;

	/*	Search "INFO", "VOIC", "SEQU" Chunk		*/
	bFound = 0x00;
	while(pPhrChk->dwPhraseSize > (UINT32)(MALIB_SIZE_OF_CHUNKHEADER))
	{
		/**/
		if(PhrChk_CompareID(bChunkName0, pPhrChk->pbPhrase, 0xFFFFFFFF) == 0L)
		{
			pPhrChk->pbPhrase += 4L;
			if(PhrChk_InfoChunkBody(pPhrChk) != 0L) {
				return (1L);
			}
		}
		/**/
		else if(PhrChk_CompareID(bChunkName1, pPhrChk->pbPhrase, 0xFFFFFFFF) == 0L)
		{
			pPhrChk->pbPhrase += 4L;
			if(PhrChk_VoiceChunkCheck(pPhrChk) != 0L) {
				return (1L);
			}
		}
		/**/
		else if(PhrChk_CompareID(bChunkName2, pPhrChk->pbPhrase, 0xFFFFFFFF) == 0L)
		{
			bFound = 0x01;
			pPhrChk->pbPhrase += 4L;
			if(PhrChk_SequenceChunkCheck(pPhrChk) != 0L) {
				return (1L);
			}
		}
		/*	other chunk!!						*/
		else
		{
			/*	Skip Chunk						*/
			pPhrChk->pbPhrase += 4L;
			dwSkip = _get4b(&(pPhrChk->pbPhrase));

			/*	Check Size			*/
			if(pPhrChk->dwPhraseSize < (UINT32)(MALIB_SIZE_OF_CHUNKHEADER + dwSkip))
			{
				pPhrChk->wErrStatus |= MALIB_PHRASE_DATA_SIZE;
				return (1L);
			}

			/*	Update	*/
			pPhrChk->pbPhrase		+= dwSkip;
			pPhrChk->dwPhraseSize -= (UINT32)(dwSkip + MALIB_SIZE_OF_CHUNKHEADER);
		}
	}

	if(bFound == 0x00) {
		pPhrChk->wErrStatus |= MALIB_PHRASE_DATA_CHUNK;
		return (1L);
	}

	/*	Update	*/
	pPhrChk->pbBuffer = pPhrChk->pbPhrase;
	pPhrChk->dwSize	  -= (UINT32)(dwSize + MALIB_SIZE_OF_CHUNKHEADER);
	return (0L);
}


/*=============================================================================
//	Function Name	:	SINT32		smafpharse_checker
//
//	Description		:	Check SMAF/Phrase
//
//	Argument		:	psPCA	...	Pointer to the checking result
//
//	Return			:	data length(msec) or error code
//
=============================================================================*/
SINT32					smafpharse_checker(
	MALIB_PPHRCHECK		pPCA
)
{
	UINT8				bFound;
	UINT32				dwSize;
	static UINT8		bChunkName[5] = "MMMG";

	/*	Check Header & CRC				*/
	if(PhrChk_OverallChunkCheck(pPCA) != 0L)		return pPCA->wErrStatus;

	/*	Check Contents Info Chunk		*/
	if(PhrChk_CntiInfoChunkCheck(pPCA) != 0L)		return pPCA->wErrStatus;

	/*	Check Option Data Chunk			*/
	if(PhrChk_OptionDataChunkCheck(pPCA) != 0L)		return pPCA->wErrStatus;

	/*	check only contetns infomation	*/
	if(pPCA->bMode & MALIB_CHKMODE_CNTIONLY)		return pPCA->wErrStatus;

	/*	Phrase Check					*/
	bFound = 0x00;
	while(pPCA->dwSize > (UINT32)(MALIB_SIZE_OF_CHUNKHEADER + MALIB_SIZE_OF_CRC))
	{
		/*	Search "MMMG" Chunk				*/
		if(PhrChk_CompareID(bChunkName, pPCA->pbBuffer, 0xFFFFFFFF) != 0L)
		{
			/*	Skip Chunk						*/
			pPCA->pbBuffer += 4L;
			dwSize = _get4b(&(pPCA->pbBuffer));

			/*	Check Size			*/
			if(pPCA->dwSize < (UINT32)(MALIB_SIZE_OF_CHUNKHEADER + dwSize + MALIB_SIZE_OF_CRC))
				return MALIB_PHRASE_DATA_SIZE;

			/*	Update	*/
			pPCA->pbBuffer	+= dwSize;
			pPCA->dwSize	-= dwSize;
		}
		/*	Found "MMMG" Chunk!!				*/
		else
		{
			bFound = 0x01;
			pPCA->pbBuffer += 4L;
			if(PhrChk_PhraseBody(pPCA) != 0L)		return pPCA->wErrStatus;

			break;		/*	Support only the first "MMMG" Chunk	*/
		}
	}

	/*	Not found "MMMG" Chunk	*/
	if(bFound == 0x00)								return MALIB_PHRASE_DATA_CHUNK;

	return MALIB_PHRASE_DATA_NOERROR;
}


/*=============================================================================
//	Function Name	:	SINT32		malib_smafphrase_checker
//
//	Description		:	Check SMAF/Phrase
//
//	Argument		:	pBuffer	...	Pointer to the data
//						dwSize	...	Data size
//						psPCA	...	Pointer to the checking result
//
//	Return			:	data length(msec) or error code
//
=============================================================================*/
SINT32					malib_smafphrase_checker(
	UINT8*				pbBuf,
	UINT32				dwSize,
	void*				pvPCA
)
{
	SINT32				sdResult;
	MALIB_PPHRCHECK		pPCA;

	if (pvPCA == NULL)		pPCA	= malib_PhrChk_Initialize(pbBuf, dwSize);
	else					pPCA	= (MALIB_PPHRCHECK)pvPCA;

	sdResult	= smafpharse_checker(pPCA);

	if (sdResult != MALIB_PHRASE_DATA_NOERROR) {
		if(sdResult & MALIB_PHRASE_DATA_SIZE)		return (MASMW_ERROR_CHUNK_SIZE);
		if(sdResult & MALIB_PHRASE_DATA_CLASS)		return (MASMW_ERROR_CONTENTS_CLASS);
		if(sdResult & MALIB_PHRASE_DATA_TYPE)		return (MASMW_ERROR_CONTENTS_TYPE);
		if(sdResult & MALIB_PHRASE_DATA_CHUNK)		return (MASMW_ERROR_CHUNK);
		if(sdResult & MALIB_PHRASE_DATA_SHORTLEN)	return (MASMW_ERROR_SHORT_LENGTH);
													return (MASMW_ERROR_FILE);
	}
	else	return (SINT32)(pPCA->pPhrInfo->dwPlayTime * MALIB_PHRASE_TIMEBASE);
}


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
//	Function Name	:	UINT8	GetWaveInfo_3(UINT8* fp, UINT32 size, UINT8 mode)
//
//	Description		:	Get data Info from the file
//
//	Argument		:	fp			: pointer to the data
//						size		: size fo the data
//						pvI			: pointer to work space
//
//	Return			: 0 : NoError, > 0 : Error
=============================================================================*/
static UINT8			GetWaveInfo_3(
	UINT8*				fp,
	UINT32				size,
	MALIB_PMAAUD_INFO	pI
)
{
	SINT32  dwSize;
	UINT16  wNextChunk;
	UINT32  dwIndex;
	UINT32	dwFs;
	UINT8	bFmt;
	UINT8*	pbWave;

	dwIndex = 0;
	bFmt	= 0xFF;
	dwFs	= 0;
	pbWave	= NULL;
	dwSize	= 0;
	while(size > 8)
	{
		dwSize = NextChunk(3, &fp[dwIndex], size, &wNextChunk);
		if(dwSize < 0)									return 1;

		if(wNextChunk != 0x000C)
		{
			dwIndex += 8 + dwSize;
			size -= 8 + dwSize;
			continue;
		}
		dwIndex += 8;
		size -= 8;

		while (size > 8)
		{
			dwSize = NextChunk(4, &fp[dwIndex], size, &wNextChunk);
			if(dwSize < 4)								return 1;

			if ((wNextChunk & 0x00FF) != 0x000D)
			{
				dwIndex += 8 + dwSize;
				size -= 8 + dwSize;
				continue;
			}

			dwSize -= 3;
			dwFs = ((UINT32)fp[dwIndex + 9] << 8) + (UINT32)fp[dwIndex + 10];
			if (dwFs < 4000)							return 1;

			switch (fp[dwIndex + 8])
			{
				case 0x01:
					if (dwFs > MALIB_MAX_STREAM_FS_8BIT)		return 1;
					if ((UINT32)dwSize <= (dwFs / 50))	return 2;
					bFmt = 3;
					pbWave	= &fp[dwIndex + 11];
					break;

				case 0x11:
					if (dwFs > MALIB_MAX_STREAM_FS_8BIT)		return 1;
					if ((UINT32)dwSize <= (dwFs / 50))	return 2;
					bFmt = 2;
					pbWave	= &fp[dwIndex + 11];
					break;

				case 0x20:
					if (dwFs > MALIB_MAX_STREAM_FS_4BIT)		return 1;
					if ((UINT32)dwSize <= (dwFs / 100))	return 2;
					bFmt = 1;
					pbWave	= &fp[dwIndex + 11];
					break;

				default:
					return 1;
			}
			break;
		}
		break;
	}

	if (bFmt == 0xFF)									return 2;

	pI->dFs				= dwFs;
	pI->dFormat			= (UINT32)bFmt;
	pI->pSource			= pbWave;
	pI->dSizeOfSource	= dwSize;

	return 0;
}


/*=============================================================================
//	Function Name	:	UINT8	GetWaveInfo_2(UINT8* fp, UINT32 size, UINT8 mode)
//
//	Description		:	Get data Info from the file
//
//	Argument		:	fp			: pointer to the data
//						size		: size fo the data
//						pvI			: pointer to work space
//
//	Return			: 0 : NoError, > 0 : Error
=============================================================================*/
static UINT8			GetWaveInfo_2(
	UINT8*				fp,
	UINT32				size,
	MALIB_PMAAUD_INFO	pI
)
{
	SINT32  dwSize;
	SINT32  dwSizeOfWave;
	UINT16  wNextChunk;
	UINT32  dwIndex;
	UINT32	dwFs;
	UINT8	bFmt;
	UINT8	bData;
	UINT8*	pbWave;

	dwIndex			= 6;
	size			-= 6;
	bFmt			= 0;
	dwFs			= 0;
	dwSize			= 0;
	dwSizeOfWave	= 0;
	pbWave			= NULL;

	bData = fp[2];
	if		(bData == 0x10)		dwFs = 4000;	/* Mono, ADPCM, 4KHz		  */
	else if	(bData == 0x11)		dwFs = 8000;	/* Mono, ADPCM, 8KHz		  */
	else						return 1;		/* Error					  */

	bData = fp[3];
	if		(bData == 0x00)		bFmt = 1;		/* 4bit						  */
	else						return 1;		/* Error					  */

	while(size > 8)
	{
		dwSize = NextChunk(6, &fp[dwIndex], size, &wNextChunk);
		if(dwSize < 0)							return 1;

		if((wNextChunk & 0x00FF) != 0x0011)
		{
			dwIndex += 8 + dwSize;
			size -= 8 + dwSize;
			continue;
		}
		pbWave			= &fp[dwIndex + 8];
		dwSizeOfWave	= dwSize;
		size = 0;
	}

	if ((UINT32)dwSizeOfWave <= (dwFs / 100))	return 2;

	pI->dFs				= dwFs;
	pI->dFormat			= (UINT32)bFmt;
	pI->pSource			= pbWave;
	pI->dSizeOfSource	= dwSizeOfWave;

	return 0;
}


/*=============================================================================
//	Function Name	:	SINT32	malib_smafaudio_checker(UINT8* fp, UINT32 fsize,
//															 UINT8 mode)
//
//	Description		:	Get data Info from the file
//
//	Argument		:	fp			: pointer to the data
//						fsize		: size fo the data
//						mode		: error check (0:No, 1:Yes, 2:CheckOnly, 3:OPDA)
//						pvI			: pointer to work space
//
//	Return			: >=0 : data length(msec), < 0 : Error
=============================================================================*/
SINT32					malib_smafaudio_checker(
	UINT8*				fp,
	UINT32				fsize,
	UINT8				mode,
	void*				pvI
)
{
	SINT32	dwTotalSize;
	UINT32	dwIndex;
	SINT32	dwSize;
	UINT16	wCRC;
	UINT16  wNextChunk;
	UINT8   bMTRError;
	UINT8	bType;
	UINT8	bClass;
	SINT32	ret;
	MALIB_PMAAUD_INFO	pI;

	if (fsize < 18)									return (MALIB_MA3AUDERR_ARGUMENT);

	if (pvI == NULL)		pI	= &sAudio_Check;
	else					pI	= (MALIB_PMAAUD_INFO)pvI;
	pI->dwSize	= 0;
	pI->pConti	= 0;
	pI->dFs		= 0;

	dwIndex = 0;
	wNextChunk = 0xFFFF;
	dwTotalSize = NextChunk(0, &fp[dwIndex], fsize, &wNextChunk);
	if ((dwTotalSize < 0) || (wNextChunk != 1))		return (MALIB_MA3AUDERR_FILE);
	if  (dwTotalSize < 10)							return (MALIB_MA3AUDERR_CHUNK_SIZE);

	if (mode != 0)
	{
		wCRC = (UINT16)(((UINT16)fp[dwTotalSize + 6] << 8) +
			   (UINT16)fp[dwTotalSize + 7]);
		if (wCRC != makeCRC(dwTotalSize + 8, fp))	return (MALIB_MA3AUDERR_FILE);
	}

	dwIndex = 8;
	dwSize = NextChunk(1, &fp[dwIndex], dwTotalSize, &wNextChunk);
	if ((dwSize < 0) || (wNextChunk != 2))			return (MALIB_MA3AUDERR_FILE);
	if (dwSize < 5)									return (MALIB_MA3AUDERR_CHUNK_SIZE);	

	ret = 0;
	bClass = fp[16];							/* Contents Class			  */
	bType  = fp[17];                            /* Contents Type              */
	pI->bCodeType = fp[18];						/* Contents Code Type		  */
	if(bType <= 0x2F) {							/* SMAF Type				  */
		pI->bSMAFType = 0;                      /* SMAF MA-1/2                */
		pI->bOptionType = 0;                    /* Option Information in CNTI */
	}
	else {
		pI->bOptionType = 1;                    /* Option Information in OPDA */
		if((bType & 0x0F) <= 0x01 )
			pI->bSMAFType = 0;                  /* SMAF MA-1/2                */
		else
			pI->bSMAFType = 1;                  /* SMAF MA-3                  */
	}

  	if ((bClass != CONTENTS_CLASS_1) && (bClass != CONTENTS_CLASS_2))
        	return MALIB_MA3AUDERR_C_CLASS;
	bType  = (UINT8)(fp[17] & 0xF0);
    if (bType <= 0x2F) {
        if ((bType != CONTENTS_TYPE_1) && (bType != CONTENTS_TYPE_2) &&
			(bType != CONTENTS_TYPE_3) && (mode != 3))
			ret = MALIB_MA3AUDERR_C_TYPE;
    }
    else {
        if ((bType != CONTENTS_TYPE_4) && (bType != CONTENTS_TYPE_5) &&
			(bType != CONTENTS_TYPE_6) && (mode != 3))
			ret = MALIB_MA3AUDERR_C_TYPE;
		bType  = (UINT8)(fp[17] & 0x0F);
		if (bType > 3)
			ret = MALIB_MA3AUDERR_C_TYPE;
    }

	pI->pConti = &(fp[16]);
	dwIndex = 16 + dwSize;
	if ((pI->bOptionType == 0) && (dwSize >= 9)){
		pI->pInfo = &(fp[21]);
		pI->dwSize = dwSize - 5;
	}
	else {
		pI->pInfo = NULL;
		pI->dwSize = 0;
	}

	dwSize = NextChunk(2, &fp[dwIndex], (UINT32)(dwTotalSize + 8 - dwIndex),
        &wNextChunk);
	if ((wNextChunk & 0x00FF) == 3)             /* Optional Data Chunk        */
	{
		if ((dwSize >= 12) && (pI->bOptionType == 1))
		{
			pI->pInfo = &(fp[dwIndex + 8]);
			pI->dwSize = (UINT32)dwSize;
		}
	}

	bMTRError = 0xFF;
	while((UINT32)dwTotalSize > (UINT32)(dwIndex + 8))
	{
		dwSize = NextChunk(2, &fp[dwIndex], (UINT32)(dwTotalSize + 8 - dwIndex),
			&wNextChunk);
		if(dwSize < 0)								return (MALIB_MA3AUDERR_FILE);

		switch((UINT8)(wNextChunk & 0x00FF)){
			case 0x04:							/* Score Track Chunk		  */
				if (((wNextChunk & 0xFF00) != 0x0500) || (pI->bSMAFType != 1)) {
					dwIndex += 8 + dwSize;
					break;
				}
				if (dwSize < 36)
				{
					dwIndex = dwTotalSize;
					bMTRError = 1;
					break;
				}
				dwIndex += 28;
				bMTRError = GetWaveInfo_3(&fp[dwIndex], (dwSize - 20), pI);
				dwIndex = dwTotalSize;
				break;
			case 0x05:							/* Audio Track Chunk		  */
				if (((wNextChunk & 0xFF00) != 0x0000) || (pI->bSMAFType != 0)) {
					dwIndex += 8 + dwSize;
					break;
				}
				if (dwSize < 14)
				{
					dwIndex = dwTotalSize;
					bMTRError = 1;
					break;
				}
				dwIndex += 8;
				bMTRError = GetWaveInfo_2(&fp[dwIndex], dwSize, pI);
				dwIndex = dwTotalSize;
				break;
			case 0x03:							/* Optional Data Chunk		  */
			case 0x06:							/* Graphics Data Chunk		  */
			case 0x07:							/* Master Track Chunk		  */
			case 0xFF:							/* Unknown Chunk			  */
				dwIndex += 8 + dwSize;
				break;
			default:
				return (MALIB_MA3AUDERR_FILE);
		}
	}
	if (mode == 3) {								pI->dFs = 0;	return 0;	}

	if (bMTRError != 0) {
		if (bMTRError == 1)							return (MALIB_MA3AUDERR_CHUNK);
		else										return (MALIB_MA3AUDERR_SHORT_LENGTH);
    }

	if (ret)										return ret;

	if (pI->dFs == 0)								return (MASMW_ERROR);

	switch(pI->dFormat) {
	case 0x01:	/*	4bit ADPCM mono		*/
		return	((pI->dSizeOfSource * 2000 + (pI->dFs - 1)) / pI->dFs);
	case 0x02:
	case 0x03:	/*	8bit PCM mono		*/
		return	((pI->dSizeOfSource * 1000 + (pI->dFs - 1)) / pI->dFs);
	default:
		return (MASMW_ERROR);
	}
}


