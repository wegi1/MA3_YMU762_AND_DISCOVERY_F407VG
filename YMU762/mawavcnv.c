/****************************************************************************
 *
 *		Copyright (C) 2002-2003	YAMAHA CORPORATION. All rights reserved.
 *
 *		Module		: mawavcnv.c
 *
 *		Description	: WAV Stream Converter Module.
 *
 *		Version		: 1.1.0.1		2003.04.09
 *
 *		History		:
 *			2002/09/09 1st try.
 *			2003/03/10 MA-5 Compatible version.
 ****************************************************************************/
#include "mawavcnv.h"

/*--------------------------------------------------------------------------*/
/*   Defines                                                                */
/*--------------------------------------------------------------------------*/
#define	WAVE_TIMEBASE			(20)			/* [ms]                    */
#define	MAWAVCNV_FORMAT_PCM		(1)
#define	MAWAVCNV_MAX_LOAD		(2)
#define	MAWAVCNV_MIN_LENGTH		(20)			/* [ms]                    */
#define	MAWAVCNV_MAX_LENGTH		(0x7FFFFFFF)	/* [ms]                    */
#define	MAWAVCNV_MAX_SIZE		(0x003FFFFF)	/* [byte]                  */
#define	MAWAVCNV_MIN_RATE		(4000)			/* [Hz]                    */
#define	MAWAVCNV_MAX_RATE		(24000)			/* [Hz]                    */
#define	MakeWord(a, b)			(UINT16)(((UINT16)a << 8) | (UINT16)b)
#define	MakeDword(a, b, c, d)	(UINT32)(((UINT32)a << 24) | ((UINT32)b << 16) | ((UINT32)c << 8) | (UINT32)d)

/*--------------------------------------------------------------------------*/
/*   Types                                                                  */
/*--------------------------------------------------------------------------*/
typedef struct _MAWAVCNV_INFO
{
	UINT8		bLoaded;						/*	Load Flag			*/
	UINT32		dFormatType;					/*						*/
	UINT32		dChannel;						/*	Number of Channels	*/
	UINT32		dFs;							/*	Fs[Hz]				*/
	UINT32		dSamplesPerSec;					/*	Samples per second	*/
	UINT32		dBlock;							/*	BytesPerSample		*/
	UINT32		dBit;							/*	Bit width			*/
	UINT8*		pbData;							/*	Top of PCM data		*/
	UINT32		dSize;							/*	Size of PCM [byte]	*/
	UINT32		dLength;						/*	Size of PCM [ms]	*/
} MAWAVCNV_INFO, *PMAWAVCNV_INFO;


typedef	struct _tagWavGlobals
{
	SINT32			sdSeqID;					/*	Sequence ID			*/
	UINT32			dMasterVol;					/*	master volume		*/
	UINT8			bPanpot;					/*	panpot				*/
	UINT32			dLoops;						/*	Loop number			*/
	UINT32			dRepeat;					/*	Number of repeat	*/
	SINT32			sdSeekTime;					/*	[ms]				*/
	SINT32			sdCurrentTime;				/*	[ms]				*/
	MAWAVCNV_INFO	DataInfo[MAWAVCNV_MAX_LOAD];
} WAVGLOBAL, *PWAVGLOBAL;


/*---------------------------------------------------------------------------*/
/*   Globals                                                                 */
/*---------------------------------------------------------------------------*/
static WAVGLOBAL	gWavInfo;
static PWAVGLOBAL	gpWavInfo;

/*---------------------------------------------------------------------------*/
/*   Functions (internal use only)                                           */
/*---------------------------------------------------------------------------*/

/*********************************************************************************
 *	Function Name	:	UINT32	GetBytePos(SINT32 sdFileID, UINT32 dTime)
 *
 *	Description		:	Get byte offset from time.
 *
 *	Argument		:	sdFileID	...	File ID.
 *						dTime		...	[ms]
 *
 *	Return			: Number of Bytes.
 *********************************************************************************/
static UINT32	GetBytePos(SINT32 sdFileID, UINT32 dTime)
{
	UINT32	dBytePosH;
	UINT32	dBytePosL;
	UINT32	dPos;
	UINT32	dPosH;
	UINT32	dPosL;
	
	/* dPos = dTime * 
	          gpWavInfo->DataInfo[sdFileID].dFs / 1000 * 
	          gpWavInfo->DataInfo[sdFileID].dBlock *
	          (gpWavInfo->DataInfo[sdFileID].dBit >> 3)
	*/
	dBytePosH = (dTime >> 16) * gpWavInfo->DataInfo[sdFileID].dFs;
	dBytePosL = (dTime & 0xFFFFL) * gpWavInfo->DataInfo[sdFileID].dFs;
	
	dPosL = dBytePosL & 0xFFFFL;
	dPosH = dBytePosH + (dBytePosL >> 16);
	
	dPos = dPosH / 1000;
	dPosH -= dPos * 1000;
	dPosL += dPosH << 16;
	dPos <<= 16;
	dPos += dPosL / 1000;
	dPos *= gpWavInfo->DataInfo[sdFileID].dBlock;

	if (dPos >= gpWavInfo->DataInfo[sdFileID].dSize) dPos = gpWavInfo->DataInfo[sdFileID].dSize - 1;

	return (dPos);
}

/*******************************************************************************
 *	Function Name	:	SINT32	WAVEChecker(SINT32 sdFileID, UINT8* pbWave, UINT32 dSize, UINT8 bMode)
 *
 *	Description		:	Check Wave format
 *							8bit Mono PCM
 *							8bit Stereo PCM
 *							16bit Mono PCM
 *
 *	Argument		:	sdFileID	...	File ID.
 *						pbWave	...	pointer to WAVE file.
 *						dSize	...	size fo WAVE file.
 *						bMode	...	check mode.
 *
 *	Return			: 0 : NoError, < 0 : Error
 *********************************************************************************/
static SINT32	WAVEChecker(SINT32 sdFileID, UINT8* pbWave, UINT32 dSize, UINT32 dMode)
{
	UINT8*	pbFmt;			/*	pointer to "fmt " chunk		*/
	UINT32	dTemp;

	UINT32	dId;
	UINT32	dRemain;
	UINT32	dRiffChunkSize;

	UINT8	bFlag = 0x00;	/*	bit0 : Found "fmt " chunk	*/
							/*	bit1 : Found "data" chunk	*/

	(void)dMode;
	
	/*	Check argument	*/
	if(pbWave == NULL)					return (MASMW_ERROR_ARGUMENT);
	if(dSize < 12L)						return (MASMW_ERROR_FILE);
	if(sdFileID > MAWAVCNV_MAX_LOAD)	return (MASMW_ERROR_ID);

	/*	Check "RIFF" and "WAVE" header ID	*/
	if(	(pbWave[0] != 'R')	||
		(pbWave[1] != 'I')	||
		(pbWave[2] != 'F')	||
		(pbWave[3] != 'F')	||
		(pbWave[8] != 'W')	||
		(pbWave[9] != 'A')	||
		(pbWave[10] != 'V')	||
		(pbWave[11] != 'E'))
	{
		MAWAVCNV_DBGMSG(("MaWavCnv : Error - Invalid header! \n"));
		return (MASMW_ERROR_FILE);
	}

	/*	Get "RIFF" chunk size	*/
	dRiffChunkSize = MakeDword(pbWave[7], pbWave[6], pbWave[5], pbWave[4]);

	/*	Check chunk size	*/
	if((dRiffChunkSize < 12L) || (dSize < (dRiffChunkSize + 8L))) {
		MAWAVCNV_DBGMSG(("MaWavCnv : Error - Invalid chunk size! \n"));
		return (MASMW_ERROR_CHUNK_SIZE);
	}

	/*	Search "fmt " and "data" header ID	*/
	pbWave = (pbWave + 12L);						/*	skip "RIFF" and "WAVE" header	*/
	dRemain = (SINT32)(dRiffChunkSize - 4L);
	while(dRemain > 8L) {
		dId		= MakeDword(pbWave[0], pbWave[1], pbWave[2], pbWave[3]);
		dSize	= MakeDword(pbWave[7], pbWave[6], pbWave[5], pbWave[4]);

		pbWave += 8L;
		dRemain -= 8L;

		/*	Invalid chunk size	*/
		if(dRemain < dSize) {
			MAWAVCNV_DBGMSG(("MaWavCnv : Error - Invalid chunk size! \n"));
			return (MASMW_ERROR_CHUNK_SIZE);
		}

		/*	Found format chunk	*/
		if((dId == MakeDword('f', 'm', 't', ' ')) && !(bFlag & 0x01)) {
			pbFmt = pbWave;

			/*	Check chunk size	*/
			if(dSize < 16L) {
				MAWAVCNV_DBGMSG(("MaWavCnv : Error - Invalid fmt chunk size(%d). \n", dSize));
				return (MASMW_ERROR_CHUNK_SIZE);
			}
			gpWavInfo->DataInfo[sdFileID].dFormatType		= (UINT32)MakeWord(pbFmt[1], pbFmt[0]);
			gpWavInfo->DataInfo[sdFileID].dChannel			= (UINT32)MakeWord(pbFmt[3], pbFmt[2]);
			gpWavInfo->DataInfo[sdFileID].dFs				= MakeDword(pbFmt[7], pbFmt[6], pbFmt[5], pbFmt[4]);
			gpWavInfo->DataInfo[sdFileID].dSamplesPerSec	= MakeDword(pbFmt[11], pbFmt[10], pbFmt[9], pbFmt[8]);
			gpWavInfo->DataInfo[sdFileID].dBlock			= (UINT32)MakeWord(pbFmt[13], pbFmt[12]);
			gpWavInfo->DataInfo[sdFileID].dBit				= (UINT32)MakeWord(pbFmt[15], pbFmt[14]);


			/*	Check format type	*/
			if(gpWavInfo->DataInfo[sdFileID].dFormatType != MAWAVCNV_FORMAT_PCM)
			{
				MAWAVCNV_DBGMSG(("MaWavCnv : Error - Invalid format(%ld). \n", gpWavInfo->DataInfo[sdFileID].dFormatType));
				return (MASMW_ERROR);
			}

			/*	Check number of channel & bit width	*/
			if (!((gpWavInfo->DataInfo[sdFileID].dChannel == 1) && (gpWavInfo->DataInfo[sdFileID].dBit == 8)))
			{
				MAWAVCNV_DBGMSG(("MaWavCnv : Error - Invalid Channel/BitWidth(%ld, %ld). \n", gpWavInfo->DataInfo[sdFileID].dChannel, gpWavInfo->DataInfo[sdFileID].dBit));
				return (MASMW_ERROR);
			}

			/*	Check sampling rate	*/
			dTemp = gpWavInfo->DataInfo[sdFileID].dFs * gpWavInfo->DataInfo[sdFileID].dBlock;
			if ((dTemp < MAWAVCNV_MIN_RATE) || (MAWAVCNV_MAX_RATE < dTemp))
			{
				MAWAVCNV_DBGMSG(("MaWavCnv : Error - Invalid Frequency(%d). \n", gpWavInfo->DataInfo[sdFileID].dFs));
				return (MASMW_ERROR);
			}
			
			/* Check Matching of block */
			if (gpWavInfo->DataInfo[sdFileID].dBlock != (gpWavInfo->DataInfo[sdFileID].dChannel * (gpWavInfo->DataInfo[sdFileID].dBit >> 3)))
			{
				MAWAVCNV_DBGMSG(("MaWavCnv : Error - Invalid block! \n"));
				/* Correct Invalid block */
				gpWavInfo->DataInfo[sdFileID].dBlock = (gpWavInfo->DataInfo[sdFileID].dChannel) * (gpWavInfo->DataInfo[sdFileID].dBit >> 3);
				/* 
				return (MASMW_ERROR_FILE);
				*/
			}
			
			bFlag |= 0x01;
		}
		else if ((dId == MakeDword('d', 'a', 't', 'a')) && !(bFlag & 0x02)) {
			/*	Found data chunk	*/
			gpWavInfo->DataInfo[sdFileID].pbData	= pbWave;
			gpWavInfo->DataInfo[sdFileID].dSize	= dSize;
			bFlag |= 0x02;
		}

		/*	skip chunk	*/
		pbWave += dSize;
		dRemain -= dSize;
		
		/*	Riff chunk size must be an even number	*/
		if ((dSize & 0x00000001) == 1)
		{
			if (dRemain > 0)
			{
				if (*pbWave == 0)
				{
					/*	skip padding	*/
					pbWave += 1;
					dRemain -= 1;
				}
			}
		}
	}

	if ((bFlag & 0x03) != 0x03) {
		/*	Not found "fmt " or "data" chunk.	*/
		gpWavInfo->DataInfo[sdFileID].pbData	= NULL;
		gpWavInfo->DataInfo[sdFileID].dSize	= 0L;
		return (MASMW_ERROR_FILE);
	}

	if (gpWavInfo->DataInfo[sdFileID].dSize > MAWAVCNV_MAX_SIZE) return (MASMW_ERROR_LONG_LENGTH);
	dTemp = gpWavInfo->DataInfo[sdFileID].dFs * gpWavInfo->DataInfo[sdFileID].dBlock;
	gpWavInfo->DataInfo[sdFileID].dLength = ((gpWavInfo->DataInfo[sdFileID].dSize * 1000L + (dTemp - 1)) / dTemp);
	if (gpWavInfo->DataInfo[sdFileID].dLength <= MAWAVCNV_MIN_LENGTH) return (MASMW_ERROR_SHORT_LENGTH);
	if (gpWavInfo->DataInfo[sdFileID].dLength > MAWAVCNV_MAX_LENGTH) return (MASMW_ERROR_LONG_LENGTH);

	MAWAVCNV_DBGMSG(("MaWavCnv : No Error!! \n"));
	MAWAVCNV_DBGMSG(("MaWavCnv : Bit:%d Ch:%d Fs:%d \n", gpWavInfo->DataInfo[sdFileID].dBit,
						gpWavInfo->DataInfo[sdFileID].dChannel,	gpWavInfo->DataInfo[sdFileID].dFs));

	return (0);
}


/*********************************************************************************
 *	Function Name	:	SINT32	MaWavCnv_SetVolume(SINT32 sdFileID, UINT8 bVol)
 *
 *	Description		:	Set Master volume
 *
 *	Argument		:	sdFileID			: File ID
 *						bVol			: Volume
 *
 *	Return			: 0 : NoError, < 0 : Error
 *********************************************************************************/
static SINT32	MaWavCnv_SetVolume(SINT32 sdFileID, UINT8 bVol)
{
	(void)sdFileID;

	MAWAVCNV_DBGMSG(("MaWavCnv_SetVolume[%08lX, %d] \n", sdFileID, bVol));

	gpWavInfo->dMasterVol = bVol & 0x7F;
	MaSndDrv_SetVolume(gpWavInfo->sdSeqID, (UINT8)gpWavInfo->dMasterVol);

	return (MASMW_SUCCESS);
}


/*********************************************************************************
 *	Function Name	:	SINT32	MaWavCnv_SetPanpot(SINT32 sdFileID, UINT8 bPan)
 *
 *	Description		:	Set Master panpot
 *
 *	Argument		:	sdFileID			: File ID
 *						bPan			: Panpot
 *
 *	Return			: 0 : NoError, < 0 : Error
 *********************************************************************************/
static SINT32	MaWavCnv_SetPanpot(SINT32 sdFileID, UINT8 bPan)
{
	(void)sdFileID;

	MAWAVCNV_DBGMSG(("MaWavCnv_SetPanpot[%08lX, %d] \n", sdFileID, bPan));

	gpWavInfo->bPanpot = (UINT8)(bPan & 0x7F);

	return (MASMW_SUCCESS);
}


/*********************************************************************************
 *	GetContentsData( SINT32 sdFile_id, PMASMW_CONTENTSINFO pvPtr )
 *
 *	Description:
 *			Get loaded contents information
 *	Argument: 
 *			sdFile_id		file id
 *			PMASMW_CONTENTSINFO pvPtr	pointer to struct MASMW_CONTENTSINFO
 *	Return:
 *			>= 0		contents binary data
 *
 ********************************************************************************/
static SINT32 GetContentsData(SINT32 sdFileID, PMASMW_CONTENTSINFO pvPtr)
{
	MAWAVCNV_DBGMSG(("MaWavCnv/ GetContentsData \n"));

	if ( pvPtr->size < 4 ) return MASMW_ERROR_ARGUMENT;

	if ( pvPtr->tag[0] =='W' )
	{
		*(pvPtr->buf ) = 0;
		*(pvPtr->buf + 1 ) = 0;
		*(pvPtr->buf + 2 ) = 0;
		*(pvPtr->buf + 3 ) = 0;

		switch  ( pvPtr->tag[1] ){
		case '0':		/* Get foramt type */
			*(pvPtr->buf) = (UINT8)((gpWavInfo->DataInfo[sdFileID].dFormatType) & 0xFF);
			*(pvPtr->buf + 1) =(UINT8)((gpWavInfo->DataInfo[sdFileID].dFormatType >> 8) & 0xFF);
			*(pvPtr->buf + 2) = 0;
			*(pvPtr->buf + 3) = 0;
			break;
		case '1':		/* Get channel numbers */
			*(pvPtr->buf) = (UINT8)((gpWavInfo->DataInfo[sdFileID].dChannel) & 0xFF);
			*(pvPtr->buf + 1) = 0;
			*(pvPtr->buf + 2) = 0;
			*(pvPtr->buf + 3) = 0;
			break;
		case '2':		/* Get sample rate */
			*(pvPtr->buf) = (UINT8)((gpWavInfo->DataInfo[sdFileID].dFs) & 0xFF);
			*(pvPtr->buf + 1) = (UINT8)((gpWavInfo->DataInfo[sdFileID].dFs >> 8) & 0xFF);
			*(pvPtr->buf + 2) = (UINT8)((gpWavInfo->DataInfo[sdFileID].dFs >> 16) & 0xFF);
			*(pvPtr->buf + 3) = (UINT8)((gpWavInfo->DataInfo[sdFileID].dFs >> 24) & 0xFF);
			break;
		case '3':		/* Get average transmit rate */
			*(pvPtr->buf) = (UINT8)((gpWavInfo->DataInfo[sdFileID].dSamplesPerSec) & 0xFF);
			*(pvPtr->buf + 1) = (UINT8)((gpWavInfo->DataInfo[sdFileID].dSamplesPerSec >> 8) & 0xFF);
			*(pvPtr->buf + 2) = (UINT8)((gpWavInfo->DataInfo[sdFileID].dSamplesPerSec >> 16) & 0xFF);
			*(pvPtr->buf + 3) = (UINT8)((gpWavInfo->DataInfo[sdFileID].dSamplesPerSec >> 24) & 0xFF);
			break;
		case '4':		/* Get data block size */
			*(pvPtr->buf) = (UINT8)((gpWavInfo->DataInfo[sdFileID].dBlock) & 0xFF);
			*(pvPtr->buf + 1) = (UINT8)((gpWavInfo->DataInfo[sdFileID].dBlock >> 8) & 0xFF);
			*(pvPtr->buf + 2) = 0;
			*(pvPtr->buf + 3) = 0;
			break;
		case '5':		/* Get bits par sample */
			*(pvPtr->buf) = (UINT8)((gpWavInfo->DataInfo[sdFileID].dBit) & 0xFF);
			*(pvPtr->buf + 1) = 0;
			*(pvPtr->buf + 2) = 0;
			*(pvPtr->buf + 3) = 0;
			break;
		case '6':		/* Get wave data size */
			*(pvPtr->buf) = (UINT8)((gpWavInfo->DataInfo[sdFileID].dSize) & 0xFF);
			*(pvPtr->buf + 1) = (UINT8)((gpWavInfo->DataInfo[sdFileID].dSize >> 8) & 0xFF);
			*(pvPtr->buf + 2) = (UINT8)((gpWavInfo->DataInfo[sdFileID].dSize >> 16) & 0xFF);
			*(pvPtr->buf + 3) = (UINT8)((gpWavInfo->DataInfo[sdFileID].dSize >> 24) & 0xFF);
			break;

		default:
			return MASMW_ERROR_ARGUMENT;
		}
		return 4;
	}
	return MASMW_ERROR;
}

/*---------------------------------------------------------------------------*/
/*   Functions                                                               */
/*---------------------------------------------------------------------------*/

/*********************************************************************************
 *	Function Name	:	SINT32	MaWavCnv_Initialize(void)
 *
 *	Description		:	Initialize()
 *
 *	Argument		:	none
 *
 *	Return			: 0 : NoError, < 0 : Error
 *********************************************************************************/
SINT32	MaWavCnv_Initialize(void)
{
	int				nIdx;
	PMAWAVCNV_INFO	pWi;

	MAWAVCNV_DBGMSG(("MaWavCnv_Initialize \n"));

	gpWavInfo = &gWavInfo;

	gpWavInfo->sdSeqID	= -1;
	gpWavInfo->dMasterVol	= 100;
	gpWavInfo->bPanpot	= 64;
	gpWavInfo->dLoops	= 0;
	gpWavInfo->dRepeat	= 1;
	gpWavInfo->sdSeekTime		= 0;
	gpWavInfo->sdCurrentTime	= 0;
	for(nIdx = 0; nIdx < MAWAVCNV_MAX_LOAD; nIdx++) {
		pWi = &(gpWavInfo->DataInfo[nIdx]);
		pWi->bLoaded	= 0;
		pWi->pbData		= NULL;
		pWi->dSize		= 0L;
	}

	return (MASMW_SUCCESS);
}


/*********************************************************************************
 *	Function Name	:	SINT32	MaWavCnv_End(void)
 *
 *	Description		:	Ending()
 *
 *	Argument		:	none
 *
 *	Return			: 0 : NoError, < 0 : Error
 *********************************************************************************/
SINT32	MaWavCnv_End(void)
{
	MAWAVCNV_DBGMSG(("MaWavCnv_End \n"));

	return (MASMW_SUCCESS);
}


/*********************************************************************************
 *	Function Name	: SINT32	MaWavCnv_Convert(void)
 *
 *	Description		: Time count. 
 *					  MaWavCnv_Convert() is called every WAVE_TIMEBASE[ms].
 *
 *	Argument		: none
 *
 *	Return			: 0
 *********************************************************************************/
SINT32	MaWavCnv_Convert(void)
{
	return (0);
}


/*********************************************************************************
 *	Function Name	:	SINT32	MaWavCnv_Load(UINT8* file_ptr, UINT32 file_size, UINT8 bMode,
 *												SINT32 (*func)(UINT8 id), void* pvExtArgs)
 *
 *	Description		:	Load()
 *
 *	Argument		:	pbFilePtr	: pointer to the data
 *						dFileSize	: size fo the data
 *						bMode		: error check (0:No, 1:Yes, 2:Check)
 *						func		: pointer of rhe callback function
 *						pvExtArgs	: NULL
 *
 *	Return			: >= 0 : File ID, < 0 : Error
 *********************************************************************************/
SINT32	MaWavCnv_Load(UINT8* pbFilePtr, UINT32 dFileSize, UINT8 bMode, SINT32 (*func)(UINT8 id), void* pvExtArgs)
{
	SINT32	sdFileID;
	SINT32	sdRet;

	(void)func;
	(void)pvExtArgs;

	MAWAVCNV_DBGMSG(("MaWavCnv_Load[%08lX %08lX %02X] \n", pbFilePtr, dFileSize, bMode));

	if (pbFilePtr == NULL)	return (MASMW_ERROR);
	if (dFileSize == 0)		return (MASMW_ERROR);

	sdFileID = ((bMode < 2) ? 1 : 0);
	if(sdFileID == 0) {
		/*	load OK!!	*/
		gpWavInfo->DataInfo[0].bLoaded	= 0;
		gpWavInfo->DataInfo[0].pbData	= NULL;
		gpWavInfo->DataInfo[0].dSize	= 0L;
	}
	else {
		/*	Already loaded!!	*/
		if(gpWavInfo->DataInfo[sdFileID].bLoaded == 1) {
			return (MASMW_ERROR);
		}
	}

	/*	Format check	*/
	sdRet = WAVEChecker(sdFileID, pbFilePtr, dFileSize, (UINT32)bMode);
	if(sdRet < 0) return (sdRet);

	/*	Load Successful	*/
	gpWavInfo->DataInfo[sdFileID].bLoaded = 1;

	return (sdFileID);
}


/*********************************************************************************
 *	Function Name	:	SINT32	MaWavCnv_Unload(SINT32 sdFileID, void* pvExtArgs)
 *
 *	Description		:	Unload()
 *
 *	Argument		:	sdFileID		: File ID
 *						pvExtArgs	: NULL
 *
 *	Return			: 0 : NoError, < 0 : Error
 *********************************************************************************/
SINT32	MaWavCnv_Unload(SINT32 sdFileID, void* pvExtArgs)
{
	(void)pvExtArgs;

	MAWAVCNV_DBGMSG(("MaWavCnv_Unload[%d] \n", sdFileID));

	if(	(sdFileID < 0)					||
		(MAWAVCNV_MAX_LOAD <= sdFileID)	||
		(gpWavInfo->DataInfo[sdFileID].bLoaded == 0))
	{
		return (MASMW_ERROR);
	}

	gpWavInfo->DataInfo[sdFileID].bLoaded	= 0;
	gpWavInfo->DataInfo[sdFileID].pbData	= NULL;
	gpWavInfo->DataInfo[sdFileID].dSize		= 0L;

	return (MASMW_SUCCESS);
}


/*********************************************************************************
 *	Function Name	:	MaWavCnv_Open(SINT32 sdFileID, UINT16 wMode, void* pvExtArgs)
 *
 *	Description		:	Open()
 *
 *	Argument		:	sdFileID	: File ID
 *						wMode		: Resouce mode
 *						pvExtArgs	: NULL
 *
 *	Return			: 0 : NoError, < 0 : Error
 *********************************************************************************/
SINT32	MaWavCnv_Open(SINT32 sdFileID, UINT16 wMode, void* pvExtArgs)
{
	SINT32	sdRet = MASMW_SUCCESS;
	UINT32	dRam;
	UINT32	dSize;
	(void)wMode;
	(void)pvExtArgs;

	MAWAVCNV_DBGMSG(("MaWavCnv_Open[%d] \n", sdFileID));

	if(	(sdFileID < 1)					||
		(MAWAVCNV_MAX_LOAD <= sdFileID)	||
		(gpWavInfo->DataInfo[sdFileID].bLoaded == 0))
	{
		return (MASMW_ERROR);
	}

	sdRet = MaSndDrv_Create((UINT8)MASMW_SEQTYPE_AUDIO,		/* AudioMode					 */
							(UINT8)MASMW_AUDIO_BASETIME,	/* Timer Setting				 */
							(UINT8)0,						/* Option : ---					 */
							(UINT8)0,						/* ResMode						 */
							1,								/* Num of Stream				 */
							MaWavCnv_Convert,				/* Callback Fn					 */
							&dRam,							/* Top Address of Reseved RAM	 */
							&dSize);						/* Size of Reserved RAM			 */

	if (sdRet < 0) return (sdRet);

	gpWavInfo->sdSeqID	= sdRet;
	gpWavInfo->dMasterVol= 100;
	gpWavInfo->bPanpot	= 64;
	gpWavInfo->dLoops	= 0;
	gpWavInfo->dRepeat	= 1;
	gpWavInfo->sdSeekTime		= 0;
	gpWavInfo->sdCurrentTime	= 0;

	MaSndDrv_SetVolume(gpWavInfo->sdSeqID, (UINT8)gpWavInfo->dMasterVol);
	MaSndDrv_SetCommand(gpWavInfo->sdSeqID, -1, MASNDDRV_CMD_MASTER_VOLUME, (UINT8)0x7F, 0, 0);

	return (MASMW_SUCCESS);
}


/*********************************************************************************
 *	Function Name	:	SINT32	MaWavCnv_Close(SINT32 sdFileID, void* pvExtArgs)
 *
 *	Description		:	Close()
 *
 *	Argument		:	sdFileID		: File ID
 *						pvExtArgs	: NULL
 *
 *	Return			: 0 : NoError, < 0 : Error
 *********************************************************************************/
SINT32	MaWavCnv_Close(SINT32 sdFileID, void* pvExtArgs)
{
	(void)pvExtArgs;

	MAWAVCNV_DBGMSG(("MaAudCnv_Close[%d] \n", sdFileID));

	if(	(sdFileID < 1)					||
		(MAWAVCNV_MAX_LOAD <= sdFileID)	||
		(gpWavInfo->DataInfo[sdFileID].bLoaded == 0))
	{
		return (MASMW_ERROR);
	}

	MaSndDrv_Free(gpWavInfo->sdSeqID);
	gpWavInfo->sdSeqID = -1;

	return (MASMW_SUCCESS);
}


/*********************************************************************************
 *	Function Name	:	SINT32	MaWavCnv_Standby(SINT32 sdFileID, void* pvExtArgs)
 *
 *	Description		:	Standby()
 *
 *	Argument		:	sdFileID		: File ID
 *						pvExtArgs	: NULL
 *
 *	Return			: 0 : NoError, < 0 : Error
 *********************************************************************************/
SINT32	MaWavCnv_Standby(SINT32 sdFileID, void* pvExtArgs)
{
	SINT32	sdRet;
	UINT32	dFormat;
	
	(void)pvExtArgs;

	MAWAVCNV_DBGMSG(("MaWavCnv_Standby[%08lX]\n", sdFileID));

	if(	(sdFileID < 1)					||
		(MAWAVCNV_MAX_LOAD <= sdFileID)	||
		(gpWavInfo->DataInfo[sdFileID].bLoaded == 0))
	{
		return (MASMW_ERROR);
	}
	
	if (gpWavInfo->DataInfo[sdFileID].dBit == 8)
	{
		dFormat = 0x02;														/* 8bit offset PCM */
	}
	else
	{
		dFormat = 0x04;														/* 16bit PCM */
	}
	if (gpWavInfo->DataInfo[sdFileID].dChannel == 2) dFormat |= 0x80;		/* Stereo */

	sdRet = MaSndDrv_SetStream(gpWavInfo->sdSeqID, 0, (UINT8)dFormat, gpWavInfo->DataInfo[sdFileID].dFs,
				 gpWavInfo->DataInfo[sdFileID].pbData, gpWavInfo->DataInfo[sdFileID].dSize);

	return (sdRet);
}


/*********************************************************************************
 *	Function Name	: SINT32	MaWavCnv_Seek(SINT32 sdFileID, UINT32 pos, UINT8 bFlag, void* pvExtArgs)
 *
 *	Description		: Seek() , Not Supported.
 *
 *	Argument		: sdFileID		: File ID
 *					  pos			: Start position(msec)
 *					  flag			: Wait 0:No wait, 1:20[ms] wait
 *					  pvExtArgs		: NULL
 *
 *	Return			: 0 : NoError, < 0 : Error
 *********************************************************************************/
SINT32	MaWavCnv_Seek(SINT32 sdFileID, UINT32 dPos, UINT8 bFlag, void* pvExtArgs)
{
	(void)bFlag;
	(void)pvExtArgs;

	MAWAVCNV_DBGMSG(("MaWavCnv_Seek[%08lX] = %ld \n", sdFileID, dPos));

	if(	(sdFileID < 1)					||
		(MAWAVCNV_MAX_LOAD <= sdFileID)	||
		(gpWavInfo->DataInfo[sdFileID].bLoaded == 0))
	{
		return (MASMW_ERROR);
	}
	gpWavInfo->sdSeekTime = dPos;
	gpWavInfo->sdCurrentTime = dPos;
	MaSndDrv_ControlSequencer(gpWavInfo->sdSeqID, 4);

	return (MASMW_SUCCESS);
}


/*********************************************************************************
 *	Function Name	:	SINT32	MaWavCnv_Start(SINT32 sdFileID, void* pvExtArgs)
 *
 *	Description		:	Start()
 *
 *	Argument		:	sdFileID	: File ID
 *						pvExtArgs	: NULL
 *
 *	Return			: 0 : NoError, < 0 : Error
 *********************************************************************************/
SINT32	MaWavCnv_Start(SINT32 sdFileID, void* pvExtArgs)
{
	(void)pvExtArgs;

	MAWAVCNV_DBGMSG(("MaWavCnv_Start[%d] \n", sdFileID));

	if(	(sdFileID < 1)					||
		(MAWAVCNV_MAX_LOAD <= sdFileID)	||
		(gpWavInfo->DataInfo[sdFileID].bLoaded == 0))
	{
		return (MASMW_ERROR);
	}
	
	MaSndDrv_SetCommand(gpWavInfo->sdSeqID, -1, MASNDDRV_CMD_STREAM_SEEK, 0, 0, GetBytePos(sdFileID, gpWavInfo->sdSeekTime));
	MaSndDrv_ControlSequencer(gpWavInfo->sdSeqID, 4);
	gpWavInfo->sdCurrentTime = gpWavInfo->sdSeekTime;

	/* control sequencer */
	MaSndDrv_ControlSequencer(gpWavInfo->sdSeqID, 1);
	MaSndDrv_SetCommand(gpWavInfo->sdSeqID, -1, MASNDDRV_CMD_STREAM_PANPOT, 0, gpWavInfo->bPanpot, 0);
	MaSndDrv_SetCommand(gpWavInfo->sdSeqID, -1, MASNDDRV_CMD_STREAM_ON, 0, 0, 127);

	return (MASMW_SUCCESS);
}


/*********************************************************************************
 *	Function Name	:	SINT32	MaWavCnv_Stop(SINT32 sdFileID, void* pvExtArgs)
 *
 *	Description		:	Stop()
 *
 *	Argument		:	sdFileID		: File ID
 *						pvExtArgs	: NULL
 *
 *	Return			: 0 : NoError, < 0 : Error
 *********************************************************************************/
SINT32	MaWavCnv_Stop(SINT32 sdFileID, void* pvExtArgs)
{
	(void)pvExtArgs;

	MAWAVCNV_DBGMSG(("MaWavCnv_Stop[%d] \n", sdFileID));

	if(	(sdFileID < 1)					||
		(MAWAVCNV_MAX_LOAD <= sdFileID)	||
		(gpWavInfo->DataInfo[sdFileID].bLoaded == 0))
	{
		return (MASMW_ERROR);
	}

	/* control sequencer */
	MaSndDrv_ControlSequencer(gpWavInfo->sdSeqID, 0);
	MaSndDrv_SetCommand(gpWavInfo->sdSeqID, -1, MASNDDRV_CMD_STREAM_OFF, 0, 0, 0);

	gpWavInfo->sdSeekTime = 0;

	return (MASMW_SUCCESS);
}


/*********************************************************************************
 *	Function Name	:	SINT32	MaWavCnv_Control(SINT32 sdFileID, UINT8 bCtrlNum, void* prm, void* pvExtArgs)
 *
 *	Description		:	Control
 *
 *	Argument		:	sdFileID		: File ID
 *						bCtrlNum		: Control command ID
 *						prm				: Parameters
 *						pvExtArgs		: NULL
 *
 *	Return			: 0 : NoError, < 0 : Error
 *********************************************************************************/
SINT32	MaWavCnv_Control(SINT32 sdFileID, UINT8 bCtrlNum, void* pvPrm, void* pvExtArgs)
{
	long	dsPos;
	(void)pvExtArgs;

	MAWAVCNV_DBGMSG(("MaWavCnv_Control[%d, %d] \n", sdFileID, bCtrlNum));

	if(	(sdFileID < 0) || (MAWAVCNV_MAX_LOAD <= sdFileID))
	{
		return (MASMW_ERROR);
	}
	if (gpWavInfo->DataInfo[sdFileID].bLoaded == 0) return (MASMW_ERROR);

	switch(bCtrlNum)
	{
	case MASMW_GET_LENGTH :
		return ((SINT32)gpWavInfo->DataInfo[sdFileID].dLength);

	case MASMW_GET_POSITION :
		if(sdFileID < 1) {
			return (MASMW_ERROR_ARGUMENT);
		}
		dsPos = MaSndDrv_GetPos(gpWavInfo->sdSeqID) + gpWavInfo->sdCurrentTime;
		if (gpWavInfo->DataInfo[sdFileID].dLength < (UINT32)dsPos) return ((SINT32)gpWavInfo->DataInfo[sdFileID].dLength);
		return (dsPos);

	case MASMW_SET_VOLUME :
		if(sdFileID < 1) {
			return (MASMW_ERROR_ARGUMENT);
		}
		return (MaWavCnv_SetVolume(sdFileID, *((UINT8*)pvPrm)));

	case MASMW_SET_PANPOT :
		if(sdFileID < 1) {
			return (MASMW_ERROR_ARGUMENT);
		}
		return (MaWavCnv_SetPanpot(sdFileID, *((UINT8*)pvPrm)));

	case MASMW_GET_CONTENTSDATA :
		return GetContentsData(sdFileID, (PMASMW_CONTENTSINFO)pvPrm);
	}

	return (MASMW_ERROR);
}

