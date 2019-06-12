/*==============================================================================
//	Copyright (C) 2001-2003 YAMAHA CORPORATION
//
//	Title		: MARMDCNV.C
//
//	Description	: MA-3 Realtime MIDI/GML Stream Converter Module.
//
//	Version		: 1.5.2.0
//
//	History		:
//				  May 8, 2001			1st try.
//				  July 16, 2001			Fix warnings.
//				  July 18, 2001			Add #BankMSB = 0x78/0x79.
//				  July 24, 2001			Fix MidVolTable.
//				  July 31, 2001			Change MonoModeOn message.
//				  September 18, 2001	Set MAX_MID_GAIN = -9[dB].
//				  Nobember 2, 2001		Change NoteOn(vel=0) => NoteOff().
//				  May 15, 2002			Add MasterVolume(Universal SysEx).
//============================================================================*/
#include "marmdcnv.h"

#define	MAX_MID_GAIN		76				/* - 9[dB] : 76             */
											/* -12[dB] : 64             */
											/* -18[dB] : 45             */

#define RMD_INTERVAL_CB		17

#define	DVA_NORMAL			0
#define	DVA_SIMPLE			2
#define	DVA_MONO			4
#define	DRUM_NORMAL			0
#define	DRUM_SIMPLE			1
#define	MELODY_NORMAL		0
#define	MELODY_SIMPLE		8
#define DRUM_GML1			0x20
#define ENC_8BIT			0
#define ENC_7BIT			1

typedef	struct _MASMW_GETCTL
{
	UINT8	bControl;				/* contorl number */
	UINT8 	bCh;					/* channel number */
} MASMW_GETCTL, *PMASMW_GETCTL;

/*=============================================================================
//	gloval values
=============================================================================*/

static const UINT32	gdwDBVolTable[128] =	/* -80 * Log(index/127) */
{
	 192, 168, 144, 130, 120, 112, 106, 101,
	  96,  92,  88,  85,  82,  79,  77,  74,
	  72,  70,  68,  66,  64,  63,  61,  59,
	  58,  56,  55,  54,  53,  51,  50,  49,
	  48,  47,  46,  45,  44,  43,  42,  41,
	  40,  39,  38,  38,  37,  36,  35,  35,
	  34,  33,  32,  32,  31,  30,  30,  29,
	  28,  28,  27,  27,  26,  25,  25,  24,
	  24,  23,  23,  22,  22,  21,  21,  20,
	  20,  19,  19,  18,  18,  17,  17,  16,
	  16,  16,  15,  15,  14,  14,  14,  13,
	  13,  12,  12,  12,  11,  11,  10,  10,
	  10,   9,   9,   9,   8,   8,   8,   7,
	   7,   7,   6,   6,   6,   5,   5,   5,
	   4,   4,   4,   3,   3,   3,   3,   2,
	   2,   2,   1,   1,   1,   1,   0,   0
};

static const UINT32	gdwMidVolTable[193] =	/* 127*10^(index / 80) */
{
	 127,  123,  120,  116,  113,  110,  107,  104,
	 101,   98,   95,   93,   90,   87,   85,   82,
	  80,   78,   76,   74,   71,   69,   67,   66,
	  64,   62,   60,   58,   57,   55,   54,   52,
	  51,   49,   48,   46,   45,   44,   43,   41,
	  40,   39,   38,   37,   36,   35,   34,   33,
	  32,   31,   30,   29,   28,   28,   27,   26,
	  25,   25,   24,   23,   23,   22,   21,   21,
	  20,   20,   19,   18,   18,   17,   17,   16,
	  16,   16,   15,   15,   14,   14,   13,   13,
	  13,   12,   12,   12,   11,   11,   11,   10,
	  10,   10,   10,    9,    9,    9,    8,    8,
	   8,    8,    8,    7,    7,    7,    7,    7,
	   6,    6,    6,    6,    6,    6,    5,    5,
	   5,    5,    5,    5,    5,    4,    4,    4,
	   4,    4,    4,    4,    4,    3,    3,    3,
	   3,    3,    3,    3,    3,    3,    3,    3,
	   3,    2,    2,    2,    2,    2,    2,    2,
	   2,    2,    2,    2,    2,    2,    2,    2,
	   2,    2,    2,    1,    1,    1,    1,    1,
	   1,    1,    1,    1,    1,    1,    1,    1,
	   1,    1,    1,    1,    1,    1,    1,    1,
	   1,    1,    1,    1,    1,    1,    1,    1,
	   1,    1,    1,    1,    1,    1,    1,    1,
	   0
};

static 	SINT32 (*gCallbackFunc)(UINT8 id);			/*              */

static SINT32		gSeqID;							/* Sequence ID  */
static SINT32		gFileID;						/* File ID      */
static UINT8		gEnable;						/* 0:disabel    */
static UINT8		gNumOfLoaded;					/*              */

static UINT8		gbMaxGain;						/* MaxGain      */
static UINT8		gbMasterVolume;					/* MasterVolume */

static UINT32		gRamBase;						/* */
static UINT32		gRamOffset;						/* */
static UINT32		gRamSize;						/* */

static UINT32		gWtWaveAdr[128];				/* */   

static UINT16		gwBank[16];
static UINT16		gwRPN[16];
static UINT8		gbExpression[16];
static UINT8		gbChVolume[16];					/* */
static UINT8		gbIntervalTime;					/* 0 or 20[ms]  */


/*=============================================================================
//	Function Name	: SINT32	SendMasterVolume(UINT8 bVol)
//
//	Description		: Send SetMasterVolume message
//
//	Argument		: bVol	Master volume (0..127)
//
//	Return			: 0 : NoError, < 0 : Error
=============================================================================*/
static SINT32	SendMasterVolume(UINT8 bVol)
{
	if (gSeqID > 0)
	{
		MaSndDrv_SetCommand(gSeqID, -1, MASNDDRV_CMD_MASTER_VOLUME, (UINT32)bVol, 0, 0);
	}
	return (MA3RMDERR_NOERROR);
}


/*==============================================================================
//	Function Name	: SINT32	ErrorFunction(UINT8 bCh, UINT8 bPara1, UINT8 bPara2)
//
//	Description		: Invalid messages
//
//	Argument		: bCh		MIDI channel (0..15)
//					  bPara1	Undefined
//					  bPara1	Undefined
//
//	Return			: 0 : NoError, < 0 : Error
//
==============================================================================*/
static SINT32	ErrorFunction(UINT8 bCh, UINT8 bPara1, UINT8 bPara2)
{
	(void)bCh;
	(void)bPara1;
	(void)bPara2;

	MARMDCNV_DBGMSG(("ErrorFunction\n"));

	return (MA3RMDERR_ARGUMENT);
}


/*==============================================================================
//	Function Name	:	SINT32	NotSupported(UINT8 bCh, UINT8 bPara1, UINT8 bPara2)
//
//	Description		:	Unsupported messages
//
//	Argument		: bCh		MIDI channel (0..15)
//					  bPara1	Undefined
//					  bPara1	Undefined
//
//	Return			: 0 : NoError, < 0 : Error
==============================================================================*/
static SINT32	NotSupported(UINT8 bCh, UINT8 bPara1, UINT8 bPara2)
{
	(void)bCh;
	(void)bPara1;
	(void)bPara2;

	MARMDCNV_DBGMSG(("NotSupported\n"));

	return (MA3RMDERR_NOERROR);
}


/*==============================================================================
//	Function Name	: SINT32	SendNoteOff(UINT8 bCh, UNIT8 bKey, UINT8 bVel)
//
//	Description		: Send NoteOff message
//
//	Argument		: bCh			...	#Channel(0..15)
//					  bKey			...	#Key (0..127)
//					  bVel			...	Velocity (0..127)
//
//	Return			: 0 : NoError, < 0 : Error
==============================================================================*/
static SINT32	SendNoteOff(UINT8 bCh, UINT8 bKey, UINT8 bVel)
{
	(void)bVel;

	MARMDCNV_DBGMSG(("SendNoteOff[%d, %d] \n", bCh, bKey));

	return (MaSndDrv_SetCommand(gSeqID, -1, MASNDDRV_CMD_NOTE_OFF, (UINT32)bCh, (UINT32)bKey, 0));
}


/*==============================================================================
//	Function Name	: SINT32	SendNoteOn(UINT8 bCh, UINT8 bKey, UINT8 bVel)
//
//	Description		: Send NoteOn message
//
//	Argument		: bCh			...	#Channel(0..15)
//					  bKey			...	#Key (0..127)
//					  bVel			...	Velocity (0..127)
//
//	Return			: 0 : NoError, < 0 : Error
==============================================================================*/
static SINT32	SendNoteOn(UINT8 bCh, UINT8 bKey, UINT8 bVel)
{
#if !RMD_DEF_VELOCITY_MODE
	static const UINT8	VelocityTable[128] = 
	{
		  0,    0,   16,   20,   23,   25,   28,   30,
		 32,   34,   36,   38,   39,   40,   43,   44,
		 45,   46,   48,   49,   51,   52,   54,   54,
		 55,   57,   57,   58,   60,   60,   62,   64,
		 64,   66,   66,   67,   67,   69,   69,   69,
		 71,   71,   74,   74,   76,   76,   76,   78,
		 78,   78,   80,   80,   80,   82,   82,   82,
		 85,   85,   85,   87,   87,   87,   90,   90,
		 90,   90,   93,   93,   93,   93,   95,   95,
		 95,   95,   98,   98,   98,   98,  101,  101,
		101,  101,  101,  104,  104,  104,  104,  104,
		107,  107,  107,  107,  107,  110,  110,  110,
		110,  110,  110,  113,  113,  113,  113,  113,
		116,  116,  116,  116,  116,  116,  120,  120,
		120,  120,  120,  120,  120,  123,  123,  123,
		123,  123,  123,  123,  127,  127,  127,  127
	};
#endif

	if (bVel == 0) return(SendNoteOff(bCh, bKey, bVel));

	MARMDCNV_DBGMSG(("SendNoteOn[%d, %d, %d]\n", bCh, bKey, bVel));

#if RMD_DEF_VELOCITY_MODE
	return (MaSndDrv_SetCommand(gSeqID, -1, MASNDDRV_CMD_NOTE_ON, (UINT32)bCh, (UINT32)bKey, (UINT32)bVel));
#else
	return (MaSndDrv_SetCommand(gSeqID, -1, MASNDDRV_CMD_NOTE_ON, (UINT32)bCh, (UINT32)bKey, (UINT32)VelocityTable[bVel & 0x7f]));
#endif
}


/*==============================================================================
//	Function Name	: SINT32	SendProgram(UINT8 bCh, UNIT8 bProg, UINT8 bPara2)
//
//	Description		: Send SetProgram message
//
//	Argument		: bCh			...	#Channel(0..15)
//					  bProg			...	#Program(0..127)
//
//	Return			: 0 : NoError, < 0 : Error
==============================================================================*/
static SINT32	SendProgram(UINT8 bCh, UINT8 bProg, UINT8 bPara2)
{
	UINT8	bBk, bPg;
	UINT16	wBb;
	UINT8	bPp;

	static const UINT8	bMBank[128] = 
	{
		1, 2, 3, 4, 5, 6, 7, 8, 9, 10,0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
	};
	(void)bPara2;
	
	MARMDCNV_DBGMSG(("SendProgram[%d] = %d\n", bCh, bProg));

	bPp = (UINT8)(bProg & 0x7f);
	wBb = gwBank[bCh];
	
	if (wBb == 0)
	{
		if (bCh == 9)
		{
			bBk = 128;
			bPg = 0;
		} else {
			bBk = 0;
			bPg = bPp;
		}
	} else {
		switch (wBb & 0xFF00)
		{
		case 0x7900:	/* Melody */
			bBk = 0;
			bPg = bPp;
			break;

		case 0x7800:	/* Drum */
			bBk = 128;
			bPg = 0;
			break;

		case 0x7C00:	/* Melody */
			bBk = (UINT8)(bMBank[wBb & 0x7F]);
			bPg = bPp;
			break;

		case 0x7D00:	/* Drum */
			bBk = (UINT8)(128 + bMBank[bPp]);
			bPg = 0;
			break;

		default:
			return (MA3RMDERR_ARGUMENT);
		}
	}
	return (MaSndDrv_SetCommand(gSeqID, -1, MASNDDRV_CMD_PROGRAM_CHANGE, (UINT32)bCh, (UINT32)bBk, (UINT32)bPg));
}

/*==============================================================================
//	Function Name	: SINT32	SendModDepth(UINT8 bCh, UINT8 bMod)
//
//	Description		: Send SetModulation message
//
//	Argument		: bCh			...	#Channel(0..15)
//					  bMod			...	Modulation(0..127)
//
//	Return			: 0 : NoError, < 0 : Error
==============================================================================*/
static SINT32	SendModDepth(UINT8 bCh, UINT8 bMod)
{
	static const UINT8	ModTable[128] = 
	{
		0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
		2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
		2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2,
		3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
		3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
		4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
		4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4
	};

	MARMDCNV_DBGMSG(("SendModDepth[%d] = %d\n", bCh, bMod));

	return (MaSndDrv_SetCommand(gSeqID, -1, MASNDDRV_CMD_MODULATION_DEPTH, (UINT32)bCh, (UINT32)ModTable[bMod], 0));
}


/*==============================================================================
//	Function Name	: SINT32	SendChVol(UINT8 bCh, UINT8 bVol)
//
//	Description		: Send ChannelVolume nessage
//
//	Argument		: bCh			...	#Channel(0..15)
//					  bVol			...	Volume(0..127)
//
//	Return			: 0 : NoError, < 0 : Error
==============================================================================*/
static SINT32	SendChVol(UINT8 bCh, UINT8 bVol)
{
	MARMDCNV_DBGMSG(("SendChVol[%d] =  %d\n", bCh, bVol));

	gbChVolume[bCh] = bVol;

	return (MaSndDrv_SetCommand(gSeqID, -1, MASNDDRV_CMD_CHANNEL_VOLUME, (UINT32)bCh, (UINT32)bVol, 0));
}


/*==============================================================================
//	Function Name	: SINT32	SendExpression(UINT8 bCh, UINT8 bVol)
//
//	Description		: Send Expression message
//
//	Argument		: bCh			...	#Channel(0..15)
//					  bVol			...	Volume(0..127)
//
//	Return			: 0 : NoError, < 0 : Error
==============================================================================*/
static SINT32	SendExpression(UINT8 bCh, UINT8 bVol)
{
	UINT32	dwVol;
	
	MARMDCNV_DBGMSG(("SendExpression[%d] =  %d\n", bCh, bVol));

	gbExpression[bCh] = bVol;
	
	dwVol = gdwDBVolTable[bVol & 0x7f] + gdwDBVolTable[gbMasterVolume];
	if (dwVol > 192) dwVol = 192;
	dwVol = gdwMidVolTable[dwVol];

	return (MaSndDrv_SetCommand(gSeqID, -1, MASNDDRV_CMD_EXPRESSION, (UINT32)bCh, dwVol, 0));
}


/*==============================================================================
//	Function Name	: SINT32	SendPanpot(UINT8 bCh, UINT8 bPan)
//
//	Description		: Set Panpot message
//
//	Argument		: bCh			...	#Channel(0..15)
//					  bPan			...	Panpot(0..127)
//
//	Return			: 0 : NoError, < 0 : Error
==============================================================================*/
static SINT32	SendPanpot(UINT8 bCh, UINT8 bPan)
{
	MARMDCNV_DBGMSG(("SendPanpot[%d] = %d\n", bCh, bPan));

	return (MaSndDrv_SetCommand(gSeqID, -1, MASNDDRV_CMD_PANPOT, (UINT32)bCh, (UINT32)bPan, 0));
}


/*==============================================================================
//	Function Name	: SINT32	SendHold1(UINT8 bCh, UINT8 bVal)
//
//	Description		: Send Hold1 message
//
//	Argument		: bCh			...	#Channel(0..15)
//					  bVal			...	Hold1(0..127)
//
//	Return			: 0 : NoError, < 0 : Error
==============================================================================*/
static SINT32	SendHold1(UINT8 bCh, UINT8 bVal)
{
	MARMDCNV_DBGMSG(("SendHold1[%d] = %d\n", bCh, bVal));

	return (MaSndDrv_SetCommand(gSeqID, -1, MASNDDRV_CMD_HOLD1, (UINT32)bCh, (UINT32)((bVal < 64) ? 0 : 1), 0));
}


/*==============================================================================
//	Function Name	: SINT32	DataEntryMSB(UINT8 bCh, UINT8 bVal)
//
//	Description		: Send BendRange message
//
//	Argument		: bCh			...	#Channel(0..15)
//					  bVal			...	data(0..127)
//
//	Return			: 0 : NoError, < 0 : Error
==============================================================================*/
static SINT32	DataEntryMSB(UINT8 bCh, UINT8 bVal)
{
	MARMDCNV_DBGMSG(("DataEntryMSB[%d] = %d\n", bCh, bVal));

	if (gwRPN[bCh] != 0) return (MA3RMDERR_NOERROR);

	return (MaSndDrv_SetCommand(gSeqID, -1, MASNDDRV_CMD_BEND_RANGE, (UINT32)bCh, (UINT32)bVal, 0));
}


/*==============================================================================
//	Function Name	: SINT32	SendAllSoundOff(UINT8 bCh)
//
//	Description		: Send AllSoundOff message
//
//	Argument		: bCh			...	#Channel(0..15)
//
//	Return			: 0 : NoError, < 0 : Error
==============================================================================*/
static SINT32	SendAllSoundOff(UINT8 bCh)
{
	MARMDCNV_DBGMSG(("SendAllSoundOff[%d] \n", bCh));

	return (MaSndDrv_SetCommand(gSeqID, -1, MASNDDRV_CMD_ALL_SOUND_OFF, (UINT32)bCh, 0, 0));
}


/*==============================================================================
//	Function Name	: SINT32	SendAllNoteOff(UINT8 bCh)
//
//	Description		: Message AllNoteOff message
//
//	Argument		: bCh			...	#Channel(0..15)
//
//	Return			: 0 : NoError, < 0 : Error
==============================================================================*/
static SINT32	SendAllNoteOff(UINT8 bCh)
{
	MARMDCNV_DBGMSG(("SendAllNoteOff[%d] \n", bCh));

	return (MaSndDrv_SetCommand(gSeqID, -1, MASNDDRV_CMD_ALL_NOTE_OFF, (UINT32)bCh, 0, 0));
}


/*==============================================================================
//	Function Name	: SINT32	SendResetAllCtl(UINT8 bCh)
//
//	Description		: Send ResetAllControllers message
//
//	Argument		: bCh			...	#Channel(0..15)
//
//	Return			: 0 : NoError, < 0 : Error
==============================================================================*/
static SINT32	SendResetAllCtl(UINT8 bCh)
{
	MARMDCNV_DBGMSG(("SendResetAllCtl[%d] \n", bCh));

	gwRPN[bCh] = 0x7F7F;
	gbExpression[bCh] = 127;

	return (MaSndDrv_SetCommand(gSeqID, -1, MASNDDRV_CMD_RESET_ALL_CONTROLLERS, (UINT32)bCh, 0, 0));
}


/*==============================================================================
//	Function Name	: SINT32	SendMonoOn(UINT8 bCh, UINT8 bVal)
//
//	Description		: Send MonoOn message
//
//	Argument		: bCh			...	#Channel(0..15)
//
//	Return			: 0 : NoError, < 0 : Error
==============================================================================*/
static SINT32	SendMonoOn(UINT8 bCh, UINT8 bVal)
{
	MARMDCNV_DBGMSG(("SendMonoOn[%d] \n", bCh));

	if (bVal != 1) return (MA3RMDERR_NOERROR);

	return (MaSndDrv_SetCommand(gSeqID, -1, MASNDDRV_CMD_MONO_MODE_ON, (UINT32)bCh, 0, 0));
}


/*==============================================================================
//	Function Name	: SINT32	SendPolyOn(UINT8 bCh, UINT8 bVal)
//
//	Description		: Send PolyOn message
//
//	Argument		: bCh			...	#Channel(0..15)
//
//	Return			: 0 : NoError, < 0 : Error
==============================================================================*/
static SINT32	SendPolyOn(UINT8 bCh, UINT8 bVal)
{
	MARMDCNV_DBGMSG(("SendPolyOn[%d] \n", bCh));

	if (bVal != 0) return (MA3RMDERR_NOERROR);

	return (MaSndDrv_SetCommand(gSeqID, -1, MASNDDRV_CMD_POLY_MODE_ON, (UINT32)bCh, 0, 0));
}


/*==============================================================================
//	Function Name	: SINT32	SendControl(UINT8 bCh, UNIT8 bMsg2, UINT8 bMsg3)
//
//	Description		: Control messages
//
//	Argument		: bCh			...	#Channel(0..15)
//					  bMsg2			...	Param-1
//					  bMsg3			...	Param-2
//
//	Return			: 0 : NoError, < 0 : Error
==============================================================================*/
static SINT32	SendControl(UINT8 bCh, UINT8 bMsg2, UINT8 bMsg3)
{
	MARMDCNV_DBGMSG(("SendControl[%d, %d, %d] \n", bCh, bMsg2, bMsg3));

	switch (bMsg2)
	{
	case 0x00:	/* Bank select(MSB) */
		gwBank[bCh] = (UINT16)((gwBank[bCh] & 0x00FF) | ((UINT16)bMsg3 << 8));
		break;
    
    case 0x20:	/* Bank select (LSB) */
		gwBank[bCh] = (UINT16)((gwBank[bCh] & 0xFF00) | ((UINT16)bMsg3));
		break;

	case 0x01:
		return (SendModDepth(bCh, bMsg3));

	case 0x06:
		return (DataEntryMSB(bCh, bMsg3));

	case 0x07:
		return (SendChVol(bCh, bMsg3));

	case 0x0A:
		return (SendPanpot(bCh, bMsg3));

	case 0x0B:
		return (SendExpression(bCh, bMsg3));

	case 0x40:
		return (SendHold1(bCh, bMsg3));

	case 0x62:	/* NRPN (LSB) */
	case 0x63:	/* NRPN (MSB) */
		gwRPN[bCh] |= 0x8000;
		break;

	case 0x64:	/* RPN (LSB) */
		gwRPN[bCh] = (UINT16)((gwRPN[bCh] & 0x7F00) | ((UINT16)bMsg3));
		break;

	case 0x65:	/* RPN (MSB) */
		gwRPN[bCh] = (UINT16)((gwRPN[bCh] & 0x007F) | ((UINT16)bMsg3 << 8));
		break;

	case 0x78:
		return (SendAllSoundOff(bCh));

	case 0x7B:
		return (SendAllNoteOff(bCh));

	case 0x79:
		return (SendResetAllCtl(bCh));

	case 0x7e:
		SendAllNoteOff(bCh);
		return (SendMonoOn(bCh,bMsg3));
		
	case 0x7f:
		SendAllNoteOff(bCh);
		return (SendPolyOn(bCh, bMsg3));

	default:
		return (MA3RMDERR_NOERROR);
	}
	return (MA3RMDERR_NOERROR);
}


/*==============================================================================
//	Function Name	: SINT32	SendPitchBend(UINT8 bCh, UNIT8 bLl, UINT8 bHh)
//
//	Description		: Send PitchBend message
//
//	Argument		: bCh				...	#Channel(0..15)
//					  bLl				...	Lower 7bit of PitchBend (0..127)
//					  bHh				...	Upper 7bit of PitchBend (0..127)
//
//	Return			: 0 : NoError, < 0 : Error
==============================================================================*/
static SINT32	SendPitchBend(UINT8 bCh, UINT8 bLl, UINT8 bHh)
{
	MARMDCNV_DBGMSG(("SendPitchBend[%d] = %d\n", bCh, bHh));

	return (MaSndDrv_SetCommand(gSeqID, -1, MASNDDRV_CMD_PITCH_BEND, (UINT32)bCh, (((UINT32)bHh << 7) | ((UINT32)bLl)), 0));
}


/*==============================================================================
//	Function Name	: SINT32	SendGmOn(void)
//
//	Description		: Send GM_ON message
//
//	Argument		: none
//
//	Return			: 0:NoError　 <0:エラーコード
//
==============================================================================*/
static SINT32	SendGmOn(void)
{
	UINT8	bCh;
	
	MARMDCNV_DBGMSG(("SendGmOn\n"));

	gbMaxGain = MAX_MID_GAIN;
	gbMasterVolume = 127;
	for (bCh = 0; bCh < 16; bCh++)
	{
		SendAllSoundOff(bCh);
		gwRPN[bCh] = 0x7f7f;
		gwBank[bCh] = (bCh == 9) ? 0x7800 : 0x7900;
		gbChVolume[bCh] = 100;
		gbExpression[bCh] = 127;
	}

	return (MaSndDrv_SetCommand(gSeqID, -1, MASNDDRV_CMD_SYSTEM_ON, 0, 0, 0));
}


/*=============================================================================
//	Function Name	: SINT32	MaRmdCnv_Initialize(void)
//
//	Description		: Initialize RealtimeMIDI converter
//
//	Argument		: none
//
//	Return			: 0 : NoError, < 0 : Error
=============================================================================*/
SINT32	MaRmdCnv_Initialize(void)
{
	UINT8	i;

	MARMDCNV_DBGMSG(("MaRmdCnv_Initialize \n"));

	gSeqID = -1;						/* Sequence ID      */
	gFileID = -1;						/* File ID          */
	gEnable = 0;						/* 0:disabel        */
	gNumOfLoaded = 0;					/*                  */

	gbMaxGain = MAX_MID_GAIN;			/* MAX_MID_GAIN[dB] */
	gbMasterVolume = 127;

	gRamBase = 0;
	gRamOffset = 0;
	gRamSize = 0;

	for (i = 0; i < 128 ; i++) gWtWaveAdr[i] = 0;

	return (MA3RMDERR_NOERROR);
}


/*=============================================================================
//	Function Name	:	SINT32	MaRmdCnv_End(void)
//
//	Description		:	Ending of RealtimeMIDI converter
//
//	Argument		:	none
//
//	Return			: 0 : NoError, < 0 : Error
=============================================================================*/
SINT32	MaRmdCnv_End(void)
{
	MARMDCNV_DBGMSG(("MaRmdCnv_End \n"));

	return (MA3RMDERR_NOERROR);
}


/*=============================================================================
//	Function Name	: SINT32	MaRmdCnv_Load(UINT8* file_ptr, UINT32 file_size, UINT8 mode,
//												SINT32 (*func)(UINT8 id), void* ext_args)
//
//	Description		: Load MIDI settings
//
//	Argument		: file_ptr	... pointer to the data
//					  file_size	... size fo the data
//					  mode		... error check (0:No, 1:Yes, 2:Check)
//					  func		... pointer of rhe callback function
//					  ext_args	... NULL
//
//	Return			: 0 : NoError, < 0 : Error
=============================================================================*/
SINT32	MaRmdCnv_Load(UINT8* file_ptr, UINT32 file_size, UINT8 mode, SINT32 (*func)(UINT8 id), void* ext_args)
{
	(void)file_ptr;
	(void)file_size;
	(void)ext_args;

	MARMDCNV_DBGMSG(("MaRmdCnv_Load[%08lX %08lX %02X] \n", file_ptr, file_size, mode));

	if (mode > 1) return (MA3RMDERR_NOERROR);
	if (gNumOfLoaded > 0) return (MA3RMDERR_GENERAL);
	gNumOfLoaded++;
	gCallbackFunc = func;
	gFileID = gNumOfLoaded;
	gbIntervalTime = 0;
	
	return ((SINT32)gFileID);
}


/*=============================================================================
//	Function Name	: SINT32	MaRmdCnv_Unload(SINT32 file_id, void* ext_args)
//
//	Description		: Unload MIDI settings
//
//	Argument		: file_id		...	データfile ID
//					  ext_args		...	NULL
//
//	Return			: 0 : NoError, < 0 : Error
=============================================================================*/
SINT32	MaRmdCnv_Unload(SINT32 file_id, void* ext_args)
{
	(void)ext_args;

	MARMDCNV_DBGMSG(("MaRmdCnv_Unload[%d] \n", file_id));

	if ((file_id < 0) || (file_id != (SINT32)gFileID))
	{
		return (MA3RMDERR_GENERAL);
	}

	gNumOfLoaded = 0;
	gFileID = -1;

	return (MA3RMDERR_NOERROR);
}


/*=============================================================================
//	Function Name	: MaRmdCnv_Open(SINT32 file_id, UINT16 mode, void* ext_args)
//
//	Description		: Enable MIDI
//
//	Argument		: file_id		...	file ID
//					  mode			...	0..16
//					  ext_args		...	NULL
//
//	Return			: 0 : NoError, < 0 : Error
=============================================================================*/
SINT32	MaRmdCnv_Open(SINT32 file_id, UINT16 mode, void* ext_args)
{
	UINT32	Ram;
	UINT32	Size;
	(void)mode;
	(void)ext_args;
	
	MARMDCNV_DBGMSG(("MaRmdCnv_Open[%d] \n", file_id));

	if ((file_id < 0) || (file_id != gFileID))
	{
		return (MA3RMDERR_GENERAL);
	}

	gSeqID = MaSndDrv_Create(
			(UINT8)MASMW_SEQTYPE_DIRECT,
			gbIntervalTime,							/* No Timer             */
			(UINT8)(DRUM_GML1 +						/*                      */
			        DRUM_NORMAL +					/*                      */
			        MELODY_NORMAL +					/*                      */
			        DVA_SIMPLE +					/*                      */
			        RMD_DEF_FM_MODE),				/*                      */
			(UINT8)0,								/*                      */
			(UINT8)0,								/* Num of Streams       */
			MaRmdCnv_Convert,						/*                      */
			&Ram,									/*                      */
			&Size);									/*                      */

	MARMDCNV_DBGMSG(("MaRmdCnv_Open/SeqID=%ld \n", gSeqID));
	if (gSeqID < 0) return (MA3RMDERR_GENERAL);

	gRamBase = Ram;
	gRamSize = Size;
	gbMaxGain = MAX_MID_GAIN;
	gbMasterVolume = 127;
	gEnable = 1;

	SendMasterVolume(gbMaxGain);

	return (SendGmOn());
}


/*=============================================================================
//	Function Name	: SINT32	MaRmdCnv_Close(SINT32 file_id, void* ext_args)
//
//	Description		: Disable MIDI
//
//	Argument		: file_id		...	file ID
//					  ext_args		...	NULL
//
//	Return			: 0 : NoError, < 0 : Error
=============================================================================*/
SINT32	MaRmdCnv_Close(SINT32 file_id, void* ext_args)
{
	UINT8	ch;
	(void)ext_args;

	MARMDCNV_DBGMSG(("MaRmdCnv_Close[%d] \n", file_id));

	if ((file_id < 0) || (file_id != gFileID))
	{
		return (MA3RMDERR_GENERAL);
	}

	for (ch = 0; ch < 15; ch++)
	{
		MaSndDrv_SetCommand(gSeqID, -1, MASNDDRV_CMD_ALL_SOUND_OFF, ch, 0, 0);
	}

	MaSndDrv_Free(gSeqID);

	gSeqID = -1;
	gEnable = 0;

	return (MA3RMDERR_NOERROR);
}


/*=============================================================================
//	Function Name	: SINT32	MaRmdCnv_Standby(SINT32 file_id, void* ext_args)
//
//	Description		: Initialize MIDI Engine
//
//	Argument		: file_id		...	file ID
//					  ext_args		...	NULL
//
//	Return			: 0 : NoError, < 0 : Error
=============================================================================*/
SINT32	MaRmdCnv_Standby(SINT32 file_id, void* ext_args)
{
	(void)ext_args;

	MARMDCNV_DBGMSG(("MaRmdCnv_Standby[%d] \n", file_id));

	if ((file_id < 0) || (file_id != gFileID))
	{
		return (MA3RMDERR_GENERAL);
	}

	return (MA3RMDERR_NOERROR);
}


/*=============================================================================
//	Function Name	: SINT32	MaRmdCnv_Seek(SINT32 file_id, UINT32 pos, UINT8 flag, void* ext_args)
//
//	Description		: No effect
//
//	Argument		: file_id		...	file ID
//					  pos			...	Start point(msec)
//					  flag			...	Wait　0:No wait　1: 20[ms] wait
//					  ext_args		...	NULL
//
//	Return			: 0 : NoError, < 0 : Error
=============================================================================*/
SINT32	MaRmdCnv_Seek(SINT32 file_id, UINT32 pos, UINT8 flag, void* ext_args)
{
	(void)pos;
	(void)flag;
	(void)ext_args;

	MARMDCNV_DBGMSG(("MaRmdCnv_Seek[%d %ld] \n", file_id, pos));

	if ((file_id < 0) || (file_id != gFileID))
	{
		return (MA3RMDERR_GENERAL);
	}

	return (MA3RMDERR_NOERROR);
}


/*=============================================================================
//	Function Name	: SINT32	MaRmdCnv_Start(SINT32 file_id, void* ext_args)
//
//	Description		: No effect
//
//	Argument		: file_id		...	file ID
//					  ext_args	...	NULL
//
//	Return			: 0 : NoError, < 0 : Error
=============================================================================*/
SINT32	MaRmdCnv_Start(SINT32 file_id, void* ext_args)
{
	(void)ext_args;

	MARMDCNV_DBGMSG(("MaRmdCnv_Start[%d] \n", file_id));

	if ((file_id < 0) || (file_id != gFileID))
	{
		return (MA3RMDERR_GENERAL);
	}
	MaSndDrv_ControlSequencer(gSeqID, 1);

	return (MA3RMDERR_NOERROR);
}


/*=============================================================================
//	Function Name	: SINT32	MaRmdCnv_Stop(SINT32 file_id, void* ext_args)
//
//	Description		: Stop MIDI Play
//
//	Argument		: file_id		...	file ID
//					  ext_args		...	NULL
//
//	Return			: 0 : NoError, < 0 : Error
=============================================================================*/
SINT32	MaRmdCnv_Stop(SINT32 file_id, void* ext_args)
{
	UINT8	bCh;
	(void)ext_args;
	
	MARMDCNV_DBGMSG(("MaRmdCnv_Stop[%d] \n", file_id));

	if ((file_id < 0) || (file_id != gFileID))
	{
		return (MA3RMDERR_GENERAL);
	}

	MaSndDrv_ControlSequencer(gSeqID, 0);

	for (bCh = 0; bCh < 16; bCh++)
	{
		MaSndDrv_SetCommand(gSeqID, -1, MASNDDRV_CMD_ALL_SOUND_OFF, bCh, 0, 0);
	}
	
	return (MA3RMDERR_NOERROR);
}


/*=============================================================================
//	Function Name	:	SINT32	MaRmdCnv_SetShortMsg(SINT32 file_id, UINT32 dwMsg)
//
//	Description		:	Send MIDI short message
//
//	Argument		: file_id		...	file ID
//					  dwMsg			...	MIDI message
//
//	Return			: 0 : NoError, < 0 : Error
=============================================================================*/
static SINT32	MaRmdCnv_SetShortMsg(SINT32 file_id, UINT32 dwMsg)
{
	UINT8	bMsg1, bMsg2, bMsg3;
	UINT8	bCh;

	static SINT32 (*MidiMsg[16])(UINT8 bChannel, UINT8 bPara1, UINT8 bPara2) =
	{
		ErrorFunction,	ErrorFunction,	ErrorFunction,	ErrorFunction, 
		ErrorFunction, 	ErrorFunction,	ErrorFunction,	ErrorFunction, 
		SendNoteOff,	SendNoteOn,		NotSupported,	SendControl,
		SendProgram,	NotSupported,	SendPitchBend,	NotSupported
	};
	(void)file_id;

	MARMDCNV_DBGMSG(("MaRmdCnv_SetShortMsg[%08lX]\n", dwMsg));

	if (gEnable == 0) return (MA3RMDERR_NOERROR);

	bMsg1 = (UINT8)((dwMsg & 0xf0) >> 4);
	bCh = (UINT8)(dwMsg & 0x0f);
	bMsg2 = (UINT8)((dwMsg >> 8) & 0x7f);
	bMsg3 = (UINT8)((dwMsg >> 16) & 0x7f);

	return (MidiMsg[bMsg1](bCh, bMsg2, bMsg3));
}


/*=============================================================================*/
/*	Decode7Enc()                                                               */
/*                                                                             */
/*	Desc.                                                                      */
/*		7bit->8bit decoder                                                     */
/*	Param                                                                      */
/*		none                                                                   */
/*	Return                                                                     */
/*      size of decoded data                                                   */
/*=============================================================================*/
static UINT32 Decode7Enc(UINT8* pSrc, UINT32 sSize, UINT8* pDst, UINT32 dSize)
{
	UINT32	i, j;
	UINT8*	sEnd;
	UINT8	tag;
	
	i = 0;
	sEnd = pSrc + sSize;

	while (i < dSize)
	{
		tag = *(pSrc++);
		if (pSrc >= sEnd) break;
		
		for (j = 0; j < 7; j++)
		{
			tag <<= 1;
			pDst[i++] = (UINT8)((tag & 0x80) | *(pSrc++));
			if (pSrc >= sEnd) break;
			if (i >= dSize) return (i);
		}
	}
	return (i);
}


/*=============================================================================
//	Function Name	:	SINT32	MaRmdCnv_SetLongMsg(SINT32 file_id, UINT8* pMsg, UINT32 dwSize)
//
//	Description		:	Send MIDI Long message
//
//	Argument		: file_id		...	file ID
//					  pMsg			...	pointer to the SysEx meassage
//					  dwSize		...	Size of the message[byte]
//
//	Return			: 0 : NoError, < 0 : Error
=============================================================================*/
static SINT32	MaRmdCnv_SetLongMsg(SINT32 file_id, UINT8* pMsg, UINT32 dwSize)
{
	SINT32	dwRet;
	UINT8	bBk, bPg, bKey;
	UINT8	bBl, bPc, bFlg;
	UINT32	dwKey, dwAdr, dwRom;
	UINT32	dwWsize;
	UINT8	bWaveID;
	UINT8	bVoice[32];
	UINT8	bCh;
	
	(void)file_id;

	MARMDCNV_DBGMSG(("MaRmdCnv_SetLongMsg[0x%08lX, %ld] \n", pMsg, dwSize));

	dwRet = MA3RMDERR_NOERROR;
	if (gEnable == 0) return (MA3RMDERR_NOERROR);
	if (dwSize < 6) return (MA3RMDERR_NOERROR);
	if (pMsg[0] != 0xf0) return (MA3RMDERR_GENERAL);
	if (pMsg[dwSize - 1] != 0xf7) return (MA3RMDERR_GENERAL);
	
	if ((pMsg[1] == 0x7e) &&
	    (pMsg[2] == 0x7f) &&
	    (pMsg[3] == 0x09) &&
	    ((pMsg[4] == 0x01) || (pMsg[4] == 0x03)) &&
	    (dwSize == 6)) return (SendGmOn());
	
	if ((pMsg[1] == 0x7f) &&
	    (pMsg[2] == 0x7f) &&
	    (pMsg[3] == 0x04) &&
	    (pMsg[4] == 0x01) &&
	    (dwSize == 8))
	{
		gbMasterVolume = pMsg[6] & 0x7f;
		SendExpression(0, gbExpression[0]);
		for (bCh = 1; bCh < 16; bCh++) SendExpression(bCh, gbExpression[bCh]);
		return (MA3RMDERR_NOERROR);
	}
	
	if ((pMsg[1] != 0x43) ||
	    (pMsg[2] != 0x79) ||
	    (pMsg[3] != 0x06) ||
	    (pMsg[4] != 0x7f) ||
	    (dwSize < 8)) return (MA3RMDERR_NOERROR);

	switch (pMsg[5])
	{
	case 0x00:		/* Max Gain */
		if (dwSize != 8) return (MA3RMDERR_GENERAL);
		gbMaxGain = (UINT8)(pMsg[6] & 0x7f);
		SendMasterVolume(gbMaxGain);
		return (MA3RMDERR_NOERROR);
	
	case 0x10:		/* UserEvent */
		if (dwSize != 8) return (MA3RMDERR_GENERAL);
		if (gCallbackFunc == NULL) return (MA3RMDERR_GENERAL);
		return (gCallbackFunc((UINT8)(pMsg[6] & 0x0f)));
	
	case 0x01:		/* Set Voice params */
		if (dwSize < 31) return (MA3RMDERR_GENERAL);
		bBl = (UINT8)(pMsg[7] & 0x7f);
		bPc = (UINT8)(pMsg[8] & 0x7f);
		bKey = (UINT8)(pMsg[9] & 0x7f);
		bFlg = (UINT8)(pMsg[10] + 1);

		switch (pMsg[6])
		{
		case 0x7c:
			bBk = (UINT8)(1 + bBl);
			bPg = bPc;
			break;
		case 0x7d:
			bBk = (UINT8)(129 + bPc);
			bPg = bKey;
			break;
		default:
			return (MA3RMDERR_GENERAL);
		}

		dwAdr = gRamBase + gRamOffset;

		switch (dwSize)
		{
		case 31:	/* WT */
			if (bFlg != 2) return (MA3RMDERR_GENERAL);
			if ((gRamOffset + 13) > gRamSize) return (MA3RMDERR_NOERROR);
			Decode7Enc(&pMsg[11], 19, bVoice, 16);

			dwKey = ((UINT32)bVoice[0] << 8) | (UINT32)bVoice[1];
			bWaveID = bVoice[15];

			if (bWaveID < 128)
			{
				bVoice[9] = (UINT8)((gWtWaveAdr[bWaveID] >> 8) & 0xff);
				bVoice[10] = (UINT8)(gWtWaveAdr[bWaveID] & 0xff);
			} else {
				dwRom = MaResMgr_GetDefWaveAddress((UINT8)(bWaveID & 0x7f));
				if (dwRom <= 0) return (MA3RMDERR_GENERAL);
				bVoice[9] = (UINT8)((dwRom >> 8) & 0xff);
				bVoice[10] = (UINT8)(dwRom & 0xff);
			}
			MaDevDrv_SendDirectRamData(dwAdr, ENC_8BIT, &bVoice[2], 13);
			dwRet = MaSndDrv_SetVoice(gSeqID, bBk, bPg, bFlg, dwKey, dwAdr);
			gRamOffset += 14;
			break;

		case 32:	/* 2-OP FM */
			if (bFlg != 1) return (MA3RMDERR_GENERAL);
			if ((gRamOffset + 16) > gRamSize) return (MA3RMDERR_NOERROR);
			Decode7Enc(&pMsg[11], 20, bVoice, 17);
			dwKey = bVoice[0];
			MaDevDrv_SendDirectRamData(dwAdr, ENC_8BIT, &bVoice[1], 16);
			dwRet = MaSndDrv_SetVoice(gSeqID, bBk, bPg, bFlg, dwKey, dwAdr);
			gRamOffset += 16;
			break;

		case 48:	/* 4-OP FM */
			if (RMD_DEF_FM_MODE != FM_4OP_MODE) return (MA3RMDERR_GENERAL);
			if (bFlg != 1) return (MA3RMDERR_GENERAL);
			if ((gRamOffset + 31) > gRamSize) return (MA3RMDERR_NOERROR);
			Decode7Enc(&pMsg[11], 36, bVoice, 31);
			dwKey = bVoice[0];
			MaDevDrv_SendDirectRamData(dwAdr, ENC_8BIT, &bVoice[1], 30);
			dwRet = MaSndDrv_SetVoice(gSeqID, bBk, bPg, bFlg, dwKey, dwAdr);
			gRamOffset += 30;
			break;
		
		default:
			return (MA3RMDERR_GENERAL);
		}
		break;
		
	case 0x03:		/* Set Wave */
		dwAdr = gRamBase + gRamOffset;
		if (dwSize < 10) return (MA3RMDERR_GENERAL);
		if (pMsg[dwSize - 1] != 0xf7) return (MA3RMDERR_GENERAL);
		dwWsize = dwSize - 9;
		dwWsize = (dwWsize >> 3) * 7 + (((dwWsize & 0x07) == 0) ? 0 : ((dwWsize & 0x07)-1));
		if ((gRamOffset + dwWsize) > gRamSize) return (MA3RMDERR_NOERROR);
		bPc = (UINT8)(pMsg[6] & 0x7f);
		bFlg = (UINT8)(pMsg[7] & 0x7f);
		dwRet = MaDevDrv_SendDirectRamData(dwAdr, ENC_7BIT, &(pMsg[8]), dwSize - 9);
		gRamOffset += ((dwWsize + 1) >> 1) << 1;
		gWtWaveAdr[bPc] = dwAdr;
		break;
		
	default:
		return (MA3RMDERR_NOERROR);
	}

	return (dwRet);
}


/*=============================================================================
//	Function Name	: SINT32	MaRmdCnv_Control(SINT32 file_id, UINT8 ctrl_num, void* prm, void* ext_args)
//
//	Description		: Special commands
//
//	Argument		: file_id		...	file ID
//					  ctrl_num		...	Command ID
//					  prm			...	param for the command
//
//	Return			: 0 : NoError, < 0 : Error
=============================================================================*/
SINT32	MaRmdCnv_Control(SINT32 file_id, UINT8 ctrl_num, void* prm, void* ext_args)
{
	(void)ext_args;

	MARMDCNV_DBGMSG(("MaRmdCnv_Control[%d, %d] \n", file_id, ctrl_num));

	if ((file_id < 0) || (file_id != gFileID))
	{
		return (MA3RMDERR_GENERAL);
	}

	switch(ctrl_num)
	{
	case MASMW_SET_VOLUME :
		return (MaSndDrv_SetVolume(gSeqID, *((UINT8*)prm)));
		
	case MASMW_SEND_MIDIMSG :
		return (MaRmdCnv_SetShortMsg(file_id, *((UINT32*)prm)));
		
	case MASMW_SEND_SYSEXMIDIMSG :
		return (MaRmdCnv_SetLongMsg(file_id, ((PMASMW_MIDIMSG)prm)->pbMsg, ((PMASMW_MIDIMSG)prm)->dSize));
		
	case MASMW_GET_LENGTH :
		return (1000);			/* dummy length (1sec)*/

	case MASMW_SET_REPEAT :
		return (MASMW_SUCCESS);			/* dummy func */

	case MASMW_GET_CONTROL_VAL:
		switch (((PMASMW_GETCTL)prm)->bControl)
		{
		case 0:	/* Master Volume */
			return (gbMasterVolume);
			
		case 1:	/* Channel Volume */
			if (((PMASMW_GETCTL)prm)->bCh >= 16) return (MASMW_ERROR);
			return (gbChVolume[((PMASMW_GETCTL)prm)->bCh]);
		}
		return (MASMW_ERROR);

	case MASMW_SET_CB_INTERVAL: 
		if ((*(UINT8*)prm != 0) && (*(UINT8*)prm != 20)) return (MASMW_ERROR);
		gbIntervalTime = *(UINT8*)prm;
		return (MASMW_SUCCESS);
	}

	return (MA3RMDERR_NOERROR);
}


/*=============================================================================
//	Function Name	: SINT32	MaRmdCnv_Convert(void)
//
//	Description		: No effect
//
//	Argument		: none
//
//	Return			: 1 : Continue
=============================================================================*/
SINT32	MaRmdCnv_Convert(void)
{
	if ((gbIntervalTime > 0) && (gCallbackFunc != NULL))
	{
		gCallbackFunc(RMD_INTERVAL_CB);
	}
	return (1);
}

