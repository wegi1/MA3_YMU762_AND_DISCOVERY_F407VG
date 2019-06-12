/*******************************************************************************
        MAMMFCNV.H
        Header file for SMAF for MA-1/MA-2/MA-3 sequencer modele

        Version        : 1.3.5.0    2002.05.29
        Copyright (C) 2001-2002 YAMAHA CORPORATION
*******************************************************************************/

#ifndef _MAMMFCNV_H
#define _MAMMFCNV_H

#define	    rightbits(n,x)	((x) & ((1 << (n)) - 1))

/*--- Definition of Static Value ---------------------------------------------*/

/* Error Status for Return Value                                              */
#define MMFCNV_ERR_ARGUMENT         MASMW_ERROR_ARGUMENT        /* -2         */
#define MMFCNV_ERR_RESOUCE          MASMW_ERROR_RESOURCE_OVER   /* -3         */
#define MMFCNV_ERR_ID               MASMW_ERROR_ID              /* -4         */
#define MMFCNV_ERR_FILE             MASMW_ERROR_FILE            /* -16        */
#define MMFCNV_ERR_C_CLASS          MASMW_ERROR_CONTENTS_CLASS  /* -17        */
#define MMFCNV_ERR_C_TYPE           MASMW_ERROR_CONTENTS_TYPE   /* -18        */
#define MMFCNV_ERR_SIZE             MASMW_ERROR_CHUNK_SIZE      /* -19        */
#define MMFCNV_ERR_CHUNK            MASMW_ERROR_CHUNK           /* -20        */
#define MMFCNV_ERR_TAG              MASMW_ERROR_UNMATCHED_TAG   /* -21        */
#define MMFCNV_ERR_LENGTH           MASMW_ERROR_SHORT_LENGTH    /* -22        */

/* Error Status for SMAF check */
#define SMAF_ERR_TYPE               0x01
#define SMAF_ERR_CHUNK              0x02
#define SMAF_ERR_SHORT_LENGTH       0x04

/* Error Status for inner function                                            */
#define F_TRUE                      MASMW_SUCCESS               /* 0          */
#define F_FALSE                     MASMW_ERROR                 /* -1         */

/* SMAF Load Mode Flag                                                        */
#define SMAF_CHECK                  0x01
#define SMAF_REGIST                 0x02
#define SMAF_CONTROL                0x04

/* SMAF Format Type */
#define SMAF_TYPE_MA1               0x01
#define SMAF_TYPE_MA2               0x02
#define SMAF_TYPE_MA3               0x04

/* SMAF Contents Class & Type */
#define CONTENTS_CLASS_1            0x00        /* Contents Class 0x00        */
#define CONTENTS_CLASS_2            0x20        /* Contents Class 0x20        */
#define CONTENTS_TYPE_1             0x00        /* Contetns Type 0x00 - 0x0F  */
#define CONTENTS_TYPE_2             0x10        /* Contents Type 0x10 - 0x1F  */
#define CONTENTS_TYPE_3             0x10        /* Contents Type 0x20 - 0x2F  */
#define CONTENTS_TYPE_4             0x30        /* Contents Type 0x30 - 0x4F  */
#define CONTENTS_TYPE_5             0x40        /* Contents Type 0x40 - 0x3F  */
#define CONTENTS_TYPE_6             0x40        /* Contents Type 0x50 - 0x5F  */

/* Evnet Flag */
#define EVENT_SYSTEM_ON             0x0001      /* System ON Event            */
#define EVENT_MASTER_VOLUME         0x0002      /* Master Volume Control      */
#define EVENT_MONO_MODE_ON          0x0004      /* Mono Mode ON Event         */
#define EVENT_MASTER_VOL            0x0020      /* Master Volume Control      */
#define EVENT_ENDTIME               0x0040      /* End Time Control           */
#define EVENT_BANK_9                0x0080      /* Program Change Channel 9   */
#define EVENT_LED_ON                0x0100      /* LED ON                     */
#define EVENT_MOTOR_ON              0x0200      /* Motor ON                   */
#define EVENT_LED_OFF               0x0400      /* LED OFF                    */
#define EVENT_MOTOR_OFF             0x0800      /* Moter OFF                  */
#define EVENT_EOS                   0x1000      /* End of Sequence            */

/*    Contents Code Type    */
#define CONTENTS_CODE_SJIS          0x00
#define CONTENTS_CODE_LATIN1        0x01
#define CONTENTS_CODE_UTF8          0x23
#define CONTENTS_CODE_BINARY        0xFF


/* Phrase Number */
#define PHRASE_PA                   1
#define PHRASE_PB                   2
#define PHRASE_PE                   3
#define PHRASE_PI                   4
#define PHRASE_PK                   5
#define PHRASE_PS                   6
#define PHRASE_PR                   7

/* Sound Driver Create Mode */
#define STANDARD                    2
#define HIGH_QUALITY                0
#define CREATE_MODE                 STANDARD

/* Wave Format Type */
#define MMF_WAVE_BIN_2S_COMP        0x00
#define MMF_WAVE_BIN_OFFSET         0x01
#define MMF_WAVE_ADPCM              0x02
#define MMF_WAVE_4BIT               0x00
#define MMF_WAVE_8BIT               0x01
#define MMF_WAVE_SF_MINIMUM         4000
#define MMF_WAVE_SF_4BIT_MAX        24000
#define MMF_WAVE_SF_8BIT_MAX        12000

/* Channel Type */
#define MMF_CH_TYPE_NORMAL          0x01
#define MMF_CH_TYPE_DRUM            0x02
#define MMF_CH_TYPE_STREAM          0x04
#define MMF_CH_TYPE_MA2             0x08


/* Others                                                                     */
#define E_C_NORMAL                  1           /* Error Check Strength High  */
#define E_C_LOOSE                   0           /* Error Check Strength Low   */
#define ERROR_CHECK_STRENGTH        E_C_NORMAL
#define POSITION_OF_CONTENTSTYPE    18
#define SIZE_OF_CHUNKHEADER         8
#define SIZE_OF_CRC                 2
#define MMF_FS_BASE                 0x576
#define PLAY_TIME_MIN               20          /* Minimum Play Time (MA-3)   */
#define CRC_INITIAL                 0xF0000000

#define EVLIST_SIZE                 10
#define OFFLIST_SIZE                64
#define SEEK_END_FLAG               0xFFFF
#define AUDIO_TRACK_NO              0x10

#define MMF_EVENT_STREAM_ON_MA2     0xFD
#define MMF_EVENT_STREAM_OFF_MA2    0xFE
#define MMF_EVENT_EOS               0xFF
#define DOCODE_SIZE_MINIMUM         9
#define DOCODE_FAILURE              0xFFFFFFFF

/* Default Value */
#define MMF_Bank_No                 0x00
#define MMF_Bank_Select_MSB         0x00
#define MMF_Bank_Selsct_LSB         0x00
#define MMF_Program_No              0x00
#define MMF_RPN_MSB                 0x7F
#define MMF_RPN_LSB                 0x7F
#define MMF_Modulation_Depth        0x00
#define MMF_Channel_Volume          0x64
#define MMF_Channel_Panpot          0x40
#define MMF_Expression              0x7F
#define MMF_Dumper_Hold             0x00
#define MMF_Pitch_Bend_Sensitivity  0x02
#define MMF_Channel_Mode            0x00
#define MMF_Pitch_Bend_MSB          0x40
#define MMF_Pitch_Bend_LSB          0x00
#define MMF_Master_Volume3          0x2D
/*#define MMF_Wave_Panpot             0x40 */
#define MMF_Wave_Panpot             0xFF


/*--- Structure --------------------------------------------------------------*/

typedef struct Phrase{
    UINT32 dwSTp;                               /* Data Offset Start Point    */
    UINT32 dwSPp;                               /* Data Offset Stop Point     */
    UINT32 dwSTt;                               /* Start Time(tick)           */
    UINT32 dwSPt;                               /* Stop Time(tick)            */
} PHRA, *PPHRA;


typedef struct AUDIO_TRACK_INFO{
    UINT8  *pATR;                               /* ATR0 Info                  */
    UINT32 dwSize;                              /* ATR0 Size                  */
    UINT8  *pPhrase;                            /* AspI Info                  */
    UINT32 dwPhraseSize;                        /* AspI Size                  */
    UINT8  *pSeq;                               /* Atsq Info                  */
    UINT32 dwSeqSize;                           /* Atsq Size                  */
    UINT32 dwPlayTime;                          /* Play Time(tick)            */
    UINT8  bTimeBase;                           /* Time Base                  */
    PHRA   Phrase;                              /* Phrase Information         */
} ATRINFO, *PATRINFO;


typedef struct SCORE_TRACK_INFO{
    UINT8  *pMTR;                               /* MTR* Info                  */
    UINT32 dwSize;                              /* MTR* Size                  */
    UINT8  *pPhrase;                            /* MspI Info                  */
    UINT32 dwPhraseSize;                        /* MspI Size                  */
    UINT8  *pSetup;                             /* Mtsu Info                  */
    UINT32 dwSetupSize;                         /* Mtsu Size                  */
    UINT8  *pSeq;                               /* Mtsq Info                  */
    UINT32 dwSeqSize;                           /* Mtsq Size                  */
    UINT8  *pWave;                              /* Mtsp Info                  */
    UINT32 dwWaveSize;                          /* Mtsp Size                  */
    UINT32 dwPlayTime;                          /* Play Time(tick)            */
    UINT8  bTimeBase;                           /* Time Base                  */
    PHRA   Phrase[8];                           /* Phrase Information         */
} MTRINFO, *PMTRINFO;


typedef struct SMAF_INFO{
    UINT8  *pFile;                              /* File Pointer               */
    UINT32 dwSize;                              /* File Size                  */
    UINT32 dwMMMDSize;                          /* SMAF Data Size             */
    UINT32 dwCRC;                               /* CRC                        */
    UINT8  bSmafType;                           /* SMAF Type                  */
    UINT8  bMaxCHNum;                           /* Number of Channel          */
    UINT8  *pOPDA;                              /* Option Info                */
    UINT32 dwOPDASize;                          /* Option Info Size           */
    UINT8  bTimeBase;                           /* Time Base                  */
    UINT32 dwStartTime;                         /* Start Point(tick)          */
    UINT32 dwPlayTime;                          /* Play Time(tick)            */
    MTRINFO ScoreTrack[6];                      /* MTR* Info                  */
    ATRINFO AudioTrack;                         /* ATR0 Info                  */
} SMAFINFO, *PSMAFINFO;


typedef struct WAVE_IF2{
    UINT32 dwSR;                                /* Sample Rate(Hz)            */
    UINT8  *pWave;                              /* Wave Data                  */
    UINT32 dwWaveSize;                          /* Wave Data Size             */
    UINT8  bKeyNo;                              /* Key #                      */
} WAVEINFO2, *PWAVE2;


typedef struct WAVE_IF3{
    UINT8  bNote;                               /* Note ON/OFF Flag           */
    UINT8  bPair;                               /* Pair ID                    */
    UINT8  bPan;                                /* Wave PanPot                */
} WAVEINFO3, *PWAVE3;


typedef struct VOICE_IF{
    UINT8  bBank;                               /* Bank No.                   */
    UINT8  bProg;                               /* Program No.                */
} VOICEINFO, *PVOICE;


typedef struct NOTE_OFF_LIST{
    UINT32 dwTime;                              /* Note OFF Time              */
    UINT32 dwKey;                               /* Key No.                    */
    UINT32 dwCh;                                /* Channel No.                */
    UINT32 dwType;                              /* Voice Type                 */
    void   *pNext;                              /* Next Node                  */
} OFFLIST, *POFFLIST;


typedef struct EVENT_LIST{
    UINT32 dwTime;                              /* Action Time                */
    UINT32 dwEvNo;                              /* Event No.                  */
    UINT32 dwVal1;                              /* Argument 1                 */
    UINT32 dwVal2;                              /* Argument 2                 */
    UINT32 dwVal3;                              /* Argument 3                 */
    UINT32 dwSize;
    UINT8  bTrackNo;                            /* Track No.                  */
    void   *pNext;                              /* Next Node                  */
} EVLIST, *PEVLIST;


typedef struct PLAY_INFO{
    void   *pNextEv;                            /* Next Event                 */
    void   *pEmptyEv;                           /* Empty Event List           */
    void   *pNextOFF;                           /* Next Note OFF              */
    void   *pEmptyOFF;                          /* Empty Note OFF List        */
    UINT32 dwPastTime;                          /* Past Time(tick)            */
    UINT16 wPitchBendFlag;                      /* Pitch Bend ON/OFF Flag     */
    UINT16 wNoteFlag;                           /* Note Flag                  */
    UINT8  bEOS;                                /* EOS Flag                   */
    UINT8  bPreEvent;                           /* Seek Event Counter         */
    UINT16 wLED;                                /* LED   ON/OFF Flag          */
    UINT16 wMOTOR;                              /* Motor ON/OFF Flag          */
    UINT8  bTimeBaseR;                          /* Tick -> HwTime Base        */
    UINT8  bStream;
} PLAYINFO, *PPLAYINFO;


typedef struct CH_INFO3 {
    UINT8  bType;                               /* ChType(Normal,Dram,Stream) */
    UINT8  bSlotNum;
    UINT8  bVel;                                /* Velocity                   */
    UINT8  bBank;                               /* Bank No.                   */
    UINT8  bBankM;                              /* Bank Select MSB            */
    UINT8  bBankL;                              /* Bank Select LSB            */
    UINT8  bPgm;                                /* Program Change             */
    UINT8  bRpnM;                               /* RPN MSB                    */
    UINT8  bRpnL;                               /* RPN LSB                    */
    UINT8  bModulation;                         /* Modulation Depth           */
    UINT8  bVolume;                             /* Channel Volume             */
    UINT8  bPanpot;                             /* Channel Panpot             */
    UINT8  bExpression;                         /* Expression                 */
    UINT8  bHold;                               /* Dumper Hold                */
    UINT8  bPitchSens;                          /* Pitch Bend Sensitivity     */
    UINT8  bMonoPoly;                           /* Channel Mode (Mono/Poly)   */
    UINT8  bPitchM;                             /* Pitch Bend MSB             */
    UINT8  bPitchL;                             /* Pitch Bend LSB             */
    UINT8  bPitchS;                             /* Pitch Bend Sensitivity     */
    UINT8  bC_Volume;                           /* Channel Volume             */
    UINT8  bLed;                                /* LED Flag                   */
    UINT8  bMotor;                              /* Motor Flag                 */
} MMF_CHINFO, *MMF_PCHINFO;



SINT32 MaMmfCnv_Initialize( void );
SINT32 MaMmfCnv_End( void );
SINT32 MaMmfCnv_Load(UINT8 *file_ptr, UINT32 file_size, UINT8 mode,
            SINT32 (*func)(UINT8 id), void * ext_args);
SINT32 MaMmfCnv_Unload(SINT32 file_id, void * ext_args);
SINT32 MaMmfCnv_Open(SINT32 file_id, UINT16 open_mode, void *ext_args);
SINT32 MaMmfCnv_Standby(SINT32 file_id, void *ext_args);
SINT32 MaMmfCnv_Close(SINT32 file_id, void *ext_args);
SINT32 MaMmfCnv_Seek(SINT32 file_id, UINT32 pos, UINT8 flag, void *ext_args);
SINT32 MaMmfCnv_Start(SINT32 file_id, void *ext_args);
SINT32 MaMmfCnv_Stop(SINT32 file_id, void *ext_args);
SINT32 MaMmfCnv_Control(SINT32 file_id, UINT8 ctrl_num, void *prm,
            void *ext_args);
SINT32 MaMmfCnv_Convert( void );

#endif
