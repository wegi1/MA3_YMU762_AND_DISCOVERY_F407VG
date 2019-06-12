/*******************************************************************************
        MAMMFCNV.C
        MA SMAF converter module

        Version        : 1.3.7.5    2003.04.08
        Copyright (C) 2001-2003 YAMAHA CORPORATION
*******************************************************************************/

#include "mamachdep.h"
#include "mammfcnv.h"
#include "madefs.h"
#include "masnddrv.h"
#include "maresmgr.h"



static SINT32       MmfSeqID;                   /* SMAF Converter ID          */
static UINT32       dwStatus;                   /* Converter Status           */

static SMAFINFO     Smaf_Info[2];               /* SMAF Information           */
static WAVEINFO2    Wave_Info2[62];             /* Wave Info (for MA-2)       */
static WAVEINFO3    Wave_Info3[32];             /* Wave Info (for MA-3)       */
static VOICEINFO    Voice_Info[16];             /* Voice Info (for MA-1/2)    */
static MMF_CHINFO   ChInfo[16];                 /* Channel Information        */

static PLAYINFO     Play_Info;                  /* Play Infomation            */
static EVLIST       Event_List[EVLIST_SIZE];    /* Evnet Information List     */
static OFFLIST      Off_List[OFFLIST_SIZE];     /* Note Off List              */

static UINT32       dwSeekTime;                 /* Seek Time (msec)           */
static UINT32       dwSeekTick;                 /* Seek Time (tick)           */
static UINT32       dwEndTime;                  /* End  Time (msec)           */
static UINT32       dwEndTick;                  /* End  Time (tick)           */
static UINT8        bMasterVol1;                /* Master Volume (seq data)   */
static UINT8        bMasterVol2;                /* Master Volume (ctrl value) */
static UINT8        bCompress[2];               /* Compressed Mode Flag       */

static UINT32       dwEventFlag;                /* Event Flag                 */
static UINT32       dwRamAdr;                   /* Ram Address                */
static UINT32       dwRamSize;                  /* Ram Size                   */

/* Audio Track Information                                                    */
static UINT8        bAudio_LED;                 /* Audio Track LED/MOTOR Flag */
static UINT8        bAudio_Vel;                 /* Audio Velocity             */
static UINT8        bAudio_Slot;                /* Audio Slot                 */

/* Huffman Decode                                                             */
static SINT16       Decode_left[2][512];        /* Huffman Table(Left Tree)   */
static SINT16       Decode_right[2][512];       /* Huffman Table(Right Tree)  */
static UINT8        *pDecodeBuffer[2];          /* Reference Pointer(Byte)    */
static SINT32       Decode_count[2];            /* Reference Pointer(bit)     */
static SINT32       Decode_bitbuf[2];           /* bit Buffer                 */
static UINT8        bBitOffset[2];              /* bit offset                 */
static UINT8        bFirstByte[2];              /* First Byte                 */

static UINT8        *pSeq[AUDIO_TRACK_NO+1];    /* Pointer to Sequence Data   */
static UINT32       dwSeqSize[AUDIO_TRACK_NO+1];/* Size of Sequence Data      */

static UINT32		dwTimeError;				/* Start Time Error			  */

const UINT16 crctable[256] = {
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

const UINT8  modetable[17] = {
    0,  1,  4,  4,  4,  8,  8,  8,  8, 12, 12, 12, 12, 15, 15, 15, 16 };

static const UINT8  VelocityTable[128] = 
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

/* voice parameter table for MA-1 */
static const UINT8 TableA[16] = {
    0, 1, 1, 1, 1, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 15 };

static const UINT8 TableD[16] = {
    0, 1, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14 };

const UINT8 _exp2[16] = {
0x80,0x00,0x1f,0x27,0x2f,0x37,0x3f,0x47,0x4f,0x57,0x5f,0x67,0x6f,0x77,0x7f,0x80
};

const UINT8 _mod2[16] = {
0x80,0x00,0x08,0x10,0x18,0x20,0x28,0x30,0x38,0x40,0x48,0x50,0x60,0x70,0x7f,0x80
};

const SINT32 bit_mask[8] = {0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x80};

static SINT32 Set_NoteOffEvent( void );

/*--- Common A    Function ---------------------------------------------------*/

/*******************************************************************************
    Name        : _get4b
    Description : Get 4Bytes From Buffer
    Parameter   : UINT8 **pBuf ... pointer to Buffer
    Return      : UINT32       ... Got Value
*******************************************************************************/
static UINT32 _get4b(UINT8 **pBuf)
{
    UINT32 d32;

    d32 = ((UINT32)(**pBuf)<<24) | ((UINT32)*((*pBuf)+1)<<16) |
          ((UINT32)*((*pBuf)+2)<<8) | (UINT32)*((*pBuf)+3);
    *pBuf += 4;

    return d32;
}


/*******************************************************************************
    Name        : makeCRC
    Description : Calculation CRC Value
    Parameter   : UINT32 n  ... File Size(Byte)
                : UINT8  *c ... Pointer to Buffer
    Return      : UINT16    ... CRC
*******************************************************************************/
static UINT16 makeCRC(
    UINT32 n,
    UINT8 *c
)
{
    UINT16 res;
    UINT16 crc;
    UINT8  data0;

    res  = 0xFFFFU;
    crc  = 0;

    while( --n >= 2 ){
        data0 = *c++;
        res = (UINT16)((res << 8) ^ crctable[(UINT8)(res >> 8) ^ data0]);
        crc = (UINT16)((crc << 8) | data0);
    }

    return (UINT16)(~res & 0xFFFFU);
}


/*******************************************************************************
    Name        : NextChunk
    Description : Judgment Next Chunk ID and Size
    Parameter   : UINT8  state  ... Analysis State
                : UINT8  *pBuff ... Pointer to Data Buffer
                : UINT32 remain ... Remain Data Size
                : UINT16 *chunk ... Pointer to Chunk ID (for write)
    Return      : >=0    ... Success (Data Size)
                : < 0    ... Error (Status or Data Size)
*******************************************************************************/
static SINT32 NextChunk(
    UINT8  state,
    UINT8  *pBuff,
    UINT32 remain,
    UINT16 *chunk
)
{
    UINT32 dwChunkID;
    UINT32 dwChunkSize;

    if(remain <= 8)
        return -1;

    dwChunkID = _get4b(&pBuff);                 /* Get Chunk ID               */
    remain -= 4;
    switch(dwChunkID){
        case 0x4D4D4D44:                        /* File Chunk                 */
            *chunk = 0x0001;
            if(state != 0)                      /* status Check               */
                return -1;
            break;
        case 0x434E5449:                        /* Contents Info Chunk        */
            *chunk = 0x0002;
            if(state != 1)                      /* status Check               */
                return -1;
            break;
        case 0x4F504441:                        /* Optional Data Chunk        */
            *chunk = 0x0003;
            if(state != 2)                      /* status Check               */
                return -1;
            break;
        case 0x4D535452:                        /* Master Track Chunk         */
            *chunk = 0x0007;
            if(state != 2)                      /* status Check               */
                return -1;
            break;
        case 0x4D737049:                        /* Seek&Phrase Info Chunk(MTR)*/
            *chunk = 0x0009;
            if(state != 3)                      /* status Check               */
                return -1;
            break;
        case 0x4D747375:                        /* Setup Data Chunk(MTR)      */
            *chunk = 0x000A;
            if(state != 3)                      /* status Check               */
                return -1;
            break;
        case 0x4D747371:                        /* Sequense Data Chunk(MTR)   */
            *chunk = 0x000B;
            if(state != 3)                      /* status Check               */
                return -1;
            break;
        case 0x4D747370:                        /* Stream PCM Wave Data Chunk */
            *chunk = 0x000C;
            if(state != 3)                      /* status Check               */
                return -1;
            break;
        case 0x41737049:                        /* Seek&Phrase Info Chunk(ATR)*/
            *chunk = 0x000E;
            if(state != 6)                      /* status Check               */
                return -1;
            break;
        case 0x41747375:                        /* Setup Data Chunk(ATR)      */
            *chunk = 0x000F;
            if(state != 6)                      /* status Check               */
                return -1;
            break;
        case 0x41747371:                        /* Sequense Data Chunk(ATR)   */
            *chunk = 0x0010;
            if(state != 6)                      /* status Check               */
                return -1;
            break;
        default:
            *chunk = (UINT16)((dwChunkID & 0x000000FF) << 8);
            switch(dwChunkID & 0xFFFFFF00){
                case 0x4D545200:                /* Score Track Chunk          */
                    *chunk |= 0x0004;
                    if(state != 2)              /* status Check               */
                        return -1;
                    break;
                case 0x41545200:                /* Audio Track Chunk          */
                    *chunk |= 0x0005;
                    if(state != 2)              /* status Check               */
                        return -1;
                    break;
                case 0x47545200:                /* Graphics Data Chunk        */
                    *chunk |= 0x0006;
                    if(state != 2)              /* status Check               */
                        return -1;
                    break;
                case 0x44636800:                /* Data Chunk(OPDA)           */
                    *chunk |= 0x0008;
                    if(state != 5)              /* status Check               */
                        return -1;
                    break;
                case 0x4D776100:                /* Wave Data Chunk(MTR)       */
                    *chunk |= 0x000D;
                    if(state != 4)              /* status Check               */
                        return -1;
                    break;
                case 0x41776100:                /* Wave Data Chunk(ATR)       */
                    *chunk |= 0x0011;
                    if(state != 6)              /* status Check               */
                        return -1;
                    break;
                default:                        /* Unknown Chunk              */
                    *chunk = 0xFFFF;
                    break;
            }
    }
    dwChunkSize = _get4b(&pBuff);               /* Get Chunk Size             */
    remain -= 4;
    if(dwChunkSize > remain)                    /* Check Chunk Size           */
        return -1;
    return (SINT32)dwChunkSize;
}


/*******************************************************************************
    Name        : Check_Initial
    Description : Initial Process
    Parameter   : struct Check ... Pointer to Information Structure
    Return      : Nothing
*******************************************************************************/
static void Check_Initial(
    PSMAFINFO Check
)
{
    UINT8  i;

    Check->pFile  = 0;
    Check->dwSize = 0;
    Check->dwCRC  = CRC_INITIAL;
    Check->bTimeBase  = 0;
    Check->dwPlayTime = 0;
    for (i = 0; i < 6; i++) {
        Check->ScoreTrack[i].pMTR         = 0;
        Check->ScoreTrack[i].dwSize       = 0;
        Check->ScoreTrack[i].pPhrase      = 0;
        Check->ScoreTrack[i].dwPhraseSize = 0;
        Check->ScoreTrack[i].pSetup       = 0;
        Check->ScoreTrack[i].dwSetupSize  = 0;
        Check->ScoreTrack[i].pSeq         = 0;
        Check->ScoreTrack[i].dwSeqSize    = 0;
    }
    Check->AudioTrack.pATR      = 0;
    Check->AudioTrack.dwSize    = 0;
    Check->AudioTrack.pSeq      = 0;
    Check->AudioTrack.dwSeqSize = 0;
}


/*******************************************************************************
    Name        : _get_timebase
    Description : Convert data in MTR to TimeBase
    Parameter   : UINT8 index ... data in MTR
    Return      : UINT8       ... TimeBase(?msec/1tick)
*******************************************************************************/
static UINT32 _get_timebase(
    UINT8 index
)
{
    switch( index ) {
        case 0x00: return  0;
        case 0x01: return  0;
        case 0x02: return  4;
        case 0x03: return  5;
        case 0x10: return 10;
        case 0x11: return 20;
        case 0x12: return 40;
        case 0x13: return 50;
        default:   return  0;                   /* ERROR                      */
    }
}


/*******************************************************************************
    Name        : MTR_Check
    Description : Score Track Chunk Check
    Parameter   : struct *MTR  ... Pointer to Score Track Chunk Information
                : UINT8  bType ... SMAF Type
    Return      :  >0 ... Error
                :   0 ... Success
*******************************************************************************/
static UINT8 MTR_Check(
    PMTRINFO MTR,
    UINT8  bType
)
{
    UINT32 index;
    UINT32 result;
    UINT32 dwSize;
    UINT8  *pBuf;
    UINT16 wNextID;
    SINT32 slNextSize;

    MTR->pPhrase = 0;
    MTR->pSetup  = 0;
    MTR->pSeq    = 0;
    MTR->pWave   = 0;

    if (bType == SMAF_TYPE_MA3) {
        if ((MTR->pMTR[0] != 0x01) && (MTR->pMTR[0] != 0x02)) {
            return SMAF_ERR_CHUNK;              /* MA-3   Format Type Error!! */
        }
    }
    else {
        if (MTR->pMTR[0] != 0x00) {
            return SMAF_ERR_CHUNK;              /* MA-1/2 Foramt Type Error!! */
        }
    }
    if (MTR->pMTR[1] != 0) {
        return SMAF_ERR_CHUNK;                  /* Sequense Type Error!!      */
    }
    if (MTR->pMTR[2] != MTR->pMTR[3]) {
        return SMAF_ERR_CHUNK;                  /* Time Base Error!!          */
    }
    result = (UINT32)_get_timebase(MTR->pMTR[2]);
    if (result == 0) {
        return SMAF_ERR_CHUNK;                  /* Time Base Error!!          */
    }
    MTR->bTimeBase = (UINT8)result;

    if (bType == SMAF_TYPE_MA3)
        index = 20;
    else
        index = 6;

    dwSize = MTR->dwSize;
    pBuf   = MTR->pMTR;
    while ((dwSize - index) > SIZE_OF_CHUNKHEADER) {
        slNextSize = NextChunk( 3, &pBuf[index], (dwSize - index), &wNextID);
        if (slNextSize < 0) {
            return SMAF_ERR_CHUNK;
        }
        index += SIZE_OF_CHUNKHEADER;
        switch((UINT8)(wNextID & 0x00FF)){
            case 0x09:                          /* Seek & Phrase Info Chunk   */
                MTR->pPhrase      = &pBuf[index];
                MTR->dwPhraseSize = (UINT32)slNextSize;
                break;
            case 0x0A:                          /* Setup Data Chunk           */
                MTR->pSetup       = &pBuf[index];
                MTR->dwSetupSize  = (UINT32)slNextSize;
                break;
            case 0x0B:                          /* Sequense Data Chunk        */
                MTR->pSeq         = &pBuf[index];
                MTR->dwSeqSize    = (UINT32)slNextSize;
                break;
            case 0x0C:                          /* Stream PCM Wave Data Chunk */
                MTR->pWave        = &pBuf[index];
                MTR->dwWaveSize   = (UINT32)slNextSize;
                break;
            case 0xFF:                          /* Unknown Chunk              */
                break;
            default:
                return SMAF_ERR_CHUNK;
        }
        index += slNextSize;
    }
    if (MTR->pSeq == 0) {
        return SMAF_ERR_SHORT_LENGTH;
    }
    else
        return F_TRUE;
}


/*******************************************************************************
    Name        : MspI_Check
    Description : Get Seek & Phrase Information
    Parameter   : struct  *MTR ... Pointer to Score Track Chunk Information
    Return      : Nothing
*******************************************************************************/
static void MspI_Check(
    PMTRINFO MTR
)
{
    UINT8  i;
    UINT8  *pBuf;
    UINT32 dwSize;
    UINT16 tag;

    for (i = 0; i < 8; i++) {
        MTR->Phrase[i].dwSTp = 0xFFFFFFFF;
        MTR->Phrase[i].dwSPp = 0xFFFFFFFF;
        MTR->Phrase[i].dwSTt = 0xFFFFFFFF;
        MTR->Phrase[i].dwSPt = 0xFFFFFFFF;
    }
    if (MTR->pPhrase == 0)
        return;
    pBuf   = MTR->pPhrase;
    dwSize = MTR->dwPhraseSize;

    while( dwSize >= 8 ) {
        tag = (UINT16)((pBuf[0] << 8)+ pBuf[1]);/* TAG                        */
        pBuf += 3;                              /* Pointer Adjustment         */
        switch( tag ) {
            case 0x7374:                        /* Start Point                */
                MTR->Phrase[0].dwSTp = _get4b(&pBuf);
                dwSize -= 8;
                break;
            case 0x7370:                        /* Stop Point                 */
                MTR->Phrase[0].dwSPp = _get4b(&pBuf);
                dwSize -= 8;
                break;
            case 0x5041:                        /* Phrase A                   */
                if (dwSize >= 12) {
                    MTR->Phrase[1].dwSTp = _get4b(&pBuf);
                    MTR->Phrase[1].dwSPp = _get4b(&pBuf);
                }
                else
                    dwSize = 0;
                break;
            case 0x5042:                        /* Phrase B                   */
                if (dwSize >= 12) {
                    MTR->Phrase[2].dwSTp = _get4b(&pBuf);
                    MTR->Phrase[2].dwSPp = _get4b(&pBuf);
                }
                else
                    dwSize = 0;
                break;
            case 0x5045:                        /* Ending                     */
                if (dwSize >= 12) {
                    MTR->Phrase[3].dwSTp = _get4b(&pBuf);
                    MTR->Phrase[3].dwSPp = _get4b(&pBuf);
                }
                else
                    dwSize = 0;
                break;
            case 0x5049:                        /* Intro                      */
                if (dwSize >= 12) {
                    MTR->Phrase[4].dwSTp = _get4b(&pBuf);
                    MTR->Phrase[4].dwSPp = _get4b(&pBuf);
                }
                else
                    dwSize = 0;
                break;
            case 0x504B:                        /* Kanso                      */
                if (dwSize >= 12) {
                    MTR->Phrase[5].dwSTp = _get4b(&pBuf);
                    MTR->Phrase[5].dwSPp = _get4b(&pBuf);
                }
                else
                    dwSize = 0;
                break;
            case 0x5053:                        /* Sabi                       */
                if (dwSize >= 12) {
                    MTR->Phrase[6].dwSTp = _get4b(&pBuf);
                    MTR->Phrase[6].dwSPp = _get4b(&pBuf);
                }
                else
                    dwSize = 0;
                break;
            case 0x5052:                        /* Refrain                    */
                if (dwSize >= 12) {
                    MTR->Phrase[7].dwSTp = _get4b(&pBuf);
                    MTR->Phrase[7].dwSPp = _get4b(&pBuf);
                }
                else
                    dwSize = 0;
                break;
            default:
                dwSize = 0;
                break;
        }
        pBuf ++;                                /* ','                        */
    }
}


/*******************************************************************************
    Name        : ST_SP_Check
    Description : st/sp Position Check
    Parameter   : struct *pPhrase ... Pointer to st,sp structure
                : UINT32 SeqSize  ... Size of Sequence data
    Return      :  >0 ... Error
                :   0 ... Success
*******************************************************************************/
static UINT8 ST_SP_Check(
    PPHRA  pPhrase,
    UINT32 SeqSize
)
{
    UINT8  result;
    UINT32 dwSt, dwSp;

    result = 0;
    dwSt   = pPhrase->dwSTp;
    dwSp   = pPhrase->dwSPp;
    if (pPhrase->dwSTp == 0xFFFFFFFF)
        dwSt = 0;
    if (pPhrase->dwSPp == 0xFFFFFFFF)
        dwSp = SeqSize;

    if ((dwSt >= dwSp) || (dwSt > SeqSize) || (dwSp > SeqSize))
        result = SMAF_ERR_CHUNK;

#if ERROR_CHECK_STRENGTH
    if (result) {
        return result;
    }
#else
    if (result) {
        pPhrase->dwSTp = 0;
        pPhrase->dwSPp = SeqSize;
    }
#endif
    return F_TRUE;
}


/*******************************************************************************
    Name        : Ev_Set
    Description : Set Event Information to Event Block
    Parameter   : UINT32 cmd  ... Command #
                : UINT32 prm1 ... Parameter 1
                : UINT32 prm2 ... Parameter 2
                : UINT32 prm3 ... Parameter 3
                : UINT32 size ... Sequence Data Size
                : struct *pEv ... Pointer to Event List Block
    Return      : Nothing
*******************************************************************************/
static void Ev_Set(
    UINT32 cmd,
    UINT32 prm1,
    UINT32 prm2,
    UINT32 prm3,
    UINT32 size,
    PEVLIST pEv
)
{
    pEv->dwEvNo = cmd;
    pEv->dwVal1 = prm1;
    pEv->dwVal2 = prm2;
    pEv->dwVal3 = prm3;
    pEv->dwSize = size;
}


/*******************************************************************************
    Name        : Get_EvList
    Description : Get New(Empty) Event List Block
    Parameter   : Nothing
    Return      :   0 ... Fail
                :  !0 ... Success(Pointer to Got Event List)
*******************************************************************************/
static PEVLIST Get_EvList( void )
{
    void  *pEv;

    pEv = Play_Info.pEmptyEv;
    if (pEv != 0)
        Play_Info.pEmptyEv = ((PEVLIST)pEv)->pNext;
    return (PEVLIST)pEv;
}


/*******************************************************************************
    Name        : Get_OffList
    Description : Get New(Empty) Note OFF List Block
    Parameter   : Nothing
    Return      :   0 ... Fail
                :  !0 ... Success(Pointer to Got Note OFF List)
*******************************************************************************/
static POFFLIST Get_OffList( void )
{
    void *pOff;

    pOff = Play_Info.pEmptyOFF;
    if (pOff != 0)
        Play_Info.pEmptyOFF = ((POFFLIST)pOff)->pNext;
    return (POFFLIST)pOff;
}


/*******************************************************************************
    Name        : Set_EvList
    Description : Set Event List Block (Action Time Order)
    Parameter   : struct *pEv ... Pointer to Event List Block
    Return      : Nothing
*******************************************************************************/
static void Set_EvList(
    PEVLIST pEv
)
{
    void **Next;

    Next = &(Play_Info.pNextEv);
    while (*Next != 0) {
        if (((PEVLIST)*Next)->dwTime > pEv->dwTime)
            break;
        Next = &(((PEVLIST)*Next)->pNext);
    }
    pEv->pNext = *Next;
    *Next = (void *)pEv;
}


/*******************************************************************************
    Name        : Set_OffList
    Description : Set Note OFF List Block (Note OFF Time Order)
    Parameter   : struct *pOff ... Pointer to Note OFF List Block
    Return      : Nothing
*******************************************************************************/
static void Set_OffList(
    POFFLIST pOff
)
{
    void **Next;

    Next = &(Play_Info.pNextOFF);
    while (*Next != 0) {
        if (((POFFLIST)*Next)->dwTime > pOff->dwTime)
            break;
        Next = &(((POFFLIST)*Next)->pNext);
    }
    pOff->pNext = *Next;
    *Next = (void *)pOff;
}


/*******************************************************************************
    Name        : Search_EOS
    Description : Search EOS Event Block (For EP Change)
    Parameter   : Nothing
    Return      :   0 ... Fail
                :  !0 ... Success(Pointer to Event List Block)
*******************************************************************************/
static PEVLIST Search_EOS( void )
{
    void **Next;
    PEVLIST pEv;

    pEv  = 0;
    Next = &(Play_Info.pNextEv);
    while (*Next != 0) {
        pEv = (PEVLIST)*Next;
        if (pEv->dwEvNo == MMF_EVENT_EOS) {
            *Next = pEv->pNext;
            break;
        }
        Next = &(pEv->pNext);
    }
    return pEv;
}


/*******************************************************************************
    Name        : Search_OffList2
    Description : Search Note OFF List Block (MA-2 FM Note)
    Parameter   : UINT8  ch     ... Channel #
                : UINT32 dwTime ... Note OFF time
    Return      :   0 ... Fail
                :  !0 ... Success(Pointer to Found Note OFF List)
*******************************************************************************/
static POFFLIST Search_OffList2(
    UINT8  ch,
    UINT32 dwTime
)
{
    void **Next;
    POFFLIST pOff;

    Next = &(Play_Info.pNextOFF);
    pOff = (POFFLIST)*Next;
    while (pOff != 0) {
        if ((pOff->dwCh == ch) && (pOff->dwTime > dwTime) && 
            (pOff->dwType != MMF_EVENT_STREAM_OFF_MA2)) {
            *Next = pOff->pNext;
            return pOff;
        }
        Next = &(pOff->pNext);
        pOff = (POFFLIST)*Next;
    }
    return 0;
}


/*******************************************************************************
    Name        : Search_OffListA
    Description : Search Note OFF List Block (MA-2 Stream Note)
    Parameter   : UINT32 dwTime ... Note OFF time
    Return      :   0 ... Fail
                :  !0 ... Success(Pointer to Found Note OFF List)
*******************************************************************************/
static POFFLIST Search_OffListA(
    UINT32 dwTime
)
{
    void **Next;
    POFFLIST pOff;

    Next = &(Play_Info.pNextOFF);
    pOff = (POFFLIST)*Next;
    while (pOff != 0) {
        if ((pOff->dwTime > dwTime) &&
            (pOff->dwType == MMF_EVENT_STREAM_OFF_MA2)) {
            *Next = pOff->pNext;
            return pOff;
        }
        Next = &(pOff->pNext);
        pOff = (POFFLIST)*Next;
    }
    return 0;
}


/*******************************************************************************
    Name        : Search_OffList3
    Description : Search Note OFF List Block (MA-3 Stream Note)
    Parameter   : UINT8  ch     ... Channel #
                : UINT8  key    ... Key #
                : UINT32 dwTime ... Note OFF time
    Return      :   0 ... Fail
                :  !0 ... Success(Pointer to Found Note OFF List)
*******************************************************************************/
static POFFLIST Search_OffList3(
    UINT8  ch,
    UINT8  key,
    UINT32 dwTime
)
{
    void **Next;
    POFFLIST pOff;

    Next = &(Play_Info.pNextOFF);
    pOff = (POFFLIST)*Next;
    while (pOff != 0) {
        if ((pOff->dwCh == ch)&&(pOff->dwKey == key)&&(pOff->dwTime > dwTime)) {
            *Next = pOff->pNext;
            return pOff;
        }
        Next = &(pOff->pNext);
        pOff = (POFFLIST)*Next;
    }
    return 0;
}


/*******************************************************************************
    Name        : Set_EventEOS
    Description : Event Convert Process
    Parameter   : Nothing
    Return      : Number of Set Command
*******************************************************************************/
static SINT32 Set_EventEOS( void )
{
    UINT32 dwTime;

    if (Play_Info.bEOS == 0xFF)
        return 0;
    if (Play_Info.bEOS == 16) {
        MaSndDrv_SetCommand(MmfSeqID, 0, MASNDDRV_CMD_USER_EVENT,
            MASMW_MAX_USEREVENT, 0, 0);
        if(Play_Info.wLED)
            dwEventFlag |= (UINT32)EVENT_LED_OFF;
        if(Play_Info.wMOTOR)
            dwEventFlag |= (UINT32)EVENT_MOTOR_OFF;
        Play_Info.wLED = 0;
        Play_Info.wMOTOR = 0;
        Play_Info.bEOS = 0xFF;
        return 1;
    }
    if ((dwEndTick > Play_Info.dwPastTime) && (Play_Info.bEOS == 0))
        dwTime = (dwEndTick - Play_Info.dwPastTime) * Play_Info.bTimeBaseR;
    else
        dwTime = 0;
    MaSndDrv_SetCommand(MmfSeqID, dwTime, MASNDDRV_CMD_ALL_SOUND_OFF,
        Play_Info.bEOS, 0, 0);
    Play_Info.bEOS++;
    return 1;
}


/*--- Common A    Function ---------------------------------------------------*/
/*--- SMAF MA-1/2 Function ---------------------------------------------------*/

/*******************************************************************************
    Name        : CNTI_Check2
    Description : Contents Info Chunk Check (SMAF MA-1/2)
    Parameter   : UINT8  *pBuffer ... Pointer to CNTI
    Return      :  <0 ... Error
                : =>0 ... Success
*******************************************************************************/
static SINT32 CNTI_Check2(
    UINT8  *pBuffer
)
{
    UINT8  bTemp;

    bTemp = pBuffer[0];                         /* Contents Class             */
    if ((bTemp != CONTENTS_CLASS_1) && (bTemp != CONTENTS_CLASS_2))
        return MMFCNV_ERR_C_CLASS;

    bTemp = (UINT8)(pBuffer[1] & 0xF0);         /* Contents Type              */
    if (bTemp <= 0x2F) {
        if ((bTemp != CONTENTS_TYPE_1) && (bTemp != CONTENTS_TYPE_2)
                && (bTemp != CONTENTS_TYPE_3))
            return MMFCNV_ERR_C_TYPE;
    }
    else {
        if ((bTemp != CONTENTS_TYPE_4) && (bTemp != CONTENTS_TYPE_5)
            && (bTemp != CONTENTS_TYPE_6))
            return MMFCNV_ERR_C_TYPE;
    }
    return F_TRUE;
}


/*******************************************************************************
    Name        : ATR_Check
    Description : Audio Track Chunk Check
    Parameter   : struct *ATR  ... Pointer to Audio Track Chunk Information
                : UINT8  no   ... Array #
    Return      :  >0 ... Error
                :   0 ... Success
*******************************************************************************/
static UINT8 ATR_Check(
    PATRINFO ATR,
    UINT8  no
)
{
    UINT32 index;
    UINT32 result;
    UINT32 dwSize;
    UINT8  *pBuf;
    UINT16 wNextID;
    SINT32 slNextSize;
    UINT32 dwSR;
    UINT8  bWaveN, bWaveF;

    ATR->pPhrase = 0;
    ATR->pSeq    = 0;

    if (ATR->pATR[0] != 0x00) {
        return SMAF_ERR_CHUNK;                  /* Format Type Error!!        */
    }
    if (ATR->pATR[1] != 0x00) {
        return SMAF_ERR_CHUNK;                  /* Sequence Type Error!!      */
    }
    switch (ATR->pATR[2]) {
        case 0x10 :                             /* Mono, ADPCM, 4000Hz        */
            dwSR = 4000;
            break;
        case 0x11 :                             /* Mono, ADPCM, 8000Hz        */
            dwSR = 8000;
            break;
        default :
            return SMAF_ERR_CHUNK;              /* Wave Type Error!!          */
    }
    if ((ATR->pATR[3] & 0xF0) != 0x00) {
        return SMAF_ERR_CHUNK;                  /* Wave Type Error!!          */
    }
    if (ATR->pATR[4] != ATR->pATR[5]) {
        return SMAF_ERR_CHUNK;                  /* Time Base Error!!          */
    }
    result = (UINT32)_get_timebase(ATR->pATR[4]);
    if (result == 0) {
        return SMAF_ERR_CHUNK;                  /* Time Base Error!!          */
    }
    ATR->bTimeBase = (UINT8)result;

    dwSize = ATR->dwSize;
    pBuf   = ATR->pATR;
    index  = 6;
    if ( no ) {
        for (bWaveN = 0; bWaveN < 62; bWaveN++)
            Wave_Info2[bWaveN].pWave = 0;
    }
    bWaveN       = 0;
    bWaveF       = 0;

    while ((dwSize - index) > SIZE_OF_CHUNKHEADER) {
        slNextSize = NextChunk( 6, &pBuf[index], (dwSize - index), &wNextID);
        if (slNextSize < 0) {
            return SMAF_ERR_CHUNK;
        }
        index += SIZE_OF_CHUNKHEADER;
        switch((UINT8)(wNextID & 0x00FF)){
            case 0x0E:                          /* Seek & Phrase Info Chunk   */
                ATR->pPhrase      = &pBuf[index];
                ATR->dwPhraseSize = (UINT32)slNextSize;
                break;
            case 0x10:                          /* Sequense Data Chunk        */
                ATR->pSeq         = &pBuf[index];
                ATR->dwSeqSize    = (UINT32)slNextSize;
                break;
            case 0x11:                          /* Wave Data Chunk            */
                bWaveN = (UINT8)((wNextID >> 8) & 0x00FF);
                if ((bWaveN < 0x01) || (0x3E < bWaveN))
                    break;
                bWaveF = 1;
                if (no == 0)
                    break;
                if (Wave_Info2[bWaveN-1].pWave != 0)
                    break;
                Wave_Info2[bWaveN-1].dwSR       = dwSR;
                Wave_Info2[bWaveN-1].pWave      = &pBuf[index];
                Wave_Info2[bWaveN-1].dwWaveSize = (UINT32)slNextSize;
                break;
            case 0x0F:                          /* Setup Data Chunk           */
            case 0xFF:                          /* Unknown Chunk              */
                break;
            default:
                return SMAF_ERR_CHUNK;
        }
        index += slNextSize;
    }
    if ((ATR->pSeq == 0) || (bWaveF == 0))
        return SMAF_ERR_SHORT_LENGTH;
    else
        return F_TRUE;
}


/*******************************************************************************
    Name        : AspI_Check
    Description : Get Seek & Phrase Information
    Parameter   : struct  *ATR ... Pointer to Audio Track Chunk Information
    Return      : Nothing
*******************************************************************************/
static void AspI_Check(
    PATRINFO ATR
)
{
    UINT8  *pBuf;
    UINT32 dwSize;
    UINT16 tag;

    ATR->Phrase.dwSTp = 0xFFFFFFFF;
    ATR->Phrase.dwSPp = 0xFFFFFFFF;
    ATR->Phrase.dwSTt = 0xFFFFFFFF;
    ATR->Phrase.dwSPt = 0xFFFFFFFF;
    if (ATR->pPhrase == 0)
        return;
    pBuf   = ATR->pPhrase;
    dwSize = ATR->dwPhraseSize;

    while( dwSize >= 8 ) {
        tag = (UINT16)((pBuf[0] << 8)+ pBuf[1]);/* TAG                        */
        pBuf += 3;                              /* Pointer Adjustment         */
        switch( tag ) {
            case 0x7374:                        /* Start Point                */
                ATR->Phrase.dwSTp = _get4b(&pBuf);
                dwSize -= 8;
                break;
            case 0x7370:                        /* Stop Point                 */
                ATR->Phrase.dwSPp = _get4b(&pBuf);
                dwSize -= 8;
                break;
            default:
                dwSize = 0;
                break;
        }
        pBuf ++;                                /* ','                        */
    }
}


/*******************************************************************************
    Name        : Mtsu_Check2
    Description : Setup Data Check
    Parameter   : struct  *MTR ... Pointer to Score Track Chunk Information
                : UINT8  Type ... SMAF Type
    Return      :  >0 ... Error
                :   0 ... Success
*******************************************************************************/
static UINT8 Mtsu_Check2(
    PMTRINFO MTR,
    UINT8  Type
)
{
#if ERROR_CHECK_STRENGTH
    UINT8  *pBuf;
    UINT32 dwSize;
    UINT32 index;

    pBuf   = MTR->pSetup;
    dwSize = MTR->dwSetupSize;
    index  = 0;

    if (pBuf == 0)
        return SMAF_ERR_CHUNK;
    while ((dwSize - index) > 20) {
        if ((pBuf[index] != 0xFF) || (pBuf[index + 1] != 0xF0))
            return SMAF_ERR_CHUNK;
        if (pBuf[index + 3] == 0x43) {
            if ((Type == SMAF_TYPE_MA1) && (pBuf[index + 4] == 0x02))
                return F_TRUE;
            if ((Type == SMAF_TYPE_MA2) && (pBuf[index + 4] == 0x03))
                return F_TRUE;
        }
        index += (pBuf[index + 2] + 3);
    }
    return SMAF_ERR_CHUNK;
#else
    return F_TRUE;
#endif
}


/*******************************************************************************
    Name        : get_flex2
    Description : Check Sequence Data Chunk (Score Track Chunk)
    Parameter   : UINT8  *pBuf  ... Pointer to Sequence Data
                : UINT32 dwSize ... Reamin Sequence Data Size
    Return      :  0xFFFFFFFF: Error
                : !0xFFFFFFFF: Success
*******************************************************************************/
static UINT32 get_flex2(
    UINT8  *pBuf,
    UINT32 dwSize
)
{
    UINT32 dwTemp;

    if ((dwSize < 1) || ((dwSize < 2) && (pBuf[0] >= 0x80)))
        return 0xFFFFFFFF;
    if ((dwSize >= 4) &&
        ((pBuf[0] == 0) && (pBuf[1] == 0) && (pBuf[2] == 0) && (pBuf[3] == 0)))
        return 0xFFFFFFFF;
    if (pBuf[0] >= 0x80)
        dwTemp = (UINT32)(((pBuf[0] & 0x7F) << 7) + (pBuf[1] & 0x7F) + 128);
    else
        dwTemp = (UINT32)(pBuf[0]);
    return dwTemp;
}


/*******************************************************************************
    Name        : SeqData_Check2
    Description : Check Sequence Data Chunk
    Parameter   : UINT8  *pBuf    ... Pointer to Track Chunk Information
                : UINT32 dwSize   ... Track Chunk Size
                : struct *pPhrase ... Pointer to Phrase Inforamtion
                : UINT8  Type ... SMAF Type
    Return      :  >0: Error
                :   0: Success
*******************************************************************************/
static UINT8 SeqData_Check2(
    UINT8  *pBuf,
    UINT32 dwSize,
    PPHRA  pPhrase,
    UINT8  Type
)
{
    UINT32 i;                                   /* Reference Point            */
    UINT32 dwDelta;                             /* Duration                   */
    UINT32 dwGate;                              /* Gate Time                  */
    UINT32 dwTime;                              /* Past Time                  */
    UINT32 dwTemp;

    i      = 0;
    dwTime = 0;
    dwGate = 0;

    if (pBuf == 0) {
        return SMAF_ERR_SHORT_LENGTH;           /* Mtsq not Exist             */
    }

    while (dwSize > i) {
        if(i == pPhrase->dwSTp)
            pPhrase->dwSTt = dwTime;
        if(i == pPhrase->dwSPp) {
            pPhrase->dwSPt = dwTime;
            break;
        }

        if ((dwSize - i) >= 4) {
            if ((pBuf[i] + pBuf[i+1] + pBuf[i+2] + pBuf[i+3]) == 0) {
                if (Type == SMAF_TYPE_MA1)
                    pPhrase->dwSPt = dwTime + dwGate;
                else
                    pPhrase->dwSPt = dwTime;
                break;
            }
        }

        dwDelta = get_flex2(&(pBuf[i]), (dwSize - i));
        if (dwDelta == 0xFFFFFFFF) {
            return SMAF_ERR_CHUNK;
        }
        if (dwDelta >= 128)
            i++;
        i++;
        dwTime += dwDelta;

        if (dwDelta < dwGate)
            dwGate -= dwDelta;
        else
            dwGate = 0;

        if ((dwSize - i) < 2) {
            return SMAF_ERR_CHUNK;
        }
        if (pBuf[i] == 0x00) {                  /* Control Event              */
            if ((pBuf[i+1] & 0x30) != 0x30)     /* 2 Byte Event               */
                i += 2;
            else                                /* 3 Byte Event               */
                i += 3;
        }
        else if (pBuf[i] == 0xFF) {             /* Exclusive or NOP Event     */
            if (pBuf[i+1] == 0x00)              /* NOP                        */
                i += 2;
            else if (pBuf[i+1] == 0xF0) {       /* Exclusive                  */
                if ((dwSize - i) < 4) {
                    return SMAF_ERR_CHUNK;      /* Size Error                 */
                }
                i += pBuf[i+2] + 3;
                if (dwSize < i) {
                    return SMAF_ERR_CHUNK;      /* Size Error                 */
                }
#if ERROR_CHECK_STRENGTH
                if (pBuf[i-1] != 0xF7) {
                    return SMAF_ERR_CHUNK;      /* Format Error               */
                }
#endif
            }
            else {
                return SMAF_ERR_CHUNK;          /* Event Format Error         */
            }
        }
        else {                                  /* Note Event                 */
            dwTemp = get_flex2(&(pBuf[i+1]), (dwSize - i - 1));
            if (dwTemp == 0xFFFFFFFF) {
                return SMAF_ERR_CHUNK;
            }
            if (dwGate < dwTemp)
                dwGate = dwTemp;
            if (dwTemp >= 128)
                i += 3;
            else
                i += 2;
        }
        if (dwSize < i) {
            return SMAF_ERR_CHUNK;
        }
    } /* while (dwSize > i) */

#if ERROR_CHECK_STRENGTH
    if ((pPhrase->dwSTt == 0xFFFFFFFF) && (pPhrase->dwSTp != 0xFFFFFFFF)) {
        return SMAF_ERR_CHUNK;
    }
#endif
    if (pPhrase->dwSTt == 0xFFFFFFFF)
        pPhrase->dwSTt = 0;

#if ERROR_CHECK_STRENGTH
    if ((pPhrase->dwSPt == 0xFFFFFFFF) && (pPhrase->dwSPp != 0xFFFFFFFF) &&
        (pPhrase->dwSPp != i)) {
        return SMAF_ERR_CHUNK;
    }
#endif
    if (pPhrase->dwSPt == 0xFFFFFFFF) {
        if (Type == SMAF_TYPE_MA1)
            pPhrase->dwSPt = dwTime + dwGate;
        else
            pPhrase->dwSPt = dwTime;
    }
    return F_TRUE;
}


/*******************************************************************************
    Name        : TrackChunk_Check2
    Description : Score Track Chunk Check
    Parameter   : struct  *SmafCheck ... Pointer to File Information
                : UINT8  no         ... Array #(0:”ñ“o˜^,1:“o˜^)
    Return      :  >0 ... Error
                :   0 ... Success
*******************************************************************************/
static UINT8 TrackChunk_Check2(
    PSMAFINFO SmafCheck,
    UINT8  no
)
{
    UINT8  result;
    UINT8  i;
    UINT32 dwPlayTime;
    PMTRINFO pMTR;
    PATRINFO pATR;

    result = 0;

    for (i = 1; i < 5; i++) {
        if (SmafCheck->ScoreTrack[i].pMTR != 0)
            result ++;
    }
    if ((result == 0) && (SmafCheck->AudioTrack.pATR == 0)) {
        if (SmafCheck->ScoreTrack[0].pMTR == 0) {
            return SMAF_ERR_SHORT_LENGTH;       /* Track Chunk Nothing        */
        }
        SmafCheck->bSmafType = SMAF_TYPE_MA1;
    }

    if (SmafCheck->bSmafType == SMAF_TYPE_MA1) {/* SMAF Type MA-1             */
        if (SmafCheck->ScoreTrack[0].dwSize <= 6) {
            return SMAF_ERR_CHUNK;              /* Lack Header Information    */
        }
        pMTR = &(SmafCheck->ScoreTrack[0]);
        result = MTR_Check(pMTR, SMAF_TYPE_MA1);
        if (result != 0)
            return result;
        MspI_Check(pMTR);
        result = ST_SP_Check(&(pMTR->Phrase[0]), pMTR->dwSeqSize);
        if (result)
            return result;
        result = Mtsu_Check2(pMTR, SMAF_TYPE_MA1);
        if (result) {
            return result;                      /* Voice Info not Exist       */
        }
        result = SeqData_Check2(pMTR->pSeq, pMTR->dwSeqSize, 
                                &(pMTR->Phrase[0]), SMAF_TYPE_MA1);
        if (result)
            return result;
        pMTR->dwPlayTime = pMTR->Phrase[0].dwSPt - pMTR->Phrase[0].dwSTt;
        SmafCheck->bTimeBase   = pMTR->bTimeBase;
        SmafCheck->dwStartTime = pMTR->Phrase[0].dwSTt;
        SmafCheck->dwPlayTime  = pMTR->dwPlayTime;
    }
    else {                                      /* SMAF Type MA-2             */
        SmafCheck->ScoreTrack[0].pMTR = 0;      /* Erase MA-1 Score Track     */
        SmafCheck->ScoreTrack[0].pSeq = 0;      /* Erase MA-1 Sequence Data   */
        for (i = 1; i < 5; i++) {               /* MTR* Check                 */
            if (SmafCheck->ScoreTrack[i].pMTR != 0) {
                if (SmafCheck->ScoreTrack[i].dwSize <= 6) {
                    return SMAF_ERR_CHUNK;      /* Lack Header Information    */
                }
                pMTR = &(SmafCheck->ScoreTrack[i]);
                result = MTR_Check(pMTR, SMAF_TYPE_MA2);
                if (result != 0)
                    return result;
                MspI_Check(pMTR);
                result = ST_SP_Check(&(pMTR->Phrase[0]), pMTR->dwSeqSize);
                if (result)
                    return result;
                result = SeqData_Check2(pMTR->pSeq, pMTR->dwSeqSize,
                    &(pMTR->Phrase[0]), SMAF_TYPE_MA2);
                if (result)
                    return result;
                pMTR->dwPlayTime = pMTR->Phrase[0].dwSPt -
                    pMTR->Phrase[0].dwSTt;
            }
            else {                              /* Empty Track                */
                pMTR = &(SmafCheck->ScoreTrack[i]);
                pMTR->pPhrase = 0;
                pMTR->pSetup  = 0;
                pMTR->pSeq    = 0;
            }
        }
        if (SmafCheck->AudioTrack.pATR != 0) {  /* ATR0 Check                 */
            if (SmafCheck->AudioTrack.dwSize <= 6) {
                return SMAF_ERR_CHUNK;          /* Lack Header Information    */
            }
            pATR = &(SmafCheck->AudioTrack);
            result = ATR_Check(pATR, no);
            if (result != 0)
                return result;
            AspI_Check(pATR);
            result = ST_SP_Check(&(pATR->Phrase), pATR->dwSeqSize);
            if (result)
                return result;
            result = SeqData_Check2(pATR->pSeq, pATR->dwSeqSize, 
                                    &(pATR->Phrase), SMAF_TYPE_MA2);
            if (result)
                return result;
            pATR->dwPlayTime = pATR->Phrase.dwSPt - pATR->Phrase.dwSTt;
        }
        dwPlayTime = 0;

        for (i = 1; i < 5; i++) {
            if (SmafCheck->ScoreTrack[i].pMTR != 0) {
                pMTR = &(SmafCheck->ScoreTrack[i]);
                SmafCheck->bTimeBase   = pMTR->bTimeBase;
                SmafCheck->dwStartTime = pMTR->Phrase[0].dwSTt;
                SmafCheck->dwPlayTime  = pMTR->dwPlayTime;
                break;
            }
        }
        if (i == 5) {
            SmafCheck->bTimeBase   = SmafCheck->AudioTrack.bTimeBase;
            SmafCheck->dwStartTime = SmafCheck->AudioTrack.Phrase.dwSTt;
            SmafCheck->dwPlayTime  = SmafCheck->AudioTrack.dwPlayTime;
        }
        for (i = i; i < 5; i++) {
            if (SmafCheck->ScoreTrack[i].pMTR == 0)
                continue;
            pMTR = &(SmafCheck->ScoreTrack[i]);
            if ((SmafCheck->bTimeBase   != pMTR->bTimeBase) ||
                (SmafCheck->dwStartTime != pMTR->Phrase[0].dwSTt)) {
                return SMAF_ERR_CHUNK;
            }
            if (SmafCheck->dwPlayTime < pMTR->dwPlayTime)
                SmafCheck->dwPlayTime = pMTR->dwPlayTime;
        }
        if (SmafCheck->AudioTrack.pATR != 0) {
            pATR = &(SmafCheck->AudioTrack);
            if ((SmafCheck->bTimeBase   != pATR->bTimeBase) ||
                (SmafCheck->dwStartTime != pATR->Phrase.dwSTt)) {
                return SMAF_ERR_CHUNK;
            }
            if (SmafCheck->dwPlayTime < pATR->dwPlayTime)
                SmafCheck->dwPlayTime = pATR->dwPlayTime;
        }

        for (i = 1; i < 5; i++) {
            if (SmafCheck->ScoreTrack[i].pMTR == 0)
                continue;
            result = Mtsu_Check2(&(SmafCheck->ScoreTrack[i]), SMAF_TYPE_MA2);
            if (result == F_TRUE)
                break;
        }
        if (result) {
            return result;                      /* Voice Info not Exist       */
        }
    }

    return F_TRUE;
}


/*******************************************************************************
    Name        : Get_LM_Status1
    Description : Get LED Motor Status
    Parameter   : Nothing
    Return      : Nothing
*******************************************************************************/
static void Get_LM_Status1( void )
{
    UINT8  i;
    UINT16 wLed, wMotor;
    SINT32 ret;
    MASMW_CONTENTSINFO Led_State;

    wLed = 0;
    wMotor = 0;
    i = 0;

    Led_State.code   = 0xFF;
    Led_State.tag[0] = 'L';
    Led_State.tag[1] = '2';
    Led_State.buf    = &i;
    Led_State.size   = 1;
    ret = MaMmfCnv_Control(1, MASMW_GET_CONTENTSDATA, &Led_State, 0);
    if (ret == 1) {
		i = (UINT8)(i & 0x0F);
        if (i <= 4)
            wLed = (UINT16)(1 << i);
        if (i == 0x0C)
            wLed = 0x000F;
    }

    i = Smaf_Info[1].ScoreTrack[0].pMTR[4];
    if (i & 0x40)
        wMotor += 1;
    if (i & 0x04)
        wMotor += 2;
    i = Smaf_Info[1].ScoreTrack[0].pMTR[5];
    if (i & 0x40)
        wMotor += 4;
    if (i & 0x04)
        wMotor += 8;

    for (i = 0; i < 16; i++) {
        if (wLed & (1 << i))
            ChInfo[i].bLed = 1;
        else
            ChInfo[i].bLed = 0;
        if (wMotor & (1 << i))
            ChInfo[i].bMotor = 1;
        else
            ChInfo[i].bMotor = 0;
    }
}


/*******************************************************************************
    Name        : Get_LM_Status2
    Description : Get LED Motor Status
    Parameter   : Nothing
    Return      : Nothing
*******************************************************************************/
static void Get_LM_Status2( void )
{
    UINT8  i, j;
    UINT16 wLed, wMotor;
    SINT32 ret;
    MASMW_CONTENTSINFO Led_State;

    wLed = 0;
    wMotor = 0;
    i = 0;

    Led_State.code   = 0xFF;
    Led_State.tag[0] = 'L';
    Led_State.tag[1] = '2';
    Led_State.buf    = &i;
    Led_State.size   = 1;
    ret = MaMmfCnv_Control(1, MASMW_GET_CONTENTSDATA, &Led_State, 0);
    if (ret == 1) {
		i = (UINT8)(i & 0x0F);
        if (i <= 0x0B)
            wLed = (UINT16)(1 << i);
        if (i == 0x0C)
            wLed = 0x000F;
        if (i == 0x0D)
            wLed = 0x00F0;
        if (i == 0x0E)
            wLed = 0x0F00;
        if (i == 0x0F)
            bAudio_LED = 0x01;
    }

    for (j = 1; j < 5; j++) {
        if (Smaf_Info[1].ScoreTrack[j].pMTR == 0)
            continue;
        i = Smaf_Info[1].ScoreTrack[j].pMTR[4];
        if (i & 0x40)
            wMotor |= (UINT16)(0x0001 << ((j - 1)* 4));
        if (i & 0x04)
            wMotor |= (UINT16)(0x0002 << ((j - 1)* 4));
        i = Smaf_Info[1].ScoreTrack[j].pMTR[5];
        if (i & 0x40)
            wMotor |= (UINT16)(0x0004 << ((j - 1)* 4));
        if (i & 0x04)
            wMotor |= (UINT16)(0x0008 << ((j - 1)* 4));
    }

    for (i = 0; i < 16; i++) {
        if (wLed & (1 << i))
            ChInfo[i].bLed = 1;
        else
            ChInfo[i].bLed = 0;
        if (wMotor & (1 << i))
            ChInfo[i].bMotor = 1;
        else
            ChInfo[i].bMotor = 0;
    }
}


/*******************************************************************************
    Name        : Stream2
    Description : Regist PCM Data
    Parameter   : Nothing
    Return      : Nothing
*******************************************************************************/
static void Stream2( void )
{
    UINT8  i, bKeyNo;

    bKeyNo = 0;
    for (i = 0; i < 62; i++) {
        if (Wave_Info2[i].pWave == 0)
            continue;
        MaSndDrv_SetStream(MmfSeqID, bKeyNo, 0, Wave_Info2[i].dwSR,
            Wave_Info2[i].pWave, Wave_Info2[i].dwWaveSize);
        Wave_Info2[i].bKeyNo = bKeyNo;
        bKeyNo ++;
        if (bKeyNo >= 32)
            break;
    }

    for (i = (UINT8)(i + 1); i < 62; i++)
        Wave_Info2[i].pWave = 0;
}


/*******************************************************************************
    Name        : Get_Param_MA1
    Description : Set Voice Parameter (MA-1)
    Parameter   : UINT8  bCount ... Voice #
                : UINT8  *pBuf  ... Pointer to Voice Parameter
    Return      :   0 ... Success
                :  !0 ... Error
*******************************************************************************/
static SINT8 Get_Param_MA1(
    UINT8  bCount,
    UINT8  *pBuf
)
{
    UINT8  i, j;
    UINT8  bBuf[16];
    UINT8  bBO, bML, bVIB, bEGT, bSUS, bRR, bDR, bAR, bSL, bTL, bWAV, bFL;
    UINT8  bSR;

    if (pBuf[0] & 0xF8)
        return F_FALSE;

    bBO = (UINT8)(pBuf[3] & 0x03);
    bBuf[0] = (UINT8)(bBO | 0x80);              /* Panpot, BO                 */
    bBuf[1] = 0x80;                             /* LFO, PE, ALG               */

    i = j = 0;
    while ( i < 5) {
        bML  = (UINT8)((pBuf[4+i] >> 5) & 0x07);
        bVIB = (UINT8)((pBuf[4+i] >> 4) & 0x01);
        bEGT = (UINT8)((pBuf[4+i] >> 3) & 0x01);
        bSUS = (UINT8)((pBuf[4+i] >> 2) & 0x01);
        bRR  = (UINT8)( ((pBuf[4+i] & 0x03) << 2) | ((pBuf[5+i] >> 6) & 0x03) );
        bDR  = (UINT8)((pBuf[5+i] >> 2) & 0x0F);
        bAR  = (UINT8)( ((pBuf[5+i] & 0x03) << 2) | ((pBuf[6+i] >> 6) & 0x03) );
        bSL  = (UINT8)((pBuf[6+i] >> 2) & 0x0F);
        bTL  = (UINT8)( ((pBuf[6+i] & 0x03) << 4) | ((pBuf[7+i] >> 4) & 0x0F) );
        bWAV = (UINT8)((pBuf[7+i] >> 3) & 0x01);
        bFL  = (UINT8)(pBuf[7+i] & 0x07);

        if (bEGT == 0) {
            bSR = TableD[bRR];
            if (bSUS == 0)
                bRR = 8;
            else
                bRR = TableD[bRR];
        }
        else {
            bSR = 0;
            if (bSUS == 0)
                bRR = TableD[bRR];
            else
                bRR = 5;
        }
        if (i != 0)
            bFL = 0;
        bDR = TableD[bDR];
        bAR = TableA[bAR];

        bBuf[2+j] = (UINT8)( (bSR  << 4) & 0xF0);
        bBuf[3+j] = (UINT8)( ((bRR << 4) & 0xF0) + (bDR & 0x0F) );
        bBuf[4+j] = (UINT8)( ((bAR << 4) & 0xF0) + (bSL & 0x0F) );
        bBuf[5+j] = (UINT8)( (bTL  << 2) & 0xFC);
        bBuf[6+j] = (UINT8)( (bVIB & 0x01) + 0x04);
        bBuf[7+j] = (UINT8)( (bML  << 4) & 0xF0);
        bBuf[8+j] = (UINT8)( ((bWAV << 3) & 0xF8) + (bFL & 0x07) );

        i += 4;
        j += 7;
    }

    Voice_Info[bCount].bBank = pBuf[1];
    Voice_Info[bCount].bProg = (UINT8)(pBuf[2] & 0x7F);

    if ((dwRamAdr % 2) && (dwRamSize != 0)) {
        dwRamAdr  ++;
        dwRamSize --;
    }
    if (dwRamSize < 16)
        return F_FALSE;
    MaDevDrv_SendDirectRamData(dwRamAdr, 0, bBuf, 16);
    MaSndDrv_SetVoice(MmfSeqID, 1, bCount, 1, 0, dwRamAdr);
    dwRamAdr  += 16;
    dwRamSize -= 16;

    return F_TRUE;
}


/*******************************************************************************
    Name        : Get_Param_MA2
    Description : Set Voice Parameter (MA-2)
    Parameter   : UINT8  bCount ... Voice #
                : UINT8  *pBuf  ... Pointer to Voice Parameter
    Return      :   0 ... Success
                :  !0 ... Error
*******************************************************************************/
static SINT8 Get_Param_MA2(
    UINT8  bCount,
    UINT8  *pBuf
)
{
    UINT8  i, j, bOperator, bSize;
    UINT8  bBuf[30];
    UINT8  bLFO, bFB, bALG, bBO, bML, bVIB, bEGT, bSUS, bKSR, bRR, bDR, bAR;
    UINT8  bSL, bTL, bKSL, bDVB, bDAM, bAM, bWS;
    UINT8  bSR;

    if (pBuf[0] & 0xF0)
        return F_FALSE;

    bLFO = (UINT8)((pBuf[3] >> 6) & 0x03);
    bFB  = (UINT8)((pBuf[3] >> 3) & 0x07);
    bALG = (UINT8)(pBuf[3] & 0x07);
    bBO  = (UINT8)(pBuf[4] & 0x03);
    bBuf[0] = (UINT8)(bBO | 0x80);
    bBuf[1] = (UINT8)( ((bLFO << 6) & 0xC0) + (bALG & 0x07) );

    if (bALG >= 2) {
        bOperator = 4;
        bSize = 30;
    }
    else {
        bOperator = 2;
        bSize = 16;
    }
    i = j = 0;
    while (bOperator != 0) {
        bML  = (UINT8)((pBuf[5+i] >> 4) & 0x0F);
        bVIB = (UINT8)((pBuf[5+i] >> 3) & 0x01);
        bEGT = (UINT8)((pBuf[5+i] >> 2) & 0x01);
        bSUS = (UINT8)((pBuf[5+i] >> 1) & 0x01);
        bKSR = (UINT8)( pBuf[5+i] & 0x01);
        bRR  = (UINT8)((pBuf[6+i] >> 4) & 0x0F);
        bDR  = (UINT8)( pBuf[6+i] & 0x0F);
        bAR  = (UINT8)((pBuf[7+i] >> 4) & 0x0F);
        bSL  = (UINT8)( pBuf[7+i] & 0x0F);
        bTL  = (UINT8)((pBuf[8+i] >> 2) & 0x3F);
        bKSL = (UINT8)( pBuf[8+i] & 0x03);
        bDVB = (UINT8)((pBuf[9+i] >> 6) & 0x03);
        bDAM = (UINT8)((pBuf[9+i] >> 4) & 0x03);
        bAM  = (UINT8)((pBuf[9+i] >> 3) & 0x01);
        bWS  = (UINT8)( pBuf[9+i] & 0x07);

        if (bEGT == 0)
            bSR = bRR;
        else
            bSR = 0;
        if (bSUS != 0)
            bRR = 4;

        bBuf[2+j] = (UINT8)( ((bSR  << 4) & 0xF0) + (bKSR & 0x01) );
        bBuf[3+j] = (UINT8)( ((bRR  << 4) & 0xF0) + (bDR  & 0x0F) );
        bBuf[4+j] = (UINT8)( ((bAR  << 4) & 0xF0) + (bSL  & 0x0F) );
        bBuf[5+j] = (UINT8)( ((bTL  << 2) & 0xFC) + (bKSL & 0x03) );
        bBuf[6+j] = (UINT8)( ((bDAM << 5) & 0x60) + ((bAM << 4) & 0x10) +
                             ((bDVB << 1) & 0x06) + (bVIB & 0x01) );
        bBuf[7+j] = (UINT8)((bML << 4) & 0xF0);
        bBuf[8+j] = (UINT8)( ((bWS  << 3) & 0xF8) + (bFB  & 0x07) );

        i += 5;
        j += 7;
        bOperator --;
        bFB = 0;
    }

    Voice_Info[bCount].bBank = pBuf[1];
    Voice_Info[bCount].bProg = (UINT8)(pBuf[2] & 0x7F);

    if ((dwRamAdr % 2) && (dwRamSize != 0)) {
        dwRamAdr  ++;
        dwRamSize --;
    }
    if (dwRamSize < bSize)
        return F_FALSE;
    MaDevDrv_SendDirectRamData(dwRamAdr, 0, bBuf, bSize);
    MaSndDrv_SetVoice(MmfSeqID, 1, bCount, 1, 0, dwRamAdr);
    dwRamAdr  += bSize;
    dwRamSize -= bSize;

    return F_TRUE;
}


/*******************************************************************************
    Name        : Set_Voice2
    Description : Set Voice Parameter
    Parameter   : Nothing
    Return      : Nothing
*******************************************************************************/
static void Set_Voice2( void )
{
    UINT8  i, bVoiceCount;
    UINT8  *pBuf;
    SINT8  ret;
    UINT32 dwSize, index;

    for (i = 0; i < 16; i++) {
        Voice_Info[i].bBank = 0xFF;
        Voice_Info[i].bProg = 0xFF;
    }

    bVoiceCount = 0;

    for (i = 0; i < 5; i++) {
        if ((Smaf_Info[1].ScoreTrack[i].pMTR == 0) ||
            (Smaf_Info[1].ScoreTrack[i].pSetup == 0))
            continue;
        index  = 0;
        pBuf   = Smaf_Info[1].ScoreTrack[i].pSetup;
        dwSize = Smaf_Info[1].ScoreTrack[i].dwSetupSize;

        while (dwSize >= (index + 21)) {
            if ((pBuf[index]   != 0xFF) || (pBuf[index+1] != 0xF0) ||
                (pBuf[index+2] <= 0x11) || (pBuf[index+3] != 0x43) ||
                (pBuf[index+4] <= 0x01) || (pBuf[index+4] >= 0x04)) {
                index += (pBuf[index+2] + 3);
                continue;
            }
            if (pBuf[index+4] == 0x02)
                ret = Get_Param_MA1(bVoiceCount, &pBuf[index+5]);
            else
                ret = Get_Param_MA2(bVoiceCount, &pBuf[index+5]);
            if (ret == F_TRUE)
                bVoiceCount ++;
            if (bVoiceCount >= 16)
                return;
            index += (pBuf[index+2] + 3);
        } /* while (dwSize >= (index + 21)) : Mtsu Search */
    } /* for (i = 0; i < 5; i++) : All MTR Search */
}


/*******************************************************************************
    Name        : Standby2
    Description : Standby Process (Set voice param)
    Parameter   : Nothing
    Return      : Nothing
*******************************************************************************/
static void Standby2( void )
{
    UINT8  i, j;
    UINT16 wKeyCon;

    wKeyCon = 0;
    if (Smaf_Info[1].bSmafType == SMAF_TYPE_MA1) {
#if MA13_KCS_IGNORE
		wKeyCon = 15;
#else
        i = Smaf_Info[1].ScoreTrack[0].pMTR[4];
        if (i & 0x80)
            wKeyCon += 1;
        if (i & 0x08)
            wKeyCon += 2;
        i = Smaf_Info[1].ScoreTrack[0].pMTR[5];
        if (i & 0x80)
            wKeyCon += 4;
        if (i & 0x08)
            wKeyCon += 8;
#endif
    }
    else {
        if (Smaf_Info[1].AudioTrack.pATR != 0)
            Stream2();                          /* Entry Stream Data          */
        for (j = 1; j < 5; j++) {
            if (Smaf_Info[1].ScoreTrack[j].pMTR == 0)
                continue;
            i = Smaf_Info[1].ScoreTrack[j].pMTR[4];
            if (i & 0x80)
                wKeyCon |= (UINT16)(0x0001 << ((j - 1)* 4));
            if (i & 0x08)
                wKeyCon |= (UINT16)(0x0002 << ((j - 1)* 4));
            i = Smaf_Info[1].ScoreTrack[j].pMTR[5];
            if (i & 0x80)
                wKeyCon |= (UINT16)(0x0004 << ((j - 1)* 4));
            if (i & 0x08)
                wKeyCon |= (UINT16)(0x0008 << ((j - 1)* 4));
        }
    }

    Set_Voice2();                               /* Entry Set Up Data          */

    for (i = 0; i < 16; i++) {
        if (wKeyCon & (1 << i))
            MaSndDrv_SetKeyControl(MmfSeqID, i, 2);
        else
            MaSndDrv_SetKeyControl(MmfSeqID, i, 1);
    }
}


/*******************************************************************************
    Name        : Note_ON2
    Description : Note ON Process (MA-1/2)
    Parameter   : UINT8  ch     ... Channel Number
                : UINT8  key    ... Key No.
                : UINT32 gtime  ... Gatetime
                : struct *pEv   ... Pointer to Event List
                ; UINT32 dwSize ... Sequence Data Size
    Return      : Nothing
*******************************************************************************/
static void Note_ON2(
    UINT8  ch,
    UINT8  oct,
    UINT8  key,
    UINT32 gtime,
    PEVLIST pEv,
    UINT32 dwSize
)
{
    POFFLIST pOffList;
    UINT8  bKey;

    if (gtime == 0) {
        Ev_Set(MASNDDRV_CMD_NOP, 0, 0, 0, dwSize, pEv);
        return ;
    }

    bKey = (UINT8)(((oct + ChInfo[ch].bType)* 12) + key);
    if (bKey < 12)
        bKey = 0;
    else if (bKey > 139)
        bKey = 127;
    else
        bKey = (UINT8)(bKey - 12);

    pOffList = Search_OffList2(ch, pEv->dwTime);
    if (pOffList == 0) {
        pOffList = Get_OffList();
        if (pOffList == 0) {
            Ev_Set(MASNDDRV_CMD_NOP, 0, 0, 0, dwSize, pEv);
            return ;
        }
    }

    pOffList->dwTime = pEv->dwTime + gtime;
    pOffList->dwKey  = bKey;
    pOffList->dwCh   = ch;
    pOffList->dwType = MASNDDRV_CMD_NOTE_OFF_MA2;
    Set_OffList(pOffList);
    Ev_Set(MASNDDRV_CMD_NOTE_ON_MA2, ch, bKey, 127, dwSize, pEv);
}


/*******************************************************************************
    Name        : Stream_ON2
    Description : Wave ON Process (MA-1/2)
    Parameter   : UINT8  ch     ... Channel Number
                : UINT8  key    ... Key No.
                : UINT32 gtime  ... Gatetime
                : struct *pEv   ... Pointer to Event List
                ; UINT32 dwSize ... Sequence Data Size
    Return      : Nothing
*******************************************************************************/
static void Stream_ON2(
    UINT8  key,
    UINT32 gtime,
    PEVLIST pEv,
    UINT32 dwSize
)
{
    POFFLIST pOffList;

    if ((key < 1) || (0x3E < key) || (gtime == 0)) {
        Ev_Set(MASNDDRV_CMD_NOP, 0, 0, 0, dwSize, pEv);
        return ;
    }
    if (Wave_Info2[key-1].pWave == 0) {
        Ev_Set(MASNDDRV_CMD_NOP, 0, 0, 0, dwSize, pEv);
        return ;
    }

    pOffList = Search_OffListA(pEv->dwTime);
    if (pOffList != 0) {
        pOffList->dwTime = pEv->dwTime + gtime;
        Set_OffList(pOffList);
        Ev_Set(MASNDDRV_CMD_NOP, 0, 0, 0, dwSize, pEv);
        return ;
    }
    pOffList = Get_OffList();
    if (pOffList == 0) {
        Ev_Set(MASNDDRV_CMD_NOP, 0, 0, 0, dwSize, pEv);
        return ;
    }
    key = Wave_Info2[key-1].bKeyNo;
    pOffList->dwTime = pEv->dwTime + gtime;
    pOffList->dwKey  = key;
    pOffList->dwCh   = 0;
    pOffList->dwType = MMF_EVENT_STREAM_OFF_MA2;
    Set_OffList(pOffList);
    Ev_Set(MMF_EVENT_STREAM_ON_MA2, 0, key, bAudio_Vel, dwSize, pEv);
}


/*******************************************************************************
    Name        : Bank_Program2
    Description : Reflection BankSelect and ProgramChange Message (MA-1/2)
    Parameter   : UINT32 dwSize ... Size of Sequence Data
                : UINT8  ch     ... Channel Number
                : struct *pEv   ... Pointer to Event List
    Return      : Nothing
*******************************************************************************/
static void Bank_Program2(
    UINT32 dwSize,
    UINT8  ch,
    PEVLIST pEv
)
{
    UINT8  i;

    for (i = 0; i < 16; i++) {
        if ((Voice_Info[i].bBank == ChInfo[ch].bBankM) &&
            (Voice_Info[i].bProg == ChInfo[ch].bPgm))
            break;
    }

    if (i == 16) {
        ChInfo[ch].bBank  = 0;
        ChInfo[ch].bBankM = 0;
        ChInfo[ch].bPgm   = 0;
    }
    else {
        ChInfo[ch].bBank  = 1;
        ChInfo[ch].bPgm   = i;
    }

    if (pEv)
        Ev_Set(MASNDDRV_CMD_PROGRAM_CHANGE, ch, ChInfo[ch].bBank,
            ChInfo[ch].bPgm, dwSize, pEv);
}


/*******************************************************************************
    Name        : Direct_PitchBend2
    Description : MA-1/2 Pitch Bend Message
    Parameter   : UINT8  *pEx     ... Pointer to Exclusive Message
                : UINT8  bTrackNo ... Score Track #
                : struct *pEv     ... Pointer to Event List
    Return      : Nothing
*******************************************************************************/
static void Direct_PitchBend2(
    UINT8  *pEx,
    UINT8  bTrackNo,
    PEVLIST pEv
)
{
    UINT8  ch, ev;
    UINT32 dwFnum;
    POFFLIST pOff;

    if ((pEx[0] != 0x90) || (pEx[3] != 0xF7)){
        Ev_Set(MASNDDRV_CMD_NOP, 0, 0, 0, dwSeqSize[bTrackNo], pEv);
        return ;
    }
    ev = (UINT8)(pEx[1] & 0xF0);
    ch = (UINT8)(pEx[1] & 0x0F);
    if (ev == 0xB0) {
        ChInfo[ch].bRpnL = pEx[2];
        ev = 0xC0;
    }
    else if (ev == 0xC0) {
        ChInfo[ch].bRpnM = pEx[2];
        ev = 0xB0;
    }
    else {
        Ev_Set(MASNDDRV_CMD_NOP, 0, 0, 0, dwSeqSize[bTrackNo], pEv);
        return ;
    }

    if (dwSeqSize[bTrackNo] > 10) {             /* Next Event Check           */
        pEx = pSeq[bTrackNo];
        if ((pEx[0]==0x00) &&                   /* Duration '0'               */
            (pEx[1]==0xFF) && (pEx[2]==0xF0) && /* Exclusive Event            */
            (pEx[3]==0x06) && (pEx[4]==0x43) && /* YAMAHA Exclusive Message   */
            ((pEx[5]==0x02)||(pEx[5]==0x03)) && /* MA-1/2 Exclusive Message   */
            (pEx[6] == 0x90) &&                 /* Pitch Bend Control         */
            ((pEx[7] & 0x0F) == ch) &&          /* Same Channel               */
            ((pEx[7] & 0xF0) == ev)) {          /* Pair Message               */
            if (ev == 0xB0)
                ChInfo[ch].bRpnL = pEx[8];
            else
                ChInfo[ch].bRpnM = pEx[8];
            pSeq[bTrackNo]      += 10;
            dwSeqSize[bTrackNo] -= 10;
        }
    }

    dwFnum = ((ChInfo[ch].bRpnM & 0x1F) << 8) + ChInfo[ch].bRpnL + 0x80000000;

    if (ChInfo[ch].bRpnM & 0x20)
        Ev_Set(MASNDDRV_CMD_NOTE_ON_MA2EX, ch, dwFnum, 127, dwSeqSize[bTrackNo],
            pEv);
    else {
        pOff = Get_OffList();
        if (pOff != 0) {
            pOff->dwTime = pEv->dwTime;
            pOff->dwKey  = dwFnum;
            pOff->dwCh   = ch;
            pOff->dwType = MASNDDRV_CMD_NOTE_OFF_MA2EX;
            Set_OffList(pOff);
        }
        Ev_Set(MASNDDRV_CMD_NOP, 0, 0, 0, dwSeqSize[bTrackNo], pEv);
    }
}


/*******************************************************************************
    Name        : NextEvent2_SeekS
    Description : Seek Process (FM Sequence)
    Parameter   : UINT8  trackno  ... Track #
    Return      : Nothing
*******************************************************************************/
static void NextEvent2_SeekS(
    UINT8 bTrackNo
)
{
    UINT8  bData0, bData1, value, chno;
    UINT8  *pEx;

    bData0 = *pSeq[bTrackNo];                   /* Event Message              */
    dwSeqSize[bTrackNo]--;
    pSeq[bTrackNo] ++;
    if (bData0 == 0x00) {                       /* Control Message            */
        bData0 = *pSeq[bTrackNo];               /* Event Message              */
        dwSeqSize[bTrackNo]--;
        pSeq[bTrackNo] ++;
        chno  = (UINT8)((bData0 >> 6) & 0x03) ;
        if (bTrackNo >= 2)
            chno = (UINT8)(chno + ((bTrackNo - 1) * 4));

        if (( bData0 & 0x30 ) != 0x30){         /* 2 Byte Event               */
            value = (UINT8)(bData0 & 0x0F);
            bData1 = (UINT8)((bData0 >> 4) & 0x03);
            if (bData1 == 0x00) {               /* Expression                 */
                if(_exp2[value] >= 0x80)
                    return;
                ChInfo[chno].bExpression = _exp2[value];
            }
            else if (bData1 == 0x02) {          /* Modulation                 */
                if(_mod2[value] >= 0x80)
                    return;
                if      (_mod2[value] == 0x00) value = 0x00;
                else if (_mod2[value] <= 0x10) value = 0x01;
                else if (_mod2[value] <= 0x30) value = 0x02;
                else if (_mod2[value] <= 0x50) value = 0x03;
                else                           value = 0x04;
                ChInfo[chno].bModulation = value;
            }
        }
        else {                                  /* 3 Byte Event               */
            bData1 = (UINT8)(bData0 & 0x0F);    /* Event No.                  */
            value = *pSeq[bTrackNo];
            dwSeqSize[bTrackNo]--;
            pSeq[bTrackNo] ++;
            if (bData1 == 0x00) {               /* Program Change             */
                ChInfo[chno].bPgm = value;
                Bank_Program2(0, chno, 0);
            }
            else if (bData1 == 0x01)            /* Bank Select                */
                ChInfo[chno].bBankM = value;
            else if (bData1 == 0x02) {          /* Octave Shift               */
                if (value <= 0x04)
                    value = (UINT8)(value + 4);
                else if ((0x81 <= value) && (value <= 0x84))
                    value = (UINT8)(0x84 - value);
                else
                    value = ChInfo[chno].bType;
                ChInfo[chno].bType = value;
            }
            else if (bData1 == 0x03) {          /* Modulation                 */
                if(value >= 0x80)
                    return ;
                if (value == 0x00)      value = 0x00;
                else if (value <= 0x10) value = 0x01;
                else if (value <= 0x30) value = 0x02;
                else if (value <= 0x50) value = 0x03;
                else                    value = 0x04;
                ChInfo[chno].bModulation = value;
            }
            else if (bData1 == 0x07) {          /* Volume                     */
                if (value >= 0x80)
                    return;
                ChInfo[chno].bVolume = value;
            }
            else if (bData1 == 0x0A) {          /* Pan                        */
                if(value >= 0x80)
                    return;
                 ChInfo[chno].bPanpot = value;
            }
            else if (bData1 == 0x0B) {          /* Expression                 */
                if(value >= 0x80)
                    return;
                ChInfo[chno].bExpression = value;
            }
        }
    }                                           /* Control Message            */
    else if (bData0 == 0xFF) {                  /* Exclusive Message or NOP   */
        bData0 = *pSeq[bTrackNo];               /* Event Message              */
        dwSeqSize[bTrackNo]--;
        pSeq[bTrackNo] ++;
        if ( bData0 != 0xF0 )                   /* NOP                        */
            return ;
        bData0 = *pSeq[bTrackNo];               /* Exclusive Message          */
        pEx = pSeq[bTrackNo];
        if ((bData0 == 6) && (pEx[1] == 0x43) &&
            ((pEx[2] == 0x02) || (pEx[2] == 0x03)) && (pEx[3] == 0x90)) {
            chno = (UINT8)(pEx[4] & 0x0F);
            if ((pEx[4] & 0xF0) == 0xB0)
                ChInfo[chno].bRpnL = pEx[5];
            else if ((pEx[4] & 0xF0) == 0xC0)
                ChInfo[chno].bRpnM = pEx[5];
        }
        dwSeqSize[bTrackNo] -= (bData0 + 1);    /* Skip Massege               */
        pSeq[bTrackNo]      += (bData0 + 1);    /* Skip Massege               */
    }
    else {                                      /* Note Message               */
        if (*pSeq[bTrackNo] >= 128) {
            dwSeqSize[bTrackNo]--;
            pSeq[bTrackNo] ++;
        }
        dwSeqSize[bTrackNo]--;
        pSeq[bTrackNo] ++;
    }
}


/*******************************************************************************
    Name        : NextEvent2_PlayS
    Description : Set Next Event (FM Sequence)
    Parameter   : struct *pEv    ... Pointer to Event List
                : UINT8  trackno ... Track #
    Return      : Nothing
*******************************************************************************/
static void NextEvent2_PlayS(
    PEVLIST pEv,
    UINT8 bTrackNo
)
{
    UINT8  bData0, bData1, value, chno, oct, key;
    UINT8  *pEx;
    UINT32 dwGateTime;

    bData0 = *pSeq[bTrackNo];                   /* Event Message              */
    dwSeqSize[bTrackNo]--;
    pSeq[bTrackNo] ++;
    if (bData0 == 0x00) {                       /* Control Message            */
        bData0 = *pSeq[bTrackNo];               /* Event Message              */
        dwSeqSize[bTrackNo]--;
        pSeq[bTrackNo] ++;
        chno  = (UINT8)((bData0 >> 6) & 0x03) ;
        if (bTrackNo >= 2)
            chno = (UINT8)(chno + ((bTrackNo - 1) * 4));

        if (( bData0 & 0x30 ) != 0x30){         /* 2 Byte Event               */
            value = (UINT8)(bData0 & 0x0F);
            bData1 = (UINT8)((bData0 >> 4) & 0x03);
            if (bData1 == 0x00) {               /* Expression                 */
                if(_exp2[value] >= 0x80)
                    Ev_Set(MASNDDRV_CMD_NOP, 0, 0, 0, dwSeqSize[bTrackNo], pEv);
                else
                    Ev_Set(MASNDDRV_CMD_EXPRESSION, chno, _exp2[value], 0,
                        dwSeqSize[bTrackNo], pEv);
                return ;
            }
            else if (bData1 == 0x02) {          /* Modulation                 */
                if(_mod2[value] >= 0x80) {
                    Ev_Set(MASNDDRV_CMD_NOP, 0, 0, 0, dwSeqSize[bTrackNo], pEv);
                    return;
                }
                if      (_mod2[value] == 0x00) value = 0x00;
                else if (_mod2[value] <= 0x10) value = 0x01;
                else if (_mod2[value] <= 0x30) value = 0x02;
                else if (_mod2[value] <= 0x50) value = 0x03;
                else                           value = 0x04;
                Ev_Set(MASNDDRV_CMD_MODULATION_DEPTH, chno, value, 0,
                    dwSeqSize[bTrackNo], pEv);
                return ;
            }
            else {
                Ev_Set(MASNDDRV_CMD_NOP, 0, 0, 0, dwSeqSize[bTrackNo], pEv);
                return ;
            }
        }
        else {                                  /* 3 Byte Event               */
            bData1 = (UINT8)(bData0 & 0x0F);    /* Event No.                  */
            value = *pSeq[bTrackNo];
            dwSeqSize[bTrackNo]--;
            pSeq[bTrackNo] ++;
            if (bData1 == 0x00) {               /* Program Change             */
                ChInfo[chno].bPgm = value;
                Bank_Program2(dwSeqSize[bTrackNo] ,chno, pEv);
                return ;
            }
            else if (bData1 == 0x01) {          /* Bank Select                */
                ChInfo[chno].bBankM = value;
                Ev_Set(MASNDDRV_CMD_NOP, 0, 0, 0, dwSeqSize[bTrackNo], pEv);
                return ;
            }
            else if (bData1 == 0x02) {          /* Octave Shift               */
                if (value <= 0x04)
                    value = (UINT8)(value + 4);
                else if ((0x81 <= value) && (value <= 0x84))
                    value = (UINT8)(0x84 - value);
                else
                    value = ChInfo[chno].bType;
                ChInfo[chno].bType = value;
                Ev_Set(MASNDDRV_CMD_NOP, 0, 0, 0, dwSeqSize[bTrackNo], pEv);
                return ;
            }
            else if (bData1 == 0x03) {          /* Modulation                 */
                if(value >= 0x80) {
                    Ev_Set(MASNDDRV_CMD_NOP, 0, 0, 0, dwSeqSize[bTrackNo], pEv);
                    return ;
                }
                if (value == 0x00)      value = 0x00;
                else if (value <= 0x10) value = 0x01;
                else if (value <= 0x30) value = 0x02;
                else if (value <= 0x50) value = 0x03;
                else                    value = 0x04;
                Ev_Set(MASNDDRV_CMD_MODULATION_DEPTH, chno, value, 0,
                    dwSeqSize[bTrackNo], pEv);
                return ;
            }
            else if (bData1 == 0x07) {          /* Volume                     */
                if (value >= 0x80) {
                    Ev_Set(MASNDDRV_CMD_NOP, 0, 0, 0, dwSeqSize[bTrackNo], pEv);
                    return ;
                }
                Ev_Set(MASNDDRV_CMD_CHANNEL_VOLUME, chno, value, 0,
                    dwSeqSize[bTrackNo], pEv);
                return ;
            }
            else if (bData1 == 0x0A) {          /* Pan                        */
                if (value >= 0x80) {
                    Ev_Set(MASNDDRV_CMD_NOP, 0, 0, 0, dwSeqSize[bTrackNo], pEv);
                    return ;
                }
                Ev_Set(MASNDDRV_CMD_PANPOT, chno, value, 0, dwSeqSize[bTrackNo],
                    pEv);
                return ;
            }
            else if (bData1 == 0x0B) {          /* Expression                 */
                if(value >= 0x80) {
                    Ev_Set(MASNDDRV_CMD_NOP, 0, 0, 0, dwSeqSize[bTrackNo], pEv);
                    return ;
                }
                Ev_Set(MASNDDRV_CMD_EXPRESSION, chno, value, 0,
                    dwSeqSize[bTrackNo], pEv);
                return ;
            }
            else {
                Ev_Set(MASNDDRV_CMD_NOP, 0, 0, 0, dwSeqSize[bTrackNo], pEv);
                return ;
            }
        }
    }                                           /* Control Message            */
    else if (bData0 == 0xFF) {                  /* Exclusive Message or NOP   */
        bData0 = *pSeq[bTrackNo];               /* Event Message              */
        dwSeqSize[bTrackNo]--;
        pSeq[bTrackNo] ++;
        if ( bData0 != 0xF0 ) {                 /* NOP                        */
            Ev_Set(MASNDDRV_CMD_NOP, 0, 0, 0, dwSeqSize[bTrackNo], pEv);
            return ;
        }
        bData0 = *pSeq[bTrackNo];               /* Exclusive Message          */
        pEx = pSeq[bTrackNo];
        dwSeqSize[bTrackNo] -= (bData0 + 1);
        pSeq[bTrackNo]      += (bData0 + 1);
        pEx ++;
        if ((pEx[0] != 0x43) || ((pEx[1] != 0x02) && (pEx[1] != 0x03))) {
            Ev_Set(MASNDDRV_CMD_NOP, 0, 0, 0, dwSeqSize[bTrackNo], pEv);
            return ;
        }
        Direct_PitchBend2(&(pEx[2]), bTrackNo, pEv);
    }
    else {                                      /* Note Message               */
        chno = (UINT8)((bData0 >> 6) & 0x03);
        if (bTrackNo >= 2)
            chno = (UINT8)(chno + ((bTrackNo - 1) * 4));
        oct = (UINT8)((bData0 >> 4) & 0x03);
        key = (UINT8)(bData0 & 0x0F);
        dwGateTime = get_flex2(pSeq[bTrackNo], dwSeqSize[bTrackNo]);
        if (dwGateTime >= 128) {
            dwSeqSize[bTrackNo]--;
            pSeq[bTrackNo] ++;
        }
        dwSeqSize[bTrackNo]--;
        pSeq[bTrackNo] ++;
        Note_ON2(chno, oct, key, dwGateTime, pEv, dwSeqSize[bTrackNo]);
    }
}


/*******************************************************************************
    Name        : NextEvent2_SeekA
    Description : Seek Process (Audio Sequence)
    Parameter   : UINT8  bTrackno ... Track #
    Return      : Nothing
*******************************************************************************/
static void NextEvent2_SeekA(
    UINT8 bTrackNo
)
{
    UINT8  bData0, bData1, value;

    bData0 = *pSeq[bTrackNo];                   /* Event Message              */
    dwSeqSize[bTrackNo]--;
    pSeq[bTrackNo] ++;
    if (bData0 == 0x00) {                       /* Control Message            */
        bData0 = *pSeq[bTrackNo];               /* Event Message              */
        dwSeqSize[bTrackNo]--;
        pSeq[bTrackNo] ++;
        if (( bData0 & 0x30 ) != 0x30)          /* 2 Byte Event               */
            return ;
        else {                                  /* 3 Byte Event               */
            bData1 = (UINT8)(bData0 & 0x0F);    /* Event No.                  */
            value = *pSeq[bTrackNo];
            dwSeqSize[bTrackNo]--;
            pSeq[bTrackNo] ++;
            if (bData1 == 0x07) {               /* Volume                     */
                if ((value >= 0x80) || ((bData0 & 0xC0) != 0))
                    return;
                bAudio_Vel = value;
            }
            else
                return ;
        }
    }                                           /* Control Message            */
    else if (bData0 == 0xFF) {                  /* Exclusive Message or NOP   */
        bData0 = *pSeq[bTrackNo];               /* Event Message              */
        dwSeqSize[bTrackNo]--;
        pSeq[bTrackNo] ++;
        if ( bData0 != 0xF0 )                   /* NOP                        */
            return ;
        bData0 = *pSeq[bTrackNo];               /* Exclusive Message          */
        dwSeqSize[bTrackNo] -= (bData0 + 1);    /* Skip Massege               */
        pSeq[bTrackNo]      += (bData0 + 1);    /* Skip Massege               */
    }
    else {                                      /* Note Message               */
        if (*pSeq[bTrackNo] >= 128) {
            dwSeqSize[bTrackNo]--;
            pSeq[bTrackNo] ++;
        }
        dwSeqSize[bTrackNo]--;
        pSeq[bTrackNo] ++;
    }
}


/*******************************************************************************
    Name        : NextEvent2_PlayA
    Description : Set Next Event (Audio Sequence)
    Parameter   : struct *pEv    ... Pointer to Event List
                : UINT8  trackno ... Track #
    Return      : Nothing
*******************************************************************************/
static void NextEvent2_PlayA(
    PEVLIST pEv,
    UINT8 bTrackNo
)
{
    UINT8  bData0, bData1, value, key;
    UINT32 dwGateTime;

    bData0 = *pSeq[bTrackNo];                   /* Event Message              */
    dwSeqSize[bTrackNo]--;
    pSeq[bTrackNo] ++;
    if (bData0 == 0x00) {                       /* Control Message            */
        bData0 = *pSeq[bTrackNo];               /* Event Message              */
        dwSeqSize[bTrackNo]--;
        pSeq[bTrackNo] ++;
        if (( bData0 & 0x30 ) != 0x30) {        /* 2 Byte Event               */
            Ev_Set(MASNDDRV_CMD_NOP, 0, 0, 0, dwSeqSize[bTrackNo], pEv);
            return ;
        }
        else {                                  /* 3 Byte Event               */
            bData1 = (UINT8)(bData0 & 0x0F);    /* Event No.                  */
            value = *pSeq[bTrackNo];
            dwSeqSize[bTrackNo]--;
            pSeq[bTrackNo] ++;
            if (bData1 == 0x07) {               /* Volume                     */
                if ((value < 0x80) && ((bData0 & 0xC0) == 0))
                    bAudio_Vel = value;
            }
            Ev_Set(MASNDDRV_CMD_NOP, 0, 0, 0, dwSeqSize[bTrackNo], pEv);
            return ;
        }
    }                                           /* Control Message            */
    else if (bData0 == 0xFF) {                  /* Exclusive Message or NOP   */
        bData0 = *pSeq[bTrackNo];               /* Event Message              */
        dwSeqSize[bTrackNo]--;
        pSeq[bTrackNo] ++;
        if ( bData0 != 0xF0 ) {                 /* NOP                        */
            Ev_Set(MASNDDRV_CMD_NOP, 0, 0, 0, dwSeqSize[bTrackNo], pEv);
            return ;
        }
        bData0 = *pSeq[bTrackNo];               /* Exclusive Message          */
        dwSeqSize[bTrackNo] -= (bData0 + 1);
        pSeq[bTrackNo]      += (bData0 + 1);
        Ev_Set(MASNDDRV_CMD_NOP, 0, 0, 0, dwSeqSize[bTrackNo], pEv);
        return ;
    }
    else {                                      /* Note Message               */
        key = (UINT8)(bData0 & 0x3F);
        dwGateTime = get_flex2(pSeq[bTrackNo], dwSeqSize[bTrackNo]);
        if (dwGateTime >= 128) {
            dwSeqSize[bTrackNo]--;
            pSeq[bTrackNo] ++;
        }
        dwSeqSize[bTrackNo]--;
        pSeq[bTrackNo] ++;
		if ((bData0 & 0xC0) == 0)
	        Stream_ON2(key, dwGateTime, pEv, dwSeqSize[bTrackNo]);
		else
        	Ev_Set(MASNDDRV_CMD_NOP, 0, 0, 0, dwSeqSize[bTrackNo], pEv);
    }
}


/*******************************************************************************
    Name        : SetPreEvent2
    Description : Collect Event Information (before Seek Time)
    Parameter   : Nothing
    Return      : Nothing
*******************************************************************************/
static void SetPreEvent2( void )
{
    UINT8  i, j;
    UINT32 dwDuration, dwPastTime;
    UINT32 dwStartTime;
    PEVLIST pEvList;

    Play_Info.dwPastTime = 0;

    if (dwSeekTick >= (dwEndTick - Smaf_Info[1].dwStartTime)) {
        dwEventFlag &= ~(UINT32)EVENT_LED_ON;
        dwEventFlag &= ~(UINT32)EVENT_MOTOR_ON;
        dwEventFlag |=  (UINT32)EVENT_SYSTEM_ON;
        Play_Info.bPreEvent = 2;
        Play_Info.dwPastTime = dwEndTick;
        return;
    }

    Play_Info.wPitchBendFlag = 0;
    for (i = 0; i < 16; i++) {
        ChInfo[i].bType       = 4;
        ChInfo[i].bSlotNum    = 0;
        ChInfo[i].bVel        = VelocityTable[64];
        ChInfo[i].bBank       = MMF_Bank_No;
        ChInfo[i].bBankM      = MMF_Bank_Select_MSB;
        ChInfo[i].bBankL      = MMF_Bank_Selsct_LSB;
        ChInfo[i].bPgm        = MMF_Program_No;
        ChInfo[i].bRpnM       = 0;
        ChInfo[i].bRpnL       = 0;
        ChInfo[i].bModulation = MMF_Modulation_Depth;
        ChInfo[i].bVolume     = MMF_Channel_Volume;
        ChInfo[i].bPanpot     = MMF_Channel_Panpot;
        ChInfo[i].bExpression = MMF_Expression;
        ChInfo[i].bHold       = MMF_Dumper_Hold;
        ChInfo[i].bPitchSens  = MMF_Pitch_Bend_Sensitivity;
        ChInfo[i].bMonoPoly   = MMF_Channel_Mode;
        ChInfo[i].bPitchM     = MMF_Pitch_Bend_MSB;
        ChInfo[i].bPitchL     = MMF_Pitch_Bend_LSB;
    }

    for (i = 0; i < 32; i++)
        Wave_Info3[i].bPan = MMF_Wave_Panpot;

    bAudio_Slot = 0;
    bAudio_Vel  = 0x63;
    dwStartTime  = Smaf_Info[1].dwStartTime + dwSeekTick;

    for (j = 0; j < 5; j++) {
        pSeq[j] = 0;
        if (Smaf_Info[1].ScoreTrack[j].pMTR == 0)
            continue;

        if (Smaf_Info[1].ScoreTrack[j].Phrase[0].dwSPp != 0xFFFFFFFF)
            dwSeqSize[j] = Smaf_Info[1].ScoreTrack[j].Phrase[0].dwSPp;
        else
            dwSeqSize[j] = Smaf_Info[1].ScoreTrack[j].dwSeqSize;
        pSeq[j]      = Smaf_Info[1].ScoreTrack[j].pSeq;
        dwPastTime   = 0;

        dwDuration   = get_flex2(pSeq[j], dwSeqSize[j]);
        if (dwDuration >= 128) {
            dwSeqSize[j] --;
            pSeq[j]      ++;
        }
        dwSeqSize[j] --;
        pSeq[j]      ++;
        while ((dwPastTime + dwDuration) < dwStartTime) {
            dwPastTime += dwDuration;
            NextEvent2_SeekS(j);
            dwDuration = get_flex2(pSeq[j], dwSeqSize[j]);
            if (dwDuration == 0xFFFFFFFF)
                break;
            if (dwDuration >= 128) {
                dwSeqSize[j] --;
                pSeq[j]      ++;
            }
            dwSeqSize[j] --;
            pSeq[j]      ++;
        }
        if (dwDuration == 0xFFFFFFFF)           /* Track EOS                  */
            continue;                           /* Skip Event Registration    */
        pEvList = Get_EvList();
        pEvList->dwTime   = dwPastTime + dwDuration;
        pEvList->bTrackNo = j;
        NextEvent2_PlayS(pEvList, j);
        Set_EvList(pEvList);
    }

    if (Smaf_Info[1].AudioTrack.pATR) {
        j = AUDIO_TRACK_NO;

        if (Smaf_Info[1].AudioTrack.Phrase.dwSPp != 0xFFFFFFFF)
            dwSeqSize[j] = Smaf_Info[1].AudioTrack.Phrase.dwSPp;
        else
            dwSeqSize[j] = Smaf_Info[1].AudioTrack.dwSeqSize;
        pSeq[j]      = Smaf_Info[1].AudioTrack.pSeq;
        dwPastTime = 0;

        dwDuration = get_flex2(pSeq[j], dwSeqSize[j]);
        if (dwDuration >= 128) {
            dwSeqSize[j] --;
            pSeq[j]      ++;
        }
        dwSeqSize[j] --;
        pSeq[j]      ++;

        while ((dwPastTime + dwDuration) < dwStartTime) {
            dwPastTime += dwDuration;
            NextEvent2_SeekA(j);
            dwDuration = get_flex2(pSeq[j], dwSeqSize[j]);
            if (dwDuration == 0xFFFFFFFF)
                break;
            if (dwDuration >= 128) {
                dwSeqSize[j] --;
                pSeq[j]      ++;
            }
            dwSeqSize[j] --;
            pSeq[j]      ++;
        }

        if (dwDuration != 0xFFFFFFFF) {
            pEvList = Get_EvList();
            pEvList->dwTime   = dwPastTime + dwDuration;
            pEvList->bTrackNo = AUDIO_TRACK_NO;
            NextEvent2_PlayA(pEvList, j);
            Set_EvList(pEvList);
        }
    }

    Play_Info.dwPastTime = dwStartTime;

        dwEventFlag &= ~(UINT32)EVENT_LED_ON;
        dwEventFlag &= ~(UINT32)EVENT_MOTOR_ON;
        dwEventFlag |=  (UINT32)EVENT_SYSTEM_ON;
}


/*******************************************************************************
    Name        : GetContentsData2
    Description : Get Contents Information (Option Data Type MA-1/2)
    Parameter   : SINT32 file_id ... File ID
                : struct pinf    ... Pointer to Structure
    Return      : >=0: Success
                : < 0: Error
*******************************************************************************/
static SINT32 GetContentsData2(
    SINT32 file_id,
    PMASMW_CONTENTSINFO pinf
)
{
    UINT8  *pBuf;
    UINT32 dwSize;
    SINT32 slTemp;
    UINT32 i;
    UINT8  flag1, flag2, bEscape;
    UINT8  bData;
	UINT8  bCode;

    pBuf   = Smaf_Info[file_id].pOPDA;
    dwSize = Smaf_Info[file_id].dwOPDASize;
    slTemp = 0;
    i      = 0;

    if ((dwSize < 4) || ((pinf->code != CONTENTS_CODE_BINARY) &&
        (Smaf_Info[file_id].pFile[18] != pinf->code)))
        return MMFCNV_ERR_TAG;

	bCode = Smaf_Info[file_id].pFile[18];

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
    return MMFCNV_ERR_TAG;
}


/*--- SAMF MA-1/2 Function ---------------------------------------------------*/
/*--- SMAF MA-3   Function ---------------------------------------------------*/

/*******************************************************************************
    Name        : CNTI_Check3
    Description : Contents Info Chunk Check
    Parameter   : UINT8  *pBuffer ... Pointer to CNTI
    Return      :  <0 ... Error
                : =>0 ... Success
*******************************************************************************/
static SINT32 CNTI_Check3(
    UINT8  *pBuffer
)
{
    UINT8  bTemp;
    SINT32 result;

    bTemp = pBuffer[0];                         /* Contents Class             */
    if ((bTemp != CONTENTS_CLASS_1) && (bTemp != CONTENTS_CLASS_2))
        return MMFCNV_ERR_C_CLASS;
    bTemp = pBuffer[1];                         /* Contents Type              */
    if (((bTemp & 0xF0)!= CONTENTS_TYPE_4) &&
        ((bTemp & 0xF0)!= CONTENTS_TYPE_5) &&
        ((bTemp & 0xF0)!= CONTENTS_TYPE_6))
        result = MMFCNV_ERR_C_TYPE;
    else {
        switch (bTemp & 0x0F) {
            case 2:
                result = 16;
                break;
            case 3:
                result = 32;
                break;
            default:
                result = MMFCNV_ERR_C_TYPE;
                break;
        }
    }
    return result;
}


/*******************************************************************************
    Name        : Decode_Getbit
    Description : Get 1 bit
    Parameter   : UINT8 no ... Array No.
    Return      : Result
*******************************************************************************/
static SINT32  Decode_Getbit(
    UINT8 no
)
{
    if(--Decode_count[no] >= 0)
        return (Decode_bitbuf[no] & bit_mask[Decode_count[no]]);
    Decode_count[no] = 7;
    Decode_bitbuf[no] = *(pDecodeBuffer[no]++);
    return (Decode_bitbuf[no] & 0x80);
}


/*******************************************************************************
    Name        : Decode_Getbits
    Description : Get 1 Byte (Convert 8 bits to 1 Byte)
    Parameter   : UINT8 no ... Array No.
    Return      : Result
*******************************************************************************/
static UINT8  Decode_Getbits(
    UINT8 no
)
{
    SINT32 x;
    SINT32 n;

    x = 0;
    n = 8;
    while (n > Decode_count[no]){
        n -= Decode_count[no];
        x |= rightbits(Decode_count[no], Decode_bitbuf[no]) << n;
        Decode_bitbuf[no] = *(pDecodeBuffer[no]++);
        Decode_count[no] = 8;
    }
    Decode_count[no] -= n;

    return (UINT8)(x | rightbits(n, (Decode_bitbuf[no] >> Decode_count[no])));
}


/*******************************************************************************
    Name        : Decode_tree
    Description : Make Huffman Decode Tree
    Parameter   : UINT8 no   ... Array No.
    Return      : Nothing
*******************************************************************************/
static void Decode_tree(
    UINT8 no
)
{
    UINT32 i, put, empty;
    SINT16 wTemp;
    SINT16 *left, *right, *oya;

    if (Decode_Getbit(no)) {
        left  = &(Decode_left[no][256]);
        right = &(Decode_right[no][256]);
        oya   = &(Decode_right[no][0]);
        for (i = 0; i < 256; i ++) {
            left[i]  = -1;
            right[i] = -1;
            oya[i] = 0;
        }
        i = 2;
        put = 0;
        empty = 1;
    }
    else {
        i = 0;
        Decode_left[no][256] = (SINT16)Decode_Getbits(no);
        return ;
    }
    while (i != 0) {
        if (Decode_Getbit(no)) {
            i ++;
            if (left[put] != -1) {
                right[put] = (SINT16)(empty + 256);
                oya[empty] = (UINT16)put;
            }
            else {
                left[put]  = (SINT16)(empty +256);
                oya[empty] = (UINT16)put;
            }
            put = empty;
            empty ++;
        }
        else {
            i --;
            wTemp = Decode_Getbits(no);
            if (left[put] != -1) {
                right[put] = wTemp;
                while ((right[put] != -1) && (put != 0))
                    put = oya[put];
            }
            else
                left[put]  = wTemp;
        }
    }

    for (i = 256; i < 512; i++) {
        if (Decode_left[no][i] == -1)
            Decode_left[no][i]  = 0;
        if (Decode_right[no][i] == -1)
            Decode_right[no][i] = 0;
    }
}


/*******************************************************************************
    Name        : Decode_init
    Description : Get Ready for Huffman Decode
    Parameter   : UINT8 no ... Array No.
    Return      : UINT32 ... Data Size of After Decode
*******************************************************************************/
static UINT32 Decode_init(
    UINT8 no
)
{
    UINT32 dwSize;

    dwSize = _get4b(&pDecodeBuffer[no]);        /* Get Size of After Decode   */
    Decode_bitbuf[no] = 0;
    Decode_count[no] = 0;
    Decode_tree(no);                            /* Make Huffman Decode Tree   */

    return dwSize;
}


/*******************************************************************************
    Name        : Decode_byte
    Description : Decode Function
    Parameter   : UINT8 no    ... Array No.
    Return      : UINT8 ... Result
*******************************************************************************/
static UINT8  Decode_byte(
    UINT8 no
)
{
    SINT32 j;
	SINT32 d;

    if (bCompress[no])                          /* Normal Mode                */
        return *(pDecodeBuffer[no]++);
    else {                                      /* Compress Mode              */
        j = 256;                                /* Huffman Decode             */
        while(j >= 256){
		    if(--Decode_count[no] >= 0)
				d = Decode_bitbuf[no] & bit_mask[Decode_count[no]];
			else {
				Decode_count[no] = 7;
				Decode_bitbuf[no] = *(pDecodeBuffer[no]++);
    			d = Decode_bitbuf[no] & 0x80;
			}
            if( d )
                j = Decode_right[no][j];
            else
                j = Decode_left[no][j];
        }
    }
    return (UINT8)(j);
}


/*******************************************************************************
    Name        : Decode_byte7bit
    Description : Decode Function
    Parameter   : UINT8 no    ... Array No.
    Return      : UINT8 ... Result(Masked Byte)
*******************************************************************************/
static UINT8  Decode_byte7bit(
    UINT8 no
)
{
    SINT32 j;
	SINT32 d;

    if (bCompress[no])                          /* Normal Mode                */
        return (UINT8)(*(pDecodeBuffer[no]++)&0x7F);
    else {                                      /* Compress Mode              */
        j = 256;                                /* Huffman Decode             */
        while(j >= 256){
		    if(--Decode_count[no] >= 0)
				d = Decode_bitbuf[no] & bit_mask[Decode_count[no]];
			else {
				Decode_count[no] = 7;
				Decode_bitbuf[no] = *(pDecodeBuffer[no]++);
    			d = Decode_bitbuf[no] & 0x80;
			}
            if( d )
                j = Decode_right[no][j];
            else
                j = Decode_left[no][j];
        }
    }
    return (UINT8)(j&0x7F);
}


/*******************************************************************************
    Name        : get_flex3
    Description : Get Data (Variable Length Data)
    Parameter   : UINT32 dwDataSize ... Remain Data Size
                : UINT8  no         ... Array No.
                : UINT8  *j         ... Return Read Data Size
    Return      : >=0    ... Success
                : -1     .. .Error
*******************************************************************************/
static SINT32 get_flex3(
    UINT32 dwDataSize,
    UINT8  no,
    UINT8  *j
)
{
    UINT8  bData;
    UINT32 dwFlex;

    if(dwDataSize == 0)                         /* ErrorCheck(Remain Size > 0)*/
        return -1;

	if( bCompress[no] ) {                       /* Normal Mode                */
      bData = *(pDecodeBuffer[no]++);           /* 1 Byte Get                 */
      *j = 1;                                   /* Read Size Counter ++       */
      dwFlex = (UINT32)(bData & 0x7F);          /* Registration Return Value  */
      while(bData & 0x80){                      /* Next Byte ?                */
        dwDataSize--;
        if((dwDataSize == 0) || (*j >= 4))      /* ErrorCheck(Remain Size > 0)*/
            return -1;
        bData = *(pDecodeBuffer[no]++);         /* 1 Byte Get                 */
        *j += 1;                                /* Read Size Counter ++       */
        dwFlex = (dwFlex << 7) + (UINT32)(bData & 0x7F);
      }                                         /* Registration Return Value  */
    }
    else {                                      /* Compress Mode              */
      bData = Decode_byte(no);                  /* 1 Byte Get                 */
      *j = 1;                                   /* Read Size Counter ++       */
      dwFlex = (UINT32)(bData & 0x7F);          /* Registration Return Value  */
      while(bData & 0x80){                      /* Next Byte ?                */
        dwDataSize--;
        if((dwDataSize == 0) || (*j >= 4))      /* ErrorCheck(Remain Size > 0)*/
            return -1;
        bData = Decode_byte(no);                /* 1 Byte Get                 */
        *j += 1;                                /* Read Size Counter ++       */
        dwFlex = (dwFlex << 7) + (UINT32)(bData & 0x7F);
      }                                         /* Registration Return Value  */
	}
    return (SINT32)dwFlex;                      /* SUCCESS                    */
}


/*******************************************************************************
    Name        : CastAwayNByte
    Description : Skip Data
    Parameter   : UINT32 dwDataSize ... Pointer to Remain Data Size
                : UINT32 n          ... Number of Skip
                : UINT8  no         ... Array No.
    Return      : >=0 ... Success
                :  -1 ... Data Size Over (Error)
*******************************************************************************/
static SINT16 CastAwayNByte(
    UINT32 dwDataSize,
    UINT32 n,
    UINT8  no
)
{
	UINT8 d;

    if(dwDataSize < n)
        return -1;

	n--;
	if( bCompress[no] ) {
        pDecodeBuffer[no] += n;
        return (SINT16)(*(pDecodeBuffer[no]++));
    }
	else {
    	while(n--) {
        	d = Decode_byte(no);
		}
    	return (SINT16)(Decode_byte(no));
    }
}


/*******************************************************************************
    Name        : SeqData_Check3
    Description : Check Sequence Data Chunk
    Parameter   : struct *pMTR ... Pointer to Track Chunk Information
                : UINT8  no    ... Load Mode
    Return      :  >0: Error
                :   0: Success
*******************************************************************************/
static UINT8 SeqData_Check3(
    PMTRINFO pMTR,
    UINT8  no
)
{
    UINT8  *pBuf;                               /* Pointer to Data Buffer     */
    UINT32 dwSize;                              /* Seqence Data Size          */
    UINT32 i;                                   /* Reference Point            */
    UINT32 dwGate;                              /* Gate Time                  */
    UINT32 dwTime;                              /* Past Time                  */
    SINT32 slTemp;
    UINT8  j;
    UINT8  bData0;
    UINT8  phrase;

    pBuf   = pSeq[5 + no];
    dwSize = dwSeqSize[5 + no];
    dwGate = 0;
    dwTime = 0;
    i      = 0;
    phrase = 0;

    if (pBuf == 0) {
        return SMAF_ERR_SHORT_LENGTH;           /* Mtsq not Exist             */
    }

    for (j = 1; j < 8; j++) {
        if(pMTR->Phrase[j].dwSTp != 0xFFFFFFFF)
            phrase = 1;
    }

    while (i <= dwSize) {

        if (i == pMTR->Phrase[0].dwSTp)
            pMTR->Phrase[0].dwSTt = dwTime;
        if (i == pMTR->Phrase[0].dwSPp)
            pMTR->Phrase[0].dwSPt = dwTime;

        if (phrase) {
            for (j = 1; j < 8; j++) {
                if(i == pMTR->Phrase[j].dwSTp)
                    pMTR->Phrase[j].dwSTt = dwTime;
                if(i == pMTR->Phrase[j].dwSPp)
                    pMTR->Phrase[j].dwSPt = dwTime;
            }
        }

        if ((pMTR->Phrase[0].dwSPt != 0xFFFFFFFF) || (i == dwSize))
            break;

        slTemp = get_flex3((dwSize - i), no, &j);
        if (slTemp < 0) {
            return SMAF_ERR_CHUNK;
        }
        i += j;
        dwTime += slTemp;

        if(dwGate > (UINT32)slTemp)
            dwGate -= slTemp;                   /* GateTime                   */
        else
            dwGate = 0;

        if ((dwSize -i) < 2) {
            return SMAF_ERR_CHUNK;
        }
        bData0 = Decode_byte(no);              /* Status Byte                */
        i ++;
        switch ( bData0 & 0xF0 )  {
            case 0x80:                          /* Note Message (No Velocity) */
                CastAwayNByte((dwSize - i), 1, no);
                slTemp = get_flex3((dwSize - i), no, &j);
                if(slTemp < 0) {
                    return SMAF_ERR_CHUNK;
                }
                if(dwGate < (UINT32)slTemp)     /* Renew GateTime             */
                    dwGate = slTemp;
                i += j + 1;
                break;
            case 0x90:                          /* Note Message (Velocity)    */
                if(CastAwayNByte((dwSize - i), 2, no) < 0) {
                    return SMAF_ERR_CHUNK;
                }
                slTemp = get_flex3((dwSize - i), no, &j);
                if(slTemp < 0) {
                    return SMAF_ERR_CHUNK;
                }
                if(dwGate < (UINT32)slTemp)     /* Renew GateTime             */
                    dwGate = slTemp;
                i += j + 2;
                break;
            case 0xA0:                          /* UnKnown Event(NOP)         */
            case 0xB0:                          /* Control Change Message     */
            case 0xE0:                          /* Pitch Bend Message         */
                if(CastAwayNByte((dwSize - i), 2, no) < 0) {
                    return SMAF_ERR_CHUNK;
                }
                i += 2;
                break;
            case 0xC0:                          /* Program Change Message     */
            case 0xD0:                          /* UnKnown Event(NOP)         */
                CastAwayNByte((dwSize - i), 1, no);
                i += 1;
                break;
            case 0xF0 :                         /* Exclusive NOP or EOS       */
                if(bData0 == 0xF0){             /* Exclusive                  */
                    slTemp = get_flex3((dwSize - i), no, &j);
                    if(slTemp < 0) {
                        return SMAF_ERR_CHUNK;
                    }
#if ERROR_CHECK_STRENGTH
                    if(CastAwayNByte((dwSize - i),(UINT32)slTemp, no) != 0xF7) {
                        return SMAF_ERR_CHUNK;
                    }
#else
                    if(CastAwayNByte((dwSize - i),(UINT32)slTemp, no) <  0x00) {
                        return SMAF_ERR_CHUNK;
                    }
#endif
                    i += slTemp + j;
                    break;
                }
                else if (bData0 == 0xFF){       /* NOP or EOS                 */
                    if((dwSize - i) < 1) {      /* ErrorCheck(FileSize and sp)*/
                        return SMAF_ERR_CHUNK;
                    }
                    bData0 = Decode_byte7bit(no);
                    if(bData0 == 0x2F){         /* EOS(Next byte must be 0x00)*/
                        i += 1;
                        if((dwSize - i) < 1) {  /* ErrorCheck(FileSize and sp)*/
                            return SMAF_ERR_CHUNK;
                        }
                        bData0 = Decode_byte7bit(no);
                        if(bData0 != 0x00) {    /* ErrorCheck(EOS Message)    */
                            return SMAF_ERR_CHUNK;
                        }
                        else {
                            dwGate = 0;
                            i = dwSize;
                            pMTR->Phrase[0].dwSPt = dwTime;
                        }
                        break;
                    }
                    else if (bData0 == 0x00) {  /* NOP                        */
                        i += 1;
                        break;
                    }
                    else {                      /* Unknown Message(Error)     */
                        return SMAF_ERR_CHUNK;
                    }
                }
                else {                          /* Unknown Message(Error)     */
                    return SMAF_ERR_CHUNK;
                }
            default :                           /* Unknown Message(Error)     */
                return SMAF_ERR_CHUNK;
        }
    }

#if ERROR_CHECK_STRENGTH
    if ((pMTR->Phrase[0].dwSTt == 0xFFFFFFFF) &&
        (pMTR->Phrase[0].dwSTp != 0xFFFFFFFF)) {
        return SMAF_ERR_CHUNK;
    }
#endif
    if (pMTR->Phrase[0].dwSTt == 0xFFFFFFFF) {
        pMTR->Phrase[0].dwSTt = 0;
        pMTR->Phrase[0].dwSTp = 0;
    }

#if ERROR_CHECK_STRENGTH
    if ((pMTR->Phrase[0].dwSPt == 0xFFFFFFFF) && 
        (pMTR->Phrase[0].dwSPp != 0xFFFFFFFF) && (pMTR->Phrase[0].dwSPp != i)) {
        return SMAF_ERR_CHUNK;
    }
#endif
    if (pMTR->Phrase[0].dwSPt == 0xFFFFFFFF)
        pMTR->Phrase[0].dwSPt = dwTime + dwGate;

    for (j = 1; j < 8; j++) {
        if ((pMTR->Phrase[j].dwSTp != 0xFFFFFFFF) &&
            (pMTR->Phrase[j].dwSTp < pMTR->Phrase[0].dwSTp))
            pMTR->Phrase[j].dwSTt = pMTR->Phrase[0].dwSTt;

        if (pMTR->Phrase[j].dwSPp != 0xFFFFFFFF) {
            if(pMTR->Phrase[0].dwSPp <= pMTR->Phrase[j].dwSPp)
                pMTR->Phrase[j].dwSPt = pMTR->Phrase[0].dwSPt;
            if(pMTR->Phrase[0].dwSTp > pMTR->Phrase[j].dwSPp)
                pMTR->Phrase[j].dwSPt = 0xFFFFFFFF;
        }
    }
    pMTR->dwPlayTime = pMTR->Phrase[0].dwSPt - pMTR->Phrase[0].dwSTt;

    return F_TRUE;                              /* SUCCESS                    */
}


/*******************************************************************************
    Name        : TrackChunk_Check3
    Description : Score Track Chunk Check
    Parameter   : struct *SmafCheck ... Pointer to File Information
                : UINT8  no         ... Load Mode
    Return      :  >0 ... Error
                :   0 ... Success
*******************************************************************************/
static UINT8 TrackChunk_Check3(
    PSMAFINFO SmafCheck,
    UINT8  no
)
{
    UINT8 result;
    PMTRINFO pMTR;

    if (SmafCheck->ScoreTrack[5].pMTR == 0) {
        return SMAF_ERR_SHORT_LENGTH;           /* No MTR5                    */
    }
    if (SmafCheck->ScoreTrack[5].dwSize <= 20) {
        return SMAF_ERR_CHUNK;                  /* Lack Header Information    */
    }
    pMTR = &(SmafCheck->ScoreTrack[5]);
    result = MTR_Check(pMTR, SMAF_TYPE_MA3);
    if (result)
        return result;

    MspI_Check(pMTR);

    pDecodeBuffer[no] = pMTR->pSeq;

    if (pMTR->pMTR[0] == 0x01) {                /* Compressed Mode            */
        bCompress[no] = 0;
        dwSeqSize[5 + no] = Decode_init(no);    /* Make Huffman Tree          */
    }
    else {
		bCompress[no] = 1;
        dwSeqSize[5 + no] = pMTR->dwSeqSize;
    }

    result = ST_SP_Check(&(pMTR->Phrase[0]), dwSeqSize[5 + no]);
    if (result)
        return result;

    pSeq[5 + no]   = pDecodeBuffer[no];
    bBitOffset[no] = (UINT8)Decode_count[no];
    bFirstByte[no] = (UINT8)Decode_bitbuf[no];

    result = SeqData_Check3(pMTR, no);
    if (result)
        return result;
    SmafCheck->bTimeBase   = pMTR->bTimeBase;
    SmafCheck->dwStartTime = pMTR->Phrase[0].dwSTt;
    SmafCheck->dwPlayTime  = pMTR->dwPlayTime;

    return F_TRUE;
}


/*******************************************************************************
    Name        : GetContentsData3
    Description : Get Contents Information (Option Type MA-3)
    Parameter   : SINT32 file_id ... File ID
                : struct pinf    ... Pointer to Structure
    Return      : >=0: Success
                : < 0: Error
*******************************************************************************/
static SINT32 GetContentsData3(
    SINT32 file_id,
    PMASMW_CONTENTSINFO pinf
)
{
    UINT8  *pBuf, *pTemp;
    UINT32 dwSize, dwTemp;
    SINT32 slTemp;
    UINT16 wNextID;
    UINT32 i;

    pBuf   = Smaf_Info[file_id].pOPDA;
    dwSize = Smaf_Info[file_id].dwOPDASize;
    pTemp  = 0;
    dwTemp = 0;
    i = 0;

    while(dwSize > (UINT32)(8 + i)) {           /* OPDA                       */
        slTemp = NextChunk(5, &(pBuf[i]), (UINT32)(dwSize - i), &wNextID);
        if (slTemp < 0)
            return F_FALSE;
        i += SIZE_OF_CHUNKHEADER;
        if ( ((wNextID & 0x00FF) == 8) &&
            (((wNextID >> 8) & 0x00FF) == pinf->code)) {
            pTemp  = &pBuf[i];
			dwTemp = slTemp;
        }
        i += slTemp;
    }

    if (pTemp == 0)
        return MMFCNV_ERR_TAG;

    pBuf   = pTemp;
    dwSize = dwTemp;
    pTemp  = 0;
    slTemp = 0;
    i      = 0;

    while(dwSize >= (UINT32)(4 + i)) {          /* Dch* (Code type Much)      */
        dwTemp = (UINT32)(((UINT32)pBuf[i+2] << 8) + pBuf[i+3]);
        if (dwSize < (UINT32)(dwTemp + 4 + i))
            return F_FALSE;                     /* TAG Size Error             */
        if ((pBuf[i] == pinf->tag[0]) && (pBuf[i+1] == pinf->tag[1])) {
            pTemp  = &(pBuf[i+4]);
            slTemp = (SINT32)dwTemp;
        }
        i += dwTemp + 4;                        /* Next Information           */
    }

    if (pTemp == 0)                             /* Inforamtion not Found      */
        return MASMW_ERROR_UNMATCHED_TAG;
    if (slTemp > (SINT32)pinf->size)            /* Inforamtion Size Check     */
        slTemp = (SINT32)pinf->size;
    for(i = 0; i < (UINT32)slTemp; i++)
        pinf->buf[i] = pTemp[i];
    return slTemp;                              /* Return Information Size    */
}


/*******************************************************************************
    Name        : Exclusive_Check
    Description : Return Size, No, Data Pointer of Exclusive Message 
    Parameter   : UINT32 dwSize ... Remain Mtsu Data
                : UINT8  *pBuf  ... pointer to message
                : UINT8  *bNo   ... pointer to Exclusive No
                : UINT8  *bData ... pointer to Exclusive Data
    Return      :  =0 ... Error
                :  >0 ... Success(Whole Exclusive Message Size)
*******************************************************************************/
static UINT32 Exclusive_Check(
    UINT32 dwSize,
    UINT8  *pBuf,
    UINT8  *bNo,
    UINT8  *bData
)
{
    UINT32 dwMSize;
    UINT32 index;

    if (dwSize < 5)
        return 0;
    if (pBuf[0] != 0xF0)
        return 0;
    index = 1;
    dwMSize = (UINT32)(pBuf[index] & 0x7F);
    while ((pBuf[index] & 0x80) && (index < 4)) {
        index ++;
        dwMSize = (UINT32)((dwMSize << 7) + pBuf[index]);
    }
    dwMSize += index + 1;                       /* Message Size '0xF0'to'0xF7'*/
    if ((index & 0x80) || (dwMSize > dwSize))   /* Message Error              */
        return 0;

    if ((pBuf[index+1] != 0x43) || (pBuf[index+2] != 0x79) ||
        (pBuf[index+3] != 0x06) || (pBuf[index+4] != 0x7F)) {
        *bData = 0;
        return dwMSize;                         /* Maker ID Error             */
    }

    *bNo   = pBuf[index+5];                     /* Message No                 */
    *bData = (UINT8)(index + 6);                /* Data Pointer               */
    return dwMSize;
}


/*******************************************************************************
    Name        : Stream_Reserve
    Description : Check Stream Reserve Message
    Parameter   : UINT32 dwSetLength ... Length of Setup Data Chunk
                : UINT8  *pSetDeta   ... Pointer to Setup Data Chunk
    Return      : 0 - 2 : Number of Reservation
                : 0x7F  : Reserve Message Nothing
                : < 0   : Error
*******************************************************************************/
static UINT8 Stream_Reserve( void )
{
    UINT32 dwSize;
    UINT32 dwExSize;
    UINT8  *pBuf;
    UINT8  bNo;
    UINT8  bPointer;
    UINT32 dwIndex;
    UINT8  bRet;

    if (Smaf_Info[1].ScoreTrack[5].pWave == 0)
        bRet = 0;
    else
        bRet = 2;

    if (Smaf_Info[1].ScoreTrack[5].pSetup == 0)
        return bRet;

    pBuf    = Smaf_Info[1].ScoreTrack[5].pSetup;
    dwSize  = Smaf_Info[1].ScoreTrack[5].dwSetupSize;
    dwIndex = 0;

    while (dwSize >= 9) {
        dwExSize = Exclusive_Check(dwSize, &pBuf[dwIndex], &bNo, &bPointer);
        if (dwExSize == 0)
            return bRet;
        if ((bPointer != 0) && (bNo == 0x07) && (dwExSize == 0x09)) {
            if (pBuf[dwIndex + bPointer] <= 2)
                return pBuf[dwIndex + bPointer];
        }
        dwIndex += dwExSize;
        dwSize  -= dwExSize;
    }
    return bRet;
}


/*******************************************************************************
    Name        : Get_LM_Status3
    Description : Get LED Motor Status
    Parameter   : Nothing
    Return      : Nothing
*******************************************************************************/
static void Get_LM_Status3( void )
{
    UINT8  i;
    UINT8  *p;

    p = (Smaf_Info[1].ScoreTrack[5].pMTR + 4);
    for (i = 0; i < 16; i++) {
        if (p[i] & 0x10)
            ChInfo[i].bLed = 1;
        else
            ChInfo[i].bLed = 0;
        if (p[i] & 0x20)
            ChInfo[i].bMotor = 1;
        else
            ChInfo[i].bMotor = 0;
    }
}


/*******************************************************************************
    Name        : Stream3
    Description : Regist PCM Data
    Parameter   : Nothing
    Return      : Nothing
*******************************************************************************/
static void Stream3( void )
{
    UINT8  *pBuf;
    UINT32 dwSize;
    UINT32 index;
    SINT32 slNextSize;
    UINT16 wNextID;
    UINT8  bWaveID;
    UINT8  bFormat;
    UINT8  bBase;
    UINT16 wFs;

    pBuf   = Smaf_Info[1].ScoreTrack[5].pWave;
    dwSize = Smaf_Info[1].ScoreTrack[5].dwWaveSize;
    index  = 0;

    while ((dwSize - index) > 8) {
        slNextSize = NextChunk( 4, &pBuf[index], (dwSize - index), &wNextID);
        if (slNextSize < 0)
            return;
        bWaveID = (UINT8)((wNextID >> 8) & 0x00FF);
        index += SIZE_OF_CHUNKHEADER;
        if ((bWaveID < 1) || (32 < bWaveID) || ((wNextID & 0x00FF) != 0x000D) ||
            (Wave_Info3[bWaveID-1].bNote == 0x00) || (slNextSize < 4)) {
            index += slNextSize;
            continue;                           /* Skip This Chunk            */
        }

        bFormat = (UINT8)((pBuf[index] >> 4) & 0x07);
        bBase   = (UINT8)(pBuf[index] & 0x0F);
        wFs     = (UINT16)((pBuf[index+1] << 8) + pBuf[index+2]);
        if ((bFormat == MMF_WAVE_BIN_2S_COMP) && (bBase == MMF_WAVE_8BIT))
            bFormat = 3;
        else if ((bFormat == MMF_WAVE_BIN_OFFSET) && (bBase == MMF_WAVE_8BIT))
            bFormat = 2;
        else if ((bFormat == MMF_WAVE_ADPCM) && (bBase == MMF_WAVE_4BIT))
            bFormat = 1;
        else {
            index += slNextSize;
            continue;                           /* Skip This Chunk            */
        }

        if (bFormat == 1) {
            if ((wFs < MMF_WAVE_SF_MINIMUM)  || /* Minimum SampleFreq  4000Hz */
                (MMF_WAVE_SF_4BIT_MAX < wFs) || /* Maximum SampleFreq 24000Hz */
                ((slNextSize - 3) <= (wFs / 100))) {
                index += slNextSize;
                continue;                       /* Skip This Chunk            */
            }
        }
        else {
            if ((wFs < MMF_WAVE_SF_MINIMUM)  || /* Minimum SampleFreq  4000Hz */
                (MMF_WAVE_SF_8BIT_MAX < wFs) || /* Maximun SampleFreq 12000Hz */
                ((slNextSize - 3) <= (wFs / 50))) {
                index += slNextSize;
                continue;                       /* Skip This Chunk            */
            }
        }
        MaSndDrv_SetStream(MmfSeqID, (UINT8)(bWaveID-1), bFormat,
            (UINT32)wFs, &(pBuf[index+3]), slNextSize-3);
        Wave_Info3[bWaveID-1].bNote = 0x00;
        index += slNextSize;
    }
}


/*******************************************************************************
    Name        : BaseWave
    Description : Regist FM Basic Wave
    Parameter   : Nothing
    Return      : Nothing
*******************************************************************************/
static void BaseWave( void )
{
    UINT8  *pBuf;
    UINT32 dwSize;
    UINT32 index, pEx;
    UINT32 dwExSize;
    UINT8  bNo;
    UINT8  bPointer;
    UINT32 dwWave;

    pBuf   = Smaf_Info[1].ScoreTrack[5].pSetup;
    dwSize = Smaf_Info[1].ScoreTrack[5].dwSetupSize;
    index  = 0;

    while (dwSize > 10) {
        dwExSize = Exclusive_Check(dwSize, &pBuf[index], &bNo, &bPointer);
        if (dwExSize == 0)
            return;
        pEx = index + bPointer;
        index  += dwExSize;
        dwSize -= dwExSize;
        if ((bPointer == 0) || (bNo != 0x0C) || (dwExSize < 8))
            continue;
        if ((pBuf[pEx] != 15) && (pBuf[pEx] != 23) && (pBuf[pEx] != 31))
            continue;

        if((dwRamAdr % 2) && (dwRamSize != 0)) {/* Pointer Adjustment         */
            dwRamAdr ++;                        /*                            */
            dwRamSize --;                       /*                            */
        }                                       /* Pointer Adjustment         */
        if (dwRamSize < 2048)
            return;

        dwWave = dwExSize - bPointer - 2;
        if (dwWave >= 2341) {
            MaDevDrv_SendDirectRamData(dwRamAdr, 1, &(pBuf[pEx+1]), 2341);
            dwRamAdr += 2048;
        }
        else {
            MaDevDrv_SendDirectRamData(dwRamAdr, 1, &(pBuf[pEx+1]), dwWave);
            dwWave = dwWave % 8;                /* Make Data Size             */
            if(dwWave == 1)
                return;
            if(dwWave != 0)
                dwWave--;
            dwWave += ((dwExSize - bPointer - 2)/ 8)* 7;
            dwRamAdr += dwWave;
            dwWave = 2048 - dwWave;
            if((dwRamAdr % 2) && (dwRamSize != 0)) {
                dwRamAdr ++;
                dwRamSize --;
                dwWave --;
            }
            MaDevDrv_SendDirectRamVal(dwRamAdr, dwWave, 0);
            dwRamAdr += dwWave;
        }
        dwRamSize -= 2048;
        MaSndDrv_SetFmExtWave(MmfSeqID, pBuf[pEx], dwRamAdr-2048);
    }
}


/*******************************************************************************
    Name        : Decode_7bitData
    Description : Decode Data (7 bit Encoded)
    Parameter   : UINT8  *p7bit    ... Pointer to Encoded Data
                : UINT32 dw7Size   ... Size of Encoded Data
                : UINT8  *p8bit    ... Pointer to Writing Decoded Data
    Return      : Nothing
*******************************************************************************/
static void Decode_7bitData(
    UINT8  *p7bit,
    UINT32 dw7Size,
    UINT8  *p8bit,
    UINT32 *pdw8Size
){
    UINT8  bTmp;
    UINT8  bMsb;
    UINT32 i;
    UINT32 dwTmp;
    UINT32 dwCount;

    *pdw8Size = 0;
    if((p7bit == 0) || (dw7Size == 0) || (p8bit == 0))
        return ;
    dwCount = dw7Size;
    while((dwTmp = (dwCount >= 8) ? 8 : dwCount) > 0){
        bTmp = *p7bit++;
        for(i = 0; i < (dwTmp - 1); i++){
            bMsb = (UINT8)((bTmp >> (6 - i)) & 0x01);
            *p8bit++ = (UINT8)((bMsb << 7) | *p7bit++);
            (*pdw8Size)++;
        }
        dwCount -= dwTmp;
    }
}


/*******************************************************************************
    Name        : Setup_WtData
    Description : Set Wave Data
    Parameter   : UINT8  bWaveID ... Wave ID
    Return      :  >0: Success(Wave Data Address)
                :   0: Error
*******************************************************************************/
static UINT32 Setup_WtData(
    UINT8  bWaveID
)
{
    UINT8  *pBuf;
    UINT32 dwSize;
    UINT32 index, pEx;
    UINT32 dwExSize;
    UINT8  bNo,bPointer;
    UINT32 dwWaveSize;

    pBuf   = Smaf_Info[1].ScoreTrack[5].pSetup;
    dwSize = Smaf_Info[1].ScoreTrack[5].dwSetupSize;
    index  = 0;

    while (dwSize > 13) {
        dwExSize = Exclusive_Check(dwSize, &pBuf[index], &bNo, &bPointer);
        if (dwExSize == 0)
            return 0;
        pEx = index + bPointer;
        index  += dwExSize;
        dwSize -= dwExSize;
        if ((bPointer == 0) || (bNo != 0x03) || (dwExSize < 11))
            continue;

        if (pBuf[pEx] != bWaveID)               /* Wave ID Check              */
            continue;
        dwWaveSize = (dwExSize - bPointer - 3) % 8;
        if(dwWaveSize != 0)
            dwWaveSize -= 1;
        dwWaveSize += ((dwExSize - bPointer - 3) / 8)* 7;
        if((dwRamAdr % 2) && (dwRamSize != 0)) {/* Pointer Adjustment         */
            dwRamAdr ++;                        /*                            */
            dwRamSize --;                       /*                            */
        }                                       /* Pointer Adjustment         */
        if (dwRamSize < dwWaveSize)
            return 0;
        MaDevDrv_SendDirectRamData(dwRamAdr, 1, &(pBuf[pEx+2]),
            (dwExSize - bPointer - 3));
        pEx = dwRamAdr;
        dwRamAdr += dwWaveSize;
        dwRamSize -= dwWaveSize;
        return pEx;
    }
    return 0;
}


/*******************************************************************************
    Name        : Set_Voice3
    Description : Regist Voice Parameter
    Parameter   : Nothing
    Return      : Nothing
*******************************************************************************/
static void Set_Voice3( void )
{
    UINT8  *pBuf;
    UINT32 dwSize;
    UINT32 index, Exp;
    UINT32 dwExSize, dwDataSize;
    SINT32 dwWaveAdr;
    UINT8  bNo,bPointer;
    UINT8  bBankM, bBankL, bProgNo, bNote, bFlag, bKeyNo;
    UINT8  bBuf[32];
    UINT32 dwFs, dwFsScale;

    pBuf   = Smaf_Info[1].ScoreTrack[5].pSetup;
    dwSize = Smaf_Info[1].ScoreTrack[5].dwSetupSize;
    index  = 0;

    while (dwSize > 13) {
        dwExSize = Exclusive_Check(dwSize, &pBuf[index], &bNo, &bPointer);
        if (dwExSize == 0)
            return;
        Exp     = index + bPointer;
        index  += dwExSize;
        dwSize -= dwExSize;
        if ((bPointer == 0) || (bNo != 0x01) || (dwExSize < 32))
            continue;

        bBankM  = pBuf[Exp];
        bBankL  = pBuf[Exp+1];
        bProgNo = pBuf[Exp+2];
        bNote   = pBuf[Exp+3];
        bFlag   = pBuf[Exp+4];

        if (((bBankM != 0x7C) || (bBankL >= 0x0A)) &&
            ((bBankM != 0x7D) || (bBankL != 0x00) || (bProgNo >= 0x0A)))
            continue;
        if (bFlag & 0x01) {                     /* Wave Table                 */
            Decode_7bitData(&(pBuf[Exp+5]),(dwExSize-13),&bBuf[0],&dwDataSize);
            if (bBuf[15] & 0x80) {              /* ROM                        */
                dwWaveAdr = MaResMgr_GetDefWaveAddress((UINT8)(bBuf[15]&0x7F));
                if (dwWaveAdr <= 0)
                    return;
            }
            else {                               /* RAM                       */
                dwWaveAdr = (SINT32)Setup_WtData((UINT8)(bBuf[15]&0x7F));
                if (dwWaveAdr == 0)
                    continue;
            }
            bBuf[9]  = (UINT8)((dwWaveAdr >> 8) & 0x00FF);
            bBuf[10] = (UINT8)(dwWaveAdr & 0x00FF);
            if((dwRamAdr % 2) && (dwRamSize != 0)) {
                dwRamAdr ++;
                dwRamSize --;
            }
            if (dwRamSize < 14)
                return;
            MaDevDrv_SendDirectRamData(dwRamAdr, 0, &bBuf[2], 13);
            dwFs = (UINT32)(((bBuf[0] << 8) & 0xFF00) + (bBuf[1] & 0x00FF));
            if (dwFs > 48000)
                continue;
            dwFsScale = dwFs;
            if (bBankM == 0x7C)                 /* Normal Voice               */
                MaSndDrv_SetVoice(MmfSeqID, (UINT8)(bBankL+1), bProgNo, 2,
                    dwFsScale, dwRamAdr);
            else                                /* Drum Voice                 */
                MaSndDrv_SetVoice(MmfSeqID, (UINT8)(bProgNo+129), bNote, 2,
                    dwFsScale, dwRamAdr);
            dwRamAdr  += 14;
            dwRamSize -= 14;
        }
        else {                                  /* FM Voice                   */
            Decode_7bitData(&(pBuf[Exp+5]),(dwExSize-13),&bBuf[0],&dwDataSize);
            if ((Smaf_Info[1].bMaxCHNum == 32) && ((bBuf[2] & 0x07) > 2))
                continue;
            bKeyNo = (UINT8)(bBuf[0] & 0x7F);
            if((dwRamAdr % 2) && (dwRamSize != 0)) {
                dwRamAdr ++;
                dwRamSize --;
            }
            if (dwRamSize < (dwDataSize - 1))
                return;
            MaDevDrv_SendDirectRamData(dwRamAdr,0,&bBuf[1],(dwDataSize-1));
            if (bBankM == 0x7C)
                MaSndDrv_SetVoice(MmfSeqID, (UINT8)(bBankL+1), bProgNo, 1,
                    bKeyNo, dwRamAdr);
            else
                MaSndDrv_SetVoice(MmfSeqID, (UINT8)(bProgNo+129), bNote, 1,
                    bKeyNo, dwRamAdr);
            dwRamAdr  += (dwDataSize - 1);
            dwRamSize -= (dwDataSize - 1);
        }
    }
}


/*******************************************************************************
    Name        : Stream_ON3
    Description : Stream ON Process
    Parameter   : UINT8  ch    ... Channel Number
                : UINT8  key   ... Key No.
                : UINT8  vel   ... Velocity
                : UINT32 gtime ... Gatetime
                : struct *pEv  ... Pointer to Event List
                : UINT32 dwSize ... Sequence Data Size
    Return      : Nothing
*******************************************************************************/
static void Stream_ON3(
    UINT8  ch,
    UINT8  key,
    UINT8  vel,
    UINT32 gtime,
    PEVLIST pEv0,
    UINT32 dwSize
)
{
    PEVLIST  pEv1;
    POFFLIST pOffList0, pOffList1, pOffList2;
    UINT32 dwOffTime, dwOnTime;
    UINT8  bPair;

    if(Wave_Info3[key].bNote == 0xFF) {
        Ev_Set(MASNDDRV_CMD_NOP, 0, 0, 0, dwSize, pEv0);
        return;
    }

    dwOffTime = gtime + pEv0->dwTime;
    if (Wave_Info3[key].bPair != 0xFF) {

        bPair = Wave_Info3[key].bPair;

        pOffList1 = Get_OffList();
        pOffList2 = Get_OffList();
        pEv1 = Get_EvList();
        if ((pOffList1 == 0) || (pOffList2 == 0) ||(pEv1 == 0)) {
            Ev_Set(MASNDDRV_CMD_NOP, 0, 0, 0, dwSize, pEv0);
            if (pOffList1) {
                pOffList1->pNext = Play_Info.pEmptyOFF;
                Play_Info.pEmptyOFF = pOffList1;
            }
            if (pOffList2) {
                pOffList2->pNext = Play_Info.pEmptyOFF;
                Play_Info.pEmptyOFF = pOffList2;
            }
            if (pEv1) {
                pEv1->pNext = Play_Info.pEmptyEv;
                Play_Info.pEmptyEv = pEv1;
            }
            return;
        }

        if(Wave_Info3[key].bNote == 0x01){      /* Master ON ?                */
            pOffList0 = Search_OffList3(ch, key, dwOffTime);
            if (pOffList0 != 0) {
                pOffList0->dwTime = pEv0->dwTime;
                Set_OffList(pOffList0);
            }
        }
        pOffList1->dwTime = dwOffTime;
        pOffList1->dwKey  = key;
        pOffList1->dwCh   = ch;
        pOffList1->dwType = MASNDDRV_CMD_STREAM_OFF;
        Set_OffList(pOffList1);

        if(Wave_Info3[bPair].bNote == 0x01){    /* Slave ON ?                 */
            pOffList0 = Search_OffList3(ch, bPair, dwOffTime);
            if (pOffList0 != 0) {
                pOffList0->dwTime = pEv0->dwTime;
                Set_OffList(pOffList0);
            }
        }
        pOffList2->dwTime = dwOffTime;
        pOffList2->dwKey  = bPair;
        pOffList2->dwCh   = ch;
        pOffList2->dwType = MASNDDRV_CMD_STREAM_OFF;
        Set_OffList(pOffList2);

        dwOnTime = pEv0->dwTime;

        pEv1->dwTime   = dwOnTime;
        pEv1->bTrackNo = 5;
        Ev_Set(MASNDDRV_CMD_STREAM_SLAVE, ch, bPair, vel, dwSize, pEv1);
		Set_EvList(pEv1);

        pEv0->bTrackNo = 5;
        Ev_Set(MASNDDRV_CMD_STREAM_ON, ch, key, vel, dwSize, pEv0);
    }
    else {

        pOffList1 = Get_OffList();
        if (pOffList1 == 0) {
            Ev_Set(MASNDDRV_CMD_NOP, 0, 0, 0, dwSize, pEv0);
            return;
        }
        if(Wave_Info3[key].bNote == 0x01){
            pOffList0 = Search_OffList3(ch, key, dwOffTime);
            if (pOffList0 != 0) {
                pOffList0->dwTime = pEv0->dwTime;
                Set_OffList(pOffList0);
            }
        }
        pOffList1->dwTime = dwOffTime;
        pOffList1->dwKey  = key;
        pOffList1->dwCh   = ch;
        pOffList1->dwType = MASNDDRV_CMD_STREAM_OFF;
        Set_OffList(pOffList1);
        Ev_Set(MASNDDRV_CMD_STREAM_ON, ch, key, vel, dwSize, pEv0);
    }
}


/*******************************************************************************
    Name        : Note_ON3
    Description : Note ON Process
    Parameter   : UINT8  ch     ... Channel Number
                : UINT8  key    ... Key No.
                : UINT8  vel    ... Velocity
                : UINT32 gtime  ... Gatetime
                : struct *pEv   ... Pointer to Event List
                ; UINT32 dwSize ... Sequence Data Size
    Return      : Nothing
*******************************************************************************/
static void Note_ON3(
    UINT8  ch, 
    UINT8  key, 
    UINT8  vel,
    UINT32 gtime,
    PEVLIST pEv,
    UINT32 dwSize
)
{
    POFFLIST pOffList;

    if (gtime == 0) {
        Ev_Set(MASNDDRV_CMD_NOP, 0, 0, 0, dwSize, pEv);
        return ;
    }

    if ((ChInfo[ch].bType == MMF_CH_TYPE_DRUM) ||
        (ChInfo[ch].bType == MMF_CH_TYPE_STREAM)){
        if(ChInfo[ch].bMonoPoly)
            ChInfo[ch].bMonoPoly = 0;
        if((key <= 0x0C) || (key >= 0x5C)) {
            ChInfo[ch].bType = MMF_CH_TYPE_STREAM;
            if (key >= 0x5C)
                key -= 79;
            if (key >= 32) {
                Ev_Set(MASNDDRV_CMD_NOP, 0, 0, 0, dwSize, pEv);
                return ;
            }
        }
        else
            ChInfo[ch].bType = MMF_CH_TYPE_DRUM;
    }

    if(ChInfo[ch].bType != MMF_CH_TYPE_STREAM){

        pOffList = Get_OffList();
        if (pOffList == 0) {
            Ev_Set(MASNDDRV_CMD_NOP, 0, 0, 0, dwSize, pEv);
            return ;
        }
        pOffList->dwTime = pEv->dwTime + gtime;
        pOffList->dwKey  = key;
        pOffList->dwCh   = ch;
        pOffList->dwType = MASNDDRV_CMD_NOTE_OFF;
        Set_OffList(pOffList);
        Ev_Set(MASNDDRV_CMD_NOTE_ON, ch, key, vel, dwSize, pEv);
    }
    else {
        Stream_ON3(ch, key, vel, gtime, pEv, dwSize);
    }
}


/*******************************************************************************
    Name        : Bank_Program3
    Description : Reflection BankSelect and ProgramChange Message
    Parameter   : UINT8  ch     ... Channel Number
                : struct *pEv   ... Pointer to Event List
                : UINT32 dwSize ... Sequence Data Size
    Return      : Nothing
*******************************************************************************/
static void Bank_Program3(
    UINT8  ch,
    PEVLIST pEv,
    UINT32 dwSize
)
{
    if (ChInfo[ch].bBankM == 0x7C){
        ChInfo[ch].bType = MMF_CH_TYPE_NORMAL;
        if(ChInfo[ch].bBankL < 0x0a)
            ChInfo[ch].bBank = (UINT8)(ChInfo[ch].bBankL + 1);
        else
            ChInfo[ch].bBank = 0;
    }
    else if (ChInfo[ch].bBankM == 0x7D) {
        ChInfo[ch].bType = MMF_CH_TYPE_DRUM;
        if ((ChInfo[ch].bBankL == 0x00) && (ChInfo[ch].bPgm < 0x0A))
            ChInfo[ch].bBank = (UINT8)(ChInfo[ch].bPgm + 129);
        else
            ChInfo[ch].bBank = (UINT8)128;
    }
    else {
        if (ch == 0x09) {
            ChInfo[ch].bType = MMF_CH_TYPE_DRUM;
            ChInfo[ch].bBank = (UINT8)128;
        }
        else {
            ChInfo[ch].bType = MMF_CH_TYPE_NORMAL;
            ChInfo[ch].bBank = 0;
        }
    }
    if (pEv)
        Ev_Set(MASNDDRV_CMD_PROGRAM_CHANGE, ch, ChInfo[ch].bBank,
            ChInfo[ch].bPgm, dwSize, pEv);
}


/*******************************************************************************
    Name        : All_OFF
    Description : Erase Note OFF Block After All Sound/Note OFF 
    Parameter   : UINT32 dwTime ... Time of Event Become Effective
                : UINT8  ch     ... Channel Number
    Return      : Nothing
*******************************************************************************/
static void All_OFF(
    UINT32 dwTime,
    UINT8  ch
)
{
    void **Next;
    POFFLIST pOff;

    Next = &(Play_Info.pNextOFF);
    while (*Next != 0) {
        pOff = (POFFLIST)*Next;
        if ((pOff->dwCh == ch) && (dwTime <= pOff->dwTime)) {
            *Next = pOff->pNext;
            pOff->pNext = Play_Info.pEmptyOFF;
            Play_Info.pEmptyOFF = (void *)pOff;
        }
        else
            Next = &(pOff->pNext);
    }
}


/*******************************************************************************
    Name        : NextEvent3_Seek
    Description : Set Next Event
    Parameter   : UINT32 *dwSize ... Read Data Size
    Return      : Nothing
*******************************************************************************/
static void NextEvent3_Seek(
    UINT32 *dwSize
)
{
    UINT32 dwExSize, dwData;
    UINT8  bData0, bData1, bData2, bData3, bData4, bChannel;
    UINT8  i;

    bData0 = Decode_byte(1);                    /* Status Byte                */
    bChannel = (UINT8)(bData0 & 0x0F);          /* Channel #                  */
    (*dwSize) --;

    switch ( bData0 & 0xF0 )  {                 /* Event #                    */
        case 0x80:                              /* Note Message (No Velocity) */
            bData1 = Decode_byte7bit(1);        /* Key #                      */
            get_flex3(0xFF, 1, &i);             /* GateTime                   */
            Play_Info.wNoteFlag = 0xFFFF;
            (*dwSize) -= (i + 1);
            break;
        case 0x90:                              /* Note Message (Velocity)    */
            bData1 = Decode_byte7bit(1);        /* Key #                      */
            bData2 = Decode_byte7bit(1);        /* Velocity                   */
            get_flex3(0xFF, 1, &i);             /* GateTime                   */
            ChInfo[bChannel].bVel = VelocityTable[bData2];
            Play_Info.wNoteFlag = 0xFFFF;
            (*dwSize) -= (i + 2);
            break;
        case 0xC0:                              /* Program Change Message     */
            bData1 = Decode_byte7bit(1);        /* Program #                  */
            ChInfo[bChannel].bPgm = bData1;
            Bank_Program3(bChannel, 0, 0);
            (*dwSize) -= 1;
            break;
        case 0xB0:                              /* Control Change Message     */
            bData1 = Decode_byte7bit(1);        /* Data Byte 1                */
            bData2 = Decode_byte7bit(1);        /* Data Byte 2                */
            (*dwSize) -= 2;
            switch (bData1) {                   /* Event Decision             */
                case 0x00:                      /* Bank Select MSB            */
                    ChInfo[bChannel].bBankM = bData2 ;
                    break;
                case 0x20:                      /* Bank Select LSB            */
                    ChInfo[bChannel].bBankL = bData2 ;
                    break;
                case 0x01:                      /* Modulation Depth           */
                    if(bData2 == 0)
                        bData2 = 0;
                    else if(bData2 <= 0x1F)
                        bData2 = 1;
                    else if(bData2 <= 0x3F)
                        bData2 = 2;
                    else if(bData2 <= 0x5F)
                        bData2 = 3;
                    else
                        bData2 = 4;
                    ChInfo[bChannel].bModulation = bData2 ;
                    break;
                case 0x07:                      /* Channel Volume             */
                    ChInfo[bChannel].bVolume = bData2 ;
                    break;
                case 0x0A:                      /* Pan                        */
                    ChInfo[bChannel].bPanpot = bData2 ;
                    break;
                case 0x0B:                      /* Expression                 */
                    ChInfo[bChannel].bExpression = bData2 ;
                    break;
                case 0x40:                      /* Hold1                      */
                    if (bData2 <= 0x3F)
                        bData2 = 0;
                    else
                        bData2 = 1;
                    ChInfo[bChannel].bHold = bData2 ;
                    break;
                case 0x06:                      /* Data Entry MSB             */
                    if(ChInfo[bChannel].bRpnM==0 && ChInfo[bChannel].bRpnL==0){
                        if(bData2 <= 0x18)
                            ChInfo[bChannel].bPitchSens = bData2 ;
                        break;
                    }
                    break;
                case 0x64:                      /* RPN LSB                    */
                    ChInfo[bChannel].bRpnL = bData2 ;
                    break;
                case 0x65:                      /* RPN MSB                    */
                    ChInfo[bChannel].bRpnM = bData2 ;
                    break;
                case 0x79:                      /* Reset All Controller       */
                    if(bData2 == 0){
                        ChInfo[bChannel].bModulation = MMF_Modulation_Depth;
                        ChInfo[bChannel].bExpression = MMF_Expression;
                        ChInfo[bChannel].bHold       = MMF_Dumper_Hold;
                        ChInfo[bChannel].bRpnL       = MMF_RPN_MSB;
                        ChInfo[bChannel].bRpnM       = MMF_RPN_LSB;
                        ChInfo[bChannel].bPitchM     = MMF_Pitch_Bend_MSB;
                        ChInfo[bChannel].bPitchL     = MMF_Pitch_Bend_LSB;
                        ChInfo[bChannel].bVel = VelocityTable[64];                    }
                    break;
                case 0x7E:                      /* Mono Mode On               */
                    if ((bData2 == 1) &&
                        ((Play_Info.wNoteFlag & (0x0001 << bChannel))== 0)){
                        ChInfo[bChannel].bMonoPoly = 1;
                        Play_Info.wNoteFlag |= (0x0001 << bChannel);
                    }
                    break;
                case 0x7F:                      /* Poly Mode On               */
                    if(bData2 == 0)
                        Play_Info.wNoteFlag |= (0x0001 << bChannel);
                    break;
                default:                        /* NOP                        */
                    break;
            }
            break;
        case 0xE0:                              /* Pitch Bend Message         */
            bData1 = Decode_byte7bit(1);
            bData2 = Decode_byte7bit(1);
            (*dwSize) -= 2;
            if(ChInfo[bChannel].bPitchSens == 0)
                break;
            ChInfo[bChannel].bPitchM = bData2;
            ChInfo[bChannel].bPitchL = bData1;
            ChInfo[bChannel].bPitchS = ChInfo[bChannel].bPitchSens;
            break;
        case 0xF0:                              /* Exclusive or NOP or EOS    */
            if(bChannel == 0x0F){               /* NOP or EOS                 */
                bData1 = Decode_byte7bit(1);
                if(bData1 == 0x2F) {            /* EOS                        */
                    (*dwSize) = 0;
                    dwEventFlag |= EVENT_EOS;
                    break;
                }
                (*dwSize) --;
                break;
            }
            dwExSize = get_flex3(0xFF, 1, &i);  /* Exclusive                  */
            (*dwSize) -= (dwExSize + i);
            if (dwExSize < 6) {
                CastAwayNByte(dwExSize, dwExSize, 1);
                break;
            }
            bData1 = Decode_byte(1);
            dwData = bData1 << 24;
            bData1 = Decode_byte(1);
            dwData = bData1 << 16 | dwData;
            bData1 = Decode_byte(1);
            dwData = bData1 <<  8 | dwData;
            bData1 = Decode_byte(1);
            dwData = bData1       | dwData;
            dwExSize -= 4;
            if (dwData != 0x4379067F) {
                CastAwayNByte(dwExSize, dwExSize, 1);
                break;
            }
            bData1 = Decode_byte7bit(1);
            dwExSize --;
            switch (bData1) {
                case 0x00:                      /* MA-3 Master Volume         */
                    bData2 = Decode_byte7bit(1);
                    dwExSize--;
                    bMasterVol1 = bData2;
                    dwEventFlag |= EVENT_MASTER_VOLUME;
                    break;
                case 0x08:                      /* Stream Pair                */
                    bData2 = Decode_byte7bit(1);
                    bData3 = Decode_byte7bit(1);
                    bData4 = Decode_byte7bit(1);
                    dwExSize -= 3;
                    if ((bData3 < 1) || (32 < bData3) ||
                        (bData4 < 1) || (32 < bData4) || (bData3 == bData4))
                        break;
                    if ((Wave_Info3[bData3-1].bNote == 0) &&
                        (Wave_Info3[bData4-1].bNote == 0)) {
                        if (bData2 != 0x00) {
                            Wave_Info3[bData3-1].bPair = 0xFF;
                            Wave_Info3[bData4-1].bPair = 0xFF;
                        }
                        else {
                            Wave_Info3[bData3-1].bPair = (UINT8)(bData4 - 1);
                            Wave_Info3[bData4-1].bPair = (UINT8)(bData3 - 1);
                        }
                    }
                    break;
                case 0x0B:                      /* MA-3 Stream PCM Wave Panpot*/
                    bData2 = Decode_byte7bit(1);
                    bData3 = Decode_byte7bit(1);
                    bData4 = Decode_byte7bit(1);
                    dwExSize -= 3;
                    if((bData2 < 1) || (32 < bData2))
                        break;
                    if (bData3 == 0x00)
                        Wave_Info3[bData2 - 1].bPan = bData4;
                    if (bData3 == 0x01)
                        Wave_Info3[bData2 - 1].bPan = MMF_Wave_Panpot;
                    if (bData3 == 0x02)
                        Wave_Info3[bData2 - 1].bPan = 0x80;
                    break;
                default:
#if 0
                    CastAwayNByte(dwExSize, dwExSize, 1);
#endif
                    break;
            }
            CastAwayNByte(dwExSize, dwExSize, 1);
            break;
        case 0xA0:
            bData1 = Decode_byte7bit(1);
            bData2 = Decode_byte7bit(1);
            (*dwSize) -= 2;
            break;
        case 0xD0:
            bData1 = Decode_byte7bit(1);
            (*dwSize) --;
            break;
        default:
            break;
    }
    return;
}


/*******************************************************************************
    Name        : NextEvent3_Play
    Description : Set Next Event
    Parameter   : struct *pEv   ... Pointer to Event Information Structure
                : UINT32 dwSize ... Sequense Data Size
    Return      : Nothing
*******************************************************************************/
static void NextEvent3_Play(
    PEVLIST pEv,
    UINT32 dwSize
)
{
    UINT32 dwExSize, dwData;
    UINT8  bData0, bData1, bData2, bData3, bData4, bChannel;
    UINT8  i;
    UINT32 dwTemp;
    UINT16 wTemp;

    bData0 = Decode_byte(1);                    /* Status Byte                */
    bChannel = (UINT8)(bData0 & 0x0F);          /* Channel #                  */
    dwSize --;

    switch ( bData0 & 0xF0 )  {                 /* Event #                    */
        case 0x80:                              /* Note Message (No Velocity) */
            bData1 = Decode_byte7bit(1);        /* Key #                      */
            dwTemp = get_flex3(0xFF, 1, &i);    /* GateTime                   */
            dwSize -= (i + 1);
            Play_Info.wNoteFlag = 0xFFFF;
            Note_ON3(bChannel,bData1,ChInfo[bChannel].bVel,dwTemp,pEv,dwSize);
            break;
        case 0x90:                              /* Note Message (Velocity)    */
            bData1 = Decode_byte7bit(1);        /* Key #                      */
            bData2 = Decode_byte7bit(1);        /* Velocity                   */
            dwTemp = get_flex3(0xFF, 1, &i);    /* GateTime                   */
            dwSize -= (i + 2);
            ChInfo[bChannel].bVel = VelocityTable[bData2];
            Play_Info.wNoteFlag = 0xFFFF;
            Note_ON3(bChannel,bData1,ChInfo[bChannel].bVel,dwTemp,pEv,dwSize);
            break;
        case 0xC0:                              /* Program Change Message     */
            bData1 = Decode_byte7bit(1);        /* Program #                  */
            dwSize --;
            ChInfo[bChannel].bPgm = bData1;
            Bank_Program3(bChannel, pEv, dwSize);
            break;
        case 0xB0:                              /* Control Change Message     */
            bData1 = Decode_byte7bit(1);        /* Data Byte 1                */
            bData2 = Decode_byte7bit(1);        /* Data Byte 2                */
            dwSize -= 2;
            switch (bData1) {                   /* Event Decision             */
                case 0x00:                      /* Bank Select MSB            */
                    ChInfo[bChannel].bBankM = bData2;
                    Ev_Set(MASNDDRV_CMD_NOP, 0, 0, 0, dwSize, pEv);
                    break;
                case 0x01:                      /* Modulation Depth           */
                    if(bData2 == 0)
                        bData2 = 0;
                    else if(bData2 <= 0x1F)
                        bData2 = 1;
                    else if(bData2 <= 0x3F)
                        bData2 = 2;
                    else if(bData2 <= 0x5F)
                        bData2 = 3;
                    else
                        bData2 = 4;
                    Ev_Set(MASNDDRV_CMD_MODULATION_DEPTH, bChannel, bData2, 0,
                        dwSize, pEv);
                    break;
                case 0x06:                      /* Data Entry MSB             */
                    if ((ChInfo[bChannel].bRpnM != 0) ||
                        (ChInfo[bChannel].bRpnL != 0) ||
                        (bData2 > 0x18)) {
                        Ev_Set(MASNDDRV_CMD_NOP, 0, 0, 0, dwSize, pEv);
                        break;
                    }
                    if (bData2 == 0) {
                        Play_Info.wPitchBendFlag |= (0x0001 << bChannel);
                        Ev_Set(MASNDDRV_CMD_NOP, 0, 0, 0, dwSize, pEv);
                        break;
                    }
                    Play_Info.wPitchBendFlag &= ~( 0x0001 << bChannel );
                    Ev_Set(MASNDDRV_CMD_BEND_RANGE, bChannel, bData2, 0,
                        dwSize, pEv);
                    break;
                case 0x07:                      /* Channel Volume             */
                    Ev_Set(MASNDDRV_CMD_CHANNEL_VOLUME, bChannel, bData2, 0,
                        dwSize, pEv);
                    break;
                case 0x0A:                      /* Pan                        */
                    Ev_Set(MASNDDRV_CMD_PANPOT,bChannel,bData2,0,dwSize,pEv);
                    break;
                case 0x0B:                      /* Expression                 */
                    Ev_Set(MASNDDRV_CMD_EXPRESSION, bChannel, bData2, 0,
                        dwSize, pEv);
                    break;
                case 0x20:                      /* Bank Select LSB            */
                    ChInfo[bChannel].bBankL = bData2 ;
                    Ev_Set(MASNDDRV_CMD_NOP, 0, 0, 0, dwSize, pEv);
                    break;
                case 0x40:                      /* Hold1                      */
                    if (bData2 <= 0x3F)
                        bData2 = 0;
                    else
                        bData2 = 1;
                    Ev_Set(MASNDDRV_CMD_HOLD1, bChannel, bData2, 0,
                        dwSize, pEv);
                    break;
                case 0x64:                      /* RPN LSB                    */
                    ChInfo[bChannel].bRpnL = bData2 ;
                    Ev_Set(MASNDDRV_CMD_NOP, 0, 0, 0, dwSize, pEv);
                    break;
                case 0x65:                      /* RPN MSB                    */
                    ChInfo[bChannel].bRpnM = bData2 ;
                    Ev_Set(MASNDDRV_CMD_NOP, 0, 0, 0, dwSize, pEv);
                    break;
                case 0x78:                      /* All Sound Off              */
                    if(bData2 == 0) {
                        Ev_Set(MASNDDRV_CMD_ALL_SOUND_OFF, bChannel, 0, 0,
                            dwSize, pEv);
                        All_OFF(pEv->dwTime, bChannel);
                    }
                    else
                        Ev_Set(MASNDDRV_CMD_NOP, 0, 0, 0, dwSize, pEv);
                    break;
                case 0x79:                      /* Reset All Controller       */
                    if (bData2 != 0) {
                        Ev_Set(MASNDDRV_CMD_NOP, 0, 0, 0, dwSize, pEv);
                        break;
                    }
                    ChInfo[bChannel].bVel = VelocityTable[64];
                    Ev_Set(MASNDDRV_CMD_RESET_ALL_CONTROLLERS, bChannel, 0, 0,
                        dwSize, pEv);
                    break;
                case 0x7B:                      /* All Note Off               */
                    if(bData2 == 0) {
                        Ev_Set(MASNDDRV_CMD_ALL_NOTE_OFF, bChannel, 0, 0,
                            dwSize, pEv);
                        All_OFF(pEv->dwTime, bChannel);
                    }
                    else
                        Ev_Set(MASNDDRV_CMD_NOP, 0, 0, 0, dwSize, pEv);
                    break;
                case 0x7E:                      /* Mono Mode On               */
                    if ((bData2 == 1) &&
                        ((Play_Info.wNoteFlag & (0x0001 << bChannel))== 0)){
                        Play_Info.wNoteFlag |= (0x0001 << bChannel);
                        Ev_Set(MASNDDRV_CMD_MONO_MODE_ON, bChannel, 0, 0,
                            dwSize, pEv);
                        break;
                    }
                    Ev_Set(MASNDDRV_CMD_NOP, 0, 0, 0, dwSize, pEv);
                    break;
                case 0x7F:                      /* Poly Mode On               */
                    if(bData2 == 0)
                        Play_Info.wNoteFlag |= (0x0001 << bChannel);
                    Ev_Set(MASNDDRV_CMD_NOP, 0, 0, 0, dwSize, pEv);
                    break;
                default:                        /* NOP                        */
                    Ev_Set(MASNDDRV_CMD_NOP, 0, 0, 0, dwSize, pEv);
                    break;
            }
            break;
        case 0xE0:                              /* Pitch Bend Message         */
            bData1 = Decode_byte7bit(1);
            bData2 = Decode_byte7bit(1);
            dwSize -= 2;
            if (Play_Info.wPitchBendFlag & (0x0001 << bChannel)) {
                Ev_Set(MASNDDRV_CMD_NOP, 0, 0, 0, dwSize, pEv);
                break;
            }
            wTemp = (UINT16)((bData1 & 0x7F) | ((bData2 & 0x7F) << 7));
            Ev_Set(MASNDDRV_CMD_PITCH_BEND, bChannel, wTemp, 0, dwSize, pEv);
            break;
        case 0xF0:                              /* Exclusive or NOP or EOS    */
            if(bChannel == 0x0F){               /* NOP or EOS                 */
                bData1 = Decode_byte7bit(1);
                if(bData1 == 0x2F)              /* EOS                        */
                    Ev_Set(MMF_EVENT_EOS, 0, 0, 0, (dwSize - 2), pEv);
                else                            /* NOP                        */
                    Ev_Set(MASNDDRV_CMD_NOP, 0, 0, 0, (dwSize - 1), pEv);
                break;
            }
            dwExSize = get_flex3(0xFF, 1, &i);  /* Exclusive                  */
            dwSize -= (dwExSize + i);
            if (dwExSize < 6) {                 /* Size Check                 */
                CastAwayNByte(dwExSize, dwExSize, 1);
                Ev_Set(MASNDDRV_CMD_NOP, 0, 0, 0, dwSize, pEv);
                break;
            }
            bData1 = Decode_byte(1);
            dwData = bData1 << 24;
            bData1 = Decode_byte(1);
            dwData = bData1 << 16 | dwData;
            bData1 = Decode_byte(1);
            dwData = bData1 <<  8 | dwData;
            bData1 = Decode_byte(1);
            dwData = bData1       | dwData;
            dwExSize -= 4;
            if (dwData != 0x4379067F) {         /* Maker ID Check             */
                CastAwayNByte(dwExSize, dwExSize, 1);
                Ev_Set(MASNDDRV_CMD_NOP, 0, 0, 0, dwSize, pEv);
                break;
            }
            bData1 = Decode_byte7bit(1);
            dwExSize --;
            switch (bData1) {
                case 0x00:                      /* MA-3 Master Volume         */
                    bData2 = Decode_byte7bit(1);
                    dwExSize--;
                    bMasterVol1 = bData2;
                    Ev_Set(MASNDDRV_CMD_MASTER_VOLUME, bMasterVol1, 0, 0,
						dwSize, pEv);
                    break;
                case 0x08:                      /* Stream Pair                */
                    Ev_Set(MASNDDRV_CMD_NOP, 0, 0, 0, dwSize, pEv);
                    bData2 = Decode_byte7bit(1);
                    bData3 = Decode_byte7bit(1);
                    bData4 = Decode_byte7bit(1);
                    dwExSize -= 3;
                    if ((bData3 < 1) || (32 < bData3) ||
                        (bData4 < 1) || (32 < bData4) || (bData3 == bData4))
                        break;
                    if ((Wave_Info3[bData3-1].bNote == 0) &&
                        (Wave_Info3[bData4-1].bNote == 0)) {
                        if (bData2 != 0x00) {
                            Wave_Info3[bData3-1].bPair = 0xFF;
                            Wave_Info3[bData4-1].bPair = 0xFF;
                        }
                        else {
                            Wave_Info3[bData3-1].bPair = (UINT8)(bData4 - 1);
                            Wave_Info3[bData4-1].bPair = (UINT8)(bData3 - 1);
                        }
                    }
                    break;
                case 0x10:                      /* MA-3 Set Interrupt         */
                    bData2 = Decode_byte7bit(1);
                    dwExSize--;
                    if (bData2 <= 15)
                        Ev_Set(MASNDDRV_CMD_USER_EVENT,bData2,0,0,dwSize,pEv);
                    else
                        Ev_Set(MASNDDRV_CMD_NOP, 0, 0, 0, dwSize, pEv);
                    break;
                case 0x0B:                      /* MA-3 Stream PCM Wave Panpot*/
                    bData2 = Decode_byte7bit(1);
                    bData3 = Decode_byte7bit(1);
                    bData4 = Decode_byte7bit(1);
                    dwExSize -= 3;
                    if((bData2 < 1) || (32 < bData2)) {
                        Ev_Set(MASNDDRV_CMD_NOP, 0, 0, 0, dwSize, pEv);
                        break;
                    }
                    if (bData3 == 0x01)
                        bData4 = 0xFF;
                    else if (bData3 == 0x02)
                        bData4 = 0x80;
                    else
                        Ev_Set(MASNDDRV_CMD_NOP, 0, 0, 0, dwSize, pEv);
                    Ev_Set(MASNDDRV_CMD_STREAM_PANPOT, (bData2 - 1), bData4, 0,
                        dwSize, pEv);
                    break;
                default:
                    Ev_Set(MASNDDRV_CMD_NOP, 0, 0, 0, dwSize, pEv);
                    break;
            }
            CastAwayNByte(dwExSize, dwExSize, 1);
            break;
        case 0xA0:
            bData1 = Decode_byte7bit(1);
            bData2 = Decode_byte7bit(1);
            dwSize -= 2;
            Ev_Set(MASNDDRV_CMD_NOP, 0, 0, 0, dwSize, pEv);
            break;
        case 0xD0:
            bData1 = Decode_byte7bit(1);
            dwSize --;
            Ev_Set(MASNDDRV_CMD_NOP, 0, 0, 0, dwSize, pEv);
            break;
        default:
            Ev_Set(MASNDDRV_CMD_NOP, 0, 0, 0, dwSize, pEv);
            break;
    }
    return;
}


/*******************************************************************************
    Name        : SetPreEvent3
    Description : Collect Event Information (before Seek Time)
    Parameter   : Nothing
    Return      : Nothing
*******************************************************************************/
static void SetPreEvent3( void )
{
    UINT8  i;
    UINT32 dwDuration;
    UINT32 dwStartTime;
    UINT32 dwSize;
    PEVLIST pEvList;

    pDecodeBuffer[1] = pSeq[6];
    Decode_bitbuf[1] = bFirstByte[1];
    Decode_count[1]  = bBitOffset[1];
    Play_Info.dwPastTime = 0;

    if (dwSeekTick >= (dwEndTick - Smaf_Info[1].dwStartTime)) {
        dwEventFlag &= ~(UINT32)EVENT_LED_ON;
        dwEventFlag &= ~(UINT32)EVENT_MOTOR_ON;
        dwEventFlag |=  (UINT32)EVENT_SYSTEM_ON;
        Play_Info.bPreEvent = 2;
   
        Play_Info.dwPastTime = dwEndTick;
        return;
    }

    Play_Info.wPitchBendFlag = 0;
    for (i = 0; i < 16; i++) {
        ChInfo[i].bType       = MMF_CH_TYPE_NORMAL;
        ChInfo[i].bSlotNum    = 0;
        ChInfo[i].bVel        = VelocityTable[64];
        ChInfo[i].bBank       = MMF_Bank_No;
        ChInfo[i].bBankM      = MMF_Bank_Select_MSB;
        ChInfo[i].bBankL      = MMF_Bank_Selsct_LSB;
        ChInfo[i].bPgm        = MMF_Program_No;
        ChInfo[i].bRpnM       = MMF_RPN_MSB;
        ChInfo[i].bRpnL       = MMF_RPN_LSB;
        ChInfo[i].bModulation = MMF_Modulation_Depth;
        ChInfo[i].bVolume     = MMF_Channel_Volume;
        ChInfo[i].bPanpot     = MMF_Channel_Panpot;
        ChInfo[i].bExpression = MMF_Expression;
        ChInfo[i].bHold       = MMF_Dumper_Hold;
        ChInfo[i].bPitchSens  = MMF_Pitch_Bend_Sensitivity;
        ChInfo[i].bMonoPoly   = MMF_Channel_Mode;
        ChInfo[i].bPitchM     = MMF_Pitch_Bend_MSB;
        ChInfo[i].bPitchL     = MMF_Pitch_Bend_LSB;
    }
    for (i = 0; i < 32; i++) {
        Wave_Info3[i].bPan  = MMF_Wave_Panpot;
        Wave_Info3[i].bPair = 0xFF;
        if (Wave_Info3[i].bNote == 1)
            Wave_Info3[i].bNote = 0;
    }

    ChInfo[9].bBank = 128;                      /* Channel 9 -> Drum Channel  */
    ChInfo[9].bType = MMF_CH_TYPE_DRUM;         /* Channel 9 -> Drum Channel  */

    dwSize      = dwSeqSize[6];
    dwStartTime = Smaf_Info[1].dwStartTime + dwSeekTick;

    dwDuration  = get_flex3(dwSize, 1, &i);

    while ((Play_Info.dwPastTime + dwDuration) < dwStartTime) {
        Play_Info.dwPastTime += dwDuration;
        dwSize -= i;
        NextEvent3_Seek(&dwSize);
        dwDuration = get_flex3(0xFF, 1, &i);
    }

    pEvList = Get_EvList();
    pEvList->dwTime   = Play_Info.dwPastTime + dwDuration;
    pEvList->bTrackNo = 5;
    NextEvent3_Play(pEvList, (dwSize - i));
    Set_EvList(pEvList);

    Play_Info.dwPastTime = dwStartTime;

    dwEventFlag &= ~(UINT32)EVENT_LED_ON;
    dwEventFlag &= ~(UINT32)EVENT_MOTOR_ON;
    dwEventFlag |=  (UINT32)(EVENT_SYSTEM_ON | EVENT_BANK_9);
}


/*******************************************************************************
    Name        : Standby3
    Description : Standby Process (Set param, wave)
    Parameter   : Nothing
    Return      : Nothing
*******************************************************************************/
static void Standby3( void )
{
#if MA13_KCS_IGNORE
#else
    UINT8  i,KeyCon;
#endif

    if ((Smaf_Info[1].ScoreTrack[5].pWave != 0) && (Play_Info.bStream != 0))
        Stream3();                              /* Entry Stream Data          */
    if (Smaf_Info[1].ScoreTrack[5].pSetup != 0) {
        BaseWave();                             /* Entry Set Up Data          */
        Set_Voice3();
    }

#if MA13_KCS_IGNORE
#else
    for(i = 0; i < 16; i++){
        KeyCon=(UINT8)((*(Smaf_Info[1].ScoreTrack[5].pMTR + 4 + i)>> 6)& 0x03);
        if ((KeyCon == 0x01) || (KeyCon == 0x02))
            MaSndDrv_SetKeyControl(MmfSeqID, i, KeyCon);
    }
#endif
}


/*******************************************************************************
    Name        : PushPreEvent3
    Description : Set Event (before Seek Time)
    Parameter   : Nothing
    Return      : =>0 : Number of Set Command
                :  <0 : Error
*******************************************************************************/
static SINT32 PushPreEvent3( void )
{
    UINT8  count, bBendm, bBendl;
    UINT16 wBend;

    if (Play_Info.bPreEvent == 2) {             /* Seek Time >= End Time      */
        dwEventFlag = EVENT_EOS;                /* Next Event = EOS           */
        Play_Info.bPreEvent = 0;
        return SEEK_END_FLAG;
    }

    for (count=0; count<16; count++) {
        if((ChInfo[count].bBank       != MMF_Bank_No) ||
           (ChInfo[count].bPgm        != MMF_Program_No)) {
            MaSndDrv_SetCommand(MmfSeqID, 0, MASNDDRV_CMD_PROGRAM_CHANGE, count,
                ChInfo[count].bBank, ChInfo[count].bPgm);
            ChInfo[count].bBank = MMF_Bank_No;
            ChInfo[count].bPgm  = MMF_Program_No;
            return 1;
        }
        if (ChInfo[count].bModulation != MMF_Modulation_Depth) {
            MaSndDrv_SetCommand(MmfSeqID, 0, MASNDDRV_CMD_MODULATION_DEPTH,
                count, ChInfo[count].bModulation, 0);
            ChInfo[count].bModulation = MMF_Modulation_Depth;
            return 1;
        }
        if (ChInfo[count].bVolume     != MMF_Channel_Volume) {
            MaSndDrv_SetCommand(MmfSeqID, 0, MASNDDRV_CMD_CHANNEL_VOLUME,
                count, ChInfo[count].bVolume, 0);
            ChInfo[count].bVolume = MMF_Channel_Volume;
            return 1;
        }
        if (ChInfo[count].bPanpot     != MMF_Channel_Panpot){
            MaSndDrv_SetCommand(MmfSeqID, 0, MASNDDRV_CMD_PANPOT,
                count, ChInfo[count].bPanpot, 0);
            ChInfo[count].bPanpot = MMF_Channel_Panpot;
            return 1;
        }
        if (ChInfo[count].bExpression != MMF_Expression){
            MaSndDrv_SetCommand(MmfSeqID, 0, MASNDDRV_CMD_EXPRESSION,
                count, ChInfo[count].bExpression, 0);
            ChInfo[count].bExpression = MMF_Expression;
            return 1;
        }
        if (ChInfo[count].bHold       != MMF_Dumper_Hold){
            MaSndDrv_SetCommand(MmfSeqID, 0, MASNDDRV_CMD_HOLD1,
                count, ChInfo[count].bHold, 0);
            ChInfo[count].bHold = MMF_Dumper_Hold;
            return 1;
        }
        if (ChInfo[count].bMonoPoly   == 0x01){
            MaSndDrv_SetCommand(MmfSeqID, 0, MASNDDRV_CMD_MONO_MODE_ON,
                count, 0, 0);
            ChInfo[count].bMonoPoly += 1;
            return 1;
        }
        if ((ChInfo[count].bPitchM     != MMF_Pitch_Bend_MSB) ||
            (ChInfo[count].bPitchL     != MMF_Pitch_Bend_LSB)) {
            if (ChInfo[count].bPitchS  != MMF_Pitch_Bend_Sensitivity) {
                MaSndDrv_SetCommand(MmfSeqID, 0, MASNDDRV_CMD_BEND_RANGE,
                    count, ChInfo[count].bPitchS, 0);
                ChInfo[count].bPitchS = MMF_Pitch_Bend_Sensitivity;
                return 1;
            }
            else {
                bBendm = (UINT8)(ChInfo[count].bPitchM & 0x7F);
                bBendl = (UINT8)(ChInfo[count].bPitchL & 0x7F);
                wBend  = (UINT16)((bBendm << 7) + bBendl);
                MaSndDrv_SetCommand(MmfSeqID, 0, MASNDDRV_CMD_PITCH_BEND,
                    count, wBend, 0);
                ChInfo[count].bPitchM = MMF_Pitch_Bend_MSB;
                ChInfo[count].bPitchL = MMF_Pitch_Bend_LSB;
                return 1;
            }
        }
        if ((ChInfo[count].bPitchSens  != MMF_Pitch_Bend_Sensitivity) &&
            (ChInfo[count].bPitchSens  != 0)) {
            MaSndDrv_SetCommand(MmfSeqID, 0, MASNDDRV_CMD_BEND_RANGE,
                count, ChInfo[count].bPitchSens, 0);
            ChInfo[count].bPitchSens = MMF_Pitch_Bend_Sensitivity;
            return 1;
        }
    }

    for (count = 0; count < 32; count++) {
        if(Wave_Info3[count].bPan != MMF_Wave_Panpot){
            MaSndDrv_SetCommand(MmfSeqID, 0, MASNDDRV_CMD_STREAM_PANPOT,
               count, Wave_Info3[count].bPan, 0);
            Wave_Info3[count].bPan = MMF_Wave_Panpot;
            return 1;
            break;
        }
    }
    Play_Info.bPreEvent = 0;
    return SEEK_END_FLAG;
}


/*******************************************************************************
    Name        : Set_NoteOffEvent
    Description : Set Note OFF Event
    Parameter   : Nothing
    Return      : Number of Set Command
*******************************************************************************/
static SINT32 Set_NoteOffEvent( void )
{
    POFFLIST pOff;
    UINT32 dwDuration, dwCh, dwKey, dwEvent;

    pOff = (POFFLIST)Play_Info.pNextOFF;
    if (pOff->dwTime >= dwEndTick) {
        dwEventFlag = EVENT_EOS;
        return Set_EventEOS();
    }

    dwDuration = (pOff->dwTime - Play_Info.dwPastTime) * Play_Info.bTimeBaseR;
    dwCh       = (UINT32)pOff->dwCh;
    dwKey      = (UINT32)pOff->dwKey;
    dwEvent    = (UINT32)pOff->dwType;

    Play_Info.dwPastTime = pOff->dwTime;
    Play_Info.pNextOFF   = pOff->pNext;
    pOff->pNext          = Play_Info.pEmptyOFF;
    Play_Info.pEmptyOFF  = (void *)pOff;

    if (dwEvent == MMF_EVENT_STREAM_OFF_MA2) {
        MaSndDrv_SetCommand(MmfSeqID, dwDuration, MASNDDRV_CMD_STREAM_OFF,
            dwCh, dwKey, 0);
        bAudio_Slot = 0;
        if (bAudio_LED == 1)
            dwEventFlag |= (UINT32)EVENT_LED_OFF;
        return 1;
    }
    if (dwEvent == MASNDDRV_CMD_STREAM_OFF)
        Wave_Info3[dwKey].bNote = 0x00;

    MaSndDrv_SetCommand(MmfSeqID, dwDuration, dwEvent, dwCh, dwKey, 0);

    ChInfo[dwCh].bSlotNum --;
    if ((ChInfo[dwCh].bSlotNum == 0) && (ChInfo[dwCh].bLed != 0)) {
        Play_Info.wLED &= ~(UINT16)(0x0001 << dwCh);
        if (Play_Info.wLED == 0)
            dwEventFlag |= (UINT32)EVENT_LED_OFF;
    }
    if ((ChInfo[dwCh].bSlotNum == 0) && (ChInfo[dwCh].bMotor != 0)) {
        Play_Info.wMOTOR &= ~(UINT16)(0x0001 << dwCh);
        if (Play_Info.wMOTOR == 0)
            dwEventFlag |= (UINT32)EVENT_MOTOR_OFF;
    }
    return 1;
}


/*******************************************************************************
    Name        : Set_Event
    Description : Event Convert Process
    Parameter   : Nothing
    Return      : Number of Set Command
*******************************************************************************/

static SINT32 Set_Event( void )
{
    PEVLIST pEv;
    UINT32 dwDuration, dwCom, dwVal1, dwVal2, dwVal3, dwSize;
    UINT8  bTrackNo, i;

    pEv = (PEVLIST)Play_Info.pNextEv;
    dwDuration = (pEv->dwTime - Play_Info.dwPastTime) * Play_Info.bTimeBaseR;
    dwCom      = pEv->dwEvNo;
    dwVal1     = pEv->dwVal1;
    dwVal2     = pEv->dwVal2;
    dwVal3     = pEv->dwVal3;
    dwSize     = pEv->dwSize;

    if ((dwCom == MMF_EVENT_EOS) || (pEv->dwTime >= dwEndTick)) {
        dwEventFlag |= EVENT_EOS;
        return Set_EventEOS();
    }

    if ((dwCom == MASNDDRV_CMD_NOTE_ON) ||
        (dwCom == MASNDDRV_CMD_STREAM_ON) ||
        (dwCom == MASNDDRV_CMD_STREAM_SLAVE) ||
        (dwCom == MASNDDRV_CMD_NOTE_ON_MA2) ||
        (dwCom == MASNDDRV_CMD_NOTE_ON_MA2EX)) {
        if (ChInfo[dwVal1].bSlotNum == 0) {
            if ((ChInfo[dwVal1].bLed) && (Play_Info.wLED == 0)) {
                dwEventFlag |= EVENT_LED_ON;
                Play_Info.wLED |= (UINT16)(1 << dwVal1);
            }
            if ((ChInfo[dwVal1].bMotor) && (Play_Info.wMOTOR == 0)) {
                dwEventFlag |= EVENT_MOTOR_ON;
                Play_Info.wMOTOR |= (UINT16)(1 << dwVal1);
            }
        }
        if ((dwCom == MASNDDRV_CMD_NOTE_ON_MA2) ||
            (dwCom == MASNDDRV_CMD_NOTE_ON_MA2EX))
            ChInfo[dwVal1].bSlotNum = 1;
        else
            ChInfo[dwVal1].bSlotNum++;
    }
    if (dwCom == MMF_EVENT_STREAM_ON_MA2) {
        if ((bAudio_LED == 1) && (bAudio_Slot == 0))
            dwEventFlag |= EVENT_LED_ON;
        bAudio_Slot = 1;
        dwCom = MASNDDRV_CMD_STREAM_ON;
    }
    if ((dwCom == MASNDDRV_CMD_ALL_SOUND_OFF) ||
        (dwCom == MASNDDRV_CMD_ALL_NOTE_OFF)) {
        ChInfo[dwVal1].bSlotNum = 0;
        if (ChInfo[dwVal1].bLed != 0) {
            Play_Info.wLED &= ~(UINT16)(0x0001 << dwVal1);
            if (Play_Info.wLED == 0)
                dwEventFlag |= (UINT32)EVENT_LED_OFF;
        }
        if (ChInfo[dwVal1].bMotor != 0) {
            Play_Info.wMOTOR &= ~(UINT16)(0x0001 << dwVal1);
            if (Play_Info.wMOTOR == 0)
                dwEventFlag |= (UINT32)EVENT_MOTOR_OFF;
        }
    }
    MaSndDrv_SetCommand(MmfSeqID, dwDuration, dwCom, dwVal1, dwVal2, dwVal3);
    Play_Info.dwPastTime = pEv->dwTime;
    Play_Info.pNextEv    = pEv->pNext;

    if (pEv->bTrackNo == 5) {
        if ((dwCom == MASNDDRV_CMD_STREAM_SLAVE) || (dwSize < 3)){
            pEv->pNext = Play_Info.pEmptyEv;
            Play_Info.pEmptyEv = (void *)pEv;
        }
        else {
            dwDuration  = get_flex3(0xFF, 1, &i);
            dwSize -= i;
            pEv->dwTime = Play_Info.dwPastTime + dwDuration;
            NextEvent3_Play(pEv, dwSize);
            Set_EvList(pEv);
        }
    }
    else {
        bTrackNo = pEv->bTrackNo;
        dwDuration  = get_flex2(pSeq[bTrackNo], dwSeqSize[bTrackNo]);
        if (dwDuration == 0xFFFFFFFF) {
            pEv->pNext = Play_Info.pEmptyEv;
            Play_Info.pEmptyEv = (void *)pEv;
            return 1;
        }
        if (dwDuration >= 128) {
            pSeq[bTrackNo]      ++;
            dwSeqSize[bTrackNo] --;
        }
        pSeq[bTrackNo]      ++;
        dwSeqSize[bTrackNo] --;
        pEv->dwTime = Play_Info.dwPastTime + dwDuration;
        if (bTrackNo == AUDIO_TRACK_NO)
            NextEvent2_PlayA(pEv, bTrackNo);
        else
            NextEvent2_PlayS(pEv, bTrackNo);
        Set_EvList(pEv);
    }
    return 1;
}


/*--- SMAF MA-3   Function ---------------------------------------------------*/
/*--- Common B    Function ---------------------------------------------------*/

/*******************************************************************************
    Name        : Ma_Load
    Description : File Laod Process
    Parameter   : UINT8  *file_ptr         ... Pointer to SMAF File
                : UINT32 file_size         ... File Size
                : UINT8  mode              ... Load Mode
    Return      : >=0 : Success
                : < 0 : Error
*******************************************************************************/
static SINT32 Ma_Load(
    UINT8  *file_ptr,
    UINT32 file_size,
    UINT8  mode
)
{
    PSMAFINFO SmafCheck;
    UINT8  *pBuffer;                            /* Buffer Pointer             */
    UINT32 dwSize;                              /* File Size                  */
    SINT32 slNextSize;                          /* Chunk Size                 */
    UINT16 wNextID;                             /* Chunk ID                   */
    UINT32 index;                               /* Pointer Index              */
    SINT8  result;                              /* Track Chunk Check Result   */
    UINT16 wCalc_CRC;                           /* calculate CRC              */
    UINT16 wFile_CRC;                           /* File CRC                   */
    UINT8  no;                                  /* Array #                    */

    index = 0;
    pBuffer = file_ptr;
    dwSize  = file_size;
    if (mode <= 1)
        no = 1;
    else
        no = 0;
    wNextID = 0xFFFF;
    SmafCheck = &(Smaf_Info[no]);
    Check_Initial(SmafCheck);                   /* Registed Info All Clear    */

    slNextSize = NextChunk( 0, &pBuffer[index], (dwSize - index), &wNextID);
    if ((slNextSize <= 0) || (wNextID != 1)) {
        return MMFCNV_ERR_FILE;                 /* MMMD Error                 */
    }
    SmafCheck->dwMMMDSize = (UINT32)slNextSize; /* File Chunk Size            */
    dwSize = SmafCheck->dwMMMDSize + SIZE_OF_CHUNKHEADER;

    wFile_CRC = 0;
    if (mode != 0) {
        wCalc_CRC = makeCRC(dwSize,&pBuffer[0]);/* CRC Check                  */
        wFile_CRC = (UINT16)((pBuffer[dwSize - 2] << 8) + pBuffer[dwSize - 1]);
        if (wCalc_CRC != wFile_CRC) {
            return MMFCNV_ERR_FILE;             /* CRC Error!!                */
        }
    }

    index += SIZE_OF_CHUNKHEADER;
    slNextSize = NextChunk( 1, &pBuffer[index], (dwSize - index), &wNextID);
    if ((slNextSize < 5) || (wNextID != 2)) {
        return MMFCNV_ERR_FILE;                 /* CNTI Error                 */
    }
    index += SIZE_OF_CHUNKHEADER;
    if ((pBuffer[index + 1] <= 0x2F) || ((pBuffer[index + 1] & 0x0F) <= 1)) {
        SmafCheck->bSmafType = SMAF_TYPE_MA2;   /* SMAF MA-1/2                */
        result = (SINT8)CNTI_Check2(&pBuffer[index]);
    }
    else {
        SmafCheck->bSmafType = SMAF_TYPE_MA3;   /* SMAF MA-3                  */
        result = (SINT8)CNTI_Check3(&pBuffer[index]);
    }
    if (pBuffer[index + 1] <= 0x2F) {           /* OPDA Type MA-1/2           */
        if (slNextSize >= 9) {                  /* Option Infomation          */
            SmafCheck->pOPDA      = &pBuffer[index + 5];
            SmafCheck->dwOPDASize = (UINT32)(slNextSize - 5);
        }
        else {
            SmafCheck->pOPDA      = 0;
            SmafCheck->dwOPDASize = 0;
        }
        index += slNextSize;                    /* Pointer Adjustment         */
    }
    else {                                      /* OPDA Type MA-3             */
        index += slNextSize;                    /* Pointer Adjustment         */
        slNextSize = NextChunk( 2, &pBuffer[index], (dwSize - index), &wNextID);
        if ((slNextSize >= 12) && ((wNextID & 0x00FF) == 0x0003)) {
            SmafCheck->pOPDA      = &pBuffer[index + SIZE_OF_CHUNKHEADER];
            SmafCheck->dwOPDASize = (UINT32)slNextSize;
        }
        else {
            SmafCheck->pOPDA      = 0;
            SmafCheck->dwOPDASize = 0;
        }
    }
    if (result == MMFCNV_ERR_C_CLASS) {
        return MMFCNV_ERR_C_CLASS;
    }
    if (0 <= result) {
        SmafCheck->bMaxCHNum = (UINT8)result;
        result = 0;
    }
    else {
        result = SMAF_ERR_TYPE;                 /* Contents Type Error !!     */
    }
    while((dwSize - index) > (SIZE_OF_CHUNKHEADER + SIZE_OF_CRC)) {
        slNextSize = NextChunk( 2, &pBuffer[index], (dwSize - index), &wNextID);
        if(slNextSize < 0){
            if(wNextID <= 0x0002)
                return MMFCNV_ERR_FILE;
            else
                return MMFCNV_ERR_SIZE;
        }
        index += SIZE_OF_CHUNKHEADER;
        switch((UINT8)(wNextID & 0x00FF)){
            case 0x04:                          /* Score Track Chunk          */
                wNextID = (UINT16)((wNextID >> 8) & 0x00FF);
                if (wNextID <= 5) {
                    SmafCheck->ScoreTrack[wNextID].pMTR   = &pBuffer[index];
                    SmafCheck->ScoreTrack[wNextID].dwSize = slNextSize;
                }
                break;
            case 0x05:                          /* Audio Track Chunk          */
                wNextID = (UINT16)((wNextID >> 8) & 0x00FF);
                if (wNextID == 0) {
                    SmafCheck->AudioTrack.pATR   = &pBuffer[index];
                    SmafCheck->AudioTrack.dwSize = slNextSize;
                }
                break;
            case 0x03:                          /* Optional Data Chunk        */
            case 0x06:                          /* Graphics Data Chunk        */
            case 0x07:                          /* Master Track Chunk         */
            case 0xFF:                          /* Unknown Chunk              */
                break;
            default:
                return MMFCNV_ERR_FILE;
        }
        index += slNextSize;
    }

    if (mode == 3) {
        SmafCheck->pFile  = file_ptr;
        SmafCheck->dwSize = file_size;
        return 0;                               /* Return File ID "0"         */
    }

    if (SmafCheck->bSmafType == SMAF_TYPE_MA2)
        result = (SINT8)(result + TrackChunk_Check2(SmafCheck, no));
    else
        result = (SINT8)(result + TrackChunk_Check3(SmafCheck, no));

    if ((result & SMAF_ERR_CHUNK) == SMAF_ERR_CHUNK)
        return MMFCNV_ERR_CHUNK;
    if ((result & SMAF_ERR_TYPE) == SMAF_ERR_TYPE)
        return MMFCNV_ERR_C_TYPE;
    if ((result & SMAF_ERR_SHORT_LENGTH) == SMAF_ERR_SHORT_LENGTH)
        return MMFCNV_ERR_LENGTH;
    if ((SmafCheck->dwPlayTime * SmafCheck->bTimeBase) <= PLAY_TIME_MIN) {
		SmafCheck->dwPlayTime = 0;
		SmafCheck->bTimeBase  = 0;
        return MMFCNV_ERR_LENGTH;
	}

    if (mode == 0) {
        SmafCheck->pFile  = file_ptr;
        SmafCheck->dwSize = file_size;
        return 1;                               /* Return File ID "1"         */
    }


    SmafCheck->pFile  = file_ptr;
    SmafCheck->dwSize = file_size;
    SmafCheck->dwCRC  = (UINT32)wFile_CRC;
    return no;                                  /* Return File ID             */
}


/*******************************************************************************
    Name        : GetPhraseList
    Description : Get Phrase List Information
    Parameter   : void *prm ... Pointer to Structure
                : UINT32 file_id ... File ID
    Return      :   0: Success
                : < 0: Error
*******************************************************************************/
static SINT32 GetPhraseList(
    void   *prm,
    UINT32 file_id
)
{
    UINT8  i;
    UINT32 dwST;
    PMASMW_PHRASELIST phrase;
    PPHRA  pPhrase;

    phrase = (PMASMW_PHRASELIST)prm;
MAMMFCNV_DBGMSG(("\nPhraseList Tag : %2x %2x",phrase->tag[0],phrase->tag[1]));

    if ((Smaf_Info[file_id].bSmafType != SMAF_TYPE_MA3) || 
        (phrase->tag[0] != 'P') ||
        (Smaf_Info[file_id].ScoreTrack[5].pPhrase == 0))
        return MASMW_ERROR_UNMATCHED_TAG;

    switch(phrase->tag[1]){
        case 0x41:                              /* A-melo                     */
            i = PHRASE_PA;
            break;
        case 0x42:                              /* B-melo                     */
            i = PHRASE_PB;
            break;
        case 0x45:                              /* Ending                     */
            i = PHRASE_PE;
            break;
        case 0x49:                              /* Intro                      */
            i = PHRASE_PI;
            break;
        case 0x4B:                              /* Interlude                  */
            i = PHRASE_PK;
            break;
        case 0x53:                              /* Bridge                     */
            i = PHRASE_PS;
            break;
        case 0x52:                              /* Refrain                    */
            i = PHRASE_PR;
            break;
        default  :                              /* Unkwon TAG                 */
            return MASMW_ERROR_UNMATCHED_TAG;
    }
    pPhrase = &(Smaf_Info[file_id].ScoreTrack[5].Phrase[i]);
    if ((pPhrase->dwSTt == 0xFFFFFFFF) || (pPhrase->dwSPt == 0xFFFFFFFF) ||
        (pPhrase->dwSPp <= pPhrase->dwSTp))
        return MASMW_ERROR_UNMATCHED_TAG;

    dwST = Smaf_Info[file_id].ScoreTrack[5].Phrase[0].dwSTt;
    phrase->start = (pPhrase->dwSTt - dwST)* Smaf_Info[file_id].bTimeBase;
    phrase->stop  = (pPhrase->dwSPt - dwST)* Smaf_Info[file_id].bTimeBase;

    return F_TRUE;
}


/*******************************************************************************
    Name        : GetLED
    Description : Get LED status
    Parameter   : SINT32 file_id ... File ID
    Return      : 0  : asynchronous
                : 1  : synchronous 
*******************************************************************************/
static UINT8 GetLED(
    SINT32 file_id
)
{
	MASMW_CONTENTSINFO Led_State;
	SINT32 ret;
	UINT8  i;
	UINT8  *p;

	if (Smaf_Info[file_id].bSmafType != SMAF_TYPE_MA3) {
		Led_State.code		= 0xFF;
		Led_State.tag[0]	= 'L';
		Led_State.tag[1]	= '2';
		Led_State.buf		= &i;
		Led_State.size		= 1;
		ret = MaMmfCnv_Control(file_id, MASMW_GET_CONTENTSDATA, &Led_State, 0);
		if (ret == 1)			return 1;
	}
	else {
		p = (Smaf_Info[file_id].ScoreTrack[5].pMTR + 4);
		for (i = 0; i < 16; i ++)
			if (p[i] & 0x10)	return 1;
	}
	return 0;
}


/*******************************************************************************
    Name        : GetVIB
    Description : Get VIB status
    Parameter   : SINT32 file_id ... File ID
    Return      : 0  : asynchronous
                : 1  : synchronous 
*******************************************************************************/
static UINT8 GetVIB(
    SINT32 file_id
)
{
	UINT8  i;
	UINT8  *p;

	if		(Smaf_Info[file_id].bSmafType == SMAF_TYPE_MA1) {
		p = Smaf_Info[file_id].ScoreTrack[0].pMTR;
		if ((p[4] & 0x44) || (p[5] & 0x44))			return 1;
		return 0;
	}
	else if	(Smaf_Info[file_id].bSmafType == SMAF_TYPE_MA2) {
		for (i = 1; i < 5; i ++) {
			p = Smaf_Info[file_id].ScoreTrack[i].pMTR;
			if (p == NULL)							continue;
			if ((p[4] & 0x44) || (p[5] & 0x44))		return 1;
		}
		return 0;
	}
	else {
		p = (Smaf_Info[file_id].ScoreTrack[5].pMTR + 4);
		for (i = 0; i < 16; i ++) {
			if (p[i] & 0x20)						return 1;
		}
		return 0;
	}
}

/*******************************************************************************
    Name        : GetKCS
    Description : Get KCS status
    Parameter   : SINT32 file_id ... File ID
    Return      : 0  : disable
                : 1  : enable 
*******************************************************************************/
static UINT8 GetKCS(
    SINT32 file_id
)
{
	UINT8  i;
	UINT8  *p;

	if		(Smaf_Info[file_id].bSmafType == SMAF_TYPE_MA1) {
#if MA13_KCS_IGNORE
		return 1;
#else
		p = Smaf_Info[file_id].ScoreTrack[0].pMTR;
		if ((p[4] & 0x88) || (p[5] & 0x88))			return 1;
#endif
	}
	else if	(Smaf_Info[file_id].bSmafType == SMAF_TYPE_MA2) {
		for (i = 1; i < 5; i ++) {
			p = Smaf_Info[file_id].ScoreTrack[i].pMTR;
			if (p == NULL)							continue;
			if ((p[4] & 0x88) || (p[5] & 0x88))		return 1;
		}
	}
	else {
#if MA13_KCS_IGNORE
		return 1;
#else
		p = (Smaf_Info[file_id].ScoreTrack[5].pMTR + 4);
		for (i = 0; i < 16; i ++)
			if ((p[i] & 0xC0) != 0x40)				return 1;
#endif
	}
	return 0;
}

/*******************************************************************************
    Name        : GetContentsData
    Description : Get Contents Information
    Parameter   : SINT32 file_id ... File ID
                : struct pinf    ... Pointer to Structure
    Return      : >=0: Success
                : < 0: Error
*******************************************************************************/
static SINT32 GetContentsData(
    SINT32 file_id,
    PMASMW_CONTENTSINFO pinf
)
{
    if ((pinf->tag[0] == 'C') &&
        (0x30 <= pinf->tag[1]) && (pinf->tag[1] <= 0x33)){
        switch(pinf->tag[1]) {
            case 0x30 :                         /* Contents Class             */
                *(pinf->buf) = Smaf_Info[file_id].pFile[16];
                break;
            case 0x31 :                         /* Contents Type              */
                *(pinf->buf) = Smaf_Info[file_id].pFile[17];
                break;
            case 0x32 :                         /* Copy Status                */
                *(pinf->buf) = Smaf_Info[file_id].pFile[19];
                break;
            case 0x33 :                         /* Copy Counts                */
                *(pinf->buf) = Smaf_Info[file_id].pFile[20];
                break;
        }
        return 1;                               /* Data Size(1Byte)           */
    }

	if ((pinf->tag[0] == 'L') &&
		(pinf->tag[1] == 'D')) {				/* LED sutatus				  */
		if (Smaf_Info[file_id].dwPlayTime == 0)		return F_FALSE;
		*(pinf->buf) = GetLED(file_id);
		return 1;								/* Data Size(1Byte)           */
	}
	if ((pinf->tag[0] == 'V') &&
		(pinf->tag[1] == 'B')) {				/* VIB sutatus				  */
		if (Smaf_Info[file_id].dwPlayTime == 0)		return F_FALSE;
		*(pinf->buf) = GetVIB(file_id);
		return 1;								/* Data Size(1Byte)           */
	}
	if ((pinf->tag[0] == 'K') &&
		(pinf->tag[1] == 'C')) {				/* key control sutatus		  */
		if (Smaf_Info[file_id].dwPlayTime == 0)		return F_FALSE;
		*(pinf->buf) = GetKCS(file_id);
		return 1;								/* Data Size(1Byte)           */
	}

    if (Smaf_Info[file_id].pFile[POSITION_OF_CONTENTSTYPE - 1] < 0x30)
        return GetContentsData2(file_id, pinf);
    else
        return GetContentsData3(file_id, pinf);
}


/*******************************************************************************
    Name        : PushFlagEvent
    Description : Set Secondary Event
    Parameter   : Nothing
    Return      : <=0: Success
                :  <0: Error
*******************************************************************************/
static SINT32 PushFlagEvent( void )
{
    PEVLIST pEv;

    if(dwEventFlag & EVENT_LED_OFF) {
        MaSndDrv_SetCommand(MmfSeqID, 0, MASNDDRV_CMD_LED_OFF, 0, 0, 0);
        dwEventFlag &= ~(UINT32)EVENT_LED_OFF;
        return 1;
    }
    else if(dwEventFlag & EVENT_MOTOR_OFF) {
        MaSndDrv_SetCommand(MmfSeqID, 0, MASNDDRV_CMD_MOTOR_OFF, 0, 0, 0);
        dwEventFlag &= ~(UINT32)EVENT_MOTOR_OFF;
        return 1;
    }
    else if (dwEventFlag & EVENT_SYSTEM_ON) {
        MaSndDrv_SetCommand(MmfSeqID, 0, MASNDDRV_CMD_SYSTEM_ON, 0, 0, 0);
        dwEventFlag &= ~(UINT32)EVENT_SYSTEM_ON;
        return 1;
    }
    else if(dwEventFlag & EVENT_MASTER_VOLUME) {


        MaSndDrv_SetCommand(MmfSeqID, 0, MASNDDRV_CMD_MASTER_VOLUME,
			bMasterVol1, 0, 0);
        dwEventFlag &= ~(UINT32)EVENT_MASTER_VOLUME;
        return 1;
    }
    else if(dwEventFlag & EVENT_LED_ON) {
        MaSndDrv_SetCommand(MmfSeqID, 0, MASNDDRV_CMD_LED_ON, 0, 0, 0);
        dwEventFlag &= ~(UINT32)EVENT_LED_ON;
        return 1;
    }
    else if(dwEventFlag & EVENT_MOTOR_ON) {
        MaSndDrv_SetCommand(MmfSeqID, 0, MASNDDRV_CMD_MOTOR_ON, 0, 0, 0);
        dwEventFlag &= ~(UINT32)EVENT_MOTOR_ON;
        return 1;
    }
    else if(dwEventFlag & EVENT_BANK_9) {
        MaSndDrv_SetCommand(MmfSeqID, 0, MASNDDRV_CMD_PROGRAM_CHANGE, 9,
            ChInfo[9].bBank, ChInfo[9].bPgm);
        dwEventFlag &= ~(UINT32)EVENT_BANK_9;
        ChInfo[9].bBank = MMF_Bank_No;
        ChInfo[9].bPgm  = MMF_Program_No;
        return 1;
    }
    else if ( dwEventFlag & EVENT_ENDTIME )
    {
        dwEndTick = dwEndTime / Smaf_Info[1].bTimeBase;
        if (dwEndTick > Smaf_Info[1].dwPlayTime)
            dwEndTick = Smaf_Info[1].dwPlayTime + Smaf_Info[1].dwStartTime;
        else
            dwEndTick += Smaf_Info[1].dwStartTime;
        pEv = Search_EOS();
        pEv->dwTime = dwEndTick;
        Set_EvList(pEv);
        return 0;
    }
    else if ( dwEventFlag & EVENT_EOS )
    {
        return Set_EventEOS();
    }

    dwEventFlag = 0;
    return 0;
}


/*******************************************************************************
    Name        : Get_LoadInfo
    Description : Write Load Information
    Parameter   : UINT32 file_id ... File ID
                : void   *pwrite ... Pointer to Data Structure
    Return      : <=0: Success
                :  <0: Error
*******************************************************************************/
static SINT32 Get_LoadInfo(
    UINT32 file_id,
    void   *pwrite
)
{
    UINT32 dwTemp, i;
    UINT32 *pLoad;
    UINT8  *pSum;
    PSMAFINFO pSmaf;
    PMTRINFO  pMTR;

    if (pwrite == NULL)
        return F_FALSE;
    pLoad = (UINT32 *)pwrite;
    pSmaf = &(Smaf_Info[file_id]);
    pMTR  = &(Smaf_Info[file_id].ScoreTrack[5]);
    pLoad[ 0] = 0;                              /* File Pointer               */
    pLoad[31] = 0;                              /* Other Information          */
    if ((pSmaf->pFile == 0) || (pSmaf->dwCRC == CRC_INITIAL))
        return F_FALSE;

    pSum   = (UINT8 *)pwrite;
    if (pSmaf->bSmafType != SMAF_TYPE_MA3) {    /* SAMF Type MA-1/2           */
        pLoad[ 0] = 1;                          /* File Pointer               */
        pSum[124] = 0xFF;                       /* SMAF Type                  */
        pSum[125] = 0x00;                       /* TimeBase                   */
        pSum[126] = 0x00;                       /* Max CH Num                 */
        pSum[127] = 0x00;                       /* Check Sum                  */
        return F_TRUE + 1;                      /* Not Supported              */
    }

    pLoad[ 0] = (UINT32)pSmaf->pFile;           /* File Pointer               */
    pLoad[ 1] = pSmaf->dwSize;                  /* File Size                  */
    pLoad[ 2] = pSmaf->dwMMMDSize + 8;          /* SMAF Data Size             */
    pLoad[ 3] = pSmaf->dwCRC;                   /* CRC                        */
    pLoad[ 4] = (UINT32)pSmaf->pOPDA;           /* Option Info                */
    pLoad[ 5] = pSmaf->dwOPDASize;              /* Option Info Size           */
    pLoad[ 6] = pSmaf->dwPlayTime;              /* Play Time                  */

    pLoad[ 7] = (UINT32)pMTR->pMTR;             /* MTR* Info                  */
    pLoad[ 8] = pMTR->dwSize;                   /* MTR* Size                  */
    pLoad[ 9] = (UINT32)pMTR->pSetup;           /* Mtsu Info                  */
    pLoad[10] = pMTR->dwSetupSize;              /* Mtsu Size                  */
    pLoad[11] = (UINT32)pMTR->pSeq;             /* Mtsq Info                  */
    pLoad[12] = pMTR->dwSeqSize;                /* Mtsq Size                  */
    pLoad[13] = (UINT32)pMTR->pWave;            /* Mtsp Info                  */
    pLoad[14] = pMTR->dwWaveSize;               /* Mtsp Size                  */

    pLoad[15] = pMTR->Phrase[0].dwSTt;          /* Start Time (tick) Whole    */
    pLoad[16] = pMTR->Phrase[0].dwSPt;          /* Stop  Time (tick) Whole    */
    pLoad[17] = pMTR->Phrase[1].dwSTt;          /* Start Time (tick) PA       */
    pLoad[18] = pMTR->Phrase[1].dwSPt;          /* Stop  Time (tick) PA       */
    pLoad[19] = pMTR->Phrase[2].dwSTt;          /* Start Time (tick) PB       */
    pLoad[20] = pMTR->Phrase[2].dwSPt;          /* Stop  Time (tick) PB       */
    pLoad[21] = pMTR->Phrase[3].dwSTt;          /* Start Time (tick) PE       */
    pLoad[22] = pMTR->Phrase[3].dwSPt;          /* Stop  Time (tick) PE       */
    pLoad[23] = pMTR->Phrase[4].dwSTt;          /* Start Time (tick) PI       */
    pLoad[24] = pMTR->Phrase[4].dwSPt;          /* Stop  Time (tick) PI       */
    pLoad[25] = pMTR->Phrase[5].dwSTt;          /* Start Time (tick) PK       */
    pLoad[26] = pMTR->Phrase[5].dwSPt;          /* Stop  Time (tick) PK       */
    pLoad[27] = pMTR->Phrase[6].dwSTt;          /* Start Time (tick) PS       */
    pLoad[28] = pMTR->Phrase[6].dwSPt;          /* Stop  Time (tick) PS       */
    pLoad[29] = pMTR->Phrase[7].dwSTt;          /* Start Time (tick) PR       */
    pLoad[30] = pMTR->Phrase[7].dwSPt;          /* Stop  Time (tick) PR       */

    pSum[124] = bCompress[file_id];             /* Compress / NoCompress      */
    pSum[125] = pSmaf->bTimeBase;               /* Time Base                  */
    pSum[126] = pSmaf->bMaxCHNum;               /* Mode 4op 16/ 2op 32        */

    dwTemp = 0;
    for (i = 0; i < 127; i++)
        dwTemp += pSum[i];

    pSum[127] = (UINT8)(dwTemp & 0x000000FF);

    return F_TRUE;
}


/*******************************************************************************
    Name        : Set_LoadInfo
    Description : Write Load Information
    Parameter   : void   *pread  ... Pointer to Data Structure
    Return      : <=0: Success
                :  <0: Error
*******************************************************************************/
static SINT32 Set_LoadInfo(
    void   *pread
)
{
    UINT32 dwTemp, i;
    UINT32 *pLoad;
    UINT8  *pSum;
    UINT8  bTimeBase, bMaxCHNum, bSmafType;
    PSMAFINFO pSmaf;
    PMTRINFO  pMTR;

    if (pread == NULL)
        return F_FALSE;
    pLoad = (UINT32 *)pread;
    pSmaf = &(Smaf_Info[1]);
    pMTR  = &(Smaf_Info[1].ScoreTrack[5]);
    if ((pSmaf->pFile != 0) || (pLoad[0] == 0))
        return F_FALSE;

    pSum   = (UINT8 *)pread;
    if( (pSum[124]==0xFF)&&(pSum[125]==0x00)&&(pSum[126]==0x00)&&(pSum[127]==0x00) )
        return F_TRUE + 1;                      /* Not Supported              */
    dwTemp = 0;
    for (i = 0; i < 127; i++)
        dwTemp += pSum[i];
    if( pSum[127] != (UINT8)(dwTemp & 0x000000FF) )
        return F_FALSE;                         /* Data Error                 */

    pSmaf->pFile          = (UINT8 *)pLoad[ 0]; /* File Pointer               */
    pSmaf->dwSize         =          pLoad[ 1]; /* File Size                  */
    pSmaf->dwMMMDSize     =          pLoad[ 2]; /* SMAF Data Size             */
    pSmaf->dwCRC          =          pLoad[ 3]; /* CRC                        */
    pSmaf->pOPDA          = (UINT8 *)pLoad[ 4]; /* Option Info                */
    pSmaf->dwOPDASize     =          pLoad[ 5]; /* Option Info Size           */
    pSmaf->dwPlayTime     =          pLoad[ 6]; /* Play Time                  */

    pMTR->pMTR            = (UINT8 *)pLoad[ 7]; /* MTR* Info                  */
    pMTR->dwSize          =          pLoad[ 8]; /* MTR* Size                  */
    pMTR->pSetup          = (UINT8 *)pLoad[ 9]; /* Mtsu Info                  */
    pMTR->dwSetupSize     =          pLoad[10]; /* Mtsu Size                  */
    pMTR->pSeq            = (UINT8 *)pLoad[11]; /* Mtsq Info                  */
    pMTR->dwSeqSize       =          pLoad[12]; /* Mtsq Size                  */
    pMTR->pWave           = (UINT8 *)pLoad[13]; /* Mtsp Info                  */
    pMTR->dwWaveSize      =          pLoad[14]; /* Mtsp Size                  */

    pMTR->Phrase[0].dwSTt =          pLoad[15]; /* Start Time (tick) Whole    */
    pMTR->Phrase[0].dwSPt =          pLoad[16]; /* Stop  Time (tick) Whole    */
    pMTR->Phrase[1].dwSTt =          pLoad[17]; /* Start Time (tick) PA       */
    pMTR->Phrase[1].dwSPt =          pLoad[18]; /* Stop  Time (tick) PA       */
    pMTR->Phrase[2].dwSTt =          pLoad[19]; /* Start Time (tick) PB       */
    pMTR->Phrase[2].dwSPt =          pLoad[20]; /* Stop  Time (tick) PB       */
    pMTR->Phrase[3].dwSTt =          pLoad[21]; /* Start Time (tick) PE       */
    pMTR->Phrase[3].dwSPt =          pLoad[22]; /* Stop  Time (tick) PE       */
    pMTR->Phrase[4].dwSTt =          pLoad[23]; /* Start Time (tick) PI       */
    pMTR->Phrase[4].dwSPt =          pLoad[24]; /* Stop  Time (tick) PI       */
    pMTR->Phrase[5].dwSTt =          pLoad[25]; /* Start Time (tick) PK       */
    pMTR->Phrase[5].dwSPt =          pLoad[26]; /* Stop  Time (tick) PK       */
    pMTR->Phrase[6].dwSTt =          pLoad[27]; /* Start Time (tick) PS       */
    pMTR->Phrase[6].dwSPt =          pLoad[28]; /* Stop  Time (tick) PS       */
    pMTR->Phrase[7].dwSTt =          pLoad[29]; /* Start Time (tick) PR       */
    pMTR->Phrase[7].dwSPt =          pLoad[30]; /* Stop  Time (tick) PR       */

    bSmafType = pSum[124];
    bTimeBase = pSum[125];
    bMaxCHNum = pSum[126];

    pMTR->pPhrase         = pMTR->pMTR;         /* MspI Info                  */
    pMTR->dwPlayTime      = pLoad[6];           /* Play Time(tick)            */
    pMTR->bTimeBase       = bTimeBase;          /* Time Base                  */

    pSmaf->bSmafType      = SMAF_TYPE_MA3;      /* SMAF Type                  */
    pSmaf->bTimeBase      = pMTR->bTimeBase;    /* Time Base                  */
    pSmaf->dwStartTime    = pLoad[15];          /* Start Point(tick)          */
    pSmaf->bMaxCHNum      = bMaxCHNum;          /* Number of Channel          */

    pDecodeBuffer[1]      = pMTR->pSeq;

    if (bSmafType) {
        bCompress[1]  = 1;
        dwSeqSize[6]  = pMTR->dwSeqSize;
    }
    else {
        bCompress[1]  = 0;
        dwSeqSize[6]  = Decode_init(1);
        bBitOffset[1] = (UINT8)Decode_count[1];
        bFirstByte[1] = (UINT8)Decode_bitbuf[1];
    }
    pSeq[6] = pDecodeBuffer[1];

    return F_TRUE;
}


/*--- Common B    Function ---------------------------------------------------*/
/*--- MaMmfCnv    API      ---------------------------------------------------*/

/*******************************************************************************
    Name        : MaMmfCnv_Initialize
    Description : Initialize SMAF Converter
    Parameter   : Nothing
    Return      :   0: Success
*******************************************************************************/
SINT32 MaMmfCnv_Initialize( void )
{
    Smaf_Info[0].pFile  = 0;
    Smaf_Info[0].dwSize = 0;
    Smaf_Info[0].bTimeBase  = 0;
    Smaf_Info[0].dwPlayTime = 0;
    Smaf_Info[1].pFile  = 0;
    Smaf_Info[1].dwSize = 0;
    Smaf_Info[1].bTimeBase  = 0;
    Smaf_Info[1].dwPlayTime = 0;
    MmfSeqID = -1;
    dwStatus =  0;

MAMMFCNV_DBGMSG((" ... MaMmfCnv_Initialize Fin\n"));

    return MASMW_SUCCESS;
}


/*******************************************************************************
    Name        : MaMmfCnv_End
    Description : End of SMAF Converter
    Parameter   : Nothing
    Return      :   0: Success
*******************************************************************************/
SINT32 MaMmfCnv_End( void )
{
    Smaf_Info[0].pFile  = 0;
    Smaf_Info[0].dwSize = 0;
    Smaf_Info[0].bTimeBase  = 0;
    Smaf_Info[0].dwPlayTime = 0;
    Smaf_Info[1].pFile  = 0;
    Smaf_Info[1].dwSize = 0;
    Smaf_Info[1].bTimeBase  = 0;
    Smaf_Info[1].dwPlayTime = 0;
    MmfSeqID = -1;
    dwStatus =  0;

MAMMFCNV_DBGMSG((" ... MaMmfCnv_End Fin\n"));

    return MASMW_SUCCESS;
}


/*******************************************************************************
    Name        : MaMmfCnv_Load
    Description : Load File Process
    Parameter   : UINT8  *file_ptr ... File Pointer
                : UINT32 file_size ... File Size
                : UINT8  mode      ... Load Mode
                : SINT32 *func     ... Pointer to Call Back Function
                : void   *ext_args ... Extension Argument
    Return      : >=0: Success
                : < 0: Error
*******************************************************************************/
SINT32 MaMmfCnv_Load(
    UINT8  *file_ptr, 
    UINT32 file_size, 
    UINT8  mode, 
    SINT32 (*func)(UINT8 id), 
    void   *ext_args
)
{
    SINT32 errstat;
    UINT32 dwCRC;
    UINT8  *pCRC;
    (void)ext_args;
    (void)func;


    if ((file_ptr == NULL) || (4 <= mode)  ||
        (file_size <= POSITION_OF_CONTENTSTYPE))
        return MMFCNV_ERR_ARGUMENT;

    if ((mode <= 1) && (dwStatus != 0))
        return F_FALSE;

    if ((mode == 0) && (Smaf_Info[1].pFile != 0)) {
        pCRC  = &(Smaf_Info[1].pFile[Smaf_Info[1].dwMMMDSize - 2]);
        dwCRC = (UINT32)((pCRC[0] << 8) + pCRC[1]);
        if ((Smaf_Info[1].dwCRC  == dwCRC) &&
            (Smaf_Info[1].pFile  == file_ptr) &&
            (Smaf_Info[1].dwSize == file_size)) {
            Smaf_Info[1].dwCRC = CRC_INITIAL;
            dwStatus = 1;
            return 1;                           /* Aleady Set Information     */
        }
    }

    errstat = Ma_Load(file_ptr, file_size, mode);

    if ((errstat == 1) && (mode <= 0x01))
        dwStatus = 1;


MAMMFCNV_DBGMSG((" ... MaMmfCnv_Load Fin  errorflag %d\n", errstat));

    return errstat;
}


/*******************************************************************************
    Name        : MaMmfCnv_Unload
    Description : Unload SMAF File Information
    Parameter   : SINT32 file_id   ... File ID
                : void   *ext_args ... Extension Argument
    Return      :   0: Success
                : < 0: Error
*******************************************************************************/
SINT32 MaMmfCnv_Unload(
    SINT32 file_id, 
    void * ext_args
)
{
    (void)ext_args;

    if ((Smaf_Info[1].pFile == 0) || (file_id != 1))
         return MASMW_ERROR_ARGUMENT;

    Smaf_Info[1].pFile  = 0;
    Smaf_Info[1].dwSize = 0;
    dwStatus = 0;

MAMMFCNV_DBGMSG((" ... MaMmfCnv_Unload Fin\n"));

    return MASMW_SUCCESS;
}


/*******************************************************************************
    Name        : MaMmfCnv_Open
    Description : Get Resource to Play
    Parameter   : SINT32 file_id   ... File ID
                : UINT16 mode      ... Load Mode
                : void   *ext_args ... Extension Argument
    Return      :   0: Success
                : < 0: Error
*******************************************************************************/
SINT32 MaMmfCnv_Open(
    SINT32 file_id,
    UINT16 mode,
    void *ext_args
)
{
    UINT8  bHwTimeBase;
    UINT8  bStream;
    UINT8  cnvmode;

    (void)mode;
    (void)ext_args;


    if ((file_id != 1) || (Smaf_Info[1].pFile == 0) || (16 < mode))
        return MASMW_ERROR_ARGUMENT;
    if (MmfSeqID >= 0)
        return F_FALSE;

    bStream = 0;
    cnvmode = 0;
    bAudio_LED = 0;

    switch (Smaf_Info[1].bSmafType) {
        case SMAF_TYPE_MA1:
            bMasterVol1 = 76;
            cnvmode = 0x1D;
            Get_LM_Status1();
            break;
        case SMAF_TYPE_MA2:
            bMasterVol1 = 0x7F;
            cnvmode = 0x1D;
            if (Smaf_Info[1].AudioTrack.pATR != 0)
                bStream = 1;
            Get_LM_Status2();
            break;
        case SMAF_TYPE_MA3:
            bMasterVol1 = 0x2D;
            if (Smaf_Info[1].bMaxCHNum == 16)
                cnvmode = (UINT8)(0x10 | CREATE_MODE);
            else
                cnvmode = (UINT8)(0x00 | CREATE_MODE);
            bStream = Stream_Reserve();
            Get_LM_Status3();
            break;
    }
    Play_Info.bStream = bStream;

    dwSeekTime  = 0;
    dwSeekTick  = 0;
    dwEndTime   = Smaf_Info[1].dwPlayTime * Smaf_Info[1].bTimeBase;
    dwEndTick   = Smaf_Info[1].dwPlayTime + Smaf_Info[1].dwStartTime;
    bMasterVol2 = 0x64;
    dwEventFlag = 0;

	dwTimeError = 0;

    if (Smaf_Info[1].bTimeBase > 10) {
        bHwTimeBase = 10;
        Play_Info.bTimeBaseR = (UINT8)(Smaf_Info[1].bTimeBase / 10);
    }
    else {
        bHwTimeBase = Smaf_Info[1].bTimeBase;
        Play_Info.bTimeBaseR = 1;
    }


    MmfSeqID = MaSndDrv_Create((UINT8)0x00, bHwTimeBase, cnvmode,
        (UINT8)0x00, bStream, MaMmfCnv_Convert, &dwRamAdr, &dwRamSize);

	MaSndDrv_SetVolume(MmfSeqID, bMasterVol2);


MAMMFCNV_DBGMSG((" ... MaMmfCnv_Open Fin  returnvalue %ld\n", MmfSeqID));

    if (MmfSeqID < 0) {
        MaSndDrv_Free(MmfSeqID);
        return MmfSeqID;
    }

    return F_TRUE;
}


/*******************************************************************************
    Name        : MaMmfCnv_Close
    Description : Open Resource
    Parameter   : SINT32 file_id   ... File ID
                : void   *ext_args ... Extension Argument
    Return      :   0: Success
                : < 0: Error
*******************************************************************************/
SINT32 MaMmfCnv_Close(
    SINT32 file_id, 
    void *ext_args
)
{
    SINT32 retval;

    (void)ext_args;

    if ((file_id != 1) || (Smaf_Info[1].pFile == 0))
        return MASMW_ERROR_ARGUMENT;
    if (MmfSeqID < 0)
        return F_FALSE;

    dwEventFlag = 0;
    bAudio_LED = 0;
    retval = MaSndDrv_Free(MmfSeqID);

MAMMFCNV_DBGMSG((" ... MaMmfCnv_Close Fin  returnvalue %ld\n", retval));

    if (retval < 0)
        return retval;

    MmfSeqID = -1;
    return F_TRUE;
}


/*******************************************************************************
    Name        : MaMmfCnv_Standby
    Description : Preparation to Play
    Parameter   : SINT32 file_id   ... File ID
                : void   *ext_args ... Extension Argument
    Return      :   0: Success
                : < 0: Error
*******************************************************************************/
SINT32 MaMmfCnv_Standby(
    SINT32 file_id,
    void *ext_args
)
{
    UINT8  i;

    (void)ext_args;


    if ((file_id != 1) || (Smaf_Info[1].pFile == 0))
        return MASMW_ERROR_ARGUMENT;
    if (MmfSeqID < 0)
        return F_FALSE;

    for (i = 0; i < 32; i++) {
        Wave_Info3[i].bNote = 0xFF;
        Wave_Info3[i].bPair = 0xFF;
    }

    switch (Smaf_Info[1].bSmafType) {
        case SMAF_TYPE_MA1:
        case SMAF_TYPE_MA2:
            Standby2();
            break;
        case SMAF_TYPE_MA3:
            Standby3();
            break;
    }
    for(i = 0; i < 16; i++){
        MaSndDrv_SetCommand(MmfSeqID, -1, MASNDDRV_CMD_CHANNEL_VOLUME, i, 0, 0);
        MaSndDrv_SetCommand(MmfSeqID, -1, MASNDDRV_CMD_PANPOT, i, 0x40, 0);
    }
    MaSndDrv_SetCommand(MmfSeqID, -1, MASNDDRV_CMD_MASTER_VOLUME,
		bMasterVol1, 0, 0);


MAMMFCNV_DBGMSG((" ... MaMmfCnv_Standby Fin\n"));
    return F_TRUE;
}


/*******************************************************************************
    Name        : MaMmfCnv_Seek
    Description : Seek Process
    Parameter   : SINT32 file_id   ... File ID
                : UINT32 pos       ... Seek Time (msec)
                : UINT8  flag      ... Mute Wait Flag
                : void   *ext_args ... Extension Argument
    Return      :   0: Success
                : < 0: Error
*******************************************************************************/
SINT32 MaMmfCnv_Seek(
    SINT32 file_id,
    UINT32 pos,
    UINT8  flag,
    void   *ext_args
)
{
    SINT32 retval;
    UINT8  i;
    PEVLIST pEvList;

    (void)flag;
    (void)ext_args;

    if ((file_id != 1) || (Smaf_Info[1].pFile == 0))
        return MASMW_ERROR_ARGUMENT;
    if (MmfSeqID < 0)
        return F_FALSE;

    dwEventFlag &= ~(UINT32)EVENT_EOS;

    for (i = 1; i < EVLIST_SIZE; i++) {         /* Initialize Event List      */
        Event_List[i].pNext = 0;
        Event_List[i-1].pNext = &Event_List[i];
    }
    for (i = 1; i < OFFLIST_SIZE; i++) {        /* Initialize Note OFF List   */
        Off_List[i].pNext = 0;
        Off_List[i-1].pNext = &Off_List[i];
    }
    Play_Info.pNextEv   = 0;                    /* Initialize Play Infomation */
    Play_Info.pEmptyEv  = (void *)Event_List;   /*                            */
    Play_Info.pNextOFF  = 0;                    /*                            */
    Play_Info.pEmptyOFF = (void *)Off_List;     /*                            */
    Play_Info.bEOS      = 0;                    /* Initialize Play Infomation */
    Play_Info.wNoteFlag = 0;
    Play_Info.wLED = 0;
    Play_Info.wMOTOR = 0;

	dwTimeError = 0;

    dwSeekTime = pos;
    if (dwSeekTime % Smaf_Info[1].bTimeBase)
        dwSeekTick = dwSeekTime / Smaf_Info[1].bTimeBase + 1;
    else
        dwSeekTick = dwSeekTime / Smaf_Info[1].bTimeBase;

    Play_Info.bPreEvent = 1;

    switch (Smaf_Info[1].bSmafType) {
        case SMAF_TYPE_MA1:
        case SMAF_TYPE_MA2:
            SetPreEvent2();
            break;
        case SMAF_TYPE_MA3:
            SetPreEvent3();
            break;
    }

    pEvList = Get_EvList();
    pEvList->dwTime = dwEndTick;
    pEvList->dwEvNo = MMF_EVENT_EOS;
    pEvList->dwSize = 0;
    Set_EvList(pEvList);

    retval = MaSndDrv_ControlSequencer(MmfSeqID, 4);

MAMMFCNV_DBGMSG((" ... MaMmfCnv_Seek Fin  returnvalue %ld\n", retval));
    return retval;
}


/*******************************************************************************
    Name        : MaMmfCnv_Start
    Description : PLAY START
    Parameter   : SINT32 file_id   ... File ID
                : void   *ext_args ... Extension Argument
    Return      :   0: Success
                : < 0: Error
*******************************************************************************/
SINT32 MaMmfCnv_Start(
    SINT32 file_id, 
    void *ext_args
)
{
    SINT32 retval;

    (void)ext_args;

MAMMFCNV_DBGMSG((" ... MaMmfCnv_Start\n"));

    if ((file_id != 1) || (Smaf_Info[1].pFile == 0))
        return MASMW_ERROR_ARGUMENT;
    if (MmfSeqID < 0)
        return F_FALSE;

    if (Play_Info.bEOS == 0xFF)
	    Play_Info.bEOS = 16;

	dwTimeError = (dwSeekTick * Smaf_Info[1].bTimeBase )- dwSeekTime;

    retval = MaSndDrv_ControlSequencer(MmfSeqID, 1);
MAMMFCNV_DBGMSG((" ... MaMmfCnv_Start Fin  returnvalue %ld\n", retval));
    return retval;
}


/*******************************************************************************
    Name        : MaMmfCnv_Stop
    Description : PLAY STOP
    Parameter   : SINT32 file_id   ... File ID
                : void   *ext_args ... Extension Argument
    Return      :   0: Success
                : < 0: Error
*******************************************************************************/
SINT32 MaMmfCnv_Stop(
    SINT32 file_id, 
    void *ext_args
)
{
    SINT32 retval;

    (void)ext_args;

MAMMFCNV_DBGMSG((" ... MaMmfCnv_Stop\n"));

    if ((file_id != 1) || (Smaf_Info[1].pFile == 0))
        return MASMW_ERROR_ARGUMENT;
    if (MmfSeqID < 0)
        return F_FALSE;

    retval = MaSndDrv_ControlSequencer(MmfSeqID, 0);
MAMMFCNV_DBGMSG((" ... MaMmfCnv_Stop Fin  returnvalue %ld\n", retval));
    return retval;
}


/*******************************************************************************
    Name        : MaMmfCnv_Convert
    Description : Event Convert Process
    Parameter   : Nothing
    Return      :   0 : Data End
                : > 0 : Number of Set Command
                : < 0 : Error
*******************************************************************************/
SINT32 MaMmfCnv_Convert( void )
{
    SINT32 slEvent;
    UINT32 dwOffTime, dwEvTime;

    if ((Smaf_Info[1].pFile == 0) || (MmfSeqID < 0))
        return F_FALSE;

    if (dwEventFlag) {
        slEvent = PushFlagEvent();
        if (slEvent)
            return slEvent;
    }
    if (Play_Info.bPreEvent) {
        slEvent = PushPreEvent3();
        if (slEvent)
            return slEvent;
    }
    if (Play_Info.pNextOFF != 0)
        dwOffTime = ((POFFLIST)(Play_Info.pNextOFF))->dwTime;
    else
        dwOffTime = 0xFFFFFFFF;
    if (Play_Info.pNextEv != 0)
        dwEvTime = ((PEVLIST)(Play_Info.pNextEv))->dwTime;
    else
        dwEvTime = 0xFFFFFFFF;

    if ((dwOffTime == 0xFFFFFFFF) && (dwEvTime == 0xFFFFFFFF)) {
        dwEvTime = (PLAY_TIME_MIN / Smaf_Info[1].bTimeBase);
        Play_Info.dwPastTime = dwEndTick - dwEvTime;
        dwEventFlag |= EVENT_EOS;
        return Set_EventEOS();
    }
    if (dwOffTime <= dwEvTime)
        slEvent = Set_NoteOffEvent();
    else
        slEvent = Set_Event();
    return slEvent;
}


/*******************************************************************************
    Name        : MaMmfCnv_Control
    Description : Receive Control Message Process
    Parameter   : SINT32 file_id   ... File ID
                : UINT8  ctrl_num  ... Message ID
                : void   *prm      ... Information Structure
                : void   *ext_args ... Extension Argument
    Return      : >=0: Success
                : < 0: Error
*******************************************************************************/
SINT32 MaMmfCnv_Control(
    SINT32 file_id,
    UINT8  ctrl_num,
    void   *prm,
    void   *ext_args
)
{
    UINT8  bTemp;
    SINT8  sbTemp;
    SINT32 slTemp;
    PMASMW_CONTENTSINFO pCnt;

    (void)ext_args;

    if ((file_id != 0) && (file_id != 1))
        return MMFCNV_ERR_ARGUMENT;
    if ((file_id == 0) &&
		(ctrl_num != MASMW_GET_LENGTH) &&
        (ctrl_num != MASMW_GET_CONTENTSDATA) &&
        (ctrl_num != MASMW_GET_PHRASELIST) &&
        (ctrl_num != MASMW_GET_LOADINFO) &&
        (ctrl_num != MASMW_SET_LOADINFO))
        return MMFCNV_ERR_ARGUMENT;
    if ((Smaf_Info[file_id].pFile == 0) &&
        (ctrl_num != MASMW_SET_LOADINFO))
        return MMFCNV_ERR_ARGUMENT;

    switch (ctrl_num) {
        case MASMW_SET_VOLUME:
            bTemp = *((UINT8 *)prm);
            if (bTemp > 127)
                return MMFCNV_ERR_ARGUMENT;
            bMasterVol2 = bTemp;
			MaSndDrv_SetVolume(MmfSeqID, bMasterVol2);
            return F_TRUE;

        case MASMW_SET_SPEED:
            bTemp = *((UINT8 *)prm);
            if ((70 <= bTemp) && (bTemp <= 130))
                return MaSndDrv_SetSpeed(MmfSeqID, bTemp);
            else
                return MMFCNV_ERR_ARGUMENT;

        case MASMW_SET_KEYCONTROL:
            sbTemp = *((SINT8 *)prm);
            if ((-12 <= sbTemp) && (sbTemp <= 12))
                return MaSndDrv_SetKey(MmfSeqID, sbTemp);
            else
                return MMFCNV_ERR_ARGUMENT;

        case MASMW_GET_TIMEERROR:
            return MaSndDrv_GetTimeError(MmfSeqID);

        case MASMW_GET_POSITION:
			slTemp = MaSndDrv_GetPos(MmfSeqID) + dwTimeError;
            return slTemp;

        case MASMW_GET_LENGTH:
			if (Smaf_Info[file_id].dwPlayTime == 0)
				return F_FALSE;
			else
				return (Smaf_Info[file_id].dwPlayTime * Smaf_Info[file_id].bTimeBase);

        case MASMW_GET_PHRASELIST:
            return GetPhraseList(prm, file_id);

        case MASMW_SET_ENDPOINT:
            dwEndTime = *((UINT32 *)prm);
            dwEventFlag |= EVENT_ENDTIME;
            break;

        case MASMW_GET_CONTENTSDATA:
            pCnt = (PMASMW_CONTENTSINFO)prm;
            return GetContentsData(file_id, pCnt);

        case MASMW_GET_LEDSTATUS:
            for (bTemp = 0; bTemp < 16; bTemp++) {
                if (ChInfo[bTemp].bLed == 1)
                    break;
            }
            if ((bTemp == 16) && (bAudio_LED == 0))
                bTemp = 0;
            else
                bTemp = 1;
            slTemp = MaSndDrv_DeviceControl(MASMW_GET_LEDSTATUS, bTemp, 0, 0);
            return (slTemp & (SINT32)bTemp);

         case MASMW_GET_VIBSTATUS:
            for (bTemp = 0; bTemp < 16; bTemp++) {
                if (ChInfo[bTemp].bMotor == 1)
                    break;
            }
            if (bTemp == 16)
                bTemp = 0;
            else
                bTemp = 1;
            slTemp = MaSndDrv_DeviceControl(MASMW_GET_VIBSTATUS, bTemp, 0, 0);
            return (slTemp & (SINT32)bTemp);

         case MASMW_GET_LOADINFO:
            slTemp = Get_LoadInfo(file_id, prm);
            return slTemp;

         case MASMW_SET_LOADINFO:
            slTemp = Set_LoadInfo(prm);
            return slTemp;

    }
    return F_TRUE;
}
