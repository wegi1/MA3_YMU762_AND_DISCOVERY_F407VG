/****************************************************************************
 *																			*
 *		Copyright (C) 2001-2003 YAMAHA CORPORATION. All rights reserved.	*
 *																			*
 *		Module		: mamachdep.h											*
 *																			*
 *		Description	: Define machine dependent types for MA SMW				*
 *																			*
 * 		Version		: 1.3.15.4	2003.04.07									*
 *																			*
 ****************************************************************************/

#ifndef __MAMACHDEP_H__
#define __MAMACHDEP_H__

#define SINT8 signed char
#define SINT16 signed short
#define SINT32 signed long
#define UINT8 unsigned char
#define UINT16 unsigned short
#define UINT32 unsigned long

///////////////////////////////////////////////////
#pragma pack(1)
// 4 bytes
typedef struct
{
    UINT16 ushCnt;
    UINT8 flag;
    UINT8 data;
} YMU262_LOG_ENTRY;
#pragma pack()

#define LOG_ENTRIES_MAX 8192
///////////////////////////////////////////////////

#ifndef NULL
 #define  NULL           ((void *)0)
#endif

#define MA_STOPWAIT			(0) /* default 1*/
#define MA_PLAYMODE_CHECK	(0)

#define MA_STATUS_REG		(*(volatile unsigned char*) 0x60000000)
#define MA_DATA_REG		(*(volatile unsigned char*) 0x60010000)

#if 0
#define	MA_ADJUST1_VALUE	(29)	/* register bank 1, ID #5 */
#define	MA_ADJUST2_VALUE	(112)	/* register bank 1, ID #6 */
#define	MA_PLL_OUT		(55298)	/* PLL Output value (kHz) ex.:55296 */
									/* (CLKI / MA3_ADJUST1_VALUE) * MA3_ADJUST2_VALUE */
#else
#define	MA_ADJUST1_VALUE	(2)	/* register bank 1, ID #5 */
#define	MA_ADJUST2_VALUE	(9)	/* register bank 1, ID #6 */
#define	MA_PLL_OUT		(55296)	/* PLL Output value (kHz) ex.:55296 */
									/* (CLKI / MA3_ADJUST1_VALUE) * MA3_ADJUST2_VALUE */
#endif

#define MA_VSEL			(1)	/* VSEL2:VSEL1 */
//#define MA_STATUS_TIMEOUT	(???)	/* 1.5msec */

#if 1
 #define MA_INT_POINT		(4)
 #define MA_FIFO_SIZE		(256)
 #define MA_SBUF_NUM		(4)
#else
 #define MA_INT_POINT		(6)
 #define MA_FIFO_SIZE		(128)
 #define MA_SBUF_NUM		(8)
#endif

/* Select stream converter */
#define MASMW_SRMCNV_SMAF			1
#define MASMW_SRMCNV_SMAF_PHRASE	1
#define MASMW_SRMCNV_REALTIME_MIDI	1
#define MASMW_SRMCNV_SMAF_AUDIO		1
#define MASMW_SRMCNV_SMF			1
#define MASMW_SRMCNV_WAV			1

/*===================================================================*/
/* Select default FM mode                                            */
/*    FM_2OP_MODE : FM 32-Voice (2-OP FM Only)                       */
/*    FM_4OP_MODE : FM 16-Voice (2-OP/4-OP FM)                       */
/*                                                                   */
/*   RMD_DEF_FM_MODE : Realtime MIDI play                            */
/*   SMF_DEF_FM_MODE : SMF play                                      */
/*                                                                   */
/*   Default setting is FM_4OP_MODE.                                 */
/*===================================================================*/

#define FM_2OP_MODE				0
#define FM_4OP_MODE				0x10

#define RMD_DEF_FM_MODE			FM_4OP_MODE			/* 16-Voice mode */
#define SMF_DEF_FM_MODE			FM_4OP_MODE			/* 16-Voice mode */

/*===================================================================*/
/* Select default NoteOn Velocity mode                               */
/*    VELOCITY_20LOG_MODE : 20 * Log (velocity / 127)                */
/*    VELOCITY_40LOG_MODE : 40 * Log (velocity / 127)                */
/*                                                                   */
/*   RMD_DEF_VELOCITY_MODE : Realtime MIDI play                      */
/*   SMF_DEF_VELOCITY_MODE : SMF play                                */
/*                                                                   */
/*===================================================================*/

#define VELOCITY_20LOG_MODE			(0)
#define VELOCITY_40LOG_MODE			(1)

#define RMD_DEF_VELOCITY_MODE		VELOCITY_40LOG_MODE			/* 40Log mode */
#define SMF_DEF_VELOCITY_MODE		VELOCITY_40LOG_MODE			/* 40Log mode */

/*===================================================================*/
/* Select default voice Map @ 4-OP mode.                             */
/*    VOICE_2OP_MAP  : 2OP-Voice (ROM-Voice)                         */
/*    VOICE_4OP_MAP  : 4OP-Voice                                     */
/*                                                                   */
/*   SMF_DEF_VOICE_MAP : SMF play                                    */
/*                                                                   */
/*   Default setting is VOICE_2OP_MAP.                               */
/*===================================================================*/

#define VOICE_2OP_MAP			(0)						/* ROM-VOICE */
#define VOICE_4OP_MAP			(1)						/* 4OP-VOICE */

#define SMF_DEF_VOICE_MAP		VOICE_2OP_MAP

/*===================================================================*/
/* Select enable or diasable lookup the KCS of SMAF/MA-1/3 data.     */
/*    '0' : enable (default)                                         */
/*    '1' : disable, always effects key control                      */
/*===================================================================*/
#define	MA13_KCS_IGNORE			(0)

#endif /*__MAMACHDEP_H__*/
