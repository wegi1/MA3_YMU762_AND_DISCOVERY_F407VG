/*==============================================================================
//	Copyright (C) 2001-2003 YAMAHA CORPORATION
//
//	Title		: MAMIDCNV.C
//
//	Description	: MA-3 SMF/GML Stream Converter Module.
//
//	Version		: 1.10.2.2
//
//	History		:
//				  May 8, 2001			1st try.
//				  July 9, 2001			Fix bugs in MaMidCnv_Control().
//				  July 16, 2001			Fix warnings.
//				  July 18, 2001			Add #BankMSB = 0x78/0x79.
//				  July 24, 2001			Fix MidVolTable.
//				  July 31, 2001			Change MonoModeOn message.
//				  September 18, 2001	Fix a bug of counting time.
//				  September 25, 2001	Add format-1 support.
//				  October 19, 2001		Change the way of setup.
//				  Nobember 2, 2001		Change NoteOn(vel=0) => NoteOff().
//				  Nobember 26, 2001		Add MA3_SMF_EXMSG_SUPPORT option.
//				  May 15, 2002			Add MasterVolume(Universal SysEx).
//				  May 31, 2002			Add SP-MIDI functions.
//				  July 2, 2002			Add 4op Voice functions.
//				  Oct 28, 2002			Modify Control(MASMW_GET_LENGTH).
//				  2003/04/18			Fix gbMuteVoice[].
//============================================================================*/
#include "mamidcnv.h"

#define MA3_SMF_EXMSG_SUPPORT		(0)		/* 1 : Support Ex-Msg      */

#define	SMF_TIMEBASE				10		/* [ms]                    */
#define	MAX_SMF_MESSAGES			192
#define	MAX_SMF_TRACKS				17
#define SMF_MAX_GAIN				76		/* - 9[dB] : 76             */
											/* -12[dB] : 64             */
											/* -18[dB] : 45             */
#define	MINIMUM_LENGTH				(20)

#define	MAMIDCNV_MAX_CHANNELS		16
#define	DVA_NORMAL					0
#define	DVA_SIMPLE					2
#define	DVA_MONO					4
#define	DRUM_NORMAL					0
#define	DRUM_SIMPLE					1
#define	MELODY_NORMAL				0
#define	MELODY_SIMPLE				8
#define DRUM_GML1					0x20
#define ENC_8BIT					0
#define ENC_7BIT					1


/*=============================================================================*/
/* Default Voices                                                              */
/*=============================================================================*/

static const UINT8 Drum_4op_Voice[61][32] = {
	{ 0x00,0x48,0x79,0x65,0xF8,0xF0,0xF0,0xE0,
	  0x44,0x60,0x26,0xF9,0xF8,0xFC,0x00,0x44,
	  0x50,0x00,0xF8,0xFB,0xFB,0x9C,0x44,0xA0,
	  0x30,0xF9,0xFB,0xEF,0x04,0x44,0x50,0x00 },	/* pe:1 sus:0 xof:1 eam:0 evb:0 Seq Click H */
	{ 0x00,0x41,0x79,0x65,0x08,0x08,0xF3,0x00,
	  0x44,0x50,0x07,0x88,0xB8,0x9B,0x02,0x44,
	  0x00,0x00,0x68,0xE8,0xF3,0x90,0x44,0xC0,
	  0x30,0x88,0x8D,0xC0,0xB2,0x44,0x10,0x30 },	/* pe:1 sus:0 xof:1 eam:0 evb:0 Brush Tap */
	{ 0x00,0x2C,0x79,0x65,0x00,0x65,0xF0,0x00,
	  0x42,0x00,0x97,0x02,0xA8,0x96,0x00,0x66,
	  0x00,0x19,0x00,0xF0,0xFF,0x00,0x00,0x00,
	  0x87,0x30,0xF6,0x36,0x54,0x64,0x10,0x00 },	/* pe:1 sus:1 xof:0 eam:0 evb:0 Brush Swirl */
	{ 0x00,0x45,0x79,0x65,0x08,0x00,0xF0,0x00,
	  0x44,0x40,0x07,0x68,0xC8,0xC9,0x00,0x44,
	  0xE0,0x68,0x48,0x7A,0xBD,0x24,0x44,0x00,
	  0x38,0xB8,0xB8,0x8D,0x02,0x44,0x00,0x00 },	/* pe:1 sus:0 xof:1 eam:0 evb:0 Brush Slap */
	{ 0x00,0x31,0x79,0x65,0x00,0x35,0xF0,0x00,
	  0x42,0x55,0x97,0x02,0xA6,0x96,0x00,0x66,
	  0x30,0x19,0x00,0xF0,0xFF,0x00,0x00,0x90,
	  0x87,0x30,0xF6,0x36,0x40,0x64,0x30,0xA0 },	/* pe:1 sus:1 xof:0 eam:0 evb:0 Brush Tap Swirl */
	{ 0x00,0x12,0x79,0x25,0x00,0x20,0x70,0x34,
	  0x24,0x83,0x17,0x00,0x95,0xF3,0x00,0x03,
	  0x06,0xD0,0x80,0x4A,0xF3,0x20,0x44,0x30,
	  0x00,0x70,0x76,0xE5,0x00,0x44,0x00,0x00 },	/* pe:1 sus:0 xof:0 eam:0 evb:1 Snare Roll */
	{ 0x00,0x57,0x41,0x65,0x58,0x97,0xFF,0x08,
	  0x44,0x70,0x0E,0x59,0xF8,0xAF,0x00,0x44,
	  0x50,0x30,0x60,0x55,0xF0,0x9C,0x44,0x20,
	  0x28,0x90,0x9A,0xCA,0x50,0x44,0x50,0x00 },	/* pe:1 sus:0 xof:1 eam:0 evb:0 Castanet */
	{ 0x01,0x3E,0x80,0x79,0x00,0x08,0xF0,0xF0,
	  0x10,0x00,0x00,0x00,0x0B,0x9B,0x0B,0x9B,
	  0x81,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 },	/* pe:1 sus:0 xof:1 eam:0 evb:0 Snare L */
	{ 0x00,0x3B,0x79,0x66,0xC8,0x89,0xF9,0x01,
	  0x44,0xF0,0xA7,0x28,0x8A,0xDB,0x00,0x44,
	  0xA0,0x18,0x28,0x58,0xD5,0x55,0x44,0xB0,
	  0x18,0x98,0x7B,0xCB,0x00,0x44,0xD0,0x40 },	/* pe:1 sus:0 xof:1 eam:0 evb:0 Sticks */
	{ 0x01,0x23,0x28,0x79,0x00,0x08,0xF0,0xF0,
	  0x10,0x00,0x00,0x00,0x03,0xA9,0x03,0xA9,
	  0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 },	/* pe:1 sus:0 xof:1 eam:0 evb:0 Bass Drum L */
	{ 0x00,0x12,0x79,0x65,0x59,0x50,0xF0,0x14,
	  0x44,0xC0,0x17,0x79,0x77,0xF7,0x00,0x44,
	  0xB0,0x00,0x68,0x6A,0xF8,0x00,0x44,0xC0,
	  0x00,0x78,0x77,0xF7,0x00,0x44,0x70,0x10 },	/* pe:1 sus:0 xof:1 eam:0 evb:0 Open Rim Shot */
	{ 0x01,0x27,0x10,0x79,0x00,0x08,0xF0,0xF0,
	  0x10,0x00,0x00,0x00,0x03,0xA9,0x03,0xA9,
	  0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 },	/* pe:1 sus:0 xof:1 eam:0 evb:0 Bass Drum M */
	{ 0x01,0x2E,0xE0,0x79,0x00,0x08,0xF0,0xF0,
	  0x10,0x00,0x00,0x00,0x03,0xA9,0x03,0xA9,
	  0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 },	/* pe:1 sus:0 xof:1 eam:0 evb:0 Bass Drum H */
	{ 0x00,0x34,0x79,0x65,0x08,0x82,0xF5,0x20,
	  0x44,0xA0,0x0E,0x39,0x39,0xDF,0x00,0x44,
	  0x70,0x30,0x08,0x90,0xBD,0x00,0x44,0x90,
	  0x97,0xB0,0x99,0xDB,0x00,0x44,0x00,0x90 },	/* pe:1 sus:0 xof:1 eam:0 evb:0 Closed Rim Shot */
	{ 0x01,0x4A,0x38,0x79,0x00,0x08,0xF0,0xF0,
	  0x10,0x00,0x00,0x00,0x0B,0x9B,0x0B,0x9B,
	  0x81,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 },	/* pe:1 sus:0 xof:1 eam:0 evb:0 Snare M */
	{ 0x00,0x19,0x69,0x65,0x48,0x64,0xF0,0x08,
	  0x44,0x00,0x07,0x98,0x46,0xF1,0x00,0x44,
	  0x00,0x28,0x08,0xF8,0xF1,0x02,0x44,0xD1,
	  0xDA,0x98,0x8A,0xF5,0x02,0x44,0xF0,0x10 },	/* pe:1 sus:0 xof:1 eam:0 evb:0 Hand Clap */
	{ 0x01,0x59,0xD8,0x79,0x00,0x08,0xF0,0xF0,
	  0x10,0x00,0x00,0x00,0x0B,0x9B,0x0B,0x9B,
	  0x81,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 },	/* pe:1 sus:0 xof:1 eam:0 evb:0 Snare H */
	{ 0x01,0x1F,0x40,0x41,0x00,0x58,0xF0,0xF0,
	  0x10,0x00,0x00,0x00,0x0A,0xD4,0x0D,0xC5,
	  0x82,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 },	/* pe:1 sus:0 xof:1 eam:0 evb:0 Floor Tom L */
	{ 0x01,0x3A,0x98,0xA9,0x00,0x08,0xF0,0xF0,
	  0x40,0x00,0x00,0x00,0x04,0xD7,0x04,0xD7,
	  0x83,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 },	/* pe:1 sus:0 xof:1 eam:0 evb:0 Hi-Hat Closed */
	{ 0x01,0x27,0x10,0x59,0x00,0x58,0xF0,0xF0,
	  0x10,0x00,0x00,0x00,0x0A,0xD4,0x0D,0xC5,
	  0x82,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 },	/* pe:1 sus:0 xof:1 eam:0 evb:0 Floor Tom H */
	{ 0x01,0x36,0xB0,0xA9,0x00,0x08,0xF0,0xF0,
	  0x40,0x00,0x00,0x00,0x04,0xD7,0x04,0xD7,
	  0x83,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 },	/* pe:1 sus:0 xof:1 eam:0 evb:0 Hi-Hat Pedal */
	{ 0x01,0x30,0xD4,0x71,0x00,0x58,0xF0,0xF0,
	  0x10,0x00,0x00,0x00,0x0A,0xD4,0x0D,0xC5,
	  0x82,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 },	/* pe:1 sus:0 xof:1 eam:0 evb:0 Low Tom */
	{ 0x01,0x3C,0x8C,0xA9,0x00,0x08,0xF0,0xF0,
	  0x30,0x00,0x00,0x00,0x0C,0xFB,0x0C,0xFB,
	  0x84,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 },	/* pe:1 sus:0 xof:1 eam:0 evb:0 Hi-Hat Open */
	{ 0x01,0x3A,0x98,0x91,0x00,0x58,0xF0,0xF0,
	  0x10,0x00,0x00,0x00,0x0A,0xD4,0x0D,0xC5,
	  0x82,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 },	/* pe:1 sus:0 xof:1 eam:0 evb:0 Mid Tom L */
	{ 0x01,0x46,0x50,0xA9,0x00,0x58,0xF0,0xF0,
	  0x10,0x00,0x00,0x00,0x0A,0xD4,0x0D,0xC5,
	  0x82,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 },	/* pe:1 sus:0 xof:1 eam:0 evb:0 Mid Tom H */
	{ 0x01,0x33,0x90,0x59,0x00,0x58,0xF0,0xF0,
	  0x00,0x00,0x00,0x00,0x0D,0xF9,0x15,0xDB,
	  0x86,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 },	/* pe:1 sus:0 xof:1 eam:0 evb:0 Crash Cymbal 1 */
	{ 0x01,0x52,0x08,0xC1,0x00,0x58,0xF0,0xF0,
	  0x10,0x00,0x00,0x00,0x0A,0xD4,0x0D,0xC5,
	  0x82,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 },	/* pe:1 sus:0 xof:1 eam:0 evb:0 High Tom */
	{ 0x01,0x3E,0x80,0x59,0x00,0x58,0xF0,0xF0,
	  0x30,0x00,0x00,0x00,0x06,0x3A,0x12,0xC0,
	  0x85,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 },	/* pe:1 sus:0 xof:1 eam:0 evb:0 Ride Cymbal 1 */
	{ 0x01,0x23,0x28,0x59,0x00,0x58,0xF0,0xF0,
	  0x10,0x00,0x00,0x00,0x0D,0xF9,0x15,0xDB,
	  0x86,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 },	/* pe:1 sus:0 xof:1 eam:0 evb:0 Chinese Cymbal */
	{ 0x00,0x4D,0x59,0x25,0x08,0x07,0xF4,0x5A,
	  0x44,0xF0,0x9F,0x48,0x45,0xFE,0x00,0x44,
	  0xF0,0x60,0x08,0x00,0xF4,0x32,0x44,0xC0,
	  0x9F,0x28,0x55,0xFD,0x6C,0x44,0xF0,0x58 },	/* pe:1 sus:0 xof:1 eam:0 evb:0 Ride Cymbal Cup */
	{ 0x00,0x6D,0x91,0x65,0x48,0x17,0x82,0x28,
	  0x44,0xB0,0x06,0xC8,0xB7,0xE5,0x00,0x44,
	  0x50,0x68,0x50,0x27,0x82,0x00,0x44,0xB0,
	  0x40,0xC0,0x77,0xD6,0x00,0x44,0xF0,0x40 },	/* pe:1 sus:0 xof:1 eam:0 evb:0 Tambourine */
	{ 0x01,0x52,0x08,0x69,0x00,0x58,0xF0,0xF0,
	  0x28,0x00,0x00,0x00,0x0D,0xF9,0x15,0xDB,
	  0x86,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 },	/* pe:1 sus:0 xof:1 eam:0 evb:0 Splash Cymbal */
	{ 0x00,0x54,0xA9,0x65,0x89,0x59,0xF3,0x02,
	  0x44,0x30,0x04,0x59,0x37,0xA3,0x00,0x44,
	  0x00,0x00,0xB9,0x9F,0xFA,0x0C,0x44,0x40,
	  0x10,0x59,0x5C,0xF3,0x80,0x44,0x10,0x00 },	/* pe:1 sus:0 xof:1 eam:0 evb:0 Cowbell */
	{ 0x01,0x36,0xB0,0x59,0x00,0x58,0xF0,0xF0,
	  0x00,0x00,0x00,0x00,0x0D,0xF9,0x15,0xDB,
	  0x86,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 },	/* pe:1 sus:0 xof:1 eam:0 evb:0 Crash Cymbal 2 */
	{ 0x00,0x22,0x39,0x25,0x28,0x61,0xD0,0x00,
	  0x24,0x12,0xEF,0x58,0x96,0xF2,0x00,0x02,
	  0x06,0x40,0x40,0x45,0xF0,0x74,0x44,0x60,
	  0xEF,0x58,0x46,0xF0,0x00,0x44,0x00,0xF0 },	/* pe:1 sus:0 xof:1 eam:0 evb:0 Vibraslap */
	{ 0x01,0x46,0x50,0x59,0x00,0x58,0xF0,0xF0,
	  0x30,0x00,0x00,0x00,0x06,0x3A,0x12,0xC0,
	  0x85,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00 },	/* pe:1 sus:0 xof:1 eam:0 evb:0 Ride Cymbal 2 */
	{ 0x00,0x24,0xC1,0x65,0xC8,0xC5,0xF0,0x00,
	  0x44,0x60,0x03,0x88,0x75,0xF0,0x00,0x44,
	  0xB0,0x00,0xC8,0xC5,0xF0,0x08,0x44,0xF0,
	  0x00,0x78,0x34,0xF0,0x00,0x44,0xF0,0x00 },	/* pe:1 sus:0 xof:1 eam:0 evb:0 Bongo H */
	{ 0x00,0x20,0xC1,0x65,0xC8,0xC5,0xF0,0x00,
	  0x44,0x60,0x03,0x88,0x75,0xF0,0x1C,0x44,
	  0xB0,0x00,0xC8,0xC5,0xF0,0x18,0x44,0xF0,
	  0x00,0x88,0x35,0xF0,0x00,0x44,0xF0,0x00 },	/* pe:1 sus:0 xof:1 eam:0 evb:0 Bongo L */
	{ 0x00,0x4C,0x49,0xE5,0xA8,0xAE,0xE1,0x18,
	  0x46,0x02,0x04,0x98,0xA8,0xF1,0x00,0x44,
	  0x10,0x40,0x00,0x00,0xF0,0xB0,0x44,0x00,
	  0x00,0x90,0xAD,0xF1,0x1C,0x44,0x00,0x00 },	/* pe:1 sus:0 xof:1 eam:0 evb:0 Conga H Mute */
	{ 0x00,0x13,0x49,0x65,0x58,0x8A,0xAC,0x98,
	  0x44,0xA0,0x1F,0x78,0x96,0xC4,0x00,0x44,
	  0xC0,0x00,0x68,0x8A,0xF8,0x00,0x44,0xC0,
	  0x00,0x78,0x97,0xF7,0x00,0x44,0x20,0x10 },	/* pe:1 sus:0 xof:1 eam:0 evb:0 Conga H Open */
	{ 0x00,0x0E,0x59,0x65,0x58,0x8A,0xAC,0x7C,
	  0x44,0xA0,0x1F,0x68,0x95,0xC4,0x00,0x44,
	  0xC0,0x00,0x68,0x8A,0xF8,0x00,0x44,0xC0,
	  0x00,0x78,0x97,0xF7,0x00,0x44,0x20,0x50 },	/* pe:1 sus:0 xof:1 eam:0 evb:0 Conga L */
	{ 0x00,0x33,0xA9,0x63,0x08,0x87,0xC0,0x58,
	  0x44,0x20,0x06,0x68,0x89,0xF0,0xC4,0x44,
	  0x70,0x00,0xA8,0x66,0xD1,0x68,0x44,0x90,
	  0x20,0x68,0x89,0xF0,0x18,0x44,0x90,0x30 },	/* pe:1 sus:0 xof:1 eam:0 evb:0 Timbale H */
	{ 0x00,0x1C,0xA9,0x63,0x08,0x87,0xC0,0x58,
	  0x44,0x20,0x06,0x68,0x89,0xF0,0xC4,0x44,
	  0x70,0x00,0xA8,0x66,0xD1,0x5C,0x44,0xC0,
	  0x20,0x68,0x89,0xF0,0x10,0x44,0x90,0x30 },	/* pe:1 sus:0 xof:1 eam:0 evb:0 Timbale L */
	{ 0x00,0x4D,0x39,0x65,0x68,0x6C,0xE2,0x54,
	  0x44,0x70,0x05,0x68,0x68,0xF1,0x28,0x44,
	  0x20,0x00,0x68,0x5C,0xE2,0x68,0x44,0x70,
	  0x00,0x68,0x68,0xF1,0x28,0x44,0x20,0x00 },	/* pe:1 sus:0 xof:1 eam:0 evb:0 Agogo H */
	{ 0x00,0x48,0x39,0x65,0x68,0x6C,0xE2,0x54,
	  0x44,0x70,0x05,0x68,0x68,0xF1,0x28,0x44,
	  0x20,0x00,0x58,0x5C,0xE2,0x34,0x44,0x70,
	  0x00,0x68,0x68,0xF1,0x74,0x44,0x20,0x00 },	/* pe:1 sus:0 xof:1 eam:0 evb:0 Agogo L */
	{ 0x00,0x49,0x39,0x63,0x38,0x50,0xF6,0x00,
	  0x44,0x70,0x07,0x49,0x48,0xEA,0x60,0x44,
	  0xF0,0x00,0x28,0x64,0xE4,0x00,0x44,0xF0,
	  0x00,0x98,0x4B,0x80,0x08,0x44,0xF0,0x30 },	/* pe:1 sus:0 xof:1 eam:0 evb:0 Cabasa */
	{ 0x00,0x74,0x31,0x64,0x38,0x50,0xF6,0x10,
	  0x44,0xF0,0x07,0x49,0x48,0x86,0x0C,0x44,
	  0xF0,0x00,0x38,0x39,0xCF,0x00,0x44,0xF0,
	  0x30,0xA8,0x3A,0x78,0x00,0x44,0xF0,0x60 },	/* pe:1 sus:0 xof:1 eam:0 evb:0 Maracas */
	{ 0x00,0x32,0xC1,0xE5,0x00,0x70,0xF0,0x28,
	  0x20,0x00,0x0D,0x02,0xC1,0xC1,0xAC,0x21,
	  0xE0,0x00,0x02,0x70,0xF0,0x5C,0x47,0x00,
	  0x02,0x02,0xC1,0x80,0x18,0x20,0xF0,0x00 },	/* pe:1 sus:1 xof:0 eam:0 evb:1 Samba Whistle H */
	{ 0x00,0x30,0xC1,0xE5,0x00,0x70,0xF0,0x28,
	  0x20,0x00,0x0D,0x02,0xC1,0xC1,0xAC,0x21,
	  0xE0,0x00,0x02,0x70,0xF0,0x5C,0x47,0x00,
	  0x02,0x02,0xC1,0x80,0x18,0x20,0xF0,0x00 },	/* pe:1 sus:1 xof:0 eam:0 evb:1 Samba Whistle L */
	{ 0x00,0x35,0xE1,0x43,0x68,0x02,0xC0,0x30,
	  0x00,0x10,0x0F,0x68,0xA3,0xC0,0x00,0x00,
	  0x30,0x20,0x08,0x61,0xD0,0x80,0x00,0xE5,
	  0x5F,0xC9,0xF7,0xF6,0x00,0x00,0x01,0x38 },	/* pe:0 sus:0 xof:1 eam:0 evb:0 Guiro Short */
	{ 0x00,0x2B,0xE1,0x43,0x68,0x02,0xC0,0x38,
	  0x00,0x10,0x0F,0x68,0xA3,0xC0,0x00,0x00,
	  0x30,0x20,0x08,0x60,0xF0,0x80,0x00,0xD0,
	  0x5F,0xE8,0xC6,0x96,0x14,0x00,0x02,0x38 },	/* pe:0 sus:0 xof:1 eam:0 evb:0 Guiro Long */
	{ 0x00,0x32,0xA9,0x64,0x68,0x67,0xDA,0x2C,
	  0x44,0x70,0x00,0xB8,0x3E,0xFA,0x34,0x44,
	  0x70,0x00,0x89,0x0B,0xFC,0x00,0x44,0x00,
	  0x00,0x89,0x83,0xF0,0x00,0x44,0xF0,0x40 },	/* pe:1 sus:0 xof:1 eam:0 evb:0 Claves */
	{ 0x00,0x42,0xC1,0x64,0x68,0x66,0xEA,0xFC,
	  0x44,0xF0,0x00,0x68,0x56,0xFB,0x00,0x44,
	  0x60,0x00,0x09,0xAC,0xDC,0x00,0x44,0x70,
	  0x10,0xC9,0x77,0xF4,0x00,0x44,0x30,0x40 },	/* pe:1 sus:0 xof:1 eam:0 evb:0 Wood Block H */
	{ 0x00,0x3C,0xC1,0x64,0x68,0x66,0xEA,0xFC,
	  0x44,0xF0,0x00,0x68,0x56,0xFB,0x00,0x44,
	  0x60,0x00,0x09,0xAA,0xFC,0x40,0x44,0xA0,
	  0x10,0xC9,0xC7,0xC4,0x00,0x44,0x30,0x80 },	/* pe:1 sus:0 xof:1 eam:0 evb:0 Wood Block L */
	{ 0x00,0x30,0x59,0x65,0xC8,0xDA,0xDB,0x3E,
	  0x44,0x00,0x00,0x68,0x50,0x60,0x18,0x44,
	  0x60,0x00,0x88,0xC8,0xF0,0x38,0x44,0x00,
	  0x00,0x78,0x70,0x60,0x08,0x44,0x60,0x00 },	/* pe:1 sus:0 xof:1 eam:0 evb:0 Cuica Mute */
	{ 0x00,0x10,0x59,0xE5,0xF8,0x88,0x9F,0x30,
	  0x00,0x00,0x81,0x88,0x86,0x74,0x00,0x00,
	  0xE0,0x40,0x00,0x07,0xF0,0x90,0x44,0x20,
	  0x00,0xB0,0xB7,0xDC,0x48,0x44,0xB0,0x00 },	/* pe:1 sus:0 xof:1 eam:0 evb:0 Cuica Open */
	{ 0x00,0x6E,0x31,0x67,0x88,0x4C,0xF0,0x14,
	  0x44,0x90,0xB0,0x98,0x55,0xB6,0x00,0x44,
	  0xF0,0x20,0xC8,0x7B,0xF3,0x1C,0x44,0xE0,
	  0x30,0x88,0xBD,0xEF,0x18,0x44,0x60,0x00 },	/* pe:1 sus:0 xof:1 eam:0 evb:0 Triangle Mute */
	{ 0x00,0x6E,0x31,0x67,0x58,0x45,0xF0,0x0C,
	  0x44,0x90,0xB0,0x98,0x55,0xB6,0x00,0x44,
	  0xF0,0x20,0x98,0x79,0xF3,0x20,0x44,0xE0,
	  0x30,0x58,0xB5,0xEF,0x0C,0x44,0x60,0x00 },	/* pe:1 sus:0 xof:1 eam:0 evb:0 Triangle Open */
	{ 0x00,0x27,0xB9,0x64,0x38,0x50,0x96,0x00,
	  0x44,0x70,0x07,0x49,0x46,0x96,0x10,0x44,
	  0x30,0x00,0x28,0x64,0xE4,0x04,0x44,0xC0,
	  0x00,0x98,0x4A,0x98,0x08,0x44,0xF0,0x30 },	/* pe:1 sus:0 xof:1 eam:0 evb:0 Shaker */
	{ 0x00,0x5C,0xC1,0x65,0x08,0x06,0x70,0x50,
	  0x44,0x60,0x10,0x58,0x5B,0x64,0x00,0x44,
	  0x20,0x18,0x0B,0x20,0x30,0x1C,0x54,0x60,
	  0x10,0x2A,0x65,0x7F,0x00,0x44,0x90,0x10 },	/* pe:1 sus:1 xof:1 eam:1 evb:0 Jingle Bells */
	{ 0x00,0x64,0xD1,0x65,0x28,0x04,0xC0,0x60,
	  0x46,0x30,0x9F,0x58,0x53,0x44,0x00,0x66,
	  0x20,0x10,0x0B,0x20,0x30,0x1C,0x77,0x70,
	  0x10,0x2A,0x64,0x56,0x08,0x66,0x30,0x68 }	/* pe:1 sus:1 xof:1 eam:1 evb:1 Bell Tree */
};

static const UINT8 FM_4op_Voice[128][32] =
{
	{ 0x00,0x00,0x79,0x43,0x02,0x67,0xFF,0x9D,
	  0x00,0x10,0x40,0x23,0x33,0xE2,0x73,0x00,
	  0x50,0x00,0x12,0x41,0xD3,0x58,0x01,0x10,
	  0x00,0x23,0x63,0xD4,0x02,0x01,0x10,0x00 },	/* pe:0 sus:1 xof:0 eam:0 evb:1 GrandPno */
	{ 0x00,0x00,0x79,0x43,0x23,0x22,0xF5,0x9E,
	  0x00,0x10,0x00,0x22,0x32,0xFF,0x72,0x00,
	  0x50,0x00,0x23,0x22,0xFD,0x66,0x01,0x10,
	  0x00,0x13,0x52,0xF4,0x28,0x01,0x10,0x00 },	/* pe:0 sus:1 xof:0 eam:0 evb:1 BritePno */
	{ 0x00,0x00,0x79,0x45,0x23,0x42,0xD6,0x51,
	  0x00,0x40,0x06,0x13,0x61,0xD7,0x22,0x01,
	  0x10,0x00,0x33,0x43,0xDE,0x2D,0x00,0x20,
	  0x00,0x13,0x51,0xDF,0x22,0x01,0x20,0x00 },	/* pe:0 sus:1 xof:0 eam:0 evb:1 E.GrandP */
	{ 0x00,0x00,0x79,0x45,0x23,0x51,0xFE,0x68,
	  0x45,0x13,0x06,0x23,0xA3,0xD3,0x0A,0x43,
	  0x27,0x00,0x22,0x51,0xC3,0x5C,0x10,0x17,
	  0x05,0x33,0xA3,0xD3,0x0A,0x44,0x23,0x00 },	/* pe:0 sus:1 xof:0 eam:1 evb:1 HnkyTonk */
	{ 0x00,0x00,0x79,0x83,0x23,0xA3,0xB3,0x6D,
	  0x30,0x30,0x01,0x22,0x92,0xB4,0x6F,0x04,
	  0x30,0x00,0x13,0x44,0xA1,0x4D,0x04,0x20,
	  0x01,0x13,0x71,0xA8,0x14,0x31,0x10,0x00 },	/* pe:0 sus:1 xof:0 eam:1 evb:1 E.Piano1 */
	{ 0x00,0x00,0x79,0x45,0x53,0xC4,0xFB,0x8C,
	  0x44,0x70,0x95,0x12,0x82,0xFF,0x12,0x41,
	  0x10,0x00,0x13,0xB0,0xF1,0x49,0x44,0x10,
	  0x02,0x13,0x72,0xFF,0x10,0x41,0x10,0x00 },	/* pe:0 sus:1 xof:0 eam:0 evb:1 E.Piano2 */
	{ 0x00,0x00,0x79,0x46,0x23,0x52,0xE0,0x01,
	  0x00,0x10,0x24,0x03,0x52,0xF3,0x53,0x00,
	  0x60,0x18,0x03,0x13,0xF6,0x72,0x00,0x70,
	  0x20,0x23,0x72,0xEF,0x10,0x03,0x10,0x28 },	/* pe:0 sus:1 xof:0 eam:0 evb:1 Harpsi. */
	{ 0x00,0x00,0x79,0x43,0x12,0x61,0xFF,0x60,
	  0x44,0x10,0x2D,0x12,0x51,0xF0,0x74,0x44,
	  0x10,0x00,0x33,0x73,0xF2,0x6D,0x44,0x70,
	  0x20,0x23,0x92,0xF2,0x20,0x41,0x30,0x00 },	/* pe:0 sus:1 xof:0 eam:0 evb:1 Clavi. */
	{ 0x00,0x00,0x79,0x45,0x63,0x56,0xEF,0x56,
	  0x00,0x90,0x12,0x42,0x44,0xDE,0x18,0x01,
	  0x10,0x00,0x63,0x66,0xEC,0x5B,0x00,0xB0,
	  0x28,0x42,0x44,0xEE,0x18,0x01,0x10,0x00 },	/* pe:0 sus:1 xof:0 eam:0 evb:1 Celesta */
	{ 0x00,0x00,0x79,0x47,0x33,0x49,0xF4,0x24,
	  0x44,0x70,0x00,0x23,0x3B,0xFB,0x3C,0x44,
	  0x40,0x00,0x23,0x43,0xF4,0x49,0x44,0x20,
	  0x01,0x32,0x44,0xFE,0x10,0x44,0x10,0x00 },	/* pe:0 sus:1 xof:0 eam:0 evb:0 Glocken */
	{ 0x00,0x00,0x79,0x45,0x23,0x25,0x50,0x82,
	  0x00,0x20,0x08,0x33,0x24,0xF0,0x05,0x01,
	  0x13,0x00,0x23,0x25,0xA0,0x70,0x00,0x90,
	  0x08,0x13,0x12,0xF0,0x18,0x01,0x17,0x00 },	/* pe:0 sus:1 xof:0 eam:0 evb:1 MusicBox */
	{ 0x00,0x00,0x79,0x45,0x22,0x44,0xC2,0x5C,
	  0x41,0x70,0x00,0x23,0x59,0xD6,0x1E,0x51,
	  0x40,0x00,0x23,0x34,0xC2,0x78,0x51,0x80,
	  0x00,0x32,0x42,0xDF,0x1C,0x31,0x10,0x00 },	/* pe:0 sus:1 xof:0 eam:1 evb:1 Vibes */
	{ 0x00,0x00,0x79,0x45,0x43,0x47,0xAF,0xA1,
	  0x00,0xC0,0x07,0x42,0x54,0xBF,0x14,0x01,
	  0x10,0x00,0x63,0x47,0xBF,0x84,0x00,0x60,
	  0x00,0x53,0x54,0xDF,0x14,0x01,0x10,0x00 },	/* pe:0 sus:1 xof:0 eam:0 evb:1 Marimba */
	{ 0x00,0x00,0x79,0x45,0x62,0x69,0xFD,0x62,
	  0x00,0x50,0x02,0x52,0x77,0xFD,0x0C,0x03,
	  0x10,0x00,0x63,0x66,0xFA,0x76,0x00,0x50,
	  0x00,0x62,0x76,0xFE,0x02,0x03,0x10,0x00 },	/* pe:0 sus:1 xof:0 eam:0 evb:1 Xylophon */
	{ 0x00,0x00,0x79,0x45,0x32,0x34,0xF5,0x41,
	  0x30,0xA0,0x80,0x22,0x33,0xF2,0x14,0x03,
	  0x10,0x00,0x32,0x34,0xF5,0x41,0x30,0x73,
	  0x47,0x22,0x33,0xF2,0x16,0x03,0x27,0x00 },	/* pe:0 sus:1 xof:0 eam:1 evb:1 TubulBel */
	{ 0x00,0x00,0x79,0x46,0x42,0x4A,0xEC,0x1A,
	  0x00,0x20,0x0B,0x32,0x33,0xB5,0x51,0x00,
	  0x30,0x08,0x32,0x33,0xD0,0x28,0x00,0x10,
	  0x00,0x42,0x44,0xC6,0x18,0x03,0x10,0x00 },	/* pe:0 sus:1 xof:0 eam:0 evb:1 Dulcimar */
	{ 0x00,0x00,0x79,0xC2,0x02,0xC4,0xF0,0x02,
	  0x30,0x00,0x00,0x02,0xC5,0xF0,0x01,0x00,
	  0x10,0x20,0x02,0xC5,0xD1,0x1E,0x30,0x31,
	  0x48,0x02,0xC1,0xF0,0x1E,0x30,0x20,0x20 },	/* pe:0 sus:1 xof:0 eam:1 evb:0 DrawOrgn */
	{ 0x00,0x00,0x79,0x47,0x02,0xA5,0xE1,0x0E,
	  0x50,0x02,0x04,0x02,0x08,0xD5,0x74,0x00,
	  0x20,0x00,0x02,0xA5,0xE1,0x06,0x13,0x13,
	  0x00,0x02,0xA6,0xE0,0x06,0x03,0x27,0x00 },	/* pe:0 sus:1 xof:0 eam:1 evb:1 PercOrgn */
	{ 0x00,0x00,0x79,0x87,0x02,0xDF,0xF0,0x25,
	  0x70,0x13,0x04,0x02,0xAF,0xB1,0x15,0x50,
	  0x10,0xA8,0x02,0xEF,0xF0,0x24,0x13,0x26,
	  0x00,0x02,0xEF,0xF0,0x24,0x33,0x07,0x88 },	/* pe:0 sus:1 xof:0 eam:1 evb:1 RockOrgn */
	{ 0x00,0x00,0x79,0x47,0x02,0x5F,0x90,0x4E,
	  0x00,0x30,0x00,0x02,0x27,0xB2,0x74,0x00,
	  0x70,0x00,0x02,0x5F,0x80,0x12,0x00,0x10,
	  0x00,0x02,0x57,0x80,0x12,0x00,0x00,0x28 },	/* pe:0 sus:1 xof:0 eam:0 evb:0 ChrchOrg */
	{ 0x00,0x00,0x79,0x85,0x03,0x58,0x71,0x62,
	  0x00,0x20,0x83,0x03,0x6F,0x50,0x02,0x01,
	  0x10,0x00,0x03,0x5C,0x63,0x29,0x00,0x10,
	  0x28,0x03,0x7F,0x50,0x01,0x01,0x20,0x00 },	/* pe:0 sus:1 xof:0 eam:0 evb:1 ReedOrgn */
	{ 0x00,0x00,0x79,0x85,0x02,0x02,0x81,0x54,
	  0x00,0x36,0x8A,0x02,0xA2,0x72,0x09,0x01,
	  0x17,0x00,0x02,0x0F,0x61,0x48,0x00,0x12,
	  0x88,0x02,0xAF,0x70,0x1E,0x01,0x23,0x00 },	/* pe:0 sus:1 xof:0 eam:0 evb:1 Acordion */
	{ 0x00,0x00,0x79,0x84,0x02,0x9F,0xF0,0xB3,
	  0x07,0xE0,0x00,0x02,0x8F,0xF0,0xA4,0x11,
	  0xA0,0x00,0x02,0x8F,0xF0,0x90,0x03,0x10,
	  0x00,0x02,0x8F,0x60,0x0E,0x01,0x20,0x00 },	/* pe:0 sus:1 xof:0 eam:1 evb:1 Harmnica */
	{ 0x00,0x00,0x79,0x05,0x02,0x0C,0x70,0x3C,
	  0x30,0x21,0x64,0x02,0xA2,0x70,0x2A,0x11,
	  0x21,0x00,0x02,0x0F,0x70,0x50,0x10,0x10,
	  0x2B,0x02,0xAF,0x70,0x28,0x31,0x10,0x80 },	/* pe:0 sus:1 xof:0 eam:1 evb:1 TangoAcd */
	{ 0x00,0x00,0x79,0x85,0x13,0x41,0xE8,0x55,
	  0x00,0x10,0x06,0x32,0x73,0xFF,0x00,0x01,
	  0x10,0x00,0x53,0x55,0xB4,0x38,0x00,0x30,
	  0x08,0x42,0x94,0xDF,0x36,0x03,0x10,0x00 },	/* pe:0 sus:1 xof:0 eam:0 evb:1 NylonGtr */
	{ 0x00,0x00,0x79,0x44,0x12,0x47,0xF2,0x6A,
	  0x44,0x90,0x2C,0x13,0x83,0xF5,0xB6,0x45,
	  0xD0,0x00,0x13,0x42,0xF1,0x5E,0x45,0x10,
	  0x06,0x22,0x83,0xDF,0x10,0x25,0x10,0x00 },	/* pe:0 sus:1 xof:0 eam:0 evb:1 SteelGtr */
	{ 0x00,0x00,0x79,0x43,0x13,0x77,0xF3,0x45,
	  0x45,0x10,0x00,0x12,0x45,0xF2,0x4B,0x45,
	  0x50,0x00,0x02,0x72,0xFF,0x7D,0x45,0x30,
	  0x00,0x02,0x82,0xCF,0x10,0x45,0x10,0x00 },	/* pe:0 sus:1 xof:0 eam:0 evb:1 JazzGtr */
	{ 0x00,0x00,0x79,0xC5,0x22,0x2A,0xF1,0x3D,
	  0x00,0x10,0x08,0x22,0x92,0xFF,0x0E,0x03,
	  0x10,0x00,0x23,0x32,0xF6,0x41,0x00,0x30,
	  0x20,0x42,0x84,0xE6,0x0E,0x03,0x10,0x00 },	/* pe:0 sus:1 xof:0 eam:0 evb:1 CleanGtr */
	{ 0x00,0x00,0x79,0x85,0x63,0x78,0xD7,0x44,
	  0x00,0x00,0x07,0x32,0x99,0xE7,0x00,0x03,
	  0x00,0x00,0x32,0x83,0xE9,0x13,0x00,0x10,
	  0x00,0x33,0xA4,0xB3,0x00,0x03,0x10,0x00 },	/* pe:0 sus:1 xof:0 eam:0 evb:1 Mute.Gtr */
	{ 0x00,0x00,0x79,0x44,0x03,0x28,0xFF,0x4C,
	  0x44,0x07,0x62,0x02,0x11,0xC1,0x3D,0x44,
	  0x23,0x00,0x02,0xA2,0xB1,0x3C,0x44,0x10,
	  0x00,0x12,0xA1,0xB1,0x28,0x44,0x20,0x00 },	/* pe:0 sus:1 xof:0 eam:0 evb:0 Ovrdrive */
	{ 0x00,0x00,0x79,0x84,0x02,0x2C,0xB0,0x21,
	  0x00,0x20,0x1C,0x02,0xA5,0xC1,0x74,0x00,
	  0x10,0x00,0x02,0xA5,0xC1,0x5C,0x00,0x20,
	  0x40,0x02,0xA1,0xC5,0x3C,0x03,0x10,0x30 },	/* pe:0 sus:1 xof:0 eam:0 evb:1 Dist.Gtr */
	{ 0x00,0x00,0x79,0x85,0x82,0x72,0xF0,0x59,
	  0x00,0x00,0x45,0x32,0x93,0xDF,0x18,0x01,
	  0x20,0x40,0x82,0x72,0xB0,0x44,0x00,0x00,
	  0x05,0x72,0x77,0xAF,0x34,0x05,0x20,0x30 },	/* pe:0 sus:1 xof:0 eam:0 evb:1 GtrHarmo */
	{ 0x00,0x00,0x79,0x45,0x33,0x83,0xBA,0x39,
	  0x00,0x10,0x03,0x32,0x83,0xCB,0x00,0x05,
	  0x10,0x00,0x32,0x13,0x91,0x1F,0x00,0x10,
	  0x00,0x32,0x83,0xCA,0x14,0x05,0x10,0x00 },	/* pe:0 sus:1 xof:0 eam:0 evb:1 Aco.Bass */
	{ 0x00,0x00,0x79,0x43,0x13,0x32,0xA1,0x72,
	  0x44,0x10,0x06,0x33,0x64,0x94,0xE8,0x44,
	  0xC0,0x00,0x23,0x33,0xB2,0x5A,0x44,0x10,
	  0x00,0x13,0x81,0xB2,0x00,0x44,0x20,0x00 },	/* pe:0 sus:1 xof:0 eam:0 evb:0 FngrBass */
	{ 0x00,0x00,0x79,0x43,0x23,0x37,0xF1,0x4E,
	  0x44,0x10,0x05,0x43,0x6B,0xC7,0x54,0x44,
	  0x70,0x00,0x23,0x69,0xF2,0x5E,0x44,0x20,
	  0x00,0x63,0x82,0xB6,0x00,0x44,0x10,0x00 },	/* pe:0 sus:1 xof:0 eam:0 evb:0 PickBass */
	{ 0x00,0x00,0x79,0x43,0x23,0x33,0xC1,0x76,
	  0x45,0x12,0x04,0x33,0x63,0xA3,0x66,0x45,
	  0x11,0x00,0x23,0x63,0x91,0x66,0x45,0x10,
	  0x00,0x23,0x81,0xB2,0x00,0x45,0x20,0x00 },	/* pe:0 sus:1 xof:0 eam:0 evb:1 Fretless */
	{ 0x00,0x00,0x79,0x43,0x23,0x37,0xF2,0x3A,
	  0x44,0x10,0x03,0x63,0x66,0xF4,0x54,0x44,
	  0x90,0x00,0x23,0x69,0xC2,0x62,0x44,0x10,
	  0x00,0xF3,0x82,0xFF,0x0C,0x44,0x10,0x00 },	/* pe:0 sus:1 xof:0 eam:0 evb:0 SlapBas1 */
	{ 0x00,0x00,0x79,0x43,0x23,0x37,0xF1,0x3A,
	  0x44,0x10,0x02,0x62,0x75,0xB2,0x48,0x44,
	  0xD0,0x00,0x23,0x69,0x92,0x7A,0x44,0x10,
	  0x00,0x63,0x82,0xF6,0x18,0x44,0x10,0x00 },	/* pe:0 sus:1 xof:0 eam:0 evb:0 SlapBas2 */
	{ 0x00,0x00,0x79,0x83,0x22,0x86,0xE5,0x38,
	  0x00,0x10,0x05,0x12,0x84,0xE6,0x9D,0x00,
	  0x20,0x40,0x12,0x82,0xE6,0x8C,0x01,0x10,
	  0x00,0x22,0x82,0xE9,0x00,0x07,0x10,0x00 },	/* pe:0 sus:1 xof:0 eam:0 evb:1 SynBass1 */
	{ 0x00,0x00,0x79,0x45,0x72,0x85,0xF6,0x50,
	  0x44,0x20,0x06,0x72,0x81,0xFC,0x01,0x44,
	  0x20,0x00,0x72,0x73,0xF6,0x50,0x44,0x10,
	  0x06,0x72,0x82,0xFC,0x01,0x44,0x10,0x00 },	/* pe:0 sus:1 xof:0 eam:0 evb:0 SynBass2 */
	{ 0x00,0x00,0x79,0x85,0x03,0x30,0x60,0x4A,
	  0x10,0x10,0x62,0x02,0x74,0x62,0x0C,0x01,
	  0x15,0x00,0x73,0xA5,0xE0,0x19,0x00,0x40,
	  0x32,0x72,0x77,0x6F,0x0E,0x03,0x11,0x00 },	/* pe:0 sus:1 xof:0 eam:1 evb:1 Violin */
	{ 0x00,0x00,0x79,0x85,0x03,0x30,0x60,0x26,
	  0x00,0x10,0x0A,0x02,0x76,0x61,0x0C,0x13,
	  0x10,0x00,0x73,0x76,0xE0,0x21,0x10,0x10,
	  0x32,0x72,0x77,0x6F,0x0E,0x03,0x10,0x00 },	/* pe:0 sus:1 xof:0 eam:1 evb:1 Viola */
	{ 0x00,0x00,0x79,0x83,0x02,0x66,0xF0,0x40,
	  0x03,0x10,0x0C,0xF2,0xE5,0xFF,0x50,0x43,
	  0x50,0x00,0x02,0x75,0xF2,0xB4,0x13,0x10,
	  0x00,0x02,0x73,0x61,0x06,0x13,0x30,0x00 },	/* pe:0 sus:1 xof:0 eam:1 evb:1 Cello */
	{ 0x00,0x00,0x79,0x83,0x00,0x26,0xF0,0x64,
	  0x01,0x10,0x06,0xF0,0xE6,0xFF,0x54,0x43,
	  0x50,0x88,0x00,0x46,0xF3,0x6C,0x13,0x30,
	  0x00,0x00,0x73,0x61,0x02,0x01,0x20,0x00 },	/* pe:0 sus:0 xof:0 eam:1 evb:1 ContraBs */
	{ 0x00,0x00,0x79,0xC5,0x03,0x32,0x71,0x5A,
	  0x10,0x12,0xA3,0x02,0x63,0x61,0x08,0x33,
	  0x20,0x00,0x03,0x43,0x70,0x59,0x00,0x14,
	  0x63,0x02,0x63,0x61,0x08,0x53,0x10,0x00 },	/* pe:0 sus:1 xof:0 eam:1 evb:1 Trem.Str */
	{ 0x00,0x00,0x79,0x85,0x52,0xBB,0xF9,0x50,
	  0x00,0x10,0x07,0x82,0x77,0xE2,0x00,0x07,
	  0x10,0x00,0x62,0x57,0xFF,0x45,0x00,0x10,
	  0x40,0x52,0x66,0xCF,0x00,0x07,0x10,0x00 },	/* pe:0 sus:1 xof:0 eam:0 evb:1 Pizz.Str */
	{ 0x00,0x00,0x79,0x43,0x52,0x88,0xF4,0xA4,
	  0x00,0x20,0x06,0x72,0x98,0xB4,0x84,0x00,
	  0x50,0x00,0x32,0x27,0xB4,0x86,0x00,0x10,
	  0x05,0x23,0x24,0xF1,0x10,0x44,0x10,0x00 },	/* pe:0 sus:1 xof:0 eam:0 evb:0 Harp */
	{ 0x00,0x00,0x79,0x43,0x43,0x38,0xF3,0x11,
	  0x44,0x10,0x03,0x23,0x22,0xFF,0x84,0x44,
	  0x07,0x00,0x33,0x37,0xF0,0x72,0x44,0x10,
	  0x00,0x33,0x34,0xFF,0x00,0x44,0x00,0x40 },	/* pe:0 sus:1 xof:0 eam:0 evb:0 Timpani */
	{ 0x00,0x00,0x79,0x87,0x00,0x6A,0x50,0x2E,
	  0x30,0x11,0x3A,0x00,0x66,0xC1,0x60,0x00,
	  0x20,0x20,0x00,0x66,0x61,0x1E,0x03,0x15,
	  0x00,0x00,0x65,0x61,0x14,0x33,0x23,0x60 },	/* pe:0 sus:0 xof:0 eam:1 evb:1 Strings1 */
	{ 0x00,0x00,0x79,0x87,0x02,0x5A,0x60,0x00,
	  0x10,0x13,0x4B,0x02,0x56,0xC1,0x5C,0x30,
	  0x10,0x10,0x02,0x66,0x50,0x02,0x05,0x15,
	  0x08,0x02,0x65,0x51,0x1C,0x03,0x17,0xD8 },	/* pe:0 sus:1 xof:0 eam:1 evb:1 Strings2 */
	{ 0x00,0x00,0x79,0x85,0x02,0x28,0x91,0x6C,
	  0x10,0x16,0x00,0x02,0x5F,0x70,0x1C,0x03,
	  0x13,0x00,0x03,0x2B,0x90,0x50,0x10,0x13,
	  0x20,0x02,0x4F,0x60,0x01,0x13,0x17,0x00 },	/* pe:0 sus:1 xof:0 eam:1 evb:1 Syn.Str1 */
	{ 0x00,0x00,0x79,0x85,0x02,0x28,0x91,0x4C,
	  0x10,0x10,0x4D,0x02,0x56,0x60,0x18,0x03,
	  0x10,0x00,0x02,0x28,0x80,0x28,0x10,0x10,
	  0xD8,0x02,0x47,0x53,0x01,0x13,0x10,0x00 },	/* pe:0 sus:1 xof:0 eam:1 evb:1 Syn.Str2 */
	{ 0x00,0x00,0x79,0x45,0x02,0x00,0xCF,0x4C,
	  0x00,0x60,0x3D,0x02,0x53,0x66,0x5E,0x03,
	  0x40,0x00,0x02,0x3F,0x70,0x79,0x00,0x10,
	  0x40,0x02,0x5F,0x50,0x00,0x03,0x20,0x00 },	/* pe:0 sus:1 xof:0 eam:0 evb:1 ChoirAah */
	{ 0x00,0x00,0x79,0x85,0x03,0x00,0xCF,0x50,
	  0x30,0x50,0x3F,0x02,0x53,0x76,0x52,0x03,
	  0x40,0x00,0x02,0x47,0x73,0x6A,0x10,0x10,
	  0x50,0x02,0x51,0x90,0x00,0x05,0x10,0x00 },	/* pe:0 sus:1 xof:0 eam:1 evb:1 VoiceOoh */
	{ 0x00,0x00,0x79,0x85,0x02,0x40,0xAF,0x58,
	  0x00,0x10,0x00,0x02,0x5F,0x71,0x22,0x03,
	  0x10,0x00,0x02,0x4F,0x90,0x69,0x10,0x10,
	  0x48,0x02,0x5F,0x70,0x02,0x13,0x30,0x00 },	/* pe:0 sus:1 xof:0 eam:1 evb:1 SynVoice */
	{ 0x00,0x00,0x79,0x87,0x43,0x64,0xF6,0x00,
	  0x30,0x40,0x45,0x32,0x35,0xC1,0x00,0x00,
	  0x10,0x30,0x62,0x67,0xC0,0x00,0x03,0x10,
	  0x30,0x72,0x67,0xB0,0x00,0x03,0x00,0x30 },	/* pe:0 sus:1 xof:0 eam:1 evb:1 Orch.Hit */
	{ 0x00,0x00,0x79,0x43,0x02,0x58,0x81,0x50,
	  0x01,0x10,0x0E,0x02,0x58,0xA4,0x5E,0x01,
	  0x30,0x00,0x02,0x67,0x71,0x6A,0x00,0x10,
	  0x00,0x02,0x8F,0x90,0x18,0x11,0x10,0x00 },	/* pe:0 sus:1 xof:0 eam:1 evb:1 Trumpet */
	{ 0x00,0x00,0x79,0x43,0x02,0x76,0x71,0x70,
	  0x01,0x10,0x47,0x02,0x58,0x94,0x3E,0x01,
	  0x10,0x00,0x02,0x77,0x61,0x6A,0x01,0x10,
	  0x08,0x02,0x8F,0x80,0x14,0x03,0x10,0x00 },	/* pe:0 sus:1 xof:0 eam:0 evb:1 Trombone */
	{ 0x00,0x00,0x79,0x43,0x02,0x65,0x62,0x88,
	  0x01,0x10,0x47,0x02,0xB8,0xC4,0x62,0x01,
	  0x10,0x08,0x02,0x97,0x93,0x46,0x01,0x10,
	  0x08,0x02,0x8F,0x70,0x00,0x03,0x20,0x00 },	/* pe:0 sus:1 xof:0 eam:0 evb:1 Tuba */
	{ 0x00,0x00,0x79,0x45,0x02,0x70,0x75,0x68,
	  0x10,0x30,0x50,0x02,0x9D,0x94,0x00,0x01,
	  0x00,0x10,0x02,0x69,0x71,0x4C,0x11,0x50,
	  0x88,0x02,0x97,0x82,0x00,0x01,0x10,0x10 },	/* pe:0 sus:1 xof:0 eam:1 evb:1 Mute.Trp */
	{ 0x00,0x00,0x79,0x45,0x02,0x09,0x70,0x43,
	  0x00,0x12,0x40,0x02,0x7E,0x90,0x06,0x01,
	  0x17,0x00,0x02,0x29,0x61,0x5A,0x30,0x16,
	  0x04,0x02,0x7E,0xA0,0x06,0x01,0x13,0x40 },	/* pe:0 sus:1 xof:0 eam:1 evb:1 Fr.Horn */
	{ 0x00,0x00,0x79,0x45,0x02,0x26,0x81,0x58,
	  0x44,0x17,0x06,0x02,0x8F,0x90,0x20,0x40,
	  0x17,0x00,0x02,0x57,0x71,0x58,0x41,0x10,
	  0x66,0x02,0x88,0x90,0x1C,0x41,0x10,0x00 },	/* pe:0 sus:1 xof:0 eam:0 evb:1 BrasSect */
	{ 0x00,0x00,0x79,0x45,0x02,0x86,0x72,0x40,
	  0x44,0x17,0x06,0x02,0xAF,0x90,0x28,0x44,
	  0x17,0x00,0x02,0x86,0x72,0x40,0x41,0x10,
	  0x06,0x02,0xA8,0x90,0x28,0x41,0x10,0x00 },	/* pe:0 sus:1 xof:0 eam:0 evb:1 SynBras1 */
	{ 0x00,0x00,0x79,0x43,0x02,0x43,0x61,0x70,
	  0x01,0x10,0x06,0x02,0x57,0x97,0x9C,0x01,
	  0x60,0x08,0x32,0x35,0x7B,0x8C,0x01,0x10,
	  0x40,0x02,0x7F,0xF0,0x10,0x03,0x10,0x00 },	/* pe:0 sus:1 xof:0 eam:0 evb:1 SynBras2 */
	{ 0x00,0x00,0x79,0x43,0x63,0x69,0xF3,0x74,
	  0x10,0x30,0x00,0x02,0x62,0x80,0x68,0x00,
	  0x10,0x00,0x02,0x05,0x80,0x31,0x10,0x10,
	  0x08,0x02,0x86,0x81,0x0C,0x03,0x10,0x00 },	/* pe:0 sus:1 xof:0 eam:1 evb:1 SprnoSax */
	{ 0x00,0x00,0x79,0x45,0x07,0x03,0x90,0x2A,
	  0x44,0x10,0x0C,0x06,0x92,0x80,0x24,0x41,
	  0x10,0x00,0x07,0x03,0x90,0x36,0x44,0x10,
	  0x4C,0x06,0x92,0x90,0x54,0x41,0x10,0x08 },	/* pe:0 sus:1 xof:0 eam:0 evb:1 Alto Sax */
	{ 0x00,0x00,0x79,0x45,0x07,0x03,0x70,0x16,
	  0x44,0x10,0x0B,0x06,0x92,0x70,0x3C,0x43,
	  0x10,0x40,0x07,0x03,0x70,0x22,0x44,0x10,
	  0x4B,0x06,0x92,0x70,0x36,0x43,0x10,0x00 },	/* pe:0 sus:1 xof:0 eam:0 evb:1 TenorSax */
	{ 0x00,0x00,0x79,0x45,0x03,0x53,0x70,0x4A,
	  0x00,0x10,0x06,0x03,0x82,0x72,0x18,0x03,
	  0x20,0x00,0x03,0x15,0x70,0x39,0x10,0x20,
	  0x10,0x03,0x84,0x71,0x14,0x05,0x10,0x00 },	/* pe:0 sus:1 xof:0 eam:1 evb:1 Bari.Sax */
	{ 0x00,0x00,0x79,0x85,0x02,0x40,0xA0,0x7A,
	  0x03,0x10,0x28,0x12,0x91,0x90,0x24,0x03,
	  0x30,0x00,0x02,0x40,0xB2,0x60,0x23,0x10,
	  0x00,0x02,0xA0,0xA0,0x38,0x03,0x20,0x00 },	/* pe:0 sus:1 xof:0 eam:0 evb:1 Oboe */
	{ 0x00,0x00,0x79,0x45,0x02,0x40,0xA0,0x8A,
	  0x03,0x10,0x28,0x12,0x91,0x91,0x2C,0x03,
	  0x30,0x00,0x02,0x40,0xB2,0x60,0x23,0x10,
	  0x00,0x12,0xA0,0xA1,0x2C,0x03,0x20,0x00 },	/* pe:0 sus:1 xof:0 eam:0 evb:1 Eng.Horn */
	{ 0x00,0x00,0x79,0x85,0x07,0x07,0xC1,0x62,
	  0x00,0x10,0x08,0x07,0x81,0x71,0x02,0x11,
	  0x30,0x48,0x07,0x07,0xC1,0x62,0x00,0x10,
	  0x08,0x07,0x81,0x71,0x0C,0x11,0x31,0x48 },	/* pe:0 sus:1 xof:0 eam:1 evb:1 Bassoon */
	{ 0x00,0x00,0x79,0x45,0x03,0x12,0x71,0x96,
	  0x11,0x20,0x07,0x02,0x82,0x81,0x0C,0x11,
	  0x10,0x00,0x03,0x12,0x51,0x69,0x11,0x40,
	  0x00,0x02,0x82,0x71,0x0C,0x00,0x10,0x00 },	/* pe:0 sus:1 xof:0 eam:1 evb:1 Clarinet */
	{ 0x00,0x00,0x79,0x85,0x02,0x7C,0xA1,0x30,
	  0x00,0x50,0x17,0x02,0x87,0x9F,0x9C,0x13,
	  0x10,0x00,0x02,0x75,0x81,0x78,0x10,0x10,
	  0x08,0x02,0xA5,0x80,0x18,0x01,0x10,0x00 },	/* pe:0 sus:1 xof:0 eam:1 evb:1 Piccolo */
	{ 0x00,0x00,0x79,0x45,0x02,0x1A,0xD1,0x1C,
	  0x10,0x30,0x07,0x02,0xB8,0x73,0x94,0x01,
	  0x30,0x00,0x02,0x98,0xE0,0x9C,0x31,0x10,
	  0x07,0x02,0xA5,0x60,0x04,0x11,0x10,0x80 },	/* pe:0 sus:1 xof:0 eam:1 evb:1 Flute */
	{ 0x00,0x00,0x79,0x85,0x72,0xA6,0x90,0xE8,
	  0x35,0x20,0x1F,0x02,0xA5,0x80,0x10,0x15,
	  0x10,0x40,0x62,0x69,0xA9,0x3C,0x01,0x70,
	  0xC0,0x02,0xA5,0x80,0x90,0x35,0x10,0x00 },	/* pe:0 sus:1 xof:0 eam:1 evb:1 Recorder */
	{ 0x00,0x00,0x79,0x85,0x02,0x60,0xA0,0x01,
	  0x20,0xD0,0x07,0x12,0xAA,0xB0,0x8C,0x21,
	  0xA0,0x18,0x02,0x4F,0x80,0xB0,0x15,0x20,
	  0x00,0x02,0x90,0x80,0x14,0x25,0x10,0x00 },	/* pe:0 sus:1 xof:0 eam:1 evb:1 PanFlute */
	{ 0x00,0x00,0x79,0x45,0x03,0x7C,0xC1,0x30,
	  0x00,0x50,0x57,0x02,0x97,0x76,0x6C,0x01,
	  0x10,0x00,0x02,0x88,0x73,0x2F,0x30,0x20,
	  0x40,0x02,0x85,0x70,0x04,0x05,0x10,0x00 },	/* pe:0 sus:1 xof:0 eam:1 evb:1 Bottle */
	{ 0x00,0x00,0x79,0x45,0x02,0x5C,0xA1,0x18,
	  0x00,0x50,0x17,0x02,0x97,0x65,0x5C,0x31,
	  0x10,0x40,0x02,0x38,0xA3,0x09,0x50,0x00,
	  0x90,0x02,0x95,0x60,0x04,0x05,0x10,0x00 },	/* pe:0 sus:1 xof:0 eam:1 evb:1 Shakhchi */
	{ 0x00,0x00,0x79,0x42,0x02,0x6A,0x60,0x14,
	  0x15,0x10,0x00,0x02,0x78,0x80,0x14,0x55,
	  0x10,0x00,0x92,0x7A,0x60,0xB0,0x55,0x10,
	  0x88,0x02,0x78,0x60,0x14,0x55,0x17,0x40 },	/* pe:0 sus:1 xof:0 eam:1 evb:1 Whistle */
	{ 0x00,0x00,0x79,0x85,0x72,0x86,0x80,0xF1,
	  0x15,0x20,0x1F,0x02,0x95,0x80,0x00,0x05,
	  0x10,0x00,0x62,0x69,0xA9,0x3C,0x01,0x10,
	  0xC0,0x02,0xA5,0x70,0x68,0x15,0x10,0x40 },	/* pe:0 sus:1 xof:0 eam:1 evb:1 Ocarina */
	{ 0x00,0x00,0x79,0x85,0x02,0x7F,0xF4,0xBB,
	  0x00,0x10,0x40,0x02,0xAF,0xA0,0x0E,0x03,
	  0x10,0x40,0x02,0x2F,0xF3,0x9B,0x10,0x20,
	  0x41,0x02,0xAF,0xA0,0x0E,0x01,0x10,0x40 },	/* pe:0 sus:1 xof:0 eam:1 evb:1 SquareLd */
	{ 0x00,0x00,0x79,0x45,0x02,0x70,0xF0,0x68,
	  0x41,0x17,0x07,0x02,0x7F,0xD0,0x28,0x41,
	  0x17,0x40,0x02,0x3F,0xF0,0x51,0x41,0x10,
	  0xA7,0x02,0x8F,0xE0,0x28,0x41,0x10,0x40 },	/* pe:0 sus:1 xof:0 eam:0 evb:1 Saw.Lead */
	{ 0x00,0x00,0x79,0x85,0x02,0x74,0xC8,0x00,
	  0x30,0x40,0x47,0x02,0x66,0xC7,0x51,0x54,
	  0x40,0x40,0x02,0x56,0x85,0x0D,0x01,0x20,
	  0x00,0x02,0x84,0x61,0x08,0x30,0x10,0x80 },	/* pe:0 sus:1 xof:0 eam:1 evb:1 CaliopLd */
	{ 0x00,0x00,0x79,0x45,0x02,0x27,0x76,0x11,
	  0x15,0x14,0x40,0x02,0x86,0xF0,0x2C,0x15,
	  0x10,0x40,0x02,0x27,0x76,0x0D,0x15,0x11,
	  0x40,0x02,0x86,0xF0,0x2C,0x15,0x12,0x40 },	/* pe:0 sus:1 xof:0 eam:1 evb:1 Chiff Ld */
	{ 0x00,0x00,0x79,0x45,0x02,0x62,0x92,0x22,
	  0x45,0x14,0x08,0x02,0x81,0x92,0x38,0x15,
	  0x20,0x60,0x02,0x62,0x92,0x2A,0x45,0x11,
	  0x08,0x02,0x81,0x92,0x38,0x15,0x21,0x60 },	/* pe:0 sus:1 xof:0 eam:1 evb:1 CharanLd */
	{ 0x00,0x00,0x79,0x45,0x02,0x00,0x4F,0x34,
	  0x00,0x70,0x30,0x02,0x83,0x76,0x4C,0x01,
	  0x20,0x00,0x02,0x9F,0x70,0x71,0x30,0x10,
	  0x40,0x02,0x8F,0x70,0x08,0x03,0x10,0x00 },	/* pe:0 sus:1 xof:0 eam:1 evb:1 VoiceLd */
	{ 0x00,0x00,0x79,0x47,0x12,0x81,0xC1,0x54,
	  0x45,0x10,0xC0,0x12,0x61,0xC2,0x3C,0x45,
	  0x11,0x00,0x02,0x80,0xC0,0x68,0x45,0x24,
	  0x00,0x12,0x81,0xC1,0x18,0x45,0x31,0x80 },	/* pe:0 sus:1 xof:0 eam:0 evb:1 Fifth Ld */
	{ 0x00,0x00,0x79,0x45,0x02,0x32,0xB0,0x58,
	  0x00,0x10,0x08,0x02,0x92,0xA0,0x44,0x03,
	  0x10,0x80,0x42,0x43,0xC5,0x2E,0x00,0x10,
	  0x05,0x02,0x93,0xD6,0x30,0x00,0x10,0x00 },	/* pe:0 sus:1 xof:0 eam:0 evb:1 Bass &Ld */
	{ 0x00,0x00,0x79,0x45,0x31,0x3F,0xF0,0x98,
	  0x44,0x70,0x0D,0x40,0x47,0xF0,0x2E,0x44,
	  0x50,0x00,0x00,0x11,0x60,0x62,0x03,0x17,
	  0x0E,0x00,0x51,0x81,0x02,0x03,0x10,0x00 },	/* pe:0 sus:0 xof:0 eam:0 evb:1 NewAgePd */
	{ 0x00,0x00,0x79,0x45,0x02,0x50,0xA0,0xA0,
	  0x01,0x11,0x07,0x02,0x40,0x30,0x08,0x05,
	  0x10,0x00,0x02,0x30,0xA0,0xBC,0x01,0x13,
	  0x07,0x02,0x40,0x30,0x08,0x05,0x12,0x00 },	/* pe:0 sus:1 xof:0 eam:0 evb:1 Warm Pad */
	{ 0x00,0x00,0x79,0x45,0x00,0x45,0x61,0x88,
	  0x05,0x17,0x18,0x00,0x65,0xA1,0x0A,0x05,
	  0x26,0x00,0x00,0x33,0x61,0x88,0x05,0x10,
	  0xDC,0x00,0x53,0x90,0x02,0x05,0x12,0x40 },	/* pe:0 sus:0 xof:0 eam:0 evb:1 PolySyPd */
	{ 0x00,0x00,0x79,0x45,0x00,0x00,0xAF,0x84,
	  0x45,0x17,0x42,0x00,0x73,0x43,0x5A,0x05,
	  0x80,0x00,0x00,0x33,0x70,0x85,0x05,0x10,
	  0x06,0x00,0x5F,0x60,0x00,0x05,0x23,0x00 },	/* pe:0 sus:0 xof:0 eam:0 evb:1 ChoirPad */
	{ 0x00,0x00,0x79,0x45,0x02,0x31,0x24,0xAA,
	  0x15,0x70,0x04,0x02,0x52,0x63,0x00,0x11,
	  0x10,0x00,0x02,0x31,0x24,0xAA,0x15,0x71,
	  0x04,0x02,0x52,0x63,0x00,0x11,0x12,0x00 },	/* pe:0 sus:1 xof:0 eam:1 evb:1 BowedPad */
	{ 0x00,0x00,0x79,0x45,0x03,0x32,0xF0,0x5E,
	  0x00,0x10,0x56,0x02,0x46,0x50,0x0E,0x01,
	  0x15,0x00,0x03,0x36,0xF3,0x1D,0x00,0x10,
	  0x02,0x02,0x47,0x50,0x11,0x03,0x11,0x00 },	/* pe:0 sus:1 xof:0 eam:0 evb:1 MetalPad */
	{ 0x00,0x00,0x79,0x45,0x00,0x31,0x41,0x94,
	  0x44,0x15,0x06,0x00,0x52,0x60,0x02,0x01,
	  0x17,0x00,0x00,0x45,0xC1,0x78,0x44,0x13,
	  0x05,0x00,0x52,0x80,0x02,0x03,0x10,0x00 },	/* pe:0 sus:0 xof:0 eam:0 evb:1 Halo Pad */
	{ 0x00,0x00,0x79,0x45,0x02,0x38,0x40,0x78,
	  0x10,0x15,0x00,0x02,0x48,0x30,0x02,0x03,
	  0x13,0x00,0x02,0x41,0x30,0x8A,0x00,0x22,
	  0x00,0x02,0x52,0x70,0x02,0x03,0x17,0x00 },	/* pe:0 sus:1 xof:0 eam:1 evb:1 SweepPad */
	{ 0x00,0x00,0x79,0x45,0x03,0x68,0xF8,0x0B,
	  0x55,0xA0,0x01,0x13,0x25,0x80,0x00,0x75,
	  0x10,0x80,0x03,0x68,0xF8,0x0B,0x45,0xA2,
	  0x01,0x13,0x25,0x80,0x00,0x55,0x13,0x80 },	/* pe:0 sus:1 xof:0 eam:1 evb:1 Rain */
	{ 0x00,0x00,0x79,0x85,0x02,0x33,0x63,0x4A,
	  0x10,0x30,0x03,0x02,0x42,0x40,0x25,0x01,
	  0x30,0x00,0x02,0x33,0x63,0x40,0x00,0x10,
	  0x85,0x02,0x41,0x41,0x0E,0x03,0x10,0x00 },	/* pe:0 sus:1 xof:0 eam:1 evb:1 SoundTrk */
	{ 0x00,0x00,0x79,0x45,0x12,0x58,0xF4,0x51,
	  0x45,0x60,0x05,0x22,0x42,0xC7,0x24,0x44,
	  0x10,0x00,0x12,0x58,0xF4,0x51,0x45,0xE2,
	  0x05,0x22,0x42,0xC7,0x24,0x44,0x12,0x00 },	/* pe:0 sus:1 xof:0 eam:0 evb:1 Crystal */
	{ 0x00,0x00,0x79,0x85,0x22,0x43,0x6F,0x55,
	  0x00,0x10,0xA4,0x02,0x43,0xF3,0x00,0x03,
	  0x10,0x00,0x02,0x43,0xCF,0x41,0x00,0x23,
	  0xA4,0x52,0x46,0x93,0x30,0x03,0x27,0x00 },	/* pe:0 sus:1 xof:0 eam:0 evb:1 Atmosphr */
	{ 0x00,0x00,0x79,0x42,0x12,0x41,0xF5,0x1E,
	  0x15,0x13,0x58,0x52,0x42,0xFF,0x1C,0x15,
	  0x13,0x48,0x12,0x41,0xF5,0x1E,0x15,0x11,
	  0x58,0x52,0x42,0xFF,0x1C,0x15,0x11,0x48 },	/* pe:0 sus:1 xof:0 eam:1 evb:1 Bright */
	{ 0x00,0x00,0x79,0x45,0x00,0x11,0x12,0x4A,
	  0x47,0x37,0x6C,0x00,0x41,0x21,0x26,0x10,
	  0x30,0x00,0x00,0x21,0x20,0x50,0x45,0x10,
	  0x04,0x00,0x31,0x30,0x0E,0x45,0x14,0x00 },	/* pe:0 sus:0 xof:0 eam:1 evb:1 Goblins */
	{ 0x00,0x00,0x79,0x85,0x02,0x03,0x45,0x88,
	  0x00,0x20,0x00,0x02,0xC2,0xA0,0x3A,0x03,
	  0x10,0x00,0x02,0x23,0x32,0x8C,0x30,0x10,
	  0x80,0x02,0x3F,0xA0,0x00,0x03,0x10,0x00 },	/* pe:0 sus:1 xof:0 eam:1 evb:1 Echoes */
	{ 0x00,0x00,0x79,0x45,0x02,0x33,0x58,0x65,
	  0x11,0x20,0x0A,0x12,0x41,0x62,0x18,0x11,
	  0x17,0x20,0x02,0x33,0x58,0x65,0x11,0x23,
	  0x0A,0x12,0x41,0x62,0x18,0x11,0x13,0x20 },	/* pe:0 sus:1 xof:0 eam:1 evb:1 Sci-Fi */
	{ 0x00,0x00,0x81,0x45,0x32,0x22,0xD5,0x2A,
	  0x45,0x20,0x03,0x62,0x42,0xFF,0x20,0x45,
	  0x70,0x48,0x32,0x22,0xD5,0x0E,0x47,0x11,
	  0x44,0x62,0x42,0xFF,0x20,0x45,0x41,0x88 },	/* pe:0 sus:1 xof:0 eam:0 evb:1 Sitar */
	{ 0x00,0x00,0x79,0x85,0x23,0x13,0xD1,0x28,
	  0x00,0x10,0x20,0x33,0x53,0xDE,0x02,0x03,
	  0x30,0x00,0x42,0x27,0xF1,0x3C,0x01,0x60,
	  0x08,0x82,0x87,0xFE,0x02,0x03,0x10,0x08 },	/* pe:0 sus:1 xof:0 eam:0 evb:1 Banjo */
	{ 0x00,0x00,0x79,0x43,0x10,0x31,0xF2,0x68,
	  0x44,0x10,0x44,0x50,0x6A,0xF7,0x50,0x45,
	  0x30,0x00,0x30,0x38,0xF3,0x60,0x44,0x50,
	  0x80,0x41,0x44,0xFF,0x04,0x45,0x30,0x40 },	/* pe:0 sus:0 xof:0 eam:0 evb:1 Shamisen */
	{ 0x00,0x00,0x79,0x43,0x50,0x57,0xF2,0x52,
	  0x00,0x30,0x46,0x80,0x89,0xF4,0x56,0x00,
	  0x50,0x80,0x20,0x32,0xFF,0xA8,0x00,0x10,
	  0x05,0x21,0x22,0xFF,0x0C,0x44,0x10,0x00 },	/* pe:0 sus:0 xof:0 eam:0 evb:0 Koto */
	{ 0x00,0x00,0x79,0x45,0x52,0x6A,0xFA,0x21,
	  0x45,0x43,0x26,0x32,0x52,0xC0,0x20,0x45,
	  0x10,0x00,0x52,0x68,0xFA,0x39,0x45,0x52,
	  0x06,0x43,0x42,0xC0,0x20,0x45,0x14,0x00 },	/* pe:0 sus:1 xof:0 eam:0 evb:1 Kalimba */
	{ 0x00,0x00,0x79,0x45,0x00,0xC9,0xB1,0x42,
	  0x44,0x10,0x49,0x00,0xDF,0x70,0x2C,0x44,
	  0x31,0x40,0x00,0x99,0xA3,0x00,0x44,0x10,
	  0x00,0x00,0xD6,0x80,0x28,0x44,0x40,0x08 },	/* pe:0 sus:0 xof:0 eam:0 evb:0 Bagpipe */
	{ 0x00,0x00,0x79,0x85,0x03,0x39,0x81,0x1E,
	  0x00,0x10,0x0A,0x02,0x76,0x62,0x00,0x41,
	  0x10,0x00,0x73,0x76,0xC0,0x25,0x00,0x20,
	  0x20,0x72,0x83,0x9E,0x66,0x05,0x10,0x00 },	/* pe:0 sus:1 xof:0 eam:0 evb:1 Fiddle */
	{ 0x00,0x00,0x79,0x45,0x02,0x40,0xA0,0x42,
	  0x03,0x10,0x28,0x12,0x91,0x90,0x34,0x03,
	  0x60,0x00,0x02,0x40,0xB2,0x60,0x23,0x10,
	  0x00,0x02,0xA0,0xA0,0x3C,0x03,0x20,0x00 },	/* pe:0 sus:1 xof:0 eam:0 evb:1 Shanai */
	{ 0x00,0x00,0x79,0x45,0x30,0x46,0xF5,0x41,
	  0x44,0xE0,0x03,0x70,0x66,0xCE,0x2E,0x44,
	  0x20,0x00,0x20,0x26,0xC5,0x78,0x44,0x77,
	  0x08,0x40,0x55,0xFD,0x04,0x54,0x60,0x00 },	/* pe:0 sus:0 xof:0 eam:1 evb:0 TnklBell */
	{ 0x00,0x00,0x79,0x45,0x40,0x4A,0xE2,0x5C,
	  0x44,0x70,0x01,0x60,0x67,0xF1,0x20,0x44,
	  0x50,0x00,0x60,0x49,0xE2,0x84,0x44,0xA7,
	  0x00,0x60,0x67,0xF7,0x10,0x44,0x20,0x00 },	/* pe:0 sus:0 xof:0 eam:0 evb:0 Agogo */
	{ 0x00,0x00,0x79,0x47,0x43,0x54,0x42,0x00,
	  0x45,0x20,0x80,0x32,0x46,0x60,0x58,0x45,
	  0x20,0x00,0x43,0x44,0xE2,0x00,0x45,0x20,
	  0x05,0x43,0x44,0xF2,0x00,0x00,0x03,0x10 },	/* pe:0 sus:1 xof:0 eam:0 evb:1 SteelDrm */
	{ 0x00,0x00,0x79,0x45,0x91,0x9A,0xF2,0x86,
	  0x44,0x51,0x05,0x71,0x7A,0xF2,0x00,0x44,
	  0x23,0x00,0xA0,0x8A,0xF2,0x70,0x44,0xA3,
	  0x00,0x70,0x7A,0xF2,0x00,0x44,0x20,0x00 },	/* pe:0 sus:0 xof:0 eam:0 evb:0 WoodBlok */
	{ 0x00,0x00,0x79,0x44,0x42,0x3B,0xF5,0x60,
	  0x00,0x00,0x27,0xA2,0x3C,0xF1,0x4C,0x00,
	  0x40,0x00,0x42,0x3C,0xF4,0x2C,0x01,0x30,
	  0x00,0x52,0x5E,0xF0,0x00,0x05,0x00,0x00 },	/* pe:0 sus:1 xof:0 eam:0 evb:1 TaikoDrm */
	{ 0x00,0x00,0x79,0x45,0x82,0x89,0xF5,0x3A,
	  0x44,0x10,0x27,0x52,0x53,0xFE,0x00,0x44,
	  0x00,0x00,0x43,0x43,0xF5,0x32,0x44,0x00,
	  0x08,0x43,0x44,0xFE,0x00,0x44,0x00,0x00 },	/* pe:0 sus:1 xof:0 eam:0 evb:0 MelodTom */
	{ 0x00,0x00,0x79,0xC5,0x81,0xAB,0xE2,0x00,
	  0x04,0x07,0x62,0x01,0x43,0xCF,0x00,0x14,
	  0x00,0x00,0x00,0x44,0xF7,0x00,0x44,0x00,
	  0xC0,0x00,0x77,0xFF,0x00,0x45,0x00,0xB0 },	/* pe:0 sus:0 xof:0 eam:1 evb:1 Syn.Drum */
	{ 0x00,0x00,0x79,0x45,0x02,0x0F,0x40,0x00,
	  0x45,0xE0,0x07,0xF2,0xFF,0x2F,0x18,0x25,
	  0x90,0x40,0x02,0x0F,0x40,0x00,0x45,0xE0,
	  0x07,0xF2,0xFF,0x2F,0x18,0x45,0xE0,0x18 },	/* pe:0 sus:1 xof:0 eam:0 evb:1 RevCymbl */
	{ 0x00,0x00,0x79,0x45,0x42,0x68,0xF2,0x02,
	  0x45,0x60,0x47,0x42,0xA6,0x88,0x3E,0x45,
	  0x30,0x10,0x22,0x66,0xF2,0x00,0x45,0x60,
	  0x51,0x82,0xA8,0x88,0x02,0x45,0x60,0x10 },	/* pe:0 sus:1 xof:0 eam:0 evb:1 FretNoiz */
	{ 0x00,0x00,0x79,0x45,0x02,0x5C,0xA1,0x38,
	  0x00,0x50,0x17,0x82,0x98,0x7F,0x14,0x11,
	  0x10,0x00,0x02,0x5C,0xA1,0x24,0x00,0x80,
	  0x07,0x72,0x98,0x8F,0x30,0x11,0x10,0x00 },	/* pe:0 sus:1 xof:0 eam:1 evb:1 BrthNoiz */
	{ 0x00,0x00,0x79,0x03,0x02,0x0F,0xF0,0x10,
	  0x00,0x30,0x07,0x22,0x22,0xFF,0x54,0x10,
	  0x00,0x20,0x42,0x40,0xFF,0x30,0x00,0x00,
	  0x08,0x42,0x42,0x1F,0x00,0x00,0x00,0x00 },	/* pe:0 sus:1 xof:0 eam:1 evb:0 Seashore */
	{ 0x00,0x00,0x79,0x85,0x33,0x37,0x3A,0x56,
	  0x55,0x50,0x00,0x73,0x76,0x53,0x01,0x55,
	  0xA0,0x00,0x33,0x37,0x3A,0x56,0x55,0x50,
	  0x00,0x43,0x46,0x53,0x01,0x75,0xA7,0x00 },	/* pe:0 sus:1 xof:0 eam:1 evb:1 Tweet */
	{ 0x00,0x00,0x79,0x45,0x03,0x02,0xB6,0x71,
	  0x45,0x50,0x15,0x33,0x34,0xF1,0x0C,0x45,
	  0x40,0x00,0x03,0x02,0xB6,0x71,0x45,0x50,
	  0x15,0x33,0x34,0xF1,0x3C,0x45,0x47,0x00 },	/* pe:0 sus:1 xof:0 eam:0 evb:1 Telphone */
	{ 0x00,0x00,0x79,0xC5,0x02,0x06,0xF0,0x04,
	  0x55,0xF0,0x75,0x02,0x50,0x20,0x10,0x55,
	  0x00,0x00,0x02,0x00,0xF0,0x60,0x44,0x00,
	  0x70,0x02,0x50,0x20,0x2C,0x44,0x00,0x00 },	/* pe:0 sus:1 xof:0 eam:1 evb:1 Helicptr */
	{ 0x00,0x00,0x79,0x45,0x02,0x1C,0xF0,0x00,
	  0x00,0x00,0xC7,0x02,0x52,0x40,0x20,0x00,
	  0x00,0x00,0x02,0x10,0x6F,0x00,0x30,0x90,
	  0xC7,0x72,0x70,0x3F,0x1C,0x00,0x30,0x00 },	/* pe:0 sus:1 xof:0 eam:1 evb:0 Applause */
	{ 0x00,0x00,0x79,0x45,0x02,0x03,0xFF,0x2C,
	  0x01,0x50,0x07,0x82,0x86,0xFB,0x10,0x11,
	  0xF0,0x30,0x02,0x02,0xFF,0x08,0x11,0x50,
	  0x0F,0x82,0x86,0xFB,0x68,0x11,0x50,0x30 }	/* pe:0 sus:1 xof:0 eam:1 evb:1 Gunshot */
};

static const UINT8 gbMuteVoice[32] =
{
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
};


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

typedef struct _MAMIDCNV_DEFINFO
{
	UINT8	prog;
	UINT8	volume;
	UINT8	expression;
	UINT8	modulation;
	UINT8	pitchbend;
	UINT8	bendrange;
	UINT8	lastbendrange;
	UINT8	panpot;
	UINT8	hold1;
	UINT8	mode;
} MAMIDCNV_DEFINFO, *PMAMIDCNV_DEFINFO;

typedef struct _MAMIDCNV_PACKET
{
	SINT32	DeltaTime;
	UINT32	MsgID;
	UINT32	p1;
	UINT32	p2;
	UINT32	p3;
} MAMIDCNV_PACKET, *PMAMIDCNV_PACKET;

typedef struct _tagTrack
{
	UINT8		bSmfCmd;				/* CMD @ now                      */
	UINT32		dwSize;					/* [byte] 0 measns nothing in it. */
	UINT8*		pBase;					/* NULL measns nothing in it.     */
	UINT32		dwOffset;				/* offset byte                    */
	SINT32		dwTicks;				/*                                */
} TRACKINFO, *PTRACKINFO;

typedef struct _MAMIDCNV_INFO
{
	UINT32	dwTimeResolution;	/*                                               */
	UINT8	bFmMode;			/*                                               */
	UINT8*	pTitle;				/*                                               */
	UINT32	dwSizeTitle;		/*                                               */
	UINT8*	pCopyright;			/*                                               */
	UINT32	dwSizeCopyright;	/*                                               */
	UINT8	bSetupBar;			/* 0:No, 1:Yes                                   */
	UINT32	dwStart;			/* Index after SetupBar                          */
	UINT32	dwTotalTicks;		/* Total ticks                                   */
	SINT32	dwDataEndTime;		/* (22.10)[ms]                                   */
	SINT32	dwDelta;			/* (22.10)[ms]                                   */
	UINT8	bNumOfTracks;		/*                                               */
	UINT8	bSmfFormat;			/*                                               */
	TRACKINFO	iTrack[MAX_SMF_TRACKS];
} MAMIDCNV_INFO, *PMAMIDCNV_INFO;

static SINT32			gSeqID;								/* Sequence ID             */
static SINT32			gFileID;							/* File ID                 */
static UINT8			gEnable;							/* 0:disabel               */
static UINT8			gNumOfLoaded;						/*                         */
static UINT8			gSetup;								/* 1: Just after seek      */

static UINT8			gbMaxGain;							/* MaxGain                 */
static UINT8			gbMasterVolume;						/* MsaterVolume            */
static UINT8			gNumOfTracks;						/* Number of tracks        */

static UINT32			gRamBase;							/*                         */
static UINT32			gRamOffset;							/*                         */
static UINT32			gRamSize;							/*                         */

static UINT32			gWtWaveAdr[128];					/*                         */

static UINT8			gSetupBar;							/* 0:No, 1:Yes             */
static MAMIDCNV_DEFINFO	gChDef[MAMIDCNV_MAX_CHANNELS];		/*                         */

static UINT8			gHoldMsgs;							/* Number of messages in Q */
static UINT8			gHoldPtrR;							/* Pointer for Read        */
static MAMIDCNV_PACKET	gMsgBuffer[MAX_SMF_MESSAGES];		/* Message Q               */

static UINT32			gSmfTimeResolution;					/*                         */
static MAMIDCNV_INFO	gSmfInfo[2];						/*                         */

static UINT32			gSmfTotalTicks;						/* Total Ticks             */
static UINT32			gSmfStart;							/* Index after SetupBar    */
static SINT32			gSmfDataEndTime;					/* (22.10)[ms]             */

static SINT32			gSmfCurrentTicks;					/* Ticks @ now             */
static SINT32			gSmfCurrentTime;					/* (22.10)[ms]             */
static SINT32			gSmfEndTime;						/* (22.10)[ms]             */
static SINT32			gSmfDelta;							/* (22.10)[ms]             */
static SINT32			gLastMsgTime;						/* (22.10)[ms]             */

static UINT16			gwBank[MAMIDCNV_MAX_CHANNELS];		/*                         */
static UINT16			gwRealBank[MAMIDCNV_MAX_CHANNELS];
static UINT16			gwRPN[MAMIDCNV_MAX_CHANNELS];		/*                         */
static UINT8			gbLEDFlag[MAMIDCNV_MAX_CHANNELS];	/* bit0: for LED, bit1: for Motor */
static SINT32			gdwNumOnLED;						
static SINT32			gdwNumOnMotor;						
static UINT8			gKEYCON[MAMIDCNV_MAX_CHANNELS];		/* 0:Melady, 1:OFF, 2:ON   */
static UINT32			gNumOn[MAMIDCNV_MAX_CHANNELS];		/* Number of KeyOn         */
static UINT8			gEndFlag;							/*                         */
static SINT32			gSeekTime;							/* [ms]                    */
static UINT32			gTrackEndFlag;						/*                         */

static UINT8			gbMipMute[MAMIDCNV_MAX_CHANNELS];	/* Mute switch (1:mute)    */

static UINT32			gdwMelodyVoice4op[128];
static UINT32			gdwDrumVoice4op[128];
static UINT32			gdwSEVoice[128];
static UINT32			gdwExMelodyVoiceWT[128];
static UINT32			gdwProg[MAMIDCNV_MAX_CHANNELS];
static UINT8			gFMMode;
/*=============================================================================
//	Function Name	:	void	ResetTimeInfo(PMAMIDCNV_INFO pI, UINT32* pdwEndFlag)
//
//	Description		:	Reset time info
//
//	Argument		:	pI			... pointer to the setup storage
//						pdwEndFlag	... pointer to the Endflag
//
//	Return			: none
=============================================================================*/
static void	ResetTimeInfo(PMAMIDCNV_INFO pI, UINT32* pdwEndFlag)
{
	UINT8		bTrack;
	PTRACKINFO	pMt;

	*pdwEndFlag = 0;

	for (bTrack = 0; bTrack < pI->bNumOfTracks; bTrack++)
	{
		pMt = &(pI->iTrack[bTrack]);

		pMt->bSmfCmd = 0;
		pMt->dwOffset = 0;
		pMt->dwTicks = 0;
		if (pMt->dwSize > 0) *pdwEndFlag |= (1L << bTrack);
	}
}


/*=============================================================================
//	Function Name	:	SINT32	GetTrackTime(PMAMIDCNV_INFO pI, UINT32* pdwEndFlag)
//
//	Description		:	Get the 1st DT on the Track
//
//	Argument		:	bTrack		... pointer to the setup storage
//						pI			... pointer to the setup storage
//						pdwEndFlag	... pointer to the Endflag
//
//	Return			: 0 : NoError, < 0 : Error code
=============================================================================*/
static SINT32	GetTrackTime(UINT8 bTrack, PMAMIDCNV_INFO pI, UINT32* pdwEndFlag)
{
	UINT8		bTemp;
	SINT32		dwTime;
	PTRACKINFO	pMt;

	if (((*pdwEndFlag >> bTrack) & 0x01) == 0) return (-1);

	pMt = &(pI->iTrack[bTrack]);

	dwTime = 0;
	do
	{
		if (pMt->dwOffset >= pMt->dwSize)
		{
			*pdwEndFlag &= ~(1L << bTrack);
			return (-1);
		}
		bTemp = pMt->pBase[pMt->dwOffset++];
		dwTime = (dwTime << 7) + (UINT32)(bTemp & 0x7f);
	} while (bTemp >= 0x80);

	pMt->dwTicks += dwTime;
	
	if (bTrack == 7)
	{
		MASMFCNV_DBGMSG(("MaMidCnv : GetTrackTime/ %d, %ld, %ld\n", bTrack, dwTime, pMt->dwTicks));
	}

	return (0);
}


/*=============================================================================
//	Function Name	:	SINT32	GetLeastTimeTrack(PMAMIDCNV_INFO pI, UINT32* pdwEndFlag)
//
//	Description		:	Get the track has 
//
//	Argument		:	pI			... pointer to the setup storage
//						pdwEndFlag	... pointer to the Endflag
//
//	Return			: 0 : NoError, < 0 : Error code
=============================================================================*/
static SINT32	GetLeastTimeTrack(PMAMIDCNV_INFO pI, UINT32* pdwEndFlag)
{
	UINT8		bTrack;
	PTRACKINFO	pMt;
	SINT32		dwMinTicks;
	SINT32		dwMinTrack;

	dwMinTrack = -1;
	dwMinTicks = 0x7fffffffL;
	
	/*--- Get First DTs -----------------------------------*/
	for (bTrack = 0; bTrack < pI->bNumOfTracks; bTrack++)
	{
		if (((*pdwEndFlag >> bTrack) & 0x01) != 0)
		{
			pMt = &(pI->iTrack[bTrack]);

			if (dwMinTicks > pMt->dwTicks)
			{
				dwMinTicks = pMt->dwTicks;
				dwMinTrack = (SINT32)bTrack;
			}
		}
	}

	return (dwMinTrack);
}


/*=============================================================================
//	Function Name	:	SINT32	GetSMFInfo(PMAMIDCNV_INFO pI)
//
//	Description		:	Get SMF info from the file
//
//	Argument		:	pI					... pointer to the setup storage
//
//	Return			: 0 : NoError, < 0 : Error code
=============================================================================*/
static SINT32	GetSMFInfo(PMAMIDCNV_INFO pI)
{
	UINT8		bCmd;
	UINT8		bCmd2;
	UINT32		dwSize;
	
	UINT8		bTemp;
	UINT32		dwTime;
	SINT32		dwTotalTicks;
	SINT32		dwCurrentTime;

	UINT8		bSetup;			/* bit0:beat=1/420, 1:tempo=240@0, 2:beat@240, 3:tempo@240 */
	PTRACKINFO	pMt;
	UINT8		bTrack;
	SINT32		dwTr;
	UINT32		dwEndFlag;
	SINT32		dwDelta;
	UINT8		bCh;
	UINT8		bKey;
	UINT32		preBank[MAMIDCNV_MAX_CHANNELS];

	MASMFCNV_DBGMSG(("MaMidCnv : GetSMFInfo\n"));

	for (bKey = 0; bKey < 128; bKey++)
	{
		gdwMelodyVoice4op[bKey] = 0;
		gdwDrumVoice4op[bKey] = 0;
		gdwSEVoice[bKey] = 0;
		gdwExMelodyVoiceWT[bKey] = 0;
	}
	for (bCh = 0; bCh < MAMIDCNV_MAX_CHANNELS; bCh++)
	{
		gwBank[bCh] = 0x7900;
		preBank[bCh] = 0x7900;
		gdwProg[bCh] = 0;
	}
	gwBank[9] = 0x7800;
	preBank[9] = 0x7800;
	gFMMode = SMF_DEF_FM_MODE;
	bSetup = 0;
	dwTotalTicks = 0;
	dwCurrentTime = 0;
	dwDelta = (500 << 10) / pI->dwTimeResolution;	/* default=0.5sec */

	pI->bFmMode = SMF_DEF_FM_MODE;
	pI->pTitle = NULL;
	pI->dwSizeTitle = 0;
	pI->pCopyright = NULL;
	pI->dwSizeCopyright = 0;
	pI->dwStart = 0;
	pI->bSetupBar = 0;

	ResetTimeInfo(pI, &dwEndFlag);
	for (bTrack = 0; bTrack < pI->bNumOfTracks; bTrack++) GetTrackTime(bTrack, pI, &dwEndFlag);

	/*--- Check SetupBar -----------------------------*/
	if (pI->bSmfFormat == 0)
	{
		while (dwEndFlag != 0)
		{
			dwTr = GetLeastTimeTrack(pI, &dwEndFlag);
			if (dwTr < 0) break;
			pMt = &(pI->iTrack[dwTr]);
			
			dwTime = pMt->dwTicks - dwTotalTicks;
			dwCurrentTime += dwTime * dwDelta;
			dwTotalTicks = pMt->dwTicks;
			if (dwCurrentTime < 0)
			{
				MASMFCNV_DBGMSG(("MaMidCnv : GetSMFInfo Error/ CT2=%ld, %ld, %ld, %ld\n", dwCurrentTime, dwTime, dwDelta, dwTr));
				return (MASMW_ERROR_LONG_LENGTH);
			}

			bCmd = pMt->pBase[pMt->dwOffset++];

			if (bCmd < 0xf0)
			{
				bCh = (UINT8)(bCmd & 0x0F);
				/*--- MidiMsg ---*/
				if (bCmd < 0x80)
				{
					bCmd = pMt->bSmfCmd;
					if (bCmd < 0x80) return (MASMW_ERROR_SMF_CMD);
					pMt->dwOffset--;
					bCh = (UINT8)(bCmd & 0x0F);
				} else {
					if (bCmd < 0xf8) pMt->bSmfCmd = bCmd;
				}
				
				switch (bCmd & 0xf0)
				{
				case 0xC0:	/* Program change */
					gwBank[bCh] = (UINT16)preBank[bCh];
					gdwProg[bCh] = pMt->pBase[pMt->dwOffset];
					pMt->dwOffset++;
					break;
				case 0xD0:	/* Channel pressure */
					pMt->dwOffset++;
					break;
					
				case 0xB0:
					switch (pMt->pBase[pMt->dwOffset])
					{
					case 0x00: /* Bank MSB */
						bCmd2 = pMt->pBase[pMt->dwOffset +1];
						preBank[bCh] = (UINT16)((preBank[bCh] & 0x00FF) | ((UINT16)bCmd2 << 8));
						pMt->dwOffset += 2;
						break;
					case 0x20: /* Bank LSB */
						bCmd2 = pMt->pBase[pMt->dwOffset +1];
						preBank[bCh] = (UINT16)((preBank[bCh] & 0xFF00) | ((UINT16)bCmd2));
						pMt->dwOffset += 2;
						break; 
					default:
						pMt->dwOffset += 2;
						break;
					}
					break;
				case 0x90:
					if ((gwBank[bCh] & 0xFF00) == 0x7800) 
					{
						gdwDrumVoice4op[pMt->pBase[pMt->dwOffset]] = 1;
						pMt->dwOffset += 2;
						break;
					}
					if ((bCh == 9) && ((gwBank[bCh] & 0xFF00) != 0x7900))
					{
						gdwDrumVoice4op[pMt->pBase[pMt->dwOffset]] = 1;
						pMt->dwOffset += 2;
						break;
					}
					gdwMelodyVoice4op[gdwProg[bCh]] = 1;
					pMt->dwOffset += 2;
					break;
				default:
					pMt->dwOffset += 2;
				}
			}
			else
			{
				switch (bCmd)
				{
				case 0xF0:			/* SysEx */
				case 0xF7:			/* SysEx */
					pMt->bSmfCmd = 0;
					dwSize = 0;
					do
					{
						bTemp = pMt->pBase[pMt->dwOffset++];
						dwSize = (dwSize << 7) + (UINT32)(bTemp & 0x7f);
					} while (bTemp >= 0x80);
					
					pMt->dwOffset += dwSize;
					break;

				case 0xF1:			/* System Msg */
				case 0xF3:			/* System Msg */
					pMt->dwOffset++;
					break;
				
				case 0xF2:			/* System Msg */
					pMt->dwOffset += 2;
					break;

				case 0xF4:			/* System Msg */
				case 0xF5:			/* System Msg */
				case 0xF6:			/* System Msg */
				case 0xF8:			/* System Msg */
				case 0xF9:			/* System Msg */
				case 0xFA:			/* System Msg */
				case 0xFB:			/* System Msg */
				case 0xFC:			/* System Msg */
				case 0xFD:			/* System Msg */
				case 0xFE:			/* System Msg */
					break;

				case 0xFF:										/* Meta          */
					bCmd2 = pMt->pBase[pMt->dwOffset++];		/* Meta Command  */
					dwSize = 0;									/* Size          */
					do
					{
						bTemp = pMt->pBase[pMt->dwOffset++];
						dwSize = (dwSize << 7) + (UINT32)(bTemp & 0x7f);
					} while (bTemp >= 0x80);

					switch (bCmd2)
					{
					case 0x03:
						if (dwTr == 0)
						{
							pI->pTitle = &pMt->pBase[pMt->dwOffset];
							pI->dwSizeTitle = dwSize;
						}
						break;
					
					case 0x02:
						pI->pCopyright = &pMt->pBase[pMt->dwOffset];
						pI->dwSizeCopyright = dwSize;
						break;

					case 0x2f:		/* End */
						dwEndFlag &= ~(1L << dwTr);
						break;
						
					case 0x51:		/* Set Tempo */
						switch (dwSize)
						{
						case 3:
						case 4:
							dwTime = ((UINT32)pMt->pBase[pMt->dwOffset] << 16) + 
							         ((UINT32)pMt->pBase[pMt->dwOffset + 1] << 8) +
							          (UINT32)pMt->pBase[pMt->dwOffset + 2];
							if ((dwTotalTicks == 0) && (dwTime == 250000)) bSetup |= 0x02;
							if (dwTotalTicks == (SINT32)pI->dwTimeResolution) bSetup |= 0x08;
							dwTime = (dwTime << 7) / 125;
							dwDelta = dwTime / pI->dwTimeResolution;
						}
						break;
						
					case 0x58:		/* Set TimeSignature */
						if ((dwTotalTicks == 0) && 
						    (pMt->pBase[pMt->dwOffset] == 1) &&
						    (pMt->pBase[pMt->dwOffset + 1] == 2)) bSetup |= 0x01;
						if (dwTotalTicks == (SINT32)pI->dwTimeResolution) bSetup |= 0x04;
						break;
					}
					pMt->dwOffset += dwSize;
					break;
				}
			}
			GetTrackTime((UINT8)dwTr, pI, &dwEndFlag);
			if (bSetup == 0x0B) break;
		}

		if (bSetup == 0x0B)
		{
			dwCurrentTime = 0;
			pI->bSetupBar = 1;
			pI->dwStart = pI->iTrack[0].dwOffset;
			
		} else {
			pI->dwTotalTicks = dwTotalTicks;
			pI->dwDataEndTime = dwCurrentTime;

			MASMFCNV_DBGMSG(("MaMidCnv : GetSMFInfo/ dwTotalTicks  = %ld\n", dwTotalTicks));
			MASMFCNV_DBGMSG(("MaMidCnv : GetSMFInfo/ dwCurrentTime = %ld\n", dwCurrentTime));

			if ((pI->dwDataEndTime >> 10) <= MINIMUM_LENGTH) return (MASMW_ERROR_SHORT_LENGTH);

			return (MASMW_SUCCESS);
		}
	}

	/*--- Check Length -----------------------------*/
	while (dwEndFlag != 0)
	{
		dwTr = GetLeastTimeTrack(pI, &dwEndFlag);
		if (dwTr < 0) break;
		pMt = &(pI->iTrack[dwTr]);
		
		dwTime = pMt->dwTicks - dwTotalTicks;
		dwCurrentTime += dwTime * dwDelta;
		dwTotalTicks = pMt->dwTicks;
		if (dwCurrentTime < 0)
		{
			MASMFCNV_DBGMSG(("MaMidCnv : GetSMFInfo Error/ CT2=%ld, %ld, %ld, %ld\n", dwCurrentTime, dwTime, dwDelta, dwTr));
			return (MASMW_ERROR_LONG_LENGTH);
		}

		bCmd = pMt->pBase[pMt->dwOffset++];

		if (bCmd < 0xf0)
		{
			bCh = (UINT8)(bCmd & 0x0F);
			/*--- MidiMsg ---*/
			if (bCmd < 0x80)
			{
				bCmd = pMt->bSmfCmd;
				if (bCmd < 0x80) return (MASMW_ERROR_SMF_CMD);
				pMt->dwOffset--;
				bCh = (UINT8)(bCmd & 0x0F);
			} else {
				if (bCmd < 0xf8) pMt->bSmfCmd = bCmd;
			}
			
			switch (bCmd & 0xf0)
			{
			case 0xC0:	/* Program change */
				gwBank[bCh] = (UINT16)preBank[bCh];
				gdwProg[bCh] = pMt->pBase[pMt->dwOffset];
				pMt->dwOffset++;
				break;
			case 0xD0:	/* Channel pressure */
				pMt->dwOffset++;
				break;
				
			case 0xB0:
				switch (pMt->pBase[pMt->dwOffset])
				{
				case 0x00: /* Bank MSB */
					bCmd2 = pMt->pBase[pMt->dwOffset +1];
					preBank[bCh] = (UINT16)((preBank[bCh] & 0x00FF) | ((UINT16)bCmd2 << 8));
					pMt->dwOffset += 2;
					break;
				case 0x20: /* Bank LSB */
					bCmd2 = pMt->pBase[pMt->dwOffset +1];
					preBank[bCh] = (UINT16)((preBank[bCh] & 0xFF00) | ((UINT16)bCmd2));
					pMt->dwOffset += 2;
					break; 
				default:
					pMt->dwOffset += 2;
					break;
				}
				break;
			case 0x90:
				if ((gwBank[bCh] & 0xFF00) == 0x7800) 
				{
					gdwDrumVoice4op[pMt->pBase[pMt->dwOffset]] = 1;
					pMt->dwOffset += 2;
					break;
				}
				if ((bCh == 9) && ((gwBank[bCh] & 0xFF00) != 0x7900))
				{
					gdwDrumVoice4op[pMt->pBase[pMt->dwOffset]] = 1;
					pMt->dwOffset += 2;
					break;
				}
				gdwMelodyVoice4op[gdwProg[bCh]] = 1;
				pMt->dwOffset += 2;
				break;
			default:
				pMt->dwOffset += 2;
			}
		}
		else
		{
			switch (bCmd)
			{
			case 0xF0:			/* SysEx */
			case 0xF7:			/* SysEx */
				pMt->bSmfCmd = 0;
				dwSize = 0;
				do
				{
					bTemp = pMt->pBase[pMt->dwOffset++];
					dwSize = (dwSize << 7) + (UINT32)(bTemp & 0x7f);
				} while (bTemp >= 0x80);
				
				pMt->dwOffset += dwSize;
				break;

			case 0xF1:			/* System Msg */
			case 0xF3:			/* System Msg */
				pMt->dwOffset++;
				break;
			
			case 0xF2:			/* System Msg */
				pMt->dwOffset += 2;
				break;

			case 0xF4:			/* System Msg */
			case 0xF5:			/* System Msg */
			case 0xF6:			/* System Msg */
			case 0xF8:			/* System Msg */
			case 0xF9:			/* System Msg */
			case 0xFA:			/* System Msg */
			case 0xFB:			/* System Msg */
			case 0xFC:			/* System Msg */
			case 0xFD:			/* System Msg */
			case 0xFE:			/* System Msg */
				break;

			case 0xFF:										/* Meta          */
				bCmd2 = pMt->pBase[pMt->dwOffset++];		/* Meta Command  */
				dwSize = 0;									/* Size          */
				do
				{
					bTemp = pMt->pBase[pMt->dwOffset++];
					dwSize = (dwSize << 7) + (UINT32)(bTemp & 0x7f);
				} while (bTemp >= 0x80);

				switch (bCmd2)
				{
				case 0x03:
					pI->pTitle = &pMt->pBase[pMt->dwOffset];
					pI->dwSizeTitle = dwSize;
					break;
				
				case 0x02:
					pI->pCopyright = &pMt->pBase[pMt->dwOffset];
					pI->dwSizeCopyright = dwSize;
					break;

				case 0x2f:		/* End */
					dwEndFlag &= ~(1L << dwTr);
					break;
					
				case 0x51:		/* Set Tempo */
					switch (dwSize)
					{
					case 3:
					case 4:
						dwTime = ((UINT32)pMt->pBase[pMt->dwOffset] << 16) + 
						         ((UINT32)pMt->pBase[pMt->dwOffset + 1] << 8) +
						          (UINT32)pMt->pBase[pMt->dwOffset + 2];
						dwTime = (dwTime << 7) / 125;
						dwDelta = dwTime / pI->dwTimeResolution;
					}
					break;
					
				case 0x58:		/* Set TimeSignature */
					break;
				}
				pMt->dwOffset += dwSize;
				break;
			}
		}
		GetTrackTime((UINT8)dwTr, pI, &dwEndFlag);
	}

	gFMMode = pI->bFmMode;

	pI->dwTotalTicks = dwTotalTicks;
	pI->dwDataEndTime = dwCurrentTime;

	MASMFCNV_DBGMSG(("MaMidCnv : GetSMFInfo/ dwTotalTicks  = %ld\n", dwTotalTicks));
	MASMFCNV_DBGMSG(("MaMidCnv : GetSMFInfo/ dwCurrentTime = %ld\n", dwCurrentTime));

	if ((pI->dwDataEndTime >> 10) <= MINIMUM_LENGTH) return (MASMW_ERROR_SHORT_LENGTH);

	return (MASMW_SUCCESS);
}


/*=============================================================================
//	Function Name	:	SINT32	CheckSMF(UINT8* fp, UINT32 fsize, UINT8 mode)
//
//	Description		:	Check SMF structure
//
//	Argument		:	fp			... pointer to the data
//						fsize		... size fo the data
//						mode		... error check (0:No, 1:Yes, 2:ErrorCheck, 3:CNTI)
//
//	Return			: 0 : NoError, < 0 : Error
=============================================================================*/
static SINT32	CheckSMF(UINT8* fp, UINT32 fsize, UINT8 mode)
{
	UINT32	dwTemp;
	UINT32	dwSize;
	PMAMIDCNV_INFO pI;
	UINT32	dwFormat;
	UINT32	dwNumOfTracks;
	UINT32	i;

	MASMFCNV_DBGMSG(("MaMidCnv : CheckSMF\n"));

	if (fsize < 22) 
	{
		MASMFCNV_DBGMSG(("MaMidCnv : CheckSMF Error/ Too small size\n"));
		return (MASMW_ERROR_FILE);
	}
	
	if ((fp[0] != 'M') ||
	    (fp[1] != 'T') ||
	    (fp[2] != 'h') ||
	    (fp[3] != 'd'))
	{
		MASMFCNV_DBGMSG(("MaMidCnv : CheckSMF Error/ Not 'MThd'\n"));
		return (MASMW_ERROR_CHUNK);
	}

	/*--- Check size ----------------------------------------------------*/
	dwTemp = ((UINT32)fp[4] << 24) + ((UINT32)fp[5] << 16) +
	         ((UINT32)fp[6] << 8) + (UINT32)fp[7];
	if (dwTemp != 6)
	{
		MASMFCNV_DBGMSG(("MaMidCnv : CheckSMF Error/ Size != 6\n"));
		return (MASMW_ERROR_CHUNK_SIZE);
	}

	if (mode < 2)
	{
		if (gNumOfLoaded > 0) return (MASMW_ERROR);
		pI = &gSmfInfo[1];
	} else {
		pI = &gSmfInfo[0];
	}
	
	/*--- Check format -------------------------------------------------*/
	dwFormat = ((UINT32)fp[8] << 8) + (UINT32)fp[9];
	if (dwFormat > 1)
	{
		MASMFCNV_DBGMSG(("MaMidCnv : CheckSMF Error/ Not Format 0 or 1\n"));
		return (MASMW_ERROR_SMF_FORMAT);
	}
	
	/*--- Check number of tracks ---------------------------------------*/
	dwNumOfTracks = ((UINT32)fp[10] << 8) + (UINT32)fp[11];
	if (dwNumOfTracks == 0)
	{
		MASMFCNV_DBGMSG(("MaMidCnv : CheckSMF Error/ Number of Tracks = 0\n"));
		return (MASMW_ERROR_SMF_TRACKNUM);
	}
	if ((dwFormat == 0) && (dwNumOfTracks != 1))
	{
		MASMFCNV_DBGMSG(("MaMidCnv : CheckSMF Error/ Number of Tracks > 1\n"));
		return (MASMW_ERROR_SMF_TRACKNUM);
	}
	
	if (dwNumOfTracks > MAX_SMF_TRACKS) dwNumOfTracks = MAX_SMF_TRACKS;
	pI->bNumOfTracks = (UINT8)dwNumOfTracks;

	/*--- Check Time unit --------------------------------------------*/
	dwTemp = ((UINT32)fp[12] << 8) + (UINT32)fp[13];
	pI->dwTimeResolution = dwTemp & 0x7fff;
	if (((dwTemp & 0x8000) != 0) || (pI->dwTimeResolution == 0))
	{
		MASMFCNV_DBGMSG(("MaMidCnv : CheckSMF Error/ Unknown TimeUnit\n"));
		return (MASMW_ERROR_SMF_TIMEUNIT);
	}
	
	fp += 14;
	fsize -= 14;
	for (i = 0; i < dwNumOfTracks; i++)
	{
		if (fsize < 8)
		{
			MASMFCNV_DBGMSG(("MaMidCnv : CheckSMF Error/ Bad size\n"));
			return (MASMW_ERROR_CHUNK_SIZE);
		}

		/*--- Check chunk name --------------------------------------------*/
		if ((fp[0] != 'M') ||
		    (fp[1] != 'T') ||
		    (fp[2] != 'r') ||
		    (fp[3] != 'k'))
		{
			MASMFCNV_DBGMSG(("MaMidCnv : CheckSMF Error/ Not 'MTrk#%d'\n", i));
			return (MASMW_ERROR_CHUNK);
		}
		
		/*--- Check size ----------------------------------------------------*/
		dwSize = ((UINT32)fp[4] << 24) + ((UINT32)fp[5] << 16) +
		         ((UINT32)fp[6] << 8) + (UINT32)fp[7];
		
		if (fsize < (dwSize + 8))
		{
			MASMFCNV_DBGMSG(("MaMidCnv : CheckSMF Error/ Bad size [%ld] vs [%ld]\n", fsize, dwSize + 22));
			return (MASMW_ERROR_CHUNK_SIZE);
		}
		pI->iTrack[i].pBase = &fp[8];
		pI->iTrack[i].dwSize = dwSize;
		fp += (dwSize + 8);
		fsize -= (dwSize + 8);
	}
	pI->bSmfFormat = (UINT8)dwFormat;

	if (mode < 2)
	{
		gSmfTimeResolution = pI->dwTimeResolution;
		gNumOfTracks = pI->bNumOfTracks;
	}
	MASMFCNV_DBGMSG(("MaMidCnv : CheckSMF / Format =  %d\n", dwFormat));
	MASMFCNV_DBGMSG(("MaMidCnv : CheckSMF / #Tracke = %d\n", gNumOfTracks));

	return (GetSMFInfo(pI));
}


#if MA3_SMF_EXMSG_SUPPORT

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

#endif

/*=============================================================================*/
/*	RegVoice()	                                                               */
/*                                                                             */
/*	Desc.                                                                      */
/*		Register default 4op (or SE) voices.                                   */
/*	Param                                                                      */
/*		none                                                                   */
/*	Return                                                                     */
/*      Error code      0 means no error                                       */
/*=============================================================================*/
static void RegVoice(void)
{
#if SMF_DEF_VOICE_MAP
	UINT8 bKey;
	UINT8 i;
static	UINT8 bVoiceParam[31];
	SINT32 ret;
#endif	
		
#if SMF_DEF_VOICE_MAP
	for (bKey=35 ; bKey<82 ; bKey++) /* Drum FM */
	{
		if (gdwDrumVoice4op[bKey] == 1 && gFMMode == FM_4OP_MODE)
		{
			if ((gRamOffset + 30) <= gRamSize)
			{
				if (Drum_4op_Voice[bKey - 24][0] == 0x00) 
				{
					for (i=0 ; i<30 ; i++)
						bVoiceParam[i] = Drum_4op_Voice[bKey - 24][i+2];
					MaDevDrv_SendDirectRamData( (gRamBase + gRamOffset), ENC_8BIT, bVoiceParam, 30);
					ret = MaSndDrv_SetVoice(gSeqID, 0x80, bKey, 1, Drum_4op_Voice[bKey - 24][1], 0xFFFFFFFF);
					ret = MaSndDrv_SetVoice(gSeqID, 0x80, bKey, 1, Drum_4op_Voice[bKey - 24][1], (gRamBase + gRamOffset));
					gRamOffset += 30;
				}
			}
		}
	}
	for (bKey=0 ; bKey<128 ; bKey++) /* Melody FM */
	{
		if (gdwMelodyVoice4op[bKey] == 1 && gFMMode == FM_4OP_MODE)
		{
			if ((gRamOffset + 30) <= gRamSize)
			{
				for (i=0 ; i<30 ; i++)
					bVoiceParam[i] = FM_4op_Voice[bKey][i+2];
				MaDevDrv_SendDirectRamData( (gRamBase + gRamOffset), ENC_8BIT, bVoiceParam, 30);
				ret = MaSndDrv_SetVoice(gSeqID, 0x00, bKey, 1, 0, 0xFFFFFFFF);
				ret = MaSndDrv_SetVoice(gSeqID, 0x00, bKey, 1, 0, (gRamBase + gRamOffset));
				gRamOffset += 30;
			}
		}
	}
#endif	
	

	MaDevDrv_SendDirectRamData((gRamBase + gRamOffset), 0, (const UINT8*)&gbMuteVoice[2], 30);
	MaSndDrv_SetVoice(gSeqID, 0x06, 0x7C, 1, 0, (gRamBase + gRamOffset));
	gRamOffset += 30;
}


/*=============================================================================*/
/*	GetSetupInfo()                                                             */
/*                                                                             */
/*	Desc.                                                                      */
/*		Register voices.                                                       */
/*	Param                                                                      */
/*		none                                                                   */
/*	Return                                                                     */
/*      Error code      0 means no error                                       */
/*=============================================================================*/
static SINT32 GetSetupInfo(void)
{
	UINT8		bCmd;
	UINT8		bCmd2;
	UINT8		bCh;
	UINT8		bSmfCmd;

	UINT8		bTemp;
	UINT32		dwTime;
	UINT32		dwSize;

	UINT32		msgSize;
	UINT32		dwIndex;
	PTRACKINFO	pMt;
	SINT32		dwRet;

#if MA3_SMF_EXMSG_SUPPORT
	UINT8		bBl;
	UINT8		bPc;
	UINT8		bKey;
	UINT8		bBk;
	UINT8		bPg;
	UINT8		bWaveID;

	UINT32		dwAdr;
	UINT32		dwRom;
	UINT32		dwKey;
	UINT32		sz;
	UINT8		bVoice[32];
#endif
	
	RegVoice();

	for (bCh = 0; bCh < MAMIDCNV_MAX_CHANNELS; bCh++)
	{
		gbLEDFlag[bCh] = 0;
		gKEYCON[bCh] = 0;
	}

	if (gSmfInfo[1].bSetupBar == 0) return (MASMW_SUCCESS);

	/*--- Get Setup info ------------------------------*/
	bSmfCmd = 0;
	dwIndex = 0;
	pMt = &(gSmfInfo[1].iTrack[0]);
	pMt->dwOffset = 0;
	dwRet = MASMW_SUCCESS;
	
	while (dwIndex < gSmfInfo[1].dwStart)
	{
		/*--- Get Delta time --------------------------*/
		dwTime = 0;
		do
		{
			bTemp = pMt->pBase[dwIndex++];
			dwTime = (dwTime << 7) + (UINT32)(bTemp & 0x7f);
		} while (bTemp >= 0x80);
		
		bCmd = pMt->pBase[dwIndex++];

		if (bCmd < 0xf0)
		{
			/*--- MidiMsg ---*/
			if (bCmd < 0x80)
			{
				bCmd = bSmfCmd;
				dwIndex--;
			} else {
				if (bCmd < 0xf8) bSmfCmd = bCmd;
			}
			
			switch (bCmd & 0xf0)
			{
			case 0xC0:	/* Program change */
			case 0xD0:	/* Channel pressure */
				dwIndex++;
				break;
				
			default:
				dwIndex += 2;
			}
		}
		else
		{
			switch (bCmd)
			{
			case 0xF0:			/* SysEx */
			case 0xF7:			/* SysEx */
				bSmfCmd = 0;
				dwSize = 0;
				do
				{
					bTemp = pMt->pBase[dwIndex++];
					dwSize = (dwSize << 7) + (UINT32)(bTemp & 0x7f);
				} while (bTemp >= 0x80);

				if (bCmd == 0xf0)
				{
					if (pMt->pBase[dwIndex + dwSize] != 0xf7) break;
					msgSize = dwSize - 1;
				} else {
					msgSize = dwSize;
				}

#if MA3_SMF_EXMSG_SUPPORT
				if (msgSize >= 5)
				{
					if ((pMt->pBase[dwIndex]     != 0x43) ||
					    (pMt->pBase[dwIndex + 1] != 0x79) ||
					    (pMt->pBase[dwIndex + 2] != 0x06)) break;
					
					switch (pMt->pBase[dwIndex + 3])
					{
					case 0x7c:
						if (pMt->pBase[dwIndex + 4] == 0x02)	/* ChMode */
						{
							bCmd = 0;
							bCmd2 = 0;
							for (bCh = 0; bCh < MAMIDCNV_MAX_CHANNELS; bCh++)
							{
								bTemp = pMt->pBase[dwIndex + 5 + bCh];
								gbLEDFlag[bCh] |= bTemp & 0x03;
								gKEYCON[bCh] = (UINT8)(bTemp >> 2);
								dwRet = MaSndDrv_SetKeyControl(gSeqID, bCh, gKEYCON[bCh]);
								if (dwRet < 0) return (dwRet);
							}
							MASMFCNV_DBGMSG(("MaMidCnv : ChMode\n"));
						}
						break;
					
					case 0x7f:
						switch (pMt->pBase[dwIndex + 4])
						{
						case 0x01:	/* Voice */
							bBl = (UINT8)(pMt->pBase[dwIndex + 6] & 0x7f);
							bPc = (UINT8)(pMt->pBase[dwIndex + 7] & 0x7f);
							bKey = (UINT8)(pMt->pBase[dwIndex + 8] & 0x7f);

							switch (pMt->pBase[dwIndex + 5])
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
								return (MASMW_ERROR);
							}

							dwAdr = gRamBase + gRamOffset;

							switch (pMt->pBase[dwIndex + 9])
							{
							case 0x00:	/* FM Voice */
								if (msgSize == 46)
								{	/* 4-OP */
									if (gFMMode != FM_4OP_MODE) return (MASMW_ERROR);
									if ((gRamOffset + 30) > gRamSize) return (MASMW_ERROR);
									sz = Decode7Enc(&pMt->pBase[dwIndex + 10], 36, bVoice, 31);
									if (sz != 31) return (MASMW_ERROR);
								} else {
									/* 2-OP */
									if ((gRamOffset + 16) > gRamSize) return (MASMW_ERROR);
									sz = Decode7Enc(&pMt->pBase[dwIndex + 10], 20, bVoice, 17);
									if (sz != 17) return (MASMW_ERROR);
								}

								sz--;
								dwKey = bVoice[0];
								MaDevDrv_SendDirectRamData(dwAdr, ENC_8BIT, &bVoice[1], sz);
								dwRet = MaSndDrv_SetVoice(gSeqID, bBk, bPg, 1, dwKey, dwAdr);
								gRamOffset += sz;
								MASMFCNV_DBGMSG(("MaMidCnv : FM Voice\n"));
								break;
								
							case 0x01:	/* WT Voice */
								if ((gRamOffset + 13) > gRamSize) return (MASMW_ERROR);
								sz = Decode7Enc(&pMt->pBase[dwIndex + 10], 19, bVoice, 16);
								dwKey = ((UINT32)bVoice[0] << 8) | (UINT32)bVoice[1];
								bWaveID = bVoice[15];

								if (bWaveID < 128)
								{
									bVoice[9] = (UINT8)((gWtWaveAdr[bWaveID] >> 8) & 0xff);
									bVoice[10] = (UINT8)(gWtWaveAdr[bWaveID] & 0xff);
								} else {
									dwRom = (UINT32)MaResMgr_GetDefWaveAddress((UINT8)(bWaveID & 0x7f));
									if (dwRom <= 0) return (MASMW_ERROR);
									bVoice[9] = (UINT8)((dwRom >> 8) & 0xff);
									bVoice[10] = (UINT8)(dwRom & 0xff);
								}
								MaDevDrv_SendDirectRamData(dwAdr, ENC_8BIT, &bVoice[2], 13);
								dwRet = MaSndDrv_SetVoice(gSeqID, bBk, bPg, 2, dwKey, dwAdr);
								gRamOffset += 14;
								MASMFCNV_DBGMSG(("MaMidCnv : WT Voice\n"));
								break;
							}
							break;
							
						case 0x03:	/* Wave */
							dwAdr = gRamBase + gRamOffset;
							if (msgSize < 9) return (MASMW_ERROR);
							sz = msgSize - 7;
							sz = (sz >> 3) * 7 + (((sz & 0x07) == 0) ? 0 : ((sz & 0x07)-1));
							if ((gRamOffset + sz) > gRamSize) return (MASMW_ERROR);
							dwRet = MaDevDrv_SendDirectRamData(dwAdr, ENC_7BIT, &pMt->pBase[dwIndex + 7], msgSize - 7);
							gRamOffset += ((sz + 1) >> 1) << 1;
							gWtWaveAdr[pMt->pBase[dwIndex + 5] & 0x7F] = dwAdr;
							MASMFCNV_DBGMSG(("MaMidCnv : WT Wave\n"));
							break;
						}
						break;
					}
				}
#endif

				dwIndex += dwSize;
				break;

			case 0xF1:			/* System Msg */
			case 0xF3:			/* System Msg */
				dwIndex++;
				break;
			
			case 0xF2:			/* System Msg */
				dwIndex += 2;
				break;

			case 0xF4:			/* System Msg */
			case 0xF5:			/* System Msg */
			case 0xF6:			/* System Msg */
			case 0xF8:			/* System Msg */
			case 0xF9:			/* System Msg */
			case 0xFA:			/* System Msg */
			case 0xFB:			/* System Msg */
			case 0xFC:			/* System Msg */
			case 0xFD:			/* System Msg */
			case 0xFE:			/* System Msg */
				break;

			case 0xFF:										/* Meta          */
				bCmd2 = pMt->pBase[dwIndex++];				/* Meta Command  */
				dwSize = 0;									/* Size          */
				do
				{
					bTemp = pMt->pBase[dwIndex++];
					dwSize = (dwSize << 7) + (UINT32)(bTemp & 0x7f);
				} while (bTemp >= 0x80);
				dwIndex += dwSize;
				break;
			}
		}
		if (dwRet < 0) break;
	}

	pMt->dwOffset = dwIndex;

	return (dwRet);
}


/*=============================================================================
//	Function Name	:	void	SendMasterVolume(SINT32 dwTime, UINT8 bVol)
//
//	Description		:	Send MasterVolume message
//
//	Argument		: dwTime	Delta time
//					  bVol		Master volume (0..127)
//
//	Return			: 0 : NoError, < 0 : Error code
=============================================================================*/
static void	SendMasterVolume(SINT32 dwTime, UINT8 bVol)
{
	PMAMIDCNV_PACKET	pMsg;

	MASMFCNV_DBGMSG(("MaMidCnv : %ld:SendMasterVolume[%d]\n", dwTime, bVol));

	if (gSeqID >= 0)
	{
		if (dwTime >= 0)
		{
			if (gHoldMsgs < MAX_SMF_MESSAGES)
			{
				pMsg = &gMsgBuffer[gHoldMsgs++];
				pMsg->DeltaTime = dwTime;
				pMsg->MsgID = MASNDDRV_CMD_MASTER_VOLUME;
				pMsg->p1 = (UINT32)bVol;
				gLastMsgTime += (dwTime * SMF_TIMEBASE) << 10;
			}
		}
	}
}


/*==============================================================================
//	Function Name	:	void	SendNop(SINT32 dwTime)
//
//	Description		:	Send NOP message
//
//	Argument		:	dwTime			...	Delta time
//						bCh				...	#Channel(0..15)
//
//	Return			:	0: No error, < 0 : Error Code
//
==============================================================================*/
static void	SendNop(SINT32 dwTime)
{
	PMAMIDCNV_PACKET	pMsg;

	MASMFCNV_DBGMSG(("MaMidCnv : %ld:SendNop\n", dwTime));

	if (dwTime >= 0)
	{
		if (gHoldMsgs < MAX_SMF_MESSAGES)
		{
			pMsg = &gMsgBuffer[gHoldMsgs++];
			pMsg->DeltaTime = dwTime;
			pMsg->MsgID = MASNDDRV_CMD_NOP;
			pMsg->p1 = 0;
			gLastMsgTime += (dwTime * SMF_TIMEBASE) << 10;
		}
	}
}


/*==============================================================================
//	Function Name	:	void	ErrorFunction(SINT32 dwTime, UINT8 bCh, UINT8 bPara1, UINT8 bPara2)
//
//	Description		:	Error function
//
//	Argument		: dwTime	Delta time
//					  bPara1	Undefined
//					  bPara2	Undefined
//
//	Return			: 0 : NoError, < 0 : Error code
==============================================================================*/
static void	ErrorFunction(SINT32 dwTime, UINT8 bCh, UINT8 bPara1, UINT8 bPara2)
{
	(void)dwTime;
	(void)bCh;
	(void)bPara1;
	(void)bPara2;

	MASMFCNV_DBGMSG(("MaMidCnv : ErrorFunction\n"));
}


/*==============================================================================
//	Function Name	:	void	NotSupported(SINT32 dwTime, UINT8 bCh, UINT8 bPara1, UINT8 bPara2)
//
//	Description		:	Undefined function
//
//	Argument		: dwTime	Delta time
//					  bPara1	Undefined
//					  bPara2	Undefined
//
//	Return			: 0 : NoError, < 0 : Error code
==============================================================================*/
static void	NotSupported(SINT32 dwTime, UINT8 bCh, UINT8 bPara1, UINT8 bPara2)
{
	(void)dwTime;
	(void)bCh;
	(void)bPara1;
	(void)bPara2;

	MASMFCNV_DBGMSG(("MaMidCnv : NotSupported\n"));
}


/*==============================================================================
//	Function Name	:	void	SendLedOn(void)
//
//	Description		:	Send LedOn message
//
//	Argument		:	none
//
//	Return			:	none
==============================================================================*/
static void SendLedOn(void)
{
	PMAMIDCNV_PACKET	pMsg;

	if (gdwNumOnLED++ > 0)	/* LED is already on. */
		return;
	if (gHoldMsgs < MAX_SMF_MESSAGES) {
		pMsg = &gMsgBuffer[gHoldMsgs++];
		pMsg->DeltaTime = 0;
		pMsg->MsgID = MASNDDRV_CMD_LED_ON;
	}
}

/*==============================================================================
//	Function Name	:	void	SendLedOff(UINT32 dwNumOff)
//
//	Description		:	Send LedOff message
//
//	Argument		:	dwNumOff		...	Number of Off
//
//	Return			:	none
==============================================================================*/
static void SendLedOff(UINT32 dwNumOff)
{
	PMAMIDCNV_PACKET	pMsg;

	if (gdwNumOnLED == 0)	/* LED is already off. */
		return;
	gdwNumOnLED -= dwNumOff;
	if(gdwNumOnLED > 0)
		return;
	if(gdwNumOnLED < 0)
		gdwNumOnLED = 0;

	if (gHoldMsgs < MAX_SMF_MESSAGES) {
		pMsg = &gMsgBuffer[gHoldMsgs++];
		pMsg->DeltaTime = 0;
		pMsg->MsgID = MASNDDRV_CMD_LED_OFF;
	}
}


/*==============================================================================
//	Function Name	:	void	SendMotorOn(void)
//
//	Description		:	Send MotorOn message
//
//	Argument		:	none
//
//	Return			:	none
==============================================================================*/
static void SendMotorOn(void)
{
	PMAMIDCNV_PACKET	pMsg;

	if (gdwNumOnMotor++ > 0)	/* Motor is already on. */
		return;

	if (gHoldMsgs < MAX_SMF_MESSAGES) {
		pMsg = &gMsgBuffer[gHoldMsgs++];
		pMsg->DeltaTime = 0;
		pMsg->MsgID = MASNDDRV_CMD_MOTOR_ON;
	}
}


/*==============================================================================
//	Function Name	:	void	SendMotorOff(UINT32 dwNumOff)
//
//	Description		:	Send MotorOff message
//
//	Argument		:	dwNumOff		...	Number of Off
//
//	Return			:	none
==============================================================================*/
static void SendMotorOff(UINT32 dwNumOff)
{
	PMAMIDCNV_PACKET	pMsg;

	if (gdwNumOnMotor == 0)	/* Motor is already off. */
		return;
	gdwNumOnMotor -= dwNumOff;
	if(gdwNumOnMotor > 0)
		return;
	if(gdwNumOnMotor < 0)
		gdwNumOnMotor = 0;

	if (gHoldMsgs < MAX_SMF_MESSAGES) {
		pMsg = &gMsgBuffer[gHoldMsgs++];
		pMsg->DeltaTime = 0;
		pMsg->MsgID = MASNDDRV_CMD_MOTOR_OFF;
	}
}


/*==============================================================================
//	Function Name	:	void	SendNoteOff(SINT32 dwTime, UINT8 bCh, UNIT8 bKey, UINT8 bVel)
//
//	Description		:	Send NoteOff message
//
//	Argument		:	dwTime			...	Delta time
//						bCh				...	#Channel(0..15)
//						bKey			...	#Key (0..127)
//						bVel			...	Velocity (0..127)
//
//	Return			: 0 : NoError, < 0 : Error code
==============================================================================*/
static void	SendNoteOff(SINT32 dwTime, UINT8 bCh, UINT8 bKey, UINT8 bVel)
{
	PMAMIDCNV_PACKET	pMsg;
	(void)bVel;

	MASMFCNV_DBGMSG(("MaMidCnv : %ld:SendNoteOff[%d, %d] \n", dwTime, bCh, bKey));

	if (dwTime >= 0)
	{
		if (gNumOn[bCh] > 0) gNumOn[bCh]--;
		
		if (gHoldMsgs < MAX_SMF_MESSAGES)
		{
			pMsg = &gMsgBuffer[gHoldMsgs++];
			pMsg->DeltaTime = dwTime;
			pMsg->MsgID = MASNDDRV_CMD_NOTE_OFF;
			pMsg->p1 = (UINT32)bCh;
			pMsg->p2 = (UINT32)bKey;
			
			if (gbLEDFlag[bCh] & 1) SendLedOff(1);
			if (gbLEDFlag[bCh] & 2) SendMotorOff(1);
			gLastMsgTime += (dwTime * SMF_TIMEBASE) << 10;
		}
	}
}


/*==============================================================================
//	Function Name	:	void	SendUserEvent(SINT32 dwTime, UINT8 bID)
//
//	Description		:	Send UserEvent message
//
//	Argument		:	dwTime			...	Delta time
//						bID				...	ID of UserEvent
//
//	Return			:	0: No error, < 0 : Error Code
//
==============================================================================*/
static void	SendUserEvent(SINT32 dwTime, UINT8 bID)
{
	PMAMIDCNV_PACKET	pMsg;

	MASMFCNV_DBGMSG(("MaMidCnv : %ld:SendUserEvent[%d]\n", dwTime, bID));

	if (dwTime >= 0)
	{
		if (gHoldMsgs < MAX_SMF_MESSAGES)
		{
			pMsg = &gMsgBuffer[gHoldMsgs++];
			pMsg->DeltaTime = dwTime;
			pMsg->MsgID = MASNDDRV_CMD_USER_EVENT;
			pMsg->p1 = (UINT32)bID;
			gLastMsgTime += (dwTime * SMF_TIMEBASE) << 10;
		}
	}
}


/*==============================================================================
//	Function Name	:	void	SendNoteOn(SINT32 dwTime, UINT8 bCh, UINT8 bKey, UINT8 bVel)
//
//	Description		:	Send NoteOn message
//
//	Argument		:	dwTime			...	Delta time
//						bCh				...	#Channel(0..15)
//						bKey			...	#Key (0..127)
//						bVel			...	Velocity (0..127)
//
//	Return			: 0 : NoError, < 0 : Error code
==============================================================================*/
static void	SendNoteOn(SINT32 dwTime, UINT8 bCh, UINT8 bKey, UINT8 bVel)
{
	PMAMIDCNV_PACKET	pMsg;

#if SMF_DEF_VELOCITY_MODE
#else
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

	if (bVel == 0)
	{
		 SendNoteOff(dwTime, bCh, bKey, bVel);
		 return;
	}
	
	MASMFCNV_DBGMSG(("MaMidCnv : %ld:SendNoteOn[%d, %d, %d]\n", dwTime, bCh, bKey, bVel));

	if (dwTime >= 0)
	{
		gNumOn[bCh]++;

		if (gbMipMute[bCh] == 0)								/* MIP support             */
		{
			if (gHoldMsgs < MAX_SMF_MESSAGES)
			{
				pMsg = &gMsgBuffer[gHoldMsgs++];
				pMsg->DeltaTime = dwTime;
				pMsg->MsgID = MASNDDRV_CMD_NOTE_ON;
				pMsg->p1 = (UINT32)bCh;
				pMsg->p2 = (UINT32)bKey;
#if SMF_DEF_VELOCITY_MODE
				pMsg->p3 = (UINT32)bVel;
#else
				pMsg->p3 = (UINT32)VelocityTable[bVel & 0x7f];
#endif
				if (gbLEDFlag[bCh] & 1) SendLedOn();
				if (gbLEDFlag[bCh] & 2) SendMotorOn();
				gLastMsgTime += (dwTime * SMF_TIMEBASE) << 10;
			}
		}
	}
}


/*==============================================================================
//	Function Name	:	void	SendProgram(SINT32 dwTime, UINT8 bCh, UNIT8 bProg, UINT8 bPara2)
//
//	Description		:	Send ProgramChange message
//
//	Argument		:	dwTime			...	Delta time
//						bCh				...	#Channel(0..15)
//						bProg			...	#Program (0..255)
//						bPara2			...	Undefined
//
//	Return			: 0 : NoError, < 0 : Error code
==============================================================================*/
static void	SendProgram(SINT32 dwTime, UINT8 bCh, UINT8 bProg, UINT8 bPara2)
{
	UINT8	bBk, bPg;
	UINT16	wBb;
	UINT8	bPp;
	PMAMIDCNV_PACKET	pMsg;

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
	
	MASMFCNV_DBGMSG(("MaMidCnv : %ld:SendProgram[%d] = %d\n", dwTime, bCh, bProg));

	gChDef[bCh].prog = bProg;

	if ((gwBank[bCh] == 0x7906) && (bProg == 0x7C))	/* Ring Vibrator (SP-MIDI) */
	{
		gbLEDFlag[bCh] |= 2;
	}
	else
	{
		gbLEDFlag[bCh] &= ~2;
	}

	if (dwTime >= 0)
	{
		bPp = bProg;
		wBb = gwBank[bCh];
		gwRealBank[bCh] = wBb;
		
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
				if (wBb == 0x7906)	/* Ring Vibrator (SP-MIDI) */
				{
					bBk = 6;
					bPg = 0x7C;
				}
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
				return;
			}
		}

		if (gHoldMsgs < MAX_SMF_MESSAGES)
		{
			pMsg = &gMsgBuffer[gHoldMsgs++];
			pMsg->DeltaTime = dwTime;
			pMsg->MsgID = MASNDDRV_CMD_PROGRAM_CHANGE;
			pMsg->p1 = (UINT32)bCh;
			pMsg->p2 = (UINT32)bBk;
			pMsg->p3 = (UINT32)bPg;
			gLastMsgTime += (dwTime * SMF_TIMEBASE) << 10;
		}
	}
}

/*==============================================================================
//	Function Name	:	void	SendModDepth(SINT32 dwTime, UINT8 bCh, UINT8 bMod)
//
//	Description		:	Send Modulation message
//
//	Argument		:	dwTime			...	Delta time
//						bCh				...	#Channel(0..15)
//						bMod			...	Modulation (0..127)
//
//	Return			: 0 : NoError, < 0 : Error code
==============================================================================*/
static void	SendModDepth(SINT32 dwTime, UINT8 bCh, UINT8 bMod)
{
	PMAMIDCNV_PACKET	pMsg;
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

	MASMFCNV_DBGMSG(("MaMidCnv : %ld:SendModDepth[%d, %d]\n", dwTime, bCh, bMod));

	if (dwTime < 0)
	{
		gChDef[bCh].modulation = bMod;
	} else {
		if (gHoldMsgs < MAX_SMF_MESSAGES)
		{
			pMsg = &gMsgBuffer[gHoldMsgs++];
			pMsg->DeltaTime = dwTime;
			pMsg->MsgID = MASNDDRV_CMD_MODULATION_DEPTH;
			pMsg->p1 = (UINT32)bCh;
			pMsg->p2 = (UINT32)ModTable[bMod];
			gLastMsgTime += (dwTime * SMF_TIMEBASE) << 10;
		}
	}
}


/*==============================================================================
//	Function Name	:	void	SendChVol(SINT32 dwTime, UINT8 bCh, UINT8 bVol)
//
//	Description		:	Send Channel Volume message
//
//	Argument		:	dwTime			...	Delta time
//						bCh				...	#Channel(0..15)
//						bVol			...	Volume (0..127)
//
//	Return			: 0 : NoError, < 0 : Error code
==============================================================================*/
static void	SendChVol(SINT32 dwTime, UINT8 bCh, UINT8 bVol)
{
	PMAMIDCNV_PACKET	pMsg;

	MASMFCNV_DBGMSG(("MaMidCnv : %ld:SendChVol[%d, %d]\n", dwTime, bCh, bVol));

	if (dwTime < 0)
	{
		gChDef[bCh].volume = bVol;
	} else {
		if (gHoldMsgs < MAX_SMF_MESSAGES)
		{
			pMsg = &gMsgBuffer[gHoldMsgs++];
			pMsg->DeltaTime = dwTime;
			pMsg->MsgID = MASNDDRV_CMD_CHANNEL_VOLUME;
			pMsg->p1 = (UINT32)bCh;
			pMsg->p2 = (UINT32)bVol;
			gLastMsgTime += (dwTime * SMF_TIMEBASE) << 10;
		}
	}
}


/*==============================================================================
//	Function Name	:	void	SendExpression(SINT32 dwTime, UINT8 bCh, UINT8 bVol)
//
//	Description		:	Send Expression message
//
//	Argument		:	dwTime			...	Delta time
//						bCh				...	#Channel(0..15)
//						bVol			...	Volume (0..127)
//
//	Return			: 0 : NoError, < 0 : Error code
==============================================================================*/
static void	SendExpression(SINT32 dwTime, UINT8 bCh, UINT8 bVol)
{
	PMAMIDCNV_PACKET	pMsg;
	UINT32				dwVol;

	MASMFCNV_DBGMSG(("MaMidCnv : %ld:SendExpression[%d, %d]\n", dwTime, bCh, bVol));

	if (bVol > 127) bVol = 127;

	if (dwTime < 0)
	{
		gChDef[bCh].expression = bVol;
	} else {
		if (gHoldMsgs < MAX_SMF_MESSAGES)
		{
			dwVol = gdwDBVolTable[bVol] + gdwDBVolTable[gbMasterVolume];
			if (dwVol > 192) dwVol = 192;

			pMsg = &gMsgBuffer[gHoldMsgs++];
			pMsg->DeltaTime = dwTime;
			pMsg->MsgID = MASNDDRV_CMD_EXPRESSION;
			pMsg->p1 = (UINT32)bCh;
			pMsg->p2 = gdwMidVolTable[dwVol];
			gLastMsgTime += (dwTime * SMF_TIMEBASE) << 10;
		}
	}
}


/*==============================================================================
//	Function Name	:	void	SendPanpot(SINT32 dwTime, UINT8 bCh, UINT8 bPan)
//
//	Description		:	Send Panpot message
//
//	Argument		:	dwTime			...	Delta time
//						bCh				...	#Channel(0..15)
//						bPan			...	Panpot (0..127)
//
//	Return			: 0 : NoError, < 0 : Error code
==============================================================================*/
static void	SendPanpot(SINT32 dwTime, UINT8 bCh, UINT8 bPan)
{
	PMAMIDCNV_PACKET	pMsg;

	MASMFCNV_DBGMSG(("MaMidCnv : %ld:SendPanpot[%d, %d]\n", dwTime, bCh, bPan));

	if (dwTime < 0)
	{
		gChDef[bCh].panpot = bPan;
	} else {
		if (gHoldMsgs < MAX_SMF_MESSAGES)
		{
			pMsg = &gMsgBuffer[gHoldMsgs++];
			pMsg->DeltaTime = dwTime;
			pMsg->MsgID = MASNDDRV_CMD_PANPOT;
			pMsg->p1 = (UINT32)bCh;
			pMsg->p2 = (UINT32)bPan;
			gLastMsgTime += (dwTime * SMF_TIMEBASE) << 10;
		}
	}
}


/*==============================================================================
//	Function Name	:	void	SendHold1(SINT32 dwTime, UINT8 bCh, UINT8 bVal)
//
//	Description		:	Send Hold1 message
//
//	Argument		:	dwTime			...	Delta time
//						bCh				...	#Channel(0..15)
//						bVal			...	Hold1 (0..127)
//
//	Return			: 0 : NoError, < 0 : Error code
==============================================================================*/
static void	SendHold1(SINT32 dwTime, UINT8 bCh, UINT8 bVal)
{
	PMAMIDCNV_PACKET	pMsg;

	MASMFCNV_DBGMSG(("MaMidCnv : %ld:SendHold1[%d, %d]\n", dwTime, bCh, bVal));

	if (dwTime < 0)
	{
		gChDef[bCh].hold1 = bVal;
	} else {
		if (gHoldMsgs < MAX_SMF_MESSAGES)
		{
			pMsg = &gMsgBuffer[gHoldMsgs++];
			pMsg->DeltaTime = dwTime;
			pMsg->MsgID = MASNDDRV_CMD_HOLD1;
			pMsg->p1 = (UINT32)bCh;
			pMsg->p2 = (UINT32)((bVal < 64) ? 0 : 1);
			gLastMsgTime += (dwTime * SMF_TIMEBASE) << 10;
		}
	}
}


/*==============================================================================
//	Function Name	:	void	SendBendRange(SINT32 dwTime, UINT8 bCh, UINT8 bVal)
//
//	Description		:	Send BendRange message
//
//	Argument		:	dwTime			...	Delta time
//						bCh				...	#Channel(0..15)
//						bVal			...	BendRange (0..24)
//
//	Return			: 0 : NoError, < 0 : Error code
==============================================================================*/
static void	SendBendRange(SINT32 dwTime, UINT8 bCh, UINT8 bVal)
{
	PMAMIDCNV_PACKET	pMsg;

	MASMFCNV_DBGMSG(("MaMidCnv : %ld:SendBendRange[%d, %d]\n", dwTime, bCh, bVal));

	if (dwTime < 0)
	{
		gChDef[bCh].lastbendrange = bVal;
	} else {
		if (gHoldMsgs < MAX_SMF_MESSAGES)
		{
			pMsg = &gMsgBuffer[gHoldMsgs++];
			pMsg->DeltaTime = dwTime;
			if (bVal <= 24)
			{
				pMsg->MsgID = MASNDDRV_CMD_BEND_RANGE;
				pMsg->p1 = (UINT32)bCh;
				pMsg->p2 = (UINT32)bVal;
			}
			else
			{
				pMsg->MsgID = MASNDDRV_CMD_NOP;
				pMsg->p1 = 0;
			}
			gLastMsgTime += (dwTime * SMF_TIMEBASE) << 10;
		}
	}
}


/*==============================================================================
//	Function Name	:	void	SendAllSoundOff(SINT32 dwTime, UINT8 bCh)
//
//	Description		:	Send AllSoundOff message
//
//	Argument		:	dwTime			...	Delta time
//						bCh				...	#Channel(0..15)
//
//	Return			: 0 : NoError, < 0 : Error code
==============================================================================*/
static void	SendAllSoundOff(SINT32 dwTime, UINT8 bCh)
{
	PMAMIDCNV_PACKET	pMsg;

	MASMFCNV_DBGMSG(("MaMidCnv : %ld:SendAllSoundOff[%d] \n", dwTime, bCh));

	if (dwTime >= 0)
	{
		if (gbLEDFlag[bCh] & 1) SendLedOff(gNumOn[bCh]);
		if (gbLEDFlag[bCh] & 2) SendMotorOff(gNumOn[bCh]);

		gNumOn[bCh] = 0;
		
		if (gHoldMsgs < MAX_SMF_MESSAGES)
		{
			pMsg = &gMsgBuffer[gHoldMsgs++];
			pMsg->DeltaTime = dwTime;
			pMsg->MsgID = MASNDDRV_CMD_ALL_SOUND_OFF;
			pMsg->p1 = (UINT32)bCh;
			gLastMsgTime += (dwTime * SMF_TIMEBASE) << 10;
		}
	}
}


/*==============================================================================
//	Function Name	:	void	SendAllNoteOff(SINT32 dwTime, UINT8 bCh)
//
//	Description		:	Send AllNoteOff messageB
//
//	Argument		:	dwTime			...	Delta time
//						bCh				...	#Channel(0..15)
//
//	Return			: 0 : NoError, < 0 : Error code
==============================================================================*/
static void	SendAllNoteOff(SINT32 dwTime, UINT8 bCh)
{
	PMAMIDCNV_PACKET	pMsg;

	MASMFCNV_DBGMSG(("MaMidCnv : %ld:SendAllNoteOff[%d] \n", dwTime, bCh));

	if (dwTime >= 0)
	{
		if (gbLEDFlag[bCh] & 1) SendLedOff(gNumOn[bCh]);
		if (gbLEDFlag[bCh] & 2) SendMotorOff(gNumOn[bCh]);

		gNumOn[bCh] = 0;
		
		if (gHoldMsgs < MAX_SMF_MESSAGES)
		{
			pMsg = &gMsgBuffer[gHoldMsgs++];
			pMsg->DeltaTime = dwTime;
			pMsg->MsgID = MASNDDRV_CMD_ALL_NOTE_OFF;
			pMsg->p1 = (UINT32)bCh;
			gLastMsgTime += (dwTime * SMF_TIMEBASE) << 10;
		}
	}
}


/*==============================================================================
//	Function Name	:	void	SendResetAllCtl(SINT32 dwTime, UINT8 bCh)
//
//	Description		:	Send ResetAllCtl message
//
//	Argument		:	dwTime			...	Delta time
//						bCh				...	#Channel(0..15)
//
//	Return			: 0 : NoError, < 0 : Error code
==============================================================================*/
static void	SendResetAllCtl(SINT32 dwTime, UINT8 bCh)
{
	PMAMIDCNV_PACKET	pMsg;

	MASMFCNV_DBGMSG(("MaMidCnv : %ld:SendResetAllCtl[%d] \n", dwTime, bCh));

	gwRPN[bCh] = 0x7F7F;
	gChDef[bCh].expression = 127;

	if (dwTime < 0)
	{
		gChDef[bCh].modulation = 0;
		gChDef[bCh].pitchbend = 0x40;
		gChDef[bCh].hold1 = 0;
	} else {
		if (gHoldMsgs < MAX_SMF_MESSAGES)
		{
			pMsg = &gMsgBuffer[gHoldMsgs++];
			pMsg->DeltaTime = dwTime;
			pMsg->MsgID = MASNDDRV_CMD_RESET_ALL_CONTROLLERS;
			pMsg->p1 = (UINT32)bCh;
			gLastMsgTime += (dwTime * SMF_TIMEBASE) << 10;
		}
		SendExpression(0, bCh, gChDef[bCh].expression);
	}
}


/*==============================================================================
//	Function Name	:	void	SendMonoOn(SINT32 dwTime, UINT8 bCh, UINT8 bVal)
//
//	Description		:	Send MonoOn message
//
//	Argument		:	dwTime			...	Delta time
//						bCh				...	#Channel(0..15)
//						bVal			...	Shuld be 0
//
//	Return			: 0 : NoError, < 0 : Error code
==============================================================================*/
static void	SendMonoOn(SINT32 dwTime, UINT8 bCh, UINT8 bVal)
{
	PMAMIDCNV_PACKET	pMsg;

	MASMFCNV_DBGMSG(("MaMidCnv : %ld:SendMonoOn[%d] \n", dwTime, bCh));

	if (bVal != 1) return;

	if (dwTime < 0)
	{
		gChDef[bCh].mode = 0;
	} else {
		if (gHoldMsgs < MAX_SMF_MESSAGES)
		{
			pMsg = &gMsgBuffer[gHoldMsgs++];
			pMsg->DeltaTime = dwTime;
			pMsg->MsgID = MASNDDRV_CMD_MONO_MODE_ON;
			pMsg->p1 = (UINT32)bCh;
			gLastMsgTime += (dwTime * SMF_TIMEBASE) << 10;
		}
	}
}


/*==============================================================================
//	Function Name	:	void	SendPolyOn(SINT32 dwTime, UINT8 bCh, UINT8 bVal)
//
//	Description		:	Send PolyOn messageB
//
//	Argument		:	dwTime			...	Delta time
//						bCh				...	#Channel(0..15)
//						bVal			...	Shuld be 0
//
//	Return			: 0 : NoError, < 0 : Error code
==============================================================================*/
static void	SendPolyOn(SINT32 dwTime, UINT8 bCh, UINT8 bVal)
{
	PMAMIDCNV_PACKET	pMsg;

	MASMFCNV_DBGMSG(("MaMidCnv : %ld:SendPolyOn[%d] \n", dwTime, bCh));

	if (bVal != 0) return;

	if (dwTime < 0)
	{
		gChDef[bCh].mode = 1;
	} else {
		if (gHoldMsgs < MAX_SMF_MESSAGES)
		{
			pMsg = &gMsgBuffer[gHoldMsgs++];
			pMsg->DeltaTime = dwTime;
			pMsg->MsgID = MASNDDRV_CMD_POLY_MODE_ON;
			pMsg->p1 = (UINT32)bCh;
			gLastMsgTime += (dwTime * SMF_TIMEBASE) << 10;
		}
	}
}


/*==============================================================================
//	Function Name	:	void	SendControl(SINT32 dwTime, UINT8 bCh, UNIT8 bMsg2, UINT8 bMsg3)
//
//	Description		:	Send Control message
//
//	Argument		:	dwTime			...	Delta time
//						bCh				...	#Channel(0..15)
//						bMsg2			...	Parameter 1
//						bMsg3			...	Parameter 2
//
//	Return			: 0 : NoError, < 0 : Error code
==============================================================================*/
static void	SendControl(SINT32 dwTime, UINT8 bCh, UINT8 bMsg2, UINT8 bMsg3)
{
	MASMFCNV_DBGMSG(("MaMidCnv : %ld:SendControl[%d, %d, %d] \n", dwTime, bCh, bMsg2, bMsg3));

	switch (bMsg2)
	{
	case 0x00:	/* Bank select(MSB) */
		gwBank[bCh] = (UINT16)((gwBank[bCh] & 0x00FF) | ((UINT16)bMsg3 << 8));
		break;
    
    case 0x20:	/* Bank select (LSB) */
		gwBank[bCh] = (UINT16)((gwBank[bCh] & 0xFF00) | ((UINT16)bMsg3));
		break;

	case 0x01:
		SendModDepth(dwTime, bCh, bMsg3);
		break;

	case 0x06:
		if (gwRPN[bCh] == 0) SendBendRange(dwTime, bCh, bMsg3);
		break;

	case 0x07:
		SendChVol(dwTime, bCh, bMsg3);
		break;

	case 0x0A:
		SendPanpot(dwTime, bCh, bMsg3);
		break;

	case 0x0B:
		SendExpression(dwTime, bCh, bMsg3);
		break;

	case 0x40:
		SendHold1(dwTime, bCh, bMsg3);
		break;

	case 0x62:	/* NRPN (LSB) */
	case 0x63:	/* NRPN (MSB) */
		gwRPN[bCh] |= 0x8000;
		break;

	case 0x64:	/* RPN (LSB) */
		gwRPN[bCh] = (UINT8)((gwRPN[bCh] & 0x7F00) | ((UINT16)bMsg3));
		break;

	case 0x65:	/* RPN (MSB) */
		gwRPN[bCh] = (UINT8)((gwRPN[bCh] & 0x007F) | ((UINT16)bMsg3 << 8));
		break;

	case 0x78:
		SendAllSoundOff(dwTime, bCh);
		break;

	case 0x7B:
		SendAllNoteOff(dwTime, bCh);
		break;

	case 0x79:
		SendResetAllCtl(dwTime, bCh);
		break;

	case 0x7e:
		SendAllNoteOff(dwTime, bCh);
		SendMonoOn(0, bCh, bMsg3);
		break;

	case 0x7f:
		SendAllNoteOff(dwTime, bCh);
		SendPolyOn(0, bCh, bMsg3);
		break;
	}
}


/*==============================================================================
//	Function Name	:	void	SendPitchBend(SINT32 dwTime, UINT8 bCh, UNIT8 bLl, UINT8 bHh)
//
//	Description		:	Send PitchBend message
//
//	Argument		:	dwTime			...	Delta time
//						bCh				...	#Channel(0..15)
//						bLl				...	Lower 7bit of PitchBend (0..127)
//						bHh				...	Upper 7bit of PitchBend (0..127)
//
//	Return			: 0 : NoError, < 0 : Error code
==============================================================================*/
static void SendPitchBend(SINT32 dwTime, UINT8 bCh, UINT8 bLl, UINT8 bHh)
{
	PMAMIDCNV_PACKET	pMsg;
	(void)bLl;

	MASMFCNV_DBGMSG(("MaMidCnv : %ld:SendPitchBend[%d, %d]\n", dwTime, bCh, bHh));

	if (dwTime < 0)
	{
		gChDef[bCh].bendrange = gChDef[bCh].lastbendrange;
		gChDef[bCh].pitchbend = bHh;
	} else {
		if (gHoldMsgs < MAX_SMF_MESSAGES)
		{
			pMsg = &gMsgBuffer[gHoldMsgs++];
			pMsg->DeltaTime = dwTime;
			pMsg->MsgID = MASNDDRV_CMD_PITCH_BEND;
			pMsg->p1 = (UINT32)bCh;
			pMsg->p2 = ((UINT32)bHh) << 7;
			gLastMsgTime += (dwTime * SMF_TIMEBASE) << 10;
		}
	}
}


/*==============================================================================
//	Function Name	:	void	SendGmOn(SINT32 dwTime)
//
//	Description		:	Send GM_ON message
//
//	Argument		:	dwTime			...	Delta time
//
//	Return			:	0: No error, < 0 : Error Code
//
==============================================================================*/
static void	SendGmOn(SINT32 dwTime)
{
	UINT8				bCh;
	PMAMIDCNV_PACKET	pMsg;
	PMAMIDCNV_DEFINFO	pCd;

	MASMFCNV_DBGMSG(("MaMidCnv : %ld:SendGmOn\n", dwTime));

	gbMaxGain = SMF_MAX_GAIN;
	gbMasterVolume = 127;
	for (bCh = 0; bCh < MAMIDCNV_MAX_CHANNELS; bCh++)
	{
		gwRPN[bCh] = 0x7f7f;
		gwBank[bCh] = (bCh == 9) ? (UINT16)0x7800 : (UINT16)0x7900;
		gwRealBank[bCh] = (bCh == 9) ? (UINT16)0x7800 : (UINT16)0x7900;
		gbMipMute[bCh] = 0;
		gbLEDFlag[bCh] = 0;
	}
	
	if (dwTime < 0)
	{
		for (bCh = 0; bCh < MAMIDCNV_MAX_CHANNELS; bCh++)
		{
			pCd = &gChDef[bCh];
			
			pCd->prog = 255;
			pCd->volume = 255;
			pCd->expression = 255;
			pCd->modulation = 255;
			pCd->pitchbend = 255;
			pCd->lastbendrange = 255;
			pCd->bendrange = 255;
			pCd->panpot = 255;
			pCd->hold1 = 255;
			pCd->mode = 255;
		}
	} else {
		SendAllSoundOff(dwTime, 0);
		for (bCh = 1; bCh < MAMIDCNV_MAX_CHANNELS; bCh++)
		{
			SendAllSoundOff(0, bCh);
		}
		for (bCh = 0; bCh < MAMIDCNV_MAX_CHANNELS; bCh++) gNumOn[bCh] = 0;

		if (gHoldMsgs < MAX_SMF_MESSAGES)
		{
			pMsg = &gMsgBuffer[gHoldMsgs++];
			pMsg->DeltaTime = 0;
			pMsg->MsgID = MASNDDRV_CMD_SYSTEM_ON;
		}
		SendMasterVolume(0, gbMaxGain);
	}
}


/*=============================================================================
//	Function Name	:	SINT32		SendShortMsg(SINT32 dwTime, UINT8 bCmd, UINT8 bCh, UINT8 p1, UINT8 p2)
//
//	Description		:	Send MIDI short message
//
//	Argument		:	dwTime			...	Delta time
//						bCmd			...	MIDI command
//						bCh				...	#Channel(0..15)
//						bP1				...	Parameter 1
//						bP2				...	Parameter 2
//
//	Return			: 0 : NoError, < 0 : Error code
=============================================================================*/
static SINT32	SendShortMsg(SINT32 dwTime, UINT8 bCmd, UINT8 bCh, UINT8 bP1, UINT8 bP2)
{
	static void (*MidiMsg[16])(SINT32 dwTm, UINT8 bChannel, UINT8 bPara1, UINT8 bPara2) =
	{
		ErrorFunction,	ErrorFunction,	ErrorFunction,	ErrorFunction, 
		ErrorFunction, 	ErrorFunction,	ErrorFunction,	ErrorFunction, 
		SendNoteOff,	SendNoteOn,		NotSupported,	SendControl,
		SendProgram,	NotSupported,	SendPitchBend,	NotSupported
	};

	static const UINT8	MidiMsgLength[16] = 
	{
		0, 0, 0, 0, 0, 0, 0, 0, 2, 2, 2, 2, 1, 1, 2, 0
	};

	MASMFCNV_DBGMSG(("MaMidCnv : SendShortMsg\n"));

	MidiMsg[bCmd](dwTime, bCh, bP1, bP2);

	return ((SINT32)MidiMsgLength[bCmd]);
}


/*=============================================================================
//	Function Name	:	SINT32	SendSysEx(SINT32 dwTime, UINT8* pMsg, UINT32 dwSize)
//
//	Description		:	Send MIDI Long message
//
//	Argument		:	dwTime			...	Delta time
//						pMsg			...	Pointer to the message
//						dwSize			...	Size of the message [byte]
//
//	Return			: 0 : NoError, < 0 : Error code
=============================================================================*/
static SINT32	SendSysEx(SINT32 dwTime, UINT8* pMsg, UINT32 dwSize)
{
	UINT8	bCh;
	UINT32	dwNumCh;
	UINT32	i;
	UINT8	bMip;
	UINT8	bMax;
	
	MASMFCNV_DBGMSG(("MaMidCnv : SendSysEx[%ld, %08lX, %ld]\n", dwTime, pMsg, dwSize));

	if (dwSize < 4) return (MASMW_SUCCESS);
	
	if ((pMsg[0] == 0x7e) &&
	    (pMsg[1] == 0x7f) &&
	    (pMsg[2] == 0x09) &&
	    ((pMsg[3] == 0x01) || (pMsg[3] == 0x03) || (pMsg[3] == 0x02)) &&
	    (dwSize == 4))
	{
		SendGmOn(dwTime);
		return (MASMW_SUCCESS);
	}
	
	if ((pMsg[0] == 0x7f) &&
	    (pMsg[1] == 0x7f) &&
	    (pMsg[2] == 0x04) &&
	    (pMsg[3] == 0x01) &&
	    (dwSize == 6))
	{
		gbMasterVolume = (UINT8)(pMsg[5] & 0x7f);
		SendExpression(dwTime, 0, gChDef[0].expression);
		for (bCh = 1; bCh < MAMIDCNV_MAX_CHANNELS; bCh++) SendExpression(0, bCh, gChDef[bCh].expression);
		return (MASMW_SUCCESS);
	}

	/*---- MIP(SP-MIDI) message ---------------------------------------------*/
	if ((pMsg[0] == 0x7f) &&
	    (pMsg[1] == 0x7f) &&
	    (pMsg[2] == 0x0B) &&
	    (pMsg[3] == 0x01))
	{
		dwNumCh = (dwSize - 4) >> 1;
		bMax = (gFMMode == FM_2OP_MODE) ? 32 : 16;
		if ((dwNumCh > 0) && (dwNumCh <= MAMIDCNV_MAX_CHANNELS))
		{
			for (i = 0; i < MAMIDCNV_MAX_CHANNELS; i++) gbMipMute[i] = 1;
			for (i = 0; i < dwNumCh; i++)
			{
				bCh = pMsg[(i << 1) + 4];
				bMip = pMsg[(i << 1) + 5];
				if (bMip <= bMax) gbMipMute[bCh] = 0;
			}
		}
	}
	/*---- SP-MIDI message ---------------------------------------------*/

	if ((pMsg[0] != 0x43) ||
	    (pMsg[1] != 0x79) ||
	    (pMsg[2] != 0x06) ||
	    (pMsg[3] != 0x7f) ||
	    (dwSize != 6)) return (MASMW_SUCCESS);

	switch (pMsg[4])
	{
	case 0x00:		/* Max Gain */
		gbMaxGain = (UINT8)(pMsg[5] & 0x7f);
		SendMasterVolume(dwTime, gbMaxGain);
		return (MASMW_SUCCESS);
	}

	return (MASMW_SUCCESS);
}


/*=============================================================================
//	Function Name	:	UINT32	GetTimeCount(void)
//
//	Description		:	Get Delta time
//
//	Argument		:	none
//
//	Return			: 	DeltaTime
=============================================================================*/
static SINT32 GetTimeCount(void)
{
	SINT32	dwTime;

	if (gSmfCurrentTime > gLastMsgTime)
	{
		dwTime = gSmfCurrentTime - gLastMsgTime;	/* [22.10][ms] */
		dwTime /= SMF_TIMEBASE;						/*             */
		dwTime += 0x0200;							/* + 0.5       */
		dwTime >>= 10;
	} else {
		dwTime = 0;
	}
	
	return (dwTime);
}


/*=============================================================================*/
/*	SeekSmfMessage(UINT32 dwSeekTime)                                          */
/*                                                                             */
/*	Desc.                                                                      */
/*		Do seek                                                                */
/* Param                                                                       */
/*		dwSeekTime :	Time [22.10 ms]                                        */
/*	Return                                                                     */
/*      none                                                                   */
/*=============================================================================*/
static void SeekSmfMessage(SINT32 dwSeekTime)
{
	UINT8		bCmd;
	UINT8		bCmd2;
	UINT8		bCh;

	UINT8		bParam1, bParam2;
	UINT32		dwSize;

	UINT8		bTemp;
	UINT32		dwTime;
	SINT32		dwTr;
	SINT32		dwCurrTime;
	SINT32		dwPreTime;

	PMAMIDCNV_INFO pI;
	PTRACKINFO	pMt;
	UINT8		bTrack;

	/*--- Initialize --------------------------*/
	dwPreTime = 0;
	gSmfCurrentTime = 0;
	gSmfCurrentTicks = 0;
	gSmfDelta = (500 << 10) / gSmfTimeResolution;	/* default=0.5sec */

	pI = &gSmfInfo[1];
	ResetTimeInfo(pI, &gTrackEndFlag);
	for (bTrack = 0; bTrack < pI->bNumOfTracks; bTrack++) GetTrackTime(bTrack, pI, &gTrackEndFlag);

	SendGmOn(-1);
	
	/*--- Seek till StartPoint ------------------------*/
	if (pI->bSetupBar == 1)
	{
		while (pI->iTrack[0].dwOffset < pI->dwStart)
		{
			/*--- Get Delta time --------------------------*/
			dwTr = GetLeastTimeTrack(pI, &gTrackEndFlag);
			if (dwTr < 0) break;
			pMt = &(pI->iTrack[dwTr]);
			dwTime = pMt->dwTicks - gSmfCurrentTicks;
			gSmfCurrentTicks = pMt->dwTicks;
			
			bCmd = pMt->pBase[pMt->dwOffset++];
			switch (bCmd)
			{
			case 0xFF:			/* Meta */
				bCmd2 = pMt->pBase[pMt->dwOffset++];
				dwSize = 0;
				do
				{
					bTemp = pMt->pBase[pMt->dwOffset++];
					dwSize = (dwSize << 7) + (UINT32)(bTemp & 0x7f);
				} while (bTemp >= 0x80);

				switch (bCmd2)
				{
				case 0x2f:		/* End */
					gTrackEndFlag &= ~(1L << dwTr);
					break;
					
				case 0x51:		/* Set Tempo */
					switch (dwSize)
					{
					case 3:
					case 4:
						dwTime = ((UINT32)pMt->pBase[pMt->dwOffset] << 16) + 
						         ((UINT32)pMt->pBase[pMt->dwOffset + 1] << 8) +
						          (UINT32)pMt->pBase[pMt->dwOffset + 2];
						dwTime = (dwTime << 7) / 125;
						gSmfDelta = dwTime / gSmfTimeResolution;
						break;
					}
					break;
				}
				pMt->dwOffset += dwSize;
				break;

			case 0xF0:			/* SysEx */
				dwSize = 0;
				do
				{
					bTemp = pMt->pBase[pMt->dwOffset++];
					dwSize = (dwSize << 7) + (UINT32)(bTemp & 0x7f);
				} while (bTemp >= 0x80);
				
				SendSysEx(-1, &pMt->pBase[pMt->dwOffset], dwSize - 1);
				pMt->dwOffset += dwSize;
				break;
				
			case 0xF7:			/* SysEx */
				dwSize = 0;
				do
				{
					bTemp = pMt->pBase[pMt->dwOffset++];
					dwSize = (dwSize << 7) + (UINT32)(bTemp & 0x7f);
				} while (bTemp >= 0x80);
				
				SendSysEx(-1, &pMt->pBase[pMt->dwOffset], dwSize);
				pMt->dwOffset += dwSize;
				break;

			case 0xF1:			/* System Msg */
			case 0xF3:			/* System Msg */
				pMt->dwOffset++;
				break;
			
			case 0xF2:			/* System Msg */
				pMt->dwOffset += 2;
				break;

			case 0xF4:			/* System Msg */
			case 0xF5:			/* System Msg */
			case 0xF6:			/* System Msg */
			case 0xF8:			/* System Msg */
			case 0xF9:			/* System Msg */
			case 0xFA:			/* System Msg */
			case 0xFB:			/* System Msg */
			case 0xFC:			/* System Msg */
			case 0xFD:			/* System Msg */
			case 0xFE:			/* System Msg */
				break;

			default:			/* MidiMsg */
				if (bCmd < 0x80)
				{
					bCmd = pMt->bSmfCmd;
					pMt->dwOffset--;
				} else {
					if (bCmd < 0xf8) pMt->bSmfCmd = bCmd;
				}
				
				bCh = (UINT8)(bCmd & 0x0f);
				bCmd = (UINT8)((bCmd & 0xf0) >> 4);
				bParam1 = pMt->pBase[pMt->dwOffset];
				bParam2 = pMt->pBase[pMt->dwOffset + 1];
				pMt->dwOffset += SendShortMsg(-1, bCmd, bCh, bParam1, bParam2);
			}
			GetTrackTime((UINT8)dwTr, pI, &gTrackEndFlag);
		}
	}
	
	/*--- Seek till SeekTime --------------------------*/
	while (gSmfCurrentTime < dwSeekTime)
	{
		/*--- Get Delta time --------------------------*/
		dwPreTime = gSmfCurrentTime;
		dwTr = GetLeastTimeTrack(pI, &gTrackEndFlag);
		if (dwTr < 0) break;
		pMt = &(pI->iTrack[dwTr]);
		dwTime = pMt->dwTicks - gSmfCurrentTicks;
		dwCurrTime = gSmfCurrentTime + dwTime * gSmfDelta;
		if (dwCurrTime >= dwSeekTime)
		{
			gSmfCurrentTime = dwSeekTime;
			break;
		}

		gSmfCurrentTime = dwCurrTime;
		gSmfCurrentTicks = pMt->dwTicks;
		
		bCmd = pMt->pBase[pMt->dwOffset++];
		switch (bCmd)
		{
		case 0xFF:			/* Meta */
			bCmd2 = pMt->pBase[pMt->dwOffset++];
			dwSize = 0;
			do
			{
				bTemp = pMt->pBase[pMt->dwOffset++];
				dwSize = (dwSize << 7) + (UINT32)(bTemp & 0x7f);
			} while (bTemp >= 0x80);

			switch (bCmd2)
			{
			case 0x2f:		/* End */
				gTrackEndFlag &= ~(1L << dwTr);
				break;
				
			case 0x51:		/* Set Tempo */
				switch (dwSize)
				{
				case 3:
				case 4:
					dwTime = ((UINT32)pMt->pBase[pMt->dwOffset] << 16) + 
					         ((UINT32)pMt->pBase[pMt->dwOffset + 1] << 8) +
					          (UINT32)pMt->pBase[pMt->dwOffset + 2];
					dwTime = (dwTime << 7) / 125;
					gSmfDelta = dwTime / gSmfTimeResolution;
					break;
				}
				break;
			}
			pMt->dwOffset += dwSize;
			break;

		case 0xF0:			/* SysEx */
			dwSize = 0;
			do
			{
				bTemp = pMt->pBase[pMt->dwOffset++];
				dwSize = (dwSize << 7) + (UINT32)(bTemp & 0x7f);
			} while (bTemp >= 0x80);
			
			SendSysEx(-1, &pMt->pBase[pMt->dwOffset], dwSize - 1);
			pMt->dwOffset += dwSize;
			break;
			
		case 0xF7:			/* SysEx */
			dwSize = 0;
			do
			{
				bTemp = pMt->pBase[pMt->dwOffset++];
				dwSize = (dwSize << 7) + (UINT32)(bTemp & 0x7f);
			} while (bTemp >= 0x80);
			
			SendSysEx(-1, &pMt->pBase[pMt->dwOffset], dwSize);
			pMt->dwOffset += dwSize;
			break;

		case 0xF1:			/* System Msg */
		case 0xF3:			/* System Msg */
			pMt->dwOffset++;
			break;
		
		case 0xF2:			/* System Msg */
			pMt->dwOffset += 2;
			break;

		case 0xF4:			/* System Msg */
		case 0xF5:			/* System Msg */
		case 0xF6:			/* System Msg */
		case 0xF8:			/* System Msg */
		case 0xF9:			/* System Msg */
		case 0xFA:			/* System Msg */
		case 0xFB:			/* System Msg */
		case 0xFC:			/* System Msg */
		case 0xFD:			/* System Msg */
		case 0xFE:			/* System Msg */
			break;

		default:			/* MidiMsg */
			if (bCmd < 0x80)
			{
				bCmd = pMt->bSmfCmd;
				pMt->dwOffset--;
			} else {
				if (bCmd < 0xf8) pMt->bSmfCmd = bCmd;
			}
			
			bCh = (UINT8)(bCmd & 0x0f);
			bCmd = (UINT8)((bCmd & 0xf0) >> 4);
			bParam1 = pMt->pBase[pMt->dwOffset];
			bParam2 = pMt->pBase[pMt->dwOffset + 1];
			pMt->dwOffset += SendShortMsg(-1, bCmd, bCh, bParam1, bParam2);
		}
		GetTrackTime((UINT8)dwTr, pI, &gTrackEndFlag);
	}

	gLastMsgTime = gSmfCurrentTime;
	gSmfCurrentTime = dwPreTime;
}


/*=============================================================================*/
/*	ConvertOneSmfMessage()                                                     */
/*                                                                             */
/*	Desc.                                                                      */
/*		SMF Message -> MA-3 packets                                            */
/* Param                                                                       */
/*      nonde                                                                  */
/*	Return                                                                     */
/*      Error code      0 : No more data, 1 : Continue                         */
/*=============================================================================*/
static SINT32 ConvertOneSmfMessage()
{
	UINT8		bCmd;
	UINT8		bCmd2;
	UINT8		bCh;

	UINT8		bParam1, bParam2;
	UINT32		dwSize;

	UINT8		bTemp;
	UINT32		dwTime;
	
	SINT32		dwTr;
	PMAMIDCNV_INFO pI;
	PTRACKINFO	pMt;


	MASMFCNV_DBGMSG(("MaMidCnv : ConvertOneSmf/CT=%ld, ET=%ld\n", gSmfCurrentTime, gSmfEndTime));

	if (gTrackEndFlag == 0) return (0);

	/*--- Get Delta time --------------------------*/
	pI = &gSmfInfo[1];
	dwTr = GetLeastTimeTrack(pI, &gTrackEndFlag);
	if (dwTr < 0) return (0);
	pMt = &(pI->iTrack[dwTr]);
	dwTime = pMt->dwTicks - gSmfCurrentTicks;
	gSmfCurrentTime += dwTime * gSmfDelta;
	gSmfCurrentTicks = pMt->dwTicks;

	if (gSmfCurrentTime >= gSmfEndTime)
	{
		gSmfCurrentTime = gSmfEndTime;
		return (0);
	}

	bCmd = pMt->pBase[pMt->dwOffset++];
	switch (bCmd)
	{
	case 0xFF:			/* Meta */
		bCmd2 = pMt->pBase[pMt->dwOffset++];
		MASMFCNV_DBGMSG(("MaMidCnv : Meta Event[%02X]\n", bCmd2));

		dwSize = 0;
		do
		{
			bTemp = pMt->pBase[pMt->dwOffset++];
			dwSize = (dwSize << 7) + (UINT32)(bTemp & 0x7f);
		} while (bTemp >= 0x80);

		switch (bCmd2)
		{
		case 0x2f:		/* End */
			gTrackEndFlag &= ~(1L << dwTr);
			break;
			
		case 0x51:		/* Set Tempo */
			switch (dwSize)
			{
			case 3:
			case 4:
				dwTime = ((UINT32)pMt->pBase[pMt->dwOffset] << 16) + 
				         ((UINT32)pMt->pBase[pMt->dwOffset + 1] << 8) +
				          (UINT32)pMt->pBase[pMt->dwOffset + 2];
				dwTime = (dwTime << 7) / 125;
				gSmfDelta = dwTime / gSmfTimeResolution;
				break;
			}
			break;
		}
		pMt->dwOffset += dwSize;
		break;

	case 0xF0:			/* SysEx */
		dwSize = 0;
		do
		{
			bTemp = pMt->pBase[pMt->dwOffset++];
			dwSize = (dwSize << 7) + (UINT32)(bTemp & 0x7f);
		} while (bTemp >= 0x80);
		SendSysEx(GetTimeCount(), &pMt->pBase[pMt->dwOffset], dwSize - 1);
		pMt->dwOffset += dwSize;
		break;
		
	case 0xF7:			/* SysEx */
		dwSize = 0;
		do
		{
			bTemp = pMt->pBase[pMt->dwOffset++];
			dwSize = (dwSize << 7) + (UINT32)(bTemp & 0x7f);
		} while (bTemp >= 0x80);
		
		SendSysEx(GetTimeCount(), &pMt->pBase[pMt->dwOffset], dwSize);
		pMt->dwOffset += dwSize;
		break;

	case 0xF1:			/* System Msg */
	case 0xF3:			/* System Msg */
		pMt->dwOffset++;
		break;
	
	case 0xF2:			/* System Msg */
		pMt->dwOffset += 2;
		break;

	case 0xF4:			/* System Msg */
	case 0xF5:			/* System Msg */
	case 0xF6:			/* System Msg */
	case 0xF8:			/* System Msg */
	case 0xF9:			/* System Msg */
	case 0xFA:			/* System Msg */
	case 0xFB:			/* System Msg */
	case 0xFC:			/* System Msg */
	case 0xFD:			/* System Msg */
	case 0xFE:			/* System Msg */
		break;

	default:			/* MidiMsg */
		if (bCmd < 0x80)
		{
			bCmd = pMt->bSmfCmd;
			pMt->dwOffset--;
		} else {
			if (bCmd < 0xf8) pMt->bSmfCmd = bCmd;
		}
		
		bCh = (UINT8)(bCmd & 0x0f);
		bCmd = (UINT8)((bCmd & 0xf0) >> 4);
		bParam1 = pMt->pBase[pMt->dwOffset];
		bParam2 = pMt->pBase[pMt->dwOffset + 1];
		pMt->dwOffset += SendShortMsg(GetTimeCount(), bCmd, bCh, bParam1, bParam2);
	}
	GetTrackTime((UINT8)dwTr, pI, &gTrackEndFlag);
	
	return (1);
}


/*=============================================================================
//	Function Name	:	SINT32	MaMidCnv_Initialize(void)
//
//	Description		:	Initialize the converter
//
//	Argument		:	none
//
//	Return			: 0 : NoError, < 0 : Error code
=============================================================================*/
SINT32	MaMidCnv_Initialize(void)
{
	UINT8	i;

	MASMFCNV_DBGMSG(("MaMidCnv_Initialize \n"));

	gSeqID = -1;						/* Sequence ID      */
	gFileID = -1;						/* File ID          */
	gEnable = 0;						/* 0:disabel        */
	gNumOfLoaded = 0;					/*                  */

	gbMaxGain = SMF_MAX_GAIN;			/* SMF_MAX_GAIN[dB] */
	gbMasterVolume = 127;

	gRamBase = 0;
	gRamOffset = 0;
	gRamSize = 0;

	for (i = 0; i < 128 ; i++) gWtWaveAdr[i] = 0;

	return (MASMW_SUCCESS);
}


/*=============================================================================
//	Function Name	:	SINT32	MaMidCnv_End(void)
//
//	Description		:	Ending
//
//	Argument		:	none
//
//	Return			: 0 : NoError, < 0 : Error code
=============================================================================*/
SINT32	MaMidCnv_End(void)
{
	MASMFCNV_DBGMSG(("MaMidCnv_End \n"));

	return (MASMW_SUCCESS);
}


/*=============================================================================
//	Function Name	:	SINT32	MaMidCnv_Load(UINT8* file_ptr, UINT32 file_size, UINT8 mode,
//												SINT32 (*func)(UINT8 id), void* ext_args)
//
//	Description		:	Load SMF data.
//
//	Argument		:	file_ptr	... pointer to the data
//						file_size	... size fo the data
//						mode		... error check (0:No, 1:Yes, 2:Check, 3:OnlyInfo)
//						func		... pointer of rhe callback function
//						ext_args	... NULL
//
//	Return			: 0 : NoError, < 0 : Error code
=============================================================================*/
SINT32	MaMidCnv_Load(UINT8* file_ptr, UINT32 file_size, UINT8 mode, SINT32 (*func)(UINT8 id), void* ext_args)
{
	UINT32	dwRet;
	(void)ext_args;
	(void)func;

	MASMFCNV_DBGMSG(("MaMidCnv_Load[%08lX %ld %d] \n", file_ptr, file_size, mode));

	if (file_ptr == NULL) return (MASMW_ERROR);
	if (file_size == 0) return (MASMW_ERROR);

	dwRet = CheckSMF(file_ptr, file_size, mode);
	if (mode > 1) return (dwRet);
	if (dwRet != MASMW_SUCCESS) return (dwRet);

	gNumOfLoaded++;
	gEnable = 0;
	gFileID = gNumOfLoaded;

	return ((UINT32)(gNumOfLoaded));
}


/*=============================================================================
//	Function Name	:	SINT32	MaMidCnv_Unload(SINT32 file_id, void* ext_args)
//
//	Description		:	Unload SMF data.
//
//	Argument		:	file_id		...	File ID
//						ext_args	...	NULL
//
//	Return			: 0 : NoError, < 0 : Error code
=============================================================================*/
SINT32	MaMidCnv_Unload(SINT32 file_id, void* ext_args)
{
	(void)ext_args;

	MASMFCNV_DBGMSG(("MaMidCnv_Unload[%d] \n", file_id));

	if ((file_id < 0) || (file_id != gFileID))
	{
		return (MASMW_ERROR);
	}

	gNumOfLoaded = 0;
	gFileID = -1;
	gEnable = 0;

	return (MASMW_SUCCESS);
}


/*=============================================================================
//	Function Name	:	MaMidCnv_Open(SINT32 file_id, UINT16 mode, void* ext_args)
//
//	Description		:	Open SMF data.
//
//	Argument		:	file_id		...	File ID
//						mode		...	Open Mode (0..16)
//						ext_args	...	NULL
//
//	Return			: 0 : NoError, < 0 : Error code
=============================================================================*/
SINT32	MaMidCnv_Open(SINT32 file_id, UINT16 mode, void* ext_args)
{
	UINT8	bTgMode;
	UINT32	Ram;
	UINT32	Size;
	PMAMIDCNV_INFO	pI;
	(void)mode;
	(void)ext_args;
	
	MASMFCNV_DBGMSG(("MaMidCnv_Open[%d] \n", file_id));

	if ((file_id < 0) || (file_id != gFileID))
	{
		return (MASMW_ERROR);
	}

	pI = &gSmfInfo[1];

	bTgMode = (UINT8)(DRUM_GML1 + DRUM_NORMAL + MELODY_NORMAL + DVA_SIMPLE + gFMMode);

	gSeqID = MaSndDrv_Create(	(UINT8)MASMW_SEQTYPE_DELAYED,
								(UINT8)SMF_TIMEBASE,		/* Timebase             */
								(UINT8)bTgMode,				/*                      */
								(UINT8)0,					/*                      */
								(UINT8)0,					/* Num of Streams       */
								MaMidCnv_Convert,			/*                      */
								&Ram,						/*                      */
								&Size);						/*                      */

	if (gSeqID < 0)
	{
		MASMFCNV_DBGMSG(("MaMidCnv_Open/ Can't Open SndDrv\n"));
		return (MASMW_ERROR);
	}
	gRamBase = Ram;
	gRamSize = Size;
	gRamOffset = 0;
	gbMaxGain = SMF_MAX_GAIN;
	gEnable = 0;
	gSetup = 0;
	gFileID = gNumOfLoaded;
	gSetupBar = pI->bSetupBar;
	gSmfStart = pI->dwStart;
	gSmfTotalTicks  = pI->dwTotalTicks;
	gSmfDataEndTime = pI->dwDataEndTime + 1;
	gSmfEndTime = gSmfDataEndTime;

	return (MASMW_SUCCESS);
}


/*=============================================================================
//	Function Name	:	SINT32	MaMidCnv_Close(SINT32 file_id, void* ext_args)
//
//	Description		:	Close the data.
//
//	Argument		:	file_id		...	File ID
//						ext_args	...	NULL
//
//	Return			: 0 : NoError, < 0 : Error code
=============================================================================*/
SINT32	MaMidCnv_Close(SINT32 file_id, void* ext_args)
{
	SINT32	dwRet;
	(void)ext_args;
	
	MASMFCNV_DBGMSG(("MaMidCnv_Close[%d] \n", file_id));

	if ((file_id < 0) || (file_id != gFileID))
	{
		return (MASMW_ERROR);
	}

	dwRet = MaSndDrv_Free(gSeqID);

	gSeqID = -1;
	gEnable = 0;

	return (dwRet);
}


/*=============================================================================
//	Function Name	:	SINT32	MaMidCnv_Standby(SINT32 file_id, void* ext_args)
//
//	Description		:	Setup for playing SMF
//
//	Argument		:	file_id		...	File ID
//						ext_args	...	NULL
//
//	Return			: 0 : NoError, < 0 : Error code
=============================================================================*/
SINT32	MaMidCnv_Standby(SINT32 file_id, void* ext_args)
{
	SINT32		dwRet;
	(void)ext_args;

	MASMFCNV_DBGMSG(("MaMidCnv_Standby[%d] \n", file_id));

	if ((file_id < 0) || (file_id != gFileID))
	{
		return (MASMW_ERROR);
	}

	dwRet = GetSetupInfo();
	gEnable = 1;

	return (dwRet);
}


/*=============================================================================
//	Function Name	:	SINT32	MaMidCnv_Seek(SINT32 file_id, UINT32 pos, UINT8 flag, void* ext_args)
//
//	Description		:	Seek the position.
//
//	Argument		:	file_id		...	File ID
//						pos			...	Start Point [ms]
//						flag		...	Wait 0 : None, 1 : 20[ms]
//						ext_args	...	NULL
//
//	Return			: 0 : NoError, < 0 : Error
=============================================================================*/
SINT32	MaMidCnv_Seek(SINT32 file_id, UINT32 pos, UINT8 flag, void* ext_args)
{
	UINT8				bCh;
	UINT8				bVal;
	PMAMIDCNV_DEFINFO	pCd;
	(void)ext_args;

	MASMFCNV_DBGMSG(("MaMidCnv_Seek[%d] = %ld[ms] \n", file_id, pos));

	if ((file_id < 0) || (file_id != gFileID))
	{
		return (MASMW_ERROR);
	}
	
	gSeekTime = pos;
	gHoldMsgs = 0;
	gHoldPtrR = 0;

	SendGmOn(0);

	SeekSmfMessage((SINT32)pos << 10);
	
	for (bCh = 0; bCh < MAMIDCNV_MAX_CHANNELS; bCh++)
	{
		pCd = &gChDef[bCh];
		
		gNumOn[bCh] = 0;

		bVal = pCd->prog;
		if (bVal < 255) SendProgram(0, bCh, bVal, 0);

		bVal = pCd->volume;
		if (bVal < 255) SendChVol(0, bCh, bVal);

		bVal = pCd->expression;
		if (bVal < 255) SendExpression(0, bCh, bVal);

		bVal = pCd->modulation;
		if (bVal < 255) SendModDepth(0, bCh, bVal);

		bVal = pCd->bendrange;
		if (bVal < 255) SendBendRange(0, bCh, bVal);

		bVal = pCd->pitchbend;
		if (bVal < 255) SendPitchBend(0, bCh, 0, bVal);

		bVal = pCd->lastbendrange;
		if (bVal != pCd->bendrange) SendBendRange(0, bCh, bVal);

		bVal = pCd->panpot;
		if (bVal < 255) SendPanpot(0, bCh, bVal);

		bVal = pCd->hold1;
		if (bVal < 255) SendHold1(0, bCh, bVal);

		if (pCd->mode == 0) SendMonoOn(0, bCh, 0);
	}
	SendMasterVolume(0, gbMaxGain);
	SendNop((SINT32)((flag == 0) ? 0 : (20 / SMF_TIMEBASE)));

	gSetup = 1;
	gEndFlag = 0;

	return (MaSndDrv_ControlSequencer(gSeqID, 4));
}


/*=============================================================================
//	Function Name	:	SINT32	MaMidCnv_Start(SINT32 file_id, void* ext_args)
//
//	Description		:	Start SMF play.
//
//	Argument		:	file_id		...	File ID
//						ext_args	...	NULL
//
//	Return			: 0 : NoError, < 0 : Error
=============================================================================*/
SINT32	MaMidCnv_Start(SINT32 file_id, void* ext_args)
{
	(void)ext_args;

	MASMFCNV_DBGMSG(("MaMidCnv_Start[%d] \n", file_id));

	if ((file_id < 0) || (file_id != gFileID))
	{
		return (MASMW_ERROR);
	}
	gSetup = 0;
	gdwNumOnLED = 0;
	gdwNumOnMotor = 0;

	return (MaSndDrv_ControlSequencer(gSeqID, 1));
}


/*=============================================================================
//	Function Name	:	SINT32	MaMidCnv_Stop(SINT32 file_id, void* ext_args)
//
//	Description		:	Stop SMF play.
//
//	Argument		:	file_id		...	File ID
//						ext_args	...	NULL
//
//	Return			: 0 : NoError, < 0 : Error
=============================================================================*/
SINT32	MaMidCnv_Stop(SINT32 file_id, void* ext_args)
{
	(void)ext_args;

	MASMFCNV_DBGMSG(("MaMidCnv_Stop[%d] \n", file_id));

	if ((file_id < 0) || (file_id != gFileID))
	{
		return (MASMW_ERROR);
	}

	return (MaSndDrv_ControlSequencer(gSeqID, 0));
}


/*=============================================================================*/
/*	GetContentsInfo(SINT32 file_id, PMASMW_CONTENTSINFO pInfo)                 */
/*                                                                             */
/*	Desc.                                                                      */
/*		Get contents info                                                      */
/*	Param                                                                      */
/*		file_id :	file ID                                                    */
/*		pInfo :		pointer to the command block                               */
/*	Return                                                                     */
/*      Error code      0 means no error                                       */
/*=============================================================================*/
static SINT32 GetContentsInfo(SINT32 file_id, PMASMW_CONTENTSINFO pInfo)
{
	PMAMIDCNV_INFO	pI;
	UINT8*	pData;
	UINT32	dwSize;
	UINT16	wIndex;
	UINT8*	pDst;
	UINT32	i;
	
	if ((file_id != 0) && (file_id != 1)) return (MASMW_ERROR);
	
	pI = &gSmfInfo[file_id];
	wIndex = (UINT16)(((UINT16)pInfo->tag[0] << 8) + (UINT16)pInfo->tag[1]);

	switch (wIndex)
	{
	case 0x5354:	/* ST */
		pData = pI->pTitle;
		dwSize = pI->dwSizeTitle;
		break;

	case 0x4352:	/* CR */
		pData = pI->pCopyright;
		dwSize = pI->dwSizeCopyright;
		break;

	default:
		return (MASMW_ERROR);
	}

	if (pData == NULL) return (MASMW_ERROR);
	if (dwSize == 0) return (MASMW_ERROR);
	if (dwSize > pInfo->size) dwSize = pInfo->size;

	pDst = pInfo->buf;
	for (i = 0; i < dwSize; i++) *pDst++ = *pData++;
	
	return (dwSize);
}


/*=============================================================================
//	Function Name	:	SINT32	MaMidCnv_Control(SINT32 file_id, UINT8 ctrl_num, void* prm, void* ext_args)
//
//	Description		:	Send Control message
//
//	Argument		:	file_id		...	File ID
//						ctrl_num	...	Command ID
//						prm			...	Parameters
//						ext_args	...	NULL
//
//	Return			: 0 : NoError, < 0 : Error
=============================================================================*/
SINT32	MaMidCnv_Control(SINT32 file_id, UINT8 ctrl_num, void* prm, void* ext_args)
{
	SINT32	dwRet;
	(void)ext_args;
	
	MASMFCNV_DBGMSG(("MaMidCnv_Control[%d, %d] \n", file_id, ctrl_num));

	if ((file_id < 0) || (file_id > 1))
	{
		return (MASMW_ERROR);
	}

	switch(ctrl_num)
	{
	case MASMW_SET_VOLUME :
		if (file_id != gFileID)
		{
			return (MASMW_ERROR);
		}
		return (MaSndDrv_SetVolume(gSeqID, *((UINT8*)prm)));

	case MASMW_GET_POSITION :
		if (file_id != gFileID)
		{
			return (MASMW_ERROR);
		}
		dwRet = MaSndDrv_GetPos(gSeqID);
		return (dwRet);

	case MASMW_GET_LENGTH :
		if ((file_id != gFileID) && (file_id != 0))
		{
			return (MASMW_ERROR);
		}
		return ((gSmfInfo[file_id].dwDataEndTime + 1) >> 10);
	
	case MASMW_GET_CONTENTSDATA :
		return (GetContentsInfo(file_id, (MASMW_CONTENTSINFO*)prm));
	
	case MASMW_SET_ENDPOINT :
		if (file_id != gFileID)
		{
			return (MASMW_ERROR);
		}
		gSmfEndTime = (SINT32)(*((UINT32*)prm) << 10);
		if (gSmfEndTime > gSmfDataEndTime) gSmfEndTime = gSmfDataEndTime;
		break;

	case MASMW_SET_SPEED:
		if (file_id != gFileID)
		{
			return (MASMW_ERROR);
		}
		return (MaSndDrv_SetSpeed(gSeqID, *((UINT8 *)prm)));

	case MASMW_SET_KEYCONTROL:
		if (file_id != gFileID)
		{
			return (MASMW_ERROR);
		}
		return MaSndDrv_SetKey(gSeqID, *((SINT8 *)prm));

	case MASMW_GET_TIMEERROR:
		if (file_id != gFileID)
		{
			return (MASMW_ERROR);
		}
		return MaSndDrv_GetTimeError(gSeqID);

	}

	return (MASMW_SUCCESS);
}


/*=============================================================================
//	Function Name	:	SINT32	MaMidCnv_Convert(void)
//
//	Description		:	Do convert
//
//	Argument		:	none
//
//	Return			: 0 : NoMoreData
//					 >0 : Number of messages
//					 <0 : Error Code
=============================================================================*/
SINT32	MaMidCnv_Convert(void)
{
	PMAMIDCNV_PACKET	pMsg;
	UINT8				i;

	if (gEnable == 0) return (0);

	if (gHoldMsgs > 0)
	{
		gHoldMsgs--;
		pMsg = &gMsgBuffer[gHoldPtrR++];
		MaSndDrv_SetCommand(gSeqID, pMsg->DeltaTime, pMsg->MsgID, pMsg->p1, pMsg->p2, pMsg->p3);
		if ((gHoldMsgs == 0) && (gEndFlag == 1)) gEndFlag = 2;
		return (1);
	};
	if (gSetup == 1) return (0xFFFF);
	if (gEndFlag == 2)
	{
		gEndFlag = 3;
		return (0);
	}
	else if (gEndFlag == 3) gEndFlag = 0;

	
	gHoldPtrR = 0;
	while (gHoldMsgs == 0)
	{
		if (ConvertOneSmfMessage() <= 0) break;
	}
	
	if (gHoldMsgs == 0)
	{
		if (gEndFlag == 0)
		{
			/*--- Ending... -----------------------*/
			MASMFCNV_DBGMSG((">> MaMidCnv_Convert/ Ending...\n"));
			SendAllSoundOff(GetTimeCount(), 0);
			for (i = 1; i < MAMIDCNV_MAX_CHANNELS; i++) SendAllSoundOff(0, i);
			SendUserEvent(0, MASMW_END_OF_SEQUENCE);
			gEndFlag = 1;
		} else {
			MASMFCNV_DBGMSG(("MaMidCnv_Convert/ End\n"));
			return (0);
		}
	}
	
	gHoldMsgs--;
	pMsg = &gMsgBuffer[gHoldPtrR++];
	MaSndDrv_SetCommand(gSeqID, pMsg->DeltaTime, pMsg->MsgID, pMsg->p1, pMsg->p2, pMsg->p3);
	return (1);
}
