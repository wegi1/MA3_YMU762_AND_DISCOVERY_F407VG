/****************************************************************************
 *																			*
 *		Copyright (C) 2001-2002	YAMAHA CORPORATION. All rights reserved.	*
 *																			*
 *		Module		: madefs.h												*
 *																			*
 *		Description : MA Definitions										*
 *																			*
 * 		Version		: 1.3.12.3	2002.11.15									*
 *																			*
 ****************************************************************************/

#ifndef __MADEFS_H__
#define __MADEFS_H__

#define MA_BUSY							(0x80)
#define MA_INT							(0x80)		/* bit 7 */
#define MA_BUSY_DW						(0x40)		/* bit 6 */
#define MA_BUSY_W						(0x20)		/* bit 5 */
#define MA_VALID_R2						(0x04)		/* bit 2 */
#define MA_VALID_R1						(0x02)		/* bit 1 */
#define MA_VALID_R0						(0x01)		/* bit 0 */
#define MA_EMP_W						(0x01)
#define MA_EMP_DW						(0x02)
#define MA_FULL_W						(0x04)
#define MA_FULL_DW						(0x08)

#define MA_DP3							(0x08)
#define MA_DP2							(0x04)
#define MA_DP1							(0x02)
#define MA_DP0							(0x01)

#define MA_PLLPD						(0x80)
#define MA_AP4R							(0x20)
#define MA_AP4L							(0x10)
#define MA_AP3							(0x08)
#define MA_AP2							(0x04)
#define MA_AP1							(0x02)
#define MA_AP0							(0x01)

#define MA_CTRLREG_SIZE					(360)

#define MA_MEMORY_SIZE					(0x6000)	/* 24Kbytes */
#define MA_ROM_SIZE						(0x4000)	/* 16Kbytes */
#define MA_RAM_SIZE						(0x2000)	/*  8Kbytes */

#define MA_ROM_START_ADDRESS			(0x0000)
#define MA_ROM_END_ADDRESS				(0x3FFF)
#define MA_RAM_START_ADDRESS			(0x4000)
#define MA_RAM_END_ADDRESS				(0x5FFF)

#define MA_NORMAL_ROM_ADDRESS			(0x0800)
#define MA_DRUM_ROM_ADDRESS				(0x1000)

#define MA_INTERRUPT_FLAG_REG			(0x00)
#define MA_DELAYED_WRITE_REG			(0x01)
#define MA_IMMEDIATE_WRITE_REG			(0x02)
#define MA_IMMEDIATE_READ_REG			(0x03)
#define MA_BASIC_SETTING_REG			(0x04)
#define MA_POWER_MANAGEMENT_DIGITAL_REG	(0x05)
#define MA_POWER_MANAGEMENT_ANALOG_REG	(0x06)
#define MA_ANALOG_EQVOL_REG				(0x07)
#define MA_ANALOG_HPVOL_L_REG			(0x08)
#define MA_ANALOG_HPVOL_R_REG			(0x09)
#define MA_ANALOG_SPVOL_REG				(0x0A)
#define MA_LED_SETTING_1_REG			(0x0B)
#define MA_LED_SETTING_2_REG			(0x0C)
#define MA_MOTOR_SETTING_1_REG			(0x0D)
#define MA_MOTOR_SETTING_2_REG			(0x0E)

#define MA_PLL_SETTING_1_REG			(0x05)
#define MA_PLL_SETTING_2_REG			(0x06)

#define MA_VALID_RX						(4)

#define MA_FM_VOICE_ADDRESS				(0)
#define MA_WT_VOICE_ADDRESS				(96)
#define MA_EXT_FM_VOICE_ADDRESS			(224)
#define MA_CHANNEL_VOLUME				(144)
#define MA_CHANNEL_PANPOT				(160)
#define MA_CHANNEL_SUSTAIN				(176)
#define MA_CHANNEL_VIBRATO				(176)
#define MA_CHANNEL_BEND					(192)
#define MA_FM_EXTWAVE_15				(320)
#define MA_FM_EXTWAVE_23				(322)
#define MA_FM_EXTWAVE_31				(324)
#define MA_LED_CTRL						(326)
#define MA_MOTOR_CTRL					(327)
#define MA_NOP_2						(328)
#define MA_WT_PG						(330)
#define MA_TIMER_MS						(338)
#define MA_TIMER_0_COUNT				(339)
#define MA_TIMER_0_TIME					(340)
#define MA_TIMER_0_CTRL					(341)
#define MA_TIMER_1_COUNT				(342)
#define MA_TIMER_1_TIME					(343)
#define MA_TIMER_1_CTRL					(344)
#define MA_SOFTINT_CTRL					(345)
#define MA_SOFTINT_RAM					(346)
#define MA_SEQUENCE						(350)
#define MA_INT_SETTING					(354)
#define MA_NOP_1						(358)

#define MA_MAX_CHANNEL					(16)
#define MA_MAX_FM_SLOT					(32)
#define MA_MAX_WT_SLOT					(8)

#define MA_MAX_RAM_ADDRESS				(0x6000)

#define MA_MAX_REG_STREAM_AUDIO			(64)

#define MA_MAX_STREAM_AUDIO				(2)
#define MA_MAX_PG						(1024 - 32)
#define MA_MIN_PG						(0)
#define MA_WAVE_SIZE					(1024 - 32)

#define MA_MIN_ROM_DRUM					(24)
#define MA_MAX_ROM_DRUM					(84)

#define MA_EVENT_INT					(2)
#define MA_SEQEVENT_INT					(3)

#define MA_KEYCTRL_NONE					(0)
#define MA_KEYCTRL_OFF					(1)
#define MA_KEYCTRL_ON					(2)

#define MA_MODE_MONO					(0)
#define MA_MODE_POLY					(1)

#define MA_MAX_ROM_WAVE					(7)

#define MA_RAM_BLOCK_SIZE				(1024)

#define MA_MIN_LENGTH					(20)

#define MA_DIRECT_FIFOSIZE				(64)

#define MA_RESET_RETRYCOUNT				(10)

/* */

#define MASMW_SUCCESS					(0)		/* success 								*/
#define MASMW_ERROR						(-1)	/* error								*/
#define MASMW_ERROR_ARGUMENT			(-2)	/* error of arguments					*/
#define MASMW_ERROR_RESOURCE_OVER		(-3)	/* over specified resources				*/
#define MASMW_ERROR_ID					(-4)	/* error id number 						*/

#define MASMW_ERROR_FILE				(-16)	/* file error							*/
#define MASMW_ERROR_CONTENTS_CLASS		(-17)	/* SMAF Contents Class shows can't play */
#define MASMW_ERROR_CONTENTS_TYPE		(-18)	/* SMAF Contents Type shows can't play	*/
#define MASMW_ERROR_CHUNK_SIZE			(-19)	/* illegal SMAF Chunk Size value		*/
#define MASMW_ERROR_CHUNK				(-20)	/* illegal SMAF Trachk Chunk value		*/
#define MASMW_ERROR_UNMATCHED_TAG		(-21)	/* unmathced specified TAG 				*/
#define MASMW_ERROR_SHORT_LENGTH		(-22)	/* short sequence 						*/
#define MASMW_ERROR_LONG_LENGTH			(-23)	/* long sequence 						*/

#define MASMW_ERROR_SMF_FORMAT			(-50)	/* invalid format type != 0/1			*/
#define MASMW_ERROR_SMF_TRACKNUM		(-51)	/* invalid number of tracks				*/
#define MASMW_ERROR_SMF_TIMEUNIT		(-52)	/* invalid time unit					*/
#define MASMW_ERROR_SMF_CMD				(-53)	/* invalid command byte					*/

#define MASMW_NUM_SRMCNV				(12)
#define MASMW_NUM_FILE					(5+1)
#define MASMW_NUM_SEQTYPE				(3)		/* number of sequence type */

#define MASMW_STATE_IDLE				(0)		/* idle state */
#define MASMW_STATE_LOADED				(1)		/* loaded state */
#define MASMW_STATE_OPENED				(2)		/* opened state */
#define MASMW_STATE_READY				(3)		/* ready state */
#define MASMW_STATE_PLAYING				(4)		/* playing state */
#define	MASMW_STATE_PAUSE				(5)		/* pause state */

#define MASMW_REPEAT					(126)
#define MASMW_END_OF_SEQUENCE			(127)
#define MASMW_END_OF_DATA				(128)

/* Sound Sequencer: Control */
#define MASMW_SET_VOLUME				(0)		/* set volume */
#define MASMW_SET_SPEED					(1)		/* set speed */
#define MASMW_SET_KEYCONTROL			(2)		/* set key control */
#define MASMW_GET_TIMEERROR				(3)		/* get time error */
#define MASMW_GET_POSITION				(4)		/* get position */
#define MASMW_GET_LENGTH				(5)		/* get length */
#define MASMW_GET_STATE					(6)		/* get state */
#define MASMW_SEND_MIDIMSG				(7)		/* send midi message */
#define MASMW_SEND_SYSEXMIDIMSG			(8)		/* send sys.ex. midi message */
#define MASMW_SET_BIND					(9)		/* set bind */
#define MASMW_GET_CONTENTSDATA			(10)	/* get contents data */
#define MASMW_GET_PHRASELIST			(11)	/* get phrase list */
#define MASMW_SET_STARTPOINT			(12)	/* set start point */
#define MASMW_SET_ENDPOINT				(13)	/* set end point */
#define MASMW_SET_PANPOT				(14)	/* set panpot */
#define MASMW_GET_LEDSTATUS				(15)	/* get LED status */
#define MASMW_GET_VIBSTATUS				(16)	/* get VIB status */
#define MASMW_SET_EVENTNOTE				(17)	/* set event note */
#define MASMW_GET_CONVERTTIME			(18)	/* get convert time */
#define MASMW_GET_LOADINFO				(19)	/* get load information */
#define MASMW_SET_LOADINFO				(20)	/* set load information */
#define MASMW_SET_SPEEDWIDE				(26)	/* set speed */
#define MASMW_SET_REPEAT				(27)	/* set number of repeat */
#define MASMW_GET_CONTROL_VAL			(29)	/* Get control value  */
#define MASMW_SET_CB_INTERVAL			(30)	/* Set calback function interval */

/* Sound Sequencer: Device Control */

#define MASMW_PWM_DIGITAL				(0)		/* power management (digital) */
#define MASMW_PWM_ANALOG				(1)		/* power management (analog) */
#define MASMW_EQ_VOLUME					(2)		/* eq volume */
#define MASMW_HP_VOLUME					(3)		/* hp volume */
#define MASMW_SP_VOLUME					(4)		/* sp volume */
#define MASMW_LED_MASTER				(5)		/* LED master select */
#define MASMW_LED_BLINK					(6)		/* LED blink setting */
#define MASMW_LED_DIRECT				(7)		/* LED direct control */
#define MASMW_MTR_MASTER				(8)		/* MOTOR master select */
#define MASMW_MTR_BLINK					(9)		/* MOTOR blink setting */
#define MASMW_MTR_DIRECT				(10)	/* MOTOR direct control */
#define MASMW_GET_PLLOUT				(11)	/* get PLL out */
#define MASMW_GET_SEQSTATUS				(12)	/* get status of HW sequencer */

#define MASMW_MAX_LED_MASTER			(2)
#define MASMW_MAX_LED_BLINK				(5)
#define MASMW_MAX_LED_DIRECT			(1)

#define MASMW_MAX_MTR_MASTER			(2)
#define MASMW_MAX_MTR_BLINK				(5)
#define MASMW_MAX_MTR_DIRECT			(1)

#define MADEVDRV_DCTRL_LED				(5)
#define MADEVDRV_DCTRL_MTR				(6)

#define MADEVDRV_GET_SEQSTATUS			(11)

#endif /*__MADEFS_H__*/
