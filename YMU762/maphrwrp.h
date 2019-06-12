/*==============================================================================
//	Copyright (C) YAMAHA Corporation 2001, All rights reserved.
//
//	Title		: MAPHRWRP.H
//
//	Description	: header of maphrwrp.h
//
//	Version		: 1.4.7.0
//
//	History		:
//				  Jun  11, 2001		1st try.
//				  July 13, 2001		Add Phrase_SetPanpot/Phrase_GetPanpot/Phrase_Terminate functions.
//				  July 23, 2001		Add Phrase_SetEvHandler function.
//				  Aug  01, 2001		Add Phrase_Pause/Phrase_Restart functions.
//									Change specification of Phrase_GetStatus function.
//				  Sep  06, 2001		Add Phrase_GetPosition/Phrase_Seek.
//				  Sep  27, 2001		Fix bug in PutBindSetting().
//				  Oct  01, 2001		Modify Phrase_SetLink().
//				  Oct  10, 2001		Modify Phrase_SetLink().
//				  Oct  15, 2001		Modify Phrase_SetLink()
//									Remove PutBindSetting().
//				  Oct  22, 2002		Add Phrase_GetLength().
//============================================================================*/
#ifndef	_MAPHRWRP_H_
#define	_MAPHRWRP_H_

#ifndef	UINT8
typedef	unsigned char	UINT8;
#endif
#ifndef	UINT16
typedef	unsigned short	UINT16;
#endif

#ifndef	UINT32
typedef	unsigned long	UINT32;
#endif

#ifndef	SINT8
typedef	signed char		SINT8;
#endif
#ifndef	SINT16
typedef	signed short	SINT16;
#endif
#ifndef	SINT32
typedef	signed long		SINT32;
#endif

#ifndef MAX_PHRASE_CHANNEL
#define	MAX_PHRASE_CHANNEL	(4)
#endif	/*	!MAX_PHRASE_CHANNEL	*/

#define	BIT_STATUS_SLAVE	(0x10)

typedef enum	_tagIdStatus {
	ID_STATUS_UNKNOWN	= 0,
	ID_STATUS_NODATA,
	ID_STATUS_READY,
	ID_STATUS_PLAY,
	ID_STATUS_ENDING,
	ID_STATUS_PAUSE,
	ID_STATUS_LINKSLAVE
} IDSTATUS;

struct	event {
	int	ch;
	int	mode;
};

#if 1
struct info {
	long	MakerID;
	int		DeviceID;
	int		VersionID;
	int		MaxVoice;
	int		MaxChannel;
	int		SupportSMAF;
	long	Latency;
};
#endif

int		Phrase_Initialize	(void);
int		Phrase_Terminate	(void);
int		Phrase_GetInfo		(struct info* dat);
int		Phrase_CheckData	(unsigned char* data, unsigned long len);
int		Phrase_SetData		(int ch, unsigned char* data, unsigned long len, int check);
int		Phrase_Seek			(int ch, long pos);
int		Phrase_Play			(int ch, int loop);
int		Phrase_Stop			(int ch);
int		Phrase_Pause		(int ch);
int		Phrase_Restart		(int ch);
int		Phrase_Kill			(void);
void	Phrase_SetVolume	(int ch, int vol);
int		Phrase_GetVolume	(int ch);
void	Phrase_SetPanpot	(int ch, int vol);
int		Phrase_GetPanpot	(int ch);
int		Phrase_GetStatus	(int ch);
long	Phrase_GetPosition	(int ch);
long	Phrase_GetLength	(int ch);
int		Phrase_RemoveData	(int ch);
int		Phrase_SetEvHandler(void (* func)(struct event*));
int		Phrase_SetLink		(int ch, unsigned long slave);
unsigned long	Phrase_GetLink(int ch);


#endif	/*	!_MAPHRWRP_H_	*/
