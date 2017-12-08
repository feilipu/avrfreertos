/*
 * wave.h
 *
 *  Created on: 07/02/2015
 *      Author: phillip
 */

#ifndef WAVE_H_
#define WAVE_H_
/*
 * Includes
 */
#include <stdint.h>

#include <avr/io.h>

#include "ff.h"

#ifdef __cplusplus
extern "C" {
#endif

/* The mmioFOURCC macro converts four characters into a four-character code.
 * This macro does not check whether the four-character code it returns is valid.

	A FOURCC is represented as a sequence of one to four
	ASCII alphanumeric characters, padded on the right with
	blank characters (ASCII character value 32) as
	required, with no embedded blanks.uint32_t, in RIFF
	chunkRIFF files;FOURCC code inBYTE, in RIFF
	chunkFOURCCdatatype;inRIFFchunk

	For example, the four-character code FOO is stored as
	a sequence of four bytes: 'F', 'O', 'O', ' ' in
	ascending addresses. For quick comparisons, a four-
	character code may also be treated as a 32-bit number.
 *
 */
#define mmioFOURCC(ch0, ch1, ch2, ch3) \
    ((uint32_t)(uint8_t)(ch0) | ((uint32_t)(uint8_t)(ch1) << 8) |  \
    ((uint32_t)(uint8_t)(ch2) << 16) | ((uint32_t)(uint8_t)(ch3) << 24 ));

/* fixme try this one written to optimise for an 8 bit machine */
/* #define mmioFOURCC(ch0, ch1, ch2, ch3) \
	( (uint32_t)(((uint16_t)(uint8_t)(ch3) << 8) | (uint8_t)(ch2)) << 16 ) |  \
	  (uint32_t)(((uint16_t)(uint8_t)(ch1) << 8) | (uint8_t)(ch0)) */


#define RIFFINFO_RIFF      mmioFOURCC ('R', 'I', 'F', 'F')
#define RIFFTYPE_WAVE      mmioFOURCC ('W', 'A', 'V', 'E')
#define RIFFWAVE_fmt       mmioFOURCC ('f', 'm', 't', ' ')
#define RIFFWAVE_slnt      mmioFOURCC ('s', 'l', 'n', 't')
#define RIFFWAVE_data      mmioFOURCC ('d', 'a', 't', 'a')

/*////////////////////////////////////////////////////////////////////////// */

/*              INFO LIST CHUNKS (from the Multimedia Programmer's Reference
                                        plus new ones)
*/

#define RIFFINFO_IARL      mmioFOURCC ('I', 'A', 'R', 'L')     /*Archival location  */
#define RIFFINFO_IART      mmioFOURCC ('I', 'A', 'R', 'T')     /*Artist  */
#define RIFFINFO_ICMS      mmioFOURCC ('I', 'C', 'M', 'S')     /*Commissioned  */
#define RIFFINFO_ICMT      mmioFOURCC ('I', 'C', 'M', 'T')     /*Comments  */
#define RIFFINFO_ICOP      mmioFOURCC ('I', 'C', 'O', 'P')     /*Copyright  */
#define RIFFINFO_ICRD      mmioFOURCC ('I', 'C', 'R', 'D')     /*Creation date of subject  */
#define RIFFINFO_ICRP      mmioFOURCC ('I', 'C', 'R', 'P')     /*Cropped  */
#define RIFFINFO_IDIM      mmioFOURCC ('I', 'D', 'I', 'M')     /*Dimensions  */
#define RIFFINFO_IDPI      mmioFOURCC ('I', 'D', 'P', 'I')     /*Dots per inch  */
#define RIFFINFO_IENG      mmioFOURCC ('I', 'E', 'N', 'G')     /*Engineer  */
#define RIFFINFO_IGNR      mmioFOURCC ('I', 'G', 'N', 'R')     /*Genre  */
#define RIFFINFO_IKEY      mmioFOURCC ('I', 'K', 'E', 'Y')     /*Keywords  */
#define RIFFINFO_ILGT      mmioFOURCC ('I', 'L', 'G', 'T')     /*Lightness settings  */
#define RIFFINFO_IMED      mmioFOURCC ('I', 'M', 'E', 'D')     /*Medium  */
#define RIFFINFO_INAM      mmioFOURCC ('I', 'N', 'A', 'M')     /*Name of subject  */
#define RIFFINFO_IPLT      mmioFOURCC ('I', 'P', 'L', 'T')     /*Palette Settings. No. of colours requested.   */
#define RIFFINFO_IPRD      mmioFOURCC ('I', 'P', 'R', 'D')     /*Product  */
#define RIFFINFO_ISBJ      mmioFOURCC ('I', 'S', 'B', 'J')     /*Subject description  */
#define RIFFINFO_ISFT      mmioFOURCC ('I', 'S', 'F', 'T')     /*Software. Name of package used to create file.  */
#define RIFFINFO_ISHP      mmioFOURCC ('I', 'S', 'H', 'P')     /*Sharpness.  */
#define RIFFINFO_ISRC      mmioFOURCC ('I', 'S', 'R', 'C')     /*Source.   */
#define RIFFINFO_ISRF      mmioFOURCC ('I', 'S', 'R', 'F')     /*Source Form. ie slide, paper  */
#define RIFFINFO_ITCH      mmioFOURCC ('I', 'T', 'C', 'H')     /*Technician who digitized the subject.  */


/* New INFO Chunks as of August 30, 1993: */
#define RIFFINFO_ISMP      mmioFOURCC ('I', 'S', 'M', 'P')     /*SMPTE time code  */
/* ISMP: SMPTE time code of digitization start point expressed as a NULL terminated
                text string "HH:MM:SS:FF". If performing MCI capture in AVICAP, this
                chunk will be automatically set based on the MCI start time.
*/
#define RIFFINFO_IDIT      mmioFOURCC ('I', 'D', 'I', 'T')     /*Digitization Time  */
/* IDIT: "Digitization Time" Specifies the time and date that the digitization commenced.
                The digitization time is contained in an ASCII string which
                contains exactly 26 characters and is in the format
                "Wed Jan 02 02:03:55 1990\n\0".
                The ctime(), asctime(), functions can be used to create strings
                in this format. This chunk is automatically added to the capture
                file based on the current system time at the moment capture is initiated.
*/

/*Template line for new additions*/
/*#define RIFFINFO_I      mmioFOURCC ('I', '', '', '')        */

/*/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////*/


/* WAVE form wFormatTag IDs */


#define  WAVE_FORMAT_UNKNOWN    0x0000  /*  Microsoft Corporation  */
#define  WAVE_FORMAT_PCM		0x0001  /*	Microsoft Corporation - PCM */
#define  WAVE_FORMAT_ADPCM      0x0002  /*  Microsoft Corporation  */
#define  WAVE_FORMAT_IEEE_FLOAT 0x0003  /*  Microsoft Corporation  */
                                        /*  IEEE754: range (+1, -1]  */
                                        /*  32-bit/64-bit format as defined by */
                                        /*  MSVC++ float/double type */
#define  WAVE_FORMAT_IBM_CVSD   0x0005  /*  IBM Corporation  */
#define  WAVE_FORMAT_ALAW       0x0006  /*  Microsoft Corporation  */
#define  WAVE_FORMAT_MULAW      0x0007  /*  Microsoft Corporation  */
#define  WAVE_FORMAT_OKI_ADPCM  0x0010  /*  OKI  */
#define  WAVE_FORMAT_DVI_ADPCM  0x0011  /*  Intel Corporation  */
#define  WAVE_FORMAT_IMA_ADPCM  (WAVE_FORMAT_DVI_ADPCM) /*  Intel Corporation  */
#define  WAVE_FORMAT_MEDIASPACE_ADPCM   0x0012  /*  Videologic  */
#define  WAVE_FORMAT_SIERRA_ADPCM       0x0013  /*  Sierra Semiconductor Corp  */
#define  WAVE_FORMAT_G723_ADPCM 0x0014  /*  Antex Electronics Corporation  */
#define  WAVE_FORMAT_DIGISTD    0x0015  /*  DSP Solutions, Inc.  */
#define  WAVE_FORMAT_DIGIFIX    0x0016  /*  DSP Solutions, Inc.  */
#define  WAVE_FORMAT_DIALOGIC_OKI_ADPCM 0x0017  /*  Dialogic Corporation  */
#define  WAVE_FORMAT_MEDIAVISION_ADPCM  0x0018  /*  Media Vision, Inc. */
#define  WAVE_FORMAT_YAMAHA_ADPCM       0x0020  /*  Yamaha Corporation of America  */
#define  WAVE_FORMAT_SONARC     0x0021  /*  Speech Compression  */
#define  WAVE_FORMAT_DSPGROUP_TRUESPEECH        0x0022  /*  DSP Group, Inc  */
#define  WAVE_FORMAT_ECHOSC1    0x0023  /*  Echo Speech Corporation  */
#define  WAVE_FORMAT_AUDIOFILE_AF36     0x0024  /*    */
#define  WAVE_FORMAT_APTX       0x0025  /*  Audio Processing Technology  */
#define  WAVE_FORMAT_AUDIOFILE_AF10     0x0026  /*    */
#define  WAVE_FORMAT_DOLBY_AC2  0x0030  /*  Dolby Laboratories  */
#define  WAVE_FORMAT_GSM610     0x0031  /*  Microsoft Corporation  */
#define  WAVE_FORMAT_MSNAUDIO   0x0032  /*  Microsoft Corporation  */
#define  WAVE_FORMAT_ANTEX_ADPCME       0x0033  /*  Antex Electronics Corporation  */
#define  WAVE_FORMAT_CONTROL_RES_VQLPC  0x0034  /*  Control Resources Limited  */
#define  WAVE_FORMAT_DIGIREAL   0x0035  /*  DSP Solutions, Inc.  */
#define  WAVE_FORMAT_DIGIADPCM  0x0036  /*  DSP Solutions, Inc.  */
#define  WAVE_FORMAT_CONTROL_RES_CR10   0x0037  /*  Control Resources Limited  */
#define  WAVE_FORMAT_NMS_VBXADPCM       0x0038  /*  Natural MicroSystems  */
#define  WAVE_FORMAT_CS_IMAADPCM 0x0039 /* Crystal Semiconductor IMA ADPCM */
#define  WAVE_FORMAT_ECHOSC3     0x003A /* Echo Speech Corporation */
#define  WAVE_FORMAT_ROCKWELL_ADPCM     0x003B  /* Rockwell International */
#define  WAVE_FORMAT_ROCKWELL_DIGITALK  0x003C  /* Rockwell International */
#define  WAVE_FORMAT_XEBEC      0x003D  /* Xebec Multimedia Solutions Limited */
#define  WAVE_FORMAT_G721_ADPCM 0x0040  /*  Antex Electronics Corporation  */
#define  WAVE_FORMAT_G728_CELP  0x0041  /*  Antex Electronics Corporation  */
#define  WAVE_FORMAT_MPEG       0x0050  /*  Microsoft Corporation  */
#define  WAVE_FORMAT_MPEGLAYER3 0x0055  /*  ISO/MPEG Layer3 Format Tag */
#define  WAVE_FORMAT_CIRRUS     0x0060  /*  Cirrus Logic  */
#define  WAVE_FORMAT_ESPCM      0x0061  /*  ESS Technology  */
#define  WAVE_FORMAT_VOXWARE    0x0062  /*  Voxware Inc  */
#define  WAVEFORMAT_CANOPUS_ATRAC       0x0063  /*  Canopus, co., Ltd.  */
#define  WAVE_FORMAT_G726_ADPCM 0x0064  /*  APICOM  */
#define  WAVE_FORMAT_G722_ADPCM 0x0065  /*  APICOM      */
#define  WAVE_FORMAT_DSAT       0x0066  /*  Microsoft Corporation  */
#define  WAVE_FORMAT_DSAT_DISPLAY       0x0067  /*  Microsoft Corporation  */
#define  WAVE_FORMAT_SOFTSOUND  0x0080  /*  Softsound, Ltd.      */
#define  WAVE_FORMAT_RHETOREX_ADPCM     0x0100  /*  Rhetorex Inc  */
#define  WAVE_FORMAT_CREATIVE_ADPCM     0x0200  /*  Creative Labs, Inc  */
#define  WAVE_FORMAT_CREATIVE_FASTSPEECH8       0x0202  /*  Creative Labs, Inc  */
#define  WAVE_FORMAT_CREATIVE_FASTSPEECH10      0x0203  /*  Creative Labs, Inc  */
#define  WAVE_FORMAT_QUARTERDECK 0x0220 /*  Quarterdeck Corporation  */
#define  WAVE_FORMAT_FM_TOWNS_SND       0x0300  /*  Fujitsu Corp.  */
#define  WAVE_FORMAT_BTV_DIGITAL        0x0400  /*  Brooktree Corporation  */
#define  WAVE_FORMAT_OLIGSM     0x1000  /*  Ing C. Olivetti & C., S.p.A.  */
#define  WAVE_FORMAT_OLIADPCM   0x1001  /*  Ing C. Olivetti & C., S.p.A.  */
#define  WAVE_FORMAT_OLICELP    0x1002  /*  Ing C. Olivetti & C., S.p.A.  */
#define  WAVE_FORMAT_OLISBC     0x1003  /*  Ing C. Olivetti & C., S.p.A.  */
#define  WAVE_FORMAT_OLIOPR     0x1004  /*  Ing C. Olivetti & C., S.p.A.  */
#define  WAVE_FORMAT_LH_CODEC   0x1100  /*  Lernout & Hauspie  */
#define  WAVE_FORMAT_NORRIS     0x1400  /*  Norris Communications, Inc.  */


#define WAVE_FORMAT_EXTENSIBLE	0xFFFE
//
//  the WAVE_FORMAT_DEVELOPMENT format tag can be used during the
//  development phase of a new wave format.  Before shipping, you MUST
//  acquire an official format tag from Microsoft.
//
#define WAVE_FORMAT_DEVELOPMENT 0xFFFF


/*****************************************************************/

typedef struct riff_tag {
	uint8_t     ckID[4];           /* Chunk type identifier Chunk ID: "RIFF" */
    uint32_t    ckSize;            /* Chunk size field (size of ckData) */
    uint8_t  	WAVEID[4];		   /* WAVE ID: "WAVE" */
} WAVERIFF;  // riff chunk

typedef WAVERIFF       *PWAVERIFF;


/* general waveform format structure (information common to all formats) - obsolete */
typedef struct waveformat_tag {
	uint8_t     ckID[4];           /* Chunk type identifier Chunk ID: "fmt " */
    uint32_t    ckSize;            /* Chunk size field (size of ckData) Chunk size: 16 */
	uint16_t	wFormatTag;        /* format type WAVE_FORMAT_PCM */
	uint16_t	nChannels;         /* number of channels (i.e. mono, stereo...) */
	uint32_t	nSamplesPerSec;    /* sample rate */
	uint32_t	nAvgBytesPerSec;   /* for buffer estimation */
	uint16_t	nBlockAlign;       /* block size of data */
} WAVEFORMAT;

typedef WAVEFORMAT       *PWAVEFORMAT;


/* specific waveform format structure for PCM data - obsolete */
typedef struct pcmwaveformat_tag {
	uint8_t     ckID[4];           /* Chunk type identifier */
    uint32_t    ckSize;            /* Chunk size field (size of ckData) */
	uint16_t	wFormatTag;        /* format type */
	uint16_t	nChannels;         /* number of channels (i.e. mono, stereo...) */
	uint32_t	nSamplesPerSec;    /* sample rate */
	uint32_t	nAvgBytesPerSec;   /* for buffer estimation */
	uint16_t	nBlockAlign;       /* block size of data */
    uint16_t	wBitsPerSample;	   /* Number of bits per sample of mono data */
} PCMWAVEFORMAT;

typedef PCMWAVEFORMAT       *PPCMWAVEFORMAT;

/* extensible waveform format structure (information common to all formats) */
typedef struct waveformatex_tag {
	uint8_t     ckID[4];           /* Chunk type identifier Chunk ID: "fmt " */
    uint32_t    ckSize;            /* Chunk size field (size of ckData) Chunk size: 16 */
	uint16_t	wFormatTag;        /* format type WAVE_FORMAT_PCM */
	uint16_t	nChannels;         /* number of channels (i.e. mono, stereo...) */
	uint32_t	nSamplesPerSec;    /* sample rate */
	uint32_t	nAvgBytesPerSec;   /* for buffer estimation */
	uint16_t	nBlockAlign;       /* block size of data */
	uint16_t	wBitsPerSample;    /* Number of bits per sample of mono data */
	uint16_t	cbSize;            /* The count in bytes of the size of
									  extra information (after cbSize) */
} WAVEFORMATEX;

typedef WAVEFORMATEX       *PWAVEFORMATEX;


typedef struct waveslnt_tag {
	uint8_t     ckID[4];           /* Chunk type identifier Chunk ID: "slnt" */
    uint32_t    ckSize;            /* Chunk size field (size of ckData) */
    uint32_t 	nSilentSamples;	   /* Number of silent samples */
    /* data follows here  - must be padded to even number of bytes */
} WAVESLNT;

typedef WAVESLNT	      *PWAVESLNT;


/*	The Data chunk contains the sampled data.

	One point about sample data that may cause some confusion is that when samples are
	represented with 8-bits, they are specified as unsigned values.
	All other sample bit-sizes are specified as signed values.
	For example a 16-bit sample can range from -32,768 to +32,767 with a mid-point (silence) at 0.
	All RIFF chunks (including WAVE "data" chunks) must be word aligned.
	If the sample data uses an odd number of bytes, a padding byte with a value of zero
	must be placed at the end of the sample data.
	The "data" chunk header's size should not include this byte.
*/

typedef struct wavedata_tag {
	uint8_t     ckID[4];           /* Chunk type identifier Chunk ID: "data" */
    uint32_t    ckSize;            /* Chunk size field (size of ckData) */
    /* data follows here  - must be padded to even number of bytes */
} WAVEDATA;

typedef WAVEDATA	      *PWAVEDATA;



typedef struct player_control {
			/** FatFS File object structure (FIL) instance for current wave file. */
	FIL *fd;
			/** Pointer to the working ring buffer */
	ringBuffer_t* play_buffer;
			/** Wave file number of channels. Mono = 1, Stereo = 2 */
	uint8_t nChannels;
			/** Wave file sample rate. Must be not greater than 44100/sec. */
	uint32_t nSamplesPerSec;
			/** Wave file bits per sample.  Must be 8 or 16. */
	uint8_t wBitsPerSample;
			/** Remaining bytes to be played in Wave file data chunk. */
	uint32_t remainingBytesInChunk;
			/** Has the value true if a wave file is playing else false. */
	volatile uint8_t isplaying;
			/** Number of times data was not available from the SD in the DAC ISR */
	uint32_t IOerrors;
} WAVEPLAYER_CONTROL;

typedef WAVEPLAYER_CONTROL	      *PWAVEPLAYER_CONTROL;

uint8_t waveplayer_create(FIL f);
void waveplayer_setSampleRate(uint32_t samplerate);

void waveplayer_play(void);
void waveplayer_stop(void);

uint8_t waveplayer_isPaused(void);

void waveplayer_pause(void);
void waveplayer_resume(void);

void waveplayer_seek(uint32_t pos);
int16_t waveplayer_readWaveData(uint8_t *buff, uint16_t len);



#ifdef __cplusplus
}
#endif


#endif /* WAVE_H_ */
