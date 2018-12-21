/*
 * Copyright (C) 2013, Broadcom Corporation. All Rights Reserved.
 * 
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION
 * OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
 * CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */
/*
 * ALSA I2S Interface for the Broadcom BCM535X family of I2S cores
 */
 
#ifndef _BCM535X_I2S_H
#define _BCM535X_I2S_H
 
 // Debug support

#define BCM535X_I2S_DEBUG 1
#if BCM535X_I2S_DEBUG
#define DBG(x...) printk(KERN_ERR x)
#else
#define DBG(x...)
#endif

#ifndef	PAD
#define	_PADLINE(line)	pad ## line
#define	_XSTR(line)	_PADLINE(line)
#define	PAD		_XSTR(__LINE__)
#endif

#define	I2S_NUM_DMA		2

#define BCM5301X_I2S_INTERRUPT		120

// I2S Configuration defines (here for now)
#define I2S_MASTER 1

#define ENABLE_TX 1
#define ENABLE_RX 1

#define RX_LOOPBACK 1
//#define TX_TEST_SINEWAVE 1

// The NS SVK has two codecs, WM8750 and WM8955. WM8955 is a DAC and supports output only. WM8750 supports both output and input. The SPI and IS2 signals from the iProc
// are muxed to only one of the two codecs, dependent upon the strapping options on the board.
// The selection below needs to match the strapping on the SVK board.
#define CODEC_WM8750 1						
//#define CODEC_WM8955 1

/* dma */
typedef volatile struct {
	dma64regs_t	dmaxmt;		/* dma tx */
	uint32 PAD[2];
	dma64regs_t	dmarcv;		/* dma rx */
	uint32 PAD[2];
} dma64_t;

// Rates
#define BCM535X_I2S_RATES \
        (SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_11025 | \
        SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_32000 | \
        SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_96000)

// Formats
#define BCM535X_I2S_FORMATS \
        (SNDRV_PCM_FMTBIT_S8  | SNDRV_PCM_FMTBIT_U8 | \
         SNDRV_PCM_FMTBIT_S16 | SNDRV_PCM_FMTBIT_U16 | \
         SNDRV_PCM_FMTBIT_S24 | SNDRV_PCM_FMTBIT_U24 | \
         SNDRV_PCM_FMTBIT_S32 | SNDRV_PCM_FMTBIT_U32)
		 

// I2S Register Bitmasks

// Device Control Register
#define I2S_DC_TDM_SEL			(0x00000010)		// TDM select
#define I2S_DC_BCLKD_IN			(0x00000020)		// Bitclock direction
#define I2S_DC_I2SCFG			(0x00000040)		// Core SRAM for half duplex
#define I2S_DC_OPCHSEL			(0x00000080)		// Select Operation Channel
#define I2S_DC_WL_TX8			(0x00004000)		// TX 8-bit word length
#define I2S_DC_WL_RX8			(0x00008000)		// RX 8-bit word length

#define I2S_DC_WL_TXRX_MASK		(0x0000FC00)		// TX/RX word length mask
#define I2S_DC_DPX_SHIFT		(8)
#define I2S_DC_DPX_MASK			(0x3<<I2S_DC_DPX_SHIFT)	// Duplex mask

// Device Status Register
#define I2S_DS_BUSY				(0x00000010)		// BUSY
#define I2S_DS_TNF_CHO			(0x00000100)		// TX FIFO not full
#define I2S_DS_TXUNDER_CHO		(0x00000200)		// TX FIFO underflow channel 0
#define I2S_DS_TXOVER_CHO		(0x00000400)		// TX FIFO overflow channel 0
#define I2S_DS_PLAYING			(0x00000800)		// I2S PLAYING
#define I2S_DS_RNE_CH0			(0x00001000)		// RX FIFO not empty
#define I2S_DS_RXUNDER_CH0		(0x00002000)		// RX FIFO underflow bit channel 0
#define I2S_DS_RXOVER_CH0		(0x00004000)		// RX FIFO overflow bit channel 0
#define I2S_DS_RECORDING		(0x00008000)		// I2S RECORDING

// I2S Control Register
#define I2S_CONTROL_PLAYEN		(0x00000001)		// Play enable
#define I2S_CONTROL_RECEN		(0x00000002)		// Record enable
#define I2S_CONTROL_CLSLM		(0x00000008)		// Data puts close to LSB/MSB
#define I2S_CONTROL_TXLRCHSEL	(0x00000010)		// Select transmit Left/right channel first
#define I2S_CONTROL_RXLRCHSEL	(0x00000020)		// Select receive Left/right channel first
#define I2S_CONTROL_MUTE		(0x00000010)		// Transmit mute


// SPDIF Transmitter Control Register
#define I2S_STXC_WL_MASK		(0x00000003)		// Word length mask
#define I2S_STXC_STX_EN			(0x00000004)		// Enable/disable TX path
#define	I2S_STXC_SPDIF_CHCODE	(0x00000008)		// Select Preamble cell-order

// CLKDIV Register
#define I2S_CLKDIV_SRATE_SHIFT		(0)
#define I2S_CLKDIV_SRATE_MASK		(0xF<<I2S_CLKDIV_SRATE_SHIFT)

/* IntStatus Register               */
/* IntMask Register                 */
#define I2S_INT_GPTIMERINT		(1<<7)	/* General Purpose Timer Int/IntEn */
#define I2S_INT_DESCERR			(1<<10)	/* Descriptor Read Error/ErrorEn */
#define I2S_INT_DATAERR			(1<<11)	/* Descriptor Data Transfer Error/ErrorEm */
#define I2S_INT_DESC_PROTO_ERR		(1<<12)	/* Descriptor Protocol Error/ErrorEn */
#define I2S_INT_RCVFIFO_OFLOW		(1<<14)	/* Receive FIFO Overflow */
#define I2S_INT_XMTFIFO_UFLOW		(1<<15)	/* Transmit FIFO Overflow */
#define I2S_INT_RCV_INT			(1<<16)	/* Receive Interrupt */
#define I2S_INT_XMT_INT			(1<<24)	/* Transmit Interrupt */
#define I2S_INT_RXSIGDET		(1<<26)	/* Receive signal toggle */
#define I2S_INT_SPDIF_PAR_ERR		(1<<27)	/* SPDIF Rx parity error */
#define I2S_INT_VALIDITY		(1<<28)	/* SPDIF Rx Validity bit interrupt */
#define I2S_INT_CHSTATUS		(1<<29)	/* SPDIF Rx channel status interrupt */
#define I2S_INT_LATE_PREAMBLE		(1<<30)	/* SPDIF Rx preamble not detected */
#define I2S_INT_SPDIF_VALID_DATA	(1<<31)	/* SPDIF Rx Valid data */

/*  I2S FIFOCounter Register             */
/* TX Fifo data counter in 4-byte units */
#define I2S_FC_TX_CNT_SHIFT		(0)
#define I2S_FC_TX_CNT_MASK		(0xFF<<I2S_FC_TX_CNT_SHIFT)
/* RX Fifo data counter in 4-byte units */
#define I2S_FC_RX_CNT_SHIFT		(8)
#define I2S_FC_RX_CNT_MASK		(0xFF<<I2S_FC_RX_CNT_SHIFT)

/* I2S DMA Registers */
#define I2S_DMA0_TX_BASE			0x1802a200	
#define I2S_DMA0_RX_BASE			0x1802a220	

#define AUDIO_DEST_HDMI				(2)
#define AUDIO_DEST_LOCAL			(0)

// IDM Register defines
#define ROM_S0_IDM_IO_STATUS			0x1810D500
#define I2S_M0_IDM_IO_CONTROL_DIRECT 	0x18117408
#define I2S_M0_IDM_RESET_CONTROL 		0x18117800
#define I2S_M0_IDM_RESET_STATUS 		0x18117804

// IDM Register Masks
#define I2S_IDM_CD_CLK_ENABLE				0x1
#define I2S_IDM_CD_CLK_DIV					0x40

// intReceLazy Register
#define I2S_INTRECE_LAZY_FC				(1<<24)		// Interrupt when rx framecount is 1

typedef struct bcm535x_i2s_info bcm535x_i2s_info_t;

// Structures

/* I2S Host Interface Register Summary */
typedef volatile struct _i2sregs {
	uint32	devcontrol;	/* 0x000	R/W	Default 0x0220	*/
	uint32	devstatus;	/* 0x004	R-only	Default 0x1000	*/
	uint32	PAD[1];		/* 0x008 */
	uint32	biststatus;	/* 0x00C	R/W	Default 0x0	*/
	uint32	PAD[4];		/* 0x010 - 0x1C */
	uint32	intstatus;	/* 0x020	R/W	Default 0x0	*/
	uint32	intmask;	/* 0x024	R/W	Default 0x0	*/
	uint32	gptimer;	/* 0x028	R/W	Default 0x0	*/
	uint32	PAD[1];		/* 0x02C */
	uint32	i2scontrol;	/* 0x030	R/W	Default 0x0	*/
	uint32	clkdivider;	/* 0x034	R/W	Default 0x1	*/
	uint32	PAD[2];		/* 0x038 - 0x3C */
	uint32	txplayth;	/* 0x040	R/W	Default 0x018	*/
	uint32	fifocounter;	/* 0x044	R-Only	Default 0x0	*/
	uint32	PAD[2];		/* 0x048 - 0x4C */
	uint32	stxctrl;	/* 0x050	R/W	Default 0x0	*/
	uint32	srxctrl;	/* 0x054	R/W	Default 0x0	*/
	uint32	srxpcnt;	/* 0x058	R/W	Default 0x0	*/
	/* TDM interface registers for corerev >= 37 */
	uint32 PAD[5];     /* 0x5C ~ 0x6C */
	uint32 tdm_control; /* 0x70 */
	uint32 tdm_ch0_ctrl; /* 0x74 */
	uint32 tdm_ch1_ctrl; /* 0x78 */
	uint32 PAD[33];

	uint32	intrecvlazy;	/* 0x100	R/W	Default 0x0	*/
	uint32	PAD[36];	/* 0x104 - 0x190 */
	uint32	gpioselect;	/* 0x194	R/W	Default 0x0	*/
	uint32	gpioouten;	/* 0x198	R/W	Default 0x0	*/
	uint32	PAD[17];	/* 0x19C - 0x1DC */
	uint32	clk_ctl_st;	/* 0x1E0	R/W	Default 0x0	*/
	uint32	PAD[1];		/* 0x1E4 */
	uint32	powercontrol;	/* 0x1E8	R/W	Default 0x0	*/
	uint32	PAD[5];		/* 0x1EC - 0x1FC */
	dma64_t	dmaregs[I2S_NUM_DMA]; /* 0x200 - 0x23C */
	uint32	PAD[16];
	/* fifoaddr	have different location	for	I2S	core rev0 due to a single DMA */
	uint32	fifoaddr;	/* 0x280	R/W	Default 0x0	*/
	uint32	fifodata;	/* 0x284	R/W	Default 0x0	*/
} i2sregs_t;

struct bcm535x_i2s_info {
	/* ALSA structs. */
	struct snd_card *card;
	struct snd_pcm *pcm;
	struct snd_soc_dai *cpu_dai;
	struct snd_soc_dai *codec_dai;
	int			irq;
	osl_t		*osh;
	void		*regsva;			/* opaque chip registers virtual address */
	i2sregs_t	*regs;			/* pointer to device registers */
	hnddma_t	*di[1];		/* hnddma handles, per fifo */
	si_t		*sih;		/* SB handle (cookie for siutils calls) */
	uint32_t 	mclk;
};

struct _i2s_clkdiv_coeffs {
	uint32_t	mclk;		/* Hz */
	uint32_t	rate;		/* Hz */
	uint16_t	fs;
	uint8_t		srate;
};

/* divider info for SRATE */
static const struct _i2s_clkdiv_coeffs i2s_clkdiv_coeffs[] = {
	/* 11.2896MHz */
	{11289600,	22050,	512,	0x3},
	{11289600,	44100,	256,	0x1},
	{11289600,	88200,	128,	0x0},
	/* 12.288MHz */
	{12288000,	8000,	1536,	0x6}, /* Can we use 6 <-> 16 for 1536 fs ? */
	{12288000,	12000,	1024,	0x5},
	{12288000,	16000,	768,	0x4},
	{12288000,	24000,	512,	0x3},
	{12288000,	32000,	384,	0x2},
	{12288000,	48000,	256,	0x1},
	{12288000,	96000,	128,	0x0},
	/* 24.567MHz */
	{24567000,	96000,	256,	0x1},
	{24567000,	192000,	128,	0x0},
};

typedef enum {
    PCM_PLAYBACK_VOLUME,
    PCM_PLAYBACK_MUTE,
    PCM_PLAYBACK_DEVICE,
} SND_BCM535x_CTRL_T;

// Functions
int bcm535x_i2s_attach(struct platform_device *pdev);
int bcm535x_pcm_new(struct snd_card *card);
int snd_bcm535x_new_ctl(bcm535x_i2s_info_t *sndbcm_535x);

extern struct snd_soc_dai bcm535x_i2s_dai;
extern bcm535x_i2s_info_t *snd_bcm535x;
extern struct snd_soc_platform bcm535x_soc_platform;

extern int WM8955_set_fmt(unsigned int fmt);
extern int WM8955_hw_params(struct snd_pcm_hw_params *params);

extern int WM8750_pcm_hw_params(struct snd_pcm_hw_params *params);
extern int WM8750_set_dai_fmt(unsigned int fmt);
extern int WM8750_set_bias_level(int level);

#endif /* _BCM535X_I2S_H */
