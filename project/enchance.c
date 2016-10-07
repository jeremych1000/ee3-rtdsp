/*************************************************************************************
DEPARTMENT OF ELECTRICAL AND ELECTRONIC ENGINEERING
IMPERIAL COLLEGE LONDON

EE 3.19: Real Time Digital Signal Processing
Dr Paul Mitcheson and Daniel Harvey

PROJECT: Frame Processing

********* ENHANCE. C **********
Shell for speech enhancement

Demonstrates overlap-add frame processing (interrupt driven) on the DSK.

*************************************************************************************
By Danny Harvey: 21 July 2006
Updated for use on CCS v4 Sept 2010
************************************************************************************/
/*
*	You should modify the code so that a speech enhancement project is built
*	on top of this template.
*/
/**************************** Pre-processor statements ******************************/
//	library required when using calloc
#include <stdlib.h>
//	Included so program can make use of DSP/BIOS configuration tool.  
#include "dsp_bios_cfg.h"

/* The file dsk6713.h must be included in every program that uses the BSL.	This
example also includes dsk6713_aic23.h because it uses the
AIC23 codec module (audio interface). */
#include "dsk6713.h"
#include "dsk6713_aic23.h"

// math library (trig functions)
#include <math.h>
#include <float.h> 

/* Some functions to help with Complex algebra and FFT. */
#include "cmplx.h"		
#include "fft_functions.h"	

// Some functions to help with writing/reading the audio ports when using interrupts.
#include <helper_functions_ISR.h>



#define WINCONST 0.85185			/* 0.46/0.54 for Hamming window */
#define FSAMP 8000.0		/* sample frequency, ensure this matches Config for AIC */
#define FFTLEN 256					/* fft length = frame length 256/8000 = 32 ms*/
#define NFREQ (1+FFTLEN/2)			/* number of frequency bins from a real FFT */
#define OVERSAMP 4					/* oversampling ratio (2 or 4) */  
#define FRAMEINC (FFTLEN/OVERSAMP)	/* Frame increment */
#define CIRCBUF (FFTLEN+FRAMEINC)	/* length of I/O buffers */

#define OUTGAIN 16000.0				/* Output gain for DAC */
#define INGAIN	(1.0/16000.0)		/* Input gain for ADC  */
// PI defined here for use in your code 
#define PI 3.141592653589793
#define TFRAME FRAMEINC/FSAMP		/* time between calculation of each frame */
#define min_fram_rate 312


/////////////////////swithchable parameter
int alpha = 20;
float lamda = 0.01;
float tau = 0.02;//from 0.02 to 0.08
int in_filt = 1;
int out_filt = 1;
int gain_var = 1;//2,3,4 only work with in- filter on


				 /******************************* Global declarations ********************************/

				 /* Audio port configuration settings: these values set registers in the AIC23 audio
				 interface to configure it. See TI doc SLWS106D 3-3 to 3-10 for more info. */
DSK6713_AIC23_Config Config = { \
/**********************************************************************/
/*	  REGISTER				FUNCTION				  SETTINGS		   */
/**********************************************************************/\
0x0017,	 /* 0 LEFTINVOL	 Left line input channel volume	 0dB				   */\
0x0017,	 /* 1 RIGHTINVOL Right line input channel volume 0dB				   */\
0x01f9,	 /* 2 LEFTHPVOL	 Left channel headphone volume	 0dB				   */\
0x01f9,	 /* 3 RIGHTHPVOL Right channel headphone volume	 0dB				   */\
0x0011,	 /* 4 ANAPATH	 Analog audio path control		 DAC on, Mic boost 20dB*/\
0x0000,	 /* 5 DIGPATH	 Digital audio path control		 All Filters off	   */\
0x0000,	 /* 6 DPOWERDOWN Power down control				 All Hardware on	   */\
0x0043,	 /* 7 DIGIF		 Digital audio interface format	 16 bit				   */\
0x008d,	 /* 8 SAMPLERATE Sample rate control		8 KHZ-ensure matches FSAMP */\
0x0001	 /* 9 DIGACT	 Digital interface activation	 On					   */\
/**********************************************************************/
};

// Codec handle:- a variable used to identify audio interface  
DSK6713_AIC23_CodecHandle H_Codec;

float *inbuffer, *outbuffer;		/* Input/output circular buffers */
float *outframe;		  /* Input and output frames */
complex *inframe;
float *last_P;
float *last_PP;
float *P;
float *PP;
float *inwin, *outwin;				/* Input and output windows */
float *frame_mag;
float *N;
float ingain, outgain;				/* ADC and DAC gains */
float cpufrac;						/* Fraction of CPU time used */
volatile int io_ptr = 0;				/* Input/ouput pointer for circular buffers */
volatile int frame_ptr = 0;			/* Frame pointer */
volatile int frame_count = 0;
volatile int min_buffer_index = 0;
float frame_min_buffer[OVERSAMP*FFTLEN] = { FLT_MAX };
float noise_amp = FLT_MAX;
float K;



/******************************* Function prototypes *******************************/
void init_hardware(void);		/* Initialize codec */
void init_HWI(void);			/* Initialize hardware interrupts */
void ISR_AIC(void);				/* Interrupt service routine for codec */
void process_frame(void);		/* Frame processing routine */
float max(float a, float b);
void init_buffer(void);
float min(float a, float b);
void ProcVswitch(void);


/********************************** Main routine ************************************/
void main()
{

	int k; // used in various for loops

		   /*	Initialize and zero fill arrays */

	inbuffer = (float *)calloc(CIRCBUF, sizeof(float));	/* Input array */
	outbuffer = (float *)calloc(CIRCBUF, sizeof(float));	/* Output array */
	inframe = (complex *)calloc(FFTLEN, sizeof(complex));	/* Array for processing*/
	outframe = (float *)calloc(FFTLEN, sizeof(float));	/* Array for processing*/
	inwin = (float *)calloc(FFTLEN, sizeof(float));	/* Input window */
	outwin = (float *)calloc(FFTLEN, sizeof(float));	/* Output window */
	frame_mag = (float *)calloc(FFTLEN, sizeof(float));
	N = (float *)calloc(FFTLEN, sizeof(float));
	last_P = (float *)calloc(FFTLEN, sizeof(float));
	P = (float *)calloc(FFTLEN, sizeof(float));
	last_PP = (float *)calloc(FFTLEN, sizeof(float));
	PP = (float *)calloc(FFTLEN, sizeof(float));
	K = exp(-TFRAME / tau);


	/* initialize board and the audio port */
	init_hardware();
	init_buffer();
	/* initialize hardware interrupts */
	init_HWI();

	/* initialize algorithm constants */

	for (k = 0; k < FFTLEN; k++)
	{
		inwin[k] = sqrt((1.0 - WINCONST*cos(PI*(2 * k + 1) / FFTLEN)) / OVERSAMP);
		outwin[k] = inwin[k];
	}
	ingain = INGAIN;
	outgain = OUTGAIN;


	/* main loop, wait for interrupt */
	while (1)	process_frame();
}
/********************************** End of Main *********************************/


/********************************** init_hardware() *********************************/
void init_hardware()
{
	// Initialize the board support library, must be called first 
	DSK6713_init();

	// Start the AIC23 codec using the settings defined above in config 
	H_Codec = DSK6713_AIC23_openCodec(0, &Config);

	/* Function below sets the number of bits in word used by MSBSP (serial port) for
	receives from AIC23 (audio port). We are using a 32 bit packet containing two
	16 bit numbers hence 32BIT is set for  receive */
	MCBSP_FSETS(RCR1, RWDLEN1, 32BIT);

	/* Configures interrupt to activate on each consecutive available 32 bits
	from Audio port hence an interrupt is generated for each L & R sample pair */
	MCBSP_FSETS(SPCR1, RINTM, FRM);

	/* These commands do the same thing as above but applied to data transfers to the
	audio port */
	MCBSP_FSETS(XCR1, XWDLEN1, 32BIT);
	MCBSP_FSETS(SPCR1, XINTM, FRM);


}
/********************************** init_buffer() *********************************/
void init_buffer(void) {
	int i = 0;
	for (i = 0; i < FFTLEN*OVERSAMP; i++) {
		frame_min_buffer[i] = FLT_MAX;
	}
}
/********************************** max() *********************************/
float max(float a, float b) {
	if (a > b) {
		return a;
	}
	else {
		return b;
	}
}
/********************************** min() *********************************/
float min(float a, float b) {
	if (a > b) {
		return b;
	}
	else {
		return a;
	}
}
/********************************** init_HWI() **************************************/
void init_HWI(void)
{
	IRQ_globalDisable();			// Globally disables interrupts
	IRQ_nmiEnable();				// Enables the NMI interrupt (used by the debugger)
	IRQ_map(IRQ_EVT_RINT1, 4);		// Maps an event to a physical interrupt
	IRQ_enable(IRQ_EVT_RINT1);		// Enables the event
	IRQ_globalEnable();				// Globally enables interrupts

}

/******************************** process_frame() ***********************************/
void process_frame(void)
{
	int k, m;
	int io_ptr0;


	/* work out fraction of available CPU time used by algorithm */
	cpufrac = ((float)(io_ptr & (FRAMEINC - 1))) / FRAMEINC;

	/* wait until io_ptr is at the start of the current frame */
	while ((io_ptr / FRAMEINC) != frame_ptr);

	/* then increment the framecount (wrapping if required) */
	if (++frame_ptr >= (CIRCBUF / FRAMEINC)) frame_ptr = 0;

	/* save a pointer to the position in the I/O buffers (inbuffer/outbuffer) where the
	data should be read (inbuffer) and saved (outbuffer) for the purpose of processing */
	io_ptr0 = frame_ptr * FRAMEINC;

	/* copy input data from inbuffer into inframe (starting from the pointer position) */

	m = io_ptr0;
	for (k = 0; k < FFTLEN; k++)
	{
		inframe[k] = cmplx(inbuffer[m] * inwin[k], 0);
		if (++m >= CIRCBUF) m = 0; /* wrap if required */
	}

	/************************* DO PROCESSING OF FRAME  HERE **************************/
	ProcVswitch();

	/* please add your code, at the moment the code simply copies the input to the
	ouptut with no processing */


	for (k = 0; k < FFTLEN; k++)
	{
		outframe[k] = inframe[k].r;/* copy input straight into output */
	}

	/********************************************************************************/

	/* multiply outframe by output window and overlap-add into output buffer */

	m = io_ptr0;

	for (k = 0; k < (FFTLEN - FRAMEINC); k++)
	{											/* this loop adds into outbuffer */
		outbuffer[m] = outbuffer[m] + outframe[k] * outwin[k];
		if (++m >= CIRCBUF) m = 0; /* wrap if required */
	}
	for (; k < FFTLEN; k++)
	{
		outbuffer[m] = outframe[k] * outwin[k];	/* this loop over-writes outbuffer */
		m++;
	}
}

/********************************** end of process_frame() *********************************/



/*************************** INTERRUPT SERVICE ROUTINE	*****************************/

// Map this to the appropriate interrupt in the CDB file

void ISR_AIC(void)
{
	short sample;
	/* Read and write the ADC and DAC using inbuffer and outbuffer */

	sample = mono_read_16Bit();
	inbuffer[io_ptr] = ((float)sample)*ingain;
	/* write new output data */
	mono_write_16Bit((int)(outbuffer[io_ptr] * outgain));

	/* update io_ptr and check for buffer wraparound */

	if (++io_ptr >= CIRCBUF) io_ptr = 0;
}


/********************************** all Versions of  proces function  *********************************/
///vswitch is original setting with no filtering,G = max(lamda,1-|N|/|X|), 
void ProcVswitch(void) {
	int k;

	fft(FFTLEN, inframe);
	for (k = 0; k < FFTLEN; k++) {
		frame_mag[k] = cabs(inframe[k]);
		if (in_filt == 2) {
			P[k] = (1 - K)*frame_mag[k] * frame_mag[k] + K*last_P[k];
			last_P[k] = P[k];
		}
		else {
			if (in_filt == 1) {
				P[k] = (1 - K)*frame_mag[k] + K*last_P[k];
				last_P[k] = P[k];
			}
		}
	}
	if (--frame_count == -1) {
		frame_count = min_fram_rate;
		if (++min_buffer_index > OVERSAMP - 1)min_buffer_index = 0;
		for (k = 0; k < FFTLEN; k++) {
			if (in_filt == 2) {
				frame_min_buffer[min_buffer_index*FFTLEN + k] = sqrt(P[k]);
			}
			else {
				if (in_filt == 1) {
					frame_min_buffer[min_buffer_index*FFTLEN + k] = P[k];
				}
				else {
					frame_min_buffer[min_buffer_index*FFTLEN + k] = frame_mag[k];
				}
			}
		}
	}
	else {
		for (k = 0; k < FFTLEN; k++) {
			if (in_filt == 2) {
				if (sqrt(P[k]) < frame_min_buffer[min_buffer_index*FFTLEN + k])frame_min_buffer[min_buffer_index*FFTLEN + k] = sqrt(P[k]);
			}
			else {
				if (in_filt == 1) {
					if (P[k] < frame_min_buffer[min_buffer_index*FFTLEN + k])frame_min_buffer[min_buffer_index*FFTLEN + k] = P[k];
				}
				else {
					if (frame_mag[k] < frame_min_buffer[min_buffer_index*FFTLEN + k])frame_min_buffer[min_buffer_index*FFTLEN + k] = frame_mag[k];
				}
			}
		}
	}

	for (k = 0; k < FFTLEN; k++) {
		N[k] = alpha*min(min(frame_min_buffer[k], frame_min_buffer[256 + k]), min(frame_min_buffer[k + 512], frame_min_buffer[k + 768]));
		if (out_filt == 1) {
			PP[k] = (1 - K)*N[k] + K*last_PP[k];
			last_PP[k] = PP[k];
		}
		if (out_filt == 1) { N[k] = PP[k]; };
		switch (gain_var) {
		case 0:inframe[k] = rmul(max(lamda, (1 - N[k] / frame_mag[k])), inframe[k]); break;
		case 1:inframe[k] = rmul(max(lamda*N[k] / frame_mag[k], (1 - N[k] / frame_mag[k])), inframe[k]); break;
		case 2:inframe[k] = rmul(max(lamda*P[k] / frame_mag[k], (1 - N[k] / frame_mag[k])), inframe[k]); break;
		case 3:inframe[k] = rmul(max(lamda*N[k] / P[k], (1 - N[k] / P[k])), inframe[k]); break;
		case 4:inframe[k] = rmul(max(lamda, (1 - N[k] / P[k])), inframe[k]); break;
		default:inframe[k] = rmul(max(lamda, (1 - N[k] / frame_mag[k])), inframe[k]); break;
		}

	}

	ifft(FFTLEN, inframe);

}
