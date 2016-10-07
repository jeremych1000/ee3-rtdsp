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
#include <stdio.h>
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



/////////////////////swithchable parameter
int alpha = 8;
int alphatmp = 8;
int alphathresholdlow = 5;
int alphathresholdhigh = 20;
int enchance6 = 0;
float SNRtmp = 0.0;
float lambda = 0.01;
float tau = 0.02; //from 0.02 to 0.08
int in_filt = 1;
int out_filt = 1;
int gain_var = 4; //2,3,4 only work with in- filter on
float RNR_threhold = 0.9;
#define RNR_flag 0
//int RNR_flag = 0;
int  min_frame_rate = 78;
float speech_thres = 4;
float prob_const = 0.2;
float sub_const = 0.85;
float delta = 0.0;

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
float *last_N;
float *last_frame_mag;
complex *RNRbuffer;
complex *last_frame;
complex *last_last_frame;
complex* out_buff;
float* speech_prob;
float* local_min;
float* subfactor;
float ingain, outgain;				/* ADC and DAC gains */
float cpufrac;						/* Fraction of CPU time used */
volatile int io_ptr = 0;				/* Input/ouput pointer for circular buffers */
volatile int frame_ptr = 0;			/* Frame pointer */
volatile int frame_count = 0;
volatile int min_buffer_index = 0;
float frame_min_buffer[OVERSAMP * FFTLEN] = {FLT_MAX};
float noise_amp = FLT_MAX;
float K;



/******************************* Function prototypes *******************************/
void init_hardware(void);		/* Initialize codec */
void init_HWI(void);			/* Initialize hardware interrupts */
void ISR_AIC(void);				/* Interrupt service routine for codec */
void process_frame(void);		/* Frame processing routine */
void init_buffer(void);
void ProcVswitch(void);
void ProcVswitch_estimator(void);
inline float max(float a, float b);
inline float min(float a, float b);
complex complex_min(complex a, complex b);
inline int ilog10c(float input);
inline float my_sqrt(const float m);

/********************************** Main routine ************************************/
void main()
{

	int k; // used in various for loops

	/*	Initialize and zero fill arrays */

	inbuffer	= (float *) calloc(CIRCBUF, sizeof(float));	/* Input array */
	outbuffer	= (float *) calloc(CIRCBUF, sizeof(float));	/* Output array */
	inframe		= (complex *) calloc(FFTLEN, sizeof(complex));	/* Array for processing*/
	outframe	= (float *) calloc(FFTLEN, sizeof(float));	/* Array for processing*/
	inwin		= (float *) calloc(FFTLEN, sizeof(float));	/* Input window */
	outwin		= (float *) calloc(FFTLEN, sizeof(float));	/* Output window */
	frame_mag	= (float *) calloc(FFTLEN, sizeof(float));
	N			= (float *) calloc(FFTLEN, sizeof(float));
	last_frame 	= (complex *) calloc(FFTLEN, sizeof(float));
	last_last_frame = (complex *) calloc(FFTLEN, sizeof(float));
	last_frame_mag = (float *) calloc(FFTLEN, sizeof(float));
	last_N = (float *) calloc(FFTLEN, sizeof(float));
	last_P = (float *) calloc(FFTLEN, sizeof(float));
	P = (float *) calloc(FFTLEN, sizeof(float));
	last_PP = (float *) calloc(FFTLEN, sizeof(float));
	PP	= (float *) calloc(FFTLEN, sizeof(float));
	RNRbuffer = (complex *) calloc(FFTLEN, sizeof(complex));
	out_buff = (complex*)calloc(FFTLEN, sizeof(complex));
	speech_prob = (float*)calloc(FFTLEN, sizeof(float));
	local_min = (float*)calloc(FFTLEN, sizeof(float));
	subfactor = (float*)calloc(FFTLEN, sizeof(float));
	K = exp(-TFRAME / tau);


	/* initialize board and the audio port */
	init_hardware();
	init_buffer();
	/* initialize hardware interrupts */
	init_HWI();

	/* initialize algorithm constants */

	for (k = 0; k < FFTLEN; k++)
	{
		inwin[k] = sqrt((1.0 - WINCONST * cos(PI * (2 * k + 1) / FFTLEN)) / OVERSAMP);
		outwin[k] = inwin[k];
	}
	ingain = INGAIN;
	outgain = OUTGAIN;

	//puts("test\n");

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
	for (i = 0; i < FFTLEN * OVERSAMP; i++) {
		frame_min_buffer[i] = FLT_MAX;
	}
}
/********************************** maxmin() *********************************/
inline float max (float a, float b) {
	return (((a) > (b)) ? (a) : (b));
}
inline float min (float a, float b) {
	return (((a) < (b)) ? (a) : (b));
}
complex complex_min(complex a, complex b) {
	return (((cabs(a)) < (cabs(b))) ? (a) : (b));
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
	cpufrac = ((float) (io_ptr & (FRAMEINC - 1))) / FRAMEINC;

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
	{	if (RNR_flag == 0) {
			outframe[k] = inframe[k].r;/* copy input straight into output */
		}
		else {
			outframe[k] = RNRbuffer[k].r;
		}

	}

	/********************************************************************************/

	/* multiply outframe by output window and overlap-add into output buffer */

	m = io_ptr0;

	for (k = 0; k < (FFTLEN - FRAMEINC); k++)
	{	/* this loop adds into outbuffer */
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
	inbuffer[io_ptr] = ((float)sample) * ingain;
	/* write new output data */
	mono_write_16Bit((int)(outbuffer[io_ptr]*outgain));

	/* update io_ptr and check for buffer wraparound */

	if (++io_ptr >= CIRCBUF) io_ptr = 0;
}


/********************************** all Versions of  proces function  *********************************/
///vswitch is original setting with no filtering,G = max(lambda,1-|N|/|X|),
void ProcVswitch(void) {
	int k;

	//////////////////////////////////
	//compute FFT of incoming frame //
	//////////////////////////////////
	fft(FFTLEN, inframe);

	for (k = 0; k < FFTLEN; k++) {
		/* calculate absolute of fft bin of signal as phase unknown */
		frame_mag[k] = cabs(inframe[k]);

		//////////////////////////////
		//enchancement 2: in_filt 2 //
		//////////////////////////////
		//low pass filter |X(\omega)|^2, then sqrt
		//Thereotically closest to paper approach as the estimator is estimating noise power and not noise magnitude.

		if (in_filt == 2) {
			P[k] = (1 - K) * (frame_mag[k] * frame_mag[k]) + K * last_P[k];
			last_P[k] = P[k];
		}
		else {

			//////////////////////////////
			//enchancement 1: in_filt 1 //
			//////////////////////////////
			//P_t(\omega)=(1-k) x abs(X(\omega))+k x P_t-1(\omega)
			//LPF on noise estimator, used to smooth subbands
			//Current frame * times + last frame due to decimation

			if (in_filt == 1) {
				P[k] = (1 - K) * frame_mag[k] + K * last_P[k];
				last_P[k] = P[k];
			}
		}
	}
	//downcounter to keep track of which frame for processing
	//execute at margins between adjacent frames
	if (--frame_count == -1) {
		frame_count = min_frame_rate;
		//handle wraparound of min_buffer_index so 0->3
		if (++min_buffer_index > OVERSAMP - 1)min_buffer_index = 0;
		for (k = 0; k < FFTLEN; k++) {
			////////////////////////////////////
			//enhancement 2 update min buffer //
			////////////////////////////////////
			if (in_filt == 2) {
				frame_min_buffer[min_buffer_index * FFTLEN + k] = sqrt(P[k]);
			}
			else {
				////////////////////////////////////
				//enhancement 1 update min buffer //
				////////////////////////////////////
				if (in_filt == 1) {
					frame_min_buffer[min_buffer_index * FFTLEN + k] = P[k];
				}
				else {
					frame_min_buffer[min_buffer_index * FFTLEN + k] = frame_mag[k];
				}
			}
		}
	}

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//main objective: implement the spectral subtraction technique                                                                                    //
	//Assuming non-musical noise, so no convolution between speech and noise.                                                                         //
	//Therefore, all noise is in the background and statistically independent, therefore treat as additive noise, so the subtraction technique works. //
	//Peaks in FFT correspond to speech, therefore find lowest valley (local minimum thereotically be noise).                                         //
	//In C, compare minimum by looping through frequency bins, and find minimum frequency bins                                                        //
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	//same updating min buffer
	else {
		for (k = 0; k < FFTLEN; k++) {
			if (in_filt == 2) {
				if (sqrt(P[k]) < frame_min_buffer[min_buffer_index * FFTLEN + k])frame_min_buffer[min_buffer_index * FFTLEN + k] = sqrt(P[k]);
			}
			else {
				if ( in_filt == 1) {
					if (P[k] < frame_min_buffer[min_buffer_index * FFTLEN + k])frame_min_buffer[min_buffer_index * FFTLEN + k] = P[k];
				}
				else {
					if (frame_mag[k] < frame_min_buffer[min_buffer_index * FFTLEN + k])frame_min_buffer[min_buffer_index * FFTLEN + k] = frame_mag[k];
				}
			}
		}
	}

	for (k = 0; k < FFTLEN; k++) {
		{
			///////////////////////////////////////////////
			//enchancement 6, calc snr, and adjust alpha //
			///////////////////////////////////////////////
			if (enchance6) {
				//calculate SNR
				SNRtmp = (frame_mag[k] * frame_mag[k]) / (N[k] * N[k]);
				//find alpha value from SNR using lookup table
				alphatmp = ilog10c(SNRtmp);
			}
		}

		//update noise buffer
		N[k] = alphatmp * min(min(frame_min_buffer[k], frame_min_buffer[FFTLEN + k]), min(frame_min_buffer[k + FFTLEN * 2], frame_min_buffer[k + FFTLEN * 3]));


		////////////////////
		//enchancement 3: //
		////////////////////
		//LPF the noise estimate to prevent subtracting discontinuties
		//Smooth noise estimator using LPF

		if (out_filt == 1) {
			PP[k] = (1 - K) * N[k] + K * last_PP[k];
			last_PP[k] = PP[k];
			N[k] = PP[k];
		}

		////////////////////////////////////////////
		//enhancement 8: residual noise reduction //
		////////////////////////////////////////////
		//if noise over some threshold, get minimum from adjacent frames instead
		if (RNR_flag == 1) {
			if ((last_N[k] / last_frame_mag[k]) > RNR_threhold) {
				//over threshold, get min from adjacent
				RNRbuffer[k] = complex_min(inframe[k], complex_min(last_frame[k], last_last_frame[k]));
			}
			else {
				//else get last inframe is the future frame
				RNRbuffer[k] = last_frame[k];
				switch (gain_var) {
				//implement gain variations
				case 0: inframe[k] = rmul(max(lambda, (1 - N[k] / frame_mag[k])), inframe[k]); break;
				case 1: inframe[k] = rmul(max(lambda * N[k] / frame_mag[k], (1 - N[k] / frame_mag[k])), inframe[k]); break;
				case 2: inframe[k] = rmul(max(lambda * P[k] / frame_mag[k], (1 - N[k] / frame_mag[k])), inframe[k]); break;
				case 3: inframe[k] = rmul(max(lambda * N[k] / P[k], (1 - N[k] / P[k])), inframe[k]); break;
				case 4: inframe[k] = rmul(max(lambda, (1 - N[k] / P[k])), inframe[k]); break;
				case 5: inframe[k] = rmul(max(lambda, sqrt(1 - (N[k] * N[k]) / (frame_mag[k] * frame_mag[k]))), inframe[k]); break;
				case 6: inframe[k] = rmul(max(lambda * N[k] / frame_mag[k], sqrt((1 - (N[k] * N[k]) / (frame_mag[k] * frame_mag[k])))), inframe[k]);    break;
				case 7: inframe[k] = rmul(max(lambda * P[k] / frame_mag[k], sqrt((1 - (N[k] * N[k]) / (frame_mag[k] * frame_mag[k])))), inframe[k]);    break;
				case 8: inframe[k] = rmul(max(lambda * N[k] / P[k], sqrt((1 - (N[k] * N[k]) / (P[k] * P[k])))), inframe[k]);    break;
				case 9: inframe[k] = rmul(max(lambda, sqrt((1 - (N[k] * N[k]) / (P[k] * P[k])))), inframe[k]);  break;
				default: inframe[k] = rmul(max(lambda, (1 - N[k] / frame_mag[k])), inframe[k]); break;
				}
			}
			//shuffle buffers
			last_frame_mag[k] = frame_mag[k];
			last_last_frame[k] = last_frame[k];
			last_frame[k] = inframe[k];
			last_N[k] = N[k];
		}
		else {
			switch (gain_var) {
			////////////////////////////
			//enhancement 4: case 0-4 //
			////////////////////////////
			//implement different variations of gain factor
			case 0: inframe[k] = rmul(max(lambda, (1 - N[k] / frame_mag[k])), inframe[k]); break;
			case 1: inframe[k] = rmul(max(lambda * N[k] / frame_mag[k], (1 - N[k] / frame_mag[k])), inframe[k]); break;
			case 2: inframe[k] = rmul(max(lambda * P[k] / frame_mag[k], (1 - N[k] / frame_mag[k])), inframe[k]); break;
			case 3: inframe[k] = rmul(max(lambda * N[k] / P[k], (1 - N[k] / P[k])), inframe[k]); break;
			case 4: inframe[k] = rmul(max(lambda, (1 - N[k] / P[k])), inframe[k]); break;
			////////////////////////////
			//enhancement 5: case 5-9 //
			////////////////////////////
			case 5: inframe[k] = rmul(max(lambda, sqrt(1 - (N[k] * N[k]) / (frame_mag[k] * frame_mag[k]))), inframe[k]); break;
			case 6: inframe[k] = rmul(max(lambda * N[k] / frame_mag[k], sqrt((1 - (N[k] * N[k]) / (frame_mag[k] * frame_mag[k])))), inframe[k]);    break;
			case 7: inframe[k] = rmul(max(lambda * P[k] / frame_mag[k], sqrt((1 - (N[k] * N[k]) / (frame_mag[k] * frame_mag[k])))), inframe[k]);    break;
			case 8: inframe[k] = rmul(max(lambda * N[k] / P[k], sqrt((1 - (N[k] * N[k]) / (P[k] * P[k])))), inframe[k]);    break;
			case 9: inframe[k] = rmul(max(lambda, sqrt((1 - (N[k] * N[k]) / (P[k] * P[k])))), inframe[k]);  break;
			///////////////////////
			//extra improvements //
			///////////////////////
			case 10: inframe[k] = rmul(max(sqrt(lambda) * N[k], 1 - N[k] / P[k]), inframe[k]);
			case 11: {
				//variation of variable alpha
				delta = (k < 32) ? 1 : (k < 64) ? 1.5 : 2.5;
				inframe[k] = rmul(max(lambda, 1 - sqrt((P[k] * P[k] - alphatmp * delta * N[k] * N[k]) / (frame_mag[k] * frame_mag[k]))), inframe[k]);
			}
			default: inframe[k] = rmul(max(lambda, (1 - N[k] / frame_mag[k])), inframe[k]); break;
			}
		}
	}
	////////////////////////////////////////////////////
	//compute ifft of frame to go back to time domain //
	////////////////////////////////////////////////////
	if (RNR_flag == 0) {
		ifft(FFTLEN, inframe);
	}
	else {
		ifft(FFTLEN, RNRbuffer);
	}

}

//////////////////////////////////
//enhancement 6: variable alpha //
//////////////////////////////////
/**
 * lookup table created using excel, returns alpha value from snr
 * @param  ratio [float snr]
 * @return       [int alpha]
 */
inline int ilog10c(float ratio) {
	if (ratio <	0.316227766	) return		20	;
	else if (ratio <	0.398107171	) return		19.28	;
	else if (ratio <	0.501187234	) return		18.56	;
	else if (ratio <	0.630957344	) return		17.84	;
	else if (ratio <	0.794328235	) return		17.12	;
	else if (ratio <	1	) return		16.4	;
	else if (ratio <	1.258925412	) return		15.68	;
	else if (ratio <	1.584893192	) return		14.96	;
	else if (ratio <	1.995262315	) return		14.24	;
	else if (ratio <	2.511886432	) return		13.52	;
	else if (ratio <	3.16227766	) return		12.8	;
	else if (ratio <	3.981071706	) return		12.08	;
	else if (ratio <	5.011872336	) return		11.36	;
	else if (ratio <	6.309573445	) return		10.64	;
	else if (ratio <	7.943282347	) return		9.92	;
	else if (ratio <	10	) return		9.2	;
	else if (ratio <	12.58925412	) return		8.48	;
	else if (ratio <	15.84893192	) return		7.76	;
	else if (ratio <	19.95262315	) return		7.04	;
	else if (ratio <	25.11886432	) return		6.32	;
	else if (ratio <	31.6227766	) return		5.6	;
	else if (ratio <	39.81071706	) return		4.88	;
	else if (ratio <	50.11872336	) return		4.16	;
	else if (ratio <	63.09573445	) return		3.44	;
	else if (ratio <	79.43282347	) return		2.72	;
	else	100	return		2	;
}

//try and implement faster square root, not really faster on DSK
/*
inline float my_sqrt(const float m)
{
	int j=0;
   float i=0;
   float x1,x2;
   while( (i*i) <= m )
          i+=0.1f;
   x1=i;
   for(j=0;j<10;j++)
   {
       x2=m;
      x2/=x1;
      x2+=x1;
      x2/=2;
      x1=x2;
   }
   return x2;
}  */
