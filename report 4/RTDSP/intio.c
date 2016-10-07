/*************************************************************************************
DEPARTMENT OF ELECTRICAL AND ELECTRONIC ENGINEERING
IMPERIAL COLLEGE LONDON

EE 3.19: Real Time Digital Signal Processing
Dr Paul Mitcheson and Daniel Harvey

LAB 5: IIR Filters

********* I N T I O. C **********

Demonstrates inputing and outputing data from the DSK's audio port using interrupts.

*************************************************************************************
Updated for use on 6713 DSK by Danny Harvey: May-Aug 2006
Updated for CCS V4 Sept 10
************************************************************************************/
/*
*	You should modify the code so that interrupts are used to service the
*  audio port.
*/
/**************************** Pre-processor statements ******************************/

#include <stdlib.h>
//  Included so program can make use of DSP/BIOS configuration tool.  
#include "dsp_bios_cfg.h"

/* The file dsk6713.h must be included in every program that uses the BSL.  This
example also includes dsk6713_aic23.h because it uses the
AIC23 codec module (audio interface). */
#include "dsk6713.h"
#include "dsk6713_aic23.h"

// math library (trig functions)
#include <math.h>

// Some functions to help with writing/reading the audio ports when using interrupts.
#include <helper_functions_ISR.h>

#include <../../iir_coef.txt>

/******************************* Global declarations ********************************/

/* Audio port configuration settings: these values set registers in the AIC23 audio
interface to configure it. See TI doc SLWS106D 3-3 to 3-10 for more info. */
DSK6713_AIC23_Config Config = { \
/**********************************************************************/
/*   REGISTER	            FUNCTION			      SETTINGS         */
/**********************************************************************/\
0x0017,  /* 0 LEFTINVOL  Left line input channel volume  0dB                   */\
0x0017,  /* 1 RIGHTINVOL Right line input channel volume 0dB                   */\
0x01f9,  /* 2 LEFTHPVOL  Left channel headphone volume   0dB                   */\
0x01f9,  /* 3 RIGHTHPVOL Right channel headphone volume  0dB                   */\
0x0011,  /* 4 ANAPATH    Analog audio path control       DAC on, Mic boost 20dB*/\
0x0000,  /* 5 DIGPATH    Digital audio path control      All Filters off       */\
0x0000,  /* 6 DPOWERDOWN Power down control              All Hardware on       */\
0x0043,  /* 7 DIGIF      Digital audio interface format  16 bit                */\
0x008d,  /* 8 SAMPLERATE Sample rate control             8 KHZ                 */\
0x0001   /* 9 DIGACT     Digital interface activation    On                    */\
/**********************************************************************/
};


// Codec handle:- a variable used to identify audio interface  
DSK6713_AIC23_CodecHandle H_Codec;
// Pre-processor definitions

/******************************* Function prototypes ********************************/
void init_hardware(void);
void init_HWI(void);
void init_buffer(void);
void interrupt_service_routine(void);
double Single_pole(double sample);
double Dir_form_2(double sample);
double Dir_form_2_transposed(double sample);

double *buffer;
#define b0 0.0588235941
#define a1 -0.8823529

/********************************** Main routine ************************************/
void main() {


	// initialize board and the audio port
	init_hardware();
	/* initialize hardware interrupts */
	init_HWI();
	buffer = calloc(N + 1, sizeof(double));
	init_buffer();

	/* loop indefinitely, waiting for interrupts */
	while (1)
	{
	};

}

/********************************** hardware() **********************************/
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

	/* These commands do the same thing as above but applied to data transfers to
	the audio port */
	MCBSP_FSETS(XCR1, XWDLEN1, 32BIT);
	MCBSP_FSETS(SPCR1, XINTM, FRM);


}

/********************************** HWI() **************************************/
/* Change IRQ_EVT_RINT1 to IRQ_EVT_XINT1 when transitioning from a recieve to transmit interrupt */
void init_HWI(void)
{
	IRQ_globalDisable();			// Globally disables interrupts
	IRQ_nmiEnable();				// Enables the NMI interrupt (used by the debugger)
	IRQ_map(IRQ_EVT_RINT1, 4);		// Maps an event to a physical interrupt
	IRQ_enable(IRQ_EVT_RINT1);		// Enables the event
	IRQ_globalEnable();				// Globally enables interrupts

}

/******************** WRITE YOUR INTERRUPT SERVICE ROUTINE HERE***********************/
/**
 * This function gets executed upon a hardware interrupt. It reads in a sample,
 * then outputs the output of one the IIR functions.
 */
void interrupt_service_routine(void) {
	double sum = 0.0;
	double sample = 0.0;
	sample = (double)mono_read_16Bit();
	//sum = Single_pole(sample);
	//sum = Dir_form_2(sample);
	sum = Dir_form_2_transposed(sample);
	mono_write_16Bit((Int16)(sum));
}

/**
* Initialises the buffer to all 0's.
*/
void init_buffer(void) {
	int index = 0;
	for (index = 0; index<N + 1; index++) {
		buffer[index] = 0;
	}
}

/**
* Difference equation implementation of the IIR filter. 
* @param  sample 
* @return        [description]
*/
double Single_pole(double sample) {
	static double out = 0.0;				//use static variable to store output value
	static double last_x = 0.0;				//use static variable to store last input value
	out = b0*sample + b0*last_x - a1*out;	//difference equation
	last_x = sample;						//put current sample into last sample variable
	return out;
}

/**
 * Elliptic IIR filter, direct form II implementation. Uses MATLAB generated
 * coefficients specified in include txt file to calculate filter. Circular
 * buffer used, initialized using calloc().
 * @param  sample sample_in, xin on diagram
 * @return        64bit output, yout on diagram
*/
double Dir_form_2(double sample) {
	static int write_index = N;				//updatable write index for newest delay line block
	double in_data = 0.0;					//for multiplying by a1, a2, a3...
	double out_data = 0.0;					//for multiplying by b1, b2, b3...
	double write_data = 0.0;				//summation of delayed coefficients, multiplied by a1, a2, a3, with input value
	int read_index, loop_index;				//declaration of iterators
	for (loop_index = 1; loop_index <= N; loop_index++) {
		read_index = ((write_index + loop_index)>N) ? write_index + loop_index - N - 1 : write_index + loop_index;	//circular buffer wraparound logic
		out_data += buffer[read_index] * b[loop_index];	//output acculumator
		in_data -= buffer[read_index] * a[loop_index];	//input acculumator
	}
	write_data = sample + in_data;			//summation of incoming sample and input side of direct form
	buffer[write_index] = write_data;		//write to newest delay line block
	out_data += write_data*b[0];			//multiplication of summed write_data, aka top line of direct form structure
	if (--write_index == -1)write_index = N;	//handle circular buffer wraparound
	return out_data;
}

/**
 * Direct form II transposed implementation. Feedforward b coefficients now
 * directly multiplied by sample in before delay/accumulate. a coefficients used
 * for feedback. Buffer size is N.
 * @param  sample sample_in, xin on diagram
 * @return        64bit output, yout on diagram
*/
double Dir_form_2_transposed(double sample) {
	double out_data = 0.0;		//output variable, used for feedback loop
	int index = 0;				//iterator declaration
	out_data = b[0] * sample + buffer[0];	//do edge case of b0 * sample in + newest delay line block
	for (index = 1; index<N; index++) {
		buffer[index - 1] = buffer[index] + b[index] * sample - a[index] * out_data;	//compute delay line, b is feedforward, a is feedback
	}
	buffer[N - 1] = sample*b[N] - a[N] * out_data; //another edge case of oldest delay line block
	return out_data;
}
