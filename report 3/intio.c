/*************************************************************************************
				   DEPARTMENT OF ELECTRICAL AND ELECTRONIC ENGINEERING
								 IMPERIAL COLLEGE LONDON

					  EE 3.19: Real Time Digital Signal Processing
						   Dr Paul Mitcheson and Daniel Harvey

								  LAB 4: FIR Filters

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

   // Some functions to help with writing/reading the audio ports when using interrupts.
#include <helper_functions_ISR.h>

#include <../coef/fir_coef_213.txt>
#define N 213

#if (N%2)
#define mid_pt ((N-1)/2)
#else
#define mid_pt (N/2)
#endif
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

//delay lines
double x[N] = { 0 };
double x_extended[2 * N] = { 0 };
 /******************************* Function prototypes ********************************/
void init_hardware(void);
void init_HWI(void);

void interrupt_service_routine(void);
double non_cir_FIR(double sample_in);
double cir_FIR(double sample_in);
double linear_cir_FIR(double sample_in);
double sym_fir(double sample);
double non_cir_FIR_sym(double sample_in);
double non_cir_FIR_2(double sample_in);
double sym_fir_ptr(double sample_in);

/********************************** Main routine ************************************/
void main() {


	// initialize board and the audio port
	init_hardware();
	/* initialize hardware interrupts */
	init_HWI();

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
 * then outputs the output of one the FIR functions.
 */
void interrupt_service_routine(void) {
	double sum = 0.0;
	double sample = 0.0;
	sample = (double)mono_read_16Bit();
	//all pass if needed
	//mono_write_16Bit((Int16)(sample*4));
	//FIR FUNCTIONS BELOW
	//sum = non_cir_FIR(sample);
	//sum = cir_FIR(sample);
	//sum = linear_cir_FIR(sample);
	//sum = sym_fir(sample);
	//sum = non_cir_FIR_sym(sample);
	//sum = non_cir_FIR_2(sample);
	sum = sym_fir_ptr(sample);
	mono_write_16Bit((Int16)(sum * 2));
}

/**
 * Naive non circular FIR implementation. Uses a for loop to implement delay
 * line.
 * @param  sample_in [16 bit sample from mono_read_16Bit, casted to 64 bit]
 * @return           [64 bit MAC value]
 */
double non_cir_FIR(double sample_in) {
	int i, j;						//define variables for use in loops
	double sum = 0;
	for (i = N - 1; i > 0; i--) {
		x[i] = x[i - 1];				//array shuffling method as described in handout
	}
	x[0] = sample_in;				//read in newest sample to one end of the array

	for (j = 0; j < N; j++) {
		sum += b[j] * x[j]; 			//MAC
	}
	return sum;
}

/**
 * Implements circular buffer FIR filter. Read/write index used to keep track of
 * oldest sample and MAC iteration. If statements used to handle wraparound.
 * @param  sample_in [16 bit sample from mono_read_16Bit, casted to 64 bit]
 * @return           [64 bit MAC value]
 */
double cir_FIR(double sample_in) {
	int j, read_index;				//define variables for use in loops
	double sum = 0.0;
	static int write_index = N - 1;		//use a static variable to prevent reinitialization between function calls, write index is the pointer to the newest sample in the vector x
	x[write_index] = sample_in;		//read the newest sample in at the pointer write index
	for (j = 0; j < N; j++) {
		read_index = (write_index + j > N - 1) ? write_index + j - N : write_index + j; //read index starts from write index (j=0 at first run) and iterates through the array, wrapping around (circular buffer) when the iterator is larger than the size of array x
		sum += b[j] * x[read_index];	//MAC
	}
	if (--write_index == -1)write_index = N - 1;	//update write_index to oldest sample for overwriting on next function call, and check for wraparound of write_index - circular buffering
	return sum;
}

/**
 * Exploits linear phase filter properties (symmetrical) to cut down half of MAC
 * iterations. Uses a circular buffer.
 * @param  sample_in [16 bit sample from mono_read_16Bit, casted to 64 bit]
 * @return           [64 bit MAC value]
 */
double linear_cir_FIR(double sample_in) {
	int j, read_index, i, read_index2;
	double sum;
	static int write_index = N - 1;		//use static to keep value of write_index between function calls
	i = N - 1;							//last array index
	x[write_index] = sample_in;		//write incoming sample to current value of write_index
	read_index = (write_index + mid_pt > N - 1) ? write_index + mid_pt - N : write_index + mid_pt;	//handle circular buffering wraparounds, and acts as start of MAC
	sum = b[mid_pt] * x[read_index];	//execute MAC on midpoint, as there is only one point to convolve, instead of two, so do outside for loop
	for (j = 0; j < mid_pt; j++) {		//calculate FIR filter inwards - read_index2 is the mirror of read_index
		read_index = (write_index + j > N - 1) ? write_index + j - N : write_index + j;
		read_index2 = ((read_index + i) > N - 1) ? read_index + i - N : read_index + i;
		sum += b[j] * (x[read_index] + x[read_index2]);
		i = i - 2;						//as calculating inwards, move both iterators
	}
	if (--write_index == -1)write_index = N - 1;	//update write_index to oldest sample for overwriting on next function call, and check for wraparound of write_index - circular buffering
	return sum;
}

/**
 * Doubles the size of the buffer used to implement the delay line to remove
 * need for if statements checking for wraparound in the MAC loop as it will
 * never overflow.
 * @param  sample_in [16 bit sample from mono_read_16Bit, casted to 64 bit]
 * @return           [64 bit MAC value]
 */
double sym_fir(double sample) {
	static int new_index = N - 1;
	int old_index, iterator;
	double sum;
	old_index = new_index + N - 1;		//index the extended buffer by adding array size
	x_extended[new_index] = sample;	//copy sample in
	x_extended[new_index + N] = sample;	//to both sections of the array
	sum = b[mid_pt] * x_extended[new_index + mid_pt];	//calculate midpoint MAC first as it only needs one value
	for (iterator = 0; iterator < mid_pt; iterator++) {	//loop iterations decreased by half
		sum += b[iterator] * (x_extended[new_index + iterator] + x_extended[old_index--]);	//calculate MAC using both sections of the array
	}
	if (--new_index < 0)new_index = N - 1;	//update oldest sample index
	return sum;
}

/**
 * Original non-circular approach, with symmetrical properties.
 * @param  sample_in [16 bit sample from mono_read_16Bit, casted to 64 bit]
 * @return           [64 bit MAC value]
 */
double non_cir_FIR_sym(double sample_in) {
	int i, j;
	double sum = 0;
	for (i = N - 1; i > 0; i--) {
		x[i] = x[i - 1];				//array shuffling
	}
	x[0] = sample_in;
	sum = b[mid_pt] * x[mid_pt];		//compute midpoint MAC first as it only requires one coefficient
	for (j = 0; j < mid_pt; j++) {		//N/2 iterations
		sum += b[j] * (x[j] + x[N - 1 - j]); //rest of MAC
	}
	return sum;
}

/**
 * This version of the non circular buffer moves the shift for loop inside the
 * MAC for loop. This version is, surprisingly, quicker than the previous
 * circular buffer linear phase functions as the compiler parallizes both the
 * shifting and the MAC. God bless Texas Instruments.
 * @param  sample_in [16 bit sample from mono_read_16Bit, casted to 64 bit]
 * @return           [64 bit MAC value]
 */
double non_cir_FIR_2(double sample_in) {
	int i;
	double sum = b[N - 1] * x[N - 1];		//might as well initialize to a value not calculated by for loop as the for loop iteration size for delay and MAC differ by 1
	x[0] = sample_in;				//read input sample
	for (i = N - 2; i >= 0; i--) {			//size of MAC loop is larger than delay loop by 1
		sum += b[i] * x[i];
		x[i + 1] = x[i];				//change from i>i-1 to i+1>i, as MAC needs to index b[0] and x[0], and x[-1] is undefined
	}
	return sum;
}

/**
 * Explots linear phase properties, and uses a double delay line for speed up.
 * Also replace array indices with const double pointers. Inward movements of
 * the points calculate the MAC, with the midpoint being calculated outside the
 * loop.
 * @param  sample_in [16 bit sample from mono_read_16Bit, casted to 64 bit]
 * @return           [64 bit MAC value]
 */
double sym_fir_ptr(double sample_in) {
	int i;
	static int index = N-1;		//starting write index for new sample
	double sum = 0.0;
	const double *ptr_b = b, *ptr_x = x_extended + index, *ptr_x2 = x_extended + index + N - 1;	//use const double pointer to index the array instead of array indices
	//const double means the pointed to data is constant, but the pointer can change
	//ptr_b points to starting address of array b

	x_extended[index] = x_extended[index + N] = sample_in;	//write incoming sample to both array and extended array
		
	for (i = 0; i < mid_pt; i++) {
		sum += *ptr_b++ * (*ptr_x++ + *ptr_x2--);	//compute MAC by moving pointers inwards
	}
	sum += *ptr_b++ * *ptr_x++;		//compute midpoint, points are conveniently in the right place after for loop
	
	if (--index < 0) index += N;		//handle write index overflow
	return sum;
}
