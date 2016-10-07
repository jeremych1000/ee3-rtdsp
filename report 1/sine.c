/*************************************************************************************
			       DEPARTMENT OF ELECTRICAL AND ELECTRONIC ENGINEERING
					   		     IMPERIAL COLLEGE LONDON 

 				      EE 3.19: Real Time Digital Signal Processing
					       Dr Paul Mitcheson and Daniel Harvey

				        LAB 2: Learning C and Sinewave Generation 

 				             ********* S I N E . C **********

             	Demonstrates outputing data from the DSK's audio port. 
   			  Used for extending knowledge of C and using look up tables.

 *************************************************************************************
  				Updated for use on 6713 DSK by Danny Harvey: May-Aug 06/Dec 07/Oct 09
				CCS V4 updates Sept 10
 ************************************************************************************/
/*
 *  Initialy this example uses the AIC23 codec module of the 6713 DSK Board Support
 *  Library to generate a 1KHz sine wave using a simple digital filter. 
 *	You should modify the code to generate a sine of variable frequency.
 */
/**************************** Pre-processor statements ******************************/

//  Included so program can make use of DSP/BIOS configuration tool.  
#include "dsp_bios_cfg.h"

/* The file dsk6713.h must be included in every program that uses the BSL.  This 
   example also includes dsk6713_aic23.h because it uses the 
   AIC23 codec module (audio interface). */
#include "dsk6713.h"
#include "dsk6713_aic23.h"

// math library (trig functions)
#include <math.h>

// Some functions to help with configuring hardware
#include "helper_functions_polling.h"


// PI defined here for use in your code 
#define PI 3.141592653589793

#include <stdio.h>

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
    0x004f,  /* 7 DIGIF      Digital audio interface format  32 bit                */\
    0x008d,  /* 8 SAMPLERATE Sample rate control             8 KHZ                 */\
    0x0001   /* 9 DIGACT     Digital interface activation    On                    */\
			 /**********************************************************************/
};


// Codec handle:- a variable used to identify audio interface  
DSK6713_AIC23_CodecHandle H_Codec;

/* Sampling frequency in HZ. Must only be set to 8000, 16000, 24000
32000, 44100 (CD standard), 48000 or 96000  */ 
int sampling_freq = 8000;


// Array of data used by sinegen to generate sine. These are the initial values.                        
float y[3] = {0,0,0};
float x[1] = {1}; // impulse to start filter

float a0 = 1.4142; // coefficients for difference equation
float b0 = 0.707;

// Holds the value of the current sample 
float sample; 

/* Left and right audio channel gain values, calculated to be less than signed 32 bit
 maximum value. */
Int32 L_Gain = 2100000000;
Int32 R_Gain = 2100000000;


/* Use this variable in your code to set the frequency of your sine wave 
   be carefull that you do not set it above the current nyquist frequency! */
float sine_freq = 1000.0;         

     
/******************************* Function prototypes ********************************/
void init_hardware(void);     
float sinegen(void);
float sinegen2(float* table, float f, float fs);
float sinegen3(float* table, float f, float fs);
void sine_init(void);
/********************************** Main routine ************************************/

//for q1, trace table
float initial[20] = {0};
int a = 0;
//for variable f
#define SINE_TABLE_SIZE 1000
float table[SINE_TABLE_SIZE] = {0};

#define ENABLE_ONE_QUADRANT 1

void main()
{
	// initialize board and the audio port
	init_hardware();
	sine_init();
    // Loop endlessley generating a sine wave 
   while(1)
    {
 		// Calculate next sample
 		//sample = sinegen();
 		sample = (ENABLE_ONE_QUADRANT)?(sinegen3(table, (float)sine_freq, (float)sampling_freq)):(sinegen2(table, (float)sine_freq, (float)sampling_freq));
 		if (a<20) {
 			initial[a] = sample; 		
 			a++;
 		}
 		
     	/* Send a sample to the audio port if it is ready to transmit.
           Note: DSK6713_AIC23_write() returns false if the port if is not ready */

        //  send to LEFT channel (poll until ready)
        while (!DSK6713_AIC23_write(H_Codec, ((Int32)(sample * L_Gain))))
        {};
		// send same sample to RIGHT channel (poll until ready)
        while (!DSK6713_AIC23_write(H_Codec, ((Int32)(sample * R_Gain))))
        {};
        
		// Set the sampling frequency. This function updates the frequency only if it 
		// has changed. Frequency set must be one of the supported sampling freq.
		set_samp_freq(&sampling_freq, Config, &H_Codec);
	
	}
	
}

/******************************* init_hardware() ************************************/
void init_hardware()
{
    // Initialize the board support library, must be called first 
    DSK6713_init();
    
    // Start the codec using the settings defined above in config 
    H_Codec = DSK6713_AIC23_openCodec(0, &Config);

	/* Defines number of bits in word used by MSBSP for communications with AIC23
	 NOTE: this must match the bit resolution set in in the AIC23 */
	MCBSP_FSETS(XCR1, XWDLEN1, 32BIT);
	
	/* Set the sampling frequency of the audio port. Must only be set to a supported 
	   frequency (8000/16000/24000/32000/44100/48000/96000) */
	
	DSK6713_AIC23_setFreq(H_Codec, get_sampling_handle(&sampling_freq));

}

/********************************** sinegen() ***************************************/   
float sinegen(void)
{
/*  This code produces a fixed sine of 1KHZ (if the sampling frequency is 8KHZ)
    using a digital filter.
 	You will need to re-write this function to produce a sine of variable frequency
 	using a look up table instead of a filter.*/
	
	// temporary variable used to output values from function
	float wave;	
	
	// represets the filter coeficients (square root of 2 and 1/square root of 2)
	float a0 = 1.4142;
	float b0 = 0.7071;

	y[0] = a0 * y[1] - y[2] + b0 * x[0]; // Difference equation

	y[2] = y[1]; // move values through buffer
	y[1] = y[0];

	x[0] = 0; // reset input to zero (to create the impulse) 

	wave = y[0];
			   
    return(wave); 
    
}

float sinegen2(float* table, float f, float fs){
	static float iterator = 0;		
	//use linear iterpolation between next value to get an average
	float result = (iterator<=SINE_TABLE_SIZE-2)?((table[(int)iterator]) + ((table[(int)(iterator+1)] - table[(int)iterator])/2)):(table[(int)iterator]);	
	iterator += f*( SINE_TABLE_SIZE )/fs;
	if (iterator > SINE_TABLE_SIZE) {iterator -= SINE_TABLE_SIZE;}
	return result;
}

float sinegen3(float* table, float f, float fs){
	static float index = 0.0;
	
	float result;	
	float step = (float)360*f/fs;
	
	if (index == 0 || index == 180 || index == 360) result = table[0];
	if (index == 90) result = table[SINE_TABLE_SIZE - 1];
	if (index == 270) result = -table[SINE_TABLE_SIZE - 1];
	if (index<90) result = table[((int)index/90)*SINE_TABLE_SIZE];
	if (index>90 && index<180) result = table[SINE_TABLE_SIZE - (((int)index-90)/90)*SINE_TABLE_SIZE];
	if (index>180 && index<270) result = -table[(((int)index-180)/90)*SINE_TABLE_SIZE];
	if (index>270 && index<360) result = -table[SINE_TABLE_SIZE - (((int)index-270)/90)*SINE_TABLE_SIZE];
	
	index += step;	
	if (index>360) index -= 360;
	
	return result;
}

void sine_init(){
	int i = 0;
	for (i=0; i<SINE_TABLE_SIZE; i++){
		table[i] = (ENABLE_ONE_QUADRANT)?(sin((i*PI/2)/SINE_TABLE_SIZE)):(sin((2*i*PI)/SINE_TABLE_SIZE));
	}
}


