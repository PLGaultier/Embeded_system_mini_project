#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <motors.h>
#include <audio/microphone.h>
#include <audio_processing.h>
#include <communications.h>
#include <fft.h>
#include <arm_math.h>
#include "leds.h"
#include "sensors/proximity.h"
#include <motion.h>


//semaphore
static BSEMAPHORE_DECL(sendToComputer_sem, TRUE);

//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
static float micLeft_cmplx_input[2 * FFT_SIZE];
static float micRight_cmplx_input[2 * FFT_SIZE];
static float micFront_cmplx_input[2 * FFT_SIZE];
static float micBack_cmplx_input[2 * FFT_SIZE];
//Arrays containing the computed magnitude of the complex numbers
static float micLeft_output[FFT_SIZE];
static float micRight_output[FFT_SIZE];
static float micFront_output[FFT_SIZE];
static float micBack_output[FFT_SIZE];

//must be accessible in both threads
static int16_t max_norm_index;

#define MIN_VALUE_THRESHOLD	10000

//400 is the code for the 6250Hz frequency used for our application
#define MIN_FREQ		350
#define MAX_FREQ		450


#define  DOT  2
#define  DASH 3

//number of 10ms high samples needed to consider a dot
#define  NB_HAUT_PAR_DOT  30

#define  NB_BAS_PAR_SPACE  NB_HAUT_PAR_DOT

//number of 10ms samples needed to consider a dash
#define  NB_HAUT_PAR_DASH  3*NB_HAUT_PAR_DOT

#define  NB_BAS_PAR_SPACE_LETTER  NB_HAUT_PAR_DASH

//margin to affirm that it is a dot
#define  MARGE  15

//number of high samples from which we consider that the level is high again
#define NB_HAUT_PAR_TEMPS_HAUT  10

//number of samples with minimal low value to stop considering that the value is high
#define NB_BAS_PAR_TEMPS_BAS  10

#define SPACE_FOR_END_LETTER  3

//sound detection threshold
#define MAX  10000
#define MIN  5000

// use of boolean
#define true	1
#define false	0

//counter of the number of samples whose value is higher than the minimum detection threshold
static int haut = 0 ;

// counter of the number of samples whose value is considered null
static int bas = 0;

//serve to save the high value after a stop to know if it was a dot or a dash
static  int sauvegarde_haut = 0;

//stop counter
static  int temps_bas = 0 ;

//high time counter
static  int temps_haut = 0;

//LUT of the 4 characters to detect, initialized to 0
static  double tab_carac[4]= {0, 0, 0,0} ;

//position in the LUT of the 4 characters
static  int numero_carac = 0 ;

//must be accessible in both threads
static int16_t max_norm_index;

//unique code that will be used to identify the letter
static int code = 0;

#define A   31
#define B   65
#define C   130
#define D   33
#define E   4
#define F   125
#define G   52
#define H   60
#define I   12
#define J   355
#define K   98
#define L   79
#define M   36
#define N   17
#define O   117
#define P   144
#define Q   295
#define R   47
#define S   28
#define T   9
#define U   93
#define V   271
#define W   112
#define X   276
#define Y   341
#define Z   84

static _Bool mot_fini = false;
static _Bool lettre_fini = false;
static _Bool rotation_on = false;
static _Bool translation_on = false;
static _Bool rotation_back_on = false;

//the maximum value given by the calculation of the unique code is 360 so we need 361 values
#define VAL_MAX_CALCUL 361

//LUT of angles in radians function of the letter multiplied by 1000 (change of scale)
#define ANG_A   1
#define ANG_B   2
#define ANG_C   3
#define ANG_D   4
#define ANG_E   5
#define ANG_F   6
#define ANG_G   7
#define ANG_H   8
#define ANG_I   9
#define ANG_J   10
#define ANG_K   11
#define ANG_L   12
#define ANG_M   13
#define ANG_N   14
#define ANG_O   15
#define ANG_P   16
#define ANG_Q   17
#define ANG_R   18
#define ANG_S   19
#define ANG_T   20
#define ANG_U   21
#define ANG_V   22
#define ANG_W   23
#define ANG_X   24
#define ANG_Y   25
#define ANG_Z   26
#define POS_INIT   0

//global table because it is a LUT
int LUT[VAL_MAX_CALCUL] = {0};

void decodage(void)
{
	if(!(mot_fini))
    {
        if (!(lettre_fini) && !(rotation_on) && !(translation_on))
        {
        	if (max_norm_index >= 350 && max_norm_index <= 450)
        	{
        		haut += 1;
        	}
        	else
        	{
        		bas += 1;
        	}
        	if (haut == NB_HAUT_PAR_TEMPS_HAUT)
        	{
        		haut = 0;
        		temps_haut += 1;

        		////chprintf((BaseSequentialStream *) &SDU1, "nmbre de temps haut= ");
				////chprintf((BaseSequentialStream *) &SDU1,"%d",temps_haut);
				////chprintf((BaseSequentialStream *) &SDU1, "  ");

        	}
        	if ((temps_haut == 1) && (haut ==0))
        	{
        		bas = 0;
        		temps_bas = 0;
        	}
        	//we spent too much time at zero value
        	if (bas == NB_BAS_PAR_TEMPS_BAS)
        	{
        		bas = 0;
        		temps_bas +=1;

                ////chprintf((BaseSequentialStream *) &SDU1, "nmbre de temps bas= ");
                ////chprintf((BaseSequentialStream *) &SDU1,"%d",temps_bas);
                ////chprintf((BaseSequentialStream *) &SDU1, "  ");

        	}
        	if ((temps_bas == 1) && (bas ==0))
        	{
        		sauvegarde_haut = haut + temps_haut*NB_HAUT_PAR_TEMPS_HAUT;

                ////chprintf((BaseSequentialStream *) &SDU1, "sauvegarde haut = ");
                ////chprintf((BaseSequentialStream *) &SDU1,"%d",sauvegarde_haut);
                ////chprintf((BaseSequentialStream *) &SDU1, "  ");
        	}
        	if ((temps_haut==1*NB_HAUT_PAR_DOT/NB_HAUT_PAR_TEMPS_HAUT) || (temps_haut==2*NB_HAUT_PAR_DOT/NB_HAUT_PAR_TEMPS_HAUT) || (temps_haut==3*NB_HAUT_PAR_DOT/NB_HAUT_PAR_TEMPS_HAUT))
        	{
        		set_body_led(1);
        	}
        	else
        	{
        		set_body_led(0);
        	}
        	if ((temps_bas==1*NB_BAS_PAR_SPACE/NB_BAS_PAR_TEMPS_BAS) || (temps_bas==2*NB_BAS_PAR_SPACE/NB_BAS_PAR_TEMPS_BAS) || (temps_bas==3*NB_BAS_PAR_SPACE/NB_BAS_PAR_TEMPS_BAS))
        	{
				set_led(LED7,1);
				set_led(LED5,1);
				set_led(LED3,1);
				set_led(LED1,1);
        	}
            else
            {
				set_led(LED7,0);
				set_led(LED5,0);
				set_led(LED3,0);
				set_led(LED1,0);
            }
        	//the time spent at the bottom represents a space and then we are back up
        	if ((temps_bas*NB_BAS_PAR_TEMPS_BAS == NB_BAS_PAR_SPACE) && (bas == 0))
        	{
				if ((NB_HAUT_PAR_DOT - MARGE <= sauvegarde_haut) && (sauvegarde_haut <= NB_HAUT_PAR_DOT + MARGE))
				{
					tab_carac[numero_carac] = DOT ;
					numero_carac ++ ;
					haut = 0;
					sauvegarde_haut= 0;
					temps_haut = 0;
				}
				//the backup corresponds to a dash
				if ((NB_HAUT_PAR_DASH - MARGE <= sauvegarde_haut) && (sauvegarde_haut <= NB_HAUT_PAR_DASH + MARGE))
				{
					tab_carac[numero_carac] = DASH;
					numero_carac ++ ;
					haut = 0;
					sauvegarde_haut= 0;
					temps_haut = 0;
				}
				//we have recorded 4 characters or the time spent at the bottom represents 3 spaces and we have gone up
				if ((numero_carac == 4) || ((temps_bas*NB_BAS_PAR_TEMPS_BAS == NB_HAUT_PAR_DASH) && (haut > NB_HAUT_PAR_TEMPS_HAUT)))
				{
					lettre_fini= true;
				}
        	}
        	//we spent more than the time between two letters down, it means that there is no more new letter approaching
        	//so it was the last letter of the word and it is finished
        	if (temps_bas*NB_BAS_PAR_TEMPS_BAS > NB_BAS_PAR_SPACE_LETTER)
        	{
        		lettre_fini = true;
        		mot_fini = true;
        	}
        }
        if (lettre_fini && !(rotation_on) && !(translation_on))
        {
        	 //ensure that the LUT is not just filled with zeros
        	if (tab_carac[0] != 0)
            {

        		//calculate the code of the sent letter
        		code = tab_carac[0]*tab_carac[0] + tab_carac[1]*tab_carac[1]*tab_carac[1] + tab_carac[2]*tab_carac[2]*tab_carac[2]*tab_carac[2]+ tab_carac[3]*tab_carac[3]*tab_carac[3]*tab_carac[3]*tab_carac[3];
        		rotation_on = true;

        		////chprintf((BaseSequentialStream *) &SDU1, "tab_carac[0]= ");
				////chprintf((BaseSequentialStream *) &SDU1,"%f",tab_carac[0]);
				////chprintf((BaseSequentialStream *) &SDU1, "   ");

				//chprintf((BaseSequentialStream *) &SDU1, "tab_carac[1]=  ");
				//chprintf((BaseSequentialStream *) &SDU1,"%f",tab_carac[1]);
				//chprintf((BaseSequentialStream *) &SDU1, "   ");

				//chprintf((BaseSequentialStream *) &SDU1, "tab_carac[2]= ");
				//chprintf((BaseSequentialStream *) &SDU1,"%f",tab_carac[2]);
				//chprintf((BaseSequentialStream *) &SDU1, "   ");

				//chprintf((BaseSequentialStream *) &SDU1, "tab_carac[3]=  ");
				//chprintf((BaseSequentialStream *) &SDU1,"%f",tab_carac[3]);
				//chprintf((BaseSequentialStream *) &SDU1, "   ");



                //chprintf((BaseSequentialStream *) &SDU1, "Voici le code     : ");
                //chprintf((BaseSequentialStream *) &SDU1, "%d", code);
                //chprintf((BaseSequentialStream *) &SDU1, "     ");

            }
            else
            {
            	code = 0;
            }

			//reset
			bas = 0;
			haut = 0;
			temps_bas = 0;
			temps_haut = 0;
			tab_carac[0] = 0;
			tab_carac[1]= 0;
			tab_carac[2]= 0;
			tab_carac[3]= 0;
			numero_carac = 0;
			mot_fini = false;
			sauvegarde_haut = 0;
        }
        if ((rotation_on) && ((code<=360) && (code >= 0)))
		{
        	//we fill the LUT with the angles corresponding to each letter
        	LUT[A] = ANG_A;
			LUT[B] = ANG_B;
			LUT[C] = ANG_C;
			LUT[D] = ANG_D;
			LUT[E] = ANG_E;
			LUT[F] = ANG_F;
			LUT[G] = ANG_G;
			LUT[H] = ANG_H;
			LUT[I] = ANG_I;
			LUT[J] = ANG_J;
			LUT[K] = ANG_K;
			LUT[L] = ANG_L;
			LUT[M] = ANG_M;
			LUT[N] = ANG_N;
			LUT[O] = ANG_O;
			LUT[P] = ANG_P;
			LUT[Q] = ANG_Q;
			LUT[R] = ANG_R;
			LUT[S] = ANG_S;
			LUT[T] = ANG_T;
			LUT[U] = ANG_U;
			LUT[V] = ANG_V;
			LUT[W] = ANG_W;
			LUT[X] = ANG_X;
			LUT[Y] = ANG_Y;
			LUT[Z] = ANG_Z;

			//the robot is oriented by an angle according to the letter collected
			orientation_robot(LUT[code]);

			translation_on = true;
            rotation_on = false;

			left_motor_set_pos(0);
			right_motor_set_pos(0);
		}
        if (translation_on)
        {
        	translation_on = avance_to_obstacle();
        	if (!(translation_on))
        	{
        		rotation_back_on = true;
        	}
        }
        else
        {
        	lettre_fini = false;
        }
        if (rotation_back_on)
		{
        	orientation_robot_back();
			rotation_back_on = false;
			rotation_on = false;

			//reset
			bas = 0;
			haut = 0;
			temps_bas = 0;
			temps_haut = 0;
			tab_carac[0] = 0;
			tab_carac[1]= 0;
			tab_carac[2]= 0;
			tab_carac[3]= 0;
			numero_carac = 0;
			mot_fini = false;
			sauvegarde_haut = 0;
			code =0;
		}
    }
}


//declaration of the decoding thread
static THD_WORKING_AREA(waDecodage, 1024);
static THD_FUNCTION(Decodage,arg)
{
	chRegSetThreadName(__FUNCTION__);
    (void)arg;
    systime_t time;

    while(1)
    {
    	time = chVTGetSystemTime();
        decodage();
        //Suspends the invoking thread until the system time arrives to the specified value
        chThdSleepUntilWindowed(time, time + MS2ST(50));
    }
}


void decodage_start(void)
{
	chThdCreateStatic(waDecodage, sizeof(waDecodage), NORMALPRIO +1, Decodage, NULL);
}



/*
*	Simple function used to detect the highest value in a buffer
*
*/
void sound_remote(float* data)
{
	float max_norm = MIN_VALUE_THRESHOLD;
	max_norm_index = -1;

	//search for the highest peak
	for(uint16_t i = MIN_FREQ ; i <= MAX_FREQ ; i++)
	{
		if(data[i] > max_norm)
		{
			max_norm = data[i];
			max_norm_index = i;
		}
	}
}


/*
*	Callback called when the demodulation of the four microphones is done.
*	We get 160 samples per mic every 10ms (16kHz)
*	
*	params :
*	int16_t *data			Buffer containing 4 times 160 samples. the samples are sorted by micro
*							so we have [micRight1, micLeft1, micBack1, micFront1, micRight2, etc...]
*	uint16_t num_samples	Tells how many data we get in total (should always be 640)
*/
void processAudioData(int16_t *data, uint16_t num_samples){

	/*
	*
	*	We get 160 samples per mic every 10ms
	*	So we fill the samples buffers to reach
	*	1024 samples, then we compute the FFTs.
	*
	*/

	static uint16_t nb_samples = 0;
	static uint8_t mustSend = 0;

	//loop to fill the buffers
	for(uint16_t i = 0 ; i < num_samples ; i+=4){
		//construct an array of complex numbers. Put 0 to the imaginary part
		micRight_cmplx_input[nb_samples] = (float)data[i + MIC_RIGHT];
		micLeft_cmplx_input[nb_samples] = (float)data[i + MIC_LEFT];
		micBack_cmplx_input[nb_samples] = (float)data[i + MIC_BACK];
		micFront_cmplx_input[nb_samples] = (float)data[i + MIC_FRONT];

		nb_samples++;

		micRight_cmplx_input[nb_samples] = 0;
		micLeft_cmplx_input[nb_samples] = 0;
		micBack_cmplx_input[nb_samples] = 0;
		micFront_cmplx_input[nb_samples] = 0;

		nb_samples++;

		//stop when buffer is full
		if(nb_samples >= (2 * FFT_SIZE)){
			break;
		}
	}

	if(nb_samples >= (2 * FFT_SIZE)){
		/*	FFT proccessing
		*
		*	This FFT function stores the results in the input buffer given.
		*	This is an "In Place" function. 
		*/

		doFFT_optimized(FFT_SIZE, micRight_cmplx_input);
		doFFT_optimized(FFT_SIZE, micLeft_cmplx_input);
		doFFT_optimized(FFT_SIZE, micFront_cmplx_input);
		doFFT_optimized(FFT_SIZE, micBack_cmplx_input);

		/*	Magnitude processing
		*
		*	Computes the magnitude of the complex numbers and
		*	stores them in a buffer of FFT_SIZE because it only contains
		*	real numbers.
		*
		*/
		arm_cmplx_mag_f32(micRight_cmplx_input, micRight_output, FFT_SIZE);
		arm_cmplx_mag_f32(micLeft_cmplx_input, micLeft_output, FFT_SIZE);
		arm_cmplx_mag_f32(micFront_cmplx_input, micFront_output, FFT_SIZE);
		arm_cmplx_mag_f32(micBack_cmplx_input, micBack_output, FFT_SIZE);

		//sends only one FFT result over 10 for 1 mic to not flood the computer
		//sends to UART3
		if(mustSend > 8){
			//signals to send the result to the computer
			chBSemSignal(&sendToComputer_sem);
			mustSend = 0;
		}
		nb_samples = 0;
		mustSend++;
		sound_remote(micLeft_output);

	}
}


void wait_send_to_computer(void){
	chBSemWait(&sendToComputer_sem);
}


float* get_audio_buffer_ptr(BUFFER_NAME_t name){
	if(name == LEFT_CMPLX_INPUT){
		return micLeft_cmplx_input;
	}
	else if (name == RIGHT_CMPLX_INPUT){
		return micRight_cmplx_input;
	}
	else if (name == FRONT_CMPLX_INPUT){
		return micFront_cmplx_input;
	}
	else if (name == BACK_CMPLX_INPUT){
		return micBack_cmplx_input;
	}
	else if (name == LEFT_OUTPUT){
		return micLeft_output;
	}
	else if (name == RIGHT_OUTPUT){
		return micRight_output;
	}
	else if (name == FRONT_OUTPUT){
		return micFront_output;
	}
	else if (name == BACK_OUTPUT){
		return micBack_output;
	}
	else{
		return NULL;
	}
}
