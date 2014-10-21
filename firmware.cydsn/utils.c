// -----------------------------------------------------------------------------
// Copyright (C)  qbrobotics. All rights reserved.
// www.qbrobotics.com
// -----------------------------------------------------------------------------

/** 
* \file 		utils.h
*
* \brief 		Definition of utility functions.
* \date 		Feb 16, 2014
* \author 		qbrobotics
* \copyright	(C)  qbrobotics. All rights reserved.
*/

#include <utils.h>
#include <math.h>

//--------------------------------------------------------------     DEFINITIONS

#define ALPHA 10
#define BETA  50

#define SIGN(A) (((A) > 0) ? (1) : ((((A) < 0) ? (-1) : (0))))


int32 filter_v(int32 new_value) {

	static int32 old_value, aux;

	aux = (old_value * (1024 - BETA) + new_value * (BETA)) / 1024;

	old_value = aux;

	return aux;
}


int32 filter_ch1(int32 new_value) {

	static int32 old_value, aux;

	aux = (old_value * (1024 - ALPHA) + new_value * (ALPHA)) / 1024;

	old_value = new_value;

	return aux;
}

int32 filter_ch2(int32 new_value) {

	static int32 old_value, aux;

	aux = (old_value * (1024 - ALPHA) + new_value * (ALPHA)) / 1024;

	old_value = new_value;

	return aux;
}

int32 filter_i1(int32 new_value) {

	static int32 old_value, aux;

	aux = (old_value * (1024 - BETA) + new_value * (BETA)) / 1024;

	old_value = aux;

	return aux;
}

int32 filter_i2(int32 new_value) {

	static int32 old_value, aux;

	aux = (old_value * (1024 - BETA) + new_value * (BETA)) / 1024;

	old_value = aux;

	return aux;
}


//==============================================================================
//																	BIT CHECKSUM
//==============================================================================


uint8 BITChecksum(uint32 mydata) {
	uint8 i;
	uint8 checksum = 0;
	for(i = 0; i < 31; ++i)
	{
       	checksum = checksum ^ (mydata & 1);
		mydata = mydata >> 1;
	}
	return checksum;
}

//==============================================================================
//                                                                ROUND_FUNCTION
//==============================================================================

int round(double x) {

    if (x < 0.0)
        return (int)(x - 0.5);
    else
        return (int)(x + 0.5);
}

//==============================================================================
//                                                                        MODULE
//==============================================================================

int32 my_mod(int32 val, int32 divisor) {

	if (val >= 0) {
		return (int32)(val % divisor);
	} else {
		return (int32)(divisor - (-val % divisor));
	}
}


//==============================================================================
//																	   CALIBRATE
//==============================================================================

void calibration(void) {
	static uint8 direction; //0 closing, 1 opening
	static uint8 closure_counter;


	// closing
	if (direction == 0) {
		g_ref.pos[0] += dx_sx_hand * calib.speed;
		// if (abs(g_ref.pos[0]) > closed_hand_pos) {
		// 	direction = 1;
		// }
		if ((g_ref.pos[0] * dx_sx_hand) > closed_hand_pos) {
			direction = 1;
		}
	} else { //opening
		g_ref.pos[0] -= dx_sx_hand * calib.speed;
		if (SIGN(g_ref.pos[0]) != dx_sx_hand) {
			direction = 0;
			closure_counter++;
			if (closure_counter == calib.repetitions) {
				closure_counter = 0;
				calib.enabled = FALSE;
			}
		}
	}
}


//==============================================================================
//																   HAND FEEDBACK
//==============================================================================

#define D_model_po 0.
#define K_model_po 0.0169
#define F_model_po 0.0

#define P1  -11  //-11
#define P2  9    //9
#define P3  11   //11


#define pi 3.14159265359

#define gear_ratio 83
#define gear_rat_inv (1.0 / gear_ratio)

#define enc 65536
#define inv_enc (1.0 / enc)

#define torq_cts 0.0184
#define torq_cts_inv (1.0 / torq_cts)

#define motor_inertia 0.000000555
#define vel_mult_cts (2 * pi / enc)

#define Fs 200 // your sampling frequency

#define aux_1 (-gear_ratio * Fs * vel_mult_cts)

#define BETA 20 //filter value


void torque_feedback() {

	static int32 velocity_sign;
	static int32 i_old_value, p_old_value;
	static int32 i_filtered,  p_filtered;

	static int32 tmp, old_output;



	// --- Current filter

	i_filtered = (i_old_value * (1024 - BETA) + g_meas.curr[0] * (BETA)) / 1024;

	i_old_value = i_filtered;

	// --- Position filter

	p_filtered = (p_old_value * (1024 - BETA) + (g_meas.pos[0] / 8) * (BETA)) / 1024;

	p_old_value = p_filtered;


	// --- Calculate curr velocity sign
	velocity_sign = SIGN(p_filtered - p_old_value);


	tmp = (velocity_sign + 1) * p_filtered;
	tmp /= 1024;

	//tmp = p_filtered / 512;


	i_filtered -= velocity_sign * P1;
	i_filtered -= tmp * P2;
	i_filtered -= (((tmp * p_filtered) / 1024) * P3) / 16;

	// --- Result ilter

	tau_feedback = (old_output * (1024 - BETA) + i_filtered * (BETA)) / 1024;

	old_output = tau_feedback;

}


/* [] END OF FILE */