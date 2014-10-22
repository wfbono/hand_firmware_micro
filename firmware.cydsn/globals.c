// ----------------------------------------------------------------------------
// Copyright (C)  qbrobotics. All rights reserved.
// www.qbrobotics.com
// ----------------------------------------------------------------------------

/** 
* \file         globals.c
*
* \brief        Global variables.
* \date         Feb 06, 2012
* \author       qbrobotics
* \copyright    (C)  qbrobotics. All rights reserved.
*/

//=================================================================     includes
#include <globals.h>

//=============================================      global variables definition


struct st_ref   g_ref;                  // motor variables
struct st_meas  g_meas;                 // measurements
struct st_data  g_rx;                   // income data
struct st_mem   g_mem, c_mem;           // memory
struct st_dev   device;                 // device related variables
struct st_calib calib;

int32 opened_hand_pos;
int32 closed_hand_pos;
int8 dx_sx_hand;            //-1 dx, 1 sx

float tau_feedback;

uint16 timer_value;

/* END OF FILE */