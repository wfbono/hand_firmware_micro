// ----------------------------------------------------------------------------
// Copyright (C)  qbrobotics. All rights reserved.
// www.qbrobotics.com
// ----------------------------------------------------------------------------

/** 
* \file 		interruptions.h
*
* \brief 		Interruptions header file.
* \date 		Feb 06, 2012
* \author 		qbrobotics
* \copyright	(C)  qbrobotics. All rights reserved.
*/

#ifndef INTERRUPTIONS_H_INCLUDED
#define INTERRUPTIONS_H_INCLUDED
// ----------------------------------------------------------------------------

//====================================================     interrupt declaration
CY_ISR_PROTO(ISR_RS485_RX_ExInterrupt);


//=====================================================     function declaration
void analog_measurements(void);
void encoder_reading(void);
void motor_control(void);
void function_scheduler(void);
void overcurrent_control(void);



// ----------------------------------------------------------------------------
#endif

/* [] END OF FILE */