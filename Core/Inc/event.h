/*
 * event.h
 *
 *  Created on: Oct 3, 2020
 *      Author: juliusz
 */

#ifndef INC_EVENT_H_
#define INC_EVENT_H_

#include "main.h"

/*##### TO BE DEFINED #####*/
/**
 * 
 * MAX_ANGLE_STANDING - max angle when standing
 * 
 * MAX_STANDING_SPEED - max speed when standing
 * 
 * MAX_ANGLE - max angle before emergency stop
 * 
 * TURNING_SPEED - initial turning speed
 * 
 * DRIVING_SPEED - initial driving speed
 */
/*#########################*/

typedef struct event_t event_t;

struct event_t {

	/* Event callback function */
	void (*action) (void);
};


event_t flash_LED;

event_t measure_angle;

event_t wait_for_angle;

event_t left_ramp;

event_t right_ramp;

event_t send_telemetry;

event_t angle_PID_tic;

event_t movement_control_tic;

event_t restart;

event_t process_rbuf;

event_t begin_waiting;

event_t reset_everything;

event_t launch;

event_t stop;

event_t restart_MPU;

event_t emergency_check;

#endif /* INC_EVENT_H_ */
