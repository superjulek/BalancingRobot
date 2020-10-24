/*
 * config.h
 *
 *  Created on: Oct 24, 2020
 *      Author: juliusz
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

/* Define below "TO BE DEFINED" from specific .h files */


/*#### general config ####*/
#define MAX_ANGLE_STANDING 0.7

#define MAX_STANDING_SPEED 4000

#define MAX_ANGLE 7

#define TURNING_SPEED 25000

#define DRIVING_SPEED 40000
/*########################*/


/*##### PID.h config #####*/
#define PID_FREQUENCY 250

#define ANGLE_CORRECTION 0.002

#define ACC_PART 0.004

#define MOUNT_ERROR -1.0
/*########################*/


/*##### MPU.h config #####*/
/*########################*/

/*#### event.h config ####*/
/**
 * In general config
 */
/*########################*/

/*## scheduler.h config ##*/
#define QUEUE_SIZE 50
/*########################*/

/*### stepper.h config ###*/
#define ACCELERATION 60000

#define RAMP_FREQUENCY 100

#define CLOCK_FREQUENCY 12000000

#define STEPS_PER_REVOLUTION 200

#define MAX_SPEED 400000

#define MAX_STEPPING_INTERVAL 4000
/*########################*/



#endif /* INC_CONFIG_H_ */
