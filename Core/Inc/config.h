/*
 * config.h
 *
 *  Created on: Oct 24, 2020
 *      Author: juliusz
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

/* Steering defines */
#define INIT_SPEED_KP 0.025
#define INIT_SPEED_KI 0.06
#define INIT_SPEED_KD 0.0002

#define INIT_ANGLE_KP 23000.
#define INIT_ANGLE_KI 190000.
#define INIT_ANGLE_KD 20.

#define MAX_STEERING_ANGLE 3500. // in mdeg

#define ANGLE_PID_DEADBAND 000. // in mrpm
#define SPEED_PID_DEADBAND 0.1 // in mdeg

#define SPEED_PID_DIVIDER 1000.

#define SPEED_PID_DIFF_AVERAGE_COEF 0.55
#define ANGLE_PID_DIFF_AVERAGE_COEF 0.95

/* Define below "TO BE DEFINED" from specific .h files */

/*#### general config ####*/
#define MAX_ANGLE_STANDING 0.7

#define MAX_STANDING_SPEED 4000

#define MAX_ANGLE 8

#define MAX_TURNING_SPEED 150000
#define TURNING_SPEED 50000

#define MAX_DRIVING_SPEED 500000
#define DRIVING_SPEED 100000
/*########################*/


/*##### PID.h config #####*/
#define ANGLE_CORRECTION 0.005

#define MOUNT_ERROR -1.0

#define DESIRED_SIGNAL_SMOOTHING 0.1
/*########################*/


/*##### MPU.h config #####*/
#define ACC_PART 0.005

#define I2C_TIMEOUT 1

#define CALIBRATION_ROUNDS 2000
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

#define MAX_SPEED 800000

#define MAX_STEPPING_INTERVAL 4000
/*########################*/

/*## bluetooth_communicator.h config ##*/
#define MESSAGE_BUFF_SIZE 20
/*########################*/



#endif /* INC_CONFIG_H_ */
