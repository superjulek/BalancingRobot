/*
 * config.h
 *
 *  Created on: Oct 24, 2020
 *      Author: juliusz
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

/* Global defines */

#define CLIMBING_MODE // more aggressive steering

/* Steering defines */
#define INIT_SPEED_KP 0.022
#define INIT_SPEED_KI 0.03
#define INIT_SPEED_KD 0.05

#define INIT_ANGLE_KP 14500.
#define INIT_ANGLE_KI 75000.
#define INIT_ANGLE_KD 5.

#ifndef CLIMBING_MODE
#define MAX_STEERING_ANGLE 4000. // in mdeg
#else
#define MAX_STEERING_ANGLE 30000.
#endif

#define ANGLE_PID_DEADBAND 00. // in mrpm
#define SPEED_PID_DEADBAND 0.0 // in mdeg

#define SPEED_PID_DIVIDER 1000. // to operate in mdeg

#define SPEED_PID_DIFF_AVERAGE_COEF 0.6
#define ANGLE_PID_DIFF_AVERAGE_COEF 0.96

#define SPEED_PID_MAX_CHANGE 375000
#define ANGLE_PID_MAX_CHANGE 0 // no ramp for angle pid

/* Define below "TO BE DEFINED" from specific .h files */

/*#### general config ####*/
#define MAX_ANGLE_STANDING 0.7

#define MAX_STANDING_SPEED 2000

#ifndef CLIMBING_MODE
#define MAX_ANGLE 8
#else
#define MAX_ANGLE 40
#endif

#define MAX_TURNING_SPEED 75000
#define TURNING_SPEED 25000

#define MAX_DRIVING_SPEED 250000
#define DRIVING_SPEED 50000
/*########################*/


/*##### PID.h config #####*/
#define MOUNT_ANGLE_CORRECTION 0.0004

#define MOUNT_ERROR -1.0
/*########################*/


/*##### MPU.h config #####*/
#define ACC_PART 0.0007

#define I2C_TIMEOUT 1

#define CALIBRATION_ROUNDS 2000

#define GYRO_Z_AXIS_ERROR -0.017
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
#define ACCELERATION 25000

#define RAMP_FREQUENCY 100

#define CLOCK_FREQUENCY 6000000

#define STEPS_PER_REVOLUTION 200

#define MAX_SPEED 400000

#define MAX_STEPPING_INTERVAL 4000
/*########################*/

/*## bluetooth_communicator.h config ##*/
#define MESSAGE_BUFF_SIZE 20
/*########################*/



#endif /* INC_CONFIG_H_ */
