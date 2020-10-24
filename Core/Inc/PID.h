/*
 * PID.h
 *
 *  Created on: Sep 20, 2020
 *      Author: juliusz
 */

#ifndef INC_PID_H_
#define INC_PID_H_


#include "main.h"
#include "general.h"
#include "mpu6050.h"

#define PID_FREQUENCY 250

#define KP 50000.
#define KI 00.
#define KD 000.

#define DEAD_ANGLE 0.1

#define ANGLE_CORRECTION 0.002

#define MAX_ANGLE_STANDING 0.7//

#define MAX_STANDING_SPEED 4000//

#define ACC_PART 0.004

#define DELAY_COEF 0.09

#define MOUNT_ERROR -1.0

#define MAX_ANGLE 7

#define MAX_MOUNT_ANGLE_CORECTION_OUTPUT 1200
#define MOUNT_ANGLE_CORECTION 0.003

typedef struct PID_coefs_t PID_coefs_t;
typedef struct PID_t PID_t;

struct PID_coefs_t {
	float KP_coef;
	float KI_coef;
	float KD_coef;
};

struct PID_t {

	/**
	 * Reset PID
	 */
	void (*reset) (PID_t *this);

	/**
	 * Set desired signal
	 */
	void (*set_desired_signal) (PID_t *this, float desired_signal);

	/**
	 * Do next time step
	 */
	void (*tic) (PID_t *this, float input_signal);

	/**
	 * Get output
	 */
	int32_t (*get_output) (PID_t *this);

	/**
	 * Get output smooth
	 */
	int32_t (*get_output_smooth) (PID_t *this);

	/**
	 * Live - tune PID coefs
	 */
	PID_coefs_t (*live_tune) (PID_t *this, PID_coefs_t coefs);
};

/**
 * Create PID instance
 */
PID_t* PID_create(PID_coefs_t coefs, float dead_band, float max_output_signal, uint16_t frequency, float delay_coef);

// Get current robot angle
float get_angle(void);

float get_angle_acc (void);

float get_new_angle (float last_angle);

// Fix angle to keep robot in balance
float follow_angle(int32_t speed, float desired_angle);

#endif /* INC_PID_H_ */
