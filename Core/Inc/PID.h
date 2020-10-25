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

/*##### TO BE DEFINED #####*/
/**
 * PID_FREQUENCY - frequency od PID (Hz) - TODO: move, used only for angle
 * 
 * ANGLE_CORRECTION - angle adjustment per cycle when standind
 * 
 * ACC_PART - accelerometer part in angle calculation
 * 
 * MOUNT_ERROR - mounting error set at startup
 */
/*#########################*/

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
	void (*reset) (PID_t *public);

	/**
	 * Set desired signal
	 */
	void (*set_desired_signal) (PID_t *public, float desired_signal);

	/**
	 * Do next time step
	 */
	void (*tic) (PID_t *public, float input_signal);

	/**
	 * Get output
	 */
	float (*get_output) (PID_t *public);

	/**
	 * Get output smooth
	 */
	float (*get_output_smooth) (PID_t *public);
	/**
	 * Set new PID coefs
	 */
	void (*set_PID_coefs) (PID_t *public, PID_coefs_t coefs);
	/**
	 * Get PID coefs
	 */
	PID_coefs_t (*get_PID_coefs) (PID_t *public);
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
