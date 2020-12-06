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
 * ANGLE_CORRECTION - angle adjustment per cycle when standind
 * 
 * ACC_PART - accelerometer part in angle calculation
 * 
 * MOUNT_ERROR - mounting error set at startup
 * 
 * DESIRED_SIGNAL_SMOOTHING - coef of smoothing desired signal
 */
/*#########################*/

#define MAX_MOUNT_ANGLE_CORECTION_OUTPUT 7000

typedef struct PID_coefs_t PID_coefs_t;
typedef struct PID_t PID_t;

struct PID_coefs_t
{
	float KP_coef;
	float KI_coef;
	float KD_coef;
};

struct PID_t
{

	/**
	 * Reset PID
	 */
	void (*reset)(PID_t *public);

	/**
	 * Set desired signal
	 */
	void (*set_desired_signal)(PID_t *public, float desired_signal);

	/**
	 * Get desired signal
	 */
	float (*get_desired_signal)(PID_t *public);

	/**
	 * Do next time step
	 */
	void (*tic)(PID_t *public, float input_signal);

	/**
	 * Get output
	 */
	float (*get_output)(PID_t *public);

	/**
	 * Set new PID coefs
	 */
	void (*set_PID_coefs)(PID_t *public, PID_coefs_t coefs);

	/**
	 * Get PID coefs
	 */
	PID_coefs_t (*get_PID_coefs)(PID_t *public);

	/**
	 * Set new desired signal to be ramped
	 */
	void (*set_desired_signal_to_ramp)(PID_t *public, float desired_signal_to_ramp);

	/**
	 * Get new desired signal to be ramped
	 */
	float (*get_desired_signal_to_ramp)(PID_t *public);

	/**
	 * Do ramp
	 */
	void (*ramp)(PID_t *public);
};

/**
 * Create PID instance
 */
PID_t *PID_create(PID_coefs_t coefs, float dead_band, float max_output_signal, uint16_t frequency, float average_coef, float max_change);

#endif /* INC_PID_H_ */
