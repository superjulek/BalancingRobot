/*
 * PID.c
 *
 *  Created on: Sep 20, 2020
 *      Author: juliusz
 */

#include "PID.h"
#include "math.h"
#include "config.h"

// Number of previous input signals held in memory
#define HISTORY_SIZE 5

extern float mount_error;

typedef struct private_PID_t private_PID_t;

struct private_PID_t {
	/**
	 * Public member
	 */
	PID_t public;

	float desired_signal;

	// First latest
	float previous_diffs[HISTORY_SIZE];

	float integral_sum;

	float output_signal;

	float max_output_signal;

	float previous_output_signal;

	float dead_band;

	float delay_coef;

	PID_coefs_t coefs;

	uint16_t frequency;
};

/**
 * Put signal in history
 */
static void put_in_history (private_PID_t *this, float diff)
{
	for (int  i = HISTORY_SIZE - 1; i > 0; --i)
	{
		this->previous_diffs[i] = this->previous_diffs[i-1];
	}
	this->previous_diffs[0] = diff;
}

static float get_derivative (private_PID_t *this)
{
	float derivative = (-11./6. * this->previous_diffs[0] + 3. * this->previous_diffs[1] -\
			3./2. * this->previous_diffs[2] + 1./3. * this->previous_diffs[3]) \
			* (float)this->frequency;
	return derivative;
}

METHOD(private_PID_t, reset, void, private_PID_t *this)
{
	this->integral_sum = 0;
	for (int i = 0; i < HISTORY_SIZE; ++i)
	{
		put_in_history(this, 0);
	}
	this->previous_output_signal = 0;
	this->output_signal = 0;
	this->desired_signal = 0;
}

METHOD(private_PID_t, set_desired_signal, void, private_PID_t *this, float desired_signal)
{
	this->desired_signal = desired_signal;
}

METHOD(private_PID_t, tic, void, private_PID_t *this, float input_signal)
{
	float diff = input_signal - this->desired_signal;
	put_in_history(this, input_signal);
	float integral_sum = this->integral_sum + diff / (float) this->frequency;
	this->previous_output_signal = this->output_signal;
	float output_signal = this->coefs.KP_coef * diff + \
			this->coefs.KI_coef * integral_sum + \
			this->coefs.KD_coef * get_derivative(this);
	if (output_signal > this->max_output_signal)
	{
		this->output_signal = this->max_output_signal;
	}
	else if (output_signal < -this->max_output_signal)
	{
		this->output_signal = -this->max_output_signal;
	}
	else if (fabs(output_signal) < this->dead_band)
	{
		this->output_signal = 0;
		this->integral_sum = integral_sum;
	}
	else
	{
		this->output_signal = output_signal;
		this->integral_sum = integral_sum;
	}
}

METHOD(private_PID_t, get_output, int32_t, private_PID_t *this)
{
	return (int32_t)this->output_signal;
}

METHOD(private_PID_t, get_output_smooth, int32_t, private_PID_t *this)
{
	return (int32_t) (this->output_signal * (1.0 - this->delay_coef) + \
			this->previous_output_signal * this->delay_coef);
}

METHOD(private_PID_t, live_tune, PID_coefs_t, private_PID_t *this, PID_coefs_t coefs)
{
	this->coefs.KP_coef *= 1 + coefs.KP_coef;
	this->coefs.KI_coef *= 1 + coefs.KI_coef;
	this->coefs.KD_coef *= 1 + coefs.KD_coef;

	return this->coefs;
}

PID_t* PID_create(PID_coefs_t coefs, float dead_band, float max_output_signal, uint16_t frequency, float delay_coef)
{
	private_PID_t *this;
	INIT(this,
			.public = {
					.reset = _reset,
					.set_desired_signal = _set_desired_signal,
					.tic = _tic,
					.get_output = _get_output,
					.get_output_smooth = _get_output_smooth,
					.live_tune = _live_tune,
			},
			.coefs = coefs,
			.dead_band = dead_band,
			.max_output_signal = max_output_signal,
			.frequency = frequency,
			.delay_coef = delay_coef);
	this->public.reset(&(this->public));

	return &this->public;
}

float get_angle(void)
{
	static float angle = MOUNT_ERROR;
	static uint8_t first_time_flag = TRUE;
	all_scaled data;
	MPU6050_GetAllScaled(&data);
	float acc_angle = ((float)data.x / (float)data.z) * 57.3;
	if (data.z == 0)
	{
		acc_angle = angle;
	}
	if (first_time_flag)
	{
		first_time_flag = FALSE;
		angle = acc_angle;
	}
	float gyro_angle = angle - data.ry * 1 / PID_FREQUENCY;
	angle = (1.0 - ACC_PART) * gyro_angle + ACC_PART * acc_angle;
	return angle  - mount_error;
}

float get_angle_acc (void)
{
	all_scaled data;
	MPU6050_GetAllScaled(&data);
	if (data.z == 0) return 90;
	float acc_angle = atan((float)data.x / (float)data.z) * 57.3 - mount_error;
	return acc_angle;
}
float get_new_angle (float last_angle)
{
	all_scaled data;
	MPU6050_GetAllScaled(&data);
	if (data.z == 0) return 90;
	float acc_angle = atan((float)data.x / (float)data.z) * 57.3 - mount_error;
	float gyro_angle = last_angle - data.ry * 1 / PID_FREQUENCY;
	return (1.0 - ACC_PART) * gyro_angle + ACC_PART * acc_angle;
}

float follow_angle(int32_t speed, float desired_angle)
{
	if (speed > 0)
	{
		return desired_angle - ANGLE_CORRECTION;
	}
	if (speed < 0)
	{
		return desired_angle + ANGLE_CORRECTION;
	}
	return desired_angle;
}
