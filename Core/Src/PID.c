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

struct private_PID_t
{
	/**
	 * Public member
	 */
	PID_t public;

	float desired_signal;

	float desired_signal_smooth;

	// First in table is most recent value
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
static void put_in_history(private_PID_t *this, float diff)
{
	for (int i = HISTORY_SIZE - 1; i > 0; --i)
	{
		this->previous_diffs[i] = this->previous_diffs[i - 1];
	}
	this->previous_diffs[0] = diff;
}

static float get_derivative(private_PID_t *this)
{
	float derivative = (11. / 6. * this->previous_diffs[0] - 3. * this->previous_diffs[1] +
						3. / 2. * this->previous_diffs[2] - 1. / 3. * this->previous_diffs[3]) *
					   (float)this->frequency;
	return derivative;
}

static void reset(PID_t *public)
{
	private_PID_t *this = (private_PID_t *)public;
	this->integral_sum = 0;
	for (int i = 0; i < HISTORY_SIZE; ++i)
	{
		put_in_history(this, 0);
	}
	this->previous_output_signal = 0;
	this->output_signal = 0;
	this->desired_signal = 0;
	this->desired_signal_smooth = 0;
}

static void set_desired_signal(PID_t *public, float desired_signal)
{
	private_PID_t *this = (private_PID_t *)public;
	this->desired_signal = desired_signal;
}

static float get_desired_signal(PID_t *public)
{
	private_PID_t *this = (private_PID_t *)public;
	return this->desired_signal;
}

static void tic(PID_t *public, float input_signal)
{
	private_PID_t *this = (private_PID_t *)public;
	this->desired_signal_smooth = this->desired_signal * (1 - DESIRED_SIGNAL_SMOOTHING) + DESIRED_SIGNAL_SMOOTHING * this->desired_signal_smooth;
	float diff = input_signal - this->desired_signal_smooth;
	put_in_history(this, input_signal);
	float integral_sum = this->integral_sum + diff / (float)this->frequency;
	this->previous_output_signal = this->output_signal;
	float output_signal = this->coefs.KP_coef * diff +
						  this->coefs.KI_coef * integral_sum +
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

static float get_output(PID_t *public)
{
	private_PID_t *this = (private_PID_t *)public;
	return this->output_signal;
}

static float get_output_smooth(PID_t *public)
{
	private_PID_t *this = (private_PID_t *)public;
	return (this->output_signal * (1.0 - this->delay_coef) +
			this->previous_output_signal * this->delay_coef);
}

static void set_PID_coefs(PID_t *public, PID_coefs_t coefs)
{
	private_PID_t *this = (private_PID_t *)public;
	this->coefs = coefs;
}

static PID_coefs_t get_PID_coefs(PID_t *public, PID_coefs_t coefs)
{
	private_PID_t *this = (private_PID_t *)public;
	return this->coefs;
}

PID_t *PID_create(PID_coefs_t coefs, float dead_band, float max_output_signal, uint16_t frequency, float delay_coef)
{
	private_PID_t *this = malloc(sizeof(private_PID_t));
	*this = (private_PID_t){
		.public = {
			.reset = reset,
			.set_desired_signal = set_desired_signal,
			.get_desired_signal = get_desired_signal,
			.tic = tic,
			.get_output = get_output,
			.get_output_smooth = get_output_smooth,
			.set_PID_coefs = set_PID_coefs,
			.get_PID_coefs = get_PID_coefs,
		},
		.coefs = coefs,
		.dead_band = dead_band,
		.max_output_signal = max_output_signal,
		.frequency = frequency,
		.delay_coef = delay_coef};

	this->public.reset(&(this->public));

	return &(this->public);
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
