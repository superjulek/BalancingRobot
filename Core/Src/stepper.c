/*
 * stepper.c
 *
 *  Created on: Aug 7, 2020
 *      Author: juliusz
 */

#include "stepper.h"
#include "config.h"

typedef struct private_stepper_t private_stepper_t;

struct private_stepper_t
{
	/**
	 * Public member
	 */
	stepper_t public;

	stepper_pins_t pins;

	int32_t desired_speed;

	int32_t actual_speed;

	TIM_HandleTypeDef *timer;

	uint8_t channel;

	microstepping_t microstepping;

	/* Max change in speed between ramps */
	int32_t max_delta_speed;

	uint8_t active;

	int8_t reverse_direction;
};

uint32_t max_autoreload = MAX_STEPPING_INTERVAL * CLOCK_FREQUENCY / 1000000;

static void change_speed(private_stepper_t *this, int32_t speed)
{
	uint32_t autoreload;
	this->actual_speed = speed;
	if (speed * this->reverse_direction > 0)
	{
		write_pin(this->pins.DIR, 1);
	}
	else
	{
		write_pin(this->pins.DIR, 0);
	}
	if (speed != 0)
	{
		autoreload = (int32_t)(CLOCK_FREQUENCY * 30) / ((int32_t)this->microstepping * STEPS_PER_REVOLUTION) * 1000 / abs(speed);
		if (autoreload > max_autoreload)
		{
			//Nie potrzeba przy dużym microsteppingu
			//autoreload = 999999;
		}
	}
	else
	{
		return;
	}
	this->timer->Instance->ARR = autoreload;
}

static void start(stepper_t *public)
{
	private_stepper_t *this = (private_stepper_t *)public;
	write_pin(this->pins.ENABLE, 0);
	this->desired_speed = 0;
	this->actual_speed = 0;
	this->active = 1;
	HAL_TIM_OC_Start(this->timer, this->channel);
}

static void set_microstepping(stepper_t *public, microstepping_t microstepping)
{
	private_stepper_t *this = (private_stepper_t *)public;
	uint8_t microstepping_bits;
	switch (microstepping)
	{
	case STEPS_FULL:
		microstepping_bits = microstepping_table[0];
		break;
	case STEPS_2:
		microstepping_bits = microstepping_table[1];
		break;
	case STEPS_4:
		microstepping_bits = microstepping_table[2];
		break;
	case STEPS_8:
		microstepping_bits = microstepping_table[3];
		break;
	case STEPS_16:
		microstepping_bits = microstepping_table[4];
		break;
	case STEPS_32:
		microstepping_bits = microstepping_table[5];
		break;
	case STEPS_64:
		microstepping_bits = microstepping_table[6];
		break;
	case STEPS_128:
		microstepping_bits = microstepping_table[7];
		break;
	}

	if (microstepping_bits == MICROSTEPPING_NOT_SUPPORTED)
	{
		microstepping_bits = microstepping_table[0];
		microstepping = STEPS_FULL;
	}

	write_pin(this->pins.MS1, microstepping_bits & 0b100);
	write_pin(this->pins.MS2, microstepping_bits & 0b010);
	write_pin(this->pins.MS3, microstepping_bits & 0b001);
	this->microstepping = microstepping;
}

static void set_speed(stepper_t *public, int32_t speed)
{
	private_stepper_t *this = (private_stepper_t *)public;
	if (speed > MAX_SPEED)
	{
		speed = MAX_SPEED;
	}
	else if (speed < -MAX_SPEED)
	{
		speed = -MAX_SPEED;
	}
	this->desired_speed = speed;
}

static void ramp(stepper_t *public)
{
	private_stepper_t *this = (private_stepper_t *)public;
	if (this->active == 1)
	{
		int32_t diff = this->desired_speed - this->actual_speed;
		if (diff > this->max_delta_speed)
		{
			change_speed(this, this->actual_speed + this->max_delta_speed);
		}
		else if (diff < -this->max_delta_speed)
		{
			change_speed(this, this->actual_speed - this->max_delta_speed);
		}
		else
		{
			change_speed(this, this->desired_speed);
		}
	}
}

static void stop(stepper_t *public)
{
	private_stepper_t *this = (private_stepper_t *)public;
	write_pin(this->pins.ENABLE, 1);
	this->desired_speed = 0;
	this->actual_speed = 0;
	this->active = 0;
	HAL_TIM_OC_Stop(this->timer, this->channel);
}

static int32_t get_actual_speed(stepper_t *public)
{
	private_stepper_t *this = (private_stepper_t *)public;
	return this->actual_speed;
}

stepper_t *stepper_create(stepper_pins_t pins, bool reverse_direction, TIM_HandleTypeDef *timer, uint32_t channel)
{
	private_stepper_t *this = malloc(sizeof(private_stepper_t));
	*this = (private_stepper_t){
		.public = {
			.start = start,
			.set_microstepping = set_microstepping,
			.set_speed = set_speed,
			.ramp = ramp,
			.stop = stop,
			.get_actual_speed = get_actual_speed,
		},
		.pins = pins,
		.active = 0,
		.max_delta_speed = ACCELERATION * 60 / RAMP_FREQUENCY,
		.reverse_direction = reverse_direction ? -1 : 1,
		.timer = timer,
		.channel = channel,
	};

	this->public.set_microstepping(&(this->public), STEPS_FULL);

	return &(this->public);
}
