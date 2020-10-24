/*
 * stepper.c
 *
 *  Created on: Aug 7, 2020
 *      Author: juliusz
 */

#include "stepper.h"
#include "config.h"

typedef struct private_stepper_t private_stepper_t;

struct private_stepper_t {
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

	/* Pointer for period variable */
	int32_t *period;
};

uint32_t max_autoreload = MAX_STEPPING_INTERVAL * CLOCK_FREQUENCY / 1000000;

static void change_speed (private_stepper_t *this, int32_t speed)
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
		autoreload = (int32_t) (CLOCK_FREQUENCY * 30) / ((int32_t)this->microstepping * STEPS_PER_REVOLUTION) * 1000 / abs(speed);
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
		this->timer->Instance->ARR=autoreload;
}
METHOD(private_stepper_t, start, void, private_stepper_t *this)
{
	write_pin(this->pins.ENABLE, 0);
	this->desired_speed = 0;
	this->actual_speed = 0;
	this->active = 1;
	HAL_TIM_OC_Start(this->timer, this->channel);
}

METHOD(stepper_t, set_microstepping, void, private_stepper_t *this, microstepping_t microstepping)
{
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
METHOD(stepper_t, set_speed, void, private_stepper_t *this, int32_t speed)
{
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

METHOD(private_stepper_t, ramp, void, private_stepper_t *this)
{
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


METHOD(private_stepper_t, stop, void, private_stepper_t *this)
{
	write_pin(this->pins.ENABLE, 1);
	this->desired_speed = 0;
	this->actual_speed = 0;
	*(this->period) = 0;
	this->active = 0;
	HAL_TIM_OC_Stop(this->timer, this->channel);
}

stepper_t* stepper_create(stepper_pins_t pins, int32_t *period, bool reverse_direction, TIM_HandleTypeDef *timer, uint32_t channel)
{
	private_stepper_t* this;
	INIT(this,
			.public = {
					.start = _start,
					.set_microstepping = _set_microstepping,
					.set_speed = _set_speed,
					.ramp = _ramp,
					.stop = _stop,
			},
			.pins = pins,
			.active = 0,
			.max_delta_speed = ACCELERATION * 60 / RAMP_FREQUENCY,
			.reverse_direction = reverse_direction ? -1 : 1,
			.period = period,
			.timer = timer,
			.channel = channel,
			);

	this->public.set_microstepping(&(this->public), STEPS_FULL);
	*(this->period) = 0;

	return &this->public;
}
