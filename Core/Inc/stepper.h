/*
 * stepper.h
 *
 *  Created on: Aug 7, 2020
 *      Author: juliusz
 */

#ifndef INC_STEPPER_H_
#define INC_STEPPER_H_

#include "main.h"
#include "general.h"

#include "stm32f4xx_hal.h"

/*##### TO BE DEFINED #####*/
/**
 * ACCELERATION - stepper acceleration (mrpss)
 * 
 * RAMP_FREQUENCY - ramp frequency (Hz)
 * 
 * CLOCK_FREQUENCY - stepper driver clock frequency (Hz)
 * 
 * STEPS_PER_REVOLUTION - number of full steps per resolution
 * 
 * MAX_SPEED - max stepper speed (mrps)
 * 
 * MAX_STEPPING_INTERVAL - max stepping interval (us)
 */
/*#########################*/

#define MICROSTEPPING_NOT_SUPPORTED 255

//#define A4988
//#define DRV8825
#define LV8719

typedef enum microstepping_t microstepping_t;
typedef struct stepper_pins_t stepper_pins_t;
typedef struct stepper_t stepper_t;
typedef enum drive_command_t drive_command_t;

enum microstepping_t {
	STEPS_FULL = 1,
	STEPS_2 = 2,
	STEPS_4 = 4,
	STEPS_8 = 8,
	STEPS_16 = 16,
	STEPS_32 = 32,
	STEPS_64 = 64,
	STEPS_128 = 128,
};

enum drive_command_t {
	STOP = 0,
	FORWARD = 1,
	BACKWARD = 2,
	LEFT = 3,
	RIGHT = 4,
	VELOCITY_BRAKE = 5,
};

#ifdef DRV8825
static uint8_t microstepping_table[] = {
		0b000,
		0b100,
		0b010,
		0b110,
		0b001,
		0b101,
		MICROSTEPPING_NOT_SUPPORTED,
		MICROSTEPPING_NOT_SUPPORTED
};
#endif
#ifdef A4988
static uint8_t microstepping_table[] = {
		0b000,
		0b100,
		0b010,
		0b110,
		0b111,
		MICROSTEPPING_NOT_SUPPORTED,
		MICROSTEPPING_NOT_SUPPORTED,
		MICROSTEPPING_NOT_SUPPORTED
};
#endif
#ifdef LV8719
static uint8_t microstepping_table[] = {
		0b000,
		0b100,
		0b010,
		0b110,
		0b001,
		0b101,
		0b011,
		0b111
};
#endif

struct stepper_pins_t {
	pin_t DIR;
	pin_t ENABLE;
	pin_t MS1;
	pin_t MS2;
	pin_t MS3;
};

struct stepper_t {

	/**
	 * Start stepper operation
	 */
	void (*start) (stepper_t *this);

	/**
	 * Set stepper microstepping
	 *
	 * @param microstepping		microstepping
	 */
	void (*set_microstepping) (stepper_t *this, microstepping_t microstepping);

	/**
	 * Set desired speed
	 *
	 * @param speed				speed in mrpm
	 */
	void (*set_speed) (stepper_t *this, int32_t speed);

	/**
	 * Ramp stepper speed
	 */
	void (*ramp) (stepper_t *this);

	/**
	 * Stop stepper operation
	 */
	void (*stop) (stepper_t *this);
};

/**
 * Create stepper instance
 */
stepper_t* stepper_create(stepper_pins_t pins, int32_t *period, bool reverse_direction, TIM_HandleTypeDef *timer, uint32_t channel);

#endif /* INC_STEPPER_H_ */
