/*
 * general.h
 *
 *  Created on: Aug 9, 2020
 *      Author: juliusz
 */

#ifndef INC_GENERAL_H_
#define INC_GENERAL_H_

#include "main.h"
#include "usart.h"
#include <stdlib.h>
#include <string.h>

typedef struct pin_t pin_t;

typedef enum robot_state_t robot_state_t;
typedef enum drive_command_t drive_command_t;

struct pin_t {
	GPIO_TypeDef *pin_port;
	uint16_t pin;
};

enum robot_state_t {
	PROGRAM_CALIBRATING,
	STOPPED,
	WAITING_FOR_LAUNCH,
	LAUNCHED,
};

enum drive_command_t {
	STOP,
	FORWARD,
	BACKWARD,
	LEFT,
	RIGHT,
	JOYSTICK_SPEED,
};

void write_pin (pin_t pin, GPIO_PinState state);

void send_string (char *string);

#define bool uint8_t

#define TRUE 1

#define FALSE 0

#define max_uint32_t 4294967295

#endif /* INC_GENERAL_H_ */
