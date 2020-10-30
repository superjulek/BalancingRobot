/*
 * general.c
 *
 *  Created on: Aug 9, 2020
 *      Author: juliusz
 */

#include "general.h"


void write_pin (pin_t pin, GPIO_PinState state)
{
	HAL_GPIO_WritePin(pin.pin_port, pin.pin, state);
}

void send_string (char *string){
	HAL_UART_Transmit_DMA(&huart1, string, strlen(string));
}

uint16_t hash_string (char *string){
	uint16_t result = string[0] + string[1] <<8;
	return result;
}
