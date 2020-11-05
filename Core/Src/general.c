/*
 * general.c
 *
 *  Created on: Aug 9, 2020
 *      Author: juliusz
 */

#include "general.h"

#define MAX_TMP_BUFF 20
char tmp_buff[MAX_TMP_BUFF];

void write_pin (pin_t pin, GPIO_PinState state)
{
	HAL_GPIO_WritePin(pin.pin_port, pin.pin, state);
}

void send_string (char *string){
	if (strlen(string) > MAX_TMP_BUFF) return; //too long to store
	memcpy(tmp_buff, string, strlen(string));
	HAL_UART_Transmit_DMA(&huart1, string, strlen(string));
}
