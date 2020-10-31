/*
 * bluetooth_communicator.h
 *
 *  Created on: Oct 31, 2020
 *      Author: juliusz
 */

#ifndef INC_BLUETOOTH_COMMUNICATOR_H_
#define INC_BLUETOOTH_COMMUNICATOR_H_

#include "usart.h"

/* Receided buffer size */
#define RECEIVED_BUFFER_SIZE        16

typedef struct telemetry_t telemetry_t;

struct telemetry_t {
	float TargetAngle;
	float Angle;
	float TargetSpeed;
	float Speed;
	float Battery;
};

void bt_send_telemetry(UART_HandleTypeDef *huart, telemetry_t telemetry);

void bt_process_received_buffer(UART_HandleTypeDef *huart, uint8_t *buffer);


#endif /* INC_BLUETOOTH_COMMUNICATOR_H_ */
