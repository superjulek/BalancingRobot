/*
 * bluetooth_communicator.h
 *
 *  Created on: Oct 31, 2020
 *      Author: juliusz
 */

#ifndef INC_BLUETOOTH_COMMUNICATOR_H_
#define INC_BLUETOOTH_COMMUNICATOR_H_

/*##### TO BE DEFINED #####*/
/**
 * MESSAGE_BUFF_SIZE - max size of sent message size 
 */
/*#########################*/
#include "usart.h"

typedef struct telemetry_t telemetry_t;
typedef struct message_t message_t;

struct telemetry_t {
	float TargetAngle;
	float Angle;
	float TargetSpeed;
	float Speed;
	float Battery;
};

struct message_t {
	uint8_t sign;
	float data[3];
} __attribute__((packed));

/* Receided buffer size */
#define RECEIVED_BUFFER_SIZE        sizeof(message_t)

void bt_send_telemetry(UART_HandleTypeDef *huart, telemetry_t telemetry);

void bt_process_received_buffer(UART_HandleTypeDef *huart, uint8_t *buffer);

void bt_send_message(UART_HandleTypeDef *huart, char *message);


#endif /* INC_BLUETOOTH_COMMUNICATOR_H_ */
