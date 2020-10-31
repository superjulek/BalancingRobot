/*
 * bluetooth_communicator.c
 *
 *  Created on: Oct 31, 2020
 *      Author: juliusz
 */

#include "bluetooth_communicator.h"
#include "PID.h"

/* Communication signs - robot -> controller */
#define TELEMETRY_SIGN              0b10000001
#define ANGLE_PID_COEFS_SIGN        0b10000010
#define SPEED_PID_COEFS_SIGN        0b10000011

/* Communication signs - controller -> robot */
#define GET_ANGLE_PID_COEFS_SIGN    0x00
#define GET_SPEED_PID_COEFS_SIGN    0x01
#define STOP_SIGN                   0x02
#define START_SIGN                  0x03
#define RESTART_SIGN                0x04
#define SET_ANGLE_PID_COEFS_SIGN    0x05
#define SET_SPEED_PID_COEFS_SIGN    0x06


extern PID_t *angle_PID;
extern PID_t *speed_PID;

uint8_t TelemetryBuff [sizeof(telemetry_t) + 1];
uint8_t PIDConfBuff [sizeof(PID_coefs_t) + 1];

void bt_send_telemetry(UART_HandleTypeDef *huart, telemetry_t telemetry)
{
	TelemetryBuff[0] = TELEMETRY_SIGN;
	memcpy (TelemetryBuff + 1, &telemetry, sizeof(telemetry_t));
	HAL_UART_Transmit_DMA(huart, TelemetryBuff, sizeof(telemetry_t) + 1);
	return;
}

void bt_send_angle_PID_coefs(UART_HandleTypeDef *huart)
{
    PID_coefs_t current_coefs = angle_PID->get_PID_coefs(angle_PID);
    PIDConfBuff[0] = ANGLE_PID_COEFS_SIGN;
    memcpy(PIDConfBuff + 1, &current_coefs, sizeof(PID_coefs_t));
    HAL_UART_Transmit_DMA(huart, PIDConfBuff, sizeof(PID_coefs_t) + 1);
}

void bt_send_speed_PID_coefs(UART_HandleTypeDef *huart)
{
    PID_coefs_t current_coefs = speed_PID->get_PID_coefs(speed_PID);
    PIDConfBuff[0] = SPEED_PID_COEFS_SIGN;
    memcpy(PIDConfBuff + 1, &current_coefs, sizeof(PID_coefs_t));
    HAL_UART_Transmit_DMA(huart, PIDConfBuff, sizeof(PID_coefs_t) + 1);
}

void bt_set_PID_coefs(uint8_t *buffer, PID_t *pid)
{
    PID_coefs_t coefs;
    memcpy(&coefs, buffer + 4, sizeof(PID_coefs_t));
    pid->set_PID_coefs(pid, coefs);
}

void bt_process_received_buffer(UART_HandleTypeDef *huart, uint8_t *buffer)
{
    uint32_t signal;
    memcpy(&signal, buffer, 4);
    switch (signal)
    {
        case GET_ANGLE_PID_COEFS_SIGN:
        {
            bt_send_angle_PID_coefs(huart);
            break;
        }
        case GET_SPEED_PID_COEFS_SIGN:
        {
            bt_send_speed_PID_coefs(huart);
            break;
        }
        case SET_ANGLE_PID_COEFS_SIGN:
        {
            bt_set_PID_coefs(buffer, angle_PID);
            break;
        }
        case SET_SPEED_PID_COEFS_SIGN:
        {
            bt_set_PID_coefs(buffer, speed_PID);
            break;
        }
        default:
        {
            /* Przyszła błędna wiadomość - być może się rozjechała - reset */
            HAL_UART_DMAStop(huart);
            HAL_UART_Receive_DMA(huart, buffer, RECEIVED_BUFFER_SIZE);
            break;
        }
    }
}
