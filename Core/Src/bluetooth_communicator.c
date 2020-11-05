/*
 * bluetooth_communicator.c
 *
 *  Created on: Oct 31, 2020
 *      Author: juliusz
 */

#include "bluetooth_communicator.h"
#include "PID.h"
#include "event.h"
#include "scheduler.h"
#include "general.h"

/* Communication signs - robot -> controller */
#define TELEMETRY_SIGN 0b10000001
#define ANGLE_PID_COEFS_SIGN 0b10000010
#define SPEED_PID_COEFS_SIGN 0b10000011
#define MANUAL_SPEEDS_SIGN 0b10000100
#define JOYSTICK_SPEEDS_SIGN 0b10000101

/* Communication signs - controller -> robot */
#define GET_ANGLE_PID_COEFS_SIGN    0x00
#define GET_SPEED_PID_COEFS_SIGN    0x01
#define STOP_ROBOT                  0x02
#define RESTART_ROBOT               0x03
#define START_ROBOT                 0x04
#define SET_ANGLE_PID_COEFS_SIGN    0x05
#define SET_SPEED_PID_COEFS_SIGN    0x06
#define GET_MANUAL_SPEED            0x07
#define GET_JOYSTICK_SPEED          0x08
#define SET_MANUAL_SPEED            0x09
#define SET_JOYSTICK_SPEED          0x0A
#define SET_MANUAL_STOP             0x0B
#define SET_MANUAL_FWD              0x0C
#define SET_MANUAL_BWD              0x0D
#define SET_MANUAL_LEFT             0x0E
#define SET_MANUAL_RIGHT            0x0F
#define SET_JOYSTICK_CONTROL        0x10
#define SET_ANGLE_CALIBRATION       0x11

typedef struct speeds_t speeds_t;

struct speeds_t
{
    float driving_speed;
    float turning_speed;
};

extern PID_t *angle_PID;
extern PID_t *speed_PID;
extern float manual_turning_speed;
extern float joystick_max_turning_speed;
extern float manual_driving_speed;
extern float joystick_max_driving_speed;
extern scheduler_t *scheduler;
extern robot_state_t state;
extern drive_command_t drive_command;

uint8_t TelemetryBuff[sizeof(telemetry_t) + 1];
uint8_t PIDConfBuff[sizeof(PID_coefs_t) + 1];
uint8_t SpeedsBuff[sizeof(speeds_t) + 1];

void bt_send_telemetry(UART_HandleTypeDef *huart, telemetry_t telemetry)
{
    TelemetryBuff[0] = TELEMETRY_SIGN;
    memcpy(TelemetryBuff + 1, &telemetry, sizeof(telemetry_t));
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

void bt_stop_robot()
{
    scheduler->add_to_queue(scheduler, stop);
}

void bt_restart_robot()
{
    scheduler->add_to_queue(scheduler, restart);
}

void bt_set_manual_stop()
{
    if (drive_command != STOP)
    {
        drive_command = STOP;
        speed_PID->reset(speed_PID);
        speed_PID->set_desired_signal(speed_PID, 0);
        send_string("stopping");
    }
}

void bt_set_manual_fwd()
{
    if (drive_command != FORWARD)
    {
        drive_command = FORWARD;
		speed_PID->reset(speed_PID);
		speed_PID->set_desired_signal(speed_PID, manual_driving_speed);
		send_string("going fwd\n");
    }
}

void bt_set_manual_bwd()
{
    if (drive_command != BACKWARD)
    {
        drive_command = BACKWARD;
		speed_PID->reset(speed_PID);
		speed_PID->set_desired_signal(speed_PID, -manual_driving_speed);
		send_string("going bwd\n");
    }
}

void bt_set_manual_left()
{
    drive_command = LEFT;
}

void bt_set_manual_right()
{
    drive_command = RIGHT;
}

void bt_set_joystick_control()
{
    drive_command = JOYSTICK_SPEED;
}

void bt_set_angle_calibration()
{
    if (drive_command != ANGLE_CALIBRATING)
    {
        drive_command = ANGLE_CALIBRATING;
        send_string("Angle calibration");
    }
}

void bt_start_robot()
{
    if (state == LAUNCHED)
    {
        send_string("ALREADY RUNNING\n");
    }
    else if (state == STOPPED)
    {
        scheduler->add_to_queue(scheduler, begin_waiting);
    }
    else
    {
        scheduler->add_to_queue(scheduler, stop);
        scheduler->add_to_queue(scheduler, begin_waiting);
    }
}

void bt_set_PID_coefs(uint8_t *buffer, PID_t *pid)
{
    PID_coefs_t coefs;
    memcpy(&coefs, buffer + 4, sizeof(PID_coefs_t));
    pid->set_PID_coefs(pid, coefs);
}

void bt_send_manual_speed(UART_HandleTypeDef *huart)
{
    speeds_t speeds;
    speeds.driving_speed = manual_driving_speed;
    speeds.turning_speed = manual_turning_speed;
    SpeedsBuff[0] = MANUAL_SPEEDS_SIGN;
    memcpy(SpeedsBuff + 1, &speeds, sizeof(speeds_t));
    HAL_UART_Transmit_DMA(huart, SpeedsBuff, sizeof(speeds_t) + 1);
}

void bt_send_joystick_speed(UART_HandleTypeDef *huart)
{
    speeds_t speeds;
    speeds.driving_speed = joystick_max_driving_speed;
    speeds.turning_speed = joystick_max_turning_speed;
    SpeedsBuff[0] = JOYSTICK_SPEEDS_SIGN;
    memcpy(SpeedsBuff + 1, &speeds, sizeof(speeds_t));
    HAL_UART_Transmit_DMA(huart, SpeedsBuff, sizeof(speeds_t) + 1);
}

void bt_set_manual_speeds(uint8_t *buffer)
{
    speeds_t speeds;
    memcpy(&speeds, buffer + 4, sizeof(speeds_t));
    manual_driving_speed = speeds.driving_speed;
    manual_turning_speed = speeds.turning_speed;
}

void bt_set_joystick_speeds(uint8_t *buffer)
{
    speeds_t speeds;
    memcpy(&speeds, buffer + 4, sizeof(speeds_t));
    joystick_max_driving_speed = speeds.driving_speed;
    joystick_max_turning_speed = speeds.turning_speed;
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
    case STOP_ROBOT:
    {
        bt_stop_robot();
        break;
    }
    case RESTART_ROBOT:
    {
        bt_restart_robot();
        break;
    }
    case START_ROBOT:
    {
        bt_start_robot();
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
    case GET_MANUAL_SPEED:
    {
        bt_send_manual_speed(huart);
        break;
    }
    case GET_JOYSTICK_SPEED:
    {
        bt_send_joystick_speed(huart);
        break;
    }
    case SET_MANUAL_SPEED:
    {
        bt_set_manual_speeds(buffer);
        break;
    }
    case SET_JOYSTICK_SPEED:
    {
        bt_set_joystick_speeds(buffer);
        break;
    }
    case SET_MANUAL_STOP:
    {
        bt_set_manual_stop();
        break;
    }
    case SET_MANUAL_FWD:
    {
        bt_set_manual_fwd();
        break;
    }
    case SET_MANUAL_BWD:
    {
        bt_set_manual_bwd();
        break;
    }
    case SET_MANUAL_LEFT:
    {
        bt_set_manual_left();
        break;
    }
    case SET_MANUAL_RIGHT:
    {
        bt_set_manual_right();
        break;
    }
    case SET_JOYSTICK_CONTROL:
    {
        bt_set_joystick_control();
        break;
    }
    case SET_ANGLE_CALIBRATION:
    {
        bt_set_angle_calibration();
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
