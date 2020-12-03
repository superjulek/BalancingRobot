/*
 * bluetooth_communicator.c
 *
 *  Created on: Oct 31, 2020
 *      Author: juliusz
 */

#include <string.h>

#include "bluetooth_communicator.h"
#include "PID.h"
#include "event.h"
#include "scheduler.h"
#include "general.h"
#include "config.h"

/* Communication signs - robot -> controller */
/* 7th bit for state */
#define TELEMETRY_SIGN          0b10000001
#define ANGLE_PID_COEFS_SIGN    0b10000010
#define SPEED_PID_COEFS_SIGN    0b10000011
#define MANUAL_SPEEDS_SIGN      0b10000100
#define JOYSTICK_SPEEDS_SIGN    0b10000101
#define MESSAGE_SIGN            0b10000110

/* Communication signs - controller -> robot */
#define GET_ANGLE_PID_COEFS_SIGN        0x00
#define GET_SPEED_PID_COEFS_SIGN        0x01
#define STOP_ROBOT                      0x02
#define RESTART_ROBOT                   0x03
#define START_ROBOT                     0x04
#define SET_ANGLE_PID_COEFS_SIGN        0x05
#define SET_SPEED_PID_COEFS_SIGN        0x06
#define GET_MANUAL_SPEED                0x07
#define GET_JOYSTICK_SPEED              0x08
#define SET_MANUAL_SPEED                0x09
#define SET_JOYSTICK_SPEED              0x0A
#define SET_MANUAL_STOP                 0x0B
#define SET_MANUAL_FWD                  0x0C
#define SET_MANUAL_BWD                  0x0D
#define SET_MANUAL_LEFT                 0x0E
#define SET_MANUAL_RIGHT                0x0F
#define SET_JOYSTICK_CONTROL            0x10
#define TOGGLE_ANGLE_CORRECTION          0x11

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
extern float set_turining_speed;
extern bool angle_correction;

/* Variables for charts */
extern float accelerometer_angle;
extern float gyroscope_angle;
extern float angle;

uint8_t TelemetryBuff[sizeof(telemetry_t) + 1];
uint8_t PIDConfBuff[sizeof(PID_coefs_t) + 1];
uint8_t SpeedsBuff[sizeof(speeds_t) + 1];
unsigned char MessageBuff[MESSAGE_BUFF_SIZE];

void bt_send_telemetry(UART_HandleTypeDef *huart, telemetry_t telemetry)
{
    TelemetryBuff[0] = TELEMETRY_SIGN;
    if (state == LAUNCHED)
        TelemetryBuff[0] |= 0b01000000;
    memcpy(TelemetryBuff + 1, &telemetry, sizeof(telemetry_t));
    HAL_UART_Transmit_DMA(huart, TelemetryBuff, sizeof(telemetry_t) + 1);
    return;
}

void bt_send_message(UART_HandleTypeDef *huart, char *message)
{
    if (strlen(message) + 1 > MESSAGE_BUFF_SIZE)
    {
        message = "Message too long";
    }
    MessageBuff[0] = MESSAGE_SIGN;
    memcpy(MessageBuff + 1, message, strlen(message)); // Sign on first place
    HAL_UART_Transmit_DMA(huart, MessageBuff, strlen(message) + 1);
    return;
}

static void bt_send_angle_PID_coefs(UART_HandleTypeDef *huart)
{
    PID_coefs_t current_coefs = angle_PID->get_PID_coefs(angle_PID);
    PIDConfBuff[0] = ANGLE_PID_COEFS_SIGN;
    memcpy(PIDConfBuff + 1, &current_coefs, sizeof(PID_coefs_t));
    HAL_UART_Transmit_DMA(huart, PIDConfBuff, sizeof(PID_coefs_t) + 1);
}

static void bt_send_speed_PID_coefs(UART_HandleTypeDef *huart)
{
    PID_coefs_t current_coefs = speed_PID->get_PID_coefs(speed_PID);
    PIDConfBuff[0] = SPEED_PID_COEFS_SIGN;
    memcpy(PIDConfBuff + 1, &current_coefs, sizeof(PID_coefs_t));
    HAL_UART_Transmit_DMA(huart, PIDConfBuff, sizeof(PID_coefs_t) + 1);
}

static void bt_stop_robot()
{
    scheduler->add_to_queue(scheduler, stop);
}

static void bt_restart_robot()
{
    scheduler->add_to_queue(scheduler, restart);
}

static void bt_set_manual_stop()
{
    if (drive_command != STOP)
    {
        drive_command = STOP;
        speed_PID->set_desired_signal_to_ramp(speed_PID, 0);
        bt_send_message(&huart1, "stopping");
    }
}

static void bt_set_manual_fwd()
{
    if (drive_command != FORWARD)
    {
        drive_command = FORWARD;
        speed_PID->set_desired_signal_to_ramp(speed_PID, manual_driving_speed);
        bt_send_message(&huart1, "going fwd\n");
    }
}

static void bt_set_manual_bwd()
{
    if (drive_command != BACKWARD)
    {
        drive_command = BACKWARD;
        speed_PID->set_desired_signal_to_ramp(speed_PID, -manual_driving_speed);
        bt_send_message(&huart1, "going bwd\n");
    }
}

static void bt_set_manual_left()
{
    if (drive_command != LEFT)
    {
        drive_command = LEFT;
        set_turining_speed = manual_turning_speed;
        speed_PID->set_desired_signal_to_ramp(speed_PID, 0);
    }
}

static void bt_set_manual_right()
{
    if (drive_command != RIGHT)
    {
        drive_command = RIGHT;
        set_turining_speed = manual_turning_speed;
        speed_PID->set_desired_signal_to_ramp(speed_PID, 0);
    }
}

static void bt_set_joystick_control(message_t message)
{
    drive_command = JOYSTICK_SPEED;
    speeds_t new_speeds = {
        .driving_speed = message.data[0],
        .turning_speed = message.data[1],
    };
    set_turining_speed = new_speeds.turning_speed * joystick_max_turning_speed;
    speed_PID->set_desired_signal_to_ramp(speed_PID, new_speeds.driving_speed * joystick_max_driving_speed);
}

static void bt_start_robot()
{
	accelerometer_angle = angle;
	gyroscope_angle = angle;
    if (state == LAUNCHED)
    {
        bt_send_message(&huart1, "ALREADY RUNNING");
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

static void bt_set_PID_coefs(message_t message, PID_t *pid)
{
    PID_coefs_t coefs = {
        .KP_coef = message.data[0],
        .KI_coef = message.data[1],
        .KD_coef = message.data[2],
    };
    pid->set_PID_coefs(pid, coefs);
}

static void bt_send_manual_speed(UART_HandleTypeDef *huart)
{
    speeds_t speeds;
    speeds.driving_speed = manual_driving_speed;
    speeds.turning_speed = manual_turning_speed;
    SpeedsBuff[0] = MANUAL_SPEEDS_SIGN;
    memcpy(SpeedsBuff + 1, &speeds, sizeof(speeds_t));
    HAL_UART_Transmit_DMA(huart, SpeedsBuff, sizeof(speeds_t) + 1);
}

static void bt_send_joystick_speed(UART_HandleTypeDef *huart)
{
    speeds_t speeds;
    speeds.driving_speed = joystick_max_driving_speed;
    speeds.turning_speed = joystick_max_turning_speed;
    SpeedsBuff[0] = JOYSTICK_SPEEDS_SIGN;
    memcpy(SpeedsBuff + 1, &speeds, sizeof(speeds_t));
    HAL_UART_Transmit_DMA(huart, SpeedsBuff, sizeof(speeds_t) + 1);
}

static void bt_set_manual_speeds(message_t message)
{
    speeds_t speeds = {
        .driving_speed = message.data[0],
        .turning_speed = message.data[1],
    };
    if (speeds.driving_speed > MAX_DRIVING_SPEED)
        speeds.driving_speed = (float)MAX_DRIVING_SPEED;
    if (speeds.driving_speed < 0)
        speeds.driving_speed = 0;
    if (speeds.turning_speed > MAX_DRIVING_SPEED)
        speeds.turning_speed = (float)MAX_TURNING_SPEED;
    if (speeds.turning_speed < 0)
        speeds.turning_speed = 0;
    manual_driving_speed = speeds.driving_speed;
    manual_turning_speed = speeds.turning_speed;
}

static void bt_set_joystick_speeds(message_t message)
{
    speeds_t speeds = {
        .driving_speed = message.data[0],
        .turning_speed = message.data[1],
    };
    if (speeds.driving_speed > MAX_DRIVING_SPEED)
        speeds.driving_speed = (float)MAX_DRIVING_SPEED;
    if (speeds.driving_speed < 0)
        speeds.driving_speed = 0;
    if (speeds.turning_speed > MAX_DRIVING_SPEED)
        speeds.turning_speed = (float)MAX_TURNING_SPEED;
    if (speeds.turning_speed < 0)
        speeds.turning_speed = 0;
    joystick_max_driving_speed = speeds.driving_speed;
    joystick_max_turning_speed = speeds.turning_speed;
}

static void bt_toggle_angle_correction()
{
    angle_correction = !angle_correction;
    if (angle_correction)
    {
        bt_send_message(&huart1, "Ang Corr ON");
    }
    else
    {
        bt_send_message(&huart1, "ANG Corr OFF");
    }
    
}

void bt_process_received_buffer(UART_HandleTypeDef *huart, uint8_t *buffer)
{
    message_t message;
    memcpy(&message, buffer, RECEIVED_BUFFER_SIZE);
    switch (message.sign)
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
        bt_set_PID_coefs(message, angle_PID);
        break;
    }
    case SET_SPEED_PID_COEFS_SIGN:
    {
        bt_set_PID_coefs(message, speed_PID);
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
        bt_set_manual_speeds(message);
        break;
    }
    case SET_JOYSTICK_SPEED:
    {
        bt_set_joystick_speeds(message);
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
        bt_set_joystick_control(message);
        break;
    }
    case TOGGLE_ANGLE_CORRECTION:
    {
        bt_toggle_angle_correction();
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
