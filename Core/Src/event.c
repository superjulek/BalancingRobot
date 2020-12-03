/*
 * event.c
 *
 *  Created on: Oct 3, 2020
 *      Author: juliusz
 */

#include "event.h"
#include "PID.h"
#include "stepper.h"
#include "scheduler.h"
#include "config.h"
#include "MPU.h"
#include "general.h"
#include "bluetooth_communicator.h"

#include <math.h>
#include <stdio.h>

/* Imported variables */
extern float angle;
extern float target_angle;
extern PID_t *angle_PID;
extern PID_t *speed_PID;
extern uint8_t RxBuff[RECEIVED_BUFFER_SIZE];
extern stepper_t *left_stepper;
extern stepper_t *right_stepper;
extern scheduler_t *scheduler;
extern MPU_t *myMPU;
extern drive_command_t drive_command;
extern float mount_error;
extern robot_state_t state;
extern float manual_turning_speed;
extern float joystick_max_turning_speed;
extern float manual_driving_speed;
extern float joystick_max_driving_speed;
extern uint32_t batt_vol;
extern float set_turining_speed;
extern bool angle_correction;
/**/

static void flash_LED_callback(void)
{
	HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
}

event_t flash_LED = {
	.action = flash_LED_callback,
};

static void measure_angle_callback(void)
{
	angle = myMPU->get_comp_angle(myMPU);
}

event_t measure_angle = {
	.action = measure_angle_callback,
};

static void wait_for_angle_callback(void)
{
	if (fabs(angle) < 1)
	{
		scheduler->add_to_queue(scheduler, reset_everything);
		scheduler->add_to_queue(scheduler, launch);
	}
}

event_t wait_for_angle = {
	.action = wait_for_angle_callback,
};

static void left_ramp_callback(void)
{
	left_stepper->ramp(left_stepper);
}

event_t left_ramp = {
	.action = left_ramp_callback,
};

static void right_ramp_callback(void)
{
	right_stepper->ramp(right_stepper);
}

event_t right_ramp = {
	.action = right_ramp_callback,
};

static void send_telemetry_callback(void)
{
	telemetry_t current_telemetry = {
		.TargetAngle = target_angle,
		.Angle = angle,
		.TargetSpeed = speed_PID->get_desired_signal(speed_PID),
		.Speed = (float)((left_stepper->get_actual_speed(left_stepper) + right_stepper->get_actual_speed(right_stepper)) / 2),
		.Battery = ((float)batt_vol * 0.001548 - 3.1) / (4.2 - 3.1),
	};
	bt_send_telemetry(&huart1, current_telemetry);
	return;
}

event_t send_telemetry = {
	.action = send_telemetry_callback,
};

static void angle_PID_tic_callback(void)
{
	angle_PID->tic(angle_PID, angle);
	int32_t speed = (int32_t)angle_PID->get_output(angle_PID);
	if (drive_command == LEFT)
	{
		left_stepper->set_speed(left_stepper, speed - (int32_t)set_turining_speed);
		right_stepper->set_speed(right_stepper, speed + (int32_t)set_turining_speed);
	}
	else if (drive_command == RIGHT || drive_command == JOYSTICK_SPEED)
	{
		left_stepper->set_speed(left_stepper, speed + (int32_t)set_turining_speed);
		right_stepper->set_speed(right_stepper, speed - (int32_t)set_turining_speed);
	}
	else
	{
		left_stepper->set_speed(left_stepper, speed);
		right_stepper->set_speed(right_stepper, speed);
	}
}

event_t angle_PID_tic = {
	.action = angle_PID_tic_callback,
};

static void movement_control_tic_callback(void)
{
	int32_t output = (int32_t)angle_PID->get_output(angle_PID);
	switch (drive_command)
	{
	case STOP:
		/* Slowly fix mount angle */
		if (abs(output) < MAX_MOUNT_ANGLE_CORECTION_OUTPUT && angle_correction)
		{
			float correction = MOUNT_ANGLE_CORRECTION * angle;
			mount_error += correction;
			target_angle -= correction;
		}
	case FORWARD:
	case BACKWARD:
	case LEFT:
	case RIGHT:
	case JOYSTICK_SPEED:
		speed_PID->ramp(speed_PID);
		speed_PID->tic(speed_PID, output);
		target_angle = -speed_PID->get_output(speed_PID) / SPEED_PID_DIVIDER;
		break;
	}
	angle_PID->set_desired_signal(angle_PID, target_angle);
}

event_t movement_control_tic = {
	.action = movement_control_tic_callback,
};

static void restart_callback(void)
{
	scheduler->add_to_queue(scheduler, stop);
	scheduler->add_to_queue(scheduler, restart_MPU);
	scheduler->add_to_queue(scheduler, begin_waiting);
}

event_t restart = {
	.action = restart_callback,
};

static void process_rbuf_callback(void)
{
	bt_process_received_buffer(&huart1, RxBuff);
	return;
}

event_t process_rbuf = {
	.action = process_rbuf_callback,
};

static void begin_waiting_callback(void)
{
	bt_send_message(&huart1, "WAITING");
	scheduler->add_to_queue(scheduler, reset_everything);
	state = WAITING_FOR_LAUNCH;
}

event_t begin_waiting = {
	.action = begin_waiting_callback,
};

static void reset_everything_callback(void)

{
	target_angle = 0;
	drive_command = STOP;
	angle_PID->reset(angle_PID);
	angle_PID->set_desired_signal(angle_PID, target_angle);
}
event_t reset_everything = {
	.action = reset_everything_callback,
};

static void launch_callback(void)
{
	bt_send_message(&huart1, "START");
	left_stepper->start(left_stepper);
	right_stepper->start(right_stepper);
	angle_PID->reset(angle_PID);
	speed_PID->reset(speed_PID);
	state = LAUNCHED;
}

event_t launch = {
	.action = launch_callback,
};

static void stop_callback(void)
{
	if (state != STOPPED)
	{
		bt_send_message(&huart1, "STOP");
		left_stepper->stop(left_stepper);
		right_stepper->stop(right_stepper);
		drive_command = STOP;
		state = STOPPED;
	}
	else
	{
		bt_send_message(&huart1, "ALREADY STOPPED");
	}
}

event_t stop = {
	.action = stop_callback,
};

static void restart_MPU_callback(void)
{
	state = PROGRAM_CALIBRATING;
	myMPU->reset(myMPU);
	myMPU->calibrate_gyro(myMPU);
	angle = myMPU->get_acc_angle(myMPU);
	myMPU->reset_mount_error(myMPU);
	myMPU->set_last_angle(myMPU, angle);
}

event_t restart_MPU = {
	.action = restart_MPU_callback,
};

static void emergency_check_callback(void)
{
	if (fabs(angle) > MAX_ANGLE)
	{
		bt_send_message(&huart1, "MAX ANGLE");
		scheduler->add_to_queue(scheduler, stop);
	}
}

event_t emergency_check = {
	.action = emergency_check_callback,
};
