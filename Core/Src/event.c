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

#include <math.h>
#include <stdio.h>

/* Imported variables */
extern float angle;
extern float target_angle;
extern PID_t *angle_PID;
extern PID_t *speed_PID;
extern uint8_t rbuf[1];
extern stepper_t *left_stepper;
extern stepper_t *right_stepper;
extern scheduler_t *scheduler;
extern MPU_t *myMPU;
extern uint8_t drive_command;
extern float mount_error;
extern robot_state_t state;
extern int32_t turning_speed_modified;
extern float driving_speed_modified;
uint8_t TelemetryBuff [sizeof(telemetry_t) + 1];
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
	TelemetryBuff[0] = TELEMETRY_SIGN;
	telemetry_t current_telemetry = {
			.TargetAngle = target_angle,
			.Angle = angle,
			.TargetSpeed = driving_speed_modified,
			.Speed = angle_PID->get_output(angle_PID),
			.Battery = 100.,
	};
	memcpy (TelemetryBuff + 1, &current_telemetry, sizeof(telemetry_t));
	HAL_UART_Transmit_DMA(&huart1, TelemetryBuff, sizeof(telemetry_t) + 1);
	return;
	char buff[30];
	sprintf(buff, "%.2f\t%.2f\t%d\n", angle, target_angle, (int)(angle_PID->get_output(angle_PID) / 1000));
	send_string(buff);
}

event_t send_telemetry = {
	.action = send_telemetry_callback,
};

static void angle_PID_tic_callback(void)
{
	angle_PID->tic(angle_PID, angle);
	int32_t speed = (int32_t)angle_PID->get_output_smooth(angle_PID);
	int32_t turning = 0;
	if (drive_command == 3)
		turning = turning_speed_modified;
	if (drive_command == 4)
		turning = -turning_speed_modified;
	left_stepper->set_speed(left_stepper, speed - turning);
	right_stepper->set_speed(right_stepper, speed + turning);
}

event_t angle_PID_tic = {
	.action = angle_PID_tic_callback,
};

static void movement_control_tic_callback(void)
{
	int32_t output = (int32_t)angle_PID->get_output(angle_PID);
	/* For smoother control */
	static int32_t averaged_output = 0;
	averaged_output = averaged_output * 0.05 + output * 0.95;
	switch (drive_command)
	{
	case LEFT:
	case RIGHT:
	case STOP:
		if ((target_angle > -MAX_ANGLE_STANDING) && (output > MAX_STANDING_SPEED))
		{
			target_angle -= ANGLE_CORRECTION;
		}
		if ((target_angle < MAX_ANGLE_STANDING) && (output < -MAX_STANDING_SPEED))
		{
			target_angle += ANGLE_CORRECTION;
		}
		/* Slowly fix mount angle */
		if (abs(output) < MAX_MOUNT_ANGLE_CORECTION_OUTPUT)
		{
			mount_error += MOUNT_ANGLE_CORECTION * angle;
			target_angle -= MOUNT_ANGLE_CORECTION * angle;
		}
		break;
	case FORWARD:
	case BACKWARD:
	case VELOCITY_BRAKE:
		speed_PID->tic(speed_PID, averaged_output);
		target_angle = -speed_PID->get_output_smooth(speed_PID) / 1000.;
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
	static bool tuning_angle = TRUE; //FALSE - tuning speed
	switch (rbuf[0])
	{
	case 0:
		scheduler->add_to_queue(scheduler, restart);
		break;
	case 1:
		scheduler->add_to_queue(scheduler, stop);
		break;
	case 2:
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
		break;
	case 3:
	case 4:
	case 5:
	case 6:
	case 7:
	case 8:
	{
	}
		{
			PID_coefs_t current_coefs;
			if (tuning_angle)
			{
				current_coefs = angle_PID->get_PID_coefs(angle_PID);
			}
			else
			{
				current_coefs = speed_PID->get_PID_coefs(speed_PID);
			}
			float inc = 0.05;
			if (rbuf[0] % 2 == 0)
				inc = -inc;
			switch (rbuf[0] % 3)
			{
			case 0:
				current_coefs.KP_coef *= (1 + inc);
				break;
			case 1:
				current_coefs.KI_coef *= (1 + inc);
				break;
			case 2:
				current_coefs.KD_coef *= (1 + inc);
				break;
			}
			if (tuning_angle)
			{
				angle_PID->set_PID_coefs(angle_PID, current_coefs);
			}
			else
			{
				speed_PID->set_PID_coefs(speed_PID, current_coefs);
			}
			char buff[12];
			switch (rbuf[0] % 3)
			{
			case 0:
				sprintf(buff, "\t%f\n", current_coefs.KP_coef);
				break;
			case 1:
				sprintf(buff, "\t%f\n", current_coefs.KI_coef);
				break;
			case 2:
				sprintf(buff, "\t%f\n", current_coefs.KD_coef);
				break;
			}
			send_string(buff);
			break;
		}
	case 9:
	{
		driving_speed_modified *= 1.05;
		if (drive_command == FORWARD)
		{
			speed_PID->set_desired_signal(speed_PID, driving_speed_modified);
		}
		if (drive_command == BACKWARD)
		{
			speed_PID->set_desired_signal(speed_PID, -driving_speed_modified);
		}
		char buff[15];
		sprintf(buff, "VP = %.0f\n", driving_speed_modified);
		send_string(buff);
		break;
	}
	case 12:
	{
		driving_speed_modified *= 0.95;
		if (drive_command == FORWARD)
		{
			speed_PID->set_desired_signal(speed_PID, driving_speed_modified);
		}
		if (drive_command == BACKWARD)
		{
			speed_PID->set_desired_signal(speed_PID, -driving_speed_modified);
		}
		char buff[15];
		sprintf(buff, "VP = %.0f\n", driving_speed_modified);
		send_string(buff);
		break;
	}
	case 13:
	{
		turning_speed_modified *= 1.05;
		char buff[15];
		sprintf(buff, "VT = %ld\n", turning_speed_modified);
		send_string(buff);
		break;
	}
	case 14:
		tuning_angle = !tuning_angle;
		send_string((tuning_angle ? "Tuning angle PID\n" : " Tuning speed PID\n"));
	case 10:
	{
		turning_speed_modified *= 0.95;
		char buff[15];
		sprintf(buff, "VT = %ld\n", turning_speed_modified);
		send_string(buff);
		break;
	}
	case 11:
		speed_PID->set_desired_signal(speed_PID, 0);
		drive_command = VELOCITY_BRAKE;
		send_string("Velocity brake!\n");
		break;
	case 15:
		drive_command = STOP;
		send_string("stop\n");
		break;
	case 16:
		drive_command = FORWARD;
		speed_PID->reset(speed_PID);
		speed_PID->set_desired_signal(speed_PID, driving_speed_modified);
		send_string("fwd\n");
		break;
	case 17:
		drive_command = BACKWARD;
		speed_PID->reset(speed_PID);
		speed_PID->set_desired_signal(speed_PID, -driving_speed_modified);
		send_string("bwd\n");
		break;
	case 18:
		drive_command = LEFT;
		send_string("left\n");
		break;
	case 19:
		drive_command = RIGHT;
		send_string("right\n");
		break;
	case 20:
		/* Live fix mounting error */
		mount_error += angle;
		scheduler->add_to_queue(scheduler, reset_everything);
		break;
	}
}

event_t process_rbuf = {
	.action = process_rbuf_callback,
};

static void begin_waiting_callback(void)
{
	send_string("WAITING\n");
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
	send_string("START\n");
	left_stepper->start(left_stepper);
	right_stepper->start(right_stepper);
	state = LAUNCHED;
}

event_t launch = {
	.action = launch_callback,
};

static void stop_callback(void)
{
	if (state != STOPPED)
	{
		send_string("STOP\n");
		left_stepper->stop(left_stepper);
		right_stepper->stop(right_stepper);
		drive_command = STOP;
		state = STOPPED;
	}
	else
	{
		send_string("ALREADY STOPPED\n");
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
		send_string("MAX ANGLE\n");
		scheduler->add_to_queue(scheduler, stop);
	}
}

event_t emergency_check = {
	.action = emergency_check_callback,
};
