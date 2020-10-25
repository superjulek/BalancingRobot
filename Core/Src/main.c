/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stepper.h"
#include "mpu6050.h"
#include "PID.h"
#include "scheduler.h"
#include "general.h"
#include "config.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
stepper_pins_t left_pins = {
		.DIR = {
				.pin_port = L_DIR_GPIO_Port,
				.pin = L_DIR_Pin,
		},
		.ENABLE = {
				.pin_port = L_ENABLE_GPIO_Port,
				.pin = L_ENABLE_Pin,
		},
		.MS1 = {
				.pin_port = L_MS1_GPIO_Port,
				.pin = L_MS1_Pin,
		},
		.MS2 = {
				.pin_port = L_MS2_GPIO_Port,
				.pin = L_MS2_Pin,
		},
		.MS3 = {
				.pin_port = L_MS3_GPIO_Port,
				.pin = L_MS3_Pin,
		},
};
stepper_pins_t right_pins = {
		.DIR = {
				.pin_port = R_DIR_GPIO_Port,
				.pin = R_DIR_Pin,
		},
		.ENABLE = {
				.pin_port = R_ENABLE_GPIO_Port,
				.pin = R_ENABLE_Pin,
		},
		.MS1 = {
				.pin_port = L_MS1_GPIO_Port,
				.pin = L_MS1_Pin,
		},
		.MS2 = {
				.pin_port = L_MS2_GPIO_Port,
				.pin = L_MS2_Pin,
		},
		.MS3 = {
				.pin_port = L_MS3_GPIO_Port,
				.pin = L_MS3_Pin,
		},
};
PID_coefs_t angle_PID_coefs = {
		.KP_coef = 7800,
		.KI_coef = 180000,
		.KD_coef = 110,
};
PID_coefs_t speed_PID_coefs = {
		.KP_coef = 0.011,
		.KI_coef = 0.065,
		.KD_coef = 0.000057,
};

/* SHARED VARIABLES - copy as extern to event.c*/
float angle;
float target_angle = 0.;
float target_position = 0;
volatile int32_t position = 0;
float mount_error = MOUNT_ERROR;
PID_t *angle_PID;
PID_t *speed_PID;
stepper_t *left_stepper;
stepper_t *right_stepper;
scheduler_t *scheduler;
uint8_t rbuf[1];
uint8_t drive_command = 0;
volatile robot_state_t state = PROGRAM_CALIBRATING;
int32_t turning_speed_modified = TURNING_SPEED;
float driving_speed_modified = DRIVING_SPEED;


extern int16_t gyro_cali_x, gyro_cali_y, gyro_cali_z;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	left_stepper = stepper_create(left_pins, TRUE, &htim5, TIM_CHANNEL_1);
	right_stepper = stepper_create(right_pins, FALSE, &htim2, TIM_CHANNEL_1);
	angle_PID = PID_create(angle_PID_coefs, 300, MAX_SPEED, 250, 0.1);
	speed_PID = PID_create(speed_PID_coefs, 200, 1800., 250, 0.99);
	scheduler = scheduler_create();
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
	left_stepper->set_microstepping(left_stepper, STEPS_128);
	right_stepper->set_microstepping(right_stepper, STEPS_128);
	HAL_GPIO_WritePin(MPU_POWER_GPIO_Port, MPU_POWER_Pin, 0);
	HAL_Delay(100);
	HAL_GPIO_WritePin(MPU_POWER_GPIO_Port, MPU_POWER_Pin, 1);
	HAL_Delay(100);
	MPU6050_Init_Custom(&hi2c1);
	HAL_Delay(100);
	while (TRUE)
	{
		if ((MPU6050_CalibrateGyro() == HAL_OK) && (gyro_cali_x != 0 || gyro_cali_y != 0 || gyro_cali_z != 0)) break;
		send_string("Gyro Cali ERROR, retry\n");
		MPU6050_RestartInternal();
	}
	angle = get_angle_acc();
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_UART_Receive_DMA(&huart1, rbuf, 1);
	scheduler->add_to_queue(scheduler, begin_waiting);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  scheduler->handle_next_event(scheduler);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_SYSTICK_Callback(void)
{
	static uint32_t counter = 0;
	counter ++;
	/* Routine while stopped, but not waiting */
	if (state == PROGRAM_CALIBRATING) return;
	if (state == STOPPED)
	{
		/* 250 Hz */
		if (counter % 4 == 0)
		{
			scheduler->add_to_queue(scheduler, measure_angle);
		}
		/* 2 Hz */
		if (counter % 500 == 0)
		{
			scheduler->add_to_queue(scheduler, send_telemetry);
		}

	}
	/* Routine while waiting */
	if (state == WAITING_FOR_LAUNCH)
	{
		/* 250 Hz */
		if (counter % 4 == 0)
		{
			scheduler->add_to_queue(scheduler, measure_angle);
		}
		/* 100 Hz */
		if (counter % 10 == 0)
		{
			scheduler->add_to_queue(scheduler, wait_for_angle);
		}
		/* 2 Hz */
		if (counter % 500 == 0)
		{
			scheduler->add_to_queue(scheduler, send_telemetry);
		}

	}
	if (state == LAUNCHED)
			{
		/* Routine while in action */
		/* 250 Hz */
		if (counter % 4 == 0)
		{
			scheduler->add_to_queue(scheduler, measure_angle);
			scheduler->add_to_queue(scheduler, angle_PID_tic);
			scheduler->add_to_queue(scheduler, movement_control_tic);
		}
		/* 100 Hz */
		if (counter % 10 == 0)
		{
			scheduler->add_to_queue(scheduler, right_ramp);
			scheduler->add_to_queue(scheduler, left_ramp);
		}
		/* 10 Hz */
		if (counter % 100 == 0)
		{
			scheduler->add_to_queue(scheduler, emergency_check);
		}
		/* 2 Hz */
		if (counter % 500 == 0)
		{
			scheduler->add_to_queue(scheduler, send_telemetry);
		}

			}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == B1_Pin)
	{
		scheduler->add_to_queue(scheduler, restart);
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/*TEST*/
	/**/
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	scheduler->add_to_queue(scheduler, process_rbuf);
	HAL_UART_Receive_DMA(&huart1, rbuf, 1);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
