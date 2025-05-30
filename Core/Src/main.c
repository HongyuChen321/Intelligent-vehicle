/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2025 STMicroelectronics.
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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ultrasonic.h"
#include "motor.h"
#include "blue_tooth.h"
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void DWT_Init(void);
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
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	Bluetooth_Init();
	DWT_Init();
	Motor_Init();
	
	double right_distance = Ultrasonic_GetDistance_RIGHT();
	double front_distance = Ultrasonic_GetDistance_FRONT();
	double max_distance = 20.0;
	
	double Kp = 2.0;
	double Ki = 0.0;
	double Kd = 1.4;
	
	int mode = 0;
	while (1)
	{
		switch(mode)
		{
			case 0:
			double error_prev = 0.0;
			double error_sum = 0; 
			uint32_t last_time = 0;
			double target_distance = 10.0;
			uint8_t base_speed = 30; 
			while(1)
			{
				//检测距离
				right_distance = GetDistance_Right(right_distance);
				front_distance = GetDistance_Front(front_distance);
				//模式切换
				if(front_distance < 10.0)	//摸墙调头
				{
					mode = 1;
					break;
				}
				if(right_distance > 25.0)	//岔路调头
				{
					mode = 2;
					break;
				}
				
				//PID控制
				if(right_distance > 20.0) {
						right_distance = 20.0; // Cap the distance to avoid excessive values
					 // Avoid negative distance
				}
				Bluetooth_SendString("right_distance:");
				Bluetooth_SendDouble(right_distance, 3);
				
				double error = target_distance - right_distance;
				
				uint32_t current_time = DWT->CYCCNT;
				double dt = (double)(current_time - last_time) / (SystemCoreClock);
				
				if (last_time == 0 || dt <= 0 || dt > 1.0) {
						dt = 0.01;
				}
				double error_diff = (error - error_prev) / dt;
				
				error_sum += error * dt;
				double error_sum_max = 50.0;
				if (error_sum > error_sum_max) error_sum = error_sum_max;
				if (error_sum < -error_sum_max) error_sum = -error_sum_max;

				double pid_output = Kp * error + Ki * error_sum + Kd * error_diff;
				
				double max_correction = base_speed * 0.8;
				if (pid_output > max_correction) pid_output = max_correction;
				if (pid_output < -max_correction) pid_output = -max_correction;

				int left_speed = base_speed + (int)pid_output;
				int right_speed = base_speed - (int)pid_output;
				
				if (left_speed < 0) left_speed = 0;
				if (right_speed < 0) right_speed = 0;
				if (left_speed > MAX_SPEED) left_speed = MAX_SPEED;
				if (right_speed > MAX_SPEED) right_speed = MAX_SPEED;
				
				int left_speed1=1.2*left_speed;//fixed motor signal
				Motor_Left(MOTOR_FORWARD,right_speed );
				Motor_Right(MOTOR_FORWARD, left_speed1);

				Bluetooth_SendString(" Error:");
				Bluetooth_SendDouble(error, 2);
				Bluetooth_SendString(" L:");
				Bluetooth_SendDouble(right_speed, 0);
				Bluetooth_SendString(" R:");
				Bluetooth_SendDouble(left_speed, 0);
				Bluetooth_SendString("\r\n");
				
				Bluetooth_SendPID(Kp, Ki, Kd);
				Bluetooth_Receive_PID(&Kp, &Ki, &Kd);
				
				error_prev = error;
				last_time = current_time;
				
				HAL_Delay(20); 
			}
			break;
			// 摸墙调头
			case 1:
				Motor_Left(MOTOR_STOP, 0);
				Motor_Right(MOTOR_STOP, 0);
				HAL_Delay(200);

				Motor_Left(MOTOR_BACKWARD, 50);   
				Motor_Right(MOTOR_FORWARD, 50);   
				HAL_Delay(900);

				Motor_Left(MOTOR_STOP, 0);
				Motor_Right(MOTOR_STOP, 0); 
				HAL_Delay(200);
				
				mode = 0;
				break;
			// 岔路调头
			case 2:
				Motor_Left(MOTOR_STOP, 0);
				Motor_Right(MOTOR_STOP, 0);
				HAL_Delay(200);
				// 前行一段距离
				Motor_Right(MOTOR_FORWARD, 50);
				Motor_Left(MOTOR_FORWARD, 50);
				HAL_Delay(500);
				Motor_Right(MOTOR_STOP, 0);
				Motor_Left(MOTOR_STOP, 0);
				HAL_Delay(200);
				// 右转90°
				Motor_Right(MOTOR_BACKWARD, 40);
				Motor_Left(MOTOR_FORWARD, 40);
				HAL_Delay(400);
				Motor_Right(MOTOR_STOP, 0);
				Motor_Left(MOTOR_STOP, 0);
				HAL_Delay(200);
				// 前行一段距离
				Motor_Right(MOTOR_FORWARD, 50);
				Motor_Left(MOTOR_FORWARD, 50);
				HAL_Delay(1000);
				Motor_Right(MOTOR_STOP, 0);
				Motor_Left(MOTOR_STOP, 0);
				HAL_Delay(200);
				// 右转90°
				Motor_Right(MOTOR_BACKWARD, 40);
				Motor_Left(MOTOR_FORWARD, 40);
				HAL_Delay(400);
				Motor_Right(MOTOR_STOP, 0);
				Motor_Left(MOTOR_STOP, 0);
				HAL_Delay(200);
				// 前行一段距离
				Motor_Right(MOTOR_FORWARD, 60);
				Motor_Left(MOTOR_FORWARD, 60);
				HAL_Delay(600);
				Motor_Right(MOTOR_STOP, 0);
				Motor_Left(MOTOR_STOP, 0);
				HAL_Delay(200);
				// 切换模式+
				mode = 0;
				break;
		}	
	}
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;                               
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;            
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
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
