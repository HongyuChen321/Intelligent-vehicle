#include "motor.h"
#include "gpio.h"
#include "tim.h"

void Motor_Init(void)
{
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
		HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
}
 
void Motor_Left(Motor_Direction direction,uint8_t speed)
{
		if(speed > MAX_SPEED)
		{
			speed = MAX_SPEED;
		}
		int period = speed * 999 / MAX_SPEED;
		switch(direction)
		{
			case MOTOR_FORWARD:
				__HAL_TIM_SET_COMPARE(&htim2, MOTOR_LEFT_IN1, period);
				__HAL_TIM_SET_COMPARE(&htim2, MOTOR_LEFT_IN2, 0);
				break;
			case MOTOR_BACKWARD:
				__HAL_TIM_SET_COMPARE(&htim2, MOTOR_LEFT_IN1, 0);
				__HAL_TIM_SET_COMPARE(&htim2, MOTOR_LEFT_IN2, period);
				break;
			case MOTOR_STOP:
				__HAL_TIM_SET_COMPARE(&htim2, MOTOR_LEFT_IN1, 0);
				__HAL_TIM_SET_COMPARE(&htim2, MOTOR_LEFT_IN2, 0);
				break;
		}
}

void Motor_Right(Motor_Direction direction,uint8_t speed)
{
		if(speed > MAX_SPEED)
		{
			speed = MAX_SPEED;
		}
		int period = speed * 999 / MAX_SPEED;
		switch(direction)
		{
			case MOTOR_FORWARD:
				__HAL_TIM_SET_COMPARE(&htim2, MOTOR_RIGHT_IN3, period);
				__HAL_TIM_SET_COMPARE(&htim2, MOTOR_RIGHT_IN4, 0);
				break;
			case MOTOR_BACKWARD:
				__HAL_TIM_SET_COMPARE(&htim2, MOTOR_RIGHT_IN3, 0);
				__HAL_TIM_SET_COMPARE(&htim2, MOTOR_RIGHT_IN4, period);
				break;
			case MOTOR_STOP:
				__HAL_TIM_SET_COMPARE(&htim2, MOTOR_RIGHT_IN3, 0);
				__HAL_TIM_SET_COMPARE(&htim2, MOTOR_RIGHT_IN4, 0);
				break;
		}
}
 
 