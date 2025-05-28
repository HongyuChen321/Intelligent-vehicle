#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f1xx_hal.h"
#include "gpio.h" 

#define MOTOR_LEFT_IN1		TIM_CHANNEL_3
#define MOTOR_LEFT_IN2		TIM_CHANNEL_4
#define MOTOR_RIGHT_IN3		TIM_CHANNEL_2
#define MOTOR_RIGHT_IN4		TIM_CHANNEL_1

#define MAX_SPEED 100

typedef enum {
    MOTOR_STOP = 0,
    MOTOR_FORWARD,
    MOTOR_BACKWARD
} Motor_Direction;

void Motor_Init(void);
void Motor_Left(Motor_Direction direction,uint8_t speed);
void Motor_Right(Motor_Direction direction,uint8_t speed);


#endif  // __MOTOR_H
