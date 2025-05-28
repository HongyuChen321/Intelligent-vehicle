#ifndef __ULTRASONIC_H
#define __ULTRASONIC_H

#define ULTRASONIC_TRIG_PORT_RIGHT GPIOB
#define ULTRASONIC_TRIG_PIN_RIGHT  GPIO_PIN_15
#define ULTRASONIC_ECHO_PORT_RIGHT GPIOB
#define ULTRASONIC_ECHO_PIN_RIGHT  GPIO_PIN_14

#define ULTRASONIC_TRIG_PORT_FRONT GPIOB
#define ULTRASONIC_TRIG_PIN_FRONT  GPIO_PIN_13
#define ULTRASONIC_ECHO_PORT_FRONT GPIOB
#define ULTRASONIC_ECHO_PIN_FRONT  GPIO_PIN_12

#define ALPHA 0.8


#include "gpio.h"
#include "stm32f1xx_hal.h"

/**
 * @brief
 * @return
 */
double Ultrasonic_GetDistance_RIGHT(void);
double Ultrasonic_GetDistance_FRONT(void);

double GetDistance_Right(double lastRightDistance);
double GetDistance_Front(double lastFrontDistance);

#endif /* __ULTRASONIC_H */
