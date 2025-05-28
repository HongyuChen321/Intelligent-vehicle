#include "ultrasonic.h"
#include "gpio.h"

static void Delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * (HAL_RCC_GetHCLKFreq() / 1000000);
    while ((DWT->CYCCNT - start) < ticks);
}

double Ultrasonic_GetDistance_RIGHT(void)
{
    uint32_t startTick, stopTick, tickDiff;
    double distance;
    uint32_t timeout;

    HAL_GPIO_WritePin(ULTRASONIC_TRIG_PORT_RIGHT, ULTRASONIC_TRIG_PIN_RIGHT, GPIO_PIN_SET);
    Delay_us(10);
    HAL_GPIO_WritePin(ULTRASONIC_TRIG_PORT_RIGHT, ULTRASONIC_TRIG_PIN_RIGHT, GPIO_PIN_RESET);

    timeout = HAL_GetTick() + 50;
    while (HAL_GPIO_ReadPin(ULTRASONIC_ECHO_PORT_RIGHT, ULTRASONIC_ECHO_PIN_RIGHT) == GPIO_PIN_RESET)
    {
        if (HAL_GetTick() > timeout) return -1; 
    }
    startTick = DWT->CYCCNT;

    timeout = HAL_GetTick() + 50;
    while (HAL_GPIO_ReadPin(ULTRASONIC_ECHO_PORT_RIGHT, ULTRASONIC_ECHO_PIN_RIGHT) == GPIO_PIN_SET)
    {
        if (HAL_GetTick() > timeout) return -1;
    }
    stopTick = DWT->CYCCNT;

    tickDiff = stopTick - startTick;
    double time = tickDiff / (double)HAL_RCC_GetHCLKFreq();
    distance = time * 34000 / 2; 

    return distance;
}

double Ultrasonic_GetDistance_FRONT(void)
{
    uint32_t startTick, stopTick, tickDiff;
    double distance;
    uint32_t timeout;

    HAL_GPIO_WritePin(ULTRASONIC_TRIG_PORT_FRONT, ULTRASONIC_TRIG_PIN_FRONT, GPIO_PIN_SET);
    Delay_us(10);
    HAL_GPIO_WritePin(ULTRASONIC_TRIG_PORT_FRONT, ULTRASONIC_TRIG_PIN_FRONT, GPIO_PIN_RESET);

    timeout = HAL_GetTick() + 50;
    while (HAL_GPIO_ReadPin(ULTRASONIC_ECHO_PORT_FRONT, ULTRASONIC_ECHO_PIN_FRONT) == GPIO_PIN_RESET)
    {
        if (HAL_GetTick() > timeout) return -1; 
    }
    startTick = DWT->CYCCNT;

    timeout = HAL_GetTick() + 50;
    while (HAL_GPIO_ReadPin(ULTRASONIC_ECHO_PORT_FRONT, ULTRASONIC_ECHO_PIN_FRONT) == GPIO_PIN_SET)
    {
        if (HAL_GetTick() > timeout) return -1;
    }
    stopTick = DWT->CYCCNT;

    tickDiff = stopTick - startTick;
    double time = tickDiff / (double)HAL_RCC_GetHCLKFreq();
    distance = time * 34000 / 2; 

    return distance;
}

double GetDistance_Right(double lastRightDistance)
{
		double temp_distance = Ultrasonic_GetDistance_RIGHT();
		return ALPHA * temp_distance + (1 - ALPHA) * lastRightDistance;
}

double GetDistance_Front(double lastFrontDistance)
{
		double temp_distance = Ultrasonic_GetDistance_FRONT();
		return ALPHA * temp_distance + (1 - ALPHA) * lastFrontDistance;
}