#ifndef __BLUETOOTH_H
#define __BLUETOOTH_H

#include "stm32f1xx_hal.h"
#include <string.h>
#include <stdio.h>

#define BLUETOOTH_UART      huart3

void Bluetooth_Init(void);

void Bluetooth_SendString(const char *str);
void Bluetooth_SendRaw(const uint8_t *data, uint16_t len);
void Bluetooth_SendDouble(double value, int precision);
void Bluetooth_SendPID(double Kp, double Ki, double Kd);

void Bluetooth_Receive_PID(double *Kp, double *Ki, double *Kd);

HAL_StatusTypeDef Bluetooth_Receive(uint8_t *buffer, uint16_t size, uint32_t timeout);

#endif
