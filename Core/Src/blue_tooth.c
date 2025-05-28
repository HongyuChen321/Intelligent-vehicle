#include "blue_tooth.h"
#include "usart.h"

void Bluetooth_Init(void)
{
    Bluetooth_SendString("Bluetooth (USART3) Ready");
}

void Bluetooth_SendString(const char *str)
{
    char buffer[256];
    snprintf(buffer, sizeof(buffer), "%s", str);
    HAL_UART_Transmit(&BLUETOOTH_UART, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
}

void Bluetooth_SendRaw(const uint8_t *data, uint16_t len)
{
    HAL_UART_Transmit(&BLUETOOTH_UART, (uint8_t *)data, len, HAL_MAX_DELAY);
}

void Bluetooth_SendDouble(double value, int precision)
{
    char buffer[64];
    snprintf(buffer, sizeof(buffer), "%.*f\r\n", precision, value);
    HAL_UART_Transmit(&BLUETOOTH_UART, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
}

void Bluetooth_SendPID(double Kp, double Ki, double Kd)
{
		char buffer[100];
    snprintf(buffer, sizeof(buffer), "Kp:%.2f Ki:%.2f Kd:%.2f\r\n", Kp, Ki, Kd);
    HAL_UART_Transmit(&BLUETOOTH_UART, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
}

void Bluetooth_Receive_PID(double *Kp, double *Ki, double *Kd)
{
    uint8_t buffer[1];

    if (Bluetooth_Receive(buffer, 1, 50) == HAL_OK)
    {
        switch (buffer[0])
        {
            case 'A':
                *Kp += 0.1;
                break;
            case 'D':
                if (*Kp > 0.1) *Kp -= 0.1;
                break;

            case 'B':
                *Ki += 0.1;
                break;
            case 'E':
                if (*Ki >= 0.1) *Ki -= 0.1;
                break;

            case 'C':
                *Kd += 0.1;
                break;
            case 'F':
                if (*Kd >= 0.1) *Kd -= 0.1;
                break;

            default:
                break;
        }
    }
}


HAL_StatusTypeDef Bluetooth_Receive(uint8_t *buffer, uint16_t size, uint32_t timeout)
{
    return HAL_UART_Receive(&BLUETOOTH_UART, buffer, size, timeout);
}