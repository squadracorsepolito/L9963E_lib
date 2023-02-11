#ifndef STM32_IF_H
#define STM32_IF_H

#include "L9963E_interface.h"

L9963E_IF_PinState GPIO_ReadPin(L9963E_IF_PINS pin);
L9963E_StatusTypeDef GPIO_WritePin(L9963E_IF_PINS pin, L9963E_IF_PinState state);
L9963E_StatusTypeDef SPI_Receive(uint8_t *data, uint8_t size, uint8_t timeout_ms);
L9963E_StatusTypeDef SPI_Transmit(uint8_t *data, uint8_t size, uint8_t timeout_ms);
uint32_t GetTickMs(void);
void DelayMs(uint32_t delay);

#endif  //STM32_IF_H