#ifndef L9963E_INTERFACE_H
#define L9963E_INTERFACE_H

#include "L9963E_status.h"

#include <inttypes.h>

typedef enum { L9963E_IF_CS, L9963E_IF_TXEN, L9963E_IF_BNE, L9963E_IF_ISOFREQ, L9963E_IF_DIS } L9963E_IF_PINS;

typedef enum {
    L9963E_IF_GPIO_PIN_RESET = 0,
    L9963E_IF_GPIO_PIN_SET,
} L9963E_IF_PinState;

typedef L9963E_IF_PinState (*L9963E_IF_GPIO_ReadPin_Ptr)(L9963E_IF_PINS pin);
typedef L9963E_StatusTypeDef (*L9963E_IF_GPIO_WritePin_Ptr)(L9963E_IF_PINS pin, L9963E_IF_PinState state);
typedef L9963E_StatusTypeDef (*L9963E_IF_SPI_Receive_Ptr)(uint8_t *data, uint8_t size, uint8_t timeout_ms);
typedef L9963E_StatusTypeDef (*L9963E_IF_SPI_Transmit_Ptr)(uint8_t *data, uint8_t size, uint8_t timeout_ms);
typedef uint32_t (*L9963E_IF_GetTickMs_Ptr)(void);
typedef void (*L9963E_IF_DelayMs_Ptr)(uint32_t delay);

struct L9963E_IfStruct {
    L9963E_IF_GPIO_ReadPin_Ptr L9963E_IF_GPIO_ReadPin;
    L9963E_IF_GPIO_WritePin_Ptr L9963E_IF_GPIO_WritePin;
    L9963E_IF_SPI_Receive_Ptr L9963E_IF_SPI_Receive;
    L9963E_IF_SPI_Transmit_Ptr L9963E_IF_SPI_Transmit;
    L9963E_IF_GetTickMs_Ptr L9963E_IF_GetTickMs;
    L9963E_IF_DelayMs_Ptr L9963E_IF_DelayMs;
};
typedef struct L9963E_IfStruct L9963E_IfTypeDef;

#endif  //L9963E_INTERFACE_H