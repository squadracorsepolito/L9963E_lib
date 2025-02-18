#ifndef L9963E_INTERFACE_H
#define L9963E_INTERFACE_H

#include "L9963E_status.h"

#include <inttypes.h>

typedef enum { L9963E_IF_CS, L9963E_IF_TXEN, L9963E_IF_BNE, L9963E_IF_ISOFREQ, L9963E_IF_DIS } L9963E_IF_PINS;

typedef enum {
    L9963E_IF_GPIO_PIN_RESET = 0,
    L9963E_IF_GPIO_PIN_SET,
} L9963E_IF_PinState;
/**
 * @brief Reads the value of a GPIO pin.
 * 
 * @param pin Target pin.
 * @return L9963E_IF_PinState (L9963E_IF_GPIO_PIN_SET if set, L9963E_IF_GPIO_PIN_RESET if reset).
 */
typedef L9963E_IF_PinState (*L9963E_IF_GPIO_ReadPin_Ptr)(L9963E_IF_PINS pin);
/**
 * @brief Sets the value of a GPIO pin.
 * 
 * @param pin Target pin.
 * @return L9963E status (L9963E_OK on success, L9963E_ERROR on failure).
 */
typedef L9963E_StatusTypeDef (*L9963E_IF_GPIO_WritePin_Ptr)(L9963E_IF_PINS pin, L9963E_IF_PinState state);
/**
 * @brief Receives a bunch of data from SPI.
 * 
 * @param data Pointer to store received data.
 * @param size Amount of bytes to be received.
 * @param timeout_ms Maximum time ms before failing.
 * @return L9963E status (L9963E_OK on success, L9963E_ERROR on failure, L9963E_TIMEOUT on timeout).
 */
typedef L9963E_StatusTypeDef (*L9963E_IF_SPI_Receive_Ptr)(uint8_t *data, uint8_t size, uint8_t timeout_ms);
/**
 * @brief Transmit a bunch of data through SPI.
 * 
 * @param data Pointer to data to be sent.
 * @param size Amount of bytes to be sent.
 * @param timeout_ms Maximum time in ms before failing.
 * @return L9963E status (L9963E_OK on success, L9963E_ERROR on failure, L9963E_TIMEOUT on timeout).
 */
typedef L9963E_StatusTypeDef (*L9963E_IF_SPI_Transmit_Ptr)(uint8_t *data, uint8_t size, uint8_t timeout_ms);
/**
 * @brief Gets actual time in ms.
 * 
 * @return Time expressed in ms.
 */
typedef uint32_t (*L9963E_IF_GetTickMs_Ptr)(void);
/**
 * @brief Implement a delay.
 * 
 * @param delay Amount of time to wait in ms.
 */
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