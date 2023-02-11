/*
 * "THE BEER-WARE LICENSE" (Revision 69):
 * Squadra Corse firmware team wrote this file. As long as you retain this notice
 * you can do whatever you want with this stuff. If we meet some day, and you 
 * think this stuff is worth it, you can buy us a beer in return.
 * 
 * Authors
 * - Federico Carbone [federico.carbone.sc@gmail.com]
 */

#ifndef L9963E_DRV_H
#define L9963E_DRV_H

#include "L9963E_burst.h"
#include "L9963E_interface.h"
#include "L9963E_registers.h"
#include "L9963E_status.h"

#include <inttypes.h>

#ifndef L9963E_DEBUG
#define L9963E_DEBUG 1
#endif  //L9963E_DEBUG

#define L9963E_DRV_CS_HIGH(HANDLE) ((HANDLE)->interface.L9963E_IF_GPIO_WritePin(L9963E_IF_CS, L9963E_IF_GPIO_PIN_SET))
#define L9963E_DRV_CS_LOW(HANDLE)  ((HANDLE)->interface.L9963E_IF_GPIO_WritePin(L9963E_IF_CS, L9963E_IF_GPIO_PIN_RESET))

#define L9963E_DRV_TXEN_HIGH(HANDLE) \
    ((HANDLE)->interface.L9963E_IF_GPIO_WritePin(L9963E_IF_TXEN, L9963E_IF_GPIO_PIN_SET))
#define L9963E_DRV_TXEN_LOW(HANDLE) \
    ((HANDLE)->interface.L9963E_IF_GPIO_WritePin(L9963E_IF_TXEN, L9963E_IF_GPIO_PIN_RESET))

#define L9963E_DRV_ISOFREQ_HIGH(HANDLE) \
    ((HANDLE)->interface.L9963E_IF_GPIO_WritePin(L9963E_IF_ISOFREQ, L9963E_IF_GPIO_PIN_SET))
#define L9963E_DRV_ISOFREQ_LOW(HANDLE) \
    ((HANDLE)->interface.L9963E_IF_GPIO_WritePin(L9963E_IF_ISOFREQ, L9963E_IF_GPIO_PIN_RESET))

#define L9963E_DRV_BNE_READ(HANDLE) ((HANDLE)->interface.L9963E_IF_GPIO_ReadPin(L9963E_IF_BNE))

#define L9963E_DRV_SPI_RECEIVE(HANDLE, DATA, SIZE, TIMEOUT_MS) \
    ((HANDLE)->interface.L9963E_IF_SPI_Receive((DATA), (SIZE), (TIMEOUT_MS)))
#define L9963E_DRV_SPI_TRANSMIT(HANDLE, DATA, SIZE, TIMEOUT_MS) \
    ((HANDLE)->interface.L9963E_IF_SPI_Transmit((DATA), (SIZE), (TIMEOUT_MS)))

#define L9963E_DRV_GETTICK(HANDLE)      ((HANDLE)->interface.L9963E_IF_GetTickMs())
#define L9963E_DRV_DELAY(HANDLE, DELAY) ((HANDLE)->interface.L9963E_IF_DelayMs((DELAY)))

#define L9963E_DEVICE_BROADCAST 0x0U

union L9963E_DRV_FrameUnion {
    uint64_t val;
    struct {
        uint64_t crc : 6, data : 18, gsw : 2, addr : 7, devid : 5, rw_burst : 1, pa : 1, : 24;
    } cmd;
};
typedef union L9963E_DRV_FrameUnion L9963E_DRV_CmdTypeDef;

struct L9963E_DRV_HandleStruct {
    L9963E_IfTypeDef interface;
};
typedef struct L9963E_DRV_HandleStruct L9963E_DRV_HandleTypeDef;

/**
 * @brief     Initialize the handle 
 * 
 * @param     handle Reference handle to be initialized
 * @param     interface Struct containing the abstraction interface
 * @return    HAL_OK on success, HAL_ERROR on failure
 */
L9963E_StatusTypeDef L9963E_DRV_init(L9963E_DRV_HandleTypeDef *handle, L9963E_IfTypeDef interface);
/**
 * @brief     Wakes up the IC 
 * 
 * @param     handle Reference to the handle
 * @return    HAL_OK on success, HAL_ERROR on failure
 */
L9963E_StatusTypeDef L9963E_DRV_wakeup(L9963E_DRV_HandleTypeDef *handle);
/**
 * @brief     Reads a register
 * 
 * @param     handle Reference to the handle
 * @param     device Device id (cannot be 0x0)
 * @param     address Address of the register to be read
 * @param     data Reference to the structure to be populated with the read data
 * @return    HAL_OK on success, HAL_ERROR on failure
 */
L9963E_StatusTypeDef L9963E_DRV_reg_read(L9963E_DRV_HandleTypeDef *handle,
                                         uint8_t device,
                                         L9963E_RegistersAddrTypeDef address,
                                         L9963E_RegisterUnionTypeDef *data,
                                         uint8_t timeout);
/**
 * @brief     Writes a register
 * 
 * @param     handle Reference to the handle
 * @param     device Device id (can be 0x0 for broadcast)
 * @param     address Address of the register to be read
 * @param     data Data to be written
 * @return    HAL_OK on success, HAL_ERROR on failure
 */
L9963E_StatusTypeDef L9963E_DRV_reg_write(L9963E_DRV_HandleTypeDef *handle,
                                          uint8_t device,
                                          L9963E_RegistersAddrTypeDef address,
                                          L9963E_RegisterUnionTypeDef *data,
                                          uint8_t timeout);
/**
 * @brief     Calculates the CRC6
 * 
 * @param     InputWord Data to be used for the calculation
 * @return    the calculated CRC6
 */
uint8_t L9963E_DRV_crc_calc(uint64_t InputWord);

L9963E_StatusTypeDef L9963E_DRV_burst_cmd(L9963E_DRV_HandleTypeDef *handle,
                                          uint8_t device,
                                          L9963E_BurstCmdTypeDef command,
                                          L9963E_BurstUnionTypeDef *data,
                                          uint8_t expected_frames_n,
                                          uint8_t timeout);

#endif  //L9963E_DRV_H