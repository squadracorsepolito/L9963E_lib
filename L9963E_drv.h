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
#include "L9963E_registers.h"
#include "L9963E_status.h"
#include "main.h"

#include <stdint.h>

#define L9963E_DRV_CS_HIGH(HANDLE) HAL_GPIO_WritePin((HANDLE)->cs_port, (HANDLE)->cs_pin, GPIO_PIN_SET)
#define L9963E_DRV_CS_LOW(HANDLE)  HAL_GPIO_WritePin((HANDLE)->cs_port, (HANDLE)->cs_pin, GPIO_PIN_RESET)

#define L9963E_DRV_TXEN_HIGH(HANDLE) HAL_GPIO_WritePin((HANDLE)->txen_port, (HANDLE)->txen_pin, GPIO_PIN_SET)
#define L9963E_DRV_TXEN_LOW(HANDLE)  HAL_GPIO_WritePin((HANDLE)->txen_port, (HANDLE)->txen_pin, GPIO_PIN_RESET)

#define L9963E_DRV_ISOFREQ_HIGH(HANDLE) HAL_GPIO_WritePin((HANDLE)->isofreq_port, (HANDLE)->isofreq_pin, GPIO_PIN_SET)
#define L9963E_DRV_ISOFREQ_LOW(HANDLE)  HAL_GPIO_WritePin((HANDLE)->isofreq_port, (HANDLE)->isofreq_pin, GPIO_PIN_RESET)

#define L9963E_DRV_BNE_READ(HANDLE) HAL_GPIO_ReadPin((HANDLE)->bne_port, (HANDLE)->bne_pin)

#define L9963E_DRV_CMD_MASK 0xFFFFFFFFFF

union L9963E_DRV_CmdUnion {
    uint64_t val;
    struct {
        uint64_t crc : 6, data : 18, gsw : 2, addr : 7, devid : 5, rw_burst : 1, pa : 1, : 24;
    } cmd;
};
typedef union L9963E_DRV_CmdUnion L9963E_DRV_CmdTypeDef;

struct L9963E_DRV_HandleStruct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;

    GPIO_TypeDef *txen_port;
    uint16_t txen_pin;

    GPIO_TypeDef *bne_port;
    uint16_t bne_pin;

    GPIO_TypeDef *isofreq_port;
    uint16_t isofreq_pin;
};
typedef struct L9963E_DRV_HandleStruct L9963E_DRV_HandleTypeDef;
/**
 * @brief     Initialize the handle 
 * 
 * @param     handle Reference handle to be initialized
 * @param     hspi Reference to the spi handle to be used
 * @param     cs_port Reference to the Chip Select port
 * @param     cs_pin Chip Select pin
 * @param     txen_port Reference to the TXEN port
 * @param     txen_pin TXEN pin
 * @param     bne_port Reference to the BNE port
 * @param     bne_pin BNE pin
 * @return    HAL_OK on success, HAL_ERROR on failure
 */
L9963E_StatusTypeDef L9963E_DRV_init(L9963E_DRV_HandleTypeDef *handle,
                                     SPI_HandleTypeDef *hspi,
                                     GPIO_TypeDef *cs_port,
                                     uint16_t cs_pin,
                                     GPIO_TypeDef *txen_port,
                                     uint16_t txen_pin,
                                     GPIO_TypeDef *bne_port,
                                     uint16_t bne_pin,
                                     GPIO_TypeDef *isofreq_port,
                                     uint16_t isofreq_pin);
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

#endif  //L9963E_DRV_DRV_H