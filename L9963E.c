/*
 * "THE BEER-WARE LICENSE" (Revision 69):
 * Squadra Corse firmware team wrote this file. As long as you retain this notice
 * you can do whatever you want with this stuff. If we meet some day, and you 
 * think this stuff is worth it, you can buy us a beer in return.
 * 
 * Authors
 * - Federico Carbone [federico.carbone.sc@gmail.com]
 */

#include "L9963E.h"

#include <string.h>

const uint16_t CrcCalc8bitLookupTab_B[] = {
    0x3123, 0x0005, 0x8474, 0x3143, 0x0007, 0x1809, 0xa800, 0x74e0, 0x0031, 0x9404, 0xe228, 0x3104, 0x0000, 0x18aa,
    0xa801, 0x8603, 0x7d07, 0x0278, 0x74e5, 0x063f, 0x7d63, 0x2a14, 0x316b, 0x000b, 0x7a05, 0x00fe, 0x192a, 0x84fe,
    0x7c8c, 0x2378, 0x7528, 0x063f, 0x1808, 0x8001, 0x00b0, 0x18ec, 0x0001, 0x7ce6, 0x5a78, 0x7ca3, 0x3214, 0x3165,
    0x000b, 0x7a20, 0xfff0, 0x7c84, 0x50ae, 0x7d67, 0x2278, 0xe844, 0x1989, 0xb008, 0x8054, 0x3104, 0x0001, 0x18aa,
    0xa801, 0x758c, 0x063f, 0x8663, 0x7cab, 0x4830, 0x7d07, 0x6630, 0x7d60, 0x3b78, 0x7c8b, 0x2378, 0x7c05, 0x3278,
    0x4816, 0x74a7, 0x063f, 0x7ca3, 0x3a14, 0x8b55, 0x7a05, 0x0090, 0x1906, 0x8001, 0x18eb, 0x0001, 0x7506, 0x063f,
    0x7ce7, 0x4830, 0x7d04, 0x30ae, 0x7c06, 0x5040, 0x7d00, 0x6630, 0x4407, 0x74e0, 0x063f, 0x7ca5, 0x0278, 0x7ce3,
    0x2a14, 0x8b57, 0xe2ea, 0x1946, 0x8001, 0x754b, 0x063f, 0x7c84, 0x58ae, 0x7d08, 0x4830, 0x7c89, 0x6630, 0x7d2c,
    0x4378, 0x7ca6, 0x6278, 0x74c7, 0x063f, 0x3143, 0x0008, 0x8a03, 0x180a, 0xa800, 0xe614, 0x192a, 0x84ff, 0x6386,
    0x752c, 0x063f, 0x190c, 0x8001, 0x7d09, 0x03a6, 0x7c0b, 0x3a78, 0x7cc8, 0x3839, 0x6810, 0x6816, 0x7ce7, 0x589e,
    0x7a20, 0xfff0, 0x8933, 0x7ce5, 0x1e30, 0x74a3, 0x063f, 0x0004, 0x7160, 0x0002, 0xe8d3, 0x7140, 0x0001, 0xe894};

#define WORD_LEN           40UL
#define CRC_LEN            6UL
#define CRC_POLY           (uint64_t)0x0000000000000059 /*L9301 CRC Poly:x^3+x^2+x+1*/
#define CRC_SEED           (uint64_t)0x0000000000000038
#define CRC_INIT_SEED_MASK (CRC_SEED << (WORD_LEN - CRC_LEN))
#define CRC_INIT_MASK      (CRC_POLY << (WORD_LEN - CRC_LEN - 1))
#define FIRST_BIT_MASK     ((uint64_t)1 << (WORD_LEN - 1))  // 0x80000000
#define CRC_LOWER_MASK     ((uint8_t)(1 << CRC_LEN) - 1)    //0b111

uint8_t L9963E_crc_calc(uint64_t InputWord) {
    uint64_t TestBitMask;
    uint64_t CRCMask;
    uint64_t BitCount;
    uint64_t LeftAlignedWord;

    InputWord &= 0xFFFFFFFFFFFFFFC0; /* Clear the CRC bit in the data frame*/
    LeftAlignedWord = InputWord ^ CRC_INIT_SEED_MASK;

    TestBitMask = ((uint64_t)1 << (WORD_LEN - 1));
    CRCMask     = CRC_INIT_MASK;  // 1111 <<
    BitCount    = (WORD_LEN - CRC_LEN);
    while (0 != BitCount--) {
        if (0 != (LeftAlignedWord & TestBitMask)) {
            LeftAlignedWord ^= CRCMask;
        } /* endif */
        CRCMask >>= 1;
        TestBitMask >>= 1;
    } /* endwhile */

    LeftAlignedWord &= (uint64_t)CRC_LOWER_MASK;
    return LeftAlignedWord;
}

HAL_StatusTypeDef L9963E_init(
    L9963E_HandleTypeDef *handle,
    SPI_HandleTypeDef *hspi,
    GPIO_TypeDef *cs_port,
    uint16_t cs_pin,
    GPIO_TypeDef *txen_port,
    uint16_t txen_pin,
    GPIO_TypeDef *bne_port,
    uint16_t bne_pin) {
    if (handle == NULL) {
        return HAL_ERROR;
    }

    if (hspi == NULL) {
        return HAL_ERROR;
    }

    if (cs_port == NULL) {
        return HAL_ERROR;
    }

    if (txen_port == NULL) {
        return HAL_ERROR;
    }

    if (bne_port == NULL) {
        return HAL_ERROR;
    }

    handle->hspi      = hspi;
    handle->cs_port   = cs_port;
    handle->cs_pin    = cs_pin;
    handle->txen_port = txen_port;
    handle->txen_pin  = txen_pin;
    handle->bne_port  = bne_port;
    handle->bne_pin   = bne_pin;

    L9963E_CS_HIGH(handle);
    L9963E_TXEN_HIGH(handle);

    return HAL_OK;
}

HAL_StatusTypeDef L9963E_wakeup(L9963E_HandleTypeDef *handle) {
    uint8_t dummy[5];
    HAL_StatusTypeDef errorcode = HAL_OK;

    if (handle == NULL) {
        return HAL_ERROR;
    }
    L9963E_CS_LOW(handle);
    errorcode = HAL_SPI_Transmit(handle->hspi, dummy, sizeof(dummy), 10);
    L9963E_CS_HIGH(handle);

    return errorcode;
}

HAL_StatusTypeDef _L9963E_reg_cmd(
    L9963E_HandleTypeDef *handle,
    uint8_t is_write,
    uint8_t device,
    L9963E_RegistersAddrTypeDef address,
    L9963E_RegisterUnionTypeDef *data) {
    union L9963E_CmdUnion dat;
    HAL_StatusTypeDef errorcode = HAL_OK;
    uint32_t current_tick;
    uint8_t d[5];

    if (handle == NULL) {
        return HAL_ERROR;
    }

    if (data == NULL) {
        return HAL_ERROR;
    }

    dat.cmd.pa       = 1;
    dat.cmd.rw_burst = is_write ? 1 : 0;
    dat.cmd.devid    = device;
    dat.cmd.addr = address, dat.cmd.data = data->generic;
    dat.cmd.crc = L9963E_crc_calc(dat.val);

    d[0] = *((uint8_t *)&dat.val + 4);
    d[1] = *((uint8_t *)&dat.val + 3);
    d[2] = *((uint8_t *)&dat.val + 2);
    d[3] = *((uint8_t *)&dat.val + 1);
    d[4] = *((uint8_t *)&dat.val + 0);

    L9963E_CS_LOW(handle);
    errorcode = HAL_SPI_Transmit(handle->hspi, d, 5, 10);
    L9963E_CS_HIGH(handle);

    if (errorcode != HAL_OK) {
        return errorcode;
    }

    dat.cmd.addr  = -1;
    dat.cmd.devid = -1;

    current_tick = HAL_GetTick();
    while (dat.cmd.addr != address && dat.cmd.devid != device) {
        while (L9963E_BNE_READ(handle) == GPIO_PIN_RESET) {
            if (HAL_GetTick() - current_tick > 10) {
                return HAL_TIMEOUT;
            }
        }

        L9963E_TXEN_LOW(handle);
        L9963E_CS_LOW(handle);
        errorcode = HAL_SPI_Receive(handle->hspi, d, 5, 100);
        L9963E_CS_HIGH(handle);
        L9963E_TXEN_HIGH(handle);

        *((uint8_t *)&dat.val + 4) = d[0];
        *((uint8_t *)&dat.val + 3) = d[1];
        *((uint8_t *)&dat.val + 2) = d[2];
        *((uint8_t *)&dat.val + 1) = d[3];
        *((uint8_t *)&dat.val + 0) = d[4];

        if (errorcode != HAL_OK) {
            return errorcode;
        }
    }

    data->generic = dat.cmd.data;

    return HAL_OK;
}

HAL_StatusTypeDef L9963E_reg_read(
    L9963E_HandleTypeDef *handle,
    uint8_t device,
    L9963E_RegistersAddrTypeDef address,
    L9963E_RegisterUnionTypeDef *data) {
    if (device == 0) {
        return HAL_ERROR;
    }

    return _L9963E_reg_cmd(handle, 0, device, address, data);
}

HAL_StatusTypeDef L9963E_reg_write(
    L9963E_HandleTypeDef *handle,
    uint8_t device,
    L9963E_RegistersAddrTypeDef address,
    L9963E_RegisterUnionTypeDef *data) {
    return _L9963E_reg_cmd(handle, 1, device, address, data);
}
