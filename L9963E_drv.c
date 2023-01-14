/*
 * "THE BEER-WARE LICENSE" (Revision 69):
 * Squadra Corse firmware team wrote this file. As long as you retain this notice
 * you can do whatever you want with this stuff. If we meet some day, and you 
 * think this stuff is worth it, you can buy us a beer in return.
 * 
 * Authors
 * - Federico Carbone [federico.carbone.sc@gmail.com]
 */

#include "L9963E_drv.h"

#include <string.h>

#define WORD_LEN           40UL
#define CRC_LEN            6UL
#define CRC_POLY           (uint64_t)0x0000000000000059 /*L9301 CRC Poly:x^3+x^2+x+1*/
#define CRC_SEED           (uint64_t)0x0000000000000038
#define CRC_INIT_SEED_MASK (CRC_SEED << (WORD_LEN - CRC_LEN))
#define CRC_INIT_MASK      (CRC_POLY << (WORD_LEN - CRC_LEN - 1))
#define FIRST_BIT_MASK     ((uint64_t)1 << (WORD_LEN - 1))  // 0x80000000
#define CRC_LOWER_MASK     ((uint8_t)(1 << CRC_LEN) - 1)    //0b111

uint8_t L9963E_DRV_crc_calc(uint64_t InputWord) {
    uint64_t TestBitMask;
    uint64_t CRCMask;
    uint64_t BitCount;
    uint64_t LeftAlignedWord;

    InputWord &= 0xFFFFFFFFFFFFFFC0; /* Clear the CRC bit in the data frame*/
    LeftAlignedWord = InputWord ^ CRC_INIT_SEED_MASK;

    TestBitMask = FIRST_BIT_MASK;
    CRCMask     = CRC_INIT_MASK;  // 1111 <<
    BitCount    = (WORD_LEN - CRC_LEN);
    while (0 != BitCount--) {
        if (0 != (LeftAlignedWord & TestBitMask)) {
            LeftAlignedWord ^= CRCMask;
        } /* endif */
        CRCMask >>= 1;
        TestBitMask >>= 1;
    } /* endwhile */

    return LeftAlignedWord & (uint64_t)CRC_LOWER_MASK;
}

L9963E_StatusTypeDef L9963E_DRV_init(L9963E_DRV_HandleTypeDef *handle,
                                     SPI_HandleTypeDef *hspi,
                                     GPIO_TypeDef *cs_port,
                                     uint16_t cs_pin,
                                     GPIO_TypeDef *txen_port,
                                     uint16_t txen_pin,
                                     GPIO_TypeDef *bne_port,
                                     uint16_t bne_pin,
                                     GPIO_TypeDef *isofreq_port,
                                     uint16_t isofreq_pin) {
#if L9963E_DEBUG
    if (handle == NULL) {
        return L9963E_ERROR;
    }

    if (hspi == NULL) {
        return L9963E_ERROR;
    }

    if (cs_port == NULL) {
        return L9963E_ERROR;
    }

    if (txen_port == NULL) {
        return L9963E_ERROR;
    }

    if (bne_port == NULL) {
        return L9963E_ERROR;
    }

    if (isofreq_port == NULL) {
        return L9963E_ERROR;
    }
#endif

    handle->hspi         = hspi;
    handle->cs_port      = cs_port;
    handle->cs_pin       = cs_pin;
    handle->txen_port    = txen_port;
    handle->txen_pin     = txen_pin;
    handle->bne_port     = bne_port;
    handle->bne_pin      = bne_pin;
    handle->isofreq_port = isofreq_port;
    handle->isofreq_pin  = isofreq_pin;

    L9963E_DRV_CS_HIGH(handle);
    L9963E_DRV_TXEN_HIGH(handle);
    L9963E_DRV_ISOFREQ_LOW(handle);

    return L9963E_OK;
}

L9963E_StatusTypeDef L9963E_DRV_wakeup(L9963E_DRV_HandleTypeDef *handle) {
    static const uint8_t dummy[5]  = {0x55, 0x55, 0x55, 0x55, 0x55};
    L9963E_StatusTypeDef errorcode = L9963E_OK;

#if L9963E_DEBUG
    if (handle == NULL) {
        return L9963E_ERROR;
    }
#endif

    L9963E_DRV_CS_LOW(handle);
    errorcode = HAL_SPI_Transmit(handle->hspi, (uint8_t *)dummy, sizeof(dummy), 10);
    L9963E_DRV_CS_HIGH(handle);

    return errorcode;
}

void _L9963E_DRV_switch_endianness(uint8_t *in, uint8_t *out) {
    out[0] = in[4];
    out[1] = in[3];
    out[2] = in[2];
    out[3] = in[1];
    out[4] = in[0];

    return;
}

L9963E_StatusTypeDef _L9963E_DRV_build_frame(uint8_t *out,
                                             uint8_t pa,
                                             uint8_t rw_burst,
                                             uint8_t devid,
                                             uint8_t addr_command,
                                             uint32_t data) {
    union L9963E_DRV_FrameUnion frame;
    frame.cmd.pa       = pa;
    frame.cmd.rw_burst = rw_burst;
    frame.cmd.devid    = devid;
    frame.cmd.addr     = addr_command;
    frame.cmd.data     = data;
    frame.cmd.crc      = L9963E_DRV_crc_calc(frame.val);

    _L9963E_DRV_switch_endianness((uint8_t *)&frame.val, out);

    return HAL_OK;
}

L9963E_StatusTypeDef _L9963E_DRV_spi_transmit(L9963E_DRV_HandleTypeDef *handle,
                                              uint8_t *data,
                                              uint8_t len,
                                              uint8_t timeout) {
    L9963E_StatusTypeDef errorcode = L9963E_OK;

    L9963E_DRV_TXEN_HIGH(handle);
    L9963E_DRV_CS_LOW(handle);
    errorcode = HAL_SPI_Transmit(handle->hspi, data, len, timeout);
    L9963E_DRV_TXEN_LOW(handle);
    L9963E_DRV_CS_HIGH(handle);

    return errorcode;
}

L9963E_StatusTypeDef _L9963E_DRV_wait_and_receive(union L9963E_DRV_FrameUnion *frame,
                                                  L9963E_DRV_HandleTypeDef *handle,
                                                  uint8_t device,
                                                  uint32_t current_tick,
                                                  uint8_t timeout) {
    uint8_t raw[5];
    L9963E_StatusTypeDef errorcode = L9963E_OK;

    frame->cmd.addr  = -1;
    frame->cmd.devid = -1;
    frame->cmd.data  = -1;

    L9963E_DRV_TXEN_LOW(handle);
    while (frame->cmd.devid != device) {
        while (L9963E_DRV_BNE_READ(handle) == GPIO_PIN_RESET) {
            if (HAL_GetTick() - current_tick >= timeout) {
                L9963E_DRV_TXEN_HIGH(handle);
                return L9963E_TIMEOUT;
            }
        }

        L9963E_DRV_CS_LOW(handle);
        errorcode = HAL_SPI_Receive(handle->hspi, raw, 5, 10);
        L9963E_DRV_CS_HIGH(handle);
        L9963E_DRV_TXEN_HIGH(handle);

        if (errorcode != L9963E_OK) {
            return errorcode;
        }

        _L9963E_DRV_switch_endianness(raw, (uint8_t *)&frame->val);

        if (frame->cmd.crc != L9963E_DRV_crc_calc(frame->val)) {
            return L9963E_CRC_ERROR;
        }
    }

    return HAL_OK;
}

L9963E_StatusTypeDef L9963E_DRV_burst_cmd(L9963E_DRV_HandleTypeDef *handle,
                                          uint8_t device,
                                          L9963E_BurstCmdTypeDef command,
                                          L9963E_BurstUnionTypeDef *data,
                                          uint8_t expected_frames_n,
                                          uint8_t timeout) {
    union L9963E_DRV_FrameUnion frame;
    L9963E_StatusTypeDef errorcode = L9963E_OK;
    uint32_t current_tick;
    uint8_t raw[5];

#if L9963E_DEBUG
    if (handle == NULL) {
        return L9963E_ERROR;
    }

    if (data == NULL) {
        return L9963E_ERROR;
    }
#endif

    _L9963E_DRV_build_frame(raw, 1, 0, device, command, 0);

    errorcode = _L9963E_DRV_spi_transmit(handle, raw, 5, 10);

    if (errorcode != L9963E_OK) {
        return errorcode;
    }

    current_tick = HAL_GetTick();
    for (uint8_t i = 0; i < expected_frames_n; ++i) {
        errorcode = _L9963E_DRV_wait_and_receive(&frame, handle, device, current_tick, timeout);

        if (errorcode != L9963E_OK) {
            return errorcode;
        }

        if (frame.cmd.addr == command)
            frame.cmd.addr = 1;

        data->generics[(frame.cmd.addr & 0b11111) - 1] = frame.cmd.data;
    }

    return L9963E_OK;
}

L9963E_StatusTypeDef _L9963E_DRV_reg_cmd(L9963E_DRV_HandleTypeDef *handle,
                                         uint8_t is_write,
                                         uint8_t device,
                                         L9963E_RegistersAddrTypeDef address,
                                         L9963E_RegisterUnionTypeDef *data,
                                         uint8_t timeout) {
    union L9963E_DRV_FrameUnion frame;
    L9963E_StatusTypeDef errorcode = L9963E_OK;
    uint8_t raw[5];

#if L9963E_DEBUG
    if (handle == NULL) {
        return L9963E_ERROR;
    }

    if (data == NULL) {
        return L9963E_ERROR;
    }
#endif

    _L9963E_DRV_build_frame(raw, 1, is_write ? 1 : 0, device, address, is_write ? data->generic : 0);

    errorcode = _L9963E_DRV_spi_transmit(handle, raw, 5, 10);

    if (errorcode != L9963E_OK) {
        return errorcode;
    }

    errorcode = _L9963E_DRV_wait_and_receive(&frame, handle, device, HAL_GetTick(), timeout);

    if (errorcode != L9963E_OK) {
        return errorcode;
    }

    if (is_write && frame.cmd.data != data->generic) {
        return L9963E_READBACK_ERROR;
    }

    data->generic = frame.cmd.data;

    return L9963E_OK;
}

L9963E_StatusTypeDef L9963E_DRV_reg_read(L9963E_DRV_HandleTypeDef *handle,
                                         uint8_t device,
                                         L9963E_RegistersAddrTypeDef address,
                                         L9963E_RegisterUnionTypeDef *data,
                                         uint8_t timeout) {
#if L9963E_DEBUG
    if (device == 0) {
        return L9963E_ERROR;
    }
#endif

    return _L9963E_DRV_reg_cmd(handle, 0, device, address, data, timeout);
}

L9963E_StatusTypeDef L9963E_DRV_reg_write(L9963E_DRV_HandleTypeDef *handle,
                                          uint8_t device,
                                          L9963E_RegistersAddrTypeDef address,
                                          L9963E_RegisterUnionTypeDef *data,
                                          uint8_t timeout) {
    return _L9963E_DRV_reg_cmd(handle, 1, device, address, data, timeout);
}