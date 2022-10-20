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

/*
#include <stdio.h>
#include <inttypes.h>

void init_CRC6_Table()
{
        uint64_t remainder;
        for(uint8_t dividend=0; dividend < 64; ++dividend) {
                uint8_t currByte = dividend;
                for(uint8_t i=0; i<6; ++i){
                        if(currByte & 0x20){
                                currByte <<= 1;
                        } else {
                                currByte <<= 1;
                                currByte ^= 0b0011001;
                        }
                }
                printf("0x%x", currByte & 0x3f);
                if(dividend != 63)
                    printf(", ");
        }
}

void main(void) {
        printf("{");
        init_CRC6_Table();
        printf("}");
        return;
}
*/

uint8_t crc_table[] = { 0x14, 0xd, 0x26, 0x3f, 0x29, 0x30, 0x1b, 0x2,
                        0x37, 0x2e, 0x5, 0x1c, 0xa, 0x13, 0x38, 0x21,
                        0xb, 0x12, 0x39, 0x20, 0x36, 0x2f, 0x4, 0x1d,
                        0x28, 0x31, 0x1a, 0x3, 0x15, 0xc, 0x27, 0x3e,
                        0x2a, 0x33, 0x18, 0x1, 0x17, 0xe, 0x25, 0x3c,
                        0x9, 0x10, 0x3b, 0x22, 0x34, 0x2d, 0x6, 0x1f,
                        0x35, 0x2c, 0x7, 0x1e, 0x8, 0x11, 0x3a, 0x23,
                        0x16, 0xf, 0x24, 0x3d, 0x2b, 0x32, 0x19, 0x0 };


HAL_StatusTypeDef L9963E_init(L9963E_HandleTypeDef *handle, SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint8_t cs_pin, GPIO_TypeDef *txen_port, uint8_t txen_pin, GPIO_TypeDef *bne_port, uint8_t bne_pin) {
    if(handle == NULL) {
        return HAL_ERROR;
    }

    if(hspi == NULL) {
        return HAL_ERROR;
    }

    if(cs_port == NULL) {
        return HAL_ERROR;
    }

    if(txen_port == NULL) {
        return HAL_ERROR;
    }

    if(bne_port == NULL) {
        return HAL_ERROR;
    }

    handle->hspi = hspi;
    handle->cs_port = cs_port;
    handle->cs_pin = cs_pin;
    handle->txen_port = txen_port;
    handle->txen_pin = txen_pin;
    handle->bne_port = bne_port;
    handle->bne_pin = bne_pin;

    L9963E_CS_HIGH(handle);
    L9963E_TXEN_HIGH(handle);

    return HAL_OK;
}

HAL_StatusTypeDef L9963E_wakeup(L9963E_HandleTypeDef *handle) {
    uint8_t dummy[5];
    HAL_StatusTypeDef errorcode = HAL_OK;

    if(handle == NULL) {
        return HAL_ERROR;
    }
    L9963E_CS_LOW(handle);
    errorcode = HAL_SPI_Transmit(handle->hspi, dummy, sizeof(dummy), 10);
    L9963E_CS_HIGH(handle);

    return errorcode;
}

HAL_StatusTypeDef _L9963E_reg_cmd(L9963E_HandleTypeDef *handle, uint8_t is_write, uint8_t device, L9963E_RegistersTypeDef address, L9963E_RegisterUnionTypeDef *data) {
    L9963E_CmdTypeDef cmd;
    HAL_StatusTypeDef errorcode = HAL_OK;
    uint32_t current_tick;

    if(handle == NULL) {
        return HAL_ERROR;
    }

    if(data == NULL) {
        return HAL_ERROR;
    }

    cmd.pa = 1;
    cmd.rw_burst = is_write ? 1 : 0;
    cmd.devid = device;
    cmd.addr = address,
    cmd.data = is_write ? data->generic : 0;
    cmd.crc = L9963E_crc_calc(&cmd);

    L9963E_CS_LOW(handle);
    errorcode = HAL_SPI_Transmit(handle->hspi, (uint8_t*)&cmd, 5, 10);
    L9963E_CS_HIGH(handle);

    if(errorcode != HAL_OK) {
        return errorcode;
    }

    cmd.addr = -1;
    cmd.devid = -1;

    current_tick = HAL_GetTick();
    while(cmd.addr != address && cmd.devid != device) {
        while(L9963E_BNE_READ(handle) == GPIO_PIN_RESET) {
            if(HAL_GetTick() - current_tick > 10) {
                return HAL_TIMEOUT;
            }
        }

        L9963E_TXEN_LOW(handle);
        errorcode = HAL_SPI_Receive(handle->hspi, (uint8_t*)&cmd, 5, 10);
        L9963E_TXEN_HIGH(handle);

        if(errorcode != HAL_OK) {
            return errorcode;
        }
    }
    
    data->generic = cmd.data;

    return HAL_OK;
}

HAL_StatusTypeDef L9963E_reg_read(L9963E_HandleTypeDef *handle, uint8_t device, L9963E_RegistersTypeDef address, L9963E_RegisterUnionTypeDef *data) {
    if(device == 0) {
        return HAL_ERROR;
    }

    return _L9963E_reg_cmd(handle, 0, device, address, data);
}

HAL_StatusTypeDef L9963E_reg_write(L9963E_HandleTypeDef *handle, uint8_t device, L9963E_RegistersAddrTypeDef address, L9963E_RegisterUnionTypeDef *data) {
    return _L9963E_reg_cmd(handle, 1, device, address, data);
}

uint8_t L9963E_crc_calc(L9963E_CmdTypeDef *cmd) {
    const uint8_t poly = 0b1011001;
    const uint8_t seed = 0b111000;
    //uint8_t index;

    uint64_t data = *((uint64_t*)cmd) & 0xFFFFFFFFC0;
    data |= seed;

    //slowest implementation possible, waiting to be verified in order to use table-based implementation
    for(int8_t shift=33; shift>=0; --shift) {
        if((data & ((uint64_t)1 << (shift+6)))) {
            data ^= ((uint64_t)poly << shift);
        }
    }

    return data;
/*
    for(int8_t shift=30; shift>=0; shift-=6) {
        index = ((d >> shift) & 0x3F) ^ crc;
        crc = crc_table[index];
    }

    return crc;
*/

/*
    // first iteration 
    _6bit |= (data[0] & 0b11000000) >> 6;
    _6bit |= (data[1] & 0b00001111) << 2;

    index = (_6bit ^ crc);
    crc = crc_table[index];
    // end first iteration

    // second iteration 
    _6bit = 0;
    _6bit |= (data[1] & 0b11110000) >> 4;
    _6bit |= (data[2] & 0b00000011) << 4;

    index = (_6bit ^ crc);
    crc = crc_table[index];
    // end second iteration

    // third iteration 
    _6bit = 0;
    _6bit |= (data[2] & 0b11111100) >> 2;

    index = (_6bit ^ crc);
    crc = crc_table[index];
    // end third iteration

    // fourth iteration 
    _6bit = 0;
    _6bit |= (data[3] & 0b00111111);

    index = (_6bit ^ crc);
    crc = crc_table[index];
    // end fourth iteration 

    // fifth iteration 
    _6bit = 0;
    _6bit |= (data[3] & 0b11000000) >> 6;
    _6bit |= (data[4] & 0b00001111) << 2;

    index = (_6bit ^ crc);
    crc = crc_table[index];
    // end fifth iteration 

    // sixth iteration 
    _6bit = 0;
    _6bit |= (data[4] & 0b11110000) >> 4;

    index = (_6bit ^ crc);
    crc = crc_table[index];
    // end sixth iteration 

    return crc;
    */
}
