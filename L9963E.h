/*
 * "THE BEER-WARE LICENSE" (Revision 69):
 * Squadra Corse firmware team wrote this file. As long as you retain this notice
 * you can do whatever you want with this stuff. If we meet some day, and you 
 * think this stuff is worth it, you can buy us a beer in return.
 * 
 * Authors
 * - Federico Carbone [federico.carbone.sc@gmail.com]
 */

#ifndef L9963E_H
#define L9963E_H

#ifndef L9963E_DEBUG
#define L9963E_DEBUG 1
#endif  //L9963E_DEBUG

#include "L9963E_drv.h"
#include "L9963E_status.h"

struct L9963E_HandleStruct {
    L9963E_DRV_HandleTypeDef drv_handle;
    uint8_t slave_n;
    uint8_t is_dual_ring;
    uint8_t out_res_tx_iso;
};
typedef struct L9963E_HandleStruct L9963E_HandleTypeDef;

enum L9963E_CommTimeoutEnum { _32MS = 0b00, _256MS = 0b01, _1024MS = 0b10, _2048MS = 0b11 };
typedef enum L9963E_CommTimeoutEnum L9963E_CommTimeoutTypeDef;

L9963E_StatusTypeDef L9963E_init(L9963E_HandleTypeDef *handle,
                                 SPI_HandleTypeDef *hspi,
                                 GPIO_TypeDef *cs_port,
                                 uint16_t cs_pin,
                                 GPIO_TypeDef *txen_port,
                                 uint16_t txen_pin,
                                 GPIO_TypeDef *bne_port,
                                 uint16_t bne_pin,
                                 GPIO_TypeDef *isofreq_port,
                                 uint16_t isofreq_pin,
                                 uint8_t slave_n);
L9963E_StatusTypeDef L9963E_addressing_procedure(L9963E_HandleTypeDef *handle,
                                                 uint8_t iso_freq_sel,
                                                 uint8_t is_dual_ring,
                                                 uint8_t out_res_tx_iso,
                                                 uint8_t lock_isofreq);
L9963E_StatusTypeDef L9963E_setCommTimeout_Broadcast(L9963E_HandleTypeDef *handle,
                                                     L9963E_CommTimeoutTypeDef commTimeout);
L9963E_StatusTypeDef L9963E_setCommTimeout(L9963E_HandleTypeDef *handle,
                                           L9963E_CommTimeoutTypeDef commTimeout,
                                           uint8_t device,
                                           uint8_t preserve_reg_value);

#endif  // L9963E_H