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

enum L9963E_CellsEnum {
    L9963E_CELL1 = 1,
    L9963E_CELL2 = 2,
    L9963E_CELL3 = 4,
    L9963E_CELL4 = 8,
    L9963E_CELL5 = 16,
    L9963E_CELL6 = 32,
    L9963E_CELL7 = 64,
    L9963E_CELL8 = 128,
    L9963E_CELL9 = 256,
    L9963E_CELL10 = 512,
    L9963E_CELL11 = 1024,
    L9963E_CELL12 = 2048,
    L9963E_CELL13 = 4096,
    L9963E_CELL14 = 8192,
};
typedef enum L9963E_CellsEnum L9963E_CellsTypeDef;

enum L9963E_GpiosEnum {
    L9963E_GPIO3 = 1,
    L9963E_GPIO4 = 2,
    L9963E_GPIO5 = 4,
    L9963E_GPIO6 = 8,
    L9963E_GPIO7 = 16,
    L9963E_GPIO8 = 32,
    L9963E_GPIO9 = 64,
};
typedef enum L9963E_GpiosEnum L9963E_GpiosTypeDef;

enum L9963E_StartConvertionOptEnum {
    L9963E_GPIO_CONV = 1,
    L9963E_BAL_TERM_CONV = 2,
    L9963E_CELL_TERM_CONV = 4,
    L9963E_GPIO_TERM_CONV = 8,
};
typedef enum L9963E_StartConvertionOptEnum L9963E_StartConvertionOptTypeDef;

enum L9963E_CommTimeoutEnum { _32MS = 0b00, _256MS = 0b01, _1024MS = 0b10, _2048MS = 0b11 };
typedef enum L9963E_CommTimeoutEnum L9963E_CommTimeoutTypeDef;

L9963E_StatusTypeDef L9963E_init(L9963E_HandleTypeDef *handle, L9963E_IfTypeDef interface, uint8_t slave_n);
L9963E_StatusTypeDef L9963E_addressing_procedure(L9963E_HandleTypeDef *handle,
                                                 uint8_t iso_freq_sel,
                                                 uint8_t is_dual_ring,
                                                 uint8_t out_res_tx_iso,
                                                 uint8_t lock_isofreq);
L9963E_StatusTypeDef L9963E_setCommTimeout(L9963E_HandleTypeDef *handle,
                                           L9963E_CommTimeoutTypeDef commTimeout,
                                           uint8_t device,
                                           uint8_t preserve_reg_value);

L9963E_StatusTypeDef L9963E_set_enabled_cells(L9963E_HandleTypeDef *handle, uint8_t device, uint16_t cells);
L9963E_StatusTypeDef L9963E_enable_vref(L9963E_HandleTypeDef *handle, uint8_t device, uint8_t preserve_reg_value);
L9963E_StatusTypeDef L9963E_start_conversion(L9963E_HandleTypeDef *handle, uint8_t device, uint8_t options);
L9963E_StatusTypeDef L9963E_poll_conversion(L9963E_HandleTypeDef *handle, uint8_t device, uint8_t *conversion_done);
L9963E_StatusTypeDef L9963E_read_cell_voltage(L9963E_HandleTypeDef *handle, uint8_t device, L9963E_CellsTypeDef cell, uint16_t *vcell, uint8_t *data_ready);
L9963E_StatusTypeDef L9963E_read_batt_voltage(L9963E_HandleTypeDef *handle, uint8_t device, uint16_t *vbatt);
L9963E_StatusTypeDef L9963E_read_gpio_voltage(L9963E_HandleTypeDef *handle, uint8_t device, L9963E_GpiosTypeDef gpio, uint16_t *vgpio, uint8_t *data_ready);

#endif  // L9963E_H