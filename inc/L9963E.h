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
    L9963E_CELL1  = 1,
    L9963E_CELL2  = 2,
    L9963E_CELL3  = 4,
    L9963E_CELL4  = 8,
    L9963E_CELL5  = 16,
    L9963E_CELL6  = 32,
    L9963E_CELL7  = 64,
    L9963E_CELL8  = 128,
    L9963E_CELL9  = 256,
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
    L9963E_GPIO_CONV      = 1,
    L9963E_BAL_TERM_CONV  = 2,
    L9963E_CELL_TERM_CONV = 4,
    L9963E_GPIO_TERM_CONV = 8,
};
typedef enum L9963E_StartConvertionOptEnum L9963E_StartConvertionOptTypeDef;

enum L9963E_CommTimeoutEnum { _32MS = 0b00, _256MS = 0b01, _1024MS = 0b10, _2048MS = 0b11 };
typedef enum L9963E_CommTimeoutEnum L9963E_CommTimeoutTypeDef;
/**
 * @brief Initializes the L9963E device.
 * 
 * @param handle Pointer to the L9963E handle structure.
 * @param interface Communication interface type.
 * @param slave_n Slave count.
 * @return L9963E status (L9963E_OK on success, L9963E_ERROR on failure).
 */
L9963E_StatusTypeDef L9963E_init(L9963E_HandleTypeDef *handle, L9963E_IfTypeDef interface, uint8_t slave_n);
/**
 * @brief Performs the addressing procedure for L9963E devices.
 * 
 * @param handle Pointer to the L9963E handle structure.
 * @param iso_freq_sel ISO frequency selection.
 * @param is_dual_ring Dual ring configuration flag.
 * @param out_res_tx_iso Output resistance TX ISO setting.
 * @param lock_isofreq Lock ISO frequency flag.
 * @return L9963E status (L9963E_OK on success, L9963E_ERROR on failure, L9963E_TIMEOUT on timeout).
 */
L9963E_StatusTypeDef L9963E_addressing_procedure(L9963E_HandleTypeDef *handle,
                                                 uint8_t iso_freq_sel,
                                                 uint8_t is_dual_ring,
                                                 uint8_t out_res_tx_iso,
                                                 uint8_t lock_isofreq);
/**
 * @brief Performs a SW reset.
 * 
 * @param handle Pointer to the L9963E handle structure.
 * @param device Target device address.
 * @param go2slp GO2SLP sent in the same frame flag.
 * @return L9963E status (L9963E_OK on success, L9963E_ERROR on failure).
 */
L9963E_StatusTypeDef L9963E_sw_rst(L9963E_HandleTypeDef *handle, uint8_t device, uint8_t go2slp);
/**
 * @brief Retrigger NVM .
 * 
 * @param handle Pointer to the L9963E handle structure.
 * @param device Target device address.
 * @param go2slp GO2SLP sent in the same frame flag.
 * @param preserve_reg_value Flag to preserve register values.
 * @return L9963E status (L9963E_OK on success, L9963E_ERROR on failure).
 */
L9963E_StatusTypeDef L9963E_trimming_retrigger(L9963E_HandleTypeDef *handle,
                                               uint8_t device,
                                               uint8_t preserve_reg_value);
/**
 * @brief Sets the communication timeout for the L9963E device.
 * 
 * @param handle Pointer to the L9963E handle structure.
 * @param commTimeout Communication timeout value.
 * @param device Target device address.
 * @param preserve_reg_value Flag to preserve register values.
 * @return L9963E status (L9963E_OK on success, L9963E_ERROR on failure).
 */
L9963E_StatusTypeDef L9963E_setCommTimeout(L9963E_HandleTypeDef *handle,
                                           L9963E_CommTimeoutTypeDef commTimeout,
                                           uint8_t device,
                                           uint8_t preserve_reg_value);
/**
 * @brief Enables specific battery cells for monitoring.
 * 
 * @param handle Pointer to the L9963E handle structure.
 * @param device Target device address.
 * @param cells Bitmask of enabled cells (ORed L9963E_CellsTypeDef values).
 * @return L9963E status (L9963E_OK on success, L9963E_ERROR on failure).
 */
L9963E_StatusTypeDef L9963E_set_enabled_cells(L9963E_HandleTypeDef *handle, uint8_t device, uint16_t cells);
/**
 * @brief Enables the voltage reference (VREF) for the device.
 * 
 * @param handle Pointer to the L9963E handle structure.
 * @param device Target device address.
 * @param preserve_reg_value Flag to preserve register values.
 * @return L9963E status (L9963E_OK on success, L9963E_ERROR on failure).
 */
L9963E_StatusTypeDef L9963E_enable_vref(L9963E_HandleTypeDef *handle, uint8_t device, uint8_t preserve_reg_value);
/**
 * @brief Starts an ADC conversion for voltage measurements.
 * 
 * @param handle Pointer to the L9963E handle structure.
 * @param device Target device address.
 * @param adc_filter_soc ADC filter setting.
 * @param options Conversion options bitmask (ORed L9963E_StartConvertionOptTypeDef values).
 * @return L9963E status (L9963E_OK on success, L9963E_ERROR on failure).
 */
L9963E_StatusTypeDef L9963E_start_conversion(L9963E_HandleTypeDef *handle,
                                             uint8_t device,
                                             uint8_t adc_filter_soc,
                                             uint8_t options);
/**
 * @brief Polls the ADC conversion status.
 * 
 * @param handle Pointer to the L9963E handle structure.
 * @param device Target device address.
 * @param conversion_done Pointer to store the conversion status.
 * @return L9963E status (L9963E_OK on success, L9963E_ERROR on failure).
 */
L9963E_StatusTypeDef L9963E_poll_conversion(L9963E_HandleTypeDef *handle, uint8_t device, uint8_t *conversion_done);
/**
 * @brief Reads the voltage of a specified battery cell.
 * 
 * @param handle Pointer to the L9963E handle structure.
 * @param device Target device address.
 * @param cell Cell number to read.
 * @param vcell Pointer to store the voltage value.
 * @param data_ready Pointer to store the data readiness flag.
 * @return L9963E status (L9963E_OK on success, L9963E_ERROR on failure).
 */
L9963E_StatusTypeDef L9963E_read_cell_voltage(L9963E_HandleTypeDef *handle,
                                              uint8_t device,
                                              L9963E_CellsTypeDef cell,
                                              uint16_t *vcell,
                                              uint8_t *data_ready);
/**
 * @brief Reads the total battery voltage.
 * 
 * @param handle Pointer to the L9963E handle structure.
 * @param device Target device address.
 * @param vbatt_monitor Pointer to store the battery voltage monitor value.
 * @param vbatt_sum Pointer to store the sum of battery voltages.
 * @return L9963E status (L9963E_OK on success, L9963E_ERROR on failure).
 */
L9963E_StatusTypeDef L9963E_read_batt_voltage(L9963E_HandleTypeDef *handle,
                                              uint8_t device,
                                              uint16_t *vbatt_monitor,
                                              uint32_t *vbatt_sum);
/**
 * @brief Reads the voltage of a specified GPIO pin.
 * 
 * @param handle Pointer to the L9963E handle structure.
 * @param device Target device address.
 * @param gpio GPIO pin number.
 * @param vgpio Pointer to store the GPIO voltage value.
 * @param data_ready Pointer to store the data readiness flag.
 * @return L9963E status (L9963E_OK on success, L9963E_ERROR on failure).
 */
L9963E_StatusTypeDef L9963E_read_gpio_voltage(L9963E_HandleTypeDef *handle,
                                              uint8_t device,
                                              L9963E_GpiosTypeDef gpio,
                                              uint16_t *vgpio,
                                              uint8_t *data_ready);

#endif  // L9963E_H