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

#include <memory.h>
#include <stddef.h>

L9963E_StatusTypeDef L9963E_init(L9963E_HandleTypeDef *handle, L9963E_IfTypeDef interface, uint8_t slave_n) {
#if L9963E_DEBUG
    if (handle == NULL) {
        return L9963E_ERROR;
    }

    if (slave_n >= 32) {
        return L9963E_ERROR;
    }
#endif

    handle->slave_n = slave_n;

    return L9963E_DRV_init(&(handle->drv_handle), interface);
}

L9963E_StatusTypeDef L9963E_addressing_procedure(L9963E_HandleTypeDef *handle,
                                                 uint8_t iso_freq_sel,
                                                 uint8_t is_dual_ring,
                                                 uint8_t out_res_tx_iso,
                                                 uint8_t lock_isofreq) {
    L9963E_RegisterUnionTypeDef write_reg;
    L9963E_RegisterUnionTypeDef read_reg;

    uint32_t tick = L9963E_DRV_GETTICK(&(handle->drv_handle));
    uint8_t x     = 1;

#if L9963E_DEBUG
    if (handle == NULL) {
        return L9963E_ERROR;
    }
#endif

    while (x <= handle->slave_n) {
        write_reg.generic = 0;
        read_reg.generic  = 0;

        //readback, if successful continue, else repeat the same cycle
        if (L9963E_DRV_reg_read(&(handle->drv_handle), x, L9963E_DEV_GEN_CFG_ADDR, &read_reg, 1) == L9963E_OK &&
            read_reg.DEV_GEN_CFG.chip_ID == x) {
            ++x;
            tick = L9963E_DRV_GETTICK(&(handle->drv_handle));
        } else {
            if (L9963E_DRV_GETTICK(&(handle->drv_handle)) - tick >= 10) {
                return L9963E_TIMEOUT;
            }

            //wakeup the device
            L9963E_DRV_wakeup(&(handle->drv_handle));
            // by default the wakeup procedure needs 2 ms of time (T_WAKEUP)
            L9963E_DRV_DELAY(&(handle->drv_handle), 2);

            //send broadcast command setting the chip_idz
            write_reg.generic                  = L9963E_DEV_GEN_CFG_DEFAULT;
            write_reg.DEV_GEN_CFG.chip_ID      = x;
            write_reg.DEV_GEN_CFG.iso_freq_sel = 0b00;

            L9963E_DRV_reg_write(
                &(handle->drv_handle), L9963E_DEVICE_BROADCAST, L9963E_DEV_GEN_CFG_ADDR, &write_reg, 1);
        }
    }

    write_reg.generic                    = L9963E_DEV_GEN_CFG_DEFAULT;
    write_reg.DEV_GEN_CFG.isotx_en_h     = 0b1;
    write_reg.DEV_GEN_CFG.out_res_tx_iso = out_res_tx_iso;
    write_reg.DEV_GEN_CFG.iso_freq_sel   = iso_freq_sel;

    if (iso_freq_sel == 0b11)
        L9963E_DRV_ISOFREQ_HIGH(&(handle->drv_handle));
    else
        L9963E_DRV_ISOFREQ_LOW(&(handle->drv_handle));

    L9963E_DRV_reg_write(&(handle->drv_handle), L9963E_DEVICE_BROADCAST, L9963E_DEV_GEN_CFG_ADDR, &write_reg, 1);

    write_reg.DEV_GEN_CFG.Farthest_Unit = 0b1;
    if (!handle->is_dual_ring) {
        write_reg.DEV_GEN_CFG.isotx_en_h = 0;
    }

    L9963E_DRV_reg_write(&(handle->drv_handle), handle->slave_n, L9963E_DEV_GEN_CFG_ADDR, &write_reg, 1);

    if (lock_isofreq == 1) {
        write_reg.generic                 = L9963E_BAL_3_DEFAULT;
        write_reg.Bal_3.Lock_isoh_isofreq = 1;

        L9963E_DRV_reg_write(&(handle->drv_handle), L9963E_DEVICE_BROADCAST, L9963E_Bal_3_ADDR, &write_reg, 1);
    }

    return L9963E_OK;
}

L9963E_StatusTypeDef L9963E_setCommTimeout(L9963E_HandleTypeDef *handle,
                                           L9963E_CommTimeoutTypeDef commTimeout,
                                           uint8_t device,
                                           uint8_t preserve_reg_value) {
    L9963E_StatusTypeDef errorcode = L9963E_OK;
    L9963E_RegisterUnionTypeDef fastch_baluv_reg;

#if L9963E_DEBUG
    if (handle == NULL) {
        return L9963E_ERROR;
    }
#endif

    if (preserve_reg_value && device != L9963E_DEVICE_BROADCAST) {
        errorcode = L9963E_DRV_reg_read(&(handle->drv_handle), device, L9963E_fastch_baluv_ADDR, &fastch_baluv_reg, 10);

        if (errorcode != L9963E_OK) {
            return errorcode;
        }
    } else {
        fastch_baluv_reg.generic = L9963E_FASTCH_BALUV_DEFAULT;
    }

    fastch_baluv_reg.fastch_baluv.CommTimeout = commTimeout;

    return L9963E_DRV_reg_write(
        &(handle->drv_handle), device, L9963E_fastch_baluv_ADDR, &fastch_baluv_reg, 10);
}

L9963E_StatusTypeDef L9963E_set_enabled_cells(L9963E_HandleTypeDef *handle, uint8_t device, uint16_t cells) {
    L9963E_RegisterUnionTypeDef vcells_en_reg = {0};

#if L9963E_DEBUG
    if (handle == NULL) {
        return L9963E_ERROR;
    }
#endif

    for(uint8_t i=0; i<14; ++i) {
        if(cells & (1<<i)) {
            switch (i+1)
            {
            case 1:
                vcells_en_reg.VCELLS_EN.VCELL1_EN = 1;
                break;
            case 2:
                vcells_en_reg.VCELLS_EN.VCELL2_EN = 1;
                break;
            case 3:
                vcells_en_reg.VCELLS_EN.VCELL3_EN = 1;
                break;
            case 4:
                vcells_en_reg.VCELLS_EN.VCELL4_EN = 1;
                break;
            case 5:
                vcells_en_reg.VCELLS_EN.VCELL5_EN = 1;
                break;
            case 6:
                vcells_en_reg.VCELLS_EN.VCELL6_EN = 1;
                break;
            case 7:
                vcells_en_reg.VCELLS_EN.VCELL7_EN = 1;
                break;
            case 8:
                vcells_en_reg.VCELLS_EN.VCELL8_EN = 1;
                break;
            case 9:
                vcells_en_reg.VCELLS_EN.VCELL9_EN = 1;
                break;
            case 10:
                vcells_en_reg.VCELLS_EN.VCELL10_EN = 1;
                break;
            case 11:
                vcells_en_reg.VCELLS_EN.VCELL11_EN = 1;
                break;
            case 12:
                vcells_en_reg.VCELLS_EN.VCELL12_EN = 1;
                break;
            case 13:
                vcells_en_reg.VCELLS_EN.VCELL13_EN = 1;
                break;
            case 14:
                vcells_en_reg.VCELLS_EN.VCELL14_EN = 1;
                break;
            default:
                break;
            }
        }
    }

    return L9963E_DRV_reg_write(
        &(handle->drv_handle), device, L9963E_VCELLS_EN_ADDR, &vcells_en_reg, 10);
}

L9963E_StatusTypeDef L9963E_start_conversion(L9963E_HandleTypeDef *handle, uint8_t device, uint8_t adc_filter_soc, uint8_t options) {
    L9963E_RegisterUnionTypeDef adcv_conv_reg = {0};

#if L9963E_DEBUG
    if (handle == NULL) {
        return L9963E_ERROR;
    }
#endif

    adcv_conv_reg.ADCV_CONV.ADC_FILTER_SOC = adc_filter_soc;

    adcv_conv_reg.ADCV_CONV.SOC = 1;
    adcv_conv_reg.ADCV_CONV.GPIO_CONV = (options & L9963E_GPIO_CONV) != 0;
    adcv_conv_reg.ADCV_CONV.GPIO_TERM_CONV = (options & L9963E_GPIO_TERM_CONV) != 0;
    adcv_conv_reg.ADCV_CONV.BAL_TERM_CONV = (options & L9963E_BAL_TERM_CONV) != 0;
    adcv_conv_reg.ADCV_CONV.CELL_TERM_CONV = (options & L9963E_CELL_TERM_CONV) != 0;

    return L9963E_DRV_reg_write(&(handle->drv_handle), device, L9963E_ADCV_CONV_ADDR, &adcv_conv_reg, 10);
}

L9963E_StatusTypeDef L9963E_poll_conversion(L9963E_HandleTypeDef *handle, uint8_t device, uint8_t *conversion_done) {
    L9963E_StatusTypeDef errorcode = L9963E_OK;
    L9963E_RegisterUnionTypeDef adcv_conv_reg = {0};

#if L9963E_DEBUG
    if (handle == NULL) {
        return L9963E_ERROR;
    }
#endif

    if(device == L9963E_DEVICE_BROADCAST) {
        *conversion_done = 0;
        return L9963E_ERROR;
    }

    errorcode = L9963E_DRV_reg_read(&(handle->drv_handle), device, L9963E_ADCV_CONV_ADDR, &adcv_conv_reg, 10);

    if(errorcode != L9963E_OK)
        return errorcode;

    *conversion_done = !adcv_conv_reg.ADCV_CONV.SOC;
    return L9963E_OK;
}

L9963E_StatusTypeDef L9963E_read_cell_voltage(L9963E_HandleTypeDef *handle, uint8_t device, L9963E_CellsTypeDef cell, uint16_t *vcell, uint8_t *data_ready) {
    L9963E_StatusTypeDef errorcode = L9963E_OK;
    L9963E_RegisterUnionTypeDef vcell_meas_reg = {0};
    L9963E_RegistersAddrTypeDef addr;

#if L9963E_DEBUG
    if (handle == NULL) {
        return L9963E_ERROR;
    }
#endif

    if(device == L9963E_DEVICE_BROADCAST) {
        *vcell = 0;
        *data_ready = 0;
        return L9963E_ERROR;
    }

    switch (cell)
    {
    case L9963E_CELL1:
        addr = L9963E_Vcell1_ADDR;
        break;
    case L9963E_CELL2:
        addr = L9963E_Vcell2_ADDR;
        break;
    case L9963E_CELL3:
        addr = L9963E_Vcell3_ADDR;
        break;
    case L9963E_CELL4:
        addr = L9963E_Vcell4_ADDR;
        break;
    case L9963E_CELL5:
        addr = L9963E_Vcell5_ADDR;
        break;
    case L9963E_CELL6:
        addr = L9963E_Vcell6_ADDR;
        break;
    case L9963E_CELL7:
        addr = L9963E_Vcell7_ADDR;
        break;
    case L9963E_CELL8:
        addr = L9963E_Vcell8_ADDR;
        break;
    case L9963E_CELL9:
        addr = L9963E_Vcell9_ADDR;
        break;
    case L9963E_CELL10:
        addr = L9963E_Vcell10_ADDR;
        break;
    case L9963E_CELL11:
        addr = L9963E_Vcell11_ADDR;
        break;
    case L9963E_CELL12:
        addr = L9963E_Vcell12_ADDR;
        break;
    case L9963E_CELL13:
        addr = L9963E_Vcell13_ADDR;
        break;
    case L9963E_CELL14:
        addr = L9963E_Vcell14_ADDR;
        break;
    default:
        addr = L9963E_Vcell1_ADDR;
        break;
    }

    errorcode = L9963E_DRV_reg_read(&(handle->drv_handle), device, addr, &vcell_meas_reg, 10);

    if(errorcode != L9963E_OK)
        return errorcode;
    
    *vcell = vcell_meas_reg.Vcell1.VCell1;
    *data_ready = vcell_meas_reg.Vcell1.d_rdy_Vcell1;

    return L9963E_OK;
}


L9963E_StatusTypeDef L9963E_read_batt_voltage(L9963E_HandleTypeDef *handle, uint8_t device, uint16_t *vbatt_monitor, uint32_t *vbatt_sum) {
    L9963E_StatusTypeDef errorcode = L9963E_OK;
    L9963E_RegisterUnionTypeDef vbattdiv_reg = {0};
    L9963E_RegisterUnionTypeDef vsumbatt_reg = {0};

#if L9963E_DEBUG
    if (handle == NULL) {
        return L9963E_ERROR;
    }
#endif

    if(device == L9963E_DEVICE_BROADCAST) {
        *vbatt_monitor = 0;
        *vbatt_sum = 0;
        return L9963E_ERROR;
    }

    errorcode = L9963E_DRV_reg_read(&(handle->drv_handle), device, L9963E_VBATTDIV_ADDR, &vbattdiv_reg, 10);

    if(errorcode != L9963E_OK)
        return errorcode;
    
    errorcode = L9963E_DRV_reg_read(&(handle->drv_handle), device, L9963E_VSUMBATT_ADDR, &vsumbatt_reg, 10);
    
    *vbatt_monitor = vbattdiv_reg.VBATTDIV.VBATT_DIV;

    if(errorcode != L9963E_OK)
        return errorcode;

    *vbatt_sum = (vsumbatt_reg.VSUMBATT.vsum_batt19_2<<2) | vbattdiv_reg.VBATTDIV.vsum_batt1_0;

    return L9963E_OK;
}

L9963E_StatusTypeDef L9963E_read_gpio_voltage(L9963E_HandleTypeDef *handle, uint8_t device, L9963E_GpiosTypeDef gpio, uint16_t *vgpio, uint8_t *data_ready) {
    L9963E_StatusTypeDef errorcode = L9963E_OK;
    L9963E_RegisterUnionTypeDef vgpio_meas_reg = {0};
    L9963E_RegistersAddrTypeDef addr;

#if L9963E_DEBUG
    if (handle == NULL) {
        return L9963E_ERROR;
    }
#endif

    switch (gpio)
    {
    case L9963E_GPIO3:
        addr = L9963E_GPIO3_MEAS_ADDR;
        break;
    case L9963E_GPIO4:
        addr = L9963E_GPIO4_MEAS_ADDR;
        break;
    case L9963E_GPIO5:
        addr = L9963E_GPIO5_MEAS_ADDR;
        break;
    case L9963E_GPIO6:
        addr = L9963E_GPIO6_MEAS_ADDR;
        break;
    case L9963E_GPIO7:
        addr = L9963E_GPIO7_MEAS_ADDR;
        break;
    case L9963E_GPIO8:
        addr = L9963E_GPIO8_MEAS_ADDR;
        break;
    case L9963E_GPIO9:
        addr = L9963E_GPIO9_MEAS_ADDR;
        break;
    default:
        addr = L9963E_GPIO3_MEAS_ADDR;
        break;
    }

    if(device == L9963E_DEVICE_BROADCAST) {
        *vgpio = 0;
        *data_ready = 0;
        return L9963E_ERROR;
    }

    errorcode = L9963E_DRV_reg_read(&(handle->drv_handle), device, addr, &vgpio_meas_reg, 10);

    if(errorcode != L9963E_OK)
        return errorcode;
    
    *vgpio = vgpio_meas_reg.GPIO3_MEAS.GPIO3_MEAS;
    *data_ready = vgpio_meas_reg.GPIO3_MEAS.d_rdy_gpio3;

    return L9963E_OK;
}

L9963E_StatusTypeDef L9963E_enable_vref(L9963E_HandleTypeDef *handle, uint8_t device, uint8_t preserve_reg_value) {
    L9963E_StatusTypeDef errorcode = L9963E_OK;
    L9963E_RegisterUnionTypeDef ncycle_prog2_reg;

#if L9963E_DEBUG
    if (handle == NULL) {
        return L9963E_ERROR;
    }
#endif

    if (preserve_reg_value && device != L9963E_DEVICE_BROADCAST) {
        errorcode = L9963E_DRV_reg_read(&(handle->drv_handle), device, L9963E_fastch_baluv_ADDR, &ncycle_prog2_reg, 10);

        if (errorcode != L9963E_OK) {
            return errorcode;
        }
    } else {
        ncycle_prog2_reg.generic = L9963E_FASTCH_BALUV_DEFAULT;
    }

    ncycle_prog2_reg.generic = L9963E_NCYCLE_PROG_2_DEFAULT;
    ncycle_prog2_reg.NCYCLE_PROG_2.VTREF_EN = 1;
    return L9963E_DRV_reg_write(&(handle->drv_handle), device, L9963E_NCYCLE_PROG_2_ADDR, &ncycle_prog2_reg, 10);
}