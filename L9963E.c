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
                                 uint8_t slave_n) {
#if L9963E_DEBUG
    if (handle == NULL) {
        return L9963E_ERROR;
    }

    if (slave_n >= 32) {
        return L9963E_ERROR;
    }
#endif

    handle->slave_n = slave_n;

    return L9963E_DRV_init(&(handle->drv_handle),
                           hspi,
                           cs_port,
                           cs_pin,
                           txen_port,
                           txen_pin,
                           bne_port,
                           bne_pin,
                           isofreq_port,
                           isofreq_pin);
}

L9963E_StatusTypeDef L9963E_addressing_procedure(L9963E_HandleTypeDef *handle,
                                                 uint8_t iso_freq_sel,
                                                 uint8_t is_dual_ring,
                                                 uint8_t out_res_tx_iso,
                                                 uint8_t lock_isofreq) {
    L9963E_RegisterUnionTypeDef write_reg;
    L9963E_RegisterUnionTypeDef read_reg;

    uint32_t tick = HAL_GetTick();
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
        if (L9963E_DRV_reg_read(&(handle->drv_handle), x, DEV_GEN_CFG, &read_reg, 1) == L9963E_OK &&
            read_reg.DEV_GEN_CFG.chip_ID == x) {
            ++x;
            tick = HAL_GetTick();
        } else {
            if (HAL_GetTick() - tick >= 10) {
                return L9963E_TIMEOUT;
            }

            //wakeup the device
            L9963E_DRV_wakeup(&(handle->drv_handle));
            // by default the wakeup procedure needs 2 ms of time (T_WAKEUP)
            HAL_Delay(2);

            //send broadcast command setting the chip_id
            write_reg.DEV_GEN_CFG.chip_ID      = x;
            write_reg.DEV_GEN_CFG.iso_freq_sel = 0b00;

            L9963E_DRV_reg_write(&(handle->drv_handle), L9963E_DEVICE_BROADCAST, DEV_GEN_CFG, &write_reg, 1);
        }
    }

    write_reg.generic                    = 0;
    write_reg.DEV_GEN_CFG.isotx_en_h     = 0b1;
    write_reg.DEV_GEN_CFG.out_res_tx_iso = out_res_tx_iso;
    write_reg.DEV_GEN_CFG.iso_freq_sel   = iso_freq_sel;

    if (iso_freq_sel == 0b11)
        L9963E_DRV_ISOFREQ_HIGH(&(handle->drv_handle));
    else
        L9963E_DRV_ISOFREQ_LOW(&(handle->drv_handle));

    L9963E_DRV_reg_write(&(handle->drv_handle), L9963E_DEVICE_BROADCAST, DEV_GEN_CFG, &write_reg, 1);

    write_reg.DEV_GEN_CFG.Farthest_Unit = 0b1;
    if (!handle->is_dual_ring) {
        write_reg.DEV_GEN_CFG.isotx_en_h = 0;
    }

    L9963E_DRV_reg_write(&(handle->drv_handle), handle->slave_n, DEV_GEN_CFG, &write_reg, 1);

    if (lock_isofreq == 1) {
        write_reg.generic                 = 0;
        write_reg.Bal_3.Lock_isoh_isofreq = 1;

        L9963E_DRV_reg_write(&(handle->drv_handle), L9963E_DEVICE_BROADCAST, Bal_3, &write_reg, 1);
    }

    return L9963E_OK;
}

L9963E_StatusTypeDef L9963E_setCommTimeout_Broadcast(L9963E_HandleTypeDef *handle,
                                                     L9963E_CommTimeoutTypeDef commTimeout) {
    L9963E_RegisterUnionTypeDef fastch_baluv_reg = {.fastch_baluv = {.CommTimeout = commTimeout}};

#if L9963E_DEBUG
    if (handle == NULL) {
        return L9963E_ERROR;
    }
#endif

    return L9963E_DRV_reg_write(&(handle->drv_handle), L9963E_DEVICE_BROADCAST, fastch_baluv, &fastch_baluv_reg, 10);
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

    if (preserve_reg_value) {
        errorcode = L9963E_DRV_reg_read(&(handle->drv_handle), device, fastch_baluv, &fastch_baluv_reg, 10);

        if (errorcode != L9963E_OK) {
            return errorcode;
        }
    }

    fastch_baluv_reg.fastch_baluv.CommTimeout = commTimeout;

    return L9963E_DRV_reg_write(&(handle->drv_handle), L9963E_DEVICE_BROADCAST, fastch_baluv, &fastch_baluv_reg, 10);
}