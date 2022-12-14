/*
 * "THE BEER-WARE LICENSE" (Revision 69):
 * Squadra Corse firmware team wrote this file. As long as you retain this notice
 * you can do whatever you want with this stuff. If we meet some day, and you 
 * think this stuff is worth it, you can buy us a beer in return.
 * 
 * Authors
 * - Federico Carbone [federico.carbone.sc@gmail.com]
 */

#include <inttypes.h>

typedef enum { _0x78BurstCmd = 0x78, _0x7ABurstCmd = 0x7A, _0x7BBurstCmd = 0x7B } L9963E_BurstCmdTypeDef;

typedef struct {
    struct {
        uint32_t VCell : 16, d_rdy_Vcell : 1, VCELL_EN : 1, : 14;
    } Frame1_14[14];
    struct {
        uint32_t vsum_batt19_2 : 18, : 14;
    } Frame15;
    struct {
        uint32_t VBATT_DIV : 16, vsum_batt1_0 : 2, : 14;
    } Frame16;
    struct {
        uint32_t eof_bal : 1, bal_on : 1, TimedBalTimer : 7, TimedBalacc : 1, VSUM_UV : 1, VSUM_OV : 1, DUTY_ON : 1,
            CONF_CYCLIC_EN : 1, OVR_LATCH : 1, SOC : 1, data_ready_vbattdiv : 1, data_ready_vsum : 1, : 14;
    } Frame17;
    struct {
        uint32_t CUR_INST_calib : 18, : 14;
    } Frame18;
} L9963E_0x78BurstTypeDef;

typedef struct {
} L9963E_0x7ABurstTypeDef;

typedef struct {
} L9963E_0x7BBurstTypeDef;