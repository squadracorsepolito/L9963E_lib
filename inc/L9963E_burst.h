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

#define L9963E_BURST_0x78_LEN 18
#define L9963E_BURST_0x7A_LEN 13
#define L9963E_BURST_0x7B_LEN 14

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
    struct {
        uint32_t wu_cyc_wup : 1, wu_faulth : 1, wu_isoline : 1, wu_spi : 1, wu_gpio7 : 1, VCOM_UV : 1, VCOM_OV : 1,
            VREG_UV : 1, VTREF_OV : 1, VTREF_UV : 1, VDIG_OV : 1, VANA_OV : 1, Otchip : 1, sense_minus_open : 1,
            sense_plus_open : 1, TCYCLE_OVF : 1, VREG_OV : 1, OVR_LATCH : 1, : 14;
    } Frame1;
    struct {
        uint32_t GPIO6_OPEN : 1, GPIO7_OPEN : 1, GPIO8_OPEN : 1, GPIO9_OPEN : 1, GPIO3_fastchg_OT : 1,
            GPIO4_fastchg_OT : 1, GPIO5_fastchg_OT : 1, GPIO6_fastchg_OT : 1, GPIO7_fastchg_OT : 1,
            GPIO8_fastchg_OT : 1, GPIO9_fastchg_OT : 1, EoBtimeerror : 1, CoCouOvF : 1, TrimmCalOk : 1, loss_gndref : 1,
            loss_cgnd : 1, loss_dgnd : 1, loss_agnd : 1, : 14;
    } Frame2;
    struct {
        uint32_t BAL1_OPEN : 1, BAL2_OPEN : 1, BAL3_OPEN : 1, BAL4_OPEN : 1, BAL5_OPEN : 1, BAL6_OPEN : 1,
            BAL7_OPEN : 1, BAL8_OPEN : 1, BAL9_OPEN : 1, BAL10_OPEN : 1, BAL11_OPEN : 1, BAL12_OPEN : 1, BAL13_OPEN : 1,
            BAL14_OPEN : 1, EEPROM_DWNLD_DONE : 1, GPIO3_OPEN : 1, GPIO4_OPEN : 1, GPIO5_OPEN : 1, : 14;
    } Frame3;
    struct {
        uint32_t BAL1_SHORT : 1, BAL2_SHORT : 1, BAL3_SHORT : 1, BAL4_SHORT : 1, BAL5_SHORT : 1, BAL6_SHORT : 1,
            BAL7_SHORT : 1, BAL8_SHORT : 1, BAL9_SHORT : 1, BAL10_SHORT : 1, BAL11_SHORT : 1, BAL12_SHORT : 1,
            BAL13_SHORT : 1, BAL14_SHORT : 1, VTREF_COMP_BIST_FAIL : 1, VCOM_COMP_BIST_FAIL : 1,
            VREG_COMP_BIST_FAIL : 1, VBAT_COMP_BIST_FAIL : 1, : 14;
    } Frame4;
    struct {
        uint32_t CELL0_OPEN : 1, CELL1_OPEN : 1, CELL2_OPEN : 1, CELL3_OPEN : 1, CELL4_OPEN : 1, CELL5_OPEN : 1,
            CELL6_OPEN : 1, CELL7_OPEN : 1, CELL8_OPEN : 1, CELL9_OPEN : 1, CELL10_OPEN : 1, CELL11_OPEN : 1,
            CELL12_OPEN : 1, CELL13_OPEN : 1, CELL14_OPEN : 1, VBAT_OPEN : 1, HWSC_DONE : 1,
            EEPROM_CRC_ERR_CAL_FF : 1, : 14;
    } Frame5;
    struct {
        uint32_t VCELL1_UV : 1, VCELL2_UV : 1, VCELL3_UV : 1, VCELL4_UV : 1, VCELL5_UV : 1, VCELL6_UV : 1,
            VCELL7_UV : 1, VCELL8_UV : 1, VCELL9_UV : 1, VCELL10_UV : 1, VCELL11_UV : 1, VCELL12_UV : 1, VCELL13_UV : 1,
            VCELL14_UV : 1, RAM_CRC_ERR : 1, EEPROM_CRC_ERR_CAL_RAM : 1, Comm_timeout_flt : 1,
            EEPROM_CRC_ERR_SECT_0 : 1, : 14;
    } Frame6;
    struct {
        uint32_t VCELL1_OV : 1, VCELL2_OV : 1, VCELL3_OV : 1, VCELL4_OV : 1, VCELL5_OV : 1, VCELL6_OV : 1,
            VCELL7_OV : 1, VCELL8_OV : 1, VCELL9_OV : 1, VCELL10_OV : 1, VCELL11_OV : 1, VCELL12_OV : 1, VCELL13_OV : 1,
            VCELL14_OV : 1, VSUM_UV : 1, VBATTCRIT_UV : 1, VBATT_WRN_UV : 1, VBATT_WRN_OV : 1, : 14;
    } Frame7;
    struct {
        uint32_t GPIO3_UT : 1, GPIO4_UT : 1, GPIO5_UT : 1, GPIO6_UT : 1, GPIO7_UT : 1, GPIO8_UT : 1, GPIO9_UT : 1,
            GPIO3_OT : 1, GPIO4_OT : 1, GPIO5_OT : 1, GPIO6_OT : 1, GPIO7_OT : 1, GPIO8_OT : 1, GPIO9_OT : 1,
            VSUM_OV : 1, VBATTCRIT_OV : 1, eof_bal : 1, bal_on : 1, : 14;
    } Frame8;
    struct {
        uint32_t VCELL1_BAL_UV : 1, VCELL2_BAL_UV : 1, VCELL3_BAL_UV : 1, VCELL4_BAL_UV : 1, VCELL5_BAL_UV : 1,
            VCELL6_BAL_UV : 1, VCELL7_BAL_UV : 1, VCELL8_BAL_UV : 1, VCELL9_BAL_UV : 1, VCELL10_BAL_UV : 1,
            VCELL11_BAL_UV : 1, VCELL12_BAL_UV : 1, VCELL13_BAL_UV : 1, VCELL14_BAL_UV : 1, GPO3on : 1, GPO4on : 1,
            GPO5on : 1, GPO6on : 1, : 14;
    } Frame9;
    struct {
        uint32_t GPIO_BIST_FAIL : 7, GPO3short : 1, GPO4short : 1, GPO5short : 1, GPO6short : 1, GPO7short : 1,
            GPO8short : 1, GPO9short : 1, GPO7on : 1, GPO8on : 1, GPO9on : 1, Fault_L_line_status : 1, : 14;
    } Frame10;
    struct {
        uint32_t MUX_BIST_FAIL : 14, HeartBeat_En : 1, FaultH_EN : 1, FaultHline_fault : 1, HeartBeat_fault : 1, : 14;
    } Frame11;
    struct {
        uint32_t BIST_BAL_COMP_LS_FAIL : 7, BIST_BAL_COMP_HS_FAIL : 7, HeartBeatCycle : 3,
            curr_sense_ovc_sleep : 1, : 14;
    } Frame12;
    struct {
        uint32_t OPEN_BIST_FAIL : 14, clk_mon_init_done : 1, clk_mon_en : 1, OSCFail : 1, curr_sense_ovc_norm : 1, : 14;
    } Frame13;
} L9963E_0x7ABurstTypeDef;

typedef struct {
    struct {
        uint32_t CoulombCntTime : 16, CoCouOvF : 1, CoulombCounter_en : 1, : 14;
    } Frame1;
    struct {
        uint32_t CoulombCounter_msb : 16, sense_minus_open : 1, sense_plus_open : 1, : 14;
    } Frame2;
    struct {
        uint32_t CoulombCounter_lsb : 16, curr_sense_ovc_norm : 1, curr_sense_ovc_sleep : 1, : 14;
    } Frame3;
    struct {
        uint32_t CUR_INST_synch : 18, : 14;
    } Frame4;
    struct {
        uint32_t CUR_INST_calib : 18, : 14;
    } Frame5;
    struct {
        uint32_t GPIO_MEAS : 16, d_rdy_gpio : 1, GPIO_OT : 1, : 14;
    } Frame6_12[7];
    struct {
        uint32_t VTREF_MEAS : 16, d_rdy_vtref : 1, TrimmCalOk : 1, : 14;
    } Frame13;
    struct {
        uint32_t TempChip : 8, GPIO3_UT : 1, GPIO4_UT : 1, GPIO5_UT : 1, GPIO6_UT : 1, GPIO7_UT : 1, GPIO8_UT : 1,
            GPIO9_UT : 1, bal_on : 1, eof_bal : 1, OTchip : 1, : 14;
    } Frame14;
} L9963E_0x7BBurstTypeDef;

typedef union {
    uint32_t generics[L9963E_BURST_0x78_LEN];
    L9963E_0x78BurstTypeDef _0x78;
    L9963E_0x7ABurstTypeDef _0x7A;
    L9963E_0x7BBurstTypeDef _0x7B;
} L9963E_BurstUnionTypeDef;