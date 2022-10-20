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

typedef enum {
    DEV_GEN_CFG            = 0x1,
    fastch_baluv           = 0x2,
    Bal_1                  = 0x3,
    Bal_2                  = 0x4,
    Bal_3                  = 0x5,
    Bal_4                  = 0x6,
    Bal_5                  = 0x7,
    Bal_6                  = 0x8,
    Bal_7                  = 0x9,
    Bal_8                  = 0xA,
    VCELL_THRESH_UV_OV     = 0xB,
    VBATT_SUM_TH           = 0xC,
    ADCV_CONV              = 0xD,
    NCYCLE_PROG_1          = 0xE,
    NCYCLE_PROG_2          = 0xF,
    BalCell14_7act         = 0x10,
    BalCell6_1act          = 0x11,
    FSM                    = 0x12,
    GPOxOn_and_GPI93       = 0x13,
    GPIO9_3_CONF           = 0x14,
    GPIO3_THR              = 0x15,
    GPIO4_THR              = 0x16,
    GPIO5_THR              = 0x17,
    GPIO6_THR              = 0x18,
    GPIO7_THR              = 0x19,
    GPIO8_THR              = 0x1A,
    GPIO9_THR              = 0x1B,
    VCELLS_EN              = 0x1C,
    Faultmask              = 0x1D,
    Faultmask2             = 0x1E,
    CSA_THRESH_NORM        = 0x1F,
    CSA_GPIO_MSK           = 0x20,
    Vcell1                 = 0x21,
    Vcell2                 = 0x22,
    Vcell3                 = 0x23,
    Vcell4                 = 0x24,
    Vcell5                 = 0x25,
    Vcell6                 = 0x26,
    Vcell7                 = 0x27,
    Vcell8                 = 0x28,
    Vcell9                 = 0x29,
    Vcell10                = 0x2A,
    Vcell11                = 0x2B,
    Vcell12                = 0x2C,
    Vcell13                = 0x2D,
    Vcell14                = 0x2E,
    Ibattery_synch         = 0x2F,
    Ibattery_calib         = 0x30,
    CoulCntrTime           = 0x31,
    CoulCntr_msb           = 0x32,
    CoulCntr_lsb           = 0x33,
    GPIO3_MEAS             = 0x34,
    GPIO4_MEAS             = 0x35,
    GPIO5_MEAS             = 0x36,
    GPIO6_MEAS             = 0x37,
    GPIO7_MEAS             = 0x38,
    GPIO8_MEAS             = 0x39,
    GPIO9_MEAS             = 0x3A,
    TempChip               = 0x3B,
    Faults1                = 0x3C,
    Faults2                = 0x3D,
    BAL_OPEN               = 0x3E,
    BAL_SHORT              = 0x3F,
    VSUMBATT               = 0x40,
    VBATTDIV               = 0x41,
    CELL_OPEN              = 0x42,
    VCELL_UV               = 0x43,
    VCELL_OV               = 0x44,
    VGPIO_OT_UT            = 0x45,
    VCELL_BAL_UV           = 0x46,
    GPIO_fastchg_OT        = 0x47,
    MUX_BIST_FAIL          = 0x48,
    BIST_COMP              = 0x49,
    OPEN_BIST_FAIL         = 0x4A,
    GPIO_BIST_FAIL         = 0x4B,
    VTREF                  = 0x4C,
    NVM_WR_1               = 0x4D,
    NVM_WR_2               = 0x4E,
    NVM_WR_3               = 0x4F,
    NVM_WR_4               = 0x50,
    NVM_WR_5               = 0x51,
    NVM_WR_6               = 0x52,
    NVM_WR_7               = 0x53,
    NVM_RD_1               = 0x54,
    NVM_RD_2               = 0x55,
    NVM_RD_3               = 0x56,
    NVM_RD_4               = 0x57,
    NVM_RD_5               = 0x58,
    NVM_RD_6               = 0x59,
    NVM_RD_7               = 0x5A,
    NVM_CMD_CNTR           = 0x5B,
    NVM_UNLCK_PRG          = 0x5C
} L9963E_RegistersAddrTypeDef;


typedef struct {
    uint32_t    :14,
                chip_ID            :5,
                isotx_en_h         :1,
                out_res_tx_iso     :2,
                iso_freq_sel       :2,
                Noreg7             :1,
                HeartBeatCycle     :3,
                FaultH_EN          :1,
                HeartBeat_En       :1,
                Farthest_Unit      :1,
                FaultL_force       :1;
} L9963E_DEV_GEN_CFGTypeDef;

typedef struct {
    uint32_t    :14,
                CommTimeout                   :2,
                Gpio_fastchg_OT_delta_thr     :8,
                Vcell_bal_UV_delta_thr        :8;
} L9963E_fastch_baluvTypeDef;

typedef struct {
    uint32_t    :14,
                comm_timeout_dis     :1,
                slp_bal_conf         :1,
                bal_start            :1,
                bal_stop             :1,
                TimedBalTimer        :7,
                WDTimedBalTimer      :7;
} L9963E_Bal_1TypeDef;

typedef struct {
    uint32_t    :14,
                Balmode               :2,
                TimedBalacc           :1,
                ThrTimedBalCell14     :7,
                Noreg7                :1,
                ThrTimedBalCell13     :7;
} L9963E_Bal_2TypeDef;

typedef struct {
    uint32_t    :14,
                first_wup_done         :1,
                trimming_retrigger     :1,
                Lock_isoh_isofreq      :1,
                ThrTimedBalCell12      :7,
                Noreg7                 :1,
                ThrTimedBalCell11      :7;
} L9963E_Bal_3TypeDef;

typedef struct {
    uint32_t    :14,
                clk_mon_en            :1,
                Noreg16               :1,
                clk_mon_init_done     :1,
                ThrTimedBalCell10     :7,
                Noreg7                :1,
                ThrTimedBalCell9      :7;
} L9963E_Bal_4TypeDef;

typedef struct {
    uint32_t    :14,
                transceiver_on_by_up        :1,
                transceiver_valid_by_up     :1,
                Noreg15                     :1,
                ThrTimedBalCell8            :7,
                Noreg7                      :1,
                ThrTimedBalCell7            :7;
} L9963E_Bal_5TypeDef;

typedef struct {
    uint32_t    :14,
                Noreg17              :1,
                Noreg16              :1,
                Noreg15              :1,
                ThrTimedBalCell6     :7,
                Noreg7               :1,
                ThrTimedBalCell5     :7;
} L9963E_Bal_6TypeDef;

typedef struct {
    uint32_t    :14,
                Noreg17              :1,
                Noreg16              :1,
                Noreg15              :1,
                ThrTimedBalCell4     :7,
                Noreg7               :1,
                ThrTimedBalCell3     :7;
} L9963E_Bal_7TypeDef;

typedef struct {
    uint32_t    :14,
                Noreg17              :1,
                Noreg16              :1,
                Noreg15              :1,
                ThrTimedBalCell2     :7,
                Noreg7               :1,
                ThrTimedBalCell1     :7;
} L9963E_Bal_8TypeDef;

typedef struct {
    uint32_t    :14,
                Noreg17           :1,
                Noreg16           :1,
                threshVcellOV     :8,
                threshVcellUV     :8;
} L9963E_VCELL_THRESH_UV_OVTypeDef;

typedef struct {
    uint32_t    :14,
                Noreg17             :1,
                Noreg16             :1,
                VBATT_SUM_OV_TH     :8,
                VBATT_SUM_UV_TH     :8;
} L9963E_VBATT_SUM_THTypeDef;

typedef struct {
    uint32_t    :14,
                ADC_CROSS_CHECK      :1,
                TCYCLE_OVF           :1,
                SOC                  :1,
                OVR_LATCH            :1,
                CONF_CYCLIC_EN       :1,
                DUTY_ON              :1,
                ADC_FILTER_SOC       :3,
                GPIO_CONV            :1,
                GPIO_TERM_CONV       :1,
                CELL_TERM_CONV       :1,
                BAL_TERM_CONV        :1,
                HWSC                 :1,
                TCYCLE               :3,
                CYCLIC_CONTINOUS     :1;
} L9963E_ADCV_CONVTypeDef;

typedef struct {
    uint32_t    :14,
                T_CELL_SET                :2,
                NCYCLE_GPIO_TERM          :3,
                NCYCLE_CELL_TERM          :3,
                NCYCLE_BAL_TERM           :3,
                BAL_TIM_AUTO_PAUSE        :1,
                BAL_AUTO_PAUSE            :1,
                CYCLIC_UPDATE             :1,
                CROSS_ODD_EVEN_CELL       :1,
                PCB_open_en_odd_curr      :1,
                PCB_open_en_even_curr     :1,
                Noreg0                    :1;
} L9963E_NCYCLE_PROG_1TypeDef;

typedef struct {
    uint32_t    :14,
                VTREF_EN             :1,
                VTREF_DYN_EN         :1,
                NCYCLE_GPIO          :3,
                NCYCLE_HWSC          :3,
                Noreg9               :1,
                ADC_FILTER_CYCLE     :3,
                TCYCLE_SLEEP         :3,
                ADC_FILTER_SLEEP     :3;
} L9963E_NCYCLE_PROG_2TypeDef;

typedef struct {
    uint32_t    :14,
                Noreg17     :1,
                Noreg16     :1,
                BAL14       :2,
                BAL13       :2,
                BAL12       :2,
                BAL11       :2,
                BAL10       :2,
                BAL9        :2,
                BAL8        :2,
                BAL7        :2;
} L9963E_BalCell14_7actTypeDef;

typedef struct {
    uint32_t    :14,
                Noreg17     :1,
                Noreg16     :1,
                BAL6        :2,
                BAL5        :2,
                BAL4        :2,
                BAL3        :2,
                BAL2        :2,
                BAL1        :2,
                Noreg3      :1,
                Noreg2      :1,
                bal_on      :1,
                eof_bal     :1;
} L9963E_BalCell6_1actTypeDef;

typedef struct {
    uint32_t    :14,
                Noreg17        :1,
                Noreg16        :1,
                SW_RST         :2,
                GO2SLP         :2,
                FSMstatus      :4,
                Noreg7         :1,
                Noreg6         :1,
                Noreg5         :1,
                wu_gpio7       :1,
                wu_spi         :1,
                wu_isoline     :1,
                wu_faulth      :1,
                wu_cyc_wup     :1;
} L9963E_FSMTypeDef;

typedef struct {
    uint32_t    :14,
                GPO9on      :1,
                GPO8on      :1,
                GPO7on      :1,
                GPO6on      :1,
                GPO5on      :1,
                GPO4on      :1,
                GPO3on      :1,
                Noreg10     :1,
                Noreg9      :1,
                GPI9        :1,
                GPI8        :1,
                GPI7        :1,
                GPI6        :1,
                GPI5        :1,
                GPI4        :1,
                GPI3        :1,
                Noreg1      :1,
                Noreg0      :1;
} L9963E_GPOxOn_and_GPI93TypeDef;

typedef struct {
    uint32_t    :14,
                GPIO9_CONFIG     :2,
                GPIO8_CONFIG     :2,
                GPIO7_CONFIG     :2,
                GPIO6_CONFIG     :2,
                GPIO5_CONFIG     :2,
                GPIO4_CONFIG     :2,
                GPIO3_CONFIG     :2,
                GPIO7_WUP_EN     :1,
                Noreg2           :1,
                Noreg1           :1,
                Noreg0           :1;
} L9963E_GPIO9_3_CONFTypeDef;

typedef struct {
    uint32_t    :14,
                GPIO3_OT_TH     :9,
                GPIO3_UT_TH     :9;
} L9963E_GPIO3_THRTypeDef;

typedef struct {
    uint32_t    :14,
                GPIO4_OT_TH     :9,
                GPIO4_UT_TH     :9;
} L9963E_GPIO4_THRTypeDef;

typedef struct {
    uint32_t    :14,
                GPIO5_OT_TH     :9,
                GPIO5_UT_TH     :9;
} L9963E_GPIO5_THRTypeDef;

typedef struct {
    uint32_t    :14,
                GPIO6_OT_TH     :9,
                GPIO6_UT_TH     :9;
} L9963E_GPIO6_THRTypeDef;

typedef struct {
    uint32_t    :14,
                GPIO7_OT_TH     :9,
                GPIO7_UT_TH     :9;
} L9963E_GPIO7_THRTypeDef;

typedef struct {
    uint32_t    :14,
                GPIO8_OT_TH     :9,
                GPIO8_UT_TH     :9;
} L9963E_GPIO8_THRTypeDef;

typedef struct {
    uint32_t    :14,
                GPIO9_OT_TH     :9,
                GPIO9_UT_TH     :9;
} L9963E_GPIO9_THRTypeDef;

typedef struct {
    uint32_t    :14,
                Noreg17        :1,
                Noreg16        :1,
                Noreg15        :1,
                Noreg14        :1,
                VCELL14_EN     :1,
                VCELL13_EN     :1,
                VCELL12_EN     :1,
                VCELL11_EN     :1,
                VCELL10_EN     :1,
                VCELL9_EN      :1,
                VCELL8_EN      :1,
                VCELL7_EN      :1,
                VCELL6_EN      :1,
                VCELL5_EN      :1,
                VCELL4_EN      :1,
                VCELL3_EN      :1,
                VCELL2_EN      :1,
                VCELL1_EN      :1;
} L9963E_VCELLS_ENTypeDef;

typedef struct {
    uint32_t    :14,
                Noreg17                :1,
                Noreg16                :1,
                Noreg15                :1,
                Noreg14                :1,
                VCELL14_BAL_UV_MSK     :1,
                VCELL13_BAL_UV_MSK     :1,
                VCELL12_BAL_UV_MSK     :1,
                VCELL11_BAL_UV_MSK     :1,
                VCELL10_BAL_UV_MSK     :1,
                VCELL9_BAL_UV_MSK      :1,
                VCELL8_BAL_UV_MSK      :1,
                VCELL7_BAL_UV_MSK      :1,
                VCELL6_BAL_UV_MSK      :1,
                VCELL5_BAL_UV_MSK      :1,
                VCELL4_BAL_UV_MSK      :1,
                VCELL3_BAL_UV_MSK      :1,
                VCELL2_BAL_UV_MSK      :1,
                VCELL1_BAL_UV_MSK      :1;
} L9963E_FaultmaskTypeDef;

typedef struct {
    uint32_t    :14,
                EEPROM_DWNLD_DONE             :1,
                EEPROM_CRC_ERR_SECT_0         :1,
                EEPROM_CRC_ERRMSK_SECT_0      :1,
                EEPROM_CRC_ERR_CAL_RAM        :1,
                EEPROM_CRC_ERRMSK_CAL_RAM     :1,
                EEPROM_CRC_ERR_CAL_FF         :1,
                EEPROM_CRC_ERRMSK_CAL_FF      :1,
                RAM_CRC_ERR                   :1,
                RAM_CRC_ERRMSK                :1,
                trim_dwnl_tried               :1,
                TrimmCalOk                    :1,
                Gpio9_fastchg_OT_MSK          :1,
                Gpio8_fastchg_OT_MSK          :1,
                Gpio7_fastchg_OT_MSK          :1,
                Gpio6_fastchg_OT_MSK          :1,
                Gpio5_fastchg_OT_MSK          :1,
                Gpio4_fastchg_OT_MSK          :1,
                Gpio3_fastchg_OT_MSK          :1;
} L9963E_Faultmask2TypeDef;

typedef struct {
    uint32_t    :14,
                adc_ovc_curr_threshold_norm     :18;
} L9963E_CSA_THRESH_NORMTypeDef;

typedef struct {
    uint32_t    :14,
                adc_ovc_curr_threshold_sleep     :5,
                CoulombCounter_en                :1,
                ovc_sleep_msk                    :1,
                ovc_norm_msk                     :1,
                sense_plus_open                  :1,
                sense_minus_open                 :1,
                Noreg7                           :1,
                Gpio9_OT_UT_MSK                  :1,
                Gpio8_OT_UT_MSK                  :1,
                Gpio7_OT_UT_MSK                  :1,
                Gpio6_OT_UT_MSK                  :1,
                Gpio5_OT_UT_MSK                  :1,
                Gpio4_OT_UT_MSK                  :1,
                Gpio3_OT_UT_MSK                  :1;
} L9963E_CSA_GPIO_MSKTypeDef;

typedef struct {
    uint32_t    :14,
                Noreg17          :1,
                d_rdy_Vcell1     :1,
                VCell1           :16;
} L9963E_Vcell1TypeDef;

typedef struct {
    uint32_t    :14,
                Noreg17          :1,
                d_rdy_Vcell2     :1,
                VCell2           :16;
} L9963E_Vcell2TypeDef;

typedef struct {
    uint32_t    :14,
                Noreg17          :1,
                d_rdy_Vcell3     :1,
                VCell3           :16;
} L9963E_Vcell3TypeDef;

typedef struct {
    uint32_t    :14,
                Noreg17          :1,
                d_rdy_Vcell4     :1,
                VCell4           :16;
} L9963E_Vcell4TypeDef;

typedef struct {
    uint32_t    :14,
                Noreg17          :1,
                d_rdy_Vcell5     :1,
                VCell5           :16;
} L9963E_Vcell5TypeDef;

typedef struct {
    uint32_t    :14,
                Noreg17          :1,
                d_rdy_Vcell6     :1,
                VCell6           :16;
} L9963E_Vcell6TypeDef;

typedef struct {
    uint32_t    :14,
                Noreg17          :1,
                d_rdy_Vcell7     :1,
                VCell7           :16;
} L9963E_Vcell7TypeDef;

typedef struct {
    uint32_t    :14,
                Noreg17          :1,
                d_rdy_Vcell8     :1,
                VCell8           :16;
} L9963E_Vcell8TypeDef;

typedef struct {
    uint32_t    :14,
                Noreg17          :1,
                d_rdy_Vcell9     :1,
                VCell9           :16;
} L9963E_Vcell9TypeDef;

typedef struct {
    uint32_t    :14,
                Noreg17           :1,
                d_rdy_Vcell10     :1,
                VCell10           :16;
} L9963E_Vcell10TypeDef;

typedef struct {
    uint32_t    :14,
                Noreg17           :1,
                d_rdy_Vcell11     :1,
                VCell11           :16;
} L9963E_Vcell11TypeDef;

typedef struct {
    uint32_t    :14,
                Noreg17           :1,
                d_rdy_Vcell12     :1,
                VCell12           :16;
} L9963E_Vcell12TypeDef;

typedef struct {
    uint32_t    :14,
                Noreg17           :1,
                d_rdy_Vcell13     :1,
                VCell13           :16;
} L9963E_Vcell13TypeDef;

typedef struct {
    uint32_t    :14,
                Noreg17           :1,
                d_rdy_Vcell14     :1,
                VCell14           :16;
} L9963E_Vcell14TypeDef;

typedef struct {
    uint32_t    :14,
                CUR_INST_Synch     :18;
} L9963E_Ibattery_synchTypeDef;

typedef struct {
    uint32_t    :14,
                CUR_INST_calib     :18;
} L9963E_Ibattery_calibTypeDef;

typedef struct {
    uint32_t    :14,
                Noreg17            :1,
                Noreg16            :1,
                CoulombCntTime     :16;
} L9963E_CoulCntrTimeTypeDef;

typedef struct {
    uint32_t    :14,
                Noreg17                :1,
                Noreg16                :1,
                CoulombCounter_msb     :16;
} L9963E_CoulCntr_msbTypeDef;

typedef struct {
    uint32_t    :14,
                Noreg17                :1,
                Noreg16                :1,
                CoulombCounter_lsb     :16;
} L9963E_CoulCntr_lsbTypeDef;

typedef struct {
    uint32_t    :14,
                ratio_abs_3_sel     :1,
                d_rdy_gpio3         :1,
                GPIO3_MEAS          :16;
} L9963E_GPIO3_MEASTypeDef;

typedef struct {
    uint32_t    :14,
                ratio_abs_4_sel     :1,
                d_rdy_gpio4         :1,
                GPIO4_MEAS          :16;
} L9963E_GPIO4_MEASTypeDef;

typedef struct {
    uint32_t    :14,
                ratio_abs_5_sel     :1,
                d_rdy_gpio5         :1,
                GPIO5_MEAS          :16;
} L9963E_GPIO5_MEASTypeDef;

typedef struct {
    uint32_t    :14,
                ratio_abs_6_sel     :1,
                d_rdy_gpio6         :1,
                GPIO6_MEAS          :16;
} L9963E_GPIO6_MEASTypeDef;

typedef struct {
    uint32_t    :14,
                ratio_abs_7_sel     :1,
                d_rdy_gpio7         :1,
                GPIO7_MEAS          :16;
} L9963E_GPIO7_MEASTypeDef;

typedef struct {
    uint32_t    :14,
                ratio_abs_8_sel     :1,
                d_rdy_gpio8         :1,
                GPIO8_MEAS          :16;
} L9963E_GPIO8_MEASTypeDef;

typedef struct {
    uint32_t    :14,
                ratio_abs_9_sel     :1,
                d_rdy_gpio9         :1,
                GPIO9_MEAS          :16;
} L9963E_GPIO9_MEASTypeDef;

typedef struct {
    uint32_t    :14,
                Noreg17      :1,
                Noreg16      :1,
                Noreg15      :1,
                Noreg14      :1,
                Noreg13      :1,
                Noreg12      :1,
                Noreg11      :1,
                Noreg10      :1,
                Noreg9       :1,
                OTchip       :1,
                TempChip     :8;
} L9963E_TempChipTypeDef;

typedef struct {
    uint32_t    :14,
                Noreg17                 :1,
                Noreg16                 :1,
                Noreg15                 :1,
                Noreg14                 :1,
                VANA_OV                 :1,
                VDIG_OV                 :1,
                VTREF_UV                :1,
                VTREF_OV                :1,
                VREG_UV                 :1,
                VREG_OV                 :1,
                VCOM_OV                 :1,
                VCOM_UV                 :1,
                HeartBeat_fault         :1,
                FaultHline_fault        :1,
                Fault_L_line_status     :1,
                Noreg2                  :1,
                Noreg1                  :1,
                Comm_timeout_flt        :1;
} L9963E_Faults1TypeDef;

typedef struct {
    uint32_t    :14,
                Noreg17                  :1,
                Noreg16                  :1,
                Noreg15                  :1,
                Noreg14                  :1,
                Noreg13                  :1,
                SPIENlatch               :1,
                Noreg11                  :1,
                OSCFail                  :1,
                Noreg9                   :1,
                loss_agnd                :1,
                loss_dgnd                :1,
                loss_cgnd                :1,
                loss_gndref              :1,
                Noreg4                   :1,
                CoCouOvF                 :1,
                EoBtimeerror             :1,
                curr_sense_ovc_sleep     :1,
                curr_sense_ovc_norm      :1;
} L9963E_Faults2TypeDef;

typedef struct {
    uint32_t    :14,
                Noreg17        :1,
                Noreg16        :1,
                BAL14_OPEN     :1,
                BAL13_OPEN     :1,
                BAL12_OPEN     :1,
                BAL11_OPEN     :1,
                BAL10_OPEN     :1,
                BAL9_OPEN      :1,
                BAL8_OPEN      :1,
                BAL7_OPEN      :1,
                BAL6_OPEN      :1,
                BAL5_OPEN      :1,
                BAL4_OPEN      :1,
                BAL3_OPEN      :1,
                BAL2_OPEN      :1,
                BAL1_OPEN      :1,
                Noreg1         :1,
                Noreg0         :1;
} L9963E_BAL_OPENTypeDef;

typedef struct {
    uint32_t    :14,
                Noreg17         :1,
                Noreg16         :1,
                BAL14_SHORT     :1,
                BAL13_SHORT     :1,
                BAL12_SHORT     :1,
                BAL11_SHORT     :1,
                BAL10_SHORT     :1,
                BAL9_SHORT      :1,
                BAL8_SHORT      :1,
                BAL7_SHORT      :1,
                BAL6_SHORT      :1,
                BAL5_SHORT      :1,
                BAL4_SHORT      :1,
                BAL3_SHORT      :1,
                BAL2_SHORT      :1,
                BAL1_SHORT      :1,
                Noreg1          :1,
                Noreg0          :1;
} L9963E_BAL_SHORTTypeDef;

typedef struct {
    uint32_t    :14,
                vsum_batt19_2     :18;
} L9963E_VSUMBATTTypeDef;

typedef struct {
    uint32_t    :14,
                vsum_batt1_0     :2,
                VBATT_DIV        :16;
} L9963E_VBATTDIVTypeDef;

typedef struct {
    uint32_t    :14,
                data_ready_vsum         :1,
                data_ready_vbattdiv     :1,
                Noreg15                 :1,
                CELL14_OPEN             :1,
                CELL13_OPEN             :1,
                CELL12_OPEN             :1,
                CELL11_OPEN             :1,
                CELL10_OPEN             :1,
                CELL9_OPEN              :1,
                CELL8_OPEN              :1,
                CELL7_OPEN              :1,
                CELL6_OPEN              :1,
                CELL5_OPEN              :1,
                CELL4_OPEN              :1,
                CELL3_OPEN              :1,
                CELL2_OPEN              :1,
                CELL1_OPEN              :1,
                CELL0_OPEN              :1;
} L9963E_CELL_OPENTypeDef;

typedef struct {
    uint32_t    :14,
                Noreg17          :1,
                VBATT_WRN_UV     :1,
                VBATTCRIT_UV     :1,
                VSUM_UV          :1,
                VCELL14_UV       :1,
                VCELL13_UV       :1,
                VCELL12_UV       :1,
                VCELL11_UV       :1,
                VCELL10_UV       :1,
                VCELL9_UV        :1,
                VCELL8_UV        :1,
                VCELL7_UV        :1,
                VCELL6_UV        :1,
                VCELL5_UV        :1,
                VCELL4_UV        :1,
                VCELL3_UV        :1,
                VCELL2_UV        :1,
                VCELL1_UV        :1;
} L9963E_VCELL_UVTypeDef;

typedef struct {
    uint32_t    :14,
                Noreg17          :1,
                VBATT_WRN_OV     :1,
                VBATTCRIT_OV     :1,
                VSUM_OV          :1,
                VCELL14_OV       :1,
                VCELL13_OV       :1,
                VCELL12_OV       :1,
                VCELL11_OV       :1,
                VCELL10_OV       :1,
                VCELL9_OV        :1,
                VCELL8_OV        :1,
                VCELL7_OV        :1,
                VCELL6_OV        :1,
                VCELL5_OV        :1,
                VCELL4_OV        :1,
                VCELL3_OV        :1,
                VCELL2_OV        :1,
                VCELL1_OV        :1;
} L9963E_VCELL_OVTypeDef;

typedef struct {
    uint32_t    :14,
                Noreg17      :1,
                Noreg16      :1,
                Noreg15      :1,
                Noreg14      :1,
                GPIO9_OT     :1,
                GPIO8_OT     :1,
                GPIO7_OT     :1,
                GPIO6_OT     :1,
                GPIO5_OT     :1,
                GPIO4_OT     :1,
                GPIO3_OT     :1,
                GPIO9_UT     :1,
                GPIO8_UT     :1,
                GPIO7_UT     :1,
                GPIO6_UT     :1,
                GPIO5_UT     :1,
                GPIO4_UT     :1,
                GPIO3_UT     :1;
} L9963E_VGPIO_OT_UTTypeDef;

typedef struct {
    uint32_t    :14,
                Noreg17            :1,
                Noreg16            :1,
                Noreg15            :1,
                Noreg14            :1,
                VCELL14_BAL_UV     :1,
                VCELL13_BAL_UV     :1,
                VCELL12_BAL_UV     :1,
                VCELL11_BAL_UV     :1,
                VCELL10_BAL_UV     :1,
                VCELL9_BAL_UV      :1,
                VCELL8_BAL_UV      :1,
                VCELL7_BAL_UV      :1,
                VCELL6_BAL_UV      :1,
                VCELL5_BAL_UV      :1,
                VCELL4_BAL_UV      :1,
                VCELL3_BAL_UV      :1,
                VCELL2_BAL_UV      :1,
                VCELL1_BAL_UV      :1;
} L9963E_VCELL_BAL_UVTypeDef;

typedef struct {
    uint32_t    :14,
                Noreg17              :1,
                Noreg16              :1,
                Noreg15              :1,
                Noreg14              :1,
                GPIO9_OPEN           :1,
                GPIO8_OPEN           :1,
                GPIO7_OPEN           :1,
                GPIO6_OPEN           :1,
                GPIO5_OPEN           :1,
                GPIO4_OPEN           :1,
                GPIO3_OPEN           :1,
                GPIO9_fastchg_OT     :1,
                GPIO8_fastchg_OT     :1,
                GPIO7_fastchg_OT     :1,
                GPIO6_fastchg_OT     :1,
                GPIO5_fastchg_OT     :1,
                GPIO4_fastchg_OT     :1,
                GPIO3_fastchg_OT     :1;
} L9963E_GPIO_fastchg_OTTypeDef;

typedef struct {
    uint32_t    :14,
                Noreg17           :1,
                Noreg16           :1,
                Noreg15           :1,
                HWSC_DONE         :1,
                MUX_BIST_FAIL     :14;
} L9963E_MUX_BIST_FAILTypeDef;

typedef struct {
    uint32_t    :14,
                VBAT_COMP_BIST_FAIL       :1,
                VREG_COMP_BIST_FAIL       :1,
                VCOM_COMP_BIST_FAIL       :1,
                VTREF_COMP_BIST_FAIL      :1,
                BIST_BAL_COMP_HS_FAIL     :7,
                BIST_BAL_COMP_LS_FAIL     :7;
} L9963E_BIST_COMPTypeDef;

typedef struct {
    uint32_t    :14,
                Noreg17            :1,
                Noreg16            :1,
                Noreg15            :1,
                Noreg14            :1,
                OPEN_BIST_FAIL     :14;
} L9963E_OPEN_BIST_FAILTypeDef;

typedef struct {
    uint32_t    :14,
                GPO9short           :1,
                GPO8short           :1,
                GPO7short           :1,
                GPO6short           :1,
                GPO5short           :1,
                GPO4short           :1,
                GPO3short           :1,
                Noreg10             :1,
                Noreg9              :1,
                Noreg8              :1,
                VTREF_BIST_FAIL     :1,
                GPIO_BIST_FAIL      :7;
} L9963E_GPIO_BIST_FAILTypeDef;

typedef struct {
    uint32_t    :14,
                Noreg17         :1,
                d_rdy_vtref     :1,
                VTREF_MEAS      :16;
} L9963E_VTREFTypeDef;

typedef struct {
    uint32_t    :14,
                Noreg17         :1,
                Noreg16         :1,
                NVM_WR_15_0     :16;
} L9963E_NVM_WR_1TypeDef;

typedef struct {
    uint32_t    :14,
                Noreg17          :1,
                Noreg16          :1,
                NVM_WR_31_16     :16;
} L9963E_NVM_WR_2TypeDef;

typedef struct {
    uint32_t    :14,
                Noreg17          :1,
                Noreg16          :1,
                NVM_WR_47_32     :16;
} L9963E_NVM_WR_3TypeDef;

typedef struct {
    uint32_t    :14,
                Noreg17          :1,
                Noreg16          :1,
                NVM_WR_63_48     :16;
} L9963E_NVM_WR_4TypeDef;

typedef struct {
    uint32_t    :14,
                Noreg17          :1,
                Noreg16          :1,
                NVM_WR_79_64     :16;
} L9963E_NVM_WR_5TypeDef;

typedef struct {
    uint32_t    :14,
                Noreg17          :1,
                Noreg16          :1,
                NVM_WR_95_80     :16;
} L9963E_NVM_WR_6TypeDef;

typedef struct {
    uint32_t    :14,
                Noreg17           :1,
                Noreg16           :1,
                NVM_WR_111_96     :16;
} L9963E_NVM_WR_7TypeDef;

typedef struct {
    uint32_t    :14,
                Noreg17         :1,
                Noreg16         :1,
                NVM_RD_15_0     :16;
} L9963E_NVM_RD_1TypeDef;

typedef struct {
    uint32_t    :14,
                Noreg17          :1,
                Noreg16          :1,
                NVM_RD_31_16     :16;
} L9963E_NVM_RD_2TypeDef;

typedef struct {
    uint32_t    :14,
                Noreg17          :1,
                Noreg16          :1,
                NVM_RD_47_32     :16;
} L9963E_NVM_RD_3TypeDef;

typedef struct {
    uint32_t    :14,
                Noreg17          :1,
                Noreg16          :1,
                NVM_RD_63_48     :16;
} L9963E_NVM_RD_4TypeDef;

typedef struct {
    uint32_t    :14,
                Noreg17          :1,
                Noreg16          :1,
                NVM_RD_79_64     :16;
} L9963E_NVM_RD_5TypeDef;

typedef struct {
    uint32_t    :14,
                Noreg17          :1,
                Noreg16          :1,
                NVM_RD_95_80     :16;
} L9963E_NVM_RD_6TypeDef;

typedef struct {
    uint32_t    :14,
                Noreg17           :1,
                Noreg16           :1,
                NVM_RD_111_96     :16;
} L9963E_NVM_RD_7TypeDef;

typedef struct {
    uint32_t    :14,
                Noreg17         :1,
                Noreg16         :1,
                Noreg15         :1,
                Noreg14         :1,
                Noreg13         :1,
                Noreg12         :1,
                NVM_WR_BUSY     :1,
                NVM_OPER        :2,
                NVM_PROGRAM     :1,
                NVM_CNTR        :8;
} L9963E_NVM_CMD_CNTRTypeDef;

typedef struct {
    uint32_t    :14,
                NVM_UNLOCK_START     :18;
} L9963E_NVM_UNLCK_PRGTypeDef;


typedef union {
    uint32_t generic;
    L9963E_DEV_GEN_CFGTypeDef                    DEV_GEN_CFG;
    L9963E_fastch_baluvTypeDef                   fastch_baluv;
    L9963E_Bal_1TypeDef                          Bal_1;
    L9963E_Bal_2TypeDef                          Bal_2;
    L9963E_Bal_3TypeDef                          Bal_3;
    L9963E_Bal_4TypeDef                          Bal_4;
    L9963E_Bal_5TypeDef                          Bal_5;
    L9963E_Bal_6TypeDef                          Bal_6;
    L9963E_Bal_7TypeDef                          Bal_7;
    L9963E_Bal_8TypeDef                          Bal_8;
    L9963E_VCELL_THRESH_UV_OVTypeDef             VCELL_THRESH_UV_OV;
    L9963E_VBATT_SUM_THTypeDef                   VBATT_SUM_TH;
    L9963E_ADCV_CONVTypeDef                      ADCV_CONV;
    L9963E_NCYCLE_PROG_1TypeDef                  NCYCLE_PROG_1;
    L9963E_NCYCLE_PROG_2TypeDef                  NCYCLE_PROG_2;
    L9963E_BalCell14_7actTypeDef                 BalCell14_7act;
    L9963E_BalCell6_1actTypeDef                  BalCell6_1act;
    L9963E_FSMTypeDef                            FSM;
    L9963E_GPOxOn_and_GPI93TypeDef               GPOxOn_and_GPI93;
    L9963E_GPIO9_3_CONFTypeDef                   GPIO9_3_CONF;
    L9963E_GPIO3_THRTypeDef                      GPIO3_THR;
    L9963E_GPIO4_THRTypeDef                      GPIO4_THR;
    L9963E_GPIO5_THRTypeDef                      GPIO5_THR;
    L9963E_GPIO6_THRTypeDef                      GPIO6_THR;
    L9963E_GPIO7_THRTypeDef                      GPIO7_THR;
    L9963E_GPIO8_THRTypeDef                      GPIO8_THR;
    L9963E_GPIO9_THRTypeDef                      GPIO9_THR;
    L9963E_VCELLS_ENTypeDef                      VCELLS_EN;
    L9963E_FaultmaskTypeDef                      Faultmask;
    L9963E_Faultmask2TypeDef                     Faultmask2;
    L9963E_CSA_THRESH_NORMTypeDef                CSA_THRESH_NORM;
    L9963E_CSA_GPIO_MSKTypeDef                   CSA_GPIO_MSK;
    L9963E_Vcell1TypeDef                         Vcell1;
    L9963E_Vcell2TypeDef                         Vcell2;
    L9963E_Vcell3TypeDef                         Vcell3;
    L9963E_Vcell4TypeDef                         Vcell4;
    L9963E_Vcell5TypeDef                         Vcell5;
    L9963E_Vcell6TypeDef                         Vcell6;
    L9963E_Vcell7TypeDef                         Vcell7;
    L9963E_Vcell8TypeDef                         Vcell8;
    L9963E_Vcell9TypeDef                         Vcell9;
    L9963E_Vcell10TypeDef                        Vcell10;
    L9963E_Vcell11TypeDef                        Vcell11;
    L9963E_Vcell12TypeDef                        Vcell12;
    L9963E_Vcell13TypeDef                        Vcell13;
    L9963E_Vcell14TypeDef                        Vcell14;
    L9963E_Ibattery_synchTypeDef                 Ibattery_synch;
    L9963E_Ibattery_calibTypeDef                 Ibattery_calib;
    L9963E_CoulCntrTimeTypeDef                   CoulCntrTime;
    L9963E_CoulCntr_msbTypeDef                   CoulCntr_msb;
    L9963E_CoulCntr_lsbTypeDef                   CoulCntr_lsb;
    L9963E_GPIO3_MEASTypeDef                     GPIO3_MEAS;
    L9963E_GPIO4_MEASTypeDef                     GPIO4_MEAS;
    L9963E_GPIO5_MEASTypeDef                     GPIO5_MEAS;
    L9963E_GPIO6_MEASTypeDef                     GPIO6_MEAS;
    L9963E_GPIO7_MEASTypeDef                     GPIO7_MEAS;
    L9963E_GPIO8_MEASTypeDef                     GPIO8_MEAS;
    L9963E_GPIO9_MEASTypeDef                     GPIO9_MEAS;
    L9963E_TempChipTypeDef                       TempChip;
    L9963E_Faults1TypeDef                        Faults1;
    L9963E_Faults2TypeDef                        Faults2;
    L9963E_BAL_OPENTypeDef                       BAL_OPEN;
    L9963E_BAL_SHORTTypeDef                      BAL_SHORT;
    L9963E_VSUMBATTTypeDef                       VSUMBATT;
    L9963E_VBATTDIVTypeDef                       VBATTDIV;
    L9963E_CELL_OPENTypeDef                      CELL_OPEN;
    L9963E_VCELL_UVTypeDef                       VCELL_UV;
    L9963E_VCELL_OVTypeDef                       VCELL_OV;
    L9963E_VGPIO_OT_UTTypeDef                    VGPIO_OT_UT;
    L9963E_VCELL_BAL_UVTypeDef                   VCELL_BAL_UV;
    L9963E_GPIO_fastchg_OTTypeDef                GPIO_fastchg_OT;
    L9963E_MUX_BIST_FAILTypeDef                  MUX_BIST_FAIL;
    L9963E_BIST_COMPTypeDef                      BIST_COMP;
    L9963E_OPEN_BIST_FAILTypeDef                 OPEN_BIST_FAIL;
    L9963E_GPIO_BIST_FAILTypeDef                 GPIO_BIST_FAIL;
    L9963E_VTREFTypeDef                          VTREF;
    L9963E_NVM_WR_1TypeDef                       NVM_WR_1;
    L9963E_NVM_WR_2TypeDef                       NVM_WR_2;
    L9963E_NVM_WR_3TypeDef                       NVM_WR_3;
    L9963E_NVM_WR_4TypeDef                       NVM_WR_4;
    L9963E_NVM_WR_5TypeDef                       NVM_WR_5;
    L9963E_NVM_WR_6TypeDef                       NVM_WR_6;
    L9963E_NVM_WR_7TypeDef                       NVM_WR_7;
    L9963E_NVM_RD_1TypeDef                       NVM_RD_1;
    L9963E_NVM_RD_2TypeDef                       NVM_RD_2;
    L9963E_NVM_RD_3TypeDef                       NVM_RD_3;
    L9963E_NVM_RD_4TypeDef                       NVM_RD_4;
    L9963E_NVM_RD_5TypeDef                       NVM_RD_5;
    L9963E_NVM_RD_6TypeDef                       NVM_RD_6;
    L9963E_NVM_RD_7TypeDef                       NVM_RD_7;
    L9963E_NVM_CMD_CNTRTypeDef                   NVM_CMD_CNTR;
    L9963E_NVM_UNLCK_PRGTypeDef                  NVM_UNLCK_PRG;
    
} L9963E_RegisterUnionTypeDef;