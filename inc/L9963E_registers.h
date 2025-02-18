/*
 * "THE BEER-WARE LICENSE" (Revision 69):
 * Squadra Corse firmware team wrote this file. As long as you retain this notice
 * you can do whatever you want with this stuff. If we meet some day, and you 
 * think this stuff is worth it, you can buy us a beer in return.
 * 
 * Authors
 * - Federico Carbone [federico.carbone.sc@gmail.com]
 * - Filippo Rossi [filippo.rossi.sc@gmail.com]
 */

// clang-format off

#include <inttypes.h>

typedef enum {
    L9963E_DEV_GEN_CFG_ADDR            = 0x1,
    L9963E_fastch_baluv_ADDR           = 0x2,
    L9963E_Bal_1_ADDR                  = 0x3,
    L9963E_Bal_2_ADDR                  = 0x4,
    L9963E_Bal_3_ADDR                  = 0x5,
    L9963E_Bal_4_ADDR                  = 0x6,
    L9963E_Bal_5_ADDR                  = 0x7,
    L9963E_Bal_6_ADDR                  = 0x8,
    L9963E_Bal_7_ADDR                  = 0x9,
    L9963E_Bal_8_ADDR                  = 0xA,
    L9963E_VCELL_THRESH_UV_OV_ADDR     = 0xB,
    L9963E_VBATT_SUM_TH_ADDR           = 0xC,
    L9963E_ADCV_CONV_ADDR              = 0xD,
    L9963E_NCYCLE_PROG_1_ADDR          = 0xE,
    L9963E_NCYCLE_PROG_2_ADDR          = 0xF,
    L9963E_BalCell14_7act_ADDR         = 0x10,
    L9963E_BalCell6_1act_ADDR          = 0x11,
    L9963E_FSM_ADDR                    = 0x12,
    L9963E_GPOxOn_and_GPI93_ADDR       = 0x13,
    L9963E_GPIO9_3_CONF_ADDR           = 0x14,
    L9963E_GPIO3_THR_ADDR              = 0x15,
    L9963E_GPIO4_THR_ADDR              = 0x16,
    L9963E_GPIO5_THR_ADDR              = 0x17,
    L9963E_GPIO6_THR_ADDR              = 0x18,
    L9963E_GPIO7_THR_ADDR              = 0x19,
    L9963E_GPIO8_THR_ADDR              = 0x1A,
    L9963E_GPIO9_THR_ADDR              = 0x1B,
    L9963E_VCELLS_EN_ADDR              = 0x1C,
    L9963E_Faultmask_ADDR              = 0x1D,
    L9963E_Faultmask2_ADDR             = 0x1E,
    L9963E_CSA_THRESH_NORM_ADDR        = 0x1F,
    L9963E_CSA_GPIO_MSK_ADDR           = 0x20,
    L9963E_Vcell1_ADDR                 = 0x21,
    L9963E_Vcell2_ADDR                 = 0x22,
    L9963E_Vcell3_ADDR                 = 0x23,
    L9963E_Vcell4_ADDR                 = 0x24,
    L9963E_Vcell5_ADDR                 = 0x25,
    L9963E_Vcell6_ADDR                 = 0x26,
    L9963E_Vcell7_ADDR                 = 0x27,
    L9963E_Vcell8_ADDR                 = 0x28,
    L9963E_Vcell9_ADDR                 = 0x29,
    L9963E_Vcell10_ADDR                = 0x2A,
    L9963E_Vcell11_ADDR                = 0x2B,
    L9963E_Vcell12_ADDR                = 0x2C,
    L9963E_Vcell13_ADDR                = 0x2D,
    L9963E_Vcell14_ADDR                = 0x2E,
    L9963E_Ibattery_synch_ADDR         = 0x2F,
    L9963E_Ibattery_calib_ADDR         = 0x30,
    L9963E_CoulCntrTime_ADDR           = 0x31,
    L9963E_CoulCntr_msb_ADDR           = 0x32,
    L9963E_CoulCntr_lsb_ADDR           = 0x33,
    L9963E_GPIO3_MEAS_ADDR             = 0x34,
    L9963E_GPIO4_MEAS_ADDR             = 0x35,
    L9963E_GPIO5_MEAS_ADDR             = 0x36,
    L9963E_GPIO6_MEAS_ADDR             = 0x37,
    L9963E_GPIO7_MEAS_ADDR             = 0x38,
    L9963E_GPIO8_MEAS_ADDR             = 0x39,
    L9963E_GPIO9_MEAS_ADDR             = 0x3A,
    L9963E_TempChip_ADDR               = 0x3B,
    L9963E_Faults1_ADDR                = 0x3C,
    L9963E_Faults2_ADDR                = 0x3D,
    L9963E_BAL_OPEN_ADDR               = 0x3E,
    L9963E_BAL_SHORT_ADDR              = 0x3F,
    L9963E_VSUMBATT_ADDR               = 0x40,
    L9963E_VBATTDIV_ADDR               = 0x41,
    L9963E_CELL_OPEN_ADDR              = 0x42,
    L9963E_VCELL_UV_ADDR               = 0x43,
    L9963E_VCELL_OV_ADDR               = 0x44,
    L9963E_VGPIO_OT_UT_ADDR            = 0x45,
    L9963E_VCELL_BAL_UV_ADDR           = 0x46,
    L9963E_GPIO_fastchg_OT_ADDR        = 0x47,
    L9963E_MUX_BIST_FAIL_ADDR          = 0x48,
    L9963E_BIST_COMP_ADDR              = 0x49,
    L9963E_OPEN_BIST_FAIL_ADDR         = 0x4A,
    L9963E_GPIO_BIST_FAIL_ADDR         = 0x4B,
    L9963E_VTREF_ADDR                  = 0x4C,
    L9963E_NVM_WR_1_ADDR               = 0x4D,
    L9963E_NVM_WR_2_ADDR               = 0x4E,
    L9963E_NVM_WR_3_ADDR               = 0x4F,
    L9963E_NVM_WR_4_ADDR               = 0x50,
    L9963E_NVM_WR_5_ADDR               = 0x51,
    L9963E_NVM_WR_6_ADDR               = 0x52,
    L9963E_NVM_WR_7_ADDR               = 0x53,
    L9963E_NVM_RD_1_ADDR               = 0x54,
    L9963E_NVM_RD_2_ADDR               = 0x55,
    L9963E_NVM_RD_3_ADDR               = 0x56,
    L9963E_NVM_RD_4_ADDR               = 0x57,
    L9963E_NVM_RD_5_ADDR               = 0x58,
    L9963E_NVM_RD_6_ADDR               = 0x59,
    L9963E_NVM_RD_7_ADDR               = 0x5A,
    L9963E_NVM_CMD_CNTR_ADDR           = 0x5B,
    L9963E_NVM_UNLCK_PRG_ADDR          = 0x5C
} L9963E_RegistersAddrTypeDef;


typedef struct {
    uint32_t    /* 0 → FAULTL not forced high1 → FAULTL forced high */
                FaultL_force       :1,/* Configures the unit as the stacktopmostSee Table 36 */
                Farthest_Unit      :1,/* Enables Heartbeat generationSee Table 36 */
                HeartBeat_En       :1,/* Enables FAULTH receiverSee Table 36 */
                FaultH_EN          :1,/* Selects heartbeat periodSee Table 37 */
                HeartBeatCycle     :3,/* Reserved */
                Noreg7             :1,/* Selects ISOH/L port carrier frequencySee Table 18 */
                iso_freq_sel       :2,/* Selects ISOH/L port differential signalamplitudeSee Table 18 */
                out_res_tx_iso     :2,/* 0 → ISOH port disabled1 → ISOH port enabled */
                isotx_en_h         :1,/* All 0s → No address (Init state)X → Dev ID of SPI Protocol (L9963ESPI Protocol Details) */
                chip_ID            :5,
/* Padding */
                :14;
} L9963E_DEV_GEN_CFGTypeDef;

typedef struct {
    uint32_t    /* Determines the balancingundervoltage thresholdSee Table 39 */
                Vcell_bal_UV_delta_thr        :8,/* Determines the fastchargeovertemperature thresholdSee Table 48 */
                Gpio_fastchg_OT_delta_thr     :8,/* Configures the communicationtimeoutSee Table 11 */
                CommTimeout                   :2,
/* Padding */
                :14;
} L9963E_fastch_baluvTypeDef;

typedef struct {
    uint32_t    /* Balancing timer watchdog. Resolutiondepends on TimedBalaccSee Table 43 */
                WDTimedBalTimer      :7,/* Balancing timer. Resolution dependson TimedBalaccSee Table 43 */
                TimedBalTimer        :7,/* 10 → balancing start01 → balancing stopOthers → no effect */
                bal_stop             :1,
                bal_start            :1,/* 0 → Silent balancing disabled1 → Silent balancing enabled */
                slp_bal_conf         :1,/* 0 → Communication timeout enabled1 → Communication timeout disabled */
                comm_timeout_dis     :1,
/* Padding */
                :14;
} L9963E_Bal_1TypeDef;

typedef struct {
    uint32_t    /* Timed balancing threshold for cell 13 */
                ThrTimedBalCell13     :7,/* Reserved */
                Noreg7                :1,/* Timed balancing threshold for cell 14 */
                ThrTimedBalCell14     :7,/* Selects balancing timer resolutionSee Table 43 */
                TimedBalacc           :1,/* 01 → Manual balancing10 → Timed balancingOthers → No effect */
                Balmode               :2,
/* Padding */
                :14;
} L9963E_Bal_2TypeDef;

typedef struct {
    uint32_t    /* Timed balancing threshold for cell 11 */
                ThrTimedBalCell11      :7,/* Reserved */
                Noreg7                 :1,/* Timed balancing threshold for cell 12 */
                ThrTimedBalCell12      :7,/* 1 → isotx_en_h and iso_freq_sel arewrite protected */
                Lock_isoh_isofreq      :1,/* Triggers NVM downloadRefer to Section  4.10.1  NVM read */
                trimming_retrigger     :1,/* 0 → First powerup not properly done1 → First powerup ended successfully */
                first_wup_done         :1,
/* Padding */
                :14;
} L9963E_Bal_3TypeDef;

typedef struct {
    uint32_t    /* Timed balancing threshold for cell 9 */
                ThrTimedBalCell9      :7,/* Reserved */
                Noreg7                :1,/* Timed balancing threshold for cell 10 */
                ThrTimedBalCell10     :7,/* 0 → Main oscillator monitor notstarted1 → Main oscillator monitor started */
                clk_mon_init_done     :1,/* Reserved */
                Noreg16               :1,/* 0 → Main oscillator monitor disabled1 → Main oscillator monitor enabled */
                clk_mon_en            :1,
/* Padding */
                :14;
} L9963E_Bal_4TypeDef;

typedef struct {
    uint32_t    /* Timed balancing threshold for cell 7 */
                ThrTimedBalCell7            :7,/* Reserved */
                Noreg7                      :1,/* Timed balancing threshold for cell 8 */
                ThrTimedBalCell8            :7,/* Reserved */
                Noreg15                     :1,/* 0 → Value on transceiver_on_by_updiscarded1 → Value on transceiver_on_by_upapplied to device configuration */
                transceiver_valid_by_up     :1,/* 0 → Transceiver mode not forced byMCU1 → Transceiver mode forced byMCU */
                transceiver_on_by_up        :1,
/* Padding */
                :14;
} L9963E_Bal_5TypeDef;

typedef struct {
    uint32_t    /* Timed balancing threshold for cell 5 */
                ThrTimedBalCell5     :7,/* Reserved */
                Noreg7               :1,/* Timed balancing threshold for cell 6 */
                ThrTimedBalCell6     :7,/* Reserved */
                Noreg15              :1,/* Reserved */
                Noreg16              :1,/* Reserved */
                Noreg17              :1,
/* Padding */
                :14;
} L9963E_Bal_6TypeDef;

typedef struct {
    uint32_t    /* Timed balancing threshold for cell 3 */
                ThrTimedBalCell3     :7,/* Reserved */
                Noreg7               :1,/* Timed balancing threshold for cell 4 */
                ThrTimedBalCell4     :7,/* Reserved */
                Noreg15              :1,/* Reserved */
                Noreg16              :1,/* Reserved */
                Noreg17              :1,
/* Padding */
                :14;
} L9963E_Bal_7TypeDef;

typedef struct {
    uint32_t    /* Timed balancing threshold for cell 1 */
                ThrTimedBalCell1     :7,/* Reserved */
                Noreg7               :1,/* Timed balancing threshold for cell 2 */
                ThrTimedBalCell2     :7,/* Reserved */
                Noreg15              :1,/* Reserved */
                Noreg16              :1,/* Reserved */
                Noreg17              :1,
/* Padding */
                :14;
} L9963E_Bal_8TypeDef;

typedef struct {
    uint32_t    /* Determines cell undervoltagethresholdSee Table 39 */
                threshVcellUV     :8,/* Determines cell overvoltage thresholdCell Voltage ADC electricalcharacteristicsSee Table 39 */
                threshVcellOV     :8,/* Reserved */
                Noreg16           :1,/* Reserved */
                Noreg17           :1,
/* Padding */
                :14;
} L9963E_VCELL_THRESH_UV_OVTypeDef;

typedef struct {
    uint32_t    /* Determines battery stackundervoltage thresholdSee Table 40 */
                VBATT_SUM_UV_TH     :8,/* Determines battery stack overvoltagethresholdSee Table 40 */
                VBATT_SUM_OV_TH     :8,/* Reserved */
                Noreg16             :1,/* Reserved */
                Noreg17             :1,
/* Padding */
                :14;
} L9963E_VBATT_SUM_THTypeDef;

typedef struct {
    uint32_t    /* 0 → Cyclic conversions triggeredperiodically by TCYCLE timer1 → Cyclic conversions performedcontinuously */
                CYCLIC_CONTINOUS     :1,/* Determines the period of cyclicconversions executed in Normalstate.See Table 68 */
                TCYCLE               :3,/* 0 → HWSC disabled for on-demandconversion1 → HWSC performed during on-demand conversion */
                HWSC                 :1,/* 0 → Balancing open check disabledfor on-demand conversion1 → Balancing open check performedduring on-demand conversion */
                BAL_TERM_CONV        :1,/* 0 → Cx open check disabled for on-demand conversion1 → Cx open check performed duringon-demand conversion */
                CELL_TERM_CONV       :1,/* 0 → GPIO open check disabled foron-demand conversion1 → GPIO open check performedduring on-demand conversion */
                GPIO_TERM_CONV       :1,/* 0 → GPIO conversion disabled for on-demand conversion1 → GPIO conversion performedduring on-demand conversion */
                GPIO_CONV            :1,/* Determines the filter window used foron-demand conversions.See Table 39 */
                ADC_FILTER_SOC       :3,/* 0 → No conversion ongoing1 → Conversion ongoing */
                DUTY_ON              :1,/* 0 → Cyclic conversions disabled1 → Cyclic conversions enabled */
                CONF_CYCLIC_EN       :1,/* 1 → Configuration override occurred */
                OVR_LATCH            :1,/* 0 → No on-demand conversion1 → Triggers on-demand conversion */
                SOC                  :1,/* 0 → No period overflow detectedduring cyclic conversions1 → Period overflow detected duringcyclic conversions */
                TCYCLE_OVF           :1,/* 0 → Cell open diagnostics executedduring Cx open check1 → ADC Cross check executedduring Cx open check */
                ADC_CROSS_CHECK      :1,
/* Padding */
                :14;
} L9963E_ADCV_CONVTypeDef;

typedef struct {
    uint32_t    /* Reserved */
                Noreg0                    :1,/* 0 → PCB open diagnostic currentdisabled on even cells1 → PCB open diagnostic currentenabled on even cells */
                PCB_open_en_even_curr     :1,/* 0 → PCB open diagnostic currentdisabled on odd cells1 → PCB open diagnostic currentenabled on odd cells */
                PCB_open_en_odd_curr      :1,/* 0 → ADCs not swapped1 → ADCs swapped */
                CROSS_ODD_EVEN_CELL       :1,/* 0 → Measurement registers notupdated during cyclic conversions1 → Measurement registers updatedduring cyclic conversions */
                CYCLIC_UPDATE             :1,/* 0 → Balancing auto pause disabled1 → Balancing auto pause enabled */
                BAL_AUTO_PAUSE            :1,/* 0 → Balancing timer not frozen duringbalancing auto pause1 → Balancing timer frozen duringbalancing auto pause */
                BAL_TIM_AUTO_PAUSE        :1,/* Determines Balancing open checkperiodicity during cyclic executionsSee Table 68 */
                NCYCLE_BAL_TERM           :3,/* Determines Cx open check periodicityduring cyclic executionsSee Table 68 */
                NCYCLE_CELL_TERM          :3,/* Determines GPIO open checkperiodicity during cyclic executionsSee Table 68 */
                NCYCLE_GPIO_TERM          :3,/* 00 → No settling time01 → 175 μs settling time10 → 350 μs settling time11 → 700 μs settling time */
                T_CELL_SET                :2,
/* Padding */
                :14;
} L9963E_NCYCLE_PROG_1TypeDef;

typedef struct {
    uint32_t    /* Determines the filter window used forcyclic conversion in cyclic wake upstate.See Table 39 */
                ADC_FILTER_SLEEP     :3,/* Determines the period of cyclicconversions executed in cyclic wakeup.See Table 68 */
                TCYCLE_SLEEP         :3,/* Determines the filter window used forcyclic conversion in Normal state.See Table 39 */
                ADC_FILTER_CYCLE     :3,/* Reserved */
                Noreg9               :1,/* Determines HWSC periodicity duringcyclic executionsSee Table 68 */
                NCYCLE_HWSC          :3,/* Determines GPIO measurementperiodicity during cyclic executionsSee Table 68 */
                NCYCLE_GPIO          :3,/* 0 → VTREF regulator always ON1 → VTREF regulator turned ON onlyduring Voltage Conversion Routine */
                VTREF_DYN_EN         :1,/* 0 → VTREF regulator disabled1 → VTREF regulator enabled */
                VTREF_EN             :1,
/* Padding */
                :14;
} L9963E_NCYCLE_PROG_2TypeDef;

typedef struct {
    uint32_t    /* 10 → Balancing enabledOthers → Balancing disabled */
                BAL7        :2,
                BAL8        :2,
                BAL9        :2,
                BAL10       :2,
                BAL11       :2,
                BAL12       :2,
                BAL13       :2,
                BAL14       :2,/* Reserved */
                Noreg16     :1,/* Reserved */
                Noreg17     :1,
/* Padding */
                :14;
} L9963E_BalCell14_7actTypeDef;

typedef struct {
    uint32_t    /* See Table 42 */
                eof_bal     :1,
                bal_on      :1,/* Reserved */
                Noreg2      :1,/* Reserved */
                Noreg3      :1,/* 10 → Balancing enabledOthers → Balancing disabled */
                BAL1        :2,
                BAL2        :2,
                BAL3        :2,
                BAL4        :2,
                BAL5        :2,
                BAL6        :2,/* Reserved */
                Noreg16     :1,/* Reserved */
                Noreg17     :1,
/* Padding */
                :14;
} L9963E_BalCell6_1actTypeDef;

typedef struct {
    uint32_t    /* 0 → Last wake up source was notTCYCLE_SLEEP1 → Last wake up source wasTCYCLE_SLEEP */
                wu_cyc_wup     :1,/* 0 → Last wake up source was notFAULTH1 → Last wake up source wasFAULTH */
                wu_faulth      :1,/* 0 → Last wake up source was notisolated SPI1 → Last wake up source wasisolated SPI */
                wu_isoline     :1,/* 0 → Last wake up source was not SPI1 → Last wake up source was SPI */
                wu_spi         :1,/* 0 → Last wake up source was notGPIO71 → Last wake up source was GPIO7 */
                wu_gpio7       :1,/* Reserved */
                Noreg5         :1,/* Reserved */
                Noreg6         :1,/* Reserved */
                Noreg7         :1,/* 0001 → Sleep0010 → Init0100 → Normal1000 → Cyclic wake up */
                FSMstatus      :4,/* 10 → Moves the device to sleepOthers → No effect */
                GO2SLP         :2,/* 10 → Triggers software resetOthers → No effect */
                SW_RST         :2,/* Reserved */
                Noreg16        :1,/* Reserved */
                Noreg17        :1,
/* Padding */
                :14;
} L9963E_FSMTypeDef;

typedef struct {
    uint32_t    /* Reserved */
                Noreg0      :1,/* Reserved */
                Noreg1      :1,/* Value read on GPIO */
                GPI3        :1,/* Value read on GPIO */
                GPI4        :1,
                GPI5        :1,
                GPI6        :1,
                GPI7        :1,
                GPI8        :1,
                GPI9        :1,/* Reserved */
                Noreg9      :1,/* Reserved */
                Noreg10     :1,/* 0 → GPIO forced low1 → GPIO forced high */
                GPO3on      :1,
                GPO4on      :1,
                GPO5on      :1,
                GPO6on      :1,
                GPO7on      :1,
                GPO8on      :1,
                GPO9on      :1,
/* Padding */
                :14;
} L9963E_GPOxOn_and_GPI93TypeDef;

typedef struct {
    uint32_t    /* Reserved */
                Noreg0           :1,/* Reserved */
                Noreg1           :1,/* Reserved */
                Noreg2           :1,/* 0 → GPIO7 not used as wake upinput1 → GPIO7 used as wake up input */
                GPIO7_WUP_EN     :1,/* 00 → Analog input01 → Not to be used10 → Digital input11 → Digital output */
                GPIO3_CONFIG     :2,
                GPIO4_CONFIG     :2,
                GPIO5_CONFIG     :2,
                GPIO6_CONFIG     :2,
                GPIO7_CONFIG     :2,
                GPIO8_CONFIG     :2,
                GPIO9_CONFIG     :2,
/* Padding */
                :14;
} L9963E_GPIO9_3_CONFTypeDef;

typedef struct {
    uint32_t    /* Determines GPIO3 undertemperaturethresholdSee Table 48 */
                GPIO3_UT_TH     :9,/* Determines GPIO3 overtemperaturethresholdSee Table 48 */
                GPIO3_OT_TH     :9,
/* Padding */
                :14;
} L9963E_GPIO3_THRTypeDef;

typedef struct {
    uint32_t    /* Determines GPIO4 undertemperaturethresholdSee Table 48 */
                GPIO4_UT_TH     :9,/* Determines GPIO4 overtemperaturethresholdSee Table 48 */
                GPIO4_OT_TH     :9,
/* Padding */
                :14;
} L9963E_GPIO4_THRTypeDef;

typedef struct {
    uint32_t    /* Determines GPIO5 undertemperaturethresholdSee Table 48 */
                GPIO5_UT_TH     :9,/* Determines GPIO5 overtemperaturethresholdSee Table 48 */
                GPIO5_OT_TH     :9,
/* Padding */
                :14;
} L9963E_GPIO5_THRTypeDef;

typedef struct {
    uint32_t    /* Determines GPIO6 undertemperaturethresholdSee Table 48 */
                GPIO6_UT_TH     :9,/* See Table 48 */
                GPIO6_OT_TH     :9,
/* Padding */
                :14;
} L9963E_GPIO6_THRTypeDef;

typedef struct {
    uint32_t    /* Determines GPIO7 undertemperaturethresholdSee Table 48 */
                GPIO7_UT_TH     :9,/* Determines GPIO7 overtemperaturethresholdSee Table 48 */
                GPIO7_OT_TH     :9,
/* Padding */
                :14;
} L9963E_GPIO7_THRTypeDef;

typedef struct {
    uint32_t    /* Determines GPIO8 undertemperaturethresholdSee Table 48 */
                GPIO8_UT_TH     :9,/* Determines GPIO8 overtemperaturethresholdSee Table 48 */
                GPIO8_OT_TH     :9,
/* Padding */
                :14;
} L9963E_GPIO8_THRTypeDef;

typedef struct {
    uint32_t    /* Determines GPIO9 undertemperaturethresholdSee Table 48 */
                GPIO9_UT_TH     :9,/* Determines GPIO9 overtemperaturethresholdSee Table 48 */
                GPIO9_OT_TH     :9,
/* Padding */
                :14;
} L9963E_GPIO9_THRTypeDef;

typedef struct {
    uint32_t    /* 0 → Cell disabled1 → Cell enabled */
                VCELL1_EN      :1,
                VCELL2_EN      :1,
                VCELL3_EN      :1,
                VCELL4_EN      :1,/* 0 → Cell disabled1 → Cell enabled */
                VCELL5_EN      :1,
                VCELL6_EN      :1,
                VCELL7_EN      :1,
                VCELL8_EN      :1,
                VCELL9_EN      :1,
                VCELL10_EN     :1,
                VCELL11_EN     :1,
                VCELL12_EN     :1,
                VCELL13_EN     :1,
                VCELL14_EN     :1,/* Reserved */
                Noreg14        :1,/* Reserved */
                Noreg15        :1,/* Reserved */
                Noreg16        :1,/* Reserved */
                Noreg17        :1,
/* Padding */
                :14;
} L9963E_VCELLS_ENTypeDef;

typedef struct {
    uint32_t    /* 0 → Balancing undervoltage notmasked1 → Balancing undervoltage masked */
                VCELL1_BAL_UV_MSK      :1,
                VCELL2_BAL_UV_MSK      :1,
                VCELL3_BAL_UV_MSK      :1,
                VCELL4_BAL_UV_MSK      :1,
                VCELL5_BAL_UV_MSK      :1,
                VCELL6_BAL_UV_MSK      :1,
                VCELL7_BAL_UV_MSK      :1,
                VCELL8_BAL_UV_MSK      :1,
                VCELL9_BAL_UV_MSK      :1,
                VCELL10_BAL_UV_MSK     :1,
                VCELL11_BAL_UV_MSK     :1,
                VCELL12_BAL_UV_MSK     :1,
                VCELL13_BAL_UV_MSK     :1,
                VCELL14_BAL_UV_MSK     :1,/* Reserved */
                Noreg14                :1,/* Reserved */
                Noreg15                :1,/* Reserved */
                Noreg16                :1,/* Reserved */
                Noreg17                :1,
/* Padding */
                :14;
} L9963E_FaultmaskTypeDef;

typedef struct {
    uint32_t    /* 0 → GPIO fast chargeovertemperature not masked1 → GPIO fast chargeovertemperature masked */
                Gpio3_fastchg_OT_MSK          :1,
                Gpio4_fastchg_OT_MSK          :1,
                Gpio5_fastchg_OT_MSK          :1,
                Gpio6_fastchg_OT_MSK          :1,
                Gpio7_fastchg_OT_MSK          :1,
                Gpio8_fastchg_OT_MSK          :1,
                Gpio9_fastchg_OT_MSK          :1,/* 0 → Trimming and calibration dataare corrupted1 → Trimming and calibration dataare integer */
                TrimmCalOk                    :1,/* 0 → No attempt to download NVMwas executed1 → Attempt to download NVMexecuted */
                trim_dwnl_tried               :1,/* 0 → CRC error in RAM content notmasked1 → CRC error in RAM contentmasked */
                RAM_CRC_ERRMSK                :1,/* 0 → No CRC error in RAM content1 → CRC error in RAM content */
                RAM_CRC_ERR                   :1,/* 0 → CRC error in current calibrationdata not masked1 → CRC error in current calibrationdata masked */
                EEPROM_CRC_ERRMSK_CAL_FF      :1,/* 0 → No CRC error in currentcalibration data1 → CRC error in current calibrationdata */
                EEPROM_CRC_ERR_CAL_FF         :1,/* 0 → CRC error in voltage calibrationdata not masked1 → CRC error in voltage calibrationdata masked */
                EEPROM_CRC_ERRMSK_CAL_RAM     :1,/* 0 → No CRC error in voltagecalibration data1 → CRC error in voltage calibrationdata */
                EEPROM_CRC_ERR_CAL_RAM        :1,/* 0 → CRC error in trimming data notmasked1 → CRC error in trimming datamasked */
                EEPROM_CRC_ERRMSK_SECT_0      :1,/* 0 → No CRC error in trimming data1 → CRC error in trimming data */
                EEPROM_CRC_ERR_SECT_0         :1,/* 0 → NVM not downloaded1 → NVM downloaded */
                EEPROM_DWNLD_DONE             :1,
/* Padding */
                :14;
} L9963E_Faultmask2TypeDef;

typedef struct {
    uint32_t    /* Determines the CSA overcurrentthreshold in Normal stateSee Table 41 */
                adc_ovc_curr_threshold_norm     :18,
/* Padding */
                :14;
} L9963E_CSA_THRESH_NORMTypeDef;

typedef struct {
    uint32_t    /* 0 → GPIO over/under temperaturenot masked1 → GPIO over/under temperaturemasked */
                Gpio3_OT_UT_MSK                  :1,
                Gpio4_OT_UT_MSK                  :1,
                Gpio5_OT_UT_MSK                  :1,
                Gpio6_OT_UT_MSK                  :1,
                Gpio7_OT_UT_MSK                  :1,
                Gpio8_OT_UT_MSK                  :1,
                Gpio9_OT_UT_MSK                  :1,/* Reserved */
                Noreg7                           :1,/* 0 → Open failure not detected onCSA inverting input1 → Open failure detected on CSAinverting input */
                sense_minus_open                 :1,/* 0 → Open failure not detected onCSA non-inverting input1 → Open failure detected on CSAnon-inverting input */
                sense_plus_open                  :1,/* 0 → CSA overcurrent in normal notmasked1 → CSA overcurrent in normalmasked */
                ovc_norm_msk                     :1,/* 0 → CSA overcurrent in cyclic wakeup not masked1 → CSA overcurrent in cyclic wakeup masked */
                ovc_sleep_msk                    :1,/* 0 → Coulomb Counter disabled1 → Coulomb Counter enabled */
                CoulombCounter_en                :1,/* Determines the CSA overcurrentthreshold in Cyclic wake up stateSee Table 41 */
                adc_ovc_curr_threshold_sleep     :5,
/* Padding */
                :14;
} L9963E_CSA_GPIO_MSKTypeDef;

typedef struct {
    uint32_t    /* Cell 1 voltage measurement */
                VCell1           :16,/* 0 → Data was already read once1 → Fresh new data */
                d_rdy_Vcell1     :1,/* Reserved */
                Noreg17          :1,
/* Padding */
                :14;
} L9963E_Vcell1TypeDef;

typedef struct {
    uint32_t    /* Cell 2 voltage measurement */
                VCell2           :16,/* 0 → Data was already read once1 → Fresh new data */
                d_rdy_Vcell2     :1,/* Reserved */
                Noreg17          :1,
/* Padding */
                :14;
} L9963E_Vcell2TypeDef;

typedef struct {
    uint32_t    /* Cell 3 voltage measurement */
                VCell3           :16,/* 0 → Data was already read once1 → Fresh new data */
                d_rdy_Vcell3     :1,/* Reserved */
                Noreg17          :1,
/* Padding */
                :14;
} L9963E_Vcell3TypeDef;

typedef struct {
    uint32_t    /* Cell 4 voltage measurement */
                VCell4           :16,/* 0 → Data was already read once1 → Fresh new data */
                d_rdy_Vcell4     :1,/* Reserved */
                Noreg17          :1,
/* Padding */
                :14;
} L9963E_Vcell4TypeDef;

typedef struct {
    uint32_t    /* Cell 5 voltage measurement */
                VCell5           :16,/* 0 → Data was already read once1 → Fresh new data */
                d_rdy_Vcell5     :1,/* Reserved */
                Noreg17          :1,
/* Padding */
                :14;
} L9963E_Vcell5TypeDef;

typedef struct {
    uint32_t    /* Cell 6 voltage measurement */
                VCell6           :16,/* 0 → Data was already read once1 → Fresh new data */
                d_rdy_Vcell6     :1,/* Reserved */
                Noreg17          :1,
/* Padding */
                :14;
} L9963E_Vcell6TypeDef;

typedef struct {
    uint32_t    /* Cell 7 voltage measurement */
                VCell7           :16,/* 0 → Data was already read once1 → Fresh new data */
                d_rdy_Vcell7     :1,/* Reserved */
                Noreg17          :1,
/* Padding */
                :14;
} L9963E_Vcell7TypeDef;

typedef struct {
    uint32_t    /* Cell 8 voltage measurement */
                VCell8           :16,/* 0 → Data was already read once1 → Fresh new data */
                d_rdy_Vcell8     :1,/* Reserved */
                Noreg17          :1,
/* Padding */
                :14;
} L9963E_Vcell8TypeDef;

typedef struct {
    uint32_t    /* Cell 9 voltage measurement */
                VCell9           :16,/* 0 → Data was already read once1 → Fresh new data */
                d_rdy_Vcell9     :1,/* Reserved */
                Noreg17          :1,
/* Padding */
                :14;
} L9963E_Vcell9TypeDef;

typedef struct {
    uint32_t    /* Cell 10 voltage measurement */
                VCell10           :16,/* 0 → Data was already read once1 → Fresh new data */
                d_rdy_Vcell10     :1,/* Reserved */
                Noreg17           :1,
/* Padding */
                :14;
} L9963E_Vcell10TypeDef;

typedef struct {
    uint32_t    /* Cell 11 voltage measurement */
                VCell11           :16,/* 0 → Data was already read once1 → Fresh new data */
                d_rdy_Vcell11     :1,/* Reserved */
                Noreg17           :1,
/* Padding */
                :14;
} L9963E_Vcell11TypeDef;

typedef struct {
    uint32_t    /* Cell 12 voltage measurement */
                VCell12           :16,/* 0 → Data was already read once1 → Fresh new data */
                d_rdy_Vcell12     :1,/* Reserved */
                Noreg17           :1,
/* Padding */
                :14;
} L9963E_Vcell12TypeDef;

typedef struct {
    uint32_t    /* Cell 13 voltage measurement */
                VCell13           :16,/* 0 → Data was already read once1 → Fresh new data */
                d_rdy_Vcell13     :1,/* Reserved */
                Noreg17           :1,
/* Padding */
                :14;
} L9963E_Vcell13TypeDef;

typedef struct {
    uint32_t    /* Cell 14 voltage measurement */
                VCell14           :16,/* 0 → Data was already read once1 → Fresh new data */
                d_rdy_Vcell14     :1,/* Reserved */
                Noreg17           :1,
/* Padding */
                :14;
} L9963E_Vcell14TypeDef;

typedef struct {
    uint32_t    /* Pack current sample synchronizedwith last on-demand conversion */
                CUR_INST_Synch     :18,
/* Padding */
                :14;
} L9963E_Ibattery_synchTypeDef;

typedef struct {
    uint32_t    /* Pack current sample continuouslyupdated */
                CUR_INST_calib     :18,
/* Padding */
                :14;
} L9963E_Ibattery_calibTypeDef;

typedef struct {
    uint32_t    /* Number of current samples acquired */
                CoulombCntTime     :16,/* Reserved */
                Noreg16            :1,/* Reserved */
                Noreg17            :1,
/* Padding */
                :14;
} L9963E_CoulCntrTimeTypeDef;

typedef struct {
    uint32_t    /* Current sample accumulator (MSB) */
                CoulombCounter_msb     :16,/* Reserved */
                Noreg16                :1,/* Reserved */
                Noreg17                :1,
/* Padding */
                :14;
} L9963E_CoulCntr_msbTypeDef;

typedef struct {
    uint32_t    /* Current sample accumulator (LSB) */
                CoulombCounter_lsb     :16,/* Reserved */
                Noreg16                :1,/* Reserved */
                Noreg17                :1,
/* Padding */
                :14;
} L9963E_CoulCntr_lsbTypeDef;

typedef struct {
    uint32_t    /* GPIO 3 measurement data */
                GPIO3_MEAS          :16,/* 0 → Data was already read once1 → Fresh new data */
                d_rdy_gpio3         :1,/* 0 → Absolute value1 → Ratiometric value */
                ratio_abs_3_sel     :1,
/* Padding */
                :14;
} L9963E_GPIO3_MEASTypeDef;

typedef struct {
    uint32_t    /* GPIO 4 measurement data */
                GPIO4_MEAS          :16,/* 0 → Data was already read once1 → Fresh new data */
                d_rdy_gpio4         :1,/* 0 → Absolute value1 → Ratiometric value */
                ratio_abs_4_sel     :1,
/* Padding */
                :14;
} L9963E_GPIO4_MEASTypeDef;

typedef struct {
    uint32_t    /* GPIO 5 measurement data */
                GPIO5_MEAS          :16,/* 0 → Data was already read once1 → Fresh new data */
                d_rdy_gpio5         :1,/* 0 → Absolute value1 → Ratiometric value */
                ratio_abs_5_sel     :1,
/* Padding */
                :14;
} L9963E_GPIO5_MEASTypeDef;

typedef struct {
    uint32_t    /* GPIO 6 measurement data */
                GPIO6_MEAS          :16,/* 0 → Data was already read once1 → Fresh new data */
                d_rdy_gpio6         :1,/* 0 → Absolute value1 → Ratiometric value */
                ratio_abs_6_sel     :1,
/* Padding */
                :14;
} L9963E_GPIO6_MEASTypeDef;

typedef struct {
    uint32_t    /* GPIO 7 measurement data */
                GPIO7_MEAS          :16,/* 0 → Data was already read once1 → Fresh new data */
                d_rdy_gpio7         :1,/* 0 → Absolute value1 → Ratiometric value */
                ratio_abs_7_sel     :1,
/* Padding */
                :14;
} L9963E_GPIO7_MEASTypeDef;

typedef struct {
    uint32_t    /* GPIO 8 measurement data */
                GPIO8_MEAS          :16,/* 0 → Data was already read once1 → Fresh new data */
                d_rdy_gpio8         :1,/* 0 → Absolute value1 → Ratiometric value */
                ratio_abs_8_sel     :1,
/* Padding */
                :14;
} L9963E_GPIO8_MEASTypeDef;

typedef struct {
    uint32_t    /* GPIO 9 measurement data */
                GPIO9_MEAS          :16,/* 0 → Data was already read once1 → Fresh new data */
                d_rdy_gpio9         :1,/* 0 → Absolute value1 → Ratiometric value */
                ratio_abs_9_sel     :1,
/* Padding */
                :14;
} L9963E_GPIO9_MEASTypeDef;

typedef struct {
    uint32_t    /* Device temperature data */
                TempChip     :8,/* 0 → No chip overtemperaturedetected1 → Chip overtemperature detected */
                OTchip       :1,/* Reserved */
                Noreg9       :1,/* Reserved */
                Noreg10      :1,/* Reserved */
                Noreg11      :1,/* Reserved */
                Noreg12      :1,/* Reserved */
                Noreg13      :1,/* Reserved */
                Noreg14      :1,/* Reserved */
                Noreg15      :1,/* Reserved */
                Noreg16      :1,/* Reserved */
                Noreg17      :1,
/* Padding */
                :14;
} L9963E_TempChipTypeDef;

typedef struct {
    uint32_t    /* 0 → Communication timeout notexpired1 → Communication timeout expired */
                Comm_timeout_flt        :1,/* Reserved */
                Noreg1                  :1,/* Reserved */
                Noreg2                  :1,/* Reads FAULTL pin value */
                Fault_L_line_status     :1,/* 1 → Fault incoming from upper level */
                FaultHline_fault        :1,/* 0 → Heartbeat absence not detected1 → Heartbeat absence detected */
                HeartBeat_fault         :1,/* 0 → VCOM undervoltage notdetected1 → VCOM undervoltage detected */
                VCOM_UV                 :1,/* 0 → VCOM overvoltage not detected1 → VCOM overvoltage detected */
                VCOM_OV                 :1,/* 0 → VREG overvoltage not detected1 → VREG overvoltage detected */
                VREG_OV                 :1,/* 0 → VREG undervoltage not detected1 → VREG undervoltage detected */
                VREG_UV                 :1,/* 0 → VTREF overvoltage not detected1 → VTREF overvoltage detected */
                VTREF_OV                :1,/* 0 → VTREF undervoltage notdetected1 → VTREF undervoltage detected */
                VTREF_UV                :1,/* 0 → VDIG overvoltage not detected1 → VDIG overvoltage detected */
                VDIG_OV                 :1,/* 0 → VANA overvoltage not detected1 → VANA overvoltage detected */
                VANA_OV                 :1,/* Reserved */
                Noreg14                 :1,/* Reserved */
                Noreg15                 :1,/* Reserved */
                Noreg16                 :1,/* Reserved */
                Noreg17                 :1,
/* Padding */
                :14;
} L9963E_Faults1TypeDef;

typedef struct {
    uint32_t    /* 0 → CSA overcurrent not detectedduring normal state1 → CSA overcurrent detected duringnormal state */
                curr_sense_ovc_norm      :1,/* 0 → CSA overcurrent not detectedduring cyclic wake up1 → CSA overcurrent detected duringcyclic wake up */
                curr_sense_ovc_sleep     :1,/* 0 → No error detected betweenbalancing timers1 → Error detected betweenwatchdog and primary balancing timer */
                EoBtimeerror             :1,/* 0 → Coulomb Counter not overflown1 → Coulomb Counter overflown */
                CoCouOvF                 :1,/* Reserved */
                Noreg4                   :1,/* 0 → GND not lost1 → GND lost */
                loss_gndref              :1,
                loss_cgnd                :1,
                loss_dgnd                :1,
                loss_agnd                :1,/* Reserved */
                Noreg9                   :1,/* 0 → Main oscillator not stuck/out ofrange1 → Main oscillator stuck/out of range */
                OSCFail                  :1,/* Reserved */
                Noreg11                  :1,/* Value latched on SPIEN at powerup */
                SPIENlatch               :1,/* Reserved */
                Noreg13                  :1,/* Reserved */
                Noreg14                  :1,/* Reserved */
                Noreg15                  :1,/* Reserved */
                Noreg16                  :1,/* Reserved */
                Noreg17                  :1,
/* Padding */
                :14;
} L9963E_Faults2TypeDef;

typedef struct {
    uint32_t    /* Reserved */
                Noreg0         :1,/* Reserved */
                Noreg1         :1,/* 0 → Balancing open not detected1 → Balancing open detected */
                BAL1_OPEN      :1,
                BAL2_OPEN      :1,
                BAL3_OPEN      :1,
                BAL4_OPEN      :1,
                BAL5_OPEN      :1,
                BAL6_OPEN      :1,
                BAL7_OPEN      :1,
                BAL8_OPEN      :1,
                BAL9_OPEN      :1,
                BAL10_OPEN     :1,
                BAL11_OPEN     :1,
                BAL12_OPEN     :1,
                BAL13_OPEN     :1,
                BAL14_OPEN     :1,/* Reserved */
                Noreg16        :1,/* Reserved */
                Noreg17        :1,
/* Padding */
                :14;
} L9963E_BAL_OPENTypeDef;

typedef struct {
    uint32_t    /* Reserved */
                Noreg0          :1,/* Reserved */
                Noreg1          :1,/* 0 → Balancing short not detected1 → Balancing short detected */
                BAL1_SHORT      :1,
                BAL2_SHORT      :1,
                BAL3_SHORT      :1,
                BAL4_SHORT      :1,
                BAL5_SHORT      :1,
                BAL6_SHORT      :1,
                BAL7_SHORT      :1,
                BAL8_SHORT      :1,
                BAL9_SHORT      :1,
                BAL10_SHORT     :1,
                BAL11_SHORT     :1,
                BAL12_SHORT     :1,
                BAL13_SHORT     :1,
                BAL14_SHORT     :1,/* Reserved */
                Noreg16         :1,/* Reserved */
                Noreg17         :1,
/* Padding */
                :14;
} L9963E_BAL_SHORTTypeDef;

typedef struct {
    uint32_t    /* Digital sum of cells (MSB) */
                vsum_batt19_2     :18,
/* Padding */
                :14;
} L9963E_VSUMBATTTypeDef;

typedef struct {
    uint32_t    /* VBAT direct conversion data */
                VBATT_DIV        :16,/* Digital sum of cells (LSB) */
                vsum_batt1_0     :2,
/* Padding */
                :14;
} L9963E_VBATTDIVTypeDef;

typedef struct {
    uint32_t    /* 0 → Cx open not detected1 → Cx open detected */
                CELL0_OPEN              :1,
                CELL1_OPEN              :1,
                CELL2_OPEN              :1,
                CELL3_OPEN              :1,
                CELL4_OPEN              :1,
                CELL5_OPEN              :1,
                CELL6_OPEN              :1,
                CELL7_OPEN              :1,
                CELL8_OPEN              :1,
                CELL9_OPEN              :1,
                CELL10_OPEN             :1,
                CELL11_OPEN             :1,
                CELL12_OPEN             :1,
                CELL13_OPEN             :1,
                CELL14_OPEN             :1,/* Reserved */
                Noreg15                 :1,/* 0 → Data was already read once1 → Fresh new data */
                data_ready_vbattdiv     :1,/* 0 → Data was already read once1 → Fresh new data */
                data_ready_vsum         :1,
/* Padding */
                :14;
} L9963E_CELL_OPENTypeDef;

typedef struct {
    uint32_t    /* 0 → Cell UV not detected1 → Cell UV detected */
                VCELL1_UV        :1,
                VCELL2_UV        :1,
                VCELL3_UV        :1,
                VCELL4_UV        :1,
                VCELL5_UV        :1,
                VCELL6_UV        :1,
                VCELL7_UV        :1,
                VCELL8_UV        :1,
                VCELL9_UV        :1,/* 0 → Cell UV not detected1 → Cell UV detected */
                VCELL10_UV       :1,
                VCELL11_UV       :1,
                VCELL12_UV       :1,
                VCELL13_UV       :1,
                VCELL14_UV       :1,/* 0 → Sum of cells UV not detected1 → Sum of cells UV detected */
                VSUM_UV          :1,/* 0 → VBAT critical UV not detected1 → VBAT critical UV detected */
                VBATTCRIT_UV     :1,/* 0 → VBAT UV comparator nottriggered1 → VBAT UV comparator triggered */
                VBATT_WRN_UV     :1,/* Reserved */
                Noreg17          :1,
/* Padding */
                :14;
} L9963E_VCELL_UVTypeDef;

typedef struct {
    uint32_t    /* 0 → Cell OV not detected1 → Cell OV detected */
                VCELL1_OV        :1,
                VCELL2_OV        :1,
                VCELL3_OV        :1,
                VCELL4_OV        :1,
                VCELL5_OV        :1,
                VCELL6_OV        :1,
                VCELL7_OV        :1,
                VCELL8_OV        :1,
                VCELL9_OV        :1,
                VCELL10_OV       :1,
                VCELL11_OV       :1,
                VCELL12_OV       :1,
                VCELL13_OV       :1,
                VCELL14_OV       :1,/* 0 → Sum of cells OV not detected1 → Sum of cells OV detected */
                VSUM_OV          :1,/* 0 → VBAT critical OV not detected1 → VBAT critical OV detected */
                VBATTCRIT_OV     :1,/* 0 → VBAT OV comparator nottriggered1 → VBAT OV comparator triggered */
                VBATT_WRN_OV     :1,/* Reserved */
                Noreg17          :1,
/* Padding */
                :14;
} L9963E_VCELL_OVTypeDef;

typedef struct {
    uint32_t    /* 0 → GPIO UT not detected1 → GPIO UT detected */
                GPIO3_UT     :1,
                GPIO4_UT     :1,
                GPIO5_UT     :1,
                GPIO6_UT     :1,
                GPIO7_UT     :1,
                GPIO8_UT     :1,
                GPIO9_UT     :1,/* 0 → GPIO OT not detected1 → GPIO OT detected */
                GPIO3_OT     :1,
                GPIO4_OT     :1,
                GPIO5_OT     :1,
                GPIO6_OT     :1,
                GPIO7_OT     :1,
                GPIO8_OT     :1,
                GPIO9_OT     :1,/* Reserved */
                Noreg14      :1,/* Reserved */
                Noreg15      :1,/* Reserved */
                Noreg16      :1,/* Reserved */
                Noreg17      :1,
/* Padding */
                :14;
} L9963E_VGPIO_OT_UTTypeDef;

typedef struct {
    uint32_t    /* 0 → Cell balancing UV not detected1 → Cell balancing UV detected */
                VCELL1_BAL_UV      :1,
                VCELL2_BAL_UV      :1,
                VCELL3_BAL_UV      :1,
                VCELL4_BAL_UV      :1,
                VCELL5_BAL_UV      :1,
                VCELL6_BAL_UV      :1,
                VCELL7_BAL_UV      :1,
                VCELL8_BAL_UV      :1,
                VCELL9_BAL_UV      :1,
                VCELL10_BAL_UV     :1,
                VCELL11_BAL_UV     :1,
                VCELL12_BAL_UV     :1,
                VCELL13_BAL_UV     :1,
                VCELL14_BAL_UV     :1,/* Reserved */
                Noreg14            :1,/* Reserved */
                Noreg15            :1,/* Reserved */
                Noreg16            :1,/* Reserved */
                Noreg17            :1,
/* Padding */
                :14;
} L9963E_VCELL_BAL_UVTypeDef;

typedef struct {
    uint32_t    /* 0 → GPIO fast charge OT notdetected1 → GPIO fast charge OT detected */
                GPIO3_fastchg_OT     :1,
                GPIO4_fastchg_OT     :1,
                GPIO5_fastchg_OT     :1,
                GPIO6_fastchg_OT     :1,
                GPIO7_fastchg_OT     :1,
                GPIO8_fastchg_OT     :1,
                GPIO9_fastchg_OT     :1,/* 0 → GPIO open not detected1 → GPIO open detected */
                GPIO3_OPEN           :1,
                GPIO4_OPEN           :1,
                GPIO5_OPEN           :1,
                GPIO6_OPEN           :1,
                GPIO7_OPEN           :1,
                GPIO8_OPEN           :1,
                GPIO9_OPEN           :1,/* Reserved */
                Noreg14              :1,/* Reserved */
                Noreg15              :1,/* Reserved */
                Noreg16              :1,/* Reserved */
                Noreg17              :1,
/* Padding */
                :14;
} L9963E_GPIO_fastchg_OTTypeDef;

typedef struct {
    uint32_t    /* 0 → No failure detected on Cxconversion path1 → Failure detected on Cxconversion path */
                MUX_BIST_FAIL     :14,/* 0 → HWSC not executed1 → HWSC terminated */
                HWSC_DONE         :1,/* Reserved */
                Noreg15           :1,/* Reserved */
                Noreg16           :1,/* Reserved */
                Noreg17           :1,
/* Padding */
                :14;
} L9963E_MUX_BIST_FAILTypeDef;

typedef struct {
    uint32_t    /* 0 → Balancing comparator BIST notfailed on odd cells1 → Balancing comparator BISTfailed on odd cells */
                BIST_BAL_COMP_LS_FAIL     :7,/* 1 → Balancing comparator BISTfailed on even cells */
                BIST_BAL_COMP_HS_FAIL     :7,/* 0 → VTREF comparator BIST notfailed1 → VTREF comparator BIST failed */
                VTREF_COMP_BIST_FAIL      :1,/* 0 → VCOM comparator BIST notfailed1 → VCOM comparator BIST failed */
                VCOM_COMP_BIST_FAIL       :1,/* 0 → VREG comparator BIST notfailed1 → VREG comparator BIST failed */
                VREG_COMP_BIST_FAIL       :1,/* 0 → VBAT comparator BIST not failed1 → VBAT comparator BIST failed */
                VBAT_COMP_BIST_FAIL       :1,
/* Padding */
                :14;
} L9963E_BIST_COMPTypeDef;

typedef struct {
    uint32_t    /* 0 → No failure detected on Sx/Bx_x-1conversion path1 → Failure detected on Sx/Bx_x-1conversion path */
                OPEN_BIST_FAIL     :14,/* Reserved */
                Noreg14            :1,/* Reserved */
                Noreg15            :1,/* Reserved */
                Noreg16            :1,/* Reserved */
                Noreg17            :1,
/* Padding */
                :14;
} L9963E_OPEN_BIST_FAILTypeDef;

typedef struct {
    uint32_t    /* 0 → No failure detected on GPIOconversion path1 → Failure detected on GPIOconversion path */
                GPIO_BIST_FAIL      :7,/* 0 → VCOM comparator BIST notfailed1 → VCOM comparator BIST failed */
                VTREF_BIST_FAIL     :1,/* Reserved */
                Noreg8              :1,/* Reserved */
                Noreg9              :1,/* Reserved */
                Noreg10             :1,/* 0 → No failure detected on Sx/Bx_x-1conversion path1 → Failure detected on Sx/Bx_x-1conversion path */
                GPO3short           :1,
                GPO4short           :1,
                GPO5short           :1,
                GPO6short           :1,
                GPO7short           :1,
                GPO8short           :1,
                GPO9short           :1,
/* Padding */
                :14;
} L9963E_GPIO_BIST_FAILTypeDef;

typedef struct {
    uint32_t    /* VTREF conversion data */
                VTREF_MEAS      :16,/* 0 → Data was already read once1 → Fresh new data */
                d_rdy_vtref     :1,/* Reserved */
                Noreg17         :1,
/* Padding */
                :14;
} L9963E_VTREFTypeDef;

typedef struct {
    uint32_t    /* Write buffer for NVM sector 0_15 */
                NVM_WR_15_0     :16,/* Reserved */
                Noreg16         :1,/* Reserved */
                Noreg17         :1,
/* Padding */
                :14;
} L9963E_NVM_WR_1TypeDef;

typedef struct {
    uint32_t    /* Write buffer for NVM sector 16_31 */
                NVM_WR_31_16     :16,/* Reserved */
                Noreg16          :1,/* Reserved */
                Noreg17          :1,
/* Padding */
                :14;
} L9963E_NVM_WR_2TypeDef;

typedef struct {
    uint32_t    /* Write buffer for NVM sector 32_47 */
                NVM_WR_47_32     :16,/* Reserved */
                Noreg16          :1,/* Reserved */
                Noreg17          :1,
/* Padding */
                :14;
} L9963E_NVM_WR_3TypeDef;

typedef struct {
    uint32_t    /* Write buffer for NVM sector 48_63 */
                NVM_WR_63_48     :16,/* Reserved */
                Noreg16          :1,/* Reserved */
                Noreg17          :1,
/* Padding */
                :14;
} L9963E_NVM_WR_4TypeDef;

typedef struct {
    uint32_t    /* Write buffer for NVM sector 64_79 */
                NVM_WR_79_64     :16,/* Reserved */
                Noreg16          :1,/* Reserved */
                Noreg17          :1,
/* Padding */
                :14;
} L9963E_NVM_WR_5TypeDef;

typedef struct {
    uint32_t    /* Write buffer for NVM sector 80_95 */
                NVM_WR_95_80     :16,/* Reserved */
                Noreg16          :1,/* Reserved */
                Noreg17          :1,
/* Padding */
                :14;
} L9963E_NVM_WR_6TypeDef;

typedef struct {
    uint32_t    /* Write buffer for NVM sector 96_111 */
                NVM_WR_111_96     :16,/* Reserved */
                Noreg16           :1,/* Reserved */
                Noreg17           :1,
/* Padding */
                :14;
} L9963E_NVM_WR_7TypeDef;

typedef struct {
    uint32_t    /* Read buffer for NVM sector 0_15 */
                NVM_RD_15_0     :16,/* Reserved */
                Noreg16         :1,/* Reserved */
                Noreg17         :1,
/* Padding */
                :14;
} L9963E_NVM_RD_1TypeDef;

typedef struct {
    uint32_t    /* Read buffer for NVM sector 16_31 */
                NVM_RD_31_16     :16,/* Reserved */
                Noreg16          :1,/* Reserved */
                Noreg17          :1,
/* Padding */
                :14;
} L9963E_NVM_RD_2TypeDef;

typedef struct {
    uint32_t    /* Read buffer for NVM sector 32_47 */
                NVM_RD_47_32     :16,/* Reserved */
                Noreg16          :1,/* Reserved */
                Noreg17          :1,
/* Padding */
                :14;
} L9963E_NVM_RD_3TypeDef;

typedef struct {
    uint32_t    /* Read buffer for NVM sector 48_63 */
                NVM_RD_63_48     :16,/* Reserved */
                Noreg16          :1,/* Reserved */
                Noreg17          :1,
/* Padding */
                :14;
} L9963E_NVM_RD_4TypeDef;

typedef struct {
    uint32_t    /* Read buffer for NVM sector 64_79 */
                NVM_RD_79_64     :16,/* Reserved */
                Noreg16          :1,/* Reserved */
                Noreg17          :1,
/* Padding */
                :14;
} L9963E_NVM_RD_5TypeDef;

typedef struct {
    uint32_t    /* Read buffer for NVM sector 80_95 */
                NVM_RD_95_80     :16,/* Reserved */
                Noreg16          :1,/* Reserved */
                Noreg17          :1,
/* Padding */
                :14;
} L9963E_NVM_RD_6TypeDef;

typedef struct {
    uint32_t    /* Read buffer for NVM sector 96_111 */
                NVM_RD_111_96     :16,/* Reserved */
                Noreg16           :1,/* Reserved */
                Noreg17           :1,
/* Padding */
                :14;
} L9963E_NVM_RD_7TypeDef;

typedef struct {
    uint32_t    /* Counts the number of write cyclesexecutedSee Table 52 */
                NVM_CNTR        :8,/* 0 → NVM controller not armedto execute operation defined byNVM_OPER1 → NVM controller armed to executeoperation defined by NVM_OPER */
                NVM_PROGRAM     :1,/* 00/01 → No Action10 → Erase11 → Write */
                NVM_OPER        :2,/* 0 → NVM controller not busy1 → NVM controller busy */
                NVM_WR_BUSY     :1,/* Reserved */
                Noreg12         :1,/* Reserved */
                Noreg13         :1,/* Reserved */
                Noreg14         :1,/* Reserved */
                Noreg15         :1,/* Reserved */
                Noreg16         :1,/* Reserved */
                Noreg17         :1,
/* Padding */
                :14;
} L9963E_NVM_CMD_CNTRTypeDef;

typedef struct {
    uint32_t    
                NVM_UNLOCK_START     :18,
/* Padding */
                :14;
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


#define L9963E_DEV_GEN_CFG_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_DEV_GEN_CFGTypeDef){ .FaultL_force = 0x0, .Farthest_Unit = 0x0, .HeartBeat_En = 0x0, .FaultH_EN = 0x0, .HeartBeatCycle = 0x4, .Noreg7 = 0x0, .iso_freq_sel = 0x0, .out_res_tx_iso = 0x0, .isotx_en_h = 0x0, .chip_ID = 0x0 })).generic)
#define L9963E_FASTCH_BALUV_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_fastch_baluvTypeDef){ .Vcell_bal_UV_delta_thr = 0x0, .Gpio_fastchg_OT_delta_thr = 0x0, .CommTimeout = 0x0 })).generic)
#define L9963E_BAL_1_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_Bal_1TypeDef){ .WDTimedBalTimer = 0x0, .TimedBalTimer = 0x0, .bal_stop = 0x0, .bal_start = 0x0, .slp_bal_conf = 0x0, .comm_timeout_dis = 0x0 })).generic)
#define L9963E_BAL_2_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_Bal_2TypeDef){ .ThrTimedBalCell13 = 0x0, .Noreg7 = 0x0, .ThrTimedBalCell14 = 0x0, .TimedBalacc = 0x0, .Balmode = 0x1 })).generic)
#define L9963E_BAL_3_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_Bal_3TypeDef){ .ThrTimedBalCell11 = 0x0, .Noreg7 = 0x0, .ThrTimedBalCell12 = 0x0, .Lock_isoh_isofreq = 0x0, .trimming_retrigger = 0x0, .first_wup_done = 0x0 })).generic)
#define L9963E_BAL_4_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_Bal_4TypeDef){ .ThrTimedBalCell9 = 0x0, .Noreg7 = 0x0, .ThrTimedBalCell10 = 0x0, .clk_mon_init_done = 0x0, .Noreg16 = 0x0, .clk_mon_en = 0x0 })).generic)
#define L9963E_BAL_5_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_Bal_5TypeDef){ .ThrTimedBalCell7 = 0x0, .Noreg7 = 0x0, .ThrTimedBalCell8 = 0x0, .Noreg15 = 0x0, .transceiver_valid_by_up = 0x0, .transceiver_on_by_up = 0x0 })).generic)
#define L9963E_BAL_6_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_Bal_6TypeDef){ .ThrTimedBalCell5 = 0x0, .Noreg7 = 0x0, .ThrTimedBalCell6 = 0x0, .Noreg15 = 0x0, .Noreg16 = 0x0, .Noreg17 = 0x0 })).generic)
#define L9963E_BAL_7_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_Bal_7TypeDef){ .ThrTimedBalCell3 = 0x0, .Noreg7 = 0x0, .ThrTimedBalCell4 = 0x0, .Noreg15 = 0x0, .Noreg16 = 0x0, .Noreg17 = 0x0 })).generic)
#define L9963E_BAL_8_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_Bal_8TypeDef){ .ThrTimedBalCell1 = 0x0, .Noreg7 = 0x0, .ThrTimedBalCell2 = 0x0, .Noreg15 = 0x0, .Noreg16 = 0x0, .Noreg17 = 0x0 })).generic)
#define L9963E_VCELL_THRESH_UV_OV_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_VCELL_THRESH_UV_OVTypeDef){ .threshVcellUV = 0x0, .threshVcellOV = 0x0, .Noreg16 = 0x0, .Noreg17 = 0x0 })).generic)
#define L9963E_VBATT_SUM_TH_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_VBATT_SUM_THTypeDef){ .VBATT_SUM_UV_TH = 0x0, .VBATT_SUM_OV_TH = 0x0, .Noreg16 = 0x0, .Noreg17 = 0x0 })).generic)
#define L9963E_ADCV_CONV_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_ADCV_CONVTypeDef){ .CYCLIC_CONTINOUS = 0x0, .TCYCLE = 0x0, .HWSC = 0x0, .BAL_TERM_CONV = 0x0, .CELL_TERM_CONV = 0x0, .GPIO_TERM_CONV = 0x0, .GPIO_CONV = 0x0, .ADC_FILTER_SOC = 0x0, .DUTY_ON = 0x0, .CONF_CYCLIC_EN = 0x0, .OVR_LATCH = 0x0, .SOC = 0x0, .TCYCLE_OVF = 0x0, .ADC_CROSS_CHECK = 0x0 })).generic)
#define L9963E_NCYCLE_PROG_1_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_NCYCLE_PROG_1TypeDef){ .Noreg0 = 0x0, .PCB_open_en_even_curr = 0x0, .PCB_open_en_odd_curr = 0x0, .CROSS_ODD_EVEN_CELL = 0x0, .CYCLIC_UPDATE = 0x0, .BAL_AUTO_PAUSE = 0x1, .BAL_TIM_AUTO_PAUSE = 0x0, .NCYCLE_BAL_TERM = 0x0, .NCYCLE_CELL_TERM = 0x0, .NCYCLE_GPIO_TERM = 0x0, .T_CELL_SET = 0x0 })).generic)
#define L9963E_NCYCLE_PROG_2_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_NCYCLE_PROG_2TypeDef){ .ADC_FILTER_SLEEP = 0x0, .TCYCLE_SLEEP = 0x0, .ADC_FILTER_CYCLE = 0x0, .Noreg9 = 0x0, .NCYCLE_HWSC = 0x0, .NCYCLE_GPIO = 0x0, .VTREF_DYN_EN = 0x0, .VTREF_EN = 0x0 })).generic)
#define L9963E_BALCELL14_7ACT_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_BalCell14_7actTypeDef){ .BAL7 = 0x1, .BAL8 = 0x1, .BAL9 = 0x1, .BAL10 = 0x1, .BAL11 = 0x1, .BAL12 = 0x1, .BAL13 = 0x1, .BAL14 = 0x1, .Noreg16 = 0x0, .Noreg17 = 0x0 })).generic)
#define L9963E_BALCELL6_1ACT_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_BalCell6_1actTypeDef){ .eof_bal = 0x0, .bal_on = 0x0, .Noreg2 = 0x0, .Noreg3 = 0x0, .BAL1 = 0x1, .BAL2 = 0x1, .BAL3 = 0x1, .BAL4 = 0x1, .BAL5 = 0x1, .BAL6 = 0x1, .Noreg16 = 0x0, .Noreg17 = 0x0 })).generic)
#define L9963E_FSM_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_FSMTypeDef){ .wu_cyc_wup = 0x0, .wu_faulth = 0x0, .wu_isoline = 0x0, .wu_spi = 0x0, .wu_gpio7 = 0x0, .Noreg5 = 0x0, .Noreg6 = 0x0, .Noreg7 = 0x0, .FSMstatus = 0x0, .GO2SLP = 0x0, .SW_RST = 0x0, .Noreg16 = 0x0, .Noreg17 = 0x0 })).generic)
#define L9963E_GPOXON_AND_GPI93_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_GPOxOn_and_GPI93TypeDef){ .Noreg0 = 0x0, .Noreg1 = 0x0, .GPI3 = 0x0, .GPI4 = 0x0, .GPI5 = 0x0, .GPI6 = 0x0, .GPI7 = 0x0, .GPI8 = 0x0, .GPI9 = 0x0, .Noreg9 = 0x0, .Noreg10 = 0x0, .GPO3on = 0x0, .GPO4on = 0x0, .GPO5on = 0x0, .GPO6on = 0x0, .GPO7on = 0x0, .GPO8on = 0x0, .GPO9on = 0x0 })).generic)
#define L9963E_GPIO9_3_CONF_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_GPIO9_3_CONFTypeDef){ .Noreg0 = 0x0, .Noreg1 = 0x0, .Noreg2 = 0x0, .GPIO7_WUP_EN = 0x0, .GPIO3_CONFIG = 0x0, .GPIO4_CONFIG = 0x0, .GPIO5_CONFIG = 0x0, .GPIO6_CONFIG = 0x0, .GPIO7_CONFIG = 0x2, .GPIO8_CONFIG = 0x2, .GPIO9_CONFIG = 0x0 })).generic)
#define L9963E_GPIO3_THR_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_GPIO3_THRTypeDef){ .GPIO3_UT_TH = 0x0, .GPIO3_OT_TH = 0x0 })).generic)
#define L9963E_GPIO4_THR_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_GPIO4_THRTypeDef){ .GPIO4_UT_TH = 0x0, .GPIO4_OT_TH = 0x0 })).generic)
#define L9963E_GPIO5_THR_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_GPIO5_THRTypeDef){ .GPIO5_UT_TH = 0x0, .GPIO5_OT_TH = 0x0 })).generic)
#define L9963E_GPIO6_THR_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_GPIO6_THRTypeDef){ .GPIO6_UT_TH = 0x0, .GPIO6_OT_TH = 0x0 })).generic)
#define L9963E_GPIO7_THR_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_GPIO7_THRTypeDef){ .GPIO7_UT_TH = 0x0, .GPIO7_OT_TH = 0x0 })).generic)
#define L9963E_GPIO8_THR_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_GPIO8_THRTypeDef){ .GPIO8_UT_TH = 0x0, .GPIO8_OT_TH = 0x0 })).generic)
#define L9963E_GPIO9_THR_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_GPIO9_THRTypeDef){ .GPIO9_UT_TH = 0x0, .GPIO9_OT_TH = 0x0 })).generic)
#define L9963E_VCELLS_EN_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_VCELLS_ENTypeDef){ .VCELL1_EN = 0x0, .VCELL2_EN = 0x0, .VCELL3_EN = 0x0, .VCELL4_EN = 0x0, .VCELL5_EN = 0x0, .VCELL6_EN = 0x0, .VCELL7_EN = 0x0, .VCELL8_EN = 0x0, .VCELL9_EN = 0x0, .VCELL10_EN = 0x0, .VCELL11_EN = 0x0, .VCELL12_EN = 0x0, .VCELL13_EN = 0x0, .VCELL14_EN = 0x0, .Noreg14 = 0x0, .Noreg15 = 0x0, .Noreg16 = 0x0, .Noreg17 = 0x0 })).generic)
#define L9963E_FAULTMASK_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_FaultmaskTypeDef){ .VCELL1_BAL_UV_MSK = 0x0, .VCELL2_BAL_UV_MSK = 0x0, .VCELL3_BAL_UV_MSK = 0x0, .VCELL4_BAL_UV_MSK = 0x0, .VCELL5_BAL_UV_MSK = 0x0, .VCELL6_BAL_UV_MSK = 0x0, .VCELL7_BAL_UV_MSK = 0x0, .VCELL8_BAL_UV_MSK = 0x0, .VCELL9_BAL_UV_MSK = 0x0, .VCELL10_BAL_UV_MSK = 0x0, .VCELL11_BAL_UV_MSK = 0x0, .VCELL12_BAL_UV_MSK = 0x0, .VCELL13_BAL_UV_MSK = 0x0, .VCELL14_BAL_UV_MSK = 0x0, .Noreg14 = 0x0, .Noreg15 = 0x0, .Noreg16 = 0x0, .Noreg17 = 0x0 })).generic)
#define L9963E_FAULTMASK2_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_Faultmask2TypeDef){ .Gpio3_fastchg_OT_MSK = 0x0, .Gpio4_fastchg_OT_MSK = 0x0, .Gpio5_fastchg_OT_MSK = 0x0, .Gpio6_fastchg_OT_MSK = 0x0, .Gpio7_fastchg_OT_MSK = 0x0, .Gpio8_fastchg_OT_MSK = 0x0, .Gpio9_fastchg_OT_MSK = 0x0, .TrimmCalOk = 0x0, .trim_dwnl_tried = 0x0, .RAM_CRC_ERRMSK = 0x0, .RAM_CRC_ERR = 0x0, .EEPROM_CRC_ERRMSK_CAL_FF = 0x0, .EEPROM_CRC_ERR_CAL_FF = 0x0, .EEPROM_CRC_ERRMSK_CAL_RAM = 0x0, .EEPROM_CRC_ERR_CAL_RAM = 0x0, .EEPROM_CRC_ERRMSK_SECT_0 = 0x0, .EEPROM_CRC_ERR_SECT_0 = 0x0, .EEPROM_DWNLD_DONE = 0x0 })).generic)
#define L9963E_CSA_THRESH_NORM_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_CSA_THRESH_NORMTypeDef){ .adc_ovc_curr_threshold_norm = 0x0 })).generic)
#define L9963E_CSA_GPIO_MSK_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_CSA_GPIO_MSKTypeDef){ .Gpio3_OT_UT_MSK = 0x0, .Gpio4_OT_UT_MSK = 0x0, .Gpio5_OT_UT_MSK = 0x0, .Gpio6_OT_UT_MSK = 0x0, .Gpio7_OT_UT_MSK = 0x0, .Gpio8_OT_UT_MSK = 0x0, .Gpio9_OT_UT_MSK = 0x0, .Noreg7 = 0x0, .sense_minus_open = 0x0, .sense_plus_open = 0x0, .ovc_norm_msk = 0x0, .ovc_sleep_msk = 0x0, .CoulombCounter_en = 0x0, .adc_ovc_curr_threshold_sleep = 0x0 })).generic)
#define L9963E_VCELL1_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_Vcell1TypeDef){ .VCell1 = 0x0, .d_rdy_Vcell1 = 0x0, .Noreg17 = 0x0 })).generic)
#define L9963E_VCELL2_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_Vcell2TypeDef){ .VCell2 = 0x0, .d_rdy_Vcell2 = 0x0, .Noreg17 = 0x0 })).generic)
#define L9963E_VCELL3_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_Vcell3TypeDef){ .VCell3 = 0x0, .d_rdy_Vcell3 = 0x0, .Noreg17 = 0x0 })).generic)
#define L9963E_VCELL4_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_Vcell4TypeDef){ .VCell4 = 0x0, .d_rdy_Vcell4 = 0x0, .Noreg17 = 0x0 })).generic)
#define L9963E_VCELL5_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_Vcell5TypeDef){ .VCell5 = 0x0, .d_rdy_Vcell5 = 0x0, .Noreg17 = 0x0 })).generic)
#define L9963E_VCELL6_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_Vcell6TypeDef){ .VCell6 = 0x0, .d_rdy_Vcell6 = 0x0, .Noreg17 = 0x0 })).generic)
#define L9963E_VCELL7_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_Vcell7TypeDef){ .VCell7 = 0x0, .d_rdy_Vcell7 = 0x0, .Noreg17 = 0x0 })).generic)
#define L9963E_VCELL8_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_Vcell8TypeDef){ .VCell8 = 0x0, .d_rdy_Vcell8 = 0x0, .Noreg17 = 0x0 })).generic)
#define L9963E_VCELL9_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_Vcell9TypeDef){ .VCell9 = 0x0, .d_rdy_Vcell9 = 0x0, .Noreg17 = 0x0 })).generic)
#define L9963E_VCELL10_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_Vcell10TypeDef){ .VCell10 = 0x0, .d_rdy_Vcell10 = 0x0, .Noreg17 = 0x0 })).generic)
#define L9963E_VCELL11_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_Vcell11TypeDef){ .VCell11 = 0x0, .d_rdy_Vcell11 = 0x0, .Noreg17 = 0x0 })).generic)
#define L9963E_VCELL12_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_Vcell12TypeDef){ .VCell12 = 0x0, .d_rdy_Vcell12 = 0x0, .Noreg17 = 0x0 })).generic)
#define L9963E_VCELL13_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_Vcell13TypeDef){ .VCell13 = 0x0, .d_rdy_Vcell13 = 0x0, .Noreg17 = 0x0 })).generic)
#define L9963E_VCELL14_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_Vcell14TypeDef){ .VCell14 = 0x0, .d_rdy_Vcell14 = 0x0, .Noreg17 = 0x0 })).generic)
#define L9963E_IBATTERY_SYNCH_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_Ibattery_synchTypeDef){ .CUR_INST_Synch = 0x0 })).generic)
#define L9963E_IBATTERY_CALIB_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_Ibattery_calibTypeDef){ .CUR_INST_calib = X })).generic)
#define L9963E_COULCNTRTIME_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_CoulCntrTimeTypeDef){ .CoulombCntTime = 0x0, .Noreg16 = 0x0, .Noreg17 = 0x0 })).generic)
#define L9963E_COULCNTR_MSB_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_CoulCntr_msbTypeDef){ .CoulombCounter_msb = 0x0, .Noreg16 = 0x0, .Noreg17 = 0x0 })).generic)
#define L9963E_COULCNTR_LSB_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_CoulCntr_lsbTypeDef){ .CoulombCounter_lsb = 0x0, .Noreg16 = 0x0, .Noreg17 = 0x0 })).generic)
#define L9963E_GPIO3_MEAS_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_GPIO3_MEASTypeDef){ .GPIO3_MEAS = 0x0, .d_rdy_gpio3 = 0x0, .ratio_abs_3_sel = 0x0 })).generic)
#define L9963E_GPIO4_MEAS_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_GPIO4_MEASTypeDef){ .GPIO4_MEAS = 0x0, .d_rdy_gpio4 = 0x0, .ratio_abs_4_sel = 0x0 })).generic)
#define L9963E_GPIO5_MEAS_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_GPIO5_MEASTypeDef){ .GPIO5_MEAS = 0x0, .d_rdy_gpio5 = 0x0, .ratio_abs_5_sel = 0x0 })).generic)
#define L9963E_GPIO6_MEAS_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_GPIO6_MEASTypeDef){ .GPIO6_MEAS = 0x0, .d_rdy_gpio6 = 0x0, .ratio_abs_6_sel = 0x0 })).generic)
#define L9963E_GPIO7_MEAS_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_GPIO7_MEASTypeDef){ .GPIO7_MEAS = 0x0, .d_rdy_gpio7 = 0x0, .ratio_abs_7_sel = 0x0 })).generic)
#define L9963E_GPIO8_MEAS_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_GPIO8_MEASTypeDef){ .GPIO8_MEAS = 0x0, .d_rdy_gpio8 = 0x0, .ratio_abs_8_sel = 0x0 })).generic)
#define L9963E_GPIO9_MEAS_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_GPIO9_MEASTypeDef){ .GPIO9_MEAS = 0x0, .d_rdy_gpio9 = 0x0, .ratio_abs_9_sel = 0x0 })).generic)
#define L9963E_TEMPCHIP_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_TempChipTypeDef){ .TempChip = 0x0, .OTchip = 0x0, .Noreg9 = 0x0, .Noreg10 = 0x0, .Noreg11 = 0x0, .Noreg12 = 0x0, .Noreg13 = 0x0, .Noreg14 = 0x0, .Noreg15 = 0x0, .Noreg16 = 0x0, .Noreg17 = 0x0 })).generic)
#define L9963E_FAULTS1_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_Faults1TypeDef){ .Comm_timeout_flt = 0x0, .Noreg1 = 0x0, .Noreg2 = 0x0, .Fault_L_line_status = 0x0, .FaultHline_fault = 0x0, .HeartBeat_fault = 0x0, .VCOM_UV = 0x0, .VCOM_OV = 0x0, .VREG_OV = 0x0, .VREG_UV = 0x0, .VTREF_OV = 0x0, .VTREF_UV = 0x0, .VDIG_OV = 0x0, .VANA_OV = 0x0, .Noreg14 = 0x0, .Noreg15 = 0x0, .Noreg16 = 0x0, .Noreg17 = 0x0 })).generic)
#define L9963E_FAULTS2_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_Faults2TypeDef){ .curr_sense_ovc_norm = 0x0, .curr_sense_ovc_sleep = 0x0, .EoBtimeerror = 0x0, .CoCouOvF = 0x0, .Noreg4 = 0x0, .loss_gndref = 0x0, .loss_cgnd = 0x0, .loss_dgnd = 0x0, .loss_agnd = 0x0, .Noreg9 = 0x0, .OSCFail = 0x0, .Noreg11 = 0x0, .SPIENlatch = 0x0, .Noreg13 = 0x0, .Noreg14 = 0x0, .Noreg15 = 0x0, .Noreg16 = 0x0, .Noreg17 = 0x0 })).generic)
#define L9963E_BAL_OPEN_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_BAL_OPENTypeDef){ .Noreg0 = 0x0, .Noreg1 = 0x0, .BAL1_OPEN = 0x0, .BAL2_OPEN = 0x0, .BAL3_OPEN = 0x0, .BAL4_OPEN = 0x0, .BAL5_OPEN = 0x0, .BAL6_OPEN = 0x0, .BAL7_OPEN = 0x0, .BAL8_OPEN = 0x0, .BAL9_OPEN = 0x0, .BAL10_OPEN = 0x0, .BAL11_OPEN = 0x0, .BAL12_OPEN = 0x0, .BAL13_OPEN = 0x0, .BAL14_OPEN = 0x0, .Noreg16 = 0x0, .Noreg17 = 0x0 })).generic)
#define L9963E_BAL_SHORT_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_BAL_SHORTTypeDef){ .Noreg0 = 0x0, .Noreg1 = 0x0, .BAL1_SHORT = 0x0, .BAL2_SHORT = 0x0, .BAL3_SHORT = 0x0, .BAL4_SHORT = 0x0, .BAL5_SHORT = 0x0, .BAL6_SHORT = 0x0, .BAL7_SHORT = 0x0, .BAL8_SHORT = 0x0, .BAL9_SHORT = 0x0, .BAL10_SHORT = 0x0, .BAL11_SHORT = 0x0, .BAL12_SHORT = 0x0, .BAL13_SHORT = 0x0, .BAL14_SHORT = 0x0, .Noreg16 = 0x0, .Noreg17 = 0x0 })).generic)
#define L9963E_VSUMBATT_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_VSUMBATTTypeDef){ .vsum_batt19_2 = 0x0 })).generic)
#define L9963E_VBATTDIV_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_VBATTDIVTypeDef){ .VBATT_DIV = 0x0, .vsum_batt1_0 = 0x0 })).generic)
#define L9963E_CELL_OPEN_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_CELL_OPENTypeDef){ .CELL0_OPEN = 0x0, .CELL1_OPEN = 0x0, .CELL2_OPEN = 0x0, .CELL3_OPEN = 0x0, .CELL4_OPEN = 0x0, .CELL5_OPEN = 0x0, .CELL6_OPEN = 0x0, .CELL7_OPEN = 0x0, .CELL8_OPEN = 0x0, .CELL9_OPEN = 0x0, .CELL10_OPEN = 0x0, .CELL11_OPEN = 0x0, .CELL12_OPEN = 0x0, .CELL13_OPEN = 0x0, .CELL14_OPEN = 0x0, .Noreg15 = 0x0, .data_ready_vbattdiv = 0x0, .data_ready_vsum = 0x0 })).generic)
#define L9963E_VCELL_UV_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_VCELL_UVTypeDef){ .VCELL1_UV = 0x0, .VCELL2_UV = 0x0, .VCELL3_UV = 0x0, .VCELL4_UV = 0x0, .VCELL5_UV = 0x0, .VCELL6_UV = 0x0, .VCELL7_UV = 0x0, .VCELL8_UV = 0x0, .VCELL9_UV = 0x0, .VCELL10_UV = 0x0, .VCELL11_UV = 0x0, .VCELL12_UV = 0x0, .VCELL13_UV = 0x0, .VCELL14_UV = 0x0, .VSUM_UV = 0x0, .VBATTCRIT_UV = 0x0, .VBATT_WRN_UV = 0x0, .Noreg17 = 0x0 })).generic)
#define L9963E_VCELL_OV_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_VCELL_OVTypeDef){ .VCELL1_OV = 0x0, .VCELL2_OV = 0x0, .VCELL3_OV = 0x0, .VCELL4_OV = 0x0, .VCELL5_OV = 0x0, .VCELL6_OV = 0x0, .VCELL7_OV = 0x0, .VCELL8_OV = 0x0, .VCELL9_OV = 0x0, .VCELL10_OV = 0x0, .VCELL11_OV = 0x0, .VCELL12_OV = 0x0, .VCELL13_OV = 0x0, .VCELL14_OV = 0x0, .VSUM_OV = 0x0, .VBATTCRIT_OV = 0x0, .VBATT_WRN_OV = 0x0, .Noreg17 = 0x0 })).generic)
#define L9963E_VGPIO_OT_UT_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_VGPIO_OT_UTTypeDef){ .GPIO3_UT = 0x0, .GPIO4_UT = 0x0, .GPIO5_UT = 0x0, .GPIO6_UT = 0x0, .GPIO7_UT = 0x0, .GPIO8_UT = 0x0, .GPIO9_UT = 0x0, .GPIO3_OT = 0x0, .GPIO4_OT = 0x0, .GPIO5_OT = 0x0, .GPIO6_OT = 0x0, .GPIO7_OT = 0x0, .GPIO8_OT = 0x0, .GPIO9_OT = 0x0, .Noreg14 = 0x0, .Noreg15 = 0x0, .Noreg16 = 0x0, .Noreg17 = 0x0 })).generic)
#define L9963E_VCELL_BAL_UV_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_VCELL_BAL_UVTypeDef){ .VCELL1_BAL_UV = 0x0, .VCELL2_BAL_UV = 0x0, .VCELL3_BAL_UV = 0x0, .VCELL4_BAL_UV = 0x0, .VCELL5_BAL_UV = 0x0, .VCELL6_BAL_UV = 0x0, .VCELL7_BAL_UV = 0x0, .VCELL8_BAL_UV = 0x0, .VCELL9_BAL_UV = 0x0, .VCELL10_BAL_UV = 0x0, .VCELL11_BAL_UV = 0x0, .VCELL12_BAL_UV = 0x0, .VCELL13_BAL_UV = 0x0, .VCELL14_BAL_UV = 0x0, .Noreg14 = 0x0, .Noreg15 = 0x0, .Noreg16 = 0x0, .Noreg17 = 0x0 })).generic)
#define L9963E_GPIO_FASTCHG_OT_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_GPIO_fastchg_OTTypeDef){ .GPIO3_fastchg_OT = 0x0, .GPIO4_fastchg_OT = 0x0, .GPIO5_fastchg_OT = 0x0, .GPIO6_fastchg_OT = 0x0, .GPIO7_fastchg_OT = 0x0, .GPIO8_fastchg_OT = 0x0, .GPIO9_fastchg_OT = 0x0, .GPIO3_OPEN = 0x0, .GPIO4_OPEN = 0x0, .GPIO5_OPEN = 0x0, .GPIO6_OPEN = 0x0, .GPIO7_OPEN = 0x0, .GPIO8_OPEN = 0x0, .GPIO9_OPEN = 0x0, .Noreg14 = 0x0, .Noreg15 = 0x0, .Noreg16 = 0x0, .Noreg17 = 0x0 })).generic)
#define L9963E_MUX_BIST_FAIL_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_MUX_BIST_FAILTypeDef){ .MUX_BIST_FAIL = 0x0, .HWSC_DONE = 0x0, .Noreg15 = 0x0, .Noreg16 = 0x0, .Noreg17 = 0x0 })).generic)
#define L9963E_BIST_COMP_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_BIST_COMPTypeDef){ .BIST_BAL_COMP_LS_FAIL = 0x0, .BIST_BAL_COMP_HS_FAIL = 0x0, .VTREF_COMP_BIST_FAIL = 0x0, .VCOM_COMP_BIST_FAIL = 0x0, .VREG_COMP_BIST_FAIL = 0x0, .VBAT_COMP_BIST_FAIL = 0x0 })).generic)
#define L9963E_OPEN_BIST_FAIL_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_OPEN_BIST_FAILTypeDef){ .OPEN_BIST_FAIL = 0x0, .Noreg14 = 0x0, .Noreg15 = 0x0, .Noreg16 = 0x0, .Noreg17 = 0x0 })).generic)
#define L9963E_GPIO_BIST_FAIL_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_GPIO_BIST_FAILTypeDef){ .GPIO_BIST_FAIL = 0x0, .VTREF_BIST_FAIL = 0x0, .Noreg8 = 0x0, .Noreg9 = 0x0, .Noreg10 = 0x0, .GPO3short = 0x0, .GPO4short = 0x0, .GPO5short = 0x0, .GPO6short = 0x0, .GPO7short = 0x0, .GPO8short = 0x0, .GPO9short = 0x0 })).generic)
#define L9963E_VTREF_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_VTREFTypeDef){ .VTREF_MEAS = 0x0, .d_rdy_vtref = 0x0, .Noreg17 = 0x0 })).generic)
#define L9963E_NVM_WR_1_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_NVM_WR_1TypeDef){ .NVM_WR_15_0 = 0x0, .Noreg16 = 0x0, .Noreg17 = 0x0 })).generic)
#define L9963E_NVM_WR_2_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_NVM_WR_2TypeDef){ .NVM_WR_31_16 = 0x0, .Noreg16 = 0x0, .Noreg17 = 0x0 })).generic)
#define L9963E_NVM_WR_3_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_NVM_WR_3TypeDef){ .NVM_WR_47_32 = 0x0, .Noreg16 = 0x0, .Noreg17 = 0x0 })).generic)
#define L9963E_NVM_WR_4_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_NVM_WR_4TypeDef){ .NVM_WR_63_48 = 0x0, .Noreg16 = 0x0, .Noreg17 = 0x0 })).generic)
#define L9963E_NVM_WR_5_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_NVM_WR_5TypeDef){ .NVM_WR_79_64 = 0x0, .Noreg16 = 0x0, .Noreg17 = 0x0 })).generic)
#define L9963E_NVM_WR_6_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_NVM_WR_6TypeDef){ .NVM_WR_95_80 = 0x0, .Noreg16 = 0x0, .Noreg17 = 0x0 })).generic)
#define L9963E_NVM_WR_7_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_NVM_WR_7TypeDef){ .NVM_WR_111_96 = 0x0, .Noreg16 = 0x0, .Noreg17 = 0x0 })).generic)
#define L9963E_NVM_RD_1_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_NVM_RD_1TypeDef){ .NVM_RD_15_0 = 0x0, .Noreg16 = 0x0, .Noreg17 = 0x0 })).generic)
#define L9963E_NVM_RD_2_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_NVM_RD_2TypeDef){ .NVM_RD_31_16 = 0x0, .Noreg16 = 0x0, .Noreg17 = 0x0 })).generic)
#define L9963E_NVM_RD_3_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_NVM_RD_3TypeDef){ .NVM_RD_47_32 = 0x0, .Noreg16 = 0x0, .Noreg17 = 0x0 })).generic)
#define L9963E_NVM_RD_4_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_NVM_RD_4TypeDef){ .NVM_RD_63_48 = 0x0, .Noreg16 = 0x0, .Noreg17 = 0x0 })).generic)
#define L9963E_NVM_RD_5_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_NVM_RD_5TypeDef){ .NVM_RD_79_64 = 0x0, .Noreg16 = 0x0, .Noreg17 = 0x0 })).generic)
#define L9963E_NVM_RD_6_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_NVM_RD_6TypeDef){ .NVM_RD_95_80 = 0x0, .Noreg16 = 0x0, .Noreg17 = 0x0 })).generic)
#define L9963E_NVM_RD_7_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_NVM_RD_7TypeDef){ .NVM_RD_111_96 = 0x0, .Noreg16 = 0x0, .Noreg17 = 0x0 })).generic)
#define L9963E_NVM_CMD_CNTR_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_NVM_CMD_CNTRTypeDef){ .NVM_CNTR = 0x0, .NVM_PROGRAM = 0x0, .NVM_OPER = 0x0, .NVM_WR_BUSY = 0x0, .Noreg12 = 0x0, .Noreg13 = 0x0, .Noreg14 = 0x0, .Noreg15 = 0x0, .Noreg16 = 0x0, .Noreg17 = 0x0 })).generic)
#define L9963E_NVM_UNLCK_PRG_DEFAULT (((L9963E_RegisterUnionTypeDef)((L9963E_NVM_UNLCK_PRGTypeDef){ .NVM_UNLOCK_START = 0x0 })).generic)

// clang-format on