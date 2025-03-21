
#include <zephyr/logging/log.h>
#include <stdlib.h>

#ifdef CONFIG_SHELL_TMC_PARAMS
#include <zephyr/shell/shell.h>
#endif /* CONFIG_SHELL_TMC_PARAMS */

#include "tmc9660_params.h"

LOG_MODULE_REGISTER(tmc9660_params, LOG_LEVEL_INF);

#define PARAM_ID_LIST_ENTRY(PARAM) PARAM, // Use enum value from tmc9660_params.h
#define PARAM_NAME_WITH_NULL(PARAM) #PARAM "\0"

static uint16_t tmc9660_rwe_param_ids[] = {
    FOREACH_TMC9660_RWE_PARAM(PARAM_ID_LIST_ENTRY)
};
static size_t tmc9660_rwe_param_count = ARRAY_SIZE(tmc9660_rwe_param_ids);

#ifdef CONFIG_TMC_RWE_PARAM_NAMES
static char tmc9660_rwe_param_names[] = FOREACH_TMC9660_RWE_PARAM(PARAM_NAME_WITH_NULL);
#endif /* CONFIG_TMC_RWE_PARAM_NAMES */


// TODO: page-aligned link inside flash!
// __attribute__((section(".flash_tmc9660_params")))
static int32_t tmc9660_rwe_param_values[ARRAY_SIZE(tmc9660_rwe_param_ids)] = {
    // "Out-of-the-box" tuning parameters, based on a QSH5718 stepper motor
    // TODO: User-facing tuning procedure which overwrites these in flash
	2, // MOTOR_TYPE
	50, // MOTOR_POLE_PAIRS
	0, // MOTOR_DIRECTION
	25000, // MOTOR_PWM_FREQUENCY
	20000, // OUTPUT_VOLTAGE_LIMIT
	1999, // MAX_TORQUE
	1999, // MAX_FLUX
	1, // PWM_SWITCHING_SCHEME
	1, // IDLE_MOTOR_PWM_BEHAVIOR
	4, // ADC_SHUNT_TYPE
	1, // CSA_GAIN_ADC_I0_TO_ADC_I2
	1, // CSA_GAIN_ADC_I3
	0, // CSA_FILTER_ADC_I0_TO_ADC_I2
	0, // CSA_FILTER_ADC_I3
	390, // CURRENT_SCALING_FACTOR
	0, // PHASE_UX1_ADC_MAPPING
	1, // PHASE_VX2_ADC_MAPPING
	2, // PHASE_WY1_ADC_MAPPING
	3, // PHASE_Y2_ADC_MAPPING
	1024, // ADC_I0_SCALE
	1024, // ADC_I1_SCALE
	1024, // ADC_I2_SCALE
	1024, // ADC_I3_SCALE
	1, // ADC_I0_INVERTED
	0, // ADC_I1_INVERTED
	1, // ADC_I2_INVERTED
	0, // ADC_I3_INVERTED
	-1, // ADC_I0_OFFSET
	11, // ADC_I1_OFFSET
	-26, // ADC_I2_OFFSET
	-12, // ADC_I3_OFFSET
	500, // OPENLOOP_CURRENT
	1638, // OPENLOOP_VOLTAGE
	8, // ACCELERATION_FF_GAIN
	4, // ACCELERATION_FF_SHIFT
	1, // RAMP_ENABLE
	1, // DIRECT_VELOCITY_MODE
	300000, // RAMP_AMAX
	8000, // RAMP_A1
	4000, // RAMP_A2
	300000, // RAMP_DMAX
	8000, // RAMP_D1
	8000, // RAMP_D2
	6000000, // RAMP_VMAX
	0, // RAMP_V1
	0, // RAMP_V2
	0, // RAMP_VSTART
	1, // RAMP_VSTOP
	0, // RAMP_TVMAX
	0, // RAMP_TZEROWAIT
	0, // ACCELERATION_FEEDFORWARD_ENABLE
	0, // VELOCITY_FEEDFORWARD_ENABLE
	0, // HALL_SECTOR_OFFSET
	0, // HALL_FILTER_LENGTH
	0, // HALL_POSITION_0_OFFSET
	10922, // HALL_POSITION_60_OFFSET
	21845, // HALL_POSITION_120_OFFSET
	-32768, // HALL_POSITION_180_OFFSET
	-21846, // HALL_POSITION_240_OFFSET
	-10923, // HALL_POSITION_300_OFFSET
	0, // HALL_INVERT_DIRECTION
	0, // HALL_EXTRAPOLATION_ENABLE
	0, // HALL_PHI_E_OFFSET
	40000, // ABN_1_STEPS
	0, // ABN_1_DIRECTION
	0, // ABN_1_INIT_METHOD
	1000, // ABN_1_INIT_DELAY
	5, // ABN_1_INIT_VELOCITY
	0, // ABN_1_N_CHANNEL_PHI_E_OFFSET
	0, // ABN_1_N_CHANNEL_INVERTED
	0, // ABN_1_N_CHANNEL_FILTERING
	3933, // TORQUE_P
	19526, // TORQUE_I
	3933, // FLUX_P
	19526, // FLUX_I
	0, // SEPARATE_TORQUE_FLUX_PI_PARAMTERS
	0, // CURRENT_NORM_P
	1, // CURRENT_NORM_I
	0, // VELOCITY_SENSOR_SELECTION
	26218, // VELOCITY_P
	100, // VELOCITY_I
	2, // VELOCITY_NORM_P
	1, // VELOCITY_NORM_I
	1, // VELOCITY_SCALING_FACTOR
	5, // VELOCITY_LOOP_DOWNSAMPLING
	0, // VELOCITY_REACHED_THRESHOLD
	2000, // VELOCITY_METER_SWITCH_THRESHOLD
	500, // VELOCITY_METER_SWITCH_HYSTERESIS
	0, // POSITION_SENSOR_SELECTION
	1024, // POSITION_SCALING_FACTOR
	30, // POSITION_P
	0, // POSITION_I
	0, // POSITION_NORM_P
	1, // POSITION_NORM_I
	0, // STOP_ON_POSITION_DEVIATION
	0, // POSITION_LOOP_DOWNSAMPLING
	-2147483648, // POSITION_LIMIT_LOW
	2147483647, // POSITION_LIMIT_HIGH
	0, // POSITION_REACHED_THRESHOLD
	0, // REFERENCE_SWITCH_ENABLE
	0, // REFERENCE_SWITCH_POLARITY_AND_SWAP
	0, // REFERENCE_SWITCH_LATCH_SETTINGS
	0, // EVENT_STOP_SETTINGS
	0, // REFERENCE_SWITCH_SEARCH_MODE
	0, // REFERENCE_SWITCH_SEARCH_SPEED
	0, // REFERENCE_SWITCH_SPEED
	0, // ABN_2_STEPS
	0, // ABN_2_DIRECTION
	0, // ABN_2_GEAR_RATIO
	0, // ABN_2_ENABLE
	0, // SPI_ENCODE_CS_SETTLE_DELAY_TIME
	0, // SPI_ENCODER_CS_IDLE_DELAY_TIME
	0, // SPI_ENCODER_MAIN_TRANSFER_CMD_SIZE
	0, // SPI_ENCODER_SECONDARY_TRANSFER_CMD_SIZE
	0, // SPI_ENCODER_TRANSFER_DATA_3_0
	0, // SPI_ENCODER_TRANSFER_DATA_7_4
	0, // SPI_ENCODER_TRANSFER_DATA_11_8
	0, // SPI_ENCODER_TRANSFER_DATA_15_12
	0, // SPI_ENCODER_TRANSFER
	0, // SPI_ENCODER_POSITION_COUNTER_MASK
	0, // SPI_ENCODER_POSITION_COUNTER_SHIFT
	0, // SPI_ENCODER_INITIALIZATION_METHOD
	0, // SPI_ENCODER_DIRECTION
	0, // SPI_ENCODER_OFFSET
	0, // SPI_LUT_CORRECTION_ENABLE
	0, // STEP_DIR_STEP_DIVIDER_SHIFT
	0, // BRAKE_CHOPPER_ENABLE
	260, // BRAKE_CHOPPER_VOLTAGE_LIMIT
	5, // BRAKE_CHOPPER_HYSTERESIS
	0, // RELEASE_BRAKE
	75, // BRAKE_RELEASING_DUTY_CYCLE
	11, // BRAKE_HOLDING_DUTY_CYCLE
	80, // BRAKE_RELEASING_DURATION
	0, // INVERT_BRAKE_OUTPUT
	3000, // THERMAL_WINDING_TIME_CONSTANT_1
	-1, // IIT_LIMIT_1
	6000, // THERMAL_WINDING_TIME_CONSTANT_2
	-1, // IIT_LIMIT_2
	0, // PWM_L_OUTPUT_POLARITY
	0, // PWM_H_OUTPUT_POLARITY
	0, // BREAK_BEFORE_MAKE_TIME_LOW_UVW
	0, // BREAK_BEFORE_MAKE_TIME_HIGH_UVW
	0, // BREAK_BEFORE_MAKE_TIME_LOW_Y2
	0, // BREAK_BEFORE_MAKE_TIME_HIGH_Y2
	1, // USE_ADAPTIVE_DRIVE_TIME_UVW
	1, // USE_ADAPTIVE_DRIVE_TIME_Y2
	40, // DRIVE_TIME_SINK_UVW
	40, // DRIVE_TIME_SOURCE_UVW
	40, // DRIVE_TIME_SINK_Y2
	40, // DRIVE_TIME_SOURCE_Y2
	10, // UVW_SINK_CURRENT
	5, // UVW_SOURCE_CURRENT
	10, // Y2_SINK_CURRENT
	5, // Y2_SOURCE_CURRENT
	7, // BOOTSTRAP_CURRENT_LIMIT

	0, // UNDERVOLTAGE_PROTECTION_SUPPLY_LEVEL
	1, // UNDERVOLTAGE_PROTECTION_VDRV_ENABLE
	1, // UNDERVOLTAGE_PROTECTION_BST_UVW_ENABLE
	1, // UNDERVOLTAGE_PROTECTION_BST_Y2_ENABLE
	1, // OVERCURRENT_PROTECTION_UVW_LOW_SIDE_ENABLE
	1, // OVERCURRENT_PROTECTION_UVW_HIGH_SIDE_ENABLE
	1, // OVERCURRENT_PROTECTION_Y2_LOW_SIDE_ENABLE
	1, // OVERCURRENT_PROTECTION_Y2_HIGH_SIDE_ENABLE
	1, // OVERCURRENT_PROTECTION_UVW_LOW_SIDE_THRESHOLD
	2, // OVERCURRENT_PROTECTION_UVW_HIGH_SIDE_THRESHOLD
	1, // OVERCURRENT_PROTECTION_Y2_LOW_SIDE_THRESHOLD
	2, // OVERCURRENT_PROTECTION_Y2_HIGH_SIDE_THRESHOLD
	2, // OVERCURRENT_PROTECTION_UVW_LOW_SIDE_BLANKING
	2, // OVERCURRENT_PROTECTION_UVW_HIGH_SIDE_BLANKING
	2, // OVERCURRENT_PROTECTION_Y2_LOW_SIDE_BLANKING
	2, // OVERCURRENT_PROTECTION_Y2_HIGH_SIDE_BLANKING
	6, // OVERCURRENT_PROTECTION_UVW_LOW_SIDE_DEGLITCH
	6, // OVERCURRENT_PROTECTION_UVW_HIGH_SIDE_DEGLITCH
	6, // OVERCURRENT_PROTECTION_Y2_LOW_SIDE_DEGLITCH
	6, // OVERCURRENT_PROTECTION_Y2_HIGH_SIDE_DEGLITCH
	0, // OVERCURRENT_PROTECTION_UVW_LOW_SIDE_USE_VDS
	0, // OVERCURRENT_PROTECTION_Y2_LOW_SIDE_USE_VDS
	1, // VGS_SHORT_ON_PROTECTION_UVW_LOW_SIDE_ENABLE
	1, // VGS_SHORT_OFF_PROTECTION_UVW_LOW_SIDE_ENABLE
	1, // VGS_SHORT_ON_PROTECTION_UVW_HIGH_SIDE_ENABLE
	1, // VGS_SHORT_OFF_PROTECTION_UVW_HIGH_SIDE_ENABLE
	1, // VGS_SHORT_ON_PROTECTION_Y2_LOW_SIDE_ENABLE
	1, // VGS_SHORT_OFF_PROTECTION_Y2_LOW_SIDE_ENABLE
	1, // VGS_SHORT_ON_PROTECTION_Y2_HIGH_SIDE_ENABLE
	1, // VGS_SHORT_OFF_PROTECTION_Y2_HIGH_SIDE_ENABLE
	1, // VGS_SHORT_PROTECTION_UVW_BLANKING
	1, // VGS_SHORT_PROTECTION_Y2_BLANKING
	1, // VGS_SHORT_PROTECTION_UVW_DEGLITCH
	1, // VGS_SHORT_PROTECTION_Y2_DEGLITCH
	0, // GDRV_RETRY_BEHAVIOUR
	0, // DRIVE_FAULT_BEHAVIOUR
	5, // FAULT_HANDLER_NUMBER_OF_RETRIES
	
    710, // SUPPLY_OVERVOLTAGE_WARNING_THRESHOLD
	78, // SUPPLY_UNDERVOLTAGE_WARNING_THRESHOLD
	65535, // EXTERNAL_TEMPERATURE_SHUTDOWN_THRESHOLD
	65535, // EXTERNAL_TEMPERATURE_WARNING_THRESHOLD
	65535, // CHIP_TEMPERATURE_SHUTDOWN_THRESHOLD
	65535, // CHIP_TEMPERATURE_WARNING_THRESHOLD
	
    0, // FIELDWEAKENING_I
	32767, // FIELDWEAKENING_VOLTAGE_THRESHOLD

	0, // TARGET_TORQUE_BIQUAD_FILTER_ENABLE
	0, // TARGET_TORQUE_BIQUAD_FILTER_ACOEFF_1
	0, // TARGET_TORQUE_BIQUAD_FILTER_ACOEFF_2
	1048576, // TARGET_TORQUE_BIQUAD_FILTER_BCOEFF_0
	0, // TARGET_TORQUE_BIQUAD_FILTER_BCOEFF_1
	0, // TARGET_TORQUE_BIQUAD_FILTER_BCOEFF_2

    // FIXME: stupid lowpass filter, need to tune a proper one
	1, // ACTUAL_VELOCITY_BIQUAD_FILTER_ENABLE
	1038090, // ACTUAL_VELOCITY_BIQUAD_FILTER_ACOEFF_1
	0, // ACTUAL_VELOCITY_BIQUAD_FILTER_ACOEFF_2
	3491, // ACTUAL_VELOCITY_BIQUAD_FILTER_BCOEFF_0
	3491, // ACTUAL_VELOCITY_BIQUAD_FILTER_BCOEFF_1
	3491, // ACTUAL_VELOCITY_BIQUAD_FILTER_BCOEFF_2
};

int tmc_params_save(struct tmc9660_dev *dev) {
    int ret = 0;
    for(int i = 0; i < tmc9660_rwe_param_count; i++) {
        if(tmc9660_get_param(dev, tmc9660_rwe_param_ids[i], &tmc9660_rwe_param_values[i]))
            ret = -1;
    }

    // TODO: commit to flash!

    return ret;
}

int tmc_params_load(struct tmc9660_dev *dev) {
    int ret = 0;
    #ifdef CONFIG_TMC_RWE_PARAM_NAMES
    char *name_ptr = tmc9660_rwe_param_names;
    #endif
    for(int i = 0; i < tmc9660_rwe_param_count; i++) {
        int ret = tmc9660_set_param_retry(dev, tmc9660_rwe_param_ids[i], tmc9660_rwe_param_values[i], 3);

        if(ret) {
#ifdef CONFIG_TMC_RWE_PARAM_NAMES
            LOG_ERR("Could not set parameter %s = %d", name_ptr, tmc9660_rwe_param_values[i]);
#else
            LOG_ERR("Could not set parameter %d = %d", tmc9660_rwe_param_ids[i], tmc9660_rwe_param_values[i]);
#endif      
            ret = -1;
        }

#ifdef CONFIG_TMC_RWE_PARAM_NAMES
        while(*name_ptr++); // Advance to next string
#endif
    }

    return ret;
}

#ifdef CONFIG_SHELL_TMC_PARAMS

extern struct tmc9660_dev g_tmc9660;

int cmd_tmc_sap(struct shell *sh, int argc, char **argv) {
    int param_id = atoi(argv[1]);
    int param_val = atoi(argv[2]);
    return tmc9660_set_param(&g_tmc9660, param_id, param_val);
}

int cmd_tmc_gap(struct shell *sh, int argc, char **argv) {
    int param_id = atoi(argv[1]);
    int val = 0;
    int ret = tmc9660_get_param(&g_tmc9660, param_id, &val);
    if(ret == 0)
        shell_print(sh, "%d", val);
    else
        shell_error(sh, "Error %d", ret);
    return ret;
}

int cmd_tmc_rwe_show(struct shell *sh, int argc, char **argv) {
    #ifdef CONFIG_TMC_RWE_PARAM_NAMES
    char *name_ptr = tmc9660_rwe_param_names;
    #endif
    for(int i = 0; i < tmc9660_rwe_param_count; i++) {
        int param_id = tmc9660_rwe_param_ids[i];
        int flash_value = tmc9660_rwe_param_values[i];
        int tmc_value = 0xcafebabe;
        int ret = tmc9660_get_param(&g_tmc9660, param_id, &tmc_value);

        if(ret) {
#ifdef CONFIG_TMC_RWE_PARAM_NAMES
            shell_warn(sh, "Error %d reading %s", ret, name_ptr);
#else
            shell_warn(sh, "Error %d reading %d", ret, param_id);
#endif
        }

        if(flash_value == tmc_value) {
#ifdef CONFIG_TMC_RWE_PARAM_NAMES
            shell_print(sh, "%s = %d", name_ptr, tmc_value);
#else
            shell_print(sh, "%d = %d", param_id, tmc_value);
#endif
        } else {
#ifdef CONFIG_TMC_RWE_PARAM_NAMES
            shell_warn(sh, "%s = %d (saved: %d)", name_ptr, tmc_value, flash_value);
#else
            shell_warn(sh, "%d = %d (saved: %d)", param_id, tmc_value, flash_value);
#endif
        }

#ifdef CONFIG_TMC_RWE_PARAM_NAMES
        while(*name_ptr++); // Advance to next string
#endif
    }

    return 0;
}

int cmd_tmc_rwe_load(struct shell *sh, int argc, char **argv) {
    tmc_params_load(&g_tmc9660);
    cmd_tmc_rwe_show(sh, argc, argv);
    return 0;
}

int cmd_tmc_rwe_save(struct shell *sh, int argc, char **argv) {
    tmc_params_save(&g_tmc9660);
    cmd_tmc_rwe_show(sh, argc, argv);
    return 0;
}

int cmd_tmc_flags(struct shell *sh, int argc, char **argv) {
	int flags = 0;
	tmc9660_get_param(&g_tmc9660, GENERAL_STATUS_FLAGS, &flags);

	if(flags & 1) shell_print(sh, "REGULATION_STOPPED");
	if(flags & 2) shell_print(sh, "REGULATION_TORQUE");
	if(flags & 4) shell_print(sh, "REGULATION_VELOCITY");
	if(flags & 8) shell_print(sh, "REGULATION_POSITION");
	if(flags & 16) shell_print(sh, "CONFIG_STORED");
	if(flags & 32) shell_print(sh, "CONFIG_LOADED");
	if(flags & 64) shell_print(sh, "CONFIG_READ_ONLY");
	if(flags & 128) shell_print(sh, "TMCL_SCRIPT_READ_ONLY");
	if(flags & 256) shell_print(sh, "BRAKE_CHOPPER_ACTIVE");
	if(flags & 512) shell_print(sh, "POSITION_REACHED");
	if(flags & 1024) shell_print(sh, "VELOCITY_REACHED");
	if(flags & 2048) shell_print(sh, "ADC_OFFSET_CALIBRATED");
	if(flags & 4096) shell_print(sh, "RAMPER_LATCHED");
	if(flags & 8192) shell_print(sh, "RAMPER_EVENT_STOP_SWITCH");
	if(flags & 16384) shell_print(sh, "RAMPER_EVENT_STOP_DEVIATION");
	if(flags & 32768) shell_print(sh, "RAMPER_VELOCITY_REACHED");
	if(flags & 65536) shell_print(sh, "RAMPER_POSITION_REACHED");
	if(flags & 131072) shell_print(sh, "RAMPER_SECOND_MOVE");
	if(flags & 262144) shell_print(sh, "IIT_1_ACTIVE");
	if(flags & 524288) shell_print(sh, "IIT_2_ACTIVE");
	if(flags & 1048576) shell_print(sh, "REFSEARCH_FINISHED");
	if(flags & 2097152) shell_print(sh, "Y2_USED_FOR_BRAKING");
	if(flags & 4194304) shell_print(sh, "FLASH_STIMULUS_AVAILABLE");
	if(flags & 8388608) shell_print(sh, "STEPDIR_INPUT_AVAILABLE");
	if(flags & 16777216) shell_print(sh, "RIGHT_REF_SWITCH_AVAILABLE");
	if(flags & 33554432) shell_print(sh, "HOME_REF_SWITCH_AVAILABLE");
	if(flags & 67108864) shell_print(sh, "LEFT_REF_SWITCH_AVAILABLE");
	if(flags & 134217728) shell_print(sh, "ABN2_FEEDBACK_AVAILABLE");
	if(flags & 268435456) shell_print(sh, "HALL_FEEDBACK_AVAILABLE");
	if(flags & 536870912) shell_print(sh, "ABN1_FEEDBACK_AVAILABLE");
	if(flags & 1073741824) shell_print(sh, "SPI_FLASH_AVAILABLE");
	if(flags & 2147483648) shell_print(sh, "I2C_EEPROM_AVAILABLE");
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_tmc,
    SHELL_CMD_ARG(gap, NULL, "Get parameter", cmd_tmc_gap, 2, 0),
    SHELL_CMD_ARG(sap, NULL, "Set parameter", cmd_tmc_sap, 3, 0),
    SHELL_CMD_ARG(flags, NULL, "Show current flags", cmd_tmc_flags, 0, 0),
    SHELL_CMD_ARG(rwe_show, NULL, "Show RWE parameters", cmd_tmc_rwe_show, 0, 1),
    SHELL_CMD(rwe_save, NULL, "Save all current parameters to flash (run this after tuning)", cmd_tmc_rwe_save),
    SHELL_CMD(rwe_load, NULL, "Load all RWE parameters from flash", cmd_tmc_rwe_load),
    SHELL_SUBCMD_SET_END
);
SHELL_CMD_REGISTER(tmc, &sub_tmc, "TMC9660 commands", NULL);

#endif /* CONFIG_SHELL_TMC_PARAMS */
