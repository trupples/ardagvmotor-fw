#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/i2c.h>

#include "tmc9660.h"

#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

static const struct gpio_dt_spec led_green = GPIO_DT_SPEC_GET(DT_PATH(leds, led_status), gpios);
static const struct gpio_dt_spec led_fault = GPIO_DT_SPEC_GET(DT_PATH(buttons, fault), gpios);
static const struct spi_dt_spec spi0 = SPI_DT_SPEC_GET(DT_NODELABEL(tmc9660_spi), SPI_MODE_CPHA | SPI_MODE_CPOL | SPI_WORD_SET(8), 0);
static const struct device *i2cbus = DEVICE_DT_GET(DT_NODELABEL(i2c0));

struct tmc9660_dev tmc9660;

#define QSH6018 1
#define QSH5718 2
#define QSH4218 3
#define DC_BASIC 4
#define MOTOR DC_BASIC

#pragma pack(push,1) // ew alignment...
const struct tmc9660_default_params {
    uint16_t param_id;
    uint32_t value;
} params[] = {
	// General motor setup
#if MOTOR == QSH6018 || MOTOR == QSH5718 || MOTOR == QSH4218
	{ .param_id = MOTOR_TYPE, .value = 2 },
	{ .param_id = MOTOR_PWM_FREQUENCY, .value = 25000 },
	{ .param_id = MOTOR_POLE_PAIRS, .value = 50 },
	{ .param_id = ABN_1_STEPS, .value = 40000 },
#elif MOTOR == DC_BASIC
    { .param_id = MOTOR_TYPE, .value = 1 },
    { .param_id = MOTOR_PWM_FREQUENCY, .value = 25000 },
    { .param_id = MOTOR_POLE_PAIRS, .value = 1 },
	{ .param_id = OPENLOOP_CURRENT, .value = 5000 },
	{ .param_id = OPENLOOP_VOLTAGE, .value = 16383 },
    { .param_id = OUTPUT_VOLTAGE_LIMIT, .value = 20000 }, // proportional to VIN, 16383 = 100%, goes up to 200%
    
	{ .param_id = TORQUE_P, .value = 2500 },
	{ .param_id = TORQUE_I, .value = 0 },
	{ .param_id = FLUX_P, .value = 2500 },
	{ .param_id = FLUX_I, .value = 0 },
	{ .param_id = CURRENT_NORM_P, .value = 0 },
	{ .param_id = CURRENT_NORM_I, .value = 0 },
    
	{ .param_id = MAX_TORQUE, .value = 10000 }, // mA
	{ .param_id = MAX_FLUX, .value = 10000 }, // mA

	{ .param_id = MOTOR_DIRECTION, .value = 1 },
#else
#error Unsupported motor!
#endif

#if MOTOR == QSH6018
	{ .param_id = RAMP_VMAX, .value = 4200000 }, // Motor struggles to get to 4.2M internal velocity units, by construction. This limits the internal target so we don't get integral windup.
	{ .param_id = MAX_TORQUE, .value = 4000 }, // mA
	{ .param_id = MAX_FLUX, .value = 4000 }, // mA
	{ .param_id = OPENLOOP_CURRENT, .value = 500 }, // 500mA openloop (such as when initially homing ABN)
	{ .param_id = OPENLOOP_VOLTAGE, .value = 1638 }, // proportional to VIN, 1638 = 10%
#elif MOTOR == QSH5718
	{ .param_id = RAMP_VMAX, .value = 4100000 },
	{ .param_id = MAX_TORQUE, .value = 10000 }, // mA
	{ .param_id = MAX_FLUX, .value = 10000 }, // mA
	{ .param_id = OPENLOOP_CURRENT, .value = 500 }, // 500mA openloop (such as when initially homing ABN)
	{ .param_id = OPENLOOP_VOLTAGE, .value = 1638 }, // proportional to VIN, 1638 = 10%
#elif MOTOR == QSH4218
	{ .param_id = RAMP_VMAX, .value = 6000000 },
	{ .param_id = MAX_TORQUE, .value = 2000 }, // mA
	{ .param_id = MAX_FLUX, .value = 2000 }, // mA
	{ .param_id = OPENLOOP_CURRENT, .value = 500 }, // 500mA openloop (such as when initially homing ABN)
	{ .param_id = OPENLOOP_VOLTAGE, .value = 1638 }, // proportional to VIN, 1638 = 10%
#endif

	// General limits
	{ .param_id = OUTPUT_VOLTAGE_LIMIT, .value = 20000 }, // proportional to VIN, 16383 = 100%, goes up to 200%

	// ADC Setup - depends on board, not motor!
	{ .param_id = ADC_SHUNT_TYPE, .value = 4 },
	{ .param_id = CSA_GAIN_ADC_I0_TO_ADC_I2, .value = 1 },
	{ .param_id = CSA_GAIN_ADC_I3, .value = 1 },
	{ .param_id = CURRENT_SCALING_FACTOR, .value = 390 },
	{ .param_id = ADC_I0_INVERTED, .value = 1 },
	{ .param_id = ADC_I1_INVERTED, .value = 0 },
	{ .param_id = ADC_I2_INVERTED, .value = 1 },
	{ .param_id = ADC_I3_INVERTED, .value = 0 },

	// Gate driver settings - seem optional but their absence might be the cause of the 6A bug???
	{ .param_id = USE_ADAPTIVE_DRIVE_TIME_UVW, .value = 1 },
	{ .param_id = USE_ADAPTIVE_DRIVE_TIME_Y2, .value = 1 },
	{ .param_id = DRIVE_TIME_SINK_UVW, .value = 40 },
	{ .param_id = DRIVE_TIME_SOURCE_UVW, .value = 40 },
	{ .param_id = DRIVE_TIME_SINK_Y2, .value = 40 },
	{ .param_id = DRIVE_TIME_SOURCE_Y2, .value = 40 },
	{ .param_id = UVW_SINK_CURRENT, .value = 10 },
	{ .param_id = UVW_SOURCE_CURRENT, .value = 5 },
	{ .param_id = Y2_SINK_CURRENT, .value = 10 },
	{ .param_id = Y2_SOURCE_CURRENT, .value = 5 },

	// Velocity ramp: set a max acceleration
#if MOTOR == QSH6018
	{ .param_id = RAMP_ENABLE, .value = 1 },
	{ .param_id = RAMP_AMAX, .value = 100000 },
	{ .param_id = RAMP_DMAX, .value = 100000 },
#elif MOTOR == QSH5718
	{ .param_id = RAMP_ENABLE, .value = 1 },
	{ .param_id = RAMP_AMAX, .value = 500000 },
	{ .param_id = RAMP_DMAX, .value = 500000 },
#elif MOTOR == QSH4218
	{ .param_id = RAMP_ENABLE, .value = 1 },
	{ .param_id = RAMP_AMAX, .value = 300000 },
	{ .param_id = RAMP_DMAX, .value = 300000 },
#endif

	// Velocity biquad
#if MOTOR == QSH6018 || MOTOR == QSH5718 || MOTOR == QSH4218
	// Custom 1st order filter because wizard chooses an unstable one
	// FIXME: talk to Trinamic people on how to get a nice filter here
	{ .param_id = ACTUAL_VELOCITY_BIQUAD_FILTER_ENABLE, .value = 1 },
	{ .param_id = ACTUAL_VELOCITY_BIQUAD_FILTER_ACOEFF_1, .value = 1038090 },
	{ .param_id = ACTUAL_VELOCITY_BIQUAD_FILTER_ACOEFF_2, .value = 0 },
	{ .param_id = ACTUAL_VELOCITY_BIQUAD_FILTER_BCOEFF_0, .value = 3491 },
	{ .param_id = ACTUAL_VELOCITY_BIQUAD_FILTER_BCOEFF_1, .value = 3491 },
	{ .param_id = ACTUAL_VELOCITY_BIQUAD_FILTER_BCOEFF_2, .value = 3491 },
#endif


	// Tuning
#if MOTOR == QSH6018
	{ .param_id = TORQUE_P, .value = 521 },
	{ .param_id = TORQUE_I, .value = 1218 },
	{ .param_id = FLUX_P, .value = 521 },
	{ .param_id = FLUX_I, .value = 1218 },
	{ .param_id = CURRENT_NORM_P, .value = 0 },
	{ .param_id = CURRENT_NORM_I, .value = 1 },
	
	{ .param_id = VELOCITY_P, .value = 315 },
	{ .param_id = VELOCITY_I, .value = 0 },
	{ .param_id = VELOCITY_NORM_P, .value = 1 },
	{ .param_id = VELOCITY_NORM_I, .value = 3 },
	
	{ .param_id = POSITION_P, .value = 10000 },
	{ .param_id = POSITION_I, .value = 0 },
	{ .param_id = POSITION_NORM_P, .value = 1 },
	{ .param_id = POSITION_NORM_I, .value = 1 },
#elif MOTOR == QSH5718
	{ .param_id = TORQUE_P, .value = 741 },
	{ .param_id = TORQUE_I, .value = 2258 },
	{ .param_id = FLUX_P, .value = 741 },
	{ .param_id = FLUX_I, .value = 2258 },
	{ .param_id = CURRENT_NORM_P, .value = 0 },
	{ .param_id = CURRENT_NORM_I, .value = 1 },
	
	{ .param_id = VELOCITY_P, .value = 31718 },
	{ .param_id = VELOCITY_I, .value = 10000 },
	{ .param_id = VELOCITY_NORM_P, .value = 2 },
	{ .param_id = VELOCITY_NORM_I, .value = 2 },
	
	{ .param_id = POSITION_P, .value = 300 },
	{ .param_id = POSITION_I, .value = 0 },
	{ .param_id = POSITION_NORM_P, .value = 0 },
	{ .param_id = POSITION_NORM_I, .value = 1 },
#elif MOTOR == QSH4218
	{ .param_id = TORQUE_P, .value = 3933 },
	{ .param_id = TORQUE_I, .value = 19526 },
	{ .param_id = FLUX_P, .value = 3933 },
	{ .param_id = FLUX_I, .value = 19526 },
	{ .param_id = CURRENT_NORM_P, .value = 0 },
	{ .param_id = CURRENT_NORM_I, .value = 1 },
	
	{ .param_id = VELOCITY_P, .value = 26218 },
	{ .param_id = VELOCITY_I, .value = 100 },
	{ .param_id = VELOCITY_NORM_P, .value = 2 },
	{ .param_id = VELOCITY_NORM_I, .value = 1 },
	
	{ .param_id = POSITION_P, .value = 30 },
	{ .param_id = POSITION_I, .value = 0 },
	{ .param_id = POSITION_NORM_P, .value = 0 },
	{ .param_id = POSITION_NORM_I, .value = 1 },
#endif

	// Protection - seems optional
	{ .param_id = OVERCURRENT_PROTECTION_UVW_LOW_SIDE_THRESHOLD, .value = 1 },
	{ .param_id = OVERCURRENT_PROTECTION_UVW_HIGH_SIDE_THRESHOLD, .value = 2 },
	{ .param_id = OVERCURRENT_PROTECTION_Y2_LOW_SIDE_THRESHOLD, .value = 1 },
	{ .param_id = OVERCURRENT_PROTECTION_Y2_HIGH_SIDE_THRESHOLD, .value = 2 },
	{ .param_id = OVERCURRENT_PROTECTION_UVW_LOW_SIDE_USE_VDS, .value = 0 },
	{ .param_id = OVERCURRENT_PROTECTION_Y2_LOW_SIDE_USE_VDS, .value = 0 },
	{ .param_id = GDRV_RETRY_BEHAVIOUR, .value = 0 },
	{ .param_id = DRIVE_FAULT_BEHAVIOUR, .value = 0 },
	{ .param_id = FAULT_HANDLER_NUMBER_OF_RETRIES, .value = 5 },
	{ .param_id = SUPPLY_OVERVOLTAGE_WARNING_THRESHOLD, .value = 710 },
	{ .param_id = SUPPLY_UNDERVOLTAGE_WARNING_THRESHOLD, .value = 78 },
	{ .param_id = EXTERNAL_TEMPERATURE_SHUTDOWN_THRESHOLD, .value = 65535 },
	{ .param_id = EXTERNAL_TEMPERATURE_WARNING_THRESHOLD, .value = 65535 },

    // { .param_id = EVENT_STOP_SETTINGS, .value = 1 }, // DO_SOFT_STOP, might even set it to 5 or 7 for max slippage functionality (cia 402)
};
#pragma pack(pop)

enum lifter_command {
    COMMAND_NOOP = 0,
    COMMAND_GO_UP = 1,
    COMMAND_GO_DOWN = 2
};

enum lifter_status {
    STATUS_NOOP = 0,
    STATUS_GOING_UP = 1,
    STATUS_GOING_DOWN = 2,
    STATUS_ARRIVED_UP = 3,
    STATUS_ARRIVED_DOWN = 4,
};

enum lifter_command i2c_command = COMMAND_NOOP;
enum lifter_status i2c_status = STATUS_NOOP;

void init_params(struct tmc9660_dev *tmc9660) {
    tmc9660_set_param(tmc9660, COMMUTATION_MODE, 0);

    for(int i = 0; i < ARRAY_SIZE(params); i++) {
        tmc9660_set_param(tmc9660, params[i].param_id, params[i].value);
    }
}

// Before write
int lifter_i2c_write_requested_cb(struct i2c_target_config *config) {
    return 0;
}                   

// Each byte of a write
int lifter_i2c_write_received_cb(struct i2c_target_config *config, uint8_t val) {
    switch(val) {
    case COMMAND_NOOP:
    case COMMAND_GO_UP:
    case COMMAND_GO_DOWN:
        i2c_command = val;
        return 0;
    default:
        return -1;
    }
}

// First byte of a read
int lifter_i2c_read_requested_cb(struct i2c_target_config *config, uint8_t *val) {
    *val = i2c_status;
    return -1;
}

// Following bytes of a read
int lifter_i2c_read_processed_cb(struct i2c_target_config *config, uint8_t *val) {
    *val = i2c_status;
    return -1;
}

// Transaction done, apply changes
int lifter_i2c_stop_cb(struct i2c_target_config *config) {
    return 0; // NOOP
}

static struct i2c_target_callbacks lifter_i2c_callbacks = {
	.write_requested = lifter_i2c_write_requested_cb,
	.write_received = lifter_i2c_write_received_cb,
	.read_requested = lifter_i2c_read_requested_cb,
	.read_processed = lifter_i2c_read_processed_cb,
	.stop = lifter_i2c_stop_cb,
};


int main()
{
    int err;

    LOG_INF("I2C Lifter firmware git revision %s", GIT_REVISION_STR);

    LOG_INF("Initializing TMC9660");
    err = tmc9660_init(&tmc9660, &spi0);
    if(err < 0)
    {
        LOG_ERR("TMC9660 init failed");
        return -1;
    }

    // Assure motor is stopped
    init_params(&tmc9660);
    tmc9660_tmcl_command(&tmc9660, MST, 0, 0, 0, NULL);

    LOG_INF("TMC9660 init OK");

	struct i2c_target_config target_cfg = {
		.address = 0x60, // TODO???
		.callbacks = &lifter_i2c_callbacks,
	};

	if (i2c_target_register(i2cbus, &target_cfg) < 0) {
		LOG_ERR("Failed to register I2C target");
		return -1;
	}

    LOG_INF("I2C init OK");

    while(true)
    {
        //int supply_voltage;
        //tmc9660_get_param(&tmc9660, SUPPLY_VOLTAGE, &supply_voltage);
        //i2c_values.supply_voltage = supply_voltage;

        enum lifter_command command = i2c_command;
        enum lifter_status status = i2c_status;

        int switch1, switch2;

        tmc9660_get_gpio_digital(&tmc9660, 2, &switch1);
        tmc9660_get_gpio_digital(&tmc9660, 15, &switch2);

        int torque;

        if(command == COMMAND_GO_UP) {
            if(switch1) {
                status = STATUS_ARRIVED_UP;
                torque = 0;
            } else {
                status = STATUS_GOING_UP;
                torque = 2500; // mA, configure this!
            }
        } else if(command == COMMAND_GO_DOWN) {
            if(switch2) {
                status = STATUS_ARRIVED_DOWN;
                torque = 0;
            } else {
                status = STATUS_GOING_DOWN;
                torque = -750; // mA, configure this!
            }
        } else {
            torque = 0;
            status = STATUS_NOOP;
        }

        if(torque != 0) {
            tmc9660_set_param(&tmc9660, COMMUTATION_MODE, 4);
            tmc9660_set_param(&tmc9660, TARGET_TORQUE, torque);
        } else {
            tmc9660_set_param(&tmc9660, COMMUTATION_MODE, 0);
        }

        i2c_status = status;
        
        LOG_INF("COMMAND %s\tSW %d %d\tSTATUS %s\tTORQUE %d mA",
            command == COMMAND_NOOP ? "NOOP" : command == COMMAND_GO_UP? "UP" : command == COMMAND_GO_DOWN ? "DOWN" : "???",
            switch1, switch2,
            status == STATUS_NOOP ? "NOOP" : status == STATUS_GOING_UP ? "GOING_UP" : status == STATUS_GOING_DOWN ? "GOING_DOWN" : status == STATUS_ARRIVED_UP ? "ARRIVED_UP" : status == STATUS_ARRIVED_DOWN ? "ARRIVED_DOWN" : "???",
            torque
        );

        k_msleep(10);
    }

    return 0;

fail:
    return err;
}
