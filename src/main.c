#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/can.h>
#include <canopennode.h>
#include "OD.h"

#include "tmc9660.h"

#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

static const struct gpio_dt_spec led_green = GPIO_DT_SPEC_GET(DT_PATH(leds, led_status), gpios);
static const struct gpio_dt_spec led_fault = GPIO_DT_SPEC_GET(DT_PATH(buttons, fault), gpios);
static const struct spi_dt_spec spi0 = SPI_DT_SPEC_GET(DT_NODELABEL(tmc9660_spi), SPI_MODE_CPHA | SPI_MODE_CPOL | SPI_WORD_SET(8), 0);
static const struct device *can = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));

#define CAN_BITRATE (DT_PROP_OR(DT_CHOSEN(zephyr_canbus), bitrate, \
					  DT_PROP_OR(DT_CHOSEN(zephyr_canbus), bus_speed, \
						     CONFIG_CAN_DEFAULT_BITRATE)) / 1000)

struct tmc9660_dev tmc9660;
struct canopen co;

/* Indicator LEDs */

/* CiA 303-3 standardizes how to indicate the status of the device using a red
 * and green LED, covering blink patterns for various errors / states.
 */
void led_callback(struct canopen *co, bool green, bool red)
{
    gpio_pin_set_dt(&led_green, green);

	/* Red LED is shared between the TMC and MAX. To light it up,
	* its GPIO must be enabled and pulled low. To turn it off,
	* its GPIO should be floated (so that the TMC may control it).
	*/
    gpio_pin_configure_dt(&led_fault, red ? GPIO_OUTPUT_ACTIVE : GPIO_INPUT);
}

int init_leds()
{
    int err;

    if(!gpio_is_ready_dt(&led_green))
    {
        LOG_ERR("Green LED device not ready");
        return -1;
    }
    
    if(!gpio_is_ready_dt(&led_fault))
    {
        LOG_ERR("Red LED device not ready");
        return -1;
    }

    if((err = gpio_pin_configure_dt(&led_green, GPIO_OUTPUT_INACTIVE)))
    {
        LOG_ERR("Green LED configure failed: %d", err);
        return err;
    }
    
    if((err = gpio_pin_configure_dt(&led_fault, GPIO_INPUT)))
    {
        LOG_ERR("Fault LED configure failed: %d", err);
        return err;
    }

    return 0;
}

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
	{ .param_id = OPENLOOP_CURRENT, .value = 1000 },
	{ .param_id = OPENLOOP_VOLTAGE, .value = 16383 },
    { .param_id = OUTPUT_VOLTAGE_LIMIT, .value = 20000 }, // proportional to VIN, 16383 = 100%, goes up to 200%
    
	{ .param_id = TORQUE_P, .value = 2500 },
	{ .param_id = TORQUE_I, .value = 0 },
	{ .param_id = FLUX_P, .value = 2500 },
	{ .param_id = FLUX_I, .value = 0 },
	{ .param_id = CURRENT_NORM_P, .value = 0 },
	{ .param_id = CURRENT_NORM_I, .value = 0 },
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

void init_params(struct tmc9660_dev *tmc9660) {
    tmc9660_set_param(tmc9660, COMMUTATION_MODE, 0);

    for(int i = 0; i < ARRAY_SIZE(params); i++) {
        tmc9660_set_param(tmc9660, params[i].param_id, params[i].value);
    }
}

void pretty_error_counts(const struct device *can) {
    static int prev_tx_err_cnt = 0, prev_rx_err_cnt = 0;
    static enum can_state prev_state = CAN_STATE_STOPPED;
	struct can_bus_err_cnt err_cnt;
	enum can_state state;
	int err;

	err = can_get_state(can, &state, &err_cnt);
	if (err != 0) {
		LOG_ERR("failed to get CAN controller state (err %d)", err);
		return;
	}
    if(prev_tx_err_cnt != err_cnt.tx_err_cnt || prev_rx_err_cnt != err_cnt.rx_err_cnt || prev_state != state) {
		LOG_DBG("CAN TEC = %d\tREC = %d\tstate = %s", err_cnt.tx_err_cnt, err_cnt.rx_err_cnt,
            state == CAN_STATE_ERROR_ACTIVE ? "ACTIVE" :
            state == CAN_STATE_ERROR_WARNING ? "WARNING" :
            state == CAN_STATE_ERROR_PASSIVE ? "PASSIVE" :
            state == CAN_STATE_BUS_OFF ? "BUS_OFF" :
            state == CAN_STATE_STOPPED ? "STOPPED" : "???"
        );
    }

    if(state == CAN_STATE_BUS_OFF) {
        // TODO rate limit, if it goes into bus off a lot, stop resetting and just leave it in the fault state
        LOG_ERR("Bus-off: killing motor");
        CO_NMT_sendInternalCommand(co.CO->NMT, CO_NMT_ENTER_STOPPED);
        tmc9660_tmcl_command(&tmc9660, MST, 0, 0, 0, NULL);
    }
    
    prev_tx_err_cnt = err_cnt.tx_err_cnt;
    prev_rx_err_cnt = err_cnt.rx_err_cnt;
    prev_state = state;
}

enum LIFTER_COMMAND {
    LIFTER_COMMAND_NOOP = 0,
    LIFTER_COMMAND_GO_UP = 1,
    LIFTER_COMMAND_GO_DOWN = 2
};

enum LIFTER_STATUS {
    LIFTER_STATUS_NOOP = 0,
    LIFTER_STATUS_GOING_UP = 1,
    LIFTER_STATUS_GOING_DOWN = 2,
    LIFTER_STATUS_ARRIVED_UP = 3,
    LIFTER_STATUS_ARRIVED_DOWN = 4,
};

#define MOVE_CURRENT 500 // mA, configure for load. FIXME larger current in the first few moments of moving from down to up

int main()
{
    int err;

    LOG_INF("Lifter firmware, git revision %s", GIT_REVISION_STR);

	err = init_leds();
    if(err < 0)
    {
        LOG_ERR("CiA 303 LED indicator init failed");
        goto fail;
    }

    // Read DIP switches for Node ID
    co.can_dev = can;
    co.node_id = 0x18;
    LOG_INF("CANopen node ID = %d", co.node_id);
    co.bitrate = CAN_BITRATE;
    co.nmt_control = 0;
    co.led_callback = led_callback;
    co.nmt_control = CO_NMT_ERR_FREE_TO_OPERATIONAL;
    // co.sync_callback = sync_callback;
    err = canopen_init(&co);
    if(err < 0)
    {
        LOG_ERR("CANopenNode init failed");
        goto fail;
    }

    LOG_INF("Initializing TMC9660");
    err = tmc9660_init(&tmc9660, &spi0);
    if(err < 0)
    {
        LOG_ERR("TMC9660 init failed");
        CO_errorReport(co.CO->em, CO_ERR_REG_GENERIC_ERR, CO_EMC_HARDWARE, 9660);
        CO_NMT_sendInternalCommand(co.CO->NMT, CO_NMT_ENTER_STOPPED);
        return -1;
    }

    init_params(&tmc9660);
    tmc9660_set_param(&tmc9660, COMMUTATION_MODE, 4);

    LOG_INF("TMC9660 init OK");

    while(true)
    {
        pretty_error_counts(co.can_dev);

        int supply_voltage, comm;
        tmc9660_get_param(&tmc9660, SUPPLY_VOLTAGE, &supply_voltage);
        tmc9660_get_param(&tmc9660, COMMUTATION_MODE, &comm);
        OD_set_u16(OD_ENTRY_H2122_TMC9660SUPPLY_VOLTAGE, 0, (uint16_t) supply_voltage, false);

        if(co.CO->NMT->operatingState == CO_NMT_STOPPED) {
            LOG_INF("NMT STOPPED");
            while(co.CO->NMT->operatingState == CO_NMT_STOPPED) { // No point in doing anything until the canopen thread takes us out of the stopped state
                k_msleep(1);
            }
            LOG_INF("NMT un-stopped");
            continue;
        }
        
        if(co.CO->NMT->operatingState != CO_NMT_OPERATIONAL) {
            k_msleep(1);
            continue;
        }

        int switch1, switch2;
        tmc9660_get_gpio_digital(&tmc9660, 2, &switch1);
        tmc9660_get_gpio_digital(&tmc9660, 15, &switch2);

        uint8_t command = 0, status = 0;
        OD_get_u8(OD_ENTRY_H2001_lifterCommand, 0, &command, false);

        switch(command) {
        case LIFTER_COMMAND_NOOP:
            status = LIFTER_STATUS_NOOP;
            tmc9660_set_param(&tmc9660, TARGET_TORQUE, 0);
            break;

        case LIFTER_COMMAND_GO_UP:
            if(!switch1) {
                if(comm != 4) {
                    tmc9660_set_param(&tmc9660, COMMUTATION_MODE, 4);
                }
                tmc9660_set_param(&tmc9660, TARGET_TORQUE, MOVE_CURRENT);
                status = LIFTER_STATUS_GOING_UP;
            } else {
                tmc9660_set_param(&tmc9660, TARGET_TORQUE, 0);
                status = LIFTER_STATUS_ARRIVED_UP;
            }
            break;
        
        case LIFTER_COMMAND_GO_DOWN:
            if(!switch2) {
                if(comm != 4) {
                    tmc9660_set_param(&tmc9660, COMMUTATION_MODE, 4);
                }
                tmc9660_set_param(&tmc9660, TARGET_TORQUE, -MOVE_CURRENT);
                status = LIFTER_STATUS_GOING_DOWN;
            } else {
                tmc9660_set_param(&tmc9660, TARGET_TORQUE, 0);
                status = LIFTER_STATUS_ARRIVED_DOWN;
            }
            break;
        }

        int prev_status;
        OD_get_u8(OD_ENTRY_H2002_lifterStatus, 0, &prev_status, false);
        OD_set_u8(OD_ENTRY_H2002_lifterStatus, 0, status, false);
        
        if(status != prev_status) {
            OD_requestTPDO(OD_getFlagsPDO(OD_ENTRY_H2002_lifterStatus), 0);
        }

        LOG_DBG("CMD %s, switches %d %d => STATUS %s",
            command == 0 ? "NOOP" : command == 1 ? "UP" : command == 2 ? "DOWN" : "???",
            switch1, switch2,
            status == 0 ? "NOOP" : status == 1 ? "GOING UP" : status == 2 ? "GOING DOWN" : status == 3 ? "ARRIVED UP" : status == 4 ? "ARRIVED DOWN" : "???"
        );
        
        k_msleep(10);
    }

    return 0;

fail:
    return err;
}
