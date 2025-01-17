#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/can.h>

#include <canopennode.h> // zephyr module specific helpers!
#include "OD.h"

#include "tmc9660.h"
#include "cia402.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

static const struct gpio_dt_spec led_green = GPIO_DT_SPEC_GET(DT_PATH(leds, led_status), gpios);
static const struct gpio_dt_spec led_fault = GPIO_DT_SPEC_GET(DT_PATH(buttons, fault), gpios);
static const struct spi_dt_spec spi0 = SPI_DT_SPEC_GET(DT_NODELABEL(tmc9660_spi), SPI_MODE_CPHA | SPI_MODE_CPOL | SPI_WORD_SET(8), 0);
static const struct device *can = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));

#define CAN_BITRATE (DT_PROP_OR(DT_CHOSEN(zephyr_canbus), bitrate, \
					  DT_PROP_OR(DT_CHOSEN(zephyr_canbus), bus_speed, \
						     CONFIG_CAN_DEFAULT_BITRATE)) / 1000)

struct tmc9660_dev tmc9660;
struct cia402 cia402;
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

/* Called between processing SYNC RPDO and TPDO. Should send RPDO data (target
 * velocity, mainly) to TMC, read back current state, and write it to the OD for
 * the TPDO that immediately follows.
 * MUST finish its work quickly and not block. It would be nice to do everything
 * in much less than 1ms.
 */
void sync_callback(struct canopen *co, bool sync)
{
    /*if(!sync) return;

    int target_vel;
    int err = OD_get_i32(OD_ENTRY_H60FF_targetVelocity, 0, &target_vel, true);
    if(err != ODR_OK)
    {
        LOG_ERR("Could not read OD Target Velocity");
        return;
    }

    if(cia402.state == CIA402_OPERATION_ENABLED) {
        LOG_INF("Setting target velocity to %d", target_vel);
        tmc9660_set_param(&tmc9660, TARGET_VELOCITY, target_vel);
    }

    int actual_pos, actual_vel, ramp_vel, target_torque, max_torque;
    err = tmc9660_get_param(&tmc9660, ACTUAL_POSITION, &actual_pos);
    if(err) LOG_ERR("ACTUAL_POSITION");
    err = tmc9660_get_param(&tmc9660, ACTUAL_VELOCITY, &actual_vel);
    if(err) LOG_ERR("ACTUAL_VELOCITY");
    err = tmc9660_get_param(&tmc9660, RAMP_VELOCITY, &ramp_vel);
    if(err) LOG_ERR("RAMP_VELOCITY");
    err = tmc9660_get_param(&tmc9660, TARGET_TORQUE, &target_torque);
    if(err) LOG_ERR("TARGET_TORQUE");
    err = tmc9660_get_param(&tmc9660, MAX_TORQUE, &max_torque);
    if(err) LOG_ERR("MAX_TORQUE");

    err = OD_set_i32(OD_ENTRY_H6063_positionActualValue, 0, actual_pos, true);
    if(err) LOG_ERR("actual_pos");
    err = OD_set_i32(OD_ENTRY_H606C_velocityActualValue, 0, actual_vel, true);
    if(err) LOG_ERR("actual_vel");
    err = OD_set_i32(OD_ENTRY_H606B_velocityDemandValue, 0, ramp_vel, true);
    if(err) LOG_ERR("ramp_vel");
    err = OD_set_i16(OD_ENTRY_H6071_targetTorque, 0, target_torque, true);
    if(err) LOG_ERR("target_torque");
    err = OD_set_u16(OD_ENTRY_H6072_maxTorque, 0, max_torque, true);
    if(err) LOG_ERR("max_torque");

    if(err != ODR_OK)
    {
        LOG_ERR("Could not write TMC status to OD");
        return;
    }*/
}

#define VERY_BAD_FAULT() // TODO

void cia402_set_state(struct cia402* cia402, enum cia402_state state)
{
    LOG_INF("CiA 402 state: %s", 
        state == CIA402_NOT_READY_TO_SWITCH_ON ? "NOT_READY_TO_SWITCH_ON" : 
        state == CIA402_SWITCH_ON_DISABLED ? "SWITCH_ON_DISABLED" : 
        state == CIA402_READY_TO_SWITCH_ON ? "READY_TO_SWITCH_ON" : 
        state == CIA402_SWITCHED_ON ? "SWITCHED_ON" : 
        state == CIA402_OPERATION_ENABLED ? "OPERATION_ENABLED" : 
        state == CIA402_QUICK_STOP_ACTIVE ? "QUICK_STOP_ACTIVE" : 
        state == CIA402_FAULT_REACTION_ACTIVE ? "FAULT_REACTION_ACTIVE" : 
        state == CIA402_FAULT ? "FAULT" : "???"
    );

    cia402->state = state;

    int statusword;
    switch(state)
    {
        default:
            cia402->state = CIA402_FAULT;
        case CIA402_FAULT:
            statusword = CIA402_BIT_FAULT;
            break;

        case CIA402_NOT_READY_TO_SWITCH_ON:
            statusword = 0;
            break;
        case CIA402_SWITCH_ON_DISABLED:
            statusword = CIA402_BIT_SWITCHONDISABLED;
            break;
        case CIA402_READY_TO_SWITCH_ON:
            statusword = CIA402_BIT_QUICKSTOP | CIA402_BIT_READYTOSWITCHON;
            break;
        case CIA402_SWITCHED_ON:
            statusword = CIA402_BIT_QUICKSTOP | CIA402_BIT_READYTOSWITCHON | CIA402_BIT_SWITCHON;
            break;
        case CIA402_OPERATION_ENABLED:
            statusword = CIA402_BIT_QUICKSTOP | CIA402_BIT_READYTOSWITCHON | CIA402_BIT_SWITCHON | CIA402_BIT_OPERATIONEN;
            break;
        case CIA402_QUICK_STOP_ACTIVE:
            statusword = CIA402_BIT_READYTOSWITCHON | CIA402_BIT_SWITCHON | CIA402_BIT_OPERATIONEN;
            break;
        case CIA402_FAULT_REACTION_ACTIVE:
            statusword = CIA402_BIT_FAULT | CIA402_BIT_READYTOSWITCHON | CIA402_BIT_SWITCHON | CIA402_BIT_OPERATIONEN;
            break;
    }

    if(OD_set_u16(OD_ENTRY_H6041_statusword, 0, statusword, true)) {
        LOG_ERR("Could not set Statusword");
        VERY_BAD_FAULT();
    }
}

#define MOTOR QSH6018

void agv_init_params(struct tmc9660_dev *tmc9660)
{
	// Turn motor off
	tmc9660_set_param(tmc9660, COMMUTATION_MODE, 0);

	// General motor setup
#if MOTOR == QSH6018 || MOTOR == QSH5718 || MOTOR == QSH4218
	tmc9660_set_param(tmc9660, MOTOR_TYPE, 2);
	tmc9660_set_param(tmc9660, MOTOR_PWM_FREQUENCY, 25000);
	tmc9660_set_param(tmc9660, MOTOR_POLE_PAIRS, 50);
	tmc9660_set_param(tmc9660, ABN_1_STEPS, 40000);
#else
#error Unsupported motor!
#endif

#if MOTOR == QSH6018
	tmc9660_set_param(tmc9660, RAMP_VMAX, 4200000); // Motor struggles to get to 4.2M internal velocity units, by construction. This limits the internal target so we don't get integral windup.
	tmc9660_set_param(tmc9660, MAX_TORQUE, 4000); // mA
	tmc9660_set_param(tmc9660, MAX_FLUX, 4000); // mA
	tmc9660_set_param(tmc9660, OPENLOOP_CURRENT, 500); // 500mA openloop (such as when initially homing ABN)
	tmc9660_set_param(tmc9660, OPENLOOP_VOLTAGE, 1638); // proportional to VIN, 1638 = 10%
#elif MOTOR == QSH5718
	tmc9660_set_param(tmc9660, RAMP_VMAX, 4100000);
	tmc9660_set_param(tmc9660, MAX_TORQUE, 4000); // mA
	tmc9660_set_param(tmc9660, MAX_FLUX, 4000); // mA
	tmc9660_set_param(tmc9660, OPENLOOP_CURRENT, 500); // 500mA openloop (such as when initially homing ABN)
	tmc9660_set_param(tmc9660, OPENLOOP_VOLTAGE, 1638); // proportional to VIN, 1638 = 10%
#elif MOTOR == QSH4218
	tmc9660_set_param(tmc9660, RAMP_VMAX, 1234);
	tmc9660_set_param(tmc9660, MAX_TORQUE, 2000); // mA
	tmc9660_set_param(tmc9660, MAX_FLUX, 2000); // mA
	tmc9660_set_param(tmc9660, OPENLOOP_CURRENT, 500); // 500mA openloop (such as when initially homing ABN)
	tmc9660_set_param(tmc9660, OPENLOOP_VOLTAGE, 1638); // proportional to VIN, 1638 = 10%
#endif

	// General limits
	tmc9660_set_param(tmc9660, OUTPUT_VOLTAGE_LIMIT, 20000); // proportional to VIN, 16383 = 100%, goes up to 200%

	// ADC Setup - depends on board, not motor!
	tmc9660_set_param(tmc9660, ADC_SHUNT_TYPE, 4);
	tmc9660_set_param(tmc9660, CSA_GAIN_ADC_I0_TO_ADC_I2, 1);
	tmc9660_set_param(tmc9660, CSA_GAIN_ADC_I3, 1);
	tmc9660_set_param(tmc9660, CURRENT_SCALING_FACTOR, 390);
	tmc9660_set_param(tmc9660, ADC_I0_INVERTED, 1);
	tmc9660_set_param(tmc9660, ADC_I1_INVERTED, 0);
	tmc9660_set_param(tmc9660, ADC_I2_INVERTED, 1);
	tmc9660_set_param(tmc9660, ADC_I3_INVERTED, 0);

	// Gate driver settings - seem optional but their absence might be the cause of the 6A bug???
	tmc9660_set_param(tmc9660, USE_ADAPTIVE_DRIVE_TIME_UVW, 1);
	tmc9660_set_param(tmc9660, USE_ADAPTIVE_DRIVE_TIME_Y2, 1);
	tmc9660_set_param(tmc9660, DRIVE_TIME_SINK_UVW, 40);
	tmc9660_set_param(tmc9660, DRIVE_TIME_SOURCE_UVW, 40);
	tmc9660_set_param(tmc9660, DRIVE_TIME_SINK_Y2, 40);
	tmc9660_set_param(tmc9660, DRIVE_TIME_SOURCE_Y2, 40);
	tmc9660_set_param(tmc9660, UVW_SINK_CURRENT, 10);
	tmc9660_set_param(tmc9660, UVW_SOURCE_CURRENT, 5);
	tmc9660_set_param(tmc9660, Y2_SINK_CURRENT, 10);
	tmc9660_set_param(tmc9660, Y2_SOURCE_CURRENT, 5);

	// Velocity ramp: set a max acceleration
#if MOTOR == QSH6018
	tmc9660_set_param(tmc9660, RAMP_ENABLE, 1);
	tmc9660_set_param(tmc9660, RAMP_AMAX, 100000);
	tmc9660_set_param(tmc9660, RAMP_DMAX, 100000);
#elif MOTOR == QSH5718
	tmc9660_set_param(tmc9660, RAMP_ENABLE, 1);
	tmc9660_set_param(tmc9660, RAMP_AMAX, 500000);
	tmc9660_set_param(tmc9660, RAMP_DMAX, 500000);
#elif MOTOR == QSH4218
	tmc9660_set_param(tmc9660, RAMP_ENABLE, 1);
	tmc9660_set_param(tmc9660, RAMP_AMAX, 300000);
	tmc9660_set_param(tmc9660, RAMP_DMAX, 300000);
#endif

	// Velocity biquad
#if MOTOR == QSH6018 || MOTOR == QSH5718 || MOTOR == QSH4218
	// Custom 1st order filter because wizard chooses an unstable one
	// FIXME: talk to Trinamic people on how to get a nice filter here
	tmc9660_set_param(tmc9660, ACTUAL_VELOCITY_BIQUAD_FILTER_ENABLE, 1);
	tmc9660_set_param(tmc9660, ACTUAL_VELOCITY_BIQUAD_FILTER_ACOEFF_1, 1038090);
	tmc9660_set_param(tmc9660, ACTUAL_VELOCITY_BIQUAD_FILTER_ACOEFF_2, 0);
	tmc9660_set_param(tmc9660, ACTUAL_VELOCITY_BIQUAD_FILTER_BCOEFF_0, 3491);
	tmc9660_set_param(tmc9660, ACTUAL_VELOCITY_BIQUAD_FILTER_BCOEFF_1, 3491);
	tmc9660_set_param(tmc9660, ACTUAL_VELOCITY_BIQUAD_FILTER_BCOEFF_2, 3491);
#endif


	// Tuning
#if MOTOR == QSH6018
	tmc9660_set_param(tmc9660, TORQUE_P, 521);
	tmc9660_set_param(tmc9660, TORQUE_I, 1218);
	tmc9660_set_param(tmc9660, FLUX_P, 521);
	tmc9660_set_param(tmc9660, FLUX_I, 1218);
	tmc9660_set_param(tmc9660, CURRENT_NORM_P, 0);
	tmc9660_set_param(tmc9660, CURRENT_NORM_I, 1);
	
	tmc9660_set_param(tmc9660, VELOCITY_P, 315);
	tmc9660_set_param(tmc9660, VELOCITY_I, 0);
	tmc9660_set_param(tmc9660, VELOCITY_NORM_P, 1);
	tmc9660_set_param(tmc9660, VELOCITY_NORM_I, 3);
	
	tmc9660_set_param(tmc9660, POSITION_P, 10000);
	tmc9660_set_param(tmc9660, POSITION_I, 0);
	tmc9660_set_param(tmc9660, POSITION_NORM_P, 1);
	tmc9660_set_param(tmc9660, POSITION_NORM_I, 1);
#elif MOTOR == QSH5718
	tmc9660_set_param(tmc9660, TORQUE_P, 741);
	tmc9660_set_param(tmc9660, TORQUE_I, 2258);
	tmc9660_set_param(tmc9660, FLUX_P, 741);
	tmc9660_set_param(tmc9660, FLUX_I, 2258);
	tmc9660_set_param(tmc9660, CURRENT_NORM_P, 0);
	tmc9660_set_param(tmc9660, CURRENT_NORM_I, 1);
	
	tmc9660_set_param(tmc9660, VELOCITY_P, 31718);
	tmc9660_set_param(tmc9660, VELOCITY_I, 10000);
	tmc9660_set_param(tmc9660, VELOCITY_NORM_P, 2);
	tmc9660_set_param(tmc9660, VELOCITY_NORM_I, 2);
	
	tmc9660_set_param(tmc9660, POSITION_P, 300);
	tmc9660_set_param(tmc9660, POSITION_I, 0);
	tmc9660_set_param(tmc9660, POSITION_NORM_P, 0);
	tmc9660_set_param(tmc9660, POSITION_NORM_I, 1);
#elif MOTOR == QSH4218
	tmc9660_set_param(tmc9660, TORQUE_P, 3933);
	tmc9660_set_param(tmc9660, TORQUE_I, 19526);
	tmc9660_set_param(tmc9660, FLUX_P, 3933);
	tmc9660_set_param(tmc9660, FLUX_I, 19526);
	tmc9660_set_param(tmc9660, CURRENT_NORM_P, 0);
	tmc9660_set_param(tmc9660, CURRENT_NORM_I, 1);
	
	tmc9660_set_param(tmc9660, VELOCITY_P, 26218);
	tmc9660_set_param(tmc9660, VELOCITY_I, 100);
	tmc9660_set_param(tmc9660, VELOCITY_NORM_P, 2);
	tmc9660_set_param(tmc9660, VELOCITY_NORM_I, 1);
	
	tmc9660_set_param(tmc9660, POSITION_P, 30);
	tmc9660_set_param(tmc9660, POSITION_I, 0);
	tmc9660_set_param(tmc9660, POSITION_NORM_P, 0);
	tmc9660_set_param(tmc9660, POSITION_NORM_I, 1);
#endif

	// Protection - seems optional
	tmc9660_set_param(tmc9660, OVERCURRENT_PROTECTION_UVW_LOW_SIDE_THRESHOLD, 1);
	tmc9660_set_param(tmc9660, OVERCURRENT_PROTECTION_UVW_HIGH_SIDE_THRESHOLD, 2);
	tmc9660_set_param(tmc9660, OVERCURRENT_PROTECTION_Y2_LOW_SIDE_THRESHOLD, 1);
	tmc9660_set_param(tmc9660, OVERCURRENT_PROTECTION_Y2_HIGH_SIDE_THRESHOLD, 2);
	tmc9660_set_param(tmc9660, OVERCURRENT_PROTECTION_UVW_LOW_SIDE_USE_VDS, 0);
	tmc9660_set_param(tmc9660, OVERCURRENT_PROTECTION_Y2_LOW_SIDE_USE_VDS, 0);
	tmc9660_set_param(tmc9660, GDRV_RETRY_BEHAVIOUR, 0);
	tmc9660_set_param(tmc9660, DRIVE_FAULT_BEHAVIOUR, 0);
	tmc9660_set_param(tmc9660, FAULT_HANDLER_NUMBER_OF_RETRIES, 5);
	tmc9660_set_param(tmc9660, SUPPLY_OVERVOLTAGE_WARNING_THRESHOLD, 710);
	tmc9660_set_param(tmc9660, SUPPLY_UNDERVOLTAGE_WARNING_THRESHOLD, 78);
	tmc9660_set_param(tmc9660, EXTERNAL_TEMPERATURE_SHUTDOWN_THRESHOLD, 65535);
	tmc9660_set_param(tmc9660, EXTERNAL_TEMPERATURE_WARNING_THRESHOLD, 65535);

    // tmc9660_set_param(tmc9660, EVENT_STOP_SETTINGS, 1); // DO_SOFT_STOP, might even set it to 5 or 7 for max slippage functionality (cia 402)

	// Start FOC
	// tmc9660_set_param(tmc9660, COMMUTATION_MODE, 5);
}

int main()
{
    int err;

	err = init_leds();
    if(err < 0)
    {
        LOG_ERR("CiA 303 LED indicator init failed");
        goto fail;
    }

    co.can_dev = can;
    co.node_id = 0x10; // TODO read DIP switch for Node ID
    co.bitrate = CAN_BITRATE;
    co.nmt_control = 0;
    co.led_callback = led_callback;
    co.sync_callback = sync_callback;
    err = canopen_init(&co);
    if(err < 0)
    {
        LOG_ERR("CANopenNode init failed");
        goto fail;
    }

    cia402.co = &co;
    cia402_set_state(&cia402, CIA402_NOT_READY_TO_SWITCH_ON); // Transition 0

    while(true)
    {
        if(co.CO->NMT->operatingState == CO_NMT_STOPPED) {
            cia402_set_state(&cia402, CIA402_FAULT); // Special transition
            k_msleep(100);
            continue;
        }
        
        if(co.CO->NMT->operatingState != CO_NMT_OPERATIONAL) {
            k_msleep(1);
            continue;
        }

        if(cia402.state == CIA402_NOT_READY_TO_SWITCH_ON) {
            LOG_INF("Initializing TMC9660");

            err = tmc9660_init(&tmc9660, &spi0);
            if(err < 0)
            {
                LOG_ERR("TMC9660 init failed");
                cia402_set_state(&cia402, CIA402_FAULT);
                CO_errorReport(co.CO->em, CO_ERR_REG_GENERIC_ERR, CO_EMC_HARDWARE, 9660);
                CO_NMT_sendInternalCommand(co.CO->NMT, CO_NMT_ENTER_STOPPED);
                continue;
            }

            // Assure motor is stopped
            tmc9660_tmcl_command(&tmc9660, MST, 0, 0, 0, NULL);

            LOG_INF("TMC9660 init OK");

            LOG_INF("Writing parameters");
            agv_init_params(&tmc9660);

            cia402_set_state(&cia402, CIA402_SWITCH_ON_DISABLED); // Transition 1
            continue;
        }

        if(cia402.state == CIA402_OPERATION_ENABLED) {
            int target_vel;
            int err = OD_get_i32(OD_ENTRY_H60FF_targetVelocity, 0, &target_vel, true);
            if(err != ODR_OK)
            {
                LOG_ERR("Could not read OD Target Velocity");
                return -1;
            }

            //LOG_INF("Setting target velocity to %d", target_vel);
            tmc9660_set_param(&tmc9660, TARGET_VELOCITY, target_vel);

            int actual_pos, actual_vel, ramp_vel, target_torque, max_torque;
            err = tmc9660_get_param(&tmc9660, ACTUAL_POSITION, &actual_pos);
            if(err) LOG_ERR("ACTUAL_POSITION %d", err);
            err = tmc9660_get_param(&tmc9660, ACTUAL_VELOCITY, &actual_vel);
            if(err) LOG_ERR("ACTUAL_VELOCITY %d", err);
            err = tmc9660_get_param(&tmc9660, RAMP_VELOCITY, &ramp_vel);
            if(err) LOG_ERR("RAMP_VELOCITY %d", err);
            err = tmc9660_get_param(&tmc9660, TARGET_TORQUE, &target_torque);
            if(err) LOG_ERR("TARGET_TORQUE %d", err);
            err = tmc9660_get_param(&tmc9660, MAX_TORQUE, &max_torque);
            if(err) LOG_ERR("MAX_TORQUE %d", err);

            err = OD_set_i32(OD_ENTRY_H6063_positionActualValue, 0, actual_pos, true);
            if(err) LOG_ERR("actual_pos");
            err = OD_set_i32(OD_ENTRY_H606C_velocityActualValue, 0, actual_vel, true);
            if(err) LOG_ERR("actual_vel");
            err = OD_set_i32(OD_ENTRY_H606B_velocityDemandValue, 0, ramp_vel, true);
            if(err) LOG_ERR("ramp_vel");
            err = OD_set_i16(OD_ENTRY_H6071_targetTorque, 0, target_torque, true);
            if(err) LOG_ERR("target_torque");
            err = OD_set_u16(OD_ENTRY_H6072_maxTorque, 0, max_torque, true);
            if(err) LOG_ERR("max_torque");

            if(err != ODR_OK)
            {
                LOG_ERR("Could not write TMC status to OD");
                return -1;
            }

            k_msleep(1);
        }

        /******* CONTROLWORD TRANSITIONS *******/
        uint16_t controlword;
        err = OD_get_u16(OD_ENTRY_H6040_controlword, 0, &controlword, true);
        if(err != ODR_OK) {
            LOG_ERR("Could not read controlword, %d", err);
            break;
        }

        bool cw_halt = (controlword >> 8) & 1;
        bool cw_fault_reset = (controlword >> 7) & 1;
        bool cw_enable_operation = (controlword >> 3) & 1;
        bool cw_quick_stop = (controlword >> 2) & 1;
        bool cw_enable_voltage = (controlword >> 1) & 1;
        bool cw_switch_on = (controlword >> 0) & 1;

        bool tr_shutdown = cw_quick_stop && cw_enable_voltage && !cw_switch_on;
        bool tr_switchon = cw_quick_stop && cw_enable_voltage && cw_switch_on;
        bool tr_disablev = !cw_enable_voltage;
        bool tr_quickstp = !cw_quick_stop && cw_enable_voltage;
        bool tr_disableo = !cw_enable_operation && cw_quick_stop && cw_enable_voltage && cw_switch_on;
        bool tr_enableop = cw_enable_operation && cw_quick_stop && cw_enable_voltage && cw_switch_on;

        enum cia402_state nextState = cia402.state; // By default, don't change state
        int flags;
        tmc9660_get_param(&tmc9660, GENERAL_STATUS_FLAGS, &flags);
        bool velocity_reached = flags & 1024;
        bool regulation_stopped = flags & 1;

        if(cia402.state == CIA402_SWITCH_ON_DISABLED) {
            if(tr_shutdown) { // Transition 2
                nextState = CIA402_READY_TO_SWITCH_ON;
            } else {
                // LOG_WRN("Controlword %04X invalid for state SWITCH_ON_DISABLED", controlword);
            }
        } else if(cia402.state == CIA402_READY_TO_SWITCH_ON) {
            if(tr_switchon) { // Transition 3
                nextState = CIA402_SWITCHED_ON;
            } else if(tr_disablev || tr_quickstp) { // Transition 7
                nextState = CIA402_SWITCH_ON_DISABLED;
            } else {
                // LOG_WRN("Controlword %04X invalid for state READY_TO_SWITCH_ON", controlword);
            }
        } else if(cia402.state == CIA402_SWITCHED_ON) {
            if(tr_enableop) { // Transition 4
                nextState = CIA402_OPERATION_ENABLED;
            } else if(tr_shutdown) { // Transition 6
                nextState = CIA402_READY_TO_SWITCH_ON;
            } else if(tr_disablev || tr_quickstp) { // Transition 10
                nextState = CIA402_SWITCH_ON_DISABLED;
            } else {
                // LOG_WRN("Controlword %04X invalid for state SWITCHED_ON", controlword);
            }
        } else if(cia402.state == CIA402_OPERATION_ENABLED) {
            if(tr_disableo) { // Transition 5
                nextState = CIA402_SWITCHED_ON;
            } else if(tr_shutdown) { // Transition 8
                nextState = CIA402_READY_TO_SWITCH_ON;
            } else if(tr_disablev) { // Transition 9
                nextState = CIA402_SWITCH_ON_DISABLED;
            } else if (tr_quickstp || cw_halt) { // Transition 11 + custom
                nextState = CIA402_QUICK_STOP_ACTIVE;
            } else {
                // LOG_WRN("Controlword %04X invalid for state OPERATION_ENABLED", controlword);
            }
        } else if(cia402.state == CIA402_QUICK_STOP_ACTIVE) {
            if(tr_disablev || velocity_reached || regulation_stopped) { // Transition 12
                nextState = CIA402_SWITCH_ON_DISABLED;
            }
            // TODO Transition 16
        } else if(cia402.state == CIA402_FAULT_REACTION_ACTIVE) {
            if(velocity_reached || regulation_stopped) { // Transition 14
                nextState = CIA402_FAULT;
            }
        } else if(cia402.state == CIA402_FAULT) {
            if(cw_fault_reset) { // Transition 15
                nextState = CIA402_SWITCH_ON_DISABLED;
            }
        } else {
            LOG_ERR("Invalid state %d", cia402.state);
        }

        /***************** STATE UPDATE AND SETUP *******************/
        if(nextState != cia402.state) {
            cia402_set_state(&cia402, nextState);

            bool shouldMotorStop = false;

            if(nextState == CIA402_SWITCH_ON_DISABLED) {
                shouldMotorStop = true;
                tmc9660_set_param(&tmc9660, COMMUTATION_MODE, 0);
            } else if(nextState == CIA402_READY_TO_SWITCH_ON) {
                shouldMotorStop = true;
                tmc9660_set_param(&tmc9660, COMMUTATION_MODE, 0);
            } else if(nextState == CIA402_SWITCHED_ON) {
                tmc9660_set_param(&tmc9660, COMMUTATION_MODE, 5);
            } else if(nextState == CIA402_OPERATION_ENABLED) {
                
            } else if(nextState == CIA402_QUICK_STOP_ACTIVE) {
                shouldMotorStop = true;
                tmc9660_tmcl_command(&tmc9660, MST, 0, 0, 0, NULL);
            } else if(nextState == CIA402_FAULT_REACTION_ACTIVE) {
                shouldMotorStop = true;
                tmc9660_tmcl_command(&tmc9660, MST, 0, 0, 0, NULL);
            }

            if(shouldMotorStop) {
                // Also set OD target velocity so next time motor is enabled, it doesn't spring back to its last set velocity of a previous run
                OD_set_i32(OD_ENTRY_H60FF_targetVelocity, 0, 0, true);
            }
        }

        k_msleep(1);
    }

    return 0;

fail:
    return err;
}
