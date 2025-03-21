#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>
#include <canopennode.h>

#include "tmc9660.h"
#include "cia402.h"
#include "OD.h"
#include "tmc9660_params.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

static const struct gpio_dt_spec led_green = GPIO_DT_SPEC_GET(DT_PATH(leds, led_status), gpios);
static const struct gpio_dt_spec led_fault = GPIO_DT_SPEC_GET(DT_PATH(buttons, fault), gpios);
static const struct spi_dt_spec spi0 = SPI_DT_SPEC_GET(DT_NODELABEL(tmc9660_spi), SPI_MODE_CPHA | SPI_MODE_CPOL | SPI_WORD_SET(8), 0);
static const struct device *can = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));

static const struct gpio_dt_spec dip1 = GPIO_DT_SPEC_GET_BY_IDX(DT_PATH(buttons, dip), gpios, 0);
static const struct gpio_dt_spec dip2 = GPIO_DT_SPEC_GET_BY_IDX(DT_PATH(buttons, dip), gpios, 1);
static const struct gpio_dt_spec dip3 = GPIO_DT_SPEC_GET_BY_IDX(DT_PATH(buttons, dip), gpios, 2);

#define CAN_BITRATE (DT_PROP_OR(DT_CHOSEN(zephyr_canbus), bitrate, \
					  DT_PROP_OR(DT_CHOSEN(zephyr_canbus), bus_speed, \
						     CONFIG_CAN_DEFAULT_BITRATE)) / 1000)

struct tmc9660_dev g_tmc9660;
struct canopen co;
struct cia402 cia402;

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

void cia402_init(struct cia402* cia402) {
    cia402->statusword_extension.read = OD_readOriginal;
    cia402->statusword_extension.write = OD_writeOriginal;
    cia402->mod_extension.read = OD_readOriginal;
    cia402->mod_extension.write = OD_writeOriginal;
    cia402->vel_extension.read = OD_readOriginal;
    cia402->vel_extension.write = OD_writeOriginal;
    cia402->pos_extension.read = OD_readOriginal;
    cia402->pos_extension.write = OD_writeOriginal;

    OD_extension_init(OD_ENTRY_H6041_statusword, &cia402->statusword_extension);
    OD_extension_init(OD_ENTRY_H6061_modesOfOperationDisplay, &cia402->mod_extension);
    OD_extension_init(OD_ENTRY_H606C_velocityActualValue, &cia402->vel_extension);
    OD_extension_init(OD_ENTRY_H6064_positionActualValue, &cia402->pos_extension);
}

void cia402_set_state(struct cia402* cia402, enum cia402_state state)
{
    LOG_INF("Setting CiA 402 state: %s", 
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

    LOG_DBG("Setting statusword to %04X", statusword);
    OD_requestTPDO(OD_getFlagsPDO(OD_ENTRY_H6041_statusword), 0);
    if(OD_set_u16(OD_ENTRY_H6041_statusword, 0, statusword, true)) {
        LOG_ERR("Could not set Statusword");
        LOG_PANIC();
        k_oops();
    }
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
        cia402_set_state(&cia402, CIA402_FAULT);
        tmc9660_tmcl_command(&tmc9660, MST, 0, 0, 0, NULL);
    }
    
    prev_tx_err_cnt = err_cnt.tx_err_cnt;
    prev_rx_err_cnt = err_cnt.rx_err_cnt;
    prev_state = state;
}

int main()
{
    int err;

    LOG_INF("Firmware git revision %s", GIT_REVISION_STR);

	err = init_leds();
    if(err < 0)
    {
        LOG_ERR("CiA 303 LED indicator init failed");
        goto fail;
    }

    // Read DIP switches for Node ID

    err = gpio_pin_configure_dt(&dip1, GPIO_INPUT);
    if(err < 0) {
        LOG_ERR("Could not initialize DIP GPIO 1: %d", err);
        goto fail;
    }
    err = gpio_pin_configure_dt(&dip2, GPIO_INPUT);
    if(err < 0) {
        LOG_ERR("Could not initialize DIP GPIO 2: %d", err);
        goto fail;
    }
    err = gpio_pin_configure_dt(&dip3, GPIO_INPUT);
    if(err < 0) {
        LOG_ERR("Could not initialize DIP GPIO 3: %d", err);
        goto fail;
    }
    
    int dip_setting = (gpio_pin_get_dt(&dip1) << 2) | (gpio_pin_get_dt(&dip2) << 1) | (gpio_pin_get_dt(&dip3) << 0); // Looking at the DIP switch with "1 2 3" going left to right = reading the number in binary
    LOG_INF("DIP switches set to %d", dip_setting);

    k_msleep(dip_setting * 5); // Delay each node's bringup differently to reduce initial clashes

    co.can_dev = can;
    co.node_id = 0x10 + dip_setting;
    LOG_INF("CANopen node ID = %d", co.node_id);
    co.bitrate = CAN_BITRATE;
    co.nmt_control = CO_NMT_STARTUP_TO_OPERATIONAL;
    co.led_callback = led_callback;
    // co.sync_callback = sync_callback;
    err = canopen_init(&co);
    if(err < 0)
    {
        LOG_ERR("CANopenNode init failed");
        goto fail;
    }

    cia402.co = &co;
    cia402_init(&cia402);
    cia402_set_state(&cia402, CIA402_NOT_READY_TO_SWITCH_ON); // Transition 0

    LOG_INF("Initializing TMC9660");
    err = tmc9660_init(&tmc9660, &spi0);
    if(err < 0)
    {
        LOG_ERR("TMC9660 init failed");
        cia402_set_state(&cia402, CIA402_FAULT);
        CO_errorReport(co.CO->em, CO_ERR_REG_GENERIC_ERR, CO_EMC_HARDWARE, 9660);
        CO_NMT_sendInternalCommand(co.CO->NMT, CO_NMT_ENTER_STOPPED);
        return;
    }

    // Assure motor is stopped
    tmc9660_tmcl_command(&tmc9660, MST, 0, 0, 0, NULL);

    LOG_INF("TMC9660 init OK");

    while(true)
    {
        pretty_error_counts(co.can_dev);

        int supply_voltage;
        tmc9660_get_param(&tmc9660, SUPPLY_VOLTAGE, &supply_voltage);
        OD_set_u16(OD_ENTRY_H2122_TMC9660SUPPLY_VOLTAGE, 0, (uint16_t) supply_voltage, false);

        if(co.CO->NMT->operatingState == CO_NMT_STOPPED) {
            cia402_set_state(&cia402, CIA402_FAULT);
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

        if(cia402.state == CIA402_NOT_READY_TO_SWITCH_ON) {
            LOG_INF("Writing TMC9660 default parameters");
            agv_init_params(&tmc9660);

            cia402_set_state(&cia402, CIA402_SWITCH_ON_DISABLED); // Transition 1
            continue;
        }

        if(cia402.state == CIA402_OPERATION_ENABLED) {
            int target_vel;
            int err = OD_get_i32(OD_ENTRY_H60FF_targetVelocity, 0, &target_vel, false);
            if(err != ODR_OK)
            {
                LOG_ERR("Could not read OD Target Velocity");
                return -1;
            }

            LOG_DBG("Setting target velocity to %d", target_vel);
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

            err = OD_set_i32(OD_ENTRY_H6064_positionActualValue, 0, actual_pos, false);
            if(err) LOG_ERR("actual_pos");
            err = OD_set_i32(OD_ENTRY_H606C_velocityActualValue, 0, actual_vel, false);
            if(err) LOG_ERR("actual_vel");
            err = OD_set_i32(OD_ENTRY_H606B_velocityDemandValue, 0, ramp_vel, false);
            if(err) LOG_ERR("ramp_vel");
            err = OD_set_i16(OD_ENTRY_H6071_targetTorque, 0, target_torque, false);
            if(err) LOG_ERR("target_torque");
            err = OD_set_u16(OD_ENTRY_H6072_maxTorque, 0, max_torque, false);
            if(err) LOG_ERR("max_torque");

            OD_requestTPDO(OD_getFlagsPDO(OD_ENTRY_H606C_velocityActualValue), 0);
            OD_requestTPDO(OD_getFlagsPDO(OD_ENTRY_H6064_positionActualValue), 0);

            if(err != ODR_OK)
            {
                LOG_ERR("Could not write TMC status to OD");
                return -1;
            }

            k_msleep(1);
        }

        /******* CONTROLWORD TRANSITIONS *******/
        uint16_t controlword;
        err = OD_get_u16(OD_ENTRY_H6040_controlword, 0, &controlword, false);
        if(err != ODR_OK) {
            LOG_ERR("Could not read controlword, %d", err);
            break;
        }

        if(controlword == 0xffff) {
            // No write
            continue;
        }

        LOG_DBG("Controlword was set to %04X", controlword);

        // Set internal controlword to 0xFFFF so we can easily detect the next write w/o a callback
        OD_set_u16(OD_ENTRY_H6040_controlword, 0, 0xFFFF, true);

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
                // XXX nextState = CIA402_QUICK_STOP_ACTIVE;
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
            } else if(nextState == CIA402_SWITCHED_ON) {
                tmc9660_set_param(&tmc9660, COMMUTATION_MODE, 5);
            } else if(nextState == CIA402_OPERATION_ENABLED) {
                
            } else if(nextState == CIA402_QUICK_STOP_ACTIVE) {
                shouldMotorStop = true;
            } else if(nextState == CIA402_FAULT_REACTION_ACTIVE) {
                shouldMotorStop = true;
            }

            if(shouldMotorStop) {
                tmc9660_tmcl_command(&tmc9660, MST, 0, 0, 0, NULL);

                // Also set OD target velocity so next time motor is enabled, it doesn't spring back to its last set velocity of a previous run
                OD_set_i32(OD_ENTRY_H60FF_targetVelocity, 0, 0, true);
            }
        }

        OD_requestTPDO(OD_getFlagsPDO(OD_ENTRY_H6041_statusword), 0);
    }

    return 0;

fail:
    return err;
}
