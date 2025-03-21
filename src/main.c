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

static struct k_thread cia402_thread_data;

K_THREAD_STACK_DEFINE(cia402_thread_stack, CONFIG_CIA402_THREAD_STACK_SIZE);

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

/* CiA402 application-specific callbacks */
void cia402_cb_initialize() {
    // tmc9660_motor_stop(&g_tmc9660);
    
    tmc9660_set_param_retry(&g_tmc9660, COMMUTATION_MODE, 0, 3);

    // Initialization is already done. If it would have failed, cia402 wouldn't
    // have been initialized in the first place.
}

void cia402_cb_start() {
    tmc9660_set_param_retry(&g_tmc9660, COMMUTATION_MODE, 5, 3);
}

bool cia402_cb_is_started() {
    // Check if ABN1 is initialized. TODO: support for non-ABN setups
    int val;
    int ret = tmc9660_get_param_retry(&g_tmc9660, ABN_1_INIT_STATE, &val, 3);
    return ret == 0 && val == 3;
}

void cia402_cb_stop() {
    // tmc9660_motor_stop(&g_tmc9660);
    tmc9660_set_param_retry(&g_tmc9660, COMMUTATION_MODE, 0, 3);

    // Set OD target velocity to 0 so next time the motor starts, it doesn't jolt back to the last seen velocity value.
    OD_set_i32(OD_ENTRY_H60FF_targetVelocity, 0, 0, false);
}

bool cia402_cb_is_stopped() {
    // Check if commutation mode is SYSTEM_OFF, seems like that's what is set after MST
    // TODO: check if logic still stands with soft stops
    int val;
    int ret = tmc9660_get_param_retry(&g_tmc9660, COMMUTATION_MODE, &val, 3);
    return ret == 0 && val == 0;
}

void cia402_cb_oops() {
    k_oops();
}

void sync_callback(struct canopen *co, bool sync) {
    if(!sync) return;
    if(cia402_current_state(&cia402) != CIA402_OPERATION_ENABLED) return;

    int target_vel;
    int err = OD_get_i32(OD_ENTRY_H60FF_targetVelocity, 0, &target_vel, false);
    if(err != ODR_OK)
    {
        LOG_ERR("Could not read OD Target Velocity");
        return;
    }

    LOG_DBG("vel = %d", target_vel);
    tmc9660_set_param(&g_tmc9660, TARGET_VELOCITY, target_vel);

    int actual_pos, actual_vel, ramp_vel, target_torque, max_torque;
    err = 0;
    err |= tmc9660_get_param(&g_tmc9660, ACTUAL_POSITION, &actual_pos);
    err |= tmc9660_get_param(&g_tmc9660, ACTUAL_VELOCITY, &actual_vel);
    err |= tmc9660_get_param(&g_tmc9660, RAMP_VELOCITY, &ramp_vel);
    err |= tmc9660_get_param(&g_tmc9660, TARGET_TORQUE, &target_torque);
    err |= tmc9660_get_param(&g_tmc9660, MAX_TORQUE, &max_torque);

    if(err) {
        LOG_ERR("Could not read TMC9660 params");
    }

    err = 0;
    err |= OD_set_i32(OD_ENTRY_H6064_positionActualValue, 0, actual_pos, false);
    err |= OD_set_i32(OD_ENTRY_H606C_velocityActualValue, 0, actual_vel, false);
    err |= OD_set_i32(OD_ENTRY_H606B_velocityDemandValue, 0, ramp_vel, false);
    err |= OD_set_i16(OD_ENTRY_H6071_targetTorque, 0, target_torque, false);
    err |= OD_set_u16(OD_ENTRY_H6072_maxTorque, 0, max_torque, false);

    if(err)
    {
        LOG_ERR("Could not write TMC9660 params to OD");
        return;
    }
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

    // Initialize canopen
    co.can_dev = can;
    co.node_id = 0x10 + dip_setting;
    LOG_INF("CANopen node ID = %d", co.node_id);
    co.bitrate = CAN_BITRATE;
    co.nmt_control = CO_NMT_STARTUP_TO_OPERATIONAL;
    co.led_callback = led_callback;
    co.sync_callback = sync_callback;
    err = canopen_init(&co);
    if(err < 0)
    {
        LOG_ERR("CANopenNode init failed");
        goto fail;
    }

    // Initialize CiA 402 
    cia402.co = &co;
    cia402.cb_initialize = cia402_cb_initialize;
    cia402.cb_start = cia402_cb_start;
    cia402.cb_is_started = cia402_cb_is_started;
    cia402.cb_stop = cia402_cb_stop;
    cia402.cb_is_stopped = cia402_cb_is_stopped;
    cia402_init(&cia402);

    // Initialize TMC9660
    err = tmc9660_init(&g_tmc9660, &spi0);
    if(err < 0)
    {
        LOG_ERR("TMC9660 init failed");
        cia402_enter_fault(&cia402);
        CO_errorReport(co.CO->em, CO_ERR_REG_GENERIC_ERR, CO_EMC_HARDWARE, 9660);
        CO_NMT_sendInternalCommand(co.CO->NMT, CO_NMT_ENTER_STOPPED);
        return -1;
    }

    // Load parameters from flash
    tmc_params_load(&g_tmc9660);

    // Make sure motor is stopped
    tmc9660_motor_stop(&g_tmc9660);

    LOG_INF("Initialization OK!");

    // Start cia402 thread
	k_tid_t cia402_tid = k_thread_create(&cia402_thread_data, cia402_thread_stack,
        K_THREAD_STACK_SIZEOF(cia402_thread_stack),
        cia402_thread_run, &cia402, NULL, NULL,
        3, 0, K_NO_WAIT);
    k_thread_name_set(cia402_tid, "canopen_cia402");

    // Miscellaneous logic
    while(true) {
        int supply_voltage;
        tmc9660_get_param(&g_tmc9660, SUPPLY_VOLTAGE, &supply_voltage);
        OD_set_u16(OD_ENTRY_H2122_TMC9660SUPPLY_VOLTAGE, 0, (uint16_t) supply_voltage, false);

        k_msleep(1000);
    }

    return 0;

fail:
    return err;
}
