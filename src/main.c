#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/can.h>
#include "tmc9660.h"
#include <canopennode.h> // zephyr module specific helpers!
#include "OD.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

static const struct gpio_dt_spec led_green = GPIO_DT_SPEC_GET(DT_PATH(leds, led_status), gpios);
static const struct gpio_dt_spec led_fault = GPIO_DT_SPEC_GET(DT_PATH(buttons, fault), gpios);
// static const struct gpio_dt_spec btn = GPIO_DT_SPEC_GET(DT_PATH(buttons, btn), gpios);

static const struct spi_dt_spec spi0 = SPI_DT_SPEC_GET(DT_NODELABEL(tmc9660_spi), SPI_MODE_CPHA | SPI_MODE_CPOL | SPI_WORD_SET(8), 0);
static const struct device *can = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));

#define CAN_BITRATE (DT_PROP_OR(DT_CHOSEN(zephyr_canbus), bitrate, \
					  DT_PROP_OR(DT_CHOSEN(zephyr_canbus), bus_speed, \
						     CONFIG_CAN_DEFAULT_BITRATE)) / 1000)

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

void init_leds()
{
    int err;

    if(!gpio_is_ready_dt(&led_green))
    {
        LOG_ERR("Green LED device not ready");
        return;
    }
    
    if(!gpio_is_ready_dt(&led_fault))
    {
        LOG_ERR("Red LED device not ready");
        return;
    }

    if((err = gpio_pin_configure_dt(&led_green, GPIO_OUTPUT_INACTIVE)))
    {
        LOG_ERR("Green LED configure failed: %d", err);
    }
    
    if((err = gpio_pin_configure_dt(&led_fault, GPIO_INPUT)))
    {
        LOG_ERR("Fault LED configure failed: %d", err);
    }
}

void read_tmc_state()
{
    int actual_pos, actual_vel, ramp_vel, target_vel, target_torque, max_torque;
    tmc9660_get_param(&tmc9660, ACTUAL_POSITION, &actual_pos);
    tmc9660_get_param(&tmc9660, ACTUAL_VELOCITY, &actual_vel);
    tmc9660_get_param(&tmc9660, RAMP_VELOCITY, &ramp_vel);
    tmc9660_get_param(&tmc9660, TARGET_VELOCITY, &target_vel);
    tmc9660_get_param(&tmc9660, TARGET_TORQUE, &target_torque);
    tmc9660_get_param(&tmc9660, MAX_TORQUE, &max_torque);
}

void after_rpdo()
{
    int vel = 123;
    tmc9660_set_param(&tmc9660, TARGET_VELOCITY, vel);
}

enum cia402_state {
    START = 0,
    NOT_READY_TO_SWITCH_ON, // Doing initialization
    SWITCH_ON_DISABLED, // Motor not electrically connected + control inactive
    READY_TO_SWITCH_ON, 
    SWITCHED_ON, // Motor electrically connected + control INACTIVE
    OPERATION_ENABLE, // Motor control ACTIVE
    QUICK_STOP_ACTIVE, // Quick stop in progress
    FAULT_REACTION_ACTIVE, // ???
    FAULT, // ???
};

enum cia402_event {
    CIA402_EVT_NONE = 0,
    
    // Host commands
    CIA402_CMD_SHUTDOWN,
    CIA402_CMD_SWITCH_ON,
    CIA402_CMD_DISABLE_VOLTAGE,
    CIA402_CMD_QUICK_STOP,
    CIA402_CMD_DISABLE_OPERATION,
    CIA402_CMD_ENABLE_OPERATION,
    CIA402_CMD_FAULT_RESET,
    CIA402_CMD_HALT,

    // Emergencies
    CIA402_EMCY,

    // External
    CIA402_NMT_NOT_OPERATIONAL, // NMT state machine brought out of operational (PRE-OPERATIONAL, STOPPED, etc). Go back to initialization
};

struct cia402 {
    struct CO *co;
    enum cia402_state state;
};

void cia402_tick(struct cia402 *cia402, enum cia402_event event)
{
    enum cia402_state nextState;
    int err;

    // Check if fault
    if(event == )

    switch(cia402->state) {
default: // fall through
case START:
    nextState = NOT_READY_TO_SWITCH_ON;
    break;

case NOT_READY_TO_SWITCH_ON:
    err = initialization();
    if(!err)
    {
        nextState = SWITCH_ON_DISABLED;
        break;
    }
    else
    {
        LOG_ERR("Failed initialization!");
        nextState = FAULT;
        break;
    }

    break;

case SWITCH_ON_DISABLED:

    break;

case READY_TO_SWITCH_ON:
    break;

case SWITCHED_ON:
    break;

case OPERATION_ENABLE:
    break;

case QUICK_STOP_ACTIVE:
    break;

case FAULT_REACTION_ACTIVE:
    break;

case FAULT:
    break;

    }
}

int main()
{
	struct canopen co = {
		.can_dev = can,
		.node_id = 0x10,
		.bitrate = CAN_BITRATE,
		.nmt_control = 0, // TODO(ioan)
		.led_callback = led_callback
	};

	init_leds();
	canopen_init(&co);

    while(1)
    {
        
    }
}
