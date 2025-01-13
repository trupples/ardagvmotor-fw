#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/sys/reboot.h>
#include <canopennode.h> // zephyr module specific helpers!
#include "OD.h"

#define NMT_CONTROL CO_NMT_STARTUP_TO_OPERATIONAL | CO_NMT_ERR_ON_ERR_REG | CO_ERR_REG_GENERIC_ERR | CO_ERR_REG_COMMUNICATION
#define FIRST_HB_TIME 500

LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

#ifdef LEDS_ARE_CURRENTLY_BROKEN
static const struct gpio_dt_spec led_green = GPIO_DT_SPEC_GET(DT_PATH(leds, led_status), gpios);
static const struct gpio_dt_spec led_fault = GPIO_DT_SPEC_GET(DT_PATH(buttons, fault), gpios);
// static const struct gpio_dt_spec btn = GPIO_DT_SPEC_GET(DT_PATH(buttons, btn), gpios);
#endif

// static const struct spi_dt_spec spi0 = SPI_DT_SPEC_GET(DT_NODELABEL(tmc9660_spi), SPI_MODE_CPHA | SPI_MODE_CPOL | SPI_WORD_SET(8), 0);
static const struct device *can = DEVICE_DT_GET(DT_CHOSEN(zephyr_canbus));

#define CAN_BITRATE (DT_PROP_OR(DT_CHOSEN(zephyr_canbus), bitrate, \
					  DT_PROP_OR(DT_CHOSEN(zephyr_canbus), bus_speed, \
						     CONFIG_CAN_DEFAULT_BITRATE)) / 1000)

#ifdef LEDS_ARE_CURRENTLY_BROKEN
/* Indicator LEDs */

/* CiA 303-3 standardizes how to indicate the status of the device using a red
 * and green LED, covering blink patterns for various errors / states.
 * 
 * Requires CONFIG_CANOPENNODE_LEDS.
 */
void green_led_cb(bool state, void *arg)
{
    gpio_pin_set_dt(&led_green, state);
}

/* Red LED is shared between the TMC and MAX. To light it up,
 * its GPIO must be enabled and pulled low. To turn it off,
 * its GPIO should be floated (so that the TMC may control it).
 */
void red_led_cb(bool state, void *arg)
{
    if(state)
    {
        gpio_pin_configure_dt(&led_fault, GPIO_OUTPUT);
        gpio_pin_set_dt(&led_fault, 1);
    }
    else
    {
        gpio_pin_set_dt(&led_fault, 0);
        gpio_pin_configure_dt(&led_fault, GPIO_INPUT);
    }
}

void init_leds(CO_NMT_t *nmt)
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
    
    canopen_leds_init(nmt, green_led_cb, NULL, red_led_cb, NULL);
}
#endif

int main()
{
	struct canopen co = {
		.can_dev = can,
		.node_id = 12,
		.bitrate = 1000,
		.nmt_control = 0, // TODO(ioan)
	};

	canopen_init(&co);

	while(true)
	{
		k_sleep(K_MSEC(1));
	}
}


// CO_LED_GREEN(co.CO->LEDs, CO_LED_CANopen)
// CO_LED_RED(co.CO->LEDs, CO_LED_CANopen)