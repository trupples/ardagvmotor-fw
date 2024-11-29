#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include "tmc9660.h"

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_NODELABEL(led_ds2), gpios);
static const struct gpio_dt_spec fault = GPIO_DT_SPEC_GET(DT_NODELABEL(fault), gpios);
static const struct spi_dt_spec spi0 = SPI_DT_SPEC_GET(DT_NODELABEL(tmc9660), SPI_MODE_CPHA | SPI_MODE_CPOL | SPI_WORD_SET(8), 0);

int main(void)
{
	printf("Hello World! %s\n", CONFIG_BOARD_TARGET);

	gpio_pin_configure_dt(&led, GPIO_OUTPUT);
	gpio_pin_configure_dt(&fault, GPIO_INPUT);

	while(gpio_pin_get_dt(&fault) == 0)
	{
		printf(".");
	}
	printf("\n");

	k_msleep(1000);

	struct tmc9660_dev tmc9660;

	 tmc9660_init(&tmc9660, &spi0);

	int i = 0;

	while(1) {
		int temp;
		int ret = tmc9660_get_param(&tmc9660, CHIP_TEMPERATURE, &temp);
		if(ret < 0) {
			printf("ERR tmc9660_get_param = %d\n", ret);
		}
		printf("Chip temperature: %d\n\n", temp);

		gpio_pin_toggle_dt(&led);

		k_msleep(100);

		i++;
		tmc9660_set_gpio(&tmc9660, 16, i%2);

		k_usleep(100);
	}

	return 0;
}
