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

	struct tmc9660_dev tmc9660 = { 0 };

	while(1)
	{
		int command = 6;
		int type = command == 6 ? CHIP_TEMPERATURE : 0;
		uint8_t tx[8] = { command, (type & 0xff), (type&0xf00)>>4, 0, 0, 0, 0, 0 };
		tx[7] = tx[0] + tx[1] + tx[2] + tx[3] + tx[4] + tx[5] + tx[6];
		uint8_t rx[16];

		struct spi_buf tx_buf[1] = {
			{ .buf = tx, .len = 8 }
		};
		struct spi_buf_set tx_bufs = { .buffers = tx_buf, .count = 1 };
		struct spi_buf rx1_buf[1] = {
			{ .buf = rx, .len = 8 },
		};
		struct spi_buf_set rx1_bufs = { .buffers = rx1_buf, .count = 1 };
		struct spi_buf rx2_buf[1] = {
			{ .buf = rx+8, .len = 8 },
		};
		struct spi_buf_set rx2_bufs = { .buffers = rx2_buf, .count = 1 };

		// printf("\n> ");
		// for(int i = 0; i < 8; i++)
		// {
		// 	printf("%02hhx ", tx[i]);
		// }
		// printf("\n");
		spi_write_dt(&spi0, &tx_bufs);
		// printf("< ");
		// for(int i = 0; i < 8; i++)
		// {
		// 	printf("%02hhx ", rx[i]);
		// }
		// printf("\n");
		k_usleep(1);
		do {
			spi_read_dt(&spi0, &rx2_bufs);
			// printf("<                         ");
			// for(int i = 8; i < 16; i++)
			// {
			// 	printf("%02hhx ", rx[i]);
			// }
			// printf("\n");
			//printf("%02hhx\n", rx[8]);
		} while(rx[8] != 0xff);
		printf("spi: %d; tmcl: %d; CHIP_TEMPERATURE = %d", rx[8], rx[9], (rx[11] << 24) | (rx[12] << 16) | (rx[13] << 8) | rx[14]);
		if(rx[9] != 100) printf(" (TMCL error)");
		printf("\n");
	// printf("\n");
	// printf("\n");
	// printf("\n");
	}

	// tmc9660_init(&tmc9660, &spi0);

	// while(0) {
	// 	int temp;
	// 	int ret = tmc9660_get_param(&tmc9660, CHIP_TEMPERATURE, &temp);
	// 	if(ret < 0) {
	// 		printf("ERR tmc9660_get_param = %d\n", ret);
	// 	}
	// 	printf("Chip temperature: %d\n\n\n\n\n", temp);

	// 	gpio_pin_toggle_dt(&led);

	// 	k_msleep(1000);
	// }

	return 0;
}
