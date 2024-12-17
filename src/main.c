#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/can.h>
#include <zephyr/console/console.h>
#include "tmc9660.h"
#include "nesimtit.h" // Bodged SPI

static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_PATH(leds, led_status), gpios);
static const struct gpio_dt_spec fault = GPIO_DT_SPEC_GET(DT_PATH(buttons, fault), gpios);
static const struct gpio_dt_spec btn = GPIO_DT_SPEC_GET(DT_PATH(buttons, btn), gpios);

static const struct spi_dt_spec spi0 = SPI_DT_SPEC_GET(DT_NODELABEL(tmc9660_spi), SPI_MODE_CPHA | SPI_MODE_CPOL | SPI_WORD_SET(8), 0);
static const struct device *can0 = DEVICE_DT_GET(DT_NODELABEL(can0));

#ifndef ESTI_NESIMTIT
#warning ESTI_NESIMTIT not set, SPI and CAN will likely misbehave :(
#endif

#define MOTOR QSH4218

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

	// Start FOC
	tmc9660_set_param(tmc9660, COMMUTATION_MODE, 5);
}

void agv_set_velocity(struct tmc9660_dev *tmc9660, float rpm)
{
	int internal_units = rpm * 4200000 / 180; // rough approximation!!! only valid for first motor
	tmc9660_set_param(tmc9660, TARGET_VELOCITY, internal_units);
}

void dump_flags(struct tmc9660_dev *tmc9660)
{
	int x;
	tmc9660_get_param(tmc9660, GENERAL_STATUS_FLAGS, &x);
	printf("GENERAL_STATUS_FLAGS: ");
	if(x & 1) printf("REGULATION_STOPPED ");
	if(x & 2) printf("REGULATION_TORQUE ");
	if(x & 4) printf("REGULATION_VELOCITY ");
	if(x & 8) printf("REGULATION_POSITION ");
	if(x & 16) printf("CONFIG_STORED ");
	if(x & 32) printf("CONFIG_LOADED ");
	if(x & 64) printf("CONFIG_READ_ONLY ");
	if(x & 128) printf("TMCL_SCRIPT_READ_ONLY ");
	if(x & 256) printf("BRAKE_CHOPPER_ACTIVE ");
	if(x & 512) printf("POSITION_REACHED ");
	if(x & 1024) printf("VELOCITY_REACHED ");
	if(x & 2048) printf("ADC_OFFSET_CALIBRATED ");
	if(x & 4096) printf("RAMPER_LATCHED ");
	if(x & 8192) printf("RAMPER_EVENT_STOP_SWITCH ");
	if(x & 16384) printf("RAMPER_EVENT_STOP_DEVIATION ");
	if(x & 32768) printf("RAMPER_VELOCITY_REACHED ");
	if(x & 65536) printf("RAMPER_POSITION_REACHED ");
	if(x & 131072) printf("RAMPER_SECOND_MOVE ");
	if(x & 262144) printf("IIT_1_ACTIVE ");
	if(x & 524288) printf("IIT_2_ACTIVE ");
	if(x & 1048576) printf("REFSEARCH_FINISHED ");
	if(x & 2097152) printf("Y2_USED_FOR_BRAKING ");
	if(x & 4194304) printf("FLASH_STIMULUS_AVAILABLE ");
	if(x & 8388608) printf("STEPDIR_INPUT_AVAILABLE ");
	if(x & 16777216) printf("RIGHT_REF_SWITCH_AVAILABLE ");
	if(x & 33554432) printf("HOME_REF_SWITCH_AVAILABLE ");
	if(x & 67108864) printf("LEFT_REF_SWITCH_AVAILABLE ");
	if(x & 134217728) printf("ABN2_FEEDBACK_AVAILABLE ");
	if(x & 268435456) printf("HALL_FEEDBACK_AVAILABLE ");
	if(x & 536870912) printf("ABN1_FEEDBACK_AVAILABLE ");
	if(x & 1073741824) printf("SPI_FLASH_AVAILABLE ");
	if(x & 2147483648) printf("I2C_EEPROM_AVAILABLE ");
	printf("\nGENERAL_ERROR_FLAGS: ");
	tmc9660_get_param(tmc9660, GENERAL_ERROR_FLAGS, &x);
	if(x & 1) printf("CONFIG_ERROR ");
	if(x & 2) printf("TMCL_SCRIPT_ERROR ");
	if(x & 4) printf("HOMESWITCH_NOT_FOUND ");
	if(x & 32) printf("HALL_ERROR ");
	if(x & 512) printf("WATCHDOG_EVENT ");
	if(x & 8192) printf("EXT_TEMP_EXCEEDED ");
	if(x & 16384) printf("CHIP_TEMP_EXCEEDED ");
	if(x & 65536) printf("ITT_1_EXCEEDED ");
	if(x & 131072) printf("ITT_2_EXCEEDED ");
	if(x & 262144) printf("EXT_TEMP_WARNING ");
	if(x & 524288) printf("SUPPLY_OVERVOLTAGE_WARNING ");
	if(x & 1048576) printf("SUPPLY_UNDERVOLTAGE_WARNING ");
	if(x & 2097152) printf("ADC_IN_OVERVOLTAGE ");
	if(x & 4194304) printf("FAULT_RETRY_HAPPEND ");
	if(x & 8388608) printf("FAULT_RETRIES_FAILED ");
	if(x & 16777216) printf("CHIP_TEMP_WARNING ");
	if(x & 67108864) printf("HEARTBEAT_STOPPED ");
	printf("\n");
}

// 2024-12-12
// fault pin can be pulled down by both the TMC and MAX. We normally use it
// to watch if the TMC is doing alright, but can also use it to signal, for
// CanOpen CiA 303-3 (indicator LED) compliance.
void demo_blink_fault() 
{
	gpio_pin_configure_dt(&fault, GPIO_OUTPUT);
	
	for(int i = 0; i < 10; i++)
	{
		gpio_pin_set_dt(&fault, 1);
		k_msleep(100);
		gpio_pin_set_dt(&fault, 0);
		k_msleep(100);
	}

	gpio_pin_configure_dt(&fault, GPIO_INPUT);
}

CAN_MSGQ_DEFINE(can_rx_msgq, 16); // Use a message queue for incoming CAN frames

// 2024-12-11 using bodged CAN, 2024-12-17 using legit CAN
// When button is pressed, send a CAN message to all other connected devices to
// blink NODE_ID times (NODE_ID defined in nesimtit.c)
void demo_can_blinky() 
{
	// Initialize CAN
	const struct can_filter filter_all_standard = {
		.id = 0,
		.mask = 0,
		.flags = 0
	};
	const struct can_filter filter_all_extended = {
		.id = 0,
		.mask = 0,
		.flags = CAN_FILTER_IDE
	};
	
	// Add all received standard and extended messages to the msgq
	can_add_rx_filter_msgq(can0, &can_rx_msgq, &filter_all_standard);
	can_add_rx_filter_msgq(can0, &can_rx_msgq, &filter_all_extended);

	can_start(can0); // CAN Bitrate and timing are automatically set based on device tree

	uint8_t count = 0;
	while(1)
	{
		struct can_frame frame = { 0 }; // Used for both TX and RX

		if(gpio_pin_get_dt(&btn))
		{
			gpio_pin_set_dt(&led, 1);
			printf("pressed!\n");

			frame.id = 5; // Change this for different nodes!
			frame.dlc = can_bytes_to_dlc(2);
			frame.data[0] = count++;
			frame.data[1] = 1;
			can_send(can0, &frame, K_FOREVER, NULL, NULL);
			printf("sent!\n");

			while(gpio_pin_get_dt(&btn));

			gpio_pin_set_dt(&led, 0);
			printf("released!\n");
		}

		else if(k_msgq_get(&can_rx_msgq, &frame, K_NO_WAIT) == 0)
		{
			int len = can_dlc_to_bytes(frame.dlc);
			printf("got %d bytes from %d:", len, frame.id);
			for(int i = 0; i < len; i++)
			{
				printf(" %hhx", frame.data[i]);
			}
			printf("\n");

			for(int i = 0; i < frame.id; i++)
			{
				gpio_pin_set_dt(&led, 1);
				k_msleep(200);
				gpio_pin_set_dt(&led, 0);
				k_msleep(200);
			}
		}

		else {
			static int k = 0;
			if(k++ % 100000 == 0) { // heartbeat
				gpio_pin_set_dt(&led, 1);
				k_msleep(10);
				gpio_pin_set_dt(&led, 0);
			}
		}
	}
}

// 2024-11-05
// Initializes TMC and sends some fixed velocity commands
// For some reason (likely insufficient initialization), speeds over 100ish RPM
// go very quickly from 100-200mA to 6A draw and the motor locks up. This has
// confirmed our MOSFETS are well sized (they barely even heat up), but
// indicates we still don't fully understand which params must be written for
// proper fumctioning.
// I intentianally omitted some parameters which seemed to only refer to 3phase
// operation, such as the overcurrent protection ones, but still, I don't see
// why overcurrent *protection* would affect the control algorithm while well
// within its limits.
void demo_tmc_spi(struct tmc9660_dev *tmc9660)
{
	printf("agv_init_params...\n");
	agv_init_params(tmc9660);
	printf("start commutation...\n");
	for(int i = 0; i < 50; i++)
	{
		dump_flags(tmc9660);
	}

	int ret;

	float rpms[] = {
		0, 30, 60, 100, 180, -180, 180, 0
	};

	for(int i = 0; i < sizeof(rpms)/sizeof(rpms[0]); i++)
	{
		float rpm = rpms[i];
		printf("Setting speed to %.1f RPM\n", rpm);
		agv_set_velocity(tmc9660, rpm);

		// Log everything until it dies
		for(int j = 0; j < 1000; j++)
		{
			int temp, volt, curr;

			ret = tmc9660_get_param(tmc9660, CHIP_TEMPERATURE, &temp);
			if(ret < 0)
			{
				printf("Error: GAP CHIP_TEMPERATURE (%d: %s)", ret, strerror(ret));
			}

			ret = tmc9660_get_param(tmc9660, SUPPLY_VOLTAGE, &volt);
			if(ret < 0)
			{
				printf("Error: GAP SUPPLY_VOLTAGE (%d: %s)", ret, strerror(ret));
			}

			ret = tmc9660_get_param(tmc9660, ACTUAL_TOTAL_MOTOR_CURRENT, &curr);
			if(ret < 0)
			{
				printf("Error: GAP ACTUAL_TOTAL_MOTOR_CURRENT (%d: %s)", ret, strerror(ret));
			}

			printf("(% 3d) T = %.1f\tV = %.1f\tI = %d mA\n", j, (double)(temp * 0.01615 - 268.15), (double)(volt * 0.1), curr);
			k_msleep(1);
		}
	}

	while(1)
	{
		int temp, volt, curr;

		ret = tmc9660_get_param(tmc9660, CHIP_TEMPERATURE, &temp);
		if(ret < 0)
		{
			printf("Error: GAP CHIP_TEMPERATURE (%d: %s)", ret, strerror(ret));
		}

		ret = tmc9660_get_param(tmc9660, SUPPLY_VOLTAGE, &volt);
		if(ret < 0)
		{
			printf("Error: GAP SUPPLY_VOLTAGE (%d: %s)", ret, strerror(ret));
		}

		ret = tmc9660_get_param(tmc9660, ACTUAL_TOTAL_MOTOR_CURRENT, &curr);
		if(ret < 0)
		{
			printf("Error: GAP ACTUAL_TOTAL_MOTOR_CURRENT (%d: %s)", ret, strerror(ret));
		}

		printf("T = %.1f\tV = %.1f\tI = %d mA\n", (double)(temp * 0.01615 - 268.15), (double)(volt * 0.1), curr);
	}
	
	while(0)
	{
		int speed = getchar(); // FIXME: Doesn't work, returns zeroes without blocking. Check out console_getchar?
		if(speed < '0' || speed > '9') {
			printf("%d\n", speed);
			k_msleep(1);
			continue;
		}

		int rpm = (speed - '0') * 18;
		printf("%d RPM\n", rpm);
		agv_set_velocity(tmc9660, rpm);
	}
}

// 2024-11-29
// Blink the two status LEDs, one through a GPIO pin of the MAX, and one through
// the GPIO pin of the TMC, through SIO (Set gpIO) TMCL commands.
void demo_blink_tmc_gpio(struct tmc9660_dev *tmc9660) 
{
	int i = 0;

	while(1) {
		int temp;
		int ret = tmc9660_get_param(tmc9660, CHIP_TEMPERATURE, &temp);
		if(ret < 0) {
			printf("ERR tmc9660_get_param = %d\n", ret);
		}
		printf("Chip temperature: %d\n\n", temp);

		gpio_pin_toggle_dt(&led);

		k_msleep(10);

		i++;
		ret = tmc9660_set_gpio(tmc9660, 16, i%2);
		if(ret < 0) {
			printf("ERR tmc9660_set_gpio = %d\n", ret);
		}

		k_msleep(10);
	}
}

// 2024-12-16
// Control TMC with human readable UART commands
void demo_uart_control(struct tmc9660_dev *tmc9660)
{
	console_getline_init();
	
	printf("Initialising TMC...\n");
	agv_init_params(tmc9660);

	printf("Commands:\n"
	       "E - Emergency stop\n"
		   "V <+-rpm> - Set velocity\n"
		   "P <pos> - Set position (absolute)\n"
		   "P <+-pos> - Set position (relative to current position)\n"
		   "T <+-current> - Set torque\n\n");

	char *line;
	while(line = console_getline())
	{
		switch(line[0])
		{
		default:
			printf("Invalid command\n");
			break;
		case 0:
			break;
		case 'E':
		case 'e':
			tmc9660_motor_stop(tmc9660);
			printf("Stopped motors! (Board has to be reset for TMC to be put in drive mode again)\n");
			break;
		case 'V':
		case 'v':
			int rpm;
			sscanf(line, "%*c %d", &rpm);
			agv_set_velocity(tmc9660, rpm);
			printf("Set velocity to %d RPM (approx)\n", rpm);
			break;
		case 'P':
		case 'p':
			int pos;
			sscanf(line, "%*c %d", &pos);
			if(line[2] == '+' || line[2] == '-')
			{
				int actual_pos;
				tmc9660_get_param(tmc9660, ACTUAL_POSITION, &actual_pos);
				pos += actual_pos;
			}
			tmc9660_set_param(tmc9660, TARGET_POSITION, pos);
			printf("Set position to %d position units\n", pos);
			break;
		case 'T':
		case 't':
			int tor;
			sscanf(line, "%*c %d", &tor);
			tmc9660_set_param(tmc9660, TARGET_TORQUE, tor);
			printf("Set torque to %d mA\n", tor);
			break;
		}

		int pos, vel, tor, temp, current;
		tmc9660_get_param(tmc9660, ACTUAL_POSITION, &pos);
		tmc9660_get_param(tmc9660, ACTUAL_VELOCITY, &vel);
		tmc9660_get_param(tmc9660, ACTUAL_TORQUE, &tor);
		tmc9660_get_param(tmc9660, CHIP_TEMPERATURE, &temp);
		tmc9660_get_param(tmc9660, ACTUAL_TOTAL_MOTOR_CURRENT, &current);

		printf("pos = %d\tvel = %d\ttor = %d\ttemp = %d\tcurrend = %d\n", pos, vel, tor, temp, current);
	}
}

int main(void)
{
	gpio_pin_configure_dt(&led, GPIO_OUTPUT);
	gpio_pin_configure_dt(&fault, GPIO_INPUT);
	gpio_pin_configure_dt(&btn, GPIO_INPUT);

	// There are a few milliseconds between when the LDOs are up (= zephyr boot)
	// and when the parameter mode application inside the TMC is fully
	// initialized, time in which the TMC is configured to assert the fault
	// signal (may be disabled via the bootstrapping settings). This waits for
	// the TMC to initialize:
	while(gpio_pin_get_dt(&fault) == 1)
	{
		printf(".");
	}
	printf("\nHello World! %s\n", CONFIG_BOARD_TARGET);

	nesimtit_init(); // Init bodged SPI
	
	struct tmc9660_dev tmc9660;
	tmc9660_init(&tmc9660, &spi0);

	demo_blink_fault();
	// demo_uart_control(&tmc9660);
	demo_can_blinky();
	// demo_blink_tmc_gpio(&tmc9660);
	// demo_tmc_spi(&tmc9660);

	return 0;
}
