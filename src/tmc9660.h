#include <stdint.h>
#include <zephyr/drivers/spi.h>

struct tmc9660_dev {
    const struct spi_dt_spec *spi;
};

enum tmc9660_param_id {
	/* Selected motor type. PWM must be turned off to change this.
	 * Min: 0; Max: 3; Default: 0
	 * Values:
	 *   NO_MOTOR = 0
	 *   DC_MOTOR = 1
	 *   STEPPER_MOTOR = 2
	 *   BLDC_MOTOR = 3
	 */
	MOTOR_TYPE = 0,

	/* Pole pair count of the motor.
	 * Min: 0; Max: 127; Default: 1
	 */
	MOTOR_POLE_PAIRS = 1,

	/* Motor direction bit. Inverts the motor direction.
	 * Min: 0; Max: 1; Default: 0
	 * Values:
	 *   NOT_INVERTED = 0
	 *   INVERTED = 1
	 */
	MOTOR_DIRECTION = 2,

	/* Set the frequency of the motor PWM.
	 * Unit: Hz
	 * Min: 10000; Max: 100000; Default: 25000
	 */
	MOTOR_PWM_FREQUENCY = 3,

	/* Selected FOC operation mode depending on feedback used for commutation.
	 * Min: 0; Max: 8; Default: 0
	 * Values:
	 *   SYSTEM_OFF = 0
	 *   SYSTEM_OFF_LOW_SIDE_FETS_ON = 1
	 *   SYSTEM_OFF_HIGH_SIDE_FETS_ON = 2
	 *   FOC_OPENLOOP_VOLTAGE_MODE = 3
	 *   FOC_OPENLOOP_CURRENT_MODE = 4
	 *   FOC_ABN = 5
	 *   FOC_HALL_SENSOR = 6
	 *   RESERVED = 7
	 *   FOC_SPI_ENC = 8
	 */
	COMMUTATION_MODE = 4,

	/* PID UQ/UD output limit for circular limiter.
	 * Min: 0; Max: 32767; Default: 8000
	 */
	OUTPUT_VOLTAGE_LIMIT = 5,

	/* Maximum motor torque. *This value can be temporarily exceeded marginally due to the operation of the current regulator.
	 * Unit: mA
	 * Min: 0; Max: 65535; Default: 2000
	 */
	MAX_TORQUE = 6,

	/* Max. motor flux. *This value can be temporarily exceeded marginally due to the operation of the current regulator.
	 * Unit: mA
	 * Min: 0; Max: 65535; Default: 2000
	 */
	MAX_FLUX = 7,

	/* PWM switching scheme.
	 * Min: 0; Max: 2; Default: 1
	 * Values:
	 *   STANDARD = 0
	 *   SVPWM = 1
	 *   FLAT_BOTTOM = 2
	 */
	PWM_SWITCHING_SCHEME = 8,

	/* Configure if the PWM should be off (high-z) or on (all motor phases same voltage) in commutation mode "System Off".
	 * Min: 0; Max: 1; Default: 1
	 * Values:
	 *   PWM_ON_WHEN_MOTOR_IDLE = 0
	 *   PWM_OFF_WHEN_MOTOR_IDLE = 1
	 */
	IDLE_MOTOR_PWM_BEHAVIOR = 9,

	/* Shunt type used for ADC measurements.
	 * Min: 0; Max: 4; Default: 4
	 * Values:
	 *   INLINE_UVW = 0
	 *   INLINE_VW = 1
	 *   INLINE_UW = 2
	 *   INLINE_UV = 3
	 *   BOTTOM_SHUNTS = 4
	 */
	ADC_SHUNT_TYPE = 12,

	/* Raw ADC measurement of the ADC I0 shunt.
	 * Min: -32768; Max: 32767; Default: 0
	 */
	ADC_I0_RAW = 13,

	/* Raw ADC measurement of the ADC I1 shunt.
	 * Min: -32768; Max: 32767; Default: 0
	 */
	ADC_I1_RAW = 14,

	/* Raw ADC measurement of the ADC I2 shunt.
	 * Min: -32768; Max: 32767; Default: 0
	 */
	ADC_I2_RAW = 15,

	/* Raw ADC measurement of the ADC I3 shunt.
	 * Min: -32768; Max: 32767; Default: 0
	 */
	ADC_I3_RAW = 16,

	/* Current sense amplifier gain for ADC I0, I1 and I2.
	 * Min: 0; Max: 4; Default: 1
	 * Values:
	 *   GAIN_5X = 0
	 *   GAIN_10X = 1
	 *   GAIN_20X = 2
	 *   GAIN_40X = 3
	 *   GAIN_1X_BYPASS_CSA = 4
	 */
	CSA_GAIN_ADC_I0_TO_ADC_I2 = 17,

	/* Current sense amplifier gain for ADC I3.
	 * Min: 0; Max: 4; Default: 1
	 * Values:
	 *   GAIN_5X = 0
	 *   GAIN_10X = 1
	 *   GAIN_20X = 2
	 *   GAIN_40X = 3
	 *   GAIN_1X_BYPASS_CSA = 4
	 */
	CSA_GAIN_ADC_I3 = 18,

	/* Current sense amplifier filter for ADC I0, I1 and I2.
	 * Min: 0; Max: 3; Default: 0
	 * Values:
	 *   T_0_55_MICROSEC = 0
	 *   T_0_75_MICROSEC = 1
	 *   T_1_0_MICROSEC = 2
	 *   T_1_35_MICROSEC = 3
	 */
	CSA_FILTER_ADC_I0_TO_ADC_I2 = 19,

	/* Current sense amplifier filter for ADC I3.
	 * Min: 0; Max: 3; Default: 0
	 * Values:
	 *   T_0_55_MICROSEC = 0
	 *   T_0_75_MICROSEC = 1
	 *   T_1_0_MICROSEC = 2
	 *   T_1_35_MICROSEC = 3
	 */
	CSA_FILTER_ADC_I3 = 20,

	/* Current scaling factor converting internal units to real- world units.
	 * Min: 1; Max: 65535; Default: 520
	 */
	CURRENT_SCALING_FACTOR = 21,

	/* Mapping ADC to UX1.
	 * Min: 0; Max: 3; Default: 0
	 * Values:
	 *   ADC_I0 = 0
	 *   ADC_I1 = 1
	 *   ADC_I2 = 2
	 *   ADC_I3 = 3
	 */
	PHASE_UX1_ADC_MAPPING = 22,

	/* Mapping ADC to VX2.
	 * Min: 0; Max: 3; Default: 1
	 * Values:
	 *   ADC_I0 = 0
	 *   ADC_I1 = 1
	 *   ADC_I2 = 2
	 *   ADC_I3 = 3
	 */
	PHASE_VX2_ADC_MAPPING = 23,

	/* Mapping ADC to WY1.
	 * Min: 0; Max: 3; Default: 2
	 * Values:
	 *   ADC_I0 = 0
	 *   ADC_I1 = 1
	 *   ADC_I2 = 2
	 *   ADC_I3 = 3
	 */
	PHASE_WY1_ADC_MAPPING = 24,

	/* Mapping ADC to Y2.
	 * Min: 0; Max: 3; Default: 3
	 * Values:
	 *   ADC_I0 = 0
	 *   ADC_I1 = 1
	 *   ADC_I2 = 2
	 *   ADC_I3 = 3
	 */
	PHASE_Y2_ADC_MAPPING = 25,

	/* Scaling applied to ADC I0.
	 * Min: 1; Max: 32767; Default: 1024
	 */
	ADC_I0_SCALE = 26,

	/* Scaling applied to ADC I1.
	 * Min: 1; Max: 32767; Default: 1024
	 */
	ADC_I1_SCALE = 27,

	/* Scaling applied to ADC I2.
	 * Min: 1; Max: 32767; Default: 1024
	 */
	ADC_I2_SCALE = 28,

	/* Scaling applied to ADC I3.
	 * Min: 1; Max: 32767; Default: 1024
	 */
	ADC_I3_SCALE = 29,

	/* Invert the reading of ADC I0.
	 * Min: 0; Max: 1; Default: 1
	 * Values:
	 *   NOT_INVERTED = 0
	 *   INVERTED = 1
	 */
	ADC_I0_INVERTED = 30,

	/* Invert the reading of ADC I1.
	 * Min: 0; Max: 1; Default: 1
	 * Values:
	 *   NOT_INVERTED = 0
	 *   INVERTED = 1
	 */
	ADC_I1_INVERTED = 31,

	/* Invert the reading of ADC I2.
	 * Min: 0; Max: 1; Default: 1
	 * Values:
	 *   NOT_INVERTED = 0
	 *   INVERTED = 1
	 */
	ADC_I2_INVERTED = 32,

	/* Invert the reading of ADC I3.
	 * Min: 0; Max: 1; Default: 1
	 * Values:
	 *   NOT_INVERTED = 0
	 *   INVERTED = 1
	 */
	ADC_I3_INVERTED = 33,

	/* Offset applied to ADC I0 measurement.
	 * Min: -32768; Max: 32767; Default: 0
	 */
	ADC_I0_OFFSET = 34,

	/* Offset applied to ADC I1 measurement.
	 * Min: -32768; Max: 32767; Default: 0
	 */
	ADC_I1_OFFSET = 35,

	/* Offset applied to ADC I2 measurement.
	 * Min: -32768; Max: 32767; Default: 0
	 */
	ADC_I2_OFFSET = 36,

	/* Offset applied to ADC I3 measurement.
	 * Min: -32768; Max: 32767; Default: 0
	 */
	ADC_I3_OFFSET = 37,

	/* Scaled and offset compensated ADC I0 measurement.
	 * Min: -32768; Max: 32767; Default: 0
	 */
	ADC_I0 = 38,

	/* Scaled and offset compensated ADC I1 measurement.
	 * Min: -32768; Max: 32767; Default: 0
	 */
	ADC_I1 = 39,

	/* Scaled and offset compensated ADC I2 measurement.
	 * Min: -32768; Max: 32767; Default: 0
	 */
	ADC_I2 = 40,

	/* Scaled and offset compensated ADC I3 measurement.
	 * Min: -32768; Max: 32767; Default: 0
	 */
	ADC_I3 = 41,

	/* Phi_e calculated by the ramper hardware. Used for commutation in openloop modes.
	 * Min: -32768; Max: 32767; Default: 0
	 */
	OPENLOOP_ANGLE = 45,

	/* Openloop current applied in openloop, current mode.
	 * Unit: mA
	 * Min: 0; Max: 65535; Default: 1000
	 */
	OPENLOOP_CURRENT = 46,

	/* Openloop voltage applied in openloop, voltage mode.
	 * Min: 0; Max: 16383; Default: 0
	 */
	OPENLOOP_VOLTAGE = 47,

	/* Gain applied to acceleration feedforward.
	 * Min: 0; Max: 65535; Default: 8
	 */
	ACCELERATION_FF_GAIN = 50,

	/* Shift applied to acceleration feedforward.
	 * Min: 0; Max: 6; Default: 4
	 * Values:
	 *   NO_SHIFT = 0
	 *   SHIFT_4_BIT = 1
	 *   SHIFT_8_BIT = 2
	 *   SHIFT_12_BIT = 3
	 *   SHIFT_16_BIT = 4
	 *   SHIFT_20_BIT = 5
	 *   SHIFT_24_BIT = 6
	 */
	ACCELERATION_FF_SHIFT = 51,

	/* Enable the application of acceleration and deceleration ramps.
	 * Min: 0; Max: 1; Default: 0
	 * Values:
	 *   DISABLED = 0
	 *   ENABLED = 1
	 */
	RAMP_ENABLE = 52,

	/* Specify the control loop structure for velocity mode. Directly regulating the velocity or regulating on a constantly calculated target position.
	 * Min: 0; Max: 1; Default: 1
	 * Values:
	 *   DISABLED = 0
	 *   ENABLED = 1
	 */
	DIRECT_VELOCITY_MODE = 53,

	/* Acceleration in top part of eight-point ramp.
	 * Unit: internal
	 * Min: 1; Max: 8388607; Default: 1000
	 */
	RAMP_AMAX = 54,

	/* First acceleration in eight-point ramp.
	 * Unit: internal
	 * Min: 1; Max: 8388607; Default: 8000
	 */
	RAMP_A1 = 55,

	/* Second acceleration in eight-point ramp.
	 * Unit: internal
	 * Min: 1; Max: 8388607; Default: 4000
	 */
	RAMP_A2 = 56,

	/* Deceleration in top part of eight-point ramp.
	 * Unit: internal
	 * Min: 1; Max: 8388607; Default: 1000
	 */
	RAMP_DMAX = 57,

	/* Second deceleration in eight-point ramp.
	 * Unit: internal
	 * Min: 1; Max: 8388607; Default: 8000
	 */
	RAMP_D1 = 58,

	/* First deceleration in eight-point ramp.
	 * Unit: internal
	 * Min: 1; Max: 8388607; Default: 8000
	 */
	RAMP_D2 = 59,

	/* Maximum velocity of eight-point ramp.
	 * Unit: internal
	 * Min: 0; Max: 134217727; Default: 134217727
	 */
	RAMP_VMAX = 60,

	/* Velocity threshold to switch from A1/D1 to A2/D2.
	 * Unit: internal
	 * Min: 0; Max: 134217727; Default: 0
	 */
	RAMP_V1 = 61,

	/* Velocity threshold to switch from A2/D2 to AMAX/DMAX.
	 * Unit: internal
	 * Min: 0; Max: 134217727; Default: 0
	 */
	RAMP_V2 = 62,

	/* Start velocity of ramp.
	 * Unit: internal
	 * Min: 0; Max: 8388607; Default: 0
	 */
	RAMP_VSTART = 63,

	/* Stop velocity of ramp. Needs to be greater than 0.
	 * Unit: internal
	 * Min: 1; Max: 8388607; Default: 1
	 */
	RAMP_VSTOP = 64,

	/* Minimum time at VMAX to start deceleration.
	 * Unit: internal
	 * Min: 0; Max: 65535; Default: 0
	 */
	RAMP_TVMAX = 65,

	/* Wait time at end of ramp to signal stop.
	 * Unit: internal
	 * Min: 0; Max: 65535; Default: 0
	 */
	RAMP_TZEROWAIT = 66,

	/* Enable the acceleration feedforward feature.
	 * Min: 0; Max: 1; Default: 0
	 * Values:
	 *   DISABLED = 0
	 *   ENABLED = 1
	 */
	ACCELERATION_FEEDFORWARD_ENABLE = 67,

	/* Enable the velocity feedforward feature.
	 * Min: 0; Max: 1; Default: 0
	 * Values:
	 *   DISABLED = 0
	 *   ENABLED = 1
	 */
	VELOCITY_FEEDFORWARD_ENABLE = 68,

	/* Target velocity calculated by ramp controller.
	 * Min: -134217727; Max: 134217727; Default: 0
	 */
	RAMP_VELOCITY = 69,

	/* Target position calculated by ramp controller.
	 * Min: -2147483648; Max: 2147483647; Default: 0
	 */
	RAMP_POSITION = 70,

	/* Phi_e calculated from hall feedback.
	 * Min: -32768; Max: 32767; Default: 0
	 */
	HALL_PHI_E = 74,

	/* Hall sensor 60-degree/sector offset composed of 120 offset (order) and 180 degree offset (polarity).
	 * Min: 0; Max: 5; Default: 0
	 * Values:
	 *   DEG_0 = 0
	 *   DEG_60 = 1
	 *   DEG_120 = 2
	 *   DEG_180 = 3
	 *   DEG_240 = 4
	 *   DEG_300 = 5
	 */
	HALL_SECTOR_OFFSET = 75,

	/* Filter length of the hall sensor input signal filters.
	 * Min: 0; Max: 255; Default: 0
	 */
	HALL_FILTER_LENGTH = 76,

	/* Hall offset compensation for 0 degree hall position.
	 * Min: -32768; Max: 32767; Default: 0
	 */
	HALL_POSITION_0_OFFSET = 77,

	/* Hall offset compensation for 60 degree hall position.
	 * Min: -32768; Max: 32767; Default: 10922
	 */
	HALL_POSITION_60_OFFSET = 78,

	/* Hall offset compensation for 120 degree hall position.
	 * Min: -32768; Max: 32767; Default: 21845
	 */
	HALL_POSITION_120_OFFSET = 79,

	/* Hall offset compensation for 180 degree hall position.
	 * Min: -32768; Max: 32767; Default: -32768
	 */
	HALL_POSITION_180_OFFSET = 80,

	/* Hall offset compensation for 240 degree hall position.
	 * Min: -32768; Max: 32767; Default: -21846
	 */
	HALL_POSITION_240_OFFSET = 81,

	/* Hall offset compensation for 300 degree hall position.
	 * Min: -32768; Max: 32767; Default: -10923
	 */
	HALL_POSITION_300_OFFSET = 82,

	/* Invert the hall angle direction.
	 * Min: 0; Max: 1; Default: 0
	 * Values:
	 *   NOT_INVERTED = 0
	 *   INVERTED = 1
	 */
	HALL_INVERT_DIRECTION = 83,

	/* Enable the hall extrapolation to generate a higher resolution position signal. The extrapolation is only active at speeds higher than 60 rpm.
	 * Min: 0; Max: 1; Default: 0
	 * Values:
	 *   DISABLED = 0
	 *   ENABLED = 1
	 */
	HALL_EXTRAPOLATION_ENABLE = 84,

	/* Use this parameter to compensate hall sensor mounting tolerances.
	 * Min: -32768; Max: 32767; Default: 0
	 */
	HALL_PHI_E_OFFSET = 85,

	/* Phi_e calculated from abn feedback.
	 * Min: -32768; Max: 32767; Default: 0
	 */
	ABN_1_PHI_E = 89,

	/* ABN 1 encoder steps per rotation (CPR).
	 * Min: 0; Max: 16777215; Default: 65536
	 */
	ABN_1_STEPS = 90,

	/* ABN 1 encoder rotation direction.
	 * Min: 0; Max: 1; Default: 0
	 * Values:
	 *   NOT_INVERTED = 0
	 *   INVERTED = 1
	 */
	ABN_1_DIRECTION = 91,

	/* Select an ABN encoder initialization method that fits best to your motor's sensors. Forces the rotor into phi_e zero using the open loop current but actively swings the rotor. Forces the rotor into phi_e 90 degree position and then into zero position using the open loop current. Turns the motor slightly in hall commutation mode until a hall signal change gives a new absolut position, which then the ABN phi_e is aligned to. Turns the motor slightly in open loop commutation mode until a N-channel is reached and gives an absolut position, which then the ABN phi_e is aligned to.
	 * Min: 0; Max: 3; Default: 0
	 * Values:
	 *   FORCED_PHI_E_ZERO_WITH_ACTIVE_SWING = 0
	 *   FORCED_PHI_E_90_ZERO = 1
	 *   USE_HALL = 2
	 *   USE_N_CHANNEL_OFFSET = 3
	 */
	ABN_1_INIT_METHOD = 92,

	/* Actual state of ABN encoder initialization.
	 * Min: 0; Max: 3; Default: 0
	 * Values:
	 *   IDLE = 0
	 *   BUSY = 1
	 *   WAIT = 2
	 *   DONE = 3
	 */
	ABN_1_INIT_STATE = 93,

	/* When one of the "Forced phi_e" initialization methods is used, this value defines the wait time until the phi_e ABN angle is set to zero. This parameter should be set in a way, that the motor has stopped mechanical oscillations after the specified time.
	 * Unit: ms
	 * Min: 1000; Max: 10000; Default: 1000
	 */
	ABN_1_INIT_DELAY = 94,

	/* Init velocity for ABN encoder initialization with N- channel offset.
	 * Min: -200000; Max: 200000; Default: 5
	 */
	ABN_1_INIT_VELOCITY = 95,

	/* Offset between phi_e zero and the ABN encoders index pulse position. This value is updated asynchronously on any ABN initialization other than the "Use-N channel offset" method. The value can then be used for the "Use N-channel offset" based initialization.
	 * Min: -32768; Max: 32767; Default: 0
	 */
	ABN_1_N_CHANNEL_PHI_E_OFFSET = 96,

	/* ABN 1 encoder N-channel is inverted.
	 * Min: 0; Max: 1; Default: 0
	 * Values:
	 *   ACTIVE_HIGH = 0
	 *   ACTIVE_LOW = 1
	 */
	ABN_1_N_CHANNEL_INVERTED = 97,

	/* ABN 1 encoder N-channel filtering. Useful for imprecise encoders with the index pulses lasting multiple A/B steps.
	 * Min: 0; Max: 4; Default: 0
	 * Values:
	 *   FILTERING_OFF = 0
	 *   N_EVENT_ON_A_HIGH_B_HIGH = 1
	 *   N_EVENT_ON_A_HIGH_B_LOW = 2
	 *   N_EVENT_ON_A_LOW_B_HIGH = 3
	 *   N_EVENT_ON_A_LOW_B_LOW = 4
	 */
	ABN_1_N_CHANNEL_FILTERING = 98,

	/* Clear the actual position on the next ABN 1 encoder N- channel event.
	 * Min: 0; Max: 1; Default: 0
	 * Values:
	 *   DISABLED = 0
	 *   ENABLED = 1
	 */
	ABN_1_CLEAR_ON_NEXT_NULL = 99,

	/* Raw ABN encoder internal counter value.
	 * Min: 0; Max: 16777215; Default: 0
	 */
	ABN_1_VALUE = 100,

	/* Target torque value. Write to activate torque regulation.
	 * Unit: mA
	 * Min: -32768; Max: 32767; Default: 0
	 */
	TARGET_TORQUE = 104,

	/* Actual motor torque value.
	 * Unit: mA
	 * Min: -32767; Max: 32768; Default: 0
	 */
	ACTUAL_TORQUE = 105,

	/* Target flux value.
	 * Unit: mA
	 * Min: -10000; Max: 10000; Default: 0
	 */
	TARGET_FLUX = 106,

	/* Actual motor flux value.
	 * Unit: mA
	 * Min: -2147483648; Max: 2147483647; Default: 0
	 */
	ACTUAL_FLUX = 107,

	/* Offset applied to torque value.
	 * Unit: mA] (peak)
	 * Min: -4700; Max: 4700; Default: 0
	 */
	TORQUE_OFFSET = 108,

	/* P parameter for torque PI regulator. Also controls flux P parameter unless separate torque/flux loops are enabled.
	 * Min: 0; Max: 32767; Default: 50
	 */
	TORQUE_P = 109,

	/* I parameter for torque PI regulator. Also controls flux I parameter unless separate torque/flux loops are enabled.
	 * Min: 0; Max: 32767; Default: 100
	 */
	TORQUE_I = 110,

	/* P parameter for flux PI regulator. Only available when separated torque/flux loops are enabled.
	 * Min: 0; Max: 32767; Default: 50
	 */
	FLUX_P = 111,

	/* I parameter for flux PI regulator. Only available when separated torque/flux loops are enabled.
	 * Min: 0; Max: 32767; Default: 100
	 */
	FLUX_I = 112,

	/* Enable to configure separate PI values for the torque and flux current control loops.
	 * Min: 0; Max: 1; Default: 0
	 * Values:
	 *   TORQUE_FLUX_PI_COMBINED = 0
	 *   TORQUE_FLUX_PI_SEPARATED = 1
	 */
	SEPARATE_TORQUE_FLUX_PI_PARAMTERS = 113,

	/* P parameter normalization format for current PI regulator.
	 * Min: 0; Max: 1; Default: 0
	 * Values:
	 *   SHIFT_8_BIT = 0
	 *   SHIFT_16_BIT = 1
	 */
	CURRENT_NORM_P = 114,

	/* I parameter normalization format for current PI regulator.
	 * Min: 0; Max: 1; Default: 1
	 * Values:
	 *   SHIFT_8_BIT = 0
	 *   SHIFT_16_BIT = 1
	 */
	CURRENT_NORM_I = 115,

	/* Torque PI regulator error.
	 * Min: -2147483648; Max: 2147483647; Default: 0
	 */
	TORQUE_PI_ERROR = 116,

	/* Flux PI regulator error.
	 * Min: -2147483648; Max: 2147483647; Default: 0
	 */
	FLUX_PI_ERROR = 117,

	/* Integrated error of torque PI regulator.
	 * Min: -2147483648; Max: 2147483647; Default: 0
	 */
	TORQUE_PI_INTEGRATOR = 118,

	/* Integrated error of flux PI regulator.
	 * Min: -2147483648; Max: 2147483647; Default: 0
	 */
	FLUX_PI_INTEGRATOR = 119,

	/* Offset applied to flux value.
	 * Unit: mA] (peak)
	 * Min: -4700; Max: 4700; Default: 0
	 */
	FLUX_OFFSET = 120,

	/* Feedback source for the velocity PI regulator.
	 * Min: 0; Max: 4; Default: 0
	 * Values:
	 *   SAME_AS_COMMUTATION = 0
	 *   DIGITAL_HALL = 1
	 *   ABN1_ENCODER = 2
	 *   ABN2_ENCODER = 3
	 *   SPI_ENCODER = 4
	 */
	VELOCITY_SENSOR_SELECTION = 123,

	/* Target velocity value. Write to activate velocity regulation.
	 * Min: -134217728; Max: 134217727; Default: 0
	 */
	TARGET_VELOCITY = 124,

	/* Actual velocity value.
	 * Min: -2147483648; Max: 2147483647; Default: 0
	 */
	ACTUAL_VELOCITY = 125,

	/* Offset applied to velocity value.
	 * Unit: rpm
	 * Min: -200000; Max: 200000; Default: 0
	 */
	VELOCITY_OFFSET = 126,

	/* P parameter for velocity PI regulator.
	 * Min: 0; Max: 32767; Default: 800
	 */
	VELOCITY_P = 127,

	/* I parameter for velocity PI regulator.
	 * Min: 0; Max: 32767; Default: 1
	 */
	VELOCITY_I = 128,

	/* P parameter normalization format for velocity PI regulator.
	 * Min: 0; Max: 3; Default: 2
	 * Values:
	 *   NO_SHIFT = 0
	 *   SHIFT_8_BIT = 1
	 *   SHIFT_16_BIT = 2
	 *   SHIFT_24_BIT = 3
	 */
	VELOCITY_NORM_P = 129,

	/* I parameter normalization format for velocity PI regulator.
	 * Min: 0; Max: 3; Default: 2
	 * Values:
	 *   SHIFT_8_BIT = 0
	 *   SHIFT_16_BIT = 1
	 *   SHIFT_24_BIT = 2
	 *   SHIFT_32_BIT = 3
	 */
	VELOCITY_NORM_I = 130,

	/* Integrated error of velocity PI regulator.
	 * Min: -2147483648; Max: 2147483647; Default: 0
	 */
	VELOCITY_PI_INTEGRATOR = 131,

	/* Velocity PI regulator error.
	 * Min: -2147483648; Max: 2147483647; Default: 0
	 */
	VELOCITY_PI_ERROR = 132,

	/* Scaling factor to convert internal velocity to real-world units.
	 * Min: 1; Max: 2047; Default: 1
	 */
	VELOCITY_SCALING_FACTOR = 133,

	/* Maximum of velocity deviation tolerated before stop event is triggered (if activated).
	 * Min: 0; Max: 200000; Default: 0
	 */
	STOP_ON_VELOCITY_DEVIATION = 134,

	/* Downsampling factor for velocity controller.
	 * Min: 0; Max: 127; Default: 5
	 */
	VELOCITY_LOOP_DOWNSAMPLING = 135,

	/* Deviation between target and actual velocity below which the velocity reached flag goes active and latches. If a new target velocity is set the flag is reset.
	 * Min: 0; Max: 2000000000; Default: 0
	 */
	VELOCITY_REACHED_THRESHOLD = 136,

	/* Velocity threshold switching from period to frequency velocity meter.
	 * Min: 0; Max: 134217727; Default: 2000
	 */
	VELOCITY_METER_SWITCH_THRESHOLD = 137,

	/* Velocity hysteresis for switching back from frequency to period velocity meter.
	 * Min: 0; Max: 65535; Default: 500
	 */
	VELOCITY_METER_SWITCH_HYSTERESIS = 138,

	/* Currently used velocity meter mode. Measurement of velocity by time measurement between position changes. Velocity Meter running at PWM frequency. Calculates the velocity using the difference of the angle in one clock cycle. Measurement of velocity by software.
	 * Min: 0; Max: 2; Default: 0
	 * Values:
	 *   PERIOD_METER = 0
	 *   FREQUENCY_METER = 1
	 *   SOFTWARE_METER = 2
	 */
	VELOCITY_METER_MODE = 139,

	/* Feedback source for the position PI regulator.
	 * Min: 0; Max: 4; Default: 0
	 * Values:
	 *   SAME_AS_COMMUTATION = 0
	 *   DIGITAL_HALL = 1
	 *   ABN1_ENCODER = 2
	 *   ABN2_ENCODER = 3
	 *   SPI_ENCODER = 4
	 */
	POSITION_SENSOR_SELECTION = 142,

	/* Target position value. Write to activate position regulation.
	 * Min: -2147483648; Max: 2147483647; Default: 0
	 */
	TARGET_POSITION = 143,

	/* Actual position value.
	 * Min: -2147483648; Max: 2147483647; Default: 0
	 */
	ACTUAL_POSITION = 144,

	/* Scaling factor to convert internal position to real-world units.
	 * Min: 1024; Max: 65535; Default: 1024
	 */
	POSITION_SCALING_FACTOR = 145,

	/* P parameter for position PI regulator.
	 * Min: 0; Max: 32767; Default: 5
	 */
	POSITION_P = 146,

	/* I parameter for position PI regulator.
	 * Min: 0; Max: 32767; Default: 0
	 */
	POSITION_I = 147,

	/* P parameter normalization format for position PI regulator.
	 * Min: 0; Max: 3; Default: 1
	 * Values:
	 *   NO_SHIFT = 0
	 *   SHIFT_8_BIT = 1
	 *   SHIFT_16_BIT = 2
	 *   SHIFT_24_BIT = 3
	 */
	POSITION_NORM_P = 148,

	/* I parameter normalization format for position PI regulator.
	 * Min: 0; Max: 3; Default: 1
	 * Values:
	 *   SHIFT_8_BIT = 0
	 *   SHIFT_16_BIT = 1
	 *   SHIFT_24_BIT = 2
	 *   SHIFT_32_BIT = 3
	 */
	POSITION_NORM_I = 149,

	/* Integrated error of position PI regulator.
	 * Min: -2147483648; Max: 2147483647; Default: 0
	 */
	POSITION_PI_INTEGRATOR = 150,

	/* Error of position PI regulator.
	 * Min: -2147483648; Max: 2147483647; Default: 0
	 */
	POSITION_PI_ERROR = 151,

	/* Maximum of position deviation tolerated before stop event is triggered (if activated).
	 * Min: 0; Max: 2147483647; Default: 0
	 */
	STOP_ON_POSITION_DEVIATION = 152,

	/* Downsampling factor for position controller.
	 * Min: 0; Max: 127; Default: 0
	 */
	POSITION_LOOP_DOWNSAMPLING = 153,

	/* Position switch latched.
	 * Min: -2147483648; Max: 2147483647; Default: 0
	 */
	LATCH_POSITION = 154,

	/* Position limit low.
	 * Min: -2147483648; Max: 2147483647; Default: -2147483648
	 */
	POSITION_LIMIT_LOW = 155,

	/* Position limit high.
	 * Min: -2147483648; Max: 2147483647; Default: 2147483647
	 */
	POSITION_LIMIT_HIGH = 156,

	/* Deviation between target and actual position below which the position reached flag goes active and latches. If a new target position is set the flag is reset.
	 * Min: 0; Max: 2000000000; Default: 0
	 */
	POSITION_REACHED_THRESHOLD = 157,

	/* Bitwise enable for stopping when reference switch input is triggered.
	 * Min: 0; Max: 7; Default: 0
	 * Values:
	 *   Stop on reference input home. = 4
	 *   Stop on reference input right. = 2
	 *   Stop on reference input left. = 1
	 *   NO_STOP_ON_SWITCH_TRIGGERED = 0
	 *   STOP_ON_L = 1
	 *   STOP_ON_R = 2
	 *   STOP_ON_R_AND_L = 3
	 *   STOP_ON_H = 4
	 *   STOP_ON_H_AND_L = 5
	 *   STOP_ON_H_AND_R = 6
	 *   STOP_ON_H_R_AND_L = 7
	 */
	REFERENCE_SWITCH_ENABLE = 161,

	/* Bitwise configuration of reference switch configuration. Options to swap left and right input and invert switch polarity.
	 * Min: 0; Max: 15; Default: 0
	 * Values:
	 *   Swap left and right switch. = 8
	 *   Invert polarity of home switch. = 4
	 *   Invert polarity of right switch. = 2
	 *   Invert polarity of left switch. = 1
	 *   NOT_SWAPPED_NOT_INVERTED = 0
	 *   L_INVERTED = 1
	 *   R_INVERTED = 2
	 *   R_AND_L_INVERTED = 3
	 *   H_INVERTED = 4
	 *   H_AND_L_INVERTED = 5
	 *   H_AND_R_INVERTED = 6
	 *   H_R_AND_L_INVERTED = 7
	 *   L_R_SWAPPED_L_INVERTED = 8
	 *   L_R_SWAPPED_R_INVERTED = 9
	 *   L_R_SWAPPED_R_AND_L_INVERTED = 10
	 *   L_R_SWAPPED_H_INVERTED = 11
	 *   L_R_SWAPPED_H_AND_L_INVERTED = 12
	 *   L_R_SWAPPED = 13
	 *   L_R_SWAPPED_H_AND_R_INVERTED = 14
	 *   L_R_SWAPPED_H_R_AND_L_INVERTED = 15
	 */
	REFERENCE_SWITCH_POLARITY_AND_SWAP = 162,

	/* Bitwise configuration of reference switch latch configuration. Writing position to latch position parameter.
	 * Min: 0; Max: 15; Default: 0
	 * Values:
	 *   Trigger latch on falling home signal. = 8
	 *   Trigger latch on rising home signal. = 4
	 *   Trigger latch on falling left and right signal. = 2
	 *   Trigger latch on rising left and right signal = 1
	 *   NO_TRIGGER = 0
	 *   L_R_RISING_EDGE = 1
	 *   L_R_FALLING_EDGE = 2
	 *   L_R_BOTH_EDGES = 3
	 *   H_RISING_EDGE = 4
	 *   H_L_R_RISING_EDGE = 5
	 *   H_RISING_L_R_FALLING_EDGE = 6
	 *   H_RISING_L_R_BOTH_EDGES = 7
	 *   H_FALLING_EDGE = 8
	 *   H_FALLING_L_R_RISING_EDGE = 9
	 *   H_L_R_FALLING_EDGE = 10
	 *   H_FALLING_L_R_BOTH_EDGES = 11
	 *   H_BOTH_EDGES = 12
	 *   H_BOTH_L_R_RISING_EDGE = 13
	 *   H_BOTH_L_R_FALLING_EDGE = 14
	 *   H_L_R_BOTH_EDGES = 15
	 */
	REFERENCE_SWITCH_LATCH_SETTINGS = 163,

	/* Bitwise configuration of stop configuration. parameter "Stop on velocity deviation". the parameter "Stop on position deviation". condition rises. Doing a soft and not a hard stop.
	 * Min: 0; Max: 7; Default: 0
	 * Values:
	 *   Stop if the velocity loop deviation is larger then the = 4
	 *   Stop if the position loop deviation is larger then = 2
	 *   If enabled the system ramps down to zero if a stop = 1
	 *   DO_HARD_STOP = 0
	 *   DO_SOFT_STOP = 1
	 *   STOP_ON_POS_DEVIATION = 2
	 *   STOP_ON_POS_DEVIATION_SOFT_STOP = 3
	 *   STOP_ON_VEL_DEVIATION = 4
	 *   STOP_ON_VEL_DEVIATION_SOFT_STOP = 5
	 *   STOP_ON_POS_VEL_DEVIATION = 6
	 *   STOP_ON_POS_VEL_DEVIATION_SOFT_STOP = 7
	 */
	EVENT_STOP_SETTINGS = 164,

	/* Determine the reference switch search sequence. Search for left limit switch. Search for right limit switch then search for left limit switch. Search for right limit switch then approach left limit switch from both sides. Approach left limit switch from both sides. Search for home switch in negative direction, turn around if left end switch detected. Search for home switch in positive direction, turn around if right end switch detected. Search for home switch in negative direction, ignore end switch. Search for home switch in positive direction, ignore end switch.
	 * Min: 1; Max: 8; Default: 0
	 * Values:
	 *   LEFT_SWITCH = 1
	 *   RIGHT_SWITCH_LEFT_SWITCH = 2
	 *   RIGHT_SWITCH_LEFT_SWITCH_BOTH_SIDES = 3
	 *   LEFT_SWITCH_BOTH_SIDES = 4
	 *   HOME_SWITCH_NEG_DIR_LEFT_END_SWITCH = 5
	 *   HOME_SWITCH_POS_DIR_RIGHT_END_SWITCH = 6
	 *   HOME_SWITCH_NEG_DIR_IGNORE_END_SWITCH = 7
	 *   HOME_SWITCH_POS_DIR_IGNORE_END_SWITCH = 8
	 */
	REFERENCE_SWITCH_SEARCH_MODE = 165,

	/* Speed used for the reference switch search sequence.
	 * Min: -134217728; Max: 134217727; Default: 0
	 */
	REFERENCE_SWITCH_SEARCH_SPEED = 166,

	/* Lower speed used e.g. for positioning the motor at a reference switch position.
	 * Min: -134217728; Max: 134217727; Default: 0
	 */
	REFERENCE_SWITCH_SPEED = 167,

	/* Right limit switch position.
	 * Min: -2147483648; Max: 2147483647; Default: 0
	 */
	RIGHT_LIMIT_SWITCH_POSITION = 168,

	/* Home switch position.
	 * Min: -2147483648; Max: 2147483647; Default: 0
	 */
	HOME_SWITCH_POSITION = 169,

	/* Last reference position.
	 * Min: -2147483648; Max: 2147483647; Default: 0
	 */
	LAST_REFERENCE_POSITION = 170,

	/* ABN 2 encoder steps per rotation (CPR).
	 * Min: 0; Max: 16777215; Default: 1024
	 */
	ABN_2_STEPS = 174,

	/* ABN 2 encoder rotation direction.
	 * Min: 0; Max: 1; Default: 0
	 * Values:
	 *   NORMAL = 0
	 *   INVERTED = 1
	 */
	ABN_2_DIRECTION = 175,

	/* ABN 2 encoder gear ratio.
	 * Min: 1; Max: 255; Default: 1
	 */
	ABN_2_GEAR_RATIO = 176,

	/* Enable the ABN 2 encoder. Disabling resets the counted steps.
	 * Min: 0; Max: 1; Default: 0
	 * Values:
	 *   DISABLED = 0
	 *   ENABLED = 1
	 */
	ABN_2_ENABLE = 177,

	/* Raw ABN2 encoder internal counter value.
	 * Min: 0; Max: 4294967295; Default: 0
	 */
	ABN_2_VALUE = 178,

	/* Add a delay from CS going low to first SCLK edge.
	 * Unit: ns
	 * Min: 0; Max: 6375; Default: 0
	 */
	SPI_ENCODE_CS_SETTLE_DELAY_TIME = 181,

	/* Extend CS idle time between SPI message frames.
	 * Unit: us
	 * Min: 0; Max: 102; Default: 0
	 */
	SPI_ENCODER_CS_IDLE_DELAY_TIME = 182,

	/* Size of the first SPI transfer frame.
	 * Min: 1; Max: 16; Default: 1
	 */
	SPI_ENCODER_MAIN_TRANSFER_CMD_SIZE = 183,

	/* Size of the optional secondary SPI transfer frame. If set to zero, no secondary SPI transfer.
	 * Min: 0; Max: 15; Default: 0
	 */
	SPI_ENCODER_SECONDARY_TRANSFER_CMD_SIZE = 184,

	/* Used to set the transmit data and read out the received data.
	 * Min: 0; Max: 4294967295; Default: 0
	 */
	SPI_ENCODER_TRANSFER_DATA_3_0 = 185,

	/* Used to set the transmit data and read out the received data.
	 * Min: 0; Max: 4294967295; Default: 0
	 */
	SPI_ENCODER_TRANSFER_DATA_7_4 = 186,

	/* Used to set the transmit data and read out the received data.
	 * Min: 0; Max: 4294967295; Default: 0
	 */
	SPI_ENCODER_TRANSFER_DATA_11_8 = 187,

	/* Used to set the transmit data and read out the received data.
	 * Min: 0; Max: 4294967295; Default: 0
	 */
	SPI_ENCODER_TRANSFER_DATA_15_12 = 188,

	/* SPI interface setting, polarity and phase.
	 * Min: 0; Max: 2; Default: 0
	 * Values:
	 *   OFF = 0
	 *   TRIGGER_SINGLE_TRANSFER = 1
	 *   CONTINUOUS_POSITION_COUNTER_READ = 2
	 */
	SPI_ENCODER_TRANSFER = 189,

	/* Mask to be used to collect the position counter value from the continuous received data.
	 * Min: 0; Max: 4294967295; Default: 0
	 */
	SPI_ENCODER_POSITION_COUNTER_MASK = 190,

	/* Right bit shift for the position counter value before mask is applied.
	 * Min: 0; Max: 127; Default: 0
	 */
	SPI_ENCODER_POSITION_COUNTER_SHIFT = 191,

	/* Actual SPI encoder position value.
	 * Min: 0; Max: 4294967295; Default: 0
	 */
	SPI_ENCODER_POSITION_COUNTER_VALUE = 192,

	/* Actual absolute encoder angle value.
	 * Min: -32768; Max: 32767; Default: 0
	 */
	SPI_ENCODER_COMMUTATION_ANGLE = 193,

	/* Select the used absolute encoder initialization mode Forces the rotor into PHI_E zero using the open loop current but actively swings the rotor. Forces the rotor into PHI_E 90 degree position and then into zero position using the open loop current.
	 * Min: 0; Max: 2; Default: 0
	 * Values:
	 *   FORCED_PHI_E_ZERO_WITH_ACTIVE_SWING = 0
	 *   FORCED_PHI_E_90_ZERO = 1
	 *   USE_OFFSET = 2
	 */
	SPI_ENCODER_INITIALIZATION_METHOD = 194,

	/* SPI encoder direction.
	 * Min: 0; Max: 1; Default: 0
	 * Values:
	 *   NOT_INVERTED = 0
	 *   INVERTED = 1
	 */
	SPI_ENCODER_DIRECTION = 195,

	/* This value represents the internal commutation offset. (0...max. encoder steps per rotation).
	 * Min: 0; Max: 4294967295; Default: 0
	 */
	SPI_ENCODER_OFFSET = 196,

	/* Enable the lookup table based encoder correction.
	 * Min: 0; Max: 1; Default: 0
	 * Values:
	 *   DISABLED = 0
	 *   ENABLED = 1
	 */
	SPI_LUT_CORRECTION_ENABLE = 197,

	/* Address to read or write the lookup table.
	 * Min: 0; Max: 255; Default: 0
	 */
	SPI_LUT_ADDRESS_SELECT = 198,

	/* Data to read or write to a lookup table address.
	 * Min: -128; Max: 127; Default: 0
	 */
	SPI_LUT_DATA = 199,

	/* All LUT table entries are multiplied with 2^SHIFT_FACTOR to compensate for larger erros if needed.
	 * Min: 0; Max: 4; Default: 0
	 */
	SPI_LUT_COMMON_SHIFT_FACTOR = 201,

	/* Configure step/dir to use between 1 and 1024 microsteps per full step.
	 * Min: 0; Max: 10; Default: 0
	 * Values:
	 *   STEP_MODE_FULL = 0
	 *   STEP_MODE_HALF = 1
	 *   STEP_MODE_QUARTER = 2
	 *   STEP_MODE_1_8TH = 3
	 *   STEP_MODE_1_16TH = 4
	 *   STEP_MODE_1_32ND = 5
	 *   STEP_MODE_1_64TH = 6
	 *   STEP_MODE_1_128TH = 7
	 *   STEP_MODE_1_256TH = 8
	 *   STEP_MODE_1_512TH = 9
	 *   STEP_MODE_1_1024TH = 10
	 */
	STEP_DIR_STEP_DIVIDER_SHIFT = 205,

	/* Enable the Step/Dir input functionality.
	 * Min: 0; Max: 1; Default: 0
	 * Values:
	 *   DISABLED = 0
	 *   ENABLED = 1
	 */
	STEP_DIR_ENABLE = 206,

	/* Enable the Step/Dir extrapolation feature.
	 * Min: 0; Max: 1; Default: 0
	 * Values:
	 *   DISABLED = 0
	 *   ENABLED = 1
	 */
	STEP_DIR_EXTRAPOLATION_ENABLE = 207,

	/* Step signal timeout limit.
	 * Unit: ms
	 * Min: 1; Max: 2000; Default: 1000
	 */
	STEP_DIR_STEP_SIGNAL_TIMEOUT_LIMIT = 208,

	/* Maximum velocity up to which extrapolation will be used.
	 * Unit: eRPM
	 * Min: 0; Max: 2147483647; Default: 2147483647
	 */
	STEP_DIR_MAXIMUM_EXTRAPOLATION_VELOCITY = 209,

	/* Enable the brake chopper functionality.
	 * Min: 0; Max: 1; Default: 0
	 * Values:
	 *   DISABLED = 0
	 *   ENABLED = 1
	 */
	BRAKE_CHOPPER_ENABLE = 212,

	/* If the brake chopper is enabled and supply voltage exceeds this value, the brake chopper output will be activated.
	 * Unit: 0.1V
	 * Min: 50; Max: 1000; Default: 260
	 */
	BRAKE_CHOPPER_VOLTAGE_LIMIT = 213,

	/* An activated brake chopper will be deactivated if the actual supply voltage is lower than BRAKE_CHOPPER_VOLTAGE_LIMIT - BRAKE_CHOPPER_HYSTERESIS.
	 * Unit: 0.1V
	 * Min: 0; Max: 50; Default: 5
	 */
	BRAKE_CHOPPER_HYSTERESIS = 214,

	/* Release the external brake by applying a PWM signal.
	 * Min: 0; Max: 1; Default: 0
	 * Values:
	 *   BRAKE_PWM_DEACTIVATED = 0
	 *   BRAKE_PWM_ACTIVATED = 1
	 */
	RELEASE_BRAKE = 216,

	/* Set the duty cycle of the first PWM phase for releasing the brake.
	 * Unit: %
	 * Min: 0; Max: 99; Default: 75
	 */
	BRAKE_RELEASING_DUTY_CYCLE = 217,

	/* Set the duty cycle of the second PWM phase to hold the brake.
	 * Unit: %
	 * Min: 0; Max: 99; Default: 11
	 */
	BRAKE_HOLDING_DUTY_CYCLE = 218,

	/* Set the duration the brake PWM uses the first duty cycle.
	 * Unit: ms
	 * Min: 0; Max: 65535; Default: 80
	 */
	BRAKE_RELEASING_DURATION = 219,

	/* Invert the brake output.
	 * Min: 0; Max: 1; Default: 0
	 * Values:
	 *   NORMAL = 0
	 *   INVERTED = 1
	 */
	INVERT_BRAKE_OUTPUT = 221,

	/* Thermal winding time constant for the used motor. Used for IIT monitoring. Setting a new value restarts the IIT monitoring.
	 * Unit: ms
	 * Min: 1000; Max: 60000; Default: 3000
	 */
	THERMAL_WINDING_TIME_CONSTANT_1 = 224,

	/* An actual IIT sum that exceeds this limit leads to trigger the IIT_1_EXCEEDED.
	 * Unit: A^2*ms
	 * Min: 0; Max: 4294967295; Default: 4294967295
	 */
	IIT_LIMIT_1 = 225,

	/* Actual sum of the IIT monitor 1.
	 * Unit: A^2*ms
	 * Min: 0; Max: 4294967295; Default: 0
	 */
	IIT_SUM_1 = 226,

	/* Thermal winding time constant for the used motor. Used for IIT monitoring. Setting a new value restarts the IIT monitoring.
	 * Unit: ms
	 * Min: 1000; Max: 60000; Default: 6000
	 */
	THERMAL_WINDING_TIME_CONSTANT_2 = 227,

	/* An actual IIT sum that exceeds this limit leads to trigger the IIT_2_EXCEEDED.
	 * Unit: A^2*ms
	 * Min: 0; Max: 4294967295; Default: 4294967295
	 */
	IIT_LIMIT_2 = 228,

	/* Actual sum of the IIT monitor 2.
	 * Unit: A^2*ms
	 * Min: 0; Max: 4294967295; Default: 0
	 */
	IIT_SUM_2 = 229,

	/* Reset both IIT sums.
	 * Min: 0; Max: 0; Default: 0
	 */
	RESET_IIT_SUMS = 230,

	/* Total current through the motor windings.
	 * Unit: mA
	 * Min: 0; Max: 65535; Default: 0
	 */
	ACTUAL_TOTAL_MOTOR_CURRENT = 231,

	/* PWM_L output polarity.
	 * Min: 0; Max: 1; Default: 0
	 * Values:
	 *   ACTIVE_HIGH = 0
	 *   ACTIVE_LOW = 1
	 */
	PWM_L_OUTPUT_POLARITY = 233,

	/* PWM_H output polarity.
	 * Min: 0; Max: 1; Default: 0
	 * Values:
	 *   ACTIVE_HIGH = 0
	 *   ACTIVE_LOW = 1
	 */
	PWM_H_OUTPUT_POLARITY = 234,

	/* Break before make time for the low side gates of the UVW phases. Applied before switching from high to low.
	 * Unit: 8.33ns
	 * Min: 0; Max: 255; Default: 0
	 */
	BREAK_BEFORE_MAKE_TIME_LOW_UVW = 235,

	/* Break before make time for the high side gates of the UVW phases. Applied before switching from low to high.
	 * Unit: 8.33ns
	 * Min: 0; Max: 255; Default: 0
	 */
	BREAK_BEFORE_MAKE_TIME_HIGH_UVW = 236,

	/* Break before make time for the low side gate of the Y2 phase. Applied before switching from high to low.
	 * Unit: 8.33ns
	 * Min: 0; Max: 255; Default: 0
	 */
	BREAK_BEFORE_MAKE_TIME_LOW_Y2 = 237,

	/* Break before make time for the high side gate of the Y2 phase. Applied before switching from low to high.
	 * Unit: 8.33ns
	 * Min: 0; Max: 255; Default: 0
	 */
	BREAK_BEFORE_MAKE_TIME_HIGH_Y2 = 238,

	/* If enabled, the discharge cycle of the low and high side gates for the UVW phases will be shortened by monitoring the gate voltages. If enabled, the value on T_DRIVE_SINK will act as an upper bound instead of a fixed time.
	 * Min: 0; Max: 1; Default: 1
	 * Values:
	 *   DISABLED = 0
	 *   ENABLED = 1
	 */
	USE_ADAPTIVE_DRIVE_TIME_UVW = 239,

	/* If enabled, the discharge cycle of the low and high side gates for the Y2 phase will be shortened by monitoring the gate voltages. If enabled, the value on T_DRIVE_SINK will act as an upper bound instead of a fixed time.
	 * Min: 0; Max: 1; Default: 1
	 * Values:
	 *   DISABLED = 0
	 *   ENABLED = 1
	 */
	USE_ADAPTIVE_DRIVE_TIME_Y2 = 240,

	/* Discharge time for the low and high side gates of the UVW phases. During this time, the full sink current will be applied. The applied time is defined as (1s / 120MHz) * (2 * DRIVE_TIME_SINK_UVW + 3).
	 * Min: 0; Max: 255; Default: 255
	 */
	DRIVE_TIME_SINK_UVW = 241,

	/* Charge time for the low and high side gates of the UVW phases. During this time, the full source current will be applied. The applied time is defined as (1s / 120MHz) * (2 * DRIVE_TIME_SOURCE_UVW + 3).
	 * Min: 0; Max: 255; Default: 255
	 */
	DRIVE_TIME_SOURCE_UVW = 242,

	/* Discharge time for the low and high side gates of the Y2 phase. During this time, the full sink current will be applied. The applied time is defined as (1s / 120MHz) * (2 * DRIVE_TIME_SINK_Y2 + 3).
	 * Min: 0; Max: 255; Default: 255
	 */
	DRIVE_TIME_SINK_Y2 = 243,

	/* Charge time for the low and high side gates of the Y2 phase. During this time, the full source current will be applied. The applied time is defined as (1s / 120MHz) * (2 * DRIVE_TIME_SOURCE_Y2 + 3).
	 * Min: 0; Max: 255; Default: 255
	 */
	DRIVE_TIME_SOURCE_Y2 = 244,

	/* Limit the maximum sink current for the low and high side gates of the UVW phases.
	 * Min: 0; Max: 15; Default: 4
	 * Values:
	 *   CUR_50_MILLIAMP = 0
	 *   CUR_100_MILLIAMP = 1
	 *   CUR_160_MILLIAMP = 2
	 *   CUR_210_MILLIAMP = 3
	 *   CUR_270_MILLIAMP = 4
	 *   CUR_320_MILLIAMP = 5
	 *   CUR_380_MILLIAMP = 6
	 *   CUR_430_MILLIAMP = 7
	 *   CUR_580_MILLIAMP = 8
	 *   CUR_720_MILLIAMP = 9
	 *   CUR_860_MILLIAMP = 10
	 *   CUR_1000_MILLIAMP = 11
	 *   CUR_1250_MILLIAMP = 12
	 *   CUR_1510_MILLIAMP = 13
	 *   CUR_1770_MILLIAMP = 14
	 *   CUR_2000_MILLIAMP = 15
	 */
	UVW_SINK_CURRENT = 245,

	/* Limit the maximum source current for the low and high side gates of the UVW phases.
	 * Min: 0; Max: 15; Default: 4
	 * Values:
	 *   CUR_25_MILLIAMP = 0
	 *   CUR_50_MILLIAMP = 1
	 *   CUR_80_MILLIAMP = 2
	 *   CUR_105_MILLIAMP = 3
	 *   CUR_135_MILLIAMP = 4
	 *   CUR_160_MILLIAMP = 5
	 *   CUR_190_MILLIAMP = 6
	 *   CUR_215_MILLIAMP = 7
	 *   CUR_290_MILLIAMP = 8
	 *   CUR_360_MILLIAMP = 9
	 *   CUR_430_MILLIAMP = 10
	 *   CUR_500_MILLIAMP = 11
	 *   CUR_625_MILLIAMP = 12
	 *   CUR_755_MILLIAMP = 13
	 *   CUR_855_MILLIAMP = 14
	 *   CUR_1000_MILLIAMP = 15
	 */
	UVW_SOURCE_CURRENT = 246,

	/* Limit the maximum sink current for the low and high side gates of the Y2 phase.
	 * Min: 0; Max: 15; Default: 4
	 * Values:
	 *   CUR_50_MILLIAMP = 0
	 *   CUR_100_MILLIAMP = 1
	 *   CUR_160_MILLIAMP = 2
	 *   CUR_210_MILLIAMP = 3
	 *   CUR_270_MILLIAMP = 4
	 *   CUR_320_MILLIAMP = 5
	 *   CUR_380_MILLIAMP = 6
	 *   CUR_430_MILLIAMP = 7
	 *   CUR_580_MILLIAMP = 8
	 *   CUR_720_MILLIAMP = 9
	 *   CUR_860_MILLIAMP = 10
	 *   CUR_1000_MILLIAMP = 11
	 *   CUR_1250_MILLIAMP = 12
	 *   CUR_1510_MILLIAMP = 13
	 *   CUR_1770_MILLIAMP = 14
	 *   CUR_2000_MILLIAMP = 15
	 */
	Y2_SINK_CURRENT = 247,

	/* Limit the maximum source current for the low and high side gates of the Y2 phase.
	 * Min: 0; Max: 15; Default: 4
	 * Values:
	 *   CUR_25_MILLIAMP = 0
	 *   CUR_50_MILLIAMP = 1
	 *   CUR_80_MILLIAMP = 2
	 *   CUR_105_MILLIAMP = 3
	 *   CUR_135_MILLIAMP = 4
	 *   CUR_160_MILLIAMP = 5
	 *   CUR_190_MILLIAMP = 6
	 *   CUR_215_MILLIAMP = 7
	 *   CUR_290_MILLIAMP = 8
	 *   CUR_360_MILLIAMP = 9
	 *   CUR_430_MILLIAMP = 10
	 *   CUR_500_MILLIAMP = 11
	 *   CUR_625_MILLIAMP = 12
	 *   CUR_755_MILLIAMP = 13
	 *   CUR_855_MILLIAMP = 14
	 *   CUR_1000_MILLIAMP = 15
	 */
	Y2_SOURCE_CURRENT = 248,

	/* Bootstrap current limit.
	 * Min: 0; Max: 7; Default: 7
	 * Values:
	 *   CUR_45_MILLIAMP = 0
	 *   CUR_91_MILLIAMP = 1
	 *   CUR_141_MILLIAMP = 2
	 *   CUR_191_MILLIAMP = 3
	 *   CUR_267_MILLIAMP = 4
	 *   CUR_292_MILLIAMP = 5
	 *   CUR_341_MILLIAMP = 6
	 *   CUR_391_MILLIAMP = 7
	 */
	BOOTSTRAP_CURRENT_LIMIT = 249,

	/* Undervoltage protection level for VS (Supply voltage). 0 disables the comparator. 1-16 are mapped to 0-15 HW values, with the comparator enabled.
	 * Min: 0; Max: 16; Default: 0
	 */
	UNDERVOLTAGE_PROTECTION_SUPPLY_LEVEL = 250,

	/* Enable the undervoltage protection for VDRV (Driver voltage).
	 * Min: 0; Max: 1; Default: 1
	 * Values:
	 *   DISABLED = 0
	 *   ENABLED = 1
	 */
	UNDERVOLTAGE_PROTECTION_VDRV_ENABLE = 251,

	/* Enable the undervoltage protection on the bootstrap capacitor of the UVW phases.
	 * Min: 0; Max: 1; Default: 1
	 * Values:
	 *   DISABLED = 0
	 *   ENABLED = 1
	 */
	UNDERVOLTAGE_PROTECTION_BST_UVW_ENABLE = 252,

	/* Enable the undervoltage protection on the bootstrap capacitor of the Y2 phase.
	 * Min: 0; Max: 1; Default: 1
	 * Values:
	 *   DISABLED = 0
	 *   ENABLED = 1
	 */
	UNDERVOLTAGE_PROTECTION_BST_Y2_ENABLE = 253,

	/* Enable the overcurrent protection on the low side of the UVW phases.
	 * Min: 0; Max: 1; Default: 1
	 * Values:
	 *   DISABLED = 0
	 *   ENABLED = 1
	 */
	OVERCURRENT_PROTECTION_UVW_LOW_SIDE_ENABLE = 254,

	/* Enable the overcurrent protection on the high side of the UVW phases.
	 * Min: 0; Max: 1; Default: 1
	 * Values:
	 *   DISABLED = 0
	 *   ENABLED = 1
	 */
	OVERCURRENT_PROTECTION_UVW_HIGH_SIDE_ENABLE = 255,

	/* Enable the overcurrent protection on the low side of the Y2 phase.
	 * Min: 0; Max: 1; Default: 1
	 * Values:
	 *   DISABLED = 0
	 *   ENABLED = 1
	 */
	OVERCURRENT_PROTECTION_Y2_LOW_SIDE_ENABLE = 256,

	/* Enable the overcurrent protection on the high side of the Y2 phase.
	 * Min: 0; Max: 1; Default: 1
	 * Values:
	 *   DISABLED = 0
	 *   ENABLED = 1
	 */
	OVERCURRENT_PROTECTION_Y2_HIGH_SIDE_ENABLE = 257,

	/* Overcurrent protection threshold for the low side of the UVW phases (uses second list if OVERCURRENT_PROTECTION_UVW_LOW_SIDE_USE_VD S=true).
	 * Min: 0; Max: 15; Default: 0
	 * Values:
	 *   V_80_OR_63_MILLIVOLT = 0
	 *   V_165_OR_125_MILLIVOLT = 1
	 *   V_250_OR_187_MILLIVOLT = 2
	 *   V_330_OR_248_MILLIVOLT = 3
	 *   V_415_OR_312_MILLIVOLT = 4
	 *   V_500_OR_374_MILLIVOLT = 5
	 *   V_582_OR_434_MILLIVOLT = 6
	 *   V_660_OR_504_MILLIVOLT = 7
	 *   V_125_OR_705_MILLIVOLT = 8
	 *   V_250_OR_940_MILLIVOLT = 9
	 *   V_375_OR_1180_MILLIVOLT = 10
	 *   V_500_OR_1410_MILLIVOLT = 11
	 *   V_625_OR_1650_MILLIVOLT = 12
	 *   V_750_OR_1880_MILLIVOLT = 13
	 *   V_875_OR_2110_MILLIVOLT = 14
	 *   V_1000_OR_2350_MILLIVOLT = 15
	 */
	OVERCURRENT_PROTECTION_UVW_LOW_SIDE_THRESHOLD = 258,

	/* Overcurrent protection threshold for the high side of the UVW phases.
	 * Min: 0; Max: 15; Default: 0
	 * Values:
	 *   V_63_MILLIVOLT = 0
	 *   V_125_MILLIVOLT = 1
	 *   V_187_MILLIVOLT = 2
	 *   V_248_MILLIVOLT = 3
	 *   V_312_MILLIVOLT = 4
	 *   V_374_MILLIVOLT = 5
	 *   V_434_MILLIVOLT = 6
	 *   V_504_MILLIVOLT = 7
	 *   V_705_MILLIVOLT = 8
	 *   V_940_MILLIVOLT = 9
	 *   V_1180_MILLIVOLT = 10
	 *   V_1410_MILLIVOLT = 11
	 *   V_1650_MILLIVOLT = 12
	 *   V_1880_MILLIVOLT = 13
	 *   V_2110_MILLIVOLT = 14
	 *   V_2350_MILLIVOLT = 15
	 */
	OVERCURRENT_PROTECTION_UVW_HIGH_SIDE_THRESHOLD = 259,

	/* Overcurrent protection threshold for the low side of the Y2 phase (uses second list if OVERCURRENT_PROTECTION_Y2_LOW_SIDE_USE_VDS= true).
	 * Min: 0; Max: 15; Default: 0
	 * Values:
	 *   V_80_OR_63_MILLIVOLT = 0
	 *   V_165_OR_125_MILLIVOLT = 1
	 *   V_250_OR_187_MILLIVOLT = 2
	 *   V_330_OR_248_MILLIVOLT = 3
	 *   V_415_OR_312_MILLIVOLT = 4
	 *   V_500_OR_374_MILLIVOLT = 5
	 *   V_582_OR_434_MILLIVOLT = 6
	 *   V_660_OR_504_MILLIVOLT = 7
	 *   V_125_OR_705_MILLIVOLT = 8
	 *   V_250_OR_940_MILLIVOLT = 9
	 *   V_375_OR_1180_MILLIVOLT = 10
	 *   V_500_OR_1410_MILLIVOLT = 11
	 *   V_625_OR_1650_MILLIVOLT = 12
	 *   V_750_OR_1880_MILLIVOLT = 13
	 *   V_875_OR_2110_MILLIVOLT = 14
	 *   V_1000_OR_2350_MILLIVOLT = 15
	 */
	OVERCURRENT_PROTECTION_Y2_LOW_SIDE_THRESHOLD = 260,

	/* Overcurrent protection threshold for the high side of the Y2 phase.
	 * Min: 0; Max: 15; Default: 0
	 * Values:
	 *   V_63_MILLIVOLT = 0
	 *   V_125_MILLIVOLT = 1
	 *   V_187_MILLIVOLT = 2
	 *   V_248_MILLIVOLT = 3
	 *   V_312_MILLIVOLT = 4
	 *   V_374_MILLIVOLT = 5
	 *   V_434_MILLIVOLT = 6
	 *   V_504_MILLIVOLT = 7
	 *   V_705_MILLIVOLT = 8
	 *   V_940_MILLIVOLT = 9
	 *   V_1180_MILLIVOLT = 10
	 *   V_1410_MILLIVOLT = 11
	 *   V_1650_MILLIVOLT = 12
	 *   V_1880_MILLIVOLT = 13
	 *   V_2110_MILLIVOLT = 14
	 *   V_2350_MILLIVOLT = 15
	 */
	OVERCURRENT_PROTECTION_Y2_HIGH_SIDE_THRESHOLD = 261,

	/* Overcurrent protection blanking time for the low side of the UVW phases.
	 * Min: 0; Max: 7; Default: 2
	 * Values:
	 *   OFF = 0
	 *   T_0_25_MICROSEC = 1
	 *   T_0_5_MICROSEC = 2
	 *   T_1_MICROSEC = 3
	 *   T_2_MICROSEC = 4
	 *   T_4_MICROSEC = 5
	 *   T_6_MICROSEC = 6
	 *   T_8_MICROSEC = 7
	 */
	OVERCURRENT_PROTECTION_UVW_LOW_SIDE_BLANKING = 262,

	/* Overcurrent protection blanking time for the high side of the UVW phases.
	 * Min: 0; Max: 7; Default: 2
	 * Values:
	 *   OFF = 0
	 *   T_0_25_MICROSEC = 1
	 *   T_0_5_MICROSEC = 2
	 *   T_1_MICROSEC = 3
	 *   T_2_MICROSEC = 4
	 *   T_4_MICROSEC = 5
	 *   T_6_MICROSEC = 6
	 *   T_8_MICROSEC = 7
	 */
	OVERCURRENT_PROTECTION_UVW_HIGH_SIDE_BLANKING = 263,

	/* Overcurrent protection blanking time for the low side of the Y2 phase.
	 * Min: 0; Max: 7; Default: 2
	 * Values:
	 *   OFF = 0
	 *   T_0_25_MICROSEC = 1
	 *   T_0_5_MICROSEC = 2
	 *   T_1_MICROSEC = 3
	 *   T_2_MICROSEC = 4
	 *   T_4_MICROSEC = 5
	 *   T_6_MICROSEC = 6
	 *   T_8_MICROSEC = 7
	 */
	OVERCURRENT_PROTECTION_Y2_LOW_SIDE_BLANKING = 264,

	/* Overcurrent protection blanking time for the high side of the Y2 phase.
	 * Min: 0; Max: 7; Default: 2
	 * Values:
	 *   OFF = 0
	 *   T_0_25_MICROSEC = 1
	 *   T_0_5_MICROSEC = 2
	 *   T_1_MICROSEC = 3
	 *   T_2_MICROSEC = 4
	 *   T_4_MICROSEC = 5
	 *   T_6_MICROSEC = 6
	 *   T_8_MICROSEC = 7
	 */
	OVERCURRENT_PROTECTION_Y2_HIGH_SIDE_BLANKING = 265,

	/* Overcurrent protection deglitch time for the low side of the UVW phases.
	 * Min: 0; Max: 7; Default: 6
	 * Values:
	 *   OFF = 0
	 *   T_0_25_MICROSEC = 1
	 *   T_0_5_MICROSEC = 2
	 *   T_1_MICROSEC = 3
	 *   T_2_MICROSEC = 4
	 *   T_4_MICROSEC = 5
	 *   T_6_MICROSEC = 6
	 *   T_8_MICROSEC = 7
	 */
	OVERCURRENT_PROTECTION_UVW_LOW_SIDE_DEGLITCH = 266,

	/* Overcurrent protection deglitch time for the high side of the UVW phases.
	 * Min: 0; Max: 7; Default: 6
	 * Values:
	 *   OFF = 0
	 *   T_0_25_MICROSEC = 1
	 *   T_0_5_MICROSEC = 2
	 *   T_1_MICROSEC = 3
	 *   T_2_MICROSEC = 4
	 *   T_4_MICROSEC = 5
	 *   T_6_MICROSEC = 6
	 *   T_8_MICROSEC = 7
	 */
	OVERCURRENT_PROTECTION_UVW_HIGH_SIDE_DEGLITCH = 267,

	/* Overcurrent protection deglitch time for the low side of the Y2 phase.
	 * Min: 0; Max: 7; Default: 6
	 * Values:
	 *   OFF = 0
	 *   T_0_25_MICROSEC = 1
	 *   T_0_5_MICROSEC = 2
	 *   T_1_MICROSEC = 3
	 *   T_2_MICROSEC = 4
	 *   T_4_MICROSEC = 5
	 *   T_6_MICROSEC = 6
	 *   T_8_MICROSEC = 7
	 */
	OVERCURRENT_PROTECTION_Y2_LOW_SIDE_DEGLITCH = 268,

	/* Overcurrent protection deglitch time for the high side of the Y2 phase.
	 * Min: 0; Max: 7; Default: 6
	 * Values:
	 *   OFF = 0
	 *   T_0_25_MICROSEC = 1
	 *   T_0_5_MICROSEC = 2
	 *   T_1_MICROSEC = 3
	 *   T_2_MICROSEC = 4
	 *   T_4_MICROSEC = 5
	 *   T_6_MICROSEC = 6
	 *   T_8_MICROSEC = 7
	 */
	OVERCURRENT_PROTECTION_Y2_HIGH_SIDE_DEGLITCH = 269,

	/* Use the VDS measurement for the overcurrent protection on the low side of the UVW phases.
	 * Min: 0; Max: 1; Default: 1
	 * Values:
	 *   DISABLED = 0
	 *   ENABLED = 1
	 */
	OVERCURRENT_PROTECTION_UVW_LOW_SIDE_USE_VDS = 270,

	/* Use the VDS measurement for the overcurrent protection on the low side of the Y2 phase.
	 * Min: 0; Max: 1; Default: 1
	 * Values:
	 *   DISABLED = 0
	 *   ENABLED = 1
	 */
	OVERCURRENT_PROTECTION_Y2_LOW_SIDE_USE_VDS = 271,

	/* Enable the gate-source short protection for the ON transition of the low side of the UVW phases.
	 * Min: 0; Max: 1; Default: 1
	 * Values:
	 *   DISABLED = 0
	 *   ENABLED = 1
	 */
	VGS_SHORT_ON_PROTECTION_UVW_LOW_SIDE_ENABLE = 272,

	/* Enable the gate-source short protection for the OFF transition of the low side of the UVW phases.
	 * Min: 0; Max: 1; Default: 1
	 * Values:
	 *   DISABLED = 0
	 *   ENABLED = 1
	 */
	VGS_SHORT_OFF_PROTECTION_UVW_LOW_SIDE_ENABLE = 273,

	/* Enable the gate-source short protection for the ON transition of the high side of the UVW phases.
	 * Min: 0; Max: 1; Default: 1
	 * Values:
	 *   DISABLED = 0
	 *   ENABLED = 1
	 */
	VGS_SHORT_ON_PROTECTION_UVW_HIGH_SIDE_ENABLE = 274,

	/* Enable the gate-source short protection for the OFF transition of the high side of the UVW phases.
	 * Min: 0; Max: 1; Default: 1
	 * Values:
	 *   DISABLED = 0
	 *   ENABLED = 1
	 */
	VGS_SHORT_OFF_PROTECTION_UVW_HIGH_SIDE_ENABLE = 275,

	/* Enable the gate-source short protection for the ON transition of the low side of the Y2 phase.
	 * Min: 0; Max: 1; Default: 1
	 * Values:
	 *   DISABLED = 0
	 *   ENABLED = 1
	 */
	VGS_SHORT_ON_PROTECTION_Y2_LOW_SIDE_ENABLE = 276,

	/* Enable the gate-source short protection for the OFF transition of the low side of the Y2 phase.
	 * Min: 0; Max: 1; Default: 1
	 * Values:
	 *   DISABLED = 0
	 *   ENABLED = 1
	 */
	VGS_SHORT_OFF_PROTECTION_Y2_LOW_SIDE_ENABLE = 277,

	/* Enable the gate-source short protection for the ON transition of the high side of the Y2 phase.
	 * Min: 0; Max: 1; Default: 1
	 * Values:
	 *   DISABLED = 0
	 *   ENABLED = 1
	 */
	VGS_SHORT_ON_PROTECTION_Y2_HIGH_SIDE_ENABLE = 278,

	/* Enable the gate-source short protection for the OFF transition of the high side of the Y2 phase.
	 * Min: 0; Max: 1; Default: 1
	 * Values:
	 *   DISABLED = 0
	 *   ENABLED = 1
	 */
	VGS_SHORT_OFF_PROTECTION_Y2_HIGH_SIDE_ENABLE = 279,

	/* Gate-source short protection blanking time for the low and high sides of the UVW phases.
	 * Min: 0; Max: 3; Default: 1
	 * Values:
	 *   OFF = 0
	 *   T_0_25_MICROSEC = 1
	 *   T_0_5_MICROSEC = 2
	 *   T_1_MICROSEC = 3
	 */
	VGS_SHORT_PROTECTION_UVW_BLANKING = 280,

	/* Gate-source short protection blanking time for the low and high sides of the Y2 phase.
	 * Min: 0; Max: 3; Default: 1
	 * Values:
	 *   OFF = 0
	 *   T_0_25_MICROSEC = 1
	 *   T_0_5_MICROSEC = 2
	 *   T_1_MICROSEC = 3
	 */
	VGS_SHORT_PROTECTION_Y2_BLANKING = 281,

	/* Gate-source short protection deglitch time for the low and high sides of the UVW phases.
	 * Min: 0; Max: 7; Default: 1
	 * Values:
	 *   OFF = 0
	 *   T_0_25_MICROSEC = 1
	 *   T_0_5_MICROSEC = 2
	 *   T_1_MICROSEC = 3
	 *   T_2_MICROSEC = 4
	 *   T_4_MICROSEC = 5
	 *   T_6_MICROSEC = 6
	 *   T_8_MICROSEC = 7
	 */
	VGS_SHORT_PROTECTION_UVW_DEGLITCH = 282,

	/* Gate-source short protection deglitch time for low and high sides of the Y2 phase.
	 * Min: 0; Max: 7; Default: 1
	 * Values:
	 *   OFF = 0
	 *   T_0_25_MICROSEC = 1
	 *   T_0_5_MICROSEC = 2
	 *   T_1_MICROSEC = 3
	 *   T_2_MICROSEC = 4
	 *   T_4_MICROSEC = 5
	 *   T_6_MICROSEC = 6
	 *   T_8_MICROSEC = 7
	 */
	VGS_SHORT_PROTECTION_Y2_DEGLITCH = 283,

	/* This value defines the state the system goes to after a fault condition on a motor phase occurs. The system switches off and discharges the gates. The motor can spin freely. The system switches off and, if possible and depending on fault, enables the LS or HS gates, braking the motor electrically.
	 * Min: 0; Max: 1; Default: 0
	 * Values:
	 *   OPEN_CIRCUIT = 0
	 *   ELECTRICAL_BRAKING = 1
	 */
	GDRV_RETRY_BEHAVIOUR = 286,

	/* This value defines the state the system goes to after a fault condition on a motor phase occurs and all retries failed. The system switches off and discharges the LS and HS gates, letting the motor spin freely. The system switches off and, if possible and depending on fault, enables the LS or HS gates, braking the motor electrically. The system switches off, discharges the LS and HS gates, and, if correctly configured, engages the mechanical brake. The system switches off, if possible and depending on fault, enables the LS or HS gates, and, if correctly configured, engages the mechanical brake.
	 * Min: 0; Max: 3; Default: 0
	 * Values:
	 *   OPEN_CIRCUIT = 0
	 *   ELECTRICAL_BRAKING = 1
	 *   MECHANICAL_BRAKING_AND_OPEN_CIRCUIT = 2
	 *   MECHANICAL_AND_ELECTRICAL_BRAKING = 3
	 */
	DRIVE_FAULT_BEHAVIOUR = 287,

	/* Maximum number of retries that are performed for every fault that is detected.
	 * Min: 0; Max: 255; Default: 5
	 */
	FAULT_HANDLER_NUMBER_OF_RETRIES = 288,

	/* Actual status flags.
	 * Min: 0; Max: 0; Default: -
	 * Values:
	 *   REGULATION_STOPPED = 1
	 *   REGULATION_TORQUE = 2
	 *   REGULATION_VELOCITY = 4
	 *   REGULATION_POSITION = 8
	 *   CONFIG_STORED = 16
	 *   CONFIG_LOADED = 32
	 *   CONFIG_READ_ONLY = 64
	 *   TMCL_SCRIPT_READ_ONLY = 128
	 *   BRAKE_CHOPPER_ACTIVE = 256
	 *   POSITION_REACHED = 512
	 *   VELOCITY_REACHED = 1024
	 *   ADC_OFFSET_CALIBRATED = 2048
	 *   RAMPER_LATCHED = 4096
	 *   RAMPER_EVENT_STOP_SWITCH = 8192
	 *   RAMPER_EVENT_STOP_DEVIATION = 16384
	 *   RAMPER_VELOCITY_REACHED = 32768
	 *   RAMPER_POSITION_REACHED = 65536
	 *   RAMPER_SECOND_MOVE = 131072
	 *   IIT_1_ACTIVE = 262144
	 *   IIT_2_ACTIVE = 524288
	 *   REFSEARCH_FINISHED = 1048576
	 *   Y2_USED_FOR_BRAKING = 2097152
	 *   FLASH_STIMULUS_AVAILABLE = 4194304
	 *   STEPDIR_INPUT_AVAILABLE = 8388608
	 *   RIGHT_REF_SWITCH_AVAILABLE = 16777216
	 *   HOME_REF_SWITCH_AVAILABLE = 33554432
	 *   LEFT_REF_SWITCH_AVAILABLE = 67108864
	 *   ABN2_FEEDBACK_AVAILABLE = 134217728
	 *   HALL_FEEDBACK_AVAILABLE = 268435456
	 *   ABN1_FEEDBACK_AVAILABLE = 536870912
	 *   SPI_FLASH_AVAILABLE = 1073741824
	 *   I2C_EEPROM_AVAILABLE = 2147483648
	 */
	GENERAL_STATUS_FLAGS = 289,

	/* The actual supply voltage.
	 * Unit: 0.1V
	 * Min: 0; Max: 1000; Default: 0
	 */
	SUPPLY_VOLTAGE = 290,

	/* The supply overvoltage warning  threshold.
	 * Unit: 0.1V
	 * Min: 0; Max: 1000; Default: 480
	 */
	SUPPLY_OVERVOLTAGE_WARNING_THRESHOLD = 291,

	/* The supply undervoltage warning  threshold.
	 * Unit: 0.1V
	 * Min: 0; Max: 1000; Default: 50
	 */
	SUPPLY_UNDERVOLTAGE_WARNING_THRESHOLD = 292,

	/* The actual temperature at the external temperature sensor.
	 * Min: 0; Max: 65535; Default: 0
	 */
	EXTERNAL_TEMPERATURE = 293,

	/* The temperature threshold at which the motor driver is shut down.
	 * Min: 0; Max: 65535; Default: 65535
	 */
	EXTERNAL_TEMPERATURE_SHUTDOWN_THRESHOLD = 294,

	/* The temperature threshold above which the warning flag is set.
	 * Min: 0; Max: 65535; Default: 65535
	 */
	EXTERNAL_TEMPERATURE_WARNING_THRESHOLD = 295,

	/* The actual temperature of the chip.
	 * Min: 0; Max: 65535; Default: 0
	 */
	CHIP_TEMPERATURE = 296,

	/* The temperature threshold at which the motor driver is shut down.
	 * Min: 0; Max: 65535; Default: 65535
	 */
	CHIP_TEMPERATURE_SHUTDOWN_THRESHOLD = 297,

	/* The temperature threshold above which the warning flag is set.
	 * Min: 0; Max: 65535; Default: 65535
	 */
	CHIP_TEMPERATURE_WARNING_THRESHOLD = 298,

	/* Actual error flags.
	 * Min: 0; Max: 0; Default: -
	 * Values:
	 *   CONFIG_ERROR = 1
	 *   TMCL_SCRIPT_ERROR = 2
	 *   HOMESWITCH_NOT_FOUND = 4
	 *   HALL_ERROR = 32
	 *   WATCHDOG_EVENT = 512
	 *   EXT_TEMP_EXCEEDED = 8192
	 *   CHIP_TEMP_EXCEEDED = 16384
	 *   ITT_1_EXCEEDED = 65536
	 *   ITT_2_EXCEEDED = 131072
	 *   EXT_TEMP_WARNING = 262144
	 *   SUPPLY_OVERVOLTAGE_WARNING = 524288
	 *   SUPPLY_UNDERVOLTAGE_WARNING = 1048576
	 *   ADC_IN_OVERVOLTAGE = 2097152
	 *   FAULT_RETRY_HAPPEND = 4194304
	 *   FAULT_RETRIES_FAILED = 8388608
	 *   CHIP_TEMP_WARNING = 16777216
	 *   HEARTBEAT_STOPPED = 67108864
	 */
	GENERAL_ERROR_FLAGS = 299,

	/* Gate driver error flags.
	 * Min: 0; Max: 0; Default: -
	 * Values:
	 *   U_LOW_SIDE_OVERCURRENT = 1
	 *   V_LOW_SIDE_OVERCURRENT = 2
	 *   W_LOW_SIDE_OVERCURRENT = 4
	 *   Y2_LOW_SIDE_OVERCURRENT = 8
	 *   U_LOW_SIDE_DISCHARGE_SHORT = 16
	 *   V_LOW_SIDE_DISCHARGE_SHORT = 32
	 *   W_LOW_SIDE_DISCHARGE_SHORT = 64
	 *   Y2_LOW_SIDE_DISCHARGE_SHORT = 128
	 *   U_LOW_SIDE_CHARGE_SHORT = 256
	 *   V_LOW_SIDE_CHARGE_SHORT = 512
	 *   W_LOW_SIDE_CHARGE_SHORT = 1024
	 *   Y2_LOW_SIDE_CHARGE_SHORT = 2048
	 *   U_BOOTSTRAP_UNDERVOLTAGE = 4096
	 *   V_BOOTSTRAP_UNDERVOLTAGE = 8192
	 *   W_BOOTSTRAP_UNDERVOLTAGE = 16384
	 *   Y2_BOOTSTRAP_UNDERVOLTAGE = 32768
	 *   U_HIGH_SIDE_OVERCURRENT = 65536
	 *   V_HIGH_SIDE_OVERCURRENT = 131072
	 *   W_HIGH_SIDE_OVERCURRENT = 262144
	 *   Y2_HIGH_SIDE_OVERCURRENT = 524288
	 *   U_HIGH_SIDE_DISCHARGE_SHORT = 1048576
	 *   V_HIGH_SIDE_DISCHARGE_SHORT = 2097152
	 *   W_HIGH_SIDE_DISCHARGE_SHORT = 4194304
	 *   Y2_HIGH_SIDE_DISCHARGE_SHORT = 8388608
	 *   U_HIGH_SIDE_CHARGE_SHORT = 16777216
	 *   V_HIGH_SIDE_CHARGE_SHORT = 33554432
	 *   W_HIGH_SIDE_CHARGE_SHORT = 67108864
	 *   Y2_HIGH_SIDE_CHARGE_SHORT = 134217728
	 *   GDRV_UNDERVOLTAGE = 536870912
	 *   GDRV_LOW_VOLTAGE = 1073741824
	 *   GDRV_SUPPLY_UNDERVOLTAGE = 2147483648
	 */
	GDRV_ERROR_FLAGS = 300,

	/* ADC channel clipped. Flags do latch, write 1 to clear.
	 * Min: 0; Max: 0; Default: -
	 * Values:
	 *   I0_CLIPPED = 1
	 *   I1_CLIPPED = 2
	 *   I2_CLIPPED = 4
	 *   I3_CLIPPED = 8
	 *   U0_CLIPPED = 16
	 *   U1_CLIPPED = 32
	 *   U2_CLIPPED = 64
	 *   U3_CLIPPED = 128
	 *   AIN0_CLIPPED = 256
	 *   AIN1_CLIPPED = 512
	 *   AIN2_CLIPPED = 1024
	 *   AIN3_CLIPPED = 2048
	 *   VM_CLIPPED = 4096
	 *   TEMP_CLIPPED = 8192
	 */
	ADC_STATUS_FLAGS = 301,

	/* Raw inputs for ABN, hall, reference switches, driver enabled, hall filtered and ABN2 or Step/Dir.
	 * Min: 0; Max: 32767; Default: 0
	 */
	MCC_INPUTS_RAW = 304,

	/* Interim result of the FOC for phase U (X in case of motor type is a stepper motor).
	 * Min: -32768; Max: 32767; Default: 0
	 */
	FOC_VOLTAGE_UX = 305,

	/* Interim result of the FOC for phase W (Y in case of motor type is a stepper motor).
	 * Min: -32768; Max: 32767; Default: 0
	 */
	FOC_VOLTAGE_WY = 306,

	/* Interim result of the FOC for phase V (BLDC motor only).
	 * Min: -32768; Max: 32767; Default: 0
	 */
	FOC_VOLTAGE_V = 307,

	/* I parameter for field weakening controller.
	 * Min: 0; Max: 32767; Default: 0
	 */
	FIELDWEAKENING_I = 308,

	/* Maximum motor voltage allowed for field weakening.
	 * Min: 0; Max: 32767; Default: 32767
	 */
	FIELDWEAKENING_VOLTAGE_THRESHOLD = 310,

	/* Interim measurement of the FOC for phase UX.
	 * Min: -32768; Max: 32767; Default: 0
	 */
	FOC_CURRENT_UX = 311,

	/* Interim measurement of the FOC for phase V.
	 * Min: -32768; Max: 32767; Default: 0
	 */
	FOC_CURRENT_V = 312,

	/* Interim measurement of the FOC for phase WY.
	 * Min: -32768; Max: 32767; Default: 0
	 */
	FOC_CURRENT_WY = 313,

	/* Interim measurement of the FOC for Uq.
	 * Min: -32768; Max: 32767; Default: 0
	 */
	FOC_VOLTAGE_UQ = 314,

	/* Interim measurement of the FOC for Iq.
	 * Min: -32768; Max: 32767; Default: 0
	 */
	FOC_CURRENT_IQ = 315,

	/* Enable the target torque biquad filter.
	 * Min: 0; Max: 1; Default: 0
	 * Values:
	 *   DISABLED = 0
	 *   ENABLED = 1
	 */
	TARGET_TORQUE_BIQUAD_FILTER_ENABLE = 318,

	/* Target torque biquad filter aCoeff_1.
	 * Min: -2147483648; Max: 2147483647; Default: 0
	 */
	TARGET_TORQUE_BIQUAD_FILTER_ACOEFF_1 = 319,

	/* Target torque biquad filter aCoeff_2.
	 * Min: -2147483648; Max: 2147483647; Default: 0
	 */
	TARGET_TORQUE_BIQUAD_FILTER_ACOEFF_2 = 320,

	/* Target torque biquad filter bCoeff_0.
	 * Min: -2147483648; Max: 2147483647; Default: 1048576
	 */
	TARGET_TORQUE_BIQUAD_FILTER_BCOEFF_0 = 321,

	/* Target torque biquad filter bCoeff_1.
	 * Min: -2147483648; Max: 2147483647; Default: 0
	 */
	TARGET_TORQUE_BIQUAD_FILTER_BCOEFF_1 = 322,

	/* Target torque biquad filter bCoeff_2.
	 * Min: -2147483648; Max: 2147483647; Default: 0
	 */
	TARGET_TORQUE_BIQUAD_FILTER_BCOEFF_2 = 323,

	/* Enable the actual velocity biquad filter.
	 * Min: 0; Max: 1; Default: 1
	 * Values:
	 *   DISABLED = 0
	 *   ENABLED = 1
	 */
	ACTUAL_VELOCITY_BIQUAD_FILTER_ENABLE = 324,

	/* Actual velocity biquad filter aCoeff_1.
	 * Min: -2147483648; Max: 2147483647; Default: 1849195
	 */
	ACTUAL_VELOCITY_BIQUAD_FILTER_ACOEFF_1 = 325,

	/* Actual velocity biquad filter aCoeff_2.
	 * Min: -2147483648; Max: 2147483647; Default: 15961938
	 */
	ACTUAL_VELOCITY_BIQUAD_FILTER_ACOEFF_2 = 326,

	/* Actual velocity biquad filter bCoeff_0.
	 * Min: -2147483648; Max: 2147483647; Default: 3665
	 */
	ACTUAL_VELOCITY_BIQUAD_FILTER_BCOEFF_0 = 327,

	/* Actual velocity biquad filter bCoeff_1.
	 * Min: -2147483648; Max: 2147483647; Default: 7329
	 */
	ACTUAL_VELOCITY_BIQUAD_FILTER_BCOEFF_1 = 328,

	/* Actual velocity biquad filter bCoeff_2.
	 * Min: -2147483648; Max: 2147483647; Default: 3665
	 */
	ACTUAL_VELOCITY_BIQUAD_FILTER_BCOEFF_2 = 329,

	/* Raw (unscaled) torque and flux target values combined into one 32 bit value. This is only used for simplified compact measurement during tuning operations.
	 * Min: 0; Max: 4294967295; Default: 0
	 */
	TORQUE_FLUX_COMBINED_TARGET_VALUES = 330,

	/* Raw (unscaled) torque and flux actual values combined into one 32 bit value. This is only used for simplified compact measurement during tuning operations.
	 * Min: 0; Max: 4294967295; Default: 0
	 */
	TORQUE_FLUX_COMBINED_ACTUAL_VALUES = 331,

	/* Raw (unscaled) voltage actual values combined into one 32 bit value. This is only used for simplified compact measurement during tuning operations.
	 * Min: 0; Max: 4294967295; Default: 0
	 */
	VOLTAGE_D_Q_COMBINED_ACTUAL_VALUES = 332,

	/* Periodically summed up actual torque value. This is only used for simplified measurement with low measurement frequency during tuning operations.
	 * Min: 0; Max: 4294967295; Default: 0
	 */
	INTEGRATED_ACTUAL_TORQUE_VALUE = 333,

	/* Periodically summed up actual velocity value. This is only used for simplified measurement with low measurement frequency during tuning operations.
	 * Min: 0; Max: 4294967295; Default: 0
	 */
	INTEGRATED_ACTUAL_VELOCITY_VALUE = 334,

    TMC9660_PARAM_MAX
};


/* Initialize a freshly booted TMC9660 and puts it into parameter mode */
int tmc9660_init(
    struct tmc9660_dev *dev,
    const struct spi_dt_spec *spi
);

/* Set parameter mode parameter. value_out contains the resulting value, which may not equal the set value in error cases. */
int tmc9660_set_param(
    struct tmc9660_dev *dev,
    enum tmc9660_param_id param,
    int value,
    int *value_out
);

/* Get parameter mode parameter */
int tmc9660_get_param(
    struct tmc9660_dev *dev,
    enum tmc9660_param_id param,
    int *value
);

/* Sets a digital GPIO output */
int tmc9660_set_gpio(
    struct tmc9660_dev *dev,
    int port,
    int value
);

/* Reads a GPIO analog input */
int tmc9660_get_gpio_analog(
    struct tmc9660_dev *dev,
    int port,
    int *value
);

/* Reads a GPIO digital input */
int tmc9660_get_gpio_digital(
    struct tmc9660_dev *dev,
    int port,
    int *value
);

/* Emergency stop, deactivates everything */
int tmc9660_motor_stop(
    struct tmc9660_dev *dev
);
