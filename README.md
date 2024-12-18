# Arduino AGV Motor Control board firmware

## Getting started

For more detail, see the official [Zephyr Getting Started Guide](https://docs.zephyrproject.org/latest/develop/getting_started/index.html).

```
mkdir zephyrworkspace
cd zephyrworkspace
python3 -m venv .venv
source .venv/bin/activate
pip install west
west init .
west update # This may take a while
west zephyr-export
```

Clone this project in the zephyrworkspace folder (though it *should*
fingers-crossed work if cloned anywhere on your machine):
```
git clone git@github.com:trupples/agv-motor-control-fw.git ardagvmotor
cd ardagvmotor
```

Use project-specific manifest file, to bring the proper, currently internal
drivers:
```
west config manifest.path ardagvmotor
west update
```

If you cloned the project somewhere else, you must change the commands above to
point to where you cloned it. Relative paths are based on the west workspace.

You will need access to https://github.com/Ioan-Dragomir_adi/zephyr, which I can
only give individually, hmu.

## Example code

`src/main.c` contains a bunch of examples, in their own functions:

- `demo_blink_fault()` - uses the fault pin as an output to blink the fault LED. Normally, the fault pin is an input to monitor the TMC.
- `demo_uart_control()` - simple UART commands to spin the motor in position / velocity / torque control modes.
- `demo_can_blinky()` - simple CAN exercise that tests both send and receive functionality. Depends on innersource can_max32 zephyr driver :/
- `demo_blink_tmc_gpio()` - sends TMCL commands to blink the LED cnnected to the TMC9660
- `demo_tmc_spi()` - initializes TMC and sends some fixed velocity commands

## Project structure

- `boards/adi/ardagvmotor`: board definition for both revA (default) and revB. Especially relevant files:
    - `ardagvmotor_defconfig`: default kconfig enabling all board functionality
    - `ardagvmotor.dts`: base device tree
    - `ardagvmotor_X.overlay`: revision overlays
- `src`
    - `tmc9660.c/h`: TMC9660 parameter mode SPI driver
    - `nesimtit.c/h`: "Romanul pune sarma" SPI implementation because the spi_max32 in zephyr misbehaves. Selected by the `ESTI_NESIMTIT` define in `CMakeList.txt`
    - `main.c`
- `objdict`: Will be relevant for CANOpenNode
- `prj.conf`: Additional kconfig settings
- `west.yml`: West manifest file. Useful for replacing the mainline repos with our custom in-dev branches
