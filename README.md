# Arduino AGV Motor Control board firmware

## Getting started

For more detail, see the official [Zephyr Getting Started Guide](https://docs.zephyrproject.org/latest/develop/getting_started/index.html).

If running Windows, WSL is highly recommended.

Install prerequisite system packages:
```
sudo apt update
sudo apt install --no-install-recommends git cmake ninja-build gperf \
  ccache dfu-util device-tree-compiler wget \
  python3-dev python3-pip python3-setuptools python3-tk python3-wheel xz-utils file \
  make gcc gcc-multilib g++-multilib libsdl2-dev libmagic1
```

Create a folder with a python virtual environment:
```
mkdir zephyrproject
cd zephyrproject
python3 -m venv .venv
source .venv/bin/activate
```

Install `west` (metatool that does everyhting zephyr-related) and do a basic
setup:
```
pip install west
west init . # This might take a while - big download
west update # This might take a while - big download
west zephyr-export
west packages pip --install
cd zephyr
west sdk install
cd ..
```

Clone this project in the `zephyrworkspace` folder (T2 topology):
```
git clone git@github.com:trupples/agv-motor-control-fw.git ardagvmotor
cd ardagvmotor
```

Use project-specific manifest file, to bring the proper, currently internal
drivers:
```
west config manifest.path ardagvmotor
west update # This will require innersource credentials
```

If you cloned the project somewhere else, you must change the `manifest.path`
command above to point to where you cloned it. Relative paths are based on the
west workspace (`zephyrproject` folder).

You will need access to https://github.com/Ioan-Dragomir_adi/zephyr, which I can
only give individually, hmu.

## Building and flashing

You must be located in the `ardagvmotor` folder and have the virtualenv loaded (Run `source ~/zephyrproject/.venv/bin/activate` in every new terminal in which you need west commands).

Build:
```
west build . -b ardagvmotor
```

Clean ("pristine") build:
```
west build . -b ardagvmotor -p
```

Flash (assuming the DAPLINK is attached to WSL using USB/IP):
```
west flash --openocd-search /MaximSDK/Tools/OpenOCD/scripts/ --openocd /MaximSDK/Tools/OpenOCD/openocd
```

If it gets stuck during flashing, just try again, there's some embedded magic I
don't understand going on and it only flashes every other try.

## Example code

`src/demos.c` contains a bunch of examples, in their own functions:

- `demo_blink_fault()` - uses the fault pin as an output to blink the fault LED. Normally, the fault pin is an input to monitor the TMC.
- `demo_uart_control()` - simple UART commands to spin the motor in position / velocity / torque control modes.
- `demo_can_blinky()` - simple CAN exercise that tests both send and receive functionality. Depends on innersource can_max32 zephyr driver :/
- `demo_blink_tmc_gpio()` - sends TMCL commands to blink the LED cnnected to the TMC9660
- `demo_tmc_spi()` - initializes TMC and sends some fixed velocity commands

## Project structure

- `boards/adi/ardagvmotor`: board definition for both revA and revB (default). Especially relevant files:
    - `ardagvmotor_defconfig`: default kconfig enabling all board functionality
    - `ardagvmotor.dts`: base device tree
    - `ardagvmotor_X.overlay`: revision overlays
- `src`
    - `tmc9660.c/h`: TMC9660 parameter mode SPI driver
    - `main.c`
- `objdict`: CANOpenNode object dictionary, EDS, XDD
- `prj.conf`: Additional kconfig settings
- `west.yml`: West manifest file. Useful for replacing the mainline repos with our custom in-dev branches
