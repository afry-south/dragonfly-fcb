# STM32F3-Discovery - Getting started template

## Installation and configuration

1. Download [ARM GCC](https://launchpad.net/gcc-arm-embedded/)
2. Download [CoIDE](http://www.coocox.com/CoIDE/CoIDE_Updates.htm)
3. Download [STM32 ST-LINK Utility](http://www.st.com/web/en/catalog/tools/PF258168)
4. Install ARM GCC
5. Install CoIDE
6. Configure CoIDE to point to ARM GCC (http://www.coocox.com/CoIDE/Compiler_Settings.html)
7. Install STM32 ST-LINK Utility
8. Plug in USB cable from PC to ST-LINK on discovery board

# Flight Control Board (FCB) TEMPLATE

The fcb-hal-template branch is used as a template project for getting started with the STM32 board and STM32Cube HAL Drivers.

Board startup code is available in startup_stm32f30x.S, where SystemInit() is called before proceeding to main().

The code includes some basic initialization and makes use of the on-board push button and some LEDs.

It also sets up a Virtual COM port over USB (requires STM32 VCP drivers) which simply echoes back entered data. To use it, the USB User
must be connected at board startup. Data can then be sent/received using a serial terminal program such as Putty or Termite with settings:
- Baudrate: 115200
- Data bits: 8
- Stop bits: 1
- Parity bit: None
- Flow control: None

More demos and projects are available in the STM32Cube folder.
