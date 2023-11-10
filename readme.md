# GPIO extender over I2C protocol

This project has the main propose to work as gpio expander over I2C protocol. It uses the mcu STM32F103C8T6 with cortex-M3. Here, we have the make file to build, flash e monitor the project. 

# Dependencies

- st link to flash, debug and traceswo

Follow the link: https://github.com/stlink-org/stlink

```
apt install stlink-tools gdb-multiarch openocd
```
Install arm-none-eabi compiler available on ARM oficial site.

Go to session x86_64 Linux hosted cross toolchains and choose **AArch32 bare-metal target (arm-none-eabi)**

# Debugger test with OpenOCD and gdb

Test the openocd and st-link V2 with the following command:

```
$  openocd -f interface/stlink.cfg -f target/stm32f1x.cfg
```

Run the gdb

```
$ gdb-multiarch build/filename.elf
```

And type

```
(gdb) target extended-remote localhost:3333
```

## OpenOCD debugger xpm (not used. Skip it!) 20230124

Follow the instructions into https://github.com/xpack-dev-tools/openocd


Install the latest xpm (https://xpack.github.io/xpm/install/)
```
sudo npm install --global xpm@latest
```

install xpack-dev-tools https://xpack.github.io/openocd/install/
https://www.npmjs.com/package/@xpack-dev-tools/openocd for eclipse use

xpm install --global @xpack-dev-tools/openocd@latest --verbose

or uninstall

```
xpm uninstall --global @xpack-dev-tools/openocd --verbose

sudo npm uninstall --global xpm
```

[arm-gnu-toolchain-12.2.rel1-x86_64-arm-none-eabi.tar.xz](https://developer.arm.com/-/media/Files/downloads/gnu/12.2.rel1/binrel/arm-gnu-toolchain-12.2.rel1-x86_64-arm-none-eabi.tar.xz?rev=7bd049b7a3034e64885fa1a71c12f91d&hash=2C60D7D4E432953DB65C4AA2E7129304F9CD05BF)

# Get started

First of all you should have one ST-Link V2 and make the properly connection.

| ST-Link V2  | STM32F103C8T6 |  
|-----------  |---------------|
| +3V3        | Vcc pad       |
| GND         | GND pad       |
| SWDIO       | PA13          |
| SWCLK       | PA14          |
| TRACESWO    | PB3           |  

$ make  
$ make flash  
$ make monitor  

# I2C parameters

The device works on I2C slave mode and has the following parameters

Device address: 0x23  
Frequency: 400 kHz

## Operation codes like as PCF8575


0b 0000 0000

| set pin |
| 0x03

| reset pin | 
| 0x
reset pin

# I2C commands

- probe
- soft reset
- config
- put
- get
- get temperature
- uptime
- reset reason
- Irms (dsp function)
- array data receive

## launch.json example

{
    "version": "0.2.0",
    "projectName": "i2c_to_gpio",
    "configurations": [
        {
            "name": "ST-LINK V2 DEBUGGER",
            "type": "cortex-debug",
            "request": "launch",
            "cwd": "${workspaceRoot}",
            "servertype": "openocd",
            "executable": "${workspaceFolder}/i2c_to_gpio/build/i2c_to_gpio.elf",
            "configFiles": [
                "interface/stlink.cfg",
                "target/stm32f1x.cfg"
            ],
            // "device": "STM32F103C8T6",
            "interface": "swd",
            "runToEntryPoint": "main",
            // "svdFile": "${workspaceFolder}/i2c_to_gpio/STM32F103.svd", // Include svd to watch device peripherals
            "preLaunchCommands": [
                "set mem inaccessible-by-default off",
                "monitor reset"
            ],
            "postLaunchCommands": [
                "monitor reset init",
                "monitor sleep 200"
            ]
            // "swoConfig":
            // {
            //     "enabled": true,
            //     "cpuFrequency": 72000000,
            //     "swoFrequency": 4000000,
            //     "source": "probe",
            //     "decoders":
            //     [
            //         {
            //             "label": "ITM port 0 output",
            //             "type": "console",
            //             "port": 0,
            //             "showOnStartup": true,
            //             "encoding": "ascii"
            //         }
            //     ]
            // }
        }
    ]
}
