# Rotating stand controlled by ATtiny Microcontroller (Attiny13)

This repository contains firmware for controlling a 28byj-48 stepper motor using an ATtiny microcontroller. The project is designed to be power-efficient by utilizing the microcontroller's low-power sleep modes and disabling unused peripherals.

The code is written in C and is intended to be compiled with a toolchain for AVR microcontrollers, such as `avr-gcc`.

## Features

* **Selectable Drive Modes:** Choose between three different control modes at compile time:
    * **Single-Phase (Wave Drive):** Optimized for low power consumption (~148 mA).
    * **Dual-Phase (Full Drive):** Provides higher torque at the cost of increased power consumption (~260 mA).
    * **Half-Step:** Offers smoother motion but has higher power consumption (~200 mA).
* **Low Power Consumption:** The firmware is optimized for low power, with the ATtiny microcontroller consuming only 2.3 mA.
* **Timer-Based Control:** The motor is driven by a Timer/Counter0 interrupt, ensuring precise and reliable stepping.
* **Sleep Mode:** The microcontroller enters an Idle sleep mode between steps to further reduce power consumption.

## Hardware Requirements

* **Microcontroller:** ATtiny (The code is based on an ATtiny, but should be adaptable to other AVR microcontrollers).
* **Stepper Motor:** 28byj-48 (5V)
* **Motor Driver:** ULN2003
* **Clock Source:** The microcontroller is configured to use an internal clock of 4.8 MHz.

### PCB Design Files

The repository includes the PCB design files in the `HW` folder. These files can be used to manufacture the custom circuit board for this project.

***

## Configuration

### Fuses

The ATtiny microcontroller's fuses must be configured as follows to achieve the desired clock speed and low power operation:

* **Low Fuse Byte:** `0x79`
* **High Fuse Byte Byte:** `0xFF`

These settings will:
* Set the internal clock to 4.8 MHz.
* Disable the internal clock division by 8.
* Disable the Brown-out Detection (BODLEVEL).
* Disable the Watchdog Timer.

### Drive Mode Selection

You can select the desired drive mode by modifying the `#define` statements at the top of the `main.c` file. Only one mode should be set to `1` at a time.

```c
// Select control type
#define SINGLE_PHASE_CONTROL (1) // Wave drive - Full step; single-phase control. (Consumption 148 mA)
#define DUAL_PHASE_CONTROL   (0) // Full drive - Full step, but higher consumption because I'm powering 2 coils, and higher force; dual-phase control. (Consumption approx. 260 mA)
#define HALF_STEP            (0) // Half step drive - Half step, higher consumption because I'm powering 2 coils. (Consumption approx. 200 mA)