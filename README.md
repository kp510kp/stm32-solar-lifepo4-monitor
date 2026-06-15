# STM32 Solar LiFePO4 Battery Monitor

This repository contains the STM32 firmware developed for a solar LiFePO4 battery monitoring system. The project uses an STM32 NUCLEO-L476RG microcontroller to acquire battery voltage and current data, display live values on an ILI9488 TFT screen, and log calibrated measurements to a CSV file on an SD card.

## Main Features

* STM32CubeIDE HAL-based firmware
* ADS1115 I2C ADC voltage measurement
* INA226 current and bus-voltage measurement
* Calibration constants applied in firmware
* ILI9488 SPI TFT display interface
* FatFs-based SD-card CSV logging
* UART debug output
* Time-based sampling loop for regular data acquisition

## Hardware Used

* STM32 NUCLEO-L476RG
* DFRobot ADS1115 16-bit I2C ADC module
* INA226 current and bus-voltage sensor
* DFR0669 ILI9488 TFT display module
* SPI microSD card module
* LiFePO4 battery monitoring prototype board

## Repository Contents

* `Core/Src/main.c` – main STM32 application firmware
* `Core/Src/ili9488.c` and `Core/Inc/ili9488.h` – TFT display driver files
* `fatfs_sd.c` and `fatfs_sd.h` – SD-card interface files
* `.ioc` file – STM32CubeIDE peripheral configuration

## Project Purpose

The firmware was developed as part of an ENGEN508 Master’s capstone project. The aim was to replace the Raspberry Pi-based monitoring approach with an STM32 microcontroller-based digital acquisition and logging system for more direct embedded control.

