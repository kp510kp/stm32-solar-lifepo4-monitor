# STM32 Solar LiFePO4 Battery Monitor

## Overview

This repository contains the STM32 firmware developed for a solar LiFePO4 battery monitoring system. The project was completed as part of the ENGEN508 Master of Engineering Practice capstone project at the University of Waikato.

The system uses an STM32 NUCLEO-L476RG microcontroller to acquire battery voltage and current measurements, display real-time data on an ILI9488 TFT display, and store calibrated measurements on a microSD card for long-term battery monitoring and analysis.

The project was developed as an embedded microcontroller-based alternative to previous Raspberry Pi monitoring systems, providing deterministic timing, local data logging, and improved system reliability.

---

## Main Features

* STM32CubeIDE HAL-based firmware
* ADS1115 16-bit I²C ADC voltage measurement
* INA226 current and bus-voltage monitoring
* Software calibration using experimentally derived gain and offset corrections
* ILI9488 SPI TFT graphical display
* FatFs-based SD-card CSV data logging
* UART serial debugging interface
* Time-based sampling and acquisition loop
* Automatic log file generation
* Low-voltage protection and monitoring functions

---

## Hardware Used

* STM32 NUCLEO-L476RG
* DFRobot ADS1115 16-bit I²C ADC module
* INA226 current and bus-voltage sensor
* DFR0669 ILI9488 TFT display module
* SPI microSD card interface
* LiFePO4 battery monitoring analogue prototype board

---

## Repository Structure

```text
Core/
├── Src/
│   ├── main.c
│   ├── ili9488.c
│   └── fatfs_sd.c
│
└── Inc/
    ├── ili9488.h
    └── fatfs_sd.h

SD CARD TESTING.ioc
README.md
```

---

## Firmware Functions

### Voltage Measurement

Battery voltage is measured using the ADS1115 ADC through the I²C interface. Calibration constants are applied in firmware to improve measurement accuracy.

### Current Measurement

Current and bus voltage are measured using the INA226 sensor. Experimental calibration was performed using laboratory reference instruments.

### TFT Display

The ILI9488 display provides real-time visualization of battery voltage, current, and system status.

### SD Card Logging

Measurement data are recorded to CSV files using the FatFs file system. A new log file is automatically generated when the system starts.

---

## Project Purpose

The objective of this project was to develop a reliable STM32-based battery monitoring platform capable of:

* Continuous battery data acquisition
* Real-time visualization
* Local SD-card storage
* Calibration and measurement validation
* Supporting future battery modelling and State-of-Charge research

---

## Author

**Krunal Patel**
Master of Engineering Practice (Electronics)
University of Waikato
Hamilton, New Zealand

---

## License

This repository is provided for academic and research purposes.
