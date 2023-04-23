# Calorie and Step Tracker

The Calorie and Step Tracker is an all-in-one fitness tracking system that utilizes an STM32L476RG microcontroller, MPU-9250 9DOF sensor, MAX30101 heart-rate sensor, and an SSD1306 OLED display to track steps, measure heart rate, and estimate calories burned during physical activity. This repository contains all necessary source code, schematics, and documentation to help you build and customize your own calorie and step tracker.

## Features

- Real-time step tracking using the MPU-9250 9DOF sensor
- Continuous heart rate monitoring with the MAX30101 high-sensitivity pulse oximeter and heart-rate sensor
- Estimation of calories burned based on the collected data
- OLED display (SSD1306) to show the elapsed time, steps taken, average heart rate, and calories burned

## Getting Started

### Prerequisites

- STM32L476RG Microcontroller
- MPU 9DOF Click carrying a MPU-9250 System in Package
- Heart Rate 4 Click carrying the MAX30101 high-sensitivity pulse oximiter and heart-rate sensor
- OLED Display (SSD1306)

### Hardware Setup

1. Assemble the hardware components following the provided schematics.
2. Connect the OLED display, MPU-9250 sensor, and MAX30101 sensor to the STM32L476RG microcontroller.

### Software Setup

1. Clone this repository to your local machine.
2. Install the necessary software tools and libraries for your microcontroller and sensors.
3. Compile and upload the source code to your microcontroller.

## Usage

Turn on the Calorie and Step Tracker, and the OLED display will show the real-time data for the elapsed time, steps taken, average heart rate, and calories burned.

