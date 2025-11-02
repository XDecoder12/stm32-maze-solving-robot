# 🤖 STM32 Maze-Solving & Line-Following Robot

A maze-solving robot based on the STM32 microcontroller, designed to follow a line and navigate a maze. This project is coded using the Arduino IDE.

## 🚀 Overview

This repository contains the full source code, hardware schematics, and documentation for building your own maze-solving robot. The robot uses 8 IR sensors to detect the line and LSR or Left-Hand-Rule Algorithm to solve the maze.

## 🛠️ Hardware Components

Here is the list of materials used in this project:

* **Microcontroller:** STM32 model STM32F103C8T6 "Blue Pill"
* **Power:** Buck Converter  to step down the voltage.
* **Motor Driver:** TB6612FNG Dual Motor Driver
* **Motors:** 2x DC geared motors
* **Sensors:** RLS08 IR Sensor
* **Chassis:** 2WD PCB cutout chassis
* **Wheels:** 2x wheels and 2x caster wheels
* **Battery:** 2x Rechargable 600mAh AA Batteries

### Breadboard Layout

## 💻 Software & Setup

The project is coded in C++ using the **Arduino IDE**.

### Required Libraries:

* **STM32 Core:** You must add the STM32 board manager to your Arduino IDE.
* The only external library used is the config.h file.

### How to Compile:

1.  Open the `.ino` file in the Arduino IDE.
2.  Select the correct board (e.g., "Generic STM32F1 series").
3.  Set the Upload Method (e.g., "STLink" or "Serial").
4.  Connect your STM32 to the computer.
5.  Press the "Upload" button.
