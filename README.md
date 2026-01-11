# 🏎️ Meshmerize: STM32 Maze-Solving Robot

Meshmerize is a high-performance autonomous robot designed to navigate complex 90-degree mazes and track lines with precision. Built around the powerful STM32F103C8T6 (Blue Pill) microcontroller, it utilizes an 8-channel IR sensor array and implements the Left-Hand Rule (LSR) algorithm to solve mazes.

## 🚀 Features

* **High-Speed Processing:** Powered by the 32-bit ARM Cortex-M3 (STM32) for faster reaction times compared to standard Arduino boards.
* **Maze Solving Algorithm:** Implements the Left-Hand Rule to navigate unknown mazes and finding the shortest path (optional implementation).
* **Precision Motor Control:** Uses the TB6612FNG driver for efficient and granular control of DC motors.
* **8-Sensor Array:** RLS08 array allows for smooth line following using weighted averages or PID control.

## 🛠️ Hardware Components

Here is the list of materials used in this project:

* **Microcontroller:** STM32 model STM32F103C8T6 "Blue Pill"
* **Power:** Buck Converter  to step down the voltage.
* **Motor Driver:** TB6612FNG Dual Motor Driver
* **Motors:** 2x DC geared motors with Encoder (N20 motors with 300rpm)
* **Sensors:** RLS08 IR Sensor
* **Chassis:** 2WD PCB cutout chassis
* **Wheels:** 2x wheels and 2x caster wheels
* **Battery:** 2x Rechargable 600mAh 3.7V Batteries
* **Switch:** 1x regular on and off switch

## 🔌 Circuit & Wiring (Need not to be exactly same as here)

Understanding the connections is crucial for replication. Below is the pin mapping used.

![Schematics Image](images_videos/Schematics%20Image.jpeg)

1. Sensor Array to STM32
    * Sensors 1-8 (Digital/Analog Out): Connected to Pins PA7 through PA0.
    * VCC/GND: Connected to 3.3V/GND respectively from the Blue Pill.

2. Motor Driver (TB6612FNG) to STM32
Connects to the STM32 to control speed (PWM) and direction (Logic High/Low).

* Motor A (Right Motor)
    * PWM Speed: PB6
    * Input 1: PB12
    * Input 2: PB13

* Motor B (Left Motor)
    * PWM Speed: PB7
    * Input 1: PB
    * Input 2: PB15

* Driver Power & Logic
    * STBY: Connect to 3.3V (Keeps driver enabled).
    * VM: Connect to Battery (+) via Switch.
    * VCC: Connect to 3.3V.
    * GND: Connect to Common Ground.

3. Encoders to STM32
Used for measuring distance and precise turns.
    * Left Encoder (A): PB10
    * Right Encoder (A): PB5

4. User Interface
Used for status indication and initiating calibration.
    * User Button: PA0 (Used to start/stop or trigger calibration).
    * Status LED: PB2 (Indicates robot state).
    * Goal LED: PB11 (Indicates when the maze is solved).

*(Note: Adjust pin numbers in config.h if your wiring differs.)*

## 🧠 The Logic: How It Works

1. Line Detection

The robot uses 8 IR sensors.
* White surface: Reflects IR light → Sensor reads HIGH (1).
* Black line: Absorbs IR light → Sensor reads LOW (0).
* By reading the position of the active sensors, the robot calculates an "Error Value" (how far off-center it is).

2. The Left-Hand Rule (LSR)

This is a left-wall-following algorithm used to solve mazes:
* Priority Left: If a left turn is available, take it.
* Straight: If no left turn, but straight is available, go straight.
* Right: If no left or straight, turn right.
* Dead End: If no paths are available, turn around (U-turn).

The robot stores these turns in memory to optimize the path on the second run (Shortest Path logic which it decides only based on the paths it traversed in the first run).

BUT STILL ONE OF THE MAIN LIMITATION OF THIS BOT IS THAT IT DOESN'T COVER THE TRUE SHORTEST PATH OF THE COMPLETE MAZE FOR WHICH WE NEED TO IMPLEMENT AND USE DIFFERENT ALGORITHMS, WHICH I WILL TRY TO IMPLEMENT AND UPDATE HERE WHENEVER I DO IT PROPERLY.

## 💻 Software & Setup

The project is coded in C++ using the **Arduino IDE**.

### Prerequisites:

1. Arduino IDE: Download and install.
2. STM32 Board Manager:
    * Go to 'File > Preferences(Settings)'.
    * Add this URL to "Additional Boards Manager URLs": http://dan.drown.org/stm32duino/package_STM32duino_index.json
    * Go to 'Tools > Board > Boards Manager', search for "STM32", and install.

## Installation:

1. Clone this repository by pasting the given command in your terminal or gitbash 'git clone https://github.com/your-username/meshmerize.git'
2. Open just_code.ino in Arduino IDE
3. Board Settings:
    * Board: Generic STM32F1 series
    * Part Number: BluePill F103C8
    * Upload Method: STLink (if using dongle) or Serial (if using FTDI).
4. Connect your STM32 and hit Upload.

## ⚙️ Calibration & Tuning

Before running the maze, you must calibrate the sensors for your specific environment (lighting conditions).

1. Sensor Threshold: Edit config.h to set the black/white threshold values.
2. PID Tuning (if applicable):
    * Increase Kp (Proportional) until the robot oscillates slightly.
    * Increase Kd (Derivative) to dampen the oscillation.
    * Set Ki (Integral) to 0 (usually not needed for line following).

*Tip: Ensure your battery is fully charged. Lower voltage can affect sensor accuracy and motor torque.*

## Gallery

![Top Image](images_videos/IMG_0701.png)
![Bottom Image](images_videos/IMG_0703.png)
![Side Image](images_videos/IMG_0704.png)
![Demo Video](images_videos/demo_video.mov)

## Contributing

Contributions are welcome! If you have a better maze-solving algorithm or hardware optimization:
    * Fork the Project.
    * Create your Feature Branch.
    * Commit your Changes.
    * Open a Pull Request.