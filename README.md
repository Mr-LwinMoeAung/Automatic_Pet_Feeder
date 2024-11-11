# Automatic Pet Feeder

This project is an automatic pet feeder system designed to dispense food for pets at regular intervals. The system includes a timer mechanism to control food dispensing and avoid accidental triggers by the pet. It utilizes an STM32 microcontroller, with peripherals like timers, I2C, an ultrasonic sensor, a PIR sensor, and an LCD to manage the feeding mechanism and control the motor.

## Features

- **Automated Food Dispensing**: Uses a stepper motor controlled by a timer to dispense food at scheduled intervals.
- **Anti-Trigger Mechanism**: Includes a delay feature that prevents food from being dispensed if the pet only briefly approaches the feeder.
- **Food Level Detection**: Uses an ultrasonic sensor to check if there is enough food in the container, preventing empty feeding cycles.
- **Proximity and Motion Sensing**: Equipped with a PIR sensor to detect the pet’s presence and prevent accidental triggering.
- **LCD Status Display**: Displays conditions such as detecting food, dispensing food, and no food, providing real-time feedback.

## Components

- **STM32F7 Microcontroller**: Main processing unit to control the pet feeder system.
- **Stepper Motor**: Controls food dispensing with accurate positioning.
- **Timers (TIM1 and TIM2)**: Manages the scheduling and timing of the motor to release food at set intervals.
- **I2C Interface**: Enables communication with sensors or other peripheral devices.
- **Ultrasonic Sensor**: Detects if there is enough food in the container, preventing operation when the container is empty.
- **PIR Sensor**: Senses the pet’s motion, ensuring food is dispensed only when the pet is actively approaching the feeder.
- **LCD Display**: Shows the current status of the feeder, including conditions like detecting food, dispensing food, and no food.

## Setup and Configuration

### Hardware Setup

1. Connect the stepper motor to the STM32F767ZI microcontroller.
2. Wire the ultrasonic sensor for food level detection and the PIR sensor for pet motion sensing to the STM32F767ZI microcontroller, ensuring correct power and signal connections.
3. Connect the LCD display via I2C or GPIO pins to provide feedback on system status.
4. Ensure proper wiring for I2C if using sensors or a display.

### Software Configuration

1. The project relies on STM32 HAL (Hardware Abstraction Layer) libraries.
2. `tim.c` configures TIM1 and TIM2, with TIM2 specifically in base mode to control the motor.
3. The ultrasonic sensor monitors the food level, while the PIR sensor detects pet presence, and the LCD displays the current system status.
4. Adjust the `tim.c` file if you need different timing or frequency settings.

## File Structure

- `main.c`: Main application code where the program flow and logic are implemented.
- `tim.c` & `tim.h`: Timer configuration files, used to set up and control TIM1 and TIM2.
- `i2c.c` & `i2c.h`: I2C configuration files for managing any I2C peripherals.
- `lcd.c` & `lcd.h`: LCD configuration files to handle the display of feeder status.
- `stm32f7xx_hal_msp.c`: Initializes the STM32 microcontroller’s HAL configurations.
- `syscalls.c` & `sysmem.c`: Handle system calls and memory management.
- `Makefile`: Allows for easy project build and compilation using make.

## Usage

### Compile and Flash

1. Use the provided Makefile to compile the code:
   ```bash
   make
2. Flash the compiled binary to the STM32F767ZI microcontroller

## Operation

1. Once powered on, the feeder will dispense food at the programmed interval.
2. The ultrasonic sensor ensures there is enough food in the container, while the PIR sensor ensures food is only dispensed when the pet approaches for an adequate duration.
3. The LCD displays the current status, such as "Detecting Food," "Dispensing Food," or "No Food."

## Troubleshooting
1. Motor Does Not Rotate: Check connections to the stepper motor and verify timer settings in tim.c.
2. Dispensing Interval Issues: Adjust the timer prescaler and counter in tim.c to modify dispensing frequency.
3. Food Getting Stuck: Ensure the container design allows smooth flow and verify the motor's torque.
4. Sensor Issues: Verify sensor connections and ensure they are receiving the correct input voltage.
5. LCD Not Displaying: Check I2C or GPIO connections to the LCD and verify the initialization settings in lcd.c.
