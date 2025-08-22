# Servo Controller ROS Package

Author: Treggon Owens

This package provides ROS integration for controlling a **16-channel PCA9685 servo board** via an Arduino Uno. It also reads **two PWM input channels** and provides a heartbeat mechanism for safety.

---

## Features

- Control 16 servo channels individually.
- Enable/disable each servo with output enable safety.
- Read two PWM input channels from the Arduino.
- Heartbeat mechanism disables servos if serial commands are lost.
- Python GUI for manual control.
- Compatible with ROS Noetic and rosserial.

---

## Directory Structure

```
servo_controller/
├── arduino/
│   └── servo_controller.ino
├── msg/
│   └── ServoArray.msg
├── scripts/
│   ├── servo_controller_node.py
│   └── servo_controller_gui.py
├── CMakeLists.txt
├── package.xml
└── README.md
```

---

## Installation

1. Clone or copy this package into your `~/catkin_ws/src` directory.
2. Build the workspace:

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

3. Install Arduino dependencies via Arduino IDE Library Manager:
   - Adafruit PWM Servo Driver

---

## Running

### Arduino

1. Open `arduino/servo_controller.ino` in Arduino IDE.
2. Select board: Arduino Uno.
3. Check pinouts, wire up the +5, GND, SCK, SDA, OE, PWM1, PWM2
4. Change .ino file to match UNO pinout
5. Upload the sketch.
6. Start rosserial:

```bash
rosrun rosserial_python serial_node.py /dev/ttyACM0
```

### Python Command Node

```bash
rosrun servo_controller servo_controller_node.py
```

### Python GUI Node

```bash
rosrun servo_controller servo_controller_gui.py
```

---

## Notes

- PWM inputs are read in microseconds.
- Heartbeat disables outputs after 1 second of no commands.
- Compatible with ROS Noetic on Ubuntu 20.04+.
