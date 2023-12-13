# Lego EV3 Segway
A segway robot is built with the LEGO MINDSTORMS EV3 robot kit and the EV3 Gyroscopic sensor. The self-balancing code is written in Python using EV3 MicroPython: which runs on top of the ev3dev Operating System (OS).

The robot can be controlled in two ways:

- **directional control** from a Node-RED flow with the Segway as an MQTT client. Commands to move forward, backward, turn left or right
can be sent to the Segway via the MQTT broker. This control method is dubbed **MQTT mode**. 

- **tether control** using the EV3 infrared sensor and beacon. In this mode, the Segway follows the beacon by first rotating (using a Proportional
controller) to reduce the angle between them to approximately 10 degrees, then it translates towards the beacon (using a Proportial 
Derivative (PD) controller) until it gets close to it then stops. This control method is dubbed **beacon mode**, which is shown in the following animation.

<p align='center'>
  <img src=docs/images/beacon_mode.gif>
</p>

🚧	***(Documentation work in progress)***

## Hardware

The following components were used for this project:

- LEGO MINDSTORMS EV3 Home Edition #31313
- Gyroscopic Sensor
- [EDIMAX EW-7811Un wireless USB adapter](https://www.edimax.com/edimax/merchandise/merchandise_detail/data/edimax/in/wireless_adapters_n150/ew-7811un/)

The robot build instructions are available [here](https://robotsquare.com/2014/06/23/tutorial-building-balanc3r/).

<p align='center'>
  <img src=docs/images/ev3_segway.jpg width=400>
</p>

## Software 

### Installation

Link setup guide

### Workflow
The code for the Segway is modified from the [GyroBoy project](https://pybricks.com/ev3-micropython/examples/gyro_boy.html).

### MQTT and Node-RED setup
Populate MQTT server details in `main.py`, show relevant lines in the code

## Control Modes

### Directional control with Node-RED

<p align='center'>
  <img src=docs/images/Node-RED-commands.png>
</p>

### Tether control with beacon

## Recommendations

## References