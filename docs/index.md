# Lego EV3 Segway
A segway robot is built with the LEGO MINDSTORMS EV3 robot kit and the EV3 Gyroscopic sensor. The self-balancing code is written in Python using EV3 MicroPython: which runs on top of the ev3dev Operating System (OS).

Give a bit of background about segways here [think this should be the first sentence or paragraph...or?]

## Hardware

### Components

The following components are needed to setup this project:

- LEGO MINDSTORMS EV3 Home Edition #31313
- Gyroscopic Sensor 

### Assembly

The Segway robot was built following this [guide](https://robotsquare.com/2014/07/01/tutorial-ev3-self-balancing-robot/).

Actually this guide: https://www.youtube.com/watch?v=Jx9VuX15nqI. The robot square link is broken. 

Be mindful that the color of some of the parts might be different. And it doesn't show how to connect the infrared sensor. Try and make it out using this link: https://www.youtube.com/watch?v=-dcGsilEEXQ

or should i decouple this part and show them how to assemble it?

Show better pictures

<p align="center">
  <img src=images/ev3_segway.jpg>
</p>

## Software

### Software architecture

Visual Studio Code

Software used

### Software install
how to download and run the code [ev3 micropython](https://pybricks.com/ev3-micropython/startinstall.html)

state version of ev3 micropython we are using

## Working principle

Need to heavily reference Gyro Boy. Should I mention it in the intro for main.py as well?

## Video demonstration

### Channel 1

### Channel 2

## Observations
Some intermittent errors running the program - elaborate, possible fix?

Occasional delay when calibrating gyro sensor. Why?

Issue with my remote - code not picking up two or more button presses correctly with repeatability.

Gyro sensor is retired...suggest alternative

Kit from 2016..age an issue?

## Future work/suggestions
Upgrades to channel two: 
if an obstacle is in its way, it will have to go back and turn left or right and proceed onto the driving to the beacon
 
- For beacon channel 3 - moving around randomly (say for 5 seconds) and avoiding obstacles. Make your robot drive around a room while avoiding obstacles with the Infrared Sensor in Proximity mode.

- For beacon channel 4 - actively avoiding objects that come too close.

## References
- [Gyro Boy](https://pybricks.github.io/ev3-micropython/examples/gyro_boy.html)

- [Ev3dev Micropython](https://www.ev3dev.org/news/2019/04/13/ev3-micropython/)
