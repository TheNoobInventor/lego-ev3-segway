# lego-ev3-segway

(To be completed)

Using ev3dev OS to self-balance a Lego Mindstorms EV3 robot. 


# Building the Segway robot

The Segway robot was build following this [guide](https://robotsquare.com/2014/07/01/tutorial-ev3-self-balancing-robot/).

<p align="center">
  <img src=images/ev3_segway.jpg>
</p>

## Future work

Upgrades to channel two: 
if an obstacle is in its way, it will have to go back and turn left or right and proceed onto the driving to the beacon
 
- For beacon channel 3 - moving around randomly (say for 5 seconds) and avoiding obstacles. Make your robot drive around a room while avoiding obstacles with the Infrared Sensor in Proximity mode.
- For beacon channel 4 - actively avoiding objects that come too close.

Combine channels 3 and 4?

Note about some intermittent errors running the program - what error and how to fix it.

Occasional delay when calibrating gyro sensor. Why?

## Reference

https://pybricks.github.io/ev3-micropython/examples/gyro_boy.html
