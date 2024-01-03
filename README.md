# Lego EV3 Segway
A segway robot is built with the LEGO MINDSTORMS EV3 robot kit and the EV3 Gyroscopic sensor. The self-balancing code is written in Python using EV3 MicroPython: which runs on top of the ev3dev Operating System (OS).

The robot can be controlled in two ways:

- **directional control** from a Node-RED flow with the segway as an MQTT client. Commands to move forward, backward, turn left or right
can be sent to the Segway via the MQTT broker. This control method is dubbed **MQTT mode**. 

- **tether control** using the EV3 infrared sensor and beacon. In this mode, the segway follows the beacon by first rotating (using a Proportional
controller) to reduce the angle between them to approximately 10 degrees, then it translates towards the beacon (using a Proportial 
Derivative (PD) controller) until it gets close to it then stops. This control method is dubbed **beacon mode**, which is shown in the animation below.

<p align='center'>
  <img src=docs/images/beacon_mode.gif>
</p>

🚧	***(Documentation work in progress)***

## Hardware

The following components were used for this project:

- LEGO MINDSTORMS EV3 Home Edition #31313
- Gyroscopic Sensor
- MicroSD card (larger than 2GB but not greater than 32GB)
- [EDIMAX EW-7811Un wireless USB adapter](https://www.edimax.com/edimax/merchandise/merchandise_detail/data/edimax/in/wireless_adapters_n150/ew-7811un/)
- PC or single board computer (to run the MQTT broker and Node-RED)

The robot build instructions are available [here](https://robotsquare.com/2014/06/23/tutorial-building-balanc3r/).

<p align='center'>
  <img src=docs/images/ev3_segway.jpg width=400>
</p>

## Software Installation

### EV3 MicroPython setup
In order to use EV3 MicroPython, the EV3 MicroPython image file is flashed onto a micro SD card and inserted into the microSD card slot on the side of the EV3 brick. The full installation process is available in this [guide](https://pybricks.com/ev3-micropython/startinstall.html).

In the guide, [Visual Studio Code](https://code.visualstudio.com/Download) is used in writing MicroPython programs and installing the EV3 MicroPython extension. After successfully flashing the image onto the microSD card, insert the card into the slot and turn on the EV3 brick by pressing the dark gray center button. An overview of how to navigate the different menus on the EV3 brick can be found in this [guide](https://pybricks.com/ev3-micropython/startbrick.html).

### Network Connection

The MQTT mode requires a connection between the host PC and the EV3 for messages to be sent and received via the MQTT broker running on the PC. To establish this connection, the EDIMAX EW-7811Un Wi-Fi USB dongle is used. 

Plug in the dongle into the EV3 bricks' USB port, then navigate to the ***Wireless and Networks > Wi-Fi*** menu on the brick. Check the ***"Powered"*** box to enable to Wi-Fi network search then connect to the network the PC is on. It is advisable to set a static IP address for the EV3 on one's router.

There are other networking options which are detailed [here](https://www.ev3dev.org/docs/networking/).

### SSH connection to EV3

With the network connection set up, the next thing is configure an SSH connection to the EV3. This will enable commands to be sent to the robot, over the network, to run programs, change settings and install packages.

This ev3dev [guide](https://www.ev3dev.org/docs/tutorials/connecting-to-ev3dev-with-ssh/) is used to set up the SSH connection which has instructions for MacOS, Ubuntu and Windows.
The SSH connection is also necessary if one wants to use the [Visual Studio Code Remote - SSH extension](https://code.visualstudio.com/docs/remote/ssh) to edit files easily in VS Code. However, after sometime the VS Code remote connection to the robot could not be established, this issue is nonexistent when creating an ssh connection to the robot using a terminal window; hopefully others do not encounter this issue.

### Clone Project Repository

Execute the following the command in a terminal window to create an ssh connection to the robot:
```
ssh robot@ev3dev.local
```

Then run this command to clone the project repository:
```
git clone https://github.com/TheNoobInventor/lego-ev3-segway.git
```

### MQTT installation

Messaging Queuing Telemetry Transport (MQTT) is a protocol commonly used for message exchange between things namely devices, sensors, devices, computers etc. It uses a *publish and subscribe* architecture such that a device, an MQTT client, can publish a message on a topic to an MQTT broker and other MQTT client(s) subscribe to the topic to receive the message.

The MQTT broker acts as a middleman to facilitate communication between devices by dispatching messages published on a topic from one client to other client(s) that subscribe on the same topic. A client can be both a publisher and subscriber. 

MQTT messages contain a payload with the data to be consumed by the client, in the case of the LEGO segway, the payload will contain commands, for instance **"TURN LEFT"**, to control the segway. Payloads are usually written in JSON format.

In this project, the MQTT broker and one MQTT client, Node-RED, are installed on the PC. The other MQTT client is already installed in the EV3 MicroPython image. The mosquitto MQTT broker is used and can be installed on a number of operating systems, distributions, or platforms; mosquitto and can be downloaded from [here](https://mosquitto.org/download/). The image below shows the connection between the MQTT clients and mosquitto.

<p align='center'>
  <img src=docs/images/mosquitto_connection.png>
</p>

### Node-RED installation

## Main program 

<p align='center'>
  <img src=docs/images/main_program.png>
</p>


### MQTT and Node-RED setup


### Control Modes

<p align='center'>
  <img src=docs/images/update_action.png>
</p>


### MQTT mode for directional control with Node-RED
<p align='center'>
  <img src=docs/images/mqtt_mode.png>
</p>


<p align='center'>
  <img src=docs/images/Node-RED-commands.png>
</p>

### Beacon mode for tether control with beacon

<p align='center'>
  <img src=docs/images/beacon_mode.png>
</p>


## Walkthrough video

## Recommendations

## References

- [HiveMQ](https://www.hivemq.com/mqtt/)