# Lego EV3 Segway
A segway robot is built with the LEGO MINDSTORMS EV3 robot kit and the EV3 Gyro sensor. The self-balancing code is written in Python using EV3 MicroPython which runs on top of the ev3dev Operating System (OS).

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
- [EV3 Gyro Sensor](https://raisingrobots.com/product/gyro-sensor/)
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

### MQTT and Node-RED setup

Messaging Queuing Telemetry Transport (MQTT) is a protocol commonly used for message exchange between things namely devices, sensors, devices, computers etc. It uses a [publish and subscribe architecture](https://ably.com/topic/pub-sub) such that a device, an MQTT client, can publish a message on a topic to an MQTT broker and other MQTT client(s) subscribe to the topic to receive the message.

The MQTT broker acts as a middleman to facilitate communication between devices by dispatching messages published on a topic from one client to other client(s) that subscribe on the same topic. A client can be both a publisher and subscriber. 

MQTT messages contain a payload with the data to be consumed by the client, in the case of the LEGO segway, the payload will contain commands, for instance **"TURN LEFT"**, to control the segway. Payloads are usually written in JSON format.

In this project, the MQTT broker and one MQTT client, Node-RED, are installed on the PC. The mosquitto MQTT broker is used and can be installed on a number of operating systems, distributions, or platforms; mosquitto and can be downloaded from [here](https://mosquitto.org/download/). 

Node-RED is a flow-based programming tool that makes the process of connecting hardware devices, APIs and online services easier. It has built-in support for MQTT and similar to the MQTT broker, it can be installed on a number of operating systems or platforms. Node-RED can be downloaded [here](https://nodered.org/docs/getting-started/local), Ubuntu is the operating system used for this project. Node-RED was installed with the Raspberry Pi [bash script](https://nodered.org/docs/getting-started/raspberrypi) as both Ubuntu and Raspberry Pi OS are Debian-based operating systems.

To run Node-RED, open a terminal window and type in this command:
```
node-red
```

This will spin up a local server which can be accessed by opening up the link in a browser. From the screenshot below, Node-RED is available at http://127.0.0.1:1880/.

<p align='center'>
  <img src=docs/images/nodered_terminal.png width=500>
</p>

Clicking on this link opens the Node-RED editor:

<p align='center'>
  <img src=docs/images/nodered_homepage.png>
</p>

In the case of Ubuntu, once the mosquitto MQTT broker is installed, the broker automatically starts as a systemd service. This can be confirmed by executing this command:

```
systemctl status mosquitto.service
```

Which will output:

<p align='center'>
  <img src=docs/images/mosquitto_service.png width=500>
</p>

To stop the mosquitto service run this command:

```
systemctl stop mosquitto.service
```

And to start it up again:

```
systemctl start mosquitto.service
```

To create a connection between the Node-RED MQTT client and mosquitto, search for the following nodes at the top left corner of the Node-RED editor and drag them into the flow workspace: *inject*, *mqtt in*, *mqtt out* and *debug*. Then wire (or connect) the debug and mqtt_in nodes by clicking the grey box (port) of the *mqtt in* node and dragging the wire to the debug node. Likewise, do the same for the *inject* node to the *mqtt out* node. The comment nodes below were added to explain what functionalities were being tested.

<p align='center'>
  <img src=docs/images/nodered_test1.png>
</p>

#### MQTT publisher

The installed mosquitto broker comes with MQTT clients that can be used to publish (`mosquitto_pub`) and subscribe (`mosquitto_sub`) to topics from a terminal window. Before demonstrating publishing in MQTT, the MQTT broker needs to be set up and this will be done in Node-RED.

First, double click on the *mqtt in* node and on the right of the *Server* field click on edit button.

<p align='center'>
  <img src=docs/images/1_mqtt_in.png width=400>
</p>

Input a name for the broker, the IP address of the PC/computer the broker is installed on, then click on the *Add* button.

<p align='center'>
  <img src=docs/images/2_mqtt_in.png width=400>
</p>

Next add a topic name, in this case **'test/topic'** was chosen, then click on *Done*. Afterwards click on the *mqtt out* node and choose the broker that was just set up and input the topic name as well. Confirm these inputs by clicking on *Done*.

<p align='center'>
  <img src=docs/images/3_mqtt_in.png width=400>
</p>

To effect the changes made, click on the *Deploy* button at the top right corner of the Node-RED tab.

<p align='left'>
  <img src=docs/images/deploy.png width=200>
</p>

A "connected" text is shown under the *mqtt in* node to signify that a connection has been established with the mosquitto broker.

<p align='center'>
  <img src=docs/images/mqtt_connected.png width=400>
</p>

To see messages in Node-RED, click on the 'debug' logo as shown below.

<p align='center'>
  <img src=docs/images/debug_panel.png width=300>
</p>

To demonstrate publishing with MQTT, open up a terminal and publish the message **"Hello World"** on the topic **'test/topic'** using the `mosquitto_pub` client:

```
mosquitto_pub -t test/topic -m "Hello World"
```

This message is seen in the debug panel of Node-RED.

<p align='center'>
  <img src=docs/images/nodered_pub_test.png width=800>
</p>

#### MQTT subscriber

Double click on the *mqtt out* node and choose the broker that was just set up and type in the topic name as well. Confirm these inputs by clicking on *Done*.

<p align='center'>
  <img src=docs/images/mqtt_out.png width=400>
</p>

Click again on the *Deploy* button to effect these changes.

The `mosquitto_sub` client will be used to demonstrate MQTT subscription. The client will subscribe to the topic **test/topic** to receive any incoming messages published from Node-RED.

Firstly, double click on the *inject* node, choose a name for it, then change the payload type from timestamp to a string.

<p align='center'>
  <img src=docs/images/1_inject_node.png width=400>
</p>

Type in a message to be sent, input the message topic as **topic/test** then click on *Done*.

<p align='center'>
  <img src=docs/images/2_inject_node.png width=400>
</p>

To subscribe to any messages published on the topic/test topic by Node-RED, using the `mosquitto_sub client`, open a terminal and run this command:

```
mosquitto_sub -t test/topic
```

To publish the message from the Node-RED client, click on the button on the left of the `inject` node as shown below.

<p align='center'>
  <img src=docs/images/inject_message.png width=400>
</p>

The published message is received by the `mosquitto_sub` client.

<p align='center'>
  <img src=docs/images/terminal_mqtt_message.png width=400>
</p>

The message is also shown in the Node-RED debug panel. 

<p align='center'>
  <img src=docs/images/publish_message_nodered.png width=800>
</p>

## Main program 

With all the requisite softwares installed and setup, it's time to delve into the main segway program. The code for the segway is modified from the [Gyro Boy project](https://pybricks.com/ev3-micropython/examples/gyro_boy.html) which balances the Gyro Boy on its two wheels by making use of the EV3 Gyro sensor. The following flow chart summarizes the major parts of the program.

<p align='center'>
  <img src=docs/images/main_program.png>
</p>

### Segway connection to MQTT and Node-RED

<p align='center'>
  <img src=docs/images/mosquitto_connection.png>
</p>

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

- [Mosquitto docs](https://mosquitto.org/documentation/)
- [HiveMQ](https://www.hivemq.com/mqtt/)
- [NodeRED](https://nodered.org/about/)
- [TheThingsIndustries](https://www.thethingsindustries.com/docs/integrations/node-red/)