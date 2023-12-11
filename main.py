#!/usr/bin/env pybricks-micropython

'''
This program balances a Lego Segway robot built using the LEGO Mindstorms EV3 home edition and a gyroscopic sensor.
The robot can be controlled in two ways:

- directional control from a NodeRED flow with the Segway as an MQTT client. Commands to move forward, backward, turn left or right
can be sent to the Segway via the MQTT broker.

- tether control using the EV3 infrared sensor and beacon. In this mode, the Segway follows the beacon by first rotating (using a Proportional
controller) to reduce the angle between them to approximately 10 degrees, then it translates towards the beacon (using a Proportial Derivative
controller) until it gets close to it then stops.

The code is modified from: https://pybricks.com/ev3-micropython/examples/gyro_boy.html

The robot build instructions are available here: https://robotsquare.com/2014/06/23/tutorial-building-balanc3r/
'''

from ucollections import namedtuple
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, InfraredSensor, GyroSensor
from pybricks.parameters import Port, Color, Button
from pybricks.media.ev3dev import SoundFile, ImageFile, Font
from pybricks.tools import wait, StopWatch 

from umqtt.simple import MQTTClient

# Initialize the EV3 brick
ev3 = EV3Brick()

# Initialize motors at port A and C
right_motor, left_motor = Motor(Port.A), Motor(Port.C)

# Initialize gyro and infrared sensors
gyro_sensor, infrared_sensor = GyroSensor(Port.S2), InfraredSensor(Port.S3)

# Initialize timers
single_loop_timer = StopWatch()
control_loop_timer = StopWatch()
fall_timer = StopWatch()
action_timer = StopWatch()

# Initialize program constants
GYRO_CALIBRATION_LOOP_COUNT = 200   # Number of iterations for gyro calibration
GYRO_OFFSET_FACTOR = 0.0005         # Gyro offset factor (obtained from GyroBoy project)
TARGET_LOOP_PERIOD = 20             # 20 milliseconds
prev_error = 0                      # Initial previous error for beacon derivative controller

# Robot action definition used to change how the robot drives
Action = namedtuple('Action', ['drive_speed', 'steering'])

# Pre-defined robot actions
FORWARD = Action(drive_speed=150, steering=0)
BACKWARD = Action(drive_speed=-60, steering=0)
TURN_LEFT = Action(drive_speed=0, steering=80)
TURN_RIGHT = Action(drive_speed=0, steering=-80)
STOP = Action(drive_speed=0, steering=0)

# Dictionary of initial state of commands from NodeRED
Node_RED_Command = {
    'move_forward': False,
    'move_backward': False,
    'turn_right': False,
    'turn_left': False,
}

# MQTT callback function
def get_commands(topic, msg):
    if msg.decode() == "FORWARD":
        Node_RED_Command['move_forward'] = True

    if msg.decode() == "BACKWARD":
        Node_RED_Command['move_backward'] = True

    if msg.decode() == "TURN_LEFT":
        ev3.screen.load_image(ImageFile.MIDDLE_RIGHT)
        Node_RED_Command['turn_left'] = True

    if msg.decode() == "TURN_RIGHT":
        ev3.screen.load_image(ImageFile.MIDDLE_LEFT)
        Node_RED_Command['turn_right'] = True

# MQTT connection setup
MQTT_ClientID = 'Segway'
SERVER = ''     # Populate with your own details
PORT = 1883
USER = ''       # Populate with your own details
PASSWORD = ''   # Populate with your own details
client = MQTTClient(MQTT_ClientID, SERVER, PORT, USER, PASSWORD)
client.connect()

Topic = 'nodered/commands'
client.set_callback(get_commands)
client.publish(Topic, 'Test')
client.subscribe(Topic)

"""
This function checks for messages from Node-RED and if the beacon is on to update the drive speed and
steering values appropriately.

To ensure that no function calls are made that would otherwise affect the control loop time in the main program, 
those calls yield to the control loop while waiting for a certain thing to happen like this:

    while not condition:
        yield

Yield is also used to update the drive speed and steering values in the main control loop:

    yield action
"""

def update_action():
    while True:
        # Check for messages from the MQTT broker
        client.check_msg()

        # MQTT mode
        if Node_RED_Command['move_forward'] == True:
            yield FORWARD 
            while action_timer.time() < 5000:
                yield
            yield STOP
            Node_RED_Command['move_forward'] = False

        if Node_RED_Command['move_backward'] == True:
            yield BACKWARD 
            while action_timer.time() < 4000:
                yield
            yield STOP
            Node_RED_Command['move_backward'] = False

        if Node_RED_Command['turn_left'] == True:
            yield TURN_LEFT 
            while action_timer.time() < 3000:
                yield
            yield STOP
            ev3.screen.load_image(ImageFile.AWAKE)
            Node_RED_Command['turn_left'] = False

        if Node_RED_Command['turn_right'] == True:
            yield TURN_RIGHT 
            while action_timer.time() < 3000:
                yield
            yield STOP
            ev3.screen.load_image(ImageFile.AWAKE)
            Node_RED_Command['turn_right'] = False

        # Beacon mode (on channel 1)
        '''
        beacon method returns relative distance between 0 and 100 (0 being very close
        and 100 means far away) and approximate angle (-75 to 75 degrees)
        between the beacon remote and infrared sensor. Remote has to be on (duh) and set to channel 1
        '''
        relative_distance, angle = infrared_sensor.beacon(1)
        action_timer.reset()
        global prev_error

        if relative_distance == None:
            yield
        else: # that means that beacon is on
            # Should these be declared here?
            angle_error = 0 - angle
            K_angle = 4
            steering = K_angle * angle_error
            action = Action(drive_speed=0, steering=steering)
            yield action

            # if it's within a certain range, then translate
            if angle_error < 10:
                error = 100 - relative_distance
                d_error = (error - prev_error)/action_timer.time()
                K_p, K_d = 6, 2.5

                drive_speed = K_p * error + K_d * d_error
                #
                action = Action(drive_speed=drive_speed, steering=0)
                prev_error = error
                if relative_distance < 10:
                    yield STOP
                else:
                    yield action

# Stops both motors
def stop_motors():
    left_motor.stop()
    right_motor.stop()

# Main loop
while True: 
    try:
        # Calculate current battery voltage
        battery_voltage = (ev3.battery.voltage())/1000

        # Battery warning for voltage less than 7.5V and breaks out of the loop
        if battery_voltage < 7.5:
            ev3.light.on(Color.ORANGE)
            ev3.screen.load_image(ImageFile.DIZZY)
            ev3.speaker.play_file(SoundFile.UH_OH)
            break

        # Sleeping eyes and light off let us know that the robot is waiting for
        # any movement to stop before the program can continue
        ev3.screen.load_image(ImageFile.SLEEPING)
        ev3.light.off()

        # Reset sensors and initialize variables
        left_motor.reset_angle(0)
        right_motor.reset_angle(0)
        fall_timer.reset()

        motor_position_sum, wheel_angle = 0, 0
        motor_position_change = [0, 0, 0, 0]
        drive_speed, steering = 0, 0
        control_loop_counter = 0
        robot_body_angle = -0.2

        # Since update_action() is a generator (it uses "yield" instead of "return") this doesn't actually run update_action() right now but
        # rather prepares it for use later.
        action_task = update_action()

        while True:
            # Calibrate gyro offset - elaborate, the gyro sensor has a max rate of rotation of 440 deg/s
            gyro_min_rate, gyro_max_rate = 440, -440 # deg/s
            gyro_sum = 0
            for _ in range(GYRO_CALIBRATION_LOOP_COUNT): # loop variable
                gyro_sensor_value = gyro_sensor.speed()
                gyro_sum += gyro_sensor_value
                if gyro_sensor_value > gyro_max_rate:
                    gyro_max_rate = gyro_sensor_value
                if gyro_sensor_value < gyro_min_rate:
                    gyro_min_rate = gyro_sensor_value
                wait(5)
            if gyro_max_rate - gyro_min_rate < 2: # Understand the sign notation used and comment on it
                break

        # Therefore, initial offset is
        gyro_offset = gyro_sum / GYRO_CALIBRATION_LOOP_COUNT
        print("Out of gyro loop")

        # Awake eyes and green light let us know that the robot is ready to go!
        ev3.speaker.play_file(SoundFile.SPEED_UP)
        ev3.screen.load_image(ImageFile.AWAKE)
        ev3.light.on(Color.GREEN)
        wait(500)

        # Balancing loop
        while True:
            # This timer measures how long a single loop takes. This will be
            # used to help keep the loop time consistent, even when different
            # actions are happening.
            single_loop_timer.reset()

            # This calculates the average control loop period. This is used in
            # the control feedback calculation instead of the single loop time
            # to filter out random fluctuations
            if control_loop_counter == 0:
                # The first time through the loop, we need to assign a value to
                # avoid dividing by zero later.

                # Dividing by 1000 because default time is in milliseconds
                average_control_loop_period = TARGET_LOOP_PERIOD / 1000
                control_loop_timer.reset()
            else:
                average_control_loop_period = (control_loop_timer.time() / 1000 / control_loop_counter)
            control_loop_counter += 1

            # Calculate robot body angle and speed...explain in detail
            gyro_sensor_value = gyro_sensor.speed()

            # Low pass filter
            gyro_offset *= (1 - GYRO_OFFSET_FACTOR)
            gyro_offset += GYRO_OFFSET_FACTOR * gyro_sensor_value
            robot_body_rate = gyro_sensor_value - gyro_offset
            robot_body_angle += robot_body_rate * average_control_loop_period

            #
            left_motor_angle, right_motor_angle = left_motor.angle(), right_motor.angle()

            # Calculate wheel angle and speed ...explain in detail
            previous_motor_sum = motor_position_sum
            motor_position_sum = left_motor_angle + right_motor_angle
            change = motor_position_sum - previous_motor_sum
            motor_position_change.insert(0, change)
            del motor_position_change[-1]

            wheel_angle += change - drive_speed * average_control_loop_period
            wheel_rate = sum(motor_position_change) / 4 / average_control_loop_period

            # This is the main control feedback calculation
            output_power = (-0.01 * drive_speed) + (1.2 * robot_body_rate +
                                                     28 * robot_body_angle +
                                                     0.075 * wheel_rate +
                                                     0.12 * wheel_angle)

            # Motor limits
            if output_power > 100:
                output_power = 100
            if output_power < -100:
                output_power = -100

            # Drive motors
            left_motor.dc(output_power - 0.1 * steering)
            right_motor.dc(output_power + 0.1 * steering)

            # Check if robot fell down. If the output speed is +/-100% for more
            # than one second, we know that we are no longer balancing properly.

            if abs(output_power) < 100:
                fall_timer.reset()
            elif fall_timer.time() > 1000:
                break

            # This runs update_action() until the next "yield" statement
            action = next(action_task)
            if action is not None:
                drive_speed, steering = action

            # Make sure loop time is at least TARGET_LOOP_PERIOD. The output power
            # calculation above depends on having a certain amount of time in each loop. #
            wait(TARGET_LOOP_PERIOD - single_loop_timer.time())

        # Handle falling over. If we get to this point in this program, it means
        # that the robot fell over

        # Stop all of the motors
        stop_motors()

        # Knocked out eyes and red light let us know that the robot lost its balance
        ev3.light.on(Color.RED)
        ev3.screen.load_image(ImageFile.KNOCKED_OUT)
        ev3.speaker.play_file(SoundFile.SPEED_DOWN)

        # Wait for a few seconds before trying to balance again.
        wait(3000)

    except KeyboardInterrupt:
        break # break out from main loop
        stop_motors()
        client.disconnect()

