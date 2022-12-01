'''
This program outputs raw gyro data to be used to calculate its variance needed for the kalman filter
formulation. 

'''

# Import packages
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import GyroSensor
from pybricks.parameters import Port, Color
from pybricks.media.ev3dev import SoundFile, ImageFile
from pybricks.tools import wait, DataLog, StopWatch

# Initialize the EV3 brick
ev3 = EV3Brick()

# Initialize gyro sensor
gyro_sensor = GyroSensor(Port.S2)
gyro_speed = gyro_sensor.speed()

raw_gyro = DataLog('time','raw gyro') # data log
data_timer = StopWatch() # timer

while True:
    try:
        ev3.screen.load_image(ImageFile.SLEEPING)
        wait(3000) # wait 3 seconds
        ev3.screen.load_image(ImageFile.AWAKE)
        ev3.light.on(Color.GREEN)

        # Reset timer and start writing data to csv file
        data_timer.reset()

        raw_gyro.log(gyro_speed, data_timer.time())

    except KeyboardInterrupt:
        ev3.light.on(Color.RED)
        ev3.screen.load_image(ImageFile.KNOCKED_OUT)
        ev3.speaker.play_file(SoundFile.SPEED_DOWN)

# Adjust program intro to include raw wheel data if doing that as well