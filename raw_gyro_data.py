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

raw_gyro = DataLog('time','raw gyro', name='raw_gyro_data', extension='csv', timestamp=False) # data log
data_timer = StopWatch() # timer

try:
    
    #
    ev3.screen.load_image(ImageFile.SLEEPING)
    wait(2000) # wait 2 seconds
    ev3.screen.load_image(ImageFile.AWAKE)
    ev3.light.on(Color.GREEN)
    
    # Reset timer
    data_timer.reset()

    while True:
        # Obtain latest gyro reading
        gyro_speed = gyro_sensor.speed()

        # Log data
        raw_gyro.log(data_timer.time(), gyro_speed)

except KeyboardInterrupt:
    ev3.light.on(Color.RED)
    ev3.screen.load_image(ImageFile.KNOCKED_OUT)
    ev3.speaker.play_file(SoundFile.SPEED_DOWN)

# Adjust program intro to include raw wheel data if doing that as well
# Comments, comments