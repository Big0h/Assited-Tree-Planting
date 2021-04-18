import adafruit_bno055
import time
import datetime
import board
import busio
import RPi.GPIO as GPIO
import numpy as np
import serial
import string
import pynmea2
import csv
from threading import Thread

# Run sudo nano /etc/profile
# sudo python /home/pi/Assited-Tree-Planting/planting-script-v4.py &



# Functions required in the script are all included below



def plant_detection():
    # Initilization of the script is now below...
    
    # Initialize the IMU
    i2c = busio.I2C(board.SCL, board.SDA)
    sensor = adafruit_bno055.BNO055_I2C(i2c)

    # Initialize the LED
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    GPIO.setup(18,GPIO.OUT) # Pin 18
    GPIO.output(18,GPIO.HIGH)

    # Set y acceleration low and high thresholds
    accLowThreshold = 5
    accHighThreshold = 20

    # Set timing thesholds to check for values over
    timeCheckLowHigh = 1
    timeCheckAV = 2

    # Set x angular velocity thresholds
    angVThreshold = 1.5

    # Finished initialization
    time.sleep(5)
    GPIO.output(18,GPIO.LOW)

    # Loop until off
    while True:

        # Has a plant just occured?
        GPIO.output(18,GPIO.LOW)
        plant = False

        # Check if the acceleration is below the initial threshold
        if sensor.acceleration[1] < accLowThreshold:

            # If the acceleration is below the threshold, loop until it isn't
            while sensor.acceleration[1] < accLowThreshold:
                continue

            # Sets an end time to check for a planting instance until
            t_end_lh = time.time() + timeCheckLowHigh

            # Loop until the end time
            while time.time() < t_end_lh:

                # Did a plant just occur?
                if plant == True:
                    break

                # Check if the acceleration is above a threshold
                if sensor.acceleration[1] > accHighThreshold:
                    
                    # If the acceleration is above the threshold, loop until it isn't
                    while sensor.acceleration[1] > accHighThreshold:
                        continue

                    # Set a new time end to loop until to check the angular velocity over
                    t_end_av = time.time() + timeCheckAV

                    # Loop until the end time
                    while time.time() < t_end_av:

                        # If the angular velocity is above the threshold, a planting instance has occured!
                        if np.abs(sensor.gyro[0]) > angVThreshold:   

                            # Calls on record location to record GPS location
                            GPS_last = record_location(datetime.datetime.now())

                            # Sets plant to true to return to top of loop
                            plant = True
                            GPIO.output(18,GPIO.HIGH)
                            time.sleep(5)
                            
                            break



# Now for the multi-threading...
Thread(target = plant_detection).start()
