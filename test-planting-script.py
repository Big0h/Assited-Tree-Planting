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
import os, os.path




# Functions required in the script are all included below


# Function used to record the planting location
def record_data(name, acc,gyro):
    # Need to fix this eventually

    # Followed this tutorial:
    # https://thispointer.com/how-to-append-text-or-lines-to-a-file-in-python/

    # Opens the file
    with open(name, "a+") as file_object:

        # Goes to start of text file
        file_object.seek(0)
        data = file_object.read(5)

        # If file is not empty, start by adding a new line!
        if len(data) > 0:
            file_object.write("\n")

        # Creates a comma delimited string of: date-time, latitude, and longitude
        recording = str(datetime.datetime.time(datetime.datetime.now())) + ", " + str(acc[0]) + ", " + str(acc[1]) + ", " + str(acc[2]) + ", " + str(gyro[0]) + ", " + str(gyro[1]) + ", " + str(gyro[2])

        # Writes the new planting instance to the txt file
        file_object.write(recording)

        # Closes the file
        file_object.close()







# Initilization of the script is now below...

# Initialize the IMU
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055_I2C(i2c)

# Find which testing number this is...
# https://stackoverflow.com/questions/2632205/how-to-count-the-number-of-files-in-a-directory-using-python

DIR = "/home/pi/Assited-Tree-Planting/test_data/"
num = len([name for name in os.listdir(DIR) if os.path.isfile(os.path.join(DIR, name))])
name = DIR + "data_" + str(num) + ".txt"

# Initialize the LED
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(18,GPIO.OUT) # Pin 18
GPIO.output(18,GPIO.HIGH)

# Finished initialization
time.sleep(5)
GPIO.output(18,GPIO.LOW)



# Loop until off
#while True:
#    record_data(name, sensor.acceleration,sensor.gyro)
#    time.sleep(0.02)

# Set the loop to record over 20 seconds
t_end = time.time() + 20

# Loop for t_end time
while time.time() < t_end:
    
    # Record the accelerations and angular velocities
    record_data(name, sensor.acceleration,sensor.gyro)
    time.sleep(0.02)

# Turn on LED for 5 seconds to mark the end of the recording
GPIO.output(18,GPIO.HIGH)
time.sleep(5)
GPIO.output(18,GPIO.LOW)
