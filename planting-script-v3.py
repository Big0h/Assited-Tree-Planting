import adafruit_bno055
import time
import datetime
import board
import busio
import RPi.GPIO as GPIO

# Need to:
# - Ensure that the IMU is being properly instantiated
# - Figure out which axes we need for acceleration and angular velocity
# - Figure out the thresholds
# - Integrate the GPS locations to record_location function



# Function used to record the planting location
def record_location(date):
    # Need to fix this eventually
    GPS = [0, 0]

    # Followed this tutorial:
    # https://thispointer.com/how-to-append-text-or-lines-to-a-file-in-python/
    name = str(datetime.date.today()) + ".txt"

    # Opens the file
    with open(name, "a+") as file_object:

        # Goes to start of text file
        file_object.seek(0)
        data = file_object.read(5)

        # If file is not empty, start by adding a new line!
        if len(data) > 0:
            file_object.write("\n")

        # Creates a comma delimited string of: date-time, longitude, and latitude
        recording = str(datetime.datetime.time(datetime.datetime.now())) + ", " + str(GPS[0]) + ", " + str(GPS[1])

        # Writes the new planting instance to the txt file
        file_object.write(recording)

        # Closes the file
        file_object.close()

        # Return the last GPS location
        return GPS


# Function used to measure distance between previous planting instance and current location
def distance(lat1, lon1, lat2, lon2):
    # Code taken from:
    # https://stackoverflow.com/questions/639695/how-to-convert-latitude-or-longitude-to-meters
    # Which was then conveted to Python.

    R = 6378.137 # Radius of earth in KM
    dLat = lat2*np.pi/180 - lat1*np.pi/180
    dLon = lon2*np.pi/180 - lon1*np.pi/180
    a = np.sin(dLat/2)*np.sin(dLat/2) + np.cos(lat1*np.pi/180)*np.cos(lat2*np.pi/180)*np.sin(dLon/2)*np.sin(dLon/2)
    c = 2*np.atan2(np.sqrt(a),np.sqrt(1 - a))
    d = R*c
    return d * 1000 # Return value in meters


# Initialize the IMU
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055_I2C(i2c)

# Initialize the LED
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(18,GPIO.OUT) # Pin 18

# Initialize GPS location
GPS = [0,0]
GPS_last = [0,0]

# Initialize the planting density that is wanted (in m)
density = 3

# Set y acceleration low and high thresholds
accLowThreshold = 0
accHighThreshold = 0

# Set timing thesholds to check for values over
timeCheckLowHigh = 0
timeCheckAV = 0

# Set x angular velocity thresholds
angVThreshold = 0


# Loop until off
while True:

    # Has a plant just occured?
    plant = False

    # Are we 2m away from our last plant?
    if distance(GPS[0], GPS[1], GPS_last[0], GPS_last[1]) >= density:
        # Turn LED on to signify ready to plant
        GPIO.output(18,GPIO.HIGH)

    else:
        # Turn LED off to signify in bad range
        GPIO.output(18,GPIO.LOW)

    # Check if the acceleration is below the initial threshold
    if sensor.acceleration < accLowThreshold:

        # If the acceleration is below the threshold, loop until it isn't
        while sensor.acceleration < accLowThreshold:
            continue

        # Sets an end time to check for a planting instance until
        t_end_lh = time.time() + timeCheckLowHigh

        # Loop until the end time
        while time.time() < t_end_lh:

            # Did a plant just occur?
            if plant == True:
                break

            # Check if the acceleration is above a threshold
            if sensor.acceleration > accHighThreshold:
                
                # If the acceleration is above the threshold, loop until it isn't
                while sensor.acceleration > accHighThreshold:
                    continue

                # Set a new time end to loop until to check the angular velocity over
                t_end_av = time.time() + timeCheckAV

                # Loop until the end time
                while time.time() < t_end_av:

                    # If the angular velocity is above the threshold, a planting instance has occured!
                    if sensor.gyro > angVThreshold:   

                        # Calls on record location to record GPS location
                        GPS_last = record_location(datetime.datetime.now())

                        # Sets plant to true to return to top of loop
                        plant = True

                        break

