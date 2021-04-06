# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

import time
import board
import busio
import adafruit_bno055

# Use these lines for I2C
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055_I2C(i2c)
#sensor.mode=adafruit_bno055.NDOF_MODE
#sensor.mode=adafruit_bno055.IMUPLUS_MODE
Hz = 100  #Sample speed
# User these lines for UART
# uart = busio.UART(board.TX, board.RX)
# sensor = adafruit_bno055.BNO055_UART(uart)
start=time.time()
last_val = 0xFFFF
temp=sensor.temperature
acc=sensor.acceleration
mag=sensor.magnetic
gyro=sensor.gyro
euler=sensor.euler
quaternion=sensor.quaternion
lin_acc= sensor.linear_acceleration
gravity= sensor.gravity
calib=sensor.calibration_status
counts = [0,0,0,0]
def temperature():
    global last_val  # pylint: disable=global-statement
    result = sensor.temperature
    if abs(result - last_val) == 128:
        result = sensor.temperature
        if abs(result - last_val) == 128:
            return 0b00111111 & result
    last_val = result
    return result


while True:
    temp=sensor.acceleration
    if temp[0]!= None:
        acc=temp
        counts[0]=counts[0]+1
    temp=sensor.magnetic
    if temp[0]!= None:
        mag=temp
        counts[1]=counts[1]+1
    temp=sensor.gyro
    if temp[0]!= None:
        gyro=temp
        counts[2]=counts[2]+1
    temp=sensor.linear_acceleration
    if temp[0]!= None:
        lin_acc=temp
        counts[3]=counts[3]+1
    #print("Temperature: {} degrees C".format(sensor.temperature))
    
    #print("Temperature: {} degrees C".format(temperature()))  # Uncomment if using a Raspberry Pi
    print("Accelerometer (m/s^2): {}".format(acc))
    #print("Magnetometer (microteslas): {}".format(mag))
    print("Gyroscope (rad/sec): {}".format(gyro))
    #print("Euler angle: {}".format(sensor.euler))
    #print("Quaternion: {}".format(sensor.quaternion))
    #print("Linear acceleration (m/s^2): {}".format(lin_acc))
    #print("Gravity (m/s^2): {}".format(sensor.gravity))
    #print("Calibration:" , sensor.calibration_status)
    #print(counts)
    #print(time.time()-start)
    time.sleep(5)
    print()
