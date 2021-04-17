import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(18,GPIO.OUT)


t_end = time.time() + 60

while time.time() < t_end:
    GPIO.output(18,GPIO.HIGH)
    time.sleep(0.5)
    GPIO.output(18,GPIO.LOW)
    time.sleep(0.5)