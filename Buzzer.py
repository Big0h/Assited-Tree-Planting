import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(13,GPIO.OUT)


GPIO.output(13,GPIO.HIGH)
time.sleep(1)
GPIO.output(13,GPIO.LOW)