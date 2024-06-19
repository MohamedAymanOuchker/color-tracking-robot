import cv2
import RPi.GPIO as GPIO
from time import sleep


# Set up GPIO mode and warnings
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

# Motor pin setup
GPIO.setup(11, GPIO.OUT)
GPIO.setup(13, GPIO.OUT)
GPIO.setup(15, GPIO.OUT)

# Servo pin setup
GPIO.setup(16, GPIO.OUT)
GPIO.setup(18, GPIO.OUT)

# Define motor control functions
def motor_forward():
    GPIO.output(11, GPIO.LOW)
    GPIO.output(13, GPIO.HIGH)
    GPIO.output(15, GPIO.HIGH)

def motor_backward():
    GPIO.output(11, GPIO.HIGH)
    GPIO.output(13, GPIO.HIGH)
    GPIO.output(15, GPIO.HIGH)

def motor_right():
    GPIO.output(11, GPIO.LOW)
    GPIO.output(13, GPIO.LOW)
    GPIO.output(15, GPIO.HIGH)

def motor_left():
    GPIO.output(11, GPIO.LOW)
    GPIO.output(13, GPIO.HIGH)
    GPIO.output(15, GPIO.LOW)

def motor_stop():
    GPIO.output(11, GPIO.LOW)
    GPIO.output(13, GPIO.LOW)
    GPIO.output(15, GPIO.LOW)

def stand():
    GPIO.output(16, GPIO.LOW)
    GPIO.output(18, GPIO.HIGH)

# Main loop
try:
    while True:
        motor_forward()
        stand()
        sleep(1.01)
        # Uncomment to stop motor after moving forward
        # motor_stop()
        # sleep(0.1)
except KeyboardInterrupt:
    # Clean up GPIO settings on exit
    GPIO.cleanup()
