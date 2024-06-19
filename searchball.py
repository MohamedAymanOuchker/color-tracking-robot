import cv2
from picamera2 import Picamera2
import RPi.GPIO as GPIO
from time import sleep
import numpy as np

# GPIO setup
def setup_gpio():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    for pin in [IN1, IN2, IN3, IN4]:
        GPIO.setup(pin, GPIO.OUT)

# Motor control functions
def motor_control(in1, in2, in3, in4):
    GPIO.output(IN1, in1)
    GPIO.output(IN2, in2)
    GPIO.output(IN3, in3)
    GPIO.output(IN4, in4)

def motor_forward():
    motor_control(GPIO.HIGH, GPIO.LOW, GPIO.HIGH, GPIO.LOW)

def motor_backward():
    motor_control(GPIO.LOW, GPIO.HIGH, GPIO.LOW, GPIO.HIGH)

def motor_right():
    motor_control(GPIO.HIGH, GPIO.LOW, GPIO.LOW, GPIO.HIGH)

def motor_left():
    motor_control(GPIO.LOW, GPIO.HIGH, GPIO.HIGH, GPIO.LOW)

def motor_stop():
    motor_control(GPIO.LOW, GPIO.LOW, GPIO.LOW, GPIO.LOW)

# Camera setup
def setup_camera():
    picam2 = Picamera2()
    camera_config = picam2.create_still_configuration(main={"size": (320, 240)})
    picam2.configure(camera_config)
    picam2.start()
    return picam2

# Image processing function
def process_image(picam2):
    img = picam2.capture_array()
    img = cv2.GaussianBlur(img, (3, 3), 0)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    hue_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    return img, hue_img

# Main loop
def main_loop(picam2):
    while True:
        img, hue_img = process_image(picam2)

        # Thresholding for red color
        lower_red = np.array([150, 100, 70])
        upper_red = np.array([215, 255, 255])
        threshold_img = cv2.inRange(hue_img, lower_red, upper_red)

        # Find contours
        contours, _ = cv2.findContours(threshold_img, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)

        action_taken = False
        for contour in contours:
            rect = cv2.boundingRect(contour)
            size = rect[2] * rect[3]
            mid = rect[0] + rect[2] / 2
            diag = (rect[2] * 2 + rect[3] * 2) * 0.5

            if size > 100:
                pt1 = (rect[0], rect[1])  # left top
                pt2 = (rect[0] + rect[2], rect[1] + rect[3])  # right bottom
                cv2.rectangle(img, pt1, pt2, (0, 0, 255), 3)

                if mid > 100:  # turn right
                    motor_right()
                    print('right')
                elif mid < 60:  # turn left
                    motor_left()
                    print('left')
                elif diag > 100:  # back
                    motor_backward()
                    print('backward')
                elif diag < 80:  # forward
                    motor_forward()
                    print('forward')
                else:
                    motor_stop()
                    print('stop')
                action_taken = True
                break

        if not action_taken:
            motor_stop()

        cv2.imshow("Colour Tracking", img)

        if cv2.waitKey(10) == 27:
            break

# Main script
if __name__ == "__main__":
    try:
        IN1, IN2, IN3, IN4 = 17, 27, 22, 23
        setup_gpio()
        picam2 = setup_camera()
        main_loop(picam2)
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        picam2.stop()
        cv2.destroyAllWindows()
        GPIO.cleanup()
