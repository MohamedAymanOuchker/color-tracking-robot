import cv2
import numpy as np
from time import sleep

# Initialize the camera
capture = cv2.VideoCapture(0)
capture.set(cv2.CAP_PROP_FRAME_WIDTH, 160)
capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 120)

while True:
    ret, img = capture.read()
    
    if not ret:
        break
    
    img = cv2.GaussianBlur(img, (3, 3), 0)
    hue_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Thresholding for red color
    lower_red = np.array([150, 100, 70])
    upper_red = np.array([215, 255, 255])
    threshold_img = cv2.inRange(hue_img, lower_red, upper_red)

    # Thresholding for blue color
    lower_blue = np.array([110, 50, 50])
    upper_blue = np.array([130, 255, 255])
    threshold_img3 = cv2.inRange(hue_img, lower_blue, upper_blue)

    # Find contours for red
    contours, _ = cv2.findContours(threshold_img, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        rect = cv2.boundingRect(contour)
        size = rect[2] * rect[3]
        if size > 500:  # Set trigger
            pt1 = (rect[0], rect[1])  # left top
            pt2 = (rect[0] + rect[2], rect[1] + rect[3])  # right bottom
            cv2.rectangle(img, pt1, pt2, (0, 0, 255), 3)  # draw red rectangle

    # Find contours for blue
    contours3, _ = cv2.findContours(threshold_img3, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    for contour3 in contours3:
        rect3 = cv2.boundingRect(contour3)
        size3 = rect3[2] * rect3[3]
        if size3 > 250:
            pt13 = (rect3[0], rect3[1])  # left top
            pt23 = (rect3[0] + rect3[2], rect3[1] + rect3[3])  # right bottom
            cv2.rectangle(img, pt13, pt23, (255, 0, 0), 3)  # draw blue rectangle

    # Show the image
    cv2.imshow("Colour Tracking", img)

    if cv2.waitKey(10) == 27:  # Escape key to exit
        break

# Release the capture and destroy windows
capture.release()
cv2.destroyAllWindows()
