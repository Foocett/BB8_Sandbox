import cv2 as cv
import numpy as np

cam = cv.VideoCapture(0)

while True:
    ret, frame = cam.read()
    if not ret:
        print("failed to grab frame")
        break

    gs = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
    cv.imshow("test", gs)

    k = cv.waitKey(1)
    if k == ord('q'):
        # ESC pressed
        print("Escape hit, closing...")
        break
