import cv2
import time

cap = cv2.VideoCapture("http://192.168.1.86:5000")

while(True):
    ret, frame = cap.read()
    cv2.imshow('frame',frame)
    cv2.waitKey(1)

cap.release()
cv2.destroyAllWindows()