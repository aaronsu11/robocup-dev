import numpy as np
import cv2

cap = cv2.VideoCapture(0)
# cap = cv2.VideoCapture('/home/aaronsu11/Videos/Tesla.mp4')

while(True):
    ret, frame = cap.read()

    cv2.imshow("Frame",frame)
    if cv2.waitKey(50) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()