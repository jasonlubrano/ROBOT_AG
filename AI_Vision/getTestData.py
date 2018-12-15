import cv2
import numpy as np
import time
import csv
import time

cap = cv2.VideoCapture(1)

count1 = 0
count2 = 0
count3 = 0
count4 = 0
count5 = 0

while(1):
    # Take each frame
    _, frame = cap.read()
    # Convert BGR to HSV
    frame = cv2.resize(frame, (500, 500), interpolation = cv2.INTER_AREA)
    # Bitwise-AND mask and original image
    cv2.imshow('frame',frame)
    print(count1)
    print(count2)
    print(count3)
    print(count4)
    k = cv2.waitKey(5) & 0xFF
    if k == 49 and count1 < 10:
        count1 += 1
        cv2.imwrite('TestData/Pink/pink' + str(count1)+ '.jpg',frame)
    elif k == 50 and count2 < 10:
        count2 += 1
        cv2.imwrite('TestData/Purple/purple' + str(count2)+ '.jpg',frame)
    elif k == 51 and count3 < 10:
        count3 += 1
        cv2.imwrite('TestData/Red/red' + str(count3)+ '.jpg',frame)
    elif k == 52 and count4 < 10:
        count4 += 1
        cv2.imwrite('TestData/White/white' + str(count4)+ '.jpg',frame)
    elif k == 53 and count5 < 10:
        count5 += 1
        cv2.imwrite('TestData/Weed/weed' + str(count5)+ '.jpg',frame)
    elif k == 27:
        break
cv2.destroyAllWindows()
