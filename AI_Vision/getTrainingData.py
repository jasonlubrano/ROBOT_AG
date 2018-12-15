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
count6 = 0
count7 = 0

while(1):
    # Take each frame
    _, frame = cap.read()
    # Convert BGR to HSV
    frame = cv2.resize(frame, (500, 500), interpolation = cv2.INTER_AREA)
    height, width = frame.shape[:2]
    frame1 = frame[int(80):int(height * .5) , int(85):int(width * .5)]
    frame2 = frame[int(80):int(height * .5) , int(width * .5):int(width - 100)]
    frame3 = frame[int((height * .5) + 40):int(height) , int(70):int(width * .5)]
    frame4 = frame[int((height * .5) + 20):int(height) , int(width * .5):int(width - 80)]

    crop = frame1
    if(count1 > 230 or count2 > 230 or count3 > 230 or count4 > 230 or count5 > 230 or count6 > 230 or count7 > 230):
        crop = frame2
    elif (count1 > 160 or count2 > 160 or count3 > 160 or count4 > 160 or count5 > 160 or count6 > 160 or count7 > 160):
        crop = frame3
    elif (count1 > 70 or count2 > 70 or count3 > 70 or count4 > 70 or count5 > 70 or count6 > 70 or count7 > 70):
        crop = frame4
    # Bitwise-AND mask and original image
    cv2.imshow('frame1',crop)
    print(count1)
    print(count2)
    print(count3)
    print(count4)
    print(count5)
    print(count6)
    print(count7)
    k = cv2.waitKey(5) & 0xFF
    if k == 49 and count1 < 300:
        count1 += 1
        cv2.imwrite('TrainingData/Lisianthus/pink' + str(count1)+ '.jpg',crop)
    elif k == 50 and count2 < 300:
        count2 += 1
        cv2.imwrite('TrainingData/Rose/purple' + str(count2)+ '.jpg',crop)
    elif k == 51 and count3 < 300:
        count3 += 1
        cv2.imwrite('TrainingData/Carnation/red' + str(count3)+ '.jpg',crop)
    elif k == 52 and count4 < 300:
        count4 += 1
        cv2.imwrite('TrainingData/Gardenia/white' + str(count4)+ '.jpg',crop)
    elif k == 53 and count5 < 300:
        count5 += 1
        cv2.imwrite('TrainingData/Weed1/weeder' + str(count5)+ '.jpg',crop)
    elif k == 54 and count6 < 300:
        count6 += 1
        cv2.imwrite('TrainingData/Weed2/weed' + str(count6)+ '.jpg',crop)
    elif k == 55 and count7 < 300:
        count7 += 1
        cv2.imwrite('TrainingData/Empty/empty' + str(count7)+ '.jpg',crop)
    elif k == 27:
        break
cv2.destroyAllWindows()
