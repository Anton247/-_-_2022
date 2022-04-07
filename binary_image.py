import cv2 as cv
from cv2 import moments
import pymurapi as mur

auv = mur.mur_init()

def binary_image():
    image = auv.get_image_bottom()
    
    imageHSV = cv.cvtColor(image, cv.COLOR_BGR2HSV)
    
    low_hsv_yellow = (15, 50, 50) 
    max_hsv_yellow = (25, 255, 255)
    yellow_hsv_mask = cv.inRange(imageHSV, low_hsv_yellow, max_hsv_yellow)
    
    cnt, _ = cv.findContours(yellow_hsv_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)

    if cnt:
        for c in cnt:
            area = cv.contourArea(c)
            if abs(area) > 300:
                hull = cv.convexHull(c)
                approx = cv.approxPolyDP(hull, cv.arcLength(c, True) * 0.02, True)
                if len(approx) == 4:
                    cv.drawContours(image, [c], 0, (0, 0, 255), 3)

    '''cv.drawContours(image, cnt, -1, (0, 0, 255), 2)
    print(cnt)
    cv.imshow("Cnt", image)
    
    moments = cv.moments(cnt[0])

    x = moments['m10'] / moments['m00']
    y = moments['m01'] / moments['m00']
    
    cv.circle(image, (int(x), int(y)), 4, (255, 0, 0))
'''
    cv.imshow("Cnt", image)
    cv.waitKey(5)
    
while True:
    binary_image()