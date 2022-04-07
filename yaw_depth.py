import cv2 as cv
import pymurapi as mur 
import math 
import time

auv = mur.mur_init()

def clamp(value, min_value, max_value):
    if value > max_value: 
        return max_value 
    if value < min_value: 
        return min_value
    return value

class PD(object):
    _kp = 0.0 
    _kd = 0.0 
    _prev_error = 0.0 
    _timestamp = 1
    
    def __init__(self):
        pass
    def set_p_gain(self, value):
        self._kp = value
    def set_d_gain(self, value):
        self._kd = value
    def process(self, error):
        timestamp = int(round(time.time() * 1000))
        output = self._kp * error + self._kd / (timestamp - self._timestamp) * (error - self._prev_error)
        self._timestamp = timestamp 
        self._prev_error = error 
        return output

def keep_yaw(yaw_to_set):
    def clamp_to_180(angle):
        if angle > 180.0:
            return angle - 360.0
        if angle < -180.0:
            return angle + 360.0
        return angle
    try:
        error = auv.get_yaw() - yaw_to_set 
        error = clamp_to_180(error) 
        output = keep_yaw.regulator.process(error) 
        output = clamp(output, -100, 100) 
        auv. set_motor_power(0, -output) 
        auv. set_motor_power(1, output)
    except AttributeError:
        keep_yaw.regulator = PD() 
        keep_yaw.regulator. set_p_gain(0.8) 
        keep_yaw.regulator.set_d_gain(0.5)

def keep_depth(depth_to_set):
    try:
        error = auv.get_depth() - depth_to_set 
        output = keep_depth.regulator.process(error) 
        output = clamp(output, -100, 100) 
        auv.set_motor_power(2, output) 
        auv.set_motor_power(3, output)
    except AttributeError:
        keep_depth.regulator = PD() 
        keep_depth.regulator.set_p_gain(80) 
        keep_depth.regulator. set_d_gain(50)

def find_red_circle(img):
    image_hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV) 
    hsv_min = (0, 50, 50) 
    hsv_max = (15, 255, 255) 
    image_bin = cv.inRange(image_hsv, hsv_min, hsv_max)
    hsv_min1 = (170, 50, 50) 
    hsv_max1 = (180, 255, 255)
    image_bin1 = cv.inRange(image_hsv, hsv_min1, hsv_max1)
    image_bin_sum = image_bin + image_bin1
    cnt, _ = cv.findContours(image_bin_sum, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_NONE)
    print(len(cnt))
    for c in cnt:
        area = cv.contourArea(c)
        if abs(area) < 300:
            continue
        ((_, _), (w, h), _) = cv.minAreaRect(c)
        (_, _), radius = cv.minEnclosingCircle(c)
        
        rectangle_area = w * h 
        circle_area = radius ** 2 * math.pi 
        aspect_ratio = w / h
        if 0.9 <= aspect_ratio <= 1.1:
            if rectangle_area > circle_area:
                moments = cv.moments(c) 
                try:
                    x = int(moments["m10"] / moments["m00"]) 
                    y = int(moments["m01"] / moments["m00"]) 
                    return True, x, y
                except ZeroDivisionError:
                    return False, 0, 0
            else:
                continue
        else:
            continue
    return False, 0, 0

def stab_on_red_circle(image):
    found, x, y = find_red_circle(image)
 
    if found:
        x_center = x - (320 / 2)
        y_center = y - (240 / 2)
        
        try:
            output_forward = stab_on_red_circle.regulator_forward.process(y_center)
            print("Ok")
            output_side = stab_on_red_circle.regulator_side.process(x_center)  
            
            output_forward = clamp(output_forward, -50, 50) 
            output_side = clamp(output_side, -50, 50)
    
            auv.set_motor_power(0, output_forward) 
            auv.set_motor_power(1, output_forward)
            auv.set_motor_power(4, output_side) 
                
        except AttributeError:
            stab_on_red_circle.regulator_forward = PD() 
            stab_on_red_circle.regulator_forward.set_p_gain(0.8) 
            stab_on_red_circle.regulator_forward. set_d_gain(0.5)
    
            stab_on_red_circle.regulator_side = PD() 
            stab_on_red_circle.regulator_side.set_p_gain(0.8) 
            stab_on_red_circle.regulator_side. set_d_gain(0.5)
            print("Om")
    
while True:
    image = auv.get_image_front()
    stab_on_red_circle(image)
    time.sleep(0.5)
    