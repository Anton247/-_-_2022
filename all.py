import cv2
import numpy as np
import math
import time
import pymurapi as mur

auv = mur.mur_init()

def shape_recognition(img, color):
    '''color = (
        ( 56, 192,  90),
        ( 74, 255, 255),
    )'''
    def find_contours(img, color):
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        img_mask = cv2.inRange(img_hsv, color[0], color[1])
        contours, _ = cv2.findContours(img_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        return contours

    contours = find_contours(img, color)
    if contours:
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < 500:
                continue
            # Описанная окружность.
            (circle_x, circle_y), circle_radius = cv2.minEnclosingCircle(cnt)
            circle_area = circle_radius ** 2 * math.pi
            # Описанный прямоугольник (с вращением)
            rectangle = cv2.minAreaRect(cnt)
            # Получим контур описанного прямоугольника
            box = cv2.boxPoints(rectangle)
            box = np.int0(box)
            # Вычислим площадь и соотношение сторон прямоугольника.
            rectangle_area = cv2.contourArea(box)
            rect_w, rect_h = rectangle[1][0], rectangle[1][1]
            aspect_ratio = max(rect_w, rect_h) / min(rect_w, rect_h)
            # Описанный треугольник
            try:
                triangle = cv2.minEnclosingTriangle(cnt)[1]
                triangle = np.int0(triangle)
                triangle_area = cv2.contourArea(triangle)
            except:
                triangle_area = 0
            # Заполним словарь, который будет содержать площади каждой из описанных фигур
            shapes_areas = {
                'circle': circle_area,
                'rectangle' if aspect_ratio > 1.25 else 'square': rectangle_area,
                'triangle': triangle_area,
            }

            # Теперь заполним аналогичный словарь, который будет содержать
            # разницу между площадью контора и площадью каждой из фигур.
            diffs = {
                name: abs(area - shapes_areas[name]) for name in shapes_areas
            }
            # Получаем имя фигуры с наименьшей разницой площади.
            shape_name = min(diffs, key=diffs.get) 
            # вычислим центр
            moments = cv2.moments(cnt)
            try:
                x = int(moments['m10'] / moments['m00'])
                y = int(moments['m01'] / moments['m00'])
                return shape_name, x, y
            except ZeroDivisionError:
                return shape_name, -1, -1
        else:
            return False, -1, -1

def clamp(value, min_value, max_value):
    if value > max_value: 
        return max_value 
    if value < min_value: 
        return min_value
    return value

class PD(object):
    _kp = 0.0 #коэффициент пропорциональности 
    _kd = 0.0 #дифференциальный пропорциональности
    _prev_error = 0.0 # предыдущая ошибка
    _timestamp = 0 # время
    
    def __init__(self):
        pass
    def set_p_gain(self, value):
        self._kp = value
    def set_d_gain(self, value):
        self._kd = value
    #расчёт регулирующего воздействия
    def process(self, error):
        timestamp = int(round(time.time() * 1000))
        output = self._kp * error + self._kd / (timestamp - self._timestamp) * (error - self._prev_error)
        self._timestamp = timestamp 
        self._prev_error = error 
        return output
 
def keep_depth(depth_to_set):
    try:
        error = auv.get_depth() - depth_to_set 
        output = keep_depth.regulator.process(error) 
        output = clamp(output, -100, 100) 
        auv.set_motor_power(2, output) 
        auv.set_motor_power(3, output)
    except AttributeError:
        keep_depth.regulator = PD() 
        keep_depth.regulator.set_p_gain(70) 
        keep_depth.regulator. set_d_gain(5)

# Функция удержания курса
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
                   
