import cv2
import numpy as np
import math
import time
import pymurapi as mur

auv = mur.mur_init()

#класс для хранения атрибутов аппарата
class Context(object):
    _yaw = 0.0 # текущий курс
    _depth = 0.0 # текущая глубина
    _speed_forward_left = 0.0 # текущая скорость вперёд-назад левого движителя
    _speed_forward_right = 0.0 # текущая скорость вперёд-назад правого движителя
    _speed_up = 0.0 # текущая скорость вверх-вниз
    _prev_update_time = 0 # контроль частоты вызова "воздействий"
    _missions = [] # последовательность действий

    def set_yaw(self, value):
        self._yaw = value
    def set_depth(self, value):
        self._depth = value
    def set_speed_forward(self, value_l, value_r):
        self._speed_forward_left = value_l
        self._speed_forward_right = value_r
    def set_speed_forward(self, value):
        self._speed_forward_left = value
        self._speed_forward_right = value
    def set_speed_forward_left(self, value):
        self._speed_forward_left = value
    def set_speed_forward_right(self, value):
        self._speed_forward_right = value
    def set_speed_up(self, value):
        self._speed_up = value

    def get_yaw(self):
        return self._yaw
    def get_depth(self):
        return self._depth
    def get_speed_forward(self):
        return (self._speed_forward_left, self._speed_forward_right)
    def get_speed_forward_left(self):
        return self._speed_forward_left
    def get_speed_forward_right(self):
        return self._speed_forward_right
    def get_speed_up(self):
        return self._speed_up
    def stop_motors(self):
        auv.set_motor_power(0, 0)
        auv.set_motor_power(1, 0)
        auv.set_motor_power(2, 0)
        auv.set_motor_power(3, 0)
        self.set_speed_forward = 0
        self.set_speed_up = 0
    def add_mission(self, func):
        self._missions.append(func)
    def pop_mission(self):
        return self._missions.pop(0)
    
    def get_missions_length(self):
        return len(self._missions)

    def update(self):
        timestamp = int(round(time.time() * 1000))
        if abs(timestamp - self._prev_update_time) > 16:
            #print(self._depth, self._speed_up)
            keep_depth(self._depth, self._speed_up)
            keep_yaw(self._yaw, self._speed_forward_left, self._speed_forward_right)
            self._prev_update_time = timestamp
        else:
            time.sleep(0.05)

cnt = Context() # хранение атрибутов аппарата

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
    _timestamp = 0 # предыдущее время
    
    def __init__(self):
        pass
    def set_p_gain(self, value):
        self._kp = value
    def set_d_gain(self, value):
        self._kd = value
    #расчёт регулирующего воздействия
    def process(self, error):
        timestamp = int(round(time.time() * 1000))
        try:
            output = self._kp * error + self._kd / (timestamp - self._timestamp) * (error - self._prev_error)
            self._timestamp = timestamp 
            self._prev_error = error 
            return output
        except:
            time.sleep(0.1)
            timestamp = int(round(time.time() * 1000))
            output = self._kp * error + self._kd / (timestamp - self._timestamp) * (error - self._prev_error)
            self._timestamp = timestamp 
            self._prev_error = error 
            return output
 
# поддержание глубины
def keep_depth(depth_to_set, speed_up):
    try:
        error = auv.get_depth() - depth_to_set 
        output = keep_depth.regulator.process(error) 
        output = clamp(output, -100, 100) 
        auv.set_motor_power(2, output + speed_up) 
        auv.set_motor_power(3, output + speed_up)
    except AttributeError:
        keep_depth.regulator = PD() 
        keep_depth.regulator.set_p_gain(70) 
        keep_depth.regulator.set_d_gain(5)

# Функция удержания курса
def keep_yaw(yaw_to_set, speed_left, speed_right):
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
        output0 = clamp(-output + speed_left, -100, 100)
        output1 = clamp(output + speed_right, -100, 100)
        auv.set_motor_power(0, output0) 
        auv.set_motor_power(1, output1)
    except AttributeError:
        keep_yaw.regulator = PD() 
        keep_yaw.regulator. set_p_gain(0.8) 
        keep_yaw.regulator.set_d_gain(0.5)

def shape_recognition(img, color): # распознавание фигур
    #color - СПИСОК!!!
    def find_contours(img, color):
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = []
        #i = 0
        for hue in color:
            mask.append(cv2.inRange(img_hsv, hue[0], hue[1]))
            #cv2.imwrite(f'm{i}.jpg', mask[i])
            #i+=1
        #print(mask[0].shape)
        img_mask = np.zeros(mask[0].shape, np.float64)
        for m in mask:
            img_mask += m
        img_mask = np.clip(img_mask , 0, 255)
        img_mask = img_mask.astype(np.uint8)
        #cv2.imwrite('img.jpg', img_mask)
        contours, _ = cv2.findContours(img_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        return contours

    contours = find_contours(img, color)
    #print('len', len(contours))
    figures = []
    if contours:
        for cnt in contours:
            area = abs(cv2.contourArea(cnt))
            if area > 500:
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
                # разницу между площадью контура и площадью каждой из фигур.
                diffs = {
                    name: abs(area - shapes_areas[name]) for name in shapes_areas
                }
                # Получаем имя фигуры с наименьшей разницой площади.
                shape_name = min(diffs, key=diffs.get) 
                
                figures.append(shape_name, cnt)
    # Список фигур - название, контур
    return figures

def shape_center(cnt):
    moments = cv2.moments(cnt)
    try:
        x = int(moments['m10'] / moments['m00'])
        y = int(moments['m01'] / moments['m00'])
        return (x, y)
    except ZeroDivisionError:
        return (0, 0)

def find_shape(image, shape, color):
    # поиск нужной фигуры, возвращаем список из интересуюющих фигур
    figures = shape_recognition(image, color)
    necessary_figure = []
    for f in figures:
        if f[0] == shape:
            necessary_figure.append(f)
    return necessary_figure

def find_circle(image, color):
    # ищем самый большой круг нужного цвета
    image_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    shape = find_shape(image_hsv, color, 'circle')
    shape_max = []
    if shape:
        max_area = cv2.contourArea(shape[0][1])
        shape_max.append(shape[0])
        for s in shape:
            area = cv2.contourArea(s[1])
            if area > max_area:
                max_area = area
                shape_max.clear()
                shape_max.append(s)
    return shape_max

def stab_on_circle(image, color):
    found = find_circle(image, color)
    if found:
        x = found[0][1]
        y = found[1][1]
        h, w, c = image.shape
        x_center = x - (w / 2)
        y_center = y - (h / 2)
        try:
            length = math.sqrt(x_center**2 + y_center**2)
            #print(length)
            if length < 3: #условие выхода из поиска
                cnt.set_speed_up(0)
                cnt.set_speed_forward(0, 0)
                cnt.set_depth(auv.get_depth())
                return True
            output_depth = stab_on_circle.regulator_depth.process(y_center)
            output_side = stab_on_circle.regulator_side.process(x_center) 
            output_depth = clamp(output_depth, -50, 50)
            output_side = clamp(output_side, -50, 50) 
            #print(output_side)
            cnt.set_speed_up(-output_depth)
            cnt.set_speed_forward_left(output_side)
            cnt.set_speed_forward_right(-output_side)
            #auv.set_motor_power(2, -output_depth) 
            #auv.set_motor_power(3, -output_depth)

            #auv.set_motor_power(0, output_side) 
            #auv.set_motor_power(1, -output_side)
            
        except AttributeError:
            stab_on_circle.regulator_depth = PD() 
            stab_on_circle.regulator_depth.set_p_gain(0.1) 
            stab_on_circle.regulator_depth.set_d_gain(0.1)

            stab_on_circle.regulator_side = PD() 
            stab_on_circle.regulator_side.set_p_gain(0.1) 
            stab_on_circle.regulator_side.set_d_gain(0.1)

    else: #если перед роботом нет красного круга, опускаемся вниз
        #auv.set_motor_power(2, -10) 
        #auv.set_motor_power(3, -10)
        cnt.set_speed_up(-10)
        cnt.set_depth(auv.get_depth())
        #keep_yaw(yaw, 0, 0)
    return False

def move_to_red_circle():
    image = auv.get_image_front()
    found = find_on_red_circle(image, cnt.get_yaw())
    print(found)
    return found

# MAIN БЛОК
color_red = [
        ((0, 50, 50), 
        (15, 250, 250)),
        ((170, 50, 50),
        (180, 250, 250)) 
    ]
time.sleep(0.05)
cnt.set_yaw(auv.get_yaw())
cnt.set_depth(auv.get_depth())
cnt.set_speed_up(-30) # скорость вниз
cnt.add_mission(move_to_red_circle) # добавляем "задачу"
cnt.add_mission(stab_on_circle)
while True:
    mission = cnt.pop_mission()
    while not mission():
        print('update')
        cnt.update()
    if cnt.get_missions_length() == 0:
        break
print('end')
'''
yaw_required = auv.get_yaw()
depth_required = 0
print(yaw_required)
while True:
    image = auv.get_image_front()
    if stab_on_red_circle(image, yaw_required):
        depth_required = auv.get_depth()
        yaw_required = auv.get_yaw()
        print('Stop stab')
        break
    time.sleep(0.05)

print(depth_required, yaw_required)

timestamp_s = int(round(time.time() ))
while True:
    keep_depth(depth_required)
    keep_yaw(yaw_required)
    time.sleep(0.05)
    timestamp_e = int(round(time.time()))
    if abs(timestamp_e - timestamp_s) > 5:
        break

yaw_required += 90 #так не правильно, но это ради теста
while True:
    keep_yaw(yaw_required)
    '''