import cv2
import numpy as np
import math

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
                return [shape_name, (x, y)]
            except ZeroDivisionError:
                return [shape_name,]
