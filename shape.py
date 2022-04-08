import cv2
import numpy as np
import math

def shape_recognition(img, color):
    #color - СПИСОК!!!
    '''color = (
        ( 56, 192,  90),
        ( 74, 255, 255),
    )'''
    def find_contours(img, color):
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = []
        i = 0
        for hue in color:
            mask.append(cv2.inRange(img_hsv, hue[0], hue[1]))
            cv2.imwrite(f'm{i}.jpg', mask[i])
            i+=1
        print(mask[0].shape)
        img_mask = np.zeros(mask[0].shape, np.float64)
        for m in mask:
            img_mask += m
        img_mask = np.clip(img_mask , 0, 255)
        img_mask = img_mask.astype(np.uint8)
        cv2.imwrite('img.jpg', img_mask)
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

image = cv2.imread('img/triangle.png')
color = [
        ((0, 50, 50), 
        (15, 255, 255)),
        ((170, 50, 50),
        (180, 255, 255)) 
    ]

found, x, y = shape_recognition(image, color)

print(found, x, y)