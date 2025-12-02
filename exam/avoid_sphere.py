from control import ROSInterface
import numpy as np
import cv2
import logging
import time
import math

REAL_RADIUS_M = 1.0

# Параметры камеры
IMG_WIDTH = 640
IMG_HEIGHT = 640
HFOV_RAD = 1.5
FOCAL_LENGTH_PX = IMG_WIDTH / HFOV_RAD


def find_sphere_and_get_radius(image):
    # Преобразование в HSV
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Определение диапазонов для красного цвета в HSV
    lower_red = np.array([0, 120, 70])
    upper_red = np.array([10, 255, 255])
    mask = cv2.inRange(hsv_image, lower_red, upper_red)

    # Поиск контуров
    contours, _ = cv2.findContours(
        mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if not contours:
        return None, None, None
    
    largest_contour = max(contours, key=cv2.contourArea)

    ((x, y), pixel_radius) = cv2.minEnclosingCircle(largest_contour)

    return (int(x), int(y)), int(pixel_radius), largest_contour


def calculate_distance(pixel_radius):
    if pixel_radius <= 0:
        return float('inf')

    angle = pixel_radius / FOCAL_LENGTH_PX

    if abs(angle) >= math.pi / 2:
        return 0.0  # Объект вплотную к камере

    # Формула для equidistant линзы
    distance = REAL_RADIUS_M / math.tan(angle)
    return distance

FINISH = 10.0 # y координата
AVOIDANCE_DISTANCE = 3.0

IMG_CENTER_X = IMG_WIDTH // 2

logger = logging.getLogger()
drone = ROSInterface(logger=logger)
time.sleep(2)
drone.takeoff(2.6)

cmd_x = 0.0
cmd_y = 0.0


while drone.get_odometry()[0][1] < FINISH:
    image = drone.get_image()
    if image is not None:
        detections = find_sphere_and_get_radius(image)
        cmd_x = 0.5
        cmd_y = 0.0
        if detections[0] is not None:
            center, radius, contour = detections
            distance = calculate_distance(radius)
            cv2.circle(image, center, radius, (0, 255, 0), 2)
            cv2.circle(image, center, 5, (255, 0, 0), -1)
            text = f"Distance: {distance:.2f} m"
            cv2.putText(image, text, (center[0] - 80, center[1] - radius - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            if distance < AVOIDANCE_DISTANCE:
                cmd_x = 0.25
                if center[0] < IMG_CENTER_X and abs(center[0] + radius - IMG_CENTER_X) < 100:
                    # Сфера слева, уклоняемся вправо
                    cmd_y = -0.5
                elif center[0] >= IMG_CENTER_X and abs(center[0] - radius - IMG_CENTER_X) < 100:
                    # Сфера справа, уклоняемся влево
                    cmd_y = 0.5
                    
    drone.publish_cmd_vel(linear_x=cmd_x, linear_y=cmd_y)
    cv2.imshow("Drone Camera", image)
    cv2.waitKey(1)
    time.sleep(0.1)

drone.publish_cmd_vel()
drone.shutdown()
