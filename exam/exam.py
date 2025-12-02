import numpy as np
from control import ROSInterface
import cv2
from ultralytics import YOLO
import logging
import time

class GateCeneteringControl:
    def __init__(self, drone, model_path):
        self.drone = drone
        self.model = YOLO(model_path)
        
        # Параметры камеры
        self.image_width = 640
        self.image_height = 640
        self.image_center_x = self.image_width // 2
        self.image_center_y = self.image_height // 2
        
        
        self.target_area = (self.image_height * self.image_width) * 0.2  # Целевая площадь приближения
        self.dead_zone = self.target_area * 0.2  # Допустимая погрешность
        self.correct_th = 15 / self.image_center_x
        
        # --- Коэффициенты P-регулятора
        self.kp_forward = 0.2
        self.kp_y = 2.0
        self.kp_z = 0.5
        
        # Ограничения скоростей
        self.max_x_vel = 0.4
        self.max_y_vel = 0.5
        self.max_z_vel = 1.0
        
        
    def get_gate_box(self, frame):
        result = self.model(frame, verbose=False)
        clossest_bbox = None
        max_area = 0
        
        if result and result[0].boxes:
            for box in result[0].boxes:
                if box.xyxy.shape[1] == 4:
                    bbox = box.xyxy[0].cpu().numpy()
                    wight = bbox[2] - bbox[0]
                    height = bbox[3] - bbox[1]
                    area = wight * height
                    if area > max_area:
                        max_area = area
                        clossest_bbox = bbox
            
        return clossest_bbox, max_area 
        
    
    def run(self):
        while True:
            time.sleep(1/30)
            frame = self.drone.get_image()
            self.drone.publish_cmd_vel()
            if frame is None:
                print('No image')
                continue
            
            gate_bbox, gate_area = self.get_gate_box(frame)
            
            if abs(gate_area - self.target_area) < self.dead_zone:
                print("Centering finish")
                self.drone.publish_cmd_vel()
                break 
            
            if gate_bbox is not None:
                
                x1, y1, x2, y2 = gate_bbox
                box_center_x = (x1 + x2) / 2
                box_center_y = (y1 + y2) / 2
                
                # Считаем смещение между центром рамки и центром камеры
                error_x = self.image_center_x - box_center_x
                error_y = self.image_center_y - box_center_y
                norm_error_x = error_x / self.image_center_x
                norm_error_y = error_y / self.image_center_y
                
                # Считаем разницу между текущей площадью рамки и целевой
                error_area = self.target_area - gate_area
                y_speed = norm_error_x * self.kp_y
                z_speed = 0.0
                if abs(error_y) > self.correct_th:
                    z_speed = norm_error_y * self.kp_z
                
                forward_speed = 0.0
                if abs(error_area) > self.correct_th:
                    forward_speed = error_area * self.kp_forward
                    
                # Применим ограничения скоростей
                y_speed = np.clip(y_speed, -self.max_y_vel, self.max_y_vel)
                z_speed = np.clip(z_speed, -self.max_z_vel, self.max_z_vel)
                forward_speed = np.clip(forward_speed, -self.max_x_vel, self.max_x_vel)
                
                print(f"Speed x: {forward_speed}, y: {y_speed}, z: {z_speed}")
                
                # Отправляем вычисленные скорости дрону
                self.drone.publish_cmd_vel(forward_speed, y_speed, z_speed)
                
                # Визуализация найденной рамки
                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                cv2.circle(frame, (int(box_center_x), int(box_center_y)), 5, (255, 0, 0), -1)
                
            else:
                # self.drone.publish_cmd_vel(linear_z=-0.2)
                # time.sleep(2/30)
                print('Lost gate')
                self.drone.publish_cmd_vel()
                # print('Rotate drone to find gate')
                # self.drone.publish_cmd_vel(angular_z=-0.1)                
                
            cv2.imshow("Gate Centering", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.drone.publish_cmd_vel()
                break
        
        cv2.destroyAllWindows()
        
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

    if abs(angle) >= np.pi / 2:
        return 0.0  # Объект вплотную к камере

    # Формула для equidistant линзы
    distance = REAL_RADIUS_M / np.tan(angle)
    return distance



GPS_COORDINATES = np.array([
    [-8.53114075e-07, -3.51647797e-06, 2],  # Начало ворота
    [3.24710903e-06, 2.46968846e-07, 3]  # Начало сферы
])

FINISH_Y = 37.0

R = 6378137.0

def convert_gps(v):
    x = R * np.cos(v[0]) * np.sin(v[1])
    y = R * np.sin(v[0])
    return np.array([x, y, v[2]])

# Шаг 1: Подключение и взлет

# Подключаюсь к дрону через RosInterface, сохраняю стартовую позицию по z, поднимаю дрон в воздух на 2 м с помощью takeoff.

logger = logging.getLogger("EXAM")
drone = ROSInterface(logger)
time.sleep(2)
ground_z = drone.get_pose()[0][2]

drone.takeoff(2)

# Шаг 2: Перемещаемся к первой GPS-координте

# Беру первую GPS-координату, преобразовываю в обычные координаты, говорю дрону лететь в полученные координаты

drone.fly_to_pose(convert_gps(GPS_COORDINATES[0]))

# Шаг 3: Центрируемся по воротам и пролетаем их. Делаем так 3 раза
# цикл от 0 до 3:
#   центруюсь по воротам с GateCeneteringControl
#   после пролетаю ворота - преобразовываю текущие координаты дрона в body frame, устанавливаю z = 2, прибавляю к x 3.0, преобразовываю обратно в
#   world frame и говорю дрону лететь к этим координатам
cnt_gates = 3
gate_controller= GateCeneteringControl(drone, './models_pt/detect_gate.pt')

def pass_gate():
    pose = drone.get_pose()[0]
    pose_bf = drone.to_local_frame(pose)
    pose_bf[0] += 3.0
    pose_bf[2] = 2.0
    pose = drone.to_world_frame(pose_bf)
    drone.fly_to_pose(pose)

for i in range(cnt_gates):
    gate_controller.run()
    pass_gate()


# Шаг 4: Перемещаемся ко второй GPS-координте

drone.fly_to_pose(convert_gps(GPS_COORDINATES[1]))

# Шаг 5: Облетаем сферы, чтобы долететь до финиша. 
# Будем делать маневр уклонения, если сфера ближе 3-х метров.


# Задаю параметры камеры и сферы. Лечу к заданной финишной координате.
# Пока текущая координата меньше чем финишная:
#   Беру изображение дрона, нахожу на нем ближайшую сферу и считаю до неё расстояние, если расстояние меньше чем 3.0 метра, то делаю уклонение
#   Для уклонения сравниваю центр сферы с центром изображения -> если центр сферы левее, то лечу вправо, если правее, то в лево
REAL_RADIUS_M = 1.0

# Параметры камеры
IMG_WIDTH = 640
IMG_HEIGHT = 640
HFOV_RAD = 1.5
FOCAL_LENGTH_PX = IMG_WIDTH / HFOV_RAD
IMG_CENTER_X = IMG_WIDTH // 2
AVOIDANCE_DISTANCE = 3.0


cmd_x = 0.0
cmd_y = 0.0

while drone.get_odometry()[0][1] < FINISH_Y:
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


# Шаг 6: остановка и приземление

drone.publish_cmd_vel()

drone.land(ground_z)

drone.shutdown()



