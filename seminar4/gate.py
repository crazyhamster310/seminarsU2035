from control import ROSInterface
import numpy as np
import cv2
import logging
import time
from ultralytics import YOLO


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
        self.dead_zone = self.target_area * 0.1  # Допустимая погрешность
        
        # --- Коэффициенты P-регулятора
        self.kp_forward = 0.2
        self.kp_y = 1.5
        self.kp_z = 0.5
        
        # Ограничения скоростей
        self.max_x_vel = 0.6
        self.max_y_vel = 0.5
        self.max_z_vel = 0.5
        
        
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
            
            if gate_area > self.target_area:
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
                z_speed = norm_error_y * self.kp_z
                
                forward_speed = 0.0
                if abs(error_area) > self.dead_zone:
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
                self.drone.publish_cmd_vel(linear_z=-0.1)
                time.sleep(1/30)
                print('Lost gate')
                print('Rotate drone to find gate')
                self.drone.publish_cmd_vel(angular_z=-0.1)                
                
            cv2.imshow("Gate Centering", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.drone.publish_cmd_vel()
                break
        
        cv2.destroyAllWindows()


def pass_gate():
    pose = drone.get_pose()[0]
    pose_bf = drone.to_local_frame(pose)
    pose_bf[0] += 3.0
    pose_bf[2] = 2.0
    pose = drone.to_world_frame(pose_bf)
    drone.fly_to_pose(pose)
                

logger = logging.getLogger()
drone = ROSInterface(logger)
time.sleep(2)

drone.takeoff(2)

gate_controller= GateCeneteringControl(drone, './models_pt/close_blur_rainbow_shadowed_1-40.pt')

cnt_gate = 3
print("Wait camera on")
while drone.get_image() is None:
    time.sleep(0.1)

print("Ready to fly")
for i in range(cnt_gate):
    gate_controller.run()
    pass_gate()

drone.shutdown()