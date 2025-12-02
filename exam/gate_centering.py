from control import ROSInterface
import numpy as np
import cv2
import logging
import time
from ultralytics import YOLO


class GateCenteringController:
    def __init__(self, drone, model_path):
        self.drone = drone
        self.model = YOLO(model_path)

        self.image_width = 640
        self.image_height = 640
        self.image_center_x = self.image_width // 2
        self.image_center_y = self.image_height // 2

        self.target_area = (self.image_width * self.image_height) * 0.2
        self.dead_zone = self.target_area * 0.1

        # --- Коэффициенты P-регулятора ---
        self.Kp_y = 1.5       # Коэффициент для боковой скорости
        self.Kp_z = 1       # Коэффициент для вертикальной скорости
        self.Kp_forward = 0.2  # Коэффициент для скорости движения вперед

        # Ограничения максимальных скоростей
        self.max_y_speed = 0.6
        self.max_z_speed = 0.5
        self.max_fwd_speed = 0.5

    def find_best_gate(self, results):
        best_box = None
        max_area = 0
        if not results or not results[0].boxes:
            return None, 0
        for box in results[0].boxes:
            if box.xyxy.shape[1] == 4:
                b = box.xyxy[0].cpu().numpy()
                area = (b[2] - b[0]) * (b[3] - b[1])
                if area > max_area:
                    max_area = area
                    best_box = b
        return best_box, max_area

    def run(self):
        while True:
            time.sleep(1/30)
            frame = self.drone.get_image()
            if frame is None:
                print('No image')
                continue

            results = self.model(frame, verbose=False)
            gate_box, current_area = self.find_best_gate(results)
            
            if current_area > self.target_area:
                self.drone.publish_cmd_vel()
                break
            
            if gate_box is not None:
                x1, y1, x2, y2 = gate_box
                box_center_x = (x1 + x2) / 2
                box_center_y = (y1 + y2) / 2

                error_x = box_center_x - self.image_center_x
                error_y = self.image_center_y - box_center_y
                error_area = self.target_area - current_area
                norm_error_x = error_x / self.image_center_x
                norm_error_y = error_y / self.image_center_y

                y_speed = -self.Kp_y * norm_error_x

                z_speed = self.Kp_z * norm_error_y

                fwd_speed = 0.0
                if abs(error_area) > self.dead_zone:
                    fwd_speed = self.Kp_forward * error_area

                # Ограничиваем скорости
                y_speed = np.clip(y_speed, -self.max_y_speed, self.max_y_speed)
                z_speed = np.clip(z_speed, -self.max_z_speed, self.max_z_speed)
                fwd_speed = np.clip(
                    fwd_speed, -self.max_fwd_speed, self.max_fwd_speed)

                print(f'x: {fwd_speed}, y: {y_speed}, z: {z_speed}')
                self.drone.publish_cmd_vel(fwd_speed, y_speed, z_speed)

                # Визуализация
                cv2.rectangle(frame, (int(x1), int(y1)),
                              (int(x2), int(y2)), (0, 255, 0), 2)
                cv2.circle(frame, (int(box_center_x), int(
                    box_center_y)), 5, (255, 0, 0), -1)

            else:
                self.drone.publish_cmd_vel()

            cv2.imshow("Drone View", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.drone.publish_cmd_vel()
                break
        cv2.destroyAllWindows()
        

def cross_gate():
    pose = drone.get_odometry()[0]
    pose[1] += 3.0
    pose[2] = 2.0
    drone.fly_to_pose(pose)

logger = logging.getLogger()
drone = ROSInterface(logger=logger)
model_path = 'models_pt/close_blur_rainbow_shadowed_1-40.pt'
time.sleep(2)
drone.takeoff(2)
controller = GateCenteringController(drone=drone, model_path=model_path)

cnt_gates = 3
for i in range(cnt_gates):
    controller.run()
    cross_gate()

drone.shutdown()
