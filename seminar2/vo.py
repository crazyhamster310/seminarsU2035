import numpy as np
import cv2
import time
import logging
from control import ROSInterface
from scipy.spatial.transform import Rotation

class VisualOdometry:
    def __init__(self, focal_length, pp, cam_matrix):
        """
        Инициализация.
        :param focal_length: Фокусное расстояние (float)
        :param pp: Главная точка (principal point) - кортеж (cx, cy)
        :param cam_matrix: Матрица камеры 3x3 (numpy array)
        """
        self.focal_length = focal_length
        self.pp = pp
        self.K = cam_matrix
        
        # НАСТРОЙКИ
        self.max_features = 400      # Максимум точек
        self.min_features = 150      # Порог, ниже которого ищем новые точки
        self.min_distance = 30       # Дистанция между точками в пикселях
        self.quality_level = 0.01    # Качество углов
        
        # Порог статики: если среднее смещение пикселей меньше этого, считаем что стоим
        self.motion_threshold = 0.8
        
        # Предыдущий кадр и точки
        self.old_frame_gray = None
        self.p0 = None
        
    def detect_features(self, image):
        """Используем Shi-Tomasi для поиска ограниченного числа качественных точек"""
        pts = cv2.goodFeaturesToTrack(image, self.max_features, self.quality_level, minDistance=self.min_distance)
        if pts is not None:
            return pts
        return np.empty((0, 1, 2), dtype=np.float32)

    def process_frame(self, frame_color, scale=1.0):
        vis_frame = frame_color.copy()
        frame_gray = cv2.cvtColor(frame_color, cv2.COLOR_BGR2GRAY)  # Переводим из цветного в Ч/Б
        
        # Делаем значения по умолчанию нулями
        delta_pose = np.zeros(3)
        delta_rot = np.zeros(3)

        # 1. Если это первый кадр — инициализация
        if self.old_frame_gray is None:
            self.old_frame_gray = frame_gray
            # Находим точки
            self.p0 = self.detect_features(frame_gray)
            return delta_pose, delta_rot, vis_frame 


        if self.p0 is None:
             self.p0 = self.detect_features(frame_gray)
             self.old_frame_gray = frame_gray
             return delta_pose, delta_rot, vis_frame
         
         
        # 2. Оптический поток
        # Вычисляем координаты ключевых точек старого изображения на новом изображении
        # p1 - вычисленные координаты точек на новом изображении
        # st - статус точек (нашли пару старая/новая или нет.)
        p1, st, err = cv2.calcOpticalFlowPyrLK(self.old_frame_gray, frame_gray, self.p0, 
                                               None, winSize=(21,21), 
                                               criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))

        # Выбор хороших точек
        if p1 is not None:
            good_new = p1[st==1]
            good_old = self.p0[st==1]
            
            # --- ПРОВЕРКА НА СТАТИКУ (Шумоподавление) ---
            # Считаем длину векторов смещения для каждой точки
        
            diff = good_new - good_old
            norm = np.linalg.norm(diff, axis=1)
            avg_flow = np.mean(norm) if len(norm) > 0 else 0
            
            # Если среднее движение пикселей очень мало — это просто дрожание камеры/сенсора
            is_moving = avg_flow > self.motion_threshold

            if is_moving:
                # Рисуем активные треки зеленым
                for new, old in zip(good_new, good_old):
                    a, b = new.ravel()
                    c, d = old.ravel()
                    cv2.line(vis_frame, (int(a), int(b)), (int(c), int(d)), (0, 255, 0), 2)
                    cv2.circle(vis_frame, (int(a), int(b)), 2, (0, 0, 255), -1)
                
                E, mask = cv2.findEssentialMat(good_new, good_old, self.K, method=cv2.RANSAC, prob=0.999, threshold=1.0)
                
                if E is not None:
                    _, R, t, mask_pose = cv2.recoverPose(E, good_new, good_old, self.K)
                    
                    # Конвертация в Body Frame
                    # t[x(право), y(вниз), z(вперед)]
                    forward = t[2, 0] * scale
                    left    = -t[0, 0] * scale
                    up      = -t[1, 0] * scale
                    
                    delta_pose = np.array([forward, left, up])
            else:
                # Если стоим на месте - рисуем точки желтым
                for pt in good_new:
                    x, y = pt.ravel()
                    cv2.circle(vis_frame, (int(x), int(y)), 2, (0, 255, 255), -1)
                
        else:
            self.p0 = self.detect_features(frame_gray)
            self.old_frame_gray = frame_gray
        
        return delta_pose, delta_rot, vis_frame


f = 640 / 1.5
cx = 320.0
cy = 320.0
K = np.array([[f, 0, cx], 
                [0, f, cy], 
                [0, 0, 1]])

vo = VisualOdometry(f, (cx, cy), K)
logger = logging.getLogger("VO")

drone = ROSInterface(logger=logger)
time.sleep(2)

drone.takeoff(2)

start_pose, start_orinet = drone.get_pose()

for _ in range(100):
    image = drone.get_image()
    if image is not None:
        delta_pose, delta_orient, frame = vo.process_frame(image, scale=0.105)
        if np.linalg.norm(delta_pose) > 0.1:
            start_pose += drone.to_world_frame(delta_pose)
        
        cv2.imshow('VO', frame)
        cv2.waitKey(1)
        print(f"VO pose: {start_pose}, True pose: {drone.get_pose()[0]}")
        drone.publish_cmd_vel(linear_x=0.5)
    time.sleep(0.1)

drone.publish_cmd_vel()  
drone.shutdown()
