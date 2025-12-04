import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from scipy.spatial.transform import Rotation as R


class ROSInterface:
    def __init__(self, logger=None):
        if not rclpy.ok():
            rclpy.init()
        # Создаем ноду контроллера
        self._node = Node("drone_controller")

        # Инициализация логирования
        self.logger = logger
        if self.logger is None:
            self.logger = self._node.get_logger()
        self.logger.info("ROS 2 node initialized.")

        # Топики для одометрии и управления по скоростям
        self.vel_topic = "/X3/cmd_vel"
        self.odom_topic = "/X3/odometry"
        # Последнее известное значение одометрии
        self._latest_odom: Odometry | None = None
        # Подписка на топик одометрии
        self._node.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            QoSPresetProfiles.SENSOR_DATA.value,
        )
        # Паблишер в топик скоростей
        self.cmd_vel_publisher_ = self._node.create_publisher(
            Twist, self.vel_topic, 10
        )

        # Событие для остановки контроллера
        self._stop_event = threading.Event()
        # Исполнитель для нод
        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self._node)
        # Начинаем крутить исполнитель в отдельном потоке, чтобы не стопорить работу
        self._spin_thread = threading.Thread(
            target=self._spin_executor, daemon=True
        )
        self._spin_thread.start()

    def _spin_executor(self) -> None:
        self.logger.info("ROS 2 spin thread started.")

        # Пока все хорошо и от нас не требуют остановку
        while rclpy.ok() and not self._stop_event.is_set():
            # Пытаемся прокрутить, либо получая одно сообщение из какого-нибудь топика, либо выходя через 0.1 с
            try:
                self._executor.spin_once(timeout_sec=0.1)
            # При ошибке делаем соответствующую запись в лог и ждем чуда
            except Exception as e:
                self.logger.error(
                    f"Exception in ROS 2 spin thread: {e}", exc_info=True
                )
                time.sleep(1)

        self.logger.info("ROS 2 spin thread stopped.")

    def odom_callback(self, msg: Odometry | None) -> None:
        # Если пришли новые данные, обновляем переменную
        if msg is not None:
            self._latest_odom = msg

    def get_odometry(
        self,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray] | None:
        # Если есть какие-то данные, запишем их для удобства в 4 массива и вернем из функции
        if self._latest_odom is not None:
            pose = self._latest_odom.pose.pose
            position = np.array(
                [pose.position.x, pose.position.y, pose.position.z]
            )
            orientation = np.array(
                [
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                ]
            )

            vel = self._latest_odom.twist.twist
            lin_vel = np.array([vel.linear.x, vel.linear.y, vel.linear.z])
            ang_vel = np.array([vel.angular.x, vel.angular.y, vel.angular.z])

            self.logger.debug(f"{time.time()},{position[0]},{position[1]},{position[2]},{lin_vel[0]},{lin_vel[1]},{lin_vel[2]}")
            return position, orientation, lin_vel, ang_vel

    def publish_cmd_vel(
        self,
        linear_x: float = 0.0,
        linear_y: float = 0.0,
        linear_z: float = 0.0,
        angular_x: float = 0.0,
        angular_y: float = 0.0,
        angular_z: float = 0.0,
    ) -> None:
        # Создадим объект для сообщения, заполним и опубликуем
        msg = Twist()
        msg.linear.x = linear_x
        msg.linear.y = linear_y
        msg.linear.z = linear_z
        msg.angular.x = angular_x
        msg.angular.y = angular_y
        msg.angular.z = angular_z
        self.cmd_vel_publisher_.publish(msg)

    def takeoff(self, z: float) -> None:
        self.logger.info(f"Takeoff to altitude {z}")

        # Подождем, пока не появятся данные одометрии
        while self._latest_odom is None:
            pass

        # Пока ниже нужной точки, немного взлетаем
        while self.get_odometry()[0][2] < z:
            self.publish_cmd_vel(linear_z=0.5)
            time.sleep(1 / 30)

        # Установим нулевую скорость, чтобы остановиться в нужной точке
        self.publish_cmd_vel()

    def to_local_frame(self, vector: np.ndarray) -> np.ndarray:
        # Переведем данные о вращении (в виде кватерниона) во внутренний формат библиотеки
        rot = R.from_quat(self.get_odometry()[1])
        # Эта функция возвращает необходимое движение, чтобы перейти от вектора rot к вектору vector (используем inverse из-за того, как задавали vector во fly_to_pose)
        local_vector = rot.apply(vector, inverse=True)
        return local_vector

    def fly_to_pose(self, pose: np.ndarray) -> None:
        self.logger.info(f"Flying to pose {pose}")

        # Коэффициент P-составляющей
        kp = 0.2

        pose_error = pose - self.get_odometry()[0]
        # Живем в цикле, пока достаточно далеко (в смысле нормы вектора ошибки) от точки
        while np.linalg.norm(pose_error) > 0.1:
            # Посчитаем отклонение
            pose_error = pose - self.get_odometry()[0]
            # и управляющее воздействие для движения в направлении целевой точки
            linear_cmd = self.to_local_frame(kp * pose_error)

            # Опубликуем скорости
            self.publish_cmd_vel(*linear_cmd)

            time.sleep(0.1)

        # Остановимся в целевой точке
        self.publish_cmd_vel()

    def shutdown(self) -> None:
        self.logger.info("Shutting down Drone Controller...")

        # Установим флаг, что пора останавливаться
        self._stop_event.set()

        # Если поток для исполнителя еще жив, зайдем в него и подождем остановки (или выведем предупреждение в лог)
        if self._spin_thread and self._spin_thread.is_alive():
            self.logger.info("Waiting for spin thread to finish...")
            self._spin_thread.join(timeout=2.0)
            if self._spin_thread.is_alive():
                self.logger.warning("Spin thread did not finish cleanly.")

        # Если нода еще существует, уничтожим ее
        if self._node:
            self.logger.info(f"Destroying node {self._node.get_name()}...")
            self._node.destroy_node()
            self.logger.info("Node destroyed.")
            self._node = None

        # Если rclpy в рабочем состоянии, отключим
        if rclpy.ok():
            self.logger.info("Shutting down rclpy...")
            rclpy.shutdown()
            self.logger.info("rclpy shut down.")

        self.logger.info("Drone Controller shutdown complete.")
