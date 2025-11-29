import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from actuator_msgs.msg import Actuators
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Imu, Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from scipy.spatial.transform import Rotation as R

import cv2
import threading
import time
import numpy as np
from collections import deque


class ROSInterface:
    def __init__(self, logger=None):
        self.speed_topic = '/X3/motor_speed'
        self.odom_topic = '/model/X3/odometry'
        self.lidar_topic = '/lidar'
        
        self.logger = logger
        self._stop_event = threading.Event()
        self._node = None
        
        # Переменные для хранения обработанных сообщений из топиков
        self._latest_odom = None  # Одометрия (состояние дрона без шума)
        self._latest_lidar = None  # Лидар
        self._latest_imu = None  # IMU
        self._latest_image_data = None  # Картинка с дрона
        self.filtered_yaw_rate = 0.0  # Сглаженная скорость по yaw
        
        self.bridge = CvBridge()

        self.yaw_rate_history = deque(maxlen=20)  #
        
        if not rclpy.ok():
            rclpy.init(args=None)
            self._initialized_rclpy = True
        self._node = Node('drone_controller')
        
        if self.logger is None:
            self.logger = self._node.get_logger()
        self.logger.info("ROS 2 node initialized.")
        
        qos_sensor_data = QoSProfile(
                    reliability=QoSReliabilityPolicy.BEST_EFFORT,
                    history=QoSHistoryPolicy.KEEP_LAST,
                    depth=1
        )
        self._node.create_subscription(Imu, "/model/X3/imu", self.imu_callback, qos_sensor_data)
        self._node.create_subscription(Image, "/camera/image_raw", self.camera_callback, qos_sensor_data)
        self._node.create_subscription(LaserScan, "/lidar", self.lidar_callback, qos_sensor_data)
        self._node.create_subscription(Odometry, "/model/X3/odometry", self.odom_callback, qos_sensor_data)
        self.speed_publisher_ = self._node.create_publisher(Actuators, self.speed_topic, 10)
        self.cmd_vel_publisher_ = self._node.create_publisher(Twist, '/X3/cmd_vel', 10)

        self._executor = MultiThreadedExecutor()
        
        self._executor.add_node(self._node)
        self._spin_thread = threading.Thread(target=self._spin_executor, daemon=True)
        self._spin_thread.start()

    def publish_motor_speed(self, speed):
        msg = Actuators()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.velocity = speed
        self.speed_publisher_.publish(msg)
    
    def publish_cmd_vel(self, linear_x=0.0, linear_y=0.0, linear_z=0.0, angular_x=0.0, angular_y=0.0, angular_z=0.0):
        msg = Twist()
        msg.linear.x = float(linear_x)
        msg.linear.y = float(linear_y)
        msg.linear.z = float(linear_z)
        msg.angular.x = angular_x
        msg.angular.y = angular_y
        msg.angular.z = angular_z
        self.cmd_vel_publisher_.publish(msg)

    def _spin_executor(self):
        self.logger.info("ROS 2 spin thread started.")
        while rclpy.ok() and not self._stop_event.is_set():
            try:
                 self._executor.spin_once(timeout_sec=0.1)
            except Exception as e:
                 self.logger.error(f"Exception in ROS 2 spin thread: {e}", exc_info=True)
                 time.sleep(1)
        self.logger.info("ROS 2 spin thread stopped.")
        
    def lidar_callback(self, msg):
        if msg is not None:
            self._latest_lidar = np.array(msg.ranges)
            
    def to_local_frame(self, vector: np.ndarray) -> np.ndarray:
        # Переведем данные о вращении (в виде кватерниона) во внутренний формат библиотеки
        rot = R.from_quat(self.get_odometry()[1])
        # Эта функция возвращает необходимое движение, чтобы перейти от вектора rot к вектору vector (используем inverse из-за того, как задавали vector во fly_to_pose)
        local_vector = rot.apply(vector, inverse=True)
        return local_vector
    
    def to_world_frame(self, vector):
        rot = R.from_quat(self.get_odometry()[1])
        return rot.apply(vector)
    
    def camera_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            self._latest_image_data = cv_image.copy()
        except CvBridgeError as e:
            self.logger.error(f"Failed to convert image: {e}")
            
    def imu_callback(self, msg):
        if msg is not None:
            orientation = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
            angular_vel = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
            linear_acceleration = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
            raw_yaw_rate = angular_vel[2]
            self.yaw_rate_history.append(raw_yaw_rate)  # Добавляем полученную скорость yaw в очередь
            if len(self.yaw_rate_history) > 0:
                # Считаем сглаженную скорость по yaw как среднюю скорость по очереди
                self.filtered_yaw_rate = np.mean(self.yaw_rate_history)
            
            self._latest_imu = (orientation, angular_vel, linear_acceleration)
            
    def get_imu(self):
        return self._latest_imu
    
    def get_pose(self):
        if self._latest_odom is not None:
            pose = self._latest_odom.pose.pose
            position = np.array([pose.position.x, pose.position.y, pose.position.z])
            orientation = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
            return position, orientation
    
    def get_raw_yaw_rate(self):
        if self._latest_imu is not None:
            return self._latest_imu[1][2]
    
    def get_filtered_yaw_rate(self):
        return self.filtered_yaw_rate
    
    def get_image(self):
        return self._latest_image_data
    
    def odom_callback(self, msg):
        if msg is not None:
            self._latest_odom = msg
    
    def get_lidar(self):
        return self._latest_lidar

    def get_odometry(self):
        if self._latest_odom is not None:
            pose = self._latest_odom.pose.pose
            position = np.array([pose.position.x, pose.position.y, pose.position.z])
            orientation = np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
            vel = self._latest_odom.twist.twist
            lin_vel = np.array([vel.linear.x, vel.linear.y, vel.linear.z])
            ang_vel = np.array([vel.angular.x, vel.angular.y, vel.angular.z])
            return position, orientation, lin_vel, ang_vel
    
    def takeoff(self, z):
        while self.get_odometry()[0][2] < z:
            self.publish_cmd_vel(linear_z=0.5)
            time.sleep(1/30)
        
        self.publish_cmd_vel()
    
    def shutdown(self):
        self.logger.info("Shutting down Drone Controller...")
        self._stop_event.set()

        if self._spin_thread and self._spin_thread.is_alive():
            self.logger.info("Waiting for spin thread to finish...")
            self._spin_thread.join(timeout=2.0)
            if self._spin_thread.is_alive():
                self.logger.warning("Spin thread did not finish cleanly.")

        if self._node:
            self.logger.info(f"Destroying node {self._node.get_name()}...")
            self._node.destroy_node()
            self.logger.info("Node destroyed.")
            self._node = None

        if self._initialized_rclpy and rclpy.ok():
            self.logger.info("Shutting down rclpy...")
            rclpy.shutdown()
            self.logger.info("rclpy shut down.")

        self.logger.info("Drone Controller shutdown complete.")