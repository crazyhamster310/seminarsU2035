from control import ROSInterface
import logging
import time
import numpy as np
import matplotlib.pyplot as plt

logger = logging.getLogger()

drone = ROSInterface(logger=logger)

time.sleep(2)

drone.takeoff(1)


# Собираем сигнал о скорости yaw в массивы
filtered_hist = []  # Массив для значений сглаженного фильтром yaw 
raw_hist = []  # Массив для сырых значений скорости yaw
true_hist = []  # Массив для реальных (не зашумелнных) значений yaw

drone.publish_cmd_vel(angular_z=-2.0)  # Говорим дрону вращаться по yaw со скоростью -2.0


# В течении 6 секунд собираем данные и добавляем в соответствующие массивы
for _ in range(60):
    filtered_hist.append(drone.get_filtered_yaw_rate())
    raw_hist.append(drone.get_raw_yaw_rate())
    true_hist.append(drone.get_odometry()[3][2])
    time.sleep(0.1)
    

drone.publish_cmd_vel()  # Останавливаем дрон

# Визуализация собранных данных
fig = plt.figure(figsize=(15, 20))
x = np.linspace(0, 60, 60)

plt.plot(x, filtered_hist, label='filtered')  # Рисуем график для обработанных yaw
plt.plot(x, raw_hist, label='raw')  # Рисуем график для сырых yaw
plt.plot(x, true_hist, label='true')  # Рисуем график для реальных значений yaw
plt.legend()
plt.show()

drone.shutdown()


