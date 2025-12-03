from control import ROSInterface
import logging
import time
import numpy as np
import matplotlib.pyplot as plt

logger = logging.getLogger()

drone = ROSInterface(logger=logger)

time.sleep(2)
drone.takeoff(4)
filtered_hist = []
raw_hist = []
true_hist = []


drone.publish_cmd_vel(angular_z=1.0)

for _ in range(60):
    filtered_hist.append(drone.get_filtered_yaw_rate())
    raw_hist.append(drone.get_raw_yaw_rate())
    true_hist.append(drone.get_odometry()[3][2])
    time.sleep(0.1)
    

fig = plt.figure(figsize=(15, 20))
x = np.linspace(0, 60, 60)
plt.plot(x, raw_hist, label='raw')
plt.plot(x, filtered_hist, label='filtered')
plt.plot(x, true_hist, label='true')
plt.legend()
plt.show()

drone.publish_cmd_vel(angular_z=0.0)
drone.shutdown()
