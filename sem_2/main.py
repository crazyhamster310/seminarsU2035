import time

import numpy as np

from control import ROSInterface

drone = ROSInterface()

time.sleep(2)
drone.takeoff(4.0)
drone.fly_to_pose(np.array([5.0, 10.0, 2.0]))
drone.shutdown()
