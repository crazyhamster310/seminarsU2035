from control import ROSInterface
import time
import logging
import numpy as np

GPS_COORDINATES = [
    [ 8.60454360e-06, 6.21721031e-06, 1.04610503e-01],
    [ 8.67838525e-06, -6.17695277e-06,  1.04767235e-01],
    [ -3.85241178e-06, -6.26745547e-06,  1.04761751e-01],
    [ -3.94268869e-06,  6.17762800e-06,  1.04767531e-01],
]

R = 6378137.0
logger = logging.getLogger()
drone = ROSInterface(logger=logger)
time.sleep(2)

drone.takeoff(6)

def convert_gps(v):
    x = R * np.cos(v[0]) * np.sin(v[1])
    y = R * np.sin(v[0])
    return x, y, 6


for gps_v in GPS_COORDINATES:
    print(f"Текущие GPS координаты: {gps_v}")
    x, y, z = convert_gps(gps_v)
    print(f"Преобразованные координаты: {x}, {y}, {z}")
    drone.fly_to_pose((x, y, z))


drone.shutdown()