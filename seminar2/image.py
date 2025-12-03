import cv2
import numpy as np

image = cv2.imread('./img2.png', cv2.IMREAD_COLOR)

image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

lower_bound = np.array([0, 50, 70])
upper_bound = np.array([10, 255, 255])

mask = cv2.inRange(image, lower_bound, upper_bound)

cv2.imshow("Image", mask)

cv2.waitKey(0)