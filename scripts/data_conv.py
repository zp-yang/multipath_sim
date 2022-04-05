import numpy as np
import rospy
import json

from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState

from concurrent.futures import ThreadPoolExecutor
from tf.transformations import quaternion_from_euler
import utm
import os

import matplotlib.pyplot as plt


data_dir = os.path.abspath( os.path.join(os.path.dirname(__file__), os.pardir)) + "/data/" 

lat_bound = np.array([22.3066, 22.2903])
lon_bound = np.array([114.171, 114.188])
origin_x = lon_bound[0] + (lon_bound[1]-lon_bound[0])/2.0
origin_y = lat_bound[0] + (lat_bound[1]-lat_bound[0])/2.0
origin = np.array([origin_x, origin_y])

origin_utm = utm.from_latlon(origin_y, origin_x)
origin_utm = np.array([origin_utm[0], origin_utm[1]])
print(origin_utm)
print(origin)

car_data = {}
with open(data_dir + "car_ground_truth.json", "r") as fstream:
    car_data = json.load(fstream)

car_data_utm = utm.from_latlon(np.array(car_data["lat"]), np.array(car_data["lon"]))
car_data_utm = np.vstack([car_data_utm[0], car_data_utm[1]]).T
print(car_data_utm.shape)
car_data_gzb = car_data_utm - origin_utm
print(car_data_gzb.shape)

# plt.figure()
# plt.plot(car_data_gzb[:, 0], car_data_gzb[:, 1])
# plt.plot(0, 0, marker='x', markersize=20)
# plt.grid()
# plt.show()

