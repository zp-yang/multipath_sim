#!/usr/bin/env python3
import os
import numpy as np
from numpy.core.arrayprint import printoptions
from numpy.linalg import pinv
from genpy.rostime import Duration
import rospy 
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState
import geometry_msgs.msg
from concurrent.futures import ThreadPoolExecutor
import numpy as np
import utm
import json
from tf.transformations import quaternion_from_euler

def target_traj_circle(t, *args):
    begin = args[0]
    theta = 2 * np.pi * 0.1 * t / 3 + np.arctan2(begin[1],begin[0])
    x = 19*np.sin(theta)
    y = 14*np.cos(theta)
    z = begin[2]
    yaw = -(theta + np.pi/2)
    pos = np.array([x, y, z])
    # return pos
    return np.array([x,y,z, 0, 0, yaw])

def target_traj_stationary(t, *args):
    begin = args[0]
    return np.concatenate([begin, [0,0,0]])

def targeet_traj_rotate(t, *args):
    begin = args[0]
    theta = 2 * np.pi * 0.1 * t / 3 + np.arctan2(begin[1],begin[0])
    yaw = -(theta + np.pi/2)
    return np.concatenate([begin, [0,0,yaw]])

def target_traj_straight(t, *args):
    begin = args[0]
    end = args[1]
    duration = args[2]
    trip = int(t / duration)

    if (trip % 2): # odd trip
        temp = begin
        begin = end
        end = temp

    max_dist = np.linalg.norm(begin-end)
    v_max = (end-begin) / duration
    #print(v_max)
    pos = begin + v_max * (t - trip*duration)
    #print(pos)
    att = np.array([0,0,0])
    # return pos
    return np.concatenate([pos, att])

def hk_traj(t,args):
    car_data_gzb = args
    index = int(t % car_data_gzb.shape[0])
    pos = [car_data_gzb[index,0],car_data_gzb[index,1],0]
    att = np.array([0,0,0])
    return np.concatenate([pos, att])

def set_drone_state(*args):
    args = args[0]
    model_name = args[0]
    traj_fn = args[1]
    traj_fn_args = args[2]
    # begin = traj_fn_args[0]
    # end = args[3]
    # duration = args[4]
    
    car_data_gzb = traj_fn_args
    begin = [car_data_gzb[0,0], car_data_gzb[0,1], 0]
    

    state_msg_0 = ModelState()
    state_msg_0.model_name = model_name
    state_msg_0.pose.position.x = begin[0]
    state_msg_0.pose.position.y = begin[1]
    state_msg_0.pose.position.z = begin[2]
    state_msg_0.pose.orientation.x = 0
    state_msg_0.pose.orientation.y = 0
    state_msg_0.pose.orientation.z = 0
    state_msg_0.pose.orientation.w = 1

    rospy.wait_for_service('/gazebo/set_model_state')
    start = rospy.get_rostime()
    elapsed = 0
    end_time = 60
    rate = rospy.Rate(1000)
    while not rospy.is_shutdown():
    # while elapsed <= end_time:
        now = rospy.get_rostime()
        elapsed = (now - start).to_sec()
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            #pose = traj_fn(elapsed, *traj_fn_args)
            pose = traj_fn(elapsed, traj_fn_args)
            print(elapsed, pose)
            state_msg_0.pose.position.x = pose[0]
            state_msg_0.pose.position.y = pose[1]
            state_msg_0.pose.position.z = pose[2]
            
            q_ = quaternion_from_euler(pose[3], pose[4], pose[5])
            state_msg_0.pose.orientation.x = q_[0]
            state_msg_0.pose.orientation.y = q_[1]
            state_msg_0.pose.orientation.z = q_[2]
            state_msg_0.pose.orientation.w = q_[3]
            
            set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            resp = set_state( state_msg_0 )

        except rospy.ServiceException as e:
            print("Service call failed: {:s}".format(str(e)))
        rate.sleep()
    pass

def hk_preprocess():
    data_dir = os.path.abspath( os.path.join(os.path.dirname(__file__), os.pardir)) + "/data/" 
    lat_bound = np.array([22.3066, 22.2903])
    lon_bound = np.array([114.171, 114.188])
    origin_x = lon_bound[0] + (lon_bound[1]-lon_bound[0])/2.0
    origin_y = lat_bound[0] + (lat_bound[1]-lat_bound[0])/2.0
    origin = np.array([origin_x, origin_y])
    origin_utm = utm.from_latlon(origin_y, origin_x)
    origin_utm = np.array([origin_utm[0], origin_utm[1]])
    car_data = {}
    with open(data_dir + "car_ground_truth.json", "r") as fstream:
        car_data = json.load(fstream)
    car_data_utm = utm.from_latlon(np.array(car_data["lat"]), np.array(car_data["lon"]))
    car_data_utm = np.vstack([car_data_utm[0], car_data_utm[1]]).T
    car_data_gzb = car_data_utm - origin_utm
    return car_data_gzb

def main():
    rospy.init_node('set_pose')
    x0_1 = np.array([-30, 5, 10])
    x0_2 = np.array([20, 5, 20])
    car_data_gzb = hk_preprocess()
    executor_args = [
        #["laser_0", target_traj_straight, [x0_1, x0_1+[60,0,0], 30]],
        ["laser_0", hk_traj, car_data_gzb],
        # ["laser_0", target_traj_stationary, [x0_1]],
        # ["laser_0", targeet_traj_rotate, [[0,0,0]]],
        # ["drone_1", target_traj_straight, [x0_2, x0_2+[-40,0,0], 30]],
        # ["drone_0", target_traj_circle, [[-10, 0, 18], [-10, 0, 18], 30]],
    ]
    
    with ThreadPoolExecutor(max_workers=5) as tpe:
        #print("started")
        tpe.map(set_drone_state, executor_args)
    #set_drone_state(executor_args[0])
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
