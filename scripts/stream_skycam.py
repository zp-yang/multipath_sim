#!/usr/bin/env python3
from cv_bridge import CvBridge, CvBridgeError
import cv2
import rospy
from sensor_msgs.msg import Image
from gazebo_msgs.srv import GetModelState
from tf.transformations import euler_from_quaternion

import numpy as np
import json
import os

data_dir = os.path.abspath( os.path.join(os.path.dirname(__file__), os.pardir)) + "/data/" 

def main():
    rospy.init_node("satview_streamer")
    
    bridge = CvBridge()
    sv_filename = data_dir + "hk_data_sv_mean.json"
    sv_data = {}
    with open(sv_filename, "rb") as fstream:
        sv_data = json.load(fstream)
    print(sv_data.keys())
    cno_max = np.max([sv_data[key]["cno_max"] for key in sv_data.keys()])
    gms = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
    img_pub = rospy.Publisher("skycam/satview", Image, queue_size=1)
    start_time = rospy.get_rostime()
    cb_args = [bridge, sv_data, gms, img_pub, start_time, cno_max]
    img_sub = rospy.Subscriber("skycam/image_raw", Image, img_sub_cb, cb_args)
    

def img_sub_cb(data, cb_args):
    bridge = cb_args[0]
    sv_data = cb_args[1]
    gms = cb_args[2]
    img_pub = cb_args[3]
    start_time = cb_args[4]
    cno_max = cb_args[5]

    model_pose = gms("laser_0", "world")
    model_euler = euler_from_quaternion(
        [model_pose.pose.orientation.x,
        model_pose.pose.orientation.y,
        model_pose.pose.orientation.z,
        model_pose.pose.orientation.w,]
        )
    # ENU (gzb) to NED
    heading = np.pi/2 - model_euler[2]

    cv_img = bridge.imgmsg_to_cv2(data, "bgr8")
    img_height = cv_img.shape[0]
    img_width = cv_img.shape[1]
    img_center = np.array([img_height/2.0, img_width/2.0]) # [250 250]
    
    r_max = np.min(img_center)
    green = (0, 255, 0)
    red = (0, 0, 255)
    blue = (255, 0, 0)
    
    now = rospy.get_rostime()
    elapsed = (now-start_time).to_sec()
    
    for sv_id in sv_data.keys():
        elev = sv_data[sv_id]["mean"][0]
        azim = sv_data[sv_id]["mean"][1]

        index = int(elapsed*10 % len(sv_data[sv_id]["cno"]))
        cno = sv_data[sv_id]["cno"][index]
        # print(sv_id+" cno: ", cno)
        # print(sv_id+" color: ", int((cno)/cno_max*255), int((cno_max - cno)/cno_max*255))

        r = (90.0 - elev)/90.0 * r_max
        theta = np.deg2rad(azim) - np.pi/2 - heading

        x = int(r*np.cos(theta) + img_center[0])
        y = int(r*np.sin(theta) + img_center[1])

        cv2.circle(cv_img, (x, y), 10, (0, int((cno)/cno_max*255), int((cno_max-cno)/cno_max*255)/2), -1)
        cv2.circle(cv_img, (x, y), 11, (0, 0, 255), 2)
    
    nesw = ["N", "E", "S", "W"]
    for i in range(4):
        theta = i*np.pi/2 - np.pi/2 - heading
        r = 235
        x = int(r*np.cos(theta) + img_center[0])
        y = int(r*np.sin(theta) + img_center[1])
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(cv_img, nesw[i], (x,y), font, 0.5, green, 2)
    
    ros_img = bridge.cv2_to_imgmsg(cv_img, "bgr8")
    img_pub.publish(ros_img)
    # cv2.imshow("skycam", cv_img)
    # k = cv2.waitKey(3) & 0xff

if __name__ == "__main__":
    main()

    try:
        rospy.spin()

    except KeyboardInterrupt:
        print("Shutting down...")
    cv2.destroyAllWindows()
