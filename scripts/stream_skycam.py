#!/usr/bin/env python3
from turtle import forward
from cv_bridge import CvBridge, CvBridgeError
import cv2
import rospy
from sensor_msgs.msg import Image

import numpy as np
import json
import os
import io
import matplotlib.pyplot as plt

data_dir = os.path.abspath( os.path.join(os.path.dirname(__file__), os.pardir)) + "/data/" 

def main():
    bridge = CvBridge()
    sv_filename = data_dir + "hk_data_sv_mean.json"
    sv_data = {}
    with open(sv_filename, "rb") as fstream:
        sv_data = json.load(fstream)
    print(sv_data.keys())

    
    cb_args = [bridge, sv_data]
    img_sub = rospy.Subscriber("skycam/image_raw", Image, img_sub_cb, cb_args)

def img_sub_cb(data, cb_args):
    bridge = cb_args[0]
    sv_data = cb_args[1]

    cv_img = bridge.imgmsg_to_cv2(data, "bgr8")
        
    fig = plt.figure(figsize=(5,5), dpi=100)
    ax_rect = [0, 0, 1, 1]
    ax_polar = fig.add_axes(ax_rect, projection="polar", label="ax_polar")
    ax_polar.patch.set_alpha(0.5)
    ax_polar.set_rlim(bottom=90, top=0)
    for sv_id in sv_data.keys():
        ax_polar.plot(np.deg2rad(sv_data[sv_id]["mean"][1]), sv_data[sv_id]["mean"][0], marker="o", markersize="20")
    
    img = plt2cv(fig)
    fig.clear()
    cv2.imshow("skycam", img+cv_img)
    k = cv2.waitKey(3) & 0xff

def plt2cv(fig):
    with io.BytesIO() as buff:
        fig.savefig(buff, format='raw')
        buff.seek(0)
        data = np.frombuffer(buff.getvalue(), dtype=np.uint8)
    w, h = fig.canvas.get_width_height()
    img = data.reshape((int(h), int(w), -1))

    # img is rgb, convert to opencv's default bgr
    img = cv2.cvtColor(img,cv2.COLOR_RGB2BGR)
    return img

if __name__ == "__main__":
    main()
    rospy.init_node("satview_streamer")

    try:
        rospy.spin()

    except KeyboardInterrupt:
        print("Shutting down...")
    cv2.destroyAllWindows()
