#!/usr/bin/env python3
from importlib import import_module
import rospy
import numpy as np
import matplotlib.pyplot as plt
# from ublox_msgs.msg import NavSAT
# from novatel_msgs.msg import BESTPOS

import rosbag
import os

def main():
    data_dir = os.path.abspath( os.path.join(os.path.dirname(__file__), os.pardir)) + "/data/" 

    bag = rosbag.Bag(data_dir+"UrbanNav-HK_TST-20210517_sensors.bag", "r")
    sat_data = {}
    num_sat = []
    
    car_data = {
        "lat": [],
        "lon": [],
        "alt": [],
        "heading": [],
    }

    topics=[
        "/ublox_node/navsat",
        "/novatel_data/inspvax",
    ]

    t_sat_list = []
    t_car_list = []
    t_list = []

    bag.read_messages(topics=topics)

    
    for topic, data, t in bag.read_messages(topics=topics):
        t_list.append(t)

        if topic == topics[1]:
            t_car_list.append(t)
            car_data["lat"].append(data.latitude)
            car_data["lon"].append(data.longitude)
            car_data["alt"].append(data.altitude)
            car_data["heading"].append(data.azimuth)
    
    # fig = plt.figure()
    # ax1 = plt.subplot(4,1,1)
    # plt.ylabel("elevation")
    # ax2 = plt.subplot(4,1,2)
    # plt.ylabel("azimuth")
    # ax3 = plt.subplot(4,1,3)
    # plt.ylabel("cno")
    # ax4 = plt.subplot(4,1,4)
    # plt.ylabel("prRes")
    # ax1.set_ylim(-90,90)
    # ax2.set_ylim(0,360)

    # for id in sat_data:
    #     if id < 5:
    #         ax1.plot(sat_data[id]["elev"], '.-')
    #         ax2.plot(sat_data[id]["azim"], '.-')

    #         ax3.plot(sat_data[id]["cno"], '.-')
    #         ax4.plot(sat_data[id]["prRes"], '.-')
    

    fig2 = plt.figure()
    plt.plot(car_data["lon"], car_data["lat"])
    plt.xlabel("lon")
    plt.ylabel("lat")

    plt.show()
    
    import json
    with open(data_dir + "/car_ground_truth.json", "w") as f:
        json.dump(car_data, f)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

