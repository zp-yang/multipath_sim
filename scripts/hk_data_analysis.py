#!/usr/bin/env python3
from importlib import import_module
import rospy
import numpy as np
import matplotlib.pyplot as plt
# from ublox_msgs.msg import NavSAT
# from novatel_msgs.msg import BESTPOS

import rosbag
import os

def bestpos_cb(data, args):
    start = args[0]
    end = args[1]
    car_data = args[2]
    elapsed = (rospy.get_rostime() - start).to_sec()
    if elapsed > end:
        rospy.signal_shutdown("fuck")

    car_data["lat"].append(data.latitude)
    car_data["lon"].append(data.longitude)
    car_data["alt"].append(data.altitude)

def m_per_lat(lat):
    return 111132.92 - 559.82 * np.cos(2*lat) + 1.175 * np.cos(4*lat) - 0.0023 * np.cos(6*lat)

def m_per_lon(lon):
    return 111412.84 * np.cos(lon) - 93.5 * np.cos(3*lon) + 0.118 * np.cos(5*lon)

def main():
    from matplotlib.animation import FuncAnimation
    data_dir = os.path.abspath( os.path.join(os.path.dirname(__file__), os.pardir)) + "/data/" 

    bag = rosbag.Bag(data_dir+"2019-04-28-20-58-02.bag", "r")
    sat_data = {}
    num_sat = []
    
    car_data = {
        "lat": [],
        "lon": [],
        "alt": [],
    }

    topics=[
        "/ublox_node/navsat",
        "/novatel_data/bestpos",
    ]

    t_sat_list = []
    t_car_list = []
    t_list = []

    bag.read_messages(topics=topics)

    
    for topic, data, t in bag.read_messages(topics=topics):
        t_list.append(t)
        if topic == topics[0]:
            num_sat.append(data.numSvs)
            t_sat_list.append(t)
            for sv in data.sv:
                id = sv.svId
                if id in sat_data:
                    sat_data[id]["elev"].append(sv.elev)
                    sat_data[id]["azim"].append(sv.azim)
                    sat_data[id]["cno"].append(sv.cno)
                    sat_data[id]["prRes"].append(sv.prRes)
                else:
                    sat_data[id] = {
                        "elev": [sv.elev], 
                        "azim": [sv.azim],
                        "cno": [sv.cno],
                        "prRes": [sv.prRes],
                        }

        elif topic == topics[1]:
            t_car_list.append(t)
            car_data["lat"].append(data.latitude)
            car_data["lon"].append(data.longitude)
            car_data["alt"].append(data.altitude)
    
    fig = plt.figure()
    ax1 = plt.subplot(4,1,1)
    plt.ylabel("elevation")
    ax2 = plt.subplot(4,1,2)
    plt.ylabel("azimuth")
    ax3 = plt.subplot(4,1,3)
    plt.ylabel("cno")
    ax4 = plt.subplot(4,1,4)
    plt.ylabel("prRes")
    ax1.set_ylim(-90,90)
    ax2.set_ylim(0,360)

    for id in sat_data:
        if id < 5:
            ax1.plot(sat_data[id]["elev"], '.-')
            ax2.plot(sat_data[id]["azim"], '.-')

            ax3.plot(sat_data[id]["cno"], '.-')
            ax4.plot(sat_data[id]["prRes"], '.-')
    

    fig2 = plt.figure()
    plt.plot(car_data["lon"], car_data["lat"])
    plt.xlabel("lon")
    plt.ylabel("lat")

    plt.show()
    

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

