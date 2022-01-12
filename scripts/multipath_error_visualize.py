#!/usr/bin/env python3
import numpy as np
from numpy.core.function_base import geomspace
from numpy.core.numeric import NaN
import roslib
import tf2_ros
roslib.load_manifest("multipath_sim")
import rospy
from multipath_sim.msg import MultipathOffset

from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

def cb_fn(data, cb_args):
    get_model_pose = cb_args[0]
    br = cb_args[1]
    odom_pub = cb_args[2]

    true_pose = get_model_pose("laser_0", "map")
    true_pos = np.array([true_pose.pose.position.x, true_pose.pose.position.y, true_pose.pose.position.z])
    affected_pos = true_pos + np.array(data.offset)
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "map"
    t.child_frame_id = "affected_pos"
    t.transform.translation.x = affected_pos[0]
    t.transform.translation.y = affected_pos[1]
    t.transform.translation.z = affected_pos[2]
    t.transform.rotation.x = 0
    t.transform.rotation.y = 0
    t.transform.rotation.z = 0
    t.transform.rotation.w = 1
    br.sendTransform(t)

    aff_odom = Odometry()
    aff_odom.header.stamp = rospy.Time.now()
    aff_odom.header.frame_id = "map"
    aff_odom.child_frame_id = "hb1"
    aff_odom.pose.pose.position.x = affected_pos[0]
    aff_odom.pose.pose.position.y = affected_pos[1]
    aff_odom.pose.pose.position.z = affected_pos[2]
    aff_odom.pose.pose.orientation = true_pose.pose.orientation
    aff_odom.twist.twist.linear.x = NaN
    aff_odom.twist.twist.linear.y = NaN
    aff_odom.twist.twist.linear.z = NaN
    aff_odom.twist.twist.angular.x = NaN
    aff_odom.twist.twist.angular.y = NaN
    aff_odom.twist.twist.angular.z = NaN

    for i in range(36):
        aff_odom.pose.covariance[i] = NaN
        aff_odom.twist.covariance[i] = NaN

    odom_pub.publish(aff_odom)

    # print("true", true_pos)
    # print("offset", data.offset)
    # print("affected", affected_pos)
    # print("+++++++++++++++++++")
    pass

def main():
    rospy.init_node('multipath_error_vis')
    br = tf2_ros.TransformBroadcaster()
    aff_odom_pub = rospy.Publisher("/multipath/hb1/aff_odom", Odometry)
    get_model_pose = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    offset_sub = rospy.Subscriber("/multipath/offset", MultipathOffset, callback=cb_fn, callback_args=[get_model_pose, br, aff_odom_pub])
    

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down...")

if __name__=="__main__":
    main()