#!/usr/bin/env python3
import numpy as np
from numpy.core.function_base import geomspace
import roslib
import tf2_ros
roslib.load_manifest("multipath_sim")
import rospy
from multipath_sim.msg import MultipathOffset

from gazebo_msgs.srv import GetModelState
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseStamped

def cb_fn(data, cb_args):
    get_model_pose = cb_args[0]
    br = cb_args[1]

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

    print("true", true_pos)
    print("offset", data.offset)
    print("affected", affected_pos)
    print("+++++++++++++++++++")
    pass

def main():
    rospy.init_node('multipath_error_vis')
    br = tf2_ros.TransformBroadcaster()

    get_model_pose = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    offset_sub = rospy.Subscriber("multipath/offset", MultipathOffset, callback=cb_fn, callback_args=[get_model_pose, br])
    

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down...")

if __name__=="__main__":
    main()