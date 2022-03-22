/*
 * Copyright 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef MULTIPATH_SIM_HH
#define MULTIPATH_SIM_HH

#include <string>

#include <boost/bind.hpp>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <ros/advertise_options.h>
#include <sensor_msgs/LaserScan.h>

#include <sdf/Param.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/plugins/RayPlugin.hh>
#include <gazebo_plugins/gazebo_ros_utils.h>
#include <gazebo_plugins/PubQueue.h>

#include <visualization_msgs/Marker.h>

// custom message
#include <multipath_sim/MultipathOffset.h>


namespace gazebo
{
  class MultipathSimPlugin : public RayPlugin
  {
    /// \brief Constructor
    public: MultipathSimPlugin();

    /// \brief Destructor
    public: ~MultipathSimPlugin();

    /// \brief Load the plugin
    /// \param take in SDF root element
    public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Keep track of number of connctions
    private: int laser_connect_count_;
    private: void LaserConnect();
    private: void LaserDisconnect();

    // Pointer to the model
    GazeboRosPtr gazebo_ros_;
    private: std::string world_name_;
    private: physics::WorldPtr world_;
    private: physics::EntityPtr parent_entity_;
    /// \brief The parent sensor
    private: sensors::RaySensorPtr parent_ray_sensor_;

    /// \brief pointer to ros node
    private: ros::NodeHandle* rosnode_;
    private: ros::Publisher pub_;
    private: PubQueue<sensor_msgs::LaserScan>::Ptr pub_queue_;
    private: ros::Publisher offset_pub_;
    private: PubQueue<multipath_sim::MultipathOffset>::Ptr offset_pub_queue_;
    private: ros::Publisher sat_ray_pub_;
    private: PubQueue<visualization_msgs::Marker>::Ptr sat_ray_pub_queue_;

    /// \brief topic name
    private: std::string topic_name_;
    private: std::string offset_topic_name_;
    private: std::string sat_ray_topic_name_;

    /// \brief frame transform name, should match link name
    private: std::string frame_name_;

    /// \brief scale factor of error
    private: double error_scale_;

    /// \brief define properties of satellites
    /// TODO: maybe get elevation and azimuth from satellite's lat lon and vehicle's position on earth
    private: int num_sat_;
    private: std::vector<double> sat_dir_elevation_;
    private: std::vector<double> sat_dir_azimuth_;

    private: bool disable_noise_;

    /// \brief tf prefix
    private: std::string tf_prefix_;

    /// \brief for setting ROS name space
    private: std::string robot_namespace_;

    // deferred load in case ros is blocking
    private: sdf::ElementPtr sdf;
    private: void LoadThread();
    private: boost::thread deferred_load_thread_;

    private: gazebo::transport::NodePtr gazebo_node_;
    private: gazebo::transport::SubscriberPtr laser_scan_sub_;
    private: void OnScan(ConstLaserScanStampedPtr &_msg);

    /// \brief prevents blocking
    private: PubMultiQueue pmq;

    /// return the mirror ray index of the current ray
    private: void setRayAngles(std::vector<double> &ray_ranges);
  };
}
#endif
