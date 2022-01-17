/*
 * Copyright 2013 Open Source Robotics Foundation
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

/*
 * Desc: Modified ros laser scanner to check for GPS sat LOS & multipath
 * Author: ZP Yang
 * Date: Dec 6 2021
 */

#include <algorithm>
#include <string>
#include <assert.h>

#include <gazebo/physics/World.hh>
#include <gazebo/physics/HingeJoint.hh>
#include <gazebo/sensors/Sensor.hh>
#include <sdf/sdf.hh>
#include <sdf/Param.hh>
#include <gazebo/common/Exception.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/transport/transport.hh>

#ifdef ENABLE_PROFILER
#include <ignition/common/Profiler.hh>
#endif

#include <tf/tf.h>
#include <tf/transform_listener.h>

#include "multipath_sim/multipath_sensor.hh"
#include <ignition/math/Rand.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Matrix3.hh>
#include <cmath>

namespace gazebo
{
// Register this plugin with the simulator
GZ_REGISTER_SENSOR_PLUGIN(MultipathSimPlugin)

////////////////////////////////////////////////////////////////////////////////
// Constructor
MultipathSimPlugin::MultipathSimPlugin()
{
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
MultipathSimPlugin::~MultipathSimPlugin()
{
  this->rosnode_->shutdown();
  delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void MultipathSimPlugin::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
{
  // load plugin
  RayPlugin::Load(_parent, this->sdf);
  // Get the world name.
  std::string worldName = _parent->WorldName();
  this->world_ = physics::get_world(worldName);
  // save pointers
  this->sdf = _sdf;

  GAZEBO_SENSORS_USING_DYNAMIC_POINTER_CAST;
  this->parent_ray_sensor_ =
    dynamic_pointer_cast<sensors::RaySensor>(_parent);

  if (!this->parent_ray_sensor_)
    gzthrow("MultipathSimPlugin controller requires a Ray Sensor as its parent");

  this->robot_namespace_ =  GetRobotNamespace(_parent, _sdf, "Laser");

  if (!this->sdf->HasElement("frameName"))
  {
    ROS_INFO_NAMED("laser", "Laser plugin missing <frameName>, defaults to /world");
    this->frame_name_ = "/world";
  }
  else
    this->frame_name_ = this->sdf->Get<std::string>("frameName");


  if (!this->sdf->HasElement("topicName"))
  {
    ROS_INFO_NAMED("laser", "Laser plugin missing <topicName>, defaults to /world");
    this->topic_name_ = "/world";
  }
  else
    this->topic_name_ = this->sdf->Get<std::string>("topicName");

  if (!this->sdf->HasElement("offsetTopicName"))
  {
    ROS_INFO_NAMED("laser", "Laser plugin missing <offsetTopicName>, defaults to /world");
    this->topic_name_ = "/world";
  }
  else
    this->offset_topic_name_ = this->sdf->Get<std::string>("offsetTopicName");


  this->laser_connect_count_ = 0;

    // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED("laser", "A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  ROS_INFO_NAMED("laser", "Starting Laser Plugin (ns = %s)", this->robot_namespace_.c_str() );
  // ros callback queue for processing subscription
  this->deferred_load_thread_ = boost::thread(
    boost::bind(&MultipathSimPlugin::LoadThread, this));

}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void MultipathSimPlugin::LoadThread()
{
  this->gazebo_node_ = gazebo::transport::NodePtr(new gazebo::transport::Node());
  this->gazebo_node_->Init(this->world_name_);

  this->pmq.startServiceThread();

  this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);

  this->tf_prefix_ = tf::getPrefixParam(*this->rosnode_);
  ROS_INFO_NAMED("laser", "Laser Plugin (ns = %s)  <tf_prefix_>, set to \"%s\"",
             this->robot_namespace_.c_str(), this->tf_prefix_.c_str());

  // resolve tf prefix
  this->frame_name_ = tf::resolve(this->tf_prefix_, this->frame_name_);

  if (this->topic_name_ != "")
  {
    ros::AdvertiseOptions ao =
      ros::AdvertiseOptions::create<sensor_msgs::LaserScan>(
      this->topic_name_, 1,
      boost::bind(&MultipathSimPlugin::LaserConnect, this),
      boost::bind(&MultipathSimPlugin::LaserDisconnect, this),
      ros::VoidPtr(), NULL);
    this->pub_ = this->rosnode_->advertise(ao);
    this->pub_queue_ = this->pmq.addPub<sensor_msgs::LaserScan>();
  }

  if (this->offset_topic_name_ != "")
  {
    ros::AdvertiseOptions ao =
      ros::AdvertiseOptions::create<multipath_sim::MultipathOffset>(
      this->offset_topic_name_, 1,
      boost::bind(&MultipathSimPlugin::LaserConnect, this),
      boost::bind(&MultipathSimPlugin::LaserDisconnect, this),
      ros::VoidPtr(), NULL);
    this->offset_pub_ = this->rosnode_->advertise(ao);
    this->offset_pub_queue_ = this->pmq.addPub<multipath_sim::MultipathOffset>();
  }

  // Initialize the controller

  // sensor generation off by default
  this->parent_ray_sensor_->SetActive(false);
}

////////////////////////////////////////////////////////////////////////////////
// Increment count
void MultipathSimPlugin::LaserConnect()
{
  this->laser_connect_count_++;
  if (this->laser_connect_count_ == 1)
    this->laser_scan_sub_ =
      this->gazebo_node_->Subscribe(this->parent_ray_sensor_->Topic(),
                                    &MultipathSimPlugin::OnScan, this);
}

////////////////////////////////////////////////////////////////////////////////
// Decrement count
void MultipathSimPlugin::LaserDisconnect()
{
  this->laser_connect_count_--;
  if (this->laser_connect_count_ == 0)
    this->laser_scan_sub_.reset();
}

////////////////////////////////////////////////////////////////////////////////
// Convert new Gazebo message to ROS message and publish it
void MultipathSimPlugin::OnScan(ConstLaserScanStampedPtr &_msg)
{
#ifdef ENABLE_PROFILER
  IGN_PROFILE("MultipathSimPlugin::OnScan");
  IGN_PROFILE_BEGIN("fill ROS message");
#endif
  // We got a new message from the Gazebo sensor.  Stuff a
  // corresponding ROS message and publish it.
  sensor_msgs::LaserScan laser_msg;
  laser_msg.header.stamp = ros::Time(_msg->time().sec(), _msg->time().nsec());
  laser_msg.header.frame_id = this->frame_name_;
  laser_msg.angle_min = _msg->scan().angle_min();
  laser_msg.angle_max = _msg->scan().angle_max();

  // gzdbg << "H Min: " << _msg->scan().angle_min() << std::endl;
  // gzdbg << "H Max: " << _msg->scan().angle_max() << std::endl;

  // gzdbg << "V Max: " << _msg->scan().vertical_angle_max() << std::endl;
  // gzdbg << "V Min: " << _msg->scan().vertical_angle_min() << std::endl;

  // first half(lower) are the the pseudo reflected rays, the second half (higher) are direct satellite rays.
  std::vector<float> sat_ray_ranges; 
  std::vector<float> rfl_ray_ranges;
  rfl_ray_ranges.resize(_msg->scan().count());
  std::copy(_msg->scan().ranges().begin(),
            _msg->scan().ranges().begin()+_msg->scan().count(),
            rfl_ray_ranges.begin());

  sat_ray_ranges.resize(_msg->scan().count());
  std::copy(_msg->scan().ranges().begin()+_msg->scan().count(),
            _msg->scan().ranges().end(),
            sat_ray_ranges.begin());

  multipath_sim::MultipathOffset offset_msg;
  offset_msg.header.stamp = ros::Time(_msg->time().sec(), _msg->time().nsec());
  offset_msg.header.frame_id = this->frame_name_;
  offset_msg.offset.resize(3);
  
  float angle_inc = _msg->scan().angle_step();
  float angle_vert_min = _msg->scan().vertical_angle_min();
  float angle_horz_min = _msg->scan().angle_min();
  float angle_horz_max = _msg->scan().angle_max();
  int range_size = _msg->scan().ranges_size();
  int v_count = _msg->scan().vertical_count();
  int direct_sat_ctr = _msg->scan().count();

  float cur_ang = _msg->scan().angle_min();

  ignition::math::Vector3<float> dir_vec{cos(angle_horz_min), sin(angle_horz_min), 0};
  ignition::math::Vector3<float> error_vec;

  // gzdbg << "Initial dir_vec0: " << dir_vec[0] << "\n";
  // gzdbg << "Initial dir_vec1: " << dir_vec[1] << "\n";
  // gzdbg << "Initial dir_vec2: " << dir_vec[2] << "\n";
  ignition::math::Matrix3<float> rot_mat;
  rot_mat(0,0) = cos(angle_inc);
  rot_mat(0,1) = sin(angle_inc);
  rot_mat(1,0) = -sin(angle_inc);
  rot_mat(1,1) = cos(angle_inc);
  for (int i=0; i < sat_ray_ranges.size(); i++)
  {
    float m = 0;
    if (sat_ray_ranges[i] < _msg->scan().range_max()) // LOS is blocked
    {
      gzdbg << "LOS blocked" << std::endl;
      direct_sat_ctr--;
      // check check mirror ray
      int mir_index = MultipathSimPlugin::mirrorRayIndex(i, _msg->scan().count());
      float mir_range = rfl_ray_ranges[mir_index];
      // gzdbg << "mir_index: " << mir_index << std::endl;
      // gzdbg << "mir_range: " << mir_range << std::endl;
      gzdbg << "\ni: " << i << "\n";
      if (mir_range < _msg->scan().range_max()) // mirror ray exists
      {
        m = mir_range * ( 1 + sin(M_PI/2.0-(_msg->scan().vertical_angle_min()+_msg->scan().vertical_angle_max())));
        error_vec += dir_vec * m;
        gzdbg << "m: " << m << std::endl;
      }
      else // GPS signal completely blocked
      {
        m = -1;
      }
    }
    dir_vec = rot_mat * dir_vec;
    cur_ang += _msg->scan().angle_step();
  }

  offset_msg.offset[0] = error_vec[0];
  offset_msg.offset[1] = error_vec[1];
  offset_msg.offset[2] = error_vec[2];

  // custom multipath offset message
  this->offset_pub_queue_->push(offset_msg, this->offset_pub_);

  laser_msg.angle_increment = _msg->scan().angle_step();
  laser_msg.time_increment = 0;  // instantaneous simulator scan
  laser_msg.scan_time = 0;  // not sure whether this is correct
  laser_msg.range_min = _msg->scan().range_min();
  laser_msg.range_max = _msg->scan().range_max();
  laser_msg.ranges.resize(_msg->scan().ranges_size());
  std::copy(_msg->scan().ranges().begin(),
            _msg->scan().ranges().end(),
            laser_msg.ranges.begin());
  laser_msg.intensities.resize(_msg->scan().intensities_size());
  std::copy(_msg->scan().intensities().begin(),
            _msg->scan().intensities().end(),
            laser_msg.intensities.begin());
  this->pub_queue_->push(laser_msg, this->pub_);
#ifdef ENABLE_PROFILER
  IGN_PROFILE_END();
#endif
}

int MultipathSimPlugin::mirrorRayIndex(int curRay, int count)
{
  // return the ray index 180 degrees offset of the current ray
  if (curRay < count/2)
  {
    return curRay + count/2;
  }
  else
  {
    return curRay % (count/2);
  }
}

}