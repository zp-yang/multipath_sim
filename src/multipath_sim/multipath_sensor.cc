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
  {
    this->frame_name_ = this->sdf->Get<std::string>("frameName");
  }

  if (!this->sdf->HasElement("topicName"))
  {
    ROS_INFO_NAMED("laser", "Laser plugin missing <topicName>, defaults to /world");
    this->topic_name_ = "/world/scan";
  }
  else
  {
    this->topic_name_ = this->sdf->Get<std::string>("topicName");
  }

  if (!this->sdf->HasElement("offsetTopicName"))
  {
    ROS_INFO_NAMED("multipath", "Multipath plugin missing <offsetTopicName>, defaults to /world");
    this->offset_topic_name_ = "/world/offset";
  }
  else
  {
    this->offset_topic_name_ = this->sdf->Get<std::string>("offsetTopicName");
  }

  if (!this->sdf->HasElement("satRayTopicName"))
  {
    ROS_INFO_NAMED("laser", "Laser plugin missing <satRayTopicName>, defaults to /world");
    this->sat_ray_topic_name_ = "/world/sat_ray";
  }
  else
  {
    this->sat_ray_topic_name_ = this->sdf->Get<std::string>("satRayTopicName");
  }

  if (!this->sdf->HasElement("errorScale"))
  {
    ROS_INFO_NAMED("multipath", "Multipath plugin missing <errorScale>, defaults to 1");
    this->error_scale_ = 1.0;
  }
  else
  {
    this->error_scale_ = this->sdf->Get<double>("errorScale");
    ROS_INFO_NAMED("multipath", "error scale set to %f\n", this->error_scale_);
  }
  if (!this->sdf->HasElement("disableNoise"))
  {
    ROS_INFO_NAMED("multipath", "Default to disable gaussian noise");
    this->disable_noise_ = true;
  }
  else
  {
    this->disable_noise_ = this->sdf->Get<bool>("disableNoise");
    ROS_INFO_NAMED("multipath", "disable noise is %d\n", this->disable_noise_);
  }

  if (!this->sdf->HasElement("satNum"))
  {
    ROS_INFO_NAMED("multipath", "Multipath plugin missing <satNum>, defaults to 8");
    this->num_sat_ = 8;
  }
  else{
    this->num_sat_ = this->sdf->Get<int>("satNum");
  }

  // Allocate enough memory for the reflection rays
  this->sat_dir_azimuth_.resize(this->num_sat_ * 2);
  this->sat_dir_elevation_.resize(this->num_sat_ * 2);

  if(!this->sdf->HasElement("satAzimuth"))
  {
    ROS_ERROR_NAMED("multipath", "Must provide azimuth of satellites");
  }
  else
  {
    std::string sat_az = this->sdf->Get<std::string>("satAzimuth");
    std::istringstream ss(sat_az);
    for (int i = 0; i < this->num_sat_; i++) {
      double sat_az_i;
      ss >> sat_az_i;
      // direct and reflection as an adjacent pair in the vector
      this->sat_dir_azimuth_[i*2] = sat_az_i;
      this->sat_dir_azimuth_[i*2+1] = sat_az_i + M_PI; // assume reflection is 180 deg azimuth from the direct ray
    }
  }

  if(!this->sdf->HasElement("satElevation"))
  {
    ROS_ERROR_NAMED("multipath", "Must provide elevation of satellites");
  }
  else
  {
    std::string sat_el = this->sdf->Get<std::string>("satElevation");
    std::istringstream ss(sat_el);
    for (int i = 0; i < this->num_sat_; i++) {
      double sat_el_i;
      ss >> sat_el_i;
      this->sat_dir_elevation_[i*2] = sat_el_i;
      this->sat_dir_elevation_[i*2+1] = sat_el_i; // assume reflection has the same elevation as the direct ray
    }
  }

  std::string parent_entity_name = this->parent_ray_sensor_->ParentName();
  ROS_INFO_NAMED("multipath",  "parent model: %s", parent_entity_name.c_str());
  this->parent_entity_ = this->world->EntityByName(parent_entity_name);

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

  if (this->sat_ray_topic_name_ != "")
  { 
    ros::AdvertiseOptions ao =
      ros::AdvertiseOptions::create<visualization_msgs::Marker>(
      this->sat_ray_topic_name_, 1,
      boost::bind(&MultipathSimPlugin::LaserConnect, this),
      boost::bind(&MultipathSimPlugin::LaserDisconnect, this),
      ros::VoidPtr(), NULL);
    this->sat_ray_pub_ = this->rosnode_->advertise(ao);
    this->sat_ray_pub_queue_ = this->pmq.addPub<visualization_msgs::Marker>();
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

  //Updating the ray angles for sateliite and reflected ray.
  setRayAngles(this->parent_ray_sensor_->LaserShape(), this->sat_dir_elevation_, this->sat_dir_azimuth_);
  //Ray_ranges stores the ranges of both satellite and reflected rays.
  std::vector<double> ray_ranges; 
  ray_ranges.resize(_msg->scan().count());
  std::copy(_msg->scan().ranges().begin(),
            _msg->scan().ranges().end(),
            ray_ranges.begin());

  multipath_sim::MultipathOffset offset_msg;
  offset_msg.header.stamp = ros::Time(_msg->time().sec(), _msg->time().nsec());
  offset_msg.header.frame_id = this->frame_name_;
  offset_msg.offset.resize(3);
  //Counter for number of satellite rays with LOS 
  int sat_ray_with_los = this->num_sat_;

  ignition::math::Vector3<double> dir_vec;
  ignition::math::Vector3<double> error_vec;
  for (int i=0; i < ray_ranges.size(); i=i+2)
  {
    double multipath_dist_increase = 0;
    dir_vec = {cos(this->sat_dir_azimuth_[i]), sin(this->sat_dir_azimuth_[i]), 0};

    // Check the LOS for sateliite ray
    if (ray_ranges[i] < _msg->scan().range_max()) 
    {
      //Decrement the counter of the satellite ray with LOS.
      sat_ray_with_los--;
      //Check range of the mirror ray corresponding to the satellite ray 
      double mir_ray_range = ray_ranges[i+1];
      if (mir_ray_range < _msg->scan().range_max()) // mirror ray exists
      {
        //Update the error vector to find the increase in the multipath distance 
        multipath_dist_increase = mir_ray_range * ( 1 + sin(M_PI/2.0 - (this->sat_dir_elevation_[i]+this->sat_dir_elevation_[i+1])));
        error_vec += dir_vec * multipath_dist_increase;
      }
    }
  }
  // no multipath or LOS with 4 direct satellites -- add noise to mocap positions
  if (this->disable_noise_)
  {
    offset_msg.offset[0] = 0;
    offset_msg.offset[0] = 0;
    offset_msg.offset[0] = 0;
  }
  else
  {
    offset_msg.offset[0] = ignition::math::Rand::DblNormal(0.0, 1.0);
    offset_msg.offset[1] = ignition::math::Rand::DblNormal(0.0, 1.0);
    offset_msg.offset[2] = ignition::math::Rand::DblNormal(0.0, 1.0);
  }
  // Adding multipath error when the satellite rays are less than 4
  if (sat_ray_with_los < 4)
  {
    // averaging from all rays and applying error scaling
    offset_msg.offset[0] += error_vec[0] / (_msg->scan().count()/2) * this->error_scale_;
    offset_msg.offset[1] += error_vec[1] / (_msg->scan().count()/2) * this->error_scale_;
    offset_msg.offset[2] += error_vec[2] / (_msg->scan().count()/2) * this->error_scale_;
  }
  
  // custom multipath offset message
  this->offset_pub_queue_->push(offset_msg, this->offset_pub_);

  // laser message
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

void MultipathSimPlugin::setRayAngles(physics::MultiRayShapePtr LaserShape, std::vector<double> elevation , std::vector<double> azimuth)
{ 
  ignition::math::Vector3d start, end, axis;
  ignition::math::Quaterniond ray;
  visualization_msgs::Marker line_list;
  line_list.header.frame_id = this->frame_name_;
  line_list.header.stamp = ros::Time::now();
  line_list.ns = "lines";
  line_list.action = visualization_msgs::Marker::ADD;
  line_list.pose.orientation.w = 1;
  line_list.type = visualization_msgs::Marker::LINE_LIST;
  line_list.scale.x = 0.05;
  line_list.color.r = 1.0;
  line_list.color.a = 1.0;
  

  for(unsigned int l=0; l < elevation.size(); l++ ) 
  {
    double yawAngle = azimuth[l];
    double pitchAngle = elevation[l];
    // since we're rotating a unit x vector, a pitch rotation will now be
    // around the negative y axis
    ray.Euler(ignition::math::Vector3d(0.0, -pitchAngle, yawAngle));
    axis = this->parent_entity_->WorldPose().Rot().Inverse() * this->parent_ray_sensor_->Pose().Rot() * ray * ignition::math::Vector3d::UnitX;
    start = (axis * LaserShape->GetMinRange()) + this->parent_ray_sensor_->Pose().Pos();
    end = (axis * LaserShape->GetMaxRange()) + this->parent_ray_sensor_->Pose().Pos();
    LaserShape->SetRay(l, start, end);
    if (!(l%2))
    {
      geometry_msgs::Point p;
      p.x = start.X();
      p.y = start.Y();
      p.z = start.Z();
      line_list.points.push_back(p);
      p.x = end.X();
      p.y = end.Y();
      p.z = end.Z();
      line_list.points.push_back(p);
    }
  }
  this->sat_ray_pub_queue_->push(line_list, this->sat_ray_pub_);
}

}