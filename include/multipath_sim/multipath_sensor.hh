#ifndef _GAZEBO_GPS_MULTIPATH_PLUGIN_HH_
#define _GAZEBO_GPS_MULTIPATH_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/util/system.hh>
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"

#include "gazebo/msgs/msgs.hh"
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/sensors/RaySensor.hh>

#include <common.h>

#include <Range.pb.h>

namespace gazebo
{
  /// \brief A Multipath Sensor Plugin
  class GAZEBO_VISIBLE MultipathPlugin : public SensorPlugin
  {
    /// \brief Constructor
    public: MultipathPlugin();

    /// \brief Destructor
    public: virtual ~MultipathPlugin();

    /// \brief Update callback
    public: virtual void OnNewLaserScans();

    /// \brief Load the plugin
    /// \param take in SDF root element
    public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Pointer to parent
    protected: physics::WorldPtr world_;

    /// \brief The parent sensor
    private:
      sensors::RaySensorPtr parentSensor_;
      std::string lidar_topic_;
      transport::NodePtr node_handle_;
      transport::PublisherPtr lidar_pub_;
      std::string namespace_;
      double min_distance_;
      double max_distance_;

      gazebo::msgs::Quaternion orientation_;

    /// \brief The connection tied to LidarPlugin::OnNewLaserScans()
    private:
      event::ConnectionPtr newLaserScansConnection_;
      sensor_msgs::msgs::Range lidar_message_;
  };
}
#endif