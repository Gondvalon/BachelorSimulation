#ifndef GAZEBO_PLUGINS_VITALRADAR_HH_
#define GAZEBO_PLUGINS_VITALRADAR_HH_

#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/util/system.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo {
    class GZ_PLUGIN_VISIBLE RayPlugin : public SensorPlugin {
    public: RayPlugin();

    public: virtual ~RayPlugin();

    public: virtual void OnNewLaserScans();

    public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

    protected: physics::WorldPtr world;

    private: sensors::RaySensorPtr parentSensor;

    private: event::ConnectionPtr newLaserScansConnection;

    private: int objectCount;

    private: std::vector<double> ranges;

    private: ignition::math::Vector3d position;
    private: std::string name;

    private: std::size_t found;
    };
}
#endif