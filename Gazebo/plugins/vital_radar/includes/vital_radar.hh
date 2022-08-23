#ifndef GAZEBO_PLUGINS_VITALRADAR_HH_
#define GAZEBO_PLUGINS_VITALRADAR_HH_

#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/util/system.hh>
#include <ignition/math/Vector3.hh>
#include <list>

namespace gazebo {
    class GZ_PLUGIN_VISIBLE RayPlugin : public SensorPlugin {
    public: RayPlugin();

    public: virtual ~RayPlugin();

    public: virtual void OnNewLaserScans();

    public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

    public: void rotateRayPoint(ignition::math::Vector3d &rotation);

    protected: physics::WorldPtr world;

    private: physics::ModelPtr radarDetection;

    private: sensors::RaySensorPtr parentSensor;

    private: event::ConnectionPtr newLaserScansConnection;

    private: int objectCount;
    private: int modelHits;
    private: int listSize;

    private: double x;
    private: double y;
    private: double z;

    private: std::vector<double> ranges;

    private: ignition::math::Vector3d position;
    private: ignition::math::Vector3d hitModelPosition;

    private: double horizontalAngleStep;
    private: double verticalAngleStep;

    private: double angleH;
    private: double angleV;

    private: std::string name;

    private: std::string modelName;

    private: std::size_t found;

    private: std::list<std::string> models;
    };
}
#endif