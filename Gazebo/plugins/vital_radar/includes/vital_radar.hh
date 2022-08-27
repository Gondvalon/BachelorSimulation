#ifndef GAZEBO_PLUGINS_VITALRADAR_HH_
#define GAZEBO_PLUGINS_VITALRADAR_HH_

#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/physics/MultiRayShape.hh>
#include <gazebo/physics/RayShape.hh>
#include <gazebo/util/system.hh>
#include <ignition/math/Vector3.hh>
#include <list>

namespace gazebo {
    class GZ_PLUGIN_VISIBLE RayPlugin : public SensorPlugin {
    public:
        RayPlugin();

        virtual ~RayPlugin();

        virtual void OnNewLaserScans();

        void Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

        void rotateRayPoint(ignition::math::Vector3d &rotation);

        protected: physics::WorldPtr world;

        
    private:
        physics::ModelPtr radarDetection;

        sensors::RaySensorPtr parentSensor;
        physics::MultiRayShapePtr multiRayShape;
        physics::RayShapePtr rayShape;

        event::ConnectionPtr newLaserScansConnection;

        int modelHits;

        std::vector<double> ranges;

        ignition::math::Vector3d position;
        ignition::math::Vector3d hitModelPosition;

        ignition::math::Vector3d rayStart;
        ignition::math::Vector3d rayEnd;
        ignition::math::Vector3d rayGradient;

        double rayTravelDist;

        double horizontalAngleStep;
        double verticalAngleStep;

        double angleH;
        double angleV;

        std::string name;

        std::string hitModelName;

        std::size_t found;

        std::list<std::string> models;
    };
}
#endif