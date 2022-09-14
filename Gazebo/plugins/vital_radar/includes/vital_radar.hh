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

        
    private:
        sensors::RaySensorPtr sensor;
        physics::WorldPtr world;

        event::ConnectionPtr newLaserScansConnection;

        std::vector<double> ranges;

        physics::RayShapePtr rayShape;

        physics::ModelPtr modelHitByRay;

        //calculating new Rays
        ignition::math::Vector3d rayStart;
        ignition::math::Vector3d rayEnd;
        ignition::math::Vector3d rayGradient;

        double heartRate;
        double respiratoryRate;

        //parameters changable in sdf sensor plugin
        int penetrableWalls;
        double signalStrength;
        //damping per cm
        double airDamping;
        double wallDamping;


        std::list<std::string> models;
        int modelHits;
    };
}
#endif