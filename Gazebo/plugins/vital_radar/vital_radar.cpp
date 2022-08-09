#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <cmath>

#include <functional>
#include "gazebo/physics/physics.hh"
#include "vital_radar.hh"

using namespace gazebo;

GZ_REGISTER_SENSOR_PLUGIN(RayPlugin)

RayPlugin::RayPlugin() {}

RayPlugin::~RayPlugin() {
    this->newLaserScansConnection.reset();

    this->parentSensor.reset();
    this->world.reset();
}

void RayPlugin::Load(sensors::SensorPtr _parent, sdf::ElementPtr /*_sdf*/) {
    this->parentSensor = std::dynamic_pointer_cast<sensors::RaySensor>(_parent);

    if(!this->parentSensor)
        printf("Sadly wrong sensor\n");
        gzthrow("RayPlugin requires Ray Senspr as its parent.\n");

    this->world = physics::get_world(this->parentSensor->WorldName());

    this->newLaserScansConnection =
            this->parentSensor->LaserShape()->ConnectNewLaserScans(std::bind(&RayPlugin::OnNewLaserScans, this));

    printf("I did it\n");

}

void RayPlugin::OnNewLaserScans() {

}
