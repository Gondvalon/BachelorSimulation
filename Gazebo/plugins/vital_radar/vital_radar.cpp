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

void RayPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) {
    this->parentSensor = std::dynamic_pointer_cast<sensors::RaySensor>(_sensor);

    if(!this->parentSensor)
        gzthrow("RayPlugin requires Ray or Lidar Sensor as its parent.\n");

    this->world = physics::get_world(this->parentSensor->WorldName());

    this->newLaserScansConnection =
            this->parentSensor->LaserShape()->ConnectNewLaserScans(std::bind(&RayPlugin::OnNewLaserScans, this));

    this->objectCount = 0;


    printf("AngleMax: %f    AngleMin: %f    AngleRes: %f\n",this->parentSensor->AngleMax().Degree(),
           this->parentSensor->AngleMin().Degree(), this->parentSensor->AngleResolution());

}

void RayPlugin::OnNewLaserScans() {
    this->parentSensor->Ranges(ranges);
    if (this->ranges.size() > 0) {
        for(int i = 0; i < this->ranges.size()-1; i++) {
            if (fabs(this->ranges.at(i)-this->ranges.at(i+1)) > 0.5) {
                this->objectCount = this->objectCount + 1;
                printf("Rays: %f    %f\n", this->ranges.at(i), this->ranges.at(i+1));
            }
        }
        printf("Objects: %d\n", this->objectCount);
    }
}
