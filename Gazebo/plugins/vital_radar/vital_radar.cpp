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
    //get model name of model which includes the sensor in world file
    this->name = this->parentSensor->ScopedName();
    std::reverse(name.begin(), name.end());
    this->found = this->name.find("rosnes");
    this->name = this->name.substr(this->found);
    this->name = this->name.substr(8);
    this->found = this->name.find("::");
    this->name = this->name.substr(0, this->found);
    std::reverse(name.begin(), name.end());
}

void RayPlugin::OnNewLaserScans() {
    this->parentSensor->Ranges(ranges);

    if (this->ranges.size() > 0) {
        for(int i = 0; i < this->ranges.size(); i++) {
            if (this->ranges.at(i) <= this->parentSensor->RangeMax()) {
                this->objectCount = this->objectCount + 1;
            }
        }
        //get radar model position
        this->position = this->world->ModelByName(this->name)->WorldPose().Pos();
        
        //printf("Position: %d    Substr: %s\n",this->found, this->name.c_str());
        printf("X: %f    Y: %f    Z: %f\n", this->position.X(), this->position.Y(), this->position.Z());
        //printf("Objects: %d\n", this->objectCount);
        this->objectCount = 0;
    }
}
