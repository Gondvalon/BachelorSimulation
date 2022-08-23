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
    //get model name of model which includes the sensor model
    this->name = this->parentSensor->ScopedName();
    printf("scopename: %s  Modelnum: %d\n", this->name.c_str(), this->world->ModelCount());
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

    this->position = this->world->ModelByName(this->name)->WorldPose().Rot().Euler();
    //printf("RollX: %f    RollY: %f    RollZ: %f\n", this->position.X(), this->position.Y(), this->position.Z());

    if (this->ranges.size() > 0) {
        for(int i = 0; i < this->ranges.size(); i++) {
            if (this->ranges.at(i) <= this->parentSensor->RangeMax()) {
                this->objectCount = this->objectCount + 1;
            }
        }
        //gets position of model in world which includes this sensor
        //this->position = this->world->ModelByName(this->name)->WorldPose().Pos();
        //printf("Position: %d    Substr: %s\n",this->found, this->name.c_str());
        //printf("X: %f    Y: %f    Z: %f\n", this->position.X(), this->position.Y(), this->position.Z());
        //printf("Objects: %d\n", this->objectCount);

        //calculate the position of the object hit by ray
        this->x = this->ranges.at(0)*cos(this->parentSensor->AngleMin().Radian())*cos(this->parentSensor->VerticalAngleMax().Radian());
        this->y = this->ranges.at(0)*sin(this->parentSensor->AngleMin().Radian())*cos(this->parentSensor->VerticalAngleMax().Radian());
        this->z = this->ranges.at(0)*(-1)*sin(this->parentSensor->VerticalAngleMax().Radian());
        //this->radarDetection = this->world->ModelBelowPoint(this->world->ModelByName(this->name)->WorldPose().Pos()
        //+ ignition::math::Vector3d(this->x, this->y, this->z));


        //calculate the rotation of the found hit object
        this->hitModelPosition = ignition::math::Vector3d(this->x * cos(this->position.Z()) * cos(this->position.Y())
                + this->y * (cos(this->position.Z()) * sin(this->position.Y()) * sin(this->position.X()) - sin(this->position.Z()) * cos(this->position.X()))
                + this->z * (cos(this->position.Z()) * sin(this->position.Y()) * cos(this->position.X()) + sin(this->position.Z()) * sin(this->position.X())),
                this->x * sin(this->position.Z()) * cos(this->position.Y())
                + this->y * (sin(this->position.Z()) * sin(this->position.Y()) * sin(this->position.X()) + cos(this->position.Z()) * cos(this->position.X()))
                + this->z * (sin(this->position.Z()) * sin(this->position.Y()) * cos(this->position.X()) - cos(this->position.Z()) * sin(this->position.X())),
                this->x * (-1) * sin(this->position.Y()) + this->y * cos(this->position.Y()) * sin(this->position.X())
                + this->z * cos(this->position.Y()) * cos(this->position.X()));

        this->radarDetection = this->world->ModelBelowPoint(this->world->ModelByName(this->name)->WorldPose().Pos()
                + this->hitModelPosition);


        if(!(this->radarDetection == NULL)) {
            this->modelName = this->radarDetection->GetName();
            printf("FoundModel: %s\n", this->modelName.c_str());
        }
        /*printf("Range: %f  LookingAt X: %f   Y: %f   Z: %f\n\n", this->ranges.at(0), (this->world->ModelByName(this->name)->WorldPose().Pos()
                + ignition::math::Vector3d(this->x, this->y, this->z)).X(), (this->world->ModelByName(this->name)->WorldPose().Pos()
                + ignition::math::Vector3d(this->x, this->y, this->z)).Y(), (this->world->ModelByName(this->name)->WorldPose().Pos()
                + ignition::math::Vector3d(this->x, this->y, this->z)).Z());
                */
        printf("Range: %f  LookingAt X: %f   Y: %f   Z: %f\n\n", this->ranges.at(0),
               this->hitModelPosition.X() + this->world->ModelByName(this->name)->WorldPose().Pos().X(),
               this->hitModelPosition.Y() + this->world->ModelByName(this->name)->WorldPose().Pos().Y(),
               this->hitModelPosition.Z() + this->world->ModelByName(this->name)->WorldPose().Pos().Z());

        this->objectCount = 0;
    }
}

void rotateRayPoint(ignition::math::Vector3d &rotation) {
    //rotates the found point of the radar, so it fits with the coordinates of the robot

}
