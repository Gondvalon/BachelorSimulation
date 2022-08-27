#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <cmath>
#include <list>

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

    printf("AngleMax: %f    AngleMin: %f    AngleRes: %f\n",this->parentSensor->AngleMax().Degree(),
           this->parentSensor->AngleMin().Degree(), this->parentSensor->AngleResolution());
    //get model name of model which includes the sensor model
    this->name = this->parentSensor->ScopedName();
    //printf("scopename: %s  Modelnum: %d\n", this->name.c_str(), this->world->ModelCount());
    std::reverse(name.begin(), name.end());
    this->found = this->name.find("rosnes");
    this->name = this->name.substr(this->found);
    this->name = this->name.substr(8);
    this->found = this->name.find("::");
    this->name = this->name.substr(0, this->found);
    std::reverse(name.begin(), name.end());

    this->horizontalAngleStep = (this->parentSensor->AngleMax() - this->parentSensor->AngleMin()).Radian()
            / (this->parentSensor->LaserShape()->GetSampleCount()-1);
    this->verticalAngleStep = (this->parentSensor->VerticalAngleMax() - this->parentSensor->VerticalAngleMin()).Radian()
            / (this->parentSensor->LaserShape()->GetVerticalSampleCount()-1);
    printf("StepH : %f   StepV: %f\n", this->horizontalAngleStep, this->verticalAngleStep);

    this->models = {};
    this->modelHits = 0;
};

void RayPlugin::OnNewLaserScans() {
    this->parentSensor->Ranges(ranges);
    this->position = this->world->ModelByName(this->name)->WorldPose().Rot().Euler();

    this->angleH = this->parentSensor->AngleMin().Radian();
    this->angleV = this->parentSensor->VerticalAngleMax().Radian();

    //printf("RollX: %f    RollY: %f    RollZ: %f\n", this->position.X(), this->position.Y(), this->position.Z());

    if (this->ranges.size() <= 0) {
        return;
    }
    for(int i = 0; i < this->ranges.size(); i++) {
        if (this->ranges.at(i) > this->parentSensor->RangeMax()) {
            continue;
        }

        this->rayTravelDist = this->parentSensor->RangeMin();
        while (this->rayTravelDist <= this->parentSensor->RangeMax()) {
            this->rayShape = this->parentSensor->LaserShape()->Ray(i);
            double temp = 0.0;
            this->rayShape->GetIntersection(temp, this->hitModelName);
            
            if (temp > this->parentSensor->RangeMax()) {
                break;
            }

            std::size_t seperator;
            seperator = this->hitModelName.find("::");
            this->hitModelName = this->hitModelName.substr(0, seperator);
            this->radarDetection = this->world->ModelByName(this->hitModelName);

            this->modelHits = this->modelHits+1;
            this->models.push_back(this->hitModelName);
            //printf("Found: %d\n\n", i);
            this->models.unique();

            printf("ModelName: %s\n", this->hitModelName.c_str());

            this->rayShape->RelativePoints(this->rayStart, this->rayEnd);

            //printf("Start x: %f  y: %f  z: %f\n", this->rayStart.X(), this->rayStart.Y(), this->rayStart.Z());
            //printf("End x: %f  y: %f  z: %f\n", this->rayEnd.X(), this->rayEnd.Y(), this->rayEnd.Z());

            //calculates gradient of current ray
            this->rayGradient = this->rayEnd - this->rayStart;
            this->rayGradient = this->rayGradient.Normalize();

            //printf("Gradient x: %f  y: %f  z: %f\n", this->rayGradient.X(), this->rayGradient.Y(), this->rayGradient.Z());

            this->rayTravelDist = this->rayTravelDist + temp + 0.001;

            printf("Dist: %f\n", this->rayTravelDist);

            this->rayShape->SetPoints(this->rayGradient * this->rayTravelDist, this->rayGradient * this->parentSensor->RangeMax());
        }
        //resets ray
        this->rayShape->SetPoints(this->rayGradient * this->parentSensor->RangeMin(), this->rayGradient * this->parentSensor->RangeMax());
    }
    this->models.sort();
    this->models.unique();
    int listSize = this->models.size();
    for(int i = 0; i < listSize; i++){
        printf("FoundModel: %s    ListSize: %d\n", this->models.front().c_str(), this->models.size());
        this->models.pop_front();
    }
    printf("ModelH: %d\n\n", this->modelHits);
    this->modelHits = 0;

    this->models.clear();

}
