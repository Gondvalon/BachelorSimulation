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
        gzthrow("RayPlugin requires Ray Sensor as its parent.\n");

    this->world = physics::get_world(this->parentSensor->WorldName());


    this->newLaserScansConnection =
            this->parentSensor->LaserShape()->ConnectNewLaserScans(std::bind(&RayPlugin::OnNewLaserScans, this));

    this->models = {};
    this->modelHits = 0;
};

void RayPlugin::OnNewLaserScans() {
    printf("Pos X:%f  Y:%f  Z:%f\n", this->parentSensor->Pose().Pos().X(), this->parentSensor->Pose().Pos().Y(), this->parentSensor->Pose().Pos().Z());
    std::string hitModelName;

    this->parentSensor->Ranges(ranges);

    if (this->ranges.size() <= 0) {
        return;
    }
    for(int i = 0; i < this->ranges.size(); i++) {
        if (this->ranges.at(i) > this->parentSensor->RangeMax()) {
            continue;
        }

        this->rayTravelDist = this->parentSensor->RangeMin();
        while (this->rayTravelDist <= this->parentSensor->RangeMax()) {
            printf("i: %d\n", i);
            this->rayShape = this->parentSensor->LaserShape()->Ray(i);
            double temp = 0.0;
            this->rayShape->GetIntersection(temp, hitModelName);
            printf("NewStart X:%f  Y:%f  Z:%f\n", this->rayShape->Start().X(), this->rayShape->Start().Y(), this->rayShape->Start().Z());
            printf("NewStart X:%f  Y:%f  Z:%f\n", this->rayShape->End().X(), this->rayShape->End().Y(), this->rayShape->End().Z());

            this->rayShape->RelativePoints(this->rayStart, this->rayEnd);

            this->rayGradient = this->rayEnd - this->rayStart;
            this->rayGradient = this->rayGradient.Normalize();

            //if no object hit then temp will be 1000
            if (temp > this->parentSensor->RangeMax()) {
                printf("Temp:%f\n", temp);
                break;
            }

            this->rayShape->GlobalPoints(this->rayStart, this->rayEnd);

            std::size_t seperator;
            seperator = hitModelName.find("::");
            hitModelName = hitModelName.substr(0, seperator);
            this->radarDetection = this->world->ModelByName(hitModelName);

            if ( this->radarDetection->GetSDF()->HasElement("human:heartRate")) {
                printf("Is alive with heart: %s", this->radarDetection->GetSDF()->GetElement("human:heartRate")->GetValue()->GetAsString().c_str());
            }

            this->modelHits = this->modelHits+1;
            this->models.push_back(hitModelName);
            this->models.unique();

            printf("Start X:%f  Y:%f  Z:%f\n",this->rayStart.X(), this->rayStart.Y(), this->rayStart.Z());
            printf("End X:%f  Y:%f  Z:%f\n",this->rayEnd.X(), this->rayEnd.Y(), this->rayEnd.Z());
            printf("Dist:%f\n", temp);


            this->rayTravelDist = this->rayTravelDist + temp + 0.001;
            printf("TravelDist:%f\n", this->rayTravelDist);

            //this->rayShape->Update();
            //printf("CPtrName: %s", this->rayShape->collisionParent->GetName().c_str());
            printf("NRayS X:%f  Y:%f  Z:%f\n",(this->rayGradient * this->rayTravelDist).X(), (this->rayGradient * this->rayTravelDist).Y(), (this->rayGradient * this->rayTravelDist).Z());
            printf("NRayE X:%f  Y:%f  Z:%f\n",(this->rayGradient * this->parentSensor->RangeMax()).X(),(this->rayGradient * this->parentSensor->RangeMax()).Y(),(this->rayGradient * this->parentSensor->RangeMax()).Z());
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
