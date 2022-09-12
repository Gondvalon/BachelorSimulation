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
    //update MultiRayShape collision box position
    physics::CollisionPtr coll = boost::dynamic_pointer_cast<physics::Collision>(this->parentSensor->LaserShape()->GetParent());
    coll->SetWorldPoseDirty();

    std::string modelHitByRay;
    this->parentSensor->Ranges(ranges);

    if (this->ranges.size() <= 0) {
        return;
    }
    for(int i = 0; i < this->ranges.size(); i++) {
        //update RayShape collision box position
        physics::CollisionPtr rayColl = boost::dynamic_pointer_cast<physics::Collision>(this->parentSensor->LaserShape()->Ray(i)->GetParent());
        rayColl->SetWorldPoseDirty();

        if (this->ranges.at(i) > this->parentSensor->RangeMax()) {
            continue;
        }

        this->rayTravelDist = this->parentSensor->RangeMin();
        while (this->rayTravelDist <= this->parentSensor->RangeMax()) {
            this->rayShape = this->parentSensor->LaserShape()->Ray(i);

            double temp = 0.0;
            this->rayShape->GetIntersection(temp, modelHitByRay);

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
            seperator = modelHitByRay.find("::");
            modelHitByRay = modelHitByRay.substr(0, seperator);
            this->radarDetection = this->world->ModelByName(modelHitByRay);

            if ( this->radarDetection->GetSDF()->HasElement("human:heartRate")) {
                printf("Is alive with heart: %s", this->radarDetection->GetSDF()->GetElement("human:heartRate")->GetValue()->GetAsString().c_str());
            }

            this->modelHits = this->modelHits+1;
            this->models.push_back(modelHitByRay);
            this->models.unique();


            this->rayTravelDist = this->rayTravelDist + temp + 0.001;

            this->rayShape->SetPoints(this->rayGradient * this->rayTravelDist, this->rayGradient * this->parentSensor->RangeMax());
        }
        //resets ray
        this->rayShape->SetPoints(this->rayGradient * this->parentSensor->RangeMin(), this->rayGradient * this->parentSensor->RangeMax());
    }
    
    this->models.sort();
    this->models.unique();
    int listSize = this->models.size();
    for(int i = 0; i < listSize; i++){
        printf("FoundModel: %s    ListSize: %ld\n", this->models.front().c_str(), this->models.size());
        this->models.pop_front();
    }
    printf("ModelH: %d\n\n", this->modelHits);
    this->modelHits = 0;

    this->models.clear();

}
