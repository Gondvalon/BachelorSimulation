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

    this->sensor.reset();
    this->world.reset();
}

void RayPlugin::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) {
    this->sensor = std::dynamic_pointer_cast<sensors::RaySensor>(_sensor);

    if(!this->sensor)
        gzthrow("RayPlugin requires Ray Sensor as its parent.\n");

    this->world = physics::get_world(this->sensor->WorldName());

    //default Parameters
    this->penetrableWalls = 1000;
    this->signalStrength = 100.0;
    this->airDamping = 0.0;

    if (_sdf->HasElement("penetrableWalls")) {
        this->penetrableWalls = std::stoi(_sdf->GetElement("penetrableWalls")->GetValue()->GetAsString());
    }
    if (_sdf->HasElement("signalStrength")) {
        this->signalStrength = std::stod(_sdf->GetElement("signalStrength")->GetValue()->GetAsString());
    }
    if (_sdf->HasElement("airDamping")) {
        this->airDamping = std::stod(_sdf->GetElement("airDamping")->GetValue()->GetAsString());
    }

    this->newLaserScansConnection =
            this->sensor->LaserShape()->ConnectNewLaserScans(std::bind(&RayPlugin::OnNewLaserScans, this));

    this->models = {};
    this->modelHits = 0;
};

void RayPlugin::OnNewLaserScans() {
    //update MultiRayShape collision box position
    physics::CollisionPtr coll = boost::dynamic_pointer_cast<physics::Collision>(this->sensor->LaserShape()->GetParent());
    coll->SetWorldPoseDirty();

    std::string nameOfHitModel;
    this->sensor->Ranges(ranges);

    if (this->ranges.size() <= 0) {
        return;
    }
    for(int i = 0; i < this->ranges.size(); i++) {
        //update RayShape collision box position
        physics::CollisionPtr rayColl = boost::dynamic_pointer_cast<physics::Collision>(this->sensor->LaserShape()->Ray(i)->GetParent());
        rayColl->SetWorldPoseDirty();

        if (this->ranges.at(i) > this->sensor->RangeMax()) {
            continue;
        }

        //setup for each ray
        double rayTravelDist = this->sensor->RangeMin();
        int penetratedWalls = 0;
        std::string oldNameOfHitModel = "";
        double raySignalStrength = this->signalStrength;
        while (rayTravelDist <= this->sensor->RangeMax()) {
            this->wallDamping = this->airDamping;
            this->rayShape = this->sensor->LaserShape()->Ray(i);

            double sectionTilHit = 0.0;
            this->rayShape->GetIntersection(sectionTilHit, nameOfHitModel);

            this->rayShape->RelativePoints(this->rayStart, this->rayEnd);
            this->rayGradient = this->rayEnd - this->rayStart;
            this->rayGradient = this->rayGradient.Normalize();

            //if no object hit then sectionTilHit will be 1000
            if (sectionTilHit > this->sensor->RangeMax()) {
                break;
            }

            std::size_t seperator;
            seperator = nameOfHitModel.find("::");
            nameOfHitModel = nameOfHitModel.substr(0, seperator);
            this->modelHitByRay = this->world->ModelByName(nameOfHitModel);

            if (nameOfHitModel == oldNameOfHitModel) {
                penetratedWalls++;
                if (this->modelHitByRay->GetSDF()->HasElement("vitalRadar:damping")) {
                    this->wallDamping = std::stod(this->modelHitByRay->GetSDF()->GetElement("vitalRadar:damping")->GetValue()->GetAsString());
                }
                raySignalStrength = raySignalStrength - this->wallDamping * sectionTilHit * 100;
                printf("penetrated: %d ray:%d\n", penetratedWalls, i);
            } else {
                raySignalStrength = raySignalStrength - this->airDamping * sectionTilHit * 100;
            }

            if (penetratedWalls > this->penetrableWalls || raySignalStrength <= 0) {
                break;
            }

            //finding and calculating found vital signs
            double signalVariation = 1.0-raySignalStrength/this->signalStrength;
            if (this->modelHitByRay->GetSDF()->HasElement("human:heartRate") && !(nameOfHitModel == oldNameOfHitModel)) {
                this->heartRate = std::stod(this->modelHitByRay->GetSDF()->GetElement("human:heartRate")->GetValue()->GetAsString());
                double heartRateRange[] = {this->heartRate - (this->heartRate * signalVariation),
                                       this->heartRate + (this->heartRate * signalVariation)};
                printf("Is alive with heart: %s\n", this->modelHitByRay->GetSDF()->GetElement("human:heartRate")->GetValue()->GetAsString().c_str());
            }
            if (this->modelHitByRay->GetSDF()->HasElement("human:respiratoryRate") && !(nameOfHitModel == oldNameOfHitModel)) {
                this->respiratoryRate = std::stod(this->modelHitByRay->GetSDF()->GetElement("human:respiratoryRate")->GetValue()->GetAsString());
                double respiratoryRateRange[] = {this->respiratoryRate - (this->respiratoryRate * signalVariation),
                                             this->respiratoryRate + (this->respiratoryRate * signalVariation)};
            }

            this->modelHits = this->modelHits+1;
            this->models.push_back(nameOfHitModel);
            this->models.unique();


            rayTravelDist = rayTravelDist + sectionTilHit + 0.001;
            oldNameOfHitModel = nameOfHitModel;
            this->rayShape->SetPoints(this->rayGradient * rayTravelDist, this->rayGradient * this->sensor->RangeMax());
        }
        //resets ray
        this->rayShape->SetPoints(this->rayGradient * this->sensor->RangeMin(), this->rayGradient * this->sensor->RangeMax());
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
