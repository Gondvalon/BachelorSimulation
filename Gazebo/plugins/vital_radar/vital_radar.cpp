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

GZ_REGISTER_SENSOR_PLUGIN(VitalRadar)

VitalRadar::VitalRadar() {}

VitalRadar::~VitalRadar() {
    this->newLaserScansConnection.reset();

    this->sensor.reset();
    this->world.reset();
}

void VitalRadar::Load(sensors::SensorPtr _sensor, sdf::ElementPtr _sdf) {
    this->sensor = std::dynamic_pointer_cast<sensors::RaySensor>(_sensor);

    if(!this->sensor)
        gzthrow("RayPlugin requires Ray Sensor as its parent.\n");

    this->world = physics::get_world(this->sensor->WorldName());

    //default Parameters
    this->namespace_.clear();
    this->topic_ = "vitalRadar";
    this->frame_id_ = "/vital_radar";
    this->penetrableObjects = 1000;
    this->signalStrength = 100.0;
    this->airDamping = 0.0;
    this->lowerDetectionBorder = 0.0;

    if (_sdf->HasElement("robotNamespace")) {
        this->namespace_ = _sdf->GetElement("robotNamespace")->GetValue()->GetAsString();
    }
    if (_sdf->HasElement("topic_")) {
        this->topic_ = _sdf->GetElement("topic_")->GetValue()->GetAsString();
    }
    if (_sdf->HasElement("topicName")) {
        topic_ = _sdf->GetElement("topicName")->GetValue()->GetAsString();
    }

    if (_sdf->HasElement("penetrableObjects")) {
        this->penetrableObjects = std::stoi(_sdf->GetElement("penetrableObjects")->GetValue()->GetAsString());
    }
    if (_sdf->HasElement("signalStrength")) {
        this->signalStrength = std::stod(_sdf->GetElement("signalStrength")->GetValue()->GetAsString());
    }
    if (_sdf->HasElement("airDamping")) {
        this->airDamping = std::stod(_sdf->GetElement("airDamping")->GetValue()->GetAsString());
    }
    if (_sdf->HasElement("lowerDetectionBorder")) {
        this->lowerDetectionBorder = std::stod(_sdf->GetElement("lowerDetectionBorder")->GetValue()->GetAsString());
    }

    this->newLaserScansConnection =
            this->sensor->LaserShape()->ConnectNewLaserScans(std::bind(&VitalRadar::OnNewLaserScans, this));

    this->models = {};
    this->modelHits = 0;
};

void VitalRadar::OnNewLaserScans() {
    //update MultiRayShape collision box position
    physics::CollisionPtr coll = boost::dynamic_pointer_cast<physics::Collision>(this->sensor->LaserShape()->GetParent());
    coll->SetWorldPoseDirty();

    std::string nameOfHitModel;
    this->sensor->Ranges(ranges);

    if (this->ranges.size() <= 0) {
        return;
    }

    struct human {
        std::string name;
        double distance;
        double signalStrength;
        ignition::math::Vector3d position;
    };
    std::vector<human> humanObjects;

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

            if (penetratedWalls > this->penetrableObjects || raySignalStrength <= 0) {
                break;
            }

            //finding and calculating found vital signs
            if (!(nameOfHitModel == oldNameOfHitModel) && (raySignalStrength/this->signalStrength) >= lowerDetectionBorder &&
                    (this->modelHitByRay->GetSDF()->HasElement("human:heartRate") || this->modelHitByRay->GetSDF()->HasElement("human:respiratoryRate"))) {
                /*double signalVariation = pow((1.0-raySignalStrength/this->signalStrength),3)*30;
                if (this->modelHitByRay->GetSDF()->HasElement("human:heartRate")) {
                    this->heartRate = std::stod(this->modelHitByRay->GetSDF()->GetElement("human:heartRate")->GetValue()->GetAsString());
                    double heartRateRange[] = {this->heartRate - (this->heartRate * signalVariation),
                                           this->heartRate + (this->heartRate * signalVariation)};
                    printf("Is alive with heart: %s\n", this->modelHitByRay->GetSDF()->GetElement("human:heartRate")->GetValue()->GetAsString().c_str());
                }
                if (this->modelHitByRay->GetSDF()->HasElement("human:respiratoryRate")) {
                    this->respiratoryRate = std::stod(this->modelHitByRay->GetSDF()->GetElement("human:respiratoryRate")->GetValue()->GetAsString());
                    double respiratoryRateRange[] = {this->respiratoryRate - (this->respiratoryRate * signalVariation),
                                                 this->respiratoryRate + (this->respiratoryRate * signalVariation)};
                }
                 */

                //humanObjects.push_back(make_tuple(nameOfHitModel, rayTravelDist+sectionTilHit+this->sensor->RangeMin(), raySignalStrength, this->rayEnd));
                humanObjects.push_back(human());
                humanObjects[humanObjects.size()-1].name = nameOfHitModel;
                humanObjects[humanObjects.size()-1].distance = rayTravelDist+sectionTilHit+this->sensor->RangeMin();
                humanObjects[humanObjects.size()-1].signalStrength = raySignalStrength;
                humanObjects[humanObjects.size()-1].position = this->rayEnd;
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

    //handling of found human objects
    std::sort(humanObjects.begin(), humanObjects.end(), [](human a, human b) {
        return a.name < b.name;
    });
    for (int i = 0; i<humanObjects.size(); i++) {
        printf("Name: %s  Distance: %f  Size:%ld\n", humanObjects[i].name.c_str(), humanObjects[i].distance, humanObjects.size());
    }
    /*for (int i = 0; i <humanObjects.size(); i++) {
        while (std::get<0>(humanObjects[i]) == std::get<0>(humanObjects[i+1])) {
            if (humanObjects.size() < 2 || (i+1)>= humanObjects.size()) {
                break;
            }
            if (std::get<1>(humanObjects[i]) <= std::get<1>(humanObjects[i+1])) {
                humanObjects.erase(humanObjects.begin()+i+1);
            } else if (std::get<0>(humanObjects[i]) == std::get<0>(humanObjects[i+1])){
                humanObjects.erase(humanObjects.begin()+i);
            }
        }
    }*/
    for (int i = 0; i < humanObjects.size(); i++) {
        while (humanObjects[i].name == humanObjects[i+1].name) {
            if (humanObjects.size() <2 || (i+1) >= humanObjects.size()) {
                break;
            }
            if (humanObjects[i].distance <= humanObjects[i+1].distance) {
                humanObjects.erase(humanObjects.begin()+i+1);
            } else if (humanObjects[i].name == humanObjects[i+1].name) {
                humanObjects.erase(humanObjects.begin()+i);
            }
        }
    }
    for (int i = 0; i<humanObjects.size(); i++) {
        printf("Name: %s  Distance: %f  Size:%ld\n", humanObjects[i].name.c_str(), humanObjects[i].distance, humanObjects.size());
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
