#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <cmath>

namespace gazebo {
    class VitalRadar : public SensorPlugin {
    public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr) {
        this->sensor = _parent;

        this->updateConnection = event::Events::ConnectCreateSensor(std::bind(&VitalRadar::OnUpdate, this));
    }

    public:
        void OnUpdate() {
            printf("Works!\n");
    }

    private: sensors::SensorPtr sensor;

    private: event::ConnectionPtr updateConnection;
    };
    GZ_REGISTER_SENSOR_PLUGIN(VitalRadar)
}