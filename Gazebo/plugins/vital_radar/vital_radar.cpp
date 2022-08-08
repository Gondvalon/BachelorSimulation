#include <gazebo/gazebo.hh>
#include <gazebo/sensors/sensors.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <cmath>

namespace gazebo {
    class VitalRadar : public SensorPlugin {
        public: void Load(sensors::SensorPtr _parent, sdf::ElementPtr) {
            this->sensor = _parent;

            printf("%d", this->sensor.IsActive());

            this->updateConnection = event::Events::ConnectCreateSensor(std::bind(&VitalRadar::OnUpdate, this));
        }

        public:
            void OnUpdate() {
                printf("Works!\n");
        }

        private: sensors::RaySensorPtr sensor;

        private: event::ConnectionPtr updateConnection;

        private: sensors::RaySensor raySensor;
    };
    GZ_REGISTER_SENSOR_PLUGIN(VitalRadar)
}