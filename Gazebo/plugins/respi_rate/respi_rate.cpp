#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <cmath>

namespace gazebo {
    class ModelBreathe : public ModelPlugin {

        public: void Load(physics::ModelPtr _parent, sdf::ElementPtr) {
            // Store the pointer to the model
            this->model = _parent;

            this->timer = common::Timer();
            this->timer.Start();

            // Listen t the update event. This event is broadcast every simuation iteration
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&ModelBreathe::OnUpdate, this));
        }

        //Called by the world update start event
        public:
            void OnUpdate() {
                //float factor = sin(scale * ((2*M_PI)/10));
                //this->time = this->t::Float();
                //check if only 1000 away from integer
                if (fmod(this->timer.GetElapsed().Double(), 1) < 0.001) {
                    factor = sin(1.5*this->timer.GetElapsed().Double()-3.1)+1;
                    scale = this->model->Scale();
                    scaFa = factor;
                    printf("Scale: %f\n", scale.X());
                    this->model->SetScale(ignition::math::Vector3d(factor, factor, factor), false);
                    //printf("Timer: %f    Factor: %f\n", this->timer.GetElapsed().Double(), factor);
                }

                //this->model->SetLinearVel(ignition::math::Vector3d(.3, 0, 0));
                //this->model->SetScale(ignition::math::Vector3d(factor, factor, factor), false);
            }

        //Pointer t model
        private: physics::ModelPtr model;

        private: double factor = 0;

        private: ignition::math::Vector3d scale;

        private: double scaFa;

        private: common::Timer timer;

        //Pointer to update event connection
        private: event::ConnectionPtr updateConnection;
    };

    //Register this plugn with the simulator
    GZ_REGISTER_MODEL_PLUGIN(ModelBreathe)
}