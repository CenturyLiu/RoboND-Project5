// function adapted from the gazebo tutorial for the model_push plugin
// link: http://gazebosim.org/tutorials?tut=plugins_model&cat=write_plugin 

#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo
{
  class ModelPush : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // initialize the counter
      this->counter = 0;

      // Store the pointer to the model
      this->model = _parent;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&ModelPush::OnUpdate, this));

      
    }

    // Called by the world update start event
    public: void OnUpdate()
    {

      if(this->counter < 10000){
          // Apply a small linear velocity to the model.
          this->model->SetLinearVel(ignition::math::Vector3d(-0.1, 0, 0));
          }
      else if(this->counter >= 10000 && this->counter < 20000){
          // Apply a small linear velocity to the model.
          this->model->SetLinearVel(ignition::math::Vector3d(0, -0.1, 0));
           }
      else if(this->counter >= 20000 && this->counter < 30000){
          this->model->SetLinearVel(ignition::math::Vector3d( 0.1, 0, 0));
      }
      else{
          this->model->SetLinearVel(ignition::math::Vector3d(0, 0.1, 0));
      }
       this->counter = (this->counter + 1) % 40000;
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    // counter for control model direction
    private: int counter;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}
