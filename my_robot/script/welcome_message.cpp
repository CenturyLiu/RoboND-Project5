#include <gazebo/gazebo.hh>

namespace gazebo
{
  class WorldPluginMyRobot : public WorldPlugin{
      public: WorldPluginMyRobot() : WorldPlugin(){

           printf("Welcome to shiji’s World!\n");

       }
      public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf){}

   };

  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(WorldPluginMyRobot)


}
