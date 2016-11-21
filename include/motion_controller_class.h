#ifndef MOTIONCONTROLLERPLUGIN_H_
#define MOTIONCONTROLLERPLUGIN_H_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

namespace gazebo
{
class GAZEBO_VISIBLE MotionControllerPlugin : public ModelPlugin {
    public:
        explicit MotionControllerPlugin();
        virtual ~MotionControllerPlugin();

        void Load(physics::ModelPtr parent, sdf::ElementPtr /*_sdf*/) override;

        void OnUpdate(const common::UpdateInfo & /*_info*/);


        // Pointer to the model
    private:

        physics::JointPtr GetJoint(const std::string& element_name);

        physics::ModelPtr       model_;
        sdf::ElementPtr         sdf_;

        event::ConnectionPtr    update_connection_;

        physics::JointPtr       center_link_joint_;
        physics::JointPtr       front_wheels_joints_[2];

};


// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(MotionControllerPlugin)

}

#endif  // MOTIONCONTROLLERPLUGIN_H_
