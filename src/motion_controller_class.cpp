#include <motion_controller_class.h>

#include <boost/bind.hpp>

#include <stdio.h>

namespace gazebo
{

MotionControllerPlugin::MotionControllerPlugin() : ModelPlugin() {
  gzmsg << "MotionControllerPlugin Created.\n";
}

MotionControllerPlugin::~MotionControllerPlugin(){

  gzmsg << "MotionControllerPlugin Destroyed.\n";

}

void MotionControllerPlugin::Load(physics::ModelPtr parent, sdf::ElementPtr /*_sdf*/) {
  // Store the pointer to the model
  this->model_ = parent;


  // Fetch center link joint
  center_link_joint_ = parent->GetJoint("centerlink2chassis_joint");

  // Fetch wheel rotationary joints
  front_wheels_joints_[0] = parent->GetJoint("right_arm2wheel_joint");
  front_wheels_joints_[1] = parent->GetJoint("left_arm2wheel_joint");

  if ( nullptr == front_wheels_joints_[0] ||
       nullptr == front_wheels_joints_[1] ||
       nullptr == center_link_joint_) {

    gzwarn << "Missing necessary joints.\n";

  }

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
                               boost::bind(&MotionControllerPlugin::OnUpdate, this, _1));

  gzmsg << "MotionControllerPlugin Loaded.\n";
}

// Called by the world update start event

void MotionControllerPlugin::OnUpdate(const common::UpdateInfo & /*_info*/) {

  if( 0.13 > center_link_joint_->GetAngle(0).Radian()) {
    center_link_joint_->SetVelocity(0,10);
  }

  front_wheels_joints_[0]->SetVelocity(0,1.0);
  front_wheels_joints_[1]->SetVelocity(0,1.0);
}


}

