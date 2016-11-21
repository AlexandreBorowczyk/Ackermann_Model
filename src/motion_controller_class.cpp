#include <motion_controller_class.h>

#include <boost/bind.hpp>

#include <stdio.h>

#include <sdf/Param.hh>

namespace gazebo
{

MotionControllerPlugin::MotionControllerPlugin() : ModelPlugin() {
  gzmsg << "MotionControllerPlugin: Created.\n";
}

MotionControllerPlugin::~MotionControllerPlugin(){

  gzmsg << "MotionControllerPlugin: Destroyed.\n";

}

void MotionControllerPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) {

  // Store the pointer to the model
  this->model_ = _parent;
  this->sdf_ = _sdf;



  // Get center link joint
  center_link_joint_ = GetJoint("center_link_joint");

  // Get right wheel rotationary joint
  front_wheels_joints_[0] = GetJoint("right_front_wheels_joint");

  // Get left wheel rotationary joint
  front_wheels_joints_[1] = GetJoint("left_front_wheels_joint");


  if(nullptr != center_link_joint_ &&
     nullptr != front_wheels_joints_[0] &&
     nullptr != front_wheels_joints_[1]) {

    // Listen to the update event. This event is broadcast every
    // simulation iteration.
    this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
                                 boost::bind(&MotionControllerPlugin::OnUpdate, this, _1));

    gzmsg << "MotionControllerPlugin: Loaded.\n";

  }
}

// Called by the world update start event

void MotionControllerPlugin::OnUpdate(const common::UpdateInfo & /*_info*/) {

  center_link_joint_->SetVelocity(0,10);

  front_wheels_joints_[0]->SetVelocity(0,2.0);
  front_wheels_joints_[1]->SetVelocity(0,2.0);
}

physics::JointPtr MotionControllerPlugin::GetJoint(const std::string& element_name) {

  physics::JointPtr joint_ptr = nullptr;

  if(nullptr != model_ &&
     nullptr != sdf_ ) {

    sdf::ElementPtr joint_name = sdf_->GetElement(element_name);
    if (nullptr == joint_name)
    {
      gzerr << "<" << element_name << ">" << " does not exist\n";

    } else {
      joint_ptr = model_->GetJoint(joint_name->GetValue()->GetAsString());
    }

  }

  return joint_ptr;

}


}

