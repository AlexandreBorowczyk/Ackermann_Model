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

  std::string parameter_name;
  sdf::ElementPtr joint_name;

  // Get center link joint
  parameter_name = "center_link_joint";
  joint_name = _sdf->GetElement(parameter_name);
  if (nullptr == joint_name)
  {
    gzerr << "<" << parameter_name << ">" << " does not exist\n";
    return;
  } else {
    center_link_joint_ = model_->GetJoint(joint_name->GetValue()->GetAsString());
  }

  // Get right wheel rotationary joint
  parameter_name = "right_front_wheels_joint";
  joint_name = _sdf->GetElement(parameter_name);
  if (nullptr == joint_name)
  {
    gzerr << "<" << parameter_name << ">" << " does not exist\n";
    return;
  } else {
    front_wheels_joints_[0] = model_->GetJoint(joint_name->GetValue()->GetAsString());
  }

  // Get left wheel rotationary joint
  parameter_name = "left_front_wheels_joint";
  joint_name = _sdf->GetElement(parameter_name);
  if (nullptr == joint_name)
  {
    gzerr << "<" << parameter_name << ">" << " does not exist\n";
    return;
  } else {
    front_wheels_joints_[1] = model_->GetJoint(joint_name->GetValue()->GetAsString());
  }

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
                               boost::bind(&MotionControllerPlugin::OnUpdate, this, _1));

  gzmsg << "MotionControllerPlugin: Loaded.\n";
}

// Called by the world update start event

void MotionControllerPlugin::OnUpdate(const common::UpdateInfo & /*_info*/) {

  if( 0.13 > center_link_joint_->GetAngle(0).Radian()) {
    center_link_joint_->SetVelocity(0,0.1);
  }

  front_wheels_joints_[0]->SetVelocity(0,2.0);
  front_wheels_joints_[1]->SetVelocity(0,2.0);
}


}

