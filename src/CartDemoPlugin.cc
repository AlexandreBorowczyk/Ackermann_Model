/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/


#include "gazebo/physics/physics.hh"
#include "gazebo/transport/transport.hh"
#include "CartDemoPlugin.hh"

using namespace gazebo;
GZ_REGISTER_MODEL_PLUGIN(CartDemoPlugin)

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

/////////////////////////////////////////////////
CartDemoPlugin::CartDemoPlugin() :
right_steering_target(0.0),
left_steering_target(0.0),
speed_target(0.0)
{
  for (int i = 0; i < NUM_JOINTS; i++)
  {
    this->jointPositions[i] = 0;
    this->jointVelocities[i] = 0;
    this->jointPIDs[i] = common::PID(500, 25, 50, 50, -50);
  }
  for (int i = NUM_JOINTS; i < (2 *NUM_JOINTS); i++)
  {
    this->jointPIDs[i] = common::PID(25, 0.5, 1, 10, -10);
  }
}

/////////////////////////////////////////////////
CartDemoPlugin::~CartDemoPlugin()
{
  std::cerr << "F-150 Killed\n";

  event::Events::DisconnectWorldUpdateBegin(updateConnection);

  rosnode_->shutdown();
  callback_queue_thread_.join();
  delete rosnode_;


}

/////////////////////////////////////////////////
void CartDemoPlugin::Load(physics::ModelPtr _model,
                           sdf::ElementPtr _sdf)
{
  std::cerr << "F-150 Spawned\n";
  //gzdbg << "Loading";
  this->model = _model;

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->model->GetWorld()->GetName());

  if (!_sdf->HasElement("robotNamespace"))
    robotNamespace.clear();
  else
    robotNamespace = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";

    if (!_sdf->HasElement("topicName"))
     topicName = "cmd_vel";
   else
     topicName = _sdf->GetElement("topicName")->Get<std::string>();

  // get all joints
  this->joints[0] = _model->GetJoint(
    _sdf->GetElement("steer_right")->Get<std::string>());

  this->joints[1] = _model->GetJoint(
    _sdf->GetElement("steer_left")->Get<std::string>());


    for (int i = 0; i < NUM_JOINTS; i++)
    {
      this->jointPIDs[i] = common::PID( _sdf->GetElement("steer_P")->Get<double>(),
                                        _sdf->GetElement("steer_I")->Get<double>(),
                                        _sdf->GetElement("steer_D")->Get<double>(),
                                        _sdf->GetElement("steer_LIM")->Get<double>(),
                                        -1.0 * _sdf->GetElement("steer_LIM")->Get<double>());
    }
    for (int i = NUM_JOINTS; i < (2 *NUM_JOINTS); i++)
    {
      this->jointPIDs[i] = common::PID( _sdf->GetElement("vel_P")->Get<double>(),
                                        _sdf->GetElement("vel_I")->Get<double>(),
                                        _sdf->GetElement("vel_D")->Get<double>(),
                                        _sdf->GetElement("vel_LIM")->Get<double>(),
                                        -1.0 * _sdf->GetElement("vel_LIM")->Get<double>());
    }

    max_steering  = std::max(std::min(_sdf->GetElement("max_steering")->Get<double>(),89.0),1.0);
    wheelbase     = _sdf->GetElement("wheelbase")->Get<double>();
    wheelspread   = _sdf->GetElement("wheelspread")->Get<double>();

  if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc,argv,"F150_plugin",ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
  }

  rosnode_ = new ros::NodeHandle(robotNamespace);

  ros::SubscribeOptions so =
      ros::SubscribeOptions::create<geometry_msgs::Twist>(topicName, 1,
                                                               boost::bind(&CartDemoPlugin::CmdCallback, this, _1),
                                                               ros::VoidPtr(), &queue_);
  sub_ = rosnode_->subscribe(so);


  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&CartDemoPlugin::OnUpdate, this));

}
/////////////////////////////////////////////////

void CartDemoPlugin::CmdCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg)
{
  double wturnRadius_target;
  double out_steering_target;
  double in_steering_target;
  float wAngularRate;
  double min_turnRadius;

  wAngularRate      = cmd_msg->angular.z;
  min_turnRadius   = (tan( (90.0 - max_steering) * cDEG2RAD) * wheelbase) + wheelspread;
  //std::cout << " min_turnRadius: [" << min_turnRadius << "]\n";

  if (fabs(wAngularRate) < 0.01)
  {
    right_steering_target = 0;
    left_steering_target  = 0;

  }
  else
  {
    wturnRadius_target = std::max( 1/fabs(wAngularRate) * 10, min_turnRadius);
    //std::cout << " TurnRadius: [" << wturnRadius_target << "]\n";
    out_steering_target   = (90 * cDEG2RAD) - atan2(wturnRadius_target,wheelbase) ;
    in_steering_target    = (90 * cDEG2RAD) - atan2((wturnRadius_target - wheelspread),wheelbase);

    if ( sgn(wAngularRate) > 0)
    {
      right_steering_target = in_steering_target;
      left_steering_target  = out_steering_target;
    }
    else
    {
      right_steering_target = -out_steering_target;
      left_steering_target  = -in_steering_target;
    }
  }



  speed_target       = cmd_msg->linear.x;

 //std::cout << " Right: [" << right_steering_target * cRAD2DEG << "] Left: [" << left_steering_target * cRAD2DEG << "]\n";
}

/////////////////////////////////////////////////
void CartDemoPlugin::Init()
{
  callback_queue_thread_ = boost::thread(boost::bind(&CartDemoPlugin::QueueThread, this));
  // physics::EntityPtr parent = boost::dynamic_pointer_cast<physics::Entity>(
  //   this->joints[0]->GetChild());
}

/////////////////////////////////////////////////
void CartDemoPlugin::OnUpdate()
{
  common::Time currTime = this->model->GetWorld()->GetSimTime();
  common::Time stepTime = currTime - this->prevUpdateTime;
  this->prevUpdateTime = currTime;



    // Steering Control

    double right_steering_curr = this->joints[0]->GetAngle(0).Radian();
    double right_steering_err = right_steering_curr - right_steering_target;
    double right_effort_cmd = this->jointPIDs[0].Update(right_steering_err, stepTime);
    this->joints[0]->SetForce(0, right_effort_cmd);


    double left_steering_curr = this->joints[1]->GetAngle(0).Radian();
    double left_steering_err = left_steering_curr - left_steering_target;
    double left_steering_effort_cmd = this->jointPIDs[1].Update(left_steering_err, stepTime);
    this->joints[1]->SetForce(0, left_steering_effort_cmd);


    // Speed Control

    double right_vel_target = -speed_target;
    double right_vel_curr = this->joints[0]->GetVelocity(1);
    double right_vel_err = right_vel_curr - right_vel_target;
    double right_vel_effort_cmd(0.0);
    right_vel_effort_cmd = this->jointPIDs[2].Update(right_vel_err, stepTime);
    this->joints[0]->SetForce(1, right_vel_effort_cmd);

    double left_vel_target = -speed_target;
    double left_vel_curr = this->joints[1]->GetVelocity(1);
    double left_vel_err = left_vel_curr - left_vel_target;
    double left_vel_effort_cmd(0.0);
    left_vel_effort_cmd = this->jointPIDs[3].Update(left_vel_err, stepTime);
    this->joints[1]->SetForce(1, left_vel_effort_cmd);

    ros::spinOnce();
}

void CartDemoPlugin::QueueThread()
{

  static const double timeout = 0.01;

  while (rosnode_->ok())
  {
    //    std::cout << "CALLING STUFF\n";

    queue_.callAvailable(ros::WallDuration(timeout));
  }

}
