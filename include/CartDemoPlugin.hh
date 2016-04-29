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
#ifndef _GAZEBO_CART_DEMO_PLUGIN_HH_
#define _GAZEBO_CART_DEMO_PLUGIN_HH_

// Gazebo
#include "gazebo/common/Plugin.hh"
#include "gazebo/physics/physics.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/util/system.hh"


// Ros
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

#include <boost/thread.hpp>
#include <boost/bind.hpp>

#include <cmath>

#define NUM_JOINTS 2
#define cDEG2RAD 0.0174533
#define cRAD2DEG 57.2958

namespace gazebo
{
  /// \brief This plugin drives a four wheeled cart model forward and back
  /// by applying a small wheel torque.  Steering is controlled via
  /// a position pid.
  /// this is a test for general rolling contact stability.
  /// should refine the test to be more specific in the future.
  class GAZEBO_VISIBLE CartDemoPlugin : public ModelPlugin
  {
    public:
      CartDemoPlugin();
      virtual ~CartDemoPlugin();
      virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
      virtual void Init();



    private:

    void OnUpdate();

    ros::NodeHandle* rosnode_;
    ros::Subscriber sub_;

    boost::mutex lock;

    std::string robotNamespace;
    std::string topicName;

    // Custom Callback Queue
    ros::CallbackQueue queue_;
    boost::thread callback_queue_thread_;
    void QueueThread();

    // Ackermann stuff
    void CmdCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg);

    /////////////////////////

    transport::NodePtr node;

    event::ConnectionPtr updateConnection;

    physics::ModelPtr model;

    physics::JointPtr joints[NUM_JOINTS];
    common::PID jointPIDs[NUM_JOINTS*2];
    double jointPositions[NUM_JOINTS];
    double jointVelocities[NUM_JOINTS];

    double right_steering_target;
    double left_steering_target;
    double speed_target;

    double max_steering; // Max steering angle
    double wheelbase; // Max steering angle
    double wheelspread; // Max steering angle

    common::Time prevUpdateTime;
  };
}
#endif
