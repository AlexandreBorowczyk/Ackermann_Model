/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <fstream>
#include <iostream>

#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <gazebo_f150/landingConfig.h>
#include <dynamic_reconfigure/server.h>
#include <tf/transform_datatypes.h>
#include <cmath>

Eigen::Vector3d desired_position(0.0,0.0,0.0);
ros::Publisher trajectory_pub;
ros::NodeHandle * nh;

ros::Time prev_iter;

double heigth(0.0);
double landing_offset(1.5);
double descent_rate(0.0);

// dynamic_reconfigure
void config_callback(gazebo_f150::landingConfig &config, uint32_t level) {
  heigth          = config.heigth;
  landing_offset  = config.landing_offset;
  descent_rate    = config.descent_rate;
}

// Subscriber
void PositionCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  /*
    ROS_INFO("Received [%f, %f, %f].",
           msg->pose.pose.position.x,
           msg->pose.pose.position.y,
           msg->pose.pose.position.z);
  */
  tf::Quaternion q(msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z,msg->pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, desired_yaw;
  m.getRPY(roll, pitch, desired_yaw);

  ros::Time now = ros::Time::now();

  desired_position(0) = msg->pose.pose.position.x - landing_offset * cos(desired_yaw);
  desired_position(1) = msg->pose.pose.position.y - landing_offset * sin(desired_yaw);

  if (descent_rate > 0)
  {

    heigth -= descent_rate * 0.001 * (now-prev_iter).toSec();

    if (heigth < 0)
    {
      heigth = 0;
      descent_rate = 0;
    }
  }



  desired_position(2) = msg->pose.pose.position.z + heigth;



  const float DEG_2_RAD = M_PI / 180.0;

  trajectory_msgs::MultiDOFJointTrajectory trajectory_msg;
  trajectory_msg.header.stamp = ros::Time::now();

  mav_msgs::msgMultiDofJointTrajectoryFromPositionYaw(desired_position,
      desired_yaw, &trajectory_msg);

  trajectory_msg.points[0].velocities[0].linear.x = msg->twist.twist.linear.x;
  trajectory_msg.points[0].velocities[0].linear.y = msg->twist.twist.linear.y;
  trajectory_msg.points[0].velocities[0].linear.z = msg->twist.twist.linear.z;

/*
  ROS_INFO("Publishing waypoint on namespace %s: [%f, %f, %f].",
           nh->getNamespace().c_str(),
           desired_position.x(),
           desired_position.y(),
           desired_position.z());
*/
  trajectory_pub.publish(trajectory_msg);
}


int main(int argc, char** argv) {

  ros::init(argc, argv, "f150_follower");
  nh = new ros::NodeHandle("");

  prev_iter = ros::Time::now();

  std::string ns;
  ros::V_string args;

  ros::removeROSArgs(argc, argv, args);

  if (args.size() == 1) {
    ns = ros::this_node::getNamespace();
  }
  else if (args.size() == 2) {
    ns = args.at(2);
  }
  else{
    ROS_ERROR("Usage: f150_follower <namespace>\n");
    return -1;
  }



  trajectory_pub =
      nh->advertise<trajectory_msgs::MultiDOFJointTrajectory>(
      ns+"/command/trajectory", 10);




  ros::Subscriber sub_ = nh->subscribe("/f150", 1, PositionCallback);

//dynamic_reconfigure start
  dynamic_reconfigure::Server<gazebo_f150::landingConfig> server;
  dynamic_reconfigure::Server<gazebo_f150::landingConfig>::CallbackType f;

  f = boost::bind(&config_callback, _1, _2);
  server.setCallback(f);
//dynamic_reconfigure start

  ROS_INFO("Started f150_follower.");

  ros::spin();

  return 0;
}
