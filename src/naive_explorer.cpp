// Copyright 2019 ROBOTIS CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Adapted from turtlebot3_drive.cpp (Authors: Taehun Lim (Darby), Ryan Shim)

#include "nav2_diff_drive_explorer/naive_explorer.hpp"

#include <memory>

using namespace std::chrono_literals;

NaiveExplorer::NaiveExplorer()
: Node("naive_explorer_node")
{
  /************************************************************
  ** Initialise variables
  ************************************************************/
  scan_data_[0] = 0.0;
  scan_data_[1] = 0.0;
  scan_data_[2] = 0.0;

  robot_pose_ = 0.0;
  prev_robot_pose_ = 0.0;
  
  this->declare_parameter("linear_vel", 0.3);
  this->declare_parameter("angular_vel", 1.5);
  this->declare_parameter("escape_range", 30.0 * DEG2RAD);
  this->declare_parameter("check_forward_dist", 0.7);
  this->declare_parameter("check_side_dist", 0.6);
  this->declare_parameter("check_forward_angle", 0);
  this->declare_parameter("check_left_angle", 30);
  this->declare_parameter("check_right_angle", 330);   
  this->declare_parameter("scan_topic", "scan");
  this->declare_parameter("odom_topic", "odom");
  this->declare_parameter("cmd_topic", "cmd_vel");

  linear_vel = this->get_parameter("linear_vel").as_double();
  angular_vel = this->get_parameter("angular_vel").as_double();
  escape_range = this->get_parameter("escape_range").as_double();
  check_forward_dist = this->get_parameter("check_forward_dist").as_double();
  check_side_dist = this->get_parameter("check_side_dist").as_double();
  check_forward_angle = this->get_parameter("check_forward_angle").as_int();
  check_left_angle = this->get_parameter("check_left_angle").as_int();
  check_right_angle = this->get_parameter("check_right_angle").as_int();
  scan_topic = this->get_parameter("scan_topic").as_string();
  odom_topic = this->get_parameter("odom_topic").as_string();
  cmd_topic = this->get_parameter("cmd_topic").as_string();
  
  /************************************************************
  ** Initialise ROS publishers and subscribers
  ************************************************************/
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

  // Initialise publishers
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_topic, qos);

  // Initialise subscribers
  scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    scan_topic, \
    rclcpp::SensorDataQoS(), \
    std::bind(
      &NaiveExplorer::scan_callback, \
      this, \
      std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    odom_topic, qos, std::bind(&NaiveExplorer::odom_callback, this, std::placeholders::_1));

  /************************************************************
  ** Initialise ROS timers
  ************************************************************/
  update_timer_ = this->create_wall_timer(10ms, std::bind(&NaiveExplorer::update_callback, this));

  RCLCPP_INFO(this->get_logger(), "Naive explorer node has been initialised");
}

NaiveExplorer::~NaiveExplorer()
{
  RCLCPP_INFO(this->get_logger(), "Naive explorer node has been terminated");
}

/********************************************************************************
** Callback functions for ROS subscribers
********************************************************************************/
void NaiveExplorer::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  tf2::Quaternion q(
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z,
    msg->pose.pose.orientation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  robot_pose_ = yaw;
}

void NaiveExplorer::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  uint16_t scan_angle[3] = {check_forward_angle, check_left_angle, check_right_angle};

  for (int num = 0; num < 3; num++) {
    if (std::isinf(msg->ranges.at(scan_angle[num]))) {
      scan_data_[num] = msg->range_max;
    } else {
      // get pointcloud distance at corresponding check angle
      scan_data_[num] = msg->ranges.at(scan_angle[num]);
    }
  }
}

void NaiveExplorer::update_cmd_vel(double linear, double angular)
{
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.x = linear;
  cmd_vel.angular.z = angular;

  cmd_vel_pub_->publish(cmd_vel);
}

/********************************************************************************
** Update functions
********************************************************************************/
void NaiveExplorer::update_callback()
{
  static uint8_t robot_state_num = 0;

  switch (robot_state_num) {
    case GET_ROBOT_DIRECTION:
      if (scan_data_[CENTER] > check_forward_dist) {
        if (scan_data_[LEFT] < check_side_dist) {
          prev_robot_pose_ = robot_pose_;
          robot_state_num = ROBOT_RIGHT_TURN;
        } else if (scan_data_[RIGHT] < check_side_dist) {
          prev_robot_pose_ = robot_pose_;
          robot_state_num = ROBOT_LEFT_TURN;
        } else {
          robot_state_num = ROBOT_DRIVE_FORWARD;
        }
      }

      if (scan_data_[CENTER] < check_forward_dist) {
        prev_robot_pose_ = robot_pose_;
        robot_state_num = ROBOT_RIGHT_TURN;
      }
      break;

    case ROBOT_DRIVE_FORWARD:
      update_cmd_vel(linear_vel, 0.0);
      robot_state_num = GET_ROBOT_DIRECTION;
      break;

    case ROBOT_RIGHT_TURN:
      if (fabs(prev_robot_pose_ - robot_pose_) >= escape_range) {
        robot_state_num = GET_ROBOT_DIRECTION;
      } else {
        update_cmd_vel(0.0, -1 * angular_vel);
      }
      break;

    case ROBOT_LEFT_TURN:
      if (fabs(prev_robot_pose_ - robot_pose_) >= escape_range) {
        robot_state_num = GET_ROBOT_DIRECTION;
      } else {
        update_cmd_vel(0.0, angular_vel);
      }
      break;

    default:
      robot_state_num = GET_ROBOT_DIRECTION;
      break;
  }
}

/*******************************************************************************
** Main
*******************************************************************************/
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NaiveExplorer>());
  rclcpp::shutdown();

  return 0;
}
