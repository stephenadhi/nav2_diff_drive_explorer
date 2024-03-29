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

#ifndef NAV2_DIFF_DRIVE_EXPLORER__NAIVE_EXPLORER_HPP_
#define NAV2_DIFF_DRIVE_EXPLORER__NAIVE_EXPLORER_HPP_

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

#define CENTER 0
#define LEFT   1
#define RIGHT  2

#define GET_ROBOT_DIRECTION 0
#define ROBOT_DRIVE_FORWARD 1
#define ROBOT_RIGHT_TURN    2
#define ROBOT_LEFT_TURN     3

class NaiveExplorer : public rclcpp::Node
{
public:
  NaiveExplorer();
  ~NaiveExplorer();

private:
  // ROS topic publishers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

  // ROS topic subscribers
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  
  // Variables
  double robot_pose_;
  double prev_robot_pose_;
  double scan_data_[3];
  double linear_vel;
  double angular_vel;
  double escape_range;
  double check_forward_dist;
  double check_side_dist;
  uint16_t check_forward_angle;
  uint16_t check_left_angle;
  uint16_t check_right_angle;
  
  std::string scan_topic;
  std::string odom_topic;
  std::string cmd_topic;


  // ROS timer
  rclcpp::TimerBase::SharedPtr update_timer_;

  // Function prototypes
  void update_callback();
  void update_cmd_vel(double linear, double angular);
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
};
#endif  // NAV2_DIFF_DRIVE_EXPLORER__NAIVE_EXPLORER_HPP_
