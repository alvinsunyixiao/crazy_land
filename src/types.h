#pragma once

#include "ros/ros.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "geometry_msgs/PoseStamped.h"

struct pose_3d_t {
  Eigen::Vector3d position;
  Eigen::Quaternion3d orientation;
  double timestamp;

  void FillPoseStampedMsg(geometry_msgs::PoseStamped* msg) {
    msg->pose.position.x = position.x();
    msg->pose.position.y = position.y();
    msg->pose.position.z = position.z();

    msg->pose.orientation.x = orientation.x();
    msg->pose.orientation.y = orientation.y();
    msg->pose.orientation.z = orientation.z();
    msg->pose.orientation.w = orientation.w();
  }
}

enum robot_status_t {
  // shared status
  UNINITIALIZED = 0,
  INITIALIZED,

  // crazyflie status
  TAKING_OFF,
  SYNCHRONIZING,
  DOCKING,

  // jackal status
  RUNNING,
  STOPPED,
};

struct jackal_state_t {
  Eigen::Rotation2Dd rotation;
  Eigen::Vector3d position;
  robot_status_t status = UNINITIALIZED;
};

struct crazyflie_state_t {
  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;
  robot_status_t status = UNINITIALIZED;
  ros::Time transition_time;
};
