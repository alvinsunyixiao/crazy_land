#pragma once

#include "ros/ros.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "geometry_msgs/PoseStamped.h"

struct pose_3d_t {
  Eigen::Vector3d t;
  Eigen::Quaterniond R;
  ros::Time timestamp;

  void FillPoseStampedMsg(geometry_msgs::PoseStamped* msg) const {
    msg->pose.position.x = t.x();
    msg->pose.position.y = t.y();
    msg->pose.position.z = t.z();

    msg->pose.orientation.x = R.x();
    msg->pose.orientation.y = R.y();
    msg->pose.orientation.z = R.z();
    msg->pose.orientation.w = R.w();
  }
};

enum robot_status_t {
  // shared status
  UNINITIALIZED = 0,
  INITIALIZED,

  // crazyflie status
  TAKING_OFF,
  SYNCHRONIZING,
  DOCKING,
  ON_VEHICLE,
  LEAVING,
  RETURNING,

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
