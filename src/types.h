#pragma once

#include "ros/ros.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>

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
