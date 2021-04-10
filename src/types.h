#pragma once

#include "ros/ros.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>

struct jackal_state_t {
  Eigen::Rotation2Dd rotation;
  Eigen::Vector3d position;
};

enum crazyflie_status_t {
  UNINITIALIZED = 0,
  INITIALIZED,
  TAKING_OFF,
  SYNCHRONIZING,
  DOCKING,
};

struct crazyflie_state_t {
  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;
  crazyflie_status_t status = UNINITIALIZED;
  ros::Time transition_time;
};
