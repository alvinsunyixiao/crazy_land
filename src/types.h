#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

struct jackal_state_t {
  Eigen::Rotation2Dd rotation;
  Eigen::Vector3d position;
};

