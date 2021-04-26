#pragma once

#include "ros/ros.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "geometry_msgs/PoseStamped.h"

struct SE3 {
  Eigen::Quaterniond R;
  Eigen::Vector3d t;

  SE3 Inv() const {
    return {
      .R = R.inverse(),
      .t = -R.inverse().toRotationMatrix() * t
    };
  }

  SE3 operator*(const SE3& others) const {
    return {
      .R = R * others.R,
      .t = R * others.t + t,
    };
  }

  Eigen::Matrix<double, 6, 1> toTwist() const {
    Eigen::Matrix<double, 6, 1> twist;
    const Eigen::AngleAxisd R_axis_angle(R);
    const Eigen::Matrix3d R_mat = R.toRotationMatrix();
    const double theta = R_axis_angle.angle();

    if (theta < 1e-9) {
      twist.tail<3>().setZero();
      twist.head<3>() = t;
      return twist;
    }

    const Eigen::Matrix3d ln_R = theta / (2 * std::sin(theta)) * (R_mat - R_mat.transpose());

    // v
    const Eigen::Matrix3d V = Eigen::Matrix3d::Identity() +
                              (1 - std::cos(theta)) * ln_R / (theta * theta) +
                              (theta - std::sin(theta)) * ln_R * ln_R / (theta * theta * theta);
    twist.head<3>() = V.inverse() * t;

    // omega
    twist.tail<3>() = R_axis_angle.axis() * theta;

    return twist;
  }

  static SE3 fromTwist(const Eigen::Matrix<double, 6, 1>& twist) {
    const double theta = twist.tail<3>().norm();

    if (theta < 1e-9) {
      return {
        .R = Eigen::Quaterniond::Identity(),
        .t = twist.head<3>()
      };
    }

    const Eigen::AngleAxisd R_axis_angle(theta, twist.tail<3>() / theta);
    const Eigen::Quaterniond R(R_axis_angle);
    const Eigen::Matrix3d R_mat = R.toRotationMatrix();
    const Eigen::Matrix3d ln_R = theta / (2 * std::sin(theta)) * (R_mat - R_mat.transpose());
    const Eigen::Matrix3d V = Eigen::Matrix3d::Identity() +
                              (1 - std::cos(theta)) * ln_R / (theta * theta) +
                              (theta - std::sin(theta)) * ln_R * ln_R / (theta * theta * theta);

    return {
      .R = R,
      .t = V * twist.head<3>(),
    };
  }
};

struct pose_3d_t {
  SE3 T;
  ros::Time timestamp;

  void FillPoseStampedMsg(geometry_msgs::PoseStamped* msg) const {
    msg->pose.position.x = T.t.x();
    msg->pose.position.y = T.t.y();
    msg->pose.position.z = T.t.z();

    msg->pose.orientation.x = T.R.x();
    msg->pose.orientation.y = T.R.y();
    msg->pose.orientation.z = T.R.z();
    msg->pose.orientation.w = T.R.w();
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
  SE3 pose;
  Eigen::Matrix<double, 6, 1> twist;
  ros::Time timestamp;
  robot_status_t status = UNINITIALIZED;
};

struct crazyflie_state_t {
  Eigen::Vector3d position;
  Eigen::Quaterniond orientation;
  robot_status_t status = UNINITIALIZED;
  ros::Time transition_time;
};
