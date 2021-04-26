#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Joy.h"

#include "types.h"

std::mutex mtx_state;

jackal_state_t jackal_state;
crazyflie_state_t crazyflie_state;

void JackalMeasurementHandler(const geometry_msgs::PoseStampedConstPtr& msg) {
  std::lock_guard<std::mutex> lock(mtx_state);
  const SE3 world_T_curr = {
    .R = Eigen::Quaterniond(msg->pose.orientation.w,
                            msg->pose.orientation.x,
                            msg->pose.orientation.y,
                            msg->pose.orientation.z),
    .t = Eigen::Vector3d(msg->pose.position.x,
                         msg->pose.position.y,
                         msg->pose.position.z)
  };
  const SE3 prev_T_curr = jackal_state.pose.Inv() * world_T_curr;
  jackal_state.twist = jackal_state.twist * 9.0 / 10 + prev_T_curr.toTwist() / 1e-2 / 10;

  jackal_state.pose.t << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  jackal_state.pose.R.x() = msg->pose.orientation.x;
  jackal_state.pose.R.y() = msg->pose.orientation.y;
  jackal_state.pose.R.z() = msg->pose.orientation.z;
  jackal_state.pose.R.w() = msg->pose.orientation.w;

  jackal_state.timestamp = msg->header.stamp;
}

void CrazyflieMeasurementHandler(const geometry_msgs::PoseStampedConstPtr& msg) {
  std::lock_guard<std::mutex> lock(mtx_state);
  crazyflie_state.position << msg->pose.position.x,
                              msg->pose.position.y,
                              msg->pose.position.z;

  switch (crazyflie_state.status) {
    case INITIALIZED:
      if (msg->header.stamp - crazyflie_state.transition_time > ros::Duration(5.)) {
        crazyflie_state.status = TAKING_OFF;
        crazyflie_state.transition_time = ros::Time::now();
      }
      break;
    case TAKING_OFF:
      if (msg->header.stamp - crazyflie_state.transition_time > ros::Duration(5.)) {
        crazyflie_state.status = SYNCHRONIZING;
        crazyflie_state.transition_time = ros::Time::now();
      }
      break;
    case SYNCHRONIZING:
      if (msg->header.stamp - crazyflie_state.transition_time > ros::Duration(10.)) {
        crazyflie_state.status = DOCKING;
        crazyflie_state.transition_time = ros::Time::now();
      }
      break;
    case DOCKING:
      if (msg->header.stamp - crazyflie_state.transition_time > ros::Duration(5.)) {
        crazyflie_state.status = ON_VEHICLE;
        crazyflie_state.transition_time = ros::Time::now();
      }
      break;
    case ON_VEHICLE:
      if (msg->header.stamp - crazyflie_state.transition_time > ros::Duration(10.)) {
        crazyflie_state.status = LEAVING;
        crazyflie_state.transition_time = ros::Time::now();
      }
      break;
    case LEAVING:
      if (msg->header.stamp - crazyflie_state.transition_time > ros::Duration(5.)) {
        crazyflie_state.status = RETURNING;
        crazyflie_state.transition_time = ros::Time::now();
      }
      break;
    default:
      break;
  };
}

void CrazyflieStatusHandler(const std_msgs::StringConstPtr& msg) {
  if (msg->data == "Initialized" && crazyflie_state.status == UNINITIALIZED) {
    std::lock_guard<std::mutex> lock(mtx_state);
    crazyflie_state.status = INITIALIZED;
    crazyflie_state.transition_time = ros::Time::now();
    ROS_INFO("Crazyflie: UNINITIALIZED -> INITIALIZED");
  }
}

void JoystickHandler(const sensor_msgs::JoyConstPtr& msg) {
  if (msg->buttons[8]) {
    std::lock_guard<std::mutex> lock(mtx_state);
    if (crazyflie_state.status == RETURNING &&
        ros::Time::now() - crazyflie_state.transition_time > ros::Duration(15.)) {
      ROS_INFO("Crazyflie: RETURNING -> TAKING OFF");
      crazyflie_state.status = TAKING_OFF;
      crazyflie_state.transition_time = ros::Time::now();
    }
  }
}

class ParametricTraj {
 public:
  virtual pose_3d_t GetWaypoint(const ros::Time& t) const = 0;
};

class CircularTraj : public ParametricTraj {
 public:
  CircularTraj(const Eigen::Vector2d& center, const double& radius, const double period)
    : center_(center), radius_(radius), period_(period) {}

  pose_3d_t GetWaypoint(const ros::Time& t) const override {
    pose_3d_t pose;
    const double t_sec = t.toSec();

    pose.timestamp = t;
    pose.T.t << radius_ * std::cos(2 * M_PI * t_sec / period_) + center_.x(),
                radius_ * std::sin(2 * M_PI * t_sec / period_) + center_.y(),
                0;

    const double theta = 2 * M_PI * t_sec / period_ + M_PI / 2;
    pose.T.R = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ());

    return pose;
  }

 private:
  const Eigen::Vector2d center_;
  const double radius_;
  const double period_;
};

std::unique_ptr<ParametricTraj> MakeTrajectory() {
  ros::NodeHandle pnode("~");
  std::string trajectory_type;
  pnode.getParam("type", trajectory_type);

  if (trajectory_type == "circular") {
    Eigen::Vector2d center;
    double radius, period;

    pnode.getParam("traj/center_x", center.x());
    pnode.getParam("traj/center_y", center.y());
    pnode.getParam("traj/radius", radius);
    pnode.getParam("traj/period", period);

    return std::make_unique<CircularTraj>(center, radius, period);
  }

  ROS_ERROR("Trajectory type %s unsupported", trajectory_type.c_str());

  return nullptr;
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "jackal_trajectory");

  ros::NodeHandle node;

  // parse parameters
  std::string jackal_name, crazyflie_name;
  std::vector<double> jk_t_cf_arr;
  std::vector<double> jk_R_cf_arr;
  node.getParam("/crazy_params/jackal_name", jackal_name);
  node.getParam("/crazy_params/crazyflie_name", crazyflie_name);
  node.getParam("/crazy_params/jk_t_cf", jk_t_cf_arr);
  node.getParam("/crazy_params/jk_R_cf", jk_R_cf_arr);
  const SE3 jk_T_cf = {
    .R = Eigen::Quaterniond(jk_R_cf_arr[3], jk_R_cf_arr[0], jk_R_cf_arr[1], jk_R_cf_arr[2]),
    .t = Eigen::Vector3d(jk_t_cf_arr[0], jk_t_cf_arr[1], jk_t_cf_arr[2]),
  };

  // subscribe to jackal measurement
  auto jk_sub = node.subscribe("/vrpn_client_node/" + jackal_name + "/pose", 10,
                               &JackalMeasurementHandler);
  auto cf_sub = node.subscribe("/vrpn_client_node/" + crazyflie_name + "/pose", 10,
                               &CrazyflieMeasurementHandler);
  auto cf_status_sub = node.subscribe("/crazy_land/crazyflie/status", 10,
                                      &CrazyflieStatusHandler);
  auto joy_sub = node.subscribe("/bluetooth_teleop/joy", 10, &JoystickHandler);

  ros::AsyncSpinner spinner(3);
  spinner.start();

  // publishable topic
  auto jackal_ctrl = node.advertise<geometry_msgs::PoseStamped>("/crazy_land/jackal/ctrl", 10);
  auto cf_ctrl = node.advertise<geometry_msgs::PoseStamped>("/crazy_land/crazyflie/ctrl", 10);

  // create trajectory object
  const auto trajectory = MakeTrajectory();

  // trajectory loop
  ros::Duration(3).sleep();
  ros::Rate ctrl_rate(50);
  while (ros::ok()) {
    // compute waypoint from trajectory
    geometry_msgs::PoseStamped jk_msg, cf_msg;
    jk_msg.header.frame_id = cf_msg.header.frame_id = "NO_OP";
    const auto t = ros::Time::now();
    const pose_3d_t jk_pose = trajectory->GetWaypoint(t);

    pose_3d_t cf_pose;

    {
      std::lock_guard<std::mutex> lock(mtx_state);

      const double t_comp = .6 + (ros::Time::now() - jackal_state.timestamp).toSec();
      const SE3 curr_T_future = SE3::fromTwist(jackal_state.twist * t_comp);
      const SE3 world_T_future = jackal_state.pose * curr_T_future;

      cf_pose.T = world_T_future * jk_T_cf;
      cf_pose.timestamp = jackal_state.timestamp + ros::Duration(t_comp);

      const double dt = (ros::Time::now() - crazyflie_state.transition_time).toSec();
      if (crazyflie_state.status == TAKING_OFF) {
        cf_msg.header.frame_id = "TAKEOFF";
        cf_pose.T.t.z() += .5;
      } else if (crazyflie_state.status == SYNCHRONIZING) {
        cf_msg.header.frame_id = "FLYTO";
        cf_pose.T.t.z() += .5;
      } else if (crazyflie_state.status == DOCKING) {
        cf_msg.header.frame_id = "FLYTO";
        cf_pose.T.t.z() += std::max(.5 * (5 - dt) / 5., 0.15);
      } else if (crazyflie_state.status == ON_VEHICLE) {
        cf_msg.header.frame_id = "SHUTDOWN";
      } else if (crazyflie_state.status == LEAVING) {
        cf_msg.header.frame_id = "FLYTO";
        cf_pose.T.t.z() += std::max(.5 * dt / 5., 0.15);
      } else if (crazyflie_state.status == RETURNING) {
        cf_msg.header.frame_id = "RETURN";
        cf_pose.T.t.x() = -1;
        cf_pose.T.t.y() = 1;
        cf_pose.T.t.z() = 1;
      }

      if (crazyflie_state.status == INITIALIZED ||
          crazyflie_state.status == UNINITIALIZED) {
        jk_msg.header.frame_id = "STOP";
      } else {
        jk_msg.header.frame_id = "GOTO";
      }
      //jk_msg.header.frame_id = "GOTO";
    }

    jk_pose.FillPoseStampedMsg(&jk_msg);
    cf_pose.FillPoseStampedMsg(&cf_msg);

    jackal_ctrl.publish(jk_msg);
    cf_ctrl.publish(cf_msg);
    ctrl_rate.sleep();
  }

  spinner.stop();
}
