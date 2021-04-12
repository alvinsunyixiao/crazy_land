#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/String.h"

#include "types.h"

std::mutex mtx_state;

jackal_state_t jackal_state;
crazyflie_state_t crazyflie_state;

void JackalMeasurementHandler(const geometry_msgs::PoseStampedConstPtr& msg) {
  std::lock_guard<std::mutex> lock(mtx_state);
  jackal_state.rotation.angle() = 2 * atan2(msg->pose.orientation.z, msg->pose.orientation.w);
  jackal_state.position << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
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
        crazyflie_state.status = ON_VEHICLE;
        crazyflie_state.transition_time = ros::Time::now();
      }
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
    pose.t << radius_ * std::cos(2 * M_PI * t_sec / period_) + center_.x(),
              radius_ * std::sin(2 * M_PI * t_sec / period_) + center_.y(),
              0;

    const double theta = 2 * M_PI * t_sec / period_ + M_PI / 2;
    pose.R = Eigen::AngleAxisd(theta, Eigen::Vector3d::UnitZ());

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
  const Eigen::Vector3d jk_t_cf(jk_t_cf_arr[0], jk_t_cf_arr[1], jk_t_cf_arr[2]);
  const Eigen::Quaterniond jk_R_cf(jk_R_cf_arr[3], jk_R_cf_arr[0], jk_R_cf_arr[1], jk_R_cf_arr[2]);

  // subscribe to jackal measurement
  auto jk_sub = node.subscribe("/vrpn_client_node/" + jackal_name + "/pose", 10,
                               &JackalMeasurementHandler);
  auto cf_sub = node.subscribe("/vrpn_client_node/" + crazyflie_name + "/pose", 10,
                               &CrazyflieMeasurementHandler);
  auto cf_status_sub = node.subscribe("/crazy_land/crazyflie/status", 10,
                                      &CrazyflieStatusHandler);

  ros::AsyncSpinner spinner(2);
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
    const pose_3d_t cf_jk_pose = trajectory->GetWaypoint(t + ros::Duration(.13));
    const pose_3d_t cf_pose = {
      .t = cf_jk_pose.R * jk_t_cf + cf_jk_pose.t +
           Eigen::Vector3d::UnitZ() * (.15 + jackal_state.position.z()),
      .R = cf_jk_pose.R * jk_R_cf,
      .timestamp = cf_jk_pose.timestamp,
    };

    {
      std::lock_guard<std::mutex> lock(mtx_state);
      if (crazyflie_state.status == TAKING_OFF) {

        cf_msg.header.frame_id = "TAKEOFF";

      } else if (crazyflie_state.status == SYNCHRONIZING) {
        cf_msg.header.frame_id = "FLYTO";
      } else if (crazyflie_state.status == ON_VEHICLE) {
        cf_msg.header.frame_id = "SHUTDOWN";
      }

      if (crazyflie_state.status == INITIALIZED ||
          crazyflie_state.status == UNINITIALIZED) {
        jk_msg.header.frame_id = "STOP";
      } else {
        jk_msg.header.frame_id = "GOTO";
      }
    }

    jk_pose.FillPoseStampedMsg(&jk_msg);
    cf_pose.FillPoseStampedMsg(&cf_msg);

    jackal_ctrl.publish(jk_msg);
    cf_ctrl.publish(cf_msg);
    ctrl_rate.sleep();
  }

  spinner.stop();
}
