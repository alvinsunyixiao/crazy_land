#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"

#include "types.h"

std::mutex mtx_state;
std::condition_variable cv_state;
jackal_state_t jackal_state;
bool updated = false;

void MeasurementHandler(const geometry_msgs::PoseStampedConstPtr& msg) {
  std::unique_lock<std::mutex> lock(mtx_state);
  jackal_state.rotation.angle() = 2 * atan2(msg->pose.orientation.z, msg->pose.orientation.w);
  jackal_state.position << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
  updated = true;

  lock.unlock();
  cv_state.notify_one();
}

class ParametricTraj {
 public:
  geometry_msgs::PoseStamped GetWaypointNow() const {
    return GetWaypoint(ros::Time::now());
  }

  virtual geometry_msgs::PoseStamped GetWaypoint(const ros::Time& t) const = 0;
};

class CircularTraj : public ParametricTraj {
 public:
  CircularTraj(const Eigen::Vector2d& center, const double& radius, const double period)
    : center_(center), radius_(radius), period_(period) {}

  geometry_msgs::PoseStamped GetWaypoint(const ros::Time& t) const override {
    geometry_msgs::PoseStamped msg;
    const double t_sec = t.toSec();

    msg.header.stamp = t;
    msg.pose.position.x = radius_ * std::cos(2 * M_PI * t_sec / period_) + center_.x();
    msg.pose.position.y = radius_ * std::sin(2 * M_PI * t_sec / period_) + center_.y();

    return msg;
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
  ros::NodeHandle pnode("~");

  // subscribe to jackal measurement
  std::string jackal_name;
  pnode.param<std::string>("jackal_name", jackal_name, "alvin_jk");
  ros::NodeHandle node;
  auto sub = node.subscribe("/vrpn_client_node/" + jackal_name + "/pose", 10,
                            &MeasurementHandler);
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // publishable topic
  auto jackal_ctrl = node.advertise<geometry_msgs::PoseStamped>("/crazy_land/jackal_ctrl", 10);
  auto cf_ctrl = node.advertise<geometry_msgs::PoseStamped>("/crazy_land/crazyflie_ctrl", 10);

  // create trajectory object
  const auto trajectory = MakeTrajectory();

  // trajectory loop
  ros::Duration(3).sleep();
  while (ros::ok()) {
    std::unique_lock<std::mutex> lock(mtx_state);
    cv_state.wait(lock, [&]() {
      return updated;
    });
    updated = false;

    // compute waypoint from trajectory
    const auto jk_msg = trajectory->GetWaypointNow();
    auto cf_msg = trajectory->GetWaypoint(ros::Time::now() + ros::Duration(0.2));
    cf_msg.pose.position.z = jackal_state.position.z() + 1.0;
    cf_msg.header.frame_id = "FLYTO";

    jackal_ctrl.publish(jk_msg);
    cf_ctrl.publish(cf_msg);
  }

  spinner.stop();
}
