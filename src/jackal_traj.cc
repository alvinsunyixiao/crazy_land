#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose2D.h"

#include "types.h"

std::mutex mtx_state;
std::condition_variable cv_state;
jackal_state_t jackal_state;
bool updated = false;

void MeasurementHandler(const geometry_msgs::PoseStampedConstPtr& msg) {
  std::unique_lock<std::mutex> lock(mtx_state);
  jackal_state.rotation.angle() = 2 * atan2(msg->pose.orientation.z, msg->pose.orientation.w);
  jackal_state.position << msg->pose.position.x, msg->pose.position.y;
  updated = true;

  lock.unlock();
  cv_state.notify_one();
}

class ParametricTraj {
 public:
  virtual geometry_msgs::Pose2D GetWaypoint(const double& time) const = 0;
};

class CircularTraj : public ParametricTraj {
 public:
  CircularTraj(const Eigen::Vector2d& center, const double& radius, const double period)
    : center_(center), radius_(radius), period_(period) {}

  geometry_msgs::Pose2D GetWaypoint(const double& time) const override {
    geometry_msgs::Pose2D msg;
    msg.x = radius_ * std::cos(2 * M_PI * time / period_) + center_.x();
    msg.y = radius_ * std::sin(2 * M_PI * time / period_) + center_.y();
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
  auto pub = node.advertise<geometry_msgs::Pose2D>("/tracking/jackal", 10);

  // create trajectory object
  const auto trajectory = MakeTrajectory();

  // trajectory loop
  ros::Duration(3).sleep();
  auto start_time = ros::Time::now();
  while (ros::ok()) {
    std::unique_lock<std::mutex> lock(mtx_state);
    cv_state.wait(lock, [&]() {
      return updated;
    });
    updated = false;

    // compute waypoint from trajectory
    double t = (ros::Time::now() - start_time).toSec();
    const auto msg = trajectory->GetWaypoint(t);

    pub.publish(msg);
    ROS_INFO("Publishing target @ (%f %f)", msg.x, msg.y);
  }

  spinner.stop();
}
