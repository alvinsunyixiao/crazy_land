#include <condition_variable>
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

void MeasurementHandler(const geometry_msgs::PoseStampedConstPtr& msg) {
  std::unique_lock<std::mutex> lock(mtx_state);
  jackal_state.rotation.angle() = 2 * atan2(msg->pose.orientation.z, msg->pose.orientation.w);
  jackal_state.position << msg->pose.position.x, msg->pose.position.y;

  lock.unlock();
  cv_state.notify_one();
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "jackal_trajectory");
  ros::NodeHandle pnode("~");

  // parse trajectory
  std::vector<double> x_traj, y_traj;
  std::vector<geometry_msgs::Pose2D> traj;
  pnode.getParam("trajectory_x", x_traj);
  pnode.getParam("trajectory_y", y_traj);
  for (int i = 0; i < x_traj.size(); ++i) {
    geometry_msgs::Pose2D msg;
    msg.x = x_traj[i];
    msg.y = y_traj[i];
    traj.push_back(msg);
  }

  // subscribe to jackal measurement
  std::string jackal_name;
  pnode.param<std::string>("jackal_name", jackal_name, "alvin_jk");
  ros::NodeHandle node;
  auto sub = node.subscribe("/vrpn_client_node/" + jackal_name + "/pose", 10,
                            &MeasurementHandler);
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // trajectory loop
  auto pub = node.advertise<geometry_msgs::Pose2D>("/tracking/jackal", 10);
  size_t waypoint_idx = 0;
  while (ros::ok()) {
    ROS_INFO("publishing waypoint @ (%f %f)", traj[waypoint_idx].x, traj[waypoint_idx].y);
    pub.publish(traj[waypoint_idx]);
    const Eigen::Vector2d target_position(traj[waypoint_idx].x, traj[waypoint_idx].y);

    std::unique_lock<std::mutex> lock(mtx_state);
    cv_state.wait(lock, [&]() {
      return (jackal_state.position - target_position).norm() < 0.1;
    });
  }

  spinner.stop();
}
