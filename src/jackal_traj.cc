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
bool updated = false;

void MeasurementHandler(const geometry_msgs::PoseStampedConstPtr& msg) {
  std::unique_lock<std::mutex> lock(mtx_state);
  jackal_state.rotation.angle() = 2 * atan2(msg->pose.orientation.z, msg->pose.orientation.w);
  jackal_state.position << msg->pose.position.x, msg->pose.position.y;
  updated = true;

  lock.unlock();
  cv_state.notify_one();
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "jackal_trajectory");
  ros::NodeHandle pnode("~");

  // parse trajectory

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
    geometry_msgs::Pose2D msg;
    msg.x = std::cos(2 * M_PI * t / 12);
    msg.y = std::sin(2 * M_PI * t / 12);
    pub.publish(msg);

    ROS_INFO("Publishing target @ (%f %f)", msg.x, msg.y);
  }

  spinner.stop();
}
