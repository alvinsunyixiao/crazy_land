#include <cmath>
#include <memory>
#include <mutex>

#include "ros/ros.h"

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"

#include <Eigen/Geometry>

struct jackel_state_t {
  Eigen::Rotation2Dd rotation;
  Eigen::Vector2d position;
};

class JackalController {
 public:
  JackalController() : pnode_("~") {
    std::string jackal_name;
    pnode_.param<std::string>("jackal_name", jackal_name, "alvin_jk");
    pnode_.param<int>("dead_button", btn_dead_, 1);
    pnode_.param<int>("control_frequency", control_freq_, 100);
    pnode_.param<int>("axis_linear", axis_linear_, 1);
    pnode_.param<int>("axis_angular", axis_angular_, 0);
    pnode_.param<double>("scale_linear", scale_linear_, .4);
    pnode_.param<double>("scale_angular", scale_angular_, 1.);
    pnode_.param<double>("max_abs_linear", max_abs_linear_, .4);
    pnode_.param<double>("max_abs_angular", max_abs_angular_, 1.);
    sub_joy_ = node_.subscribe("/bluetooth_teleop/joy", 10,
                                &JackalController::JoystickHandler, this);
    sub_meas_ = node_.subscribe("/vrpn_client_node/" + jackal_name + "/pose", 10,
                                &JackalController::MeasurementHandler, this);
    pub_cmd_ = node_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  }

 private:
  void ControlLoop() {
    ros::Rate rate(control_freq_);
    while (ros::ok()) {
      rate.sleep();
    }
  }

  void JoystickHandler(const sensor_msgs::JoyConstPtr& msg) {
    if (msg->buttons[btn_dead_]) {
      std::lock_guard<std::mutex> lock(mtx_dead_);
      is_dead_ = true;
    }

    SendCommand(msg->axes[axis_linear_] * scale_linear_,
                msg->axes[axis_angular_] * scale_angular_);
  }

  void MeasurementHandler(const geometry_msgs::PoseStampedConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mtx_state_);
    state_.rotation.angle() = 2 * atan2(msg->pose.orientation.z, msg->pose.orientation.w);
    state_.position << msg->pose.position.x, msg->pose.position.y;
  }

  void SendCommand(const double linear_vel, const double angular_vel) {
    {
      std::lock_guard<std::mutex> lock(mtx_dead_);
      if (is_dead_) {
        geometry_msgs::Twist msg{};
        pub_cmd_.publish(msg);
        return;
      }
    }

    geometry_msgs::Twist msg{};
    msg.linear.x = std::min(std::max(linear_vel, -max_abs_linear_), max_abs_linear_);
    msg.angular.z = std::min(std::max(angular_vel, -max_abs_angular_), max_abs_angular_);

    pub_cmd_.publish(msg);
  }

  jackel_state_t state_;
  std::mutex mtx_state_;

  bool is_dead_ = false;
  std::mutex mtx_dead_;

  // params
  int btn_dead_;
  int axis_linear_;
  int axis_angular_;
  int control_freq_;
  double max_abs_linear_;
  double max_abs_angular_;
  double scale_linear_;
  double scale_angular_;

  ros::NodeHandle node_;
  ros::NodeHandle pnode_;
  ros::Subscriber sub_joy_;
  ros::Subscriber sub_meas_;
  ros::Publisher pub_cmd_;
};

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "jackel_control");

  JackalController controller;

  ros::spin();

  return 0;
}
