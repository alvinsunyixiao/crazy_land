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
  JackalController() {
    std::string jackal_name;
    ros::param::param<std::string>("~jackal_name", jackal_name, "alvin_jk");
    ros::param::param("~dead_button", btn_dead_, 1);
    ros::param::param("~control_frequency", control_freq_, 100);
    ros::param::param("~axis_linear", axis_linear_, 1);
    ros::param::param("~axis_angular", axis_angular_, 0);
    ros::param::param("~scale_linear", scale_linear_, .4);
    ros::param::param("~scale_angular", scale_angular_, 1.);
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
    const Eigen::Quaterniond rot_3d(msg->pose.orientation.w, msg->pose.orientation.x,
                                    msg->pose.orientation.y, msg->pose.orientation.z);
    std::lock_guard<std::mutex> lock(mtx_state_);
    state_.rotation.angle() = rot_3d.toRotationMatrix().eulerAngles(2, 1, 0)[0];
    state_.position << msg->pose.position.x, msg->pose.position.y, 0;
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
    msg.linear.x = linear_vel;
    msg.angular.z = angular_vel;

    pub_cmd_.publish(msg);
  }

  jackel_state_t state_;
  std::mutex mtx_state_;

  bool is_dead_ = false;
  std::mutex mtx_dead_;

  int btn_dead_;
  int axis_linear_;
  int axis_angular_;
  int control_freq_;
  double scale_linear_;
  double scale_angular_;

  ros::NodeHandle node_;
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
