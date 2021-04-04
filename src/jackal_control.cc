#include <cmath>
#include <memory>
#include <mutex>
#include <thread>

#include "ros/ros.h"

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "sensor_msgs/Joy.h"

#include "types.h"

class JackalController {
 public:
  JackalController() : pnode_("~"), t_control_(&JackalController::ControlLoop, this) {
    // initialize target to origin
    target_pose_.position.setZero();
    target_pose_.rotation.angle() = 0;

    std::string jackal_name;
    node_.getParam("/crazy_params/jackal_name", jackal_name);
    node_.getParam("/crazy_params/btn_circle", btn_manual_);
    node_.getParam("/crazy_params/btn_cross", btn_dead_);
    pnode_.param<int>("control_frequency", control_freq_, 100);
    pnode_.param<int>("axis_linear", axis_linear_, 1);
    pnode_.param<int>("axis_angular", axis_angular_, 0);
    pnode_.param<double>("scale_linear", scale_linear_, .4);
    pnode_.param<double>("scale_angular", scale_angular_, 1.);
    pnode_.param<double>("max_abs_linear", max_abs_linear_, .7);
    pnode_.param<double>("max_abs_angular", max_abs_angular_, 2.);
    pnode_.param<double>("error_deadband", error_deadband_, 0.02);
    pnode_.param<double>("gain_linear", gain_linear_, 4.0);
    pnode_.param<double>("gain_angular", gain_angular_, 8.0);
    pnode_.param<double>("target_max_abs_x", target_max_abs_x_, 1.1);
    pnode_.param<double>("target_max_abs_y", target_max_abs_y_, 1.1);
    sub_joy_ = node_.subscribe("/bluetooth_teleop/joy", 10,
                                &JackalController::JoystickHandler, this);
    sub_meas_ = node_.subscribe("/vrpn_client_node/" + jackal_name + "/pose", 10,
                                &JackalController::MeasurementHandler, this);
    sub_target_ = node_.subscribe("/tracking/jackal", 10,
                                  &JackalController::TargetHandler, this);
    pub_cmd_ = node_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  }

  ~JackalController() {
    t_control_.join();
  }

 private:
  void ControlLoop() {
    ros::Duration(5).sleep();
    ros::Rate rate(control_freq_);
    while (ros::ok()) {
      {
        std::lock_guard<std::mutex> lock(mtx_mode_);
        if (is_manual_) { continue; }
      }
      // read state and target
      jackal_state_t current_state, target_state;
      {
        std::lock_guard<std::mutex> lock(mtx_state_);
        current_state = state_;
      }
      {
        std::lock_guard<std::mutex> lock(mtx_target_);
        target_state = target_pose_;
      }

      // angular error
      const Eigen::Vector2d position_diff = target_state.position - current_state.position;
      const Eigen::Rotation2Dd target_rotation(std::atan2(position_diff.y(), position_diff.x()));
      const double error_rot = (target_rotation * current_state.rotation.inverse()).smallestAngle();

      // linear error
      const double error_pos = position_diff.norm();

      SendCommand(error_pos > error_deadband_ ? error_pos * gain_linear_ : 0.,
                  error_pos > error_deadband_ ? error_rot * gain_angular_ : 0.);
      rate.sleep();
    }
  }

  void JoystickHandler(const sensor_msgs::JoyConstPtr& msg) {
    if (msg->buttons[btn_dead_]) {
      std::lock_guard<std::mutex> lock(mtx_dead_);
      is_dead_ = true;
    }

    {
      std::lock_guard<std::mutex> lock(mtx_mode_);
      is_manual_ = msg->buttons[btn_manual_];
      if (!is_manual_) { return; }
    }

    SendCommand(msg->axes[axis_linear_] * scale_linear_,
                msg->axes[axis_angular_] * scale_angular_);
  }

  void MeasurementHandler(const geometry_msgs::PoseStampedConstPtr& msg) {
    std::lock_guard<std::mutex> lock(mtx_state_);
    state_.rotation.angle() = 2 * atan2(msg->pose.orientation.z, msg->pose.orientation.w);
    state_.position << msg->pose.position.x, msg->pose.position.y;
  }

  void TargetHandler(const geometry_msgs::Pose2DConstPtr& msg) {
    ROS_INFO("Tracking target @ (%f %f)", msg->x, msg->y);
    std::lock_guard<std::mutex> lock(mtx_target_);
    // clamp x y to stay within bound
    const double x_safe = std::min(std::max(msg->x, -target_max_abs_x_), target_max_abs_x_);
    const double y_safe = std::min(std::max(msg->y, -target_max_abs_y_), target_max_abs_y_);

    target_pose_.position << x_safe, y_safe;
    target_pose_.rotation.angle() = msg->theta;
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

  jackal_state_t state_;
  std::mutex mtx_state_;

  bool is_dead_ = false;
  std::mutex mtx_dead_;

  bool is_manual_ = false;
  std::mutex mtx_mode_;

  jackal_state_t target_pose_;
  std::mutex mtx_target_;

  // params
  int btn_dead_;
  int btn_manual_;
  int axis_linear_;
  int axis_angular_;
  int control_freq_;
  double error_deadband_;
  double max_abs_linear_;
  double max_abs_angular_;
  double scale_linear_;
  double scale_angular_;
  double gain_linear_;
  double gain_angular_;
  double target_max_abs_x_;
  double target_max_abs_y_;

  // ROS Subscriber / Publisher
  ros::NodeHandle node_;
  ros::NodeHandle pnode_;
  ros::Subscriber sub_joy_;
  ros::Subscriber sub_meas_;
  ros::Subscriber sub_target_;
  ros::Publisher pub_cmd_;

  std::thread t_control_;

};

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "jackal_control");

  JackalController controller;

  ros::spin();

  return 0;
}
