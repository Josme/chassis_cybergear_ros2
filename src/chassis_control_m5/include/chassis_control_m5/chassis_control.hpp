#ifndef JOY_TO_CAN_NODE_HPP
#define JOY_TO_CAN_NODE_HPP

#include <can_msgs/msg/frame.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "chassis_control_m5/motor/cybergear_can_interface_ros2.hh"
#include "chassis_control_m5/motor/cybergear_controller.hh"
#include "chassis_control_m5/odom.hpp"
#include "chassis_control_m5/battery_manager.hpp"
#include <sensor_msgs/msg/battery_state.hpp>
enum class ControlMode
{
  Position = MODE_POSITION,
  Velocity = MODE_SPEED,
  Current = MODE_CURRENT,
  Motion = MODE_MOTION
};

class ChassisControl : public rclcpp::Node {
 public:
  ChassisControl();
  ~ChassisControl();

  typedef std::unordered_map<std::string, std::string> MotorJontNames;

 private:
  rclcpp::Node::SharedPtr nh_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;

  std::shared_ptr<CybergearController> motor_controller_;
  std::shared_ptr<CybergearCanInterfaceRos2> motor_controller_interface_;
  std::shared_ptr<OdometryNode> odometry_node_;
  std::shared_ptr<BatteryManager> battery_manager_node_;
  rclcpp::TimerBase::SharedPtr timer_;
  MotorJontNames motor_jont_names;
  CybergearSoftwareConfig left_config_;
  CybergearSoftwareConfig right_config_;
  std::vector<CybergearSoftwareConfig> software_configs_;
  uint8_t master_id_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_pub_;

  double rate_;
  double wheelbase_length_;
  double left_wheel_radius_;
  double right_wheel_radius_;
  double right_gear_ratio_;
  double left_gear_ratio_;
  rclcpp::Time get_cmd_vel_time_;
  rclcpp::Time update_battery_time_;


  void inputCommandCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void timer_callback();
};

#endif
