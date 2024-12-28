#include "chassis_control_m5/chassis_control.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chassis_control_m5/motor/cybergear_driver_defs.hh>
#include <cstring>

ChassisControl::~ChassisControl()
{
  motor_controller_->send_speed_command(left_config_.id, 0);
  motor_controller_->send_speed_command(right_config_.id, 0);
}
ChassisControl::ChassisControl()
    : Node("chassis_node"),
      timer_(),
      software_configs_(),
      left_config_(),
      right_config_(),
      motor_jont_names()
{
  // declare motor configs
  std::vector<uint8_t> ids;
  declare_parameter("motors.master_id", 0x00);
  declare_parameter("motors.rate", 100.0);

  std::vector<std::string> motor_names;
  std::vector<std::string> joint_names;
  joint_names.push_back("left_motor_joint_name");
  joint_names.push_back("right_motor_joint_name");
  motor_names.push_back("right");
  motor_names.push_back("left");
  declare_parameter("motors.joint_names", joint_names);
  declare_parameter("motors.wheelbase", 0.2);

  joint_names = get_parameter("motors.joint_names").as_string_array();
  wheelbase_length_ = get_parameter("motors.wheelbase").as_double();
  std::cout << joint_names[0] << " " << joint_names[1] << std::endl;
  motor_jont_names[motor_names[0]] = joint_names[1];
  motor_jont_names[motor_names[1]] = joint_names[0];

  for (auto name : motor_names)
  {
    declare_parameter("motors." + name + ".id", 0x00);
    declare_parameter("motors." + name + ".velocity_limit",
                      DEFAULT_VELOCITY_LIMIT);
    declare_parameter("motors." + name + ".current_limit",
                      DEFAULT_CURRENT_LIMIT);
    declare_parameter("motors." + name + ".torque_limit", DEFAULT_TORQUE_LIMIT);
    declare_parameter("motors." + name + ".direction", 1);
    declare_parameter("motors." + name + ".wheel_radius", 0.2);
    declare_parameter("motors." + name + ".gear_ratio", 1.0);
  }

  master_id_ = get_parameter("motors.master_id").as_int();
  rate_ = get_parameter("motors.rate").as_double();
  for (auto name : motor_names)
  {
    CybergearSoftwareConfig config;
    config.id = get_parameter("motors." + name + ".id").as_int();
    ids.push_back(config.id);
    config.limit_speed =
        get_parameter("motors." + name + ".velocity_limit").as_double();
    config.limit_current =
        get_parameter("motors." + name + ".current_limit").as_double();
    config.limit_torque =
        get_parameter("motors." + name + ".torque_limit").as_double();
    config.direction = get_parameter("motors." + name + ".direction").as_int();

    if (name == "left")
    {
      left_config_ = config;
      left_wheel_radius_ =
          get_parameter("motors." + name + ".wheel_radius").as_double();
      left_gear_ratio_ =
          get_parameter("motors." + name + ".gear_ratio").as_double();
    }
    if (name == "right")
    {
      right_config_ = config;
      right_wheel_radius_ =
          get_parameter("motors." + name + ".wheel_radius").as_double();
      right_gear_ratio_ =
          get_parameter("motors." + name + ".gear_ratio").as_double();
    }

    software_configs_.push_back(config);
  }

  motor_controller_ = std::make_shared<CybergearController>(master_id_);

  motor_controller_interface_ = std::make_shared<CybergearCanInterfaceRos2>(
      std::string("motor_controller_interface"));

  motor_controller_->init(ids, software_configs_, (uint8_t)ControlMode::Velocity,
                          motor_controller_interface_.get(), 1000);

  motor_controller_->set_run_mode((uint8_t)ControlMode::Velocity);

  motor_controller_->enable_motors();

  auto duration = std::chrono::duration<int64_t, std::milli>(
      static_cast<int64_t>(1.0 / rate_ * 1000.0));
  timer_ = create_wall_timer(duration,
                             std::bind(&ChassisControl::timer_callback, this));

  cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", rclcpp::QoS(1),
      std::bind(&ChassisControl::inputCommandCallback, this,
                std::placeholders::_1));

  js_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 1);

  odometry_node_ = std::make_shared<OdometryNode>(right_wheel_radius_, wheelbase_length_);
  battery_manager_node_ = std::make_shared<BatteryManager>();
  get_cmd_vel_time_ = this->now();
  update_battery_time_ = this->now();
}

void ChassisControl::timer_callback()
{
  if ((this->now() - get_cmd_vel_time_).seconds() > 1)
  {
    motor_controller_->send_speed_command(left_config_.id, 0);
    motor_controller_->send_speed_command(right_config_.id, 0);

    get_cmd_vel_time_ = this->now();
  }
  if ((this->now() - update_battery_time_).seconds() > 1)
  {
    battery_manager_node_->update(motor_controller_->get_vbus(right_config_.id));
    update_battery_time_ = this->now();
  }

  motor_controller_->process_packet();
  double left_speed = 0.0;
  double right_speed = 0.0;
  for (auto it : motor_jont_names)
  {
    MotorStatus motor_status;
    if (it.first == "right")
    {
      motor_controller_->get_motor_status(right_config_.id, motor_status);
      right_speed = motor_status.velocity;
    }
    else if (it.first == "left")
    {
      motor_controller_->get_motor_status(left_config_.id, motor_status);
      left_speed = motor_status.velocity;
    }
    sensor_msgs::msg::JointState joint;
    joint.name = {it.second};
    joint.position = {motor_status.position};
    joint.velocity = {motor_status.velocity};
    joint.effort = {motor_status.effort};
    js_pub_->publish(joint);
  }

  odometry_node_->updateOdometry(left_speed, right_speed);

  rclcpp::spin_some(motor_controller_interface_->get_node_base_interface());
  rclcpp::spin_some(odometry_node_->get_node_base_interface());
  fflush(stdout);
}

void ChassisControl::inputCommandCallback(
    const geometry_msgs::msg::Twist::SharedPtr msg)
{
  double v_L = msg->linear.x - msg->angular.z * wheelbase_length_ / 2.0;
  double v_R = msg->linear.x + msg->angular.z * wheelbase_length_ / 2.0;

  double right_speed =
      v_R / right_wheel_radius_ * left_gear_ratio_;
  double left_speed =
      v_L / left_wheel_radius_ * right_gear_ratio_;

  motor_controller_->send_speed_command(left_config_.id, left_speed);
  motor_controller_->send_speed_command(right_config_.id, right_speed);
  get_cmd_vel_time_ = this->now();
  RCLCPP_INFO(
      get_logger(),
      "Received cmd_vel message. linear x: %f, y: %f  m_right: %f m_left: %f",
      msg->linear.x, msg->linear.y, right_speed, left_speed);
}
