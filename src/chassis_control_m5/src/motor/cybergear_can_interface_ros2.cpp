#include "cybergear_can_interface_ros2.hh"


#include <cstdint>
#include <cstring>

#include "cybergear_can_interface.hh"
#include "cybergear_driver_utils.hh"

CybergearCanInterfaceRos2::CybergearCanInterfaceRos2(std::string node_name)
    : CybergearCanInterface(), Node(node_name) {
  can_subscription_ = this->create_subscription<can_msgs::msg::Frame>(
      "/CAN/can0/receive", 10,
      std::bind(&CybergearCanInterfaceRos2::CanMsgCallback, this, std::placeholders::_1));
  can_publisher_ =
      this->create_publisher<can_msgs::msg::Frame>("/CAN/can0/transmit", 10);
  while (rclcpp::ok() && can_publisher_->get_subscription_count() == 0) {
        rclcpp::sleep_for(std::chrono::milliseconds(100));  // 每隔 100ms 检查一次
  }
}

CybergearCanInterfaceRos2::~CybergearCanInterfaceRos2() {}

bool CybergearCanInterfaceRos2::send_message(uint32_t id, const uint8_t* data,
                                             uint8_t len, bool ext) {
  can_msgs::msg::Frame TxMessage;
  if (len > 8) len = 8;
  TxMessage.id = id;
  TxMessage.dlc = len;
  TxMessage.is_extended = ext;
  TxMessage.is_rtr = false;
  TxMessage.is_error = false;
  
  for(int i = 0; i < len; ++i) {
    TxMessage.data[i] = data[i];
  }
  can_publisher_->publish(TxMessage);
  return true;
}

bool CybergearCanInterfaceRos2::read_message(unsigned long& id, uint8_t* data,
                                             uint8_t& len) {
  CG_DEBUG_FUNC
  std::lock_guard<std::mutex> lock(mu_dq);
  // check empty
  if (receive_buffer_.empty()) return false;

  // get mseesage from buffer
  can_msgs::msg::Frame msg = receive_buffer_.front();
  receive_buffer_.pop_front();
  id = msg.id;
  len = msg.dlc;
  for(int i = 0; i < len; ++i) {
    data[i] = msg.data[i];
  }
  return true;
}

bool CybergearCanInterfaceRos2::available() {
  CG_DEBUG_FUNC
  return (receive_buffer_.empty() == false);
}

bool CybergearCanInterfaceRos2::support_interrupt() { return true; }

void CybergearCanInterfaceRos2::CanMsgCallback(
    const can_msgs::msg::Frame::SharedPtr msg) {
  std::lock_guard<std::mutex> lock(mu_dq);
  receive_buffer_.push_back(*msg);
}
