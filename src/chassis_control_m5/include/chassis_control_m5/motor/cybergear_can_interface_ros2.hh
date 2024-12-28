#ifndef CYBERGEAR_CAN_INTERFACE_ESP32_HH
#define CYBERGEAR_CAN_INTERFACE_ESP32_HH

#include <deque>
#include "cybergear_can_interface.hh"
#include <rclcpp/rclcpp.hpp>
#include "can_msgs/msg/frame.hpp"


class CybergearCanInterfaceRos2 : public CybergearCanInterface, public rclcpp::Node
{
public:
  CybergearCanInterfaceRos2(std::string node_name="cybergear_can");
  virtual ~CybergearCanInterfaceRos2();
  virtual bool send_message(uint32_t id, const uint8_t * data, uint8_t len, bool ext);
  virtual bool read_message(unsigned long & id, uint8_t * data, uint8_t & len);
  virtual bool available();
  virtual bool support_interrupt();

private:
  std::mutex mu_dq;  //!< mutex
  std::deque<can_msgs::msg::Frame> receive_buffer_;  //!< receive buffer
  rclcpp::Node::SharedPtr nh_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_subscription_;
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_publisher_;
  void CanMsgCallback(const can_msgs::msg::Frame::SharedPtr msg);

};

#endif  // ESP_CAN_INTERFACE_HH
