#include "ros2socketcan.h"

using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

ros2socketcan::ros2socketcan(std::string can_socket2)
    : Node("ros2" + can_socket2), stream(ios), signals(ios, SIGINT, SIGTERM) {
  declare_parameter("can_socket_interface", "can0");
  declare_parameter("can_bitrate", 1000000);

  can_socket_interface_ = get_parameter("can_socket_interface").as_string();
  can_bitrate_ = get_parameter("can_bitrate").as_int();
}

void ros2socketcan::Init() {

  std::string interface(can_socket_interface_);

  std::string bringDownCmd = "sudo ip link set down " + interface;
  int result = system(bringDownCmd.c_str());
  if (result != 0) {
    std::cerr << "Failed to bring down " << interface << std::endl;
    return;
  }

  std::string setBitrateCmd = "sudo ip link set " + interface +
                              " type can bitrate " +
                              std::to_string(can_bitrate_);
  result = system(setBitrateCmd.c_str());
  if (result != 0) {
    std::cerr << "Failed to set bitrate for " << interface << std::endl;
    return;
  }

  std::string bringUpCmd = "sudo ip link set up " + interface;

  result = system(bringUpCmd.c_str());
  if (result != 0) {
    std::cerr << "Failed to bring up " << interface << std::endl;
    return;
  }

  topicname_receive << "CAN/" << interface << "/" << "receive";
  topicname_transmit << "CAN/" << interface << "/" << "transmit";

  rclcpp::executors::MultiThreadedExecutor exec;

  publisher_ = this->create_publisher<can_msgs::msg::Frame>(
      topicname_receive.str(), 10);
  test_pub_ = this->create_publisher<can_msgs::msg::Frame>(
      topicname_transmit.str(), 10);
  subscription_ = this->create_subscription<can_msgs::msg::Frame>(
      topicname_transmit.str(), 1,
      std::bind(&ros2socketcan::CanPublisher, this, std::placeholders::_1));

  strcpy(ifr.ifr_name, interface.c_str());
  ioctl(natsock, SIOCGIFINDEX, &ifr);

  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(natsock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
    perror("Error in socket bind");
  }

  stream.assign(natsock);

  std::cout << "ROS2 to CAN-Bus topic:" << subscription_->get_topic_name()
            << std::endl;
  std::cout << "CAN-Bus to ROS2 topic:" << publisher_->get_topic_name()
            << std::endl;

  stream.async_read_some(boost::asio::buffer(&rec_frame, sizeof(rec_frame)),
                         std::bind(&ros2socketcan::CanListener, this,
                                   std::ref(rec_frame), std::ref(stream)));

  signals.async_wait(std::bind(&ros2socketcan::stop, this));

  boost::system::error_code ec;

  std::size_t (boost::asio::io_service::*run)() = &boost::asio::io_service::run;
  std::thread bt(std::bind(run, &ios));
  bt.detach();

  rclcpp::spin(shared_from_this());
}

void ros2socketcan::stop() {
  printf(
      "\nEnd of Listener Thread. Please press strg+c again to stop the whole "
      "program.\n");
  ios.stop();
  signals.clear();
}

ros2socketcan::~ros2socketcan() { printf("\nEnd of Publisher Thread. \n"); }

void ros2socketcan::CanSend(const can_msgs::msg::Frame msg) {
  struct can_frame frame1;

  frame1.can_id = msg.id;

  if (msg.is_extended == 1) {
    frame1.can_id = frame1.can_id + CAN_EFF_FLAG;
  }

  if (msg.is_error == 1) {
    frame1.can_id = frame1.can_id + CAN_ERR_FLAG;
  }

  if (msg.is_rtr == 1) {
    frame1.can_id = frame1.can_id + CAN_RTR_FLAG;
  }

  frame1.can_dlc = msg.dlc;

  for (int i = 0; i < (int)frame1.can_dlc; i++) {
    frame1.data[i] = msg.data[i];
  }

  printf("S | %x | %s | ", frame1.can_id, frame1.data);
  for (int j = 0; j < (int)frame1.can_dlc; j++) {
    printf("%x ", frame1.data[j]);
  }
  printf("\n");

  can_frame send_frame = frame1;
  stream.async_write_some(boost::asio::buffer(&send_frame, sizeof(send_frame)),
                          std::bind(&ros2socketcan::CanSendConfirm, this));
}

void ros2socketcan::CanPublisher(const can_msgs::msg::Frame::SharedPtr msg) {
  can_msgs::msg::Frame msg1 = *msg;
  CanSend(msg1);
}

void ros2socketcan::CanSendConfirm(void) {
  // std::cout << "Message sent" << std::endl;
}

void ros2socketcan::CanListener(
    struct can_frame& rec_frame,
    boost::asio::posix::basic_stream_descriptor<>& stream) {
  can_msgs::msg::Frame frame;

  std::stringstream s;
  can_frame tmp_rec_frame = rec_frame;

  frame.id = tmp_rec_frame.can_id;
  frame.dlc = int(tmp_rec_frame.can_dlc);

  printf("R | %x | ", tmp_rec_frame.can_id);
  for (int i = 0; i < tmp_rec_frame.can_dlc; i++) {
    frame.data[i] = tmp_rec_frame.data[i];
    s << tmp_rec_frame.data[i];
  }

  if (tmp_rec_frame.can_id&CAN_EFF_FLAG) {
    frame.is_extended = true;
  }

  if (tmp_rec_frame.can_id&CAN_ERR_FLAG) {
    frame.is_error = true;
  }

  if (tmp_rec_frame.can_id&CAN_RTR_FLAG) {
    frame.is_rtr = true;
  }
  current_frame = frame;
  std::cout << s.str() << " | ";

  for (int j = 0; j < (int)tmp_rec_frame.can_dlc; j++) {
    printf("%x ", tmp_rec_frame.data[j]);
  }
  printf("\n");

  publisher_->publish(current_frame);

  stream.async_read_some(boost::asio::buffer(&rec_frame, sizeof(rec_frame)),
                         std::bind(&ros2socketcan::CanListener, this,
                                   std::ref(rec_frame), std::ref(stream)));
}

int main(int argc, char* argv[]) {
  std::cout << programdescr << std::endl;
  rclcpp::init(argc, argv);

  auto ros2canptr = std::make_shared<ros2socketcan>();
  ros2canptr->Init();

  rclcpp::shutdown();
  return 0;
}
