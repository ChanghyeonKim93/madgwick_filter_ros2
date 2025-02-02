#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "madgwick_filter.h"

using OdometryMsg = nav_msgs::msg::Odometry;
using ImuMsg = sensor_msgs::msg::Imu;

class MadgwickFilterNode : public rclcpp::Node {
 public:
  MadgwickFilterNode(const std::string& node_name) : Node(node_name) {
    std::cerr << "node starts\n";

    madgwick_filter::Parameters parameters;
    if (!LoadParameters(&parameters))
      throw std::runtime_error("wrong parameters");
    filter_ = std::make_unique<madgwick_filter::MadgwickFilter>(parameters);
  }

  ~MadgwickFilterNode() {}

  bool LoadParameters(madgwick_filter::Parameters* params) { return true; }

 private:
  // Subscibers
  rclcpp::Subscription<ImuMsg>::SharedPtr imu_subscriber_;

  // Publishers
  rclcpp::Publisher<OdometryMsg>::SharedPtr pose_publisher_;

  rclcpp::TimerBase::SharedPtr publisher_timer_;

  std::unique_ptr<madgwick_filter::MadgwickFilter> filter_{nullptr};
};

#define NODE_NAME "madgwick_filter_node"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<MadgwickFilterNode>(NODE_NAME));
    rclcpp::shutdown();
  } catch (std::exception& e) {
    std::cerr << e.what() << std::endl;
  }

  return 0;
}