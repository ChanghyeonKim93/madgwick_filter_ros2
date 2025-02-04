#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "bridge.h"
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

    PrepareSubscribers();
    PreparePublishers();
  }

  ~MadgwickFilterNode() {}

  bool LoadParameters(madgwick_filter::Parameters* params) { return true; }
  void PrepareSubscribers() {
    imu_subscriber_ = this->create_subscription<ImuMsg>(
        "/imu/data", rclcpp::SensorDataQoS(),
        std::bind(&MadgwickFilterNode::CallbackImu, this,
                  std::placeholders::_1));
  }
  void PreparePublishers() {
    pose_publisher_ =
        this->create_publisher<OdometryMsg>("~/pose", rclcpp::SensorDataQoS());
  }
  void CallbackImu(const ImuMsg::SharedPtr msg) {
    madgwick_filter::bridge::Imu imu_data;
    imu_data.time = static_cast<double>(msg->header.stamp.sec) +
                    msg->header.stamp.nanosec * 1e-9;
    imu_data.linear_acceleration.x = msg->linear_acceleration.x;
    imu_data.linear_acceleration.y = msg->linear_acceleration.y;
    imu_data.linear_acceleration.z = msg->linear_acceleration.z;
    imu_data.angular_velocity.x = msg->angular_velocity.x;
    imu_data.angular_velocity.y = msg->angular_velocity.y;
    imu_data.angular_velocity.z = msg->angular_velocity.z;
    filter_->Update(imu_data);

    const auto bridge_orietation = filter_->GetOrientation();
    OdometryMsg odom_msg;
    odom_msg.header.frame_id = "map";
    odom_msg.header.stamp = now();
    odom_msg.pose.pose.position.x = 0.0;
    odom_msg.pose.pose.position.y = 0.0;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation.w = bridge_orietation.w;
    odom_msg.pose.pose.orientation.x = bridge_orietation.x;
    odom_msg.pose.pose.orientation.y = bridge_orietation.y;
    odom_msg.pose.pose.orientation.z = bridge_orietation.z;
    // odom_msg.pose.pose.orientation.w = msg->orientation.w;
    // odom_msg.pose.pose.orientation.x = msg->orientation.x;
    // odom_msg.pose.pose.orientation.y = msg->orientation.y;
    // odom_msg.pose.pose.orientation.z = msg->orientation.z;
    pose_publisher_->publish(odom_msg);
  }
  void CallbackTimer() {}

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