#pragma once

#include <atomic>
#include <string>
#include <thread>

#include <opencv2/videoio.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/image.hpp>

class ImageReceiver : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit ImageReceiver(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ImageReceiver() override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State & state) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  void capture_loop();
  void stop_capture();

  std::string vehicle_type_;
  int vehicle_id_;
  std::string unreal_ip_;
  int rtsp_port_;
  std::string camera_type_;
  std::string rtsp_url_;
  std::string topic_;
  std::string gst_pipeline_;

  cv::VideoCapture cap_;
  std::thread capture_thread_;
  std::atomic<bool> running_{false};

  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};
