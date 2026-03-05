#include "ImageReceiver.hpp"

#include <chrono>
#include <thread>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgproc.hpp>
#include <std_msgs/msg/header.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

ImageReceiver::ImageReceiver(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("image_receiver", options)
{
  declare_parameter<std::string>("vehicle_type", "x500");
  declare_parameter<int>("vehicle_id", 0);
  declare_parameter<std::string>("unreal_ip", "127.0.0.1");
  declare_parameter<int>("rtsp_port", 8554);
  declare_parameter<std::string>("camera_type", "front");
}

ImageReceiver::~ImageReceiver()
{
  stop_capture();
}

CallbackReturn ImageReceiver::on_configure(const rclcpp_lifecycle::State &)
{
  vehicle_type_ = get_parameter("vehicle_type").as_string();
  vehicle_id_   = get_parameter("vehicle_id").as_int();
  unreal_ip_    = get_parameter("unreal_ip").as_string();
  rtsp_port_    = get_parameter("rtsp_port").as_int();
  camera_type_  = get_parameter("camera_type").as_string();

  rtsp_url_ = "rtsp://" + unreal_ip_ + ":" + std::to_string(rtsp_port_) +
              "/" + vehicle_type_ + "_" + std::to_string(vehicle_id_) +
              "/" + camera_type_;

  topic_ = "/vehicle" + std::to_string(vehicle_id_ + 1) +
           "/camera/" + camera_type_ + "/image_raw";

  gst_pipeline_ =
    "rtspsrc location=" + rtsp_url_ + " protocols=tcp latency=0 ! "
    "rtph264depay ! h264parse ! avdec_h264 ! "
    "videoconvert ! video/x-raw,format=BGR ! "
    "appsink drop=true sync=false";

  RCLCPP_INFO(get_logger(), "Configured: RTSP=%s  topic=%s", rtsp_url_.c_str(), topic_.c_str());

  publisher_ = create_publisher<sensor_msgs::msg::Image>(topic_, rclcpp::SensorDataQoS());

  return CallbackReturn::SUCCESS;
}

CallbackReturn ImageReceiver::on_activate(const rclcpp_lifecycle::State & state)
{
  LifecycleNode::on_activate(state);

  running_ = true;
  capture_thread_ = std::thread(&ImageReceiver::capture_loop, this);

  RCLCPP_INFO(get_logger(), "Activated: waiting for subscribers on %s", topic_.c_str());
  return CallbackReturn::SUCCESS;
}

CallbackReturn ImageReceiver::on_deactivate(const rclcpp_lifecycle::State & state)
{
  auto sub_count = publisher_->get_subscription_count();
  if (sub_count > 0) {
    RCLCPP_WARN(get_logger(), "Deactivating with %zu active subscriber(s), stream will be interrupted", sub_count);
  }
  stop_capture();
  LifecycleNode::on_deactivate(state);
  RCLCPP_INFO(get_logger(), "Deactivated");
  return CallbackReturn::SUCCESS;
}

CallbackReturn ImageReceiver::on_cleanup(const rclcpp_lifecycle::State &)
{
  stop_capture();
  publisher_.reset();
  RCLCPP_INFO(get_logger(), "Cleaned up");
  return CallbackReturn::SUCCESS;
}

CallbackReturn ImageReceiver::on_shutdown(const rclcpp_lifecycle::State &)
{
  stop_capture();
  publisher_.reset();
  RCLCPP_INFO(get_logger(), "Shutdown");
  return CallbackReturn::SUCCESS;
}

void ImageReceiver::capture_loop()
{
  cv::Mat frame;
  while (running_) {
    // Pause RTSP stream when no subscribers (UE5 optimization)
    if (publisher_->get_subscription_count() == 0) {
      if (cap_.isOpened()) {
        cap_.release();
        RCLCPP_INFO(get_logger(), "No subscribers, pausing RTSP stream");
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      continue;
    }

    // Reconnect when subscribers appear
    if (!cap_.isOpened()) {
      RCLCPP_INFO(get_logger(), "Subscriber detected, reconnecting to %s", rtsp_url_.c_str());
      cap_.open(gst_pipeline_, cv::CAP_GSTREAMER);
      if (!cap_.isOpened()) {
        RCLCPP_WARN(get_logger(), "Reconnect failed, retrying...");
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        continue;
      }
    }

    if (!cap_.read(frame) || frame.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Empty frame, retrying...");
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }

    auto msg = cv_bridge::CvImage(
      std_msgs::msg::Header{},
      "bgr8",
      frame
    ).toImageMsg();

    msg->header.stamp = now();
    msg->header.frame_id = "vehicle" + std::to_string(vehicle_id_ + 1) +
                           "_" + camera_type_;

    publisher_->publish(*msg);
  }
}

void ImageReceiver::stop_capture()
{
  running_ = false;
  if (capture_thread_.joinable()) {
    capture_thread_.join();
  }
  if (cap_.isOpened()) {
    cap_.release();
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ImageReceiver>();

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();
  return 0;
}
