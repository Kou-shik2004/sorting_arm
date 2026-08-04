#ifndef SORTING_ARM_PERCEPTION__PERCEPTION_NODE_HPP_
#define SORTING_ARM_PERCEPTION__PERCEPTION_NODE_HPP_

#include <builtin_interfaces/msg/time.hpp>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <message_filters/subscriber.hpp>
#include <message_filters/sync_policies/approximate_time.hpp>
#include <message_filters/synchronizer.hpp>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sorting_arm_interfaces/srv/detect_objects.hpp>
#include <std_msgs/msg/header.hpp>
#include <string>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <vector>

#include "sorting_arm_perception/rgbd_detector.hpp"

namespace sorting_arm_perception {

class PerceptionNode : public rclcpp::Node {
 public:
  PerceptionNode();

 private:
  using DetectObjects = sorting_arm_interfaces::srv::DetectObjects;
  using ApproximateTimePolicy =
      message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
  using Synchronizer = message_filters::Synchronizer<ApproximateTimePolicy>;

  struct CaptureConfig {
    bool registration_verified = false;
    double maximum_pairing_interval_seconds = 0.0;
    double capture_timeout_seconds = 0.0;
    double transform_timeout_seconds = 0.0;
  };

  struct SynchronizedPair {
    sensor_msgs::msg::Image::ConstSharedPtr rgb;
    sensor_msgs::msg::Image::ConstSharedPtr depth;
    builtin_interfaces::msg::Time capture_stamp;
    std::int64_t timestamp_difference_nanoseconds = 0;
    std::uint64_t sequence = 0;
  };

  struct DisplayConfig {
    std::vector<std::string> arm_joint_names;
    double motion_threshold_radians = 0.0;
  };

  void load_parameters();
  void synchronized_callback(sensor_msgs::msg::Image::ConstSharedPtr rgb, sensor_msgs::msg::Image::ConstSharedPtr depth);
  void camera_info_callback(sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info);
  void joint_state_callback(sensor_msgs::msg::JointState::ConstSharedPtr joint_state);
  void detect(const std::shared_ptr<DetectObjects::Request> request, std::shared_ptr<DetectObjects::Response> response);

  bool validate_camera_info(const sensor_msgs::msg::CameraInfo& camera_info, std::string& error) const;
  bool validate_projection_contract(const SynchronizedPair& pair, const sensor_msgs::msg::CameraInfo& camera_info,
                                    image_geometry::PinholeCameraModel& camera_model, bool& rectify_pixels,
                                    std::string& phase, std::string& error) const;
  void publish_debug(const std_msgs::msg::Header& header, const cv::Mat& image) const;
  void publish_debug_failure(const std_msgs::msg::Header& header, const cv::Mat& rgb_image, const std::string& phase,
                             const std::string& message) const;
  void publish_display(const sensor_msgs::msg::Image::ConstSharedPtr& rgb);
  void clear_display_overlay();
  void activate_display_overlay(const std::vector<DetectedCube>& cubes);
  static void fail(const std::shared_ptr<DetectObjects::Response>& response, const std::string& phase,
                   const std::string& message);

  DetectorConfig detector_config_;
  CaptureConfig capture_config_;
  DisplayConfig display_config_;
  RgbdDetector detector_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::CallbackGroup::SharedPtr input_group_;
  rclcpp::CallbackGroup::SharedPtr service_group_;
  message_filters::Subscriber<sensor_msgs::msg::Image> rgb_subscriber_;
  message_filters::Subscriber<sensor_msgs::msg::Image> depth_subscriber_;
  std::unique_ptr<Synchronizer> synchronizer_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr display_publisher_;
  rclcpp::Service<DetectObjects>::SharedPtr service_;

  std::mutex input_mutex_;
  std::condition_variable input_condition_;
  SynchronizedPair synchronized_pair_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr latest_camera_info_;

  std::mutex display_mutex_;
  std::vector<double> latest_arm_joint_positions_;
  std::vector<double> overlay_arm_joint_positions_;
  std::vector<DetectedCube> display_overlay_;
  bool complete_joint_state_received_ = false;
};

}  // namespace sorting_arm_perception

#endif  // SORTING_ARM_PERCEPTION__PERCEPTION_NODE_HPP_
