#include "sorting_arm_perception/perception_node.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cv_bridge/cv_bridge.hpp>
#include <functional>
#include <iterator>
#include <limits>
#include <memory>
#include <opencv2/imgproc.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/distortion_models.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <stdexcept>
#include <string>
#include <tf2/exceptions.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <vector>

namespace sorting_arm_perception {
namespace {

bool finite_values(const std::vector<double>& values) {
  return std::all_of(values.begin(), values.end(), [](double value) { return std::isfinite(value); });
}

bool ordered_hue_range(const HsvRange& range) {
  return range.minimum_hue >= 0 && range.maximum_hue <= 179 && range.minimum_hue <= range.maximum_hue;
}

std::int64_t timestamp_difference(const builtin_interfaces::msg::Time& first, const builtin_interfaces::msg::Time& second) {
  const rclcpp::Time first_time(first, RCL_ROS_TIME);
  const rclcpp::Time second_time(second, RCL_ROS_TIME);
  if (first_time >= second_time) {
    return (first_time - second_time).nanoseconds();
  }
  return (second_time - first_time).nanoseconds();
}

}  // namespace

PerceptionNode::PerceptionNode()
    : Node("camera_object_provider"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_, this, true) {
  load_parameters();

  input_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  service_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  rclcpp::SubscriptionOptions input_options;
  input_options.callback_group = input_group_;
  const rclcpp::SensorDataQoS sensor_qos;
  rgb_subscriber_.subscribe(this, "/camera/image_raw", sensor_qos.get_rmw_qos_profile(), input_options);
  depth_subscriber_.subscribe(this, "/camera/depth/image_raw", sensor_qos.get_rmw_qos_profile(), input_options);

  ApproximateTimePolicy policy(5);
  policy.setMaxIntervalDuration(rclcpp::Duration::from_seconds(capture_config_.maximum_pairing_interval_seconds));
  // we cast policy const so Jazzy selects policy plus two inputs, not the
  // three-input overload that treats policy as a filter
  synchronizer_ =
      std::make_unique<Synchronizer>(static_cast<const ApproximateTimePolicy&>(policy), rgb_subscriber_, depth_subscriber_);
  synchronizer_->registerCallback(
      std::bind(&PerceptionNode::synchronized_callback, this, std::placeholders::_1, std::placeholders::_2));

  camera_info_subscription_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera/camera_info", sensor_qos, std::bind(&PerceptionNode::camera_info_callback, this, std::placeholders::_1),
      input_options);
  if (display_config_.show_viewer) {
    joint_state_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", sensor_qos, std::bind(&PerceptionNode::joint_state_callback, this, std::placeholders::_1),
        input_options);
    viewer_ = std::make_unique<PerceptionViewer>(display_config_.viewer_scale);
  }

  const rclcpp::QoS debug_qos = rclcpp::QoS(1).reliable().durability_volatile();
  debug_publisher_ = create_publisher<sensor_msgs::msg::Image>("/perception/debug_image", debug_qos);
  service_ = create_service<DetectObjects>(
      "detect_objects", std::bind(&PerceptionNode::detect, this, std::placeholders::_1, std::placeholders::_2),
      rclcpp::ServicesQoS(), service_group_);
}

void PerceptionNode::load_parameters() {
  const auto cube_dimensions = declare_parameter<std::vector<double>>("cube_dimensions");
  if (cube_dimensions.size() != 3 || !finite_values(cube_dimensions) ||
      std::any_of(cube_dimensions.begin(), cube_dimensions.end(), [](double value) { return value <= 0.0; })) {
    throw std::runtime_error("cube_dimensions must contain three positive finite values");
  }
  detector_config_.cube_dimensions = {cube_dimensions[0], cube_dimensions[1], cube_dimensions[2]};

  const auto source_min = declare_parameter<std::vector<double>>("source_area.minimum");
  const auto source_max = declare_parameter<std::vector<double>>("source_area.maximum");
  if (source_min.size() != 3 || source_max.size() != 3 || !finite_values(source_min) || !finite_values(source_max)) {
    throw std::runtime_error("source area bounds must contain three finite values");
  }
  detector_config_.source_min = Eigen::Vector3d(source_min[0], source_min[1], source_min[2]);
  detector_config_.source_max = Eigen::Vector3d(source_max[0], source_max[1], source_max[2]);
  if ((detector_config_.source_min.array() >= detector_config_.source_max.array()).any()) {
    throw std::runtime_error("source_area.minimum must be below source_area.maximum");
  }

  const auto exclusion_min_x = declare_parameter<std::vector<double>>("excluded_areas.minimum_x");
  const auto exclusion_min_y = declare_parameter<std::vector<double>>("excluded_areas.minimum_y");
  const auto exclusion_max_x = declare_parameter<std::vector<double>>("excluded_areas.maximum_x");
  const auto exclusion_max_y = declare_parameter<std::vector<double>>("excluded_areas.maximum_y");
  const std::size_t exclusion_count = exclusion_min_x.size();
  if (exclusion_min_y.size() != exclusion_count || exclusion_max_x.size() != exclusion_count ||
      exclusion_max_y.size() != exclusion_count) {
    throw std::runtime_error("excluded_areas coordinate lists must have equal sizes");
  }
  for (std::size_t index = 0; index < exclusion_count; ++index) {
    const ExclusionArea area{Eigen::Vector2d(exclusion_min_x[index], exclusion_min_y[index]),
                             Eigen::Vector2d(exclusion_max_x[index], exclusion_max_y[index])};
    if (!area.minimum.allFinite() || !area.maximum.allFinite() || area.minimum.x() >= area.maximum.x() ||
        area.minimum.y() >= area.maximum.y()) {
      throw std::runtime_error("excluded_areas entry must have finite increasing bounds");
    }
    detector_config_.exclusion_areas.push_back(area);
  }

  const auto load_hue_range = [this](const std::string& name) {
    const auto values = declare_parameter<std::vector<std::int64_t>>(name);
    if (values.size() != 2) {
      throw std::runtime_error(name + " must contain minimum and maximum hue");
    }
    const HsvRange range{static_cast<int>(values[0]), static_cast<int>(values[1])};
    if (!ordered_hue_range(range)) {
      throw std::runtime_error(name + " must be ordered within [0, 179]");
    }
    return range;
  };
  detector_config_.red_low = load_hue_range("colour.red_low_hue");
  detector_config_.red_high = load_hue_range("colour.red_high_hue");
  detector_config_.blue = load_hue_range("colour.blue_hue");

  const std::int64_t minimum_saturation = declare_parameter<std::int64_t>("colour.minimum_saturation");
  const std::int64_t minimum_value = declare_parameter<std::int64_t>("colour.minimum_value");
  if (minimum_saturation < 0 || minimum_saturation > 255 || minimum_value < 0 || minimum_value > 255) {
    throw std::runtime_error("colour saturation and value limits must be within [0, 255]");
  }
  detector_config_.minimum_saturation = static_cast<int>(minimum_saturation);
  detector_config_.minimum_value = static_cast<int>(minimum_value);

  const std::int64_t tuned_image_width = declare_parameter<std::int64_t>("detector.tuned_image_width");
  const std::int64_t tuned_image_height = declare_parameter<std::int64_t>("detector.tuned_image_height");
  if (tuned_image_width <= 0 || tuned_image_height <= 0) {
    throw std::runtime_error("detector.tuned_image_width and tuned_image_height must be positive");
  }
  detector_config_.tuned_image_width = static_cast<std::size_t>(tuned_image_width);
  detector_config_.tuned_image_height = static_cast<std::size_t>(tuned_image_height);

  detector_config_.minimum_contour_area = declare_parameter<double>("detector.minimum_contour_area");
  const std::int64_t minimum_top_face_area = declare_parameter<std::int64_t>("detector.minimum_top_face_area");
  detector_config_.minimum_valid_depth_fraction = declare_parameter<double>("detector.minimum_valid_depth_fraction");
  detector_config_.top_depth_percentile = declare_parameter<double>("detector.top_depth_percentile");
  detector_config_.top_depth_tolerance = declare_parameter<double>("detector.top_depth_tolerance");
  detector_config_.physical_size_tolerance = declare_parameter<double>("detector.physical_size_tolerance");
  if (!std::isfinite(detector_config_.minimum_contour_area) || detector_config_.minimum_contour_area <= 0.0 ||
      minimum_top_face_area <= 0) {
    throw std::runtime_error("detector pixel-area limits must be positive");
  }
  detector_config_.minimum_top_face_area = static_cast<std::size_t>(minimum_top_face_area);
  if (!std::isfinite(detector_config_.minimum_valid_depth_fraction) ||
      detector_config_.minimum_valid_depth_fraction <= 0.0 || detector_config_.minimum_valid_depth_fraction > 1.0) {
    throw std::runtime_error("detector.minimum_valid_depth_fraction must be within (0, 1]");
  }
  if (!std::isfinite(detector_config_.top_depth_percentile) || detector_config_.top_depth_percentile < 0.0 ||
      detector_config_.top_depth_percentile > 1.0) {
    throw std::runtime_error("detector.top_depth_percentile must be within [0, 1]");
  }
  if (!std::isfinite(detector_config_.top_depth_tolerance) || detector_config_.top_depth_tolerance <= 0.0 ||
      !std::isfinite(detector_config_.physical_size_tolerance) || detector_config_.physical_size_tolerance <= 0.0) {
    throw std::runtime_error("detector metric tolerances must be positive and finite");
  }

  capture_config_.registration_verified = declare_parameter<bool>("registration.verified");
  capture_config_.maximum_pairing_interval_seconds = declare_parameter<double>("synchronization.maximum_interval");
  capture_config_.capture_timeout_seconds = declare_parameter<double>("capture_timeout");
  capture_config_.transform_timeout_seconds = declare_parameter<double>("transform_timeout");
  if (!std::isfinite(capture_config_.maximum_pairing_interval_seconds) ||
      capture_config_.maximum_pairing_interval_seconds <= 0.0 || !std::isfinite(capture_config_.capture_timeout_seconds) ||
      capture_config_.capture_timeout_seconds <= 0.0 || !std::isfinite(capture_config_.transform_timeout_seconds) ||
      capture_config_.transform_timeout_seconds <= 0.0) {
    throw std::runtime_error("synchronization and capture timeouts must be positive and finite");
  }

  display_config_.show_viewer = declare_parameter<bool>("show_viewer", false);
  display_config_.motion_threshold_radians = declare_parameter<double>("display.motion_threshold_radians");
  if (!std::isfinite(display_config_.motion_threshold_radians) || display_config_.motion_threshold_radians <= 0.0) {
    throw std::runtime_error("display.motion_threshold_radians must be positive and finite");
  }

  display_config_.viewer_scale = declare_parameter<double>("display.viewer_scale", 1.0);
  if (!std::isfinite(display_config_.viewer_scale) || display_config_.viewer_scale <= 0.0) {
    throw std::runtime_error("display.viewer_scale must be positive and finite");
  }
}

void PerceptionNode::synchronized_callback(sensor_msgs::msg::Image::ConstSharedPtr rgb,
                                           sensor_msgs::msg::Image::ConstSharedPtr depth) {
  {
    std::lock_guard<std::mutex> lock(input_mutex_);
    synchronized_pair_.rgb = rgb;
    synchronized_pair_.depth = depth;
    synchronized_pair_.capture_stamp = rgb->header.stamp;
    synchronized_pair_.timestamp_difference_nanoseconds = timestamp_difference(rgb->header.stamp, depth->header.stamp);
    ++synchronized_pair_.sequence;
  }
  input_condition_.notify_all();
  update_viewer(rgb);
}

void PerceptionNode::camera_info_callback(sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info) {
  std::string error;
  if (!validate_camera_info(*camera_info, error)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "ignoring invalid CameraInfo: %s", error.c_str());
    return;
  }

  std::lock_guard<std::mutex> lock(input_mutex_);
  if (latest_camera_info_ == nullptr || rclcpp::Time(camera_info->header.stamp, get_clock()->get_clock_type()) >
                                            rclcpp::Time(latest_camera_info_->header.stamp, get_clock()->get_clock_type())) {
    latest_camera_info_ = camera_info;
  }
}

void PerceptionNode::joint_state_callback(sensor_msgs::msg::JointState::ConstSharedPtr joint_state) {
  std::vector<double> arm_positions;
  for (const std::string& joint_name : arm_joint_names_) {
    const auto name = std::find(joint_state->name.begin(), joint_state->name.end(), joint_name);
    if (name == joint_state->name.end()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "joint state is missing display arm joint '%s'",
                           joint_name.c_str());
      return;
    }
    const auto index = static_cast<std::size_t>(std::distance(joint_state->name.begin(), name));
    if (index >= joint_state->position.size() || !std::isfinite(joint_state->position[index])) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "joint state has no finite position for display arm joint '%s'",
                           joint_name.c_str());
      return;
    }
    arm_positions.push_back(joint_state->position[index]);
  }

  std::lock_guard<std::mutex> lock(display_mutex_);
  latest_arm_joint_positions_ = arm_positions;
  complete_joint_state_received_ = true;
  if (display_overlay_.empty()) {
    return;
  }
  for (std::size_t index = 0; index < arm_positions.size(); ++index) {
    if (std::abs(arm_positions[index] - overlay_arm_joint_positions_[index]) > display_config_.motion_threshold_radians) {
      display_overlay_.clear();
      overlay_arm_joint_positions_.clear();
      RCLCPP_INFO(get_logger(), "cleared detection boxes because the camera arm moved");
      return;
    }
  }
}

bool PerceptionNode::validate_camera_info(const sensor_msgs::msg::CameraInfo& camera_info, std::string& error) const {
  if (camera_info.width == 0 || camera_info.height == 0) {
    error = "image dimensions are zero";
    return false;
  }
  if (camera_info.width != detector_config_.tuned_image_width || camera_info.height != detector_config_.tuned_image_height) {
    error = "pixel-area limits are tuned for " + std::to_string(detector_config_.tuned_image_width) + "x" +
            std::to_string(detector_config_.tuned_image_height) + ", stream is " + std::to_string(camera_info.width) + "x" +
            std::to_string(camera_info.height);
    return false;
  }
  if (camera_info.header.frame_id.empty()) {
    error = "optical frame is empty";
    return false;
  }
  const auto finite_array = [](const auto& values) {
    return std::all_of(values.begin(), values.end(), [](double value) { return std::isfinite(value); });
  };
  if (!finite_array(camera_info.k) || !finite_array(camera_info.r) || !finite_array(camera_info.p) ||
      !finite_values(camera_info.d)) {
    error = "calibration contains non-finite values";
    return false;
  }
  if (camera_info.k[0] <= 0.0 || camera_info.k[4] <= 0.0 || camera_info.p[0] <= 0.0 || camera_info.p[5] <= 0.0) {
    error = "focal values must be positive";
    return false;
  }

  std::size_t minimum_coefficients = 0;
  if (camera_info.distortion_model == sensor_msgs::distortion_models::PLUMB_BOB) {
    minimum_coefficients = 5;
  } else if (camera_info.distortion_model == sensor_msgs::distortion_models::RATIONAL_POLYNOMIAL) {
    minimum_coefficients = 8;
  } else if (camera_info.distortion_model == sensor_msgs::distortion_models::EQUIDISTANT) {
    minimum_coefficients = 4;
  } else {
    error = "unsupported distortion model '" + camera_info.distortion_model + "'";
    return false;
  }
  if (camera_info.d.size() < minimum_coefficients) {
    error = "distortion coefficient vector is too short for " + camera_info.distortion_model;
    return false;
  }

  image_geometry::PinholeCameraModel camera_model;
  if (!camera_model.fromCameraInfo(camera_info) || !camera_model.initialized() || !std::isfinite(camera_model.fx()) ||
      camera_model.fx() <= 0.0 || !std::isfinite(camera_model.fy()) || camera_model.fy() <= 0.0) {
    error = "PinholeCameraModel could not initialize";
    return false;
  }
  return true;
}

bool PerceptionNode::validate_projection_contract(const SynchronizedPair& pair,
                                                  const sensor_msgs::msg::CameraInfo& camera_info,
                                                  image_geometry::PinholeCameraModel& camera_model, bool& rectify_pixels,
                                                  std::string& phase, std::string& error) const {
  if (!capture_config_.registration_verified) {
    phase = "registration";
    error =
        "RGB-depth ray registration has not been verified; keep registration.verified false until runtime evidence exists";
    return false;
  }
  if (pair.rgb->header.frame_id.empty() || pair.depth->header.frame_id.empty() ||
      pair.rgb->header.frame_id != pair.depth->header.frame_id || pair.rgb->header.frame_id != camera_info.header.frame_id) {
    phase = "registration";
    error = "RGB, registered depth, and CameraInfo must use one optical frame";
    return false;
  }
  if (pair.rgb->width != pair.depth->width || pair.rgb->height != pair.depth->height ||
      pair.rgb->width != camera_info.width || pair.rgb->height != camera_info.height) {
    phase = "registration";
    error = "RGB, registered depth, and CameraInfo dimensions do not match";
    return false;
  }
  if (camera_info.binning_x > 1 || camera_info.binning_y > 1 || camera_info.roi.x_offset != 0 ||
      camera_info.roi.y_offset != 0 || (camera_info.roi.width != 0 && camera_info.roi.width != camera_info.width) ||
      (camera_info.roi.height != 0 && camera_info.roi.height != camera_info.height)) {
    phase = "projection";
    error = "CameraInfo binning or ROI state is outside the raw-image projection contract";
    return false;
  }
  if (!camera_model.fromCameraInfo(camera_info) || !camera_model.initialized()) {
    phase = "camera_info";
    error = "PinholeCameraModel could not initialize from retained CameraInfo";
    return false;
  }
  rectify_pixels = std::any_of(camera_info.d.begin(), camera_info.d.end(), [](double value) { return value != 0.0; });
  return true;
}

void PerceptionNode::detect(const std::shared_ptr<DetectObjects::Request> request,
                            std::shared_ptr<DetectObjects::Response> response) {
  response->objects.clear();
  clear_display_overlay();
  if (request->expected_count == 0U) {
    fail(response, "object_count", "expected_count must be positive");
    return;
  }
  const rclcpp::Time request_time = now();

  SynchronizedPair pair;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info;
  {
    std::unique_lock<std::mutex> lock(input_mutex_);
    const std::uint64_t request_sequence = synchronized_pair_.sequence;
    const auto fresh_pair_ready = [this, request_sequence, request_time] {
      return synchronized_pair_.sequence > request_sequence && synchronized_pair_.rgb != nullptr &&
             rclcpp::Time(synchronized_pair_.capture_stamp, get_clock()->get_clock_type()) >= request_time;
    };
    if (!input_condition_.wait_for(lock, std::chrono::duration<double>(capture_config_.capture_timeout_seconds),
                                   fresh_pair_ready)) {
      fail(response, "capture", "timed out waiting for a post-request synchronized RGB-D pair");
      return;
    }
    pair = synchronized_pair_;
    camera_info = latest_camera_info_;
  }

  const auto maximum_difference =
      static_cast<std::int64_t>(capture_config_.maximum_pairing_interval_seconds * 1'000'000'000.0);
  RCLCPP_INFO(get_logger(), "fresh RGB-D pair has %.6f s timestamp difference",
              static_cast<double>(pair.timestamp_difference_nanoseconds) / 1'000'000'000.0);
  if (pair.timestamp_difference_nanoseconds > maximum_difference) {
    fail(response, "capture", "synchronized RGB-D pair exceeds maximum timestamp interval");
    return;
  }
  if (pair.rgb->encoding != sensor_msgs::image_encodings::RGB8) {
    fail(response, "encoding", "RGB image encoding must be rgb8, got " + pair.rgb->encoding);
    return;
  }
  if (pair.depth->encoding != sensor_msgs::image_encodings::TYPE_32FC1) {
    fail(response, "encoding", "depth image encoding must be 32FC1 metres, got " + pair.depth->encoding);
    return;
  }
  if (pair.rgb->width != pair.depth->width || pair.rgb->height != pair.depth->height) {
    fail(response, "registration", "RGB and depth dimensions do not match");
    return;
  }
  if (camera_info == nullptr) {
    fail(response, "camera_info", "no valid CameraInfo has been received");
    return;
  }

  cv_bridge::CvImageConstPtr rgb_bridge;
  cv_bridge::CvImageConstPtr depth_bridge;
  try {
    rgb_bridge = cv_bridge::toCvShare(pair.rgb, sensor_msgs::image_encodings::RGB8);
    depth_bridge = cv_bridge::toCvShare(pair.depth, sensor_msgs::image_encodings::TYPE_32FC1);
  } catch (const cv_bridge::Exception& exception) {
    fail(response, "encoding", std::string("image conversion failed: ") + exception.what());
    return;
  }

  image_geometry::PinholeCameraModel camera_model;
  bool rectify_pixels = false;
  std::string phase;
  std::string error;
  if (!validate_projection_contract(pair, *camera_info, camera_model, rectify_pixels, phase, error)) {
    publish_debug_failure(pair.rgb->header, rgb_bridge->image, phase, error);
    fail(response, phase, error);
    return;
  }

  geometry_msgs::msg::TransformStamped transform;
  try {
    transform = tf_buffer_.lookupTransform("world", pair.rgb->header.frame_id,
                                           rclcpp::Time(pair.capture_stamp, get_clock()->get_clock_type()),
                                           rclcpp::Duration::from_seconds(capture_config_.transform_timeout_seconds));
  } catch (const tf2::TransformException& exception) {
    const std::string message = std::string("exact capture-time transform failed: ") + exception.what();
    publish_debug_failure(pair.rgb->header, rgb_bridge->image, "transform", message);
    fail(response, "transform", message);
    return;
  }

  DetectorResult detector_result;
  try {
    detector_result =
        detector_.detect(rgb_bridge->image, depth_bridge->image, camera_model, tf2::transformToEigen(transform),
                         rectify_pixels, request->expected_count, detector_config_);
  } catch (const cv::Exception& exception) {
    const std::string message = std::string("OpenCV detector failure: ") + exception.what();
    publish_debug_failure(pair.rgb->header, rgb_bridge->image, "segmentation", message);
    fail(response, "segmentation", message);
    return;
  }

  publish_debug(pair.rgb->header, detector_result.debug_image);
  if (!detector_result.ok) {
    fail(response, detector_result.phase, detector_result.message);
    return;
  }

  for (const DetectedCube& cube : detector_result.cubes) {
    sorting_arm_interfaces::msg::DetectedObject object;
    object.id = cube.id;
    object.label = cube.label;
    object.centre.header.stamp = pair.capture_stamp;
    object.centre.header.frame_id = "world";
    object.centre.pose.position.x = cube.centre.x();
    object.centre.pose.position.y = cube.centre.y();
    object.centre.pose.position.z = cube.centre.z();
    object.centre.pose.orientation.w = 1.0;
    object.primitive_type = shape_msgs::msg::SolidPrimitive::BOX;
    for (const double dimension : detector_config_.cube_dimensions) {
      object.dimensions.push_back(dimension);
    }
    response->objects.push_back(object);
  }
  response->result.ok = true;
  response->result.native_code = 0;
  response->result.phase = detector_result.phase;
  response->result.message = detector_result.message;
  activate_display_overlay(detector_result.cubes);
}

void PerceptionNode::publish_debug(const std_msgs::msg::Header& header, const cv::Mat& image) const {
  if (image.empty()) {
    return;
  }
  const auto message = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, image).toImageMsg();
  debug_publisher_->publish(*message);
}

void PerceptionNode::publish_debug_failure(const std_msgs::msg::Header& header, const cv::Mat& rgb_image,
                                           const std::string& phase, const std::string& message) const {
  cv::Mat debug_image = rgb_image.clone();
  cv::putText(debug_image, phase + ": " + message, cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 0),
              2, cv::LINE_AA);
  publish_debug(header, debug_image);
}

void PerceptionNode::update_viewer(const sensor_msgs::msg::Image::ConstSharedPtr& rgb) {
  if (viewer_ == nullptr || !viewer_->active()) {
    return;
  }
  if (rgb->encoding != sensor_msgs::image_encodings::RGB8) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "display requires rgb8 input, got '%s'", rgb->encoding.c_str());
    return;
  }

  cv_bridge::CvImageConstPtr rgb_image;
  try {
    rgb_image = cv_bridge::toCvShare(rgb, sensor_msgs::image_encodings::RGB8);
  } catch (const cv_bridge::Exception& exception) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "viewer image conversion failed: %s", exception.what());
    return;
  }

  std::vector<DetectedCube> overlay;
  {
    std::lock_guard<std::mutex> lock(display_mutex_);
    overlay = display_overlay_;
  }
  if (overlay.empty()) {
    try {
      viewer_->show_rgb(rgb_image->image);
    } catch (const cv::Exception& exception) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "viewer conversion failed: %s", exception.what());
    }
    return;
  }

  try {
    cv::Mat display_image = rgb_image->image.clone();
    detector_.draw_detections(display_image, overlay);
    viewer_->show_rgb(display_image);
  } catch (const cv::Exception& exception) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "viewer annotation failed: %s", exception.what());
  }
}

void PerceptionNode::clear_display_overlay() {
  std::lock_guard<std::mutex> lock(display_mutex_);
  display_overlay_.clear();
  overlay_arm_joint_positions_.clear();
}

void PerceptionNode::activate_display_overlay(const std::vector<DetectedCube>& cubes) {
  if (viewer_ == nullptr || !viewer_->active()) {
    return;
  }
  std::lock_guard<std::mutex> lock(display_mutex_);
  if (!complete_joint_state_received_) {
    RCLCPP_WARN(get_logger(), "detection succeeded, but boxes cannot persist without a complete arm joint state");
    return;
  }
  display_overlay_ = cubes;
  overlay_arm_joint_positions_ = latest_arm_joint_positions_;
}

void PerceptionNode::fail(const std::shared_ptr<DetectObjects::Response>& response, const std::string& phase,
                          const std::string& message) {
  response->objects.clear();
  response->result.ok = false;
  response->result.native_code = 0;
  response->result.phase = phase;
  response->result.message = message;
}

}  // namespace sorting_arm_perception
