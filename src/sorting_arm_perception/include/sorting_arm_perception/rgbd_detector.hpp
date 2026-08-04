#ifndef SORTING_ARM_PERCEPTION__RGBD_DETECTOR_HPP_
#define SORTING_ARM_PERCEPTION__RGBD_DETECTOR_HPP_

#include <Eigen/Geometry>
#include <array>
#include <cstddef>
#include <image_geometry/pinhole_camera_model.hpp>
#include <opencv2/core.hpp>
#include <string>
#include <vector>

namespace sorting_arm_perception {

struct HsvRange {
  int minimum_hue = 0;
  int maximum_hue = 0;
};

struct DetectorConfig {
  std::size_t expected_object_count = 0;
  std::array<double, 3> cube_dimensions{};
  Eigen::Vector3d source_min = Eigen::Vector3d::Zero();
  Eigen::Vector3d source_max = Eigen::Vector3d::Zero();
  HsvRange red_low;
  HsvRange red_high;
  HsvRange blue;
  int minimum_saturation = 0;
  int minimum_value = 0;
  double minimum_contour_area = 0.0;
  std::size_t minimum_top_face_area = 0;
  double minimum_valid_depth_fraction = 0.0;
  double top_depth_percentile = 0.0;
  double top_depth_tolerance = 0.0;
  double physical_size_tolerance = 0.0;
};

struct DetectedCube {
  std::string id;
  std::string label;
  Eigen::Vector3d centre = Eigen::Vector3d::Zero();
  std::array<cv::Point2f, 4> image_corners{};
  cv::Point2f image_centroid;
};

struct DetectorResult {
  bool ok = false;
  std::string phase;
  std::string message;
  std::vector<DetectedCube> cubes;
  cv::Mat debug_image;
};

class RgbdDetector {
 public:
  DetectorResult detect(const cv::Mat& rgb_image, const cv::Mat& depth_image,
                        const image_geometry::PinholeCameraModel& camera_model, const Eigen::Isometry3d& world_from_camera,
                        bool rectify_pixels, const DetectorConfig& config) const;
  void draw_detections(cv::Mat& rgb_image, const std::vector<DetectedCube>& cubes) const;
};

}  // namespace sorting_arm_perception

#endif  // SORTING_ARM_PERCEPTION__RGBD_DETECTOR_HPP_
