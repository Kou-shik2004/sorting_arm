#include "sorting_arm_perception/rgbd_detector.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <opencv2/imgproc.hpp>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace sorting_arm_perception {
namespace {

struct Candidate {
  std::string label;
  std::vector<cv::Point> contour;
};

struct DepthPixel {
  cv::Point pixel;
  float depth = 0.0F;
};

std::string format_value(double value) {
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(2) << value;
  return stream.str();
}

void annotate_candidate(cv::Mat& debug_image, const std::vector<cv::Point>& contour, const std::string& text,
                        const cv::Scalar& colour) {
  const std::vector<std::vector<cv::Point>> contours{contour};
  cv::drawContours(debug_image, contours, 0, colour, 2);
  const cv::Rect bounds = cv::boundingRect(contour);
  const cv::Point origin(bounds.x, std::max(15, bounds.y - 5));
  cv::putText(debug_image, text, origin, cv::FONT_HERSHEY_SIMPLEX, 0.45, colour, 1, cv::LINE_AA);
}

double median(std::vector<float> values) {
  std::sort(values.begin(), values.end());
  const std::size_t middle = values.size() / 2;
  if (values.size() % 2 == 1) {
    return values[middle];
  }
  return (static_cast<double>(values[middle - 1]) + static_cast<double>(values[middle])) / 2.0;
}

bool project_pixel(const cv::Point2d& raw_pixel, double depth, const image_geometry::PinholeCameraModel& camera_model,
                   bool rectify_pixels, Eigen::Vector3d& camera_point, std::string& error) {
  try {
    const cv::Point2d projection_pixel = rectify_pixels ? camera_model.rectifyPoint(raw_pixel) : raw_pixel;
    const cv::Point3d ray = camera_model.projectPixelTo3dRay(projection_pixel);
    if (!std::isfinite(ray.x) || !std::isfinite(ray.y) || !std::isfinite(ray.z) || std::abs(ray.z) < 1e-12) {
      error = "camera model returned an invalid projection ray";
      return false;
    }
    // depth is optical Z in metres, so we scale the ray by Z / ray.z
    camera_point = Eigen::Vector3d(ray.x, ray.y, ray.z) * (depth / ray.z);
  } catch (const cv::Exception& exception) {
    error = std::string("pixel rectification or projection failed: ") + exception.what();
    return false;
  }

  if (!camera_point.allFinite()) {
    error = "projected camera point is not finite";
    return false;
  }
  return true;
}

bool inside_source_area(const Eigen::Vector3d& centre, const DetectorConfig& config) {
  return centre.x() >= config.source_min.x() && centre.x() <= config.source_max.x() && centre.y() >= config.source_min.y() &&
         centre.y() <= config.source_max.y() && centre.z() >= config.source_min.z() && centre.z() <= config.source_max.z();
}

bool inside_exclusion_area(const Eigen::Vector3d& centre, const DetectorConfig& config) {
  for (const auto& area : config.exclusion_areas) {
    if (centre.x() >= area.minimum.x() && centre.x() <= area.maximum.x() && centre.y() >= area.minimum.y() &&
        centre.y() <= area.maximum.y()) {
      return true;
    }
  }
  return false;
}

bool physical_size_matches(const std::array<Eigen::Vector3d, 4>& corners, const DetectorConfig& config, double& first_side,
                           double& second_side) {
  first_side = (corners[1] - corners[0]).norm();
  second_side = (corners[2] - corners[1]).norm();
  std::array<double, 2> measured{first_side, second_side};
  std::array<double, 2> expected{config.cube_dimensions[0], config.cube_dimensions[1]};
  std::sort(measured.begin(), measured.end());
  std::sort(expected.begin(), expected.end());
  return std::abs(measured[0] - expected[0]) <= config.physical_size_tolerance &&
         std::abs(measured[1] - expected[1]) <= config.physical_size_tolerance;
}

cv::Scalar label_colour(const std::string& label) {
  if (label == "red") {
    return cv::Scalar(255, 0, 0);
  }
  return cv::Scalar(0, 0, 255);
}

void draw_detection_label(cv::Mat& image, const DetectedCube& cube, const cv::Scalar& colour) {
  std::vector<cv::Point2f> corners(cube.image_corners.begin(), cube.image_corners.end());
  const cv::Rect bounds = cv::boundingRect(corners);
  const std::string text = cube.id.empty() ? cube.label + " accepted" : cube.id + " " + cube.label;
  constexpr double font_scale = 0.65;
  constexpr int thickness = 2;
  int baseline = 0;
  const cv::Size text_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, font_scale, thickness, &baseline);

  int text_x = bounds.x;
  int text_y = bounds.y - 8;
  if (text_y - text_size.height - baseline < 0) {
    text_y = bounds.y + text_size.height + baseline + 8;
  }
  text_x = std::clamp(text_x, 0, std::max(0, image.cols - text_size.width - 8));
  text_y = std::clamp(text_y, text_size.height + baseline + 4, image.rows - 4);

  const cv::Rect background(text_x, text_y - text_size.height - baseline - 4, text_size.width + 8,
                            text_size.height + baseline + 8);
  cv::rectangle(image, background, cv::Scalar(0, 0, 0), cv::FILLED);
  cv::putText(image, text, cv::Point(text_x + 4, text_y), cv::FONT_HERSHEY_SIMPLEX, font_scale, cv::Scalar(255, 255, 255),
              thickness, cv::LINE_AA);

  for (std::size_t index = 0; index < cube.image_corners.size(); ++index) {
    const std::size_t next = (index + 1) % cube.image_corners.size();
    cv::line(image, cube.image_corners[index], cube.image_corners[next], colour, thickness, cv::LINE_AA);
  }
  cv::circle(image, cube.image_centroid, 4, colour, cv::FILLED, cv::LINE_AA);
}

}  // namespace

void RgbdDetector::draw_detections(cv::Mat& rgb_image, const std::vector<DetectedCube>& cubes) const {
  for (const DetectedCube& cube : cubes) {
    draw_detection_label(rgb_image, cube, label_colour(cube.label));
  }
}

DetectorResult RgbdDetector::detect(const cv::Mat& rgb_image, const cv::Mat& depth_image,
                                    const image_geometry::PinholeCameraModel& camera_model,
                                    const Eigen::Isometry3d& world_from_camera, bool rectify_pixels,
                                    std::size_t expected_count, const DetectorConfig& config) const {
  DetectorResult result;
  result.debug_image = rgb_image.clone();

  cv::Mat hsv_image;
  cv::cvtColor(rgb_image, hsv_image, cv::COLOR_RGB2HSV);

  const cv::Scalar red_low_minimum(config.red_low.minimum_hue, config.minimum_saturation, config.minimum_value);
  const cv::Scalar red_low_maximum(config.red_low.maximum_hue, 255, 255);
  const cv::Scalar red_high_minimum(config.red_high.minimum_hue, config.minimum_saturation, config.minimum_value);
  const cv::Scalar red_high_maximum(config.red_high.maximum_hue, 255, 255);
  const cv::Scalar blue_minimum(config.blue.minimum_hue, config.minimum_saturation, config.minimum_value);
  const cv::Scalar blue_maximum(config.blue.maximum_hue, 255, 255);

  cv::Mat red_low_mask;
  cv::Mat red_high_mask;
  cv::Mat red_mask;
  cv::Mat blue_mask;
  cv::inRange(hsv_image, red_low_minimum, red_low_maximum, red_low_mask);
  cv::inRange(hsv_image, red_high_minimum, red_high_maximum, red_high_mask);
  cv::bitwise_or(red_low_mask, red_high_mask, red_mask);
  cv::inRange(hsv_image, blue_minimum, blue_maximum, blue_mask);

  std::vector<Candidate> candidates;
  const auto collect_candidates = [&candidates](const cv::Mat& mask, const std::string& label) {
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for (const auto& contour : contours) {
      candidates.push_back(Candidate{label, contour});
    }
  };
  collect_candidates(red_mask, "red");
  collect_candidates(blue_mask, "blue");

  if (candidates.empty()) {
    result.phase = "segmentation";
    result.message = "no red or blue contours found";
    cv::putText(result.debug_image, result.message, cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                cv::Scalar(255, 255, 0), 2, cv::LINE_AA);
    return result;
  }

  std::vector<std::string> rejections;
  for (const Candidate& candidate : candidates) {
    const double contour_area = cv::contourArea(candidate.contour);
    if (contour_area < config.minimum_contour_area) {
      const std::string reason = candidate.label + ": contour area " + format_value(contour_area) + " px";
      rejections.push_back(reason);
      annotate_candidate(result.debug_image, candidate.contour, reason, cv::Scalar(255, 255, 0));
      continue;
    }

    cv::Mat candidate_mask = cv::Mat::zeros(depth_image.size(), CV_8UC1);
    const std::vector<std::vector<cv::Point>> one_contour{candidate.contour};
    cv::drawContours(candidate_mask, one_contour, 0, cv::Scalar(255), cv::FILLED);
    const int candidate_pixel_count = cv::countNonZero(candidate_mask);
    if (candidate_pixel_count <= 0) {
      const std::string reason = candidate.label + ": empty contour mask";
      rejections.push_back(reason);
      annotate_candidate(result.debug_image, candidate.contour, reason, cv::Scalar(255, 255, 0));
      continue;
    }

    std::vector<DepthPixel> valid_pixels;
    const cv::Rect bounds = cv::boundingRect(candidate.contour);
    for (int row = bounds.y; row < bounds.y + bounds.height; ++row) {
      for (int column = bounds.x; column < bounds.x + bounds.width; ++column) {
        if (candidate_mask.at<std::uint8_t>(row, column) == 0) {
          continue;
        }
        const float depth = depth_image.at<float>(row, column);
        if (std::isfinite(depth) && depth > 0.0F) {
          valid_pixels.push_back(DepthPixel{cv::Point(column, row), depth});
        }
      }
    }

    const double valid_fraction = static_cast<double>(valid_pixels.size()) / static_cast<double>(candidate_pixel_count);
    if (valid_fraction < config.minimum_valid_depth_fraction) {
      const std::string reason = candidate.label + ": valid depth " + format_value(valid_fraction);
      rejections.push_back(reason);
      annotate_candidate(result.debug_image, candidate.contour, reason, cv::Scalar(255, 255, 0));
      continue;
    }

    std::vector<float> valid_depths;
    for (const DepthPixel& sample : valid_pixels) {
      valid_depths.push_back(sample.depth);
    }
    std::sort(valid_depths.begin(), valid_depths.end());
    const std::size_t percentile_index =
        static_cast<std::size_t>(std::floor(config.top_depth_percentile * static_cast<double>(valid_depths.size() - 1)));
    const double percentile_depth = valid_depths[percentile_index];

    std::vector<cv::Point> top_face_pixels;
    std::vector<float> top_face_depths;
    for (const DepthPixel& sample : valid_pixels) {
      if (std::abs(static_cast<double>(sample.depth) - percentile_depth) <= config.top_depth_tolerance) {
        top_face_pixels.push_back(sample.pixel);
        top_face_depths.push_back(sample.depth);
      }
    }

    if (top_face_pixels.size() < config.minimum_top_face_area) {
      const std::string reason = candidate.label + ": top face " + std::to_string(top_face_pixels.size()) + " px";
      rejections.push_back(reason);
      annotate_candidate(result.debug_image, candidate.contour, reason, cv::Scalar(255, 255, 0));
      continue;
    }

    const double top_depth = median(top_face_depths);
    const cv::RotatedRect rectangle = cv::minAreaRect(top_face_pixels);
    double centroid_column = 0.0;
    double centroid_row = 0.0;
    for (const cv::Point& pixel : top_face_pixels) {
      centroid_column += pixel.x;
      centroid_row += pixel.y;
    }
    centroid_column /= static_cast<double>(top_face_pixels.size());
    centroid_row /= static_cast<double>(top_face_pixels.size());

    std::array<cv::Point2f, 4> rectangle_pixels{};
    rectangle.points(rectangle_pixels.data());
    Eigen::Vector3d camera_top;
    std::string projection_error;
    if (!project_pixel(cv::Point2d(centroid_column, centroid_row), top_depth, camera_model, rectify_pixels, camera_top,
                       projection_error)) {
      const std::string reason = candidate.label + ": " + projection_error;
      rejections.push_back(reason);
      annotate_candidate(result.debug_image, candidate.contour, reason, cv::Scalar(255, 255, 0));
      continue;
    }

    std::array<Eigen::Vector3d, 4> world_corners{};
    bool corners_valid = true;
    for (std::size_t index = 0; index < rectangle_pixels.size(); ++index) {
      Eigen::Vector3d camera_corner;
      if (!project_pixel(rectangle_pixels[index], top_depth, camera_model, rectify_pixels, camera_corner,
                         projection_error)) {
        corners_valid = false;
        break;
      }
      world_corners[index] = world_from_camera * camera_corner;
    }
    if (!corners_valid) {
      const std::string reason = candidate.label + ": " + projection_error;
      rejections.push_back(reason);
      annotate_candidate(result.debug_image, candidate.contour, reason, cv::Scalar(255, 255, 0));
      continue;
    }

    const Eigen::Vector3d world_top = world_from_camera * camera_top;
    Eigen::Vector3d world_centre = world_top;
    // cubes are upright, so the top face becomes centre along world Z
    world_centre.z() -= config.cube_dimensions[2] / 2.0;
    if (!world_centre.allFinite() || !inside_source_area(world_centre, config)) {
      const std::string reason = candidate.label + ": outside source area";
      rejections.push_back(reason);
      annotate_candidate(result.debug_image, candidate.contour, reason, cv::Scalar(255, 255, 0));
      continue;
    }
    if (inside_exclusion_area(world_centre, config)) {
      const std::string reason = candidate.label + ": inside tray exclusion area";
      rejections.push_back(reason);
      annotate_candidate(result.debug_image, candidate.contour, reason, cv::Scalar(255, 255, 0));
      continue;
    }

    double first_side = 0.0;
    double second_side = 0.0;
    if (!physical_size_matches(world_corners, config, first_side, second_side)) {
      const std::string reason =
          candidate.label + ": size " + format_value(first_side) + " x " + format_value(second_side) + " m";
      rejections.push_back(reason);
      annotate_candidate(result.debug_image, candidate.contour, reason, cv::Scalar(255, 255, 0));
      continue;
    }

    result.cubes.push_back(DetectedCube{"", candidate.label, world_centre, rectangle_pixels,
                                        cv::Point2f(static_cast<float>(centroid_column), static_cast<float>(centroid_row))});
  }

  if (result.cubes.size() != expected_count) {
    draw_detections(result.debug_image, result.cubes);
    const std::size_t accepted_count = result.cubes.size();
    result.cubes.clear();
    result.phase = "object_count";
    result.message =
        "accepted " + std::to_string(accepted_count) + " of expected " + std::to_string(expected_count) + " cubes";

    // trays always land in rejections and always come first; skip them so a
    // real cube's reject reason isnt hidden behind the tray box.
    const std::string exclusion_suffix = ": inside tray exclusion area";
    std::vector<std::string> significant_rejections;
    for (const std::string& rejection : rejections) {
      const bool is_tray = rejection.size() >= exclusion_suffix.size() &&
                            rejection.compare(rejection.size() - exclusion_suffix.size(), exclusion_suffix.size(),
                                               exclusion_suffix) == 0;
      if (!is_tray) {
        significant_rejections.push_back(rejection);
      }
    }
    const std::vector<std::string>& reported = significant_rejections.empty() ? rejections : significant_rejections;
    if (!reported.empty()) {
      result.message += "; rejected: ";
      for (std::size_t index = 0; index < reported.size(); ++index) {
        if (index > 0) {
          result.message += ", ";
        }
        result.message += reported[index];
      }
    }
    cv::putText(result.debug_image, result.message, cv::Point(10, 25), cv::FONT_HERSHEY_SIMPLEX, 0.55,
                cv::Scalar(255, 255, 0), 2, cv::LINE_AA);
    return result;
  }

  std::sort(result.cubes.begin(), result.cubes.end(), [](const DetectedCube& left, const DetectedCube& right) {
    if (left.centre.x() != right.centre.x()) {
      return left.centre.x() < right.centre.x();
    }
    return left.centre.y() < right.centre.y();
  });
  for (std::size_t index = 0; index < result.cubes.size(); ++index) {
    result.cubes[index].id = "box_" + std::to_string(index + 1);
  }
  draw_detections(result.debug_image, result.cubes);

  result.ok = true;
  result.phase = "complete";
  result.message = "detected " + std::to_string(result.cubes.size()) + " cubes";
  return result;
}

}  // namespace sorting_arm_perception
