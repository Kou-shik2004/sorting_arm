#include "sorting_arm_perception/perception_viewer.hpp"

#include <chrono>
#include <exception>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/logging.hpp>
#include <utility>

using namespace std::chrono_literals;

namespace sorting_arm_perception {

PerceptionViewer::PerceptionViewer(double window_scale)
    : window_scale_(window_scale), worker_([this](std::stop_token stop_token) { run(stop_token); }) {}

bool PerceptionViewer::active() const { return active_.load(); }

void PerceptionViewer::show_rgb(const cv::Mat& rgb_image) {
  if (!active()) {
    return;
  }

  cv::Mat bgr_image;
  cv::cvtColor(rgb_image, bgr_image, cv::COLOR_RGB2BGR);
  {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    if (!active()) {
      return;
    }
    latest_bgr_image_ = std::move(bgr_image);
    ++frame_sequence_;
  }
  frame_condition_.notify_one();
}

void PerceptionViewer::run(std::stop_token stop_token) noexcept {
  try {
    cv::namedWindow("object_detector", cv::WINDOW_NORMAL);
    cv::Mat displayed_image;
    cv::Size displayed_size;
    std::uint64_t displayed_sequence = 0;

    while (!stop_token.stop_requested()) {
      {
        std::unique_lock<std::mutex> lock(frame_mutex_);
        frame_condition_.wait_for(lock, 33ms, [this, displayed_sequence] { return frame_sequence_ != displayed_sequence; });
        if (frame_sequence_ != displayed_sequence) {
          displayed_image = latest_bgr_image_;
          displayed_sequence = frame_sequence_;
        }
      }

      if (!displayed_image.empty()) {
        if (displayed_image.size() != displayed_size) {
          displayed_size = displayed_image.size();
          cv::resizeWindow("object_detector", static_cast<int>(displayed_size.width * window_scale_),
                            static_cast<int>(displayed_size.height * window_scale_));
        }
        cv::imshow("object_detector", displayed_image);
      }

      const int key = cv::waitKey(1);
      if (key == 27 || cv::getWindowProperty("object_detector", cv::WND_PROP_VISIBLE) < 1.0) {
        RCLCPP_INFO(rclcpp::get_logger("camera_object_provider"),
                    "viewer closed; detection remains active and the window returns after restart");
        break;
      }
    }

    cv::destroyWindow("object_detector");
  } catch (const cv::Exception& exception) {
    RCLCPP_ERROR(rclcpp::get_logger("camera_object_provider"), "OpenCV viewer stopped: %s", exception.what());
  } catch (const std::exception& exception) {
    RCLCPP_ERROR(rclcpp::get_logger("camera_object_provider"), "viewer stopped: %s", exception.what());
  } catch (...) {
    RCLCPP_ERROR(rclcpp::get_logger("camera_object_provider"), "viewer stopped after an unknown exception");
  }

  {
    std::lock_guard<std::mutex> lock(frame_mutex_);
    active_.store(false);
    latest_bgr_image_.release();
  }
}

}  // namespace sorting_arm_perception
