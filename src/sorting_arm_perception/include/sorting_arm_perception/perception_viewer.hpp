#ifndef SORTING_ARM_PERCEPTION__PERCEPTION_VIEWER_HPP_
#define SORTING_ARM_PERCEPTION__PERCEPTION_VIEWER_HPP_

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <opencv2/core.hpp>
#include <stop_token>
#include <thread>

namespace sorting_arm_perception {

class PerceptionViewer {
 public:
  explicit PerceptionViewer(double window_scale = 1.0);

  PerceptionViewer(const PerceptionViewer&) = delete;
  PerceptionViewer& operator=(const PerceptionViewer&) = delete;
  PerceptionViewer(PerceptionViewer&&) = delete;
  PerceptionViewer& operator=(PerceptionViewer&&) = delete;

  [[nodiscard]] bool active() const;
  void show_rgb(const cv::Mat& rgb_image);

 private:
  void run(std::stop_token stop_token) noexcept;

  // scales only the display window, never the frame the detector runs on
  double window_scale_ = 1.0;
  std::atomic_bool active_{true};
  std::mutex frame_mutex_;
  std::condition_variable frame_condition_;
  cv::Mat latest_bgr_image_;
  std::uint64_t frame_sequence_ = 0;

  // destroyed first, so its automatic stop and join finish before frame state
  std::jthread worker_;
};

}  // namespace sorting_arm_perception

#endif  // SORTING_ARM_PERCEPTION__PERCEPTION_VIEWER_HPP_
