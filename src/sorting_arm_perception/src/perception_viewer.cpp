#include <chrono>
#include <cv_bridge/cv_bridge.hpp>
#include <exception>
#include <memory>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace {

using namespace std::chrono_literals;

constexpr char kDisplayTopic[] = "/perception/display_image";
constexpr char kWindowName[] = "object_detector";

class PerceptionViewer : public rclcpp::Node {
 public:
  PerceptionViewer() : Node("perception_viewer") {
    cv::namedWindow(kWindowName, cv::WINDOW_NORMAL);
    image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
        kDisplayTopic, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::Image::ConstSharedPtr image) { image_callback(image); });
    render_timer_ = create_wall_timer(33ms, [this] { render(); });
  }

  ~PerceptionViewer() override { cv::destroyWindow(kWindowName); }

 private:
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr& image) {
    try {
      const cv_bridge::CvImageConstPtr rgb = cv_bridge::toCvShare(image, sensor_msgs::image_encodings::RGB8);
      cv::cvtColor(rgb->image, latest_bgr_image_, cv::COLOR_RGB2BGR);
      if (latest_bgr_image_.size() != window_image_size_) {
        window_image_size_ = latest_bgr_image_.size();
        resize_window_ = true;
      }
    } catch (const cv_bridge::Exception& exception) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "viewer image conversion failed: %s", exception.what());
    }
  }

  void render() {
    if (resize_window_) {
      cv::resizeWindow(kWindowName, window_image_size_.width, window_image_size_.height);
      resize_window_ = false;
    }
    if (!latest_bgr_image_.empty()) {
      cv::imshow(kWindowName, latest_bgr_image_);
    }
    const int key = cv::waitKey(1);
    if (key == 27 || cv::getWindowProperty(kWindowName, cv::WND_PROP_VISIBLE) < 1.0) {
      rclcpp::shutdown();
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::TimerBase::SharedPtr render_timer_;
  cv::Mat latest_bgr_image_;
  cv::Size window_image_size_;
  bool resize_window_ = false;
};

}  // namespace

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(std::make_shared<PerceptionViewer>());
  } catch (const cv::Exception& exception) {
    RCLCPP_FATAL(rclcpp::get_logger("perception_viewer"), "OpenCV viewer failed: %s", exception.what());
    rclcpp::shutdown();
    return 1;
  } catch (const std::exception& exception) {
    RCLCPP_FATAL(rclcpp::get_logger("perception_viewer"), "viewer failed: %s", exception.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
