#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "ros_gz_interfaces/srv/spawn_entity.hpp"

namespace sorting_arm_bringup {
namespace {

using SpawnEntity = ros_gz_interfaces::srv::SpawnEntity;

// Source region on the table where cubes may appear. Must lie inside the wrist camera's
// field of view at the home pose and clear of the trays. Tune against the runtime view.
constexpr double kSourceMinX = 0.30;
constexpr double kSourceMaxX = 0.60;
constexpr double kSourceMinY = -0.18;
constexpr double kSourceMaxY = 0.18;
constexpr double kSpawnZ = 0.520;

// centre spacing that lets the gripper pick a cube without fouling a neighbour
constexpr double kMinSpacing = 0.09;

// bounds the rejection sampling so a too-small region fails loud instead of looping forever
constexpr int kMaxSampleAttempts = 2000;

// a run may be all one colour, so the count is capped at one tray's capacity
constexpr std::int64_t kMinObjectCount = 2;
constexpr std::int64_t kMaxObjectCount = 8;

std::vector<std::pair<double, double>> random_spaced_positions(std::size_t count, std::mt19937& rng) {
  std::uniform_real_distribution<double> x_dist(kSourceMinX, kSourceMaxX);
  std::uniform_real_distribution<double> y_dist(kSourceMinY, kSourceMaxY);
  std::vector<std::pair<double, double>> positions;

  int attempts = 0;
  while (positions.size() < count && attempts < kMaxSampleAttempts) {
    ++attempts;
    double x = x_dist(rng);
    double y = y_dist(rng);
    bool clear = std::none_of(positions.begin(), positions.end(), [x, y](const std::pair<double, double>& placed) {
      return std::hypot(x - placed.first, y - placed.second) < kMinSpacing;
    });
    if (clear) {
      positions.emplace_back(x, y);
    }
  }

  if (positions.size() < count) {
    throw std::runtime_error("could not place " + std::to_string(count) +
                             " spaced cubes in the source region; widen the region or lower object_count");
  }
  return positions;
}

class CubeSpawner : public rclcpp::Node {
 public:
  CubeSpawner() : Node("cube_spawner") {
    auto object_count = declare_parameter<std::int64_t>("object_count");
    if (object_count < kMinObjectCount || object_count > kMaxObjectCount) {
      throw std::runtime_error("object_count must be between " + std::to_string(kMinObjectCount) + " and " +
                               std::to_string(kMaxObjectCount));
    }
    object_count_ = static_cast<std::size_t>(object_count);

    red_model_ = declare_parameter<std::string>("red_model");
    blue_model_ = declare_parameter<std::string>("blue_model");
    client_ = create_client<SpawnEntity>("/world/sorting_cell/create");
  }

  void spawn_all() {
    if (!client_->wait_for_service(std::chrono::seconds(30))) {
      throw std::runtime_error("Gazebo create service /world/sorting_cell/create did not appear");
    }

    std::mt19937 rng{std::random_device{}()};
    auto positions = random_spaced_positions(object_count_, rng);

    // equal split: each tray holds at most ceil(count/2) = 4 cubes for count <= 8
    std::size_t red_count = object_count_ / 2;
    if (object_count_ % 2 == 1 && std::bernoulli_distribution(0.5)(rng)) {
      ++red_count;
    }
    std::vector<bool> is_red(object_count_, false);
    std::fill(is_red.begin(), is_red.begin() + static_cast<std::ptrdiff_t>(red_count), true);
    std::shuffle(is_red.begin(), is_red.end(), rng);

    for (std::size_t index = 0; index < positions.size(); ++index) {
      bool red = is_red[index];
      std::string name = (red ? "red_cube_" : "blue_cube_") + std::to_string(index);

      auto request = std::make_shared<SpawnEntity::Request>();
      request->entity_factory.name = name;
      request->entity_factory.sdf_filename = red ? red_model_ : blue_model_;
      request->entity_factory.relative_to = "world";
      request->entity_factory.pose.position.x = positions[index].first;
      request->entity_factory.pose.position.y = positions[index].second;
      request->entity_factory.pose.position.z = kSpawnZ;
      request->entity_factory.pose.orientation.w = 1.0;

      auto future = client_->async_send_request(request);
      if (rclcpp::spin_until_future_complete(get_node_base_interface(), future) != rclcpp::FutureReturnCode::SUCCESS) {
        throw std::runtime_error("create service call did not complete for " + name);
      }
      if (!future.get()->success) {
        throw std::runtime_error("Gazebo refused to spawn " + name);
      }
      RCLCPP_INFO(get_logger(), "spawned %s at (%.3f, %.3f)", name.c_str(), positions[index].first,
                  positions[index].second);
    }
    RCLCPP_INFO(get_logger(), "spawned %zu cubes", positions.size());
  }

 private:
  std::size_t object_count_ = 0;
  std::string red_model_;
  std::string blue_model_;
  rclcpp::Client<SpawnEntity>::SharedPtr client_;
};

}  // namespace
}  // namespace sorting_arm_bringup

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<sorting_arm_bringup::CubeSpawner>();
    node->spawn_all();
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("cube_spawner"), "spawn failed: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
