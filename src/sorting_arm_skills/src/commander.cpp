#include <moveit/move_group_interface/move_group_interface.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("test_moveit");
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);

  // I have to spin this in a seperate thread
  auto my_spinner = std::thread([&exec]() { exec.spin(); });
  auto arm = moveit::planning_interface::MoveGroupInterface(node, "arm");
  auto gripper =
      moveit::planning_interface::MoveGroupInterface(node, "gripper");

  arm.setMaxAccelerationScalingFactor(1.0);
  arm.setMaxVelocityScalingFactor(1.0);
  gripper.setMaxAccelerationScalingFactor(1.0);
  gripper.setMaxVelocityScalingFactor(1.0);

  // //named goal for gripper (Doesnt work only GripperCommand works refer rca/gripper-command-vs-moveit-planning.md)
  // gripper.setStartStateToCurrentState();
  // gripper.setNamedTarget("gripper_close");

  // // First plan , if it succeeds then only execute
  // moveit::planning_interface::MoveGroupInterface::Plan plan1;
  // bool success = (gripper.plan(plan1) ==  moveit::core::MoveItErrorCode::SUCCESS );
  // if(success) gripper.execute(plan1);

  // // Joint Goal
  // std::vector<double> joints {0.34,-0.5,0.5,1.5,-1.3,0.8};
  // arm.setStartStateToCurrentState();
  // arm.setJointValueTarget(joints);
  // moveit::planning_interface::MoveGroupInterface::Plan plan1;
  // bool succ = (arm.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);
  // if(succ) arm.execute(plan1);

  // Pose Goal
  auto msg =
      arm.getCurrentPose(); // use sim time needed by default uses wall time
  msg.pose.position.y -= 0.05;
  arm.setStartStateToCurrentState();
  arm.setPoseTarget(msg);

  moveit::planning_interface::MoveGroupInterface::Plan plan1;
  bool succ = (arm.plan(plan1) == moveit::core::MoveItErrorCode::SUCCESS);
  if (succ)
    arm.execute(plan1);

  // close the node and join the thread with our main thread
  rclcpp::shutdown();
  my_spinner.join();
  return 0;
}