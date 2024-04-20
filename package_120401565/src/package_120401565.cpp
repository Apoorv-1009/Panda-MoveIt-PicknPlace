#include <memory>
#include <unistd.h>
#include <vector>
#include <chrono>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

int main(int argc, char * argv[])
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "hello_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  // Create a ROS logger
  auto const logger = rclcpp::get_logger("hello_moveit");

  // Create the MoveIt MoveGroup Interface
  using moveit::planning_interface::MoveGroupInterface;
  auto move_group_interface = MoveGroupInterface(node, "panda_arm");
  auto move_group_hand = MoveGroupInterface(node, "panda_hand");
  std::vector<double> gripper_open = {0.01, 0.01};
  std::vector<double> gripper_closed = {0.0, 0.0};

  // Declare function variables
  int const sleep_time = 2;

  // Declare success and plan variables
  bool success;
  moveit::planning_interface::MoveGroupInterface::Plan plan;

  //////////// OPEN THE GRIPPER ////////////
  RCLCPP_INFO_STREAM(node->get_logger(), "Opening gripper");

  move_group_hand.setJointValueTarget(gripper_open);
  // Create a plan to that target pose

  std::tie(success, plan) = [&move_group_hand]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_hand.plan(msg));
    return std::make_pair(ok, msg);
  }();
  // Execute the plan
  if(success) {
    move_group_hand.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planing failed!");
  }

  ////////// MOVE TO TARGET POSE1 //////////
  RCLCPP_INFO_STREAM(node->get_logger(), "Planning to move to target pose 1");
  
  // Set a target Pose 1 
  auto target_pose = []{
    geometry_msgs::msg::Pose msg;
    msg.position.x = -0.023064;
    msg.position.y = -0.24096;
    msg.position.z =  0.6723;
    msg.orientation.x = 0.92387;
    msg.orientation.y = 0.3827;
    msg.orientation.z = 3.9324e-05;
    msg.orientation.w = 2.0581e-06;
    return msg;
  }();

  move_group_interface.setPoseTarget(target_pose);
  // Create a plan to that target pose
  std::tie(success, plan) = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();
  // Execute the plan
  if(success) {
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planing failed!");
  }

  ////////// CLOSE THE GRIPPER //////////
  RCLCPP_INFO_STREAM(node->get_logger(), "Closing gripper");

  move_group_hand.setJointValueTarget(gripper_closed);
  // Create a plan to that target pose

  std::tie(success, plan) = [&move_group_hand]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_hand.plan(msg));
    return std::make_pair(ok, msg);
  }();
  // Execute the plan
  if(success) {
    move_group_hand.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planing failed!");
  }

  // Sleep for few seconds
  std::this_thread::sleep_for(std::chrono::seconds(sleep_time));

  ////////// MOVE TO TARGET POSE2 //////////
  RCLCPP_INFO_STREAM(node->get_logger(), "Planning to move to target pose 2");
  // Set a target Pose 2 
  target_pose = []{
    geometry_msgs::msg::Pose msg;
    msg.position.x = 0.5385292768478394;
    msg.position.y = 0.5385292768478394;
    msg.position.z = 0.46101266145706177;
    msg.orientation.x = 0.9238730072975159;
    msg.orientation.y = 0.382699191570282;
    msg.orientation.z = 2.52739084771747e-07;
    msg.orientation.w = -1.8647500837687403e-05;
    return msg;
  }();
  move_group_interface.setPoseTarget(target_pose);
  // Create a plan to that target pose
  std::tie(success, plan) = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();
  // Execute the plan
  if(success) {
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planing failed!");
  }

  //////////// OPEN THE GRIPPER ////////////
  RCLCPP_INFO_STREAM(node->get_logger(), "Opening gripper");

  move_group_hand.setJointValueTarget(gripper_open);
  // Create a plan to that target pose

  std::tie(success, plan) = [&move_group_hand]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_hand.plan(msg));
    return std::make_pair(ok, msg);
  }();
  // Execute the plan
  if(success) {
    move_group_hand.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planing failed!");
  }

  // Sleep for few seconds
  std::this_thread::sleep_for(std::chrono::seconds(sleep_time));

  ////////// MOVE TO HOME //////////
  RCLCPP_INFO_STREAM(node->get_logger(), "Planning to move to Home");
  move_group_interface.setNamedTarget("home");

  // Create a plan to that target pose
  std::tie(success, plan) = [&move_group_interface]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group_interface.plan(msg));
    return std::make_pair(ok, msg);
  }();

  // Execute the plan
  if(success) {
    move_group_interface.execute(plan);
  } else {
    RCLCPP_ERROR(logger, "Planing failed!");
  }

  // Shutdown ROS
  rclcpp::shutdown();
  return 0;
}
