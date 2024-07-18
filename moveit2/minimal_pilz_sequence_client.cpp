#include <memory>
#include <string>
using std::string;
#include <fstream>
#include <iostream>
#include <math.h>
#include <sstream>
#include <unistd.h>
#include <vector>

#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit/robot_model/robot_model.h"
#include "moveit/robot_state/conversions.h"
#include "moveit/robot_state/robot_state.h"
#include "moveit/robot_trajectory/robot_trajectory.h"
#include "moveit/trajectory_processing/iterative_time_parameterization.h"

using moveit::planning_interface::MoveGroupInterface;
#include "moveit_msgs/action/move_group_sequence.hpp"
#include "moveit_msgs/msg/motion_sequence_request.hpp"
using moveit_msgs::action::MoveGroupSequence;

#include "rclcpp/rclcpp.hpp"
using namespace rclcpp;
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/LinearMath/Quaternion.h"
using tf2::Quaternion;

using geometry_msgs::msg::Pose;

class MotionController : public Node {
public:
  MotionController(NodeOptions node_options)
      : Node("motion_controller", node_options) {
    RCLCPP_INFO(get_logger(), "Initiated %s node", this->get_name());
    move_group_name = "panda_arm";
    max_tcp_speed = 0.4; // same value as in pilz_cartesian_limits.yaml
  }

  void init() {
    RCLCPP_DEBUG(get_logger(), "Creating `move_group object");
    move_group_ = std::make_shared<MoveGroupInterface>(shared_from_this(),
                                                       move_group_name);
    move_group_->setMaxVelocityScalingFactor(0.6); // travel speed
    move_group_->setPlanningTime(10);

    RCLCPP_INFO(get_logger(), "Controlled Link: %s",
                move_group_->getEndEffectorLink().c_str());

    RCLCPP_INFO(get_logger(), "Maximum end effector lin speed: %f [m/s]",
                max_tcp_speed);

    pilz_sequence_client_ = rclcpp_action::create_client<MoveGroupSequence>(
        this, "sequence_move_group");
  }

  void run() {
    move_group_->setPlanningPipelineId("pilz_industrial_motion_planner");
    move_group_->setPlannerId("LIN");
    move_group_->setMaxAccelerationScalingFactor(0.1);
    move_group_->setNumPlanningAttempts(1);
    move_group_->setPlanningTime(10.0);

    auto home_pose = move_group_->getCurrentPose().pose;
    RCLCPP_INFO(get_logger(),
                "current_pose = x%.2f y%.2f z%.2f qx%.2f qy%.2f qz%.2f qw%.2f",
                home_pose.position.x, home_pose.position.y,
                home_pose.position.z, home_pose.orientation.x,
                home_pose.orientation.y, home_pose.orientation.z,
                home_pose.orientation.w);

    std::vector<Pose> waypoints;
    waypoints.push_back(
        generateRelativePose(0.350, 0.3, -0.2, 0.0, 0.0, 0.0, home_pose));
    waypoints.push_back(
        generateRelativePose(-0.10, 0.3, -0.2, 0.0, 0.0, 0.0, home_pose));
    waypoints.push_back(
        generateRelativePose(0.350, 0.3, -0.2, 0.1, 0.0, 0.0, home_pose));
    waypoints.push_back(
        generateRelativePose(-0.10, 0.3, -0.2, 0.0, 0.0, 0.0, home_pose));
    waypoints.push_back(
        generateRelativePose(0.350, 0.3, -0.2, 0.4, 0.0, 0.0, home_pose));
    waypoints.push_back(
        generateRelativePose(-0.10, 0.3, -0.2, 0.0, 0.0, 0.0, home_pose));
    waypoints.push_back(
        generateRelativePose(0.350, 0.3, -0.2, 0.8, 0.0, 0.0, home_pose));
    waypoints.push_back(
        generateRelativePose(-0.10, 0.3, -0.2, 0.0, 0.0, 0.0, home_pose));
    waypoints.push_back(
        generateRelativePose(0.350, 0.3, -0.2, 1.6, 0.0, 0.0, home_pose));
    waypoints.push_back(
        generateRelativePose(-0.10, 0.3, -0.2, 0.0, 0.0, 0.0, home_pose));
    waypoints.push_back(home_pose);

    auto sequence_goal = MoveGroupSequence::Goal();
    for (Pose waypoint : waypoints) {
      const double max_velocity_scaling_factor = 0.1;
      move_group_->setMaxVelocityScalingFactor(max_velocity_scaling_factor);
      move_group_->setPoseTarget(waypoint);

      moveit_msgs::msg::MotionSequenceItem motion_sequence_item;
      move_group_->constructMotionPlanRequest(motion_sequence_item.req);
      motion_sequence_item.blend_radius = 0.0;

      sequence_goal.request.items.push_back(motion_sequence_item);
    }
    // add start state only to first waypoint
    moveit::core::robotStateToRobotStateMsg(
        *move_group_->getCurrentState(),
        sequence_goal.request.items[0].req.start_state);

    sequence_goal.planning_options.plan_only = false;
    sendPilzSequenceGoal(sequence_goal); // plan and execute
  }

private:
  string move_group_name;
  std::shared_ptr<MoveGroupInterface> move_group_;
  double max_tcp_speed;
  rclcpp_action::Client<MoveGroupSequence>::SharedPtr pilz_sequence_client_;

  void sendPilzSequenceGoal(MoveGroupSequence::Goal sequence_request) {
    std::chrono::milliseconds pilz_timeout(1000); // 1[s] timeout
    if (!pilz_sequence_client_->wait_for_action_server(pilz_timeout)) {
      RCLCPP_ERROR(get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
    }

    RCLCPP_INFO(get_logger(), "Sending goal");

    pilz_sequence_client_->async_send_goal(sequence_request);
    sleep(5);
  }

  Pose generateRelativePose(double x, double y, double z, double roll,
                            double pitch, double yaw,
                            const Pose &reference_pose) {
    Pose pose;
    pose.position.x = x + reference_pose.position.x;
    pose.position.y = y + reference_pose.position.y;
    pose.position.z = z + reference_pose.position.z;

    Quaternion q;
    q.setRPY(roll, pitch + M_PI, yaw + M_PI);
    q.normalize();

    pose.orientation.x = q.x();
    pose.orientation.y = q.y();
    pose.orientation.z = q.z();
    pose.orientation.w = q.w();

    return pose;
  }
};

int main(int argc, char **argv) {
  init(argc, argv);
  auto motion_controller_node = std::make_shared<MotionController>(
      NodeOptions().automatically_declare_parameters_from_overrides(true));

  // needed for getCurrentPose() ->
  // https://robotics.stackexchange.com/questions/103393/how-to-extract-position-of-the-gripper-in-ros2-moveit2/103394#103394
  executors::SingleThreadedExecutor executor;
  executor.add_node(motion_controller_node);

  std::thread spinner = std::thread([&executor]() { executor.spin(); });

  // Call initialize function after creating MotionController instance
  motion_controller_node->init();
  motion_controller_node->run();

  shutdown();
  spinner.join();
  return 0;
}
