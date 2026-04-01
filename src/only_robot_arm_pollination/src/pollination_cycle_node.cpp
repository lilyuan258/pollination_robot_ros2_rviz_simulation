#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <Eigen/Geometry>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <moveit/robot_state/conversions.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/robot_model/robot_model.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <visualization_msgs/msg/marker_array.hpp>

using namespace std::chrono_literals;

class PollinationCycleNode : public rclcpp::Node
{
public:
  PollinationCycleNode()
  : Node("pollination_cycle_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_),
    stop_requested_(false)
  {
    world_frame_ = declare_parameter<std::string>("world_frame", "world");
    anchor_frame_ = declare_parameter<std::string>("anchor_frame", "link_1");
    planning_group_ = declare_parameter<std::string>("planning_group", "arm");
    end_effector_link_ = declare_parameter<std::string>("end_effector_link", "pollination_tip_tcp");
    flower_topic_ = declare_parameter<std::string>("flower_topic", "/watermelon_flower");
    flower_marker_id_ = declare_parameter<int>("flower_marker_id", 1);
    ik_timeout_ = declare_parameter<double>("ik_timeout", 0.1);
    prevalidate_pollination_pose_ = declare_parameter<bool>("prevalidate_pollination_pose", false);
    planning_time_ = declare_parameter<double>("planning_time", 10.0);
    max_velocity_scaling_ = declare_parameter<double>("max_velocity_scaling", 0.2);
    max_acceleration_scaling_ = declare_parameter<double>("max_acceleration_scaling", 0.2);
    num_planning_attempts_ = declare_parameter<int>("num_planning_attempts", 10);
    base_clockwise_rotation_rad_ = declare_parameter<double>("base_clockwise_rotation_rad", M_PI_2);
    ik_roll_samples_ = declare_parameter<int>("ik_roll_samples", 8);
    pre_approach_offset_m_ = declare_parameter<double>("pre_approach_offset_m", 0.12);
    pollination_offset_m_ = declare_parameter<double>("pollination_offset_m", 0.05);
    cartesian_step_m_ = declare_parameter<double>("cartesian_step_m", 0.005);
    cartesian_jump_threshold_ = declare_parameter<double>("cartesian_jump_threshold", 0.0);
    cartesian_min_fraction_ = declare_parameter<double>("cartesian_min_fraction", 0.95);
    goal_position_tolerance_m_ = declare_parameter<double>("goal_position_tolerance_m", 0.003);
    goal_orientation_tolerance_rad_ =
      declare_parameter<double>("goal_orientation_tolerance_rad", 0.03);
    pre_approach_orientation_tolerance_rad_ =
      declare_parameter<double>("pre_approach_orientation_tolerance_rad", 0.45);
    dwell_seconds_ = declare_parameter<double>("dwell_seconds", 1.0);
    cycle_pause_seconds_ = declare_parameter<double>("cycle_pause_seconds", 1.0);
    flower_message_timeout_sec_ = declare_parameter<double>("flower_message_timeout_sec", 1.0);
    joint_state_publish_rate_hz_ = declare_parameter<double>("joint_state_publish_rate_hz", 30.0);
    joint_names_ = declare_parameter<std::vector<std::string>>(
      "joint_names",
      std::vector<std::string>{"joint_1", "joinit_2", "joint_3", "joint_4", "joint_5", "joint_6"});
    initial_positions_ = declare_parameter<std::vector<double>>(
      "initial_positions",
      std::vector<double>{0.0, 2.087, -2.495, 0.085, 0.458, 0.017});

    flower_sub_ = create_subscription<visualization_msgs::msg::MarkerArray>(
      flower_topic_,
      rclcpp::QoS(10),
      std::bind(&PollinationCycleNode::handleFlowerMarkers, this, std::placeholders::_1));
  }

  ~PollinationCycleNode() override
  {
    stop_requested_.store(true);
    if (worker_.joinable())
    {
      worker_.join();
    }
  }

  void initialize()
  {
    if (joint_names_.size() != initial_positions_.size())
    {
      throw std::runtime_error("joint_names and initial_positions must have the same length");
    }

    {
      std::scoped_lock lock(joint_state_mutex_);
      current_joint_positions_ = initial_positions_;
      current_joint_velocities_.assign(joint_names_.size(), 0.0);
    }

    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    display_trajectory_pub_ = create_publisher<moveit_msgs::msg::DisplayTrajectory>(
      "/display_planned_path",
      10);

    publishCurrentJointState();
    joint_state_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / joint_state_publish_rate_hz_),
      std::bind(&PollinationCycleNode::publishCurrentJointState, this));

    {
      std::scoped_lock lock(flower_mutex_);
      flower_state_.valid = false;
      accepted_flower_marker_after_ = now();
    }

    move_group_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
      shared_from_this(),
      planning_group_);
    move_group_->setPoseReferenceFrame(world_frame_);
    move_group_->setEndEffectorLink(end_effector_link_);
    move_group_->setPlanningTime(planning_time_);
    move_group_->setNumPlanningAttempts(std::max(1, num_planning_attempts_));
    move_group_->setMaxVelocityScalingFactor(max_velocity_scaling_);
    move_group_->setMaxAccelerationScalingFactor(max_acceleration_scaling_);
    move_group_->setPlannerId("RRTConnectkConfigDefault");
    planning_scene_ = std::make_shared<planning_scene::PlanningScene>(move_group_->getRobotModel());

    worker_ = std::thread(&PollinationCycleNode::runCycleLoop, this);
  }

private:
  struct FlowerState
  {
    geometry_msgs::msg::Point center;
    rclcpp::Time marker_stamp;
    rclcpp::Time receipt_stamp;
    bool valid{false};
  };

  struct ApproachTargets
  {
    geometry_msgs::msg::Pose pre_pose;
    geometry_msgs::msg::Pose pollination_pose;
    std::vector<double> pre_joint_target;
    std::vector<double> pollination_joint_target;
    Eigen::Vector3d axis_to_flower{Eigen::Vector3d::Zero()};
    std::string strategy;
    bool valid{false};
  };

  struct JointTargetSolution
  {
    geometry_msgs::msg::Pose pose;
    std::vector<double> joint_values;
    double score{std::numeric_limits<double>::infinity()};
    int pose_index{-1};
    bool valid{false};
  };

  void handleFlowerMarkers(const visualization_msgs::msg::MarkerArray::SharedPtr msg)
  {
    for (const auto & marker : msg->markers)
    {
      if (marker.id != flower_marker_id_)
      {
        continue;
      }

      const auto marker_stamp =
        rclcpp::Time(marker.header.stamp, get_clock()->get_clock_type());
      if (marker_stamp.nanoseconds() > 0 && marker_stamp < accepted_flower_marker_after_)
      {
        RCLCPP_INFO_THROTTLE(
          get_logger(),
          *get_clock(),
          3000,
          "Ignoring stale flower marker older than the current node startup.");
        continue;
      }

      std::scoped_lock lock(flower_mutex_);
      if (
        flower_state_.valid && marker_stamp.nanoseconds() > 0 &&
        marker_stamp <= flower_state_.marker_stamp)
      {
        continue;
      }
      flower_state_.center = marker.pose.position;
      flower_state_.marker_stamp = marker_stamp;
      flower_state_.receipt_stamp = now();
      flower_state_.valid = true;
      return;
    }
  }

  void runCycleLoop()
  {
    while (rclcpp::ok() && !stop_requested_.load())
    {
      if (!waitUntilReady())
      {
        return;
      }

      auto flower = snapshotFlower();
      if (!flower.valid)
      {
        sleepSeconds(0.5);
        continue;
      }

      RCLCPP_INFO(
        get_logger(),
        "Starting pollination cycle for flower center [%.3f, %.3f, %.3f] in %s.",
        flower.center.x,
        flower.center.y,
        flower.center.z,
        world_frame_.c_str());

      const auto initial_target = initial_positions_;
      auto rotated_target = initial_positions_;
      rotated_target.front() = wrapAngle(rotated_target.front() + base_clockwise_rotation_rad_);

      if (!moveToJointTarget(initial_target, "initial_contracted"))
      {
        sleepSeconds(1.0);
        continue;
      }

      if (!moveToJointTarget(rotated_target, "base_clockwise_90deg"))
      {
        sleepSeconds(1.0);
        continue;
      }

      geometry_msgs::msg::Point anchor_point;
      RCLCPP_INFO(
        get_logger(),
        "Resolving anchor frame %s in %s before target generation.",
        anchor_frame_.c_str(),
        world_frame_.c_str());
      if (!lookupAnchorPoint(anchor_point))
      {
        sleepSeconds(1.0);
        continue;
      }

      RCLCPP_INFO(
        get_logger(),
        "Anchor point %s resolved at [%.3f, %.3f, %.3f].",
        anchor_frame_.c_str(),
        anchor_point.x,
        anchor_point.y,
        anchor_point.z);

      RCLCPP_INFO(
        get_logger(),
        "Selecting approach targets with pre offset %.3f m and pollination offset %.3f m.",
        pre_approach_offset_m_,
        pollination_offset_m_);
      const auto approach_targets = selectApproachTargets(flower.center, anchor_point);
      if (!approach_targets.valid)
      {
        RCLCPP_WARN(get_logger(), "Failed to find a reachable pollination axis candidate.");
        attemptReturnToInitial();
        sleepSeconds(1.0);
        continue;
      }

      const auto & pre_pose = approach_targets.pre_pose;
      const auto & pollination_pose = approach_targets.pollination_pose;

      RCLCPP_INFO(
        get_logger(),
        "Selected pollination axis [%0.3f, %0.3f, %0.3f] using strategy %s.",
        approach_targets.axis_to_flower.x(),
        approach_targets.axis_to_flower.y(),
        approach_targets.axis_to_flower.z(),
        approach_targets.strategy.c_str());
      logPose("pre_approach_axis_aligned", pre_pose);
      logPose("pollination_pose", pollination_pose);

      if (!moveToJointTarget(approach_targets.pre_joint_target, "pre_approach"))
      {
        attemptReturnToInitial();
        sleepSeconds(1.0);
        continue;
      }

      if (!moveThroughPollinationApproach(flower.center, approach_targets))
      {
        attemptReturnToInitial();
        sleepSeconds(1.0);
        continue;
      }

      if (!executeCartesianSegment({pollination_pose}, "pollination_pose"))
      {
        RCLCPP_WARN(
          get_logger(),
          "Cartesian advance to pollination pose failed. Falling back to pose/joint goal refinement.");
        const auto pollination_goal_candidates =
          buildPollinationGoalCandidates(flower.center, approach_targets);
        if (
          !approach_targets.pollination_joint_target.empty() &&
          moveToJointTarget(approach_targets.pollination_joint_target, "pollination_pose_fallback"))
        {
          // The exact joint target was prevalidated and succeeded.
        }
        else if (
          !moveToPoseCandidates(
            pollination_goal_candidates,
            "pollination_pose_fallback",
            goal_position_tolerance_m_,
            goal_orientation_tolerance_rad_))
        {
          attemptReturnToInitial();
          sleepSeconds(1.0);
          continue;
        }
      }

      RCLCPP_INFO(
        get_logger(),
        "Pollinating flower at [%.3f, %.3f, %.3f] in %s.",
        flower.center.x,
        flower.center.y,
        flower.center.z,
        world_frame_.c_str());
      sleepSeconds(dwell_seconds_);

      if (!executeCartesianSegment({pre_pose}, "retreat"))
      {
        RCLCPP_WARN(
          get_logger(),
          "Cartesian retreat failed. Falling back to stored pre-alignment joint target.");
        moveToJointTarget(approach_targets.pre_joint_target, "retreat_fallback");
      }
      attemptReturnToInitial();
      sleepSeconds(cycle_pause_seconds_);
    }
  }

  bool waitUntilReady()
  {
    while (rclcpp::ok() && !stop_requested_.load())
    {
      const auto state = move_group_->getCurrentState(2.0);
      if (state)
      {
        std::scoped_lock lock(flower_mutex_);
        if (flower_state_.valid)
        {
          return true;
        }
      }

      RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        3000,
        "Waiting for MoveIt current state and watermelon flower marker...");
      sleepSeconds(0.5);
    }

    return false;
  }

  FlowerState snapshotFlower()
  {
    std::scoped_lock lock(flower_mutex_);
    auto snapshot = flower_state_;
    if (!snapshot.valid)
    {
      return snapshot;
    }

    if ((now() - snapshot.receipt_stamp).seconds() > flower_message_timeout_sec_)
    {
      snapshot.valid = false;
    }
    return snapshot;
  }

  bool buildRobotStateFromSimulatedJoints(moveit::core::RobotState & state) const
  {
    if (move_group_ == nullptr || move_group_->getRobotModel() == nullptr)
    {
      return false;
    }

    state = moveit::core::RobotState(move_group_->getRobotModel());
    state.setToDefaultValues();

    std::vector<double> joint_positions;
    {
      std::scoped_lock lock(joint_state_mutex_);
      joint_positions = current_joint_positions_;
    }

    if (joint_positions.size() != joint_names_.size())
    {
      return false;
    }

    for (std::size_t i = 0; i < joint_names_.size(); ++i)
    {
      state.setVariablePosition(joint_names_[i], joint_positions[i]);
    }
    state.update();
    return true;
  }

  bool setMoveGroupStartStateFromSimulatedJoints()
  {
    moveit::core::RobotState state(move_group_->getRobotModel());
    if (!buildRobotStateFromSimulatedJoints(state))
    {
      move_group_->setStartStateToCurrentState();
      return false;
    }

    move_group_->setStartState(state);
    return true;
  }

  bool lookupAnchorPoint(geometry_msgs::msg::Point & anchor_point)
  {
    try
    {
      const auto transform = tf_buffer_.lookupTransform(
        world_frame_,
        anchor_frame_,
        tf2::TimePointZero);
      anchor_point.x = transform.transform.translation.x;
      anchor_point.y = transform.transform.translation.y;
      anchor_point.z = transform.transform.translation.z;
      return true;
    }
    catch (const tf2::TransformException & ex)
    {
      RCLCPP_WARN(get_logger(), "Failed to lookup %s -> %s: %s", world_frame_.c_str(), anchor_frame_.c_str(), ex.what());
      return false;
    }
  }

  geometry_msgs::msg::Pose makeTcpPose(
    const geometry_msgs::msg::Point & flower_center,
    const geometry_msgs::msg::Point & anchor_point,
    double offset_m) const
  {
    Eigen::Vector3d flower(flower_center.x, flower_center.y, flower_center.z);
    Eigen::Vector3d anchor(anchor_point.x, anchor_point.y, anchor_point.z);
    Eigen::Vector3d tool_minus_z = flower - anchor;
    if (tool_minus_z.norm() < 1e-6)
    {
      tool_minus_z = Eigen::Vector3d(0.0, 0.0, -1.0);
    }
    tool_minus_z.normalize();

    return makeTcpPoseFromAxis(flower_center, tool_minus_z, offset_m);
  }

  geometry_msgs::msg::Pose makeTcpPoseFromAxis(
    const geometry_msgs::msg::Point & flower_center,
    const Eigen::Vector3d & axis_to_flower,
    double offset_m) const
  {
    Eigen::Vector3d tool_minus_z = axis_to_flower;
    if (tool_minus_z.norm() < 1e-6)
    {
      tool_minus_z = Eigen::Vector3d(0.0, 0.0, -1.0);
    }
    tool_minus_z.normalize();

    const Eigen::Vector3d z_axis = -tool_minus_z;
    Eigen::Vector3d reference = Eigen::Vector3d::UnitZ();
    if (std::abs(reference.dot(z_axis)) > 0.95)
    {
      reference = Eigen::Vector3d::UnitX();
    }

    Eigen::Vector3d x_axis = reference.cross(z_axis).normalized();
    Eigen::Vector3d y_axis = z_axis.cross(x_axis).normalized();

    Eigen::Matrix3d rotation;
    rotation.col(0) = x_axis;
    rotation.col(1) = y_axis;
    rotation.col(2) = z_axis;

    Eigen::Quaterniond orientation(rotation);
    orientation.normalize();

    const Eigen::Vector3d flower(flower_center.x, flower_center.y, flower_center.z);
    const Eigen::Vector3d tcp_position = flower + z_axis * offset_m;

    geometry_msgs::msg::Pose pose;
    pose.position.x = tcp_position.x();
    pose.position.y = tcp_position.y();
    pose.position.z = tcp_position.z();
    pose.orientation.x = orientation.x();
    pose.orientation.y = orientation.y();
    pose.orientation.z = orientation.z();
    pose.orientation.w = orientation.w();
    return pose;
  }

  bool isStateCollisionFree(const moveit::core::RobotState & state) const
  {
    if (planning_scene_ == nullptr)
    {
      return false;
    }

    return state.satisfiesBounds() && !planning_scene_->isStateColliding(state, planning_group_);
  }

  JointTargetSolution findBestExactJointTarget(
    const std::vector<geometry_msgs::msg::Pose> & pose_candidates,
    const moveit::core::RobotState & seed_state,
    const moveit::core::JointModelGroup * joint_model_group,
    const std::vector<double> & reference_values,
    double timeout_seconds) const
  {
    JointTargetSolution best_solution;

    for (std::size_t pose_index = 0; pose_index < pose_candidates.size(); ++pose_index)
    {
      const auto & pose_candidate = pose_candidates[pose_index];
      moveit::core::RobotState candidate_state = seed_state;
      const bool ik_ok = candidate_state.setFromIK(
        joint_model_group,
        pose_candidate,
        end_effector_link_,
        timeout_seconds);
      if (!ik_ok)
      {
        continue;
      }

      candidate_state.update();
      if (!isStateCollisionFree(candidate_state))
      {
        continue;
      }

      std::vector<double> target_values;
      candidate_state.copyJointGroupPositions(joint_model_group, target_values);
      const double candidate_score = scoreJointDistance(reference_values, target_values);
      if (candidate_score < best_solution.score)
      {
        best_solution.pose = pose_candidate;
        best_solution.joint_values = target_values;
        best_solution.score = candidate_score;
        best_solution.pose_index = static_cast<int>(pose_index);
        best_solution.valid = true;
      }
    }

    return best_solution;
  }

  static JointTargetSolution chooseBetterSolution(
    const JointTargetSolution & first,
    const JointTargetSolution & second)
  {
    if (!first.valid)
    {
      return second;
    }
    if (!second.valid)
    {
      return first;
    }
    return (second.score < first.score) ? second : first;
  }

  static int cyclicRollDistance(int first_index, int second_index, int roll_count)
  {
    if (roll_count <= 0)
    {
      return 0;
    }

    const int direct_distance = std::abs(first_index - second_index);
    return std::min(direct_distance, roll_count - direct_distance);
  }

  ApproachTargets selectApproachTargets(
    const geometry_msgs::msg::Point & flower_center,
    const geometry_msgs::msg::Point & anchor_point)
  {
    moveit::core::RobotState current_state(move_group_->getRobotModel());
    if (!buildRobotStateFromSimulatedJoints(current_state))
    {
      RCLCPP_WARN(get_logger(), "Simulated joint state unavailable while selecting pollination targets.");
      ApproachTargets empty_targets;
      return empty_targets;
    }

    const moveit::core::JointModelGroup * joint_model_group =
      current_state.getJointModelGroup(planning_group_);
    if (joint_model_group == nullptr)
    {
      RCLCPP_ERROR(get_logger(), "Joint model group %s does not exist.", planning_group_.c_str());
      ApproachTargets empty_targets;
      return empty_targets;
    }

    std::vector<double> current_values;
    current_state.copyJointGroupPositions(joint_model_group, current_values);

    const Eigen::Vector3d flower(flower_center.x, flower_center.y, flower_center.z);
    const Eigen::Vector3d anchor(anchor_point.x, anchor_point.y, anchor_point.z);
    Eigen::Vector3d line_to_flower = flower - anchor;
    if (line_to_flower.norm() < 1e-6)
    {
      line_to_flower = Eigen::Vector3d(0.0, 0.0, -1.0);
    }
    line_to_flower.normalize();

    Eigen::Vector3d lateral_to_flower = flower - anchor;
    lateral_to_flower.z() = 0.0;
    if (lateral_to_flower.norm() < 1e-6)
    {
      lateral_to_flower = Eigen::Vector3d(0.0, (flower.y() >= anchor.y()) ? 1.0 : -1.0, 0.0);
    }
    lateral_to_flower.normalize();

    const auto search_started = std::chrono::steady_clock::now();
    RCLCPP_INFO(
      get_logger(),
      "Beginning approach search for flower [%.3f, %.3f, %.3f] from anchor [%.3f, %.3f, %.3f] with %d roll samples.",
      flower_center.x,
      flower_center.y,
      flower_center.z,
      anchor_point.x,
      anchor_point.y,
      anchor_point.z,
      ik_roll_samples_);

    std::vector<std::pair<Eigen::Vector3d, std::string>> axis_candidates;
    axis_candidates.reserve(16);
    for (const double yaw_deg : {0.0, -20.0, 20.0, -35.0, 35.0})
    {
      const Eigen::Vector3d yaw_axis =
        Eigen::AngleAxisd(yaw_deg * M_PI / 180.0, Eigen::Vector3d::UnitZ()) * lateral_to_flower;
      for (const double downward_blend : {0.25, 0.40, 0.0})
      {
        const Eigen::Vector3d candidate =
          ((1.0 - downward_blend) * yaw_axis + downward_blend * Eigen::Vector3d(0.0, 0.0, -1.0))
            .normalized();
        axis_candidates.emplace_back(
          candidate,
          "lateral_yaw_" + std::to_string(static_cast<int>(yaw_deg)) +
            "_down_" + std::to_string(static_cast<int>(downward_blend * 100.0)));
      }
    }
    axis_candidates.emplace_back(line_to_flower, "anchor_line_fallback");

    ApproachTargets best_targets;
    ApproachTargets best_pre_only_targets;
    double best_score = std::numeric_limits<double>::infinity();
    double best_pre_only_score = std::numeric_limits<double>::infinity();
    int roll_evaluations = 0;
    int pre_solution_count = 0;
    int pollination_solution_count = 0;

    for (std::size_t axis_index = 0; axis_index < axis_candidates.size(); ++axis_index)
    {
      const auto & axis_to_flower = axis_candidates[axis_index].first;
      const auto & strategy = axis_candidates[axis_index].second;

      const auto base_pre_pose =
        makeTcpPoseFromAxis(flower_center, axis_to_flower, pre_approach_offset_m_);
      const auto base_pollination_pose =
        makeTcpPoseFromAxis(flower_center, axis_to_flower, pollination_offset_m_);

      const auto pre_candidates = buildRollVariants(base_pre_pose);
      const auto pollination_candidates = buildRollVariants(base_pollination_pose);
      const auto roll_count = std::min(pre_candidates.size(), pollination_candidates.size());

      for (std::size_t roll_index = 0; roll_index < roll_count; ++roll_index)
      {
        ++roll_evaluations;
        if ((roll_evaluations % 24) == 0)
        {
          RCLCPP_INFO(
            get_logger(),
            "Approach search progress: axis %zu/%zu, roll evaluations %d, current best %s.",
            axis_index + 1,
            axis_candidates.size(),
            roll_evaluations,
            best_targets.valid ? best_targets.strategy.c_str() : "none");
        }

        const auto pre_solution = findBestExactJointTarget(
          {pre_candidates[roll_index]},
          current_state,
          joint_model_group,
          current_values,
          ik_timeout_);
        if (!pre_solution.valid)
        {
          continue;
        }
        ++pre_solution_count;

        const double pre_only_score = pre_solution.score + 0.01 * static_cast<double>(roll_index);
        if (pre_only_score < best_pre_only_score)
        {
          best_pre_only_score = pre_only_score;
          best_pre_only_targets.pre_pose = pre_solution.pose;
          best_pre_only_targets.pollination_pose = pollination_candidates[roll_index];
          best_pre_only_targets.pre_joint_target = pre_solution.joint_values;
          best_pre_only_targets.pollination_joint_target.clear();
          best_pre_only_targets.axis_to_flower = axis_to_flower;
          best_pre_only_targets.strategy =
            strategy + "_pre_roll_" + std::to_string(roll_index) + "_pre_only";
          best_pre_only_targets.valid = true;
        }

        if (!prevalidate_pollination_pose_)
        {
          continue;
        }

        moveit::core::RobotState pre_state = current_state;
        pre_state.setJointGroupPositions(joint_model_group, pre_solution.joint_values);
        pre_state.update();

        const auto pollination_solution_from_pre = findBestExactJointTarget(
          pollination_candidates,
          pre_state,
          joint_model_group,
          pre_solution.joint_values,
          ik_timeout_);
        const auto pollination_solution_from_current = findBestExactJointTarget(
          pollination_candidates,
          current_state,
          joint_model_group,
          pre_solution.joint_values,
          ik_timeout_);
        const auto pollination_solution = chooseBetterSolution(
          pollination_solution_from_pre,
          pollination_solution_from_current);
        if (!pollination_solution.valid)
        {
          continue;
        }
        ++pollination_solution_count;

        const double transition_score =
          scoreJointDistance(pre_solution.joint_values, pollination_solution.joint_values);
        const int roll_distance = cyclicRollDistance(
          static_cast<int>(roll_index),
          pollination_solution.pose_index,
          static_cast<int>(roll_count));
        const double total_score =
          pre_solution.score + 0.35 * transition_score + 0.01 * static_cast<double>(roll_distance);
        if (total_score < best_score)
        {
          best_score = total_score;
          best_targets.pre_pose = pre_solution.pose;
          best_targets.pollination_pose = pollination_solution.pose;
          best_targets.pre_joint_target = pre_solution.joint_values;
          best_targets.pollination_joint_target = pollination_solution.joint_values;
          best_targets.axis_to_flower = axis_to_flower;
          best_targets.strategy =
            strategy + "_pre_roll_" + std::to_string(roll_index) +
            "_poll_roll_" + std::to_string(pollination_solution.pose_index);
          best_targets.valid = true;
          RCLCPP_INFO(
            get_logger(),
            "Improved target candidate: %s with score %.4f.",
            best_targets.strategy.c_str(),
            best_score);
        }
      }
    }

    if (!best_targets.valid && best_pre_only_targets.valid)
    {
      RCLCPP_WARN(
        get_logger(),
        "No 50 mm pollination IK candidate was prevalidated. Falling back to the best pre-alignment target %s and attempting staged in-motion refinement.",
        best_pre_only_targets.strategy.c_str());
      best_targets = best_pre_only_targets;
    }

    const auto search_elapsed =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - search_started).count();
    if (!best_targets.valid)
    {
      RCLCPP_WARN(
        get_logger(),
        "Approach search failed after %.2f s, %zu axis candidates, %d roll evaluations, %d pre IK hits and %d pollination IK hits.",
        search_elapsed,
        axis_candidates.size(),
        roll_evaluations,
        pre_solution_count,
        pollination_solution_count);
    }
    else
    {
      RCLCPP_INFO(
        get_logger(),
        "Approach search succeeded in %.2f s using %s after %zu axis candidates, %d roll evaluations, %d pre IK hits and %d pollination IK hits.",
        search_elapsed,
        best_targets.strategy.c_str(),
        axis_candidates.size(),
        roll_evaluations,
        pre_solution_count,
        pollination_solution_count);
    }

    return best_targets;
  }

  bool moveThroughPollinationApproach(
    const geometry_msgs::msg::Point & flower_center,
    const ApproachTargets & approach_targets)
  {
    const std::vector<std::pair<double, std::string>> staged_offsets{
      {0.09, "approach_90mm"},
      {0.07, "approach_70mm"}};

    for (const auto & staged_offset : staged_offsets)
    {
      const auto stage_pose =
        makeTcpPoseFromAxis(flower_center, approach_targets.axis_to_flower, staged_offset.first);
      if (
        !moveToPoseCandidates(
          buildRollVariants(stage_pose),
          staged_offset.second,
          goal_position_tolerance_m_,
          goal_orientation_tolerance_rad_))
      {
        return false;
      }
    }

    return true;
  }

  std::vector<geometry_msgs::msg::Pose> buildPollinationGoalCandidates(
    const geometry_msgs::msg::Point & flower_center,
    const ApproachTargets & approach_targets) const
  {
    std::vector<geometry_msgs::msg::Pose> candidates;
    candidates.push_back(approach_targets.pollination_pose);

    Eigen::Vector3d primary_axis = approach_targets.axis_to_flower;
    if (primary_axis.norm() < 1e-6)
    {
      primary_axis = Eigen::Vector3d(0.0, -1.0, 0.0);
    }
    primary_axis.normalize();

    Eigen::Vector3d reference = Eigen::Vector3d::UnitZ();
    if (std::abs(reference.dot(primary_axis)) > 0.9)
    {
      reference = Eigen::Vector3d::UnitX();
    }
    const Eigen::Vector3d lateral_axis = reference.cross(primary_axis).normalized();
    const Eigen::Vector3d vertical_axis = primary_axis.cross(lateral_axis).normalized();

    std::vector<Eigen::Vector3d> axis_candidates{primary_axis};
    for (const double tilt_deg : {12.0, -12.0, 20.0, -20.0})
    {
      axis_candidates.push_back(
        (Eigen::AngleAxisd(tilt_deg * M_PI / 180.0, lateral_axis) * primary_axis).normalized());
      axis_candidates.push_back(
        (Eigen::AngleAxisd(tilt_deg * M_PI / 180.0, vertical_axis) * primary_axis).normalized());
    }
    for (const double yaw_deg : {-12.0, 12.0})
    {
      axis_candidates.push_back(
        (Eigen::AngleAxisd(yaw_deg * M_PI / 180.0, Eigen::Vector3d::UnitZ()) * primary_axis)
          .normalized());
    }

    for (const auto & axis_candidate : axis_candidates)
    {
      const auto base_pose =
        makeTcpPoseFromAxis(flower_center, axis_candidate, pollination_offset_m_);
      const auto roll_variants = buildRollVariants(base_pose, std::min(ik_roll_samples_, 4));
      candidates.insert(candidates.end(), roll_variants.begin(), roll_variants.end());
    }

    return candidates;
  }

  bool moveToPreApproach(
    const geometry_msgs::msg::Pose & pre_pose,
    const std::string & stage_name)
  {
    std::vector<geometry_msgs::msg::Pose> candidates;
    const auto current_pose_stamped = move_group_->getCurrentPose(end_effector_link_);
    const auto current_orientation = toEigenQuaternion(current_pose_stamped.pose.orientation);
    const auto desired_orientation = toEigenQuaternion(pre_pose.orientation);

    auto current_orientation_pose = pre_pose;
    current_orientation_pose.orientation = current_pose_stamped.pose.orientation;
    candidates.push_back(current_orientation_pose);

    for (const double blend : {0.35, 0.7})
    {
      auto blended_pose = pre_pose;
      const auto blended_orientation = current_orientation.slerp(blend, desired_orientation).normalized();
      blended_pose.orientation = toGeometryQuaternion(blended_orientation);
      candidates.push_back(blended_pose);
    }

    const auto exact_candidates = buildRollVariants(pre_pose);
    candidates.insert(candidates.end(), exact_candidates.begin(), exact_candidates.end());

    return moveToPoseCandidates(
      candidates,
      stage_name,
      goal_position_tolerance_m_,
      pre_approach_orientation_tolerance_rad_);
  }

  bool moveToJointTarget(const std::vector<double> & target, const std::string & stage_name)
  {
    RCLCPP_INFO(get_logger(), "Planning joint stage %s.", stage_name.c_str());
    setMoveGroupStartStateFromSimulatedJoints();
    move_group_->setJointValueTarget(target);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto result = move_group_->plan(plan);
    if (result != moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_WARN(get_logger(), "Planning failed for stage %s.", stage_name.c_str());
      return false;
    }

    publishDisplayTrajectory(plan.trajectory);
    if (!simulateTrajectory(plan.trajectory, stage_name))
    {
      return false;
    }

    RCLCPP_INFO(get_logger(), "Completed stage %s.", stage_name.c_str());
    return true;
  }

  bool moveToPoseSeeded(const geometry_msgs::msg::Pose & target_pose, const std::string & stage_name)
  {
    return moveToPoseCandidates(
      buildRollVariants(target_pose),
      stage_name,
      goal_position_tolerance_m_,
      goal_orientation_tolerance_rad_);
  }

  bool moveToPoseCandidates(
    const std::vector<geometry_msgs::msg::Pose> & pose_candidates,
    const std::string & stage_name,
    double goal_position_tolerance,
    double goal_orientation_tolerance)
  {
    RCLCPP_INFO(
      get_logger(),
      "Planning pose-derived stage %s with %zu pose candidates.",
      stage_name.c_str(),
      pose_candidates.size());
    moveit::core::RobotState current_state(move_group_->getRobotModel());
    if (!buildRobotStateFromSimulatedJoints(current_state))
    {
      RCLCPP_WARN(get_logger(), "Simulated joint state unavailable before stage %s.", stage_name.c_str());
      return false;
    }

    const moveit::core::JointModelGroup * joint_model_group =
      current_state.getJointModelGroup(planning_group_);
    if (joint_model_group == nullptr)
    {
      RCLCPP_ERROR(get_logger(), "Joint model group %s does not exist.", planning_group_.c_str());
      return false;
    }

    std::vector<double> current_values;
    current_state.copyJointGroupPositions(joint_model_group, current_values);

    const auto best_solution = findBestExactJointTarget(
      pose_candidates,
      current_state,
      joint_model_group,
      current_values,
      ik_timeout_);

    if (!best_solution.valid)
    {
      RCLCPP_WARN(
        get_logger(),
        "Exact IK failed for stage %s. Trying pose-goal planning fallback.",
        stage_name.c_str());
      return moveToPoseGoalCandidates(
        pose_candidates,
        stage_name,
        goal_position_tolerance,
        goal_orientation_tolerance);
    }

    return moveToJointTarget(best_solution.joint_values, stage_name);
  }

  bool moveToPoseGoalCandidates(
    const std::vector<geometry_msgs::msg::Pose> & pose_candidates,
    const std::string & stage_name,
    double goal_position_tolerance,
    double goal_orientation_tolerance)
  {
    move_group_->setGoalPositionTolerance(goal_position_tolerance);
    move_group_->setGoalOrientationTolerance(goal_orientation_tolerance);

    for (const auto & pose_candidate : pose_candidates)
    {
      setMoveGroupStartStateFromSimulatedJoints();
      move_group_->clearPoseTargets();
      move_group_->setPoseTarget(pose_candidate, end_effector_link_);

      moveit::planning_interface::MoveGroupInterface::Plan plan;
      const auto result = move_group_->plan(plan);
      move_group_->clearPoseTargets();
      if (result != moveit::core::MoveItErrorCode::SUCCESS)
      {
        continue;
      }

      publishDisplayTrajectory(plan.trajectory);
      if (!simulateTrajectory(plan.trajectory, stage_name))
      {
        return false;
      }

      RCLCPP_INFO(get_logger(), "Completed stage %s with pose-goal fallback.", stage_name.c_str());
      return true;
    }

    RCLCPP_WARN(get_logger(), "Pose-goal planning failed for stage %s.", stage_name.c_str());
    return false;
  }

  bool poseHasIk(
    const geometry_msgs::msg::Pose & pose,
    int roll_samples,
    double timeout_seconds) const
  {
    moveit::core::RobotState current_state(move_group_->getRobotModel());
    if (!buildRobotStateFromSimulatedJoints(current_state))
    {
      return false;
    }

    const moveit::core::JointModelGroup * joint_model_group =
      current_state.getJointModelGroup(planning_group_);
    if (joint_model_group == nullptr)
    {
      return false;
    }

    std::vector<double> current_values;
    current_state.copyJointGroupPositions(joint_model_group, current_values);

    return findBestExactJointTarget(
             buildRollVariants(pose, roll_samples),
             current_state,
             joint_model_group,
             current_values,
             timeout_seconds)
      .valid;
  }

  bool executeCartesianSegment(
    const std::vector<geometry_msgs::msg::Pose> & waypoints,
    const std::string & stage_name)
  {
    RCLCPP_INFO(
      get_logger(),
      "Planning Cartesian stage %s with %zu waypoint(s).",
      stage_name.c_str(),
      waypoints.size());
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double fraction = move_group_->computeCartesianPath(
      waypoints,
      cartesian_step_m_,
      cartesian_jump_threshold_,
      trajectory,
      true);

    if (fraction < cartesian_min_fraction_)
    {
      RCLCPP_WARN(
        get_logger(),
        "Cartesian path fraction %.3f below threshold %.3f for stage %s.",
        fraction,
        cartesian_min_fraction_,
        stage_name.c_str());
      return false;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory = trajectory;
    publishDisplayTrajectory(plan.trajectory);
    if (!simulateTrajectory(plan.trajectory, stage_name))
    {
      return false;
    }

    RCLCPP_INFO(get_logger(), "Completed stage %s.", stage_name.c_str());
    return true;
  }

  void publishCurrentJointState()
  {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = now();
    msg.name = joint_names_;

    {
      std::scoped_lock lock(joint_state_mutex_);
      msg.position = current_joint_positions_;
      msg.velocity = current_joint_velocities_;
    }

    joint_state_pub_->publish(msg);
  }

  void publishDisplayTrajectory(const moveit_msgs::msg::RobotTrajectory & trajectory)
  {
    const auto current_state = move_group_->getCurrentState(2.0);
    if (!current_state)
    {
      RCLCPP_WARN(get_logger(), "Skipping display trajectory publish because current state is unavailable.");
      return;
    }

    moveit_msgs::msg::DisplayTrajectory display_msg;
    display_msg.model_id = "only_robot_arm";
    moveit::core::robotStateToRobotStateMsg(*current_state, display_msg.trajectory_start);
    display_msg.trajectory.push_back(trajectory);
    display_trajectory_pub_->publish(display_msg);
  }

  bool simulateTrajectory(
    const moveit_msgs::msg::RobotTrajectory & trajectory,
    const std::string & stage_name)
  {
    const auto & joint_trajectory = trajectory.joint_trajectory;
    if (joint_trajectory.joint_names.empty() || joint_trajectory.points.empty())
    {
      RCLCPP_WARN(get_logger(), "Trajectory is empty for stage %s.", stage_name.c_str());
      return false;
    }

    std::map<std::string, std::size_t> local_joint_index;
    for (std::size_t i = 0; i < joint_names_.size(); ++i)
    {
      local_joint_index[joint_names_[i]] = i;
    }

    const auto stage_start = std::chrono::steady_clock::now();
    std::chrono::duration<double> previous_time_from_start{0.0};

    for (const auto & point : joint_trajectory.points)
    {
      const auto target_time_from_start =
        std::chrono::duration<double>(rclcpp::Duration(point.time_from_start).seconds());
      const auto segment_sleep = target_time_from_start - previous_time_from_start;
      if (segment_sleep.count() > 0.0)
      {
        std::this_thread::sleep_until(stage_start + target_time_from_start);
      }

      {
        std::scoped_lock lock(joint_state_mutex_);
        for (std::size_t i = 0; i < joint_trajectory.joint_names.size(); ++i)
        {
          const auto it = local_joint_index.find(joint_trajectory.joint_names[i]);
          if (it == local_joint_index.end())
          {
            continue;
          }

          const auto local_index = it->second;
          current_joint_positions_[local_index] = point.positions[i];
          current_joint_velocities_[local_index] =
            (i < point.velocities.size()) ? point.velocities[i] : 0.0;
        }
      }

      publishCurrentJointState();
      previous_time_from_start = target_time_from_start;
    }

    {
      std::scoped_lock lock(joint_state_mutex_);
      std::fill(current_joint_velocities_.begin(), current_joint_velocities_.end(), 0.0);
    }
    publishCurrentJointState();
    return true;
  }

  std::vector<geometry_msgs::msg::Pose> buildRollVariants(
    const geometry_msgs::msg::Pose & base_pose) const
  {
    return buildRollVariants(base_pose, ik_roll_samples_);
  }

  std::vector<geometry_msgs::msg::Pose> buildRollVariants(
    const geometry_msgs::msg::Pose & base_pose,
    int sample_count) const
  {
    std::vector<geometry_msgs::msg::Pose> variants;
    const int resolved_sample_count = std::max(1, sample_count);
    variants.reserve(resolved_sample_count);

    const Eigen::Quaterniond base_orientation(
      base_pose.orientation.w,
      base_pose.orientation.x,
      base_pose.orientation.y,
      base_pose.orientation.z);

    for (int sample = 0; sample < resolved_sample_count; ++sample)
    {
      const double roll =
        (2.0 * M_PI * static_cast<double>(sample)) / static_cast<double>(resolved_sample_count);
      const Eigen::Quaterniond roll_rotation(Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitZ()));
      const Eigen::Quaterniond candidate_orientation = base_orientation * roll_rotation;

      auto pose_variant = base_pose;
      pose_variant.orientation.x = candidate_orientation.x();
      pose_variant.orientation.y = candidate_orientation.y();
      pose_variant.orientation.z = candidate_orientation.z();
      pose_variant.orientation.w = candidate_orientation.w();
      variants.push_back(pose_variant);
    }

    return variants;
  }

  static Eigen::Quaterniond toEigenQuaternion(const geometry_msgs::msg::Quaternion & quaternion_msg)
  {
    Eigen::Quaterniond quaternion(
      quaternion_msg.w,
      quaternion_msg.x,
      quaternion_msg.y,
      quaternion_msg.z);
    quaternion.normalize();
    return quaternion;
  }

  static geometry_msgs::msg::Quaternion toGeometryQuaternion(
    const Eigen::Quaterniond & quaternion)
  {
    geometry_msgs::msg::Quaternion quaternion_msg;
    quaternion_msg.x = quaternion.x();
    quaternion_msg.y = quaternion.y();
    quaternion_msg.z = quaternion.z();
    quaternion_msg.w = quaternion.w();
    return quaternion_msg;
  }

  void logPose(const std::string & label, const geometry_msgs::msg::Pose & pose) const
  {
    RCLCPP_INFO(
      get_logger(),
      "%s pose position=[%.3f, %.3f, %.3f] orientation=[%.4f, %.4f, %.4f, %.4f]",
      label.c_str(),
      pose.position.x,
      pose.position.y,
      pose.position.z,
      pose.orientation.x,
      pose.orientation.y,
      pose.orientation.z,
      pose.orientation.w);
  }

  static double scoreJointDistance(
    const std::vector<double> & current_values,
    const std::vector<double> & target_values)
  {
    double score = 0.0;
    const auto count = std::min(current_values.size(), target_values.size());
    for (std::size_t i = 0; i < count; ++i)
    {
      const double delta = wrapAngle(target_values[i] - current_values[i]);
      score += delta * delta;
    }
    return score;
  }

  void attemptReturnToInitial()
  {
    moveToJointTarget(initial_positions_, "return_initial");
  }

  void sleepSeconds(double seconds) const
  {
    std::this_thread::sleep_for(std::chrono::duration<double>(seconds));
  }

  static double wrapAngle(double angle)
  {
    while (angle > M_PI)
    {
      angle -= 2.0 * M_PI;
    }
    while (angle < -M_PI)
    {
      angle += 2.0 * M_PI;
    }
    return angle;
  }

  std::string world_frame_;
  std::string anchor_frame_;
  std::string planning_group_;
  std::string end_effector_link_;
  std::string flower_topic_;
  int flower_marker_id_{1};
  double ik_timeout_{0.1};
  bool prevalidate_pollination_pose_{false};
  double planning_time_{10.0};
  double max_velocity_scaling_{0.2};
  double max_acceleration_scaling_{0.2};
  double flower_message_timeout_sec_{1.0};
  int num_planning_attempts_{10};
  double base_clockwise_rotation_rad_{M_PI_2};
  int ik_roll_samples_{8};
  double pre_approach_offset_m_{0.12};
  double pollination_offset_m_{0.05};
  double cartesian_step_m_{0.005};
  double cartesian_jump_threshold_{0.0};
  double cartesian_min_fraction_{0.95};
  double goal_position_tolerance_m_{0.003};
  double goal_orientation_tolerance_rad_{0.03};
  double pre_approach_orientation_tolerance_rad_{0.45};
  double dwell_seconds_{1.0};
  double cycle_pause_seconds_{1.0};
  double joint_state_publish_rate_hz_{30.0};
  std::vector<std::string> joint_names_;
  std::vector<double> initial_positions_;
  std::vector<double> current_joint_positions_;
  std::vector<double> current_joint_velocities_;

  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr flower_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_trajectory_pub_;
  rclcpp::TimerBase::SharedPtr joint_state_timer_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  planning_scene::PlanningScenePtr planning_scene_;

  std::mutex flower_mutex_;
  mutable std::mutex joint_state_mutex_;
  FlowerState flower_state_;
  rclcpp::Time accepted_flower_marker_after_;
  std::atomic<bool> stop_requested_;
  std::thread worker_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<PollinationCycleNode>();
  node->initialize();

  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
