#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <future>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <Eigen/Geometry>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/robot_state/conversions.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace
{
using namespace std::chrono_literals;

double normalizeAngle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

Eigen::Quaterniond toEigen(const geometry_msgs::msg::Quaternion & q)
{
  return Eigen::Quaterniond(q.w, q.x, q.y, q.z);
}

Eigen::Vector3d toEigen(const geometry_msgs::msg::Point & p)
{
  return Eigen::Vector3d(p.x, p.y, p.z);
}

geometry_msgs::msg::Quaternion toMsg(const Eigen::Quaterniond & q_in)
{
  Eigen::Quaterniond q = q_in.normalized();
  geometry_msgs::msg::Quaternion msg;
  msg.x = q.x();
  msg.y = q.y();
  msg.z = q.z();
  msg.w = q.w();
  return msg;
}
}  // namespace

class PollinationCycleNode : public rclcpp::Node
{
public:
  PollinationCycleNode()
  : Node("pollination_cycle_node"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_),
    started_(false),
    stop_requested_(false),
    axis_tip_initialized_(false)
  {
    world_frame_ = this->declare_parameter<std::string>("world_frame", "world");
    anchor_frame_ = this->declare_parameter<std::string>("anchor_frame", "link_1");
    tip_frame_ = this->declare_parameter<std::string>("tip_frame", "pollination_tip_link");
    joint6_frame_ = this->declare_parameter<std::string>("joint6_frame", "link_6");
    planning_group_ = this->declare_parameter<std::string>("planning_group", "arm");

    dx_world_ = this->declare_parameter<double>("dx_world", 0.1);
    dy_world_ = this->declare_parameter<double>("dy_world", -0.8);
    flower_center_z_ = this->declare_parameter<double>("flower_center_z", 0.1);

    pollination_offset_m_ = this->declare_parameter<double>("pollination_offset_m", 0.10);
    pre_pollination_offset_m_ = this->declare_parameter<double>("pre_pollination_offset_m", 0.12);
    dwell_sec_ = this->declare_parameter<double>("dwell_sec", 0.8);
    loop_pause_sec_ = this->declare_parameter<double>("loop_pause_sec", 0.6);
    base_clockwise_delta_rad_ =
      this->declare_parameter<double>("base_clockwise_delta_rad", M_PI / 2.0);

    planning_time_sec_ = this->declare_parameter<double>("planning_time_sec", 5.0);
    goal_position_tolerance_m_ = this->declare_parameter<double>("goal_position_tolerance_m", 0.003);
    goal_orientation_tolerance_rad_ =
      this->declare_parameter<double>("goal_orientation_tolerance_rad", 0.03);
    execution_publish_rate_hz_ = this->declare_parameter<double>("execution_publish_rate_hz", 100.0);
    ik_timeout_sec_ = this->declare_parameter<double>("ik_timeout_sec", 0.25);

    const auto default_joint_names = std::vector<std::string>{
      "joint_1", "joinit_2", "joint_3", "joint_4", "joint_5", "joint_6"};
    const auto default_joint_values = std::vector<double>{0.0, 2.087, -2.495, 0.085, 0.458, 0.017};
    auto contracted_joint_names =
      this->declare_parameter<std::vector<std::string>>("contracted_joint_names", default_joint_names);
    auto contracted_joint_values =
      this->declare_parameter<std::vector<double>>("contracted_joint_values", default_joint_values);

    const size_t count = std::min(contracted_joint_names.size(), contracted_joint_values.size());
    for (size_t i = 0; i < count; ++i) {
      contracted_joint_target_[contracted_joint_names[i]] = contracted_joint_values[i];
    }
    RCLCPP_INFO(
      this->get_logger(), "Contracted joint seed size: %zu", contracted_joint_target_.size());

    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::QoS(100));

    joint_state_timer_ = this->create_wall_timer(
      50ms, std::bind(&PollinationCycleNode::publishJointStateHeartbeat, this));
    startup_timer_ = this->create_wall_timer(500ms, std::bind(&PollinationCycleNode::startup, this));

    RCLCPP_INFO(
      this->get_logger(),
      "Pollination cycle node initialized. Waiting for MoveIt to become ready...");
  }

  ~PollinationCycleNode() override
  {
    stop_requested_.store(true);
    if (worker_thread_.joinable()) {
      worker_thread_.join();
    }
  }

private:
  void startup()
  {
    if (started_.exchange(true)) {
      return;
    }
    startup_timer_->cancel();

    worker_thread_ = std::thread(&PollinationCycleNode::workerMain, this);
  }

  void workerMain()
  {
    if (stop_requested_.load() || !rclcpp::ok()) {
      return;
    }

    try {
      move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), planning_group_);
    } catch (const std::exception & ex) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create MoveGroupInterface: %s", ex.what());
      return;
    }

    move_group_->setPoseReferenceFrame(world_frame_);
    move_group_->setEndEffectorLink(tip_frame_);
    move_group_->setPlanningTime(planning_time_sec_);
    move_group_->setGoalPositionTolerance(goal_position_tolerance_m_);
    move_group_->setGoalOrientationTolerance(goal_orientation_tolerance_rad_);
    move_group_->setMaxVelocityScalingFactor(0.3);
    move_group_->setMaxAccelerationScalingFactor(0.3);

    const auto * jmg =
      move_group_->getRobotModel()->getJointModelGroup(planning_group_);
    if (jmg != nullptr) {
      arm_joint_names_ = jmg->getVariableNames();
    }

    if (arm_joint_names_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Planning group '%s' has no joints.", planning_group_.c_str());
      return;
    }

    ik_client_ = this->create_client<moveit_msgs::srv::GetPositionIK>("/compute_ik");

    seedContractedState();

    if (!waitForTransformsReady(8.0)) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Required transforms did not become ready after seeding the contracted state.");
      return;
    }

    computeTipAxisInTipFrame();
    runLoop();
  }

  void seedContractedState()
  {
    std::vector<std::string> names;
    std::vector<double> positions;
    names.reserve(contracted_joint_target_.size());
    positions.reserve(contracted_joint_target_.size());
    for (const auto & [name, value] : contracted_joint_target_) {
      names.push_back(name);
      positions.push_back(value);
    }

    for (int i = 0; i < 40 && rclcpp::ok(); ++i) {
      publishJointState(names, positions);
      std::this_thread::sleep_for(25ms);
    }
    updateLastJointValues(names, positions);
  }

  bool waitForTransformsReady(double timeout_sec)
  {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_sec);
    while (std::chrono::steady_clock::now() < deadline && rclcpp::ok() && !stop_requested_.load()) {
      std::vector<std::string> names;
      std::vector<double> positions;
      names.reserve(contracted_joint_target_.size());
      positions.reserve(contracted_joint_target_.size());
      for (const auto & [name, value] : contracted_joint_target_) {
        names.push_back(name);
        positions.push_back(value);
      }
      publishJointState(names, positions);
      updateLastJointValues(names, positions);

      const auto tf_world_tip = lookupTransform(world_frame_, tip_frame_);
      const auto tf_world_joint6 = lookupTransform(world_frame_, joint6_frame_);
      const auto tf_world_anchor = lookupTransform(world_frame_, anchor_frame_);
      if (tf_world_tip && tf_world_joint6 && tf_world_anchor) {
        return true;
      }
      std::this_thread::sleep_for(40ms);
    }
    return false;
  }

  std::optional<geometry_msgs::msg::TransformStamped> lookupTransform(
    const std::string & target_frame, const std::string & source_frame)
  {
    try {
      return tf_buffer_.lookupTransform(target_frame, source_frame, tf2::TimePointZero);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
        this->get_logger(),
        "TF lookup failed (%s <- %s): %s",
        target_frame.c_str(), source_frame.c_str(), ex.what());
      return std::nullopt;
    }
  }

  void computeTipAxisInTipFrame()
  {
    auto tf_world_tip = lookupTransform(world_frame_, tip_frame_);
    auto tf_world_joint6 = lookupTransform(world_frame_, joint6_frame_);
    if (!tf_world_tip || !tf_world_joint6) {
      axis_tip_local_ = Eigen::Vector3d::UnitZ();
      axis_tip_initialized_ = false;
      RCLCPP_WARN(
        this->get_logger(),
        "Failed to infer joint_6 axis in tip frame. Fallback to tip Z-axis.");
      return;
    }

    const Eigen::Quaterniond q_world_tip = toEigen(tf_world_tip->transform.rotation);
    const Eigen::Quaterniond q_world_joint6 = toEigen(tf_world_joint6->transform.rotation);

    const Eigen::Vector3d axis_world = q_world_joint6 * Eigen::Vector3d::UnitZ();
    axis_tip_local_ = q_world_tip.inverse() * axis_world;
    axis_tip_local_.normalize();
    axis_tip_initialized_ = true;

    RCLCPP_INFO(
      this->get_logger(),
      "Tip-frame joint_6 axis inferred as [%.4f, %.4f, %.4f].",
      axis_tip_local_.x(), axis_tip_local_.y(), axis_tip_local_.z());
  }

  bool getFlowerCenter(geometry_msgs::msg::Point & flower_center)
  {
    auto tf_world_anchor = lookupTransform(world_frame_, anchor_frame_);
    if (!tf_world_anchor) {
      return false;
    }

    flower_center.x = tf_world_anchor->transform.translation.x + dx_world_;
    flower_center.y = tf_world_anchor->transform.translation.y + dy_world_;
    flower_center.z = flower_center_z_;
    return true;
  }

  bool buildPollinationTargetPoses(
    const geometry_msgs::msg::Point & flower_center,
    geometry_msgs::msg::PoseStamped & pre_pose,
    geometry_msgs::msg::PoseStamped & pollination_pose)
  {
    auto tf_world_tip = lookupTransform(world_frame_, tip_frame_);
    if (!tf_world_tip) {
      return false;
    }

    const Eigen::Quaterniond q_world_tip = toEigen(tf_world_tip->transform.rotation);
    const Eigen::Vector3d tip_pos_world(
      tf_world_tip->transform.translation.x,
      tf_world_tip->transform.translation.y,
      tf_world_tip->transform.translation.z);
    const Eigen::Vector3d flower_world = toEigen(flower_center);

    Eigen::Vector3d current_axis_world = q_world_tip * axis_tip_local_;
    current_axis_world.normalize();

    Eigen::Vector3d desired_axis_world = flower_world - tip_pos_world;
    if (desired_axis_world.norm() < 1e-6) {
      desired_axis_world = current_axis_world;
    }
    desired_axis_world.normalize();

    const Eigen::Quaterniond q_align =
      Eigen::Quaterniond::FromTwoVectors(current_axis_world, desired_axis_world);
    const Eigen::Quaterniond q_target = (q_align * q_world_tip).normalized();

    const Eigen::Vector3d pollination_pos =
      flower_world - desired_axis_world * pollination_offset_m_;
    const Eigen::Vector3d pre_pos =
      flower_world - desired_axis_world * pre_pollination_offset_m_;

    pre_pose.header.frame_id = world_frame_;
    pre_pose.pose.position.x = pre_pos.x();
    pre_pose.pose.position.y = pre_pos.y();
    pre_pose.pose.position.z = pre_pos.z();
    pre_pose.pose.orientation = toMsg(q_target);

    pollination_pose.header.frame_id = world_frame_;
    pollination_pose.pose.position.x = pollination_pos.x();
    pollination_pose.pose.position.y = pollination_pos.y();
    pollination_pose.pose.position.z = pollination_pos.z();
    pollination_pose.pose.orientation = toMsg(q_target);

    return true;
  }

  bool findReachablePollinationFallback(
    const geometry_msgs::msg::Point & flower_center,
    geometry_msgs::msg::PoseStamped & pollination_pose_out,
    std::map<std::string, double> & joint_target_out)
  {
    auto tf_world_tip = lookupTransform(world_frame_, tip_frame_);
    if (!tf_world_tip) {
      return false;
    }

    const Eigen::Quaterniond q_world_tip = toEigen(tf_world_tip->transform.rotation);
    const Eigen::Vector3d tip_pos_world(
      tf_world_tip->transform.translation.x,
      tf_world_tip->transform.translation.y,
      tf_world_tip->transform.translation.z);
    const Eigen::Vector3d flower_world = toEigen(flower_center);

    Eigen::Vector3d base_axis = flower_world - tip_pos_world;
    if (base_axis.norm() < 1e-6) {
      base_axis = q_world_tip * axis_tip_local_;
    }
    base_axis.normalize();

    Eigen::Vector3d current_axis_world = q_world_tip * axis_tip_local_;
    if (current_axis_world.norm() < 1e-6) {
      current_axis_world = base_axis;
    }
    current_axis_world.normalize();

    Eigen::Vector3d ref = std::abs(base_axis.z()) < 0.85 ? Eigen::Vector3d::UnitZ() : Eigen::Vector3d::UnitX();
    Eigen::Vector3d basis_u = base_axis.cross(ref);
    if (basis_u.norm() < 1e-6) {
      basis_u = base_axis.cross(Eigen::Vector3d::UnitY());
    }
    basis_u.normalize();
    Eigen::Vector3d basis_v = base_axis.cross(basis_u).normalized();

    const std::vector<std::pair<double, double>> tilt_candidates_deg = {
      {0.0, 0.0},
      {10.0, 0.0},
      {-10.0, 0.0},
      {0.0, 10.0},
      {0.0, -10.0},
      {15.0, 15.0},
      {15.0, -15.0},
      {-15.0, 15.0},
      {-15.0, -15.0},
      {20.0, 0.0},
      {-20.0, 0.0},
      {0.0, 20.0},
      {0.0, -20.0},
      {25.0, 10.0},
      {25.0, -10.0},
      {-25.0, 10.0},
      {-25.0, -10.0}};

    for (const auto & [tilt_u_deg, tilt_v_deg] : tilt_candidates_deg) {
      const double tilt_u = tilt_u_deg * M_PI / 180.0;
      const double tilt_v = tilt_v_deg * M_PI / 180.0;
      Eigen::Vector3d axis_candidate =
        Eigen::AngleAxisd(tilt_u, basis_u) * (Eigen::AngleAxisd(tilt_v, basis_v) * base_axis);
      if (axis_candidate.norm() < 1e-6) {
        continue;
      }
      axis_candidate.normalize();

      const Eigen::Quaterniond q_align =
        Eigen::Quaterniond::FromTwoVectors(current_axis_world, axis_candidate);
      const Eigen::Quaterniond q_target = (q_align * q_world_tip).normalized();
      const Eigen::Vector3d pollination_pos =
        flower_world - axis_candidate * pollination_offset_m_;

      geometry_msgs::msg::PoseStamped candidate_pose;
      candidate_pose.header.frame_id = world_frame_;
      candidate_pose.pose.position.x = pollination_pos.x();
      candidate_pose.pose.position.y = pollination_pos.y();
      candidate_pose.pose.position.z = pollination_pos.z();
      candidate_pose.pose.orientation = toMsg(q_target);

      std::map<std::string, double> candidate_joint_target;
      if (computeIkJointTarget(candidate_pose, candidate_joint_target)) {
        pollination_pose_out = candidate_pose;
        joint_target_out = candidate_joint_target;
        RCLCPP_INFO(
          this->get_logger(),
          "Recovered pollination IK with axis tilt (u=%.1f deg, v=%.1f deg).",
          tilt_u_deg, tilt_v_deg);
        return true;
      }
    }

    return false;
  }

  bool computeIkJointTarget(
    const geometry_msgs::msg::PoseStamped & pose_target,
    std::map<std::string, double> & joint_target_out)
  {
    if (!ik_client_->wait_for_service(2s)) {
      RCLCPP_WARN(this->get_logger(), "IK service /compute_ik not available yet.");
      return false;
    }

    const auto current_joint_map = currentOrContractedJointMap();

    auto try_ik =
      [&](const geometry_msgs::msg::PoseStamped & pose,
      const bool avoid_collisions,
      std::map<std::string, double> & out,
      int32_t & error_code) -> bool
      {
        moveit::core::RobotState seed_state(move_group_->getRobotModel());
        seed_state.setToDefaultValues();
        applyJointMapToRobotState(current_joint_map, seed_state);
        moveit_msgs::msg::RobotState seed_state_msg;
        moveit::core::robotStateToRobotStateMsg(seed_state, seed_state_msg);

        auto req = std::make_shared<moveit_msgs::srv::GetPositionIK::Request>();
        req->ik_request.group_name = planning_group_;
        req->ik_request.robot_state = seed_state_msg;
        req->ik_request.avoid_collisions = avoid_collisions;
        req->ik_request.pose_stamped = pose;
        req->ik_request.ik_link_name = tip_frame_;
        const auto timeout_sec = static_cast<int32_t>(std::floor(ik_timeout_sec_));
        const auto timeout_nsec = static_cast<uint32_t>(
          (ik_timeout_sec_ - static_cast<double>(timeout_sec)) * 1e9);
        req->ik_request.timeout.sec = timeout_sec;
        req->ik_request.timeout.nanosec = timeout_nsec;

        auto future = ik_client_->async_send_request(req);
        const auto wait_timeout =
          std::chrono::duration<double>(std::max(0.6, ik_timeout_sec_ * 3.0));
        if (future.wait_for(wait_timeout) != std::future_status::ready) {
          error_code = moveit_msgs::msg::MoveItErrorCodes::TIMED_OUT;
          RCLCPP_WARN(this->get_logger(), "IK service call timed out.");
          return false;
        }

        const auto response = future.get();
        error_code = response->error_code.val;
        if (error_code != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
          return false;
        }

        out.clear();
        for (const auto & joint_name : arm_joint_names_) {
          const auto it = std::find(
            response->solution.joint_state.name.begin(),
            response->solution.joint_state.name.end(),
            joint_name);
          if (it == response->solution.joint_state.name.end()) {
            error_code = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
            RCLCPP_WARN(
              this->get_logger(),
              "IK solution missing joint '%s'.",
              joint_name.c_str());
            return false;
          }
          const auto index =
            static_cast<size_t>(std::distance(response->solution.joint_state.name.begin(), it));
          out[joint_name] = response->solution.joint_state.position[index];
        }
        return true;
      };

    int32_t last_error_code = moveit_msgs::msg::MoveItErrorCodes::FAILURE;
    if (try_ik(pose_target, true, joint_target_out, last_error_code)) {
      return true;
    }

    const Eigen::Quaterniond q_target = toEigen(pose_target.pose.orientation);
    Eigen::Vector3d axis_world = q_target * axis_tip_local_;
    if (axis_world.norm() < 1e-9) {
      axis_world = q_target * Eigen::Vector3d::UnitZ();
    }
    axis_world.normalize();

    const std::vector<double> twist_candidates = {
      M_PI / 12.0, -M_PI / 12.0, M_PI / 6.0, -M_PI / 6.0, M_PI / 4.0, -M_PI / 4.0,
      M_PI / 3.0, -M_PI / 3.0, M_PI / 2.0, -M_PI / 2.0, 2.0 * M_PI / 3.0, -2.0 * M_PI / 3.0,
      3.0 * M_PI / 4.0, -3.0 * M_PI / 4.0, 5.0 * M_PI / 6.0, -5.0 * M_PI / 6.0, M_PI};

    std::map<std::string, double> best_solution;
    double best_score = std::numeric_limits<double>::max();
    bool found = false;

    auto score_solution = [&](const std::map<std::string, double> & solution) {
        double score = 0.0;
        for (const auto & joint_name : arm_joint_names_) {
          const auto it_ref = current_joint_map.find(joint_name);
          const auto it_sol = solution.find(joint_name);
          if (it_ref != current_joint_map.end() && it_sol != solution.end()) {
            score += std::abs(normalizeAngle(it_sol->second - it_ref->second));
          }
        }
        return score;
      };

    for (const bool avoid_collisions : {true, false}) {
      for (const double twist : twist_candidates) {
        geometry_msgs::msg::PoseStamped candidate_pose = pose_target;
        const Eigen::Quaterniond q_twist(Eigen::AngleAxisd(twist, axis_world));
        candidate_pose.pose.orientation = toMsg((q_twist * q_target).normalized());

        std::map<std::string, double> candidate_solution;
        if (!try_ik(candidate_pose, avoid_collisions, candidate_solution, last_error_code)) {
          continue;
        }

        const double score = score_solution(candidate_solution);
        if (!found || score < best_score) {
          best_score = score;
          best_solution = candidate_solution;
          found = true;
        }
      }

      if (found) {
        joint_target_out = best_solution;
        RCLCPP_INFO(
          this->get_logger(),
          "IK solved with axial twist sweep (avoid_collisions=%s).",
          avoid_collisions ? "true" : "false");
        return true;
      }
    }

    RCLCPP_WARN(
      this->get_logger(),
      "IK failed with MoveIt error code: %d",
      last_error_code);
    return false;
  }

  bool planAndSimulateToJointTarget(
    const std::map<std::string, double> & joint_target,
    const std::string & stage_name)
  {
    moveit::core::RobotState start_state(move_group_->getRobotModel());
    start_state.setToDefaultValues();
    applyJointMapToRobotState(currentOrContractedJointMap(), start_state);
    move_group_->setStartState(start_state);
    move_group_->setJointValueTarget(joint_target);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const bool planned = static_cast<bool>(move_group_->plan(plan));
    if (!planned) {
      RCLCPP_WARN(this->get_logger(), "[%s] planning failed.", stage_name.c_str());
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "[%s] planning success, simulating trajectory...", stage_name.c_str());
    return simulatePlan(plan);
  }

  bool planAndSimulateToPoseTarget(
    const geometry_msgs::msg::PoseStamped & pose_target,
    const std::string & stage_name)
  {
    std::map<std::string, double> ik_joint_target;
    if (!computeIkJointTarget(pose_target, ik_joint_target)) {
      RCLCPP_WARN(this->get_logger(), "[%s] IK failed.", stage_name.c_str());
      return false;
    }

    return planAndSimulateToJointTarget(ik_joint_target, stage_name);
  }

  bool simulatePlan(const moveit::planning_interface::MoveGroupInterface::Plan & plan)
  {
    const auto & jt = plan.trajectory.joint_trajectory;
    if (jt.points.empty() || jt.joint_names.empty()) {
      RCLCPP_WARN(this->get_logger(), "Planned trajectory is empty.");
      return false;
    }

    const auto start = std::chrono::steady_clock::now();
    const auto publish_period_ns = static_cast<int64_t>(1e9 / std::max(1.0, execution_publish_rate_hz_));
    const auto publish_period = std::chrono::nanoseconds(publish_period_ns);
    auto next_publish_tick = start;

    for (const auto & point : jt.points) {
      if (stop_requested_.load() || !rclcpp::ok()) {
        return false;
      }

      const auto target_ns = static_cast<int64_t>(point.time_from_start.sec) * 1000000000LL +
        static_cast<int64_t>(point.time_from_start.nanosec);
      const auto target_time = start + std::chrono::nanoseconds(target_ns);

      while (std::chrono::steady_clock::now() < target_time) {
        if (stop_requested_.load() || !rclcpp::ok()) {
          return false;
        }
        std::this_thread::sleep_for(2ms);
      }

      publishJointState(jt.joint_names, point.positions);
      updateLastJointValues(jt.joint_names, point.positions);

      if (std::chrono::steady_clock::now() < next_publish_tick) {
        std::this_thread::sleep_for(next_publish_tick - std::chrono::steady_clock::now());
      }
      next_publish_tick += publish_period;
    }

    for (int i = 0; i < 5; ++i) {
      std::vector<double> final_positions;
      final_positions.reserve(jt.joint_names.size());
      for (const auto & name : jt.joint_names) {
        final_positions.push_back(last_joint_values_[name]);
      }
      publishJointState(jt.joint_names, final_positions);
      std::this_thread::sleep_for(10ms);
    }

    return true;
  }

  void publishJointState(
    const std::vector<std::string> & joint_names,
    const std::vector<double> & positions)
  {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = this->now();
    msg.name = joint_names;
    msg.position = positions;
    joint_state_pub_->publish(msg);
  }

  void publishJointStateHeartbeat()
  {
    const auto joint_map = currentOrContractedJointMap();
    if (joint_map.empty()) {
      return;
    }

    std::vector<std::string> names;
    std::vector<double> positions;
    if (!arm_joint_names_.empty()) {
      names.reserve(arm_joint_names_.size());
      positions.reserve(arm_joint_names_.size());
      for (const auto & name : arm_joint_names_) {
        const auto it = joint_map.find(name);
        if (it != joint_map.end()) {
          names.push_back(name);
          positions.push_back(it->second);
        }
      }
    } else {
      names.reserve(joint_map.size());
      positions.reserve(joint_map.size());
      for (const auto & [name, value] : joint_map) {
        names.push_back(name);
        positions.push_back(value);
      }
    }

    if (!names.empty()) {
      publishJointState(names, positions);
    }
  }

  void updateLastJointValues(
    const std::vector<std::string> & joint_names,
    const std::vector<double> & positions)
  {
    std::scoped_lock<std::mutex> lock(joint_state_mutex_);
    const size_t count = std::min(joint_names.size(), positions.size());
    for (size_t i = 0; i < count; ++i) {
      last_joint_values_[joint_names[i]] = positions[i];
    }
  }

  std::map<std::string, double> currentOrContractedJointMap() const
  {
    std::scoped_lock<std::mutex> lock(joint_state_mutex_);
    if (last_joint_values_.empty()) {
      return contracted_joint_target_;
    }
    auto out = contracted_joint_target_;
    for (const auto & [name, value] : last_joint_values_) {
      out[name] = value;
    }
    return out;
  }

  void applyJointMapToRobotState(
    const std::map<std::string, double> & joint_map,
    moveit::core::RobotState & state) const
  {
    for (const auto & joint_name : arm_joint_names_) {
      const auto it = joint_map.find(joint_name);
      if (it != joint_map.end()) {
        state.setVariablePosition(joint_name, it->second);
      }
    }
    state.update();
  }

  bool sleepWithStop(double seconds)
  {
    const auto total = std::chrono::duration<double>(seconds);
    const auto tick = 100ms;
    auto elapsed = std::chrono::duration<double>(0.0);
    while (elapsed < total) {
      if (stop_requested_.load() || !rclcpp::ok()) {
        return false;
      }
      std::this_thread::sleep_for(tick);
      elapsed += tick;
    }
    return true;
  }

  void runLoop()
  {
    RCLCPP_INFO(this->get_logger(), "Pollination state machine started.");
    while (!stop_requested_.load() && rclcpp::ok()) {
      RCLCPP_INFO(this->get_logger(), "Stage 1/8: move to contracted state.");
      if (!planAndSimulateToJointTarget(contracted_joint_target_, "contracted")) {
        sleepWithStop(0.5);
        continue;
      }

      auto rotate_target = currentOrContractedJointMap();
      if (rotate_target.find("joint_1") == rotate_target.end()) {
        RCLCPP_ERROR(this->get_logger(), "Joint target missing joint_1.");
        sleepWithStop(0.5);
        continue;
      }
      rotate_target["joint_1"] =
        normalizeAngle(rotate_target["joint_1"] + base_clockwise_delta_rad_);

      RCLCPP_INFO(this->get_logger(), "Stage 2/8: rotate base clockwise 90 deg.");
      if (!planAndSimulateToJointTarget(rotate_target, "base_clockwise_90deg")) {
        sleepWithStop(0.5);
        continue;
      }

      geometry_msgs::msg::Point flower_center;
      if (!getFlowerCenter(flower_center)) {
        sleepWithStop(0.5);
        continue;
      }

      geometry_msgs::msg::PoseStamped pre_pose;
      geometry_msgs::msg::PoseStamped pollination_pose;
      if (!buildPollinationTargetPoses(flower_center, pre_pose, pollination_pose)) {
        sleepWithStop(0.5);
        continue;
      }

      RCLCPP_INFO(
        this->get_logger(),
        "Flower center in world: [%.4f, %.4f, %.4f], pollination distance %.3f m.",
        flower_center.x, flower_center.y, flower_center.z, pollination_offset_m_);

      RCLCPP_INFO(this->get_logger(), "Stage 3/8: move to pre-pollination pose.");
      if (!planAndSimulateToPoseTarget(pre_pose, "pre_pollination")) {
        sleepWithStop(0.5);
        continue;
      }

      RCLCPP_INFO(this->get_logger(), "Stage 4/8: move to pollination pose (50 mm offset).");
      if (!planAndSimulateToPoseTarget(pollination_pose, "pollination_pose")) {
        geometry_msgs::msg::PoseStamped fallback_pollination_pose;
        std::map<std::string, double> fallback_pollination_joint_target;
        if (
          !findReachablePollinationFallback(
            flower_center, fallback_pollination_pose, fallback_pollination_joint_target) ||
          !planAndSimulateToJointTarget(fallback_pollination_joint_target, "pollination_pose_fallback"))
        {
          sleepWithStop(0.5);
          continue;
        }
        pollination_pose = fallback_pollination_pose;
      }

      RCLCPP_INFO(this->get_logger(), "Stage 5/8: dwell for pollination.");
      if (!sleepWithStop(dwell_sec_)) {
        return;
      }

      RCLCPP_INFO(this->get_logger(), "Stage 6/8: retreat to pre-pollination pose.");
      if (!planAndSimulateToPoseTarget(pre_pose, "retreat")) {
        sleepWithStop(0.5);
        continue;
      }

      RCLCPP_INFO(this->get_logger(), "Stage 7/8: return to contracted state.");
      if (!planAndSimulateToJointTarget(contracted_joint_target_, "return_contracted")) {
        sleepWithStop(0.5);
        continue;
      }

      RCLCPP_INFO(this->get_logger(), "Stage 8/8: cycle pause.");
      if (!sleepWithStop(loop_pause_sec_)) {
        return;
      }
    }
  }

private:
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
  rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedPtr ik_client_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;

  rclcpp::TimerBase::SharedPtr joint_state_timer_;
  rclcpp::TimerBase::SharedPtr startup_timer_;
  std::thread worker_thread_;

  std::atomic<bool> started_;
  std::atomic<bool> stop_requested_;

  std::string world_frame_;
  std::string anchor_frame_;
  std::string tip_frame_;
  std::string joint6_frame_;
  std::string planning_group_;

  double dx_world_;
  double dy_world_;
  double flower_center_z_;

  double pollination_offset_m_;
  double pre_pollination_offset_m_;
  double dwell_sec_;
  double loop_pause_sec_;
  double base_clockwise_delta_rad_;

  double planning_time_sec_;
  double goal_position_tolerance_m_;
  double goal_orientation_tolerance_rad_;
  double execution_publish_rate_hz_;
  double ik_timeout_sec_;

  std::vector<std::string> arm_joint_names_;
  std::map<std::string, double> contracted_joint_target_;
  std::map<std::string, double> last_joint_values_;
  mutable std::mutex joint_state_mutex_;

  Eigen::Vector3d axis_tip_local_;
  bool axis_tip_initialized_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PollinationCycleNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
