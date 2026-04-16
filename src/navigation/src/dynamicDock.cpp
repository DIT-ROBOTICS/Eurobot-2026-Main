#include "dynamicDock.hpp"
#include <cmath>

// MAGENTA colored logs for DynamicDock testing
#define DYN_COLOR "\033[35m"
#define DYN_RESET "\033[0m"
#define DYN_INFO(node, fmt, ...) RCLCPP_INFO(node->get_logger(), DYN_COLOR "[DynamicDock] " fmt DYN_RESET, ##__VA_ARGS__)
#define DYN_WARN(node, fmt, ...) RCLCPP_WARN(node->get_logger(), DYN_COLOR "[DynamicDock] " fmt DYN_RESET, ##__VA_ARGS__)
#define DYN_ERROR(node, fmt, ...) RCLCPP_ERROR(node->get_logger(), DYN_COLOR "[DynamicDock] " fmt DYN_RESET, ##__VA_ARGS__)

namespace {
constexpr double MAP_CENTER_X = 1.5;
constexpr double MAP_CENTER_Y = 1.0;
}

DynamicDock::DynamicDock(const std::string& name, const NodeConfig& conf,
                         const RosNodeParams& params, BT::Blackboard::Ptr blackboard)
    : RosActionNode<opennav_docking_msgs::action::DockRobot>(name, conf, params),
      tf_buffer(params.nh.lock()->get_clock()),
      listener(tf_buffer),
      blackboard(blackboard) {

    node = params.nh.lock();
    frame_id = "map";
    dock_finished = false;
    dock_error = false;
    isPureDocking = false;
    dock_recov_times = 0;
    dock_type = "dock_y_loose_linearBoost";

    dock_side_pub = node->create_publisher<std_msgs::msg::Int16>("/robot/dock_side", 10);

    if (!node->has_parameter("staging_dist_back")) {
        node->declare_parameter("staging_dist_back", 0.205);
    }
    staging_dist_back_ = node->get_parameter("staging_dist_back").as_double();

    DYN_INFO(node, "Initialized staging_dist_back=%.3f", staging_dist_back_);
}

BT::PortsList DynamicDock::providedPorts() {
    return {
        BT::InputPort<int>("targetPoseIdx", "Target pose index into robbery_poses"),
        BT::InputPort<int>("targetPoseSideIdx", "Robot side index (always BACK for steal)"),
        BT::InputPort<int>("targetDirection", "Ignored for DynamicDock"),
        BT::InputPort<std::string>("dock_type", "dock_y_loose_linearBoost", "Dock type"),
        BT::InputPort<bool>("isPureDocking", false, "Whether to skip navigation to staging pose"),
        BT::OutputPort<geometry_msgs::msg::PoseStamped>("finish_pose", "Final robot pose after docking")
    };
}

geometry_msgs::msg::PoseStamped DynamicDock::calculateDynamicDockPose(int pose_idx) {
    geometry_msgs::msg::PoseStamped dock_pose;
    dock_pose.header.frame_id = "map";
    dock_pose.header.stamp = node->now();

    geometry_msgs::msg::PoseArray robbery_poses;
    if (!blackboard->get<geometry_msgs::msg::PoseArray>("robbery_poses", robbery_poses)) {
        DYN_ERROR(node, "robbery_poses not found in blackboard");
        return dock_pose;
    }
    if (pose_idx < 0 || pose_idx >= static_cast<int>(robbery_poses.poses.size())) {
        DYN_ERROR(node, "Invalid pose_idx=%d for robbery_poses (size=%zu)",
                  pose_idx, robbery_poses.poses.size());
        return dock_pose;
    }

    const auto& target = robbery_poses.poses[pose_idx];
    double tx = target.position.x;
    double ty = target.position.y;

    tf2::Quaternion q_in;
    tf2::fromMsg(target.orientation, q_in);
    double theta = tf2::impl::getYaw(q_in);

    // Two BACK-alignment candidates along orientation axis.
    // At robot yaw φ, BACK points in direction (φ - π/2).
    // Option A: BACK along θ      → φ_A = θ + π/2 → staging = target - stage*(cos θ, sin θ)
    // Option B: BACK along θ + π  → φ_B = θ + 3π/2 → staging = target + stage*(cos θ, sin θ)
    double c = std::cos(theta);
    double s = std::sin(theta);
    double stage = staging_dist_back_;

    double staging_ax = tx - stage * c;
    double staging_ay = ty - stage * s;
    double yaw_a = theta + PI / 2.0;

    double staging_bx = tx + stage * c;
    double staging_by = ty + stage * s;
    double yaw_b = theta + 3.0 * PI / 2.0;

    double da = std::hypot(staging_ax - MAP_CENTER_X, staging_ay - MAP_CENTER_Y);
    double db = std::hypot(staging_bx - MAP_CENTER_X, staging_by - MAP_CENTER_Y);

    double chosen_x, chosen_y, chosen_yaw;
    const char* chosen_tag;
    if (da <= db) {
        chosen_x = staging_ax;
        chosen_y = staging_ay;
        chosen_yaw = yaw_a;
        chosen_tag = "A";
    } else {
        chosen_x = staging_bx;
        chosen_y = staging_by;
        chosen_yaw = yaw_b;
        chosen_tag = "B";
    }

    while (chosen_yaw < 0) chosen_yaw += 2.0 * PI;
    while (chosen_yaw >= 2.0 * PI) chosen_yaw -= 2.0 * PI;

    // Match onDock.cpp's xy-staging convention: (x, y) is the staged pose, z = 0.
    dock_pose.pose.position.x = chosen_x;
    dock_pose.pose.position.y = chosen_y;
    dock_pose.pose.position.z = 0.0;

    tf2::Quaternion q_out;
    q_out.setRPY(0, 0, chosen_yaw);
    dock_pose.pose.orientation = tf2::toMsg(q_out);

    DYN_INFO(node,
             "Target=(%.3f, %.3f, yaw=%.1f deg), option %s staging=(%.3f, %.3f) yaw=%.1f deg "
             "(dA=%.3f, dB=%.3f)",
             tx, ty, theta * 180.0 / PI, chosen_tag,
             chosen_x, chosen_y, chosen_yaw * 180.0 / PI, da, db);

    return dock_pose;
}

bool DynamicDock::setGoal(RosActionNode::Goal& dock_goal) {
    auto pose_idx_opt = getInput<int>("targetPoseIdx");
    if (!pose_idx_opt) {
        DYN_ERROR(node, "Missing targetPoseIdx input");
        return false;
    }
    target_pose_idx = pose_idx_opt.value();

    auto side_idx_opt = getInput<int>("targetPoseSideIdx");
    target_side = side_idx_opt ? static_cast<RobotSide>(side_idx_opt.value()) : RobotSide::BACK;

    getInput<std::string>("dock_type", dock_type);
    getInput<bool>("isPureDocking", isPureDocking);

    blackboard->set<bool>("Timeout", false);

    goal_pose = calculateDynamicDockPose(target_pose_idx);

    dock_goal.use_dock_id = false;
    dock_goal.dock_type = dock_type;
    dock_goal.dock_pose = goal_pose;
    dock_goal.max_staging_time = 1000.0;
    dock_goal.navigate_to_staging_pose = !isPureDocking;

    std_msgs::msg::Int16 side_msg;
    side_msg.data = static_cast<int16_t>(target_side);
    dock_side_pub->publish(side_msg);
    DYN_INFO(node, "Published dock_side=%d to /robot/dock_side", static_cast<int>(target_side));

    DYN_INFO(node, "Goal set: pose_idx=%d, side=%d, dock_type=%s, pure=%s",
             target_pose_idx, static_cast<int>(target_side),
             dock_type.c_str(), isPureDocking ? "true" : "false");

    return true;
}

NodeStatus DynamicDock::onFeedback(const std::shared_ptr<const Feedback> feedback) {
    dock_recov_times = feedback->num_retries;
    if (dock_recov_times > 2) {
        DYN_WARN(node, "Too many retries, returning current pose: (%.2f, %.2f)",
                 robot_pose.pose.position.x, robot_pose.pose.position.y);
        setOutput<geometry_msgs::msg::PoseStamped>("finish_pose", ConvertPoseFormat(robot_pose));
        return NodeStatus::SUCCESS;
    }
    return NodeStatus::RUNNING;
}

NodeStatus DynamicDock::onResultReceived(const WrappedResult& wr) {
    DYN_INFO(node, "Received dock result");
    dock_finished = true;

    if (!wr.result->success) {
        dock_error = true;
        if (wr.result->error_code == 905) {
            blackboard->set<bool>("Timeout", true);
        }
        blackboard->set<bool>("enable_vision_check", true);
        if (!blackboard->get<geometry_msgs::msg::PoseStamped>("robot_pose", robot_pose)) {
            DYN_WARN(node, "Failed to get robot_pose from blackboard");
        }
        DYN_ERROR(node, "Dock failed with error_code=%d, robot at (%.2f, %.2f)",
                  wr.result->error_code, robot_pose.pose.position.x, robot_pose.pose.position.y);
        return NodeStatus::SUCCESS;
    }

    return goalErrorDetect();
}

NodeStatus DynamicDock::goalErrorDetect() {
    double dock_dist_error = 0.05;
    double dock_ang_error = 0.1;

    if (node->has_parameter("dock_dist_error_tolerance")) {
        dock_dist_error = node->get_parameter("dock_dist_error_tolerance").as_double();
    }
    if (node->has_parameter("dock_angle_error_tolerance")) {
        dock_ang_error = node->get_parameter("dock_angle_error_tolerance").as_double();
    }

    blackboard->set<bool>("enable_vision_check", true);
    if (!blackboard->get<geometry_msgs::msg::PoseStamped>("robot_pose", robot_pose)) {
        DYN_WARN(node, "Failed to get robot_pose from blackboard");
    }
    double dist_error = calculateDistance(robot_pose.pose, goal_pose.pose);
    double ang_error = calculateAngleDifference(robot_pose.pose, goal_pose.pose);

    if (dist_error < dock_dist_error && ang_error < dock_ang_error) {
        DYN_INFO(node, "Dock SUCCESS! Robot at (%.2f, %.2f), error: dist=%.3f, ang=%.3f",
                 robot_pose.pose.position.x, robot_pose.pose.position.y, dist_error, ang_error);
        setOutput<geometry_msgs::msg::PoseStamped>("finish_pose", ConvertPoseFormat(robot_pose));
        return NodeStatus::SUCCESS;
    } else {
        DYN_WARN(node, "Dock completed with error: dist=%.3f (tol=%.3f), ang=%.3f (tol=%.3f)",
                 dist_error, dock_dist_error, ang_error, dock_ang_error);
        setOutput<geometry_msgs::msg::PoseStamped>("finish_pose", ConvertPoseFormat(goal_pose));
        return NodeStatus::SUCCESS;
    }
}
