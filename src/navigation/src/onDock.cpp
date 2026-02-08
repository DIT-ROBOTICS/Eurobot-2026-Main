#include "onDock.hpp"

// YELLOW colored logs for OnDockAction testing
#define DOCK_COLOR "\033[33m"  // Yellow
#define DOCK_RESET "\033[0m"
#define DOCK_INFO(node, fmt, ...) RCLCPP_INFO(node->get_logger(), DOCK_COLOR "[OnDockAction] " fmt DOCK_RESET, ##__VA_ARGS__)
#define DOCK_WARN(node, fmt, ...) RCLCPP_WARN(node->get_logger(), DOCK_COLOR "[OnDockAction] " fmt DOCK_RESET, ##__VA_ARGS__)
#define DOCK_ERROR(node, fmt, ...) RCLCPP_ERROR(node->get_logger(), DOCK_COLOR "[OnDockAction] " fmt DOCK_RESET, ##__VA_ARGS__)

OnDockAction::OnDockAction(const std::string& name, const NodeConfig& conf, 
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
    
    // Load map points from parameter
    loadMapPoints();
    
    RCLCPP_INFO(node->get_logger(), "[OnDockAction] Initialized with %zu map points", 
                map_points.size() / VALUES_PER_POINT);
}

void OnDockAction::loadMapPoints() {
    if (!node->has_parameter("map_points")) {
        node->declare_parameter("map_points", std::vector<double>{});
    }
    
    if (node->get_parameter("map_points", map_points)) {
        RCLCPP_DEBUG(node->get_logger(), "[OnDockAction] Loaded %zu map point values", 
                     map_points.size());
    } else {
        RCLCPP_ERROR(node->get_logger(), "[OnDockAction] Failed to load map_points!");
    }
}

BT::PortsList OnDockAction::providedPorts() {
    return {
        // Inputs from DecisionCore
        BT::InputPort<int>("targetPoseIdx", "Target pose index in map_points"),
        BT::InputPort<int>("targetPoseSideIdx", "Robot side index to face target"),
        BT::InputPort<int>("targetDirection", "Target direction"),
        // Optional overrides
        BT::InputPort<std::string>("dock_type", "dock_y_loose_linearBoost", "Dock type"),
        BT::InputPort<bool>("isPureDocking", false, "Whether to skip navigation to staging pose"),
        // Output
        BT::OutputPort<geometry_msgs::msg::PoseStamped>("finish_pose", "Final robot pose after docking")
    };
}

/**
 * @brief Calculate yaw angle based on robot side that should face the target
 * 
 * When a specific robot side needs to face the target:
 * - FRONT facing target: robot yaw = target direction
 * - RIGHT facing target: robot yaw = target direction - 90°
 * - BACK facing target: robot yaw = target direction + 180°
 * - LEFT facing target: robot yaw = target direction + 90°
 */
double OnDockAction::getSideYaw(RobotSide side, double base_direction) {
    // base_direction: 0=East(+X), 1=North(+Y), 2=West(-X), 3=South(-Y) (in 90° units)
    // 
    // Robot coordinate system:
    //   FRONT = +Y of robot
    //   RIGHT = +X of robot
    //   BACK  = -Y of robot
    //   LEFT  = -X of robot
    //
    // yaw_offset: rotation needed so that chosen side faces the target direction
    double yaw_offset = 0.0;
    
    switch (side) {
        case RobotSide::FRONT:
            // FRONT (+Y) should face target: rotate -90° so +Y aligns with base direction
            yaw_offset = -PI / 2.0;  // -90°
            break;
        case RobotSide::RIGHT:
            // RIGHT (+X) should face target: no rotation needed, +X already aligns
            yaw_offset = 0.0;
            break;
        case RobotSide::BACK:
            // BACK (-Y) should face target: rotate +90° so -Y aligns with base direction
            yaw_offset = PI / 2.0;  // +90°
            break;
        case RobotSide::LEFT:
            // LEFT (-X) should face target: rotate 180° so -X aligns with base direction
            yaw_offset = PI;  // 180°
            break;
    }
    
    // Convert base direction to radians and add offset
    double base_yaw = base_direction * PI / 2.0;
    return base_yaw + yaw_offset;
}

/**
 * @brief Calculate dock pose from map_points index and robot side
 * 
 * map_points layout per point (7 values):
 *   [0] x position
 *   [1] y position
 *   [2] direction (0=E, 1=N, 2=W, 3=S)
 *   [3] back_offset
 *   [4] shift
 *   [5] front_offset
 *   [6] dock_dist
 */
geometry_msgs::msg::PoseStamped OnDockAction::calculateDockPose(int pose_idx, RobotSide robot_side) {
    geometry_msgs::msg::PoseStamped dock_pose;
    dock_pose.header.frame_id = "map";
    dock_pose.header.stamp = node->now();
    
    // Calculate data index
    int data_idx = pose_idx * VALUES_PER_POINT;
    
    if (data_idx + VALUES_PER_POINT > static_cast<int>(map_points.size())) {
        RCLCPP_ERROR(node->get_logger(), "[OnDockAction] Invalid pose index %d", pose_idx);
        return dock_pose;
    }
    
    // Get base position and direction
    double base_x = map_points[data_idx + IDX_X];
    double base_y = map_points[data_idx + IDX_Y];
    double base_dir = map_points[data_idx + IDX_DIR];
    double back_offset = map_points[data_idx + IDX_BACK_OFFSET];
    double shift = map_points[data_idx + IDX_SHIFT];
    double front_offset = map_points[data_idx + IDX_FRONT_OFFSET];
    double dock_dist = map_points[data_idx + IDX_DOCK_DIST];
    
    // Select offset based on robot side
    // FRONT/BACK use front_offset/back_offset, LEFT/RIGHT would need different handling
    double offset = 0.0;
    switch (robot_side) {
        case RobotSide::FRONT:
            offset = front_offset;
            break;
        case RobotSide::BACK:
            offset = back_offset;
            break;
        case RobotSide::LEFT:
        case RobotSide::RIGHT:
            offset = front_offset; // Default to front for sides
            break;
    }
    
    // Apply offset based on direction (direction tells us which axis to offset on)
    // Direction encoding: 0=East (+X), 1=North (+Y), 2=West (-X), 3=South (-Y)
    double final_x = base_x;
    double final_y = base_y;
    
    int dir_int = static_cast<int>(base_dir);
    switch (dir_int) {
        case 0: // East: offset along X, shift along Y
            final_x += offset;
            final_y += shift;
            break;
        case 1: // North: offset along Y, shift along -X
            final_y += offset;
            final_x -= shift;
            break;
        case 2: // West: offset along -X, shift along -Y
            final_x -= offset;
            final_y -= shift;
            break;
        case 3: // South: offset along -Y, shift along X
            final_y -= offset;
            final_x += shift;
            break;
    }
    
    // Set position
    dock_pose.pose.position.x = final_x;
    dock_pose.pose.position.y = final_y;
    dock_pose.pose.position.z = dock_dist; // Store dock_dist in z for reference
    
    // Calculate orientation based on robot side
    double yaw = getSideYaw(robot_side, base_dir);
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    dock_pose.pose.orientation = tf2::toMsg(q);
    
    RCLCPP_INFO(node->get_logger(), 
                "[OnDockAction] Calculated dock pose: idx=%d, side=%d -> (%.3f, %.3f) yaw=%.1f°",
                pose_idx, static_cast<int>(robot_side), final_x, final_y, yaw * 180.0 / PI);
    
    return dock_pose;
}

bool OnDockAction::setGoal(RosActionNode::Goal& dock_goal) {
    // Get inputs from DecisionCore
    auto pose_idx_opt = getInput<int>("targetPoseIdx");
    auto side_idx_opt = getInput<int>("targetPoseSideIdx");
    auto dir_opt = getInput<int>("targetDirection");
    
    if (!pose_idx_opt) {
        RCLCPP_ERROR(node->get_logger(), "[OnDockAction] Missing targetPoseIdx input");
        return false;
    }
    
    target_pose_idx = pose_idx_opt.value();
    target_side = side_idx_opt ? static_cast<RobotSide>(side_idx_opt.value()) : RobotSide::FRONT;
    target_direction = dir_opt ? static_cast<Direction>(dir_opt.value()) : Direction::NORTH;
    
    // Get optional parameters
    getInput<std::string>("dock_type", dock_type);
    getInput<bool>("isPureDocking", isPureDocking);
    
    // Reset timeout flag
    blackboard->set<bool>("Timeout", false);
    
    // Calculate dock pose from map_points
    goal_pose = calculateDockPose(target_pose_idx, target_side);
    
    // Set up dock goal
    dock_goal.use_dock_id = false;
    dock_goal.dock_type = dock_type;
    dock_goal.dock_pose = goal_pose;
    dock_goal.max_staging_time = 1000.0;
    dock_goal.navigate_to_staging_pose = !isPureDocking;
    
    RCLCPP_INFO(node->get_logger(), 
                "[OnDockAction] Goal set: pose_idx=%d, side=%d, dock_type=%s, pure=%s",
                target_pose_idx, static_cast<int>(target_side), 
                dock_type.c_str(), isPureDocking ? "true" : "false");
    
    return true;
}

NodeStatus OnDockAction::onFeedback(const std::shared_ptr<const Feedback> feedback) {
    dock_recov_times = feedback->num_retries;
    
    RCLCPP_DEBUG(node->get_logger(), "[OnDockAction] Feedback: retries=%d", dock_recov_times);
    
    // If too many retries, consider it failed
    if (dock_recov_times > 2) {
        RCLCPP_WARN(node->get_logger(), 
                    "[OnDockAction] Too many retries, returning current pose: (%.2f, %.2f)",
                    robot_pose.pose.position.x, robot_pose.pose.position.y);
        setOutput<geometry_msgs::msg::PoseStamped>("finish_pose", ConvertPoseFormat(robot_pose));
        return NodeStatus::SUCCESS;
    }
    
    return NodeStatus::RUNNING;
}

NodeStatus OnDockAction::onResultReceived(const WrappedResult& wr) {
    RCLCPP_INFO(node->get_logger(), "[OnDockAction] Received dock result");
    dock_finished = true;
    
    if (!wr.result->success) {
        dock_error = true;
        
        if (wr.result->error_code == 905) {
            blackboard->set<bool>("Timeout", true);
        }
        
        blackboard->set<bool>("enable_vision_check", true);
        
        RCLCPP_ERROR(node->get_logger(), 
                     "[OnDockAction] Dock failed with error_code=%d, robot at (%.2f, %.2f)",
                     wr.result->error_code, robot_pose.pose.position.x, robot_pose.pose.position.y);
        
        return NodeStatus::FAILURE;
    }
    
    return goalErrorDetect();
}

NodeStatus OnDockAction::goalErrorDetect() {
    double dock_dist_error = 0.05;  // 5cm tolerance
    double dock_ang_error = 0.1;    // ~5.7° tolerance
    
    // Try to get parameters
    if (node->has_parameter("dock_dist_error_tolerance")) {
        dock_dist_error = node->get_parameter("dock_dist_error_tolerance").as_double();
    }
    if (node->has_parameter("dock_angle_error_tolerance")) {
        dock_ang_error = node->get_parameter("dock_angle_error_tolerance").as_double();
    }
    
    blackboard->set<bool>("enable_vision_check", true);    
    // Check pose accuracy
    double dist_error = calculateDistance(robot_pose.pose, goal_pose.pose);
    double ang_error = calculateAngleDifference(robot_pose.pose, goal_pose.pose);
    
    if (dist_error < dock_dist_error && ang_error < dock_ang_error) {
        RCLCPP_INFO(node->get_logger(), 
                    "[OnDockAction] Dock SUCCESS! Robot at (%.2f, %.2f), error: dist=%.3f, ang=%.3f",
                    robot_pose.pose.position.x, robot_pose.pose.position.y, dist_error, ang_error);
        setOutput<geometry_msgs::msg::PoseStamped>("finish_pose", ConvertPoseFormat(robot_pose));
        return NodeStatus::SUCCESS;
    } else {
        RCLCPP_WARN(node->get_logger(), 
                    "[OnDockAction] Dock completed but with error: dist=%.3f (tol=%.3f), ang=%.3f (tol=%.3f)",
                    dist_error, dock_dist_error, ang_error, dock_ang_error);
        setOutput<geometry_msgs::msg::PoseStamped>("finish_pose", ConvertPoseFormat(goal_pose));
        return NodeStatus::SUCCESS;  // Still return success, just report the error
    }
}