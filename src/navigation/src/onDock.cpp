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
    
    // Create publisher to notify camera team which side is docking
    dock_side_pub = node->create_publisher<std_msgs::msg::Int16>("/robot/dock_side", 10);
        
    // Load dock type parameters
    if (!node->has_parameter("normal_dock_type_y")) {
        node->declare_parameter("normal_dock_type_y", std::string("mission_dock_y_gentle"));
    }
    if (!node->has_parameter("normal_dock_type_x")) {
        node->declare_parameter("normal_dock_type_x", std::string("mission_dock_x_gentle"));
    }
    if (!node->has_parameter("cam_dock_type_y")) {
        node->declare_parameter("cam_dock_type_y", std::string("mission_dock_cam_y"));
    }
    if (!node->has_parameter("cam_dock_type_x")) {
        node->declare_parameter("cam_dock_type_x", std::string("mission_dock_cam_x"));
    }
    if (!node->has_parameter("fast_normal_dock_type_y")) {
        node->declare_parameter("fast_normal_dock_type_y", std::string("mission_dock_y_gentle"));
    }
    if (!node->has_parameter("fast_normal_dock_type_x")) {
        node->declare_parameter("fast_normal_dock_type_x", std::string("mission_dock_x_gentle"));
    }
    if (!node->has_parameter("fast_cam_dock_type_y")) {
        node->declare_parameter("fast_cam_dock_type_y", std::string("mission_dock_cam_y"));
    }
    if (!node->has_parameter("fast_cam_dock_type_x")) {
        node->declare_parameter("fast_cam_dock_type_x", std::string("mission_dock_cam_x"));
    }
    if (!node->has_parameter("slow_normal_dock_type_y")) {
        node->declare_parameter("slow_normal_dock_type_y", std::string("mission_dock_y_gentle"));
    }
    if (!node->has_parameter("slow_normal_dock_type_x")) {
        node->declare_parameter("slow_normal_dock_type_x", std::string("mission_dock_x_gentle"));
    }
    if (!node->has_parameter("slow_cam_dock_type_y")) {
        node->declare_parameter("slow_cam_dock_type_y", std::string("mission_dock_cam_y"));
    }
    if (!node->has_parameter("slow_cam_dock_type_x")) {
        node->declare_parameter("slow_cam_dock_type_x", std::string("mission_dock_cam_x"));
    }
    normal_dock_type_y_param = node->get_parameter("normal_dock_type_y").as_string();
    normal_dock_type_x_param = node->get_parameter("normal_dock_type_x").as_string();
    cam_dock_type_y_param = node->get_parameter("cam_dock_type_y").as_string();
    cam_dock_type_x_param = node->get_parameter("cam_dock_type_x").as_string();
    fast_normal_dock_type_y_param = node->get_parameter("fast_normal_dock_type_y").as_string();
    fast_normal_dock_type_x_param = node->get_parameter("fast_normal_dock_type_x").as_string();
    fast_cam_dock_type_y_param = node->get_parameter("fast_cam_dock_type_y").as_string();
    fast_cam_dock_type_x_param = node->get_parameter("fast_cam_dock_type_x").as_string();
    slow_normal_dock_type_y_param = node->get_parameter("slow_normal_dock_type_y").as_string();
    slow_normal_dock_type_x_param = node->get_parameter("slow_normal_dock_type_x").as_string();
    slow_cam_dock_type_y_param = node->get_parameter("slow_cam_dock_type_y").as_string();
    slow_cam_dock_type_x_param = node->get_parameter("slow_cam_dock_type_x").as_string();
    
    RCLCPP_INFO(node->get_logger(), "[OnDockAction] Initialized dock types: ny=%s, nx=%s, cy=%s, cx=%s", 
            normal_dock_type_y_param.c_str(), normal_dock_type_x_param.c_str(),
            cam_dock_type_y_param.c_str(), cam_dock_type_x_param.c_str());
    
    // Load staging distance parameters for each robot side
    if (!node->has_parameter("staging_dist_front")) {
        node->declare_parameter("staging_dist_front", 0.205);
    }
    if (!node->has_parameter("staging_dist_right")) {
        node->declare_parameter("staging_dist_right", 0.205);
    }
    if (!node->has_parameter("staging_dist_back")) {
        node->declare_parameter("staging_dist_back", 0.205);
    }
    if (!node->has_parameter("staging_dist_left")) {
        node->declare_parameter("staging_dist_left", 0.205);
    }
    
    staging_dist_front_ = node->get_parameter("staging_dist_front").as_double();
    staging_dist_right_ = node->get_parameter("staging_dist_right").as_double();
    staging_dist_back_ = node->get_parameter("staging_dist_back").as_double();
    staging_dist_left_ = node->get_parameter("staging_dist_left").as_double();
    
    RCLCPP_INFO(node->get_logger(), "[OnDockAction] Initialized staging distances: front=%.3f, right=%.3f, back=%.3f, left=%.3f",
                staging_dist_front_, staging_dist_right_, staging_dist_back_, staging_dist_left_);
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
    // base_direction: Direction enum value (0=NORTH, 1=EAST, 2=SOUTH, 3=WEST)
    // 
    // Robot coordinate system (ROS standard):
    //   At yaw=0°: +X points EAST, +Y points NORTH
    //   FRONT = +Y of robot
    //   RIGHT = +X of robot
    //   BACK  = -Y of robot
    //   LEFT  = -X of robot
    //
    // World yaw angles:
    //   0° = EAST, 90° = NORTH, 180° = WEST, 270° = SOUTH
    
    // Convert Direction enum to world yaw angle
    double target_yaw = 0.0;
    int dir = static_cast<int>(base_direction);
    switch (dir) {
        case 0: // NORTH
            target_yaw = PI / 2.0;  // 90°
            break;
        case 1: // EAST
            target_yaw = 0.0;  // 0°
            break;
        case 2: // SOUTH
            target_yaw = 3.0 * PI / 2.0;  // 270°
            break;
        case 3: // WEST
            target_yaw = PI;  // 180°
            break;
    }
    
    // Calculate robot yaw so the chosen side faces the target direction
    double yaw = 0.0;
    switch (side) {
        case RobotSide::FRONT:
            // FRONT (+Y) should face target_yaw
            // At yaw θ, +Y points to θ + 90°
            // We want θ + 90° = target_yaw, so θ = target_yaw - 90°
            yaw = target_yaw - PI / 2.0;
            break;
        case RobotSide::RIGHT:
            // RIGHT (+X) should face target_yaw
            // At yaw θ, +X points to θ
            // We want θ = target_yaw
            yaw = target_yaw;
            break;
        case RobotSide::BACK:
            // BACK (-Y) should face target_yaw
            // At yaw θ, -Y points to θ + 270° = θ - 90°
            // We want θ - 90° = target_yaw, so θ = target_yaw + 90°
            yaw = target_yaw + PI / 2.0;
            break;
        case RobotSide::LEFT:
            // LEFT (-X) should face target_yaw
            // At yaw θ, -X points to θ + 180°
            // We want θ + 180° = target_yaw, so θ = target_yaw - 180°
            yaw = target_yaw - PI;
            break;
    }
    
    // Normalize to [0, 2π)
    while (yaw < 0) yaw += 2.0 * PI;
    while (yaw >= 2.0 * PI) yaw -= 2.0 * PI;
    
    return yaw;
}

/**
 * @brief Calculate dock pose from map_points index, robot_config and robot side
 * 
 * map_points layout per point (5 values):
 *   [0] x position (final dock pose x)
 *   [1] y position (final dock pose y)
 *   [2] stage_dist (staging distance)
 *   [3] sign (1 or -1)
 *   [4] dock_type (0=dock_y, 1=dock_x, 2=cam_dock_y, 3=cam_dock_x)
 * 
 * get from robot_config:
 *   1. staging_dist_front
 *   2. staging_dist_right
 *   3. staging_dist_right
 *   4. staging_dist_back
 * 
 * Position z = stage_dist * sign (nav system uses this for staging)
 * Orientation: chosen side faces the target point
 */
geometry_msgs::msg::PoseStamped OnDockAction::calculateDockPose(int pose_idx, RobotSide robot_side, DockType chosen_dock_type) {
    geometry_msgs::msg::PoseStamped dock_pose;
    dock_pose.header.frame_id = "map";
    dock_pose.header.stamp = node->now();
    
    if (pose_idx >= static_cast<int>(map_point_list.size())) {
        RCLCPP_ERROR(node->get_logger(), "[OnDockAction] Invalid pose index %d", pose_idx);
        return dock_pose;
    }
    
    // Get position and staging info from map_points
    double x = map_point_list[pose_idx].x;
    double y = map_point_list[pose_idx].y;
    double stage_dist = map_point_list[pose_idx].staging_dist;
    double sign = map_point_list[pose_idx].sign;

    // If map_point.direction is explicitly set (not -1), apply staging directly to x/y
    // using targetDirection from DecisionCore.
    const bool use_xy_staging = (map_point_list[pose_idx].direction != -1);
    if (use_xy_staging) {
        switch (target_direction) {
            case Direction::NORTH:
                y -= stage_dist;
                sign = -1.0;
                break;
            case Direction::EAST:
                x -= stage_dist;
                sign = -1.0;
                break;
            case Direction::SOUTH:
                y += stage_dist;
                sign = 1.0;
                break;
            case Direction::WEST:
                x += stage_dist;
                sign = 1.0;
                break;
            default:
                DOCK_ERROR(node,
                           "Invalid target_direction=%d for pose=%s, x, y are not shifted",
                           static_cast<int>(target_direction),
                           goalPoseToString(static_cast<GoalPose>(pose_idx)).c_str());
                break;
        }
        DOCK_INFO(node,
                  "Applied XY staging offset for pose=%s, dir=%d, stage_dist=%.3f -> staged_xy=(%.3f, %.3f)",
                  goalPoseToString(static_cast<GoalPose>(pose_idx)).c_str(),
                  static_cast<int>(target_direction), stage_dist, x, y);
    }

    // When x/y staging is explicitly applied, set z offset to 0 to avoid double staging.
    double z_stage_for_nav = stage_dist * (-sign);
    if ( chosen_dock_type == DockType::CAM_DOCK_X || chosen_dock_type == DockType::CAM_DOCK_Y ) {
        switch ( target_side ) {
            case RobotSide::FRONT:
                z_stage_for_nav = staging_dist_front_ * (-sign);
                break;
            case RobotSide::RIGHT:
                z_stage_for_nav = staging_dist_right_ * (-sign);
                break;
            case RobotSide::BACK:
                z_stage_for_nav = staging_dist_back_ * (-sign);
                break;
            case RobotSide::LEFT:
                z_stage_for_nav = staging_dist_left_ * (-sign);
                break;
        }
    }
    else {
        if ( use_xy_staging ) {
            z_stage_for_nav = 0.0;
        }
    }
    
    // get input of targetDirection, also load MapPointList
    // use target_pose_idx to know which map_point
    // if map_point.direction is other than -1, shift x and y with the staging_dist with targetDirection
    // Set position: x, y are final dock pose, z = stage_dist * sign for nav system
    if(chosen_dock_type == DockType::MISSION_DOCK_Y || chosen_dock_type == DockType::CAM_DOCK_Y) {
        dock_pose.pose.position.z = z_stage_for_nav;
        dock_pose.pose.position.y = y;  // Adjust y for staging
        dock_pose.pose.position.x = x;
    }
    else if(chosen_dock_type == DockType::MISSION_DOCK_X || chosen_dock_type == DockType::CAM_DOCK_X) {
        dock_pose.pose.position.z = z_stage_for_nav;
        dock_pose.pose.position.x = x;  // Adjust x for staging
        dock_pose.pose.position.y = y;
    }
    else {  // For camera docks, use the exact position without staging offset
        dock_pose.pose.position.x = x;
        dock_pose.pose.position.y = y;
        dock_pose.pose.position.z = stage_dist * sign;  // Still set z for nav system, but it won't affect camera docking
    }
    
    // Calculate orientation based on robot side
    // The chosen side should face the target direction (from DecisionCore)
    double base_dir = static_cast<double>(target_direction);
    double yaw = getSideYaw(robot_side, base_dir);
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    dock_pose.pose.orientation = tf2::toMsg(q);
    
    RCLCPP_INFO(node->get_logger(), 
                "[OnDockAction] Dock pose: idx=%s, side=%d -> (%.3f, %.3f, z=%.3f) yaw=%.1f°",
                goalPoseToString(static_cast<GoalPose>(pose_idx)).c_str(), static_cast<int>(robot_side), dock_pose.pose.position.x, dock_pose.pose.position.y, dock_pose.pose.position.z, yaw * 180.0 / PI);
    
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
    
    if (!blackboard->get<std::vector<MapPoint>>("MapPointList", map_point_list)) {
        RCLCPP_ERROR(node->get_logger(), "[OnDockAction] Failed to get MapPointList from blackboard");
        return false;
    }
    
    std::string controller_type;
    if (!blackboard->get<std::string>("controller_type", controller_type) || controller_type.empty()) {
        controller_type = "Normal";
    }

    // Get dock_type from map_points and controller_type
    if (target_pose_idx < static_cast<int>(map_point_list.size())) {
        int dock_type_val = static_cast<int>(map_point_list[target_pose_idx].dock_type);
        const bool is_fast = (controller_type == "Fast");
        const bool is_slow = (controller_type == "Slow");

        switch (dock_type_val) {
            case 0: // MISSION_DOCK_Y
                dock_type = is_fast ? fast_normal_dock_type_y_param : (is_slow ? slow_normal_dock_type_y_param : normal_dock_type_y_param);
                break;
            case 1: // MISSION_DOCK_X
                dock_type = is_fast ? fast_normal_dock_type_x_param : (is_slow ? slow_normal_dock_type_x_param : normal_dock_type_x_param);
                break;
            case 2: // CAM_DOCK_Y
                dock_type = is_fast ? fast_cam_dock_type_y_param : (is_slow ? slow_cam_dock_type_y_param : cam_dock_type_y_param);
                break;
            case 3: // CAM_DOCK_X
                dock_type = is_fast ? fast_cam_dock_type_x_param : (is_slow ? slow_cam_dock_type_x_param : cam_dock_type_x_param);
                break;
            default:
                dock_type = normal_dock_type_y_param; // fallback
                DOCK_WARN(node, "Unknown dock_type value %d, using default", dock_type_val);
                break;
        }

        DOCK_INFO(node, "Selected dock_type=%s for controller_type=%s and dock_type_val=%d", dock_type.c_str(), controller_type.c_str(), dock_type_val);
    }
    
    getInput<bool>("isPureDocking", isPureDocking);
    
    // Reset timeout flag
    blackboard->set<bool>("Timeout", false);
    
    // Calculate dock pose from map_points
    DOCK_INFO(node, "Calculating dock pose for pose_idx=%s, side=%d, direction=%d, dock_type=%d",
              goalPoseToString(static_cast<GoalPose>(target_pose_idx)).c_str(), static_cast<int>(target_side), static_cast<int>(target_direction), 
              static_cast<int>(map_point_list[target_pose_idx].dock_type));
    goal_pose = calculateDockPose(target_pose_idx, target_side, static_cast<DockType>(map_point_list[target_pose_idx].dock_type));
    
    // Set up dock goal
    dock_goal.use_dock_id = false;
    dock_goal.dock_type = dock_type;
    dock_goal.dock_pose = goal_pose;
    dock_goal.max_staging_time = 1000.0;
    dock_goal.navigate_to_staging_pose = !isPureDocking;
    
    // Publish dock side to camera team
    std_msgs::msg::Int16 side_msg;
    side_msg.data = static_cast<int16_t>(target_side);
    dock_side_pub->publish(side_msg);
    DOCK_INFO(node, "Published dock_side=%d to /robot/dock_side", static_cast<int>(target_side));
    
    DOCK_INFO(node, "Goal set: pose_idx=%s, side=%d, dock_type=%s, pure=%s",
              goalPoseToString(static_cast<GoalPose>(target_pose_idx)).c_str(), static_cast<int>(target_side), 
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
        if (!blackboard->get<geometry_msgs::msg::PoseStamped>("robot_pose", robot_pose)) {
            RCLCPP_WARN(node->get_logger(), "[OnDockAction] Failed to get robot_pose from blackboard");
        }
        RCLCPP_ERROR(node->get_logger(), 
                     "[OnDockAction] Dock failed with error_code=%d, robot at (%.2f, %.2f)",
                     wr.result->error_code, robot_pose.pose.position.x, robot_pose.pose.position.y);
        
        return NodeStatus::SUCCESS;  // Return success to end the action, but report the error through blackboard and logs
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
    if (!blackboard->get<geometry_msgs::msg::PoseStamped>("robot_pose", robot_pose)) {
        RCLCPP_WARN(node->get_logger(), "[OnDockAction] Failed to get robot_pose from blackboard");
    }
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