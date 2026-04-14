#include "relativeNav.hpp"

RelativeNavNode::RelativeNavNode(const std::string& name, const NodeConfig& conf,
                                 const RosNodeParams& params, BT::Blackboard::Ptr blackboard)
    : RosActionNode<opennav_docking_msgs::action::DockRobot>(name, conf, params),
      tf_buffer(params.nh.lock()->get_clock()),
      listener(tf_buffer),
      blackboard(blackboard) {
        node = params.nh.lock();
        
        // Declare parameters with default values
        frame_id = "map";
        dock_finished = false;
        dock_error = false;
        isPureDocking = false;
        dock_recov_times = 0;
        dock_type = "dock_y_loose_Slow";
        RCLCPP_INFO(node->get_logger(), "[RelativeNavNode] Initialized with default dock_type: %s", dock_type.c_str());
}

BT::PortsList RelativeNavNode::providedPorts() {
    return {
        BT::InputPort<int>("targetPoseSideIdx", "Target approach direction (0-3)"),
        BT::InputPort<double>("offset", 0.2, "Distance to keep from the target pose"),
        BT::OutputPort<geometry_msgs::msg::PoseStamped>("finish_pose") 
    };
}

void RelativeNavNode::getInputPort() {
    auto side_opt = getInput<int>("targetPoseSideIdx");
    auto offset_opt = getInput<double>("offset");

    if (side_opt) {
        target_side = static_cast<RobotSide>(side_opt.value());
        RCLCPP_INFO(node->get_logger(), "[RelativeNavNode] Received targetPoseSideIdx: %d", side_opt.value());
    } else {
        RCLCPP_WARN(node->get_logger(), "[RelativeNavNode] targetPoseSideIdx not provided, defaulting to 0");
        target_side = RobotSide::FRONT; // default
    }
    
    if (offset_opt) {
        offset = offset_opt.value();
        RCLCPP_INFO(node->get_logger(), "[RelativeNavNode] Received offset: %.2f", offset);
    } else {
        RCLCPP_WARN(node->get_logger(), "[RelativeNavNode] offset not provided, defaulting to 0.2");
        offset = 0.2; // default
    }
}

bool RelativeNavNode::setGoal(RosActionNode::Goal& dock_goal) {
    getInputPort(); // Ensure we have the latest inputs
    
    // Get current robot pose
    if(!blackboard->get<geometry_msgs::msg::PoseStamped>("robot_pose", robot_pose)) {
        RCLCPP_ERROR(node->get_logger(), "[RelativeNavNode] Failed to get robot pose from blackboard");
        return false;
    }

    // look up transform from robot frame to the opposite of target_side
    std::string target_frame;
    switch (target_side) {
        case RobotSide::FRONT: target_frame = "back_side"; break;
        case RobotSide::BACK: target_frame = "front_side"; break;
        case RobotSide::LEFT: target_frame = "right_side"; break;
        case RobotSide::RIGHT: target_frame = "left_side"; break;
        default: target_frame = "base_footprint"; break;
    }

    // navigate to the target_frame
    try {
        geometry_msgs::msg::TransformStamped transform = tf_buffer.lookupTransform(frame_id, target_frame, tf2::TimePointZero);
        target_pose.header.frame_id = frame_id;
        target_pose.pose.position.x = transform.transform.translation.x;
        target_pose.pose.position.y = transform.transform.translation.y;
        target_pose.pose.position.z = 0.0; // keep same z
        target_pose.pose.orientation = transform.transform.rotation;
        RCLCPP_INFO(node->get_logger(), "[RelativeNavNode] Target pose in %s frame: (%.2f, %.2f)", frame_id.c_str(), target_pose.pose.position.x, target_pose.pose.position.y);
    } catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(node->get_logger(), "[RelativeNavNode] Failed to lookup transform to target frame %s: %s", target_frame.c_str(), ex.what());
        return false;
    }

    goal_pose = target_pose; // Store for error detection

    dock_goal.use_dock_id = false;
    dock_goal.dock_type = dock_type;
    dock_goal.dock_pose = target_pose;
    dock_goal.max_staging_time = 1000.0;
    dock_goal.navigate_to_staging_pose = true; // always navigate to staging pose for relative navigation
    RCLCPP_INFO(node->get_logger(), "[RelativeNavNode] Set goal to navigate to %s with offset %.2f", target_frame.c_str(), offset);
    return true;
}

NodeStatus RelativeNavNode::onFeedback(const std::shared_ptr<const Feedback> feedback) {
    dock_recov_times = feedback->num_retries;
    if (dock_recov_times > 2) {
        RCLCPP_WARN(node->get_logger(), 
                    "[RelativeNavNode] Too many retries, returning current pose: (%.2f, %.2f)",
                    robot_pose.pose.position.x, robot_pose.pose.position.y);
        setOutput<geometry_msgs::msg::PoseStamped>("finish_pose", ConvertPoseFormat(robot_pose));
        return NodeStatus::SUCCESS;
    }
    return NodeStatus::RUNNING;
}

NodeStatus RelativeNavNode::onResultReceived(const WrappedResult& wr) {
    RCLCPP_INFO(node->get_logger(), "[RelativeNavNode] Received dock result");
    dock_finished = true;
    
    if (!wr.result->success) {
        dock_error = true;
        
        if (wr.result->error_code == 905) {
            blackboard->set<bool>("Timeout", true);
        }
        
        if (!blackboard->get<geometry_msgs::msg::PoseStamped>("robot_pose", robot_pose)) {
            RCLCPP_WARN(node->get_logger(), "[RelativeNavNode] Failed to get robot_pose from blackboard");
        }
        RCLCPP_ERROR(node->get_logger(), 
                     "[RelativeNavNode] Dock failed with error_code=%d, robot at (%.2f, %.2f)",
                     wr.result->error_code, robot_pose.pose.position.x, robot_pose.pose.position.y);
        
        return NodeStatus::SUCCESS;  // Return success to end the action, but report the error through blackboard and logs
    }
    
    return goalErrorDetect();
}

NodeStatus RelativeNavNode::goalErrorDetect() {
    double dock_dist_error = 0.05;  // 5cm tolerance
    double dock_ang_error = 0.1;    // ~5.7° tolerance
    
    // Try to get parameters
    if (node->has_parameter("dock_dist_error_tolerance")) {
        dock_dist_error = node->get_parameter("dock_dist_error_tolerance").as_double();
    }
    if (node->has_parameter("dock_angle_error_tolerance")) {
        dock_ang_error = node->get_parameter("dock_angle_error_tolerance").as_double();
    }
    
    // Check pose accuracy
    if (!blackboard->get<geometry_msgs::msg::PoseStamped>("robot_pose", robot_pose)) {
        RCLCPP_WARN(node->get_logger(), "[RelativeNavNode] Failed to get robot_pose from blackboard");
    }
    double dist_error = calculateDistance(robot_pose.pose, goal_pose.pose);
    double ang_error = calculateAngleDifference(robot_pose.pose, goal_pose.pose);
    
    if (dist_error < dock_dist_error && ang_error < dock_ang_error) {
        RCLCPP_INFO(node->get_logger(), 
                    "[RelativeNavNode] Dock SUCCESS! Robot at (%.2f, %.2f), error: dist=%.3f, ang=%.3f",
                    robot_pose.pose.position.x, robot_pose.pose.position.y, dist_error, ang_error);
        setOutput<geometry_msgs::msg::PoseStamped>("finish_pose", ConvertPoseFormat(robot_pose));
        return NodeStatus::SUCCESS;
    } else {
        RCLCPP_WARN(node->get_logger(), 
                    "[RelativeNavNode] Dock result might be inaccurate. error: dist=%.3f, ang=%.3f",
                    dist_error, ang_error);
        setOutput<geometry_msgs::msg::PoseStamped>("finish_pose", ConvertPoseFormat(robot_pose));
        return NodeStatus::SUCCESS;
    }
}
