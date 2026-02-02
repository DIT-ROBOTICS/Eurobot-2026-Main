#include "nav_action_node.hpp"

NavigationActionNode::NavigationActionNode(const std::string& name, const NodeConfig& conf, const RosNodeParams& params)
    : RosActionNode<opennav_docking_msgs::action::DockRobot>(name, conf, params), tf_buffer(params.nh.lock()->get_clock()), listener(tf_buffer) {
        node = params.nh.lock();
        node->get_parameter("frame_id", frame_id);
        tf_buffer.setUsingDedicatedThread(true);
        nav_finished = false;
        nav_error = false;
        nav_recov_times = 0;
        shift = 0;
        offset = 0;
    }

BT::PortsList NavigationActionNode::providedPorts() {
    return {
        BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
        BT::InputPort<std::string>("dock_type"),
        BT::OutputPort<geometry_msgs::msg::PoseStamped>("finish_pose") // finish_pose stand for the final result of this action, where does the robot at after finish this action call.
    };
}

bool NavigationActionNode::setGoal(RosActionNode::Goal& dock_goal) {
    auto goal_msg = getInput<geometry_msgs::msg::PoseStamped>("goal");
    getInput<std::string>("dock_type", dock_type);

    // message transport
    rclcpp::Time now = node->now();
    goal.header.stamp = now;
    goal.header.frame_id = goal_msg.value().header.frame_id;
    goal.pose.position.x = goal_msg.value().pose.position.x;
    goal.pose.position.y = goal_msg.value().pose.position.y;
    goal.pose.position.z = 0.0;
    tf2::Quaternion q;
    q.setRPY(0, 0, goal_msg.value().pose.orientation.z * PI / 2);
    goal.pose.orientation = tf2::toMsg(q);
    
    // dock goal setup
    dock_goal.dock_type = dock_type;
    dock_goal.dock_pose = goal;
    dock_goal.use_dock_id = false;
    dock_goal.max_staging_time = 1000.0;
    dock_goal.navigate_to_staging_pose = true;
    
    RCLCPP_INFO(node->get_logger(), "NavigationActionNode: Goal set to x: %.2f, y: %.2f, yaw: %.2f", goal.pose.position.x, goal.pose.position.y, goal_msg.value().pose.orientation.z * 90); // z stand for yaw in 90 degree unit.
    return true;
}

NodeStatus NavigationActionNode::onFeedback(const std::shared_ptr<const Feedback> feedback) {
    nav_recov_times = feedback->num_retries;
    if (nav_recov_times > 2) return goalErrorDetect();
    return NodeStatus::RUNNING;
}

NodeStatus NavigationActionNode::onResultReceived(const WrappedResult& wr) {
    nav_finished = true;
    switch(wr.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(node->get_logger(), "NavigationActionNode: Goal succeeded.");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            nav_error = true;
            RCLCPP_ERROR(node->get_logger(), "NavigationActionNode: Goal aborted.");
            return NodeStatus::FAILURE;
        case rclcpp_action::ResultCode::CANCELED:
            nav_error = true;
            RCLCPP_ERROR(node->get_logger(), "NavigationActionNode: Goal canceled.");
            return NodeStatus::FAILURE;
        default:
            nav_error = true;
            RCLCPP_ERROR(node->get_logger(), "NavigationActionNode: Unknown result code.");
            return NodeStatus::FAILURE;
    }
    return goalErrorDetect();
}

NodeStatus NavigationActionNode::goalErrorDetect() {
    double nav_dist_error = node->get_parameter("nav_dist_error_torlence").as_double();
    double nav_angle_error = node->get_parameter("nav_angle_error_torlence").as_double();

    LocReceiver::UpdateRobotPose(robot_pose, tf_buffer, frame_id);
    if(calculateDistance(robot_pose.pose, goal.pose) < nav_dist_error && calculateAngleDifference(robot_pose.pose, goal.pose) < nav_angle_error) {
        RCLCPP_INFO_STREAM(logger(), "success! finish_pose: " << robot_pose.pose.position.x << ", " << robot_pose.pose.position.y << ", " << ConvertPoseFormat(robot_pose).pose.position.z);
        RCLCPP_INFO_STREAM(logger(), "-----------------");
        setOutput<geometry_msgs::msg::PoseStamped>("finish_pose", ConvertPoseFormat(robot_pose));
        return NodeStatus::SUCCESS;
    }
    else {
        nav_error = true;
        RCLCPP_INFO_STREAM(logger(), "fail! finish_pose: " << robot_pose.pose.position.x << ", " << robot_pose.pose.position.y << ", " << ConvertPoseFormat(robot_pose).pose.position.z);
        RCLCPP_INFO_STREAM(logger(), "z" << ConvertPoseFormat(goal).pose.position.z);
        RCLCPP_INFO_STREAM(logger(), "-----------------");
        setOutput<geometry_msgs::msg::PoseStamped>("finish_pose", ConvertPoseFormat(goal));
        return NodeStatus::SUCCESS;
    }
}
