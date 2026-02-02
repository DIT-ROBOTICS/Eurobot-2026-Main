#include "dock_action_node.hpp"

using namespace std;

Docking::Docking(const std::string& name, const NodeConfig& conf, const RosNodeParams& params, BT::Blackboard::Ptr blackboard)
    : RosActionNode<opennav_docking_msgs::action::DockRobot>(name, conf, params), tf_buffer(params.nh.lock()->get_clock()), listener(tf_buffer), blackboard(blackboard) {
        node = params.nh.lock();
        node->get_parameter("frame_id", frame_id);
        tf_buffer.setUsingDedicatedThread(true);
        dock_finished = false;
        dock_error = false;
        isPureDocking = true;
        dock_recov_times = 0;
        shift = 0;
        offset = 0;
}


BT::PortsList Docking::providedPorts() {
    return {
        BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
        BT::InputPort<double>("offset"),
        BT::InputPort<double>("shift"),
        BT::InputPort<std::string>("dock_type"),
        BT::InputPort<bool>("isPureDocking"),
        BT::OutputPort<geometry_msgs::msg::PoseStamped>("finish_pose") // finish_pose stand for the final result of this action, where does the robot at after finish this action call.
    };
}   

bool Docking::setGoal(RosActionNode::Goal& dock_goal) {
    auto goal_msg = getInput<geometry_msgs::msg::PoseStamped>("goal");
    getInput<std::string>("dock_type", dock_type);
    getInput<bool>("isPureDocking", isPureDocking);
    getInput<double>("offset", offset);
    getInput<double>("shift", shift);
    blackboard->set<bool>("Timeout", false);

    // message transport
    rclcpp::Time now = node->now();
    goal.header.stamp = now;
    goal.header.frame_id = goal_msg.value().header.frame_id;
    goal.pose.position.x = goal_msg.value().pose.position.x;
    goal.pose.position.y = goal_msg.value().pose.position.y;
    tf2::Quaternion q;
    q.setRPY(0, 0, goal_msg.value().pose.orientation.z * PI / 2);
    goal.pose.orientation = tf2::toMsg(q);

    if(dock_type == "mission_dock_x" || dock_type.substr(0, 6) == "dock_x") {
        RCLCPP_INFO(node->get_logger(), "Docking: Using dock_type %s", dock_type.c_str());
        goal.pose.position.x += offset;
        goal.pose.position.y += shift;
    } else if(dock_type == "mission_dock_y" || dock_type.substr(0, 6) == "dock_y") {
        RCLCPP_INFO(node->get_logger(), "Docking: Using dock_type %s", dock_type.c_str());
        goal.pose.position.x += shift;
        goal.pose.position.y += offset;
    } else {
        RCLCPP_WARN(node->get_logger(), "Docking: Unknown dock_type %s, using default dock_x", dock_type.c_str());
        return false;
    }
    goal.pose.position.z = offset;
    
    // dock goal setup
    dock_goal.use_dock_id = false;
    dock_goal.dock_type = dock_type;
    dock_goal.dock_pose = goal;
    dock_goal.max_staging_time = 1000.0;
    dock_goal.navigate_to_staging_pose = !isPureDocking;
    
    RCLCPP_INFO(node->get_logger(), "Docking: Goal set to x: %.2f, y: %.2f, yaw: %.2f", goal.pose.position.x, goal.pose.position.y, goal_msg.value().pose.orientation.z * 90); // z stand for yaw in 90 degree unit.
    return true;
}

NodeStatus Docking::onFeedback(const std::shared_ptr<const Feedback> feedback) {
    dock_recov_times = feedback->num_retries;
    if (dock_recov_times > 2) {
        LocReceiver::UpdateRobotPose(robot_pose, tf_buffer, frame_id);
        RCLCPP_INFO_STREAM(logger(), "success! finish_pose: " << robot_pose.pose.position.x << ", " << robot_pose.pose.position.y << ", " << ConvertPoseFormat(robot_pose).pose.position.z);
        RCLCPP_INFO_STREAM(logger(), "-----------------");
        setOutput<geometry_msgs::msg::PoseStamped>("finish_pose", ConvertPoseFormat(robot_pose));
        return NodeStatus::SUCCESS;
    }
    return NodeStatus::RUNNING;
}

NodeStatus Docking::onResultReceived(const WrappedResult& wr) {
    RCLCPP_INFO_STREAM(node->get_logger(), "Docking: get dock result");
    dock_finished = true;
    if (!wr.result->success) {
        dock_error = true;
        dock_finished = true;
        if (wr.result->error_code == 905)
            blackboard->set<bool>("Timeout", true);
        blackboard->set<bool>("enable_vision_check", true);
        LocReceiver::UpdateRobotPose(robot_pose, tf_buffer, frame_id);
        RCLCPP_INFO_STREAM(logger(), "error code: " << wr.result->error_code << " RETURN FAILURE! finish_pose: " << robot_pose.pose.position.x << ", " << robot_pose.pose.position.y << ", " << robot_pose.pose.position.z);
        RCLCPP_INFO_STREAM(logger(), "-----------------");
        return NodeStatus::FAILURE;
    }
    return goalErrorDetect();
}

NodeStatus Docking::goalErrorDetect() {
    double dock_dist_error_ = node->get_parameter("dock_dist_error_torlence").as_double();
    double dock_ang_error_ = node->get_parameter("dock_angle_error_torlence").as_double();
    blackboard->set<bool>("enable_vision_check", true);

    // check the correctness of the final pose
    LocReceiver::UpdateRobotPose(robot_pose, tf_buffer, frame_id);
    if (calculateDistance(robot_pose.pose, goal.pose) < dock_dist_error_ && calculateAngleDifference(robot_pose.pose, goal.pose) < dock_ang_error_) {
        RCLCPP_INFO_STREAM(logger(), "success! finish_pose: " << robot_pose.pose.position.x << ", " << robot_pose.pose.position.y << ", " << ConvertPoseFormat(robot_pose).pose.position.z);
        RCLCPP_INFO_STREAM(logger(), "-----------------");
        setOutput<geometry_msgs::msg::PoseStamped>("finish_pose", ConvertPoseFormat(robot_pose));
        return NodeStatus::SUCCESS;
    } else {
        dock_error = true;
        RCLCPP_INFO_STREAM(logger(), "fail! finish_pose: " << robot_pose.pose.position.x << ", " << robot_pose.pose.position.y << ", " << ConvertPoseFormat(robot_pose).pose.position.z);
        RCLCPP_INFO_STREAM(logger(), "-----------------");
        setOutput<geometry_msgs::msg::PoseStamped>("finish_pose", ConvertPoseFormat(goal));
        return NodeStatus::SUCCESS;
    }
}