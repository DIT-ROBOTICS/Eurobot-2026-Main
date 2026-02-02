#include "rotate_action_node.hpp"

RotateActionNode::RotateActionNode(const std::string& name, const NodeConfig& conf, const RosNodeParams& params)
    : RosActionNode<opennav_docking_msgs::action::DockRobot>(name, conf, params), tf_buffer(params.nh.lock()->get_clock()), listener(tf_buffer) {
    node = params.nh.lock();
    node->get_parameter("frame_id", frame_id);
    tf_buffer.setUsingDedicatedThread(true);
    rotate_finished = false;
    rotate_error = false;
    rotate_recov_times = 0;
}

BT::PortsList RotateActionNode::providedPorts() {
    return {
        BT::InputPort<geometry_msgs::msg::PoseStamped>("goal"),
        BT::InputPort<double>("degree"),
        BT::OutputPort<geometry_msgs::msg::PoseStamped>("finish_pose") // finish_pose stand for the final result of this action, where does the robot at after finish this action call.
    };
}

bool RotateActionNode::setGoal(RosActionNode::Goal& rotate_goal) {
    auto goal_msg = getInput<geometry_msgs::msg::PoseStamped>("goal");
    double rad = getInput<double>("degree").value() * PI / 180;

    LocReceiver::UpdateRobotPose(robot_pose, tf_buffer, frame_id);
    tf2::Quaternion q_current;
    tf2::fromMsg(robot_pose.pose.orientation, q_current);
    double cur_yaw = tf2::impl::getYaw(q_current);
    double target_yaw = cur_yaw + rad;

    while (target_yaw > PI) target_yaw -= 2 * PI;
    while (target_yaw < -PI) target_yaw += 2 * PI;

    // message transport
    rclcpp::Time now = node->now();
    goal.header.stamp = now;
    goal.header.frame_id = goal_msg.value().header.frame_id;
    goal.pose = robot_pose.pose;
    goal.pose.position.z = 0;
    tf2::Quaternion q;
    q.setRPY(0, 0, target_yaw);
    goal.pose.orientation = tf2::toMsg(q);

    // rotate goal setup
    rotate_goal.dock_pose = goal;
    rotate_goal.use_dock_id = false;
    rotate_goal.dock_type = "dock_slow_precise";
    rotate_goal.max_staging_time = 2000.0;
    rotate_goal.navigate_to_staging_pose = true;

    RCLCPP_INFO(logger(), "RotateActionNode: Start Rotating from %f to %f (delta: %f deg)", 
                cur_yaw * 180 / PI, target_yaw * 180 / PI, rad * 180 / PI);
    return true;
}

NodeStatus RotateActionNode::onFeedback(const std::shared_ptr<const Feedback> feedback) {
    rotate_recov_times = feedback->num_retries;
    if(rotate_recov_times > 2) return goalErrorDetect();
    return NodeStatus::RUNNING;
}

NodeStatus RotateActionNode::onResultReceived(const WrappedResult& wr) {
    if (wr.code == rclcpp_action::ResultCode::SUCCEEDED) {
        rotate_finished = true;
        RCLCPP_INFO(logger(), "RotateActionNode: Rotation succeeded.");
        setOutput("finish_pose", robot_pose);
        return NodeStatus::SUCCESS;
    } else {
        rotate_error = true;
        RCLCPP_ERROR(logger(), "RotateActionNode: Rotation failed.");
        return goalErrorDetect();
    }
}

NodeStatus RotateActionNode::goalErrorDetect() {
    double rotate_dist_error_ = node->get_parameter("rotate_dist_error_tolerance").as_double();
    double rotate_ang_error_ = node->get_parameter("rotate_ang_error_tolerance").as_double();

    LocReceiver::UpdateRobotPose(robot_pose, tf_buffer, frame_id);
    // check the correctness of the final pose
    if (calculateDistance(robot_pose.pose, goal.pose) < rotate_dist_error_ && calculateAngleDifference(robot_pose.pose, goal.pose) < rotate_ang_error_) {
        rotate_finished = true;
        RCLCPP_INFO(logger(), "RotateActionNode: success! final_direction: (%f, %f)", robot_pose.pose.orientation.w, robot_pose.pose.orientation.z);
        // RCLCPP_INFO_STREAM(logger(), "z" << ConvertPoseFormat(robot_pose_).pose.position.z);
        RCLCPP_INFO_STREAM(logger(), "-----------------");
        setOutput<geometry_msgs::msg::PoseStamped>("finish_pose", ConvertPoseFormat(robot_pose));
        return NodeStatus::SUCCESS;
    } else {
        rotate_error = true;
        RCLCPP_INFO(logger(), "RotateActionNode: fail! final_direction: (%f, %f)", robot_pose.pose.orientation.w, robot_pose.pose.orientation.z);
        // RCLCPP_INFO_STREAM(logger(), "z" << ConvertPoseFormat(goal_).pose.position.z);
        RCLCPP_INFO_STREAM(logger(), "-----------------");
        setOutput<geometry_msgs::msg::PoseStamped>("finish_pose", ConvertPoseFormat(goal));
        return NodeStatus::SUCCESS;
    }
}
