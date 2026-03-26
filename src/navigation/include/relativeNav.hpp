#ifndef BT_RELATIVE_NAV_HPP
#define BT_RELATIVE_NAV_HPP

#include "navigation_util.hpp"

class RelativeNavNode : public BT::RosActionNode<opennav_docking_msgs::action::DockRobot> {

public: 
    RelativeNavNode(const std::string& name, const NodeConfig& conf, const RosNodeParams& params);

    static BT::PortsList providedPorts();

    bool setGoal(RosActionNode::Goal& goal) override;
    NodeStatus onResultReceived(const WrappedResult& wr) override;
    NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) override;
    void getInputPort();

private:
    std::shared_ptr<rclcpp::Node> node;
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener listener;
    bool nav_finished;
    bool nav_error;
    int nav_recov_times;
    double offset;
    std::string dock_type;
    geometry_msgs::msg::PoseStamped goal;
    geometry_msgs::msg::PoseStamped robot_pose;
    geometry_msgs::msg::PoseStamped target_pose;
    RobotSide target_side;
    std::string frame_id;
}