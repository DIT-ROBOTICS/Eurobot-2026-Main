#ifndef BT_NAV_ACTION_NODE_HPP
#define BT_NAV_ACTION_NODE_HPP

#include "navigation_util.hpp"

class NavigationActionNode : public BT::RosActionNode<opennav_docking_msgs::action::DockRobot> {

public:
    NavigationActionNode(const std::string& name, const NodeConfig& conf, const RosNodeParams& params);

    static BT::PortsList providedPorts();

    bool setGoal(RosActionNode::Goal& goal) override;
    NodeStatus onResultReceived(const WrappedResult& wr) override;
    NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) override;

private:
    NodeStatus goalErrorDetect();
    std::shared_ptr<rclcpp::Node> node;
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener listener;
    bool nav_finished;
    bool nav_error;
    int nav_recov_times;
    int mav_type;
    double offset = 0;
    double shift = 0;
    std::string dock_type;
    geometry_msgs::msg::PoseStamped goal;
    geometry_msgs::msg::PoseStamped robot_pose;
    std::string frame_id;
};

#endif // BT_NAV_ACTION_NODE_HPP