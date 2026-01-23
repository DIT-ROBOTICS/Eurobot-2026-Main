#ifndef BT_NAV_ACTION_NODE_HPP
#define BT_NAV_ACTION_NODE_HPP

#include "navigation_util.hpp"

class Docking : public BT::RosActionNode<opennav_docking_msgs::action::DockRobot> {

public:
    Docking(const std::string& name, const NodeConfig& conf, const RosNodeParams& params, BT::Blackboard::Ptr blackboard);
    static PortsList providedPorts();
    bool setGoal(RosActionNode::Goal& goal) override;
    NodeStatus onResultReceived(const WrappedResult& wr) override;
    NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) override;

private:
    NodeStatus goalErrorDetect();
    std::shared_ptr<rclcpp::Node> node;
    BT::Blackboard::Ptr blackboard;
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener listener;
    bool dock_finished;
    bool dock_error;
    bool isPureDocking;
    double offset;
    double shift = 0;
    int dock_recov_times;
    std::string dock_type;
    geometry_msgs::msg::PoseStamped goal;
    geometry_msgs::msg::PoseStamped robot_pose;
    std::string frame_id;
};

#endif // BT_NAV_ACTION_NODE_HPP
