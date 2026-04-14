#ifndef BT_RELATIVE_NAV_HPP
#define BT_RELATIVE_NAV_HPP

#include "navigation_util.hpp"
#include "bt_config.hpp"

class RelativeNavNode : public BT::RosActionNode<opennav_docking_msgs::action::DockRobot> {

public: 
    RelativeNavNode(const std::string& name, const BT::NodeConfig& conf,
                    const RosNodeParams& params, BT::Blackboard::Ptr blackboard);

    static BT::PortsList providedPorts();

    bool setGoal(RosActionNode::Goal& goal) override;
    NodeStatus onResultReceived(const WrappedResult& wr) override;
    NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) override;
    void getInputPort();
    NodeStatus goalErrorDetect();

private:
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener listener;
    BT::Blackboard::Ptr blackboard;
    std::shared_ptr<rclcpp::Node> node;
    
    bool dock_finished;
    bool dock_error;
    bool isPureDocking;
    int dock_recov_times;
    
    double offset;
    std::string dock_type;
    geometry_msgs::msg::PoseStamped goal_pose;
    geometry_msgs::msg::PoseStamped robot_pose;
    geometry_msgs::msg::PoseStamped target_pose;
    RobotSide target_side;
    std::string frame_id;
};

#endif // BT_RELATIVE_NAV_HPP
