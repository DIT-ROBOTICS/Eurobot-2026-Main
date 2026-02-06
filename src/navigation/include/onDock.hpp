#ifndef ON_DOCK_HPP
#define ON_DOCK_HPP

#include "navigation_util.hpp"
#include "bt_config.hpp"

using namespace std;
using namespace BT;

/**
 * @brief OnDockAction - BT Action Node for docking based on DecisionCore output
 * 
 * Receives from DecisionCore:
 *   - targetPoseIdx: GoalPose index
 *   - targetPoseSideIdx: which robot side should face the target
 *   - targetDirection: approach direction (disabled for now)
 * 
 * Uses map_points to:
 *   1. Get target position (x, y)
 *   2. Get pre-configured offsets (back_offset, shift, front_offset, dock_dist)
 *   3. Calculate staging pose based on robot side
 */
class OnDockAction : public RosActionNode<opennav_docking_msgs::action::DockRobot> {
public:
    OnDockAction(const std::string& name, const NodeConfig& conf, 
                 const RosNodeParams& params, BT::Blackboard::Ptr blackboard);
    
    static PortsList providedPorts();
    bool setGoal(RosActionNode::Goal& goal) override;
    NodeStatus onResultReceived(const WrappedResult& wr) override;
    NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) override;

private:
    // Helper methods
    void loadMapPoints();
    geometry_msgs::msg::PoseStamped calculateDockPose(int pose_idx, RobotSide robot_side);
    double getSideYaw(RobotSide side, double base_direction);
    
    // Error detection
    NodeStatus goalErrorDetect();
    
    // Node references
    std::shared_ptr<rclcpp::Node> node;
    BT::Blackboard::Ptr blackboard;
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener listener;
    
    // Map points data (7 values per point)
    std::vector<double> map_points;
    static constexpr int VALUES_PER_POINT = 7;
    // Indices within each point
    static constexpr int IDX_X = 0;
    static constexpr int IDX_Y = 1;
    static constexpr int IDX_DIR = 2;
    static constexpr int IDX_BACK_OFFSET = 3;
    static constexpr int IDX_SHIFT = 4;
    static constexpr int IDX_FRONT_OFFSET = 5;
    static constexpr int IDX_DOCK_DIST = 6;
    
    // Dock state
    bool dock_finished;
    bool dock_error;
    bool isPureDocking;
    int dock_recov_times;
    std::string dock_type;
    std::string frame_id;
    
    // Pose data
    geometry_msgs::msg::PoseStamped goal_pose;
    geometry_msgs::msg::PoseStamped robot_pose;
    
    // Input values
    int target_pose_idx;
    RobotSide target_side;
    Direction target_direction;
};

#endif // ON_DOCK_HPP