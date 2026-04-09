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
    geometry_msgs::msg::PoseStamped calculateDockPose(int pose_idx, RobotSide robot_side, DockType chosen_dock_type);
    double getSideYaw(RobotSide side, double base_direction);
    
    // Error detection
    NodeStatus goalErrorDetect();
    
    // Node references
    std::shared_ptr<rclcpp::Node> node;
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener listener;
    BT::Blackboard::Ptr blackboard;
    
    // Publisher to notify camera team which side is docking
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr dock_side_pub;
    
    // Map points data
    std::vector<MapPoint> map_point_list;
    
    // Dock state
    bool dock_finished;
    bool dock_error;
    bool isPureDocking;
    int dock_recov_times;
    std::string dock_type;
    std::string frame_id;
    std::string normal_dock_type_y_param;  // From YAML: normal_dock_type_y
    std::string normal_dock_type_x_param;  // From YAML: normal_dock_type_x
    std::string cam_dock_type_y_param;     // From YAML: cam_dock_type_y
    std::string cam_dock_type_x_param;     // From YAML: cam_dock_type_x
    std::string fast_normal_dock_type_y_param;  // From YAML: fast_normal_dock_type_y
    std::string fast_normal_dock_type_x_param;  // From YAML: fast_normal_dock_type_x
    std::string fast_cam_dock_type_y_param;     // From YAML: fast_cam_dock_type_y
    std::string fast_cam_dock_type_x_param;     // From YAML: fast_cam_dock_type_x
    std::string slow_normal_dock_type_y_param;  // From YAML: slow_normal_dock_type_y
    std::string slow_normal_dock_type_x_param;  // From YAML: slow_normal_dock_type_x
    std::string slow_cam_dock_type_y_param;     // From YAML: slow_cam_dock_type_y
    std::string slow_cam_dock_type_x_param;     // From YAML: slow_cam_dock_type_x
    
    // Staging distances per robot side (from YAML)
    double staging_dist_front_;
    double staging_dist_right_;
    double staging_dist_back_;
    double staging_dist_left_;
    
    // Pose data
    geometry_msgs::msg::PoseStamped goal_pose;
    geometry_msgs::msg::PoseStamped robot_pose;
    
    // Input values
    int target_pose_idx;
    RobotSide target_side;
    Direction target_direction;
};

#endif // ON_DOCK_HPP
