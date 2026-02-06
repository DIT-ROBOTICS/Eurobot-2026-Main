#ifndef DECISION_CORE_HPP
#define DECISION_CORE_HPP

#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include <vector>
#include <cmath>
#include <queue>
#include "bt_config.hpp"

using namespace BT;
using namespace std;

/**
 * @brief PointScore - Used for priority queue to rank goal points by reward
 * Higher score = higher priority (max-heap behavior in priority_queue)
 */
struct PointScore {
    GoalPose pose;
    int score;
    
    PointScore(GoalPose p, int s) : pose(p), score(s) {}
    
    // For max-heap: higher score should have higher priority
    bool operator<(const PointScore& other) const {
        return score < other.score;
    }
};
/**
 * @brief DecisionCore - A BT SyncActionNode that decides the next target
 * 
 * Reads from blackboard:
 *   - json_point: sequence of point indices from mission_sequence.json
 *   - collection_info: camera data about collection points
 *   - pantry_info: camera data about pantry status
 * 
 * Writes to blackboard:
 *   - current_target_index: index of current point
 *   - current_target_pose: PoseStamped of target
 */
class DecisionCore : public BT::SyncActionNode {
public:
    // BT function
    DecisionCore(const std::string& name, const BT::NodeConfig& config, 
                 const RosNodeParams& params, BT::Blackboard::Ptr blackboard);
    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
    
    // system function
    void doTake();
    void doPut();
    void doFlip();
    void doDock();

private:
    // subscriber
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr json_point_sub;
    
    // init
    void loadMapPoints();
    // only get json once
    void loadJsonPoint(const std_msgs::msg::Int32MultiArray::SharedPtr msg);

    void getVisionData();
    void getFieldInfo();
    void getRobotInfo();
    void getInputPort();
    void sortPantryPriority();
    void sortCollectionPriority();
    pair<GoalPose, RobotSide> getTargetPointInfo(ActionType action_type); // return {point_index, side_index}
    RobotSide getTargetSideIndex();
    Direction decideDirection(GoalPose goal_pose, RobotSide robot_side);
    ActionType decideNextActionType(ActionType action_type);
    void writeOutputPort();
    void writeBlackboard();
    
    // Reward calculation helpers
    int calculatePantryScore(int pantry_idx);
    int calculateCollectionScore(int collection_idx);
    
    // Distance and location helpers
    double calculateDistance(GoalPose pose);
    double calculateRivalDistance(GoalPose pose);
    geometry_msgs::msg::Point getPointPosition(GoalPose pose);
    bool isOwnSidePantry(GoalPose pose);
    bool isOwnSideCollection(GoalPose pose);
    bool isMiddlePantry(GoalPose pose);
    bool isMiddleCollection(GoalPose pose);
    void updatePoseData();

    // blackboard variable
    vector<FieldStatus> collection_info;
    vector<FieldStatus> pantry_info;
    vector<FieldStatus> robot_side_status;
    vector<vector<FlipStatus>> hazelnut_status;
    priority_queue<PointScore> pantry_priority;
    priority_queue<PointScore> collection_priority;
    geometry_msgs::msg::PoseStamped robot_pose;
    geometry_msgs::msg::PoseStamped rival_pose;
    vector<double> map_points;
    Team current_team;
    
    // Reward constants
    static constexpr int SCORE_BASE_AVAILABLE = 100;
    static constexpr int SCORE_MIDDLE_BONUS = 80;       // E,F for pantry; middle collection
    static constexpr int SCORE_OWN_SIDE_BONUS = 40;
    static constexpr int SCORE_OPPONENT_SIDE_PENALTY = -20;
    static constexpr int SCORE_DISTANCE_FACTOR = 10;    // Points per meter closer
    static constexpr double RIVAL_PROXIMITY_THRESHOLD = 0.3; // meters
    static constexpr int SCORE_RIVAL_NEARBY_PENALTY = -200;


    // system variable
    shared_ptr<rclcpp::Node> node_ptr;
    BT::Blackboard::Ptr blackboard_ptr;
    vector<int> json_point;
    bool has_received_json_point;
    RobotSide decided_robot_side;
    ActionType decided_action_type;
    ActionType next_action_type;
    GoalPose target_goal_pose_idx;
    RobotSide target_pose_side_idx;
    RobotSide target_robot_side;
    Direction target_direction;
};

#endif // DECISION_CORE_HPP
