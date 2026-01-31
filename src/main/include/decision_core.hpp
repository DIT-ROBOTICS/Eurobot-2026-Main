#ifndef DECISION_CORE_HPP
#define DECISION_CORE_HPP

#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include <vector>
#include <cmath>

using namespace BT;

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
    /**
     * @brief Construct DecisionCore node
     * @param name Node name for BT
     * @param config BT node configuration
     * @param params ROS node parameters
     * @param blackboard Shared blackboard pointer
     */
    DecisionCore(const std::string& name, const BT::NodeConfig& config, 
                 const RosNodeParams& params, BT::Blackboard::Ptr blackboard);

    /**
     * @brief Define BT ports for this node
     * @return List of input/output ports
     */
    static BT::PortsList providedPorts();

    /**
     * @brief Main tick function called by behavior tree
     * @return SUCCESS if target found, FAILURE otherwise
     */
    BT::NodeStatus tick() override;

private:
    // ============ Private Helper Methods ============
    
    /**
     * @brief Load map points from ROS parameter
     */
    void loadMapPoints();

    /**
     * @brief Get mission sequence from blackboard
     * @return Vector of point indices
     */
    std::vector<int> getMissionSequence();

    /**
     * @brief Get camera sensor data from blackboard
     * @param collection_info Output collection info
     * @param pantry_info Output pantry info
     */
    void getSensorData(std_msgs::msg::Int32MultiArray& collection_info,
                       std_msgs::msg::Int32MultiArray& pantry_info);

    /**
     * @brief Convert point index to PoseStamped
     * @param point_index Index into map_points array
     * @return PoseStamped with position and orientation
     */
    geometry_msgs::msg::PoseStamped indexToPose(int point_index);

    /**
     * @brief Convert direction value to quaternion orientation
     * @param direction Direction value (0=E, 1=N, 2=W, 3=S)
     * @return Quaternion orientation
     */
    geometry_msgs::msg::Quaternion directionToQuaternion(double direction);

    /**
     * @brief Determine action type based on target and state
     * @param point_index Target point index
     * @return Action type string ("navigate", "dock", "collect", etc.)
     */
    std::string determineActionType(int point_index);

    /**
     * @brief Check if sequence is complete
     * @param sequence_size Total size of sequence
     * @return true if all points visited
     */
    bool isSequenceComplete(size_t sequence_size);

    /**
     * @brief Reset sequence to start
     */
    void resetSequence();

    /**
     * @brief Advance to next point in sequence
     */
    void advanceSequence();

    /**
     * @brief Write decision outputs to blackboard
     * @param target_index Target point index
     * @param target_pose Target pose
     * @param action_type Action type string
     */
    void writeOutputs(int target_index, 
                      const geometry_msgs::msg::PoseStamped& target_pose,
                      const std::string& action_type);

    // ============ Member Variables ============
    
    std::shared_ptr<rclcpp::Node> node_;
    BT::Blackboard::Ptr blackboard_;
    
    // Map point data (7 values per point: x, y, dir, back_off, shift, front_off, dock_dist)
    std::vector<double> map_points_;
    static constexpr int VALUES_PER_POINT = 7;
    
    // Sequence tracking
    int current_sequence_index_;
    
    // Default sequence if none provided
    const std::vector<int> DEFAULT_SEQUENCE = {0, 1, 2, 3};
};

#endif // DECISION_CORE_HPP
