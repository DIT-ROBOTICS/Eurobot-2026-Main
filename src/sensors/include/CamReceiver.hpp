#include "receiver_util.hpp"
using namespace std;

// receive the vision data
// reconstruct and organize to the message, then store to the BT blackboard
class CamReceiver : public BT::SyncActionNode
{
public:
    CamReceiver(const std::string& name, const BT::NodeConfig& config, const RosNodeParams& params, BT::Blackboard::Ptr blackboard);
    
    /* Node remapping function */
    static BT::PortsList providedPorts();
    
    /* Start and running function */
    BT::NodeStatus tick() override;

private:
    // Initialize default values for when camera is down (fallback)
    void initializeDefaultStatus();
    
    // collection for material that original placed, pantry for the collected materials will be placed.
    // for encoding rules, the ref: https://www.notion.so/ditrobotics/2e2c8b048fcf8030a296da55379e06dc?source=copy_link#2e2c8b048fcf8081b59ade2d18562558
    void collection_info_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
    void pantry_info_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg);
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr collection_sub;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr pantry_sub;

    shared_ptr<rclcpp::Node> node_;
    BT::Blackboard::Ptr blackboard_;
    std_msgs::msg::Int32MultiArray collection_info;
    std_msgs::msg::Int32MultiArray pantry_info;
};