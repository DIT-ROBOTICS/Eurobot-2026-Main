#include "CamReceiver.hpp"
#include "bt_config.hpp"

CamReceiver::CamReceiver(const std::string& name, const BT::NodeConfig& config, const RosNodeParams& params, BT::Blackboard::Ptr blackboard)
    : BT::SyncActionNode(name, config), node_(params.nh.lock()), blackboard_(blackboard) {

    collection_sub = node_->create_subscription<std_msgs::msg::Int32MultiArray>(
        "collection_status", 10, std::bind(&CamReceiver::collection_info_callback, this, std::placeholders::_1));
    pantry_sub = node_->create_subscription<std_msgs::msg::Int32MultiArray>(
        "pantry_status", 10, std::bind(&CamReceiver::pantry_info_callback, this, std::placeholders::_1));
    
    // Initialize with default values (camera fallback)
    // Pantry: all EMPTY (available for PUT)
    // Collection: all OCCUPIED (has hazelnuts for TAKE)
    initializeDefaultStatus();
    
    RCLCPP_INFO(node_->get_logger(), "CamReceiver: Initialized with default values (pantry=EMPTY, collection=OCCUPIED)");
}

void CamReceiver::initializeDefaultStatus() {
    // Initialize pantry_info: all EMPTY (available to put hazelnuts)
    std::vector<FieldStatus> default_pantry(PANTRY_LENGTH, FieldStatus::EMPTY);
    blackboard_->set<std::vector<FieldStatus>>("pantry_info", default_pantry);
    
    // Initialize collection_info: all OCCUPIED (has hazelnuts to take)
    std::vector<FieldStatus> default_collection(COLLECTION_LENGTH, FieldStatus::OCCUPIED);
    blackboard_->set<std::vector<FieldStatus>>("collection_info", default_collection);
    
    // Initialize robot_side_status: all EMPTY (robot sides have no hazelnuts)
    std::vector<FieldStatus> default_robot_sides(ROBOT_SIDES, FieldStatus::EMPTY);
    blackboard_->set<std::vector<FieldStatus>>("robot_side_status", default_robot_sides);
    
    // Initialize hazelnut_status: all NO_FLIP
    std::vector<std::vector<FlipStatus>> default_hazelnut(
        ROBOT_SIDES, std::vector<FlipStatus>(HAZELNUT_LENGTH, FlipStatus::NO_FLIP));
    blackboard_->set<std::vector<std::vector<FlipStatus>>>("hazelnut_status", default_hazelnut);
}

PortsList CamReceiver::providedPorts() {
    return {};
}

void CamReceiver::collection_info_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
    RCLCPP_DEBUG(node_->get_logger(), "CamReceiver: Received collection info update.");
    collection_info = *msg;
    
    if (!collection_info.data.empty()) {
        // Convert Int32MultiArray to vector<FieldStatus> for blackboard
        std::vector<FieldStatus> collection_status;
        for (const auto& val : collection_info.data) {
            collection_status.push_back(static_cast<FieldStatus>(val));
        }
        blackboard_->set<std::vector<FieldStatus>>("collection_info", collection_status);
        blackboard_->set<std_msgs::msg::Int32MultiArray>("collection_info_raw", collection_info);
    }
}

void CamReceiver::pantry_info_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
    RCLCPP_DEBUG(node_->get_logger(), "CamReceiver: Received pantry info update.");
    pantry_info = *msg;
    
    if (!pantry_info.data.empty()) {
        // Convert Int32MultiArray to vector<FieldStatus> for blackboard
        std::vector<FieldStatus> pantry_status;
        for (const auto& val : pantry_info.data) {
            pantry_status.push_back(static_cast<FieldStatus>(val));
        }
        blackboard_->set<std::vector<FieldStatus>>("pantry_info", pantry_status);
        blackboard_->set<std_msgs::msg::Int32MultiArray>("pantry_info_raw", pantry_info);
    }
}

BT::NodeStatus CamReceiver::tick() {
    RCLCPP_DEBUG(node_->get_logger(), "CamReceiver tick");
    
    // Log current status for debugging
    std::vector<FieldStatus> pantry_status;
    std::vector<FieldStatus> collection_status;
    
    if (blackboard_->get<std::vector<FieldStatus>>("pantry_info", pantry_status)) {
        for (size_t i = 0; i < pantry_status.size(); ++i) {
            RCLCPP_DEBUG(node_->get_logger(), "Pantry[%zu]: %d", i, static_cast<int>(pantry_status[i]));
        }
    }
    
    if (blackboard_->get<std::vector<FieldStatus>>("collection_info", collection_status)) {
        for (size_t i = 0; i < collection_status.size(); ++i) {
            RCLCPP_DEBUG(node_->get_logger(), "Collection[%zu]: %d", i, static_cast<int>(collection_status[i]));
        }
    }
    
    return BT::NodeStatus::SUCCESS;
}