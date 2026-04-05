#include "controller_type_publisher.hpp"

ControllerTypePublisher::ControllerTypePublisher(const std::string& name, const NodeConfig& config,
                                                 const RosNodeParams& params, BT::Blackboard::Ptr blackboard)
    : BT::SyncActionNode(name, config),
      node_(params.nh.lock()),
      blackboard_(blackboard),
      controller_type_info_("Fast"),
      mode_("Auto") {}

BT::PortsList ControllerTypePublisher::providedPorts() {
    return {
        BT::InputPort<std::string>("controller_type", "Fast", "Controller type to publish"),
        BT::InputPort<std::string>("mode", "Auto", "Current mode (Auto/Manual)")
    };
}

BT::NodeStatus ControllerTypePublisher::tick() {
    std::string input_mode;
    std::string current_controller_type;

    if (!getInput<std::string>("controller_type", controller_type_info_)) {
        controller_type_info_ = "Fast";
    }
 
    if (!getInput<std::string>("mode", input_mode)) {
        input_mode = "Auto";
    }

    mode_ = input_mode;
    blackboard_->set<std::string>("mode", mode_);

    if (!blackboard_->get<std::string>("controller_type", current_controller_type)) {
        current_controller_type = "Slow";
        blackboard_->set<std::string>("controller_type", current_controller_type);
    }

    readBlackboard();

    if (mode_ == "Auto" && pantry_info.data.empty() && collection_info.data.empty() &&
        current_controller_type != controller_type_info_) {
        blackboard_->set<std::string>("controller_type", controller_type_info_);
        RCLCPP_INFO(node_->get_logger(),
                    "[ControllerTypePublisher] set controller_type blackboard: %s -> %s (mode=%s)",
                    current_controller_type.c_str(), controller_type_info_.c_str(), mode_.c_str());
        return BT::NodeStatus::SUCCESS;
    }

    else if (mode_ == "Manual" && current_controller_type != controller_type_info_) {
        blackboard_->set<std::string>("controller_type", controller_type_info_);
        RCLCPP_INFO(node_->get_logger(),
                    "[ControllerTypePublisher]  set controller_type blackboard: %s -> %s (mode=%s)",
                    current_controller_type.c_str(), controller_type_info_.c_str(), mode_.c_str());
        return BT::NodeStatus::SUCCESS;
    }

    RCLCPP_DEBUG(node_->get_logger(),
                 "[ControllerTypePublisher] skipped update (mode=%s, collection=%zu, pantry=%zu, current=%s, target=%s)",
                 mode_.c_str(), collection_info.data.size(), pantry_info.data.size(),
                 current_controller_type.c_str(), controller_type_info_.c_str());

    return BT::NodeStatus::SUCCESS;
}

void ControllerTypePublisher::readBlackboard() {
    std::vector<int> collection_sequence;
    std::vector<int> pantry_sequence;

    if (blackboard_->get<std::vector<int>>("collection_sequence", collection_sequence)) {
        collection_info.data = collection_sequence;
    }

    if (blackboard_->get<std::vector<int>>("pantry_sequence", pantry_sequence)) {
        pantry_info.data = pantry_sequence;
    }
}
