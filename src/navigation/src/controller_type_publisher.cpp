#include "controller_type_publisher.hpp"

namespace {
std::atomic<bool> controller_type_monitor_started(false);
std::atomic<bool> controller_type_initialized(false);
std::atomic<bool> controller_type_published(false);
}

ControllerTypePublisher::ControllerTypePublisher(const std::string& name, const NodeConfig& config,
                                                 const RosNodeParams& params, BT::Blackboard::Ptr blackboard)
    : BT::SyncActionNode(name, config),
      node_(params.nh.lock()),
      blackboard_(blackboard),
      controller_type_info("Fast") {
    controller_type_pub_ = node_->create_publisher<std_msgs::msg::String>("/controller_type", 10);
}

BT::PortsList ControllerTypePublisher::providedPorts() {
    return {
        BT::InputPort<std::string>("controller_type", "Fast", "Controller type to publish")
    };
}

BT::NodeStatus ControllerTypePublisher::tick() {
    std::string current_controller_type;

    if (!getInput<std::string>("controller_type", controller_type_info)) {
        controller_type_info = "Fast";
    }

    if (!blackboard_->get<std::string>("controller_type", current_controller_type)) {
        current_controller_type = "";
        blackboard_->set<std::string>("controller_type", current_controller_type);
    }

    if (!controller_type_initialized.exchange(true)) {
        std_msgs::msg::String msg;
        msg.data = "Slow";
        blackboard_->set<std::string>("controller_type", msg.data);
        controller_type_pub_->publish(msg);
        RCLCPP_INFO(node_->get_logger(),"[ControllerTypePublisher] initialized /controller_type: %s", msg.data.c_str());
    }

    if (!controller_type_monitor_started.exchange(true)) {
        std::thread(&ControllerTypePublisher::spinThread, this).detach();
    }

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

void ControllerTypePublisher::publishControllerType() {
    std_msgs::msg::String msg;
    msg.data = controller_type_info;
    blackboard_->set<std::string>("controller_type", msg.data);
    controller_type_pub_->publish(msg);
    RCLCPP_INFO(node_->get_logger(),"[ControllerTypePublisher] published /controller_type: %s", msg.data.c_str());
}

void ControllerTypePublisher::spinThread() {
    while (rclcpp::ok() && !controller_type_published) {
        std::string current_controller_type;

        readBlackboard();

        if (!blackboard_->get<std::string>("controller_type", current_controller_type)) {
            current_controller_type = "";
            blackboard_->set<std::string>("controller_type", current_controller_type);
        }

        if (pantry_info.data.empty() && collection_info.data.empty()) {
            if (current_controller_type != controller_type_info) {
                publishControllerType();
            }
            controller_type_published = true;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}
