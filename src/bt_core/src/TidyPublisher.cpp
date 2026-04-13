#include "TidyPublisher.hpp"

// MAGENTA colored logs for TidyPublisher
#define TD_COLOR "\033[95m"
#define TD_RESET "\033[0m"
#define TD_INFO(node, fmt, ...) RCLCPP_INFO(node->get_logger(), TD_COLOR "[TidyPublisher] " fmt TD_RESET, ##__VA_ARGS__)
#define TD_WARN(node, fmt, ...) RCLCPP_WARN(node->get_logger(), TD_COLOR "[TidyPublisher] " fmt TD_RESET, ##__VA_ARGS__)

TidyPublisher::TidyPublisher(const std::string& name,
                             const BT::NodeConfig& config,
                             const RosNodeParams& params,
                             BT::Blackboard::Ptr blackboard)
    : BT::StatefulActionNode(name, config),
      node_(params.nh.lock()),
      blackboard_(blackboard),
            side_idx_(0),
            pulse_ms_(200),
            pulse_on_sent_(false),
    pulse_off_sent_(false) {
    tidy_pub_ = node_->create_publisher<std_msgs::msg::Bool>("/robot/on_tidy", 10);
    TD_INFO(node_, "Initialized — publishing to /robot/on_tidy");
}

BT::PortsList TidyPublisher::providedPorts() {
    return {
        BT::InputPort<std::string>("state", "off", "state: 'on', 'off' or 'auto'"),
        BT::InputPort<int>("pulse_ms", 200, "pulse duration in ms (used when state='auto')"),
        BT::InputPort<int>("targetPoseSideIdx", 0, "Robot side index to check")
    };
}

BT::NodeStatus TidyPublisher::onStart() {
    if (!getInput<std::string>("state", state_)) {
        TD_WARN(node_, "Missing state, defaulting to 'off'");
        state_ = "off";
    }

    if (!getInput<int>("targetPoseSideIdx", side_idx_)) {
        TD_WARN(node_, "Missing targetPoseSideIdx, defaulting to 0");
        side_idx_ = 0;
    }

    if (!getInput<int>("pulse_ms", pulse_ms_)) {
        pulse_ms_ = 200;
    }

    if (side_idx_ < 0 || side_idx_ >= ROBOT_SIDES) {
        TD_WARN(node_, "Invalid targetPoseSideIdx: %d", side_idx_);
        return BT::NodeStatus::FAILURE;
    }

    if (state_ == "off") {
        if (side_idx_ != 2) {
            TD_INFO(node_, "Skip OFF publish: targetPoseSideIdx=%d (only side 2 enabled)", side_idx_);
            return BT::NodeStatus::SUCCESS;
        }

        std_msgs::msg::Bool msg;
        msg.data = false;
        TD_INFO(node_, "Publishing /robot/on_tidy: false");
        tidy_pub_->publish(msg);
        return BT::NodeStatus::SUCCESS;
    }

    if (state_ == "on") {
        if (side_idx_ != 2) {
            TD_INFO(node_, "Skip ON publish: targetPoseSideIdx=%d (only side 2 enabled)", side_idx_);
            return BT::NodeStatus::SUCCESS;
        }

        std_msgs::msg::Bool msg;
        msg.data = true;
        TD_INFO(node_, "Publishing /robot/on_tidy: true");
        tidy_pub_->publish(msg);
        return BT::NodeStatus::SUCCESS;
    }

    if (state_ == "auto") {
        pulse_on_sent_ = false;
        pulse_off_sent_ = false;

        if (side_idx_ != 2) {
            TD_INFO(node_, "AUTO no-op: targetPoseSideIdx=%d (only side 2 enabled)", side_idx_);
        }

        return BT::NodeStatus::RUNNING;
    }

    TD_WARN(node_, "Invalid state '%s' (expected 'on', 'off' or 'auto')", state_.c_str());
    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus TidyPublisher::onRunning() {
    if (state_ == "auto" && side_idx_ != 2) {
        // Keep RUNNING so Parallel result remains controlled by OnDockAction.
        return BT::NodeStatus::RUNNING;
    }

    // Shared condition logic for state==auto
    std::vector<std::vector<FlipStatus>> hazelnut_status;
    if (!blackboard_->get<std::vector<std::vector<FlipStatus>>>("hazelnut_status", hazelnut_status)) {
        TD_WARN(node_, "hazelnut_status not in blackboard");
        return BT::NodeStatus::RUNNING;
    }

    if (side_idx_ < 0 || side_idx_ >= static_cast<int>(hazelnut_status.size())) {
        TD_WARN(node_, "targetPoseSideIdx out of range: %d", side_idx_);
        return BT::NodeStatus::FAILURE;
    }

    bool all_no_take = true;
    for (int i = 0; i < HAZELNUT_LENGTH && i < static_cast<int>(hazelnut_status[side_idx_].size()); ++i) {
        if (hazelnut_status[side_idx_][i] != FlipStatus::NO_TAKE) {
            all_no_take = false;
            break;
        }
    }

    if (state_ == "auto") {
        // Keep original trigger logic: only send ON when condition is met.
        if (!pulse_on_sent_ && !all_no_take) {
            std_msgs::msg::Bool msg;
            msg.data = true;
            TD_INFO(node_, "Publishing /robot/on_tidy: true (auto trigger, %d ms)", pulse_ms_);
            tidy_pub_->publish(msg);

            pulse_on_sent_ = true;
            pulse_start_time_ = std::chrono::steady_clock::now();
        }

        if (!pulse_off_sent_) {
            if (pulse_on_sent_) {
                auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - pulse_start_time_).count();

                if (elapsed_ms >= pulse_ms_) {
                    std_msgs::msg::Bool msg;
                    msg.data = false;
                    TD_INFO(node_, "Publishing /robot/on_tidy: false (auto end)");
                    tidy_pub_->publish(msg);
                    pulse_off_sent_ = true;
                }
            }
        }

        // Keep RUNNING so Parallel result is controlled by OnDockAction.
        return BT::NodeStatus::RUNNING;
    }

    return BT::NodeStatus::RUNNING;
}

void TidyPublisher::onHalted() {
    if (state_ == "auto" && pulse_on_sent_ && !pulse_off_sent_) {
        std_msgs::msg::Bool msg;
        msg.data = false;
        TD_INFO(node_, "Publishing /robot/on_tidy: false (auto halt safety)");
        tidy_pub_->publish(msg);
        pulse_off_sent_ = true;
    }

    TD_INFO(node_, "Halted (preempted)");
}
