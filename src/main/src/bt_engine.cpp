#include "bt_engine.hpp"

ReadySignal::ReadySignal() : Node("ready_signal"){
    ready_sub = this->create_subscription<std_msgs::msg::String>("/robot/startup/plan", 2, std::bind(&ReadySignal::readyCallback, this, std::placeholders::_1));
    ready_srv_client = this->create_client<btcpp_ros2_interfaces::srv::StartUpSrv>("/robot/startup/ready_signal");
    is_main_ready = false;
}

void ReadySignal::readySignalCallback(const std_msgs::msg::String::SharedPtr msg) {
    if (msg != NULL && !is_main_ready)
        is_main_ready = true;
}

void ReadySignal::sendReadySignal(int group, int status) {
    RCLCPP_INFO_STREAM(this->get_logger(), "send ready signal");

    auto request = std::make_shared<btcpp_ros2_interfaces::srv::StartUpSrv::Request>();
    request->group = group;
    request->state = status;

    ready_srv_client->async_send_request(request, 
        [this](rclcpp::Client<btcpp_ros2_interfaces::srv::StartUpSrv>::SharedFuture future) {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "response: success=%d, group=%d", int(response->success), response->group);
        }
    );
}

void BTengine::BTengine() : Node("bt_engine"), ReadySignal(), rate(100) {}

std::shared_ptr<rclcpp::Node> BTengine::getNode() {
    return shared_from_this();
}

void BTengine::timeCallback(const std_msgs::msg::Float32::SharedPtr msg) {
    game_time = msg->data;
    blackboard->set<double>("game_time", game_time);
}

void BTengine::readyCallback(const std_msgs::msg::String::SharedPtr msg) {
    if(isReady) return;
    
    if(msg->data.back() == 'Y') blackboard->set<int>("team", YELLOW);
    else blackboard->set<int>("team", BLUE);
    
    if(msg->data.back() == 'W') blackboard->set<int>("robot", WHITE);
    else blackboard->set<int>("robot", BLACK);
}

