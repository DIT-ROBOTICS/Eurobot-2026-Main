#include "startup.hpp"

const int TIME_RATE = 100;
const int SIMA_TICK_THRESHOLD = 85;
const int GAME_TIME = 100; 

StartUp::StartUp() : Node("startup_node"){
    // game timer
    timer = this->create_wall_timer(
        std::chrono::microseconds(TIME_RATE),
        std::bind(&StartUp::stateTransition, this));

    game_time_pub = this->create_publisher<std_msgs::msg::Float32>("/robot/startup/game_time", 2);
    rate = rclcpp::Rate(TIME_RATE);

    // State checker for other groups
    are_you_ready_pub = this->create_publisher<std_msgs::msg::Bool>("/robot/startup/are_you_ready", 2);
    ready_srv_server = this->create_service<btcpp_ros2_interfaces::srv::StartUpSrv>(
        "/robot/startup/ready_signal", std::bind(&StartUp::systemCheckFeedback, this, std::placeholders::_1, std::placeholders::_2));
    plug_sub = this->create_subscription<std_msgs::msg::Bool>("/robot/startup/plug", 2, std::bind(&StartUp::startCallback, this, std::placeholders::_1));
    plan_file_pub = this->create_publisher<std_msgs::msg::String>("/robot/startup/plan_file", 2);
    start_signal_client = this->create_client<std_srvs::srv::SetBool>(
        "/robot/start_signal");

    // sima
    start_sima_pub = this->create_publisher<std_msgs::msg::Int16>("/sima/start", 2);
    start_ninja_pub = this->create_publisher<std_msgs::msg::Int16>("/ninja/start", 2);
    sima_tick_threshold = SIMA_TICK_THRESHOLD;
    sima_selected_plan = 0;
    sima_started = false;

    // web
    web_plan_client = this->create_client<std_srvs::srv::Trigger>("/robot/startup/web_plan");
    web_request_received = false;

    // default variable value
    team = Team::YELLOW;
    robot = Robot::WHITE;
    selected_plan = 0;
    plan_file_name = "";
    startup_state = StartUpState::INIT;
    start_position = geometry_msgs::msg::PoseWithCovarianceStamped();
    group_state = {0, 0, 0, 0, 0};
    is_plugged = false;
    initParam();
}

void StartUp::initParam() {
    this->declare_parameter<std::string>("robot_name", "White");
    this->declare_parameter<std::vector<double>>("blue_team.start_pose", std::vector<double>{0.0, 0.0, 0.0});
    this->declare_parameter<std::vector<double>>("yellow_team.start_pose", std::vector<double>{0.0, 0.0, 0.0});

    this->get_parameter("robot_name", robot_name);
    this->get_parameter("blue_team.start_pose", blue_start_pose);
    this->get_parameter("yellow_team.start_pose", yellow_start_pose);

    RCLCPP_INFO(this->get_logger(), "[StartUp]: Loaded config for robot: %s", robot_name.c_str());
}

void StartUp::stateTransition() {
    switch(startup_state) {
        case StartUpState::INIT:
            getWebPlan();
            parsePlanCode();
            startup_state = StartUpState::READY;
            break;
        case StartUpState::READY:
            publishSystemCheckSignal();
            if(isAllSystemReady()) {
                // TODO: add one hot trigger for startSignal and initialPose, if not theorically publish once
                publishStartSignal();
                publishInitialPose();
                startup_state = StartUpState::START;
            }
            break;
        case StartUpState::START:
            publishTime();
            if(gameOver()) startup_state = StartUpState::END;
            break;
        case StartUpState::END:
            RCLCPP_INFO(this->get_logger(), "[StartUp]: End state");
            break;
        case StartUpState::ERROR:
            RCLCPP_ERROR(this->get_logger(), "[StartUp]: Error state");
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "[StartUp]: Unknown state");
            break;
    }
}

void StartUp::getWebPlan() {
    if(!web_plan_client->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_ERROR(this->get_logger(), "Service not available, use default plan");
        selected_plan = 11;
        return;
    }
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    web_request_received = true;
    web_plan_client->async_send_request(
        request,
        [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future){
            try {
                auto response = future.get();
                if(response->success){
                    selected_plan = std::stoi(response->message);
                    RCLCPP_INFO(this->get_logger(), "[StartUp]: Received relected plan: %d", selected_plan);
                } else {
                    RCLCPP_ERROR(this->get_logger(), "[StartUp]: Failed to get plan: %s", response->message.c_str());
                    web_request_received = false;
                }
            } catch (const std::exception& e) {
                RCLCPP_ERROR(this->get_logger(), "[StartUp]: Failed to get plan: %s", e.what());
                web_request_received = false;
            }
        }
    );
    // TODO: get Sima plan from defined config file
    sima_selected_plan = selected_plan; // temporary
}

void StartUp::parsePlanCode() {
    // get info
    robot = stringToRobot(robot_name);
    // Assuming last digit 0 = Yellow, 1 = Blue to match enum (Yellow=0, Blue=1)
    team = static_cast<Team>(selected_plan % 10);
    selected_plan = selected_plan / 10;

    // translate plan code and other info into plan file name
    // Format: "robot_team_planNumber" e.g. "white_yellow_1.xml" 
    plan_file_name = robotToString(robot) + "_" + teamToString(team) + "_" + std::to_string(selected_plan) + ".xml";
    RCLCPP_INFO(this->get_logger(), "[StartUp]: Parsed Plan: %s", plan_file_name.c_str());

    // publish plan file
    auto msg = std_msgs::msg::String();
    msg.data = plan_file_name;
    plan_file_pub->publish(msg);
}

void StartUp::publishSystemCheckSignal() {
    auto msg = std_msgs::msg::Bool();
    msg.data = true;
    are_you_ready_pub->publish(msg);
}

void StartUp::systemCheckFeedback(const std::shared_ptr<btcpp_ros2_interfaces::srv::StartUpSrv::Request> request,
    std::shared_ptr<btcpp_ros2_interfaces::srv::StartUpSrv::Response> response) {
    RCLCPP_INFO(this->get_logger(), "[StartUp]: Received system check feedback from group %d, state: %d", request->group_id, request->state);
    group_state[request->group_id] = request->state;
    response->group = request->group;
    response->success = true;
    switch(request->group){
        case 1: // main
            if(request->state == 1) { group_state[1] = 1; RCLCPP_INFO(this->get_logger(), "[StartUp]: Main system is ready");}
            break;
        case 2: // vision
            if(request->state == 1) { group_state[2] = 1; RCLCPP_INFO(this->get_logger(), "[StartUp]: Vision system is ready");}
            break;
        case 3: // navigation
            if(request->state == 1) { group_state[3] = 1; RCLCPP_INFO(this->get_logger(), "[StartUp]: Navigation system is ready");}
            break;
        case 4: // localization
            if(request->state == 1) { group_state[4] = 1; RCLCPP_INFO(this->get_logger(), "[StartUp]: Localization system is ready");}
            break;
        // TODO: add more group by increase group_state size
    }
    RCLCPP_INFO(this->get_logger(), "[StartUp]: Response %d to %d", response->group, response->success);
}

bool StartUp::isAllSystemReady() {
    for(int i = 1; i < group_state.size(); i++) {
        if(group_state[i] != 1) {
            group_state[0] = 0;
            return false;
        }
    }
    group_state[0] = 1;
    if(is_plugged && group_state[0] == 1) {
        RCLCPP_INFO(this->get_logger(), "[StartUp]: All system is ready & Received start signal");
        return true;
    }
    return false;
}

void StartUp::startCallback(const std::shared_ptr<std_msgs::msg::Bool> msg) {
    if(startup_state == StartUpState::READY) {
        is_plugged = msg->data;
    }
}

void StartUp::publishStartSignal() {
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = true;
    start_signal_client->async_send_request(request, 
        [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "[StartUp]: Start signal sent, response: %s", response->success ? "true" : "false");
        }
    );  
}

void StartUp::publishInitialPose() {
    auto msg = geometry_msgs::msg::PoseWithCovarianceStamped();
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = "map";
    // start position data received
    if(team == Team::YELLOW) {
        start_position.header.frame_id = "map";
        start_position.pose.pose.position.x = yellow_start_pose[0];
        start_position.pose.pose.position.y = yellow_start_pose[1];
        start_position.pose.pose.position.z = 0.0;
        start_position.pose.pose.orientation = yaw2qua(yellow_start_pose[2]);
    } else {
        start_position.header.frame_id = "map";
        start_position.pose.pose.position.x = blue_start_pose[0];
        start_position.pose.pose.position.y = blue_start_pose[1];
        start_position.pose.pose.position.z = 0.0;
        start_position.pose.pose.orientation = yaw2qua(blue_start_pose[2]);
    }
    start_position.covariance[0]  = 1e-4;
    start_position.covariance[7]  = 1e-4;
    start_position.covariance[35] = 0.068; // (15 deg)^2
    initialpose_pub->publish(start_position);
}

void StartUp::tickSima(double game_time) {
    if(game_time >= sima_tick_threshold && !sima_started) {
        auto msg = std_msgs::msg::Int16();
        msg.data = sima_selected_plan;
        start_sima_pub->publish(msg);
        sima_started = true;

        // TODO: add continuous publishing if sima is not started for one time publish
        // add after testing sima
    }
}

void StartUp::publishTime() {
    double cur_time = this->get_clock()->now().seconds();
    std_msgs::msg::Float32 cur_time_msg;
    cur_time_msg.data = cur_time - start_time;
    game_time_pub->publish(cur_time_msg);
    tickSima(cur_time_msg.data);
}

bool StartUp::gameOver(double game_time) {
    if(game_time >= GAME_TIME) return true;
    return false;
}

geometry_msgs::msg::Quaternion StartUp::yaw2qua(double yaw) {
    tf2::Quaternion qua;
    qua.setRPY(0.0, 0.0, yaw);
    return tf2::toMsg(qua);
}