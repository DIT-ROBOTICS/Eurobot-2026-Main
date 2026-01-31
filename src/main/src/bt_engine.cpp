#include "bt_engine.hpp"

const int TIME_RATE = 100;
const std::string JSON_FILE_PATH = "/home/robot/plan/";
const std::string BT_XML_DIRECTORY = "/home/robot/plan/bt/";
const std::string TREE_NAME = "MainTree";
const std::string BT_TREE_NODE_MODEL = "/home/robot/plan/bt/tree_node_model.xml";
const int TERMINATE_TIME = 101;

BTengine::BTengine() : rclcpp::Node("bt_engine") {
    start_srv_server = this->create_service<std_srvs::srv::SetBool>("/robot/start_signal", 
        std::bind(&BTengine::startCallback, this, std::placeholders::_1, std::placeholders::_2));

    are_you_ready_sub = this->create_subscription<std_msgs::msg::Bool>("/robot/startup/are_you_ready", 2, 
        std::bind(&BTengine::readyCallback, this, std::placeholders::_1));

    ready_srv_client = this->create_client<btcpp_ros2_interfaces::srv::StartUpSrv>("/robot/startup/ready_signal");
    
    plan_file_sub = this->create_subscription<std_msgs::msg::String>("/robot/startup/plan_file", 2, 
        std::bind(&BTengine::planFileCallback, this, std::placeholders::_1));

    team = Team::YELLOW;
    robot = Robot::WHITE;
    selected_plan = 0;
    plan_file_name = "";
    game_time = 0.0;
    blackboard = BT::Blackboard::create();
    game_time_sub = this->create_subscription<std_msgs::msg::Float32>("/robot/startup/game_time", 2, 
        std::bind(&BTengine::gameTimeCallback, this, std::placeholders::_1));

    node = this->shared_from_this();
    params = BT::RosNodeParams(node);
    rate = std::make_shared<rclcpp::Rate>(TIME_RATE);
    
    // Initialize new variables
    isReady = false;
    tree_name = TREE_NAME;
    bt_tree_node_model = BT_TREE_NODE_MODEL;
    group = 1;  // Main BT group
    
    initParam();
}

void BTengine::initParam() {
    // TODO: add param init
}

void BTengine::readyCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    if(msg->data) {
        sentReadySignal();   
        RCLCPP_INFO(this->get_logger(), "[BTengine]: Received ready signal");
    }
}

void BTengine::sentReadySignal() {
    auto request = std::make_shared<btcpp_ros2_interfaces::srv::StartUpSrv::Request>();
    request->group = group;
    request->state = static_cast<int>(StartUpState::READY);
    int captured_group = group;  // Copy member variable for lambda capture
    ready_srv_client->async_send_request(request,
      [this, captured_group](rclcpp::Client<btcpp_ros2_interfaces::srv::StartUpSrv>::SharedFuture future) {
        auto response = future.get();
        if (response->success) {
            RCLCPP_INFO(this->get_logger(), "[BTengine]: ReadySignal SUCCESS: group=%d", response->group);
        } 
      });
}

void BTengine::planFileCallback(const std_msgs::msg::String::SharedPtr msg) {
    plan_file_name = msg->data;

    // translate plan file name to team and robot
    // Format: "robot_team_planNumber" e.g. "white_yellow_1.xml" 
    // first 5 latter is robot name
    // 6th latter is team name
    // last 1 latter is plan number
    // spilt through "_"
    std::vector<std::string> plan_file_name_split = split(plan_file_name, '_');
    robot = stringToRobot(plan_file_name_split[0]);
    team = stringToTeam(plan_file_name_split[1]);
    selected_plan = std::stoi(plan_file_name_split[2]);

    RCLCPP_INFO(this->get_logger(), "[BTengine]: Received plan file for robot %s, team %s, plan %d", robotToString(robot).c_str(), teamToString(team).c_str(), selected_plan);
    
    // Mark as ready - createTree() is waiting for this
    isReady = true;
}

void BTengine::addJsonPoint() {
    std::vector<int> default_point = {1,1,1,1};

    std::ifstream json_file(JSON_FILE_PATH);
    if (!json_file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "[BTengine]: Failed to open json file");
        json_point = default_point;
        return;
    }
    Json::Value root;
    Json::CharReaderBuilder reader;
    std::string errors;
    
    if (!Json::parseFromStream(reader, json_file, &root, &errors)) {
        RCLCPP_ERROR(this->get_logger(), "[BTengine]: Failed to parse json: %s", errors.c_str());
        json_point = default_point;
        json_file.close();
        return;
    }
    json_file.close();

    // Extract "sequence" array
    if (root.isMember("sequence") && root["sequence"].isArray()) {
        json_point.clear();
        for (const auto& val : root["sequence"]) {
            if (val.isInt()) {
                json_point.push_back(val.asInt());
            }
        }
        RCLCPP_INFO(this->get_logger(), "[BTengine]: Loaded %zu points from json sequence", json_point.size());
    } else {
        RCLCPP_WARN(this->get_logger(), "[BTengine]: No 'sequence' array found in json, using default");
        json_point = default_point;
    }

    auto msg = std_msgs::msg::Int32MultiArray();
    msg.data = json_point;
    json_point_pub->publish(msg);
}

void BTengine::createTreeNodes() {
    params.nh = node;
    
    // TODO: add more nodes
    // sensors
    factory.registerNodeType<CamReceiver>("CamReceiver", params, blackboard);


    // navigation
    factory.registerNodeType<NavigationActionNode>("NavigationActionNode", params);
    factory.registerNodeType<Docking>("Docking", params, blackboard);
    factory.registerNodeType<StopRobotNode>("StopRobotNode", params);
    factory.registerNodeType<RotateActionNode>("RotateActionNode", params);

    // utils

    // firmware
}

void BTengine::createTree() {
    RCLCPP_INFO_STREAM(this->get_logger(), "[BTengine]: --Loading XML--");
    
    // Wait until plan file is received
    while (rclcpp::ok() && !isReady) {
        rclcpp::spin_some(node);
        rate->sleep();
    }
    
    // Build the full path to the XML file
    std::string xml_file_path = BT_XML_DIRECTORY + plan_file_name;
    RCLCPP_INFO_STREAM(this->get_logger(), "[BTengine]: Loading tree from: " << xml_file_path);
    
    // Register behavior tree from XML file
    try {
        factory.registerBehaviorTreeFromFile(xml_file_path);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "[BTengine]: Failed to load XML: %s", e.what());
        return;
    }
    
    // Export tree nodes model to XML (for Groot visualization)
    std::string xml_models = BT::writeTreeNodesModelXML(factory);
    std::ofstream file(bt_tree_node_model);
    if (file.is_open()) {
        file << xml_models;
        file.close();
        RCLCPP_INFO(this->get_logger(), "[BTengine]: Tree node model saved to: %s", bt_tree_node_model.c_str());
    } else {
        RCLCPP_WARN(this->get_logger(), "[BTengine]: Failed to save tree node model");
    }
    
    // Create tree with blackboard
    RCLCPP_INFO(this->get_logger(), "[BTengine]: --Create tree--");
    tree = factory.createTree(tree_name, blackboard);
    
    // Send ready signal to startup
    sentReadySignal();
    
    RCLCPP_INFO(this->get_logger(), "[BTengine]: Tree created successfully");
}

void BTengine::gameTimeCallback(const std_msgs::msg::Float32::SharedPtr msg) {
    game_time = msg->data;
    blackboard->set<double>("game_time", game_time);
}

void BTengine::runTree() {
    RCLCPP_INFO(this->get_logger(), "[BTengine]: --Running tree--");
    
    while (rclcpp::ok() && game_time < TERMINATE_TIME) {
        BT::NodeStatus status = tree.tickOnce();
        
        if (status == BT::NodeStatus::SUCCESS || status == BT::NodeStatus::FAILURE) {
            RCLCPP_INFO(this->get_logger(), "[BTengine]: Tree finished with status: %s", 
                        BT::toStr(status).c_str());
            break;
        }
        
        rclcpp::spin_some(node);
        rate->sleep();
    }
    
    RCLCPP_INFO(this->get_logger(), "[BTengine]: --Tree execution ended--");
}

void BTengine::setBlackboard() {
    blackboard->set<int>("team", static_cast<int>(team));
    blackboard->set<int>("robot", static_cast<int>(robot));
    blackboard->set<int>("selected_plan", selected_plan);
    blackboard->set<double>("game_time", game_time);
    
    RCLCPP_INFO(this->get_logger(), "[BTengine]: Blackboard initialized");
}

void BTengine::startCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    
    if (request->data && isReady) {
        response->success = true;
        response->message = "BTengine starting tree execution";
        RCLCPP_INFO(this->get_logger(), "[BTengine]: Received start signal, running tree...");
        runTree();
    } else {
        response->success = false;
        response->message = isReady ? "Already running" : "Not ready yet";
    }
}

// Main function
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto bt_engine = std::make_shared<BTengine>();
    
    bt_engine->createTreeNodes();
    bt_engine->setBlackboard();
    bt_engine->createTree();
    
    RCLCPP_INFO(bt_engine->get_logger(), "[BTengine]: Node ready, waiting for start signal...");
    
    rclcpp::spin(bt_engine);
    rclcpp::shutdown();
    
    return 0;
}

