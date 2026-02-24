#include "bt_engine.hpp"

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
    
    // Initialize parameters first
    initParam();
    
    rate = std::make_shared<rclcpp::Rate>(time_rate);
    
    // Initialize new variables
    isReady = false;
    readySent = false;  // Will be set to true after first ready signal
    treeCreated = false;  // Will be set to true after tree is created
    canStart = false;  // Will be set to true when start signal is received
    group = 1;  // Main BT group
    file_logged = false;
}

void BTengine::init() {
    // This must be called after construction when shared_ptr is available
    node = this->shared_from_this();
    params = BT::RosNodeParams(node);
}

void BTengine::initParam() {
    // Declare parameters with defaults
    this->declare_parameter<int>("time_rate", 100);
    this->declare_parameter<int>("game_time", 100);
    this->declare_parameter<std::string>("pkg_share_dir", "/home/main/eurobot-2026-main-ws/install/bt_core/share/bt_core");
    this->declare_parameter<std::string>("tree_name", "MainTree");
    this->declare_parameter<std::string>("json_file_path", "params/mission_sequence.json");
    this->declare_parameter<std::string>("bt_xml_directory", "bt/");
    this->declare_parameter<std::string>("bt_tree_node_model", "bt/tree_node_model.xml");
    
    // Get parameters
    this->get_parameter("time_rate", time_rate);
    this->get_parameter("game_time", terminate_time);
    this->get_parameter("pkg_share_dir", pkg_share_dir);
    this->get_parameter("tree_name", tree_name);
    
    std::string json_rel_path, bt_xml_rel_path, bt_model_rel_path;
    this->get_parameter("json_file_path", json_rel_path);
    this->get_parameter("bt_xml_directory", bt_xml_rel_path);
    this->get_parameter("bt_tree_node_model", bt_model_rel_path);
    
    // Build full paths
    json_file_path = pkg_share_dir + "/" + json_rel_path;
    bt_xml_directory = pkg_share_dir + "/" + bt_xml_rel_path;
    bt_tree_node_model = pkg_share_dir + "/" + bt_model_rel_path;
    
    RCLCPP_INFO(this->get_logger(), "[BTengine] JSON path: %s", json_file_path.c_str());
    RCLCPP_INFO(this->get_logger(), "[BTengine] BT XML dir: %s", bt_xml_directory.c_str());
    RCLCPP_INFO(this->get_logger(), "[BTengine] Time rate: %d, Terminate time: %d", time_rate, terminate_time);
}

void BTengine::readyCallback(const std_msgs::msg::Bool::SharedPtr msg) {
    // Only send ready signal after tree is actually created
    if(msg->data && treeCreated && !readySent) {
        sentReadySignal();
        readySent = true;  // Only send once
        RCLCPP_INFO(this->get_logger(), "[BTengine]: Sent ready signal to startup (tree is ready)");
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

    if (!file_logged) {
        RCLCPP_INFO(this->get_logger(), "[BTengine]: Received plan file for robot %s, team %s, plan %d", robotToString(robot).c_str(), teamToString(team).c_str(), selected_plan);
        file_logged = true;
    }
    
    // Mark as ready - createTree() is waiting for this
    isReady = true;
}

void BTengine::addJsonPoint() {
    std::ifstream json_file(json_file_path);
    if (!json_file.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "[BTengine]: Failed to open json file");
        return;
    }
    Json::Value root;
    Json::CharReaderBuilder reader;
    std::string errors;
    
    if (!Json::parseFromStream(reader, json_file, &root, &errors)) {
        RCLCPP_ERROR(this->get_logger(), "[BTengine]: Failed to parse json: %s", errors.c_str());
        json_file.close();
        return;
    }
    json_file.close();
    
    // Extract "pantry_sequence" array for PUT action priority
    std::vector<int> pantry_seq;
    if (root.isMember("pantry_sequence") && root["pantry_sequence"].isArray()) {
        for (const auto& val : root["pantry_sequence"]) {
            if (val.isInt()) {
                pantry_seq.push_back(val.asInt());
            }
        }
        blackboard->set<std::vector<int>>("pantry_sequence", pantry_seq);
        RCLCPP_INFO(this->get_logger(), "[BTengine]: Loaded pantry_sequence with %zu items", pantry_seq.size());
    } else {
        blackboard->set<std::vector<int>>("pantry_sequence", std::vector<int>());
        RCLCPP_WARN(this->get_logger(), "[BTengine]: No 'pantry_sequence' found, will use reward system");
    }
    
    // Extract "collection_sequence" array for TAKE action priority
    std::vector<int> collection_seq;
    if (root.isMember("collection_sequence") && root["collection_sequence"].isArray()) {
        for (const auto& val : root["collection_sequence"]) {
            if (val.isInt()) {
                collection_seq.push_back(val.asInt());
            }
        }
        blackboard->set<std::vector<int>>("collection_sequence", collection_seq);
        RCLCPP_INFO(this->get_logger(), "[BTengine]: Loaded collection_sequence with %zu items", collection_seq.size());
    } else {
        blackboard->set<std::vector<int>>("collection_sequence", std::vector<int>());
        RCLCPP_WARN(this->get_logger(), "[BTengine]: No 'collection_sequence' found, will use reward system");
    }
}

void BTengine::createTreeNodes() {
    params.nh = node;
    
    // sensors / receivers
    factory.registerNodeType<CamReceiver>("CamReceiver", params, blackboard);
    factory.registerNodeType<LocReceiver>("LocReceiver", params, blackboard);

    // decision core
    factory.registerNodeType<DecisionCore>("DecisionCore", params, blackboard);
    
    // mission publisher
    factory.registerNodeType<MissionPublisher>("MissionPublisher", params, blackboard);

    // field updater
    factory.registerNodeType<FieldUpdater>("FieldUpdater", params, blackboard);
    
    params.default_port_value = "dock_robot";
    // navigation
    // factory.registerNodeType<NavigationActionNode>("NavigationActionNode", params);  // Source commented out
    // factory.registerNodeType<Docking>("Docking", params, blackboard);  // Source commented out
    factory.registerNodeType<OnDockAction>("OnDockAction", params, blackboard);
    // factory.registerNodeType<StopRobotNode>("StopRobotNode", params);  // Source commented out
    // factory.registerNodeType<RotateActionNode>("RotateActionNode", params);  // Source commented out

    // utils

    // firmware
}

void BTengine::createTree() {
    if (!isReady) {
        RCLCPP_WARN(this->get_logger(), "[BTengine]: createTree called but plan file not ready yet");
        return;
    }
    
    RCLCPP_INFO(this->get_logger(), "[BTengine]: Plan file received: %s, creating tree...", plan_file_name.c_str());
    
    // Build the full path to the XML file
    std::string xml_file_path = bt_xml_directory + plan_file_name;
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
    try {
        tree = factory.createTree(tree_name, blackboard);
        treeCreated = true;
        RCLCPP_INFO(this->get_logger(), "[BTengine]: Tree created successfully");
    } catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "[BTengine]: Failed to create tree: %s", e.what());
        treeCreated = false;
    }
}

void BTengine::gameTimeCallback(const std_msgs::msg::Float32::SharedPtr msg) {
    game_time = msg->data;
    blackboard->set<double>("game_time", game_time);
}

void BTengine::runTree() {
    BT::NodeStatus status = BT::NodeStatus::RUNNING;
    
    RCLCPP_INFO(this->get_logger(), "[BTengine]: --Running tree--");
    
    try {
        do {
            rate->sleep();
            status = tree.rootNode()->executeTick();
        } while (rclcpp::ok() && status == BT::NodeStatus::RUNNING && game_time < terminate_time);
        
        RCLCPP_INFO(this->get_logger(), "[BTengine]: Tree finished with status: %s", 
                    BT::toStr(status).c_str());
    }
    catch (const std::exception& e) {
        RCLCPP_ERROR(this->get_logger(), "[BTengine]: Exception during tree execution: %s", e.what());
    }
    catch (...) {
        RCLCPP_ERROR(this->get_logger(), "[BTengine]: Unknown exception during tree execution");
    }
    
    RCLCPP_INFO(this->get_logger(), "[BTengine]: --Tree execution ended--");
}

void BTengine::setBlackboard() {
    blackboard->set<std::string>("team", teamToString(team));
    blackboard->set<std::string>("robot", robotToString(robot));
    blackboard->set<int>("selected_plan", selected_plan);
    blackboard->set<double>("game_time", game_time);
    // json_point is loaded separately via loadSequenceFromJson in DecisionCore
    
    RCLCPP_INFO(this->get_logger(), "[BTengine]: Blackboard initialized");
}

void BTengine::startCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
    
    if (request->data && isReady && treeCreated) {
        response->success = true;
        response->message = "BTengine starting tree execution";
        setStartFlag(true);
        RCLCPP_INFO(this->get_logger(), "[BTengine]: Received start signal, tree will start running");
    } else {
        response->success = false;
        if (!treeCreated) {
            response->message = "Tree not created yet";
        } else if (!isReady) {
            response->message = "Not ready yet";
        } else {
            response->message = "Unknown state";
        }
        RCLCPP_WARN(this->get_logger(), "[BTengine]: Start rejected: %s", response->message.c_str());
    }
}

bool BTengine::isTreeCreated() {
    return treeCreated;
}

bool BTengine::isStarted() {
    return canStart;
}

void BTengine::setStartFlag(bool start) {
    canStart = start;
}

// Main function
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto bt_engine = std::make_shared<BTengine>();
    
    // Initialize shared_from_this() after construction
    bt_engine->init();
    
    bt_engine->createTreeNodes();
    bt_engine->addJsonPoint();
    bt_engine->setBlackboard();
    
    // Spin until tree is created
    while (rclcpp::ok() && !bt_engine->isTreeCreated()) {
        rclcpp::spin_some(bt_engine);
        
        // Only call createTree once isReady is true (plan file received)
        if (bt_engine->isReady && !bt_engine->isTreeCreated()) {
            bt_engine->createTree();
        }
        
        bt_engine->rate->sleep();
    }
    
    RCLCPP_INFO(bt_engine->get_logger(), "[BTengine]: Node ready, waiting for start signal...");
    
    // Continue spinning while running tree
    while (rclcpp::ok()) {
        rclcpp::spin_some(bt_engine);
        
        if (bt_engine->isStarted()) {
            bt_engine->runTree();
            bt_engine->setStartFlag(false);  // Reset start flag after running
        }
        
        bt_engine->rate->sleep();
    }
    
    rclcpp::shutdown();
    
    return 0;
}

