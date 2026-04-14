#include "CamReceiver.hpp"
#include "bt_config.hpp"

CamReceiver::CamReceiver(const std::string& name, const BT::NodeConfig& config, 
                         const RosNodeParams& params, BT::Blackboard::Ptr blackboard)
    : BT::SyncActionNode(name, config), 
      node_(params.nh.lock()), 
      blackboard_(blackboard),
      running_(true) {

    // Create a dedicated callback group for our subscriptions
    callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    
    // Create a dedicated executor for spinning our callback group
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_callback_group(callback_group_, node_->get_node_base_interface());

    // Create subscription options with our callback group
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = callback_group_;

    // Subscribe to collection status
    collection_sub = node_->create_subscription<std_msgs::msg::Int32MultiArray>(
        "collection_status", 10, 
        std::bind(&CamReceiver::collection_info_callback, this, std::placeholders::_1),
        sub_options);
    
    // Subscribe to pantry status
    pantry_sub = node_->create_subscription<std_msgs::msg::Int32MultiArray>(
        "pantry_status", 10, 
        std::bind(&CamReceiver::pantry_info_callback, this, std::placeholders::_1),
        sub_options);
    
    // Subscribe to hazelnut flip information
    hazelnut_flip_sub = node_->create_subscription<std_msgs::msg::Int32MultiArray>(
        "/robot/vision/hazelnut/flip", 10,
        std::bind(&CamReceiver::hazelnut_flip_callback, this, std::placeholders::_1),
        sub_options);
    
        // Subscribe to pickup mask (0 means NO_TAKE should be forced)
        hazelnut_pickup_sub = node_->create_subscription<std_msgs::msg::Int32MultiArray>(
            "/robot/docking/pickup", 10,
            std::bind(&CamReceiver::hazelnut_pickup_callback, this, std::placeholders::_1),
            sub_options);

    // Subscribe to on take feedback
    on_take_feedback_sub = node_->create_subscription<std_msgs::msg::Int32MultiArray>(
        "/robot/vision/onTakeSuccess", 10,
        std::bind(&CamReceiver::onTakeFeedback, this, std::placeholders::_1),
        sub_options);

    // Subscribe to robbery pose updates
    robbery_pose_sub = node_->create_subscription<geometry_msgs::msg::PoseArray>(
        "/robbery_poses", 10,
        std::bind(&CamReceiver::robberyPoseCallback, this, std::placeholders::_1),
        sub_options);
    
    // Initialize with default values (camera fallback)
    initializeDefaultStatus();
    
    // Start background spin thread
    spin_thread_ = std::thread(&CamReceiver::spinThread, this);
    
    RCLCPP_INFO(node_->get_logger(), "CamReceiver: Started with background spin thread");
}

CamReceiver::~CamReceiver() {
    // Stop the spin thread
    running_ = false;
    if (spin_thread_.joinable()) {
        spin_thread_.join();
    }
    RCLCPP_INFO(node_->get_logger(), "CamReceiver: Destroyed, spin thread stopped");
}

void CamReceiver::spinThread() {
    RCLCPP_INFO(node_->get_logger(), "CamReceiver: Spin thread started");
    while (running_ && rclcpp::ok()) {
        executor_->spin_some(std::chrono::milliseconds(10));
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    RCLCPP_INFO(node_->get_logger(), "CamReceiver: Spin thread exiting");
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
        ROBOT_SIDES, std::vector<FlipStatus>(HAZELNUT_LENGTH, FlipStatus::NO_TAKE));
    blackboard_->set<std::vector<std::vector<FlipStatus>>>("hazelnut_status", default_hazelnut);

    // Initialize raw collection info for debugging/visualization
    std_msgs::msg::Int32MultiArray default_collection_raw;
    default_collection_raw.data = std::vector<int>(COLLECTION_LENGTH, 1); //
    blackboard_->set<std_msgs::msg::Int32MultiArray>("collection_info_raw", default_collection_raw);

    // Initialize raw pantry info with 0 (EMPTY) for debugging/visualization
    std_msgs::msg::Int32MultiArray default_pantry_raw;
    default_pantry_raw.data = std::vector<int>(PANTRY_LENGTH, 0);
    blackboard_->set<std_msgs::msg::Int32MultiArray>("pantry_info_raw", default_pantry_raw);

    // Initialize visited lists
    blackboard_->set<std::vector<int>>("visited_collections", std::vector<int>());
    blackboard_->set<std::vector<int>>("visited_pantries", std::vector<int>());

    // Initialize robbery poses
    geometry_msgs::msg::PoseArray default_robbery_poses;
    blackboard_->set<geometry_msgs::msg::PoseArray>("robbery_pantry_poses", default_robbery_poses);
}

PortsList CamReceiver::providedPorts() {
    return {};
}

void CamReceiver::collection_info_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
    RCLCPP_DEBUG(node_->get_logger(), "CamReceiver: Received collection info update.");
    collection_info = *msg;
    
    if (!collection_info.data.empty()) {
        std::vector<FieldStatus> existing_status;
        if (!blackboard_->get<std::vector<FieldStatus>>("collection_info", existing_status)) {
            RCLCPP_WARN(node_->get_logger(), "CamReceiver: collection_info not found in blackboard");
        }
        
        std::vector<int> visited_collections;
        if (!blackboard_->get<std::vector<int>>("visited_collections", visited_collections)) {
            RCLCPP_DEBUG(node_->get_logger(), "CamReceiver: visited_collections not found in blackboard");
        }

        // Convert Int32MultiArray to vector<FieldStatus> for blackboard
        std::vector<FieldStatus> collection_status;
        for (size_t i = 0; i < collection_info.data.size(); ++i) {
            int val = collection_info.data[i];
            
            // Force close if this collection has already been visited
            if (std::find(visited_collections.begin(), visited_collections.end(), i) != visited_collections.end()) {
                val = static_cast<int>(FieldStatus::EMPTY);
            }

            if (val == -1 && i < existing_status.size()) {
                collection_status.push_back(existing_status[i]);
            } else {
                collection_status.push_back(static_cast<FieldStatus>(val));
            }
        }
        // RCLCPP_INFO(node_->get_logger(), "CamReceiver: writeing updated collection_info to blackboard");
        blackboard_->set<std::vector<FieldStatus>>("collection_info", collection_status);
        blackboard_->set<std_msgs::msg::Int32MultiArray>("collection_info_raw", collection_info);
    }
}

void CamReceiver::pantry_info_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
    RCLCPP_DEBUG(node_->get_logger(), "CamReceiver: Received pantry info update.");
    pantry_info = *msg;
    
    if (!pantry_info.data.empty()) {
        std::vector<FieldStatus> existing_status;
        if (!blackboard_->get<std::vector<FieldStatus>>("pantry_info", existing_status)) {
            RCLCPP_WARN(node_->get_logger(), "CamReceiver: pantry_info not found in blackboard");
        }
        
        std::vector<int> visited_pantries;
        if (!blackboard_->get<std::vector<int>>("visited_pantries", visited_pantries)) {
            RCLCPP_DEBUG(node_->get_logger(), "CamReceiver: visited_pantries not found in blackboard");
        }

        // Convert Int32MultiArray to vector<FieldStatus> for blackboard
        std::vector<FieldStatus> pantry_status;
        for (size_t i = 0; i < pantry_info.data.size(); ++i) {
            int val = pantry_info.data[i];
            
            // Force close if this pantry has already been visited
            if (std::find(visited_pantries.begin(), visited_pantries.end(), i) != visited_pantries.end()) {
                val = static_cast<int>(FieldStatus::OCCUPIED);
            }

            if (val == -1 && i < existing_status.size()) pantry_status.push_back(existing_status[i]);
            else pantry_status.push_back(static_cast<FieldStatus>(val));
            
        }
        blackboard_->set<std::vector<FieldStatus>>("pantry_info", pantry_status);
        blackboard_->set<std_msgs::msg::Int32MultiArray>("pantry_info_raw", pantry_info);
    }
}

void CamReceiver::hazelnut_flip_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
    RCLCPP_DEBUG(node_->get_logger(), "CamReceiver: Received hazelnut flip info update.");
    
    if (msg->data.size() < 5) {
        RCLCPP_WARN(node_->get_logger(), "CamReceiver: Invalid hazelnut flip data size: %zu", msg->data.size());
        return;
    }
    
    // Parse message: indices 0-3 are flip status (0=NO_FLIP, 1=NEED_FLIP), index 4 is side index
    int side_idx = msg->data[4];
    
    if (side_idx < 0 || side_idx >= ROBOT_SIDES) {
        RCLCPP_WARN(node_->get_logger(), "CamReceiver: Invalid side index: %d", side_idx);
        return;
    }
    current_hazelnut_side_idx_ = side_idx;
    has_hazelnut_side_idx_ = true;
    
    // Check robot_side_status: skip update if this side is already OCCUPIED
    // (camera may report all zeros when it can't see the hazelnuts during movement)
    std::vector<FieldStatus> robot_sides;
    if (!blackboard_->get<std::vector<FieldStatus>>("robot_side_status", robot_sides)) {
        robot_sides = std::vector<FieldStatus>(ROBOT_SIDES, FieldStatus::EMPTY);
    }

    // Get current hazelnut_status from blackboard
    std::vector<std::vector<FlipStatus>> hazelnut_status;
    if (!blackboard_->get<std::vector<std::vector<FlipStatus>>>("hazelnut_status", hazelnut_status)) {
        // Initialize if not found
        hazelnut_status = std::vector<std::vector<FlipStatus>>(
            ROBOT_SIDES, std::vector<FlipStatus>(HAZELNUT_LENGTH, FlipStatus::NO_FLIP));
    }
    
    // Update the specified side with flip information
    for (int i = 0; i < std::min(4, HAZELNUT_LENGTH); ++i) {
        if(robot_sides[side_idx] == FieldStatus::OCCUPIED) continue;
        if(hazelnut_status[side_idx][i] == FlipStatus::NO_TAKE) continue; // Don't update if already forced NO_TAKE by pickup mask
        if (msg->data[i] == 1) {
            hazelnut_status[side_idx][i] = FlipStatus::NEED_FLIP;
        } else {
            hazelnut_status[side_idx][i] = FlipStatus::NO_FLIP;
        }
    }
    
    // Write back to blackboard
    blackboard_->set<std::vector<std::vector<FlipStatus>>>("hazelnut_status", hazelnut_status);
    
    RCLCPP_DEBUG(node_->get_logger(), 
                "CamReceiver: Updated hazelnut flip status for side %d: [%d, %d, %d, %d]",
                side_idx, msg->data[0], msg->data[1], msg->data[2], msg->data[3]);
}

void CamReceiver::hazelnut_pickup_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
    RCLCPP_DEBUG(node_->get_logger(), "CamReceiver: Received hazelnut pickup mask update.");

    if (msg->data.size() < 4) {
        RCLCPP_WARN(node_->get_logger(), "CamReceiver: Invalid hazelnut pickup data size: %zu", msg->data.size());
        return;
    }

    int side_idx = -1;
    if (msg->data.size() >= 5) {
        side_idx = msg->data[4];
    } else if (has_hazelnut_side_idx_) {
        side_idx = current_hazelnut_side_idx_;
    }

    if (side_idx < 0 || side_idx >= ROBOT_SIDES) {
        // RCLCPP_WARN(node_->get_logger(), "CamReceiver: Invalid/missing side index for pickup mask: %d", side_idx);
        return;
    }

    std::vector<std::vector<FlipStatus>> hazelnut_status;
    if (!blackboard_->get<std::vector<std::vector<FlipStatus>>>("hazelnut_status", hazelnut_status)) {
        hazelnut_status = std::vector<std::vector<FlipStatus>>(
            ROBOT_SIDES, std::vector<FlipStatus>(HAZELNUT_LENGTH, FlipStatus::NO_FLIP));
    }

    // pickup=0 means force NO_TAKE directly (override)
    for (int i = 0; i < std::min(4, HAZELNUT_LENGTH); i++) {
         if (hazelnut_status[side_idx][i] == FlipStatus::NEED_FLIP || hazelnut_status[side_idx][i] == FlipStatus::NO_FLIP) {
            continue;
        } else if (msg->data[i] == 1) {
            hazelnut_status[side_idx][i] = FlipStatus::NEED_TAKE;
        } else if (msg->data[i] == 0) {
            hazelnut_status[side_idx][i] = FlipStatus::NO_TAKE;
        }
    }

    blackboard_->set<std::vector<std::vector<FlipStatus>>>("hazelnut_status", hazelnut_status);

    RCLCPP_DEBUG(node_->get_logger(),
                "CamReceiver: Applied pickup mask for side %d: [%d, %d, %d, %d]",
                side_idx, msg->data[0], msg->data[1], msg->data[2], msg->data[3]);
}

void CamReceiver::onTakeFeedback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
    RCLCPP_DEBUG(node_->get_logger(), "CamReceiver: Received on take feedback update.");
    
    if(msg->data.size() < 4) {
        RCLCPP_WARN(node_->get_logger(), "CamReceiver: Invalid on take feedback data size: %zu", msg->data.size());
        return;
    }
    
    // Read current robot_side_status from blackboard (or initialize a default)
    std::vector<FieldStatus> robot_sides;
    if (!blackboard_->get<std::vector<FieldStatus>>("robot_side_status", robot_sides)) {
        robot_sides = std::vector<FieldStatus>(ROBOT_SIDES, FieldStatus::EMPTY);
    }
    for(size_t i = 1; i < msg->data.size() && i < robot_sides.size(); i++) { // TODO: start with 0
        robot_sides[i] = static_cast<FieldStatus>(msg->data[i]);
        // RCLCPP_WARN(node_->get_logger(), "CamReceiver: Updated robot side status for side %d: %d", i, static_cast<int>(robot_sides[i]));
    }

    blackboard_->set<std::vector<FieldStatus>>("robot_side_status", robot_sides);
}

void CamReceiver::robberyPoseCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
    RCLCPP_DEBUG(node_->get_logger(), "CamReceiver: Received robbery pose update with %zu poses.", msg->poses.size());
    
    // Store the received robbery poses in the blackboard
    blackboard_->set<geometry_msgs::msg::PoseArray>("robbery_pantry_poses", *msg);
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