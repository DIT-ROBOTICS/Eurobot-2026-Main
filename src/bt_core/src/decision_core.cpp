#include "decision_core.hpp"

// CYAN colored logs for DecisionCore testing
#define DC_COLOR "\033[36m"  // Cyan
#define DC_RESET "\033[0m"
#define DC_INFO(node, fmt, ...) RCLCPP_INFO(node->get_logger(), DC_COLOR "[DecisionCore] " fmt DC_RESET, ##__VA_ARGS__)
#define DC_WARN(node, fmt, ...) RCLCPP_WARN(node->get_logger(), DC_COLOR "[DecisionCore] " fmt DC_RESET, ##__VA_ARGS__)
#define DC_ERROR(node, fmt, ...) RCLCPP_ERROR(node->get_logger(), DC_COLOR "[DecisionCore] " fmt DC_RESET, ##__VA_ARGS__)

DecisionCore::DecisionCore(const string& name, const BT::NodeConfig& config, const RosNodeParams& params, BT::Blackboard::Ptr blackboard) : StatefulActionNode(name, config)
{
    node_ptr = params.nh.lock();
    blackboard_ptr = blackboard;

    // Initialize field status
    collection_info = std::vector<FieldStatus>(COLLECTION_LENGTH, FieldStatus::OCCUPIED);
    pantry_info = std::vector<FieldStatus>(PANTRY_LENGTH, FieldStatus::EMPTY);
    robot_side_status = std::vector<FieldStatus>(4, FieldStatus::EMPTY);
    hazelnut_status = std::vector<vector<FlipStatus>>(4, vector<FlipStatus>(HAZELNUT_LENGTH, FlipStatus::NO_FLIP));
    pantry_priority = std::priority_queue<PointScore>();
    collection_priority = std::priority_queue<PointScore>();
    robot_pose = geometry_msgs::msg::PoseStamped();
    map_point_list = std::vector<MapPoint>();
    
    // Sequence priority initialization
    pantry_sequence = std::vector<int>();
    collection_sequence = std::vector<int>();
    use_pantry_sequence = false;
    use_collection_sequence = false;
    
    // Publishers
    score_marker_pub_ = node_ptr->create_publisher<visualization_msgs::msg::MarkerArray>("spectral_scores", 10);
    
    // load variables
    loadSequenceFromJson();
    readBlackboard();
    updatePoseData();
    loadSpectrumParams();
}

void DecisionCore::loadSpectrumParams() {
    // defaults
    pantry_params.aggressiveness = 0.0;
    pantry_params.sensitivity = 2.0;
    pantry_params.rival_sigma = 0.3;
    pantry_params.rival_distance_threshold = 0.25;

    collection_params.aggressiveness = 0.0;
    collection_params.sensitivity = 2.0;
    collection_params.rival_sigma = 0.3;
    collection_params.rival_distance_threshold = 0.25;

    if (!node_ptr->has_parameter("pantry_aggressiveness")) node_ptr->declare_parameter("pantry_aggressiveness", 0.0);
    if (!node_ptr->has_parameter("pantry_sensitivity")) node_ptr->declare_parameter("pantry_sensitivity", 2.0);
    if (!node_ptr->has_parameter("pantry_rival_sigma")) node_ptr->declare_parameter("pantry_rival_sigma", 0.3);
    if (!node_ptr->has_parameter("pantry_rival_distance_threshold")) node_ptr->declare_parameter("pantry_rival_distance_threshold", 0.25);

    if (!node_ptr->has_parameter("collection_aggressiveness")) node_ptr->declare_parameter("collection_aggressiveness", 0.0);
    if (!node_ptr->has_parameter("collection_sensitivity")) node_ptr->declare_parameter("collection_sensitivity", 2.0);
    if (!node_ptr->has_parameter("collection_rival_sigma")) node_ptr->declare_parameter("collection_rival_sigma", 0.3);
    if (!node_ptr->has_parameter("collection_rival_distance_threshold")) node_ptr->declare_parameter("collection_rival_distance_threshold", 0.25);

    node_ptr->get_parameter("pantry_aggressiveness", pantry_params.aggressiveness);
    node_ptr->get_parameter("pantry_sensitivity", pantry_params.sensitivity);
    node_ptr->get_parameter("pantry_rival_sigma", pantry_params.rival_sigma);
    node_ptr->get_parameter("pantry_rival_distance_threshold", pantry_params.rival_distance_threshold);

    node_ptr->get_parameter("collection_aggressiveness", collection_params.aggressiveness);
    node_ptr->get_parameter("collection_sensitivity", collection_params.sensitivity);
    node_ptr->get_parameter("collection_rival_sigma", collection_params.rival_sigma);
    node_ptr->get_parameter("collection_rival_distance_threshold", collection_params.rival_distance_threshold);
}

void DecisionCore::loadSequenceFromJson() {
    // Try to get sequences from blackboard (loaded by bt_engine from JSON)
    if (blackboard_ptr->get<vector<int>>("pantry_sequence", pantry_sequence)) {
        use_pantry_sequence = !pantry_sequence.empty();
        RCLCPP_INFO(node_ptr->get_logger(), "Loaded pantry_sequence with %zu items, use_sequence=%s",
                    pantry_sequence.size(), use_pantry_sequence ? "true" : "false");
    } else {
        use_pantry_sequence = false;
        RCLCPP_WARN(node_ptr->get_logger(), "pantry_sequence not found in blackboard");
    }
    
    if (blackboard_ptr->get<vector<int>>("collection_sequence", collection_sequence)) {
        use_collection_sequence = !collection_sequence.empty();
        RCLCPP_INFO(node_ptr->get_logger(), "Loaded collection_sequence with %zu items, use_sequence=%s",
                    collection_sequence.size(), use_collection_sequence ? "true" : "false");
    } else {
        use_collection_sequence = false;
        RCLCPP_WARN(node_ptr->get_logger(), "collection_sequence not found in blackboard");
    }
}

void DecisionCore::readBlackboard() {
    if (!blackboard_ptr->get<vector<FieldStatus>>("robot_side_status", robot_side_status)) 
        RCLCPP_WARN(node_ptr->get_logger(), "robot_side_status not found in blackboard");

    if (!blackboard_ptr->get<geometry_msgs::msg::PoseStamped>("robot_pose", robot_pose)) 
        RCLCPP_WARN(node_ptr->get_logger(), "robot_pose not found in blackboard");

    if (!blackboard_ptr->get<geometry_msgs::msg::PoseStamped>("rival_pose", rival_pose)) 
        RCLCPP_WARN(node_ptr->get_logger(), "rival_pose not found in blackboard");

    if (!blackboard_ptr->get<vector<FieldStatus>>("collection_info", collection_info)) 
        RCLCPP_WARN(node_ptr->get_logger(), "collection_info not found in blackboard");
    
    if (!blackboard_ptr->get<vector<FieldStatus>>("pantry_info", pantry_info)) 
        RCLCPP_WARN(node_ptr->get_logger(), "pantry_info not found in blackboard");
    
    if (!blackboard_ptr->get<vector<vector<FlipStatus>>>("hazelnut_status", hazelnut_status)) 
        RCLCPP_WARN(node_ptr->get_logger(), "hazelnut_status not found in blackboard");

    if (!blackboard_ptr->get<vector<MapPoint>>("MapPointList", map_point_list)) 
        RCLCPP_ERROR(node_ptr->get_logger(), "MapPointList not found in blackboard");

    // Get team from blackboard
    string team_str;
    if (blackboard_ptr->get<string>("team", team_str)) 
        current_team = stringToTeam(team_str);
    else {
        current_team = Team::YELLOW; // default
        RCLCPP_WARN(node_ptr->get_logger(), "Team not found in blackboard, defaulting to YELLOW");
    }

    // Get robot from blackboard
    string robot_str;
    if (blackboard_ptr->get<string>("robot", robot_str)) 
        current_robot = stringToRobot(robot_str);
    else {
        current_robot = Robot::WHITE; // default
        RCLCPP_WARN(node_ptr->get_logger(), "Robot not found in blackboard, defaulting to WHITE");
    }
}

BT::PortsList DecisionCore::providedPorts()
{
    return {
        InputPort<string>("ActionType"),
        InputPort<int>("RobotSideIdx"),
        OutputPort<string>("nextActionType"),
        OutputPort<int>("targetPoseIdx"),
        OutputPort<int>("targetPoseSideIdx"),
        OutputPort<int>("targetDirection")
    };
}

void DecisionCore::getInputPort() {
    auto action_type_opt = getInput<string>("ActionType");
    if (action_type_opt) {
        decided_action_type = stringToActionType(action_type_opt.value());
    } else {
        RCLCPP_ERROR(node_ptr->get_logger(), "ActionType input not found");
        decided_action_type = ActionType::TAKE; // default
    }
    
    auto side_idx_opt = getInput<int>("RobotSideIdx");
    if (side_idx_opt) {
        default_robot_side = static_cast<RobotSide>(side_idx_opt.value());
    } else {
        default_robot_side = RobotSide::FRONT; // default
    }
}

void DecisionCore::writeOutputPort() {
    setOutput<string>("nextActionType", actionTypeToString(decided_action_type));
    setOutput<int>("targetPoseIdx", static_cast<int>(target_goal_pose_idx));
    setOutput<int>("targetPoseSideIdx", static_cast<int>(target_pose_side_idx));
    setOutput<int>("targetDirection", static_cast<int>(target_direction));
}

void DecisionCore::updateVisitedPoints() {
    // Add to visited_collections
    if(decided_action_type == ActionType::TAKE) {
        std::vector<int> visited_collections;
        if (!blackboard_ptr->get<std::vector<int>>("visited_collections", visited_collections)) {
            DC_WARN(node_ptr, "Failed to get visited_collections from blackboard (might be empty/first time)");
        }
        
        int local_idx = static_cast<int>(target_goal_pose_idx) - PANTRY_LENGTH;
        if (std::find(visited_collections.begin(), visited_collections.end(), local_idx) == visited_collections.end()) {
            visited_collections.push_back(local_idx);
            blackboard_ptr->set<std::vector<int>>("visited_collections", visited_collections);
            DC_INFO(node_ptr, "Added collection field %d (Pose %d) to visited_collections", local_idx, static_cast<int>(target_goal_pose_idx));
        }
    }
    else if (decided_action_type == ActionType::PUT) {
        // Add to visited_pantries
        std::vector<int> visited_pantries;
        if (!blackboard_ptr->get<std::vector<int>>("visited_pantries", visited_pantries)) {
            DC_WARN(node_ptr, "Failed to get visited_pantries from blackboard (might be empty/first time)");
        }
        
        int local_idx = static_cast<int>(target_goal_pose_idx) - COLLECTION_LENGTH;
        if (std::find(visited_pantries.begin(), visited_pantries.end(), local_idx) == visited_pantries.end()) {
            visited_pantries.push_back(local_idx);
            blackboard_ptr->set<std::vector<int>>("visited_pantries", visited_pantries);
            DC_INFO(node_ptr, "Added pantry field %d (Pose %d) to visited_pantries", local_idx, static_cast<int>(target_goal_pose_idx));
        }
    }
}

BT::NodeStatus DecisionCore::onStart() {
    readBlackboard();
    getInputPort();
    updatePoseData();
    publishScoreMarkers();
    
    DC_INFO(node_ptr, "Processing action: %s", actionTypeToString(decided_action_type).c_str());
    switch(decided_action_type) {
        case ActionType::TAKE:
            return doTake();
        case ActionType::PUT:
            return doPut();
        case ActionType::FLIP:
            return doFlip();
        case ActionType::DOCK:
            return doDock();
        case ActionType::GO_HOME:
            return doGoHome();
        case ActionType::CURSOR:
            return doCursor();
        default:
            DC_ERROR(node_ptr, "Invalid action type");
            throw std::runtime_error("Invalid action type");
    }
    return BT::NodeStatus::FAILURE;
}

BT::NodeStatus DecisionCore::onRunning() {
    // Re-evaluate on every running tick
    return onStart();
}

void DecisionCore::onHalted() {
    // Nothing to clean up
}

BT::NodeStatus DecisionCore::doTake() {
    auto target_point_info_opt = getTargetPointInfo(ActionType::TAKE);
    if (!target_point_info_opt) {
        DC_INFO(node_ptr, "No valid TAKE target found. Waiting...");
        return BT::NodeStatus::RUNNING;
    }
    auto target_point_info = target_point_info_opt.value();
    target_goal_pose_idx = target_point_info.first;
    target_pose_side_idx = target_point_info.second;
    updateVisitedPoints();
    target_direction = decideDirection(target_goal_pose_idx, target_pose_side_idx);
    decided_action_type = ActionType::DOCK;
    writeOutputPort();
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus DecisionCore::doPut() {
    auto target_point_info_opt = getTargetPointInfo(ActionType::PUT);
    if (!target_point_info_opt) {
        DC_INFO(node_ptr, "No valid PUT target found. Waiting...");
        return BT::NodeStatus::RUNNING;
    }
    auto target_point_info = target_point_info_opt.value();
    target_goal_pose_idx = target_point_info.first;
    target_pose_side_idx = target_point_info.second;
    updateVisitedPoints();
    target_direction = decideDirection(target_goal_pose_idx, target_pose_side_idx);
    decided_action_type = ActionType::DOCK;
    writeOutputPort();
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus DecisionCore::doFlip() {
    decided_action_type = ActionType::FLIP;
    // TODO: more action inside Flip
    target_pose_side_idx = getTargetSideIndex(ActionType::FLIP);
    writeOutputPort();
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus DecisionCore::doGoHome() {    
    printFieldInfo();
    if(current_team == Team::YELLOW) target_goal_pose_idx = GoalPose::YellowHome;
    else if(current_team == Team::BLUE) target_goal_pose_idx = GoalPose::BlueHome;

    if(current_robot == Robot::WHITE) target_pose_side_idx = RobotSide::FRONT;
    else if(current_robot == Robot::BLACK) target_pose_side_idx = RobotSide::BACK;
    
    target_direction = decideDirection(target_goal_pose_idx, target_pose_side_idx);
    decided_action_type = ActionType::DOCK;
    writeOutputPort();
    return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus DecisionCore::doCursor() {
    if(current_team == Team::YELLOW) target_goal_pose_idx = GoalPose::YellowCursor;
    else if(current_team == Team::BLUE) target_goal_pose_idx = GoalPose::BlueCursor;

    if(current_robot == Robot::WHITE) target_pose_side_idx = RobotSide::FRONT;
    else if(current_robot == Robot::BLACK) target_pose_side_idx = RobotSide::BACK;
    
    target_direction = Direction::SOUTH;
    decided_action_type = ActionType::DOCK;
    writeOutputPort();
    return BT::NodeStatus::SUCCESS;
}

std::optional<pair<GoalPose, RobotSide>> DecisionCore::getTargetPointInfo(ActionType action_type) {
    RobotSide selected_side = getTargetSideIndex(action_type);
    printFieldInfo();
    if (action_type == ActionType::PUT) {
        // Fetch current sequence from blackboard to ensure we have the latest state
        if (blackboard_ptr->get<vector<int>>("pantry_sequence", pantry_sequence)) {
             use_pantry_sequence = !pantry_sequence.empty();
        }

        // PRIORITY 1: Use pantry_sequence if available
        while (use_pantry_sequence && !pantry_sequence.empty()) {
            int next_pantry = pantry_sequence.front();
            pantry_sequence.erase(pantry_sequence.begin());  // Pop from front
            
            // Check if pantry is already full (OCCUPIED)
            if (pantry_info[next_pantry] == FieldStatus::OCCUPIED) {
                DC_WARN(node_ptr, "[PUT->PANTRY] Skipping pantry %s (ALREADY OCCUPIED)", goalPoseToString(static_cast<GoalPose>(next_pantry)).c_str());
                // Update blackboard with modified sequence even if skipping
                blackboard_ptr->set<vector<int>>("pantry_sequence", pantry_sequence);
                continue; // Try next in sequence
            }
            
            // Update blackboard with modified sequence
            blackboard_ptr->set<vector<int>>("pantry_sequence", pantry_sequence);
            use_pantry_sequence = !pantry_sequence.empty();  // Update flag
            
            GoalPose pose = static_cast<GoalPose>(next_pantry);
            DC_INFO(node_ptr, 
                    "[PUT->PANTRY] Selected pantry %s from sequence, remaining: %zu",
                    goalPoseToString(pose).c_str(), pantry_sequence.size());
            return make_pair(pose, selected_side);
        }
        
        // PRIORITY 2: Fallback to reward system
        sortPantryPriority();
        
        if (pantry_priority.empty()) {
            RCLCPP_WARN(node_ptr->get_logger(), "No available pantry points");
            return std::nullopt; // fallback
        }
        
        GoalPose best_pantry = pantry_priority.top().pose;
        DC_INFO(node_ptr, "[PUT->PANTRY] Selected pantry %s with score: %d", 
                goalPoseToString(best_pantry).c_str(), pantry_priority.top().score);
        return make_pair(best_pantry, selected_side);
        
    } else if (action_type == ActionType::TAKE) {
        // Fetch current sequence from blackboard to ensure we have the latest state
        if (blackboard_ptr->get<vector<int>>("collection_sequence", collection_sequence)) {
            use_collection_sequence = !collection_sequence.empty();
        }

        // PRIORITY 1: Use collection_sequence if available
        while (use_collection_sequence && !collection_sequence.empty()) {
            int next_collection = collection_sequence.front();
            collection_sequence.erase(collection_sequence.begin());  // Pop from front
            
            // Check if collection is already empty (EMPTY)
            // Collection poses start at PANTRY_LENGTH (10) in GoalPose enum
            // But collection_info vector is 0-indexed relative to collections
            int collection_local_idx = next_collection - PANTRY_LENGTH;
            
            if (collection_local_idx >= 0 && collection_local_idx < static_cast<int>(collection_info.size())) {
                if (collection_info[collection_local_idx] == FieldStatus::EMPTY) {
                   DC_WARN(node_ptr, "[TAKE->COLLECTION] Skipping collection %s (ALREADY EMPTY)", goalPoseToString(static_cast<GoalPose>(next_collection)).c_str());
                   // Update blackboard with modified sequence even if skipping
                   blackboard_ptr->set<vector<int>>("collection_sequence_current", collection_sequence);
                   continue; // Try next in sequence
                }
            } else {
                DC_WARN(node_ptr, "[TAKE->COLLECTION] Index out of bounds: %d (local: %d)", next_collection, collection_local_idx);
            }
            
            // Update blackboard with modified sequence
            blackboard_ptr->set<vector<int>>("collection_sequence", collection_sequence);
            use_collection_sequence = !collection_sequence.empty();  // Update flag
            
            GoalPose pose = static_cast<GoalPose>(next_collection);
            DC_INFO(node_ptr, 
                    "[TAKE->COLLECTION] Selected collection %s from sequence, remaining: %zu",
                    goalPoseToString(pose).c_str(), collection_sequence.size());
            return make_pair(pose, selected_side);
        }
        
        // PRIORITY 2: Fallback to reward system
        sortCollectionPriority();
        
        if (collection_priority.empty()) {
            RCLCPP_WARN(node_ptr->get_logger(), "No available collection points");
            return std::nullopt; // fallback
        }
        
        GoalPose best_collection = collection_priority.top().pose;
        DC_INFO(node_ptr, "[TAKE->COLLECTION] Selected collection %s with score: %d",
                goalPoseToString(best_collection).c_str(), collection_priority.top().score);
        return make_pair(best_collection, selected_side);
    }
    
    RCLCPP_ERROR(node_ptr->get_logger(), "getTargetPointInfo called with invalid action");
    return std::nullopt;
}

RobotSide DecisionCore::getTargetSideIndex(ActionType action_type) {
    int default_idx = static_cast<int>(default_robot_side);
    
    if (action_type == ActionType::TAKE) {
        // For TAKE: Need an EMPTY side to hold hazelnuts
        DC_INFO(node_ptr, "[TAKE]: robot_side_status: %d, %d, %d, %d ", 
            static_cast<int>(robot_side_status[0]), 
            static_cast<int>(robot_side_status[1]), 
            static_cast<int>(robot_side_status[2]), 
            static_cast<int>(robot_side_status[3]));
        // Priority 1: Use default_robot_side if it's EMPTY
        if (default_idx >= 0 && default_idx < ROBOT_SIDES && 
            robot_side_status[default_idx] == FieldStatus::EMPTY) {
            DC_INFO(node_ptr, "Using default_robot_side %d (EMPTY) for TAKE", default_idx);
            return default_robot_side;
        }
        // Priority 2: Find first EMPTY side
        for (int i = 0; i < ROBOT_SIDES; ++i) {
            if (robot_side_status[i] == FieldStatus::EMPTY) {
                DC_INFO(node_ptr, "Using side %d (EMPTY) for TAKE", i);
                return static_cast<RobotSide>(i);
            }
        }
    } else if (action_type == ActionType::PUT) {
        // For PUT: Need an OCCUPIED side that has hazelnuts to put
        DC_INFO(node_ptr, "[PUT]: robot_side_status: %d, %d, %d, %d ", 
            static_cast<int>(robot_side_status[0]), 
            static_cast<int>(robot_side_status[1]), 
            static_cast<int>(robot_side_status[2]), 
            static_cast<int>(robot_side_status[3]));
        // Priority 1: Use default_robot_side if it's OCCUPIED
        if (default_idx >= 0 && default_idx < ROBOT_SIDES && 
            robot_side_status[default_idx] == FieldStatus::OCCUPIED) {
            DC_INFO(node_ptr, "Using default_robot_side %d (OCCUPIED) for PUT", default_idx);
            return default_robot_side;
        }
        // Priority 2: Find first OCCUPIED side
        for (int i = 0; i < ROBOT_SIDES; ++i) {
            if (robot_side_status[i] == FieldStatus::OCCUPIED) {
                DC_INFO(node_ptr, "Using side %d (OCCUPIED) for PUT", i);
                return static_cast<RobotSide>(i);
            }
        }
    }
    else return default_robot_side; // For FLIP and DOCK, just return default side without checks
    
    // Fallback: return default_robot_side
    DC_WARN(node_ptr, "No suitable side found, using default_robot_side %d", default_idx);
    return default_robot_side;
}

// ============ Pose Data Update ============
void DecisionCore::updatePoseData() {
    // Get robot pose from blackboard
    if (blackboard_ptr->get<geometry_msgs::msg::PoseStamped>("robot_pose", robot_pose)) {
        RCLCPP_DEBUG(node_ptr->get_logger(), "Robot pose: (%.2f, %.2f)", 
                     robot_pose.pose.position.x, robot_pose.pose.position.y);
    }
    
    // Get rival pose from blackboard
    if (blackboard_ptr->get<geometry_msgs::msg::PoseStamped>("rival_pose", rival_pose)) {
        RCLCPP_DEBUG(node_ptr->get_logger(), "Rival pose: (%.2f, %.2f)", 
                     rival_pose.pose.position.x, rival_pose.pose.position.y);
    }
}

// ============ Position Helpers ============
geometry_msgs::msg::Point DecisionCore::getPointPosition(GoalPose pose) {
    geometry_msgs::msg::Point point;
    int idx = static_cast<int>(pose);
    
    if (idx < static_cast<int>(map_point_list.size())) {
        point.x = map_point_list[idx].x;
        point.y = map_point_list[idx].y;
        point.z = 0.0;
    }
    return point;
}

double DecisionCore::calculateDistance(GoalPose pose) {
    geometry_msgs::msg::Point target = getPointPosition(pose);
    double dx = target.x - robot_pose.pose.position.x;
    double dy = target.y - robot_pose.pose.position.y;
    return sqrt(dx * dx + dy * dy);
}

double DecisionCore::calculateRivalDistance(GoalPose pose) {
    geometry_msgs::msg::Point target = getPointPosition(pose);
    double dx = target.x - rival_pose.pose.position.x;
    double dy = target.y - rival_pose.pose.position.y;
    return sqrt(dx * dx + dy * dy);
}

double DecisionCore::calculateSpectralScore(GoalPose pose, SpectrumParams params) {
    geometry_msgs::msg::Point pt = getPointPosition(pose);
    
    // 1. 座標歸一化 (自動切換黃藍方)
    double x_rel = (current_team == Team::YELLOW) ? (pt.x / 3.0) : ((3.0 - pt.x) / 3.0);
    
    // 2. 計算位置光譜分 (Exponential Spectrum)
    double pos_score = std::exp(params.aggressiveness * params.sensitivity * x_rel);
    
    // 3. 計算距離代價 (越高越近越好)
    double d_robot = calculateDistance(pose);
    double dist_factor = std::max(0.0, 1.0 - (d_robot / 3.5));
    
    // 4. 計算敵人斥力場
    double d_rival = calculateRivalDistance(pose);
    double rival_factor = 1.0 - std::exp(-std::pow(d_rival / params.rival_sigma, 2));
    
    // 如果敵人已經非常靠近，直接視為不可用
    if (d_rival < params.rival_distance_threshold) return -1000.0; 

    // 綜合評分 (乘法能確保任一項為0則總分為0)
    // Scale up the score so it acts nicely when casted to int in the priority queue
    return pos_score * dist_factor * rival_factor * 100.0;
}

int DecisionCore::calculatePantryScore(int pantry_idx) {
    GoalPose pose = static_cast<GoalPose>(pantry_idx);
    
    // Base: availability check
    if (pantry_info[pantry_idx] == FieldStatus::OCCUPIED) {
        return -1000; // Cannot put if occupied, skip entirely
    }
    
    if (pantry_info[pantry_idx] == FieldStatus::UNKNOWN) {
        // Handle UNKNOWN if necessary (for now treat as EMPTY but maybe lower score)
    }

    double score = calculateSpectralScore(pose, pantry_params);
    return static_cast<int>(score);
}

int DecisionCore::calculateCollectionScore(int collection_idx) {
    GoalPose pose = static_cast<GoalPose>(PANTRY_LENGTH + collection_idx);
    
    // Base: availability check
    if (collection_info[collection_idx] == FieldStatus::EMPTY) {
        return -1000; // Nothing to collect, skip entirely
    }
    
    double score = calculateSpectralScore(pose, collection_params);
    return static_cast<int>(score);
}

void DecisionCore::sortPantryPriority() {
    // Update pose data before scoring
    updatePoseData();
    
    // Clear and rebuild priority queue
    pantry_priority = priority_queue<PointScore>();
    
    for (int i = 0; i < PANTRY_LENGTH; ++i) {
        int score = calculatePantryScore(i);
        if (score > 0) { // Only add viable options
            GoalPose pose = static_cast<GoalPose>(i);
            pantry_priority.push(PointScore(pose, score));
            RCLCPP_DEBUG(node_ptr->get_logger(), "Pantry %s score: %d", goalPoseToString(pose).c_str(), score);
        }
    }
}

void DecisionCore::sortCollectionPriority() {
    // Update pose data before scoring
    updatePoseData();
    
    // Clear and rebuild priority queue
    collection_priority = priority_queue<PointScore>();
    
    for (int i = 0; i < COLLECTION_LENGTH; ++i) {
        int score = calculateCollectionScore(i);
        if (score > 0) { // Only add viable options
            GoalPose pose = static_cast<GoalPose>(PANTRY_LENGTH + i);
            collection_priority.push(PointScore(pose, score));
            RCLCPP_DEBUG(node_ptr->get_logger(), "Collection %s score: %d", goalPoseToString(pose).c_str(), score);
        }
    }
}

void DecisionCore::printFieldInfo() {
    std::string coll_str = "";
    for (int i = 0; i < COLLECTION_LENGTH; ++i) {
        coll_str += "[" + std::to_string(i + PANTRY_LENGTH) + "]" + fieldStatusToString(collection_info[i]).c_str() + " ";
    }
    DC_INFO(node_ptr, "[FIELD INFO] Collection: %s", coll_str.c_str());

    std::string pan_str = "";
    for (int i = 0; i < PANTRY_LENGTH; ++i) {
        pan_str += "[" + std::to_string(i) + "]" + fieldStatusToString(pantry_info[i]).c_str() + " ";
    }
    DC_INFO(node_ptr, "[FIELD INFO] Pantry: %s", pan_str.c_str());
}

void DecisionCore::doDock() {
    // Dock action - just pass through the current target
    writeOutputPort();
}

Direction DecisionCore::decideDirection(GoalPose goal_pose, RobotSide robot_side) {
    (void)robot_side; // Suppress unused parameter warning
    int idx = static_cast<int>(goal_pose);
    DC_INFO(node_ptr, "[DecisionCore] decideDirection for goal_pose index %d", idx);
    if (idx >= static_cast<int>(map_point_list.size())) {
        DC_ERROR(node_ptr, "Invalid goal_pose index %d for decideDirection", idx);
        return Direction::EAST;
    }
    
    double sign = map_point_list[idx].sign;
    int dock_type = static_cast<int>(map_point_list[idx].dock_type);
    
    // sign indicates which direction the target is (where selected side should face):
    //   dock_x, sign=1.0  -> target at WEST
    // chuang: why +x is west
    //   dock_x, sign=-1.0 -> target at EAST
    //   dock_y, sign=1.0  -> target at SOUTH
    //   dock_y, sign=-1.0 -> target at NORTH
    
    Direction result;
    if (dock_type == 0 || dock_type == 2) {  // DOCK_Y types
        result = (sign > 0) ? Direction::SOUTH : Direction::NORTH;
        DC_INFO(node_ptr, "Dock direction for pose %d DOCK_Y (sign=%.1f): %s", 
                idx, sign, result == Direction::SOUTH ? "SOUTH" : "NORTH");
    } else {  // DOCK_X types
        result = (sign > 0) ? Direction::WEST : Direction::EAST;
        DC_INFO(node_ptr, "Dock direction for pose %d DOCK_X (sign=%.1f): %s", 
                idx, sign, result == Direction::WEST ? "WEST" : "EAST");
    }
    
    return result;
}

void DecisionCore::publishScoreMarkers() {
    if (!score_marker_pub_ || map_point_list.empty()) return;

    visualization_msgs::msg::MarkerArray marker_array;
    rclcpp::Time now = node_ptr->now();

    auto add_score_marker = [&](GoalPose pose, int score, int id) {
        geometry_msgs::msg::Point pt = getPointPosition(pose);
        
        // Use cylindrical marker to indicate score via height and color
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map"; // Assuming map frame
        marker.header.stamp = now;
        marker.ns = "decision_scores";
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.action = visualization_msgs::msg::Marker::ADD;
        
        marker.pose.position.x = pt.x;
        marker.pose.position.y = pt.y;
        
        // Scale height somewhat proportionally to score (max score around ~100)
        double height = std::max(0.01, std::min(1.0, (score + 200.0) / 300.0)); // Rough normalization
        marker.pose.position.z = height / 2.0;

        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = height;

        // Color mapping based on score
        if (score < -500) {
            marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0; // Red (Unavailable)
            marker.color.a = 0.5;
        } else if (score < 50) {
            marker.color.r = 1.0; marker.color.g = 1.0; marker.color.b = 0.0; // Yellow (Low score)
            marker.color.a = 0.8;
        } else {
            marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0; // Green (High score)
            marker.color.a = 1.0;
        }
        
        // Text marker for exact score
        visualization_msgs::msg::Marker text_marker = marker;
        text_marker.id = id + 100; // Offset ID for text
        text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        text_marker.pose.position.z = height + 0.1; // Float above cylinder
        text_marker.scale.z = 0.1; // Text size
        text_marker.text = std::to_string(score);
        text_marker.color.r = 1.0; text_marker.color.g = 1.0; text_marker.color.b = 1.0; // White text
        text_marker.color.a = 1.0;

        marker_array.markers.push_back(marker);
        marker_array.markers.push_back(text_marker);
    };

    // Pantry score markers
    for (int i = 0; i < PANTRY_LENGTH; ++i) {
        int score = calculatePantryScore(i);
        add_score_marker(static_cast<GoalPose>(i), score, i);
    }

    // Collection score markers
    for (int i = 0; i < COLLECTION_LENGTH; ++i) {
        int score = calculateCollectionScore(i);
        add_score_marker(static_cast<GoalPose>(PANTRY_LENGTH + i), score, PANTRY_LENGTH + i);
    }

    score_marker_pub_->publish(marker_array);
}
