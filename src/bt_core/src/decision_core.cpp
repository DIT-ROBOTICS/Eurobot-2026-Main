#include "decision_core.hpp"

// CYAN colored logs for DecisionCore testing
#define DC_COLOR "\033[36m"  // Cyan
#define DC_RESET "\033[0m"
#define DC_INFO(node, fmt, ...) RCLCPP_INFO(node->get_logger(), DC_COLOR "[DecisionCore] " fmt DC_RESET, ##__VA_ARGS__)
#define DC_WARN(node, fmt, ...) RCLCPP_WARN(node->get_logger(), DC_COLOR "[DecisionCore] " fmt DC_RESET, ##__VA_ARGS__)
#define DC_ERROR(node, fmt, ...) RCLCPP_ERROR(node->get_logger(), DC_COLOR "[DecisionCore] " fmt DC_RESET, ##__VA_ARGS__)

DecisionCore::DecisionCore(const string& name, const BT::NodeConfig& config, const RosNodeParams& params, BT::Blackboard::Ptr blackboard) : SyncActionNode(name, config)
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
    map_points = std::vector<double>();
    
    // Sequence priority initialization
    pantry_sequence = std::vector<int>();
    collection_sequence = std::vector<int>();
    use_pantry_sequence = false;
    use_collection_sequence = false;
    
    // map point
    loadMapPoints();
    loadSequenceFromJson();  // Load sequences from blackboard
    writeBlackboard();
}

void DecisionCore::loadMapPoints() {
    if(!node_ptr->has_parameter("map_points")) node_ptr->declare_parameter("map_points", std::vector<double>());
    if(node_ptr->get_parameter("map_points", map_points)) {
        RCLCPP_INFO(node_ptr->get_logger(), "map_points loaded");
    } else {
        RCLCPP_ERROR(node_ptr->get_logger(), "map_points not found");
        throw std::runtime_error("map_points not found");
    }
}

void DecisionCore::writeBlackboard() {
    blackboard_ptr->set<vector<FieldStatus>>("robot_side_status", robot_side_status);
    blackboard_ptr->set<vector<vector<FlipStatus>>>("hazelnut_status", hazelnut_status);
    blackboard_ptr->set<geometry_msgs::msg::PoseStamped>("robot_pose", robot_pose);
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
        decided_robot_side = static_cast<RobotSide>(side_idx_opt.value());
    } else {
        decided_robot_side = RobotSide::FRONT; // default
    }
}

void DecisionCore::getVisionData() {
    // Get collection_info from blackboard (stored as vector<FieldStatus> by CamReceiver)
    std::vector<FieldStatus> collection_data;
    if (!blackboard_ptr->get<std::vector<FieldStatus>>("collection_info", collection_data)) {
        RCLCPP_WARN(node_ptr->get_logger(), "collection_info not found, using defaults");
    } else {
        collection_info = collection_data;
    }
    
    // Get pantry_info from blackboard
    std::vector<FieldStatus> pantry_data;
    if (!blackboard_ptr->get<std::vector<FieldStatus>>("pantry_info", pantry_data)) {
        RCLCPP_WARN(node_ptr->get_logger(), "pantry_info not found, using defaults");
    } else {
        pantry_info = pantry_data;
    }

    // TODO: get robot side status
    // TODO: get hazelnut status
}

void DecisionCore::writeOutputPort() {
    setOutput<string>("nextActionType", actionTypeToString(decided_action_type));
    setOutput<int>("targetPoseIdx", static_cast<int>(target_goal_pose_idx));
    setOutput<int>("targetPoseSideIdx", static_cast<int>(target_pose_side_idx));
    setOutput<int>("targetDirection", static_cast<int>(target_direction));
}

BT::NodeStatus DecisionCore::tick() {
    getInputPort();
    switch(decided_action_type) {
        case ActionType::TAKE:
            doTake();
            break;
        case ActionType::PUT:
            doPut();
            break;
        case ActionType::FLIP:
            doFlip();
            break;
        case ActionType::DOCK:
            doDock();
            break;
        default:
            RCLCPP_ERROR(node_ptr->get_logger(), "Invalid action type");
            throw std::runtime_error("Invalid action type");
    }
    writeBlackboard();
    return BT::NodeStatus::SUCCESS;
}

void DecisionCore::doTake() {
    pair<GoalPose, RobotSide> target_point_info = getTargetPointInfo(ActionType::TAKE);
    target_goal_pose_idx = target_point_info.first;
    target_pose_side_idx = target_point_info.second;
    target_direction = decideDirection(target_goal_pose_idx, target_pose_side_idx);
    decided_action_type = ActionType::DOCK;
    writeOutputPort();
}

void DecisionCore::doPut() {
    pair<GoalPose, RobotSide> target_point_info = getTargetPointInfo(ActionType::PUT);
    target_goal_pose_idx = target_point_info.first;
    target_pose_side_idx = target_point_info.second;
    target_direction = decideDirection(target_goal_pose_idx, target_pose_side_idx);
    decided_action_type = ActionType::DOCK;
    writeOutputPort();
}

void DecisionCore::doFlip() {
    decided_action_type = ActionType::FLIP;
    // TODO: more action inside Flip
    writeOutputPort();
}

pair<GoalPose, RobotSide> DecisionCore::getTargetPointInfo(ActionType action_type) {
    RobotSide selected_side = getTargetSideIndex();
    
    if (action_type == ActionType::PUT) {
        // PRIORITY 1: Use pantry_sequence if available
        if (use_pantry_sequence && !pantry_sequence.empty()) {
            int next_pantry = pantry_sequence.front();
            pantry_sequence.erase(pantry_sequence.begin());  // Pop from front
            
            // Update blackboard with modified sequence
            blackboard_ptr->set<vector<int>>("pantry_sequence", pantry_sequence);
            use_pantry_sequence = !pantry_sequence.empty();  // Update flag
            
            GoalPose pose = static_cast<GoalPose>(next_pantry);
            RCLCPP_INFO(node_ptr->get_logger(), 
                        "[SEQUENCE] Selected pantry from sequence: %d, remaining: %zu",
                        next_pantry, pantry_sequence.size());
            return {pose, selected_side};
        }
        
        // PRIORITY 2: Fallback to reward system
        sortPantryPriority();
        
        if (pantry_priority.empty()) {
            RCLCPP_WARN(node_ptr->get_logger(), "No available pantry points");
            return {GoalPose::A, selected_side}; // fallback
        }
        
        GoalPose best_pantry = pantry_priority.top().pose;
        RCLCPP_INFO(node_ptr->get_logger(), "[REWARD] Selected pantry: %d with score: %d", 
                    static_cast<int>(best_pantry), pantry_priority.top().score);
        return {best_pantry, selected_side};
        
    } else if (action_type == ActionType::TAKE) {
        // PRIORITY 1: Use collection_sequence if available
        if (use_collection_sequence && !collection_sequence.empty()) {
            int next_collection = collection_sequence.front();
            collection_sequence.erase(collection_sequence.begin());  // Pop from front
            
            // Update blackboard with modified sequence
            blackboard_ptr->set<vector<int>>("collection_sequence", collection_sequence);
            use_collection_sequence = !collection_sequence.empty();  // Update flag
            
            GoalPose pose = static_cast<GoalPose>(next_collection);
            RCLCPP_INFO(node_ptr->get_logger(), 
                        "[SEQUENCE] Selected collection from sequence: %d, remaining: %zu",
                        next_collection, collection_sequence.size());
            return {pose, selected_side};
        }
        
        // PRIORITY 2: Fallback to reward system
        sortCollectionPriority();
        
        if (collection_priority.empty()) {
            RCLCPP_WARN(node_ptr->get_logger(), "No available collection points");
            return {GoalPose::K, selected_side}; // fallback
        }
        
        GoalPose best_collection = collection_priority.top().pose;
        RCLCPP_INFO(node_ptr->get_logger(), "[REWARD] Selected collection: %d with score: %d",
                    static_cast<int>(best_collection), collection_priority.top().score);
        return {best_collection, selected_side};
    }
    
    RCLCPP_ERROR(node_ptr->get_logger(), "getTargetPointInfo called with invalid action");
    return {GoalPose::A, RobotSide::FRONT};
}

RobotSide DecisionCore::getTargetSideIndex() {
    // Find first empty robot side
    for (int i = 0; i < ROBOT_SIDES; ++i) {
        if (robot_side_status[i] == FieldStatus::EMPTY) {
            return static_cast<RobotSide>(i);
        }
    }
    // If all full (for PUT), find first full side
    for (int i = 0; i < ROBOT_SIDES; ++i) {
        if (robot_side_status[i] == FieldStatus::OCCUPIED) {
            return static_cast<RobotSide>(i);
        }
    }
    return RobotSide::FRONT; // fallback
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
    
    // Get team from blackboard
    string team_str;
    if (blackboard_ptr->get<string>("team", team_str)) {
        current_team = stringToTeam(team_str);
    } else {
        current_team = Team::YELLOW; // default
        RCLCPP_WARN(node_ptr->get_logger(), "Team not found in blackboard, defaulting to YELLOW");
    }
}

// ============ Position Helpers ============
geometry_msgs::msg::Point DecisionCore::getPointPosition(GoalPose pose) {
    geometry_msgs::msg::Point point;
    int idx = static_cast<int>(pose);
    
    // 7 values per point: x, y, dir, back_off, shift, front_off, dock_dist
    constexpr int VALUES_PER_POINT = 7;
    int data_idx = idx * VALUES_PER_POINT;
    
    if (data_idx + 1 < static_cast<int>(map_points.size())) {
        point.x = map_points[data_idx];
        point.y = map_points[data_idx + 1];
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

// ============ Team-Side Classification ============
// Pantry layout:
//   YELLOW side: A, B, C, D (idx 0-3)
//   Middle (contested): E, F (idx 4-5)
//   BLUE side: G, H, I, J (idx 6-9)
bool DecisionCore::isOwnSidePantry(GoalPose pose) {
    if (current_team == Team::YELLOW) {
        return pose == GoalPose::A || pose == GoalPose::B || 
               pose == GoalPose::C || pose == GoalPose::D;
    } else { // BLUE
        return pose == GoalPose::G || pose == GoalPose::H || 
               pose == GoalPose::I || pose == GoalPose::J;
    }
}

bool DecisionCore::isMiddlePantry(GoalPose pose) {
    return pose == GoalPose::E || pose == GoalPose::F;
}

// Collection layout:
//   YELLOW side: L, K, N, M (idx 1, 0, 3, 2 in collection array)
//   BLUE side: P, O, K, Q (idx 5, 4, 0, 6 in collection array)
//   Note: K is shared/middle
bool DecisionCore::isOwnSideCollection(GoalPose pose) {
    if (current_team == Team::YELLOW) {
        return pose == GoalPose::L || pose == GoalPose::M || pose == GoalPose::N;
    } else { // BLUE
        return pose == GoalPose::O || pose == GoalPose::P || pose == GoalPose::Q;
    }
}

bool DecisionCore::isMiddleCollection(GoalPose pose) {
    // K is in the middle, contested by both teams
    return pose == GoalPose::K;
}

// ============ Advanced Reward System ============
// Pantry reward rules (for PUT):
//   +100 if EMPTY (available)
//   +80 for middle spots (E, F) - contested, get there first
//   +40 for own side spots
//   -20 for opponent side spots
//   +10 per meter closer (distance bonus)
//   -200 if rival is within 0.3m (give up)
int DecisionCore::calculatePantryScore(int pantry_idx) {
    int score = 0;
    GoalPose pose = static_cast<GoalPose>(pantry_idx);
    
    // Base: availability check
    if (pantry_info[pantry_idx] == FieldStatus::EMPTY) {
        score += SCORE_BASE_AVAILABLE;
    } else if (pantry_info[pantry_idx] == FieldStatus::OCCUPIED) {
        return -1000; // Cannot put if occupied, skip entirely
    }
    
    // Middle bonus: E, F are contested, prioritize
    if (isMiddlePantry(pose)) {
        score += SCORE_MIDDLE_BONUS;
    }
    // Own side bonus
    else if (isOwnSidePantry(pose)) {
        score += SCORE_OWN_SIDE_BONUS;
    }
    // Opponent side penalty
    else {
        score += SCORE_OPPONENT_SIDE_PENALTY;
    }
    
    // Distance bonus: closer = better
    double distance = calculateDistance(pose);
    score += static_cast<int>((3.0 - distance) * SCORE_DISTANCE_FACTOR); // Assume max field ~3m
    
    // Rival proximity penalty
    double rival_dist = calculateRivalDistance(pose);
    if (rival_dist < RIVAL_PROXIMITY_THRESHOLD) {
        score += SCORE_RIVAL_NEARBY_PENALTY;
        RCLCPP_DEBUG(node_ptr->get_logger(), "Pantry %d: rival too close (%.2fm), penalizing", 
                     pantry_idx, rival_dist);
    }
    
    return score;
}

// Collection reward rules (for TAKE):
//   +100 if OCCUPIED (has hazelnuts)
//   +80 for middle/contested spots (K, and team-specific contested)
//   +40 for own side spots
//   -20 for opponent side spots
//   +10 per meter closer
//   -200 if rival is within 0.3m
int DecisionCore::calculateCollectionScore(int collection_idx) {
    int score = 0;
    GoalPose pose = static_cast<GoalPose>(PANTRY_LENGTH + collection_idx);
    
    // Base: availability check
    if (collection_info[collection_idx] == FieldStatus::OCCUPIED) {
        score += SCORE_BASE_AVAILABLE;
    } else if (collection_info[collection_idx] == FieldStatus::EMPTY) {
        return -1000; // Nothing to collect, skip entirely
    }
    
    // Middle bonus: K is contested by both
    if (isMiddleCollection(pose)) {
        score += SCORE_MIDDLE_BONUS;
    }
    // Own side bonus
    else if (isOwnSideCollection(pose)) {
        score += SCORE_OWN_SIDE_BONUS;
    }
    // Opponent side penalty
    else {
        score += SCORE_OPPONENT_SIDE_PENALTY;
    }
    
    // Distance bonus: closer = better
    double distance = calculateDistance(pose);
    score += static_cast<int>((3.0 - distance) * SCORE_DISTANCE_FACTOR);
    
    // Rival proximity penalty
    double rival_dist = calculateRivalDistance(pose);
    if (rival_dist < RIVAL_PROXIMITY_THRESHOLD) {
        score += SCORE_RIVAL_NEARBY_PENALTY;
        RCLCPP_DEBUG(node_ptr->get_logger(), "Collection %d: rival too close (%.2fm), penalizing", 
                     collection_idx, rival_dist);
    }
    
    return score;
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
            RCLCPP_DEBUG(node_ptr->get_logger(), "Pantry %d score: %d", i, score);
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
            RCLCPP_DEBUG(node_ptr->get_logger(), "Collection %d score: %d", i, score);
        }
    }
}

void DecisionCore::doDock() {
    // Dock action - just pass through the current target
    writeOutputPort();
}

Direction DecisionCore::decideDirection(GoalPose goal_pose, RobotSide robot_side) {
    // Get target position
    geometry_msgs::msg::Point target = getPointPosition(goal_pose);
    
    // Calculate angle from robot to target
    double dx = target.x - robot_pose.pose.position.x;
    double dy = target.y - robot_pose.pose.position.y;
    double angle = atan2(dy, dx);
    
    // Convert angle to direction (0=East, 1=North, 2=West, 3=South)
    // Normalize angle to [0, 2*PI)
    if (angle < 0) angle += 2 * M_PI;
    
    // Each quadrant is PI/4 to 3*PI/4, etc.
    if (angle < M_PI / 4 || angle >= 7 * M_PI / 4) {
        return Direction::EAST;   // 0
    } else if (angle < 3 * M_PI / 4) {
        return Direction::NORTH;  // 1
    } else if (angle < 5 * M_PI / 4) {
        return Direction::WEST;   // 2
    } else {
        return Direction::SOUTH;  // 3
    }
}
