#ifndef BT_CONFIG_HPP
#define BT_CONFIG_HPP

#include <string>
#include <stdexcept>
#include <vector>
#include <sstream>
#include <algorithm>

enum class DockType {
    MISSION_DOCK_Y = 0,
    MISSION_DOCK_X = 1,
    CAM_DOCK_Y = 2,
    CAM_DOCK_X = 3
};

struct MapPoint {
    double x;
    double y;
    double staging_dist;
    double sign;
    DockType dock_type;
    int direction;
};

enum class StartUpState {
    INIT,
    READY,
    START,
    END,
    ERROR
};

enum class Team {
    YELLOW,
    BLUE
};

enum class Robot {
    WHITE,
    BLACK
};

enum class ActionType {
    TAKE,
    PUT,
    FLIP,
    DOCK,
    GO_HOME,
    CURSOR,
    STEAL,
    NAV,
    ROTATE
};

enum class RobotSide {
    FRONT,
    RIGHT,
    BACK,
    LEFT
};

enum class GoalPose {
    A, B, C, D, E, F, G, H, I, J, // pantry pose point index
    K, L, M, N, O, P, Q, R, // collection pose point index
    YellowNinjaPantry, BlueNinjaPantry, YellowCursor, BlueCursor, YellowHome, BlueHome 
};

constexpr int PANTRY_LENGTH = 10;
constexpr int COLLECTION_LENGTH = 8;
constexpr int HAZELNUT_LENGTH = 4;
constexpr int ROBOT_SIDES = 4;

enum class FieldStatus {
    UNKNOWN = -1
    EMPTY = 0,
    OCCUPIED = 1,
    OCCUPIED_BY_RIVAL = 2,
    OCCIPIED_BY_BOTH = 3,
    OCCIPIED_CAN_STEAL = 4
};

enum class FlipStatus {
    NO_FLIP,
    NEED_FLIP,
    NO_TAKE,
    NEED_TAKE
};

enum class Direction {
    NORTH,
    EAST,
    SOUTH,
    WEST
};

inline std::string teamToString(Team team) {
    switch (team) {
        case Team::YELLOW: return "yellow";
        case Team::BLUE: return "blue";
        default: return "unknown";
    }
}

inline std::string robotToString(Robot robot) {
    switch (robot) {
        case Robot::WHITE: return "white";
        case Robot::BLACK: return "black";
        default: return "unknown";
    }
}

inline Team stringToTeam(const std::string& str) {
    if (str == "yellow" || str == "YELLOW") return Team::YELLOW;
    if (str == "blue" || str == "BLUE") return Team::BLUE;
    throw std::runtime_error("Invalid team string: " + str);
}

inline Robot stringToRobot(const std::string& str) {
    std::string lower_str = str;
    std::transform(lower_str.begin(), lower_str.end(), lower_str.begin(), ::tolower);
    if (lower_str == "white") return Robot::WHITE;
    if (lower_str == "black") return Robot::BLACK;
    throw std::runtime_error("Invalid robot string: " + str);
}

// Utility function to split strings
inline std::vector<std::string> split(const std::string& str, char delimiter) {
    std::vector<std::string> tokens;
    std::stringstream ss(str);
    std::string token;
    while (std::getline(ss, token, delimiter)) {
        tokens.push_back(token);
    }
    return tokens;
}

inline ActionType stringToActionType(const std::string& str) {
    std::string lower_str = str;
    std::transform(lower_str.begin(), lower_str.end(), lower_str.begin(), ::tolower);
    if (lower_str == "take") return ActionType::TAKE;
    if (lower_str == "put") return ActionType::PUT;
    if (lower_str == "flip") return ActionType::FLIP;
    if (lower_str == "dock") return ActionType::DOCK;
    if (lower_str == "go_home") return ActionType::GO_HOME;
    if (lower_str == "cursor") return ActionType::CURSOR;
    if (lower_str == "steal") return ActionType::STEAL;
    if (lower_str == "nav") return ActionType::NAV;
    if (lower_str == "rotate") return ActionType::ROTATE;
    throw std::runtime_error("Invalid action type string: " + str);
}

inline std::string actionTypeToString(ActionType action_type) {
    switch (action_type) {
        case ActionType::TAKE: return "take";
        case ActionType::PUT: return "put";
        case ActionType::FLIP: return "flip";
        case ActionType::DOCK: return "dock";
        case ActionType::GO_HOME: return "go_home";
        case ActionType::CURSOR: return "cursor";
        case ActionType::STEAL: return "steal";
        case ActionType::NAV: return "nav";
        case ActionType::ROTATE: return "rotate";
        default: return "unknown";
    }
}

inline std::string fieldStatusToString(FieldStatus status) {
    switch (status) {
        case FieldStatus::EMPTY: return "EMPTY";
        case FieldStatus::OCCUPIED: return "OCCUPIED";
        case FieldStatus::UNKNOWN: return "UNKNOWN";
        case FieldStatus::OCCUPIED_BY_RIVAL: return "OCCUPIED_BY_RIVAL";
        case FieldStatus::OCCIPIED_BY_BOTH: return "OCCUPIED_BY_BOTH";
        case FieldStatus::OCCIPIED_CAN_STEAL: return "OCCUPIED_CAN_STEAL";
        default: return "unknown";
    }
}

inline std::string goalPoseToString(GoalPose goal_pose) {
    switch (goal_pose) {
        case GoalPose::A: return "A";
        case GoalPose::B: return "B";
        case GoalPose::C: return "C";
        case GoalPose::D: return "D";
        case GoalPose::E: return "E";
        case GoalPose::F: return "F";
        case GoalPose::G: return "G";
        case GoalPose::H: return "H";
        case GoalPose::I: return "I";
        case GoalPose::J: return "J";
        case GoalPose::K: return "K";
        case GoalPose::L: return "L";
        case GoalPose::M: return "M";
        case GoalPose::N: return "N";
        case GoalPose::O: return "O";
        case GoalPose::P: return "P";
        case GoalPose::Q: return "Q";
        case GoalPose::R: return "R";
        case GoalPose::YellowNinjaPantry: return "YellowNinjaPantry";
        case GoalPose::BlueNinjaPantry: return "BlueNinjaPantry";
        case GoalPose::YellowCursor: return "YellowCursor";
        case GoalPose::BlueCursor: return "BlueCursor";
        case GoalPose::YellowHome: return "YellowHome";
        case GoalPose::BlueHome: return "BlueHome";
        default: return "unknown";
    }
}

#endif // BT_CONFIG_HPP
