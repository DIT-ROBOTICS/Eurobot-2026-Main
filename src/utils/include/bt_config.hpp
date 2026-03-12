#ifndef BT_CONFIG_HPP
#define BT_CONFIG_HPP

#include <string>
#include <stdexcept>
#include <vector>
#include <sstream>
#include <algorithm>

enum class DockType {
    MISSION_DOCK_Y,
    MISSION_DOCK_X,
    CAM_DOCK_Y,
    CAM_DOCK_X
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
    EMPTY = 0,
    OCCUPIED = 1,
    UNKNOWN = -1
};

enum class FlipStatus {
    NO_FLIP,
    NEED_FLIP
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
        case ActionType::NAV: return "nav";
        case ActionType::ROTATE: return "rotate";
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
