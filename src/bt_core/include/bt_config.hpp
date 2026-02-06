#ifndef BT_CONFIG_HPP
#define BT_CONFIG_HPP

#include <string>
#include <stdexcept>
#include <vector>
#include <sstream>
#include <algorithm>

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
    K, L, M, N, O, P, Q, R // collection pose point index
    YellowNinjaPantry, BlueNinjaPantry, YellowCursor, BlueCursor, YellowHome, BlueHome 
};

enum PANTRY_LENGTH = 10;
enum COLLECTION_LENGTH = 8;



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

#endif // BT_CONFIG_HPP
