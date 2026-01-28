#ifndef BT_CONFIG_HPP
#define BT_CONFIG_HPP

#include <string>
#include <stdexcept>

enum class Team {
    YELLOW,
    BLUE
};

enum class Robot {
    WHITE,
    BLACK
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
    if (str == "white" || str == "WHITE") return Robot::WHITE;
    if (str == "black" || str == "BLACK") return Robot::BLACK;
    throw std::runtime_error("Invalid robot string: " + str);
}

#endif // BT_CONFIG_HPP
