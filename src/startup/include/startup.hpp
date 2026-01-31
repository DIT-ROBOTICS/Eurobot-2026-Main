/* rclcpp publisher */
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "btcpp_ros2_interfaces/srv/start_up_srv.hpp" 

// tf2 
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/impl/utils.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>

#include <jsoncpp/json/json.h>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <stdio.h>
#include <string>

#include "btcpp_ros2_interfaces/msg/obstacles.hpp"
#include "btcpp_ros2_interfaces/msg/circle_obstacle.hpp"
#include "btcpp_ros2_interfaces/msg/segment_obstacle.hpp"

#include "../../main/include/bt_config.hpp"

#define PI 3.1415926

// StartUpState enum is now defined in bt_config.hpp

class StartUp : public rclcpp::Node {

public:
    // constructor
    StartUp();
    void initParam();

    // control flow
    void stateTransition();

    // init phase
    void getWebPlan();
    void parsePlanCode();

    // ready phase
    void publishSystemCheckSignal();
    void systemCheckFeedback(const std::shared_ptr<btcpp_ros2_interfaces::srv::StartUpSrv::Request> request,
        std::shared_ptr<btcpp_ros2_interfaces::srv::StartUpSrv::Response> response);
    bool isAllSystemReady();
    void startCallback(const std::shared_ptr<std_msgs::msg::Bool> msg);
    
    // ready -> start
    void publishStartSignal();
    void publishInitialPose();

    // start phase
    void tickSima(double game_time);
    void publishTime();
    bool gameOver(double game_time);

    // utils
    geometry_msgs::msg::Quaternion yaw2qua(double yaw);

private:
    // game timer
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr game_time_pub;
    std::shared_ptr<rclcpp::Rate> rate;
    int game_time;

    // State checker for other groups
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr are_you_ready_pub;   
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr plan_file_pub;
    rclcpp::Service<btcpp_ros2_interfaces::srv::StartUpSrv>::SharedPtr ready_srv_server;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr plug_sub;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_pub;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr start_signal_client;

    // Sima
    // 2026 Eurobot game have two types of sima
    // sima will be start by a int topic for potiential plan selection
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr start_sima_pub;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr start_ninja_pub;
    int sima_tick_threshold;
    int sima_selected_plan;
    bool sima_started;

    // web
    // robot computer given a web interface for plan selection
    // web interface will help select team, plan, two digit message
    // for example: 40 -> 4th plan of Yellow
    //              31 -> 3th plan of Blue
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr web_plan_client;
    bool web_request_received;

    // program variables
    Team team; // came from param, can be modify by web interface
    Robot robot; // came from param
    int selected_plan; // came from plan code
    std::string plan_file_name; // parse by plan code, robot name

    StartUpState startup_state;
    geometry_msgs::msg::PoseWithCovarianceStamped start_position;
    double start_time;
    bool is_plugged;
    int group_state[5];

    // Parameters
    std::string robot_name;
    std::vector<double> blue_start_pose;
    std::vector<double> yellow_start_pose;
};