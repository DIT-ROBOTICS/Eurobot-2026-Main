// BT 
#include "behaviortree_ros2/bt_action_node.hpp"

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/decorators/loop_node.h"
#include "behaviortree_cpp/xml_parsing.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "behaviortree_cpp/utils/shared_library.h"
#include "behaviortree_cpp/blackboard.h"
// ROS
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/client.hpp"  // Service client
#include "rclcpp/service.hpp" // Service server
// ros message
#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "btcpp_ros2_interfaces/srv/start_up_srv.hpp"

// navigation nodes
#include "dock_node.hpp"
#include "material_checker_node.hpp"
#include "mission_checker_node.hpp" 
#include "nav_node.hpp"
#include "rotate_node.hpp"
#include "stopRobot_node.hpp"

// receiver nodes
#include "CamReceiver.hpp"
#include "NavReceiver.hpp"
#include "LocReceiver.hpp"
#include "TopicSubTest.hpp"

// utils nodes
#include "bt_start_action.hpp"
#include "get_black_board_action.hpp"
#include "get_location_action.hpp"
#include "set_board_action.hpp" 
#include "script_gen_action.hpp"
#include "timer_checker.hpp"

// firmware nodes
#include "banner_checker_node.hpp"
#include "firmware_mission_topic_node.hpp"
#include "integrated_mission_node.hpp"
#include "mission_fail_node.hpp"
#include "mission_success_node.hpp"
#include "nav_client_node.hpp"
#include "mission_start_node.hpp"

// C++
#include <memory>
#include <string>
#include <jsoncpp/json/json.h>
#include <fstream>

// BTcpp
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/client.hpp"
#include "rclcpp/service.hpp"
#include "btcpp_ros2_interfaces/srv/start_up_srv.hpp"
#include "bt_config.hpp"

using namespace BT;
using namespace std;

class ReadySignal : virtual public rclcpp::Node{

public:
    ReadySignal();

protected:
    virtual void readySignalCallback(const std_msgs::String::SharedPtr msg);
    void sendReadySignal(int group, int status);
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ready_sub;
    rclcpp::Client<btcpp_ros2_interfaces::src::StartUpSrv>::SharedPtr ready_srv_client;
    bool is_main_ready;
};

class BTengine : virtual public rclcpp::Node, public ReadySignal {

public:
    BTengine();
    std::shared_ptr<rclcpp::Node> getNode();
    void timeCallback(const std_msgs::msg::Float32::SharedPtr msg);
    void readyCallback(const std_msgs::msg::String::SharedPtr msg); override;
    void startCallback(const std::shared_ptr<btcpp_ros2_interfaces::srv::StartUpSrv::Request> request, 
        std::shared_ptr<btcpp_ros2_interfaces::srv::StartUpSrv::Response> response);
    
    bool shellCmd(const string &cmd, string &result);
    void initParam();
    void createTreeNodes();
    void getPlanSequence();
    void createTree();
    void runTree();

private:
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr time_sub;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_pub;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr start_srv_server;

    BT::Blackboard::Ptr blackboard;
    std::shared_ptr<rclcpp::Node> node;

    rclcpp::Rate rate;
    std::String xml_models;
    
    //Behavior Tree Factory
    BT::Tree tree;
    BT::BehaviorTreeFactory factory;
    BT::RosNodeParams params;

    //Ros msg
    std_msgs::msg::Bool stop_robot;
    std_msgs::msg:Int32MultiArray collection_info, pantry_info;

    // system variables
    double game_time;
    enum Team team;
    enum Robot robot;
    bool isReady;
    bool canStart;
    std::string groot_filename;
    std::string config_path;
    std::vector<int> plan_script;

    // parameters
    std::string groop_xml_config_directory;
    std::string bt_node_model;
    std::string bot_yellow_file;
    std::string bot_blue_file;
    std::string tree_name;
    std::string user_name;
};

