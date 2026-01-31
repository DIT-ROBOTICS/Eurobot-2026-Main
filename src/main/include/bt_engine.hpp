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

// receiver nodes

// utils nodes

// firmware nodes

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

class BTengine : virtual public rclcpp::Node {

public:
    BTengine();
    void initParam();

    /**
     * @brief sent ready signal client request to startup node
     */
    void sentReadySignal();
    
    void createTreeNodes();

    /**
     * @brief read plan file and create tree
     */
    
    void addJsonPoint();
    void createTree();
    void runTree();
    void setBlackboard();
    void publishGameTime();
    void gameTimeCallback(const std_msgs::msg::Float32::SharedPtr msg);
    void readyCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void planFileCallback(const std_msgs::msg::String::SharedPtr msg);
    void startCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response);

private:
    // startup relative
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr start_srv_server;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr are_you_ready_sub;
    rclcpp::Client<bt_cpp_ros2_interfaces::srv::StartUpSrv>::SharedPtr ready_srv_client;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr plan_file_sub;   
    // blackboard variable setting
    // TODO: add more blackboard variables for another extension
    Team team;
    Robot robot;
    int selected_plan;
    std::string plan_file_name;
    double game_time;
    BT::Blackboard::Ptr blackboard;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr game_time_sub;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr json_point_pub;
    std::vector<int> json_point;
    
    // Tree creation state
    bool isReady;           // Flag indicating plan_file received
    std::string tree_name;  // Name of the main tree to create (e.g., "MainTree")
    std::string bt_tree_node_model;  // Path to save tree node model XML
    int group;              // Group ID for ready signal

    // BT utilities
    BT::Tree tree;
    BT::BehaviorTreeFactory factory;
    BT::RosNodeParams params;

    // ros variables
    std::shared_ptr<rclcpp::Node> node;
    rclcpp::Rate rate;
};

