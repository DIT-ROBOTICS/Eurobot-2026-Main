#ifndef UTILS_HPP
#define UTILS_HPP

#include <filesystem>
#include <fstream>
#include <deque>
#include <bitset>
#include <vector>
#include <cmath>
#include <string.h>
#include <math.h>

// Use behavior tree
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/behavior_tree.h>
#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_cpp/decorators/loop_node.h"
#include "behaviortree_cpp/utils/shared_library.h"
#include "behaviortree_cpp/blackboard.h"

// Use ROS
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>

// Use ros message
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

// tf2 
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/impl/utils.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>

#include "receiver_util.hpp"

using namespace BT;
using namespace std;

#define PI 3.1415926

namespace MATH {
    double inline calculateDistance(const geometry_msgs::msg::Pose &pose1, const geometry_msgs::msg::Pose &pose2){
        tf2::Vector3 position1(pose1.position.x, pose1.position.y, 0);
        tf2::Vector3 position2(pose2.position.x, pose2.position.y, 0);
        double dist = position1.distance(position2);
        RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "distance: " << dist);
        return dist;
    }
    double inline calculateAngleDifference(const geometry_msgs::msg::Pose &pose1, const geometry_msgs::msg::Pose &pose2){
        tf2::Quaternion orientation1, orientation2;
        tf2::fromMsg(pose1.orientation, orientation1);
        tf2::fromMsg(pose2.orientation, orientation2);
        double yaw1 = tf2::impl::getYaw(orientation1);
        double yaw2 = tf2::impl::getYaw(orientation2);
        return std::fabs(yaw1 - yaw2);
    }
    geometry_msgs::msg::PoseStamped inline ConvertPoseFormat(geometry_msgs::msg::PoseStamped &pose_){
        tf2::Quaternion quaternion;
        tf2::fromMsg(pose_.pose.orientation, quaternion);
        pose_.pose.position.z = tf2::impl::getYaw(quaternion);
        return pose_;
    }
}

#endif // UTILS_HPP