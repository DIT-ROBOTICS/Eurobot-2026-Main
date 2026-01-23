#ifndef NAVIGATION_UTIL_HPP
#define NAVIGATION_UTIL_HPP

// behavior tree
#include <behaviortree_ros2/bt_action_node.hpp>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/behavior_tree.h>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logger.hpp>

// ROS message
#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"

// tf2
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/impl/utils.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>

// action message
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "opennav_docking_msgs/action/dock_robot.hpp"

// sensors
#include "LocReceiver.hpp"

// utils - centralized utility functions
#include "utils.hpp"

#include <deque>

#define PI 3.1415926
using namespace std;
using namespace BT;

// Use utility functions from MATH namespace
using MATH::calculateDistance;
using MATH::calculateAngleDifference;

inline geometry_msgs::msg::PoseStamped ConvertPoseFormat(geometry_msgs::msg::PoseStamped pose_) {
    tf2::Quaternion quaternion;
    tf2::fromMsg(pose_.pose.orientation, quaternion);
    pose_.pose.position.z = tf2::impl::getYaw(quaternion) * 2 / PI;
    return pose_;
}

#endif // NAVIGATION_UTIL_HPP
