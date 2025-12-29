#ifndef TF2_OBSTACLE_DETECTOR__OBSTACLE_MONITOR_HPP_
#define TF2_OBSTACLE_DETECTOR__OBSTACLE_MONITOR_HPP_

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace tf2_obstacle_detector{
    class ObstacleMonitor : public rclcpp::Node
    {
        public:
            ObstacleMonitor();
        private:
            void control_cycle();

            rclcpp::TimerBase::SharedPtr  timer_;
            rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

            tf2_ros::Buffer tf_buffer_;
            tf2_ros::TransformListener tf_listener_;

    };
} //namespace



#endif