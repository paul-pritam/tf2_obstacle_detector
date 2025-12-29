#ifndef TF2_OBSTACLE_DETECTOR__OBSTACLE_DETECTOR_HPP_
#define TF2_OBSTACLE_DETECTOR__OBSTACLE_DETECTOR_HPP_

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace tf2_obstacle_detector
{
    class ObstacleDetector : public rclcpp::Node
        {
        public:
            ObstacleDetector();
        private:
            void scan_callback(const sensor_msgs::msg::LaserScan::UniquePtr msg);//callback for laser scan subscriber
            rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;//laser scan subscriber
            std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;//transform broadcaster to publish obstacle tf
            tf2_ros::Buffer tf_buffer_;//tf2 buffer
            tf2_ros::TransformListener tf_listener_;//tf2 listener to fill the buffer
        };  
} // namespace tf2_obstacle_detector

#endif // TF2_OBSTACLE_DETECTOR__OBSTACLE_DETECTOR_HPP_