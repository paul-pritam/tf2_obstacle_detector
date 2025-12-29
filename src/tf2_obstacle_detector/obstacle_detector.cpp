#include "tf2_obstacle_detector/obstacle_detector.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <cmath>
#include <limits>

namespace tf2_obstacle_detector
{
    ObstacleDetector::ObstacleDetector() 
    : Node("obstacle_detector_node"), 
      tf_buffer_(this->get_clock()), 
      tf_listener_(tf_buffer_)
    {
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "input_scan", rclcpp::SensorDataQoS(), 
            std::bind(&ObstacleDetector::scan_callback, this, std::placeholders::_1));
        
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
    }

    void ObstacleDetector::scan_callback(const sensor_msgs::msg::LaserScan::UniquePtr scan_msg)
    {
       float min_dist = std::numeric_limits<float>::infinity();
       int closest_idx = -1;

       for (size_t i = 0; i < scan_msg->ranges.size(); ++i)
       {
           float r = scan_msg->ranges[i];

           if (std::isfinite(r) && r >= scan_msg->range_min && r <= scan_msg->range_max)
           {
                if (r < min_dist){
                    min_dist = r;
                    closest_idx = i;
                }
           }
       }


       if (closest_idx != -1 && min_dist < 2.0){
            
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000, 
                "Obstacle Detected! Dist: %.2fm. Calculating Transform...", min_dist);

            geometry_msgs::msg::TransformStamped map2laser_msg; 

            try{

                map2laser_msg = tf_buffer_.lookupTransform(
                    "map", 
                    scan_msg->header.frame_id, 
                    rclcpp::Time(0), 
                    rclcpp::Duration::from_seconds(0.1));
                
                tf2::Transform map2laser;
                tf2::fromMsg(map2laser_msg.transform, map2laser);

                // Polar -> Cartesian
                float obstacle_angle = scan_msg->angle_min + (closest_idx * scan_msg->angle_increment); 

                float x = min_dist * std::cos(obstacle_angle);
                float y = min_dist * std::sin(obstacle_angle);

                tf2::Transform laser2obstacle;
                laser2obstacle.setOrigin(tf2::Vector3(x, y, 0.0));
                laser2obstacle.setRotation(tf2::Quaternion(0, 0, 0, 1)); // No rotation relative to laser

                // Matrix Multiplication: Map -> Laser * Laser -> Obstacle
                tf2::Transform map2obstacle = map2laser * laser2obstacle;

                geometry_msgs::msg::TransformStamped detection_tf;
                detection_tf.header.frame_id = "map";
                detection_tf.child_frame_id = "detected_obstacle";
                detection_tf.header.stamp = scan_msg->header.stamp;
                detection_tf.transform = tf2::toMsg(map2obstacle);
                
                tf_broadcaster_->sendTransform(detection_tf);
                
            }
            catch(tf2::TransformException & ex){
                RCLCPP_WARN(get_logger(), "TF Error (Map Lookup): %s", ex.what());
            }
       }
       else {

           if (closest_idx == -1) {
               RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "No valid points in scan.");
           } else {
               RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Obstacle too far: %.2fm (Limit is 2.0m)", min_dist);
           }
       }
    }
}