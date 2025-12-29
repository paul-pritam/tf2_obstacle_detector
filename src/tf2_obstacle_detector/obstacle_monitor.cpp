#include "tf2_obstacle_detector/obstacle_monitor.hpp"

namespace tf2_obstacle_detector
{
    ObstacleMonitor::ObstacleMonitor():Node("obstacle_monitor"), tf_buffer_(this->get_clock()),tf_listener_(tf_buffer_){

        marker_pub_=create_publisher<visualization_msgs::msg::Marker>("obstacle_marker",1);
        timer_=create_wall_timer(std::chrono::milliseconds(500),std::bind(&ObstacleMonitor::control_cycle,this));

    }

    void ObstacleMonitor::control_cycle(){
        geometry_msgs::msg::TransformStamped robot2obstacle;
        
        try{
            robot2obstacle=tf_buffer_.lookupTransform("base_link","detected_obstacle",tf2::TimePointZero);
            
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id="base_link";
            marker.header.stamp=now();
            marker.type=visualization_msgs::msg::Marker::ARROW;
            marker.action=visualization_msgs::msg::Marker::ADD;
            marker.pose.orientation.w=1.0;
            marker.scale.x=0.05;
            marker.scale.y=0.1;
            marker.scale.z=0.1;
            marker.color.r=1.0;
            marker.color.a=1.0;

            geometry_msgs::msg::Point start,end;
            start.x=0.0;
            start.y=0.0;
            end.x=robot2obstacle.transform.translation.x;
            end.y=robot2obstacle.transform.translation.y;
            marker.points.push_back(start);
            marker.points.push_back(end);

            marker_pub_->publish(marker);
        }
        catch (tf2::TransformException & ex){
            RCLCPP_WARN(get_logger(),"No obstacle detected : %s",ex.what());
        }


    }
} // namespace tf2_obstacle_detector
