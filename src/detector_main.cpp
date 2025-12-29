#include <rclcpp/rclcpp.hpp>
#include <memory>

#include "tf2_obstacle_detector/obstacle_detector.hpp"
#include "tf2_obstacle_detector/obstacle_monitor.hpp"

int main (int argc, char *argv[]){
    rclcpp::init(argc, argv);

    auto obstacle_detector = std::make_shared<tf2_obstacle_detector::ObstacleDetector>();
    auto obstacle_monitor = std::make_shared<tf2_obstacle_detector::ObstacleMonitor>();
    
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(obstacle_detector);
    executor.add_node(obstacle_monitor);

    executor.spin();

    return 0;

}