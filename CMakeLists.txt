cmake_minimum_required(VERSION 3.8)
project(tf2_obstacle_detector)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(dependencies
  rclcpp
  tf2_ros
  sensor_msgs
  geometry_msgs
  visualization_msgs
  tf2_geometry_msgs
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
  src/tf2_obstacle_detector/obstacle_detector.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

add_executable(obstacle_detector_node_exe 
  src/detector_main.cpp
  src/tf2_obstacle_detector/obstacle_monitor.cpp  # <--- ADD THIS LINE
  src/tf2_obstacle_detector/obstacle_detector.cpp # <--- You probably need this too
)
ament_target_dependencies(obstacle_detector_node_exe ${dependencies})
target_link_libraries(obstacle_detector_node_exe ${PROJECT_NAME})

install(TARGETS
  ${PROJECT_NAME}
  obstacle_detector_node_exe
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include/)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(${dependencies})
ament_package()
