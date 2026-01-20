# TF2 Obstacle Detector

A ROS 2 package for detecting obstacles from laser scan data and broadcasting their positions as TF2 (Transform) frames. This package converts sensor readings into geometric transforms, enabling other nodes to easily access obstacle locations within the ROS 2 coordinate transformation system.

## Overview

The `tf2_obstacle_detector` package provides two main components:

1. **Obstacle Detector**: Processes laser scan data and broadcasts detected obstacles as TF2 frames
2. **Obstacle Monitor**: Visualizes detected obstacles as markers in RViz and tracks their positions in the TF2 tree

This approach integrates obstacle detection with ROS 2's standard transformation framework, allowing seamless integration with other navigation and planning modules.

## Features

- **TF2-Based Representation**: Obstacles published as coordinate frames for standard ROS 2 integration
- **Laser Scan Processing**: Real-time obstacle detection from sensor data
- **RViz Visualization**: Visual markers showing obstacle positions
- **Transform Broadcasting**: Automatic frame publishing for obstacle positions
- **Multi-Node Architecture**: Separate detection and monitoring components
- **Efficient Processing**: Single-threaded executor for responsive updates

## Dependencies

- **ROS 2**: Core middleware
- **rclcpp**: ROS 2 C++ client library
- **tf2_ros**: Transform library and utilities
- **sensor_msgs**: Laser scan data types
- **geometry_msgs**: Geometric primitives and transforms
- **visualization_msgs**: RViz marker visualization
- **tf2_geometry_msgs**: TF2 message conversion utilities

## Architecture

### Components

#### Obstacle Detector Node

- Subscribes to laser scan data (`/scan`)
- Analyzes scan ranges to identify obstacles
- Broadcasts detected obstacle poses as TF2 frames
- Publishes transform data to the TF2 tree
- Maintains TF2 buffer and listener for coordinate transformations

**Key Functions:**
- `scan_callback()`: Processes incoming laser scan messages
- Obstacle identification and position calculation
- TF2 frame broadcasting for each detected obstacle

#### Obstacle Monitor Node

- Monitors TF2 frames published by Obstacle Detector
- Queries obstacle transforms at regular intervals
- Publishes RViz visualization markers
- Displays obstacle positions and sizes
- Runs on 50ms control cycle for responsive visualization

**Key Functions:**
- `control_cycle()`: Periodically queries and visualizes obstacles
- Marker generation for RViz display
- TF2 query and transform listener management

### Signal Flow

```
Laser Scan (/scan)
    |
    v
[Obstacle Detector]
    |
    +---> TF2 Broadcast (obstacle frames)
    |
    v
TF2 Tree
    |
    v
[Obstacle Monitor]
    |
    v
RViz Markers (/visualization_marker)
```

## Building

From the workspace root:

```bash
colcon build --packages-select tf2_obstacle_detector
```

## Running

### Launch the Package

```bash
ros2 launch tf2_obstacle_detector detector.launch.py
```

### Manual Node Execution

```bash
ros2 run tf2_obstacle_detector detector_main
```

### With Visualization

```bash
# Terminal 1: Start simulation with laser scanner
ros2 launch mybot launch_sim.launch.py

# Terminal 2: Run obstacle detector
ros2 run tf2_obstacle_detector detector_main

# Terminal 3: Launch RViz with robot description
ros2 launch mybot rsp.launch.py

# In RViz:
# 1. Add a Marker display
# 2. Set topic to /visualization_marker
# 3. Add TF display to see obstacle frames
```

## Topics

### Subscribed Topics

- `/scan` (`sensor_msgs/LaserScan`): Laser scan data from robot's LIDAR sensor

### Published Topics

- `/visualization_marker` (`visualization_msgs/Marker`): RViz visualization of obstacles
  - Markers displayed as spheres at obstacle positions
  - Updated at 20 Hz control cycle
  - Color indicates obstacle detection confidence

### TF2 Frames

- **base_link**: Robot base frame
- **obstacle_<id>**: Individual obstacle frame for each detected obstacle
  - Position: X, Y, Z coordinates in world frame
  - Orientation: Rotation of obstacle relative to base frame
  - Updated continuously as new scans arrive

## Configuration

### Detector Parameters

In [obstacle_detector.cpp](src/tf2_obstacle_detector/obstacle_detector.cpp):

- **Laser Scan Subscription**: Topic name `/scan`, QoS policy `SensorDataQoS`
- **Frame ID**: Publishes obstacles relative to scanner frame
- **Update Rate**: Processes each incoming laser scan (typically 10-40 Hz)

### Monitor Parameters

In [obstacle_monitor.cpp](src/tf2_obstacle_detector/obstacle_monitor.cpp):

- **Control Cycle**: 50ms (20 Hz visualization update)
- **Marker Scale**: Configurable sphere size for visualization
- **Color**: Visualization marker color scheme
- **TF Query Timeout**: Configurable timeout for TF lookups

### Tuning Tips

- Adjust marker scale to better visualize obstacle sizes
- Modify control cycle frequency for faster/slower visualization
- Configure TF query timeout based on network latency
- Adjust laser scan filtering thresholds if needed

## Usage Example

### Basic Integration

```cpp
// In your navigation node:
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

tf2_ros::Buffer tf_buffer;
tf2_ros::TransformListener tf_listener(tf_buffer);

// Query obstacle position:
try {
  auto transform = tf_buffer.lookupTransform(
    "base_link", "obstacle_0", tf2::TimePointZero);
  
  double x = transform.transform.translation.x;
  double y = transform.transform.translation.y;
  double z = transform.transform.translation.z;
  
  // Use obstacle position for planning...
} catch (tf2::TransformException &ex) {
  RCLCPP_WARN(this->get_logger(), "%s", ex.what());
}
```

## Node Graph

```
/scan (LaserScan)
    |
    v
[Obstacle Detector]
    |
    +---> TF2 Broadcast
    |
    +---> [Obstacle Monitor]
           |
           v
    /visualization_marker (Marker)
           |
           v
         RViz
```

## Workflow

1. **Detection Phase**:
   - Laser scanner generates scan data
   - Obstacle Detector processes scan ranges
   - Identifies closest/most significant obstacles
   - Broadcasts obstacle positions as TF2 frames

2. **Monitoring Phase**:
   - Obstacle Monitor queries TF2 frames
   - Creates RViz markers for visualization
   - Publishes markers for display
   - Maintains real-time obstacle state

3. **Integration Phase**:
   - Other nodes query obstacle TF frames
   - Use obstacle positions for path planning
   - Perform collision checking
   - Adjust motion commands accordingly

## Troubleshooting

### No Obstacles Detected
- Verify laser scan is being published: `ros2 topic echo /scan`
- Check scan range and angle parameters
- Ensure obstacles are within sensor range
- Verify frame IDs match between detector and scanner

### TF Frames Not Broadcasting
- Check TF tree: `ros2 run tf2_tools view_frames.py`
- Verify Obstacle Detector node is running
- Check for TF broadcast errors in node logs
- Ensure transform listener has sufficient time to update

### No RViz Visualization
- Verify Obstacle Monitor node is running
- Check visualization marker topic: `ros2 topic echo /visualization_marker`
- Add Marker display in RViz and set correct topic
- Set correct fixed frame in RViz (usually `base_link` or `map`)

### Slow Updates
- Reduce control cycle time for faster updates
- Check CPU load and network latency
- Increase laser scan update frequency if supported
- Verify TF buffer is properly initialized

## Performance

- **Detection Update Rate**: ~10-40 Hz (depends on laser scanner)
- **Visualization Update Rate**: 20 Hz (50ms control cycle)
- **Memory Usage**: Minimal, scales with number of obstacles
- **CPU Load**: Low single-core usage
- **Latency**: < 50ms end-to-end

## Integration with Other Packages

This package works well with:

- **vff_avoidance**: Use TF obstacles for vector field force computation
- **fsm_bumpgo_cpp**: Query obstacles for bump-and-go behavior modification
- **Navigation2**: Integrate with Nav2 costmaps and planners
- **cv_object_tracker**: Combine with vision-based detection

## License

Apache License 2.0 - See LICENSE file for details

## Maintainer

- **Email**: pritampaulwork7@gmail.com
- **Author**: ubuntu
