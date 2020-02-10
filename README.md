# Pathfinder ROS

### 1. Overview

This ROS package allows generation of motion-profiled spline path trajectories, supporting constraints on velocity, acceleration, and jerk (time derivative of acceleration). 

For spline interpolation, this package uses a build of the [Pathfinder library](https://github.com/JacisNonsense/Pathfinder) and creates ROS bindings and utility scripts for usage with ROS. 

This package publishes data in various message formats including [nav_msgs/Path](http://docs.ros.org/api/nav_msgs/html/msg/Path.html), [geometry_msgs/PoseArray](http://docs.ros.org/api/geometry_msgs/html/msg/PoseArray.html), and the custom trajectory datatype [pathfinder_ros/Path](msg/Path.msg)

### 2. ROS Nodes

#### 2.1 `pathfinder_node`

`pathfinder_node` is the main node for running Pathfinder ROS. It generates and publishes a new path every time it receives updated waypoint information.

##### 2.1.1 Published Topics

`/pathfinder_ros/path` ([nav_msgs/Path](http://docs.ros.org/api/nav_msgs/html/msg/Path.html))
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Publishes a path object representing the spline path. Primarily for visualization and debugging.

`/pathfinder_ros/segments` ([geometry_msgs/PoseArray](http://docs.ros.org/api/geometry_msgs/html/msg/PoseArray.html))
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Publishes a pose array representing the segments of the spline path (position and heading for each point)

`/pathfinder_ros/path_references` ([pathfinder_ros/Path](msg/Path.msg))
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Publishes a trajectory containing the segments of the path along with followable references for each point (coordinates, position and it's time derivatives, heading, and angular velocity).

##### 2.1.2 Subscribed Topics

`/pathfinder_ros/waypoints` ([geometry_msgs/PoseArray](http://docs.ros.org/api/geometry_msgs/html/msg/PoseArray.html))
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Waypoints (knot points) for the spline path. Note that having too many knot points or having heavily varying direction between the knot points may create a very high-curvature path with high change in curvature, which may be difficult for a robot to follow smoothly. Additionally, in some cases the spline interpolation can create a path that strays extremely far from the waypoints if they are not aligned well.

##### 2.1.3 Parameters

`sample_count` (`int`, default: 10000)
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Number of samples to use for the trajectory generator. Higher numbers will create a more precise trajectory at the expense of higher computational time.

`time_step` (`float`, default: 0.05)
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Length of time, in seconds, for each segment in the trajectory. A smaller value will create a more precise trajectory at the cost of higher computational time. This value should be greater than the minimum control loop time for the robot to prevent the trajectory-following loop from getting behind on the path.

`max_velocity` (`float`, default: 0.8)
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Maximum robot velocity to use in the trajectory, in meters per second. **Warning:** This applies only for the generated trajectory, the trajectory-following code may cause the robot to drive at higher velocities if it does not have its own constraints on velocity.

`max_acceleration` (`float`, default: 100000.0)
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Maximum robot acceleration to use in the trajectory, in meters per second squared. **Warning:** This applies only for the generated trajectory, the trajectory-following code may cause the robot to drive with higher acceleration if it does not have its own constraints on acceleration.

`max_jerk` (`float`, default: 0.8)
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Maximum robot jerk (time derivative of acceleration) to use in the trajectory, in meters per second cubed. **Warning:** This applies only for the generated trajectory, the trajectory-following code may cause the robot to drive with higher jerk if it does not have its own constraints on jerk.

#### 2.2 `simple_path_to_nav_goal.py`

`simple_path_to_nav_goal.py` is a simple node for publishing a path to be used by `pathfinder_node` with only 2 waypoints, one being the robot's current position and the other being a goal point (which can be selected through RViz or through the `move_base` goal topic)

##### 2.2.1 Published Topics

`/pathfinder_ros/waypoints` ([geometry_msgs/PoseArray](http://docs.ros.org/api/geometry_msgs/html/msg/PoseArray.html))
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Waypoints for the spline path (to be used by `pathfinder_node`). This will include the robot's current pose as well as the goal pose.

##### 2.2.2 Subscribed Topics

`/move_base_simple/goal` ([geometry_msgs/PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html))
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Goal pose for the robot. This can be sent using RViz "Set 2D Nav Goal" tool or manually through a node or `rostopic pub`. This is used as the second waypoint for the generated waypoints.


