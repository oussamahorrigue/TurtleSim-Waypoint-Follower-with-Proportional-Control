# TurtleSim Waypoint Follower with Proportional Control

This repository contains a Python implementation of a waypoint following behavior for the TurtleSim simulator in ROS. The code utilizes a proportional control strategy to guide the turtle to a series of specified waypoints.

## Features

- Waypoint following: The turtle autonomously navigates through a sequence of predefined waypoints.
- Proportional control: The linear and angular velocities of the turtle are adjusted based on the distance and angular errors between the current pose and the target waypoint.
- Smooth trajectory: The turtle moves towards each waypoint while maintaining a smooth trajectory.
- Adjustable stay durations: The turtle stays at each waypoint for a specified duration of time before moving to the next waypoint.

## Dependencies

- ROS (Robot Operating System): The code is designed to work with ROS, specifically the TurtleSim simulator.

## Getting Started

1. Clone the repository:

git clone https://github.com/oussamahorrigue/TurtleSim-Waypoint-Follower-with-Proportional-Control.git

2. Build the ROS workspace:

- cd TurtleSim-Waypoint-Follower-with-Proportional-Control
- catkin_make


3. Run the waypoint follower node:

- source devel/setup.bash
- rosrun my_package waypoint_multiple_stop_last_point.py


## Customization

You can customize the waypoints, stay durations, and control parameters by modifying the `waypoint.py` file. 

- Modify the `target_poses` list in the `__init__` method of the `WaypointFollower` class to define your desired waypoints.
- Adjust the `stay_duration` variable in the `stay_at_waypoint` method to set the desired stay duration at each waypoint.
- Fine-tune the control gains (`linear_vel_gain` and `angular_vel_gain`) in the `move_to_next_waypoint` method to achieve the desired performance.

Refer to the comments in the code for more guidance on making the desired changes.

## Contributions

Contributions to the project are welcome! If you find any issues or have ideas for improvements, please open an issue or submit a pull request.

## License

This project is licensed under the [MIT License](LICENSE).

## Acknowledgments

- The code in this repository is based on the concepts and principles of ROS and proportional control.
- Credits to the TurtleSim simulator provided by ROS for providing the platform for testing and simulation.



