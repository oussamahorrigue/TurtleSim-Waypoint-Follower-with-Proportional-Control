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

```bash
git clone https://github.com/your-username/your-repo.git
