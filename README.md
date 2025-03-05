# Trajectory Visualization and Storage for AMR Navigation

This ROS package provides tools for visualizing and storing the trajectory of an Autonomous Mobile Robot (AMR) using the ROS framework. It includes two nodes:
1. **Trajectory Publisher and Saver Node**: Collects and saves the robot's trajectory.
2. **Trajectory Reader and Publisher Node**: Reads and visualizes the saved trajectory.

## Features
- **Real-time trajectory visualization** in RViz.
- **Save trajectory data** to a file in JSON format.
- **Load and visualize saved trajectories** in RViz.
- **Custom ROS service** for saving trajectory data.

## Prerequisites
- **ROS Noetic** (or your ROS distribution).
- **jsoncpp** library for JSON parsing.
- **RViz** for visualization.

## Installation
1. Clone the repository:
   ```bash
   git clone https://github.com/your-username/trajectory-visualization-ros.git
   cd trajectory-visualization-ros
   
2. Build the ROS package:


cd trajectory_visualization
catkin_make

Source the workspace:
3. source devel/setup.bash

## Usage
Run the Trajectory Publisher and Saver Node:

rosrun trajectory_visualization trajectory_publisher_saver
Save the Trajectory:
Call the service to save the trajectory:

rosservice call /save_trajectory "{file_name: '/path/to/trajectory.json', duration: 10}"
Run the Trajectory Reader and Publisher Node:

rosrun trajectory_visualization trajectory_reader_publisher /path/to/trajectory.json
Visualize in RViz:

Open RViz:

rosrun rviz rviz
Add a MarkerArray display and set the Marker Topic to /saved_trajectory_markers.

Set the Fixed Frame to odom (or the frame used in your code).

File Structure

trajectory_visualization/
├── CMakeLists.txt
├── package.xml
├── launch/
├── src/
│   ├── trajectory_publisher_saver.cpp
│   └── trajectory_reader_publisher.cpp
├── srv/
│   └── SaveTrajectory.srv
└── README.md
