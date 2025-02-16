# Turtlesim Exercise - Learning ROS

## Overview
This repository documents my experience with learning the Robot Operating System (ROS) by working with **Turtlesim**, a simple ROS-based simulation tool. The goal of this exercise was to understand ROS concepts, such as working with nodes, topics, and message passing, while implementing a movement pattern inspired by real-world search techniques.

## Implementation
### 1. **Setting Up the Environment**
One of the key advantages of this project was that no manual installation of software was required. The repository is configured with a **Docker container**, which, when built, installs all the necessary dependencies automatically. This allowed me to simply clone the repository, build the container, and start working immediately.

### 2. **Controlling the Turtle Using Python**
I used **Python** within ROS to programmatically control the turtle in Turtlesim. This involved publishing velocity commands to the appropriate ROS topics to move the turtle in a structured manner.

### 3. **Navy Sector Search Pattern**
For the movement logic, I implemented a **Navy sector search pattern**, which is commonly used in maritime search and rescue operations. The turtle starts at a randomly chosen location and moves outward in expanding sectors, rotating 120 degrees after each pass to cover a circular search area systematically.

### 4. **Recording Data Using ROSBag**
To analyze and replay the turtle’s movement, I recorded the motion data using **ROSBag**, a ROS tool for logging and replaying message data. This allows for post-processing and visualization of the executed search pattern.

## Files in This Repository
- **Python Script (`scripts/turtlesim_sector_search.py`)**: Controls the turtle's movement following the sector search pattern.
- **ROSBag File (`turtlesim_sector_search.bag`)**: Contains recorded movement data for analysis and playback.

## How to Run the Code
### Prerequisites
Since this repository is configured with Docker, no manual installation of ROS or other dependencies is necessary. Simply build and run the Docker container.

### Running the Docker Container
```bash
# Build the Docker image
.docker/build_image.sh

# Run the container
.docker/run_user.sh
```

Once inside the container, source the workspace:
```bash
source devel/setup.bash
```

### Running the Python Script
```bash
roscore &
rosrun turtlesim turtlesim_node &
rosrun my_package turtlesim_sector_search.py
```

### Recording and Playing Back the ROSBag
To record:
```bash
rosbag record -O turtlesim_sector_search.bag /turtle1/pose /turtle1/cmd_vel
```
To replay:
```bash
rosbag play turtlesim_sector_search.bag
```

## Conclusion
Through this exercise, I gained hands-on experience in:
- Using **ROS nodes and topics** to control a simulation.
- Implementing **Python-based movement control** in ROS.
- Applying **real-world movement patterns** for robotic navigation.
- Capturing and analyzing data with **ROSBag**.
- Using **Docker** to set up and run ROS environments efficiently.

This project provided valuable insight into robotic motion planning and the fundamentals of ROS development.

## Credits
Special thanks to my lecturer **[Huanyu Li](https://www.linkedin.com/in/huanyu-li-457590268/)** and student assistant **[Shu Xiao](https://www.linkedin.com/in/shu-xiao-a74342133/)** for their guidance and support throughout this project.
ChatGPT was used to assit me with setting up the code correctly in python in order to execute the sector search pattern as I designed it.


