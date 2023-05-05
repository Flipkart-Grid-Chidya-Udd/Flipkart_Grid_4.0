# Flipkart_Grid_4.0

## Autonomous Drone Delivery with ROS and Intel Realsense
This project is a submission for the Flipkart Grid 4.0 competition, which involved building an autonomous drone for indoor delivery. The drone is able to recognize and pick up packages using an electromagnet, and drop them off at a desired location. This README provides an overview of the project and instructions for using it.

# Hardware and Software Requirements
The following hardware and software are required to run this project:

Pixhawk F450 drone with PX4 firmware
Jetson Nano
Intel Realsense D435i camera
Electromagnet
ROS Melodic
Mavros
AprilTag ROS package
Intel Realsense ROS package
# Installation and Setup
Install ROS Melodic and Mavros on your computer.

Install the Intel Realsense ROS package by following the instructions here.

Clone this repository to your computer.

Connect the Intel Realsense camera to the Jetson Nano and the Jetson Nano to the Pixhawk F450 drone.

Launch the following ROS launch files:

rs_camera.launch: This launches the Intel Realsense camera and publishes the camera image to a ROS topic.
px4.launch: This launches Mavros and connects to the Pixhawk F450 drone.
continues_detection.launch: This launches the AprilTag ROS package for detecting the 4x4 grid of AprilTags on the floor.
# Usage
Once you have installed and set up the necessary hardware and software, you can use the drone for autonomous delivery. Here are the steps to follow:

Power on the drone and wait for it to initialize.

Launch the rs_camera.launch, px4.launch, and continues_detection.launch files.

Place the packages to be delivered within the view of the Intel Realsense camera. Make sure the packages are either marked with an Aruco code or have a distinct color on top.

Launch the fast planning with dynamic octomapping algorithm by running the following command:

Copy code
''' roslaunch dynamic_fast_planner dynamic_fast_planner.launch '''
The drone will begin to plan a path to pick up the package using the shortest path possible.

Once the package has been picked up, the drone will plan a path to the desired delivery location and drop the package using the electromagnet.

# Conclusion
This project demonstrates the use of ROS and Intel Realsense for building an autonomous drone for indoor delivery. The project includes obstacle avoidance and motion planning using dynamic octomapping, localization using AprilTags and the Intel Realsense camera, and package recognition using Aruco codes or color detection. We hope that this README provides a useful guide for others who are interested in building similar projects.
