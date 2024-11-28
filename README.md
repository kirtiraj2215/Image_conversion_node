ROS 2 Image Conversion Package

Overview

This package provides a ROS 2 node (image_conversion_node) that subscribes to an image topic (e.g., from the usb_cam package), processes the image (converting between grayscale and color modes), and republishes it to another topic. The node allows switching between grayscale and color modes via a ROS 2 service.

Features

	•	Subscribes to a camera image topic.
	•	Publishes the processed image to a separate topic.
	•	Provides a service to toggle between grayscale and color processing modes.

Requirements

	•	ROS 2 (Tested on Jazzy or compatible versions)
	•	OpenCV (for image processing)
	•	usb_cam package for camera input

Installation

	1.	Clone the repository into your ROS 2 workspace:

cd ~/ros2_ws/src
git clone <your-repo-link>


	2.	Build the workspace:

cd ~/ros2_ws
colcon build
source install/setup.bash


	3.	Verify the installation:

ros2 pkg list | grep image_conversion



Usage

Launch the Nodes

Run the following command to start the image conversion node and usb_cam node:

ros2 launch image_conversion image_conversion_launch.py

This will:
	•	Start the usb_cam node to publish camera images.
	•	Start the image_conversion_node to process the images.

Topics

	•	Input Topic: /usb_cam/image_raw (default, configurable via parameters)
	•	Output Topic: /image_converted (default, configurable via parameters)

Service

The image_conversion_node provides a service to toggle the mode:
	•	Service Name: set_mode
	•	Request: A boolean (true for grayscale mode, false for color mode)

Call the service:

Example to switch to grayscale mode:

ros2 service call /set_mode std_srvs/srv/SetBool "{data: true}"

Switch back to color mode:

ros2 service call /set_mode std_srvs/srv/SetBool "{data: false}"

Configuration

Parameters can be set in the launch file (image_conversion_launch.py):
	•	input_topic: Camera input topic (default: /usb_cam/image_raw)
	•	output_topic: Processed image output topic (default: /image_converted)

Example modification in the launch file:

launch_ros.actions.Node(
    package='image_conversion',
    executable='image_conversion_node',
    name='image_conversion_node',
    parameters=[{'input_topic': '/camera/image_raw', 'output_topic': '/image_processed'}]
)

Launch File

The launch file (image_conversion_launch.py) starts both the usb_cam and image_conversion nodes. You can modify parameters as needed.

Structure

image_conversion/
├── CMakeLists.txt
├── package.xml
├── launch/
│   └── image_conversion_launch.py
├── src/
│   ├── image_conversion_node.cpp
├── include/
│   └── image_conversion/
│       └── image_conversion_node.hpp
