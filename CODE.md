---
layout: default
title: Code
nav_order: 3
---


<h2> Dependencies </h2>

There are several libraries being used in the software stack such as OpenCV, ROS1, ROS2, Python, C++, and C. Additionally, there are URDF, XML, and JSON files used for configuration. 
Since we are using custom PCBS, this required us to flash our own firmware to integrate into our software stack. 

<h2>Software Framework </h2>
The entirety of the software stack leveraged using the new packages, and inherently distributed aspects, of ROS2. This included building the communication between the robots using name specs, and ensuring that each individual ROS2 node was able to communicate on the same network. This entailed using namespaces to ensure each individual rover would have a different software identifier, while also standardizing performance through automated bash scripts that would launch, irrespective of the platform.

The platforms our software included were, linux x86 environments for running ROS2 as a base station on your computers, and also on Jetpack, an ARM based version of Ubuntu. 

<h2>Bash Scripts</h2>
Using bash scripts, we were able to assign higher level name spaces by carefully providing command-line variables to ROS2 launch file, and were used to launch a series of processes that would handle several other aspects of the software stack.

These bash scripts consisted of: 
1. launching the realsense-ros node, which consists of a ROS wrapper necessary to view and utilize data coming from the Intel D435 and T265
2. Pass in custom parameters to optimize performance on the Jetson Nano, such as decimation filters to increase the speed of our depth image, and increases to D435 frames per second to obtain better Aruco tag detection.  Furthermore,
3. Building the Realsense SDK from source to include and specify correct optimizations for CUDA and leverage the Jetson Nano’s GPU accelerators. 
4. Automate tasks such as sourcing the correct directories, and also installing the correct dependencies necessary, and to even install ROS—if necessary. 
5. Group launch several scripts which set up network permissions, opened USB ports for serial communication, created a roscore network, initializing rosserial, and initializing the ros1/ros2 bridge.
