# ROS2 interface for Cortex MoCap system
Repository of ROS2 interface for Cortex motion capture system.

Github CI | SonarCloud
------------| ---------------
[![Build Status](https://github.com/kroshu/ros2_cortex/workflows/CI/badge.svg?branch=master)](https://github.com/kroshu/ros2_cortex/actions) | [![Quality Gate Status](https://sonarcloud.io/api/project_badges/measure?project=kroshu_ros2_cortex&metric=alert_status)](https://sonarcloud.io/dashboard?id=kroshu_ros2_cortex)

The repository contains a mock version of the Cortex SDK. The CortexClient class provides a C++ wrapper, which can function with both the SDK and the mock, depending on what the user compiles. The CortexClientNode class creates a ROS2 node which uses the Cortex system through the wrapper. Deriving from this class and overriding the dataHandlerFunc_ funtion, the user is able to create its own ROS2 node with the custom frame of data handler function. The base class provides parameters and services which cover the requests provided by the SDK.

One example is SimpleFodPrinter, which simply prints the frame id and the number of unidentified markers of the actual frame of data. Another example is MarkerPublisher, which publishes visualization_msgs/MarkerArray ROS2 messages, converting every frame of data to marker data. The MotionTracker node subscribes to the topic published py MarkerPublisher and controls a robot accordingly. Both the CortexClientNode classes and the MotionTracker class is derived from the ROS2BaseNode class implemented in https://github.com/kroshu/kroshu_ros2_core.

