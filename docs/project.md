---
title: Project
layout: default
nav_order: 1
---

# Project
The project is central to ADA526. In the project you will perhaps learn the most and have the most fun!


## Base Plate
Each group gets a standardized base plate for mounting the robot on. The mounting holes for the robot are in a 150mm square pattern, M6 screws will be inserted from the bottom of the plate. The plate will also be used as the arena for the competition at the end of the semester. Therefore, it is important that the robot can reach all the corners of the plate.
<iframe src="https://myhvl14.autodesk360.com/shares/public/SH512d4QTec90decfa6e5fab6acd7b1201ea?mode=embed" width="640" height="480" allowfullscreen="true" webkitallowfullscreen="true" mozallowfullscreen="true"  frameborder="0"></iframe>


## Kinematic Structure
You can choose one of three pre-defined kinematic structures. The relative orientations of the joints are fixed, but you should modify the link lengths and offsets such that your robot has sufficient workspace.
You find more detailed information about the kinematic structures in the [How-to Guides: Kinematic Structure](../docs/how-to guides/kinematic_structure.html).

### Configuration 1
![A Config 1 Robot](../assets/images/config_1.png)

## Actuators
We use Dynamixel X-series servo motors in the project.

![x_series](https://emanual.robotis.com/assets/images/dxl/x/x_series_product.png)

 Each group gets the following kit:

| Qty | Description                               |    
| :-- | :---------------------------------------- | 
| 4   | [Dynamixel XM430-W350-T]                  |  
| 1   | [Dynamixel XM540-W150-T]                  |  
| 1   | [U2D2 TTL to USB communication converter] |  
| 1   | [U2D2 Power Hub]                          |
| 1   | [12V 5A power supply]                     |

[Dynamixel XM430-W350-T]: https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/
[Dynamixel XM540-W150-T]: https://emanual.robotis.com/docs/en/dxl/x/xm540-w150/
[U2D2 TTL to USB communication converter]: https://emanual.robotis.com/docs/en/parts/interface/u2d2/
[U2D2 Power Hub]: https://emanual.robotis.com/docs/en/parts/interface/u2d2_power_hub/
[12V 5A power supply]: https://www.robotshop.com/en/12v-5a-power-supply.html


## Sensors
The Dynamixel motors have built-in encoders that we use to measure the joint angles for control. 
In additon, we use [Intel RealSense D435i] depth cameras in the project. These cameras can record depth and RGB images, and have an IMU. The camera is connected to the PC via USB 3.0.
For communication with the camera, we use the ROS2 wrapper of the [Intel RealSense SDK 2.0].

![d435i](https://www.intelrealsense.com/wp-content/uploads/2020/05/depth-camera-d435_details.jpg)


[Intel RealSense D435i]: https://www.intelrealsense.com/depth-camera-d435i/
[Intel RealSense SDK 2.0]: https://dev.intelrealsense.com/docs/ros-wrapper


## Software
We use the [Robotics Toolbox for Python] by Peter Corke. The toolbox is a collection of Python modules that implement functions for robot kinematics, dynamics, and trajectory generation.
We will use the toolbox in the design phase to asses the kinematic structure (link lengths etc.) of the robot, and when controlling the robot to implement the inverse kinematics and trajectory generation.
For the underlying math, to represent, plot and manipulate position and orientation of objects, the toolbox makes use of the [Spatial Math] package.
For control, we will integrate the toolbox with [ROS2 Foxy Fitzroy] by importing it in our ROS nodes.

[Robotics Toolbox for Python]:https://petercorke.github.io/robotics-toolbox-python/index.html
[Spatial Math]:https://bdaiinstitute.github.io/spatialmath-python/index.html
[ROS2 Foxy Fitzroy]:https://docs.ros.org/en/foxy/Tutorials.html
