---
title: Project
layout: default
nav_order: 2
---

# Project
The project is central to ADA526. In the project you will perhaps learn the most and have the most fun!


## Kinematic Structure
You can choose one of three pre-defined kinematic structures. The relative orientations of the joints are fixed, but you should modify the link lengths and offsets such that your robot has sufficient reach.

![config1](/assets/images/config.png) 

![config_1_param](../../assets/images/config-1-params.png)

*Kinematic Configuration 1: In the second image you can see the parameters you can change in the design process: the link lengths (green) and the link offsets (orange). The joint angles (blue) are used to control the robot during operation.*

### Denavit-Hartenbeg Parameters
{: .d-inline-block }
Optional
{: .label .label-blue } 

Denavit-Hartenberg (DH) parameters are what's behind the formal definition of the kinematic structure. For this project, we exposed only some of the parameters (link lengths and offsets) to keep things simple. Some parameters we kept fixed, such as the relative orientations of the joints.

In general, DH parameters are a set of four parameters for each joint that define the relative orientation and position of the joint axes. The DH parameters are defined as follows: 
<iframe width="560" height="315" src="https://www.youtube.com/embed/rA9tm0gTln8" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>


## Actuators
We use Dynamixel X-series servo motors in the project.

![x_series](https://emanual.robotis.com/assets/images/dxl/x/x_series_product.png)

 Each group gets a kit gets the following kit:

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
We use the [Intel RealSense D435i] depth camera in the project. The camera can record depth and RGB images, and has an IMU. The camera is connected to the PC via USB 3.0.
For communication with the camera, we use the ROS2 wrapper of the [Intel RealSense SDK 2.0].

![d435i](https://www.intelrealsense.com/wp-content/uploads/2020/05/depth-camera-d435_details.jpg)


[Intel RealSense D435i]: https://www.intelrealsense.com/depth-camera-d435i/
[Intel RealSense SDK 2.0]: https://dev.intelrealsense.com/docs/ros-wrapper


## Software
We use the [Robotics Toolbox for Python] in the project. The toolbox is a collection of Python modules that implement functions for robot kinematics, dynamics, and trajectory generation. The toolbox is based on the Robotics Toolbox for MATLAB by Peter Corke.
We will use the toolbox in the design phase to asses the kinematic structure (link lengths etc.) of the robot, and when controlling the robot to implement the inverse kinematics and trajectory generation.
For control, we will integrate the toolbox with [ROS2 Foxy Fitzroy] by importing it in our ROS nodes.

[Robotics Toolbox for Python]:https://petercorke.github.io/robotics-toolbox-python/index.html
[ROS2 Foxy Fitzroy]:https://docs.ros.org/en/foxy/Tutorials.html