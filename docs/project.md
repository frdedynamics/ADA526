---
title: Project
layout: default
nav_order: 2
---

# Project
The project is central to ADA526. In the project you will perhaps learn the most and have the most fun!
In the project you will design and build a robot arm from scratch, and implement basic control algorithms to control the arm based on sensor inputs. Both parts, making the robot and controlling it are important.
Your robots will take part in a competition at the end of the semester, where the tasks require both good design and good control.

## Competition
At the end of semester, we want to organize a showcase and competition. Each group gets the chance to present their robot and then competes in three challenges.
The challenges focus on different qualities of your robot. You will collect points in each challenge, and the total sum of points collected will determine the winner. There is no minimum number of points you need to reach in each challenges - it is up to you if you want to be strategic and specialize your robot design towards a subset of the challenges, or if you want to build an all-round robot that can perform well in all challenges. 

### Challenge 1: Precision and Accuracy
This challenge is about testing the precision and accuracy of your robot. We will provide fixtures for replaceable paper targets that fit into the mounting holes on the base plate. We will also provide a pen holder to be mounted to the end effector of your robot. The pen holder has an attachment for small payloads. The goal is to make five dots with the pen at the center of the target, while small payloads are added to the end effector in-between the trials. The distribution of the dots around the center of the target (distance of mean to center, and standard deviation) will be used to calculate the score. We may define a (rather generous) time limit per group. This challenge is not about speed, but we want to prevent tinkering during the trials.
 - In advance: Each group must define a home pose for their robot which can't be changed after the target coordinates are announced.
 - The target coordinates and the payloads are announced. The order of the payloads for each trial is randomized.
 - The groups perform the challenge one after another. 5 trials per group.
 - Each trial starts from and ends at the defined home pose.
 - The robot must neither be touched nor manually controlled during the trials.
 - The score is calculated based on distance of mean to center and standard deviation. If the marking is not just a dot, the point of the parking with the greatest distance to the center is used for the calculation.

![Accuracy vs. precision](../assets/images/accuracy-vs-precision.jpg)


### Challenge 2: Vision, Control and Speed
In this challenge we test your robot's capabilities of vision-based control and dynamic movements. We will provide two targets that can easily be tracked by a camera (e.g. a red ball and a green ball) which fit into the mounting holes in the base plate, and an end effector with a tip that must touch the targets. The goal is to identify the location of the targets with the camera, to then move the tip of the end effector from one target to the other as fast as possible. The time between the two targets is measured and used to calculate a score. Three different locations of the targets are used, and the order of the targets is randomized. The coordinates will not be announced in advance -- you have to use the camera to identify their location.
 - In advance: Each group must define a home pose for their robot which can't be changed after the first targets are placed.
 - The groups perform the challenge one after another. One trial for each of the three target locations per group.
 - The targets are placed on the base plate. The order of the targets is randomized.
 - Each trial starts from the defined home pose.
 - The tip on the end effector must reach the first target. Time is taken. The tip must then reach the second target. Time is taken. The time between the two targets is used to calculate the score.
 - The robot must neither be touched nor manually controlled during the trials. 


### Challenge 3: Strength
This challenges tests the physical strength of your robot. We will provide an end effector to which increasing payloads can be attached. The goal is to lift the payload at a distance of 45cm from the robot's base above a height of 15cm and hold it there for 5 seconds. The groups can decide how much weight they want to try to lift. Each group can freeze the payload they lifted successfully three times. At the end the highest frozen payload counts, and the group with the highest frozen payloads wins.
A lift counts as failed if the payload is not lifted and held above the required height for the required time, if a motor overloads, if any part of the robot except its base touches the base plate or if the robot is touched by hand. This challenge has no requirements regarding the control of the robot. You can use any control strategy you want, including manual control/teleoperation. 
 - Each group announces a starting payload $$\geq 0$$g in advance.
 - One group is picked at random to start.
 - The group tries to lift their announced payload. If they succeed, they can decide to freeze the payload, or they can risk it and announce a higher payload without freezing, saving a freeze for later. Repeat until the group has used up all three freezes or fails to lift the announced payload.
 - The next group is picked at random and the process repeats.
 - The group with the highest frozen payload wins.


## Base Plate
Each group gets a standardized base plate for mounting the robot on. The mounting holes for the robot are in a 150mm square pattern, M6 screws will be inserted from the bottom of the plate. The plate will also be used as the arena for the competition at the end of the semester. Therefore, it is important that the robot can reach all the corners of the plate.
[Download Base Plate](https://a360.co/3r8Vwxc){: .btn .btn-blue}
<iframe src="https://myhvl14.autodesk360.com/shares/public/SH512d4QTec90decfa6e5fab6acd7b1201ea?mode=embed" width="640" height="480" allowfullscreen="true" webkitallowfullscreen="true" mozallowfullscreen="true"  frameborder="0"></iframe>


## Kinematic Structure
The kinematic structure of the robot is pre-defined. This means, the number of joints and the relative orientations of the joint axes are fixed, but you have to modify the link lengths and offsets such that your robot's workspace more or less covers the base plate.
You find more detailed information about kinematic structures in the [How-to Guides: Kinematic Structure](../docs/how-to guides/kinematic_structure.html).

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
In addition, we use [Intel RealSense D435i] depth cameras in the project. These cameras can record depth and RGB images, and have an integrated IMU. The camera is connected to the PC via USB 3.0.
For communication with the camera, we use the ROS2 wrapper of the [Intel RealSense SDK 2.0].

![d435i](https://www.intelrealsense.com/wp-content/uploads/2020/05/depth-camera-d435_details.jpg)

Example of a point cloud with overlayed RGB colors recorded with a RealSense camera in ROS:
![point_cloud](https://user-images.githubusercontent.com/17433152/35396613-ddcb1d6c-01f5-11e8-8887-4debf178d0cc.gif)


[Intel RealSense D435i]: https://www.intelrealsense.com/depth-camera-d435i/
[Intel RealSense SDK 2.0]: https://dev.intelrealsense.com/docs/ros-wrapper


## Software
We use the [Robotics Toolbox for Python] by Peter Corke. The toolbox is a collection of Python modules that implement functions for robot kinematics, dynamics, and trajectory generation.
We will use the toolbox in the design phase to asses the kinematic structure (link lengths etc.) of the robot, and when controlling the robot to implement the inverse kinematics and trajectory generation.
For the underlying math, to represent, plot and manipulate position and orientation of objects, the toolbox makes use of the [Spatial Math] package.
To simplify some of the rich functionality of the robotics toolbox, we provide [adatools], a collection of convenience functions and examples.
For control, we will integrate the toolbox with [ROS2 Foxy Fitzroy] by importing it in our ROS nodes. 
Camera data will be streamed into the ROS network and used to generate commands for the motors.

[Robotics Toolbox for Python]:https://petercorke.github.io/robotics-toolbox-python/index.html
[Spatial Math]:https://bdaiinstitute.github.io/spatialmath-python/index.html
[ROS2 Foxy Fitzroy]:https://docs.ros.org/en/foxy/Tutorials.html
[adatools]:https://github.com/frdedynamics/adatools
<!--Mention robot_from_dh ? -->

## System Architecture
The central hub for data processing is your own PC running a virtual machine with Ubuntu 20.04. Motors and cameras are connected to your PC, or rather the VM, by USB. Sensor inputs and motor commands as well as computer vision data are exchanged via the ROS2 middleware. 
Just Fusion 360 for CAD and Bambu Studio for preparing 3D-prints are running on your host system.

![System Architecture](../assets/images/system_architecture.png)