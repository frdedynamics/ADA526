---
title: Preparing Your PC
layout: default
parent: How-to Guides
nav_order: 1
---

# Preparing Your PC
On your Windows PC:
- Create a GitHub account if you don't have one. [Apply for GitHub Global Campus](https://docs.github.com/en/education/explore-the-benefits-of-teaching-and-learning-with-github-education/github-global-campus-for-students/apply-to-github-global-campus-as-a-student) to access GitHub Pro features such as Copilot and more.
- Install [Fusion 360]
    - Create an Autodesk account with your HVL email and [verify your student status](https://www.autodesk.com/support/technical/article/caas/sfdcarticles/sfdcarticles/How-to-verify-your-student-eligibility.html).
- Install [Bambu Studio]
- Install [VMWare Player]
    - Download our Ubuntu 20.04 [virtual machine] for VMWare Player with pre-installed VSCode, ROS2, Robotics toolbox, Dynamixel Wizard and Dynamixel SDK.
    - Unzip the virtual machine to a folder of your choice.
    - In the VMWare Player, open the virtual machine `ctrl+O` by selecting the `.vmx` file in the folder you unzipped the virtual machine to.
    - Start the virtual machine by clicking `Play virtual machine`.

## On the Virtual Machine
The user password on the VM is `student`.  Here are some things you should do on the VM to get started:

### Configure Git and GitHub
- Set your Git credentials by opening a terminal and typing  
```bash
git config --global user.name "Your user name"
git config --global user.email "Your email address"
```  
with your user name and **email address you use on github.com**.
- Open VSCode (pre-installed) and install the [GitHub Pull Requests and Issues](vscode:extension/GitHub.vscode-pull-request-github) extension.
    - A new GitHub icon appears in the left sidebar. Click it and sign in with your GitHub account.
    
### Install adatools
- Install the [adatools] Python package by following the instructions in the ReadMe on GitHub.

### RealSense Camera
The RealSense driver itself is already installed on the VM, but we noticed that some things are still missing:
- Install the kernel drivers package  
```bash
    sudo mkdir -p /etc/apt/keyrings
    curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null

    sudo apt-get install apt-transport-https

    echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
    sudo tee /etc/apt/sources.list.d/librealsense.list
    sudo apt-get update
```


```sudo apt-get install librealsense2-dkms```
- Install the [vision_opencv package](https://github.com/ros-perception/vision_opencv/tree/foxy)
```bash
    cd ros2_ws/src
    git clone https://github.com/ros-perception/vision_opencv.git -b foxy
    cd ..
    colcon build --symlink-install
```
- Install the [machine vision toolbox](https://github.com/petercorke/machinevision-toolbox-python) for Python from Peter Corke  
```pip install machinevision-toolbox-python```
- For all plotting functions of the vision toolbox to work, you most likely need to upgrade ```matplotlib```  
```pip install --upgrade matplotlib```



[Fusion 360]: https://www.autodesk.com/education/edu-software/overview?sorting=featured&filters=individual#card-f360
[Bambu Studio]: https://bambulab.com/en/download/studio
[VMWare Player]: https://customerconnect.vmware.com/en/downloads/details?downloadGroup=WKST-PLAYER-1625&productId=1039&rPId=98562
[virtual machine]: https://drive.google.com/file/d/15QU57vWVVieqcQ1c6Yy_SgfXyAmGCMJW/view?usp=sharing
[adatools]: https://github.com/frdedynamics/adatools
