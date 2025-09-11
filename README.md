# 1 Main Content
This study addresses the issues of local optima and excessive computational overhead in high-resolution maps encountered in robotic path planning. We propose an improved method that, without altering the core path planning algorithm, introduces a preprocessing module and a postprocessing module to reduce computational load and enhance path quality, respectively. Path validation was conducted in Gazebo, demonstrating that the added preprocessing and postprocessing modules are compatible with nearly all path planning algorithms. Our code also supports modular adaptation for commonly used path planning algorithms. For different cost map resolutions and planning algorithms, flexible interfaces have been implemented in the launch files, allowing users to make adjustments simply by modifying the launch file—without needing to alter the original algorithms or robot execution files to adjust cost map resolution scaling.
# 2 Approach for Improvement
The preprocessing module ensures the safety of the path planning algorithm by inflating the obstacle map, avoiding the complexity of traditional adaptive distance-maintenance methods and improving computational speed. The postprocessing module, based on keypoint extraction, reduces the numerous path points to a set of essential key points, thereby mitigating issues with local optimal paths.
# 3 Operating Environment
This study was conducted on a Linux system running Ubuntu 20.04. Please ensure your environment matches Ubuntu 20.04. The compatibility of this source code with Ubuntu 18.04 has not been tested; use on Ubuntu 18.04 carries potential risks.
# 4 Environment Configuration
## &nbsp;&nbsp;&nbsp;&nbsp;(1) System Installation
&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Before replicating this source code, ensure that the ROS (Robot Operating System) is installed on your Ubuntu system. If ROS is not installed, you can follow the steps below as a reference:
```bash
sudo apt update
sudo apt upgrade -y
sudo apt install curl gnupg2 lsb-release -y
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-desktop-full -y
```
## &nbsp;&nbsp;&nbsp;&nbsp;(2) Download and Extract the Project
```bash
sudo apt update
sudo apt install git -y
git clone https://github.com/Aguang-NCWU/morphologicalandKeypoints.git
cd morphologicalandKeypoints
```
## &nbsp;&nbsp;&nbsp;&nbsp;(3) Install Required Packages
```bash
sudo apt install ros-noetic-gmapping
sudo apt install ros-noetic-map-server
sudo apt install ros-noetic-navigation
```
## &nbsp;&nbsp;&nbsp;&nbsp;(4) Launch the Project
```bash
catkin_make
```
If compilation is successful, you will see an output as shown in the figure. If it fails, it may be due to missing packages or an incomplete ROS installation. Follow the prompts to install any missing packages and recompile until successful.
<div align="center">
  <img src="https://github.com/user-attachments/assets/29db39cb-8fab-4f1a-8782-f5d84cfdc8e7" width="554" height="376" alt="image" />
</div>

```bash
cd morphologicalandKeypoints
cd morphologicalandKeypoints
```







