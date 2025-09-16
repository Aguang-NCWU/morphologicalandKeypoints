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
source ./devel/setup.bash
roslaunch launch_and_map robotPathPlanningControlWithGui.launch
```
(In the launch file, you can select different maps, resolutions, and algorithms for testing)

The target point is selected as shown in the figure below.
<div align="center">
<img width="554" height="443" alt="image" src="https://github.com/user-attachments/assets/49336a38-922f-4075-a942-83ef6337d567" />
</div>
You can compare the original and improved path planning by switching the path planning algorithm in the robotPathPlanningControlWithGui node of the launch_and_map package.
Algorithms with the origin prefix represent the original version, while those with the improved prefix represent the optimized version.
Below is the path comparison of the A* algorithm before(left) and after(right) improvement.
<p align="center">
  <img src="https://github.com/user-attachments/assets/95f23ea9-1daf-4a25-8588-510c6593fd7b" width="45%" />
  <img src="https://github.com/user-attachments/assets/195704c2-0c34-45fb-b0da-5a2e05a4ad83" width="45%" />
</p>
The robot car’s movement can be observed within the Gazebo simulation environment.
<img width="1850" height="1055" alt="gazebo1" src="https://github.com/user-attachments/assets/1d886eac-147f-4757-b980-bb73f4bed827" />

# 4 Other issues
The following presents the results of multiple experiments I conducted, along with comparative plots of the robot’s performance before and after optimization, drawn based on its execution characteristics.

| Core Algorithm | without optimization module |with preprocessing module | with preprocessing and postprocessing modules|
|-------|-------|-------|-------|
| A* Algorithm | <img width="191" height="182" alt="image" src="https://github.com/user-attachments/assets/28ba38b9-a05e-4d9a-a089-588826534ac3" /> |<img width="191" height="182" alt="image" src="https://github.com/user-attachments/assets/86f58615-322c-4c9e-a510-3c0587ea571a" />|<img width="190" height="182" alt="image" src="https://github.com/user-attachments/assets/abec9270-9db2-4f1c-b155-09ae0d9665aa" />|
|Dijkstra Algorithm | <img width="188" height="182" alt="image" src="https://github.com/user-attachments/assets/274963e6-9945-4703-94f4-6b73eb729a93" /> | <img width="191" height="182" alt="image" src="https://github.com/user-attachments/assets/c27af1d5-3aa6-42b4-9bef-14aea7c2ffbb" /> |<img width="190" height="182" alt="image" src="https://github.com/user-attachments/assets/051bdbf9-7c5f-4864-a573-62419de5e3d1" />|
|Ant Colony Algorithm |<img width="190" height="182" alt="image" src="https://github.com/user-attachments/assets/18505f43-8b88-4a58-bf3e-14b332f6b12b" /> | <img width="190" height="182" alt="image" src="https://github.com/user-attachments/assets/3516104c-0717-4dc6-b530-76ab64be5c2d" />|<img width="194" height="182" alt="image" src="https://github.com/user-attachments/assets/5c2aa587-5e48-483c-b8fa-793ae688bf22" />|
|Bidirectional A* Algorithm | <img width="189" height="182" alt="image" src="https://github.com/user-attachments/assets/ee2bead2-361f-41d0-b62d-b96778420337" />| <img width="190" height="182" alt="image" src="https://github.com/user-attachments/assets/985751c2-670f-455b-962c-f9c679969f7d" />|<img width="190" height="182" alt="image" src="https://github.com/user-attachments/assets/1c6ad070-2991-4a5c-bbf5-204959c3e24a" />|
|JPS Algorithm | <img width="190" height="182" alt="image" src="https://github.com/user-attachments/assets/3bfc7dc8-c317-4ca0-9f0c-5befc6b03e59" />| <img width="189" height="182" alt="image" src="https://github.com/user-attachments/assets/57cdedf0-099e-4bb5-81b5-923b7034256e" /> |<img width="191" height="182" alt="image" src="https://github.com/user-attachments/assets/4666139b-2101-4909-9213-3a9eaa80ca49" />|
|RRT Algorithm |<img width="189" height="182" alt="image" src="https://github.com/user-attachments/assets/29e166a1-a9a8-4109-ba18-f899cba7fd70" />|<img width="192" height="182" alt="image" src="https://github.com/user-attachments/assets/500ccfed-80a1-4180-8202-b1127107dd6e" />|<img width="182" height="182" alt="image" src="https://github.com/user-attachments/assets/d79c79a2-75be-48a0-b5a8-960fded8d67a" />|














Please leave a message if you have any questions

