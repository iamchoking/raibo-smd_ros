# raibo-smd_ros
Launch + dependency based implementation for RAIBO's sensor module's ROS2 base stack
***
## Overview
Brief overview and progress of of RAIBO's **S**ensor **M**odule **D**evelopment (*raibo-smd*) is outlined [here](https://docs.google.com/presentation/d/1AeIsxL6ZXaKZeVFKT8nhwfjp_lDAwOdkxUtU9UpYwn8/edit?usp=sharing)

***
## Installation
This package is tested on the **ROS2 foxy** distribution ([link](https://docs.ros.org/en/foxy/Installation.html))

## 1. Install Dependencies
</br>

### 1.0. ```ros2-foxy```
You need a base underlay workspace to install this package from source (i.e. "```dev_ws``` folder")
Installation Instructions for ROS2 [here](https://docs.ros.org/en/foxy/Installation.html)
Instrtuctions for creating your ```dev_ws``` is [here](https://docs.ros.org/en/foxy/Tutorials/Workspace/Creating-A-Workspace.html)
</br></br>
### 1.1. ```librealsense2``` ([link](https://github.com/IntelRealSense/librealsense/blob/development/doc/distribution_linux.md))
*Note: The release does not have to be v2.41.0 specifically, but make sure that the ros-wrapper is compatible with it.*

You can use any method of installation.

Make sure that the installation can detect and retrieve data from both cameras:
```
rs-enumerate-devices -s
```
The above command must output each of the connected devices on each line.
```
realsense-viewer
```
The above command must start a graphical interface, which connects to one of the Realsense devices.
</br></br>
### 1.2. ```realsense-ros``` ([link](https://github.com/intel/ros2_intel_realsense)) ([link](https://github.com/IntelRealSense/realsense-ros/tree/ros2))
*Note: **Binary** installation is recommended to keep development workspace minimal.*

**IMPORTANT: The link for installation shows instructions to install for the ros distribution *dashing*. Change all the ```dashing``` statements to either ```foxy``` or ```${ROS_DISTRO}```.**

So, the binary installation is as follows:

- Install ROS2 Dependancies:
```
sudo apt-get install ros-${ROS_DISTRO}-cv-bridge ros-${ROS_DISTRO}-librealsense2 ros-${ROS_DISTRO}-message-filters ros-${ROS_DISTRO}-image-transport
```
- Install non-ROS Dependancies:
```
sudo apt-get install -y libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev
sudo apt-get install -y libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev
```
- Install Package

```
sudo apt-get install ros-${ROS_DISTRO}-realsense2-camera-msgs ros-${ROS_DISTRO}-realsense2-camera
```
**The validity of the commands above are unstable. If the below commands do not work, use ```apt-cache search realsense``` to search for relevant packages. As a last resort, install from source.**
- Check Installation
```
ros2 pkg list | grep realsense
```
the above command must include:
```
realsense2_camera
realsense2_camera_msgs
```
</br></br>
### 1.3. ```velodyne``` (ROS2 Wrapper) ([link](http://wiki.ros.org/velodyne/Tutorials/Getting%20Started%20with%20the%20Velodyne%20VLP16))

For the *ip and route configuration*, use the following IP address (factory default).
```
 192.168.  1.201
         ^^^ ^^^
(192.168. XX. YY)
```
If problem about the IP address persists, check out the discussion [here](https://answers.ros.org/question/244445/having-problems-with-velodyne-vlp-16-and-ros/)

Additionally, since RAIBO uses a usb-c hub for ethernet connection, the adapter alias will be something different than [```eth0```].

Use ```sudo ifconfig -a``` command to see possible internet adapters, and choose the alias with the connected MAC that matches that of the LiDAR. (i.e.: ```enx00e04c680015```)
</br></br>
## 2. Install From Source
- Move to your underlay source and clone this repo.
```
cd ~/dev_ws/src
git clone https://github.com/iamchoking/raibo-smd_ros.git
```
- Build Package
```
cd ~/dev_ws
source /opt/ros/${ROS_DISTRO}/setup.bash
colcon build --symlink-install --base-paths src/raibo-smd_ros
```

***
***
## Usage
Simple launch commands can manipulate the ```realsense_ros``` and ```velodyne``` nodes, publish pre-determined ```tf```'s and visualise the pointCloud output using [```rviz2```](https://github.com/ros2/rviz)
- Launch head module only
```
ros2 launch raibo-smd_ros head_launch.py
```
- Launch lidar module only
```
ros2 launch raibo-smd_ros lidar_launch.py
```
- Launch all modules
```
ros2 launch raibo-smd_ros full_launch.py
```
