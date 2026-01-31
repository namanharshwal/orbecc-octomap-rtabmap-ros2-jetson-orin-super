Complete Professional README.md for Orbecc Camera RTAB-Map SLAM
bash
cat > README.md << 'EOF'
# Orbecc Gemini 335 RGB-D SLAM on Jetson Orin Nano

Complete ROS2 Humble workspace for Orbecc Gemini 335 depth camera integration with RTAB-Map SLAM on NVIDIA Jetson Orin Nano Super Developer Kit.

![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)
![Ubuntu 22.04](https://img.shields.io/badge/Ubuntu-22.04-orange)
![Jetson](https://img.shields.io/badge/Jetson-Orin%20Nano-green)

---

## Table of Contents

- [Overview](#overview)
- [Hardware Requirements](#hardware-requirements)
- [Software Requirements](#software-requirements)
- [Installation](#installation)
- [Configuration](#configuration)
- [Usage](#usage)
- [Troubleshooting](#troubleshooting)
- [Advanced Topics](#advanced-topics)
- [Contributing](#contributing)
- [License](#license)

---

## Overview

This workspace provides a complete solution for:
- **RGB-D SLAM** using RTAB-Map (Real-Time Appearance-Based Mapping)
- **Visual Odometry** from depth camera
- **3D Mapping** and **Loop Closure Detection**
- **Point Cloud Generation** and visualization
- Integration with **Orbecc Gemini 335** infrared depth camera

### Key Features

- ✅ Real-time 3D mapping and localization
- ✅ Visual and ICP-based odometry
- ✅ Works in both lit and dark environments (IR-based depth)
- ✅ Loop closure detection for drift correction
- ✅ Occupancy grid generation for navigation
- ✅ Configurable SLAM parameters
- ✅ RViz2 and RTAB-Map visualization

---

## Hardware Requirements

### Required Hardware

| Component | Specification | Notes |
|-----------|--------------|-------|
| **Compute** | NVIDIA Jetson Orin Nano Super Developer Kit (8GB) | JetPack 6.0+ recommended |
| **Camera** | Orbecc Gemini 335 RGB-D Camera | USB 3.0 connection required |
| **Power** | 15W USB-C PD adapter | For Jetson Orin Nano |
| **Storage** | 64GB+ microSD or NVMe SSD | Fast storage recommended |
| **Cable** | USB 3.0 Type-A to Type-C cable | For camera connection |

### Camera Specifications

- **RGB Resolution**: 1920x1080 @ 30fps (configurable to 640x480 @ 15fps)
- **Depth Resolution**: 640x480 @ 30fps
- **Depth Technology**: Infrared stereo vision (850nm)
- **Depth Range**: 0.3m to 5.0m
- **Field of View**: 86° (H) × 57° (V)
- **IMU**: 6-axis IMU integrated
- **Interface**: USB 3.0

---

## Software Requirements

### Operating System

- **Ubuntu**: 22.04 LTS (Jammy Jellyfish)
- **JetPack**: 6.0 or later (for Jetson)
- **Kernel**: 5.15+ recommended

### ROS2 Distribution

- **ROS2 Humble Hawksbill** (LTS - supported until 2027)

### Dependencies

```bash
# Core ROS2 packages
ros-humble-desktop
ros-humble-rtabmap-ros
ros-humble-rtabmap-odom
ros-humble-rtabmap-slam
ros-humble-rtabmap-viz
ros-humble-image-transport
ros-humble-cv-bridge
ros-humble-vision-opencv

# Build tools
python3-colcon-common-extensions
python3-rosdep
Installation
Step 1: Install ROS2 Humble
If ROS2 Humble is not already installed:

bash
# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Setup sources
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble
sudo apt update
sudo apt upgrade -y
sudo apt install ros-humble-desktop -y

# Install development tools
sudo apt install python3-colcon-common-extensions python3-rosdep -y

# Initialize rosdep
sudo rosdep init
rosdep update
Step 2: Install RTAB-Map
bash
# Install RTAB-Map ROS2 packages
sudo apt install -y \
  ros-humble-rtabmap-ros \
  ros-humble-rtabmap-odom \
  ros-humble-rtabmap-slam \
  ros-humble-rtabmap-viz \
  ros-humble-rtabmap-launch

# Install additional dependencies
sudo apt install -y \
  ros-humble-image-transport \
  ros-humble-image-transport-plugins \
  ros-humble-cv-bridge \
  ros-humble-vision-opencv \
  ros-humble-image-pipeline

# Install visualization tools
sudo apt install -y \
  ros-humble-rviz2 \
  ros-humble-rqt \
  ros-humble-rqt-image-view \
  ros-humble-rqt-graph
Step 3: Clone This Repository
bash
# Clone with submodules
git clone --recurse-submodules https://github.com/YOUR_USERNAME/orbecc_ws.git
cd orbecc_ws

# If you already cloned without --recurse-submodules:
git submodule init
git submodule update
Step 4: Install OrbbecSDK Dependencies
bash
cd ~/orbecc_ws/src/OrbbecSDK_ROS2

# Install udev rules for camera access
sudo cp orbbec_camera/scripts/99-obsensor-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger

# Install camera dependencies
sudo apt install -y \
  libgflags-dev \
  libgoogle-glog-dev \
  libeigen3-dev \
  libusb-1.0-0-dev
Step 5: Build Workspace
bash
cd ~/orbecc_ws

# Install workspace dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build workspace
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source workspace
source ~/orbecc_ws/install/setup.bash

# Add to .bashrc for automatic sourcing
echo "source ~/orbecc_ws/install/setup.bash" >> ~/.bashrc
Step 6: Verify Installation
bash
# Check if camera node is available
ros2 pkg list | grep orbbec_camera

# Check RTAB-Map installation
ros2 pkg list | grep rtabmap

# Test camera detection
lsusb | grep Orbbec
# Should show: "Bus XXX Device XXX: ID 2bc5:XXXX Orbbec"
Configuration
Camera Configuration
The workspace includes optimized camera settings in config/ directory:

config/camera_params.yaml
text
# Optimized settings for SLAM
depth_registration: true
color_format: "YUYV"
color_width: 640
color_height: 480
color_fps: 15
depth_width: 640
depth_height: 480
depth_fps: 15
enable_publish_tf: true
tf_publish_rate: 30.0
RTAB-Map Configuration Files
config/rgbd_odom.yaml
Visual odometry configuration for RGB-D SLAM.

config/rtabmap_slam.yaml
RTAB-Map SLAM node configuration with loop closure settings.

config/rgbd_odom_relaxed.yaml
Relaxed parameters for challenging environments (low light, low texture).

Usage
Quick Start: Visual SLAM (Recommended)
Terminal 1: Launch Camera

bash
source ~/orbecc_ws/install/setup.bash

ros2 launch orbbec_camera gemini_330_series.launch.py \
  depth_registration:=true \
  color_format:=YUYV \
  color_width:=640 \
  color_height:=480 \
  color_fps:=15 \
  depth_width:=640 \
  depth_height:=480 \
  depth_fps:=15 \
  enable_publish_tf:=true \
  base_frame_id:=camera_link
Terminal 2: Launch RTAB-Map SLAM

bash
source /opt/ros/humble/setup.bash

ros2 launch rtabmap_launch rtabmap.launch.py \
  rtabmap_args:="--delete_db_on_start" \
  rgb_topic:=/camera/color/image_raw \
  depth_topic:=/camera/depth/image_raw \
  camera_info_topic:=/camera/color/camera_info \
  frame_id:=camera_link \
  approx_sync:=true \
  queue_size:=50 \
  visual_odometry:=true \
  icp_odometry:=false \
  rtabmap_viz:=true \
  rviz:=false
Alternative: ICP Odometry (Dark Environments)
For environments with poor lighting or low texture:

bash
source /opt/ros/humble/setup.bash

ros2 launch rtabmap_launch rtabmap.launch.py \
  rtabmap_args:="--delete_db_on_start --Reg/Strategy 1" \
  rgb_topic:=/camera/color/image_raw \
  depth_topic:=/camera/depth/image_raw \
  camera_info_topic:=/camera/color/camera_info \
  frame_id:=camera_link \
  approx_sync:=true \
  queue_size:=100 \
  visual_odometry:=false \
  icp_odometry:=true \
  rtabmap_viz:=true
Key Difference: ICP uses only depth/IR data, works in complete darkness.

Verification Commands
Check System Status
bash
# 1. Check all active topics
ros2 topic list | grep -E "camera|rtabmap"

# 2. Check camera data streams
ros2 topic hz /camera/color/image_raw    # Should show ~15 Hz
ros2 topic hz /camera/depth/image_raw    # Should show ~15 Hz

# 3. Check odometry
ros2 topic hz /rtabmap/odom              # Should show ~3-10 Hz
ros2 topic echo /rtabmap/odom_info --once | grep -E "inliers|features"

# 4. Check SLAM status
ros2 topic hz /rtabmap/info              # Should show ~1 Hz
ros2 topic echo /rtabmap/info --once | grep "total_nodes"

# 5. Check TF tree
ros2 run tf2_tools view_frames
xdg-open frames.pdf
Expected Output Indicators
Metric	Good Value	Poor Value	Meaning
Camera Hz	12-15 Hz	<5 Hz	Camera stream rate
Odometry Hz	5-15 Hz	<2 Hz	Odometry update rate
Inliers (Visual)	>20	<5	Feature matches
Inliers (ICP)	>100 points	<50 points	Point matches
Nodes	Increasing	Static	Map growth
Loop Closures	Occasional	None after revisit	Drift correction
Visualization
RTAB-Map Visualizer
The RTAB-Map visualization window shows:

3D Map View: Real-time point cloud and trajectory

Loop Closure Detection: Visual matches display

Odometry Panel: Feature tracks and inliers

Statistics: Node count, processing time

RViz2 Visualization
bash
# Launch RViz2 separately
source /opt/ros/humble/setup.bash
rviz2

# In RViz:
# 1. Set Fixed Frame: camera_link or map
# 2. Add -> PointCloud2 -> Topic: /rtabmap/cloud_map
# 3. Add -> Image -> Topic: /camera/color/image_raw
# 4. Add -> Map -> Topic: /rtabmap/map (occupancy grid)
# 5. Add -> Path -> Topic: /rtabmap/mapPath (trajectory)
View Camera Streams
bash
# View color image
ros2 run rqt_image_view rqt_image_view /camera/color/image_raw

# View depth image
ros2 run rqt_image_view rqt_image_view /camera/depth/image_raw

# View multiple streams
rqt
# Then: Plugins -> Visualization -> Image View
Troubleshooting
Camera Not Detected
Symptom: lsusb doesn't show Orbbec camera

Solution:

bash
# Check USB connection
lsusb | grep Orbbec

# Try different USB port (use USB 3.0 blue port)

# Check udev rules
ls /etc/udev/rules.d/ | grep obsensor

# Reinstall udev rules
cd ~/orbecc_ws/src/OrbbecSDK_ROS2/orbbec_camera/scripts
sudo cp 99-obsensor-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger

# Reboot
sudo reboot
Camera Permission Denied
Symptom: Error opening camera, permission denied

Solution:

bash
# Add user to video group
sudo usermod -a -G video $USER

# Set permissions (temporary)
sudo chmod 666 /dev/video*

# Reboot for group changes
sudo reboot
Zero Inliers / No Odometry
Symptom: Odom: quality=0, inliers=0/XX

Causes and Solutions:

Poor Lighting

bash
# Solution: Turn on room lights
# Verify features detected:
ros2 topic echo /rtabmap/odom_info | grep features
# Should show features > 100
Low Texture Environment

bash
# Solution: Point camera at textured surfaces
# Avoid: blank walls, ceilings, uniform floors
# Use: bookshelves, posters, furniture
Camera Not Moving

bash
# Solution: Move camera slowly (0.1-0.3 m/s)
# Odometry requires motion to estimate pose
Wrong Parameters

bash
# Solution: Use relaxed parameters
ros2 launch rtabmap_launch rtabmap.launch.py \
  rtabmap_args:="--Vis/MinInliers 5 --Vis/MaxFeatures 1000" \
  ...other args...
TF Frame Errors
Symptom: "Could not find a connection between 'odom' and 'camera_link'"

Solution:

bash
# Check TF tree
ros2 run tf2_tools view_frames
xdg-open frames.pdf

# Ensure camera launch includes:
# base_frame_id:=camera_link
# enable_publish_tf:=true

# Relaunch camera with correct frame_id
"No Odometry Provided" Error
Symptom: [ERROR] RGB-D SLAM mode is enabled but no odometry is provided

Solution:

bash
# Ensure visual_odometry:=true in rtabmap launch
# Or use separate odometry node with correct remapping

# Verify odometry is publishing:
ros2 topic hz /rtabmap/odom

# Check SLAM is subscribed:
ros2 node info /rtabmap/rtabmap | grep odom
High CPU Usage
Symptom: CPU >90%, system lag

Solutions:

bash
# 1. Reduce camera resolution
color_width:=640 color_height:=480 color_fps:=10

# 2. Reduce SLAM update rate
rtabmap_args:="--Rtabmap/DetectionRate 0.5"

# 3. Limit features
rtabmap_args:="--Vis/MaxFeatures 400"

# 4. Enable Jetson max performance
sudo nvpmodel -m 0
sudo jetson_clocks
Memory Issues
Symptom: Out of memory, SLAM crashes

Solutions:

bash
# 1. Limit working memory
rtabmap_args:="--Mem/STMSize 10"

# 2. Enable memory management
rtabmap_args:="--Mem/IncrementalMemory true --Mem/ReduceGraph true"

# 3. Save and restart periodically
# Stop SLAM, restart with new database

# 4. Monitor memory
htop
watch -n 1 free -h
Loop Closure Not Detected
Symptom: loop_closure_id: 0 always

Causes:

Not revisiting same location

Too few features in common view

Large viewpoint change

Solutions:

bash
# 1. Relax loop closure threshold
rtabmap_args:="--Rtabmap/LoopThr 0.8"

# 2. Increase loop closure candidates
rtabmap_args:="--Rtabmap/TimeThr 0"

# 3. Ensure good feature tracking
# Move slowly, maintain overlap
Advanced Topics
Saving and Loading Maps
Save Map
bash
# Map is saved automatically to ~/.ros/rtabmap.db
# To specify location:
rtabmap_args:="--Mem/DatabasePath /path/to/map.db"
Load Existing Map
bash
# Remove --delete_db_on_start
# Use localization mode:
ros2 launch rtabmap_launch rtabmap.launch.py \
  rtabmap_args:="" \
  localization:=true \
  ...other args...
Export Map
bash
# Export to point cloud
ros2 service call /rtabmap/export_map \
  rtabmap_msgs/srv/ExportMap \
  "{global: true, optimized: true, graphOnly: false}"

# Find exported file in:
ls ~/.ros/
Custom Parameter Tuning
Edit config/rtabmap_slam.yaml:

text
rtabmap:
  ros__parameters:
    # Odometry parameters
    Vis/MinInliers: "20"           # Min feature matches
    Vis/MaxFeatures: "1000"        # Max features per frame
    
    # Loop closure
    Rtabmap/LoopThr: "0.11"        # Loop closure threshold
    Rtabmap/TimeThr: "0"           # Time threshold (0=disabled)
    
    # Memory management
    Mem/STMSize: "30"              # Short-term memory size
    Mem/IncrementalMemory: "true"  # Enable SLAM mode
    
    # Optimization
    RGBD/OptimizeFromGraphEnd: "false"
    RGBD/LinearUpdate: "0.1"       # Min translation to add node
    RGBD/AngularUpdate: "0.1"      # Min rotation to add node
Multi-Camera Setup
For multiple cameras:

bash
# Launch camera 1
ros2 launch orbbec_camera gemini_330_series.launch.py \
  camera_name:=camera1 \
  ...

# Launch camera 2
ros2 launch orbbec_camera gemini_330_series.launch.py \
  camera_name:=camera2 \
  ...

# Use multi-camera RTAB-Map launch
# (Requires custom launch file)
Integration with Navigation Stack
bash
# Install Nav2
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup

# Launch RTAB-Map with map server
ros2 launch rtabmap_launch rtabmap.launch.py \
  publish_tf_map:=true \
  ...

# Use /rtabmap/map topic for Nav2
Performance Optimization
Jetson Orin Nano Specific
bash
# Set maximum performance mode
sudo nvpmodel -m 0
sudo jetson_clocks

# Check current mode
sudo nvpmodel -q

# Monitor performance
tegrastats

# Install jtop for monitoring
sudo -H pip3 install -U jetson-stats
jtop
Recommended Settings by Environment
Environment	Lighting	Visual Odom	ICP Odom	Resolution	FPS
Indoor Office	Good	✅ Yes	❌ No	640x480	15
Warehouse	Medium	✅ Yes	⚠️ Optional	640x480	10
Dark Room	Poor	❌ No	✅ Yes	640x480	15
Outdoor	Bright	✅ Yes	❌ No	1280x720	10
Workspace Structure
text
orbecc_ws/
├── .git/                         # Git repository
├── .gitignore                    # Git ignore file
├── .gitmodules                   # Git submodule config
├── README.md                     # This file
├── LICENSE                       # License file
├── config/                       # Configuration files
│   ├── rgbd_odom.yaml           # Visual odometry config
│   ├── rgbd_odom_relaxed.yaml   # Relaxed odometry config
│   ├── rtabmap_params.yaml      # RTAB-Map parameters
│   └── rtabmap_slam.yaml        # SLAM configuration
├── src/                          # Source packages
│   └── OrbbecSDK_ROS2/          # Orbecc camera driver (submodule)
│       ├── orbbec_camera/       # Camera ROS2 node
│       └── orbbec_camera_msgs/  # Camera message definitions
├── build/                        # Build artifacts (gitignored)
├── install/                      # Install files (gitignored)
└── log/                          # Build logs (gitignored)
System Requirements Summary
Minimum Requirements
Jetson Orin Nano 8GB

Ubuntu 22.04

32GB storage

USB 3.0 port

Recommended Requirements
Jetson Orin Nano Super 8GB

Ubuntu 22.04 with JetPack 6.0

128GB NVMe SSD

USB 3.0 hub with power

Good lighting (>300 lux)

Known Issues
Issue 1: Camera Timeout on First Launch
Symptom: Camera fails to initialize on first launch after boot
Workaround: Unplug and replug camera, or run sudo systemctl restart udev

Issue 2: Drift in Long Trajectories
Symptom: Map drift without loop closures
Workaround: Periodically revisit starting location for loop closure

Issue 3: High Latency with Visualization
Symptom: Lag when rtabmap_viz is running
Workaround: Disable visualization for mapping, enable only for review:

bash
rtabmap_viz:=false
FAQ
Q: Can I use this with other Orbecc cameras (Astra, Femto)?
A: Yes, but you may need to change the launch file name. Check orbbec_camera/launch/ for your model.

Q: Does this work on x86_64 PC?
A: Yes! Works on any Ubuntu 22.04 system with ROS2 Humble.

Q: Can I run SLAM in real-time and save the map?
A: Yes, map is automatically saved to ~/.ros/rtabmap.db during operation.

Q: How do I reset the map and start fresh?
A: Use --delete_db_on_start flag or delete ~/.ros/rtabmap.db

Q: What's the maximum mapping area?
A: Tested up to 100m x 100m indoor environments. Limited by memory and computation.

Contributing
Contributions are welcome! Please:

Fork the repository

Create a feature branch: git checkout -b feature/my-feature

Test thoroughly on Jetson Orin Nano

Commit changes: git commit -m "Add feature"

Push to branch: git push origin feature/my-feature

Open a Pull Request

Development Guidelines
Follow ROS2 naming conventions

Test on Jetson Orin Nano before PR

Document all parameters

Include launch file examples

License
This project is licensed under the MIT License - see the LICENSE file for details.

Individual components:

OrbbecSDK_ROS2: Apache 2.0 License

RTAB-Map: BSD License

ROS2: Apache 2.0 License

Acknowledgments
Orbecc - For Gemini 335 camera and SDK

RTAB-Map Team - For excellent SLAM library

ROS2 Community - For robotics middleware

NVIDIA - For Jetson platform

Citation
If you use this workspace in your research, please cite:

text
@software{orbecc_rtabmap_ws,
  author = {Your Name},
  title = {Orbecc Gemini 335 RTAB-Map SLAM Workspace},
  year = {2026},
  url = {https://github.com/YOUR_USERNAME/orbecc_ws}
}
Contact
Author: Your Name

GitHub: @YOUR_USERNAME

Email: your.email@example.com

Issues: GitHub Issues

Version History
v1.0.0 (2026-02-01)
Initial release

Orbecc Gemini 335 support

RTAB-Map RGB-D SLAM integration

Visual and ICP odometry modes

Jetson Orin Nano optimization

Comprehensive documentation

Additional Resources
Documentation
RTAB-Map Documentation

ROS2 Humble Docs

Orbecc SDK Docs

Jetson Orin Docs

Tutorials
RTAB-Map Tutorials

ROS2 Tutorials

Community
ROS Discourse

NVIDIA Jetson Forums

RTAB-Map Google Group

Last Updated: February 1, 2026
