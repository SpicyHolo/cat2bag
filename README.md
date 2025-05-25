# cat2bag
ROS2 inteface for KITTI odometry dataset (velodyne data)
It converts velodyne laser data, with ground truth poses to rosbag (ros2) format.
Uses [pykitti](https://github.com/utiasSTARS/pykitti).

# Install
Requires ROS2, and a few python libraries.
```bash
pip3 install pykitti
pip3 uninstall numpy 
pip3 install "numpy<2"
pip3 install transforms3d
```
Download, install in ros2 workspace
```bash
cd ros2_ws/src
git clone https://github.com/SpicyHolo/cat2bag
cd ..
colcon build --symlink-install
source install/setup.bash
```

# Usage
### KITTI Dataset file structure
```
├── KITTI
│   └── odometry
│       ├── poses // ground truth poses
│       └── sequences
│           ├── 00 
│           ├── 01 
│           ├── ...
│           └── 21
│               ├── velodyne
│               ├── calib.txt
│               └── times.txt
```
Make sure to merge calib and laser data archives to get `calib.txt` for each sequence.

Path to KITTI dataset is set using ros2 run arguments,
same for lidar topic, pose (ground truth) topic and chosen sequence (01-10) //
Publisher (with 10Hz standard KITTI frequency)
```
ros2 run cat2bag publish --ros-args \
    -p dataset_path:=/ros2_ws/KITTI/odometry \
    -p lidar_topic_name:=/kitti/velodyne_points \
    -p pose_topic_name:=/kitti/pose \
    -p sequence:=0 \
```

Convert to ros2bag
```
ros2 run cat2bag bag --ros-args \
    -p dataset_path:=/ros2_ws/KITTI/odometry \
    -p lidar_topic_name:=/kitti/velodyne_points \
    -p pose_topic_name:=/kitti/pose \
    -p sequence:=0 \
```
