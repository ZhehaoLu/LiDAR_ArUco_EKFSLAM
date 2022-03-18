# LiDAR ArUco EKF SLAM
## Project Overview
This repository builds feature-based EKF SLAM on `Turtlebot3` from scratch. 
The core modules are:
- `Odometry` - odometry calculations are performed by `Differential Drive Kinematics`
- `Measurement` - both LiDAR and ArUco measurements are implemented
  - LiDAR measurement - unknown data association in `lidar_ekf_slam` package
  - ArUco measurement - known data association in `aruco_ekf_slam` package

Please navigate to the respective directories for a detailed description of each component.
## LiDAR EKF SLAM - LiDAR measurement
### Core Component
- `turtlelib` - contains 2D Lie Group operations for Transforms, Vectors and Twists as well as differential drive robot kinematics for odometry updates
- `nuturtle_description` - houses the description of a differential drive robot with a caster wheel for support
- `nuturtle_control` - develops some nodes that are useful in both simulation and on the real robot
- `nusim` - includes functions of position tracking and obstacle setting
- `nuslam` - introduces LiDAR feature detection and EKF SLAM implementation
### Animation & Video
- Rviz Simulation
  - Youtube Link: https://youtu.be/ZHGrI5ZNQc0

![ezgif com-gif-maker(2)](https://user-images.githubusercontent.com/85860671/159085600-b7557e4e-97f7-4c85-8e48-2db831ec95b1.gif)

- Real World Video
 - Youtube Link: https://youtu.be/6MdL9HDXfKs

![ezgif com-gif-maker](https://user-images.githubusercontent.com/85860671/159086766-164e4652-1598-41ea-bbc6-f887efa64803.gif)

### Start Guide
- `catkin_make` - build the package
- `roslaunch nuslam unknown_data_assoc.launch` - launch `LiDAR EKF SLAM` algorithm with unknown data association 

## ArUco EKF SLAM - ArUco measurement
### Dataset
Dataset Link: https://pan.baidu.com/s/10crRfgGcZ-XgcBjefxAPDg     Password: pqby
- The launch file (slam.launch) should be modified to the correct download path

### Animation
![aruco_ekf_slam_gif](https://user-images.githubusercontent.com/85860671/153775051-1f493f74-f297-4429-ab6b-63b8ffa021af.gif)

### Start Guide
---
- `catkin_make` - build the package
- `roslaunch aruco_ekf_slam slam.launch` - launch `ArUco EKF SLAM` algorithm with known data association
- `rosbag play aruco_slam_data_qhd1.bag -r 5` - play the rosbag

## Prerequisites
Ubuntu20.04, OpenCV 3.1, Eigen, ROS
## References
1. OpenCV library for the fundamental functions of Aruco codes: https://docs.opencv.org/3.3.0/d9/d6d/tutorial_table_of_content_aruco.html
2. Probalisitic Robotics  -- by Thrun, S., Burgard, W., Fox, D. The MIT Press (2005) 
