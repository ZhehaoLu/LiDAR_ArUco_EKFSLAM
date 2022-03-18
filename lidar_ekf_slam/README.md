# ME495 Sensing, Navigation and Machine Learning For Robotics
* Zhehao (Jonas) Lu
* Winter 2022
# Package List
This repository consists of several ROS packages
- `nuturtle_description` implements Turtlebot3 URDF modification and visualization in rviz
- `turtlelib` is a handwritten library including vector, twist and transformation calculations
- `nusim` includes functions of position tracking and obstacle setting
- `nuturtle_control` develops some nodes that are useful in both simulation and on the real robot
- `nuslam` introduces EKF algorithm for optimization

# Video & Screenshot
- `HW4 Video` Link for Rviz Simulation: 
https://youtu.be/ZHGrI5ZNQc0
- `HW4 Video` Link for Real World Testing: 
https://youtu.be/6MdL9HDXfKs

- `HW4 screenshot` EKF algorithm with unknown data association
![record](https://user-images.githubusercontent.com/85860671/158699779-fa1c3cc8-7115-46ae-b840-eda01c736bc1.png)

- `HW3 screenshot` EKF algorithm with known data association
![1](https://user-images.githubusercontent.com/85860671/156291943-b08acc94-2fd5-4725-bea1-f1137cc972ff.png)

# Physical Testing
- the initial configuration of all experiments are: [0,0,0]

1. [Forward Drive] Drive the robot forward and backward in a straight line several times, and then stopping when the turtlebot is in its initial configuration

* link of video: https://www.youtube.com/watch?v=WkXTSufuvzs
* link of rviz screencast: https://www.youtube.com/watch?v=aQ4EoheSC7g
* Final Odometry Record:
Position: [x,y,z] = [0.078,-0.003,0.0]
Orientation: [x,y,z,w] = [0.0,0.0,0.012,0.999]

2. [Pure Rotation] Rotate the robot clockwise and counter clockwise several times, stopping when the turtlebot is in its initial configuration

* link of video: https://www.youtube.com/watch?v=zeSzThRpZmk
* link of rviz screencast: https://www.youtube.com/watch?v=o9_bItID7-g
* Final Odometry Record:
Position: [x,y,z] = [0.002,-0.001,0.0]
Orientation: [x,y,z,w] = [0.0,0.0,0.785,0.620]

3. [Make Circle] Drive the robot in a circle, clockwise and counter clockwise several times, stopping when the turtlebot is its initial configuration

* link of video: https://www.youtube.com/watch?v=McGvS7-VhHM&t=1s
* link of rviz screencast: https://www.youtube.com/watch?v=lsR362bYf5c
* Final Odometry Record:
Position: [x,y,z] = [0.044,-0.005,0.0]
Orientation: [x,y,z,w] = [0.0,0.0,-0.152,0.988]

4. [Worst Scenerio] Try one of the previous experiments again - Make Circle

* link of video: https://www.youtube.com/watch?v=awr9UOc-azg
* link of rviz screencast: https://www.youtube.com/watch?v=SJqj5DVDj1o
* Final Odometry Record:
Position: [x,y,z] = [0.097,-0.035,0.0]
Orientation: [x,y,z,w] = [0.0,0.0,0.349,0.937]

# Answer Questions
1. Was your driving able to make a difference?
- Yes.

2. What did you change with your driving to accomplish this?
- When the turtlebot makes more circles, the accuracy of the odometry becomes worse gradually. This is caused by the fact that the current configuration is calculated in a way that takes the last configuration into consideration, which means the drift is accumulating as time goes by. Therefore, just extending the driving duration (with the same speed) make the odometry less accurate. 





