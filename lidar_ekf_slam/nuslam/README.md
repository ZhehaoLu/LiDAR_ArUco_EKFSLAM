# Video Link
- Link for Rviz Simulation: 
https://youtu.be/ZHGrI5ZNQc0
- Link for Real World Testing: 
https://youtu.be/6MdL9HDXfKs

# Physical Testing
- The initial and final position of the experiment is: [0,0]

1. [Odometry] pure odometry is used in this case (without EKF)

* Final Odometry Position: [x,y] = [0.18,0.12]

2. [EKF SLAM] the position estimate is calculated by both odometry and EKF

* Final EKF Record Position: [x,y] = [0.06,0.00]

# Conclusion
- EKF SLAM algorithm outperform the odometry.

# Screenshot

![record](https://user-images.githubusercontent.com/85860671/158702216-4d25131c-966f-4afe-8cb9-18b2dacc68d4.png)




