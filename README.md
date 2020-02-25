# ros_stereo_odo
A ros compatible implementation of stereo visual odometry 

## Getting Started

1. Clone Package into a ROS workspace
2. Run the command "roslaunch ros_stereo_odo stereo_odom_node"


## Outline of Method

1. Rectify Images and Extract Fast Features
2. Match Features using LK Optical Flow
3. Use Camera Matrcicies to Project Points into 3D
4. Use Maximum Clique Inlier Detection to Prune Points
5. Use LM Optimization to calculate camera pose change


### Based off Avi Singh's Blog Post on Stereo Odometry
https://avisingh599.github.io/vision/visual-odometry-full/
