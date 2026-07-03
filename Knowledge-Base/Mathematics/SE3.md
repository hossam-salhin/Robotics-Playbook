# SE(3) — Rigid Body Transformation in 3D

## What problem does this solve?
Real robots operate in 3D space. Sensors are tilted,
mounted at angles, or placed on robots that roll and
pitch on uneven terrain. SE(3) extends SE(2) to handle
full 3D rotation and translation.

## Engineering Intuition
A ground robot is dominated by yaw — its heading.
Roll and pitch are small corrections from terrain.
This is why 2D SLAM works well for warehouse robots
despite the world being 3D: the yaw component carries
almost all the orientation information.

## Mathematical Idea
A 3D pose is encoded as a 4×4 matrix:

$$T = \begin{bmatrix}
R_{3\times3} & t_{3\times1} \\
0\ 0\ 0 & 1
\end{bmatrix}$$

Where R is a 3×3 rotation matrix and t is a 3×1
translation vector. Points use homogeneous coordinates
[x, y, z, 1]. Same chaining rule as SE(2):

$$T_{A \to C} = T_{B \to C} \cdot T_{A \to B}$$

## Connection to Previous Concepts
Direct extension of SE(2). Same homogeneous coordinate
logic, same chaining by multiplication, one dimension
higher.

## Relationship to ROS2
geometry_msgs/TransformStamped encodes SE(3).
The translation field is t. The rotation field is
a quaternion (which encodes R).

## Relationship to TF2
Every transform in the TF tree is an SE(3) operation.
tf2::Transform stores translation + quaternion,
computes the full 4×4 internally.

## Relationship to Navigation
3D sensor data (cameras, 3D LiDAR) requires SE(3)
transforms before projecting to 2D costmaps.

## Relationship to SLAM
3D SLAM (e.g. cartographer in 3D mode) uses SE(3)
pose graphs. Each node is an SE(3) pose.

## Relationship to EKF
Full 3D state estimation uses SE(3) motion models.
robot_localization with 3D mode operates in SE(3).

## Real Robotics Applications
- 3D LiDAR point cloud transforms
- Camera extrinsic calibration
- IMU orientation integration
- Robot arm end-effector pose

## Common Mistakes
1. Using SE(2) for sensors that have non-zero tilt
2. Forgetting the bottom row [0,0,0,1]
3. Mixing up 3×3 rotation blocks with full 4×4 transforms
