# SE(2) — Rigid Body Transformation in 2D

## What problem does this solve?
A sensor reading is expressed in the sensor's own frame.
A robot exists in a world frame.
SE(2) provides the mathematical tool to express any point
from one 2D frame in another — handling both rotation
and translation in a single operation.

## Engineering Intuition
Simple addition of coordinates only works when frames
are aligned (no rotation). The moment a frame is rotated,
x and y components mix together. SE(2) handles any
rotation and any translation through one matrix multiply.

## Mathematical Idea
A 2D pose (x, y, θ) is encoded as a 3×3 matrix:

$$T = \begin{bmatrix}
\cos\theta & -\sin\theta & t_x \\
\sin\theta & \cos\theta & t_y \\
0 & 0 & 1
\end{bmatrix}$$

Points are expressed in homogeneous coordinates [x, y, 1].
The third coordinate is a scale factor for the translation:
- 1 → real physical point (translation applied)
- 0 → direction vector (translation ignored)
- 2 → translation doubled (meaningless physically)

Chaining frames = matrix multiplication, right to left:
$$T_{A \to C} = T_{B \to C} \cdot T_{A \to B}$$

## Connection to Previous Concepts
Foundation concept — everything else builds on this.

## Relationship to ROS2
Every transform published on /tf is an SE(2) or SE(3)
operation. tf2::Transform encodes exactly this matrix
internally.

## Relationship to TF2
TF2 maintains the full transform tree and computes
chained transforms automatically. Understanding SE(2)
means understanding what TF2 computes at every lookup.

## Relationship to Navigation
The Nav2 costmap, footprint, and path all exist in
the map frame. Sensor data arrives in sensor frames.
SE(2) is the bridge between them.

## Relationship to SLAM
SLAM builds a map by accumulating SE(2)/SE(3) transforms
between robot poses. The pose graph is a graph of
SE(2) transformations connected by constraints.

## Relationship to EKF
The EKF state vector includes robot pose (x, y, θ).
The motion model predicts the new pose by composing
SE(2) transformations from odometry.

## Real Robotics Applications
- LiDAR point cloud transformation to world frame
- Camera detection localization in world coordinates
- Robot footprint projection onto costmap
- Odometry accumulation over time

## Common Mistakes
1. Adding translations directly when rotation is present
2. Multiplying transformation matrices in wrong order
3. Forgetting that matrix multiplication is right-to-left
4. Treating [x, y, 0] as a point instead of a direction

## References
- Siciliano, Robotics: Modelling, Planning and Control, Ch.2
- ROS2 TF2 Concepts: docs.ros.org/en/humble/Concepts/Intermediate/About-Tf2.html
