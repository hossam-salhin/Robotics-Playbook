# Quaternions

## What problem does this solve?
Two problems with rotation matrices and Euler angles:
1. Numerical drift during continuous integration
2. Gimbal lock — loss of one rotational DOF

Quaternions solve both.

## Engineering Intuition
w=1 → no rotation
w=0 → 180° rotation
Reading w from an IMU quaternion gives immediate
intuition about how far the robot has rotated from
its reference orientation.

## Mathematical Idea
A quaternion encodes an axis-angle rotation:

$$q = \begin{bmatrix}
a_x\sin(\theta/2) \\
a_y\sin(\theta/2) \\
a_z\sin(\theta/2) \\
\cos(\theta/2)
\end{bmatrix} = (x, y, z, w)$$

Where (ax, ay, az) is the unit rotation axis
and θ is the rotation angle.

Key examples:
- No rotation: (0, 0, 0, 1)
- 180° yaw: (0, 0, 1, 0)
- 90° yaw: (0, 0, 0.707, 0.707)

Staying valid = keeping magnitude = 1.
Re-normalization: divide by sqrt(x²+y²+z²+w²).
Far cheaper than matrix re-orthogonalization.

## Why Not Euler Angles
No gimbal lock. No loss of DOF at any orientation.
Smooth interpolation (SLERP) between orientations.

## Relationship to ROS2
geometry_msgs/Quaternion: x, y, z, w fields.
All TF2 rotations stored as quaternions.
IMU drivers publish orientation as quaternions.
robot_localization fuses quaternion orientations.

## Relationship to TF2
tf2::Quaternion is the primary rotation type.
tf2::Transform = translation + quaternion = SE(3).

## Common Mistakes
1. Confusing (x,y,z) with the raw axis vector —
   they are axis scaled by sin(θ/2)
2. Evaluating sin(90°) = 0 instead of 1
3. Forgetting w encodes cos(θ/2), not θ directly
4. Using (0,0,0,0) — not a valid quaternion
