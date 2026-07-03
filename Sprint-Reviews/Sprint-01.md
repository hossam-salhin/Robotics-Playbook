# Sprint 1 Review — The Mathematical Language of Robotics

## What I Learned

### Coordinate Frames and Transforms
Every sensor reading exists in a frame. To use it,
you must transform it through the chain:
sensor → base_link → world.
This chain is what TF2 computes automatically — but
now I understand what it actually calculates.

### SE(2)
A 3×3 matrix encoding 2D rotation and translation
together. Points use homogeneous coordinates [x,y,1].
The 1 is a scale factor for translation — not arbitrary.
Chaining frames = matrix multiplication, right to left.

### SE(3)
Direct extension to 3D. 4×4 matrix with a 3×3
rotation block and 3×1 translation column.
Bottom row [0,0,0,1] — same logic as SE(2).

### Rotation Matrices
Three individual axis rotations: Rx, Ry, Rz.
ROS ZYX convention: R = Rz · Ry · Rx
Rightmost matrix applies first.
Order matters — rotations do not commute.

### Euler Angles
Human-readable but fail in engineering practice.
Two critical problems:
1. Gimbal Lock — loss of one DOF at pitch=90°
2. Numerical drift — matrix loses orthonormality
   after continuous integration

### Quaternions
Encode rotation as axis-angle: q = (x,y,z,w)
x,y,z = axis scaled by sin(θ/2)
w = cos(θ/2)
Solves both Euler angle problems.
w=1: no rotation. w=0: 180° rotation.
Reading w gives immediate orientation intuition.

## What Changed in My Understanding
- Transforms are not just a ROS2 tool — they are
  a mathematical operation I can now derive manually
- Rotation order felt intuitive before but I could
  not explain why. Now I can derive the correct order
  from first principles
- Quaternions felt like magic numbers from IMU output.
  Now I can compute them, read them, and verify them

## Remaining Weaknesses
- Quaternion multiplication (composing two rotations
  as quaternions) not yet covered
- SLERP interpolation mentioned but not implemented
- SE(3) exercise not yet done with real numbers

## Connections Between Topics
SE(2) → SE(3): same structure, one dimension higher
Euler Angles → Quaternions: same rotation, better representation
Homogeneous coordinates: the thread connecting SE(2), SE(3),
and the reason translation works inside matrix multiplication

## Relationship to AMR Project
- Full TF tree (map→odom→base_link→sensors) is a
  chain of SE(3) transforms computed every cycle
- IMU publishes quaternions — now I understand every
  field of that message
- TF2 timestamp bugs: stale transforms bias obstacle
  positions in the direction of motion — a systematic
  error, not random noise

## Relationship to Robo Core
- A Transform3D class belongs in Coding Lab Sprint 1
- The sensor registry publishes data in sensor frames —
  transform chain needed before any fusion is possible

## What Should Be Reinforced Before Moving Forward
- Complete the SE(3) numerical exercise (camera landmark,
  roll+pitch+yaw robot)
- Implement Transform2D and Transform3D in Coding Lab
- Verify quaternion reading fluency on 2-3 more examples

## Sprint 1 Interview Questions
1. A point is at (1, 0, 0) in a frame rotated 90° about z
   relative to the world. Where is it in the world frame?
   Derive it using the rotation matrix, not intuition.

2. Your TF tree shows map→odom→base_link→lidar.
   The odom→base_link transform has a stale timestamp.
   What breaks? In which direction is the error? Why?

3. An IMU publishes quaternion (0, 0.707, 0, 0.707).
   What is the rotation axis? What is the angle?
   Which Euler angle does this correspond to?

4. Why does ROS2 use quaternions instead of
   rotation matrices internally? Give two reasons
   with engineering justification, not just names.

5. What is Gimbal Lock? Give a concrete physical
   example on a robot. Why are quaternions immune?
