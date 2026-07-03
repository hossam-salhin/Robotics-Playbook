# Euler Angles — Roll, Pitch, Yaw

## What problem does this solve?
Human-readable representation of 3D orientation.
Easy to understand and visualize intuitively.

## Engineering Intuition
Roll = rotation around x (tipping sideways)
Pitch = rotation around y (nodding forward/back)
Yaw = rotation around z (spinning on the spot)

For ground robots: yaw dominates. Roll and pitch
are small terrain corrections.

## The Two Conventions
Intrinsic: each rotation is around the body's
current (already-rotated) axes. Order: roll→pitch→yaw.

Extrinsic: each rotation is around the fixed world
axes. Order reversed: yaw→pitch→roll.

Both produce the same final orientation.
ROS uses intrinsic RPY.

## Why Euler Angles Fail in Practice

### Problem 1: Gimbal Lock
When pitch = 90°, roll and yaw rotate around the
same physical axis. One degree of freedom is lost.
Certain orientations become unreachable.

### Problem 2: Numerical Drift
Continuous integration of rotation matrices causes
floating point errors to accumulate — the matrix
slowly stops being a valid rotation.

## Relationship to ROS2
tf2::Matrix3x3::getRPY() converts quaternions to
Euler angles for human inspection. Internally ROS2
always uses quaternions, never raw Euler angles.

## Common Mistakes
1. Assuming rotation order doesn't matter
2. Using Euler angles for continuous integration
3. Mixing intrinsic and extrinsic conventions
