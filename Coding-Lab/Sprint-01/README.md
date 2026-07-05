# Coding Lab — Sprint 01: Rigid Body Transforms

## Objective
Implement SE(2) and SE(3) transformation matrices from scratch in C++17.
No external libraries. Pure mathematics, pure code.

## What This Implements

### Transform2D — SE(2)
Full 2D rigid body transform encoding rotation and translation
in a single 3×3 homogeneous matrix.

### Transform3D — SE(3)
Full 3D rigid body transform encoding rotation (Rz·Ry·Rx)
and translation in a single 4×4 homogeneous matrix.

## API

```cpp
// 2D
Transform2D T(x, y, theta_degrees);
T.apply(x, y);                    // transform a point
T.compose(other);                 // chain transforms
T.inverse();                      // reverse transform
T.isIdentity(epsilon);            // validate identity

// 3D
Transform3D T(x, y, z, roll, pitch, yaw);  // angles in degrees
T.apply(x, y, z);
T.compose(other);
T.inverse();
T.isIdentity(epsilon);