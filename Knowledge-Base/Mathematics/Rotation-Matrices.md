# Rotation Matrices in 3D

## What problem does this solve?
Representing how a frame is rotated relative to another
in 3D, as a mathematical object that can be composed,
inverted, and applied to points.

## Engineering Intuition
Each individual rotation matrix rotates around one axis
only — the row/column of that axis stays untouched,
the other two axes mix via sin/cos. Combining three
axis rotations gives full 3D orientation.

## Mathematical Idea
$$R_x(\phi) = \begin{bmatrix}1&0&0\\0&\cos\phi&-\sin\phi\\0&\sin\phi&\cos\phi\end{bmatrix}$$

$$R_y(\theta) = \begin{bmatrix}\cos\theta&0&\sin\theta\\0&1&0\\-\sin\theta&0&\cos\theta\end{bmatrix}$$

$$R_z(\psi) = \begin{bmatrix}\cos\psi&-\sin\psi&0\\\sin\psi&\cos\psi&0\\0&0&1\end{bmatrix}$$

Full rotation (ROS ZYX convention):
$$R = R_z(\psi)\cdot R_y(\theta)\cdot R_x(\phi)$$

## Properties
- Determinant = 1 (always)
- R^T = R^{-1} (transpose = inverse)
- Columns are orthonormal unit vectors

## Common Mistakes
1. Applying rotations in wrong order
2. Numerical drift after many multiplications
   (matrix loses orthonormality over time)
3. Confusing intrinsic vs extrinsic order
