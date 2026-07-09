# TF2 Concepts — Reading Notes

Source: https://docs.ros.org/en/humble/Concepts/Intermediate/About-Tf2.html
Sprint: 1

Key ideas:
- Every transform has a parent frame, child frame, and timestamp
- TF2 maintains a time-buffered tree of transforms
- lookupTransform requires a timestamp — stale transforms 
  produce systematic directional errors in the direction of motion
- map → odom → base_link → sensors is the standard chain

Connection to Sprint 1:
Each edge in the TF tree is an SE(3) transform — exactly 
what Transform3D implements.