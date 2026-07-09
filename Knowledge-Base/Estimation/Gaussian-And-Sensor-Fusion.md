# Probabilistic State Estimation — Gaussian Distributions and Sensor Fusion

## What problem does this solve?
A robot using wheel encoders alone accumulates uncertainty 
with every motion step. After many steps, the position 
estimate is nearly useless. We need a mathematical framework 
that represents not just where the robot probably is, but 
how confident we are — and how to improve that confidence 
using sensor measurements.

## Engineering Intuition
Robot state is never a single point. It is a probability 
distribution — a range of possible states with associated 
likelihoods. Motion widens this distribution (more uncertain). 
Measurements narrow it (more certain). The entire field of 
robot localization is built on managing this distribution.

## The Gaussian Distribution
The Gaussian (normal) distribution is the fundamental tool 
for representing uncertainty in robotics.

In 1D, it is completely defined by two numbers:

**Mean (μ):** The center of the distribution. 
The most likely value. Your best estimate.

**Variance (σ²):** How spread out the uncertainty is.
- Small σ² → narrow spike → high confidence
- Large σ² → wide flat curve → low confidence

Mathematical form:
$$p(x) = \frac{1}{\sqrt{2\pi\sigma^2}} 
e^{-\frac{(x-\mu)^2}{2\sigma^2}}$$

Reading this equation:
- When x = μ: exponent = 0, p(x) is maximum
- As x moves away from μ: exponent grows negative, p(x) drops
- σ² controls how fast it drops

## Uncertainty Accumulates with Motion
Every motion step adds uncertainty:
$$\sigma^2_{new} = \sigma^2_{old} + \sigma^2_{motion}$$

This is why dead reckoning (encoders only) fails over long 
distances. The Gaussian gets wider with every step.

## Sensor Fusion — Combining Two Gaussians
When two independent estimates of the same quantity exist, 
they are combined by multiplying their Gaussians:

**Combined mean:**
$$\mu_{c} = \frac{\sigma^2_2 \cdot \mu_1 + 
\sigma^2_1 \cdot \mu_2}{\sigma^2_1 + \sigma^2_2}$$

**Combined variance:**
$$\sigma^2_{c} = \frac{\sigma^2_1 \cdot \sigma^2_2}
{\sigma^2_1 + \sigma^2_2}$$

### Critical insight on weighting
Each mean is weighted by the **other** sensor's variance.
- Small variance on sensor B → large weight on μ_B
- Large variance on sensor A → small weight on μ_A
- Result is always pulled toward the more certain estimate

### Properties of combined Gaussian
- Combined variance is always smaller than both inputs
- Two sources of information are always more certain than one
- The more certain sensor dominates the combined mean

### Numerical example
Motion model: μ₁ = 10.0, σ²₁ = 0.9
LiDAR:        μ₂ = 10.2, σ²₂ = 0.1

$$\mu_c = \frac{0.1 \times 10.0 + 0.9 \times 10.2}
{0.9 + 0.1} = 10.18$$

$$\sigma^2_c = \frac{0.9 \times 0.1}{0.9 + 0.1} = 0.09$$

Result: closer to LiDAR (more certain), narrower than both.

## The Kalman Filter — Core Intuition
The Kalman Filter applies sensor fusion in two alternating 
steps, continuously:

**Predict:** Apply motion model → mean shifts, variance grows
$$\mu_{pred} = \mu + u$$
$$\sigma^2_{pred} = \sigma^2 + \sigma^2_{motion}$$

**Update:** Apply measurement → mean corrects, variance shrinks
(Uses the sensor fusion equations above)

**Variance equilibrium:** Over time, variance reaches a stable 
steady state where prediction growth and update shrinkage 
balance. The filter self-regulates — no manual tuning needed 
at runtime.

## From 1D to 2D — The Covariance Matrix
In 2D, robot state is (x, y). Uncertainty in x and y are 
not always independent — they can be correlated.

Example: Robot moving northeast at 45°. Wheel slip causes 
error in both x and y simultaneously from the same source. 
Large error in x implies large error in y — positive 
correlation.

The covariance matrix encodes this:
$$\Sigma = \begin{bmatrix} 
\sigma^2_x & \sigma_{xy} \\ 
\sigma_{xy} & \sigma^2_y 
\end{bmatrix}$$

- Diagonal entries: variance in each dimension
- Off-diagonal entries: correlation between dimensions
- σ_{xy} = 0: x and y uncertainty are independent
- σ_{xy} large positive: x and y move together

### Why the covariance matrix is always symmetric
σ_{xy} = σ_{yx} because "how x and y vary together" is 
the same relationship regardless of direction. 
One physical relationship, not two.

**Engineering rule:** If you compute a covariance matrix 
that is not symmetric, you have a bug.

### Geometric interpretation
The covariance matrix defines an uncertainty ellipse:
- Circle → equal uncertainty in all directions (σ_{xy} = 0, σ²_x = σ²_y)
- Ellipse axis-aligned → unequal variance, no correlation
- Ellipse tilted → correlated uncertainty (robot moving at an angle)

## Connections

### To Sprint 1
Uncertainty lives in the same coordinate frames from SE(2) 
and SE(3). The covariance ellipse exists in the robot's 
pose space. Transforming uncertainty between frames requires 
the rotation matrices from Sprint 1.

### To ROS2
- sensor_msgs/Imu includes a 9-element covariance array 
  (flattened 3×3 matrix for orientation, angular velocity, 
  linear acceleration)
- nav_msgs/Odometry includes pose covariance (6×6) and 
  twist covariance (6×6)
- robot_localization reads these covariances to weight 
  sensor inputs correctly

### To AMR Project
robot_localization runs an EKF using exactly these equations. 
The predict step uses wheel odometry. The update step uses 
IMU and/or LiDAR. The covariance matrices in your launch 
file configuration directly encode σ²_motion and σ²_sensor.

### To SLAM
SLAM maintains a covariance over the full map and robot pose 
simultaneously. Loop closure is a measurement update that 
dramatically shrinks the covariance of all past poses.

### To EKF (next session)
The Extended Kalman Filter extends these equations to 
nonlinear motion and sensor models — which is what real 
robots require.

## Common Mistakes
1. Thinking each mean is weighted by its own variance — 
   it is weighted by the OTHER sensor's variance
2. Assuming combined variance is the average of both — 
   it is always smaller than both
3. Forgetting that motion adds variance — dead reckoning 
   always degrades
4. Treating covariance matrix as diagonal when robot moves 
   at an angle — off-diagonal terms are real and matter
5. Computing a non-symmetric covariance matrix — 
   always a bug

## References
- Probabilistic Robotics, Thrun et al. — Pages 1–4, 27–32
- robot_localization ROS2 documentation
- nav_msgs/Odometry message definition
