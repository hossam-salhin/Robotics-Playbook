# Sprint 2 — Complete Master Review
# Probabilistic State Estimation

---

## The Central Question of Sprint 2

A robot moves through a warehouse using wheel encoders.
After 100 steps — where is it?

The answer is never a single point.
The answer is always a probability distribution.

This is the foundation of everything in Sprint 2.

---

## CHAPTER 1 — Representing Uncertainty

### Why a Single Point Is Wrong
Encoders drift. Motors slip. Terrain is uneven.
Every motion command produces a slightly different 
result each time it is executed.

After 10 steps, the robot is probably near x=5.0.
But it could be at 4.8, or 5.2, or 4.5.
Each of these has a different likelihood.

### The Gaussian Distribution
We represent this uncertainty as a Gaussian —
a bell curve over all possible positions.

Completely defined by two numbers:

**Mean (μ):** Center of the bell curve.
Your best estimate. Most likely value.
Example: μ = 5.0m

**Variance (σ²):** Width of the bell curve.
How spread out the uncertainty is.
- Small σ² → narrow spike → high confidence
- Large σ² → wide flat curve → low confidence

Mathematical form:
$$p(x) = \frac{1}{\sqrt{2\pi\sigma^2}}
e^{-\frac{(x-\mu)^2}{2\sigma^2}}$$

Reading this:
- At x=μ: exponent=0, p(x) is maximum
- Further from μ: p(x) drops
- σ² controls how fast it drops

### Motion Always Adds Uncertainty
Every motion step makes the Gaussian wider:
$$\sigma^2_{new} = \sigma^2_{old} + \sigma^2_{motion}$$

This is dead reckoning failure.
Encoders alone always degrade over time.

### Sensor Fusion — Combining Two Gaussians
When two independent estimates exist for the same 
quantity, multiply their Gaussians.

Result:
$$\mu_c = \frac{\sigma^2_2 \cdot \mu_1 + 
\sigma^2_1 \cdot \mu_2}{\sigma^2_1 + \sigma^2_2}$$

$$\sigma^2_c = \frac{\sigma^2_1 \cdot \sigma^2_2}
{\sigma^2_1 + \sigma^2_2}$$

**The weighting rule:**
Each mean is weighted by the OTHER sensor's variance.
Small variance on sensor B → large weight on μ_B.
The more certain sensor pulls the result toward itself.

**Always true:**
Combined variance < both individual variances.
Two sources of information are always more certain 
than either alone.

### Numerical Example — Sensor Fusion
Motion model: μ₁=10.0, σ²₁=0.9
LiDAR:        μ₂=10.2, σ²₂=0.1

$$\mu_c = \frac{0.1(10.0)+0.9(10.2)}{0.9+0.1}
= \frac{1.0+9.18}{1.0} = 10.18$$

$$\sigma^2_c = \frac{0.9 \times 0.1}{0.9+0.1} = 0.09$$

Result pulled toward LiDAR (more certain).
Combined variance 0.09 < both 0.9 and 0.1.

### From 1D to 2D — The Covariance Matrix
In 2D, robot has uncertainty in both x and y.
These can be correlated.

Example: Robot moving northeast at 45°.
Wheel slip affects x and y simultaneously.
Large error in x implies large error in y.
They share the same physical cause.

The covariance matrix encodes this:
$$\Sigma = \begin{bmatrix}
\sigma^2_x & \sigma_{xy} \\
\sigma_{xy} & \sigma^2_y
\end{bmatrix}$$

- Diagonal: variance in each dimension
- Off-diagonal σ_xy: correlation between x and y
- σ_xy=0: independent (no correlation)
- σ_xy large positive: move together

**Why symmetric:** σ_xy = σ_yx because "how x and y 
vary together" is one physical relationship.
If your covariance matrix is not symmetric — you have a bug.

**For AMR state (x, y, θ):**
Covariance matrix is 3×3 with 6 unique values:
$$\Sigma = \begin{bmatrix}
\sigma^2_x & \sigma_{xy} & \sigma_{x\theta} \\
\sigma_{xy} & \sigma^2_y & \sigma_{y\theta} \\
\sigma_{x\theta} & \sigma_{y\theta} & \sigma^2_\theta
\end{bmatrix}$$

In ROS2, full 3D pose covariance is 6×6 = 36 values
(x, y, z, roll, pitch, yaw).
For ground robot, most z/roll/pitch terms are zero.

---

## CHAPTER 2 — The Bayes Filter

### The Two Fundamental Sources of Uncertainty
(From Probabilistic Robotics, Thrun et al.)

1. **Partial observability:** Sensors never give complete,
   perfect information. Limited range, noise, occlusion.

2. **Unpredictable actions:** Motion commands never execute
   perfectly. Wheel slip, terrain, motor variance.

Together: a robot can never know its exact state.
It can only maintain a **belief** — a probability 
distribution over all possible states.

### What Is a Belief?
Belief is a conditional probability:
$$bel(x_t) = p(x_t | z_{1:t}, u_{1:t})$$

The probability of being at state x at time t,
given ALL past measurements z and ALL past 
motion commands u up to time t.

Everything is timestamped — this is why TF2 
requires timestamps on every transform.

### The Bayes Filter Algorithm
Two steps, repeated forever:

**PREDICT (motion):**
$$\bar{bel}(x_t) = \int p(x_t|u_t,x_{t-1})
\cdot bel(x_{t-1})dx_{t-1}$$

Apply motion model to current belief.
Uncertainty grows. Gaussian widens.

**UPDATE (measurement):**
$$bel(x_t) = \eta \cdot p(z_t|x_t) 
\cdot \bar{bel}(x_t)$$

Multiply belief by likelihood of measurement.
Normalize so probabilities sum to 1.
Uncertainty shrinks. Gaussian narrows.

η = normalizer = 1/sum (makes everything sum to 1)

### Bayes Filter — Discrete Example (The Corridor)

**Setup:**
Three positions: A, B, C.
Prior (no information): P(A)=P(B)=P(C)=1/3

Sensor detects "door":
- A: real door, P(detect|A) = 1.0
- B: fake door, P(detect|B) = 0.5
- C: real door, P(detect|C) = 1.0

**UPDATE step:**
Multiply prior × likelihood:
- A: 1/3 × 1.0 = 0.333
- B: 1/3 × 0.5 = 0.167
- C: 1/3 × 1.0 = 0.333
Sum = 0.833 ← must normalize

Normalize (÷ 0.833):
- P(A) = 0.4
- P(B) = 0.2
- P(C) = 0.4

**PREDICT step:**
Robot moves forward one step with noise:
- 70%: moved one step forward
- 20%: stayed in place
- 10%: moved two steps forward

Redistribute from each position:
From A(0.4): stays→A=0.08, →B=0.28, →C=0.04
From B(0.2): stays→B=0.04, →C=0.14
From C(0.4): stays→C=0.08 (rest lost at wall)

Collect:
- P(A) = 0.08
- P(B) = 0.32
- P(C) = 0.26
Uncertainty increased. Belief spread out.

### Key Insight
Bayes Filter is the foundation of ALL probabilistic
robotics algorithms:
- AMCL → particle filter implementation of Bayes Filter
- Kalman Filter → Gaussian implementation of Bayes Filter
- EKF → nonlinear Gaussian implementation
- GraphSLAM → batch version of Bayes Filter

---

## CHAPTER 3 — The Kalman Filter

### Why Kalman Filter
Bayes Filter is general but expensive.
For continuous state spaces (real robots),
integrating over all possible states is intractable.

The Kalman Filter is a Bayes Filter with 3 assumptions:
1. All distributions are Gaussian
2. Motion model is linear: x' = Ax + Bu
3. Sensor model is linear: z = Hx

Under these assumptions: exact, efficient,
closed-form solution exists.

### The Five KF Equations

**PREDICT — two equations:**

$$\bar{\mu}_t = A_t\mu_{t-1} + B_tu_t$$
New mean = motion model × old mean + control effect.

$$\bar{\Sigma}_t = A_t\Sigma_{t-1}A_t^T + R_t$$
New covariance = propagated old covariance + motion noise.
Uncertainty always grows during prediction.

**KALMAN GAIN — one equation:**

$$K_t = \bar{\Sigma}_t H_t^T
(H_t\bar{\Sigma}_t H_t^T + Q_t)^{-1}$$

Decides how much to trust measurement vs prediction.
Computed before update equations.

**UPDATE — two equations:**

$$\mu_t = \bar{\mu}_t + K_t(z_t - H_t\bar{\mu}_t)$$
New mean = predicted mean + 
Kalman Gain × innovation.

$$\Sigma_t = (I - K_tH_t)\bar{\Sigma}_t$$
New covariance = always smaller than predicted.
Uncertainty always shrinks during update.

### Complete Symbol Reference

| Symbol | What It Is | Physical Meaning | In Your AMR |
|---|---|---|---|
| μ | Mean vector | Best state estimate | Robot pose (x,y,θ) |
| Σ | Covariance matrix | Uncertainty | 3×3 pose uncertainty |
| ū, Σ̄ | Predicted mean/cov | After motion, before measurement | Between predict and update |
| u_t | Control input | Motion command | Wheel velocity |
| z_t | Measurement | Sensor reading | IMU/LiDAR data |
| A_t | Motion model matrix | How state evolves | Pose transition |
| B_t | Control matrix | How command affects state | Velocity→pose |
| H_t | Measurement model | Maps state to sensor space | Pose→expected reading |
| R_t | Motion noise covariance | Encoder/motor uncertainty | Set in robot_localization config |
| Q_t | Sensor noise covariance | Sensor measurement noise | Set in robot_localization config |
| K_t | Kalman Gain | Trust balance weight | Computed internally |
| I | Identity matrix | — | Used in covariance update |
| η | Normalizer | Makes probabilities sum to 1 | Built into K equation |

### Understanding H — The Measurement Model

H maps the state vector to what the sensor measures.

**H = 1 (scalar):**
Sensor directly measures state.
Example: GPS measures position directly.
z = x (measurement = state)

**H = matrix:**
Sensor measures something derived from state.
Example: IMU measures acceleration, not position.

For state (x, y, θ) and IMU measuring 
x-acceleration only:
$$H = \begin{bmatrix} 0 & 0 & 0 & 0 & 0 & 
0 & 0 & 0 & 0 & 0 & 0 & 0 & 1 & 0 & 0\end{bmatrix}$$

H selects the x-acceleration element from 
the full 15-element state vector.

**Why 15 states?**
To make H linear for IMU, state must include 
position, velocity, AND acceleration:
(x, y, z, φ, θ, ψ, ẋ, ẏ, ż, φ̇, θ̇, ψ̇, ẍ, ÿ, z̈)

This is the robot_localization state vector.

**H is nonlinear:**
LiDAR measures distance to walls.
Converting robot pose to expected LiDAR distance 
requires ray casting — inherently nonlinear.
H must be replaced by its Jacobian (same as A→J).

### Understanding the Kalman Gain

$$K_t = \bar{\Sigma}_t H_t^T
(H_t\bar{\Sigma}_t H_t^T + Q_t)^{-1}$$

The innovation:
$$(z_t - H_t\bar{\mu}_t)$$
= actual measurement − predicted measurement
= how wrong the prediction was

K_t scales the correction:

| Condition | K_t | Behavior |
|---|---|---|
| Q_t large (noisy sensor) | Small | Trust prediction more |
| Q_t small (accurate sensor) | Large | Trust measurement more |
| Σ̄_t large (uncertain prediction) | Large | Trust measurement more |
| Σ̄_t small (confident prediction) | Small | Trust prediction more |
| Q_t → ∞ | → 0 | Ignore measurement completely |
| Q_t → 0 | → H⁻¹ | Trust measurement completely |

**In 1D, Kalman Gain simplifies to:**
$$K = \frac{\bar{\sigma}^2}{\bar{\sigma}^2 + Q}$$

### Variance Behavior Over Time
- Predict: variance grows by R_t every step
- Update: variance shrinks by factor (1-K_t)
- Over time: reaches steady state equilibrium
- Equilibrium = where growth and shrinkage balance
- Determined by ratio of R_t to Q_t
- Filter self-regulates — no manual tuning at runtime

### Numerical Example — Full KF Cycle

**Initial:** μ=0.0, σ²=0.1
**Motion:** u=2.0m, R=0.3
**Sensor:** z₁=2.1m, Q=0.2, H=1

**Timestep 1 — Predict:**
$$\bar{\mu}_1 = 0.0 + 2.0 = 2.0$$
$$\bar{\sigma}^2_1 = 0.1 + 0.3 = 0.4$$

**Timestep 1 — Kalman Gain:**
$$K_1 = \frac{0.4}{0.4+0.2} = 0.667$$

**Timestep 1 — Update:**
Innovation = 2.1 - 2.0 = 0.1
$$\mu_1 = 2.0 + 0.667(0.1) = 2.067$$
$$\sigma^2_1 = (1-0.667)(0.4) = 0.133$$

**Timestep 2 — Predict (z₂=4.3):**
$$\bar{\mu}_2 = 2.067 + 2.0 = 4.067$$
$$\bar{\sigma}^2_2 = 0.133 + 0.3 = 0.433$$

**Timestep 2 — Kalman Gain:**
$$K_2 = \frac{0.433}{0.433+0.2} = 0.684$$

**Timestep 2 — Update:**
Innovation = 4.3 - 4.067 = 0.233
$$\mu_2 = 4.067 + 0.684(0.233) = 4.226$$
$$\sigma^2_2 = (1-0.684)(0.433) = 0.137$$

### Critical Verification — KF = Sensor Fusion
Session 1 fusion formula at timestep 1:
$$\mu_c = \frac{0.2(2.0)+0.4(2.1)}{0.6} = 2.067 ✓$$
$$\sigma^2_c = \frac{0.4 \times 0.2}{0.6} = 0.133 ✓$$

**Kalman Filter update IS Gaussian sensor fusion.**
Same mathematics. Different notation.

---

## CHAPTER 4 — The Extended Kalman Filter

### Why KF Fails for Real Robots

Standard KF requires:
$$\mathbf{x'} = A\mathbf{x} + B\mathbf{u}$$

Real differential drive motion model:
$$x' = x + d\cos\theta$$
$$y' = y + d\sin\theta$$
$$\theta' = \theta$$

Contains sin(θ) and cos(θ).
These are nonlinear — cannot be written as Ax+Bu.
Standard KF breaks.

### The Solution — The Jacobian Matrix

The Jacobian linearizes a nonlinear function 
at a specific point.

It is a matrix of ALL partial derivatives:
"If state changes by tiny amount, 
how much does output change?"

### Computing the Motion Model Jacobian

For the differential drive motion model:

$$J = \frac{\partial f}{\partial \mathbf{x}} =
\begin{bmatrix}
\frac{\partial x'}{\partial x} &
\frac{\partial x'}{\partial y} &
\frac{\partial x'}{\partial \theta} \\
\frac{\partial y'}{\partial x} &
\frac{\partial y'}{\partial y} &
\frac{\partial y'}{\partial \theta} \\
\frac{\partial \theta'}{\partial x} &
\frac{\partial \theta'}{\partial y} &
\frac{\partial \theta'}{\partial \theta}
\end{bmatrix}$$

Computing each entry:
- ∂x'/∂x = 1 (x'=x+constant)
- ∂x'/∂y = 0 (x' doesn't depend on y)
- ∂x'/∂θ = -d·sinθ
- ∂y'/∂x = 0
- ∂y'/∂y = 1
- ∂y'/∂θ = d·cosθ
- ∂θ'/∂x = 0
- ∂θ'/∂y = 0
- ∂θ'/∂θ = 1

$$J = \begin{bmatrix}
1 & 0 & -d\sin\theta \\
0 & 1 & d\cos\theta \\
0 & 0 & 1
\end{bmatrix}$$

**Critical:** J changes with θ at every timestep.
EKF recomputes J continuously using current θ.

### KF vs EKF Comparison

| | Kalman Filter | Extended Kalman Filter |
|---|---|---|
| Motion model | Linear: x'=Ax+Bu | Nonlinear: x'=f(x,u) |
| Mean prediction | Aμ+Bu | f(μ,u) directly |
| Covariance predict | AΣAᵀ+R | JΣJᵀ+R |
| A matrix | Fixed, constant | Replaced by J, recomputed every step |
| Measurement model | Linear: z=Hx | Can be nonlinear: z=h(x) |
| H matrix | Fixed | May be replaced by Jacobian |
| Valid for | Linear systems only | Nonlinear systems |

### EKF Equations

**Predict:**
$$\bar{\mu}_t = f(\mu_{t-1}, u_t)$$
$$\bar{\Sigma}_t = J_t\Sigma_{t-1}J_t^T + R_t$$

**Update (same as KF if H is linear):**
$$K_t = \bar{\Sigma}_t H^T(H\bar{\Sigma}_t H^T+Q_t)^{-1}$$
$$\mu_t = \bar{\mu}_t + K_t(z_t - H\bar{\mu}_t)$$
$$\Sigma_t = (I-K_tH)\bar{\Sigma}_t$$

### Numerical Example — EKF Predict

**Initial:**
$$\mu=\begin{bmatrix}2.0\\1.0\\45°\end{bmatrix},
\Sigma=\begin{bmatrix}0.1&0&0\\0&0.1&0\\0&0&0.05
\end{bmatrix}$$

**Motion:** d=1.0m, 
$$R=\begin{bmatrix}0.1&0&0\\0&0.1&0\\0&0&0.01
\end{bmatrix}$$

**Step 1 — New mean (nonlinear model directly):**
$$\bar{\mu}=\begin{bmatrix}
2.0+1.0\cos45°\\1.0+1.0\sin45°\\45°
\end{bmatrix}=\begin{bmatrix}2.707\\1.707\\45°
\end{bmatrix}$$

**Step 2 — Jacobian at θ=45°, d=1.0:**
$$J=\begin{bmatrix}
1&0&-0.707\\0&1&0.707\\0&0&1
\end{bmatrix}$$

**Step 3 — JΣ:**
$$J\Sigma=\begin{bmatrix}
0.1&0&-0.03535\\0&0.1&0.03535\\0&0&0.05
\end{bmatrix}$$

**Step 4 — JΣJᵀ:**
$$J\Sigma J^T=\begin{bmatrix}
0.1249&-0.02499&-0.03535\\
-0.02499&0.1249&0.03535\\
-0.03535&0.03535&0.05
\end{bmatrix}$$

**Step 5 — Add R:**
$$\bar{\Sigma}=\begin{bmatrix}
0.2249&-0.02499&-0.03535\\
-0.02499&0.2249&0.03535\\
-0.03535&0.03535&0.06
\end{bmatrix}$$

**What the result shows:**
- Diagonal grew: motion increased uncertainty
- Off-diagonal appeared: x,y,θ became correlated
  because moving at 45° links position and heading errors
- Matrix symmetric: sanity check passes ✓

### The Measurement Model for LiDAR
LiDAR measures distances to walls — nonlinear.
Requires ray casting: given pose (x,y,θ), 
cast ray in direction θ, find wall intersection,
compute expected distance.

This function h(x) is nonlinear.
H is replaced by its Jacobian H_t.
Same concept as A→J for the motion model.

---

## CHAPTER 5 — The Complete Picture

### How Everything Connects


GAUSSIAN (μ, σ²)

    ↓ represents

ROBOT BELIEF

    ↓ updated by

BAYES FILTER (predict + update)

    ↓ Gaussian special case

KALMAN FILTER (linear)

    ↓ nonlinear extension  

EXTENDED KALMAN FILTER

     ↓ runs inside

ROBOT_LOCALIZATION (your AMR)


### Sensor Fusion = KF Update
Session 1 fusion formula and KF update equation
produce identical results — verified numerically.
They are the same mathematical operation.

### Where This Lives in Your AMR

| Concept | Where in AMR |
|---|---|
| R_t matrix | Motion noise in robot_localization config |
| Q_t matrix | Sensor noise in robot_localization config |
| EKF predict | Triggered by /odom topic |
| EKF update | Triggered by /imu topic |
| Published result | /odometry/filtered topic |
| Covariance output | pose.covariance field (36 values, 6×6) |

### Floating Point Rule (From Sprint 1)
Never use == with doubles.
Always use epsilon comparison:
```cpp
std::abs(value - expected) < 1e-9
```
Applies to all covariance matrix checks.

---

## Common Mistakes — Master List

1. Thinking robot state is a single point
2. Adding translations without accounting for rotation
3. Thinking combined variance is average of both —
   it is always SMALLER than both
4. Thinking each mean weighted by its OWN variance —
   it is weighted by the OTHER sensor's variance
5. Forgetting motion always grows variance
6. Forgetting update always shrinks variance
7. Setting Q_t=0 or R_t=0 — filter diverges
8. Thinking K_t is constant — recomputed every step
9. Using standard KF for nonlinear robot — use EKF
10. Forgetting to recompute J at every timestep
11. Using J from wrong θ — must use current estimate
12. Non-symmetric covariance matrix — always a bug
13. Thinking H=1 always — only when sensor directly
    measures state
14. Forgetting normalizer η in Bayes Filter update

---

## Sprint 2 Interview Questions

**Q1 — Conceptual:**
What are the two steps of the Bayes Filter?
What happens to uncertainty in each step and why?

**Q2 — Numerical:**
Encoder variance σ²=0.5, LiDAR variance Q=0.1.
After predict, covariance=0.6, mean=5.0.
LiDAR reads z=5.3.
Calculate: K, updated mean, updated variance.

**Q3 — Conceptual:**
Why does the standard Kalman Filter fail for 
a differential drive robot?
What specifically makes the motion model nonlinear?

**Q4 — Conceptual:**
What is the Jacobian?
Why must EKF recompute it at every timestep?

**Q5 — Mathematical:**
Robot at θ=0°, d=1m. Write the Jacobian.
Robot at θ=90°, d=1m. Write the Jacobian.
What changed and why does it matter physically?

**Q6 — Analytical:**
After EKF predict at 45°, off-diagonal covariance 
terms appear even though initial Σ was diagonal.
Why? What do these terms represent physically?