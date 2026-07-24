# Sprint 2 Review — Probabilistic State Estimation

## What I Learned

### Session 1 — Gaussian Distributions and Sensor Fusion
- Robot state is a probability distribution, not a point
- Gaussian defined by mean (best estimate) and 
  variance (confidence)
- Motion adds uncertainty: σ²_new = σ²_old + σ²_motion
- Sensor fusion combines two Gaussians into one
- More certain sensor pulls combined mean toward itself
- Combined variance always smaller than both inputs
- Covariance matrix encodes correlated uncertainty in 2D/3D
- Symmetric by definition — asymmetry means bug

### Session 2 — Bayes Filter and Kalman Filter
- Bayes Filter: universal predict-update cycle
- Predict: uncertainty grows (Gaussian widens)
- Update: uncertainty shrinks (Gaussian narrows)
- Kalman Filter: Bayes Filter assuming Gaussians + linearity
- Five equations: 2 predict, 1 Kalman Gain, 2 update
- Kalman Gain automatically balances trust between 
  prediction and measurement
- KF update = Gaussian sensor fusion (verified numerically)
- Variance reaches steady state equilibrium over time

### Session 3 — Extended Kalman Filter
- Real robot motion model is nonlinear (sin, cos of θ)
- Standard KF fails — A matrix cannot represent nonlinearity
- Solution: Jacobian matrix linearizes at current state
- EKF recomputes Jacobian at every timestep
- Predict mean: apply f(x,u) directly
- Predict covariance: JΣJᵀ + R instead of AΣAᵀ + R
- Off-diagonal covariance terms appear after motion at angle
- Measurement model H maps state to sensor reading
- Nonlinear H also replaced by its Jacobian

## What Changed in My Understanding
- Sensor fusion was intuitive before Sprint 2 —
  now I understand the mathematical mechanism
- robot_localization config parameters (R, Q matrices)
  now have clear physical meaning
- The connection between Bayes Filter, KF, and EKF
  is now one continuous idea, not three separate topics
- Covariance is not just "uncertainty" —
  it encodes directional and correlated uncertainty

## Remaining Weaknesses
- EKF update step not yet computed numerically
- Measurement Jacobian not yet derived
- Particle filter not yet covered
- AMCL internals not yet connected to Bayes Filter

## Connections Between Topics

Gaussian (Session 1)

↓

Sensor Fusion (Session 1)

↓ same math

Bayes Filter Update (Session 2)

↓ Gaussian special case

Kalman Filter (Session 2)

↓ nonlinear extension

Extended Kalman Filter (Session 3)

↓ nonlinear measurement

Measurement Jacobian → Sprint 3


## Relationship to AMR Project
- robot_localization runs EKF predict at every 
  odometry message
- robot_localization runs EKF update at every 
  IMU message
- R matrix = encoder drift variance (set in config)
- Q matrix = IMU noise variance (set in config)
- Covariance published on /odometry/filtered topic
- AMCL kidnapping bug connects to Bayes Filter 
  recovery behavior — Sprint 3

## Relationship to Robo Core
- SensorRegistry runs sensors in separate threads
- Sensor fusion belongs as a module in Robo Core
- EKF belongs as a class in Coding Lab Sprint 2

## Sprint 2 Interview Questions

**Q1:** What are the two steps of the Bayes Filter?
What happens to uncertainty in each step?

**Q2:** A robot has encoder variance R=0.5 and 
LiDAR variance Q=0.1. After the predict step,
covariance is 0.6. Calculate K, updated mean 
(predicted=5.0, measurement=5.3), updated variance.

**Q3:** Why does the standard Kalman Filter fail 
for a differential drive robot? What specifically 
breaks?

**Q4:** What is the Jacobian and why does the EKF 
need to recompute it at every timestep?

**Q5:** Your robot moves at θ=0° with d=1m.
Write the Jacobian. Now at θ=90°. 
What changed and why does it matter?

**Q6:** After an EKF predict step at 45°, why do 
off-diagonal covariance terms appear even if they 
were zero before?

## What Must Be Done Before Sprint 3
1. Answer Sprint 2 interview questions
2. Implement EKF in Coding Lab Sprint 2
3. Read Probabilistic Robotics chapter 2 fully
4. Connect robot_localization config to R and Q matrices
   in your AMR launch files