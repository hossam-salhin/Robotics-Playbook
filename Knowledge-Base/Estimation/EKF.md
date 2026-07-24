# Sprint 2 — Session 3 Complete Review
# Extended Kalman Filter (EKF)

---

## Why Standard KF Fails for Real Robots

Standard Kalman Filter requires linear motion model:
$$\mathbf{x'} = A\mathbf{x} + B\mathbf{u}$$

Real differential drive robot motion model:
$$x' = x + d\cos\theta$$
$$y' = y + d\sin\theta$$
$$\theta' = \theta$$

Contains sin(θ) and cos(θ) — nonlinear.
Cannot be written as Ax + Bu.
Standard KF breaks.

---

## The Solution — Linearization via Jacobian

The Jacobian is a matrix of partial derivatives.
It linearizes a nonlinear function at a specific point.

For robotics: it answers
"If state changes by a tiny amount, 
how much does the output change?"

### Computing the Motion Model Jacobian

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

Substituting the motion model:
$$J = \begin{bmatrix} 
1 & 0 & -d\sin\theta \\ 
0 & 1 & d\cos\theta \\ 
0 & 0 & 1 
\end{bmatrix}$$

**Critical property:** J changes with θ at every timestep.
This is why EKF recomputes J continuously.

---

## EKF vs KF — The Key Difference

| | Kalman Filter | Extended Kalman Filter |
|---|---|---|
| Motion model | Linear: x'=Ax+Bu | Nonlinear: x'=f(x,u) |
| Covariance propagation | AΣAᵀ+R | JΣJᵀ+R |
| A matrix | Fixed, constant | Replaced by J, recomputed every step |
| Mean prediction | Aμ+Bu | f(μ,u) directly |
| Valid for | Linear systems | Nonlinear systems |

---

## EKF Equations

**Predict Step:**
$$\bar{\mu}_t = f(\mu_{t-1}, u_t)$$
Apply nonlinear motion model directly to mean.

$$\bar{\Sigma}_t = J_t\Sigma_{t-1}J_t^T + R_t$$
Propagate covariance using Jacobian instead of A.

**Update Step (same as KF if sensor model is linear):**
$$K_t = \bar{\Sigma}_t H^T(H\bar{\Sigma}_t H^T + Q_t)^{-1}$$
$$\mu_t = \bar{\mu}_t + K_t(z_t - H\bar{\mu}_t)$$
$$\Sigma_t = (I - K_tH)\bar{\Sigma}_t$$

If sensor model is also nonlinear:
H is replaced by its own Jacobian H_t,
computed at current state estimate.

---

## Numerical Example — Full EKF Predict

**Initial state:**
$$\mu = \begin{bmatrix} 2.0 \\ 1.0 \\ 45° \end{bmatrix}, \quad
\Sigma = \begin{bmatrix} 0.1 & 0 & 0 \\ 
0 & 0.1 & 0 \\ 0 & 0 & 0.05 \end{bmatrix}$$

**Motion:** d=1.0m forward, 
$$R = \begin{bmatrix} 0.1&0&0\\0&0.1&0\\0&0&0.01\end{bmatrix}$$

**Step 1 — New mean (nonlinear model):**
$$\bar{\mu} = \begin{bmatrix} 
2.0 + 1.0\cos45° \\ 
1.0 + 1.0\sin45° \\ 
45° 
\end{bmatrix} = \begin{bmatrix} 2.707 \\ 1.707 \\ 45° \end{bmatrix}$$

**Step 2 — Jacobian at θ=45°:**
$$J = \begin{bmatrix} 
1 & 0 & -0.707 \\ 
0 & 1 & 0.707 \\ 
0 & 0 & 1 
\end{bmatrix}$$

**Step 3 — New covariance:**
$$J\Sigma = \begin{bmatrix} 
0.1 & 0 & -0.03535 \\ 
0 & 0.1 & 0.03535 \\ 
0 & 0 & 0.05 
\end{bmatrix}$$

$$J\Sigma J^T = \begin{bmatrix} 
0.1249 & -0.02499 & -0.03535 \\ 
-0.02499 & 0.1249 & 0.03535 \\ 
-0.03535 & 0.03535 & 0.05 
\end{bmatrix}$$

$$\bar{\Sigma} = J\Sigma J^T + R = \begin{bmatrix} 
0.2249 & -0.02499 & -0.03535 \\ 
-0.02499 & 0.2249 & 0.03535 \\ 
-0.03535 & 0.03535 & 0.06 
\end{bmatrix}$$

---

## What the Result Tells Us

**Diagonal grew:** Motion increased uncertainty.
- σ²_x: 0.1 → 0.2249
- σ²_y: 0.1 → 0.2249  
- σ²_θ: 0.05 → 0.06

**Off-diagonal appeared:** x,y,θ became correlated.
Moving at 45° means position error and heading error 
are now linked — exactly as discussed in Session 1.

**Matrix is symmetric:** σ_xy = σ_yx. Sanity check passes.

---

## Connection to Your AMR

robot_localization runs this exact computation:
- Every odometry message triggers predict step
- J recomputed at current (x,y,θ) estimate
- Covariance propagated through J
- Every IMU/LiDAR message triggers update step
- K_t computed, mean and covariance corrected

The covariance values you set in robot_localization 
config are R_t (motion noise) and Q_t (sensor noise).
Now you understand what they actually control.

---

## Common Mistakes

1. Using KF for nonlinear systems — always use EKF
2. Forgetting to recompute J at every timestep —
   J must use current θ, not initial θ
3. Confusing f(x,u) with J — 
   f gives new mean, J propagates covariance
4. Forgetting to add R after JΣJᵀ
5. Not checking symmetry of result covariance

---

## Connection Map

Session 1          Session 2          Session 3
─────────          ─────────          ─────────
Gaussian      →    Bayes Filter   →   EKF
σ², Σ         →    Predict/Update →   JΣJᵀ+R
Sensor fusion →    KF update      →   Same equations
Covariance    →    R_t, Q_t       →   Config params

---

## One Question Before Sprint 3

Your EKF predict step is now complete.
The update step uses H — the measurement model.

Your AMR's LiDAR doesn't measure (x,y,θ) directly.
It measures distances to walls and features.

What do you think the measurement model H 
(or its Jacobian) needs to encode for a LiDAR 
in a known map?