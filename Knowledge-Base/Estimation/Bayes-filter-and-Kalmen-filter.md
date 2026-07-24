# Sprint 2 — Session 2 Complete Review
# Bayes Filter and Kalman Filter

---

## The Journey of This Session

### Where We Started
Session 1 ended with one question:
"How do you represent changing uncertainty mathematically?"

You answered: something like a constraint matrix.
The correct answer: a Gaussian distribution — 
defined by mean and covariance.

Session 2 built everything on top of that foundation.

---

## Part 1 — The Corridor Problem (Bayes Filter Intuition)

### The Setup
A robot is in a corridor with three positions: A, B, C.
At startup: complete ignorance.
P(A) = P(B) = P(C) = 1/3

Sensor detects "door":
- Door A: real door, detected 100% of the time
- Door B: fake door, detected 50% of the time  
- Door C: real door, detected 100% of the time

### The Update Step — Applying Bayes Rule
For each position, ask:
"How likely is this sensor reading if I were here?"

This is called the **likelihood**: P(measurement | position)
- P(detect door | A) = 1.0
- P(detect door | B) = 0.5
- P(detect door | C) = 1.0

Multiply prior × likelihood:
- A: 1/3 × 1.0 = 0.333
- B: 1/3 × 0.5 = 0.167
- C: 1/3 × 1.0 = 0.333
- Sum = 0.833 ← does not sum to 1

**Normalize** — divide each by the sum:
- P(A) = 0.333/0.833 = 0.4
- P(B) = 0.167/0.833 = 0.2
- P(C) = 0.333/0.833 = 0.4
- Sum = 1.0 ✓

**What happened:** One sensor reading shifted belief 
away from the fake door toward the real doors.
This is the UPDATE step of the Bayes Filter.

### The Predict Step — Motion
Robot moves forward one step with noise:
- 70% moved exactly one step forward
- 20% stayed in place
- 10% moved two steps forward

Starting from P(A)=0.4, P(B)=0.2, P(C)=0.4,
redistribute probability mass:

From A (0.4):
- Stays at A: 0.4 × 0.2 = 0.08
- Moves to B: 0.4 × 0.7 = 0.28
- Moves to C: 0.4 × 0.1 = 0.04

From B (0.2):
- Stays at B: 0.2 × 0.2 = 0.04
- Moves to C: 0.2 × 0.7 = 0.14
- Moves beyond C: 0.2 × 0.1 = 0 (wall)

From C (0.4):
- Stays at C: 0.4 × 0.2 = 0.08
- Moves beyond C: 0.4 × 0.7 = 0 (wall — lost)
- Moves two beyond C: 0.4 × 0.1 = 0 (wall — lost)

Collect at each position:
- P(A) = 0.08
- P(B) = 0.28 + 0.04 = 0.32
- P(C) = 0.04 + 0.14 + 0.08 = 0.26
- Sum = 0.66 (0.34 lost at wall — real behavior 
  requires proper map boundaries)

**What happened:** Motion spread the probability mass.
Uncertainty increased. The Gaussian got wider.
This is the PREDICT step of the Bayes Filter.

### The Bayes Filter Formal Equations

**Predict:**
$$\bar{bel}(x_t) = \int p(x_t|u_t,x_{t-1}) 
\cdot bel(x_{t-1})dx_{t-1}$$

**Update:**
$$bel(x_t) = \eta \cdot p(z_t|x_t) \cdot \bar{bel}(x_t)$$

Where η is the normalizer (1/sum).

Every probabilistic robotics algorithm —
AMCL, EKF, particle filter — is this same 
two-step cycle implemented differently.

---

## Part 2 — The Kalman Filter

### Why Kalman Filter
The Bayes Filter is general but computationally expensive 
for continuous state spaces.

The Kalman Filter is a special case that assumes:
1. All distributions are Gaussian
2. Motion model is linear
3. Sensor model is linear

Under these assumptions, the Bayes Filter has 
an exact, efficient closed-form solution.

### The Five Equations

**PREDICT STEP — uncertainty grows:**

$$\bar{\mu}_t = A_t\mu_{t-1} + B_tu_t$$
New mean = motion model applied to old mean
+ effect of control command

$$\bar{\Sigma}_t = A_t\Sigma_{t-1}A_t^T + R_t$$
New covariance = propagated old covariance 
+ motion noise R_t
Uncertainty always grows during prediction.

**UPDATE STEP — uncertainty shrinks:**

$$K_t = \bar{\Sigma}_t H_t^T
(H_t\bar{\Sigma}_t H_t^T + Q_t)^{-1}$$
Kalman Gain — how much to trust measurement vs prediction

$$\mu_t = \bar{\mu}_t + K_t(z_t - H_t\bar{\mu}_t)$$
Updated mean = predicted mean + 
Kalman Gain × innovation

$$\Sigma_t = (I - K_tH_t)\bar{\Sigma}_t$$
Updated covariance — always smaller than predicted.
Uncertainty shrinks after every measurement.

### Symbol Reference Table

| Symbol | Physical Meaning | In Your AMR |
|---|---|---|
| μ | Mean — best state estimate | Robot pose (x,y,θ) |
| Σ | Covariance — uncertainty | 3×3 pose uncertainty |
| u_t | Control input | Wheel velocity command |
| z_t | Measurement | LiDAR scan / IMU reading |
| A_t | Motion model matrix | How pose evolves |
| B_t | Control matrix | How command affects pose |
| H_t | Measurement model | Maps state to sensor space |
| R_t | Motion noise covariance | Encoder drift |
| Q_t | Sensor noise covariance | LiDAR/IMU noise |
| K_t | Kalman Gain | Trust balance |

### The Kalman Gain — Deep Understanding

$$K_t = \bar{\Sigma}_t H_t^T
(H_t\bar{\Sigma}_t H_t^T + Q_t)^{-1}$$

The innovation:
$$(z_t - H_t\bar{\mu}_t)$$
= what sensor actually measured − what you predicted 
it would measure

K_t scales how much of this innovation you apply:

| Condition | Effect on K_t | Filter Behavior |
|---|---|---|
| Q_t large (noisy sensor) | K_t small | Trust prediction |
| Q_t small (accurate sensor) | K_t large | Trust measurement |
| Σ̄_t large (uncertain prediction) | K_t large | Trust measurement |
| Σ̄_t small (confident prediction) | K_t small | Trust prediction |

**Extreme cases:**
- Q_t → ∞: K_t → 0, filter ignores measurement entirely
- Q_t → 0: K_t → H_t⁻¹, filter trusts measurement completely
- Σ̄_t → 0: K_t → 0, filter trusts prediction completely

### Numerical Verification
Motion model: μ̄ = 10.0, σ̄² = 0.9
Measurement: z = 10.2, Q = 0.1, H = 1

$$K = \frac{0.9}{0.9+0.1} = 0.9$$

$$\mu = 10.0 + 0.9(10.2-10.0) = 10.18$$

$$\sigma^2 = (1-0.9) \times 0.9 = 0.09$$

**Critical verification:**
Session 1 sensor fusion formula gives identical result:
$$\mu_c = \frac{0.1(10.0)+0.9(10.2)}{0.9+0.1} = 10.18$$
$$\sigma^2_c = \frac{0.9 \times 0.1}{0.9+0.1} = 0.09$$

**Conclusion: Kalman Filter update = Gaussian sensor 
fusion. Same mathematics, different notation.**

### Variance Behavior Over Time
- Each predict step: variance grows by R_t
- Each update step: variance shrinks by factor (1-K_t)
- Over time: reaches steady state equilibrium
- Equilibrium determined by ratio of R_t to Q_t
- Filter self-regulates — no manual tuning at runtime

---

## Part 3 — The Linearity Problem

### Why Standard KF Fails for Real Robots
Standard KF requires linear motion model: x' = Ax + Bu

Your AMR differential drive motion model:
$$x' = x + d\cos\theta$$
$$y' = y + d\sin\theta$$
$$\theta' = \theta + \Delta\theta$$

Contains sin(θ) and cos(θ) — nonlinear functions of state.

Cannot be written as x' = Ax + Bu.
Standard Kalman Filter breaks.

**Solution: Extended Kalman Filter (Session 3)**
Linearizes the nonlinear model at each timestep 
using the Jacobian matrix.

---

## Connection Map

Session 1                    Session 2
─────────                    ─────────
Gaussian (μ, σ²)     ──>     Bayes Filter predict/update
Sensor fusion        ──>     Kalman Filter update step
Covariance matrix    ──>     Σ in all KF equations
“Trust by variance”  ──>     Kalman Gain K_t

Session 2                    Real Systems
─────────                    ────────────
Bayes Filter         ──>     AMCL (particle filter)
Kalman Filter        ──>     robot_localization (EKF)
R_t, Q_t matrices    ──>     Config params in your AMR
Innovation term      ──>    Residual in EKF debug output

---

## Common Mistakes — Complete List

1. Forgetting to normalize after applying likelihood
2. Thinking Kalman Filter and sensor fusion are different
3. Predicting increases uncertainty — never decreases it
4. Update decreases uncertainty — never increases it
5. Large Q_t means trust sensor less, not more
6. Setting R_t or Q_t to zero — filter diverges
7. Applying standard KF to nonlinear system — use EKF
8. Thinking K_t is constant — it changes every timestep
9. Ignoring off-diagonal covariance terms — 
   they encode real physical correlations
10. Not verifying covariance matrix is symmetric — 
    asymmetry always means a bug

---

## One Question Before Session 3

Your AMR motion model contains cos(θ) and sin(θ).
These are nonlinear. The EKF linearizes them using 
the Jacobian matrix.

The Jacobian is a matrix of partial derivatives.
For your motion model, it answers:

"If I change x by a tiny amount, how much does x' change?
If I change θ by a tiny amount, how much does x' change?"

Before Session 3:
Compute the partial derivative of x' = x + d·cos(θ) 
with respect to θ.

What do you get?