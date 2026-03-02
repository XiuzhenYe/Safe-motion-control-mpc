# Safe Motion Control with MPC (CBF-Constrained)

Real-time safe motion control using Model Predictive Control (MPC) with Control Barrier Function (CBF) safety constraints for robotic obstacle avoidance.

This project demonstrates how nonlinear safety constraints can be integrated into a convex MPC framework using Sequential Convex Programming (SCP), enabling real-time, safety-aware motion control for robotics.

---

## 🚀 Overview

We consider a 2D double-integrator robot:

\[
\dot q = v, \quad \dot v = u
\]

The objective is to drive the robot to a goal position while avoiding circular obstacles.

Obstacle avoidance constraints are inherently nonconvex:

\[
\|q - q_o\| \ge r
\]

To enable real-time optimization with convex solvers, the problem is solved using:

- Control Barrier Functions (CBF)
- Sequential Convex Programming (SCP)
- Convex Quadratic Programming (CVXPY + OSQP)

---

## 🧠 Key Idea

Instead of solving a fully nonlinear MPC problem directly:

1. Linearize the CBF constraint around a reference trajectory.
2. Solve a convex MPC problem (QP).
3. Update the reference trajectory.
4. Repeat for a few SCP iterations.

This produces a computationally efficient approximation of nonlinear safe MPC suitable for real-time robotic control.

---

## 🔐 Safety Formulation

### Safety Function

For each obstacle:

\[
h(q) = \|q - q_o\| - r_{safe}
\]

Safe set:

\[
h(q) \ge 0
\]

### Velocity-Level CBF Condition

\[
\nabla h(q)^T v \ge -\alpha h(q)
\]

To maintain convexity, the nonlinear distance function is linearized:

\[
h(q) \approx n_k^T (q - q_o) - r_{safe}
\]

where \(n_k\) is computed from a reference trajectory.

This converts the safety constraint into a linear inequality inside MPC.

---

## ⚙️ MPC Formulation

### Decision Variables
- Position trajectory \(q_k\)
- Velocity trajectory \(v_k\)
- Control inputs \(u_k\)

### Objective
- Drive to goal
- Minimize control effort
- Penalize safety violation (slack variable)

### Constraints
- Discrete double-integrator dynamics
- Velocity bounds
- Acceleration bounds
- Linearized CBF safety constraint

The resulting optimization problem is a convex Quadratic Program solved using OSQP.

---

## 🛠 Features

- ✅ Real-time convex MPC
- ✅ CBF-based obstacle avoidance
- ✅ Sequential convex programming (SCP)
- ✅ Slack-relaxed safety constraints for guaranteed feasibility
- ✅ Time-colored trajectory visualization
- ✅ Closest-obstacle selection

