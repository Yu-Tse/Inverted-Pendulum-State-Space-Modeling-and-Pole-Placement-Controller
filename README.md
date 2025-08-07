# Inverted Pendulum: State-Space Modeling and Pole Placement Controller

This repository demonstrates the complete workflow for modeling, linearizing, and controlling a classic inverted pendulum system using modern state-space techniques. The process includes:

- **Lagrangian mechanics derivation** for equations of motion
- **Linearization** around the upright equilibrium
- **State-space modeling** of the linearized system
- **State feedback control (pole placement)** to meet specified time-domain requirements
- **Simulation of closed-loop system** using MATLAB

## Problem Description

The inverted pendulum on a cart is a standard benchmark problem in control theory, often used to validate modern control methods due to its inherent instability and coupling of dynamics. This project demonstrates my ability to:

- Derive system equations from first principles
- Translate physical models into computational (state-space) form
- Design and implement a pole-placement controller
- Validate performance using simulation

## System Dynamics

- States: Cart position (X), Cart velocity (X_dot), Pole angle (theta), Pole angular velocity (theta_dot)
- Inputs: Control force applied to the cart
- Outputs: Cart position and pole angle

## Derivation of System Equations

This section provides a step-by-step derivation of the equations of motion for the inverted pendulum on a cart using Lagrangian mechanics, followed by linearization and state-space formulation.

### 1. Kinematics

Define the position of the pendulum mass in Cartesian coordinates:
- Let \( X \) be the position of the cart, \( l \) the length to the pendulum mass, and \( \theta \) the angle from vertical.

\[
x_m = X + l \sin\theta \\
y_m = l \cos\theta
\]

The velocities are:
\[
\dot{x}_m = \dot{X} + l \cos\theta \cdot \dot{\theta} \\
\dot{y}_m = -l \sin\theta \cdot \dot{\theta}
\]

### 2. Energy Expressions

**Potential Energy:**
\[
V = mgy_m = mgl\cos\theta
\]

**Kinetic Energy:**
\[
T = \frac{1}{2} M \dot{X}^2 + \frac{1}{2} m \left( \dot{x}_m^2 + \dot{y}_m^2 \right )
\]

Expanding and simplifying:

\[
T = \frac{1}{2}(M + m)\dot{X}^2 + \frac{1}{2} m l^2 \dot{\theta}^2 + m l \dot{X} \dot{\theta} \cos\theta
\]

### 3. Lagrangian and Equations of Motion

The Lagrangian is \( L = T - V \):

\[
L = \frac{1}{2}(M + m)\dot{X}^2 + \frac{1}{2} m l^2 \dot{\theta}^2 + m l \dot{X} \dot{\theta} \cos\theta - mgl\cos\theta
\]

Apply the Euler-Lagrange equations for each generalized coordinate (\( X \) and \( \theta \)):

\[
\frac{d}{dt} \left( \frac{\partial L}{\partial \dot{X}} \right ) - \frac{\partial L}{\partial X} = u(t)
\]
\[
\frac{d}{dt} \left( \frac{\partial L}{\partial \dot{\theta}} \right ) - \frac{\partial L}{\partial \theta} = 0
\]

This yields the coupled nonlinear equations:

\[
(M + m)\ddot{X} + m l \ddot{\theta} \cos\theta - m l \dot{\theta}^2 \sin\theta = u(t)
\]
\[
m l^2 \ddot{\theta} + m l \ddot{X} \cos\theta - mgl\sin\theta = 0
\]

### 4. Linearization (Small-Angle Approximation)

Assume small angle \( \theta \approx 0 \), so \( \sin\theta \approx \theta \), \( \cos\theta \approx 1 \), and neglect higher-order terms:

\[
(M + m)\ddot{X} + m l \ddot{\theta} = u(t)
\]
\[
m l^2 \ddot{\theta} + m l \ddot{X} - mgl\theta = 0
\]

Insert system parameters (for example, \( M=1 \), \( m=0.2 \), \( l=1 \)), and define state variables:

\[
x_1 = X, \quad x_2 = \dot{X}, \quad x_3 = \theta, \quad x_4 = \dot{\theta}
\]

The system can be written in state-space form:

\[
\begin{cases}
\dot{x}_1 = x_2 \\
\dot{x}_2 = -1.962 x_3 + u \\
\dot{x}_3 = x_4 \\
\dot{x}_4 = 11.772 x_3 - u
\end{cases}
\]

Or, in matrix notation:

\[
\dot{\mathbf{x}} = A\mathbf{x} + B u
\]
\[
\mathbf{y} = C\mathbf{x}
\]

Where:
\[
A = \begin{bmatrix}
0 & 1 & 0 & 0 \\
0 & 0 & -1.962 & 0 \\
0 & 0 & 0 & 1 \\
0 & 0 & 11.772 & 0
\end{bmatrix}
, \quad
B = \begin{bmatrix}
0 \\ 1 \\ 0 \\ -1
\end{bmatrix}
\]

### 5. Controller Design

A state-feedback controller is designed via pole placement to achieve desired settling time and overshoot.

---

**This derivation demonstrates a systematic approach from physics-based modeling to control-oriented linearization and state-space representation, reflecting solid engineering fundamentals and practical MATLAB implementation skills.**


## Controller Design

The state feedback controller is designed based on the following requirements:
- Settling time: 5 seconds
- Maximum overshoot: 20%
- Full state feedback (assumes all states are measurable)

Poles are selected to satisfy these requirements and ensure rapid decay of higher-order modes.



## Simulation Results

Sample simulation results are shown below:

![Step Response X](<img width="865" height="649" alt="image" src="https://github.com/user-attachments/assets/6ada0240-4ed6-46df-a92f-bf110ad07b3a" />)

![Step Response Theta](<img width="865" height="649" alt="image" src="https://github.com/user-attachments/assets/44ff8c62-5735-4678-865a-7877d9bb5e2d" />)

## What I Learned

- Practical application of Lagrangian mechanics for system modeling
- Translating physics into computational models for simulation and control
- Systematic controller design (pole placement) for desired dynamic response
- Critical evaluation of controller performance through simulation

## Disclaimer

This project was completed as part of coursework for Modern Control Systems and is intended to showcase my engineering ability and understanding of control systems. All code and documentation are original unless otherwise noted.

---

If you have any questions or feedback, please feel free to open an issue or contact [me](yutsewu0209@gmail.com).
