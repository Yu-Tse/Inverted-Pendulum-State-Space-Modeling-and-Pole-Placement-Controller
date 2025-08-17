# Inverted Pendulum: State-Space Modeling and Pole Placement Controller

This repository demonstrates the complete workflow for modeling, linearizing, and controlling a classic inverted pendulum system using modern state-space techniques. The process includes:

- **Lagrangian mechanics derivation** for equations of motion
- **Linearization** around the upright equilibrium
- **State-space modeling** of the linearized system
- **State feedback control (pole placement)** to meet specified time-domain requirements
- **Simulation of closed-loop system** using MATLAB

---

## Problem Description

The inverted pendulum on a cart is a standard benchmark problem in control theory, often used to validate modern control methods due to its inherent instability and coupling of dynamics. This project demonstrates my ability to:

- Derive system equations from first principles
- Translate physical models into computational (state-space) form
- Design and implement a pole-placement controller
- Validate performance using simulation

---

## System Dynamics

- **States:** Cart position ($X$), Cart velocity ($\dot{X}$), Pole angle ($\theta$), Pole angular velocity ($\dot{\theta}$)
- **Inputs:** Control force applied to the cart
- **Outputs:** Cart position and pole angle

---

## Derivation of System Equations

This section provides a step-by-step derivation of the equations of motion for the inverted pendulum on a cart using Lagrangian mechanics, followed by linearization and state-space formulation.

### 1. Kinematics

$$
    \begin{align*}
        x_m &= X + l\sin\theta \\
        y_m &= l\cos\theta \\
        \dot{x}_m &= \dot{X} + l\cos\theta\,\dot{\theta} \\
        \dot{y}_m &= -l\sin\theta\,\dot{\theta}
    \end{align*}
$$

### 2. Energy

- **Potential:**     $V = mgy_m = mgl\cos\theta$
- **Kinetic:**      $T = \frac{1}{2}M\dot{X}^2 + \frac{1}{2}m(\dot{x}_m^2 + \dot{y}_m^2)$


### 3. Lagrangian

$$
    L = T - V = \frac{1}{2}(M+m)\dot{X}^2 + \frac{1}{2}ml^2\dot{\theta}^2 + ml\dot{X}\dot{\theta}\cos\theta - mgl\cos\theta
$$

### 4. Euler-Lagrange Equations

$$
    \frac{d}{dt}\left(\frac{\partial L}{\partial \dot{X}}\right) - \frac{\partial L}{\partial X} = u(t)
$$
$$
    \frac{d}{dt}\left(\frac{\partial L}{\partial \dot{\theta}}\right) - \frac{\partial L}{\partial \theta} = 0
$$

### 5. Nonlinear Equations of Motion

$$
    \begin{cases}
    (M + m)\ddot{X} + ml\ddot{\theta}\cos\theta - ml\dot{\theta}^2\sin\theta = u(t) \\
    ml^2\ddot{\theta} + ml\ddot{X}\cos\theta - mgl\sin\theta = 0
    \end{cases}
$$

### 6. Linearization (Small Angle)


Assume $\, \sin\theta \approx \theta \,$ and $\, \cos\theta \approx 1 \,$.

$$
    \begin{cases}
    (M + m)\ddot{X} + ml\ddot{\theta} = u(t) \\
    ml^2\ddot{\theta} + ml\ddot{X} - mgl\theta = 0
    \end{cases}
$$


### 7. State-Space Representation


Let the state variables be defined as:
| State Variable   | Definition       | Physical Meaning           |
|------------------|------------------|----------------------------|
| $x_1$            | $X$              | cart position              |
| $x_2$            | $\dot{X}$        | cart velocity              |
| $x_3$            | $\theta$         | pendulum angle             |
| $x_4$            | $\dot{\theta}$   | pendulum angular velocity  |


$$
    \begin{bmatrix}
    \dot{x}_1 \\ \dot{x}_2 \\ \dot{x}_3 \\ \dot{x}_4
    \end{bmatrix}
    =
    \begin{bmatrix}
    0 & 1 & 0 & 0 \\
    0 & 0 & -1.962 & 0 \\
    0 & 0 & 0 & 1 \\
    0 & 0 & 11.772 & 0
    \end{bmatrix}
    \begin{bmatrix}
    x_1 \\ x_2 \\ x_3 \\ x_4
    \end{bmatrix}
    +
    \begin{bmatrix}
    0 \\ 1 \\ 0 \\ -1
    \end{bmatrix}
    u
$$


---

## Controller Design

The state feedback controller is designed based on the following requirements:
- Settling time: 5 seconds
- Maximum overshoot: 20%
- Full state feedback (assumes all states are measurable)


**Controllability:**
The controllability matrix

$$
\mathcal{C} = \big[B \;\ AB \;\ A^2B \;\ A^3B \big]
$$

is full rank, so the system is controllable.

---

**Design Requirements:**

* Settling time: \$T\_s = 5 \text{s}\$
* Maximum overshoot: \$20%\$
* Damping ratio: \$\zeta = 0.4559\$
* Natural frequency: \$\omega\_n = 1.7546\$

---

**Pole Selection:**

Poles are chosen by shifting ×10 the real part for faster decay:

$$
-\zeta\omega_n \pm \omega_n j, \quad
-3\omega_n \pm \omega_n j, \quad
-10\zeta\omega_n \pm \omega_n j
$$

---

**Feedback Gain:**

$$
K = 
\begin{bmatrix}
-25.4266 & -17.0053 & -133.5957 & -34.6053
\end{bmatrix}
$$


---

## Simulation Results

---

<img width="560" height="420" alt="x" src="https://github.com/user-attachments/assets/de52993b-170f-4407-8bce-05036a3a6451" />

<img width="560" height="420" alt="theta" src="https://github.com/user-attachments/assets/9b99c928-c43a-4239-a41c-b5168a3c23b0" />

---

## What I Learned

- Practical application of Lagrangian mechanics for system modeling
- Translating physics into computational models for simulation and control
- Systematic controller design (pole placement) for desired dynamic response
- Critical evaluation of controller performance through simulation

---

## Disclaimer

This project was completed as part of coursework for Modern Control Systems and is intended to showcase my engineering ability and understanding of control systems. All code and documentation are original unless otherwise noted.

---

If you have any questions or feedback, please feel free to open an issue or contact [me](mailto:yutsewu0209@gmail.com).
