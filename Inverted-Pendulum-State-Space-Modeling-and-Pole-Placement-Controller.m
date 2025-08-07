clear; clc;

% System Matrices for Linearized Inverted Pendulum
% State variables: x = [X, X_dot, theta, theta_dot]'
% Where X = cart position, theta = pole angle

A = [0 1 0 0;
     0 0 -1.962 0;
     0 0 0 1;
     0 0 11.772 0];

B = [0; 1; 0; -1];

% Output: Both cart position (X) and pole angle (theta)
C = [1 0 0 0;   % X
     0 0 1 0];  % theta

D = [0; 0];

% Desired specifications for control design
settlingTime = 5;     % Desired settling time Ts [seconds]
maxOvershoot = 0.2;   % Maximum overshoot (20%)

% Calculate damping ratio (zeta) and natural frequency (wn)
zeta = -log(maxOvershoot) / sqrt(pi^2 + log(maxOvershoot)^2);
wn = 4 / (settlingTime * zeta);

% Place closed-loop poles for desired performance
% Two dominant poles, two far poles for fast settling
desiredPoles = [-zeta*wn + 1i*wn; 
                -zeta*wn - 1i*wn;
                -10*zeta*wn + 1i*wn; 
                -10*zeta*wn - 1i*wn];

% State feedback controller gain K
K = place(A, B, desiredPoles);

% Closed-loop state-space system (full-state feedback)
sys = ss(A - B*K, B, C, D);

% Step response simulation for cart position (X)
figure(1)
step(sys(1));
xlabel('time (seconds)')
ylabel('x')
title('Step Response: Cart Position')

% Step response simulation for pole angle (theta)
figure(2)
step(sys(2));
xlabel('time (seconds)')
ylabel('\theta')
title('Step Response: Pole Angle')

% End of file
