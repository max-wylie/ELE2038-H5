% Differential equations:
% \dot{\alpha} = -0.31 \alpha + 57.4 r + 0.232 \delta
% \dot{r} = -0.016 \alpha - 0.425 r + 0.0203 \delta
% \dot{\theta} = 56.7 r

% State-space model: x = [alpha; r; theta], u = delta, y = theta
A = [-0.31, 57.4, 0;    % \dot{\alpha}
     -0.016, -0.425, 0; % \dot{r}
     0, 56.7, 0];      % \dot{\theta}
B = [0.232; 0.0203; 0]; % Input matrix
C = [0, 0, 1];          % Output matrix (y = theta)
D = 0;                  % Direct feedthrough

% Create state-space system
sys_ss = ss(A, B, C, D);

% Convert state-space to transfer function G_p(s) = C(sI - A)^(-1)B
G_p = tf(sys_ss);
disp('Derived G_p(s) from state-space:');
G_p
