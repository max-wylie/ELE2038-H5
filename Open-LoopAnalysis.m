% Aircraft Pitch Control System - Open-Loop Analysis

clear all;
close all;
clc;

%% System Parameters
% Aircraft dynamics coefficients (from differential equations)

% \dot{\alpha} = -0.31 \alpha + 57.4 r + 0.232 \delta
% \dot{r} = -0.016 \alpha - 0.425 r + 0.0203 \delta
% \dot{\theta} = 56.7 r

% Actuator: 1st order, time constant = 14.5 ms, unit gain
tau_a = 0.0145; % seconds
G_a = tf(1, [tau_a 1]); % G_a(s) = 1 / (0.0145s + 1)

% Aircraft dynamics: G_p(s) = theta(s) / delta(s)
num_p = [1.15101 0.146345]; % 1.15101 (s + 0.1271)
den_p = [1 0.735 1.05015 0]; % s (s^2 + 0.735s + 1.05015)
G_p = tf(num_p, den_p);

% Sensor: 1st order with delay, time constant = 0.0021 s, delay = 0.0063 s
tau_m = 0.0021; % seconds
delay_m = 0.0063; % seconds
G_m = tf(1, [tau_m 1], 'InputDelay', delay_m); % G_m(s) = e^(-0.0063s) / (0.0021s + 1)

% Open-loop transfer function: G(s) = G_a(s) * G_p(s) * G_m(s)
G = series(series(G_a, G_p), G_m);

%% Analysis
% Poles and zeros
poles = pole(G);
zeros = zero(G);
fprintf('Poles:\n');
disp(poles);
fprintf('Zeros:\n');
disp(zeros);

% Step response
figure(1);
t = 0:0.01:20; % Time vector up to 20 seconds
step(G, t);
title('Step Response of Open-Loop System G(s)');
xlabel('Time (s)');
ylabel('Pitch Angle \theta (rad)');
grid on;

% Bode plot
figure(2);
bode(G);
title('Bode Plot of Open-Loop System G(s)');
grid on;

% Impulse response
figure(3);
t = 0:0.01:20; % Time vector up to 20 seconds (same as step response)
impulse(G, t);
title('Impulse Response of Open-Loop System G(s)');
xlabel('Time (s)');
ylabel('Pitch Angle \theta (rad)');
grid on;

% Save figures for report
print(1, '-dpng', 'step_response.png');
print(2, '-dpng', 'bode_plot.png');
print(3, '-dpng', 'impulse_response.png');

%% Display transfer function
disp('Open-loop transfer function G(s):');
G
