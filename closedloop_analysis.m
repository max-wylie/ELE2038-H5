clear all;
close all;
clc;

%% Define Plant (Consistent Across All Sections)
s = tf('s');
P_pitch = (1.151*s + 0.1774)/(s^3 + 0.739*s^2 + 0.921*s); % Aircraft dynamics
Gm = exp(-0.0063*s)/(0.0021*s + 1); % Sensor with delay
Ga = 1/(0.0145*s + 1); % Actuator
Plant = P_pitch * Gm * Ga;

% Time vector for consistency
t = 0:0.01:20;

%% Finding Ku with Proportional Control
% Try proportional control only (vary Kp)
Kp = 22.5; % Start with a guess, found to be Ku
C = Kp;
% Closed-loop system with unity feedback
Gcl = feedback(C * Plant, 1);

% Step response to test stability and find sustained oscillations (Fig. 3a)
figure(1);
step(Gcl, t);
title('Closed-Loop Step Response with K_p = 22.5 (Sustained Oscillations)');
xlabel('Time (s)');
ylabel('Pitch Angle \theta (rad)');
xlim([0 20]); % Zoom in on time axis (0 to 20 seconds)
ylim([0 2]); % Zoom in on amplitude
grid on;
print('-dpng', 'sustained_oscillations');

% Ku found as 22.5, Tu (period) = 1.2 s from oscillations

%% PID Tuning with Ziegler-Nichols
% Ziegler-Nichols PID parameters
Ku = 22.5;
Tu = 1.2;
Kp = 22.5; % Adjusted from standard 0.6*Ku
Ti = 8; % Adjusted from Tu/2
Td = Tu / 2; % 0.6

% PID controller: Kp * (1 + 1/(Ti*s) + Td*s)
PID = Kp * (1 + 1/(Ti*s) + Td*s);

% Closed-loop system
Gcl_PID = feedback(PID * Plant, 1);

% Plot step response (Fig. 3b)
figure(2);
step(Gcl_PID, t);
title('Closed-Loop Step Response with Ziegler-Nichols PID Controller');
xlabel('Time (s)');
ylabel('Pitch Angle \theta (rad)');
grid on;
print('-dpng', 'closed_loop_step_response');

% Show overshoot and performance info
info = stepinfo(Gcl_PID);
fprintf('Step Response Overshoot: %.2f%%\n', info.Overshoot);
fprintf('Rise Time: %.2f s\n', info.RiseTime);
fprintf('Settling Time: %.2f s\n', info.SettlingTime);
fprintf('Steady-State Error: %.2f%%\n', (1 - info.SteadyStateValue) * 100);

%% Disturbance Check
% Ziegler-Nichols PID parameters (repeated for clarity)
Ku = 22.5;
Tu = 1.2;
Kp = 22.5;
Ti = 8;
Td = Tu / 2; % 0.6

% PID controller
PID = Kp * (1 + 1/(Ti*s) + Td*s);

% Closed-loop response to reference input (not plotted again)
Gcl_PID = feedback(PID * Plant, 1);

% Disturbance enters before the plant
T_disturbance = feedback(Plant, PID); % Output due to disturbance

% Simulate step disturbance (Fig. 3c)
figure(3);
step(T_disturbance, t);
title('Response to Step Disturbance at Plant Input');
ylabel('Pitch Angle \theta (rad)');
xlabel('Time (s)');
grid on;
print('-dpng', 'disturbance_response');

% Simulate step disturbance and capture output
[y_d, t_d] = step(T_disturbance, t);

% Compute approximate steady-state value (average of last N samples)
N = 50; % Adjustable
y_final = mean(y_d(end-N+1:end)); % Handles slow settling and noise

% Get peak value
y_peak = max(y_d);

% Calculate overshoot (ensure no division by zero)
if abs(y_final) > 1e-4
    overshoot_d = ((y_peak - y_final) / abs(y_final)) * 100;
else
    overshoot_d = 0; % Treat as no overshoot if final value is too small
end

% Print result
fprintf('Disturbance Overshoot: %.2f%%\n', overshoot_d);

disp('Closed-loop plots saved: sustained_oscillations.png, closed_loop_step_response.png, disturbance_response.png');
