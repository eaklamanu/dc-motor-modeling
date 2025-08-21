% =========================================================================
% motor_first_order.m
%
% Modeling a DC motor as a first-order system in Octave.
% =========================================================================
%
% Description:
%   This script models a DC motor using a first-order transfer function:
%
%       ω(s) / V(s) = K / (τs + 1)
%
%   where:
%       ω(s) = angular velocity (rad/s)
%       V(s) = input voltage (V)
%       K    = system gain (rad/s per V)
%       τ    = time constant (s)
%
%   The script simulates the open-loop step response.
%   The time constant is also clearly marked on the script for easy analysis.
%
% Dependencies:
%   GNU Octave + Control Package
%   (install using: pkg install -forge control)
%
% =========================================================================

clear; clc; close all;
pkg load control;

% --- Parameters (example values, replace with your own) ---
K = 10;      % system gain
tau = 0.5;   % time constant (s)

% --- Transfer function model ---
s = tf('s');
G = K / (tau*s + 1);

% Open-Loop Step Response
figure;
[y, t] = impulse(G);
plot(t,y, "LineWidth", 2); hold on;
grid on;

title("Open Loop Impulse Response");
xlabel("time(s)");
ylabel("Speed");
grid on;

%Marking the Time Constant on the Plot of Step Response
% Compute time constant
p = pole(G);              % Get system poles
tau1 = 1 / abs(real(p(1))); % Time constant (first-order: 0.5)
% Find the response value at t ≈ tau
[~, idx1] = min(abs(t - tau1));  % Find the index closest to tau
y_tau1 = y(idx1);
t_tau1 = t(idx1);
% Mark the time constant on the plot
plot(t_tau1, y_tau1, 'ro', 'MarkerFaceColor', 'r');
text(t_tau1, y_tau1 + 0.01, sprintf('tau = %.2fs', tau1), 'FontSize', 12);


% --- Save plots ---
%print('-dpng', '../results/impulse_response.png');
print("../results/impulse_response.png", "-dpng");


% --- Notes ---
% The impulse response shows how a system reacts to a sudden disturbance,
% and how quickly it returns to steady-state — which gives insight into how
% it might recover from brief noise-like inputs.

