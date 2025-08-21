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

% 2. Closed Loop Transfer Function
H = feedback(G, 1) ; %negative feedback with feedback path gain of 1
disp("Closed Loop Transfer Function: ");
H

% 3. Step Response Comparison
figure;
[y_open, t_open] = step(G);
[y_closed, t_closed] = step(H);

% 4. Find the time constant
tc_open = t_open(find(y_open >= 0.632 * y_open(end),1));
tc_closed = t_closed(find(y_closed >= 0.632 * y_closed(end),1));


% 5. Plot the Step Responses
plot(t_open, y_open, 'r', 'LineWidth', 2); hold on;
plot(t_closed, y_closed, 'b', 'LineWidth', 2);
legend("Open Loop","Closed Loop");
xlabel("Time(s)");
ylabel("Output");

% 6. Mark Time Constants on the Graph

%For open-loop
plot([tc_open tc_open],[0 0.632 * y_open(end)], 'r--');
plot([0 tc_open],[0.632 * y_open(end) 0.632 * y_open(end)], 'r--');
text(tc_open,0.632 * y_open(end) + 0.50, sprintf("\\tau = %.2f s (Open)", tc_open), ...
 "Color", 'r', "FontSize", 10 );


%For closed-loop
plot([tc_closed tc_closed],[0 0.632 * y_closed(end)], 'k--');
plot([0 tc_closed],[0.632 * y_closed(end) 0.632 * y_closed(end)], 'k--');
text(tc_closed,0.632 * y_closed(end)+ 0.01, sprintf("\\tau = %.2f s (Closed)", tc_closed), ...
 "Color", 'b', "FontSize", 10 );


% --- Save plots ---
%print('-dpng', '../results/impulse_response.png');
print("../results/impulse_response.png", "-dpng");


% --- Notes ---
% Closed-loop system reaches steady-state faster

