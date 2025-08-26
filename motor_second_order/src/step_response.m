% =========================================================================
% Modeling a DC motor as a second-order system in Octave.
% =========================================================================
% Dependencies:
%   GNU Octave + Control Package
%   (install using: pkg install -forge control)
% =========================================================================

clear; clc; close all;
pkg load control;

% --- Motor parameters (example values; replace with your data) ---
R  = 1.0;      % Armature resistance (Ohm)
L  = 0.5e-3;   % Armature inductance (H)
Kt = 0.05;     % Torque constant (N·m/A)
Ke = 0.05;     % Back-EMF constant (V·s/rad)
J  = 2.0e-4;   % Rotor inertia (kg·m^2)
B  = 1.0e-4;   % Viscous friction (N·m·s/rad)

% --- Transfer function: speed (rad/s) per input voltage (V) ---
num = [Kt];
den = conv([J, B], [L, R]) + [0, 0, Kt*Ke];  % (Js+B)(Ls+R) + Kt*Ke
G = tf(num, den);    % Plant
H = feedback(G,1);

outdir = "/home/rain/Documents/control_projects/dc_motor_modeling/motor_second_order/results";

% Open-Loop Step Response
figure;
[y,t] = step(G);
plot(t,y, "LineWidth", 2);
title("Open Loop Step Response");
xlabel("time(s)");
ylabel("Speed");
grid on;
print(fullfile(outdir, "step_response_open"), "-dpng");

figure;
[y1,t] = step(H);
plot(t,y1, "LineWidth", 2);
title("Closed-Loop Step Response");
xlabel("time(s)"); ylabel("Speed");
grid on;
print(fullfile(outdir, "step_response_closed"), "-dpng");



% --- Save plots ---
print('-dpng', '../results/step_response_closed.png');


% --- Notes ---
% Open loop response reaches steady state in about 0.5s
% Closed-loop response reaches steady state in about 0.15s


