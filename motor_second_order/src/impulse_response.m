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

% Open-Loop Impulse Response
figure;
[y, t] = impulse(G);
plot(t,y, "LineWidth", 2);
title("Open Loop Impulse Response");
xlabel("time(s)");
ylabel("Speed");
grid on;
print('-dpng', '../results/impulse_response_open.png');


% Closed-Loop Impulse Response
figure;
[y1, t] = impulse(H);
plot(t,y1, "LineWidth", 2);
title("Closed-Loop Impulse Response");
xlabel("time(s)");
ylabel("Speed");
grid on;
print("../results/impulse_response_closed.png", "-dpng"); % Save plot



% --- Notes ---
% The impulse response shows how a system reacts to a sudden disturbance,
% and how quickly it returns to steady-state — which gives insight into how
% it might recover from brief noise-like inputs.

% Open-loop response dissipates sudden input response in about 0.4s.
% Closed-loop response dissipates sudden input response in about 0.02s.

