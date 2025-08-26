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

%Plotting the Bode Plot
figure;
bode(H);
h = gca();                      % Get handle to current axes
set(h, "gridcolor", [0.3 0.3 0.3]);    % Set to a dark gray (RGB values)
set(h, "gridlinestyle", "-");         % Solid line
set(h, "gridalpha", 1);               % Full opacity (1 = solid, 0 = transparent)
set(h, "LineWidth", 1.2);         % Thicker grid lines
grid on;

% Save plot
outdir = "/home/rain/Documents/control_projects/dc_motor_modeling/motor_second_order/results";
print(fullfile(outdir, "bode_plot_sys.png"), "-dpng");
