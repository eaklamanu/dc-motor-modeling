% =========================================================================
% Modeling a DC motor as a first-order system in Octave.
% =========================================================================
% Dependencies:
%   GNU Octave + Control Package
%   (install using: pkg install -forge control)
% =========================================================================

clear; clc; close all;
pkg load control;

% DC Motor as a First Order System
K = 10;
tau = 0.5;
s = tf('s');
G  = K/(tau*s + 1) ;
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
outdir = "/home/rain/Documents/control_projects/dc_motor_modeling/motor_first_order/results";
print(fullfile(outdir, "bode_plot.png"), "-dpng");
