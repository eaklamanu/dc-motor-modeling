% =========================================================================
%   The script simulates the pole-zero map.
% Dependencies:
%   GNU Octave + Control Package
%   (install using: pkg install -forge control)
%
% =========================================================================


clear all;
clc;

pkg load control;

% DC Motor as a First Order System
K = 10;
tau = 0.5;

s = tf('s');
G  = K/(tau*s + 1) ;
disp("Open Loop Transfer Function: ");
G

% 2. Plotting the pzmap
figure;
pzmap(G);
title("Pole-Zero Map - Open Loop");
xlabel("zeros");
grid on;
h = gca();                      % Get handle to current axes
set(h, "gridcolor", [0.3 0.3 0.3]);    % Set to a dark gray (RGB values)
set(h, "gridlinestyle", "-");         % Solid line
set(h, "gridalpha", 1);               % Full opacity (1 = solid, 0 = transparent)
set(h, "LineWidth", 1.2);         % Thicker grid lines

% --- Save plots ---
%print("../results/pzmap.png", "-dpng");
print("/home/rain/Documents/control_projects/dc_motor_modeling/motor_first_order/results/pzmap.png","-dpng");

% --- Notes ---
%From the pzmap of a first-order system you can conclude:
%Stability (pole left or right of imaginary axis).
%Speed of response (distance of pole from origin).
%Type of response (always exponential, no oscillations) for first order systems.


