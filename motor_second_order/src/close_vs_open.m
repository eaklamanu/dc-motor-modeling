% =========================================================================
% Modeling a DC motor as a first-order system in Octave.
% =========================================================================
% Dependencies:
%   GNU Octave + Control Package
%   (install using: pkg install -forge control)
% =========================================================================

clear; clc; close all;
pkg load control;
addpath("/functions");

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

disp("Plant G(s) = ω(s)/V(s):");
G

% Closed Loop Transfer Function
H = feedback(G, 1) ; %negative feedback with feedback path gain of 1
disp("Closed Loop Transfer Function: ");
H

% Step Response Comparison
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
text(tc_open,0.632 * y_open(end) + 0.1, sprintf("\\tau = %.2f s (Open)", tc_open), ...
 "Color", 'r', "FontSize", 10 );


%For closed-loop
plot([tc_closed tc_closed],[0 0.632 * y_closed(end)], 'k--');
plot([0 tc_closed],[0.632 * y_closed(end) 0.632 * y_closed(end)], 'k--');
text(tc_closed,0.632 * y_closed(end)+ 0.01, sprintf("\\tau = %.2f s (Closed)", tc_closed), ...
 "Color", 'b', "FontSize", 10 );


% --- Save plots ---
%print('-dpng', '../results/impulse_response.png');
print("../results/open_vs_closed.png", "-dpng");


 %Getting Step Info
 info_open = stepinfo(G);
 info_closed = stepinfo(H);
 disp("Display Step Info for G");
 info_open

 disp("Display Step Info for H");
 info_closed

 results = {
  "System", "RiseTime", "SettlingTime", "PeakTime", "Overshoot", ...
  "SteadyStateError", "SteadyStateValue" ;
  "Open Loop", info_open.RiseTime, info_open.SettlingTime,info_open.PeakTime, ...
  info_open.Overshoot, info_open.SteadyStateError, info_open.SteadyStateValue ;
  "Closed Loop", info_closed.RiseTime, info_closed.SettlingTime, info_closed.PeakTime, ...
  info_closed.Overshoot, info_closed.SteadyStateError, info_closed.SteadyStateValue
};

fid = fopen("../results/open_vs_closed_loop.csv", "w");
fprintf(fid, "System,RiseTime,SettlingTime, PeakTime, Overshoot,SteadyStateError\n");
fprintf(fid, "Open Loop,%f,%f,%f,%f,%f\n", info_open.RiseTime, info_open.SettlingTime,info_open.PeakTime, ...
  info_open.Overshoot, info_open.SteadyStateError);
fprintf(fid, "Closed Loop,%f,%f,%f,%f,%f\n", info_closed.RiseTime, info_closed.SettlingTime, info_closed.PeakTime, ...
  info_closed.Overshoot, info_closed.SteadyStateError);
fclose(fid);





% --- Notes ---
% Closed-loop system reaches steady-state faster
% Compare system parameters for open and closed loop systems
