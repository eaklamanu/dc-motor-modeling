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

% --- Parameters ---
% First-order plant
s = tf('s');
G = 1/(s+1);


% Time vector for simulation
t = 0:0.01:10;

%% 1. Proportional control (P)
Kp_low = 0.5;   % Low gain
Kp_high = 5;    % High gain

C_P_low = Kp_low;
C_P_high = Kp_high;

T_P_low = feedback(C_P_low*G, 1);
T_P_high = feedback(C_P_high*G, 1);

disp("stepinfo of T_P_low : ");
stepinfo(T_P_low)
disp("T_P_high : ");
stepinfo(T_P_high)


figure;
step(T_P_low, t);
title('P Control (Low Gain) - Slow but Stable');
grid on;
drawnow;


figure;
step(T_P_high, t);
title('P Control (High Gain) - Faster but Overshoots');
grid on;
drawnow;


% --- Save plots ---
%print('-dpng', '../results/impulse_response.png');
print("../results/open_vs_closed.png", "-dpng");


 %Getting Step Info
 info_T_P_low = stepinfo(T_P_low);
 info_T_P_high = stepinfo(T_P_high);

 results = {
  "System", "RiseTime", "SettlingTime", "PeakTime", "Overshoot", ...
  "SteadyStateError" ;
  "Open Loop", info_T_P_low.RiseTime, info_T_P_low.SettlingTime,info_T_P_low.PeakTime, ...
  info_T_P_low.Overshoot, info_T_P_low.SteadyStateError ;
  "Closed Loop", info_T_P_high.RiseTime, info_T_P_high.SettlingTime, info_T_P_high.PeakTime, ...
  info_T_P_high.Overshoot, info_T_P_high.SteadyStateError
};

fid = fopen("/home/rain/Documents/control_projects/dc_motor_modeling/motor_first_order/results/p_control_high_vs_low.csv", "w"); % change path based on folder structure
fprintf(fid, "System,RiseTime,SettlingTime, PeakTime, Overshoot,SteadyStateError\n");
fprintf(fid, "P_control_Low_Gain,%f,%f,%f,%f,%f\n", info_T_P_low.RiseTime, info_T_P_low.SettlingTime,info_T_P_low.PeakTime, ...
  info_T_P_low.Overshoot, info_T_P_low.SteadyStateError);
fprintf(fid, "P_control_High_Gain,%f,%f,%f,%f,%f\n", info_T_P_high.RiseTime, info_T_P_high.SettlingTime, info_T_P_high.PeakTime, ...
  info_T_P_high.Overshoot, info_T_P_high.SteadyStateError);
fclose(fid);





% --- Notes ---
% High P Gain - reaches steady state faster but overshoots
% Low P Gain - slow but stable
