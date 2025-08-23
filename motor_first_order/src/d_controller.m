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

clc;
clear;
pkg load control;
graphics_toolkit("qt");

% First-order plant
s = tf('s');
G = 1/(s+1);


% Time vector for simulation
t = 0:0.1:10;

%% Derivative control (D)
Kd_small = 0.5;
Kd_large = 5;

C_D_small = Kd_small*s;
C_D_large = Kd_large*s;

T_D_small = feedback(C_D_small*G, 1);
T_D_large = feedback(C_D_large*G, 1);

figure;
plot(2,1,1);
step(T_D_small, t);
title('D Control (Small Gain) - Slight Improvement in Damping');
grid on;
drawnow;

figure;
plot(2,1,2);
step(T_D_large, t);
title('D Control (Large Gain) - Very Damped but Sensitive to Noise');
grid on;
drawnow;

 %Getting Step Info
 info_T_D_small = stepinfo(T_D_small);
 info_T_D_large = stepinfo(T_D_large);


results = {
  "System", "RiseTime", "SettlingTime", "PeakTime", "Overshoot", ...
  "SteadyStateError" ;
  "Low Gain", info_T_D_small.RiseTime, info_T_D_small.SettlingTime,info_T_D_small.PeakTime, ...
  info_T_D_small.Overshoot, info_T_D_small.SteadyStateError ;
  "High Gain", info_T_D_large.RiseTime, info_T_D_large.SettlingTime, info_T_D_large.PeakTime, ...
  info_T_D_large.Overshoot, info_T_D_large.SteadyStateError
};

fid = fopen("/home/rain/Documents/control_projects/dc_motor_modeling/motor_first_order/results/d_control.csv", "w"); % change path based on folder structure
fprintf(fid, "System,RiseTime,SettlingTime, PeakTime, Overshoot,SteadyStateError\n");
fprintf(fid, "D_control_Low_Gain,%f,%f,%f,%f,%f\n", info_T_D_small.RiseTime, info_T_D_small.SettlingTime,info_T_D_small.PeakTime, ...
  info_T_D_small.Overshoot, info_T_D_small.SteadyStateError );
fprintf(fid, "D_control_High_Gain,%f,%f,%f,%f,%f\n", info_T_D_large.RiseTime, info_T_D_large.SettlingTime, info_T_D_large.PeakTime, ...
  info_T_D_large.Overshoot, info_T_D_large.SteadyStateError);
fclose(fid);


% --- Notes ---
% High Kd Gain - Very Damped but Sensitive to Noise
% Low Kd Gain - Slight Improvement in Damping

