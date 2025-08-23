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
t = 0:0.1:10;

%% Integral control (I)
Ki_small = 0.5;
Ki_large = 5;

C_I_small = Ki_small/s;
C_I_large = Ki_large/s;

T_I_low = feedback(C_I_small*G, 1);
T_I_high = feedback(C_I_large*G, 1);

figure;
%subplot(2,1,1);
step(T_I_low, t);
title('I Control (Small Gain) - Eliminates Steady-State Error Slowly');
grid on;
drawnow;

figure;
%subplot(2,1,2);
step(T_I_high, t);
title('I Control (Large Gain) - Eliminates Error Quickly but Overshoots & Oscillates');
grid on;
drawnow;

 %Getting Step Info
 info_T_I_low = stepinfo(T_I_low);
 info_T_I_high = stepinfo(T_I_high);


 results = {
  "System", "RiseTime", "SettlingTime", "PeakTime", "Overshoot", ...
  "SteadyStateError" ;
  "Low Gain", info_T_I_low.RiseTime, info_T_I_low.SettlingTime,info_T_I_low.PeakTime, ...
  info_T_I_low.Overshoot, info_T_I_low.SteadyStateError ;
  "High", info_T_I_high.RiseTime, info_T_I_high.SettlingTime, info_T_I_high.PeakTime, ...
  info_T_I_high.Overshoot, info_T_I_high.SteadyStateError
};

fid = fopen("/home/rain/Documents/control_projects/dc_motor_modeling/motor_first_order/results/i_control.csv", "w"); % change path based on folder structure
fprintf(fid, "System,RiseTime,SettlingTime, PeakTime, Overshoot,SteadyStateError\n");
fprintf(fid, "I_control_Low_Gain,%f,%f,%f,%f,%f\n", info_T_I_low.RiseTime, info_T_I_low.SettlingTime,info_T_I_low.PeakTime, ...
  info_T_I_low.Overshoot, info_T_I_low.SteadyStateError);
fprintf(fid, "I_control_High_Gain,%f,%f,%f,%f,%f\n", info_T_I_high.RiseTime, info_T_I_high.SettlingTime, info_T_I_high.PeakTime, ...
  info_T_I_high.Overshoot, info_T_I_high.SteadyStateError);
fclose(fid);


% --- Notes ---
% High Ki Gain - Eliminates Error Quickly but Overshoots & Oscillates
% Low Ki Gain - Eliminates Steady-State Error Slowly
