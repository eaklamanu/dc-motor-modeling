% =========================================================================
%   The script shows some system parameters for a second order control system.
% Dependencies:
%   GNU Octave + Control Package
%   (install using: pkg install -forge control)
% =========================================================================
clc;  clear;
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
H = feedback(G,1)

% Step Response
[y,t_out] = step(H);

% --- Steady-state value ---
tail_n = max(5, ceil(0.1 * length(y)));
y_final = mean(y(end - tail_n + 1:end));

% --- Settling Time (±2% band) ---
tol = 0.02 * abs(y_final);
idx_settle = find(abs(y - y_final) > tol, 1, 'last');
settling_time = t_out(idx_settle);

% --- Rise Time (10% to 90% of final value) ---
y_10 = 0.1 * y_final;
y_90 = 0.9 * y_final;
t_rise_start = t_out(find(y >= y_10, 1, 'first'));
t_rise_end = t_out(find(y >= y_90, 1, 'first'));
rise_time = t_rise_end - t_rise_start;

% --- Percentage Overshoot ---
[y_peak, idx_peak] = max(y);
perc_overshoot = ((y_peak - y_final) / y_final) * 100;
t_peak = t_out(idx_peak);

% --- Plot ---
figure;
plot(t_out, y, 'b', 'LineWidth', 2); hold on;

% Tolerance bands
plot(t_out, (y_final + tol) * ones(size(t_out)), 'r--', 'LineWidth', 1.5);
plot(t_out, (y_final - tol) * ones(size(t_out)), 'r--', 'LineWidth', 1.5);

% Settling time
plot(settling_time, y(idx_settle), 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
line([settling_time settling_time], ylim, 'Color', 'g', 'LineStyle', '--');

% Rise time
plot([t_rise_start t_rise_end], [y_10 y_90], 'mo-', 'LineWidth', 2);

% Overshoot point
plot(t_peak, y_peak, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');

% Labels & legend
xlabel('Time (s)');
ylabel('Output');
title('Step Response with Settling Time, Rise Time, and Overshoot');
legend('Response', '+2% Band', '-2% Band', ...
       'Settling Time', 'Settling Time','Rise Time', ...
         'Overshoot','Location', 'Southeast');

grid on;

% --- Display results ---
disp(['Final Value: ' num2str(y_final)]);
disp(['Settling Time: ' num2str(settling_time) ' s']);
disp(['Rise Time (10%-90%): ' num2str(rise_time) ' s']);
disp(['Percentage Overshoot: ' num2str(perc_overshoot) ' %']);


% --- Save plot ---
outdir = "/home/rain/Documents/control_projects/dc_motor_modeling/motor_second_order/results" ;
print(fullfile(outdir, "sys_params.png"), "-dpng");
