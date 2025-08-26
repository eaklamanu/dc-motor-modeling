% =========================================================================
% PID tuning for a second-order DC motor using Ziegler–Nichols (closed-loop)
% =========================================================================

clear; clc; close all;
pkg load control;
pkg load signal;
addpath("/home/rain/Documents/control_projects/dc_motor_modeling/motor_first_order/src/functions");

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

% --- Ensure results folder exists ---
outdir = "/home/rain/Documents/control_projects/dc_motor_modeling/motor_second_order/results";
if ~exist(outdir, "dir"), mkdir(outdir); end

% --- 1) P-only closed loop (initial small Kp just to see) ---
Kp0 = 1.0;
C_P = pid(Kp0, 0, 0);
T_P0 = feedback(C_P*G, 1);
figure; step(T_P0, 0:0.001:0.3); grid on;
title("Closed-loop step: P-only (initial Kp)");
print(fullfile(outdir, "step_P_initial.png"), "-dpng");

% --- 2) Compute Ku and Tu from open-loop margins (preferred for 2nd-order) ---
% margin() returns GainMargin (as a factor), PhaseMargin (deg),
% Wcg (gain crossover), Wcp (phase crossover). We need GM and Wcp.
[GM, PM, Wcg, Wcp] = margin(G);

if isfinite(GM) && isfinite(Wcp) && Wcp > 0
  Ku = GM;                 % For P-only loop, Ku ≈ GM (factor)
  Tu = 2*pi/Wcp;
else
  warning("Could not find finite GM/Wcp. Consider relay autotune or add small delay.");
  % Fallback: try a simple sweep to estimate Ku by simulation
  Kp_vals = logspace(-2, 3, 60);  % sweep
  Ku = NaN; Tu = NaN;
  for k = 1:numel(Kp_vals)
    Ttest = feedback(pid(Kp_vals(k),0,0)*G, 1);
    t = linspace(0, 2, 5000);
    y = step(Ttest, t);
    % crude oscillation detection: check if peaks ~ constant amplitude
    [pks, locs] = findpeaks(y, "MinPeakDistance", 20);
    if numel(pks) >= 5
      last = pks(end-4:end);
      amp_ratio = max(last)/min(last);
      if amp_ratio < 1.1   % ~sustained
        Ku = Kp_vals(k);
        Tu = mean(diff(t(locs(end-4:end))));
        break;
      end
    end
  end
end

printf("Estimated Ku = %.4g, Tu = %.4g s\n", Ku, Tu);

% --- 3) Ziegler–Nichols PID from Ku, Tu ---
if ~isnan(Ku) && ~isnan(Tu)
  Kp = 0.6*Ku;
  Ti = Tu/2;   Ki = Kp/Ti;
  Td = Tu/8;   Kd = Kp*Td;
else
  % Reasonable fallback if Ku/Tu failed (gentler than Z–N)
  % IMC-like: pick closed-loop time ~ 3*tau_eq
  [z, p, k] = zpkdata(G, "v");
  % rough equivalent time constant for two real poles
  tau_eq = sum(-1./real(p(real(p)<0)));
  Kp = 1.0; Ki = 5.0; Kd = 0.0;  % tweak later
  Ti = Kp/Ki; Td = Kd/Kp;
end

C_zn = pid(Kp, Ki, Kd);
disp("Z–N PID gains:"); Kp, Ki, Kd



% --- 4) Closed-loop with Z–N PID ---
% P controller only
C_P = pid(Kp,0,0);
T_P = feedback(C_P*G, 1);

% PI controller
tplot = 0:0.00005:0.03;
C_PI = pid(Kp, Ki, 0);
T_PI = feedback(C_PI *G, 1);

% Effect of adding Integrator to P controller
step(T_P,T_PI,tplot);grid on;
legend("P-only-tuned", "PI (Z–N)");
title("Step response comparison");
print(fullfile(outdir, "step_compare_P_vs_PI.png"), "-dpng");


T_zn = feedback(C_zn*G, 1);

% Plots
tplot = 0:0.0005:0.3;
figure; step(T_P0, T_zn, tplot); grid on;
legend("P-only (initial)", "PID (Z–N)");
title("Step response comparison");
print(fullfile(outdir, "step_compare_P_vs_ZN.png"), "-dpng");

% --- 5) Performance metrics ---
info_P0 = stepinfo(T_P0); % no tuning
info_ZN = stepinfo(T_zn);
info_PI = stepinfo(T_PI);
info_P = stepinfo(T_P);

% Build a small cell table and write CSV
results = {
  "System","RiseTime","SettlingTime","Overshoot(%)","SteadyStateError";
  "P-only-no_tuning",  info_P0.RiseTime, info_P0.SettlingTime, info_P0.Overshoot, info_P0.SteadyStateError;
  "P-only-tuned", info_P.RiseTime, info_P.SettlingTime, info_P.Overshoot, info_P.SteadyStateError;
  "PI", info_PI.RiseTime, info_PI.SettlingTime, info_PI.Overshoot, info_PI.SteadyStateError;
  "Z-N PID", info_ZN.RiseTime, info_ZN.SettlingTime, info_ZN.Overshoot, info_ZN.SteadyStateError
};

% Simple CSV writer (no dependencies)
csvpath = fullfile(outdir, "pid_second_order_results.csv");
fid = fopen(csvpath, "w");
fprintf(fid, "System,RiseTime,SettlingTime,Overshoot,SteadyStateError\n");
fprintf(fid, "P-only-no_tuning,%.6g,%.6g,%.6g,%.6g\n", ...
        info_P0.RiseTime, info_P0.SettlingTime, info_P0.Overshoot, info_P0.SteadyStateError);
fprintf(fid, "P-only-tuned,%.6g,%.6g,%.6g,%.6g\n", ...
        info_P.RiseTime, info_P.SettlingTime, info_P.Overshoot, info_P.SteadyStateError);
fprintf(fid, "PI,%.6g,%.6g,%.6g,%.6g\n", ...
        info_PI.RiseTime, info_PI.SettlingTime, info_PI.Overshoot, info_PI.SteadyStateError);
fprintf(fid, "Z-N PID,%.6g,%.6g,%.6g,%.6g\n", ...
        info_ZN.RiseTime, info_ZN.SettlingTime, info_ZN.Overshoot, info_ZN.SteadyStateError);
fclose(fid);

% Save bode and pole-zero map for documentation
figure; bode(G); grid on; title("Plant Bode");
print(fullfile(outdir, "bode_plant_tuning.png"), "-dpng");

figure; pzmap(G); grid on; title("Plant pzmap");
print(fullfile(outdir, "pzmap_plant.png"), "-dpng");

disp(["Saved results to: " csvpath]);

