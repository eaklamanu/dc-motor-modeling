
function info = stepinfo(arg1, arg2, tol)
  % Flexible stepinfo function for Octave
  % Usage:
  %   info = stepinfo(sys)        % sys is a transfer function
  %   info = stepinfo(y, t)       % y and t are response and time
  %   info = stepinfo(..., tol)   % optional tolerance (default 0.02)

  if nargin < 3
    tol = 0.02;  % default 2% tolerance
  end

  if nargin == 1  % transfer function
    [y, t] = step(arg1);
  elseif nargin >= 2  % y and t vectors
    y = arg1(:);
    t = arg2(:);
  else
    error('Invalid input. Provide sys or (y, t).');
  end

  % Use average of last 10% of points as final value
  n = length(y);
  tail_n = max(5, ceil(0.1 * n));  % at least 5 points
  y_final = mean(y(end - tail_n + 1:end));

  % Rise Time (10% to 90%)
  y_10 = 0.1 * y_final;
  y_90 = 0.9 * y_final;
  idx_rise_start = find(y >= y_10, 1);
  idx_rise_end = find(y >= y_90, 1);
  rise_time = t(idx_rise_end) - t(idx_rise_start);

  % Settling Time
  idx_settle = find(abs(y - y_final) > tol * abs(y_final));
  if isempty(idx_settle)
    settling_time = 0;
  else
    settling_time = t(max(idx_settle));
  end

  % Peak and Overshoot
  [y_peak, idx_peak] = max(y);
  peak_time = t(idx_peak);
  overshoot = (y_peak - y_final) / abs(y_final) * 100;

  % Steady-state error
  steady_state_error = abs(y(end) - y_final);

    % Steady-state value
  steady_state = y_final;

  % Build result struct
  info.RiseTime = rise_time;
  info.SettlingTime = settling_time;
  info.PeakTime = peak_time;
  info.Overshoot = overshoot + "%";
  info.SteadyStateError = steady_state_error;
  info.SteadyStateValue = steady_state;


  end
