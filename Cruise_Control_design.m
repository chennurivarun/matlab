%% Adaptive Cruise Control Simulation Script
% Author: [Your Name/Group]
% Date: [Current Date]
% Description:
% Simulates an ACC system with PID speed control, adaptive distance keeping,
% basic vehicle dynamics, and simulated sensor spoofing attacks.
% Includes hysteresis in adaptive logic switching.
% *** REQUIRES PARAMETER TUNING FOR REALISTIC BEHAVIOR ***

clear; % Clear variables from previous runs
clc;   % Clear the command window
close all; % Close all figures

fprintf('Starting ACC Simulation Script (with Hysteresis)...\n');

% ==========================================================================
% SECTION 1: SIMULATION PARAMETERS
% ==========================================================================
T_sim = 60;         % Total simulation time (seconds)
dt = 0.05;          % Simulation time step (seconds)
t = 0:dt:T_sim;     % Time vector for simulation steps

% ==========================================================================
% SECTION 2: VEHICLE PARAMETERS (Ego Vehicle - *** TUNE THESE ***)
% ==========================================================================
m = 1500;           % Ego vehicle mass (kg)
Cd = 0.3;           % Aerodynamic drag coefficient (unitless)
A = 2.5;            % Vehicle frontal area (m^2)
rho = 1.2;          % Air density (kg/m^3)
g = 9.81;           % Acceleration due to gravity (m/s^2)

vehicle_time_constant = 0.5; % Example time constant (seconds) - TUNE THIS
plant_num = 1;
plant_den = [vehicle_time_constant 1];

% --- Control & Physical Limits ---
a_max = 2.0;        % Maximum comfortable/engine acceleration (m/s^2)
a_min = -5.0;       % Maximum comfortable/braking deceleration (m/s^2) - Must be negative
F_max = m * a_max;  % Max engine/motor force (N) - Simplified relationship
F_min = m * a_min;  % Max braking force (N) - Simplified relationship
v_max_limit = 40;   % Absolute maximum speed allowed for the vehicle (m/s)

% ==========================================================================
% SECTION 3: ACC CONTROLLER PARAMETERS (*** TUNE THESE ***)
% ==========================================================================
v_set = 30;         % Driver's desired cruising speed (m/s)
T_gap = 1.8;        % Desired safe time gap to maintain behind lead vehicle (s)
d_min = 5;          % Minimum absolute safety distance cushion (m)
hyst_delta = 2.0;   % Hysteresis buffer for switching logic (m) % *** HYSTERESIS CHANGE ***

% ==========================================================================
% SECTION 4: PID SPEED CONTROLLER GAINS (*** CRITICAL TUNING REQUIRED ***)
% ==========================================================================
% --- USE THE GAINS THAT PRODUCED YOUR LAST STABLE PLOT AS A STARTING POINT ---
Kp = 150;  % EXAMPLE - REPLACE WITH YOUR BEST GAINS SO FAR
Ki = 15;   % EXAMPLE - REPLACE WITH YOUR BEST GAINS SO FAR
Kd = 50;   % EXAMPLE - REPLACE WITH YOUR BEST GAINS SO FAR

% ==========================================================================
% SECTION 5: LEAD VEHICLE BEHAVIOR (Define Scenario)
% ==========================================================================
d_lead_initial_offset = 80; % Initial distance of lead vehicle ahead of ego vehicle (m)
v_lead = zeros(size(t));    % Pre-allocate lead vehicle speed array

% --- Define Lead Vehicle Speed Profile Over Time ---
v_lead(t <= 10) = 25;
v_lead(t > 10 & t <= 30) = 15;
v_lead(t > 30 & t <= 50) = 15;
v_lead(t > 50) = 20;

% --- Calculate Lead Vehicle Position Over Time ---
pos_lead = zeros(size(t));
pos_lead(1) = d_lead_initial_offset;
for k_lead = 1:length(t)-1
    pos_lead(k_lead+1) = pos_lead(k_lead) + v_lead(k_lead) * dt;
end

% ==========================================================================
% SECTION 6: SENSOR SPOOFING ATTACK PARAMETERS (Configure Scenario)
% ==========================================================================
enable_spoofing = true;
attack_start_time = 25;
attack_end_time = 40;
spoofing_type = 'offset';
spoofing_value = -30;

% ==========================================================================
% SECTION 7: INITIALIZATION (State Variables and Log Arrays)
% ==========================================================================
n_steps = length(t);
v_ego       = zeros(n_steps, 1);
pos_ego     = zeros(n_steps, 1);
a_ego       = zeros(n_steps, 1);
d_rel       = zeros(n_steps, 1);
d_sensed    = zeros(n_steps, 1);
d_safe      = zeros(n_steps, 1);
v_target    = zeros(n_steps, 1);
speed_error = zeros(n_steps, 1);
F_control   = zeros(n_steps, 1);

% --- Set Initial Conditions ---
v_ego(1)    = 28.0;
pos_ego(1)  = 0.0;
d_rel(1)    = pos_lead(1) - pos_ego(1);
d_sensed(1) = d_rel(1);
v_target(1) = v_set;
d_safe(1)   = max(d_min, d_min + T_gap * v_ego(1)); % Ensure initial safe distance >= d_min

% --- PID Controller Internal State ---
integral_error = 0.0;
previous_error = 0.0;

% --- Hysteresis State Variable --- % *** HYSTERESIS FIX ***
% Initialize was_too_close based on the very first time step's condition
was_too_close = (d_sensed(1) < d_safe(1));

% --- Display Configuration Summary ---
fprintf('Simulation Setup:\n');
fprintf(' Time: Total=%.1fs, Step=%.3fs\n', T_sim, dt);
fprintf(' Ego Vehicle: Mass=%.0fkg, Max Accel=%.1fm/s^2, Max Decel=%.1fm/s^2\n', m, a_max, a_min);
fprintf(' ACC Params: Set Speed=%.1fm/s, Time Gap=%.1fs, Min Distance=%.1fm, Hysteresis=%.1fm\n', v_set, T_gap, d_min, hyst_delta); % *** HYSTERESIS CHANGE ***
fprintf(' PID Gains: Kp=%.1f, Ki=%.1f, Kd=%.1f\n', Kp, Ki, Kd);
if enable_spoofing
    fprintf(' Spoofing: ENABLED, Type=%s, Value=%.1f, Time=[%.1fs, %.1fs]\n', ...
            spoofing_type, spoofing_value, attack_start_time, attack_end_time);
else
    fprintf(' Spoofing: DISABLED\n');
end
fprintf('Initialization complete. Starting simulation loop (%d steps)...\n', n_steps);

% ==========================================================================
% SECTION 8: SIMULATION LOOP
% ==========================================================================
tic;
for k = 1:n_steps-1

    % --- 8.1: Calculate Actual Relative Distance ---
    d_rel(k) = pos_lead(k) - pos_ego(k);
    d_rel(k) = max(0, d_rel(k)); % Prevent negative distance

    % --- 8.2: Sensor Model & Spoofing Injection ---
    current_d_sensed = d_rel(k);
    if enable_spoofing && (t(k) >= attack_start_time) && (t(k) < attack_end_time)
        switch spoofing_type
            case 'offset'
                current_d_sensed = current_d_sensed + spoofing_value;
            case 'scale'
                current_d_sensed = current_d_sensed * spoofing_value;
        end
        current_d_sensed = max(0, current_d_sensed); % Ensure non-negative
    end
    d_sensed(k) = current_d_sensed;

    % --- 8.3: Adaptive Logic - Determine Target Speed (WITH HYSTERESIS) --- % *** HYSTERESIS CHANGE ***
    d_safe(k) = max(d_min, d_min + T_gap * v_ego(k)); % Calculate safe distance, ensure >= d_min

    % Conditions for switching logic
    is_too_close_now = (d_sensed(k) < d_safe(k));                 % Are we currently closer than the threshold?
    is_safe_with_hysteresis = (d_sensed(k) >= d_safe(k) + hyst_delta); % Are we clearly safe by a margin?

    if was_too_close % If previous state was 'following lead speed'
        if is_safe_with_hysteresis % Check if we are now safe by a margin
            v_target(k) = v_set;      % Switch back to aiming for set speed
            was_too_close = false;    % Update the state memory
        else
            v_target(k) = v_lead(k);  % Stay following lead speed
            was_too_close = true;     % Remain in this state
        end
    else % If previous state was 'aiming for set speed'
        if is_too_close_now % Check if we have become too close
            v_target(k) = v_lead(k);  % Switch to following lead speed
            was_too_close = true;     % Update the state memory
        else
            v_target(k) = v_set;      % Stay aiming for set speed
            was_too_close = false;    % Remain in this state
        end
    end
    % --- End of Hysteresis Logic --- %

    % --- Clamp the target speed ---
    v_target(k) = max(0, min(v_target(k), v_max_limit));

    % --- 8.4: PID Speed Controller Calculation ---
    speed_error(k) = v_target(k) - v_ego(k);
    integral_error = integral_error + speed_error(k) * dt;
    % Add anti-windup logic here if needed
    derivative_error = (speed_error(k) - previous_error) / dt;
    previous_error = speed_error(k);
    F_control(k) = Kp * speed_error(k) + Ki * integral_error + Kd * derivative_error;
    F_control(k) = max(min(F_control(k), F_max), F_min); % Apply force limits

    % --- 8.5: Vehicle Dynamics Update ---
    F_drag = 0.5 * rho * Cd * A * v_ego(k)^2;
    F_net = F_control(k) - F_drag;
    a_ego(k) = F_net / m;
    v_ego(k+1) = v_ego(k) + a_ego(k) * dt;
    v_ego(k+1) = max(0, min(v_ego(k+1), v_max_limit)); % Apply speed limits
    pos_ego(k+1) = pos_ego(k) + v_ego(k+1) * dt;

end % End of simulation loop
simulation_duration = toc;
fprintf('Simulation loop completed in %.2f seconds.\n', simulation_duration);

% --- Fill in final values for logged arrays ---
k = n_steps;
d_rel(k) = max(0, pos_lead(k) - pos_ego(k));
d_sensed(k) = d_sensed(k-1);
d_safe(k) = max(d_min, d_min + T_gap * v_ego(k));
v_target(k) = v_target(k-1);
speed_error(k) = v_target(k) - v_ego(k);
F_control(k) = F_control(k-1);
a_ego(k) = a_ego(k-1);

% ==========================================================================
% SECTION 9: POST-PROCESSING - PLOTTING RESULTS
% ==========================================================================
fprintf('Plotting simulation results...\n');
figure('Name', 'ACC Simulation Results via MATLAB Script (Hysteresis)', 'Position', [50, 50, 1000, 800]);

% --- Plot 1: Speeds vs. Time ---
ax1 = subplot(3, 1, 1);
plot(ax1, t, v_ego, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Ego Speed (v_{ego})'); hold(ax1, 'on');
plot(ax1, t, v_lead, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Lead Speed (v_{lead})');
plot(ax1, t, v_target, 'g:', 'LineWidth', 2, 'DisplayName', 'Target Speed (v_{target})');
plot(ax1, t, ones(size(t))*v_set, 'k-.', 'LineWidth', 1, 'DisplayName', 'Set Speed (v_{set})');
hold(ax1, 'off'); ylabel(ax1, 'Speed (m/s)'); title(ax1, 'Vehicle Speeds Over Time');
legend(ax1, 'show', 'Location', 'best'); grid(ax1, 'on'); xlim(ax1, [0 T_sim]);

% --- Plot 2: Distances vs. Time ---
ax2 = subplot(3, 1, 2);
plot(ax2, t, d_rel, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Actual Distance (d_{rel})'); hold(ax2, 'on');
plot(ax2, t, d_sensed, 'm-.', 'LineWidth', 1.5, 'DisplayName', 'Sensed Distance (d_{sensed})');
plot(ax2, t, d_safe, 'k:', 'LineWidth', 2.5, 'DisplayName', 'Safe Distance Threshold (d_{safe})');
hold(ax2, 'off'); ylabel(ax2, 'Distance (m)'); title(ax2, 'Relative Distances Over Time');
legend(ax2, 'show', 'Location', 'best'); grid(ax2, 'on'); xlim(ax2, [0 T_sim]);
ylim(ax2, [0, max(max(d_rel), max(d_safe))*1.1 + 10]);

if enable_spoofing % Highlight attack period
    ylim_vals = ylim(ax2);
    patch(ax2, [attack_start_time attack_end_time attack_end_time attack_start_time], ...
          [ylim_vals(1) ylim_vals(1) ylim_vals(2) ylim_vals(2)], ...
          'red', 'FaceAlpha', 0.1, 'EdgeColor', 'none', 'DisplayName', 'Spoofing Attack Interval');
    legend(ax2, 'show', 'Location', 'best'); % Update legend
end

% --- Plot 3: Acceleration vs. Time ---
ax3 = subplot(3, 1, 3);
plot(ax3, t, a_ego, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Actual Acceleration (a_{ego})'); hold(ax3, 'on');
plot(ax3, t, ones(size(t))*a_max, 'k--', 'LineWidth', 1, 'DisplayName', 'Max Accel Limit');
plot(ax3, t, ones(size(t))*a_min, 'k--', 'LineWidth', 1, 'DisplayName', 'Max Decel Limit');
hold(ax3, 'off'); ylabel(ax3, 'Acceleration (m/s^2)'); xlabel(ax3, 'Time (s)');
title(ax3, 'Ego Vehicle Acceleration Over Time'); legend(ax3, 'show', 'Location', 'best');
grid(ax3, 'on'); xlim(ax3, [0 T_sim]); ylim(ax3, [a_min - 0.5, a_max + 0.5]);

% --- Add Overall Figure Title ---
sgtitle('Adaptive Cruise Control Simulation Results (MATLAB Script with Hysteresis)', 'FontSize', 14, 'FontWeight', 'bold');

fprintf('Plotting complete.\n');
fprintf('Simulation Script Finished.\n');
