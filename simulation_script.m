%% Adaptive Cruise Control Simulation Script
% Author: [Your Name/Group]
% Date: [Current Date]
% Description:
% Simulates an ACC system with PID speed control, adaptive distance keeping,
% basic vehicle dynamics, and simulated sensor spoofing attacks.
% This script performs a discrete-time simulation in MATLAB.
% *** REQUIRES PARAMETER TUNING FOR REALISTIC BEHAVIOR ***

clear; % Clear variables from previous runs
clc;   % Clear the command window
close all; % Close all figures

fprintf('Starting ACC Simulation Script...\n');

% ==========================================================================
% SECTION 1: SIMULATION PARAMETERS
% ==========================================================================
T_sim = 60;         % Total simulation time (seconds)
dt = 0.05;          % Simulation time step (seconds)
                    % Smaller dt -> more accuracy, slower simulation
t = 0:dt:T_sim;     % Time vector for simulation steps

% ==========================================================================
% SECTION 2: VEHICLE PARAMETERS (Ego Vehicle - *** TUNE THESE ***)
% ==========================================================================
m = 1500;           % Ego vehicle mass (kg)
Cd = 0.3;           % Aerodynamic drag coefficient (unitless)
A = 2.5;            % Vehicle frontal area (m^2)
rho = 1.2;          % Air density (kg/m^3)
g = 9.81;           % Acceleration due to gravity (m/s^2)
                    % (Used for rolling resistance/gradient if added)

% --- Control & Physical Limits ---
a_max = 2.0;        % Maximum comfortable/engine acceleration (m/s^2)
a_min = -5.0;       % Maximum comfortable/braking deceleration (m/s^2) - Must be negative
F_max = m * a_max;  % Max engine/motor force (N) - Simplified relationship
F_min = m * a_min;  % Max braking force (N) - Simplified relationship
v_max_limit = 40;   % Absolute maximum speed allowed for the vehicle (m/s) (~144 km/h)

% ==========================================================================
% SECTION 3: ACC CONTROLLER PARAMETERS (*** TUNE THESE ***)
% ==========================================================================
v_set = 30;         % Driver's desired cruising speed (m/s) (~108 km/h)
T_gap = 1.8;        % Desired safe time gap to maintain behind lead vehicle (s)
d_min = 5;          % Minimum absolute safety distance cushion (m)

% ==========================================================================
% SECTION 4: PID SPEED CONTROLLER GAINS (*** CRITICAL TUNING REQUIRED ***)
% ==========================================================================
% These gains determine how the controller responds to speed errors.
% Tuning process often involves adjusting Kp first, then Ki, then Kd.
Kp = 800;           % Proportional gain (Units: N / (m/s)) - Affects reaction speed
Ki = 40;            % Integral gain (Units: N / (m/s*s)) - Affects steady-state error elimination
Kd = 150;           % Derivative gain (Units: N / (m/s^2)) - Affects damping and overshoot

% ==========================================================================
% SECTION 5: LEAD VEHICLE BEHAVIOR (Define Scenario)
% ==========================================================================
d_lead_initial_offset = 80; % Initial distance of lead vehicle ahead of ego vehicle (m)
v_lead = zeros(size(t));    % Pre-allocate lead vehicle speed array for efficiency

% --- Define Lead Vehicle Speed Profile Over Time ---
v_lead(t <= 10) = 25;               % Scenario Part 1: Constant speed (25 m/s)
v_lead(t > 10 & t <= 30) = 15;      % Scenario Part 2: Slows down (to 15 m/s)
v_lead(t > 30 & t <= 50) = 15;      % Scenario Part 3: Maintains lower speed
v_lead(t > 50) = 20;                % Scenario Part 4: Speeds up slightly (to 20 m/s)

% --- Calculate Lead Vehicle Position Over Time ---
% Assuming ego vehicle starts at position 0.
pos_lead = zeros(size(t));
pos_lead(1) = d_lead_initial_offset; % Initial position relative to origin
for k_lead = 1:length(t)-1
    pos_lead(k_lead+1) = pos_lead(k_lead) + v_lead(k_lead) * dt;
end

% ==========================================================================
% SECTION 6: SENSOR SPOOFING ATTACK PARAMETERS (Configure Scenario)
% ==========================================================================
enable_spoofing = true; % SET TO 'false' TO DISABLE ATTACK SIMULATION
attack_start_time = 25; % Time the simulated attack begins (s)
attack_end_time = 40;   % Time the simulated attack ends (s)

% --- Type of Spoofing ---
% 'offset': Adds a fixed value to the true distance.
% 'scale': Multiplies the true distance by a factor.
% (Could add 'noise', 'stuck', 'replay' types for more advanced simulation)
spoofing_type = 'offset';

% --- Spoofing Value ---
% For 'offset': The distance offset in meters (e.g., -30 means sensor reports 30m closer).
% For 'scale': The scaling factor (e.g., 0.5 means sensor reports half the actual distance).
spoofing_value = -30;

% ==========================================================================
% SECTION 7: INITIALIZATION (State Variables and Log Arrays)
% ==========================================================================
% Pre-allocate arrays to store simulation results for speed and efficiency
n_steps = length(t);
v_ego       = zeros(n_steps, 1); % Ego vehicle speed (m/s)
pos_ego     = zeros(n_steps, 1); % Ego vehicle position (m)
a_ego       = zeros(n_steps, 1); % Ego vehicle actual acceleration (m/s^2)
d_rel       = zeros(n_steps, 1); % Actual relative distance (m)
d_sensed    = zeros(n_steps, 1); % Sensed relative distance (potentially spoofed) (m)
d_safe      = zeros(n_steps, 1); % Calculated safe following distance (m)
v_target    = zeros(n_steps, 1); % Target speed for PID controller (m/s)
speed_error = zeros(n_steps, 1); % Speed error (v_target - v_ego)
F_control   = zeros(n_steps, 1); % Force calculated by PID controller (N)

% --- Set Initial Conditions for the Ego Vehicle ---
v_ego(1)    = 28.0; % Example: Start slightly below set speed
pos_ego(1)  = 0.0;  % Start at origin
d_rel(1)    = pos_lead(1) - pos_ego(1); % Initial actual distance
d_sensed(1) = d_rel(1); % Initially, sensed distance equals actual distance
v_target(1) = v_set;    % Initially, target the driver's set speed
d_safe(1)   = d_min + T_gap * v_ego(1); % Initial safe distance calculation

% --- PID Controller Internal State Variables ---
integral_error = 0.0;   % Accumulator for the integral term
previous_error = 0.0;   % Speed error from the previous time step (for derivative term)

% --- Display Configuration Summary ---
fprintf('Simulation Setup:\n');
fprintf(' Time: Total=%.1fs, Step=%.3fs\n', T_sim, dt);
fprintf(' Ego Vehicle: Mass=%.0fkg, Max Accel=%.1fm/s^2, Max Decel=%.1fm/s^2\n', m, a_max, a_min);
fprintf(' ACC Params: Set Speed=%.1fm/s, Time Gap=%.1fs, Min Distance=%.1fm\n', v_set, T_gap, d_min);
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
tic; % Start timer to measure simulation duration
for k = 1:n_steps-1 % Loop from the first step to the second-to-last step

    % --- 8.1: Calculate Actual Relative Distance ---
    d_rel(k) = pos_lead(k) - pos_ego(k);
    % Prevent distance from becoming negative if a simulated collision occurs
    if d_rel(k) < 0
        d_rel(k) = 0;
    end

    % --- 8.2: Sensor Model & Spoofing Injection ---
    % Start with ideal sensor reading (actual distance)
    current_d_sensed = d_rel(k);
    % Optional: Add sensor noise here
    % current_d_sensed = current_d_sensed + randn() * sensor_noise_std_dev;

    % Apply spoofing if enabled and within the attack time window
    if enable_spoofing && (t(k) >= attack_start_time) && (t(k) < attack_end_time)
        switch spoofing_type
            case 'offset'
                current_d_sensed = current_d_sensed + spoofing_value;
            case 'scale'
                current_d_sensed = current_d_sensed * spoofing_value;
            % Add cases for other spoofing types if implemented
        end
        % Ensure sensed distance doesn't become negative due to spoofing
        current_d_sensed = max(0, current_d_sensed);
    end
    d_sensed(k) = current_d_sensed; % Log the sensed distance

    % --- 8.3: Adaptive Logic - Determine Target Speed ---
    % Calculate the desired safe following distance based on current ego speed
    d_safe(k) = d_min + T_gap * v_ego(k);
    % Ensure calculated safe distance is at least the minimum cushion
    d_safe(k) = max(d_min, d_safe(k));

    % Compare sensed distance to safe distance to decide control mode
    if d_sensed(k) < d_safe(k)
        % Condition: Following distance is (or appears to be) unsafe (too close)
        % Control Strategy: Target the lead vehicle's current speed.
        % (More sophisticated strategies could aim for a speed that restores
        % the safe distance gap over time, or factor in relative speed).
        v_target(k) = v_lead(k);
    else
        % Condition: Following distance is safe.
        % Control Strategy: Target the driver's set cruising speed.
        v_target(k) = v_set;
    end

    % --- Clamp the target speed ---
    % Ensure target speed is not negative and does not exceed the vehicle's max limit
    v_target(k) = max(0, min(v_target(k), v_max_limit));

    % --- 8.4: PID Speed Controller Calculation ---
    % Calculate the current speed error
    speed_error(k) = v_target(k) - v_ego(k);

    % Calculate Integral Term (Accumulated Error)
    integral_error = integral_error + speed_error(k) * dt;
    % Optional: Implement anti-windup logic for the integral term here.
    % A simple method is to clamp the integral term itself, or freeze it
    % if the controller output is already saturated.
    % Example Clamp: max_integral = F_max / Ki; integral_error = max(min(integral_error, max_integral), -max_integral);

    % Calculate Derivative Term (Rate of Change of Error)
    % Uses the difference between current error and previous step's error
    derivative_error = (speed_error(k) - previous_error) / dt;
    previous_error = speed_error(k); % Store current error for the next step's derivative calculation

    % Calculate PID Control Output (Desired Force)
    F_control(k) = Kp * speed_error(k) + Ki * integral_error + Kd * derivative_error;

    % --- Apply Control Force Limits (Saturation) ---
    % Ensure the calculated force is within the vehicle's physical capabilities
    F_control(k) = max(min(F_control(k), F_max), F_min);

    % --- 8.5: Vehicle Dynamics Update ---
    % Calculate forces resisting motion (simplified)
    F_drag = 0.5 * rho * Cd * A * v_ego(k)^2; % Aerodynamic drag force
    % Optional: Add Rolling Resistance Force (e.g., Frr = Crr * m * g)
    % Optional: Add Gradient Force (e.g., Fg = m * g * sin(road_angle))

    % Calculate Net Force acting on the vehicle
    F_net = F_control(k) - F_drag; % (Subtract other resistance forces if added)

    % Calculate Actual Acceleration (Newton's Second Law: a = F/m)
    a_ego(k) = F_net / m;

    % --- Integrate state variables using Euler method ---
    % Update velocity for the *next* time step (k+1)
    v_ego(k+1) = v_ego(k) + a_ego(k) * dt;

    % Apply physical speed limits for the next step
    v_ego(k+1) = max(0, min(v_ego(k+1), v_max_limit));

    % Update position for the *next* time step (k+1)
    % Using v_ego(k+1) here is slightly more accurate/stable for position in Euler integration
    pos_ego(k+1) = pos_ego(k) + v_ego(k+1) * dt;

end % End of simulation loop (for k = 1:n_steps-1)

simulation_duration = toc; % Stop timer
fprintf('Simulation loop completed in %.2f seconds.\n', simulation_duration);

% --- Fill in final values for logged arrays (for plotting consistency) ---
k = n_steps; % Index for the final time step
d_rel(k) = pos_lead(k) - pos_ego(k);
d_rel(k) = max(0, d_rel(k)); % Ensure not negative
d_sensed(k) = d_sensed(k-1); % Assume sensor reading persists
d_safe(k) = d_min + T_gap * v_ego(k);
d_safe(k) = max(d_min, d_safe(k));
v_target(k) = v_target(k-1); % Assume target persists
speed_error(k) = v_target(k) - v_ego(k);
F_control(k) = F_control(k-1); % Assume control force persists
a_ego(k) = a_ego(k-1); % Assume acceleration persists

% ==========================================================================
% SECTION 9: POST-PROCESSING - PLOTTING RESULTS
% ==========================================================================
fprintf('Plotting simulation results...\n');

figure('Name', 'ACC Simulation Results via MATLAB Script', 'Position', [50, 50, 1000, 800]); % Create a figure window

% --- Plot 1: Speeds vs. Time ---
ax1 = subplot(3, 1, 1); % Create axes in 1st row of a 3x1 grid
plot(ax1, t, v_ego, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Ego Speed (v_{ego})'); hold(ax1, 'on');
plot(ax1, t, v_lead, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Lead Speed (v_{lead})');
plot(ax1, t, v_target, 'g:', 'LineWidth', 2, 'DisplayName', 'Target Speed (v_{target})');
plot(ax1, t, ones(size(t))*v_set, 'k-.', 'LineWidth', 1, 'DisplayName', 'Set Speed (v_{set})');
hold(ax1, 'off');
ylabel(ax1, 'Speed (m/s)');
title(ax1, 'Vehicle Speeds Over Time');
legend(ax1, 'show', 'Location', 'best');
grid(ax1, 'on');
xlim(ax1, [0 T_sim]); % Set x-axis limits to simulation time

% --- Plot 2: Distances vs. Time ---
ax2 = subplot(3, 1, 2); % Create axes in 2nd row
plot(ax2, t, d_rel, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Actual Distance (d_{rel})'); hold(ax2, 'on');
plot(ax2, t, d_sensed, 'm-.', 'LineWidth', 1.5, 'DisplayName', 'Sensed Distance (d_{sensed})');
plot(ax2, t, d_safe, 'k:', 'LineWidth', 2.5, 'DisplayName', 'Safe Distance Threshold (d_{safe})');
hold(ax2, 'off');
ylabel(ax2, 'Distance (m)');
title(ax2, 'Relative Distances Over Time');
legend(ax2, 'show', 'Location', 'best');
grid(ax2, 'on');
xlim(ax2, [0 T_sim]);
ylim(ax2, [0, max(max(d_rel), max(d_safe))*1.1 + 10]); % Adjust y-axis limit for better visibility

% --- Highlight Attack Period on Distance Plot ---
if enable_spoofing
    ylim_vals = ylim(ax2); % Get current y-axis limits
    patch(ax2, [attack_start_time attack_end_time attack_end_time attack_start_time], ...
          [ylim_vals(1) ylim_vals(1) ylim_vals(2) ylim_vals(2)], ...
          'red', 'FaceAlpha', 0.1, 'EdgeColor', 'none', 'DisplayName', 'Spoofing Attack Interval');
    fprintf(' -> Spoofing attack interval highlighted on distance plot.\n');
    legend(ax2, 'show', 'Location', 'best'); % Re-show legend to include patch
end

% --- Plot 3: Acceleration vs. Time ---
ax3 = subplot(3, 1, 3); % Create axes in 3rd row
plot(ax3, t, a_ego, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Actual Acceleration (a_{ego})'); hold(ax3, 'on');
plot(ax3, t, ones(size(t))*a_max, 'k--', 'LineWidth', 1, 'DisplayName', 'Max Accel Limit');
plot(ax3, t, ones(size(t))*a_min, 'k--', 'LineWidth', 1, 'DisplayName', 'Max Decel Limit');
hold(ax3, 'off');
ylabel(ax3, 'Acceleration (m/s^2)');
xlabel(ax3, 'Time (s)'); % Add x-axis label only to the bottom plot
title(ax3, 'Ego Vehicle Acceleration Over Time');
legend(ax3, 'show', 'Location', 'best');
grid(ax3, 'on');
xlim(ax3, [0 T_sim]);
ylim(ax3, [a_min - 0.5, a_max + 0.5]); % Adjust y limits slightly beyond saturation

% --- Add Overall Figure Title ---
sgtitle('Adaptive Cruise Control Simulation Results (MATLAB Script)', 'FontSize', 14, 'FontWeight', 'bold');

fprintf('Plotting complete.\n');

% ==========================================================================
% SECTION 10: OPTIONAL - SAVE RESULTS
% ==========================================================================
% saveData = table(t', v_ego, v_lead, v_target, d_rel, d_sensed, d_safe, a_ego);
% saveData.Properties.VariableNames = {'Time_s', 'EgoSpeed_mps', 'LeadSpeed_mps', 'TargetSpeed_mps', 'ActualDist_m', 'SensedDist_m', 'SafeDist_m', 'EgoAccel_mps2'};
% filename = 'acc_simulation_results.csv';
% writetable(saveData, filename);
% fprintf('Results saved to %s\n', filename);

fprintf('Simulation Script Finished.\n');
