%% Final Adaptive Cruise Control Simulation Code
% This script models an adaptive cruise control (ACC) system with:
% - A PID controller that regulates vehicle speed.
% - A simulated distance sensor that measures the gap to a lead vehicle.
% - Adaptive logic that decelerates the vehicle if the distance is unsafe.
% - Injection of sensor spoofing events to simulate cyber-attack scenarios.

%% System Parameters
m = 1020;             % Vehicle mass [kg]
Cd = 1.2;             % Drag coefficient
plant_num = 8.56123e-4;
plant_den = [2 2/50.5355];
G_plant = tf(plant_num, plant_den); % Transfer function for plant dynamics

% PID Controller Gains (obtained from PID Tuner)
Kp = 179.7119;
Ki = 11.2187;
Kd = 0;

% Safety and Adaptive Control Parameters
safe_distance = 10;   % Minimum safe distance [m]
dt = 0.1;             % Time step [s]
sim_time = 100;       % Total simulation time [s]
time = 0:dt:sim_time; % Time vector

% Desired speed input (in Kph converted to m/s)
V_set_speed_kph = input('Enter set speed in Kph: ');
V_set_speed = V_set_speed_kph / 3.6;

%% Initialization
num_steps = length(time);
velocity = zeros(num_steps,1);       % Vehicle speed [m/s]
distance = zeros(num_steps,1);         % Inter-vehicle distance [m]
control_signal = zeros(num_steps,1);   % Control command signal
sensor_reading = zeros(num_steps,1);   % Simulated sensor measurement
spoofing_flag = zeros(num_steps,1);    % Indicator of spoofing events

% Initial Conditions
velocity(1) = 28.22;  % Initial vehicle speed [m/s]
distance(1) = 20;     % Initial gap between vehicles [m]

%% Simulation Loop
% Use a simple discrete-time loop to simulate the system dynamics
for k = 2:num_steps
    % --- Sensor Simulation with Spoofing ---
    % Every 20 seconds, there is a chance of sensor spoofing.
    if mod(time(k),20) < dt
        if rand() < 0.3  % 30% chance of attack
            % Simulate spoofed sensor reading (falsely high value)
            sensor_reading(k) = distance(k-1) + 20;
            spoofing_flag(k) = 1;
        else
            sensor_reading(k) = distance(k-1);
        end
    else
        sensor_reading(k) = distance(k-1);
    end
    
    % --- Adaptive Control Logic ---
    % If the sensor reading is below the safe distance, reduce the speed target.
    if sensor_reading(k) < safe_distance
        adaptive_set_speed = min(V_set_speed, velocity(k-1) - 2);  % Force deceleration
    else
        adaptive_set_speed = V_set_speed;
    end
    
    % --- PID Control Computation ---
    error = adaptive_set_speed - velocity(k-1);
    
    % Use simple discrete PID control with Euler integration
    persistent integral prev_error;
    if isempty(integral)
        integral = 0;
        prev_error = 0;
    end
    integral = integral + error * dt;
    derivative = (error - prev_error) / dt;
    control_signal(k) = Kp * error + Ki * integral + Kd * derivative;
    prev_error = error;
    
    % --- Vehicle Dynamics Simulation ---
    % Update vehicle speed with a basic model accounting for control input and drag.
    drag_effect = 0.02 * velocity(k-1);
    velocity(k) = velocity(k-1) + (control_signal(k) - drag_effect) * dt;
    
    % --- Update Inter-Vehicle Distance ---
    % Assume the lead vehicle travels at a constant speed slightly lower than the set speed.
    lead_vehicle_speed = V_set_speed - 2; 
    distance(k) = distance(k-1) + (lead_vehicle_speed - velocity(k-1)) * dt;
end

%% Plotting Results
figure;
subplot(3,1,1)
plot(time, velocity, 'LineWidth', 2); grid on;
xlabel('Time (s)'); ylabel('Vehicle Speed (m/s)');
title('Adaptive Cruise Control: Velocity Response');

subplot(3,1,2)
plot(time, distance, 'LineWidth', 2); grid on;
xlabel('Time (s)'); ylabel('Inter-Vehicle Distance (m)');
title('Distance between Vehicles');

subplot(3,1,3)
plot(time, sensor_reading, 'LineWidth', 2); grid on;
xlabel('Time (s)'); ylabel('Sensor Reading (m)');
title('Sensor Measurement with Occasional Spoofing');
