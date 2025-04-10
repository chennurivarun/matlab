% simulation_script.m
% Enhanced script to run ACC simulations with parameter sweeps and plotting

%% Setup
modelName = 'ACC_Model';
open_system(modelName);

% Simulation parameters
simTime = 20; % seconds
set_param(modelName, 'StopTime', num2str(simTime));

% Define initial speeds to test (m/s)
initialSpeeds = [15, 20, 25]; % Example speeds

% Preallocate results storage
results = struct();

%% Parameter sweep over initial speeds
for idx = 1:length(initialSpeeds)
    initSpeed = initialSpeeds(idx);
    
    % Set initial speed in the model (assumes variable 'initial_speed' exists)
    try
        set_param([modelName '/Vehicle Dynamics'], 'InitialSpeed', num2str(initSpeed));
    catch
        % If parameter not found, warn and continue
        warning('Initial speed parameter not set in model. Please configure manually.');
    end
    
    % Run simulation
    simOut = sim(modelName, 'StopTime', num2str(simTime));
    
    % Store results
    results(idx).speed = initSpeed;
    results(idx).simOut = simOut;
    
    % Extract signals (assuming 'simout' logged to workspace)
    try
        simout = simOut.get('simout');
        time = simout.time;
        values = simout.signals.values;
        
        % Plot speed profile
        figure;
        plot(time, values);
        xlabel('Time (s)');
        ylabel('Vehicle Speed (m/s)');
        title(['ACC Simulation - Initial Speed: ' num2str(initSpeed) ' m/s']);
        grid on;
    catch
        warning('No logged signal ''simout'' found. Please configure signal logging in the model.');
    end
end

disp('Simulations completed.');
