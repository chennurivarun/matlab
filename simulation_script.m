% simulation_script.m
% This script runs the ACC simulation and plots the results.

% Open the Simulink model
open_system('ACC_Model.slx');

% Set simulation time (adjust as necessary)
set_param('ACC_Model', 'StopTime', '20');

% Run the simulation
sim('ACC_Model');

% (Optional) Plot results if your model sends signals to the MATLAB workspace.
% Example:
% plot(simout.time, simout.signals.values);
% xlabel('Time (s)');
% ylabel('Vehicle Speed');
% title('ACC System Simulation');
