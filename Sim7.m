clc;
clear;
close all;

%% Simulation Parameters
rng(42); % Set random seed for reproducibility

% Motor specifications (37kW tunnel booster fan)
motor.RatedPower = 37e3; % 37 kW
motor.RatedVoltage = 400; % 400 V
motor.RatedCurrent = motor.RatedPower / (sqrt(3) * motor.RatedVoltage / 1000); % Approx 53.4 A
motor.RatedSpeed = 1480; % RPM
motor.PolePairs = 2; % 4 poles = 2 pole pairs
motor.Frequency = 50; % Hz
motor.Slip = 0.0133; % Typical slip

% Simulation time parameters
simulationTime = 5; % seconds
samplingRate = 500; % Hz
numSamples = simulationTime * samplingRate;
dt = 1/samplingRate;

% Sensor parameters
sensors.VibrationRange = [0 20]; % g
sensors.AcousticRange = [30 120]; % dB
sensors.RPMAccuracy = 0.5; % %
sensors.CurrentAccuracy = 0.2; % %
sensors.VoltageAccuracy = 0.1; % %
sensors.TempRange = [20 120]; % °C

%% Create output directory
if ~exist('motor_fault_data', 'dir')
    mkdir('motor_fault_data');
end

%% Normal Operation Simulation
disp('Simulating normal operation...');
simulateAndSaveMotorData('normal', motor, sensors, numSamples, dt, 'motor_fault_data/normal_operation.csv');

%% Fault Simulations
faultTypes = {
    'broken_rotor_bar',...
    'bearing_damage_outer_race',...
    'bearing_damage_inner_race',...
    'bearing_damage_rolling_element',...
    'stator_winding_short',...
    'stator_winding_open',...
    'voltage_unbalance',...
    'rotor_eccentricity',...
    'phase_loss',...
    'belt_misalignment',...
    'mechanical_unbalance'};

for i = 1:length(faultTypes)
    fault = faultTypes{i};
    disp(['Simulating fault: ' fault ' (' num2str(i) '/' num2str(length(faultTypes)) ')']);
    
    filename = ['motor_fault_data/' strrep(fault, ' ', '_') '.csv'];
    simulateAndSaveMotorData(fault, motor, sensors, numSamples, dt, filename);
    
    clear data;
    pause(0.1);
end

%% Visualization - Individual Fault Comparisons
disp('Generating individual fault comparison plots...');
parameters = {
    'RPM', 'Motor Speed (RPM)';
    'Current_PhaseA', 'Phase A Current (A)';
    'Vibration_X', 'X-axis Vibration (g)';
    'Temperature', 'Temperature (°C)';
    'Acoustic', 'Acoustic Noise (dB)'
};

% Load normal operation data
normalData = readtable('motor_fault_data/normal_operation.csv');

% Create directory for comparison plots
if ~exist('fault_comparisons', 'dir')
    mkdir('fault_comparisons');
end

% Generate comparison plots for each fault
for i = 1:length(faultTypes)
    fault = faultTypes{i};
    faultFilename = ['motor_fault_data/' strrep(fault, ' ', '_') '.csv'];
    faultData = readtable(faultFilename);
    
    % Ensure both datasets have same length
    minSamples = min(height(normalData), height(faultData));
    time = normalData.Time(1:minSamples);
    
    % Create figure for this fault
    fig = figure('Units', 'normalized', 'Position', [0.1 0.1 0.8 0.8], 'Visible', 'off');
    sgtitle(['Fault: ' strrep(fault, '_', ' ')], 'FontSize', 14, 'FontWeight', 'bold');
    
    % Plot each parameter
    for p = 1:size(parameters,1)
        param = parameters{p,1};
        param_label = parameters{p,2};
        
        subplot(3,2,p);
        hold on;
        grid on;
        
        % Plot normal operation
        plot(time, normalData.(param)(1:minSamples), 'b', 'LineWidth', 1.5, 'DisplayName', 'Normal');
        
        % Plot fault condition
        plot(time, faultData.(param)(1:minSamples), 'r', 'LineWidth', 1.5, 'DisplayName', 'Fault');
        
        xlabel('Time (s)');
        ylabel(param_label);
        title(strrep(param, '_', ' '));
        legend('Location', 'best');
        
        % Add statistical comparison
        normalMean = mean(normalData.(param)(1:minSamples));
        faultMean = mean(faultData.(param)(1:minSamples));
        changePercent = (faultMean - normalMean)/normalMean * 100;
        
        text(0.02, 0.98, sprintf('Δ = %.1f%%', changePercent),...
            'Units', 'normalized', 'VerticalAlignment', 'top',...
            'FontSize', 10, 'Color', 'k', 'FontWeight', 'bold');
    end
    
    % Save the figure
    saveas(fig, sprintf('fault_comparisons/%s_comparison.png', fault));
    close(fig);
end

%% Generate Statistical Summary Table
disp('Generating statistical summary...');
% Create combined fault types list including normal operation
allConditions = [{'normal'}; faultTypes(:)];
varNames = {'Condition'};
for p = 1:size(parameters,1)
    param = parameters{p,1};
    varNames = [varNames, {[param '_mean'], [param '_std'], [param '_min'], [param '_max']}];
end

statsTable = table('Size', [length(allConditions), length(varNames)],...
    'VariableTypes', [{'string'}, repmat({'double'}, 1, length(varNames)-1)],...
    'VariableNames', varNames);

for i = 1:length(allConditions)
    condition = allConditions{i};
    if strcmp(condition, 'normal')
        filename = 'motor_fault_data/normal_operation.csv';
    else
        filename = ['motor_fault_data/' strrep(condition, ' ', '_') '.csv'];
    end
    data = readtable(filename);
    
    statsTable.Condition(i) = condition;
    
    for p = 1:size(parameters,1)
        param = parameters{p,1};
        statsTable{i, [param '_mean']} = mean(data.(param));
        statsTable{i, [param '_std']} = std(data.(param));
        statsTable{i, [param '_min']} = min(data.(param));
        statsTable{i, [param '_max']} = max(data.(param));
    end
end

% Save statistics to CSV
writetable(statsTable, 'fault_statistics.csv');
disp('Statistical summary saved to fault_statistics.csv');

%% Supporting Functions
function simulateAndSaveMotorData(condition, motor, sensors, numSamples, dt, filename)
    batchSize = min(1000, numSamples);
    numBatches = ceil(numSamples / batchSize);
    
    fid = fopen(filename, 'w');
    if fid == -1
        error(['Cannot create file: ' filename]);
    end
    
    fprintf(fid, 'Time,RPM,Current_PhaseA,Current_PhaseB,Current_PhaseC,');
    fprintf(fid, 'Voltage_PhaseA,Voltage_PhaseB,Voltage_PhaseC,');
    fprintf(fid, 'Temperature,Vibration_X,Vibration_Y,Vibration_Z,Acoustic\n');
    
    [faultSeverity, params, motorAdj] = adjustFaultParameters(condition, motor);
    
    for batch = 1:numBatches
        startIdx = (batch-1) * batchSize + 1;
        endIdx = min(batch * batchSize, numSamples);
        currentBatchSize = endIdx - startIdx + 1;
        
        timeVector = ((startIdx-1):(endIdx-1)) * dt;
        timeVector = timeVector(:);
        
        data = generateMotorBatch(timeVector, motorAdj, sensors, condition, faultSeverity, params);
        
        for i = 1:currentBatchSize
            fprintf(fid, '%.6f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.4f,%.4f,%.4f,%.2f\n', ...
                data.Time(i), data.RPM(i), ...
                data.Current_PhaseA(i), data.Current_PhaseB(i), data.Current_PhaseC(i), ...
                data.Voltage_PhaseA(i), data.Voltage_PhaseB(i), data.Voltage_PhaseC(i), ...
                data.Temperature(i), data.Vibration_X(i), data.Vibration_Y(i), ...
                data.Vibration_Z(i), data.Acoustic(i));
        end
        
        clear data timeVector;
        
        if mod(batch, 10) == 0 || batch == numBatches
            fprintf('  Progress: %d/%d batches completed\n', batch, numBatches);
        end
    end
    
    fclose(fid);
    disp(['Saved: ' filename]);
end

function data = generateMotorBatch(timeVector, motor, sensors, condition, faultSeverity, params)
    batchSize = length(timeVector);
    data = struct();
    data.Time = timeVector;
    
    electricalFrequency = motor.Frequency;
    mechanicalFrequency = electricalFrequency / motor.PolePairs;
    
    data.RPM = generateRPMBatch(timeVector, motor, faultSeverity, condition, params, sensors);
    rpmHz = data.RPM / 60;
    
    currentMag = motor.RatedCurrent;
    phaseShift = 2*pi/3;
    currentNoise = (sensors.CurrentAccuracy/100) * currentMag;
    
    if strcmp(condition, 'phase_loss')
        data.Current_PhaseA = currentMag * sin(2*pi*electricalFrequency*timeVector) + currentNoise*randn(batchSize,1);
        data.Current_PhaseB = zeros(batchSize,1);
        data.Current_PhaseC = currentMag * sin(2*pi*electricalFrequency*timeVector + phaseShift) + currentNoise*randn(batchSize,1);
    else
        data.Current_PhaseA = currentMag * sin(2*pi*electricalFrequency*timeVector) + currentNoise*randn(batchSize,1);
        data.Current_PhaseB = currentMag * sin(2*pi*electricalFrequency*timeVector - phaseShift) + currentNoise*randn(batchSize,1);
        data.Current_PhaseC = currentMag * sin(2*pi*electricalFrequency*timeVector + phaseShift) + currentNoise*randn(batchSize,1);
    end
    
    voltageMag = motor.RatedVoltage;
    voltageNoise = (sensors.VoltageAccuracy/100) * voltageMag;
    
    if strcmp(condition, 'voltage_unbalance')
        voltageMagB = voltageMag * (1 - 0.05*faultSeverity);
        voltageMagC = voltageMag * (1 - 0.025*faultSeverity);
    else
        voltageMagB = voltageMag;
        voltageMagC = voltageMag;
    end
    
    data.Voltage_PhaseA = voltageMag * sin(2*pi*electricalFrequency*timeVector) + voltageNoise*randn(batchSize,1);
    data.Voltage_PhaseB = voltageMagB * sin(2*pi*electricalFrequency*timeVector - phaseShift) + voltageNoise*randn(batchSize,1);
    data.Voltage_PhaseC = voltageMagC * sin(2*pi*electricalFrequency*timeVector + phaseShift) + voltageNoise*randn(batchSize,1);
    
    baseTemp = 40 + 10*randn();
    if contains(condition, {'broken_rotor_bar', 'stator_winding', 'phase_loss'})
        baseTemp = baseTemp + 30 * faultSeverity;
    elseif contains(condition, 'bearing_damage')
        baseTemp = baseTemp + 20 * faultSeverity;
    end
    
    data.Temperature = baseTemp + timeVector * 2 * faultSeverity + 2*randn(batchSize,1);
    data.Temperature = max(min(data.Temperature, sensors.TempRange(2)), sensors.TempRange(1));
    
    baseVib = 0.1 + 0.05*randn(batchSize,1);
    vib1x = 0.3 * sin(2*pi*rpmHz.*timeVector);
    
    if strcmp(condition, 'broken_rotor_bar')
        sidebandFreq = mean(rpmHz) * motor.Slip;
        vibSidebands = 0.2*faultSeverity * sin(2*pi*sidebandFreq*timeVector);
        baseVib = baseVib + vibSidebands;
    elseif contains(condition, 'bearing_damage') && isfield(params, 'BearingDefectFrequency')
        vibDefect = 0.5*faultSeverity * sin(2*pi*params.BearingDefectFrequency*timeVector);
        baseVib = baseVib + vibDefect;
    elseif strcmp(condition, 'mechanical_unbalance')
        vib1x = vib1x * (1 + 3*faultSeverity);
    end
    
    vibTotal = baseVib + vib1x;
    vibTotal = max(min(vibTotal, sensors.VibrationRange(2)), sensors.VibrationRange(1));
    
    data.Vibration_X = vibTotal + 0.02*randn(batchSize,1);
    data.Vibration_Y = 0.9*vibTotal + 0.02*randn(batchSize,1);
    data.Vibration_Z = 0.7*vibTotal + 0.02*randn(batchSize,1);
    
    baseNoise = 70 + 5*randn(batchSize,1);
    if strcmp(condition, 'broken_rotor_bar')
        sidebandFreq = mean(rpmHz) * motor.Slip;
        acousticMod = 8*faultSeverity * sin(2*pi*sidebandFreq*timeVector);
        baseNoise = baseNoise + acousticMod;
    elseif contains(condition, 'bearing_damage') && isfield(params, 'BearingDefectFrequency')
        acousticDefect = 12*faultSeverity * sin(2*pi*params.BearingDefectFrequency*timeVector);
        baseNoise = baseNoise + acousticDefect;
    end
    
    data.Acoustic = max(min(baseNoise, sensors.AcousticRange(2)), sensors.AcousticRange(1));
end

function rpmSignal = generateRPMBatch(timeVector, motor, faultSeverity, condition, params, sensors)
    baseRPM = motor.RatedSpeed;
    rpmFluctuation = 0.02 * baseRPM;
    
    if contains(condition, {'broken_rotor_bar', 'stator_winding_short', 'stator_winding_open'})
        baseRPM = baseRPM * (1 - 0.03*faultSeverity);
        rpmFluctuation = rpmFluctuation * (1 + faultSeverity);
    elseif strcmp(condition, 'phase_loss')
        baseRPM = baseRPM * 0.85;
        rpmFluctuation = rpmFluctuation * 2;
    elseif strcmp(condition, 'voltage_unbalance')
        baseRPM = baseRPM * (1 - 0.01*faultSeverity);
    end
    
    rpmSignal = baseRPM + rpmFluctuation*sin(2*pi*0.1*timeVector) + ...
        0.2*rpmFluctuation*randn(size(timeVector));
    
    if contains(condition, 'bearing_damage') && isfield(params, 'BearingDefectFrequency')
        rpmSignal = rpmSignal .* (1 + 0.05*faultSeverity*sin(2*pi*params.BearingDefectFrequency*timeVector));
    end
    
    rpmNoise = (sensors.RPMAccuracy/100) * baseRPM * randn(size(rpmSignal));
    rpmSignal = rpmSignal + rpmNoise;
end

function [faultSeverity, params, motor] = adjustFaultParameters(condition, motor)
    faultSeverity = 0;
    params = struct('Condition', condition);
    
    switch condition
        case 'normal'
            
        case 'broken_rotor_bar'
            faultSeverity = 0.7;
            params.BrokenBars = ceil(faultSeverity * 4);
            
        case 'bearing_damage_outer_race'
            faultSeverity = 0.6;
            params.BearingDefectFrequency = 3.2 * (motor.Frequency / motor.PolePairs);
            
        case 'bearing_damage_inner_race'
            faultSeverity = 0.65;
            params.BearingDefectFrequency = 4.8 * (motor.Frequency / motor.PolePairs);
            
        case 'bearing_damage_rolling_element'
            faultSeverity = 0.55;
            params.BearingDefectFrequency = 2.9 * (motor.Frequency / motor.PolePairs);
            
        case 'stator_winding_short'
            faultSeverity = 0.8;
            params.ShortCircuitPercentage = 15 * faultSeverity;
            motor.RatedCurrent = motor.RatedCurrent * (1 + 0.3*faultSeverity);
            
        case 'stator_winding_open'
            faultSeverity = 1.0;
            params.OpenWindings = 1;
            motor.RatedCurrent = motor.RatedCurrent * 1.5;
            
        case 'voltage_unbalance'
            faultSeverity = 0.4;
            params.VoltageUnbalance = 5 * faultSeverity;
            
        case 'rotor_eccentricity'
            faultSeverity = 0.5;
            params.Eccentricity = 15 * faultSeverity;
            
        case 'phase_loss'
            faultSeverity = 1.0;
            params.PhaseLoss = 1;
            
        case 'belt_misalignment'
            faultSeverity = 0.45;
            params.MisalignmentAngle = 2 * faultSeverity;
            
        case 'mechanical_unbalance'
            faultSeverity = 0.5;
            params.UnbalanceMass = 0.1 * faultSeverity;
            params.UnbalanceRadius = 0.1;
    end
end