% =========================================================================
% test_mpc_sine_wave.m
%
% Sine-wave tracking test for the ABENICS nonlinear orientation-command MPC.
%
% Signal chain:
% q_ref -> MPC -> q_des_mpc -> IK -> theta_cmd -> temporary plant
%       -> theta_actual -> FK -> q_pred
%
% IMPORTANT:
% This temporary second-order plant is only for early MPC tuning.
% Repeat final tuning with the real Simulink PID/plant.
% =========================================================================

clear;
clc;
close all;

clear abenicsOrientationMPC;
run("params_abenics_coordinate.m");

params.mpc.debug = true;
simulationTime = 0.2;

params.mpc.Np = 10;
params.mpc.Nc = 3;
params.mpc.maxIterations = 5;
params.mpc.maxFunctionEvaluations = 300;

testStartTime = tic;

% =========================================================================
% TEST SETTINGS
% =========================================================================

simulationTime = 10;          % seconds
sineFrequency = 0.20;         % Hz
sineAmplitude = deg2rad(5);   % +/- 5 degrees

% 1 = roll, 2 = pitch, 3 = yaw
sineAxis = 1;

% q = [roll; pitch; yaw]
q_center = deg2rad([15; 10; 5]);

% Optional one-run tuning overrides.
% Change only one parameter group per run.
%
% params.mpc.wTrack = 150;
% params.mpc.wTerminal = 500;
% params.mpc.wSmooth = 20;
% params.mpc.wMotor = 10;
% params.mpc.wOmega = 1.0;
% params.mpc.wSingularity = 1500;
% params.mpc.maxQStep = deg2rad(1);

% =========================================================================
% TIME AND INITIAL CONDITIONS
% =========================================================================

numSteps = round(simulationTime / params.Ts) + 1;
time = (0:numSteps - 1) * params.Ts;

q_des_prev = q_center;

theta_actual = abenicsIK(q_center, params);
theta_actual = theta_actual(:);

omega_actual = zeros(4, 1);

% =========================================================================
% PREALLOCATE LOGS
% =========================================================================

q_ref_log = zeros(3, numSteps);
q_des_log = zeros(3, numSteps);
q_pred_log = zeros(3, numSteps);
q_error_log = zeros(3, numSteps);
q_des_step_log = zeros(3, numSteps);

theta_cmd_log = zeros(4, numSteps);
theta_actual_log = zeros(4, numSteps);
omega_log = zeros(4, numSteps);
alpha_log = zeros(4, numSteps);

singularity_log = zeros(1, numSteps);

% =========================================================================
% CLOSED-LOOP SINE-WAVE TEST
% =========================================================================

for k = 1:numSteps

    q_ref = q_center;
    q_ref(sineAxis) = q_center(sineAxis) ...
        + sineAmplitude * sin(2 * pi * sineFrequency * time(k));

    q_des_mpc = abenicsOrientationMPC( ...
        q_ref, ...
        theta_actual, ...
        q_des_prev, ...
        params);

    q_des_mpc = q_des_mpc(:);

    theta_cmd = abenicsIK(q_des_mpc, params);
    theta_cmd = theta_cmd(:);

    theta_error = atan2( ...
        sin(theta_cmd - theta_actual), ...
        cos(theta_cmd - theta_actual));

    KpPlant = expandToVectorLocal(params.plant.KpPlant, 4);
    KdPlant = expandToVectorLocal(params.plant.KdPlant, 4);

    alpha = KpPlant .* theta_error - KdPlant .* omega_actual;

    alpha = min(max( ...
        alpha, ...
        -params.mpc.alphaMax(:)), ...
         params.mpc.alphaMax(:));

    omega_actual = omega_actual + params.Ts * alpha;

    omega_actual = min(max( ...
        omega_actual, ...
        -params.mpc.omegaMax(:)), ...
         params.mpc.omegaMax(:));

    theta_actual = theta_actual + params.Ts * omega_actual;

    q_pred = abenicsFK(theta_actual, params);
    q_pred = q_pred(:);

    [s, ~] = singularityMeasure(theta_actual, q_pred, params);

    q_error = atan2( ...
        sin(q_pred - q_ref), ...
        cos(q_pred - q_ref));

    q_des_step = atan2( ...
        sin(q_des_mpc - q_des_prev), ...
        cos(q_des_mpc - q_des_prev));

    q_ref_log(:, k) = q_ref;
    q_des_log(:, k) = q_des_mpc;
    q_pred_log(:, k) = q_pred;
    q_error_log(:, k) = q_error;
    q_des_step_log(:, k) = q_des_step;

    theta_cmd_log(:, k) = theta_cmd;
    theta_actual_log(:, k) = theta_actual;
    omega_log(:, k) = omega_actual;
    alpha_log(:, k) = alpha;

    singularity_log(k) = s;

    q_des_prev = q_des_mpc;
end

% =========================================================================
% CONVERT TO DEGREES
% =========================================================================

q_ref_deg = rad2deg(q_ref_log);
q_des_deg = rad2deg(q_des_log);
q_pred_deg = rad2deg(q_pred_log);
q_error_deg = rad2deg(q_error_log);
q_des_step_deg = rad2deg(q_des_step_log);

theta_cmd_deg = rad2deg(theta_cmd_log);
theta_actual_deg = rad2deg(theta_actual_log);
omega_deg = rad2deg(omega_log);
alpha_deg = rad2deg(alpha_log);

singularity_deg = rad2deg(singularity_log);

orientationNames = ["Roll", "Pitch", "Yaw"];
thetaNames = ["theta rA", "theta pA", "theta rB", "theta pB"];

% =========================================================================
% FIGURE 1: ORIENTATION TRACKING
% =========================================================================

figure("Name", "ABENICS MPC Sine-Wave Tracking");

for axisIndex = 1:3
    subplot(3, 1, axisIndex);

    plot(time, q_ref_deg(axisIndex, :), "--", "LineWidth", 1.5);
    hold on;
    plot(time, q_des_deg(axisIndex, :), "LineWidth", 1.5);
    plot(time, q_pred_deg(axisIndex, :), "LineWidth", 1.5);

    grid on;
    ylabel(orientationNames(axisIndex) + " (deg)");

    if axisIndex == 1
        title("ABENICS MPC Sine-Wave Tracking");
        legend("q ref", "q des MPC", "q pred", "Location", "best");
    end
end

xlabel("Time (s)");

% =========================================================================
% FIGURE 2: ORIENTATION ERROR
% =========================================================================

figure("Name", "ABENICS Sine-Wave Tracking Error");

for axisIndex = 1:3
    subplot(3, 1, axisIndex);

    plot(time, q_error_deg(axisIndex, :), "LineWidth", 1.5);
    yline(0, "--");

    grid on;
    ylabel(orientationNames(axisIndex) + " error (deg)");

    if axisIndex == 1
        title("q pred Minus q ref");
    end
end

xlabel("Time (s)");

% =========================================================================
% FIGURE 3: SINGULARITY DISTANCE
% =========================================================================

figure("Name", "ABENICS Sine-Wave Singularity Distance");

plot(time, singularity_deg, "LineWidth", 1.5);
hold on;

yline(rad2deg(params.singularity.warningDistance), ...
    "--", "Warning distance");

yline(rad2deg(params.singularity.dangerDistance), ...
    "--", "Danger distance");

grid on;
xlabel("Time (s)");
ylabel("Singularity distance (deg)");
title("Distance-Based Singularity Measurement");
legend("s", "Warning distance", "Danger distance", "Location", "best");

% =========================================================================
% FIGURE 4: MP-GEAR TRACKING
% =========================================================================

figure("Name", "ABENICS Sine-Wave MP Gear Tracking");

for motorIndex = 1:4
    subplot(4, 1, motorIndex);

    plot(time, theta_cmd_deg(motorIndex, :), "--", "LineWidth", 1.5);
    hold on;
    plot(time, theta_actual_deg(motorIndex, :), "LineWidth", 1.5);

    grid on;
    ylabel(thetaNames(motorIndex) + " (deg)");

    if motorIndex == 1
        title("IK Command and Temporary Plant Response");
        legend("theta cmd", "theta actual", "Location", "best");
    end
end

xlabel("Time (s)");

% =========================================================================
% FIGURE 5: MP-GEAR VELOCITY
% =========================================================================

figure("Name", "ABENICS Sine-Wave MP Gear Velocity");

for motorIndex = 1:4
    subplot(4, 1, motorIndex);

    plot(time, omega_deg(motorIndex, :), "LineWidth", 1.5);
    hold on;
    yline(rad2deg(params.mpc.omegaMax(motorIndex)), "--");
    yline(-rad2deg(params.mpc.omegaMax(motorIndex)), "--");

    grid on;
    ylabel(thetaNames(motorIndex) + " (deg/s)");

    if motorIndex == 1
        title("Temporary Plant MP-Gear Velocity");
    end
end

xlabel("Time (s)");

% =========================================================================
% FIGURE 6: MPC COMMAND STEP SIZE
% =========================================================================

figure("Name", "ABENICS Sine-Wave MPC Step Size");

for axisIndex = 1:3
    subplot(3, 1, axisIndex);

    plot(time, q_des_step_deg(axisIndex, :), "LineWidth", 1.5);
    hold on;
    yline(rad2deg(params.mpc.maxQStep), "--");
    yline(-rad2deg(params.mpc.maxQStep), "--");

    grid on;
    ylabel("Delta " + orientationNames(axisIndex) + " (deg)");

    if axisIndex == 1
        title("Per-Sample q des MPC Step");
    end
end

xlabel("Time (s)");

% =========================================================================
% SUMMARY METRICS
% =========================================================================

firstCycleTime = 1 / sineFrequency;
steadyMask = time >= firstCycleTime;

if ~any(steadyMask)
    warning([ ...
        "Simulation is shorter than one sine-wave cycle. " ...
        "Steady-state metrics will use the full simulation instead."]);

    steadyMask = true(size(time));
end

rmsError = sqrt(mean(q_error_deg(:, steadyMask).^2, 2));
peakError = max(abs(q_error_deg(:, steadyMask)), [], 2);

minimumSingularityDistance = min(singularity_deg);
maximumQStepObserved = max(abs(q_des_step_deg), [], 2);
maximumOmegaObserved = max(abs(omega_deg), [], 2);
maximumAlphaObserved = max(abs(alpha_deg), [], 2);

referenceSteady = q_ref_deg(sineAxis, steadyMask);
predictedSteady = q_pred_deg(sineAxis, steadyMask);

referenceAmplitude = 0.5 * (max(referenceSteady) - min(referenceSteady));
predictedAmplitude = 0.5 * (max(predictedSteady) - min(predictedSteady));

if referenceAmplitude > 0
    amplitudeRatio = predictedAmplitude / referenceAmplitude;
else
    amplitudeRatio = NaN;
end

fprintf("\n============================================================\n");
fprintf("ABENICS MPC SINE-WAVE TEST\n");
fprintf("============================================================\n");

fprintf("Test axis: %s\n", orientationNames(sineAxis));
fprintf("Frequency: %.4f Hz\n", sineFrequency);
fprintf("Amplitude: %.4f deg\n", rad2deg(sineAmplitude));
fprintf("Center orientation: [%.4f %.4f %.4f] deg\n", ...
    rad2deg(q_center(1)), ...
    rad2deg(q_center(2)), ...
    rad2deg(q_center(3)));

fprintf("\nSteady periodic RMS orientation error:\n");
fprintf("Roll:  %.4f deg\n", rmsError(1));
fprintf("Pitch: %.4f deg\n", rmsError(2));
fprintf("Yaw:   %.4f deg\n", rmsError(3));

fprintf("\nSteady periodic peak absolute error:\n");
fprintf("Roll:  %.4f deg\n", peakError(1));
fprintf("Pitch: %.4f deg\n", peakError(2));
fprintf("Yaw:   %.4f deg\n", peakError(3));

fprintf("\nDriven-axis reference amplitude: %.4f deg\n", referenceAmplitude);
fprintf("Driven-axis q_pred amplitude:    %.4f deg\n", predictedAmplitude);
fprintf("Driven-axis amplitude ratio:     %.4f\n", amplitudeRatio);

fprintf("\nMinimum singularity distance: %.4f deg\n", ...
    minimumSingularityDistance);

fprintf("Warning distance: %.4f deg\n", ...
    rad2deg(params.singularity.warningDistance));

fprintf("Danger distance: %.4f deg\n", ...
    rad2deg(params.singularity.dangerDistance));

fprintf("\nMaximum observed q_des_mpc step:\n");
fprintf("Roll:  %.4f deg\n", maximumQStepObserved(1));
fprintf("Pitch: %.4f deg\n", maximumQStepObserved(2));
fprintf("Yaw:   %.4f deg\n", maximumQStepObserved(3));

fprintf("\nMaximum absolute MP-gear velocity:\n");
for motorIndex = 1:4
    fprintf("%s: %.4f deg/s\n", ...
        thetaNames(motorIndex), ...
        maximumOmegaObserved(motorIndex));
end

fprintf("\nMaximum absolute MP-gear acceleration:\n");
for motorIndex = 1:4
    fprintf("%s: %.4f deg/s^2\n", ...
        thetaNames(motorIndex), ...
        maximumAlphaObserved(motorIndex));
end

fprintf("\nAutomatic checks:\n");

if minimumSingularityDistance > rad2deg(params.singularity.dangerDistance)
    fprintf("PASS: trajectory stayed outside dangerDistance\n");
else
    fprintf("FAIL: trajectory reached or entered dangerDistance\n");
end

if all(maximumQStepObserved <= rad2deg(params.mpc.maxQStep) + 1e-8)
    fprintf("PASS: q_des_mpc respected maxQStep\n");
else
    fprintf("FAIL: q_des_mpc exceeded maxQStep\n");
end

if all(maximumOmegaObserved <= rad2deg(params.mpc.omegaMax(:)) + 1e-8)
    fprintf("PASS: temporary plant respected omegaMax\n");
else
    fprintf("FAIL: temporary plant exceeded omegaMax\n");
end

if all(maximumAlphaObserved <= rad2deg(params.mpc.alphaMax(:)) + 1e-8)
    fprintf("PASS: temporary plant respected alphaMax\n");
else
    fprintf("FAIL: temporary plant exceeded alphaMax\n");
end

fprintf("\nReminder: repeat final tuning with the real Simulink PID/plant.\n");

totalTestRuntime = toc(testStartTime);

fprintf("\nTotal test runtime: %.2f seconds\n", ...
    totalTestRuntime);

averageTimePerStep = totalTestRuntime / numSteps;

fprintf("Average time per MPC update: %.6f seconds\n", ...
    averageTimePerStep);

fprintf("Required time for 50 Hz: %.6f seconds\n", params.Ts);

% =========================================================================
% Local helper: allow scalar or vector plant gains
% =========================================================================
function v = expandToVectorLocal(value, n)

    value = value(:);

    if numel(value) == 1
        v = value * ones(n, 1);
    else
        v = value;
    end
end
