% =========================================================================
% test_mpc_tuning_final.m
%
% Fixed multi-stage tuning benchmark for the ABENICS nonlinear
% orientation-command MPC.
%
% Real signal chain represented in this MATLAB-only test:
%
%   q_ref
%      -> abenicsOrientationMPC
%      -> q_des_mpc
%      -> abenicsIK
%      -> theta_cmd
%      -> temporary second-order plant
%      -> theta_actual
%      -> abenicsFK
%      -> q_pred
%
% IMPORTANT:
% The temporary second-order plant is only for early MPC tuning.
% Final tuning must later be repeated with the real Simulink PID/plant.
% =========================================================================

clear;A
clc;
close all;

% Reset persistent theta history used by abenicsOrientationMPC.
clear abenicsOrientationMPC;

% Load coordinate, MPC, singularity, and temporary plant parameters.
run("params_abenics_coordinate.m");

% -------------------------------------------------------------------------
% Optional one-run tuning overrides
% -------------------------------------------------------------------------
% Change only ONE parameter group per run.
%
% Examples:
% params.mpc.wTrack = 150;
% params.mpc.wTerminal = 500;
% params.mpc.wSmooth = 20;
% params.mpc.wMotor = 10;
% params.mpc.wOmega = 1.0;
% params.mpc.wSingularity = 1500;
% params.mpc.maxQStep = deg2rad(1);

% -------------------------------------------------------------------------
% Fixed benchmark timing
% -------------------------------------------------------------------------
simulationTime = 7.0;
numSteps = round(simulationTime / params.Ts) + 1;
time = (0:numSteps - 1) * params.Ts;

% -------------------------------------------------------------------------
% Fixed benchmark orientations
% q = [roll; pitch; yaw]
% -------------------------------------------------------------------------
q_start = deg2rad([15; 10; 5]);

q_rollStep  = deg2rad([20; 10; 5]);
q_pitchStep = deg2rad([20; 15; 5]);
q_yawStep   = deg2rad([20; 15; 10]);
q_combined  = deg2rad([24; 18; 8]);
q_return    = q_start;

% -------------------------------------------------------------------------
% Initial conditions
% -------------------------------------------------------------------------
q_des_prev = q_start;

% Start theta_actual at the IK solution corresponding to q_start.
theta_actual = abenicsIK(q_start, params);
theta_actual = theta_actual(:);

% Start with zero MP-gear velocity.
omega_actual = zeros(4, 1);

% -------------------------------------------------------------------------
% Preallocate logs
% -------------------------------------------------------------------------
q_ref_log = zeros(3, numSteps);
q_des_log = zeros(3, numSteps);
q_pred_log = zeros(3, numSteps);
q_error_log = zeros(3, numSteps);

theta_cmd_log = zeros(4, numSteps);
theta_actual_log = zeros(4, numSteps);
theta_error_log = zeros(4, numSteps);

omega_log = zeros(4, numSteps);
alpha_log = zeros(4, numSteps);

singularity_log = zeros(1, numSteps);
q_des_step_log = zeros(3, numSteps);

% -------------------------------------------------------------------------
% Closed-loop benchmark
% -------------------------------------------------------------------------
for k = 1:numSteps

    % Fixed reference schedule:
    % 0.0-0.5 s : hold start
    % 0.5-1.5 s : roll-only step
    % 1.5-2.5 s : pitch-only step
    % 2.5-3.5 s : yaw-only step
    % 3.5-5.0 s : combined move
    % 5.0-7.0 s : return to start
    if time(k) < 0.5
        q_ref = q_start;
    elseif time(k) < 1.5
        q_ref = q_rollStep;
    elseif time(k) < 2.5
        q_ref = q_pitchStep;
    elseif time(k) < 3.5
        q_ref = q_yawStep;
    elseif time(k) < 5.0
        q_ref = q_combined;
    else
        q_ref = q_return;
    end

    % ---------------------------------------------------------------------
    % MPC outputs only q_des_mpc.
    % ---------------------------------------------------------------------
    q_des_mpc = abenicsOrientationMPC( ...
        q_ref, ...
        theta_actual, ...
        q_des_prev, ...
        params);

    q_des_mpc = q_des_mpc(:);

    % ---------------------------------------------------------------------
    % IK converts q_des_mpc into output-side MP-gear targets.
    % ---------------------------------------------------------------------
    theta_cmd = abenicsIK(q_des_mpc, params);
    theta_cmd = theta_cmd(:);

    % ---------------------------------------------------------------------
    % Temporary second-order plant used only for MATLAB tuning.
    % This approximates the combined PID + motor + mechanical response.
    % ---------------------------------------------------------------------
    theta_error = atan2( ...
        sin(theta_cmd - theta_actual), ...
        cos(theta_cmd - theta_actual));

    KpPlant = expandToVectorLocal(params.plant.KpPlant, 4);
    KdPlant = expandToVectorLocal(params.plant.KdPlant, 4);

    alpha = KpPlant .* theta_error - KdPlant .* omega_actual;

    % Enforce the temporary acceleration limit.
    alpha = min(max(alpha, -params.mpc.alphaMax(:)), params.mpc.alphaMax(:));

    % Integrate acceleration into velocity.
    omega_actual = omega_actual + params.Ts * alpha;

    % Enforce the temporary velocity limit.
    omega_actual = min(max( ...
        omega_actual, ...
        -params.mpc.omegaMax(:)), ...
         params.mpc.omegaMax(:));

    % Integrate velocity into MP-gear angle.
    theta_actual = theta_actual + params.Ts * omega_actual;

    % ---------------------------------------------------------------------
    % FK converts actual MP-gear angles into predicted CS-gear orientation.
    % ---------------------------------------------------------------------
    q_pred = abenicsFK(theta_actual, params);
    q_pred = q_pred(:);

    % Distance-based singularity measurement.
    [s, ~] = singularityMeasure(theta_actual, q_pred, params);

    % Wrapped orientation tracking error.
    q_error = atan2( ...
        sin(q_pred - q_ref), ...
        cos(q_pred - q_ref));

    % Wrapped MPC command step.
    q_des_step = atan2( ...
        sin(q_des_mpc - q_des_prev), ...
        cos(q_des_mpc - q_des_prev));

    % ---------------------------------------------------------------------
    % Log current timestep.
    % ---------------------------------------------------------------------
    q_ref_log(:, k) = q_ref;
    q_des_log(:, k) = q_des_mpc;
    q_pred_log(:, k) = q_pred;
    q_error_log(:, k) = q_error;

    theta_cmd_log(:, k) = theta_cmd;
    theta_actual_log(:, k) = theta_actual;
    theta_error_log(:, k) = theta_error;

    omega_log(:, k) = omega_actual;
    alpha_log(:, k) = alpha;

    singularity_log(k) = s;
    q_des_step_log(:, k) = q_des_step;

    % Previous MPC command for the next timestep.
    q_des_prev = q_des_mpc;
end

% -------------------------------------------------------------------------
% Convert logged angular values to degrees for plots and printed metrics
% -------------------------------------------------------------------------
q_ref_deg = rad2deg(q_ref_log);
q_des_deg = rad2deg(q_des_log);
q_pred_deg = rad2deg(q_pred_log);
q_error_deg = rad2deg(q_error_log);

theta_cmd_deg = rad2deg(theta_cmd_log);
theta_actual_deg = rad2deg(theta_actual_log);
theta_error_deg = rad2deg(theta_error_log);

omega_deg = rad2deg(omega_log);
alpha_deg = rad2deg(alpha_log);

singularity_deg = rad2deg(singularity_log);
q_des_step_deg = rad2deg(q_des_step_log);

orientationNames = ["Roll", "Pitch", "Yaw"];
thetaNames = ["theta rA", "theta pA", "theta rB", "theta pB"];

% =========================================================================
% FIGURE 1: q_ref, q_des_mpc, and q_pred
% =========================================================================
figure("Name", "ABENICS MPC Orientation Tracking");

for axisIndex = 1:3
    subplot(3, 1, axisIndex);

    plot(time, q_ref_deg(axisIndex, :), "--", "LineWidth", 1.5);
    hold on;
    plot(time, q_des_deg(axisIndex, :), "LineWidth", 1.5);
    plot(time, q_pred_deg(axisIndex, :), "LineWidth", 1.5);

    grid on;
    ylabel(orientationNames(axisIndex) + " (deg)");

    if axisIndex == 1
        title("ABENICS MPC Fixed Tuning Benchmark");
        legend("q ref", "q des MPC", "q pred", "Location", "best");
    end
end

xlabel("Time (s)");

% =========================================================================
% FIGURE 2: orientation tracking error
% =========================================================================
figure("Name", "ABENICS Orientation Tracking Error");

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
% FIGURE 3: singularity distance
% =========================================================================
figure("Name", "ABENICS Singularity Distance");

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
% FIGURE 4: theta_cmd and theta_actual
% =========================================================================
figure("Name", "ABENICS MP Gear Tracking");

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
% FIGURE 5: MP-gear velocity
% =========================================================================
figure("Name", "ABENICS MP Gear Velocity");

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
% FIGURE 6: MPC output step size
% =========================================================================
figure("Name", "ABENICS MPC Command Step Size");

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
% Summary metrics
% =========================================================================

% Exclude the initial hold period from the overall tracking metrics.
activeSamples = time >= 0.5;

rmsError = sqrt(mean(q_error_deg(:, activeSamples).^2, 2));
finalError = q_error_deg(:, end);

minimumSingularityDistance = min(singularity_deg);
maximumQStepObserved = max(abs(q_des_step_deg), [], 2);
maximumOmegaObserved = max(abs(omega_deg), [], 2);
maximumAlphaObserved = max(abs(alpha_deg), [], 2);

thetaCommandStep = zeros(size(theta_cmd_deg));
thetaCommandStep(:, 2:end) = rad2deg(atan2( ...
    sin(theta_cmd_log(:, 2:end) - theta_cmd_log(:, 1:end-1)), ...
    cos(theta_cmd_log(:, 2:end) - theta_cmd_log(:, 1:end-1))));

maximumThetaCommandStep = max(abs(thetaCommandStep), [], 2);

fprintf("\n============================================================\n");
fprintf("ABENICS MPC FIXED TUNING BENCHMARK\n");
fprintf("============================================================\n");

fprintf("\nMPC parameters used:\n");
fprintf("wTrack       = %.4f\n", params.mpc.wTrack);
fprintf("wTerminal    = %.4f\n", params.mpc.wTerminal);
fprintf("wSmooth      = %.4f\n", params.mpc.wSmooth);
fprintf("wMotor       = %.4f\n", params.mpc.wMotor);
fprintf("wSingularity = %.4f\n", params.mpc.wSingularity);
fprintf("wOmega       = %.4f\n", params.mpc.wOmega);
fprintf("maxQStep     = %.4f deg\n", rad2deg(params.mpc.maxQStep));

fprintf("\nOverall RMS orientation error after 0.5 s:\n");
fprintf("Roll:  %.4f deg\n", rmsError(1));
fprintf("Pitch: %.4f deg\n", rmsError(2));
fprintf("Yaw:   %.4f deg\n", rmsError(3));

fprintf("\nFinal orientation error:\n");
fprintf("Roll:  %.4f deg\n", finalError(1));
fprintf("Pitch: %.4f deg\n", finalError(2));
fprintf("Yaw:   %.4f deg\n", finalError(3));

fprintf("\nMaximum observed q_des_mpc step:\n");
fprintf("Roll:  %.4f deg\n", maximumQStepObserved(1));
fprintf("Pitch: %.4f deg\n", maximumQStepObserved(2));
fprintf("Yaw:   %.4f deg\n", maximumQStepObserved(3));

fprintf("\nMinimum singularity distance: %.4f deg\n", ...
    minimumSingularityDistance);

fprintf("Warning distance: %.4f deg\n", ...
    rad2deg(params.singularity.warningDistance));

fprintf("Danger distance: %.4f deg\n", ...
    rad2deg(params.singularity.dangerDistance));

fprintf("\nMaximum absolute theta_cmd step:\n");
for motorIndex = 1:4
    fprintf("%s: %.4f deg\n", ...
        thetaNames(motorIndex), ...
        maximumThetaCommandStep(motorIndex));
end

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

% -------------------------------------------------------------------------
% Phase-by-phase RMS error
% -------------------------------------------------------------------------
phaseNames = [ ...
    "Initial hold", ...
    "Roll-only step", ...
    "Pitch-only step", ...
    "Yaw-only step", ...
    "Combined move", ...
    "Return to start"];

phaseStarts = [0.0, 0.5, 1.5, 2.5, 3.5, 5.0];
phaseEnds   = [0.5, 1.5, 2.5, 3.5, 5.0, 7.0];

fprintf("\nPhase-by-phase RMS orientation error:\n");

for phaseIndex = 1:numel(phaseNames)
    phaseMask = time >= phaseStarts(phaseIndex) & time < phaseEnds(phaseIndex);

    phaseRms = sqrt(mean(q_error_deg(:, phaseMask).^2, 2));

    fprintf("%-18s | roll=%7.4f deg | pitch=%7.4f deg | yaw=%7.4f deg\n", ...
        phaseNames(phaseIndex), ...
        phaseRms(1), ...
        phaseRms(2), ...
        phaseRms(3));
end

% -------------------------------------------------------------------------
% Basic automatic checks
% -------------------------------------------------------------------------
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
