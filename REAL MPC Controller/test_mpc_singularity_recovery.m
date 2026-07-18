% =========================================================================
% test_mpc_singularity_recovery.m
%
% Tests persistent emergency-recovery hysteresis from the singular pose.
%
% Start:  q = [0; 0; 0] deg
% Target: q = [2; 2; 0] deg
%
% Required behavior:
%   1. Emergency recovery increases singularity distance.
%   2. Recovery remains active past dangerDistance.
%   3. Normal MPC is allowed to resume only after recoveryClearDistance.
%   4. The trajectory does not re-enter danger after clearing recovery.
%
% This test uses the same temporary second-order plant as the sine-wave test.
% Repeat final validation with the real Simulink PID/plant.
% =========================================================================

clear;
clc;
close all;

run("params_abenics_coordinate.m");

% =========================================================================
% TEST SETTINGS
% =========================================================================

simulationTime = 6.0;
q_start  = deg2rad([0; 0; 0]);
q_target = deg2rad([2; 2; 0]);

% Recovery must have freedom to move all three orientation axes.
params.mpc.maxQStep = deg2rad([2; 2; 2]);

% Enable only if the public MPC debug print is present. When enabled, this
% script captures emergencyRecovery, fallback, and accepted messages.
captureControllerDiagnostics = false;
params.mpc.debug = captureControllerDiagnostics;

% Shorter validation settings. Set false to use the parameter-file values.
useFastValidationSettings = true;

if useFastValidationSettings
    params.mpc.Np = 10;
    params.mpc.Nc = 3;
    params.mpc.maxIterations = 15;
    params.mpc.maxFunctionEvaluations = 1000;
end

if isfield(params.mpc, 'recoveryClearDistance')
    recoveryClearDistance = params.mpc.recoveryClearDistance;
else
    recoveryClearDistance = deg2rad(3);
end

clear abenicsOrientationMPC;

% =========================================================================
% INITIAL CONDITIONS
% =========================================================================

numSteps = round(simulationTime / params.Ts) + 1;
time = (0:numSteps - 1) * params.Ts;

q_des_prev = q_start;

theta_actual = abenicsIK(q_start, params);
theta_actual = theta_actual(:);

omega_actual = zeros(4, 1);

q_des_log = zeros(3, numSteps);
q_pred_log = zeros(3, numSteps);
q_error_log = zeros(3, numSteps);
q_step_log = zeros(3, numSteps);

theta_cmd_log = zeros(4, numSteps);
theta_actual_log = zeros(4, numSteps);
omega_log = zeros(4, numSteps);
alpha_log = zeros(4, numSteps);

singularity_log = zeros(1, numSteps);
pole_distance_log = zeros(6, numSteps);

fallback_log = false(1, numSteps);
accepted_log = false(1, numSteps);
emergency_log = false(1, numSteps);
diagnostic_text_found = false(1, numSteps);

% =========================================================================
% CLOSED-LOOP RECOVERY TEST
% =========================================================================

testTimer = tic;

for k = 1:numSteps

    if captureControllerDiagnostics
        debugText = evalc([ ...
            'q_des_mpc = abenicsOrientationMPC(' ...
            'q_target, theta_actual, q_des_prev, params);' ...
            ]);

        diagnostic_text_found(k) = ...
            contains(debugText, 'accepted=') || ...
            contains(debugText, 'fallback=');

        accepted_log(k) = contains(debugText, 'accepted=1');
        fallback_log(k) = contains(debugText, 'fallback=1');
        emergency_log(k) = contains(debugText, 'emergencyRecovery');
    else
        q_des_mpc = abenicsOrientationMPC( ...
            q_target, theta_actual, q_des_prev, params);
    end

    q_des_mpc = q_des_mpc(:);

    theta_cmd = abenicsIK(q_des_mpc, params);
    theta_cmd = theta_cmd(:);

    [theta_actual, omega_actual, alpha] = temporaryPlantStep( ...
        theta_actual, omega_actual, theta_cmd, params);

    q_pred = abenicsFK(theta_actual, params);
    q_pred = q_pred(:);

    [s, info] = singularityMeasure(theta_actual, q_pred, params);

    q_error = wrappedDifference(q_pred, q_target);
    q_step = wrappedDifference(q_des_mpc, q_des_prev);

    q_des_log(:, k) = q_des_mpc;
    q_pred_log(:, k) = q_pred;
    q_error_log(:, k) = q_error;
    q_step_log(:, k) = q_step;

    theta_cmd_log(:, k) = theta_cmd;
    theta_actual_log(:, k) = theta_actual;
    omega_log(:, k) = omega_actual;
    alpha_log(:, k) = alpha;

    singularity_log(k) = s;
    pole_distance_log(:, k) = info.poleDistances(:);

    q_des_prev = q_des_mpc;
end

totalRuntime = toc(testTimer);

% =========================================================================
% METRICS
% =========================================================================

singularity_deg = rad2deg(singularity_log);
q_des_deg = rad2deg(q_des_log);
q_pred_deg = rad2deg(q_pred_log);
q_error_deg = rad2deg(q_error_log);
q_step_deg = rad2deg(q_step_log);

dangerDistance = params.singularity.dangerDistance;
warningDistance = params.singularity.warningDistance;

firstDangerExitIndex = find( ...
    singularity_log >= dangerDistance, 1, 'first');

firstRecoveryClearIndex = find( ...
    singularity_log >= recoveryClearDistance, 1, 'first');

if isempty(firstDangerExitIndex)
    firstDangerExitTime = NaN;
else
    firstDangerExitTime = time(firstDangerExitIndex);
end

if isempty(firstRecoveryClearIndex)
    firstRecoveryClearTime = NaN;
else
    firstRecoveryClearTime = time(firstRecoveryClearIndex);
end

if isempty(firstRecoveryClearIndex)
    reenteredDangerAfterClear = true;
else
    reenteredDangerAfterClear = any( ...
        singularity_log(firstRecoveryClearIndex:end) < dangerDistance);
end

initialDistance = singularity_deg(1);
maximumDistance = max(singularity_deg);
finalDistance = singularity_deg(end);
distanceIncrease = maximumDistance - initialDistance;

finalOrientationError = abs(q_error_deg(:, end));
maximumQStepObserved = max(abs(q_step_deg), [], 2);

qCommandViolation = max([ ...
    max(params.mpc.qMin(:) - q_des_log, [], 'all'), ...
    max(q_des_log - params.mpc.qMax(:), [], 'all'), ...
    0]);

thetaCommandViolation = max([ ...
    max(params.mpc.thetaMin(:) - theta_cmd_log, [], 'all'), ...
    max(theta_cmd_log - params.mpc.thetaMax(:), [], 'all'), ...
    0]);

thetaPredViolation = max([ ...
    max(params.mpc.thetaMin(:) - theta_actual_log, [], 'all'), ...
    max(theta_actual_log - params.mpc.thetaMax(:), [], 'all'), ...
    0]);

omegaViolation = max([ ...
    max(-params.mpc.omegaMax(:) - omega_log, [], 'all'), ...
    max(omega_log - params.mpc.omegaMax(:), [], 'all'), ...
    0]);

alphaViolation = max([ ...
    max(-params.mpc.alphaMax(:) - alpha_log, [], 'all'), ...
    max(alpha_log - params.mpc.alphaMax(:), [], 'all'), ...
    0]);

maximumPhysicalViolation = max([ ...
    qCommandViolation, thetaCommandViolation, thetaPredViolation, ...
    omegaViolation, alphaViolation]);

if captureControllerDiagnostics && any(diagnostic_text_found)
    fallbackCount = sum(fallback_log);
    acceptedCount = sum(accepted_log);
    emergencyCount = sum(emergency_log);

    if isempty(firstRecoveryClearIndex)
        emergencyAfterClearCount = NaN;
    else
        emergencyAfterClearCount = sum( ...
            emergency_log(firstRecoveryClearIndex:end));
    end
else
    fallbackCount = NaN;
    acceptedCount = NaN;
    emergencyCount = NaN;
    emergencyAfterClearCount = NaN;
end

% =========================================================================
% PRINT RESULTS
% =========================================================================

fprintf("\n============================================================\n");
fprintf("ABENICS SINGULARITY RECOVERY TEST\n");
fprintf("============================================================\n");

fprintf("Start orientation:  [0, 0, 0] deg\n");
fprintf("Target orientation: [2, 2, 0] deg\n\n");

fprintf("Initial singularity distance: %.6f deg\n", initialDistance);
fprintf("Maximum singularity distance: %.6f deg\n", maximumDistance);
fprintf("Final singularity distance:   %.6f deg\n", finalDistance);
fprintf("Maximum distance increase:    %.6f deg\n", distanceIncrease);

fprintf("\nDanger distance:         %.6f deg\n", ...
    rad2deg(dangerDistance));
fprintf("Recovery-clear distance: %.6f deg\n", ...
    rad2deg(recoveryClearDistance));
fprintf("Warning distance:        %.6f deg\n", ...
    rad2deg(warningDistance));

fprintf("\nFirst danger exit time:    %.6f s\n", ...
    firstDangerExitTime);
fprintf("First recovery-clear time: %.6f s\n", ...
    firstRecoveryClearTime);
fprintf("Re-entered danger after clear: %d\n", ...
    reenteredDangerAfterClear);

fprintf("\nFinal absolute orientation error:\n");
fprintf("Roll:  %.6f deg\n", finalOrientationError(1));
fprintf("Pitch: %.6f deg\n", finalOrientationError(2));
fprintf("Yaw:   %.6f deg\n", finalOrientationError(3));

fprintf("\nMaximum q_des_mpc step:\n");
fprintf("Roll:  %.6f deg\n", maximumQStepObserved(1));
fprintf("Pitch: %.6f deg\n", maximumQStepObserved(2));
fprintf("Yaw:   %.6f deg\n", maximumQStepObserved(3));

fprintf("\nMaximum physical-limit violation: %.3e rad\n", ...
    maximumPhysicalViolation);

fprintf("Fallback count: %g\n", fallbackCount);
fprintf("Accepted continuous-MPC count: %g\n", acceptedCount);
fprintf("Emergency-recovery count: %g\n", emergencyCount);
fprintf("Emergency count after recovery clear: %g\n", ...
    emergencyAfterClearCount);

fprintf("\nTotal runtime: %.4f s\n", totalRuntime);
fprintf("Average controller-update time: %.6f s\n", ...
    totalRuntime / numSteps);
fprintf("Required update time at 50 Hz: %.6f s\n", params.Ts);

fprintf("\nAutomatic checks:\n");

if distanceIncrease > 0.1
    fprintf("PASS: singularity distance increased meaningfully\n");
else
    fprintf("FAIL: recovery made little or no progress\n");
end

if ~isnan(firstRecoveryClearTime)
    fprintf("PASS: measured state reached recoveryClearDistance\n");
else
    fprintf("FAIL: measured state never reached recoveryClearDistance\n");
end

if ~reenteredDangerAfterClear
    fprintf("PASS: trajectory did not re-enter danger after recovery clear\n");
else
    fprintf("FAIL: trajectory re-entered danger or never cleared recovery\n");
end

if maximumPhysicalViolation <= 1e-8
    fprintf("PASS: q, theta, omega, and alpha limits were respected\n");
else
    fprintf("FAIL: at least one physical limit was violated\n");
end

if captureControllerDiagnostics && any(diagnostic_text_found)
    if emergencyAfterClearCount == 0
        fprintf("PASS: emergency recovery stopped after recovery clear\n");
    else
        fprintf("FAIL: emergency recovery continued after recovery clear\n");
    end
else
    fprintf( ...
        "INFO: diagnostic capture was disabled or unavailable; internal mode transition was not directly verified\n");
end

% =========================================================================
% FIGURE 1: SINGULARITY RECOVERY
% =========================================================================

figure("Name", "ABENICS Singularity Recovery");

plot(time, singularity_deg, "LineWidth", 1.6);
hold on;

yline(rad2deg(dangerDistance), "--", "Danger distance");
yline(rad2deg(recoveryClearDistance), "--", "Recovery clear");
yline(rad2deg(warningDistance), "--", "Warning distance");

grid on;
xlabel("Time (s)");
ylabel("Nearest pole distance (deg)");
title("Persistent Singularity-Recovery Hysteresis");
legend("s", "Danger", "Recovery clear", "Warning", ...
    "Location", "best");

% =========================================================================
% FIGURE 2: ORIENTATION COMMAND AND RESPONSE
% =========================================================================

orientationNames = ["Roll", "Pitch", "Yaw"];

figure("Name", "ABENICS Recovery Orientation");

for axisIndex = 1:3
    subplot(3, 1, axisIndex);

    plot( ...
        time, ...
        rad2deg(q_target(axisIndex)) * ones(size(time)), ...
        "--", ...
        "LineWidth", 1.2);

    hold on;

    plot(time, q_des_deg(axisIndex, :), "LineWidth", 1.4);
    plot(time, q_pred_deg(axisIndex, :), "LineWidth", 1.4);

    grid on;
    ylabel(orientationNames(axisIndex) + " (deg)");

    if axisIndex == 1
        title("Recovery Command and Predicted Orientation");
        legend("q target", "q des MPC", "q pred", ...
            "Location", "best");
    end
end

xlabel("Time (s)");

% =========================================================================
% FIGURE 3: INDIVIDUAL POLE DISTANCES
% =========================================================================

figure("Name", "ABENICS Recovery Pole Distances");

poleNames = ["+X", "-X", "+Y", "-Y", "+Z", "-Z"];

hold on;

for poleIndex = 1:6
    plot( ...
        time, ...
        rad2deg(pole_distance_log(poleIndex, :)), ...
        "LineWidth", 1.2, ...
        "DisplayName", poleNames(poleIndex));
end

yline(rad2deg(dangerDistance), "--", "Danger distance");

grid on;
xlabel("Time (s)");
ylabel("Pole distance (deg)");
title("Six Physical Pole Distances");
legend("Location", "best");

% =========================================================================
% LOCAL FUNCTIONS
% =========================================================================

function [thetaNext, omegaNext, alpha] = temporaryPlantStep( ...
    theta, omega, thetaCmd, params)

    KpPlant = expandToVectorLocal(params.plant.KpPlant, 4);
    KdPlant = expandToVectorLocal(params.plant.KdPlant, 4);

    thetaError = wrappedDifference(thetaCmd, theta);

    alpha = KpPlant .* thetaError - KdPlant .* omega;
    alpha = min(max( ...
        alpha, -params.mpc.alphaMax(:)), params.mpc.alphaMax(:));

    omegaNext = omega + params.Ts * alpha;
    omegaNext = min(max( ...
        omegaNext, -params.mpc.omegaMax(:)), params.mpc.omegaMax(:));

    thetaNext = theta + params.Ts * omegaNext;
end

function d = wrappedDifference(a, b)
    d = atan2(sin(a - b), cos(a - b));
end

function v = expandToVectorLocal(value, n)
    value = value(:);

    if numel(value) == 1
        v = value * ones(n, 1);
    else
        v = value;
    end
end
