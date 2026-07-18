% =========================================================================
% test_mpc_singularity_warning_sweep.m
%
% Tunes params.mpc.wSingularity while the direct path crosses the current
% distance-based singularity at q = [0; 0; 0].
%
% Start:  q = [0;  5; 0] deg   (inside warning, outside danger)
% Target: q = [0; -5; 0] deg   (inside warning, outside danger)
%
% A direct pitch-only route passes through the singular point. A successful
% MPC should use a coordinated roll/yaw detour while respecting dangerDistance.
%
% This test uses the same temporary second-order plant as the sine-wave test.j
% Repeat final validation with the real Simulink PID/plant.
% =========================================================================

clear;
clc;
close all;

run("params_abenics_coordinate.m");

% =========================================================================
% TEST SETTINGS
% =========================================================================

simulationTime = 1;
q_start  = deg2rad([0;  15; 0]);
q_target = deg2rad([0; -15; 0]);

% Singularity avoidance needs freedom to use all three orientation axes.
params.mpc.maxQStep = deg2rad([2; 2; 2]);

wSingularityValues = [2000];

% Enable only if the public MPC debug print is present. When enabled, this
% script captures accepted=, fallback=, and reason= from each controller call.
captureControllerDiagnostics = true;

% Shorter validation settings. Set false to use the parameter-file values.
useFastValidationSettings = true;

if useFastValidationSettings
    params.mpc.Np = 20;
    params.mpc.Nc = 6;
    params.mpc.maxIterations = 25;
    params.mpc.maxFunctionEvaluations = 2000;
end

baseParams = params;
numWeights = numel(wSingularityValues);
results = repmat(struct(), 1, numWeights);

% =========================================================================
% RUN EVERY SINGULARITY WEIGHT
% =========================================================================

for weightIndex = 1:numWeights

    params = baseParams;
    params.mpc.wSingularity = wSingularityValues(weightIndex);
    params.mpc.debug = captureControllerDiagnostics;

    clear abenicsOrientationMPC;

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

    testTimer = tic;

    for k = 1:numSteps

        if captureControllerDiagnostics
            debugText = evalc([ ...
                'q_des_mpc = abenicsOrientationMPC(' ...
                'q_target, theta_actual, q_des_prev, params);' ...
                ]);

            fprintf('%s', debugText);

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

    singularity_deg = rad2deg(singularity_log);
    q_pred_deg = rad2deg(q_pred_log);
    q_error_deg = rad2deg(q_error_log);
    q_step_deg = rad2deg(q_step_log);

    minimumSingularityDistance = min(singularity_deg);
    timeBelowWarning = sum( ...
        singularity_log < params.singularity.warningDistance) * params.Ts;

    maximumRollDetour = max(abs(q_pred_deg(1, :) - rad2deg(q_target(1))));
    maximumYawDetour  = max(abs(q_pred_deg(3, :) - rad2deg(q_target(3))));

    finalError = abs(q_error_deg(:, end));
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

    dangerViolation = max( ...
        params.singularity.dangerDistance - ...
        min(pole_distance_log, [], 'all'), ...
        0);

    maximumPhysicalViolation = max([ ...
        qCommandViolation, thetaCommandViolation, thetaPredViolation, ...
        omegaViolation, alphaViolation, dangerViolation]);

    if captureControllerDiagnostics && any(diagnostic_text_found)
        fallbackCount = sum(fallback_log);
        acceptedCount = sum(accepted_log);
        emergencyCount = sum(emergency_log);
    else
        fallbackCount = NaN;
        acceptedCount = NaN;
        emergencyCount = NaN;
    end

    results(weightIndex).wSingularity = wSingularityValues(weightIndex);
    results(weightIndex).time = time;
    results(weightIndex).qDesDeg = rad2deg(q_des_log);
    results(weightIndex).qPredDeg = q_pred_deg;
    results(weightIndex).singularityDeg = singularity_deg;

    results(weightIndex).minimumSingularityDistance = ...
        minimumSingularityDistance;
    results(weightIndex).timeBelowWarning = timeBelowWarning;
    results(weightIndex).maximumRollDetour = maximumRollDetour;
    results(weightIndex).maximumYawDetour = maximumYawDetour;
    results(weightIndex).finalPitchError = finalError(2);
    results(weightIndex).maximumQStep = max(maximumQStepObserved);
    results(weightIndex).maximumPhysicalViolation = ...
        maximumPhysicalViolation;
    results(weightIndex).fallbackCount = fallbackCount;
    results(weightIndex).acceptedCount = acceptedCount;
    results(weightIndex).emergencyCount = emergencyCount;
    results(weightIndex).totalRuntime = totalRuntime;
    results(weightIndex).averageUpdateTime = totalRuntime / numSteps;
end

% =========================================================================
% SUMMARY TABLE
% =========================================================================

summaryTable = table( ...
    [results.wSingularity].', ...
    [results.minimumSingularityDistance].', ...
    [results.timeBelowWarning].', ...
    [results.maximumRollDetour].', ...
    [results.maximumYawDetour].', ...
    [results.finalPitchError].', ...
    [results.maximumQStep].', ...
    [results.maximumPhysicalViolation].', ...
    [results.fallbackCount].', ...
    [results.averageUpdateTime].', ...
    'VariableNames', { ...
        'wSingularity', ...
        'MinimumDistance_deg', ...
        'TimeBelowWarning_s', ...
        'MaximumRollDetour_deg', ...
        'MaximumYawDetour_deg', ...
        'FinalPitchError_deg', ...
        'MaximumQStep_deg', ...
        'MaximumPhysicalViolation_rad', ...
        'FallbackCount', ...
        'AverageUpdateTime_s'});

fprintf("\n============================================================\n");
fprintf("ABENICS WARNING-ZONE SINGULARITY WEIGHT SWEEP\n");
fprintf("============================================================\n");
fprintf("Start:  [0,  15, 0] deg\n");
fprintf("Target: [0, -15, 0] deg\n");
fprintf("Warning distance: %.4f deg\n", ...
    rad2deg(baseParams.singularity.warningDistance));
fprintf("Danger distance:  %.4f deg\n\n", ...
    rad2deg(baseParams.singularity.dangerDistance));

disp(summaryTable);

fprintf("Interpretation:\n");
fprintf("- MinimumDistance must remain at or above dangerDistance.\n");
fprintf("- Lower TimeBelowWarning is generally better.\n");
fprintf("- Smaller roll/yaw detours are better after safety is satisfied.\n");
fprintf("- Choose the lowest weight that gives a smooth safe detour.\n");
fprintf("- NaN FallbackCount means diagnostic capture was disabled.\n");

% =========================================================================
% FIGURE 1: SINGULARITY DISTANCE
% =========================================================================

figure("Name", "ABENICS Warning-Zone Singularity Sweep");
hold on;

for weightIndex = 1:numWeights
    plot( ...
        results(weightIndex).time, ...
        results(weightIndex).singularityDeg, ...
        "LineWidth", 1.4, ...
        "DisplayName", "wSingularity = " + ...
            string(results(weightIndex).wSingularity));
end

yline(rad2deg(baseParams.singularity.warningDistance), ...
    "--", "Warning distance", "LineWidth", 1.2);

yline(rad2deg(baseParams.singularity.dangerDistance), ...
    "--", "Danger distance", "LineWidth", 1.2);

grid on;
xlabel("Time (s)");
ylabel("Nearest pole distance (deg)");
title("Warning-Zone Singularity Avoidance");
legend("Location", "best");

% =========================================================================
% FIGURE 2: PITCH TRACKING AND DETOUR AXES
% =========================================================================

figure("Name", "ABENICS Warning-Zone Detours");

subplot(3, 1, 1);
hold on;

for weightIndex = 1:numWeights
    plot( ...
        results(weightIndex).time, ...
        results(weightIndex).qPredDeg(2, :), ...
        "LineWidth", 1.3, ...
        "DisplayName", "w = " + ...
            string(results(weightIndex).wSingularity));
end

yline(rad2deg(q_target(2)), "--", "Pitch target");
grid on;
ylabel("Pitch (deg)");
title("Pitch Tracking");
legend("Location", "best");

subplot(3, 1, 2);
hold on;

for weightIndex = 1:numWeights
    plot( ...
        results(weightIndex).time, ...
        results(weightIndex).qPredDeg(1, :), ...
        "LineWidth", 1.3);
end

grid on;
ylabel("Roll detour (deg)");
title("Roll Detour");

subplot(3, 1, 3);
hold on;

for weightIndex = 1:numWeights
    plot( ...
        results(weightIndex).time, ...
        results(weightIndex).qPredDeg(3, :), ...
        "LineWidth", 1.3);
end

grid on;
xlabel("Time (s)");
ylabel("Yaw detour (deg)");
title("Yaw Detour");

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
