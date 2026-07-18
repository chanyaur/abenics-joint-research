% =========================================================================
% test_mpc_cem_detour.m
%
% Closed-loop +X proof test for the ABENICS SO(3) CEM nonlinear MPC.
%
% Required route:
%   start  = [0;  15; 0] deg
%   target = [0; -15; 0] deg
%
% The direct pitch path crosses the +X singular pole. The CEM MPC must trial
% continuous future command sequences, reject unsafe trajectories, and select
% its own roll/yaw-assisted detour. No external route path is supplied.
% =========================================================================

clear;
clc;
close all;

run("params_abenics_coordinate.m");
rng(7, 'twister');

% =========================================================================
% TEST CONFIGURATION
% =========================================================================

simulationTime = 2.0;  % validated full +X detour and settling test
q_start  = deg2rad([0;  15; 0]);
q_target = deg2rad([0; -15; 0]);

params.mpc.Np = 20;
params.mpc.Nc = 12;
params.mpc.maxQStep = deg2rad([2; 2; 2]);
params.mpc.wSingularity = 4000;

params.mpc.cemNumberOfKnots = 4;
params.mpc.cemPopulationSize = 64;
params.mpc.cemIterations = 3;
params.mpc.cemProgressEveryCandidates = 16;
params.mpc.cemEliteFraction = 0.15;
params.mpc.cemInitialStd = deg2rad([1.25; 1.25; 1.25]);
params.mpc.cemMinimumStd = deg2rad([0.10; 0.10; 0.10]);
params.mpc.cemMaximumStd = deg2rad([2.00; 2.00; 2.00]);
params.mpc.cemSigmaFloor = deg2rad([0.35; 0.35; 0.35]);
params.mpc.cemSettlingErrorThreshold = deg2rad(3.0);
params.mpc.cemExplorationFraction = 0.30;
params.mpc.cemExplorationScale = 1.50;
params.mpc.cemExplorationStd = deg2rad([1.25; 1.25; 1.25]);
params.mpc.cemStagnationUpdates = 8;
params.mpc.cemStagnationTolerance = deg2rad(0.25);
params.mpc.cemStagnationDisableBelowError = deg2rad(3.0);
params.mpc.cemCovarianceResetScale = 2.50;
params.mpc.cemStagnationMeanDirectBlend = 0.75;
params.mpc.cemTransitionSamples = 3;
params.mpc.transitionSafetySamples = 9;
params.mpc.thetaMin = deg2rad([-360; -360; -360; -360]);
params.mpc.thetaMax = deg2rad([ 360;  360;  360;  360]);
params.mpc.enforceThetaPositionLimits = true;

params.mpc.debug = true;
params.mpc.liveProgress = true;
params.mpc.enableTestDiagnostics = true;

clear abenicsOrientationMPC;
global ABENICS_CEM_LAST_DIAGNOSTICS
ABENICS_CEM_LAST_DIAGNOSTICS = [];

% =========================================================================
% INITIAL STATE
% =========================================================================

numberOfUpdates = round(simulationTime / params.Ts);
numberOfSamples = numberOfUpdates + 1;
time = (0:numberOfUpdates) * params.Ts;

q_des_prev = q_start;
theta_actual = abenicsIK(q_start, params);
theta_actual = theta_actual(:);
omega_actual = zeros(4, 1);

q_command_log = NaN(3, numberOfSamples);
q_actual_log = NaN(3, numberOfSamples);
theta_actual_log = NaN(4, numberOfSamples);
omega_log = NaN(4, numberOfSamples);
pole_distance_log = NaN(6, numberOfSamples);
transition_minimum_log = NaN(1, numberOfUpdates);
controller_time_log = NaN(1, numberOfUpdates);
accepted_log = false(1, numberOfUpdates);
fallback_log = false(1, numberOfUpdates);
recovery_log = false(1, numberOfUpdates);
stagnation_reset_log = false(1, numberOfUpdates);
safe_candidate_log = zeros(1, numberOfUpdates);
evaluated_candidate_log = zeros(1, numberOfUpdates);
best_cost_log = NaN(1, numberOfUpdates);
best_predicted_pole_log = NaN(1, numberOfUpdates);

q_initial = abenicsFK(theta_actual, params);
q_initial = q_initial(:);
[~, initialPoleDistances] = testPoleDistancesFromQ(q_initial, params);

q_command_log(:, 1) = q_des_prev;
q_actual_log(:, 1) = q_initial;
theta_actual_log(:, 1) = theta_actual;
omega_log(:, 1) = omega_actual;
pole_distance_log(:, 1) = initialPoleDistances;

% =========================================================================
% CLOSED-LOOP SIMULATION
% =========================================================================

for updateIndex = 1:numberOfUpdates
    fprintf(['\n[TEST] update %d/%d | t=%.3f s | ', ...
             'q=[%.3f %.3f %.3f] deg\n'], ...
        updateIndex, numberOfUpdates, time(updateIndex), ...
        rad2deg(q_actual_log(1, updateIndex)), ...
        rad2deg(q_actual_log(2, updateIndex)), ...
        rad2deg(q_actual_log(3, updateIndex)));

    controllerTimer = tic;
    q_command = abenicsOrientationMPC( ...
        q_target, theta_actual, q_des_prev, params);
    controller_time_log(updateIndex) = toc(controllerTimer);

    diagnostics = ABENICS_CEM_LAST_DIAGNOSTICS;
    if isstruct(diagnostics) && isfield(diagnostics, 'accepted')
        accepted_log(updateIndex) = diagnostics.accepted;
        fallback_log(updateIndex) = diagnostics.fallbackUsed;
        recovery_log(updateIndex) = diagnostics.recoveryUsed;
        if isfield(diagnostics, 'stagnationResetTriggered')
            stagnation_reset_log(updateIndex) = ...
                diagnostics.stagnationResetTriggered;
        end
        safe_candidate_log(updateIndex) = diagnostics.safeCandidates;
        evaluated_candidate_log(updateIndex) = ...
            diagnostics.candidatesEvaluated;
        best_cost_log(updateIndex) = diagnostics.bestCost;
        best_predicted_pole_log(updateIndex) = ...
            diagnostics.bestMinimumPoleDistance;
    end

    theta_command = abenicsIK(q_command, params);
    theta_command = theta_command(:);
    theta_command = unwrapThetaNearestLocal( ...
        theta_command, theta_actual, params);

    theta_start = theta_actual;
    [theta_actual, omega_actual, ~] = testPlantStep( ...
        theta_actual, omega_actual, theta_command, params);

    transition_minimum_log(updateIndex) = ...
        testTransitionMinimumPoleDistance( ...
            theta_start, theta_actual, ...
            params.mpc.transitionSafetySamples, params);

    q_actual = abenicsFK(theta_actual, params);
    q_actual = q_actual(:);
    [~, actualPoleDistances] = ...
        testPoleDistancesFromQ(q_actual, params);

    q_des_prev = q_command;

    sampleIndex = updateIndex + 1;
    q_command_log(:, sampleIndex) = q_command;
    q_actual_log(:, sampleIndex) = q_actual;
    theta_actual_log(:, sampleIndex) = theta_actual;
    omega_log(:, sampleIndex) = omega_actual;
    pole_distance_log(:, sampleIndex) = actualPoleDistances;

    fprintf([ ...
        '[TEST] completed in %.3f s | q_cmd=[%.3f %.3f %.3f] deg | ', ...
        'q_next=[%.3f %.3f %.3f] deg | transitionMin=%.3f deg\n'], ...
        controller_time_log(updateIndex), ...
        rad2deg(q_command(1)), rad2deg(q_command(2)), ...
        rad2deg(q_command(3)), rad2deg(q_actual(1)), ...
        rad2deg(q_actual(2)), rad2deg(q_actual(3)), ...
        rad2deg(transition_minimum_log(updateIndex)));
end

% =========================================================================
% SUMMARY
% =========================================================================

pitch = q_actual_log(2, :);
crossingIndex = find(pitch <= 0, 1, 'first');
pitchCrossedZero = ~isempty(crossingIndex);
if pitchCrossedZero
    pitchCrossingTime = time(crossingIndex);
else
    pitchCrossingTime = NaN;
end

minimumEndpointPoleDistance = min(pole_distance_log(:));
minimumTransitionPoleDistance = min(transition_minimum_log);
minimumSixPoleDistance = min( ...
    minimumEndpointPoleDistance, minimumTransitionPoleDistance);

finalOrientation = q_actual_log(:, end);
finalAbsoluteError = abs(wrappedDifference(finalOrientation, q_target));
maximumRollDetour = max(abs(q_actual_log(1, :)));
maximumYawDetour = max(abs(q_actual_log(3, :)));
nonfiniteCount = sum(~isfinite(q_actual_log(:))) + ...
    sum(~isfinite(theta_actual_log(:))) + ...
    sum(~isfinite(omega_log(:)));

hardSafetyPass = minimumSixPoleDistance >= ...
    params.singularity.dangerDistance - ...
    params.mpc.constraintTolerance;
basicPass = pitchCrossedZero && hardSafetyPass && nonfiniteCount == 0;

fprintf('\n============================================================\n');
fprintf('ABENICS SO(3) CEM NONLINEAR MPC DETOUR TEST\n');
fprintf('============================================================\n');
fprintf('Start:                    [0,  15, 0] deg\n');
fprintf('Target:                   [0, -15, 0] deg\n');
fprintf('Prediction/control:       Np=%d, Nc=%d\n', ...
    params.mpc.Np, params.mpc.Nc);
fprintf('CEM population/iterations %d x %d\n', ...
    params.mpc.cemPopulationSize, params.mpc.cemIterations);
fprintf('Continuous control knots: %d\n', ...
    params.mpc.cemNumberOfKnots);
fprintf('Pitch crossed zero:       %d\n', pitchCrossedZero);
fprintf('Pitch crossing time:      %.6f s\n', pitchCrossingTime);
fprintf('Maximum roll detour:      %.6f deg\n', ...
    rad2deg(maximumRollDetour));
fprintf('Maximum yaw detour:       %.6f deg\n', ...
    rad2deg(maximumYawDetour));
fprintf('Final orientation:        [%.6f, %.6f, %.6f] deg\n', ...
    rad2deg(finalOrientation(1)), rad2deg(finalOrientation(2)), ...
    rad2deg(finalOrientation(3)));
fprintf('Final absolute error:     [%.6f, %.6f, %.6f] deg\n', ...
    rad2deg(finalAbsoluteError(1)), rad2deg(finalAbsoluteError(2)), ...
    rad2deg(finalAbsoluteError(3)));
fprintf('Minimum six-pole distance %.6f deg\n', ...
    rad2deg(minimumSixPoleDistance));
fprintf('Hard safety pass:         %d\n', hardSafetyPass);
fprintf('Accepted CEM updates:     %d / %d\n', ...
    sum(accepted_log), numberOfUpdates);
fprintf('Fallback count:           %d\n', sum(fallback_log));
fprintf('Recovery count:           %d\n', sum(recovery_log));
fprintf('Stagnation reset count:   %d\n', sum(stagnation_reset_log));
fprintf('Nonfinite value count:    %d\n', nonfiniteCount);
fprintf('Average controller call:  %.6f s\n', ...
    mean(controller_time_log));
fprintf('Worst controller call:    %.6f s\n', ...
    max(controller_time_log));
fprintf('Average safe candidates:  %.2f / %.2f evaluated\n', ...
    mean(safe_candidate_log), mean(evaluated_candidate_log));
fprintf('OVERALL BASIC PASS:       %d\n', basicPass);

% Leave useful logs in the workspace for inspection.
results.time = time;
results.q_command = q_command_log;
results.q_actual = q_actual_log;
results.theta_actual = theta_actual_log;
results.omega = omega_log;
results.pole_distances = pole_distance_log;
results.transition_minimum = transition_minimum_log;
results.controller_time = controller_time_log;
results.accepted = accepted_log;
results.fallback = fallback_log;
results.recovery = recovery_log;
results.safe_candidates = safe_candidate_log;
results.evaluated_candidates = evaluated_candidate_log;
results.best_cost = best_cost_log;
results.best_predicted_pole = best_predicted_pole_log;
results.basicPass = basicPass;

% =========================================================================
% Local helpers
% =========================================================================
function thetaContinuous = unwrapThetaNearestLocal( ...
    thetaRaw, thetaReference, params)

    thetaRaw = thetaRaw(:);
    thetaReference = thetaReference(:);
    thetaContinuous = thetaRaw;

    if ~isfield(params.mpc, 'thetaUnwrapEnabled') || ...
            ~params.mpc.thetaUnwrapEnabled
        return;
    end

    if isfield(params.mpc, 'thetaPeriodic')
        periodicMask = logical(expandParameter(params.mpc.thetaPeriodic, 4));
    else
        periodicMask = true(4, 1);
    end

    for motorIndex = 1:4
        if periodicMask(motorIndex)
            thetaContinuous(motorIndex) = thetaRaw(motorIndex) + ...
                2*pi * round((thetaReference(motorIndex) - ...
                thetaRaw(motorIndex)) / (2*pi));
        end
    end
end

function [thetaNext, omegaNext, alpha] = testPlantStep( ...
    theta, omega, thetaCommand, params)

    KpPlant = expandParameter(params.plant.KpPlant, 4);
    KdPlant = expandParameter(params.plant.KdPlant, 4);
    thetaError = thetaCommand - theta;
    alpha = KpPlant .* thetaError - KdPlant .* omega;
    omegaNext = omega + params.Ts * alpha;
    thetaNext = theta + params.Ts * omegaNext;
end

function minimumDistance = testTransitionMinimumPoleDistance( ...
    thetaStart, thetaEnd, samples, params)

    thetaDifference = thetaEnd - thetaStart;
    minimumDistance = inf;
    samples = max(2, round(samples));

    for sampleIndex = 1:samples
        lambda = sampleIndex / samples;
        thetaSample = thetaStart + lambda * thetaDifference;
        qSample = abenicsFK(thetaSample, params);
        [sampleMinimum, ~] = ...
            testPoleDistancesFromQ(qSample(:), params);
        minimumDistance = min(minimumDistance, sampleMinimum);
    end
end


function [minimumDistance, distances] = testPoleDistancesFromQ(q, params)
    q = q(:);
    roll = q(1);
    pitch = q(2);
    yaw = q(3);

    cr = cos(roll); sr = sin(roll);
    cp = cos(pitch); sp = sin(pitch);
    cy = cos(yaw); sy = sin(yaw);

    Rx = [1, 0, 0; 0, cr, -sr; 0, sr, cr];
    Ry = [cp, 0, sp; 0, 1, 0; -sp, 0, cp];
    Rz = [cy, -sy, 0; sy, cy, 0; 0, 0, 1];
    rotation = Rx * Ry * Rz;

    bodyAxis = params.singularity.trackedBodyAxis(:);
    trackedAxis = rotation * (bodyAxis / norm(bodyAxis));
    trackedAxis = trackedAxis / norm(trackedAxis);
    poleAxes = params.singularity.poleAxes;

    distances = zeros(size(poleAxes, 2), 1);
    for poleIndex = 1:size(poleAxes, 2)
        pole = poleAxes(:, poleIndex);
        pole = pole / norm(pole);
        distances(poleIndex) = acos(min(1, max(-1, ...
            dot(pole, trackedAxis))));
    end
    minimumDistance = min(distances);
end

function difference = wrappedDifference(a, b)
    difference = atan2(sin(a - b), cos(a - b));
end

function vector = expandParameter(value, numberOfElements)
    value = value(:);
    if numel(value) == 1
        vector = value * ones(numberOfElements, 1);
    else
        vector = value;
    end
end
