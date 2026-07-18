% =========================================================================
% test_mpc_cem_horizon_comparison.m
%
% Focused diagnosis of prediction-horizon myopia in the working v2.2
% SO(3) CEM controller.
%
% Runs exactly the same difficult case twice:
%   Case:       -X pole
%   Seed:       1
%   Nc:         12
%   Horizon 1:  Np = 20  -> 0.40 s prediction
%   Horizon 2:  Np = 60  -> 1.20 s prediction
%
% Every setting other than Np is held constant.
% =========================================================================

clear;
clc;
close all;

run("params_abenics_coordinate.m");

%% Fixed experiment settings
seed = 4;
simulationTime = 6.0;
NpValues = [32, 33];
Nc = 12;
finalPhysicalErrorLimit = deg2rad(1.0);

% -X crossing case
qStart = deg2rad([0; 0; 165]);
qTarget = deg2rad([0; 0; -165]);
intendedPoleIndex = 2;  % -X in [ +X, -X, +Y, -Y, +Z, -Z ]

%% Preserve the known v2.2 baseline settings
params.singularity.dangerDistance = deg2rad(2.0);
params.singularity.warningDistance = deg2rad(10.0);
params.mpc.wSingularity = 4000;
params.mpc.Nc = Nc;
params.mpc.maxQStep = deg2rad([2; 2; 2]);
params.mpc.cemNumberOfKnots = 4;
params.mpc.cemPopulationSize = 64;
params.mpc.cemIterations = 3;
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

% Global orientation test range and confirmed full-revolution MP range
params.mpc.qMin = deg2rad([-720; -720; -720]);
params.mpc.qMax = deg2rad([ 720;  720;  720]);
params.mpc.thetaMin = deg2rad([-360; -360; -360; -360]);
params.mpc.thetaMax = deg2rad([ 360;  360;  360;  360]);
params.mpc.enforceThetaPositionLimits = true;
params.mpc.thetaUnwrapEnabled = true;
params.mpc.thetaPeriodic = true(4, 1);

params.mpc.debug = false;
params.mpc.liveProgress = false;
params.mpc.enableTestDiagnostics = true;

numberOfRuns = numel(NpValues);
numberOfUpdates = round(simulationTime / params.Ts);
constraintTolerance = 1e-6;
if isfield(params.mpc, 'constraintTolerance')
    constraintTolerance = params.mpc.constraintTolerance;
end

% Summary storage
passed = false(numberOfRuns, 1);
crossed = false(numberOfRuns, 1);
crossingTime = NaN(numberOfRuns, 1);
finalErrorDeg = NaN(numberOfRuns, 1);
minimumPoleDeg = NaN(numberOfRuns, 1);
acceptedUpdates = zeros(numberOfRuns, 1);
fallbacks = zeros(numberOfRuns, 1);
recoveries = zeros(numberOfRuns, 1);
stagnationResets = zeros(numberOfRuns, 1);
dynamicsRejections = zeros(numberOfRuns, 1);
poleRejections = zeros(numberOfRuns, 1);
averageSolveTime = NaN(numberOfRuns, 1);
worstSolveTime = NaN(numberOfRuns, 1);
totalRuntime = NaN(numberOfRuns, 1);

% Time histories for comparison plots
physicalErrorLog = NaN(numberOfRuns, numberOfUpdates + 1);
poleDistanceLog = NaN(numberOfRuns, numberOfUpdates + 1);
sideLogAll = NaN(numberOfRuns, numberOfUpdates + 1);

global ABENICS_CEM_LAST_DIAGNOSTICS

for runIndex = 1:numberOfRuns
    params.mpc.Np = NpValues(runIndex);

    fprintf('\n============================================================\n');
    fprintf('HORIZON TEST | -X | SEED %d | Np=%d | Nc=%d\n', ...
        seed, params.mpc.Np, params.mpc.Nc);
    fprintf('============================================================\n');
    fprintf('Prediction duration: %.2f s\n', params.mpc.Np * params.Ts);
    fprintf('Simulation duration: %.2f s\n', simulationTime);

    % Reset both the random stream and persistent controller state so the
    % two runs differ only in prediction horizon.
    rng(seed, 'twister');
    clear abenicsOrientationMPC;
    ABENICS_CEM_LAST_DIAGNOSTICS = [];

    targetRotation = qToRotmXYZ(qTarget);
    intendedPole = params.singularity.poleAxes(:, intendedPoleIndex);
    intendedPole = intendedPole / norm(intendedPole);
    targetTrackedAxis = trackedAxisFromRotation(targetRotation, params);
    targetSideAxis = targetTrackedAxis - ...
        dot(targetTrackedAxis, intendedPole) * intendedPole;
    targetSideAxis = targetSideAxis / norm(targetSideAxis);

    qDesPrevious = qStart;
    thetaActual = abenicsIK(qStart, params);
    thetaActual = thetaActual(:);
    omegaActual = zeros(4, 1);

    acceptedLog = false(1, numberOfUpdates);
    fallbackLog = false(1, numberOfUpdates);
    recoveryLog = false(1, numberOfUpdates);
    resetLog = false(1, numberOfUpdates);
    rejectionLog = zeros(6, numberOfUpdates);
    solveTimeLog = NaN(1, numberOfUpdates);

    qActual = abenicsFK(thetaActual, params);
    actualRotation = qToRotmXYZ(qActual(:));
    [minimumPole, poleDistances, trackedAxis] = ...
        poleDistancesFromRotm(actualRotation, params);

    physicalErrorLog(runIndex, 1) = ...
        rotationDistance(actualRotation, targetRotation);
    poleDistanceLog(runIndex, 1) = minimumPole;
    sideLogAll(runIndex, 1) = dot(targetSideAxis, trackedAxis);

    versionChecked = false;
    runTimer = tic;

    for updateIndex = 1:numberOfUpdates
        qCommand = abenicsOrientationMPC( ...
            qTarget, thetaActual, qDesPrevious, params);

        diagnostics = ABENICS_CEM_LAST_DIAGNOSTICS;
        if isstruct(diagnostics) && isfield(diagnostics, 'accepted')
            if ~versionChecked && isfield(diagnostics, 'version')
                if abs(diagnostics.version - 2.2) > 1e-9
                    error(['This test requires the working v2.2 controller. ', ...
                        'Loaded diagnostics.version = %.3f.'], ...
                        diagnostics.version);
                end
                versionChecked = true;
            end

            acceptedLog(updateIndex) = diagnostics.accepted;
            fallbackLog(updateIndex) = diagnostics.fallbackUsed;
            recoveryLog(updateIndex) = diagnostics.recoveryUsed;

            if isfield(diagnostics, 'stagnationResetTriggered')
                resetLog(updateIndex) = ...
                    diagnostics.stagnationResetTriggered;
            end
            if isfield(diagnostics, 'rejectionCounts') && ...
                    numel(diagnostics.rejectionCounts) == 6
                rejectionLog(:, updateIndex) = ...
                    diagnostics.rejectionCounts(:);
            end
            if isfield(diagnostics, 'solveTime')
                solveTimeLog(updateIndex) = diagnostics.solveTime;
            end
        end

        thetaCommand = abenicsIK(qCommand, params);
        thetaCommand = unwrapThetaNearestLocal( ...
            thetaCommand(:), thetaActual, params);

        thetaStart = thetaActual;
        [thetaActual, omegaActual] = plantStep( ...
            thetaActual, omegaActual, thetaCommand, params);

        transitionMinimum = transitionPoleMinimum( ...
            thetaStart, thetaActual, ...
            params.mpc.transitionSafetySamples, params);

        qActual = abenicsFK(thetaActual, params);
        actualRotation = qToRotmXYZ(qActual(:));
        [minimumAtUpdate, poleDistances, trackedAxis] = ...
            poleDistancesFromRotm(actualRotation, params);

        physicalErrorLog(runIndex, updateIndex + 1) = ...
            rotationDistance(actualRotation, targetRotation);
        poleDistanceLog(runIndex, updateIndex + 1) = ...
            min(transitionMinimum, minimumAtUpdate);
        sideLogAll(runIndex, updateIndex + 1) = ...
            dot(targetSideAxis, trackedAxis);

        qDesPrevious = qCommand;
    end

    totalRuntime(runIndex) = toc(runTimer);

    crossingIndex = find(sideLogAll(runIndex, :) >= 0, 1, 'first');
    crossed(runIndex) = ~isempty(crossingIndex);
    if crossed(runIndex)
        crossingTime(runIndex) = (crossingIndex - 1) * params.Ts;
    end

    finalPhysicalError = physicalErrorLog(runIndex, end);
    minimumPoleDistance = min(poleDistanceLog(runIndex, :));

    finalErrorDeg(runIndex) = rad2deg(finalPhysicalError);
    minimumPoleDeg(runIndex) = rad2deg(minimumPoleDistance);
    acceptedUpdates(runIndex) = sum(acceptedLog);
    fallbacks(runIndex) = sum(fallbackLog);
    recoveries(runIndex) = sum(recoveryLog);
    stagnationResets(runIndex) = sum(resetLog);
    dynamicsRejections(runIndex) = sum(rejectionLog(5, :));
    poleRejections(runIndex) = sum(rejectionLog(6, :));
    averageSolveTime(runIndex) = mean(solveTimeLog, 'omitnan');
    worstSolveTime(runIndex) = max(solveTimeLog, [], 'omitnan');

    passed(runIndex) = ...
        crossed(runIndex) && ...
        finalPhysicalError <= finalPhysicalErrorLimit && ...
        minimumPoleDistance >= ...
            params.singularity.dangerDistance - constraintTolerance && ...
        all(acceptedLog) && ...
        ~any(fallbackLog) && ~any(recoveryLog);

    fprintf('Crossed target side:      %d\n', crossed(runIndex));
    fprintf('Crossing time:            %.3f s\n', crossingTime(runIndex));
    fprintf('Final physical error:     %.3f deg\n', finalErrorDeg(runIndex));
    fprintf('Minimum pole distance:    %.3f deg\n', minimumPoleDeg(runIndex));
    fprintf('Accepted updates:         %d/%d\n', ...
        acceptedUpdates(runIndex), numberOfUpdates);
    fprintf('Fallback/recovery:        %d/%d\n', ...
        fallbacks(runIndex), recoveries(runIndex));
    fprintf('Stagnation resets:        %d\n', stagnationResets(runIndex));
    fprintf('Dynamics/pole rejections: %d/%d\n', ...
        dynamicsRejections(runIndex), poleRejections(runIndex));
    fprintf('Average controller call:  %.4f s\n', ...
        averageSolveTime(runIndex));
    fprintf('Worst controller call:    %.4f s\n', ...
        worstSolveTime(runIndex));
    fprintf('CASE PASS:                %d\n', passed(runIndex));
end

%% Comparison summary
predictionDuration = NpValues(:) * params.Ts;
resultTable = table( ...
    NpValues(:), predictionDuration, repmat(Nc, numberOfRuns, 1), ...
    crossed, crossingTime, finalErrorDeg, minimumPoleDeg, ...
    acceptedUpdates, fallbacks, recoveries, stagnationResets, ...
    dynamicsRejections, poleRejections, averageSolveTime, ...
    worstSolveTime, totalRuntime, passed, ...
    'VariableNames', { ...
        'Np', 'PredictionDuration_s', 'Nc', ...
        'Crossed', 'CrossingTime_s', 'FinalPhysicalError_deg', ...
        'MinimumPoleDistance_deg', 'AcceptedUpdates', 'Fallbacks', ...
        'Recoveries', 'StagnationResets', 'DynamicsRejections', ...
        'PoleRejections', 'AverageControllerCall_s', ...
        'WorstControllerCall_s', 'TotalRuntime_s', 'Pass'});

fprintf('\n============================================================\n');
fprintf('PREDICTION-HORIZON COMPARISON SUMMARY\n');
fprintf('============================================================\n');
disp(resultTable);

if ~crossed(1) && crossed(2)
    fprintf(['DIAGNOSIS: Np=60 crossed while Np=20 did not. ', ...
        'This strongly supports prediction-horizon myopia.\n']);
elseif crossed(1) && crossed(2)
    fprintf(['DIAGNOSIS: Both horizons crossed. Compare final error, ', ...
        'clearance, and consistency before concluding.\n']);
elseif ~crossed(1) && ~crossed(2)
    fprintf(['DIAGNOSIS: Neither horizon crossed. Horizon length alone ', ...
        'did not fix this seed; inspect feasible candidates and cost.\n']);
else
    fprintf(['DIAGNOSIS: Np=20 crossed but Np=60 did not. The longer ', ...
        'horizon changed feasibility/cost unexpectedly.\n']);
end

%% Comparison plots
time = (0:numberOfUpdates) * params.Ts;
labels = strings(size(NpValues));

for runIndex = 1:numel(NpValues)
    labels(runIndex) = sprintf( ...
        'Np=%d (%.1f s)', ...
        NpValues(runIndex), ...
        NpValues(runIndex) * params.Ts);
end

figure('Name', 'ABENICS Horizon Comparison: Physical Error');
hold on;
for runIndex = 1:numberOfRuns
    plot(time, rad2deg(physicalErrorLog(runIndex, :)), ...
        'LineWidth', 1.4, 'DisplayName', labels(runIndex));
end
yline(rad2deg(finalPhysicalErrorLimit), '--', ...
    '1 deg final-error limit');
xlabel('Time (s)');
ylabel('Physical orientation error (deg)');
title('-X pole test: target error');
grid on;
legend('Location', 'best');

figure('Name', 'ABENICS Horizon Comparison: Pole Clearance');
hold on;
for runIndex = 1:numberOfRuns
    plot(time, rad2deg(poleDistanceLog(runIndex, :)), ...
        'LineWidth', 1.4, 'DisplayName', labels(runIndex));
end
yline(rad2deg(params.singularity.dangerDistance), '--', ...
    '2 deg hard limit');
xlabel('Time (s)');
ylabel('Minimum six-pole distance (deg)');
title('-X pole test: singularity clearance');
grid on;
legend('Location', 'best');

assignin('base', 'horizonComparisonResults', resultTable);
assignin('base', 'horizonComparisonPhysicalErrorLog_deg', ...
    rad2deg(physicalErrorLog));
assignin('base', 'horizonComparisonPoleDistanceLog_deg', ...
    rad2deg(poleDistanceLog));

% =========================================================================
% Local helpers
% =========================================================================
function thetaContinuous = unwrapThetaNearestLocal( ...
    thetaRaw, thetaReference, params)

    thetaRaw = thetaRaw(:);
    thetaReference = thetaReference(:);
    thetaContinuous = thetaRaw;

    if isfield(params.mpc, 'thetaUnwrapEnabled') && ...
            ~params.mpc.thetaUnwrapEnabled
        return;
    end

    periodicMask = true(4, 1);
    if isfield(params.mpc, 'thetaPeriodic')
        periodicMask = logical(expandParameter( ...
            params.mpc.thetaPeriodic, 4));
    end

    for motorIndex = 1:4
        if periodicMask(motorIndex)
            thetaContinuous(motorIndex) = thetaRaw(motorIndex) + ...
                2*pi * round((thetaReference(motorIndex) - ...
                thetaRaw(motorIndex)) / (2*pi));
        end
    end
end

function [thetaNext, omegaNext] = plantStep( ...
    theta, omega, thetaCommand, params)

    KpPlant = expandParameter(params.plant.KpPlant, 4);
    KdPlant = expandParameter(params.plant.KdPlant, 4);
    alpha = KpPlant .* (thetaCommand - theta) - KdPlant .* omega;
    omegaNext = omega + params.Ts * alpha;
    thetaNext = theta + params.Ts * omegaNext;
end

function minimumAll = transitionPoleMinimum( ...
    thetaStart, thetaEnd, samples, params)

    minimumAll = inf;
    thetaDifference = thetaEnd - thetaStart;
    for sampleIndex = 1:samples
        lambda = sampleIndex / samples;
        thetaSample = thetaStart + lambda * thetaDifference;
        qSample = abenicsFK(thetaSample, params);
        [~, distances] = poleDistancesFromRotm( ...
            qToRotmXYZ(qSample(:)), params);
        minimumAll = min(minimumAll, min(distances));
    end
end

function [minimumDistance, distances, trackedAxis] = ...
    poleDistancesFromRotm(rotation, params)

    trackedAxis = trackedAxisFromRotation(rotation, params);
    poleAxes = params.singularity.poleAxes;
    distances = zeros(size(poleAxes, 2), 1);
    for poleIndex = 1:size(poleAxes, 2)
        pole = poleAxes(:, poleIndex);
        pole = pole / norm(pole);
        cosine = min(1, max(-1, dot(pole, trackedAxis)));
        distances(poleIndex) = acos(cosine);
    end
    minimumDistance = min(distances);
end

function trackedAxis = trackedAxisFromRotation(rotation, params)
    bodyAxis = params.singularity.trackedBodyAxis(:);
    bodyAxis = bodyAxis / norm(bodyAxis);
    trackedAxis = rotation * bodyAxis;
    trackedAxis = trackedAxis / norm(trackedAxis);
end

function distance = rotationDistance(rotationOne, rotationTwo)
    distance = norm(rotmToRotvec(rotationOne.' * rotationTwo));
end

function rotation = qToRotmXYZ(q)
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
end

function rotationVector = rotmToRotvec(rotation)
    cosineAngle = min(1, max(-1, (trace(rotation) - 1) / 2));
    angle = acos(cosineAngle);

    if angle < 1e-8
        rotationVector = 0.5 * vee3(rotation - rotation.');
        return;
    end

    if pi - angle < 1e-6
        axis = sqrt(max(0, (diag(rotation) + 1) / 2));
        [~, largestIndex] = max(axis);
        if largestIndex == 1
            axis(2) = signNonzero(rotation(1, 2) + rotation(2, 1)) * axis(2);
            axis(3) = signNonzero(rotation(1, 3) + rotation(3, 1)) * axis(3);
        elseif largestIndex == 2
            axis(1) = signNonzero(rotation(1, 2) + rotation(2, 1)) * axis(1);
            axis(3) = signNonzero(rotation(2, 3) + rotation(3, 2)) * axis(3);
        else
            axis(1) = signNonzero(rotation(1, 3) + rotation(3, 1)) * axis(1);
            axis(2) = signNonzero(rotation(2, 3) + rotation(3, 2)) * axis(2);
        end
        if norm(axis) < 1e-9
            axis = [1; 0; 0];
        else
            axis = axis / norm(axis);
        end
        rotationVector = angle * axis;
        return;
    end

    axis = vee3(rotation - rotation.') / (2 * sin(angle));
    rotationVector = angle * axis;
end

function vector = vee3(matrix)
    vector = [matrix(3, 2); matrix(1, 3); matrix(2, 1)];
end

function output = signNonzero(input)
    if input < 0
        output = -1;
    else
        output = 1;
    end
end

function vector = expandParameter(value, numberOfElements)
    value = value(:);
    if numel(value) == 1
        vector = value * ones(numberOfElements, 1);
    elseif numel(value) == numberOfElements
        vector = value;
    else
        error('Expected scalar or %dx1 parameter.', numberOfElements);
    end
end
