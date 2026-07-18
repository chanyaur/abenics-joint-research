% =========================================================================
% test_mpc_cem_robustness_seed_sweep.m
%
% Stochastic robustness regression for the two previously inconsistent
% six-pole cases: -X and -Z. Runs seeds 1:5 for 6 seconds per case.
% =========================================================================

clear;
clc;
close all;

run("params_abenics_coordinate.m");

%% Test settings
seeds = 1:5;
simulationTime = 6.0;
finalPhysicalErrorLimit = deg2rad(1.0);

params.singularity.dangerDistance = deg2rad(2.0);
params.singularity.warningDistance = deg2rad(10.0);
params.mpc.wSingularity = 4000;
params.mpc.Np = 20;
params.mpc.Nc = 12;
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

params.mpc.qMin = deg2rad([-720; -720; -720]);
params.mpc.qMax = deg2rad([ 720;  720;  720]);
params.mpc.thetaMin = deg2rad([-360; -360; -360; -360]);
params.mpc.thetaMax = deg2rad([ 360;  360;  360;  360]);
params.mpc.enforceThetaPositionLimits = true;

params.mpc.debug = false;
params.mpc.liveProgress = false;
params.mpc.enableTestDiagnostics = true;

caseNames = ["-X"; "-Z"];
poleIndices = [2; 6];
qStartDeg = [0, -15; 0, 89; 165, 0];
qTargetDeg = [0, 15; 0, 89; -165, 0];

numberOfSeeds = numel(seeds);
numberOfCases = numel(caseNames);
numberOfUpdates = round(simulationTime / params.Ts);

passed = false(numberOfSeeds, numberOfCases);
crossed = false(numberOfSeeds, numberOfCases);
crossingTime = NaN(numberOfSeeds, numberOfCases);
finalErrorDeg = NaN(numberOfSeeds, numberOfCases);
minimumPoleDeg = NaN(numberOfSeeds, numberOfCases);
acceptedUpdates = zeros(numberOfSeeds, numberOfCases);
fallbacks = zeros(numberOfSeeds, numberOfCases);
recoveries = zeros(numberOfSeeds, numberOfCases);
stagnationResets = zeros(numberOfSeeds, numberOfCases);
dynamicsRejections = zeros(numberOfSeeds, numberOfCases);
poleRejections = zeros(numberOfSeeds, numberOfCases);

runtimeByRun = NaN(numberOfSeeds, numberOfCases);

global ABENICS_CEM_LAST_DIAGNOSTICS

for seedIndex = 1:numberOfSeeds
    for caseIndex = 1:numberOfCases
        seed = seeds(seedIndex);
        qStart = deg2rad(qStartDeg(:, caseIndex));
        qTarget = deg2rad(qTargetDeg(:, caseIndex));
        intendedPoleIndex = poleIndices(caseIndex);

        fprintf('\n============================================================\n');
        fprintf('ROBUSTNESS SEED %d | CASE %s\n', seed, caseNames(caseIndex));
        fprintf('============================================================\n');

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

        minimumPole = inf;
        sideLog = NaN(1, numberOfUpdates + 1);
        acceptedLog = false(1, numberOfUpdates);
        fallbackLog = false(1, numberOfUpdates);
        recoveryLog = false(1, numberOfUpdates);
        resetLog = false(1, numberOfUpdates);
        rejectionLog = zeros(6, numberOfUpdates);

        qActual = abenicsFK(thetaActual, params);
        actualRotation = qToRotmXYZ(qActual(:));
        [~, poleDistances, trackedAxis] = ...
            poleDistancesFromRotm(actualRotation, params);
        minimumPole = min(minimumPole, min(poleDistances));
        sideLog(1) = dot(targetSideAxis, trackedAxis);

        runTimer = tic;
        for updateIndex = 1:numberOfUpdates
            qCommand = abenicsOrientationMPC( ...
                qTarget, thetaActual, qDesPrevious, params);

            diagnostics = ABENICS_CEM_LAST_DIAGNOSTICS;
            if isstruct(diagnostics) && isfield(diagnostics, 'accepted')
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
            minimumPole = min(minimumPole, transitionMinimum);

            qActual = abenicsFK(thetaActual, params);
            actualRotation = qToRotmXYZ(qActual(:));
            [~, poleDistances, trackedAxis] = ...
                poleDistancesFromRotm(actualRotation, params);
            minimumPole = min(minimumPole, min(poleDistances));
            sideLog(updateIndex + 1) = dot(targetSideAxis, trackedAxis);
            qDesPrevious = qCommand;
        end
        runtimeByRun(seedIndex, caseIndex) = toc(runTimer);

        crossingIndex = find(sideLog >= 0, 1, 'first');
        crossed(seedIndex, caseIndex) = ~isempty(crossingIndex);
        if crossed(seedIndex, caseIndex)
            crossingTime(seedIndex, caseIndex) = ...
                (crossingIndex - 1) * params.Ts;
        end

        finalError = rotationDistance(actualRotation, targetRotation);
        finalErrorDeg(seedIndex, caseIndex) = rad2deg(finalError);
        minimumPoleDeg(seedIndex, caseIndex) = rad2deg(minimumPole);
        acceptedUpdates(seedIndex, caseIndex) = sum(acceptedLog);
        fallbacks(seedIndex, caseIndex) = sum(fallbackLog);
        recoveries(seedIndex, caseIndex) = sum(recoveryLog);
        stagnationResets(seedIndex, caseIndex) = sum(resetLog);
        dynamicsRejections(seedIndex, caseIndex) = sum(rejectionLog(5, :));
        poleRejections(seedIndex, caseIndex) = sum(rejectionLog(6, :));

        passed(seedIndex, caseIndex) = ...
            crossed(seedIndex, caseIndex) && ...
            finalError <= finalPhysicalErrorLimit && ...
            minimumPole >= params.singularity.dangerDistance - ...
                params.mpc.constraintTolerance && ...
            all(acceptedLog) && ...
            ~any(fallbackLog) && ~any(recoveryLog);

        fprintf('Crossed target side:      %d\n', crossed(seedIndex, caseIndex));
        fprintf('Crossing time:            %.3f s\n', crossingTime(seedIndex, caseIndex));
        fprintf('Final physical error:     %.3f deg\n', finalErrorDeg(seedIndex, caseIndex));
        fprintf('Minimum pole distance:    %.3f deg\n', minimumPoleDeg(seedIndex, caseIndex));
        fprintf('Accepted updates:         %d/%d\n', ...
            acceptedUpdates(seedIndex, caseIndex), numberOfUpdates);
        fprintf('Fallback/recovery:        %d/%d\n', ...
            fallbacks(seedIndex, caseIndex), recoveries(seedIndex, caseIndex));
        fprintf('Stagnation resets:        %d\n', ...
            stagnationResets(seedIndex, caseIndex));
        fprintf('Dynamics/pole rejections: %d/%d\n', ...
            dynamicsRejections(seedIndex, caseIndex), ...
            poleRejections(seedIndex, caseIndex));
        fprintf('CASE PASS:                %d\n', passed(seedIndex, caseIndex));
    end
end

%% Summary
seedColumn = repelem(seeds(:), numberOfCases, 1);
caseColumn = repmat(caseNames, numberOfSeeds, 1);
resultTable = table( ...
    seedColumn, caseColumn, ...
    reshape(passed.', [], 1), ...
    reshape(crossed.', [], 1), ...
    reshape(crossingTime.', [], 1), ...
    reshape(finalErrorDeg.', [], 1), ...
    reshape(minimumPoleDeg.', [], 1), ...
    reshape(acceptedUpdates.', [], 1), ...
    reshape(fallbacks.', [], 1), ...
    reshape(recoveries.', [], 1), ...
    reshape(stagnationResets.', [], 1), ...
    reshape(dynamicsRejections.', [], 1), ...
    reshape(poleRejections.', [], 1), ...
    reshape(runtimeByRun.', [], 1), ...
    'VariableNames', { ...
        'Seed', 'Pole', 'Pass', 'Crossed', 'CrossingTime_s', ...
        'FinalPhysicalError_deg', 'MinimumPoleDistance_deg', ...
        'AcceptedUpdates', 'Fallbacks', 'Recoveries', ...
        'StagnationResets', 'DynamicsRejections', 'PoleRejections', ...
        'Runtime_s'});

fprintf('\n============================================================\n');
fprintf('ROBUSTNESS SEED-SWEEP SUMMARY\n');
fprintf('============================================================\n');
disp(resultTable);
fprintf('Successful runs: %d / %d\n', sum(passed(:)), numel(passed));
fprintf('-X successes:    %d / %d\n', sum(passed(:, 1)), numberOfSeeds);
fprintf('-Z successes:    %d / %d\n', sum(passed(:, 2)), numberOfSeeds);
fprintf('OVERALL 10/10 PASS: %d\n', all(passed(:)));

assignin('base', 'robustnessSeedSweepResults', resultTable);
assignin('base', 'robustnessSeedSweepPassMatrix', passed);

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