% =========================================================================
% test_mpc_cem_all_six_poles.m
%
% All-six-pole validation for SO(3) CEM MPC with continuous motor angles.
%
% Main corrections from the earlier test:
%   1. Direct paths are interpolated on SO(3), not linearly in Euler angles.
%   2. Final target error is the physical rotation angle between rotations.
%   3. A side crossing alone is not a pass; the target must also be reached.
%   4. Rejection categories are accumulated and printed for every case.
%   5. +Y, -Y, +Z, and -Z receive longer settling simulations.
%   6. Direct-path IK reachability and motor-angle branch jumps are reported.
%
% The controller interface remains Euler q = [roll; pitch; yaw], but all
% test geometry and pass/fail logic use rotation matrices.
% =========================================================================

clear;
clc;
close all;

run("params_abenics_coordinate.m");

%% ========================================================================
%  TEST CONFIGURATION
%  ========================================================================

randomSeedBase = 200;

% Set false to run only the fast IK/motor-angle reachability report without
% executing the CEM closed-loop simulations.
runClosedLoop = true;

% Longer cases are used where the previous test remained safe but had not
% finished the maneuver after two seconds.
simulationTimeByCase = [2.5; 6.0; 4.0; 4.0; 4.0; 6.0];

% A case must finish within this physical orientation angle of its target.
finalPhysicalErrorLimit = deg2rad(1.0);

params.singularity.dangerDistance  = deg2rad(2.0);
params.singularity.warningDistance = deg2rad(10.0);
params.mpc.wSingularity = 4000;

params.mpc.Np = 0;
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

% This global-orientation test must allow Euler representations near 180 deg
% and near pitch +/-90 deg. The physical orientation is still constrained by
% IK, motor, plant, and pole checks.
params.mpc.qMin = deg2rad([-720; -720; -720]);
params.mpc.qMax = deg2rad([ 720;  720;  720]);

% Confirmed full +/-360 deg continuous MP-shaft range for this test.
params.mpc.thetaMin = deg2rad([-360; -360; -360; -360]);
params.mpc.thetaMax = deg2rad([ 360;  360;  360;  360]);
params.mpc.enforceThetaPositionLimits = true;

params.mpc.debug = false;
params.mpc.liveProgress = false;
params.mpc.enableTestDiagnostics = true;

%% ========================================================================
%  SIX TEST CASES
%  ========================================================================

caseNames = ["+X"; "-X"; "+Y"; "-Y"; "+Z"; "-Z"];
poleIndices = (1:6).';

% Columns are [roll; pitch; yaw] in degrees.
qStartDeg = [ ...
      0,    0,    0,     0,   -15,   -15;
     15,    0,    0,     0,   -89,    89;
      0,  165,   75,   -75,     0,     0];

qTargetDeg = [ ...
      0,    0,    0,     0,    15,    15;
    -15,    0,    0,     0,   -89,    89;
      0, -165,  105,  -105,     0,     0];

numberOfCases = numel(caseNames);
rejectionNames = [ ...
    "nonfinite/internal";
    "q bounds";
    "IK failure";
    "theta bounds";
    "dynamics";
    "pole distance"];

crossedPoleSide = false(numberOfCases, 1);
targetReached = false(numberOfCases, 1);
hardSafetyPass = false(numberOfCases, 1);
allUpdatesAccepted = false(numberOfCases, 1);
casePass = false(numberOfCases, 1);
caseErrored = false(numberOfCases, 1);

crossingTime = NaN(numberOfCases, 1);
minimumAllPoleDistanceDeg = NaN(numberOfCases, 1);
minimumIntendedPoleDistanceDeg = NaN(numberOfCases, 1);
directPathMinimumDeg = NaN(numberOfCases, 1);
finalPhysicalErrorDeg = NaN(numberOfCases, 1);
finalRollErrorDeg = NaN(numberOfCases, 1);
finalPitchErrorDeg = NaN(numberOfCases, 1);
finalYawErrorDeg = NaN(numberOfCases, 1);
acceptedUpdates = zeros(numberOfCases, 1);
fallbackCount = zeros(numberOfCases, 1);
recoveryCount = zeros(numberOfCases, 1);
stagnationResetCount = zeros(numberOfCases, 1);
averageControllerTime = NaN(numberOfCases, 1);
worstControllerTime = NaN(numberOfCases, 1);
averageSafeCandidates = NaN(numberOfCases, 1);
rejectionTotals = zeros(numberOfCases, 6);
targetThetaBoundViolationDeg = NaN(numberOfCases, 1);
directThetaBoundViolationDeg = NaN(numberOfCases, 1);
directMaximumThetaStepDeg = NaN(numberOfCases, 1);
directRawBranchJumps = NaN(numberOfCases, 1);
errorMessage = strings(numberOfCases, 1);
caseLogs = cell(numberOfCases, 1);

global ABENICS_CEM_LAST_DIAGNOSTICS

for caseIndex = 1:numberOfCases
    simulationTime = simulationTimeByCase(caseIndex);
    numberOfUpdates = round(simulationTime / params.Ts);
    numberOfSamples = numberOfUpdates + 1;

    fprintf('\n============================================================\n');
    fprintf('STARTING SO(3) POLE CASE %s (%d/%d)\n', ...
        caseNames(caseIndex), caseIndex, numberOfCases);
    fprintf('============================================================\n');

    qStart = deg2rad(qStartDeg(:, caseIndex));
    qTarget = deg2rad(qTargetDeg(:, caseIndex));
    intendedPoleIndex = poleIndices(caseIndex);

    startRotation = qToRotmXYZ(qStart);
    targetRotation = qToRotmXYZ(qTarget);
    intendedPole = params.singularity.poleAxes(:, intendedPoleIndex);
    intendedPole = intendedPole / norm(intendedPole);

    startTrackedAxis = trackedAxisFromRotation(startRotation, params);
    targetTrackedAxis = trackedAxisFromRotation(targetRotation, params);

    targetSideAxis = targetTrackedAxis - ...
        dot(targetTrackedAxis, intendedPole) * intendedPole;
    if norm(targetSideAxis) < 1e-10
        error('Target tangent direction is undefined for case %s.', ...
            caseNames(caseIndex));
    end
    targetSideAxis = targetSideAxis / norm(targetSideAxis);

    directPathMinimum = directPathPoleMinimumSO3( ...
        startRotation, targetRotation, intendedPoleIndex, 401, params);
    directPathMinimumDeg(caseIndex) = rad2deg(directPathMinimum);

    fprintf('Simulation time: %.2f s\n', simulationTime);
    fprintf('Start q:         [%.1f %.1f %.1f] deg\n', ...
        qStartDeg(:, caseIndex));
    fprintf('Target q:        [%.1f %.1f %.1f] deg\n', ...
        qTargetDeg(:, caseIndex));
    fprintf('SO(3) direct minimum to %s pole: %.3f deg\n', ...
        caseNames(caseIndex), directPathMinimumDeg(caseIndex));

    try
        rng(randomSeedBase + caseIndex, 'twister');
        clear abenicsOrientationMPC;
        ABENICS_CEM_LAST_DIAGNOSTICS = [];

        qDesPrevious = qStart;
        thetaActual = abenicsIK(qStart, params);
        thetaActual = thetaActual(:);
        omegaActual = zeros(4, 1);

        reachability = analyzeMotorReachabilitySO3( ...
            startRotation, targetRotation, qStart, thetaActual, 181, params);
        targetThetaBoundViolationDeg(caseIndex) = ...
            rad2deg(reachability.targetBoundViolation);
        directThetaBoundViolationDeg(caseIndex) = ...
            rad2deg(reachability.maximumBoundViolation);
        directMaximumThetaStepDeg(caseIndex) = ...
            rad2deg(reachability.maximumContinuousStep);
        directRawBranchJumps(caseIndex) = reachability.rawBranchJumps;

        fprintf('Target nearest-equivalent theta: [%.1f %.1f %.1f %.1f] deg\n', ...
            rad2deg(reachability.targetThetaContinuous));
        fprintf('Target theta-limit violation:    %.3f deg\n', ...
            targetThetaBoundViolationDeg(caseIndex));
        fprintf('SO(3) direct IK max violation:   %.3f deg\n', ...
            directThetaBoundViolationDeg(caseIndex));
        fprintf('SO(3) direct IK max step:        %.3f deg\n', ...
            directMaximumThetaStepDeg(caseIndex));
        fprintf('Raw IK branch jumps (>180 deg):  %d\n', ...
            directRawBranchJumps(caseIndex));

        if ~runClosedLoop
            caseLogs{caseIndex}.motorReachability = reachability;
            continue;
        end

        qActualLog = NaN(3, numberOfSamples);
        thetaActualLog = NaN(4, numberOfSamples);
        omegaLog = NaN(4, numberOfSamples);
        poleDistanceLog = NaN(6, numberOfSamples);
        sideValueLog = NaN(1, numberOfSamples);
        physicalErrorLog = NaN(1, numberOfSamples);
        transitionAllPoleLog = NaN(1, numberOfUpdates);
        transitionIntendedPoleLog = NaN(1, numberOfUpdates);
        controllerTimeLog = NaN(1, numberOfUpdates);
        acceptedLog = false(1, numberOfUpdates);
        fallbackLog = false(1, numberOfUpdates);
        recoveryLog = false(1, numberOfUpdates);
        stagnationResetLog = false(1, numberOfUpdates);
        safeCandidateLog = zeros(1, numberOfUpdates);
        rejectionLog = zeros(6, numberOfUpdates);

        qActual = abenicsFK(thetaActual, params);
        qActual = qActual(:);
        actualRotation = qToRotmXYZ(qActual);
        [~, poleDistances, trackedAxis] = ...
            poleDistancesFromRotm(actualRotation, params);

        qActualLog(:, 1) = qActual;
        thetaActualLog(:, 1) = thetaActual;
        omegaLog(:, 1) = omegaActual;
        poleDistanceLog(:, 1) = poleDistances;
        sideValueLog(1) = dot(targetSideAxis, trackedAxis);
        physicalErrorLog(1) = rotationDistance(actualRotation, targetRotation);

        fprintf('Initial physical target error: %.3f deg\n', ...
            rad2deg(physicalErrorLog(1)));
        fprintf('Initial intended-pole distance: %.3f deg\n', ...
            rad2deg(poleDistances(intendedPoleIndex)));

        for updateIndex = 1:numberOfUpdates
            controllerTimer = tic;
            qCommand = abenicsOrientationMPC( ...
                qTarget, thetaActual, qDesPrevious, params);
            controllerTimeLog(updateIndex) = toc(controllerTimer);

            diagnostics = ABENICS_CEM_LAST_DIAGNOSTICS;
            if isstruct(diagnostics) && isfield(diagnostics, 'accepted')
                acceptedLog(updateIndex) = diagnostics.accepted;
                fallbackLog(updateIndex) = diagnostics.fallbackUsed;
                recoveryLog(updateIndex) = diagnostics.recoveryUsed;
                if isfield(diagnostics, 'stagnationResetTriggered')
                    stagnationResetLog(updateIndex) = ...
                        diagnostics.stagnationResetTriggered;
                end
                safeCandidateLog(updateIndex) = diagnostics.safeCandidates;
                if isfield(diagnostics, 'rejectionCounts') && ...
                        numel(diagnostics.rejectionCounts) == 6
                    rejectionLog(:, updateIndex) = ...
                        diagnostics.rejectionCounts(:);
                end
            end

            thetaCommand = abenicsIK(qCommand, params);
            thetaCommand = thetaCommand(:);
            thetaCommand = unwrapThetaNearestLocal( ...
                thetaCommand, thetaActual, params);

            thetaStart = thetaActual;
            [thetaActual, omegaActual] = plantStep( ...
                thetaActual, omegaActual, thetaCommand, params);

            [transitionAllPoleLog(updateIndex), ...
             transitionIntendedPoleLog(updateIndex)] = ...
                transitionPoleMinimum( ...
                    thetaStart, thetaActual, ...
                    params.mpc.transitionSafetySamples, ...
                    intendedPoleIndex, params);

            qActual = abenicsFK(thetaActual, params);
            qActual = qActual(:);
            actualRotation = qToRotmXYZ(qActual);
            [~, poleDistances, trackedAxis] = ...
                poleDistancesFromRotm(actualRotation, params);

            qDesPrevious = qCommand;
            sampleIndex = updateIndex + 1;
            qActualLog(:, sampleIndex) = qActual;
            thetaActualLog(:, sampleIndex) = thetaActual;
            omegaLog(:, sampleIndex) = omegaActual;
            poleDistanceLog(:, sampleIndex) = poleDistances;
            sideValueLog(sampleIndex) = dot(targetSideAxis, trackedAxis);
            physicalErrorLog(sampleIndex) = ...
                rotationDistance(actualRotation, targetRotation);
        end

        crossingIndex = find(sideValueLog >= 0, 1, 'first');
        crossedPoleSide(caseIndex) = ~isempty(crossingIndex);
        if crossedPoleSide(caseIndex)
            crossingTime(caseIndex) = (crossingIndex - 1) * params.Ts;
        end

        endpointMinimumAll = min(poleDistanceLog(:));
        transitionMinimumAll = min(transitionAllPoleLog);
        minimumAllPoleDistance = min(endpointMinimumAll, transitionMinimumAll);

        endpointMinimumIntended = min( ...
            poleDistanceLog(intendedPoleIndex, :));
        transitionMinimumIntended = min(transitionIntendedPoleLog);
        minimumIntendedPoleDistance = min( ...
            endpointMinimumIntended, transitionMinimumIntended);

        minimumAllPoleDistanceDeg(caseIndex) = ...
            rad2deg(minimumAllPoleDistance);
        minimumIntendedPoleDistanceDeg(caseIndex) = ...
            rad2deg(minimumIntendedPoleDistance);

        hardSafetyPass(caseIndex) = minimumAllPoleDistance >= ...
            params.singularity.dangerDistance - ...
            params.mpc.constraintTolerance;

        acceptedUpdates(caseIndex) = sum(acceptedLog);
        fallbackCount(caseIndex) = sum(fallbackLog);
        recoveryCount(caseIndex) = sum(recoveryLog);
        stagnationResetCount(caseIndex) = sum(stagnationResetLog);
        allUpdatesAccepted(caseIndex) = ...
            acceptedUpdates(caseIndex) == numberOfUpdates;

        finalPhysicalError = physicalErrorLog(end);
        finalPhysicalErrorDeg(caseIndex) = rad2deg(finalPhysicalError);
        targetReached(caseIndex) = ...
            finalPhysicalError <= finalPhysicalErrorLimit;

        finalEulerError = abs(wrappedDifference(qActualLog(:, end), qTarget));
        finalRollErrorDeg(caseIndex) = rad2deg(finalEulerError(1));
        finalPitchErrorDeg(caseIndex) = rad2deg(finalEulerError(2));
        finalYawErrorDeg(caseIndex) = rad2deg(finalEulerError(3));

        averageControllerTime(caseIndex) = mean(controllerTimeLog);
        worstControllerTime(caseIndex) = max(controllerTimeLog);
        averageSafeCandidates(caseIndex) = mean(safeCandidateLog);
        rejectionTotals(caseIndex, :) = sum(rejectionLog, 2).';

        nonfiniteCount = sum(~isfinite(qActualLog(:))) + ...
            sum(~isfinite(thetaActualLog(:))) + ...
            sum(~isfinite(omegaLog(:))) + ...
            sum(~isfinite(physicalErrorLog(:)));

        casePass(caseIndex) = crossedPoleSide(caseIndex) && ...
            targetReached(caseIndex) && ...
            hardSafetyPass(caseIndex) && ...
            allUpdatesAccepted(caseIndex) && ...
            fallbackCount(caseIndex) == 0 && ...
            recoveryCount(caseIndex) == 0 && ...
            nonfiniteCount == 0;

        caseLogs{caseIndex}.qActual = qActualLog;
        caseLogs{caseIndex}.thetaActual = thetaActualLog;
        caseLogs{caseIndex}.omega = omegaLog;
        caseLogs{caseIndex}.poleDistances = poleDistanceLog;
        caseLogs{caseIndex}.sideValue = sideValueLog;
        caseLogs{caseIndex}.physicalError = physicalErrorLog;
        caseLogs{caseIndex}.transitionAllPole = transitionAllPoleLog;
        caseLogs{caseIndex}.transitionIntendedPole = ...
            transitionIntendedPoleLog;
        caseLogs{caseIndex}.controllerTime = controllerTimeLog;
        caseLogs{caseIndex}.accepted = acceptedLog;
        caseLogs{caseIndex}.fallback = fallbackLog;
        caseLogs{caseIndex}.recovery = recoveryLog;
        caseLogs{caseIndex}.stagnationReset = stagnationResetLog;
        caseLogs{caseIndex}.rejections = rejectionLog;
        caseLogs{caseIndex}.motorReachability = reachability;

        fprintf('\nCASE %s RESULT\n', caseNames(caseIndex));
        fprintf('Crossed to target side:         %d\n', ...
            crossedPoleSide(caseIndex));
        fprintf('Crossing time:                  %.3f s\n', ...
            crossingTime(caseIndex));
        fprintf('Final physical rotation error:  %.3f deg\n', ...
            finalPhysicalErrorDeg(caseIndex));
        fprintf('Target reached (<= %.2f deg):   %d\n', ...
            rad2deg(finalPhysicalErrorLimit), targetReached(caseIndex));
        fprintf('Minimum distance to all poles:  %.3f deg\n', ...
            minimumAllPoleDistanceDeg(caseIndex));
        fprintf('Minimum distance to %s pole:    %.3f deg\n', ...
            caseNames(caseIndex), ...
            minimumIntendedPoleDistanceDeg(caseIndex));
        fprintf('Accepted updates:               %d/%d\n', ...
            acceptedUpdates(caseIndex), numberOfUpdates);
        fprintf('Fallback/recovery:              %d/%d\n', ...
            fallbackCount(caseIndex), recoveryCount(caseIndex));
        fprintf('Stagnation resets:              %d\n', ...
            stagnationResetCount(caseIndex));
        fprintf('Euler component errors:         [%.3f %.3f %.3f] deg\n', ...
            finalRollErrorDeg(caseIndex), ...
            finalPitchErrorDeg(caseIndex), ...
            finalYawErrorDeg(caseIndex));
        fprintf('Rejections [internal q IK theta dynamics pole]:\n');
        fprintf('                               [%d %d %d %d %d %d]\n', ...
            rejectionTotals(caseIndex, :));
        fprintf('CASE PASS:                      %d\n', ...
            casePass(caseIndex));

    catch caseException
        caseErrored(caseIndex) = true;
        errorMessage(caseIndex) = string(caseException.message);
        fprintf(2, '\nCASE %s ERROR:\n%s\n', ...
            caseNames(caseIndex), getReport(caseException, 'basic'));
    end
end

%% ========================================================================
%  AGGREGATE SUMMARY
%  ========================================================================

allPoleResults = table( ...
    caseNames, simulationTimeByCase, ...
    directPathMinimumDeg, targetThetaBoundViolationDeg, ...
    directThetaBoundViolationDeg, directMaximumThetaStepDeg, ...
    directRawBranchJumps, crossedPoleSide, crossingTime, ...
    targetReached, finalPhysicalErrorDeg, ...
    minimumIntendedPoleDistanceDeg, minimumAllPoleDistanceDeg, ...
    hardSafetyPass, acceptedUpdates, fallbackCount, recoveryCount, ...
    stagnationResetCount, ...
    finalRollErrorDeg, finalPitchErrorDeg, finalYawErrorDeg, ...
    averageControllerTime, worstControllerTime, ...
    averageSafeCandidates, ...
    rejectionTotals(:, 1), rejectionTotals(:, 2), ...
    rejectionTotals(:, 3), rejectionTotals(:, 4), ...
    rejectionTotals(:, 5), rejectionTotals(:, 6), ...
    casePass, caseErrored, errorMessage, ...
    'VariableNames', { ...
        'Pole', 'SimulationTime_s', ...
        'DirectPathMin_deg', 'TargetThetaViolation_deg', ...
        'DirectThetaViolation_deg', 'DirectMaxThetaStep_deg', ...
        'DirectRawBranchJumps', 'CrossedTargetSide', 'CrossingTime_s', ...
        'TargetReached', 'FinalPhysicalError_deg', ...
        'MinIntendedPole_deg', 'MinAllPoles_deg', ...
        'HardSafetyPass', 'AcceptedUpdates', 'Fallbacks', 'Recoveries', ...
        'StagnationResets', ...
        'FinalRollError_deg', 'FinalPitchError_deg', 'FinalYawError_deg', ...
        'AverageControllerTime_s', 'WorstControllerTime_s', ...
        'AverageSafeCandidates', ...
        'RejectInternal', 'RejectQ', 'RejectIK', 'RejectTheta', ...
        'RejectDynamics', 'RejectPole', ...
        'CasePass', 'CaseErrored', 'ErrorMessage'});

fprintf('\n\n============================================================\n');
fprintf('ABENICS SO(3) CEM ALL-SIX-POLE SUMMARY\n');
fprintf('============================================================\n');
fprintf('Hard danger distance:      %.3f deg\n', ...
    rad2deg(params.singularity.dangerDistance));
fprintf('Singularity cost weight:   %.1f\n', params.mpc.wSingularity);
fprintf('Final physical error limit %.3f deg\n', ...
    rad2deg(finalPhysicalErrorLimit));
fprintf('Cases passed:              %d / %d\n', ...
    sum(casePass), numberOfCases);
fprintf('Cases errored:             %d / %d\n', ...
    sum(caseErrored), numberOfCases);
fprintf('Closed-loop simulations:   %d\n\n', runClosedLoop);

disp(allPoleResults(:, { ...
    'Pole', 'SimulationTime_s', 'DirectPathMin_deg', ...
    'TargetThetaViolation_deg', 'DirectThetaViolation_deg', ...
    'DirectRawBranchJumps', 'CrossedTargetSide', 'CrossingTime_s', ...
    'TargetReached', 'FinalPhysicalError_deg', ...
    'MinIntendedPole_deg', 'MinAllPoles_deg', ...
    'AcceptedUpdates', 'Fallbacks', 'Recoveries', ...
    'StagnationResets', ...
    'RejectIK', 'RejectDynamics', 'RejectPole', ...
    'CasePass', 'CaseErrored'}));

fprintf('OVERALL SIX-POLE PASS:     %d\n', all(casePass));

% Clearance comparison figure.
figure('Name', 'ABENICS SO(3) CEM Six-Pole Results', ...
    'NumberTitle', 'off');
tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile;
bar(categorical(caseNames), minimumAllPoleDistanceDeg);
hold on;
yline(rad2deg(params.singularity.dangerDistance), '--', ...
    'Hard minimum', 'LineWidth', 1.5);
grid on;
ylabel('Minimum distance to any pole (deg)');
title('Closed-loop pole clearance');

nexttile;
bar(categorical(caseNames), finalPhysicalErrorDeg);
hold on;
yline(rad2deg(finalPhysicalErrorLimit), '--', ...
    'Target tolerance', 'LineWidth', 1.5);
grid on;
ylabel('Final physical orientation error (deg)');
title('Physical target convergence');

assignin('base', 'allPoleResults', allPoleResults);
assignin('base', 'allPoleCaseLogs', caseLogs);
assignin('base', 'allPoleRejectionNames', rejectionNames);

%% ========================================================================
%  LOCAL HELPERS
%  ========================================================================

function report = analyzeMotorReachabilitySO3( ...
    startRotation, targetRotation, qStart, thetaStart, samples, params)

    report.targetThetaContinuous = NaN(4, 1);
    report.targetBoundViolation = Inf;
    report.maximumBoundViolation = Inf;
    report.maximumContinuousStep = Inf;
    report.rawBranchJumps = 0;
    report.thetaMinimum = NaN(4, 1);
    report.thetaMaximum = NaN(4, 1);
    report.valid = false;

    samples = max(2, round(samples));
    relativeVector = rotmToRotvec(startRotation.' * targetRotation);
    qPrevious = qStart(:);
    thetaPrevious = thetaStart(:);
    thetaRawPrevious = thetaStart(:);
    thetaHistory = NaN(4, samples);
    maximumStep = 0;
    rawBranchJumps = 0;

    try
        for sampleIndex = 1:samples
            lambda = (sampleIndex - 1) / (samples - 1);
            rotationSample = startRotation * ...
                rotvecToRotm(lambda * relativeVector);
            qSample = rotmToQXYZContinuousLocal( ...
                rotationSample, qPrevious);
            thetaRaw = abenicsIK(qSample, params);
            thetaRaw = thetaRaw(:);
            thetaContinuous = unwrapThetaNearestLocal( ...
                thetaRaw, thetaPrevious, params);

            if sampleIndex > 1
                rawBranchJumps = rawBranchJumps + ...
                    sum(abs(thetaRaw - thetaRawPrevious) > pi);
                maximumStep = max(maximumStep, ...
                    max(abs(thetaContinuous - thetaPrevious)));
            end

            thetaHistory(:, sampleIndex) = thetaContinuous;
            qPrevious = qSample;
            thetaPrevious = thetaContinuous;
            thetaRawPrevious = thetaRaw;
        end
    catch
        return;
    end

    thetaMin = params.mpc.thetaMin(:);
    thetaMax = params.mpc.thetaMax(:);
    lowerViolation = max(0, thetaMin - thetaHistory);
    upperViolation = max(0, thetaHistory - thetaMax);
    violationHistory = max(lowerViolation, upperViolation);

    targetTheta = thetaHistory(:, end);
    targetViolation = max([ ...
        0; thetaMin - targetTheta; targetTheta - thetaMax]);

    report.targetThetaContinuous = targetTheta;
    report.targetBoundViolation = targetViolation;
    report.maximumBoundViolation = max(violationHistory(:));
    report.maximumContinuousStep = maximumStep;
    report.rawBranchJumps = rawBranchJumps;
    report.thetaMinimum = min(thetaHistory, [], 2);
    report.thetaMaximum = max(thetaHistory, [], 2);
    report.valid = true;
end

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

function q = rotmToQXYZContinuousLocal(rotation, qReference)

    qReference = qReference(:);
    sinePitch = min(1, max(-1, rotation(1, 3)));
    principalPitch = asin(sinePitch);
    cosinePitch = cos(principalPitch);

    if abs(cosinePitch) > 1e-8
        principalRoll = atan2(-rotation(2, 3), rotation(3, 3));
        principalYaw = atan2(-rotation(1, 2), rotation(1, 1));
        candidates = [ ...
            principalRoll, principalRoll + pi, principalRoll + pi;
            principalPitch, pi - principalPitch, -pi - principalPitch;
            principalYaw, principalYaw + pi, principalYaw + pi];
    elseif sinePitch >= 0
        combined = atan2(rotation(2, 1), rotation(2, 2));
        combined = nearestEquivalentAngleLocal( ...
            combined, qReference(1) + qReference(3));
        rollValue = 0.5 * ...
            (combined + qReference(1) - qReference(3));
        yawValue = combined - rollValue;
        candidates = [rollValue; pi/2; yawValue];
    else
        difference = atan2(rotation(2, 1), rotation(2, 2));
        difference = nearestEquivalentAngleLocal( ...
            difference, qReference(3) - qReference(1));
        rollValue = 0.5 * ...
            (qReference(1) + qReference(3) - difference);
        yawValue = rollValue + difference;
        candidates = [rollValue; -pi/2; yawValue];
    end

    if size(candidates, 2) == 1
        q = candidates;
        for axisIndex = 1:3
            q(axisIndex) = nearestEquivalentAngleLocal( ...
                q(axisIndex), qReference(axisIndex));
        end
        return;
    end

    bestScore = inf;
    q = candidates(:, 1);
    for candidateIndex = 1:size(candidates, 2)
        candidate = candidates(:, candidateIndex);
        for axisIndex = 1:3
            candidate(axisIndex) = nearestEquivalentAngleLocal( ...
                candidate(axisIndex), qReference(axisIndex));
        end
        score = sum((candidate - qReference).^2);
        if score < bestScore
            bestScore = score;
            q = candidate;
        end
    end
end

function adjusted = nearestEquivalentAngleLocal(angleValue, referenceValue)
    adjusted = angleValue + ...
        2*pi * round((referenceValue - angleValue) / (2*pi));
end

function [thetaNext, omegaNext] = plantStep( ...
    theta, omega, thetaCommand, params)

    KpPlant = expandParameter(params.plant.KpPlant, 4);
    KdPlant = expandParameter(params.plant.KdPlant, 4);
    thetaError = thetaCommand - theta;
    alpha = KpPlant .* thetaError - KdPlant .* omega;
    omegaNext = omega + params.Ts * alpha;
    thetaNext = theta + params.Ts * omegaNext;
end

function [minimumAll, minimumIntended] = transitionPoleMinimum( ...
    thetaStart, thetaEnd, samples, intendedPoleIndex, params)

    thetaDifference = thetaEnd - thetaStart;
    minimumAll = inf;
    minimumIntended = inf;
    samples = max(2, round(samples));

    for sampleIndex = 1:samples
        lambda = sampleIndex / samples;
        thetaSample = thetaStart + lambda * thetaDifference;
        qSample = abenicsFK(thetaSample, params);
        rotationSample = qToRotmXYZ(qSample(:));
        [sampleMinimum, distances] = ...
            poleDistancesFromRotm(rotationSample, params);
        minimumAll = min(minimumAll, sampleMinimum);
        minimumIntended = min( ...
            minimumIntended, distances(intendedPoleIndex));
    end
end

function minimumDistance = directPathPoleMinimumSO3( ...
    startRotation, targetRotation, poleIndex, samples, params)

    relativeVector = rotmToRotvec(startRotation.' * targetRotation);
    minimumDistance = inf;

    for sampleIndex = 0:(samples - 1)
        lambda = sampleIndex / (samples - 1);
        rotationSample = startRotation * rotvecToRotm(lambda * relativeVector);
        [~, distances] = poleDistancesFromRotm(rotationSample, params);
        minimumDistance = min(minimumDistance, distances(poleIndex));
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
        distances(poleIndex) = acos(min(1, max(-1, ...
            dot(pole, trackedAxis))));
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
end

function rotation = rotvecToRotm(rotationVector)
    rotationVector = rotationVector(:);
    angle = norm(rotationVector);
    skewMatrix = skew3(rotationVector);

    if angle < 1e-9
        rotation = eye(3) + skewMatrix + 0.5 * skewMatrix^2;
        return;
    end

    rotation = eye(3) + ...
        (sin(angle) / angle) * skewMatrix + ...
        ((1 - cos(angle)) / angle^2) * skewMatrix^2;
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

function matrix = skew3(vector)
    matrix = [ ...
         0,         -vector(3),  vector(2);
         vector(3),  0,         -vector(1);
        -vector(2),  vector(1),  0];
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
