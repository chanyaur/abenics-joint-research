% =========================================================================
% test_mpc_cem_final_singularity_validation.m
%
% FINAL SINGULARITY VALIDATION FOR THE ABENICS SO(3) CEM MPC
%
% This test validates the selected controller across all six signed world
% poles (+X, -X, +Y, -Y, +Z, -Z) and multiple random CEM seeds.
%
% A run passes only when it:
%   1. Reaches the target side of the intended pole.
%   2. Finishes within the physical SO(3) target-error limit.
%   3. Never enters the hard six-pole danger region.
%   4. Accepts every MPC update.
%   5. Uses no fallback or recovery command.
%   6. Produces no nonfinite values or runtime errors.
%
% IMPORTANT:
%   - Use the working single-mode SO(3) CEM controller (v2.2 baseline).
%   - This script tests singularity behavior, not real-time feasibility.
%   - "smoke" mode runs one seed first.
%   - "final" mode runs five seeds across all six poles and can take a
%     long time.
% =========================================================================

clear;
clc;
close all;

%% ========================================================================
%  USER SETTINGS -- CHANGE ONLY THIS SECTION
%  ========================================================================

% Run "smoke" first. Change to "final" only after smoke mode passes.
testMode = "final";       % "smoke" or "final"

% Candidate final horizon. Change this one value if you decide to validate
% Np = 30 instead of Np = 33.
finalNp = 33;
finalNc = 12;

% Strict final physical orientation accuracy.
finalPhysicalErrorLimitDeg = 1.0;

% Hard singularity clearance and soft warning region.
hardDangerDistanceDeg = 2.0;
warningDistanceDeg = 10.0;

% Final CEM singularity weight currently under test.
finalSingularityWeight = 4000;

% Save partial results after every completed run so a long final test is not
% lost if MATLAB is interrupted.
saveCheckpoints = true;
checkpointMatFile = "final_singularity_validation_checkpoint.mat";
resultCsvFile = "final_singularity_validation_results.csv";
summaryCsvFile = "final_singularity_validation_summary.csv";

%% ========================================================================
%  LOAD PARAMETERS
%  ========================================================================

if isfile("params_abenics_coordinate.m")
    run("params_abenics_coordinate.m");
elseif isfile("params_abenics.m")
    run("params_abenics.m");
else
    error('FinalSingularityTest:MissingParams', ...
        ['Could not find params_abenics_coordinate.m or ', ...
         'params_abenics.m in the current folder.']);
end

%% ========================================================================
%  TEST MODE
%  ========================================================================

switch lower(testMode)
    case "smoke"
        seeds = 1;
        % Enough time to expose basic crossing and settling problems.
        simulationTimeByCase = [3; 4; 4; 4; 4; 4];

    case "final"
        seeds = 1:5;
        % Uniform long validation window for final repeatability.
        simulationTimeByCase = 6 * ones(6, 1);

    otherwise
        error('FinalSingularityTest:BadMode', ...
            'testMode must be "smoke" or "final".');
end

%% ========================================================================
%  FREEZE CONTROLLER SETTINGS FOR THIS VALIDATION
%  ========================================================================

params.singularity.dangerDistance = deg2rad(hardDangerDistanceDeg);
params.singularity.warningDistance = deg2rad(warningDistanceDeg);

params.mpc.Np = finalNp;
params.mpc.Nc = finalNc;
params.mpc.wSingularity = finalSingularityWeight;
params.mpc.maxQStep = deg2rad([2; 2; 2]);

% Working v2.2 CEM configuration.
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

% Global orientation testing requires wide Euler-interface bounds.
params.mpc.qMin = deg2rad([-720; -720; -720]);
params.mpc.qMax = deg2rad([ 720;  720;  720]);

% Confirmed full +/-360 degree MP-shaft validation range.
params.mpc.thetaMin = deg2rad([-360; -360; -360; -360]);
params.mpc.thetaMax = deg2rad([ 360;  360;  360;  360]);
params.mpc.thetaUnwrapEnabled = true;
params.mpc.thetaPeriodic = true(4, 1);
params.mpc.enforceThetaPositionLimits = true;

if ~isfield(params.mpc, 'constraintTolerance')
    params.mpc.constraintTolerance = 1e-6;
end

params.mpc.debug = false;
params.mpc.liveProgress = false;
params.mpc.enableTestDiagnostics = true;

finalPhysicalErrorLimit = deg2rad(finalPhysicalErrorLimitDeg);

%% ========================================================================
%  SIX POLE-CROSSING CASES
%  ========================================================================

caseNames = ["+X"; "-X"; "+Y"; "-Y"; "+Z"; "-Z"];
poleIndices = (1:6).';

% Columns are q = [roll; pitch; yaw] in degrees.
qStartDeg = [ ...
      0,    0,    0,     0,   -15,   -15;
     15,    0,    0,     0,   -89,    89;
      0,  165,   75,   -75,     0,     0];

qTargetDeg = [ ...
      0,    0,    0,     0,    15,    15;
    -15,    0,    0,     0,   -89,    89;
      0, -165,  105,  -105,     0,     0];

numberOfSeeds = numel(seeds);
numberOfCases = numel(caseNames);
numberOfRuns = numberOfSeeds * numberOfCases;

fprintf('\n============================================================\n');
fprintf('ABENICS FINAL SINGULARITY VALIDATION\n');
fprintf('============================================================\n');
fprintf('Mode:                       %s\n', testMode);
fprintf('Controller horizon:         Np=%d, Nc=%d\n', finalNp, finalNc);
fprintf('Prediction duration:        %.3f s\n', finalNp * params.Ts);
fprintf('Seeds:                      %s\n', mat2str(seeds));
fprintf('Pole cases:                 6\n');
fprintf('Total closed-loop runs:     %d\n', numberOfRuns);
fprintf('Hard pole distance:         %.3f deg\n', hardDangerDistanceDeg);
fprintf('Final physical error limit: %.3f deg\n', ...
    finalPhysicalErrorLimitDeg);
fprintf('Singularity weight:         %.1f\n', finalSingularityWeight);
fprintf('============================================================\n');

%% ========================================================================
%  RESULT STORAGE
%  ========================================================================

passed = false(numberOfSeeds, numberOfCases);
crossed = false(numberOfSeeds, numberOfCases);
targetReached = false(numberOfSeeds, numberOfCases);
hardSafetyPass = false(numberOfSeeds, numberOfCases);
allUpdatesAccepted = false(numberOfSeeds, numberOfCases);
caseErrored = false(numberOfSeeds, numberOfCases);
nonfiniteDetected = false(numberOfSeeds, numberOfCases);

crossingTime = NaN(numberOfSeeds, numberOfCases);
finalErrorDeg = NaN(numberOfSeeds, numberOfCases);
minimumPoleDeg = NaN(numberOfSeeds, numberOfCases);
minimumIntendedPoleDeg = NaN(numberOfSeeds, numberOfCases);
directPathMinimumDeg = NaN(numberOfSeeds, numberOfCases);
acceptedUpdates = zeros(numberOfSeeds, numberOfCases);
fallbacks = zeros(numberOfSeeds, numberOfCases);
recoveries = zeros(numberOfSeeds, numberOfCases);
stagnationResets = zeros(numberOfSeeds, numberOfCases);
nonfiniteCount = zeros(numberOfSeeds, numberOfCases);
safeCandidateTotal = zeros(numberOfSeeds, numberOfCases);
candidateEvaluationTotal = zeros(numberOfSeeds, numberOfCases);
rejectionTotals = zeros(numberOfSeeds, numberOfCases, 6);
averageControllerCall = NaN(numberOfSeeds, numberOfCases);
worstControllerCall = NaN(numberOfSeeds, numberOfCases);
runtimeByRun = NaN(numberOfSeeds, numberOfCases);
controllerVersion = NaN(numberOfSeeds, numberOfCases);
errorMessage = strings(numberOfSeeds, numberOfCases);

% Optional detailed logs for smoke-mode inspection.
caseLogs = cell(numberOfSeeds, numberOfCases);

global ABENICS_CEM_LAST_DIAGNOSTICS

%% ========================================================================
%  RUN ALL REQUESTED SEEDS AND POLES
%  ========================================================================

completedRunCount = 0;

for seedIndex = 1:numberOfSeeds
    seed = seeds(seedIndex);

    for caseIndex = 1:numberOfCases
        completedRunCount = completedRunCount + 1;
        simulationTime = simulationTimeByCase(caseIndex);
        numberOfUpdates = round(simulationTime / params.Ts);
        numberOfSamples = numberOfUpdates + 1;

        fprintf('\n============================================================\n');
        fprintf('RUN %d/%d | SEED %d | POLE %s\n', ...
            completedRunCount, numberOfRuns, seed, caseNames(caseIndex));
        fprintf('============================================================\n');
        fprintf('Simulation duration: %.2f s (%d MPC updates)\n', ...
            simulationTime, numberOfUpdates);

        qStart = deg2rad(qStartDeg(:, caseIndex));
        qTarget = deg2rad(qTargetDeg(:, caseIndex));
        intendedPoleIndex = poleIndices(caseIndex);

        startRotation = qToRotmXYZ(qStart);
        targetRotation = qToRotmXYZ(qTarget);
        intendedPole = params.singularity.poleAxes(:, intendedPoleIndex);
        intendedPole = intendedPole / norm(intendedPole);

        targetTrackedAxis = trackedAxisFromRotation(targetRotation, params);
        targetSideAxis = targetTrackedAxis - ...
            dot(targetTrackedAxis, intendedPole) * intendedPole;
        if norm(targetSideAxis) < 1e-10
            error('FinalSingularityTest:UndefinedTargetSide', ...
                'Target-side tangent direction is undefined for pole %s.', ...
                caseNames(caseIndex));
        end
        targetSideAxis = targetSideAxis / norm(targetSideAxis);

        directMinimum = directPathPoleMinimumSO3( ...
            startRotation, targetRotation, intendedPoleIndex, 401, params);
        directPathMinimumDeg(seedIndex, caseIndex) = rad2deg(directMinimum);

        fprintf('Start q:                    [%.1f %.1f %.1f] deg\n', ...
            qStartDeg(:, caseIndex));
        fprintf('Target q:                   [%.1f %.1f %.1f] deg\n', ...
            qTargetDeg(:, caseIndex));
        fprintf('Direct path pole minimum:   %.3f deg\n', ...
            directPathMinimumDeg(seedIndex, caseIndex));

        try
            rng(seed, 'twister');
            clear abenicsOrientationMPC;
            ABENICS_CEM_LAST_DIAGNOSTICS = [];

            qDesPrevious = qStart;
            thetaActual = abenicsIK(qStart, params);
            thetaActual = thetaActual(:);
            omegaActual = zeros(4, 1);

            qActualLog = NaN(3, numberOfSamples);
            physicalErrorLog = NaN(1, numberOfSamples);
            minimumPoleLog = NaN(1, numberOfSamples);
            intendedPoleLog = NaN(1, numberOfSamples);
            sideValueLog = NaN(1, numberOfSamples);
            controllerTimeLog = NaN(1, numberOfUpdates);
            acceptedLog = false(1, numberOfUpdates);
            fallbackLog = false(1, numberOfUpdates);
            recoveryLog = false(1, numberOfUpdates);
            resetLog = false(1, numberOfUpdates);
            nonfiniteLog = false(1, numberOfUpdates);
            safeCandidateLog = zeros(1, numberOfUpdates);
            evaluatedCandidateLog = zeros(1, numberOfUpdates);
            rejectionLog = zeros(6, numberOfUpdates);

            qActual = callForwardKinematics(thetaActual, params);
            qActual = qActual(:);
            actualRotation = qToRotmXYZ(qActual);
            [~, poleDistances, trackedAxis] = ...
                poleDistancesFromRotm(actualRotation, params);

            qActualLog(:, 1) = qActual;
            physicalErrorLog(1) = rotationDistance( ...
                actualRotation, targetRotation);
            minimumPoleLog(1) = min(poleDistances);
            intendedPoleLog(1) = poleDistances(intendedPoleIndex);
            sideValueLog(1) = dot(targetSideAxis, trackedAxis);

            runTimer = tic;

            for updateIndex = 1:numberOfUpdates
                callTimer = tic;
                qCommand = abenicsOrientationMPC( ...
                    qTarget, thetaActual, qDesPrevious, params);
                controllerTimeLog(updateIndex) = toc(callTimer);

                diagnostics = ABENICS_CEM_LAST_DIAGNOSTICS;
                if isstruct(diagnostics) && isfield(diagnostics, 'accepted')
                    acceptedLog(updateIndex) = logical(diagnostics.accepted);
                    fallbackLog(updateIndex) = logical( ...
                        diagnostics.fallbackUsed);
                    recoveryLog(updateIndex) = logical( ...
                        diagnostics.recoveryUsed);

                    if isfield(diagnostics, 'version')
                        controllerVersion(seedIndex, caseIndex) = ...
                            diagnostics.version;
                    end
                    if isfield(diagnostics, 'stagnationResetTriggered')
                        resetLog(updateIndex) = logical( ...
                            diagnostics.stagnationResetTriggered);
                    end
                    if isfield(diagnostics, 'safeCandidates')
                        safeCandidateLog(updateIndex) = ...
                            diagnostics.safeCandidates;
                    end
                    if isfield(diagnostics, 'candidatesEvaluated')
                        evaluatedCandidateLog(updateIndex) = ...
                            diagnostics.candidatesEvaluated;
                    end
                    if isfield(diagnostics, 'rejectionCounts') && ...
                            numel(diagnostics.rejectionCounts) == 6
                        rejectionLog(:, updateIndex) = ...
                            diagnostics.rejectionCounts(:);
                    end
                else
                    % Missing diagnostics means the expected CEM controller
                    % is not active, so the strict test must fail.
                    nonfiniteLog(updateIndex) = true;
                end

                if any(~isfinite(qCommand)) || ...
                        ~isequal(size(qCommand(:)), [3, 1])
                    nonfiniteLog(updateIndex) = true;
                    break;
                end
                qCommand = qCommand(:);

                thetaCommand = abenicsIK(qCommand, params);
                thetaCommand = unwrapThetaNearestLocal( ...
                    thetaCommand(:), thetaActual, params);

                thetaStart = thetaActual;
                [thetaActual, omegaActual] = plantStep( ...
                    thetaActual, omegaActual, thetaCommand, params);

                if any(~isfinite(thetaActual)) || ...
                        any(~isfinite(omegaActual))
                    nonfiniteLog(updateIndex) = true;
                    break;
                end

                [transitionAllMinimum, transitionIntendedMinimum] = ...
                    transitionPoleMinimum( ...
                        thetaStart, thetaActual, intendedPoleIndex, ...
                        params.mpc.transitionSafetySamples, params);

                qActual = callForwardKinematics(thetaActual, params);
                qActual = qActual(:);
                actualRotation = qToRotmXYZ(qActual);
                [~, poleDistances, trackedAxis] = ...
                    poleDistancesFromRotm(actualRotation, params);

                qActualLog(:, updateIndex + 1) = qActual;
                physicalErrorLog(updateIndex + 1) = rotationDistance( ...
                    actualRotation, targetRotation);
                minimumPoleLog(updateIndex + 1) = min( ...
                    min(poleDistances), transitionAllMinimum);
                intendedPoleLog(updateIndex + 1) = min( ...
                    poleDistances(intendedPoleIndex), ...
                    transitionIntendedMinimum);
                sideValueLog(updateIndex + 1) = ...
                    dot(targetSideAxis, trackedAxis);

                qDesPrevious = qCommand;
            end

            runtimeByRun(seedIndex, caseIndex) = toc(runTimer);

            crossingIndex = find(sideValueLog >= 0, 1, 'first');
            crossed(seedIndex, caseIndex) = ~isempty(crossingIndex);
            if crossed(seedIndex, caseIndex)
                crossingTime(seedIndex, caseIndex) = ...
                    (crossingIndex - 1) * params.Ts;
            end

            finalValidIndex = find(isfinite(physicalErrorLog), 1, 'last');
            if isempty(finalValidIndex)
                finalPhysicalError = inf;
            else
                finalPhysicalError = physicalErrorLog(finalValidIndex);
            end

            finalErrorDeg(seedIndex, caseIndex) = ...
                rad2deg(finalPhysicalError);
            minimumPoleDeg(seedIndex, caseIndex) = ...
                rad2deg(min(minimumPoleLog, [], 'omitnan'));
            minimumIntendedPoleDeg(seedIndex, caseIndex) = ...
                rad2deg(min(intendedPoleLog, [], 'omitnan'));

            acceptedUpdates(seedIndex, caseIndex) = sum(acceptedLog);
            fallbacks(seedIndex, caseIndex) = sum(fallbackLog);
            recoveries(seedIndex, caseIndex) = sum(recoveryLog);
            stagnationResets(seedIndex, caseIndex) = sum(resetLog);
            nonfiniteCount(seedIndex, caseIndex) = sum(nonfiniteLog);
            nonfiniteDetected(seedIndex, caseIndex) = any(nonfiniteLog);
            safeCandidateTotal(seedIndex, caseIndex) = ...
                sum(safeCandidateLog);
            candidateEvaluationTotal(seedIndex, caseIndex) = ...
                sum(evaluatedCandidateLog);
            rejectionTotals(seedIndex, caseIndex, :) = ...
                reshape(sum(rejectionLog, 2), 1, 1, 6);

            averageControllerCall(seedIndex, caseIndex) = ...
                mean(controllerTimeLog, 'omitnan');
            worstControllerCall(seedIndex, caseIndex) = ...
                max(controllerTimeLog, [], 'omitnan');

            targetReached(seedIndex, caseIndex) = ...
                finalPhysicalError <= finalPhysicalErrorLimit;
            hardSafetyPass(seedIndex, caseIndex) = ...
                deg2rad(minimumPoleDeg(seedIndex, caseIndex)) >= ...
                params.singularity.dangerDistance - ...
                params.mpc.constraintTolerance;
            allUpdatesAccepted(seedIndex, caseIndex) = ...
                all(acceptedLog) && numel(acceptedLog) == numberOfUpdates;

            passed(seedIndex, caseIndex) = ...
                crossed(seedIndex, caseIndex) && ...
                targetReached(seedIndex, caseIndex) && ...
                hardSafetyPass(seedIndex, caseIndex) && ...
                allUpdatesAccepted(seedIndex, caseIndex) && ...
                fallbacks(seedIndex, caseIndex) == 0 && ...
                recoveries(seedIndex, caseIndex) == 0 && ...
                ~nonfiniteDetected(seedIndex, caseIndex);

            caseLogs{seedIndex, caseIndex} = struct( ...
                'qActual', qActualLog, ...
                'physicalError', physicalErrorLog, ...
                'minimumPole', minimumPoleLog, ...
                'intendedPole', intendedPoleLog, ...
                'sideValue', sideValueLog, ...
                'controllerTime', controllerTimeLog, ...
                'accepted', acceptedLog, ...
                'fallback', fallbackLog, ...
                'recovery', recoveryLog, ...
                'stagnationReset', resetLog, ...
                'rejections', rejectionLog);

        catch caughtError
            caseErrored(seedIndex, caseIndex) = true;
            errorMessage(seedIndex, caseIndex) = string( ...
                getReport(caughtError, 'extended', ...
                'hyperlinks', 'off'));
            passed(seedIndex, caseIndex) = false;
            fprintf('CASE ERROR:\n%s\n', errorMessage(seedIndex, caseIndex));
        end

        fprintf('\nCASE RESULT | SEED %d | POLE %s\n', ...
            seed, caseNames(caseIndex));
        fprintf('Crossed target side:         %d\n', ...
            crossed(seedIndex, caseIndex));
        fprintf('Crossing time:               %.3f s\n', ...
            crossingTime(seedIndex, caseIndex));
        fprintf('Final physical error:        %.3f deg\n', ...
            finalErrorDeg(seedIndex, caseIndex));
        fprintf('Minimum all-pole distance:   %.3f deg\n', ...
            minimumPoleDeg(seedIndex, caseIndex));
        fprintf('Minimum intended-pole dist.: %.3f deg\n', ...
            minimumIntendedPoleDeg(seedIndex, caseIndex));
        fprintf('Accepted updates:            %d/%d\n', ...
            acceptedUpdates(seedIndex, caseIndex), numberOfUpdates);
        fprintf('Fallback/recovery:           %d/%d\n', ...
            fallbacks(seedIndex, caseIndex), ...
            recoveries(seedIndex, caseIndex));
        fprintf('Stagnation resets:           %d\n', ...
            stagnationResets(seedIndex, caseIndex));
        fprintf('Average/worst MPC call:      %.4f / %.4f s\n', ...
            averageControllerCall(seedIndex, caseIndex), ...
            worstControllerCall(seedIndex, caseIndex));
        fprintf('Nonfinite values:            %d\n', ...
            nonfiniteCount(seedIndex, caseIndex));
        fprintf('CASE PASS:                   %d\n', ...
            passed(seedIndex, caseIndex));

        if saveCheckpoints
            save(checkpointMatFile, ...
                'testMode', 'finalNp', 'finalNc', 'seeds', ...
                'caseNames', 'qStartDeg', 'qTargetDeg', ...
                'simulationTimeByCase', 'passed', 'crossed', ...
                'targetReached', 'hardSafetyPass', ...
                'allUpdatesAccepted', 'caseErrored', ...
                'nonfiniteDetected', 'crossingTime', ...
                'finalErrorDeg', 'minimumPoleDeg', ...
                'minimumIntendedPoleDeg', 'directPathMinimumDeg', ...
                'acceptedUpdates', 'fallbacks', 'recoveries', ...
                'stagnationResets', 'nonfiniteCount', ...
                'safeCandidateTotal', 'candidateEvaluationTotal', ...
                'rejectionTotals', 'averageControllerCall', ...
                'worstControllerCall', 'runtimeByRun', ...
                'controllerVersion', 'errorMessage', 'caseLogs');
        end
    end
end

%% ========================================================================
%  FLAT RESULT TABLE
%  ========================================================================

seedColumn = repelem(seeds(:), numberOfCases, 1);
poleColumn = repmat(caseNames, numberOfSeeds, 1);
simulationColumn = repmat(simulationTimeByCase, numberOfSeeds, 1);

resultTable = table( ...
    seedColumn, poleColumn, simulationColumn, ...
    reshape(passed.', [], 1), ...
    reshape(crossed.', [], 1), ...
    reshape(targetReached.', [], 1), ...
    reshape(hardSafetyPass.', [], 1), ...
    reshape(allUpdatesAccepted.', [], 1), ...
    reshape(crossingTime.', [], 1), ...
    reshape(finalErrorDeg.', [], 1), ...
    reshape(minimumPoleDeg.', [], 1), ...
    reshape(minimumIntendedPoleDeg.', [], 1), ...
    reshape(directPathMinimumDeg.', [], 1), ...
    reshape(acceptedUpdates.', [], 1), ...
    reshape(fallbacks.', [], 1), ...
    reshape(recoveries.', [], 1), ...
    reshape(stagnationResets.', [], 1), ...
    reshape(nonfiniteCount.', [], 1), ...
    reshape(rejectionTotals(:, :, 1).', [], 1), ...
    reshape(rejectionTotals(:, :, 2).', [], 1), ...
    reshape(rejectionTotals(:, :, 3).', [], 1), ...
    reshape(rejectionTotals(:, :, 4).', [], 1), ...
    reshape(rejectionTotals(:, :, 5).', [], 1), ...
    reshape(rejectionTotals(:, :, 6).', [], 1), ...
    reshape(averageControllerCall.', [], 1), ...
    reshape(worstControllerCall.', [], 1), ...
    reshape(runtimeByRun.', [], 1), ...
    reshape(controllerVersion.', [], 1), ...
    reshape(caseErrored.', [], 1), ...
    reshape(errorMessage.', [], 1), ...
    'VariableNames', { ...
        'Seed', 'Pole', 'SimulationTime_s', ...
        'Pass', 'Crossed', 'TargetReached', 'HardSafetyPass', ...
        'AllUpdatesAccepted', 'CrossingTime_s', ...
        'FinalPhysicalError_deg', 'MinimumAllPoleDistance_deg', ...
        'MinimumIntendedPoleDistance_deg', ...
        'DirectPathMinimum_deg', 'AcceptedUpdates', ...
        'Fallbacks', 'Recoveries', 'StagnationResets', ...
        'NonfiniteCount', 'RejectInternal', 'RejectQBounds', ...
        'RejectIK', 'RejectTheta', 'RejectDynamics', 'RejectPole', ...
        'AverageControllerCall_s', 'WorstControllerCall_s', ...
        'Runtime_s', 'ControllerVersion', 'CaseErrored', ...
        'ErrorMessage'});

%% ========================================================================
%  PER-POLE FINAL SUMMARY
%  ========================================================================

polePassCount = sum(passed, 1).';
poleRunCount = numberOfSeeds * ones(numberOfCases, 1);
poleAllPassed = polePassCount == poleRunCount;
worstFinalErrorDeg = max(finalErrorDeg, [], 1, 'omitnan').';
worstMinimumClearanceDeg = min(minimumPoleDeg, [], 1, 'omitnan').';
meanCrossingTime = mean(crossingTime, 1, 'omitnan').';
totalFallbacksByPole = sum(fallbacks, 1).';
totalRecoveriesByPole = sum(recoveries, 1).';
meanControllerCallByPole = mean( ...
    averageControllerCall, 1, 'omitnan').';
worstControllerCallByPole = max( ...
    worstControllerCall, [], 1, 'omitnan').';

summaryTable = table( ...
    caseNames, polePassCount, poleRunCount, poleAllPassed, ...
    worstFinalErrorDeg, worstMinimumClearanceDeg, ...
    meanCrossingTime, totalFallbacksByPole, ...
    totalRecoveriesByPole, meanControllerCallByPole, ...
    worstControllerCallByPole, ...
    'VariableNames', { ...
        'Pole', 'PassCount', 'RunCount', 'AllSeedsPassed', ...
        'WorstFinalPhysicalError_deg', ...
        'WorstMinimumPoleDistance_deg', ...
        'MeanCrossingTime_s', 'TotalFallbacks', ...
        'TotalRecoveries', 'MeanControllerCall_s', ...
        'WorstControllerCall_s'});

fprintf('\n============================================================\n');
fprintf('FINAL SINGULARITY VALIDATION RESULTS\n');
fprintf('============================================================\n');
disp(resultTable);

fprintf('\n============================================================\n');
fprintf('PER-POLE SUMMARY\n');
fprintf('============================================================\n');
disp(summaryTable);

successfulRuns = sum(passed(:));
overallPass = all(passed(:));

fprintf('\n============================================================\n');
fprintf('FINAL DECISION\n');
fprintf('============================================================\n');
fprintf('Successful runs:             %d / %d\n', ...
    successfulRuns, numberOfRuns);
fprintf('All six poles passed:        %d\n', all(poleAllPassed));
fprintf('No hard safety violations:   %d\n', all(hardSafetyPass(:)));
fprintf('No fallback commands:        %d\n', all(fallbacks(:) == 0));
fprintf('No recovery commands:        %d\n', all(recoveries(:) == 0));
fprintf('No nonfinite values:         %d\n', ...
    all(nonfiniteCount(:) == 0));
fprintf('OVERALL FINAL PASS:          %d\n', overallPass);

if testMode == "smoke"
    if overallPass
        fprintf(['\nSMOKE TEST PASSED. Change testMode to "final" ', ...
            'and rerun for the five-seed validation.\n']);
    else
        fprintf(['\nSMOKE TEST FAILED. Do not run final mode yet. ', ...
            'Inspect the failed pole rows first.\n']);
    end
else
    if overallPass
        fprintf(['\nFINAL SINGULARITY VALIDATION PASSED. ', ...
            'The six-pole avoidance behavior is repeatable for the ', ...
            'tested seeds and constraints.\n']);
    else
        fprintf(['\nFINAL SINGULARITY VALIDATION FAILED. ', ...
            'Do not declare the singularity controller final.\n']);
    end
end

writetable(resultTable, resultCsvFile);
writetable(summaryTable, summaryCsvFile);

assignin('base', 'finalSingularityValidationResults', resultTable);
assignin('base', 'finalSingularityValidationSummary', summaryTable);
assignin('base', 'finalSingularityValidationPassMatrix', passed);
assignin('base', 'finalSingularityValidationOverallPass', overallPass);
assignin('base', 'finalSingularityValidationLogs', caseLogs);

%% ========================================================================
%  SUMMARY PLOTS
%  ========================================================================

figure('Name', 'ABENICS Final Singularity Validation', ...
    'NumberTitle', 'off');

tiledlayout(2, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile;
imagesc(double(passed));
axis tight;
xticks(1:numberOfCases);
xticklabels(caseNames);
yticks(1:numberOfSeeds);
yticklabels(string(seeds));
xlabel('Intended singular pole');
ylabel('Random seed');
title(sprintf('Strict pass matrix | Np=%d, Nc=%d', finalNp, finalNc));
colorbar('Ticks', [0, 1], 'TickLabels', {'Fail', 'Pass'});

nexttile;
hold on;
for caseIndex = 1:numberOfCases
    xValues = caseIndex * ones(numberOfSeeds, 1);
    plot(xValues, minimumPoleDeg(:, caseIndex), 'o', ...
        'DisplayName', caseNames(caseIndex));
end
yline(hardDangerDistanceDeg, '--', 'Hard danger distance');
xlim([0.5, numberOfCases + 0.5]);
xticks(1:numberOfCases);
xticklabels(caseNames);
ylabel('Minimum six-pole distance (deg)');
xlabel('Intended singular pole');
title('Minimum clearance for every seed');
grid on;
hold off;

% =========================================================================
% LOCAL FUNCTIONS
% =========================================================================

function q = callForwardKinematics(theta, params)
    % Prefer the project-standard analytic FK name. Retain compatibility
    % with repositories that still use abenicsFK.m.
    if exist('abenicsFL', 'file') == 2
        q = abenicsFL(theta, params);
    elseif exist('abenicsFK', 'file') == 2
        q = abenicsFK(theta, params);
    else
        error('FinalSingularityTest:MissingFK', ...
            'Neither abenicsFL.m nor abenicsFK.m was found.');
    end
end

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

function [minimumAll, minimumIntended] = transitionPoleMinimum( ...
    thetaStart, thetaEnd, intendedPoleIndex, samples, params)

    minimumAll = inf;
    minimumIntended = inf;
    thetaDifference = thetaEnd - thetaStart;

    for sampleIndex = 1:samples
        lambda = sampleIndex / samples;
        thetaSample = thetaStart + lambda * thetaDifference;
        qSample = callForwardKinematics(thetaSample, params);
        [~, distances] = poleDistancesFromRotm( ...
            qToRotmXYZ(qSample(:)), params);
        minimumAll = min(minimumAll, min(distances));
        minimumIntended = min( ...
            minimumIntended, distances(intendedPoleIndex));
    end
end

function minimumDistance = directPathPoleMinimumSO3( ...
    startRotation, targetRotation, poleIndex, samples, params)

    relativeVector = rotmToRotvec(startRotation.' * targetRotation);
    minimumDistance = inf;

    for sampleIndex = 0:(samples - 1)
        lambda = sampleIndex / max(1, samples - 1);
        sampleRotation = startRotation * ...
            rotvecToRotm(lambda * relativeVector);
        [~, distances] = poleDistancesFromRotm(sampleRotation, params);
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

function rotation = rotvecToRotm(rotationVector)
    rotationVector = rotationVector(:);
    angle = norm(rotationVector);
    skewMatrix = skew3(rotationVector);

    if angle < 1e-9
        rotation = eye(3) + skewMatrix + 0.5 * skewMatrix^2;
        return;
    end

    coefficientOne = sin(angle) / angle;
    coefficientTwo = (1 - cos(angle)) / angle^2;
    rotation = eye(3) + coefficientOne * skewMatrix + ...
        coefficientTwo * skewMatrix^2;
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
            axis(2) = signNonzero(rotation(1, 2) + rotation(2, 1)) * ...
                axis(2);
            axis(3) = signNonzero(rotation(1, 3) + rotation(3, 1)) * ...
                axis(3);
        elseif largestIndex == 2
            axis(1) = signNonzero(rotation(1, 2) + rotation(2, 1)) * ...
                axis(1);
            axis(3) = signNonzero(rotation(2, 3) + rotation(3, 2)) * ...
                axis(3);
        else
            axis(1) = signNonzero(rotation(1, 3) + rotation(3, 1)) * ...
                axis(1);
            axis(2) = signNonzero(rotation(2, 3) + rotation(3, 2)) * ...
                axis(2);
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

function vector = expandParameter(value, numberOfElements)
    value = value(:);

    if numel(value) == 1
        vector = value * ones(numberOfElements, 1);
    elseif numel(value) == numberOfElements
        vector = value;
    else
        error('FinalSingularityTest:ParameterSize', ...
            'Expected a scalar or %dx1 parameter.', numberOfElements);
    end
end
