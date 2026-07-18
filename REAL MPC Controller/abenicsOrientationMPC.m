function q_des_mpc = abenicsOrientationMPC( ...
    q_ref, theta_actual, q_des_prev, params)
%ABENICSORIENTATIONMPC SO(3) sampling-based nonlinear MPC using CEM.
%
% Public interface is unchanged:
%   q_des_mpc = abenicsOrientationMPC( ...
%       q_ref, theta_actual, q_des_prev, params)
%
% Inputs:
%   q_ref         3x1 current desired CS-gear orientation [roll; pitch; yaw], rad
%   theta_actual  4x1 measured output-side MP angles, rad
%   q_des_prev    3x1 previously applied MPC orientation command, rad
%   params        ABENICS parameter structure
%
% Output:
%   q_des_mpc     3x1 orientation command sent to IK
%
% This controller is a true receding-horizon MPC. At every update it:
%   1. Samples many continuous local rotation-vector command sequences.
%   2. Predicts each sequence through IK, the internal plant, FK, and the
%      six-pole singularity detector.
%   3. Hard-rejects any sequence that violates orientation, motor, dynamic,
%      or pole-distance limits.
%   4. Refines the sampling distribution around the best safe sequences.
%   5. Applies only the first command of the best predicted sequence.
%
% Candidate orientations are propagated on SO(3) with local rotation
% vectors. Euler angles are used only at the public interface and IK call.
% Periodic IK motor angles are unwrapped to the nearest equivalent value
% before motor-limit, dynamics, and smoothness calculations.
% A fixed-spread broad subset, covariance floor, and stagnation reset
% prevent premature convergence without prescribing a detour route.
% Optional reference preview is supplied through params.mpc.qRefHorizon
% (3xNp) when params.mpc.useReferencePreview is true. Otherwise, the
% current q_ref is repeated across the horizon for backward compatibility.
% No external route planner, fixed detour path, or fmincon solve is used.

    persistent thetaPrevious
    persistent previousBestDeltaPhi
    persistent previousStdKnots
    persistent previousReference
    persistent updateCounter
    persistent bestTargetError
    persistent stagnationCounter
    persistent stagnationResetCount

    q_ref = q_ref(:);
    theta_actual = theta_actual(:);
    q_des_prev = q_des_prev(:);

    validateInputs(q_ref, theta_actual, q_des_prev, params);

    Ts = params.Ts;
    Np = max(1, round(readMpcSetting(params, 'Np', 33)));
    Nc = max(1, round(readMpcSetting(params, 'Nc', 12)));
    qRefHorizon = buildReferenceHorizon(q_ref, Np, params);
    previewTerminalReference = qRefHorizon(:, end);
    numberOfKnots = max(2, round(readMpcSetting( ...
        params, 'cemNumberOfKnots', 4)));
    numberOfKnots = min(numberOfKnots, Nc);

    maxQStep = expandToVector(readMpcSetting( ...
        params, 'maxQStep', deg2rad([2; 2; 2])), 3);

    if isempty(thetaPrevious) || ~isequal(size(thetaPrevious), [4, 1])
        thetaActualContinuous = theta_actual;
        omegaEstimate = zeros(4, 1);
    else
        thetaActualContinuous = unwrapThetaNearest( ...
            theta_actual, thetaPrevious, params);
        omegaEstimate = thetaContinuousDifference( ...
            thetaActualContinuous, thetaPrevious, params) / Ts;
    end
    x0 = [thetaActualContinuous; omegaEstimate];

    if isempty(updateCounter)
        updateCounter = 0;
    end
    updateCounter = updateCounter + 1;

    resetReferenceThreshold = readMpcSetting( ...
        params, 'cemReferenceResetThreshold', deg2rad(8));
    referenceChanged = isempty(previousReference) || ...
        ~isequal(size(previousReference), [3, 1]) || ...
        rotationDistanceFromQ(q_ref, previousReference) > ...
        resetReferenceThreshold;

    warmDeltaPhi = buildWarmStartDeltaPhi( ...
        previousBestDeltaPhi, previewTerminalReference, q_des_prev, ...
        maxQStep, Nc, referenceChanged);
    meanKnots = compressDeltaPhiToKnots(warmDeltaPhi, numberOfKnots);

    initialStd = expandToVector(readMpcSetting( ...
        params, 'cemInitialStd', deg2rad([1.25; 1.25; 1.25])), 3);
    minimumStd = expandToVector(readMpcSetting( ...
        params, 'cemMinimumStd', deg2rad([0.10; 0.10; 0.10])), 3);
    maximumStd = expandToVector(readMpcSetting( ...
        params, 'cemMaximumStd', deg2rad([2.0; 2.0; 2.0])), 3);

    if referenceChanged || isempty(previousStdKnots) || ...
            ~isequal(size(previousStdKnots), [3, numberOfKnots]) || ...
            any(~isfinite(previousStdKnots(:)))
        stdKnots = repmat(initialStd, 1, numberOfKnots);
    else
        warmInflation = readMpcSetting(params, 'cemWarmStartStdInflation', 1.25);
        stdKnots = warmInflation * previousStdKnots;
        stdKnots = max(stdKnots, repmat(minimumStd, 1, numberOfKnots));
        stdKnots = min(stdKnots, repmat(maximumStd, 1, numberOfKnots));
    end

    q_current = abenicsFK(thetaActualContinuous, params);
    q_current = q_current(:);
    [currentSingularity, currentPoleDistances] = ...
        poleDistancesFromQ(q_current, params);
    currentTargetError = rotationDistanceFromQ(q_current, q_ref);

    % Keep a meaningful search spread while the target is still far away.
    % Near the target, return to the smaller settling floor so final accuracy
    % is not sacrificed by permanent large random commands.
    sigmaFloor = expandToVector(readMpcSetting( ...
        params, 'cemSigmaFloor', deg2rad([0.35; 0.35; 0.35])), 3);
    settlingErrorThreshold = readMpcSetting( ...
        params, 'cemSettlingErrorThreshold', deg2rad(3.0));
    if currentTargetError > settlingErrorThreshold
        effectiveMinimumStd = max(minimumStd, sigmaFloor);
    else
        effectiveMinimumStd = minimumStd;
    end
    stdKnots = max(stdKnots, ...
        repmat(effectiveMinimumStd, 1, numberOfKnots));

    % Detect repeated lack of physical target-error improvement. A reset
    % widens the distribution and re-centers it toward the direct SO(3)
    % command, where broad antithetic samples can rediscover either detour
    % side without prescribing a route.
    stagnationTolerance = readMpcSetting( ...
        params, 'cemStagnationTolerance', deg2rad(0.25));
    stagnationUpdatesRequired = max(1, round(readMpcSetting( ...
        params, 'cemStagnationUpdates', 8)));
    stagnationDisableBelowError = readMpcSetting( ...
        params, 'cemStagnationDisableBelowError', deg2rad(3.0));

    if isempty(stagnationResetCount)
        stagnationResetCount = 0;
    end
    if referenceChanged || isempty(bestTargetError) || ...
            ~isscalar(bestTargetError) || ~isfinite(bestTargetError)
        bestTargetError = currentTargetError;
        stagnationCounter = 0;
    elseif currentTargetError < bestTargetError - stagnationTolerance
        bestTargetError = currentTargetError;
        stagnationCounter = 0;
    else
        if isempty(stagnationCounter)
            stagnationCounter = 0;
        end
        stagnationCounter = stagnationCounter + 1;
    end

    stagnationResetTriggered = false;
    if currentTargetError > stagnationDisableBelowError && ...
            stagnationCounter >= stagnationUpdatesRequired
        directDeltaPhiForReset = buildDirectDeltaPhi( ...
            previewTerminalReference, q_des_prev, maxQStep, Nc);
        directKnotsForReset = compressDeltaPhiToKnots( ...
            directDeltaPhiForReset, numberOfKnots);
        resetScale = readMpcSetting( ...
            params, 'cemCovarianceResetScale', 2.5);
        directBlend = min(1, max(0, readMpcSetting( ...
            params, 'cemStagnationMeanDirectBlend', 0.75)));
        resetStd = max(repmat(initialStd, 1, numberOfKnots), ...
            resetScale * stdKnots);
        resetStd = min(resetStd, repmat(maximumStd, 1, numberOfKnots));
        stdKnots = max(resetStd, ...
            repmat(effectiveMinimumStd, 1, numberOfKnots));
        meanKnots = (1 - directBlend) * meanKnots + ...
            directBlend * directKnotsForReset;
        meanKnots = clampMatrixByRows( ...
            meanKnots, -maxQStep, maxQStep);

        stagnationResetTriggered = true;
        stagnationResetCount = stagnationResetCount + 1;
        stagnationCounter = 0;
        bestTargetError = currentTargetError;
    end

    warningDistance = params.singularity.warningDistance;
    nearWarningInflation = readMpcSetting( ...
        params, 'cemNearSingularityStdInflation', 1.5);
    if currentSingularity < warningDistance
        stdKnots = min( ...
            nearWarningInflation * stdKnots, ...
            repmat(maximumStd, 1, numberOfKnots));
    end

    liveProgress = logical(readMpcSetting(params, 'liveProgress', false));
    if liveProgress
        fprintf(['\n[CEM] update=%d | q=[%.3f %.3f %.3f] deg | ', ...
                 'targetErr=%.3f deg | minPole=%.3f deg | ', ...
                 'stagnation=%d reset=%d | Np=%d Nc=%d knots=%d\n'], ...
            updateCounter, rad2deg(q_current(1)), rad2deg(q_current(2)), ...
            rad2deg(q_current(3)), rad2deg(currentTargetError), ...
            rad2deg(currentSingularity), stagnationCounter, ...
            stagnationResetTriggered, Np, Nc, numberOfKnots);
        drawnow;
    end

    solveTimer = tic;
    [best, cemDiagnostics, ~, finalStd] = runCemSearch( ...
        meanKnots, stdKnots, x0, qRefHorizon, q_des_prev, ...
        maxQStep, effectiveMinimumStd, initialStd, ...
        Np, Nc, params, liveProgress);
    solveTime = toc(solveTimer);

    fallbackUsed = false;
    recoveryUsed = false;
    accepted = best.valid;
    fallbackReason = "none";

    if best.valid
        previousRotation = qToRotmXYZ(q_des_prev);
        candidateRotation = previousRotation * ...
            rotvecToRotm(best.deltaPhi(:, 1));
        q_candidate = rotmToQXYZContinuous( ...
            candidateRotation, q_des_prev);
        [firstValid, ~] = validateOneStepCommand( ...
            q_candidate, q_des_prev, x0, params);

        if firstValid
            q_des_mpc = q_candidate;
        else
            accepted = false;
            fallbackReason = "bestFirstCommandFailedIndependentValidation";
        end
    end

    if ~accepted
        fallbackUsed = true;
        [q_fallback, fallbackValid, fallbackInfo] = ...
            chooseSafeFallbackCommand( ...
                q_ref, q_des_prev, x0, currentSingularity, ...
                currentPoleDistances, params);

        if fallbackValid
            q_des_mpc = q_fallback;
            recoveryUsed = fallbackInfo.recoveryMode;
            if fallbackReason == "none"
                fallbackReason = fallbackInfo.reason;
            end
        else
            % Last-resort hold. This is reported as invalid rather than
            % silently claiming safety.
            q_des_mpc = q_des_prev;
            fallbackReason = "noValidatedSafeCandidate";
        end
    end


    % Only a fully safe CEM winner updates the warm-start trajectory.
    if accepted
        previousBestDeltaPhi = best.deltaPhi;
        previousStdKnots = finalStd;
    elseif isempty(previousBestDeltaPhi) || ...
            ~isequal(size(previousBestDeltaPhi), [3, Nc])
        previousBestDeltaPhi = zeros(3, Nc);
        previousStdKnots = repmat(initialStd, 1, numberOfKnots);
    else
        previousBestDeltaPhi = shiftDeltaPhi(previousBestDeltaPhi);
        previousStdKnots = max( ...
            finalStd, repmat(effectiveMinimumStd, 1, numberOfKnots));
    end

    thetaPrevious = thetaActualContinuous;
    previousReference = q_ref;

    diagnostics.version = 2.2;
    diagnostics.referencePreviewUsed = logical(readMpcSetting( ...
        params, 'useReferencePreview', false));
    diagnostics.previewTerminalReference = previewTerminalReference;
    diagnostics.update = updateCounter;
    diagnostics.accepted = accepted;
    diagnostics.fallbackUsed = fallbackUsed;
    diagnostics.recoveryUsed = recoveryUsed;
    diagnostics.fallbackReason = fallbackReason;
    diagnostics.population = cemDiagnostics.population;
    diagnostics.iterationsRequested = cemDiagnostics.iterationsRequested;
    diagnostics.iterationsCompleted = cemDiagnostics.iterationsCompleted;
    diagnostics.candidatesEvaluated = cemDiagnostics.candidatesEvaluated;
    diagnostics.safeCandidates = cemDiagnostics.safeCandidates;
    diagnostics.bestCost = best.cost;
    diagnostics.bestMinimumPoleDistance = best.minimumPoleDistance;
    diagnostics.bestFirstTransitionPoleDistance = ...
        best.firstTransitionPoleDistance;
    diagnostics.solveTime = solveTime;
    diagnostics.currentMinimumPoleDistance = currentSingularity;
    diagnostics.currentTargetError = currentTargetError;
    diagnostics.finalMeanStd = mean(finalStd, 2);
    diagnostics.effectiveMinimumStd = effectiveMinimumStd;
    diagnostics.explorationCandidatesPerIteration = ...
        cemDiagnostics.explorationCandidatesPerIteration;
    diagnostics.stagnationCounter = stagnationCounter;
    diagnostics.stagnationResetTriggered = stagnationResetTriggered;
    diagnostics.stagnationResetCount = stagnationResetCount;
    diagnostics.rejectionCounts = cemDiagnostics.rejectionCounts;

    publishDiagnostics(diagnostics, params);

    if isfield(params, 'mpc') && isfield(params.mpc, 'debug') && ...
            params.mpc.debug
        fprintf([ ...
            'MPCUPDATE version=cem2.2so3robust update=%d accepted=%d fallback=%d ', ...
            'recovery=%d population=%d iterations=%d evaluated=%d ', ...
            'safe=%d bestCost=%.12g bestMinPole=%.9g ', ...
            'firstTransitionPole=%.9g currentMinPole=%.9g ', ...
            'targetError=%.9g stagnation=%d reset=%d resetCount=%d ', ...
            'explorationPerIter=%d solveTime=%.6f ', ...
            'meanStdRoll=%.9g meanStdPitch=%.9g ', ...
            'meanStdYaw=%.9g rejectNonfinite=%d rejectQ=%d ', ...
            'rejectIK=%d rejectTheta=%d rejectDynamics=%d ', ...
            'rejectPole=%d reason=%s\n'], ...
            diagnostics.update, diagnostics.accepted, ...
            diagnostics.fallbackUsed, diagnostics.recoveryUsed, ...
            diagnostics.population, diagnostics.iterationsCompleted, ...
            diagnostics.candidatesEvaluated, diagnostics.safeCandidates, ...
            diagnostics.bestCost, diagnostics.bestMinimumPoleDistance, ...
            diagnostics.bestFirstTransitionPoleDistance, ...
            diagnostics.currentMinimumPoleDistance, ...
            diagnostics.currentTargetError, ...
            diagnostics.stagnationCounter, ...
            diagnostics.stagnationResetTriggered, ...
            diagnostics.stagnationResetCount, ...
            diagnostics.explorationCandidatesPerIteration, ...
            diagnostics.solveTime, diagnostics.finalMeanStd(1), ...
            diagnostics.finalMeanStd(2), diagnostics.finalMeanStd(3), ...
            diagnostics.rejectionCounts(1), ...
            diagnostics.rejectionCounts(2), ...
            diagnostics.rejectionCounts(3), ...
            diagnostics.rejectionCounts(4), ...
            diagnostics.rejectionCounts(5), ...
            diagnostics.rejectionCounts(6), ...
            char(diagnostics.fallbackReason));
    end
end

% =========================================================================
% CEM search
% =========================================================================
function [best, diagnostics, meanKnots, stdKnots] = runCemSearch( ...
    meanKnots, stdKnots, x0, qRefHorizon, q_des_prev, ...
    maxQStep, effectiveMinimumStd, initialStd, ...
    Np, Nc, params, liveProgress)

    population = max(8, round(readMpcSetting( ...
        params, 'cemPopulationSize', 64)));
    if mod(population, 2) ~= 0
        population = population + 1;
    end

    iterations = max(1, round(readMpcSetting( ...
        params, 'cemIterations', 3)));
    eliteFraction = readMpcSetting(params, 'cemEliteFraction', 0.15);
    eliteCount = max(2, min(population, round(eliteFraction * population)));
    smoothing = readMpcSetting(params, 'cemSmoothing', 0.70);
    temporalCorrelation = readMpcSetting( ...
        params, 'cemTemporalCorrelation', 0.85);
    explorationFraction = readMpcSetting( ...
        params, 'cemExplorationFraction', 0.30);
    explorationScale = readMpcSetting( ...
        params, 'cemExplorationScale', 1.50);
    progressEveryCandidates = max(1, round(readMpcSetting( ...
        params, 'cemProgressEveryCandidates', 16)));

    minimumStd = expandToVector(effectiveMinimumStd, 3);
    maximumStd = expandToVector(readMpcSetting( ...
        params, 'cemMaximumStd', deg2rad([2.0; 2.0; 2.0])), 3);
    explorationStd = expandToVector(readMpcSetting( ...
        params, 'cemExplorationStd', initialStd), 3);
    minStdMatrix = repmat(minimumStd, 1, size(meanKnots, 2));
    maxStdMatrix = repmat(maximumStd, 1, size(meanKnots, 2));
    explorationStdMatrix = repmat( ...
        explorationStd, 1, size(meanKnots, 2));

    directDeltaPhi = buildDirectDeltaPhi( ...
        qRefHorizon(:, end), q_des_prev, maxQStep, Nc);
    directKnots = compressDeltaPhiToKnots( ...
        directDeltaPhi, size(meanKnots, 2));

    best = emptyCandidateResult(Nc);
    diagnostics.population = population;
    diagnostics.iterationsRequested = iterations;
    diagnostics.iterationsCompleted = 0;
    diagnostics.candidatesEvaluated = 0;
    diagnostics.safeCandidates = 0;
    diagnostics.rejectionCounts = zeros(6, 1);
    pairCount = floor((population - 3) / 2);
    exploratoryPairs = round(explorationFraction * pairCount);
    diagnostics.explorationCandidatesPerIteration = ...
        min(population - 3, 2 * exploratoryPairs);

    for iteration = 1:iterations
        iterationTimer = tic;
        candidateKnots = sampleCandidateKnots( ...
            meanKnots, stdKnots, directKnots, maxQStep, ...
            population, temporalCorrelation, ...
            explorationFraction, explorationScale, ...
            explorationStdMatrix);

        costs = inf(population, 1);
        results = repmat(emptyCandidateResult(Nc), population, 1);
        iterationSafe = 0;
        iterationRejections = zeros(6, 1);

        for candidateIndex = 1:population
            deltaPhi = expandKnotsToDeltaPhi( ...
                candidateKnots(:, :, candidateIndex), Nc, maxQStep);
            result = evaluateCandidate( ...
                deltaPhi, x0, qRefHorizon, q_des_prev, Np, Nc, params);
            results(candidateIndex) = result;
            diagnostics.candidatesEvaluated = ...
                diagnostics.candidatesEvaluated + 1;

            if result.valid
                costs(candidateIndex) = result.cost;
                iterationSafe = iterationSafe + 1;
                diagnostics.safeCandidates = diagnostics.safeCandidates + 1;
                if ~best.valid || result.cost < best.cost
                    best = result;
                end
            else
                rejectionIndex = max(1, min(6, result.rejectionCode));
                iterationRejections(rejectionIndex) = ...
                    iterationRejections(rejectionIndex) + 1;
                diagnostics.rejectionCounts(rejectionIndex) = ...
                    diagnostics.rejectionCounts(rejectionIndex) + 1;
            end

            if liveProgress && ...
                    (mod(candidateIndex, progressEveryCandidates) == 0 || ...
                     candidateIndex == population)
                fprintf([ ...
                    '[CEM] iter=%d/%d | evaluated=%d/%d | safe=%d | ', ...
                    'elapsed=%.2f s\n'], ...
                    iteration, iterations, candidateIndex, population, ...
                    iterationSafe, toc(iterationTimer));
                drawnow;
            end
        end

        diagnostics.iterationsCompleted = iteration;

        safeIndices = find(isfinite(costs));
        if isempty(safeIndices)
            % Re-expand the search instead of collapsing around infeasibility.
            stdKnots = min(1.35 * stdKnots, maxStdMatrix);
            meanKnots = 0.5 * meanKnots + 0.5 * directKnots;

            if liveProgress
                fprintf([ ...
                    '[CEM] iter=%d/%d | safe=0/%d | ', ...
                    'expanding search | std=[%.3f %.3f %.3f] deg\n'], ...
                    iteration, iterations, population, ...
                    rad2deg(mean(stdKnots(1, :))), ...
                    rad2deg(mean(stdKnots(2, :))), ...
                    rad2deg(mean(stdKnots(3, :))));
                drawnow;
            end
            continue;
        end

        [~, order] = sort(costs, 'ascend');
        eliteIndices = order(1:min(eliteCount, numel(safeIndices)));
        eliteIndices = eliteIndices(isfinite(costs(eliteIndices)));

        eliteKnots = candidateKnots(:, :, eliteIndices);
        rankWeights = log(numel(eliteIndices) + 0.5) - ...
            log((1:numel(eliteIndices))');
        rankWeights = rankWeights / sum(rankWeights);

        newMean = zeros(size(meanKnots));
        for eliteRank = 1:numel(eliteIndices)
            newMean = newMean + rankWeights(eliteRank) * ...
                eliteKnots(:, :, eliteRank);
        end

        newVariance = zeros(size(stdKnots));
        for eliteRank = 1:numel(eliteIndices)
            difference = eliteKnots(:, :, eliteRank) - newMean;
            newVariance = newVariance + rankWeights(eliteRank) * ...
                difference.^2;
        end
        newStd = sqrt(max(newVariance, 0));

        meanKnots = smoothing * newMean + (1 - smoothing) * meanKnots;
        stdKnots = smoothing * newStd + (1 - smoothing) * stdKnots;
        stdKnots = max(stdKnots, minStdMatrix);
        stdKnots = min(stdKnots, maxStdMatrix);
        meanKnots = clampMatrixByRows(meanKnots, -maxQStep, maxQStep);

        bestIterationIndex = safeIndices( ...
            find(costs(safeIndices) == min(costs(safeIndices)), 1, 'first'));
        bestIteration = results(bestIterationIndex);

        if liveProgress
            fprintf([ ...
                '[CEM] iter=%d/%d | safe=%d/%d | best=%.6g | ', ...
                'minPole=%.3f deg | firstPole=%.3f deg | ', ...
                'std=[%.3f %.3f %.3f] deg | elapsed=%.2f s | ', ...
                'reject=[%d %d %d %d %d %d]\n'], ...
                iteration, iterations, iterationSafe, population, ...
                bestIteration.cost, ...
                rad2deg(bestIteration.minimumPoleDistance), ...
                rad2deg(bestIteration.firstTransitionPoleDistance), ...
                rad2deg(mean(stdKnots(1, :))), ...
                rad2deg(mean(stdKnots(2, :))), ...
                rad2deg(mean(stdKnots(3, :))), toc(iterationTimer), ...
                iterationRejections(1), iterationRejections(2), ...
                iterationRejections(3), iterationRejections(4), ...
                iterationRejections(5), iterationRejections(6));
            drawnow;
        end
    end
end

% =========================================================================
% Candidate sampling
% =========================================================================
function candidateKnots = sampleCandidateKnots( ...
    meanKnots, stdKnots, directKnots, maxQStep, population, ...
    temporalCorrelation, explorationFraction, explorationScale, ...
    explorationStdKnots)

    numberOfKnots = size(meanKnots, 2);
    candidateKnots = zeros(3, numberOfKnots, population);

    candidateKnots(:, :, 1) = meanKnots;
    candidateKnots(:, :, 2) = zeros(3, numberOfKnots);
    candidateKnots(:, :, 3) = directKnots;

    nextIndex = 4;
    pairCount = floor((population - 3) / 2);
    exploratoryPairs = round(explorationFraction * pairCount);

    for pairIndex = 1:pairCount
        noise = temporallyCorrelatedNoise( ...
            3, numberOfKnots, temporalCorrelation);

        if pairIndex <= exploratoryPairs
            % Broad samples never shrink with the learned covariance. Half
            % remain goal-directed around the direct command and half search
            % around the learned route. Antithetic pairs test both lateral
            % detour directions with the same random magnitude.
            if mod(pairIndex, 2) == 1
                center = directKnots;
            else
                center = meanKnots;
            end
            scale = max( ...
                explorationScale * stdKnots, explorationStdKnots);
        else
            center = meanKnots;
            scale = stdKnots;
        end

        positiveSample = center + scale .* noise;
        negativeSample = center - scale .* noise;

        candidateKnots(:, :, nextIndex) = ...
            clampMatrixByRows(positiveSample, -maxQStep, maxQStep);
        nextIndex = nextIndex + 1;

        if nextIndex <= population
            candidateKnots(:, :, nextIndex) = ...
                clampMatrixByRows(negativeSample, -maxQStep, maxQStep);
            nextIndex = nextIndex + 1;
        end
    end

    while nextIndex <= population
        noise = temporallyCorrelatedNoise( ...
            3, numberOfKnots, temporalCorrelation);
        candidateKnots(:, :, nextIndex) = clampMatrixByRows( ...
            meanKnots + stdKnots .* noise, -maxQStep, maxQStep);
        nextIndex = nextIndex + 1;
    end
end

function noise = temporallyCorrelatedNoise(rows, columns, correlation)

    correlation = min(0.999, max(0, correlation));
    noise = zeros(rows, columns);
    noise(:, 1) = randn(rows, 1);
    innovationScale = sqrt(max(0, 1 - correlation^2));

    for column = 2:columns
        noise(:, column) = correlation * noise(:, column - 1) + ...
            innovationScale * randn(rows, 1);
    end
end

% =========================================================================
% Candidate evaluation
% =========================================================================
function result = evaluateCandidate( ...
    deltaPhi, x0, qRefHorizon, q_des_prev, Np, Nc, params)

    result = emptyCandidateResult(Nc);
    result.deltaPhi = deltaPhi;

    if ~isequal(size(deltaPhi), [3, Nc]) || ...
            any(~isfinite(deltaPhi(:)))
        result.rejectionCode = 1;
        return;
    end

    qMin = params.mpc.qMin(:);
    qMax = params.mpc.qMax(:);
    thetaMin = params.mpc.thetaMin(:);
    thetaMax = params.mpc.thetaMax(:);
    omegaMax = expandToVector(params.mpc.omegaMax, 4);
    alphaMax = expandToVector(params.mpc.alphaMax, 4);
    dangerDistance = params.singularity.dangerDistance;
    warningDistance = params.singularity.warningDistance;
    tolerance = readMpcSetting(params, 'constraintTolerance', 1e-6);
    transitionSamples = max(1, round(readMpcSetting( ...
        params, 'cemTransitionSamples', 3)));

    wTrack = params.mpc.wTrack;
    wTerminal = params.mpc.wTerminal;
    wSmooth = params.mpc.wSmooth;
    wMotor = params.mpc.wMotor;
    wSingularity = params.mpc.wSingularity;
    wOmega = params.mpc.wOmega;

    commandRotation = qToRotmXYZ(q_des_prev);
    qCommandPrevious = q_des_prev;
    xPred = x0;
    thetaPreviousPred = x0(1:4);
    minimumPoleDistance = inf;
    firstTransitionPoleDistance = NaN;
    totalCost = 0;

    try
        thetaCommandPrevious = abenicsIK(q_des_prev, params);
        thetaCommandPrevious = thetaCommandPrevious(:);
        thetaCommandPrevious = unwrapThetaNearest( ...
            thetaCommandPrevious, thetaPreviousPred, params);
        if ~thetaWithinPositionLimits(thetaCommandPrevious, params, tolerance)
            result.rejectionCode = 4;
            return;
        end
    catch
        result.rejectionCode = 3;
        return;
    end

    try
        for step = 1:Np
            if step <= Nc
                commandIncrement = deltaPhi(:, step);
                commandRotation = commandRotation * ...
                    rotvecToRotm(commandIncrement);
            else
                commandIncrement = zeros(3, 1);
            end

            qCommand = rotmToQXYZContinuous( ...
                commandRotation, qCommandPrevious);

            if any(qCommand > qMax + tolerance) || ...
                    any(qCommand < qMin - tolerance)
                result.rejectionCode = 2;
                return;
            end

            thetaCommand = abenicsIK(qCommand, params);
            thetaCommand = thetaCommand(:);
            if ~isequal(size(thetaCommand), [4, 1]) || ...
                    any(~isfinite(thetaCommand))
                result.rejectionCode = 3;
                return;
            end
            thetaCommand = unwrapThetaNearest( ...
                thetaCommand, thetaCommandPrevious, params);
            if ~thetaWithinPositionLimits(thetaCommand, params, tolerance)
                result.rejectionCode = 4;
                return;
            end

            [xNext, alphaPred] = ...
                abenicsPlantStepLocal(xPred, thetaCommand, params);
            thetaPred = xNext(1:4);
            omegaPred = xNext(5:8);

            if any(~isfinite(xNext)) || any(~isfinite(alphaPred)) || ...
                    ~thetaWithinPositionLimits(thetaPred, params, tolerance) || ...
                    any(abs(omegaPred) > omegaMax + tolerance) || ...
                    any(abs(alphaPred) > alphaMax + tolerance)
                result.rejectionCode = 5;
                return;
            end

            [transitionSafe, transitionMinimum] = ...
                validateThetaTransitionPoleSafety( ...
                    thetaPreviousPred, thetaPred, dangerDistance, ...
                    transitionSamples, tolerance, params);
            if ~transitionSafe
                result.rejectionCode = 6;
                result.minimumPoleDistance = transitionMinimum;
                return;
            end
            if step == 1
                firstTransitionPoleDistance = transitionMinimum;
            end
            minimumPoleDistance = min( ...
                minimumPoleDistance, transitionMinimum);

            qPred = abenicsFK(thetaPred, params);
            qPred = qPred(:);
            predictedRotation = qToRotmXYZ(qPred);
            [sPred, poleDistances] = ...
                poleDistancesFromRotm(predictedRotation, params);
            if any(~isfinite(qPred)) || any(~isfinite(poleDistances)) || ...
                    numel(poleDistances) ~= 6 || ...
                    min(poleDistances) < dangerDistance - tolerance
                result.rejectionCode = 6;
                return;
            end

            referenceRotationStep = qToRotmXYZ( ...
                qRefHorizon(:, step));
            orientationError = rotmToRotvec( ...
                predictedRotation.' * referenceRotationStep);
            deltaThetaCommand = thetaContinuousDifference( ...
                thetaCommand, thetaCommandPrevious, params);
            warningDeficit = max(0, warningDistance - sPred);

            totalCost = totalCost + ...
                wTrack * sum(orientationError.^2) + ...
                wSmooth * sum(commandIncrement.^2) + ...
                wMotor * sum(deltaThetaCommand.^2) + ...
                wOmega * sum(omegaPred.^2) + ...
                wSingularity * warningDeficit^2;

            qCommandPrevious = qCommand;
            thetaCommandPrevious = thetaCommand;
            thetaPreviousPred = thetaPred;
            xPred = xNext;

            if step == Np
                terminalReferenceRotation = qToRotmXYZ( ...
                    qRefHorizon(:, end));
                terminalError = rotmToRotvec( ...
                    predictedRotation.' * terminalReferenceRotation);
                totalCost = totalCost + ...
                    wTerminal * sum(terminalError.^2);
                result.terminalOrientation = qPred;
                result.terminalPhysicalError = norm(terminalError);
            end
        end
    catch
        result.rejectionCode = 1;
        return;
    end

    if ~isfinite(totalCost)
        result.rejectionCode = 1;
        return;
    end

    result.valid = true;
    result.cost = totalCost;
    result.minimumPoleDistance = minimumPoleDistance;
    result.firstTransitionPoleDistance = firstTransitionPoleDistance;
    result.rejectionCode = 0;
end

function result = emptyCandidateResult(Nc)

    result.valid = false;
    result.cost = inf;
    result.deltaPhi = zeros(3, Nc);
    result.minimumPoleDistance = NaN;
    result.firstTransitionPoleDistance = NaN;
    result.terminalOrientation = NaN(3, 1);
    result.terminalPhysicalError = NaN;
    result.rejectionCode = 1;
end

% =========================================================================
% Independent first-command validation and safe fallback
% =========================================================================
function [valid, result] = validateOneStepCommand( ...
    qCandidate, q_des_prev, x0, params)

    result.minimumTransitionPoleDistance = NaN;
    result.nextMinimumPoleDistance = NaN;
    result.nextOrientation = NaN(3, 1);
    valid = false;

    maxRotationStep = expandToVector(params.mpc.maxQStep, 3);
    qMin = params.mpc.qMin(:);
    qMax = params.mpc.qMax(:);
    thetaMin = params.mpc.thetaMin(:);
    thetaMax = params.mpc.thetaMax(:);
    omegaMax = expandToVector(params.mpc.omegaMax, 4);
    alphaMax = expandToVector(params.mpc.alphaMax, 4);
    dangerDistance = params.singularity.dangerDistance;
    tolerance = readMpcSetting(params, 'constraintTolerance', 1e-6);
    transitionSamples = max(2, round(readMpcSetting( ...
        params, 'transitionSafetySamples', 9)));

    previousRotation = qToRotmXYZ(q_des_prev);
    candidateRotation = qToRotmXYZ(qCandidate);
    physicalStep = rotmToRotvec(previousRotation.' * candidateRotation);

    if any(~isfinite(qCandidate)) || ...
            any(qCandidate > qMax + tolerance) || ...
            any(qCandidate < qMin - tolerance) || ...
            any(abs(physicalStep) > maxRotationStep + tolerance)
        return;
    end

    try
        thetaCommand = abenicsIK(qCandidate, params);
        thetaCommand = thetaCommand(:);
        thetaCommand = unwrapThetaNearest(thetaCommand, x0(1:4), params);
        if ~thetaWithinPositionLimits(thetaCommand, params, tolerance)
            return;
        end

        [xNext, alphaNext] = ...
            abenicsPlantStepLocal(x0, thetaCommand, params);
        thetaNext = xNext(1:4);
        omegaNext = xNext(5:8);
        if ~thetaWithinPositionLimits(thetaNext, params, tolerance) || ...
                any(abs(omegaNext) > omegaMax + tolerance) || ...
                any(abs(alphaNext) > alphaMax + tolerance)
            return;
        end

        [transitionSafe, transitionMinimum] = ...
            validateThetaTransitionPoleSafety( ...
                x0(1:4), thetaNext, dangerDistance, ...
                transitionSamples, tolerance, params);
        if ~transitionSafe
            return;
        end

        qNext = abenicsFK(thetaNext, params);
        qNext = qNext(:);
        [sNext, ~] = poleDistancesFromQ(qNext, params);

        result.minimumTransitionPoleDistance = transitionMinimum;
        result.nextMinimumPoleDistance = sNext;
        result.nextOrientation = qNext;
        valid = true;
    catch
        valid = false;
    end
end

function [qFallback, valid, info] = chooseSafeFallbackCommand( ...
    q_ref, q_des_prev, x0, currentMinimumPole, currentPoleDistances, params)

    maxRotationStep = expandToVector(params.mpc.maxQStep, 3);
    dangerDistance = params.singularity.dangerDistance;
    warningDistance = params.singularity.warningDistance;
    tolerance = readMpcSetting(params, 'constraintTolerance', 1e-6);
    previousRotation = qToRotmXYZ(q_des_prev);
    referenceRotation = qToRotmXYZ(q_ref);

    directions = zeros(3, 27);
    directionIndex = 1;
    for axisOne = -1:1
        for axisTwo = -1:1
            for axisThree = -1:1
                directions(:, directionIndex) = ...
                    [axisOne; axisTwo; axisThree];
                directionIndex = directionIndex + 1;
            end
        end
    end

    directVector = rotmToRotvec(previousRotation.' * referenceRotation);
    directVector = clampVector( ...
        directVector, -maxRotationStep, maxRotationStep);

    fractions = [0.25, 0.5, 1.0];
    candidateVectors = zeros(3, 2 + 26 * numel(fractions));
    candidateVectors(:, 1) = zeros(3, 1);
    candidateVectors(:, 2) = directVector;
    candidateCount = 2;

    for fraction = fractions
        for directionIndex = 1:27
            direction = directions(:, directionIndex);
            if all(direction == 0)
                continue;
            end
            candidateCount = candidateCount + 1;
            candidateVectors(:, candidateCount) = ...
                fraction * direction .* maxRotationStep;
        end
    end

    valid = false;
    qFallback = q_des_prev;
    info.recoveryMode = false;
    info.reason = "validatedHold";
    bestScore = inf;
    bestClearance = -inf;

    insideDanger = currentMinimumPole < dangerDistance - tolerance;
    [~, closestPoleIndex] = min(currentPoleDistances);

    for candidateIndex = 1:candidateCount
        candidateRotation = previousRotation * ...
            rotvecToRotm(candidateVectors(:, candidateIndex));
        qCandidate = rotmToQXYZContinuous( ...
            candidateRotation, q_des_prev);

        if insideDanger
            [candidateValid, candidateResult] = ...
                validateOutwardRecoveryCommand( ...
                    qCandidate, q_des_prev, x0, closestPoleIndex, ...
                    currentPoleDistances, params);
            if ~candidateValid
                continue;
            end

            score = -candidateResult.selectedNextPoleDistance;
            recoveryMode = true;
            reason = "validatedOutwardRecovery";
        else
            [candidateValid, candidateResult] = ...
                validateOneStepCommand(qCandidate, q_des_prev, x0, params);
            if ~candidateValid
                continue;
            end

            nextRotation = qToRotmXYZ(candidateResult.nextOrientation);
            physicalError = rotmToRotvec( ...
                nextRotation.' * referenceRotation);
            clearanceDeficit = max(0, warningDistance - ...
                candidateResult.nextMinimumPoleDistance);
            score = sum(physicalError.^2) + ...
                0.1 * sum(candidateVectors(:, candidateIndex).^2) + ...
                10 * clearanceDeficit^2;
            recoveryMode = false;
            reason = "validatedSafeFallback";
        end

        clearance = candidateResult.minimumTransitionPoleDistance;
        if score < bestScore - 1e-12 || ...
          (abs(score - bestScore) <= 1e-12 && clearance > bestClearance)
            bestScore = score;
            bestClearance = clearance;
            qFallback = qCandidate;
            valid = true;
            info.recoveryMode = recoveryMode;
            info.reason = reason;
        end
    end
end

function [valid, result] = validateOutwardRecoveryCommand( ...
    qCandidate, q_des_prev, x0, closestPoleIndex, ...
    currentPoleDistances, params)

    valid = false;
    result.minimumTransitionPoleDistance = NaN;
    result.selectedNextPoleDistance = NaN;
    result.nextOrientation = NaN(3, 1);

    maxRotationStep = expandToVector(params.mpc.maxQStep, 3);
    qMin = params.mpc.qMin(:);
    qMax = params.mpc.qMax(:);
    thetaMin = params.mpc.thetaMin(:);
    thetaMax = params.mpc.thetaMax(:);
    omegaMax = expandToVector(params.mpc.omegaMax, 4);
    alphaMax = expandToVector(params.mpc.alphaMax, 4);
    dangerDistance = params.singularity.dangerDistance;
    tolerance = readMpcSetting(params, 'constraintTolerance', 1e-6);
    samples = max(2, round(readMpcSetting( ...
        params, 'transitionSafetySamples', 9)));

    previousRotation = qToRotmXYZ(q_des_prev);
    candidateRotation = qToRotmXYZ(qCandidate);
    physicalStep = rotmToRotvec(previousRotation.' * candidateRotation);
    if any(~isfinite(qCandidate)) || ...
            any(qCandidate > qMax + tolerance) || ...
            any(qCandidate < qMin - tolerance) || ...
            any(abs(physicalStep) > maxRotationStep + tolerance)
        return;
    end

    try
        thetaCommand = abenicsIK(qCandidate, params);
        thetaCommand = thetaCommand(:);
        thetaCommand = unwrapThetaNearest(thetaCommand, x0(1:4), params);
        if ~thetaWithinPositionLimits(thetaCommand, params, tolerance)
            return;
        end

        [xNext, alphaNext] = ...
            abenicsPlantStepLocal(x0, thetaCommand, params);
        thetaNext = xNext(1:4);
        omegaNext = xNext(5:8);
        if ~thetaWithinPositionLimits(thetaNext, params, tolerance) || ...
                any(abs(omegaNext) > omegaMax + tolerance) || ...
                any(abs(alphaNext) > alphaMax + tolerance)
            return;
        end

        thetaDifference = thetaContinuousDifference(thetaNext, x0(1:4), params);
        selectedDistances = zeros(samples, 1);
        otherMinimum = inf;
        transitionMinimum = inf;

        for sampleIndex = 1:samples
            lambda = sampleIndex / samples;
            thetaSample = x0(1:4) + lambda * thetaDifference;
            qSample = abenicsFK(thetaSample, params);
            [~, distances] = poleDistancesFromQ(qSample(:), params);
            if numel(distances) ~= 6 || any(~isfinite(distances))
                return;
            end

            selectedDistances(sampleIndex) = ...
                distances(closestPoleIndex);
            otherDistances = distances;
            otherDistances(closestPoleIndex) = inf;
            otherMinimum = min(otherMinimum, min(otherDistances));
            transitionMinimum = min(transitionMinimum, min(distances));
        end

        if any(diff([currentPoleDistances(closestPoleIndex); ...
                selectedDistances]) < -tolerance)
            return;
        end
        if selectedDistances(end) <= ...
                currentPoleDistances(closestPoleIndex) + tolerance
            return;
        end
        if otherMinimum < dangerDistance - tolerance
            return;
        end

        qNext = abenicsFK(thetaNext, params);
        result.minimumTransitionPoleDistance = transitionMinimum;
        result.selectedNextPoleDistance = selectedDistances(end);
        result.nextOrientation = qNext(:);
        valid = true;
    catch
        valid = false;
    end
end

% =========================================================================
% Transition safety
% =========================================================================
function [safe, minimumDistance] = validateThetaTransitionPoleSafety( ...
    thetaStart, thetaEnd, requiredDistance, samples, tolerance, params)

    safe = false;
    minimumDistance = inf;

    if ~isequal(size(thetaStart), [4, 1]) || ...
            ~isequal(size(thetaEnd), [4, 1]) || ...
            any(~isfinite(thetaStart)) || any(~isfinite(thetaEnd))
        minimumDistance = NaN;
        return;
    end

    thetaDifference = thetaContinuousDifference(thetaEnd, thetaStart, params);

    try
        for sampleIndex = 1:samples
            lambda = sampleIndex / samples;
            thetaSample = thetaStart + lambda * thetaDifference;
            qSample = abenicsFK(thetaSample, params);
            [~, distances] = poleDistancesFromQ(qSample(:), params);
            if numel(distances) ~= 6 || any(~isfinite(distances))
                minimumDistance = NaN;
                return;
            end
            minimumDistance = min(minimumDistance, min(distances));
            if minimumDistance < requiredDistance - tolerance
                return;
            end
        end
    catch
        minimumDistance = NaN;
        return;
    end

    safe = isfinite(minimumDistance) && ...
        minimumDistance >= requiredDistance - tolerance;
end

% =========================================================================
% Direct six-pole distance calculation
% =========================================================================
function [minimumDistance, distances] = poleDistancesFromQ(q, params)

    q = q(:);
    if ~isequal(size(q), [3, 1]) || any(~isfinite(q))
        minimumDistance = NaN;
        distances = NaN(6, 1);
        return;
    end

    [minimumDistance, distances] = ...
        poleDistancesFromRotm(qToRotmXYZ(q), params);
end

function [minimumDistance, distances] = ...
    poleDistancesFromRotm(rotation, params)

    if ~isequal(size(rotation), [3, 3]) || ...
            any(~isfinite(rotation(:)))
        minimumDistance = NaN;
        distances = NaN(6, 1);
        return;
    end

    bodyAxis = params.singularity.trackedBodyAxis(:);
    bodyAxis = bodyAxis / norm(bodyAxis);
    trackedAxis = rotation * bodyAxis;
    trackedAxis = trackedAxis / norm(trackedAxis);

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

function q = rotmToQXYZContinuous(rotation, qReference)

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
        combined = nearestEquivalentAngle( ...
            combined, qReference(1) + qReference(3));
        rollValue = 0.5 * ...
            (combined + qReference(1) - qReference(3));
        yawValue = combined - rollValue;
        candidates = [rollValue; pi/2; yawValue];
    else
        difference = atan2(rotation(2, 1), rotation(2, 2));
        difference = nearestEquivalentAngle( ...
            difference, qReference(3) - qReference(1));
        rollValue = 0.5 * ...
            (qReference(1) + qReference(3) - difference);
        yawValue = rollValue + difference;
        candidates = [rollValue; -pi/2; yawValue];
    end

    if size(candidates, 2) == 1
        q = candidates;
        q(1) = nearestEquivalentAngle(q(1), qReference(1));
        q(2) = nearestEquivalentAngle(q(2), qReference(2));
        q(3) = nearestEquivalentAngle(q(3), qReference(3));
        return;
    end

    bestScore = inf;
    q = candidates(:, 1);
    for candidateIndex = 1:size(candidates, 2)
        candidate = candidates(:, candidateIndex);
        for axisIndex = 1:3
            candidate(axisIndex) = nearestEquivalentAngle( ...
                candidate(axisIndex), qReference(axisIndex));
        end
        score = sum((candidate - qReference).^2);
        if score < bestScore
            bestScore = score;
            q = candidate;
        end
    end
end

function adjusted = nearestEquivalentAngle(angleValue, referenceValue)
    adjusted = angleValue + 2*pi * round((referenceValue - angleValue)/(2*pi));
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

function distance = rotationDistanceFromQ(qOne, qTwo)
    rotationOne = qToRotmXYZ(qOne);
    rotationTwo = qToRotmXYZ(qTwo);
    distance = norm(rotmToRotvec(rotationOne.' * rotationTwo));
end

% =========================================================================
% Knot and warm-start helpers
% =========================================================================
function deltaPhi = buildWarmStartDeltaPhi( ...
    previousBestDeltaPhi, q_ref, q_des_prev, maxRotationStep, Nc, reset)

    if ~reset && isequal(size(previousBestDeltaPhi), [3, Nc]) && ...
            all(isfinite(previousBestDeltaPhi(:)))
        deltaPhi = shiftDeltaPhi(previousBestDeltaPhi);
    else
        deltaPhi = buildDirectDeltaPhi( ...
            q_ref, q_des_prev, maxRotationStep, Nc);
    end
end

function deltaPhi = buildDirectDeltaPhi( ...
    q_ref, q_des_prev, maxRotationStep, Nc)

    previousRotation = qToRotmXYZ(q_des_prev);
    referenceRotation = qToRotmXYZ(q_ref);
    totalRotationVector = rotmToRotvec( ...
        previousRotation.' * referenceRotation);
    baseIncrement = clampVector( ...
        totalRotationVector / Nc, -maxRotationStep, maxRotationStep);
    deltaPhi = repmat(baseIncrement, 1, Nc);
end

function shifted = shiftDeltaPhi(deltaPhi)
    shifted = [deltaPhi(:, 2:end), deltaPhi(:, end)];
end

function knots = compressDeltaPhiToKnots(deltaPhi, numberOfKnots)
    Nc = size(deltaPhi, 2);
    knotLocations = round(linspace(1, Nc, numberOfKnots));
    knots = deltaPhi(:, knotLocations);
end

function deltaPhi = expandKnotsToDeltaPhi(knots, Nc, maxRotationStep)

    numberOfKnots = size(knots, 2);
    knotLocations = linspace(1, Nc, numberOfKnots);
    deltaPhi = zeros(3, Nc);

    for step = 1:Nc
        if step <= knotLocations(1)
            deltaPhi(:, step) = knots(:, 1);
            continue;
        end
        if step >= knotLocations(end)
            deltaPhi(:, step) = knots(:, end);
            continue;
        end

        leftIndex = find(knotLocations <= step, 1, 'last');
        rightIndex = leftIndex + 1;
        leftLocation = knotLocations(leftIndex);
        rightLocation = knotLocations(rightIndex);
        fraction = (step - leftLocation) / ...
            max(eps, rightLocation - leftLocation);
        deltaPhi(:, step) = (1 - fraction) * knots(:, leftIndex) + ...
            fraction * knots(:, rightIndex);
    end

    deltaPhi = clampMatrixByRows( ...
        deltaPhi, -maxRotationStep, maxRotationStep);
end

% =========================================================================
% Internal second-order plant prediction
% =========================================================================
function [xNext, alpha] = abenicsPlantStepLocal(x, thetaCommand, params)

    Ts = params.Ts;
    theta = x(1:4);
    omega = x(5:8);
    KpPlant = expandToVector(params.plant.KpPlant, 4);
    KdPlant = expandToVector(params.plant.KdPlant, 4);

    thetaError = thetaContinuousDifference(thetaCommand, theta, params);
    alpha = KpPlant .* thetaError - KdPlant .* omega;
    omegaNext = omega + Ts * alpha;
    thetaNext = theta + Ts * omegaNext;
    xNext = [thetaNext; omegaNext];
end

% =========================================================================
% Continuous periodic motor-angle handling
% =========================================================================
function thetaContinuous = unwrapThetaNearest(thetaRaw, thetaReference, params)

    thetaRaw = thetaRaw(:);
    thetaReference = thetaReference(:);
    thetaContinuous = thetaRaw;

    if ~logical(readMpcSetting(params, 'thetaUnwrapEnabled', true))
        return;
    end

    periodicMask = readMpcSetting(params, 'thetaPeriodic', true(4, 1));
    periodicMask = logical(expandToVector(periodicMask, 4));

    for motorIndex = 1:4
        if periodicMask(motorIndex)
            thetaContinuous(motorIndex) = thetaRaw(motorIndex) + ...
                2*pi * round((thetaReference(motorIndex) - ...
                thetaRaw(motorIndex)) / (2*pi));
        end
    end
end

function difference = thetaContinuousDifference(thetaA, thetaB, params)

    if logical(readMpcSetting(params, 'thetaUnwrapEnabled', true))
        difference = thetaA - thetaB;
    else
        difference = wrapAngleDifference(thetaA, thetaB);
    end
end

function inside = thetaWithinPositionLimits(theta, params, tolerance)

    enforceLimits = logical(readMpcSetting( ...
        params, 'enforceThetaPositionLimits', true));
    if ~enforceLimits
        inside = true;
        return;
    end

    thetaMin = params.mpc.thetaMin(:);
    thetaMax = params.mpc.thetaMax(:);
    inside = all(theta <= thetaMax + tolerance) && ...
        all(theta >= thetaMin - tolerance);
end

% =========================================================================
% Diagnostics
% =========================================================================
function publishDiagnostics(diagnostics, params)

    enabled = logical(readMpcSetting( ...
        params, 'enableTestDiagnostics', false));
    if ~enabled
        return;
    end

    persistent ABENICS_CEM_LAST_DIAGNOSTICS
    ABENICS_CEM_LAST_DIAGNOSTICS = diagnostics;
end

% =========================================================================
% Reference-preview helper
% =========================================================================
function qRefHorizon = buildReferenceHorizon(q_ref, Np, params)
%BUILDREFERENCEHORIZON Return a finite 3xNp orientation reference sequence.
%
% When preview is disabled, the current q_ref is repeated.
% When preview is enabled, params.mpc.qRefHorizon may contain:
%   - 3x1: repeated across the horizon
%   - 3xM, M < Np: padded using its final column
%   - 3xM, M >= Np: first Np columns are used

    qRefHorizon = repmat(q_ref(:), 1, Np);

    usePreview = false;
    if isfield(params, 'mpc') && ...
            isfield(params.mpc, 'useReferencePreview')
        usePreview = logical(params.mpc.useReferencePreview);
    end

    if ~usePreview
        return;
    end

    if ~isfield(params.mpc, 'qRefHorizon')
        error('abenicsOrientationMPC:MissingReferencePreview', ...
            ['params.mpc.useReferencePreview is true, but ', ...
             'params.mpc.qRefHorizon is missing.']);
    end

    preview = params.mpc.qRefHorizon;
    if ~isnumeric(preview) || size(preview, 1) ~= 3 || ...
            isempty(preview) || any(~isfinite(preview(:)))
        error('abenicsOrientationMPC:BadReferencePreview', ...
            'params.mpc.qRefHorizon must be a finite numeric 3xM matrix.');
    end

    numberOfPreviewColumns = size(preview, 2);
    if numberOfPreviewColumns >= Np
        qRefHorizon = preview(:, 1:Np);
    else
        qRefHorizon(:, 1:numberOfPreviewColumns) = preview;
        qRefHorizon(:, numberOfPreviewColumns + 1:end) = ...
            repmat(preview(:, end), 1, Np - numberOfPreviewColumns);
    end
end

% =========================================================================
% General utilities
% =========================================================================
function validateInputs(q_ref, theta_actual, q_des_prev, params)

    if ~isequal(size(q_ref), [3, 1])
        error('abenicsOrientationMPC:qRefSize', ...
            'q_ref must be 3x1 [roll; pitch; yaw].');
    end
    if ~isequal(size(theta_actual), [4, 1])
        error('abenicsOrientationMPC:thetaActualSize', ...
            'theta_actual must be 4x1.');
    end
    if ~isequal(size(q_des_prev), [3, 1])
        error('abenicsOrientationMPC:qDesPrevSize', ...
            'q_des_prev must be 3x1 [roll; pitch; yaw].');
    end
    if any(~isfinite(q_ref)) || any(~isfinite(theta_actual)) || ...
            any(~isfinite(q_des_prev))
        error('abenicsOrientationMPC:nonfiniteInput', ...
            'Controller inputs must be finite.');
    end
    requiredFields = {'Ts', 'mpc', 'plant', 'singularity'};
    for index = 1:numel(requiredFields)
        if ~isfield(params, requiredFields{index})
            error('abenicsOrientationMPC:missingParameter', ...
                'params.%s is required.', requiredFields{index});
        end
    end
end

function difference = wrapAngleDifference(a, b)

    difference = atan2(sin(a - b), cos(a - b));
end

function output = clampVector(input, minimum, maximum)

    output = min(max(input, minimum), maximum);
end

function output = clampMatrixByRows(input, minimum, maximum)

    minimum = minimum(:);
    maximum = maximum(:);
    output = input;
    for row = 1:size(input, 1)
        output(row, :) = min(max(input(row, :), minimum(row)), maximum(row));
    end
end

function vector = expandToVector(value, numberOfElements)

    value = value(:);
    if numel(value) == 1
        vector = value * ones(numberOfElements, 1);
    elseif numel(value) == numberOfElements
        vector = value;
    else
        error('abenicsOrientationMPC:parameterSize', ...
            'Expected a scalar or %dx1 parameter.', numberOfElements);
    end
end

function value = readMpcSetting(params, fieldName, defaultValue)

    if isfield(params, 'mpc') && isfield(params.mpc, fieldName)
        value = params.mpc.(fieldName);
    else
        value = defaultValue;
    end
end
