function q_des_mpc = abenicsOrientationMPC( ...
    q_ref, theta_actual, q_des_prev, params)
%ABENICSORIENTATIONMPC V5 clearance-aware orientation-command MPC wrapper.
%
% Public output remains only:
%   q_des_mpc = [roll; pitch; yaw]

    persistent theta_prev_for_omega
    persistent recoveryActive
    persistent previousDeltaQ
    persistent singularTargetWarningShown

    % Geometry-aware detour commitment state.
    persistent detourActive
    persistent committedPoleIndex
    persistent committedDetourSide
    persistent committedClearance
    persistent committedReference
    persistent detourFailureCount
    persistent detourClearCounter

    controllerState.theta_prev_for_omega = theta_prev_for_omega;
    controllerState.recoveryActive = recoveryActive;
    controllerState.previousDeltaQ = previousDeltaQ;
    controllerState.detourActive = detourActive;
    controllerState.committedPoleIndex = committedPoleIndex;
    controllerState.committedDetourSide = committedDetourSide;
    controllerState.committedClearance = committedClearance;
    controllerState.committedReference = committedReference;
    controllerState.detourFailureCount = detourFailureCount;
    controllerState.detourClearCounter = detourClearCounter;

    [q_des_mpc, diagnostics, controllerState] = ...
        abenicsOrientationMPCCore( ...
            q_ref, theta_actual, q_des_prev, params, controllerState);

    % Initialize the warning memory on the first controller call.
    if isempty(singularTargetWarningShown)
        singularTargetWarningShown = false;
    end

    emitWarning = true;
    if isfield(params.mpc, 'emitSingularTargetWarning')
        emitWarning = logical(params.mpc.emitSingularTargetWarning);
    end

    if diagnostics.singularTargetOverrideActive
        if emitWarning && ~singularTargetWarningShown
            warning( ...
                'abenicsOrientationMPC:SingularTargetOverride', ...
                ['Requested target is %.4f deg from a singular pole. ', ...
                 'Singular-target override is active, so normal ', ...
                 'singularity protection is being relaxed.'], ...
                rad2deg(diagnostics.targetSingularityDistance));
        end
        singularTargetWarningShown = true;
    else
        singularTargetWarningShown = false;
    end

    % Diagnostic printing must not control persistent-state writeback.
    if isfield(params.mpc, 'debug') && params.mpc.debug
        fprintf( ...
            ['exit=%g | accepted=%d | fallback=%d | reason=%s | ', ...
             'solutionFinite=%d | objectiveFinite=%d | rolloutFinite=%d | ', ...
             'constraints=%d | firstCommand=%d | ', ...
             'constraint=%s | step=%g | iterations=%d | ', ...
             'evaluations=%d | violation=%.3e | blocked=%d | pole=%g | ', ...
             'detourActive=%d | side=%g | starts=%d | solves=%d | ', ...
             'winner=%g | solveTime=%.6f\n'], ...
            diagnostics.exitflag, ...
            diagnostics.solutionAccepted, ...
            diagnostics.fallbackUsed, ...
            char(diagnostics.fallbackReason), ...
            diagnostics.solutionFinite, ...
            diagnostics.objectiveFinite, ...
            diagnostics.rolloutFinite, ...
            diagnostics.constraintsSatisfied, ...
            diagnostics.firstCommandValid, ...
            char(diagnostics.maxConstraintName), ...
            diagnostics.maxConstraintStep, ...
            diagnostics.iterations, ...
            diagnostics.functionEvaluations, ...
            diagnostics.maxConstraintViolation, ...
            diagnostics.routeBlocked, ...
            diagnostics.blockingPoleIndex, ...
            diagnostics.detourActive, ...
            diagnostics.committedDetourSide, ...
            diagnostics.numberOfStarts, ...
            diagnostics.numberOfFminconSolves, ...
            diagnostics.winningStartIndex, ...
            diagnostics.solveTime);

        % Numeric-only update record for deterministic MATLAB test parsing.
        fprintf( ...
            ['MPCUPDATE version=5 blocked=%d pole=%g directBlocked=%d ', ...
             'detourActive=%d committedSide=%g starts=%d solves=%d ', ...
             'winnerIndex=%g winnerType=%g winnerSide=%g ', ...
             'winnerRouteEligible=%d winnerProgress=%.9g ', ...
             'winnerProgressMode=%g winnerFirstProgress=%.9g ', ...
             'winnerFirstPole=%.9g winnerNearPole=%.9g ', ...
             'winnerMinimumNearPole=%.9g ', ...
             'winningClearance=%.9g accepted=%d fallback=%d recovery=%d ', ...
             'totalSolveTime=%.9g worstStartSolveTime=%.9g ', ...
             'minimumNominalPole=%.9g minimumDirectPole=%.9g\n'], ...
            diagnostics.routeBlocked, ...
            diagnostics.blockingPoleIndex, ...
            diagnostics.directRouteBlocked, ...
            diagnostics.detourActive, ...
            diagnostics.committedDetourSide, ...
            diagnostics.numberOfStarts, ...
            diagnostics.numberOfFminconSolves, ...
            diagnostics.winningStartIndex, ...
            diagnostics.winningStartType, ...
            diagnostics.winningDetourSide, ...
            diagnostics.winningRouteEligible, ...
            diagnostics.winningProgress, ...
            diagnostics.winningProgressMode, ...
            diagnostics.winningFirstProgress, ...
            diagnostics.winningFirstPoleDistance, ...
            diagnostics.winningNearPoleDistance, ...
            diagnostics.winningMinimumNearPoleDistance, ...
            diagnostics.winningClearance, ...
            diagnostics.solutionAccepted, ...
            diagnostics.fallbackUsed, ...
            diagnostics.recoveryUsed, ...
            diagnostics.totalSolveTime, ...
            diagnostics.worstStartSolveTime, ...
            diagnostics.minimumNominalPoleDistance, ...
            diagnostics.minimumDirectPoleDistance);

        for startIndex = 1:diagnostics.numberOfStarts
            fprintf( ...
                ['MPCSTART index=%d | type=%g | pole=%g | side=%g | ', ...
                 'clearance=%.9g | seedValid=%d | seedBlend=%.9g | ', ...
                 'seedViolation=%.9g | exit=%g | accepted=%d | ', ...
                 'routeEligible=%d | progressMode=%g | ', ...
                 'progressSide=%g | firstProgress=%.9g | progress=%.9g | ', ...
                 'firstPole=%.9g | nearPole=%.9g | minNearPole=%.9g | ', ...
                 'objective=%.12g | minPole=%.9g | iterations=%d | ', ...
                 'evaluations=%d | solveTime=%.6f | reject=%g\n'], ...
                startIndex, ...
                diagnostics.startType(startIndex), ...
                diagnostics.startPoleIndex(startIndex), ...
                diagnostics.startSide(startIndex), ...
                diagnostics.startClearance(startIndex), ...
                diagnostics.startSeedValid(startIndex), ...
                diagnostics.startSeedBlendScale(startIndex), ...
                diagnostics.startSeedConstraintViolation(startIndex), ...
                diagnostics.startExitflag(startIndex), ...
                diagnostics.startAccepted(startIndex), ...
                diagnostics.startRouteEligible(startIndex), ...
                diagnostics.startProgressMode(startIndex), ...
                diagnostics.startProgressSide(startIndex), ...
                diagnostics.startFirstProgress(startIndex), ...
                diagnostics.startProgress(startIndex), ...
                diagnostics.startFirstPoleDistance(startIndex), ...
                diagnostics.startNearPoleDistance(startIndex), ...
                diagnostics.startMinimumNearPoleDistance(startIndex), ...
                diagnostics.startObjective(startIndex), ...
                diagnostics.startMinimumPoleDistance(startIndex), ...
                diagnostics.startIterations(startIndex), ...
                diagnostics.startFunctionEvaluations(startIndex), ...
                diagnostics.startSolveTime(startIndex), ...
                diagnostics.startRejectionCode(startIndex));
        end

        if diagnostics.fallbackUsed && diagnostics.usedFmincon
            fprintf('Solver message: %s\n', char(diagnostics.message));
        end
    end

    % Always write back state, regardless of debug settings.
    theta_prev_for_omega = controllerState.theta_prev_for_omega;
    recoveryActive = controllerState.recoveryActive;
    previousDeltaQ = controllerState.previousDeltaQ;
    detourActive = controllerState.detourActive;
    committedPoleIndex = controllerState.committedPoleIndex;
    committedDetourSide = controllerState.committedDetourSide;
    committedClearance = controllerState.committedClearance;
    committedReference = controllerState.committedReference;
    detourFailureCount = controllerState.detourFailureCount;
    detourClearCounter = controllerState.detourClearCounter;
end


function [q_des_mpc, diagnostics, stateOut] = ...
    abenicsOrientationMPCCore( ...
        q_ref, theta_actual, q_des_prev, params, stateIn)
%ABENICSORIENTATIONMPCCORE V5 continuous nonlinear MPC core for ABENICS.
%
% The MPC remains a direct-shooting fmincon controller. Geometry-aware
% paths are used only as initial guesses. Every accepted command comes from
% an independently validated optimized solution.

    if nargin < 5 || isempty(stateIn)
        stateIn = struct();
    end

    q_ref = q_ref(:);
    theta_actual = theta_actual(:);
    q_des_prev = q_des_prev(:);

    if ~isequal(size(q_ref), [3, 1])
        error('abenicsOrientationMPCCore:qRefSize', ...
              'q_ref must be 3x1: [roll; pitch; yaw].');
    end
    if ~isequal(size(theta_actual), [4, 1])
        error('abenicsOrientationMPCCore:thetaActualSize', ...
              ['theta_actual must be 4x1: ', ...
               '[theta_rA; theta_pA; theta_rB; theta_pB].']);
    end
    if ~isequal(size(q_des_prev), [3, 1])
        error('abenicsOrientationMPCCore:qDesPrevSize', ...
              'q_des_prev must be 3x1: [roll; pitch; yaw].');
    end

    Ts = params.Ts;
    Np = round(readMpcSetting(params, 'Np', 20));
    Nc = round(readMpcSetting(params, 'Nc', 5));

    if Np < 1 || Nc < 1 || Nc > Np
        error('abenicsOrientationMPCCore:horizon', ...
              'Require Np >= 1 and 1 <= Nc <= Np.');
    end

    params.mpc.Np = Np;
    params.mpc.Nc = Nc;

    qMin = params.mpc.qMin(:);
    qMax = params.mpc.qMax(:);
    thetaMin = params.mpc.thetaMin(:);
    thetaMax = params.mpc.thetaMax(:);
    omegaMax = expandToVector(params.mpc.omegaMax, 4);
    alphaMax = expandToVector(params.mpc.alphaMax, 4);
    maxQStep = expandToVector(params.mpc.maxQStep, 3);

    recoveryMaxQStep = expandToVector( ...
        readMpcSetting(params, 'recoveryMaxQStep', params.mpc.maxQStep), 3);
    recoveryMaxQStep = min(recoveryMaxQStep, maxQStep);

    dangerDistance = params.singularity.dangerDistance;
    warningDistance = params.singularity.warningDistance;
    recoveryClearDistance = readMpcSetting( ...
        params, 'recoveryClearDistance', deg2rad(3));
    constraintTolerance = readMpcSetting( ...
        params, 'constraintTolerance', 1e-6);

    enableDetourMultistart = logical(readMpcSetting( ...
        params, 'enableDetourMultistart', true));
    detourTriggerDistance = readMpcSetting( ...
        params, 'detourTriggerDistance', warningDistance);
    detourClearDistance = readMpcSetting( ...
        params, 'detourClearDistance', warningDistance);
    detourClearances = readMpcSetting( ...
        params, 'detourClearances', warningDistance);
    detourClearances = detourClearances(:);
    detourTargetTolerance = readMpcSetting( ...
        params, 'detourTargetTolerance', deg2rad(1));
    detourReferenceChangeTolerance = readMpcSetting( ...
        params, 'detourReferenceChangeTolerance', deg2rad(5));
    maxDetourStarts = max(1, round(readMpcSetting( ...
        params, 'maxDetourStarts', 3)));
    detourClearConfirmations = max(1, round(readMpcSetting( ...
        params, 'detourClearConfirmations', 3)));
    maxDetourFailures = max(1, round(readMpcSetting( ...
        params, 'maxDetourFailures', 2)));
    detourContinuationClearance = readMpcSetting( ...
        params, 'detourContinuationClearance', ...
        max(recoveryClearDistance + deg2rad(2), deg2rad(5)));
    detourMinimumOptimizedProgress = readMpcSetting( ...
        params, 'detourMinimumOptimizedProgress', deg2rad(0.5));

    % V5 near-term route execution settings. Progress is measured as actual
    % displacement of the tracked body axis in the pole tangent plane. Unlike
    % azimuth, this metric collapses toward zero near the pole and therefore
    % cannot reward an almost-singular crossing.
    detourProgressMinimumClearance = readMpcSetting( ...
        params, 'detourProgressMinimumClearance', ...
        detourContinuationClearance);
    detourNearTermCommandCount = max(1, round(readMpcSetting( ...
        params, 'detourNearTermCommandCount', 3)));
    detourFirstTangentialDisplacement = readMpcSetting( ...
        params, 'detourFirstTangentialDisplacement', deg2rad(0.02));
    detourNearTermTangentialDisplacement = readMpcSetting( ...
        params, 'detourNearTermTangentialDisplacement', deg2rad(0.08));
    detourMinimumOutwardProgress = readMpcSetting( ...
        params, 'detourMinimumOutwardProgress', deg2rad(0.25));
    transitionSafetySamples = max(2, round(readMpcSetting( ...
        params, 'transitionSafetySamples', 9)));

    params.mpc.detourProgressMinimumClearance = ...
        detourProgressMinimumClearance;
    params.mpc.detourNearTermCommandCount = ...
        min(detourNearTermCommandCount, Np);
    params.mpc.detourFirstTangentialDisplacement = ...
        detourFirstTangentialDisplacement;
    params.mpc.detourNearTermTangentialDisplacement = ...
        detourNearTermTangentialDisplacement;
    params.mpc.detourMinimumOutwardProgress = ...
        detourMinimumOutwardProgress;
    params.mpc.transitionSafetySamples = transitionSafetySamples;

    if isempty(detourClearances) || ...
            any(~isfinite(detourClearances)) || ...
            any(detourClearances <= dangerDistance)
        error('abenicsOrientationMPCCore:detourClearances', ...
              'Every detour clearance must be finite and exceed dangerDistance.');
    end
    if ~isscalar(detourContinuationClearance) || ...
            ~isfinite(detourContinuationClearance) || ...
            detourContinuationClearance <= dangerDistance
        error('abenicsOrientationMPCCore:detourContinuationClearance', ...
              ['detourContinuationClearance must be finite and exceed ', ...
               'dangerDistance.']);
    end
    if ~isscalar(detourMinimumOptimizedProgress) || ...
            ~isfinite(detourMinimumOptimizedProgress) || ...
            detourMinimumOptimizedProgress < 0
        error('abenicsOrientationMPCCore:detourMinimumOptimizedProgress', ...
              'detourMinimumOptimizedProgress must be a finite nonnegative scalar.');
    end
    if ~isscalar(detourProgressMinimumClearance) || ...
            ~isfinite(detourProgressMinimumClearance) || ...
            detourProgressMinimumClearance <= dangerDistance
        error('abenicsOrientationMPCCore:detourProgressMinimumClearance', ...
              ['detourProgressMinimumClearance must be finite and exceed ', ...
               'dangerDistance.']);
    end
    progressScalars = [ ...
        detourFirstTangentialDisplacement; ...
        detourNearTermTangentialDisplacement; ...
        detourMinimumOutwardProgress];
    if any(~isfinite(progressScalars)) || any(progressScalars < 0)
        error('abenicsOrientationMPCCore:detourProgressSettings', ...
              'V5 progress settings must be finite nonnegative scalars.');
    end
    if detourNearTermTangentialDisplacement < ...
            detourFirstTangentialDisplacement
        error('abenicsOrientationMPCCore:detourProgressOrdering', ...
              ['detourNearTermTangentialDisplacement must be at least ', ...
               'detourFirstTangentialDisplacement.']);
    end

    % The first implementation intentionally uses one clearance and at most
    % three starts: shifted, positive-side, and negative-side.
    initialDetourClearance = detourClearances(1);

    params.mpc.detourTriggerDistance = detourTriggerDistance;
    params.mpc.detourClearDistance = detourClearDistance;
    params.mpc.detourTargetTolerance = detourTargetTolerance;
    params.mpc.detourReferenceChangeTolerance = ...
        detourReferenceChangeTolerance;

    q_ref = clampVector(q_ref, qMin, qMax);
    q_des_prev = clampVector(q_des_prev, qMin, qMax);

    diagnostics = initializeDiagnostics(maxDetourStarts);
    diagnostics.Np = Np;
    diagnostics.Nc = Nc;
    diagnostics.recoveryClearDistance = recoveryClearDistance;

    % ------------------------------------------------------------------
    % Recover persistent state.
    % ------------------------------------------------------------------
    theta_prev_for_omega = readStateField( ...
        stateIn, 'theta_prev_for_omega', []);
    recoveryActive = logicalScalarState( ...
        stateIn, 'recoveryActive', false);

    previousDeltaQ = readStateField( ...
        stateIn, 'previousDeltaQ', zeros(3, Nc));
    if ~isequal(size(previousDeltaQ), [3, Nc]) || ...
            any(~isfinite(previousDeltaQ(:)))
        previousDeltaQ = zeros(3, Nc);
    end

    detourActive = logicalScalarState(stateIn, 'detourActive', false);
    committedPoleIndex = numericScalarState( ...
        stateIn, 'committedPoleIndex', 0);
    committedDetourSide = numericScalarState( ...
        stateIn, 'committedDetourSide', 0);
    committedClearance = numericScalarState( ...
        stateIn, 'committedClearance', initialDetourClearance);
    committedReference = readStateField( ...
        stateIn, 'committedReference', zeros(3, 1));
    if ~isequal(size(committedReference), [3, 1]) || ...
            any(~isfinite(committedReference))
        committedReference = zeros(3, 1);
    end
    detourFailureCount = max(0, round(numericScalarState( ...
        stateIn, 'detourFailureCount', 0)));
    detourClearCounter = max(0, round(numericScalarState( ...
        stateIn, 'detourClearCounter', 0)));

    if ~(committedDetourSide == -1 || ...
         committedDetourSide == 0 || ...
         committedDetourSide == 1)
        committedDetourSide = 0;
    end
    if committedPoleIndex < 1 || committedPoleIndex > 6
        if detourActive
            detourActive = false;
        end
        committedPoleIndex = 0;
    end

    % ------------------------------------------------------------------
    % Estimate current MP-gear velocity.
    % ------------------------------------------------------------------
    if isempty(theta_prev_for_omega) || ...
            ~isequal(size(theta_prev_for_omega), [4, 1]) || ...
            any(~isfinite(theta_prev_for_omega))
        theta_prev_for_omega = theta_actual;
    end

    omega_est = wrapAngleDifference( ...
        theta_actual, theta_prev_for_omega) / Ts;
    theta_prev_for_omega = theta_actual;
    omega_est = clampVector(omega_est, -omegaMax, omegaMax);
    x0 = [theta_actual; omega_est];

    % ------------------------------------------------------------------
    % Measured current orientation and physical pole distance.
    % ------------------------------------------------------------------
    currentStateValid = true;
    try
        q_current = abenicsFK(theta_actual, params);
        q_current = q_current(:);
        [s_current, currentSingularityInfo] = singularityMeasure( ...
            theta_actual, q_current, params);

        currentStateValid = ...
            isequal(size(q_current), [3, 1]) && ...
            all(isfinite(q_current)) && ...
            isfinite(s_current) && ...
            isfield(currentSingularityInfo, 'poleDistances') && ...
            numel(currentSingularityInfo.poleDistances) == 6 && ...
            all(isfinite(currentSingularityInfo.poleDistances(:)));
    catch currentStateException
        currentStateValid = false;
        diagnostics.message = string(currentStateException.message);
        q_current = q_des_prev;
        s_current = NaN;
        currentSingularityInfo.poleDistances = NaN(6, 1);
    end

    diagnostics.currentStateValid = currentStateValid;
    diagnostics.currentSingularityDistance = s_current;

    if ~currentStateValid
        q_des_mpc = q_des_prev;
        previousDeltaQ = zeros(3, Nc);
        [detourActive, committedPoleIndex, committedDetourSide, ...
            committedClearance, committedReference, ...
            detourFailureCount, detourClearCounter] = ...
            clearedDetourState(initialDetourClearance);

        diagnostics.fallbackUsed = true;
        diagnostics.fallbackCount = 1;
        diagnostics.fallbackReason = "invalidCurrentStateHold";

        q_des_mpc = limitQStep(q_des_mpc, q_des_prev, maxQStep);
        q_des_mpc = clampVector(q_des_mpc, qMin, qMax);
        stateOut = buildStateOut( ...
            theta_prev_for_omega, recoveryActive, previousDeltaQ, ...
            detourActive, committedPoleIndex, committedDetourSide, ...
            committedClearance, committedReference, ...
            detourFailureCount, detourClearCounter);
        return;
    end

    % ------------------------------------------------------------------
    % Requested target validity.
    % ------------------------------------------------------------------
    targetStateValid = true;
    try
        theta_target = abenicsIK(q_ref, params);
        theta_target = theta_target(:);
        [s_target, targetSingularityInfo] = singularityMeasure( ...
            theta_target, q_ref, params);
        targetPoleDistances = targetSingularityInfo.poleDistances(:);

        targetStateValid = ...
            isequal(size(theta_target), [4, 1]) && ...
            all(isfinite(theta_target)) && ...
            isfinite(s_target) && ...
            isequal(size(targetPoleDistances), [6, 1]) && ...
            all(isfinite(targetPoleDistances));
    catch
        targetStateValid = false;
        s_target = inf;
        targetPoleDistances = inf(6, 1);
    end

    allowSingularTarget = logical(readMpcSetting( ...
        params, 'allowSingularTarget', false));
    singularTargetOverrideActive = ...
        allowSingularTarget && targetStateValid && s_target < warningDistance;

    params.mpc.singularTargetOverrideActive = ...
        singularTargetOverrideActive;
    params.mpc.targetSingularityDistance = s_target;
    params.mpc.targetPoleDistances = targetPoleDistances;
    params.mpc.currentPoleDistances = ...
        currentSingularityInfo.poleDistances(:);

    diagnostics.targetStateValid = targetStateValid;
    diagnostics.targetSingularityDistance = s_target;
    diagnostics.singularTargetOverrideActive = ...
        singularTargetOverrideActive;

    % ------------------------------------------------------------------
    % Preserve existing recovery hysteresis behavior.
    % ------------------------------------------------------------------
    targetInsideDangerOverride = ...
        singularTargetOverrideActive && s_target < dangerDistance;

    if targetInsideDangerOverride
        disableRecoveryForSingularTarget = logical(readMpcSetting( ...
            params, 'disableRecoveryForSingularTarget', false));
        targetInsideDangerOverride = ... %#ok<NASGU>
            singularTargetOverrideActive && ...
            disableRecoveryForSingularTarget && ...
            s_target < dangerDistance;
    else
        if s_current < dangerDistance
            recoveryActive = true;
        elseif recoveryActive && s_current >= recoveryClearDistance
            recoveryActive = false;
        end
    end
    diagnostics.recoveryActive = recoveryActive;

    % ------------------------------------------------------------------
    % Emergency recovery bypasses normal detour MPC.
    % ------------------------------------------------------------------
    if recoveryActive
        [q_des_mpc, recoveryCommandValid, recoveryInfo] = ...
            emergencyRecoveryCommand( ...
                q_ref, q_des_prev, x0, ...
                qMin, qMax, thetaMin, thetaMax, ...
                omegaMax, alphaMax, recoveryMaxQStep, params);
        previousDeltaQ = zeros(3, Nc);
        [detourActive, committedPoleIndex, committedDetourSide, ...
            committedClearance, committedReference, ...
            detourFailureCount, detourClearCounter] = ...
            clearedDetourState(initialDetourClearance);

        diagnostics.recoveryUsed = true;
        diagnostics.recoveryCommandValid = recoveryCommandValid;
        diagnostics.recoveryMinimumTransitionPoleDistance = ...
            recoveryInfo.minimumTransitionPoleDistance;
        diagnostics.fallbackUsed = true;
        diagnostics.fallbackCount = 1;
        if recoveryCommandValid
            diagnostics.fallbackReason = "emergencyRecovery";
        else
            diagnostics.fallbackReason = "emergencyRecoverySafeHold";
        end

        q_des_mpc = limitQStep( ...
            q_des_mpc, q_des_prev, recoveryMaxQStep);
        q_des_mpc = clampVector(q_des_mpc, qMin, qMax);
        stateOut = buildStateOut( ...
            theta_prev_for_omega, recoveryActive, previousDeltaQ, ...
            detourActive, committedPoleIndex, committedDetourSide, ...
            committedClearance, committedReference, ...
            detourFailureCount, detourClearCounter);
        return;
    end

    % ------------------------------------------------------------------
    % Shift and repair the previous optimized move sequence.
    % ------------------------------------------------------------------
    lb = repmat(-maxQStep, Nc, 1);
    ub = repmat( maxQStep, Nc, 1);

    shiftedZ0 = buildShiftedWarmStart(previousDeltaQ, Nc);
    shiftedZ0 = clampVector(shiftedZ0, lb, ub);
    [shiftedZ0, shiftedWarmStartFeasible] = repairWarmStart( ...
        shiftedZ0, x0, q_des_prev, constraintTolerance, params);
    diagnostics.shiftedWarmStartFeasible = shiftedWarmStartFeasible;

    directZ0 = buildDirectIncrementSeed( ...
        q_des_prev, q_ref, maxQStep, Nc);

    % ------------------------------------------------------------------
    % Detect a blocking pole from predictions rooted in the actual x0.
    % ------------------------------------------------------------------
    [routeBlocked, blockingPoleIndex, directRollout, blockInfo] = ...
        detectBlockingPole( ...
            x0, q_current, q_des_prev, q_ref, ...
            shiftedZ0, directZ0, targetStateValid, ...
            enableDetourMultistart, params);

    diagnostics.routeBlocked = routeBlocked;
    diagnostics.blockingPoleIndex = blockingPoleIndex;
    diagnostics.minimumNominalPoleDistance = blockInfo.minimumDistance;
    diagnostics.directRolloutFinite = directRollout.finite;
    diagnostics.directRouteBlocked = blockInfo.directBlocked;
    diagnostics.minimumDirectPoleDistance = ...
        blockInfo.directMinimumDistance;

    % ------------------------------------------------------------------
    % Maintain or clear route commitment using hysteresis.
    % ------------------------------------------------------------------
    referenceChanged = false;
    if detourActive
        referenceChanged = max(abs(wrapAngleDifference( ...
            q_ref, committedReference))) > ...
            detourReferenceChangeTolerance;
    end

    targetReached = max(abs(wrapAngleDifference( ...
        q_current, q_ref))) <= detourTargetTolerance;

    if ~enableDetourMultistart || referenceChanged || targetReached
        [detourActive, committedPoleIndex, committedDetourSide, ...
            committedClearance, committedReference, ...
            detourFailureCount, detourClearCounter] = ...
            clearedDetourState(initialDetourClearance);
    elseif detourActive && routeBlocked && ...
            blockingPoleIndex ~= committedPoleIndex
        % The physical blocking geometry changed. Drop stale commitment and
        % reconsider both sides around the newly selected signed pole.
        [detourActive, committedPoleIndex, committedDetourSide, ...
            committedClearance, committedReference, ...
            detourFailureCount, detourClearCounter] = ...
            clearedDetourState(initialDetourClearance);
    elseif detourActive
        poleBehind = isCommittedPoleBehind( ...
            directRollout, committedPoleIndex, ...
            detourTriggerDistance, detourClearDistance);

        % Commitment clearing is based on the direct route itself, not the
        % shifted committed rollout. Otherwise a stale shifted path can keep
        % routeBlocked true even after the pole has genuinely been passed.
        directRouteClear = directRollout.finite && ...
            ~blockInfo.directBlocked;

        if directRouteClear && poleBehind
            detourClearCounter = detourClearCounter + 1;
        else
            detourClearCounter = 0;
        end

        if detourClearCounter >= detourClearConfirmations
            [detourActive, committedPoleIndex, committedDetourSide, ...
                committedClearance, committedReference, ...
                detourFailureCount, detourClearCounter] = ...
                clearedDetourState(initialDetourClearance);
        end
    else
        detourClearCounter = 0;
    end

    % ------------------------------------------------------------------
    % Build the fixed-size start set.
    % Start codes: shifted=0, positive=+1, negative=-1, unused=99.
    % ------------------------------------------------------------------
    START_UNUSED = 99;
    Z0 = NaN(3 * Nc, maxDetourStarts);
    startEnabled = false(maxDetourStarts, 1);
    startType = START_UNUSED * ones(maxDetourStarts, 1);
    startPoleIndex = zeros(maxDetourStarts, 1);
    startSide = zeros(maxDetourStarts, 1);
    startClearance = NaN(maxDetourStarts, 1);
    startSeedValid = false(maxDetourStarts, 1);
    startSeedBlendScale = NaN(maxDetourStarts, 1);
    startSeedConstraintViolation = NaN(maxDetourStarts, 1);
    startPreRejectCode = zeros(maxDetourStarts, 1);

    numberOfStarts = 1;
    Z0(:, 1) = shiftedZ0;
    startEnabled(1) = true;
    startType(1) = 0;
    startSeedValid(1) = true;
    startSeedBlendScale(1) = 0;

    selectedDetourPole = 0;
    if detourActive
        selectedDetourPole = committedPoleIndex;
    elseif routeBlocked
        selectedDetourPole = blockingPoleIndex;
    end

    if enableDetourMultistart && selectedDetourPole >= 1
        if detourActive
            requestedSides = committedDetourSide;

            % Continue on the committed side without forcing the measured
            % state all the way back to the original 10-degree ring. As the
            % plant approaches the pole, gradually reduce the working ring
            % only as far as the configured continuation clearance.
            currentCommittedPoleDistance = ...
                currentSingularityInfo.poleDistances( ...
                    committedPoleIndex);
            requestedClearance = min( ...
                committedClearance, ...
                max(detourContinuationClearance, ...
                    currentCommittedPoleDistance));
        else
            requestedSides = [1; -1];
            requestedClearance = initialDetourClearance;
        end

        for sideIndex = 1:numel(requestedSides)
            if numberOfStarts >= maxDetourStarts
                break;
            end

            numberOfStarts = numberOfStarts + 1;
            thisSide = requestedSides(sideIndex);

            startType(numberOfStarts) = thisSide;
            startPoleIndex(numberOfStarts) = selectedDetourPole;
            startSide(numberOfStarts) = thisSide;
            startClearance(numberOfStarts) = requestedClearance;

            [zSeed, seedInfo, seedGeometryValid] = ...
                generateGeometricDetourSeed( ...
                    q_current, q_des_prev, q_ref, ...
                    selectedDetourPole, thisSide, ...
                    requestedClearance, maxQStep, params);

            if seedGeometryValid
                [zSeed, seedRolloutValid, seedRejectCode, ...
                    seedBlendScale, seedConstraintViolation] = ...
                    prepareGeometricSeedForSolve( ...
                        zSeed, shiftedZ0, x0, ...
                        q_current, q_des_prev, ...
                        selectedDetourPole, thisSide, ...
                        constraintTolerance, params);
            else
                seedRolloutValid = false;
                seedRejectCode = seedInfo.reasonCode;
                seedBlendScale = NaN;
                seedConstraintViolation = NaN;
            end

            startSeedValid(numberOfStarts) = seedRolloutValid;
            startSeedBlendScale(numberOfStarts) = seedBlendScale;
            startSeedConstraintViolation(numberOfStarts) = ...
                seedConstraintViolation;
            startPreRejectCode(numberOfStarts) = seedRejectCode;

            if seedRolloutValid
                Z0(:, numberOfStarts) = zSeed;
                startEnabled(numberOfStarts) = true;
            end
        end
    end

    diagnostics.numberOfStarts = numberOfStarts;
    diagnostics.startType(1:numberOfStarts) = ...
        startType(1:numberOfStarts);
    diagnostics.startPoleIndex(1:numberOfStarts) = ...
        startPoleIndex(1:numberOfStarts);
    diagnostics.startSide(1:numberOfStarts) = ...
        startSide(1:numberOfStarts);
    diagnostics.startClearance(1:numberOfStarts) = ...
        startClearance(1:numberOfStarts);
    diagnostics.startSeedValid(1:numberOfStarts) = ...
        startSeedValid(1:numberOfStarts);
    diagnostics.startSeedBlendScale(1:numberOfStarts) = ...
        startSeedBlendScale(1:numberOfStarts);
    diagnostics.startSeedConstraintViolation(1:numberOfStarts) = ...
        startSeedConstraintViolation(1:numberOfStarts);

    % ------------------------------------------------------------------
    % Run the same continuous fmincon problem from every enabled start.
    % ------------------------------------------------------------------
    objectiveFunction = @(z) continuousMpcObjective( ...
        z, x0, q_ref, q_des_prev, params);
    constraintFunction = @(z) continuousMpcConstraints( ...
        z, x0, q_des_prev, params);

    solverAvailable = exist('fmincon', 'file') == 2;
    diagnostics.solverAvailable = solverAvailable;

    if solverAvailable
        options = optimoptions('fmincon', ...
            'Algorithm', 'sqp', ...
            'Display', 'none', ...
            'MaxIterations', readMpcSetting( ...
                params, 'maxIterations', 15), ...
            'MaxFunctionEvaluations', readMpcSetting( ...
                params, 'maxFunctionEvaluations', 2000), ...
            'ConstraintTolerance', constraintTolerance, ...
            'OptimalityTolerance', readMpcSetting( ...
                params, 'optimalityTolerance', 1e-4), ...
            'StepTolerance', readMpcSetting( ...
                params, 'stepTolerance', 1e-6));
    else
        options = [];
    end

    solveResult = runMpcMultiStart( ...
        Z0, startEnabled, startPreRejectCode, numberOfStarts, ...
        startType, startSide, ...
        objectiveFunction, constraintFunction, options, ...
        lb, ub, x0, q_ref, q_current, q_des_prev, ...
        qMin, qMax, maxQStep, constraintTolerance, ...
        solverAvailable, routeBlocked, detourActive, ...
        selectedDetourPole, committedDetourSide, ...
        detourMinimumOptimizedProgress, params);

    diagnostics.usedFmincon = solveResult.numberOfSolves > 0;
    diagnostics.numberOfFminconSolves = solveResult.numberOfSolves;
    diagnostics.numberOfAcceptedStarts = solveResult.numberAccepted;
    diagnostics.numberOfRouteEligibleStarts = ...
        solveResult.numberRouteEligible;
    diagnostics.totalSolveTime = solveResult.totalSolveTime;
    diagnostics.worstStartSolveTime = solveResult.worstSolveTime;
    diagnostics.solveTime = solveResult.totalSolveTime;
    diagnostics.winningStartIndex = solveResult.bestIndex;
    diagnostics.winningRouteEligible = ...
        solveResult.bestRouteEligible;
    diagnostics.winningProgress = solveResult.bestProgress;
    diagnostics.winningProgressMode = solveResult.bestProgressMode;
    diagnostics.winningFirstProgress = solveResult.bestFirstProgress;
    diagnostics.winningFirstPoleDistance = ...
        solveResult.bestFirstPoleDistance;
    diagnostics.winningNearPoleDistance = ...
        solveResult.bestNearPoleDistance;
    diagnostics.winningMinimumNearPoleDistance = ...
        solveResult.bestMinimumNearPoleDistance;

    diagnostics.startExitflag(1:numberOfStarts) = ...
        solveResult.exitflag(1:numberOfStarts);
    diagnostics.startAccepted(1:numberOfStarts) = ...
        solveResult.acceptedStart(1:numberOfStarts);
    diagnostics.startObjective(1:numberOfStarts) = ...
        solveResult.objective(1:numberOfStarts);
    diagnostics.startMinimumPoleDistance(1:numberOfStarts) = ...
        solveResult.minimumPoleDistance(1:numberOfStarts);
    diagnostics.startIterations(1:numberOfStarts) = ...
        solveResult.iterations(1:numberOfStarts);
    diagnostics.startFunctionEvaluations(1:numberOfStarts) = ...
        solveResult.functionEvaluations(1:numberOfStarts);
    diagnostics.startSolveTime(1:numberOfStarts) = ...
        solveResult.solveTime(1:numberOfStarts);
    diagnostics.startRejectionCode(1:numberOfStarts) = ...
        solveResult.rejectionCode(1:numberOfStarts);
    diagnostics.startRouteEligible(1:numberOfStarts) = ...
        solveResult.routeEligible(1:numberOfStarts);
    diagnostics.startProgressSide(1:numberOfStarts) = ...
        solveResult.progressSide(1:numberOfStarts);
    diagnostics.startProgress(1:numberOfStarts) = ...
        solveResult.progress(1:numberOfStarts);
    diagnostics.startProgressMode(1:numberOfStarts) = ...
        solveResult.progressMode(1:numberOfStarts);
    diagnostics.startFirstProgress(1:numberOfStarts) = ...
        solveResult.firstProgress(1:numberOfStarts);
    diagnostics.startFirstPoleDistance(1:numberOfStarts) = ...
        solveResult.firstPoleDistance(1:numberOfStarts);
    diagnostics.startNearPoleDistance(1:numberOfStarts) = ...
        solveResult.nearPoleDistance(1:numberOfStarts);
    diagnostics.startMinimumNearPoleDistance(1:numberOfStarts) = ...
        solveResult.minimumNearPoleDistance(1:numberOfStarts);

    diagnostics.solutionAccepted = solveResult.accepted;
    diagnostics.exitflag = solveResult.reportExitflag;
    diagnostics.objective = solveResult.reportObjective;
    diagnostics.iterations = solveResult.reportIterations;
    diagnostics.functionEvaluations = ...
        solveResult.reportFunctionEvaluations;
    diagnostics.message = solveResult.reportMessage;

    validation = solveResult.reportValidation;
    diagnostics.solutionFinite = validation.solutionFinite;
    diagnostics.objectiveFinite = validation.objectiveFinite;
    diagnostics.constraintsSatisfied = validation.constraintsSatisfied;
    diagnostics.firstCommandValid = validation.firstCommandValid;
    diagnostics.firstTransitionSafe = validation.firstTransitionSafe;
    diagnostics.minimumFirstTransitionPoleDistance = ...
        validation.minimumFirstTransitionPoleDistance;
    diagnostics.rolloutFinite = validation.rolloutFinite;
    diagnostics.maxConstraintViolation = ...
        validation.maxConstraintViolation;
    diagnostics.maxConstraintIndex = validation.maxConstraintIndex;
    diagnostics.maxConstraintStep = validation.maxConstraintStep;
    diagnostics.maxConstraintName = validation.maxConstraintName;

    optimizationAccepted = solveResult.accepted;
    zOptimal = solveResult.bestZ;

    % ------------------------------------------------------------------
    % Receding-horizon output or existing validated fallback.
    % ------------------------------------------------------------------
    if optimizationAccepted
        deltaQOptimal = reshape(zOptimal, 3, Nc);
        q_des_mpc = q_des_prev + deltaQOptimal(:, 1);
        previousDeltaQ = deltaQOptimal;

        winningIndex = solveResult.bestIndex;
        winningStartType = startType(winningIndex);
        winningSide = solveResult.bestProgressSide;
        winningClearance = startClearance(winningIndex);

        if winningStartType == 0 && detourActive
            if ~(winningSide == 1 || winningSide == -1)
                winningSide = committedDetourSide;
            end
            winningClearance = committedClearance;
        elseif winningStartType == 0 && routeBlocked
            winningClearance = initialDetourClearance;
        end

        diagnostics.winningStartType = winningStartType;
        diagnostics.winningDetourSide = winningSide;
        diagnostics.winningClearance = winningClearance;

        if enableDetourMultistart && ...
                selectedDetourPole >= 1 && ...
                solveResult.bestRouteEligible && ...
                (winningSide == 1 || winningSide == -1)
            detourActive = true;
            committedPoleIndex = selectedDetourPole;
            committedDetourSide = winningSide;
            committedClearance = winningClearance;
            committedReference = q_ref;
            detourFailureCount = 0;
            detourClearCounter = 0;

        elseif detourActive && ~solveResult.bestRouteEligible
            % A safe solution was available, but none of the optimized
            % candidates continued around the committed side. Keep the
            % current output safe while counting this as a route failure.
            detourFailureCount = detourFailureCount + 1;
            if detourFailureCount >= maxDetourFailures
                [detourActive, committedPoleIndex, committedDetourSide, ...
                    committedClearance, committedReference, ...
                    detourFailureCount, detourClearCounter] = ...
                    clearedDetourState(initialDetourClearance);
            end

        elseif ~routeBlocked && ~detourActive
            [detourActive, committedPoleIndex, committedDetourSide, ...
                committedClearance, committedReference, ...
                detourFailureCount, detourClearCounter] = ...
                clearedDetourState(initialDetourClearance);
        end
    else
        previousDeltaQ = zeros(3, Nc);

        if detourActive
            detourFailureCount = detourFailureCount + 1;
            if detourFailureCount >= maxDetourFailures
                [detourActive, committedPoleIndex, committedDetourSide, ...
                    committedClearance, committedReference, ...
                    detourFailureCount, detourClearCounter] = ...
                    clearedDetourState(initialDetourClearance);
            end
        end

        [q_fallback, fallbackValid, fallbackInfo] = ...
            chooseValidatedFallbackCommand( ...
                q_ref, q_current, q_des_prev, x0, ...
                routeBlocked || detourActive, selectedDetourPole, ...
                committedDetourSide, qMin, qMax, thetaMin, thetaMax, ...
                omegaMax, alphaMax, maxQStep, dangerDistance, params);

        diagnostics.fallbackUsed = true;
        diagnostics.fallbackCount = 1;
        diagnostics.directFallbackValid = fallbackValid;
        diagnostics.directFallbackMaxViolation = ...
            fallbackInfo.maxViolation;
        diagnostics.fallbackMinimumTransitionPoleDistance = ...
            fallbackInfo.minimumTransitionPoleDistance;

        q_des_mpc = q_fallback;
        if fallbackValid && fallbackInfo.routeAware && ...
                ~detourActive && ...
                (fallbackInfo.selectedSide == 1 || ...
                 fallbackInfo.selectedSide == -1)
            detourActive = true;
            committedPoleIndex = selectedDetourPole;
            committedDetourSide = fallbackInfo.selectedSide;
            committedClearance = initialDetourClearance;
            committedReference = q_ref;
            detourFailureCount = 0;
            detourClearCounter = 0;
        end
        if fallbackValid && fallbackInfo.routeAware
            diagnostics.fallbackReason = "validatedRouteFallback";
        elseif fallbackValid
            diagnostics.fallbackReason = "validatedDirectFallback";
        else
            diagnostics.fallbackReason = "validatedSafeHoldUnavailable";
        end
    end

    q_des_mpc = limitQStep(q_des_mpc, q_des_prev, maxQStep);
    q_des_mpc = clampVector(q_des_mpc, qMin, qMax);

    if ~isequal(size(q_des_mpc), [3, 1]) || ...
            any(~isfinite(q_des_mpc))
        q_des_mpc = q_des_prev;
        previousDeltaQ = zeros(3, Nc);
        diagnostics.fallbackUsed = true;
        diagnostics.fallbackCount = 1;
        diagnostics.fallbackReason = "nonfiniteFinalOutputHold";
    end

    diagnostics.detourActive = detourActive;
    diagnostics.committedPoleIndex = committedPoleIndex;
    diagnostics.committedDetourSide = committedDetourSide;
    diagnostics.detourClearCounter = detourClearCounter;
    diagnostics.detourFailureCount = detourFailureCount;

    stateOut = buildStateOut( ...
        theta_prev_for_omega, recoveryActive, previousDeltaQ, ...
        detourActive, committedPoleIndex, committedDetourSide, ...
        committedClearance, committedReference, ...
        detourFailureCount, detourClearCounter);
end


% =========================================================================
% Shifted/direct initial guesses and blocking detection
% =========================================================================
function z0 = buildShiftedWarmStart(previousDeltaQ, Nc)

    shiftedDeltaQ = zeros(3, Nc);
    if Nc > 1
        shiftedDeltaQ(:, 1:Nc-1) = previousDeltaQ(:, 2:Nc);
    end
    shiftedDeltaQ(:, Nc) = zeros(3, 1);
    z0 = shiftedDeltaQ(:);
end


function [z0, feasibleFound] = repairWarmStart( ...
    z0, x0, q_des_prev, constraintTolerance, params)

    warmStartScales = [1.0, 0.5, 0.25, 0.0];
    feasibleFound = false;

    for scaleIndex = 1:numel(warmStartScales)
        zCandidate = warmStartScales(scaleIndex) * z0;
        [cInitial, ceqInitial] = continuousMpcConstraints( ...
            zCandidate, x0, q_des_prev, params);

        inequalitiesValid = isempty(cInitial) || ...
            all(cInitial <= constraintTolerance);
        equalitiesValid = isempty(ceqInitial) || ...
            all(abs(ceqInitial) <= constraintTolerance);

        if inequalitiesValid && equalitiesValid
            z0 = zCandidate;
            feasibleFound = true;
            return;
        end
    end

    z0 = zeros(size(z0));
end


function z = buildDirectIncrementSeed( ...
    q_des_prev, q_ref, maxQStep, Nc)

    deltaQ = zeros(3, Nc);
    qCursor = q_des_prev;

    for i = 1:Nc
        delta = wrapAngleDifference(q_ref, qCursor);
        deltaQ(:, i) = clampVector(delta, -maxQStep, maxQStep);
        qCursor = qCursor + deltaQ(:, i);
    end

    z = deltaQ(:);
end


function [blocked, poleIndex, directRollout, info] = ...
    detectBlockingPole( ...
        x0, q_current, q_des_prev, q_ref, ...
        shiftedZ0, directZ0, targetStateValid, ...
        enableDetourMultistart, params)

    blocked = false;
    poleIndex = 0;
    info.minimumDistance = inf;
    info.minimumStep = NaN;
    info.sourceCode = 0;
    info.directMinimumDistance = inf;
    info.directPoleIndex = 0;
    info.directBlocked = false;

    directRollout = simulateContinuousMpcTrajectory( ...
        directZ0, x0, q_des_prev, params);
    shiftedRollout = simulateContinuousMpcTrajectory( ...
        shiftedZ0, x0, q_des_prev, params);

    if ~enableDetourMultistart || ...
            ~targetStateValid || ...
            any(~isfinite(q_current)) || ...
            any(~isfinite(q_ref))
        return;
    end

    triggerDistance = params.mpc.detourTriggerDistance;
    bestDistances = inf(6, 1);
    bestSteps = NaN(6, 1);
    bestSources = zeros(6, 1);

    if directRollout.finite
        directDistances = inf(6, 1);
        for j = 1:6
            [directDistances(j), bestSteps(j)] = min( ...
                directRollout.poleDistances(j, :));
            bestDistances(j) = directDistances(j);
            bestSources(j) = 1;
        end

        [info.directMinimumDistance, info.directPoleIndex] = ...
            min(directDistances);
        info.directBlocked = ...
            info.directMinimumDistance < triggerDistance;
    end

    if shiftedRollout.finite
        for j = 1:6
            [shiftDistance, shiftStep] = min( ...
                shiftedRollout.poleDistances(j, :));
            if shiftDistance < bestDistances(j)
                bestDistances(j) = shiftDistance;
                bestSteps(j) = shiftStep;
                bestSources(j) = 2;
            end
        end
    end

    [minimumDistance, candidatePole] = min(bestDistances);
    if ~isfinite(minimumDistance)
        return;
    end

    info.minimumDistance = minimumDistance;
    info.minimumStep = bestSteps(candidatePole);
    info.sourceCode = bestSources(candidatePole);

    if minimumDistance < triggerDistance
        blocked = true;
        poleIndex = candidatePole;
    end
end


function behind = isCommittedPoleBehind( ...
    directRollout, poleIndex, triggerDistance, clearDistance)

    behind = false;
    if ~directRollout.finite || poleIndex < 1 || poleIndex > 6
        return;
    end

    distances = directRollout.poleDistances(poleIndex, :);
    if any(~isfinite(distances))
        return;
    end

    [minimumDistance, minimumStep] = min(distances);
    earlyStepLimit = max(2, ceil(0.15 * numel(distances)));
    distanceTrend = distances(end) - distances(1);

    behind = ...
        minimumDistance >= triggerDistance && ...
        distances(1) >= clearDistance && ...
        minimumStep <= earlyStepLimit && ...
        distanceTrend >= -1e-6;
end


% =========================================================================
% Geometry-aware seed generation
% =========================================================================
function [zSeed, info, valid] = generateGeometricDetourSeed( ...
    q_current, q_des_prev, q_ref, ...
    poleIndex, detourSide, clearance, maxQStep, params)

    % Numeric geometry reason codes are kept fixed for test translation.
    REASON_OK = 0;
    REASON_BAD_INPUT = 101;
    REASON_BAD_POLE = 102;
    REASON_BAD_CLEARANCE = 103;
    REASON_UNSAFE_ENDPOINT = 104;
    REASON_TANGENT_BASIS = 105;
    REASON_RAW_AXIS_UNSAFE = 106;
    REASON_ROTATION_PATH = 107;
    REASON_ORIENTATION_UNSAFE = 108;
    REASON_Q_LIMIT = 109;
    REASON_RESAMPLE = 110;
    REASON_NO_SIDE_PROGRESS = 111;

    Nc = params.mpc.Nc;
    zSeed = zeros(3 * Nc, 1);
    info.reasonCode = REASON_BAD_INPUT;
    info.minimumRawPoleDistance = NaN;
    info.phiTarget = NaN;
    info.phiRoute = NaN;
    info.reachedDenseIndex = 0;
    info.ringStartIndex = 0;
    info.ringEndIndex = 0;
    valid = false;

    q_current = q_current(:);
    q_des_prev = q_des_prev(:);
    q_ref = q_ref(:);
    maxQStep = maxQStep(:);

    if ~isequal(size(q_current), [3, 1]) || ...
            ~isequal(size(q_des_prev), [3, 1]) || ...
            ~isequal(size(q_ref), [3, 1]) || ...
            ~isequal(size(maxQStep), [3, 1]) || ...
            any(~isfinite([q_current; q_des_prev; q_ref; maxQStep])) || ...
            ~(detourSide == 1 || detourSide == -1)
        return;
    end

    poleAxes = params.singularity.poleAxes;
    if ~isequal(size(poleAxes), [3, 6]) || ...
            poleIndex < 1 || poleIndex > 6
        info.reasonCode = REASON_BAD_POLE;
        return;
    end

    dangerDistance = params.singularity.dangerDistance;
    geometryTolerance = readMpcSetting( ...
        params, 'detourGeometryTolerance', 1e-10);
    axisSampleStep = readMpcSetting( ...
        params, 'detourAxisSampleStep', deg2rad(0.25));
    axisSampleStep = max(axisSampleStep, deg2rad(0.05));

    if ~isscalar(clearance) || ~isfinite(clearance) || ...
            clearance <= dangerDistance || clearance >= pi
        info.reasonCode = REASON_BAD_CLEARANCE;
        return;
    end

    bodyAxis = params.singularity.trackedBodyAxis(:);
    if numel(bodyAxis) ~= 3 || norm(bodyAxis) < geometryTolerance
        info.reasonCode = REASON_BAD_INPUT;
        return;
    end
    bodyAxis = bodyAxis / norm(bodyAxis);

    p = poleAxes(:, poleIndex);
    if norm(p) < geometryTolerance
        info.reasonCode = REASON_BAD_POLE;
        return;
    end
    p = p / norm(p);

    R_current = qToRotmXYZ(q_current);
    R_command = qToRotmXYZ(q_des_prev);
    R_ref = qToRotmXYZ(q_ref);

    v_start = normalizeVector(R_current * bodyAxis, geometryTolerance);
    v_target = normalizeVector(R_ref * bodyAxis, geometryTolerance);

    startPoleDistances = axisPoleDistances(v_start, poleAxes);
    targetPoleDistances = axisPoleDistances(v_target, poleAxes);
    if min(startPoleDistances) < dangerDistance || ...
            min(targetPoleDistances) < dangerDistance
        info.reasonCode = REASON_UNSAFE_ENDPOINT;
        return;
    end

    d_start = acos(clampScalar(dot(v_start, p), -1, 1));
    d_target = acos(clampScalar(dot(v_target, p), -1, 1));

    % The current or target axis may already lie inside the requested ring
    % while still remaining outside dangerDistance. In that case, the entry
    % or exit meridian moves monotonically outward to, or inward from, the
    % ring. Rejecting d_start < clearance caused committed detour seeds to
    % disappear precisely when the measured plant approached the pole.
    % Safety is enforced below by checking every raw axis sample against all
    % six hard pole regions, rather than by requiring both endpoints to lie
    % outside the nominal ring clearance.

    a_start_raw = v_start - dot(v_start, p) * p;
    a_target_raw = v_target - dot(v_target, p) * p;

    [fallbackAxis, fallbackValid] = deterministicTangentBasis( ...
        p, geometryTolerance);
    if norm(a_start_raw) < geometryTolerance
        if ~fallbackValid
            info.reasonCode = REASON_TANGENT_BASIS;
            return;
        end
        a_start = fallbackAxis;
    else
        a_start = a_start_raw / norm(a_start_raw);
    end

    if norm(a_target_raw) < geometryTolerance
        if ~fallbackValid
            info.reasonCode = REASON_TANGENT_BASIS;
            return;
        end
        a_target = fallbackAxis;
    else
        a_target = a_target_raw / norm(a_target_raw);
    end

    % Verify that a fallback basis did not invent a mismatched endpoint.
    reconstructedStart = ...
        cos(d_start) * p + sin(d_start) * a_start;
    reconstructedTargetBase = ...
        cos(d_target) * p + sin(d_target) * a_target;
    if acos(clampScalar(dot(reconstructedStart, v_start), -1, 1)) > 1e-6 || ...
       acos(clampScalar(dot(reconstructedTargetBase, v_target), -1, 1)) > 1e-6
        info.reasonCode = REASON_TANGENT_BASIS;
        return;
    end

    b = cross(p, a_start);
    if norm(b) < geometryTolerance
        info.reasonCode = REASON_TANGENT_BASIS;
        return;
    end
    b = b / norm(b);

    phiTarget = atan2(dot(a_target, b), dot(a_target, a_start));
    if detourSide > 0
        phiRoute = phiTarget;
        if phiRoute <= 0
            phiRoute = phiRoute + 2 * pi;
        end
    else
        phiRoute = phiTarget;
        if phiRoute >= 0
            phiRoute = phiRoute - 2 * pi;
        end
    end

    info.phiTarget = phiTarget;
    info.phiRoute = phiRoute;

    entryCount = max(2, ceil(abs(d_start - clearance) / ...
        axisSampleStep) + 1);
    ringArcLength = abs(phiRoute) * sin(clearance);
    ringCount = max(2, ceil(ringArcLength / axisSampleStep) + 1);
    exitCount = max(2, ceil(abs(d_target - clearance) / ...
        axisSampleStep) + 1);

    entryDistances = linspace(d_start, clearance, entryCount);
    ringPhi = linspace(0, phiRoute, ringCount);
    exitDistances = linspace(clearance, d_target, exitCount);

    axisPath = zeros(3, entryCount + ringCount + exitCount - 2);
    axisIndex = 0;

    for i = 1:entryCount
        axisIndex = axisIndex + 1;
        axisPath(:, axisIndex) = ...
            sphericalAxisPoint(p, a_start, b, entryDistances(i), 0);
    end

    info.ringStartIndex = axisIndex;
    for i = 2:ringCount
        axisIndex = axisIndex + 1;
        axisPath(:, axisIndex) = ...
            sphericalAxisPoint(p, a_start, b, clearance, ringPhi(i));
    end
    info.ringEndIndex = axisIndex;

    for i = 2:exitCount
        axisIndex = axisIndex + 1;
        axisPath(:, axisIndex) = sphericalAxisPoint( ...
            p, a_start, b, exitDistances(i), phiRoute);
    end

    axisPath = axisPath(:, 1:axisIndex);
    [axisSafe, minimumRawDistance] = validateAxisPathAgainstAllPoles( ...
        axisPath, poleAxes, dangerDistance);
    info.minimumRawPoleDistance = minimumRawDistance;
    if ~axisSafe
        info.reasonCode = REASON_RAW_AXIS_UNSAFE;
        return;
    end

    [rotationPath, rotationValid] = axisPathToRotationPath( ...
        axisPath, R_current, R_ref, bodyAxis, axisSampleStep, ...
        geometryTolerance, q_des_prev, params);
    if ~rotationValid
        info.reasonCode = REASON_ROTATION_PATH;
        return;
    end

    % Bridge command orientation to the measured-orientation path. This is
    % where q_des_prev remains the optimization origin while q_current
    % remains the physical geometry origin.
    [bridgePath, bridgeValid] = interpolateRotations( ...
        R_command, rotationPath(:, :, 1), axisSampleStep);
    if ~bridgeValid
        info.reasonCode = REASON_ROTATION_PATH;
        return;
    end

    fullRotationPath = bridgePath;
    if size(rotationPath, 3) > 1
        fullRotationPath = cat(3, fullRotationPath, ...
            rotationPath(:, :, 2:end));
    end

    % Do not reject the command-space bridge merely because its interpolated
    % tracked axis crosses a singular cap. The bridge joins q_des_prev to a
    % geometry rooted in q_current; it is an initial command construction,
    % not the physical plant trajectory. The actual predicted plant path is
    % checked immediately afterward by validateInitialSeedRollout, and every
    % optimized result still passes the complete nonlinear constraints.

    qMin = params.mpc.qMin(:);
    qMax = params.mpc.qMax(:);
    denseStepFraction = readMpcSetting( ...
        params, 'detourDenseQStepFraction', 0.5);
    denseStepFraction = min(max(denseStepFraction, 0.05), 0.95);

    [qDense, densePathValid] = ...
        rotationPathToAdaptiveUnwrappedQ( ...
            fullRotationPath, q_des_prev, ...
            denseStepFraction * maxQStep, qMin, qMax);
    if ~densePathValid || any(~isfinite(qDense(:)))
        info.reasonCode = REASON_RESAMPLE;
        return;
    end

    [qSeed, reachedIndex, resampleValid] = ...
        resampleReachableOrientationPrefix(qDense, maxQStep, Nc);
    info.reachedDenseIndex = reachedIndex;
    if ~resampleValid
        info.reasonCode = REASON_RESAMPLE;
        return;
    end

    % Both sides must progress far enough to become distinct. A seed that
    % only performs the common meridian entry gives fmincon no side cue.
    R_seed_end = qToRotmXYZ(qSeed(:, end));
    v_seed_end = R_seed_end * bodyAxis;
    tangent_seed_end = v_seed_end - dot(v_seed_end, p) * p;
    if norm(tangent_seed_end) < geometryTolerance
        info.reasonCode = REASON_NO_SIDE_PROGRESS;
        return;
    end
    tangent_seed_end = tangent_seed_end / norm(tangent_seed_end);
    phi_seed_end = atan2( ...
        dot(tangent_seed_end, b), dot(tangent_seed_end, a_start));
    minimumSeedSideProgress = readMpcSetting( ...
        params, 'detourMinimumSeedSideProgress', deg2rad(0.5));
    if detourSide * phi_seed_end <= minimumSeedSideProgress
        info.reasonCode = REASON_NO_SIDE_PROGRESS;
        return;
    end

    deltaQ = zeros(3, Nc);
    deltaQ(:, 1) = wrapAngleDifference(qSeed(:, 1), q_des_prev);
    for i = 2:Nc
        deltaQ(:, i) = wrapAngleDifference(qSeed(:, i), qSeed(:, i-1));
    end

    if any(abs(deltaQ(:)) > repmat(maxQStep, Nc, 1) + 1e-9)
        info.reasonCode = REASON_RESAMPLE;
        return;
    end

    % Numerical safeguard only. The path construction above, not this line,
    % is responsible for satisfying the variable bounds.
    deltaQ = min(max(deltaQ, -maxQStep), maxQStep);
    zSeed = deltaQ(:);

    info.reasonCode = REASON_OK;
    valid = true;
end


function point = sphericalAxisPoint(p, a, b, distance, phi)

    point = cos(distance) * p + sin(distance) * ...
        (cos(phi) * a + sin(phi) * b);
    point = point / norm(point);
end


function [basis, valid] = deterministicTangentBasis(p, tolerance)

    worldBasis = eye(3);
    alignments = abs(worldBasis' * p);
    [~, order] = sort(alignments, 'ascend');

    basis = zeros(3, 1);
    valid = false;
    for i = 1:3
        candidate = worldBasis(:, order(i));
        candidate = candidate - dot(candidate, p) * p;
        if norm(candidate) > tolerance
            basis = candidate / norm(candidate);
            valid = true;
            return;
        end
    end
end


function [safe, minimumDistance] = validateAxisPathAgainstAllPoles( ...
    axisPath, poleAxes, dangerDistance)

    minimumDistance = inf;
    safe = true;

    for i = 1:size(axisPath, 2)
        distances = axisPoleDistances(axisPath(:, i), poleAxes);
        minimumDistance = min(minimumDistance, min(distances));
        if any(distances < dangerDistance - 1e-12) || ...
                any(~isfinite(distances))
            safe = false;
            return;
        end
    end
end


function distances = axisPoleDistances(axisVector, poleAxes)

    axisVector = axisVector / norm(axisVector);
    normalizedPoles = poleAxes;
    for j = 1:size(normalizedPoles, 2)
        normalizedPoles(:, j) = ...
            normalizedPoles(:, j) / norm(normalizedPoles(:, j));
    end
    dots = normalizedPoles' * axisVector;
    distances = acos(min(1, max(-1, dots)));
end


function [rotationPath, valid] = axisPathToRotationPath( ...
    axisPath, R_start, R_ref, bodyAxis, sampleStep, tolerance, ...
    q_command_origin, params)

    valid = false;
    rotationPath = zeros(3, 3, size(axisPath, 2));
    R_previous = R_start;

    q_previous = q_command_origin(:);
    if ~isequal(size(q_previous), [3, 1]) || any(~isfinite(q_previous))
        return;
    end

    try
        theta_previous = abenicsIK(q_previous, params);
        theta_previous = theta_previous(:);
    catch
        return;
    end

    if ~isequal(size(theta_previous), [4, 1]) || ...
            any(~isfinite(theta_previous))
        return;
    end

    for i = 1:size(axisPath, 2)
        desiredAxis = axisPath(:, i);
        v_previous = R_previous * bodyAxis;

        [R_delta, deltaValid] = shortestAxisMappingRotation( ...
            v_previous, desiredAxis, tolerance);
        if ~deltaValid
            return;
        end

        % R_base maps the tracked body axis to the required geometric axis.
        % Rotation about desiredAxis is still free. Select that twist so the
        % IK branch changes continuously instead of introducing the 80-100
        % degree theta_rB jumps seen with the pure minimum-rotation seed.
        R_base = projectToRotationMatrix(R_delta * R_previous);
        [R_next, q_next, theta_next, twistValid] = ...
            selectIkContinuousTwist( ...
                R_base, desiredAxis, q_previous, ...
                theta_previous, params, tolerance);
        if ~twistValid
            return;
        end

        R_previous = R_next;
        q_previous = q_next;
        theta_previous = theta_next;
        rotationPath(:, :, i) = R_previous;
    end

    % Correct the remaining twist only after the tracked-axis detour has
    % safely reached the target axis. Because both endpoint rotations share
    % the same tracked axis, this correction is a twist about that axis.
    [correctionPath, correctionValid] = interpolateRotations( ...
        rotationPath(:, :, end), R_ref, sampleStep);
    if ~correctionValid
        return;
    end

    if size(correctionPath, 3) > 1
        rotationPath = cat(3, rotationPath, correctionPath(:, :, 2:end));
    end
    valid = true;
end


function [R_selected, q_selected, theta_selected, valid] = ...
    selectIkContinuousTwist( ...
        R_base, desiredAxis, q_previous, theta_previous, ...
        params, tolerance)

    R_selected = eye(3);
    q_selected = NaN(3, 1);
    theta_selected = NaN(4, 1);
    valid = false;

    if norm(desiredAxis) < tolerance
        return;
    end
    desiredAxis = desiredAxis / norm(desiredAxis);

    coarseCount = round(readMpcSetting( ...
        params, 'detourTwistCoarseCount', 37));
    fineCount = round(readMpcSetting( ...
        params, 'detourTwistFineCount', 21));
    coarseCount = max(5, coarseCount);
    fineCount = max(3, fineCount);

    qWeight = readMpcSetting( ...
        params, 'detourTwistQWeight', 0.05);
    twistWeight = readMpcSetting( ...
        params, 'detourTwistAngleWeight', 1e-5);

    coarseAngles = linspace(-pi, pi, coarseCount);
    [bestAngle, bestCost, bestMaximumThetaStep, ...
        R_selected, q_selected, theta_selected, coarseValid] = ...
        searchTwistAngles( ...
            coarseAngles, R_base, desiredAxis, ...
            q_previous, theta_previous, ...
            qWeight, twistWeight, params);

    if ~coarseValid
        return;
    end

    coarseSpacing = 2 * pi / max(1, coarseCount - 1);
    fineAngles = linspace( ...
        bestAngle - coarseSpacing, ...
        bestAngle + coarseSpacing, fineCount);

    [~, fineCost, fineMaximumThetaStep, ...
        R_fine, q_fine, theta_fine, fineValid] = ...
        searchTwistAngles( ...
            fineAngles, R_base, desiredAxis, ...
            q_previous, theta_previous, ...
            qWeight, twistWeight, params);

    if fineValid && ...
            (fineCost < bestCost - 1e-14 || ...
            (abs(fineCost - bestCost) <= 1e-14 && ...
             fineMaximumThetaStep < bestMaximumThetaStep))
        R_selected = R_fine;
        q_selected = q_fine;
        theta_selected = theta_fine;
    end

    valid = ...
        all(isfinite(R_selected(:))) && ...
        all(isfinite(q_selected)) && ...
        all(isfinite(theta_selected));
end


function [bestAngle, bestCost, bestMaximumThetaStep, ...
    R_best, q_best, theta_best, valid] = ...
    searchTwistAngles( ...
        twistAngles, R_base, desiredAxis, ...
        q_previous, theta_previous, ...
        qWeight, twistWeight, params)

    bestAngle = NaN;
    bestCost = inf;
    bestMaximumThetaStep = inf;
    R_best = eye(3);
    q_best = NaN(3, 1);
    theta_best = NaN(4, 1);
    valid = false;

    qMin = params.mpc.qMin(:);
    qMax = params.mpc.qMax(:);

    for candidateIndex = 1:numel(twistAngles)
        twistAngle = twistAngles(candidateIndex);
        R_candidate = projectToRotationMatrix( ...
            axisAngleToRotm(desiredAxis, twistAngle) * R_base);

        q_raw = rotmToQXYZ(R_candidate);
        q_candidate = q_previous + ...
            wrapAngleDifference(q_raw, q_previous);

        if any(~isfinite(q_candidate)) || ...
                any(q_candidate < qMin - 1e-10) || ...
                any(q_candidate > qMax + 1e-10)
            continue;
        end

        try
            theta_candidate = abenicsIK(q_candidate, params);
            theta_candidate = theta_candidate(:);
        catch
            continue;
        end

        if ~isequal(size(theta_candidate), [4, 1]) || ...
                any(~isfinite(theta_candidate))
            continue;
        end

        thetaStep = wrapAngleDifference( ...
            theta_candidate, theta_previous);
        qStep = wrapAngleDifference(q_candidate, q_previous);

        maximumThetaStep = max(abs(thetaStep));
        candidateCost = ...
            sum(thetaStep.^2) + ...
            qWeight * sum(qStep.^2) + ...
            twistWeight * twistAngle^2;

        if candidateCost < bestCost - 1e-14 || ...
          (abs(candidateCost - bestCost) <= 1e-14 && ...
           maximumThetaStep < bestMaximumThetaStep)
            bestAngle = twistAngle;
            bestCost = candidateCost;
            bestMaximumThetaStep = maximumThetaStep;
            R_best = R_candidate;
            q_best = q_candidate;
            theta_best = theta_candidate;
            valid = true;
        end
    end
end


function [R_delta, valid] = shortestAxisMappingRotation( ...
    fromAxis, toAxis, tolerance)

    valid = false;
    R_delta = eye(3);
    if norm(fromAxis) < tolerance || norm(toAxis) < tolerance
        return;
    end

    a = fromAxis / norm(fromAxis);
    b = toAxis / norm(toAxis);
    cosine = clampScalar(dot(a, b), -1, 1);

    if cosine > 1 - 1e-12
        valid = true;
        return;
    end

    if cosine < -1 + 1e-10
        [axis, axisValid] = deterministicTangentBasis(a, tolerance);
        if ~axisValid
            return;
        end
        R_delta = axisAngleToRotm(axis, pi);
        valid = true;
        return;
    end

    rotationAxis = cross(a, b);
    sine = norm(rotationAxis);
    if sine < tolerance
        return;
    end
    rotationAxis = rotationAxis / sine;
    angle = atan2(sine, cosine);
    R_delta = axisAngleToRotm(rotationAxis, angle);
    valid = true;
end


function [path, valid] = interpolateRotations(R_start, R_end, sampleStep)

    valid = false;
    q0 = rotmToQuaternion(R_start);
    q1 = rotmToQuaternion(R_end);
    if any(~isfinite([q0; q1]))
        path = zeros(3, 3, 0);
        return;
    end

    quaternionDot = abs(dot(q0, q1));
    quaternionDot = clampScalar(quaternionDot, -1, 1);
    relativeAngle = 2 * acos(quaternionDot);
    count = max(2, ceil(relativeAngle / sampleStep) + 1);
    path = zeros(3, 3, count);

    for i = 1:count
        lambda = (i - 1) / (count - 1);
        q = quaternionSlerp(q0, q1, lambda);
        path(:, :, i) = quaternionToRotm(q);
    end
    valid = true;
end


function [safe, minimumDistance] = validateRotationPathAgainstAllPoles( ...
    rotationPath, bodyAxis, poleAxes, dangerDistance)

    safe = true;
    minimumDistance = inf;
    for i = 1:size(rotationPath, 3)
        axisVector = rotationPath(:, :, i) * bodyAxis;
        distances = axisPoleDistances(axisVector, poleAxes);
        minimumDistance = min(minimumDistance, min(distances));
        if any(distances < dangerDistance - 1e-12) || ...
                any(~isfinite(distances))
            safe = false;
            return;
        end
    end
end


function [qPath, valid] = rotationPathToAdaptiveUnwrappedQ( ...
    rotationPath, q_start, maximumDenseStep, qMin, qMax)

    qPath = q_start(:);
    valid = false;

    if ~isequal(size(qPath), [3, 1]) || ...
            any(~isfinite(qPath)) || ...
            ~isequal(size(maximumDenseStep), [3, 1]) || ...
            any(~isfinite(maximumDenseStep)) || ...
            any(maximumDenseStep <= 0) || ...
            size(rotationPath, 1) ~= 3 || ...
            size(rotationPath, 2) ~= 3 || ...
            size(rotationPath, 3) < 1
        return;
    end

    R_cursor = qToRotmXYZ(qPath(:, end));
    startTarget = 1;

    % Avoid duplicating the command-origin rotation when the bridge already
    % begins exactly at q_start.
    if norm(rotationPath(:, :, 1) - R_cursor, 'fro') < 1e-10
        startTarget = 2;
    end

    for targetIndex = startTarget:size(rotationPath, 3)
        R_target = rotationPath(:, :, targetIndex);

        q_target_raw = rotmToQXYZ(R_target);
        q_target = qPath(:, end) + ...
            wrapAngleDifference(q_target_raw, qPath(:, end));

        estimatedPieces = max(1, ceil(max( ...
            abs(wrapAngleDifference(q_target, qPath(:, end))) ./ ...
            maximumDenseStep)));
        pieces = estimatedPieces;
        segmentAccepted = false;

        for refinement = 1:10
            qSegment = zeros(3, pieces);
            q_previous = qPath(:, end);
            segmentValid = true;

            q0 = rotmToQuaternion(R_cursor);
            q1 = rotmToQuaternion(R_target);

            for pieceIndex = 1:pieces
                lambda = pieceIndex / pieces;
                qQuaternion = quaternionSlerp(q0, q1, lambda);
                R_piece = quaternionToRotm(qQuaternion);
                q_raw = rotmToQXYZ(R_piece);
                q_piece = q_previous + ...
                    wrapAngleDifference(q_raw, q_previous);

                step = abs(wrapAngleDifference(q_piece, q_previous));
                if any(step > maximumDenseStep + 1e-10) || ...
                        any(q_piece < qMin - 1e-9) || ...
                        any(q_piece > qMax + 1e-9) || ...
                        any(~isfinite(q_piece))
                    segmentValid = false;
                    break;
                end

                qSegment(:, pieceIndex) = q_piece;
                q_previous = q_piece;
            end

            if segmentValid
                qPath = [qPath, qSegment]; %#ok<AGROW>
                R_cursor = R_target;
                segmentAccepted = true;
                break;
            end

            pieces = 2 * pieces;
        end

        if ~segmentAccepted
            return;
        end
    end

    valid = size(qPath, 2) >= 2 && all(isfinite(qPath(:)));
end


function qPath = rotationPathToUnwrappedQ(rotationPath, q_start)

    count = size(rotationPath, 3);
    qPath = zeros(3, count);
    qPath(:, 1) = q_start;

    for i = 2:count
        qRaw = rotmToQXYZ(rotationPath(:, :, i));
        qPath(:, i) = qPath(:, i-1) + ...
            wrapAngleDifference(qRaw, qPath(:, i-1));
    end
end


function [qSeed, reachedIndex, valid] = ...
    resampleReachableOrientationPrefix(qDense, maxQStep, Nc)

    qSeed = zeros(3, Nc);
    reachedIndex = 1;
    valid = false;

    if size(qDense, 2) < 2
        return;
    end

    currentIndex = 1;
    currentQ = qDense(:, 1);

    for moveIndex = 1:Nc
        if currentIndex >= size(qDense, 2)
            qSeed(:, moveIndex:Nc) = repmat( ...
                currentQ, 1, Nc - moveIndex + 1);
            reachedIndex = currentIndex;
            valid = true;
            return;
        end

        cumulativeTravel = zeros(3, 1);
        chosenIndex = currentIndex;

        for candidateIndex = currentIndex + 1:size(qDense, 2)
            segment = abs(wrapAngleDifference( ...
                qDense(:, candidateIndex), ...
                qDense(:, candidateIndex - 1)));
            trialTravel = cumulativeTravel + segment;

            if all(trialTravel <= maxQStep + 1e-12)
                cumulativeTravel = trialTravel;
                chosenIndex = candidateIndex;
            else
                break;
            end
        end

        if chosenIndex == currentIndex
            return;
        end

        currentIndex = chosenIndex;
        currentQ = qDense(:, currentIndex);
        qSeed(:, moveIndex) = currentQ;
        reachedIndex = currentIndex;
    end

    valid = reachedIndex > 1;
end


function [valid, rejectionCode] = validateInitialSeedRollout( ...
    zSeed, x0, q_des_prev, constraintTolerance, params)

    % Initial guesses may be dynamically infeasible; fmincon is responsible
    % for satisfying theta/omega/alpha constraints. Before using a seed, we
    % require a finite shared rollout, q-command bounds, and all six hard
    % pole distances. Final optimized solutions still undergo the complete
    % independent nonlinear-constraint validation.
    %
    % 0 valid, 201 nonfinite rollout, 202 q-limit failure,
    % 203 pole-distance failure.
    rejectionCode = 0;
    rollout = simulateContinuousMpcTrajectory( ...
        zSeed, x0, q_des_prev, params);
    if ~rollout.finite
        valid = false;
        rejectionCode = 201;
        return;
    end

    qMin = params.mpc.qMin(:);
    qMax = params.mpc.qMax(:);
    qMinHorizon = repmat(qMin, 1, params.mpc.Np);
    qMaxHorizon = repmat(qMax, 1, params.mpc.Np);

    qValid = ...
        all(rollout.Qseq(:) <= qMaxHorizon(:) + constraintTolerance) && ...
        all(rollout.Qseq(:) >= qMinHorizon(:) - constraintTolerance);
    if ~qValid
        valid = false;
        rejectionCode = 202;
        return;
    end

    poleValid = all(rollout.poleDistances(:) >= ...
        params.singularity.dangerDistance - constraintTolerance);
    if ~poleValid
        valid = false;
        rejectionCode = 203;
        return;
    end

    valid = true;
end


function [zPrepared, valid, rejectionCode, ...
    usedBlendScale, preparedConstraintViolation] = ...
    prepareGeometricSeedForSolve( ...
        zGeometric, zShifted, x0, ...
        q_current, q_des_prev, poleIndex, requestedSide, ...
        constraintTolerance, params)

    % 0 accepted, 204 no side-preserving candidate, 205 unusable seed.
    zPrepared = zGeometric;
    valid = false;
    rejectionCode = 205;
    usedBlendScale = NaN;
    preparedConstraintViolation = inf;

    blendScales = readMpcSetting( ...
        params, 'detourSeedBlendScales', ...
        [1.0; 0.75; 0.50; 0.35; 0.20; 0.10]);
    blendScales = blendScales(:);
    blendScales = blendScales(isfinite(blendScales));
    blendScales = min(1, max(0, blendScales));
    blendScales = unique(blendScales, 'stable');

    if isempty(blendScales) || blendScales(1) ~= 1
        blendScales = [1; blendScales];
    end

    allowInfeasibleSeed = logical(readMpcSetting( ...
        params, 'allowInfeasibleDetourSeed', true));
    minimumSideProgress = readMpcSetting( ...
        params, 'detourMinimumSeedSideProgress', deg2rad(0.5));

    bestViolation = inf;
    bestScale = -inf;
    bestCandidate = [];
    sideCandidateFound = false;

    for scaleIndex = 1:numel(blendScales)
        lambda = blendScales(scaleIndex);
        zCandidate = zShifted + lambda * (zGeometric - zShifted);

        [basicValid, basicRejectCode] = validateInitialSeedRollout( ...
            zCandidate, x0, q_des_prev, ...
            constraintTolerance, params);
        if ~basicValid
            rejectionCode = basicRejectCode;
            continue;
        end

        [candidateSide, sideProgress] = inferCommandSeedSide( ...
            zCandidate, q_current, q_des_prev, poleIndex, params);
        if candidateSide ~= requestedSide || ...
                sideProgress < minimumSideProgress
            rejectionCode = 204;
            continue;
        end
        sideCandidateFound = true;

        [cCandidate, ceqCandidate] = continuousMpcConstraints( ...
            zCandidate, x0, q_des_prev, params);
        [constraintFeasible, maximumViolation] = ...
            evaluateConstraintFeasibility( ...
                cCandidate, ceqCandidate, constraintTolerance);

        if constraintFeasible
            zPrepared = zCandidate;
            valid = true;
            rejectionCode = 0;
            usedBlendScale = lambda;
            preparedConstraintViolation = maximumViolation;
            return;
        end

        if maximumViolation < bestViolation - 1e-12 || ...
          (abs(maximumViolation - bestViolation) <= 1e-12 && ...
           lambda > bestScale)
            bestViolation = maximumViolation;
            bestScale = lambda;
            bestCandidate = zCandidate;
        end
    end

    % A mildly infeasible but side-preserving start can still be useful to
    % SQP. Prefer the least-violating blend rather than the raw seed that
    % produced the large exitflag=-2 failures in the first test.
    if allowInfeasibleSeed && ~isempty(bestCandidate)
        zPrepared = bestCandidate;
        valid = true;
        rejectionCode = 0;
        usedBlendScale = bestScale;
        preparedConstraintViolation = bestViolation;
        return;
    end

    if ~sideCandidateFound
        rejectionCode = 204;
    end
end


function [feasible, maximumViolation] = ...
    evaluateConstraintFeasibility(c, ceq, tolerance)

    if isempty(c)
        inequalityViolation = 0;
    else
        inequalityViolation = max([0; c(:)]);
    end

    if isempty(ceq)
        equalityViolation = 0;
    else
        equalityViolation = max(abs(ceq(:)));
    end

    maximumViolation = max(inequalityViolation, equalityViolation);
    feasible = isfinite(maximumViolation) && ...
        maximumViolation <= tolerance;
end


function [side, maximumProgress] = inferCommandSeedSide( ...
    zSeed, q_current, q_des_prev, poleIndex, params)

    side = 0;
    maximumProgress = 0;
    Nc = params.mpc.Nc;

    if ~isequal(size(zSeed), [3 * Nc, 1]) || ...
            poleIndex < 1 || poleIndex > 6
        return;
    end

    bodyAxis = params.singularity.trackedBodyAxis(:);
    poleAxes = params.singularity.poleAxes;
    if numel(bodyAxis) ~= 3 || norm(bodyAxis) < 1e-12 || ...
            ~isequal(size(poleAxes), [3, 6])
        return;
    end
    bodyAxis = bodyAxis / norm(bodyAxis);

    p = poleAxes(:, poleIndex);
    if norm(p) < 1e-12
        return;
    end
    p = p / norm(p);

    v_start = qToRotmXYZ(q_current) * bodyAxis;
    a_start = v_start - dot(v_start, p) * p;
    if norm(a_start) < 1e-12
        return;
    end
    a_start = a_start / norm(a_start);

    b = cross(p, a_start);
    if norm(b) < 1e-12
        return;
    end
    b = b / norm(b);

    deltaQ = reshape(zSeed, 3, Nc);
    qCursor = q_des_prev;
    signedProgress = 0;

    for i = 1:Nc
        qCursor = qCursor + deltaQ(:, i);
        v = qToRotmXYZ(qCursor) * bodyAxis;
        tangent = v - dot(v, p) * p;
        if norm(tangent) < 1e-12
            continue;
        end
        tangent = tangent / norm(tangent);
        phi = atan2(dot(tangent, b), dot(tangent, a_start));

        if abs(phi) > abs(signedProgress)
            signedProgress = phi;
        end
    end

    maximumProgress = abs(signedProgress);
    if maximumProgress > 0
        side = sign(signedProgress);
    end
end


function [side, maximumProgress] = ...
    measureOptimizedDetourProgress( ...
        zCandidate, x0, q_current, q_des_prev, poleIndex, params)

    side = 0;
    maximumProgress = 0;

    [commandSide, commandProgress] = inferCommandSeedSide( ...
        zCandidate, q_current, q_des_prev, poleIndex, params);

    predictedSide = 0;
    predictedProgress = 0;
    rollout = simulateContinuousMpcTrajectory( ...
        zCandidate, x0, q_des_prev, params);

    if rollout.finite && poleIndex >= 1 && poleIndex <= 6
        bodyAxis = params.singularity.trackedBodyAxis(:);
        p = params.singularity.poleAxes(:, poleIndex);

        if numel(bodyAxis) == 3 && norm(bodyAxis) > 1e-12 && ...
                norm(p) > 1e-12
            bodyAxis = bodyAxis / norm(bodyAxis);
            p = p / norm(p);

            v_start = qToRotmXYZ(q_current) * bodyAxis;
            a_start = v_start - dot(v_start, p) * p;

            if norm(a_start) > 1e-12
                a_start = a_start / norm(a_start);
                b = cross(p, a_start);

                if norm(b) > 1e-12
                    b = b / norm(b);
                    previousRawPhi = 0;
                    unwrappedPhi = 0;
                    maximumPositive = 0;
                    minimumNegative = 0;

                    for i = 1:params.mpc.Np
                        v = qToRotmXYZ(rollout.qPred(:, i)) * ...
                            bodyAxis;
                        tangent = v - dot(v, p) * p;
                        if norm(tangent) < 1e-12
                            continue;
                        end

                        tangent = tangent / norm(tangent);
                        rawPhi = atan2( ...
                            dot(tangent, b), ...
                            dot(tangent, a_start));
                        phiIncrement = atan2( ...
                            sin(rawPhi - previousRawPhi), ...
                            cos(rawPhi - previousRawPhi));
                        unwrappedPhi = unwrappedPhi + phiIncrement;
                        previousRawPhi = rawPhi;

                        maximumPositive = max( ...
                            maximumPositive, unwrappedPhi);
                        minimumNegative = min( ...
                            minimumNegative, unwrappedPhi);
                    end

                    if maximumPositive >= abs(minimumNegative)
                        predictedProgress = maximumPositive;
                        if predictedProgress > 0
                            predictedSide = 1;
                        end
                    else
                        predictedProgress = abs(minimumNegative);
                        if predictedProgress > 0
                            predictedSide = -1;
                        end
                    end
                end
            end
        end
    end

    % Use the stronger of command-horizon and physical-rollout evidence.
    % If both are significant but disagree in sign, do not classify the
    % solution as a coherent detour route.
    disagreementThreshold = readMpcSetting( ...
        params, 'detourMinimumOptimizedProgress', deg2rad(0.5));

    if commandProgress >= disagreementThreshold && ...
            predictedProgress >= disagreementThreshold && ...
            commandSide ~= predictedSide
        side = 0;
        maximumProgress = 0;
    elseif predictedProgress > commandProgress
        side = predictedSide;
        maximumProgress = predictedProgress;
    else
        side = commandSide;
        maximumProgress = commandProgress;
    end
end


% =========================================================================
% V5 clearance-aware near-term detour progress
% =========================================================================
function [c, ceq] = routeAwareMpcConstraints( ...
    z, x0, q_current, q_des_prev, poleIndex, requiredSide, params)

    [cBase, ceq] = continuousMpcConstraints( ...
        z, x0, q_des_prev, params);
    [cRoute, routeFinite] = nearTermRouteConstraintValues( ...
        z, x0, q_current, q_des_prev, ...
        poleIndex, requiredSide, params);

    if ~routeFinite
        nearCount = min(max(1, round(readMpcSetting( ...
            params, 'detourNearTermCommandCount', 3))), params.mpc.Np);
        cRoute = 1e6 * ones(nearCount + 2, 1);
    end

    c = [cBase(:); cRoute(:)];
end


function [cRoute, finiteStatus] = nearTermRouteConstraintValues( ...
    z, x0, q_current, q_des_prev, poleIndex, requiredSide, params)

    cRoute = 1e6 * ones(8, 1);
    finiteStatus = false;

    if poleIndex < 1 || poleIndex > 6 || ...
            ~(requiredSide == 1 || requiredSide == -1)
        return;
    end

    rollout = simulateContinuousMpcTrajectory( ...
        z, x0, q_des_prev, params);
    if ~rollout.finite
        return;
    end

    geometry = detourTangentGeometry(q_current, poleIndex, params);
    if ~geometry.valid
        return;
    end

    nearCount = min(max(1, round(readMpcSetting( ...
        params, 'detourNearTermCommandCount', 3))), params.mpc.Np);
    clearanceFloor = readMpcSetting( ...
        params, 'detourProgressMinimumClearance', ...
        readMpcSetting(params, 'detourContinuationClearance', deg2rad(5)));
    firstRequired = readMpcSetting( ...
        params, 'detourFirstTangentialDisplacement', deg2rad(0.02));
    nearRequired = readMpcSetting( ...
        params, 'detourNearTermTangentialDisplacement', deg2rad(0.08));
    outwardRequired = readMpcSetting( ...
        params, 'detourMinimumOutwardProgress', deg2rad(0.25));

    vSequence = zeros(3, nearCount);
    poleDots = zeros(nearCount, 1);
    for k = 1:nearCount
        vSequence(:, k) = qToRotmXYZ(rollout.qPred(:, k)) * ...
            geometry.bodyAxis;
        poleDots(k) = dot(vSequence(:, k), geometry.pole);
    end

    if geometry.currentPoleDistance >= clearanceFloor
        radialConstraints = poleDots - cos(clearanceFloor);
        firstSigned = requiredSide * dot( ...
            vSequence(:, 1) - geometry.vStart, geometry.sideTangent);
        nearSigned = requiredSide * dot( ...
            vSequence(:, nearCount) - geometry.vStart, ...
            geometry.sideTangent);

        cRoute = [ ...
            radialConstraints; ...
            sin(firstRequired) - firstSigned; ...
            sin(nearRequired) - nearSigned];
    else
        % Outward-first mode: while inside the clearance floor, every early
        % predicted plant step must move monotonically away from the pole.
        previousDot = dot(geometry.vStart, geometry.pole);
        monotonicConstraints = zeros(nearCount, 1);
        for k = 1:nearCount
            monotonicConstraints(k) = poleDots(k) - previousDot;
            previousDot = poleDots(k);
        end

        desiredNearDistance = min( ...
            clearanceFloor, ...
            geometry.currentPoleDistance + outwardRequired);
        cRoute = [ ...
            monotonicConstraints; ...
            poleDots(nearCount) - cos(desiredNearDistance); ...
            0];
    end

    finiteStatus = all(isfinite(cRoute));
end


function info = measureClearanceAwareProgress( ...
    zCandidate, x0, q_current, q_des_prev, ...
    poleIndex, requiredSide, params)

    info.eligible = false;
    info.mode = 0;  % 0 none, 1 tangential, 2 outward-first
    info.side = 0;
    info.firstProgress = 0;
    info.nearProgress = 0;
    info.firstPoleDistance = NaN;
    info.nearPoleDistance = NaN;
    info.minimumNearPoleDistance = NaN;

    if poleIndex < 1 || poleIndex > 6
        return;
    end

    rollout = simulateContinuousMpcTrajectory( ...
        zCandidate, x0, q_des_prev, params);
    geometry = detourTangentGeometry(q_current, poleIndex, params);
    if ~rollout.finite || ~geometry.valid
        return;
    end

    nearCount = min(max(1, round(readMpcSetting( ...
        params, 'detourNearTermCommandCount', 3))), params.mpc.Np);
    clearanceFloor = readMpcSetting( ...
        params, 'detourProgressMinimumClearance', ...
        readMpcSetting(params, 'detourContinuationClearance', deg2rad(5)));
    firstRequired = readMpcSetting( ...
        params, 'detourFirstTangentialDisplacement', deg2rad(0.02));
    nearRequired = readMpcSetting( ...
        params, 'detourNearTermTangentialDisplacement', deg2rad(0.08));
    outwardRequired = readMpcSetting( ...
        params, 'detourMinimumOutwardProgress', deg2rad(0.25));
    tolerance = readMpcSetting(params, 'constraintTolerance', 1e-6);

    vSequence = zeros(3, nearCount);
    distanceSequence = zeros(nearCount, 1);
    for k = 1:nearCount
        vSequence(:, k) = qToRotmXYZ(rollout.qPred(:, k)) * ...
            geometry.bodyAxis;
        distanceSequence(k) = acos(clampScalar( ...
            dot(vSequence(:, k), geometry.pole), -1, 1));
    end

    info.firstPoleDistance = distanceSequence(1);
    info.nearPoleDistance = distanceSequence(nearCount);
    info.minimumNearPoleDistance = min(distanceSequence);

    if geometry.currentPoleDistance < clearanceFloor
        info.mode = 2;
        desiredNearDistance = min( ...
            clearanceFloor, ...
            geometry.currentPoleDistance + outwardRequired);
        nondecreasing = all(diff([ ...
            geometry.currentPoleDistance; distanceSequence]) >= -tolerance);
        info.firstProgress = max(0, ...
            distanceSequence(1) - geometry.currentPoleDistance);
        info.nearProgress = max(0, ...
            distanceSequence(nearCount) - geometry.currentPoleDistance);
        if requiredSide == 1 || requiredSide == -1
            info.side = requiredSide;
        end
        info.eligible = nondecreasing && ...
            distanceSequence(nearCount) >= desiredNearDistance - tolerance;
        return;
    end

    info.mode = 1;
    rawFirst = dot( ...
        vSequence(:, 1) - geometry.vStart, geometry.sideTangent);
    rawNear = dot( ...
        vSequence(:, nearCount) - geometry.vStart, geometry.sideTangent);

    if requiredSide == 1 || requiredSide == -1
        candidateSide = requiredSide;
    elseif abs(rawNear) > 1e-12
        candidateSide = sign(rawNear);
    else
        candidateSide = 0;
    end

    info.side = candidateSide;
    if candidateSide == 0
        return;
    end

    firstSigned = candidateSide * rawFirst;
    nearSigned = candidateSide * rawNear;
    info.firstProgress = asin(clampScalar(firstSigned, -1, 1));
    info.nearProgress = asin(clampScalar(nearSigned, -1, 1));

    [firstTransitionClearanceSafe, ~] = ...
        validateThetaTransitionPoleSafety( ...
            x0(1:4), rollout.thetaPred(:, 1), ...
            clearanceFloor, params);
    clearanceSatisfied = ...
        info.minimumNearPoleDistance >= clearanceFloor - tolerance && ...
        firstTransitionClearanceSafe;
    info.eligible = clearanceSatisfied && ...
        firstSigned >= sin(firstRequired) - tolerance && ...
        nearSigned >= sin(nearRequired) - tolerance;
end


function geometry = detourTangentGeometry(q_current, poleIndex, params)

    geometry.valid = false;
    geometry.bodyAxis = NaN(3, 1);
    geometry.pole = NaN(3, 1);
    geometry.vStart = NaN(3, 1);
    geometry.radial = NaN(3, 1);
    geometry.sideTangent = NaN(3, 1);
    geometry.currentPoleDistance = NaN;

    if poleIndex < 1 || poleIndex > 6
        return;
    end

    bodyAxis = params.singularity.trackedBodyAxis(:);
    pole = params.singularity.poleAxes(:, poleIndex);
    if numel(bodyAxis) ~= 3 || norm(bodyAxis) < 1e-12 || ...
            norm(pole) < 1e-12
        return;
    end

    bodyAxis = bodyAxis / norm(bodyAxis);
    pole = pole / norm(pole);
    vStart = qToRotmXYZ(q_current) * bodyAxis;
    radial = vStart - dot(vStart, pole) * pole;
    if norm(radial) < 1e-10
        return;
    end
    radial = radial / norm(radial);
    sideTangent = cross(pole, radial);
    if norm(sideTangent) < 1e-10
        return;
    end
    sideTangent = sideTangent / norm(sideTangent);

    geometry.valid = true;
    geometry.bodyAxis = bodyAxis;
    geometry.pole = pole;
    geometry.vStart = vStart;
    geometry.radial = radial;
    geometry.sideTangent = sideTangent;
    geometry.currentPoleDistance = acos(clampScalar( ...
        dot(vStart, pole), -1, 1));
end


% =========================================================================
% Multi-start fmincon solve manager
% =========================================================================
function result = runMpcMultiStart( ...
    Z0, startEnabled, preRejectCode, numberOfStarts, ...
    startType, startSide, ...
    objectiveFunction, constraintFunction, options, ...
    lb, ub, x0, q_ref, q_current, q_des_prev, ...
    qMin, qMax, maxQStep, constraintTolerance, ...
    solverAvailable, routeBlocked, detourActive, ...
    selectedDetourPole, committedDetourSide, ...
    detourMinimumOptimizedProgress, params) %#ok<INUSD>

    maxStarts = size(Z0, 2);
    result.accepted = false;
    result.bestZ = [];
    result.bestIndex = 0;
    result.bestObjective = inf;
    result.bestMinimumPoleDistance = -inf;
    result.bestRouteEligible = false;
    result.bestProgressSide = 0;
    result.bestProgress = 0;
    result.bestProgressMode = 0;
    result.bestFirstProgress = 0;
    result.bestFirstPoleDistance = NaN;
    result.bestNearPoleDistance = NaN;
    result.bestMinimumNearPoleDistance = NaN;
    result.numberOfSolves = 0;
    result.numberAccepted = 0;
    result.numberRouteEligible = 0;
    result.totalSolveTime = 0;
    result.worstSolveTime = 0;

    result.exitflag = NaN(maxStarts, 1);
    result.acceptedStart = false(maxStarts, 1);
    result.routeEligible = false(maxStarts, 1);
    result.progressMode = zeros(maxStarts, 1);
    result.progressSide = zeros(maxStarts, 1);
    result.firstProgress = zeros(maxStarts, 1);
    result.progress = zeros(maxStarts, 1);
    result.firstPoleDistance = NaN(maxStarts, 1);
    result.nearPoleDistance = NaN(maxStarts, 1);
    result.minimumNearPoleDistance = NaN(maxStarts, 1);
    result.objective = NaN(maxStarts, 1);
    result.minimumPoleDistance = NaN(maxStarts, 1);
    result.iterations = zeros(maxStarts, 1);
    result.functionEvaluations = zeros(maxStarts, 1);
    result.solveTime = zeros(maxStarts, 1);
    result.rejectionCode = zeros(maxStarts, 1);

    result.reportExitflag = NaN;
    result.reportObjective = NaN;
    result.reportIterations = 0;
    result.reportFunctionEvaluations = 0;
    result.reportMessage = "";
    result.reportValidation = emptyValidationResult();

    zByStart = cell(maxStarts, 1);
    validationByStart = cell(maxStarts, 1);
    messageByStart = strings(maxStarts, 1);

    routeMode = ...
        selectedDetourPole >= 1 && ...
        (routeBlocked || detourActive);

    if ~solverAvailable
        for i = 1:numberOfStarts
            if startEnabled(i)
                result.rejectionCode(i) = 11;
            else
                result.rejectionCode(i) = preRejectCode(i);
            end
        end
        result.reportMessage = ...
            "fmincon is unavailable. Optimization Toolbox was not found.";
        return;
    end

    objectiveTieTolerance = readMpcSetting( ...
        params, 'detourObjectiveTieTolerance', 1e-6);

    bestAnyIndex = 0;
    bestAnyObjective = inf;
    bestAnyMinimumPoleDistance = -inf;

    bestRouteIndex = 0;
    bestRouteObjective = inf;
    bestRouteMinimumPoleDistance = -inf;

    firstSolvedIndex = 0;

    for i = 1:numberOfStarts
        if ~startEnabled(i)
            result.rejectionCode(i) = preRejectCode(i);
            continue;
        end

        requiredSide = 0;
        if detourActive
            requiredSide = committedDetourSide;
        elseif startType(i) == 1 || startType(i) == -1
            requiredSide = startSide(i);
        end

        thisConstraintFunction = constraintFunction;
        if routeMode && (requiredSide == 1 || requiredSide == -1)
            thisConstraintFunction = @(z) routeAwareMpcConstraints( ...
                z, x0, q_current, q_des_prev, ...
                selectedDetourPole, requiredSide, params);
        end

        result.numberOfSolves = result.numberOfSolves + 1;
        solveTimer = tic;

        try
            [zCandidate, fval, exitflag, output] = fmincon( ...
                objectiveFunction, Z0(:, i), ...
                [], [], [], [], lb, ub, ...
                thisConstraintFunction, options);
            elapsed = toc(solveTimer);

            if firstSolvedIndex == 0
                firstSolvedIndex = i;
            end

            zByStart{i} = zCandidate;
            result.solveTime(i) = elapsed;
            result.totalSolveTime = result.totalSolveTime + elapsed;
            result.worstSolveTime = max(result.worstSolveTime, elapsed);
            result.exitflag(i) = exitflag;
            result.objective(i) = fval;

            if isfield(output, 'iterations')
                result.iterations(i) = output.iterations;
            end
            if isfield(output, 'funcCount')
                result.functionEvaluations(i) = output.funcCount;
            end
            if isfield(output, 'message')
                messageByStart(i) = string(output.message);
            end

            [accepted, validation] = validateOptimizedSolution( ...
                zCandidate, fval, exitflag, ...
                x0, q_ref, q_des_prev, ...
                qMin, qMax, maxQStep, ...
                constraintTolerance, params);
            validationByStart{i} = validation;

            optimizedRollout = simulateContinuousMpcTrajectory( ...
                zCandidate, x0, q_des_prev, params);
            if optimizedRollout.finite
                result.minimumPoleDistance(i) = min( ...
                    optimizedRollout.poleDistances, [], 'all');
            end

            result.acceptedStart(i) = accepted;
            result.rejectionCode(i) = classifyValidationRejection( ...
                accepted, exitflag, validation);

            if accepted
                result.numberAccepted = result.numberAccepted + 1;

                progressInfo = measureClearanceAwareProgress( ...
                    zCandidate, x0, q_current, q_des_prev, ...
                    selectedDetourPole, requiredSide, params);

                result.progressMode(i) = progressInfo.mode;
                result.progressSide(i) = progressInfo.side;
                result.firstProgress(i) = progressInfo.firstProgress;
                result.progress(i) = progressInfo.nearProgress;
                result.firstPoleDistance(i) = ...
                    progressInfo.firstPoleDistance;
                result.nearPoleDistance(i) = ...
                    progressInfo.nearPoleDistance;
                result.minimumNearPoleDistance(i) = ...
                    progressInfo.minimumNearPoleDistance;

                routeEligible = ~routeMode || progressInfo.eligible;
                result.routeEligible(i) = routeEligible;
                if routeEligible && routeMode
                    result.numberRouteEligible = ...
                        result.numberRouteEligible + 1;
                end

                comparisonTolerance = objectiveTieTolerance * max( ...
                    [1, abs(bestAnyObjective), abs(fval)]);
                chooseAny = ...
                    bestAnyIndex == 0 || ...
                    fval < bestAnyObjective - comparisonTolerance || ...
                    (abs(fval - bestAnyObjective) <= ...
                        comparisonTolerance && ...
                     result.minimumPoleDistance(i) > ...
                        bestAnyMinimumPoleDistance);

                if chooseAny
                    bestAnyIndex = i;
                    bestAnyObjective = fval;
                    bestAnyMinimumPoleDistance = ...
                        result.minimumPoleDistance(i);
                end

                if routeEligible
                    comparisonTolerance = objectiveTieTolerance * max( ...
                        [1, abs(bestRouteObjective), abs(fval)]);
                    chooseRoute = ...
                        bestRouteIndex == 0 || ...
                        fval < bestRouteObjective - ...
                            comparisonTolerance || ...
                        (abs(fval - bestRouteObjective) <= ...
                            comparisonTolerance && ...
                         result.minimumPoleDistance(i) > ...
                            bestRouteMinimumPoleDistance);

                    if chooseRoute
                        bestRouteIndex = i;
                        bestRouteObjective = fval;
                        bestRouteMinimumPoleDistance = ...
                            result.minimumPoleDistance(i);
                    end
                end
            end

        catch solverException
            elapsed = toc(solveTimer);
            result.solveTime(i) = elapsed;
            result.totalSolveTime = result.totalSolveTime + elapsed;
            result.worstSolveTime = max(result.worstSolveTime, elapsed);
            result.rejectionCode(i) = 4;
            messageByStart(i) = string(solverException.message);
            if firstSolvedIndex == 0
                firstSolvedIndex = i;
            end
        end
    end

    % V5 does not apply a validated-but-stalling optimizer result while a
    % route is blocked. If no candidate makes safe physical near-term
    % progress, the caller uses a separately validated route-aware fallback.
    if routeMode
        chosenIndex = bestRouteIndex;
    else
        chosenIndex = bestAnyIndex;
    end

    if chosenIndex > 0
        result.accepted = true;
        result.bestIndex = chosenIndex;
        result.bestZ = zByStart{chosenIndex};
        result.bestObjective = result.objective(chosenIndex);
        result.bestMinimumPoleDistance = ...
            result.minimumPoleDistance(chosenIndex);
        result.bestRouteEligible = ...
            result.routeEligible(chosenIndex);
        result.bestProgressSide = ...
            result.progressSide(chosenIndex);
        result.bestProgress = result.progress(chosenIndex);
        result.bestProgressMode = result.progressMode(chosenIndex);
        result.bestFirstProgress = result.firstProgress(chosenIndex);
        result.bestFirstPoleDistance = ...
            result.firstPoleDistance(chosenIndex);
        result.bestNearPoleDistance = ...
            result.nearPoleDistance(chosenIndex);
        result.bestMinimumNearPoleDistance = ...
            result.minimumNearPoleDistance(chosenIndex);

        result.reportValidation = validationByStart{chosenIndex};
        result.reportExitflag = result.exitflag(chosenIndex);
        result.reportObjective = result.objective(chosenIndex);
        result.reportIterations = result.iterations(chosenIndex);
        result.reportFunctionEvaluations = ...
            result.functionEvaluations(chosenIndex);
        result.reportMessage = messageByStart(chosenIndex);

    elseif firstSolvedIndex > 0
        result.reportExitflag = result.exitflag(firstSolvedIndex);
        result.reportObjective = result.objective(firstSolvedIndex);
        result.reportIterations = result.iterations(firstSolvedIndex);
        result.reportFunctionEvaluations = ...
            result.functionEvaluations(firstSolvedIndex);
        result.reportMessage = messageByStart(firstSolvedIndex);
        if ~isempty(validationByStart{firstSolvedIndex})
            result.reportValidation = ...
                validationByStart{firstSolvedIndex};
        end
    end
end


function code = classifyValidationRejection(accepted, exitflag, validation)

    if accepted
        code = 0;
    elseif exitflag < 0
        code = 5;
    elseif ~validation.solutionFinite
        code = 6;
    elseif ~validation.objectiveFinite
        code = 7;
    elseif ~validation.rolloutFinite
        code = 8;
    elseif ~validation.constraintsSatisfied
        code = 9;
    elseif ~validation.firstCommandValid
        code = 10;
    elseif ~validation.firstTransitionSafe
        code = 13;
    else
        code = 12;
    end
end


function side = inferDetourSideFromSolution( ...
    zOptimal, x0, q_current, q_des_prev, poleIndex, params)

    side = 0;
    rollout = simulateContinuousMpcTrajectory( ...
        zOptimal, x0, q_des_prev, params);
    if ~rollout.finite || poleIndex < 1 || poleIndex > 6
        return;
    end

    bodyAxis = params.singularity.trackedBodyAxis(:);
    bodyAxis = bodyAxis / norm(bodyAxis);
    p = params.singularity.poleAxes(:, poleIndex);
    p = p / norm(p);

    R_current = qToRotmXYZ(q_current);
    v_start = R_current * bodyAxis;
    a_start_raw = v_start - dot(v_start, p) * p;
    if norm(a_start_raw) < 1e-10
        return;
    end
    a_start = a_start_raw / norm(a_start_raw);
    b = cross(p, a_start);
    if norm(b) < 1e-10
        return;
    end
    b = b / norm(b);

    for i = 1:params.mpc.Np
        R = qToRotmXYZ(rollout.qPred(:, i));
        v = R * bodyAxis;
        tangent = v - dot(v, p) * p;
        if norm(tangent) < 1e-10
            continue;
        end
        tangent = tangent / norm(tangent);
        phi = atan2(dot(tangent, b), dot(tangent, a_start));
        if abs(phi) > deg2rad(0.25)
            side = sign(phi);
            return;
        end
    end
end


% =========================================================================
% Rotation utilities used only for seed construction
% =========================================================================
function R = qToRotmXYZ(q)

    r = q(1); p = q(2); y = q(3);
    cr = cos(r); sr = sin(r);
    cp = cos(p); sp = sin(p);
    cy = cos(y); sy = sin(y);

    Rx = [1, 0, 0; 0, cr, -sr; 0, sr, cr];
    Ry = [cp, 0, sp; 0, 1, 0; -sp, 0, cp];
    Rz = [cy, -sy, 0; sy, cy, 0; 0, 0, 1];
    R = Rx * Ry * Rz;
end


function q = rotmToQXYZ(R)

    pitch = asin(clampScalar(R(1, 3), -1, 1));
    cp = cos(pitch);
    if abs(cp) > 1e-12
        roll = atan2(-R(2, 3), R(3, 3));
        yaw = atan2(-R(1, 2), R(1, 1));
    else
        roll = 0;
        yaw = atan2(R(2, 1), R(2, 2));
    end
    q = [roll; pitch; yaw];
end


function R = axisAngleToRotm(axis, angle)

    axis = axis / norm(axis);
    K = [0, -axis(3), axis(2); ...
         axis(3), 0, -axis(1); ...
        -axis(2), axis(1), 0];
    R = eye(3) + sin(angle) * K + (1 - cos(angle)) * (K * K);
end


function R = projectToRotationMatrix(Rraw)

    [U, ~, V] = svd(Rraw);
    R = U * diag([1, 1, det(U * V')]) * V';
end


function quaternion = rotmToQuaternion(R)

    traceValue = trace(R);
    quaternion = zeros(4, 1);

    if traceValue > 0
        s = sqrt(traceValue + 1) * 2;
        quaternion(1) = 0.25 * s;
        quaternion(2) = (R(3, 2) - R(2, 3)) / s;
        quaternion(3) = (R(1, 3) - R(3, 1)) / s;
        quaternion(4) = (R(2, 1) - R(1, 2)) / s;
    elseif R(1, 1) > R(2, 2) && R(1, 1) > R(3, 3)
        s = sqrt(1 + R(1, 1) - R(2, 2) - R(3, 3)) * 2;
        quaternion(1) = (R(3, 2) - R(2, 3)) / s;
        quaternion(2) = 0.25 * s;
        quaternion(3) = (R(1, 2) + R(2, 1)) / s;
        quaternion(4) = (R(1, 3) + R(3, 1)) / s;
    elseif R(2, 2) > R(3, 3)
        s = sqrt(1 + R(2, 2) - R(1, 1) - R(3, 3)) * 2;
        quaternion(1) = (R(1, 3) - R(3, 1)) / s;
        quaternion(2) = (R(1, 2) + R(2, 1)) / s;
        quaternion(3) = 0.25 * s;
        quaternion(4) = (R(2, 3) + R(3, 2)) / s;
    else
        s = sqrt(1 + R(3, 3) - R(1, 1) - R(2, 2)) * 2;
        quaternion(1) = (R(2, 1) - R(1, 2)) / s;
        quaternion(2) = (R(1, 3) + R(3, 1)) / s;
        quaternion(3) = (R(2, 3) + R(3, 2)) / s;
        quaternion(4) = 0.25 * s;
    end

    quaternion = quaternion / norm(quaternion);
end


function R = quaternionToRotm(q)

    q = q / norm(q);
    w = q(1); x = q(2); y = q(3); z = q(4);
    R = [ ...
        1 - 2 * (y^2 + z^2), 2 * (x*y - z*w), 2 * (x*z + y*w); ...
        2 * (x*y + z*w), 1 - 2 * (x^2 + z^2), 2 * (y*z - x*w); ...
        2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x^2 + y^2)];
end


function q = quaternionSlerp(q0, q1, lambda)

    q0 = q0 / norm(q0);
    q1 = q1 / norm(q1);
    cosine = dot(q0, q1);

    if cosine < 0
        q1 = -q1;
        cosine = -cosine;
    end
    cosine = clampScalar(cosine, -1, 1);

    if cosine > 0.9995
        q = (1 - lambda) * q0 + lambda * q1;
        q = q / norm(q);
        return;
    end

    angle = acos(cosine);
    q = (sin((1 - lambda) * angle) * q0 + ...
         sin(lambda * angle) * q1) / sin(angle);
    q = q / norm(q);
end


function unitVector = normalizeVector(vector, tolerance)

    if norm(vector) < tolerance
        unitVector = NaN(size(vector));
    else
        unitVector = vector / norm(vector);
    end
end


function value = clampScalar(value, lower, upper)

    value = min(upper, max(lower, value));
end
% =========================================================================
% Continuous MPC objective
% =========================================================================
function J = continuousMpcObjective(z, x0, q_ref, q_des_prev, params)

    rollout = simulateContinuousMpcTrajectory( ...
        z, x0, q_des_prev, params);

    if ~rollout.finite
        J = 1e20;
        return;
    end

    Np = params.mpc.Np;

    wTrack = params.mpc.wTrack;
    wTerminal = params.mpc.wTerminal;
    wSmooth = params.mpc.wSmooth;
    wMotor = params.mpc.wMotor;
    wSingularity = params.mpc.wSingularity;
    wOmega = params.mpc.wOmega;

    warningDistance = params.singularity.warningDistance;

    J = 0;

    try
        theta_cmd_prev = abenicsIK(q_des_prev, params);
        theta_cmd_prev = theta_cmd_prev(:);
    catch
        J = 1e20;
        return;
    end

    q_des_previous_i = q_des_prev;

    for i = 1:Np
        q_des_i = rollout.Qseq(:, i);
        q_pred_i = rollout.qPred(:, i);
        theta_cmd_i = rollout.thetaCmd(:, i);
        omega_pred_i = rollout.omegaPred(:, i);
        s_i = rollout.singularityDistance(i);

        % 1. Predicted orientation tracking cost
        q_error_i = wrapAngleDifference(q_pred_i, q_ref);
        J = J + wTrack * sum(q_error_i.^2);

        % 2. Command-increment smoothness cost. This is zero after move
        % blocking begins because Qseq holds its final command.
        delta_q_i = wrapAngleDifference(q_des_i, q_des_previous_i);
        J = J + wSmooth * sum(delta_q_i.^2);

        % 3. IK motor-command motion cost
        delta_theta_cmd_i = wrapAngleDifference( ...
            theta_cmd_i, theta_cmd_prev);
        J = J + wMotor * sum(delta_theta_cmd_i.^2);

        % 4. Predicted MP-gear velocity cost
        J = J + wOmega * sum(omega_pred_i.^2);

        % 5. Warning-region singularity penalty. It remains active from
        % recoveryClearDistance up through warningDistance during normal MPC.
        effectiveWarningDistance = warningDistance;

if isfield(params.mpc, ...
        'singularTargetOverrideActive') && ...
        params.mpc.singularTargetOverrideActive

    % Gradually reduce the warning threshold as the horizon approaches
    % the explicitly requested unsafe target.
    lambda = i / Np;

    targetWarningDistance = min( ...
        warningDistance, ...
        params.mpc.targetSingularityDistance);

    effectiveWarningDistance = ...
        (1 - lambda) * warningDistance + ...
        lambda * targetWarningDistance;
end

warningDeficit = max( ...
    0, effectiveWarningDistance - s_i);

J = J + ...
    wSingularity * warningDeficit^2;

        q_des_previous_i = q_des_i;
        theta_cmd_prev = theta_cmd_i;
    end

    q_terminal_error = wrapAngleDifference( ...
        rollout.qPred(:, Np), q_ref);

    J = J + wTerminal * sum(q_terminal_error.^2);

    if ~isfinite(J)
        J = 1e20;
    end
end

% =========================================================================
% Continuous MPC nonlinear constraints
% =========================================================================
function [c, ceq] = continuousMpcConstraints( ...
    z, x0, q_des_prev, params)

    rollout = simulateContinuousMpcTrajectory( ...
        z, x0, q_des_prev, params);

    Np = params.mpc.Np;
    numConstraints = 44 * Np;

    if ~rollout.finite
        c = 1e6 * ones(numConstraints, 1);
        ceq = [];
        return;
    end

    qMin = params.mpc.qMin(:);
    qMax = params.mpc.qMax(:);
    thetaMin = params.mpc.thetaMin(:);
    thetaMax = params.mpc.thetaMax(:);
    omegaMax = expandToVector(params.mpc.omegaMax, 4);
    alphaMax = expandToVector(params.mpc.alphaMax, 4);
    dangerDistance = params.singularity.dangerDistance;

    qMinHorizon = repmat(qMin, 1, Np);
    qMaxHorizon = repmat(qMax, 1, Np);
    thetaMinHorizon = repmat(thetaMin, 1, Np);
    thetaMaxHorizon = repmat(thetaMax, 1, Np);
    omegaMaxHorizon = repmat(omegaMax, 1, Np);
    alphaMaxHorizon = repmat(alphaMax, 1, Np);

    requiredPoleDistances = ...
        dangerDistance * ones(6, Np);

    if isfield(params.mpc, ...
            'singularTargetOverrideActive') && ...
            params.mpc.singularTargetOverrideActive && ...
            params.mpc.targetSingularityDistance < dangerDistance

        currentPoleDistances = ...
            params.mpc.currentPoleDistances(:);

        targetPoleDistances = ...
            params.mpc.targetPoleDistances(:);

        % If already inside danger, begin from the current measured distance.
        % Otherwise begin with the normal dangerDistance requirement.
        startingRequirement = min( ...
            currentPoleDistances, ...
            dangerDistance * ones(6, 1));

        % Only poles that are unsafe at the target are relaxed.
        targetRequirement = min( ...
            targetPoleDistances, ...
            dangerDistance * ones(6, 1));

        for i = 1:Np
            lambda = i / Np;

            requiredPoleDistances(:, i) = ...
                (1 - lambda) * startingRequirement + ...
                lambda * targetRequirement;
        end
    end

    % fmincon inequality convention: c <= 0.
    % Positive and negative constraints are kept separate. No abs() is used.
    c = [ ...
        rollout.Qseq - qMaxHorizon; ...
        qMinHorizon - rollout.Qseq; ...
        rollout.thetaCmd - thetaMaxHorizon; ...
        thetaMinHorizon - rollout.thetaCmd; ...
        rollout.thetaPred - thetaMaxHorizon; ...
        thetaMinHorizon - rollout.thetaPred; ...
        rollout.omegaPred - omegaMaxHorizon; ...
       -omegaMaxHorizon - rollout.omegaPred; ...
        rollout.alphaPred - alphaMaxHorizon; ...
       -alphaMaxHorizon - rollout.alphaPred; ...
        requiredPoleDistances - rollout.poleDistances];

    c = c(:);
    ceq = [];

    if any(~isfinite(c))
        c = 1e6 * ones(numConstraints, 1);
    end
end

% =========================================================================
% Shared direct-shooting rollout used by objective and constraints
% =========================================================================
function rollout = simulateContinuousMpcTrajectory( ...
    z, x0, q_des_prev, params)

    Np = params.mpc.Np;
    Nc = params.mpc.Nc;

    rollout.deltaQ = NaN(3, Nc);
    rollout.Qseq = NaN(3, Np);
    rollout.thetaCmd = NaN(4, Np);
    rollout.thetaPred = NaN(4, Np);
    rollout.omegaPred = NaN(4, Np);
    rollout.alphaPred = NaN(4, Np);
    rollout.qPred = NaN(3, Np);
    rollout.singularityDistance = NaN(1, Np);
    rollout.poleDistances = NaN(6, Np);
    rollout.finite = false;

    if ~isequal(size(z), [3 * Nc, 1]) || ...
            ~isequal(size(x0), [8, 1]) || ...
            ~isequal(size(q_des_prev), [3, 1]) || ...
            any(~isfinite(z)) || ...
            any(~isfinite(x0)) || ...
            any(~isfinite(q_des_prev))
        return;
    end

    % Required cumulative command reconstruction. Do not clamp Qseq here.
    deltaQ = reshape(z, 3, Nc);
    Qseq = zeros(3, Np);
    q_current = q_des_prev;

    for i = 1:Np
        if i <= Nc
            q_current = q_current + deltaQ(:, i);
        end

        Qseq(:, i) = q_current;
    end

    rollout.deltaQ = deltaQ;
    rollout.Qseq = Qseq;

    x_pred = x0;

    try
        for i = 1:Np
            q_des_i = Qseq(:, i);

            theta_cmd_i = abenicsIK(q_des_i, params);
            theta_cmd_i = theta_cmd_i(:);

            [x_pred, alpha_i] = abenicsPlantStepLocal( ...
                x_pred, theta_cmd_i, params);

            theta_pred_i = x_pred(1:4);
            omega_pred_i = x_pred(5:8);

            q_pred_i = abenicsFK(theta_pred_i, params);
            q_pred_i = q_pred_i(:);

            [s_i, singularityInfo_i] = singularityMeasure( ...
                theta_pred_i, q_pred_i, params);

            poleDistances_i = singularityInfo_i.poleDistances(:);

            if ~isequal(size(theta_cmd_i), [4, 1]) || ...
               ~isequal(size(theta_pred_i), [4, 1]) || ...
               ~isequal(size(omega_pred_i), [4, 1]) || ...
               ~isequal(size(alpha_i), [4, 1]) || ...
               ~isequal(size(q_pred_i), [3, 1]) || ...
               ~isequal(size(poleDistances_i), [6, 1])
                return;
            end

            rollout.thetaCmd(:, i) = theta_cmd_i;
            rollout.thetaPred(:, i) = theta_pred_i;
            rollout.omegaPred(:, i) = omega_pred_i;
            rollout.alphaPred(:, i) = alpha_i;
            rollout.qPred(:, i) = q_pred_i;
            rollout.singularityDistance(i) = s_i;
            rollout.poleDistances(:, i) = poleDistances_i;
        end
    catch
        return;
    end

    rollout.finite = ...
        all(isfinite(rollout.deltaQ(:))) && ...
        all(isfinite(rollout.Qseq(:))) && ...
        all(isfinite(rollout.thetaCmd(:))) && ...
        all(isfinite(rollout.thetaPred(:))) && ...
        all(isfinite(rollout.omegaPred(:))) && ...
        all(isfinite(rollout.alphaPred(:))) && ...
        all(isfinite(rollout.qPred(:))) && ...
        all(isfinite(rollout.singularityDistance(:))) && ...
        all(isfinite(rollout.poleDistances(:)));
end

% =========================================================================
% Validate a returned fmincon solution before accepting it
% =========================================================================
function [valid, result] = validateOptimizedSolution( ...
    zOptimal, fval, exitflag, ...
    x0, q_ref, q_des_prev, ...
    qMin, qMax, maxQStep, ...
    constraintTolerance, params)

    Nc = params.mpc.Nc;

    result.maxConstraintIndex = NaN;
    result.maxConstraintStep = NaN;
    result.maxConstraintName = "none";

    result.solutionFinite = false;
    result.objectiveFinite = false;
    result.constraintsSatisfied = false;
    result.firstCommandValid = false;
    result.firstTransitionSafe = false;
    result.minimumFirstTransitionPoleDistance = NaN;
    result.rolloutFinite = false;
    result.maxConstraintViolation = inf;

    if ~isequal(size(zOptimal), [3 * Nc, 1])
        valid = false;
        return;
    end

    result.solutionFinite = all(isfinite(zOptimal));
    result.objectiveFinite = isfinite(fval);

    if ~result.solutionFinite || ~result.objectiveFinite
        valid = false;
        return;
    end

    rollout = simulateContinuousMpcTrajectory( ...
        zOptimal, x0, q_des_prev, params);
    result.rolloutFinite = rollout.finite;

    if ~rollout.finite
        valid = false;
        return;
    end

    objectiveCheck = continuousMpcObjective( ...
        zOptimal, x0, q_ref, q_des_prev, params);
    result.objectiveFinite = ...
        result.objectiveFinite && isfinite(objectiveCheck);

    [c, ceq] = continuousMpcConstraints( ...
        zOptimal, x0, q_des_prev, params);

    result.maxConstraintIndex = NaN;
    result.maxConstraintStep = NaN;
    result.maxConstraintName = "none";
    
    if ~isempty(c)
    
        [maximumConstraintValue, maxIndex] = max(c);
    
        % Only label it as a failed constraint when it exceeds tolerance.
        if maximumConstraintValue > constraintTolerance
    
            result.maxConstraintIndex = maxIndex;
            result.maxConstraintStep = ceil(maxIndex / 44);
    
            constraintWithinStep = mod(maxIndex - 1, 44) + 1;
    
            if constraintWithinStep <= 3
                result.maxConstraintName = "q upper";
    
            elseif constraintWithinStep <= 6
                result.maxConstraintName = "q lower";
    
            elseif constraintWithinStep <= 10
                result.maxConstraintName = "theta command upper";
    
            elseif constraintWithinStep <= 14
                result.maxConstraintName = "theta command lower";
    
            elseif constraintWithinStep <= 18
                result.maxConstraintName = "theta predicted upper";
    
            elseif constraintWithinStep <= 22
                result.maxConstraintName = "theta predicted lower";
    
            elseif constraintWithinStep <= 26
                result.maxConstraintName = "omega upper";
    
            elseif constraintWithinStep <= 30
                result.maxConstraintName = "omega lower";
    
            elseif constraintWithinStep <= 34
                result.maxConstraintName = "alpha upper";
    
            elseif constraintWithinStep <= 38
                result.maxConstraintName = "alpha lower";
    
            else
                result.maxConstraintName = "pole distance";
            end
        end
    end

    if isempty(c)
        inequalityViolation = 0;
    else
        inequalityViolation = max([0; c(:)]);
    end

    if isempty(ceq)
        equalityViolation = 0;
    else
        equalityViolation = max(abs(ceq(:)));
    end

    result.maxConstraintViolation = max( ...
        inequalityViolation, equalityViolation);

    result.constraintsSatisfied = ...
        isfinite(result.maxConstraintViolation) && ...
        result.maxConstraintViolation <= constraintTolerance;

    deltaQOptimal = reshape(zOptimal, 3, Nc);
    firstCommand = q_des_prev + deltaQOptimal(:, 1);
    firstStep = wrapAngleDifference(firstCommand, q_des_prev);

    result.firstCommandValid = ...
        isequal(size(firstCommand), [3, 1]) && ...
        all(isfinite(firstCommand)) && ...
        all(firstCommand <= qMax + constraintTolerance) && ...
        all(firstCommand >= qMin - constraintTolerance) && ...
        all(firstStep <= maxQStep + constraintTolerance) && ...
        all(firstStep >= -maxQStep - constraintTolerance);

    [result.firstTransitionSafe, transitionInfo] = ...
        validateThetaTransitionPoleSafety( ...
            x0(1:4), rollout.thetaPred(:, 1), ...
            params.singularity.dangerDistance, params);
    result.minimumFirstTransitionPoleDistance = ...
        transitionInfo.minimumPoleDistance;

    % exitflag = 0 means fmincon stopped because it reached an iteration or
    % function-evaluation limit. Accept it only when the returned solution
    % independently passes every finite-value, rollout, command, and
    % constraint check.
    solverStatusAcceptable = exitflag >= 0;
    
    valid = ...
        solverStatusAcceptable && ...
        result.solutionFinite && ...
        result.objectiveFinite && ...
        result.rolloutFinite && ...
        result.constraintsSatisfied && ...
        result.firstCommandValid && ...
        result.firstTransitionSafe;
end

% =========================================================================
% Validate the bounded direct fallback command
% =========================================================================
function [valid, result] = validateOneStepCommand( ...
    q_candidate, q_des_prev, x0, ...
    qMin, qMax, thetaMin, thetaMax, ...
    omegaMax, alphaMax, maxQStep, ...
    dangerDistance, params)

    violations = [];
    result.maxViolation = inf;
    result.minimumTransitionPoleDistance = NaN;
    result.qNext = NaN(3, 1);
    result.thetaNext = NaN(4, 1);
    result.poleDistances = NaN(6, 1);

    if ~isequal(size(q_candidate), [3, 1]) || ...
            any(~isfinite(q_candidate))
        valid = false;
        return;
    end

    q_step = wrapAngleDifference(q_candidate, q_des_prev);

    violations = [violations; ...
        q_candidate - qMax; ...
        qMin - q_candidate; ...
        q_step - maxQStep; ...
       -maxQStep - q_step]; %#ok<AGROW>

    try
        theta_cmd = abenicsIK(q_candidate, params);
        theta_cmd = theta_cmd(:);

        [x_next, alpha_next] = abenicsPlantStepLocal( ...
            x0, theta_cmd, params);

        theta_next = x_next(1:4);
        omega_next = x_next(5:8);

        q_next = abenicsFK(theta_next, params);
        q_next = q_next(:);

        [~, singularityInfo] = singularityMeasure( ...
            theta_next, q_next, params);

        poleDistances = singularityInfo.poleDistances(:);

        [transitionSafe, transitionInfo] = ...
            validateThetaTransitionPoleSafety( ...
                x0(1:4), theta_next, dangerDistance, params);

        violations = [violations; ...
            theta_cmd - thetaMax; ...
            thetaMin - theta_cmd; ...
            theta_next - thetaMax; ...
            thetaMin - theta_next; ...
            omega_next - omegaMax; ...
           -omegaMax - omega_next; ...
            alpha_next - alphaMax; ...
           -alphaMax - alpha_next; ...
            dangerDistance - poleDistances]; %#ok<AGROW>

        finiteStatus = ...
            all(isfinite(theta_cmd)) && ...
            all(isfinite(theta_next)) && ...
            all(isfinite(omega_next)) && ...
            all(isfinite(alpha_next)) && ...
            all(isfinite(q_next)) && ...
            numel(poleDistances) == 6 && ...
            all(isfinite(poleDistances)) && transitionSafe;

        result.minimumTransitionPoleDistance = ...
            transitionInfo.minimumPoleDistance;
        result.qNext = q_next;
        result.thetaNext = theta_next;
        result.poleDistances = poleDistances;
    catch
        finiteStatus = false;
    end

    if isempty(violations)
        maxViolation = 0;
    else
        maxViolation = max([0; violations(:)]);
    end

    result.maxViolation = maxViolation;
    fallbackTolerance = readMpcSetting( ...
        params, 'constraintTolerance', 1e-6);
    valid = finiteStatus && isfinite(maxViolation) && ...
        maxViolation <= fallbackTolerance;
end


function [safe, info] = validateThetaTransitionPoleSafety( ...
    thetaStart, thetaEnd, requiredDistance, params)

    safe = false;
    info.minimumPoleDistance = NaN;
    samples = max(2, round(readMpcSetting( ...
        params, 'transitionSafetySamples', 9)));
    tolerance = readMpcSetting(params, 'constraintTolerance', 1e-6);

    if ~isequal(size(thetaStart), [4, 1]) || ...
            ~isequal(size(thetaEnd), [4, 1]) || ...
            any(~isfinite(thetaStart)) || any(~isfinite(thetaEnd))
        return;
    end

    thetaDelta = wrapAngleDifference(thetaEnd, thetaStart);
    minimumDistance = inf;

    try
        for sampleIndex = 1:samples
            lambda = sampleIndex / samples;
            thetaSample = thetaStart + lambda * thetaDelta;
            qSample = abenicsFK(thetaSample, params);
            [~, sampleInfo] = singularityMeasure( ...
                thetaSample, qSample(:), params);
            distances = sampleInfo.poleDistances(:);
            if numel(distances) ~= 6 || any(~isfinite(distances))
                return;
            end
            minimumDistance = min(minimumDistance, min(distances));
        end
    catch
        return;
    end

    info.minimumPoleDistance = minimumDistance;
    safe = isfinite(minimumDistance) && ...
        minimumDistance >= requiredDistance - tolerance;
end


function [safe, info] = validateSelectedPoleOutwardTransition( ...
    thetaStart, thetaEnd, poleIndex, startingDistance, params)

    safe = false;
    info.minimumSelectedDistance = NaN;
    samples = max(2, round(readMpcSetting( ...
        params, 'transitionSafetySamples', 9)));
    tolerance = readMpcSetting(params, 'constraintTolerance', 1e-6);

    if poleIndex < 1 || poleIndex > 6 || ...
            ~isequal(size(thetaStart), [4, 1]) || ...
            ~isequal(size(thetaEnd), [4, 1])
        return;
    end

    thetaDelta = wrapAngleDifference(thetaEnd, thetaStart);
    selectedDistances = zeros(samples, 1);
    try
        for sampleIndex = 1:samples
            lambda = sampleIndex / samples;
            thetaSample = thetaStart + lambda * thetaDelta;
            qSample = abenicsFK(thetaSample, params);
            [~, sampleInfo] = singularityMeasure( ...
                thetaSample, qSample(:), params);
            distances = sampleInfo.poleDistances(:);
            if numel(distances) ~= 6 || any(~isfinite(distances))
                return;
            end
            selectedDistances(sampleIndex) = distances(poleIndex);
        end
    catch
        return;
    end

    info.minimumSelectedDistance = min(selectedDistances);
    safe = all(diff([startingDistance; selectedDistances]) >= -tolerance);
end


function [qFallback, valid, info] = chooseValidatedFallbackCommand( ...
    q_ref, q_current, q_des_prev, x0, ...
    routeMode, poleIndex, committedSide, ...
    qMin, qMax, thetaMin, thetaMax, ...
    omegaMax, alphaMax, maxQStep, dangerDistance, params)

    info.maxViolation = inf;
    info.minimumTransitionPoleDistance = NaN;
    info.routeAware = false;
    info.selectedSide = 0;
    qFallback = q_des_prev;
    valid = false;

    geometry = detourTangentGeometry(q_current, poleIndex, params);
    info.routeAware = logical(routeMode && geometry.valid);
    clearanceFloor = readMpcSetting( ...
        params, 'detourProgressMinimumClearance', ...
        readMpcSetting(params, 'detourContinuationClearance', deg2rad(5)));
    tolerance = readMpcSetting(params, 'constraintTolerance', 1e-6);

    directionSet = zeros(3, 27);
    directionSet(:, 1) = zeros(3, 1);
    cursor = 2;
    for ir = -1:1
        for ip = -1:1
            for iy = -1:1
                if ir == 0 && ip == 0 && iy == 0
                    continue;
                end
                directionSet(:, cursor) = [ir; ip; iy];
                cursor = cursor + 1;
            end
        end
    end

    fractions = [0.25, 0.5, 1.0];
    bestPrimary = -inf;
    bestClearance = -inf;
    bestTracking = inf;

    % Include the bounded direct command first, then hold and the local grid.
    candidateCommands = NaN(3, 2 + 26 * numel(fractions));
    candidateCommands(:, 1) = clampVector( ...
        stepToward(q_des_prev, q_ref, maxQStep), qMin, qMax);
    candidateCommands(:, 2) = q_des_prev;
    candidateIndex = 2;
    for fraction = fractions
        for directionIndex = 2:27
            candidateIndex = candidateIndex + 1;
            candidateCommands(:, candidateIndex) = clampVector( ...
                q_des_prev + fraction * ...
                    directionSet(:, directionIndex) .* maxQStep, ...
                qMin, qMax);
        end
    end

    for candidateIndex = 1:size(candidateCommands, 2)
        qCandidate = candidateCommands(:, candidateIndex);
        [candidateValid, candidateInfo] = validateOneStepCommand( ...
            qCandidate, q_des_prev, x0, ...
            qMin, qMax, thetaMin, thetaMax, ...
            omegaMax, alphaMax, maxQStep, dangerDistance, params);
        if ~candidateValid
            continue;
        end

        primaryScore = 0;
        selectedDistance = inf;
        candidateSide = 0;

        if info.routeAware && geometry.valid
            selectedDistance = candidateInfo.poleDistances(poleIndex);
            vNext = qToRotmXYZ(candidateInfo.qNext) * geometry.bodyAxis;
            rawTangential = dot( ...
                vNext - geometry.vStart, geometry.sideTangent);

            if geometry.currentPoleDistance < clearanceFloor
                [outwardTransitionSafe, ~] = ...
                    validateSelectedPoleOutwardTransition( ...
                        x0(1:4), candidateInfo.thetaNext, poleIndex, ...
                        geometry.currentPoleDistance, params);
                if ~outwardTransitionSafe || selectedDistance < ...
                        geometry.currentPoleDistance - tolerance
                    continue;
                end
                if committedSide == 1 || committedSide == -1
                    candidateSide = committedSide;
                end
                primaryScore = selectedDistance - ...
                    geometry.currentPoleDistance;
            else
                if selectedDistance < clearanceFloor - tolerance || ...
                        candidateInfo.minimumTransitionPoleDistance < ...
                            clearanceFloor - tolerance
                    continue;
                end
                if committedSide == 1 || committedSide == -1
                    candidateSide = committedSide;
                    primaryScore = committedSide * rawTangential;
                    if primaryScore <= 0
                        continue;
                    end
                else
                    candidateSide = sign(rawTangential);
                    primaryScore = abs(rawTangential);
                    if candidateSide == 0 || primaryScore <= 0
                        continue;
                    end
                end
            end
        else
            selectedDistance = min(candidateInfo.poleDistances);
            primaryScore = -sum(wrapAngleDifference( ...
                candidateInfo.qNext, q_ref).^2);
        end

        trackingCost = sum(wrapAngleDifference( ...
            candidateInfo.qNext, q_ref).^2);
        choose = ...
            ~valid || ...
            primaryScore > bestPrimary + 1e-12 || ...
            (abs(primaryScore - bestPrimary) <= 1e-12 && ...
             selectedDistance > bestClearance + 1e-12) || ...
            (abs(primaryScore - bestPrimary) <= 1e-12 && ...
             abs(selectedDistance - bestClearance) <= 1e-12 && ...
             trackingCost < bestTracking);

        if choose
            qFallback = qCandidate;
            valid = true;
            bestPrimary = primaryScore;
            bestClearance = selectedDistance;
            bestTracking = trackingCost;
            info.maxViolation = candidateInfo.maxViolation;
            info.minimumTransitionPoleDistance = ...
                candidateInfo.minimumTransitionPoleDistance;
            info.selectedSide = candidateSide;
        end
    end

    if ~valid
        % Return hold only as an explicit last resort; report it as invalid so
        % the diagnostic never mistakes an unvalidated command for safety.
        qFallback = q_des_prev;
    end
end


% =========================================================================
% Emergency one-step singularity-recovery command
% =========================================================================
function [q_recovery, valid, info] = emergencyRecoveryCommand( ...
    q_ref, q_des_prev, x0, ...
    qMin, qMax, thetaMin, thetaMax, ...
    omegaMax, alphaMax, maxQStep, params)

    q_recovery = q_des_prev;
    valid = false;
    info.minimumTransitionPoleDistance = NaN;

    theta_start = x0(1:4);
    q_start = abenicsFK(theta_start, params);
    q_start = q_start(:);

    [s_start, startInfo] = singularityMeasure( ...
        theta_start, q_start, params);
    [~, recoveryPoleIndex] = min(startInfo.poleDistances(:));

    directions = zeros(3, 27);
    directions(:, 1) = zeros(3, 1);
    directionIndex = 2;
    for ir = -1:1
        for ip = -1:1
            for iy = -1:1
                if ir == 0 && ip == 0 && iy == 0
                    continue;
                end
                directions(:, directionIndex) = [ir; ip; iy];
                directionIndex = directionIndex + 1;
            end
        end
    end

    fractions = [0.25, 0.5, 1.0];
    bestDistance = -inf;
    bestTrackingCost = inf;
    dangerDistance = params.singularity.dangerDistance;
    tolerance = readMpcSetting(params, 'constraintTolerance', 1e-6);
    samples = max(2, round(readMpcSetting( ...
        params, 'transitionSafetySamples', 9)));

    candidateCommands = NaN(3, 2 + 26 * numel(fractions));
    candidateCommands(:, 1) = clampVector( ...
        stepToward(q_des_prev, q_ref, maxQStep), qMin, qMax);
    candidateCommands(:, 2) = q_des_prev;
    candidateIndex = 2;
    for fraction = fractions
        for d = 2:27
            candidateIndex = candidateIndex + 1;
            candidateCommands(:, candidateIndex) = clampVector( ...
                q_des_prev + fraction * directions(:, d) .* maxQStep, ...
                qMin, qMax);
        end
    end

    for candidateIndex = 1:size(candidateCommands, 2)
        q_candidate = candidateCommands(:, candidateIndex);
        q_step = wrapAngleDifference(q_candidate, q_des_prev);
        if any(q_step > maxQStep + tolerance) || ...
                any(q_step < -maxQStep - tolerance)
            continue;
        end

        try
            theta_cmd = abenicsIK(q_candidate, params);
            theta_cmd = theta_cmd(:);
            if any(theta_cmd < thetaMin) || any(theta_cmd > thetaMax)
                continue;
            end

            [x_next, alpha_next] = abenicsPlantStepLocal( ...
                x0, theta_cmd, params);
            theta_next = x_next(1:4);
            omega_next = x_next(5:8);
            if any(abs(omega_next) > omegaMax) || ...
                    any(abs(alpha_next) > alphaMax)
                continue;
            end

            thetaDelta = wrapAngleDifference(theta_next, theta_start);
            selectedDistances = zeros(samples, 1);
            otherMinimum = inf;
            transitionMinimum = inf;
            transitionFinite = true;
            for sampleIndex = 1:samples
                lambda = sampleIndex / samples;
                thetaSample = theta_start + lambda * thetaDelta;
                qSample = abenicsFK(thetaSample, params);
                [~, sampleInfo] = singularityMeasure( ...
                    thetaSample, qSample(:), params);
                distances = sampleInfo.poleDistances(:);
                if numel(distances) ~= 6 || any(~isfinite(distances))
                    transitionFinite = false;
                    break;
                end
                selectedDistances(sampleIndex) = ...
                    distances(recoveryPoleIndex);
                otherDistances = distances;
                otherDistances(recoveryPoleIndex) = inf;
                otherMinimum = min(otherMinimum, min(otherDistances));
                transitionMinimum = min(transitionMinimum, min(distances));
            end
            if ~transitionFinite
                continue;
            end

            % The recovery step may begin inside danger, but it must never
            % move closer to or cross the current pole, and all other poles
            % must remain outside the normal hard limit.
            nondecreasing = all(diff([s_start; selectedDistances]) >= ...
                -tolerance);
            s_next = selectedDistances(end);
            if ~nondecreasing || s_next <= s_start + tolerance || ...
                    otherMinimum < dangerDistance - tolerance
                continue;
            end

            q_next = abenicsFK(theta_next, params);
            trackingCost = sum(wrapAngleDifference( ...
                q_next(:), q_ref).^2);

            if s_next > bestDistance + 1e-12 || ...
              (abs(s_next - bestDistance) <= 1e-12 && ...
               trackingCost < bestTrackingCost)
                bestDistance = s_next;
                bestTrackingCost = trackingCost;
                q_recovery = q_candidate;
                info.minimumTransitionPoleDistance = transitionMinimum;
                valid = true;
            end
        catch
            continue;
        end
    end
end


% =========================================================================
% Internal second-order plant prediction
% =========================================================================
function [x_next, alpha] = abenicsPlantStepLocal(x, theta_cmd, params)

    Ts = params.Ts;

    theta = x(1:4);
    omega = x(5:8);

    KpPlant = expandToVector(params.plant.KpPlant, 4);
    KdPlant = expandToVector(params.plant.KdPlant, 4);

    theta_error = wrapAngleDifference(theta_cmd, theta);

    % Existing approximation of lower-level PID + motor + mechanics.
    alpha = KpPlant .* theta_error - KdPlant .* omega;

    omega_next = omega + Ts * alpha;
    theta_next = theta + Ts * omega_next;

    x_next = [theta_next;
              omega_next];
end

% =========================================================================
% Move a vector toward a target by a bounded per-axis step
% =========================================================================
function q_next = stepToward(q_current, q_target, maxStep)

    delta = wrapAngleDifference(q_target, q_current);
    boundedStep = clampVector(delta, -maxStep, maxStep);

    q_next = q_current + boundedStep;
end

% =========================================================================
% Enforce the final output step limit
% =========================================================================
function q_limited = limitQStep(q_candidate, q_prev, maxStep)

    delta = wrapAngleDifference(q_candidate, q_prev);
    boundedDelta = clampVector(delta, -maxStep, maxStep);

    q_limited = q_prev + boundedDelta;
end

% =========================================================================
% Wrapped elementwise angular difference in [-pi, pi]
% =========================================================================
function d = wrapAngleDifference(a, b)

    d = atan2(sin(a - b), cos(a - b));
end

% =========================================================================
% Elementwise vector clamp
% =========================================================================
function y = clampVector(x, xmin, xmax)

    y = min(max(x, xmin), xmax);
end

% =========================================================================
% Expand a scalar parameter to a vector
% =========================================================================
function v = expandToVector(value, n)

    value = value(:);

    if numel(value) == 1
        v = value * ones(n, 1);
    elseif numel(value) == n
        v = value;
    else
        error('abenicsOrientationMPCCore:parameterSize', ...
              'Expected a scalar or %dx1 parameter.', n);
    end
end

% =========================================================================
% Read an MPC setting while keeping the required first-version defaults
% =========================================================================
function value = readMpcSetting(params, fieldName, defaultValue)

    if isfield(params, 'mpc') && isfield(params.mpc, fieldName)
        value = params.mpc.(fieldName);
    else
        value = defaultValue;
    end
end

% =========================================================================
% Persistent-state helpers
% =========================================================================
function value = readStateField(stateIn, fieldName, defaultValue)

    if isfield(stateIn, fieldName) && ~isempty(stateIn.(fieldName))
        value = stateIn.(fieldName);
    else
        value = defaultValue;
    end
end


function value = logicalScalarState(stateIn, fieldName, defaultValue)

    raw = readStateField(stateIn, fieldName, defaultValue);
    if isempty(raw) || ~isfinite(double(raw(1)))
        value = logical(defaultValue);
    else
        value = logical(raw(1));
    end
end


function value = numericScalarState(stateIn, fieldName, defaultValue)

    raw = readStateField(stateIn, fieldName, defaultValue);
    if isempty(raw) || ~isfinite(raw(1))
        value = defaultValue;
    else
        value = raw(1);
    end
end


function [detourActive, committedPoleIndex, committedDetourSide, ...
    committedClearance, committedReference, ...
    detourFailureCount, detourClearCounter] = ...
    clearedDetourState(defaultClearance)

    detourActive = false;
    committedPoleIndex = 0;
    committedDetourSide = 0;
    committedClearance = defaultClearance;
    committedReference = zeros(3, 1);
    detourFailureCount = 0;
    detourClearCounter = 0;
end


function stateOut = buildStateOut( ...
    theta_prev_for_omega, recoveryActive, previousDeltaQ, ...
    detourActive, committedPoleIndex, committedDetourSide, ...
    committedClearance, committedReference, ...
    detourFailureCount, detourClearCounter)

    stateOut.theta_prev_for_omega = theta_prev_for_omega;
    stateOut.recoveryActive = recoveryActive;
    stateOut.previousDeltaQ = previousDeltaQ;
    stateOut.detourActive = detourActive;
    stateOut.committedPoleIndex = committedPoleIndex;
    stateOut.committedDetourSide = committedDetourSide;
    stateOut.committedClearance = committedClearance;
    stateOut.committedReference = committedReference;
    stateOut.detourFailureCount = detourFailureCount;
    stateOut.detourClearCounter = detourClearCounter;
end


% =========================================================================
% Fixed diagnostics structures
% =========================================================================
function result = emptyValidationResult()

    result.maxConstraintIndex = NaN;
    result.maxConstraintStep = NaN;
    result.maxConstraintName = "none";
    result.solutionFinite = false;
    result.objectiveFinite = false;
    result.constraintsSatisfied = false;
    result.firstCommandValid = false;
    result.firstTransitionSafe = false;
    result.minimumFirstTransitionPoleDistance = NaN;
    result.rolloutFinite = false;
    result.maxConstraintViolation = inf;
end


function diagnostics = initializeDiagnostics(maxStarts)

    diagnostics.Np = NaN;
    diagnostics.Nc = NaN;
    diagnostics.solverAvailable = false;
    diagnostics.usedFmincon = false;
    diagnostics.exitflag = NaN;
    diagnostics.iterations = 0;
    diagnostics.functionEvaluations = 0;
    diagnostics.solveTime = 0;
    diagnostics.totalSolveTime = 0;
    diagnostics.worstStartSolveTime = 0;
    diagnostics.objective = NaN;
    diagnostics.maxConstraintViolation = inf;

    diagnostics.maxConstraintIndex = NaN;
    diagnostics.maxConstraintStep = NaN;
    diagnostics.maxConstraintName = "none";

    diagnostics.solutionFinite = false;
    diagnostics.objectiveFinite = false;
    diagnostics.rolloutFinite = false;
    diagnostics.constraintsSatisfied = false;
    diagnostics.firstCommandValid = false;
    diagnostics.firstTransitionSafe = false;
    diagnostics.minimumFirstTransitionPoleDistance = NaN;
    diagnostics.solutionAccepted = false;
    diagnostics.fallbackUsed = false;
    diagnostics.fallbackCount = 0;
    diagnostics.fallbackReason = "none";
    diagnostics.directFallbackValid = false;
    diagnostics.directFallbackMaxViolation = NaN;
    diagnostics.fallbackMinimumTransitionPoleDistance = NaN;
    diagnostics.recoveryActive = false;
    diagnostics.recoveryUsed = false;
    diagnostics.recoveryCommandValid = false;
    diagnostics.recoveryMinimumTransitionPoleDistance = NaN;
    diagnostics.recoveryClearDistance = NaN;
    diagnostics.currentStateValid = false;
    diagnostics.currentSingularityDistance = NaN;
    diagnostics.targetStateValid = false;
    diagnostics.targetSingularityDistance = NaN;
    diagnostics.singularTargetOverrideActive = false;
    diagnostics.message = "";

    diagnostics.shiftedWarmStartFeasible = false;
    diagnostics.routeBlocked = false;
    diagnostics.blockingPoleIndex = 0;
    diagnostics.minimumNominalPoleDistance = inf;
    diagnostics.directRolloutFinite = false;
    diagnostics.directRouteBlocked = false;
    diagnostics.minimumDirectPoleDistance = inf;
    diagnostics.detourActive = false;
    diagnostics.committedPoleIndex = 0;
    diagnostics.committedDetourSide = 0;
    diagnostics.detourClearCounter = 0;
    diagnostics.detourFailureCount = 0;

    diagnostics.numberOfStarts = 0;
    diagnostics.numberOfFminconSolves = 0;
    diagnostics.numberOfAcceptedStarts = 0;
    diagnostics.numberOfRouteEligibleStarts = 0;
    diagnostics.winningStartIndex = 0;
    diagnostics.winningStartType = 99;
    diagnostics.winningDetourSide = 0;
    diagnostics.winningClearance = NaN;
    diagnostics.winningRouteEligible = false;
    diagnostics.winningProgress = 0;
    diagnostics.winningProgressMode = 0;
    diagnostics.winningFirstProgress = 0;
    diagnostics.winningFirstPoleDistance = NaN;
    diagnostics.winningNearPoleDistance = NaN;
    diagnostics.winningMinimumNearPoleDistance = NaN;

    diagnostics.startType = 99 * ones(maxStarts, 1);
    diagnostics.startPoleIndex = zeros(maxStarts, 1);
    diagnostics.startSide = zeros(maxStarts, 1);
    diagnostics.startClearance = NaN(maxStarts, 1);
    diagnostics.startSeedValid = false(maxStarts, 1);
    diagnostics.startSeedBlendScale = NaN(maxStarts, 1);
    diagnostics.startSeedConstraintViolation = NaN(maxStarts, 1);
    diagnostics.startExitflag = NaN(maxStarts, 1);
    diagnostics.startAccepted = false(maxStarts, 1);
    diagnostics.startObjective = NaN(maxStarts, 1);
    diagnostics.startMinimumPoleDistance = NaN(maxStarts, 1);
    diagnostics.startIterations = zeros(maxStarts, 1);
    diagnostics.startFunctionEvaluations = zeros(maxStarts, 1);
    diagnostics.startSolveTime = zeros(maxStarts, 1);
    diagnostics.startRejectionCode = zeros(maxStarts, 1);
    diagnostics.startRouteEligible = false(maxStarts, 1);
    diagnostics.startProgressMode = zeros(maxStarts, 1);
    diagnostics.startProgressSide = zeros(maxStarts, 1);
    diagnostics.startFirstProgress = zeros(maxStarts, 1);
    diagnostics.startProgress = zeros(maxStarts, 1);
    diagnostics.startFirstPoleDistance = NaN(maxStarts, 1);
    diagnostics.startNearPoleDistance = NaN(maxStarts, 1);
    diagnostics.startMinimumNearPoleDistance = NaN(maxStarts, 1);
end
