function q_des_mpc = abenicsOrientationMPC( ...
    q_ref, theta_actual, q_des_prev, params)

    persistent theta_prev_for_omega
    persistent recoveryActive
    persistent previousDeltaQ
    persistent singularTargetWarningShown

    controllerState.theta_prev_for_omega = theta_prev_for_omega;
    controllerState.recoveryActive = recoveryActive;
    controllerState.previousDeltaQ = previousDeltaQ;

    [q_des_mpc, diagnostics, controllerState] = ...
        abenicsOrientationMPCCore( ...
            q_ref, ...
            theta_actual, ...
            q_des_prev, ...
            params, ...
            controllerState);

    % Initialize the warning memory on the first controller call.
    if isempty(singularTargetWarningShown)
        singularTargetWarningShown = false;
    end

    % Warnings are enabled by default.
    emitWarning = true;

    % Allow the parameter file to disable MATLAB warnings.
    if isfield(params.mpc, 'emitSingularTargetWarning')
        emitWarning = logical( ...
            params.mpc.emitSingularTargetWarning);
    end

    % Check whether the core detected an unsafe requested target.
    if diagnostics.singularTargetOverrideActive

        % Print only once while this unsafe target remains active.
        if emitWarning && ~singularTargetWarningShown
            warning( ...
                'abenicsOrientationMPC:SingularTargetOverride', ...
                ['Requested target is %.4f deg from a singular pole. ', ...
                'Singular-target override is active, so normal ', ...
                'singularity protection is being relaxed.'], ...
                rad2deg( ...
                diagnostics.targetSingularityDistance));
        end

        % Remember that the warning has been shown.
        singularTargetWarningShown = true;

    else
        % Reset after returning to a safe target.
        % A future unsafe target will generate a new warning.
        singularTargetWarningShown = false;
    end

    if isfield(params.mpc, 'debug') && params.mpc.debug
        if isfield(params.mpc, 'debug') && params.mpc.debug

    fprintf( ...
        ['exit=%g | accepted=%d | fallback=%d | reason=%s | ', ...
         'solutionFinite=%d | objectiveFinite=%d | rolloutFinite=%d | ', ...
         'constraints=%d | firstCommand=%d | ', ...
         'constraint=%s | step=%g | iterations=%d | ', ...
         'evaluations=%d | violation=%.3e\n'], ...
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
        diagnostics.maxConstraintViolation);

    % Print the full fmincon explanation only when a solve was rejected.
    if diagnostics.fallbackUsed && diagnostics.usedFmincon
        fprintf( ...
            'Solver message: %s\n', ...
            char(diagnostics.message));
    end
end

    theta_prev_for_omega = controllerState.theta_prev_for_omega;
    recoveryActive = controllerState.recoveryActive;
    previousDeltaQ = controllerState.previousDeltaQ;
end


function [q_des_mpc, diagnostics, stateOut] = abenicsOrientationMPCCore(q_ref, theta_actual, q_des_prev, params, stateIn)

    % Existing MPC core code continues here

%ABENICSORIENTATIONMPCCORE Continuous nonlinear MPC core for ABENICS.
%
% This function replaces the normal predetermined candidate-path search
% with direct-shooting nonlinear MPC solved by fmincon.
%
% Inputs:
%   q_ref        : 3x1 target orientation [roll; pitch; yaw], rad
%   theta_actual : 4x1 measured output-side MP-gear angles, rad
%   q_des_prev   : 3x1 previous MPC orientation command, rad
%   params       : ABENICS parameter structure
%   stateIn      : controller state structure. Optional for direct tests.
%
% Outputs:
%   q_des_mpc    : 3x1 safe orientation command, rad
%   diagnostics  : solver, fallback, recovery, and constraint information
%   stateOut     : updated controller state for the next call
%
% The Simulink-facing abenicsOrientationMPC wrapper stores stateOut in its
% required persistent variables and returns only q_des_mpc.
%
% Required control chain:
%   q_ref -> MPC -> q_des_mpc -> abenicsIK -> theta_cmd -> PID/plant
%
% The MPC never outputs motor torque, velocity, current, or theta_cmd.

    if nargin < 5 || isempty(stateIn)
        stateIn = struct();
    end

    % ---------------------------------------------------------------------
    % Force expected column shapes and check public inputs
    % ---------------------------------------------------------------------
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

    % ---------------------------------------------------------------------
    % Read controller settings
    % ---------------------------------------------------------------------
    Ts = params.Ts;

    Np = round(readMpcSetting(params, 'Np', 20));
    Nc = round(readMpcSetting(params, 'Nc', 5));

    if Np < 1 || Nc < 1 || Nc > Np
        error('abenicsOrientationMPCCore:horizon', ...
              'Require Np >= 1 and 1 <= Nc <= Np.');
    end

    % Store the resolved values in this local params copy so every shared
    % objective, constraint, and rollout call uses exactly the same horizons.
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
        readMpcSetting(params, ...
        'recoveryMaxQStep', params.mpc.maxQStep), 3);

    % Recovery should never be allowed a larger step than normal MPC.
    recoveryMaxQStep = min(recoveryMaxQStep, maxQStep);

    dangerDistance = params.singularity.dangerDistance;
    warningDistance = params.singularity.warningDistance;
    
    recoveryClearDistance = readMpcSetting( ...
        params, 'recoveryClearDistance', deg2rad(3));

    constraintTolerance = readMpcSetting( ...
        params, 'constraintTolerance', 1e-6);

    % Preserve the existing protection that the previous command and target
    % used by the controller remain inside the configured orientation box.
    q_ref = clampVector(q_ref, qMin, qMax);
    q_des_prev = clampVector(q_des_prev, qMin, qMax);

    % ---------------------------------------------------------------------
    % Initialize diagnostics
    % ---------------------------------------------------------------------
    diagnostics = initializeDiagnostics();
    diagnostics.Np = Np;
    diagnostics.Nc = Nc;
    diagnostics.recoveryClearDistance = recoveryClearDistance;
    

    % ---------------------------------------------------------------------
    % Recover persistent controller state supplied by the public wrapper
    % ---------------------------------------------------------------------
    theta_prev_for_omega = [];
    if isfield(stateIn, 'theta_prev_for_omega')
        theta_prev_for_omega = stateIn.theta_prev_for_omega;
    end

    recoveryActive = false;
    if isfield(stateIn, 'recoveryActive') && ...
            ~isempty(stateIn.recoveryActive)
        recoveryActive = logical(stateIn.recoveryActive(1));
    end

    previousDeltaQ = zeros(3, Nc);
    if isfield(stateIn, 'previousDeltaQ') && ...
            isequal(size(stateIn.previousDeltaQ), [3, Nc])
        previousDeltaQ = stateIn.previousDeltaQ;
    end

    % Required behavior: reinitialize whenever dimensions do not equal
    % exactly [3, Nc].
    if ~isequal(size(previousDeltaQ), [3, Nc]) || ...
            any(~isfinite(previousDeltaQ(:)))
        previousDeltaQ = zeros(3, Nc);
    end

    % ---------------------------------------------------------------------
    % Estimate current MP-gear velocity using the existing theta history
    % ---------------------------------------------------------------------
    if isempty(theta_prev_for_omega) || ...
            ~isequal(size(theta_prev_for_omega), [4, 1]) || ...
            any(~isfinite(theta_prev_for_omega))
        theta_prev_for_omega = theta_actual;
    end

    omega_est = wrapAngleDifference( ...
        theta_actual, theta_prev_for_omega) / Ts;

    theta_prev_for_omega = theta_actual;

    % Limit only the measured velocity estimate used by the predictor.
    omega_est = clampVector(omega_est, -omegaMax, omegaMax);

    x0 = [theta_actual;
          omega_est];

    % ---------------------------------------------------------------------
    % Measure the current physical singularity distance
    % ---------------------------------------------------------------------
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
        s_current = NaN;
    end

    diagnostics.currentStateValid = currentStateValid;
    diagnostics.currentSingularityDistance = s_current;

    if ~currentStateValid
        % There is no trustworthy model state from which to optimize or
        % choose a recovery direction. The safest bounded output is hold.
        q_des_mpc = q_des_prev;
        previousDeltaQ = zeros(3, Nc);

        diagnostics.fallbackUsed = true;
        diagnostics.fallbackCount = 1;
        diagnostics.fallbackReason = "invalidCurrentStateHold";

        q_des_mpc = limitQStep(q_des_mpc, q_des_prev, maxQStep);
        q_des_mpc = clampVector(q_des_mpc, qMin, qMax);

        stateOut = buildStateOut( ...
            theta_prev_for_omega, recoveryActive, previousDeltaQ);
        return;
    end

    % ---------------------------------------------------------------------
    % Measure singularity distance at the requested target
    % ---------------------------------------------------------------------
    targetStateValid = true;

    try
        % Convert the requested target into its corresponding MP-gear command.
        theta_target = abenicsIK(q_ref, params);
        theta_target = theta_target(:);

        % Measure all six physical pole distances at the requested target.
        [s_target, targetSingularityInfo] = singularityMeasure( ...
            theta_target, q_ref, params);

        targetPoleDistances = ...
            targetSingularityInfo.poleDistances(:);

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

    % Read whether the user has explicitly enabled unsafe targets.
    allowSingularTarget = logical(readMpcSetting( ...
        params, 'allowSingularTarget', false));

    % Activate the override when the target lies inside the warning region.
    % This includes targets inside dangerDistance.
    singularTargetOverrideActive = ...
        allowSingularTarget && ...
        targetStateValid && ...
        s_target < warningDistance;

    % Store these in the local params copy so the objective and constraints
    % use exactly the same target classification.
    params.mpc.singularTargetOverrideActive = ...
        singularTargetOverrideActive;

    params.mpc.targetSingularityDistance = ...
        s_target;

    params.mpc.targetPoleDistances = ...
        targetPoleDistances;

    params.mpc.currentPoleDistances = ...
        currentSingularityInfo.poleDistances(:);

    % Diagnostics for MATLAB testing and warning generation.
    diagnostics.targetStateValid = targetStateValid;
    diagnostics.targetSingularityDistance = s_target;
    diagnostics.singularTargetOverrideActive = ...
        singularTargetOverrideActive;

    % ---------------------------------------------------------------------
    % Persistent singularity-recovery hysteresis
    % ---------------------------------------------------------------------
    % Do not clear recovery merely after crossing dangerDistance. Recovery
    % remains active until the measured current state reaches the separate
    % recoveryClearDistance.
    targetInsideDangerOverride = ...
    singularTargetOverrideActive && ...
    s_target < dangerDistance;

if targetInsideDangerOverride
    % The user explicitly requested entry into danger. Do not repeatedly
    % activate emergency recovery while intentionally approaching it.
    disableRecoveryForSingularTarget = logical(readMpcSetting( ...
        params, 'disableRecoveryForSingularTarget', false));
    
    targetInsideDangerOverride = ...
        singularTargetOverrideActive && ...
        disableRecoveryForSingularTarget && ...
        s_target < dangerDistance;

else
    % Normal safety behavior remains unchanged.
    if s_current < dangerDistance
        recoveryActive = true;
    elseif recoveryActive && ...
           s_current >= recoveryClearDistance
        recoveryActive = false;
    end
end
    diagnostics.recoveryActive = recoveryActive;

    % ---------------------------------------------------------------------
    % Recovery mode bypasses normal fmincon MPC completely
    % ---------------------------------------------------------------------
    if recoveryActive
        q_des_mpc = emergencyRecoveryCommand( ...
            q_ref, q_des_prev, x0, ...
            qMin, qMax, thetaMin, thetaMax, ...
            omegaMax, alphaMax, recoveryMaxQStep, params);
        % Required: never preserve a normal-MPC warm start during recovery.
        previousDeltaQ = zeros(3, Nc);

        diagnostics.recoveryUsed = true;
        diagnostics.fallbackUsed = true;
        diagnostics.fallbackCount = 1;
        diagnostics.fallbackReason = "emergencyRecovery";

        q_des_mpc = limitQStep( q_des_mpc, q_des_prev, recoveryMaxQStep);
        q_des_mpc = clampVector(q_des_mpc, qMin, qMax);

        stateOut = buildStateOut( ...
            theta_prev_for_omega, recoveryActive, previousDeltaQ);
        return;
    end

    % ---------------------------------------------------------------------
    % Build the shifted warm start for command increments
    % ---------------------------------------------------------------------
    oldDeltaQ = previousDeltaQ;
    shiftedDeltaQ = zeros(3, Nc);

    if Nc > 1
        shiftedDeltaQ(:, 1:Nc-1) = oldDeltaQ(:, 2:Nc);
    end

    % Required correction: append zero. Do not repeat the previous final
    % increment.
    shiftedDeltaQ(:, Nc) = zeros(3, 1);

    lb = repmat(-maxQStep, Nc, 1);
    ub = repmat( maxQStep, Nc, 1);

    z0 = shiftedDeltaQ(:);
    z0 = clampVector(z0, lb, ub);

    % ---------------------------------------------------------------------
    % Repair an infeasible shifted warm start
    % ---------------------------------------------------------------------
    warmStartScales = [1.0, 0.5, 0.25, 0.0];
    feasibleWarmStartFound = false;

    for scaleIndex = 1:numel(warmStartScales)

        zCandidate = warmStartScales(scaleIndex) * z0;

        [cInitial, ceqInitial] = continuousMpcConstraints( ...
            zCandidate, x0, q_des_prev, params);

        inequalitiesValid = ...
            isempty(cInitial) || ...
            all(cInitial <= constraintTolerance);

        equalitiesValid = ...
            isempty(ceqInitial) || ...
            all(abs(ceqInitial) <= constraintTolerance);

        if inequalitiesValid && equalitiesValid
            z0 = zCandidate;
            feasibleWarmStartFound = true;
            break;
        end
    end

    % If none of the scaled warm starts is feasible, use zero increments as
    % the most conservative available initial guess. fmincon may still begin
    % infeasible if current plant motion alone violates a predicted limit.
    if ~feasibleWarmStartFound
        z0 = zeros(3 * Nc, 1);
    end

    % ---------------------------------------------------------------------
    % Run continuous direct-shooting nonlinear MPC
    % ---------------------------------------------------------------------
    objectiveFunction = @(z) continuousMpcObjective( ...
        z, x0, q_ref, q_des_prev, params);

    constraintFunction = @(z) continuousMpcConstraints( ...
        z, x0, q_des_prev, params);

    solverAvailable = exist('fmincon', 'file') == 2;
    diagnostics.solverAvailable = solverAvailable;

    optimizationAccepted = false;
    zOptimal = [];

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

        solveTimer = tic;

        try
            [zOptimal, fval, exitflag, output] = fmincon( ...
                objectiveFunction, z0, ...
                [], [], [], [], lb, ub, ...
                constraintFunction, options);

            diagnostics.solveTime = toc(solveTimer);
            diagnostics.usedFmincon = true;
            diagnostics.exitflag = exitflag;
            diagnostics.objective = fval;

            if isfield(output, 'iterations')
                diagnostics.iterations = output.iterations;
            end

            if isfield(output, 'funcCount')
                diagnostics.functionEvaluations = output.funcCount;
            end

            if isfield(output, 'message')
                diagnostics.message = string(output.message);
            end

            [optimizationAccepted, validation] = ...
                validateOptimizedSolution( ...
                    zOptimal, fval, exitflag, ...
                    x0, q_ref, q_des_prev, ...
                    qMin, qMax, maxQStep, ...
                    constraintTolerance, params);

            diagnostics.solutionFinite = validation.solutionFinite;
            diagnostics.objectiveFinite = validation.objectiveFinite;
            diagnostics.constraintsSatisfied = ...
                validation.constraintsSatisfied;
            diagnostics.firstCommandValid = validation.firstCommandValid;
            diagnostics.rolloutFinite = validation.rolloutFinite;
            
            diagnostics.maxConstraintViolation = ...
                validation.maxConstraintViolation;
            
            % Identify which nonlinear constraint failed.
            diagnostics.maxConstraintIndex = ...
                validation.maxConstraintIndex;
            
            diagnostics.maxConstraintStep = ...
                validation.maxConstraintStep;
            
            diagnostics.maxConstraintName = ...
                validation.maxConstraintName;
            
            diagnostics.solutionAccepted = optimizationAccepted;
        catch solverException
            diagnostics.solveTime = toc(solveTimer);
            diagnostics.usedFmincon = true;
            diagnostics.message = string(solverException.message);
            optimizationAccepted = false;
        end
    else
        diagnostics.message = ...
            "fmincon is unavailable. Optimization Toolbox was not found.";
    end

    % ---------------------------------------------------------------------
    % Receding-horizon output or validated fallback
    % ---------------------------------------------------------------------
    if optimizationAccepted
        deltaQOptimal = reshape(zOptimal, 3, Nc);

        q_des_mpc = q_des_prev + deltaQOptimal(:, 1);

        previousDeltaQ = deltaQOptimal;
    else
        % A failed or invalid solve must not seed the next warm start.
        previousDeltaQ = zeros(3, Nc);

        q_direct = stepToward(q_des_prev, q_ref, maxQStep);
        q_direct = clampVector(q_direct, qMin, qMax);

        [directStepValid, directValidation] = validateOneStepCommand( ...
            q_direct, q_des_prev, x0, ...
            qMin, qMax, thetaMin, thetaMax, ...
            omegaMax, alphaMax, maxQStep, ...
            dangerDistance, params);

        diagnostics.fallbackUsed = true;
        diagnostics.fallbackCount = 1;
        diagnostics.directFallbackValid = directStepValid;
        diagnostics.directFallbackMaxViolation = ...
            directValidation.maxViolation;

        if directStepValid
            q_des_mpc = q_direct;
            diagnostics.fallbackReason = "boundedDirectStep";
        else
            q_des_mpc = q_des_prev;
            diagnostics.fallbackReason = "holdPreviousCommand";
        end
    end

    % ---------------------------------------------------------------------
    % Final output protections only
    % ---------------------------------------------------------------------
    % Qseq is deliberately not clamped inside the optimizer. These final
    % protections apply only to the one public controller output.
    q_des_mpc = limitQStep(q_des_mpc, q_des_prev, maxQStep);
    q_des_mpc = clampVector(q_des_mpc, qMin, qMax);

    if ~isequal(size(q_des_mpc), [3, 1]) || any(~isfinite(q_des_mpc))
        q_des_mpc = q_des_prev;
        previousDeltaQ = zeros(3, Nc);
        diagnostics.fallbackUsed = true;
        diagnostics.fallbackCount = 1;
        diagnostics.fallbackReason = "nonfiniteFinalOutputHold";
    end

    stateOut = buildStateOut( ...
        theta_prev_for_omega, recoveryActive, previousDeltaQ);
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
        result.firstCommandValid;
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

    if ~isequal(size(q_candidate), [3, 1]) || ...
            any(~isfinite(q_candidate))
        valid = false;
        result.maxViolation = inf;
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
            all(isfinite(poleDistances));
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

% =========================================================================
% Emergency one-step singularity-recovery command
% =========================================================================
function q_recovery = emergencyRecoveryCommand( ...
    q_ref, q_des_prev, x0, ...
    qMin, qMax, thetaMin, thetaMax, ...
    omegaMax, alphaMax, maxQStep, params)

    theta_start = x0(1:4);
    q_start = abenicsFK(theta_start, params);
    q_start = q_start(:);

    [s_start, ~] = singularityMeasure( ...
        theta_start, q_start, params);

    % One direct command plus all nonzero combinations of:
    % {-max step, 0, +max step} in roll, pitch, and yaw.
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

    q_recovery = q_des_prev;
    bestFound = false;
    bestDistance = -inf;
    bestTrackingCost = inf;

    for c = 1:27
        if c == 1
            % First test a normal step toward q_ref.
            q_candidate = stepToward( ...
                q_des_prev, q_ref, maxQStep);
        else
            q_candidate = q_des_prev + ...
                directions(:, c) .* maxQStep;
        end

        q_candidate = clampVector(q_candidate, qMin, qMax);

        theta_cmd = abenicsIK(q_candidate, params);
        theta_cmd = theta_cmd(:);

        if any(theta_cmd < thetaMin) || ...
           any(theta_cmd > thetaMax)
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

        q_next = abenicsFK(theta_next, params);
        q_next = q_next(:);

        [s_next, ~] = singularityMeasure( ...
            theta_next, q_next, params);

        trackingError = wrapAngleDifference(q_next, q_ref);
        trackingCost = sum(trackingError.^2);

        % Primary recovery objective: maximize predicted singularity
        % distance. Tracking error remains only a tie-breaker.
        if s_next > bestDistance + 1e-12 || ...
          (abs(s_next - bestDistance) <= 1e-12 && ...
           trackingCost < bestTrackingCost)

            bestDistance = s_next;
            bestTrackingCost = trackingCost;
            q_recovery = q_candidate;
            bestFound = true;
        end
    end

    % Do not replace a singular hold with another command unless the
    % internal model predicts at least some movement away from the pole.
    if ~bestFound || bestDistance <= s_start
        q_recovery = q_des_prev;
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
% Build the state returned to the persistent public wrapper
% =========================================================================
function stateOut = buildStateOut( ...
    theta_prev_for_omega, recoveryActive, previousDeltaQ)

    stateOut.theta_prev_for_omega = theta_prev_for_omega;
    stateOut.recoveryActive = recoveryActive;
    stateOut.previousDeltaQ = previousDeltaQ;
end

% =========================================================================
% Initialize a fixed diagnostics structure
% =========================================================================
function diagnostics = initializeDiagnostics()

    diagnostics.Np = NaN;
    diagnostics.Nc = NaN;
    diagnostics.solverAvailable = false;
    diagnostics.usedFmincon = false;
    diagnostics.exitflag = NaN;
    diagnostics.iterations = 0;
    diagnostics.functionEvaluations = 0;
    diagnostics.solveTime = 0;
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
    diagnostics.solutionAccepted = false;
    diagnostics.fallbackUsed = false;
    diagnostics.fallbackCount = 0;
    diagnostics.fallbackReason = "none";
    diagnostics.directFallbackValid = false;
    diagnostics.directFallbackMaxViolation = NaN;
    diagnostics.recoveryActive = false;
    diagnostics.recoveryUsed = false;
    diagnostics.recoveryClearDistance = NaN;
    diagnostics.currentStateValid = false;
    diagnostics.currentSingularityDistance = NaN;
    diagnostics.targetStateValid = false;
    diagnostics.targetSingularityDistance = NaN;
    diagnostics.singularTargetOverrideActive = false;
    diagnostics.message = "";
end
end