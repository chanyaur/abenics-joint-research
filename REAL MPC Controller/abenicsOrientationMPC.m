function q_des_mpc = abenicsOrientationMPC(q_ref, theta_actual, q_des_prev, params)
%ABENICSORIENTATIONMPC Nonlinear dynamic orientation-command MPC for ABENICS.
%
% Main output:
%   q_des_mpc = [roll_des_mpc; pitch_des_mpc; yaw_des_mpc] in radians
%
% The controller outputs only a safe desired CS-gear orientation. It does
% not output motor torque, motor velocity, or theta_cmd. Simulink should
% send q_des_mpc into abenicsIK, then into the lower-level PID and plant.
%
% Existing project functions:
%   theta_cmd = abenicsIK(q_des, params)
%   q_pred    = abenicsFK(theta_actual, params)
%   [s, info] = singularityMeasure(theta, q, params)
%
% Inputs:
%   q_ref        : 3x1 raw target orientation [roll; pitch; yaw], rad
%   theta_actual : 4x1 output-side MP-gear angles, rad
%   q_des_prev   : 3x1 previous MPC output, rad
%   params       : ABENICS parameter structure
%
% Internal prediction state:
%   x = [theta; omega], where theta and omega are each 4x1
%
% Singularity handling:
%   - Distance-based singularity detection only
%   - A safe trajectory may not enter the danger zone
%   - A trajectory starting inside danger may remain there temporarily
%     only while making measurable progress away from the singularity
%   - After a recovery trajectory leaves danger, it may not re-enter
%
% The internal second-order plant is the MPC prediction model. It does not
% replace the real Simulink PID and plant.

    % ---------------------------------------------------------------------
    % Force expected column shapes
    % ---------------------------------------------------------------------
    q_ref        = q_ref(:);
    theta_actual = theta_actual(:);
    q_des_prev   = q_des_prev(:);

    % ---------------------------------------------------------------------
    % Read MPC parameters
    % ---------------------------------------------------------------------
    Ts = params.Ts;

    qMin      = params.mpc.qMin(:);
    qMax      = params.mpc.qMax(:);
    thetaMin  = params.mpc.thetaMin(:);
    thetaMax  = params.mpc.thetaMax(:);
    omegaMax  = params.mpc.omegaMax(:);
    alphaMax  = params.mpc.alphaMax(:);
    maxQStep  = expandToVector(params.mpc.maxQStep, 3);

    wTrack       = params.mpc.wTrack;
    wTerminal    = params.mpc.wTerminal;
    wSmooth      = params.mpc.wSmooth;
    wMotor       = params.mpc.wMotor;
    wSingularity = params.mpc.wSingularity;
    wOmega       = params.mpc.wOmega;

    % Define params.mpc.avoidOffset explicitly for strict Simulink code
    % generation. Normal MATLAB uses 8 deg if it is not present.
    if isfield(params.mpc, 'avoidOffset')
        avoidOffset = params.mpc.avoidOffset;
    else
        avoidOffset = deg2rad(8);
    end

    % ---------------------------------------------------------------------
    % Estimate current MP-gear velocity
    % ---------------------------------------------------------------------
    persistent theta_prev_for_omega

    if isempty(theta_prev_for_omega)
        theta_prev_for_omega = theta_actual;
    end

    omega_est = wrapAngleDifference( ...
        theta_actual, theta_prev_for_omega) / Ts;

    theta_prev_for_omega = theta_actual;

    % Limit only the velocity estimate used by the internal predictor.
    omega_est = clampVector(omega_est, -omegaMax, omegaMax);

    % Measured/simulated plant feedback initializes every MPC prediction.
    x0 = [theta_actual;
          omega_est];

    q_des_prev = clampVector(q_des_prev, qMin, qMax);

    % ---------------------------------------------------------------------
    % Generate parameterized q_des trajectories
    % ---------------------------------------------------------------------
    Qcandidates = generateCandidateSequences( ...
        q_ref, q_des_prev, params, avoidOffset);

    numCandidates = size(Qcandidates, 3);

    bestJ = inf;
    bestIndex = 1;
    foundValid = false;

    % ---------------------------------------------------------------------
    % Evaluate every candidate trajectory
    % ---------------------------------------------------------------------
    for c = 1:numCandidates
        Qseq = Qcandidates(:, :, c);

        [J, valid] = evaluateMpcTrajectory( ...
            Qseq, x0, q_ref, q_des_prev, ...
            thetaMin, thetaMax, omegaMax, alphaMax, ...
            wTrack, wTerminal, wSmooth, wMotor, ...
            wSingularity, wOmega, params);

        if valid && J < bestJ
            bestJ = J;
            bestIndex = c;
            foundValid = true;
        end
    end

    % ---------------------------------------------------------------------
    % Receding-horizon output
    % ---------------------------------------------------------------------
    if foundValid
        % Apply only the first command from the best predicted sequence.
        q_des_mpc = Qcandidates(:, 1, bestIndex);
    else
        % Dedicated emergency recovery fallback:
        % choose the one-step command whose predicted plant response moves
        % farthest away from the nearest singular pole.
        q_des_mpc = emergencyRecoveryCommand( ...
            q_ref, q_des_prev, x0, ...
            qMin, qMax, thetaMin, thetaMax, ...
            omegaMax, alphaMax, maxQStep, params);
    end

    % Final output protections.
    q_des_mpc = limitQStep(q_des_mpc, q_des_prev, maxQStep);
    q_des_mpc = clampVector(q_des_mpc, qMin, qMax);
end

% =========================================================================
% Evaluate one candidate trajectory
% =========================================================================
function [J, valid] = evaluateMpcTrajectory( ...
    Qseq, x0, q_ref, q_des_prev, ...
    thetaMin, thetaMax, omegaMax, alphaMax, ...
    wTrack, wTerminal, wSmooth, wMotor, ...
    wSingularity, wOmega, params)

    Np = size(Qseq, 2);

    J = 0;
    valid = true;

    x_pred = x0;
    q_des_prev_i = q_des_prev;

    theta_cmd_prev_i = abenicsIK(q_des_prev_i, params);
    theta_cmd_prev_i = theta_cmd_prev_i(:);

    % Determine whether the real starting state is already singular.
    theta_start = x0(1:4);
    q_start = abenicsFK(theta_start, params);
    q_start = q_start(:);

    [s_start, ~] = singularityMeasure( ...
        theta_start, q_start, params);

    dangerDistance  = params.singularity.dangerDistance;
    warningDistance = params.singularity.warningDistance;

    startedInDanger = s_start < dangerDistance;

    % A normal trajectory starts with the hard danger constraint enabled.
    % A recovery trajectory enables it only after first escaping danger.
    escapedDanger = ~startedInDanger;

    % Used to detect movement back toward the singularity.
    s_prev = s_start;

    q_pred_i = q_start;

    for i = 1:Np
        q_des_i = Qseq(:, i);

        % -----------------------------------------------------------------
        % Desired-orientation constraints
        % -----------------------------------------------------------------
        if any(q_des_i < params.mpc.qMin(:)) || ...
           any(q_des_i > params.mpc.qMax(:))

            valid = false;
            J = inf;
            return;
        end

        % -----------------------------------------------------------------
        % IK and commanded MP-gear angle constraints
        % -----------------------------------------------------------------
        theta_cmd_i = abenicsIK(q_des_i, params);
        theta_cmd_i = theta_cmd_i(:);

        if any(theta_cmd_i < thetaMin) || ...
           any(theta_cmd_i > thetaMax)

            valid = false;
            J = inf;
            return;
        end

        % -----------------------------------------------------------------
        % Internal second-order plant prediction
        % -----------------------------------------------------------------
        [x_pred, alpha_i] = abenicsPlantStepLocal( ...
            x_pred, theta_cmd_i, params);

        theta_pred_i = x_pred(1:4);
        omega_pred_i = x_pred(5:8);

        if any(abs(omega_pred_i) > omegaMax)
            valid = false;
            J = inf;
            return;
        end

        if any(abs(alpha_i) > alphaMax)
            valid = false;
            J = inf;
            return;
        end

        % -----------------------------------------------------------------
        % Predict CS-gear orientation
        % -----------------------------------------------------------------
        q_pred_i = abenicsFK(theta_pred_i, params);
        q_pred_i = q_pred_i(:);

        % -----------------------------------------------------------------
        % Distance-based singularity handling
        % -----------------------------------------------------------------
        [s_i, ~] = singularityMeasure( ...
            theta_pred_i, q_pred_i, params);

        if escapedDanger
            % Normal mode or already recovered:
            % entering or re-entering danger is forbidden.
            if s_i < dangerDistance
                valid = false;
                J = inf;
                return;
            end
        else
            % Recovery mode:
            % the first predicted steps may remain inside danger while the
            % plant gradually follows an escape command.

            dangerDeficit = max(0, dangerDistance - s_i);

            % Remaining inside danger is expensive but not immediately
            % invalid when the actual starting pose is already singular.
            J = J + ...
                10 * wSingularity * dangerDeficit^2;

            % Penalize any predicted step that moves closer to the pole.
            backwardMotion = max(0, s_prev - s_i);

            J = J + ...
                5 * wSingularity * backwardMotion^2;

            % Once recovery crosses the danger boundary, normal hard
            % rejection applies to all later predicted steps.
            if s_i >= dangerDistance
                escapedDanger = true;
            end
        end

        % Apply the warning-region cost exactly once.
        warningDeficit = max(0, warningDistance - s_i);

        J = J + ...
            wSingularity * warningDeficit^2;

        s_prev = s_i;

        % -----------------------------------------------------------------
        % 1. Predicted orientation tracking cost
        % -----------------------------------------------------------------
        q_err_i = wrapAngleDifference(q_pred_i, q_ref);

        J = J + ...
            wTrack * sum(q_err_i.^2);

        % -----------------------------------------------------------------
        % 2. q_des command smoothness cost
        % -----------------------------------------------------------------
        dq_des_i = wrapAngleDifference(q_des_i, q_des_prev_i);

        J = J + ...
            wSmooth * sum(dq_des_i.^2);

        % -----------------------------------------------------------------
        % 3. IK motor-command motion cost
        % -----------------------------------------------------------------
        dtheta_cmd_i = wrapAngleDifference( ...
            theta_cmd_i, theta_cmd_prev_i);

        J = J + ...
            wMotor * sum(dtheta_cmd_i.^2);

        % -----------------------------------------------------------------
        % 4. Predicted MP-gear velocity cost
        % -----------------------------------------------------------------
        J = J + ...
            wOmega * sum(omega_pred_i.^2);

        q_des_prev_i = q_des_i;
        theta_cmd_prev_i = theta_cmd_i;
    end

    % ---------------------------------------------------------------------
    % Recovery trajectory requirement
    % ---------------------------------------------------------------------
    if startedInDanger
        recoveryProgress = s_prev - s_start;

        % The horizon must predict at least a small movement away from the
        % singularity. It does not have to leave danger in one MPC update.
        minRecoveryProgress = 0.05 * dangerDistance;

        if recoveryProgress < minRecoveryProgress
            valid = false;
            J = inf;
            return;
        end

        % A trajectory that improves but remains in danger is allowed so
        % recovery can occur gradually across multiple MPC updates.
        if ~escapedDanger
            terminalDangerDeficit = max( ...
                0, dangerDistance - s_prev);

            J = J + ...
                25 * wSingularity * terminalDangerDeficit^2;
        end
    end

    % ---------------------------------------------------------------------
    % Terminal orientation cost
    % ---------------------------------------------------------------------
    q_err_final = wrapAngleDifference(q_pred_i, q_ref);

    J = J + ...
        wTerminal * sum(q_err_final.^2);
end

% =========================================================================
% Generate parameterized candidate q_des sequences
% =========================================================================
function Qcandidates = generateCandidateSequences( ...
    q_ref, q_des_prev, params, avoidOffset)

    Np = params.mpc.Np;

    qMin = params.mpc.qMin(:);
    qMax = params.mpc.qMax(:);
    maxQStep = expandToVector(params.mpc.maxQStep, 3);

    q_ref = clampVector(q_ref(:), qMin, qMax);
    q_des_prev = clampVector(q_des_prev(:), qMin, qMax);

    % Candidate types:
    %   1      hold the current command
    %   2      move directly toward q_ref
    %   3-10   move through an offset waypoint, then return toward q_ref
    %   11-18  move toward an offset waypoint and hold it
    %
    % The final eight candidates are important when q_ref itself lies on a
    % singular axis. They let the MPC recover without being forced to
    % re-enter danger during the second half of the horizon.
    numCandidates = 18;
    Qcandidates = zeros(3, Np, numCandidates);

    % Candidate 1: hold.
    for i = 1:Np
        Qcandidates(:, i, 1) = q_des_prev;
    end

    % Candidate 2: direct path to q_ref.
    Qcandidates(:, :, 2) = buildBoundedPath( ...
        q_des_prev, q_ref, q_ref, ...
        maxQStep, qMin, qMax, Np, false);

    % Offsets are [roll; pitch; yaw].
    offsets = avoidOffset * [ ...
        0,  0,  0,  0,  0,  0,  0,  0; ...
        1, -1,  0,  0,  1,  1, -1, -1; ...
        0,  0,  1, -1,  1, -1,  1, -1];

    for j = 1:8
        waypoint = clampVector( ...
            q_ref + offsets(:, j), qMin, qMax);

        % Candidates 3-10: detour, then approach q_ref.
        Qcandidates(:, :, 2 + j) = buildBoundedPath( ...
            q_des_prev, waypoint, q_ref, ...
            maxQStep, qMin, qMax, Np, true);

        % Candidates 11-18: escape toward the waypoint and remain there.
        Qcandidates(:, :, 10 + j) = buildBoundedPath( ...
            q_des_prev, waypoint, waypoint, ...
            maxQStep, qMin, qMax, Np, false);
    end
end

% =========================================================================
% Build one smooth, bounded q_des path
% =========================================================================
function Qseq = buildBoundedPath( ...
    q_start, firstTarget, secondTarget, ...
    maxQStep, qMin, qMax, Np, switchAtHalf)

    Qseq = zeros(3, Np);
    q_curr = q_start;

    halfN = max(1, floor(Np / 2));

    for i = 1:Np
        if switchAtHalf && i > halfN
            target_i = secondTarget;
        else
            target_i = firstTarget;
        end

        q_next = stepToward(q_curr, target_i, maxQStep);
        q_next = clampVector(q_next, qMin, qMax);

        Qseq(:, i) = q_next;
        q_curr = q_next;
    end
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

        trackingError = wrapAngleDifference( ...
            q_next, q_ref);

        trackingCost = sum(trackingError.^2);

        % Primary fallback objective: maximize predicted singularity
        % distance. Tracking error is only a tie-breaker.
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
function [x_next, alpha] = abenicsPlantStepLocal( ...
    x, theta_cmd, params)

    Ts = params.Ts;

    theta = x(1:4);
    omega = x(5:8);

    KpPlant = expandToVector(params.plant.KpPlant, 4);
    KdPlant = expandToVector(params.plant.KdPlant, 4);

    theta_error = wrapAngleDifference(theta_cmd, theta);

    % Approximation of lower-level PID + motor + mechanics.
    alpha = KpPlant .* theta_error - ...
            KdPlant .* omega;

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
    else
        v = value;
    end
end
