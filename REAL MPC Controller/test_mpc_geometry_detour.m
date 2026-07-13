% =========================================================================
% test_mpc_geometry_detour.m — V5 clearance-aware detour proof
%
% Closed-loop proof test for the geometry-aware multi-start ABENICS MPC.
%
% Required route:
%   start  = [0;  15; 0] deg
%   target = [0; -15; 0] deg
%
% The direct pitch path crosses the +X singular pole. The controller must
% use a continuously optimized roll/yaw detour while preserving all six hard
% pole-distance constraints. Geometric routes are only fmincon initial guesses.
%
% This script uses the same temporary second-order plant used by the existing
% MPC test. Final validation must later be repeated with the Simulink PID/plant.
% =========================================================================

clear;
clc;
close all;

run("params_abenics_coordinate.m");

% =========================================================================
% TEST CONFIGURATION
% =========================================================================

simulationTime = 3.0;  % long enough to cross, clear, and return toward target
q_start  = deg2rad([0;  15; 0]);
q_target = deg2rad([0; -15; 0]);

params.mpc.Np = 20;
params.mpc.Nc = 6;
params.mpc.maxQStep = deg2rad([2; 2; 2]);
params.mpc.wSingularity = 2000;
params.mpc.maxIterations = 25;
params.mpc.maxFunctionEvaluations = 2000;

params.mpc.enableDetourMultistart = true;
params.mpc.detourTriggerDistance = ...
    params.singularity.warningDistance;
params.mpc.detourClearDistance = ...
    params.singularity.warningDistance;
params.mpc.detourClearances = ...
    params.singularity.warningDistance;  % first proof: one 10 deg clearance
params.mpc.maxDetourStarts = 3;
params.mpc.detourClearConfirmations = 3;
params.mpc.maxDetourFailures = 2;

% Required for fixed numeric MPCUPDATE/MPCSTART diagnostic records.
params.mpc.debug = true;

% Set true to echo every controller diagnostic line during the simulation.
% Keeping it false gives a cleaner and more accurate controller-call timing.
printEveryControllerUpdate = false;

clear abenicsOrientationMPC;

% =========================================================================
% INITIAL STATE
% =========================================================================

numUpdates = round(simulationTime / params.Ts);
numSamples = numUpdates + 1;
time = (0:numUpdates) * params.Ts;

q_des_prev = q_start;
theta_actual = abenicsIK(q_start, params);
theta_actual = theta_actual(:);
omega_actual = zeros(4, 1);

q_des_log = NaN(3, numSamples);
q_pred_log = NaN(3, numSamples);
q_error_log = NaN(3, numSamples);
q_step_log = NaN(3, numSamples);

theta_cmd_log = NaN(4, numSamples);
theta_actual_log = NaN(4, numSamples);
omega_log = NaN(4, numSamples);
alpha_log = NaN(4, numSamples);

singularity_log = NaN(1, numSamples);
pole_distance_log = NaN(6, numSamples);

controller_call_time = NaN(1, numUpdates);
solver_total_time = NaN(1, numUpdates);
solver_worst_start_time = NaN(1, numUpdates);
number_of_solves = NaN(1, numUpdates);
number_of_starts = NaN(1, numUpdates);
route_blocked_log = false(1, numUpdates);
direct_blocked_log = false(1, numUpdates);
detour_active_log = false(1, numUpdates);
committed_side_log = zeros(1, numUpdates);
winning_start_index_log = zeros(1, numUpdates);
winning_start_type_log = 99 * ones(1, numUpdates);
winning_side_log = zeros(1, numUpdates);
winning_route_eligible_log = false(1, numUpdates);
winning_progress_log = zeros(1, numUpdates);
winning_progress_mode_log = zeros(1, numUpdates);
winning_first_progress_log = zeros(1, numUpdates);
winning_first_pole_log = NaN(1, numUpdates);
winning_near_pole_log = NaN(1, numUpdates);
winning_minimum_near_pole_log = NaN(1, numUpdates);
accepted_log = false(1, numUpdates);
fallback_log = false(1, numUpdates);
emergency_log = false(1, numUpdates);
update_record_found = false(1, numUpdates);

emptyStartRecord = struct( ...
    'updateIndex', 0, ...
    'time_s', 0, ...
    'startIndex', 0, ...
    'startType', 99, ...
    'poleIndex', 0, ...
    'side', 0, ...
    'clearance_rad', NaN, ...
    'seedValid', false, ...
    'seedBlendScale', NaN, ...
    'seedConstraintViolation', NaN, ...
    'exitflag', NaN, ...
    'accepted', false, ...
    'routeEligible', false, ...
    'progressMode', 0, ...
    'progressSide', 0, ...
    'firstProgress_rad', 0, ...
    'progress_rad', 0, ...
    'firstPoleDistance_rad', NaN, ...
    'nearPoleDistance_rad', NaN, ...
    'minimumNearPoleDistance_rad', NaN, ...
    'objective', NaN, ...
    'minimumPoleDistance_rad', NaN, ...
    'iterations', 0, ...
    'functionEvaluations', 0, ...
    'solveTime_s', 0, ...
    'rejectionCode', 0);
startRecords = repmat(emptyStartRecord, 0, 1);

% Log the true initial state at t = 0 before any controller or plant update.
q_initial = abenicsFK(theta_actual, params);
q_initial = q_initial(:);
[s_initial, info_initial] = singularityMeasure( ...
    theta_actual, q_initial, params);
theta_cmd_initial = abenicsIK(q_des_prev, params);

q_des_log(:, 1) = q_des_prev;
q_pred_log(:, 1) = q_initial;
q_error_log(:, 1) = wrappedDifference(q_initial, q_target);
q_step_log(:, 1) = zeros(3, 1);
theta_cmd_log(:, 1) = theta_cmd_initial(:);
theta_actual_log(:, 1) = theta_actual;
omega_log(:, 1) = omega_actual;
alpha_log(:, 1) = zeros(4, 1);
singularity_log(1) = s_initial;
pole_distance_log(:, 1) = info_initial.poleDistances(:);

% =========================================================================
% CLOSED-LOOP SIMULATION
% =========================================================================

for updateIndex = 1:numUpdates

    callTimer = tic;
    debugText = evalc([ ...
        'q_des_mpc = abenicsOrientationMPC(' ...
        'q_target, theta_actual, q_des_prev, params);' ...
        ]);
    controller_call_time(updateIndex) = toc(callTimer);

    if printEveryControllerUpdate
        fprintf('%s', debugText);
    end

    updateLine = regexp( ...
        debugText, 'MPCUPDATE[^\r\n]*', 'match', 'once');

    if ~isempty(updateLine)
        update_record_found(updateIndex) = true;
        route_blocked_log(updateIndex) = logical( ...
            debugNumber(updateLine, 'blocked', 0));
        direct_blocked_log(updateIndex) = logical( ...
            debugNumber(updateLine, 'directBlocked', 0));
        detour_active_log(updateIndex) = logical( ...
            debugNumber(updateLine, 'detourActive', 0));
        committed_side_log(updateIndex) = ...
            debugNumber(updateLine, 'committedSide', 0);
        number_of_starts(updateIndex) = ...
            debugNumber(updateLine, 'starts', NaN);
        number_of_solves(updateIndex) = ...
            debugNumber(updateLine, 'solves', NaN);
        winning_start_index_log(updateIndex) = ...
            debugNumber(updateLine, 'winnerIndex', 0);
        winning_start_type_log(updateIndex) = ...
            debugNumber(updateLine, 'winnerType', 99);
        winning_side_log(updateIndex) = ...
            debugNumber(updateLine, 'winnerSide', 0);
        winning_route_eligible_log(updateIndex) = logical( ...
            debugNumber(updateLine, 'winnerRouteEligible', 0));
        winning_progress_log(updateIndex) = ...
            debugNumber(updateLine, 'winnerProgress', 0);
        winning_progress_mode_log(updateIndex) = ...
            debugNumber(updateLine, 'winnerProgressMode', 0);
        winning_first_progress_log(updateIndex) = ...
            debugNumber(updateLine, 'winnerFirstProgress', 0);
        winning_first_pole_log(updateIndex) = ...
            debugNumber(updateLine, 'winnerFirstPole', NaN);
        winning_near_pole_log(updateIndex) = ...
            debugNumber(updateLine, 'winnerNearPole', NaN);
        winning_minimum_near_pole_log(updateIndex) = ...
            debugNumber(updateLine, 'winnerMinimumNearPole', NaN);
        accepted_log(updateIndex) = logical( ...
            debugNumber(updateLine, 'accepted', 0));
        fallback_log(updateIndex) = logical( ...
            debugNumber(updateLine, 'fallback', 0));
        emergency_log(updateIndex) = logical( ...
            debugNumber(updateLine, 'recovery', 0));
        solver_total_time(updateIndex) = ...
            debugNumber(updateLine, 'totalSolveTime', NaN);
        solver_worst_start_time(updateIndex) = ...
            debugNumber(updateLine, 'worstStartSolveTime', NaN);
    end

    startLines = regexp(debugText, 'MPCSTART[^\r\n]*', 'match');
    for lineIndex = 1:numel(startLines)
        line = startLines{lineIndex};
        record = emptyStartRecord;
        record.updateIndex = updateIndex;
        record.time_s = time(updateIndex);
        record.startIndex = debugNumber(line, 'index', 0);
        record.startType = debugNumber(line, 'type', 99);
        record.poleIndex = debugNumber(line, 'pole', 0);
        record.side = debugNumber(line, 'side', 0);
        record.clearance_rad = debugNumber(line, 'clearance', NaN);
        record.seedValid = logical(debugNumber(line, 'seedValid', 0));
        record.seedBlendScale = debugNumber(line, 'seedBlend', NaN);
        record.seedConstraintViolation = ...
            debugNumber(line, 'seedViolation', NaN);
        record.exitflag = debugNumber(line, 'exit', NaN);
        record.accepted = logical(debugNumber(line, 'accepted', 0));
        record.routeEligible = logical( ...
            debugNumber(line, 'routeEligible', 0));
        record.progressMode = debugNumber(line, 'progressMode', 0);
        record.progressSide = debugNumber(line, 'progressSide', 0);
        record.firstProgress_rad = ...
            debugNumber(line, 'firstProgress', 0);
        record.progress_rad = debugNumber(line, 'progress', 0);
        record.firstPoleDistance_rad = ...
            debugNumber(line, 'firstPole', NaN);
        record.nearPoleDistance_rad = ...
            debugNumber(line, 'nearPole', NaN);
        record.minimumNearPoleDistance_rad = ...
            debugNumber(line, 'minNearPole', NaN);
        record.objective = debugNumber(line, 'objective', NaN);
        record.minimumPoleDistance_rad = ...
            debugNumber(line, 'minPole', NaN);
        record.iterations = debugNumber(line, 'iterations', 0);
        record.functionEvaluations = ...
            debugNumber(line, 'evaluations', 0);
        record.solveTime_s = debugNumber(line, 'solveTime', 0);
        record.rejectionCode = debugNumber(line, 'reject', 0);
        startRecords(end + 1, 1) = record; %#ok<SAGROW>
    end

    q_des_mpc = q_des_mpc(:);
    theta_cmd = abenicsIK(q_des_mpc, params);
    theta_cmd = theta_cmd(:);

    [theta_actual, omega_actual, alpha] = temporaryPlantStep( ...
        theta_actual, omega_actual, theta_cmd, params);

    q_pred = abenicsFK(theta_actual, params);
    q_pred = q_pred(:);
    [s, info] = singularityMeasure(theta_actual, q_pred, params);

    sampleIndex = updateIndex + 1;
    q_des_log(:, sampleIndex) = q_des_mpc;
    q_pred_log(:, sampleIndex) = q_pred;
    q_error_log(:, sampleIndex) = ...
        wrappedDifference(q_pred, q_target);
    q_step_log(:, sampleIndex) = ...
        wrappedDifference(q_des_mpc, q_des_prev);

    theta_cmd_log(:, sampleIndex) = theta_cmd;
    theta_actual_log(:, sampleIndex) = theta_actual;
    omega_log(:, sampleIndex) = omega_actual;
    alpha_log(:, sampleIndex) = alpha;

    singularity_log(sampleIndex) = s;
    pole_distance_log(:, sampleIndex) = info.poleDistances(:);

    q_des_prev = q_des_mpc;
end

if ~all(update_record_found)
    warning('test_mpc_geometry_detour:MissingDiagnostics', ...
        ['MPCUPDATE records were missing for %d controller calls. ', ...
         'Confirm that the updated controller is on the MATLAB path.'], ...
        sum(~update_record_found));
end

% =========================================================================
% METRICS AND VALIDATION
% =========================================================================

q_des_deg = rad2deg(q_des_log);
q_pred_deg = rad2deg(q_pred_log);
q_error_deg = rad2deg(q_error_log);
q_step_deg = rad2deg(q_step_log);
pole_distance_deg = rad2deg(pole_distance_log);

minimumDistanceByPole_deg = min(pole_distance_deg, [], 2);
minimumDistance_deg = min(minimumDistanceByPole_deg);
minimumDistance_rad = min(pole_distance_log(:));

pitchCrossed = false;
pitchCrossingTime_s = NaN;
for sampleIndex = 1:numSamples - 1
    pitchA = q_pred_deg(2, sampleIndex);
    pitchB = q_pred_deg(2, sampleIndex + 1);
    if pitchA > 0 && pitchB <= 0
        pitchCrossed = true;
        denominator = pitchA - pitchB;
        if abs(denominator) > eps
            fraction = pitchA / denominator;
        else
            fraction = 0;
        end
        pitchCrossingTime_s = ...
            time(sampleIndex) + fraction * params.Ts;
        break;
    end
end

maximumRollDetour_deg = max(abs(q_pred_deg(1, :)));
maximumYawDetour_deg = max(abs(q_pred_deg(3, :)));
finalOrientationError_deg = abs(q_error_deg(:, end));
finalRoll_deg = q_pred_deg(1, end);
finalPitch_deg = q_pred_deg(2, end);
finalYaw_deg = q_pred_deg(3, end);

qCommandViolation = maximumViolation( ...
    q_des_log, params.mpc.qMin(:), params.mpc.qMax(:));
thetaCommandViolation = maximumViolation( ...
    theta_cmd_log, params.mpc.thetaMin(:), params.mpc.thetaMax(:));
thetaActualViolation = maximumViolation( ...
    theta_actual_log, params.mpc.thetaMin(:), params.mpc.thetaMax(:));
omegaViolation = maximumViolation( ...
    omega_log, -params.mpc.omegaMax(:), params.mpc.omegaMax(:));
alphaViolation = maximumViolation( ...
    alpha_log, -params.mpc.alphaMax(:), params.mpc.alphaMax(:));

maxQStep = params.mpc.maxQStep(:);
qStepViolation = max([0; ...
    reshape(q_step_log - repmat(maxQStep, 1, numSamples), [], 1); ...
    reshape(-q_step_log - repmat(maxQStep, 1, numSamples), [], 1)]);

dangerViolation = max( ...
    params.singularity.dangerDistance - minimumDistance_rad, 0);

nonfiniteCount = sum(~isfinite([ ...
    q_des_log(:); q_pred_log(:); theta_cmd_log(:); ...
    theta_actual_log(:); omega_log(:); alpha_log(:); ...
    singularity_log(:); pole_distance_log(:)]));

maximumPhysicalViolation = max([ ...
    qCommandViolation, qStepViolation, thetaCommandViolation, ...
    thetaActualViolation, omegaViolation, alphaViolation, ...
    dangerViolation]);

fallbackCount = sum(fallback_log);
acceptedCount = sum(accepted_log);
emergencyCount = sum(emergency_log);

validControllerTimes = controller_call_time(isfinite(controller_call_time));
validSolveTimes = solver_total_time(isfinite(solver_total_time));
validWorstStartTimes = ...
    solver_worst_start_time(isfinite(solver_worst_start_time));
validSolveCounts = number_of_solves(isfinite(number_of_solves));

averageControllerCallTime_s = safeMean(validControllerTimes);
worstControllerCallTime_s = safeMax(validControllerTimes);
averageSolveTimePerUpdate_s = safeMean(validSolveTimes);
worstSolveTimePerUpdate_s = safeMax(validSolveTimes);
worstIndividualStartTime_s = safeMax(validWorstStartTimes);
averageSolvesPerUpdate = safeMean(validSolveCounts);

success = ...
    pitchCrossed && ...
    finalPitch_deg < 0 && ...
    minimumDistance_rad >= ...
        params.singularity.dangerDistance - 1e-9 && ...
    emergencyCount == 0 && ...
    maximumPhysicalViolation <= ...
        params.mpc.constraintTolerance + 1e-9 && ...
    nonfiniteCount == 0;

% =========================================================================
% SUMMARY OUTPUT
% =========================================================================

fprintf('\n============================================================\n');
fprintf('ABENICS V5 CLEARANCE-AWARE MULTI-START DETOUR TEST\n');
fprintf('============================================================\n');
fprintf('Start:                    [0,  15, 0] deg\n');
fprintf('Target:                   [0, -15, 0] deg\n');
fprintf('Prediction/control:       Np=%d, Nc=%d\n', ...
    params.mpc.Np, params.mpc.Nc);
fprintf('Maximum q step:           [%.2f, %.2f, %.2f] deg\n', ...
    rad2deg(params.mpc.maxQStep));
fprintf('Detour clearance:         %.4f deg\n', ...
    rad2deg(params.mpc.detourClearances(1)));
fprintf('Pitch crossed zero:       %d\n', pitchCrossed);
fprintf('Pitch crossing time:      %.6f s\n', pitchCrossingTime_s);
fprintf('Maximum roll detour:      %.6f deg\n', maximumRollDetour_deg);
fprintf('Maximum yaw detour:       %.6f deg\n', maximumYawDetour_deg);
fprintf('Final orientation:        [%.6f, %.6f, %.6f] deg\n', ...
    finalRoll_deg, finalPitch_deg, finalYaw_deg);
fprintf('Final absolute error:     [%.6f, %.6f, %.6f] deg\n', ...
    finalOrientationError_deg);
fprintf('Minimum six-pole distance %.6f deg\n', minimumDistance_deg);
fprintf('Fallback count:           %d\n', fallbackCount);
fprintf('Accepted update count:    %d / %d\n', ...
    acceptedCount, numUpdates);
fprintf('Emergency recovery count: %d\n', emergencyCount);
fprintf('Nonfinite value count:    %d\n', nonfiniteCount);
fprintf('Maximum violation:        %.3e rad\n', ...
    maximumPhysicalViolation);
fprintf('Average controller call:  %.6f s\n', ...
    averageControllerCallTime_s);
fprintf('Worst controller call:    %.6f s\n', ...
    worstControllerCallTime_s);
fprintf('Average solve/update:     %.6f s\n', ...
    averageSolveTimePerUpdate_s);
fprintf('Worst solve/update:       %.6f s\n', ...
    worstSolveTimePerUpdate_s);
fprintf('Worst individual start:   %.6f s\n', ...
    worstIndividualStartTime_s);
fprintf('Average solves/update:    %.3f\n', averageSolvesPerUpdate);
fprintf('Route-eligible winners:  %d / %d\n', ...
    sum(winning_route_eligible_log), numUpdates);
fprintf('Maximum winning first physical progress: %.6f deg\n', ...
    rad2deg(max(winning_first_progress_log)));
fprintf('Maximum winning near-term physical progress: %.6f deg\n', ...
    rad2deg(max(winning_progress_log)));
finiteWinnerNearPole = winning_minimum_near_pole_log( ...
    isfinite(winning_minimum_near_pole_log));
if isempty(finiteWinnerNearPole)
    minimumWinningNearPole_deg = NaN;
else
    minimumWinningNearPole_deg = rad2deg(min(finiteWinnerNearPole));
end
fprintf('Minimum winning near-term pole distance: %.6f deg\n', ...
    minimumWinningNearPole_deg);
fprintf('OVERALL BASIC PASS:       %d\n', success);

poleNames = ["+X"; "-X"; "+Y"; "-Y"; "+Z"; "-Z"];
poleTable = table( ...
    (1:6).', poleNames, minimumDistanceByPole_deg, ...
    'VariableNames', { ...
        'PoleIndex', 'PoleAxis', 'MinimumDistance_deg'});

fprintf('\nMinimum distance to each physical pole:\n');
disp(poleTable);

if isempty(startRecords)
    startTable = table();
    fprintf('No MPCSTART records were found.\n');
else
    startTable = struct2table(startRecords);
    startTable.clearance_deg = rad2deg(startTable.clearance_rad);
    startTable.minimumPoleDistance_deg = ...
        rad2deg(startTable.minimumPoleDistance_rad);
    startTable.firstProgress_deg = ...
        rad2deg(startTable.firstProgress_rad);
    startTable.progress_deg = rad2deg(startTable.progress_rad);
    startTable.firstPoleDistance_deg = ...
        rad2deg(startTable.firstPoleDistance_rad);
    startTable.nearPoleDistance_deg = ...
        rad2deg(startTable.nearPoleDistance_rad);
    startTable.minimumNearPoleDistance_deg = ...
        rad2deg(startTable.minimumNearPoleDistance_rad);

    startTypeName = strings(height(startTable), 1);
    rejectionName = strings(height(startTable), 1);
    for row = 1:height(startTable)
        startTypeName(row) = startTypeCodeName( ...
            startTable.startType(row));
        rejectionName(row) = rejectionCodeName( ...
            startTable.rejectionCode(row));
    end
    startTable.startTypeName = startTypeName;
    startTable.rejectionName = rejectionName;

    startTable = movevars(startTable, ...
        {'startTypeName', 'rejectionName', ...
         'clearance_deg', 'minimumPoleDistance_deg', ...
         'firstProgress_deg', 'progress_deg', ...
         'firstPoleDistance_deg', 'nearPoleDistance_deg', ...
         'minimumNearPoleDistance_deg'}, ...
        'After', 'startType');

    fprintf('\nFirst attempted MPC starts (full startTable remains in workspace):\n');
    maximumDisplayedRows = min(36, height(startTable));
    disp(startTable(1:maximumDisplayedRows, :));

    rejectionCodes = unique(startTable.rejectionCode);
    rejectionCounts = zeros(numel(rejectionCodes), 1);
    firstRejectionUpdate = zeros(numel(rejectionCodes), 1);
    rejectionNames = strings(numel(rejectionCodes), 1);
    for codeIndex = 1:numel(rejectionCodes)
        thisCode = rejectionCodes(codeIndex);
        matchingRows = find(startTable.rejectionCode == thisCode);
        rejectionCounts(codeIndex) = numel(matchingRows);
        firstRejectionUpdate(codeIndex) = ...
            startTable.updateIndex(matchingRows(1));
        rejectionNames(codeIndex) = rejectionCodeName(thisCode);
    end
    rejectionSummary = table( ...
        rejectionCodes, rejectionNames, rejectionCounts, ...
        firstRejectionUpdate, ...
        'VariableNames', { ...
            'RejectionCode', 'RejectionName', 'Count', ...
            'FirstUpdate'});
    fprintf('\nStart-result code summary:\n');
    disp(rejectionSummary);
end

% =========================================================================
% PLOTS
% =========================================================================

figure('Name', 'ABENICS Detour Pole Distance');
plot(time, rad2deg(singularity_log), 'LineWidth', 1.4);
hold on;
yline(rad2deg(params.singularity.warningDistance), ...
    '--', 'Warning distance', 'LineWidth', 1.2);
yline(rad2deg(params.singularity.dangerDistance), ...
    '--', 'Danger distance', 'LineWidth', 1.2);
grid on;
xlabel('Time (s)');
ylabel('Nearest pole distance (deg)');
title('Geometry-Aware Detour Safety');

figure('Name', 'ABENICS Detour Orientation');
subplot(3, 1, 1);
plot(time, q_pred_deg(1, :), 'LineWidth', 1.3);
hold on;
yline(rad2deg(q_target(1)), '--', 'Target');
grid on;
ylabel('Roll (deg)');
title('Temporary Roll Detour');

subplot(3, 1, 2);
plot(time, q_pred_deg(2, :), 'LineWidth', 1.3);
hold on;
yline(rad2deg(q_target(2)), '--', 'Target');
yline(0, ':');
grid on;
ylabel('Pitch (deg)');
title('Pitch Crossing');

subplot(3, 1, 3);
plot(time, q_pred_deg(3, :), 'LineWidth', 1.3);
hold on;
yline(rad2deg(q_target(3)), '--', 'Target');
grid on;
xlabel('Time (s)');
ylabel('Yaw (deg)');
title('Temporary Yaw Detour');

figure('Name', 'ABENICS Detour Runtime');
subplot(2, 1, 1);
plot(time(1:end-1), solver_total_time, 'LineWidth', 1.2);
hold on;
yline(params.Ts, '--', 'Controller Ts');
grid on;
ylabel('Total fmincon time (s)');
title('Optimization Time per MPC Update');

subplot(2, 1, 2);
stairs(time(1:end-1), number_of_solves, 'LineWidth', 1.2);
grid on;
xlabel('Time (s)');
ylabel('fmincon solves');
title('Number of Starts Solved per Update');

% =========================================================================
% LOCAL FUNCTIONS
% =========================================================================

function value = debugNumber(line, key, defaultValue)

    expression = [regexptranslate('escape', key), '=([^\s|]+)'];
    token = regexp(line, expression, 'tokens', 'once');
    if isempty(token)
        value = defaultValue;
        return;
    end

    value = str2double(token{1});
    if isnan(value) && ~strcmpi(token{1}, 'NaN')
        value = defaultValue;
    end
end


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
    elseif numel(value) == n
        v = value;
    else
        error('test_mpc_geometry_detour:parameterSize', ...
            'Expected scalar or %dx1 value.', n);
    end
end


function violation = maximumViolation(values, lower, upper)

    lower = lower(:);
    upper = upper(:);
    lowerGrid = repmat(lower, 1, size(values, 2));
    upperGrid = repmat(upper, 1, size(values, 2));
    violation = max([0; ...
        reshape(lowerGrid - values, [], 1); ...
        reshape(values - upperGrid, [], 1)]);
end


function value = safeMean(values)
    if isempty(values)
        value = NaN;
    else
        value = mean(values);
    end
end


function value = safeMax(values)
    if isempty(values)
        value = NaN;
    else
        value = max(values);
    end
end


function name = startTypeCodeName(code)
    if code == 0
        name = "shifted";
    elseif code == 1
        name = "positive";
    elseif code == -1
        name = "negative";
    elseif code == 99
        name = "unused";
    else
        name = "unknown";
    end
end


function name = rejectionCodeName(code)
    switch code
        case 0
            name = "accepted";
        case 4
            name = "solver exception";
        case 5
            name = "negative exitflag";
        case 6
            name = "nonfinite solution";
        case 7
            name = "nonfinite objective";
        case 8
            name = "nonfinite rollout";
        case 9
            name = "nonlinear constraint violation";
        case 10
            name = "invalid first command";
        case 11
            name = "fmincon unavailable";
        case 12
            name = "unclassified optimized rejection";
        case 13
            name = "unsafe first plant transition";
        case 101
            name = "bad seed input";
        case 102
            name = "bad pole";
        case 103
            name = "bad clearance";
        case 104
            name = "unsafe endpoint";
        case 105
            name = "tangent basis failure";
        case 106
            name = "unsafe raw axis path";
        case 107
            name = "rotation path failure";
        case 108
            name = "unsafe orientation path";
        case 109
            name = "seed q-limit failure";
        case 110
            name = "seed resampling failure";
        case 111
            name = "no detour-side progress";
        case 201
            name = "nonfinite seed rollout";
        case 202
            name = "seed rollout q-limit failure";
        case 203
            name = "seed rollout pole failure";
        case 204
            name = "no side-preserving repaired seed";
        case 205
            name = "unusable repaired seed";
        otherwise
            name = "unknown code " + string(code);
    end
end
