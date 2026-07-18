% =========================================================================
% test_mpc_cem_single_axis_sines.m
%
% ABENICS SO(3) CEM MPC — SINGLE-AXIS SINE-WAVE TRACKING TEST
%
% Purpose:
%   Test normal tracking performance with optional future-reference preview
%   without retuning singularity values.
%   Runs roll-only, pitch-only, and yaw-only sine references using the same
%   controller settings and random seed.
%
% Required files in the current MATLAB folder:
%   abenicsOrientationMPC.m   % working v2.2 SO(3) CEM controller
%   abenicsIK.m
%   abenicsFL.m               % preferred project FK filename
%     OR abenicsFK.m          % accepted fallback filename
%   params_abenics_coordinate.m
%     OR params_abenics.m
%
% Main reported metrics:
%   - RMS and peak tracking error for all three orientation components
%   - Driven-axis amplitude ratio and phase lag
%   - Maximum physical q_cmd step
%   - Maximum motor velocity and acceleration
%   - Minimum distance to all six singular poles
%   - Accepted/fallback/recovery counts
%   - Average and worst controller runtime
%
% IMPORTANT:
%   The singularity settings are deliberately frozen:
%       dangerDistance  = 2 deg
%       warningDistance = 10 deg
%       wSingularity    = 4000
%   Do not use this script to tune those values.
% =========================================================================

clear;
clc;
close all;

%% ========================================================================
%  USER SETTINGS — CHANGE ONLY THIS SECTION
%  ========================================================================

% "smoke" runs one full 0.2 Hz period plus a small margin.
% "full" runs three periods and evaluates steady-state after the first cycle.
testMode = "smoke";                  % "smoke" or "full"

axesToTest = ["roll", "pitch", "yaw"];
randomSeed = 1;

% Proper moving-reference preview. Keep true for the upgraded controller.
% Set false only when deliberately reproducing the old no-preview baseline.
useReferencePreview = true;

centerDeg = [20; 20; 20];
amplitudeDeg = 5;
frequencyHz = 0.20;

% Controller horizon selected after the horizon study.
finalNp = 33;
finalNc = 12;

% Tracking acceptance targets. These do not change the controller.
rmsErrorLimitDeg = 0.50;
peakErrorLimitDeg = 1.00;
amplitudeRatioLimits = [0.95, 1.05];

% Save summary table for comparing future weight changes.
saveResultsCsv = true;
resultsCsvFile = "single_axis_sine_reference_preview_results.csv";

%% ========================================================================
%  TEST DURATION
%  ========================================================================

periodSeconds = 1 / frequencyHz;

switch lower(testMode)
    case "smoke"
        simulationTime = 1.2 * periodSeconds;
        steadyStateStartTime = 0;

    case "full"
        simulationTime = 3.0 * periodSeconds;
        steadyStateStartTime = periodSeconds;

    otherwise
        error('SineTrackingTest:BadMode', ...
            'testMode must be "smoke" or "full".');
end

%% ========================================================================
%  LOAD PARAMETERS
%  ========================================================================

if isfile("params_abenics_coordinate.m")
    run("params_abenics_coordinate.m");
elseif isfile("params_abenics.m")
    run("params_abenics.m");
else
    error('SineTrackingTest:MissingParams', ...
        ['Could not find params_abenics_coordinate.m or ', ...
         'params_abenics.m in the current folder.']);
end

%% ========================================================================
%  FREEZE CONTROLLER AND SINGULARITY SETTINGS
%  ========================================================================

params.mpc.Np = finalNp;
params.mpc.Nc = finalNc;

params.singularity.dangerDistance = deg2rad(2.0);
params.singularity.warningDistance = deg2rad(10.0);
params.mpc.wSingularity = 4000;

% Confirmed project test range.
params.mpc.qMin = deg2rad([-720; -720; -720]);
params.mpc.qMax = deg2rad([ 720;  720;  720]);
params.mpc.thetaMin = deg2rad([-360; -360; -360; -360]);
params.mpc.thetaMax = deg2rad([ 360;  360;  360;  360]);
params.mpc.thetaUnwrapEnabled = true;
params.mpc.thetaPeriodic = true(4, 1);
params.mpc.enforceThetaPositionLimits = true;

% Keep the known v2.2 CEM configuration.
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

params.mpc.debug = false;
params.mpc.liveProgress = false;
params.mpc.enableTestDiagnostics = true;
params.mpc.useReferencePreview = useReferencePreview;

%% ========================================================================
%  PRINT TEST CONFIGURATION
%  ========================================================================

fprintf('\n============================================================\n');
fprintf('ABENICS SINGLE-AXIS SINE TRACKING TEST\n');
fprintf('============================================================\n');
fprintf('Mode:                     %s\n', testMode);
fprintf('Axes:                     %s\n', strjoin(axesToTest, ', '));
fprintf('Center orientation:       [%.1f %.1f %.1f] deg\n', centerDeg);
fprintf('Amplitude:                %.2f deg\n', amplitudeDeg);
fprintf('Frequency:                %.3f Hz\n', frequencyHz);
fprintf('Period:                   %.3f s\n', periodSeconds);
fprintf('Simulation time/case:     %.3f s\n', simulationTime);
fprintf('Steady-state window start %.3f s\n', steadyStateStartTime);
fprintf('Controller horizon:       Np=%d, Nc=%d\n', finalNp, finalNc);
fprintf('Prediction duration:      %.3f s\n', finalNp * params.Ts);
fprintf('Reference preview:        %d\n', useReferencePreview);
fprintf('Frozen singularity setup: hard=2 deg, warning=10 deg, weight=4000\n');
fprintf('============================================================\n');

if isfield(params.mpc, 'wTrack')
    fprintf('wTrack:       %.6g\n', params.mpc.wTrack);
end
if isfield(params.mpc, 'wTerminal')
    fprintf('wTerminal:    %.6g\n', params.mpc.wTerminal);
end
if isfield(params.mpc, 'wSmooth')
    fprintf('wSmooth:      %.6g\n', params.mpc.wSmooth);
end
if isfield(params.mpc, 'wMotor')
    fprintf('wMotor:       %.6g\n', params.mpc.wMotor);
end
if isfield(params.mpc, 'wOmega')
    fprintf('wOmega:       %.6g\n', params.mpc.wOmega);
end

%% ========================================================================
%  STORAGE
%  ========================================================================

axisNames = ["roll", "pitch", "yaw"];
numberOfCases = numel(axesToTest);

drivenAxisIndex = zeros(numberOfCases, 1);
rmsErrorDeg = NaN(numberOfCases, 3);
peakErrorDeg = NaN(numberOfCases, 3);
physicalRmsErrorDeg = NaN(numberOfCases, 1);
physicalPeakErrorDeg = NaN(numberOfCases, 1);
amplitudeRatio = NaN(numberOfCases, 1);
phaseLagDeg = NaN(numberOfCases, 1);
phaseLagSeconds = NaN(numberOfCases, 1);
maximumCommandStepDeg = NaN(numberOfCases, 1);
maximumMotorVelocityDegPerSec = NaN(numberOfCases, 1);
maximumMotorAccelerationDegPerSec2 = NaN(numberOfCases, 1);
minimumPoleDistanceDeg = NaN(numberOfCases, 1);
acceptedUpdates = zeros(numberOfCases, 1);
fallbackCount = zeros(numberOfCases, 1);
recoveryCount = zeros(numberOfCases, 1);
stagnationResetCount = zeros(numberOfCases, 1);
nonfiniteCount = zeros(numberOfCases, 1);
averageControllerCall = NaN(numberOfCases, 1);
worstControllerCall = NaN(numberOfCases, 1);
casePass = false(numberOfCases, 1);

caseLogs = cell(numberOfCases, 1);

global ABENICS_CEM_LAST_DIAGNOSTICS

%% ========================================================================
%  RUN EACH SINGLE-AXIS TEST
%  ========================================================================

for caseIndex = 1:numberOfCases
    axisName = lower(string(axesToTest(caseIndex)));
    axisIndex = find(axisNames == axisName, 1);

    if isempty(axisIndex)
        error('SineTrackingTest:BadAxis', ...
            'Unknown axis "%s". Use roll, pitch, or yaw.', axisName);
    end
    drivenAxisIndex(caseIndex) = axisIndex;

    fprintf('\n============================================================\n');
    fprintf('CASE %d/%d — %s-ONLY SINE\n', ...
        caseIndex, numberOfCases, upper(axisName));
    fprintf('============================================================\n');

    % Same random seed for every axis so comparisons are fair.
    rng(randomSeed, 'twister');
    clear abenicsOrientationMPC;
    ABENICS_CEM_LAST_DIAGNOSTICS = [];

    numberOfUpdates = round(simulationTime / params.Ts);
    numberOfSamples = numberOfUpdates + 1;
    time = (0:numberOfUpdates) * params.Ts;

    qRefLog = zeros(3, numberOfSamples);
    qActualLog = zeros(3, numberOfSamples);
    qCommandLog = zeros(3, numberOfUpdates);
    thetaLog = zeros(4, numberOfSamples);
    omegaLog = zeros(4, numberOfSamples);
    alphaLog = zeros(4, numberOfUpdates);
    physicalErrorLog = zeros(1, numberOfSamples);
    poleDistanceLog = inf(1, numberOfSamples);
    solveTimeLog = NaN(1, numberOfUpdates);
    acceptedLog = false(1, numberOfUpdates);
    fallbackLog = false(1, numberOfUpdates);
    recoveryLog = false(1, numberOfUpdates);
    resetLog = false(1, numberOfUpdates);

    % Start exactly at the center orientation.
    qDesPrevious = deg2rad(centerDeg);
    thetaActual = abenicsIK(qDesPrevious, params);
    thetaActual = thetaActual(:);
    omegaActual = zeros(4, 1);

    versionChecked = false;

    for sampleIndex = 1:numberOfSamples
        currentTime = time(sampleIndex);

        qReferenceDeg = centerDeg;
        qReferenceDeg(axisIndex) = centerDeg(axisIndex) + ...
            amplitudeDeg * sin(2*pi*frequencyHz*currentTime);
        qReference = deg2rad(qReferenceDeg);

        qActual = callAbenicsFK(thetaActual, params);
        qActual = qActual(:);

        qRefLog(:, sampleIndex) = qReference;
        qActualLog(:, sampleIndex) = qActual;
        thetaLog(:, sampleIndex) = thetaActual;
        omegaLog(:, sampleIndex) = omegaActual;

        actualRotation = qToRotmXYZLocal(qActual);
        referenceRotation = qToRotmXYZLocal(qReference);
        physicalErrorLog(sampleIndex) = ...
            rotationDistanceLocal(actualRotation, referenceRotation);
        poleDistanceLog(sampleIndex) = ...
            minimumPoleDistanceLocal(actualRotation, params);

        if sampleIndex > numberOfUpdates
            break;
        end

        % Build the future reference sequence used by predicted steps
        % 1...Np. Step i corresponds to t + i*Ts.
        if useReferencePreview
            futureTime = currentTime + ...
                (1:params.mpc.Np) * params.Ts;
            qRefHorizonDeg = repmat( ...
                centerDeg, 1, params.mpc.Np);
            qRefHorizonDeg(axisIndex, :) = ...
                centerDeg(axisIndex) + amplitudeDeg * ...
                sin(2*pi*frequencyHz*futureTime);
            params.mpc.qRefHorizon = deg2rad(qRefHorizonDeg);
        else
            params.mpc.qRefHorizon = repmat( ...
                qReference, 1, params.mpc.Np);
        end

        qCommand = abenicsOrientationMPC( ...
            qReference, thetaActual, qDesPrevious, params);
        qCommandLog(:, sampleIndex) = qCommand(:);

        diagnostics = ABENICS_CEM_LAST_DIAGNOSTICS;

        if isstruct(diagnostics)
            if ~versionChecked && isfield(diagnostics, 'version')
                if abs(diagnostics.version - 2.2) > 1e-9
                    error('SineTrackingTest:WrongController', ...
                        ['This script requires the working v2.2 controller. ', ...
                         'Loaded diagnostics.version = %.3f.'], ...
                        diagnostics.version);
                end
                versionChecked = true;
            end

            if isfield(diagnostics, 'accepted')
                acceptedLog(sampleIndex) = logical(diagnostics.accepted);
            end
            if isfield(diagnostics, 'fallbackUsed')
                fallbackLog(sampleIndex) = ...
                    logical(diagnostics.fallbackUsed);
            end
            if isfield(diagnostics, 'recoveryUsed')
                recoveryLog(sampleIndex) = ...
                    logical(diagnostics.recoveryUsed);
            end
            if isfield(diagnostics, 'stagnationResetTriggered')
                resetLog(sampleIndex) = ...
                    logical(diagnostics.stagnationResetTriggered);
            end
            if isfield(diagnostics, 'solveTime')
                solveTimeLog(sampleIndex) = diagnostics.solveTime;
            end
        end

        thetaCommand = abenicsIK(qCommand, params);
        thetaCommand = unwrapThetaNearestLocal( ...
            thetaCommand(:), thetaActual, params);

        [thetaNext, omegaNext, alphaActual] = plantStepLocal( ...
            thetaActual, omegaActual, thetaCommand, params);

        % Include interpolated transition clearance, not only endpoint
        % clearance, so a between-sample pole crossing cannot be missed.
        transitionMinimum = transitionPoleMinimumLocal( ...
            thetaActual, thetaNext, ...
            params.mpc.transitionSafetySamples, params);

        thetaActual = thetaNext;
        omegaActual = omegaNext;
        alphaLog(:, sampleIndex) = alphaActual;
        poleDistanceLog(sampleIndex) = min( ...
            poleDistanceLog(sampleIndex), transitionMinimum);

        qDesPrevious = qCommand;
    end

    %% --------------------------------------------------------------------
    %  METRICS
    %  ---------------------------------------------------------------------

    steadyMask = time >= steadyStateStartTime;
    steadyTime = time(steadyMask);

    componentError = wrapAngleDifferenceLocal( ...
        qRefLog - qActualLog);
    steadyComponentError = componentError(:, steadyMask);

    rmsErrorDeg(caseIndex, :) = ...
        rad2deg(sqrt(mean(steadyComponentError.^2, 2))).';
    peakErrorDeg(caseIndex, :) = ...
        rad2deg(max(abs(steadyComponentError), [], 2)).';

    physicalRmsErrorDeg(caseIndex) = rad2deg( ...
        sqrt(mean(physicalErrorLog(steadyMask).^2)));
    physicalPeakErrorDeg(caseIndex) = rad2deg( ...
        max(abs(physicalErrorLog(steadyMask))));

    actualDrivenDeg = rad2deg(qActualLog(axisIndex, steadyMask));
    [actualAmplitudeDeg, actualPhaseRad] = fitSineLocal( ...
        steadyTime(:), actualDrivenDeg(:), frequencyHz);

    amplitudeRatio(caseIndex) = actualAmplitudeDeg / amplitudeDeg;
    phaseLagRad = wrapToPiLocal(-actualPhaseRad);
    phaseLagDeg(caseIndex) = rad2deg(phaseLagRad);
    phaseLagSeconds(caseIndex) = ...
        phaseLagRad / (2*pi*frequencyHz);

    if numberOfUpdates >= 2
        maximumCommandStepDeg(caseIndex) = maxPhysicalCommandStepLocal( ...
            qCommandLog);
    else
        maximumCommandStepDeg(caseIndex) = NaN;
    end

    maximumMotorVelocityDegPerSec(caseIndex) = ...
        max(abs(rad2deg(omegaLog(:))));
    maximumMotorAccelerationDegPerSec2(caseIndex) = ...
        max(abs(rad2deg(alphaLog(:))));

    minimumPoleDistanceDeg(caseIndex) = ...
        rad2deg(min(poleDistanceLog));

    acceptedUpdates(caseIndex) = sum(acceptedLog);
    fallbackCount(caseIndex) = sum(fallbackLog);
    recoveryCount(caseIndex) = sum(recoveryLog);
    stagnationResetCount(caseIndex) = sum(resetLog);

    allFiniteValues = [ ...
        qRefLog(:); qActualLog(:); qCommandLog(:); ...
        thetaLog(:); omegaLog(:); alphaLog(:); ...
        physicalErrorLog(:); poleDistanceLog(:)];
    nonfiniteCount(caseIndex) = sum(~isfinite(allFiniteValues));

    finiteSolveTimes = solveTimeLog(isfinite(solveTimeLog));
    if ~isempty(finiteSolveTimes)
        averageControllerCall(caseIndex) = mean(finiteSolveTimes);
        worstControllerCall(caseIndex) = max(finiteSolveTimes);
    end

    drivenRms = rmsErrorDeg(caseIndex, axisIndex);
    drivenPeak = peakErrorDeg(caseIndex, axisIndex);

    casePass(caseIndex) = ...
        drivenRms <= rmsErrorLimitDeg && ...
        drivenPeak <= peakErrorLimitDeg && ...
        amplitudeRatio(caseIndex) >= amplitudeRatioLimits(1) && ...
        amplitudeRatio(caseIndex) <= amplitudeRatioLimits(2) && ...
        minimumPoleDistanceDeg(caseIndex) >= ...
            rad2deg(params.singularity.dangerDistance) && ...
        acceptedUpdates(caseIndex) == numberOfUpdates && ...
        fallbackCount(caseIndex) == 0 && ...
        recoveryCount(caseIndex) == 0 && ...
        nonfiniteCount(caseIndex) == 0;

    fprintf('Driven-axis RMS error:      %.4f deg\n', drivenRms);
    fprintf('Driven-axis peak error:     %.4f deg\n', drivenPeak);
    fprintf('RMS error [R P Y]:          [%.4f %.4f %.4f] deg\n', ...
        rmsErrorDeg(caseIndex, :));
    fprintf('Peak error [R P Y]:         [%.4f %.4f %.4f] deg\n', ...
        peakErrorDeg(caseIndex, :));
    fprintf('Physical SO(3) RMS error:   %.4f deg\n', ...
        physicalRmsErrorDeg(caseIndex));
    fprintf('Amplitude ratio:            %.4f\n', ...
        amplitudeRatio(caseIndex));
    fprintf('Phase lag:                  %.3f deg (%.4f s)\n', ...
        phaseLagDeg(caseIndex), phaseLagSeconds(caseIndex));
    fprintf('Maximum q_cmd step:         %.4f deg\n', ...
        maximumCommandStepDeg(caseIndex));
    fprintf('Maximum motor velocity:     %.3f deg/s\n', ...
        maximumMotorVelocityDegPerSec(caseIndex));
    fprintf('Maximum motor acceleration: %.3f deg/s^2\n', ...
        maximumMotorAccelerationDegPerSec2(caseIndex));
    fprintf('Minimum six-pole distance:  %.4f deg\n', ...
        minimumPoleDistanceDeg(caseIndex));
    fprintf('Accepted updates:           %d/%d\n', ...
        acceptedUpdates(caseIndex), numberOfUpdates);
    fprintf('Fallback/recovery:          %d/%d\n', ...
        fallbackCount(caseIndex), recoveryCount(caseIndex));
    fprintf('Stagnation resets:          %d\n', ...
        stagnationResetCount(caseIndex));
    fprintf('Average/worst MPC call:     %.4f / %.4f s\n', ...
        averageControllerCall(caseIndex), ...
        worstControllerCall(caseIndex));
    fprintf('CASE PASS:                  %d\n', ...
        casePass(caseIndex));

    %% --------------------------------------------------------------------
    %  PLOTS
    %  ---------------------------------------------------------------------

    figure('Name', sprintf('%s-axis sine tracking', upper(axisName)));

    subplot(3, 1, 1);
    plot(time, rad2deg(qRefLog(1, :)), '--', 'LineWidth', 1.2);
    hold on;
    plot(time, rad2deg(qActualLog(1, :)), 'LineWidth', 1.2);
    grid on;
    ylabel('Roll (deg)');
    legend('Reference', 'Actual', 'Location', 'best');
    title(sprintf('%s-only sine | %.1f deg, %.2f Hz', ...
        upper(axisName), amplitudeDeg, frequencyHz));

    subplot(3, 1, 2);
    plot(time, rad2deg(qRefLog(2, :)), '--', 'LineWidth', 1.2);
    hold on;
    plot(time, rad2deg(qActualLog(2, :)), 'LineWidth', 1.2);
    grid on;
    ylabel('Pitch (deg)');
    legend('Reference', 'Actual', 'Location', 'best');

    subplot(3, 1, 3);
    plot(time, rad2deg(qRefLog(3, :)), '--', 'LineWidth', 1.2);
    hold on;
    plot(time, rad2deg(qActualLog(3, :)), 'LineWidth', 1.2);
    grid on;
    ylabel('Yaw (deg)');
    xlabel('Time (s)');
    legend('Reference', 'Actual', 'Location', 'best');

    figure('Name', sprintf('%s-axis error and safety', upper(axisName)));

    subplot(2, 1, 1);
    plot(time, rad2deg(componentError(1, :)), 'LineWidth', 1.1);
    hold on;
    plot(time, rad2deg(componentError(2, :)), 'LineWidth', 1.1);
    plot(time, rad2deg(componentError(3, :)), 'LineWidth', 1.1);
    grid on;
    ylabel('Error (deg)');
    legend('Roll', 'Pitch', 'Yaw', 'Location', 'best');
    title('Orientation tracking error');

    subplot(2, 1, 2);
    plot(time, rad2deg(poleDistanceLog), 'LineWidth', 1.2);
    hold on;
    yline(rad2deg(params.singularity.dangerDistance), '--', ...
        'Hard danger distance');
    yline(rad2deg(params.singularity.warningDistance), ':', ...
        'Warning distance');
    grid on;
    ylabel('Minimum pole distance (deg)');
    xlabel('Time (s)');
    title('Six-pole clearance');

    caseLogs{caseIndex} = struct( ...
        'axisName', axisName, ...
        'time', time, ...
        'qRef', qRefLog, ...
        'qActual', qActualLog, ...
        'qCommand', qCommandLog, ...
        'theta', thetaLog, ...
        'omega', omegaLog, ...
        'alpha', alphaLog, ...
        'componentError', componentError, ...
        'physicalError', physicalErrorLog, ...
        'poleDistance', poleDistanceLog, ...
        'accepted', acceptedLog, ...
        'fallback', fallbackLog, ...
        'recovery', recoveryLog, ...
        'solveTime', solveTimeLog);
end

%% ========================================================================
%  SUMMARY
%  ========================================================================

DrivenAxis = axesToTest(:);
ReferencePreview = repmat(logical(useReferencePreview), numberOfCases, 1);
DrivenRmsError_deg = NaN(numberOfCases, 1);
DrivenPeakError_deg = NaN(numberOfCases, 1);

for caseIndex = 1:numberOfCases
    axisIndex = drivenAxisIndex(caseIndex);
    DrivenRmsError_deg(caseIndex) = rmsErrorDeg(caseIndex, axisIndex);
    DrivenPeakError_deg(caseIndex) = peakErrorDeg(caseIndex, axisIndex);
end

resultsTable = table( ...
    DrivenAxis, ...
    ReferencePreview, ...
    DrivenRmsError_deg, ...
    DrivenPeakError_deg, ...
    rmsErrorDeg(:, 1), ...
    rmsErrorDeg(:, 2), ...
    rmsErrorDeg(:, 3), ...
    amplitudeRatio, ...
    phaseLagDeg, ...
    phaseLagSeconds, ...
    maximumCommandStepDeg, ...
    maximumMotorVelocityDegPerSec, ...
    maximumMotorAccelerationDegPerSec2, ...
    minimumPoleDistanceDeg, ...
    acceptedUpdates, ...
    fallbackCount, ...
    recoveryCount, ...
    stagnationResetCount, ...
    averageControllerCall, ...
    worstControllerCall, ...
    casePass, ...
    'VariableNames', { ...
        'DrivenAxis', ...
        'ReferencePreview', ...
        'DrivenRmsError_deg', ...
        'DrivenPeakError_deg', ...
        'RollRmsError_deg', ...
        'PitchRmsError_deg', ...
        'YawRmsError_deg', ...
        'AmplitudeRatio', ...
        'PhaseLag_deg', ...
        'PhaseLag_s', ...
        'MaxCommandStep_deg', ...
        'MaxMotorVelocity_deg_s', ...
        'MaxMotorAcceleration_deg_s2', ...
        'MinimumPoleDistance_deg', ...
        'AcceptedUpdates', ...
        'Fallbacks', ...
        'Recoveries', ...
        'StagnationResets', ...
        'AverageControllerCall_s', ...
        'WorstControllerCall_s', ...
        'Pass'});

fprintf('\n============================================================\n');
fprintf('SINGLE-AXIS SINE TRACKING SUMMARY\n');
fprintf('============================================================\n');
disp(resultsTable);

fprintf('Passed cases: %d / %d\n', sum(casePass), numberOfCases);
fprintf('OVERALL TRACKING PASS: %d\n', all(casePass));

if saveResultsCsv
    writetable(resultsTable, resultsCsvFile);
    fprintf('Saved summary: %s\n', resultsCsvFile);
end

% Leave detailed logs in the workspace for manual inspection.
assignin('base', 'singleAxisSineResults', resultsTable);
assignin('base', 'singleAxisSineLogs', caseLogs);

%% ========================================================================
%  LOCAL FUNCTIONS
%  ========================================================================

function q = callAbenicsFK(theta, params)
% Prefer the project-standard filename abenicsFL.m. Accept abenicsFK.m as
% a compatibility fallback for older CEM packages.

    if exist('abenicsFL', 'file') == 2
        q = abenicsFL(theta, params);
    elseif exist('abenicsFK', 'file') == 2
        q = abenicsFK(theta, params);
    else
        error('SineTrackingTest:MissingFK', ...
            'Could not find abenicsFL.m or abenicsFK.m.');
    end
end

function [thetaNext, omegaNext, alpha] = plantStepLocal( ...
    theta, omega, thetaCommand, params)

    KpPlant = expandToVectorLocal(params.plant.KpPlant, 4);
    KdPlant = expandToVectorLocal(params.plant.KdPlant, 4);

    thetaError = thetaCommand - theta;
    alpha = KpPlant .* thetaError - KdPlant .* omega;
    omegaNext = omega + params.Ts * alpha;
    thetaNext = theta + params.Ts * omegaNext;
end

function thetaContinuous = unwrapThetaNearestLocal( ...
    thetaRaw, thetaReference, params)

    thetaRaw = thetaRaw(:);
    thetaReference = thetaReference(:);
    thetaContinuous = thetaRaw;

    enabled = true;
    if isfield(params, 'mpc') && ...
            isfield(params.mpc, 'thetaUnwrapEnabled')
        enabled = logical(params.mpc.thetaUnwrapEnabled);
    end

    if ~enabled
        return;
    end

    periodicMask = true(4, 1);
    if isfield(params.mpc, 'thetaPeriodic')
        periodicMask = logical( ...
            expandToVectorLocal(params.mpc.thetaPeriodic, 4));
    end

    for motorIndex = 1:4
        if periodicMask(motorIndex)
            thetaContinuous(motorIndex) = thetaRaw(motorIndex) + ...
                2*pi * round((thetaReference(motorIndex) - ...
                thetaRaw(motorIndex)) / (2*pi));
        end
    end
end

function minimumDistance = transitionPoleMinimumLocal( ...
    thetaStart, thetaEnd, samples, params)

    samples = max(2, round(samples));
    minimumDistance = inf;
    thetaDifference = thetaEnd - thetaStart;

    for sampleIndex = 1:samples
        fraction = sampleIndex / samples;
        thetaSample = thetaStart + fraction * thetaDifference;
        qSample = callAbenicsFK(thetaSample, params);
        rotation = qToRotmXYZLocal(qSample(:));
        minimumDistance = min( ...
            minimumDistance, ...
            minimumPoleDistanceLocal(rotation, params));
    end
end

function minimumDistance = minimumPoleDistanceLocal(rotation, params)

    bodyAxis = params.singularity.trackedBodyAxis(:);
    bodyAxis = bodyAxis / norm(bodyAxis);
    trackedAxis = rotation * bodyAxis;
    trackedAxis = trackedAxis / norm(trackedAxis);

    poleAxes = params.singularity.poleAxes;
    minimumDistance = inf;

    for poleIndex = 1:size(poleAxes, 2)
        pole = poleAxes(:, poleIndex);
        pole = pole / norm(pole);
        cosineValue = min(1, max(-1, dot(trackedAxis, pole)));
        minimumDistance = min(minimumDistance, acos(cosineValue));
    end
end

function rotation = qToRotmXYZLocal(q)

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

function distance = rotationDistanceLocal(rotationOne, rotationTwo)

    relative = rotationOne.' * rotationTwo;
    cosineAngle = min(1, max(-1, (trace(relative) - 1) / 2));
    distance = acos(cosineAngle);
end

function difference = wrapAngleDifferenceLocal(difference)

    difference = atan2(sin(difference), cos(difference));
end

function [amplitude, phase] = fitSineLocal(time, signal, frequency)

    omega = 2*pi*frequency;
    designMatrix = [ ...
        ones(size(time)), ...
        sin(omega*time), ...
        cos(omega*time)];

    coefficients = designMatrix \ signal;
    sineCoefficient = coefficients(2);
    cosineCoefficient = coefficients(3);

    amplitude = hypot(sineCoefficient, cosineCoefficient);
    phase = atan2(cosineCoefficient, sineCoefficient);
end

function maximumStepDeg = maxPhysicalCommandStepLocal(qCommandLog)

    numberOfCommands = size(qCommandLog, 2);
    maximumStep = 0;

    for commandIndex = 2:numberOfCommands
        previousRotation = qToRotmXYZLocal( ...
            qCommandLog(:, commandIndex - 1));
        currentRotation = qToRotmXYZLocal( ...
            qCommandLog(:, commandIndex));
        maximumStep = max(maximumStep, ...
            rotationDistanceLocal(previousRotation, currentRotation));
    end

    maximumStepDeg = rad2deg(maximumStep);
end

function wrapped = wrapToPiLocal(angle)

    wrapped = atan2(sin(angle), cos(angle));
end

function vector = expandToVectorLocal(value, numberOfElements)

    value = value(:);

    if numel(value) == 1
        vector = value * ones(numberOfElements, 1);
    elseif numel(value) == numberOfElements
        vector = value;
    else
        error('SineTrackingTest:ParameterSize', ...
            'Expected scalar or %dx1 value.', numberOfElements);
    end
end
