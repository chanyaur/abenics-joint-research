% =========================================================================
% test_cem_no_singularity_animation.m
%
% CLEAN CEM REFINEMENT TEST — NO SINGULARITY IN THE WAY
%
% Purpose:
%   Visualize CEM sampling and refinement without a nearby singularity
%   dominating or truncating the candidate paths.
%
% The test uses a start and target whose tracked +X body-axis path remains
% far from all six singularity poles.
%
% Scene 1:
%   Plot every complete command-space trajectory sampled by CEM.
%
%       iteration 1 = green
%       iteration 2 = blue
%       iteration 3 = purple
%       iteration 4 = magenta
%       iteration 5 = bright violet
%
%   Rejected candidate = dark dotted
%   Safe candidate     = thin solid
%   Elite candidate    = thick bright
%   Winning candidate  = flashing red/yellow
%
% Every displayed population path is reconstructed from the exact deltaPhi
% sampled by the real CEM controller. It therefore plots completely even if
% the internal candidate evaluator later rejects that candidate.
%
% Scene 2:
%   Run the actual receding-horizon process:
%
%       solve CEM
%       apply only the first command
%       advance the internal plant by one sample
%       calculate actual orientation with FK
%       solve again
%
%   The actual applied trajectory is yellow and turns green once the target
%   is reached.
%
% OUTPUTS
% -------
%   CEM_no_singularity_animation.mp4
%   CEM_no_singularity_animation_data.mat
%   CEM_no_singularity_animation_final.png
% =========================================================================

clear;
clc;
close all;

%% ========================================================================
%  FORCE MATCHING ANIMATION CONTROLLER
%  ========================================================================

scriptFolder = fileparts(mfilename('fullpath'));
addpath(scriptFolder, '-begin');

clear abenicsOrientationMPC_animation;
rehash;

resolvedController = which('abenicsOrientationMPC_animation');

if isempty(resolvedController)
    error('CEMNoSingularity:MissingController', ...
        'abenicsOrientationMPC_animation.m was not found.');
end

fprintf('Animation controller: %s\n', resolvedController);

%% ========================================================================
%  TEST SETTINGS
%  ========================================================================

randomSeed = 1;
maxSeedAttempts = 10;

% Both orientations are far from all six tracked-axis poles.
qStartDeg  = [15; 20; 15];
qTargetDeg = [43; 25; 43];

targetToleranceDeg = 1.0;
requiredConsecutiveUpdates = 5;
maximumMpcUpdates = 250;

videoFrameRate = 45;
candidateFramesPerPath = 1;
eliteHoldFrames = 15;
winnerFlashCycles = 5;
winnerFlashFramesPerState = 3;
framesPerAppliedUpdate = 2;
finalHoldFrames = 45;

% -------------------------------------------------------------------------
% Candidate visibility controls
% -------------------------------------------------------------------------
% These affect only rendering. They do not change CEM sampling, costs,
% validity, elites, or the selected winner.
showCandidateEndpointMarkers = true;
candidateEndpointMarkerSize = 6;

% Draw candidate lines after the sphere regardless of 3-D depth. This makes
% back-side candidates visible through the translucent explanatory sphere.
drawCandidatesOnTopOfSphere = true;

% Camera bounds are calculated from the start, target, direct route,
% winner, and SAFE candidates only. Rejected outliers do not force the
% camera to zoom away from the useful refinement region.
safePathZoomPadding = 0.18;

requestedVideoFile = "CEM_no_singularity_animation.mp4";
dataFile = "CEM_no_singularity_animation_data.mat";
finalImageFile = "CEM_no_singularity_animation_final.png";

iterationColors = [ ...
    0.15, 0.90, 0.30;
    0.15, 0.55, 1.00;
    0.58, 0.28, 1.00;
    0.82, 0.24, 0.92;
    1.00, 0.55, 0.05];

%% ========================================================================
%  LOAD PARAMETERS
%  ========================================================================

if isfile("params_abenics_coordinate_current_v22.m")
    run("params_abenics_coordinate_current_v22.m");
elseif isfile("params_abenics_coordinate_merged.m")
    run("params_abenics_coordinate_merged.m");
elseif isfile("params_abenics_coordinate.m")
    run("params_abenics_coordinate.m");
else
    error('CEMNoSingularity:MissingParams', ...
        'Could not find the current ABENICS parameter file.');
end

% Animation-only settings.
params.mpc.Np = 75;
params.mpc.Nc = 40;
params.mpc.cemNumberOfKnots = 8;
params.mpc.cemPopulationSize = 128;
params.mpc.cemIterations = 5;

params.mpc.maxQStep = deg2rad([2; 2; 2]);

params.mpc.cemInitialStd = deg2rad([1.5; 1.5; 1.5]);
params.mpc.cemMinimumStd = deg2rad([0.10; 0.10; 0.10]);
params.mpc.cemMaximumStd = deg2rad([3.0; 3.0; 3.0]);

params.mpc.cemExplorationFraction = 0.15;
params.mpc.cemExplorationScale = 1.1;
params.mpc.cemExplorationStd = deg2rad([1.25; 1.25; 1.25]);

params.mpc.useReferencePreview = true;
params.mpc.qRefHorizon = repmat( ...
    deg2rad(qTargetDeg), 1, params.mpc.Np);

params.mpc.debug = false;
params.mpc.liveProgress = false;
params.mpc.enableTestDiagnostics = false;
params.mpc.captureCEMAnimation = true;

qStart = deg2rad(qStartDeg);
qTarget = deg2rad(qTargetDeg);

thetaState = reshape(double(abenicsIK(qStart, params)), 4, 1);
omegaState = zeros(4, 1);
qActual = reshape(double(abenicsFK(thetaState, params)), 3, 1);
qDesPrevious = qStart;

startClearanceDeg = minimumPoleDistanceDegLocal(qStart, params);
targetClearanceDeg = minimumPoleDistanceDegLocal(qTarget, params);

directClearanceDeg = directRouteMinimumPoleDistanceDegLocal( ...
    qStart, qTarget, params, 200);

fprintf('\n============================================================\n');
fprintf('CLEAN CEM REFINEMENT TEST — NO SINGULARITY IN THE WAY\n');
fprintf('============================================================\n');
fprintf('Start q:                 [%7.3f %7.3f %7.3f] deg\n', qStartDeg);
fprintf('Target q:                [%7.3f %7.3f %7.3f] deg\n', qTargetDeg);
fprintf('Start pole clearance:    %.3f deg\n', startClearanceDeg);
fprintf('Target pole clearance:   %.3f deg\n', targetClearanceDeg);
fprintf('Direct-route clearance:  %.3f deg\n', directClearanceDeg);
fprintf('Hard danger distance:    %.3f deg\n', ...
    rad2deg(params.singularity.dangerDistance));
fprintf('Warning distance:        %.3f deg\n', ...
    rad2deg(params.singularity.warningDistance));
fprintf('Population:              %d\n', params.mpc.cemPopulationSize);
fprintf('CEM iterations:          %d\n', params.mpc.cemIterations);
fprintf('============================================================\n');

if directClearanceDeg < ...
        rad2deg(params.singularity.warningDistance) + 5
    warning('CEMNoSingularity:RouteNotFarEnough', ...
        ['The selected route is closer to a pole than intended. ' ...
         'Direct minimum clearance is %.3f deg.'], ...
        directClearanceDeg);
end

%% ========================================================================
%  GET A REPRODUCIBLE FIRST CEM SOLVE
%  ========================================================================

global ABENICS_CEM_ANIMATION_DATA

firstCapture = struct;
firstCommand = qDesPrevious;
chosenSeed = NaN;
firstSolveTime = NaN;
firstSolveFound = false;

for seedAttempt = 1:maxSeedAttempts
    seedValue = randomSeed + seedAttempt - 1;

    clear abenicsOrientationMPC_animation;
    rehash;

    ABENICS_CEM_ANIMATION_DATA = [];
    rng(seedValue);

    attemptTimer = tic;

    commandAttempt = abenicsOrientationMPC_animation( ...
        qTarget, thetaState, qDesPrevious, params);

    attemptTime = toc(attemptTimer);
    captureAttempt = ABENICS_CEM_ANIMATION_DATA;

    safeCount = countSafeCandidatesLocal(captureAttempt);
    expectedCount = ...
        params.mpc.cemPopulationSize * params.mpc.cemIterations;

    hasWinner = ...
        isstruct(captureAttempt) && ...
        isfield(captureAttempt, 'winningDeltaPhi') && ...
        isequal(size(captureAttempt.winningDeltaPhi), ...
            [3, params.mpc.Nc]) && ...
        all(isfinite(captureAttempt.winningDeltaPhi(:)));

    accepted = ...
        isstruct(captureAttempt) && ...
        isfield(captureAttempt, 'accepted') && ...
        captureAttempt.accepted;

    fprintf( ...
        ['Seed %3d: captured=%4d/%4d | safe=%4d | ' ...
         'winner=%d | accepted=%d\n'], ...
        seedValue, countCapturedDeltaPhiLocal(captureAttempt), ...
        expectedCount, safeCount, hasWinner, accepted);

    if hasWinner && accepted && safeCount > 0
        chosenSeed = seedValue;
        firstCapture = captureAttempt;
        firstCommand = commandAttempt;
        firstSolveTime = attemptTime;
        firstSolveFound = true;
        break;
    end
end

if ~firstSolveFound
    error('CEMNoSingularity:NoFirstSolve', ...
        ['No accepted first CEM solve with captured deltaPhi was found ' ...
         'after %d seeds.'], ...
        maxSeedAttempts);
end

fprintf('Selected seed:           %d\n', chosenSeed);
fprintf('First solve time:        %.3f s\n', firstSolveTime);
fprintf('First safe candidates:   %d\n', ...
    countSafeCandidatesLocal(firstCapture));

%% ========================================================================
%  REBUILD EVERY COMPLETE COMMAND PATH
%  ========================================================================

numberOfIterations = firstCapture.iterationsCompleted;
population = firstCapture.population;

commandPaths = cell(numberOfIterations, population);
plottedPathCount = 0;

for iterationIndex = 1:numberOfIterations
    record = firstCapture.iteration(iterationIndex);

    for candidateIndex = 1:population
        deltaPhi = record.deltaPhi{candidateIndex};

        [~, axisPath] = rebuildCommandPathLocal( ...
            deltaPhi, qStart, params);

        commandPaths{iterationIndex, candidateIndex} = axisPath;

        if isFinitePathLocal(axisPath)
            plottedPathCount = plottedPathCount + 1;
        end
    end
end

expectedPathCount = numberOfIterations * population;

fprintf('Complete paths rebuilt:  %d/%d\n', ...
    plottedPathCount, expectedPathCount);

if plottedPathCount ~= expectedPathCount
    error('CEMNoSingularity:MissingPaths', ...
        ['Only %d of %d sampled CEM command paths were reconstructed. ' ...
         'The video was not generated.'], ...
        plottedPathCount, expectedPathCount);
end

[~, firstWinnerCommandPath] = rebuildCommandPathLocal( ...
    firstCapture.winningDeltaPhi, qStart, params);

%% ========================================================================
%  VIDEO AND SCENE
%  ========================================================================

[videoWriter, actualVideoFile] = createVideoWriterLocal( ...
    requestedVideoFile, videoFrameRate);

open(videoWriter);
videoCleanup = onCleanup(@() closeVideoSafelyLocal(videoWriter));

figureHandle = figure( ...
    'Name', 'CEM refinement without singularity', ...
    'Color', [0.02, 0.02, 0.03], ...
    'Position', [80, 60, 1320, 850], ...
    'Renderer', 'opengl');

axesHandle = axes( ...
    figureHandle, ...
    'Color', [0.02, 0.02, 0.03], ...
    'XColor', [0.85, 0.85, 0.85], ...
    'YColor', [0.85, 0.85, 0.85], ...
    'ZColor', [0.85, 0.85, 0.85]);

hold(axesHandle, 'on');
grid(axesHandle, 'on');
axis(axesHandle, 'equal');
daspect(axesHandle, [1, 1, 1]);

if drawCandidatesOnTopOfSphere
    % The sphere is created before the candidate lines. childorder keeps
    % later line objects visible even when they lie on the far hemisphere.
    set(axesHandle, 'SortMethod', 'childorder');
end

xlabel(axesHandle, 'World X');
ylabel(axesHandle, 'World Y');
zlabel(axesHandle, 'World Z');

title(axesHandle, ...
    'CEM Iterative Refinement Simulation', ...
    'Color', 'w', ...
    'FontSize', 18, ...
    'FontWeight', 'bold');

[sphereX, sphereY, sphereZ] = sphere(80);

surf(axesHandle, sphereX, sphereY, sphereZ, ...
    'FaceColor', [0.55, 0.60, 0.68], ...
    'FaceAlpha', 0.025, ...
    'EdgeAlpha', 0.018, ...
    'EdgeColor', [0.75, 0.78, 0.85]);

startAxis = trackedAxisFromQLocal(qStart, params);
targetAxis = trackedAxisFromQLocal(qTarget, params);

plot3(axesHandle, ...
    startAxis(1), startAxis(2), startAxis(3), ...
    'o', ...
    'MarkerSize', 10, ...
    'MarkerFaceColor', [1, 1, 1], ...
    'MarkerEdgeColor', [0.15, 0.15, 0.15], ...
    'LineWidth', 1.4);

plot3(axesHandle, ...
    targetAxis(1), targetAxis(2), targetAxis(3), ...
    'p', ...
    'MarkerSize', 15, ...
    'MarkerFaceColor', [1.00, 0.85, 0.05], ...
    'MarkerEdgeColor', [1.00, 1.00, 0.80], ...
    'LineWidth', 1.5);

directAxisPath = sphericalGeodesicLocal( ...
    startAxis, targetAxis, 180);

plot3(axesHandle, ...
    directAxisPath(1, :), ...
    directAxisPath(2, :), ...
    directAxisPath(3, :), ...
    '--', ...
    'Color', [0.88, 0.88, 0.88], ...
    'LineWidth', 1.3);

text(axesHandle, ...
    startAxis(1), startAxis(2), startAxis(3), ...
    '  start', ...
    'Color', 'w', ...
    'FontWeight', 'bold');

text(axesHandle, ...
    targetAxis(1), targetAxis(2), targetAxis(3), ...
    '  target', ...
    'Color', [1.00, 0.90, 0.20], ...
    'FontWeight', 'bold');

statusText = text(axesHandle, ...
    0.02, 0.98, ...
    'Preparing complete CEM populations...', ...
    'Units', 'normalized', ...
    'Color', 'w', ...
    'FontSize', 14, ...
    'FontWeight', 'bold', ...
    'VerticalAlignment', 'top');

detailText = text(axesHandle, ...
    0.02, 0.91, ...
    sprintf( ...
        ['direct-route pole clearance = %.2f deg\n' ...
         'complete sampled paths = %d/%d'], ...
        directClearanceDeg, ...
        plottedPathCount, expectedPathCount), ...
    'Units', 'normalized', ...
    'Color', [0.86, 0.86, 0.86], ...
    'FontSize', 11, ...
    'VerticalAlignment', 'top');

legendText = text(axesHandle, ...
    0.02, 0.80, ...
    sprintf([ ...
        'bright dotted = rejected during iteration\n' ...
        'thin colored = safe\n' ...
        'thick bright = elite\n' ...
        'flashing red = winner\n' ...
        'yellow = actual applied trajectory']), ...
    'Units', 'normalized', ...
    'Color', [0.82, 0.82, 0.82], ...
    'FontSize', 10, ...
    'VerticalAlignment', 'top');

% Fit the camera to the useful refinement region. Rejected outliers are
% still shown during each active iteration, but they do not determine the
% permanent zoom.
safeZoomPoints = [ ...
    startAxis, ...
    targetAxis, ...
    directAxisPath, ...
    firstWinnerCommandPath];

safeZoomPathCount = 0;

for iterationIndex = 1:numberOfIterations
    recordForZoom = firstCapture.iteration(iterationIndex);

    for candidateIndex = 1:population
        if recordForZoom.valid(candidateIndex)
            safeZoomPoints = [ ...
                safeZoomPoints, ...
                commandPaths{iterationIndex, candidateIndex}]; %#ok<AGROW>

            safeZoomPathCount = safeZoomPathCount + 1;
        end
    end
end

safePlotBounds = boundsFromUnitSpherePointsLocal( ...
    safeZoomPoints, safePathZoomPadding);

% During an active iteration, temporarily expand the camera to include every
% candidate from that iteration, including rejected outliers. After rejected
% candidates are removed, return to the tighter safe-path view.
iterationPlotBounds = cell(numberOfIterations, 1);

for iterationIndex = 1:numberOfIterations
    activeIterationPoints = safeZoomPoints;

    for candidateIndex = 1:population
        activeIterationPoints = [ ...
            activeIterationPoints, ...
            commandPaths{iterationIndex, candidateIndex}]; %#ok<AGROW>
    end

    iterationPlotBounds{iterationIndex} = ...
        boundsFromUnitSpherePointsLocal( ...
            activeIterationPoints, 0.10);
end

plotBounds = safePlotBounds;
currentCameraBounds = safePlotBounds;

applyBoundsLocal(axesHandle, currentCameraBounds);

% Better viewing angle for qStart=[15;20;15] deg and
% qTarget=[43;25;43] deg in the positive-X region.
view(axesHandle, 138, 20);

fprintf('Safe paths used for zoom: %d\n', safeZoomPathCount);

for frameIndex = 1:25
    writeCurrentFrameLocal(videoWriter, figureHandle);
end

%% ========================================================================
%  SCENE 1 — ALL COMPLETE CEM PATHS
%  ========================================================================

iterationLines = cell(numberOfIterations, 1);
candidateMeta = cell(numberOfIterations, 1);

for iterationIndex = 1:numberOfIterations
    record = firstCapture.iteration(iterationIndex);
    iterationColor = iterationColors( ...
        min(iterationIndex, size(iterationColors, 1)), :);

    % Expand enough to show every path in this active iteration.
    activeCameraBounds = iterationPlotBounds{iterationIndex};

    animateAxesBoundsLocal( ...
        axesHandle, currentCameraBounds, activeCameraBounds, ...
        8, videoWriter, figureHandle);

    currentCameraBounds = activeCameraBounds;

    stylePreviousIterationsLocal( ...
        iterationLines, candidateMeta, ...
        iterationColors, iterationIndex);

    currentLines = gobjects(population, 1);
    currentMeta.valid = logical(record.valid(:));
    currentMeta.elite = false(population, 1);

    eliteIndices = record.eliteIndices;
    eliteIndices = eliteIndices( ...
        eliteIndices >= 1 & eliteIndices <= population);
    currentMeta.elite(eliteIndices) = true;

    statusText.String = sprintf( ...
        'CEM iteration %d of %d', ...
        iterationIndex, numberOfIterations);

    for candidateIndex = 1:population
        path = commandPaths{iterationIndex, candidateIndex};

        if currentMeta.valid(candidateIndex)
            lineColor = iterationColor;
            lineStyle = '-';
            lineWidth = 0.90;
        else
            % Rejected candidates are only slightly dimmer than safe
            % candidates while they are being evaluated. Their dotted style
            % communicates rejection without making them disappear.
            lineColor = colorWithFloorLocal( ...
                iterationColor, 0.80, 0.14);
            lineStyle = ':';
            lineWidth = 0.78;
        end

        if showCandidateEndpointMarkers
            currentLines(candidateIndex) = plot3( ...
                axesHandle, ...
                path(1, :), path(2, :), path(3, :), ...
                'Color', lineColor, ...
                'LineStyle', lineStyle, ...
                'LineWidth', lineWidth, ...
                'Marker', '.', ...
                'MarkerIndices', size(path, 2), ...
                'MarkerSize', candidateEndpointMarkerSize);
        else
            currentLines(candidateIndex) = plot3( ...
                axesHandle, ...
                path(1, :), path(2, :), path(3, :), ...
                'Color', lineColor, ...
                'LineStyle', lineStyle, ...
                'LineWidth', lineWidth);
        end

        detailText.String = sprintf( ...
            ['plotting complete path %d/%d\n' ...
             'iteration %d/%d — safe so far %d'], ...
            candidateIndex, population, ...
            iterationIndex, numberOfIterations, ...
            nnz(record.valid(1:candidateIndex)));

        for repeatedFrame = 1:candidateFramesPerPath
            writeCurrentFrameLocal(videoWriter, figureHandle);
        end
    end

    for eliteIndex = eliteIndices(:).'
        currentLines(eliteIndex).Color = min( ...
            1, 1.35 * iterationColor);
        currentLines(eliteIndex).LineWidth = 2.8;
        currentLines(eliteIndex).LineStyle = '-';
    end

    % Show rejected candidates during the active iteration, then remove
    % only those rejected lines. All safe and elite paths remain visible.
    rejectedIndices = find(~currentMeta.valid);

    for rejectedIndex = rejectedIndices(:).'
        if isgraphics(currentLines(rejectedIndex))
            delete(currentLines(rejectedIndex));
        end
    end

    iterationLines{iterationIndex} = currentLines;
    candidateMeta{iterationIndex} = currentMeta;

    detailText.String = sprintf( ...
        ['iteration %d complete — safe %d/%d — elites %d\n' ...
         'rejected removed; safe and elite paths retained'], ...
        iterationIndex, ...
        nnz(record.valid), population, ...
        numel(eliteIndices));

    % Once rejected candidates are gone, return to the tighter view around
    % the retained safe paths, start, target, direct route, and winner.
    animateAxesBoundsLocal( ...
        axesHandle, currentCameraBounds, safePlotBounds, ...
        10, videoWriter, figureHandle);

    currentCameraBounds = safePlotBounds;

    for frameIndex = 1:eliteHoldFrames
        writeCurrentFrameLocal(videoWriter, figureHandle);
    end
end

winnerLine = plot3( ...
    axesHandle, ...
    firstWinnerCommandPath(1, :), ...
    firstWinnerCommandPath(2, :), ...
    firstWinnerCommandPath(3, :), ...
    'Color', [1, 0, 0], ...
    'LineWidth', 5.0);

winnerEndpointErrorDeg = axisDistanceDegLocal( ...
    firstWinnerCommandPath(:, end), targetAxis);

statusText.String = 'Winning sampled command path';
detailText.String = sprintf( ...
    ['winner command-space endpoint error = %.3f deg\n' ...
     'receding-horizon control applies only its first command'], ...
    winnerEndpointErrorDeg);

for flashCycle = 1:winnerFlashCycles
    winnerLine.Color = [1, 0, 0];
    winnerLine.LineWidth = 5.0;

    for frameIndex = 1:winnerFlashFramesPerState
        writeCurrentFrameLocal(videoWriter, figureHandle);
    end

    winnerLine.Color = [1, 0.92, 0.20];
    winnerLine.LineWidth = 7.0;

    for frameIndex = 1:winnerFlashFramesPerState
        writeCurrentFrameLocal(videoWriter, figureHandle);
    end
end

%% ========================================================================
%  SCENE 2 — RECEDING-HORIZON ARRIVAL
%  ========================================================================

dimAllCandidatesForJourneyLocal( ...
    iterationLines, candidateMeta, iterationColors);

if isgraphics(winnerLine)
    delete(winnerLine);
end

actualAxis = trackedAxisFromQLocal(qActual, params);
actualAxes = actualAxis;

actualLine = plot3( ...
    axesHandle, ...
    actualAxis(1), actualAxis(2), actualAxis(3), ...
    'Color', [1.00, 0.88, 0.10], ...
    'LineWidth', 4.2);

actualPoint = plot3( ...
    axesHandle, ...
    actualAxis(1), actualAxis(2), actualAxis(3), ...
    'o', ...
    'MarkerSize', 9, ...
    'MarkerFaceColor', [1.00, 0.88, 0.10], ...
    'MarkerEdgeColor', [1, 1, 1]);

planLine = gobjects(1);

actualQHistory = qActual;
commandHistory = zeros(3, 0);
errorHistoryDeg = zeros(1, 0);
minimumPoleHistoryDeg = zeros(1, 0);
acceptedHistory = false(1, 0);
fallbackHistory = false(1, 0);
solveTimeHistory = zeros(1, 0);

consecutiveInsideTolerance = 0;
targetReached = false;

pendingCapture = firstCapture;
pendingCommand = firstCommand;

for updateIndex = 1:maximumMpcUpdates
    if updateIndex == 1
        updateCapture = pendingCapture;
        qCommand = pendingCommand;
        solveTime = firstSolveTime;
    else
        ABENICS_CEM_ANIMATION_DATA = [];

        solveTimer = tic;

        qCommand = abenicsOrientationMPC_animation( ...
            qTarget, thetaState, qDesPrevious, params);

        solveTime = toc(solveTimer);
        updateCapture = ABENICS_CEM_ANIMATION_DATA;
    end

    if isgraphics(planLine)
        delete(planLine);
    end

    if isstruct(updateCapture) && ...
            isfield(updateCapture, 'winningAxisPath') && ...
            isFinitePathLocal(updateCapture.winningAxisPath)

        predictedPath = finitePathPrefixLocal( ...
            updateCapture.winningAxisPath);

        planLine = plot3( ...
            axesHandle, ...
            predictedPath(1, :), ...
            predictedPath(2, :), ...
            predictedPath(3, :), ...
            'Color', [1.00, 0.12, 0.12], ...
            'LineWidth', 2.4);
    else
        planLine = plot3(axesHandle, NaN, NaN, NaN);
    end

    [thetaState, omegaState] = plantStepAnimationLocal( ...
        thetaState, omegaState, qCommand, params);

    qActual = reshape(double(abenicsFK(thetaState, params)), 3, 1);
    qDesPrevious = qCommand;

    actualAxis = trackedAxisFromQLocal(qActual, params);
    actualAxes(:, end + 1) = actualAxis; %#ok<SAGROW>

    actualLine.XData = actualAxes(1, :);
    actualLine.YData = actualAxes(2, :);
    actualLine.ZData = actualAxes(3, :);

    actualPoint.XData = actualAxis(1);
    actualPoint.YData = actualAxis(2);
    actualPoint.ZData = actualAxis(3);

    physicalErrorDeg = rotationDistanceDegLocal( ...
        qActual, qTarget);

    poleClearanceDeg = minimumPoleDistanceDegLocal( ...
        qActual, params);

    actualQHistory(:, end + 1) = qActual; %#ok<SAGROW>
    commandHistory(:, end + 1) = qCommand; %#ok<SAGROW>
    errorHistoryDeg(end + 1) = physicalErrorDeg; %#ok<SAGROW>
    minimumPoleHistoryDeg(end + 1) = poleClearanceDeg; %#ok<SAGROW>
    solveTimeHistory(end + 1) = solveTime; %#ok<SAGROW>

    accepted = ...
        isstruct(updateCapture) && ...
        isfield(updateCapture, 'accepted') && ...
        updateCapture.accepted;

    fallback = ...
        isstruct(updateCapture) && ...
        isfield(updateCapture, 'fallbackUsed') && ...
        updateCapture.fallbackUsed;

    acceptedHistory(end + 1) = accepted; %#ok<SAGROW>
    fallbackHistory(end + 1) = fallback; %#ok<SAGROW>

    if physicalErrorDeg <= targetToleranceDeg
        consecutiveInsideTolerance = ...
            consecutiveInsideTolerance + 1;
    else
        consecutiveInsideTolerance = 0;
    end

    statusText.String = sprintf( ...
        'Receding-horizon MPC update %d', updateIndex);

    detailText.String = sprintf( ...
        ['actual SO(3) error = %.3f deg\n' ...
         'pole clearance = %.3f deg — accepted %d — fallback %d'], ...
        physicalErrorDeg, poleClearanceDeg, ...
        accepted, fallback);

    for frameIndex = 1:framesPerAppliedUpdate
        writeCurrentFrameLocal(videoWriter, figureHandle);
    end

    if consecutiveInsideTolerance >= requiredConsecutiveUpdates
        targetReached = true;
        break;
    end
end

updatesCompleted = numel(errorHistoryDeg);
finalErrorDeg = errorHistoryDeg(end);

if isgraphics(planLine)
    delete(planLine);
end

if targetReached
    statusText.String = sprintf( ...
        'Target reached after %d MPC updates', updatesCompleted);

    detailText.String = sprintf( ...
        ['final SO(3) error = %.3f deg\n' ...
         'minimum journey pole clearance = %.3f deg'], ...
        finalErrorDeg, min(minimumPoleHistoryDeg));

    for flashIndex = 1:6
        actualPoint.MarkerFaceColor = [1.00, 0.88, 0.10];
        actualPoint.MarkerSize = 10;

        for frameIndex = 1:3
            writeCurrentFrameLocal(videoWriter, figureHandle);
        end

        actualPoint.MarkerFaceColor = [0.20, 1.00, 0.35];
        actualPoint.MarkerSize = 14;

        for frameIndex = 1:3
            writeCurrentFrameLocal(videoWriter, figureHandle);
        end
    end
else
    statusText.String = 'Maximum MPC update count reached';
    detailText.String = sprintf( ...
        'final SO(3) error = %.3f deg', finalErrorDeg);
end

actualPoint.MarkerFaceColor = [0.20, 1.00, 0.35];
actualPoint.MarkerSize = 13;

for frameIndex = 1:finalHoldFrames
    writeCurrentFrameLocal(videoWriter, figureHandle);
end

%% ========================================================================
%  SAVE
%  ========================================================================

results.qStart = qStart;
results.qTarget = qTarget;
results.chosenSeed = chosenSeed;
results.directClearanceDeg = directClearanceDeg;
results.expectedPathCount = expectedPathCount;
results.plottedPathCount = plottedPathCount;
results.safeZoomPathCount = safeZoomPathCount;
results.safePathZoomPadding = safePathZoomPadding;
results.plotBounds = plotBounds;
results.safePlotBounds = safePlotBounds;
results.iterationPlotBounds = iterationPlotBounds;
results.actualQHistory = actualQHistory;
results.commandHistory = commandHistory;
results.actualAxisHistory = actualAxes;
results.errorHistoryDeg = errorHistoryDeg;
results.minimumPoleHistoryDeg = minimumPoleHistoryDeg;
results.acceptedHistory = acceptedHistory;
results.fallbackHistory = fallbackHistory;
results.solveTimeHistory = solveTimeHistory;
results.targetReached = targetReached;
results.updatesCompleted = updatesCompleted;
results.finalErrorDeg = finalErrorDeg;

save(dataFile, ...
    'results', ...
    'firstCapture', ...
    'commandPaths', ...
    'params');

exportgraphics(figureHandle, finalImageFile, ...
    'Resolution', 180);

close(videoWriter);
clear videoCleanup;

fprintf('\n============================================================\n');
fprintf('NO-SINGULARITY ANIMATION COMPLETE\n');
fprintf('============================================================\n');
fprintf('Complete paths plotted:      %d/%d\n', ...
    plottedPathCount, expectedPathCount);
fprintf('Direct-route clearance:      %.3f deg\n', ...
    directClearanceDeg);
fprintf('Target reached:              %d\n', targetReached);
fprintf('MPC updates:                 %d\n', updatesCompleted);
fprintf('Final SO(3) error:           %.4f deg\n', finalErrorDeg);
fprintf('Minimum journey clearance:   %.4f deg\n', ...
    min(minimumPoleHistoryDeg));
fprintf('Accepted updates:            %d/%d\n', ...
    nnz(acceptedHistory), updatesCompleted);
fprintf('Fallback updates:            %d/%d\n', ...
    nnz(fallbackHistory), updatesCompleted);
fprintf('Video:                       %s\n', actualVideoFile);
fprintf('Data:                        %s\n', dataFile);
fprintf('Final image:                 %s\n', finalImageFile);
fprintf('============================================================\n');

%% ========================================================================
%  LOCAL FUNCTIONS
%  ========================================================================

function count = countSafeCandidatesLocal(capture)

    count = 0;

    if ~isstruct(capture) || ...
            ~isfield(capture, 'iteration')
        return;
    end

    for iterationIndex = 1:numel(capture.iteration)
        count = count + ...
            nnz(capture.iteration(iterationIndex).valid);
    end
end

function count = countCapturedDeltaPhiLocal(capture)

    count = 0;

    if ~isstruct(capture) || ...
            ~isfield(capture, 'iteration')
        return;
    end

    for iterationIndex = 1:numel(capture.iteration)
        record = capture.iteration(iterationIndex);

        if ~isfield(record, 'deltaPhi')
            continue;
        end

        for candidateIndex = 1:numel(record.deltaPhi)
            deltaPhi = record.deltaPhi{candidateIndex};

            if isnumeric(deltaPhi) && ...
                    size(deltaPhi, 1) == 3 && ...
                    ~isempty(deltaPhi) && ...
                    all(isfinite(deltaPhi(:)))
                count = count + 1;
            end
        end
    end
end

function [qPath, axisPath] = rebuildCommandPathLocal( ...
    deltaPhi, qStart, params)

    if isempty(deltaPhi) || ...
            size(deltaPhi, 1) ~= 3 || ...
            any(~isfinite(deltaPhi(:)))

        qPath = NaN(3, 0);
        axisPath = NaN(3, 0);
        return;
    end

    numberOfSteps = size(deltaPhi, 2);
    qPath = NaN(3, numberOfSteps + 1);
    axisPath = NaN(3, numberOfSteps + 1);

    rotation = qToRotmXYZLocal(qStart);
    qPrevious = qStart(:);

    qPath(:, 1) = qPrevious;
    axisPath(:, 1) = trackedAxisFromRotmLocal( ...
        rotation, params);

    for stepIndex = 1:numberOfSteps
        rotation = rotation * ...
            rotvecToRotmLocal(deltaPhi(:, stepIndex));

        qCurrent = rotmToQXYZContinuousLocal( ...
            rotation, qPrevious);

        qPath(:, stepIndex + 1) = qCurrent;
        axisPath(:, stepIndex + 1) = ...
            trackedAxisFromRotmLocal(rotation, params);

        qPrevious = qCurrent;
    end
end

function finite = isFinitePathLocal(path)

    finite = ...
        isnumeric(path) && ...
        size(path, 1) == 3 && ...
        size(path, 2) >= 2 && ...
        all(isfinite(path(:)));
end

function path = finitePathPrefixLocal(path)

    if isempty(path)
        path = zeros(3, 0);
        return;
    end

    firstInvalid = find( ...
        ~all(isfinite(path), 1), ...
        1, 'first');

    if ~isempty(firstInvalid)
        path = path(:, 1:firstInvalid - 1);
    end
end

function stylePreviousIterationsLocal( ...
    iterationLines, candidateMeta, ...
    iterationColors, currentIteration)

    for previousIteration = 1:currentIteration - 1
        lines = iterationLines{previousIteration};
        meta = candidateMeta{previousIteration};

        if isempty(lines)
            continue;
        end

        age = currentIteration - previousIteration;

        if age == 1
            safeBrightness = 0.58;
            eliteBrightness = 0.85;
            rejectedBrightness = 0.34;
            safeColorFloor = 0.10;
            eliteColorFloor = 0.14;
            rejectedColorFloor = 0.09;
            safeWidth = 0.66;
            eliteWidth = 1.85;
            rejectedWidth = 0.50;
        else
            safeBrightness = 0.40;
            eliteBrightness = 0.66;
            rejectedBrightness = 0.22;
            safeColorFloor = 0.075;
            eliteColorFloor = 0.11;
            rejectedColorFloor = 0.065;
            safeWidth = 0.50;
            eliteWidth = 1.35;
            rejectedWidth = 0.38;
        end

        baseColor = iterationColors( ...
            min(previousIteration, size(iterationColors, 1)), :);

        for lineIndex = 1:numel(lines)
            if ~isgraphics(lines(lineIndex))
                continue;
            end

            if meta.elite(lineIndex)
                lines(lineIndex).Color = colorWithFloorLocal( ...
                    baseColor, eliteBrightness, eliteColorFloor);
                lines(lineIndex).LineWidth = eliteWidth;
                lines(lineIndex).LineStyle = '-';
            elseif meta.valid(lineIndex)
                lines(lineIndex).Color = colorWithFloorLocal( ...
                    baseColor, safeBrightness, safeColorFloor);
                lines(lineIndex).LineWidth = safeWidth;
                lines(lineIndex).LineStyle = '-';
            else
                lines(lineIndex).Color = colorWithFloorLocal( ...
                    baseColor, rejectedBrightness, rejectedColorFloor);
                lines(lineIndex).LineWidth = rejectedWidth;
                lines(lineIndex).LineStyle = ':';
            end
        end
    end
end

function dimAllCandidatesForJourneyLocal( ...
    iterationLines, candidateMeta, iterationColors)

    for iterationIndex = 1:numel(iterationLines)
        lines = iterationLines{iterationIndex};
        meta = candidateMeta{iterationIndex};

        baseColor = iterationColors( ...
            min(iterationIndex, size(iterationColors, 1)), :);

        for lineIndex = 1:numel(lines)
            if ~isgraphics(lines(lineIndex))
                continue;
            end

            if meta.elite(lineIndex)
                lines(lineIndex).Color = colorWithFloorLocal( ...
                    baseColor, 0.38, 0.08);
                lines(lineIndex).LineWidth = 0.95;
                lines(lineIndex).LineStyle = '-';
            elseif meta.valid(lineIndex)
                lines(lineIndex).Color = colorWithFloorLocal( ...
                    baseColor, 0.22, 0.055);
                lines(lineIndex).LineWidth = 0.44;
                lines(lineIndex).LineStyle = '-';
            else
                lines(lineIndex).Color = colorWithFloorLocal( ...
                    baseColor, 0.13, 0.045);
                lines(lineIndex).LineWidth = 0.32;
                lines(lineIndex).LineStyle = ':';
            end
        end
    end
end

function [thetaNext, omegaNext] = plantStepAnimationLocal( ...
    theta, omega, qCommand, params)

    thetaCommand = reshape( ...
        double(abenicsIK(qCommand, params)), ...
        4, 1);

    thetaCommand = unwrapThetaNearestLocal( ...
        thetaCommand, theta, params);

    KpPlant = expandToVectorLocal( ...
        params.plant.KpPlant, 4);

    KdPlant = expandToVectorLocal( ...
        params.plant.KdPlant, 4);

    alpha = ...
        KpPlant .* (thetaCommand - theta) - ...
        KdPlant .* omega;

    omegaNext = omega + params.Ts * alpha;
    thetaNext = theta + params.Ts * omegaNext;
end

function thetaContinuous = unwrapThetaNearestLocal( ...
    thetaRaw, thetaReference, params)

    thetaRaw = thetaRaw(:);
    thetaReference = thetaReference(:);
    thetaContinuous = thetaRaw;

    unwrapEnabled = true;

    if isfield(params, 'mpc') && ...
            isfield(params.mpc, 'thetaUnwrapEnabled')
        unwrapEnabled = logical(params.mpc.thetaUnwrapEnabled);
    end

    if ~unwrapEnabled
        return;
    end

    periodicMask = true(4, 1);

    if isfield(params, 'mpc') && ...
            isfield(params.mpc, 'thetaPeriodic')
        periodicMask = logical( ...
            expandToVectorLocal(params.mpc.thetaPeriodic, 4));
    end

    for motorIndex = 1:4
        if periodicMask(motorIndex)
            thetaContinuous(motorIndex) = ...
                thetaRaw(motorIndex) + ...
                2*pi * round( ...
                    (thetaReference(motorIndex) - ...
                     thetaRaw(motorIndex)) / (2*pi));
        end
    end
end

function vector = expandToVectorLocal(value, count)

    value = value(:);

    if numel(value) == 1
        vector = value * ones(count, 1);
    elseif numel(value) == count
        vector = value;
    else
        error('CEMNoSingularity:BadVectorParameter', ...
            'Expected a scalar or %dx1 value.', count);
    end
end

function axisVector = trackedAxisFromQLocal(q, params)

    axisVector = trackedAxisFromRotmLocal( ...
        qToRotmXYZLocal(q), params);
end

function axisVector = trackedAxisFromRotmLocal(rotation, params)

    bodyAxis = params.singularity.trackedBodyAxis(:);
    bodyAxis = bodyAxis / norm(bodyAxis);

    axisVector = rotation * bodyAxis;
    axisVector = axisVector / norm(axisVector);
end

function distanceDeg = minimumPoleDistanceDegLocal(q, params)

    axisVector = trackedAxisFromQLocal(q, params);
    poleAxes = params.singularity.poleAxes;

    distances = zeros(size(poleAxes, 2), 1);

    for poleIndex = 1:size(poleAxes, 2)
        pole = poleAxes(:, poleIndex);
        pole = pole / norm(pole);

        distances(poleIndex) = acos( ...
            min(1, max(-1, dot(pole, axisVector))));
    end

    distanceDeg = rad2deg(min(distances));
end

function minimumDistanceDeg = directRouteMinimumPoleDistanceDegLocal( ...
    qStart, qTarget, params, samples)

    startRotation = qToRotmXYZLocal(qStart);
    targetRotation = qToRotmXYZLocal(qTarget);

    relativeRotation = startRotation.' * targetRotation;
    totalRotvec = rotmToRotvecLocal(relativeRotation);

    minimumDistanceDeg = inf;

    for sampleIndex = 0:samples
        fraction = sampleIndex / samples;

        rotation = startRotation * ...
            rotvecToRotmLocal(fraction * totalRotvec);

        axisVector = trackedAxisFromRotmLocal( ...
            rotation, params);

        poleAxes = params.singularity.poleAxes;

        for poleIndex = 1:size(poleAxes, 2)
            pole = poleAxes(:, poleIndex);
            pole = pole / norm(pole);

            distanceDeg = rad2deg(acos( ...
                min(1, max(-1, dot(pole, axisVector)))));

            minimumDistanceDeg = min( ...
                minimumDistanceDeg, distanceDeg);
        end
    end
end

function distanceDeg = rotationDistanceDegLocal(qOne, qTwo)

    rotationOne = qToRotmXYZLocal(qOne);
    rotationTwo = qToRotmXYZLocal(qTwo);

    relativeRotation = rotationOne.' * rotationTwo;
    cosineAngle = min( ...
        1, max(-1, ...
        (trace(relativeRotation) - 1) / 2));

    distanceDeg = rad2deg(acos(cosineAngle));
end

function distanceDeg = axisDistanceDegLocal(axisOne, axisTwo)

    axisOne = axisOne(:) / norm(axisOne);
    axisTwo = axisTwo(:) / norm(axisTwo);

    distanceDeg = rad2deg(acos( ...
        min(1, max(-1, dot(axisOne, axisTwo)))));
end

function rotation = qToRotmXYZLocal(q)

    roll = q(1);
    pitch = q(2);
    yaw = q(3);

    cr = cos(roll); sr = sin(roll);
    cp = cos(pitch); sp = sin(pitch);
    cy = cos(yaw); sy = sin(yaw);

    Rx = [ ...
        1, 0, 0;
        0, cr, -sr;
        0, sr, cr];

    Ry = [ ...
        cp, 0, sp;
        0, 1, 0;
        -sp, 0, cp];

    Rz = [ ...
        cy, -sy, 0;
        sy, cy, 0;
        0, 0, 1];

    rotation = Rx * Ry * Rz;
end

function rotation = rotvecToRotmLocal(rotationVector)

    rotationVector = rotationVector(:);
    angle = norm(rotationVector);

    skewMatrix = [ ...
        0, -rotationVector(3), rotationVector(2);
        rotationVector(3), 0, -rotationVector(1);
        -rotationVector(2), rotationVector(1), 0];

    if angle < 1e-9
        rotation = eye(3) + ...
            skewMatrix + ...
            0.5 * skewMatrix^2;
        return;
    end

    rotation = ...
        eye(3) + ...
        sin(angle) / angle * skewMatrix + ...
        (1 - cos(angle)) / angle^2 * skewMatrix^2;
end

function rotationVector = rotmToRotvecLocal(rotation)

    cosineAngle = min( ...
        1, max(-1, ...
        (trace(rotation) - 1) / 2));

    angle = acos(cosineAngle);

    if angle < 1e-8
        rotationVector = 0.5 * [ ...
            rotation(3, 2) - rotation(2, 3);
            rotation(1, 3) - rotation(3, 1);
            rotation(2, 1) - rotation(1, 2)];
        return;
    end

    axisVector = [ ...
        rotation(3, 2) - rotation(2, 3);
        rotation(1, 3) - rotation(3, 1);
        rotation(2, 1) - rotation(1, 2)] / ...
        (2 * sin(angle));

    rotationVector = angle * axisVector;
end

function q = rotmToQXYZContinuousLocal(rotation, qReference)

    qReference = qReference(:);

    sinePitch = min(1, max(-1, rotation(1, 3)));
    principalPitch = asin(sinePitch);
    cosinePitch = cos(principalPitch);

    if abs(cosinePitch) > 1e-8
        principalRoll = atan2( ...
            -rotation(2, 3), rotation(3, 3));

        principalYaw = atan2( ...
            -rotation(1, 2), rotation(1, 1));

        candidates = [ ...
            principalRoll, principalRoll + pi, principalRoll + pi;
            principalPitch, pi - principalPitch, -pi - principalPitch;
            principalYaw, principalYaw + pi, principalYaw + pi];
    else
        candidates = [ ...
            qReference(1);
            principalPitch;
            qReference(3)];
    end

    bestScore = inf;
    q = candidates(:, 1);

    for candidateIndex = 1:size(candidates, 2)
        candidate = candidates(:, candidateIndex);

        for axisIndex = 1:3
            candidate(axisIndex) = ...
                candidate(axisIndex) + ...
                2*pi * round( ...
                    (qReference(axisIndex) - ...
                     candidate(axisIndex)) / (2*pi));
        end

        score = sum((candidate - qReference).^2);

        if score < bestScore
            bestScore = score;
            q = candidate;
        end
    end
end

function path = sphericalGeodesicLocal( ...
    startVector, endVector, numberOfPoints)

    startVector = startVector(:) / norm(startVector);
    endVector = endVector(:) / norm(endVector);

    totalAngle = acos(min( ...
        1, max(-1, dot(startVector, endVector))));

    path = zeros(3, numberOfPoints);

    if totalAngle < 1e-10
        path = repmat(startVector, 1, numberOfPoints);
        return;
    end

    denominator = sin(totalAngle);

    for pointIndex = 1:numberOfPoints
        fraction = ...
            (pointIndex - 1) / ...
            max(1, numberOfPoints - 1);

        path(:, pointIndex) = ...
            sin((1 - fraction) * totalAngle) / denominator * ...
                startVector + ...
            sin(fraction * totalAngle) / denominator * ...
                endVector;

        path(:, pointIndex) = ...
            path(:, pointIndex) / norm(path(:, pointIndex));
    end
end

function bounds = boundsFromUnitSpherePointsLocal(points, padding)

    lower = min(points, [], 2);
    upper = max(points, [], 2);

    center = 0.5 * (lower + upper);
    halfSpan = 0.5 * (upper - lower);

    halfSpan = max(halfSpan, [0.04; 0.04; 0.04]);
    halfSpan = (1 + padding) * halfSpan;

    largestHalfSpan = max(halfSpan);
    halfSpan(:) = largestHalfSpan;

    bounds = [ ...
        center(1) - halfSpan(1), center(1) + halfSpan(1);
        center(2) - halfSpan(2), center(2) + halfSpan(2);
        center(3) - halfSpan(3), center(3) + halfSpan(3)];

    bounds(:, 1) = max(bounds(:, 1), -1.05);
    bounds(:, 2) = min(bounds(:, 2),  1.05);
end

function outputColor = colorWithFloorLocal( ...
    baseColor, scale, absoluteFloor)
% Preserve the iteration hue while guaranteeing a minimum visible RGB
% level against the black animation background.

    baseColor = reshape(baseColor, 1, 3);
    outputColor = max(scale * baseColor, ...
        absoluteFloor * ones(1, 3));
    outputColor = min(outputColor, 1);
end

function [writer, actualFile] = createVideoWriterLocal( ...
    requestedFile, frameRate)

    try
        writer = VideoWriter(requestedFile, 'MPEG-4');
        writer.Quality = 95;
        actualFile = requestedFile;
    catch
        actualFile = replace( ...
            string(requestedFile), ".mp4", ".avi");

        writer = VideoWriter( ...
            actualFile, 'Motion JPEG AVI');

        writer.Quality = 95;
    end

    writer.FrameRate = frameRate;
end

function closeVideoSafelyLocal(writer)

    try
        close(writer);
    catch
    end
end

function writeCurrentFrameLocal(writer, figureHandle)

    drawnow;
    frame = getframe(figureHandle);
    writeVideo(writer, frame);
end

function animateAxesBoundsLocal( ...
    axesHandle, startBounds, endBounds, frameCount, ...
    videoWriter, figureHandle)

    for frameIndex = 1:frameCount
        fraction = frameIndex / max(1, frameCount);
        fraction = 3*fraction^2 - 2*fraction^3;

        bounds = ...
            (1 - fraction) * startBounds + ...
            fraction * endBounds;

        applyBoundsLocal(axesHandle, bounds);
        writeCurrentFrameLocal(videoWriter, figureHandle);
    end
end

function applyBoundsLocal(axesHandle, bounds)

    xlim(axesHandle, bounds(1, :));
    ylim(axesHandle, bounds(2, :));
    zlim(axesHandle, bounds(3, :));
end
