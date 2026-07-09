clear; clc; close all;

% ============================================================
% ABENICS Distance-Based Singularity Workspace Scan
%
% Uses:
%   q -> IK -> theta -> FK -> q_pred -> singularityMeasure
%
% Singularity measure:
%   s = angular distance to nearest pole axis
% ============================================================

% -----------------------------
% Load params
% -----------------------------
params_abenics_coordinate;

if ~isfield(params, 'beta')
    params.beta = pi/2;
end

% -----------------------------
% Distance-based singularity settings
% -----------------------------
params.singularity.method = "poleDistance";

params.singularity.trackedBodyAxis = [1; 0; 0];

params.singularity.poleAxes = [ ...
     1,  0,  0;
    -1,  0,  0;
     0,  1,  0;
     0, -1,  0;
     0,  0,  1;
     0,  0, -1]';

params.singularity.warningDistance = deg2rad(10);
params.singularity.dangerDistance  = deg2rad(1);

% -----------------------------
% Scan range
% -----------------------------
rollDegVals  = -30:5:30;
pitchDegVals = -30:5:30;
yawDegVals   = -30:5:30;

nTotal = numel(rollDegVals) * numel(pitchDegVals) * numel(yawDegVals);

% -----------------------------
% Preallocate
% -----------------------------
rollList = zeros(nTotal, 1);
pitchList = zeros(nTotal, 1);
yawList = zeros(nTotal, 1);

sList = zeros(nTotal, 1);
sDegList = zeros(nTotal, 1);
nearestPoleList = zeros(nTotal, 1);
maxQErrList = zeros(nTotal, 1);

statusList = strings(nTotal, 1);
errorList = strings(nTotal, 1);

idx = 0;

% -----------------------------
% Main scan
% -----------------------------
for rDeg = rollDegVals
    for pDeg = pitchDegVals
        for yDeg = yawDegVals

            idx = idx + 1;

            q_des = deg2rad([rDeg; pDeg; yDeg]);

            rollList(idx) = rDeg;
            pitchList(idx) = pDeg;
            yawList(idx) = yDeg;

            try
                % q -> IK -> theta
                theta = abenicsIK(q_des, params);

                % theta -> FK -> q_pred
                q_pred = abenicsFK(theta, params);

                % distance-based singularity score
                [s, info] = singularityMeasure(theta, q_pred, params);

                % q recovery error
                q_err = wrapAngleDifference_local(q_pred, q_des);
                maxQErrList(idx) = max(abs(rad2deg(q_err)));

                sList(idx) = s;
                sDegList(idx) = rad2deg(s);
                nearestPoleList(idx) = info.nearestPoleIndex;

                if info.isDanger
                    statusList(idx) = "DANGER";
                elseif info.isWarning
                    statusList(idx) = "WARNING";
                else
                    statusList(idx) = "SAFE";
                end

                errorList(idx) = "";

            catch ME
                sList(idx) = NaN;
                sDegList(idx) = NaN;
                nearestPoleList(idx) = NaN;
                maxQErrList(idx) = NaN;
                statusList(idx) = "ERROR";
                errorList(idx) = string(ME.message);
            end
        end
    end
end

% -----------------------------
% Build results table
% -----------------------------
results = table( ...
    rollList, pitchList, yawList, ...
    sList, sDegList, nearestPoleList, maxQErrList, ...
    statusList, errorList, ...
    'VariableNames', { ...
        'roll_deg', 'pitch_deg', 'yaw_deg', ...
        's_rad', 's_deg', 'nearestPoleIndex', 'maxQError_deg', ...
        'status', 'errorMessage'});

% -----------------------------
% Summary
% -----------------------------
nSafe = sum(results.status == "SAFE");
nWarning = sum(results.status == "WARNING");
nDanger = sum(results.status == "DANGER");
nError = sum(results.status == "ERROR");

fprintf('\n==============================\n');
fprintf('ABENICS Distance-Based Singularity Workspace Scan Summary\n');
fprintf('Total points:   %d\n', nTotal);
fprintf('SAFE:           %d\n', nSafe);
fprintf('WARNING:        %d\n', nWarning);
fprintf('DANGER:         %d\n', nDanger);
fprintf('ERROR:          %d\n', nError);

fprintf('\nThresholds:\n');
fprintf('warningDistance = %.3f deg\n', rad2deg(params.singularity.warningDistance));
fprintf('dangerDistance  = %.3f deg\n', rad2deg(params.singularity.dangerDistance));

% -----------------------------
% Lowest distances
% -----------------------------
validResults = results(~isnan(results.s_rad), :);
validResults = sortrows(validResults, 's_rad', 'ascend');

fprintf('\n==============================\n');
fprintf('Closest points to singular poles:\n');
disp(validResults(1:min(20, height(validResults)), :));

% -----------------------------
% Save
% -----------------------------
save('singularityDistanceWorkspaceResults.mat', 'results');
writetable(results, 'singularityDistanceWorkspaceResults.csv');

fprintf('\nSaved:\n');
fprintf('singularityDistanceWorkspaceResults.mat\n');
fprintf('singularityDistanceWorkspaceResults.csv\n');

% -----------------------------
% 3D scatter plot
% -----------------------------
figure;
scatter3(results.roll_deg, results.pitch_deg, results.yaw_deg, 35, results.s_deg, 'filled');
xlabel('roll deg');
ylabel('pitch deg');
zlabel('yaw deg');
title('ABENICS Distance to Nearest Singular Pole Axis');
colorbar;
grid on;

% -----------------------------
% Distance slice at yaw = 0
% -----------------------------
yawSliceDeg = 0;
sliceRows = results(results.yaw_deg == yawSliceDeg, :);

Sgrid = NaN(numel(pitchDegVals), numel(rollDegVals));

for i = 1:height(sliceRows)
    rIdx = find(rollDegVals == sliceRows.roll_deg(i));
    pIdx = find(pitchDegVals == sliceRows.pitch_deg(i));
    Sgrid(pIdx, rIdx) = sliceRows.s_deg(i);
end

figure;
imagesc(rollDegVals, pitchDegVals, Sgrid);
set(gca, 'YDir', 'normal');
xlabel('roll deg');
ylabel('pitch deg');
title('Distance to Nearest Singular Pole Axis at yaw = 0 deg');
colorbar;
grid on;

% -----------------------------
% Status slice at yaw = 0
% SAFE = 1, WARNING = 0.5, DANGER = 0, ERROR = NaN
% -----------------------------
StatusGrid = NaN(numel(pitchDegVals), numel(rollDegVals));

for i = 1:height(sliceRows)
    rIdx = find(rollDegVals == sliceRows.roll_deg(i));
    pIdx = find(pitchDegVals == sliceRows.pitch_deg(i));

    if sliceRows.status(i) == "SAFE"
        StatusGrid(pIdx, rIdx) = 1;
    elseif sliceRows.status(i) == "WARNING"
        StatusGrid(pIdx, rIdx) = 0.5;
    elseif sliceRows.status(i) == "DANGER"
        StatusGrid(pIdx, rIdx) = 0;
    else
        StatusGrid(pIdx, rIdx) = NaN;
    end
end

figure;
imagesc(rollDegVals, pitchDegVals, StatusGrid);
set(gca, 'YDir', 'normal');
xlabel('roll deg');
ylabel('pitch deg');
title('Distance-Based Singularity Status at yaw = 0 deg');
colorbar;
grid on;

% ============================================================
% Local helper
% ============================================================
function d = wrapAngleDifference_local(a, b)
    d = atan2(sin(a - b), cos(a - b));
end