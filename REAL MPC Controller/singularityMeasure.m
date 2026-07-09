function [s, info] = singularityMeasure(theta, q, params)
%SINGULARITYMEASURE Distance-based ABENICS singularity detector.
%
% Inputs:
%   theta  = [theta_rA; theta_pA; theta_rB; theta_pB] in radians
%            kept for interface compatibility, but not used in this
%            distance-only version
%
%   q      = [roll; pitch; yaw] in radians
%
%   params = ABENICS parameter structure
%
% Outputs:
%   s      = distance to nearest singular pole axis, in radians
%            large = safer
%            small = closer to singularity
%
%   info   = diagnostic structure

    % -----------------------------
    % Input checks
    % -----------------------------
    if ~isequal(size(theta), [4, 1])
        error('singularityMeasure:thetaSize', ...
              'theta must be 4x1: [theta_rA; theta_pA; theta_rB; theta_pB].');
    end

    if ~isequal(size(q), [3, 1])
        error('singularityMeasure:qSize', ...
              'q must be 3x1: [roll; pitch; yaw].');
    end

    if ~isfield(params, 'singularity')
        error('singularityMeasure:MissingSettings', ...
              'params.singularity settings are required.');
    end

    if ~isfield(params.singularity, 'trackedBodyAxis')
        error('singularityMeasure:MissingTrackedAxis', ...
              'params.singularity.trackedBodyAxis is required.');
    end

    if ~isfield(params.singularity, 'poleAxes')
        error('singularityMeasure:MissingPoleAxes', ...
              'params.singularity.poleAxes is required.');
    end

    if ~isfield(params.singularity, 'warningDistance')
        error('singularityMeasure:MissingWarningDistance', ...
              'params.singularity.warningDistance is required.');
    end

    if ~isfield(params.singularity, 'dangerDistance')
        error('singularityMeasure:MissingDangerDistance', ...
              'params.singularity.dangerDistance is required.');
    end

    bodyAxis = params.singularity.trackedBodyAxis;
    poleAxes = params.singularity.poleAxes;

    warningDistance = params.singularity.warningDistance;
    dangerDistance  = params.singularity.dangerDistance;

    % -----------------------------
    % Normalize tracked body axis
    % -----------------------------
    bodyAxis = bodyAxis / norm(bodyAxis);

    % -----------------------------
    % Convert q into rotation matrix
    %
    % Project convention:
    % R = Rx(roll) * Ry(pitch) * Rz(yaw)
    % -----------------------------
    roll  = q(1);
    pitch = q(2);
    yaw   = q(3);

    R = localRx(roll) * localRy(pitch) * localRz(yaw);

    % -----------------------------
    % Tracked CS gear axis in world frame
    % -----------------------------
    trackedAxisWorld = R * bodyAxis;
    trackedAxisWorld = trackedAxisWorld / norm(trackedAxisWorld);

    % -----------------------------
    % Distance to nearest singular pole axis
    %
    % Each pole axis is one of:
    % +X, -X, +Y, -Y, +Z, -Z
    %
    % Angular distance:
    % d = acos(dot(axis, pole))
    % -----------------------------
    poleDots = poleAxes' * trackedAxisWorld;

    % Clamp to avoid acos numerical issues
    poleDotsClamped = min(1, max(-1, poleDots));

    poleDistances = acos(poleDotsClamped);

    [nearestDistance, nearestIdx] = min(poleDistances);

    % Singularity score
    s = nearestDistance;

    % -----------------------------
    % Status flags
    % -----------------------------
    isDanger = s < dangerDistance;
    isWarning = s < warningDistance && ~isDanger;
    isSafe = ~isWarning && ~isDanger;

    % -----------------------------
    % Diagnostics
    % -----------------------------
    info.method = "poleDistance";
    info.s = s;
    info.s_deg = rad2deg(s);

    info.trackedBodyAxis = bodyAxis;
    info.trackedAxisWorld = trackedAxisWorld;

    info.poleAxes = poleAxes;
    info.poleDots = poleDots;
    info.poleDistances = poleDistances;
    info.poleDistances_deg = rad2deg(poleDistances);

    info.nearestPoleIndex = nearestIdx;
    info.nearestPoleAxis = poleAxes(:, nearestIdx);
    info.nearestPoleDistance = nearestDistance;
    info.nearestPoleDistance_deg = rad2deg(nearestDistance);

    info.warningDistance = warningDistance;
    info.dangerDistance = dangerDistance;
    info.warningDistance_deg = rad2deg(warningDistance);
    info.dangerDistance_deg = rad2deg(dangerDistance);

    info.isSafe = isSafe;
    info.isWarning = isWarning;
    info.isDanger = isDanger;

    info.note = "Distance-only singularity detector. No Jacobian is used.";
end

% ------------------------------------------------------------
% Rotation matrix about X axis
% ------------------------------------------------------------
function R = localRx(theta)
    c = cos(theta);
    s = sin(theta);

    R = [1, 0, 0;
         0, c, -s;
         0, s,  c];
end

% ------------------------------------------------------------
% Rotation matrix about Y axis
% ------------------------------------------------------------
function R = localRy(theta)
    c = cos(theta);
    s = sin(theta);

    R = [ c, 0, s;
          0, 1, 0;
         -s, 0, c];
end

% ------------------------------------------------------------
% Rotation matrix about Z axis
% ------------------------------------------------------------
function R = localRz(theta)
    c = cos(theta);
    s = sin(theta);

    R = [c, -s, 0;
         s,  c, 0;
         0,  0, 1];
end