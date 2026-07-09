function [q_pred, info] = abenicsFK(theta_actual, params)
%abenicsFK Analytic 4-input FK for ABENICS orientation prediction.
%
% Input:
%   theta_actual = [theta_rA; theta_pA; theta_rB; theta_pB] in radians
%   params       = ABENICS parameter structure with params.beta
%
% Output:
%   q_pred       = [roll; pitch; yaw] in radians
%   info         = diagnostic structure
%
% Project convention:
%   theta_actual is output-side MP gear angle, not raw motor shaft angle.
%
% Important:
%   This FK uses all four theta values.
%   It builds one CS-gear orientation axis from module A,
%   one CS-gear orientation axis from module B,
%   then uses a cross product to complete the rotation matrix.

    if ~isequal(size(theta_actual), [4, 1])
        error('abenicsFK:thetaSize', ...
              'theta_actual must be [theta_rA; theta_pA; theta_rB; theta_pB].');
    end

    if nargin < 2 || ~isfield(params, 'beta')
        error('abenicsFK:missingBeta', ...
              'params.beta is required.');
    end

    beta = params.beta;

    if ~isscalar(beta)
        error('abenicsFK:betaSize', ...
              'params.beta must be scalar.');
    end

    theta_rA = theta_actual(1);
    theta_pA = theta_actual(2);
    theta_rB = theta_actual(3);
    theta_pB = theta_actual(4);

    % Convert output-side MP gear angles to paper-style chain angles
    theta_A1 = theta_rA;
    theta_A2 = -0.5 * theta_pA;

    theta_B1 = theta_rB;
    theta_B2 = -0.5 * theta_pB;

    % -----------------------------
    % Module A axis contribution
    % -----------------------------
    c1 = [cos(theta_A2);
          sin(theta_A1) * sin(theta_A2);
         -cos(theta_A1) * sin(theta_A2)];

    % -----------------------------
    % Module B axis contribution
    %
    % This is the sign/order-sensitive part.
    % It must be verified by IK -> FK round-trip tests.
    % -----------------------------
    v2_B = [cos(theta_B2);
           -sin(theta_B1) * sin(theta_B2);
            cos(theta_B1) * sin(theta_B2)];

    c2 = localRz(beta) * v2_B;

    % -----------------------------
    % Diagnostics before cleanup
    % -----------------------------
    c1Norm = norm(c1);
    c2Norm = norm(c2);
    rawColumnDot = dot(c1, c2);
    rawCross = cross(c1, c2);
    rawCrossNorm = norm(rawCross);

    % -----------------------------
    % Orthonormalize columns
    %
    % e1 = cleaned first column
    % e2 = cleaned second column, made perpendicular to e1
    % e3 = e1 x e2
    % -----------------------------
    tol = 1e-12;

    if c1Norm < tol
        error('abenicsFK:c1Bad', ...
              'Module A column is near zero.');
    end

    e1 = c1 / c1Norm;

    c2_perp = c2 - dot(e1, c2) * e1;
    c2PerpNorm = norm(c2_perp);

    if c2PerpNorm < tol
        error('abenicsFK:columnsParallel', ...
              'Module A and B columns are nearly parallel. FK orientation is poorly defined.');
    end

    e2 = c2_perp / c2PerpNorm;

    e3 = cross(e1, e2);
    e3 = e3 / norm(e3);

    R_BH = [e1, e2, e3];

    % Convert rotation matrix into [roll; pitch; yaw]
    q_pred = localRotmToXYZEuler(R_BH);

    % -----------------------------
    % Diagnostics
    % -----------------------------
    info = struct();
    info.c1 = c1;
    info.c2 = c2;
    info.e1 = e1;
    info.e2 = e2;
    info.e3 = e3;
    info.R_BH = R_BH;

    info.c1Norm = c1Norm;
    info.c2Norm = c2Norm;
    info.columnDot = rawColumnDot;
    info.crossNorm = rawCrossNorm;
    info.c2PerpNorm = c2PerpNorm;
    info.detR = det(R_BH);

    % Useful singularity-ish clue:
    % small means the two module-defined axes are almost parallel
    info.columnSeparationScore = rawCrossNorm;

    if rawCrossNorm < 1e-6
        warning('abenicsFK:nearParallelColumns', ...
                'Module A and B columns are nearly parallel. This may indicate a singular or poorly conditioned configuration.');
    end

    if ~isequal(size(q_pred), [3, 1])
        error('abenicsFK:qPredSize', ...
              'q_pred must be 3x1.');
    end
end

function R = localRz(theta)
    c = cos(theta);
    s = sin(theta);

    R = [c, -s, 0;
         s,  c, 0;
         0,  0, 1];
end

function x_clamped = localClamp(x, lower, upper)
    x_clamped = min(upper, max(lower, x));
end

function q = localRotmToXYZEuler(R)
% Uses convention:
%   R = Rx(roll) * Ry(pitch) * Rz(yaw)

    tol = 1e-12;

    pitch_arg = localClamp(R(1,3), -1, 1);
    pitch = asin(pitch_arg);

    cp = cos(pitch);

    if abs(cp) > tol
        roll = atan2(-R(2,3), R(3,3));
        yaw  = atan2(-R(1,2), R(1,1));
    else
        roll = 0;
        yaw  = atan2(R(2,1), R(2,2));
    end

    q = [roll;
         pitch;
         yaw];
end