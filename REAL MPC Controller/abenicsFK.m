function q_pred = abenicsFK(theta_actual, beta)
%ABENICSFK Convert output-side MP gear angles into predicted CS gear orientation.
%
% Input:
%   theta_actual = [theta_rA; theta_pA; theta_rB; theta_pB] in radians
%   beta         = angle formed by the two driving modules in radians
%
% Output:
%   q_pred = [roll; pitch; yaw] in radians
%
% Project convention:
%   theta_actual is output-side MP-gear angle feedback,
%   not raw motor encoder shaft angle.
%
% Important naming convention:
%   q_pred   = model-predicted CS gear orientation from FK
%   q_actual = real IMU-measured CS gear orientation on hardware

%#codegen

    if ~isequal(size(theta_actual), [4, 1])
        error('abenicsFK:thetaActualSize', ...
              'theta_actual must be a 4x1 vector.');
    end

    if ~isscalar(beta)
        error('abenicsFK:betaSize', ...
              'beta must be a scalar angle in radians.');
    end

    theta_rA = theta_actual(1); %separates into individual values
    theta_pA = theta_actual(2);
    theta_rB = theta_actual(3);
    theta_pB = theta_actual(4); %#ok<NASGU>

    % Paper Eq. 17-18, reversed for FK
    theta_A1 = theta_rA;
    theta_A2 = -0.5 * theta_pA;
    theta_B1 = theta_rB;

    CA1 = cos(theta_A1);
    SA1 = sin(theta_A1);

    CA2 = cos(theta_A2);
    SA2 = sin(theta_A2);

    CB1 = cos(theta_B1);
    SB1 = sin(theta_B1);

    Cbeta = cos(beta);
    Sbeta = sin(beta);

    tol = 1e-12;

    % Paper FK theta_A3 equation, implemented with atan2
    A3_num = SA1*SB1 + CA1*CB1*Cbeta; % solves 

    A3_den = -CA1*CA2*SB1 ...
             + SA2*CB1*Sbeta ...
             + SA1*CA2*CB1*Cbeta;

    theta_A3 = localAtan2Safe(A3_num, A3_den, tol);

    % Paper Eq. 19-21 / Eq. 26
    R_BH = localRx(theta_A1) * localRy(theta_A2) * localRx(theta_A3);

    % Convert model-predicted rotation matrix to [roll; pitch; yaw]
    q_pred = localRotmToXYZEuler(R_BH);

    if ~isequal(size(q_pred), [3, 1])
        error('abenicsFK:qPredSize', ...
              'q_pred must be a 3x1 vector.');
    end
end

function R = localRx(theta)
    c = cos(theta);
    s = sin(theta);

    R = [1, 0, 0;
         0, c, -s;
         0, s,  c];
end

function R = localRy(theta)
    c = cos(theta);
    s = sin(theta);

    R = [ c, 0, s;
          0, 1, 0;
         -s, 0, c];
end

function x_clamped = localClamp(x, lower, upper)
    x_clamped = min(upper, max(lower, x));
end

function angle = localAtan2Safe(num, den, tol)
    if abs(num) < tol && abs(den) < tol
        angle = 0;
    else
        angle = atan2(num, den);
    end
end

function q = localRotmToXYZEuler(R)

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