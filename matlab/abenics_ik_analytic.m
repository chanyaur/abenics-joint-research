function [theta, dep, info] = abenics_ik_analytic(target, p)
%ABENICS_IK_ANALYTIC  Closed-form inverse kinematics (Abe et al. eqs 54-59).
%
%   [theta, dep, info] = ABENICS_IK_ANALYTIC(target, p)
%     target : desired CS-gear orientation as XYZ-Euler [r p y] (rad),
%              a 1x4 quaternion [w x y z], or a 3x3 rotation matrix.
%     theta  : [thetaA1; thetaA2; thetaB1] independent MP-gear angles [rad]
%     dep    : struct with dependent joints .thetaA3 .thetaB2 .thetaB3
%     info   : .w manipulability, .singular, .resid (FK orientation residual)
%
%   BRANCH SELECTION: the raw formulas (eqs 54-59) resolve thetaB1 only up to a
%   pi ambiguity (thetaB1 and thetaB1+pi flip the loop-closure angle thetaA3 by
%   pi, i.e. give different orientations). We therefore evaluate BOTH branches
%   with the forward kinematics and return the one that actually reproduces the
%   target. This makes the result exact (no iteration) across all octants.
%
%   For smooth trajectory tracking that must cross a pole, prefer abenics_ik
%   (numeric, warm-started) which stays on one continuous branch.

if nargin < 2, p = abenics_params(); end
beta = p.beta;

% --- normalise target to a quaternion + Euler [r p y] -------------------
if numel(target) == 3 && isvector(target)
    q_des = eul2quat_xyz(target); e = target(:).';
elseif numel(target) == 4
    q_des = target(:).'/norm(target); e = quat2eul_xyz(q_des);
else
    q_des = rotm2quat_wxyz(target); e = quat2eul_xyz(q_des);
end
r = e(1); pp = e(2); yy = e(3);

Cr=cos(r); Sr=sin(r); Cp=cos(pp); Sp=sin(pp); Cy=cos(yy); Sy=sin(yy);
Cb=cos(beta); Sb=sin(beta);

% --- A-chain = X-Y-X Euler decomposition of the target, eqs (54)-(56) -----
thetaA1  =  atan2( Cr*Sy + Cy*Sp*Sr,  Cr*Cy*Sp - Sr*Sy );         % (54)
thetaA2  =  acos( max(min(Cp*Cy,1),-1) );                         % (55)
thetaA3t = -atan2( Cp*Sy, Sp );                                   % (56) target A3

% --- solve thetaB1 from the FK loop-closure so it is self-consistent ------
% FK computes thetaA3 = atan2(num,den) with
%   num = SA1*SB1 + CA1*Cb*CB1
%   den = -CA1*CA2*SB1 + (SA2*Sb + SA1*CA2*Cb)*CB1
% Requiring atan2(num,den) = thetaA3t gives  P*SB1 + Q*CB1 = 0.
SA1=sin(thetaA1); CA1=cos(thetaA1);
SA2=sin(thetaA2); CA2=cos(thetaA2);
c3=cos(thetaA3t); s3=sin(thetaA3t);
P = SA1*c3 + CA1*CA2*s3;
Q = CA1*Cb*c3 - (SA2*Sb + SA1*CA2*Cb)*s3;
thetaB1 = atan2(-Q, P);

% --- resolve the remaining pi-ambiguity via the forward kinematics --------
cand = [thetaB1, wrapToPi_local(thetaB1 + pi)];
bestErr = inf; theta = [thetaA1; thetaA2; cand(1)];
for b = cand
    th = [thetaA1; thetaA2; b];
    q  = abenics_fk(th, p);
    err = 2*acos(min(1, abs(q*q_des.')));
    if err < bestErr, bestErr = err; theta = th; end
end

% --- dependent joints for the chosen branch -----------------------------
[thetaA3, thetaB2, thetaB3] = abenics_dependent(theta, p);
dep = struct('thetaA3',thetaA3,'thetaB2',thetaB2,'thetaB3',thetaB3);

info.w        = abenics_manipulability(theta, p);
info.singular = info.w < p.w_min;
info.resid    = bestErr;
end

% ------------------------------------------------------------------------
function a = wrapToPi_local(a)
a = mod(a+pi, 2*pi) - pi;
end
