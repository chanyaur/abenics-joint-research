function [q, eul, R] = abenics_fk(theta, p)
%ABENICS_FK  Forward kinematics: MP-gear driving angles -> CS-gear orientation.
%
%   [q, eul, R] = ABENICS_FK(theta, p)
%     theta : [thetaA1; thetaA2; thetaB1]  independent MP-gear angles [rad]
%     p     : params struct from abenics_params (uses p.beta)
%     q     : 1x4 quaternion [w x y z] of CS-gear (ball) orientation
%     eul   : [theta_r theta_p theta_y] intrinsic XYZ Euler angles [rad]
%     R     : 3x3 rotation matrix (BRH)
%
%   Source: Abe et al., "ABENICS", IEEE T-RO 37(5), 2021.
%
%   IMPLEMENTATION NOTE -------------------------------------------------------
%   This uses the paper's ROTATION-MATRIX form, eq.(26):
%       BRH = Rx(thetaA1) * Ry(thetaA2) * Rx(thetaA3)
%   where thetaA3 is the closed-link dependent angle (expression just above
%   eq.39). This form is exact and is self-consistent with the paper's inverse
%   kinematics (eqs 54-59). The alternative EXPANDED XYZ-Euler form (eqs 39-41)
%   was found to be inconsistent (a transcription/typo issue in that expansion:
%   e.g. theta=[0,0.5,0] must give Ry(0.5), which eq.(26) reproduces but the
%   39-41 expansion does not), so it is NOT used here.
%
%   NOTE: theta = [0;0;0] does NOT give identity -- it gives Rx(90 deg), which
%   is a pole/singularity (thetaA2 = 0). Manipulability -> 0 there.
%   Jacobian / IK / manipulability all derive from THIS function.
%   --------------------------------------------------------------------------

if nargin < 2, p = abenics_params(); end

thetaA1 = theta(1);
thetaA2 = theta(2);
thetaB1 = theta(3);
beta    = p.beta;

CA1 = cos(thetaA1); SA1 = sin(thetaA1);
CA2 = cos(thetaA2); SA2 = sin(thetaA2);
CB1 = cos(thetaB1); SB1 = sin(thetaB1);
Cb  = cos(beta);    Sb  = sin(beta);

% Dependent passive joint theta_A3 from closed-link circularity (r21 = 0).
thetaA3 = atan2( SA1*SB1 + CA1*CB1*Cb, ...
                 -CA1*CA2*SB1 + SA2*CB1*Sb + SA1*CA2*CB1*Cb );
CA3 = cos(thetaA3); SA3 = sin(thetaA3);

% CS-gear orientation, eq.(26)  ( = Rx(A1)*Ry(A2)*Rx(A3) ).
R = [ CA2,       SA2*SA3,                 CA3*SA2;
      SA1*SA2,   CA1*CA3 - CA2*SA1*SA3,  -CA1*SA3 - CA2*CA3*SA1;
     -CA1*SA2,   CA1*CA2*SA3 + CA3*SA1,   CA1*CA2*CA3 - SA1*SA3 ];

q   = rotm2quat_wxyz(R);
eul = quat2eul_xyz(q);
end
