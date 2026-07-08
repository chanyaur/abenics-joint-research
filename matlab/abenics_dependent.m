function [thetaA3, thetaB2, thetaB3] = abenics_dependent(theta, p)
%ABENICS_DEPENDENT  Passive / dependent joint angles from the independent ones.
%
%   [thetaA3, thetaB2, thetaB3] = ABENICS_DEPENDENT(theta, p)
%     theta   : [thetaA1; thetaA2; thetaB1] independent MP-gear angles [rad]
%     thetaA3 : passive joint of link chain A (from r21 = 0)
%     thetaB2 : dependent DRIVEN joint (from the r12 = 0 constraint) -- this is
%               the B-module pitch motor you must command in hardware.
%     thetaB3 : passive joint of link chain B.
%
%   Source: Abe et al. 2021, the closed-form expressions given just above
%   eq.(39) (theta_A3, theta_B2). theta_B3 is recovered from r22/r23 (34)-(35).

if nargin < 2, p = abenics_params(); end
thetaA1 = theta(1); thetaA2 = theta(2); thetaB1 = theta(3);
beta = p.beta;
CA1=cos(thetaA1); SA1=sin(thetaA1);
CA2=cos(thetaA2); SA2=sin(thetaA2);
CB1=cos(thetaB1); SB1=sin(thetaB1);
Cb=cos(beta); Sb=sin(beta);

% theta_A3 = atan( (SA1 SB1 + CA1 CB1 Cb) /
%                  (-CA1 CA2 SB1 + SA2 CB1 Sb + SA1 CA2 CB1 Cb) )
thetaA3 = atan2( SA1*SB1 + CA1*CB1*Cb, ...
                 -CA1*CA2*SB1 + SA2*CB1*Sb + SA1*CA2*CB1*Cb );

% theta_B2 = -atan( (CA2 Cb + SA1 SA2 Sb) /
%                   (CA1 SA2 CB1 - CA2 SB1 Sb + SA1 SA2 SB1 Cb) )
thetaB2 = -atan2( CA2*Cb + SA1*SA2*Sb, ...
                  CA1*SA2*CB1 - CA2*SB1*Sb + SA1*SA2*SB1*Cb );

% theta_B3 from r32 = SB3 (37) and r22 = CB3 (34)
CB2=cos(thetaB2); SB2=sin(thetaB2);
O = SB2*Cb + CB2*SB1*Sb;
N = -SA1*CB2*Cb*SB1 + SA1*SB2*Sb - CA1*CB1*CB2;
SB3 = -CA2*O - SA2*N;                                   % r32 (37)
CB3 = CA1*SA2*SB1 + CA2*CB1*Sb - CB1*Cb*SA1*SA2;        % r22 (34)
thetaB3 = atan2(SB3, CB3);
end
