function Je = abenics_jacobian_euler(theta, p, h)
%ABENICS_JACOBIAN_EULER  Paper Jacobian J_A: d(XYZ-Euler)/d(theta), eq.(44).
%
%   Je = ABENICS_JACOBIAN_EULER(theta, p) returns the 3x3 matrix
%       [dr; dp; dy] = Je * [dthetaA1; dthetaA2; dthetaB1]
%   matching Abe et al. eq.(44). Central-difference of the FK Euler output.
%
%   NOTE: this representation ALSO becomes singular at Euler gimbal-lock
%   (p = +-90 deg) in addition to the true mechanism pole. For the physically
%   meaningful, representation-free singularity metric use ABENICS_JACOBIAN
%   (angular-velocity form) + ABENICS_MANIPULABILITY.

if nargin < 2, p = abenics_params(); end
if nargin < 3, h = 1e-6; end
theta = theta(:); n = numel(theta);
Je = zeros(3, n);
for i = 1:n
    dp = zeros(n,1); dp(i) = h;
    [~, ep] = abenics_fk(theta + dp, p);
    [~, em] = abenics_fk(theta - dp, p);
    d = ep - em;
    d = mod(d + pi, 2*pi) - pi;      % wrap each Euler component
    Je(:,i) = d(:) / (2*h);
end
end
