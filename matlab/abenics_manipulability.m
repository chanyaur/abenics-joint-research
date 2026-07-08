function [w, sigma] = abenics_manipulability(theta, p)
%ABENICS_MANIPULABILITY  Singularity metric for the ABENICS Jacobian.
%
%   [w, sigma] = ABENICS_MANIPULABILITY(theta, p)
%     w     : Yoshikawa manipulability = sqrt(det(J*J')) = |det(J)| (3x3 J).
%             w -> 0 near the CS-gear poles (singularity); this is the value
%             the baseline reactive controller reacts to and the MPC constrains
%             (keep w >= p.w_min to bound MP-gear velocity spikes).
%     sigma : singular values of J (smallest one is the tightest indicator).
%
%   See also ABENICS_JACOBIAN.
if nargin < 2, p = abenics_params(); end
J = abenics_jacobian(theta, p);
sigma = svd(J);
w = prod(sigma);          % = sqrt(det(J*J')) for square J
end
