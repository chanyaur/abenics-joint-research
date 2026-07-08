function [theta, info] = abenics_ik(R_des, theta0, p, opts)
%ABENICS_IK  Inverse kinematics: desired CS-gear orientation -> MP-gear angles.
%
%   [theta, info] = ABENICS_IK(R_des, theta0, p, opts)
%     R_des  : desired CS-gear orientation, 3x3 rotation matrix OR 1x4 quat.
%     theta0 : initial guess [thetaA1; thetaA2; thetaB1] (warm-start from the
%              previous step when tracking a trajectory). Pass [] to auto-seed
%              from the robust closed form (abenics_ik_analytic).
%     p      : params struct.
%     opts   : optional struct: .maxIter (default 100), .tol (1e-9 rad),
%              .lambda_sing (near-singularity damping, 0.05). Damped least
%              squares stays stable through singularities; the routine also
%              returns the best iterate, so it never degrades a good seed.
%
%   info: .iters, .resid (final orientation error norm), .converged, .w
%         (manipulability at the solution).

if nargin < 3 || isempty(p), p = abenics_params(); end
if nargin < 4, opts = struct(); end
if ~isfield(opts,'maxIter'),     opts.maxIter     = 100;  end
if ~isfield(opts,'tol'),         opts.tol         = 1e-9; end
if ~isfield(opts,'lambda_sing'), opts.lambda_sing = 0.05; end  % damping near singularity

if numel(R_des) == 4
    q_des = R_des(:).' / norm(R_des);
else
    q_des = rotm2quat_wxyz(R_des);   % shared matlab/rotm2quat_wxyz.m
end

% Seed: warm-start if given, else the robust closed-form solution.
if nargin < 2 || isempty(theta0)
    theta = abenics_ik_analytic(q_des, p);
else
    theta = theta0(:);
end
best_theta = theta; best_resid = inf;   % never return worse than the seed
svth   = 0.05;                          % singular-value damping threshold
stepMx = 0.5;                           % max ||dtheta|| per iter [rad]
resid = inf; iter = 0;
for iter = 1:opts.maxIter
    q = abenics_fk(theta, p);
    err = orientation_error(q_des, q);   % world-frame rotation vector
    resid = norm(err);
    if resid < best_resid, best_resid = resid; best_theta = theta; end
    if resid < opts.tol, break; end
    J = abenics_jacobian(theta, p);
    % SVD-based damped least squares (warning-free). Damping activates only for
    % the smallest singular value as it drops below svth (Nakamura DLS), so
    % well-conditioned directions still converge to machine precision while
    % near-singular ones cannot blow the step up.
    [Usv, Ssv, Vsv] = svd(J);
    s  = diag(Ssv);
    if s(end) < svth
        lam2 = opts.lambda_sing^2 * (1 - (s(end)/svth))^2;
    else
        lam2 = 0;
    end
    dtheta = Vsv * ((s ./ (s.^2 + lam2)) .* (Usv.' * err));
    n = norm(dtheta);
    if n > stepMx, dtheta = dtheta*(stepMx/n); end   % clamp overshoot
    theta = theta + dtheta;
end
theta = best_theta;                     % best iterate (>= seed quality)
resid = best_resid;

info.iters     = iter;
info.resid     = resid;
info.converged = resid < opts.tol;
info.w         = abenics_manipulability(theta, p);
end

% ----------------------------------------------------------------------
function e = orientation_error(q_des, q)
qe = quatmul(q_des, [q(1), -q(2), -q(3), -q(4)]);   % q_des * conj(q)
if qe(1) < 0, qe = -qe; end
v = qe(2:4); s = norm(v);
if s < 1e-12
    e = 2*v(:);
else
    e = (2*atan2(s, qe(1))/s) * v(:);
end
end
