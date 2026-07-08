function J = abenics_jacobian(theta, p, h)
%ABENICS_JACOBIAN  Maps MP-gear rates to CS-gear spatial angular velocity.
%
%   J = ABENICS_JACOBIAN(theta, p) returns the 3x3 Jacobian such that
%       omega = J * thetadot
%   where omega is the world-frame angular velocity of the CS-gear and
%   thetadot = d/dt [thetaA1; thetaA2; thetaB1].
%
%   Computed by central finite differencing of abenics_fk, so it is always
%   consistent with whatever FK is currently loaded. Near CS-gear poles J
%   loses rank (that is the singularity we control around).

if nargin < 2, p = abenics_params(); end
if nargin < 3, h = 1e-6; end

theta = theta(:);
n = numel(theta);
J = zeros(3, n);
for i = 1:n
    dp = zeros(n,1); dp(i) = h;
    qp = abenics_fk(theta + dp, p);
    qm = abenics_fk(theta - dp, p);
    % world-frame incremental rotation qp*conj(qm), scaled to angular rate
    dq = quatmul(qp, quatconj(qm));
    J(:,i) = quat2rotvec(dq) / (2*h);
end
end

function qc = quatconj(q)
qc = [q(1), -q(2), -q(3), -q(4)];
end

function v = quat2rotvec(q)
%QUAT2ROTVEC  Quaternion [w x y z] -> rotation vector (axis*angle).
q = q / norm(q);
if q(1) < 0, q = -q; end          % shortest rotation
vpart = q(2:4);
s = norm(vpart);
if s < 1e-12
    v = 2*vpart;                  % small-angle limit
else
    ang = 2*atan2(s, q(1));
    v = (ang/s) * vpart;
end
v = v(:);
end
