function test_abenics_kinematics()
%TEST_ABENICS_KINEMATICS  Sanity + correctness checks for the kinematics core.
%   Run:  >> addpath('matlab'); test_abenics_kinematics
%
%   No Simulink / extra toolboxes required. This is the gate before wiring
%   anything into onlyJoints.slx.

p = abenics_params();
fprintf('=== ABENICS kinematics core tests (beta = %.4f rad) ===\n', p.beta);

%% 1. FK returns valid rotations + a known-case check.
% (theta = 0 is a POLE, giving Rx(90 deg) -- NOT identity -- so we do not test
%  identity there. Instead we check a generic pose is a proper rotation and a
%  known non-singular case: theta = [0, 0.5, 0] must give a pure pitch Ry(0.5).)
[~, ~, Rg] = abenics_fk([0.3; 0.6; -0.2], p);
report('FK returns a proper rotation (orthonormal, det=1)', ...
    norm(Rg*Rg.' - eye(3)) < 1e-9 && abs(det(Rg)-1) < 1e-9);
[~, ~, Rp] = abenics_fk([0; 0.5; 0], p);
report('FK([0,0.5,0]) == Ry(0.5)   (paper eq.26 sanity)', ...
    norm(Rp - roty(0.5)) < 1e-9);

%% 2. eul<->quat round trip
e = [0.3 -0.5 0.8];
report('eul2quat/quat2eul round trip', ...
    max(abs(wrapToPiLocal(quat2eul_xyz(eul2quat_xyz(e)) - e))) < 1e-9);

%% 3. Jacobian: step-size stability
th = [0.4; 0.6; -0.3];
Jc = abenics_jacobian(th, p, 1e-6);
Jf = abenics_jacobian(th, p, 1e-4);
report('Jacobian step-size stable', max(abs(Jc(:)-Jf(:))) < 1e-3);

%% 4a. Numeric (DLS) FK/IK orientation round trip (robust auto-seed, off pole)
maxErr = 0;
grid = linspace(-0.6, 0.6, 5);
for a1 = grid, for a2 = [0.3 0.6 0.9], for b1 = grid
    q = abenics_fk([a1;a2;b1], p);
    thSol = abenics_ik(quat2rotm_wxyz(q), [], p);   % [] => robust closed-form seed
    q2 = abenics_fk(thSol, p);
    maxErr = max(maxErr, 2*acos(min(1,abs(q*q2.'))));
end, end, end
report(sprintf('numeric FK/IK round trip (max orient err %.2e rad)', maxErr), maxErr < 1e-4);

%% 4b. Analytic FK/IK orientation round trip: FK(ik_analytic(R)) == R.
% Compared in ORIENTATION space (acos in eqs 55/58 loses joint sign, so an
% angle-space comparison would spuriously fail; the orientation is the invariant).
maxErrA = 0;
for r = [-0.5 0 0.5], for pp = [0.2 0.5 0.9], for yy = [-0.4 0 0.4]
    qd = eul2quat_xyz([r pp yy]);
    th = abenics_ik_analytic([r pp yy], p);
    q2 = abenics_fk(th, p);
    maxErrA = max(maxErrA, 2*acos(min(1,abs(qd*q2.'))));
end, end, end
report(sprintf('analytic FK/IK round trip (max orient err %.2e rad)', maxErrA), maxErrA < 1e-6);

%% 5. Coupled single-axis response about a NON-singular pose (documented).
fprintf('\n--- single-axis gain about a non-singular pose [0.2 0.6 0.1] ---\n');
base = [0.2; 0.6; 0.1];
probeAxis('thetaA1', base, 1, p);
probeAxis('thetaA2', base, 2, p);
probeAxis('thetaB1', base, 3, p);

%% 6. Singularity sweep: manipulability collapses as thetaA2 -> 0 (the pole).
fprintf('\n--- manipulability sweep over thetaA2 (pole indicator) ---\n');
for a2 = [0.01 0.2 0.5 0.9 1.2 1.5]
    fprintf('   thetaA2=%.2f rad :  w = %.4e\n', a2, abenics_manipulability([0.3;a2;0.2], p));
end
fprintf('\nDone. FK uses paper eq.(26); IK is eqs 54-59. If 1-4b PASS the core is\n');
fprintf('trustworthy -- proceed to wire onlyJoints.slx.\n');
end

% ---------------------------------------------------------------------------
function probeAxis(name, base, idx, p)
h = 1e-3;
dp = zeros(3,1); dp(idx) = h;
qp = abenics_fk(base+dp, p);
qm = abenics_fk(base-dp, p);
sep  = 2*acos(min(1,abs(qp*qm.')));   % ball rotation between +h and -h
gain = sep/(2*h);                     % rad ball rotation per rad input
fprintf('   %-8s : gain ~ %.3f rad/rad\n', name, gain);
end

function R = roty(a)
R = [cos(a) 0 sin(a); 0 1 0; -sin(a) 0 cos(a)];
end

function report(name, ok)
if ok, s = 'PASS'; else, s = '**FAIL**'; end
fprintf('[%s] %s\n', s, name);
end

function a = wrapToPiLocal(a)
a = mod(a+pi, 2*pi) - pi;
end
