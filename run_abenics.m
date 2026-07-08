%RUN_ABENICS  Driver for the command-driven ABENICS model (Phase 1).
%   Commands a DESIRED CS-gear orientation trajectory (roll/pitch/yaw), solves
%   inverse kinematics for the MP-gear angles thetaCmd = [thetaA1 thetaA2 thetaB1],
%   drives onlyJoints_driven.slx with it, and plots command vs achieved.
%
%   The trajectory is kept OFF the pole (pitch centred ~30 deg) so manipulability
%   stays healthy and the animation is smooth. Prereq: run  wire_abenics  once.
%
%   IMPORTANT: cd into the repo root first, so the CAD .step geometry (referenced
%   by relative name) resolves in Mechanics Explorer.

clc;
here = fileparts(mfilename('fullpath'));
cd(here);                                   % so File Solid geometry paths resolve
addpath(fullfile(here,'matlab'));
p = abenics_params();

%% ---- 0. TEST MODE + gear direction knobs -----------------------------------
%   testMode : 'A'    -> roll gear A only (Assembly_3, gearA_ts)
%              'B'    -> roll gear B only (Assembly_1, gearB_ts)
%              'both' -> both gears roll (out of phase)
%   signA/signB : flip to -1 if that gear rolls the WRONG way.
%   Each gear is driven by the no-slip rolling law about its TRUE meshing axis.
testMode = 'both';
signA    = +1;             % gear A = Assembly_3 = gearA_ts
signB    = -1;             % gear B = Assembly_1 = gearB_ts

%% ---- 1. Centre pose + each gear's TRUE meshing axis (paper link chains) ----
qc    = eul2quat_xyz([deg2rad(0) deg2rad(40) deg2rad(10)]);  % centre orientation
thC   = abenics_ik(qc, [], p);              % [thetaA1 thetaA2 thetaB1] at centre
Rx = @(a)[1 0 0; 0 cos(a) -sin(a); 0 sin(a) cos(a)];
Rz = @(a)[cos(a) -sin(a) 0; sin(a) cos(a) 0; 0 0 1];
nA = Rx(thC(1))*[0;1;0];          nA = nA/norm(nA);   % gear A (Ry(thetaA2)) axis
nB = Rz(p.beta)*Rx(thC(3))*[0;1;0]; nB = nB/norm(nB); % gear B (Ry(thetaB2)) axis
ratio = p.gearRatio;                        % 2:1 rolling ratio (physical)

%% ---- 2. Desired ball motion about those axes -> IK -> rolling law ----------
T = 10;  t = (0:p.Ts:T).';  N = numel(t);
% Smaller swing keeps the trajectory clear of the singularity (w >> w_min).
switch testMode
    case 'hold', aA = 0*t;                            aB = 0*t;   % everything static
    case 'A',    aA = deg2rad(18)*sin(2*pi*0.10*t);  aB = 0*t;
    case 'B',    aA = 0*t;                            aB = deg2rad(18)*sin(2*pi*0.10*t);
    otherwise,   aA = deg2rad(15)*sin(2*pi*0.10*t);   aB = deg2rad(15)*sin(2*pi*0.10*t - pi/2);
end
thetaSig = zeros(N,3); gA = zeros(N,1); gB = zeros(N,1);
eul = zeros(N,3); wtraj = zeros(N,1); guess = thC;
for k = 1:N
    qd = quatmul(axisangle(nB, aB(k)), quatmul(axisangle(nA, aA(k)), qc));
    th = abenics_ik(qd, guess, p);  guess = th;
    thetaSig(k,:) = th.';
    qk = abenics_fk(th, p);
    rv = quat2rotvec(quatmul(qk, [qc(1) -qc(2) -qc(3) -qc(4)]));   % ball rot re centre
    gA(k) = ratio*(rv.'*nA);          % no-slip rolling about each gear's true axis
    gB(k) = ratio*(rv.'*nB);
    [~, eul(k,:)] = abenics_fk(th, p);
    wtraj(k) = abenics_manipulability(th, p);
end

% Isolate for tuning: hold the non-tested gear still in single-gear modes.
switch testMode
    case 'A', gB(:) = 0;
    case 'B', gA(:) = 0;
end

thetaCmd = timeseries(thetaSig, t);      %#ok<NASGU>
gearA_ts = timeseries(signA*gA, t);      %#ok<NASGU>  gear A = Assembly_3
gearB_ts = timeseries(signB*gB, t);      %#ok<NASGU>  gear B = Assembly_1
assignin('base','thetaCmd', thetaCmd);
assignin('base','gearA_ts', gearA_ts);
assignin('base','gearB_ts', gearB_ts);

%% ---- 3. Plots --------------------------------------------------------------
figure('Name',['ABENICS test mode ' testMode]);
subplot(3,1,1); plot(t, rad2deg([gA gB])); grid on;
    ylabel('gear angle [deg]'); legend('gear A','gear B'); title(['testMode = ' testMode]);
subplot(3,1,2); plot(t, rad2deg(eul)); grid on;
    ylabel('CS orient [deg]'); legend('roll','pitch','yaw');
subplot(3,1,3); plot(t, wtraj); grid on; yline(p.w_min,'r--','w_{min}');
    ylabel('manipulability w'); xlabel('t [s]');

%% ---- 4. Run the command-driven Simulink model -----------------------------
model = 'onlyJoints_driven';
if exist([model '.slx'],'file')
    open_system(model);
    sim(model, 'StopTime', num2str(T));
    fprintf('Simulation done. In Mechanics Explorer click Fit; press play.\n');
    fprintf('If you still see only frames, run: cd(''%s'') then re-run.\n', here);
else
    fprintf('Run wire_abenics first to create %s.slx.\n', model);
end

% --- local helper: quaternion for rotation about unit axis by angle ---------
function q = axisangle(ax, a)
ax = ax(:).'/norm(ax);
q = [cos(a/2), sin(a/2)*ax];
end
