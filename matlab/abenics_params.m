function p = abenics_params()
%ABENICS_PARAMS  Geometry / configuration for the ABENICS ball-joint model.
%   p = ABENICS_PARAMS() returns a struct with all mechanism constants used by
%   the kinematics core (abenics_fk / abenics_jacobian / abenics_ik /
%   abenics_manipulability), the Simulink model, and the (later) MPC + tendon
%   code. Keep this as the single source of truth for numbers.
%
%   Values below are taken from the CAD (module 2 mm; MP-gear 18T, CS-gear 36T,
%   driven-gear-nut 20T). Update against the hardware / Abe et al. 2021 as
%   needed -- everything downstream reads from here.

% --- Mechanism type -----------------------------------------------------
p.beta = pi/2;        % module angle between the two driving modules [rad]
                      %   pi/2 = perpendicular type, pi = opposite type

% --- Gear geometry (from CAD, module m = 2 mm) --------------------------
p.module   = 2;               % gear module [mm]
p.z_cs     = 36;              % CS-gear tooth count
p.z_mp     = 18;              % MP-gear tooth count
p.z_nut    = 20;             % driven-gear-nut tooth count
p.r_cs     = p.module*p.z_cs/2;   % CS-gear pitch radius = 36 mm (ball radius)
p.r_mp     = p.module*p.z_mp/2;   % MP-gear pitch radius = 18 mm
p.gearRatio = p.z_cs/p.z_mp;      % 2.0

% --- Independent driving angles ----------------------------------------
% theta = [thetaA1; thetaA2; thetaB1]  (thetaB2 is the DEPENDENT joint)
p.nInputs = 3;
p.inputNames = {'thetaA1','thetaA2','thetaB1'};

% --- Limits (used by IK / MPC later) -----------------------------------
p.thetadot_max = deg2rad(720);   % max MP-gear rate [rad/s] (placeholder)
p.w_min        = 0.05;           % manipulability floor for singularity zone

% --- Solver / timing ----------------------------------------------------
p.Ts = 0.01;          % control / logging sample time [s]

% --- Tendon routing (placeholders, filled in Phase 5) ------------------
% Attachment points on the ball (body frame) and on the fixed frame.
p.tendon.ball_pts  = [];   % 3xN
p.tendon.frame_pts = [];   % 3xN
p.tendon.k         = 0;    % stiffness [N/mm]
p.tendon.c         = 0;    % damping   [N/(mm/s)]
end
