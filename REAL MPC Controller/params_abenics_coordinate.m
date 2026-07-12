% =========================================================================
% params_abenics_coordinate.m
%
% Coordinate and signal convention file for the ABENICS orientation-state MPC
% project.
%
% This file does NOT contain inverse kinematics, forward kinematics,
% singularity detection, plant dynamics, or MPC.
%
% It only defines the shared coordinate system and signal order.
% =========================================================================

clear params

% -------------------------------------------------------------------------
% Sample time
% -------------------------------------------------------------------------
params.Ts = 0.02;              % sample time in seconds, 0.02 s = 50 Hz

% -------------------------------------------------------------------------
% Angle unit convention
% -------------------------------------------------------------------------
params.angleUnit = "rad";      % use radians internally

%Ryan Added
params.uMAX = 2;

% -------------------------------------------------------------------------
% World coordinate frame
% -------------------------------------------------------------------------
% Convention:
% +X = forward
% +Y = left
% +Z = up

params.world.x = [1; 0; 0];    % world X-axis, forward
params.world.y = [0; 1; 0];    % world Y-axis, left
params.world.z = [0; 0; 1];    % world Z-axis, up

% -------------------------------------------------------------------------
% CS gear orientation convention
% -------------------------------------------------------------------------
% q = [roll; pitch; yaw]
%
% roll  = rotation about world X
% pitch = rotation about world Y
% yaw   = rotation about world Z

params.qOrder = ["roll", "pitch", "yaw"];

% -------------------------------------------------------------------------
% Motor angle convention
% -------------------------------------------------------------------------
% IMPORTANT:
% Always keep this exact motor angle order:
%
% theta = [theta_rA;
%          theta_pA;
%          theta_rB;
%          theta_pB]

params.thetaOrder = ["theta_rA", "theta_pA", "theta_rB", "theta_pB"];

% Encoder feedback uses the same order as theta.
params.thetaActualOrder = params.thetaOrder;

% -------------------------------------------------------------------------
% Motor command convention
% -------------------------------------------------------------------------
% u = [u_rA;
%      u_pA;
%      u_rB;
%      u_pB]
%
% For the first simple model, u is a motor velocity command.

params.uOrder = ["u_rA", "u_pA", "u_rB", "u_pB"];

% -------------------------------------------------------------------------
% Module naming convention
% -------------------------------------------------------------------------
params.moduleA.name = "A";
params.moduleB.name = "B";

% -------------------------------------------------------------------------
% Module mounting yaw convention
% -------------------------------------------------------------------------
% PLACEHOLDER:
% For now, module A is assumed to be mounted at yaw = 0 rad.
% Module B is assumed to be mounted opposite module A at yaw = pi rad.
%
% This is only a coordinate placeholder.
% It is NOT a confirmed ABENICS physical equation.
% The IK/FK Agent must verify or replace this later.

params.moduleA.baseYaw = 0; %IMPORTANT IMPORTANT IMPORTANT change when we figure out the real MP gear orientation and moutning
params.moduleB.baseYaw = pi/2; %because the mp gears are at 90 deg

params.beta = pi/2;  % angle between driving modules, radians

% -------------------------------------------------------------------------
% Sensor layout convention
% -------------------------------------------------------------------------
% Actuators:
% - 4 rotary motors
%

%IMPORTANT IMPORTANT IMPORTANT, here I made it define these temporary
%params, because otherwise it would just be not expliciity defined
params.signal.u.name = "motor command input";
params.signal.u.order = ["u_rA", "u_pA", "u_rB", "u_pB"];
params.signal.u.size = [4, 1];
params.signal.u.unit = "rad/s";

params.signal.theta_actual.name = "encoder motor angle feedback";
params.signal.theta_actual.order = ["theta_rA", "theta_pA", "theta_rB", "theta_pB"];
params.signal.theta_actual.size = [4, 1];
params.signal.theta_actual.unit = "rad";

params.signal.q_actual.name = "IMU CS gear orientation feedback";
params.signal.q_actual.order = ["roll", "pitch", "yaw"];
params.signal.q_actual.size = [3, 1];
params.signal.q_actual.unit = "rad";

params.signal.q_des.name = "desired CS gear orientation";
params.signal.q_des.order = ["roll_des", "pitch_des", "yaw_des"];
params.signal.q_des.size = [3, 1];
params.signal.q_des.unit = "rad";

params.sensor.encoderOutput = "theta_actual";
params.sensor.imuOutput = "q_actual";

% -------------------------------------------------------------------------
% Project-stage note
% -------------------------------------------------------------------------
params.stage = "Coordinate convention only. No IK, FK, singularity, plant, or MPC yet.";

% =========================================================================
% PID + PLANT PARAMETERS (struct 'pp')
%
% Parameters for the ABENICS inner loop:  Calculate Error -> PID -> Plant,
% i.e. the referenced model built by build_abenics_pid_plant.m
% (generates abenics_pid_plant.slx). Kept in this file because the build
% script and the model PreLoadFcn both source params_abenics_coordinate.
%
% Convention (matches thetaOrder above / abenicsIK.m / abenicsFK.m):
%   theta = [theta_rA; theta_pA; theta_rB; theta_pB]   (4 motor angles, rad)
%   PID output  = motor TORQUE command                 (N*m)
%   Plant maps torque -> actual motor position accounting for motor lag,
%   inertia, friction/load, acceleration limit, and backlash.
%
% All plant/PID gains are 4x1 (one per motor). Placeholder numeric values
% are tagged  % TUNE  -- real system-id values come later.
% =========================================================================
clear pp
pp.thetaOrder = ["theta_rA", "theta_pA", "theta_rB", "theta_pB"];
pp.nMotors    = 4;

% -------------------------------------------------------------------------
% Sample times
% -------------------------------------------------------------------------
pp.Ts       = params.Ts;    % controller / interface rate (s), 0.02 = 50 Hz
pp.Ts_plant = 1e-3;         % fast plant sub-step for the continuous dynamics (s)

% -------------------------------------------------------------------------
% PID controller (per motor).  Output = torque command (N*m).
% -------------------------------------------------------------------------
pp.Kp = [2.0;  2.0;  2.0;  2.0];    % TUNE proportional gain
pp.Ki = [1.0;  1.0;  1.0;  1.0];    % TUNE integral gain
pp.Kd = [0.10; 0.10; 0.10; 0.10];   % TUNE derivative gain
pp.N  = [100;  100;  100;  100];    % derivative filter coefficient (rad/s)

pp.tau_max = [1.0; 1.0; 1.0; 1.0];  % TUNE torque saturation / motor effort limit (N*m)

% -------------------------------------------------------------------------
% Plant: realistic torque-driven actuator (per motor).
% Dynamics per motor:
%   tau_applied = firstOrderLag(tau_cmd, tau_e)          % motor/electrical lag
%   net         = tau_applied - b*omega - Tc*tanh(omega/omega_eps) - load
%   alpha       = saturate( net / J, +/- alpha_max )     % acceleration limit
%   omega       = integrate(alpha)   (optional +/- omega_max limit)
%   theta_motor = integrate(omega)
%   theta       = backlash(theta_motor, width = backlash) % transmission slop
% -------------------------------------------------------------------------
pp.J   = [1.0e-3; 1.0e-3; 1.0e-3; 1.0e-3];  % TUNE rotor+load inertia (kg*m^2)
pp.b   = [5.0e-3; 5.0e-3; 5.0e-3; 5.0e-3];  % TUNE viscous friction (N*m*s/rad)
pp.Tc  = [2.0e-2; 2.0e-2; 2.0e-2; 2.0e-2];  % TUNE Coulomb friction magnitude (N*m)
pp.tau_e = [5.0e-3; 5.0e-3; 5.0e-3; 5.0e-3];% TUNE motor torque lag time constant (s)

pp.alpha_max = [500; 500; 500; 500];        % TUNE acceleration limit (rad/s^2)
pp.omega_max = [50;  50;  50;  50];         % TUNE velocity limit (rad/s)

pp.backlash  = [1.0e-3; 1.0e-3; 1.0e-3; 1.0e-3]; % TUNE backlash dead-band WIDTH (rad)

pp.load      = [0; 0; 0; 0];                % constant external load torque (N*m), 0 for now

% Smoothing constant for the Coulomb friction tanh() so it stays differentiable
% for the ODE solver (small -> closer to ideal sign()).
pp.omega_eps = 1e-3;                        % rad/s

% -------------------------------------------------------------------------
% Initial conditions
% -------------------------------------------------------------------------
pp.theta0 = [0; 0; 0; 0];   % initial motor positions (rad)
pp.omega0 = [0; 0; 0; 0];   % initial motor velocities (rad/s)

% -------------------------------------------------------------------------
% Sanity checks: every per-motor field must be nMotors x 1.
% -------------------------------------------------------------------------
perMotorFields = { 'Kp','Ki','Kd','N','tau_max','J','b','Tc','tau_e', ...
                   'alpha_max','omega_max','backlash','load','theta0','omega0' };
for k = 1:numel(perMotorFields)
    f = perMotorFields{k};
    if ~isequal(size(pp.(f)), [pp.nMotors, 1])
        error('params_abenics_coordinate:size', ...
              'pp.%s must be %dx1.', f, pp.nMotors);
    end
end