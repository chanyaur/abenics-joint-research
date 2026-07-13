% =========================================================================
% params_abenics_coordinate.m
%
% Single parameter file for the ABENICS orientation-state MPC project.
% Replaces params_abenics_coordinate.m + params_pid_plant.m.
%
% Defines two structs:
%
%   params  coordinate system, signal order, and naming conventions
%   pp      PID + plant parameters for the inner loop
%           (Calculate Error -> PID -> Plant), consumed by the referenced
%           model built by build_abenics_pid_plant.m
%
% This file does NOT contain inverse kinematics, forward kinematics,
% singularity detection, or MPC.  Those live in abenicsIK.m / abenicsFK.m.
%
% NOTE on naming vs the hand-drawn control loop:
%   The drawing labels motor-space q_des/q_actual and ball pose theta.
%   That is INVERTED from the code. Here we use the CODE convention:
%     drawing "q_des"    -> theta_des    (input to the inner-loop model)
%     drawing "q_actual" -> theta_actual (output of the inner-loop model)
%     drawing "theta"    -> ball pose, produced by FK, NOT part of that model.
%
% Placeholder numeric values are tagged  % TUNE  -- real system-id values
% come later.
% =========================================================================

clear params pp

% #########################################################################
% PART 1 -- COORDINATE AND SIGNAL CONVENTIONS  (struct: params)
% #########################################################################

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
params.stage = "Coordinate convention + PID/plant inner loop. No singularity handling or MPC yet.";


% #########################################################################
% PART 2 -- PID + PLANT INNER LOOP  (struct: pp)
% #########################################################################
%
% Convention (matches abenicsIK.m / abenicsFK.m):
%   theta = [theta_rA; theta_pA; theta_rB; theta_pB]   (4 motor angles, rad)
%   PID output  = motor TORQUE command                 (N*m)
%   Plant maps torque -> actual motor position accounting for motor lag,
%   inertia, friction/load, acceleration limit, and backlash.
%
% All plant/PID gains are 4x1 (one per motor) so a For-Each Subsystem can
% partition them across the four motors.

% Derived from PART 1 -- do not hardcode these.
pp.thetaOrder = params.thetaOrder;
pp.nMotors    = numel(params.thetaOrder);

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
        error('params_abenics:size', ...
              'pp.%s must be %dx1.', f, pp.nMotors);
    end
end

fprintf('params_abenics: loaded coordinate conventions + PID/plant params for %d motors, Ts=%.4gs.\n', ...
        pp.nMotors, pp.Ts);


% -------------------------------------------------------------------------
% Distance-based singularity detection settings
% -------------------------------------------------------------------------
% This detector assumes ABENICS singularities occur when the tracked CS gear
% axis points too close to one of the 6 world pole axes:
% ±X, ±Y, ±Z.
%
% These thresholds are starting values, not physically validated.

params.singularity.method = "poleDistance";

params.singularity.trackedBodyAxis = [1; 0; 0]; % CS gear local X-axis

params.singularity.poleAxes = [ ...
    1,  0,  0;
    -1,  0,  0;
    0,  1,  0;
    0, -1,  0;
    0,  0,  1;
    0,  0, -1]';

params.singularity.warningDistance = deg2rad(10); % rad %this are changeable variables IMPORTANT IMPORTANT IMPORATN
params.singularity.dangerDistance  = deg2rad(1);  % rad
% =========================================================================
% MPC settings
% =========================================================================
% These settings are only for the nonlinear dynamic orientation-command MPC.
%
% The MPC output is:
%
%   q_des_mpc = [roll_des_mpc;
%                pitch_des_mpc;
%                yaw_des_mpc]
%
% The MPC does NOT output motor command u.
% The MPC does NOT output theta_cmd as its main output.
% The MPC output q_des_mpc goes into abenicsIK, which then produces
% theta_cmd for the existing PID / plant.
%
% The internal MPC prediction model uses a simplified second-order
% position-tracking model only for prediction. It does NOT replace the real
% Simulink PID / plant.
% =========================================================================

% -----------------------------
% MPC sample time and horizon
% -----------------------------
params.mpc.Np = 10;               % 20-step prediction horizon
params.mpc.Nc = 5;
params.mpc.maxQStep = deg2rad([2; 0; 0]); % max q_des_mpc movement per MPC step

% -----------------------------
% fmincon solver settings
% -----------------------------
params.mpc.maxIterations = 15;
params.mpc.maxFunctionEvaluations = 2000;
params.mpc.constraintTolerance = 1e-6;
params.mpc.optimalityTolerance = 1e-4;
params.mpc.stepTolerance = 1e-6;

params.mpc.recoveryClearDistance = deg2rad(3);

% -----------------------------
% CS gear orientation limits
% q = [roll; pitch; yaw]
% -----------------------------
params.mpc.qMin = deg2rad([-45; -45; -45]);
params.mpc.qMax = deg2rad([ 45;  45;  45]);

% -----------------------------
% Output-side MP gear angle limits
% theta = [theta_rA; theta_pA; theta_rB; theta_pB]
% -----------------------------
params.mpc.thetaMin = deg2rad([-180; -180; -180; -180]);
params.mpc.thetaMax = deg2rad([ 180;  180;  180;  180]);

% -----------------------------
% Temporary simulation motor/gear velocity and acceleration limits
% These are starting simulation values, not confirmed hardware limits.
% -----------------------------
params.mpc.omegaMax = deg2rad([180; 180; 180; 180]);   % rad/s
params.mpc.alphaMax = deg2rad([720; 720; 720; 720]);   % rad/s^2

% -----------------------------
% Singularity thresholds
% Distance-based singularity detector only.
% Do NOT use Jacobian singularity logic for this MPC.
% -----------------------------
params.singularity.warningDistance = deg2rad(10); % penalize inside 10 degrees
params.singularity.dangerDistance  = deg2rad(1);  % reject inside 1 degree

% -----------------------------
% Internal MPC plant prediction model
% This approximates PID + motor + mechanics inside the MPC prediction only.
% It does NOT replace the real Simulink PID / plant.
% -----------------------------
params.plant.KpPlant = 80;
params.plant.KdPlant = 12;

% -----------------------------
% MPC cost weights
% -----------------------------
params.mpc.wTrack       = 2000;    % follow q_ref during the horizon
params.mpc.wTerminal    = 1000;    % end the horizon close to q_ref
params.mpc.wSmooth      = 1000;     % avoid jumpy q_des_mpc commands
params.mpc.wMotor       = 100;      % avoid huge IK motor jumps
params.mpc.wSingularity = 1000;   % strongly avoid singularity warning zone
params.mpc.wOmega       = 0.5;    % avoid excessive predicted motor velocity

% recovery response added by Ryan
params.mpc.recoveryMaxQStep = deg2rad(0.25); % prevents intense overshoot in the recovery response from singularity
% Unsafe-target override
params.mpc.allowSingularTarget = false;

% Print one MATLAB warning when override becomes active
params.mpc.emitSingularTargetWarning = true;

params.mpc.disableRecoveryForSingularTarget = false;

% -----------------------------
% Candidate detour size
% Used by the sampling-based MPC to generate pitch/yaw detour paths.
% -----------------------------