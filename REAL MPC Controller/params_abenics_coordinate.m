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

% -------------------------------------------------------------------------
% Tendon preload (semi-active, constant setpoint version)
% -------------------------------------------------------------------------
% Represents the steady-state torque delivered by a semi-active antagonistic
% tendon pair (1 active tensioner + 1 passive spring) at a FIXED tension
% setpoint. Tensioner actuator dynamics (lag, its own PID loop) are NOT
% modeled here -- this is the settled output only, added directly into
% net_torque. Sized to exceed expected disturbance torque so the mechanism
% stays pinned against one flank of the backlash gap.
pp.tau_preload = [0; 0; 0; 0];              % TUNE constant tendon preload torque (N*m)

% Preload at which effective backlash reaches zero [N*m]
pp.tau_preload_full_engagement = [0.05; 0.05; 0.05];

% Smoothing constant for the Coulomb friction tanh() so it stays differentiable
% for the ODE solver (small -> closer to ideal sign()).
pp.omega_eps = 1e-3;                        % rad/s

% -------------------------------------------------------------------------
% Mesh-level (CS-gear / MP-gear) preload and backlash
% -------------------------------------------------------------------------
% Separate from the per-motor driving-module backlash above. This acts
% AFTER forward kinematics (abenicsFK.m), on ball orientation directly --
% i.e. on q = [roll; pitch; yaw] -- not on any individual motor angle.
% Represents tendons routed to the CS-gear/output link (per the ABENICS
% figure), pulling the CS-gear into its mesh with the MP-gears, independent
% of the 4 driving motors. Per-axis so roll/pitch/yaw can be tuned
% separately -- yaw is the one flagged in dial-gauge source notes as most
% affected by gravity-driven backlash near r = +/-90 deg.
%
% q_actual is computed from a force balance at the mesh, not an assumed
% constant offset:
%
%   q_bias = clip( (tau_preload_mesh - tau_gravity(q)) / k_mesh , +/- mesh_backlash/2 )
%   q_actual = backlash( q_pred + q_bias , width = mesh_backlash )
%
% tau_gravity(q) is a simple placeholder model of gravity-induced torque on
% the output link, worst-case near roll = +/-90 deg per dial-gauge source
% notes (yaw vibration attributed to gravity there):
%
%   tau_gravity(q) = tau_gravity_max .* sin(q)   (elementwise, q = [roll;pitch;yaw])
%
pp.mesh_backlash    = deg2rad([0.8; 0.8; 0.8]);  % TUNE mesh backlash width, [roll; pitch; yaw] (rad)
                                                   % source: dial gauge measurement, ~0.8 deg coupling backlash
pp.tau_preload_mesh = [0.025; 0.025; 0.025]; % nominal preload; plot script overrides this during sweeps (N*m)
pp.tau_gravity_max  = [0.025; 0.025; 0.025];    % TUNE worst-case gravity torque on output link, [roll; pitch; yaw] (N*m)
pp.k_mesh           = [5; 5; 5];    % TUNE effective mesh contact stiffness, [roll; pitch; yaw] (N*m/rad)

% rad/(N*m)
pp.alpha_preload = pp.mesh_backlash ...
    ./ pp.tau_preload_full_engagement;


% -------------------------------------------------------------------------
% Initial conditions
% -------------------------------------------------------------------------
pp.theta0 = [0; 0; 0; 0];   % initial motor positions (rad)
pp.omega0 = [0; 0; 0; 0];   % initial motor velocities (rad/s)

% -------------------------------------------------------------------------
% Sanity checks: every per-motor field must be nMotors x 1.
% -------------------------------------------------------------------------
perMotorFields = { 'Kp','Ki','Kd','N','tau_max','J','b','Tc','tau_e', ...
                   'alpha_max','omega_max','backlash','load','tau_preload','theta0','omega0' };
for k = 1:numel(perMotorFields)
    f = perMotorFields{k};
    if ~isequal(size(pp.(f)), [pp.nMotors, 1])
        error('params_abenics:size', ...
              'pp.%s must be %dx1.', f, pp.nMotors);
    end
end

fprintf('params_abenics: loaded coordinate conventions + PID/plant params for %d motors, Ts=%.4gs.\n', ...
        pp.nMotors, pp.Ts);

% Mesh-level fields are 3x1 (roll/pitch/yaw), not 4x1 (per-motor) -- checked separately.
perAxisFields = {'mesh_backlash', 'tau_preload_mesh', 'tau_gravity_max', 'k_mesh'};
for k = 1:numel(perAxisFields)
    f = perAxisFields{k};
    if ~isequal(size(pp.(f)), [3, 1])
        error('params_abenics:size', ...
              'pp.%s must be 3x1 ([roll; pitch; yaw]).', f);
    end
end

% -------------------------------------------------------------------------
% Distance-based singularity detection settings
% -------------------------------------------------------------------------
% The tracked CS-gear local +X axis is checked against all six signed world
% poles: +/-X, +/-Y, +/-Z.

params.singularity.method = "poleDistance";
params.singularity.trackedBodyAxis = [1; 0; 0];

params.singularity.poleAxes = [ ...
     1,  0,  0;
    -1,  0,  0;
     0,  1,  0;
     0, -1,  0;
     0,  0,  1;
     0,  0, -1]';

% Frozen safety configuration used by the validated v2.2 controller.
params.singularity.warningDistance = deg2rad(10);
params.singularity.dangerDistance  = deg2rad(2);

% =========================================================================
% SO(3) CEM ORIENTATION-STATE MPC — CURRENT STABLE V2.2 CONFIGURATION
% =========================================================================
%
% Public controller:
%   q_des_mpc = abenicsOrientationMPC( ...
%       q_ref, theta_actual, q_des_prev, params)
%
% The MPC output is q_des_mpc = [roll; pitch; yaw].
% IK converts q_des_mpc to the four output-side MP-gear commands.

% -------------------------------------------------------------------------
% Prediction and control horizons
% -------------------------------------------------------------------------
params.mpc.Np = 33;
params.mpc.Nc = 12;
params.mpc.maxQStep = deg2rad([2; 2; 2]);
params.mpc.orientationRepresentation = "SO3LocalRotationVector";

% -------------------------------------------------------------------------
% Hard constraints
% -------------------------------------------------------------------------
params.mpc.constraintTolerance = 1e-6;

% Normal operating Euler-interface limits.
params.mpc.qMin = deg2rad([-45; -45; -45]);
params.mpc.qMax = deg2rad([ 45;  45;  45]);

% Continuous output-side MP-shaft range.
params.mpc.thetaMin = deg2rad([-360; -360; -360; -360]);
params.mpc.thetaMax = deg2rad([ 360;  360;  360;  360]);

params.mpc.thetaUnwrapEnabled = true;
params.mpc.thetaPeriodic = true(4, 1);
params.mpc.enforceThetaPositionLimits = true;

params.mpc.omegaMax = deg2rad([180; 180; 180; 180]);
params.mpc.alphaMax = deg2rad([720; 720; 720; 720]);

% Check interpolated motion between predicted discrete states.
params.mpc.cemTransitionSamples = 3;
params.mpc.transitionSafetySamples = 9;

% -------------------------------------------------------------------------
% Internal MPC prediction model
% -------------------------------------------------------------------------
params.plant.KpPlant = 80;
params.plant.KdPlant = 12;

% -------------------------------------------------------------------------
% Cost weights
% -------------------------------------------------------------------------
params.mpc.wTrack       = 2000;
params.mpc.wTerminal    = 1000;
params.mpc.wSmooth      = 1000;
params.mpc.wMotor       = 100;
params.mpc.wSingularity = 4000;
params.mpc.wOmega       = 0.5;

% -------------------------------------------------------------------------
% Recovery behavior
% -------------------------------------------------------------------------
params.mpc.recoveryClearDistance = deg2rad(3);
params.mpc.recoveryMaxQStep = deg2rad(0.25);

params.mpc.allowSingularTarget = false;
params.mpc.emitSingularTargetWarning = true;
params.mpc.disableRecoveryForSingularTarget = false;

% -------------------------------------------------------------------------
% Cross-Entropy Method search
% -------------------------------------------------------------------------
params.mpc.cemNumberOfKnots = 4;
params.mpc.cemPopulationSize = 64;
params.mpc.cemIterations = 3;
params.mpc.cemProgressEveryCandidates = 16;
params.mpc.cemEliteFraction = 0.15;
params.mpc.cemSmoothing = 0.70;

params.mpc.cemInitialStd = deg2rad([1.25; 1.25; 1.25]);
params.mpc.cemMinimumStd = deg2rad([0.10; 0.10; 0.10]);
params.mpc.cemMaximumStd = deg2rad([2.00; 2.00; 2.00]);

params.mpc.cemWarmStartStdInflation = 1.25;
params.mpc.cemNearSingularityStdInflation = 1.50;
params.mpc.cemTemporalCorrelation = 0.85;

% Prevent premature covariance collapse while still far from the target.
params.mpc.cemSigmaFloor = deg2rad([0.35; 0.35; 0.35]);
params.mpc.cemSettlingErrorThreshold = deg2rad(3.0);

% Preserve broad route exploration.
params.mpc.cemExplorationFraction = 0.30;
params.mpc.cemExplorationScale = 1.50;
params.mpc.cemExplorationStd = deg2rad([1.25; 1.25; 1.25]);

% Recover from stalled warm-start distributions.
params.mpc.cemStagnationUpdates = 8;
params.mpc.cemStagnationTolerance = deg2rad(0.25);
params.mpc.cemStagnationDisableBelowError = deg2rad(3.0);
params.mpc.cemCovarianceResetScale = 2.50;
params.mpc.cemStagnationMeanDirectBlend = 0.75;

% Reset the warm start after a meaningful target change.
params.mpc.cemReferenceResetThreshold = deg2rad(4);

% The Simulink wrapper fills qRefHorizon on every controller update.
params.mpc.useReferencePreview = true;

% -------------------------------------------------------------------------
% Diagnostics
% -------------------------------------------------------------------------
params.mpc.debug = false;
params.mpc.liveProgress = false;
params.mpc.enableTestDiagnostics = false;

params.stage = ...
    "Stable SO(3) CEM v2.2 MPC plus PID/plant and mesh-contact model";

% =========================================================================
% MESH-CONTACT STRUCT USED BY THE CURRENT SIMULINK MODEL
% =========================================================================

% new struct

meshContact.tau_preload_mesh = pp.tau_preload_mesh;
meshContact.tau_gravity_max  = pp.tau_gravity_max;
meshContact.k_mesh           = pp.k_mesh;
meshContact.mesh_backlash    = pp.mesh_backlash;
meshContact.J_mesh           = [1e-3; 1e-3; 1e-3];
% Near-critical damping for J_mesh = 1e-3 and k_mesh = 5:
% c_critical = 2*sqrt(k*J) = 0.1414 N*m*s/rad.
meshContact.c_contact        = [0.15; 0.15; 0.15];
meshContact.c_free           = [0.001; 0.001; 0.001];
meshContact.Ts               = pp.Ts_plant;
meshContact.alpha_preload = pp.alpha_preload;