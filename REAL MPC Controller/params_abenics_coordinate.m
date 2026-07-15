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
%
% MERGE NOTE:
%   This version keeps the first file's SO(3) CEM MPC configuration as the
%   source of truth and adds only the second file's tendon-preload and
%   mesh-level backlash parameters.
%
%   The second file's older fmincon/detour controller settings were NOT
%   imported.
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
% Tendon preload (semi-active, constant-setpoint approximation)
% -------------------------------------------------------------------------
% Represents the settled torque delivered by a semi-active antagonistic
% tendon pair: one active tensioner and one passive spring.
%
% This parameter does NOT model tensioner lag or a separate tension-control
% loop. The plant must explicitly add pp.tau_preload to its motor net-torque
% equation for this setting to affect simulation.
%
% Suggested sign convention in the plant:
%   net = tau_applied + tau_preload ...
%         - b.*omega ...
%         - Tc.*tanh(omega./omega_eps) ...
%         - load;
pp.tau_preload = [0; 0; 0; 0];  % TUNE constant preload torque per motor (N*m)

% Smoothing constant for the Coulomb friction tanh() so it stays differentiable
% for the ODE solver (small -> closer to ideal sign()).
pp.omega_eps = 1e-3;                        % rad/s

% -------------------------------------------------------------------------
% Mesh-level CS-gear / MP-gear preload and backlash
% -------------------------------------------------------------------------
% This is separate from the four per-motor driving-module backlash values.
% It represents backlash at the final CS/MP gear mesh after forward
% kinematics. The current project FK filename is abenicsFL.m.
%
% These fields are parameters only. They do not change q_pred or q_actual
% unless the plant/sensor model explicitly implements the mesh model.
%
% Proposed simulation model:
%
%   tau_gravity = tau_gravity_max .* sin(q_pred);
%
%   q_bias_raw = (tau_preload_mesh - tau_gravity) ./ k_mesh;
%
%   q_bias = min(max(q_bias_raw, -mesh_backlash/2), ...
%                                mesh_backlash/2);
%
%   q_model = applyBacklash(q_pred + q_bias, mesh_backlash);
%
% q_model is the simulated orientation after mesh effects. On hardware,
% q_actual remains the IMU-measured CS-gear orientation.
%
% The 0.8-degree value is a preliminary dial-gauge-based estimate and must
% remain marked as TUNE until experimentally validated.
pp.mesh_backlash = deg2rad([0.8; 0.8; 0.8]); ...
    % TUNE mesh backlash width [roll; pitch; yaw] (rad)

pp.tau_preload_mesh = [1; 1; 1]; ...
    % TUNE mesh preload torque [roll; pitch; yaw] (N*m)

pp.tau_gravity_max = [0.025; 0.025; 0.025]; ...
    % TUNE maximum gravity disturbance [roll; pitch; yaw] (N*m)

pp.k_mesh = [5; 5; 5]; ...
    % TUNE effective mesh stiffness [roll; pitch; yaw] (N*m/rad)

% -------------------------------------------------------------------------
% Initial conditions
% -------------------------------------------------------------------------
pp.theta0 = [0; 0; 0; 0];   % initial motor positions (rad)
pp.omega0 = [0; 0; 0; 0];   % initial motor velocities (rad/s)

% -------------------------------------------------------------------------
% Sanity checks: every per-motor field must be nMotors x 1.
% -------------------------------------------------------------------------
perMotorFields = { 'Kp','Ki','Kd','N','tau_max','J','b','Tc','tau_e', ...
                   'alpha_max','omega_max','backlash','load', ...
                   'tau_preload','theta0','omega0' };
for k = 1:numel(perMotorFields)
    f = perMotorFields{k};
    if ~isequal(size(pp.(f)), [pp.nMotors, 1])
        error('params_abenics:size', ...
              'pp.%s must be %dx1.', f, pp.nMotors);
    end
end

fprintf('params_abenics: loaded coordinate conventions + PID/plant params for %d motors, Ts=%.4gs.\n', ...
        pp.nMotors, pp.Ts);


% Mesh-level fields use [roll; pitch; yaw], so they must be 3x1 rather than
% the 4x1 motor-space shape used above.
perAxisFields = { ...
    'mesh_backlash', ...
    'tau_preload_mesh', ...
    'tau_gravity_max', ...
    'k_mesh'};

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
params.singularity.dangerDistance  = deg2rad(2);  % rad
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
% Prediction and control horizons
% -----------------------------
% The CEM decision space uses cemNumberOfKnots smooth local
% rotation-vector knots on SO(3), even though the command sequence contains
% Nc physical rotation increments.
params.mpc.Np = 33;
params.mpc.Nc = 12;
params.mpc.maxQStep = deg2rad([2; 2; 2]);
params.mpc.orientationRepresentation = "SO3LocalRotationVector";

% -----------------------------
% Hard constraints
% -----------------------------
params.mpc.constraintTolerance = 1e-6;
params.mpc.qMin = deg2rad([-45; -45; -45]);
params.mpc.qMax = deg2rad([ 45;  45;  45]);
params.mpc.thetaMin = deg2rad([-360; -360; -360; -360]);
params.mpc.thetaMax = deg2rad([ 360;  360;  360;  360]);

% IK can return equivalent rotary angles separated by integer multiples of
% 2*pi. Unwrap each periodic MP-gear angle to the representation nearest the
% previous continuous motor command/state before checking limits or dynamics.
params.mpc.thetaUnwrapEnabled = true;
params.mpc.thetaPeriodic = true(4, 1);

% The current test configuration allows each output-side MP shaft to use the
% full +/-360 deg continuous range confirmed for this project. Continuous IK
% unwrapping is still required so crossing a revolution boundary is smooth.
params.mpc.enforceThetaPositionLimits = true;

params.mpc.omegaMax = deg2rad([180; 180; 180; 180]);
params.mpc.alphaMax = deg2rad([720; 720; 720; 720]);

% Sample predicted motion between discrete plant states so a candidate cannot
% jump across a pole while both endpoints appear safe.
params.mpc.cemTransitionSamples = 3;
params.mpc.transitionSafetySamples = 9;

% -----------------------------
% Internal MPC plant prediction model
% -----------------------------
params.plant.KpPlant = 80;
params.plant.KdPlant = 12;

% -----------------------------
% Existing MPC cost weights retained as the CEM starting baseline
% -----------------------------
params.mpc.wTrack       = 2000;
params.mpc.wTerminal    = 1000;
params.mpc.wSmooth      = 1000;
params.mpc.wMotor       = 100;
params.mpc.wSingularity = 3000;
params.mpc.wOmega       = 0.5;

% -----------------------------
% Cross-Entropy Method search settings
% -----------------------------
% Four smooth knots produce 12 continuous decision variables:
%   3 local rotation-vector components x 4 knots.
params.mpc.cemNumberOfKnots = 4;

% Fixed computation budget. Every MPC update evaluates exactly this many
% candidates unless MATLAB is interrupted.
params.mpc.cemPopulationSize = 64;
params.mpc.cemIterations = 3;
params.mpc.cemProgressEveryCandidates = 16;
params.mpc.cemEliteFraction = 0.15;
params.mpc.cemSmoothing = 0.70;

% Initial and minimum search spread for local SO(3) rotation vectors.
params.mpc.cemInitialStd = deg2rad([1.25; 1.25; 1.25]);
params.mpc.cemMinimumStd = deg2rad([0.10; 0.10; 0.10]);
params.mpc.cemMaximumStd = deg2rad([2.00; 2.00; 2.00]);
params.mpc.cemWarmStartStdInflation = 1.25;
params.mpc.cemNearSingularityStdInflation = 1.50;

% While the physical target error is still larger than 3 deg, prevent the
% learned distribution from collapsing below 0.35 deg per knot component.
% Once near the target, cemMinimumStd is used again for accurate settling.
params.mpc.cemSigmaFloor = deg2rad([0.35; 0.35; 0.35]);
params.mpc.cemSettlingErrorThreshold = deg2rad(3.0);

% Correlated noise makes complete candidate paths smooth and gives the
% population a realistic chance to sustain roll/yaw motion around a pole.
params.mpc.cemTemporalCorrelation = 0.85;

% A permanent broad subset continues trialing competing routes even after the
% learned covariance narrows. The fixed exploration std does not collapse.
%params.mpc.cemExplorationFraction = 0.30;
%params.mpc.cemExplorationStd = deg2rad([1.25; 1.25; 1.25]);

% If physical target error fails to improve by 0.25 deg over 8 updates while
% still more than 3 deg from target, widen the covariance and re-center most
% of the mean toward the direct SO(3) command. Broad antithetic samples then
% test both sides of the blocked direct route; no detour is predefined.
%params.mpc.cemStagnationUpdates = 8;
%params.mpc.cemStagnationTolerance = deg2rad(0.25);
%params.mpc.cemCovarianceResetScale = 2.50;
%params.mpc.cemStagnationMeanDirectBlend = 0.75;

% Reset the warm-start distribution when the requested reference changes by
% more than this physical SO(3) rotation distance.
params.mpc.cemReferenceResetThreshold = deg2rad(8);

% Multimodal CEM
params.mpc.cemModeCount = 4;

% Total population remains 64:
% 4 modes x 16 candidates = 64 candidates per iteration
params.mpc.cemPopulationPerMode = 16;

% Four elites per mode when using 16 candidates
params.mpc.cemEliteFraction = 0.25;

% Initial symmetric separation between search modes
params.mpc.cemModeLateralBias = deg2rad(0.75);
params.mpc.cemModeTwistBias = deg2rad(0.35);

% Bias is strongest at the beginning of the predicted command sequence
params.mpc.cemModeBiasProfile = [1.00, 0.70, 0.35, 0.00];

% Only activate four-mode search when the direct path approaches this close
params.mpc.cemMultimodalActivationDistance = ...
    params.singularity.warningDistance;

% -----------------------------
% Diagnostics
% -----------------------------
params.mpc.debug = true;
params.mpc.liveProgress = true;
params.mpc.enableTestDiagnostics = false;

params.stage = "SO(3) CEM MPC plus tendon-preload and mesh-backlash parameters";
