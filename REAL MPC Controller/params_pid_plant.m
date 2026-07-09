% =========================================================================
% params_pid_plant.m
%
% Parameters for the ABENICS inner loop:  Calculate Error -> PID -> Plant.
%
% This is the parameter file for the referenced model built by
% build_abenics_pid_plant.m (generates abenics_pid_plant.slx).
%
% Convention (matches params_abenics_coordinate.m / abenicsIK.m / abenicsFK.m):
%   theta = [theta_rA; theta_pA; theta_rB; theta_pB]   (4 motor angles, rad)
%   PID output  = motor TORQUE command                 (N*m)
%   Plant maps torque -> actual motor position accounting for motor lag,
%   inertia, friction/load, acceleration limit, and backlash.
%
% NOTE on naming vs the hand-drawn control loop:
%   The drawing labels motor-space q_des/q_actual and ball pose theta.
%   That is INVERTED from the code. Here we use the CODE convention:
%     drawing "q_des"    -> theta_des    (input to this model)
%     drawing "q_actual" -> theta_actual (output of this model)
%     drawing "theta"    -> ball pose, produced by FK, NOT part of this model.
%
% All plant/PID gains are 4x1 (one per motor) so a For-Each Subsystem can
% partition them across the four motors.
%
% Placeholder numeric values are tagged  % TUNE  -- real system-id values
% come later; this task only fixes the structure.
% =========================================================================

% -------------------------------------------------------------------------
% Pull shared sample time / signal order from the coordinate convention file.
% Do NOT edit params_abenics_coordinate.m (shared, avoids merge conflicts).
% -------------------------------------------------------------------------
if exist('params_abenics_coordinate', 'file') == 2
    params_abenics_coordinate;          % defines struct 'params'
end
if exist('params', 'var') == 1 && isfield(params, 'Ts')
    Ts_ctrl = params.Ts;                % controller / interface rate
else
    Ts_ctrl = 0.02;                     % fallback: 50 Hz
end

clear pp
pp.thetaOrder = ["theta_rA", "theta_pA", "theta_rB", "theta_pB"];
pp.nMotors    = 4;

% -------------------------------------------------------------------------
% Sample times
% -------------------------------------------------------------------------
pp.Ts       = Ts_ctrl;      % controller / interface rate (s), 0.02 = 50 Hz
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
        error('params_pid_plant:size', ...
              'pp.%s must be %dx1.', f, pp.nMotors);
    end
end

fprintf('params_pid_plant: loaded PID+plant params for %d motors, Ts=%.4gs.\n', ...
        pp.nMotors, pp.Ts);
