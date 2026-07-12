function build_abenics_pid_plant()
%BUILD_ABENICS_PID_PLANT  Programmatically build the referenced model
%   abenics_pid_plant.slx  =  Calculate Error -> PID -> Plant  (inner loop).
%
% This script is the SOURCE OF TRUTH for the referenced model. The .slx is a
% regenerable build artifact -- if it ever conflicts in git, just re-run this.
%
% Model interface (code convention; see params_abenics_coordinate.m for the
% naming caveat vs the hand-drawn diagram):
%   Inport  theta_des     4x1  safe target motor angles (drawing's "q_des")
%   Outport theta_actual  4x1  actual motor angles      (drawing's "q_actual")
%
% Structure:
%   theta_des --(+)--> [PID] --tau--> [Plant] --> theta_actual
%                (-)                                    |
%                 \-------------- feedback ------------/
%
%   PID   : PID Controller block, torque output, output saturated to +/-tau_max
%           with clamping anti-windup (torque/effort limit lives here).
%   Plant : realistic torque-driven actuator, fully vectorized over 4 motors:
%             motor lag -> net torque (viscous + Coulomb + load) ->
%             acceleration limit -> velocity-limited integrator ->
%             position integrator -> backlash.
%
% Design choices (vs the plan):
%   - Vectorized (4x1 signals) instead of a For-Each Subsystem: PID Controller,
%     Integrator, Saturation and Backlash all support vector signals + vector
%     parameters, so one channel of blocks covers all 4 motors. Simpler + fewer
%     blocks to script.
%   - Torque saturation is the PID block's own output limit (with anti-windup),
%     not a separate downstream Saturation, so the integrator cannot wind up.
%
% Run params_abenics_coordinate first (this script also wires it as the model
% PreLoadFcn so 'pp' resolves from the base workspace whenever the model is
% opened/run).

    mdl = 'abenics_pid_plant';
    here = fileparts(mfilename('fullpath'));
    outFile = fullfile(here, [mdl '.slx']);

    % Make sure params exist now (build-time expressions like 'pp.Kp' are only
    % evaluated at compile/sim time, but we validate presence early for clarity).
    if evalin('base', "exist('pp','var')") ~= 1
        evalin('base', 'params_abenics_coordinate;');
    end

    % ---- fresh start -----------------------------------------------------
    if bdIsLoaded(mdl), close_system(mdl, 0); end
    if exist(outFile, 'file'), delete(outFile); end
    new_system(mdl);
    load_system(mdl);

    % =====================================================================
    % ROOT LEVEL
    % =====================================================================
    add_block('simulink/Sources/In1',  [mdl '/theta_des']);
    set_param([mdl '/theta_des'], 'PortDimensions', '4');

    add_block('simulink/Math Operations/Sum', [mdl '/Calculate Error'], ...
              'IconShape', 'rectangular', 'Inputs', '+-');

    add_block('built-in/Subsystem', [mdl '/PID']);
    add_block('built-in/Subsystem', [mdl '/Plant']);

    add_block('simulink/Sinks/Out1', [mdl '/theta_actual']);
    set_param([mdl '/theta_actual'], 'PortDimensions', '4');

    buildPID([mdl '/PID']);
    buildPlant([mdl '/Plant']);

    % ---- root wiring -----------------------------------------------------
    add_line(mdl, 'theta_des/1',       'Calculate Error/1', 'autorouting', 'on');
    add_line(mdl, 'Calculate Error/1', 'PID/1',             'autorouting', 'on');
    add_line(mdl, 'PID/1',             'Plant/1',            'autorouting', 'on');
    add_line(mdl, 'Plant/1',           'theta_actual/1',     'autorouting', 'on');
    % immediate feedback of theta_actual into the error node
    add_line(mdl, 'Plant/1',           'Calculate Error/2',  'autorouting', 'on');

    % =====================================================================
    % SOLVER / CONFIG  (continuous torque plant -> fixed-step ODE)
    % =====================================================================
    set_param(mdl, 'SolverType', 'Fixed-step');
    set_param(mdl, 'Solver',     'ode4');           % Runge-Kutta, fixed-step
    set_param(mdl, 'FixedStep',  'pp.Ts_plant');    % fast plant sub-step (1e-3 s)
    set_param(mdl, 'StopTime',   '10');
    % load params automatically whenever the model is opened / referenced
    set_param(mdl, 'PreLoadFcn', 'params_abenics_coordinate;');

    % =====================================================================
    % LAYOUT + SAVE
    % =====================================================================
    Simulink.BlockDiagram.arrangeSystem(mdl);
    Simulink.BlockDiagram.arrangeSystem([mdl '/PID']);
    Simulink.BlockDiagram.arrangeSystem([mdl '/Plant']);

    save_system(mdl, outFile);
    fprintf('build_abenics_pid_plant: wrote %s\n', outFile);
    % leave the model open for inspection
end

% =========================================================================
% PID subsystem:  e (4x1) -> PID Controller (torque, saturated) -> tau (4x1)
% =========================================================================
function buildPID(sub)
    delete_line_safe(sub);
    delete_default_io(sub);

    add_block('simulink/Sources/In1',  [sub '/e']);
    add_block('simulink/Sinks/Out1',   [sub '/tau']);

    pid = [sub '/PID Controller'];
    add_block('simulink/Continuous/PID Controller', pid);
    set_param(pid, ...
        'TimeDomain',            'Continuous-time', ...
        'Controller',            'PD', ...
        'P',                     'pp.Kp', ...
        'D',                     'pp.Kd', ...
        'N',                     'pp.N', ...
        'LimitOutput',           'on', ...            % torque / effort limit
        'UpperSaturationLimit',  'pp.tau_max', ...
        'LowerSaturationLimit',  '-pp.tau_max');

    add_line(sub, 'e/1',              'PID Controller/1', 'autorouting', 'on');
    add_line(sub, 'PID Controller/1', 'tau/1',            'autorouting', 'on');
end

% =========================================================================
% Plant subsystem (vectorized 4x1):  tau_cmd -> theta_actual
%
%   tau_app = firstOrderLag(tau_cmd, tau_e)                 [state]
%   net     = tau_app - b.*omega - Tc.*tanh(omega/eps) - load
%   alpha   = saturate(net ./ J, +/- alpha_max)
%   omega   = integrate(alpha), limited to +/- omega_max     [state]
%   thetaM  = integrate(omega)                               [state]
%   theta   = backlash(thetaM, width = backlash)
% =========================================================================
function buildPlant(sub)
    delete_line_safe(sub);
    delete_default_io(sub);

    add_block('simulink/Sources/In1',  [sub '/tau_cmd']);
    add_block('simulink/Sinks/Out1',   [sub '/theta_actual']);

    % ---- motor torque lag: d(tau_app)/dt = (tau_cmd - tau_app)/tau_e -----
    add_block('simulink/Math Operations/Sum', [sub '/lag_err'], ...
              'IconShape', 'rectangular', 'Inputs', '+-');
    add_block('simulink/Math Operations/Gain', [sub '/inv_tau_e'], ...
              'Gain', '1./pp.tau_e', 'Multiplication', 'Element-wise(K.*u)');
    add_block('simulink/Continuous/Integrator', [sub '/tau_app'], ...
              'InitialCondition', '0');

    % ---- friction / load -------------------------------------------------
    add_block('simulink/Math Operations/Gain', [sub '/viscous'], ...
              'Gain', 'pp.b', 'Multiplication', 'Element-wise(K.*u)');
    add_block('simulink/Math Operations/Gain', [sub '/inv_omega_eps'], ...
              'Gain', '1./pp.omega_eps', 'Multiplication', 'Element-wise(K.*u)');
    add_block('simulink/Math Operations/Trigonometric Function', [sub '/tanh'], ...
              'Operator', 'tanh');
    add_block('simulink/Math Operations/Gain', [sub '/coulomb'], ...
              'Gain', 'pp.Tc', 'Multiplication', 'Element-wise(K.*u)');
    add_block('simulink/Sources/Constant', [sub '/load'], 'Value', 'pp.load');

    % net = tau_app - viscous - coulomb - load
    add_block('simulink/Math Operations/Sum', [sub '/net_torque'], ...
              'IconShape', 'rectangular', 'Inputs', '+---');

    % ---- acceleration -> velocity -> position ----------------------------
    add_block('simulink/Math Operations/Gain', [sub '/inv_J'], ...
              'Gain', '1./pp.J', 'Multiplication', 'Element-wise(K.*u)');
    add_block('simulink/Discontinuities/Saturation', [sub '/accel_limit'], ...
              'UpperLimit', 'pp.alpha_max', 'LowerLimit', '-pp.alpha_max');
    add_block('simulink/Continuous/Integrator', [sub '/omega'], ...
              'InitialCondition', 'pp.omega0', ...
              'LimitOutput', 'on', ...
              'UpperSaturationLimit', 'pp.omega_max', ...
              'LowerSaturationLimit', '-pp.omega_max');
    add_block('simulink/Continuous/Integrator', [sub '/theta_motor'], ...
              'InitialCondition', 'pp.theta0');
    add_block('simulink/Discontinuities/Backlash', [sub '/backlash'], ...
              'BacklashWidth', 'pp.backlash', 'InitialOutput', 'pp.theta0');

    % ---- wiring ----------------------------------------------------------
    add_line(sub, 'tau_cmd/1',  'lag_err/1',   'autorouting', 'on');
    add_line(sub, 'lag_err/1',  'inv_tau_e/1', 'autorouting', 'on');
    add_line(sub, 'inv_tau_e/1','tau_app/1',   'autorouting', 'on');
    add_line(sub, 'tau_app/1',  'lag_err/2',   'autorouting', 'on');  % lag feedback

    add_line(sub, 'tau_app/1',  'net_torque/1', 'autorouting', 'on');
    add_line(sub, 'viscous/1',  'net_torque/2', 'autorouting', 'on');
    add_line(sub, 'coulomb/1',  'net_torque/3', 'autorouting', 'on');
    add_line(sub, 'load/1',     'net_torque/4', 'autorouting', 'on');

    add_line(sub, 'net_torque/1', 'inv_J/1',       'autorouting', 'on');
    add_line(sub, 'inv_J/1',      'accel_limit/1', 'autorouting', 'on');
    add_line(sub, 'accel_limit/1','omega/1',        'autorouting', 'on');
    add_line(sub, 'omega/1',      'theta_motor/1',  'autorouting', 'on');
    add_line(sub, 'theta_motor/1','backlash/1',     'autorouting', 'on');
    add_line(sub, 'backlash/1',   'theta_actual/1', 'autorouting', 'on');

    % omega feedback into friction terms
    add_line(sub, 'omega/1', 'viscous/1',       'autorouting', 'on');
    add_line(sub, 'omega/1', 'inv_omega_eps/1', 'autorouting', 'on');
    add_line(sub, 'inv_omega_eps/1', 'tanh/1',  'autorouting', 'on');
    add_line(sub, 'tanh/1',  'coulomb/1',        'autorouting', 'on');
end

% =========================================================================
% helpers
% =========================================================================
function tf = bdIsLoaded(mdl)
    tf = ~isempty(find_system('SearchDepth', 0, 'Type', 'block_diagram', 'Name', mdl));
end

function delete_default_io(sub)
% Remove the In1/Out1 that Simulink auto-adds to a new empty Subsystem.
    for nm = {'In1', 'Out1'}
        b = [sub '/' nm{1}];
        if getSimulinkBlockHandle(b) > 0, delete_block(b); end
    end
end

function delete_line_safe(~)
% Placeholder: fresh subsystems have no lines; kept for symmetry/robustness.
end
