function wire_pid_plant_into_main(mainDir)
%WIRE_PID_PLANT_INTO_MAIN  Splice abenics_pid_plant into real_MPC_schema.slx.
%
% wire_pid_plant_into_main()         operates on real_MPC_schema.slx next to
%                                    this script (normal use).
% wire_pid_plant_into_main(folder)   operates on a copy in <folder> (for testing
%                                    without touching the shared model).
%
% Replaces the placeholder plant in the main control-loop model with a single
% Model Reference block that points at abenics_pid_plant (your inner loop:
% Calculate Error -> PID -> Plant).
%
% In real_MPC_schema.slx the placeholder plant is:
%     ... --> [Saturation] --> [Discrete-Time Integrator] --> [ABENICS FK Algo]/1
% This script:
%   1. finds the Saturation + Discrete-Time Integrator blocks,
%   2. records what fed the Saturation input (the theta_des source) and what the
%      Integrator output drove (the FK / feedback path),
%   3. deletes both placeholder blocks,
%   4. drops in a Model block referencing abenics_pid_plant,
%   5. reconnects: <theta_des source> -> Model.theta_des,
%                  Model.theta_actual -> <old integrator destinations>.
%
% Because a realistic torque plant has continuous states, the parent solver is
% switched to a fixed-step ODE (ode4). If the team needs pure discrete, rebuild
% the plant discretely (one-line switch in build_abenics_pid_plant.m).
%
% ------------------------------------------------------------------------
% MANUAL GUI FALLBACK (if you'd rather click it in):
%   1. Open real_MPC_schema.slx. Delete the Saturation and Discrete-Time
%      Integrator blocks (the placeholder plant feeding "ABENICS FK Algo").
%   2. Drag in a Model block (Ports & Subsystems > Model). Set its "Model name"
%      to  abenics_pid_plant.
%   3. Wire the signal that used to feed Saturation into the Model block's
%      theta_des inport; wire the Model block's theta_actual outport to
%      "ABENICS FK Algo" input 1 (and to the MPC feedback, same net).
%   4. Model Settings > Solver: Fixed-step, ode4, fixed step = pp.Ts_plant.
% ------------------------------------------------------------------------

    main = 'real_MPC_schema';
    ref  = 'abenics_pid_plant';
    scriptDir = fileparts(mfilename('fullpath'));
    addpath(scriptDir);
    if nargin < 1 || isempty(mainDir)
        mainDir = scriptDir;    % normal use: edit the real shared model in place
    end

    % make sure the referenced model exists so the Model block can resolve ports
    if exist([ref '.slx'], 'file') ~= 4
        fprintf('%s.slx not found -- building it first...\n', ref);
        build_abenics_pid_plant;
    end

    load_system(fullfile(mainDir, [main '.slx']));

    % ---- locate the placeholder plant blocks -----------------------------
    sat = findOne(main, 'BlockType', 'Saturate');
    intg = findOne(main, 'BlockType', 'DiscreteIntegrator');

    % ---- record connectivity BEFORE deleting -----------------------------
    % source that feeds the Saturation input -> becomes theta_des source
    satPC   = get_param(sat, 'PortConnectivity');
    srcBlk  = satPC(1).SrcBlock;      % handle (-1 if unconnected)
    srcPort = satPC(1).SrcPort;       % 0-based

    % destinations driven by the Integrator output -> become theta_actual dests
    intPC   = get_param(intg, 'PortConnectivity');
    outCon  = intPC(end);             % last connectivity entry = the output port
    dstBlks = outCon.DstBlock;        % array of handles
    dstPorts= outCon.DstPort;         % array, 0-based

    pos = get_param(intg, 'Position'); % reuse footprint for the Model block

    % ---- delete placeholder blocks (and their lines) ---------------------
    deleteBlockAndLines(sat);
    deleteBlockAndLines(intg);

    % ---- add the Model Reference block -----------------------------------
    mb = [main '/PID Plant'];
    add_block('simulink/Ports & Subsystems/Model', mb);
    set_param(mb, 'ModelNameDialog', [ref '.slx']);
    set_param(mb, 'Position', pos + [-20 -20 40 20]);
    % force port resolution from the referenced model interface
    try, set_param(mb, 'ModelName', ref); catch, end

    mph     = get_param(mb, 'PortHandles');
    modelIn = mph.Inport(1);
    modelOut= mph.Outport(1);

    % ---- reconnect -------------------------------------------------------
    if isnumeric(srcBlk) && all(srcBlk > 0)
        sph    = get_param(srcBlk, 'PortHandles');
        srcOut = sph.Outport(srcPort + 1);
        add_line(main, srcOut, modelIn, 'autorouting', 'on');
    else
        warning(['Saturation input was unconnected -- connect the MPC/theta_des ' ...
                 'source to the "PID Plant" block''s theta_des inport by hand.']);
    end

    for k = 1:numel(dstBlks)
        db = dstBlks(k);
        if db <= 0, continue; end
        dph   = get_param(db, 'PortHandles');
        dstIn = dph.Inport(dstPorts(k) + 1);
        add_line(main, modelOut, dstIn, 'autorouting', 'on');
    end

    % ---- parent solver: continuous plant needs an ODE solver -------------
    set_param(main, 'SolverType', 'Fixed-step');
    set_param(main, 'Solver',     'ode4');
    set_param(main, 'FixedStep',  'pp.Ts_plant');
    warning(['Parent solver set to fixed-step ode4 (step = pp.Ts_plant). ' ...
             'Run params_abenics before simulating so pp resolves.']);

    Simulink.BlockDiagram.arrangeSystem(main);
    save_system(main);
    fprintf('wire_pid_plant_into_main: %s now references %s.\n', main, ref);
end

% ---------------------------------------------------------------------------
function h = findOne(sys, param, value)
    hs = find_system(sys, 'SearchDepth', 1, param, value);
    if isempty(hs)
        error('wire_pid_plant_into_main:notFound', ...
              'Could not find a block with %s=%s in %s.', param, value, sys);
    end
    if numel(hs) > 1
        warning('Multiple %s=%s blocks; using the first: %s', param, value, hs{1});
    end
    h = get_param(hs{1}, 'Handle');
end

function deleteBlockAndLines(blk)
    lh = get_param(blk, 'LineHandles');
    segs = [lh.Inport(:); lh.Outport(:)];
    for L = segs(:)'
        if L > 0, delete_line(L); end
    end
    delete_block(blk);
end
