%DIAG_ABENICS  Report the state of onlyJoints_driven so issues are visible as text.
%   Run:  >> diag_abenics     then paste ALL the output into the chat.
model = 'onlyJoints_driven';
fprintf('==================== ABENICS DIAGNOSTIC ====================\n');
fprintf('pwd = %s\n', pwd);
fprintf('model file exists: %d\n', exist([model '.slx'],'file')==4);

bdclose('all');
try
    load_system(model);
catch e
    fprintf('LOAD FAILED: %s\n', e.message); return;
end

% --- key blocks present? ---
blks = {'thetaCmd_src','thetaDemux','MATLAB Function','Quat2Eul','Gimbal Joint', ...
        'Revolute1','Revolute1_2_'};
fprintf('\n-- blocks present --\n');
for i=1:numel(blks)
    fprintf('  %-16s %d\n', blks{i}, getSimulinkBlockHandle([model '/' blks{i}])>0);
end

% --- joint actuation modes ---
fprintf('\n-- joint actuation modes --\n');
for j = {'Revolute1','Revolute1_2_'}
    try
        fprintf('  %-14s Motion=%s Torque=%s\n', j{1}, ...
            get_param([model '/' j{1}],'MotionActuationMode'), ...
            get_param([model '/' j{1}],'TorqueActuationMode'));
    catch e, fprintf('  %-14s ERR %s\n', j{1}, e.message); end
end

% --- thetaCmd in base workspace? ---
fprintf('\n-- command signal --\n');
if evalin('base','exist(''thetaCmd'',''var'')')
    tc = evalin('base','thetaCmd');
    try, sz = size(tc.Data); catch, sz = size(tc); end
    fprintf('  thetaCmd present, Data size = [%s]\n', num2str(sz));
else
    fprintf('  thetaCmd NOT in base workspace (run the first half of run_abenics)\n');
end

% --- try to compile and capture the REAL error ---
fprintf('\n-- compile test --\n');
try
    set_param(model,'SimulationCommand','update');
    fprintf('  COMPILE OK\n');
catch e
    fprintf('  COMPILE FAILED:\n    %s\n', e.message);
    if ~isempty(e.cause)
        for c = 1:numel(e.cause)
            fprintf('    cause %d: %s\n', c, e.cause{c}.message);
        end
    end
end
fprintf('===========================================================\n');
