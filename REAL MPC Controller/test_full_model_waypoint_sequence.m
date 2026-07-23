% =========================================================================
% test_full_model_waypoint_sequence.m
%
% ABENICS FULL SIMULINK MODEL — MULTI-WAYPOINT ORIENTATION TEST
%
% Runs the complete model:
% q_ref -> MPC -> q_des_mpc -> IK -> PID/plant -> FK -> mesh/backlash
% -> q_actual
%
% ONE-TIME SIMULINK SETUP
% 1. Replace the q_ref Constant block with a From Workspace block.
% 2. Set its variable name to: q_ref_sequence
% 3. Name these signal lines exactly:
%       q_ref
%       q_des_mpc
%       q_actual
%    q_actual must be the line AFTER the MP-CS Backlash block.
% 4. Keep the MPC running at the validated 0.02 s simulated sample time.
% =========================================================================

clear;
clc;
close all;

rng(1);

clear abenicsOrientationMPC
clear abenicsOrientationMPC_simulink

%% USER SETTINGS
modelName = "real_MPC_schema";

run("params_abenics_coordinate.m");

% Each row is [roll pitch yaw] in degrees.
waypointsDeg = [ ...
    20, 20, 20;
    26, 20, 20;
    26, 15, 20;
    26, 15, 24;
    26, 18, 27;
    ];

q_initial = deg2rad(waypointsDeg(1,:).');

theta_initial = abenicsIK(q_initial, params);

assignin('base', 'q_initial', q_initial);
assignin('base', 'theta_initial', theta_initial);

holdTime_s = 3;
transitionTime_s = 0.02;
useSmoothTransitions = false;
TsRef = 0.02;
finalWindow_s = 0.40;

saveResults = true;
resultsMatFile = "full_model_waypoint_results.mat";
waypointCsvFile = "full_model_waypoint_summary.csv";
trackingCsvFile = "full_model_tracking_summary.csv";

%% LOAD PARAMETERS
if isfile("params_abenics_coordinate_merged.m")
    run("params_abenics_coordinate_merged.m");
elseif isfile("params_abenics_coordinate.m")
    run("params_abenics_coordinate.m");
elseif isfile("params_abenics.m")
    run("params_abenics.m");
else
    error("WaypointTest:MissingParams", ...
        "Could not find the ABENICS parameter file.");
end

params.Ts = TsRef;
params.mpc.debug = false;
params.mpc.liveProgress = false;
if isfield(params.mpc,"enableTestDiagnostics")
    params.mpc.enableTestDiagnostics = false;
end

assignin("base","params",params);
assignin("base","pp",pp);

%% BUILD REFERENCE TRAJECTORY
[tRefCommand, qRefCommandDeg, arrivalTimes, holdEndTimes] = ...
    buildWaypointTrajectory(waypointsDeg, holdTime_s, transitionTime_s, ...
    TsRef, useSmoothTransitions);

q_ref_sequence = timeseries(deg2rad(qRefCommandDeg), tRefCommand);
q_ref_sequence.Name = "q_ref_sequence";
assignin("base","q_ref_sequence",q_ref_sequence);

stopTime = tRefCommand(end);

fprintf('\n============================================================\n');
fprintf('ABENICS FULL-MODEL MULTI-WAYPOINT TEST\n');
fprintf('============================================================\n');
fprintf('Model:                 %s\n', modelName);
fprintf('Waypoints:             %d\n', size(waypointsDeg,1));
fprintf('Transition time:       %.3f s\n', transitionTime_s);
fprintf('Hold time:             %.3f s\n', holdTime_s);
fprintf('Reference sample time: %.3f s\n', TsRef);
fprintf('Simulation stop time:  %.3f s\n', stopTime);
fprintf('============================================================\n');
disp(array2table(waypointsDeg, ...
    'VariableNames', {'Roll_deg','Pitch_deg','Yaw_deg'}));

%% PREPARE MODEL
if ~bdIsLoaded(modelName)
    load_system(modelName);
end

set_param(modelName, ...
    "StopTime", num2str(stopTime,"%.12g"), ...
    "SimulationMode", "normal", ...
    "SignalLogging", "off");

% A previous version of this test may have enabled signal logging directly
% on one or more source output ports. Disable those settings because this
% version records data only through To Workspace blocks.
allOutputPorts = find_system( ...
    modelName, ...
    "FindAll", "on", ...
    "LookUnderMasks", "all", ...
    "FollowLinks", "on", ...
    "Type", "port", ...
    "PortType", "outport");

for portIndex = 1:numel(allOutputPorts)
    try
        set_param(allOutputPorts(portIndex), "DataLogging", "off");
    catch
        % Not every output port exposes DataLogging.
    end
end

% Confirm From Workspace reference source exists.
fromBlocks = find_system(modelName, ...
    "LookUnderMasks","all", ...
    "FollowLinks","on", ...
    "BlockType","FromWorkspace");

foundReferenceBlock = false;
for i = 1:numel(fromBlocks)
    try
        expression = string(get_param(fromBlocks{i},"VariableName"));
        if contains(expression,"q_ref_sequence")
            set_param(fromBlocks{i},"VariableName","q_ref_sequence");
            fprintf('Reference source:       %s\n', fromBlocks{i});
            foundReferenceBlock = true;
            break;
        end
    catch
    end
end

if ~foundReferenceBlock
    error('WaypointTest:MissingReferenceBlock', ...
        ['Replace the q_ref Constant with a From Workspace block and ' ...
         'set its variable name to q_ref_sequence.']);
end

% Read the required signals using To Workspace blocks.
%
% Required To Workspace variable names:
%   q_ref_log
%   q_des_mpc_log
%   q_actual_log
%
% Each To Workspace block must use:
%   Save format: Timeseries
%   Sample time: -1
%
% q_actual_log must branch from the signal AFTER the MP-CS Backlash block.

requiredWorkspaceVariables = [ ...
    "q_ref_log", ...
    "q_des_mpc_log", ...
    "q_actual_log"];

toWorkspaceBlocks = find_system( ...
    modelName, ...
    "LookUnderMasks", "all", ...
    "FollowLinks", "on", ...
    "BlockType", "ToWorkspace");

foundWorkspaceVariables = strings(0,1);

for blockIndex = 1:numel(toWorkspaceBlocks)
    blockPath = toWorkspaceBlocks{blockIndex};

    try
        variableName = string(get_param(blockPath, "VariableName"));
    catch
        continue;
    end

    foundWorkspaceVariables(end+1,1) = variableName; %#ok<SAGROW>

    if any(variableName == requiredWorkspaceVariables)
        % Use a timeseries so time and the 3x1 signal are preserved.
        try
            set_param(blockPath, "SaveFormat", "Timeseries");
        catch
            % Some releases expose slightly different options. The user
            % should manually choose Save format = Timeseries if needed.
        end
    end
end

for variableIndex = 1:numel(requiredWorkspaceVariables)
    requiredName = requiredWorkspaceVariables(variableIndex);

    if ~any(foundWorkspaceVariables == requiredName)
        error('WaypointTest:MissingToWorkspaceBlock', ...
            ['Add a To Workspace block for ''%s'' and set its Variable name ' ...
             'to ''%s''. Use Save format = Timeseries.'], ...
            char(requiredName), char(requiredName));
    end
end

%% RUN COMPLETE SIMULINK MODEL
clear abenicsOrientationMPC;
clear abenicsOrientationMPC_simulink;
rehash;

fprintf('\nStarting full Simulink model...\n');
wallStart = tic;
simOut = sim(modelName,"ReturnWorkspaceOutputs","on");

qDesTs    = simOut.get('q_des_mpc_log');
thetaTs   = simOut.get('theta_actual_log');
qPredTs   = simOut.get('q_pred_log');
qActualTs = simOut.get('q_actual_log');

[tDes, qDes]       = unpackSignal(qDesTs, 3);
[tTheta, theta]    = unpackSignal(thetaTs, 4);
[tPred, qPred]     = unpackSignal(qPredTs, 3);
[tActual, qActual] = unpackSignal(qActualTs, 3);

qDesDeg    = rad2deg(qDes);
qPredDeg   = rad2deg(qPred);
qActualDeg = rad2deg(qActual);
thetaDeg   = rad2deg(theta);

wallClockRuntime_s = toc(wallStart);
fprintf('Simulation complete in %.3f wall-clock seconds.\n', ...
    wallClockRuntime_s);

%% READ FULL-MODEL SIGNALS FROM TO WORKSPACE BLOCKS
qRefSignal = getSimulationVariableLocal(simOut, "q_ref_log");
qDesSignal = getSimulationVariableLocal(simOut, "q_des_mpc_log");
qActualSignal = getSimulationVariableLocal(simOut, "q_actual_log");

[tRef, qRefRad] = workspaceSignalToMatrixLocal(qRefSignal, "q_ref_log");
[tDes, qDesRad] = workspaceSignalToMatrixLocal(qDesSignal, "q_des_mpc_log");
[tActual, qActualRad] = workspaceSignalToMatrixLocal(qActualSignal, "q_actual_log");

validateThreeAxis(qRefRad,"q_ref");
validateThreeAxis(qDesRad,"q_des_mpc");
validateThreeAxis(qActualRad,"q_actual");

qRefDeg = rad2deg(qRefRad);
qDesDeg = rad2deg(qDesRad);
qActualDeg = rad2deg(qActualRad);

% Compare all signals at q_actual timestamps.
qRefAtActualDeg = interp1(tRef,qRefDeg,tActual,"previous","extrap");
qDesAtActualDeg = interp1(tDes,qDesDeg,tActual,"previous","extrap");

trackingErrorDeg = wrapDeg(qRefAtActualDeg - qActualDeg);
commandErrorDeg = wrapDeg(qRefAtActualDeg - qDesAtActualDeg);

physicalTrackingErrorDeg = zeros(numel(tActual),1);
physicalCommandErrorDeg = zeros(numel(tActual),1);
for k = 1:numel(tActual)
    Rref = qToRotmXYZ(deg2rad(qRefAtActualDeg(k,:).'));
    Rdes = qToRotmXYZ(deg2rad(qDesAtActualDeg(k,:).'));
    Ract = qToRotmXYZ(deg2rad(qActualDeg(k,:).'));

    physicalTrackingErrorDeg(k) = rad2deg(rotationDistance(Rref,Ract));
    physicalCommandErrorDeg(k) = rad2deg(rotationDistance(Rref,Rdes));
end

%% OVERALL SUMMARY
axisNames = ["Roll";"Pitch";"Yaw"];
actualRms = sqrt(mean(trackingErrorDeg.^2,1)).';
actualPeak = max(abs(trackingErrorDeg),[],1).';
commandRms = sqrt(mean(commandErrorDeg.^2,1)).';
commandPeak = max(abs(commandErrorDeg),[],1).';

trackingSummary = table(axisNames,actualRms,actualPeak,commandRms,commandPeak, ...
    'VariableNames',{'Axis','ActualRmsError_deg','ActualPeakError_deg', ...
    'MpcCommandRmsError_deg','MpcCommandPeakError_deg'});

fprintf('\n============================================================\n');
fprintf('FULL-MODEL TRACKING SUMMARY\n');
fprintf('============================================================\n');
disp(trackingSummary);
fprintf('Physical SO(3) RMS actual error:  %.4f deg\n', ...
    sqrt(mean(physicalTrackingErrorDeg.^2)));
fprintf('Physical SO(3) peak actual error: %.4f deg\n', ...
    max(physicalTrackingErrorDeg));

%% PER-WAYPOINT FINAL VALUES
nWaypoints = size(waypointsDeg,1);
Waypoint = (1:nWaypoints).';
ArrivalTime_s = arrivalTimes(:);
HoldEndTime_s = holdEndTimes(:);
TargetRoll_deg = waypointsDeg(:,1);
TargetPitch_deg = waypointsDeg(:,2);
TargetYaw_deg = waypointsDeg(:,3);

ActualRoll_deg = NaN(nWaypoints,1);
ActualPitch_deg = NaN(nWaypoints,1);
ActualYaw_deg = NaN(nWaypoints,1);
RollError_deg = NaN(nWaypoints,1);
PitchError_deg = NaN(nWaypoints,1);
YawError_deg = NaN(nWaypoints,1);
PhysicalFinalError_deg = NaN(nWaypoints,1);

for i = 1:nWaypoints
    windowEnd = holdEndTimes(i);
    windowStart = max(arrivalTimes(i),windowEnd-finalWindow_s);
    mask = tActual >= windowStart & tActual <= windowEnd;
    if ~any(mask)
        continue;
    end

    qFinalDeg = mean(qActualDeg(mask,:),1);
    qTargetDeg = waypointsDeg(i,:);
    qErrorDeg = wrapDeg(qTargetDeg-qFinalDeg);

    ActualRoll_deg(i) = qFinalDeg(1);
    ActualPitch_deg(i) = qFinalDeg(2);
    ActualYaw_deg(i) = qFinalDeg(3);
    RollError_deg(i) = qErrorDeg(1);
    PitchError_deg(i) = qErrorDeg(2);
    YawError_deg(i) = qErrorDeg(3);

    Rt = qToRotmXYZ(deg2rad(qTargetDeg.'));
    Ra = qToRotmXYZ(deg2rad(qFinalDeg.'));
    PhysicalFinalError_deg(i) = rad2deg(rotationDistance(Rt,Ra));
end

waypointSummary = table(Waypoint,ArrivalTime_s,HoldEndTime_s, ...
    TargetRoll_deg,TargetPitch_deg,TargetYaw_deg, ...
    ActualRoll_deg,ActualPitch_deg,ActualYaw_deg, ...
    RollError_deg,PitchError_deg,YawError_deg,PhysicalFinalError_deg);

fprintf('\n============================================================\n');
fprintf('PER-WAYPOINT FINAL POSITION SUMMARY\n');
fprintf('============================================================\n');
disp(waypointSummary);

%% PLOTS: q_ref, q_des_mpc, q_actual
figure('Name','ABENICS full-model waypoint tracking','Color','w');
tl = tiledlayout(3,1,'TileSpacing','compact','Padding','compact');
yLabels = ["Roll (deg)","Pitch (deg)","Yaw (deg)"];

for axisIndex = 1:3
    nexttile;
    plot(tRef,qRefDeg(:,axisIndex),'--','LineWidth',1.7);
    hold on;
    stairs(tDes,qDesDeg(:,axisIndex),'LineWidth',1.2);
    plot(tActual,qActualDeg(:,axisIndex),'LineWidth',1.7);

    for waypointIndex = 1:nWaypoints
        xline(arrivalTimes(waypointIndex),':',sprintf('P%d',waypointIndex), ...
            'LabelVerticalAlignment','bottom','HandleVisibility','off');
    end

    grid on;
    ylabel(yLabels(axisIndex));
    if axisIndex == 1
        legend('q_{ref}','q_{des,MPC}', ...
            'q_{actual} after PID/plant/backlash','Location','best');
    end
    if axisIndex == 3
        xlabel('Simulation time (s)');
    end
end
title(tl,'ABENICS multi-waypoint full-model orientation tracking');

figure('Name','ABENICS physical tracking error','Color','w');
plot(tActual,physicalTrackingErrorDeg,'LineWidth',1.6);
hold on;
plot(tActual,physicalCommandErrorDeg,'--','LineWidth',1.3);
for waypointIndex = 1:nWaypoints
    xline(arrivalTimes(waypointIndex),':',sprintf('P%d',waypointIndex), ...
        'HandleVisibility','off');
end
grid on;
xlabel('Simulation time (s)');
ylabel('SO(3) error (deg)');
title('Physical orientation error');
legend('q_{ref} to q_{actual}','q_{ref} to q_{des,MPC}', ...
    'Location','best');

figure('Name','ABENICS waypoint final error','Color','w');
bar(Waypoint,[abs(RollError_deg),abs(PitchError_deg), ...
    abs(YawError_deg),PhysicalFinalError_deg]);
grid on;
xlabel('Waypoint');
ylabel('Final error (deg)');
title('Final full-model error at each waypoint');
legend('|Roll error|','|Pitch error|','|Yaw error|', ...
    'Physical SO(3) error','Location','best');

axisNames = {'Roll', 'Pitch', 'Yaw'};

figure('Color','w');
layout = tiledlayout(3,1, ...
    'TileSpacing','compact', ...
    'Padding','compact');

for axisIndex = 1:3
    nexttile;

    plot(tDes, qDesDeg(:,axisIndex), ...
        'LineWidth', 1.2);
    hold on;

    plot(tPred, qPredDeg(:,axisIndex), ...
        'LineWidth', 1.2);

    plot(tActual, qActualDeg(:,axisIndex), ...
        'LineWidth', 1.2);

    grid on;
    ylabel([axisNames{axisIndex}, ' (deg)']);

    if axisIndex == 1
        legend( ...
            'q_{des,MPC}', ...
            'q_{pred} after PID/plant/FK', ...
            'q_{actual} after bias/backlash', ...
            'Location','best');
    end

    if axisIndex == 3
        xlabel('Time (s)');
    end
end

figure('Color','w');

plot(tTheta, thetaDeg, 'LineWidth', 1.1);
grid on;

xlabel('Time (s)');
ylabel('Output-side MP angle (deg)');
title('PID/plant motor-angle output');

legend( ...
    '\theta_{rA}', ...
    '\theta_{pA}', ...
    '\theta_{rB}', ...
    '\theta_{pB}', ...
    'Location','best');

title(layout, ...
    'Location of tracking error in the complete model');

%% SAVE
if saveResults
    writetable(waypointSummary,waypointCsvFile);
    writetable(trackingSummary,trackingCsvFile);
    save(resultsMatFile, ...
        'waypointsDeg','tRef','qRefDeg','tDes','qDesDeg', ...
        'tActual','qActualDeg','trackingErrorDeg', ...
        'physicalTrackingErrorDeg','physicalCommandErrorDeg', ...
        'waypointSummary','trackingSummary','wallClockRuntime_s');

    fprintf('\nSaved:\n');
    fprintf('  %s\n',resultsMatFile);
    fprintf('  %s\n',waypointCsvFile);
    fprintf('  %s\n',trackingCsvFile);
end

assignin('base','fullModelWaypointSummary',waypointSummary);
assignin('base','fullModelTrackingSummary',trackingSummary);
assignin('base','fullModelWaypointSimulationOutput',simOut);

%% LOCAL FUNCTIONS
function [time,dataDeg,arrivalTimes,holdEndTimes] = ...
    buildWaypointTrajectory(waypointsDeg,holdTime,transitionTime,Ts,useSmooth)

    validateattributes(waypointsDeg,{'numeric'},{'2d','ncols',3,'finite'});
    n = size(waypointsDeg,1);
    if n < 1
        error('WaypointTest:NoWaypoints','At least one waypoint is required.');
    end

    time = [];
    dataDeg = [];
    arrivalTimes = zeros(n,1);
    holdEndTimes = zeros(n,1);
    currentTime = 0;

    nHold = max(1,round(holdTime/Ts));
    tHold = currentTime + (0:nHold).'*Ts;
    time = [time;tHold]; %#ok<AGROW>
    dataDeg = [dataDeg;repmat(waypointsDeg(1,:),numel(tHold),1)]; %#ok<AGROW>
    arrivalTimes(1) = 0;
    holdEndTimes(1) = tHold(end);
    currentTime = tHold(end);

    for i = 2:n
        nTransition = max(1,round(transitionTime/Ts));
        tau = (1:nTransition).'/nTransition;
        if useSmooth
            blend = 3*tau.^2-2*tau.^3;
        else
            blend = ones(size(tau));
        end

        q0 = waypointsDeg(i-1,:);
        q1 = waypointsDeg(i,:);
        qTransition = q0 + blend.*(q1-q0);
        tTransition = currentTime + (1:nTransition).'*Ts;

        time = [time;tTransition]; %#ok<AGROW>
        dataDeg = [dataDeg;qTransition]; %#ok<AGROW>
        currentTime = tTransition(end);
        arrivalTimes(i) = currentTime;

        nHold = max(1,round(holdTime/Ts));
        tHold = currentTime + (1:nHold).'*Ts;
        time = [time;tHold]; %#ok<AGROW>
        dataDeg = [dataDeg;repmat(q1,numel(tHold),1)]; %#ok<AGROW>
        currentTime = tHold(end);
        holdEndTimes(i) = currentTime;
    end

    [time,uniqueIndex] = unique(time,'stable');
    dataDeg = dataDeg(uniqueIndex,:);
end

function value = getSimulationVariableLocal(simOut, variableName)
% Retrieve a To Workspace variable from SimulationOutput, with a base-
% workspace fallback for Simulink releases that do not package it in simOut.

    value = [];

    try
        availableVariables = string(simOut.who);

        if any(availableVariables == variableName)
            value = simOut.get(char(variableName));
        end
    catch
        % Continue to base-workspace fallback.
    end

    if isempty(value)
        existsInBase = evalin('base', ...
            sprintf('exist(''%s'',''var'')', char(variableName)));

        if existsInBase
            value = evalin('base', char(variableName));
        end
    end

    if isempty(value)
        error('WaypointTest:MissingWorkspaceOutput', ...
            ['The To Workspace variable ''%s'' was not produced. Check that ' ...
             'the block is connected and Save format is Timeseries.'], ...
            char(variableName));
    end
end

function [time,data] = workspaceSignalToMatrixLocal(value, variableName)
% Convert a To Workspace timeseries, timetable, or structure into:
%   time = Nx1
%   data = Nx3 [roll pitch yaw]

    if isa(value,'timeseries')
        time = double(value.Time(:));
        data = squeeze(double(value.Data));

    elseif istimetable(value)
        time = seconds(value.Properties.RowTimes);
        data = double(value.Variables);

    elseif isstruct(value) && isfield(value,'time') && ...
            isfield(value,'signals')
        time = double(value.time(:));
        data = squeeze(double(value.signals.values));

    else
        error('WaypointTest:SignalType', ...
            ['Unsupported To Workspace format for %s: %s. ' ...
             'Set Save format to Timeseries.'], ...
            char(variableName), class(value));
    end

    if isvector(data)
        data = data(:);
    end

    % Common logged layouts include Nx3, 3xN, Nx1x3, or 1x3xN.
    data = squeeze(data);

    if size(data,1) == 3 && size(data,2) == numel(time)
        data = data.';
    elseif size(data,2) ~= 3 && numel(data) == 3*numel(time)
        data = reshape(data, numel(time), 3);
    end
end

function validateThreeAxis(data,name)
    if size(data,2) ~= 3
        error('WaypointTest:SignalSize', ...
            '%s must be Nx3 [roll pitch yaw].',name);
    end
end

function angleDeg = wrapDeg(angleDeg)
    angleDeg = rad2deg(atan2(sind(angleDeg),cosd(angleDeg)));
end

function R = qToRotmXYZ(q)
    r=q(1); p=q(2); y=q(3);
    cr=cos(r); sr=sin(r);
    cp=cos(p); sp=sin(p);
    cy=cos(y); sy=sin(y);
    Rx=[1 0 0;0 cr -sr;0 sr cr];
    Ry=[cp 0 sp;0 1 0;-sp 0 cp];
    Rz=[cy -sy 0;sy cy 0;0 0 1];
    R=Rx*Ry*Rz;
end

function d = rotationDistance(R1,R2)
    Re=R1.'*R2;
    c=(trace(Re)-1)/2;
    c=min(1,max(-1,c));
    d=acos(c);
end

function [time, data] = unpackSignal(signal, expectedWidth)

time = double(signal.Time(:));
data = squeeze(double(signal.Data));

if size(data,1) == expectedWidth && ...
        size(data,2) == numel(time)
    data = data.';
end

if size(data,2) ~= expectedWidth && ...
        numel(data) == expectedWidth*numel(time)
    data = reshape(data, numel(time), expectedWidth);
end

if size(data,2) ~= expectedWidth
    error('Unexpected signal width. Expected %d columns.', ...
        expectedWidth);
end
end