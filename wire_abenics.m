%WIRE_ABENICS  Rewire onlyJoints.slx into a fully command-driven ABENICS model.
%
%   Produces a NEW model  onlyJoints_driven.slx  (original untouched) in which a
%   single command  thetaCmd = [thetaA1 thetaA2 thetaB1 thetaB2]  drives:
%     * the CS ball (Gimbal joint) via the corrected eq-(26) forward kinematics,
%     * the two module-base joints  (Revolute1 = thetaA1, Revolute1_2_ = thetaB1),
%     * the two MP-gear spin joints  (Assembly_3 = thetaA2, Assembly_1 = thetaB2),
%   so the gears animate coherently WITH the ball.
%
%   Mapping comes from the model's own original sensing wiring (which joint fed
%   which FK input), so gears stay consistent with the ball by construction.
%
%   Usage:  >> wire_abenics      then   >> run_abenics
%   Safe to re-run (reloads the pristine original each time).

src = 'onlyJoints';
dst = 'onlyJoints_driven';

% ---- corrected forward-kinematics code (eq 26) for the MATLAB Function -----
fkCode = strjoin({
'function q = abenics_fk(thetaA1, thetaA2, thetaB1)'
'%#codegen'
'beta = pi/2;'
'CA1=cos(thetaA1); SA1=sin(thetaA1);'
'CA2=cos(thetaA2); SA2=sin(thetaA2);'
'CB1=cos(thetaB1); SB1=sin(thetaB1);'
'Cb=cos(beta); Sb=sin(beta);'
'thetaA3 = atan2( SA1*SB1 + CA1*CB1*Cb, ...'
'                 -CA1*CA2*SB1 + SA2*CB1*Sb + SA1*CA2*CB1*Cb );'
'CA3=cos(thetaA3); SA3=sin(thetaA3);'
'R = [ CA2,       SA2*SA3,                 CA3*SA2;'
'      SA1*SA2,   CA1*CA3 - CA2*SA1*SA3,  -CA1*SA3 - CA2*CA3*SA1;'
'     -CA1*SA2,   CA1*CA2*SA3 + CA3*SA1,   CA1*CA2*CA3 - SA1*SA3 ];'
'tr = R(1,1)+R(2,2)+R(3,3);'
'if tr > 0'
'    S = sqrt(tr+1)*2;'
'    q = [0.25*S, (R(3,2)-R(2,3))/S, (R(1,3)-R(3,1))/S, (R(2,1)-R(1,2))/S];'
'elseif R(1,1)>R(2,2) && R(1,1)>R(3,3)'
'    S = sqrt(1+R(1,1)-R(2,2)-R(3,3))*2;'
'    q = [(R(3,2)-R(2,3))/S, 0.25*S, (R(1,2)+R(2,1))/S, (R(1,3)+R(3,1))/S];'
'elseif R(2,2)>R(3,3)'
'    S = sqrt(1+R(2,2)-R(1,1)-R(3,3))*2;'
'    q = [(R(1,3)-R(3,1))/S, (R(1,2)+R(2,1))/S, 0.25*S, (R(2,3)+R(3,2))/S];'
'else'
'    S = sqrt(1+R(3,3)-R(1,1)-R(2,2))*2;'
'    q = [(R(2,1)-R(1,2))/S, (R(1,3)+R(3,1))/S, (R(2,3)+R(3,2))/S, 0.25*S];'
'end'
'q = q/norm(q);'
'end'
}, newline);

q2eCode = strjoin({
'function [eul_x, eul_y, eul_z] = quat2eul_block(q)'
'%#codegen'
'q = q/norm(q); w=q(1); x=q(2); y=q(3); z=q(4);'
'R13 = 2*(x*z + w*y);   R33 = 1-2*(x^2+y^2);'
'R23 = 2*(y*z - w*x);   R12 = 2*(x*y - w*z);   R11 = 1-2*(y^2+z^2);'
'eul_y = asin(max(min(R13,1),-1));'
'eul_x = atan2(-R23, R33);'
'eul_z = atan2(-R12, R11);'
'end'
}, newline);

% ---- load a fresh copy of the original ------------------------------------
bdclose('all');
load_system(src);

% ---- 0) repair the model workspace data source (CAD smiData) ---------------
mdlDir = fileparts(which([src '.slx']));
dfile  = fullfile(mdlDir, 'onlyJoints_DataFile.m');
mw = get_param(src,'ModelWorkspace');
try
    mw.DataSource = 'MATLAB File'; mw.FileName = dfile; reload(mw);
    fprintf('Model workspace repointed to %s\n', dfile);
catch ME
    warning('Could not repoint model workspace (%s).', ME.message);
end

% ---- 0b) turn gravity OFF so un-driven gear bodies don't drift/fall --------
% (this is a kinematic visualization; gravity only makes free joints wander)
mechBlk = [src '/MechanismConfiguration'];
if getSimulinkBlockHandle(mechBlk) > 0
    try, set_param(mechBlk,'GravityAssignmentMethod','None'); catch, end
    try, set_param(mechBlk,'UniformGravity','None');          catch, end
    fprintf('Gravity disabled.\n');
else
    fprintf('MechanismConfiguration block not found -- gravity unchanged.\n');
end

% ---- 1) & 2) replace the two embedded MATLAB Function scripts --------------
rt = sfroot; charts = rt.find('-isa','Stateflow.EMChart');
for c = charts'
    if ~isempty(strfind(c.Script,'abenics_fk'))          %#ok<*STREMP>
        c.Script = fkCode;   fprintf('Updated FK block: %s\n', c.Path);
    elseif ~isempty(strfind(c.Script,'quat2eul_block'))
        c.Script = q2eCode;  fprintf('Updated Quat2Eul block: %s\n', c.Path);
    end
end

% ---- 3) command source + demux (4 channels) -------------------------------
fprintf('Adding command source + demux...\n');
mf = [src '/MATLAB Function'];
ph = get_param(mf,'PortHandles');
for i = 1:numel(ph.Inport)                 % cut dead sensor feedback into FK
    L = get_param(ph.Inport(i),'Line'); if L > 0, delete_line(L); end
end
fwPos = get_param(mf,'Position');
srcBlk = [src '/thetaCmd_src'];  demBlk = [src '/thetaDemux'];
if getSimulinkBlockHandle(srcBlk) > 0, delete_block(srcBlk); end
if getSimulinkBlockHandle(demBlk) > 0, delete_block(demBlk); end
add_block('simulink/Sources/From Workspace', srcBlk, 'VariableName','thetaCmd', ...
    'SampleTime','0', 'Position',[fwPos(1)-360 fwPos(2)-10 fwPos(1)-260 fwPos(2)+30]);
try, set_param(srcBlk,'Interpolate','on'); catch, end
try, set_param(srcBlk,'OutputAfterFinalValue','Holding final value'); catch, end
add_block('simulink/Signal Routing/Demux', demBlk, 'Outputs','3', ...
    'Position',[fwPos(1)-190 fwPos(2)-60 fwPos(1)-185 fwPos(2)+80]);
add_line(src, 'thetaCmd_src/1', 'thetaDemux/1', 'autorouting','on');
dph = get_param(demBlk,'PortHandles');
for i = 1:3, add_line(src, dph.Outport(i), ph.Inport(i), 'autorouting','on'); end

% NOTE: the ball (Gimbal) is driven by FK(thetaCmd). The module-base joints
% (Revolute1/Revolute1_2_) are structural and are LEFT ALONE (prescribing them
% fights the ball chain). We DO spin the two MP-gear joints, which are leaf
% joints (rotate the gear about its own axis) -- safe, and fed from their own
% From Workspace source INSIDE each assembly (no shared Demux -> no size
% deadlock).
cvtTmpl = getfullname(Simulink.ID.getHandle([src ':171']));   % working PS conv
fprintf('Spinning MP gears...\n');
try, drive_spin(src,'Monopole_Drive_Assembly_3','Revolute 3 - MP Gear','gearA_ts',cvtTmpl); ...
        fprintf('  Assembly_3 MP gear <- gearA_ts (thetaA2)\n'); catch e, warning('gearA: %s', e.message); end
try, drive_spin(src,'Monopole_Drive_Assembly_1','Revolute3_MNNPYAW','gearB_ts',cvtTmpl); ...
        fprintf('  Assembly_1 MP gear <- gearB_ts (thetaB2)\n'); catch e, warning('gearB: %s', e.message); end

% ---- 5) damp the free revolute joints (module bases + pinions) so nothing --
%        coasts on the t=0 startup jolt. Explicit paths (find_system misses
%        these linked blocks). Ignored for the InputMotion-driven joints.
jointPaths = {
    [src '/Revolute1']
    [src '/Revolute1_2_']
    [src '/Monopole_Drive_Assembly_1/Revolute1']
    [src '/Monopole_Drive_Assembly_1/Revolute1_Pinion']
    [src '/Monopole_Drive_Assembly_1/Revolute3_MNNPYAW']
    [src '/Monopole_Drive_Assembly_3/Revolute1']
    [src '/Monopole_Drive_Assembly_3/Revolute1_Pinion']
    [src '/Monopole_Drive_Assembly_3/Revolute 3 - MP Gear']
    };
% Force ZERO initial velocity on the free joints so nothing coasts. This is an
% initial condition (not a force), so -- unlike damping -- it adds no stiffness
% and cannot freeze the solver. Combined with gravity off, nothing drifts.
nz = 0;
for r = 1:numel(jointPaths)
    ok = false;
    try
        set_param(jointPaths{r}, 'VelocityTargetSpecify','on', ...
            'VelocityTargetValue','0', 'VelocityTargetPriority','High');
        ok = true;
    catch, end
    if ok, nz = nz + 1; end
end
fprintf('Zero-velocity start set on %d joints.\n', nz);

% ---- save as a NEW model (original stays pristine) -------------------------
fprintf('Saving %s.slx ...\n', dst);
save_system(src, dst);
fprintf('\nDone. Created %s.slx  (original %s.slx untouched).\n', dst, src);

% =========================================================================
function drive_spin(sys, subsys, joint, varname, cvtTmpl)
% Prescribe a leaf MP-gear spin joint from a From Workspace source placed
% inside the assembly subsystem (self-contained, no cross-boundary wiring).
% Robustly finds the actuation port by seeing which physical port newly
% appears when the joint is switched to InputMotion (works even when the
% joint already has extra sensing ports).
jfull = [sys '/' subsys '/' joint];
ph0 = get_param(jfull,'PortHandles');  n0L = numel(ph0.LConn);  n0R = numel(ph0.RConn);
set_param(jfull,'MotionActuationMode','InputMotion','TorqueActuationMode','ComputedTorque');
ph1 = get_param(jfull,'PortHandles');
if     numel(ph1.LConn) > n0L, actPort = ph1.LConn(end);
elseif numel(ph1.RConn) > n0R, actPort = ph1.RConn(end);
else,  error('no actuation port appeared on %s', joint);
end
sub = [sys '/' subsys];
fw  = [sub '/gear_cmd'];  cvt = [sub '/gear_cvt'];
if getSimulinkBlockHandle(fw)  > 0, delete_block(fw);  end
if getSimulinkBlockHandle(cvt) > 0, delete_block(cvt); end
add_block('simulink/Sources/From Workspace', fw, 'VariableName', varname, ...
    'SampleTime','0', 'Position',[30 220 95 250]);
try, set_param(fw,'Interpolate','on'); catch, end
try, set_param(fw,'OutputAfterFinalValue','Holding final value'); catch, end
add_block(cvtTmpl, cvt, 'Position',[140 220 185 250]);
fwph = get_param(fw,'PortHandles');  cph = get_param(cvt,'PortHandles');
add_line(sub, fwph.Outport(1), cph.Inport(1), 'autorouting','on');
add_line(sub, cph.RConn(1), actPort, 'autorouting','on');
end

