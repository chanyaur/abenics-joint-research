% %% run this file first before running reduced_abenics.slx
% 
% time = (0:params.Ts:10)';
% 
% q = repmat(deg2rad([15 10 5]),length(time),1);
% 
% % q(:,2) = deg2rad(10) + deg2rad(5)*sin(2*pi*0.2*time);  % 5 deg to 10 deg
% 
% q(:,2) = deg2rad(10) + deg2rad(30)*sin(2*pi*0.1*time);  % -20 deg to 40 deg
% 
% q_ref_ts = timeseries(q,time);
% 
% disp("loaded animation constant to feed into MPC");


%% Time
time = (0:params.Ts:20)';

roll  = deg2rad(35) * sin(2*pi*0.10*time);
pitch = deg2rad(30) * sin(2*pi*0.13*time + pi/2);
yaw   = deg2rad(20) * sin(2*pi*0.07*time + pi/3);

q = [roll pitch yaw];

q_ref_ts = timeseries(q,time);

disp("loaded animation constant to feed into MPC");