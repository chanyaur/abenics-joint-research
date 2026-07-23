%% run this file first before running reduced_abenics.slx

time = (0:params.Ts:10)';

q = repmat(deg2rad([15 10 5]),length(time),1);

% q(:,2) = deg2rad(10) + deg2rad(5)*sin(2*pi*0.2*time);  % 5 deg to 10 deg

q(:,2) = deg2rad(10) + deg2rad(30)*sin(2*pi*0.1*time);  % -20 deg to 40 deg

q_ref_ts = timeseries(q,time);

disp("loaded animation constant to feed into MPC");
