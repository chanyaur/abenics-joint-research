clear; clc;

Ts = 0.02; %sampling period in seconds, 50 Hz

% theta_actual(k+1) = theta_actual(k) + Ts * u(k);

A = eye(4); %position matrix stating that the position starts at itself. Eye is an 1 populated identity matrix
B = Ts * eye(4); %each velocity command changes its angle by a product of the time (sampling rate) and the amount of change
C = eye(4); %the plant output is exactly the motor angles
D = zeros(4,4); %something about u not being able to directly jump to the theta_actual

plant = ss(A,B,C,D, Ts); % this is my plant model, that updates every Ts seconds

%creation of mpc controller
mpcobj = mpc(plant, Ts);
mpcobj.PredictionHorizon = 20; %predicts 20 time steps into the future, or 0.4 seconds ahead
mpcobj.ControlHorizon = 5; % NEW NEW NEW, tells the MPC ot optimize the next 5 control moves
mpcobj.Weights.OutputVariables = [10 10 10 10]; %the weights of each variable, this means each is weighted the same. This is SPECIFICALLY the weights of the motor angles
mpcobj.Weights.ManipulatedVariablesRate = [0.1 0.1 0.1 0.1]; %penalizes sudden changes in the motor velocity command, I think this is the u in Chris's diagram
u_max = [2; 2; 2; 2]; % the maximize amount of velocity for each motor

for i = 1:4 %basically assigns the that the min and max velocity for each motor input is +- 2 rad/s
    mpcobj.MV(i).Min = -u_max(i);
    mpcobj.MV(i).Max =  u_max(i);
end

q_des_const = [1; 0.2; 0.1; 0]; %fake placeholder desired ball orientation
q_des_const = q_des_const / norm(q_des_const); %turn q_des_const into a unit vector