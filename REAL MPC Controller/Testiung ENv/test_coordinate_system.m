% =========================================================================
% test_coordinate_system.m
%
% Basic test script for the ABENICS coordinate and signal convention.
%
% This does NOT test real ABENICS kinematics.
% It only checks that the coordinate system, angle units, motor order,
% and signal vectors are consistent.
% =========================================================================

clear; clc;

% -------------------------------------------------------------------------
% Load coordinate parameters
% -------------------------------------------------------------------------
run("params_abenics.m");

fprintf("ABENICS Coordinate System Test\n");
fprintf("==============================\n\n");

% -------------------------------------------------------------------------
% Create a test desired CS gear orientation
% -------------------------------------------------------------------------
% q_des = [roll_des; pitch_des; yaw_des]
%
% Internally, all values are in radians.
% deg2rad(...) is used only because degrees are easier for humans to read.

q_des = [deg2rad(10);
         deg2rad(5);
         deg2rad(-20)];

% -------------------------------------------------------------------------
% Create a test motor angle vector
% -------------------------------------------------------------------------
% theta = [theta_rA;
%          theta_pA;
%          theta_rB;
%          theta_pB]

theta = [deg2rad(0);
         deg2rad(15);
         deg2rad(0);
         deg2rad(-15)];

% -------------------------------------------------------------------------
% Create a test motor command vector
% -------------------------------------------------------------------------
% u = [u_rA;
%      u_pA;
%      u_rB;
%      u_pB]
%
% For now, u is a motor velocity command.
% These values are rad/s.

u = [0.10;
     0.00;
    -0.10;
     0.00];

% -------------------------------------------------------------------------
% Simulated feedback names for now
% -------------------------------------------------------------------------
% In the real system:
% theta_actual comes from 4 rotary encoders.
% q_actual comes from the IMU on the CS gear.
%
% In this test script, we just create example values.

theta_actual = theta;
q_actual = q_des;

% -------------------------------------------------------------------------
% Print world frame
% -------------------------------------------------------------------------
fprintf("World coordinate frame:\n");
fprintf("+X = forward = [%g; %g; %g]\n", params.world.x);
fprintf("+Y = left    = [%g; %g; %g]\n", params.world.y);
fprintf("+Z = up      = [%g; %g; %g]\n\n", params.world.z);

% -------------------------------------------------------------------------
% Print desired orientation
% -------------------------------------------------------------------------
fprintf("Desired CS gear orientation q_des:\n");
for i = 1:3
    fprintf("%-8s = %+8.4f rad = %+8.3f deg\n", ...
        params.qOrder(i), q_des(i), rad2deg(q_des(i)));
end
fprintf("\n");

% -------------------------------------------------------------------------
% Print motor angles
% -------------------------------------------------------------------------
fprintf("Motor angle vector theta:\n");
for i = 1:4
    fprintf("%-10s = %+8.4f rad = %+8.3f deg\n", ...
        params.thetaOrder(i), theta(i), rad2deg(theta(i)));
end
fprintf("\n");

% -------------------------------------------------------------------------
% Print motor commands
% -------------------------------------------------------------------------
fprintf("Motor command vector u:\n");
for i = 1:4
    fprintf("%-6s = %+8.4f rad/s\n", params.uOrder(i), u(i));
end
fprintf("\n");

% -------------------------------------------------------------------------
% Print feedback layout
% -------------------------------------------------------------------------
fprintf("Feedback signal layout:\n");
fprintf("theta_actual comes from 4 rotary encoders.\n");
fprintf("q_actual comes from the IMU on the CS gear.\n\n");

fprintf("theta_actual:\n");
for i = 1:4
    fprintf("%-10s = %+8.4f rad = %+8.3f deg\n", ...
        params.thetaActualOrder(i), theta_actual(i), rad2deg(theta_actual(i)));
end
fprintf("\n");

fprintf("q_actual:\n");
for i = 1:3
    fprintf("%-8s = %+8.4f rad = %+8.3f deg\n", ...
        params.qOrder(i), q_actual(i), rad2deg(q_actual(i)));
end
fprintf("\n");

% -------------------------------------------------------------------------
% Basic consistency checks
% -------------------------------------------------------------------------
expectedThetaOrder = ["theta_rA", "theta_pA", "theta_rB", "theta_pB"];
expectedUOrder     = ["u_rA", "u_pA", "u_rB", "u_pB"];
expectedQOrder     = ["roll", "pitch", "yaw"];

assert(params.angleUnit == "rad", ...
    "ERROR: params.angleUnit must be rad.");

assert(isequal(params.thetaOrder, expectedThetaOrder), ...
    "ERROR: params.thetaOrder does not match the required motor angle order.");

assert(isequal(params.uOrder, expectedUOrder), ...
    "ERROR: params.uOrder does not match the required motor command order.");

assert(isequal(params.qOrder, expectedQOrder), ...
    "ERROR: params.qOrder must be [roll, pitch, yaw].");

assert(numel(q_des) == 3, ...
    "ERROR: q_des must have exactly 3 elements: [roll; pitch; yaw].");

assert(numel(q_actual) == 3, ...
    "ERROR: q_actual must have exactly 3 elements: [roll; pitch; yaw].");

assert(numel(theta) == 4, ...
    "ERROR: theta must have exactly 4 elements.");

assert(numel(theta_actual) == 4, ...
    "ERROR: theta_actual must have exactly 4 elements.");

assert(numel(u) == 4, ...
    "ERROR: u must have exactly 4 elements.");

fprintf("All coordinate convention checks passed.\n");
fprintf("You are ready to build the starter Simulink signal-flow model.\n");