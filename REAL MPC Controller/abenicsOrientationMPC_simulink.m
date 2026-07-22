function q_des_mpc = abenicsOrientationMPC_simulink( ...
    q_ref, theta_actual, q_des_prev)
%ABENICSORIENTATIONMPC_SIMULINK Simulation-only Simulink wrapper.
%
% This function is called through coder.extrinsic from the Simulink
% MATLAB Function block. The full CEM controller therefore runs in the
% MATLAB interpreter rather than being compiled by Simulink Coder.
%
% Inputs:
%   q_ref         3x1 desired CS-gear orientation, rad
%   theta_actual  4x1 measured output-side MP-gear angles, rad
%   q_des_prev    3x1 previously applied MPC command, rad
%
% Output:
%   q_des_mpc     3x1 current MPC orientation command, rad
%
% The parameter structure is read from the MATLAB base workspace.
% The current q_ref is repeated across the prediction horizon so the MPC
% tracks the current Simulink waypoint instead of using a stale preview.

    % Enforce expected sizes and double precision.
    q_ref = reshape(double(q_ref), 3, 1);
    theta_actual = reshape(double(theta_actual), 4, 1);
    q_des_prev = reshape(double(q_des_prev), 3, 1);

    % Read the current controller parameters from the base workspace.
    paramsLocal = evalin('base', 'params');

    % Build a fixed reference across the entire prediction horizon.
    Np = max(1, round(double(paramsLocal.mpc.Np)));

    paramsLocal.mpc.useReferencePreview = false;
    paramsLocal.mpc.qRefHorizon = repmat(q_ref, 1, Np);

    % Run the interpreted CEM orientation MPC.
    q_des_mpc = abenicsOrientationMPC( ...
        q_ref, ...
        theta_actual, ...
        q_des_prev, ...
        paramsLocal);

    % Guarantee a 3x1 double output for Simulink.
    q_des_mpc = reshape(double(q_des_mpc), 3, 1);
end