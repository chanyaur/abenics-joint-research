function q_des_mpc = abenicsOrientationMPC_simulink( ...
    q_ref, theta_actual, q_des_prev)
%ABENICSORIENTATIONMPC_SIMULINK Simulation-only wrapper for Simulink.
%
% This wrapper is intentionally executed through coder.extrinsic from a
% MATLAB Function block. Therefore, the full CEM controller runs in the
% MATLAB interpreter instead of being compiled by Simulink Coder.
%
% Inputs:
%   q_ref         3x1 desired CS-gear orientation, rad
%   theta_actual  4x1 output-side MP-gear angles, rad
%   q_des_prev    3x1 previously applied MPC orientation command, rad
%
% Output:
%   q_des_mpc     3x1 current MPC orientation command, rad
%
% The parameter structure is read from the MATLAB base workspace so it does
% not need to cross a Simulink signal port.
%
% For the current fixed-target Simulink integration, the preview horizon is
% the fixed q_ref repeated across Np. A future moving-reference Simulink
% version should accept a 3xNp q_ref_horizon input from a trajectory-preview
% generator.

    q_ref = reshape(double(q_ref), 3, 1);
    theta_actual = reshape(double(theta_actual), 4, 1);
    q_des_prev = reshape(double(q_des_prev), 3, 1);

    paramsLocal = evalin('base', 'params');

    % A fixed point target has the same reference at every future step.
    Np = max(1, round(paramsLocal.mpc.Np));
    paramsLocal.mpc.useReferencePreview = true;
    paramsLocal.mpc.qRefHorizon = repmat(q_ref, 1, Np);

    q_des_mpc = abenicsOrientationMPC( ...
        q_ref, theta_actual, q_des_prev, paramsLocal);

    q_des_mpc = reshape(double(q_des_mpc), 3, 1);
end
