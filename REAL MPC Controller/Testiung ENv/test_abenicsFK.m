clear; clc;

if exist('params_abenics_coordinate.m', 'file') == 2
    run('params_abenics_coordinate.m');
else
    error('Cannot find params_abenics_coordinate.m in the current MATLAB folder.');
end

if ~isfield(params, 'beta')
    params.beta = pi/2;
end

beta = params.beta;

q_cases = {
    'zero',  [0; 0; 0];
    'pitch', [0; deg2rad(10); 0];
    'yaw',   [0; 0; deg2rad(10)];
    'combo', [deg2rad(10); deg2rad(10); deg2rad(10)]
};

fprintf('\nIK then FK Consistency Test\n');
fprintf('beta = %.6f rad = %.3f deg\n\n', beta, rad2deg(beta));

for k = 1:size(q_cases, 1)

    case_name = q_cases{k, 1};
    q_des = q_cases{k, 2};

    theta_ref = abenicsIK(q_des, params);
    q_pred = abenicsFK(theta_ref, beta);

    q_error = q_pred - q_des;

    fprintf('Case: %s\n', case_name);

    fprintf('q_des [deg] = [% .3f; % .3f; % .3f]\n', ...
            rad2deg(q_des(1)), rad2deg(q_des(2)), rad2deg(q_des(3)));

    fprintf('theta_ref [deg] = [% .3f; % .3f; % .3f; % .3f]\n', ...
            rad2deg(theta_ref(1)), rad2deg(theta_ref(2)), ...
            rad2deg(theta_ref(3)), rad2deg(theta_ref(4)));

    fprintf('q_pred [deg] = [% .3f; % .3f; % .3f]\n', ...
            rad2deg(q_pred(1)), rad2deg(q_pred(2)), rad2deg(q_pred(3)));

    fprintf('error [deg] = [% .6f; % .6f; % .6f]\n\n', ...
            rad2deg(q_error(1)), rad2deg(q_error(2)), rad2deg(q_error(3)));
end

fprintf('Done.\n');
fprintf('Reminder: q_pred is FK model prediction. q_actual should mean IMU feedback on hardware.\n');