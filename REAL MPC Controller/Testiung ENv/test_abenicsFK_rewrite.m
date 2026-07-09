clear; clc;

if exist('params_abenics_coordinate.m', 'file') ~= 2
    error('Cannot find params_abenics_coordinate.m.');
end

run('params_abenics_coordinate.m');

if ~exist('params', 'var')
    error('params_abenics_coordinate.m must create params.');
end

if ~isfield(params, 'beta')
    params.beta = pi/2;
end

passTol = deg2rad(0.1);

q_cases = {
    'zero',   [0; 0; 0];
    'roll',   [deg2rad(10); 0; 0];
    'pitch',  [0; deg2rad(10); 0];
    'yaw',    [0; 0; deg2rad(10)];
    'combo',  [deg2rad(10); deg2rad(10); deg2rad(10)];
    'mixed1', [deg2rad(-10); deg2rad(5); deg2rad(15)];
    'mixed2', [deg2rad(20); deg2rad(-10); deg2rad(30)]
};

fprintf('\nABENICS Analytic 4-Input FK Test\n');
fprintf('beta = %.6f rad = %.3f deg\n', params.beta, rad2deg(params.beta));
fprintf('Pass tolerance = %.3f deg\n\n', rad2deg(passTol));

allPassed = true;

for k = 1:size(q_cases, 1)

    name = q_cases{k, 1};
    q_test = q_cases{k, 2};

    theta = abenicsIK(q_test, params);

    [q_recovered, info] = abenicsFK(theta, params);

    q_error = wrapAngleDifferenceLocal(q_recovered, q_test);

    passed = max(abs(q_error)) < passTol;

    if ~passed
        allPassed = false;
    end

    fprintf('Test: %s\n', name);

    fprintf('q_test [deg]      = [% .3f; % .3f; % .3f]\n', ...
        rad2deg(q_test(1)), rad2deg(q_test(2)), rad2deg(q_test(3)));

    fprintf('theta [deg]       = [% .3f; % .3f; % .3f; % .3f]\n', ...
        rad2deg(theta(1)), rad2deg(theta(2)), ...
        rad2deg(theta(3)), rad2deg(theta(4)));

    fprintf('q_recovered [deg] = [% .3f; % .3f; % .3f]\n', ...
        rad2deg(q_recovered(1)), rad2deg(q_recovered(2)), rad2deg(q_recovered(3)));

    fprintf('q_error [deg]     = [% .6f; % .6f; % .6f]\n', ...
        rad2deg(q_error(1)), rad2deg(q_error(2)), rad2deg(q_error(3)));

    fprintf('columnDot         = %.12e\n', info.columnDot);
    fprintf('crossNorm         = %.12e\n', info.crossNorm);
    fprintf('detR              = %.12e\n', info.detR);

    if passed
        fprintf('RESULT: PASS\n\n');
    else
        fprintf('RESULT: FAIL\n\n');
    end
end

if allPassed
    fprintf('ALL ANALYTIC 4-INPUT FK TESTS PASSED.\n');
    fprintf('Analytic FK can replace old incomplete FK after one more Simulink check.\n');
else
    fprintf('ONE OR MORE TESTS FAILED.\n');
    fprintf('Do not use this FK for singularity detection or MPC yet.\n');
    fprintf('Most likely issue: c2 sign/order or Rz(beta) direction.\n');
end

function d = wrapAngleDifferenceLocal(a, b)
    d = atan2(sin(a - b), cos(a - b));
end