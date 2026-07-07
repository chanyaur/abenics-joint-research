clear; clc;

% ------------------------------------------------------------
% Load ABENICS coordinate parameters
% ------------------------------------------------------------
if exist('params_abenics_coordinate.m', 'file') ~= 2
    error('Cannot find params_abenics_coordinate.m in the current MATLAB folder.');
end

run('params_abenics_coordinate.m');

if ~exist('params', 'var')
    error('params_abenics_coordinate.m must create a variable named params.');
end

% ------------------------------------------------------------
% Add paper beta if missing
% beta = angle formed by the two driving modules
% Use pi/2 only if your physical module A/B layout is 90 degrees.
% ------------------------------------------------------------
if ~isfield(params, 'beta')
    warning(['params.beta was missing. Setting params.beta = pi/2 for testing only. ', ...
        'Confirm this from your physical module layout.']);
    params.beta = pi/2;
end

% ------------------------------------------------------------
% Desired CS-gear orientations
% q_des = [roll; pitch; yaw] in radians
% ------------------------------------------------------------
q_des_0     = [0; 0; 0];
q_des_roll  = [deg2rad(10); 0; 0];
q_des_pitch = [0; deg2rad(10); 0];
q_des_yaw   = [0; 0; deg2rad(10)];
q_des_combo = [deg2rad(10); deg2rad(10); deg2rad(10)];

cases = {
    'zero',  q_des_0;
    'roll',  q_des_roll;
    'pitch', q_des_pitch;
    'yaw',   q_des_yaw;
    'combo', q_des_combo
    };

previous_theta = [];
jump_threshold_rad = pi;   % simple warning threshold for large jumps

fprintf('\nABENICS IK Test\n');
fprintf('beta = %.6f rad = %.3f deg\n\n', params.beta, rad2deg(params.beta));

for k = 1:size(cases, 1)

    case_name = cases{k, 1};
    q_des = cases{k, 2};

    theta_ref = abenicsIK(q_des, params);

    fprintf('Case: %s\n', case_name);
    fprintf('q_des [rad] = [% .6f; % .6f; % .6f]\n', ...
        q_des(1), q_des(2), q_des(3));
    fprintf('q_des [deg] = [% .3f; % .3f; % .3f]\n', ...
        rad2deg(q_des(1)), rad2deg(q_des(2)), rad2deg(q_des(3)));

    fprintf('theta_ref [rad] = [% .6f; % .6f; % .6f; % .6f]\n', ...
        theta_ref(1), theta_ref(2), theta_ref(3), theta_ref(4));
    fprintf('theta_ref [deg] = [% .3f; % .3f; % .3f; % .3f]\n', ...
        rad2deg(theta_ref(1)), rad2deg(theta_ref(2)), ...
        rad2deg(theta_ref(3)), rad2deg(theta_ref(4)));

    % Check for NaN or Inf
    if any(isnan(theta_ref)) || any(isinf(theta_ref))
        warning('Case "%s" produced NaN or Inf in theta_ref.', case_name);
    end

    % Check for large jumps compared with previous case
    if ~isempty(previous_theta)
        jump = norm(theta_ref - previous_theta);
        if jump > jump_threshold_rad
            warning(['Case "%s" jumped by %.3f rad from the previous case. ', ...
                'This may be a branch/singularity issue, not necessarily bad code.'], ...
                case_name, jump);
        end
    end

    previous_theta = theta_ref;

    fprintf('\n');
end

fprintf('Test complete.\n');
fprintf('Reminder: theta_ref is output-side MP-gear angle reference, not raw motor encoder angle.\n');
fprintf('Do not claim physical validation until FK/experiment confirms these branch choices.\n');