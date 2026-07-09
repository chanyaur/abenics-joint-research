clear; clc;

% Load params script
params_abenics_coordinate;

% Make sure beta exists
if ~isfield(params, 'beta')
    params.beta = pi/2;
end

% Make sure singularity settings exist
if ~isfield(params, 'singularity')
    params.singularity.epsilon = 1e-6;
    params.singularity.warningThreshold = 1e-2;
    params.singularity.dangerThreshold  = 1e-3;
end

testNames = {
    'zero'
    'roll 10 deg'
    'pitch 10 deg'
    'yaw 10 deg'
    'combo 10/10/10 deg'
    };

qTests = {
    [0; 0; 0]
    [deg2rad(10); 0; 0]
    [0; deg2rad(10); 0]
    [0; 0; deg2rad(10)]
    [deg2rad(10); deg2rad(10); deg2rad(10)]
    };

for k = 1:numel(qTests)
    q_des = qTests{k};

    theta = abenicsIK(q_des, params);
    q_pred = localCallFK_test(theta, params);

    [s, info] = singularityMeasure(theta, q_pred, params);

    fprintf('\n==============================\n');
    fprintf('Test: %s\n', testNames{k});

    fprintf('q_des deg:\n');
    disp(rad2deg(q_des));

    fprintf('theta deg:\n');
    disp(rad2deg(theta));

    fprintf('q_pred deg:\n');
    disp(rad2deg(q_pred));

    fprintf('singularity score s:\n');
    disp(s);

    fprintf('singular values:\n');
    disp(info.singularValues);

    fprintf('condition number:\n');
    disp(info.conditionNumber);

    if info.isDanger
        fprintf('STATUS: DANGER near singularity\n');
    elseif info.isWarning
        fprintf('STATUS: WARNING near singularity\n');
    else
        fprintf('STATUS: SAFE by temporary thresholds\n');
    end

    fprintf('q mismatch deg:\n');
    disp(rad2deg(info.qMismatch));

    fprintf('Inputs used:\n');
    disp(info.inputsUsed);

    fprintf('Jacobian size:\n');
    disp(info.jacobianSize);

    if isfield(info, 'columnDot')
        fprintf('columnDot:\n');
        disp(info.columnDot);
    end

    if isfield(info, 'crossNorm')
        fprintf('crossNorm:\n');
        disp(info.crossNorm);
    end

    if isfield(info, 'detR')
        fprintf('detR:\n');
        disp(info.detR);
    end
end

% Local FK caller for test script
function q_fk = localCallFK_test(theta, params)
try
    q_fk = abenicsFL(theta, params);
catch
    try
        q_fk = abenicsFL(theta, params.beta);
    catch
        try
            q_fk = abenicsFK(theta, params);
        catch
            q_fk = abenicsFK(theta, params.beta);
        end
    end
end
end