%% Compare mesh tendon preload values
% Run this file from the folder containing the model and parameter script.

% clear; clc;
clf;

model_name = 'tendons_simulink';
preload_values = [0, 0.0125, 0.025, 0.0375, 0.05]; % N*m
axis_idx = 1;                                        % 1=roll, 2=pitch, 3=yaw

n_cases = numel(preload_values);
loop_area = zeros(n_cases,1);
rms_error = zeros(n_cases,1);
peak_error = zeros(n_cases,1);
mean_error = zeros(n_cases,1);

colors = lines(n_cases);
figure(1); clf; hold on; grid on;
figure(2); clf; hold on; grid on;

for case_idx = 1:n_cases
    preload = preload_values(case_idx);

    % Reload all structs, then actually apply this sweep value.
    params_abenics_coordinate;
    pp.tau_preload_mesh = preload * ones(3,1);
    meshContact.tau_preload_mesh = pp.tau_preload_mesh;
    meshContact.Ts = pp.Ts_plant;

    fprintf('\nUsing pp.tau_preload_mesh = [%.4g; %.4g; %.4g] N*m\n', ...
        pp.tau_preload_mesh);

    out = sim(model_name);
    q_pred_result = out.q_pred;
    q_actual_result = out.q_actual;

    qpred = localNx3(q_pred_result.Data);
    qactual = localNx3(q_actual_result.Data);
    t_pred = q_pred_result.Time(:);
    t_actual = q_actual_result.Time(:);

    % Put q_actual on the q_pred time grid if Simulink logged them differently.
    if ~isequal(t_pred, t_actual)
        qactual = interp1(t_actual, qactual, t_pred, 'linear', 'extrap');
    end
    t = t_pred;

    x = qpred(:,axis_idx);
    y = qactual(:,axis_idx);
    e = x - y;

    % Ignore the first 20% so initialization does not dominate comparisons.
    keep = t >= t(1) + 0.20*(t(end)-t(1));
    x_eval = x(keep);
    y_eval = y(keep);
    e_eval = e(keep);

    rms_error(case_idx) = sqrt(mean(e_eval.^2));
    peak_error(case_idx) = max(abs(e_eval));
    mean_error(case_idx) = mean(e_eval);

    % Closed-curve line integral. This is a descriptive hysteresis measure;
    % RMS and peak error below are the primary tracking metrics.
    xc = [x_eval; x_eval(1)];
    yc = [y_eval; y_eval(1)];
    loop_area(case_idx) = 0.5*abs(sum( ...
        xc(1:end-1).*yc(2:end) - xc(2:end).*yc(1:end-1)));

    figure(1);
    plot(x_eval, y_eval, 'Color', colors(case_idx,:), ...
        'DisplayName', sprintf('%.4g N m', preload));

    figure(2);
    plot(t, rad2deg(e), 'Color', colors(case_idx,:), ...
        'DisplayName', sprintf('%.4g N m', preload));
end

figure(1);
plot(xlim, xlim, 'k--', 'DisplayName', 'q_{actual}=q_{pred}');
xlabel('q_{pred} (rad)');
ylabel('q_{actual} (rad)');
title(sprintf('Mesh hysteresis comparison, axis %d', axis_idx));
legend('Location','best'); axis equal;

figure(2);
xlabel('Time (s)');
ylabel('q_{pred}-q_{actual} (deg)');
title(sprintf('Mesh tracking error, axis %d', axis_idx));
legend('Location','best');

results = table(preload_values(:), loop_area, rad2deg(rms_error), ...
    rad2deg(peak_error), rad2deg(mean_error), ...
    'VariableNames', {'Preload_Nm','LoopArea_rad2','RMS_Error_deg', ...
    'Peak_Error_deg','Mean_Error_deg'});

disp(results);

figure(3); clf;
tiledlayout(1,2);
nexttile;
plot(preload_values, rad2deg(rms_error), 'o-', 'LineWidth', 1.5);
grid on; xlabel('Preload (N m)'); ylabel('RMS error (deg)');
nexttile;
plot(preload_values, rad2deg(peak_error), 'o-', 'LineWidth', 1.5);
grid on; xlabel('Preload (N m)'); ylabel('Peak error (deg)');

function x = localNx3(data)
% Normalize common To Workspace layouts to N-by-3 without a blind transpose.
    x = squeeze(data);
    if size(x,2) == 3
        return;
    elseif size(x,1) == 3
        x = x.';
    else
        error('Expected a logged 3-axis signal; received size %s.', mat2str(size(data)));
    end
end
