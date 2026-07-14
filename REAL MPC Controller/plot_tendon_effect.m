% run and plot tendon effect

for preload_values = [0, 0.0125, 0.025, 0.0375, 0.05]

    clear q_pred_result q_actual_result qpred qactual t   % avoid stale-variable confusion between runs
    
    params_abenics_coordinate;   % RELOAD pp (and params) from disk -- must run every time you edit the .m file
    
    fprintf('Using pp.tau_preload_mesh = [%.4g; %.4g; %.4g]\n\n', pp.tau_preload_mesh);  % sanity check before simulating
    
    out = sim('real_MPC_schema');
    q_pred_result   = out.q_pred;
    q_actual_result = out.q_actual;
    
    qpred   = squeeze(q_pred_result.Data)';    % Nx3 (squeeze removes the middle singleton dim, ' transposes to Nx3)
    qactual = squeeze(q_actual_result.Data)';  % Nx3
    
    t = q_pred_result.Time;   % Nx1, should match q_actual_result.Time
    if ~isequal(t, q_actual_result.Time)
        warning('q_pred and q_actual have different time vectors -- results below may be misleading.');
    end
    
    % plot
    figure(1);
    hold on;
    axis_idx = 1; % roll
    x = qpred(:, axis_idx);
    y = qactual(:, axis_idx);
    plot(x, y, 'r-'); hold on; plot(x, x, 'k--');
    xlabel('q\_pred'); ylabel('q\_actual');
    title(sprintf('Hysteresis loop, tau\\_preload\\_mesh = %.4g', pp.tau_preload_mesh(axis_idx)));
    loop_area = polyarea(x, y);
    fprintf('loop area: %.6g\n', loop_area);
    
    xlim([0.19 0.23]);
    ylim([0.19 0.23]);
    
    figure(2);
    hold on;
    plot(t, x - y);  % q_pred - q_actual, i.e. tracking error over time
    xlabel('time (s)'); ylabel('q\_pred - q\_actual (rad)');
    title('Backlash tracking error');
end