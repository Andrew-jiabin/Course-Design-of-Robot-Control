function [rise_time, peak_time, overshoot, settling_time] = analyzeStepResponse(t, y, y_ref)
    % 上升时间：从 10% 到 90% 参考值的时间
    ref_value = y_ref(1); % 阶跃参考值（假设恒定）
    idx_10 = find(y > 0.1*ref_value, 1, 'first');
    idx_90 = find(y > 0.9*ref_value, 1, 'first');
    rise_time = t(idx_90) - t(idx_10);
    
    % 峰值时间：达到峰值的时间
    [peak_val, peak_idx] = max(y);
    peak_time = t(peak_idx);
    
    % 超调量：(峰值 - 稳态值)/稳态值 * 100% ，稳态值近似取最后部分平均值
    steady_state_idx = round(length(t)*0.9):length(t);
    steady_state_val = mean(y(steady_state_idx));
    overshoot = (peak_val - steady_state_val) / steady_state_val * 100;
    
    % 调整时间：进入并保持在 2% 误差带内的时间
    error_band = 0.02 * ref_value; % 2% 误差带
    settling_idx = find(abs(y - steady_state_val) < error_band, 1, 'first');
    settling_time = t(settling_idx);
end