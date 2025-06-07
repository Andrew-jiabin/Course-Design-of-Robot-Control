function tau = adaptive_pid(qd, dqd, q, dq, e_int, Kp_base, Ki_base, Kd, e0, alpha, beta)
    % 自适应PID控制器

    % --- 误差计算 ---
    e = qd - q;
    de = dqd - dq;

    % --- 增益调整 ---
    is_large_error = abs(e) > e0;
    is_small_error = ~is_large_error;

    Kp = Kp_base .* (1 + alpha * is_large_error);
    Ki = Ki_base .* (1 + beta  * is_small_error);
    tau_ff=[0; 0; 0];
    % --- PID 控制项 ---
    tau_pid = Kp .* e + Ki .* e_int + Kd .* de;


 
    % --- 总控制输出 ---
    tau = tau_pid + tau_ff;  % 全部为 3×1 向量 
end
