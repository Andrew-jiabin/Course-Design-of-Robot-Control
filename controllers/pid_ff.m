function tau = pid_ff(qd, dqd, q, dq, e_int, Kp, Ki, Kd, e0, alpha, beta)
    % 自适应PID + 简化前馈补偿控制器

    % --- 误差计算 ---
    e = qd - q;
    de = dqd - dq;
 
 
    % --- PID 控制项 ---
    tau_pid = Kp .* e + Ki .* e_int + Kd .* de;

    % --- 简化动力学补偿 ---
    % 重力项 G(q)
    G = [0.2*9.8*cos(q(1));
         0.1*9.8*cos(q(2));
         0.05*9.8*cos(q(3))];

    % 科氏项近似 C(q,dq)*dq，这里我们人为简化为常数比例
    c_coeff = [0.05; 0.05; 0.05];
    C = c_coeff .* dq;  % 元素相乘，返回 3x1 向量 

    tau_ff = C + G;
 
    % --- 总控制输出 ---
    tau = tau_pid + tau_ff;  % 全部为 3×1 向量 
end
