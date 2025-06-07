% main_test.m
clear; clc;

%% 控制器参数
Kp_base = [200; 200; 200];
Ki_base = [50; 50; 50];
Kd      = [20; 20; 20];
e0 = 0.001; alpha = 2; beta = 1;

%% 仿真设置
Ts = 0.001;
T_end = 3;
t = 0:Ts:T_end;
n = length(t);

% 期望轨迹：阶跃
qd = repmat([0.5; 0.5; 0.5], 1, n);  % 期望角度
dqd = zeros(3, n);  % 静止目标

% 初始化变量
q_a = zeros(3, n); dq_a = zeros(3, n); e_int_a = zeros(3,1);
q_b = zeros(3, n); dq_b = zeros(3, n); e_int_b = zeros(3,1);
q_c = zeros(3, n); dq_c = zeros(3, n); e_int_c = zeros(3,1);
q_d = zeros(3, n); dq_d = zeros(3, n); e_int_d = zeros(3,1);
tau_a = zeros(3, n); tau_b = zeros(3, n);tau_c = zeros(3, n);tau_d = zeros(3, n);

%% 仿真循环
for k = 1:n-1
    %% -------- 方案A：固定PID --------
    e_a = qd(:,k) - q_a(:,k);
    de_a = dqd(:,k) - dq_a(:,k);
    e_int_a = e_int_a + e_a * Ts;
    tau_a(:,k) = Kp_base .* e_a + Ki_base .* e_int_a + Kd .* de_a;

    ddq_a = phantom_dynamics(q_a(:,k), dq_a(:,k), tau_a(:,k));
    dq_a(:,k+1) = dq_a(:,k) + ddq_a * Ts;
    q_a(:,k+1)  = q_a(:,k)  + dq_a(:,k) * Ts;


    %% -------- 方案B：PID + 前馈 --------
    e_b = qd(:,k) - q_b(:,k);
    de_b = dqd(:,k) - dq_b(:,k);
    e_int_b = e_int_b + e_b * Ts;
    tau_b(:,k) = pid_ff(qd(:,k), dqd(:,k), q_b(:,k), dq_b(:,k), ...
        e_int_b, Kp_base, Ki_base, Kd, e0, alpha, beta);

    ddq_b = phantom_dynamics(q_b(:,k), dq_b(:,k), tau_b(:,k));
    dq_b(:,k+1) = dq_b(:,k) + ddq_b * Ts;
    q_b(:,k+1)  = q_b(:,k)  + dq_b(:,k) * Ts;

    %% -------- 方案C：自适应 + PID --------
    e_c = qd(:,k) - q_c(:,k);
    de_c = dqd(:,k) - dq_c(:,k);
    e_int_c = e_int_c + e_c * Ts;
    tau_c(:,k) = adaptive_pid(qd(:,k), dqd(:,k), q_c(:,k), dq_c(:,k), ...
        e_int_c, Kp_base, Ki_base, Kd, e0, alpha, beta);

    ddq_c = phantom_dynamics(q_c(:,k), dq_c(:,k), tau_c(:,k));
    dq_c(:,k+1) = dq_c(:,k) + ddq_c * Ts;
    q_c(:,k+1)  = q_c(:,k)  + dq_c(:,k) * Ts;


    %% -------- 方案D：PID + 自适应 + 前馈 --------
    e_d = qd(:,k) - q_d(:,k);
    de_d = dqd(:,k) - dq_d(:,k);
    e_int_d = e_int_d + e_d * Ts;
    tau_d(:,k) = adaptive_pid_ff(qd(:,k), dqd(:,k), q_d(:,k), dq_d(:,k), ...
        e_int_d, Kp_base, Ki_base, Kd, e0, alpha, beta);

    ddq_d = phantom_dynamics(q_d(:,k), dq_d(:,k), tau_d(:,k));
    dq_d(:,k+1) = dq_d(:,k) + ddq_d * Ts;
    q_d(:,k+1)  = q_d(:,k)  + dq_d(:,k) * Ts;
end

%% 结果分析 
guanjie=3; %设置分析的关节
q1_desired = qd(guanjie,:);
q1_a = q_a(guanjie,:);
q1_b = q_b(guanjie,:);
q1_c = q_c(guanjie,:);
q1_d = q_d(guanjie,:);
% 绘图
figure;
plot(t, q1_desired, 'y--', t, q1_a, 'b', t, q1_b, 'k',t, q1_c, 'g', t, q1_d, 'r');
legend('期望轨迹','方案A: 固定PID','方案B: PID + 前馈','方案C: PID + 自适应','方案D: PID + 自适应 + 前馈');
xlabel('时间 (s)');
ylabel('关节1角度 (rad)');
title('关节1控制对比');

% 误差指标
e_a_all = abs(q1_desired - q1_a);
e_b_all = abs(q1_desired - q1_b);
e_c_all = abs(q1_desired - q1_c);
e_d_all = abs(q1_desired - q1_d);
ITAE_a = sum(e_a_all.*t) ;
ITAE_b = sum(e_b_all.*t) ;
ITAE_c = sum(e_c_all.*t) ;
ITAE_d = sum(e_d_all.*t) ;

fprintf('ITAE 方案A: %.4f\n', IAE_a);
[rise_time, peak_time, overshoot, settling_time] =analyzeStepResponse(t,q1_a,q1_desired)
fprintf('ITAE 方案B: %.4f\n', IAE_b);
[rise_time, peak_time, overshoot, settling_time] =analyzeStepResponse(t,q1_b,q1_desired)
fprintf('ITAE 方案C: %.4f\n', IAE_c);
[rise_time, peak_time, overshoot, settling_time] =analyzeStepResponse(t,q1_c,q1_desired)
fprintf('ITAE 方案D: %.4f\n', IAE_d);
[rise_time, peak_time, overshoot, settling_time] =analyzeStepResponse(t,q1_d,q1_desired)