function tau = pid_controller(qd, dqd, q, dq, e_int, Kp, Ki, Kd)
% pid_controller - PID 控制器函数
% 输入：
%   qd, dqd - 期望角度与角速度
%   q, dq   - 实际角度与角速度
%   e_int   - 积分项累积
%   Kp, Ki, Kd - PID增益
% 输出：
%   tau - 控制力矩输出

e = qd - q;
de = dqd - dq;

tau = Kp*e + Ki*e_int + Kd*de;
end
