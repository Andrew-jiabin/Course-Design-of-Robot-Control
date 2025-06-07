function ddq = phantom_dynamics(q, dq, tau)
% phantom_dynamics - 简化的三自由度 Phantom Omni 机器人动力学模型
% 输入：
%   q   - 关节角度 [3x1]
%   dq  - 关节角速度 [3x1]
%   tau - 驱动力矩 [3x1]
% 输出：
%   ddq - 关节角加速度 [3x1]

% 简化惯性矩阵 M(q)
M = diag([0.5, 0.4, 0.3]);  % kg*m^2

% 科氏/离心项（简化为与速度成正比）
C = diag([0.05, 0.05, 0.05]) * dq;

% 重力项
g = 9.8;
G = [0.2*g*cos(q(1));
     0.1*g*cos(q(2));
     0.05*g*cos(q(3))];

% 求解加速度
ddq = M \ (tau - C - G);
end
