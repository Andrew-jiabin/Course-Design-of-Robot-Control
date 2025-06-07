function phantom_omni_sfun(block)
% 三自由度Phantom Omni机器人控制S-Function
% 支持两种控制模式: 
%   flag_pid=1: 普通PID
%   flag_pid=0: 自适应PIDor前馈补偿（需要自行调整代码，无法通过参数直接设置）

setup(block);
function DoPostPropSetup(block)
    % 配置9个Dwork缓冲区
    block.NumDworks = 9;

    % 为每个状态定义存储空间
    state_names = {'q1','q2','q3', 'dq1','dq2','dq3', 'e_int1','e_int2','e_int3'};
    for i = 1:9
        block.Dwork(i).Name = state_names{i};
        block.Dwork(i).Dimensions = 1;
        block.Dwork(i).DatatypeID = 0; % 0=double
        block.Dwork(i).Complexity = 'Real';
    end
function setup(block)
% 基础设置
block.NumInputPorts  = 1;   % 期望关节角度 [q1_d, q2_d, q3_d]
block.NumOutputPorts = 1;   % 实际关节角度 [q1, q2, q3]
block.NumDialogPrms  = 11;  % [Ts, flag_pid, Kp, Ki, Kd, e0, alpha, beta, m1, m2, m3]

% 参数说明
% 1: Ts - 采样时间
% 2: flag_pid - 控制器标志 (1=普通PID, 0=自适应PID+前馈)
% 3-5: Kp, Ki, Kd - PID基础增益
% 6: e0 - 自适应误差阈值
% 7-8: alpha, beta - 自适应参数
% 9-11: m1,m2,m3 - 关节质量(用于重力补偿)

% 配置端口属性
block.SetPreCompInpPortInfoToDynamic;
block.SetPreCompOutPortInfoToDynamic;


block.InputPort(1).Dimensions = 3;
block.InputPort(1).DirectFeedthrough = false; % 避免代数环
block.OutputPort(1).Dimensions = 3;


% 设置采样时间
Ts = block.DialogPrm(1).Data;
block.SampleTimes = [Ts 0];

% 注册离散状态(9个: 3角度+3速度+3积分项)
block.NumContStates = 0;
block.NumDworks = 0;


% 注册更新和输出函数
block.RegBlockMethod('InitializeConditions', @InitializeConditions);
block.RegBlockMethod('Outputs', @Outputs);
block.RegBlockMethod('Update', @Update);
block.RegBlockMethod('SetInputPortSamplingMode',@SetInputPortSamplingMode);
block.RegBlockMethod('PostPropagationSetup', @DoPostPropSetup); % 新增
function SetInputPortSamplingMode(block, port, mode)
block.InputPort(port).SamplingMode = mode;

block.OutputPort(1).SamplingMode = mode;


function InitializeConditions(block)
% 初始化状态变量
for i = 1:9
    block.Dwork(i).Data = 0;
end

function Outputs(block)
% 输出当前关节角度
block.OutputPort(1).Data = [block.Dwork(1).Data,block.Dwork(2).Data,block.Dwork(3).Data;]; 

function Update(block)
% 获取参数

Ts       = block.DialogPrm(1).Data;
flag_pid = block.DialogPrm(2).Data;
Kp       = block.DialogPrm(3).Data;
Ki       = block.DialogPrm(4).Data;
Kd       = block.DialogPrm(5).Data;
e0       = block.DialogPrm(6).Data;
alpha    = block.DialogPrm(7).Data;
beta     = block.DialogPrm(8).Data;
m1       = block.DialogPrm(9).Data;
m2       = block.DialogPrm(10).Data;
m3       = block.DialogPrm(11).Data;
m        = [m1; m2; m3];


% 获取当前状态
q = [block.Dwork(1).Data;  % q1
     block.Dwork(2).Data;  % q2
     block.Dwork(3).Data]; % q3

dq = [block.Dwork(4).Data;  % dq1
      block.Dwork(5).Data;  % dq2
      block.Dwork(6).Data]; % dq3

e_int = [block.Dwork(7).Data;  % e_int1
         block.Dwork(8).Data;  % e_int2
         block.Dwork(9).Data]; % e_int3

% 获取期望角度
qd = block.InputPort(1).Data;
 

% 假设期望速度为零 (位置控制)
dqd = [0; 0; 0];

% 计算误差
e = block.InputPort(1).Data;
de = dqd - dq;

% 更新误差积分 (离散积分)
e_int_new = e_int + e * Ts;

% 选择控制器
if flag_pid == 1  % 普通PID
    tau = Kp.*e + Ki.*e_int_new + Kd.*de;
else % 自适应PIDor前馈（需要直接调整代码）
    % 自适应增益调整
    is_large_error = abs(e) > e0;
    is_small_error = ~is_large_error;

    Kp_adj = Kp .* (1 + alpha * is_large_error);
    Ki_adj = Ki .* (1 + beta * is_small_error);
    % Kp_adj = Kp .* (1 );
    % Ki_adj = Ki .* (1 );
    % PID部分
    tau_pid = Kp_adj.*e + Ki_adj.*e_int_new + Kd.*de;

    % 前馈补偿 (重力+速度阻尼)
    g = 9.8;
    G = [0.2*m(1)*g*cos(q(1));
         0.1*m(2)*g*cos(q(2));
         0.05*m(3)*g*cos(q(3))];

    C = [0.05*dq(1); 
         0.05*dq(2);
         0.05*dq(3)];
  
    % tau = tau_pid + G + C;
    tau = tau_pid ; 
end

% 计算动力学
ddq = phantom_dynamics(q, dq, tau);

% 状态更新 (欧拉积分)
dq_new = dq + ddq * Ts;
q_new = q + dq * Ts;  % 使用当前速度

% 保存新状态
block.Dwork(1).Data = q_new(1);  % q1
block.Dwork(2).Data = q_new(2);  % q2
block.Dwork(3).Data = q_new(3);  % q3
block.Dwork(4).Data = dq_new(1); % dq1
block.Dwork(5).Data = dq_new(2); % dq2
block.Dwork(6).Data = dq_new(3); % dq3
block.Dwork(7).Data = e_int_new(1); % e_int1
block.Dwork(8).Data = e_int_new(2); % e_int2
block.Dwork(9).Data = e_int_new(3); % e_int3

% 嵌入式动力学函数
function ddq = phantom_dynamics(q, dq, tau)
% 简化三自由度机器人动力学模型
M = diag([0.5, 0.4, 0.3]);  % 惯性矩阵
C = diag([0.05, 0.05, 0.05]) * dq;  % 科氏/离心项
g = 9.8;
G = [0.2*g*cos(q(1));
     0.1*g*cos(q(2));
     0.05*g*cos(q(3))];  % 重力项
ddq = M \ (tau - C - G);

