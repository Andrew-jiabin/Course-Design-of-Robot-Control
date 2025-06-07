% frequency_response_phantom.m

model = 'simulation';  % 模型名称
frequencies = logspace(-1, 2, 20);  % 频率范围 0.1Hz 到 100Hz
amplitude = 0.1;
Ts = 0.001;
response_amp = zeros(size(frequencies));
response_phase = zeros(size(frequencies));

load_system(model);

for i = 1:length(frequencies)
    freq = frequencies(i);
    simTime = 5 / freq;

    % 设置每个正弦源频率
    omega = 2*pi*freq;
    set_param([model '/Sine Wave'], 'Amplitude', num2str(amplitude), 'Frequency', num2str(omega));
    set_param([model '/Sine Wave1'], 'Amplitude', num2str(amplitude), 'Frequency', num2str(omega));
    set_param([model '/Sine Wave2'], 'Amplitude', num2str(amplitude), 'Frequency', num2str(omega));

    % 设置仿真时间
    set_param(model, 'StopTime', num2str(simTime));

    % 运行模型
    simOut = sim(model);

    % 提取输出
    t = simOut.tout;
    y = simOut.logsout.getElement('a1').Values.Data;  % 假设输出为 logsout 内的变量
    u = simOut.logsout.getElement('ref').Values.Data;

    % 只分析第1个关节（a1）
    y1 = y(:,1);

    % 稳态分析
    t0 = round(length(t)*0.5);
    y_steady = y1(t0:end);
    u_steady = u(t0:end);

    % 幅频
    Ay = (max(y_steady) - min(y_steady)) / 2;
    Au = (max(u_steady) - min(u_steady)) / 2;
    response_amp(i) = Ay / Au;

    % 相频
    [~, lag] = max(xcorr(y_steady, u_steady));
    delay = lag - length(y_steady);
    response_phase(i) = -delay * 2 * pi * freq * Ts;
end

% 绘图
figure;
subplot(2,1,1);
semilogx(frequencies, 20*log10(response_amp));
xlabel('Frequency (Hz)');
ylabel('Gain (dB)');
title('幅频响应');


% figure;
% semilogx(frequencies,  data_record_pid_ori(2,:) ,'b', ...
%          frequencies,  data_record_pid_ff(2,:) ,'k', ...
%          frequencies,  data_record_pid_ad(2,:) ,'g', ...
%          frequencies,  data_record_pid_ff_ad(2,:) ,'r');
% xlabel('Frequency (Hz)');
% ylabel('Gain (dB)');
% title('幅频响应');
% 




