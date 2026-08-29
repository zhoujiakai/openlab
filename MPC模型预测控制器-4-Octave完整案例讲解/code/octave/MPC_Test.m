%% MPC 模型预测控制示例（修复版）
% 来源：https://www.bilibili.com/opus/666629407425691664
% 兼容 GNU Octave 和 MATLAB。

clear;
close all;
clc;

% Octave 的 quadprog 位于 optim 包；MATLAB 不需要加载包。
if exist('OCTAVE_VERSION', 'builtin')
    try
        pkg load optim;
    catch
        warning(['未找到 Octave optim 包，将使用无约束二次规划的' ...
                 '线性方程回退解法。']);
    end
end

%% 第一步：定义离散状态空间模型 x(k+1) = A*x(k) + B*u(k)
A = [1, 0.1; -1, 2];
B = [0.2, 1; 0.5, 2];

n = size(A, 1);  % 状态数量
p = size(B, 2);  % 输入数量

assert(size(A, 2) == n, 'A 必须是方阵。');
assert(size(B, 1) == n, 'A 与 B 的行数必须一致。');

%% 第二步：定义 MPC 权重
Q = [100, 0; 0, 1];  % 预测区间内的状态权重
F = [100, 0; 0, 1];  % 终端状态权重
R = [1, 0; 0, 0.1];  % 控制输入权重

assert(isequal(size(Q), [n, n]), 'Q 的尺寸必须是 n x n。');
assert(isequal(size(F), [n, n]), 'F 的尺寸必须是 n x n。');
assert(isequal(size(R), [p, p]), 'R 的尺寸必须是 p x p。');

%% 第三步：设置仿真长度、初始状态和预测区间
k_steps = 100;
N = 5;

% k_steps 次状态更新会产生 k_steps+1 个状态点。
% 原帖只预分配了 k_steps 列，却在最后写入第 k_steps+1 列。
X_K = zeros(n, k_steps + 1);
U_K = zeros(p, k_steps);
X_K(:, 1) = [20; -20];

%% 第四步：构造二次规划矩阵
[E, H] = MPC_Matrices(A, B, Q, R, F, N);

%% 第五步：滚动优化并推进系统
for k = 1:k_steps
    U_K(:, k) = Prediction(X_K(:, k), E, H, N, p);
    X_K(:, k + 1) = A * X_K(:, k) + B * U_K(:, k);
end

%% 第六步：绘图
state_steps = 0:k_steps;
input_steps = 0:(k_steps - 1);
state_labels = arrayfun(@(i) sprintf('x%d', i), 1:n, ...
                            'UniformOutput', false);
input_labels = arrayfun(@(i) sprintf('u%d', i), 1:p, ...
                            'UniformOutput', false);

figure('Name', 'MPC simulation');

subplot(2, 1, 1);
plot(state_steps, X_K.', 'LineWidth', 1.2);
grid on;
xlabel('k');
ylabel('State');
title('State trajectories');
legend(state_labels, 'Location', 'best');

subplot(2, 1, 2);
plot(input_steps, U_K.', 'LineWidth', 1.2);
grid on;
xlabel('k');
ylabel('Input');
title('Control inputs');
legend(input_labels, 'Location', 'best');
