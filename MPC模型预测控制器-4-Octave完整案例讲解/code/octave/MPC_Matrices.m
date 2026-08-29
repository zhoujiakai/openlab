function [E, H] = MPC_Matrices(A, B, Q, R, F, N)
%MPC_MATRICES 构造无约束线性 MPC 的二次规划矩阵。
%
% 预测模型：
%   X = M*x(k) + C*U
%
% 代价函数中与 U 有关的部分：
%   U'*H*U + 2*x(k)'*E'*U

    n = size(A, 1);
    p = size(B, 2);

    assert(size(A, 2) == n, 'A 必须是方阵。');
    assert(size(B, 1) == n, 'A 与 B 的行数必须一致。');
    assert(isequal(size(Q), [n, n]), 'Q 的尺寸必须是 n x n。');
    assert(isequal(size(F), [n, n]), 'F 的尺寸必须是 n x n。');
    assert(isequal(size(R), [p, p]), 'R 的尺寸必须是 p x p。');
    assert(isscalar(N) && N >= 1 && N == fix(N), ...
           'N 必须是正整数。');

    % M 将当前状态映射到 x(k),...,x(k+N)。
    M = zeros((N + 1) * n, n);
    M(1:n, :) = eye(n);

    % C 将未来控制序列映射到预测状态序列。
    C = zeros((N + 1) * n, N * p);

    A_power = eye(n);
    for i = 1:N
        rows = i * n + (1:n);
        previous_rows = (i - 1) * n + (1:n);

        C(rows, :) = [A_power * B, C(previous_rows, 1:end-p)];
        A_power = A * A_power;
        M(rows, :) = A_power;
    end

    Q_bar = blkdiag(kron(eye(N), Q), F);
    R_bar = kron(eye(N), R);

    E = C' * Q_bar * M;
    H = C' * Q_bar * C + R_bar;

    % 消除浮点运算造成的微小非对称，避免部分求解器拒绝 H。
    H = (H + H') / 2;
end
