function u_k = Prediction(x_k, E, H, N, p)
%PREDICTION 求解未来控制序列，并只返回第一步控制输入。

    assert(iscolumn(x_k), 'x_k 必须是列向量。');
    assert(isequal(size(H), [N * p, N * p]), ...
           'H 的尺寸必须是 (N*p) x (N*p)。');
    assert(isequal(size(E), [N * p, numel(x_k)]), ...
           'E 的尺寸必须是 (N*p) x n。');

    f = E * x_k;
    H = (H + H') / 2;

    % quadprog 最小化 0.5*U'*H*U + f'*U。
    % 这与原代价函数相差一个正的常数倍，不改变最优解。
    if exist('quadprog', 'file') == 2
        [U_k, ~, exitflag] = quadprog(H, f);
        if isempty(U_k) || any(~isfinite(U_k)) || exitflag < 0
            error('二次规划未能得到有效控制序列，exitflag = %d。', ...
                  exitflag);
        end
    else
        % 当前示例没有输入/状态约束，因此最优性条件为 H*U + f = 0。
        % 该回退路径让代码在未安装 optim 包时仍可运行。
        if rcond(H) < 1e-12
            U_k = -pinv(H) * f;
        else
            U_k = -(H \ f);
        end
    end

    u_k = U_k(1:p);
end
