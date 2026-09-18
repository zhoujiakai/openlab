"""构造无约束线性 MPC 的预测矩阵和二次型代价矩阵。"""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray


FloatArray = NDArray[np.float64]


def mpc_matrices(
    a: FloatArray,
    b: FloatArray,
    q: FloatArray,
    r: FloatArray,
    f: FloatArray,
    horizon: int,
) -> tuple[FloatArray, FloatArray]:
    """返回代价函数所需的 E、H 矩阵。

    预测模型写成 ``X = M @ x(k) + C @ U``，与控制序列有关的
    代价为 ``U.T @ H @ U + 2 * x(k).T @ E.T @ U``。
    """
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    q = np.asarray(q, dtype=float)
    r = np.asarray(r, dtype=float)
    f = np.asarray(f, dtype=float)

    if a.ndim != 2 or a.shape[0] != a.shape[1]:
        raise ValueError("A 必须是方阵")

    n = a.shape[0]
    if b.ndim != 2 or b.shape[0] != n:
        raise ValueError("A 与 B 的行数必须一致")

    p = b.shape[1]
    if q.shape != (n, n):
        raise ValueError("Q 的尺寸必须是 n x n")
    if f.shape != (n, n):
        raise ValueError("F 的尺寸必须是 n x n")
    if r.shape != (p, p):
        raise ValueError("R 的尺寸必须是 p x p")
    if not isinstance(horizon, int) or horizon < 1:
        raise ValueError("预测区间 horizon 必须是正整数")

    # 当前状态到 x(k), ..., x(k+N) 的映射。
    m = np.zeros(((horizon + 1) * n, n))
    m[:n, :] = np.eye(n)

    # 未来控制序列到预测状态序列的映射。
    c = np.zeros(((horizon + 1) * n, horizon * p))
    a_power = np.eye(n)

    for i in range(1, horizon + 1):
        rows = slice(i * n, (i + 1) * n)
        previous_rows = slice((i - 1) * n, i * n)

        c[rows, :p] = a_power @ b
        if horizon > 1:
            c[rows, p:] = c[previous_rows, :-p]

        a_power = a @ a_power
        m[rows, :] = a_power

    q_bar = np.zeros(((horizon + 1) * n, (horizon + 1) * n))
    for i in range(horizon):
        rows = slice(i * n, (i + 1) * n)
        q_bar[rows, rows] = q
    q_bar[horizon * n :, horizon * n :] = f

    r_bar = np.kron(np.eye(horizon), r)

    e = c.T @ q_bar @ m
    h = c.T @ q_bar @ c + r_bar

    # 消除浮点运算产生的微小非对称。
    h = (h + h.T) / 2.0
    return e, h
