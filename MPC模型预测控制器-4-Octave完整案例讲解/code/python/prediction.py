"""求解无约束 MPC 控制序列。"""

from __future__ import annotations

import numpy as np
from numpy.typing import NDArray


FloatArray = NDArray[np.float64]


def prediction(
    x_k: FloatArray,
    e: FloatArray,
    h: FloatArray,
    horizon: int,
    input_count: int,
) -> FloatArray:
    """计算最优控制序列，并只返回当前时刻需要执行的第一步。

    当前案例没有状态或输入约束，所以二次规划的一阶最优条件为
    ``H @ U + E @ x_k = 0``，可直接用线性方程求解。
    """
    x_k = np.asarray(x_k, dtype=float).reshape(-1)
    e = np.asarray(e, dtype=float)
    h = np.asarray(h, dtype=float)

    expected = horizon * input_count
    if h.shape != (expected, expected):
        raise ValueError("H 的尺寸必须是 (N*p) x (N*p)")
    if e.shape != (expected, x_k.size):
        raise ValueError("E 的尺寸必须是 (N*p) x n")

    h = (h + h.T) / 2.0
    gradient = e @ x_k

    try:
        control_sequence = np.linalg.solve(h, -gradient)
    except np.linalg.LinAlgError:
        # 仅在 H 奇异或病态时使用最小二乘回退。
        control_sequence = np.linalg.lstsq(h, -gradient, rcond=None)[0]

    if not np.all(np.isfinite(control_sequence)):
        raise RuntimeError("未能得到有效的控制序列")

    return control_sequence[:input_count]
