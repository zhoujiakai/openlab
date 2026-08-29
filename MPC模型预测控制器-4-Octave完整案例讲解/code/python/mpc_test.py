"""MPC 模型预测控制完整案例的 Python 版本。"""

from __future__ import annotations

from pathlib import Path

import numpy as np

from mpc_matrices import mpc_matrices
from prediction import prediction


def simulate(steps: int = 100, horizon: int = 5) -> tuple[np.ndarray, np.ndarray]:
    """运行 MPC 闭环仿真，返回状态历史和输入历史。"""
    a = np.array([[1.0, 0.1], [-1.0, 2.0]])
    b = np.array([[0.2, 1.0], [0.5, 2.0]])

    q = np.array([[100.0, 0.0], [0.0, 1.0]])
    f = np.array([[100.0, 0.0], [0.0, 1.0]])
    r = np.array([[1.0, 0.0], [0.0, 0.1]])

    n = a.shape[0]
    p = b.shape[1]

    # steps 次状态更新会产生 steps+1 个状态点。
    states = np.zeros((n, steps + 1))
    inputs = np.zeros((p, steps))
    states[:, 0] = np.array([20.0, -20.0])

    e, h = mpc_matrices(a, b, q, r, f, horizon)

    for k in range(steps):
        inputs[:, k] = prediction(states[:, k], e, h, horizon, p)
        states[:, k + 1] = a @ states[:, k] + b @ inputs[:, k]

    return states, inputs


def save_results(states: np.ndarray, inputs: np.ndarray) -> None:
    """保存数值结果；若安装了 matplotlib，则同时保存曲线图。"""
    output_dir = Path(__file__).resolve().parent / "results"
    output_dir.mkdir(exist_ok=True)

    state_table = np.column_stack((np.arange(states.shape[1]), states.T))
    input_table = np.column_stack((np.arange(inputs.shape[1]), inputs.T))
    np.savetxt(
        output_dir / "states.csv",
        state_table,
        delimiter=",",
        header="k," + ",".join(f"x{i + 1}" for i in range(states.shape[0])),
        comments="",
    )
    np.savetxt(
        output_dir / "inputs.csv",
        input_table,
        delimiter=",",
        header="k," + ",".join(f"u{i + 1}" for i in range(inputs.shape[0])),
        comments="",
    )

    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("未安装 matplotlib：已保存 CSV，跳过绘图。")
        return

    figure, axes = plt.subplots(2, 1, figsize=(9, 7), constrained_layout=True)

    state_steps = np.arange(states.shape[1])
    for i, state in enumerate(states, start=1):
        axes[0].plot(state_steps, state, label=f"x{i}")
    axes[0].set(title="State trajectories", xlabel="k", ylabel="State")
    axes[0].grid(True)
    axes[0].legend()

    input_steps = np.arange(inputs.shape[1])
    for i, control_input in enumerate(inputs, start=1):
        axes[1].plot(input_steps, control_input, label=f"u{i}")
    axes[1].set(title="Control inputs", xlabel="k", ylabel="Input")
    axes[1].grid(True)
    axes[1].legend()

    figure.savefig(output_dir / "mpc_simulation.png", dpi=160)
    plt.close(figure)


def main() -> None:
    states, inputs = simulate()
    save_results(states, inputs)

    print("仿真完成")
    print(f"最终状态: {states[:, -1]}")
    print(f"各输入最大绝对值: {np.max(np.abs(inputs), axis=1)}")


if __name__ == "__main__":
    main()
