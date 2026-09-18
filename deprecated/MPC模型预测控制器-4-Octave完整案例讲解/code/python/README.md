# Python 版本

这是同目录 Octave/MATLAB MPC 案例的 Python 翻译版，算法和参数保持一致。

## 文件

- `mpc_test.py`：主程序、闭环仿真、结果保存和可选绘图；
- `mpc_matrices.py`：构造预测模型及二次型代价矩阵；
- `prediction.py`：求解未来控制序列并返回第一步输入。

## 依赖与运行

必须安装 NumPy：

```shell
python3 -m pip install numpy
python3 mpc_test.py
```

Matplotlib 是可选依赖。安装后程序会额外生成曲线图：

```shell
python3 -m pip install matplotlib
```

数值结果保存在运行目录旁的 `results/` 文件夹中。由于本例没有状态或输入约束，Python 版直接求解二次函数的一阶最优条件，不依赖 SciPy 的优化模块。
