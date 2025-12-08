# 基于 Acados 的差速机器人 MPC 控制实现指南

本文档详细介绍了如何在当前工程中集成基于 `acados` 的模型预测控制（MPC）。

## 1. 数学建模 (Mathematical Modeling)

针对您的差速驱动机器人（Differential Drive Robot），我们使用标准的运动学模型。

### 1.1 状态量与控制量
- **状态向量 (State Vector)** $x$:
  $$x = [p_x, p_y, \theta]^\top$$
  其中 $(p_x, p_y)$ 为机器人在世界坐标系下的位置，$\theta$ 为航向角。

- **控制输入 (Control Input)** $u$:
  $$u = [v, \omega]^\top$$
  其中 $v$ 为线速度，$\omega$ 为角速度。

### 1.2 连续时间动力学方程 (ODE)
$$
\dot{x} = f(x, u) = \begin{bmatrix} \dot{p}_x \\ \dot{p}_y \\ \dot{\theta} \end{bmatrix} = \begin{bmatrix} v \cos(\theta) \\ v \sin(\theta) \\ \omega \end{bmatrix}
$$

## 2. MPC 问题构建 (Formulation)

我们需要在每个控制周期求解一个最优控制问题（OCP）。

### 2.1 代价函数 (Cost Function)
采用非线性最小二乘形式（Nonlinear Least Squares）：

$$
J = \int_{0}^{T} \left( \| h(x,u) - y_{ref} \|_W^2 \right) dt + \| h_e(x(T)) - y_{ref,e} \|_{W_e}^2
$$

- **运行代价 (Stage Cost)**: 惩罚当前状态/输入与参考轨迹的偏差。
  - $h(x,u) = [p_x, p_y, \theta, v, \omega]^\top$
  - $y_{ref}$ 为参考轨迹点。
  - $W$ 为权重矩阵 (对角阵)，例如 $W = \text{diag}(Q_x, Q_y, Q_\theta, R_v, R_\omega)$。
- **终端代价 (Terminal Cost)**: 惩罚预测时域末端状态的偏差。
  - $h_e(x) = [p_x, p_y, \theta]^\top$
  - $W_e$ 为终端权重矩阵。

### 2.2 约束条件 (Constraints)
- **输入约束 (Input Bounds)**:
  $$
  v_{\min} \leq v \leq v_{\max} \\
  \omega_{\min} \leq \omega \leq \omega_{\max}
  $$
  根据 `nav2_params.yaml`，建议设置：$v \in [-0.3, 0.5]$, $\omega \in [-0.5, 0.5]$。

## 3. 工程实现方案 (Implementation Plan)

### 3.1 目录结构建议
在 `src` 下新建一个功能包 `acados_mpc_ros`：
```
src/acados_mpc_ros/
├── CMakeLists.txt
├── package.xml
├── scripts/
│   └── generate_c_code.py   #用于生成 acados C 代码的 Python 脚本
├── src/
│   └── mpc_node.cpp         # ROS 2 节点 (C++)
└── launch/
    └── mpc.launch.py
```

### 3.2 步骤一：安装 Acados
1. 克隆 acados 仓库并编译（需要 CMake 和 C 编译器）。
2. 安装 Python 接口：`pip install -e acados/interfaces/acados_template`。
3. 设置环境变量 `LD_LIBRARY_PATH` 和 `ACADOS_SOURCE_DIR`。

### 3.3 步骤二：编写生成脚本 (`generate_c_code.py`)
使用 `acados_template` 定义模型和 OCP。

```python
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
import casadi as ca
import numpy as np

def export_ode_model():
    model = AcadosModel()
    model.name = 'diff_drive'
    
    # 定义符号变量
    x = ca.SX.sym('x')
    y = ca.SX.sym('y')
    theta = ca.SX.sym('theta')
    sym_x = ca.vertcat(x, y, theta)
    
    v = ca.SX.sym('v')
    omega = ca.SX.sym('omega')
    sym_u = ca.vertcat(v, omega)
    
    # 动力学
    x_dot = ca.vertcat(v * ca.cos(theta), v * ca.sin(theta), omega)
    
    model.f_expl_expr = x_dot
    model.x = sym_x
    model.u = sym_u
    return model

def main():
    # 创建 OCP 对象
    ocp = AcadosOcp()
    model = export_ode_model()
    ocp.model = model
    
    # 设置预测时域
    Tf = 2.0  # 预测 2 秒
    N = 20    # 分 20 段
    ocp.dims.N = N
    ocp.solver_options.tf = Tf
    
    # 设置代价函数类型
    ocp.cost.cost_type = 'NONLINEAR_LS'
    ocp.cost.cost_type_e = 'NONLINEAR_LS'
    
    # 权重矩阵 (需根据实际调试调整)
    Q = np.diag([10.0, 10.0, 5.0]) # x, y, theta
    R = np.diag([0.1, 0.1])        # v, omega
    ocp.cost.W = ca.block_cat(Q, R)
    ocp.cost.W_e = Q
    
    # 设置约束
    ocp.constraints.lbu = np.array([-0.3, -0.5])
    ocp.constraints.ubu = np.array([ 0.5,  0.5])
    ocp.constraints.idxbu = np.array([0, 1])
    
    # 设置求解器选项
    ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
    ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
    ocp.solver_options.integrator_type = 'ERK'
    ocp.solver_options.nlp_solver_type = 'SQP_RTI' # 实时迭代，速度快
    
    # 生成代码
    AcadosOcpSolver(ocp, json_file = 'acados_ocp.json')

if __name__ == '__main__':
    main()
```

### 3.4 步骤三：ROS 2 节点开发 (`mpc_node.cpp`)
该节点需要完成以下逻辑：

1.  **初始化**：加载生成的 acados solver。
2.  **订阅 `/Odometry`**：获取机器人当前状态 $(x_0, y_0, \theta_0)$，更新 OCP 的初始条件约束 `ocp.constraints.lbx_0` 和 `ubx_0`。
3.  **订阅 `/plan` (nav_msgs/Path)**：
    - 将全局路径裁剪并插值，生成未来 $N$ 个时刻的参考轨迹 $(x_{ref}, y_{ref}, \theta_{ref})$。
    - 更新 OCP 的参考值 `y_ref`。
4.  **求解**：调用 `acados_solve()`。
5.  **发布 `/cmd_vel`**：取优化结果的第一个控制输入 $u_0 = [v^*, \omega^*]$ 发布。

## 4. 结合当前工程的建议

1.  **依赖管理**：由于 acados 是外部库，建议将其作为 `third_party` 子模块放入您的工作空间，或者在系统级安装。
2.  **接口兼容**：
    - 您目前的 `nav2_params.yaml` 中配置了 `FollowPath` 插件。如果您希望完全替换 Nav2 的控制器，可以编写一个符合 `nav2_core::Controller` 接口的插件（`nav2_acados_controller`），这样可以直接在 Nav2 框架内使用，无需重写路径订阅逻辑。
    - **推荐方案**：编写 Nav2 插件。参考 `nav2_regulated_pure_pursuit_controller` 的源码，将其核心计算部分替换为 acados solver 的调用。

这样您既能利用 Nav2 的代价地图（Costmap）进行碰撞检测（在 MPC 代价中加入障碍物项，或在回调中检查），又能享受 acados 的高性能优化。
