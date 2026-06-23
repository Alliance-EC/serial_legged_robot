# serial_legged_robot · 五连杆轮腿机器人建模、控制与仿真

面向 RoboMaster 平衡步兵（双轮腿）机器人，搭建从动力学建模、虚拟模型控制（VMC）、卡尔曼状态观测到仿真→C++ 代码生成的 MATLAB/Simulink 开发链路。

## 致谢与贡献边界

本仓库在已有战队框架基础上改进与扩展，部分模块参考了公开方案，特此说明：

- **VMC 腿部姿态推导**参考了知乎公开方案 [基于VMC的轮腿机器人控制](https://zhuanlan.zhihu.com/p/563048952)（见 `small_leg_calc.m` 中标注，该文件名标 `(改)`）。
- 工程框架托管于战队组织 Alliance-EC 账号下，用于团队代码统一管理。

**本人（岑子涵 · 南京理工大学自动化）在此基础上独立完成的部分：**

- 系统动力学符号建模 → 线性化 → `c2d` 离散 → `dlqr` → 双腿长 `poly44` 增益调度拟合（`sys_calc.m`）；
- 带打滑检测的卡尔曼状态/速度观测器（`kalman_observer/`，打滑时动态抬高轮速测量方差）；
- IMU 加速度计速度观测实验（`IMU_position/`）；
- 仿真 → C++ 代码生成的部署链路（`function/codegen/`）。

> 关联学术工作：五连杆闭链双轮腿机器人运动控制与越障算法研究（本人第一作者，**拟投 IEEE ICRA 2027，in preparation，尚未提交**）。本仓为工程仿真与复现底座；论文核心的「最优杆长求解」算法见独立仓库 [LinkageLengthSolver](https://github.com/Alliance-EC/LinkageLengthSolver)。

## 仿真环境

- MATLAB R2023b（依赖 Symbolic Math Toolbox、Control System Toolbox、Parallel Computing Toolbox、MATLAB Coder）
- 首次运行 `startup.m` 会触发 `sys_calc`/`leg_calc` 的符号计算与拟合，耗时较长。

## 文件结构

```
sys_calc.m            系统参数计算（动力学建模 + dlqr + poly44 增益调度）
small_leg_calc.m      腿参数计算 / VMC（VMC 部分参考上述知乎方案）
lb_sfit.sfit          腿长拟合 (cftool)
leg_sfit.sfit         转动惯量拟合 (cftool)
leg_sim.slx           五连杆仿真
leg_sim_calc.m        从仿真中拟合腿部数据
sys_sim.slx           系统仿真
kalman_observer/      卡尔曼观测器（速度 / 全状态，含打滑检测）
IMU_position/         加速度计速度观测实验
function/codegen/     MATLAB Coder 生成的 C++ 函数（仿真→部署）
data/                 数据文件

# 以下为未完成 / 实验性分支，非主要工作：
sys_calc_whx.m / sys_sim_whx.slx   哈工程建模分支（未完成）
agentData*.mat / evaluatePolicy.m  早期 DDPG 试验（非本仓重点）
```
