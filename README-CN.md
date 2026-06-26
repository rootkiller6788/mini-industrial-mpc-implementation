# Mini Industrial MPC Implementation（迷你工业MPC实施）

一套**从零构建、零外部依赖的 C 语言实现**，覆盖工业模型预测控制（MPC）算法与工程实践。每个子模块对标 MIT 和 Stanford 研究生课程，从子空间辨识、约束处理到 AspenTech DMC3 集成和操作员培训接口，打通控制理论与工业部署的全链路。

## 子模块

| 子模块 | 主题 | 对标课程 |
|--------|------|----------|
| [mini-mpc-constraint-handling-priority](mini-mpc-constraint-handling-priority/) | 约束优先级管理、可行性分析、约束松弛策略、QP 优化 | MIT 6.251J, Stanford AA203 |
| [mini-mpc-disturbance-feedforward](mini-mpc-disturbance-feedforward/) | 可测/不可测扰动前馈、Kalman 观测器、扰动建模 | MIT 6.241J, MIT 10.492 |
| [mini-mpc-ill-conditioned-process](mini-mpc-ill-conditioned-process/) | 条件数估计、SVD（Golub-Reinsch）、预处理、Tikhonov 正则化、RGA 灵敏度 | MIT 18.065, MIT 10.492 |
| [mini-mpc-industrial-software-aspentech](mini-mpc-industrial-software-aspentech/) | AspenTech DMC3 接口、NMPC、鲁棒 MPC、自适应 MPC、滚动时域估计（MHE） | MIT 10.492, Stanford AA203 |
| [mini-mpc-integrating-process-level](mini-mpc-integrating-process-level/) | DMC、GPC（CARIMA）、积分过程建模、液位约束、缓冲罐动态 | MIT 10.492, MIT 10.450 |
| [mini-mpc-operator-training-interface](mini-mpc-operator-training-interface/) | 操作员培训仿真器（OTS）、绩效评估、场景生成、操作指导 | MIT 16.842, MIT 2.154 |
| [mini-mpc-performance-monitoring-kpi](mini-mpc-performance-monitoring-kpi/) | Harris 性能指数、最小方差基准、KPI 监控、根因诊断 | MIT 10.492, Stanford EE364A |
| [mini-mpc-subspace-identification](mini-mpc-subspace-identification/) | N4SID、MOESP、CVA、分块 Hankel 矩阵、斜投影、模型验证 | Stanford EE364B, MIT 6.241J |

## 设计哲学

- **零外部依赖** — 纯 C 语言（C99/C11），仅使用 `libc` 和 `libm`
- **模块自包含** — 每个目录拥有独立的 `Makefile`、`include/`、`src/`、`examples/`、`tests/`、`docs/`
- **工业标准对标** — 每个模块的 `docs/` 中包含 ISA-88、ISA-95、IEC 61511、ISA-18.2 等工业标准参考
- **可运行演示** — DMC 控制器、GPC 控制器、AspenTech DMC3 接口、N4SID 辨识、OTS 场景、KPI 仪表盘

## 构建

每个模块独立构建。进入任意模块目录执行：

```bash
cd mini-mpc-constraint-handling-priority
make all    # 构建全部目标
make test   # 运行测试
```

需要 **GCC** 和 **GNU Make**。

## 项目结构

```
mini-industrial-mpc-implementation/
├── mini-mpc-constraint-handling-priority/  # 约束层级、可行性分析、松弛策略
├── mini-mpc-disturbance-feedforward/       # 含可测/不可测扰动补偿的 MPC
├── mini-mpc-ill-conditioned-process/       # SVD、预处理、Tikhonov、RGA 处理病态系统
├── mini-mpc-industrial-software-aspentech/ # AspenTech DMC3 接口、NMPC、鲁棒/自适应 MPC、MHE
├── mini-mpc-integrating-process-level/     # DMC、GPC、CARIMA 处理积分（非自衡）过程
├── mini-mpc-operator-training-interface/   # 操作员培训仿真器及绩效评估与操作指导
├── mini-mpc-performance-monitoring-kpi/    # Harris 指数、最小方差基准、KPI 诊断
├── mini-mpc-subspace-identification/       # N4SID、MOESP、CVA — 数据驱动的状态空间辨识
├── .gitignore                              # 构建产物与 IDE 忽略规则
├── README.md                               # 英文说明文档
└── README-CN.md                            # 中文说明文档（本文件）
```

## 许可证

MIT
