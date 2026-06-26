# mini-mpc-constraint-handling-priority

**MPC Constraint Handling & Priority Management** — 工业模型预测控制的约束处理与优先级管理系统。

## Module Status: COMPLETE ✅

- **L1-L6**: Complete (全部核心定义、概念、结构、法则、算法、经典问题已实现)
- **L7**: Partial (4/7 industrial vendors: AspenTech DMC3, Honeywell RMPCT, Shell SMOC, ABB Predict & Control)
- **L8**: Partial (5/8 advanced topics: explicit MPC, constraint funnels, lexicographic MPC, conditioning, KKT slack verification)
- **L9**: Partial (Documented in knowledge graph, not fully implemented)

**准入条件达标**: include/ + src/ = **3,223 行** ≥ 3,000 ✅  
**make all 编译**: 零错误 ✅ (1 benign strncpy warning)  
**make test**: 49/49 通过 ✅  
**凑行数扫描**: 0 matches ✅  
**无 TODO/FIXME/stub/placeholder**: ✅

---

## 九层知识覆盖摘要

| Level | 名称 | 覆盖度 | 核心条目 |
|-------|------|--------|---------|
| **L1** | Definitions | ✅ Complete | 10 enum/struct types, 5+ struct definitions |
| **L2** | Core Concepts | ✅ Complete | Hard/soft classification, slack variables, Farkas lemma, exact penalty |
| **L3** | Engineering Structures | ✅ Complete | Priority indexing, active sets, QP formulation, vendor configs |
| **L4** | Engineering Laws | ✅ Complete | KKT conditions, Farkas lemma, Hoffman bound, complementary slackness |
| **L5** | Algorithms | ✅ Complete | Active-set QP, sequential relaxation, IIS detection, lexicographic MPC |
| **L6** | Canonical Problems | ✅ Complete | Input saturation, output prioritization, feasibility recovery, desaturation |
| **L7** | Industrial Applications | ⚡ Partial | AspenTech DMC3, Honeywell RMPCT, Shell SMOC, ABB Predict & Control |
| **L8** | Advanced Topics | ⚡ Partial | Explicit MPC, constraint funnels, conditioning, KKT verification |
| **L9** | Industry Frontiers | 📋 Partial | IT/OT convergence, autonomous operations (文档化) |

**评分**: L1-L6 Complete × 2 = 12, L7-L9 Partial × 1 = 3, **Total = 15/18** (COMPLETE)

---

## 核心定义

### 约束类型

| 约束类别 | 含义 | 示例 |
|---------|------|------|
| **HARD_INPUT** | 执行器物理限幅 | 阀门开度 [0%, 100%] |
| **HARD_RATE** | 变化率限幅 | 升温速率 ≤ 5°C/min |
| **HARD_OUTPUT** | 安全/质量硬限 | 反应器压力 < 50 bar |
| **SOFT_OUTPUT** | 经济/质量软限 | 产品纯度 > 95% (可临时违反) |
| **COUPLING** | 多变量耦合约束 | MV1 + MV2 ≤ 总进料上限 |

### 优先级体系

| 优先级 | 级别 | 可放松? | 工业场景 |
|--------|------|---------|---------|
| **CRITICAL (0)** | 安全/物理限幅 | ❌ 永不可放松 | 安全阀设定, 执行器限幅 |
| **HIGH (1)** | 工艺稳定性 | ⚠️ 最后手段 | 关键产品质量, 催化剂保护 |
| **MEDIUM (2)** | 经济优化 | ✅ 按需放松 | 产品收率, 能耗优化 |
| **LOW (3)** | 操作偏好 | ✅ 首批放松 | 操作员设定, 次要指标 |
| **MONITOR (4)** | 仅监控 | ✅ 不参与QP | 报警限, 趋势监控 |

---

## 核心定理

| 定理 | 公式 | 验证 |
|------|------|------|
| KKT最优性条件 | Hx* + f - A'λ* = 0, λ_i·(A_i x* - b_i) = 0 | `mpc_qp_verify_kkt()` ✅ |
| Farkas引理 (不可行性证明) | ∃ y ≥ 0: y'A = 0, y'b < 0 ⇔ 无可行解 | `mpc_feasibility_farkas_certificate()` ✅ |
| Fletcher精确罚函数 | ρ > |λ*| ⇒ s = 0 在最优解处 | `mpc_relaxation_set_exact_penalty()` ✅ |
| Hoffman误差界 | dist(x, F) ≤ κ·‖max(0, Ax-b)‖ | `mpc_feasibility_hoffman_bound()` ✅ |
| 互补松弛条件 | s > 0 ⇒ μ = 0, λ = ρ₁ + 2ρ₂s | `mpc_relaxation_check_kkt_slacks()` ✅ |
| 优先级传递性 | p₁ < p₂ ∧ p₂ < p₃ ⇒ p₁ < p₃ | `priority_transitive` (Lean) ✅ |
| 优先级反对称性 | p₁ < p₂ ⇒ ¬(p₂ < p₁) | `priority_antisymmetric` (Lean) ✅ |

---

## 核心算法

| 算法 | 文件 | 知识点 |
|------|------|--------|
| 原始Active-Set QP求解器 | `mpc_constraint_optimization.c` | Nocedal & Wright Ch.16 |
| 优先级排序 (快速排序) | `mpc_constraint_priority.c` | O(n log n) 稳定排序 |
| 顺序优先级放松 | `mpc_constraint_relaxation.c` | AspenTech DMC3风格 |
| IIS识别 (删除过滤器) | `mpc_constraint_feasibility.c` | Chinneck & Dravnieks 1991 |
| 字典序MPC | `mpc_constraint_industrial.c` | Kerrigan & Maciejowski 2002 |
| 输入饱和检测与管理 | `mpc_constraint_industrial.c` | 抗饱和MPC |
| 输出优先级排序 | `mpc_constraint_industrial.c` | 多CV约束冲突解决 |
| 罚函数权重自整定 | `mpc_constraint_relaxation.c` | Lagrange乘子估计 |
| 显式MPC临界域计算 | `mpc_constraint_optimization.c` | Bemporad et al. 2002 |
| 约束漏斗管理 | `mpc_constraint_industrial.c` | Honeywell RMPCT概念 |

---

## 经典问题

| 问题 | 示例文件 | 描述 |
|------|---------|------|
| 输入饱和管理 | `example_input_saturation.c` | 精馏塔回流饱和, 再沸器补偿 |
| 输出优先级冲突 | `example_output_prioritization.c` | FCC装置4CV约束优先级排序 |
| 约束放松恢复可行性 | `example_constraint_relaxation.c` | 反应器干扰下的逐级放松 |

---

## 九校课程映射

| 学校 | 课程 | 对应章节 |
|------|------|---------|
| **MIT** | 6.302 Feedback Systems | Ch.7: Constraints in Control |
| **Stanford** | ENGR205 Process Control | Ch.8: MPC Implementation |
| **Berkeley** | ME233 Advanced Control | Module 5: Constrained Optimization |
| **CMU** | 24-677 Adv Ctrl Systems | Module 6: Industrial MPC |
| **Georgia Tech** | ECE 6550 Nonlinear Control | Ch.9: Constraint Handling |
| **Purdue** | ME 575 Industrial Control | Ch.7: Process Constraints |
| **RWTH Aachen** | Industrial Control Systems | PLC/SCADA Constraints |
| **清华** | 过程控制工程 | 第6章: 约束预测控制 |
| **ISA/IEC** | ISA-88/95, IEC 61131-3 | Constraint Management |

---

## 文件清单

```
mini-mpc-constraint-handling-priority/
├── Makefile                              # make all / make test / make count / make audit
├── README.md                             # 本文件 (知识覆盖报告)
├── include/
│   ├── mpc_constraint_defs.h             # 核心类型定义 + 全部API声明 (247+行)
│   ├── mpc_constraint_priority.h         # 优先级管理接口 (49行)
│   ├── mpc_constraint_relaxation.h       # 软约束放松接口 (48行)
│   ├── mpc_constraint_optimization.h     # QP优化接口 (53行)
│   └── mpc_constraint_feasibility.h       # 可行性分析接口 (41行)
├── src/
│   ├── mpc_constraint_defs.c             # 核心实现 (331行)
│   ├── mpc_constraint_priority.c         # 优先级管理实现 (350行)
│   ├── mpc_constraint_relaxation.c       # 放松算法实现 (405行)
│   ├── mpc_constraint_optimization.c     # QP求解器实现 (558行)
│   ├── mpc_constraint_feasibility.c       # 可行性分析实现 (494行)
│   ├── mpc_constraint_industrial.c       # 工业应用实现 (647行)
│   └── mpc_constraint_formal.lean        # Lean 4 形式化 (350行, 不计入行数)
├── tests/
│   └── test_mpc_constraints.c            # 49项断言测试
├── examples/
│   ├── example_input_saturation.c        # 输入饱和管理示例
│   ├── example_output_prioritization.c   # 输出优先级排序示例
│   └── example_constraint_relaxation.c   # 约束放松恢复示例
├── docs/
│   ├── knowledge-graph.md                # 九层知识覆盖表
│   ├── coverage-report.md               # 覆盖评估报告
│   ├── gap-report.md                    # 缺失知识点列表
│   ├── course-alignment.md              # 九校课程映射
│   └── course-tree.md                   # 前置依赖树
├── demos/
└── benches/
```

---

## 构建与测试

```bash
make all          # 编译库 + 测试 + 示例
make test         # 运行49项测试
make examples     # 编译所有示例
make count        # 统计代码行数
make audit        # 凑行数扫描
make clean        # 清理
```

**编译要求**: GCC (C11), GNU Make, 零外部依赖 (仅 libc + libm).

---

## 参考教材

1. Rawlings, Mayne & Diehl (2017), *Model Predictive Control: Theory, Computation, and Design*, 2nd ed., Nob Hill.
2. Nocedal & Wright (2006), *Numerical Optimization*, 2nd ed., Springer.
3. Maciejowski (2002), *Predictive Control with Constraints*, Prentice Hall.
4. Camacho & Bordons (2007), *Model Predictive Control*, 2nd ed., Springer.
5. Qin & Badgwell (2003), "A survey of industrial model predictive control technology", *Control Engineering Practice*.
6. Bemporad, Morari, Dua & Pistikopoulos (2002), "The explicit linear quadratic regulator for constrained systems", *Automatica*.
7. Chinneck (2008), *Feasibility and Infeasibility in Optimization*, Springer.
8. Fletcher (1987), *Practical Methods of Optimization*, 2nd ed., Wiley.
9. Kerrigan & Maciejowski (2002), "Designing model predictive controllers with prioritised constraints and objectives".
10. de Oliveira & Biegler (1994), "Constraint handling and stability properties of model-predictive control", *Automatica*.
