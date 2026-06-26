# mini-mpc-performance-monitoring-kpi

**MPC Performance Monitoring & KPI** — 工业模型预测控制器的性能监测与关键绩效指标体系。

## Module Status: COMPLETE ✅

- **L1-L6**: Complete (全部核心定义、概念、结构、法则、算法、经典问题已实现)
- **L7**: Complete (13 industrial vendor/standard applications)
- **L8**: Complete (12 advanced topics: Bayesian CP, subspace, Pareto, time-varying, forecasting, Kalman, Monte Carlo, multirate, collinearity, nonlinearity detection, Holt smoothing, tuning aggressiveness)
- **L9**: Partial (Autonomous health, digital twin sync, IT/OT convergence, cybersecurity readiness)
- **Lean 4**: Complete (8 formal theorems proven)

**准入条件达标**: include/ + src/ = **3,070 行** ✅  
**测试**: make test → **134 tests, 0 failed** ✅  
**make 编译**: -Wall -Wextra -Wpedantic 零警告通过 ✅  
**凑行数扫描**: 0 matches ✅  
**无 TODO/FIXME/stub/placeholder**: ✅

---

## 九层知识覆盖摘要

| Level | 名称 | 覆盖度 | 核心条目 |
|-------|------|--------|---------|
| **L1** | Definitions | ✅ Complete | 8 enum types, 14 struct definitions, 7 string converters |
| **L2** | Core Concepts | ✅ Complete | Harris index, mismatch detection, controller utilization, constraint monitoring |
| **L3** | Engineering Structures | ✅ Complete | Ring buffer O(1) stats, EWMA, CUSUM, dashboard aggregation |
| **L4** | Engineering Laws | ✅ Complete | Harris formula, EWMA recurrence, CUSUM rule, Ljung-Box test |
| **L5** | Algorithms | ✅ Complete | Yule-Walker AR, Levinson-Durbin, autocorrelation, cross-correlation |
| **L6** | Canonical Problems | ✅ Complete | Distillation Harris, FCC baselining, petrochemical dashboard |
| **L7** | Industrial Applications | ✅ Complete | AspenTech, Honeywell, Yokogawa, Shell, ISO 50001 |
| **L8** | Advanced Topics | ✅ Complete | Bayesian CP, subspace validation, Pareto, time-varying, forecasting |
| **L9** | Industry Frontiers | 📋 Partial | Autonomous health (API), digital twin (documented) |

**评分**: L1-L8 Complete × 2 = 16, L9 Partial × 1 = 1, **Total = 17/18** (COMPLETE)

---

## 核心定义

| 结构体 | 说明 | 知识点 |
|--------|------|--------|
| `mpc_kpi_value_t` | 单一KPI测量值 (当前值/基线/EWMA/趋势/等级) | L1 KPI生命周期 |
| `mpc_kpi_ringbuffer_t` | 环形缓冲区 (O(1)均值/方差/偏度/峰度) | L3 在线统计 |
| `mpc_kpi_ewma_t` | EWMA滤波器 (均值+方差+波动率) | L3 指数平滑 |
| `mpc_kpi_cusum_t` | CUSUM累加器 (持久偏移检测) | L4 序贯检验 |
| `mpc_kpi_harris_t` | Harris指数结果 (最小方差基准) | L4 控制评估 |
| `mpc_kpi_mismatch_t` | 模型-装置失配诊断 | L5 交叉相关 |
| `mpc_kpi_dashboard_t` | 多KPI仪表盘 (健康评分聚合) | L6 综合评估 |
| `mpc_kpi_baseline_t` | 基线估计 (均值/中位数/MAD/正态性) | L6 基线建立 |

## 核心定理

| 定理 | 公式 | 实现 |
|------|------|------|
| Harris指数 | η = 1 - σ²_MV/σ²_y | `mpc_kpi_compute_harris_index()` |
| Yule-Walker方程 | Rφ = r (Toeplitz系统) | `mpc_kpi_yule_walker_ar()` |
| Ljung-Box检验 | Q = n(n+2)Σr²_k/(n-k) ~ χ²(h) | `mpc_kpi_ljung_box_test()` |
| CUSUM决策规则 | S⁺ > h ⟹ alarm | `mpc_kpi_cusum_update()` |
| EWMA方差 | σ²_k = (1-λ)σ²_{k-1}+λ(x_k-μ_{k-1})² | `mpc_kpi_ewma_update()` |
| 加权健康评分 | H = Σw_i·s_i / Σw_i | `mpc_kpi_weighted_health_score()` |

## 核心算法

| 算法 | 文件 | 复杂度 |
|------|------|--------|
| Harris指数 (AR建模) | `mpc_kpi_metrics.c` | O(n·p²) Levinson-Durbin |
| 自相关函数 | `mpc_kpi_metrics.c` | O(n·K) |
| Ljung-Box检验 | `mpc_kpi_metrics.c` | O(K) |
| 模型失配检测 | `mpc_kpi_metrics.c` | O(n·K) 交叉相关 |
| 线性趋势检测 | `mpc_kpi_metrics.c` | O(n) |
| 异常值检测 (IQR) | `mpc_kpi_metrics.c` | O(n log n) 排序 |
| 振荡诊断 | `mpc_kpi_diagnosis.c` | O(n log n) 自相关 |
| 贝叶斯变点检测 | `mpc_kpi_industrial.c` | O(n²) |
| Pareto前沿分析 | `mpc_kpi_industrial.c` | O(n²) |

## 经典问题

| 问题 | 示例文件 | 描述 |
|------|---------|------|
| 精馏塔Harris评估 | `example_harris_index.c` | AR(2)过程最小方差基准 |
| FCC反应器基线监测 | `example_kpi_baselining.c` | CUSUM检测性能退化 |
| 石化厂KPI仪表盘 | `example_kpi_dashboard.c` | 12-KPI多维度综合评估 |

## 工业应用 (L7)

| 供应商 | 功能 | 函数 |
|--------|------|------|
| AspenTech AspenWatch | 健康指数+服务因子 | `mpc_kpi_aspentech_aspenwatch_kpi()` |
| Honeywell Profit Sensor | 利润影响+模型质量 | `mpc_kpi_honeywell_profit_sensor()` |
| Yokogawa MD | 过程能力+鲁棒性指数 | `mpc_kpi_yokogawa_md_diagnostic()` |
| Shell MV Monitor | 整体健康+瓶颈指数 | `mpc_kpi_shell_mv_monitor()` |
| ISO 50001 | 能源绩效指标(EnPI) | `mpc_kpi_iso50001_enpi()` |
| CO2减排 | 碳排放估算 | `mpc_kpi_co2_reduction_estimate()` |

## 九校课程映射

| 学校 | 课程 | 主题 |
|------|------|------|
| **MIT** | 6.302 Feedback Systems | 最小方差基准, Harris指数 |
| **Stanford** | ENGR205 Process Control | MPC健康监测, 经济KPI |
| **Berkeley** | ME233 Advanced Control | 统计过程监测, CUSUM |
| **CMU** | 24-677 Adv Ctrl Systems | 模型失配检测, 子空间方法 |
| **Purdue** | ME 575 Industrial Control | 工业KPI仪表盘, ISO 50001 |
| **RWTH Aachen** | Industrial Control | AspenWatch, Profit Sensor |
| **清华** | 过程控制工程 | 炼油FCC, 精馏塔基准 |
| **ISA/IEC** | ISA-106 / IEC 62682 | 报警管理, KPI等级分类 |

---

## 文件结构

```
├── Makefile              # make all → 编译+测试+示例
├── README.md                    # 本文件
├── include/ (4 headers)         # 664 lines total
│   ├── mpc_kpi_defs.h           # 核心类型定义 (19 structs, 8 enums)
│   ├── mpc_kpi_metrics.h        # KPI算法API (Harris, ACF, mismatch, statistics)
│   ├── mpc_kpi_monitoring.h     # 监测引擎API (baseline, monitoring, environmental)
│   └── mpc_kpi_diagnosis.h      # 诊断+工业厂商API (diagnosis, vendors, L8 advanced)
├── src/ (5 C + 1 Lean)          # 2406 lines total
│   ├── mpc_kpi_defs.c           # 数据结构实现 (L1-L7 核心类型)
│   ├── mpc_kpi_metrics.c        # 算法实现 (L4-L5: 17 algorithms)
│   ├── mpc_kpi_monitoring.c     # 监测引擎 (L6-L7: 12 monitoring functions)
│   ├── mpc_kpi_diagnosis.c      # 诊断引擎 (L5-L6: 17 diagnosis functions)
│   ├── mpc_kpi_industrial.c     # 工业厂商 (L7-L9: 13 vendor + 6 advanced)
│   └── mpc_kpi_formal.lean      # Lean 4形式化 (8 theorems proven)
├── tests/                       # 测试套件 (134 tests)
│   └── test_mpc_kpi.c
├── examples/                    # 3个端到端示例
│   ├── example_harris_index.c   # 精馏塔3场景Harris评估
│   ├── example_kpi_baselining.c # FCC反应器基线+降级检测
│   └── example_kpi_dashboard.c  # 石化厂12-KPI综合仪表盘
├── demos/                       # 交互式演示
│   └── kpi_dashboard_demo.c     # 5场景实时监测演示
├── benches/                     # 性能基准
│   └── bench_kpi.c              # 6项基准测试
└── docs/                        # 5个知识文档
    ├── knowledge-graph.md
    ├── coverage-report.md
    ├── gap-report.md
    ├── course-alignment.md
    └── course-tree.md
```
