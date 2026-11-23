# 原理图生成签核报告 · `cramped_and_stretched_mirror_amp`

- **综合结论**：`PASS`（`signoff=false`，静态电气与图面几何门禁）
- **电气等价与连通性（LVS + 拆网预检）**：`PASS`
- **Virtuoso Bridge 精确几何预检**：`PASS`
- **可读性几何 DRC**：`PASS`

## 关键量化指标

| 指标 | 数值 | 门禁要求 |
|---|---:|---|
| 器件实例数 | `3` | 与输入网表一致 |
| 网络总数 | `5` | 内部网单连通分量 |
| 符号重叠数 (`symbolOverlapCount`) | `0` | `= 0` |
| 压字/文字重叠数 (`labelOverlapCount`) | `0` | `= 0` |
| 导线穿器件本体数 (`wireThroughBodyCount`) | `0` | `= 0` |
| 导线擦异网引脚数 (`wireOverForeignPinCount`) | `0` | `= 0` |
| 异网共线重叠数 (`collinearForeignWireOverlapCount`) | `0` | `= 0` |
| 歧义拐点/T点跨线数 (`ambiguousCrossingCount`) | `0` | `= 0` |
| 正交跨线交叉总数 (`wireCrossingCount`) | `0` | 越少越好（无连接圆点） |
| 总正交走线长度（格） | `146` | 紧凑无绕远 |

## 面向大模型的迭代建议 (`llmRefinementSuggestions`)

- 当前原理图零异网交叉、零压字、零穿模，可直接用于复核或导入目标库。

