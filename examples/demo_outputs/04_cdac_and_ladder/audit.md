# 原理图生成签核报告 · `sky130_cdac_and_ladder`

- **综合结论**：`PASS`（`signoff=false`，静态电气与图面几何门禁）
- **电气等价与连通性（LVS + 拆网预检）**：`PASS`
- **Virtuoso Bridge 精确几何预检**：`PASS`
- **可读性几何 DRC**：`PASS`

## 关键量化指标

| 指标 | 数值 | 门禁要求 |
|---|---:|---|
| 器件实例数 | `12` | 与输入网表一致 |
| 网络总数 | `16` | 内部网单连通分量 |
| 符号重叠数 (`symbolOverlapCount`) | `0` | `= 0` |
| 压字/文字重叠数 (`labelOverlapCount`) | `0` | `= 0` |
| 导线穿器件本体数 (`wireThroughBodyCount`) | `0` | `= 0` |
| 导线擦异网引脚数 (`wireOverForeignPinCount`) | `0` | `= 0` |
| 异网共线重叠数 (`collinearForeignWireOverlapCount`) | `0` | `= 0` |
| 歧义拐点/T点跨线数 (`ambiguousCrossingCount`) | `0` | `= 0` |
| 正交跨线交叉总数 (`wireCrossingCount`) | `6` | 越少越好（无连接圆点） |
| 总正交走线长度（格） | `989` | 紧凑无绕远 |

## 面向大模型的迭代建议 (`llmRefinementSuggestions`)

- 网络 'VDAC' 当前产生 3 次跨线交叉：若为偏置或控制网，可在 intent.json 中设 `netPolicies.VDAC = "perimeter_bus"`，或调整相关功能组的 `stage_order` / 管子 `gate_left`/`gate_right` 朝向以消除交叉。
- 网络 'VREFL' 当前产生 3 次跨线交叉：若为偏置或控制网，可在 intent.json 中设 `netPolicies.VREFL = "perimeter_bus"`，或调整相关功能组的 `stage_order` / 管子 `gate_left`/`gate_right` 朝向以消除交叉。
- 网络 'VREFH' 当前产生 3 次跨线交叉：若为偏置或控制网，可在 intent.json 中设 `netPolicies.VREFH = "perimeter_bus"`，或调整相关功能组的 `stage_order` / 管子 `gate_left`/`gate_right` 朝向以消除交叉。

