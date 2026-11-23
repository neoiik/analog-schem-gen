# 验证记录（VALIDATION · 0.1.0）

本次验证均为**纯本地静态与数值回归测试**，未启动任何电路仿真器，未连接远程/虚拟机 SSH 会话（`simulations_run=0`、`signoff=false`）。

## 一、单元测试与已知解 / 反例覆盖（17 项全通过）

在本目录执行：

```powershell
python -B -m unittest discover -s tests -v
```

| 测试模块 | 验证内容 | 类型 | 结果 |
| :--- | :--- | :--- | :--- |
| `test_parser_and_pdk.py` | `sky130_fd_pr__nfet_01v8`、`pfet_01v8_hvt`、三端 `res_high_po_0p35`、`cap_mim_m3_1` 及 Spectre 语法解析；验证 PMOS `R0` 源上漏下、NMOS `R0` 漏上源下、水平桥接电容左 `P` 右 `M` 精确变换 | 契约与极性验证 | PASS |
| `test_placer_and_qp.py` | **已知解 1**：三同宽器件初始净空 `1` 格与 `11` 格、目标净空 `4` 格，验证 QP 同时将左侧过窄间隙撑开为 `4` 格、右侧过大空隙收紧为 `4` 格；**已知解 2**：偏置支路非对称推挤下，差分对与尾流管引脚严格保持整数中心对称 $x_{L} + x_{R} = 2 x_{\text{tail}}$；**已知解 3**：组内紧凑与组间留白分层 | 解析已知解验证 | PASS |
| `test_router_and_nudging.py` | **已知解 4**：走廊内四条异网平行水平线初值 `[100, 103, 105, 130]`、最小轨距 `10`、走廊 `[100, 130]`，验证分轨推开精确收敛至 `[100, 110, 120, 130]`；**已知解 5**：验证“同网共干、异网分轨”；**已知解 6**：5 线入窄走廊精确报告溢出量 `overflow = 10.0` | 解析已知解验证 | PASS |
| `test_auditor_and_bridge.py` | **真实 Bridge 离线联调**：调用本机 `virtuoso-bridge-lite` 的 `load_schematic_manifest`、`load_process_map` 与 `plan_manifest_circuit`，验证 `afterViolationCount == 0`；**反例拦截**：篡改端子网络（`LVS_TERMINAL_NET_MISMATCH`）、删减内部网导线制造断路（`SPLIT_INTERNAL_NET`）、注入异网共线/歧义拐点（`COLLINEAR_WIRE_OVERLAP` / `AMBIGUOUS_JUNCTION_CROSSING`）、压字草图（`SYMBOL_OVERLAP` / `LABEL_SYMBOL_OVERLAP`）均被准确拦截 | Bridge 联调 + 反例拦截 | PASS |
| `test_end_to_end_cli.py` | `analyze → compile → audit`、`auto`、`probe-pdk` 全子命令端到端验证 | CLI 回归 | PASS |

---

## 二、三组 SKY130 代表性基准实测指标（`examples/demo_outputs/`）

通过 `python -B schem_gen.py demo --out examples/demo_outputs` 生成：

| 基准案例 | 规模 | 电气与 Bridge 预检 | 符号/压字/穿模/共线/歧义拐点 | 异网正交交叉数 (`wireCrossingCount`) | 总线长（格） |
| :--- | :--- | :---: | :---: | :---: | :---: |
| `01_two_stage_ota`（SKY130 两级米勒调零运放） | 12 实例 / 10 网络 | PASS | 全部为 `0` | **`0`** | `714` |
| `02_folded_cascode_ota`（SKY130 四层折叠共源共栅 OTA） | 12 实例 / 15 网络 | PASS | 全部为 `0` | **`1`**（仅差分折叠对偶跨接 1 处正交跨线） | `566` |
| `03_draft_refined`（局部重叠压字 + 局部拉太开的缺陷草图整理） | 3 实例 / 5 网络 | 整理前 FAIL → **整理后 PASS** | 整理前 `9` 处违例 → **整理后全部为 `0`** | `1` → **`0`** | `191` → **`156`** |

---

## 三、已知验证边界（实事求是声明）

1. **Virtuoso 实机导入边界**：本轮已在纯 Python 离线模式下通过本机 `virtuoso-bridge-lite` 的精确几何求解器（`plan_manifest_circuit`），尚未在运行态 Virtuoso 中对实装 SKY130 OA 库执行 GUI 写入与原生 `si` 网表回读；首次接入实装 SKY130 OA 库前须运行 `probe-pdk --live` 核对真实符号 `pinOffsets`。
2. **适用电路规模**：当前直接布局器面向常见模拟单元/子模块级网表（约 2～80 个器件，如运放核心、偏置产生器、比较器、带隙核心、LDO 误差放大器）。上百器件的完整顶层芯片建议先按 `.subckt` 模块分层生成子图与 Symbol，再拼装顶层连接图。
