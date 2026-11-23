# 验证记录（VALIDATION · 0.2.1）

本次验证均为**纯本地静态与数值回归测试**，未启动任何电路仿真器，未连接远程/虚拟机 SSH 会话（`simulations_run=0`、`signoff=false`）。

## 一、单元测试与已知解 / 反例覆盖（26 项全通过）

在本目录执行：

```powershell
python -B -m unittest discover -s tests -v
```

| 测试模块 | 验证内容 | 类型 | 结果 |
| :--- | :--- | :--- | :---: |
| `test_parser_and_pdk.py` | SKY130 原语、三端多晶硅电阻、MIM 电容、Spectre 语法解析，以及 PMOS/NMOS 极性与水平桥接精确变换 | 契约与极性验证 | PASS |
| `test_placer_and_qp.py` | **已知解 1**：`1` 格与 `11` 格净空同时撑开/收紧至 `4` 格；**已知解 2**：非对称推挤下差分对与尾流管严格整数中心对称 $x_L + x_R = 2 x_{\text{tail}}$；**已知解 3**：组内紧凑与组间留白分层 | 解析已知解验证 | PASS |
| `test_router_and_nudging.py` | **已知解 4**：四线初值 `[100, 103, 105, 130]` 走廊分轨推开精确收敛至 `[100, 110, 120, 130]`；**已知解 5**：同网共干、异网分轨；**已知解 6**：窄走廊溢出量 `overflow = 10.0` 检测 | 解析已知解验证 | PASS |
| `test_auditor_and_bridge.py` | 本机 `virtuoso-bridge-lite` 精确几何 `afterViolationCount == 0` 离线联调；LVS 错网、内部网断路（`SPLIT_INTERNAL_NET`）、异网共线/歧义拐点、压字草图反例拦截 | Bridge 联调 + 反例拦截 | PASS |
| `test_v02_features.py` | **v0.2 新增 9 项测试**：`.param` 代数表达式求值（`{48*a2_tail}`、`9.1meg`）；CDAC 电容阵列严格同高与等距步长 + 蛇形电阻串；局部保等价翻转消交叉搜索；`WireLabelStub` 短桩贴标与单页拖拽编辑器导出；Xschem `.sym` 只读符号提取已知解；多页层次化展平还原 LVS 正例与篡改端口反例拦截；162 实例 `OPA316` 11 页全流程编译与展平 LVS 回归；上下自适应母线、禁止端口绕背及边缘端口顺序重排与直线射线对齐验证；**竖向支路同列对齐、子页全实线无孤立短桩及宽图画幅均衡折叠验证** | v0.2 演进能力验证 | PASS |
| `test_end_to_end_cli.py` | CLI 全子命令端到端回归 | CLI 回归 | PASS |

---

## 二、五组 SKY130 基准电路实测结果（`examples/demo_outputs/`）

| 基准案例 | 规模与结构特点 | 电气 / 展平 LVS / Bridge 预检 | 硬性可读性 DRC 违例 | 关键结果摘要 |
| :--- | :--- | :---: | :---: | :--- |
| `01_two_stage_ota` | 12 实例 · 两级米勒调零运放 | PASS | `0` | 全图异网正交交叉 `0` |
| `02_folded_cascode_ota` | 12 实例 · 四层折叠共源共栅 OTA | PASS | `0` | 四层串叠对齐，仅 1 处对偶折叠跨线 |
| `03_draft_refined` | 3 实例 · 过挤重叠 + 过松草图整理 | 整理前 FAIL → **整理后 PASS** | `9` → **`0`** | 自动消除重叠压字并收紧空白 |
| `04_cdac_and_ladder` | 12 实例 · 6 单元二进制电容阵列 + 6 级蛇形电阻梯 | PASS | `0` | 电容排严格同行等距步长，电阻串蛇形折叠 |
| `05_rcopa316_hierarchy` | **162 实例 / 82 网络 · 工业级轨到轨 Class-AB 运放（含 `.param` 公式、过驱恢复与双向短路限流）** | **展平还原 LVS 100% PASS** | **全部 11 页均为 `0`** | 自动保护原子簇拆分为 **10 张功能子页 + 1 张 `opamp_core` 顶层框图**，生成单文件可交互浏览的 `index.html` 原理图册 |
