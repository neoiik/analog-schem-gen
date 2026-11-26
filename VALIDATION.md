# 验证记录（VALIDATION · 0.3.3）

本次验证均为**纯本地静态与数值回归测试**，未启动任何电路仿真器，未连接远程/虚拟机 SSH 会话（`simulations_run=0`、`signoff=false`）。

## 一、单元测试与已知解 / 反例覆盖（0.3.3 增补：65 项全通过；0.3.3 基线：62 项 / 0.3.2：56 项 / 0.3.1：47 项 / 0.2.3：43 项 / 0.2.2：42 项 / 0.2.1 基线：35/39 项历史记录保留）

在本目录执行：

```powershell
python -B -m unittest discover -s tests -v
```

| 测试模块 | 验证内容 | 类型 | 结果 |
| :--- | :--- | :--- | :---: |
| `test_parser_and_pdk.py` | SKY130 原语、三端多晶硅电阻、MIM 电容、Spectre 语法解析、PMOS/NMOS 极性与水平桥接精确变换；**2026-11-26 增补**：SPICE `.subckt ... params:` 端口剥离、行为级黑盒宏（`B`/`E`/`F`/`G`/`H`）识别、`VON` 输出端口方向判定与 `PdkRegistry.ensure_subckt_symbols()` 自动框符号注册 | 契约与极性验证 | PASS |
| `test_placer_and_qp.py` | **已知解 1**：`1` 格与 `11` 格净空同时撑开/收紧至 `4` 格；**已知解 2**：非对称推挤下差分对与尾流管严格整数中心对称 $x_L + x_R = 2 x_{\text{tail}}$；**已知解 3**：组内紧凑与组间留白分层；2D 短距张力场等距阵列松弛、引脚对向竖排、无极性 2 端被动器件 180° 自动翻转、串联链同列有序化、跨自定义电源共栅镜管对齐，以及**多轨穿行触发整阵均匀拉稀与差分折叠内/外 L 嵌套直穿**；**2026-11-26 增补**：水平接地/接电源侧引脚（如接地基极 PNP `C=BASE=VSS` 或栅接轨 MOS）防死循环与零违例出图 | 解析已知解 + 几何回归 | PASS |
| `test_router_and_nudging.py` | **已知解 4**：四线初值 `[100, 103, 105, 130]` 走廊分轨推开精确收敛至 `[100, 110, 120, 130]`；**已知解 5**：同网共干、异网分轨；**已知解 6**：窄走廊溢出量 `overflow = 10.0` 检测；顶部 `perimeter_bus` 单平直母线与输出端并联负载直贯主干（无阶梯/无 U 形凹坑）；**2026-11-26 增补**：横向受遮挡的局部内区电源符号不得跨列劫持远端模块电源引脚 | 解析已知解 + 走线回归 | PASS |
| `test_auditor_and_bridge.py` | 本机 `virtuoso-bridge-lite` 精确几何 `afterViolationCount == 0` 离线联调；LVS 错网、内部网断路（`SPLIT_INTERNAL_NET`）、异网共线/歧义拐点、压字草图反例拦截 | Bridge 联调 + 反例拦截 | PASS |
| `test_v02_features.py` | **v0.2/v0.3 新增 10 项测试**：`.param` 代数表达式求值（`{48*a2_tail}`、`9.1meg`）；CDAC 电容阵列严格同高与等距步长 + 蛇形电阻串；局部保等价翻转消交叉搜索；`WireLabelStub` 短桩贴标与单页拖拽编辑器导出；Xschem `.sym` 只读符号提取已知解；多页层次化展平还原 LVS 正例与篡改端口反例拦截；162 实例 `OPA316` 11 页全流程编译与展平 LVS 回归；上下自适应母线、禁止端口绕背及边缘端口顺序重排与直线射线对齐验证；竖向支路同列对齐、子页全实线无孤立短桩及宽图画幅均衡折叠验证；**非电源顶层端口单 Port 化 + 短距共享器件实体导线成簇直连 + 远端支路 WireLabel 飞线及断路反例拦截** | 演进能力 + 反例验证 | PASS |
| `test_topology_semantics.py` | **v0.2.2–0.2.3 第一/二期语义与几何规则升级（4 项已知解与反例测试）**：双叠共栅 `Super-MOS` 复合差分对归约（二极管侧在左）；`mos_cap`/`mos_active_res`/`source_follower` 角色分类与反例拦截；广义 `miller_bridge`（`horizontal_gate_down`/`R90` 横置串联有源电阻，2D 衬底短桩与 `virtuoso-bridge-lite` 完全一致，`M10` 在上、`M11–M13` 居中、`M14` 在下垂直夹心，`Vb6`/`Vb7` 居中抽头）与互补源随 `complementary_sf` 识别；`sandwich_mirror_core` 桥接电阻（`R1`）同列居下挂接于尾管 `M30` 下方，彻底消除 `sym_x` 对称放大引起的差分对肥胖（`M27→M29` 由 `60` 格收紧至 `22` 格，与 `M6→M8` 一致）；紧凑贴边电源/地轨与孤立冗余轨符号清理（零悬浮地符号）；41 管双通路运放零样本 `auto`（全实线 `crossings=19`、`wireLen=3229` / 线标版 `crossings=2`、`wireLen=1846`）全门禁与 Bridge 预检 PASS | 拓扑语义已知解 + 横置器件 + 反例验证 | PASS |
| `test_branch_abstraction_and_validator.py` | **v0.3.1 Coarse-to-Fine 语义提取与前置校验升级（4 项已知解与反例测试）**：1) 混合 BJT+MOS+R+C 匿名节点带隙基准（BGR）无损竖向支路折叠（保持 `M5-Q1-Q2-R1`、`Q9-M32-Q15-R7`、`Q10-M33-Q8-R8-R9`、`Q12-R10-R11` 完整，且中点钳位管 `Q11` 不劫持分压电阻链）、全局偏置树（`n_bn`/`n_bp`）剥离与反馈参考总线区分、跨器件耦合镜像误差放大器与 8:1 准对称带隙核心及反馈环（SCC）识别；2) 规整列阵列（`PMOS×2 × 4`、`NMOS×2 × 3`）去重与单汇总节点零误报反馈环；3) 前置语义校验器 `validate-modules` 反例拦截（漏器件、重器件、撕裂原子串联链、共享信号母线上零内连孤立错放支路、不兼容镜像骨架与缺核心标记）；4) 完整 Coarse→Meso→Fine CLI 闭环（`analyze` 产 `module_plan.draft.json` → `inspect --around/--module` → `validate-modules` → `plan-to-intent` → `compile --module-plan` 零手写朝向/纵层通过 LVS+Bridge+DRC，且反例错误方案在布局前即被拦截） | 支路折叠已知解 + 语义门禁反例 + CLI 端到端 | PASS |
| `test_end_to_end_cli.py` | CLI 全子命令端到端回归 | CLI 回归 | PASS |
| `test_architecture_consistency.py` | **v0.3.2 架构一致性门禁（9 项）**：全部基准网表（含 `examples/` 演示与合成 BGR/IDAC/双通路运放）的 `intent.draft.json` 与 `module_plan.draft.json` 均通过自家 `validate-modules`（HARD 撕裂 = 0），两份草案器件归属完全一致；自动分页不拆 HARD 段；未改动的模块草案经 `compile --module-plan` 交叉数不高于 `auto`；驼峰 `spacing` 键生效、未知键告警、`stageOrder` 别名；`refine` 与解析器共用电源轨规则；优化器不改动已声明镜像骨架的器件；`schematic_gen/` 中不含演示电路词汇；Pass 3 主干仅在退化分段时标注不适用。**v0.3.3 新增 6 项**：布局层（`placer`/`router`/`optimizer`）不导入语义层或审计层、全工具无跨模块私有导入；阈值与器件族可按工艺覆盖且代码中无 SKY130 类名硬编码；走线策略优先级（优化器不覆盖 `llm`/`skeleton` 来源，人工 intent 默认视为 `llm`）；`label_stub` 降级在审计中报告；分级简报摘要行数 < 完整版 60% 且 ≤ 220 行、所有支路编号仍可引用；差分对左右判据记录 `lr_tiebreak` 并在简报中声明命名仅作平局裁决 | 架构一致性 + 分层/规则门禁 | PASS |
| `test_placer_and_qp.py`（调整 1 项） | `test_co_gated_series_stack_pairing_priority_and_row_alignment` 删除“M5 因编号 4<5<6 必须夹在 M1-4 与 M6-9 之间”的命名断言，改为拓扑断言“从管 M5 位于二极管主管 M1-4 的从管侧”；同构从柱 M6-9 / M34-37 的取舍保留为编号平局裁决 | 去命名依赖 | PASS |

---

## 二、五组 SKY130 基准电路实测结果（`examples/demo_outputs/`）

| 基准案例 | 规模与结构特点 | 电气 / 展平 LVS / Bridge 预检 | 硬性可读性 DRC 违例 | 关键结果摘要 |
| :--- | :--- | :---: | :---: | :--- |
| `01_two_stage_ota` | 12 实例 · 两级米勒调零运放 | PASS | `0` | 全图异网正交交叉 `0` |
| `02_folded_cascode_ota` | 12 实例 · 四层折叠共源共栅 OTA | PASS | `0` | 四层串叠对齐，仅 1 处对偶折叠跨线 |
| `03_draft_refined` | 3 实例 · 过挤重叠 + 过松草图整理 | 整理前 FAIL → **整理后 PASS** | `9` → **`0`** | 自动消除重叠压字并收紧空白 |
| `04_cdac_and_ladder` | 12 实例 · 6 单元二进制电容阵列 + 6 级蛇形电阻梯 | PASS | `0` | 电容排严格同行等距步长，电阻串蛇形折叠 |
| `05_rcopa316_hierarchy` | **162 实例 / 82 网络 · 工业级轨到轨 Class-AB 运放（含 `.param` 公式、过驱恢复与双向短路限流）** | **展平还原 LVS 100% PASS** | **全部 11 页均为 `0`** | 自动保护原子簇拆分为 **10 张功能子页 + 1 张 `opamp_core` 顶层框图**，生成单文件可交互浏览的 `index.html` 原理图册 |


## 三、0.3.2 架构修复前后对照（离线探针，`auto` / `compile --module-plan` 交叉数·线长）

| 网表 | 草案 HARD 撕裂（前→后） | 分页撕裂（前→后，后者均为 SOFT） | `auto` 路径（前→后） | `module_plan` 草案路径（前→后） |
| :--- | :---: | :---: | :--- | :--- |
| 66 管 BGR（`work/`） | 2 → **0** | 4 → **0** HARD | 63/6985 → **50/5955** | 46/5201 → **47/5315** |
| 44 管 IDAC（`work/`） | 0 → 0 | 2 → **0** HARD | 25/2744 → 25/2744 | 25/3045 → **22/2622** |
| 41 管双通路运放（`work/`） | 0 → 0 | 3 → **0** HARD | 2/1846 → 2/1846 | 33/3648 → **2/1846** |
| 两级 OTA（`examples/`） | 0 → 0 | – | 0/586 → 0/586 | 3/844 → **0/586** |
| 折叠共源共栅（`examples/`） | 0 → 0 | – | 2/673 → 2/673 | 2/816 → **2/673** |
| OPA316 演示（`examples/`） | 18 → **0** | 6 → **0** HARD | – | 草案未通过 → **通过** |

说明：旧版把抽头节点处的拆分也计为撕裂；0.3.2 起区分 HARD（纯串联私有节点 / 同一 Super-MOS 叠管，判 ERROR）与 SOFT（多器件抽头节点，判 WARNING）。OPA316 等 `examples/` 网表只作演示与回归基准，分页排序已改为纯拓扑判据，不再使用演示电路的分段标题关键字。


## 四、0.3.3 分层与规则集中后的对照（离线探针）

| 网表 | `auto` 交叉/线长 | `module_plan` 草案路径 | 简报行数（0.3.1 → 0.3.3 摘要版） |
| :--- | :--- | :--- | :--- |
| 66 管 BGR（`work/`） | 49 / 6067 | 49 / 6067（与 `auto` 完全一致） | 177 → 142 |
| 44 管 IDAC（`work/`） | 25 / 2744 | 25 / 2744 | 246 → 98 |
| 41 管双通路运放（`work/`） | 2 / 1846 | 2 / 1846 | 194 → 188 |
| 两级 OTA（`examples/`） | 0 / 586 | 0 / 586 | 101 → 101 |
| 折叠共源共栅（`examples/`） | 2 / 673 | 2 / 673 | 91 → 92 |
| OPA316 演示（`examples/`） | – | 草案通过 | 591 → **204**（完整版另存 `semantic_digest_full.md`） |

说明：0.3.3 起展开器按草案全局列序输出（保留分析器跨组折叠列），未改动的模块草案与 `auto` 结果逐项相同；骨架只在大模型启用 `skeleton` 后才改变布局。命名启发式（编号插单管）默认关闭，可在规则文件 `topologyThresholds.allowNamingLayoutRules` 中恢复旧行为。
