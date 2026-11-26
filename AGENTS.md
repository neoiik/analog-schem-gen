# 原理图绘制工具 · 模型维护与调用约定

1. **职责与定位**：本目录提供面向网表的高可读性模拟电路原理图生成与既有草图保结构整理工具（首版默认配置开源 Apache-2.0 `SKY130` 工艺，架构与工艺无关）。通过 `pdk/<process>_process_map.json` 切换目标工艺，不在此通用目录内存放受 NDA 保护的商业原厂模型、PDK 原文件、SSH 密钥或具体项目电路。
2. **安全与离线边界**：`analyze`、`compile`、`auto`、`refine`、`audit`、`demo` 均为纯本地离线数据计算与文本/SVG 生成命令，**无模拟器求解入口、无网络连接、不主动调用 SSH 或 Virtuoso Bridge**。开发回归与静态检查不解除任何项目的暂停仿真约定。
3. **模算协同颗粒度**：遵循 [模算协同颗粒度规范](docs/模算协同颗粒度规范.md) 与 [SKILL.md](SKILL.md)。大模型通过编辑零坐标的 `intent.json` 决定功能分组顺序、器件语义朝向、网络走线策略与端口摆放；精确网格坐标、包络避让、差分整数中心对称、正交 Steiner 主干、走廊平行线分轨推开及拥塞扩宽一律由底层确定性算法完成，禁止手工篡改 `manifest.json` 中的浮点/整数坐标。
4. **真实 OA 写入前置核查**：`pdk/sky130_process_map.json` 为符合 `virtuoso-bridge-process-map-v1` 契约的标准 `0.0625"` 网格映射模板。未来经用户当次授权向真实 Virtuoso OA 库执行 `import_manifest` 前，须先用 `probe-pdk --live` 对照目标安装库的真实 symbol terminal 中心坐标与 CDF 参数名完成核查。所有离线静态报告均标记 `signoff=false`。
5. **维护与测试要求**：修改间距优化（QP）、走廊分轨推开（Edge Nudging）、正交路由或 DRC 审计逻辑时，必须保留并运行已知解测试与反例测试（`python -B -m unittest discover -s tests -v`），同步更新 [接口与数据结构](docs/接口与数据结构.md)、[TOOL_INDEX.json](TOOL_INDEX.json) 与 [VALIDATION.md](VALIDATION.md)。不主动生成哈希校验文件。
6. **演示与规则分离**：`examples/` 与 `work/` 下的网表（含 162 管 OPA316）只作演示和回归基准。`schematic_gen/` 内的规则必须由端口拓扑驱动，不得写入演示电路的器件名、网络名、分段注释或页名关键字；回归测试可以引用演示网表。
7. **原子性单一真源**：`branch_abstraction` 的 `hardSegments` 是唯一的不可拆单元定义。`analyze` 草案、`partition`/`hierarchy` 分页与 `validate-modules` 必须一致；新增分组逻辑后须保证全部基准网表的草案通过 `validate-modules`。
8. **分层与规则集中**：通用拓扑原语、默认纵层、叠管识别、器件族与全部可调阈值只放在 `schematic_gen/topology_core.py`（按工艺可在 `<pdk>_netlist_rules.json` 的 `topologyThresholds` / `deviceFamilies` 覆盖），不得在其他模块重复实现或写死器件类名。依赖方向：`placer`/`router`/`optimizer` 不导入 `analyzer`/`auditor`（优化代价经 `quality.py` 注入）；跨模块不导入下划线私有函数。器件编号与网络名只能作拓扑完全对称时的平局裁决。走线策略优先级固定为 `llm > skeleton > optimizer > analyzer`（`netPolicySources`）。
