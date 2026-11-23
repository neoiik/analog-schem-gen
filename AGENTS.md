# 原理图绘制工具 · 模型维护与调用约定

1. **职责与定位**：本目录提供面向网表的高可读性模拟电路原理图生成与既有草图保结构整理工具（首版默认配置开源 Apache-2.0 `SKY130` 工艺，架构与工艺无关）。通过 `pdk/<process>_process_map.json` 切换目标工艺，不在此通用目录内存放受 NDA 保护的商业原厂模型、PDK 原文件、SSH 密钥或具体项目电路。
2. **安全与离线边界**：`analyze`、`compile`、`auto`、`refine`、`audit`、`demo` 均为纯本地离线数据计算与文本/SVG 生成命令，**无模拟器求解入口、无网络连接、不主动调用 SSH 或 Virtuoso Bridge**。开发回归与静态检查不解除任何项目的暂停仿真约定。
3. **模算协同颗粒度**：遵循 [模算协同颗粒度规范](docs/模算协同颗粒度规范.md) 与 [SKILL.md](SKILL.md)。大模型通过编辑零坐标的 `intent.json` 决定功能分组顺序、器件语义朝向、网络走线策略与端口摆放；精确网格坐标、包络避让、差分整数中心对称、正交 Steiner 主干、走廊平行线分轨推开及拥塞扩宽一律由底层确定性算法完成，禁止手工篡改 `manifest.json` 中的浮点/整数坐标。
4. **真实 OA 写入前置核查**：`pdk/sky130_process_map.json` 为符合 `virtuoso-bridge-process-map-v1` 契约的标准 `0.0625"` 网格映射模板。未来经用户当次授权向真实 Virtuoso OA 库执行 `import_manifest` 前，须先用 `probe-pdk --live` 对照目标安装库的真实 symbol terminal 中心坐标与 CDF 参数名完成核查。所有离线静态报告均标记 `signoff=false`。
5. **维护与测试要求**：修改间距优化（QP）、走廊分轨推开（Edge Nudging）、正交路由或 DRC 审计逻辑时，必须保留并运行已知解测试与反例测试（`python -B -m unittest discover -s tests -v`），同步更新 [接口与数据结构](docs/接口与数据结构.md)、[TOOL_INDEX.json](TOOL_INDEX.json) 与 [VALIDATION.md](VALIDATION.md)。不主动生成哈希校验文件。
