# analog-schem-gen

**Analog Circuit Schematic Generator & Refiner (LLM × Algorithm Co-Execution)**  
**模拟电路原理图生成与整理引擎（大模型 × 确定性算法协同架构）**

[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](LICENSE)
[![Python](https://img.shields.io/badge/Python-3.10%2B-3776AB.svg)](pyproject.toml)

[English](#english) | [中文说明](#中文说明)

---

<a id="english"></a>

## Why `analog-schem-gen`?

When AI agents try to draw analog circuit schematics directly from SPICE/Spectre netlists by guessing `(x, y)` coordinates, two failure modes consistently appear:
1. **Broken connectivity**: pins miss by a single grid unit, or disconnected wire stubs fail EDA `schCheck`.
2. **Unreadable spaghetti**: parameter labels (`W/L/nf/m`) collide with adjacent transistors, global bias/supply lines cut across differential pairs, and parallel wires overlap in corridors.

`analog-schem-gen` solves this by splitting schematic generation at the **exact boundary between LLM semantic reasoning and deterministic geometric optimization**:

| Layer | Owner | Responsibility |
| :--- | :--- | :--- |
| **Circuit Semantics & Visual Critique** | **LLM Agent** | Reviews candidate motifs (`analysis.json`) and authors a **coordinate-free** `intent.json`: left-to-right functional stage order (`stage_order`), semantic device orientations (`gate_left`, `gate_right`, `horizontal_l2r`), net decluttering policies (`supply_local`, `full_wire`, `perimeter_bus`), and port placement (`near_pin`, `local_repeat`). |
| **Constrained Placement & Track Routing** | **Deterministic Engine** | Reads target PDK symbol & parameter text bounding boxes (`process-map.json`), solves **Convex Quadratic Programming (QP)** to simultaneously expand cramped clearances, compact loose intra-group gaps, and enforce **exact integer differential symmetry**, then builds **Orthogonal Steiner Trunks** with **A* obstacle-avoidance + corridor parallel track nudging (Edge Nudging)**. |
| **Electrical & Readability Gate** | **Deterministic Auditor** | Verifies LVS netlist equivalence, single-component internal wire continuity, `virtuoso-bridge-exact-schematic-v1` exact geometry compatibility, and zero symbol/label/wire collisions. |

### Sample Generated Schematics (Reference Profile: Open-Source SKY130)

**1. Two-Stage Miller-Compensated Op-Amp (12 instances · 0 DRC violations · 0 wire crossings)**

![Two-Stage Miller Op-Amp](examples/demo_outputs/01_two_stage_ota/preview.svg)

**2. Four-Tier Folded-Cascode OTA (12 instances · 0 DRC violations · 1 topological folding cross)**

![Folded-Cascode OTA](examples/demo_outputs/02_folded_cascode_ota/preview.svg)

**3. Binary CDAC Capacitor Array & Serpentine Resistor Ladder (`v0.2` Array Micro-Templates · 0 DRC violations)**

![CDAC and Serpentine Resistor Ladder](examples/demo_outputs/04_cdac_and_ladder/preview.svg)

**4. 162-Instance Rail-to-Rail Class-AB Op-Amp (`OPA316` Core · Multi-Sheet Hierarchy: 1 Top + 10 Subsheets · Flatten-Back LVS 100% PASS)**

![OPA316 Hierarchical Top Sheet](examples/demo_outputs/05_rcopa316_hierarchy/sheets/opamp_core/preview.svg)

### PDK-Agnostic Design

`analog-schem-gen` is **not tied to any single foundry PDK**. Target symbol pin offsets, bounding boxes, and CDF parameter names are decoupled into JSON profiles under [`pdk/`](pdk/):
- `sky130` ([`pdk/sky130_process_map.json`](pdk/sky130_process_map.json) + [`pdk/sky130_netlist_rules.json`](pdk/sky130_netlist_rules.json)) is bundled out-of-the-box as an open-source Apache-2.0 reference profile.
- Bring any custom or commercial PDK (e.g., 180nm, 65nm, 28nm) by adding a `<pdk>_process_map.json` (`virtuoso-bridge-process-map-v1`) and `<pdk>_netlist_rules.json` without changing engine code.

### What's New in `v0.2.0`

- **Multi-Sheet Hierarchical Compilation & Flatten-Back LVS (`partition` / `hierarchy`)**: Scales cleanly to **40–160+ instance** industrial analog blocks (validated on the 162-instance SKY130 rail-to-rail Class-AB `OPA316` core) by protecting atomic analog clusters (diff pairs, mirrors, cascodes, Miller RC), synthesizing block symbols, compiling bottom-up into a standalone multi-sheet HTML **Schematic Book (`index.html`)**, and proving **100% 1-to-1 leaf-device & net equivalence** back against the flat netlist.
- **Regular Array Micro-Templates**: Automatic detection and rigid equal-pitch / serpentine placement for binary/thermometer **capacitor & current banks (`parallel_bank`)** and **resistor ladders (`serpentine_chain`)**, with optional **Bus Ripper** rendering.
- **Bounded Crossing Minimization (`optimizer.py`)**: Equivalence-preserving local device flip / symmetric-pair swap search driven by a lexicographic readability objective ($10^5 N_{\text{DRC}} + 100 N_{\text{cross}} + 8 N_{\text{bend}} + L_{\text{wire}}$).
- **`.param` Expression Evaluation & Dual-Mode Wire Labels**: Safe AST evaluation of SPICE algebraic `.param` formulas (e.g., `{48*a2_tail}`, `9.1meg`), plus optional native short-stub wire labels (`--allow-wire-labels` + Virtuoso SKILL helper `wire_labels.il`).
- **Read-Only PDK Profiler & Browser Drag/Flip Editor (`extract-pdk` / `export-editor`)**: Extract `process_map.json` directly from Xschem `.sym` files or live Virtuoso sessions, and interactively drag/flip devices in a zero-dependency `editor.html` before re-running `refine`.

### Quick Start

```bash
pip install -e .

# Step 1 (Algorithm): Analyze netlist, detect motifs, and emit draft intent
python -B schem_gen.py analyze examples/sky130_two_stage_ota.spice --out work/01_analyze

# Step 2 (LLM / Designer): Refine coordinate-free intent.json (see examples/sky130_two_stage_intent.json)

# Step 3 (Algorithm): Compile exact manifest, publication SVG, and LVS/DRC audit report
python -B schem_gen.py compile examples/sky130_two_stage_ota.spice \
  --intent examples/sky130_two_stage_intent.json \
  --pdk-map pdk/sky130_process_map.json --process sky130 \
  --out work/02_compile

# Multi-sheet hierarchical schematic book for large netlists (e.g., 162-instance OPA316 core)
python -B schem_gen.py hierarchy examples/sky130_rcopa316_core.spice --out work/opa316_book

# Zero-shot automatic mode, interactive browser editor, and structure-preserving draft cleanup
python -B schem_gen.py auto examples/sky130_cdac_and_ladder.spice --out work/cdac_auto
python -B schem_gen.py export-editor examples/sky130_two_stage_ota.spice --intent examples/sky130_two_stage_intent.json --out work/02_compile/editor.html
python -B schem_gen.py refine examples/cramped_draft_manifest.json --out work/refined

# Read-only PDK symbol profiler (offline Xschem .sym directory or live Virtuoso Bridge)
python -B schem_gen.py extract-pdk --from-xschem /path/to/xschem/symbols --process my_pdk --out pdk/my_pdk_map.json

# Run all 26 analytical known-solution, counterexample, hierarchy LVS, and Bridge tests
python -B -m unittest discover -s tests -v
```

---

<a id="中文说明"></a>

## 中文说明

### 一、设计理念：为什么采用“模算协同”？

让大模型直接根据网表手写坐标画原理图，往往要么“引脚偏一格连不上”，要么“管子压字、偏置线横穿核心糊成一团”。`analog-schem-gen` 将流程切分为**零坐标语义意图（`intent.json`）**与**确定性几何求解**两层：

1. **大模型负责电路语义与读图审美**（见 [SKILL.md](SKILL.md) 与 [模算协同颗粒度规范](docs/模算协同颗粒度规范.md)）：
   - 审阅 `analyze` 提取的候选基元与歧义项（`ambiguitiesForLLM`）；
   - 决定功能组从左到右的信号流顺序（如 `偏置列 -> 差分输入级 -> 米勒调零补偿 -> 第二级放大 -> 输出负载`）；
   - 指定离散语义朝向（如差分对背靠背、有源镜像负载面对面共栅、米勒 `RZ+CC` 用 `horizontal_l2r` 水平跨接）与跨级偏置分流策略（`perimeter_bus` 外围母线、`near_pin` 就近贴靠端口），编译后结合 `preview.svg` 复核。
2. **确定性算法负责精确坐标与分轨走线**：
   - **双向弹性间距 + 严格整数对称**（[placer.py](schematic_gen/placer.py)）：将符号本体框与 `W/L` 参数文字框统一纳入包络，通过凸二次规划（Convex QP）同时做到“过挤处撑开、过松处收紧、组间留白”，并保证差分对与尾流管严格整数中心对称 $x_L + x_R = 2 x_{\text{tail}}$；
   - **正交 Steiner 主干 + 走廊分轨推开**（[router.py](schematic_gen/router.py)）：实现“同网共干、异网分轨（Edge Nudging）”，走廊不足时自动扩宽器件间距重排；
   - **双重签核门禁**（[auditor.py](schematic_gen/auditor.py)）：离线验证 LVS 端子网络一致性、内部网单连通分量（防止 Virtuoso `schCheck` 拆网）、`virtuoso-bridge-exact-schematic-v1` 精确几何兼容性及 6 项硬性可读性 DRC。

### 二、工艺无关架构（PDK-Agnostic）

本工具**不绑定任何特定工艺**。所有器件符号尺寸、引脚网格偏移（`pinOffsets`）与参数名映射均通过 [`pdk/`](pdk/) 下的外部 JSON 配置注入：
- 仓库默认随附开源 Apache-2.0 **SKY130** 参考配置（[`pdk/sky130_process_map.json`](pdk/sky130_process_map.json)），克隆即可直接运行演示与回归测试；
- 接入其他工艺（如 180nm / 65nm / 28nm 原厂库）时，仅需在本地提供对应的 `<process>_process_map.json` 与 `<process>_netlist_rules.json`（向真实 Virtuoso OA 库导入前可通过 `probe-pdk --live` 校验符号引脚坐标），通用引擎代码无需修改。

### 三、`v0.2.0` 新增核心能力

1. **大图（40～160+ 管）多页层次化拆分与展平还原 LVS 签核（`partition` / `hierarchy`）**：自动识别并锁死差分对、电流镜、共源共栅与 RC 补偿等**不可切分原子簇**，按功能段切分子页、自动合成 `TOP` 框图符号并生成单文件交互式原理图册 `index.html`，最后递归展平与原始平坦网表做 100% 端子网络等价核对（已在 162 实例轨到轨 Class-AB 运放 `OPA316` 核心网表上全流程通过）。
2. **规则阵列微模板与局部翻转消交叉优化**：原生识别并刚性排布 CDAC/电流源并联排（`parallel_bank` + 可选 `BusRipper`）与蛇形分压电阻串（`serpentine_chain`）；内置保等价局部离散翻转搜索（[optimizer.py](schematic_gen/optimizer.py)）自动压降非必要跨线。
3. **代数 `.param` 表达式解析与双模 Wire Label 贴标**：安全求值 `{48*a2_tail}`、`9.1meg` 等公式参数；支持 `--allow-wire-labels` 生成原生短桩网络标签及 Virtuoso `wire_labels.il` 辅助脚本。
4. **只读 PDK 符号自动提取与零依赖浏览器微调器**：`extract-pdk` 支持从 Xschem `.sym` 或在线 Virtuoso 只读提取 `process_map.json`；`export-editor` 导出单文件 `editor.html` 支持拖拽/翻转器件后回灌 `refine` 自动重连线。

### 四、文档与目录导航

- **AI Agent 技能指令**：[SKILL.md](SKILL.md)
- **模算协同分工详解**：[docs/模算协同颗粒度规范.md](docs/模算协同颗粒度规范.md)
- **CLI 与 JSON 接口契约**：[docs/接口与数据结构.md](docs/接口与数据结构.md)
- **演进路线图与 `v0.2` 详细方案**：[docs/演进路线图.md](docs/演进路线图.md) · [docs/v0.2_详细方案设计.md](docs/v0.2_详细方案设计.md)
- **26 项已知解/反例与 5 组基准电路实测记录**：[VALIDATION.md](VALIDATION.md)
- **维护与边界约定**：[AGENTS.md](AGENTS.md)

## License

Licensed under the [Apache License, Version 2.0](LICENSE).
