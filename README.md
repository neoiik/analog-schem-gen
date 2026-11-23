# analog-schem-gen

**Analog Circuit Schematic Generator & Refiner (LLM × Algorithm Co-Execution)**  
**模拟电路原理图生成与整理引擎（大模型语义决策 × 确定性算法排版布线）**

[![Release](https://img.shields.io/badge/Release-v0.3.0-00897b.svg)](https://github.com/neoiik/analog-schem-gen/releases/tag/v0.3.0)
[![License](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](LICENSE)
[![Python](https://img.shields.io/badge/Python-3.10%2B-3776AB.svg)](pyproject.toml)

[English](#english) | [中文说明](#中文说明)

---

<a id="english"></a>

## Overview

Turn raw SPICE / CDL / Spectre netlists (from 10-transistor OTAs up to 160+ transistor industrial op-amps) into **clean, grid-aligned, textbook-style schematics** — without letting LLMs guess `(x, y)` coordinates.

| Layer | Owner | Role |
| :--- | :--- | :--- |
| **Semantics (`IntentSpec`)** | **LLM / Designer** | Stage order, device orientations (`gate_left`, `horizontal_l2r`), and net bus policies — **zero coordinates**. |
| **Placement & Routing** | **Deterministic Solver** | Shortest-connection 2D vertical stacking, **Convex QP** symmetric placement, collinear shared `VDD`/`VSS` rails, and orthogonal Steiner routing. |
| **Verification & Export** | **Deterministic Auditor** | Netlist connectivity & collision check, standalone SVG / multi-sheet HTML book, and Virtuoso-ready `manifest.json`. |

### What's New in v0.3.0

- **Shortest-Connection 2D Stacking**: Removes artificial horizontal group walls; parallel banks stack directly over driving blocks, and coupled dual-tail / active-load pairs align in shared columns.
- **Signal-Cohesion Vertical Floating**: Short active sub-branches (e.g., co-gated startup pairs) cohere tightly around mutual signal nodes instead of stretching to global top/bottom rows.
- **Collinear Shared `VDD` / `VSS` Rails**: Merges supply/ground taps onto flat horizontal rail trunks with a single `VDD` / `GND` symbol and clean `T`-junction solder dots.
- **Nearest-Boundary Ports & Post-Order Hierarchy**: Places I/O ports on the nearest unobstructed sheet edge and synchronizes child symbol pin orders bottom-up.

### Three Ways to Use (No EDA Environment Required)

Runs standalone on **Python 3.10+** (Windows / Linux / macOS — **no Virtuoso or commercial PDK installation required**):

1. **Inside AI Agents / CLI (Core)**: In **Claude Code, Cursor, Antigravity, or terminal**, generate clean **SVG schematics** and multi-sheet HTML books in one command right after simulation or netlist tuning.
2. **Bridge into Cadence Virtuoso**: Push the generated `manifest.json` via `virtuoso-bridge` to materialize native, editable OA schematics inside Virtuoso in one click.
3. **Built-In Browser Micro-Editor**: Run `export-editor` to get a single-file offline HTML canvas — open in any browser to drag, flip, and export updated coordinates.

## Examples (Reference Profile: SKY130)

**1. Two-Stage Miller Op-Amp**

![Two-Stage Miller Op-Amp](examples/demo_outputs/01_two_stage_ota/preview.svg)

**2. Folded-Cascode OTA**

![Folded-Cascode OTA](examples/demo_outputs/02_folded_cascode_ota/preview.svg)

**3. 162-Instance Rail-to-Rail Class-AB Op-Amp (`OPA316` · 1 Top + 10 Subsheets)**

*Level 0 — Synthesized Top Block Diagram (`opamp_core`):*

![OPA316 Top Sheet](examples/demo_outputs/05_rcopa316_hierarchy/sheets/opamp_core/preview.svg)

*Level 1 — Subsheet P03: Matched Complementary Input Stage:*

![OPA316 P03 Complementary Input Stage](examples/demo_outputs/05_rcopa316_hierarchy/sheets/opamp_core_p03_matched_complementary_input_/preview.svg)

## Quick Start

### Option A: One-Prompt Setup for AI Agents (Recommended)

Paste into **Claude Code / Cursor / Antigravity / Codex**:

```text
Clone https://github.com/neoiik/analog-schem-gen.git, install it with `pip install -e .`, read `SKILL.md`, and generate a clean schematic for <path/to/your_circuit.spice>.
```

### Option B: Manual CLI

```bash
git clone https://github.com/neoiik/analog-schem-gen.git && cd analog-schem-gen
pip install -e .

# Single-sheet schematic (<= 35 instances)
analog-schem-gen auto examples/sky130_two_stage_ota.spice --out out/ota

# Multi-sheet hierarchical HTML schematic book (40-160+ instances)
analog-schem-gen hierarchy examples/sky130_rcopa316_core.spice --out out/opa316
```

<details>
<summary><strong>Advanced Commands (LLM Co-Execution, Browser Drag Editor, Draft Cleanup, Custom PDK)</strong></summary>

```bash
# Step-by-step LLM x Algorithm co-execution (analyze -> edit intent.json -> compile)
analog-schem-gen analyze examples/sky130_two_stage_ota.spice --out out/01_analyze
analog-schem-gen compile examples/sky130_two_stage_ota.spice --intent examples/sky130_two_stage_intent.json --out out/02_compile

# Export standalone browser drag/flip editor -> refine modified manifest
analog-schem-gen export-editor examples/sky130_two_stage_ota.spice --out out/editor.html
analog-schem-gen refine examples/cramped_draft_manifest.json --out out/refined

# Extract PDK symbol map from Xschem .sym files (or pass --from-virtuoso --live)
analog-schem-gen extract-pdk --from-xschem /path/to/xschem/symbols --process my_pdk --out pdk/my_pdk_map.json

# Run unit & regression tests
python -B -m unittest discover -s tests -v
```
</details>

---

<a id="中文说明"></a>

## 中文说明

将无坐标的纯文本 SPICE / CDL / Spectre 网表（从基础运放到 160+ 器件的复杂多级电路），自动编译为**纵向成列规整、少交叉、无压字**的原理图（SVG / 交互式多页图册 / Virtuoso 精确坐标清单）。

### 一、核心思想与 v0.3.0 更新亮点

- **大模型只做器件语义解释，确定性算法完成排版布线**：大模型仅输出模块信号流向、器件语义朝向与总线策略（零坐标）；几何坐标与正交走线全部由凸二次规划（Convex QP）与正交 Steiner 路由器精确求解。
- **v0.3.0 核心升级**：
  - **最简连接 2D 纵向叠放**：取消按功能分区的横向硬隔离，并联电容阵与驱动单元一对一正叠，双尾耦合与有源负载同列贯通；
  - **信号优先纵向内聚浮动**：深栈电路中的短支路（如共栅反相启动对）优先围绕局部紧密信号聚合，不再强拉对齐全局顶底排；
  - **共享水平共线 `VDD` / `VSS` 母线**：电源与地引脚自动汇流为单根水平对齐母线并共用单个符号（端部直角折弯、内部 `T` 型焊点）；
  - **就近边界端口与后序层次同步**：输入/输出端口按最近无遮挡边界直出，多级子电路自底向上同步引脚顺序。

### 二、适用环境与三种使用方式（无 IC 环境亦可独立出图）

基于 **Python 3.10+** 跨平台运行（Windows / Linux / macOS），**无需安装 Virtuoso 或任何商业 EDA / PDK 环境，只有纯文本网表即可直接出图**：

1. **在 AI Agent / 命令行里直接用（最核心）**：在 **Claude Code、Cursor、Antigravity** 或终端里，AI 跑完仿真调好网表后，一行命令直接生成干净的 **SVG 原理图**与多页 HTML 图册（用于报告、网页预览、Markdown 展示）。
2. **配合 Virtuoso 用**：生成的精确坐标 JSON 清单（`manifest.json`），可通过 `virtuoso-bridge` 直接推入 **Cadence Virtuoso**，在 OA 库里一键生成原生可编辑原理图。
3. **自带轻量浏览器画布微调**：如果对某几个器件位置想手动挪一下，执行 `export-editor` 会生成一个**单文件离线 HTML**，直接在浏览器里点开就能拖拽、翻转并导出新坐标。

### 三、快速开始

#### 方式 1：让 AI 一键部署并画图（推荐）

将下面这段话发给 **Claude Code / Cursor / Antigravity / Codex** 等 Agent：

```text
克隆 https://github.com/neoiik/analog-schem-gen.git 并执行 pip install -e . 安装，阅读仓库根目录的 SKILL.md，把我的网表 <你的网表路径.spice> 生成为高可读原理图（若器件超过 40 个请用 hierarchy 生成多页原理图册）。
```

#### 方式 2：命令行运行

```bash
git clone https://github.com/neoiik/analog-schem-gen.git && cd analog-schem-gen
pip install -e .

# 单页原理图 (SVG + Virtuoso 清单)
analog-schem-gen auto examples/sky130_two_stage_ota.spice --out out/ota

# 大规模网表多页交互式原理图册 (index.html)
analog-schem-gen hierarchy examples/sky130_rcopa316_core.spice --out out/opa316
```

### 四、文档索引

- AI 技能规范：[SKILL.md](SKILL.md)
- 模算协同与接口规范：[docs/模算协同颗粒度规范.md](docs/模算协同颗粒度规范.md) · [docs/接口与数据结构.md](docs/接口与数据结构.md)
- 演进路线与测试报告：[docs/演进路线图.md](docs/演进路线图.md) · [VALIDATION.md](VALIDATION.md)

## License

[Apache License 2.0](LICENSE)
