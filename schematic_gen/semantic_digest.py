"""LLM-facing compact high-signal circuit semantic digest renderer (`semantic_digest.md`).

Translates raw netlists + universal vertical branch folding + Pass 0-3 deterministic topology analysis
into a concise, coarse-to-fine Markdown briefing so an LLM can accurately extract:
  1. Core anchor circuits & functional modules (stripping global bias rails first),
  2. Vertical current branches (`BR01..BRk`) with zero torn series chains,
  3. Cross-device-type quasi-symmetric pairs, repeated arrays, and feedback loops,
  4. Directed signal-flow backbone & genuine semantic ambiguities
without manually traversing hundreds of flat SPICE lines or raw JSON.
"""

from __future__ import annotations

from collections import Counter, defaultdict
from typing import Any

from schematic_gen.models import TopologyAnalysis
from schematic_gen.topology_core import THRESHOLDS

_SUMMARY_LIST_CAP = 8


def _format_ctrl_summary(ctrl_inputs: list[dict[str, Any]]) -> str:
    if not ctrl_inputs:
        return "-"
    parts: list[str] = []
    seen: set[str] = set()
    for c in ctrl_inputs:
        net = c["net"]
        scope = c.get("biasScope", "signal")
        tag = "[D=G]" if c.get("isDiodeSelf") else ("[BIAS]" if scope == "global" else ("[REF]" if scope == "feedback_ref" else "[SIG]"))
        token = f"`{net}`{tag}"
        if token not in seen:
            seen.add(token)
            parts.append(token)
    return ", ".join(parts)


def _format_tap_summary(signal_taps: list[dict[str, Any]], ctrl_nets: set[str]) -> str:
    parts: list[str] = []
    seen: set[str] = set()
    for t in signal_taps:
        net = t["net"]
        if t.get("biasScope") == "global" and net in ctrl_nets:
            continue
        tag = "[PORT]" if t.get("isPort") else ""
        token = f"`{net}`{tag}"
        if token not in seen:
            seen.add(token)
            parts.append(token)
    return ", ".join(parts) if parts else "-"


def render_semantic_digest_md(analysis: TopologyAnalysis, detail: str = "summary") -> str:
    """Render the LLM semantic briefing.

    `detail="summary"` (default, `semantic_digest.md`) keeps large circuits within an LLM-friendly budget:
    full branch table only up to `digest_full_table_max_branches`, inline signal edges only for small circuits,
    capped lists elsewhere. `detail="full"` (`semantic_digest_full.md`) prints everything.
    """
    full = detail == "full"
    circuit = analysis.circuit
    inst_by_ref = circuit.instance_map()
    rails = set(circuit.supply_nets) | set(circuit.ground_nets)
    backbone = analysis.signal_backbone or {}
    roles = analysis.device_roles or {}
    ba = analysis.branch_abstraction or {}

    # Device class breakdown
    cls_counts = Counter(
        (
            "MOS" if i.device_class == "mos" else
            "BJT" if i.device_class == "bjt" else
            "R" if i.device_class.startswith("res") else
            "C" if i.device_class.startswith("cap") else
            "SRC" if i.device_class in {"isource", "vsource"} else
            i.device_class.upper()
        )
        for i in circuit.instances
    )
    cls_str = " / ".join(f"{cnt} {k}" for k, cnt in cls_counts.most_common())

    lines: list[str] = [
        f"# 电路语义分析简报 · `{circuit.cell_name}`",
        "",
        "> 本简报由**通用竖向支路折叠 + 偏置树剥离 + Pass 0–3 确定性拓扑分析器**自动生成，供大模型按**「核心锚点 → 功能模块 → 支路骨架 → 原子器件」**顺序做从粗到细决策（`module_plan.json` / `intent.json`）。",
        "",
        "## 1. 电路规模、器件构成与端口分类",
        "",
        f"- **规模**：`{len(circuit.instances)}` 器件（{cls_str}）/ `{len(circuit.nets)}` 网络 / `{len(circuit.ports)}` 顶层端口",
    ]
    if ba:
        lines.append(
            f"- **无损折叠摘要**：`{ba.get('deviceCount', len(circuit.instances))}` 器件 → "
            f"**`{ba.get('branchCount', 0)}` 条竖向支路/桥接单元**（自动隐藏 `{ba.get('hiddenSeriesNetCount', 0)}` 个纯串联内部节点；"
            f"识别 `{len(ba.get('globalBiasNets', []))}` 条全局偏置轨：`{', '.join(ba.get('globalBiasNets', [])) or '无'}`）"
        )
    lines.append(
        f"- **电源轨**：`{', '.join(circuit.supply_nets) or '(无)'}` ｜ **地轨**：`{', '.join(circuit.ground_nets) or '(无)'}`"
    )

    # Classify non-rail ports
    diff_gate_nets: set[str] = set()
    for m in analysis.motifs:
        if m.motif_type == "diff_pair":
            for k in ("left", "right"):
                r = m.roles.get(k)
                if r in inst_by_ref:
                    diff_gate_nets.add(inst_by_ref[r].terminals.get("G", ""))

    sig_inputs: list[str] = []
    bias_ports: list[str] = []
    outputs: list[str] = []
    other_ports: list[str] = []
    for p in circuit.ports:
        if p.net in rails:
            continue
        fo = analysis.net_fanout.get(p.net, 0)
        tag = f"`{p.name}` (fanout={fo})"
        if p.direction == "output":
            outputs.append(tag)
        elif p.net in diff_gate_nets:
            sig_inputs.append(tag)
        elif p.direction == "input":
            bias_ports.append(tag)
        else:
            other_ports.append(tag)

    lines.append(f"- **差分/信号输入端口**：{', '.join(sig_inputs) or '(无)'}")
    lines.append(f"- **偏置/控制端口**：{', '.join(bias_ports) or '(无)'}")
    lines.append(f"- **输出端口**：{', '.join(outputs) or '(无)'}")
    if other_ports:
        lines.append(f"- **其他端口**：{', '.join(other_ports)}")
    lines.append("")

    # Section 2: Vertical Branch Folding Map (Universal Coarse-to-Fine View)
    branches: list[dict[str, Any]] = ba.get("branches", [])
    compact_branches = (not full) and len(branches) > THRESHOLDS.digest_full_table_max_branches
    if branches:
        lines.append("## 2. 竖向支路折叠地图（Vertical Branch Map · 原子串联不撕裂）")
        lines.append("")
        lines.append("下表已将共栅叠管与纯串联支路按 `VDD → VSS` 方向无损折叠。编写 `module_plan.json` 时可直接引用 `BRxx` 编号：")
        lines.append("")
        anchor_brs = {bid for anc in ba.get("coreAnchors", []) for bid in anc.get("branches", [])}
        table_brs = [b for b in branches if (not compact_branches) or b["id"] in anchor_brs]
        if compact_branches:
            lines.append(
                f"> 摘要模式：共 `{len(branches)}` 条支路，下表只列核心锚点涉及的 `{len(table_brs)}` 条；"
                "其余按类型列出编号，完整表见 `semantic_digest_full.md`，或用 `inspect --module BRxx,...` 查看局部。"
            )
            lines.append("")
        lines.append("| 支路 | 顶→底器件序列 | 轨 | 结构签名 | 控制端 (`G/BASE`) | 对外信号端 | 类型/参数线索 |")
        lines.append("| :--- | :--- | :---: | :--- | :--- | :--- | :--- |")
        for b in table_brs:
            inst_seq = " → ".join(b["instances"])
            rail_s = f"{b.get('topRail') or '·'}→{b.get('bottomRail') or '·'}"
            ctrl_s = _format_ctrl_summary(b.get("ctrlInputs", []))
            ctrl_net_set = {c["net"] for c in b.get("ctrlInputs", [])}
            tap_s = _format_tap_summary(b.get("signalTaps", []), ctrl_net_set)
            extra_parts = [f"`{b.get('kind', 'branch')}`"]
            if b.get("sizeHighlights"):
                extra_parts.append(", ".join(b["sizeHighlights"]))
            lines.append(
                f"| **`{b['id']}`** | `{inst_seq}` | {rail_s} | `{b.get('signature', '')}` | {ctrl_s} | {tap_s} | {' · '.join(extra_parts)} |"
            )
        lines.append("")
        if compact_branches:
            rest_by_kind: dict[str, list[str]] = defaultdict(list)
            for b in branches:
                if b["id"] not in anchor_brs:
                    rest_by_kind[b.get("kind", "branch")].append(f"{b['id']}({b.get('signature', '')})")
            lines.append("**其余支路（按类型）**：")
            for kind_k, items in sorted(rest_by_kind.items()):
                lines.append(f"- `{kind_k}`（{len(items)}）：{', '.join(items)}")
            lines.append("")

    # Section 3: Bias Trees, Core Anchors, Quasi-Symmetric Pairs, Arrays & Signal-Only Inter-Branch Coupling
    if ba:
        lines.append("## 3. 偏置树剥离、核心电路锚点与纯信号耦合（Coarse-to-Fine 核心依据）")
        lines.append("")

        bias_trees: list[dict[str, Any]] = ba.get("biasTrees", [])
        if bias_trees:
            lines.append("### 3.1 偏置分配树与参考总线（划模块边界时请先剥离 `[global]` 偏置轨）")
            lines.append("")
            shown_bt = bias_trees if full else bias_trees[:_SUMMARY_LIST_CAP]
            for bt in shown_bt:
                src_s = ", ".join(bt.get("sourceBranches", [])) or ", ".join(bt.get("sourceDevices", []))
                drv_s = ", ".join(bt.get("drivenBranches", [])) or "(无)"
                lines.append(
                    f"- **`{bt['net']}`** (`{bt['kind']}` / scope=`{bt['scope']}` / fanout=`{bt['fanout']}`): "
                    f"源头 `{src_s}` ⇒ 驱动支路 `{drv_s}`"
                )
            if len(shown_bt) < len(bias_trees):
                lines.append(f"- …另有 `{len(bias_trees) - len(shown_bt)}` 条局部偏置/参考网（见完整版）")
            lines.append("")

        core_anchors: list[dict[str, Any]] = ba.get("coreAnchors", [])
        if core_anchors:
            lines.append("### 3.2 核心电路锚点候选（Core Anchors · 建议先锚定再分外围/衔接支路）")
            lines.append("")
            for idx, anc in enumerate(core_anchors[:10], start=1):
                lines.append(
                    f"{idx}. **`[{anc['kind']}]` `{anc['id']}`**（支路 `{', '.join(anc.get('branches', []))}`，"
                    f"共 `{len(anc.get('instances', []))}` 器件）：{anc.get('reason', '')}"
                )
            lines.append("")

        qpairs: list[dict[str, Any]] = ba.get("quasiSymmetricPairs", [])
        if qpairs:
            lines.append("### 3.3 跨器件类型准对称/镜像支路候选")
            lines.append("")
            shown_qp = qpairs if full else qpairs[:_SUMMARY_LIST_CAP]
            for qp in shown_qp:
                lines.append(
                    f"- **`{qp['id']}`** (`{qp['kind']}`, 相似度=`{qp['similarity']}`): {qp['reason']}"
                )
            if len(shown_qp) < len(qpairs):
                lines.append(f"- …另有 `{len(qpairs) - len(shown_qp)}` 组（见完整版）")
            lines.append("")

        arrays: list[dict[str, Any]] = ba.get("repeatedArrays", [])
        if arrays:
            lines.append("### 3.4 规整重复列阵列候选（`>= 3` 列）")
            lines.append("")
            for arr in arrays:
                lines.append(
                    f"- **`{arr['id']}`**: `{arr['columnCount']}` 列 `{arr['signature']}`（支路 `{', '.join(arr['branches'])}`），共用母线 `{arr['commonBusNet']}`"
                )
            lines.append("")

        loops: list[dict[str, Any]] = ba.get("feedbackLoops", [])
        if loops:
            lines.append("### 3.5 检测到的闭环反馈回路（Feedback Loops · 强反馈电路核心）")
            lines.append("")
            for fl in loops:
                lines.append(
                    f"- **`{fl['id']}`**：包含支路 `{', '.join(fl['branches'])}`（共 `{len(fl['instances'])}` 器件），"
                    f"环内控制网 `{', '.join(fl['ctrlNets'])}`"
                )
            lines.append("")

        sig_edges: list[dict[str, Any]] = ba.get("signalEdges", [])
        shared_buses: list[dict[str, Any]] = ba.get("sharedSignalBuses", [])
        hyper_bus_names = {sb["net"] for sb in shared_buses}
        show_edges = full or len(branches) <= THRESHOLDS.digest_signal_edges_max_branches
        if (sig_edges or shared_buses) and not show_edges:
            lines.append("### 3.6 支路纯信号耦合")
            lines.append("")
            lines.append(
                f"> 摘要模式省略（`{len(sig_edges)}` 条边）：用 `inspect --around <BRxx|器件|网络> --hops 1` 按需追线，"
                "或查看 `semantic_digest_full.md`。"
            )
            lines.append("")
        if (sig_edges or shared_buses) and show_edges:
            lines.append("### 3.6 剥离全局偏置轨后的支路纯信号耦合（Signal-Only Inter-Branch Edges）")
            lines.append("")
            lines.append("```text")
            for sb in shared_buses:
                drvs = ", ".join(sb.get("driverBranches", [])) or "-"
                ctrls = ", ".join(sb.get("ctrlBranches", [])) or "-"
                lines.append(
                    f"[SHARED BUS] {sb['net']} (branches={sb['branchCount']}, fanout={sb['fanout']}): "
                    f"ch/drv [{drvs}] ===> ctrl [{ctrls}]"
                )
            by_src: dict[str, list[dict[str, Any]]] = defaultdict(list)
            for e in sig_edges:
                if e["net"] in hyper_bus_names:
                    continue
                by_src[e["src"]].append(e)
            branch_sig_by_id = {b["id"]: b["signature"] for b in branches}
            for src in sorted(by_src.keys()):
                outs = by_src[src]
                links = [
                    f"--{e['net']}{'(ctrl)' if e['edgeKind']=='sig_ctrl' else '(ch)'}--> {e['dst']}"
                    for e in sorted(outs, key=lambda x: (x["net"], x["dst"]))
                ]
                lines.append(f"{src} [{branch_sig_by_id.get(src, '')}] : " + " | ".join(links))
            lines.append("```")
            lines.append("")

    # Section 4: Directed Signal-Flow Backbone & Bottleneck Cuts (Pass 3)
    lines.append("## 4. 宏观信号流主干（Signal-Flow Backbone & Bottleneck Cuts）")
    lines.append("")
    macro_stages: list[dict[str, Any]] = backbone.get("macroStages", [])
    blocks: list[dict[str, Any]] = backbone.get("blocks", [])
    edges: list[dict[str, Any]] = backbone.get("edges", [])
    cut_nets: list[dict[str, Any]] = backbone.get("cutNets", [])

    # Pass 3 assumes feed-forward stages separated by single D->G cuts. On feedback-dominated circuits
    # (bandgap / LDO loops) or degenerate partitions (one giant stage + 1-device tails) it is NOT applicable
    # and must not compete with Section 3 core anchors.
    n_dev = max(1, len(circuit.instances))
    loop_refs = {r for fl in ba.get("feedbackLoops", []) for r in fl.get("instances", [])}
    largest_stage = max((len(s.get("instances", [])) for s in macro_stages), default=0)
    backbone_degenerate = bool(macro_stages) and (
        (largest_stage / n_dev >= THRESHOLDS.backbone_degenerate_fraction and len(macro_stages) >= 2)
        or (len(macro_stages) == 1 and n_dev >= THRESHOLDS.backbone_single_stage_min_devices)
    )
    if backbone_degenerate:
        lines.append(
            f"> ⚠️ **Pass 3 主干对本电路不适用**（反馈环覆盖 `{len(loop_refs)}/{n_dev}` 器件，"
            f"最大阶段占 `{largest_stage}/{n_dev}`）：请以第 3 节核心锚点与反馈环划分模块，忽略本节阶段划分。"
        )
        lines.append("")
        macro_stages = []
        blocks = []

    if macro_stages:
        lines.append(f"分析器沿单向 `D→G` 瓶颈割边自动识别出 **{len(macro_stages)} 个宏观主干阶段**：")
        lines.append("")
        for st_idx, stage in enumerate(macro_stages, 1):
            st_label = stage.get("label", stage.get("id", f"Stage {st_idx}"))
            st_insts = stage.get("instances", [])
            st_blocks = stage.get("blockIds", [])
            lines.append(
                f"{st_idx}. **{st_label}**（`{len(st_insts)}` 器件，含子块 `{', '.join(st_blocks)}`）：`{', '.join(st_insts)}`"
            )
        lines.append("")

    if blocks and (full or len(blocks) <= 12):
        lines.append("```text")
        block_by_id = {b["id"]: b for b in blocks}
        out_edges: dict[str, list[dict[str, Any]]] = defaultdict(list)
        for e in edges:
            out_edges[e["src"]].append(e)
        cut_net_set = {c["net"] for c in cut_nets}
        for b in sorted(blocks, key=lambda x: (x.get("depth", 0), x.get("order", 0), x["id"])):
            b_insts = b.get("instances", [])
            inst_preview = ",".join(b_insts[:8]) + (f",...(+{len(b_insts)-8})" if len(b_insts) > 8 else "")
            lines.append(
                f"[D{b.get('depth', 0)}] {b['id']} ({b.get('kind', 'block')}): {{{inst_preview}}}"
            )
            for e in out_edges.get(b["id"], []):
                dst_b = block_by_id.get(e["dst"], {})
                is_cut = e["net"] in cut_net_set
                marker = " ===[BOTTLENECK CUT]===> " if is_cut else " ---> "
                lines.append(
                    f"      --{e['net']} ({e.get('edgeKind', 'sig')}){marker}{e['dst']} ({dst_b.get('kind', '')})"
                )
        lines.append("```")
        lines.append("")

    if cut_nets:
        lines.append("**关键级间瓶颈割边（Macro Cut Nets）**：")
        for c in cut_nets:
            lines.append(
                f"- `{c['net']}`（割宽={c.get('cutWidth', 1)}，类型=`{c.get('edgeKind', 'D->G')}`）："
                f"`{c.get('srcBlock')}` → `{c.get('dstBlock')}`"
            )
        lines.append("")

    # Section 5: Special Device Functional Roles & Key Motifs
    lines.append("## 5. 特殊器件语义角色与关键电路基元（Pass 0–2）")
    lines.append("")

    by_role: dict[str, list[tuple[str, dict[str, Any]]]] = defaultdict(list)
    for ref, info in sorted(roles.items()):
        r_name = str(info.get("role", ""))
        if r_name in {
            "mos_cap",
            "mos_active_res",
            "source_follower",
            "mos_diode",
            "bjt_diode",
            "bjt_follower",
            "composite_stack_member",
        }:
            by_role[r_name].append((ref, info))

    role_titles = [
        ("mos_cap", "MOS 电容 (`D==S`, 等效双端电容)"),
        ("mos_active_res", "浮动有源 MOS 电阻/传输管 (常与 MOSCAP 水平串联)"),
        ("source_follower", "MOS 源极跟随器 (`D` 接电源/地轨)"),
        ("bjt_follower", "BJT 射极跟随器/钳位管 (`C` 接电源/地轨)"),
        ("mos_diode", "二极管接法 MOS (`D==G`)"),
        ("bjt_diode", "二极管接法 BJT (`C==B`)"),
    ]
    any_special = False
    for r_key, r_title in role_titles:
        items = by_role.get(r_key, [])
        if not items:
            continue
        any_special = True
        desc_parts: list[str] = []
        shown_items = items if full else items[:_SUMMARY_LIST_CAP]
        for ref, info in shown_items:
            detail_s = info.get("summary", "") if full else ""
            desc_parts.append(f"`{ref}`" + (f" ({detail_s})" if detail_s else ""))
        more = "" if len(shown_items) == len(items) else f" …（共 {len(items)} 个）"
        lines.append(f"- **{r_title}**：{', '.join(desc_parts)}{more}")

    # Highlight key higher-level motifs
    highlight_types = [
        ("co_gated_stack_mirror", "共栅复合叠管组 (`Super-MOS`)"),
        ("diff_pair", "完整差分输入级（含叠管与有源负载）"),
        ("miller_bridge", "广义 Miller 频率补偿桥"),
        ("rc_series", "串联 RC 补偿网络"),
        ("sandwich_mirror_core", "三明治准镜像对称核心"),
        ("mirror_block_pair", "镜像对称子电路对"),
        ("complementary_sf", "互补源极跟随器输出缓冲"),
    ]
    for m_type, m_title in highlight_types:
        ms = [m for m in analysis.motifs if m.motif_type == m_type]
        if not ms:
            continue
        any_special = True
        lines.append(f"- **{m_title}**（{len(ms)} 组）：")
        for m in (ms if full else ms[:_SUMMARY_LIST_CAP]):
            lines.append(f"  - `{m.id}`：{m.notes}")
        if not full and len(ms) > _SUMMARY_LIST_CAP:
            lines.append(f"  - …另有 `{len(ms) - _SUMMARY_LIST_CAP}` 组（见完整版）")

    naming_lr = [m.id for m in analysis.motifs if m.motif_type == "diff_pair" and m.roles.get("lr_tiebreak") == "naming"]
    lines.append(
        "- **命名使用声明**：器件编号/网络名只在拓扑完全对称时作最终平局裁决（权重低于任何拓扑项），不代表功能归属。"
        + (f" 本电路差分对 `{', '.join(naming_lr)}` 的左右由编号决定，可按需翻转。" if naming_lr else "")
    )

    if not any_special:
        lines.append("- （无特殊复合器件或非标准角色，均为常规单管/无源器件。）")
    lines.append("")

    # Section 6: Cross-Block Shared Nets & Routing Hints
    lines.append("## 6. 跨块共享网络与走线策略提示")
    lines.append("")
    shared_nets: list[dict[str, Any]] = backbone.get("crossStageNets", [])
    if shared_nets:
        shown_sn = shared_nets if full else shared_nets[:_SUMMARY_LIST_CAP]
        for sn in shown_sn:
            lines.append(
                f"- `{sn['net']}`（{'顶层端口' if sn.get('isPort') else '内部信号'}，扇出={sn.get('fanout', 0)}，"
                f"跨越 `{', '.join(sn.get('stages', []))}`）：建议 `{sn.get('suggestedPolicy', 'perimeter_bus')}`"
                + (f" — {sn['reason']}" if sn.get("reason") and full else "")
            )
        if len(shown_sn) < len(shared_nets):
            lines.append(f"- …另有 `{len(shared_nets) - len(shown_sn)}` 条（默认策略已写入草案，见完整版）")
    else:
        lines.append("- 未发现跨越多宏观阶段的高扇出长线。")
    lines.append("")

    # Section 7: Targeted Ambiguities for LLM Judgment
    lines.append("## 7. 需大模型裁决的语义分歧点 (`ambiguitiesForLLM`)")
    lines.append("")
    if analysis.ambiguities_for_llm:
        # Aggregate the (often numerous) net-policy questions into one compact table to save LLM context.
        net_amb = [a for a in analysis.ambiguities_for_llm if a.category == "net_policy"]
        other_amb = [a for a in analysis.ambiguities_for_llm if a.category != "net_policy"]
        if net_amb:
            lines.append(
                f"**[net_policy] 跨级网络走线策略（{len(net_amb)} 条，默认值已写入草案，仅需改动例外）**："
            )
            lines.append("")
            lines.append("| 网络 | 默认 | 可选 |")
            lines.append("| :--- | :--- | :--- |")
            shown_net_amb = net_amb if full else net_amb[:_SUMMARY_LIST_CAP]
            for amb in shown_net_amb:
                lines.append(
                    f"| `{', '.join(amb.subjects)}` | `{amb.default_choice}` | `{' / '.join(amb.options)}` |"
                )
            if len(shown_net_amb) < len(net_amb):
                rest = sorted({s for a_ in net_amb[len(shown_net_amb):] for s in a_.subjects})
                lines.append(f"| …另 `{len(rest)}` 条：`{', '.join(rest)}` | 见草案 | 见完整版 |")
            lines.append("")
        for idx, amb in enumerate(other_amb, 1):
            lines.append(
                f"{idx}. **`[{amb.category}]` `{amb.id}`**：{amb.question}"
            )
            lines.append(
                f"   - 选项：`{', '.join(amb.options)}` ｜ 默认：`{amb.default_choice}`"
            )
            if amb.rationale:
                lines.append(f"   - 依据：{amb.rationale}")
    else:
        lines.append("- 无明显拓扑歧义，可直接采用 `intent.draft.json` 编译并复核。")
    lines.append("")

    return "\n".join(lines)
