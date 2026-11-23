#!/usr/bin/env python3
"""CLI entry point for `analog-schem-gen`: PDK-agnostic high-readability analog schematic generator.

Supports fine-grained LLM + Algorithm Co-Execution (`analyze` -> LLM edits `intent.json`
-> `compile` -> LLM critiques `preview.svg` + `audit.json`) and standalone modes (`auto`,
`refine`, `audit`, `demo`). Offline and pure-data by default; zero solver/simulation entry.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

from schematic_gen.analyzer import analyze_circuit
from schematic_gen.auditor import audit_manifest_and_solved, audit_solved_schematic
from schematic_gen.manifest_builder import (
    build_bridge_manifest,
    build_canvas_ir,
    ensure_virtuoso_bridge_on_path,
)
from schematic_gen.models import AuditReport, SchematicIntent
from schematic_gen.parser import parse_netlist_file
from schematic_gen.pdk import (
    DEFAULT_NETLIST_RULES_PATH,
    DEFAULT_PROCESS_MAP_PATH,
    PdkRegistry,
)
from schematic_gen.refiner import circuit_and_intent_from_manifest, refine_existing_manifest
from schematic_gen.router import compile_schematic
from schematic_gen.svg_renderer import render_schematic_svg


HERE = Path(__file__).resolve().parent


def _write_json(path: Path, data: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def _format_audit_markdown(report: AuditReport) -> str:
    m = report.metrics
    status = "PASS" if report.overall_passed else "FAIL"
    lines = [
        f"# 原理图生成签核报告 · `{report.cell_name}`",
        "",
        f"- **综合结论**：`{status}`（`signoff=false`，静态电气与图面几何门禁）",
        f"- **电气等价与连通性（LVS + 拆网预检）**：`{'PASS' if report.electrical_passed else 'FAIL'}`",
        f"- **Virtuoso Bridge 精确几何预检**：`{'PASS' if report.bridge_manifest_valid else 'FAIL'}`",
        f"- **可读性几何 DRC**：`{'PASS' if report.readability_passed else 'FAIL'}`",
        "",
        "## 关键量化指标",
        "",
        "| 指标 | 数值 | 门禁要求 |",
        "|---|---:|---|",
        f"| 器件实例数 | `{m.get('instanceCount', 0)}` | 与输入网表一致 |",
        f"| 网络总数 | `{m.get('netCount', 0)}` | 内部网单连通分量 |",
        f"| 符号重叠数 (`symbolOverlapCount`) | `{m.get('symbolOverlapCount', 0)}` | `= 0` |",
        f"| 压字/文字重叠数 (`labelOverlapCount`) | `{m.get('labelOverlapCount', 0)}` | `= 0` |",
        f"| 导线穿器件本体数 (`wireThroughBodyCount`) | `{m.get('wireThroughBodyCount', 0)}` | `= 0` |",
        f"| 导线擦异网引脚数 (`wireOverForeignPinCount`) | `{m.get('wireOverForeignPinCount', 0)}` | `= 0` |",
        f"| 异网共线重叠数 (`collinearForeignWireOverlapCount`) | `{m.get('collinearForeignWireOverlapCount', 0)}` | `= 0` |",
        f"| 歧义拐点/T点跨线数 (`ambiguousCrossingCount`) | `{m.get('ambiguousCrossingCount', 0)}` | `= 0` |",
        f"| 正交跨线交叉总数 (`wireCrossingCount`) | `{m.get('wireCrossingCount', 0)}` | 越少越好（无连接圆点） |",
        f"| 总正交走线长度（格） | `{m.get('totalWireLengthGrids', 0)}` | 紧凑无绕远 |",
        "",
    ]
    if report.findings:
        lines.append("## 检查明细 (`findings`)")
        lines.append("")
        for f in report.findings:
            lines.append(f"- **[{f.severity}] `{f.code}`**：{f.message}")
        lines.append("")
    if report.llm_refinement_suggestions:
        lines.append("## 面向大模型的迭代建议 (`llmRefinementSuggestions`)")
        lines.append("")
        for s in report.llm_refinement_suggestions:
            lines.append(f"- {s}")
        lines.append("")
    return "\n".join(lines)


def cmd_analyze(args: argparse.Namespace) -> int:
    circuit = parse_netlist_file(
        args.netlist,
        rules_path=args.rules,
        cell_name_override=args.cell_name,
    )
    analysis = analyze_circuit(circuit)
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    _write_json(out_dir / "analysis.json", analysis.to_dict())
    _write_json(out_dir / "intent.draft.json", analysis.draft_intent)
    print(f"[analyze] cell={circuit.cell_name} instances={len(circuit.instances)} nets={len(circuit.nets)}")
    print(f"[analyze] motifs={len(analysis.motifs)} llm_ambiguities={len(analysis.ambiguities_for_llm)}")
    print(f"[analyze] wrote {out_dir / 'analysis.json'} and {out_dir / 'intent.draft.json'}")
    return 0


def _emit_solved_bundle(
    solved: Any,
    pdk: PdkRegistry,
    out_dir: Path,
    *,
    debug_boxes: bool = False,
) -> AuditReport:
    out_dir.mkdir(parents=True, exist_ok=True)
    manifest = build_bridge_manifest(solved, pdk)
    canvas_ir = build_canvas_ir(solved)
    svg_text = render_schematic_svg(solved, pdk, show_groups=True, show_debug_boxes=debug_boxes)
    report = audit_solved_schematic(solved, pdk)

    _write_json(out_dir / "intent.used.json", solved.intent.to_dict())
    _write_json(out_dir / "manifest.json", manifest)
    _write_json(out_dir / "canvas_ir.json", canvas_ir)
    (out_dir / "preview.svg").write_text(svg_text + "\n", encoding="utf-8")
    _write_json(out_dir / "audit.json", report.to_dict())
    (out_dir / "audit.md").write_text(_format_audit_markdown(report) + "\n", encoding="utf-8")
    return report


def cmd_compile(args: argparse.Namespace) -> int:
    circuit = parse_netlist_file(
        args.netlist,
        rules_path=args.rules,
        cell_name_override=args.cell_name,
    )
    intent_raw = json.loads(Path(args.intent).read_text(encoding="utf-8"))
    intent = SchematicIntent.from_dict(intent_raw)
    if args.cell_name:
        intent.cell_name = args.cell_name
    pdk = PdkRegistry(args.pdk_map, args.process)
    solved = compile_schematic(circuit, intent, pdk=pdk)
    out_dir = Path(args.out)
    report = _emit_solved_bundle(solved, pdk, out_dir, debug_boxes=args.debug_boxes)
    print(
        f"[compile] cell={circuit.cell_name} overall={'PASS' if report.overall_passed else 'FAIL'} "
        f"crossings={report.metrics['wireCrossingCount']} iters={solved.congestion_iterations}"
    )
    print(f"[compile] wrote manifest, preview.svg, and audit report to {out_dir}")
    return 0 if report.overall_passed else 2


def cmd_auto(args: argparse.Namespace) -> int:
    circuit = parse_netlist_file(
        args.netlist,
        rules_path=args.rules,
        cell_name_override=args.cell_name,
    )
    analysis = analyze_circuit(circuit)
    intent = SchematicIntent.from_dict(analysis.draft_intent)
    pdk = PdkRegistry(args.pdk_map, args.process)
    solved = compile_schematic(circuit, intent, pdk=pdk)
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    _write_json(out_dir / "analysis.json", analysis.to_dict())
    report = _emit_solved_bundle(solved, pdk, out_dir, debug_boxes=args.debug_boxes)
    print(
        f"[auto] cell={circuit.cell_name} overall={'PASS' if report.overall_passed else 'FAIL'} "
        f"crossings={report.metrics['wireCrossingCount']}"
    )
    return 0 if report.overall_passed else 2


def cmd_refine(args: argparse.Namespace) -> int:
    manifest_in = json.loads(Path(args.manifest).read_text(encoding="utf-8"))
    pdk = PdkRegistry(args.pdk_map, args.process)
    circuit_before, _ = circuit_and_intent_from_manifest(manifest_in)
    audit_before = audit_manifest_and_solved(manifest_in, circuit_before, pdk)

    solved_after = refine_existing_manifest(manifest_in, pdk=pdk)
    out_dir = Path(args.out)
    audit_after = _emit_solved_bundle(solved_after, pdk, out_dir, debug_boxes=args.debug_boxes)
    _write_json(
        out_dir / "refine_comparison.json",
        {
            "before": audit_before.to_dict(),
            "after": audit_after.to_dict(),
        },
    )
    print(
        f"[refine] cell={circuit_before.cell_name} before={'PASS' if audit_before.overall_passed else 'FAIL'} "
        f"-> after={'PASS' if audit_after.overall_passed else 'FAIL'}"
    )
    return 0 if audit_after.overall_passed else 2


def cmd_audit(args: argparse.Namespace) -> int:
    manifest_in = json.loads(Path(args.manifest).read_text(encoding="utf-8"))
    pdk = PdkRegistry(args.pdk_map, args.process)
    if args.netlist:
        circuit = parse_netlist_file(args.netlist, rules_path=args.rules)
    else:
        circuit, _ = circuit_and_intent_from_manifest(manifest_in)
    report = audit_manifest_and_solved(manifest_in, circuit, pdk)
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    _write_json(out_dir / "audit.json", report.to_dict())
    (out_dir / "audit.md").write_text(_format_audit_markdown(report) + "\n", encoding="utf-8")
    print(f"[audit] cell={circuit.cell_name} overall={'PASS' if report.overall_passed else 'FAIL'}")
    return 0 if report.overall_passed else 2


def cmd_probe_pdk(args: argparse.Namespace) -> int:
    if not args.live:
        print(
            "[probe-pdk] Offline safe mode by default (no SSH / Virtuoso connection initiated).\n"
            "Pass `--live` only when authorized to probe an active Virtuoso Bridge session."
        )
        return 0
    ensure_virtuoso_bridge_on_path()
    from virtuoso_bridge import VirtuosoClient
    from virtuoso_bridge.virtuoso.schematic import load_process_map, validate_process_master_offsets

    proc_map = load_process_map(args.pdk_map)
    client = VirtuosoClient.from_env(profile=args.profile)
    checked = validate_process_master_offsets(client, proc_map, [args.process])
    if args.out:
        _write_json(Path(args.out), {"process": args.process, "checkedMasters": checked})
    print(f"[probe-pdk] verified {len(checked)} live symbol masters for process={args.process!r}")
    return 0


def cmd_demo(args: argparse.Namespace) -> int:
    out_root = Path(args.out)
    examples_dir = HERE / "examples"
    pdk = PdkRegistry()

    summary: dict[str, Any] = {"demos": []}

    # 1. SKY130 Two-Stage Miller Op-Amp (LLM-refined intent)
    c1 = parse_netlist_file(examples_dir / "sky130_two_stage_ota.spice")
    i1 = SchematicIntent.from_dict(
        json.loads((examples_dir / "sky130_two_stage_intent.json").read_text(encoding="utf-8"))
    )
    s1 = compile_schematic(c1, i1, pdk=pdk)
    r1 = _emit_solved_bundle(s1, pdk, out_root / "01_two_stage_ota")
    summary["demos"].append({"case": "01_two_stage_ota", "passed": r1.overall_passed, "metrics": r1.metrics})

    # 2. SKY130 Folded-Cascode OTA (LLM-refined intent)
    c2 = parse_netlist_file(examples_dir / "sky130_folded_cascode.spice")
    i2 = SchematicIntent.from_dict(
        json.loads((examples_dir / "sky130_folded_cascode_intent.json").read_text(encoding="utf-8"))
    )
    s2 = compile_schematic(c2, i2, pdk=pdk)
    r2 = _emit_solved_bundle(s2, pdk, out_root / "02_folded_cascode_ota")
    summary["demos"].append({"case": "02_folded_cascode_ota", "passed": r2.overall_passed, "metrics": r2.metrics})

    # 3. Cramped & Stretched Draft Schematic Cleanup (`refine`)
    draft = json.loads((examples_dir / "cramped_draft_manifest.json").read_text(encoding="utf-8"))
    c3, _ = circuit_and_intent_from_manifest(draft)
    r3_before = audit_manifest_and_solved(draft, c3, pdk)
    s3 = refine_existing_manifest(draft, pdk=pdk)
    r3_after = _emit_solved_bundle(s3, pdk, out_root / "03_draft_refined")
    _write_json(
        out_root / "03_draft_refined" / "refine_comparison.json",
        {"before": r3_before.to_dict(), "after": r3_after.to_dict()},
    )
    summary["demos"].append(
        {
            "case": "03_draft_refined",
            "beforePassed": r3_before.overall_passed,
            "afterPassed": r3_after.overall_passed,
            "beforeMetrics": r3_before.metrics,
            "afterMetrics": r3_after.metrics,
        }
    )

    _write_json(out_root / "demo_summary.json", summary)
    all_ok = r1.overall_passed and r2.overall_passed and (not r3_before.readability_passed) and r3_after.overall_passed
    print(f"[demo] completed 3 SKY130 cases -> {out_root} (all_ok={all_ok})")
    return 0 if all_ok else 2


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="analog-schem-gen",
        description="PDK-agnostic high-readability analog circuit schematic generator (LLM + Algorithm co-execution)",
    )
    sub = parser.add_subparsers(dest="command", required=True)

    def _add_common_pdk(p: argparse.ArgumentParser) -> None:
        p.add_argument("--pdk-map", type=Path, default=DEFAULT_PROCESS_MAP_PATH, help="PDK process-map JSON 路径")
        p.add_argument("--process", default="sky130", help="目标工艺名（默认 sky130）")
        p.add_argument("--rules", type=Path, default=DEFAULT_NETLIST_RULES_PATH, help="网表原语识别规则 JSON")

    p_an = sub.add_parser("analyze", help="步骤1（算法）：解析网表、提取基元候选并生成供大模型审阅的草案 Intent")
    p_an.add_argument("netlist", type=Path, help="输入 SPICE/CDL/Spectre 网表路径")
    p_an.add_argument("--out", type=Path, required=True, help="输出目录")
    p_an.add_argument("--cell-name", default=None, help="可选覆盖单元名")
    p_an.add_argument("--rules", type=Path, default=DEFAULT_NETLIST_RULES_PATH)
    p_an.set_defaults(func=cmd_analyze)

    p_co = sub.add_parser("compile", help="步骤2（算法）：按大模型确认的 Intent 求解约束间距、正交分轨走线、SVG 与签核报告")
    p_co.add_argument("netlist", type=Path, help="输入基准网表路径")
    p_co.add_argument("--intent", type=Path, required=True, help="大模型确认/精修后的 intent.json")
    p_co.add_argument("--out", type=Path, required=True, help="输出目录")
    p_co.add_argument("--cell-name", default=None)
    p_co.add_argument("--debug-boxes", action="store_true", help="在 SVG 中叠加显示符号框与参数文字包络框")
    _add_common_pdk(p_co)
    p_co.set_defaults(func=cmd_compile)

    p_au = sub.add_parser("auto", help="零样本全自动生成（使用确定性草案 Intent 直接编译）")
    p_au.add_argument("netlist", type=Path, help="输入基准网表路径")
    p_au.add_argument("--out", type=Path, required=True, help="输出目录")
    p_au.add_argument("--cell-name", default=None)
    p_au.add_argument("--debug-boxes", action="store_true")
    _add_common_pdk(p_au)
    p_au.set_defaults(func=cmd_auto)

    p_rf = sub.add_parser("refine", help="保结构整理既有原理图清单（撑开过挤压字、收紧过大空隙、重走正交分轨线）")
    p_rf.add_argument("manifest", type=Path, help="既有草图 manifest.json")
    p_rf.add_argument("--out", type=Path, required=True, help="输出目录")
    p_rf.add_argument("--debug-boxes", action="store_true")
    _add_common_pdk(p_rf)
    p_rf.set_defaults(func=cmd_refine)

    p_ck = sub.add_parser("audit", help="独立对任意 Manifest 执行 LVS 电气等价 + Bridge 几何 + 可读性 DRC 检查")
    p_ck.add_argument("manifest", type=Path, help="待检查 manifest.json")
    p_ck.add_argument("--netlist", type=Path, default=None, help="基准网表路径（省略则检查内部连通与几何 DRC）")
    p_ck.add_argument("--out", type=Path, required=True, help="输出目录")
    _add_common_pdk(p_ck)
    p_ck.set_defaults(func=cmd_audit)

    p_pr = sub.add_parser("probe-pdk", help="核对实时 Virtuoso 会话中目标 PDK 符号引脚偏移（默认离线提示，需显式 --live）")
    p_pr.add_argument("--pdk-map", type=Path, default=DEFAULT_PROCESS_MAP_PATH)
    p_pr.add_argument("--process", default="sky130")
    p_pr.add_argument("--profile", default=None)
    p_pr.add_argument("--out", type=Path, default=None)
    p_pr.add_argument("--live", action="store_true", help="显式确认连接已授权运行的 Virtuoso Bridge")
    p_pr.set_defaults(func=cmd_probe_pdk)

    p_dm = sub.add_parser("demo", help="离线运行 3 组 SKY130 代表性基准示例（两级运放、折叠共源共栅、缺陷草图整理）")
    p_dm.add_argument("--out", type=Path, required=True, help="演示输出目录")
    p_dm.set_defaults(func=cmd_demo)

    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    return int(args.func(args))


if __name__ == "__main__":
    raise SystemExit(main())
