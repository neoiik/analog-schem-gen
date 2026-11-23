#!/usr/bin/env python3
"""CLI entry point for `analog-schem-gen` v0.2.0.

Supports single-sheet LLM + Algorithm Co-Execution (`analyze`, `compile`, `auto`, `refine`, `audit`),
large-netlist multi-sheet hierarchical partitioning & flatten-back LVS (`partition`, `hierarchy`),
read-only PDK profiling (`extract-pdk`), and interactive browser drag-refinement (`export-editor`).
Offline and pure-data by default; zero solver/simulation entry.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

from schematic_gen.analyzer import analyze_circuit
from schematic_gen.auditor import audit_manifest_and_solved, audit_solved_schematic
from schematic_gen.hierarchy import (
    HierarchyIntent,
    compile_hierarchical_circuit,
    propose_flat_partition,
    render_hierarchy_book_html,
)
from schematic_gen.interactive_editor import render_interactive_editor_html
from schematic_gen.manifest_builder import (
    build_bridge_manifest,
    build_canvas_ir,
    build_virtuoso_wire_label_skill,
    ensure_virtuoso_bridge_on_path,
)
from schematic_gen.models import AuditReport, SchematicIntent
from schematic_gen.optimizer import optimize_schematic_crossings
from schematic_gen.parser import parse_netlist_design_file, parse_netlist_file
from schematic_gen.pdk import (
    DEFAULT_NETLIST_RULES_PATH,
    DEFAULT_PROCESS_MAP_PATH,
    PdkRegistry,
)
from schematic_gen.pdk_extractor import (
    extract_process_map_from_virtuoso_live,
    extract_process_map_from_xschem_dir,
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
    if len(circuit.instances) > 32:
        part = propose_flat_partition(circuit)
        _write_json(out_dir / "hierarchy_intent.draft.json", part.to_dict())
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
    if solved.wire_label_stubs:
        (out_dir / "wire_labels.il").write_text(
            build_virtuoso_wire_label_skill(solved, pdk), encoding="utf-8"
        )
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
    if args.optimize_crossings:
        solved = optimize_schematic_crossings(
            circuit, intent, pdk=pdk, allow_wire_labels=args.allow_wire_labels
        )
    else:
        solved = compile_schematic(
            circuit, intent, pdk=pdk, allow_wire_labels=args.allow_wire_labels
        )
    out_dir = Path(args.out)
    report = _emit_solved_bundle(solved, pdk, out_dir, debug_boxes=args.debug_boxes)
    print(
        f"[compile] cell={circuit.cell_name} overall={'PASS' if report.overall_passed else 'FAIL'} "
        f"crossings={report.metrics['wireCrossingCount']} opt_moves={len(solved.optimization_moves)}"
    )
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
    if args.optimize_crossings:
        solved = optimize_schematic_crossings(
            circuit, intent, pdk=pdk, allow_wire_labels=args.allow_wire_labels
        )
    else:
        solved = compile_schematic(
            circuit, intent, pdk=pdk, allow_wire_labels=args.allow_wire_labels
        )
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    _write_json(out_dir / "analysis.json", analysis.to_dict())
    report = _emit_solved_bundle(solved, pdk, out_dir, debug_boxes=args.debug_boxes)
    print(
        f"[auto] cell={circuit.cell_name} overall={'PASS' if report.overall_passed else 'FAIL'} "
        f"crossings={report.metrics['wireCrossingCount']} opt_moves={len(solved.optimization_moves)}"
    )
    return 0 if report.overall_passed else 2


def cmd_partition(args: argparse.Namespace) -> int:
    circuit = parse_netlist_file(args.netlist, rules_path=args.rules, cell_name_override=args.cell_name)
    h_intent = propose_flat_partition(circuit, max_page_size=args.max_page_size)
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    _write_json(out_dir / "hierarchy_intent.draft.json", h_intent.to_dict())
    print(
        f"[partition] cell={circuit.cell_name} instances={len(circuit.instances)} "
        f"-> {len(h_intent.pages)} pages (global_bias_nets={len(h_intent.global_bias_nets)})"
    )
    return 0


def cmd_hierarchy(args: argparse.Namespace) -> int:
    design = parse_netlist_design_file(
        args.netlist, rules_path=args.rules, top_cell_override=args.cell_name
    )
    pdk = PdkRegistry(args.pdk_map, args.process)
    h_intent = (
        HierarchyIntent.from_dict(json.loads(Path(args.hierarchy_intent).read_text(encoding="utf-8")))
        if args.hierarchy_intent
        else None
    )
    target: Any = design.cells[design.top_cell] if len(design.cells) == 1 else design
    res = compile_hierarchical_circuit(
        target,
        pdk=pdk,
        hierarchy_intent=h_intent,
        max_page_size=args.max_page_size,
        allow_wire_labels=not args.no_wire_labels,
        optimize_crossings=not args.no_optimize_crossings,
    )

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)
    if res.hierarchy_intent is not None:
        _write_json(out_dir / "hierarchy_intent.used.json", res.hierarchy_intent.to_dict())
    _write_json(out_dir / "combined_manifest.json", res.combined_manifest)
    _write_json(out_dir / "augmented_process_map.json", res.augmented_process_map)

    pdk_aug = PdkRegistry(args.pdk_map, args.process)
    pdk_aug.raw_map = res.augmented_process_map
    pdk_aug.devices.update(res.augmented_process_map["processes"][args.process].get("devices", {}))

    pages_summary: list[dict[str, Any]] = []
    for cname, solved in res.solved_pages.items():
        page_dir = out_dir / "sheets" / cname
        rep = _emit_solved_bundle(solved, pdk_aug, page_dir)
        pages_summary.append(
            {
                "cellName": cname,
                "passed": rep.overall_passed,
                "instances": rep.metrics["instanceCount"],
                "crossings": rep.metrics["wireCrossingCount"],
            }
        )

    book_html = render_hierarchy_book_html(res, pdk_aug)
    (out_dir / "index.html").write_text(book_html, encoding="utf-8")
    summary = {
        "schema": "schem_gen.hierarchy_summary.v1",
        "topCell": res.design.top_cell,
        "overallPassed": res.overall_passed,
        "flattenLvsPassed": res.flatten_lvs_passed,
        "flattenLvsDiffs": res.flatten_lvs_Diffs,
        "sheetCount": len(res.solved_pages),
        "sheets": pages_summary,
    }
    _write_json(out_dir / "hierarchy_summary.json", summary)
    print(
        f"[hierarchy] top={res.design.top_cell} sheets={len(res.solved_pages)} "
        f"flatten_lvs={'PASS' if res.flatten_lvs_passed else 'FAIL'} "
        f"overall={'PASS' if res.overall_passed else 'FAIL'}"
    )
    return 0 if res.overall_passed else 2


def cmd_extract_pdk(args: argparse.Namespace) -> int:
    if args.from_xschem:
        pmap = extract_process_map_from_xschem_dir(
            args.from_xschem,
            process_name=args.process,
            target_library=args.target_library,
            output_library=args.output_library,
        )
        _write_json(Path(args.out), pmap)
        print(
            f"[extract-pdk] extracted {len(pmap['processes'][args.process]['devices'])} "
            f"symbols from Xschem -> {args.out}"
        )
        return 0
    if args.from_virtuoso:
        if not args.live:
            print("[extract-pdk] Pass `--live` when authorized to probe live Virtuoso symbol masters.")
            return 0
        spec = json.loads(Path(args.from_virtuoso).read_text(encoding="utf-8"))
        masters = {str(k): (str(v[0]), str(v[1])) for k, v in spec.items()}
        pmap = extract_process_map_from_virtuoso_live(
            masters,
            process_name=args.process,
            output_library=args.output_library,
            profile=args.profile,
        )
        _write_json(Path(args.out), pmap)
        print(f"[extract-pdk] probed {len(masters)} Virtuoso masters -> {args.out}")
        return 0
    raise ValueError("Specify either `--from-xschem <sym_dir>` or `--from-virtuoso <masters.json>`")


def cmd_export_editor(args: argparse.Namespace) -> int:
    circuit = parse_netlist_file(args.netlist, rules_path=args.rules)
    pdk = PdkRegistry(args.pdk_map, args.process)
    if args.intent:
        intent = SchematicIntent.from_dict(json.loads(Path(args.intent).read_text(encoding="utf-8")))
    else:
        intent = SchematicIntent.from_dict(analyze_circuit(circuit).draft_intent)
    solved = compile_schematic(circuit, intent, pdk=pdk)
    html_str = render_interactive_editor_html(solved, pdk)
    out_file = Path(args.out)
    out_file.parent.mkdir(parents=True, exist_ok=True)
    out_file.write_text(html_str, encoding="utf-8")
    print(f"[export-editor] wrote standalone interactive drag editor -> {out_file}")
    return 0


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

    # 1. SKY130 Two-Stage Miller Op-Amp
    c1 = parse_netlist_file(examples_dir / "sky130_two_stage_ota.spice")
    i1 = SchematicIntent.from_dict(
        json.loads((examples_dir / "sky130_two_stage_intent.json").read_text(encoding="utf-8"))
    )
    s1 = compile_schematic(c1, i1, pdk=pdk)
    r1 = _emit_solved_bundle(s1, pdk, out_root / "01_two_stage_ota")
    summary["demos"].append({"case": "01_two_stage_ota", "passed": r1.overall_passed, "metrics": r1.metrics})

    # 2. SKY130 Folded-Cascode OTA
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

    # 4. Regular Array Bank (CDAC + Serpentine Resistor Ladder)
    cdac_path = examples_dir / "sky130_cdac_and_ladder.spice"
    r4_ok = True
    if cdac_path.is_file():
        c4 = parse_netlist_file(cdac_path)
        i4 = SchematicIntent.from_dict(analyze_circuit(c4).draft_intent)
        s4 = optimize_schematic_crossings(c4, i4, pdk=pdk)
        r4 = _emit_solved_bundle(s4, pdk, out_root / "04_cdac_and_ladder")
        r4_ok = r4.overall_passed
        summary["demos"].append({"case": "04_cdac_and_ladder", "passed": r4.overall_passed, "metrics": r4.metrics})

    _write_json(out_root / "demo_summary.json", summary)
    all_ok = (
        r1.overall_passed
        and r2.overall_passed
        and (not r3_before.readability_passed)
        and r3_after.overall_passed
        and r4_ok
    )
    print(f"[demo] completed {len(summary['demos'])} SKY130 cases -> {out_root} (all_ok={all_ok})")
    return 0 if all_ok else 2


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="analog-schem-gen",
        description="PDK-agnostic high-readability analog circuit schematic generator v0.3.0",
    )
    sub = parser.add_subparsers(dest="command", required=True)

    def _add_common_pdk(p: argparse.ArgumentParser) -> None:
        p.add_argument("--pdk-map", type=Path, default=DEFAULT_PROCESS_MAP_PATH, help="PDK process-map JSON")
        p.add_argument("--process", default="sky130", help="Target process key (default: sky130)")
        p.add_argument("--rules", type=Path, default=DEFAULT_NETLIST_RULES_PATH, help="Netlist primitive rules JSON")

    p_an = sub.add_parser("analyze", help="Step 1 (Algo): parse netlist, detect motifs, and emit draft intent")
    p_an.add_argument("netlist", type=Path)
    p_an.add_argument("--out", type=Path, required=True)
    p_an.add_argument("--cell-name", default=None)
    p_an.add_argument("--rules", type=Path, default=DEFAULT_NETLIST_RULES_PATH)
    p_an.set_defaults(func=cmd_analyze)

    p_co = sub.add_parser("compile", help="Step 2 (Algo): solve QP placement, orthogonal routing, SVG, and DRC")
    p_co.add_argument("netlist", type=Path)
    p_co.add_argument("--intent", type=Path, required=True)
    p_co.add_argument("--out", type=Path, required=True)
    p_co.add_argument("--cell-name", default=None)
    p_co.add_argument("--optimize-crossings", action="store_true", help="Run bounded local flip/swap crossing search")
    p_co.add_argument("--allow-wire-labels", action="store_true", help="Allow native Wire-Label stubs (+ wire_labels.il)")
    p_co.add_argument("--debug-boxes", action="store_true")
    _add_common_pdk(p_co)
    p_co.set_defaults(func=cmd_compile)

    p_au = sub.add_parser("auto", help="Zero-shot automatic schematic generation")
    p_au.add_argument("netlist", type=Path)
    p_au.add_argument("--out", type=Path, required=True)
    p_au.add_argument("--cell-name", default=None)
    p_au.add_argument("--optimize-crossings", action="store_true")
    p_au.add_argument("--allow-wire-labels", action="store_true")
    p_au.add_argument("--debug-boxes", action="store_true")
    _add_common_pdk(p_au)
    p_au.set_defaults(func=cmd_auto)

    p_pt = sub.add_parser("partition", help="Propose Atomic-Cluster-protected multi-sheet partition for large netlists")
    p_pt.add_argument("netlist", type=Path)
    p_pt.add_argument("--out", type=Path, required=True)
    p_pt.add_argument("--cell-name", default=None)
    p_pt.add_argument("--max-page-size", type=int, default=24)
    p_pt.add_argument("--rules", type=Path, default=DEFAULT_NETLIST_RULES_PATH)
    p_pt.set_defaults(func=cmd_partition)

    p_hi = sub.add_parser("hierarchy", help="Compile multi-sheet hierarchical schematic book + flatten-back LVS")
    p_hi.add_argument("netlist", type=Path)
    p_hi.add_argument("--hierarchy-intent", type=Path, default=None, help="Optional LLM-refined hierarchy_intent.json")
    p_hi.add_argument("--out", type=Path, required=True)
    p_hi.add_argument("--cell-name", default=None)
    p_hi.add_argument("--max-page-size", type=int, default=24)
    p_hi.add_argument("--no-wire-labels", action="store_true", help="Disable Wire-Label stubs (strict physical perimeter bus)")
    p_hi.add_argument("--no-optimize-crossings", action="store_true")
    _add_common_pdk(p_hi)
    p_hi.set_defaults(func=cmd_hierarchy)

    p_ex = sub.add_parser("extract-pdk", help="Read-only extract process-map from Xschem .sym dir or live Virtuoso")
    p_ex.add_argument("--from-xschem", type=Path, default=None, help="Directory containing Xschem .sym files")
    p_ex.add_argument("--from-virtuoso", type=Path, default=None, help="JSON mapping device keys to [lib, cell]")
    p_ex.add_argument("--process", default="custom_pdk")
    p_ex.add_argument("--target-library", default="pdk_primitives")
    p_ex.add_argument("--output-library", default="generated_schem")
    p_ex.add_argument("--profile", default=None)
    p_ex.add_argument("--live", action="store_true")
    p_ex.add_argument("--out", type=Path, required=True)
    p_ex.set_defaults(func=cmd_extract_pdk)

    p_ed = sub.add_parser("export-editor", help="Export zero-dependency interactive HTML drag/flip schematic editor")
    p_ed.add_argument("netlist", type=Path)
    p_ed.add_argument("--intent", type=Path, default=None)
    p_ed.add_argument("--out", type=Path, required=True)
    _add_common_pdk(p_ed)
    p_ed.set_defaults(func=cmd_export_editor)

    p_rf = sub.add_parser("refine", help="Structure-preserving cleanup of existing draft manifest")
    p_rf.add_argument("manifest", type=Path)
    p_rf.add_argument("--out", type=Path, required=True)
    p_rf.add_argument("--debug-boxes", action="store_true")
    _add_common_pdk(p_rf)
    p_rf.set_defaults(func=cmd_refine)

    p_ck = sub.add_parser("audit", help="Standalone LVS + Bridge preflight + readability DRC audit")
    p_ck.add_argument("manifest", type=Path)
    p_ck.add_argument("--netlist", type=Path, default=None)
    p_ck.add_argument("--out", type=Path, required=True)
    _add_common_pdk(p_ck)
    p_ck.set_defaults(func=cmd_audit)

    p_pr = sub.add_parser("probe-pdk", help="Verify live Virtuoso symbol pinOffsets against process-map")
    p_pr.add_argument("--pdk-map", type=Path, default=DEFAULT_PROCESS_MAP_PATH)
    p_pr.add_argument("--process", default="sky130")
    p_pr.add_argument("--profile", default=None)
    p_pr.add_argument("--out", type=Path, default=None)
    p_pr.add_argument("--live", action="store_true")
    p_pr.set_defaults(func=cmd_probe_pdk)

    p_dm = sub.add_parser("demo", help="Run built-in SKY130 benchmark cases offline")
    p_dm.add_argument("--out", type=Path, required=True)
    p_dm.set_defaults(func=cmd_demo)

    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    return int(args.func(args))


if __name__ == "__main__":
    raise SystemExit(main())
