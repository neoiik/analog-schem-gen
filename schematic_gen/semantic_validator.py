"""Pre-layout semantic module validator, coarse-to-fine skeleton-to-intent compiler, and netlist inspector.

Supports the strict Coarse -> Meso -> Fine workflow:
  1. `propose_module_plan_draft`: turns `TopologyAnalysis` into a lightweight editable
     `schem_gen.module_plan.v1` template using folded branch IDs (`BR01..BRk`) and core anchors.
  2. `validate_module_plan`: millisecond-fast deterministic pre-layout semantic audit checking:
       - 100% device coverage (no missing / duplicate / unknown refs),
       - Atomic vertical chain & diff-pair integrity (no torn series stacks),
       - Intra-module non-bias signal cohesion (flags misplaced/orphan branches),
       - Inter-module private signal cut anomalies,
       - Declared symmetry/mirror skeleton compatibility,
       - Core anchor & topological evidence completeness.
  3. `expand_module_plan_to_intent`: deterministically compiles a validated `module_plan.v1`
     (Coarse modules + Meso skeletons + optional Fine overrides) into a full `SchematicIntent`
     ready for Convex QP placement & orthogonal routing — relieving the LLM from hand-writing
     dozens of per-device `orientations` and `tierOverrides`.
  4. `inspect_netlist_neighborhood`: interactive read-only k-hop signal tracer & Stage-2
     Meso Focus Window inspector (stripping power rails and global bias trees by default).
"""

from __future__ import annotations

from collections import Counter, defaultdict, deque
from dataclasses import asdict, dataclass, field
from typing import Any

from schematic_gen.analyzer import analyze_circuit, order_columns_and_orient_gates
from schematic_gen.topology_core import (
    THRESHOLDS,
    assign_default_tiers as _assign_default_tiers,
    ctrl_pin_of,
    is_diode_connected,
    lcs_len as _lcs_len,
    ref_num as _ref_num,
)
from schematic_gen.models import (
    AuditFinding,
    FunctionalGroupSpec,
    NetlistCircuit,
    SchematicIntent,
    SpacingConfig,
    TopologyAnalysis,
)


@dataclass
class ModuleValidationReport:
    """Deterministic pre-layout semantic audit report for a `module_plan.v1` or `intent.v1`."""

    cell_name: str
    input_schema: str
    overall_passed: bool
    coverage_passed: bool
    atomic_chains_passed: bool
    symmetry_passed: bool
    metrics: dict[str, Any]
    findings: list[AuditFinding] = field(default_factory=list)
    fix_suggestions: list[str] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema": "schem_gen.module_audit.v1",
            "signoff": False,
            "cellName": self.cell_name,
            "inputSchema": self.input_schema,
            "overallPassed": self.overall_passed,
            "coveragePassed": self.coverage_passed,
            "atomicChainsPassed": self.atomic_chains_passed,
            "symmetryPassed": self.symmetry_passed,
            "metrics": dict(self.metrics),
            "findings": [f.to_dict() for f in self.findings],
            "fixSuggestions": list(self.fix_suggestions),
        }


def _resolve_plan_modules(
    plan_data: dict[str, Any],
    analysis: TopologyAnalysis,
) -> tuple[str, list[dict[str, Any]]]:
    """Normalize either `schem_gen.module_plan.v1` or `schem_gen.intent.v1` into a uniform module list."""
    ba = analysis.branch_abstraction or {}
    branch_by_id = {b["id"]: b for b in ba.get("branches", [])}
    branch_of_ref = ba.get("branchOfRef", {})
    schema = str(plan_data.get("schema", ""))

    raw_list = plan_data.get("modules")
    if raw_list is None:
        raw_list = plan_data.get("groups", [])
        if not schema:
            schema = "schem_gen.intent.v1"
    elif not schema:
        schema = "schem_gen.module_plan.v1"

    resolved: list[dict[str, Any]] = []
    for idx, mod in enumerate(raw_list):
        mod_id = str(mod.get("id", f"mod_{idx + 1}"))
        explicit_branches = [str(b) for b in mod.get("branches", [])]
        explicit_insts = [str(r) for r in mod.get("instances", [])]
        unknown_branches = [b for b in explicit_branches if b not in branch_by_id]

        expanded_insts: list[str] = []
        seen_r: set[str] = set()
        for bid in explicit_branches:
            if bid in branch_by_id:
                for r in branch_by_id[bid]["instances"]:
                    if r not in seen_r:
                        seen_r.add(r)
                        expanded_insts.append(r)
        for r in explicit_insts:
            if r not in seen_r:
                seen_r.add(r)
                expanded_insts.append(r)

        touched_branches = sorted(
            {branch_of_ref[r] for r in expanded_insts if r in branch_of_ref}
            | {b for b in explicit_branches if b in branch_by_id}
        )
        resolved.append(
            {
                "id": mod_id,
                "label": str(mod.get("label", mod_id)),
                "role": str(mod.get("role", "aux")),
                "isCore": bool(mod.get("isCore", mod.get("is_core", False))),
                "stage_order": int(mod.get("stage_order", mod.get("stageOrder", idx * 10))),
                "branches": explicit_branches,
                "touchedBranches": touched_branches,
                "unknownBranches": unknown_branches,
                "instances": expanded_insts,
                "evidence": str(mod.get("evidence", "")).strip(),
                "skeleton": mod.get("skeleton") if isinstance(mod.get("skeleton"), dict) else None,
            }
        )
    return schema, resolved


def validate_module_plan(
    circuit: NetlistCircuit,
    plan_data: dict[str, Any],
    analysis: TopologyAnalysis | None = None,
) -> ModuleValidationReport:
    """Run 6 deterministic pre-layout semantic checks on a `module_plan.v1` or `intent.v1`."""
    if analysis is None:
        analysis = analyze_circuit(circuit)
    ba = analysis.branch_abstraction or {}
    branches = ba.get("branches", [])
    branch_by_id = {b["id"]: b for b in branches}
    branch_of_ref = ba.get("branchOfRef", {})
    global_bias_nets = set(ba.get("globalBiasNets", []))
    rails = set(circuit.supply_nets) | set(circuit.ground_nets)
    inst_by_ref = circuit.instance_map()
    all_refs = set(inst_by_ref.keys())

    schema, modules = _resolve_plan_modules(plan_data, analysis)
    findings: list[AuditFinding] = []
    suggestions: list[str] = []

    # --- Check 1: Exact 100% Coverage (missing / unknown / duplicate refs & unknown branch IDs) ---
    ref_owners: dict[str, list[str]] = defaultdict(list)
    unknown_refs: list[str] = []
    for mod in modules:
        for ub in mod["unknownBranches"]:
            findings.append(
                AuditFinding(
                    severity="ERROR",
                    code="UNKNOWN_BRANCH_ID",
                    message=f"模块 `{mod['id']}` 引用了不存在的支路编号 `{ub}`。",
                    subjects=[mod["id"], ub],
                )
            )
        for r in mod["instances"]:
            if r not in all_refs:
                unknown_refs.append(r)
                findings.append(
                    AuditFinding(
                        severity="ERROR",
                        code="UNKNOWN_INSTANCE",
                        message=f"模块 `{mod['id']}` 引用了网表中不存在的器件 `{r}`。",
                        subjects=[mod["id"], r],
                    )
                )
            else:
                ref_owners[r].append(mod["id"])

    missing_refs = sorted(all_refs - set(ref_owners.keys()), key=_ref_num)
    dup_refs = sorted([r for r, owners in ref_owners.items() if len(owners) > 1], key=_ref_num)

    if missing_refs:
        missing_brs = sorted({branch_of_ref[r] for r in missing_refs if r in branch_of_ref})
        findings.append(
            AuditFinding(
                severity="ERROR",
                code="MISSING_INSTANCES",
                message=f"遗漏 {len(missing_refs)} 个器件未归入任何模块：{missing_refs}（对应支路 {missing_brs}）。",
                subjects=missing_refs,
            )
        )
        suggestions.append(f"将遗漏支路 {missing_brs}（器件 {missing_refs}）补充归入合适的功能模块。")
    if dup_refs:
        for r in dup_refs:
            findings.append(
                AuditFinding(
                    severity="ERROR",
                    code="DUPLICATE_INSTANCE",
                    message=f"器件 `{r}` 被重复归入多个模块：{ref_owners[r]}。",
                    subjects=[r, *ref_owners[r]],
                )
            )
        suggestions.append(f"移除重复器件 {dup_refs} 的冗余归属，确保每个器件仅属于 1 个模块。")

    coverage_passed = (
        not missing_refs
        and not dup_refs
        and not unknown_refs
        and not any(f.code == "UNKNOWN_BRANCH_ID" for f in findings)
    )

    # Primary module owner per valid ref
    mod_of_ref = {r: owners[0] for r, owners in ref_owners.items()}

    # --- Check 2: Atomic Vertical Chain & Diff-Pair Integrity (no torn series stacks!) ---
    # HARD joints (private pure-series node / same Super-MOS stack) must never be split -> ERROR.
    # SOFT joints (continuation through a tapped node) may be a legitimate module boundary -> WARNING.
    torn_chains: list[dict[str, Any]] = []
    for b in branches:
        if len(b["instances"]) < 2:
            continue
        mods_in_b = sorted({mod_of_ref[r] for r in b["instances"] if r in mod_of_ref})
        if len(mods_in_b) < 2:
            continue
        hard_segs = b.get("hardSegments") or [list(b["instances"])]
        torn_hard = [
            seg for seg in hard_segs
            if len({mod_of_ref[r] for r in seg if r in mod_of_ref}) >= 2
        ]
        sev = "ERROR" if torn_hard else "WARNING"
        code = "TORN_ATOMIC_BRANCH" if torn_hard else "SPLIT_AT_SOFT_JOINT"
        torn_chains.append(
            {
                "branch": b["id"],
                "instances": list(b["instances"]),
                "splitAcross": mods_in_b,
                "severity": sev,
                "tornHardSegments": torn_hard,
            }
        )
        if torn_hard:
            msg = (
                f"原子竖向支路 `{b['id']}` ({' → '.join(b['instances'])}) 的硬串联段 "
                f"{[' → '.join(s) for s in torn_hard]} 被撕裂到多个模块：{mods_in_b}。"
            )
            suggestions.append(
                f"硬串联段 {[' → '.join(s) for s in torn_hard]} 必须归入同一模块（当前分属 {mods_in_b}）。"
            )
        else:
            msg = (
                f"支路 `{b['id']}` ({' → '.join(b['instances'])}) 仅在软连接点（多器件抽头节点）处分属 {mods_in_b}，"
                "若这是有意的模块边界（如折叠点、级间接口）可保留。"
            )
        findings.append(
            AuditFinding(severity=sev, code=code, message=msg, subjects=[b["id"], *b["instances"]])
        )

    # Also verify differential pairs aren't split across modules
    for m in analysis.motifs:
        if m.motif_type == "diff_pair":
            core_pair = [m.roles.get("left"), m.roles.get("right")]
            core_pair = [r for r in core_pair if r in mod_of_ref]
            mods_dp = sorted({mod_of_ref[r] for r in core_pair})
            if len(mods_dp) >= 2:
                torn_chains.append({"motif": m.id, "instances": core_pair, "splitAcross": mods_dp, "severity": "ERROR"})
                findings.append(
                    AuditFinding(
                        severity="ERROR",
                        code="TORN_DIFF_PAIR",
                        message=f"差分输入对 `{m.id}` ({core_pair}) 被拆分到不同模块：{mods_dp}。",
                        subjects=core_pair,
                    )
                )
                suggestions.append(f"将差分对 {core_pair} 合并到同一核心模块中。")

    atomic_chains_passed = not any(f.severity == "ERROR" and f.code in {"TORN_ATOMIC_BRANCH", "TORN_DIFF_PAIR"} for f in findings)

    # --- Check 3: Intra-Module Non-Bias Signal Cohesion (Orphan / Misplaced Branch Detection) ---
    # Build branch-to-branch low-fanout non-bias signal adjacency
    sig_edges = ba.get("signalEdges", [])
    shared_bus_nets = {sb["net"] for sb in ba.get("sharedSignalBuses", [])}
    local_sig_adj: dict[str, dict[str, set[str]]] = defaultdict(lambda: defaultdict(set))
    all_sig_adj: dict[str, dict[str, set[str]]] = defaultdict(lambda: defaultdict(set))
    for e in sig_edges:
        u, v, net = e["src"], e["dst"], e["net"]
        all_sig_adj[u][v].add(net)
        all_sig_adj[v][u].add(net)
        if e["fanout"] <= THRESHOLDS.local_fanout_max and net not in shared_bus_nets:
            local_sig_adj[u][v].add(net)
            local_sig_adj[v][u].add(net)

    # Majority module of each branch
    mod_of_branch: dict[str, str] = {}
    for b in branches:
        cnts = Counter(mod_of_ref[r] for r in b["instances"] if r in mod_of_ref)
        if cnts:
            mod_of_branch[b["id"]] = cnts.most_common(1)[0][0]

    orphan_branches: list[dict[str, Any]] = []
    cohesion_summary: dict[str, dict[str, Any]] = {}
    for mod in modules:
        mid = mod["id"]
        m_brs = [bid for bid in mod["touchedBranches"] if mod_of_branch.get(bid) == mid]
        m_bset = set(m_brs)
        # Count connected components inside module using non-bias signal edges (or shared global bias if role=='bias')
        seen_c: set[str] = set()
        comp_cnt = 0
        for bid in m_brs:
            if bid in seen_c:
                continue
            comp_cnt += 1
            q = deque([bid])
            seen_c.add(bid)
            while q:
                cur = q.popleft()
                for nb in all_sig_adj.get(cur, {}):
                    if nb in m_bset and nb not in seen_c:
                        seen_c.add(nb)
                        q.append(nb)
        cohesion_summary[mid] = {
            "instanceCount": len(mod["instances"]),
            "branchCount": len(m_brs),
            "signalComponents": comp_cnt,
        }

        if len(m_brs) < 2 or mod["role"] == "bias":
            continue
        for bid in m_brs:
            intra_nets = {
                n
                for nb, nset in all_sig_adj.get(bid, {}).items()
                if nb in m_bset
                for n in nset
            }
            if intra_nets:
                continue
            # Branch has ZERO non-bias signal links inside its own module; check external signal neighbors
            # (preferring low-fanout private links first, then falling back to shared signal buses like `E`/`VREF`).
            ext_by_mod: dict[str, set[str]] = defaultdict(set)
            for nb, nset in local_sig_adj.get(bid, {}).items():
                nb_mod = mod_of_branch.get(nb)
                if nb_mod and nb_mod != mid:
                    ext_by_mod[nb_mod].update(nset)
            if not ext_by_mod:
                for nb, nset in all_sig_adj.get(bid, {}).items():
                    nb_mod = mod_of_branch.get(nb)
                    if nb_mod and nb_mod != mid:
                        ext_by_mod[nb_mod].update(nset)
            if ext_by_mod:
                best_ext_mod, best_nets = max(ext_by_mod.items(), key=lambda kv: (len(kv[1]), kv[0]))
                orphan_branches.append(
                    {
                        "branch": bid,
                        "instances": list(branch_by_id[bid]["instances"]),
                        "assignedModule": mid,
                        "suggestedModule": best_ext_mod,
                        "viaNets": sorted(best_nets),
                    }
                )
                findings.append(
                    AuditFinding(
                        severity="WARNING",
                        code="ORPHAN_BRANCH_IN_MODULE",
                        message=(
                            f"支路 `{bid}` ({' → '.join(branch_by_id[bid]['instances'])}) 在模块 `{mid}` 内无非偏置信号连接，"
                            f"却经由局部信号网 `{', '.join(sorted(best_nets))}` 直连外部模块 `{best_ext_mod}`。"
                        ),
                        subjects=[bid, mid, best_ext_mod],
                    )
                )
                suggestions.append(
                    f"核对支路 `{bid}` ({branch_by_id[bid]['instances']}) 是否应从 `{mid}` 移至 `{best_ext_mod}`（直连网 `{', '.join(sorted(best_nets))}`）。"
                )

    # --- Check 4: Inter-Module Private Signal Cut Audit ---
    inter_cuts: dict[tuple[str, str], set[str]] = defaultdict(set)
    for net in circuit.nets:
        if net in rails or net in global_bias_nets or net in shared_bus_nets:
            continue
        if analysis.net_fanout.get(net, 0) > THRESHOLDS.local_fanout_max:
            continue
        touched_mods = sorted(
            {
                mod_of_ref[inst.ref]
                for inst in circuit.instances
                if inst.ref in mod_of_ref and any(p != "B" and tn == net for p, tn in inst.terminals.items())
            }
        )
        if len(touched_mods) == 2:
            inter_cuts[(touched_mods[0], touched_mods[1])].add(net)

    inter_cut_list = [
        {"moduleA": a, "moduleB": b, "cutWidth": len(nets), "nets": sorted(nets)}
        for (a, b), nets in sorted(inter_cuts.items(), key=lambda kv: (-len(kv[1]), kv[0]))
    ]
    for cut in inter_cut_list:
        if cut["cutWidth"] >= THRESHOLDS.inter_module_cut_warn:
            findings.append(
                AuditFinding(
                    severity="WARNING",
                    code="HIGH_INTER_MODULE_CUT",
                    message=(
                        f"模块 `{cut['moduleA']}` 与 `{cut['moduleB']}` 之间存在 {cut['cutWidth']} 条私有局部割边 "
                        f"`{', '.join(cut['nets'])}`：若非主反馈闭环接口，请核对边界切分是否过细。"
                    ),
                    subjects=[cut["moduleA"], cut["moduleB"], *cut["nets"]],
                )
            )

    # --- Check 5: Declared Symmetry / Mirror Skeleton Verification ---
    symmetry_errors = 0
    for mod in modules:
        skel = mod.get("skeleton")
        if not skel:
            continue
        stype = str(skel.get("type", ""))
        if stype in {"mirror_pair", "sandwich_mirror_core"}:
            l_brs = [str(b) for b in skel.get("leftBranches", [])]
            r_brs = [str(b) for b in skel.get("rightBranches", [])]
            l_cols = [[str(x) for x in c] for c in skel.get("leftColumns", [])]
            r_cols = [[str(x) for x in c] for c in skel.get("rightColumns", [])]

            if l_brs or r_brs:
                if len(l_brs) != len(r_brs):
                    symmetry_errors += 1
                    findings.append(
                        AuditFinding(
                            severity="ERROR",
                            code="SYMMETRY_COLUMN_COUNT_MISMATCH",
                            message=f"模块 `{mod['id']}` 的镜像骨架左右支路数不一致：{l_brs} vs {r_brs}。",
                            subjects=[mod["id"], *l_brs, *r_brs],
                        )
                    )
                else:
                    mod_inst_set = set(mod["instances"])
                    for lb, rb in zip(l_brs, r_brs):
                        if lb not in branch_by_id or rb not in branch_by_id:
                            symmetry_errors += 1
                            findings.append(
                                AuditFinding(
                                    severity="ERROR",
                                    code="SYMMETRY_UNKNOWN_BRANCH",
                                    message=f"模块 `{mod['id']}` 的镜像对 ({lb}, {rb}) 包含不存在的支路编号。",
                                    subjects=[mod["id"], lb, rb],
                                )
                            )
                            continue
                        b_l, b_r = branch_by_id[lb], branch_by_id[rb]
                        if not (set(b_l["instances"]) <= mod_inst_set and set(b_r["instances"]) <= mod_inst_set):
                            symmetry_errors += 1
                            findings.append(
                                AuditFinding(
                                    severity="ERROR",
                                    code="SYMMETRY_OUTSIDE_MODULE",
                                    message=f"模块 `{mod['id']}` 的镜像支路 ({lb}, {rb}) 中有器件未包含在该模块成员列表中。",
                                    subjects=[mod["id"], lb, rb],
                                )
                            )
                            continue
                        sim = 2.0 * _lcs_len(b_l["tokens"], b_r["tokens"]) / max(
                            1, len(b_l["tokens"]) + len(b_r["tokens"])
                        )
                        if sim < THRESHOLDS.skeleton_min_similarity:
                            symmetry_errors += 1
                            findings.append(
                                AuditFinding(
                                    severity="ERROR",
                                    code="SYMMETRY_TYPE_MISMATCH",
                                    message=(
                                        f"模块 `{mod['id']}` 声明镜像配对 `{lb}` ({b_l['signature']}) ↔ `{rb}` ({b_r['signature']})，"
                                        f"但器件结构相似度仅 {sim:.2f} (< 0.50)，请核对是否配错支路。"
                                    ),
                                    subjects=[mod["id"], lb, rb],
                                )
                            )
                        elif sim < 1.0:
                            findings.append(
                                AuditFinding(
                                    severity="INFO",
                                    code="QUASI_SYMMETRY_RECORDED",
                                    message=(
                                        f"模块 `{mod['id']}` 为准对称镜像 `{lb}` ({b_l['signature']}) ↔ `{rb}` ({b_r['signature']})，"
                                        f"相似度 {sim:.2f}（自动按最长公共子序列对齐相同层并兼容差异器件）。"
                                    ),
                                    subjects=[mod["id"], lb, rb],
                                )
                            )
            elif l_cols or r_cols:
                if len(l_cols) != len(r_cols):
                    symmetry_errors += 1
                    findings.append(
                        AuditFinding(
                            severity="ERROR",
                            code="SYMMETRY_COLUMN_COUNT_MISMATCH",
                            message=f"模块 `{mod['id']}` 的镜像列数不一致：{len(l_cols)} vs {len(r_cols)}。",
                            subjects=[mod["id"]],
                        )
                    )

    symmetry_passed = symmetry_errors == 0

    # --- Check 6: Core Anchor & Topological Evidence Check (for `module_plan.v1`) ---
    if schema == "schem_gen.module_plan.v1":
        if not any(m["isCore"] for m in modules):
            findings.append(
                AuditFinding(
                    severity="WARNING",
                    code="NO_CORE_ANCHOR_MARKED",
                    message="未标记任何核心功能模块（`isCore: true`）；建议显式锚定 1～2 个最核心电路模块（如带隙核、误差放大器、主差分核或权重阵）。",
                    subjects=[],
                )
            )
            suggestions.append("在 `modules[]` 中为核心电路模块设置 `\"isCore\": true`。")
        missing_ev = [m["id"] for m in modules if not m["evidence"]]
        if missing_ev:
            findings.append(
                AuditFinding(
                    severity="INFO",
                    code="MISSING_TOPOLOGY_EVIDENCE",
                    message=f"模块 {missing_ev} 未填写 `evidence` 拓扑依据；填写简洁拓扑依据有助于避免按编号顺序误分组。",
                    subjects=missing_ev,
                )
            )

    has_error = any(f.severity == "ERROR" for f in findings)
    metrics = {
        "instanceCount": len(all_refs),
        "coveredInstanceCount": len(ref_owners),
        "coverageRatio": round(len(ref_owners) / max(1, len(all_refs)), 4),
        "moduleCount": len(modules),
        "coreModuleCount": sum(1 for m in modules if m["isCore"]),
        "tornChainCount": sum(1 for t in torn_chains if t["severity"] == "ERROR"),
        "softSplitCount": sum(1 for t in torn_chains if t["severity"] == "WARNING"),
        "orphanBranchCount": len(orphan_branches),
        "errorCount": sum(1 for f in findings if f.severity == "ERROR"),
        "warningCount": sum(1 for f in findings if f.severity == "WARNING"),
        "interModuleSignalCuts": inter_cut_list,
        "moduleCohesion": cohesion_summary,
    }

    return ModuleValidationReport(
        cell_name=circuit.cell_name,
        input_schema=schema,
        overall_passed=not has_error,
        coverage_passed=coverage_passed,
        atomic_chains_passed=atomic_chains_passed,
        symmetry_passed=symmetry_passed,
        metrics=metrics,
        findings=findings,
        fix_suggestions=list(dict.fromkeys(suggestions)),
    )


def format_module_audit_markdown(report: ModuleValidationReport) -> str:
    """Render compact human/LLM-readable Markdown summary for `validate-modules`."""
    m = report.metrics
    status = "PASS" if report.overall_passed else "FAIL"
    lines = [
        f"# 语义模块划分审计报告 · `{report.cell_name}`",
        "",
        f"- **综合门禁**：`{status}`（`schema={report.input_schema}`，布局前置静态校验）",
        f"- **100% 器件覆盖**：`{'PASS' if report.coverage_passed else 'FAIL'}`（`{m['coveredInstanceCount']}/{m['instanceCount']}`）",
        f"- **原子串联支路完整**：`{'PASS' if report.atomic_chains_passed else 'FAIL'}`（硬撕裂数=`{m['tornChainCount']}`，软连接点分属=`{m.get('softSplitCount', 0)}`）",
        f"- **镜像/对称骨架核验**：`{'PASS' if report.symmetry_passed else 'FAIL'}`",
        f"- **模块内聚预警（孤立错放支路数）**：`{m['orphanBranchCount']}` ｜ 警告总数=`{m['warningCount']}`",
        "",
    ]
    cuts = m.get("interModuleSignalCuts", [])
    if cuts:
        lines.append("## 模块间非偏置局部信号割边")
        lines.append("")
        for c in cuts[:12]:
            lines.append(f"- `{c['moduleA']}` ↔ `{c['moduleB']}`（割宽 `{c['cutWidth']}`）：`{', '.join(c['nets'])}`")
        lines.append("")
    if report.findings:
        lines.append("## 检查明细 (`findings`)")
        lines.append("")
        for f in report.findings:
            lines.append(f"- **[{f.severity}] `{f.code}`**：{f.message}")
        lines.append("")
    if report.fix_suggestions:
        lines.append("## 建议修正动作")
        lines.append("")
        for s in report.fix_suggestions:
            lines.append(f"- {s}")
        lines.append("")
    return "\n".join(lines)


def propose_module_plan_draft(
    circuit: NetlistCircuit,
    analysis: TopologyAnalysis | None = None,
) -> dict[str, Any]:
    """Generate a lightweight `schem_gen.module_plan.v1` draft grounded in folded branch IDs."""
    if analysis is None:
        analysis = analyze_circuit(circuit)
    ba = analysis.branch_abstraction or {}
    branch_by_id = {b["id"]: b for b in ba.get("branches", [])}
    branch_of_ref = ba.get("branchOfRef", {})
    draft_intent = analysis.draft_intent or {}

    # Identify core branch sets from detected anchors
    core_anchor_brs: set[str] = set()
    for anc in ba.get("coreAnchors", []):
        if anc["kind"] in {"sandwich_mirror_core", "coupled_mirror_pair", "quasi_symmetric_core", "diff_pair", "repeated_branch_array"}:
            core_anchor_brs.update(anc.get("branches", []))

    # Mirror the draft intent's exact group membership: whole folded branches where a branch lies fully in one
    # group, loose `instances` where the draft intentionally splits a branch at a SOFT joint. This keeps
    # `module_plan.draft.json` and `intent.draft.json` semantically identical (single source of truth).
    groups = sorted(draft_intent.get("groups", []), key=lambda g: g.get("stage_order", 0))
    group_of_ref = {r: g["id"] for g in groups for r in g.get("instances", [])}
    draft_motif_refs: set[str] = set()
    for dm in draft_intent.get("motifs", []):
        for key in ("left", "right", "tail", "loadLeft", "loadRight"):
            if dm.get(key):
                draft_motif_refs.add(str(dm[key]))
        for key in ("instances", "leftColumn", "rightColumn"):
            draft_motif_refs.update(str(r) for r in (dm.get(key) or []))
        for key in ("leftColumns", "rightColumns"):
            for col in dm.get(key) or []:
                draft_motif_refs.update(str(r) for r in col)

    # Attach skeleton hints for recognized quasi-symmetric pairs / arrays / bias roots
    qpair_by_bset: dict[frozenset[str], dict[str, Any]] = {}
    for qp in ba.get("quasiSymmetricPairs", []):
        if qp["kind"] in {"coupled_mirror_pair", "quasi_symmetric_core"}:
            key = frozenset(qp["leftBranches"] + qp["rightBranches"])
            qpair_by_bset[key] = qp

    modules_out: list[dict[str, Any]] = []
    covered_refs: set[str] = set()
    for idx, g in enumerate(groups):
        gid = g["id"]
        g_refs = [r for r in g.get("instances", []) if r in branch_of_ref]
        if not g_refs:
            continue
        g_ref_set = set(g_refs)
        g_brs = [
            b["id"] for b in ba.get("branches", [])
            if b["instances"] and all(group_of_ref.get(r) == gid for r in b["instances"])
        ]
        whole_refs = {r for bid in g_brs for r in branch_by_id[bid]["instances"]}
        loose_refs = [r for r in g_refs if r not in whole_refs]
        covered_refs.update(g_ref_set)
        g_bset = set(g_brs)
        touched_bset = {branch_of_ref[r] for r in g_refs}
        is_core = bool(touched_bset & core_anchor_brs) or g.get("role") in {"input_diff", "cascode_core"}
        skel: dict[str, Any] | None = None

        # Only propose a skeleton where the analyzer has NOT already locked the devices in a motif
        for kset, qp in qpair_by_bset.items():
            k_refs = {r for bid in kset for r in branch_by_id[bid]["instances"]}
            if kset <= g_bset and not (k_refs & draft_motif_refs):
                skel = {
                    "type": "mirror_pair",
                    "leftBranches": list(qp["leftBranches"]),
                    "rightBranches": list(qp["rightBranches"]),
                }
                break

        mod_entry: dict[str, Any] = {
            "id": gid,
            "label": str(g.get("label", gid)),
            "role": str(g.get("role", "aux")),
            "isCore": is_core,
            "stage_order": idx * 10,
            "branches": g_brs,
            "evidence": f"算法初始草案分组（含 {len(g_brs)} 条完整支路）；请对照 semantic_digest.md 第 2/3 节复核核心锚点与模块边界。",
        }
        if loose_refs:
            mod_entry["instances"] = loose_refs
        if skel is not None:
            # Proposal only: the draft must reproduce `intent.draft.json` exactly. The LLM activates it by
            # renaming `suggestedSkeleton` -> `skeleton` after checking the module with `inspect --module`.
            mod_entry["suggestedSkeleton"] = skel
        modules_out.append(mod_entry)

    # Fallback for any leftover device never grouped by the draft
    leftover_refs = [r for r in branch_of_ref if r not in covered_refs]
    leftover = sorted({branch_of_ref[r] for r in leftover_refs})
    if leftover:
        modules_out.append(
            {
                "id": "residual_branches",
                "label": "未分组支路",
                "role": "aux",
                "isCore": False,
                "stage_order": len(modules_out) * 10,
                "instances": leftover_refs,
                "evidence": "待归类残余器件。",
            }
        )

    return {
        "schema": "schem_gen.module_plan.v1",
        "cellName": circuit.cell_name,
        "circuitType": "analog_circuit",
        "authoredBy": "analyzer_branch_draft",
        "llmNotes": (
            "Coarse-to-Fine 模块草案：请按「1. 锁定核心锚点 -> 2. 划分功能模块(branches)与 evidence -> "
            "3. 为关键模块填写 skeleton（mirror_pair / diff_stage / parallel_array / bias_bank）」顺序修订，"
            "再用 `validate-modules` 核验后通过 `compile --module-plan` 直接出图。"
        ),
        "modules": modules_out,
        # Editing surfaces only: analyzer defaults are inherited automatically during expansion. Anything the
        # LLM writes here is recorded with provenance `llm` and outranks skeleton / optimizer / analyzer.
        "netPolicies": {},
        "portSides": {},
        "overrides": {},
    }


def _align_quasi_mirror_pairs(
    left_refs: list[str],
    right_refs: list[str],
    left_tokens: list[str],
    right_tokens: list[str],
) -> list[list[str]]:
    """Pair corresponding devices across two (possibly unequal-length) mirrored columns via LCS."""
    m, n = len(left_tokens), len(right_tokens)
    dp = [[0] * (n + 1) for _ in range(m + 1)]
    for i in range(m - 1, -1, -1):
        for j in range(n - 1, -1, -1):
            if left_tokens[i] == right_tokens[j]:
                dp[i][j] = 1 + dp[i + 1][j + 1]
            else:
                dp[i][j] = max(dp[i + 1][j], dp[i][j + 1])
    pairs: list[list[str]] = []
    i = j = 0
    while i < m and j < n:
        if left_tokens[i] == right_tokens[j]:
            pairs.append([left_refs[i], right_refs[j]])
            i += 1
            j += 1
        elif dp[i + 1][j] >= dp[i][j + 1]:
            i += 1
        else:
            j += 1
    return pairs


def _assign_branch_tiers(
    refs: list[str],
    inst_by_ref: dict[str, Any],
    default_tiers: dict[str, int],
) -> dict[str, int]:
    """Map an ordered top-to-bottom branch smoothly into vertical tiers 1..5."""
    out: dict[str, int] = {}
    n = len(refs)
    if n == 0:
        return out
    if n == 1:
        r = refs[0]
        out[r] = default_tiers.get(r, 3)
        return out
    # Preserve monotonic top-to-bottom progression across tiers 1..5
    for idx, r in enumerate(refs):
        inst = inst_by_ref[r]
        if idx == 0 and inst.polarity in {"pmos", "pnp"}:
            t = 1
        elif idx == n - 1 and inst.polarity in {"nmos", "npn"}:
            t = 5
        else:
            t = 1 + int(round(4.0 * idx / max(1, n - 1)))
        out[r] = max(1, min(5, t))
    return out


def expand_module_plan_to_intent(
    circuit: NetlistCircuit,
    plan_data: dict[str, Any],
    analysis: TopologyAnalysis | None = None,
) -> SchematicIntent:
    """Deterministically expand a Coarse-to-Fine `module_plan.v1` into a full `SchematicIntent`.

    Automatically derives:
      - Ordered `groups` & `columnOrderHints` from module `stage_order` + folded branches + skeletons,
      - `motifs` (`mirror_block_pair`, `diff_pair`, `parallel_bank`, `horizontal_bridge`),
      - Default `orientations` (mirrored faces, bias reference facing right, signal-neighbor gate facing),
      - Monotonic `tierOverrides` from top-to-bottom folded vertical branches,
      - Clean `netPolicies` & `portSides` (honoring any Fine-stage overrides supplied by the LLM).
    """
    if analysis is None:
        analysis = analyze_circuit(circuit)
    ba = analysis.branch_abstraction or {}
    branch_by_id = {b["id"]: b for b in ba.get("branches", [])}
    branch_of_ref = ba.get("branchOfRef", {})
    global_bias_nets = set(ba.get("globalBiasNets", []))
    inst_by_ref = circuit.instance_map()
    rails = set(circuit.supply_nets) | set(circuit.ground_nets)
    default_tiers = _assign_default_tiers(circuit)
    fanout = analysis.net_fanout
    device_roles = analysis.device_roles

    # Collect diode refs (both MOS & BJT) for orientation rules
    stack_by_ref: dict[str, dict[str, Any]] = {}
    for m in analysis.motifs:
        if m.motif_type == "co_gated_stack_mirror":
            for s in m.roles.get("stacks", []):
                if isinstance(s, list):
                    for r in s:
                        stack_by_ref[r] = {"refs": s, "is_diode_master": False}
    diode_refs = {
        inst.ref
        for inst in circuit.instances
        if is_diode_connected(inst, rails, device_roles, stack_by_ref)
    }

    _schema, modules = _resolve_plan_modules(plan_data, analysis)
    modules.sort(key=lambda m: (m["stage_order"], m["id"]))

    groups: list[FunctionalGroupSpec] = []
    intent_motifs: list[dict[str, Any]] = []
    orientations: dict[str, str] = {}
    tier_overrides: dict[str, int] = {}
    column_hints: list[list[str]] = []

    draft_intent_dict = analysis.draft_intent or {}
    draft_orients: dict[str, str] = dict(draft_intent_dict.get("orientations", {}))
    draft_tiers: dict[str, int] = {str(k): int(v) for k, v in draft_intent_dict.get("tierOverrides", {}).items()}
    draft_cols: list[list[str]] = [[str(x) for x in c] for c in draft_intent_dict.get("columnOrderHints", [])]
    draft_intent_motifs: list[dict[str, Any]] = list(draft_intent_dict.get("motifs", []))
    skeleton_cols_by_mod: dict[str, list[list[str]]] = {}
    skeleton_refs_all: set[str] = set()
    mod_of_ref_exp: dict[str, str] = {}
    mod_insts_by_id: dict[str, list[str]] = {}

    def _motif_refs(m: dict[str, Any]) -> set[str]:
        refs: set[str] = set()
        for key in ("left", "right", "tail", "loadLeft", "loadRight"):
            if m.get(key):
                refs.add(str(m[key]))
        for key in ("instances", "leftColumn", "rightColumn", "stackedDrivers"):
            refs.update(str(r) for r in (m.get(key) or []))
        for key in ("leftColumns", "rightColumns"):
            for col in m.get(key) or []:
                refs.update(str(r) for r in col)
        for pair in m.get("pairs") or []:
            refs.update(str(r) for r in pair)
        return refs

    for mod in modules:
        mod_insts = [r for r in mod["instances"] if r in inst_by_ref]
        if not mod_insts:
            continue
        mod_inst_set = set(mod_insts)
        groups.append(
            FunctionalGroupSpec(
                id=mod["id"],
                label=mod["label"],
                role=mod["role"],
                stage_order=mod["stage_order"],
                instances=list(mod_insts),
            )
        )

        # Build ordered branch columns belonging to this module
        def _cols_for_branches(bids: list[str]) -> list[list[str]]:
            out_c: list[list[str]] = []
            for bid in bids:
                if bid in branch_by_id:
                    c = [r for r in branch_by_id[bid]["instances"] if r in mod_inst_set]
                    if c:
                        out_c.append(c)
            return out_c

        skel = mod.get("skeleton") or {}
        stype = str(skel.get("type", ""))
        placed_in_mod: set[str] = set()
        mod_cols: list[list[str]] = []

        if stype == "mirror_pair":
            l_brs = [str(b) for b in skel.get("leftBranches", [])]
            c_brs = [str(b) for b in skel.get("centerBranches", [])]
            r_brs = [str(b) for b in skel.get("rightBranches", [])]
            l_cols = _cols_for_branches(l_brs)
            c_cols = _cols_for_branches(c_brs)
            r_cols = _cols_for_branches(r_brs)
            # Align paired columns left[0] <-> right[0] (outermost) .. left[-1] <-> right[-1] (innermost)
            pairs_all: list[list[str]] = []
            if isinstance(skel.get("pairs"), list) and skel["pairs"]:
                pairs_all = [[str(p[0]), str(p[1])] for p in skel["pairs"] if len(p) == 2]
            else:
                for lb, rb in zip(l_brs, r_brs):
                    if lb in branch_by_id and rb in branch_by_id:
                        pairs_all.extend(
                            _align_quasi_mirror_pairs(
                                [r for r in branch_by_id[lb]["instances"] if r in mod_inst_set],
                                [r for r in branch_by_id[rb]["instances"] if r in mod_inst_set],
                                branch_by_id[lb]["tokens"],
                                branch_by_id[rb]["tokens"],
                            )
                        )
            # Emit columns outer-left -> inner-left -> center -> inner-right -> outer-right
            ordered_r_cols = list(reversed(r_cols))
            for col in l_cols:
                mod_cols.append(col)
                placed_in_mod.update(col)
                for r in col:
                    inst = inst_by_ref[r]
                    if inst.device_class in {"mos", "bjt"}:
                        orientations.setdefault(r, "gate_left")
                    else:
                        orientations.setdefault(r, "vertical")
            for col in c_cols:
                mod_cols.append(col)
                placed_in_mod.update(col)
                for r in col:
                    inst = inst_by_ref[r]
                    orientations.setdefault(r, "gate_left" if inst.device_class in {"mos", "bjt"} else "vertical")
            for col in ordered_r_cols:
                mod_cols.append(col)
                placed_in_mod.update(col)
                for r in col:
                    inst = inst_by_ref[r]
                    if inst.device_class in {"mos", "bjt"}:
                        orientations.setdefault(r, "gate_right")
                    else:
                        orientations.setdefault(r, "vertical")

            # Face inner common-gate / common-base mirrored top devices toward center axis
            for rl, rr in pairs_all:
                if rl in inst_by_ref and rr in inst_by_ref:
                    il, ir = inst_by_ref[rl], inst_by_ref[rr]
                    cpl = ctrl_pin_of(il, device_roles)
                    cpr = ctrl_pin_of(ir, device_roles)
                    if cpl and cpr and il.terminals.get(cpl) == ir.terminals.get(cpr):
                        # Common-control mirror pair: face each other if inner, or mirrored cleanly
                        if rl in (l_cols[-1] if l_cols else []) and rr in (r_cols[-1] if r_cols else []):
                            orientations[rl] = "gate_right"
                            orientations[rr] = "gate_left"

            if l_cols and r_cols:
                intent_motifs.append(
                    {
                        "type": "mirror_block_pair",
                        "id": f"mirror_{mod['id']}",
                        "leftColumns": l_cols,
                        "rightColumns": r_cols,
                        "pairs": pairs_all,
                    }
                )

        elif stype == "bias_bank":
            ref_brs = [str(b) for b in skel.get("referenceBranches", [])]
            mir_brs = [str(b) for b in skel.get("mirrorBranches", [])]
            for col in _cols_for_branches(ref_brs):
                mod_cols.append(col)
                placed_in_mod.update(col)
                for r in col:
                    inst = inst_by_ref[r]
                    orientations.setdefault(r, "gate_right" if inst.device_class in {"mos", "bjt"} else "vertical")
            for col in _cols_for_branches(mir_brs):
                mod_cols.append(col)
                placed_in_mod.update(col)
                for r in col:
                    inst = inst_by_ref[r]
                    orientations.setdefault(r, "gate_left" if inst.device_class in {"mos", "bjt"} else "vertical")

        elif stype == "parallel_array":
            arr_brs = [str(b) for b in skel.get("branches", mod["touchedBranches"])]
            for col in _cols_for_branches(arr_brs):
                mod_cols.append(col)
                placed_in_mod.update(col)
                for r in col:
                    inst = inst_by_ref[r]
                    orientations.setdefault(r, "gate_left" if inst.device_class in {"mos", "bjt"} else "vertical")
            # Emit `parallel_bank` motif if 1-device passive array (e.g. CDAC)
            if mod_cols and all(len(c) == 1 and inst_by_ref[c[0]].device_class.startswith(("cap", "res")) for c in mod_cols):
                arr_insts = [c[0] for c in mod_cols]
                intent_motifs.append(
                    {
                        "type": "parallel_bank",
                        "id": f"bank_{mod['id']}",
                        "instances": arr_insts,
                        "commonNet": str(skel.get("commonNet", "")),
                    }
                )

        elif stype == "horizontal_bridge":
            h_insts = [str(r) for r in skel.get("instances", mod_insts) if str(r) in mod_inst_set]
            for r in h_insts:
                mod_cols.append([r])
                placed_in_mod.add(r)
                inst = inst_by_ref[r]
                orientations.setdefault(r, "horizontal_gate_down" if inst.device_class == "mos" else "horizontal_l2r")

        # Inherit every analyzer-recognized intent motif (diff_pair, sandwich/mirror cores, horizontal Miller
        # bridges, symmetric columns, arrays...) fully contained in this module and not overridden by a skeleton.
        skeleton_refs = set(placed_in_mod)
        for dm in draft_intent_motifs:
            m_refs = _motif_refs(dm)
            if m_refs and m_refs <= mod_inst_set and not (m_refs & skeleton_refs):
                intent_motifs.append(dict(dm))

        # Columns are emitted globally after all modules are processed (see below) so that the analyzer's
        # deliberate cross-group column folds survive intact.
        skeleton_cols_by_mod[mod["id"]] = [list(c) for c in mod_cols]
        skeleton_refs_all.update(skeleton_refs)
        for r in mod_insts:
            mod_of_ref_exp[r] = mod["id"]
        mod_insts_by_id[mod["id"]] = list(mod_insts)

    # Global column emission: draft column order is preserved verbatim (including columns that fold devices of
    # two adjacent modules); a module's skeleton columns are inserted where that module first appears.
    emitted_cols: list[tuple[list[str], bool]] = []
    emitted_skel_mods: set[str] = set()
    placed_global: set[str] = set(skeleton_refs_all)

    def _emit_skeleton_of(mod_id: str | None) -> None:
        if mod_id and mod_id in skeleton_cols_by_mod and mod_id not in emitted_skel_mods:
            emitted_skel_mods.add(mod_id)
            for sc in skeleton_cols_by_mod[mod_id]:
                emitted_cols.append((sc, True))

    for dcol in draft_cols:
        for r in dcol:
            _emit_skeleton_of(mod_of_ref_exp.get(r))
        c = [r for r in dcol if r in mod_of_ref_exp and r not in placed_global]
        if c:
            emitted_cols.append((c, False))
            placed_global.update(c)
    for mod_id in skeleton_cols_by_mod:
        _emit_skeleton_of(mod_id)

    # Devices the draft never placed fall back to their folded vertical branch, per module
    for mod_id, m_insts in mod_insts_by_id.items():
        m_set = set(m_insts)
        leftovers = [r for r in m_insts if r not in placed_global]
        if not leftovers:
            continue
        extra_cols: list[list[str]] = []
        seen_rem_br: set[str] = set()
        for r in leftovers:
            if r in placed_global:
                continue
            bid = branch_of_ref.get(r)
            if bid and bid not in seen_rem_br:
                seen_rem_br.add(bid)
                c = [x for x in branch_by_id[bid]["instances"] if x in m_set and x not in placed_global]
            else:
                c = [r]
            if c:
                extra_cols.append(c)
                placed_global.update(c)
        extra_cols = order_columns_and_orient_gates(
            extra_cols,
            [c for c, _ in emitted_cols],
            inst_by_ref,
            fanout,
            rails,
            diode_refs,
            circuit,
            orientations,
        )
        emitted_cols.extend((c, False) for c in extra_cols)

    for col, is_skeleton_col in emitted_cols:
        column_hints.append(col)
        branch_tiers = _assign_branch_tiers(col, inst_by_ref, default_tiers) if is_skeleton_col else {}
        for r in col:
            if r in draft_tiers and not is_skeleton_col:
                tier_overrides.setdefault(r, draft_tiers[r])
            elif r in branch_tiers:
                tier_overrides.setdefault(r, branch_tiers[r])
            elif r in draft_tiers:
                tier_overrides.setdefault(r, draft_tiers[r])
            inst = inst_by_ref[r]
            orientations.setdefault(
                r,
                draft_orients.get(
                    r, "gate_left" if inst.device_class in {"mos", "bjt"} else "vertical"
                ),
            )

    # Net policies: inherit the analyzer's draft policies & critical nets (cross-stage buses already resolved),
    # force supply rails local, then apply plan-level overrides. Provenance is recorded so the crossing
    # optimizer can never overwrite an LLM decision (priority: llm > skeleton > optimizer > analyzer).
    net_policies: dict[str, str] = {
        str(k): str(v) for k, v in (draft_intent_dict.get("netPolicies") or {}).items()
    }
    net_policy_sources: dict[str, str] = {k: "analyzer" for k in net_policies}
    critical_nets: list[str] = [str(n) for n in draft_intent_dict.get("criticalNets", [])]
    for net in circuit.nets:
        if net in rails:
            net_policies[net] = "supply_local"
            net_policy_sources[net] = "analyzer"
        elif net not in net_policies:
            net_policies[net] = "perimeter_bus" if net in global_bias_nets else "full_wire"
            net_policy_sources[net] = "skeleton" if net in global_bias_nets else "analyzer"

    for net, pol in (plan_data.get("netPolicies") or {}).items():
        net_policies[str(net)] = str(pol)
        net_policy_sources[str(net)] = "llm"

    # Port sides: start from analyzer defaults, apply plan overrides
    port_sides = dict(draft_intent_dict.get("portSides", {}))
    for pname, pside in (plan_data.get("portSides") or {}).items():
        port_sides[str(pname)] = str(pside)

    # Apply Fine-stage overrides (only the exceptions LLM wants to tweak!)
    overrides = plan_data.get("overrides") or {}
    for r, ori in (overrides.get("orientations") or {}).items():
        orientations[str(r)] = str(ori)
    for r, tv in (overrides.get("tierOverrides") or {}).items():
        tier_overrides[str(r)] = int(tv)
    for net, pol in (overrides.get("netPolicies") or {}).items():
        net_policies[str(net)] = str(pol)
        net_policy_sources[str(net)] = "llm"
    for pname, pside in (overrides.get("portSides") or {}).items():
        port_sides[str(pname)] = str(pside)
    if overrides.get("columnOrderHints"):
        column_hints = [[str(x) for x in col] for col in overrides["columnOrderHints"]]
    if overrides.get("motifs"):
        intent_motifs.extend(list(overrides["motifs"]))

    spacing = SpacingConfig.from_dict(overrides.get("spacing") or plan_data.get("spacing"))

    return SchematicIntent(
        cell_name=str(plan_data.get("cellName") or circuit.cell_name),
        groups=groups,
        motifs=intent_motifs,
        orientations=orientations,
        tier_overrides=tier_overrides,
        column_order_hints=column_hints,
        net_policies=net_policies,
        critical_nets=critical_nets,
        port_sides=port_sides,
        spacing=spacing,
        authored_by=str(plan_data.get("authoredBy", "module_plan_expanded")),
        llm_notes=str(plan_data.get("llmNotes", "Expanded deterministically from Coarse-to-Fine module_plan.v1")),
        net_policy_sources=net_policy_sources,
    )


def inspect_netlist_neighborhood(
    circuit: NetlistCircuit,
    analysis: TopologyAnalysis | None = None,
    *,
    around: str | None = None,
    hops: int = 2,
    include_bias: bool = False,
    module_targets: list[str] | None = None,
) -> dict[str, Any]:
    """Interactive read-only local signal tracer (`--around`) or Stage-2 Meso Focus Window (`--module`)."""
    if analysis is None:
        analysis = analyze_circuit(circuit)
    ba = analysis.branch_abstraction or {}
    branches = ba.get("branches", [])
    branch_by_id = {b["id"]: b for b in branches}
    branch_of_ref = ba.get("branchOfRef", {})
    global_bias_nets = set(ba.get("globalBiasNets", []))
    rails = set(circuit.supply_nets) | set(circuit.ground_nets)
    inst_by_ref = circuit.instance_map()

    # Build branch adjacency (stripping rails + optionally global bias rails)
    skip_nets = set(rails) | (set() if include_bias else global_bias_nets)
    branch_adj: dict[str, list[tuple[str, str]]] = defaultdict(list)
    net_to_branches: dict[str, set[str]] = defaultdict(set)
    for b in branches:
        bid = b["id"]
        for r in b["instances"]:
            for p, n in inst_by_ref[r].terminals.items():
                if p != "B" and n not in rails:
                    net_to_branches[n].add(bid)

    for n, bset in net_to_branches.items():
        if n in skip_nets or len(bset) < 2:
            continue
        blist = sorted(bset)
        for i_u, u in enumerate(blist):
            for v in blist[i_u + 1 :]:
                branch_adj[u].append((v, n))
                branch_adj[v].append((u, n))

    # --- Mode B: Stage-2 Meso Focus Window (`--module`) ---
    if module_targets:
        focus_brs: set[str] = set()
        focus_refs: set[str] = set()
        for tok in module_targets:
            t = tok.strip()
            if t in branch_by_id:
                focus_brs.add(t)
                focus_refs.update(branch_by_id[t]["instances"])
            elif t in inst_by_ref:
                focus_refs.add(t)
                if t in branch_of_ref:
                    focus_brs.add(branch_of_ref[t])

        internal_nets: list[dict[str, Any]] = []
        external_signals: list[dict[str, Any]] = []
        external_bias: list[dict[str, Any]] = []

        touched_nets = sorted(
            {
                n
                for r in focus_refs
                for p, n in inst_by_ref[r].terminals.items()
                if p != "B" and n not in rails
            }
        )
        for n in touched_nets:
            in_conns = [
                f"{r}.{p}"
                for r in sorted(focus_refs, key=_ref_num)
                for p, tn in inst_by_ref[r].terminals.items()
                if p != "B" and tn == n
            ]
            ext_brs = sorted(net_to_branches.get(n, set()) - focus_brs)
            entry = {
                "net": n,
                "internalPins": in_conns,
                "externalBranches": ext_brs,
                "isGlobalBias": n in global_bias_nets,
            }
            if not ext_brs:
                internal_nets.append(entry)
            elif n in global_bias_nets:
                external_bias.append(entry)
            else:
                external_signals.append(entry)

        # Surface any quasi-symmetric pairs inside this focus window
        local_qpairs = [
            qp
            for qp in ba.get("quasiSymmetricPairs", [])
            if set(qp["leftBranches"] + qp["rightBranches"]) <= focus_brs
        ]
        device_table = [
            {
                "ref": r,
                "branch": branch_of_ref.get(r, ""),
                "deviceClass": inst_by_ref[r].device_class,
                "polarity": inst_by_ref[r].polarity,
                "terminals": {p: n for p, n in inst_by_ref[r].terminals.items() if p != "B"},
                "params": dict(inst_by_ref[r].params),
            }
            for r in sorted(focus_refs, key=_ref_num)
        ]
        return {
            "mode": "module_focus",
            "cellName": circuit.cell_name,
            "branches": [branch_by_id[b] for b in sorted(focus_brs)],
            "devices": device_table,
            "internalNets": internal_nets,
            "externalSignalNets": external_signals,
            "externalBiasNets": external_bias,
            "localQuasiSymmetricPairs": local_qpairs,
        }

    # --- Mode A: k-hop Local Signal Neighborhood Tracer (`--around`) ---
    seed_branches: list[str] = []
    seed_desc = str(around or "")
    if around in branch_by_id:
        seed_branches = [around]
    elif around in inst_by_ref:
        bid = branch_of_ref.get(around)
        if bid:
            seed_branches = [bid]
    elif around in net_to_branches:
        seed_branches = sorted(net_to_branches[around])
    else:
        raise ValueError(f"Target {around!r} not found as branch ID, device ref, or net name in {circuit.cell_name}.")

    dist: dict[str, int] = {b: 0 for b in seed_branches}
    q = deque(seed_branches)
    traversed_edges: list[dict[str, str]] = []
    seen_te: set[tuple[str, str, str]] = set()

    while q:
        u = q.popleft()
        d_u = dist[u]
        if d_u >= max(0, hops):
            continue
        for v, n in branch_adj.get(u, []):
            ek = (min(u, v), max(u, v), n)
            if ek not in seen_te:
                seen_te.add(ek)
                traversed_edges.append({"from": u, "to": v, "net": n})
            if v not in dist:
                dist[v] = d_u + 1
                q.append(v)

    reached = [
        {"hop": dist[bid], **branch_by_id[bid]}
        for bid in sorted(dist.keys(), key=lambda x: (dist[x], x))
    ]
    hidden_bias_on_reached = sorted(
        {
            c["net"]
            for b in reached
            for c in b.get("ctrlInputs", [])
            if c["net"] in global_bias_nets
        }
    )
    return {
        "mode": "neighborhood_trace",
        "cellName": circuit.cell_name,
        "seed": seed_desc,
        "hops": hops,
        "includeBias": include_bias,
        "reachedBranches": reached,
        "edges": traversed_edges,
        "hiddenGlobalBiasNets": hidden_bias_on_reached if not include_bias else [],
    }


def format_inspect_markdown(data: dict[str, Any]) -> str:
    """Format `inspect_netlist_neighborhood` output into compact LLM-ready Markdown."""
    lines: list[str] = []
    if data["mode"] == "module_focus":
        lines.append(f"# Meso 局部聚焦视窗 · `{data['cellName']}`")
        lines.append("")
        lines.append("## 1. 模块内支路与器件明细")
        lines.append("")
        for b in data["branches"]:
            lines.append(f"- **`{b['id']}`** (`{b['signature']}`, {b.get('topRail') or '·'}→{b.get('bottomRail') or '·'}): `{' → '.join(b['instances'])}`")
        lines.append("")
        lines.append("| 器件 | 支路 | 类型/极性 | 端子连接（不含 B） | 关键参数 |")
        lines.append("| :--- | :---: | :--- | :--- | :--- |")
        for d in data["devices"]:
            t_str = ", ".join(f"{p}={n}" for p, n in d["terminals"].items())
            p_str = ", ".join(f"{k}={v}" for k, v in d["params"].items() if k in {"w", "l", "m", "r", "c", "dc"}) or "-"
            lines.append(f"| `{d['ref']}` | `{d['branch']}` | `{d['deviceClass']}/{d['polarity']}` | `{t_str}` | `{p_str}` |")
        lines.append("")
        lines.append("## 2. 模块对外信号接口 vs 内部私有网")
        lines.append("")
        if data["externalSignalNets"]:
            lines.append("**对外非偏置信号接口**：")
            for e in data["externalSignalNets"]:
                lines.append(f"- `{e['net']}`: 内部 `{', '.join(e['internalPins'])}` ↔ 外部支路 `{', '.join(e['externalBranches'])}`")
            lines.append("")
        if data["externalBiasNets"]:
            lines.append("**接入的全局偏置轨**：")
            for e in data["externalBiasNets"]:
                lines.append(f"- `{e['net']}` [GLOBAL_BIAS]: 内部 `{', '.join(e['internalPins'])}`")
            lines.append("")
        if data["internalNets"]:
            lines.append("**模块内部私有网**：")
            for e in data["internalNets"]:
                lines.append(f"- `{e['net']}`: `{', '.join(e['internalPins'])}`")
            lines.append("")
        if data["localQuasiSymmetricPairs"]:
            lines.append("## 3. 模块内准对称候选")
            lines.append("")
            for qp in data["localQuasiSymmetricPairs"]:
                lines.append(f"- **`{qp['id']}`** (相似度={qp['similarity']}): {qp['reason']}")
            lines.append("")
        return "\n".join(lines)

    # Neighborhood trace mode
    lines.append(f"# 局部信号邻域追踪 · `{data['cellName']}` (`around={data['seed']}`, `hops={data['hops']}`)")
    lines.append("")
    if data.get("hiddenGlobalBiasNets"):
        lines.append(f"> 已自动隐藏全局偏置轨 `{', '.join(data['hiddenGlobalBiasNets'])}`（如需显示请加 `--include-bias`）。")
        lines.append("")
    lines.append("| 跳数 | 支路 | 顶→底器件序列 | 结构签名 | 控制端 | 对外信号端 |")
    lines.append("| :---: | :--- | :--- | :--- | :--- | :--- |")
    for b in data["reachedBranches"]:
        ctrls = ", ".join(dict.fromkeys(c["net"] for c in b.get("ctrlInputs", []))) or "-"
        taps = ", ".join(dict.fromkeys(t["net"] for t in b.get("signalTaps", []))) or "-"
        lines.append(f"| `{b['hop']}` | **`{b['id']}`** | `{' → '.join(b['instances'])}` | `{b['signature']}` | `{ctrls}` | `{taps}` |")
    lines.append("")
    if data["edges"]:
        lines.append("**连通信号边**：")
        for e in data["edges"]:
            lines.append(f"- `{e['from']}` —`{e['net']}`— `{e['to']}`")
        lines.append("")
    return "\n".join(lines)
