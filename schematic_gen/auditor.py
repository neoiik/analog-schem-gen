"""Dual Electrical Equivalence (LVS + schCheck continuity) and Readability DRC Auditor."""

from __future__ import annotations

from collections import Counter, defaultdict
from typing import Any

from schematic_gen.manifest_builder import (
    build_bridge_manifest,
    verify_with_virtuoso_bridge_offline,
)
from schematic_gen.models import (
    AuditFinding,
    AuditReport,
    BBox,
    NetlistCircuit,
    Point,
    SolvedSchematic,
)
from schematic_gen.pdk import PdkRegistry


def _boxes_overlap_strict(a: BBox, b: BBox) -> bool:
    """Return True if two axis-aligned boxes overlap with positive area or shared interior."""
    return min(a[2], b[2]) > max(a[0], b[0]) and min(a[3], b[3]) > max(a[1], b[1])


def _seg_points(a: Point, b: Point) -> list[Point]:
    """Enumerate integer grid points along an orthogonal segment."""
    if a[0] == b[0]:
        return [(a[0], y) for y in range(min(a[1], b[1]), max(a[1], b[1]) + 1)]
    if a[1] == b[1]:
        return [(x, a[1]) for x in range(min(a[0], b[0]), max(a[0], b[0]) + 1)]
    return [a, b]


def audit_solved_schematic(
    solved: SolvedSchematic,
    pdk: PdkRegistry | None = None,
) -> AuditReport:
    """Run complete Electrical LVS + Virtuoso Bridge Offline Check + Readability DRC."""
    pdk = pdk or PdkRegistry()
    manifest = build_bridge_manifest(solved, pdk)
    return audit_manifest_and_solved(
        manifest_data=manifest,
        circuit=solved.circuit,
        pdk=pdk,
        solved_hint=solved,
    )


def audit_manifest_and_solved(
    manifest_data: dict[str, Any],
    circuit: NetlistCircuit,
    pdk: PdkRegistry | None = None,
    solved_hint: SolvedSchematic | None = None,
) -> AuditReport:
    """Audit any `virtuoso-bridge-exact-schematic-v1` manifest against reference `NetlistCircuit`."""
    pdk = pdk or PdkRegistry()
    findings: list[AuditFinding] = []
    suggestions: list[str] = []

    # 1. Virtuoso Bridge Offline Exact-Geometry & Schema Preflight
    bridge_res = verify_with_virtuoso_bridge_offline(
        manifest_data, pdk.raw_map, pdk.process_name
    )
    bridge_ok = bool(bridge_res.get("passed", False))
    if not bridge_ok:
        findings.append(
            AuditFinding(
                severity="ERROR",
                code="BRIDGE_MANIFEST_INVALID",
                message=f"Virtuoso Bridge 清单/精确几何校验未通过: {bridge_res.get('error')}",
            )
        )

    m_circuit = (manifest_data.get("circuits") or [{}])[0]
    m_insts = {str(item["reference"]): item for item in m_circuit.get("instances", [])}
    ref_insts = circuit.instance_map()

    # 2. Electrical LVS Check (Instances, Parameters, Terminal-to-Net Partition, Physical Wire Continuity)
    elec_errors = 0
    if set(m_insts) != set(ref_insts):
        elec_errors += 1
        missing = sorted(set(ref_insts) - set(m_insts))
        extra = sorted(set(m_insts) - set(ref_insts))
        findings.append(
            AuditFinding(
                severity="ERROR",
                code="LVS_INSTANCE_MISMATCH",
                message=f"实例集合不一致：缺失 {missing}，多余 {extra}",
                subjects=missing + extra,
            )
        )

    for ref, ref_inst in ref_insts.items():
        if ref not in m_insts:
            continue
        m_item = m_insts[ref]
        m_terms = {
            str(n["sourcePinName"]): str(n["netName"]) for n in m_item.get("nodes", [])
        }
        if m_terms != ref_inst.terminals:
            elec_errors += 1
            findings.append(
                AuditFinding(
                    severity="ERROR",
                    code="LVS_TERMINAL_NET_MISMATCH",
                    message=f"{ref} 端子网络连接不一致：期望 {ref_inst.terminals}，实际 {m_terms}",
                    subjects=[ref],
                )
            )
        m_params = {str(k): str(v) for k, v in (m_item.get("sourceParameters") or {}).items()}
        for pk, pv in ref_inst.params.items():
            if m_params.get(pk) != str(pv):
                elec_errors += 1
                findings.append(
                    AuditFinding(
                        severity="ERROR",
                        code="LVS_PARAM_MISMATCH",
                        message=f"{ref} 参数 {pk} 不一致：期望 {pv!r}，实际 {m_params.get(pk)!r}",
                        subjects=[ref],
                    )
                )

    # Physical wire continuity per net (`schCheck` split-net prevention!)
    geom = m_circuit.get("sourceGeometry", {})
    port_names = {str(p["name"]): str(p["netName"]) for p in m_circuit.get("ports", [])}
    supplies_and_ports = set(port_names.values())

    # Reconstruct pin coordinates & bounding boxes from manifest + PDK
    inst_boxes: dict[str, BBox] = {}
    label_boxes: dict[str, BBox] = {}
    pin_coords: dict[tuple[str, str], Point] = {}
    pin_owner: dict[Point, tuple[str, str, str]] = {}  # pt -> (net, ref, pin)

    bulk_label_refs = {str(x["reference"]): str(x["netName"]) for x in geom.get("localBulkLabels", [])}

    for ref, m_item in m_insts.items():
        pos = m_item["sourcePosition"]
        px, py = int(round(float(pos["x"]))), int(round(float(pos["y"])))
        tr = m_item.get("sourceTransform") or {}
        rot = int(round(float(tr.get("rotation", 0))))
        mir = str(tr.get("mirror", "none"))
        dev_key = str(
            m_item["kind"] if m_item.get("deviceClass") == "mos" else m_item["deviceClass"]
        )
        rel_p = pdk.source_pin_offsets(dev_key, rot, mir)
        rel_b = pdk.source_body_box(dev_key, rot, mir)
        inst_boxes[ref] = (px + rel_b[0], py + rel_b[1], px + rel_b[2], py + rel_b[3])

        if ref in ref_insts:
            rel_l, _, _ = pdk.relative_label_and_envelope(
                ref_insts[ref], rot, mir, has_bulk_label_stub=(ref in bulk_label_refs)
            )
            label_boxes[ref] = (px + rel_l[0], py + rel_l[1], px + rel_l[2], py + rel_l[3])
        else:
            label_boxes[ref] = (px + rel_b[2] + 1, py - 2, px + rel_b[2] + 6, py + 2)

        for node in m_item.get("nodes", []):
            cpin = str(node["sourcePinName"])
            net = str(node["netName"])
            dx, dy = rel_p[cpin]
            pt = (px + dx, py + dy)
            pin_coords[(ref, cpin)] = pt
            pin_owner[pt] = (net, ref, cpin)

    port_occ_pts: dict[str, tuple[str, Point]] = {}
    for po in geom.get("portOccurrences", []):
        oid = str(po["occurrenceId"])
        pos = po["sourcePosition"]
        pt = (int(round(float(pos["x"]))), int(round(float(pos["y"]))))
        net = str(po["netName"])
        port_occ_pts[oid] = (net, pt)
        pin_owner[pt] = (net, oid, str(po["name"]))

    junc_pts: dict[str, tuple[str, Point]] = {}
    for j in geom.get("junctions", []):
        jid = str(j["id"])
        pos = j["sourcePosition"]
        pt = (int(round(float(pos["x"]))), int(round(float(pos["y"]))))
        junc_pts[jid] = (str(j["netName"]), pt)

    # Extract all orthogonal wire segments per net
    segments_by_net: dict[str, list[tuple[Point, Point]]] = defaultdict(list)
    bends_and_juncs: list[tuple[str, Point]] = [
        (net, pt) for net, pt in junc_pts.values()
    ]

    for rt in geom.get("routes", []):
        net = str(rt["netName"])
        st = rt["start"]
        sp = st["sourcePoint"]
        pts: list[Point] = [(int(round(float(sp["x"]))), int(round(float(sp["y"]))))]
        for step in rt.get("steps", []):
            if step.get("kind") == "bend":
                bp = step["position"]
                bpt = (int(round(float(bp["x"]))), int(round(float(bp["y"]))))
                pts.append(bpt)
                bends_and_juncs.append((net, bpt))
            else:
                ep = step["sourcePoint"]
                pts.append((int(round(float(ep["x"]))), int(round(float(ep["y"])))))
        for a, b in zip(pts, pts[1:]):
            if a != b:
                segments_by_net[net].append((a, b))

    for bs in geom.get("localBulkShorts", []):
        ref = str(bs["reference"])
        net = str(bs["netName"])
        p_b = pin_coords[(ref, str(bs["bulkPinName"]))]
        p_s = pin_coords[(ref, str(bs["sourcePinName"]))]
        segments_by_net[net].append((p_b, p_s))

    for bl in geom.get("localBulkLabels", []):
        ref = str(bl["reference"])
        net = str(bl["netName"])
        p_b = pin_coords[(ref, str(bl["pinName"]))]
        mx = int(round(float(m_insts[ref]["sourcePosition"]["x"])))
        dx = 1 if p_b[0] >= mx else -1
        p_end = (p_b[0] + dx * pdk.local_stub_len, p_b[1])
        segments_by_net[net].append((p_b, p_end))
        port_occ_pts[f"BULK_{ref}"] = (net, p_end)

    # Include explicit wire-label fly-wire stubs when auditing a `SolvedSchematic` with `allow_wire_labels=True`
    wire_label_endpoints: dict[str, list[Point]] = defaultdict(list)
    if solved_hint is not None:
        for wl in solved_hint.wire_label_stubs:
            p1, p2 = (wl.x1, wl.y1), (wl.x2, wl.y2)
            segments_by_net[wl.net_name].append((p1, p2))
            wire_label_endpoints[wl.net_name].append(p2)

    # Check physical graph connectivity per net
    for net in circuit.nets:
        net_adj: dict[Point, set[Point]] = defaultdict(set)
        for a, b in segments_by_net.get(net, []):
            pts_line = _seg_points(a, b)
            for u, v in zip(pts_line, pts_line[1:]):
                net_adj[u].add(v)
                net_adj[v].add(u)
        # Same-named explicit WireLabelStubs merge logically (just like Virtuoso `schCreateWireLabel`)
        wl_pts = wire_label_endpoints.get(net, [])
        for u, v in zip(wl_pts, wl_pts[1:]):
            net_adj[u].add(v)
            net_adj[v].add(u)

        req_pins = [
            pin_coords[(inst.ref, pin)]
            for inst in circuit.instances
            for pin, pnet in inst.terminals.items()
            if pnet == net and (inst.ref in m_insts)
        ]
        net_ports = [pt for pnet, pt in port_occ_pts.values() if pnet == net]

        if len(req_pins) + len(net_ports) <= 1:
            continue

        if net in supplies_and_ports and len(net_ports) >= 2:
            # Repeated top-level port: every terminal must reach at least one port occurrence of `net`!
            port_set = set(net_ports)
            for rp in req_pins:
                seen = {rp}
                q = [rp]
                reached_port = rp in port_set
                for cur in q:
                    if cur in port_set:
                        reached_port = True
                        break
                    for nxt in net_adj.get(cur, set()):
                        if nxt not in seen:
                            seen.add(nxt)
                            q.append(nxt)
                if not reached_port:
                    elec_errors += 1
                    findings.append(
                        AuditFinding(
                            severity="ERROR",
                            code="DISCONNECTED_PORT_NET_PIN",
                            message=f"网络 {net!r} 上的端子 {rp} 未连接到任何 {net!r} 端口符号",
                            subjects=[net],
                        )
                    )
        else:
            # Internal net (or single-port net): ALL terminals & ports must form ONE connected wire tree!
            targets = set(req_pins) | set(net_ports)
            root = next(iter(targets))
            seen = {root}
            q = [root]
            for cur in q:
                for nxt in net_adj.get(cur, set()):
                    if nxt not in seen:
                        seen.add(nxt)
                        q.append(nxt)
            missing_pts = targets - seen
            if missing_pts:
                elec_errors += 1
                findings.append(
                    AuditFinding(
                        severity="ERROR",
                        code="SPLIT_INTERNAL_NET",
                        message=f"内部网络 {net!r} 导线未完全连通（Virtuoso schCheck 将拆网报错），孤立端点: {sorted(missing_pts)}",
                        subjects=[net],
                    )
                )

    electrical_passed = (elec_errors == 0) and bridge_ok

    # 3. Readability Geometric DRC Checks
    symbol_overlaps = 0
    label_overlaps = 0
    wire_through_body = 0
    wire_over_foreign_pin = 0
    collinear_foreign_overlaps = 0
    ambiguous_crossings = 0
    parallel_crowds = 0

    refs_sorted = sorted(inst_boxes)
    for i, r1 in enumerate(refs_sorted):
        for r2 in refs_sorted[i + 1 :]:
            if _boxes_overlap_strict(inst_boxes[r1], inst_boxes[r2]):
                symbol_overlaps += 1
                findings.append(
                    AuditFinding(
                        severity="ERROR",
                        code="SYMBOL_OVERLAP",
                        message=f"器件符号重叠：{r1} 与 {r2}",
                        subjects=[r1, r2],
                    )
                )
            if _boxes_overlap_strict(label_boxes[r1], label_boxes[r2]) or _boxes_overlap_strict(
                label_boxes[r1], inst_boxes[r2]
            ) or _boxes_overlap_strict(inst_boxes[r1], label_boxes[r2]):
                label_overlaps += 1
                findings.append(
                    AuditFinding(
                        severity="ERROR",
                        code="LABEL_SYMBOL_OVERLAP",
                        message=f"参数文字框与相邻器件/文字重叠：{r1} 与 {r2}",
                        subjects=[r1, r2],
                    )
                )

    # Grid-level wire DRC
    h_grid: dict[Point, set[str]] = defaultdict(set)
    v_grid: dict[Point, set[str]] = defaultdict(set)
    total_wire_len = 0

    for net, segs in segments_by_net.items():
        for a, b in segs:
            total_wire_len += abs(b[0] - a[0]) + abs(b[1] - a[1])
            pts_line = _seg_points(a, b)
            is_h = a[1] == b[1]
            for pt in pts_line:
                if is_h:
                    h_grid[pt].add(net)
                else:
                    v_grid[pt].add(net)

                # Foreign pin collision?
                p_info = pin_owner.get(pt)
                if p_info is not None and p_info[0] != net:
                    wire_over_foreign_pin += 1
                    findings.append(
                        AuditFinding(
                            severity="ERROR",
                            code="WIRE_OVER_FOREIGN_PIN",
                            message=f"网络 {net!r} 导线擦过异网端子 {p_info[1]}.{p_info[2]} ({p_info[0]}) @ {pt}",
                            subjects=[net, p_info[1]],
                        )
                    )

                # Interior symbol body collision?
                for ref, bbox in inst_boxes.items():
                    if bbox[0] < pt[0] < bbox[2] and bbox[1] < pt[1] < bbox[3]:
                        # Allow own bulk pin at (px+1, py) inside [-3..2, -3..3] ONLY for that instance's own terminal net!
                        own_pins = {
                            pin_coords[(ref, cp)]
                            for cp, cnet in ref_insts[ref].terminals.items()
                            if cnet == net
                        } if ref in ref_insts else set()
                        # Allow short path from own interior B pin straight out
                        if pt in own_pins:
                            continue
                        if any(
                            op[1] == pt[1] and bbox[0] <= pt[0] <= bbox[2]
                            for cp, op in ((cp, pin_coords[(ref, cp)]) for cp in (ref_insts[ref].terminals if ref in ref_insts else {}) if ref_insts[ref].terminals[cp] == net and cp == "B")
                        ):
                            continue
                        if any(
                            op[0] == pt[0] and bbox[1] <= pt[1] <= bbox[3]
                            for cp, op in ((cp, pin_coords[(ref, cp)]) for cp in (ref_insts[ref].terminals if ref in ref_insts else {}) if ref_insts[ref].terminals[cp] == net and cp == "B")
                        ):
                            continue
                        wire_through_body += 1
                        findings.append(
                            AuditFinding(
                                severity="ERROR",
                                code="WIRE_THROUGH_SYMBOL",
                                message=f"网络 {net!r} 导线穿过器件 {ref} 本体内部 @ {pt}",
                                subjects=[net, ref],
                            )
                        )

                # Parameter label box collision?
                for ref, lbox in label_boxes.items():
                    if lbox[0] < pt[0] < lbox[2] and lbox[1] < pt[1] < lbox[3]:
                        label_overlaps += 1
                        findings.append(
                            AuditFinding(
                                severity="ERROR",
                                code="WIRE_OVER_TEXT_LABEL",
                                message=f"网络 {net!r} 导线压过器件 {ref} 的 W/L 参数文字框 @ {pt}",
                                subjects=[net, ref],
                            )
                        )

    # Collinear foreign wire overlap & perpendicular crossings
    for pt, nets_h in h_grid.items():
        if len(nets_h) > 1:
            collinear_foreign_overlaps += 1
            findings.append(
                AuditFinding(
                    severity="ERROR",
                    code="COLLINEAR_WIRE_OVERLAP",
                    message=f"异网水平导线共线重叠 @ {pt}: {sorted(nets_h)}",
                    subjects=sorted(nets_h),
                )
            )
    for pt, nets_v in v_grid.items():
        if len(nets_v) > 1:
            collinear_foreign_overlaps += 1
            findings.append(
                AuditFinding(
                    severity="ERROR",
                    code="COLLINEAR_WIRE_OVERLAP",
                    message=f"异网垂直导线共线重叠 @ {pt}: {sorted(nets_v)}",
                    subjects=sorted(nets_v),
                )
            )

    crossings_by_net: Counter[str] = Counter()
    crossing_pairs: list[tuple[str, str, Point]] = []
    for pt in set(h_grid) & set(v_grid):
        for nh in h_grid[pt]:
            for nv in v_grid[pt]:
                if nh != nv:
                    crossing_pairs.append((nh, nv, pt))
                    crossings_by_net[nh] += 1
                    crossings_by_net[nv] += 1

    # Check if any bend or junction sits on a foreign wire (ambiguous T/corner crossing!)
    for net, bj_pt in bends_and_juncs:
        foreign = (h_grid.get(bj_pt, set()) | v_grid.get(bj_pt, set())) - {net}
        if foreign:
            ambiguous_crossings += 1
            findings.append(
                AuditFinding(
                    severity="ERROR",
                    code="AMBIGUOUS_JUNCTION_CROSSING",
                    message=f"网络 {net!r} 的拐点/接点 @ {bj_pt} 落在异网导线 {sorted(foreign)} 上，产生视觉连接歧义",
                    subjects=[net, *sorted(foreign)],
                )
            )

    # Check parallel spacing crowds (adjacent 1-grid parallel run >= 3 grids long)
    for (x, y), nets_h in h_grid.items():
        adj = h_grid.get((x, y + 1), set())
        adj_next = h_grid.get((x + 1, y + 1), set())
        cur_next = h_grid.get((x + 1, y), set())
        for n1 in nets_h & cur_next:
            for n2 in adj & adj_next:
                if n1 != n2:
                    parallel_crowds += 1

    if parallel_crowds > 0:
        findings.append(
            AuditFinding(
                severity="WARNING",
                code="PARALLEL_TRACK_CROWDED",
                message=f"检测到 {parallel_crowds} 处相邻 1 格平行异网走线，建议增大通道间距或运行 refine 推开轨道。",
            )
        )

    # Generate actionable LLM refinement suggestions
    for net, c_cnt in crossings_by_net.most_common(3):
        if c_cnt >= 2:
            suggestions.append(
                f"网络 {net!r} 当前产生 {c_cnt} 次跨线交叉：若为偏置或控制网，可在 intent.json 中设 "
                f"`netPolicies.{net} = \"perimeter_bus\"`，或调整相关功能组的 `stage_order` / 管子 `gate_left`/`gate_right` 朝向以消除交叉。"
            )
    if not suggestions and len(crossing_pairs) == 0:
        suggestions.append("当前原理图零异网交叉、零压字、零穿模，可直接用于复核或导入目标库。")

    hard_readability_errors = (
        symbol_overlaps
        + label_overlaps
        + wire_through_body
        + wire_over_foreign_pin
        + collinear_foreign_overlaps
        + ambiguous_crossings
    )
    readability_passed = hard_readability_errors == 0

    metrics = {
        "instanceCount": len(m_insts),
        "netCount": len(circuit.nets),
        "routeSegmentCount": sum(len(v) for v in segments_by_net.values()),
        "totalWireLengthGrids": total_wire_len,
        "symbolOverlapCount": symbol_overlaps,
        "labelOverlapCount": label_overlaps,
        "wireThroughBodyCount": wire_through_body,
        "wireOverForeignPinCount": wire_over_foreign_pin,
        "collinearForeignWireOverlapCount": collinear_foreign_overlaps,
        "ambiguousCrossingCount": ambiguous_crossings,
        "parallelCrowdGridPairs": parallel_crowds,
        "wireCrossingCount": len(crossing_pairs),
        "crossingsByNet": dict(crossings_by_net),
    }

    return AuditReport(
        cell_name=circuit.cell_name,
        electrical_passed=electrical_passed,
        readability_passed=readability_passed,
        bridge_manifest_valid=bridge_ok,
        metrics=metrics,
        findings=findings,
        llm_refinement_suggestions=suggestions,
    )
