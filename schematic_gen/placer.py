"""Constrained placement engine: Micro-templates + Convex QP compaction/expansion + integer symmetry."""

from __future__ import annotations

from collections import defaultdict
import math
from typing import Any

import numpy as np
from scipy.optimize import minimize

from schematic_gen.analyzer import _assign_default_tiers
from schematic_gen.models import (
    BBox,
    InstanceIR,
    NetlistCircuit,
    PlacedInstance,
    Point,
    PortOccurrencePlaced,
    SchematicIntent,
)
from schematic_gen.pdk import PdkRegistry, semantic_to_transform


def solve_1d_compaction_qp(
    node_ids: list[str],
    initial_pos: dict[str, float],
    extents: dict[str, tuple[int, int]],
    separations: list[tuple[str, str, int]],
    target_springs: list[tuple[str, str, int, float]],
    equalities: list[tuple[dict[str, float], float]] | None = None,
    symmetric_triples: list[tuple[str, str, str, int, int, int]] | None = None,
    anchor_weight: float = 0.04,
) -> dict[str, int]:
    """Solve 1D constrained placement with BOTH expansion (min clearance) and compaction (target gap).

    Parameters
    ----------
    node_ids:
        Ordered variable identifiers.
    initial_pos:
        Initial reference coordinate `x0[u]` for gentle anchor regularization.
    extents:
        `u -> (left_ext, right_ext)` non-negative extents around `x[u]`, so occupied span is
        `[x[u] - left_ext, x[u] + right_ext]`.
    separations:
        Hard separation constraints `(u, v, min_gap)`:
        `(x[v] - left_ext[v]) - (x[u] + right_ext[u]) >= min_gap`.
    target_springs:
        Soft ideal net-gap springs `(u, v, ideal_gap, weight)` minimizing
        `weight * (((x[v] - left_ext[v]) - (x[u] + right_ext[u])) - ideal_gap)^2`.
    equalities:
        General linear equalities `({var: coeff}, rhs)` -> `sum(coeff * x[var]) == rhs`.
    symmetric_triples:
        Exact integer-preserving symmetry `(left_id, center_id, right_id, d_l, d_c, d_r)` enforcing
        `(x[left] + d_l) + (x[right] + d_r) == 2 * (x[center] + d_c)`.
    """
    if not node_ids:
        return {}
    n = len(node_ids)
    idx_of = {u: i for i, u in enumerate(node_ids)}
    x0 = np.array([float(initial_pos.get(u, i * 16.0)) for i, u in enumerate(node_ids)], dtype=float)

    equalities = list(equalities or [])
    symmetric_triples = list(symmetric_triples or [])
    for l_id, c_id, r_id, d_l, d_c, d_r in symmetric_triples:
        if l_id in idx_of and c_id in idx_of and r_id in idx_of:
            # x_l + x_r - 2*x_c == 2*d_c - d_l - d_r
            coeffs: dict[str, float] = {l_id: 1.0, r_id: 1.0, c_id: -2.0}
            rhs = float(2 * d_c - d_l - d_r)
            equalities.append((coeffs, rhs))

    # Build quadratic matrix H and linear vector f for 0.5 * x^T H x + f^T x
    H = np.eye(n, dtype=float) * (2.0 * anchor_weight)
    f = -2.0 * anchor_weight * x0

    for u, v, ideal_gap, weight in target_springs:
        if u not in idx_of or v not in idx_of or weight <= 0:
            continue
        iu, iv = idx_of[u], idx_of[v]
        # Want x[v] - x[u] == right_ext[u] + left_ext[v] + ideal_gap
        desired_diff = float(extents[u][1] + extents[v][0] + ideal_gap)
        w2 = 2.0 * float(weight)
        H[iu, iu] += w2
        H[iv, iv] += w2
        H[iu, iv] -= w2
        H[iv, iu] -= w2
        f[iu] += w2 * desired_diff
        f[iv] -= w2 * desired_diff

    def obj(x: np.ndarray) -> float:
        return float(0.5 * x @ (H @ x) + f @ x)

    def jac(x: np.ndarray) -> np.ndarray:
        return H @ x + f

    constraints: list[dict[str, Any]] = []
    for u, v, min_gap in separations:
        if u not in idx_of or v not in idx_of or u == v:
            continue
        iu, iv = idx_of[u], idx_of[v]
        req = float(extents[u][1] + extents[v][0] + min_gap)
        constraints.append(
            {
                "type": "ineq",
                "fun": lambda x, a=iu, b=iv, r=req: x[b] - x[a] - r,
                "jac": lambda x, a=iu, b=iv: _unit_diff_grad(n, a, b),
            }
        )

    for coeff_map, rhs in equalities:
        row = np.zeros(n, dtype=float)
        for var, c in coeff_map.items():
            if var in idx_of:
                row[idx_of[var]] += float(c)
        if np.any(np.abs(row) > 1e-12):
            constraints.append(
                {
                    "type": "eq",
                    "fun": lambda x, r=row, b=float(rhs): float(r @ x - b),
                    "jac": lambda x, r=row: r,
                }
            )

    res = minimize(
        obj,
        x0,
        jac=jac,
        constraints=constraints,
        method="SLSQP",
        options={"ftol": 1e-9, "maxiter": 500},
    )
    x_cont = res.x if res.success else x0

    # Exact integer quantization preserving rigid pin alignments, symmetric triples, and all separations!
    x_int = {u: int(round(float(x_cont[idx_of[u]]))) for u in node_ids}

    # Build rigid offset components from 2-variable equalities `x[v] - x[u] == delta`
    rigid_adj: dict[str, list[tuple[str, int]]] = defaultdict(list)
    for coeff_map, rhs in equalities:
        if len(coeff_map) == 2:
            items = list(coeff_map.items())
            (k1, c1), (k2, c2) = items[0], items[1]
            if k1 in idx_of and k2 in idx_of and math.isclose(c1 + c2, 0.0, abs_tol=1e-9):
                # c1 * x[k1] - c1 * x[k2] == rhs  =>  x[k1] - x[k2] == rhs / c1
                delta = int(round(rhs / c1))
                rigid_adj[k2].append((k1, delta))
                rigid_adj[k1].append((k2, -delta))

    visited: set[str] = set()
    cluster_of: dict[str, int] = {}
    clusters: list[dict[str, int]] = []  # cluster_idx -> {node: offset_from_anchor}
    cluster_pos: list[int] = []

    for u in node_ids:
        if u in visited:
            continue
        cid = len(clusters)
        rel = {u: 0}
        queue = [u]
        visited.add(u)
        for cur in queue:
            for nxt, d in rigid_adj[cur]:
                if nxt not in visited:
                    visited.add(nxt)
                    rel[nxt] = rel[cur] + d
                    queue.append(nxt)
        anchor_val = int(
            round(sum(x_cont[idx_of[node]] - off for node, off in rel.items()) / len(rel))
        )
        for node, off in rel.items():
            cluster_of[node] = cid
            x_int[node] = anchor_val + off
        clusters.append(rel)
        cluster_pos.append(anchor_val)

    def _shift_cluster(cid: int, delta: int) -> None:
        if delta == 0:
            return
        cluster_pos[cid] += delta
        for node in clusters[cid]:
            x_int[node] += delta

    # Enforce exact integer symmetry on symmetric triples
    sym_by_cluster: dict[int, tuple[int, int, int, str, str, str, int, int, int]] = {}
    for l_id, c_id, r_id, d_l, d_c, d_r in symmetric_triples:
        if l_id not in cluster_of or c_id not in cluster_of or r_id not in cluster_of:
            continue
        cl, cc, cr = cluster_of[l_id], cluster_of[c_id], cluster_of[r_id]
        if len({cl, cc, cr}) < 3:
            continue
        axis = x_int[c_id] + d_c
        span_l = axis - (x_int[l_id] + d_l)
        span_r = (x_int[r_id] + d_r) - axis
        half_span = max(1, int(math.ceil((span_l + span_r) / 2.0)))
        _shift_cluster(cl, (axis - half_span - d_l) - x_int[l_id])
        _shift_cluster(cr, (axis + half_span - d_r) - x_int[r_id])
        sym_meta = (cl, cc, cr, l_id, c_id, r_id, d_l, d_c, d_r)
        sym_by_cluster[cl] = sym_meta
        sym_by_cluster[cc] = sym_meta
        sym_by_cluster[cr] = sym_meta

    # Monotonic sweep to repair any +/-1 rounding violation without breaking symmetry
    for _pass in range(len(node_ids) + 4):
        changed = False
        for u, v, min_gap in separations:
            if u not in x_int or v not in x_int:
                continue
            req = extents[u][1] + extents[v][0] + min_gap
            deficit = req - (x_int[v] - x_int[u])
            if deficit <= 0:
                continue
            changed = True
            cu, cv = cluster_of[u], cluster_of[v]
            if cv in sym_by_cluster:
                cl, cc, cr, l_id, c_id, r_id, d_l, d_c, d_r = sym_by_cluster[cv]
                if cu not in {cl, cc, cr}:
                    # External push from the left -> translate entire symmetric triple together!
                    _shift_cluster(cl, deficit)
                    _shift_cluster(cc, deficit)
                    _shift_cluster(cr, deficit)
                elif cu == cl and cv == cc:
                    # Need wider left-to-center -> keep cl fixed, shift cc by +deficit and cr by +2*deficit!
                    _shift_cluster(cc, deficit)
                    _shift_cluster(cr, 2 * deficit)
                elif cu == cc and cv == cr:
                    # Need wider center-to-right -> keep cl fixed, shift cc by +deficit and cr by +2*deficit!
                    _shift_cluster(cc, deficit)
                    _shift_cluster(cr, 2 * deficit)
                elif cu == cl and cv == cr:
                    dh = int(math.ceil(deficit / 2.0))
                    _shift_cluster(cc, dh)
                    _shift_cluster(cr, 2 * dh)
                else:
                    _shift_cluster(cv, deficit)
            else:
                _shift_cluster(cv, deficit)
        if not changed:
            break

    # Shift minimum coordinate so layout starts at a clean positive coordinate if negative
    return x_int


def _unit_diff_grad(n: int, iu: int, iv: int) -> np.ndarray:
    g = np.zeros(n, dtype=float)
    g[iu] = -1.0
    g[iv] = 1.0
    return g


def _build_columns(circuit: NetlistCircuit, intent: SchematicIntent) -> tuple[list[list[str]], dict[str, str]]:
    """Assign every circuit instance to an ordered vertical column and a functional group."""
    all_refs = [i.ref for i in circuit.instances]
    group_of: dict[str, str] = {}
    ordered_groups = sorted(intent.groups, key=lambda g: g.stage_order)
    for g in ordered_groups:
        for r in g.instances:
            if r in all_refs and r not in group_of:
                group_of[r] = g.id
    for r in all_refs:
        group_of.setdefault(r, "default_group")

    columns: list[list[str]] = []
    placed: set[str] = set()

    # Respect explicit column hints first, ordered by the stage_order of their primary group
    stage_rank = {g.id: g.stage_order for g in ordered_groups}
    stage_rank.setdefault("default_group", 999)

    hinted_cols: list[tuple[int, int, list[str]]] = []
    for idx, raw_col in enumerate(intent.column_order_hints):
        clean = [r for r in raw_col if r in all_refs and r not in placed]
        if clean:
            placed.update(clean)
            rank = min(stage_rank.get(group_of[r], 999) for r in clean)
            hinted_cols.append((rank, idx, clean))

    for g in ordered_groups:
        for r in g.instances:
            if r in all_refs and r not in placed:
                placed.add(r)
                hinted_cols.append((g.stage_order, 1000 + len(hinted_cols), [r]))

    for r in all_refs:
        if r not in placed:
            placed.add(r)
            hinted_cols.append((999, 2000 + len(hinted_cols), [r]))

    hinted_cols.sort(key=lambda item: (item[0], item[1]))
    columns = [col for _, _, col in hinted_cols]
    return columns, group_of


def _alignment_pin_for_column(inst: InstanceIR) -> str:
    """Choose the primary vertical branch pin (`D` or `P`) for straight vertical column alignment."""
    if inst.device_class == "mos":
        return "D"
    if "P" in inst.terminals:
        return "P"
    return next(iter(inst.terminals))


def place_schematic(
    circuit: NetlistCircuit,
    intent: SchematicIntent,
    pdk: PdkRegistry,
    *,
    extra_col_gaps: dict[tuple[int, int], int] | None = None,
    extra_tier_gaps: dict[tuple[int, int], int] | None = None,
    allow_wire_labels: bool = False,
) -> tuple[dict[str, PlacedInstance], list[PortOccurrencePlaced], list[dict[str, str]], list[dict[str, str]]]:
    """Place all instances and top-level/local supply ports using Micro-templates + Convex QP."""
    inst_map = circuit.instance_map()
    supplies = set(circuit.supply_nets)
    grounds = set(circuit.ground_nets)
    rails = supplies | grounds
    port_nets = {p.net: p.name for p in circuit.ports}

    default_tiers = _assign_default_tiers(circuit)
    tiers = {
        r: int(intent.tier_overrides.get(r, default_tiers[r]))
        for r in inst_map
    }
    columns, group_of = _build_columns(circuit, intent)
    col_of: dict[str, int] = {r: ci for ci, col in enumerate(columns) for r in col}

    # Determine bulk handling per instance BEFORE computing bounding envelopes!
    local_bulk_shorts: list[dict[str, str]] = []
    local_bulk_labels: list[dict[str, str]] = []
    has_bulk_stub_map: dict[str, bool] = {}

    for inst in circuit.instances:
        if "B" not in inst.terminals:
            has_bulk_stub_map[inst.ref] = False
            continue
        b_net = inst.terminals["B"]
        s_pin = "S" if "S" in inst.terminals else ("M" if "M" in inst.terminals else None)
        if s_pin and inst.terminals[s_pin] == b_net and inst.device_class == "mos":
            local_bulk_shorts.append(
                {
                    "reference": inst.ref,
                    "bulkPinName": "B",
                    "sourcePinName": s_pin,
                    "netName": b_net,
                }
            )
            has_bulk_stub_map[inst.ref] = False
        elif b_net in port_nets and (b_net in rails or intent.net_policies.get(b_net) == "bulk_local"):
            local_bulk_labels.append(
                {
                    "reference": inst.ref,
                    "pinName": "B",
                    "name": port_nets[b_net],
                    "netName": b_net,
                }
            )
            has_bulk_stub_map[inst.ref] = True
        else:
            has_bulk_stub_map[inst.ref] = False

    # Compute relative pin offsets, body boxes, and label envelopes for every instance
    transforms: dict[str, tuple[int, str, str, str]] = {}
    rel_pins: dict[str, dict[str, Point]] = {}
    rel_body: dict[str, BBox] = {}
    rel_label: dict[str, BBox] = {}
    rel_env: dict[str, BBox] = {}
    label_lines_map: dict[str, list[str]] = {}

    # Pre-detect non-rail ports using `near_pin` or `local_repeat` so their flag/text box is reserved in QP!
    local_port_nets: set[str] = set()
    for p in circuit.ports:
        if p.net in rails:
            continue
        pside = intent.port_sides.get(p.name, "")
        if pside in {"near_pin", "local_repeat"}:
            local_port_nets.add(p.net)
    if allow_wire_labels:
        for net, pol in intent.net_policies.items():
            if pol == "label_stub" and net not in rails:
                local_port_nets.add(net)

    # Pre-process `serpentine_chain` orientation overrides if not explicitly set
    for m in intent.motifs:
        if m.get("type") == "serpentine_chain":
            s_insts = [str(x) for x in m.get("instances", []) if str(x) in inst_map]
            rpc = max(1, int(m.get("rowsPerColumn", 4)))
            for ci_s in range(0, len(s_insts), rpc):
                chunk = s_insts[ci_s : ci_s + rpc]
                c_idx = ci_s // rpc
                for r in chunk:
                    intent.orientations.setdefault(
                        r, "vertical_flipped" if (c_idx % 2 == 1) else "vertical"
                    )

    # Ensure every non-serpentine column is ordered top-to-bottom by potential tier
    serp_inst_set = {
        str(x)
        for m in intent.motifs
        if m.get("type") == "serpentine_chain"
        for x in m.get("instances", [])
    }
    for col in columns:
        if len(col) >= 2 and not any(r in serp_inst_set for r in col):
            col.sort(key=lambda r: tiers[r])

    # Pre-build column neighbor map so stacked vertical 2-pin passives orient shared pins toward each other
    col_pos_map: dict[str, tuple[list[str], int]] = {
        r: (col, idx_c) for col in columns for idx_c, r in enumerate(col)
    }

    for inst in circuit.instances:
        if inst.device_class == "subckt":
            default_sem = "R0"
        elif inst.device_class == "mos":
            default_sem = "gate_left"
        else:
            p_net = inst.terminals.get("P", "")
            m_net = inst.terminals.get("M", "")
            if (m_net in supplies and p_net not in supplies) or (
                p_net in grounds and m_net not in grounds
            ):
                default_sem = "vertical_flipped"
            else:
                default_sem = "vertical"
        sem = intent.orientations.get(inst.ref, default_sem)
        # Auto-correct upside-down vertical passives (rail polarity OR facing a same-column stacked neighbor!)
        if sem == "vertical" and inst.device_class not in {"mos", "subckt"} and "P" in inst.terminals and "M" in inst.terminals:
            p_net = inst.terminals.get("P", "")
            m_net = inst.terminals.get("M", "")
            if (m_net in supplies and p_net not in supplies) or (
                p_net in grounds and m_net not in grounds
            ):
                sem = "vertical_flipped"
            elif p_net not in rails and m_net not in rails and inst.ref in col_pos_map:
                col_members, idx_c = col_pos_map[inst.ref]
                if idx_c > 0:
                    up_nets = set(inst_map[col_members[idx_c - 1]].terminals.values()) - rails
                    if m_net in up_nets and p_net not in up_nets:
                        sem = "vertical_flipped"
                elif idx_c + 1 < len(col_members):
                    dn_nets = set(inst_map[col_members[idx_c + 1]].terminals.values()) - rails
                    if p_net in dn_nets and m_net not in dn_nets:
                        sem = "vertical_flipped"
        rot, mir, cad = semantic_to_transform(sem)
        transforms[inst.ref] = (rot, mir, cad, sem)
        rel_pins[inst.ref] = pdk.source_pin_offsets(inst.device_key, rot, mir)
        rel_body[inst.ref] = pdk.source_body_box(inst.device_key, rot, mir)
        lbox, env, lines = pdk.relative_label_and_envelope(
            inst, rot, mir, has_bulk_label_stub=has_bulk_stub_map[inst.ref]
        )
        ex0, ey0, ex1, ey1 = env
        # Reserve horizontal clearance when a local input/bias port flag attaches directly to Gate
        if "G" in inst.terminals and inst.terminals["G"] in local_port_nets:
            gx, _ = rel_pins[inst.ref]["G"]
            if gx < 0:
                ex0 = min(ex0, gx - 11)
            else:
                ex1 = max(ex1, gx + 11)
        # Reserve vertical clearance for local VDD/VSS tap + symbol so stacked rows never seal off pins
        for pin_n, net_n in inst.terminals.items():
            if pin_n != "B" and net_n in rails and pin_n in rel_pins[inst.ref]:
                _, ry = rel_pins[inst.ref][pin_n]
                is_vdd_pin = net_n in supplies
                go_up_pin = is_vdd_pin if (ry <= 0) else False
                if go_up_pin:
                    ey0 = min(ey0, ry - 8)
                else:
                    ey1 = max(ey1, ry + 8)
        rel_label[inst.ref] = lbox
        rel_env[inst.ref] = (ex0, ey0, ex1, ey1)
        label_lines_map[inst.ref] = lines

    # Resolve row sub-ordering if two devices in the SAME column share the SAME tier
    effective_row_key: dict[str, tuple[int, int]] = {}
    for col in columns:
        by_t: dict[int, list[str]] = defaultdict(list)
        for r in col:
            by_t[tiers[r]].append(r)
        for t, members in by_t.items():
            for sub_idx, r in enumerate(members):
                effective_row_key[r] = (t, sub_idx)

    # --- X-Axis Convex QP Setup ---
    node_ids = [i.ref for i in circuit.instances]
    extents_x = {r: (-rel_env[r][0], rel_env[r][2]) for r in node_ids}
    init_x = {r: float(24 + col_of[r] * 18) for r in node_ids}
    sep_x: list[tuple[str, str, int]] = []
    springs_x: list[tuple[str, str, int, float]] = []
    eq_x: list[tuple[dict[str, float], float]] = []
    sym_x: list[tuple[str, str, str, int, int, int]] = []

    # Within each vertical column, align connected pins (including 1-to-1 passive-over-subckt stacking)!
    col_rel_offset: dict[str, int] = {r: 0 for r in node_ids}
    for col in columns:
        if len(col) >= 2:
            for u_ref, v_ref in zip(col, col[1:]):
                iu, iv = inst_map[u_ref], inst_map[v_ref]
                if (iu.device_class == "subckt") != (iv.device_class == "subckt"):
                    u_terms = {p: n for p, n in iu.terminals.items() if p != "B" and n not in rails}
                    v_terms = {p: n for p, n in iv.terminals.items() if p != "B" and n not in rails}
                    shared_nv = set(u_terms.values()) & set(v_terms.values())
                    if shared_nv:
                        sh_n = sorted(shared_nv)[0]
                        pu = next(p for p, n in u_terms.items() if n == sh_n)
                        pv = next(p for p, n in v_terms.items() if n == sh_n)
                        du = rel_pins[u_ref][pu][0]
                        dv = rel_pins[v_ref][pv][0]
                        if iv.device_class == "subckt":
                            bx0, _, bx1, _ = rel_body[v_ref]
                            if dv <= bx0 or dv >= bx1:
                                dv += 2 if dv > 0 else -2
                        else:
                            bx0, _, bx1, _ = rel_body[u_ref]
                            if du <= bx0 or du >= bx1:
                                du += 2 if du > 0 else -2
                        delta = int(round(du - dv))
                        eq_x.append(({v_ref: 1.0, u_ref: -1.0}, float(delta)))
                        col_rel_offset[v_ref] = col_rel_offset[u_ref] + delta
                        continue
                du = rel_pins[u_ref][_alignment_pin_for_column(iu)][0]
                dv = rel_pins[v_ref][_alignment_pin_for_column(iv)][0]
                delta = int(round(du - dv))
                eq_x.append(({v_ref: 1.0, u_ref: -1.0}, float(delta)))
                col_rel_offset[v_ref] = col_rel_offset[u_ref] + delta

    # Add symmetric triples for diff pairs AND rigid uniform-pitch equalities for regular array motifs!
    for m in intent.motifs:
        mtype = m.get("type")
        if mtype == "diff_pair":
            l_ref = m.get("left")
            r_ref = m.get("right")
            t_ref = m.get("tail")
            if l_ref in inst_map and r_ref in inst_map and t_ref in inst_map:
                dl = rel_pins[l_ref]["S"][0]
                dr = rel_pins[r_ref]["S"][0]
                dt = rel_pins[t_ref]["D"][0]
                sym_x.append((l_ref, t_ref, r_ref, dl, dt, dr))
        elif mtype == "parallel_bank":
            b_insts = [str(x) for x in m.get("instances", []) if str(x) in inst_map]
            if len(b_insts) >= 2:
                # Force uniform row alignment and exact rigid integer pitch across the entire bank
                # (accounting for any wider subckt/MOS drivers stacked in the same columns)!
                common_tier = tiers[b_insts[0]]
                for r in b_insts:
                    tiers[r] = common_tier
                    effective_row_key[r] = (common_tier, 0)
                has_stacked_subckt = any(
                    inst_map[a].device_class == "subckt"
                    for u in b_insts
                    for a in columns[col_of[u]]
                )
                extra_bank_room = 6 if has_stacked_subckt else 1
                min_auto_pitch = 8
                for u, v in zip(b_insts, b_insts[1:]):
                    cu, cv = columns[col_of[u]], columns[col_of[v]]
                    for a in cu:
                        for b in cv:
                            if tiers[a] == tiers[b]:
                                # Required x[v] - x[u] so that x[b] - x[a] >= extents_x[a][1] + extents_x[b][0] + clr
                                off_a = col_rel_offset[a] - col_rel_offset[u]
                                off_b = col_rel_offset[b] - col_rel_offset[v]
                                req_uv = (
                                    extents_x[a][1]
                                    + extents_x[b][0]
                                    + intent.spacing.min_clearance_x
                                    + extra_bank_room
                                    + off_a
                                    - off_b
                                )
                                min_auto_pitch = max(min_auto_pitch, req_uv)
                pitch = max(int(m.get("pitchX", 0)), min_auto_pitch)
                for u, v in zip(b_insts, b_insts[1:]):
                    eq_x.append(({v: 1.0, u: -1.0}, float(pitch)))
        elif mtype == "serpentine_chain":
            s_insts = [str(x) for x in m.get("instances", []) if str(x) in inst_map]
            rpc = max(1, int(m.get("rowsPerColumn", 4)))
            col_anchors: list[str] = []
            base_tier = tiers[s_insts[0]] if s_insts else 3
            for ci_s in range(0, len(s_insts), rpc):
                chunk = s_insts[ci_s : ci_s + rpc]
                c_idx = ci_s // rpc
                ordered_top_down = list(reversed(chunk)) if (c_idx % 2 == 1) else list(chunk)
                col_anchors.append(ordered_top_down[0])
                for sub_r, r in enumerate(ordered_top_down):
                    tiers[r] = base_tier
                    effective_row_key[r] = (base_tier, sub_r)
                    if sub_r > 0:
                        eq_x.append(({r: 1.0, ordered_top_down[0]: -1.0}, 0.0))
            if len(col_anchors) >= 2:
                s_pitch = max(
                    extents_x[u][1] + extents_x[v][0] + intent.spacing.min_clearance_x + 2
                    for u, v in zip(col_anchors, col_anchors[1:])
                )
                for u, v in zip(col_anchors, col_anchors[1:]):
                    eq_x.append(({v: 1.0, u: -1.0}, float(s_pitch)))

    extra_col_gaps = extra_col_gaps or {}
    # Column-to-column separation & connection-driven compaction springs (NO horizontal group wall when tiers don't overlap!)
    for ci in range(len(columns) - 1):
        col_left = columns[ci]
        col_right = columns[ci + 1]
        same_group = group_of[col_left[0]] == group_of[col_right[0]]
        base_target = (
            intent.spacing.intra_group_gap_x if same_group else intent.spacing.inter_group_gap_x
        )
        corridor_bonus = extra_col_gaps.get((ci, ci + 1), 0)
        tiers_left = {tiers[u] for u in col_left}
        tiers_right = {tiers[v] for v in col_right}
        shares_tier = bool(tiers_left & tiers_right)

        if shares_tier:
            min_gap = max(intent.spacing.min_clearance_x, (base_target // 2)) + corridor_bonus
            target_gap = max(base_target, min_gap)
            spring_w = 6.0 if same_group else 3.5
            for u in col_left:
                for v in col_right:
                    sep_x.append((u, v, min_gap if tiers[u] == tiers[v] else (2 + corridor_bonus)))
            springs_x.append((col_left[0], col_right[0], target_gap, spring_w))
        else:
            diff_tails = {
                str(m["tail"])
                for m in intent.motifs
                if m.get("type") == "diff_pair" and m.get("tail")
            }
            if any(r in diff_tails for r in col_left + col_right):
                # Centered tail M5 between M1 & M2: keep compact body-only subcolumn offset
                for u in col_left:
                    for v in col_right:
                        body_gap = -(extents_x[u][1] + extents_x[v][0]) + 8 + corridor_bonus
                        sep_x.append((u, v, body_gap))
            else:
                # Non-tail staggered columns: keep full clearance so Y-floating branches never crowd neighbor labels/gates
                stagger_min = max(intent.spacing.min_clearance_x, 4) + corridor_bonus
                for u in col_left:
                    for v in col_right:
                        sep_x.append((u, v, stagger_min))
                springs_x.append(
                    (col_left[0], col_right[0], max(stagger_min, base_target // 2 + corridor_bonus), 4.0)
                )

    # Also enforce non-overlap between columns skipping an intermediate column (e.g. left leg vs right leg across tail column!)
    for ci in range(len(columns)):
        for cj in range(ci + 2, len(columns)):
            same_grp = group_of[columns[ci][0]] == group_of[columns[cj][0]]
            for u in columns[ci]:
                for v in columns[cj]:
                    if tiers[u] == tiers[v]:
                        # Direct horizontal neighbors on the same row (e.g. M1 and M2 across M5!)
                        pair_min = intent.spacing.intra_group_gap_x if same_grp else intent.spacing.inter_group_gap_x
                        corridor_mid = sum(
                            extra_col_gaps.get((k, k + 1), 0) for k in range(ci, cj)
                        )
                        sep_x.append((u, v, pair_min + corridor_mid))
                        if cj == ci + 2 and same_grp:
                            springs_x.append((u, v, pair_min + corridor_mid, 4.0))

    solved_x = solve_1d_compaction_qp(
        node_ids=node_ids,
        initial_pos=init_x,
        extents=extents_x,
        separations=sep_x,
        target_springs=springs_x,
        equalities=eq_x,
        symmetric_triples=sym_x,
    )

    # --- Y-Axis Convex QP Setup ---
    # Group instances by discrete row `(tier, sub_idx)` so devices in the same potential tier align cleanly!
    row_keys = sorted(set(effective_row_key.values()))
    row_ids = [f"row_{t}_{s}" for t, s in row_keys]
    row_id_of = {rk: f"row_{rk[0]}_{rk[1]}" for rk in row_keys}
    members_by_row: dict[str, list[str]] = defaultdict(list)
    for r, rk in effective_row_key.items():
        members_by_row[row_id_of[rk]].append(r)

    extents_y: dict[str, tuple[int, int]] = {}
    init_y: dict[str, float] = {}
    for idx, rk in enumerate(row_keys):
        rid = row_id_of[rk]
        up_ext = max(-rel_env[r][1] for r in members_by_row[rid])
        dn_ext = max(rel_env[r][3] for r in members_by_row[rid])
        extents_y[rid] = (up_ext, dn_ext)
        init_y[rid] = float(20 + idx * 18)

    sep_y: list[tuple[str, str, int]] = []
    springs_y: list[tuple[str, str, int, float]] = []
    extra_tier_gaps = extra_tier_gaps or {}

    for idx in range(len(row_keys) - 1):
        rk_a, rk_b = row_keys[idx], row_keys[idx + 1]
        rid_a, rid_b = row_id_of[rk_a], row_id_of[rk_b]
        same_tier = rk_a[0] == rk_b[0]
        bonus_y = extra_tier_gaps.get((rk_a[0], rk_b[0]), 0)
        min_gy = intent.spacing.min_clearance_y + bonus_y
        tgt_gy = (
            intent.spacing.intra_stack_gap_y if same_tier else intent.spacing.inter_tier_gap_y
        ) + bonus_y
        sep_y.append((rid_a, rid_b, min_gy))
        springs_y.append((rid_a, rid_b, tgt_gy, 5.0))

    solved_row_y = solve_1d_compaction_qp(
        node_ids=row_ids,
        initial_pos=init_y,
        extents=extents_y,
        separations=sep_y,
        target_springs=springs_y,
    )

    # Per-instance Y-Axis Convex QP for deep-stack schematics (`max_col_depth >= 4`, e.g. 4-5 tier bias/cascode cores),
    # plus exact rail-pin alignment for singleton shunt columns (e.g. RESET, C1/C2, CL) on standard 2-3 tier sheets!
    max_col_depth = max((len(c) for c in columns), default=1)
    if (intent.authored_by != "analyzer_draft" and bool(intent.tier_overrides)) or max_col_depth <= 3:
        solved_inst_y = {r: solved_row_y[row_id_of[effective_row_key[r]]] for r in node_ids}
        if intent.authored_by == "analyzer_draft":
            net_inst_fanout: dict[str, int] = defaultdict(int)
            for inst in circuit.instances:
                for p, n in inst.terminals.items():
                    if p != "B" and n not in rails:
                        net_inst_fanout[n] += 1

            parallel_bank_set = {
                str(x)
                for m in intent.motifs
                if m.get("type") == "parallel_bank"
                for x in m.get("instances", [])
            }
            for ci_s, col_s in enumerate(columns):
                if len(col_s) != 1:
                    continue
                r_s = col_s[0]
                if r_s in parallel_bank_set:
                    continue
                inst_s = inst_map[r_s]
                # Skip if X-overlapping with another column (e.g. centered diff-pair tail)
                if any(
                    cj != ci_s
                    and max(solved_x[r_s] - extents_x[r_s][0], solved_x[v] - extents_x[v][0])
                    < min(solved_x[r_s] + extents_x[r_s][1], solved_x[v] + extents_x[v][1])
                    for cj, col_o in enumerate(columns)
                    for v in col_o
                ):
                    continue
                gnd_dy = next(
                    (
                        rel_pins[r_s][p][1]
                        for p, n in inst_s.terminals.items()
                        if p != "B" and n in grounds and rel_pins[r_s][p][1] >= 0
                    ),
                    None,
                )
                if gnd_dy is None:
                    continue
                is_passive_shunt = inst_s.device_class not in {"mos", "subckt"}
                has_local_sig = any(
                    2 <= net_inst_fanout[n] <= 5
                    for p, n in inst_s.terminals.items()
                    if p != "B" and n not in rails
                )
                if not is_passive_shunt and has_local_sig:
                    continue
                # Align `r_s`'s ground pin Y with the nearest non-singleton (or neighbor) column's bottom ground pin!
                best_nb_pin_y: int | None = None
                best_dist = 10**9
                for cj, col_nb in enumerate(columns):
                    if cj == ci_s:
                        continue
                    r_bot = col_nb[-1]
                    if len(col_nb) == 1 and inst_map[r_bot].device_class not in {"mos", "subckt"}:
                        continue
                    nb_dy = next(
                        (
                            rel_pins[r_bot][p][1]
                            for p, n in inst_map[r_bot].terminals.items()
                            if p != "B" and n in grounds and rel_pins[r_bot][p][1] >= 0
                        ),
                        None,
                    )
                    if nb_dy is not None:
                        pref_penalty = 0 if len(col_nb) >= 2 else 20
                        d_score = abs(cj - ci_s) + pref_penalty
                        if d_score < best_dist:
                            best_dist = d_score
                            best_nb_pin_y = solved_inst_y[r_bot] + nb_dy
                if best_nb_pin_y is not None:
                    solved_inst_y[r_s] = best_nb_pin_y - gnd_dy
    else:
        extents_y_inst = {r: (-rel_env[r][1], rel_env[r][3]) for r in node_ids}
        init_y_inst = {r: float(solved_row_y[row_id_of[effective_row_key[r]]]) for r in node_ids}
        sep_y_inst: list[tuple[str, str, int]] = []
        springs_y_inst: list[tuple[str, str, int, float]] = []
        eq_y_inst: list[tuple[dict[str, float], float]] = []

        def _add_pin_y_spring(u: str, dy_u: int, v: str, dy_v: int, w: float) -> None:
            if u == v or w <= 0:
                return
            ideal_g = int(round((dy_u - dy_v) - extents_y_inst[u][1] - extents_y_inst[v][0]))
            springs_y_inst.append((u, v, ideal_g, w))

        # 1) Intra-column vertical separations & tight signal-branch cohesion springs
        for col in columns:
            for u, v in zip(col, col[1:]):
                has_mid_rail = any(
                    (n in supplies and rel_pins[v][p][1] < 0)
                    for p, n in inst_map[v].terminals.items()
                    if p != "B"
                ) or any(
                    (n in grounds and rel_pins[u][p][1] > 0)
                    for p, n in inst_map[u].terminals.items()
                    if p != "B"
                )
                mid_rail_pad = 6 if has_mid_rail else 0
                cmos_corridor_pad = (
                    4
                    if (
                        inst_map[u].device_class == "mos"
                        and inst_map[v].device_class == "mos"
                        and inst_map[u].polarity != inst_map[v].polarity
                    )
                    else 0
                )
                same_tier = tiers[u] == tiers[v]
                bonus_y = extra_tier_gaps.get((tiers[u], tiers[v]), 0) + mid_rail_pad
                min_gy = intent.spacing.min_clearance_y + bonus_y
                tgt_gy = (
                    intent.spacing.intra_stack_gap_y if same_tier else intent.spacing.inter_tier_gap_y
                ) + bonus_y + cmos_corridor_pad
                sep_y_inst.append((u, v, min_gy))
                u_sig = {n for p, n in inst_map[u].terminals.items() if p != "B"} - rails
                v_sig = {n for p, n in inst_map[v].terminals.items() if p != "B"} - rails
                w_intra = 7.5 if (u_sig & v_sig) else 5.0
                springs_y_inst.append((u, v, tgt_gy, w_intra))

        # 2) Cross-column vertical separation whenever X envelopes strictly overlap (e.g. centered diff-pair tail M5)
        for ci in range(len(columns)):
            for cj in range(ci + 1, len(columns)):
                for u in columns[ci]:
                    for v in columns[cj]:
                        u_l, u_r = solved_x[u] - extents_x[u][0], solved_x[u] + extents_x[u][1]
                        v_l, v_r = solved_x[v] - extents_x[v][0], solved_x[v] + extents_x[v][1]
                        if max(u_l, v_l) < min(u_r, v_r):
                            top_r, bot_r = (
                                (u, v)
                                if (effective_row_key[u], ci) <= (effective_row_key[v], cj)
                                else (v, u)
                            )
                            bonus_y = extra_tier_gaps.get((tiers[top_r], tiers[bot_r]), 0)
                            sep_y_inst.append((top_r, bot_r, intent.spacing.min_clearance_y + bonus_y))
                            springs_y_inst.append(
                                (top_r, bot_r, intent.spacing.inter_tier_gap_y + bonus_y, 5.5)
                            )

        def _add_ordered_diff_sep(a: str | None, b: str | None, add_spring: bool = False) -> None:
            if not a or not b or a not in inst_map or b not in inst_map:
                return
            u_top, v_bot = (a, b) if effective_row_key[a] <= effective_row_key[b] else (b, a)
            sep_y_inst.append((u_top, v_bot, intent.spacing.min_clearance_y))
            if add_spring:
                springs_y_inst.append((u_top, v_bot, intent.spacing.inter_tier_gap_y, 6.0))

        for m in intent.motifs:
            if m.get("type") == "diff_pair":
                l_r, r_r, t_r = m.get("left"), m.get("right"), m.get("tail")
                ll_r, lr_r = m.get("loadLeft"), m.get("loadRight")
                _add_ordered_diff_sep(ll_r, l_r)
                _add_ordered_diff_sep(lr_r, r_r)
                _add_ordered_diff_sep(l_r, t_r, add_spring=True)
                _add_ordered_diff_sep(r_r, t_r)

        # 3) Cycle-safe exact row alignment equalities for true symmetric / array / matched-mirror motifs
        uf_parent = {r: r for r in node_ids}

        def _uf_find(x_id: str) -> str:
            while uf_parent[x_id] != x_id:
                uf_parent[x_id] = uf_parent[uf_parent[x_id]]
                x_id = uf_parent[x_id]
            return x_id

        def _creates_sep_cycle(ca: str, cb: str) -> bool:
            adj_c: dict[str, set[str]] = defaultdict(set)
            for su, sv, _ in sep_y_inst:
                cu, cv = _uf_find(su), _uf_find(sv)
                if cu != cv:
                    adj_c[cu].add(cv)
            for start_c, goal_c in ((ca, cb), (cb, ca)):
                seen_c = {start_c}
                stack_c = [start_c]
                while stack_c:
                    cur_c = stack_c.pop()
                    if cur_c == goal_c:
                        return True
                    for nxt_c in adj_c.get(cur_c, ()):
                        if nxt_c not in seen_c:
                            seen_c.add(nxt_c)
                            stack_c.append(nxt_c)
            return False

        def _align_same_y(a: str | None, b: str | None) -> None:
            if not a or not b or a not in inst_map or b not in inst_map:
                return
            ca, cb = _uf_find(a), _uf_find(b)
            if ca == cb or _creates_sep_cycle(ca, cb):
                return
            uf_parent[cb] = ca
            eq_y_inst.append(({b: 1.0, a: -1.0}, 0.0))

        diff_by_grp: dict[str, list[dict[str, Any]]] = defaultdict(list)
        for m in intent.motifs:
            mtype = m.get("type")
            if mtype == "diff_pair":
                l_r, r_r = m.get("left"), m.get("right")
                ll_r, lr_r = m.get("loadLeft"), m.get("loadRight")
                _align_same_y(l_r, r_r)
                _align_same_y(ll_r, lr_r)
                if l_r in group_of:
                    diff_by_grp[group_of[l_r]].append(m)
            elif mtype == "folded_cascode":
                l_col = [str(x) for x in m.get("leftColumn", [])]
                r_col = [str(x) for x in m.get("rightColumn", [])]
                if len(l_col) == len(r_col):
                    for lu, ru in zip(l_col, r_col):
                        _align_same_y(lu, ru)
            elif mtype == "parallel_bank":
                b_insts = [str(x) for x in m.get("instances", []) if str(x) in inst_map]
                for u, v in zip(b_insts, b_insts[1:]):
                    _align_same_y(u, v)
                s_drvs = [str(x) for x in m.get("stackedDrivers", []) if str(x) in inst_map]
                for u, v in zip(s_drvs, s_drvs[1:]):
                    _align_same_y(u, v)
            elif mtype == "serpentine_chain":
                s_insts = [str(x) for x in m.get("instances", []) if str(x) in inst_map]
                rpc = max(1, int(m.get("rowsPerColumn", 4)))
                by_sub: dict[int, list[str]] = defaultdict(list)
                for ci_s in range(0, len(s_insts), rpc):
                    chunk = s_insts[ci_s : ci_s + rpc]
                    ordered_td = list(reversed(chunk)) if ((ci_s // rpc) % 2 == 1) else list(chunk)
                    for sub_r, r_s in enumerate(ordered_td):
                        by_sub[sub_r].append(r_s)
                for r_list in by_sub.values():
                    for u, v in zip(r_list, r_list[1:]):
                        _align_same_y(u, v)

        for d_list in diff_by_grp.values():
            for dm_a, dm_b in zip(d_list, d_list[1:]):
                _align_same_y(dm_a.get("left"), dm_b.get("left"))
                _align_same_y(dm_a.get("loadLeft"), dm_b.get("loadLeft"))
                _align_same_y(dm_a.get("tail"), dm_b.get("tail"))

        # Align matched current-mirror MOS sharing `(polarity, G, S_rail)` at column extremes,
        # plus matched cascode-mirror MOS sharing `(polarity, G, tier)`
        mirror_buckets: dict[tuple[str, str, str], list[str]] = defaultdict(list)
        cas_buckets: dict[tuple[str, str, int], list[str]] = defaultdict(list)
        for r in node_ids:
            inst = inst_map[r]
            if inst.device_class != "mos":
                continue
            g_net = inst.terminals.get("G", "")
            s_net = inst.terminals.get("S", "")
            if g_net and g_net not in rails:
                col_r = columns[col_of[r]]
                if s_net in supplies and col_r[0] == r:
                    mirror_buckets[(inst.polarity or "", g_net, s_net)].append(r)
                elif s_net in grounds and col_r[-1] == r:
                    mirror_buckets[(inst.polarity or "", g_net, s_net)].append(r)
                elif tiers[r] in {2, 4}:
                    cas_buckets[(inst.polarity or "", g_net, tiers[r])].append(r)
        for m_refs in mirror_buckets.values():
            for u, v in zip(m_refs, m_refs[1:]):
                _align_same_y(u, v)
        for c_refs in cas_buckets.values():
            for u, v in zip(c_refs, c_refs[1:]):
                _align_same_y(u, v)

        # 4) Cross-column non-rail signal pin alignment springs (shortest connection priority!)
        has_sig_spring: set[str] = set()
        for net in circuit.nets:
            if net in rails or intent.net_policies.get(net) == "perimeter_bus":
                continue
            conn = [
                (r, p, rel_pins[r][p][1])
                for r in node_ids
                for p, n in inst_map[r].terminals.items()
                if n == net and p != "B"
            ]
            if not (2 <= len(conn) <= 6):
                continue
            base_w = 4.8 / float(len(conn) - 1)
            for idx_a, (ra, pa, dya) in enumerate(conn):
                for rb, pb, dyb in conn[idx_a + 1 :]:
                    col_dist = abs(col_of[ra] - col_of[rb])
                    if 1 <= col_dist <= 2:
                        gate_boost = 1.35 if (pa == "G" or pb == "G") else 1.0
                        w_pair = base_w * gate_boost * (1.0 if col_dist == 1 else 0.65)
                        _add_pin_y_spring(ra, dya, rb, dyb, w_pair)
                        has_sig_spring.add(ra)
                        has_sig_spring.add(rb)

        # 5) Outer supply/ground pin alignment springs:
        #    - Strong (`2.8`) for rail-shunt passives or singletons with no mid-sheet signal springs (e.g. RESET, C1, CL)
        #    - Gentle (`0.45`) for active signal branches (e.g. M8, M9, M3 in bias_ini) so mutual signal connections win!
        top_vdd_pins: list[tuple[str, int]] = []
        bot_vss_pins: list[tuple[str, int]] = []
        for col in columns:
            r_top = col[0]
            for p, n in inst_map[r_top].terminals.items():
                if p != "B" and n in supplies and rel_pins[r_top][p][1] <= 0:
                    top_vdd_pins.append((r_top, rel_pins[r_top][p][1]))
                    break
            r_bot = col[-1]
            for p, n in inst_map[r_bot].terminals.items():
                if p != "B" and n in grounds and rel_pins[r_bot][p][1] >= 0:
                    bot_vss_pins.append((r_bot, rel_pins[r_bot][p][1]))
                    break

        max_col_len = max((len(c) for c in columns), default=1)
        for rail_pin_list in (top_vdd_pins, bot_vss_pins):
            for (ra, dya), (rb, dyb) in zip(rail_pin_list, rail_pin_list[1:]):
                la, lb = len(columns[col_of[ra]]), len(columns[col_of[rb]])
                a_shunt = (ra not in has_sig_spring) or (
                    inst_map[ra].device_class not in {"mos", "subckt"} and la == 1
                )
                b_shunt = (rb not in has_sig_spring) or (
                    inst_map[rb].device_class not in {"mos", "subckt"} and lb == 1
                )
                short_active_branch = (
                    max_col_len >= 4
                    and min(la, lb) <= max_col_len - 2
                    and not a_shunt
                    and not b_shunt
                )
                w_rail = 0.35 if short_active_branch else 3.6
                _add_pin_y_spring(ra, dya, rb, dyb, w_rail)

        solved_inst_y = solve_1d_compaction_qp(
            node_ids=node_ids,
            initial_pos=init_y_inst,
            extents=extents_y_inst,
            separations=sep_y_inst,
            target_springs=springs_y_inst,
            equalities=eq_y_inst,
            anchor_weight=0.02,
        )

    # Normalize coordinates so top-left starts cleanly around (28, 24)
    raw_min_x = min(solved_x[r] + rel_env[r][0] for r in node_ids)
    raw_min_y = min(solved_inst_y[r] + rel_env[r][1] for r in node_ids)
    shift_x = 28 - raw_min_x
    shift_y = 24 - raw_min_y

    bulk_label_by_ref = {item["reference"]: item["netName"] for item in local_bulk_labels}
    bulk_short_refs = {item["reference"] for item in local_bulk_shorts}

    placements: dict[str, PlacedInstance] = {}
    for inst in circuit.instances:
        r = inst.ref
        px = solved_x[r] + shift_x
        py = solved_inst_y[r] + shift_y
        rot, mir, cad, sem = transforms[r]
        pins_abs = {p: (px + dx, py + dy) for p, (dx, dy) in rel_pins[r].items()}
        bx0, by0, bx1, by1 = rel_body[r]
        lx0, ly0, lx1, ly1 = rel_label[r]
        ex0, ey0, ex1, ey1 = rel_env[r]

        placements[r] = PlacedInstance(
            ref=r,
            instance_id=f"source-{r}",
            device_class=inst.device_class,
            kind=inst.kind,
            polarity=inst.polarity,
            device_key=inst.device_key,
            x=px,
            y=py,
            rotation=rot,
            mirror=mir,
            cadence_orient=cad,
            semantic_orient=sem,
            pin_points=pins_abs,
            body_box=(px + bx0, py + by0, px + bx1, py + by1),
            label_box=(px + lx0, py + ly0, px + lx1, py + ly1),
            full_envelope=(px + ex0, py + ey0, px + ex1, py + ey1),
            label_lines=label_lines_map[r],
            tier=tiers[r],
            group_id=group_of[r],
            has_bulk_short=r in bulk_short_refs,
            bulk_label_net=bulk_label_by_ref.get(r),
        )

    # --- Place Port Occurrences (Local Supply/Ground Taps + Clean Left/Right Signal Pins) ---
    placed_ports: list[PortOccurrencePlaced] = []
    min_env_x = min(p.full_envelope[0] for p in placements.values())
    max_env_x = max(p.full_envelope[2] for p in placements.values())
    min_env_y = min(p.full_envelope[1] for p in placements.values())
    max_env_y = max(p.full_envelope[3] for p in placements.values())

    # Nearest-boundary port side adaptation: don't wrap a far-left output port across the entire schematic!
    span_env_w = max_env_x - min_env_x
    effective_port_side: dict[str, str] = {}
    for p in circuit.ports:
        raw_side = intent.port_sides.get(
            p.name,
            "local_repeat" if p.net in rails else ("right" if p.direction == "output" else "left"),
        )
        if intent.authored_by == "analyzer_draft" and raw_side in {"left", "right"} and span_env_w >= 70:
            c_xs = [
                placements[inst.ref].pin_points[pin][0]
                for inst in circuit.instances
                for pin, net in inst.terminals.items()
                if net == p.net and pin != "B"
            ]
            if c_xs and raw_side == "right" and max(c_xs) <= min_env_x + 0.35 * span_env_w:
                raw_side = "left"
        effective_port_side[p.name] = raw_side

    n_left_edge = sum(
        1 for p in circuit.ports if p.net not in rails and effective_port_side[p.name] == "left"
    )
    n_right_edge = sum(
        1 for p in circuit.ports if p.net not in rails and effective_port_side[p.name] == "right"
    )
    left_port_x = min_env_x - max(intent.spacing.port_margin_x, 6 + 2 * n_left_edge)
    right_port_x = max_env_x + max(intent.spacing.port_margin_x, 6 + 2 * n_right_edge)
    used_left_ys: set[int] = set()
    used_right_ys: set[int] = set()
    used_port_owner: dict[Point, str] = {
        pt: inst.terminals[pin]
        for inst in circuit.instances
        for pin, pt in placements[inst.ref].pin_points.items()
    }

    min_phys_y = min(min(p.body_box[1], p.label_box[1]) for p in placements.values())
    max_phys_y = max(max(p.body_box[3], p.label_box[3]) for p in placements.values())
    mid_phys_y = (min_phys_y + max_phys_y) / 2.0
    next_top_rail_y = min_phys_y - 3
    next_bot_rail_y = max_phys_y + 3

    has_top_perim_bus = False
    has_bot_perim_bus = False
    for b_net, pol in intent.net_policies.items():
        if pol != "perimeter_bus":
            continue
        b_ys = [
            placements[inst.ref].pin_points[p][1]
            for inst in circuit.instances
            for p, n in inst.terminals.items()
            if n == b_net and p != "B" and p in placements[inst.ref].pin_points
        ]
        if b_ys:
            if (sum(b_ys) / len(b_ys)) <= mid_phys_y:
                has_top_perim_bus = True
            else:
                has_bot_perim_bus = True

    def _box_hits_seg(box: BBox, x0: int, y0: int, x1: int, y1: int, pad: int = 1) -> bool:
        return (
            max(x0, box[0] - pad) <= min(x1, box[2] + pad)
            and max(y0, box[1] - pad) <= min(y1, box[3] + pad)
        )

    def _vert_clear_to_rail(ax: int, y_from: int, y_to: int, own_ref: str, net_n: str) -> bool:
        ya, yb = min(y_from, y_to), max(y_from, y_to)
        own_pl = placements[own_ref]
        if _box_hits_seg(own_pl.label_box, ax, ya, ax, yb, pad=0):
            return False
        for r_o, b_o in placements.items():
            if r_o == own_ref:
                continue
            if _box_hits_seg(b_o.body_box, ax, ya, ax, yb, pad=1) or _box_hits_seg(b_o.label_box, ax, ya, ax, yb, pad=1):
                return False
        for cy in range(ya, yb + 1):
            if cy != y_from and used_port_owner.get((ax, cy)) not in (None, net_n):
                return False
        return True

    def _horiz_clear_on_rail(x_a: int, x_b: int, y_r: int, net_n: str) -> bool:
        xa, xb = min(x_a, x_b), max(x_a, x_b)
        for b_o in placements.values():
            if _box_hits_seg(b_o.body_box, xa, y_r, xb, y_r, pad=1) or _box_hits_seg(b_o.label_box, xa, y_r, xb, y_r, pad=1):
                return False
        for cx in range(xa, xb + 1):
            if used_port_owner.get((cx, y_r)) not in (None, net_n):
                return False
        return True

    for port in circuit.ports:
        side = effective_port_side[port.name]
        if port.net in rails:
            is_vdd = port.net in supplies
            sym_id = "vdd-port" if is_vdd else "ground"
            rot = 90 if is_vdd else 270
            occ_idx = 0

            rail_terms: list[tuple[int, int, bool, str]] = []
            for inst in circuit.instances:
                pl = placements[inst.ref]
                for pin, net in inst.terminals.items():
                    if net != port.net or pin == "B":
                        continue
                    ax, ay = pl.pin_points[pin]
                    go_up = is_vdd if (ay <= pl.y) else (False if ay > pl.y else is_vdd)
                    rail_terms.append((ax, ay, go_up, inst.ref))

            if not rail_terms:
                continue

            # 1) Merge unobstructed outward terminals onto ONE shared top VDD rail or bottom VSS rail + single symbol!
            unobstructed_global: list[tuple[int, int, bool, str]] = []
            interior_terms: list[tuple[int, int, bool, str]] = []
            global_rail_y = next_top_rail_y if is_vdd else next_bot_rail_y
            for ax, ay, go_up, ref_n in rail_terms:
                if go_up == is_vdd and (
                    (is_vdd and ay >= global_rail_y and _vert_clear_to_rail(ax, ay, global_rail_y, ref_n, port.net))
                    or (not is_vdd and ay <= global_rail_y and _vert_clear_to_rail(ax, ay, global_rail_y, ref_n, port.net))
                ):
                    unobstructed_global.append((ax, ay, go_up, ref_n))
                else:
                    interior_terms.append((ax, ay, go_up, ref_n))

            if unobstructed_global:
                g_xs = sorted({t[0] for t in unobstructed_global})
                max_outer_gap = 40 if (has_top_perim_bus if is_vdd else has_bot_perim_bus) else 100000
                g_clusters: list[list[int]] = [[g_xs[0]]]
                for nxt_gx in g_xs[1:]:
                    if (nxt_gx - g_clusters[-1][-1] <= max_outer_gap) and _horiz_clear_on_rail(
                        g_clusters[-1][-1], nxt_gx, global_rail_y, port.net
                    ):
                        g_clusters[-1].append(nxt_gx)
                    else:
                        g_clusters.append([nxt_gx])

                extreme_sy = global_rail_y
                for cl_gxs in g_clusters:
                    if len(cl_gxs) == 1:
                        sx, sy = cl_gxs[0], global_rail_y
                    else:
                        sx = (cl_gxs[0] + cl_gxs[-1]) // 2
                        if sx in cl_gxs and (cl_gxs[-1] - cl_gxs[0]) >= 4:
                            sx += 2
                        sy = global_rail_y + (-2 if is_vdd else 2)
                        while used_port_owner.get((sx, sy)) not in (None, port.net):
                            sx += 2
                        for gx in range(cl_gxs[0], cl_gxs[-1] + 1):
                            used_port_owner.setdefault((gx, global_rail_y), port.net)
                    used_port_owner[(sx, sy)] = port.net
                    occ_idx += 1
                    placed_ports.append(
                        PortOccurrencePlaced(
                            occurrence_id=f"P_{port.name}_{occ_idx}",
                            port_name=port.name,
                            net_name=port.net,
                            direction=port.direction,
                            symbol_id=sym_id,
                            x=sx,
                            y=sy,
                            rotation=rot,
                            mirror="none",
                        )
                    )
                    extreme_sy = min(extreme_sy, sy) if is_vdd else max(extreme_sy, sy)
                if is_vdd:
                    next_top_rail_y = min(next_top_rail_y, extreme_sy - 3)
                else:
                    next_bot_rail_y = max(next_bot_rail_y, extreme_sy + 3)

            # 2) For interior terminals blocked from the outer rail (e.g. stacked sub-rows), merge nearby clear taps!
            local_taps: list[tuple[int, int, bool, int, str]] = []
            for ax, ay, go_up, ref_n in interior_terms:
                ty = (ay - 3) if go_up else (ay + 3)
                while not _vert_clear_to_rail(ax, ay, ty, ref_n, port.net) or (
                    used_port_owner.get((ax, ty)) not in (None, port.net)
                ):
                    ty += -2 if go_up else 2
                local_taps.append((ax, ty, go_up, ay, ref_n))

            # Group `local_taps` by `(ty, go_up)` and step slightly past subckt label boxes if that merges a sub-row!
            by_ty: dict[tuple[int, bool], list[tuple[int, int, str]]] = defaultdict(list)
            seen_ty_x: set[tuple[int, bool, int]] = set()
            for ax, ty, go_up, ay, ref_n in local_taps:
                if (ty, go_up, ax) not in seen_ty_x:
                    seen_ty_x.add((ty, go_up, ax))
                    by_ty[(ty, go_up)].append((ax, ay, ref_n))

            for (ty, go_up), items_raw in sorted(by_ty.items()):
                items_sorted = sorted(items_raw, key=lambda it: it[0])
                xs_sorted = [it[0] for it in items_sorted]
                ty_eff = ty
                if len(xs_sorted) >= 2:
                    best_clusters_n = len(xs_sorted)
                    for step_k in (0, 1, 2, 3):
                        ty_try = ty + (-2 * step_k if go_up else 2 * step_k)
                        if not all(
                            _vert_clear_to_rail(ax_i, ay_i, ty_try, ref_i, port.net)
                            for ax_i, ay_i, ref_i in items_sorted
                        ):
                            continue
                        tmp_cl: list[list[int]] = [[xs_sorted[0]]]
                        for nxt_x in xs_sorted[1:]:
                            if (nxt_x - tmp_cl[-1][-1] <= 44) and _horiz_clear_on_rail(
                                tmp_cl[-1][-1], nxt_x, ty_try, port.net
                            ):
                                tmp_cl[-1].append(nxt_x)
                            else:
                                tmp_cl.append([nxt_x])
                        if len(tmp_cl) < best_clusters_n:
                            best_clusters_n = len(tmp_cl)
                            ty_eff = ty_try
                            if best_clusters_n == 1:
                                break

                clusters_x: list[list[int]] = [[xs_sorted[0]]]
                for nxt_x in xs_sorted[1:]:
                    if (nxt_x - clusters_x[-1][-1] <= 44) and _horiz_clear_on_rail(
                        clusters_x[-1][-1], nxt_x, ty_eff, port.net
                    ):
                        clusters_x[-1].append(nxt_x)
                    else:
                        clusters_x.append([nxt_x])
                for cl_xs in clusters_x:
                    if len(cl_xs) == 1:
                        sx, sy = cl_xs[0], ty_eff
                    else:
                        mid_cx = (cl_xs[0] + cl_xs[-1]) // 2
                        cand_sy = ty_eff + (-2 if go_up else 2)
                        sx, sy = cl_xs[0], ty_eff
                        for dx_try in (0, 2, -2, 4, -4, 8, -8, 10, -10):
                            try_sx = mid_cx + dx_try
                            if cl_xs[0] <= try_sx <= cl_xs[-1] and (
                                try_sx not in cl_xs or (cl_xs[-1] - cl_xs[0]) < 4
                            ):
                                if _horiz_clear_on_rail(try_sx, try_sx, cand_sy, port.net) and _horiz_clear_on_rail(
                                    try_sx, try_sx, ty_eff, port.net
                                ):
                                    sx, sy = try_sx, cand_sy
                                    break
                        for gx in range(cl_xs[0], cl_xs[-1] + 1):
                            used_port_owner.setdefault((gx, ty_eff), port.net)
                    if used_port_owner.get((sx, sy)) == port.net and any(
                        po.x == sx and po.y == sy and po.net_name == port.net for po in placed_ports
                    ):
                        continue
                    used_port_owner[(sx, sy)] = port.net
                    occ_idx += 1
                    placed_ports.append(
                        PortOccurrencePlaced(
                            occurrence_id=f"P_{port.name}_{occ_idx}",
                            port_name=port.name,
                            net_name=port.net,
                            direction=port.direction,
                            symbol_id=sym_id,
                            x=sx,
                            y=sy,
                            rotation=rot,
                            mirror="none",
                        )
                    )
            continue

        if side in {"near_pin", "local_repeat"}:
            # Place clean port flag adjacent to each connected terminal without crossing `label_box` or foreign ports!
            occ_idx = 0
            for inst in circuit.instances:
                pl = placements[inst.ref]
                bx0, by0, bx1, by1 = pl.body_box
                for pin, net in inst.terminals.items():
                    if net != port.net or pin == "B":
                        continue
                    ax, ay = pl.pin_points[pin]
                    if ax <= bx0:
                        tx, ty, step_dx, step_dy, rot_p, mir_flag = ax - 4, ay, -2, 0, 0, "none"
                    elif ax >= bx1:
                        tx, ty, step_dx, step_dy, rot_p, mir_flag = ax + 4, ay, 2, 0, 0, "horizontal"
                    elif ay <= by0:
                        tx, ty, step_dx, step_dy, rot_p, mir_flag = ax, ay - 4, 0, -2, 90, "none"
                    else:
                        tx, ty, step_dx, step_dy, rot_p, mir_flag = ax, ay + 4, 0, 2, 270, "none"
                    while (used_port_owner.get((tx, ty)) not in (None, port.net)) or any(
                        b.full_envelope[0] - 1 <= tx <= b.full_envelope[2] + 1
                        and b.full_envelope[1] - 1 <= ty <= b.full_envelope[3] + 1
                        for r_other, b in placements.items()
                        if r_other != inst.ref
                    ):
                        tx += step_dx
                        ty += step_dy
                    if used_port_owner.get((tx, ty)) == port.net:
                        continue
                    used_port_owner[(tx, ty)] = port.net
                    occ_idx += 1
                    occ_id = f"P_{port.name}" if side == "near_pin" and occ_idx == 1 else f"P_{port.name}_{occ_idx}"
                    placed_ports.append(
                        PortOccurrencePlaced(
                            occurrence_id=occ_id,
                            port_name=port.name,
                            net_name=port.net,
                            direction=port.direction,
                            symbol_id="port",
                            x=tx,
                            y=ty,
                            rotation=rot_p,
                            mirror=mir_flag,
                        )
                    )
            continue

        # Signal/Bias edge port: align Y with the primary connected terminal so input/output enters straight!
        connected_ys = [
            placements[inst.ref].pin_points[pin][1]
            for inst in circuit.instances
            for pin, net in inst.terminals.items()
            if net == port.net and pin != "B"
        ]
        pref_y = int(round(sum(connected_ys) / len(connected_ys))) if connected_ys else (min_env_y + max_env_y) // 2

        if side == "right":
            py = pref_y
            while any(abs(py - uy) < 4 for uy in used_right_ys):
                py += 4
            used_right_ys.add(py)
            placed_ports.append(
                PortOccurrencePlaced(
                    occurrence_id=f"P_{port.name}",
                    port_name=port.name,
                    net_name=port.net,
                    direction=port.direction,
                    symbol_id="port",
                    x=right_port_x,
                    y=py,
                    rotation=0,
                    mirror="horizontal",
                )
            )
        elif side == "top":
            placed_ports.append(
                PortOccurrencePlaced(
                    occurrence_id=f"P_{port.name}",
                    port_name=port.name,
                    net_name=port.net,
                    direction=port.direction,
                    symbol_id="port",
                    x=min_env_x + len(placed_ports) * 6,
                    y=min_env_y - intent.spacing.port_margin_y,
                    rotation=90,
                    mirror="none",
                )
            )
        elif side == "bottom":
            placed_ports.append(
                PortOccurrencePlaced(
                    occurrence_id=f"P_{port.name}",
                    port_name=port.name,
                    net_name=port.net,
                    direction=port.direction,
                    symbol_id="port",
                    x=min_env_x + len(placed_ports) * 6,
                    y=max_env_y + intent.spacing.port_margin_y,
                    rotation=270,
                    mirror="none",
                )
            )
        else:
            # Default: left side input port
            py = pref_y
            while any(abs(py - uy) < 4 for uy in used_left_ys):
                py += 4
            used_left_ys.add(py)
            placed_ports.append(
                PortOccurrencePlaced(
                    occurrence_id=f"P_{port.name}",
                    port_name=port.name,
                    net_name=port.net,
                    direction=port.direction,
                    symbol_id="port",
                    x=left_port_x,
                    y=py,
                    rotation=0,
                    mirror="none",
                )
            )

    return placements, placed_ports, local_bulk_labels, local_bulk_shorts
