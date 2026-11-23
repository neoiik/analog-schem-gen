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

    for inst in circuit.instances:
        sem = intent.orientations.get(
            inst.ref,
            "gate_left" if inst.device_class == "mos" else "vertical",
        )
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

    # Within each vertical column, align primary Drain/P pin X coordinates!
    for col in columns:
        if len(col) >= 2:
            anchor_ref = col[0]
            anchor_dx = rel_pins[anchor_ref][_alignment_pin_for_column(inst_map[anchor_ref])][0]
            for other_ref in col[1:]:
                # Only align if they don't occupy the exact same (tier, sub_idx)
                other_dx = rel_pins[other_ref][_alignment_pin_for_column(inst_map[other_ref])][0]
                # x[other] + other_dx == x[anchor] + anchor_dx  =>  x[other] - x[anchor] == anchor_dx - other_dx
                eq_x.append(({other_ref: 1.0, anchor_ref: -1.0}, float(anchor_dx - other_dx)))

    # Add symmetric triples for diff pairs!
    for m in intent.motifs:
        if m.get("type") == "diff_pair":
            l_ref = m.get("left")
            r_ref = m.get("right")
            t_ref = m.get("tail")
            if l_ref in inst_map and r_ref in inst_map and t_ref in inst_map:
                dl = rel_pins[l_ref]["S"][0]
                dr = rel_pins[r_ref]["S"][0]
                dt = rel_pins[t_ref]["D"][0]
                sym_x.append((l_ref, t_ref, r_ref, dl, dt, dr))

    extra_col_gaps = extra_col_gaps or {}
    # Column-to-column separation & group compaction springs
    for ci in range(len(columns) - 1):
        col_left = columns[ci]
        col_right = columns[ci + 1]
        same_group = group_of[col_left[0]] == group_of[col_right[0]]
        base_target = (
            intent.spacing.intra_group_gap_x if same_group else intent.spacing.inter_group_gap_x
        )
        corridor_bonus = extra_col_gaps.get((ci, ci + 1), 0)
        min_gap = max(intent.spacing.min_clearance_x, (base_target // 2)) + corridor_bonus
        target_gap = max(base_target, min_gap)
        spring_w = 6.0 if same_group else 2.5

        # Enforce pairwise clearance across adjacent columns; also protect non-adjacent columns in same tier
        for u in col_left:
            for v in col_right:
                sep_x.append((u, v, min_gap))
        springs_x.append((col_left[0], col_right[0], target_gap, spring_w))

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

    # Normalize coordinates so top-left starts cleanly around (28, 24)
    raw_min_x = min(solved_x[r] + rel_env[r][0] for r in node_ids)
    raw_min_y = min(solved_row_y[row_id_of[effective_row_key[r]]] + rel_env[r][1] for r in node_ids)
    shift_x = 28 - raw_min_x
    shift_y = 24 - raw_min_y

    bulk_label_by_ref = {item["reference"]: item["netName"] for item in local_bulk_labels}
    bulk_short_refs = {item["reference"] for item in local_bulk_shorts}

    placements: dict[str, PlacedInstance] = {}
    for inst in circuit.instances:
        r = inst.ref
        px = solved_x[r] + shift_x
        py = solved_row_y[row_id_of[effective_row_key[r]]] + shift_y
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

    left_port_x = min_env_x - intent.spacing.port_margin_x
    right_port_x = max_env_x + intent.spacing.port_margin_x
    used_left_ys: set[int] = set()
    used_right_ys: set[int] = set()
    used_supply_xy: set[tuple[str, int, int]] = set()

    for port in circuit.ports:
        side = intent.port_sides.get(
            port.name,
            "local_repeat" if port.net in rails else ("right" if port.direction == "output" else "left"),
        )
        if port.net in rails:
            is_vdd = port.net in supplies
            sym_id = "vdd-port" if is_vdd else "ground"
            rot = 90 if is_vdd else 270
            # Create one crisp vertical local supply/ground tap for each non-bulk terminal connected to this rail!
            occ_idx = 0
            for inst in circuit.instances:
                pl = placements[inst.ref]
                for pin, net in inst.terminals.items():
                    if net != port.net or pin == "B":
                        continue
                    ax, ay = pl.pin_points[pin]
                    ty = (ay - 4) if is_vdd else (ay + 4)
                    while any(
                        b.full_envelope[0] - 1 <= ax <= b.full_envelope[2] + 1
                        and b.full_envelope[1] - 1 <= ty <= b.full_envelope[3] + 1
                        for b in placements.values()
                    ):
                        ty += -2 if is_vdd else 2
                    key = (port.name, ax, ty)
                    if key in used_supply_xy:
                        continue
                    used_supply_xy.add(key)
                    occ_idx += 1
                    placed_ports.append(
                        PortOccurrencePlaced(
                            occurrence_id=f"P_{port.name}_{occ_idx}",
                            port_name=port.name,
                            net_name=port.net,
                            direction=port.direction,
                            symbol_id=sym_id,
                            x=ax,
                            y=ty,
                            rotation=rot,
                            mirror="none",
                        )
                    )
            continue

        if side in {"near_pin", "local_repeat"}:
            # Place clean horizontal port flag directly adjacent to each connected terminal (e.g. diff pair VINP/VINN or cascode bias!)
            occ_idx = 0
            for inst in circuit.instances:
                pl = placements[inst.ref]
                for pin, net in inst.terminals.items():
                    if net != port.net or pin == "B":
                        continue
                    ax, ay = pl.pin_points[pin]
                    faces_right = ax > pl.x
                    tx = (ax + 4) if faces_right else (ax - 4)
                    mir_flag = "horizontal" if faces_right else "none"
                    key = (port.name, tx, ay)
                    if key in used_supply_xy:
                        continue
                    used_supply_xy.add(key)
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
                            y=ay,
                            rotation=0,
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
