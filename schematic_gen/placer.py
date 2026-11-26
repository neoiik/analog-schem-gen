"""Constrained placement engine: Micro-templates + Convex QP compaction/expansion + integer symmetry."""

from __future__ import annotations

from collections import defaultdict
import math
from typing import Any

import numpy as np
from scipy.optimize import minimize

from schematic_gen.topology_core import (
    assign_default_tiers as _assign_default_tiers,
    detect_co_gated_mos_stacks,
    is_nonpolar_2pin_class,
)
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
    equidistant_triples: list[tuple[str, str, str, float]] | None = None,
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

    # --- Exact Algebraic Elimination of Rigid 2-Variable Equalities (`x[k1] - x[k2] == delta`) ---
    # Reduces large 60+ device QPs with 500+ pairwise column inequalities down to a compact,
    # full-rank cluster QP (~15-30 variables, ~25 non-redundant inequalities) so SLSQP converges
    # reliably instead of failing with `status=8` on rank-deficient duplicate constraints!
    rigid_adj_f: dict[str, list[tuple[str, float]]] = defaultdict(list)
    rigid_adj: dict[str, list[tuple[str, int]]] = defaultdict(list)
    general_equalities: list[tuple[dict[str, float], float]] = []

    for coeff_map, rhs in equalities:
        nz = [(k, float(c)) for k, c in coeff_map.items() if k in idx_of and abs(float(c)) > 1e-12]
        if len(nz) == 2:
            (k1, c1), (k2, c2) = nz[0], nz[1]
            if math.isclose(c1 + c2, 0.0, abs_tol=1e-9):
                delta_f = float(rhs) / c1
                delta_i = int(round(delta_f))
                rigid_adj_f[k2].append((k1, delta_f))
                rigid_adj_f[k1].append((k2, -delta_f))
                rigid_adj[k2].append((k1, delta_i))
                rigid_adj[k1].append((k2, -delta_i))
                continue
        if nz:
            general_equalities.append(({k: c for k, c in nz}, float(rhs)))

    visited_r: set[str] = set()
    cluster_of: dict[str, int] = {}
    clusters: list[dict[str, int]] = []
    off_f: dict[str, float] = {}
    for u in node_ids:
        if u in visited_r:
            continue
        cid = len(clusters)
        rel_f: dict[str, float] = {u: 0.0}
        rel_i: dict[str, int] = {u: 0}
        queue_r = [u]
        visited_r.add(u)
        for cur in queue_r:
            for (nxt, df), (_, di) in zip(rigid_adj_f[cur], rigid_adj[cur]):
                if nxt not in visited_r:
                    visited_r.add(nxt)
                    rel_f[nxt] = rel_f[cur] + df
                    rel_i[nxt] = rel_i[cur] + di
                    queue_r.append(nxt)
        for node in rel_f:
            cluster_of[node] = cid
            off_f[node] = rel_f[node]
        clusters.append(rel_i)

    m = len(clusters)
    y0 = np.zeros(m, dtype=float)
    for cid, rel_i in enumerate(clusters):
        y0[cid] = sum(x0[idx_of[node]] - off_f[node] for node in rel_i) / float(len(rel_i))

    # Build reduced quadratic objective: 0.5 * y^T H y + f^T y
    H = np.zeros((m, m), dtype=float)
    f = np.zeros(m, dtype=float)

    for cid, rel_i in enumerate(clusters):
        # Normalize gentle anchor regularization per rigid cluster so tall 8-device stacks
        # do not experience 8x heavier drag toward arbitrary initial `x0` than 1-device columns.
        w2_anc = 2.0 * float(anchor_weight) / float(len(rel_i))
        for u in rel_i:
            H[cid, cid] += w2_anc
            f[cid] += w2_anc * (off_f[u] - x0[idx_of[u]])

    for u, v, ideal_gap, weight in target_springs:
        if u not in idx_of or v not in idx_of or weight <= 0:
            continue
        cu, cv = cluster_of[u], cluster_of[v]
        if cu == cv:
            continue
        # Want (y[cv] + off_f[v]) - (y[cu] + off_f[u]) == right_ext[u] + left_ext[v] + ideal_gap
        desired_diff = float(extents[u][1] + extents[v][0] + ideal_gap) - (off_f[v] - off_f[u])
        w2 = 2.0 * float(weight)
        H[cu, cu] += w2
        H[cv, cv] += w2
        H[cu, cv] -= w2
        H[cv, cu] -= w2
        f[cu] += w2 * desired_diff
        f[cv] -= w2 * desired_diff

    # 1D Foam / Bubble Equidistance energy:
    # minimize weight * ((x[mid] - x[u]) - (x[v] - x[mid]))^2 = weight * (x[u] - 2*x[mid] + x[v])^2
    for u, mid, v, weight in equidistant_triples or []:
        if u not in idx_of or mid not in idx_of or v not in idx_of or weight <= 0:
            continue
        cu, cm, cv = cluster_of[u], cluster_of[mid], cluster_of[v]
        if cu == cm == cv:
            continue
        c_vec: dict[int, float] = defaultdict(float)
        c_vec[cu] += 1.0
        c_vec[cm] -= 2.0
        c_vec[cv] += 1.0
        const_off = off_f[u] - 2.0 * off_f[mid] + off_f[v]
        w2 = 2.0 * float(weight)
        for ci_k, a_k in c_vec.items():
            if abs(a_k) <= 1e-12:
                continue
            f[ci_k] += w2 * a_k * const_off
            for cj_k, a_j in c_vec.items():
                if abs(a_j) <= 1e-12:
                    continue
                H[ci_k, cj_k] += w2 * a_k * a_j

    # Deduplicate separation inequalities per cluster pair `(cu, cv) -> max_req`
    max_sep_by_cluster: dict[tuple[int, int], float] = {}
    for u, v, min_gap in separations:
        if u not in idx_of or v not in idx_of or u == v:
            continue
        cu, cv = cluster_of[u], cluster_of[v]
        if cu == cv:
            continue
        req_c = float(extents[u][1] + extents[v][0] + min_gap) - (off_f[v] - off_f[u])
        prev_r = max_sep_by_cluster.get((cu, cv))
        if prev_r is None or req_c > prev_r:
            max_sep_by_cluster[(cu, cv)] = req_c

    # Build a feasible starting point `y_init` via Bellman-Ford relaxation on `max_sep_by_cluster`
    # when there are no general equalities (or gently push violations) so SLSQP starts clean
    y_init = y0.copy()
    for _ in range(m + 2):
        changed_f = False
        for (cu, cv), req_c in max_sep_by_cluster.items():
            if y_init[cv] - y_init[cu] < req_c:
                y_init[cv] = y_init[cu] + req_c
                changed_f = True
        if not changed_f:
            break

    def obj(y: np.ndarray) -> float:
        return float(0.5 * y @ (H @ y) + f @ y)

    def jac(y: np.ndarray) -> np.ndarray:
        return H @ y + f

    constraints: list[dict[str, Any]] = []
    for (cu, cv), req_c in max_sep_by_cluster.items():
        constraints.append(
            {
                "type": "ineq",
                "fun": lambda y, a=cu, b=cv, r=req_c: y[b] - y[a] - r,
                "jac": lambda y, a=cu, b=cv: _unit_diff_grad(m, a, b),
            }
        )

    seen_eq_rows: list[tuple[np.ndarray, float]] = []
    for coeff_map, rhs in general_equalities:
        row = np.zeros(m, dtype=float)
        rhs_adj = float(rhs)
        for var, c in coeff_map.items():
            if var in idx_of:
                row[cluster_of[var]] += float(c)
                rhs_adj -= float(c) * off_f[var]
        norm_r = float(np.linalg.norm(row))
        if norm_r <= 1e-12:
            continue
        row_u = row / norm_r
        rhs_u = rhs_adj / norm_r
        if any(
            np.allclose(row_u, pr, atol=1e-9) and math.isclose(rhs_u, pb, abs_tol=1e-9)
            or np.allclose(row_u, -pr, atol=1e-9) and math.isclose(rhs_u, -pb, abs_tol=1e-9)
            for pr, pb in seen_eq_rows
        ):
            continue
        seen_eq_rows.append((row_u, rhs_u))
        constraints.append(
            {
                "type": "eq",
                "fun": lambda y, r=row_u, b=rhs_u: float(r @ y - b),
                "jac": lambda y, r=row_u: r,
            }
        )

    def _max_cstr_viol(y_cand: np.ndarray) -> float:
        mv = 0.0
        for (cu, cv), req_c in max_sep_by_cluster.items():
            v_i = req_c - float(y_cand[cv] - y_cand[cu])
            if v_i > mv:
                mv = v_i
        for row_u, rhs_u in seen_eq_rows:
            v_e = abs(float(row_u @ y_cand - rhs_u))
            if v_e > mv:
                mv = v_e
        return mv

    res = minimize(
        obj,
        y_init,
        jac=jac,
        constraints=constraints,
        method="SLSQP",
        options={"ftol": 1e-9, "maxiter": 500},
    )
    if res.success:
        y_sol = res.x
    else:
        # Retry from original y0 with looser ftol, and accept any feasible lower-energy iterate
        res2 = minimize(
            obj,
            y0,
            jac=jac,
            constraints=constraints,
            method="SLSQP",
            options={"ftol": 1e-6, "maxiter": 500},
        )
        cands = [y_init]
        if res.x is not None and _max_cstr_viol(res.x) <= 1e-2:
            cands.append(res.x)
        if res2.x is not None and _max_cstr_viol(res2.x) <= 1e-2:
            cands.append(res2.x)
        y_sol = min(cands, key=obj)

    x_cont = np.array([float(y_sol[cluster_of[u]] + off_f[u]) for u in node_ids], dtype=float)

    # Exact integer quantization preserving rigid pin alignments, symmetric triples, and all separations!
    x_int: dict[str, int] = {}
    cluster_pos: list[int] = []
    for cid, rel_i in enumerate(clusters):
        anchor_val = int(round(float(y_sol[cid])))
        cluster_pos.append(anchor_val)
        for node, off_val in rel_i.items():
            x_int[node] = anchor_val + off_val

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
            cu, cv = cluster_of[u], cluster_of[v]
            if cu == cv:
                continue
            changed = True
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
    """Choose the primary vertical branch pin (`D`, `C`, or `P`) for straight vertical column alignment."""
    if inst.device_class == "mos":
        return "D"
    if inst.device_class == "bjt":
        return "C"
    if "P" in inst.terminals:
        return "P"
    return next(iter(inst.terminals))


def cluster_local_port_terminals(
    terminals: list[tuple[str, str, Point]],
    spacing: Any,
    placements: dict[str, PlacedInstance] | None = None,
) -> list[list[tuple[str, str, Point]]]:
    """Group nearby terminals of a shared port net into short-range local clusters.

    Terminals on the same device (e.g. diode-connected D+G) or within adjacent
    column/stack spacing are grouped into one cluster so they are physically
    connected with solid orthogonal wires, while distant branches remain separate
    clusters.
    """
    n = len(terminals)
    if n <= 1:
        return [list(terminals)] if terminals else []

    max_dx = max(36, 4 * spacing.intra_group_gap_x + 16)
    max_dy = max(24, 2 * spacing.intra_stack_gap_y + 10)
    max_manhattan = max(44, 4 * spacing.intra_group_gap_x + 2 * spacing.intra_stack_gap_y + 8)

    parent = list(range(n))

    def _find(i: int) -> int:
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    def _union(a: int, b: int) -> None:
        ra, rb = _find(a), _find(b)
        if ra != rb:
            parent[rb] = ra

    def _intermediate_cols_between(ref_a: str, ref_b: str, xa: int, xb: int) -> int:
        if not placements:
            return 0
        lo_x, hi_x = min(xa, xb), max(xa, xb)
        if hi_x - lo_x <= 12:
            return 0
        mid_col_xs: set[int] = set()
        for r_k, pl_k in placements.items():
            if r_k in {ref_a, ref_b}:
                continue
            if (lo_x + 5) < pl_k.x < (hi_x - 5):
                # Bucket nearby devices belonging to the same column (within 6 grids)
                if not any(abs(pl_k.x - cx) <= 6 for cx in mid_col_xs):
                    mid_col_xs.add(pl_k.x)
        return len(mid_col_xs)

    for i in range(n):
        ref_i, pin_i, (xi, yi) = terminals[i]
        for j in range(i + 1, n):
            ref_j, pin_j, (xj, yj) = terminals[j]
            if ref_i == ref_j:
                _union(i, j)
                continue
            dx = abs(xi - xj)
            dy = abs(yi - yj)
            # Same vertical column (direct unobstructed vertical alignment, e.g. M10.D -> M9.D),
            # same-row co-gated mirror pair (e.g. M25.G -- M26.G across <= 1 intermediate column),
            # or short-range adjacent columns/stacks stay in the same physically wired cluster.
            same_row_gate_pair = (
                pin_i == "G"
                and pin_j == "G"
                and dy == 0
                and dx <= max(60, max_dx + 20)
                and _intermediate_cols_between(ref_i, ref_j, xi, xj) <= 1
            )
            if dx <= 2 or same_row_gate_pair or (dx <= max_dx and dy <= max_dy and (dx + dy) <= max_manhattan):
                _union(i, j)

    groups: dict[int, list[tuple[str, str, Point]]] = defaultdict(list)
    for idx, term in enumerate(terminals):
        groups[_find(idx)].append(term)

    clusters = list(groups.values())
    for cl in clusters:
        cl.sort(key=lambda item: (item[2][0], item[2][1], item[0], item[1]))

    def _cluster_rank(cl: list[tuple[str, str, Point]]) -> tuple[int, int, int, int]:
        has_driver = any(pin != "G" for _, pin, _ in cl)
        min_x = min(pt[0] for _, _, pt in cl)
        min_y = min(pt[1] for _, _, pt in cl)
        return (-len(cl), 0 if has_driver else 1, min_x, min_y)

    clusters.sort(key=_cluster_rank)
    return clusters


def relax_lattice_tension_field(
    init_col_x: list[float],
    init_grid_y: list[list[float]],
    min_dx: list[int],
    min_dy: list[list[int]],
    *,
    relax_x: bool = True,
    relax_y: bool = True,
    anchor_bot_y: float | None = None,
    iterations: int = 40,
) -> tuple[list[int], list[list[int]]]:
    """2D abstract lattice short-range tension field relaxation (PBD alternating relaxation).

    Treats devices in a regular C x R block as abstract lattice nodes connected by
    short-range 4-neighbor edges with adaptive shared rest lengths (s_x, s_y):
      - Horizontal edges align rows (target dY = 0) and equalize adjacent column spacing (target dX = s_x).
      - Vertical edges equalize adjacent row spacing (target dY = s_y).
      - Boundary nodes use the shared rest length (s_x, s_y) so outer cells never shrink inward.
      - Hard non-overlap projections guarantee all minimum clearances (min_dx, min_dy) remain satisfied.
    """
    n_cols = len(init_col_x)
    n_rows = len(init_grid_y[0]) if n_cols > 0 and init_grid_y else 0
    if n_cols == 0 or n_rows == 0:
        return [], []

    # --- 1. Horizontal 1D short-range tension field on column axes ---
    if relax_x and n_cols >= 3 and min_dx:
        X = [float(v) for v in init_col_x]
        max_min_dx = float(max(min_dx))
        s_x = max(max_min_dx, (X[-1] - X[0]) / float(n_cols - 1))
        x0_anchor = X[0]

        for _ in range(iterations):
            X_next = list(X)
            # Left boundary: anchored gently near x0_anchor while balanced against X[1] - s_x (no shrinkage)
            target_0 = 0.55 * x0_anchor + 0.45 * (X[1] - s_x)
            X_next[0] = max(x0_anchor, 0.4 * X[0] + 0.6 * target_0)
            # Interior nodes: equal short-range tension from left (X[c-1] + s_x) and right (X[c+1] - s_x)
            for c in range(1, n_cols - 1):
                target_c = 0.5 * ((X[c - 1] + s_x) + (X[c + 1] - s_x))
                X_next[c] = 0.35 * X[c] + 0.65 * target_c
            # Right boundary: virtual outer mirror keeps distance s_x from X[n_cols - 2] (no shrinkage)
            target_last = X[n_cols - 2] + s_x
            X_next[n_cols - 1] = 0.35 * X[n_cols - 1] + 0.65 * target_last

            # Hard non-overlap projection sweep
            for c in range(n_cols - 1):
                if X_next[c + 1] - X_next[c] < float(min_dx[c]):
                    X_next[c + 1] = X_next[c] + float(min_dx[c])

            X = X_next
            mean_gap = (X[-1] - X[0]) / float(n_cols - 1)
            s_x = max(max_min_dx, 0.96 * mean_gap)

        pitch_x = int(math.ceil(s_x - 1e-6))
        pitch_x = max(pitch_x, max(min_dx))
        x0_int = int(round(X[0]))
        out_x = [x0_int + c * pitch_x for c in range(n_cols)]
    else:
        out_x = [int(round(v)) for v in init_col_x]

    # --- 2. Vertical 2D 4-neighbor short-range tension field on lattice nodes ---
    if relax_y and n_rows >= 2:
        Y = [[float(init_grid_y[c][r]) for r in range(n_rows)] for c in range(n_cols)]
        flat_min_dy = [min_dy[c][r] for c in range(n_cols) for r in range(n_rows - 1)]
        max_min_dy = float(max(flat_min_dy)) if flat_min_dy else 10.0
        avg_top_init = sum(Y[c][0] for c in range(n_cols)) / float(n_cols)
        avg_bot_init = sum(Y[c][n_rows - 1] for c in range(n_cols)) / float(n_cols)
        bot_anchor = float(anchor_bot_y) if anchor_bot_y is not None else avg_bot_init
        s_y = max(max_min_dy, (avg_bot_init - avg_top_init) / float(max(1, n_rows - 1)))

        w_h = 2.0  # horizontal row-alignment virtual tension
        w_v = 1.5  # vertical short-range neighbor tension

        for _ in range(iterations):
            Y_next = [list(col) for col in Y]
            for c in range(n_cols):
                for r in range(n_rows):
                    num = 0.0
                    den = 0.0
                    # Short-range horizontal row alignment edges
                    if c > 0:
                        num += w_h * Y[c - 1][r]
                        den += w_h
                    if c + 1 < n_cols:
                        num += w_h * Y[c + 1][r]
                        den += w_h
                    # Short-range vertical spacing edges with shared rest length s_y (prevents boundary shrinkage)
                    if r > 0:
                        num += w_v * (Y[c][r - 1] + s_y)
                        den += w_v
                    if r + 1 < n_rows:
                        num += w_v * (Y[c][r + 1] - s_y)
                        den += w_v
                    # Gentle boundary anchor so the whole matrix doesn't drift globally
                    if r == n_rows - 1:
                        w_b = 1.2 if anchor_bot_y is not None else 0.35
                        num += w_b * bot_anchor
                        den += w_b
                    elif r == 0 and anchor_bot_y is None:
                        num += 0.35 * avg_top_init
                        den += 0.35

                    target_y = num / den if den > 0 else Y[c][r]
                    Y_next[c][r] = 0.35 * Y[c][r] + 0.65 * target_y

            # Hard vertical non-overlap projection per column
            for c in range(n_cols):
                for r in range(n_rows - 1):
                    req_dy = float(min_dy[c][r])
                    if Y_next[c][r + 1] - Y_next[c][r] < req_dy:
                        mid = 0.5 * (Y_next[c][r] + Y_next[c][r + 1])
                        Y_next[c][r] = mid - 0.5 * req_dy
                        Y_next[c][r + 1] = mid + 0.5 * req_dy

            Y = Y_next
            gaps = [Y[c][r + 1] - Y[c][r] for c in range(n_cols) for r in range(n_rows - 1)]
            mean_dy = sum(gaps) / float(len(gaps)) if gaps else max_min_dy
            s_y = max(max_min_dy, 0.96 * mean_dy)

        pitch_y = int(math.ceil(s_y - 1e-6))
        if flat_min_dy:
            pitch_y = max(pitch_y, max(flat_min_dy))
        if anchor_bot_y is not None:
            y_bot_int = int(round(anchor_bot_y))
            y_top_int = y_bot_int - (n_rows - 1) * pitch_y
        else:
            y_top_int = int(round(sum(Y[c][0] for c in range(n_cols)) / float(n_cols)))
        out_y = [[y_top_int + r * pitch_y for r in range(n_rows)] for _ in range(n_cols)]
    else:
        out_y = [[int(round(init_grid_y[c][r])) for r in range(n_rows)] for c in range(n_cols)]

    return out_x, out_y


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
    # Distinguish wide circuit port flags (`port_flag_nets`, +/-11 grids) from compact internal
    # `label_stub` nets (`label_stub_nets`, +/-3 grids) so co-gated mirror columns do not get bloated!
    port_flag_nets: set[str] = set()
    label_stub_nets: set[str] = set()
    for p in circuit.ports:
        if p.net in rails:
            continue
        pside = intent.port_sides.get(p.name, "")
        if pside in {"near_pin", "local_repeat"}:
            port_flag_nets.add(p.net)
    if allow_wire_labels:
        for net, pol in intent.net_policies.items():
            if pol == "label_stub" and net not in rails:
                label_stub_nets.add(net)
    local_port_nets: set[str] = port_flag_nets | label_stub_nets

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

    # Ensure every non-serpentine column is ordered top-to-bottom:
    # First preserve exact series-connected vertical chains (e.g. VDDL -> M35 -> R2 -> R3 -> VSS)
    # so a middle series device is never jumped over by a coarse tier sort; fall back to tier sort otherwise.
    serp_inst_set = {
        str(x)
        for m in intent.motifs
        if m.get("type") == "serpentine_chain"
        for x in m.get("instances", [])
    }

    def _vertical_branch_nets(ref_v: str) -> set[str]:
        iv = inst_map[ref_v]
        if iv.device_class == "mos":
            return {iv.terminals[p] for p in ("D", "S") if p in iv.terminals} - rails
        if iv.device_class == "bjt":
            return {iv.terminals[p] for p in ("C", "E") if p in iv.terminals} - rails
        if iv.device_class != "subckt":
            return {iv.terminals[p] for p in ("P", "M") if p in iv.terminals} - rails
        return set()

    def _order_column_top_to_bottom(col_refs: list[str]) -> list[str]:
        if len(col_refs) < 2:
            return list(col_refs)
        vnets = {r: _vertical_branch_nets(r) for r in col_refs}
        adj: dict[str, set[str]] = {r: set() for r in col_refs}
        for i_a, ra in enumerate(col_refs):
            for rb in col_refs[i_a + 1 :]:
                if vnets[ra] & vnets[rb]:
                    adj[ra].add(rb)
                    adj[rb].add(ra)

        # Check if `col_refs` forms a single connected series chain (path graph: all deg <= 2, 2 endpoints)
        endpoints = [r for r in col_refs if len(adj[r]) == 1]
        if len(endpoints) == 2 and all(len(adj[r]) <= 2 for r in col_refs):
            chain = [endpoints[0]]
            seen_c = {endpoints[0]}
            while len(chain) < len(col_refs):
                nxts = [nb for nb in adj[chain[-1]] if nb not in seen_c]
                if not nxts:
                    break
                chain.append(nxts[0])
                seen_c.add(nxts[0])
            if len(chain) == len(col_refs):
                def _dir_score(seq: list[str]) -> float:
                    sc = 0.0
                    top_i, bot_i = inst_map[seq[0]], inst_map[seq[-1]]
                    if any(n in supplies and p != "B" for p, n in top_i.terminals.items()):
                        sc += 100.0
                    if any(n in grounds and p != "B" for p, n in bot_i.terminals.items()):
                        sc += 100.0
                    if any(n in grounds and p != "B" for p, n in top_i.terminals.items()):
                        sc -= 100.0
                    if any(n in supplies and p != "B" for p, n in bot_i.terminals.items()):
                        sc -= 100.0
                    # Pin-facing flow score along adjacent devices in `seq`
                    for u_r, v_r in zip(seq, seq[1:]):
                        iu, iv = inst_map[u_r], inst_map[v_r]
                        if iu.device_class == "mos":
                            u_bot = iu.terminals.get("D" if iu.polarity == "pmos" else "S", "")
                        elif iu.device_class == "bjt":
                            u_bot = iu.terminals.get("C" if iu.polarity == "pnp" else "E", "")
                        else:
                            u_bot = ""
                        if iv.device_class == "mos":
                            v_top = iv.terminals.get("S" if iv.polarity == "pmos" else "D", "")
                        elif iv.device_class == "bjt":
                            v_top = iv.terminals.get("E" if iv.polarity == "pnp" else "C", "")
                        else:
                            v_top = ""
                        if u_bot and v_top and u_bot == v_top:
                            sc += 20.0
                        elif u_bot and u_bot in vnets[v_r]:
                            sc += 10.0
                        elif v_top and v_top in vnets[u_r]:
                            sc += 10.0
                    # Prefer increasing tier index top-to-bottom as gentle tie-breaker
                    sc += sum((tiers[seq[k + 1]] - tiers[seq[k]]) for k in range(len(seq) - 1))
                    return sc

                rev = list(reversed(chain))
                return chain if _dir_score(chain) >= _dir_score(rev) else rev

        return sorted(col_refs, key=lambda r: tiers[r])

    for col in columns:
        if len(col) >= 2 and not any(r in serp_inst_set for r in col):
            col[:] = _order_column_top_to_bottom(col)

    # Long branches (> 5 devices) cannot map 1:1 onto the 5 potential tiers. Equal tiers inside a column are
    # resolved by sub-rows below, but a tier that DECREASES along the resolved top-to-bottom order would draw a
    # lower chain device above an upper one. Lift such tiers so every column is monotonic (no inversions).
    tier_inversions_fixed = 0
    for col in columns:
        if len(col) < 2 or any(r in serp_inst_set for r in col):
            continue
        for k in range(1, len(col)):
            if tiers[col[k]] < tiers[col[k - 1]]:
                tiers[col[k]] = tiers[col[k - 1]]
                tier_inversions_fixed += 1

    # Pre-build column neighbor map so stacked vertical 2-pin passives orient shared pins toward each other
    col_pos_map: dict[str, tuple[list[str], int]] = {
        r: (col, idx_c) for col in columns for idx_c, r in enumerate(col)
    }

    for inst in circuit.instances:
        if inst.device_class == "subckt":
            default_sem = "R0"
        elif inst.device_class in {"mos", "bjt"}:
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
        # Auto-orient non-polarized 2-pin passives (resistors / capacitors) by 180° so P/M face
        # their connected neighbors directly (0 U-turns around symbol bodies regardless of SPICE pin order).
        # Skip polarized elements (diodes, sources) and explicit serpentine_chain members.
        is_nonpolar_2pin = (
            is_nonpolar_2pin_class(inst.device_class)
            and "P" in inst.terminals
            and "M" in inst.terminals
            and inst.ref not in serp_inst_set
        )
        if is_nonpolar_2pin and sem in {"vertical", "vertical_flipped"}:
            p_net = inst.terminals.get("P", "")
            m_net = inst.terminals.get("M", "")
            if (m_net in supplies and p_net not in supplies) or (
                p_net in grounds and m_net not in grounds
            ):
                sem = "vertical_flipped"
            elif (p_net in supplies and m_net not in supplies) or (
                m_net in grounds and p_net not in grounds
            ):
                sem = "vertical"
            elif p_net not in rails and m_net not in rails:
                flip_vote = 0.0
                if inst.ref in col_pos_map:
                    col_members, idx_c = col_pos_map[inst.ref]
                    if idx_c > 0:
                        up_nets = set(inst_map[col_members[idx_c - 1]].terminals.values()) - rails
                        if m_net in up_nets and p_net not in up_nets:
                            flip_vote += 4.0
                        elif p_net in up_nets and m_net not in up_nets:
                            flip_vote -= 4.0
                    if idx_c + 1 < len(col_members):
                        dn_nets = set(inst_map[col_members[idx_c + 1]].terminals.values()) - rails
                        if p_net in dn_nets and m_net not in dn_nets:
                            flip_vote += 4.0
                        elif m_net in dn_nets and p_net not in dn_nets:
                            flip_vote -= 4.0
                if abs(flip_vote) < 1e-6:
                    # Fallback to nearby connected devices' vertical potential tiers
                    my_col = col_of[inst.ref]
                    p_tiers, m_tiers = [], []
                    for other in circuit.instances:
                        if other.ref == inst.ref:
                            continue
                        if abs(col_of[other.ref] - my_col) > 2:
                            continue
                        o_nets = {n for pn, n in other.terminals.items() if pn != "B"}
                        if p_net in o_nets:
                            p_tiers.append(tiers[other.ref])
                        if m_net in o_nets:
                            m_tiers.append(tiers[other.ref])
                    if p_tiers and m_tiers:
                        avg_pt = sum(p_tiers) / len(p_tiers)
                        avg_mt = sum(m_tiers) / len(m_tiers)
                        if avg_pt > avg_mt + 0.1:
                            flip_vote += 1.0
                        elif avg_mt > avg_pt + 0.1:
                            flip_vote -= 1.0
                if flip_vote > 0.0:
                    sem = "vertical_flipped"
                elif flip_vote < 0.0:
                    sem = "vertical"
        elif is_nonpolar_2pin and sem in {"horizontal_l2r", "horizontal_r2l"}:
            p_net = inst.terminals.get("P", "")
            m_net = inst.terminals.get("M", "")
            my_col = col_of[inst.ref]

            def _net_horiz_bias(target_net: str) -> float:
                if not target_net or target_net in rails:
                    return 0.0
                num, den = 0.0, 0.0
                for other in circuit.instances:
                    if other.ref == inst.ref:
                        continue
                    oc = col_of[other.ref]
                    if oc == my_col:
                        continue
                    if any(pn != "B" and n == target_net for pn, n in other.terminals.items()):
                        dist = abs(oc - my_col)
                        w = 1.0 / (1.0 + 0.6 * (dist - 1))
                        num += w * (1.0 if oc > my_col else -1.0)
                        den += w
                return (num / den) if den > 0.0 else 0.0

            p_bias = _net_horiz_bias(p_net)
            m_bias = _net_horiz_bias(m_net)
            # horizontal_l2r puts P on LEFT and M on RIGHT; horizontal_r2l puts P on RIGHT and M on LEFT
            if p_bias > m_bias + 0.05:
                sem = "horizontal_r2l"
            elif m_bias > p_bias + 0.05:
                sem = "horizontal_l2r"
        rot, mir, cad = semantic_to_transform(sem)
        transforms[inst.ref] = (rot, mir, cad, sem)
        rel_pins[inst.ref] = pdk.source_pin_offsets(inst.device_key, rot, mir)
        rel_body[inst.ref] = pdk.source_body_box(inst.device_key, rot, mir)
        lbox, env, lines = pdk.relative_label_and_envelope(
            inst, rot, mir, has_bulk_label_stub=has_bulk_stub_map[inst.ref]
        )
        ex0, ey0, ex1, ey1 = env
        # Reserve horizontal clearance when a local input/bias port flag attaches directly to Gate
        # (wide port polygon flag = 11 grids; compact internal `label_stub` = 3 grids so facing
        # co-gated mirrors stay compact and well-proportioned!)
        if "G" in inst.terminals:
            gnet_pad = inst.terminals["G"]
            gate_pad = 11 if gnet_pad in port_flag_nets else (3 if gnet_pad in label_stub_nets else 0)
            if gate_pad > 0:
                gx, gy = rel_pins[inst.ref]["G"]
                if gx == 0 and gy != 0:
                    # Horizontal MOS: gate faces UP (gy < 0) or DOWN (gy > 0) -> pad Y, not X!
                    v_pad = min(gate_pad, 7)
                    if gy < 0:
                        ey0 = min(ey0, gy - v_pad)
                    else:
                        ey1 = max(ey1, gy + v_pad)
                elif gx < 0:
                    ex0 = min(ex0, gx - gate_pad)
                else:
                    ex1 = max(ex1, gx + gate_pad)
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
    body_ext_x = {r: (-rel_body[r][0], rel_body[r][2]) for r in node_ids}
    init_x = {r: float(24 + col_of[r] * 18) for r in node_ids}
    sep_x: list[tuple[str, str, int]] = []
    springs_x: list[tuple[str, str, int, float]] = []
    eq_x: list[tuple[dict[str, float], float]] = []
    sym_x: list[tuple[str, str, str, int, int, int]] = []

    def _is_horiz_device(ref_h: str) -> bool:
        ih = inst_map[ref_h]
        p_h = rel_pins[ref_h]
        if ih.device_class == "mos" and "D" in p_h and "S" in p_h:
            return p_h["D"][1] == p_h["S"][1] and p_h["D"][0] != p_h["S"][0]
        if "P" in p_h and "M" in p_h:
            return p_h["P"][1] == p_h["M"][1] and p_h["P"][0] != p_h["M"][0]
        return False

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
                # If a horizontal bridge/device (e.g. R1 under tail M30) shares a column with an unconnected
                # vertical device, align centers (`delta=0`) instead of snapping its left P-end to the D-pin!
                u_sig_c = {n for p, n in iu.terminals.items() if p != "B"} - rails
                v_sig_c = {n for p, n in iv.terminals.items() if p != "B"} - rails
                if not (u_sig_c & v_sig_c) and (_is_horiz_device(u_ref) != _is_horiz_device(v_ref)):
                    eq_x.append(({v_ref: 1.0, u_ref: -1.0}, 0.0))
                    col_rel_offset[v_ref] = col_rel_offset[u_ref]
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
        elif mtype == "mirror_block_pair":
            l_cols = [
                [str(x) for x in col if str(x) in inst_map]
                for col in m.get("leftColumns", [])
            ]
            r_cols = [
                [str(x) for x in col if str(x) in inst_map]
                for col in m.get("rightColumns", [])
            ]
            # Enforce exact equal integer horizontal pitch between mirrored outer->inner column pairs!
            if len(l_cols) >= 2 and len(l_cols) == len(r_cols):
                for k_m in range(len(l_cols) - 1):
                    lo_col, li_col = l_cols[k_m], l_cols[k_m + 1]
                    ro_col, ri_col = r_cols[k_m], r_cols[k_m + 1]
                    if not (lo_col and li_col and ro_col and ri_col):
                        continue
                    a_lo, a_li = lo_col[0], li_col[0]
                    a_ri, a_ro = ri_col[0], ro_col[0]
                    # Compute minimum overlap-free column pitch across BOTH mirrored pairs (lo->li and ri->ro)
                    corr_lo = (extra_col_gaps or {}).get((col_of[a_lo], col_of[a_li]), 0)
                    corr_ro = (extra_col_gaps or {}).get((col_of[a_ri], col_of[a_ro]), 0)
                    corr_m = max(corr_lo, corr_ro)
                    req_side = 10
                    for c_left, c_right, anc_l, anc_r in (
                        (lo_col, li_col, a_lo, a_li),
                        (ri_col, ro_col, a_ri, a_ro),
                    ):
                        for u_m in c_left:
                            for v_m in c_right:
                                off_u = col_rel_offset[u_m] - col_rel_offset[anc_l]
                                off_v = col_rel_offset[v_m] - col_rel_offset[anc_r]
                                clr_m = (intent.spacing.intra_group_gap_x if tiers[u_m] == tiers[v_m] else intent.spacing.min_clearance_x) + corr_m
                                req_uv = extents_x[u_m][1] + extents_x[v_m][0] + clr_m + off_u - off_v
                                req_side = max(req_side, req_uv)
                    d_lo = rel_pins[a_lo][_alignment_pin_for_column(inst_map[a_lo])][0]
                    d_li = rel_pins[a_li][_alignment_pin_for_column(inst_map[a_li])][0]
                    d_ri = rel_pins[a_ri][_alignment_pin_for_column(inst_map[a_ri])][0]
                    d_ro = rel_pins[a_ro][_alignment_pin_for_column(inst_map[a_ro])][0]
                    # Rigid 2-var equalities guarantee exact integer mirror symmetry:
                    # (x[a_li]+d_li) - (x[a_lo]+d_lo) == (x[a_ro]+d_ro) - (x[a_ri]+d_ri) == req_side
                    eq_x.append(({a_li: 1.0, a_lo: -1.0}, float(req_side + d_lo - d_li)))
                    eq_x.append(({a_ro: 1.0, a_ri: -1.0}, float(req_side + d_ri - d_ro)))

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
                # Centered tail M5/M30 (and optional under-tail bridge like R1) between diff legs:
                # keep compact body-only subcolumn offset so diff pair never bloats horizontally!
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

    # Regular multi-column matrix detection (e.g. 4x4 bias mirror arrays):
    # Find maximal runs of consecutive sibling columns in the same functional group with identical stack depth >= 2.
    # Instead of stuffing redundant hard equalities into SLSQP, we keep QP fast & unconstrained here and
    # apply the 2D abstract lattice short-range tension field right after QP!
    sym_members_x = {item[0] for item in sym_x} | {item[1] for item in sym_x} | {item[2] for item in sym_x}
    for m in intent.motifs:
        if m.get("type") == "mirror_block_pair":
            for col_m in m.get("leftColumns", []) + m.get("rightColumns", []):
                sym_members_x.update(str(r) for r in col_m)
    matrix_runs: list[list[list[str]]] = []
    matrix_run_col_ranges: list[tuple[int, int]] = []
    ci_m = 0
    while ci_m < len(columns):
        depth_m = len(columns[ci_m])
        grp_m = group_of[columns[ci_m][0]]
        sig_m = tuple(inst_map[r].device_class for r in columns[ci_m])
        if depth_m >= 2 and not (set(columns[ci_m]) & sym_members_x):
            cj_m = ci_m + 1
            while (
                cj_m < len(columns)
                and len(columns[cj_m]) == depth_m
                and group_of[columns[cj_m][0]] == grp_m
                and tuple(inst_map[r].device_class for r in columns[cj_m]) == sig_m
                and not (set(columns[cj_m]) & sym_members_x)
            ):
                cj_m += 1
            if cj_m - ci_m >= 2:
                matrix_runs.append(columns[ci_m:cj_m])
                matrix_run_col_ranges.append((ci_m, cj_m))
            ci_m = cj_m
        else:
            ci_m += 1

    foam_x: list[tuple[str, str, str, float]] = []
    for ci in range(len(columns) - 2):
        r0, r1, r2 = columns[ci][0], columns[ci + 1][0], columns[ci + 2][0]
        if (
            group_of[r0] == group_of[r1] == group_of[r2]
            and not ({r0, r1, r2} & sym_members_x)
        ):
            foam_x.append((r0, r1, r2, 6.0))

    solved_x = solve_1d_compaction_qp(
        node_ids=node_ids,
        initial_pos=init_x,
        extents=extents_x,
        separations=sep_x,
        target_springs=springs_x,
        equalities=eq_x,
        symmetric_triples=sym_x,
        equidistant_triples=foam_x,
    )

    # Apply 1D horizontal short-range tension field on >= 3-column regular matrix runs
    # (operating on each column's vertical-wire axis so all 4 vertical stems become exactly equidistant!)
    sep_x_max_req: dict[tuple[str, str], int] = {}
    for u_s, v_s, mg_s in sep_x:
        req_s = extents_x[u_s][1] + extents_x[v_s][0] + mg_s
        if req_s > sep_x_max_req.get((u_s, v_s), -10**9):
            sep_x_max_req[(u_s, v_s)] = req_s

    col_wire_off: dict[str, int] = {
        r_c: rel_pins[r_c][_alignment_pin_for_column(inst_map[r_c])][0]
        for r_c in node_ids
    }
    sym_triples_by_col: dict[int, tuple[int, int, int]] = {}
    for l_id, c_id, r_id, _, _, _ in sym_x:
        if l_id in col_of and c_id in col_of and r_id in col_of:
            tr = (col_of[l_id], col_of[c_id], col_of[r_id])
            for c_idx in tr:
                sym_triples_by_col[c_idx] = tr

    did_lattice_x = False
    for run, (c_start, c_end) in zip(matrix_runs, matrix_run_col_ranges):
        if len(run) < 3:
            continue
        init_wire_xs = [float(solved_x[col[0]] + col_wire_off[col[0]]) for col in run]
        min_dx_run: list[int] = []
        for j in range(len(run) - 1):
            req_j = 8
            for u_m in run[j]:
                for v_m in run[j + 1]:
                    if (u_m, v_m) in sep_x_max_req:
                        # (wire_x_next - off_v) - (wire_x_cur - off_u) >= req  =>  wire_dx >= req + off_v - off_u
                        req_w = sep_x_max_req[(u_m, v_m)] + col_wire_off[v_m] - col_wire_off[u_m]
                        if req_w > req_j:
                            req_j = req_w
            min_dx_run.append(req_j)

        relaxed_wire_xs, _ = relax_lattice_tension_field(
            init_col_x=init_wire_xs,
            init_grid_y=[[0.0] for _ in run],
            min_dx=min_dx_run,
            min_dy=[[] for _ in run],
            relax_x=True,
            relax_y=False,
        )
        # Shift each column in `run` rigidly to its relaxed equidistant vertical-wire X
        for j, col in enumerate(run):
            old_wx = solved_x[col[0]] + col_wire_off[col[0]]
            delta_x = relaxed_wire_xs[j] - old_wx
            if delta_x != 0:
                did_lattice_x = True
                for r_c in col:
                    solved_x[r_c] += delta_x

        # Propagate any rightward push cleanly to downstream columns without breaking `sym_x` triples
        if did_lattice_x:
            for _sweep in range(len(columns) + 2):
                moved_any = False
                for u_s, v_s, mg_s in sep_x:
                    cu, cv = col_of[u_s], col_of[v_s]
                    if cv < c_end:
                        continue
                    req_s = extents_x[u_s][1] + extents_x[v_s][0] + mg_s
                    deficit = req_s - (solved_x[v_s] - solved_x[u_s])
                    if deficit > 0:
                        moved_any = True
                        if cv in sym_triples_by_col:
                            cl_s, cc_s, cr_s = sym_triples_by_col[cv]
                            if cu not in {cl_s, cc_s, cr_s}:
                                for c_sym in {cl_s, cc_s, cr_s}:
                                    for r_c in columns[c_sym]:
                                        solved_x[r_c] += deficit
                            else:
                                for r_c in columns[cv]:
                                    solved_x[r_c] += deficit
                        else:
                            for r_c in columns[cv]:
                                solved_x[r_c] += deficit
                if not moved_any:
                    break

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
    if max_col_depth <= 3:
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

        def _pin_facing_vertical_order(a: str, b: str) -> tuple[str, str]:
            """Determine vertical top/bottom order `(u_top, v_bot)` from actual shared pin geometry.

            If `a`'s downward-facing pin (`dy > 0`) connects to `b`'s upward-facing pin (`dy < 0`)
            on a non-rail net (e.g. PMOS tail `M19.D` -> PMOS diff pair `M20.S/M21.S`, or NMOS
            diff pair `M1.S` -> NMOS tail `M5.D`), return `(a, b)` so the wire flows directly
            without U-turns around device bodies.
            """
            ia, ib = inst_map[a], inst_map[b]
            pa_map, pb_map = rel_pins[a], rel_pins[b]
            vote_a_above_b = 0
            for pa, na in ia.terminals.items():
                if pa == "B" or na in rails or pa not in pa_map:
                    continue
                dya = pa_map[pa][1]
                if dya == 0:
                    continue
                for pb, nb in ib.terminals.items():
                    if pb == "B" or nb != na or pb not in pb_map:
                        continue
                    dyb = pb_map[pb][1]
                    if dya > 0 and dyb < 0:
                        vote_a_above_b += 1
                    elif dya < 0 and dyb > 0:
                        vote_a_above_b -= 1
            if vote_a_above_b > 0:
                return (a, b)
            if vote_a_above_b < 0:
                return (b, a)
            return (a, b) if effective_row_key[a] <= effective_row_key[b] else (b, a)

        def _add_ordered_diff_sep(a: str | None, b: str | None, add_spring: bool = False) -> None:
            if not a or not b or a not in inst_map or b not in inst_map:
                return
            u_top, v_bot = _pin_facing_vertical_order(a, b)
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
        mirror_aligned_refs: set[str] = set()
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
            elif mtype == "mirror_block_pair":
                for pair_item in m.get("pairs", []):
                    if len(pair_item) == 2:
                        u_p, v_p = str(pair_item[0]), str(pair_item[1])
                        _align_same_y(u_p, v_p)
                        mirror_aligned_refs.add(u_p)
                        mirror_aligned_refs.add(v_p)
            elif mtype == "parallel_bank":
                b_insts = [str(x) for x in m.get("instances", []) if str(x) in inst_map]
                for u, v in zip(b_insts, b_insts[1:]):
                    _align_same_y(u, v)
                s_drvs = [str(x) for x in m.get("stackedDrivers", []) if str(x) in inst_map]
                for u, v in zip(s_drvs, s_drvs[1:]):
                    _align_same_y(u, v)
            elif mtype == "horizontal_bridge":
                # Keep horizontal Miller / active-MOS bridge chain (e.g. M11 — M12 — M13) aligned on ONE flat row,
                # sandwiched cleanly between its upper host amplifier (e.g. PMOS M10) and lower load (e.g. NMOS M14)!
                hb_insts = [str(x) for x in m.get("instances", []) if str(x) in inst_map]
                for u, v in zip(hb_insts, hb_insts[1:]):
                    _align_same_y(u, v)
                if hb_insts:
                    hb_rep = hb_insts[0]
                    h_amp = str(m.get("hostAmp") or "")
                    h_loads = [str(x) for x in m.get("hostLoads", []) if str(x) in inst_map]
                    if h_amp in inst_map:
                        if inst_map[h_amp].polarity == "pmos":
                            sep_y_inst.append((h_amp, hb_rep, intent.spacing.min_clearance_y))
                        else:
                            sep_y_inst.append((hb_rep, h_amp, intent.spacing.min_clearance_y))
                    for hl in h_loads:
                        if inst_map[hl].polarity == "nmos":
                            sep_y_inst.append((hb_rep, hl, intent.spacing.min_clearance_y))
                        else:
                            sep_y_inst.append((hl, hb_rep, intent.spacing.min_clearance_y))
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

        # Priority 2: Row-by-row exact alignment for matched co-gated series-stacked MOS groups
        # (e.g. M10-M13 <-> M14-M17 <-> M27-M30, M1-M4 <-> M6-M9 <-> M34-M37, M21-M22 <-> M23-M24).
        # Must run BEFORE single-device cas_buckets so equal-length co-gated stacks align layer-for-layer
        # from their supply/ground rail anchor (0, 1, ..., k-1) without cross-layer tier misalignments!
        co_gated_stacks_in_cols: dict[tuple[str, str, str], list[tuple[bool, list[str]]]] = defaultdict(list)
        stack_aligned_refs: set[str] = set(mirror_aligned_refs)
        col_stack_members: list[list[str]] = []
        for stk in detect_co_gated_mos_stacks(circuit):
            stk_refs = [r for r in stk["refs"] if r in col_of]
            if len(stk_refs) < 2:
                continue
            cols_touched = {col_of[r] for r in stk_refs}
            if len(cols_touched) != 1:
                continue
            ci_s = next(iter(cols_touched))
            col_s = columns[ci_s]
            ordered_in_col = [r for r in col_s if r in set(stk_refs)]
            if len(ordered_in_col) >= 2:
                col_stack_members.append(ordered_in_col)
                key_s = (str(stk["polarity"]), str(stk["gate_net"]), str(stk["anchor_side"]))
                co_gated_stacks_in_cols[key_s].append((bool(stk["is_diode_master"]), ordered_in_col))

        for (pol_s, gn_s, anchor_side), stk_entries in co_gated_stacks_in_cols.items():
            if len(stk_entries) < 2:
                continue
            # Pairing priority: diode master & longest matched stacks align first!
            stk_entries.sort(key=lambda item: (not item[0], -len(item[1]), col_of[item[1][0]]))
            ref_stack = stk_entries[0][1]
            for _, other_stack in stk_entries[1:]:
                n_pair = min(len(ref_stack), len(other_stack))
                for layer_i in range(n_pair):
                    u_s = ref_stack[layer_i] if anchor_side == "top" else ref_stack[-1 - layer_i]
                    v_s = other_stack[layer_i] if anchor_side == "top" else other_stack[-1 - layer_i]
                    _align_same_y(u_s, v_s)
                    stack_aligned_refs.add(u_s)
                    stack_aligned_refs.add(v_s)

            # Co-gated stack zone exclusion: when an intermediate column sits strictly between two
            # flanking co-gated stacks (e.g. [M5, Q1, Q2, R1] between M1-M4 and M6-M9), push any
            # non-member devices (Q1, Q2, R1) strictly OUTSIDE the co-gated stack's vertical zone
            # so the co-gated group box stays visually pure and its bottom bridge corridor stays open!
            stks_by_col = sorted((col_of[stk[0]], stk) for _, stk in stk_entries)
            for (ci_l, stk_l), (ci_r, _stk_r) in zip(stks_by_col, stks_by_col[1:]):
                if ci_r - ci_l != 2:
                    continue
                mid_col = columns[ci_l + 1]
                has_group_member = any(
                    inst_map[r].device_class == "mos"
                    and inst_map[r].polarity == pol_s
                    and inst_map[r].terminals.get("G", "") == gn_s
                    for r in mid_col
                )
                if not has_group_member:
                    continue
                for r_mid in mid_col:
                    is_member = (
                        inst_map[r_mid].device_class == "mos"
                        and inst_map[r_mid].polarity == pol_s
                        and inst_map[r_mid].terminals.get("G", "") == gn_s
                    )
                    if is_member:
                        continue
                    if anchor_side == "top":
                        sep_y_inst.append((stk_l[-1], r_mid, intent.spacing.inter_tier_gap_y + 2))
                    else:
                        sep_y_inst.append((r_mid, stk_l[0], intent.spacing.inter_tier_gap_y + 2))

        # Priority 3: Align matched single-device current-mirror MOS & BJT sharing `(polarity, G/BASE, rail)`
        # at column extremes, plus non-stack cascode-mirror MOS & interior co-base BJT sharing `(class, polarity, ctrl, tier)`.
        mirror_buckets: dict[tuple[str, str, str], list[str]] = defaultdict(list)
        cas_buckets: dict[tuple[str, str, str, int], list[str]] = defaultdict(list)
        for r in node_ids:
            inst = inst_map[r]
            if inst.device_class == "bjt":
                b_net = inst.terminals.get("BASE", "")
                r_net = inst.terminals.get("E" if inst.polarity == "pnp" else "C", "")
                if b_net and b_net not in rails:
                    col_r = columns[col_of[r]]
                    if r_net in rails and col_r[0] == r:
                        mirror_buckets[(inst.polarity or "", b_net, r_net)].append(r)
                    elif tiers[r] in {2, 3, 4} and r not in stack_aligned_refs:
                        cas_buckets[("bjt", inst.polarity or "", b_net, tiers[r])].append(r)
                continue
            if inst.device_class != "mos":
                continue
            g_net = inst.terminals.get("G", "")
            s_net = inst.terminals.get("S", "")
            if g_net and g_net not in rails:
                col_r = columns[col_of[r]]
                if col_r[0] == r and (s_net in supplies or (inst.polarity == "pmos" and s_net)):
                    mirror_buckets[(inst.polarity or "", g_net, s_net)].append(r)
                elif col_r[-1] == r and (s_net in grounds or (inst.polarity == "nmos" and s_net)):
                    mirror_buckets[(inst.polarity or "", g_net, s_net)].append(r)
                elif tiers[r] in {2, 3, 4} and r not in stack_aligned_refs:
                    cas_buckets[("mos", inst.polarity or "", g_net, tiers[r])].append(r)
        top_mirror_refs: set[str] = set()
        cas_aligned_refs: set[str] = set()
        for m_refs in mirror_buckets.values():
            if len(m_refs) >= 2:
                for u, v in zip(m_refs, m_refs[1:]):
                    _align_same_y(u, v)
                for mr in m_refs:
                    if columns[col_of[mr]][0] == mr:
                        top_mirror_refs.add(mr)
        for c_refs in cas_buckets.values():
            if len(c_refs) >= 2:
                for u, v in zip(c_refs, c_refs[1:]):
                    _align_same_y(u, v)
                    cas_aligned_refs.add(u)
                    cas_aligned_refs.add(v)

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

        # 1D Foam / Bubble Equidistance within multi-device vertical stacks
        # Soften equidistance across boundaries between an aligned co-gated stack and lone outer
        # elements (e.g. I1 above M10-M13) so aligned stack layers stay compact and flush!
        stack_id_of: dict[str, int] = {
            r_s: idx_s for idx_s, stk_refs in enumerate(col_stack_members) for r_s in stk_refs
        }
        foam_y: list[tuple[str, str, str, float]] = []
        for col in columns:
            if len(col) >= 3:
                for k in range(len(col) - 2):
                    a_f, b_f, c_f = col[k], col[k + 1], col[k + 2]
                    sids = {stack_id_of.get(a_f, -1), stack_id_of.get(b_f, -1), stack_id_of.get(c_f, -1)}
                    crosses_stack_boundary = len(sids) > 1 and any(x >= 0 for x in sids)
                    w_foam = 1.5 if crosses_stack_boundary else 12.0
                    foam_y.append((a_f, b_f, c_f, w_foam))

        solved_inst_y = solve_1d_compaction_qp(
            node_ids=node_ids,
            initial_pos=init_y_inst,
            extents=extents_y_inst,
            separations=sep_y_inst,
            target_springs=springs_y_inst,
            equalities=eq_y_inst,
            equidistant_triples=foam_y,
            anchor_weight=0.02,
        )

        # Post-QP 2D abstract lattice short-range tension field for regular multi-column blocks
        # (aligns all rows horizontally and equalizes vertical pitch without over-constraining SLSQP!)
        sep_y_max_req: dict[tuple[str, str], int] = {}
        for u_s, v_s, mg_s in sep_y_inst:
            req_s = extents_y_inst[u_s][1] + extents_y_inst[v_s][0] + mg_s
            if req_s > sep_y_max_req.get((u_s, v_s), -10**9):
                sep_y_max_req[(u_s, v_s)] = req_s

        # Estimate shared bottom ground-rail Y across all ground-connected columns so relaxed matrices stay flush on VSS
        gnd_bot_ys: list[int] = [
            solved_inst_y[r_bot]
            for r_bot, _ in bot_vss_pins
            if r_bot in solved_inst_y
        ]
        shared_gnd_y: float | None = float(max(gnd_bot_ys)) if gnd_bot_ys else None

        # Count horizontal signal wires entering or flying straight through each adjacent row gap
        # inside regular matrix runs; when >= 2 tracks must share a gap (e.g. folded-cascode diff
        # inputs net12 + net13 crossing M27-M26), widen `min_dy_grid` so the ENTIRE matrix array
        # relaxes uniformly to a breathable equidistant pitch!
        output_port_nets_pre = {p.net for p in circuit.ports if p.direction == "output"}
        matrix_gap_tracks: dict[tuple[str, str], set[str]] = defaultdict(set)
        for net_mt in circuit.nets:
            if net_mt in rails or net_mt in local_port_nets:
                continue
            pol_mt = intent.net_policies.get(net_mt, "full_wire")
            if allow_wire_labels and pol_mt == "label_stub":
                continue
            if pol_mt == "perimeter_bus" and net_mt not in output_port_nets_pre:
                continue
            mt_terms = [
                (inst.ref, p)
                for inst in circuit.instances
                for p, n in inst.terminals.items()
                if n == net_mt and p != "B" and inst.ref in col_of
            ]
            mt_cols = {col_of[r] for r, _ in mt_terms}
            if len(mt_cols) < 2:
                continue
            mt_refs = {r for r, _ in mt_terms}
            for run in matrix_runs:
                if len(run[0]) < 3:
                    continue
                for col_r in run:
                    ci_r = col_of[col_r[0]]
                    # Case 1: Net enters an inter-row junction (u, v) on `col_r` from another column
                    for r_i in range(len(col_r) - 1):
                        u_r, v_r = col_r[r_i], col_r[r_i + 1]
                        if u_r in mt_refs and v_r in mt_refs:
                            matrix_gap_tracks[(u_r, v_r)].add(net_mt)
                    # Case 2: Net flies horizontally across `col_r` (`min_col < ci_r < max_col`, not on `col_r`)
                    if min(mt_cols) < ci_r < max(mt_cols) and ci_r not in mt_cols:
                        left_c = max(c_k for c_k in mt_cols if c_k < ci_r)
                        right_c = min(c_k for c_k in mt_cols if c_k > ci_r)
                        if right_c - left_c <= 4:
                            # Use destination column Y when it belongs to the same matrix run (e.g. M28/M29 junction!)
                            run_cols_idx = {col_of[c_k[0]] for c_k in run}
                            if right_c in run_cols_idx:
                                tgt_ys = [solved_inst_y[r] + rel_pins[r][p][1] for r, p in mt_terms if col_of[r] == right_c]
                                y_cross = sum(tgt_ys) / len(tgt_ys)
                            elif left_c in run_cols_idx:
                                tgt_ys = [solved_inst_y[r] + rel_pins[r][p][1] for r, p in mt_terms if col_of[r] == left_c]
                                y_cross = sum(tgt_ys) / len(tgt_ys)
                            else:
                                l_ys = [solved_inst_y[r] + rel_pins[r][p][1] for r, p in mt_terms if col_of[r] == left_c]
                                r_ys = [solved_inst_y[r] + rel_pins[r][p][1] for r, p in mt_terms if col_of[r] == right_c]
                                y_cross = 0.5 * ((sum(l_ys) / len(l_ys)) + (sum(r_ys) / len(r_ys)))
                            if solved_inst_y[col_r[0]] < y_cross < solved_inst_y[col_r[-1]]:
                                best_k = min(
                                    range(len(col_r) - 1),
                                    key=lambda k: abs(0.5 * (solved_inst_y[col_r[k]] + solved_inst_y[col_r[k + 1]]) - y_cross),
                                )
                                matrix_gap_tracks[(col_r[best_k], col_r[best_k + 1])].add(net_mt)

        for run in matrix_runs:
            depth_r = len(run[0])
            if depth_r < 2:
                continue
            run_refs = {r_c for col in run for r_c in col}
            # Preserve exact QP row alignment when any co-gated stack member in `run` has aligned peers outside `run`
            run_uf_roots = {_uf_find(r_c) for r_c in run_refs if r_c in stack_aligned_refs}
            if run_uf_roots and any(
                r_ext not in run_refs and _uf_find(r_ext) in run_uf_roots
                for r_ext in stack_aligned_refs
            ):
                continue
            init_y_grid = [[float(solved_inst_y[r_c]) for r_c in col] for col in run]
            min_dy_grid = []
            for col in run:
                col_dys: list[int] = []
                for r_i in range(depth_r - 1):
                    u_r, v_r = col[r_i], col[r_i + 1]
                    base_dy = sep_y_max_req.get((u_r, v_r), 10)
                    n_tr = len(matrix_gap_tracks.get((u_r, v_r), set()))
                    if n_tr >= 2:
                        # Reserve 3 grids per horizontal track + 3 grids margin between bodies
                        req_clear_tr = 3 * n_tr + 3
                        body_span = rel_body[u_r][3] - rel_body[v_r][1]
                        base_dy = max(base_dy, body_span + req_clear_tr)
                    col_dys.append(base_dy)
                min_dy_grid.append(col_dys)
            # Anchor bottom row to shared VSS rail when the bottom row connects to ground
            bot_refs = [col[-1] for col in run]
            bot_on_gnd = any(
                any(n in grounds and p != "B" for p, n in inst_map[rb].terminals.items())
                for rb in bot_refs
            )
            _, relaxed_y_grid = relax_lattice_tension_field(
                init_col_x=[float(j) for j in range(len(run))],
                init_grid_y=init_y_grid,
                min_dx=[1] * max(0, len(run) - 1),
                min_dy=min_dy_grid,
                relax_x=False,
                relax_y=True,
                anchor_bot_y=shared_gnd_y if bot_on_gnd else None,
            )
            for j, col in enumerate(run):
                for r_i, r_c in enumerate(col):
                    solved_inst_y[r_c] = relaxed_y_grid[j][r_i]

        # Keep ground-connected non-matrix columns flush with shared VSS rail WITHOUT breaking
        # top-of-column current mirrors (e.g. M30==M32 stays aligned while M31==M33 both sit on VSS!).
        if shared_gnd_y is not None:
            matrix_member_set = {r_c for run in matrix_runs for col in run for r_c in col}
            gnd_target_int = int(round(shared_gnd_y))
            for col in columns:
                r_bot = col[-1]
                if r_bot in matrix_member_set or any(r_c in stack_aligned_refs for r_c in col):
                    continue
                if any(n in grounds and p != "B" for p, n in inst_map[r_bot].terminals.items()):
                    dy_gnd = gnd_target_int - solved_inst_y[r_bot]
                    if dy_gnd == 0:
                        continue
                    if len(col) == 1:
                        solved_inst_y[r_bot] = gnd_target_int
                    elif len(col) == 2 and col[0] not in top_mirror_refs:
                        # Never drag `col[0]` downward if an explicit cross-column vertical separation
                        # constraint (e.g. horizontal_bridge hostAmp M10 above M11-M13) pins it above another device!
                        has_cross_col_below = any(
                            su == col[0] and sv != col[1] for su, sv, _ in sep_y_inst
                        )
                        if dy_gnd > 0 and has_cross_col_below:
                            solved_inst_y[col[1]] = gnd_target_int
                        else:
                            solved_inst_y[col[0]] += dy_gnd
                            solved_inst_y[col[1]] += dy_gnd
                    elif dy_gnd > 0:
                        # Keep top mirror and any interior cascode-aligned peers (e.g. M32==M33, Q15==Q8)
                        # pinned to their exact QP rows; drop bottom to VSS and only space unpinned devices below!
                        solved_inst_y[r_bot] = gnd_target_int
                        if len(col) >= 3:
                            pin_idx = 0
                            for k_p in range(1, len(col) - 1):
                                if col[k_p] in cas_aligned_refs or col[k_p] in top_mirror_refs:
                                    pin_idx = k_p
                            if pin_idx < len(col) - 2:
                                y_pin = solved_inst_y[col[pin_idx]]
                                span_c = gnd_target_int - y_pin
                                n_seg = (len(col) - 1) - pin_idx
                                step_c = span_c / float(n_seg)
                                for step_i, k_i in enumerate(range(pin_idx + 1, len(col) - 1), start=1):
                                    cand_y = int(round(y_pin + step_i * step_c))
                                    prev_req = solved_inst_y[col[k_i - 1]] + sep_y_max_req.get((col[k_i - 1], col[k_i]), 10)
                                    solved_inst_y[col[k_i]] = max(cand_y, prev_req)

        # Post-QP Slack-Yield Pin Alignment Pass:
        # When an unconstrained device has free vertical slack in its column (e.g. M18 sitting above VSS
        # below M6-M9) and yielding its Y aligns its signal pin (M18.D) horizontally with an adjacent-column
        # low-fanout gate/base pin (M20.G on net12), snap it to harvest a 0-bend horizontal wire at zero cost!
        motif_locked_y: set[str] = set()
        for m_lk in intent.motifs:
            for k_r in ("left", "right", "tail", "loadLeft", "loadRight", "hostAmp"):
                if m_lk.get(k_r):
                    motif_locked_y.add(str(m_lk[k_r]))
            for r_l in m_lk.get("instances", []) + m_lk.get("stackedDrivers", []) + m_lk.get("hostLoads", []):
                motif_locked_y.add(str(r_l))
            for r_l in m_lk.get("leftColumn", []) + m_lk.get("rightColumn", []):
                motif_locked_y.add(str(r_l))
            for col_l in m_lk.get("leftColumns", []) + m_lk.get("rightColumns", []):
                for r_l in col_l:
                    motif_locked_y.add(str(r_l))
        y_locked_all = (
            stack_aligned_refs
            | cas_aligned_refs
            | top_mirror_refs
            | motif_locked_y
            | {r_c for run in matrix_runs for col in run for r_c in col}
        )
        net_fanout_y: dict[str, int] = defaultdict(int)
        for inst_f in circuit.instances:
            for pf, nf in inst_f.terminals.items():
                if pf != "B" and nf not in rails:
                    net_fanout_y[nf] += 1

        for col in columns:
            for idx_u, u_ref in enumerate(col):
                if u_ref in y_locked_all:
                    continue
                iu = inst_map[u_ref]
                y_lo_ok = (
                    solved_inst_y[col[idx_u - 1]] + sep_y_max_req.get((col[idx_u - 1], u_ref), 10)
                    if idx_u > 0
                    else solved_inst_y[u_ref] - 18
                )
                if idx_u < len(col) - 1:
                    y_hi_ok = solved_inst_y[col[idx_u + 1]] - sep_y_max_req.get((u_ref, col[idx_u + 1]), 10)
                elif shared_gnd_y is not None and any(
                    n in grounds and p != "B" for p, n in iu.terminals.items()
                ):
                    y_hi_ok = int(round(shared_gnd_y))
                else:
                    y_hi_ok = solved_inst_y[u_ref] + 18
                if y_lo_ok > y_hi_ok:
                    continue

                best_y_snap: int | None = None
                best_delta = 10**9
                for pu, nu in iu.terminals.items():
                    if pu in {"B", "G", "BASE"} or nu in rails or net_fanout_y.get(nu, 99) > 3:
                        continue
                    for v_ref in node_ids:
                        if v_ref == u_ref or abs(col_of[v_ref] - col_of[u_ref]) != 1:
                            continue
                        iv = inst_map[v_ref]
                        for pv, nv in iv.terminals.items():
                            if nv == nu and pv in {"G", "BASE"}:
                                y_gate = solved_inst_y[v_ref] + rel_pins[v_ref][pv][1]
                                y_tgt = y_gate - rel_pins[u_ref][pu][1]
                                # If the horizontal wire exits across `u`'s own label side AND `u` has an
                                # upper/lower vertical stem on `nu` (e.g. M9.D -> M18.D tapping into M20.G),
                                # shift `u` slightly further so `u`'s label box clears `y_gate` for a 0-bend line!
                                lb_x0, lb_y0, lb_x1, lb_y1 = rel_label[u_ref]
                                go_right = col_of[v_ref] > col_of[u_ref]
                                hits_label_x = (lb_x1 > rel_pins[u_ref][pu][0]) if go_right else (lb_x0 < rel_pins[u_ref][pu][0])
                                has_upper_stem = idx_u > 0 and nu in set(inst_map[col[idx_u - 1]].terminals.values())
                                has_lower_stem = idx_u < len(col) - 1 and nu in set(inst_map[col[idx_u + 1]].terminals.values())
                                by0, by1 = rel_body[u_ref][1], rel_body[u_ref][3]
                                if hits_label_x and has_upper_stem and rel_pins[u_ref][pu][1] < 0:
                                    y_tgt = max(y_tgt, y_gate - min(lb_y0, by0) + 2)
                                elif hits_label_x and has_lower_stem and rel_pins[u_ref][pu][1] > 0:
                                    y_tgt = min(y_tgt, y_gate - max(lb_y1, by1) - 2)
                                d_mv = abs(y_tgt - solved_inst_y[u_ref])
                                if y_lo_ok <= y_tgt <= y_hi_ok and 0 < d_mv <= 18 and d_mv < best_delta:
                                    best_delta = d_mv
                                    best_y_snap = y_tgt
                if best_y_snap is not None:
                    solved_inst_y[u_ref] = best_y_snap

        # Post-QP horizontal fly-through channel opening:
        # When an active signal/output wire (e.g. VOUT between C3/C4 and M35/R2) must pass straight
        # horizontally through an intermediate >=3-device stack (e.g. M32-M36-M37-M33), gently lift
        # the upper half of that stack (and its aligned top mirror peers like M30) by a few grids so
        # the horizontal wire shoots straight through with 0 detours over the top!
        output_port_nets_set = {p.net for p in circuit.ports if p.direction == "output"}
        mirror_peers_of: dict[str, set[str]] = defaultdict(set)
        for m_refs in mirror_buckets.values():
            if len(m_refs) >= 2:
                for mr in m_refs:
                    mirror_peers_of[mr].update(m_refs)

        opened_pairs: set[tuple[str, str]] = set()
        for net_ft in circuit.nets:
            if net_ft in rails or net_ft in local_port_nets:
                continue
            pol_ft = intent.net_policies.get(net_ft, "full_wire")
            if allow_wire_labels and pol_ft == "label_stub":
                continue
            if pol_ft == "perimeter_bus" and net_ft not in output_port_nets_set:
                continue
            ft_terms = [
                (inst.ref, p)
                for inst in circuit.instances
                for p, n in inst.terminals.items()
                if n == net_ft and p != "B" and inst.ref in col_of
            ]
            ft_cols = {col_of[r] for r, _ in ft_terms}
            if len(ft_cols) < 2:
                continue
            for ci_mid in range(min(ft_cols) + 1, max(ft_cols)):
                if ci_mid in ft_cols:
                    continue
                col_mid = columns[ci_mid]
                if len(col_mid) < 3 or any(r in matrix_member_set or r in stack_aligned_refs for r in col_mid):
                    continue
                left_c = max(c_k for c_k in ft_cols if c_k < ci_mid)
                right_c = min(c_k for c_k in ft_cols if c_k > ci_mid)
                if right_c - left_c > 3:
                    continue
                left_ys = [solved_inst_y[r] + rel_pins[r][p][1] for r, p in ft_terms if col_of[r] == left_c]
                right_ys = [solved_inst_y[r] + rel_pins[r][p][1] for r, p in ft_terms if col_of[r] == right_c]
                if not left_ys or not right_ys:
                    continue
                y_pass = 0.5 * ((sum(left_ys) / len(left_ys)) + (sum(right_ys) / len(right_ys)))
                if not (solved_inst_y[col_mid[0]] < y_pass < solved_inst_y[col_mid[-1]]):
                    continue
                best_k = min(
                    range(len(col_mid) - 1),
                    key=lambda k: abs(0.5 * (solved_inst_y[col_mid[k]] + solved_inst_y[col_mid[k + 1]]) - y_pass),
                )
                u_ft, v_ft = col_mid[best_k], col_mid[best_k + 1]
                if (u_ft, v_ft) in opened_pairs:
                    continue
                cur_clear = (solved_inst_y[v_ft] + rel_env[v_ft][1]) - (solved_inst_y[u_ft] + rel_env[u_ft][3])
                need_lift = 9 - cur_clear
                if need_lift > 0:
                    opened_pairs.add((u_ft, v_ft))
                    to_lift: set[str] = set(col_mid[: best_k + 1])
                    # Keep top-of-column current mirrors (e.g. M30 == M32) strictly aligned!
                    for lr in list(to_lift):
                        if lr in top_mirror_refs:
                            to_lift.update(mirror_peers_of.get(lr, set()))
                    for lr in to_lift:
                        solved_inst_y[lr] -= need_lift

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
        for cy in range(ya, yb + 1):
            if cy != y_from and own_pl.body_box[0] <= ax <= own_pl.body_box[2] and own_pl.body_box[1] <= cy <= own_pl.body_box[3]:
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

    def _sym_box_clear(sx_c: int, sy_c: int, net_n: str, vdd_flag: bool) -> bool:
        y0_s, y1_s = (sy_c - 3, sy_c) if vdd_flag else (sy_c, sy_c + 3)
        for b_o in placements.values():
            if _box_hits_seg(b_o.body_box, sx_c - 1, y0_s, sx_c + 1, y1_s, pad=1) or _box_hits_seg(
                b_o.label_box, sx_c - 1, y0_s, sx_c + 1, y1_s, pad=1
            ):
                return False
        for cx in (sx_c - 1, sx_c, sx_c + 1):
            for cy in range(y0_s, y1_s + 1):
                if used_port_owner.get((cx, cy)) not in (None, net_n):
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
                    go_up = (ay < pl.y) if (ay != pl.y) else is_vdd
                    rail_terms.append((ax, ay, go_up, inst.ref))

            if not rail_terms:
                continue

            # 1) Merge unobstructed outward terminals onto ONE shared top VDD rail or bottom VSS rail + single symbol!
            # When unrelated non-rail bridge devices sit below the ground pins (or above supply pins),
            # prefer a tight rail directly adjacent to the rail pins whenever all outward rail pins
            # have unobstructed drops and clear symbol headroom — avoiding long stalks and router/placer mismatch.
            unobstructed_global: list[tuple[int, int, bool, str]] = []
            interior_terms: list[tuple[int, int, bool, str]] = []
            global_rail_y = next_top_rail_y if is_vdd else next_bot_rail_y
            same_dir_terms = [(ax, ay, ref_n) for ax, ay, go_up, ref_n in rail_terms if go_up == is_vdd]
            if same_dir_terms:
                tight_rail_y = (
                    (min(ay for _, ay, _ in same_dir_terms) - 3)
                    if is_vdd
                    else (max(ay for _, ay, _ in same_dir_terms) + 3)
                )
                if (is_vdd and tight_rail_y > global_rail_y) or (not is_vdd and tight_rail_y < global_rail_y):
                    xs_sd = sorted({ax for ax, _, _ in same_dir_terms})
                    sym_check_y = tight_rail_y + (-2 if is_vdd else 2)
                    if (
                        all(
                            ((is_vdd and ay >= tight_rail_y) or (not is_vdd and ay <= tight_rail_y))
                            and _vert_clear_to_rail(ax, ay, tight_rail_y, ref_n, port.net)
                            for ax, ay, ref_n in same_dir_terms
                        )
                        and _horiz_clear_on_rail(xs_sd[0], xs_sd[-1], tight_rail_y, port.net)
                        and all(_sym_box_clear(ax, sym_check_y, port.net, is_vdd) for ax in xs_sd)
                        and _sym_box_clear((xs_sd[0] + xs_sd[-1]) // 2, sym_check_y, port.net, is_vdd)
                    ):
                        global_rail_y = tight_rail_y
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
            covered_refs = {t[3] for t in unobstructed_global}
            local_taps: list[tuple[int, int, bool, int, str]] = []
            for ax, ay, go_up, ref_n in interior_terms:
                pl_i = placements[ref_n]
                # Horizontal side pin (e.g. grounded BJT BASE=VSS or tied MOS gate) whose device already has
                # a vertical rail pin on the same net: let the router connect it directly to the device's rail.
                if ay == pl_i.y and (
                    ref_n in covered_refs
                    or any(r_o == ref_n and y_o != pl_i.y for _, y_o, _, r_o in rail_terms)
                ):
                    continue
                tap_x = ax
                if ay == pl_i.y:
                    tap_x = (pl_i.body_box[0] - 2) if ax < pl_i.x else (pl_i.body_box[2] + 2)
                ty = (ay - 3) if go_up else (ay + 3)
                found_clear = False
                for _ in range(12):
                    if _vert_clear_to_rail(tap_x, ay, ty, ref_n, port.net) and (
                        used_port_owner.get((tap_x, ty)) in (None, port.net)
                    ):
                        found_clear = True
                        break
                    ty += -2 if go_up else 2
                if found_clear:
                    local_taps.append((tap_x, ty, go_up, ay, ref_n))
            if not unobstructed_global and not local_taps:
                fb_xs = sorted({t[0] for t in rail_terms})
                fb_x = fb_xs[len(fb_xs) // 2]
                fb_y = next_top_rail_y if is_vdd else next_bot_rail_y
                local_taps.append((fb_x, fb_y, is_vdd, fb_y, rail_terms[0][3]))

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
                    if len(cl_xs) == 1 and go_up == is_vdd:
                        sx, sy = cl_xs[0], ty_eff
                        while not _sym_box_clear(sx, sy, port.net, is_vdd):
                            sy += -2 if go_up else 2
                    elif len(cl_xs) == 1:
                        # Inverted-side rail pin (e.g. bottom VDD pin or top VSS pin): hook sideways into upright symbol!
                        base_x = cl_xs[0]
                        sx, sy = base_x - 5, ty_eff + (-2 if is_vdd else 2)
                        for dx_inv in (-5, 5, -6, 6, -8, 8, -10, 10):
                            cand_x = base_x + dx_inv
                            cand_y = ty_eff + (-2 if is_vdd else 2)
                            if (
                                _horiz_clear_on_rail(base_x, cand_x, ty_eff, port.net)
                                and _horiz_clear_on_rail(cand_x, cand_x, cand_y, port.net)
                                and _sym_box_clear(cand_x, cand_y, port.net, is_vdd)
                            ):
                                sx, sy = cand_x, cand_y
                                break
                    else:
                        mid_cx = (cl_xs[0] + cl_xs[-1]) // 2
                        cand_sy = ty_eff + (-2 if is_vdd else 2)
                        sx, sy = cl_xs[0], ty_eff
                        for dx_try in (0, 2, -2, 4, -4, 8, -8, 10, -10):
                            try_sx = mid_cx + dx_try
                            if cl_xs[0] <= try_sx <= cl_xs[-1] and (
                                try_sx not in cl_xs or (cl_xs[-1] - cl_xs[0]) < 4
                            ):
                                if (
                                    _horiz_clear_on_rail(try_sx, try_sx, cand_sy, port.net)
                                    and _horiz_clear_on_rail(try_sx, try_sx, ty_eff, port.net)
                                    and _sym_box_clear(try_sx, cand_sy, port.net, is_vdd)
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
            # Place a SINGLE top-level port flag adjacent to the primary local cluster;
            # nearby shared devices are wired together physically instead of duplicating ports.
            port_terms: list[tuple[str, str, Point]] = []
            for inst in circuit.instances:
                pl = placements[inst.ref]
                for pin, net in inst.terminals.items():
                    if net == port.net and pin != "B" and pin in pl.pin_points:
                        port_terms.append((inst.ref, pin, pl.pin_points[pin]))
            if not port_terms:
                continue

            clusters = cluster_local_port_terminals(port_terms, intent.spacing, placements=placements)
            primary_cl = clusters[0]
            cl_xs = [pt[0] for _, _, pt in primary_cl]
            cl_min_x, cl_max_x = min(cl_xs), max(cl_xs)

            best_cand: tuple[tuple[int, int, int, int, int], int, int, int, str] | None = None
            for ref_c, pin_c, (ax, ay) in primary_cl:
                pl = placements[ref_c]
                bx0, by0, bx1, by1 = pl.body_box
                if ax <= bx0:
                    tx, ty, step_dx, step_dy, rot_p, mir_flag = ax - 4, ay, -2, 0, 0, "none"
                    is_horiz = 0
                    faces_outer = 0 if ax == cl_min_x else 1
                elif ax >= bx1:
                    tx, ty, step_dx, step_dy, rot_p, mir_flag = ax + 4, ay, 2, 0, 0, "horizontal"
                    is_horiz = 0
                    faces_outer = 0 if ax == cl_max_x else 1
                elif ay <= by0:
                    tx, ty, step_dx, step_dy, rot_p, mir_flag = ax, ay - 4, 0, -2, 90, "none"
                    is_horiz = 1
                    faces_outer = 1
                else:
                    tx, ty, step_dx, step_dy, rot_p, mir_flag = ax, ay + 4, 0, 2, 270, "none"
                    is_horiz = 1
                    faces_outer = 1

                ux = 0 if step_dx == 0 else (1 if step_dx > 0 else -1)
                uy = 0 if step_dy == 0 else (1 if step_dy > 0 else -1)

                def _port_footprint_hits(px_t: int, py_t: int) -> bool:
                    # Port flag + label spans ~5 grids outward from (px_t, py_t) in direction (ux, uy)
                    fx0 = min(px_t, px_t + ux * 5) - (1 if ux == 0 else 0)
                    fx1 = max(px_t, px_t + ux * 5) + (1 if ux == 0 else 0)
                    fy0 = min(py_t, py_t + uy * 5) - (1 if uy == 0 else 0)
                    fy1 = max(py_t, py_t + uy * 5) + (1 if uy == 0 else 0)
                    if used_port_owner.get((px_t, py_t)) not in (None, port.net):
                        return True
                    for r_other, b in placements.items():
                        if r_other == ref_c:
                            continue
                        if not (
                            fx1 < b.full_envelope[0] - 1
                            or fx0 > b.full_envelope[2] + 1
                            or fy1 < b.full_envelope[1] - 1
                            or fy0 > b.full_envelope[3] + 1
                        ):
                            return True
                    return False

                steps_moved = 0
                while steps_moved < 24 and _port_footprint_hits(tx, ty):
                    tx += step_dx
                    ty += step_dy
                    steps_moved += 1

                if used_port_owner.get((tx, ty)) == port.net:
                    continue
                # Penalize flags sandwiched directly between two stacked/adjacent pins of the same cluster
                between_stacked_pins = 0
                if uy != 0:
                    for r_o, _, (ox, oy) in primary_cl:
                        if r_o != ref_c and ox == ax and ((uy > 0 and oy > ay) or (uy < 0 and oy < ay)):
                            between_stacked_pins = 1
                            break
                elif ux != 0 and pin_c != "G" and _is_horiz_device(ref_c):
                    # Horizontally sandwiched channel node between two horizontal series devices (e.g. M12.D <-> M13.S)
                    for r_o, p_o, (ox, oy) in primary_cl:
                        if r_o != ref_c and p_o != "G" and oy == ay and ((ux > 0 and ox > ax) or (ux < 0 and ox < ax)):
                            between_stacked_pins = 2
                            break
                score = (between_stacked_pins, steps_moved, faces_outer, is_horiz, tx, ty)
                if best_cand is None or score < best_cand[0]:
                    best_cand = (score, tx, ty, rot_p, mir_flag)

            if best_cand is not None:
                best_score, tx, ty, rot_p, mir_flag = best_cand
                # Case 1: Vertically sandwiched divider node (e.g. R2.M <-> R3.P) -> tap horizontally off midpoint
                if best_score[0] == 1 and len(primary_cl) >= 2:
                    p_ys = sorted(pt[1] for _, _, pt in primary_cl)
                    mid_x = primary_cl[0][2][0]
                    mid_y = (p_ys[0] + p_ys[-1]) // 2
                    for side_dir, r_try, m_try in ((-1, 0, "none"), (1, 0, "horizontal")):
                        cand_x = mid_x + side_dir * 5
                        fx0 = min(cand_x, cand_x + side_dir * 5)
                        fx1 = max(cand_x, cand_x + side_dir * 5)
                        if used_port_owner.get((cand_x, mid_y)) in (None, port.net) and not any(
                            not (
                                fx1 < b.full_envelope[0] - 1
                                or fx0 > b.full_envelope[2] + 1
                                or mid_y + 1 < b.full_envelope[1] - 1
                                or mid_y - 1 > b.full_envelope[3] + 1
                            )
                            for b in placements.values()
                        ):
                            tx, ty, rot_p, mir_flag = cand_x, mid_y, r_try, m_try
                            break
                # Case 2: Horizontally sandwiched series node (e.g. M12.D <-> M13.S `Vb6`) -> tap vertically off midpoint!
                elif best_score[0] == 2 and len(primary_cl) >= 2:
                    p_xs = sorted(pt[0] for _, _, pt in primary_cl)
                    mid_x = (p_xs[0] + p_xs[-1]) // 2
                    mid_y = primary_cl[0][2][1]
                    placed_mid_v = False
                    for v_dir, r_try in ((-1, 90), (1, 270)):
                        for dist_v in (4, 6, 8, 10):
                            cand_y = mid_y + v_dir * dist_v
                            fy0 = min(cand_y, cand_y + v_dir * 5)
                            fy1 = max(cand_y, cand_y + v_dir * 5)
                            if used_port_owner.get((mid_x, cand_y)) in (None, port.net) and not any(
                                not (
                                    mid_x + 1 < b.full_envelope[0]
                                    or mid_x - 1 > b.full_envelope[2]
                                    or fy1 < b.full_envelope[1]
                                    or fy0 > b.full_envelope[3]
                                )
                                for b in placements.values()
                            ):
                                tx, ty, rot_p, mir_flag = mid_x, cand_y, r_try, "none"
                                placed_mid_v = True
                                break
                        if placed_mid_v:
                            break
                # Case 3: Paired horizontal MOS gates sharing a bottom/top bias port (e.g. M12.G & M13.G on `Vb7`):
                # center the downward/upward bias flag cleanly at the midpoint between the two gates!
                elif rot_p in {90, 270} and len(primary_cl) >= 2 and len({pt[1] for _, _, pt in primary_cl}) == 1:
                    p_xs = sorted(pt[0] for _, _, pt in primary_cl)
                    mid_x = (p_xs[0] + p_xs[-1]) // 2
                    v_dir = -1 if rot_p == 90 else 1
                    cand_y = primary_cl[0][2][1] + v_dir * 5
                    fy0 = min(cand_y, cand_y + v_dir * 5)
                    fy1 = max(cand_y, cand_y + v_dir * 5)
                    if used_port_owner.get((mid_x, cand_y)) in (None, port.net) and not any(
                        not (
                            mid_x + 2 < b.full_envelope[0] - 1
                            or mid_x - 2 > b.full_envelope[2] + 1
                            or fy1 < b.full_envelope[1] - 1
                            or fy0 > b.full_envelope[3] + 1
                        )
                        for b in placements.values()
                    ):
                        tx, ty = mid_x, cand_y
                used_port_owner[(tx, ty)] = port.net
                placed_ports.append(
                    PortOccurrencePlaced(
                        occurrence_id=f"P_{port.name}",
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
