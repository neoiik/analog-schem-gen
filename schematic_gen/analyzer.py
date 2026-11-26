"""Deterministic analog motif analyzer + draft IntentSpec builder for LLM co-execution."""

from __future__ import annotations

from collections import Counter, defaultdict
import re
from typing import Any

from schematic_gen.models import (
    AmbiguityItem,
    CandidateMotif,
    FunctionalGroupSpec,
    InstanceIR,
    NetlistCircuit,
    SchematicIntent,
    SpacingConfig,
    TopologyAnalysis,
)
from schematic_gen.branch_abstraction import build_branch_abstraction
from schematic_gen.semantic_digest import render_semantic_digest_md
from schematic_gen.topology_semantics import (
    build_reduced_mos_units,
    build_signal_flow_backbone,
    classify_device_roles,
    detect_complementary_sf,
    detect_composite_diff_pairs,
    detect_miller_bridges,
    detect_quasi_mirror_and_sandwich_cores,
)


def _compute_net_fanout(circuit: NetlistCircuit) -> dict[str, int]:
    counts: Counter[str] = Counter()
    for inst in circuit.instances:
        for pin, net in inst.terminals.items():
            if pin == "B" and net in (set(circuit.supply_nets) | set(circuit.ground_nets)):
                continue
            counts[net] += 1
    for port in circuit.ports:
        if port.net not in (set(circuit.supply_nets) | set(circuit.ground_nets)):
            counts[port.net] += 1
    for n in circuit.nets:
        counts.setdefault(n, 0)
    return dict(counts)


# Shared primitives now live in `topology_core` (layer-neutral); re-exported here for compatibility.
from schematic_gen.topology_core import (  # noqa: E402
    THRESHOLDS,
    assign_default_tiers as _assign_default_tiers,
    detect_co_gated_mos_stacks,
    is_nonpolar_2pin_class,
)


def detect_mirror_subcircuit_pairs(
    circuit: NetlistCircuit,
    *,
    excluded_refs: set[str] | None = None,
    min_half_instances: int = 4,
    min_private_nets: int = 2,
) -> list[dict[str, Any]]:
    """Detect multi-device mirror-isomorphic half-circuit pairs via WL coloring + dual-BFS expansion.

    Discovers symmetric differential / folded subcircuits (such as `{Q3,R3,M21,M22,M25,Q4,R4}`
    <-> `{Q5,R5,M23,M24,M26,Q6,R6}`) purely from netlist graph topology without relying on
    device names.
    """
    excluded = set(excluded_refs or ())
    rails = set(circuit.supply_nets) | set(circuit.ground_nets)
    supplies = set(circuit.supply_nets)
    grounds = set(circuit.ground_nets)
    inst_map = {i.ref: i for i in circuit.instances if i.ref not in excluded}
    if len(inst_map) < 2 * min_half_instances:
        return []

    fanout = _compute_net_fanout(circuit)

    # Bipartite incidence (ignoring bulk pin "B")
    net_inc: dict[str, list[tuple[str, str]]] = defaultdict(list)
    for r, inst in inst_map.items():
        for pin, net in inst.terminals.items():
            if pin != "B":
                net_inc[net].append((r, pin))

    def _norm_pin_role(inst: InstanceIR, pin: str) -> str:
        if is_nonpolar_2pin_class(inst.device_class):
            return "PM" if pin in {"P", "M"} else pin
        return pin

    # 1. Deterministic Weisfeiler-Lehman (WL) 3-round structural coloring
    def _relabel(sig_map: dict[str, Any]) -> dict[str, int]:
        palette: dict[Any, int] = {}
        out: dict[str, int] = {}
        for key in sorted(sig_map.keys()):
            sig = sig_map[key]
            if sig not in palette:
                palette[sig] = len(palette)
            out[key] = palette[sig]
        return out

    inst_color: dict[str, int] = _relabel(
        {
            r: (
                inst.device_class,
                inst.polarity,
                inst.kind,
                inst.params.get("r") or inst.params.get("c") or "",
            )
            for r, inst in inst_map.items()
        }
    )

    port_nets = {p.net for p in circuit.ports}

    def _is_private_net(net: str) -> bool:
        return net not in rails and net not in port_nets and 2 <= fanout.get(net, 99) <= 3

    init_net_sig: dict[str, Any] = {}
    for net in circuit.nets:
        if net in supplies:
            init_net_sig[net] = ("RAIL_VDD", net)
        elif net in grounds:
            init_net_sig[net] = ("RAIL_VSS", net)
        elif _is_private_net(net):
            init_net_sig[net] = ("PRIV", fanout.get(net, 0))
        else:
            # Shared bias nets or asymmetric external interface nets (e.g. C/B, net3/A)
            # are abstracted identically during local WL coloring so outer fanout differences
            # do not obscure internal subcircuit mirror isomorphism.
            init_net_sig[net] = ("BUS_OR_EXT",)
    net_color: dict[str, int] = _relabel(init_net_sig)

    for _ in range(3):
        raw_inst: dict[str, Any] = {}
        for r, inst in inst_map.items():
            nbrs = tuple(
                sorted(
                    (_norm_pin_role(inst, pin), net_color.get(net, -1))
                    for pin, net in inst.terminals.items()
                    if pin != "B"
                )
            )
            raw_inst[r] = (inst_color[r], nbrs)
        inst_color = _relabel(raw_inst)

        raw_net: dict[str, Any] = {net: (0, net_color[net]) for net in net_color}
        for net, inc in net_inc.items():
            if not _is_private_net(net):
                continue
            nbrs = tuple(
                sorted((_norm_pin_role(inst_map[r], pin), inst_color[r]) for r, pin in inc)
            )
            raw_net[net] = (1, net_color[net], nbrs)
        net_color = _relabel(raw_net)

    def _expand_from_seed(seed_a: str, seed_b: str) -> tuple[dict[str, str], dict[str, str]] | None:
        inst_bij: dict[str, str] = {seed_a: seed_b}
        rev_inst: dict[str, str] = {seed_b: seed_a}
        net_bij: dict[str, str] = {}
        rev_net: dict[str, str] = {}
        boundary_net_pairs: list[tuple[str, str]] = []
        queue: list[tuple[str, str]] = [(seed_a, seed_b)]

        while queue:
            ua, ub = queue.pop(0)
            ia, ib = inst_map[ua], inst_map[ub]

            # Build pin-pair candidates between ua and ub
            pin_pairs: list[tuple[str, str]] = []
            if is_nonpolar_2pin_class(ia.device_class):
                # Match P/M pins of non-polarized 2-pin passives by net compatibility
                pa_m = ia.terminals.get("P", ""), ia.terminals.get("M", "")
                pb_m = ib.terminals.get("P", ""), ib.terminals.get("M", "")
                direct_ok = (
                    (pa_m[0] == pb_m[0] or net_bij.get(pa_m[0]) == pb_m[0] or (pa_m[0] not in net_bij and pb_m[0] not in rev_net and net_color.get(pa_m[0]) == net_color.get(pb_m[0])))
                    and (pa_m[1] == pb_m[1] or net_bij.get(pa_m[1]) == pb_m[1] or (pa_m[1] not in net_bij and pb_m[1] not in rev_net and net_color.get(pa_m[1]) == net_color.get(pb_m[1])))
                )
                swap_ok = (
                    (pa_m[0] == pb_m[1] or net_bij.get(pa_m[0]) == pb_m[1] or (pa_m[0] not in net_bij and pb_m[1] not in rev_net and net_color.get(pa_m[0]) == net_color.get(pb_m[1])))
                    and (pa_m[1] == pb_m[0] or net_bij.get(pa_m[1]) == pb_m[0] or (pa_m[1] not in net_bij and pb_m[0] not in rev_net and net_color.get(pa_m[1]) == net_color.get(pb_m[0])))
                )
                if swap_ok and not direct_ok:
                    pin_pairs = [("P", "M"), ("M", "P")]
                elif direct_ok:
                    pin_pairs = [("P", "P"), ("M", "M")]
                else:
                    return None
            else:
                for pin in ia.terminals:
                    if pin != "B" and pin in ib.terminals:
                        pin_pairs.append((pin, pin))

            for pin_a, pin_b in pin_pairs:
                na, nb = ia.terminals[pin_a], ib.terminals[pin_b]
                if na == nb:
                    continue
                if na in rails or nb in rails:
                    return None
                if not (_is_private_net(na) and _is_private_net(nb)):
                    # Record non-control boundary interface net pairs (e.g. Q4.E=net3 <-> Q6.E=A)
                    if pin_a not in {"G", "BASE"} and pin_b not in {"G", "BASE"}:
                        boundary_net_pairs.append((na, nb))
                    continue
                if na in net_bij:
                    if net_bij[na] != nb:
                        return None
                elif nb in rev_net:
                    return None
                else:
                    net_bij[na] = nb
                    rev_net[nb] = na

                # Expand across private net pair `(na, nb)` to unmatched isomorphic neighbors
                nbrs_a = [
                    (r, _norm_pin_role(inst_map[r], p))
                    for r, p in net_inc.get(na, [])
                    if r != ua and r not in inst_bij and r not in rev_inst
                ]
                nbrs_b = [
                    (r, _norm_pin_role(inst_map[r], p))
                    for r, p in net_inc.get(nb, [])
                    if r != ub and r not in rev_inst and r not in inst_bij
                ]
                if len(nbrs_a) != len(nbrs_b):
                    continue
                used_b_local: set[str] = set()
                for ra, role_a in sorted(nbrs_a):
                    cands_b = [
                        rb
                        for rb, role_b in sorted(nbrs_b)
                        if rb not in used_b_local
                        and rb != ra
                        and role_b == role_a
                        and inst_color[ra] == inst_color[rb]
                    ]
                    if len(cands_b) == 1:
                        rb = cands_b[0]
                        used_b_local.add(rb)
                        inst_bij[ra] = rb
                        rev_inst[rb] = ra
                        queue.append((ra, rb))

        # Attach unique shared-bias active loads on paired current-carrying boundary nets
        # (e.g. M25 on net3 <-> M26 on A where both share G=net2 and S=VDD).
        paired_boundary_nets = (
            set(net_bij.keys())
            | set(rev_net.keys())
            | {na for na, _ in boundary_net_pairs}
            | {nb for _, nb in boundary_net_pairs}
        )
        for na, nb in boundary_net_pairs:
            if na in net_bij or nb in rev_net or na == nb:
                continue
            cands_a = [
                (r, p)
                for r, p in net_inc.get(na, [])
                if r not in inst_bij
                and r not in rev_inst
                and p not in {"G", "BASE", "B"}
                and inst_map[r].device_class in {"mos", "bjt"}
            ]
            cands_b = [
                (r, p)
                for r, p in net_inc.get(nb, [])
                if r not in inst_bij
                and r not in rev_inst
                and p not in {"G", "BASE", "B"}
                and inst_map[r].device_class in {"mos", "bjt"}
            ]
            matched_loads: list[tuple[str, str]] = []
            for ra, pa in sorted(cands_a):
                ia2 = inst_map[ra]
                other_a = {
                    p: n for p, n in ia2.terminals.items() if p not in {"B", pa}
                }
                if not other_a or any(n in paired_boundary_nets for n in other_a.values()):
                    continue
                hits = [
                    rb
                    for rb, pb in sorted(cands_b)
                    if rb != ra
                    and pb == pa
                    and inst_color[ra] == inst_color[rb]
                    and {
                        p: n for p, n in inst_map[rb].terminals.items() if p not in {"B", pb}
                    }
                    == other_a
                ]
                if len(hits) == 1:
                    matched_loads.append((ra, hits[0]))
            if len(matched_loads) == 1:
                ra, rb = matched_loads[0]
                if ra not in inst_bij and rb not in rev_inst and ra not in rev_inst and rb not in inst_bij:
                    inst_bij[ra] = rb
                    rev_inst[rb] = ra
                    net_bij[na] = nb
                    rev_net[nb] = na

        # Verify strict disjointness of the two halves
        if set(inst_bij.keys()) & set(inst_bij.values()):
            return None
        return inst_bij, net_bij

    # 2. Gather candidate seed pairs sharing a non-rail bias/control net and identical WL color
    by_color: dict[Any, list[str]] = defaultdict(list)
    for r, col_h in inst_color.items():
        if inst_map[r].device_class in {"mos", "bjt"}:
            by_color[col_h].append(r)

    raw_candidates: list[tuple[int, int, dict[str, str], dict[str, str]]] = []
    for _col_h, refs in by_color.items():
        if len(refs) < 2:
            continue
        refs_sorted = sorted(refs)
        for i_a, ra in enumerate(refs_sorted):
            for rb in refs_sorted[i_a + 1 :]:
                ia, ib = inst_map[ra], inst_map[rb]
                ctrl_p = "G" if ia.device_class == "mos" else "BASE"
                shared_ctrl = (
                    ia.terminals.get(ctrl_p) == ib.terminals.get(ctrl_p)
                    and ia.terminals.get(ctrl_p, "") not in rails
                )
                if not shared_ctrl:
                    continue
                res_exp = _expand_from_seed(ra, rb)
                if res_exp is None:
                    continue
                inst_bij, net_bij = res_exp
                # Count truly internal private net pairs (both ends inside the matched halves)
                half_a = set(inst_bij.keys())
                half_b = set(inst_bij.values())
                internal_priv = sum(
                    1
                    for na, nb in net_bij.items()
                    if sum(1 for r, _ in net_inc.get(na, []) if r in half_a) >= 2
                    and sum(1 for r, _ in net_inc.get(nb, []) if r in half_b) >= 2
                )
                if len(inst_bij) >= min_half_instances and internal_priv >= min_private_nets:
                    raw_candidates.append((len(inst_bij), internal_priv, inst_bij, net_bij))

    # Deduplicate and keep maximal disjoint mirror subcircuit pairs
    raw_candidates.sort(key=lambda item: (-item[0], -item[1], sorted(item[2].keys())))
    claimed: set[str] = set()
    results: list[dict[str, Any]] = []
    for _sz, internal_priv, inst_bij, net_bij in raw_candidates:
        all_members = set(inst_bij.keys()) | set(inst_bij.values())
        if all_members & claimed:
            continue
        claimed.update(all_members)
        # Canonicalize left vs right by lowest reference number in each half
        def _min_num(s: set[str]) -> tuple[int, str]:
            nums = [int(m.group(1)) for r in s if (m := re.search(r"(\d+)$", r))]
            return (min(nums) if nums else 10**9, min(s))

        if _min_num(set(inst_bij.values())) < _min_num(set(inst_bij.keys())):
            inst_bij = {v: k for k, v in inst_bij.items()}
            net_bij = {v: k for k, v in net_bij.items()}
        pairs_sorted = sorted(inst_bij.items(), key=lambda kv: kv[0])
        results.append(
            {
                "leftInstances": [k for k, _ in pairs_sorted],
                "rightInstances": [v for _, v in pairs_sorted],
                "pairs": [[k, v] for k, v in pairs_sorted],
                "privateNetPairs": [[k, v] for k, v in sorted(net_bij.items())],
                "internalPrivateCount": internal_priv,
            }
        )
    return results


def _bundle_vertical_branches(
    pool_refs: list[str],
    inst_by_ref: dict[str, InstanceIR],
    default_tiers: dict[str, int],
    fanout: dict[str, int],
    rails: set[str],
    atomic_stacks: list[list[str]] | None = None,
) -> list[list[str]]:
    """Bundle vertically connected series/degenerated/divider/mirror devices into aligned columns.

    Priority 1: Preserve atomic co-gated series MOS stacks (`M10-M13`, `M14-M17`, `M1-M4`, etc.)
    as unbroken vertical sub-columns even when consecutive members share the same default tier.
    Priority 2: Pair remaining vertical branches along shared non-rail `D/S`, `C/E`, `P/M` nets.
    """
    if not pool_refs:
        return []

    pool_set = set(pool_refs)
    pool_pos = {r: idx for idx, r in enumerate(pool_refs)}

    def _vert_nets(ref: str) -> set[str]:
        inst = inst_by_ref[ref]
        if inst.device_class == "mos":
            return {inst.terminals[p] for p in ("D", "S") if p in inst.terminals} - rails
        if inst.device_class == "bjt":
            return {inst.terminals[p] for p in ("C", "E") if p in inst.terminals} - rails
        if inst.device_class != "subckt":
            return {inst.terminals[p] for p in ("P", "M") if p in inst.terminals} - rails
        return set()

    def _top_bot_pins(ref: str) -> tuple[str, str]:
        inst = inst_by_ref[ref]
        if inst.device_class == "mos":
            return ("S", "D") if inst.polarity == "pmos" else ("D", "S")
        if inst.device_class == "bjt":
            return ("E", "C") if inst.polarity == "pnp" else ("C", "E")
        return ("P", "M")

    # Seed columns with atomic co-gated series stacks (never split!)
    col_list: list[list[str]] = []
    col_idx_of: dict[str, int] = {}
    locked_order_cols: set[int] = set()
    for stk in atomic_stacks or []:
        clean = [r for r in stk if r in pool_set and r not in col_idx_of]
        if len(clean) >= 2:
            cid = len(col_list)
            col_list.append(clean)
            locked_order_cols.add(cid)
            for r in clean:
                col_idx_of[r] = cid
    for r in pool_refs:
        if r not in col_idx_of:
            cid = len(col_list)
            col_list.append([r])
            col_idx_of[r] = cid

    vnets = {r: _vert_nets(r) for r in pool_refs}
    # Count non-gate vertical drivers per net so a diode-master (D==G) can still stack directly
    # with its sole opposite-polarity drain driver even when the bias net has high gate fanout (>8)!
    vert_drivers_by_net: dict[str, set[str]] = defaultdict(set)
    for r in pool_refs:
        for vn in vnets[r]:
            vert_drivers_by_net[vn].add(r)

    edges: list[tuple[int, str, str]] = []
    for i_u, u in enumerate(pool_refs):
        for v in pool_refs[i_u + 1 :]:
            if col_idx_of[u] == col_idx_of[v]:
                continue
            tu, tv = default_tiers[u], default_tiers[v]
            shared = vnets[u] & vnets[v]
            if not shared:
                continue
            best_n = min(shared, key=lambda n: (len(vert_drivers_by_net[n]), fanout.get(n, 99)))
            fn = fanout.get(best_n, 99)
            vdeg = len(vert_drivers_by_net[best_n])
            if fn > 8 and vdeg > 2:
                continue
            eff_fn = vdeg if (vdeg <= 2 and fn > 8) else fn

            # Determine vertical top->bottom direction; allow same-tier strict pin-facing series links!
            u_top_p, u_bot_p = _top_bot_pins(u)
            v_top_p, v_bot_p = _top_bot_pins(v)
            u_bot_n = inst_by_ref[u].terminals.get(u_bot_p, "")
            v_top_n = inst_by_ref[v].terminals.get(v_top_p, "")
            v_bot_n = inst_by_ref[v].terminals.get(v_bot_p, "")
            u_top_n = inst_by_ref[u].terminals.get(u_top_p, "")

            strict_u_above_v = bool(u_bot_n and u_bot_n == v_top_n and u_bot_n not in rails)
            strict_v_above_u = bool(v_bot_n and v_bot_n == u_top_n and v_bot_n not in rails)

            if tu < tv:
                hi, lo = u, v
            elif tv < tu:
                hi, lo = v, u
            elif strict_u_above_v and not strict_v_above_u and fn <= 2:
                hi, lo = u, v
            elif strict_v_above_u and not strict_u_above_v and fn <= 2:
                hi, lo = v, u
            else:
                continue

            base = 120 if eff_fn <= 2 else (85 if eff_fn == 3 else (60 - 3 * eff_fn))
            iu, iv = inst_by_ref[hi], inst_by_ref[lo]
            mos_bonus = 0
            if iu.polarity == "pmos" and iv.polarity == "nmos":
                if iu.terminals.get("D") == iv.terminals.get("D"):
                    mos_bonus += 18
                if iu.terminals.get("G") == iv.terminals.get("G") and iu.terminals.get("G") not in rails:
                    mos_bonus += 35
            # Bonus when merging an atomic stack with its direct top/bottom driver or load
            if (col_idx_of[hi] in locked_order_cols or col_idx_of[lo] in locked_order_cols) and eff_fn <= 3:
                mos_bonus += 24
            tier_pen = 12 * max(0, abs(default_tiers[lo] - default_tiers[hi] - 1))
            edges.append((base + mos_bonus - tier_pen, hi, lo))

    edges.sort(key=lambda e: (-e[0], pool_pos[e[1]], pool_pos[e[2]]))

    for _, hi, lo in edges:
        c_hi = col_idx_of[hi]
        c_lo = col_idx_of[lo]
        if c_hi == c_lo:
            continue
        list_hi = col_list[c_hi]
        list_lo = col_list[c_lo]
        # Require clean boundary connection (bottom of upper col -> top of lower col) or strict tier separation
        strict_boundary = (
            hi == list_hi[-1]
            and lo == list_lo[0]
            and bool(vnets[hi] & vnets[lo])
            and default_tiers[hi] <= default_tiers[lo]
        )
        tier_disjoint = max(default_tiers[x] for x in list_hi) < min(default_tiers[y] for y in list_lo)
        if strict_boundary or tier_disjoint:
            if c_hi in locked_order_cols or c_lo in locked_order_cols:
                merged = list_hi + list_lo
                locked_order_cols.add(c_hi)
            else:
                merged = sorted(list_hi + list_lo, key=lambda x: default_tiers[x])
            col_list[c_hi] = merged
            col_list[c_lo] = []
            for x in merged:
                col_idx_of[x] = c_hi

    return [c for c in col_list if c]


def _order_columns_and_orient_gates(
    cols: list[list[str]],
    prior_cols: list[list[str]],
    inst_by_ref: dict[str, InstanceIR],
    fanout: dict[str, int],
    rails: set[str],
    diode_refs: set[str],
    circuit: NetlistCircuit,
    orientations: dict[str, str],
) -> list[list[str]]:
    """Order vertical branch columns for shortest inter-column wiring and orient MOS gates toward connected neighbors."""
    if not cols:
        return []

    in_port_nets = {p.net for p in circuit.ports if p.direction == "input"} - rails
    out_port_nets = {p.net for p in circuit.ports if p.direction == "output"} - rails

    def _col_nets(c: list[str]) -> set[str]:
        return {
            n
            for r in c
            for p, n in inst_by_ref[r].terminals.items()
            if p != "B" and n not in rails
        }

    cnets = [_col_nets(c) for c in cols]
    prior_union = {n for c in prior_cols for n in _col_nets(c)}

    # Device-group pairing priority metadata:
    # Priority 1: Matched co-gated series MOS stacks (same polarity & gate_net, ranked by stack length & master-slave)
    # Priority 2: Matched single-device MOS / BJT mirrors sharing (polarity, gate/base, rail)
    # Priority 3: General signal-net connectivity (16 / fanout)
    all_stacks = detect_co_gated_mos_stacks(circuit)

    def _ref_num(ref: str) -> int | None:
        m_num = re.search(r"(\d+)$", ref)
        return int(m_num.group(1)) if m_num else None

    def _stack_num_span(stk: dict[str, Any]) -> tuple[int, int] | None:
        nums = [n for r in stk["refs"] if (n := _ref_num(r)) is not None]
        return (min(nums), max(nums)) if nums else None

    def _col_stacks(col: list[str]) -> list[dict[str, Any]]:
        cset = set(col)
        return [s for s in all_stacks if all(r in cset for r in s["refs"])]

    def _group_pair_priority(col_a: list[str], col_b: list[str]) -> float:
        st_a = _col_stacks(col_a)
        st_b = _col_stacks(col_b)
        score = 0.0
        for sa in st_a:
            for sb in st_b:
                if sa["polarity"] == sb["polarity"] and sa["gate_net"] == sb["gate_net"]:
                    la, lb = int(sa["length"]), int(sb["length"])
                    has_master = bool(sa["is_diode_master"] or sb["is_diode_master"])
                    span_a = _stack_num_span(sa)
                    span_b = _stack_num_span(sb)
                    name_bonus = 0.0
                    if span_a and span_b:
                        gap = max(0, max(span_a[0], span_b[0]) - min(span_a[1], span_b[1]))
                        # Naming proximity is a pure tie-break (weight below every topology increment)
                        name_bonus = THRESHOLDS.naming_tiebreak_weight / (1.0 + gap)
                    if la == lb:
                        # Priority Tier 1a: Exact k-to-k co-gated stack pair, tie-broken by naming-order proximity
                        # (e.g. M1-4 pairs with M6-9 ahead of M34-37; M10-13 pairs with M14-17 ahead of M27-30)
                        score += 28.0 * la + (45.0 if has_master else 0.0) + name_bonus
                    else:
                        # Priority Tier 1b: Partial-length co-gated stack pair (lower priority than exact k-to-k)
                        score += 10.0 * min(la, lb) + (12.0 if has_master else 0.0) + 0.4 * name_bonus

        # Priority Tier 2: Multi-layer stacked cascode / bandgap mirror columns sharing >= 2 matched
        # (device_class, polarity, ctrl_net) single-device pairs (e.g. Q9/Q10 + M32/M33 + Q15/Q8)
        ctrl_a: dict[tuple[str, str, str], list[str]] = defaultdict(list)
        ctrl_b: dict[tuple[str, str, str], list[str]] = defaultdict(list)
        for r_a in col_a:
            ia = inst_by_ref[r_a]
            if ia.device_class in {"mos", "bjt"}:
                cp = "G" if ia.device_class == "mos" else "BASE"
                gn = ia.terminals.get(cp, "")
                if gn and gn not in rails:
                    ctrl_a[(ia.device_class, ia.polarity or "", gn)].append(r_a)
        for r_b in col_b:
            ib = inst_by_ref[r_b]
            if ib.device_class in {"mos", "bjt"}:
                cp = "G" if ib.device_class == "mos" else "BASE"
                gn = ib.terminals.get(cp, "")
                if gn and gn not in rails:
                    ctrl_b[(ib.device_class, ib.polarity or "", gn)].append(r_b)
        shared_ctrl_keys = set(ctrl_a.keys()) & set(ctrl_b.keys())
        if len(shared_ctrl_keys) >= 2:
            matched_layers = 0
            prox_bonus = 0.0
            for k_c in shared_ctrl_keys:
                matched_layers += min(len(ctrl_a[k_c]), len(ctrl_b[k_c]))
                nums_a = [n for r in ctrl_a[k_c] if (n := _ref_num(r)) is not None]
                nums_b = [n for r in ctrl_b[k_c] if (n := _ref_num(r)) is not None]
                if nums_a and nums_b:
                    d_num = min(abs(na - nb) for na in nums_a for nb in nums_b)
                    prox_bonus += THRESHOLDS.naming_tiebreak_weight / (1.0 + d_num)
            score += 24.0 * matched_layers + prox_bonus
        return score

    if len(cols) >= 2:
        n_c = len(cols)
        adj_w: list[list[float]] = [[0.0] * n_c for _ in range(n_c)]
        for i in range(n_c):
            for j in range(i + 1, n_c):
                sh = cnets[i] & cnets[j]
                w = sum(16.0 / max(2, fanout.get(n, 2)) for n in sh)
                w += _group_pair_priority(cols[i], cols[j])
                adj_w[i][j] = adj_w[j][i] = w

        # Left-anchor score: prioritize columns whose co-gated stacks pair with prior_cols (e.g. bias_gen)!
        left_bias: list[float] = []
        for i in range(n_c):
            sc = sum(10.0 / max(2, fanout.get(n, 2)) for n in (cnets[i] & prior_union))
            sc += sum(_group_pair_priority(pc, cols[i]) for pc in prior_cols)
            sc += 4.0 * len(cnets[i] & in_port_nets)
            sc -= 4.0 * len(cnets[i] & out_port_nets)
            left_bias.append(sc)

        start_idx = max(range(n_c), key=lambda i: (left_bias[i], -len(cols[i]), -i))
        order = [start_idx]
        rem = set(range(n_c)) - {start_idx}
        while rem:
            last = order[-1]
            nxt = max(rem, key=lambda j: (adj_w[last][j], left_bias[j], -j))
            order.append(nxt)
            rem.remove(nxt)

        # Quick 2-opt refinement to eliminate unnecessary horizontal skips while preserving
        # strong prior-stage boundary pairing (e.g. bias_gen M10-13 <-> col[0] M14-17)!
        prior_boundary_w = [
            (_group_pair_priority(prior_cols[-1], cols[i]) if prior_cols else 0.0)
            for i in range(n_c)
        ]

        def _chain_score(seq: list[int]) -> float:
            s = prior_boundary_w[seq[0]] if seq else 0.0
            s += sum(adj_w[seq[k]][seq[k + 1]] for k in range(len(seq) - 1))
            s += 0.35 * sum(left_bias[seq[k]] * (len(seq) - 1 - k) for k in range(len(seq)))
            return s

        improved = True
        while improved:
            improved = False
            best_sc = _chain_score(order)
            for a in range(n_c - 1):
                for b in range(a + 1, n_c):
                    cand = order[:a] + list(reversed(order[a : b + 1])) + order[b + 1 :]
                    cand_sc = _chain_score(cand)
                    if cand_sc > best_sc + 1e-6:
                        order = cand
                        best_sc = cand_sc
                        improved = True
        cols = [cols[i] for i in order]

        # Post-ordering between-stack singleton insertion by naming order:
        # When two adjacent paired co-gated stacks (e.g. M1-4 and M6-9 on gate net2) have a naming gap
        # containing a same-polarity, same-gate singleton MOS (e.g. M5 with 4 < 5 < 6), place that
        # singleton column directly between them so naming flow and top gate bus stay natural!
        inserted_singletons: set[int] = set()
        k_scan = 0
        # Legacy naming-order move (disabled by default; layout follows topology, not reference numbers)
        while THRESHOLDS.allow_naming_layout_rules and k_scan < len(cols) - 1:
            st_l = _col_stacks(cols[k_scan])
            st_r = _col_stacks(cols[k_scan + 1])
            moved = False
            for sl in st_l:
                for sr in st_r:
                    if sl["polarity"] != sr["polarity"] or sl["gate_net"] != sr["gate_net"]:
                        continue
                    sp_l = _stack_num_span(sl)
                    sp_r = _stack_num_span(sr)
                    if not sp_l or not sp_r:
                        continue
                    lo_gap = min(sp_l[1], sp_r[1])
                    hi_gap = max(sp_l[0], sp_r[0])
                    if hi_gap - lo_gap < 2:
                        continue
                    pol_g = sl["polarity"]
                    gn_g = sl["gate_net"]
                    # Find a column holding a co-gated singleton MOS whose ref number lies strictly in (lo_gap, hi_gap)
                    cand_idxs: list[tuple[int, int]] = []
                    for m_idx, m_col in enumerate(cols):
                        if m_idx in (k_scan, k_scan + 1) or id(m_col) in inserted_singletons:
                            continue
                        if any(s["polarity"] == pol_g and s["gate_net"] == gn_g for s in _col_stacks(m_col)):
                            continue
                        for r_m in m_col:
                            im = inst_by_ref[r_m]
                            if (
                                im.device_class == "mos"
                                and im.polarity == pol_g
                                and im.terminals.get("G") == gn_g
                            ):
                                rn = _ref_num(r_m)
                                if rn is not None and lo_gap < rn < hi_gap:
                                    cand_idxs.append((rn, m_idx))
                                    break
                    if cand_idxs:
                        cand_idxs.sort(key=lambda item: item[0])
                        _, src_idx = cand_idxs[0]
                        moved_col = cols.pop(src_idx)
                        inserted_singletons.add(id(moved_col))
                        ins_at = k_scan + 1 if src_idx > k_scan else k_scan
                        cols.insert(ins_at, moved_col)
                        k_scan = ins_at + 1
                        moved = True
                        break
                if moved:
                    break
            if not moved:
                k_scan += 1

    # Orient co-gated series stacks uniformly toward their paired stack neighbor, then individual MOS/BJT gates
    full_seq = list(prior_cols) + list(cols)
    offset = len(prior_cols)
    for local_i, col in enumerate(cols):
        abs_i = offset + local_i
        for stk in _col_stacks(col):
            g_net = stk["gate_net"]
            if all(r in orientations for r in stk["refs"]):
                continue
            left_p = 0.0
            right_p = 0.0
            for j, other_col in enumerate(full_seq):
                if j == abs_i:
                    continue
                decay = 1.0 / (1.0 + 0.65 * (abs(j - abs_i) - 1))
                w_stk = 0.0
                for os_item in _col_stacks(other_col):
                    if os_item["polarity"] == stk["polarity"] and os_item["gate_net"] == g_net:
                        w_stk += (6.0 if os_item["is_diode_master"] else 3.0) * min(stk["length"], os_item["length"]) * decay
                if w_stk == 0.0:
                    for ro in other_col:
                        if inst_by_ref[ro].terminals.get("G") == g_net:
                            w_stk += (2.0 if ro in diode_refs else 1.0) * decay
                if j < abs_i:
                    left_p += w_stk
                else:
                    right_p += w_stk
            chosen_ori = "gate_right" if (right_p > left_p + 0.1) else "gate_left"
            for r in stk["refs"]:
                orientations.setdefault(r, chosen_ori)

        for r in col:
            inst = inst_by_ref[r]
            if inst.device_class not in {"mos", "bjt"} or r in orientations:
                continue
            ctrl_pin = "G" if inst.device_class == "mos" else "BASE"
            g_net = inst.terminals.get(ctrl_pin, "")
            if not g_net or g_net in rails:
                orientations[r] = "gate_left"
                continue
            left_pull = 1.8 if g_net in in_port_nets else 0.0
            right_pull = 1.8 if g_net in out_port_nets else 0.0
            for j, other_col in enumerate(full_seq):
                if j == abs_i:
                    continue
                dist_decay = 1.0 / (1.0 + 0.55 * (abs(j - abs_i) - 1))
                w_col = 0.0
                for ro in other_col:
                    io = inst_by_ref[ro]
                    for po, no in io.terminals.items():
                        if po == "B" or no != g_net:
                            continue
                        w_col += (2.2 if (po not in {"G", "BASE"} or ro in diode_refs) else 1.2) * dist_decay
                if j < abs_i:
                    left_pull += w_col
                else:
                    right_pull += w_col
            orientations[r] = "gate_right" if (right_pull > left_pull + 0.1) else "gate_left"

    # Face-to-face orientation for adjacent 2-column local co-gated / co-base mirror pairs
    # (e.g. Q9<->Q10 on A, M32<->M33 on net4, Q15<->Q8 on VREF, Q7<->Q13 on net1):
    # orient left member `gate_right` and right member `gate_left` for 0-bend straight horizontal bars!
    stack_ref_set = {r for s in all_stacks for r in s["refs"]}
    for local_i in range(len(cols) - 1):
        col_l, col_r = cols[local_i], cols[local_i + 1]
        abs_l, abs_r = offset + local_i, offset + local_i + 1
        for rl in col_l:
            if rl in stack_ref_set:
                continue
            il = inst_by_ref[rl]
            if il.device_class not in {"mos", "bjt"}:
                continue
            cp = "G" if il.device_class == "mos" else "BASE"
            gn = il.terminals.get(cp, "")
            if not gn or gn in rails:
                continue
            matches_r = [
                rr
                for rr in col_r
                if rr not in stack_ref_set
                and inst_by_ref[rr].device_class == il.device_class
                and inst_by_ref[rr].polarity == il.polarity
                and inst_by_ref[rr].terminals.get(cp, "") == gn
            ]
            if len(matches_r) != 1:
                continue
            rr = matches_r[0]
            # Confirm neither immediately outer neighbor shares the exact same (class, polarity, ctrl_net)
            has_outer_peer = False
            for nb_idx in (abs_l - 1, abs_r + 1):
                if 0 <= nb_idx < len(full_seq):
                    if any(
                        inst_by_ref[ro].device_class == il.device_class
                        and inst_by_ref[ro].polarity == il.polarity
                        and inst_by_ref[ro].terminals.get(cp, "") == gn
                        for ro in full_seq[nb_idx]
                    ):
                        has_outer_peer = True
                        break
            if not has_outer_peer:
                orientations[rl] = "gate_right"
                orientations[rr] = "gate_left"

    return cols


# Public name for other semantic-layer modules (semantic_validator) — avoids importing a private helper.
order_columns_and_orient_gates = _order_columns_and_orient_gates


def _apply_coarse_macro_first_ordering(
    column_hints: list[list[str]],
    groups: list[FunctionalGroupSpec],
    intent_motifs: list[dict[str, Any]],
    co_gated_stacks: list[dict[str, Any]],
    inst_by_ref: dict[str, InstanceIR],
    fanout: dict[str, int],
    rails: set[str],
    diode_refs: set[str],
    circuit: NetlistCircuit,
    orientations: dict[str, str],
) -> tuple[list[list[str]], list[FunctionalGroupSpec], list[dict[str, Any]]]:
    """Two-phase Coarse-to-Fine Macro-Anchor-First planner for multi-macro analog sheets.

    Phase 1 (Coarse): Extract atomic macro clusters in structural priority order:
      P1: Multi-column mirror subcircuit pairs (`mirror_block_pair`)
      P2: Multi-layer 2-column stacked local mirrors (e.g. Q9/M32/Q15 <-> Q10/M33/Q8)
      P3: Co-gated series MOS stack macros (including vertical bonding across a single
          diode/cascode bridge, e.g. M34-37 + M31 + M27-30, and paired bias-mirror blocks)
      Then find the globally optimal left-to-right macro sequence using directed driver->slave
      signal flow, local inter-macro net affinity, and device naming-order progression.

    Phase 2 (Fine): Incrementally insert residual (non-macro) branches into inter-macro slots
      minimizing horizontal wire span without ever splitting an atomic macro block.
    """
    from itertools import permutations

    cols = [list(c) for c in column_hints if c]
    if len(cols) < 8:
        return cols, groups, []

    def _ref_num(ref: str) -> int | None:
        m = re.search(r"(\d+)$", ref)
        return int(m.group(1)) if m else None

    def _col_sig_nets(col: list[str]) -> set[str]:
        return {
            n
            for r in col
            for p, n in inst_by_ref[r].terminals.items()
            if p != "B" and n not in rails
        }

    def _col_stacks(col: list[str]) -> list[dict[str, Any]]:
        cset = set(col)
        return [s for s in co_gated_stacks if all(r in cset for r in s["refs"])]

    # --- Phase 1a: Vertical stack bonding across a single diode/cascode bridge ---
    # e.g. PMOS stack [M34..M37] -> diode/cascode [M31] -> NMOS stack [M27..M30]
    merged_idx: set[int] = set()
    bonded_cols: list[list[str]] = []
    # Map each column index to its pure-stack type
    pmos_stack_ci = [
        i
        for i, c in enumerate(cols)
        if any(s["polarity"] == "pmos" for s in _col_stacks(c))
        and not any(s["polarity"] == "nmos" for s in _col_stacks(c))
    ]
    nmos_stack_ci = [
        i
        for i, c in enumerate(cols)
        if any(s["polarity"] == "nmos" for s in _col_stacks(c))
        and not any(s["polarity"] == "pmos" for s in _col_stacks(c))
    ]
    bridge_merges: dict[int, list[int]] = {}
    peeled_residuals: list[list[str]] = []
    for pi in pmos_stack_ci:
        if pi in merged_idx:
            continue
        p_col = cols[pi]
        p_nets = _col_sig_nets(p_col)
        # Case A: Bridge diode/cascode (e.g. M31) is ALREADY bundled inside `p_col` (M34..M37 + M31)
        p_diodes = [r for r in p_col if r in diode_refs and not any(r in s["refs"] for s in _col_stacks(p_col))]
        if p_diodes:
            bridge_nets = {
                n
                for dr in p_diodes
                for p, n in inst_by_ref[dr].terminals.items()
                if p != "B" and n not in rails
            }
            for ni in nmos_stack_ci:
                if ni in merged_idx or ni == pi:
                    continue
                n_col = cols[ni]
                if any(inst_by_ref[r].device_class in {"isource", "vsource"} for r in n_col):
                    continue
                n_top_nets = {
                    inst_by_ref[r].terminals.get("D", "")
                    for r in n_col
                    if inst_by_ref[r].device_class == "mos"
                } - rails
                if n_top_nets & bridge_nets:
                    # Peel any trailing shunt passive (e.g. C2) out of `p_col` into residuals before bonding!
                    keep_p = [r for r in p_col if not inst_by_ref[r].device_class.startswith("cap")]
                    peel_p = [r for r in p_col if inst_by_ref[r].device_class.startswith("cap")]
                    cols[pi] = keep_p
                    for pr in peel_p:
                        peeled_residuals.append([pr])
                    bridge_merges[pi] = [pi, ni]
                    merged_idx.update({pi, ni})
                    break
            if pi in bridge_merges:
                continue
        # Case B: Separate diode/cascode bridge column (len <= 2, no stack) between `pi` and `ni`
        for mi, mcol in enumerate(cols):
            if mi in merged_idx or mi == pi or len(mcol) > 2 or _col_stacks(mcol):
                continue
            if not any(r in diode_refs for r in mcol):
                continue
            m_nets = _col_sig_nets(mcol)
            sh_pm = {n for n in (p_nets & m_nets) if fanout.get(n, 99) <= 4}
            if not sh_pm:
                continue
            other_m_nets = m_nets - sh_pm
            for ni in nmos_stack_ci:
                if ni in merged_idx or ni == pi or ni == mi:
                    continue
                n_col = cols[ni]
                if any(inst_by_ref[r].device_class in {"isource", "vsource"} for r in n_col):
                    continue
                n_top_nets = {
                    inst_by_ref[r].terminals.get("D", "")
                    for r in n_col
                    if inst_by_ref[r].device_class == "mos"
                } - rails
                if n_top_nets & other_m_nets:
                    bridge_merges[pi] = [pi, mi, ni]
                    merged_idx.update({pi, mi, ni})
                    break
            if pi in bridge_merges:
                break

    for i, c in enumerate(cols):
        if i in bridge_merges:
            bonded_cols.append([r for ci_b in bridge_merges[i] for r in cols[ci_b]])
        elif i not in merged_idx:
            bonded_cols.append(c)
    bonded_cols.extend(peeled_residuals)
    cols = bonded_cols

    # --- Phase 1b: Extract atomic coarse macro blocks in priority order (P1 -> P2 -> P3) ---
    claimed_ci: set[int] = set()
    macros: list[dict[str, Any]] = []

    # Priority 1: `mirror_block_pair` motifs
    for mp in intent_motifs:
        if mp.get("type") != "mirror_block_pair":
            continue
        mp_set = {str(r) for col_m in mp.get("leftColumns", []) + mp.get("rightColumns", []) for r in col_m}
        mp_cis = [i for i, c in enumerate(cols) if i not in claimed_ci and set(c) & mp_set]
        if len(mp_cis) >= 2:
            claimed_ci.update(mp_cis)
            macros.append(
                {
                    "id": str(mp.get("id", f"macro_mirror_{len(macros)+1}")),
                    "label": "镜像对称主体",
                    "role": "gain_stage",
                    "priority": 1,
                    "kind": "mirror_block_pair",
                    "cols": [cols[i] for i in mp_cis],
                }
            )

    # Priority 2: Multi-layer stacked local mirror pairs (>= 2 matched `(class, polarity, ctrl_net)` layers)
    def _col_ctrl_sig(col: list[str]) -> dict[tuple[str, str, str], list[str]]:
        out: dict[tuple[str, str, str], list[str]] = defaultdict(list)
        for r in col:
            inst = inst_by_ref[r]
            if inst.device_class in {"mos", "bjt"}:
                cp = "G" if inst.device_class == "mos" else "BASE"
                gn = inst.terminals.get(cp, "")
                if gn and gn not in rails:
                    out[(inst.device_class, inst.polarity or "", gn)].append(r)
        return out

    unclaimed_nonstack = [i for i in range(len(cols)) if i not in claimed_ci and not _col_stacks(cols[i])]
    local_pairs: list[tuple[int, int, int]] = []
    for idx_a, ia in enumerate(unclaimed_nonstack):
        ca = _col_ctrl_sig(cols[ia])
        for ib in unclaimed_nonstack[idx_a + 1 :]:
            cb = _col_ctrl_sig(cols[ib])
            sh_k = set(ca.keys()) & set(cb.keys())
            if len(sh_k) >= 2:
                local_pairs.append((len(sh_k), ia, ib))
    local_pairs.sort(key=lambda t: (-t[0], t[1], t[2]))
    for _cnt, ia, ib in local_pairs:
        if ia in claimed_ci or ib in claimed_ci:
            continue
        # Order left vs right by lowest ref number in the top matched layer
        na_min = min((_ref_num(r) or 10**9) for r in cols[ia])
        nb_min = min((_ref_num(r) or 10**9) for r in cols[ib])
        pair_cols = [cols[ia], cols[ib]] if na_min <= nb_min else [cols[ib], cols[ia]]
        claimed_ci.update({ia, ib})
        macros.append(
            {
                "id": f"macro_local_mirror_{len(macros)+1}",
                "label": "多层局部镜像主体",
                "role": "input_diff",
                "priority": 2,
                "kind": "local_mirror_stack",
                "cols": pair_cols,
            }
        )

    # Priority 3: Co-gated series MOS stack macros (grouping paired bias stacks + between-stack singleton)
    stack_cis = [i for i in range(len(cols)) if i not in claimed_ci and _col_stacks(cols[i])]
    # Partition `stack_cis` into connected components by exact equal-length co-gated stack pairing
    # (keeping tall bridged columns like M34-37+M31+M27-30 as their own distinct macro if unpaired or bridge-merged!)
    bridged_refs = {r for ci_list in bridge_merges.values() for ci_b in ci_list for r in column_hints[ci_b]}
    bias_stack_cis: list[int] = []
    other_stack_cis: list[int] = []
    for si in stack_cis:
        if set(cols[si]) & bridged_refs:
            other_stack_cis.append(si)
        else:
            bias_stack_cis.append(si)

    if bias_stack_cis:
        # Also pull in any between-stack naming singleton column (e.g. [M5, Q1] sitting between M1-4 and M6-9)
        lo_b, hi_b = min(bias_stack_cis), max(bias_stack_cis)
        bias_group_cis = list(bias_stack_cis)
        pmos_stack_nums = [
            n
            for bi in bias_stack_cis
            for s in _col_stacks(cols[bi])
            if s["polarity"] == "pmos"
            for r in s["refs"]
            if (n := _ref_num(r)) is not None
        ]
        pmos_lo = min(pmos_stack_nums) if pmos_stack_nums else 0
        pmos_hi = max(pmos_stack_nums) if pmos_stack_nums else 0
        for mid_i in range(lo_b + 1, hi_b):
            if mid_i not in claimed_ci and mid_i not in bias_group_cis and mid_i not in other_stack_cis:
                # Only absorb if it holds a co-gated singleton MOS whose ref number lies inside the stack span
                stk_g = {s["gate_net"] for bi in bias_stack_cis for s in _col_stacks(cols[bi])}
                if any(
                    inst_by_ref[r].device_class == "mos"
                    and inst_by_ref[r].terminals.get("G", "") in stk_g
                    and (not THRESHOLDS.allow_naming_layout_rules or pmos_lo < (_ref_num(r) or 0) < pmos_hi)
                    for r in cols[mid_i]
                ):
                    bias_group_cis.append(mid_i)
        bias_group_cis.sort()
        claimed_ci.update(bias_group_cis)
        macros.append(
            {
                "id": f"macro_bias_stacks_{len(macros)+1}",
                "label": "共栅叠管电流镜主体",
                "role": "bias",
                "priority": 3,
                "kind": "co_gated_stack_group",
                "cols": [cols[i] for i in bias_group_cis],
            }
        )

    for oi in other_stack_cis:
        if oi in claimed_ci:
            continue
        claimed_ci.add(oi)
        macros.append(
            {
                "id": f"macro_stack_col_{len(macros)+1}",
                "label": "共栅叠管偏置柱",
                "role": "bias",
                "priority": 4,
                "kind": "co_gated_stack_column",
                "cols": [cols[oi]],
            }
        )

    if len(macros) < 2:
        return cols, groups, []

    # --- Phase 1c: Globally order macro blocks Left -> Right ---
    # Compute per-macro net sets, driver/slave roles, bias-source flags, and naming order
    n_mac = len(macros)
    macro_refs = [{r for col in m["cols"] for r in col} for m in macros]
    macro_nets = [
        {n for r in refs for p, n in inst_by_ref[r].terminals.items() if p != "B" and n not in rails}
        for refs in macro_refs
    ]
    # Local inter-macro nets: touched by 2 macros (excludes global bias buses touching 3+ macros)
    net_macro_cnt: dict[str, int] = defaultdict(int)
    for mns in macro_nets:
        for n in mns:
            net_macro_cnt[n] += 1
    local_inter_nets = {n for n, c in net_macro_cnt.items() if c == 2}

    # Directed driver -> slave edges between macros:
    # Macro A drives net `n` via a non-control (or diode-connected) pin, Macro B receives `n` on G/BASE!
    dir_edges: dict[tuple[int, int], float] = defaultdict(float)
    for n in local_inter_nets:
        owners = [mi for mi in range(n_mac) if n in macro_nets[mi]]
        if len(owners) != 2:
            continue
        ma, mb = owners[0], owners[1]
        a_drives = any(
            (r in diode_refs and inst_by_ref[r].terminals.get("G", inst_by_ref[r].terminals.get("BASE")) == n)
            or any(p not in {"B", "G", "BASE"} and tn == n for p, tn in inst_by_ref[r].terminals.items())
            for r in macro_refs[ma]
        )
        b_ctrl = any(
            inst_by_ref[r].terminals.get("G" if inst_by_ref[r].device_class == "mos" else "BASE") == n
            and r not in diode_refs
            for r in macro_refs[mb]
            if inst_by_ref[r].device_class in {"mos", "bjt"}
        )
        b_drives = any(
            (r in diode_refs and inst_by_ref[r].terminals.get("G", inst_by_ref[r].terminals.get("BASE")) == n)
            or any(p not in {"B", "G", "BASE"} and tn == n for p, tn in inst_by_ref[r].terminals.items())
            for r in macro_refs[mb]
        )
        a_ctrl = any(
            inst_by_ref[r].terminals.get("G" if inst_by_ref[r].device_class == "mos" else "BASE") == n
            and r not in diode_refs
            for r in macro_refs[ma]
            if inst_by_ref[r].device_class in {"mos", "bjt"}
        )
        w_n = 18.0 / max(2, fanout.get(n, 2))
        if a_drives and b_ctrl and not (b_drives and a_ctrl):
            dir_edges[(ma, mb)] += w_n + (12.0 if fanout.get(n, 99) <= 4 else 0.0)
        elif b_drives and a_ctrl and not (a_drives and b_ctrl):
            dir_edges[(mb, ma)] += w_n + (12.0 if fanout.get(n, 99) <= 4 else 0.0)

    def _macro_is_bias_root(mi: int) -> bool:
        refs = macro_refs[mi]
        return any(inst_by_ref[r].device_class in {"isource", "vsource"} for r in refs) or any(
            s["is_diode_master"] and all(r in refs for r in s["refs"]) for s in co_gated_stacks
        )

    def _macro_mos_median_num(mi: int) -> float:
        mnums = sorted(n for r in macro_refs[mi] if inst_by_ref[r].device_class == "mos" and (n := _ref_num(r)) is not None)
        if not mnums:
            mnums = sorted(n for r in macro_refs[mi] if (n := _ref_num(r)) is not None)
        return float(mnums[len(mnums) // 2]) if mnums else 999.0

    # Pin primary bias root leftmost when unique, and exhaustively optimize ordering of remaining macros (n_mac <= 7)
    root_macros = [mi for mi in range(n_mac) if _macro_is_bias_root(mi)]
    fixed_left = sorted(root_macros, key=_macro_mos_median_num)
    free_macros = [mi for mi in range(n_mac) if mi not in fixed_left]
    mos_med = [_macro_mos_median_num(mi) for mi in range(n_mac)]

    def _eval_macro_seq(seq: list[int]) -> float:
        pos = {mi: idx for idx, mi in enumerate(seq)}
        sc = 0.0
        # 1. Minimize local inter-macro distance (strong adjacency bonus for dist == 1)
        for mi in range(n_mac):
            for mj in range(mi + 1, n_mac):
                sh = macro_nets[mi] & macro_nets[mj] & local_inter_nets
                if not sh:
                    continue
                d = abs(pos[mi] - pos[mj])
                w_pair = sum(14.0 / max(2, fanout.get(n, 2)) for n in sh)
                sc -= w_pair * (d - 1)
        # 2. Respect directed driver -> slave control flow (e.g. M31(net4) -> M32/M33, M26(A) -> Q9/Q10)
        for (src, dst), w_e in dir_edges.items():
            if pos[src] < pos[dst]:
                sc += w_e + (0.6 * w_e if pos[dst] == pos[src] + 1 else 0.0)
            else:
                sc -= 1.2 * w_e
        # 3. Naming-order progression: pure tie-break (summed weight stays below one topology edge)
        w_name = 0.1 * THRESHOLDS.naming_tiebreak_weight
        for a_i in range(len(seq)):
            for b_i in range(a_i + 1, len(seq)):
                if mos_med[seq[a_i]] < mos_med[seq[b_i]]:
                    sc += w_name
                elif mos_med[seq[a_i]] > mos_med[seq[b_i]]:
                    sc -= w_name
        return sc

    if len(free_macros) <= 6:
        best_seq = max(
            (fixed_left + list(perm) for perm in permutations(free_macros)),
            key=_eval_macro_seq,
        )
    else:
        # Topological order of the directed driver->slave macro graph; naming only breaks ties
        free_set = set(free_macros)
        indeg_m = {mi: 0 for mi in free_macros}
        for (src_m, dst_m), _w in dir_edges.items():
            if src_m in free_set and dst_m in free_set and src_m != dst_m:
                indeg_m[dst_m] += 1
        topo_seq: list[int] = []
        ready = sorted([mi for mi, d in indeg_m.items() if d == 0], key=_macro_mos_median_num)
        while ready:
            cur_m = ready.pop(0)
            topo_seq.append(cur_m)
            for (src_m, dst_m), _w in dir_edges.items():
                if src_m == cur_m and dst_m in indeg_m and dst_m not in topo_seq:
                    indeg_m[dst_m] -= 1
                    if indeg_m[dst_m] == 0:
                        ready.append(dst_m)
            ready.sort(key=_macro_mos_median_num)
        topo_seq += sorted((mi for mi in free_macros if mi not in topo_seq), key=_macro_mos_median_num)
        best_seq = fixed_left + topo_seq

    ordered_macros = [macros[mi] for mi in best_seq]

    # --- Phase 2: Incrementally insert residual columns into inter-macro slots ---
    res_cols_raw = [cols[i] for i in range(len(cols)) if i not in claimed_ci]

    # Bundle vertically series-connected residual passive singletons (e.g. C1(C-E) + C2(E-VSS) -> [C1, C2])
    res_used: set[int] = set()
    res_cols: list[list[str]] = []
    for i, rc in enumerate(res_cols_raw):
        if i in res_used:
            continue
        if len(rc) == 1 and inst_by_ref[rc[0]].device_class.startswith(("cap", "res")):
            r0 = rc[0]
            nets0 = _col_sig_nets(rc)
            on_gnd0 = bool(set(inst_by_ref[r0].terminals.values()) & rails)
            partner_j = -1
            for j in range(i + 1, len(res_cols_raw)):
                if j in res_used or len(res_cols_raw[j]) != 1:
                    continue
                rj = res_cols_raw[j][0]
                if inst_by_ref[rj].device_class != inst_by_ref[r0].device_class:
                    continue
                netsj = _col_sig_nets(res_cols_raw[j])
                on_gndj = bool(set(inst_by_ref[rj].terminals.values()) & rails)
                if (nets0 & netsj) and (on_gnd0 != on_gndj):
                    partner_j = j
                    break
            if partner_j >= 0:
                res_used.update({i, partner_j})
                rj = res_cols_raw[partner_j][0]
                res_cols.append([r0, rj] if not on_gnd0 else [rj, r0])
                continue
        res_used.add(i)
        res_cols.append(rc)

    # Represent placed sequence as list of `(is_macro_id, cols_list)` units so residuals ONLY insert between units!
    units: list[dict[str, Any]] = [
        {
            "id": m["id"],
            "label": m["label"],
            "role": m["role"],
            "is_macro": True,
            "cols": [list(c) for c in m["cols"]],
        }
        for m in ordered_macros
    ]

    output_port_nets = {p.net for p in circuit.ports if p.direction == "output"}
    port_nets_all = {p.net for p in circuit.ports} - rails
    # Order residual columns: internal high-connectivity branches first, output-stage drivers last
    def _res_priority(rc: list[str]) -> tuple[int, int, int]:
        rnets = _col_sig_nets(rc)
        is_out_drv = any(
            inst_by_ref[r].device_class in {"bjt", "mos"}
            and any(p not in {"B", "G", "BASE"} and n in output_port_nets for p, n in inst_by_ref[r].terminals.items())
            for r in rc
        )
        is_out_res = all(inst_by_ref[r].device_class.startswith("res") for r in rc) and bool(rnets & output_port_nets)
        placed_nets_init = {n for u in units for c in u["cols"] for n in _col_sig_nets(c)}
        sh_cnt = len({n for n in (rnets & placed_nets_init) if fanout.get(n, 99) <= 6})
        min_n = min((_ref_num(r) or 999) for r in rc)
        return (1 if (is_out_drv or is_out_res) else 0, -sh_cnt, min_n)

    res_cols.sort(key=_res_priority)

    for rc in res_cols:
        rnets = _col_sig_nets(rc)
        local_rnets = {n for n in rnets if fanout.get(n, 99) <= 6} or rnets
        # Score every inter-unit slot `0 .. len(units)`
        best_slot = len(units)
        best_score = -1e18
        for slot in range(len(units) + 1):
            sc = 0.0
            for ui, u in enumerate(units):
                unets = {n for c in u["cols"] for n in _col_sig_nets(c)}
                sh = local_rnets & unets
                if not sh:
                    continue
                # Distance from `slot` to unit `ui` (0 if directly adjacent on left or right)
                dist = (ui - slot) if ui >= slot else (slot - 1 - ui)
                w_u = sum(16.0 / max(2, fanout.get(n, 2)) for n in sh)
                sc -= w_u * dist
                # Slight edge-side preference when `u` is a multi-column macro (e.g. C1/C2 near Q3 on left of Macro1, C3 near Q5 on right of Macro1!)
                if len(u["cols"]) >= 2:
                    left_edge_nets = _col_sig_nets(u["cols"][0])
                    right_edge_nets = _col_sig_nets(u["cols"][-1])
                    if (local_rnets & left_edge_nets) and not (local_rnets & right_edge_nets):
                        sc += 8.0 if slot <= ui else -6.0
                    elif (local_rnets & right_edge_nets) and not (local_rnets & left_edge_nets):
                        sc += 8.0 if slot > ui else -6.0
            # Output-port / VREF-connected residuals bias toward the right edge
            if rnets & output_port_nets:
                sc += 2.5 * slot
            elif rnets & port_nets_all:
                sc += 0.3 * slot
            if sc > best_score:
                best_score = sc
                best_slot = slot
        units.insert(
            best_slot,
            {
                "id": f"residual_{rc[0]}",
                "label": "衔接与输出支路",
                "role": "output_stage" if (rnets & output_port_nets) else "bias",
                "is_macro": False,
                "cols": [rc],
            },
        )

    # Flatten final column order and rebuild clean contiguous FunctionalGroupSpec list
    new_cols: list[list[str]] = []
    new_groups: list[FunctionalGroupSpec] = []
    coarse_meta: list[dict[str, Any]] = []
    # Merge adjacent non-macro residual units into one clean group so stage boxes stay tidy
    stage_idx = 0
    ui = 0
    while ui < len(units):
        u = units[ui]
        if u["is_macro"]:
            grp_insts = [r for c in u["cols"] for r in c]
            new_cols.extend(u["cols"])
            new_groups.append(
                FunctionalGroupSpec(
                    id=u["id"],
                    label=u["label"],
                    role=u["role"],
                    stage_order=stage_idx * 10,
                    instances=grp_insts,
                )
            )
            coarse_meta.append(
                {
                    "id": u["id"],
                    "label": u["label"],
                    "stageOrder": stage_idx * 10,
                    "columns": u["cols"],
                }
            )
            stage_idx += 1
            ui += 1
        else:
            merged_res_cols: list[list[str]] = []
            while ui < len(units) and not units[ui]["is_macro"]:
                merged_res_cols.extend(units[ui]["cols"])
                ui += 1
            grp_insts = [r for c in merged_res_cols for r in c]
            new_cols.extend(merged_res_cols)
            new_groups.append(
                FunctionalGroupSpec(
                    id=f"residual_stage_{stage_idx}",
                    label="衔接与输出支路",
                    role="output_stage" if stage_idx >= len(ordered_macros) else "bias",
                    stage_order=stage_idx * 10,
                    instances=grp_insts,
                )
            )
            stage_idx += 1

    # Refresh face-to-face orientations on local mirror pairs after macro ordering
    stack_ref_set = {r for s in co_gated_stacks for r in s["refs"]}
    mp_locked_refs = {
        str(r)
        for mp in intent_motifs
        if mp.get("type") == "mirror_block_pair"
        for col_m in mp.get("leftColumns", []) + mp.get("rightColumns", [])
        for r in col_m
    }
    for ci_o in range(len(new_cols) - 1):
        col_l, col_r = new_cols[ci_o], new_cols[ci_o + 1]
        for rl in col_l:
            if rl in stack_ref_set or rl in mp_locked_refs:
                continue
            il = inst_by_ref[rl]
            if il.device_class not in {"mos", "bjt"}:
                continue
            cp = "G" if il.device_class == "mos" else "BASE"
            gn = il.terminals.get(cp, "")
            if not gn or gn in rails:
                continue
            matches_r = [
                rr
                for rr in col_r
                if rr not in stack_ref_set
                and rr not in mp_locked_refs
                and inst_by_ref[rr].device_class == il.device_class
                and inst_by_ref[rr].polarity == il.polarity
                and inst_by_ref[rr].terminals.get(cp, "") == gn
            ]
            if len(matches_r) == 1:
                orientations[rl] = "gate_right"
                orientations[matches_r[0]] = "gate_left"

    return new_cols, new_groups, coarse_meta


def _compact_wide_columns_for_aspect_ratio(
    column_hints: list[list[str]],
    groups: list[FunctionalGroupSpec],
    inst_by_ref: dict[str, InstanceIR],
    default_tiers: dict[str, int],
    locked_refs: set[str],
    rails: set[str],
) -> list[list[str]]:
    """Prevent overly wide horizontal ribbon schematics by folding disjoint subcolumns & passive banks."""
    cols = [list(c) for c in column_hints if c]
    if len(cols) < 6:
        return cols


    group_of = {r: g.id for g in groups for r in g.instances}

    # Pass 1: Fold upper-only (`max_tier <= 2`) + lower-only (`min_tier >= 4`) columns by shared connectivity (across groups too!)
    merged_flags = [False] * len(cols)
    active_count = len(cols)
    for i, col_u in enumerate(cols):
        if active_count < 6 or merged_flags[i] or any(r in locked_refs for r in col_u):
            continue
        max_t_u = max(default_tiers[r] for r in col_u)
        if max_t_u > 2:
            continue

        u_nets = {n for r in col_u for n in inst_by_ref[r].terminals.values()} - rails
        best_j = -1
        best_shared = -1.0
        for j in range(len(cols)):
            if j == i:
                continue
            col_l = cols[j]
            if merged_flags[j] or any(r in locked_refs for r in col_l):
                continue
            min_t_l = min(default_tiers[r] for r in col_l)
            if min_t_l <= max_t_u:
                continue
            lower_bonus = 2.0 if min_t_l >= 4 else 0.0
            same_gid_bonus = 0.5 if group_of.get(col_u[0]) == group_of.get(col_l[0]) else 0.0
            l_nets = {n for r in col_l for n in inst_by_ref[r].terminals.values()} - rails
            sh = len(u_nets & l_nets) + same_gid_bonus + lower_bonus
            if sh > best_shared:
                best_shared = sh
                best_j = j
        if best_j >= 0:
            cols[i] = sorted(col_u + cols[best_j], key=lambda r: default_tiers[r])
            merged_flags[best_j] = True
            active_count -= 1

    cols = [c for idx, c in enumerate(cols) if not merged_flags[idx]]
    if len(cols) < 6:
        return cols


    # Pass 2: Fold wide single-tier 2-pin passive runs (`>= 5` singletons in same `(group, tier)`) into 2 rows
    passive_buckets: dict[tuple[str, int], list[int]] = {}
    for idx, col in enumerate(cols):
        if len(col) == 1 and col[0] not in locked_refs:
            inst = inst_by_ref[col[0]]
            if inst.device_class not in {"mos", "subckt"}:
                key = (group_of.get(col[0], ""), default_tiers[col[0]])
                passive_buckets.setdefault(key, []).append(idx)

    drop_indices: set[int] = set()
    for _, idx_list in passive_buckets.items():
        if len(idx_list) < 5:
            continue
        avail = list(idx_list)
        while len(avail) >= 2:
            i_a = avail.pop(0)
            ra = cols[i_a][0]
            na = set(inst_by_ref[ra].terminals.values()) - rails
            # Pair with a passive sharing a net (e.g. REPG + CEPG on `ep`) if available, else next neighbor
            partner_pos = next(
                (
                    pos
                    for pos, i_b in enumerate(avail)
                    if (set(inst_by_ref[cols[i_b][0]].terminals.values()) - rails) & na
                ),
                0,
            )
            i_b = avail.pop(partner_pos)
            cols[i_a] = [ra, cols[i_b][0]]
            drop_indices.add(i_b)

    return [c for idx, c in enumerate(cols) if idx not in drop_indices]


def _repair_hard_segment_tears(
    column_hints: list[list[str]],
    groups: list[FunctionalGroupSpec],
    branch_abstraction: dict[str, Any],
    column_exempt_refs: set[str] | None = None,
) -> tuple[list[list[str]], list[str]]:
    """Make the draft respect HARD atomic segments (private pure-series nodes / same Super-MOS stack).

    For every HARD segment whose members ended up in different columns or functional groups, gather the
    whole segment into the column (and group) already holding most of its members, so the analyzer's own
    draft always passes `validate-modules`. Devices in `column_exempt_refs` (serpentine ladders, parallel
    banks, horizontal bridges deliberately spread over several columns) are only regrouped, never re-columned.
    Mutates `groups` in place; returns new columns + repair log.
    """
    exempt = column_exempt_refs or set()
    cols = [list(c) for c in column_hints if c]
    log: list[str] = []
    group_by_id = {g.id: g for g in groups}
    for b in branch_abstraction.get("branches", []):
        # Horizontal bridges (RZ+CC, MOSCAP+MOSRES Miller) deliberately occupy several columns: repair groups only.
        allow_column_merge = b.get("kind") not in {"bridge_passive", "shunt_passive"}
        for seg in b.get("hardSegments", []):
            if len(seg) < 2:
                continue
            seg_allow_cols = allow_column_merge and not (set(seg) & exempt)
            col_idx_of = {r: ci for ci, col in enumerate(cols) for r in col}
            grp_of = {r: g.id for g in groups for r in g.instances}
            seg_cols = {col_idx_of[r] for r in seg if r in col_idx_of}
            seg_grps = {grp_of[r] for r in seg if r in grp_of}
            needs_col = seg_allow_cols and len(seg_cols) > 1
            if not needs_col and len(seg_grps) <= 1:
                continue
            col_votes = Counter(col_idx_of[r] for r in seg if r in col_idx_of)
            first_col = col_idx_of.get(seg[0])
            target_ci = (
                max(col_votes, key=lambda ci: (col_votes[ci], ci == first_col, -ci)) if col_votes else None
            )
            grp_votes = Counter(grp_of[r] for r in seg if r in grp_of)
            if needs_col and target_ci is not None:
                target_col = cols[target_ci]
                target_gid = next((grp_of[r] for r in target_col if r in grp_of and r in seg), None)
            else:
                target_col = []
                target_gid = None
            if target_gid is None and grp_votes:
                target_gid = max(grp_votes, key=lambda gid: (grp_votes[gid], gid == grp_of.get(seg[0])))
            if needs_col and target_ci is not None:
                moved: list[str] = []
                for r in seg:
                    ci = col_idx_of.get(r)
                    if ci is not None and ci != target_ci:
                        cols[ci] = [x for x in cols[ci] if x != r]
                        moved.append(r)
                # Insert moved members next to their segment neighbours, preserving top-to-bottom branch order
                for r in moved:
                    pos_in_seg = seg.index(r)
                    anchor_above = next((seg[k] for k in range(pos_in_seg - 1, -1, -1) if seg[k] in target_col), None)
                    if anchor_above is not None:
                        target_col.insert(target_col.index(anchor_above) + 1, r)
                    else:
                        anchor_below = next((seg[k] for k in range(pos_in_seg + 1, len(seg)) if seg[k] in target_col), None)
                        target_col.insert(target_col.index(anchor_below) if anchor_below else len(target_col), r)
            if target_gid in group_by_id:
                for r in seg:
                    for g in groups:
                        if r in g.instances and g.id != target_gid:
                            g.instances.remove(r)
                    if r not in group_by_id[target_gid].instances:
                        group_by_id[target_gid].instances.append(r)
            cols = [c for c in cols if c]
            log.append(f"{b['id']} 硬串联段 {'→'.join(seg)} 合并到同一列/组 `{target_gid}`")
    groups[:] = [g for g in groups if g.instances]
    return cols, log


def analyze_circuit(circuit: NetlistCircuit) -> TopologyAnalysis:
    """Extract candidate analog motifs, potential tiers, ambiguities for LLM, and a clean draft intent."""
    supplies = set(circuit.supply_nets)
    grounds = set(circuit.ground_nets)
    rails = supplies | grounds
    inst_by_ref = circuit.instance_map()
    fanout = _compute_net_fanout(circuit)
    default_tiers = _assign_default_tiers(circuit)

    motifs: list[CandidateMotif] = []
    mos_list = [i for i in circuit.instances if i.device_class == "mos"]

    # Pass 0: Canonical co-gated series MOS stacks + topology-driven device functional roles
    co_gated_stacks = detect_co_gated_mos_stacks(circuit)
    stack_by_member: dict[str, dict[str, Any]] = {
        r: stk for stk in co_gated_stacks for r in stk["refs"]
    }
    device_roles = classify_device_roles(circuit, co_gated_stacks)
    reduced_units = build_reduced_mos_units(circuit, co_gated_stacks, device_roles)

    # 1. Diode-connected MOS
    diode_refs: set[str] = set()
    for m in mos_list:
        if m.terminals["D"] == m.terminals["G"] and m.terminals["D"] not in rails and m.terminals["D"] != m.terminals["S"]:
            diode_refs.add(m.ref)
            motifs.append(
                CandidateMotif(
                    id=f"diode_{m.ref}",
                    motif_type="diode_mos",
                    instances=[m.ref],
                    nets=[m.terminals["D"]],
                    roles={"diode": m.ref},
                    notes=f"{m.ref} 栅漏短接在网络 {m.terminals['D']}，常作电流镜参考端或有源二极管负载。",
                )
            )

    # 2. Differential pairs on canonical Super-MOS units (handles both single-FET and split-length stacked diff pairs!)
    diff_pair_motifs, used_in_diff_pair, used_in_mirror_load = detect_composite_diff_pairs(
        circuit, reduced_units
    )
    motifs.extend(diff_pair_motifs)

    # 3. Current mirrors: MOS sharing gate net and same S rail
    by_gate_rail: dict[tuple[str, str, str], list[InstanceIR]] = defaultdict(list)
    for m in mos_list:
        if device_roles.get(m.ref, {}).get("role") in {"mos_cap", "mos_active_res", "source_follower"}:
            continue
        g_net = m.terminals["G"]
        s_net = m.terminals["S"]
        if g_net not in rails and s_net in rails:
            by_gate_rail[(m.polarity, g_net, s_net)].append(m)

    for (pol, g_net, s_net), members in sorted(by_gate_rail.items()):
        if len(members) < 2:
            continue
        masters = [m.ref for m in members if m.ref in diode_refs]
        slaves = [m.ref for m in members if m.ref not in diode_refs]
        motifs.append(
            CandidateMotif(
                id=f"mirror_{g_net}",
                motif_type="current_mirror",
                instances=[m.ref for m in members],
                nets=[g_net, s_net],
                roles={
                    "gate_net": g_net,
                    "rail_net": s_net,
                    "master": masters[0] if masters else None,
                    "slaves": slaves,
                },
                confidence=1.0 if masters else 0.85,
                notes=(
                    f"共栅电流镜（栅网 {g_net}）：参考管 {masters[0]} → 镜像支路 {slaves}"
                    if masters
                    else f"外部偏置共栅电流源组（栅网 {g_net}）：支路 {[m.ref for m in members]}"
                ),
            )
        )

    # 3b. Co-gated series-stacked composite MOS mirror groups (high device-group pairing priority!)
    stacks_by_group: dict[tuple[str, str], list[dict[str, Any]]] = defaultdict(list)
    for stk in co_gated_stacks:
        stacks_by_group[(stk["polarity"], stk["gate_net"])].append(stk)
    for (pol, g_net), stk_list in sorted(stacks_by_group.items()):
        if len(stk_list) < 2:
            continue
        ordered_stks = sorted(
            stk_list,
            key=lambda s: (not s["is_diode_master"], -int(s["length"]), s["refs"][0]),
        )
        motifs.append(
            CandidateMotif(
                id=f"co_gated_stack_{pol}_{g_net}",
                motif_type="co_gated_stack_mirror",
                instances=[r for s in ordered_stks for r in s["refs"]],
                nets=[g_net],
                roles={
                    "polarity": pol,
                    "gate_net": g_net,
                    "stacks": [list(s["refs"]) for s in ordered_stks],
                    "master_stack": list(ordered_stks[0]["refs"]) if ordered_stks[0]["is_diode_master"] else None,
                    "pairing_priority": "co_gated_series_stack",
                },
                confidence=1.0,
                notes=(
                    f"全共栅串联器件组配对（{pol.upper()} 栅网 {g_net}）："
                    f"{[s['refs'] for s in ordered_stks]}，须保持原子纵叠、优先相邻配对并逐层水平对齐。"
                ),
            )
        )

    # 4. Cascode vertical pairs (same polarity M_cas.S == M_base.D on low-fanout internal net)
    for m_cas in mos_list:
        if m_cas.ref in used_in_diff_pair or m_cas.ref in used_in_mirror_load:
            continue
        if device_roles.get(m_cas.ref, {}).get("role") in {"mos_cap", "mos_active_res", "source_follower"}:
            continue
        mid_net = m_cas.terminals["S"]
        if mid_net in rails:
            continue
        candidates = [
            m_base
            for m_base in mos_list
            if m_base.ref != m_cas.ref
            and m_base.ref not in used_in_diff_pair
            and m_base.ref not in used_in_mirror_load
            and device_roles.get(m_base.ref, {}).get("role") not in {"mos_cap", "mos_active_res", "source_follower"}
            and m_base.polarity == m_cas.polarity
            and m_base.terminals["D"] == mid_net
            and m_base.terminals["S"] in rails
        ]
        if len(candidates) == 1:
            m_base = candidates[0]
            upper_ref, lower_ref = (
                (m_base.ref, m_cas.ref) if m_cas.polarity == "pmos" else (m_cas.ref, m_base.ref)
            )
            motifs.append(
                CandidateMotif(
                    id=f"cascode_{upper_ref}_{lower_ref}",
                    motif_type="cascode_pair",
                    instances=[upper_ref, lower_ref],
                    nets=[mid_net],
                    roles={"upper": upper_ref, "lower": lower_ref, "mid_net": mid_net},
                    notes=f"同列串叠支路：{upper_ref}（上）与 {lower_ref}（下）经内部节点 {mid_net} 串联。",
                )
            )

    # 5. Series RC compensation network (e.g. Rz + Cc sharing 2-pin internal node)
    res_list = [i for i in circuit.instances if i.device_class.startswith("res")]
    cap_list = [i for i in circuit.instances if i.device_class.startswith("cap")]
    rc_refs: set[str] = set()
    for r in res_list:
        for c in cap_list:
            r_pins = {r.terminals["P"], r.terminals["M"]}
            c_pins = {c.terminals["P"], c.terminals["M"]}
            shared = (r_pins & c_pins) - rails
            if len(shared) == 1:
                mid = next(iter(shared))
                if fanout.get(mid, 0) == 2:
                    left_net = next(iter(r_pins - {mid}))
                    right_net = next(iter(c_pins - {mid}))
                    rc_refs.update({r.ref, c.ref})
                    motifs.append(
                        CandidateMotif(
                            id=f"rc_comp_{r.ref}_{c.ref}",
                            motif_type="rc_series",
                            instances=[r.ref, c.ref],
                            nets=[left_net, mid, right_net],
                            roles={
                                "resistor": r.ref,
                                "capacitor": c.ref,
                                "mid_net": mid,
                                "from_net": left_net,
                                "to_net": right_net,
                            },
                            notes=f"串联 RC 网络 ({r.ref} + {c.ref})，跨接 {left_net} → {mid} → {right_net}（典型米勒调零补偿）。",
                        )
                    )

    # 5b. Pass 1: Generalized Miller bridges (MOSCAP + active MOS resistors) & complementary source-follower buffers
    miller_motifs = detect_miller_bridges(circuit, device_roles, existing_rc_refs=rc_refs)
    motifs.extend(miller_motifs)
    miller_refs: set[str] = {r for mm in miller_motifs for r in mm.instances}

    comp_sf_motifs = detect_complementary_sf(circuit, device_roles)
    motifs.extend(comp_sf_motifs)
    comp_sf_refs: set[str] = {r for sm in comp_sf_motifs for r in sm.instances}

    # 6. Regular parallel banks (>= 4 matched-class 2-terminal passives/sources sharing one common plate/rail)
    bank_refs: set[str] = set()
    by_common_plate: dict[tuple[str, str], list[InstanceIR]] = defaultdict(list)
    for inst in circuit.instances:
        if inst.device_class.startswith(("cap", "isource")):
            p_net = inst.terminals.get("P", "")
            m_net = inst.terminals.get("M", "")
            if p_net and p_net != m_net:
                by_common_plate[(inst.device_class, p_net)].append(inst)
            if m_net and m_net != p_net and m_net not in grounds:
                by_common_plate[(inst.device_class, m_net)].append(inst)

    def _nat_key(ref: str) -> list[Any]:
        import re as _re
        low = ref.lower()
        is_dum = 0 if ("dum" in low) else 1
        return [is_dum, *[int(t) if t.isdigit() else t for t in _re.split(r"(\d+)", low)]]

    for (dcls, common_net), members in sorted(by_common_plate.items()):
        uniq = [m for m in members if m.ref not in rc_refs and m.ref not in bank_refs]
        if len(uniq) >= 4:
            # Ensure opposite terminals go to >= 3 distinct nets (e.g. CDAC bit switches SW0..SW7)
            opp_nets = {
                (m.terminals["M"] if m.terminals["P"] == common_net else m.terminals["P"])
                for m in uniq
            }
            if len(opp_nets) >= 3:
                ordered_refs = [m.ref for m in sorted(uniq, key=lambda x: _nat_key(x.ref))]
                bank_refs.update(ordered_refs)
                motifs.append(
                    CandidateMotif(
                        id=f"bank_{common_net}",
                        motif_type="parallel_bank",
                        instances=ordered_refs,
                        nets=[common_net, *sorted(opp_nets)],
                        roles={"common_net": common_net, "instances": ordered_refs},
                        notes=f"规整并联阵列 ({len(ordered_refs)} 单元，公共母线 {common_net})：{ordered_refs}",
                    )
                )

    # 7. Series resistor ladder / serpentine chain (>= 4 two-pin resistors chained end-to-end)
    chain_refs: set[str] = set()
    two_pin_res = [r for r in res_list if r.ref not in rc_refs and len(r.terminals) == 2]
    if len(two_pin_res) >= 4:
        res_by_net: dict[str, list[InstanceIR]] = defaultdict(list)
        for r in two_pin_res:
            for n in (r.terminals["P"], r.terminals["M"]):
                if n not in rails:
                    res_by_net[n].append(r)
        # Find chain endpoints (connected to != 2 resistors in `two_pin_res`)
        visited_r: set[str] = set()
        for start_r in sorted(two_pin_res, key=lambda x: _nat_key(x.ref)):
            if start_r.ref in visited_r:
                continue
            p_deg = len(res_by_net.get(start_r.terminals["P"], []))
            m_deg = len(res_by_net.get(start_r.terminals["M"], []))
            if p_deg == 2 and m_deg == 2:
                continue
            chain = [start_r]
            visited_r.add(start_r.ref)
            cur = start_r
            while True:
                nxt_candidates = [
                    cand
                    for n in (cur.terminals["P"], cur.terminals["M"])
                    if n not in rails and len(res_by_net.get(n, [])) == 2
                    for cand in res_by_net[n]
                    if cand.ref not in visited_r
                ]
                if len(nxt_candidates) != 1:
                    break
                cur = nxt_candidates[0]
                visited_r.add(cur.ref)
                chain.append(cur)
            if len(chain) >= 4:
                c_refs = [x.ref for x in chain]
                chain_refs.update(c_refs)
                motifs.append(
                    CandidateMotif(
                        id=f"ladder_{c_refs[0]}_{c_refs[-1]}",
                        motif_type="serpentine_chain",
                        instances=c_refs,
                        nets=[],
                        roles={"instances": c_refs, "rowsPerColumn": min(4, len(c_refs))},
                        notes=f"串联分压电阻链 ({len(c_refs)} 级)：{c_refs}",
                    )
                )

    # Build functional groups + stage order + orientations + column hints for draft intent
    assigned: set[str] = set()
    groups: list[FunctionalGroupSpec] = []
    orientations: dict[str, str] = {}
    tier_overrides_draft: dict[str, int] = {}
    column_hints: list[list[str]] = []
    intent_motifs: list[dict[str, Any]] = []

    # Group A: Bias generator column(s) (current sources + diode bias masters not inside diff stage)
    diff_motifs = [m for m in motifs if m.motif_type == "diff_pair"]
    diff_member_set = {ref for m in diff_motifs for ref in m.instances}

    bias_insts: list[str] = []
    for inst in circuit.instances:
        if inst.device_class in {"isource", "vsource"}:
            bias_insts.append(inst.ref)
            for m in mos_list:
                if m.ref in diode_refs and m.ref not in diff_member_set:
                    if set(inst.terminals.values()) & {m.terminals["D"], m.terminals["G"]}:
                        # If the diode master is part of a co-gated series stack (e.g. M10-M13), keep the whole atomic stack!
                        stk = stack_by_member.get(m.ref)
                        to_add = list(stk["refs"]) if stk else [m.ref]
                        for r_add in to_add:
                            if r_add not in bias_insts:
                                bias_insts.append(r_add)

    # Also detect cascode bias branches driven by bias nets
    for m in motifs:
        if m.motif_type == "current_mirror" and m.roles.get("master"):
            master_ref = m.roles["master"]
            if master_ref not in diff_member_set and master_ref not in bias_insts:
                # Check if master is driven only by non-signal bias devices
                m_inst = inst_by_ref[master_ref]
                d_net = m_inst.terminals["D"]
                port_names = {p.net for p in circuit.ports}
                if d_net in port_names or any(
                    inst_by_ref[b].terminals.get("D") == d_net for b in bias_insts if b in inst_by_ref
                ):
                    stk = stack_by_member.get(master_ref)
                    for r_add in (list(stk["refs"]) if stk else [master_ref]):
                        if r_add not in bias_insts:
                            bias_insts.append(r_add)

    if bias_insts:
        groups.append(
            FunctionalGroupSpec(
                id="bias_gen",
                label="偏置产生电路",
                role="bias",
                stage_order=0,
                instances=list(bias_insts),
            )
        )
        assigned.update(bias_insts)
        column_hints.append(list(bias_insts))
        for ref in bias_insts:
            inst = inst_by_ref[ref]
            if inst.device_class == "mos":
                # Diode bias master (or its entire co-gated stack) on left faces gate_right towards slave stages!
                in_master_stack = bool(stack_by_member.get(ref, {}).get("is_diode_master"))
                orientations[ref] = "gate_right" if (ref in diode_refs or in_master_stack) else "gate_left"
            else:
                orientations[ref] = "vertical"

    # Pre-detect quasi-mirror sandwich cores wrapping a diff pair (e.g. M17-M21 <- M22-M30 + R1 -> M31-M35)
    # so the enclosed diff pair is emitted together inside its symmetric sandwich group!
    pre_excluded_for_quasi = set(assigned) | diff_member_set | rc_refs | miller_refs | comp_sf_refs
    quasi_cores_pre = detect_quasi_mirror_and_sandwich_cores(
        circuit,
        device_roles=device_roles,
        diff_motifs=diff_motifs,
        excluded_refs=pre_excluded_for_quasi,
    )
    sandwich_cores = [q for q in quasi_cores_pre if q.get("enclosedDiffPairId")]
    sandwich_diff_ids = {str(q["enclosedDiffPairId"]) for q in sandwich_cores}
    diff_by_id = {dm.id: dm for dm in diff_motifs}

    def _emit_diff_pair_columns(dm: CandidateMotif) -> list[list[str]]:
        """Populate orientations, tier overrides, intent_motifs, and columns for a single or composite diff pair."""
        left_ref = dm.roles["left"]
        right_ref = dm.roles["right"]
        tail_ref = dm.roles.get("tail_device")
        load_l = dm.roles.get("active_load_left")
        load_r = dm.roles.get("active_load_right")
        left_stack = list(dm.roles.get("left_stack") or [left_ref])
        right_stack = list(dm.roles.get("right_stack") or [right_ref])
        tail_stack = list(dm.roles.get("tail_stack") or ([tail_ref] if tail_ref else []))
        load_l_stack = list(dm.roles.get("load_left_stack") or ([load_l] if load_l else []))
        load_r_stack = list(dm.roles.get("load_right_stack") or ([load_r] if load_r else []))

        for r in left_stack:
            orientations[r] = "gate_left"
        for r in right_stack:
            orientations[r] = "gate_right"
        for r in tail_stack:
            orientations[r] = "gate_left"
        for r in load_l_stack:
            orientations[r] = "gate_right"
        for r in load_r_stack:
            orientations[r] = "gate_left"

        # Assign clean vertical potential tiers for composite stacked diff pairs (4-high legs + tail at tier 5)
        if dm.roles.get("is_composite"):
            is_nmos_diff = inst_by_ref[left_ref].polarity == "nmos"
            if is_nmos_diff:
                for i_l, r in enumerate(load_l_stack):
                    tier_overrides_draft[r] = min(2, 1 + i_l)
                for i_r, r in enumerate(load_r_stack):
                    tier_overrides_draft[r] = min(2, 1 + i_r)
                base_d = 3 if len(left_stack) >= 2 else 4
                for i_d, r in enumerate(left_stack):
                    tier_overrides_draft[r] = min(4, base_d + i_d)
                for i_d, r in enumerate(right_stack):
                    tier_overrides_draft[r] = min(4, base_d + i_d)
                for r in tail_stack:
                    tier_overrides_draft[r] = 5
            else:
                for r in tail_stack:
                    tier_overrides_draft[r] = 1
                for i_d, r in enumerate(left_stack):
                    tier_overrides_draft[r] = min(3, 2 + i_d)
                for i_d, r in enumerate(right_stack):
                    tier_overrides_draft[r] = min(3, 2 + i_d)
                for i_l, r in enumerate(load_l_stack):
                    tier_overrides_draft[r] = min(5, 4 + i_l)
                for i_r, r in enumerate(load_r_stack):
                    tier_overrides_draft[r] = min(5, 4 + i_r)

        emitted_cols: list[list[str]] = []
        if load_l_stack and load_r_stack:
            is_pmos_diff = inst_by_ref[left_ref].polarity == "pmos"
            left_col = [*left_stack, *load_l_stack] if is_pmos_diff else [*load_l_stack, *left_stack]
            right_col = [*right_stack, *load_r_stack] if is_pmos_diff else [*load_r_stack, *right_stack]
            emitted_cols.append(left_col)
            if tail_stack:
                emitted_cols.append(tail_stack)
            emitted_cols.append(right_col)
        else:
            emitted_cols.append(left_stack)
            if tail_stack:
                emitted_cols.append(tail_stack)
            emitted_cols.append(right_stack)

        intent_motifs.append(
            {
                "type": "diff_pair",
                "left": left_ref,
                "right": right_ref,
                "tail": tail_ref,
                "loadLeft": load_l,
                "loadRight": load_r,
                "style": "back_to_back",
            }
        )
        return emitted_cols

    # Group B: Standalone differential input stage(s) (non-sandwich)
    standalone_diff_motifs = [dm for dm in diff_motifs if dm.id not in sandwich_diff_ids]

    def _are_diff_pairs_truly_coupled(dm0: CandidateMotif, dm1: CandidateMotif) -> bool:
        l0, r0 = inst_by_ref[dm0.roles["left"]], inst_by_ref[dm0.roles["right"]]
        l1, r1 = inst_by_ref[dm1.roles["left"]], inst_by_ref[dm1.roles["right"]]
        if {l0.terminals["G"], r0.terminals["G"]} != {l1.terminals["G"], r1.terminals["G"]}:
            return False
        # Rail-to-rail complementary input pair (NMOS + PMOS)
        if l0.polarity != l1.polarity:
            return True
        # Same-polarity dual diff pair ONLY if their differential drains or active loads couple directly
        d0 = {dm0.roles.get("drain_left", ""), dm0.roles.get("drain_right", "")} - rails
        d1 = {dm1.roles.get("drain_left", ""), dm1.roles.get("drain_right", "")} - rails
        if d0 & d1:
            return True
        l0_r = dm0.roles.get("active_load_right")
        l1_l = dm1.roles.get("active_load_left")
        if l0_r and l1_l and inst_by_ref[l0_r].terminals.get("G") == inst_by_ref[l1_l].terminals.get("G"):
            return True
        return False

    coupled_dual_diff = (
        len(standalone_diff_motifs) == 2
        and _are_diff_pairs_truly_coupled(standalone_diff_motifs[0], standalone_diff_motifs[1])
    )
    coupled_diff_members: list[str] = []

    for idx, dm in enumerate(standalone_diff_motifs):
        g_members = [r for r in dm.instances if r not in assigned]
        assigned.update(g_members)
        if coupled_dual_diff:
            coupled_diff_members.extend(g_members)
        else:
            groups.append(
                FunctionalGroupSpec(
                    id=f"input_diff_{idx + 1}",
                    label="级联差分输入级" if dm.roles.get("is_composite") else "差分输入级",
                    role="input_diff",
                    stage_order=10 + idx * 5,
                    instances=g_members,
                )
            )
        column_hints.extend(_emit_diff_pair_columns(dm))

    if coupled_dual_diff and coupled_diff_members:
        groups.append(
            FunctionalGroupSpec(
                id="input_diff_1",
                label="双尾耦合差分输入与有源负载级",
                role="input_diff",
                stage_order=10,
                instances=coupled_diff_members,
            )
        )
        # If inner adjacent active loads (dm0.load_r and dm1.load_l) share a current-mirror Gate net,
        # orient them face-to-face across the central boundary for a 1-hop mirror tie!
        l0_r = standalone_diff_motifs[0].roles.get("active_load_right")
        l1_l = standalone_diff_motifs[1].roles.get("active_load_left")
        if l0_r and l1_l and inst_by_ref[l0_r].terminals.get("G") == inst_by_ref[l1_l].terminals.get("G"):
            orientations[l0_r] = "gate_right"
            orientations[l1_l] = "gate_left"

    # Group C: True Folded-Cascode full stacks (only when full 4-FET PMOS+NMOS columns or diff-pair folding branches exist;
    # standalone 2-FET mirror cascodes stay unassigned so Group E chains them vertically with lower branches/resistors!)
    cascode_motifs = [m for m in motifs if m.motif_type == "cascode_pair"]
    unassigned_cascodes = [
        m for m in cascode_motifs if any(r not in assigned for r in m.instances)
    ]
    if unassigned_cascodes:
        cas_members: list[str] = []
        by_drain_col: list[list[str]] = []
        for m in unassigned_cascodes:
            for r in m.instances:
                if r not in assigned and r not in cas_members:
                    cas_members.append(r)

        pmos_stacks = [
            m.instances for m in unassigned_cascodes if inst_by_ref[m.instances[0]].polarity == "pmos"
        ]
        nmos_stacks = [
            m.instances for m in unassigned_cascodes if inst_by_ref[m.instances[0]].polarity == "nmos"
        ]
        matched_n: set[int] = set()
        for p_stack in pmos_stacks:
            p_bot_drain = inst_by_ref[p_stack[-1]].terminals["D"]
            match_idx = next(
                (
                    ni
                    for ni, n_stack in enumerate(nmos_stacks)
                    if ni not in matched_n and inst_by_ref[n_stack[0]].terminals["D"] == p_bot_drain
                ),
                None,
            )
            if match_idx is not None:
                matched_n.add(match_idx)
                by_drain_col.append([*p_stack, *nmos_stacks[match_idx]])
            else:
                by_drain_col.append(list(p_stack))
        for ni, n_stack in enumerate(nmos_stacks):
            if ni not in matched_n:
                by_drain_col.append(list(n_stack))

        diff_drain_nets = {
            inst_by_ref[r].terminals["D"]
            for dm in diff_motifs
            for r in (dm.roles["left"], dm.roles["right"])
        }
        cas_all_nets = {
            n
            for c in by_drain_col
            for r in c
            for p, n in inst_by_ref[r].terminals.items()
            if p != "B" and n not in rails
        }
        is_folded_cascode_core = any(len(c) >= 4 for c in by_drain_col) or bool(diff_drain_nets & cas_all_nets)

        if is_folded_cascode_core:
            if len(by_drain_col) == 2 and len(by_drain_col[0]) == len(by_drain_col[1]):
                for r in by_drain_col[0]:
                    orientations.setdefault(r, "gate_right")
                for r in by_drain_col[1]:
                    orientations.setdefault(r, "gate_left")
                intent_motifs.append(
                    {
                        "type": "symmetric_columns",
                        "leftColumn": by_drain_col[0],
                        "rightColumn": by_drain_col[1],
                    }
                )
            else:
                for col in by_drain_col:
                    for r in col:
                        orientations.setdefault(r, "gate_left")

            for col in by_drain_col:
                column_hints.append([r for r in col if r not in assigned])

            groups.append(
                FunctionalGroupSpec(
                    id="cascode_stage",
                    label="共源共栅叠层支路",
                    role="cascode_core",
                    stage_order=20,
                    instances=cas_members,
                )
            )
            assigned.update(cas_members)

    # Group D: Compensation networks (e.g. Rz + Cc bridging Stage 1 and Stage 2)
    comp_members = [r for r in rc_refs if r not in assigned]
    if comp_members:
        groups.append(
            FunctionalGroupSpec(
                id="compensation",
                label="频率补偿网络",
                role="compensation",
                stage_order=25,
                instances=comp_members,
            )
        )
        assigned.update(comp_members)
        for rc_m in [m for m in motifs if m.motif_type == "rc_series"]:
            r_ref = rc_m.roles["resistor"]
            c_ref = rc_m.roles["capacitor"]
            r_inst = inst_by_ref[r_ref]
            c_inst = inst_by_ref[c_ref]
            mid = rc_m.roles["mid_net"]
            # Orient horizontally left-to-right from Stage 1 (`from_net`) -> `mid` -> Stage 2 (`to_net`)
            orientations[r_ref] = "horizontal_l2r" if r_inst.terminals["M"] == mid else "horizontal_r2l"
            orientations[c_ref] = "horizontal_l2r" if c_inst.terminals["P"] == mid else "horizontal_r2l"
            column_hints.append([r_ref])
            column_hints.append([c_ref])

    # Group D1b: Generalized Miller compensation bridges (MOSCAP + active MOS resistors) paired with host CS amp
    miller_group_members: set[str] = set()
    for mm_idx, mm in enumerate(miller_motifs):
        chain_refs = [r for r in mm.roles.get("chain", mm.instances) if r not in assigned]
        host_ref = mm.roles.get("host_amp")
        load_refs_m = [r for r in mm.roles.get("host_loads", []) if r not in assigned]
        host_col = [r for r in ([host_ref] if host_ref and host_ref not in assigned else []) + load_refs_m]
        if not chain_refs and not host_col:
            continue
        grp_insts = list(chain_refs) + list(host_col)
        assigned.update(grp_insts)
        miller_group_members.update(grp_insts)
        groups.append(
            FunctionalGroupSpec(
                id=f"gain_miller_{host_ref or (mm_idx + 1)}",
                label=f"增益与 Miller 补偿级 ({host_ref or mm_idx + 1})",
                role="compensation",
                stage_order=18 + mm_idx,
                instances=grp_insts,
            )
        )
        # Emit horizontal Miller bridge (MOSCAP + horizontal active MOS resistors on tier 3) -> host CS amp column
        # Compute left-to-right terminal flow along `chain_refs` so each horizontal MOS aligns S/D directly left-to-right!
        from_net_m = str(mm.roles.get("from_net", ""))
        cur_left_net = from_net_m
        bridge_horiz_refs: list[str] = []
        for r in chain_refs:
            inst_r = inst_by_ref[r]
            role_r = device_roles.get(r, {}).get("role", "")
            tier_overrides_draft[r] = 3
            if role_r == "mos_cap":
                # MOSCAP: gate faces left toward `from_net` (or right if flipped)
                orientations[r] = "gate_left" if inst_r.terminals.get("G") == cur_left_net else "gate_right"
                cur_left_net = inst_r.terminals.get("D", cur_left_net) if inst_r.terminals.get("G") == cur_left_net else inst_r.terminals.get("G", cur_left_net)
                column_hints.append([r])
                bridge_horiz_refs.append(r)
            elif role_r == "mos_active_res":
                # Horizontal active MOS resistor (`R90`/`MXR90`): channel runs left->right, Gate faces DOWN!
                # For PMOS: `horizontal_gate_down` (R90) puts S on LEFT and D on RIGHT;
                # for NMOS: `horizontal_gate_down` (R90) puts D on LEFT and S on RIGHT.
                left_pin_natural = "S" if inst_r.polarity == "pmos" else "D"
                right_pin_natural = "D" if inst_r.polarity == "pmos" else "S"
                if inst_r.terminals.get(left_pin_natural) == cur_left_net:
                    orientations[r] = "horizontal_gate_down"
                    cur_left_net = inst_r.terminals.get(right_pin_natural, cur_left_net)
                else:
                    orientations[r] = "horizontal_gate_down_flipped"
                    cur_left_net = inst_r.terminals.get(left_pin_natural, cur_left_net)
                column_hints.append([r])
                bridge_horiz_refs.append(r)
            elif inst_r.device_class.startswith(("res", "cap")):
                orientations[r] = "horizontal_l2r" if inst_r.terminals.get("P") == cur_left_net else "horizontal_r2l"
                cur_left_net = inst_r.terminals.get("M", cur_left_net) if inst_r.terminals.get("P") == cur_left_net else inst_r.terminals.get("P", cur_left_net)
                column_hints.append([r])
                bridge_horiz_refs.append(r)
            else:
                orientations[r] = "gate_left" if inst_r.device_class == "mos" else "vertical"
                column_hints.append([r])

        if len(bridge_horiz_refs) >= 2:
            intent_motifs.append(
                {
                    "type": "horizontal_bridge",
                    "id": f"hbridge_{mm.id}",
                    "instances": list(bridge_horiz_refs),
                    "hostAmp": host_ref,
                    "hostLoads": list(load_refs_m),
                }
            )
        if host_col:
            for r in host_col:
                orientations[r] = "gate_left" if inst_by_ref[r].device_class == "mos" else "vertical"
                if r == host_ref:
                    tier_overrides_draft[r] = 1 if inst_by_ref[r].polarity == "pmos" else 5
                else:
                    tier_overrides_draft[r] = 5 if inst_by_ref[r].polarity == "nmos" else 1
            column_hints.append(host_col)

    # Group D2: Regular parallel banks (with automatic 1-to-1 vertical driver stacking!) & serpentine resistor chains
    for bm in [m for m in motifs if m.motif_type == "parallel_bank"]:
        b_members = [r for r in bm.instances if r not in assigned]
        if len(b_members) >= 4:
            cnet = bm.roles["common_net"]
            # Check if bank opposite terminals connect 1-to-1 (`fanout <= 3`) to unassigned driver blocks/MOS (e.g. SCDAC XU0..XU5)
            driver_for_cap: dict[str, str] = {}
            claimed_drivers: set[str] = set()
            for r in b_members:
                inst = inst_by_ref[r]
                opp_net = inst.terminals["M"] if inst.terminals.get("P") == cnet else inst.terminals.get("P", "")
                if not opp_net or opp_net in rails or fanout.get(opp_net, 99) > 3:
                    continue
                drv_cands = [
                    cand.ref
                    for cand in circuit.instances
                    if cand.ref not in assigned
                    and cand.ref not in b_members
                    and cand.ref not in claimed_drivers
                    and cand.device_class in {"subckt", "mos"}
                    and opp_net in {n for p_k, n in cand.terminals.items() if p_k != "B"}
                ]
                if len(drv_cands) == 1:
                    driver_for_cap[r] = drv_cands[0]
                    claimed_drivers.add(drv_cands[0])

            stack_drivers = len(driver_for_cap) >= max(2, len(b_members) // 2)
            grp_members = list(b_members) + ([driver_for_cap[r] for r in b_members if r in driver_for_cap] if stack_drivers else [])
            groups.append(
                FunctionalGroupSpec(
                    id=bm.id,
                    label="规整并联阵列与驱动单元" if stack_drivers else "规整并联阵列",
                    role="array_bank",
                    stage_order=26,
                    instances=grp_members,
                )
            )
            assigned.update(grp_members)
            for r in b_members:
                inst = inst_by_ref[r]
                orientations[r] = "vertical" if inst.terminals.get("P") == cnet else "vertical_flipped"
                if stack_drivers:
                    tier_overrides_draft[r] = 1
                    if r in driver_for_cap:
                        drv = driver_for_cap[r]
                        tier_overrides_draft[drv] = 4
                        column_hints.append([r, drv])
                    else:
                        column_hints.append([r])
                else:
                    column_hints.append([r])
            intent_motifs.append(
                {
                    "type": "parallel_bank",
                    "id": bm.id,
                    "instances": b_members,
                    "commonTopNet": cnet,
                    "stackedDrivers": list(driver_for_cap.values()) if stack_drivers else [],
                }
            )

    for sm in [m for m in motifs if m.motif_type == "serpentine_chain"]:
        s_members = [r for r in sm.instances if r not in assigned]
        if len(s_members) >= 4:
            rows_per_col = int(sm.roles.get("rowsPerColumn", 4))
            groups.append(
                FunctionalGroupSpec(
                    id=sm.id,
                    label="串联电阻梯阵列",
                    role="resistor_ladder",
                    stage_order=27,
                    instances=s_members,
                )
            )
            assigned.update(s_members)
            for col_i in range(0, len(s_members), rows_per_col):
                chunk = s_members[col_i : col_i + rows_per_col]
                col_idx = col_i // rows_per_col
                if col_idx % 2 == 1:
                    chunk = list(reversed(chunk))
                    for r in chunk:
                        orientations[r] = "vertical_flipped"
                else:
                    for r in chunk:
                        orientations[r] = "vertical"
                column_hints.append(chunk)
            intent_motifs.append(
                {
                    "type": "serpentine_chain",
                    "id": sm.id,
                    "instances": s_members,
                    "rowsPerColumn": rows_per_col,
                }
            )

    # Group D3: Multi-column mirror-isomorphic half-circuit pairs (WL graph isomorphism + dual-BFS)
    # e.g. folded emitter-follower differential amp `{Q3,R3,M21,M22,M25,Q4,R4} <-> {Q5,R5,M23,M24,M26,Q6,R6}`!
    mirror_pairs = detect_mirror_subcircuit_pairs(circuit, excluded_refs=assigned)
    for mp_idx, mp in enumerate(mirror_pairs):
        left_insts = [r for r in mp["leftInstances"] if r not in assigned]
        right_insts = [r for r in mp["rightInstances"] if r not in assigned]
        if len(left_insts) < 4 or len(left_insts) != len(right_insts):
            continue
        bij = {u: v for u, v in mp["pairs"] if u in set(left_insts) and v in set(right_insts)}
        if len(bij) < 4:
            continue
        left_cols_raw = _bundle_vertical_branches(
            left_insts,
            inst_by_ref,
            default_tiers,
            fanout,
            rails,
            atomic_stacks=[list(s["refs"]) for s in co_gated_stacks],
        )
        if len(left_cols_raw) < 2:
            continue

        # Order left columns outer -> inner: place the column sharing a common mirror gate/net
        # with the right half on the INNERMOST side (next to the central symmetry axis!) so
        # shared bias loads (e.g. M25/M26) bridge 0-bend face-to-face across the center, while
        # external-input columns (e.g. Q3/Q5) face outward!
        def _inner_affinity(l_col: list[str]) -> tuple[int, int, str]:
            shared_ctrl_cnt = 0
            shared_top_cnt = 0
            for r in l_col:
                iu, iv = inst_by_ref[r], inst_by_ref[bij[r]]
                if iu.device_class in {"mos", "bjt"}:
                    cp = "G" if iu.device_class == "mos" else "BASE"
                    gn = iu.terminals.get(cp, "")
                    if gn and gn not in rails and gn == iv.terminals.get(cp, ""):
                        shared_ctrl_cnt += 1
                        if default_tiers[r] <= 2:
                            shared_top_cnt += 1
            return (shared_top_cnt, shared_ctrl_cnt, l_col[0])

        left_cols_ordered = sorted(left_cols_raw, key=_inner_affinity)
        right_cols_ordered = [[bij[r] for r in l_col] for l_col in left_cols_ordered]

        # Fold symmetrically: [L_outer, ..., L_inner, R_inner, ..., R_outer]
        folded_cols = list(left_cols_ordered) + list(reversed(right_cols_ordered))
        col_idx_in_left = {r: ci for ci, col in enumerate(left_cols_ordered) for r in col}

        # Assign strictly complementary horizontal mirror orientations:
        # - Innermost column pair sharing the same gate/base net faces INWARD (face-to-face across center)
        # - Internal signals driven from an inner column face INWARD; signals from outer column or external face OUTWARD!
        n_lcols = len(left_cols_ordered)
        for ci_l, l_col in enumerate(left_cols_ordered):
            for r_l in l_col:
                r_r = bij[r_l]
                iu = inst_by_ref[r_l]
                if iu.device_class not in {"mos", "bjt"}:
                    orientations.setdefault(r_l, "vertical")
                    orientations.setdefault(r_r, "vertical")
                    continue
                cp = "G" if iu.device_class == "mos" else "BASE"
                net_l = iu.terminals.get(cp, "")
                net_r = inst_by_ref[r_r].terminals.get(cp, "")
                face_inward = False
                if ci_l == n_lcols - 1 and net_l and net_l not in rails and net_l == net_r:
                    face_inward = True
                elif net_l and net_l not in rails:
                    # Check if net_l is driven by a column closer to center inside left half
                    drv_cols = {
                        col_idx_in_left[o]
                        for o in left_insts
                        if o != r_l
                        and any(p not in {"B", "G", "BASE"} and n == net_l for p, n in inst_by_ref[o].terminals.items())
                    }
                    if drv_cols and min(drv_cols) > ci_l:
                        face_inward = True
                if face_inward:
                    orientations[r_l] = "gate_right"
                    orientations[r_r] = "gate_left"
                else:
                    orientations[r_l] = "gate_left"
                    orientations[r_r] = "gate_right"

        mp_members = [r for col in folded_cols for r in col]
        groups.append(
            FunctionalGroupSpec(
                id=f"mirror_pair_{mp_idx + 1}",
                label="镜像对称放大级",
                role="gain_stage",
                stage_order=28 + mp_idx,
                instances=mp_members,
            )
        )
        assigned.update(mp_members)
        for col in folded_cols:
            column_hints.append(col)
        intent_motifs.append(
            {
                "type": "mirror_block_pair",
                "id": f"mirror_pair_{mp_idx + 1}",
                "leftColumns": left_cols_ordered,
                "rightColumns": right_cols_ordered,
                "pairs": [[u, bij[u]] for col in left_cols_ordered for u in col],
            }
        )
        motifs.append(
            CandidateMotif(
                id=f"mirror_pair_{mp_idx + 1}",
                motif_type="mirror_block_pair",
                instances=mp_members,
                nets=[n for pair in mp["privateNetPairs"] for n in pair],
                roles={
                    "leftColumns": left_cols_ordered,
                    "rightColumns": right_cols_ordered,
                    "pairs": [[u, bij[u]] for col in left_cols_ordered for u in col],
                },
                confidence=1.0,
                notes=(
                    f"图同构镜像对称子电路（每侧 {len(left_insts)} 器件 / {len(left_cols_ordered)} 列）："
                    f"左 {left_cols_ordered} ↔ 右 {right_cols_ordered}，按中轴对称折叠并逐对水平对齐。"
                ),
            )
        )

    # Group D3b: Fault-tolerant quasi-mirror sandwich cores wrapping a diff pair (e.g. Left_Wing <- Diff_Core (+R1) -> Right_Wing)
    sandwich_locked_refs: set[str] = set()
    for sc_idx, sc in enumerate(sandwich_cores):
        dm_id = str(sc.get("enclosedDiffPairId") or "")
        dm_center = diff_by_id.get(dm_id)
        if dm_center is None:
            continue
        left_insts = [r for r in sc["leftInstances"] if r not in assigned]
        right_insts = [r for r in sc["rightInstances"] if r not in assigned]
        bridge_devs = [r for r in sc.get("bridgeDevices", []) if r not in assigned]
        diff_insts = [r for r in dm_center.instances if r not in assigned]
        if not left_insts or len(left_insts) != len(right_insts) or not diff_insts:
            continue
        bij = {u: v for u, v in sc["pairs"] if u in set(left_insts) and v in set(right_insts)}
        if len(bij) != len(left_insts):
            continue

        # Bundle left wing vertical branches (keep mos_cap devices in their own inner coupling column)
        cap_left = [r for r in left_insts if device_roles.get(r, {}).get("role") == "mos_cap"]
        noncap_left = [r for r in left_insts if r not in set(cap_left)]
        left_cols_raw = _bundle_vertical_branches(
            noncap_left,
            inst_by_ref,
            default_tiers,
            fanout,
            rails,
            atomic_stacks=[list(s["refs"]) for s in co_gated_stacks],
        )
        for cr in cap_left:
            left_cols_raw.append([cr])

        # Order left wing columns outer -> inner (closest to enclosed diff pair's `drain_left` goes innermost!)
        da_center = dm_center.roles.get("drain_left", "")

        def _wing_inner_rank(l_col: list[str]) -> tuple[int, int, str]:
            touches_da = int(
                any(
                    da_center in {n for p, n in inst_by_ref[r].terminals.items() if p != "B"}
                    for r in l_col
                )
            )
            # Taller DC current column sits between outer feedback FET and inner coupling MOSCAP
            return (touches_da, len(l_col), l_col[0])

        left_cols_ordered = sorted(left_cols_raw, key=_wing_inner_rank)
        right_cols_ordered = [[bij[r] for r in l_col] for l_col in left_cols_ordered]

        # Assign complementary mirror orientations & clean tiers on left/right wings
        col_idx_l = {r: ci for ci, col in enumerate(left_cols_ordered) for r in col}
        port_net_set = {p.net for p in circuit.ports}
        for ci_l, l_col in enumerate(left_cols_ordered):
            for r_l in l_col:
                r_r = bij[r_l]
                iu = inst_by_ref[r_l]
                role_l = device_roles.get(r_l, {}).get("role", "")
                if role_l == "mos_cap":
                    tier_overrides_draft[r_l] = 2
                    tier_overrides_draft[r_r] = 2
                    orientations[r_l] = "gate_right"
                    orientations[r_r] = "gate_left"
                elif iu.device_class == "mos":
                    g_l = iu.terminals.get("G", "")
                    # External bias gate faces outward; internally driven gate faces toward its driving inner column
                    if g_l in port_net_set:
                        face_inward = False
                    else:
                        drv_cols = {
                            col_idx_l[o]
                            for o in left_insts
                            if o != r_l
                            and any(p not in {"B", "G"} and n == g_l for p, n in inst_by_ref[o].terminals.items())
                        }
                        face_inward = bool(drv_cols and max(drv_cols) > ci_l)
                    if face_inward:
                        orientations[r_l] = "gate_right"
                        orientations[r_r] = "gate_left"
                    else:
                        orientations[r_l] = "gate_left"
                        orientations[r_r] = "gate_right"
                    # Set crisp vertical potential tiers for 3-high cascode column (PMOS=1, cascode NMOS=3, bot NMOS=5)
                    if len(l_col) == 3:
                        t_map = {0: 1, 1: 3, 2: 5}
                        idx_in_c = l_col.index(r_l)
                        tier_overrides_draft[r_l] = t_map[idx_in_c]
                        tier_overrides_draft[r_r] = t_map[idx_in_c]
                    elif len(l_col) == 1:
                        tier_overrides_draft[r_l] = 1 if iu.polarity == "pmos" else 5
                        tier_overrides_draft[r_r] = 1 if iu.polarity == "pmos" else 5

        # Emit center diff pair columns + hang optional passive bridge (e.g. R1) cleanly UNDER tail column
        # (NEVER insert an extra column between left_leg, tail, and right_leg, which bloats diff-pair symmetry 2.5x!)
        center_cols = _emit_diff_pair_columns(dm_center)
        if bridge_devs:
            tail_r_sc = dm_center.roles.get("tail_device")
            for br in bridge_devs:
                orientations[br] = "horizontal_l2r"
                tier_overrides_draft[br] = 6
            tail_col_idx = next(
                (ci_c for ci_c, col_c in enumerate(center_cols) if tail_r_sc and tail_r_sc in col_c),
                -1,
            )
            if tail_col_idx >= 0:
                center_cols[tail_col_idx] = list(center_cols[tail_col_idx]) + list(bridge_devs)
            else:
                center_cols.insert(min(1, len(center_cols)), list(bridge_devs))

        # Assemble left_wing -> center_diff -> right_wing (reversed outer->inner becomes inner->outer)
        right_cols_mirrored = list(reversed(right_cols_ordered))
        all_sc_cols = [*left_cols_ordered, *center_cols, *right_cols_mirrored]
        sc_members = [r for col in all_sc_cols for r in col]
        assigned.update(sc_members)
        sandwich_locked_refs.update(sc_members)

        groups.append(
            FunctionalGroupSpec(
                id=f"sandwich_core_{sc_idx + 1}",
                label="三明治镜像对称核心",
                role="cascode_core",
                stage_order=26 + sc_idx,
                instances=sc_members,
            )
        )
        for col in all_sc_cols:
            column_hints.append(col)

        intent_motifs.append(
            {
                "type": "mirror_block_pair",
                "id": f"sandwich_wings_{sc_idx + 1}",
                "leftColumns": left_cols_ordered,
                "rightColumns": right_cols_ordered,
                "pairs": [[u, bij[u]] for col in left_cols_ordered for u in col],
            }
        )
        asym_note = ""
        if sc.get("asymmetricTaps"):
            t_desc = ", ".join(f"{t['ref']}.{t['pin']}@{t['net']}" for t in sc["asymmetricTaps"][:3])
            asym_note = f"；容忍单端引出破缺 ({t_desc})"
        motifs.append(
            CandidateMotif(
                id=f"sandwich_core_{sc_idx + 1}",
                motif_type="sandwich_mirror_core",
                instances=sc_members,
                nets=[n for pair in sc["privateNetPairs"] for n in pair],
                roles={
                    "leftColumns": left_cols_ordered,
                    "rightColumns": right_cols_ordered,
                    "pairs": [[u, bij[u]] for col in left_cols_ordered for u in col],
                    "enclosedDiffPairId": dm_id,
                    "bridgeDevices": list(bridge_devs),
                    "asymmetricTaps": list(sc.get("asymmetricTaps", [])),
                },
                confidence=0.95,
                notes=(
                    f"三明治准镜像对称核心：左翼 {left_cols_ordered} ← 中央差分级 {dm_id}"
                    + (f"（桥接 {bridge_devs}）" if bridge_devs else "")
                    + f" → 右翼 {right_cols_mirrored}{asym_note}。"
                ),
            )
        )

    # Group E0: Complementary push-pull source-follower output buffers (e.g. M38/M39 + C1)
    comp_sf_locked_refs: set[str] = set()
    for sf_idx, sfm in enumerate(comp_sf_motifs):
        sf_insts = [r for r in sfm.instances if r not in assigned]
        if not sf_insts:
            continue
        assigned.update(sf_insts)
        comp_sf_locked_refs.update(sf_insts)
        u_n = sfm.roles["upper_nmos"]
        l_p = sfm.roles["lower_pmos"]
        shunts = [r for r in sfm.roles.get("shunt_loads", []) if r in sf_insts]
        sf_col = [r for r in (u_n, l_p) if r in sf_insts]
        if sf_col:
            orientations[u_n] = "gate_left"
            orientations[l_p] = "gate_left"
            tier_overrides_draft[u_n] = 2
            tier_overrides_draft[l_p] = 4
            column_hints.append(sf_col)
        for sh in shunts:
            orientations[sh] = "vertical"
            tier_overrides_draft[sh] = 4
            column_hints.append([sh])
        groups.append(
            FunctionalGroupSpec(
                id=f"output_buffer_{sf_idx + 1}" if sf_idx > 0 else "output_buffer",
                label="互补源极跟随器输出缓冲",
                role="output_stage",
                stage_order=60 + sf_idx,
                instances=sf_insts,
            )
        )

    # Group E: Remaining gain / bias / output branches (complete VDD->VSS vertical pairing + connectivity ordering)
    remaining = [i.ref for i in circuit.instances if i.ref not in assigned]
    if remaining:
        output_port_nets = {p.net for p in circuit.ports if p.direction == "output"}
        load_refs: list[str] = []
        branch_pool: list[str] = []

        for r in remaining:
            inst = inst_by_ref[r]
            terms_set = set(inst.terminals.values())
            # Isolate shunt RC to ground on an output port so we can place it immediately beside its driving column!
            if (
                inst.device_class.startswith(("cap", "res"))
                and (terms_set & grounds)
                and (terms_set & output_port_nets)
            ):
                load_refs.append(r)
                orientations.setdefault(r, "vertical")
            else:
                branch_pool.append(r)

        bundled_cols = _bundle_vertical_branches(
            branch_pool,
            inst_by_ref,
            default_tiers,
            fanout,
            rails,
            atomic_stacks=[list(s["refs"]) for s in co_gated_stacks],
        )
        bundled_cols = _order_columns_and_orient_gates(
            bundled_cols, column_hints, inst_by_ref, fanout, rails, diode_refs, circuit, orientations
        )
        out_stage_refs: list[str] = []
        for col in bundled_cols:
            column_hints.append(col)
            for r in col:
                out_stage_refs.append(r)
                inst = inst_by_ref[r]
                orientations.setdefault(
                    r, "gate_left" if inst.device_class in {"mos", "bjt"} else "vertical"
                )

        if out_stage_refs:
            groups.append(
                FunctionalGroupSpec(
                    id="gain_output_stage",
                    label="增益与输出级",
                    role="output_stage",
                    stage_order=30,
                    instances=out_stage_refs,
                )
            )
        if load_refs:
            trailing_loads: list[str] = []
            group_by_member = {r: g for g in groups for r in g.instances}
            for lr in load_refs:
                l_inst = inst_by_ref[lr]
                sig_nets = (set(l_inst.terminals.values()) - rails) & output_port_nets
                # Find existing column driving `sig_nets`
                drv_col_idx = next(
                    (
                        ci
                        for ci in range(len(column_hints) - 1, -1, -1)
                        if any(
                            sig_nets & (set(inst_by_ref[cr].terminals.values()) - rails)
                            for cr in column_hints[ci]
                        )
                    ),
                    -1,
                )
                if 0 <= drv_col_idx < len(column_hints) - 1:
                    # Insert shunt load column immediately adjacent to its driving column
                    # (to the LEFT of a diff-pair left leg so we never split `(left, tail, right)`!)
                    diff_lefts = {dm.roles["left"] for dm in diff_motifs}
                    drv_col = column_hints[drv_col_idx]
                    ins_pos = drv_col_idx if any(r in diff_lefts for r in drv_col) else (drv_col_idx + 1)
                    drv_rep = drv_col[0]
                    column_hints.insert(ins_pos, [lr])
                    tier_overrides_draft[lr] = 5
                    if drv_rep in group_by_member:
                        group_by_member[drv_rep].instances.append(lr)
                        group_by_member[lr] = group_by_member[drv_rep]
                    else:
                        trailing_loads.append(lr)
                else:
                    column_hints.append([lr])
                    trailing_loads.append(lr)

            if trailing_loads:
                groups.append(
                    FunctionalGroupSpec(
                        id="output_load",
                        label="输出负载",
                        role="output_stage",
                        stage_order=35,
                        instances=trailing_loads,
                    )
                )

    # Pass 3: Directed inter-block signal-flow backbone & bottleneck-cut macro stage analysis
    signal_backbone = build_signal_flow_backbone(circuit, motifs, device_roles, fanout)

    # When multi-stage backbone features are present (sandwich core, bottleneck cuts, or complementary SF),
    # align left-to-right column & group reading order directly to the directed signal-flow DAG!
    has_multistage_backbone = bool(
        sandwich_cores
        or signal_backbone.get("selectedCutNets")
        or comp_sf_motifs
        or miller_motifs
    )
    if has_multistage_backbone and signal_backbone.get("blocks"):
        bb_blocks = signal_backbone["blocks"]
        bb_block_by_ref = {r: b for b in bb_blocks for r in b["instances"]}
        bb_stage_rank = {
            r: (
                st_idx,
                int(bb_block_by_ref[r].get("stageLocalOrder", 0)),
                int(bb_block_by_ref[r].get("depth", 0)),
                int(bb_block_by_ref[r].get("order", 0)),
            )
            for st_idx, st in enumerate(signal_backbone.get("macroStages", []))
            for r in st["instances"]
            if r in bb_block_by_ref
        }

        # Split catch-all `gain_output_stage` into per-backbone-block groups so columns like M15/M16
        # (upstream of sandwich core) and M36/M37 (downstream of sandwich core) land on their exact side!
        refined_groups: list[FunctionalGroupSpec] = []
        for g in groups:
            if g.id == "gain_output_stage":
                by_bid: dict[str, list[str]] = defaultdict(list)
                for r in g.instances:
                    b = bb_block_by_ref.get(r)
                    by_bid[b["id"] if b else "misc"].append(r)
                if len(by_bid) >= 2:
                    for bid, mems in by_bid.items():
                        b_meta = next((b for b in bb_blocks if b["id"] == bid), {})
                        refined_groups.append(
                            FunctionalGroupSpec(
                                id=bid,
                                label=str(b_meta.get("label", "增益支路")),
                                role="gain_stage",
                                stage_order=g.stage_order,
                                instances=mems,
                            )
                        )
                    continue
            refined_groups.append(g)
        groups = refined_groups

        # Order columns by macro-stage -> block topological depth -> block order, preserving intra-group order
        indexed_cols = list(enumerate([c for c in column_hints if c]))

        def _col_backbone_key(item: tuple[int, list[str]]) -> tuple[int, int, int, int, int]:
            orig_i, col = item
            ranks = [bb_stage_rank[r] for r in col if r in bb_stage_rank]
            if ranks:
                best = min(ranks)
                return (best[0], best[1], best[2], best[3], orig_i)
            return (999, 999, 999, 999, orig_i)

        indexed_cols.sort(key=_col_backbone_key)
        column_hints = [c for _, c in indexed_cols]

        col_pos_of_ref = {r: ci for ci, col in enumerate(column_hints) for r in col}
        groups.sort(key=lambda g: min((col_pos_of_ref.get(r, 999) for r in g.instances), default=999))
        for g_idx, g in enumerate(groups):
            g.stage_order = (g_idx + 1) * 10

    # Coarse-to-Fine Macro-Anchor-First ordering for complex multi-macro analog sheets:
    # lock mirror/local-mirror/stack macros in left-to-right order first, then insert residual devices!
    coarse_blocks_meta: list[dict[str, Any]] = []
    if mirror_pairs and not has_multistage_backbone and len(column_hints) >= 8:
        column_hints, groups, coarse_blocks_meta = _apply_coarse_macro_first_ordering(
            column_hints=column_hints,
            groups=groups,
            intent_motifs=intent_motifs,
            co_gated_stacks=co_gated_stacks,
            inst_by_ref=inst_by_ref,
            fanout=fanout,
            rails=rails,
            diode_refs=diode_refs,
            circuit=circuit,
            orientations=orientations,
        )

    # Fold overly wide horizontal sheets (`>= 7` columns) into balanced 2D row/column grids
    locked_for_folding: set[str] = set()
    for cb_item in coarse_blocks_meta:
        for cb_col in cb_item.get("columns", []):
            locked_for_folding.update(str(r) for r in cb_col)
    for m_item in intent_motifs:
        for k_role in ("left", "right", "tail", "loadLeft", "loadRight"):
            if m_item.get(k_role):
                locked_for_folding.add(str(m_item[k_role]))
        for r_m in m_item.get("instances", []) + m_item.get("stackedDrivers", []):
            locked_for_folding.add(str(r_m))
        for r_m in m_item.get("leftColumn", []) + m_item.get("rightColumn", []):
            locked_for_folding.add(str(r_m))
        for col_m in m_item.get("leftColumns", []) + m_item.get("rightColumns", []):
            for r_m in col_m:
                locked_for_folding.add(str(r_m))
    for r_comp in comp_members:
        locked_for_folding.add(r_comp)
    locked_for_folding.update(miller_group_members)
    locked_for_folding.update(sandwich_locked_refs)
    locked_for_folding.update(comp_sf_locked_refs)
    if has_multistage_backbone:
        # Preserve left-to-right signal-flow progression across macro amplifier stages
        locked_for_folding.update(i.ref for i in circuit.instances)
    column_hints = _compact_wide_columns_for_aspect_ratio(
        column_hints, groups, inst_by_ref, default_tiers, locked_for_folding, rails
    )

    # Universal branch abstraction is the single source of truth for atomic (HARD) series segments:
    # repair the draft so the analyzer's own output always passes `validate-modules`.
    branch_abstraction = build_branch_abstraction(
        circuit,
        device_roles=device_roles,
        motifs=motifs,
        co_gated_stacks=co_gated_stacks,
        net_fanout=fanout,
    )
    column_exempt_refs = {
        str(r)
        for m in intent_motifs
        if m.get("type") in {"serpentine_chain", "parallel_bank", "horizontal_bridge", "matrix_array"}
        for r in (m.get("instances") or [])
    }
    column_hints, hard_repair_log = _repair_hard_segment_tears(
        column_hints, groups, branch_abstraction, column_exempt_refs
    )
    branch_abstraction["draftRepairs"] = hard_repair_log

    # Net classification policies
    net_policies: dict[str, str] = {}
    critical_nets: list[str] = []
    ambiguities: list[AmbiguityItem] = []

    port_by_net = {p.net: p for p in circuit.ports}
    cross_stage_by_net = {item["net"]: item for item in signal_backbone.get("crossStageNets", [])}
    for net in circuit.nets:
        if net in rails:
            net_policies[net] = "supply_local"
            continue
        # Check if net connects across >= 3 groups (e.g., global bias line)
        groups_touched = {
            g.id
            for g in groups
            for ref in g.instances
            if net in inst_by_ref[ref].terminals.values()
        }
        cs_meta = cross_stage_by_net.get(net)
        if len(groups_touched) >= 3 and fanout.get(net, 0) >= 3:
            net_policies[net] = "perimeter_bus"
            ambiguities.append(
                AmbiguityItem(
                    id=f"net_policy_{net}",
                    category="net_policy",
                    subjects=[net],
                    question=(
                        f"偏置/公共网络 {net!r} 跨越 {len(groups_touched)} 个功能组（扇出 {fanout[net]}）："
                        "应走底部/顶部外围汇流母线 (`perimeter_bus`) 还是穿核心直连 (`full_wire`)？"
                    ),
                    options=["perimeter_bus", "full_wire", "label_stub"],
                    default_choice="perimeter_bus",
                    rationale="跨多级偏置线走外围母线或启用线标短桩可避免横穿差分输入对与主信号走廊。",
                )
            )
        elif has_multistage_backbone and cs_meta and cs_meta.get("suggestedPolicy") in {"perimeter_bus", "label_stub"}:
            # Cross-stage shared ports / long feedforward nets: route along perimeter_bus by default in strict mode
            net_policies[net] = "perimeter_bus"
            ambiguities.append(
                AmbiguityItem(
                    id=f"net_policy_{net}",
                    category="net_policy",
                    subjects=[net],
                    question=(
                        f"跨阶段/多支路网络 {net!r}（扇出 {fanout[net]}，{cs_meta.get('reason', '')}）："
                        "是否采用外围母线 (`perimeter_bus`) 或在 `--allow-wire-labels` 下改用线标短桩 (`label_stub`)？"
                    ),
                    options=["perimeter_bus", "label_stub", "full_wire"],
                    default_choice="perimeter_bus",
                    rationale="跨宏观大块的共享输入/偏置口或前馈长线走外围母线或局部线标可显著降低主干交叉。",
                )
            )
        else:
            net_policies[net] = "full_wire"
            if net in port_by_net or len(groups_touched) >= 2 or fanout.get(net, 0) >= 2:
                critical_nets.append(net)

    # Targeted high-value LLM ambiguities surfaced by Pass 1-3 semantic analysis
    for b_amb in signal_backbone.get("boundaryAmbiguousBlocks", []):
        ambiguities.append(
            AmbiguityItem(
                id=f"macro_boundary_{b_amb['blockId']}",
                category="macro_boundary",
                subjects=list(b_amb["instances"]),
                question=(
                    f"转换支路 {b_amb['blockId']} ({b_amb['instances']}) 夹在两条 1-edge 瓶颈割边 "
                    f"`{b_amb['upstreamCutNet']}`（左）与 `{b_amb['downstreamCutNet']}`（右）之间："
                    "它应归入右侧主放大级（默认），还是归入左侧辅助增益级？"
                ),
                options=["right_main_stage", "left_aux_stage"],
                default_choice="right_main_stage",
                rationale="沿上游割边切分可让电流镜转换级紧邻主放大级输入侧，与典型论文分块一致。",
            )
        )
    for sc in sandwich_cores:
        if sc.get("asymmetricTaps"):
            t_str = ", ".join(f"{t['ref']}.{t['pin']}@{t['net']}" for t in sc["asymmetricTaps"])
            ambiguities.append(
                AmbiguityItem(
                    id=f"quasi_symmetry_{sc.get('enclosedDiffPairId', 'core')}",
                    category="quasi_symmetry",
                    subjects=list(sc["leftInstances"]) + list(sc["rightInstances"]),
                    question=(
                        f"检测到准镜像三明治核心（左翼 {sc['leftInstances']} ↔ 右翼 {sc['rightInstances']}，"
                        f"包夹 {sc.get('enclosedDiffPairId')}，桥接 {sc.get('bridgeDevices', [])}），"
                        f"仅因单端外部抽头 ({t_str}) 非严格同构：是否保持严格左右镜像对称布局？"
                    ),
                    options=["lock_sandwich_mirror", "break_symmetry"],
                    default_choice="lock_sandwich_mirror",
                    rationale="单端输出抽头是模拟差分核心的常规破缺，保持翼部镜像对称可显著提升电路可读性。",
                )
            )
    for mm in miller_motifs:
        if mm.roles.get("bridge_kind") == "mos_cap_active_res":
            ambiguities.append(
                AmbiguityItem(
                    id=f"bridge_style_{mm.id}",
                    category="bridge_style",
                    subjects=list(mm.instances),
                    question=(
                        f"广义 Miller 补偿桥 {mm.instances}（MOSCAP + 浮动有源 MOS 电阻）跨接在 "
                        f"{mm.roles.get('host_amp')} 的 {mm.roles.get('from_net')} ↔ {mm.roles.get('to_net')} 之间："
                        "保持紧邻增益级的独立补偿列，还是后续以水平桥接形式居中排布？"
                    ),
                    options=["adjacent_comp_columns", "horizontal_inline_bridge"],
                    default_choice="adjacent_comp_columns",
                    rationale="当前版支持标准竖直 MOS 朝向，已将其锁定在宿主增益级旁以保持紧凑无交叉。",
                )
            )

    if len(groups) >= 2:
        ambiguities.append(
            AmbiguityItem(
                id="stage_reading_order",
                category="stage_order",
                subjects=[g.id for g in sorted(groups, key=lambda x: x.stage_order)],
                question="请核对从左到右的功能级阅读顺序是否符合电路信号流向（偏置 → 输入级 → 中间/补偿级 → 输出级）。",
                options=["keep_draft_order", "custom_reorder_in_intent"],
                default_choice="keep_draft_order",
                rationale="大模型可根据电路具体拓扑（如反馈回路或折叠支路位置）调整各组 stage_order。",
            )
        )

    # Port side defaults
    diff_gate_nets = {
        inst_by_ref[r].terminals["G"]
        for dm in diff_motifs
        for r in (dm.roles["left"], dm.roles["right"])
    }
    pure_gate_nets = {
        n
        for n in circuit.nets
        if all(
            pin == "G"
            for inst in circuit.instances
            for pin, pnet in inst.terminals.items()
            if pnet == n
        )
    }
    port_sides: dict[str, str] = {}
    col_of_ref_final = {r: ci for ci, col in enumerate(column_hints) for r in col}
    hbridge_refs_all = {
        str(r)
        for m in intent_motifs
        if m.get("type") == "horizontal_bridge"
        for r in m.get("instances", [])
    }
    for p in circuit.ports:
        if p.net in supplies or p.net in grounds:
            port_sides[p.name] = "local_repeat"
        elif p.direction == "output":
            port_sides[p.name] = "right"
        elif (p.net in diff_gate_nets or p.net in pure_gate_nets) and fanout.get(p.net, 0) <= 2:
            port_sides[p.name] = "near_pin"
        elif fanout.get(p.net, 0) >= 2 and p.direction == "input" and p.net in pure_gate_nets:
            # Single-column or horizontal-bridge local gate bias (e.g. Vb7 on horizontal M12/M13) -> near_pin!
            driven_refs = [
                inst.ref
                for inst in circuit.instances
                if any(pin != "B" and n == p.net for pin, n in inst.terminals.items())
            ]
            driven_cols = {col_of_ref_final[r] for r in driven_refs if r in col_of_ref_final}
            is_hbridge_only = bool(driven_refs) and all(r in hbridge_refs_all for r in driven_refs)
            port_sides[p.name] = "near_pin" if (len(driven_cols) == 1 or is_hbridge_only) else "local_repeat"
        else:
            touched_refs = [
                inst.ref
                for inst in circuit.instances
                if any(pin != "B" and n == p.net for pin, n in inst.terminals.items())
            ]
            touched_cols = {col_of_ref_final[r] for r in touched_refs if r in col_of_ref_final}
            is_hbridge_only = bool(touched_refs) and all(r in hbridge_refs_all for r in touched_refs)
            if p.direction == "input" and (len(touched_cols) == 1 or is_hbridge_only) and fanout.get(p.net, 0) <= 3:
                port_sides[p.name] = "near_pin"
            else:
                port_sides[p.name] = "left"

    draft_intent = SchematicIntent(
        cell_name=circuit.cell_name,
        groups=sorted(groups, key=lambda g: g.stage_order),
        motifs=intent_motifs,
        orientations=orientations,
        tier_overrides=tier_overrides_draft,
        column_order_hints=[c for c in column_hints if c],
        net_policies=net_policies,
        critical_nets=critical_nets,
        port_sides=port_sides,
        spacing=SpacingConfig(),
        authored_by="analyzer_draft",
        llm_notes="由确定性拓扑分析器（Pass 0-3 复合归约 + 准对称 + 信号流主干）生成的初始草案；请先阅读 semantic_digest.md 核对宏观主干与裁决项后再微调 intent.json。",
    )

    # Surface novel quasi-symmetric cores & feedback loops discovered by universal branch abstraction
    existing_amb_subjects = {tuple(sorted(a.subjects)) for a in ambiguities}
    existing_motif_sets = [
        set(m.instances)
        for m in motifs
        if m.motif_type in {"diff_pair", "sandwich_mirror_core", "mirror_block_pair"}
    ]
    for qp in branch_abstraction.get("quasiSymmetricPairs", []):
        if qp["kind"] not in {"coupled_mirror_pair", "quasi_symmetric_core"}:
            continue
        subj_set = set(qp["leftInstances"] + qp["rightInstances"])
        subj = sorted(subj_set)
        if tuple(subj) in existing_amb_subjects:
            continue
        # Skip sub-pairs already fully enclosed inside a recognized diff_pair / sandwich_mirror_core / mirror_block_pair
        if any(subj_set <= ms for ms in existing_motif_sets):
            continue
        existing_amb_subjects.add(tuple(subj))
        ambiguities.append(
            AmbiguityItem(
                id=f"branch_qsym_{qp['id'].lower()}",
                category="quasi_symmetry",
                subjects=subj,
                question=(
                    f"支路折叠检测到{qp['reason']}（结构相似度={qp['similarity']}）："
                    "是否将该对支路锁定为一个核心对称/镜像功能模块（`mirror_pair`）？"
                ),
                options=["lock_mirror_module", "keep_independent"],
                default_choice="lock_mirror_module",
                rationale="跨器件类型（MOS+BJT+R）准对称支路对往往是带隙核心或误差放大器主体，优先锚定可稳定全图骨架。",
            )
        )
    for fl in branch_abstraction.get("feedbackLoops", []):
        subj_set = set(fl["instances"])
        subj = sorted(subj_set)
        if len(fl["branches"]) >= 2 and tuple(subj) not in existing_amb_subjects and not any(subj_set <= ms for ms in existing_motif_sets):
            existing_amb_subjects.add(tuple(subj))
            ambiguities.append(
                AmbiguityItem(
                    id=f"feedback_loop_{fl['id'].lower()}",
                    category="feedback_loop",
                    subjects=subj,
                    question=(
                        f"剥离全局偏置轨后检测到闭环反馈核心 {fl['id']}（含支路 {', '.join(fl['branches'])}，"
                        f"反馈控制网 {', '.join(fl['ctrlNets'][:5])}）：请确认环内各功能模块（如误差放大器 vs 基准核/输出级）的左右排布顺序。"
                    ),
                    options=["error_amp_left_core_right", "core_left_error_amp_right", "custom_module_plan"],
                    default_choice="error_amp_left_core_right",
                    rationale="强反馈闭环电路没有单一方向割边，建议先在 module_plan.json 中显式指定核心与环内模块顺序。",
                )
            )

    analysis_obj = TopologyAnalysis(
        circuit=circuit,
        motifs=motifs,
        net_fanout=fanout,
        net_classifications_default=net_policies,
        default_tiers=default_tiers,
        ambiguities_for_llm=ambiguities,
        draft_intent=draft_intent.to_dict(),
        device_roles=device_roles,
        signal_backbone=signal_backbone,
        branch_abstraction=branch_abstraction,
    )
    analysis_obj.semantic_digest_md = render_semantic_digest_md(analysis_obj)
    return analysis_obj
