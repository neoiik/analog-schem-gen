"""Deterministic analog motif analyzer + draft IntentSpec builder for LLM co-execution."""

from __future__ import annotations

from collections import Counter, defaultdict
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


def _assign_default_tiers(circuit: NetlistCircuit) -> dict[str, int]:
    """Assign vertical potential tiers (1 = top PMOS near VDD .. 5 = bottom NMOS near VSS)."""
    supplies = set(circuit.supply_nets)
    grounds = set(circuit.ground_nets)
    tiers: dict[str, int] = {}

    for inst in circuit.instances:
        if inst.device_class == "subckt":
            tiers[inst.ref] = 3
        elif inst.polarity == "pmos":
            # PMOS with Source directly on VDD -> Tier 1 (PMOS_TOP); cascode PMOS -> Tier 2 (PMOS_CAS)
            tiers[inst.ref] = 1 if inst.terminals.get("S") in supplies else 2
        elif inst.polarity == "nmos":
            # NMOS with Source directly on VSS -> Tier 5 (NMOS_BOT); diff-pair/cascode NMOS -> Tier 4 (NMOS_MID)
            tiers[inst.ref] = 5 if inst.terminals.get("S") in grounds else 4
        elif inst.device_class in {"isource", "vsource"}:
            p_net = inst.terminals.get("P", "")
            m_net = inst.terminals.get("M", "")
            if p_net in supplies and m_net not in grounds:
                tiers[inst.ref] = 1
            elif m_net in grounds and p_net not in supplies:
                tiers[inst.ref] = 5
            else:
                tiers[inst.ref] = 3
        else:
            # Passives (resistors, capacitors, diodes)
            p_net = inst.terminals.get("P", "")
            m_net = inst.terminals.get("M", "")
            if p_net in supplies or m_net in supplies:
                tiers[inst.ref] = 1
            elif p_net in grounds or m_net in grounds:
                tiers[inst.ref] = 5
            else:
                # Floating / inter-stage signal network (e.g. Miller Rz/Cc) -> Mid-rail Tier 3
                tiers[inst.ref] = 3
    return tiers


def _bundle_vertical_branches(
    pool_refs: list[str],
    inst_by_ref: dict[str, InstanceIR],
    default_tiers: dict[str, int],
    fanout: dict[str, int],
    rails: set[str],
) -> list[list[str]]:
    """Bundle vertically connected series/degenerated/divider/mirror devices into aligned columns.

    Pairs devices `(u, v)` with `tier[u] < tier[v]` sharing a non-rail vertical terminal net
    (`D/S` for MOS, `P/M` for passives/sources) in descending specificity (`fanout == 2` series
    nodes first), eliminating unnecessary horizontal step jogs and reducing horizontal sprawl.
    """
    if not pool_refs:
        return []

    def _vert_nets(ref: str) -> set[str]:
        inst = inst_by_ref[ref]
        if inst.device_class == "mos":
            return {inst.terminals[p] for p in ("D", "S") if p in inst.terminals} - rails
        if inst.device_class != "subckt":
            return {inst.terminals[p] for p in ("P", "M") if p in inst.terminals} - rails
        return set()

    vnets = {r: _vert_nets(r) for r in pool_refs}
    edges: list[tuple[int, str, str]] = []
    for i_u, u in enumerate(pool_refs):
        for v in pool_refs[i_u + 1 :]:
            tu, tv = default_tiers[u], default_tiers[v]
            if tu == tv:
                continue
            hi, lo = (u, v) if tu < tv else (v, u)
            shared = vnets[hi] & vnets[lo]
            if not shared:
                continue
            best_n = min(shared, key=lambda n: fanout.get(n, 99))
            fn = fanout.get(best_n, 99)
            if fn > 8:
                continue
            base = 120 if fn <= 2 else (85 if fn == 3 else (60 - 3 * fn))
            iu, iv = inst_by_ref[hi], inst_by_ref[lo]
            mos_bonus = 0
            if iu.polarity == "pmos" and iv.polarity == "nmos":
                if iu.terminals.get("D") == iv.terminals.get("D"):
                    mos_bonus += 18
                if iu.terminals.get("G") == iv.terminals.get("G") and iu.terminals.get("G") not in rails:
                    mos_bonus += 35
            tier_pen = 12 * abs(default_tiers[lo] - default_tiers[hi] - 1)
            edges.append((base + mos_bonus - tier_pen, hi, lo))

    edges.sort(key=lambda e: (-e[0], pool_refs.index(e[1]), pool_refs.index(e[2])))
    col_list: list[list[str]] = [[r] for r in pool_refs]
    col_idx_of: dict[str, int] = {r: i for i, r in enumerate(pool_refs)}

    for _, hi, lo in edges:
        c_hi = col_idx_of[hi]
        c_lo = col_idx_of[lo]
        if c_hi == c_lo:
            continue
        list_hi = col_list[c_hi]
        list_lo = col_list[c_lo]
        if max(default_tiers[x] for x in list_hi) < min(default_tiers[y] for y in list_lo):
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

    if len(cols) >= 2:
        n_c = len(cols)
        adj_w: list[list[float]] = [[0.0] * n_c for _ in range(n_c)]
        for i in range(n_c):
            for j in range(i + 1, n_c):
                sh = cnets[i] & cnets[j]
                w = sum(16.0 / max(2, fanout.get(n, 2)) for n in sh)
                adj_w[i][j] = adj_w[j][i] = w

        # Left-anchor score: connection to already-placed left stages or input ports
        left_bias: list[float] = []
        for i in range(n_c):
            sc = sum(10.0 / max(2, fanout.get(n, 2)) for n in (cnets[i] & prior_union))
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

        # Quick 2-opt refinement to eliminate unnecessary horizontal skips
        def _chain_score(seq: list[int]) -> float:
            s = sum(adj_w[seq[k]][seq[k + 1]] for k in range(len(seq) - 1))
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

    # Orient MOS gates toward the column(s) driving or sharing their Gate net!
    full_seq = list(prior_cols) + list(cols)
    offset = len(prior_cols)
    for local_i, col in enumerate(cols):
        abs_i = offset + local_i
        for r in col:
            inst = inst_by_ref[r]
            if inst.device_class != "mos" or r in orientations:
                continue
            g_net = inst.terminals.get("G", "")
            if not g_net or g_net in rails:
                orientations[r] = "gate_left"
                continue
            left_pull = 1.8 if g_net in in_port_nets else 0.0
            right_pull = 1.8 if g_net in out_port_nets else 0.0
            for j, other_col in enumerate(full_seq):
                if j == abs_i:
                    continue
                w_col = 0.0
                for ro in other_col:
                    io = inst_by_ref[ro]
                    for po, no in io.terminals.items():
                        if po == "B" or no != g_net:
                            continue
                        w_col += 2.2 if (po != "G" or ro in diode_refs) else 1.2
                if j < abs_i:
                    left_pull += w_col
                else:
                    right_pull += w_col
            orientations[r] = "gate_right" if (right_pull > left_pull + 0.1) else "gate_left"

    return cols


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


def analyze_circuit(circuit: NetlistCircuit) -> TopologyAnalysis:
    """Extract candidate analog motifs, potential tiers, ambiguities for LLM, and a clean draft intent."""
    supplies = set(circuit.supply_nets)
    grounds = set(circuit.ground_nets)
    rails = supplies | grounds
    inst_by_ref = circuit.instance_map()
    fanout = _compute_net_fanout(circuit)
    default_tiers = _assign_default_tiers(circuit)

    motifs: list[CandidateMotif] = []
    used_in_diff_pair: set[str] = set()
    used_in_mirror_load: set[str] = set()

    mos_list = [i for i in circuit.instances if i.device_class == "mos"]

    # 1. Diode-connected MOS
    diode_refs: set[str] = set()
    for m in mos_list:
        if m.terminals["D"] == m.terminals["G"] and m.terminals["D"] not in rails:
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

    # 2. Differential pairs: two matched MOS sharing internal tail net S
    by_tail: dict[tuple[str, str | None, str], list[InstanceIR]] = defaultdict(list)
    for m in mos_list:
        s_net = m.terminals["S"]
        if s_net not in rails:
            by_tail[(m.polarity, m.kind, s_net)].append(m)

    for (pol, kind, tail_net), members in sorted(by_tail.items()):
        if len(members) != 2:
            continue
        m_a, m_b = members
        if m_a.terminals["G"] == m_b.terminals["G"] or m_a.terminals["D"] == m_b.terminals["D"]:
            continue
        if m_a.params.get("w") != m_b.params.get("w") or m_a.params.get("l") != m_b.params.get("l"):
            continue
        # Order left vs right cleanly: prefer VINP/INP/+ on left, VINN/INN/- on right
        ga, gb = m_a.terminals["G"].upper(), m_b.terminals["G"].upper()
        if ("N" in ga and "P" in gb) and not ("P" in ga):
            m_a, m_b = m_b, m_a

        used_in_diff_pair.update({m_a.ref, m_b.ref})

        # Find tail current source/sink connected to tail_net
        tail_dev = next(
            (
                x.ref
                for x in mos_list
                if x.ref not in {m_a.ref, m_b.ref}
                and x.terminals["D"] == tail_net
                and x.terminals["S"] in rails
            ),
            None,
        )
        # Find matched active load pair directly above/below diff pair drains
        # (exclude folded-cascode base sinks where `da`/`db` also feeds a same-polarity cascode Source!)
        da, db = m_a.terminals["D"], m_b.terminals["D"]
        cands_a = [
            x
            for x in mos_list
            if x.polarity != pol
            and x.ref not in used_in_mirror_load
            and x.terminals["D"] == da
            and x.terminals["S"] in rails
            and not any(
                c.ref != x.ref and c.polarity == x.polarity and c.terminals.get("S") == da
                for c in mos_list
            )
        ]
        cands_b = [
            x
            for x in mos_list
            if x.polarity != pol
            and x.ref not in used_in_mirror_load
            and x.terminals["D"] == db
            and x.terminals["S"] in rails
            and not any(
                c.ref != x.ref and c.polarity == x.polarity and c.terminals.get("S") == db
                for c in mos_list
            )
        ]
        best_load_pair: tuple[str, str] | None = None
        best_load_rank = -1
        for xa in cands_a:
            for xb in cands_b:
                if xa.ref == xb.ref or xa.terminals["S"] != xb.terminals["S"]:
                    continue
                same_wl = xa.params.get("w") == xb.params.get("w") and xa.params.get("l") == xb.params.get("l")
                if xa.terminals["G"] == xb.terminals["G"]:
                    rk = 30 + (5 if same_wl else 0)
                elif (xa.ref in diode_refs) and (xb.ref in diode_refs) and same_wl:
                    rk = 25
                elif (xa.ref not in diode_refs) and (xb.ref not in diode_refs) and same_wl:
                    rk = 20
                else:
                    continue
                if rk > best_load_rank:
                    best_load_rank = rk
                    best_load_pair = (xa.ref, xb.ref)

        roles: dict[str, Any] = {
            "left": m_a.ref,
            "right": m_b.ref,
            "tail_net": tail_net,
            "style": "back_to_back",
        }
        inst_group = [m_a.ref, m_b.ref]
        if tail_dev:
            roles["tail_device"] = tail_dev
            inst_group.append(tail_dev)
        load_a = best_load_pair[0] if best_load_pair else None
        load_b = best_load_pair[1] if best_load_pair else None
        if load_a and load_b:
            roles["active_load_left"] = load_a
            roles["active_load_right"] = load_b
            used_in_mirror_load.update({load_a, load_b})
            inst_group.extend([load_a, load_b])

        motifs.append(
            CandidateMotif(
                id=f"diff_{m_a.ref}_{m_b.ref}",
                motif_type="diff_pair",
                instances=inst_group,
                nets=[m_a.terminals["G"], m_b.terminals["G"], tail_net, da, db],
                roles=roles,
                notes=f"差分输入对 ({m_a.ref}, {m_b.ref}) 共尾节点 {tail_net}"
                + (f"，尾流管 {tail_dev}" if tail_dev else "")
                + (f"，有源镜像负载 ({load_a}, {load_b})" if load_a and load_b else ""),
            )
        )

    # 3. Current mirrors: MOS sharing gate net and same S rail
    by_gate_rail: dict[tuple[str, str, str], list[InstanceIR]] = defaultdict(list)
    for m in mos_list:
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

    # 4. Cascode vertical pairs (same polarity M_cas.S == M_base.D on low-fanout internal net)
    for m_cas in mos_list:
        if m_cas.ref in used_in_diff_pair:
            continue
        mid_net = m_cas.terminals["S"]
        if mid_net in rails:
            continue
        candidates = [
            m_base
            for m_base in mos_list
            if m_base.ref != m_cas.ref
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
                        if m.ref not in bias_insts:
                            bias_insts.append(m.ref)

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
                    bias_insts.append(master_ref)

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
                # Diode bias master on left faces gate_right towards slave stages!
                orientations[ref] = "gate_right" if ref in diode_refs else "gate_left"
            else:
                orientations[ref] = "vertical"

    # Group B: Differential input stage(s)
    coupled_dual_diff = (
        len(diff_motifs) == 2
        and {
            inst_by_ref[diff_motifs[0].roles["left"]].terminals["G"],
            inst_by_ref[diff_motifs[0].roles["right"]].terminals["G"],
        }
        == {
            inst_by_ref[diff_motifs[1].roles["left"]].terminals["G"],
            inst_by_ref[diff_motifs[1].roles["right"]].terminals["G"],
        }
    )
    coupled_diff_members: list[str] = []

    for idx, dm in enumerate(diff_motifs):
        left_ref = dm.roles["left"]
        right_ref = dm.roles["right"]
        tail_ref = dm.roles.get("tail_device")
        load_l = dm.roles.get("active_load_left")
        load_r = dm.roles.get("active_load_right")
        g_members = [r for r in dm.instances if r not in assigned]
        assigned.update(g_members)
        if coupled_dual_diff:
            coupled_diff_members.extend(g_members)
        else:
            groups.append(
                FunctionalGroupSpec(
                    id=f"input_diff_{idx + 1}",
                    label="差分输入级",
                    role="input_diff",
                    stage_order=10 + idx * 5,
                    instances=g_members,
                )
            )

        # Back-to-back diff pair: left gate_left (R0), right gate_right (MY)
        orientations[left_ref] = "gate_left"
        orientations[right_ref] = "gate_right"
        if tail_ref:
            orientations[tail_ref] = "gate_left"
        if load_l and load_r:
            # Default face-to-face active mirror load above diff pair: gates face center!
            orientations[load_l] = "gate_right"
            orientations[load_r] = "gate_left"
            column_hints.append([load_l, left_ref])
            if tail_ref:
                column_hints.append([tail_ref])
            column_hints.append([load_r, right_ref])
        else:
            column_hints.append([left_ref])
            if tail_ref:
                column_hints.append([tail_ref])
            column_hints.append([right_ref])

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
        l0_r = diff_motifs[0].roles.get("active_load_right")
        l1_l = diff_motifs[1].roles.get("active_load_left")
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
            branch_pool, inst_by_ref, default_tiers, fanout, rails
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
                    r, "gate_left" if inst.device_class == "mos" else "vertical"
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

    # Fold overly wide horizontal sheets (`>= 7` columns) into balanced 2D row/column grids
    locked_for_folding: set[str] = set()
    for m_item in intent_motifs:
        for k_role in ("left", "right", "tail", "loadLeft", "loadRight"):
            if m_item.get(k_role):
                locked_for_folding.add(str(m_item[k_role]))
        for r_m in m_item.get("instances", []) + m_item.get("stackedDrivers", []):
            locked_for_folding.add(str(r_m))
        for r_m in m_item.get("leftColumn", []) + m_item.get("rightColumn", []):
            locked_for_folding.add(str(r_m))
    for r_comp in comp_members:
        locked_for_folding.add(r_comp)
    column_hints = _compact_wide_columns_for_aspect_ratio(
        column_hints, groups, inst_by_ref, default_tiers, locked_for_folding, rails
    )

    # Net classification policies
    net_policies: dict[str, str] = {}
    critical_nets: list[str] = []
    ambiguities: list[AmbiguityItem] = []

    port_by_net = {p.net: p for p in circuit.ports}
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
                    options=["perimeter_bus", "full_wire"],
                    default_choice="perimeter_bus",
                    rationale="跨多级偏置线走外围母线可避免横穿差分输入对与主信号走廊。",
                )
            )
        else:
            net_policies[net] = "full_wire"
            if net in port_by_net or len(groups_touched) >= 2 or fanout.get(net, 0) >= 2:
                critical_nets.append(net)

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
    for p in circuit.ports:
        if p.net in supplies or p.net in grounds:
            port_sides[p.name] = "local_repeat"
        elif p.direction == "output":
            port_sides[p.name] = "right"
        elif (p.net in diff_gate_nets or p.net in pure_gate_nets) and fanout.get(p.net, 0) <= 2:
            port_sides[p.name] = "near_pin"
        elif fanout.get(p.net, 0) >= 2 and p.direction == "input" and p.net in pure_gate_nets:
            port_sides[p.name] = "local_repeat"
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
        llm_notes="由确定性拓扑分析器生成的初始草案；大模型可在此基础上复核并微调分组顺序、器件朝向及跨级偏置网络走线策略。",
    )

    return TopologyAnalysis(
        circuit=circuit,
        motifs=motifs,
        net_fanout=fanout,
        net_classifications_default=net_policies,
        default_tiers=default_tiers,
        ambiguities_for_llm=ambiguities,
        draft_intent=draft_intent.to_dict(),
    )
