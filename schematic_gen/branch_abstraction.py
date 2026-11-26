"""Universal topology-driven vertical branch folding, bias-tree separation, and core anchor extraction.

Designed to turn flat, anonymous 40-160+ device analog netlists (MOS, BJT, R, C, diodes, sources —
covering OpAmps, Bandgap References, LDOs, IDACs, Comparators, etc.) into a compact ~10-30 vertical
branch skeleton BEFORE LLM semantic planning:
  1. Bias Distribution Tree Separation: distinguishes high-fanout diode/port bias rails from
     low-fanout local signal nets so global bias lines do not glue unrelated functional blocks together.
  2. Lossless Vertical Current Branch Folding (`BR01..BRk`): collapses co-gated `Super-MOS` stacks,
     pure degree-2 series chains (MOS/BJT/R), and unambiguous top-to-bottom active/passive stacks
     into top-to-bottom current branches (`VDD -> ... -> VSS`) while keeping floating bridges separate.
  3. Cross-Device-Type Quasi-Symmetric Pair & Repeated Array Detection: discovers symmetric/mirrored
     branch pairs (including asymmetric-in-detail cores like BGR `Q9/M32/Q15/R7 <-> Q10/M33/Q8/R8/R9`
     and multi-column coupled mirror amps) as well as regular `>= 3` column arrays (IDAC/CDAC).
  4. Signal-Only Inter-Branch Graph, Feedback Loop (SCC) Detection & Core Anchor Ranking.
"""

from __future__ import annotations

from collections import defaultdict, deque
import re
from typing import Any

from schematic_gen.models import CandidateMotif, InstanceIR, NetlistCircuit
from schematic_gen.topology_core import (
    THRESHOLDS,
    channel_pins_of,
    ctrl_pin_of,
    is_cap_device,
    is_diode_connected,
    is_passive_device,
    lcs_len as _lcs_len,
    nat_key as _nat_key,
    ref_num as _ref_num,
    top_bot_pins_of,
)


def _device_token(inst: InstanceIR, device_roles: dict[str, dict[str, Any]]) -> str:
    role = device_roles.get(inst.ref, {}).get("role", "")
    if role == "mos_cap":
        return "moscap"
    if role == "mos_active_res":
        return "mosres"
    if inst.device_class == "mos":
        return f"mos:{inst.polarity}"
    if inst.device_class == "bjt":
        return f"bjt:{inst.polarity}"
    if inst.device_class.startswith("res"):
        return "res"
    if inst.device_class.startswith("cap"):
        return "cap"
    return inst.device_class


def _compact_signature(refs: list[str], inst_by_ref: dict[str, InstanceIR], device_roles: dict[str, dict[str, Any]]) -> str:
    """Render a compact human/LLM signature like `PMOS×4-NMOS×4` or `PNP-NMOS-NPN-R`."""
    short_map = {
        "mos:pmos": "PMOS",
        "mos:nmos": "NMOS",
        "bjt:pnp": "PNP",
        "bjt:npn": "NPN",
        "moscap": "MOSCAP",
        "mosres": "MOSRES",
        "res": "R",
        "cap": "C",
        "diode": "D",
        "isource": "ISRC",
        "vsource": "VSRC",
        "subckt": "BLK",
    }
    toks = [short_map.get(_device_token(inst_by_ref[r], device_roles), inst_by_ref[r].device_class.upper()) for r in refs]
    runs: list[str] = []
    i = 0
    while i < len(toks):
        j = i + 1
        while j < len(toks) and toks[j] == toks[i]:
            j += 1
        cnt = j - i
        runs.append(f"{toks[i]}×{cnt}" if cnt > 1 else toks[i])
        i = j
    return "-".join(runs)


def _extract_size_highlights(refs: list[str], inst_by_ref: dict[str, InstanceIR]) -> list[str]:
    """Surface notable sizing clues (multipliers m>1, large W, R/C values) that help identify core vs aux."""
    notes: list[str] = []
    for r in refs:
        inst = inst_by_ref[r]
        m_val = inst.params.get("m", "")
        if m_val and m_val not in {"1", "1.0"}:
            notes.append(f"{r}:m={m_val}")
        if inst.device_class.startswith("cap") and inst.params.get("c"):
            notes.append(f"{r}={inst.params['c']}")
        elif inst.device_class in {"isource", "vsource"}:
            val = inst.params.get("dc") or inst.params.get("idc") or inst.params.get("vdc")
            if val:
                notes.append(f"{r}={val}")
    return notes[:5]


def _detect_bias_trees_pre(
    circuit: NetlistCircuit,
    device_roles: dict[str, dict[str, Any]],
    motifs: list[CandidateMotif],
    co_gated_stacks: list[dict[str, Any]],
    net_fanout: dict[str, int],
) -> dict[str, dict[str, Any]]:
    """Pre-classify bias distribution trees & reference buses prior to branch folding."""
    rails = set(circuit.supply_nets) | set(circuit.ground_nets)
    inst_by_ref = circuit.instance_map()
    port_by_net = {p.net: p for p in circuit.ports}
    stack_by_ref = {r: s for s in co_gated_stacks for r in s.get("refs", [])}

    diff_gate_nets: set[str] = set()
    for m in motifs:
        if m.motif_type == "diff_pair":
            for k in ("left", "right"):
                ref = m.roles.get(k)
                if ref in inst_by_ref:
                    cp = ctrl_pin_of(inst_by_ref[ref], device_roles)
                    if cp:
                        diff_gate_nets.add(inst_by_ref[ref].terminals.get(cp, ""))

    bias_map: dict[str, dict[str, Any]] = {}
    for net in circuit.nets:
        if net in rails or net in diff_gate_nets:
            continue
        diode_sources: list[str] = []
        non_diode_ctrl: list[str] = []
        other_ch_drivers: list[str] = []
        res_touch: list[str] = []

        for inst in circuit.instances:
            r = inst.ref
            cp = ctrl_pin_of(inst, device_roles)
            ch_pins = channel_pins_of(inst, device_roles)
            is_d = is_diode_connected(inst, rails, device_roles, stack_by_ref)

            touches_ctrl = bool(cp and inst.terminals.get(cp) == net)
            touches_ch = any(inst.terminals.get(p) == net for p in ch_pins)

            if inst.device_class in {"isource", "vsource"} and touches_ch:
                diode_sources.append(r)
            elif is_d and (touches_ctrl or touches_ch):
                # Diode-connected device whose gate/base equals net is a bias master
                if touches_ctrl:
                    diode_sources.append(r)
                else:
                    other_ch_drivers.append(r)
            else:
                if touches_ctrl:
                    non_diode_ctrl.append(r)
                if touches_ch:
                    other_ch_drivers.append(r)
                    if inst.device_class.startswith("res"):
                        res_touch.append(r)

        p = port_by_net.get(net)
        fo = net_fanout.get(net, len(diode_sources) + len(non_diode_ctrl) + len(other_ch_drivers))

        # 1. Diode/current-source driven bias mirror rail
        if diode_sources and len(non_diode_ctrl) >= 2:
            # Count distinct stacks/devices controlled
            ctrl_units = {
                stack_by_ref[r]["refs"][0] if r in stack_by_ref else r
                for r in non_diode_ctrl
            }
            bias_map[net] = {
                "net": net,
                "kind": "diode_bias_tree",
                "scope": "global" if len(ctrl_units) >= 3 else "local",
                "fanout": fo,
                "isPort": p is not None,
                "sourceDevices": sorted(set(diode_sources), key=_ref_num),
                "drivenDevices": sorted(set(non_diode_ctrl), key=_ref_num),
                "otherChannelDevices": sorted(set(other_ch_drivers), key=_ref_num),
            }
        # 2. Pure / predominantly control-pin top-level bias port
        elif p is not None and p.direction != "output" and len(non_diode_ctrl) >= 2 and len(other_ch_drivers) <= 1:
            ctrl_units = {
                stack_by_ref[r]["refs"][0] if r in stack_by_ref else r
                for r in non_diode_ctrl
            }
            bias_map[net] = {
                "net": net,
                "kind": "port_bias_bus",
                "scope": "global" if len(ctrl_units) >= 3 else "local",
                "fanout": fo,
                "isPort": True,
                "sourceDevices": [f"PORT:{p.name}"],
                "drivenDevices": sorted(set(non_diode_ctrl), key=_ref_num),
                "otherChannelDevices": sorted(set(other_ch_drivers), key=_ref_num),
            }
        # 3. Reference / feedback voltage bus (e.g. VREF feeding multiple bases/gates + divider/output)
        elif len(non_diode_ctrl) >= 2 and (p is not None or len(res_touch) >= 2) and fo >= 4:
            bias_map[net] = {
                "net": net,
                "kind": "reference_feedback_bus",
                "scope": "feedback_ref",
                "fanout": fo,
                "isPort": p is not None,
                "sourceDevices": sorted(set(other_ch_drivers), key=_ref_num),
                "drivenDevices": sorted(set(non_diode_ctrl), key=_ref_num),
                "otherChannelDevices": sorted(set(other_ch_drivers), key=_ref_num),
            }

    return bias_map


def build_branch_abstraction(
    circuit: NetlistCircuit,
    *,
    device_roles: dict[str, dict[str, Any]],
    motifs: list[CandidateMotif],
    co_gated_stacks: list[dict[str, Any]],
    net_fanout: dict[str, int],
) -> dict[str, Any]:
    """Build universal coarse-to-fine vertical branch abstraction for any analog circuit."""
    inst_by_ref = circuit.instance_map()
    supplies = set(circuit.supply_nets)
    grounds = set(circuit.ground_nets)
    rails = supplies | grounds
    port_by_net = {p.net: p for p in circuit.ports}
    stack_by_ref = {r: s for s in co_gated_stacks for r in s.get("refs", [])}

    # --- Step 1: Pre-identify bias trees ---
    bias_map = _detect_bias_trees_pre(circuit, device_roles, motifs, co_gated_stacks, net_fanout)

    # --- Step 2: Identify floating bridges, shunt caps, diff-pair tails, and subckts ---
    bridge_groups: list[tuple[str, list[str]]] = []
    bridge_refs: set[str] = set()

    for m in motifs:
        if m.motif_type in {"miller_bridge", "rc_series"}:
            members = [r for r in m.instances if r in inst_by_ref and r not in bridge_refs]
            if members:
                bridge_groups.append(("bridge_passive", members))
                bridge_refs.update(members)

    # Precompute channel incidence per net (excluding bulk & subckts & already-claimed bridges)
    raw_ch_inc: dict[str, list[tuple[str, str]]] = defaultdict(list)
    active_ch_inc: dict[str, list[tuple[str, str]]] = defaultdict(list)
    for inst in circuit.instances:
        if inst.ref in bridge_refs or inst.device_class == "subckt":
            continue
        for p in channel_pins_of(inst, device_roles):
            n = inst.terminals.get(p, "")
            if n and n not in rails:
                raw_ch_inc[n].append((inst.ref, p))
                if inst.device_class in {"mos", "bjt"} and not is_cap_device(inst, device_roles):
                    active_ch_inc[n].append((inst.ref, p))

    shunt_cap_refs: set[str] = set()
    for inst in circuit.instances:
        r = inst.ref
        if r in bridge_refs or inst.device_class == "subckt":
            continue
        if is_cap_device(inst, device_roles):
            ch_nets = [inst.terminals[p] for p in channel_pins_of(inst, device_roles) if inst.terminals.get(p)]
            non_rail = [n for n in ch_nets if n not in rails]
            if len(non_rail) == 1 and any(n in rails for n in ch_nets):
                # Shunt decoupling / load capacitor to VDD or VSS
                shunt_cap_refs.add(r)
            elif len(non_rail) == 2:
                # Floating inter-node capacitor (e.g. C1(C-E) or MOSCAP M21/M31)
                bridge_groups.append(("bridge_passive", [r]))
                bridge_refs.add(r)
        elif inst.device_class.startswith("res"):
            p_n = inst.terminals.get("P", "")
            m_n = inst.terminals.get("M", "")
            if p_n and m_n and p_n not in rails and m_n not in rails:
                # Cross-branch horizontal bridge resistor: BOTH ends touch >= 2 active MOS/BJT channels
                act_p = {ref for ref, _ in active_ch_inc.get(p_n, []) if ref != r}
                act_m = {ref for ref, _ in active_ch_inc.get(m_n, []) if ref != r}
                if len(act_p) >= 2 and len(act_m) >= 2:
                    bridge_groups.append(("bridge_passive", [r]))
                    bridge_refs.add(r)

    # Identify diff-pair tail nets so left/right legs never merge with tail or each other
    diff_tail_nets: set[str] = set()
    diff_leg_refs: set[str] = set()
    for m in motifs:
        if m.motif_type == "diff_pair":
            tn = m.roles.get("tail_net")
            if tn:
                diff_tail_nets.add(str(tn))
            for k in ("left_stack", "right_stack"):
                for rr in m.roles.get(k, []):
                    diff_leg_refs.add(str(rr))
            for k in ("left", "right"):
                if m.roles.get(k):
                    diff_leg_refs.add(str(m.roles[k]))

    # --- Step 3: Fold vertical current chains top-to-bottom ---
    # Start with pre-merged co-gated Super-MOS stacks; exclude bridges, shunt caps, and subckts
    standalone_refs = bridge_refs | shunt_cap_refs | {i.ref for i in circuit.instances if i.device_class == "subckt"}
    chains: list[list[str]] = []
    chain_of: dict[str, int] = {}

    for stk in co_gated_stacks:
        clean = [r for r in stk.get("refs", []) if r in inst_by_ref and r not in standalone_refs and r not in chain_of]
        if len(clean) >= 2:
            cid = len(chains)
            chains.append(list(clean))
            for r in clean:
                chain_of[r] = cid

    for inst in circuit.instances:
        if inst.ref not in standalone_refs and inst.ref not in chain_of:
            cid = len(chains)
            chains.append([inst.ref])
            chain_of[inst.ref] = cid

    # Helper: does a chain touch VDD at top or VSS at bottom?
    def _touches_supply(ref: str) -> bool:
        inst = inst_by_ref[ref]
        tp, _ = top_bot_pins_of(inst)
        if tp:
            return inst.terminals.get(tp, "") in supplies
        return any(inst.terminals.get(p, "") in supplies for p in channel_pins_of(inst, device_roles))

    def _touches_ground(ref: str) -> bool:
        inst = inst_by_ref[ref]
        _, bp = top_bot_pins_of(inst)
        if bp:
            return inst.terminals.get(bp, "") in grounds
        return any(inst.terminals.get(p, "") in grounds for p in channel_pins_of(inst, device_roles))

    # Build filtered channel incidence for foldable devices
    fold_ch_inc: dict[str, list[tuple[str, str]]] = defaultdict(list)
    for cid, refs in enumerate(chains):
        for r in refs:
            inst = inst_by_ref[r]
            for p in channel_pins_of(inst, device_roles):
                n = inst.terminals.get(p, "")
                if n and n not in rails:
                    fold_ch_inc[n].append((r, p))

    # Score candidate vertical edges `(score, upper_ref, lower_ref, net)`
    cand_edges: list[tuple[float, str, str, str]] = []
    for net, conns in fold_ch_inc.items():
        if net in diff_tail_nets:
            continue
        # Deduplicate per `(ref, pin)`
        uniq_conns = list(dict.fromkeys(conns))
        ch_deg = len({r for r, _ in uniq_conns})
        if ch_deg < 2 or ch_deg > THRESHOLDS.fold_max_channel_degree:
            continue

        for i_a, (ra, pa) in enumerate(uniq_conns):
            ia = inst_by_ref[ra]
            ta, ba = top_bot_pins_of(ia)
            for rb, pb in uniq_conns[i_a + 1 :]:
                if ra == rb or chain_of[ra] == chain_of[rb]:
                    continue
                ib = inst_by_ref[rb]
                tb, bb = top_bot_pins_of(ib)

                # Never merge two devices via same-side pin of same polarity (e.g. diff pair S-S or E-E)
                if ia.device_class == ib.device_class and ia.polarity == ib.polarity and pa == pb and ta is not None:
                    continue

                # Determine valid `(hi -> lo)` direction(s)
                pairs_to_eval: list[tuple[str, str, str, str]] = []
                a_as_hi = (ba == pa) or (ta is None and ba is None)
                b_as_lo = (tb == pb) or (tb is None and bb is None)
                b_as_hi = (bb == pb) or (tb is None and bb is None)
                a_as_lo = (ta == pa) or (ta is None and ba is None)

                if a_as_hi and b_as_lo:
                    pairs_to_eval.append((ra, pa, rb, pb))
                if b_as_hi and a_as_lo:
                    pairs_to_eval.append((rb, pb, ra, pa))

                # Resolve passive-passive ordering by rail proximity / ref number
                if ta is None and tb is None and len(pairs_to_eval) == 2:
                    a_gnd, b_gnd = _touches_ground(ra), _touches_ground(rb)
                    a_sup, b_sup = _touches_supply(ra), _touches_supply(rb)
                    if (a_sup and not b_sup) or (b_gnd and not a_gnd):
                        pairs_to_eval = [(ra, pa, rb, pb)]
                    elif (b_sup and not a_sup) or (a_gnd and not b_gnd):
                        pairs_to_eval = [(rb, pb, ra, pa)]
                    else:
                        pairs_to_eval = [(ra, pa, rb, pb)] if _ref_num(ra) <= _ref_num(rb) else [(rb, pb, ra, pa)]

                for hi, phi, lo, plo in pairs_to_eval:
                    ihi, ilo = inst_by_ref[hi], inst_by_ref[lo]
                    thi, bhi = top_bot_pins_of(ihi)
                    tlo, blo = top_bot_pins_of(ilo)
                    strict_active_pair = (bhi == phi and tlo == plo)

                    if ch_deg == 2:
                        # Pure series pass-through node!
                        score = 520.0 if strict_active_pair else 500.0
                        if ihi.device_class == ilo.device_class and is_passive_device(ihi):
                            score += 10.0
                    else:
                        # Tapped signal node (`ch_deg` in 3..5): only allow high-confidence vertical continuation
                        if strict_active_pair:
                            score = 260.0 - 20.0 * ch_deg
                            # Opposite-polarity active stack (PMOS/PNP over NMOS/NPN, e.g. Q9->M32, M4->M14)
                            if {ihi.polarity, ilo.polarity} in ({"pmos", "nmos"}, {"pnp", "nmos"}, {"pmos", "npn"}, {"pnp", "npn"}):
                                score += 40.0
                            # Diode master paired with its direct driver/load (e.g. M37->M31->M27)
                            if is_diode_connected(ihi, rails, device_roles, stack_by_ref) or is_diode_connected(
                                ilo, rails, device_roles, stack_by_ref
                            ):
                                score += 45.0
                            # Penalize pulling a floating clamp/follower (like Q11 with E=VREF) over a ground-bound leg
                            if (ilo.device_class == "bjt" and ilo.polarity == "npn" and ilo.terminals.get("E") in port_by_net) or (
                                ihi.device_class == "bjt" and ihi.polarity == "npn" and ihi.terminals.get("C") in supplies and _touches_ground(lo)
                            ):
                                score -= 70.0
                        elif is_passive_device(ihi) and is_passive_device(ilo) and ihi.device_class == ilo.device_class:
                            # Same-class resistive divider chain (e.g. R8 -> R9 -> VSS, R10 -> R11 -> VSS).
                            # On a private internal emitter degeneration node (e.g. net5, NOT a port/reference bus),
                            # let the direct active emitter->resistor leg (Q8.E->R8) beat passive->passive (R7->R8).
                            # On a top-level reference/divider port (e.g. VREF), keep the resistive divider (R10->R11) intact!
                            has_active_bot_here = (
                                net not in port_by_net
                                and net not in bias_map
                                and any(
                                    inst_by_ref[rc].device_class in {"mos", "bjt"}
                                    and top_bot_pins_of(inst_by_ref[rc])[1] == pc
                                    for rc, pc in uniq_conns
                                )
                            )
                            score = (165.0 if has_active_bot_here else 225.0) - 15.0 * ch_deg + (20.0 if _touches_ground(lo) else 0.0)
                        elif (bhi == phi and is_passive_device(ilo)) or (is_passive_device(ihi) and tlo == plo):
                            # Active-to-resistor emitter/source degeneration or load resistor (e.g. Q8.E -> R8)
                            if ch_deg > 4:
                                continue
                            score = 215.0 - 15.0 * ch_deg
                            # Do not let a rail-collector clamp/follower (C=VDD, e.g. Q13) steal a grounded divider resistor
                            # when another passive resistor continues down to VSS on the same node!
                            has_other_passive = any(
                                is_passive_device(inst_by_ref[rc]) and rc not in {hi, lo}
                                for rc, _ in uniq_conns
                            )
                            if (
                                ihi.device_class == "bjt"
                                and ihi.polarity == "npn"
                                and ihi.terminals.get("C") in supplies
                                and _touches_ground(lo)
                                and has_other_passive
                            ):
                                score -= 80.0
                        else:
                            continue

                    # Bonus when continuing an existing co-gated stack or nearby ref numbering
                    if hi in stack_by_ref or lo in stack_by_ref:
                        score += 18.0
                    # Small deterministic tie-breaker
                    cand_edges.append((score, hi, lo, net))

    cand_edges.sort(key=lambda item: (-item[0], _ref_num(item[1]), _ref_num(item[2]), item[3]))

    # Track internal junction nets between adjacent devices inside each chain to enforce pin exclusivity!
    def _shared_ch_nets(ref_a: str, ref_b: str) -> set[str]:
        ia, ib = inst_by_ref[ref_a], inst_by_ref[ref_b]
        na = {ia.terminals[p] for p in channel_pins_of(ia, device_roles) if ia.terminals.get(p) not in rails}
        nb = {ib.terminals[p] for p in channel_pins_of(ib, device_roles) if ib.terminals.get(p) not in rails}
        return na & nb

    for score, hi, lo, net_m in cand_edges:
        if score < 120.0:
            continue
        c_hi = chain_of[hi]
        c_lo = chain_of[lo]
        if c_hi == c_lo:
            continue
        list_hi = chains[c_hi]
        list_lo = chains[c_lo]
        if not list_hi or not list_lo:
            continue
        # Strict boundary chaining: `hi` must be bottom of upper chain, `lo` must be top of lower chain
        if list_hi[-1] != hi or list_lo[0] != lo:
            continue
        # Pin-exclusivity invariant: the connecting net `net_m` must NOT already be the internal junction
        # net tying `hi` to `list_hi[-2]` or tying `lo` to `list_lo[1]` (prevents mid-divider tap merges like Q11->R10@VREF)!
        if len(list_hi) >= 2 and net_m in _shared_ch_nets(list_hi[-2], hi):
            continue
        if len(list_lo) >= 2 and net_m in _shared_ch_nets(lo, list_lo[1]):
            continue
        # Rail monotonicity invariant: upper chain cannot already end at VSS; lower chain cannot start at VDD
        if _touches_ground(list_hi[-1]) or _touches_supply(list_lo[0]):
            continue
        # Prevent merging two independent supply-to-ground columns
        if any(_touches_ground(r) for r in list_hi) or any(_touches_supply(r) for r in list_lo):
            continue

        merged = list_hi + list_lo
        chains[c_hi] = merged
        chains[c_lo] = []
        for r in merged:
            chain_of[r] = c_hi

    # --- Step 4: Assemble complete ordered branch list (`BR01..BRk`) ---
    raw_branches: list[tuple[str, list[str]]] = []
    for refs in chains:
        if refs:
            raw_branches.append(("vertical", refs))
    for r in sorted(shunt_cap_refs, key=_ref_num):
        raw_branches.append(("shunt_passive", [r]))
    for bkind, refs in bridge_groups:
        raw_branches.append((bkind, refs))
    for inst in circuit.instances:
        if inst.device_class == "subckt":
            raw_branches.append(("subckt_block", [inst.ref]))

    # Deterministic ordering: bias/source roots first, then lowest ref number
    def _branch_sort_key(item: tuple[str, list[str]]) -> tuple[int, int, str]:
        bkind, refs = item
        has_src = any(inst_by_ref[r].device_class in {"isource", "vsource"} for r in refs)
        has_global_diode = any(
            r in bt["sourceDevices"]
            for bt in bias_map.values()
            if bt["scope"] == "global"
            for r in refs
        )
        prio = 0 if has_src else (1 if has_global_diode else (3 if bkind in {"shunt_passive", "bridge_passive"} else 2))
        return (prio, min(_ref_num(r) for r in refs), refs[0])

    raw_branches.sort(key=_branch_sort_key)

    branches: list[dict[str, Any]] = []
    branch_of_ref: dict[str, str] = {}
    for idx, (raw_kind, refs) in enumerate(raw_branches, start=1):
        bid = f"BR{idx:02d}"
        for r in refs:
            branch_of_ref[r] = bid

        top_r = refs[0]
        bot_r = refs[-1]
        top_rail = ""
        bot_rail = ""
        for p in channel_pins_of(inst_by_ref[top_r], device_roles):
            n = inst_by_ref[top_r].terminals.get(p, "")
            if n in supplies:
                top_rail = n
                break
        for p in channel_pins_of(inst_by_ref[bot_r], device_roles):
            n = inst_by_ref[bot_r].terminals.get(p, "")
            if n in grounds:
                bot_rail = n
                break

        ref_set = set(refs)
        # Classify internal-only series nets vs external signal taps vs control inputs
        ctrl_inputs: list[dict[str, Any]] = []
        signal_taps: list[dict[str, Any]] = []
        internal_series_nets: set[str] = set()

        # Precompute external device touches per net
        branch_local_nets: set[str] = set()
        for r in refs:
            inst = inst_by_ref[r]
            for p, n in inst.terminals.items():
                if p != "B" and n not in rails:
                    branch_local_nets.add(n)

        for n in sorted(branch_local_nets):
            ext_devs = [
                inst.ref
                for inst in circuit.instances
                if inst.ref not in ref_set
                and any(p != "B" and tn == n for p, tn in inst.terminals.items())
            ]
            if not ext_devs and n not in port_by_net:
                internal_series_nets.add(n)

        # Classify each adjacent joint as HARD (private pure-series node or same co-gated Super-MOS stack:
        # tearing it would split one physical current path) or SOFT (high-confidence continuation through a
        # tapped node shared with other devices: a module boundary there is allowed but flagged).
        joint_kinds: list[str] = []
        for r_a, r_b in zip(refs, refs[1:]):
            ia, ib = inst_by_ref[r_a], inst_by_ref[r_b]
            na = {ia.terminals.get(p, "") for p in channel_pins_of(ia, device_roles)} - rails - {""}
            nb = {ib.terminals.get(p, "") for p in channel_pins_of(ib, device_roles)} - rails - {""}
            shared_j = na & nb
            same_stack = (
                r_a in stack_by_ref
                and r_b in stack_by_ref
                and stack_by_ref[r_a].get("refs") == stack_by_ref[r_b].get("refs")
            )
            joint_kinds.append("hard" if same_stack or (shared_j & internal_series_nets) else "soft")
        hard_segments: list[list[str]] = [[refs[0]]]
        for r_next, jk in zip(refs[1:], joint_kinds):
            if jk == "hard":
                hard_segments[-1].append(r_next)
            else:
                hard_segments.append([r_next])

        seen_ctrl: set[tuple[str, str]] = set()
        seen_taps: set[tuple[str, str]] = set()
        for r in refs:
            inst = inst_by_ref[r]
            cp = ctrl_pin_of(inst, device_roles)
            is_d = is_diode_connected(inst, rails, device_roles, stack_by_ref)
            if cp:
                gn = inst.terminals.get(cp, "")
                if gn and gn not in rails and gn not in internal_series_nets:
                    key_c = (gn, "diode" if is_d else "ctrl")
                    if key_c not in seen_ctrl:
                        seen_ctrl.add(key_c)
                        bt = bias_map.get(gn)
                        ctrl_inputs.append(
                            {
                                "ref": r,
                                "pin": cp,
                                "net": gn,
                                "isDiodeSelf": is_d,
                                "biasScope": bt["scope"] if bt else "signal",
                            }
                        )
            for p in channel_pins_of(inst, device_roles):
                tn = inst.terminals.get(p, "")
                if tn and tn not in rails and tn not in internal_series_nets:
                    if (r, tn) not in seen_taps:
                        seen_taps.add((r, tn))
                        bt = bias_map.get(tn)
                        signal_taps.append(
                            {
                                "ref": r,
                                "pin": p,
                                "net": tn,
                                "isPort": tn in port_by_net,
                                "biasScope": bt["scope"] if bt else "signal",
                            }
                        )

        # Determine semantic branch kind
        if raw_kind in {"shunt_passive", "bridge_passive", "subckt_block"}:
            bkind = raw_kind
        elif any(inst_by_ref[r].device_class in {"isource", "vsource"} for r in refs) or any(
            r in bt["sourceDevices"] for bt in bias_map.values() if bt["scope"] == "global" for r in refs
        ):
            bkind = "bias_diode_column"
        elif any(r in diff_leg_refs for r in refs):
            bkind = "diff_leg"
        elif all(
            (c["biasScope"] == "global" or c["isDiodeSelf"]) for c in ctrl_inputs
        ) and ctrl_inputs and not any(t["biasScope"] == "signal" and t["isPort"] for t in signal_taps):
            # Controlled solely by global bias rails (still may feed local signal tap!)
            non_bias_sig_taps = [t for t in signal_taps if t["biasScope"] != "global"]
            bkind = "bias_mirror_column" if len(non_bias_sig_taps) <= 1 else "active_branch"
        elif len(refs) == 1 and device_roles.get(refs[0], {}).get("role") in {"source_follower", "bjt_follower", "bjt_diode"}:
            bkind = "clamp_or_follower"
        else:
            bkind = "active_branch"

        tokens = [_device_token(inst_by_ref[r], device_roles) for r in refs]
        branches.append(
            {
                "id": bid,
                "instances": refs,
                "size": len(refs),
                "kind": bkind,
                "topRail": top_rail,
                "bottomRail": bot_rail,
                "signature": _compact_signature(refs, inst_by_ref, device_roles),
                "tokens": tokens,
                "ctrlInputs": ctrl_inputs,
                "signalTaps": signal_taps,
                "internalSeriesNets": sorted(internal_series_nets),
                "jointKinds": joint_kinds,
                "hardSegments": hard_segments,
                "sizeHighlights": _extract_size_highlights(refs, inst_by_ref),
                "hasDiode": any(is_diode_connected(inst_by_ref[r], rails, device_roles, stack_by_ref) for r in refs),
            }
        )

    branch_by_id = {b["id"]: b for b in branches}

    # Backfill `sourceBranches` and `drivenBranches` on bias_trees and refine `scope` using branch count!
    global_bias_nets: set[str] = set()
    bias_trees: list[dict[str, Any]] = []
    for net, bt in sorted(bias_map.items(), key=lambda kv: (-kv[1]["fanout"], kv[0])):
        src_brs = sorted({branch_of_ref[r] for r in bt["sourceDevices"] if r in branch_of_ref})
        drv_brs = sorted(
            {branch_of_ref[r] for r in bt["drivenDevices"] if r in branch_of_ref} - set(src_brs)
        )
        # Re-evaluate global vs local strictly by distinct driven branches (`>= 3` branches -> global bias rail)
        if bt["kind"] in {"diode_bias_tree", "port_bias_bus"}:
            bt["scope"] = "global" if len(drv_brs) >= THRESHOLDS.global_bias_min_branches else "local"
        if bt["scope"] == "global":
            global_bias_nets.add(net)
        bt_out = dict(bt)
        bt_out["sourceBranches"] = src_brs
        bt_out["drivenBranches"] = drv_brs
        bias_trees.append(bt_out)

    # Refresh `biasScope` labels inside branch `ctrlInputs` / `signalTaps`
    scope_by_net = {bt["net"]: bt["scope"] for bt in bias_trees}
    for b in branches:
        for c in b["ctrlInputs"]:
            c["biasScope"] = scope_by_net.get(c["net"], "signal")
        for t in b["signalTaps"]:
            t["biasScope"] = scope_by_net.get(t["net"], "signal")

    # --- Step 5: Signal-Only Inter-Branch Graph (stripping VDD/VSS AND global bias rails) ---
    sig_edges: list[dict[str, Any]] = []
    seen_se: set[tuple[str, str, str, str]] = set()
    branch_sig_adj: dict[str, set[str]] = defaultdict(set)
    branch_sig_nets: dict[str, set[str]] = defaultdict(set)
    dir_adj: dict[str, set[str]] = defaultdict(set)

    for net in circuit.nets:
        if net in rails or net in global_bias_nets:
            continue
        drivers: set[str] = set()
        ctrl_sinks: set[str] = set()
        for b in branches:
            bid = b["id"]
            has_ch = any(t["net"] == net for t in b["signalTaps"])
            has_diode_c = any(c["net"] == net and c["isDiodeSelf"] for c in b["ctrlInputs"])
            has_ctrl = any(c["net"] == net and not c["isDiodeSelf"] for c in b["ctrlInputs"])
            if has_ch or has_diode_c:
                drivers.add(bid)
            if has_ctrl:
                ctrl_sinks.add(bid)

        all_touch = drivers | ctrl_sinks
        if len(all_touch) < 2:
            continue
        for bid in all_touch:
            branch_sig_nets[bid].add(net)

        if drivers and ctrl_sinks:
            for src in sorted(drivers):
                for dst in sorted(ctrl_sinks):
                    if src == dst:
                        continue
                    key = (src, dst, net, "sig_ctrl")
                    if key not in seen_se:
                        seen_se.add(key)
                        sig_edges.append(
                            {
                                "src": src,
                                "dst": dst,
                                "net": net,
                                "edgeKind": "sig_ctrl",
                                "fanout": net_fanout.get(net, len(all_touch)),
                                "isPort": net in port_by_net,
                            }
                        )
                        branch_sig_adj[src].add(dst)
                        branch_sig_adj[dst].add(src)
                        dir_adj[src].add(dst)
        if len(drivers) >= 2:
            d_list = sorted(drivers)
            for i_d in range(len(d_list)):
                for j_d in range(i_d + 1, len(d_list)):
                    u, v = d_list[i_d], d_list[j_d]
                    key = (u, v, net, "sig_channel")
                    if key not in seen_se:
                        seen_se.add(key)
                        sig_edges.append(
                            {
                                "src": u,
                                "dst": v,
                                "net": net,
                                "edgeKind": "sig_channel",
                                "fanout": net_fanout.get(net, len(all_touch)),
                                "isPort": net in port_by_net,
                            }
                        )
                        branch_sig_adj[u].add(v)
                        branch_sig_adj[v].add(u)
                        # Low-fanout channel coupling participates bidirectionally in loop analysis
                        if net_fanout.get(net, 99) <= 5:
                            dir_adj[u].add(v)
                            dir_adj[v].add(u)

    # --- Step 6: Feedback Loop Detection (Tarjan SCC on signal-only branch graph) ---
    # Use directed `sig_ctrl` edges + tight private `sig_channel` edges, and require >= 2 distinct
    # signal/control nets inside the SCC so single-net diode mirrors or summing-node cliques are NOT false loops!
    # Also exclude hyperedge summing buses touching >= 6 branches from forming artificial cliques.
    bus_branch_counts: dict[str, set[str]] = defaultdict(set)
    for e in sig_edges:
        bus_branch_counts[e["net"]].add(e["src"])
        bus_branch_counts[e["net"]].add(e["dst"])
    shared_signal_buses: list[dict[str, Any]] = []
    hyper_bus_nets: set[str] = set()
    for net, bset in sorted(bus_branch_counts.items()):
        if len(bset) >= THRESHOLDS.hyper_bus_min_branches:
            hyper_bus_nets.add(net)
            drvs = sorted({e["src"] for e in sig_edges if e["net"] == net and e["edgeKind"] == "sig_ctrl"} | {
                b["id"] for b in branches if any(t["net"] == net for t in b["signalTaps"])
            })
            ctrls = sorted({e["dst"] for e in sig_edges if e["net"] == net and e["edgeKind"] == "sig_ctrl"})
            shared_signal_buses.append(
                {
                    "net": net,
                    "branchCount": len(bset),
                    "fanout": net_fanout.get(net, len(bset)),
                    "driverBranches": drvs,
                    "ctrlBranches": ctrls,
                    "allBranches": sorted(bset),
                }
            )

    loop_adj: dict[str, set[str]] = defaultdict(set)
    for e in sig_edges:
        if e["net"] in hyper_bus_nets and e["edgeKind"] == "sig_channel":
            continue
        if e["edgeKind"] == "sig_ctrl":
            loop_adj[e["src"]].add(e["dst"])
        elif e["edgeKind"] == "sig_channel" and e["fanout"] <= 4:
            if branch_by_id[e["src"]]["kind"] != "shunt_passive" and branch_by_id[e["dst"]]["kind"] != "shunt_passive":
                loop_adj[e["src"]].add(e["dst"])
                loop_adj[e["dst"]].add(e["src"])

    idx_counter = [0]
    indices: dict[str, int] = {}
    lowlink: dict[str, int] = {}
    stack_t: list[str] = []
    on_stack: set[str] = set()
    raw_sccs: list[list[str]] = []

    def _strongconnect(v: str) -> None:
        indices[v] = lowlink[v] = idx_counter[0]
        idx_counter[0] += 1
        stack_t.append(v)
        on_stack.add(v)
        for w in sorted(loop_adj.get(v, ())):
            if w not in indices:
                _strongconnect(w)
                lowlink[v] = min(lowlink[v], lowlink[w])
            elif w in on_stack:
                lowlink[v] = min(lowlink[v], indices[w])
        if lowlink[v] == indices[v]:
            comp: list[str] = []
            while True:
                w = stack_t.pop()
                on_stack.remove(w)
                comp.append(w)
                if w == v:
                    break
            if len(comp) >= 2:
                raw_sccs.append(sorted(comp))

    for b in branches:
        if b["id"] not in indices:
            _strongconnect(b["id"])

    feedback_loops: list[dict[str, Any]] = []
    for li, comp in enumerate(raw_sccs, start=1):
        comp_set = set(comp)
        ctrl_nets_in = sorted(
            {
                e["net"]
                for e in sig_edges
                if e["edgeKind"] == "sig_ctrl" and e["src"] in comp_set and e["dst"] in comp_set
            }
        )
        all_loop_nets = sorted(
            {
                e["net"]
                for e in sig_edges
                if e["src"] in comp_set and e["dst"] in comp_set
            }
        )
        # Require >= 2 distinct control nets (real multi-node feedback loop, e.g. BGR/LDO,
        # NOT a single-net local mirror pair or single summing node clique!).
        if len(ctrl_nets_in) < 2:
            continue
        feedback_loops.append(
            {
                "id": f"LOOP{len(feedback_loops) + 1}",
                "branches": comp,
                "instances": [r for bid in comp for r in branch_by_id[bid]["instances"]],
                "ctrlNets": ctrl_nets_in,
                "signalNets": all_loop_nets,
            }
        )

    # --- Step 7: Repeated Regular Branch Arrays (>= 3 matched columns, e.g. IDAC/CDAC) ---
    # Detect regular arrays FIRST so identical array columns are surfaced once as an array
    # rather than fragmented into redundant pairwise quasi-symmetric pairs!
    by_sig_and_bus: dict[tuple[str, str], list[str]] = defaultdict(list)
    for b in branches:
        if b["kind"] in {"bias_diode_column", "subckt_block"}:
            continue
        bus_nets = sorted({c["net"] for c in b["ctrlInputs"]} | {t["net"] for t in b["signalTaps"] if t["biasScope"] == "global"})
        if not bus_nets:
            bus_nets = sorted({t["net"] for t in b["signalTaps"]})
        for bn in bus_nets:
            by_sig_and_bus[(b["signature"], bn)].append(b["id"])

    repeated_arrays: list[dict[str, Any]] = []
    seen_arr_sets: list[set[str]] = []
    array_member_branches: set[str] = set()
    for (sig, bn), bids in sorted(by_sig_and_bus.items(), key=lambda kv: (-len(kv[1]), kv[0])):
        uniq_b = sorted(set(bids))
        if len(uniq_b) < 3:
            continue
        bset = set(uniq_b)
        if any(bset <= prev for prev in seen_arr_sets):
            continue
        seen_arr_sets.append(bset)
        array_member_branches.update(bset)
        repeated_arrays.append(
            {
                "id": f"ARR_{len(repeated_arrays) + 1}",
                "signature": sig,
                "commonBusNet": bn,
                "branches": uniq_b,
                "columnCount": len(uniq_b),
                "instances": [r for bid in uniq_b for r in branch_by_id[bid]["instances"]],
            }
        )

    # --- Step 8: Cross-Device-Type Quasi-Symmetric Pairs & Multi-Column Coupled Mirrors ---
    quasi_pairs: list[dict[str, Any]] = []
    # First check 2-branch coupled pairs (e.g. [Q3-R3-M21-M22 -> M25-Q4-R4] <-> [Q5-R5-M23-M24 -> M26-Q6-R6])
    private_ctrl_pairs: list[tuple[str, str, str]] = []
    for e in sig_edges:
        if e["edgeKind"] == "sig_ctrl" and e["fanout"] <= 3:
            b_src = branch_by_id[e["src"]]
            b_dst = branch_by_id[e["dst"]]
            if b_src["size"] >= 2 and b_dst["size"] >= 2:
                private_ctrl_pairs.append((e["src"], e["dst"], e["net"]))

    claimed_coupled: set[str] = set()
    for i_p, (s1, d1, n1) in enumerate(private_ctrl_pairs):
        if s1 in claimed_coupled or d1 in claimed_coupled:
            continue
        for s2, d2, n2 in private_ctrl_pairs[i_p + 1 :]:
            if len({s1, d1, s2, d2}) < 4 or s2 in claimed_coupled or d2 in claimed_coupled:
                continue
            sim_s = 2.0 * _lcs_len(branch_by_id[s1]["tokens"], branch_by_id[s2]["tokens"]) / max(
                1, len(branch_by_id[s1]["tokens"]) + len(branch_by_id[s2]["tokens"])
            )
            sim_d = 2.0 * _lcs_len(branch_by_id[d1]["tokens"], branch_by_id[d2]["tokens"]) / max(
                1, len(branch_by_id[d1]["tokens"]) + len(branch_by_id[d2]["tokens"])
            )
            if sim_s >= THRESHOLDS.coupled_mirror_min_similarity and sim_d >= THRESHOLDS.coupled_mirror_min_similarity:
                claimed_coupled.update({s1, d1, s2, d2})
                quasi_pairs.append(
                    {
                        "id": f"QSYM_COUPLED_{len(quasi_pairs) + 1}",
                        "kind": "coupled_mirror_pair",
                        "leftBranches": [s1, d1],
                        "rightBranches": [s2, d2],
                        "leftInstances": branch_by_id[s1]["instances"] + branch_by_id[d1]["instances"],
                        "rightInstances": branch_by_id[s2]["instances"] + branch_by_id[d2]["instances"],
                        "similarity": round(0.5 * (sim_s + sim_d), 3),
                        "sharedSignals": sorted(
                            (branch_sig_nets[s1] | branch_sig_nets[d1]) & (branch_sig_nets[s2] | branch_sig_nets[d2])
                        ),
                        "reason": (
                            f"耦合双列镜像对：[{s1}({branch_by_id[s1]['signature']})→{d1}({branch_by_id[d1]['signature']})] "
                            f"↔ [{s2}({branch_by_id[s2]['signature']})→{d2}({branch_by_id[d2]['signature']})]"
                        ),
                    }
                )
                break

    # Next check single-branch quasi-symmetric pairs (e.g. BGR core Q9-M32-Q15-R7 <-> Q10-M33-Q8-R8-R9)
    cand_single_pairs: list[tuple[float, str, str, list[str], list[str]]] = []
    for i_a, ba in enumerate(branches):
        if ba["id"] in claimed_coupled or ba["id"] in array_member_branches or ba["kind"] in {"shunt_passive", "bridge_passive", "subckt_block"}:
            continue
        if ba["size"] < 2 and ba["kind"] != "diff_leg":
            continue
        ctrl_a_local = {c["net"] for c in ba["ctrlInputs"] if c["biasScope"] != "global"}
        ctrl_a_all = {c["net"] for c in ba["ctrlInputs"]}
        sig_a = branch_sig_nets[ba["id"]] - hyper_bus_nets
        for bb in branches[i_a + 1 :]:
            if bb["id"] in claimed_coupled or bb["id"] in array_member_branches or bb["kind"] in {"shunt_passive", "bridge_passive", "subckt_block"}:
                continue
            if bb["size"] < 2 and bb["kind"] != "diff_leg":
                continue
            sim = 2.0 * _lcs_len(ba["tokens"], bb["tokens"]) / max(1, len(ba["tokens"]) + len(bb["tokens"]))
            if sim < THRESHOLDS.quasi_symmetry_min_similarity:
                continue
            ctrl_b_local = {c["net"] for c in bb["ctrlInputs"] if c["biasScope"] != "global"}
            ctrl_b_all = {c["net"] for c in bb["ctrlInputs"]}
            shared_local_ctrl = sorted(ctrl_a_local & ctrl_b_local)
            shared_sig = sorted(sig_a & (branch_sig_nets[bb["id"]] - hyper_bus_nets))
            shared_global_ctrl = sorted((ctrl_a_all & ctrl_b_all) & global_bias_nets)

            if shared_local_ctrl or shared_sig:
                score_p = sim * 100.0 + 30.0 * len(shared_local_ctrl) + 15.0 * len(shared_sig) + 3.0 * min(ba["size"], bb["size"])
                cand_single_pairs.append((score_p, ba["id"], bb["id"], shared_local_ctrl, shared_sig))
            elif shared_global_ctrl and sim == 1.0 and ba["size"] >= 3 and (ba["hasDiode"] or bb["hasDiode"]):
                score_p = 75.0 + 4.0 * ba["size"]
                cand_single_pairs.append((score_p, ba["id"], bb["id"], [], []))

    cand_single_pairs.sort(key=lambda t: (-t[0], t[1], t[2]))
    claimed_single: set[str] = set(claimed_coupled)
    for _sc, bid_a, bid_b, sh_ctrl, sh_sig in cand_single_pairs:
        if bid_a in claimed_single or bid_b in claimed_single:
            continue
        ba, bb = branch_by_id[bid_a], branch_by_id[bid_b]
        sim = 2.0 * _lcs_len(ba["tokens"], bb["tokens"]) / max(1, len(ba["tokens"]) + len(bb["tokens"]))
        is_bias_pair = not sh_ctrl and not sh_sig
        claimed_single.update({bid_a, bid_b})
        quasi_pairs.append(
            {
                "id": f"QSYM_{len(quasi_pairs) + 1}",
                "kind": "bias_mirror_pair" if is_bias_pair else "quasi_symmetric_core",
                "leftBranches": [bid_a],
                "rightBranches": [bid_b],
                "leftInstances": list(ba["instances"]),
                "rightInstances": list(bb["instances"]),
                "similarity": round(sim, 3),
                "sharedCtrlNets": sh_ctrl,
                "sharedSignals": sorted(set(sh_ctrl) | set(sh_sig)),
                "reason": (
                    f"{'偏置镜主从配对' if is_bias_pair else '准对称功能核配对'}："
                    f"{bid_a}({ba['signature']}) ↔ {bid_b}({bb['signature']})"
                    + (f"，共控网 `{', '.join(sh_ctrl)}`" if sh_ctrl else "")
                    + (f"，共用信号 `{', '.join(sh_sig)}`" if sh_sig and not sh_ctrl else "")
                ),
            }
        )

    # --- Step 9: Core Anchor Candidates (Ranked High-Level Anchors for Coarse-to-Fine Planning) ---
    core_anchors: list[dict[str, Any]] = []
    for m in motifs:
        if m.motif_type in {"sandwich_mirror_core", "diff_pair"}:
            brs = sorted({branch_of_ref[r] for r in m.instances if r in branch_of_ref})
            core_anchors.append(
                {
                    "id": f"ANCHOR_{m.id}",
                    "kind": m.motif_type,
                    "priority": 1 if m.motif_type == "sandwich_mirror_core" else 2,
                    "branches": brs,
                    "instances": list(m.instances),
                    "reason": m.notes,
                }
            )
    for qp in quasi_pairs:
        if qp["kind"] in {"coupled_mirror_pair", "quasi_symmetric_core"}:
            brs = qp["leftBranches"] + qp["rightBranches"]
            core_anchors.append(
                {
                    "id": f"ANCHOR_{qp['id']}",
                    "kind": qp["kind"],
                    "priority": 1 if qp["kind"] == "coupled_mirror_pair" else 2,
                    "branches": brs,
                    "instances": qp["leftInstances"] + qp["rightInstances"],
                    "reason": qp["reason"],
                }
            )
    for arr in repeated_arrays:
        core_anchors.append(
            {
                "id": f"ANCHOR_{arr['id']}",
                "kind": "repeated_branch_array",
                "priority": 2,
                "branches": list(arr["branches"]),
                "instances": list(arr["instances"]),
                "reason": f"{arr['columnCount']} 列规整阵列 ({arr['signature']}) 共用母线 `{arr['commonBusNet']}`",
            }
        )
    for fl in feedback_loops:
        core_anchors.append(
            {
                "id": f"ANCHOR_{fl['id']}",
                "kind": "feedback_loop",
                "priority": 3,
                "branches": list(fl["branches"]),
                "instances": list(fl["instances"]),
                "reason": f"闭环反馈核心（{len(fl['branches'])} 支路，经由控制网 `{', '.join(fl['ctrlNets'][:5])}`）",
            }
        )
    bias_root_brs = [b["id"] for b in branches if b["kind"] == "bias_diode_column"]
    if bias_root_brs:
        core_anchors.append(
            {
                "id": "ANCHOR_BIAS_ROOT",
                "kind": "bias_root",
                "priority": 4,
                "branches": bias_root_brs,
                "instances": [r for bid in bias_root_brs for r in branch_by_id[bid]["instances"]],
                "reason": f"全局偏置源/二极管参考主干（驱动偏置树 `{', '.join(sorted(global_bias_nets)) or '局部偏置'}`）",
            }
        )

    core_anchors.sort(key=lambda a: (a["priority"], -len(a["instances"]), a["id"]))

    hidden_series_nets = sorted({n for b in branches for n in b["internalSeriesNets"]})

    return {
        "schema": "schem_gen.branch_abstraction.v1",
        "deviceCount": len(circuit.instances),
        "branchCount": len(branches),
        "hiddenSeriesNetCount": len(hidden_series_nets),
        "globalBiasNets": sorted(global_bias_nets),
        "biasTrees": bias_trees,
        "branches": branches,
        "branchOfRef": branch_of_ref,
        "signalEdges": sig_edges,
        "sharedSignalBuses": shared_signal_buses,
        "feedbackLoops": feedback_loops,
        "quasiSymmetricPairs": quasi_pairs,
        "repeatedArrays": repeated_arrays,
        "coreAnchors": core_anchors,
    }
