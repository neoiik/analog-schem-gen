"""Pass 0-3 deterministic analog circuit semantic & signal-flow backbone analyzer.

Provides four pure, topology-driven analysis passes (zero reliance on device naming):
  - Pass 0: Device functional role classification (`mos_cap`, `mos_active_res`,
            `mos_diode`, `source_follower`, `current_source`, `composite_stack_member`)
            and canonical composite MOS reduction (`Super-MOS` for split-length /
            co-gated series stacks) + composite differential pair detection.
  - Pass 1: Generalized Miller compensation bridge (`miller_bridge`) and complementary
            source-follower output buffer (`complementary_sf`) detection.
  - Pass 2: Fault-tolerant quasi-mirror subcircuit & sandwich symmetric core detection
            (`detect_quasi_mirror_and_sandwich_cores`), tolerating single-ended output
            taps and using passive bridge elements / diff-pair outputs as symmetry seeds.
  - Pass 3: Directed inter-block signal-flow backbone DAG & bottleneck cut-net macro
            stage partitioning (`build_signal_flow_backbone`).
"""

from __future__ import annotations

from collections import defaultdict, deque
from dataclasses import dataclass, field
import re
from typing import Any

from schematic_gen.models import CandidateMotif, InstanceIR, NetlistCircuit


@dataclass
class ReducedMosUnit:
    """Canonical MOS or co-gated series-stacked composite MOS (`Super-MOS`)."""

    unit_id: str
    refs: list[str]  # ordered top-to-bottom (PMOS: S->D; NMOS: D->S)
    polarity: str  # "pmos" | "nmos"
    kind: str | None
    gate: str
    eff_d: str
    eff_s: str
    is_diode: bool
    is_rail_source: bool
    wl_sig: tuple[Any, ...]
    anchor_rail: str = ""


def _unique_non_bulk_devices_by_net(circuit: NetlistCircuit) -> dict[str, list[tuple[str, str]]]:
    """Map each net to `(ref, pin)` occurrences ignoring bulk `B`."""
    out: dict[str, list[tuple[str, str]]] = defaultdict(list)
    for inst in circuit.instances:
        for pin, net in inst.terminals.items():
            if pin != "B":
                out[net].append((inst.ref, pin))
    return out


def _unique_device_refs_on_net(net_terms: dict[str, list[tuple[str, str]]], net: str) -> set[str]:
    return {ref for ref, _ in net_terms.get(net, [])}


def _ref_num(ref: str) -> int:
    m = re.search(r"(\d+)$", ref)
    return int(m.group(1)) if m else 10**9


def classify_device_roles(
    circuit: NetlistCircuit,
    co_gated_stacks: list[dict[str, Any]] | None = None,
) -> dict[str, dict[str, Any]]:
    """Pass 0a: Classify each device's analog functional role purely from port topology."""
    supplies = set(circuit.supply_nets)
    grounds = set(circuit.ground_nets)
    rails = supplies | grounds
    port_nets = {p.net for p in circuit.ports}
    input_port_nets = {p.net for p in circuit.ports if p.direction == "input" and p.net not in rails}
    inst_by_ref = circuit.instance_map()
    net_terms = _unique_non_bulk_devices_by_net(circuit)

    stack_by_ref: dict[str, dict[str, Any]] = {}
    for stk in co_gated_stacks or []:
        for r in stk.get("refs", []):
            stack_by_ref[r] = stk

    roles: dict[str, dict[str, Any]] = {}
    mos_cap_refs: set[str] = set()
    mos_diode_refs: set[str] = set()
    sf_refs: set[str] = set()

    # First pass: unambiguous structural roles (MOSCAP, diode, source follower, BJT diode/follower)
    for inst in circuit.instances:
        r = inst.ref
        if inst.device_class == "bjt":
            c = inst.terminals.get("C", "")
            b = inst.terminals.get("BASE", "")
            e = inst.terminals.get("E", "")
            if c and c == b and c not in rails and c != e:
                roles[r] = {
                    "role": "bjt_diode",
                    "polarity": inst.polarity,
                    "diodeNet": c,
                    "emitterNet": e,
                    "summary": f"C=B={c}, E={e}",
                }
            elif (
                (inst.polarity == "npn" and c in supplies and e not in rails and b != c and b != e)
                or (inst.polarity == "pnp" and c in grounds and e not in rails and b != c and b != e)
            ):
                roles[r] = {
                    "role": "bjt_follower",
                    "polarity": inst.polarity,
                    "baseNet": b,
                    "emitterNet": e,
                    "collectorRail": c,
                    "summary": f"C={c}, B={b}, E={e}",
                }
            else:
                roles[r] = {
                    "role": "bjt",
                    "polarity": inst.polarity,
                    "deviceClass": "bjt",
                    "summary": f"C={c}, B={b}, E={e}",
                }
            continue
        if inst.device_class != "mos":
            roles[r] = {
                "role": inst.device_class,
                "deviceClass": inst.device_class,
                "summary": "/".join(
                    f"{p}={n}" for p, n in inst.terminals.items() if p != "B"
                ),
            }
            continue

        d = inst.terminals.get("D", "")
        g = inst.terminals.get("G", "")
        s = inst.terminals.get("S", "")

        if d and d == s and g and g != d and not (d in rails and g in rails):
            mos_cap_refs.add(r)
            roles[r] = {
                "role": "mos_cap",
                "polarity": inst.polarity,
                "gateNet": g,
                "bodyNode": d,
                "summary": f"G={g} | DS={d}",
            }
        elif d and d == g and d not in rails and d != s:
            mos_diode_refs.add(r)
            roles[r] = {
                "role": "mos_diode",
                "polarity": inst.polarity,
                "diodeNet": d,
                "sourceNet": s,
                "summary": f"D=G={d}, S={s}",
            }
        elif (
            (inst.polarity == "nmos" and d in supplies and s not in rails and g != d and g != s)
            or (inst.polarity == "pmos" and d in grounds and s not in rails and g != d and g != s)
        ):
            sf_refs.add(r)
            roles[r] = {
                "role": "source_follower",
                "polarity": inst.polarity,
                "gateNet": g,
                "sourceNet": s,
                "drainRail": d,
                "summary": f"D={d}, G={g}, S={s}",
            }

    # Second pass: floating series active MOS resistors / pass devices chained from a DC-blocked MOSCAP plate
    # Key physical criterion: a MOSCAP DS node that connects EXCLUSIVELY to `{mos_cap, next_dev}` (no DC rail path)
    # cannot be a normal cascode column node; floating MOS devices chained from it until reaching a multi-device DC node
    # are active triode resistors (e.g. Miller zero-nulling MOS resistors).
    mos_active_res_refs: set[str] = set()
    frontier_nets: deque[tuple[str, int]] = deque()
    seen_chain_nets: set[str] = set()

    for cap_ref in sorted(mos_cap_refs):
        ds_net = inst_by_ref[cap_ref].terminals.get("D", "")
        if not ds_net or ds_net in rails:
            continue
        devs_on_ds = _unique_device_refs_on_net(net_terms, ds_net)
        if len(devs_on_ds) == 2:
            frontier_nets.append((ds_net, 0))
            seen_chain_nets.add(ds_net)

    while frontier_nets:
        cur_net, hops = frontier_nets.popleft()
        if hops >= 4:
            continue
        for ref, pin in net_terms.get(cur_net, []):
            if ref in mos_cap_refs or ref in mos_diode_refs or ref in sf_refs or ref in mos_active_res_refs:
                continue
            inst = inst_by_ref[ref]
            if inst.device_class != "mos" or pin not in {"D", "S"}:
                continue
            d = inst.terminals.get("D", "")
            s = inst.terminals.get("S", "")
            g = inst.terminals.get("G", "")
            # Must be strictly floating (neither D nor S tied to VDD/VSS) and not diode-tied
            if d in rails or s in rails or d == s or g in {d, s}:
                continue
            other_net = s if pin == "D" else d
            mos_active_res_refs.add(ref)
            roles[ref] = {
                "role": "mos_active_res",
                "polarity": inst.polarity,
                "gateNet": g,
                "endA": cur_net,
                "endB": other_net,
                "summary": f"{cur_net} — {other_net} (G={g})",
            }
            if other_net not in rails and other_net not in seen_chain_nets:
                if len(_unique_device_refs_on_net(net_terms, other_net)) == 2:
                    seen_chain_nets.add(other_net)
                    frontier_nets.append((other_net, hops + 1))

    # Third pass: annotate remaining MOS devices (composite stacks, current sources, cascode/amp)
    diode_nets = {inst_by_ref[r].terminals["D"] for r in mos_diode_refs}
    for inst in circuit.instances:
        r = inst.ref
        if r in roles:
            if r in stack_by_ref:
                stk = stack_by_ref[r]
                roles[r]["stackRefs"] = list(stk["refs"])
                roles[r]["stackPosition"] = stk["refs"].index(r)
            continue

        d = inst.terminals.get("D", "")
        g = inst.terminals.get("G", "")
        s = inst.terminals.get("S", "")
        if r in stack_by_ref:
            stk = stack_by_ref[r]
            roles[r] = {
                "role": "composite_stack_member",
                "polarity": inst.polarity,
                "stackRefs": list(stk["refs"]),
                "stackPosition": stk["refs"].index(r),
                "gateNet": g,
                "summary": f"stack={'-'.join(stk['refs'])}, G={g}",
            }
        elif s in rails and (g in input_port_nets or g in diode_nets):
            roles[r] = {
                "role": "current_source",
                "polarity": inst.polarity,
                "gateNet": g,
                "drainNet": d,
                "railNet": s,
                "summary": f"G={g}, D={d}, S={s}",
            }
        elif s not in rails and d not in rails and g in (input_port_nets | port_nets | diode_nets):
            roles[r] = {
                "role": "cascode_or_input_mos",
                "polarity": inst.polarity,
                "gateNet": g,
                "drainNet": d,
                "sourceNet": s,
                "summary": f"D={d}, G={g}, S={s}",
            }
        else:
            roles[r] = {
                "role": "mos_amp",
                "polarity": inst.polarity,
                "gateNet": g,
                "drainNet": d,
                "sourceNet": s,
                "summary": f"D={d}, G={g}, S={s}",
            }

    return roles


def build_reduced_mos_units(
    circuit: NetlistCircuit,
    co_gated_stacks: list[dict[str, Any]],
    device_roles: dict[str, dict[str, Any]],
) -> list[ReducedMosUnit]:
    """Pass 0b: Collapse co-gated series MOS chains into canonical `Super-MOS` units."""
    rails = set(circuit.supply_nets) | set(circuit.ground_nets)
    inst_by_ref = circuit.instance_map()
    excluded_roles = {"mos_cap", "mos_active_res"}

    in_stack: set[str] = set()
    units: list[ReducedMosUnit] = []

    for stk in co_gated_stacks:
        refs = [r for r in stk["refs"] if r in inst_by_ref]
        if len(refs) < 2:
            continue
        if any(device_roles.get(r, {}).get("role") in excluded_roles for r in refs):
            continue
        pol = str(stk["polarity"])
        top_inst = inst_by_ref[refs[0]]
        bot_inst = inst_by_ref[refs[-1]]
        gate = str(stk["gate_net"])
        if pol == "pmos":
            eff_s = top_inst.terminals["S"]
            eff_d = bot_inst.terminals["D"]
        else:
            eff_d = top_inst.terminals["D"]
            eff_s = bot_inst.terminals["S"]
        wl_sig = (
            len(refs),
            tuple((inst_by_ref[r].params.get("w"), inst_by_ref[r].params.get("l")) for r in refs),
        )
        in_stack.update(refs)
        units.append(
            ReducedMosUnit(
                unit_id=refs[0],
                refs=list(refs),
                polarity=pol,
                kind=top_inst.kind,
                gate=gate,
                eff_d=eff_d,
                eff_s=eff_s,
                is_diode=(eff_d == gate and eff_d not in rails),
                is_rail_source=(eff_s in rails),
                wl_sig=wl_sig,
                anchor_rail=eff_s if eff_s in rails else "",
            )
        )

    for inst in circuit.instances:
        if inst.device_class != "mos" or inst.ref in in_stack:
            continue
        if device_roles.get(inst.ref, {}).get("role") in excluded_roles:
            continue
        eff_d = inst.terminals.get("D", "")
        eff_s = inst.terminals.get("S", "")
        gate = inst.terminals.get("G", "")
        units.append(
            ReducedMosUnit(
                unit_id=inst.ref,
                refs=[inst.ref],
                polarity=inst.polarity,
                kind=inst.kind,
                gate=gate,
                eff_d=eff_d,
                eff_s=eff_s,
                is_diode=(eff_d == gate and eff_d not in rails),
                is_rail_source=(eff_s in rails),
                wl_sig=(1, ((inst.params.get("w"), inst.params.get("l")),)),
                anchor_rail=eff_s if eff_s in rails else "",
            )
        )

    return units


def detect_composite_diff_pairs(
    circuit: NetlistCircuit,
    units: list[ReducedMosUnit],
) -> tuple[list[CandidateMotif], set[str], set[str]]:
    """Detect both single-MOS and composite-stacked (`Super-MOS`) differential pairs + loads."""
    rails = set(circuit.supply_nets) | set(circuit.ground_nets)
    mos_list = [i for i in circuit.instances if i.device_class == "mos"]
    motifs: list[CandidateMotif] = []
    used_in_diff: set[str] = set()
    used_in_load: set[str] = set()

    by_tail: dict[tuple[str, str | None, str], list[ReducedMosUnit]] = defaultdict(list)
    for u in units:
        if u.eff_s and u.eff_s not in rails:
            by_tail[(u.polarity, u.kind, u.eff_s)].append(u)

    for (pol, _kind, tail_net), members in sorted(by_tail.items()):
        if len(members) != 2:
            continue
        ua, ub = members
        if ua.gate == ub.gate or ua.eff_d == ub.eff_d:
            continue
        if ua.wl_sig != ub.wl_sig:
            continue

        # Initial left/right (no net-name heuristics): declared subckt port order of the two gate nets, then
        # reference number as the final tie-break. The diode-load rule below overrides this when a mirror load exists.
        port_order = {p.net: i for i, p in enumerate(circuit.ports)}
        pa, pb = port_order.get(ua.gate), port_order.get(ub.gate)
        lr_tiebreak = "port_order"
        if pa is not None and pb is not None and pa != pb:
            if pb < pa:
                ua, ub = ub, ua
        else:
            lr_tiebreak = "naming"
            if _ref_num(ub.refs[0]) < _ref_num(ua.refs[0]):
                ua, ub = ub, ua

        used_in_diff.update(ua.refs)
        used_in_diff.update(ub.refs)

        # Find tail current source/sink unit connected to `tail_net`
        tail_unit = next(
            (
                u
                for u in units
                if u.unit_id not in {ua.unit_id, ub.unit_id}
                and u.eff_d == tail_net
                and u.is_rail_source
            ),
            None,
        )

        # Find matched active load pair (single or Super-MOS) directly on `(ua.eff_d, ub.eff_d)`
        # Exclude folded-cascode base sinks where `da`/`db` also feeds a same-polarity cascode Source
        da, db = ua.eff_d, ub.eff_d
        cands_a = [
            x
            for x in units
            if x.polarity != pol
            and not (set(x.refs) & used_in_load)
            and x.eff_d == da
            and x.is_rail_source
            and not any(
                c.ref not in set(x.refs)
                and c.polarity == x.polarity
                and c.terminals.get("S") == da
                and c.terminals.get("D") != c.terminals.get("S")
                for c in mos_list
            )
        ]
        cands_b = [
            x
            for x in units
            if x.polarity != pol
            and not (set(x.refs) & used_in_load)
            and x.eff_d == db
            and x.is_rail_source
            and not any(
                c.ref not in set(x.refs)
                and c.polarity == x.polarity
                and c.terminals.get("S") == db
                and c.terminals.get("D") != c.terminals.get("S")
                for c in mos_list
            )
        ]
        best_load_pair: tuple[ReducedMosUnit, ReducedMosUnit] | None = None
        best_rank = -1
        for xa in cands_a:
            for xb in cands_b:
                if xa.unit_id == xb.unit_id or xa.eff_s != xb.eff_s:
                    continue
                same_wl = xa.wl_sig == xb.wl_sig
                if xa.gate == xb.gate:
                    rk = 30 + (5 if same_wl else 0) + (3 if len(xa.refs) >= 2 else 0)
                elif xa.is_diode and xb.is_diode and same_wl:
                    rk = 25
                elif (not xa.is_diode) and (not xb.is_diode) and same_wl:
                    rk = 20
                else:
                    continue
                if rk > best_rank:
                    best_rank = rk
                    best_load_pair = (xa, xb)

        # Primary rule for single-ended mirror-loaded diff pairs:
        # place the diode-connected mirror reference leg on the LEFT and the single-ended
        # output leg on the RIGHT so signal flows naturally left-to-right into the next stage!
        if best_load_pair is not None:
            la0, lb0 = best_load_pair
            if lb0.is_diode and not la0.is_diode:
                ua, ub = ub, ua
                da, db = ua.eff_d, ub.eff_d
                best_load_pair = (lb0, la0)
            if la0.is_diode != lb0.is_diode:
                lr_tiebreak = "diode_load"

        # Primary anchor refs (bottom/tail-adjacent MOS of each diff stack for exact tail symmetry)
        left_anchor = ua.refs[-1] if pol == "nmos" else ua.refs[0]
        right_anchor = ub.refs[-1] if pol == "nmos" else ub.refs[0]
        roles: dict[str, Any] = {
            "left": left_anchor,
            "right": right_anchor,
            "left_stack": list(ua.refs),
            "right_stack": list(ub.refs),
            "tail_net": tail_net,
            "drain_left": da,
            "drain_right": db,
            "style": "back_to_back",
            "lr_tiebreak": lr_tiebreak,
            "is_composite": len(ua.refs) >= 2 or (best_load_pair is not None and len(best_load_pair[0].refs) >= 2),
        }
        inst_group: list[str] = [*ua.refs, *ub.refs]
        if tail_unit is not None:
            roles["tail_device"] = tail_unit.refs[0]
            roles["tail_stack"] = list(tail_unit.refs)
            inst_group.extend(tail_unit.refs)

        if best_load_pair is not None:
            la, lb = best_load_pair
            load_l_anchor = la.refs[-1] if la.polarity == "pmos" else la.refs[0]
            load_r_anchor = lb.refs[-1] if lb.polarity == "pmos" else lb.refs[0]
            roles["active_load_left"] = load_l_anchor
            roles["active_load_right"] = load_r_anchor
            roles["load_left_stack"] = list(la.refs)
            roles["load_right_stack"] = list(lb.refs)
            used_in_load.update(la.refs)
            used_in_load.update(lb.refs)
            inst_group.extend(la.refs)
            inst_group.extend(lb.refs)

        left_desc = "+".join(ua.refs)
        right_desc = "+".join(ub.refs)
        load_desc = (
            f"，有源镜像负载 ({'+'.join(best_load_pair[0].refs)}, {'+'.join(best_load_pair[1].refs)})"
            if best_load_pair is not None
            else ""
        )
        tail_desc = f"，尾流管 {'+'.join(tail_unit.refs)}" if tail_unit is not None else ""
        motifs.append(
            CandidateMotif(
                id=f"diff_{left_anchor}_{right_anchor}",
                motif_type="diff_pair",
                instances=inst_group,
                nets=[ua.gate, ub.gate, tail_net, da, db],
                roles=roles,
                notes=f"差分输入对 ({left_desc}, {right_desc}) 共尾节点 {tail_net}{tail_desc}{load_desc}",
            )
        )

    return motifs, used_in_diff, used_in_load


def detect_miller_bridges(
    circuit: NetlistCircuit,
    device_roles: dict[str, dict[str, Any]],
    existing_rc_refs: set[str] | None = None,
) -> list[CandidateMotif]:
    """Pass 1a: Detect generalized Miller frequency-compensation bridges (MOSCAP + active MOS res / passive RC)."""
    rails = set(circuit.supply_nets) | set(circuit.ground_nets)
    inst_by_ref = circuit.instance_map()
    net_terms = _unique_non_bulk_devices_by_net(circuit)
    excluded = set(existing_rc_refs or ())

    # Build 2-port endpoints for every candidate bridge element (`mos_cap`, `mos_active_res`, floating cap/res)
    bridge_ports: dict[str, tuple[str, str]] = {}
    for inst in circuit.instances:
        r = inst.ref
        if r in excluded:
            continue
        role = device_roles.get(r, {}).get("role", "")
        if role == "mos_cap":
            g = inst.terminals.get("G", "")
            ds = inst.terminals.get("D", "")
            if g and ds and g != ds and g not in rails and ds not in rails:
                bridge_ports[r] = (g, ds)
        elif role == "mos_active_res":
            d = inst.terminals.get("D", "")
            s = inst.terminals.get("S", "")
            if d and s and d != s and d not in rails and s not in rails:
                bridge_ports[r] = (s, d)
        elif inst.device_class.startswith(("cap", "res")):
            p = inst.terminals.get("P", "")
            m = inst.terminals.get("M", "")
            if p and m and p != m and p not in rails and m not in rails:
                bridge_ports[r] = (p, m)

    # Identify host common-source amplifier MOS devices (non-diode, non-cap, S in rails)
    host_amps: list[InstanceIR] = [
        i
        for i in circuit.instances
        if i.device_class == "mos"
        and device_roles.get(i.ref, {}).get("role") not in {"mos_cap", "mos_active_res", "mos_diode", "source_follower"}
        and i.terminals.get("S", "") in rails
        and i.terminals.get("G", "") not in rails
        and i.terminals.get("D", "") not in rails
        and i.terminals.get("G") != i.terminals.get("D")
    ]

    # Walk maximal exclusive-node chains starting from capacitive bridge seeds
    cap_seeds = [
        r
        for r in sorted(bridge_ports.keys(), key=_ref_num)
        if device_roles.get(r, {}).get("role") == "mos_cap" or inst_by_ref[r].device_class.startswith("cap")
    ]
    used_in_bridge: set[str] = set()
    results: list[CandidateMotif] = []

    for seed in cap_seeds:
        if seed in used_in_bridge:
            continue
        a0, b0 = bridge_ports[seed]

        def _extend_side(start_ref: str, side_net: str) -> tuple[list[str], str]:
            ext: list[str] = []
            cur_ref = start_ref
            cur_net = side_net
            seen_r = {start_ref}
            while True:
                devs = _unique_device_refs_on_net(net_terms, cur_net)
                if len(devs) != 2:
                    break
                nxt_candidates = [
                    x for x in devs if x not in seen_r and x in bridge_ports and x not in used_in_bridge
                ]
                if len(nxt_candidates) != 1:
                    break
                nxt = nxt_candidates[0]
                seen_r.add(nxt)
                ext.append(nxt)
                ea, eb = bridge_ports[nxt]
                cur_net = eb if ea == cur_net else ea
                cur_ref = nxt
            return ext, cur_net

        left_ext, end_a = _extend_side(seed, a0)
        right_ext, end_b = _extend_side(seed, b0)
        chain = list(reversed(left_ext)) + [seed] + right_ext

        # Match against a host common-source amplifier bridging `host.G <-> host.D`
        matched_hosts = [
            h
            for h in host_amps
            if {h.terminals["G"], h.terminals["D"]} == {end_a, end_b}
        ]
        if not matched_hosts:
            continue

        host = matched_hosts[0]
        from_net = host.terminals["G"]
        to_net = host.terminals["D"]
        if end_b == from_net and end_a == to_net:
            chain = list(reversed(chain))

        # Also find complementary active load(s) sharing `to_net` with `host.D` (e.g. M14 under M10)
        host_loads = [
            i.ref
            for i in circuit.instances
            if i.device_class == "mos"
            and i.ref != host.ref
            and i.ref not in set(chain)
            and i.polarity != host.polarity
            and i.terminals.get("D") == to_net
            and i.terminals.get("S") in rails
        ]
        used_in_bridge.update(chain)
        internal_nets = sorted(
            {n for r in chain for n in bridge_ports[r]} - {from_net, to_net}
        )
        kind_tag = (
            "mos_cap_active_res"
            if any(device_roles.get(r, {}).get("role") == "mos_active_res" for r in chain)
            else ("mos_cap_miller" if any(device_roles.get(r, {}).get("role") == "mos_cap" for r in chain) else "passive_miller")
        )
        results.append(
            CandidateMotif(
                id=f"miller_{host.ref}_{'_'.join(chain)}",
                motif_type="miller_bridge",
                instances=list(chain),
                nets=[from_net, *internal_nets, to_net],
                roles={
                    "chain": list(chain),
                    "host_amp": host.ref,
                    "host_loads": host_loads,
                    "from_net": from_net,
                    "to_net": to_net,
                    "bridge_kind": kind_tag,
                },
                confidence=1.0,
                notes=(
                    f"广义 Miller 补偿桥 ({' — '.join(chain)}) 跨接在增益管 {host.ref}"
                    f"（配载 {host_loads or '无'}）的栅漏之间：{from_net} → {to_net}。"
                ),
            )
        )

    return results


def detect_complementary_sf(
    circuit: NetlistCircuit,
    device_roles: dict[str, dict[str, Any]],
) -> list[CandidateMotif]:
    """Pass 1b: Detect complementary push-pull source-follower output buffers (e.g. NMOS+PMOS SF + CL)."""
    rails = set(circuit.supply_nets) | set(circuit.ground_nets)
    grounds = set(circuit.ground_nets)
    inst_by_ref = circuit.instance_map()

    n_sfs = [
        inst_by_ref[r]
        for r, info in sorted(device_roles.items())
        if info.get("role") == "source_follower" and inst_by_ref[r].polarity == "nmos"
    ]
    p_sfs = [
        inst_by_ref[r]
        for r, info in sorted(device_roles.items())
        if info.get("role") == "source_follower" and inst_by_ref[r].polarity == "pmos"
    ]

    used_p: set[str] = set()
    results: list[CandidateMotif] = []
    for mn in n_sfs:
        gn = mn.terminals["G"]
        sn = mn.terminals["S"]
        match_p = next(
            (
                mp
                for mp in p_sfs
                if mp.ref not in used_p
                and mp.terminals["G"] == gn
                and mp.terminals["S"] == sn
            ),
            None,
        )
        if match_p is None:
            continue
        used_p.add(match_p.ref)
        shunt_loads = [
            inst.ref
            for inst in circuit.instances
            if inst.device_class.startswith(("cap", "res"))
            and {inst.terminals.get("P", ""), inst.terminals.get("M", "")} & {sn}
            and {inst.terminals.get("P", ""), inst.terminals.get("M", "")} & (grounds | rails)
        ]
        results.append(
            CandidateMotif(
                id=f"comp_sf_{mn.ref}_{match_p.ref}",
                motif_type="complementary_sf",
                instances=[mn.ref, match_p.ref, *shunt_loads],
                nets=[gn, sn],
                roles={
                    "upper_nmos": mn.ref,
                    "lower_pmos": match_p.ref,
                    "input_net": gn,
                    "output_net": sn,
                    "shunt_loads": shunt_loads,
                },
                confidence=1.0,
                notes=(
                    f"互补源极跟随器输出缓冲 ({mn.ref} NMOS 上管 + {match_p.ref} PMOS 下管"
                    + (f"，负载 {shunt_loads}" if shunt_loads else "")
                    + f")：栅极输入 {gn} → 源极输出 {sn}。"
                ),
            )
        )
    return results


def detect_quasi_mirror_and_sandwich_cores(
    circuit: NetlistCircuit,
    *,
    device_roles: dict[str, dict[str, Any]],
    diff_motifs: list[CandidateMotif],
    excluded_refs: set[str] | None = None,
    min_half_instances: int = 3,
) -> list[dict[str, Any]]:
    """Pass 2: Fault-tolerant quasi-mirror subcircuit & sandwich symmetric core discovery.

    Unlike strict WL isomorphism, this pass:
      1. Allows internal local nets with up to 6 incident terminals (abstracting external
         single-ended output gate taps such as `M36.G` on `net4`);
      2. Expands matched internal net pairs `(na, nb)` by unique 1-to-1 `(role, polarity, pin)`
         correspondence instead of failing when `len(nbrs_a) != len(nbrs_b)`;
      3. Seeds from shared bias ports, 2-pin passive bridge elements (`R1`), and diff-pair
         differential outputs `(drain_left, drain_right)`;
      4. Automatically detects when a quasi-mirror pair wraps around an existing `diff_pair`
         to form a `sandwich_mirror_core` (`[Left_Wing | Diff_Core (+ bridge) | Right_Wing]`).
    """
    excluded = set(excluded_refs or ())
    rails = set(circuit.supply_nets) | set(circuit.ground_nets)
    supplies = set(circuit.supply_nets)
    grounds = set(circuit.ground_nets)
    port_nets = {p.net for p in circuit.ports}
    inst_by_ref = circuit.instance_map()
    inst_map = {i.ref: i for i in circuit.instances if i.ref not in excluded}
    if len(inst_map) < 2 * min_half_instances:
        return []

    full_net_terms = _unique_non_bulk_devices_by_net(circuit)
    pool_net_inc: dict[str, list[tuple[str, str]]] = defaultdict(list)
    for r, inst in inst_map.items():
        for pin, net in inst.terminals.items():
            if pin != "B":
                pool_net_inc[net].append((r, pin))

    def _norm_pin(inst: InstanceIR, pin: str) -> str:
        role = device_roles.get(inst.ref, {}).get("role", "")
        if role == "mos_cap":
            return "G" if pin == "G" else "DS"
        if inst.device_class.startswith(("res", "cap")):
            return "PM" if pin in {"P", "M"} else pin
        return pin

    def _is_local_expandable_net(net: str) -> bool:
        if net in rails or net in port_nets:
            return False
        # Must not be a global high-fanout bus; allow up to 6 unique devices (tolerates 1-2 external gate taps)
        return 2 <= len(_unique_device_refs_on_net(full_net_terms, net)) <= 6

    # Soft structural signature per device (ignoring asymmetric single-ended external gate fanout!)
    def _dev_soft_sig(ref: str) -> tuple[Any, ...]:
        inst = inst_map[ref]
        role = device_roles.get(ref, {}).get("role", "")
        role_bucket = role if role in {"mos_cap", "mos_active_res", "source_follower"} else "active"
        # Include fixed rail/port bindings on pins (e.g. S=VDD, G=VB2) while abstracting all internal nets
        pin_anchors: list[tuple[str, str]] = []
        for pin, net in inst.terminals.items():
            if pin == "B":
                continue
            npin = _norm_pin(inst, pin)
            if net in supplies:
                tag = "VDD"
            elif net in grounds:
                tag = "VSS"
            elif net in port_nets:
                tag = f"PORT:{net}"
            else:
                tag = "INT"
            pin_anchors.append((npin, tag))
        return (
            inst.device_class,
            inst.polarity,
            inst.kind,
            role_bucket,
            tuple(sorted(pin_anchors)),
        )

    soft_sig = {r: _dev_soft_sig(r) for r in inst_map}

    # Build 2-pin passive bridge candidates (e.g. R1 bridging net14 <-> net3)
    passive_bridges: list[tuple[str, str, str]] = []
    for r, inst in inst_map.items():
        if inst.device_class.startswith(("res", "cap")):
            pa, pb = inst.terminals.get("P", ""), inst.terminals.get("M", "")
            if pa and pb and pa != pb and pa not in rails and pb not in rails:
                passive_bridges.append((r, pa, pb))

    # Gather diverse symmetry seed pairs `(ra, rb)`
    seed_pairs: set[tuple[str, str]] = set()
    by_sig: dict[tuple[Any, ...], list[str]] = defaultdict(list)
    for r, sig in soft_sig.items():
        if inst_map[r].device_class in {"mos", "bjt"}:
            by_sig[sig].append(r)

    # Seed A: Same soft signature sharing a non-rail bias/control port (e.g. M18<->M32 on VB2)
    for sig, refs in by_sig.items():
        if len(refs) < 2:
            continue
        refs_s = sorted(refs)
        for i_a, ra in enumerate(refs_s):
            for rb in refs_s[i_a + 1 :]:
                cp = "G" if inst_map[ra].device_class == "mos" else "BASE"
                ga, gb = inst_map[ra].terminals.get(cp, ""), inst_map[rb].terminals.get(cp, "")
                if ga and ga == gb and ga not in rails:
                    seed_pairs.add((ra, rb))

    # Seed B: Matched neighbors across a 2-pin passive bridge (e.g. R1 on net14 <-> net3)
    for b_ref, na, nb in passive_bridges:
        na_devs = [r for r, _ in pool_net_inc.get(na, []) if r != b_ref and inst_map[r].device_class in {"mos", "bjt"}]
        nb_devs = [r for r, _ in pool_net_inc.get(nb, []) if r != b_ref and inst_map[r].device_class in {"mos", "bjt"}]
        for ra in na_devs:
            for rb in nb_devs:
                if ra != rb and soft_sig[ra] == soft_sig[rb]:
                    seed_pairs.add((min(ra, rb), max(ra, rb)))

    # Seed C: Matched downstream devices fed by a diff_pair's `(drain_left, drain_right)` (e.g. M21<->M31 on net16/net19)
    for dm in diff_motifs:
        da = dm.roles.get("drain_left")
        db = dm.roles.get("drain_right")
        if not da or not db or da == db:
            continue
        da_devs = [r for r, _ in pool_net_inc.get(da, []) if inst_map[r].device_class in {"mos", "bjt"}]
        db_devs = [r for r, _ in pool_net_inc.get(db, []) if inst_map[r].device_class in {"mos", "bjt"}]
        for ra in da_devs:
            for rb in db_devs:
                if ra != rb and soft_sig[ra] == soft_sig[rb]:
                    seed_pairs.add((min(ra, rb), max(ra, rb)))

    def _expand_quasi(seed_a: str, seed_b: str) -> dict[str, Any] | None:
        inst_bij: dict[str, str] = {seed_a: seed_b}
        rev_inst: dict[str, str] = {seed_b: seed_a}
        net_bij: dict[str, str] = {}
        rev_net: dict[str, str] = {}
        queue: deque[tuple[str, str]] = deque([(seed_a, seed_b)])

        while queue:
            ua, ub = queue.popleft()
            ia, ib = inst_map[ua], inst_map[ub]

            # Build canonical pin pairs (collapse D/S for mos_cap)
            pin_pairs: list[tuple[str, str]] = []
            if device_roles.get(ua, {}).get("role") == "mos_cap":
                pin_pairs = [("G", "G"), ("D", "D")]
            elif ia.device_class.startswith(("res", "cap")):
                pa, ma = ia.terminals.get("P", ""), ia.terminals.get("M", "")
                pb, mb = ib.terminals.get("P", ""), ib.terminals.get("M", "")
                if (pa == pb or net_bij.get(pa) == pb) or (ma == mb or net_bij.get(ma) == mb):
                    pin_pairs = [("P", "P"), ("M", "M")]
                else:
                    pin_pairs = [("P", "M"), ("M", "P")]
            else:
                for p in ia.terminals:
                    if p != "B" and p in ib.terminals:
                        pin_pairs.append((p, p))

            for pa, pb in pin_pairs:
                na, nb = ia.terminals.get(pa, ""), ib.terminals.get(pb, "")
                if not na or not nb or na == nb:
                    continue
                if na in rails or nb in rails:
                    return None
                if not (_is_local_expandable_net(na) and _is_local_expandable_net(nb)):
                    continue
                if na in net_bij:
                    if net_bij[na] != nb:
                        return None
                elif nb in rev_net:
                    return None
                else:
                    net_bij[na] = nb
                    rev_net[nb] = na

                # Collect unmatched pool neighbors on `(na, nb)` deduplicated by `(ref, norm_pin)`
                nbrs_a = sorted(
                    {
                        (r, _norm_pin(inst_map[r], p))
                        for r, p in pool_net_inc.get(na, [])
                        if r != ua and r not in inst_bij and r not in rev_inst
                    }
                )
                nbrs_b = sorted(
                    {
                        (r, _norm_pin(inst_map[r], p))
                        for r, p in pool_net_inc.get(nb, [])
                        if r != ub and r not in rev_inst and r not in inst_bij
                    }
                )

                # Unique 1-to-1 match by `(norm_pin, soft_sig)` — tolerates asymmetric external taps!
                matched_b_local: set[str] = set()
                for ra, role_a in nbrs_a:
                    if ra in inst_bij or ra in rev_inst:
                        continue
                    cands_b = [
                        rb
                        for rb, role_b in nbrs_b
                        if rb not in matched_b_local
                        and rb not in rev_inst
                        and rb not in inst_bij
                        and rb != ra
                        and role_b == role_a
                        and soft_sig[ra] == soft_sig[rb]
                    ]
                    if len(cands_b) == 1:
                        rb = cands_b[0]
                        # Ensure symmetric uniqueness (only one ra matches this rb on same role/sig)
                        rev_cands_a = [
                            ra2
                            for ra2, role_a2 in nbrs_a
                            if ra2 not in inst_bij
                            and ra2 not in rev_inst
                            and role_a2 == role_a
                            and soft_sig[ra2] == soft_sig[rb]
                        ]
                        if len(rev_cands_a) == 1:
                            matched_b_local.add(rb)
                            inst_bij[ra] = rb
                            rev_inst[rb] = ra
                            queue.append((ra, rb))

        half_a = set(inst_bij.keys())
        half_b = set(inst_bij.values())
        if half_a & half_b:
            return None

        # Verify internal connectivity & count matched internal private net pairs
        internal_priv = sum(
            1
            for na, nb in net_bij.items()
            if len({r for r, _ in pool_net_inc.get(na, []) if r in half_a}) >= 2
            and len({r for r, _ in pool_net_inc.get(nb, []) if r in half_b}) >= 2
        )
        if len(inst_bij) < min_half_instances or internal_priv < 1:
            return None

        # Find any passive bridge connecting a matched net pair `(na, nb)` across the two halves
        bridges = [
            b_ref
            for b_ref, pa, pb in passive_bridges
            if b_ref not in half_a
            and b_ref not in half_b
            and ((net_bij.get(pa) == pb) or (net_bij.get(pb) == pa))
        ]

        # Detect asymmetric external taps on matched internal nets (e.g. M36.G hanging off net4),
        # excluding the enclosed diff-pair itself so only true outer single-ended taps are reported.
        enclosed_diff: CandidateMotif | None = None
        swap_lr = False
        half_a_nets = {n for r in half_a for p, n in inst_map[r].terminals.items() if p != "B"}
        half_b_nets = {n for r in half_b for p, n in inst_map[r].terminals.items() if p != "B"}
        for dm in diff_motifs:
            da = dm.roles.get("drain_left")
            db = dm.roles.get("drain_right")
            if not da or not db:
                continue
            if da in half_a_nets and db in half_b_nets and da not in half_b_nets and db not in half_a_nets:
                enclosed_diff = dm
                swap_lr = False
                break
            if db in half_a_nets and da in half_b_nets and db not in half_b_nets and da not in half_a_nets:
                enclosed_diff = dm
                swap_lr = True
                break

        if swap_lr:
            inst_bij = {v: k for k, v in inst_bij.items()}
            net_bij = {v: k for k, v in net_bij.items()}
            half_a, half_b = half_b, half_a
        elif enclosed_diff is None:
            # Canonicalize left vs right by lowest reference number
            if min(_ref_num(r) for r in half_b) < min(_ref_num(r) for r in half_a):
                inst_bij = {v: k for k, v in inst_bij.items()}
                net_bij = {v: k for k, v in net_bij.items()}
                half_a, half_b = half_b, half_a

        asym_taps: list[dict[str, str]] = []
        enclosed_refs = set(enclosed_diff.instances) if enclosed_diff is not None else set()
        all_core_refs = half_a | half_b | set(bridges) | enclosed_refs
        for na, nb in sorted(net_bij.items()):
            ext_a = [
                (r, p) for r, p in full_net_terms.get(na, []) if r not in all_core_refs
            ]
            ext_b = [
                (r, p) for r, p in full_net_terms.get(nb, []) if r not in all_core_refs
            ]
            if len(ext_a) != len(ext_b):
                for r, p in ext_a:
                    asym_taps.append({"side": "left", "net": na, "ref": r, "pin": p})
                for r, p in ext_b:
                    asym_taps.append({"side": "right", "net": nb, "ref": r, "pin": p})

        # Require either >= 2 internal private nets OR an enclosed diff pair / passive bridge
        if internal_priv < 2 and enclosed_diff is None and not bridges:
            return None

        pairs_sorted = sorted(inst_bij.items(), key=lambda kv: _ref_num(kv[0]))
        return {
            "leftInstances": [k for k, _ in pairs_sorted],
            "rightInstances": [v for _, v in pairs_sorted],
            "pairs": [[k, v] for k, v in pairs_sorted],
            "privateNetPairs": [[k, v] for k, v in sorted(net_bij.items())],
            "internalPrivateCount": internal_priv,
            "bridgeDevices": sorted(bridges, key=_ref_num),
            "enclosedDiffPairId": enclosed_diff.id if enclosed_diff is not None else None,
            "enclosedDiffInstances": list(enclosed_diff.instances) if enclosed_diff is not None else [],
            "asymmetricTaps": asym_taps,
            "isQuasi": bool(asym_taps) or bool(bridges) or (enclosed_diff is not None),
        }

    candidates: list[dict[str, Any]] = []
    for ra, rb in sorted(seed_pairs):
        res = _expand_quasi(ra, rb)
        if res is not None:
            candidates.append(res)

    candidates.sort(
        key=lambda c: (
            0 if c["enclosedDiffPairId"] else 1,
            -len(c["pairs"]),
            -len(c["bridgeDevices"]),
            -int(c["internalPrivateCount"]),
            c["leftInstances"],
        )
    )

    claimed: set[str] = set()
    final_results: list[dict[str, Any]] = []
    for cand in candidates:
        all_members = (
            set(cand["leftInstances"])
            | set(cand["rightInstances"])
            | set(cand["bridgeDevices"])
        )
        if all_members & claimed:
            continue
        claimed.update(all_members)
        final_results.append(cand)

    return final_results


def build_signal_flow_backbone(
    circuit: NetlistCircuit,
    motifs: list[CandidateMotif],
    device_roles: dict[str, dict[str, Any]],
    net_fanout: dict[str, int],
) -> dict[str, Any]:
    """Pass 3: Build directed inter-block signal-flow DAG and bottleneck-cut macro stages."""
    rails = set(circuit.supply_nets) | set(circuit.ground_nets)
    inst_by_ref = circuit.instance_map()
    port_by_net = {p.net: p for p in circuit.ports}
    input_port_nets = {p.net for p in circuit.ports if p.direction == "input" and p.net not in rails}
    output_port_nets = {p.net for p in circuit.ports if p.direction == "output" and p.net not in rails}

    # --- Step 1: Partition all instances into atomic functional blocks ---
    claimed: set[str] = set()
    blocks: list[dict[str, Any]] = []

    def _add_block(bid: str, kind: str, label: str, insts: list[str], extra: dict[str, Any] | None = None) -> None:
        uniq = [r for r in insts if r in inst_by_ref and r not in claimed]
        if not uniq:
            return
        claimed.update(uniq)
        b = {
            "id": bid,
            "kind": kind,
            "label": label,
            "instances": uniq,
        }
        if extra:
            b.update(extra)
        blocks.append(b)

    # Priority 1: Sandwich symmetric cores (left wing + enclosed diff pair + bridge + right wing)
    for m in motifs:
        if m.motif_type == "sandwich_mirror_core":
            _add_block(
                m.id,
                "sandwich_mirror_core",
                "三明治对称放大核心",
                list(m.instances),
                {
                    "enclosedDiffPairId": m.roles.get("enclosedDiffPairId"),
                    "bridgeDevices": list(m.roles.get("bridgeDevices", [])),
                },
            )

    # Priority 2: Complementary source-follower output buffers
    for m in motifs:
        if m.motif_type == "complementary_sf":
            _add_block(
                m.id,
                "output_buffer",
                "互补源极跟随器输出缓冲",
                list(m.instances),
                {"outputNet": m.roles.get("output_net")},
            )

    # Priority 3: Generalized Miller bridge + host amplifier + host complementary load
    for m in motifs:
        if m.motif_type == "miller_bridge":
            host = m.roles.get("host_amp")
            loads = list(m.roles.get("host_loads", []))
            members = [r for r in [host, *loads, *m.instances] if r]
            _add_block(
                f"gain_miller_{host}",
                "gain_with_miller",
                f"共源增益与 Miller 补偿 ({host})",
                members,
                {"millerBridgeId": m.id, "hostAmp": host},
            )

    # Priority 4: Remaining differential input stages
    for m in motifs:
        if m.motif_type == "diff_pair":
            if any(r in claimed for r in m.instances):
                continue
            _add_block(
                m.id,
                "diff_stage",
                "级联差分输入级" if m.roles.get("is_composite") else "差分输入级",
                list(m.instances),
            )

    # Priority 5: Standalone mirror_block_pair, parallel_bank, serpentine_chain, rc_series
    for m in motifs:
        if m.motif_type in {"mirror_block_pair", "parallel_bank", "serpentine_chain", "rc_series"}:
            if any(r in claimed for r in m.instances):
                continue
            _add_block(m.id, m.motif_type, m.notes.split("：")[0][:24], list(m.instances))

    # Priority 6: Remaining instances grouped by DC channel connectivity (D/S/C/E/P/M on non-rail internal nets)
    rem_refs = [i.ref for i in circuit.instances if i.ref not in claimed]
    if rem_refs:
        rem_set = set(rem_refs)
        adj: dict[str, set[str]] = defaultdict(set)
        ch_by_net: dict[str, list[str]] = defaultdict(list)
        for r in rem_refs:
            inst = inst_by_ref[r]
            for pin, net in inst.terminals.items():
                if pin in {"B", "G", "BASE"} or net in rails or net in input_port_nets:
                    continue
                ch_by_net[net].append(r)
        for net, rlist in ch_by_net.items():
            if len(rlist) <= 6:
                for u in rlist:
                    for v in rlist:
                        if u != v:
                            adj[u].add(v)
        seen_rem: set[str] = set()
        for r in sorted(rem_refs, key=_ref_num):
            if r in seen_rem:
                continue
            comp: list[str] = []
            q: deque[str] = deque([r])
            seen_rem.add(r)
            while q:
                cur = q.popleft()
                comp.append(cur)
                for nxt in sorted(adj.get(cur, ()), key=_ref_num):
                    if nxt in rem_set and nxt not in seen_rem:
                        seen_rem.add(nxt)
                        q.append(nxt)
            comp.sort(key=_ref_num)
            has_diode = any(device_roles.get(x, {}).get("role") == "mos_diode" for x in comp)
            has_out = any(
                output_port_nets & {n for p, n in inst_by_ref[x].terminals.items() if p != "B"}
                for x in comp
            )
            has_src = any(inst_by_ref[x].device_class in {"isource", "vsource"} for x in comp)
            b_kind = (
                "bias_source"
                if has_src
                else ("output_load" if has_out else ("mirror_branch" if has_diode else "gain_branch"))
            )
            b_label = (
                "偏置支路"
                if has_src
                else ("输出支路" if has_out else ("反相镜转换支路" if has_diode else "增益支路"))
            )
            _add_block(f"branch_{'_'.join(comp[:2])}", b_kind, b_label, comp)

    block_of_ref = {r: b["id"] for b in blocks for r in b["instances"]}
    block_by_id = {b["id"]: b for b in blocks}

    # --- Step 2: Build directed inter-block signal edges ---
    edges: list[dict[str, Any]] = []
    seen_edge_keys: set[tuple[str, str, str]] = set()
    for net in circuit.nets:
        if net in rails or net in input_port_nets:
            continue
        drivers: set[str] = set()
        gate_sinks: set[str] = set()
        for inst in circuit.instances:
            bid = block_of_ref.get(inst.ref)
            if not bid:
                continue
            role = device_roles.get(inst.ref, {}).get("role", "")
            for pin, pnet in inst.terminals.items():
                if pnet != net or pin == "B":
                    continue
                if pin in {"G", "BASE"} and role != "mos_cap":
                    gate_sinks.add(bid)
                else:
                    drivers.add(bid)

        # Case A: Clean directed `D/S -> G` signal flow edges
        if drivers and gate_sinks:
            for src in sorted(drivers):
                for dst in sorted(gate_sinks):
                    if src == dst:
                        continue
                    ek = (src, dst, net)
                    if ek not in seen_edge_keys:
                        seen_edge_keys.add(ek)
                        edges.append({"src": src, "dst": dst, "net": net, "edgeKind": "D->G"})
        # Case B: Channel-to-channel coupling between distinct blocks (e.g. cap coupling or load shunt)
        elif len(drivers) >= 2:
            d_list = sorted(drivers, key=lambda b_id: min(_ref_num(r) for r in block_by_id[b_id]["instances"]))
            for i_d in range(len(d_list) - 1):
                src, dst = d_list[i_d], d_list[i_d + 1]
                ek = (src, dst, net)
                if ek not in seen_edge_keys:
                    seen_edge_keys.add(ek)
                    edges.append({"src": src, "dst": dst, "net": net, "edgeKind": "channel"})

    # --- Step 3: Longest-path DAG depth ordering (with cycle guard) ---
    indeg: dict[str, int] = {b["id"]: 0 for b in blocks}
    out_adj: dict[str, list[str]] = defaultdict(list)
    for e in edges:
        out_adj[e["src"]].append(e["dst"])
        indeg[e["dst"]] += 1

    depth: dict[str, int] = {b["id"]: 0 for b in blocks}
    topo_q: deque[str] = deque(sorted([bid for bid, d in indeg.items() if d == 0]))
    topo_order: list[str] = []
    while topo_q:
        u = topo_q.popleft()
        topo_order.append(u)
        for v in out_adj.get(u, []):
            depth[v] = max(depth[v], depth[u] + 1)
            indeg[v] -= 1
            if indeg[v] == 0:
                topo_q.append(v)
    # Append any cycle nodes gracefully
    for b in blocks:
        if b["id"] not in topo_order:
            topo_order.append(b["id"])

    # Anchor output_buffer / output_port blocks strictly at rightmost depth
    max_d = max(depth.values(), default=0)
    for b in blocks:
        has_out_port = any(
            output_port_nets & {n for p, n in inst_by_ref[r].terminals.items() if p != "B"}
            for r in b["instances"]
        )
        if b["kind"] == "output_buffer" or (has_out_port and not out_adj.get(b["id"])):
            depth[b["id"]] = max(depth[b["id"]], max_d)

    for ord_idx, bid in enumerate(
        sorted(
            [b["id"] for b in blocks],
            key=lambda x: (
                depth[x],
                min(_ref_num(r) for r in block_by_id[x]["instances"]),
                x,
            ),
        )
    ):
        block_by_id[bid]["depth"] = depth[bid]
        block_by_id[bid]["order"] = ord_idx

    # --- Step 4: Bottleneck cut-edge (bridge) detection on block graph ---
    undir_adj: dict[str, list[tuple[str, int]]] = defaultdict(list)
    for ei, e in enumerate(edges):
        undir_adj[e["src"]].append((e["dst"], ei))
        undir_adj[e["dst"]].append((e["src"], ei))

    def _components_without_edge(skip_ei: int) -> list[set[str]]:
        seen: set[str] = set()
        comps: list[set[str]] = []
        for b in blocks:
            bid = b["id"]
            if bid in seen:
                continue
            comp: set[str] = set()
            q_c: deque[str] = deque([bid])
            seen.add(bid)
            while q_c:
                u = q_c.popleft()
                comp.add(u)
                for v, ei_v in undir_adj.get(u, []):
                    if ei_v == skip_ei or v in seen:
                        continue
                    seen.add(v)
                    q_c.append(v)
            comps.append(comp)
        return comps

    base_comp_count = len(_components_without_edge(-1))
    cut_nets: list[dict[str, Any]] = []
    for ei, e in enumerate(edges):
        if e["edgeKind"] != "D->G":
            continue
        comps = _components_without_edge(ei)
        if len(comps) > base_comp_count:
            src_comp = next(c for c in comps if e["src"] in c)
            dst_comp = next(c for c in comps if e["dst"] in c)
            src_inst_cnt = sum(len(block_by_id[b]["instances"]) for b in src_comp)
            dst_inst_cnt = sum(len(block_by_id[b]["instances"]) for b in dst_comp)
            cut_nets.append(
                {
                    "net": e["net"],
                    "edgeIndex": ei,
                    "edgeKind": e["edgeKind"],
                    "cutWidth": 1,
                    "srcBlock": e["src"],
                    "dstBlock": e["dst"],
                    "srcSideBlocks": sorted(src_comp),
                    "dstSideBlocks": sorted(dst_comp),
                    "srcSideInstances": src_inst_cnt,
                    "dstSideInstances": dst_inst_cnt,
                }
            )

    # --- Step 5: Select macro-stage cuts to partition into readable macro blocks ---
    # Prefer cuts that separate:
    #   (a) an output_buffer stage on the right, OR
    #   (b) a multi-block upstream sub-amplifier (e.g. diff_stage + gain_with_miller) from the main amplifier!
    selected_cut_edges: set[int] = set()
    boundary_ambiguous_blocks: list[dict[str, Any]] = []

    # Check consecutive single-block bridge chains between two cuts (e.g. `M15/M16` between `net0` and `net12`)
    cut_by_dst = {c["dstBlock"]: c for c in cut_nets}
    cut_by_src = {c["srcBlock"]: c for c in cut_nets}

    for c in cut_nets:
        ei = c["edgeIndex"]
        src_b = block_by_id[c["srcBlock"]]
        dst_b = block_by_id[c["dstBlock"]]

        # Cut directly entering an output buffer is always a primary macro boundary
        if dst_b["kind"] == "output_buffer":
            selected_cut_edges.add(ei)
            continue

        # Cut leaving a multi-block amplifier sub-path (e.g. Gain Booster diff+CS/Miller -> mirror/main amp)
        src_has_diff = any(block_by_id[b]["kind"] == "diff_stage" for b in c["srcSideBlocks"])
        dst_has_core = any(
            block_by_id[b]["kind"] in {"sandwich_mirror_core", "diff_stage", "mirror_block_pair"}
            for b in c["dstSideBlocks"]
        )
        if src_has_diff and dst_has_core and len(c["srcSideBlocks"]) >= 2:
            # If this cut leads into a small 1-block converter (`mirror_branch`, e.g. M15/M16) that ALSO has an
            # outgoing 1-edge cut (`net12`), pick the upstream cut (`net0`) by default so the mirror converter stays
            # adjacent to the main amp (matching standard Gain Booster | Main Amplifier split) and flag the ambiguity!
            if src_b["kind"] == "mirror_branch" and src_b["id"] in cut_by_dst:
                prev_cut = cut_by_dst[src_b["id"]]
                boundary_ambiguous_blocks.append(
                    {
                        "blockId": src_b["id"],
                        "instances": list(src_b["instances"]),
                        "upstreamCutNet": prev_cut["net"],
                        "downstreamCutNet": c["net"],
                        "defaultSide": "right_main_stage",
                    }
                )
                continue
            selected_cut_edges.add(ei)

    # Build macro stages from connected components after removing `selected_cut_edges`
    seen_ms: set[str] = set()
    macro_stages: list[dict[str, Any]] = []
    for b in sorted(blocks, key=lambda x: (x["depth"], x["order"])):
        if b["id"] in seen_ms:
            continue
        comp_b: list[str] = []
        q_m: deque[str] = deque([b["id"]])
        seen_ms.add(b["id"])
        while q_m:
            u = q_m.popleft()
            comp_b.append(u)
            for v, ei_v in undir_adj.get(u, []):
                if ei_v in selected_cut_edges or v in seen_ms:
                    continue
                seen_ms.add(v)
                q_m.append(v)
        comp_b.sort(key=lambda x: (block_by_id[x]["depth"], block_by_id[x]["order"]))
        st_insts = [r for bid in comp_b for r in block_by_id[bid]["instances"]]
        kinds = {block_by_id[bid]["kind"] for bid in comp_b}
        if "output_buffer" in kinds and len(comp_b) == 1:
            st_label = "Output Buffer / 输出缓冲级"
            st_role = "output_buffer"
        elif "sandwich_mirror_core" in kinds:
            st_label = "Main Amplifier / 主对称放大核心"
            st_role = "main_amplifier"
        elif "diff_stage" in kinds and "gain_with_miller" in kinds and bool(selected_cut_edges):
            st_label = "Gain Booster / 辅助增益与补偿级"
            st_role = "gain_booster"
        elif "diff_stage" in kinds and "gain_with_miller" in kinds:
            st_label = "Amplifier Core / 多级放大器主干"
            st_role = "amplifier_core"
        elif "diff_stage" in kinds:
            st_label = "Differential Input Stage / 差分放大级"
            st_role = "input_diff"
        elif "bias_source" in kinds:
            st_label = "Bias / 偏置基准级"
            st_role = "bias"
        else:
            st_label = f"Stage {len(macro_stages) + 1}"
            st_role = "gain_stage"
        macro_stages.append(
            {
                "id": f"macro_stage_{len(macro_stages) + 1}",
                "label": st_label,
                "role": st_role,
                "minDepth": min(block_by_id[bid]["depth"] for bid in comp_b),
                "minOrder": min(block_by_id[bid]["order"] for bid in comp_b),
                "blockIds": comp_b,
                "instances": st_insts,
            }
        )

    macro_stages.sort(key=lambda s: (s["minDepth"], s["minOrder"]))

    # Order blocks within each macro stage:
    #   0) upstream stage entry blocks (receiving a selected cut edge from an earlier macro stage, e.g. M15/M16 on net0)
    #   1) parallel/core interior blocks (e.g. sandwich_mirror_core M17-M35+R1)
    #   2) downstream fan-in / exit blocks (receiving edges from sibling blocks in the same stage, e.g. M36/M37)
    stage_idx_of_block: dict[str, int] = {}
    for s_idx, st in enumerate(macro_stages):
        for bid in st["blockIds"]:
            stage_idx_of_block[bid] = s_idx

    for s_idx, st in enumerate(macro_stages):
        st_set = set(st["blockIds"])
        local_in_from_prev_stage = {
            e["dst"]
            for e in edges
            if e["dst"] in st_set and stage_idx_of_block.get(e["src"], s_idx) < s_idx
        }
        local_in_from_same_stage = {
            e["dst"]
            for e in edges
            if e["dst"] in st_set and e["src"] in st_set
        }
        local_out_to_same_stage = {
            e["src"]
            for e in edges
            if e["src"] in st_set and e["dst"] in st_set
        }

        def _intra_stage_rank(bid: str) -> tuple[int, int, int]:
            in_prev = bid in local_in_from_prev_stage
            in_same = bid in local_in_from_same_stage
            out_same = bid in local_out_to_same_stage
            if in_prev and not in_same:
                slot = 0  # Upstream stage entry converter (e.g. M15/M16 immediately after net0)
            elif not in_same and out_same:
                slot = 1  # Parallel core feeding stage combiner (e.g. sandwich_core_1)
            elif in_same and out_same:
                slot = 2
            elif in_same:
                slot = 3  # Stage combiner / output driver (e.g. M36/M37)
            else:
                slot = 1
            return (slot, block_by_id[bid]["depth"], block_by_id[bid]["order"])

        st["blockIds"] = sorted(st["blockIds"], key=_intra_stage_rank)
        st["instances"] = [r for bid in st["blockIds"] for r in block_by_id[bid]["instances"]]
        for local_ord, bid in enumerate(st["blockIds"]):
            block_by_id[bid]["stageIndex"] = s_idx
            block_by_id[bid]["stageLocalOrder"] = local_ord

    # Compute final linear block index across all ordered macro stages (used to detect non-adjacent long nets!)
    linear_block_idx: dict[str, int] = {}
    for st in macro_stages:
        for bid in st["blockIds"]:
            linear_block_idx[bid] = len(linear_block_idx)

    # --- Step 6: Identify cross-stage shared ports & truly long non-adjacent internal nets ---
    stage_of_ref = {r: st["id"] for st in macro_stages for r in st["instances"]}
    sandwich_wing_pairs: list[tuple[set[str], set[str]]] = []
    for m in motifs:
        if m.motif_type in {"sandwich_mirror_core", "mirror_block_pair"}:
            l_set = {r for col in m.roles.get("leftColumns", []) for r in col}
            r_set = {r for col in m.roles.get("rightColumns", []) for r in col}
            if l_set and r_set:
                sandwich_wing_pairs.append((l_set, r_set))

    cross_stage_nets: list[dict[str, Any]] = []
    for net in circuit.nets:
        if net in rails:
            continue
        touched_refs = sorted(
            {
                inst.ref
                for inst in circuit.instances
                if any(p != "B" and n == net for p, n in inst.terminals.items())
            }
        )
        touched_stages = sorted({stage_of_ref[r] for r in touched_refs if r in stage_of_ref})
        touched_blocks = sorted(
            {block_of_ref[r] for r in touched_refs if r in block_of_ref},
            key=lambda bid: linear_block_idx.get(bid, 0),
        )
        block_span = (
            linear_block_idx[touched_blocks[-1]] - linear_block_idx[touched_blocks[0]]
            if len(touched_blocks) >= 2
            else 0
        )
        spans_mirror_wings = any(
            bool(set(touched_refs) & ls) and bool(set(touched_refs) & rs)
            for ls, rs in sandwich_wing_pairs
        )
        is_port = net in port_by_net
        fo = net_fanout.get(net, 0)

        if is_port and (len(touched_stages) >= 2 or block_span >= 2 or spans_mirror_wings):
            cross_stage_nets.append(
                {
                    "net": net,
                    "isPort": True,
                    "fanout": fo,
                    "stages": touched_stages,
                    "blocks": touched_blocks,
                    "suggestedPolicy": "label_stub",
                    "reason": (
                        f"跨阶段/对称翼共用顶层端口 ({', '.join(touched_refs)})，建议局部线标或外围母线避免横穿核心"
                    ),
                }
            )
        elif not is_port and block_span >= 2:
            # Truly non-adjacent internal signal skipping over intermediate block(s) (e.g. net12 skipping sandwich core)
            cross_stage_nets.append(
                {
                    "net": net,
                    "isPort": False,
                    "fanout": fo,
                    "stages": touched_stages,
                    "blocks": touched_blocks,
                    "suggestedPolicy": "perimeter_bus",
                    "reason": (
                        f"跨越中间子块的非相邻内部长信号（{' → '.join(touched_blocks)}），建议底部外围母线或线标"
                    ),
                }
            )

    return {
        "blocks": sorted(blocks, key=lambda b: (b["depth"], b["order"])),
        "edges": edges,
        "cutNets": cut_nets,
        "selectedCutNets": [c["net"] for c in cut_nets if c["edgeIndex"] in selected_cut_edges],
        "macroStages": macro_stages,
        "boundaryAmbiguousBlocks": boundary_ambiguous_blocks,
        "crossStageNets": cross_stage_nets,
    }
