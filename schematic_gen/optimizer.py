"""Bounded equivalence-preserving local flip & column-swap crossing minimization (Phase 2.1 M3)."""

from __future__ import annotations

import copy
from typing import Any

from schematic_gen.auditor import audit_solved_schematic
from schematic_gen.models import NetlistCircuit, SchematicIntent, SolvedSchematic
from schematic_gen.pdk import PdkRegistry
from schematic_gen.router import compile_schematic


_FLIP_MAP: dict[str, str] = {
    "gate_left": "gate_right",
    "gate_right": "gate_left",
    "vertical": "vertical_flipped",
    "vertical_flipped": "vertical",
    "horizontal_l2r": "horizontal_r2l",
    "horizontal_r2l": "horizontal_l2r",
}


def _cost_of_solved(solved: SolvedSchematic, pdk: PdkRegistry) -> tuple[float, int, dict[str, int]]:
    """Compute lexicographic cost `1e5 * hard_drc + 100 * crossings + 8 * bends + wire_len`."""
    rep = audit_solved_schematic(solved, pdk)
    m = rep.metrics
    hard_drc = (
        int(m.get("symbolOverlapCount", 0))
        + int(m.get("labelOverlapCount", 0))
        + int(m.get("wireThroughBodyCount", 0))
        + int(m.get("wireOverForeignPinCount", 0))
        + int(m.get("collinearForeignWireOverlapCount", 0))
        + int(m.get("ambiguousCrossingCount", 0))
        + (0 if rep.electrical_passed else 10)
    )
    crossings = int(m.get("wireCrossingCount", 0))
    bends = sum(max(0, len(rt.points()) - 2) for rt in solved.routes)
    wire_len = int(m.get("totalWireLengthGrids", 0))
    score = 1e5 * hard_drc + 100.0 * crossings + 8.0 * bends + float(wire_len)
    return score, crossings, dict(m.get("crossingsByNet", {}))


def optimize_schematic_crossings(
    circuit: NetlistCircuit,
    intent: SchematicIntent,
    pdk: PdkRegistry | None = None,
    *,
    allow_wire_labels: bool = False,
    max_evals: int = 24,
) -> SolvedSchematic:
    """Run compile + bounded local flip/swap hill climbing to eliminate avoidable wire crossings."""
    pdk = pdk or PdkRegistry()
    best_intent = copy.deepcopy(intent)
    best_solved = compile_schematic(
        circuit, best_intent, pdk=pdk, allow_wire_labels=allow_wire_labels
    )
    best_score, best_cross, cross_by_net = _cost_of_solved(best_solved, pdk)
    if best_cross == 0 and best_score < 1e5:
        return best_solved

    inst_map = circuit.instance_map()
    locked_refs: set[str] = set()
    for m in best_intent.motifs:
        if m.get("type") in {"diff_pair", "parallel_bank", "serpentine_chain"}:
            for key in ("left", "right", "loadLeft", "loadRight"):
                if m.get(key):
                    locked_refs.add(str(m[key]))
            for r in m.get("instances", []):
                locked_refs.add(str(r))

    evals_used = 0
    moves_taken: list[str] = []

    while evals_used < max_evals and best_cross > 0:
        improved = False
        # Prioritize instances & nets actually involved in current crossings!
        hot_nets = {n for n, cnt in cross_by_net.items() if cnt > 0}
        candidate_intents: list[tuple[str, SchematicIntent]] = []

        # 1. Flip orientation of unlocked instances connected to crossing nets
        for inst in circuit.instances:
            if inst.ref in locked_refs:
                continue
            if not (set(inst.terminals.values()) & hot_nets):
                continue
            if inst.device_class == "mos":
                cur_o = best_intent.orientations.get(inst.ref, "gate_left")
            elif inst.device_class in {"resistor", "capacitor", "cap_mim", "cap_mim_m4"}:
                cur_o = best_intent.orientations.get(inst.ref, "vertical")
            else:
                continue
            flipped = _FLIP_MAP.get(cur_o)
            if not flipped:
                continue
            cand = copy.deepcopy(best_intent)
            cand.orientations[inst.ref] = flipped
            candidate_intents.append((f"flip {inst.ref}: {cur_o} -> {flipped}", cand))

        # 2. Promote crossing non-critical multi-pin nets to `perimeter_bus`
        crit = set(best_intent.critical_nets)
        rails = set(circuit.supply_nets) | set(circuit.ground_nets)
        for net, cnt in sorted(cross_by_net.items(), key=lambda x: -x[1]):
            if cnt >= 1 and net not in crit and net not in rails:
                if best_intent.net_policies.get(net, "full_wire") == "full_wire":
                    cand = copy.deepcopy(best_intent)
                    cand.net_policies[net] = "perimeter_bus"
                    candidate_intents.append((f"netPolicy {net}: full_wire -> perimeter_bus", cand))

        # 2b. Move crossing low-fanout Gate input ports from far-left `left` edge to `near_pin`
        for p_port in circuit.ports:
            if p_port.direction == "input" and p_port.net in hot_nets and p_port.net not in rails:
                if best_intent.port_sides.get(p_port.name, "left") == "left":
                    conn_pins = [
                        pin
                        for inst in circuit.instances
                        for pin, pnet in inst.terminals.items()
                        if pnet == p_port.net and pin != "B"
                    ]
                    if 1 <= len(conn_pins) <= 2 and all(pin == "G" for pin in conn_pins):
                        cand = copy.deepcopy(best_intent)
                        cand.port_sides[p_port.name] = "near_pin"
                        candidate_intents.append((f"portSide {p_port.name}: left -> near_pin", cand))

        # 3. Adjacent intra-group column swap for unlocked columns touching crossing nets
        group_of = {r: g.id for g in best_intent.groups for r in g.instances}
        cols = best_intent.column_order_hints
        for ci in range(len(cols) - 1):
            c_a, c_b = cols[ci], cols[ci + 1]
            if not c_a or not c_b:
                continue
            if any(r in locked_refs for r in c_a + c_b):
                continue
            if group_of.get(c_a[0]) != group_of.get(c_b[0]):
                continue
            nets_ab = {
                n
                for r in c_a + c_b
                if r in inst_map
                for n in inst_map[r].terminals.values()
            }
            if not (nets_ab & hot_nets):
                continue
            cand = copy.deepcopy(best_intent)
            cand.column_order_hints[ci], cand.column_order_hints[ci + 1] = (
                list(c_b),
                list(c_a),
            )
            candidate_intents.append((f"swap columns {c_a} <-> {c_b}", cand))

        for desc, cand_intent in candidate_intents:
            if evals_used >= max_evals:
                break
            evals_used += 1
            try:
                cand_solved = compile_schematic(
                    circuit, cand_intent, pdk=pdk, allow_wire_labels=allow_wire_labels
                )
            except Exception:
                continue
            sc, cr, c_map = _cost_of_solved(cand_solved, pdk)
            if sc + 1e-6 < best_score:
                best_score = sc
                best_cross = cr
                cross_by_net = c_map
                best_intent = cand_intent
                best_solved = cand_solved
                moves_taken.append(f"{desc} (crossings -> {cr})")
                improved = True
                break

        if not improved:
            break

    best_solved.optimization_moves = moves_taken
    return best_solved
