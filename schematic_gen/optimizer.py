"""Bounded equivalence-preserving local flip & column-swap crossing minimization (Phase 2.1 M3).

Layering: this L3 module does not import the L4 auditor. The scalar quality cost is injected through
`cost_fn` (default: `schematic_gen.quality.solved_quality_cost`, imported lazily).
"""

from __future__ import annotations

import copy
from typing import Any, Callable

from schematic_gen.models import NetlistCircuit, SchematicIntent, SolvedSchematic
from schematic_gen.pdk import PdkRegistry
from schematic_gen.router import compile_schematic
from schematic_gen.topology_core import is_nonpolar_2pin_class, net_policy_may_override

CostFn = Callable[[SolvedSchematic, PdkRegistry], tuple[float, int, dict[str, int]]]


_FLIP_MAP: dict[str, str] = {
    "gate_left": "gate_right",
    "gate_right": "gate_left",
    "vertical": "vertical_flipped",
    "vertical_flipped": "vertical",
    "horizontal_l2r": "horizontal_r2l",
    "horizontal_r2l": "horizontal_l2r",
}


def _default_cost_fn() -> CostFn:
    from schematic_gen.quality import solved_quality_cost

    return solved_quality_cost


def optimize_schematic_crossings(
    circuit: NetlistCircuit,
    intent: SchematicIntent,
    pdk: PdkRegistry | None = None,
    *,
    allow_wire_labels: bool = False,
    max_evals: int = 24,
    cost_fn: CostFn | None = None,
) -> SolvedSchematic:
    """Run compile + bounded local flip/swap hill climbing to eliminate avoidable wire crossings."""
    pdk = pdk or PdkRegistry()
    _cost_of_solved = cost_fn or _default_cost_fn()
    best_intent = copy.deepcopy(intent)
    best_solved = compile_schematic(
        circuit, best_intent, pdk=pdk, allow_wire_labels=allow_wire_labels
    )
    best_score, best_cross, cross_by_net = _cost_of_solved(best_solved, pdk)
    if best_cross == 0 and best_score < 1e5:
        return best_solved

    inst_map = circuit.instance_map()
    locked_refs: set[str] = set()
    # Lock every device participating in a declared or recognized symmetry/structure motif so the
    # local search can never break LLM-declared mirror skeletons, sandwich cores or bridge orientations.
    for m in best_intent.motifs:
        for key in ("left", "right", "tail", "loadLeft", "loadRight"):
            if m.get(key):
                locked_refs.add(str(m[key]))
        for key in ("instances", "leftColumn", "rightColumn", "stackedDrivers"):
            for r in m.get(key, []) or []:
                locked_refs.add(str(r))
        for key in ("leftColumns", "rightColumns"):
            for col in m.get(key, []) or []:
                for r in col:
                    locked_refs.add(str(r))
        for pair in m.get("pairs", []) or []:
            for r in pair:
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
            if inst.device_class in {"mos", "bjt"}:
                cur_o = best_intent.orientations.get(inst.ref, "gate_left")
            elif is_nonpolar_2pin_class(inst.device_class):
                cur_o = best_intent.orientations.get(inst.ref, "vertical")
            else:
                continue
            flipped = _FLIP_MAP.get(cur_o)
            if not flipped:
                continue
            cand = copy.deepcopy(best_intent)
            cand.orientations[inst.ref] = flipped
            candidate_intents.append((f"flip {inst.ref}: {cur_o} -> {flipped}", cand))

        # 2. Promote crossing non-critical multi-pin nets to `perimeter_bus` — only where the optimizer has
        #    authority (never over LLM- or skeleton-sourced policies; see NET_POLICY_PRIORITY).
        crit = set(best_intent.critical_nets)
        rails = set(circuit.supply_nets) | set(circuit.ground_nets)
        for net, cnt in sorted(cross_by_net.items(), key=lambda x: -x[1]):
            if cnt >= 1 and net not in crit and net not in rails:
                if best_intent.net_policies.get(net, "full_wire") == "full_wire" and net_policy_may_override(
                    best_intent.net_policy_source(net), "optimizer"
                ):
                    cand = copy.deepcopy(best_intent)
                    cand.net_policies[net] = "perimeter_bus"
                    cand.net_policy_sources[net] = "optimizer"
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
