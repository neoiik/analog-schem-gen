"""Layout quality cost (L4 signoff metrics turned into a scalar for L3 search).

Kept separate so the L3 optimizer never imports the L4 auditor at module level: callers inject this
(or any other) cost function into `optimize_schematic_crossings(cost_fn=...)`.
"""

from __future__ import annotations

from schematic_gen.auditor import audit_solved_schematic
from schematic_gen.models import SolvedSchematic
from schematic_gen.pdk import PdkRegistry


def solved_quality_cost(solved: SolvedSchematic, pdk: PdkRegistry) -> tuple[float, int, dict[str, int]]:
    """Lexicographic cost `1e5 * hard_drc + 100 * crossings + 8 * bends + wire_len`."""
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
