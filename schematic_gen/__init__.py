"""PDK-Agnostic High-Readability Analog Schematic Generator (`analog-schem-gen`).

Co-execution architecture separating LLM circuit-semantic judgment (`IntentSpec` / `HierarchyIntent`)
from deterministic constrained placement (Convex QP + Array Supernodes), Steiner trunk synthesis,
grid-aligned obstacle-avoiding orthogonal routing + track nudging, local crossing optimization,
hierarchical multi-sheet partitioning, and flatten-back LVS / readability DRC verification.
"""

__version__ = "0.3.0"
