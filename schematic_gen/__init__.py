"""PDK-Agnostic High-Readability Analog Schematic Generator (`analog-schem-gen`).

Co-execution architecture separating LLM circuit-semantic judgment (`IntentSpec`)
from deterministic constrained placement (Convex QP), Steiner trunk synthesis,
grid-aligned obstacle-avoiding orthogonal routing + track nudging, and
dual electrical/readability DRC verification.
"""

__version__ = "0.1.0"
