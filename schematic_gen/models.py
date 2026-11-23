"""Core data models for netlist IR, topology analysis, LLM intent, layout, and DRC."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any


Point = tuple[int, int]
BBox = tuple[int, int, int, int]  # (min_x, min_y, max_x, max_y) in source y-down grid units


@dataclass
class InstanceIR:
    """Normalized circuit primitive or hierarchical subcircuit instance."""

    ref: str
    raw_name: str
    device_class: str  # e.g. "mos", "res_poly_3t", "cap_mim", "resistor", "capacitor", "isource", "vsource", "diode", "subckt"
    kind: str | None   # for mos: "nmos", "pmos", "nmos_lvt", "pmos_hvt", etc.
    polarity: str      # "nmos", "pmos", "passive", "source", "block"
    source_model: str
    terminals: dict[str, str]       # canonical pin ("D","G","S","B" or "P","M","B" or subckt pin) -> net_name
    params: dict[str, str] = field(default_factory=dict)
    param_expressions: dict[str, str] = field(default_factory=dict)
    section_hint: str = ""
    subckt_cell: str | None = None

    @property
    def device_key(self) -> str:
        if self.device_class == "subckt" and self.subckt_cell:
            return f"subckt:{self.subckt_cell}"
        return str(self.kind if self.device_class == "mos" else self.device_class)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class PortIR:
    """Top-level logical circuit port."""

    name: str
    net: str
    direction: str = "inout"  # "input", "output", "inout"

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class NetlistCircuit:
    """Normalized flat or leaf-cell netlist ready for topology analysis."""

    cell_name: str
    ports: list[PortIR]
    instances: list[InstanceIR]
    nets: list[str]
    supply_nets: list[str] = field(default_factory=list)
    ground_nets: list[str] = field(default_factory=list)

    def instance_map(self) -> dict[str, InstanceIR]:
        return {inst.ref: inst for inst in self.instances}

    def port_map(self) -> dict[str, PortIR]:
        return {port.name: port for port in self.ports}

    def to_dict(self) -> dict[str, Any]:
        return {
            "cellName": self.cell_name,
            "ports": [p.to_dict() for p in self.ports],
            "instances": [i.to_dict() for i in self.instances],
            "nets": list(self.nets),
            "supplyNets": list(self.supply_nets),
            "groundNets": list(self.ground_nets),
        }


@dataclass
class NetlistDesign:
    """Multi-subcircuit hierarchical netlist collection."""

    cells: dict[str, NetlistCircuit]
    top_cell: str
    global_params: dict[str, float] = field(default_factory=dict)

    def to_dict(self) -> dict[str, Any]:
        return {
            "topCell": self.top_cell,
            "globalParams": dict(self.global_params),
            "cells": {k: v.to_dict() for k, v in self.cells.items()},
        }


@dataclass
class CandidateMotif:
    """Deterministically recognized analog circuit building block."""

    id: str
    motif_type: str  # "diff_pair", "current_mirror", "cascode_pair", "diode_mos", "rc_series", "vertical_branch"
    instances: list[str]
    nets: list[str]
    roles: dict[str, Any] = field(default_factory=dict)
    confidence: float = 1.0
    notes: str = ""

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class AmbiguityItem:
    """Explicit question surfaced by the analyzer for LLM semantic judgment."""

    id: str
    category: str  # "stage_order", "net_policy", "motif_orientation", "bridge_style"
    subjects: list[str]
    question: str
    options: list[str]
    default_choice: str
    rationale: str = ""

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class TopologyAnalysis:
    """Output of `analyze`: normalized circuit, candidate motifs, net stats, and LLM prompts."""

    circuit: NetlistCircuit
    motifs: list[CandidateMotif]
    net_fanout: dict[str, int]
    net_classifications_default: dict[str, str]
    default_tiers: dict[str, int]
    ambiguities_for_llm: list[AmbiguityItem]
    draft_intent: dict[str, Any]

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema": "schem_gen.analysis.v1",
            "circuit": self.circuit.to_dict(),
            "candidateMotifs": [m.to_dict() for m in self.motifs],
            "netFanout": dict(self.net_fanout),
            "defaultNetPolicies": dict(self.net_classifications_default),
            "defaultTiers": dict(self.default_tiers),
            "ambiguitiesForLLM": [a.to_dict() for a in self.ambiguities_for_llm],
            "draftIntent": self.draft_intent,
        }


@dataclass
class FunctionalGroupSpec:
    """Named functional module with left-to-right reading stage order."""

    id: str
    label: str
    role: str          # "bias", "input_diff", "cascode_core", "gain_stage", "compensation", "output_stage", "cmfb", "aux"
    stage_order: int   # 0 = leftmost, increasing to the right
    instances: list[str]

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class SpacingConfig:
    """Target gaps and clearance bounds in integer gridUnits (0.0625 inch each)."""

    intra_group_gap_x: int = 8
    inter_group_gap_x: int = 14
    min_clearance_x: int = 5
    intra_stack_gap_y: int = 8
    inter_tier_gap_y: int = 10
    min_clearance_y: int = 6
    wire_track_pitch: int = 2
    shape_buffer: int = 2
    port_margin_x: int = 12
    port_margin_y: int = 8

    @classmethod
    def from_dict(cls, raw: dict[str, Any] | None) -> "SpacingConfig":
        if not raw:
            return cls()
        allowed = {f for f in cls.__dataclass_fields__}
        filtered = {k: int(v) for k, v in raw.items() if k in allowed}
        return cls(**filtered)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class SchematicIntent:
    """Semantic contract authored/refined by the LLM (zero raw pixel coordinates)."""

    cell_name: str
    groups: list[FunctionalGroupSpec]
    motifs: list[dict[str, Any]]
    orientations: dict[str, str]      # ref -> "gate_left" | "gate_right" | "vertical" | "vertical_flipped" | "horizontal_l2r" | "horizontal_r2l"
    tier_overrides: dict[str, int]    # ref -> 1..5 potential tier
    column_order_hints: list[list[str]]  # optional explicit left-to-right columns or aligned vertical stacks
    net_policies: dict[str, str]      # net -> "supply_local" | "bulk_local" | "full_wire" | "perimeter_bus"
    critical_nets: list[str]          # nets whose continuous readability is highest priority
    port_sides: dict[str, str]        # port_name -> "left" | "right" | "top" | "bottom" | "local_repeat"
    spacing: SpacingConfig = field(default_factory=SpacingConfig)
    authored_by: str = "heuristic_draft"
    llm_notes: str = ""

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "SchematicIntent":
        groups = [
            FunctionalGroupSpec(
                id=str(g["id"]),
                label=str(g.get("label", g["id"])),
                role=str(g.get("role", "aux")),
                stage_order=int(g.get("stage_order", idx)),
                instances=[str(x) for x in g.get("instances", [])],
            )
            for idx, g in enumerate(data.get("groups", []))
        ]
        return cls(
            cell_name=str(data.get("cellName") or data.get("cell_name") or "circuit"),
            groups=groups,
            motifs=list(data.get("motifs", [])),
            orientations={str(k): str(v) for k, v in data.get("orientations", {}).items()},
            tier_overrides={str(k): int(v) for k, v in data.get("tierOverrides", data.get("tier_overrides", {})).items()},
            column_order_hints=[[str(x) for x in col] for col in data.get("columnOrderHints", data.get("column_order_hints", []))],
            net_policies={str(k): str(v) for k, v in data.get("netPolicies", data.get("net_policies", {})).items()},
            critical_nets=[str(x) for x in data.get("criticalNets", data.get("critical_nets", []))],
            port_sides={str(k): str(v) for k, v in data.get("portSides", data.get("port_sides", {})).items()},
            spacing=SpacingConfig.from_dict(data.get("spacing")),
            authored_by=str(data.get("authoredBy", data.get("authored_by", "user_or_llm"))),
            llm_notes=str(data.get("llmNotes", data.get("llm_notes", ""))),
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema": "schem_gen.intent.v1",
            "cellName": self.cell_name,
            "authoredBy": self.authored_by,
            "llmNotes": self.llm_notes,
            "groups": [g.to_dict() for g in self.groups],
            "motifs": list(self.motifs),
            "orientations": dict(self.orientations),
            "tierOverrides": dict(self.tier_overrides),
            "columnOrderHints": list(self.column_order_hints),
            "netPolicies": dict(self.net_policies),
            "criticalNets": list(self.critical_nets),
            "portSides": dict(self.port_sides),
            "spacing": self.spacing.to_dict(),
        }


@dataclass
class PlacedInstance:
    """Solved integer-grid placement and bounding envelopes for one instance."""

    ref: str
    instance_id: str
    device_class: str
    kind: str | None
    polarity: str
    device_key: str
    x: int
    y: int
    rotation: int
    mirror: str
    cadence_orient: str
    semantic_orient: str
    pin_points: dict[str, Point]  # canonical pin -> exact (x, y) in source y-down coords
    body_box: BBox
    label_box: BBox
    full_envelope: BBox
    label_lines: list[str]
    tier: int
    group_id: str
    has_bulk_short: bool = False
    bulk_label_net: str | None = None


@dataclass
class PortOccurrencePlaced:
    """Solved port symbol occurrence (edge IO pin or local VDD/VSS tap)."""

    occurrence_id: str
    port_name: str
    net_name: str
    direction: str
    symbol_id: str  # "port", "vdd-port", "ground"
    x: int
    y: int
    rotation: int
    mirror: str


@dataclass
class JunctionPlaced:
    """Explicit T-junction or trunk anchor on a net."""

    id: str
    net_name: str
    x: int
    y: int
    role: str = "steiner"


@dataclass
class RouteStepPlaced:
    """One step in an orthogonal route (either bend or terminal/port/junction endpoint)."""

    kind: str  # "bend" | "instance" | "port" | "junction"
    x: int
    y: int
    instance_id: str | None = None
    pin_name: str | None = None
    junction_id: str | None = None


@dataclass
class RoutePlaced:
    """Complete orthogonal polyline segment chain connecting two endpoints."""

    id: str
    net_name: str
    start: RouteStepPlaced
    steps: list[RouteStepPlaced]
    presentation: str = "wire"

    def points(self) -> list[Point]:
        pts = [(self.start.x, self.start.y)]
        for st in self.steps:
            pts.append((st.x, st.y))
        return pts


@dataclass
class WireLabelStubPlaced:
    """Short directional wire stub carrying a Virtuoso/Xschem net label (`schCreateWireLabel`)."""

    id: str
    net_name: str
    ref: str
    pin_name: str
    x1: int
    y1: int
    x2: int
    y2: int


@dataclass
class BusRipperPlaced:
    """Visual bus trunk with angled ripper taps for regular arrays (`SW<7:0>`)."""

    id: str
    bus_name: str
    trunk_y: int
    x_start: int
    x_end: int
    taps: list[tuple[str, int, int]] = field(default_factory=list)  # (net_name, tap_x, pin_y)


@dataclass
class SolvedSchematic:
    """Complete solved schematic geometry prior to Manifest/SVG serialization."""

    circuit: NetlistCircuit
    intent: SchematicIntent
    placements: dict[str, PlacedInstance]
    ports: list[PortOccurrencePlaced]
    junctions: dict[str, JunctionPlaced]
    routes: list[RoutePlaced]
    local_bulk_labels: list[dict[str, str]]
    local_bulk_shorts: list[dict[str, str]]
    bounds: BBox
    congestion_iterations: int = 1
    corridor_expansions: list[str] = field(default_factory=list)
    wire_label_stubs: list[WireLabelStubPlaced] = field(default_factory=list)
    bus_rippers: list[BusRipperPlaced] = field(default_factory=list)
    optimization_moves: list[str] = field(default_factory=list)


@dataclass
class AuditFinding:
    """Single electrical or readability DRC finding."""

    severity: str   # "ERROR", "WARNING", "INFO"
    code: str
    message: str
    subjects: list[str] = field(default_factory=list)

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class AuditReport:
    """Dual electrical equivalence & readability DRC report + LLM feedback."""

    cell_name: str
    electrical_passed: bool
    readability_passed: bool
    bridge_manifest_valid: bool
    metrics: dict[str, Any]
    findings: list[AuditFinding]
    llm_refinement_suggestions: list[str]

    @property
    def overall_passed(self) -> bool:
        return self.electrical_passed and self.readability_passed and self.bridge_manifest_valid

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema": "schem_gen.audit.v1",
            "signoff": False,
            "cellName": self.cell_name,
            "overallPassed": self.overall_passed,
            "electricalPassed": self.electrical_passed,
            "readabilityPassed": self.readability_passed,
            "bridgeManifestValid": self.bridge_manifest_valid,
            "metrics": dict(self.metrics),
            "findings": [f.to_dict() for f in self.findings],
            "llmRefinementSuggestions": list(self.llm_refinement_suggestions),
        }
