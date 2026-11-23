"""Structure-preserving draft schematic refiner (expands cramped spots, compacts excessive gaps, re-routes & nudges)."""

from __future__ import annotations

from collections import defaultdict
from typing import Any

from schematic_gen.models import (
    FunctionalGroupSpec,
    InstanceIR,
    NetlistCircuit,
    PortIR,
    SchematicIntent,
    SolvedSchematic,
    SpacingConfig,
)
from schematic_gen.pdk import PdkRegistry, source_orientation
from schematic_gen.router import compile_schematic


_CADENCE_TO_SEMANTIC = {
    "R0": "gate_left",
    "MY": "gate_right",
    "MX": "gate_left_flipped",
    "R180": "gate_right_flipped",
    "R90": "horizontal_l2r",
    "R270": "horizontal_r2l",
}


def circuit_and_intent_from_manifest(
    manifest_data: dict[str, Any],
    *,
    spacing: SpacingConfig | None = None,
) -> tuple[NetlistCircuit, SchematicIntent]:
    """Extract electrical `NetlistCircuit` + topological `SchematicIntent` from an existing draft manifest.

    Preserves the draft's left-to-right column order, vertical row tiering, and symbol orientations,
    while allowing `compile_schematic` to fix overlaps, compact excessive whitespace, and re-route cleanly.
    """
    raw_circuit = (manifest_data.get("circuits") or [{}])[0]
    cell_name = str(raw_circuit.get("cellName", "refined_circuit"))

    ports = [
        PortIR(
            name=str(p["name"]),
            net=str(p["netName"]),
            direction=str(p.get("direction", "inout")),
        )
        for p in raw_circuit.get("ports", [])
    ]
    nets = [str(n) for n in raw_circuit.get("nets", [])]
    supplies = [n for n in nets if n.upper() in {"VDD", "AVDD", "VPWR", "VCC"}]
    grounds = [n for n in nets if n.upper() in {"VSS", "AVSS", "VGND", "GND", "0"}]

    instances: list[InstanceIR] = []
    pos_by_ref: dict[str, tuple[float, float]] = {}
    orientations: dict[str, str] = {}

    for item in raw_circuit.get("instances", []):
        ref = str(item["reference"])
        dev_class = str(item["deviceClass"])
        kind = str(item["kind"]) if item.get("kind") else None
        polarity = (
            kind
            if dev_class == "mos" and kind in {"nmos", "pmos"}
            else ("pmos" if (kind or "").startswith("pmos") else ("nmos" if dev_class == "mos" else "passive"))
        )
        terms = {
            str(n["sourcePinName"]): str(n["netName"]) for n in item.get("nodes", [])
        }
        params = {str(k): str(v) for k, v in (item.get("sourceParameters") or {}).items()}
        instances.append(
            InstanceIR(
                ref=ref,
                raw_name=ref,
                device_class=dev_class,
                kind=kind,
                polarity=polarity,
                source_model=str(item.get("sourceTarget", dev_class)),
                terminals=terms,
                params=params,
            )
        )
        sp = item["sourcePosition"]
        pos_by_ref[ref] = (float(sp["x"]), float(sp["y"]))
        tr = item.get("sourceTransform") or {}
        cad = source_orientation(int(round(float(tr.get("rotation", 0)))), str(tr.get("mirror", "none")))
        if dev_class != "mos" and cad == "R0":
            orientations[ref] = "vertical"
        else:
            orientations[ref] = _CADENCE_TO_SEMANTIC.get(cad, "gate_left")

    circuit = NetlistCircuit(
        cell_name=cell_name,
        ports=ports,
        instances=instances,
        nets=nets,
        supply_nets=supplies,
        ground_nets=grounds,
    )

    # Cluster X coordinates within tolerance 3.0 into vertical columns (preserving left-to-right order!)
    sorted_by_x = sorted(instances, key=lambda i: (pos_by_ref[i.ref][0], pos_by_ref[i.ref][1]))
    columns: list[list[str]] = []
    col_xs: list[float] = []
    for inst in sorted_by_x:
        x, _ = pos_by_ref[inst.ref]
        if columns and abs(x - col_xs[-1]) <= 3.5:
            columns[-1].append(inst.ref)
        else:
            columns.append([inst.ref])
            col_xs.append(x)

    # Cluster Y coordinates within tolerance 3.5 into discrete vertical tiers 1..K (preserving top-to-bottom order!)
    sorted_by_y = sorted(instances, key=lambda i: pos_by_ref[i.ref][1])
    tier_overrides: dict[str, int] = {}
    cur_tier = 1
    last_y: float | None = None
    for inst in sorted_by_y:
        _, y = pos_by_ref[inst.ref]
        if last_y is not None and abs(y - last_y) > 3.5:
            cur_tier += 1
        tier_overrides[inst.ref] = cur_tier
        last_y = y

    for col in columns:
        col.sort(key=lambda r: pos_by_ref[r][1])

    # Keep all instances in one or gap-split groups so excessive gaps get compacted cleanly!
    group = FunctionalGroupSpec(
        id="refined_core",
        label="保结构自动整理组",
        role="core",
        stage_order=0,
        instances=[i.ref for i in sorted_by_x],
    )
    net_policies = {
        n: ("supply_local" if n in (set(supplies) | set(grounds)) else "full_wire")
        for n in nets
    }

    intent = SchematicIntent(
        cell_name=cell_name,
        groups=[group],
        motifs=[],
        orientations=orientations,
        tier_overrides=tier_overrides,
        column_order_hints=columns,
        net_policies=net_policies,
        critical_nets=[n for n in nets if n not in (set(supplies) | set(grounds))],
        port_sides={
            p.name: ("local_repeat" if p.net in (set(supplies) | set(grounds)) else ("right" if p.direction == "output" else "left"))
            for p in ports
        },
        spacing=spacing or SpacingConfig(),
        authored_by="structure_preserving_refiner",
        llm_notes="从既有原理图草图提取的行列顺序与朝向，经 QP 撑开过挤区、收紧过松空白并重走正交分轨导线。",
    )
    return circuit, intent


def refine_existing_manifest(
    manifest_data: dict[str, Any],
    pdk: PdkRegistry | None = None,
    *,
    spacing: SpacingConfig | None = None,
) -> SolvedSchematic:
    """Refine an existing cramped/stretched schematic manifest while preserving topological structure."""
    pdk = pdk or PdkRegistry()
    circuit, intent = circuit_and_intent_from_manifest(manifest_data, spacing=spacing)
    return compile_schematic(circuit, intent, pdk=pdk)
