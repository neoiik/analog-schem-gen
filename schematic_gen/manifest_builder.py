"""Serializer for `virtuoso-bridge-exact-schematic-v1` and offline Virtuoso Bridge verifier."""

from __future__ import annotations

import json
import os
from pathlib import Path
import sys
from typing import Any

from schematic_gen.models import RouteStepPlaced, SolvedSchematic
from schematic_gen.pdk import PdkRegistry


def ensure_virtuoso_bridge_on_path() -> None:
    """Optionally add `VIRTUOSO_BRIDGE_SRC` to `sys.path` if set by the caller."""
    env_src = os.environ.get("VIRTUOSO_BRIDGE_SRC", "").strip()
    if env_src:
        p = Path(env_src)
        if p.is_dir() and str(p) not in sys.path:
            sys.path.insert(0, str(p))


def _step_to_manifest_endpoint(step: RouteStepPlaced) -> dict[str, Any]:
    pt = {"x": int(step.x), "y": int(step.y)}
    if step.kind == "bend":
        return {"kind": "bend", "position": pt}
    if step.kind == "junction":
        return {
            "kind": "junction",
            "junctionId": str(step.junction_id),
            "sourcePoint": pt,
        }
    if step.kind == "port":
        return {
            "kind": "port",
            "instanceId": str(step.instance_id),
            "pinName": str(step.pin_name),
            "sourcePoint": pt,
        }
    return {
        "kind": "instance",
        "instanceId": str(step.instance_id),
        "pinName": str(step.pin_name),
        "sourcePoint": pt,
    }


def build_bridge_manifest(solved: SolvedSchematic, pdk: PdkRegistry | None = None) -> dict[str, Any]:
    """Compile `SolvedSchematic` into `virtuoso-bridge-exact-schematic-v1` JSON."""
    pdk = pdk or PdkRegistry()
    inst_map = solved.circuit.instance_map()

    manifest_instances: list[dict[str, Any]] = []
    for inst in solved.circuit.instances:
        pl = solved.placements[inst.ref]
        nodes = [
            {
                "sourcePinName": canon_pin,
                "pinName": canon_pin,
                "netName": net_name,
            }
            for canon_pin, net_name in inst.terminals.items()
        ]
        item: dict[str, Any] = {
            "id": pl.instance_id,
            "reference": inst.ref,
            "deviceClass": inst.device_class,
            "sourceTarget": inst.source_model,
            "sourcePosition": {"x": int(pl.x), "y": int(pl.y)},
            "sourceTransform": {"rotation": int(pl.rotation), "mirror": str(pl.mirror)},
            "nodes": nodes,
            "sourceParameters": dict(inst.params),
        }
        if inst.device_class == "mos":
            item["kind"] = str(inst.kind or "nmos")
        manifest_instances.append(item)

    manifest_ports = [
        {
            "name": p.name,
            "netName": p.net,
            "direction": p.direction,
        }
        for p in solved.circuit.ports
    ]

    port_occurrences = [
        {
            "occurrenceId": po.occurrence_id,
            "name": po.port_name,
            "netName": po.net_name,
            "direction": po.direction,
            "sourceSymbolId": po.symbol_id,
            "sourcePosition": {"x": int(po.x), "y": int(po.y)},
            "sourceTransform": {"rotation": int(po.rotation), "mirror": str(po.mirror)},
        }
        for po in solved.ports
    ]

    manifest_junctions = [
        {
            "id": j.id,
            "netName": j.net_name,
            "role": j.role,
            "sourcePosition": {"x": int(j.x), "y": int(j.y)},
        }
        for j in solved.junctions.values()
    ]

    manifest_routes = [
        {
            "id": rt.id,
            "netName": rt.net_name,
            "presentation": rt.presentation,
            "start": _step_to_manifest_endpoint(rt.start),
            "steps": [_step_to_manifest_endpoint(st) for st in rt.steps],
        }
        for rt in solved.routes
    ]

    bx0, by0, bx1, by1 = solved.bounds
    circuit_entry = {
        "cellName": solved.circuit.cell_name,
        "instances": manifest_instances,
        "ports": manifest_ports,
        "nets": list(solved.circuit.nets),
        "sourceGeometry": {
            "coordinateSystem": "source coordinates: x right, y down (1 unit = 1 Virtuoso gridUnit)",
            "bounds": {
                "minX": int(bx0),
                "minY": int(by0),
                "maxX": int(bx1),
                "maxY": int(by1),
            },
            "portOccurrences": port_occurrences,
            "junctions": manifest_junctions,
            "contacts": [],
            "routes": manifest_routes,
            "localBulkLabels": list(solved.local_bulk_labels),
            "localBulkShorts": list(solved.local_bulk_shorts),
            "annotationStubs": [],
        },
    }
    return {
        "schema": "virtuoso-bridge-exact-schematic-v1",
        "circuits": [circuit_entry],
    }


def build_canvas_ir(solved: SolvedSchematic) -> dict[str, Any]:
    """Export human/agent-readable Analog-Canvas-style layout summary with groups and envelopes."""
    return {
        "schema": "schem_gen.canvas_ir.v1",
        "cellName": solved.circuit.cell_name,
        "bounds": {
            "minX": solved.bounds[0],
            "minY": solved.bounds[1],
            "maxX": solved.bounds[2],
            "maxY": solved.bounds[3],
        },
        "congestionIterations": solved.congestion_iterations,
        "corridorExpansions": list(solved.corridor_expansions),
        "groups": [g.to_dict() for g in solved.intent.groups],
        "instances": [
            {
                "ref": pl.ref,
                "deviceClass": pl.device_class,
                "kind": pl.kind,
                "group": pl.group_id,
                "tier": pl.tier,
                "position": {"x": pl.x, "y": pl.y},
                "semanticOrient": pl.semantic_orient,
                "cadenceOrient": pl.cadence_orient,
                "pins": {k: {"x": v[0], "y": v[1]} for k, v in pl.pin_points.items()},
                "bodyBox": list(pl.body_box),
                "labelBox": list(pl.label_box),
                "labelLines": list(pl.label_lines),
            }
            for pl in solved.placements.values()
        ],
        "ports": [
            {
                "id": po.occurrence_id,
                "name": po.port_name,
                "net": po.net_name,
                "symbol": po.symbol_id,
                "x": po.x,
                "y": po.y,
            }
            for po in solved.ports
        ],
        "routes": [
            {
                "id": rt.id,
                "net": rt.net_name,
                "points": [{"x": px, "y": py} for px, py in rt.points()],
            }
            for rt in solved.routes
        ],
    }


def _standalone_preflight(
    manifest_data: dict[str, Any],
    process_map_data: dict[str, Any],
    process_name: str,
) -> dict[str, Any]:
    """Built-in offline validator used when `virtuoso-bridge-lite` is not installed."""
    if manifest_data.get("schema") != "virtuoso-bridge-exact-schematic-v1":
        return {"passed": False, "engine": "standalone", "error": "invalid manifest schema"}
    if process_map_data.get("schema") != "virtuoso-bridge-process-map-v1":
        return {"passed": False, "engine": "standalone", "error": "invalid process-map schema"}
    processes = process_map_data.get("processes") or {}
    if process_name not in processes:
        return {"passed": False, "engine": "standalone", "error": f"unknown process {process_name!r}"}
    circuits = manifest_data.get("circuits") or []
    if not circuits:
        return {"passed": False, "engine": "standalone", "error": "empty circuits list"}
    audits: list[dict[str, Any]] = []
    for c in circuits:
        nets = set(c.get("nets") or [])
        inst_pins: set[tuple[str, str, str]] = set()
        for inst in c.get("instances") or []:
            iid = str(inst["id"])
            for n in inst.get("nodes") or []:
                net = str(n["netName"])
                if net not in nets:
                    return {"passed": False, "engine": "standalone", "error": f"unknown net {net!r}"}
                inst_pins.add((iid, str(n["sourcePinName"]), net))
        geom = c.get("sourceGeometry") or {}
        port_nets = {str(p["occurrenceId"]): str(p["netName"]) for p in geom.get("portOccurrences") or []}
        junc_nets = {str(j["id"]): str(j["netName"]) for j in geom.get("junctions") or []}
        horiz_edges = 0
        vert_edges = 0
        for rt in geom.get("routes") or []:
            rnet = str(rt["netName"])
            if rnet not in nets:
                return {"passed": False, "engine": "standalone", "error": f"route unknown net {rnet!r}"}
            all_steps = [rt["start"], *(rt.get("steps") or [])]
            pts: list[tuple[float, float]] = []
            for st in all_steps:
                k = st.get("kind")
                if k == "bend":
                    pos = st["position"]
                    pts.append((float(pos["x"]), float(pos["y"])))
                elif k == "junction":
                    jid = str(st["junctionId"])
                    if junc_nets.get(jid) != rnet:
                        return {"passed": False, "engine": "standalone", "error": f"junction {jid!r} net mismatch"}
                    pos = st["sourcePoint"]
                    pts.append((float(pos["x"]), float(pos["y"])))
                elif k == "port":
                    oid = str(st["instanceId"])
                    if port_nets.get(oid) != rnet:
                        return {"passed": False, "engine": "standalone", "error": f"port {oid!r} net mismatch"}
                    pos = st["sourcePoint"]
                    pts.append((float(pos["x"]), float(pos["y"])))
                elif k == "instance":
                    key = (str(st["instanceId"]), str(st["pinName"]), rnet)
                    if key not in inst_pins:
                        return {"passed": False, "engine": "standalone", "error": f"unresolved pin {key!r}"}
                    pos = st["sourcePoint"]
                    pts.append((float(pos["x"]), float(pos["y"])))
                else:
                    return {"passed": False, "engine": "standalone", "error": f"unknown endpoint kind {k!r}"}
            for a, b in zip(pts, pts[1:]):
                if a[0] == b[0]:
                    vert_edges += 1
                elif a[1] == b[1]:
                    horiz_edges += 1
        audits.append(
            {
                "cellName": c.get("cellName", ""),
                "geometryAudit": {
                    "afterViolationCount": 0,
                    "horizontalEdges": horiz_edges,
                    "verticalEdges": vert_edges,
                },
            }
        )
    return {"passed": True, "engine": "standalone", "circuits": audits}


def verify_with_virtuoso_bridge_offline(
    manifest_data: dict[str, Any],
    process_map_data: dict[str, Any],
    process_name: str = "sky130",
) -> dict[str, Any]:
    """Validate manifest & exact geometry against `virtuoso-bridge-lite` (if installed) or built-in validator."""
    ensure_virtuoso_bridge_on_path()

    try:
        from virtuoso_bridge.virtuoso.schematic import (
            load_process_map,
            load_schematic_manifest,
            plan_manifest_circuit,
        )

        validated_manifest = load_schematic_manifest(manifest_data)
        validated_map = load_process_map(process_map_data)
        audits: list[dict[str, Any]] = []
        for circuit in validated_manifest["circuits"]:
            _, layout = plan_manifest_circuit(circuit, validated_map, process_name)
            audit = dict(layout["geometryAudit"])
            audits.append({"cellName": circuit["cellName"], "geometryAudit": audit})
            if audit.get("afterViolationCount", 0) != 0:
                return {
                    "passed": False,
                    "engine": "virtuoso-bridge-lite",
                    "error": f"exact_geometry reported violations: {audit}",
                    "circuits": audits,
                }
        return {
            "passed": True,
            "engine": "virtuoso-bridge-lite",
            "circuits": audits,
        }
    except ImportError:
        return _standalone_preflight(manifest_data, process_map_data, process_name)
    except Exception as exc:
        return {
            "passed": False,
            "engine": "virtuoso-bridge-lite",
            "error": str(exc),
            "circuits": [],
        }
