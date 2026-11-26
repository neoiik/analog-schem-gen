"""Flatten-back LVS for multi-sheet schematic books (pure manifest/netlist comparison, no layout deps)."""

from __future__ import annotations

from typing import Any

from schematic_gen.models import NetlistCircuit


def verify_hierarchy_flatten_lvs(
    original_flat: NetlistCircuit,
    combined_manifest: dict[str, Any],
    top_cell_name: str,
) -> tuple[bool, list[str]]:
    """Flatten `TOP` + child subckt manifests in memory and verify 1-to-1 isomorphism with `original_flat`."""
    by_cell = {str(c["cellName"]): c for c in combined_manifest.get("circuits", [])}
    if top_cell_name not in by_cell:
        return False, [f"Top cell {top_cell_name!r} missing from combined manifest"]

    flat_extracted: dict[str, tuple[str, dict[str, str], dict[str, str]]] = {}
    diffs: list[str] = []

    def _expand_cell(cell_name: str, net_subst: dict[str, str]) -> None:
        c_obj = by_cell[cell_name]
        for item in c_obj.get("instances", []):
            dev_cls = str(item["deviceClass"])
            node_map = {
                str(n["sourcePinName"]): net_subst.get(str(n["netName"]), str(n["netName"]))
                for n in item.get("nodes", [])
            }
            if dev_cls.startswith("subckt:"):
                child_cell = dev_cls.split(":", 1)[1]
                if child_cell not in by_cell:
                    diffs.append(f"Missing child manifest for {child_cell!r}")
                    return
                _expand_cell(child_cell, node_map)
            else:
                ref = str(item["reference"])
                if ref in flat_extracted:
                    diffs.append(f"Duplicate flattened leaf instance {ref!r}")
                params = {str(k): str(v) for k, v in (item.get("sourceParameters") or {}).items()}
                flat_extracted[ref] = (str(item.get("sourceTarget", "")), node_map, params)

    top_identity = {p.name: p.net for p in original_flat.ports}
    _expand_cell(top_cell_name, top_identity)

    orig_map = original_flat.instance_map()
    if set(flat_extracted) != set(orig_map):
        missing = sorted(set(orig_map) - set(flat_extracted))
        extra = sorted(set(flat_extracted) - set(orig_map))
        diffs.append(f"Leaf instance set mismatch: missing={missing[:10]}, extra={extra[:10]}")

    for ref, orig_inst in orig_map.items():
        if ref not in flat_extracted:
            continue
        model_got, terms_got, params_got = flat_extracted[ref]
        if model_got != orig_inst.source_model:
            diffs.append(f"{ref}: model {model_got!r} != {orig_inst.source_model!r}")
        if terms_got != orig_inst.terminals:
            diffs.append(f"{ref}: terminals {terms_got} != {orig_inst.terminals}")
        for pk, pv in orig_inst.params.items():
            if params_got.get(pk) != str(pv):
                diffs.append(f"{ref}.{pk}: {params_got.get(pk)!r} != {pv!r}")

    return len(diffs) == 0, diffs
