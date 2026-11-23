"""Read-only PDK profiler (`extract-pdk`): offline Xschem `.sym` parser & live Virtuoso OA probe."""

from __future__ import annotations

from pathlib import Path
import re
from typing import Any

from schematic_gen.manifest_builder import ensure_virtuoso_bridge_on_path


_XSCHEM_BOX_PIN_RE = re.compile(
    r"^B\s+5\s+([+-]?\d*\.?\d+)\s+([+-]?\d*\.?\d+)\s+([+-]?\d*\.?\d+)\s+([+-]?\d*\.?\d+)\s+\{([^}]*)\}"
)
_XSCHEM_LINE_RE = re.compile(
    r"^L\s+\d+\s+([+-]?\d*\.?\d+)\s+([+-]?\d*\.?\d+)\s+([+-]?\d*\.?\d+)\s+([+-]?\d*\.?\d+)"
)


def parse_xschem_sym_file(sym_path: Path | str, *, xschem_unit_per_grid: float = 10.0) -> dict[str, Any]:
    """Read-only extract normalized Virtuoso-style `pinOffsets` and `bodyBox` from an Xschem `.sym` file.

    Xschem coordinates have y increasing DOWN (typically 10 units per grid step, e.g. D=(20,-30), G=(-20,0)).
    `virtuoso-bridge-process-map-v1` stores `pinOffsets` in Virtuoso y-UP grid units!
    """
    p = Path(sym_path)
    text = p.read_text(encoding="utf-8", errors="replace")
    pins_v: dict[str, list[int]] = {}
    xs: list[float] = []
    ys_v: list[float] = []
    sym_type = "subcircuit"

    type_match = re.search(r"type\s*=\s*([A-Za-z0-9_]+)", text)
    if type_match:
        sym_type = type_match.group(1).lower()

    for raw in text.splitlines():
        line = raw.strip()
        bm = _XSCHEM_BOX_PIN_RE.match(line)
        if bm:
            x0, y0, x1, y1, attrs = (
                float(bm.group(1)),
                float(bm.group(2)),
                float(bm.group(3)),
                float(bm.group(4)),
                bm.group(5),
            )
            nm = re.search(r"name\s*=\s*([^\s}]+)", attrs)
            if nm:
                pname = nm.group(1)
                cx = int(round(((x0 + x1) / 2.0) / xschem_unit_per_grid))
                # Flip Xschem y-down to Virtuoso y-up:
                cy = int(round(-((y0 + y1) / 2.0) / xschem_unit_per_grid))
                pins_v[pname] = [cx, cy]
                xs.append(float(cx))
                ys_v.append(float(cy))
            continue
        lm = _XSCHEM_LINE_RE.match(line)
        if lm:
            lx0, ly0, lx1, ly1 = (float(lm.group(i)) / xschem_unit_per_grid for i in range(1, 5))
            xs.extend([lx0, lx1])
            ys_v.extend([-ly0, -ly1])

    if not pins_v:
        raise ValueError(f"No terminal pin boxes (`B 5 ... name=...`) found in {p}")

    bx0 = int(round(min(xs))) if xs else -2
    by0 = int(round(min(ys_v))) if ys_v else -3
    bx1 = int(round(max(xs))) if xs else 2
    by1 = int(round(max(ys_v))) if ys_v else 3

    return {
        "cell": p.stem,
        "xschemType": sym_type,
        "pinOffsets": pins_v,
        "bodyBox": [bx0, by0, bx1, by1],
    }


def extract_process_map_from_xschem_dir(
    sym_dir: Path | str,
    *,
    process_name: str = "custom_open_pdk",
    target_library: str = "open_pdk_pr",
    output_library: str = "generated_schem",
    symbol_map: dict[str, str] | None = None,
    xschem_unit_per_grid: float = 10.0,
) -> dict[str, Any]:
    """Build a `virtuoso-bridge-process-map-v1` JSON offline from a directory of `.sym` files."""
    root = Path(sym_dir)
    if not root.is_dir():
        raise FileNotFoundError(f"Symbol directory not found: {root}")

    default_candidates = symbol_map or {
        "nmos": "nfet_01v8.sym",
        "nmos_lvt": "nfet_01v8_lvt.sym",
        "nmos_hv": "nfet_g5v0d10v5.sym",
        "pmos": "pfet_01v8.sym",
        "pmos_hvt": "pfet_01v8_hvt.sym",
        "pmos_hv": "pfet_g5v0d10v5.sym",
        "res_poly_3t": "res_high_po.sym",
        "cap_mim": "cap_mim_m3_1.sym",
        "diode": "diode.sym",
    }

    devices: dict[str, Any] = {}
    for dev_key, filename in default_candidates.items():
        candidate_path = root / filename
        if not candidate_path.is_file():
            continue
        parsed = parse_xschem_sym_file(candidate_path, xschem_unit_per_grid=xschem_unit_per_grid)
        pins_v: dict[str, list[int]] = parsed["pinOffsets"]
        pin_map: dict[str, str] = {}
        for p_actual in pins_v:
            up = p_actual.upper()
            if up in {"D", "G", "S", "B", "P", "M"}:
                pin_map[up] = p_actual
            elif p_actual in {"c0", "d0", "r0"}:
                pin_map["P"] = p_actual
            elif p_actual in {"c1", "d1", "r1"}:
                pin_map["M"] = p_actual

        devices[dev_key] = {
            "library": target_library,
            "cell": parsed["cell"],
            "view": "symbol",
            "pinMap": pin_map,
            "pinOffsets": pins_v,
            "parameterMap": {"w": "w", "l": "l", "nf": "nf", "m": "m"},
            "bodyBox": parsed["bodyBox"],
        }

    if not devices:
        # Fallback: scan up to 20 `.sym` files in directory
        for sp in sorted(root.glob("*.sym"))[:20]:
            try:
                parsed = parse_xschem_sym_file(sp, xschem_unit_per_grid=xschem_unit_per_grid)
            except ValueError:
                continue
            devices[sp.stem] = {
                "library": target_library,
                "cell": parsed["cell"],
                "view": "symbol",
                "pinMap": {k: k for k in parsed["pinOffsets"]},
                "pinOffsets": parsed["pinOffsets"],
                "parameterMap": {"w": "w", "l": "l", "m": "m"},
                "bodyBox": parsed["bodyBox"],
            }

    if not devices:
        raise ValueError(f"No valid `.sym` device symbols extracted from {root}")

    return {
        "schema": "virtuoso-bridge-process-map-v1",
        "gridUnit": 0.0625,
        "sourceScaleInGridUnits": 1,
        "originInGridUnits": [16, 16],
        "localLabelStubInGridUnits": 4,
        "_metadata": {
            "extractedFrom": "xschem_symbols_offline",
            "processName": process_name,
        },
        "processes": {
            process_name: {
                "outputLibrary": output_library,
                "devices": devices,
            }
        },
    }


def extract_process_map_from_virtuoso_live(
    device_masters: dict[str, tuple[str, str]],
    *,
    process_name: str,
    output_library: str,
    grid_unit: float = 0.0625,
    profile: str | None = None,
) -> dict[str, Any]:
    """Read-only probe of live Virtuoso symbol masters & CDF parameters (requires authorized `--live`)."""
    ensure_virtuoso_bridge_on_path()
    from virtuoso_bridge import VirtuosoClient
    from virtuoso_bridge.virtuoso.ops import escape_skill_string
    from virtuoso_bridge.virtuoso.schematic.manifest import _decode_skill_output, _read_master_offsets

    client = VirtuosoClient.from_env(profile=profile)
    devices: dict[str, Any] = {}

    for dev_key, (lib, cell) in device_masters.items():
        dummy_spec = {"library": lib, "cell": cell, "view": "symbol"}
        offsets = _read_master_offsets(client, dummy_spec, grid_unit)
        # Read symbol bBox & CDF parameter names read-only
        el, ec = escape_skill_string(lib), escape_skill_string(cell)
        skill = f'''
        let((cv box ccdf pnames)
          cv = dbOpenCellViewByType("{el}" "{ec}" "symbol" "schematicSymbol" "r")
          unless(cv error("symbol not found"))
          box = cv~>bBox
          ccdf = cdfGetCellCDF(ddGetObj("{el}" "{ec}"))
          pnames = if(ccdf mapcar(lambda((p) p~>name) ccdf~>parameters) nil)
          dbClose(cv)
          sprintf(nil "%.6g|%.6g|%.6g|%.6g|%s"
            xCoord(car(box)) yCoord(car(box)) xCoord(cadr(box)) yCoord(cadr(box))
            buildString(pnames ","))
        )
        '''
        resp = client.execute_skill(skill, timeout=30)
        raw_out = _decode_skill_output(getattr(resp, "output", ""))
        parts = raw_out.split("|")
        if len(parts) == 5:
            bx0 = int(round(float(parts[0]) / grid_unit))
            by0 = int(round(float(parts[1]) / grid_unit))
            bx1 = int(round(float(parts[2]) / grid_unit))
            by1 = int(round(float(parts[3]) / grid_unit))
            cdf_names = {x.strip() for x in parts[4].split(",") if x.strip()}
        else:
            bx0, by0, bx1, by1 = -2, -3, 2, 3
            cdf_names = set()

        pmap = {k: k for k in ("w", "l", "nf", "m", "r", "c") if k in cdf_names}
        devices[dev_key] = {
            "library": lib,
            "cell": cell,
            "view": "symbol",
            "pinMap": {k.upper(): k for k in offsets},
            "pinOffsets": offsets,
            "parameterMap": pmap,
            "bodyBox": [bx0, by0, bx1, by1],
        }

    return {
        "schema": "virtuoso-bridge-process-map-v1",
        "gridUnit": grid_unit,
        "sourceScaleInGridUnits": 1,
        "originInGridUnits": [16, 16],
        "localLabelStubInGridUnits": 4,
        "processes": {
            process_name: {
                "outputLibrary": output_library,
                "devices": devices,
            }
        },
    }
