"""PDK process-map metrics, exact transform math, and symbol/text bounding envelopes."""

from __future__ import annotations

import json
import math
from pathlib import Path
import re
from typing import Any, Mapping

from schematic_gen.models import BBox, InstanceIR, NetlistCircuit, Point


HERE = Path(__file__).resolve().parent.parent
DEFAULT_PROCESS_MAP_PATH = HERE / "pdk" / "sky130_process_map.json"
DEFAULT_NETLIST_RULES_PATH = HERE / "pdk" / "sky130_netlist_rules.json"

# Exact match with virtuoso_bridge.virtuoso.schematic.exact_geometry._ORIENTATION_BY_SOURCE
ORIENTATION_BY_SOURCE: dict[tuple[int, str], str] = {
    (0, "none"): "R0",
    (0, "horizontal"): "MY",
    (0, "vertical"): "MX",
    (0, "both"): "R180",
    (90, "none"): "R270",
    (90, "horizontal"): "MYR90",
    (90, "vertical"): "MXR90",
    (90, "both"): "R90",
    (180, "none"): "R180",
    (180, "horizontal"): "MX",
    (180, "vertical"): "MY",
    (180, "both"): "R0",
    (270, "none"): "R90",
    (270, "horizontal"): "MXR90",
    (270, "vertical"): "MYR90",
    (270, "both"): "R270",
}

SEMANTIC_TRANSFORMS: dict[str, tuple[int, str]] = {
    "gate_left": (0, "none"),          # R0: Gate at left, Drain/Source on right
    "gate_right": (0, "horizontal"),   # MY: Gate at right, Drain/Source on left
    "gate_left_flipped": (0, "vertical"),    # MX: Gate at left, vertical flip
    "gate_right_flipped": (0, "both"),       # R180: Gate at right, vertical flip
    "horizontal_gate_down": (270, "none"),           # R90: horizontal MOS channel, Gate down (PMOS S-left/D-right)
    "horizontal_gate_down_flipped": (270, "horizontal"),  # MXR90: horizontal MOS channel, Gate down (PMOS D-left/S-right)
    "horizontal_gate_up": (90, "none"),              # R270: horizontal MOS channel, Gate up (PMOS D-left/S-right)
    "horizontal_gate_up_flipped": (270, "vertical"), # MYR90: horizontal MOS channel, Gate up (PMOS S-left/D-right)
    "vertical": (0, "none"),           # R0: P at top, M at bottom
    "p_top": (0, "none"),
    "vertical_flipped": (0, "vertical"),     # MX: M at top, P at bottom
    "m_top": (0, "vertical"),
    "horizontal_l2r": (270, "none"),   # Cadence R90: P at left, M at right
    "p_left": (270, "none"),
    "horizontal_r2l": (90, "none"),    # Cadence R270: P at right, M at left
    "m_left": (90, "none"),
    "R0": (0, "none"),
    "MY": (0, "horizontal"),
    "MX": (0, "vertical"),
    "R180": (180, "none"),
    "R90": (270, "none"),
    "R270": (90, "none"),
}


def source_orientation(rotation: int, mirror: str = "none") -> str:
    """Translate source (rotation, mirror) to Cadence orientation string."""
    key = (int(rotation) % 360, str(mirror))
    if key not in ORIENTATION_BY_SOURCE:
        raise ValueError(f"Unsupported source transform: {key!r}")
    return ORIENTATION_BY_SOURCE[key]


def orient_offset_virtuoso(offset: tuple[float, float], cadence_orient: str) -> tuple[int, int]:
    """Apply Cadence orientation to Virtuoso (x-right, y-up) pin offset."""
    x, y = float(offset[0]), float(offset[1])
    transforms = {
        "R0": (x, y),
        "R90": (-y, x),
        "R180": (-x, -y),
        "R270": (y, -x),
        "MY": (-x, y),
        "MX": (x, -y),
        "MXR90": (y, x),
        "MYR90": (-y, -x),
    }
    if cadence_orient not in transforms:
        raise ValueError(f"Unsupported Cadence orientation: {cadence_orient!r}")
    tx, ty = transforms[cadence_orient]
    return int(round(tx)), int(round(ty))


def semantic_to_transform(semantic: str) -> tuple[int, str, str]:
    """Return (rotation, mirror, cadence_orient) for a semantic orientation."""
    if semantic not in SEMANTIC_TRANSFORMS:
        raise ValueError(
            f"Unknown semantic orientation {semantic!r}; choose from {sorted(SEMANTIC_TRANSFORMS)}"
        )
    rot, mir = SEMANTIC_TRANSFORMS[semantic]
    return rot, mir, source_orientation(rot, mir)


class PdkRegistry:
    """Target PDK symbol metrics and process-map registry."""

    def __init__(
        self,
        process_map_path: Path | str = DEFAULT_PROCESS_MAP_PATH,
        process_name: str = "sky130",
    ) -> None:
        self.process_map_path = Path(process_map_path)
        self.raw_map: dict[str, Any] = json.loads(
            self.process_map_path.read_text(encoding="utf-8")
        )
        self.process_name = process_name
        self.grid_unit = float(self.raw_map.get("gridUnit", 0.0625))
        self.local_stub_len = int(round(float(self.raw_map.get("localLabelStubInGridUnits", 4))))

        processes = self.raw_map.get("processes", {})
        if process_name not in processes:
            raise ValueError(f"Process {process_name!r} not in {sorted(processes)}")
        self.devices: dict[str, dict[str, Any]] = {}
        self.devices.update(self.raw_map.get("sharedDevices") or {})
        proc_cfg = processes[process_name]
        self.output_library = str(proc_cfg.get("outputLibrary", "generated_schem"))
        self.devices.update(proc_cfg.get("devices") or {})

    def register_block_symbol(
        self,
        cell_name: str,
        *,
        left_pins: list[str],
        right_pins: list[str],
        top_pins: list[str],
        bottom_pins: list[str],
        pin_pitch: int = 4,
    ) -> dict[str, Any]:
        """Synthesize a clean integer-grid rectangular symbol spec for a hierarchical subcircuit."""
        n_side = max(len(left_pins), len(right_pins), 1)
        n_tb = max(len(top_pins), len(bottom_pins), 1)
        half_w = max(5, ((n_tb + 1) * pin_pitch) // 2)
        half_h = max(4, ((n_side + 1) * pin_pitch) // 2)

        pin_offsets: dict[str, list[int]] = {}
        pin_map: dict[str, str] = {}

        def _spread(count: int, step: int) -> list[int]:
            if count <= 0:
                return []
            start = ((count - 1) * step) // 2
            return [start - idx * step for idx in range(count)]

        for p, vy in zip(left_pins, _spread(len(left_pins), pin_pitch)):
            pin_offsets[p] = [-half_w, vy]
            pin_map[p] = p
        for p, vy in zip(right_pins, _spread(len(right_pins), pin_pitch)):
            pin_offsets[p] = [half_w, vy]
            pin_map[p] = p
        for p, vx in zip(top_pins, list(reversed(_spread(len(top_pins), pin_pitch)))):
            pin_offsets[p] = [vx, half_h]
            pin_map[p] = p
        for p, vx in zip(bottom_pins, list(reversed(_spread(len(bottom_pins), pin_pitch)))):
            pin_offsets[p] = [vx, -half_h]
            pin_map[p] = p

        dev_key = f"subckt:{cell_name}"
        spec: dict[str, Any] = {
            "library": self.output_library,
            "cell": cell_name,
            "view": "symbol",
            "pinMap": pin_map,
            "pinOffsets": pin_offsets,
            "parameterMap": {},
            "bodyBox": [-(half_w - 1), -(half_h - 1), half_w - 1, half_h - 1],
            "defaultLabelSide": "top",
        }
        self.devices[dev_key] = spec
        self.raw_map["processes"][self.process_name].setdefault("devices", {})[dev_key] = spec
        return spec

    def ensure_subckt_symbols(
        self,
        circuit: NetlistCircuit,
        *,
        rules_path: Path | str = DEFAULT_NETLIST_RULES_PATH,
    ) -> list[str]:
        """Auto-register clean block symbols for any black-box `subckt` instances missing from this PDK map."""
        supply_pats = [r"^VDD$", r"^AVDD$", r"^DVDD$", r"^VPWR$", r"^VCCA$", r"^VCC$"]
        ground_pats = [r"^VSS$", r"^AVSS$", r"^DVSS$", r"^VGND$", r"^GND$", r"^0$", r"^VSSA$"]
        rp = Path(rules_path)
        if rp.is_file():
            rdata = json.loads(rp.read_text(encoding="utf-8"))
            supply_pats = list(rdata.get("supplyNetPatterns") or supply_pats)
            ground_pats = list(rdata.get("groundNetPatterns") or ground_pats)
        sup_res = [re.compile(p, re.IGNORECASE) for p in supply_pats]
        gnd_res = [re.compile(p, re.IGNORECASE) for p in ground_pats]
        out_prefixes = ("VOUT", "OUT", "VOP", "VON", "VOM", "IOUT", "Q")

        registered: list[str] = []
        for inst in circuit.instances:
            if inst.device_class != "subckt":
                continue
            dev_key = inst.device_key
            if dev_key in self.devices:
                continue
            cell_name = inst.subckt_cell or inst.source_model
            left_pins: list[str] = []
            right_pins: list[str] = []
            top_pins: list[str] = []
            bottom_pins: list[str] = []
            for pname in inst.terminals:
                if any(rx.match(pname) for rx in sup_res):
                    top_pins.append(pname)
                elif any(rx.match(pname) for rx in gnd_res):
                    bottom_pins.append(pname)
                elif any(pname.upper().startswith(pfx) for pfx in out_prefixes):
                    right_pins.append(pname)
                else:
                    left_pins.append(pname)
            self.register_block_symbol(
                cell_name,
                left_pins=left_pins,
                right_pins=right_pins,
                top_pins=top_pins,
                bottom_pins=bottom_pins,
            )
            registered.append(cell_name)
        return registered

    def device_spec(self, device_key: str) -> dict[str, Any]:
        if device_key not in self.devices:
            raise KeyError(
                f"Device key {device_key!r} missing from process {self.process_name!r}; "
                f"available: {sorted(self.devices)}"
            )
        return self.devices[device_key]

    def target_pin_name(self, device_key: str, canonical_pin: str) -> str:
        spec = self.device_spec(device_key)
        pin_map = spec.get("pinMap") or {}
        return str(pin_map.get(canonical_pin, canonical_pin))

    def source_pin_offsets(
        self,
        device_key: str,
        rotation: int,
        mirror: str,
    ) -> dict[str, Point]:
        """Return canonical_pin -> (dx, dy) in source y-down integer grid units."""
        spec = self.device_spec(device_key)
        cadence_orient = source_orientation(rotation, mirror)
        pin_map: Mapping[str, str] = spec.get("pinMap") or {}
        offsets: Mapping[str, list[float]] = spec["pinOffsets"]

        # Map canonical pins (and direct target pins if unmapped)
        result: dict[str, Point] = {}
        if pin_map:
            for canon_pin, target_pin in pin_map.items():
                raw = offsets[target_pin]
                tx, ty = orient_offset_virtuoso((raw[0], raw[1]), cadence_orient)
                # Source coordinates have y increasing DOWN, Virtuoso has y UP:
                result[canon_pin] = (tx, -ty)
        else:
            for pin_name, raw in offsets.items():
                tx, ty = orient_offset_virtuoso((raw[0], raw[1]), cadence_orient)
                result[pin_name] = (tx, -ty)
        return result

    def source_body_box(
        self,
        device_key: str,
        rotation: int,
        mirror: str,
    ) -> BBox:
        """Return relative symbol body bounding box (bx0, by0, bx1, by1) in y-down coords."""
        spec = self.device_spec(device_key)
        cadence_orient = source_orientation(rotation, mirror)
        raw_box = spec.get("bodyBox", [-2, -3, 2, 3])
        corners_v = [
            (raw_box[0], raw_box[1]),
            (raw_box[0], raw_box[3]),
            (raw_box[2], raw_box[1]),
            (raw_box[2], raw_box[3]),
        ]
        transformed = [orient_offset_virtuoso(pt, cadence_orient) for pt in corners_v]
        xs = [pt[0] for pt in transformed]
        ys = [-pt[1] for pt in transformed]
        return (min(xs), min(ys), max(xs), max(ys))

    def format_label_lines(self, inst: InstanceIR) -> list[str]:
        """Format compact human-readable instance & parameter text lines."""
        lines = [inst.ref]
        p = inst.params
        if inst.device_class == "subckt":
            return [inst.ref, inst.subckt_cell or inst.source_model]
        if inst.device_class == "mos":
            w = p.get("w", "")
            l = p.get("l", "")
            nf = p.get("nf", "1")
            m = p.get("m", "1")
            if w and l:
                wl_str = f"{w}/{l}"
                extras = []
                if nf and str(nf) != "1":
                    extras.append(f"nf={nf}")
                if m and str(m) != "1":
                    extras.append(f"m={m}")
                if extras:
                    wl_str += " " + ",".join(extras)
                lines.append(wl_str)
        elif inst.device_class == "bjt":
            m = p.get("m", "1")
            if m and str(m) != "1":
                lines.append(f"m={m}")
        elif p.get("w") and p.get("l") and inst.device_class not in {"isource", "vsource", "subckt"}:
            # Geometric passives of any PDK (poly / high-R resistors, MIM / MOM caps): show W/L instead of value
            w = p.get("w", "")
            l = p.get("l", "")
            m = p.get("mf") or p.get("m", "1")
            if w and l:
                s = f"{w}/{l}"
                if m and str(m) != "1":
                    s += f" m={m}"
                lines.append(s)
        else:
            val = p.get("r") or p.get("c") or p.get("dc") or ""
            if val:
                lines.append(str(val))
        return lines

    def relative_label_and_envelope(
        self,
        inst: InstanceIR,
        rotation: int,
        mirror: str,
        has_bulk_label_stub: bool = False,
    ) -> tuple[BBox, BBox, list[str]]:
        """Compute relative (label_box, full_envelope, label_lines) in source y-down grid units.

        Guarantees parameter text is placed in a wire-free quadrant and accounted for
        in both QP separation inequalities and A* obstacle avoidance.
        """
        device_key = inst.device_key
        body = self.source_body_box(device_key, rotation, mirror)
        pins = self.source_pin_offsets(device_key, rotation, mirror)
        lines = self.format_label_lines(inst)

        # Estimate text width/height in 0.0625" grid units (~0.55 grid per character, min 5)
        max_chars = max((len(line) for line in lines), default=3)
        text_w = max(5, int(math.ceil(max_chars * 0.55)))
        text_h = max(3, len(lines) * 2)

        cadence_orient = source_orientation(rotation, mirror)
        if inst.device_class == "subckt":
            # Place subckt block title offset to the right of x=0 so top/bottom center supply pins never cross it!
            lx0 = 2
            ly1 = body[1] - 1
            label_box: BBox = (lx0, ly1 - text_h, lx0 + text_w, ly1)
        elif inst.device_class in {"mos", "bjt"}:
            # Gate/Base side has control wire; place text on opposite (channel/C-E) side.
            ctrl_pin = "G" if inst.device_class == "mos" else "BASE"
            gx, gy = pins[ctrl_pin]
            if gx == 0 and gy != 0:
                # Horizontal MOS (R90/R270/MXR90/MYR90): D/S run left-right, G faces down (gy>0) or up (gy<0).
                if gy > 0:
                    # Gate faces DOWN, bulk faces UP at (0, -1): place label ABOVE body, shifted right if vertical bulk stub exists
                    lx0 = 2 if has_bulk_label_stub else (-text_w // 2)
                    ly1 = body[1] - 1
                    label_box = (lx0, ly1 - text_h, lx0 + text_w, ly1)
                else:
                    # Gate faces UP, bulk faces DOWN at (0, +1): place label BELOW body, shifted right if vertical bulk stub exists
                    lx0 = 2 if has_bulk_label_stub else (-text_w // 2)
                    ly0 = body[3] + 1
                    label_box = (lx0, ly0, lx0 + text_w, ly0 + text_h)
            elif gx < 0:
                # Gate on left -> channel on right (x = +1)
                if has_bulk_label_stub:
                    # Bulk stub extends right from (1, 0) to (1 + stub, 0); place text above y=0 stub or right of body
                    lx0 = body[2] + 1
                    ly0 = body[1] - 1
                    label_box: BBox = (lx0, ly0, lx0 + text_w, min(0, ly0 + text_h))
                else:
                    lx0 = body[2] + 1
                    ly0 = -text_h // 2
                    label_box = (lx0, ly0, lx0 + text_w, ly0 + text_h)
            else:
                # Gate on right (MY) -> channel on left (x = -1)
                if has_bulk_label_stub:
                    lx1 = body[0] - 1
                    ly0 = body[1] - 1
                    label_box = (lx1 - text_w, ly0, lx1, min(0, ly0 + text_h))
                else:
                    lx1 = body[0] - 1
                    ly0 = -text_h // 2
                    label_box = (lx1 - text_w, ly0, lx1, ly0 + text_h)
        elif cadence_orient in {"R90", "R270", "MXR90", "MYR90"}:
            # Horizontal 2-terminal passive (e.g., Miller Cc/Rz): place text neatly ABOVE body
            lx0 = -text_w // 2
            ly1 = body[1] - 1
            label_box = (lx0, ly1 - text_h, lx0 + text_w, ly1)
        else:
            # Vertical 2/3-terminal passive or source: place text on the side opposite to bulk pin if any
            bx = pins["B"][0] if "B" in pins else -1
            if bx < 0:
                lx0 = body[2] + 1
                ly0 = -text_h // 2
                label_box = (lx0, ly0, lx0 + text_w, ly0 + text_h)
            else:
                lx1 = body[0] - 1
                ly0 = -text_h // 2
                label_box = (lx1 - text_w, ly0, lx1, ly0 + text_h)

        env_x0 = min(body[0], label_box[0])
        env_y0 = min(body[1], label_box[1])
        env_x1 = max(body[2], label_box[2])
        env_y1 = max(body[3], label_box[3])

        if has_bulk_label_stub and "B" in pins:
            bx, by = pins["B"]
            # Match virtuoso-bridge-lite exact_geometry 2D outward bulk-stub vector
            if abs(bx) >= abs(by):
                out_dx = 1 if bx >= 0 else -1
                stub_end_x = bx + out_dx * (self.local_stub_len + 3)
                env_x0 = min(env_x0, stub_end_x)
                env_x1 = max(env_x1, stub_end_x)
                env_y0 = min(env_y0, by - 1)
                env_y1 = max(env_y1, by + 1)
            else:
                out_dy = 1 if by >= 0 else -1
                stub_end_y = by + out_dy * (self.local_stub_len + 2)
                env_x0 = min(env_x0, bx - 2)
                env_x1 = max(env_x1, bx + 2)
                env_y0 = min(env_y0, stub_end_y)
                env_y1 = max(env_y1, stub_end_y)

        return label_box, (env_x0, env_y0, env_x1, env_y1), lines
