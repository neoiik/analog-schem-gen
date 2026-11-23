"""PDK process-map metrics, exact transform math, and symbol/text bounding envelopes."""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any, Mapping

from schematic_gen.models import BBox, InstanceIR, Point


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
        self.devices.update(proc_cfg.get("devices") or {})

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
        elif inst.device_class in {"res_poly_3t", "res_xhigh_po_3t", "cap_mim", "cap_mim_m4"}:
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
        if inst.device_class == "mos":
            # Gate side has horizontal gate wire; place text on opposite (channel) side.
            gx, _ = pins["G"]
            if gx < 0:
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
            out_dx = 1 if bx >= 0 else -1
            stub_end_x = bx + out_dx * (self.local_stub_len + 3)
            env_x0 = min(env_x0, stub_end_x)
            env_x1 = max(env_x1, stub_end_x)
            env_y0 = min(env_y0, by - 1)
            env_y1 = max(env_y1, by + 1)

        return label_box, (env_x0, env_y0, env_x1, env_y1), lines
