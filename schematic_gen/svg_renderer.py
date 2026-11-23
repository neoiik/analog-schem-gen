"""Publication-grade standalone SVG schematic renderer for human & multimodal LLM inspection."""

from __future__ import annotations

from collections import Counter
from html import escape
from typing import Any

from schematic_gen.models import PlacedInstance, Point, SolvedSchematic
from schematic_gen.pdk import PdkRegistry


SCALE = 12  # pixels per 0.0625-inch schematic grid unit


def _px(grid_val: float) -> float:
    return round(float(grid_val) * SCALE, 2)


def _draw_mos_svg(pl: PlacedInstance, is_pmos: bool) -> list[str]:
    """Render crisp NMOS or PMOS symbol matching exact pin coordinates."""
    gx, gy = pl.pin_points["G"]
    dx, dy = pl.pin_points["D"]
    sx, sy = pl.pin_points["S"]
    bx, by = pl.pin_points["B"]

    # Gate faces left if gx < dx else right
    sign = 1 if dx > gx else -1  # +1 when Gate is left, Channel is right
    ch_x = dx  # Channel vertical connection X
    oxide_x = pl.x  # Oxide/channel plate X
    gate_bar_x = pl.x - sign * 1  # Gate vertical bar X

    y_top = min(dy, sy)
    y_bot = max(dy, sy)
    y_finger_top = pl.y - 1.8
    y_finger_bot = pl.y + 1.8

    elems: list[str] = []
    stroke = "#162d31"

    # Gate lead + bubble (for PMOS) + vertical gate plate
    if is_pmos:
        bubble_r = 0.42
        bubble_cx = gate_bar_x - sign * bubble_r
        lead_end_x = gate_bar_x - sign * (2 * bubble_r)
        elems.append(
            f'<line x1="{_px(gx)}" y1="{_px(gy)}" x2="{_px(lead_end_x)}" y2="{_px(gy)}" stroke="{stroke}" stroke-width="1.8"/>'
        )
        elems.append(
            f'<circle cx="{_px(bubble_cx)}" cy="{_px(gy)}" r="{_px(bubble_r)}" fill="#fcfdfb" stroke="{stroke}" stroke-width="1.6"/>'
        )
    else:
        elems.append(
            f'<line x1="{_px(gx)}" y1="{_px(gy)}" x2="{_px(gate_bar_x)}" y2="{_px(gy)}" stroke="{stroke}" stroke-width="1.8"/>'
        )

    elems.append(
        f'<line x1="{_px(gate_bar_x)}" y1="{_px(pl.y - 1.9)}" x2="{_px(gate_bar_x)}" y2="{_px(pl.y + 1.9)}" stroke="{stroke}" stroke-width="2.0"/>'
    )
    # Channel plate
    elems.append(
        f'<line x1="{_px(oxide_x)}" y1="{_px(pl.y - 2.3)}" x2="{_px(oxide_x)}" y2="{_px(pl.y + 2.3)}" stroke="{stroke}" stroke-width="2.2"/>'
    )
    # Top and bottom horizontal channel fingers + vertical leads to D and S
    elems.append(
        f'<polyline points="{_px(oxide_x)},{_px(y_finger_top)} {_px(ch_x)},{_px(y_finger_top)} {_px(ch_x)},{_px(y_top)}" fill="none" stroke="{stroke}" stroke-width="1.8"/>'
    )
    elems.append(
        f'<polyline points="{_px(oxide_x)},{_px(y_finger_bot)} {_px(ch_x)},{_px(y_finger_bot)} {_px(ch_x)},{_px(y_bot)}" fill="none" stroke="{stroke}" stroke-width="1.8"/>'
    )
    # Bulk lead from channel center to B pin
    elems.append(
        f'<line x1="{_px(oxide_x)}" y1="{_px(by)}" x2="{_px(bx)}" y2="{_px(by)}" stroke="{stroke}" stroke-width="1.5"/>'
    )

    # Source arrow showing NMOS (outward on S finger) vs PMOS (inward on S finger)
    s_is_top = sy < dy
    sy_finger = y_finger_top if s_is_top else y_finger_bot
    mid_fx = (oxide_x + ch_x) / 2.0
    if not is_pmos:
        # NMOS arrow pointing from oxide_x toward ch_x on Source finger
        tip_x = ch_x
        base_x = ch_x - sign * 0.65
    else:
        # PMOS arrow pointing from ch_x toward oxide_x on Source finger
        tip_x = oxide_x
        base_x = oxide_x + sign * 0.65
    elems.append(
        f'<polygon points="{_px(tip_x)},{_px(sy_finger)} {_px(base_x)},{_px(sy_finger - 0.42)} {_px(base_x)},{_px(sy_finger + 0.42)}" fill="{stroke}"/>'
    )
    return elems


def _draw_subckt_block_svg(pl: PlacedInstance) -> list[str]:
    """Render a crisp hierarchical functional block symbol with internal pin labels."""
    bx0, by0, bx1, by1 = pl.body_box
    elems: list[str] = [
        f'<rect x="{_px(bx0)}" y="{_px(by0)}" width="{_px(bx1 - bx0)}" height="{_px(by1 - by0)}" rx="3" fill="#f5faf8" stroke="#162d31" stroke-width="2.0"/>'
    ]
    for pin_name, (px, py) in pl.pin_points.items():
        if px <= bx0:
            elems.append(f'<line x1="{_px(px)}" y1="{_px(py)}" x2="{_px(bx0)}" y2="{_px(py)}" stroke="#162d31" stroke-width="1.8"/>')
            elems.append(f'<text x="{_px(bx0) + 4}" y="{_px(py) + 3.5}" text-anchor="start" font-size="9.5" font-family="Consolas,monospace" fill="#25474d">{escape(pin_name)}</text>')
        elif px >= bx1:
            elems.append(f'<line x1="{_px(bx1)}" y1="{_px(py)}" x2="{_px(px)}" y2="{_px(py)}" stroke="#162d31" stroke-width="1.8"/>')
            elems.append(f'<text x="{_px(bx1) - 4}" y="{_px(py) + 3.5}" text-anchor="end" font-size="9.5" font-family="Consolas,monospace" fill="#25474d">{escape(pin_name)}</text>')
        elif py <= by0:
            elems.append(f'<line x1="{_px(px)}" y1="{_px(py)}" x2="{_px(px)}" y2="{_px(by0)}" stroke="#162d31" stroke-width="1.8"/>')
            elems.append(f'<text x="{_px(px)}" y="{_px(by0) + 10}" text-anchor="middle" font-size="9" font-family="Consolas,monospace" fill="#455a64">{escape(pin_name)}</text>')
        else:
            elems.append(f'<line x1="{_px(px)}" y1="{_px(by1)}" x2="{_px(px)}" y2="{_px(py)}" stroke="#162d31" stroke-width="1.8"/>')
            elems.append(f'<text x="{_px(px)}" y="{_px(by1) - 3}" text-anchor="middle" font-size="9" font-family="Consolas,monospace" fill="#455a64">{escape(pin_name)}</text>')
    return elems


def _draw_passive_or_source_svg(pl: PlacedInstance) -> list[str]:
    """Render resistor, MIM capacitor, diode, or current/voltage source."""
    stroke = "#162d31"
    elems: list[str] = []
    p0 = pl.pin_points.get("P", (pl.x, pl.y - 3))
    p1 = pl.pin_points.get("M", (pl.x, pl.y + 3))
    is_horiz = p0[1] == p1[1]

    if pl.device_class.startswith("res"):
        if not is_horiz:
            y_a, y_b = min(p0[1], p1[1]), max(p0[1], p1[1])
            elems.append(f'<line x1="{_px(pl.x)}" y1="{_px(y_a)}" x2="{_px(pl.x)}" y2="{_px(pl.y - 2.0)}" stroke="{stroke}" stroke-width="1.8"/>')
            elems.append(f'<line x1="{_px(pl.x)}" y1="{_px(pl.y + 2.0)}" x2="{_px(pl.x)}" y2="{_px(y_b)}" stroke="{stroke}" stroke-width="1.8"/>')
            zig = [
                (pl.x, pl.y - 2.0),
                (pl.x + 0.85, pl.y - 1.5),
                (pl.x - 0.85, pl.y - 0.75),
                (pl.x + 0.85, pl.y),
                (pl.x - 0.85, pl.y + 0.75),
                (pl.x + 0.85, pl.y + 1.5),
                (pl.x, pl.y + 2.0),
            ]
        else:
            x_a, x_b = min(p0[0], p1[0]), max(p0[0], p1[0])
            elems.append(f'<line x1="{_px(x_a)}" y1="{_px(pl.y)}" x2="{_px(pl.x - 2.0)}" y2="{_px(pl.y)}" stroke="{stroke}" stroke-width="1.8"/>')
            elems.append(f'<line x1="{_px(pl.x + 2.0)}" y1="{_px(pl.y)}" x2="{_px(x_b)}" y2="{_px(pl.y)}" stroke="{stroke}" stroke-width="1.8"/>')
            zig = [
                (pl.x - 2.0, pl.y),
                (pl.x - 1.5, pl.y - 0.85),
                (pl.x - 0.75, pl.y + 0.85),
                (pl.x, pl.y - 0.85),
                (pl.x + 0.75, pl.y + 0.85),
                (pl.x + 1.5, pl.y - 0.85),
                (pl.x + 2.0, pl.y),
            ]
        pts_str = " ".join(f"{_px(x)},{_px(y)}" for x, y in zig)
        elems.append(f'<polyline points="{pts_str}" fill="none" stroke="{stroke}" stroke-width="1.8"/>')
        if "B" in pl.pin_points:
            bx, by = pl.pin_points["B"]
            elems.append(
                f'<line x1="{_px(bx)}" y1="{_px(by)}" x2="{_px(pl.x)}" y2="{_px(pl.y)}" stroke="#5c7075" stroke-width="1.3" stroke-dasharray="3,2"/>'
            )
        return elems

    if pl.device_class.startswith("cap"):
        if not is_horiz:
            y_a, y_b = min(p0[1], p1[1]), max(p0[1], p1[1])
            elems.append(f'<line x1="{_px(pl.x)}" y1="{_px(y_a)}" x2="{_px(pl.x)}" y2="{_px(pl.y - 0.45)}" stroke="{stroke}" stroke-width="1.8"/>')
            elems.append(f'<line x1="{_px(pl.x)}" y1="{_px(pl.y + 0.45)}" x2="{_px(pl.x)}" y2="{_px(y_b)}" stroke="{stroke}" stroke-width="1.8"/>')
            elems.append(f'<line x1="{_px(pl.x - 1.4)}" y1="{_px(pl.y - 0.45)}" x2="{_px(pl.x + 1.4)}" y2="{_px(pl.y - 0.45)}" stroke="{stroke}" stroke-width="2.2"/>')
            elems.append(f'<line x1="{_px(pl.x - 1.4)}" y1="{_px(pl.y + 0.45)}" x2="{_px(pl.x + 1.4)}" y2="{_px(pl.y + 0.45)}" stroke="{stroke}" stroke-width="2.2"/>')
        else:
            x_a, x_b = min(p0[0], p1[0]), max(p0[0], p1[0])
            elems.append(f'<line x1="{_px(x_a)}" y1="{_px(pl.y)}" x2="{_px(pl.x - 0.45)}" y2="{_px(pl.y)}" stroke="{stroke}" stroke-width="1.8"/>')
            elems.append(f'<line x1="{_px(pl.x + 0.45)}" y1="{_px(pl.y)}" x2="{_px(x_b)}" y2="{_px(pl.y)}" stroke="{stroke}" stroke-width="1.8"/>')
            elems.append(f'<line x1="{_px(pl.x - 0.45)}" y1="{_px(pl.y - 1.4)}" x2="{_px(pl.x - 0.45)}" y2="{_px(pl.y + 1.4)}" stroke="{stroke}" stroke-width="2.2"/>')
            elems.append(f'<line x1="{_px(pl.x + 0.45)}" y1="{_px(pl.y - 1.4)}" x2="{_px(pl.x + 0.45)}" y2="{_px(pl.y + 1.4)}" stroke="{stroke}" stroke-width="2.2"/>')
        return elems

    if pl.device_class in {"isource", "vsource"}:
        y_a, y_b = min(p0[1], p1[1]), max(p0[1], p1[1])
        r = 1.6
        elems.append(f'<line x1="{_px(pl.x)}" y1="{_px(y_a)}" x2="{_px(pl.x)}" y2="{_px(pl.y - r)}" stroke="{stroke}" stroke-width="1.8"/>')
        elems.append(f'<line x1="{_px(pl.x)}" y1="{_px(pl.y + r)}" x2="{_px(pl.x)}" y2="{_px(y_b)}" stroke="{stroke}" stroke-width="1.8"/>')
        elems.append(f'<circle cx="{_px(pl.x)}" cy="{_px(pl.y)}" r="{_px(r)}" fill="#fcfdfb" stroke="{stroke}" stroke-width="1.8"/>')
        if pl.device_class == "isource":
            dir_y = 1 if p1[1] > p0[1] else -1
            elems.append(f'<line x1="{_px(pl.x)}" y1="{_px(pl.y - 0.9 * dir_y)}" x2="{_px(pl.x)}" y2="{_px(pl.y + 0.85 * dir_y)}" stroke="{stroke}" stroke-width="1.7"/>')
            elems.append(
                f'<polygon points="{_px(pl.x)},{_px(pl.y + 1.05 * dir_y)} {_px(pl.x - 0.45)},{_px(pl.y + 0.35 * dir_y)} {_px(pl.x + 0.45)},{_px(pl.y + 0.35 * dir_y)}" fill="{stroke}"/>'
            )
        else:
            elems.append(f'<text x="{_px(pl.x)}" y="{_px(pl.y - 0.25)}" text-anchor="middle" font-size="11" font-family="Consolas,monospace" fill="{stroke}">+</text>')
            elems.append(f'<text x="{_px(pl.x)}" y="{_px(pl.y + 1.0)}" text-anchor="middle" font-size="11" font-family="Consolas,monospace" fill="{stroke}">-</text>')
        return elems

    # Diode fallback
    y_a, y_b = min(p0[1], p1[1]), max(p0[1], p1[1])
    elems.append(f'<line x1="{_px(pl.x)}" y1="{_px(y_a)}" x2="{_px(pl.x)}" y2="{_px(y_b)}" stroke="{stroke}" stroke-width="1.6"/>')
    elems.append(
        f'<polygon points="{_px(pl.x - 1.1)},{_px(pl.y - 0.9)} {_px(pl.x + 1.1)},{_px(pl.y - 0.9)} {_px(pl.x)},{_px(pl.y + 0.9)}" fill="#fcfdfb" stroke="{stroke}" stroke-width="1.8"/>'
    )
    elems.append(f'<line x1="{_px(pl.x - 1.1)}" y1="{_px(pl.y + 0.9)}" x2="{_px(pl.x + 1.1)}" y2="{_px(pl.y + 0.9)}" stroke="{stroke}" stroke-width="2.0"/>')
    return elems


def render_schematic_svg(
    solved: SolvedSchematic,
    pdk: PdkRegistry | None = None,
    *,
    show_groups: bool = True,
    show_debug_boxes: bool = False,
) -> str:
    """Render a standalone, textbook-style vector schematic SVG."""
    pdk = pdk or PdkRegistry()
    bx0, by0, bx1, by1 = solved.bounds
    pad = 4
    vx0 = _px(bx0 - pad)
    vy0 = _px(by0 - pad)
    vw = _px((bx1 - bx0) + 2 * pad)
    vh = _px((by1 - by0) + 2 * pad)

    supplies = set(solved.circuit.supply_nets)
    grounds = set(solved.circuit.ground_nets)
    rails = supplies | grounds

    out: list[str] = [
        f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="{vx0} {vy0} {vw} {vh}" width="{int(vw)}" height="{int(vh)}" font-family="Segoe UI, Microsoft YaHei, sans-serif">',
        f'<rect x="{vx0}" y="{vy0}" width="{vw}" height="{vh}" fill="#fcfdfb"/>',
    ]

    # Subtle header caption
    out.append(
        f'<text x="{vx0 + 16}" y="{vy0 + 22}" font-size="13" font-weight="600" fill="#2c4a4e">'
        f'{escape(solved.circuit.cell_name)} · {escape(pdk.process_name.upper())} Schematic (gridUnit = {pdk.grid_unit:g}")</text>'
    )

    # 1. Optional functional group dashed frames (skip overlapping boxes when 2D placement interleaves groups)
    if show_groups:
        grp_boxes: list[tuple[str, float, float, float, float]] = []
        for grp in solved.intent.groups:
            members = [solved.placements[r] for r in grp.instances if r in solved.placements]
            if not members:
                continue
            gx0 = min(m.full_envelope[0] for m in members) - 2
            gy0 = min(m.full_envelope[1] for m in members) - 2.5
            gx1 = max(m.full_envelope[2] for m in members) + 2
            gy1 = max(m.full_envelope[3] for m in members) + 2
            grp_boxes.append((grp.label, gx0, gy0, gx1, gy1))
        for idx_g, (glabel, gx0, gy0, gx1, gy1) in enumerate(grp_boxes):
            overlaps_other = any(
                idx_o != idx_g
                and max(gx0, ox0) < min(gx1, ox1) - 0.5
                and max(gy0, oy0) < min(gy1, oy1) - 0.5
                for idx_o, (_, ox0, oy0, ox1, oy1) in enumerate(grp_boxes)
            )
            if overlaps_other:
                continue
            out.append(
                f'<rect x="{_px(gx0)}" y="{_px(gy0)}" width="{_px(gx1 - gx0)}" height="{_px(gy1 - gy0)}" '
                f'rx="6" fill="#f3f8f6" fill-opacity="0.55" stroke="#b9d0cb" stroke-width="1" stroke-dasharray="5,4"/>'
            )
            out.append(
                f'<text x="{_px(gx0) + 6}" y="{_px(gy0) + 12}" font-size="10" fill="#527370">{escape(glabel)}</text>'
            )

    # 2. Wires (color-coded by semantic role)
    endpoint_degree: Counter[tuple[str, Point]] = Counter()
    for rt in solved.routes:
        pts = rt.points()
        if len(pts) < 2:
            continue
        net = rt.net_name
        pol = solved.intent.net_policies.get(net, "full_wire")
        if net in rails:
            w_color = "#546e7a"
        elif pol == "perimeter_bus":
            w_color = "#9a5b13"
        elif net in solved.intent.critical_nets:
            w_color = "#00695c"
        else:
            w_color = "#1f4e78"
        pts_attr = " ".join(f"{_px(x)},{_px(y)}" for x, y in pts)
        out.append(
            f'<polyline points="{pts_attr}" fill="none" stroke="{w_color}" stroke-width="1.8" stroke-linecap="round" stroke-linejoin="round"/>'
        )
        endpoint_degree[(net, pts[0])] += 1
        endpoint_degree[(net, pts[-1])] += 1

    # Local bulk shorts & bulk label stubs
    for bs in solved.local_bulk_shorts:
        pl = solved.placements[bs["reference"]]
        bpt = pl.pin_points[bs["bulkPinName"]]
        spt = pl.pin_points[bs["sourcePinName"]]
        out.append(
            f'<polyline points="{_px(bpt[0])},{_px(bpt[1])} {_px(spt[0])},{_px(spt[1])}" fill="none" stroke="#546e7a" stroke-width="1.5"/>'
        )
        endpoint_degree[(bs["netName"], spt)] += 1

    for bl in solved.local_bulk_labels:
        pl = solved.placements[bl["reference"]]
        ax, ay = pl.pin_points["B"]
        dx = 1 if ax >= pl.x else -1
        ex = ax + dx * pdk.local_stub_len
        out.append(
            f'<line x1="{_px(ax)}" y1="{_px(ay)}" x2="{_px(ex)}" y2="{_px(ay)}" stroke="#546e7a" stroke-width="1.4"/>'
        )
        anchor = "start" if dx > 0 else "end"
        tx = _px(ex) + (4 if dx > 0 else -4)
        out.append(
            f'<text x="{tx}" y="{_px(ay) + 3.5}" text-anchor="{anchor}" font-size="9.5" font-family="Consolas,monospace" fill="#455a64">{escape(bl["name"])}</text>'
        )

    # Wire-label fly-wire stubs (`WireLabelStubPlaced`) + Bus Rippers (`BusRipperPlaced`)
    for wl in solved.wire_label_stubs:
        out.append(
            f'<line x1="{_px(wl.x1)}" y1="{_px(wl.y1)}" x2="{_px(wl.x2)}" y2="{_px(wl.y2)}" stroke="#8d4e0f" stroke-width="1.7"/>'
        )
        if wl.y1 == wl.y2:
            go_right = wl.x2 >= wl.x1
            tx = _px(wl.x2) + (4 if go_right else -4)
            anch = "start" if go_right else "end"
            ty = _px(wl.y2) + 3.5
        else:
            tx = _px(wl.x2) + 3
            anch = "start"
            ty = _px(wl.y2) + (10 if wl.y2 > wl.y1 else -3)
        out.append(
            f'<text x="{tx}" y="{ty}" text-anchor="{anch}" font-size="10" font-weight="600" font-family="Consolas,monospace" fill="#7a3e00">{escape(wl.net_name)}</text>'
        )

    for br in solved.bus_rippers:
        by_px = _px(br.trunk_y)
        out.append(
            f'<line x1="{_px(br.x_start)}" y1="{by_px}" x2="{_px(br.x_end)}" y2="{by_px}" stroke="#103b5c" stroke-width="3.4" stroke-linecap="round"/>'
        )
        out.append(
            f'<text x="{_px(br.x_end) + 6}" y="{by_px + 4}" text-anchor="start" font-size="11" font-weight="700" font-family="Consolas,monospace" fill="#103b5c">{escape(br.bus_name)}</text>'
        )
        for t_net, tx_g, ty_g in br.taps:
            # Draw vertical tap + 45-degree ripper tick onto bus bar
            out.append(
                f'<polyline points="{_px(tx_g)},{_px(ty_g)} {_px(tx_g)},{_px(br.trunk_y - 1)} {_px(tx_g + 1)},{by_px}" fill="none" stroke="#1f4e78" stroke-width="1.6"/>'
            )
            out.append(
                f'<text x="{_px(tx_g) + 3}" y="{_px(br.trunk_y - 1.4)}" text-anchor="start" font-size="9" font-family="Consolas,monospace" fill="#1f4e78">{escape(t_net)}</text>'
            )

    # 3. Symbols + Parameter Labels
    for pl in solved.placements.values():
        if show_debug_boxes:
            bx0, by0, bx1, by1 = pl.body_box
            lx0, ly0, lx1, ly1 = pl.label_box
            out.append(
                f'<rect x="{_px(bx0)}" y="{_px(by0)}" width="{_px(bx1 - bx0)}" height="{_px(by1 - by0)}" fill="none" stroke="#e57373" stroke-dasharray="2,2"/>'
            )
            out.append(
                f'<rect x="{_px(lx0)}" y="{_px(ly0)}" width="{_px(lx1 - lx0)}" height="{_px(ly1 - ly0)}" fill="none" stroke="#64b5f6" stroke-dasharray="2,2"/>'
            )

        if pl.device_class == "subckt":
            out.extend(_draw_subckt_block_svg(pl))
        elif pl.device_class == "mos":
            out.extend(_draw_mos_svg(pl, is_pmos=(pl.polarity == "pmos")))
        else:
            out.extend(_draw_passive_or_source_svg(pl))

        # Parameter text inside `pl.label_box`
        lx0, ly0, lx1, ly1 = pl.label_box
        align_right = lx1 <= pl.x
        tx = _px(lx1) - 2 if align_right else _px(lx0) + 2
        anchor = "end" if align_right else "start"
        if pl.device_class == "subckt" or pl.cadence_orient in {"R90", "R270", "MXR90", "MYR90"}:
            tx = _px((lx0 + lx1) / 2.0)
            anchor = "middle"
        base_y = _px(ly0) + 12
        for line_i, line_txt in enumerate(pl.label_lines):
            weight = "600" if line_i == 0 else "400"
            color = "#102a2e" if line_i == 0 else "#3b595d"
            out.append(
                f'<text x="{tx}" y="{base_y + line_i * 12}" text-anchor="{anchor}" font-size="10.5" font-weight="{weight}" font-family="Consolas,monospace" fill="{color}">{escape(line_txt)}</text>'
            )

    # 4. Port Symbols (VDD bar, GND triangle, Signal I/O flag)
    for po in solved.ports:
        px, py = _px(po.x), _px(po.y)
        if po.symbol_id == "vdd-port":
            out.append(f'<line x1="{px - 9}" y1="{py}" x2="{px + 9}" y2="{py}" stroke="#37474f" stroke-width="2.3"/>')
            out.append(
                f'<text x="{px}" y="{py - 5}" text-anchor="middle" font-size="10" font-weight="600" font-family="Consolas,monospace" fill="#37474f">{escape(po.port_name)}</text>'
            )
        elif po.symbol_id == "ground":
            out.append(
                f'<polygon points="{px - 8},{py} {px + 8},{py} {px},{py + 9}" fill="none" stroke="#37474f" stroke-width="1.8"/>'
            )
        else:
            is_right = po.mirror == "horizontal"
            dir_s = 1 if is_right else -1
            flag = f"{px},{py} {px + dir_s * 7},{py - 5} {px + dir_s * 18},{py - 5} {px + dir_s * 18},{py + 5} {px + dir_s * 7},{py + 5}"
            out.append(f'<polygon points="{flag}" fill="#e6f2f0" stroke="#00695c" stroke-width="1.5"/>')
            tx = px + dir_s * 22
            anchor = "start" if is_right else "end"
            out.append(
                f'<text x="{tx}" y="{py + 4}" text-anchor="{anchor}" font-size="11" font-weight="600" font-family="Consolas,monospace" fill="#004d40">{escape(po.port_name)}</text>'
            )

    # 5. Explicit Solder Dots at Junctions & Multi-Wire Pin Joins
    solder_pts: set[Point] = {(j.x, j.y) for j in solved.junctions.values()}
    for (_net, pt), deg in endpoint_degree.items():
        # Pin with >= 2 incident wire branches (plus the device terminal itself = 3-way T-join!)
        if deg >= 2:
            solder_pts.add(pt)
    for jx, jy in sorted(solder_pts):
        out.append(f'<circle cx="{_px(jx)}" cy="{_px(jy)}" r="3.3" fill="#0d2629"/>')

    out.append("</svg>")
    return "\n".join(out)
