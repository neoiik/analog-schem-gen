"""Orthogonal Steiner router, corridor parallel-segment nudger, and closed-loop congestion detector."""

from __future__ import annotations

from collections import defaultdict
import heapq
from itertools import permutations
import math
from typing import Any

import numpy as np
from scipy.optimize import minimize

from schematic_gen.models import (
    BBox,
    BusRipperPlaced,
    JunctionPlaced,
    NetlistCircuit,
    PlacedInstance,
    Point,
    PortOccurrencePlaced,
    RoutePlaced,
    RouteStepPlaced,
    SchematicIntent,
    SolvedSchematic,
    WireLabelStubPlaced,
)
from schematic_gen.pdk import PdkRegistry
from schematic_gen.placer import place_schematic


DIRS: tuple[tuple[int, int], ...] = ((1, 0), (-1, 0), (0, 1), (0, -1))


def nudge_parallel_corridor_tracks(
    segments: list[dict[str, Any]],
    *,
    corridor_low: float,
    corridor_high: float,
    min_pitch: float = 10.0,
    snap_step: float | None = None,
) -> dict[str, Any]:
    """Push apart crowded parallel segments in one orthogonal corridor (Edge Nudging).

    Same-net segments with overlapping span share a common track ("同网共干");
    different-net overlapping segments are separated by at least ``min_pitch`` ("异网分轨")
    while minimizing quadratic displacement from their initial positions.

    Returns a dictionary containing ``tracks`` (``seg_id -> solved_coord``) and
    ``overflow`` (how much additional corridor width is required if infeasible).
    """
    if not segments:
        return {"tracks": {}, "overflow": 0.0, "feasible": True}

    # Merge same-net overlapping segments into shared track groups
    ordered = sorted(segments, key=lambda s: (float(s["pos"]), float(s["span_start"])))
    groups: list[dict[str, Any]] = []
    group_of_seg: dict[str, int] = {}

    for seg in ordered:
        sid = str(seg["id"])
        net = str(seg["net"])
        a, b = sorted((float(seg["span_start"]), float(seg["span_end"])))
        pos = float(seg["pos"])
        merged = False
        for gi, grp in enumerate(groups):
            if grp["net"] == net and abs(grp["pos"] - pos) < min_pitch * 0.6:
                if not (b < grp["a"] or a > grp["b"]):
                    grp["a"] = min(grp["a"], a)
                    grp["b"] = max(grp["b"], b)
                    grp["members"].append(sid)
                    group_of_seg[sid] = gi
                    merged = True
                    break
        if not merged:
            gi = len(groups)
            groups.append({"net": net, "a": a, "b": b, "pos": pos, "members": [sid]})
            group_of_seg[sid] = gi

    m = len(groups)
    y0 = np.array([g["pos"] for g in groups], dtype=float)

    # Build interval-overlap separation pairs between distinct nets
    sep_pairs: list[tuple[int, int]] = []
    for i in range(m):
        for j in range(i + 1, m):
            if groups[i]["net"] == groups[j]["net"]:
                continue
            # Check interval overlap along corridor axis
            if min(groups[i]["b"], groups[j]["b"]) >= max(groups[i]["a"], groups[j]["a"]):
                sep_pairs.append((i, j))

    # Longest chain length determines minimum corridor width needed
    chain_len = [0] * m
    for i in range(m):
        for j in range(i + 1, m):
            if (i, j) in sep_pairs:
                chain_len[j] = max(chain_len[j], chain_len[i] + 1)
    required_span = max(chain_len, default=0) * float(min_pitch)
    avail_span = float(corridor_high) - float(corridor_low)
    overflow = max(0.0, required_span - avail_span)
    eff_high = float(corridor_high) + overflow

    def obj(y: np.ndarray) -> float:
        return float(np.sum((y - y0) ** 2))

    def jac(y: np.ndarray) -> np.ndarray:
        return 2.0 * (y - y0)

    constraints: list[dict[str, Any]] = []
    for i, j in sep_pairs:
        constraints.append(
            {
                "type": "ineq",
                "fun": lambda y, a=i, b=j, p=float(min_pitch): y[b] - y[a] - p,
            }
        )

    bounds = [(float(corridor_low), eff_high) for _ in range(m)]
    res = minimize(obj, np.clip(y0, corridor_low, eff_high), jac=jac, bounds=bounds, constraints=constraints, method="SLSQP")
    y_sol = res.x if res.success else np.clip(y0, corridor_low, eff_high)

    if snap_step is not None and snap_step > 0:
        y_sol = np.round(y_sol / snap_step) * snap_step
        # Ensure snap didn't collapse adjacent tracks
        for i, j in sep_pairs:
            if y_sol[j] - y_sol[i] < min_pitch - 1e-9:
                y_sol[j] = y_sol[i] + min_pitch

    tracks = {
        sid: float(round(y_sol[gi], 6))
        for sid, gi in group_of_seg.items()
    }
    return {
        "tracks": tracks,
        "overflow": float(round(overflow, 6)),
        "feasible": overflow <= 1e-9,
    }


def _simplify_polyline(points: list[Point]) -> list[Point]:
    """Remove duplicate consecutive points and collinear intermediate vertices."""
    if not points:
        return []
    dedup: list[Point] = [points[0]]
    for pt in points[1:]:
        if pt != dedup[-1]:
            dedup.append(pt)
    if len(dedup) <= 2:
        return dedup
    out: list[Point] = [dedup[0]]
    for prev, cur, nxt in zip(dedup, dedup[1:], dedup[2:]):
        if (prev[0] == cur[0] == nxt[0]) or (prev[1] == cur[1] == nxt[1]):
            continue
        out.append(cur)
    out.append(dedup[-1])
    return out


class _GridOccupancy:
    """Tracks symbol bodies, text labels, pins, and routed wire segments on the integer grid."""

    def __init__(
        self,
        placements: dict[str, PlacedInstance],
        ports: list[PortOccurrencePlaced],
        local_bulk_labels: list[dict[str, str]],
        local_bulk_shorts: list[dict[str, str]],
        pdk: PdkRegistry,
        track_pitch: int = 2,
    ) -> None:
        self.placements = placements
        self.ports = ports
        self.track_pitch = max(2, int(track_pitch))

        # Strictly blocked interior symbol body points + text label boxes
        self.body_blocked: set[Point] = set()
        self.label_blocked: set[Point] = set()
        # Pin ownership: point -> net_name
        self.pin_owner: dict[Point, str] = {}
        self.pin_escape: dict[Point, Point] = {}
        self.pin_escape_owner: dict[Point, set[str]] = defaultdict(set)
        self.horiz_pin_escape_pts: dict[Point, str] = {}
        self.pin_apron_y: set[Point] = set()
        self.edge_wall_xs: set[int] = set()
        self.left_port_nets: set[str] = set()
        self.right_port_nets: set[str] = set()

        for pl in placements.values():
            bx0, by0, bx1, by1 = pl.body_box
            pin_set = set(pl.pin_points.values())
            for x in range(bx0, bx1 + 1):
                for y in range(by0, by1 + 1):
                    if (x, y) not in pin_set:
                        self.body_blocked.add((x, y))

            lx0, ly0, lx1, ly1 = pl.label_box
            for x in range(lx0, lx1 + 1):
                for y in range(ly0, ly1 + 1):
                    self.label_blocked.add((x, y))

        # Reserve local bulk label stubs & shorts
        self.wire_h: dict[Point, set[str]] = defaultdict(set)  # grid points belonging to horizontal wire of net
        self.wire_v: dict[Point, set[str]] = defaultdict(set)  # grid points belonging to vertical wire of net
        self.wire_corners: dict[Point, set[str]] = defaultdict(set)  # bends, junctions, endpoints

        for item in local_bulk_labels:
            ref = item["reference"]
            net = item["netName"]
            pl = placements[ref]
            ax, ay = pl.pin_points["B"]
            dx = 1 if ax >= pl.x else -1
            ex = ax + dx * pdk.local_stub_len
            for x in range(min(ax, ex), max(ax, ex) + 1):
                self.wire_h[(x, ay)].add(net)
            self.wire_corners[(ax, ay)].add(net)
            self.wire_corners[(ex, ay)].add(net)
            # Reserve small label box around bulk label tip
            for lx in range(ex, ex + dx * 3 + (1 if dx > 0 else -1), dx):
                self.label_blocked.add((lx, ay))

        for item in local_bulk_shorts:
            ref = item["reference"]
            net = item["netName"]
            pl = placements[ref]
            bx, by = pl.pin_points[item["bulkPinName"]]
            sx, sy = pl.pin_points[item["sourcePinName"]]
            if bx == sx:
                for y in range(min(by, sy), max(by, sy) + 1):
                    self.wire_v[(bx, y)].add(net)
            else:
                for x in range(min(bx, sx), max(bx, sx) + 1):
                    self.wire_h[(x, by)].add(net)
            self.wire_corners[(bx, by)].add(net)
            self.wire_corners[(sx, sy)].add(net)

        for pt_port in ports:
            self.register_port(pt_port)

    def register_port(self, pt_port: PortOccurrencePlaced) -> None:
        self.pin_owner[(pt_port.x, pt_port.y)] = pt_port.net_name
        self.wire_corners[(pt_port.x, pt_port.y)].add(pt_port.net_name)
        # Reserve port text area behind the port pin so wires don't overwrite port names!
        if pt_port.symbol_id == "port":
            if pt_port.rotation == 90:
                for ty in range(pt_port.y - 5, pt_port.y):
                    for tx in range(pt_port.x - 1, pt_port.x + 2):
                        self.label_blocked.add((tx, ty))
                self.pin_escape[(pt_port.x, pt_port.y)] = (pt_port.x, pt_port.y + 1)
                self.pin_escape_owner[(pt_port.x, pt_port.y + 1)].add(pt_port.net_name)
            elif pt_port.rotation == 270:
                for ty in range(pt_port.y + 1, pt_port.y + 6):
                    for tx in range(pt_port.x - 1, pt_port.x + 2):
                        self.label_blocked.add((tx, ty))
                self.pin_escape[(pt_port.x, pt_port.y)] = (pt_port.x, pt_port.y - 1)
                self.pin_escape_owner[(pt_port.x, pt_port.y - 1)].add(pt_port.net_name)
            elif pt_port.mirror == "horizontal":
                # Right-side port: text extends right of pt_port.x
                for tx in range(pt_port.x + 1, pt_port.x + 8):
                    for ty in range(pt_port.y - 1, pt_port.y + 2):
                        self.label_blocked.add((tx, ty))
                self.pin_escape[(pt_port.x, pt_port.y)] = (pt_port.x - 1, pt_port.y)
                self.pin_escape_owner[(pt_port.x - 1, pt_port.y)].add(pt_port.net_name)
            else:
                # Left-side port: text extends left of pt_port.x
                for tx in range(pt_port.x - 7, pt_port.x):
                    for ty in range(pt_port.y - 1, pt_port.y + 2):
                        self.label_blocked.add((tx, ty))
                self.pin_escape[(pt_port.x, pt_port.y)] = (pt_port.x + 1, pt_port.y)
                self.pin_escape_owner[(pt_port.x + 1, pt_port.y)].add(pt_port.net_name)
        elif pt_port.symbol_id == "vdd-port":
            for ty in range(pt_port.y - 3, pt_port.y):
                self.body_blocked.add((pt_port.x, ty))
        elif pt_port.symbol_id == "ground":
            for ty in range(pt_port.y + 1, pt_port.y + 4):
                self.body_blocked.add((pt_port.x, ty))

    def register_instance_pins(
        self, circuit: NetlistCircuit, bulk_handled: set[tuple[str, str]] | None = None
    ) -> None:
        bulk_handled = bulk_handled or set()
        for inst in circuit.instances:
            pl = self.placements[inst.ref]
            bx0, by0, bx1, by1 = pl.body_box
            for pin, net in inst.terminals.items():
                px, py = pl.pin_points[pin]
                self.pin_owner[(px, py)] = net
                self.wire_corners[(px, py)].add(net)
                # Outward escape step from symbol boundary
                if py <= by0:
                    esc = (px, py - 1)
                    if (inst.ref, pin) not in bulk_handled:
                        for ax in range(bx0 - 1, bx1 + 2):
                            self.pin_apron_y.add((ax, by0 - 1))
                            self.pin_apron_y.add((ax, by0 - 2))
                elif py >= by1:
                    esc = (px, py + 1)
                    if (inst.ref, pin) not in bulk_handled:
                        for ax in range(bx0 - 1, bx1 + 2):
                            self.pin_apron_y.add((ax, by1 + 1))
                            self.pin_apron_y.add((ax, by1 + 2))
                elif px <= bx0:
                    esc = (px - 1, py)
                    if (inst.ref, pin) not in bulk_handled:
                        self.horiz_pin_escape_pts[esc] = net
                else:
                    esc = (px + 1, py)
                    if (inst.ref, pin) not in bulk_handled:
                        self.horiz_pin_escape_pts[esc] = net
                self.pin_escape[(px, py)] = esc
                if (inst.ref, pin) not in bulk_handled and esc not in self.body_blocked:
                    self.pin_escape_owner[esc].add(net)

    def add_polyline(self, net: str, pts: list[Point]) -> None:
        if len(pts) < 2:
            return
        self.wire_corners[pts[0]].add(net)
        self.wire_corners[pts[-1]].add(net)
        for idx, (a, b) in enumerate(zip(pts, pts[1:])):
            if idx > 0:
                self.wire_corners[a].add(net)
            if a[0] == b[0]:
                x = a[0]
                for y in range(min(a[1], b[1]), max(a[1], b[1]) + 1):
                    self.wire_v[(x, y)].add(net)
            elif a[1] == b[1]:
                y = a[1]
                for x in range(min(a[0], b[0]), max(a[0], b[0]) + 1):
                    self.wire_h[(x, y)].add(net)

    def step_cost(
        self,
        net: str,
        u: Point,
        v: Point,
        prev_dir: int | None,
        new_dir: int,
        goal_set: set[Point],
        allow_tight_parallel: bool = False,
        bus_y_target: int | None = None,
    ) -> float | None:
        """Return step cost from `u` to adjacent grid point `v`, or `None` if illegal."""
        # 1. Never enter symbol bodies, text labels, or edge-port outer walls (except exact goal pin itself)
        if v not in goal_set:
            if v in self.body_blocked or v in self.label_blocked or v[0] in self.edge_wall_xs:
                return None
        owner = self.pin_owner.get(v)
        if owner is not None and owner != net:
            return None
        esc_owners = self.pin_escape_owner.get(v)
        if esc_owners and net not in esc_owners:
            return None

        # 2. Never step onto a foreign bend/junction/pin corner!
        foreign_corners = self.wire_corners.get(v, set()) - {net}
        if foreign_corners:
            return None

        is_horiz = new_dir in (0, 1)  # (1,0) or (-1,0)
        same_dir_map = self.wire_h if is_horiz else self.wire_v
        perp_dir_map = self.wire_v if is_horiz else self.wire_h

        # 3. Never overlap collinearly with a foreign segment!
        if same_dir_map.get(v, set()) - {net}:
            return None
        if same_dir_map.get(u, set()) - {net}:
            return None

        # 4. Enforce parallel track pitch (`edge nudging` spacing >= track_pitch) against foreign parallel wires!
        pitch = 1 if allow_tight_parallel else self.track_pitch
        if pitch >= 2:
            vx, vy = v
            ux, uy = u
            for offset in range(1, pitch):
                if is_horiz:
                    for ny in (vy - offset, vy + offset):
                        if (self.wire_h.get((vx, ny), set()) - {net}) and (
                            self.wire_h.get((ux, ny), set()) - {net}
                        ):
                            return None
                else:
                    for nx in (vx - offset, vx + offset):
                        if (self.wire_v.get((nx, vy), set()) - {net}) and (
                            self.wire_v.get((nx, uy), set()) - {net}
                        ):
                            return None

        # 5. Prevent bending right on top of a perpendicular foreign wire!
        u_perp_foreign = perp_dir_map.get(u, set()) - {net}
        if prev_dir is not None and prev_dir != new_dir:
            # At a bend `u`, `u` cannot sit on ANY foreign wire!
            if (self.wire_h.get(u, set()) - {net}) or (self.wire_v.get(u, set()) - {net}):
                return None

        # 6. Compute cost (reward reusing same-net trunk -> Steiner tree, row/column grid alignment, & apron clearance!)
        reusing_same_net = (net in same_dir_map.get(v, set())) and (net in same_dir_map.get(u, set()))
        cost = 0.08 if reusing_same_net else 1.0

        if prev_dir is not None and prev_dir != new_dir:
            if self.horiz_pin_escape_pts.get(u) == net:
                cost += 5.2
            else:
                cost += 6.5

        if not reusing_same_net and v not in goal_set and not allow_tight_parallel:
            if is_horiz:
                if v in self.pin_apron_y:
                    cost += 3.0
                if v[1] % 2 != 0 and u not in self.pin_owner:
                    cost += 0.05
            else:
                if v[0] % 2 != 0 and u not in self.pin_owner and self.horiz_pin_escape_pts.get(u) != net:
                    cost += 0.03

        # Perpendicular crossing over foreign wire: legal at interior point, calibrated against ~20-grid detour
        if perp_dir_map.get(v, set()) - {net}:
            if v in goal_set:
                return None
            cost += 22.0
        if u_perp_foreign and False:
            pass

        # Soft guide toward top/bottom perimeter bus Y for multi-group bias nets
        if bus_y_target is not None and is_horiz:
            cost += min(1.1, abs(v[1] - bus_y_target) * 0.045)

        return cost


def _astar_connect_to_tree(
    net: str,
    start: Point,
    tree_points: set[Point],
    occ: _GridOccupancy,
    search_bounds: BBox,
    *,
    allow_tight_parallel: bool = False,
    bus_y_target: int | None = None,
) -> list[Point] | None:
    """Find minimum-cost orthogonal path from `start` to ANY point in `tree_points`."""
    if start in tree_points:
        return [start]

    bx0, by0, bx1, by1 = search_bounds
    target_list = list(tree_points)

    def heuristic(pt: Point) -> float:
        px, py = pt
        return float(min(abs(px - tx) + abs(py - ty) for tx, ty in target_list)) * 0.95

    # State: (x, y, dir_idx_or_neg1)
    start_state = (start[0], start[1], -1)
    g_score: dict[tuple[int, int, int], float] = {start_state: 0.0}
    parent: dict[tuple[int, int, int], tuple[int, int, int] | None] = {start_state: None}
    pq: list[tuple[float, int, tuple[int, int, int]]] = [(heuristic(start), 0, start_state)]
    push_id = 0

    required_first = occ.pin_escape.get(start)

    while pq:
        _, _, cur = heapq.heappop(pq)
        cx, cy, cdir = cur
        cur_g = g_score[cur]

        if (cx, cy) in tree_points and cur != start_state:
            # Ensure we don't join at a point where a foreign perpendicular wire is crossing!
            if not ((occ.wire_h.get((cx, cy), set()) - {net}) or (occ.wire_v.get((cx, cy), set()) - {net})):
                path: list[Point] = []
                st: tuple[int, int, int] | None = cur
                while st is not None:
                    path.append((st[0], st[1]))
                    st = parent[st]
                path.reverse()
                return _simplify_polyline(path)

        for d_idx, (dx, dy) in enumerate(DIRS):
            nx, ny = cx + dx, cy + dy
            if not (bx0 <= nx <= bx1 and by0 <= ny <= by1):
                continue
            # Encourage clean pin escape on very first step if blocked otherwise
            if cur == start_state and required_first is not None and (nx, ny) != required_first:
                # Allow direct connection only if (nx, ny) is outside body_blocked
                if (nx, ny) in occ.body_blocked:
                    continue

            sc = occ.step_cost(
                net,
                (cx, cy),
                (nx, ny),
                None if cdir < 0 else cdir,
                d_idx,
                tree_points,
                allow_tight_parallel=allow_tight_parallel,
                bus_y_target=bus_y_target,
            )
            if sc is None:
                continue
            nxt_state = (nx, ny, d_idx)
            cand_g = cur_g + sc
            if cand_g + 1e-9 < g_score.get(nxt_state, float("inf")):
                g_score[nxt_state] = cand_g
                parent[nxt_state] = cur
                push_id += 1
                heapq.heappush(pq, (cand_g + heuristic((nx, ny)), push_id, nxt_state))

    return None


def _split_segment_at_point(
    routes: list[RoutePlaced],
    net: str,
    join_pt: Point,
    junction_id: str,
) -> bool:
    """If `join_pt` lies in the open interior of an existing route segment of `net`, split that route at `junction_id`."""
    jx, jy = join_pt
    for r_idx, route in enumerate(routes):
        if route.net_name != net:
            continue
        pts = route.points()
        # Check if join_pt is already start or final step
        if join_pt == pts[0] or join_pt == pts[-1]:
            return False
        for seg_i, (a, b) in enumerate(zip(pts, pts[1:])):
            on_seg = False
            if join_pt == a and seg_i > 0:
                # Join is right at an existing bend `steps[seg_i - 1]`! Convert bend into two routes meeting at junction!
                left_steps = list(route.steps[: seg_i - 1]) + [
                    RouteStepPlaced(kind="junction", x=jx, y=jy, junction_id=junction_id)
                ]
                right_start = RouteStepPlaced(kind="junction", x=jx, y=jy, junction_id=junction_id)
                right_steps = list(route.steps[seg_i:])
                routes[r_idx] = RoutePlaced(
                    id=f"{route.id}_a",
                    net_name=net,
                    start=route.start,
                    steps=left_steps,
                    presentation=route.presentation,
                )
                routes.insert(
                    r_idx + 1,
                    RoutePlaced(
                        id=f"{route.id}_b",
                        net_name=net,
                        start=right_start,
                        steps=right_steps,
                        presentation=route.presentation,
                    ),
                )
                return True
            if a[0] == b[0] == jx and min(a[1], b[1]) < jy < max(a[1], b[1]):
                on_seg = True
            elif a[1] == b[1] == jy and min(a[0], b[0]) < jx < max(a[0], b[0]):
                on_seg = True
            if on_seg:
                left_steps = list(route.steps[:seg_i]) + [
                    RouteStepPlaced(kind="junction", x=jx, y=jy, junction_id=junction_id)
                ]
                right_start = RouteStepPlaced(kind="junction", x=jx, y=jy, junction_id=junction_id)
                right_steps = list(route.steps[seg_i:])
                routes[r_idx] = RoutePlaced(
                    id=f"{route.id}_a",
                    net_name=net,
                    start=route.start,
                    steps=left_steps,
                    presentation=route.presentation,
                )
                routes.insert(
                    r_idx + 1,
                    RoutePlaced(
                        id=f"{route.id}_b",
                        net_name=net,
                        start=right_start,
                        steps=right_steps,
                        presentation=route.presentation,
                    ),
                )
                return True
    return False


def _pair_port_cross_cost(
    px: int,
    ya: int,
    txa: int,
    tya: int,
    yb: int,
    txb: int,
    tyb: int,
) -> float:
    """Analytical crossing penalty between two same-side edge-port L-routes `(ya < yb)`."""
    pen = 0.0
    # Does upper port A's vertical leg at `txa` cross lower port B's horizontal ray at `yb`?
    if min(ya, tya) <= yb <= max(ya, tya) and min(px, txb) <= txa <= max(px, txb):
        pen += 28.0
    # Does lower port B's vertical leg at `txb` cross upper port A's horizontal ray at `ya`?
    if min(yb, tyb) <= ya <= max(yb, tyb) and min(px, txa) <= txb <= max(px, txa):
        pen += 28.0
    return pen


def _optimize_side_edge_ports(
    side_ports: list[PortOccurrencePlaced],
    *,
    go_right: bool,
    tree_points_by_net: dict[str, set[Point]],
    occ: _GridOccupancy,
    y_lo: int,
    y_hi: int,
    x_limit: int,
) -> list[tuple[PortOccurrencePlaced, int, int, int]]:
    """Choose optimal vertical ordering and integer Y coordinates for left/right edge ports.

    Snaps ports onto 0-bend straight horizontal rays onto internal Steiner trunks whenever
    unobstructed, and nests 1-bend L-routes (river routing) to avoid hook-arounds and crossings.
    Returns ordered list of `(port_occ, kind_rank, ref_tx, ref_ty)` ready for registration & routing.
    """
    k = len(side_ports)
    if k == 0:
        return []

    px = side_ports[0].x
    dx = 1 if go_right else -1
    dir_h = 0 if go_right else 1
    span_needed = 6 * k + 14
    if y_hi - y_lo < span_needed:
        extra = (span_needed - (y_hi - y_lo)) // 2 + 2
        y_lo -= extra
        y_hi += extra

    # Precompute candidate `(y, u_cost, kind_rank, ref_tx, ref_ty)` per port
    cands_per_port: list[list[tuple[int, float, int, int, int]]] = []
    init_sort_keys: list[tuple[float, float, int]] = []

    for idx, po in enumerate(side_ports):
        net = po.net_name
        t_pts = tree_points_by_net.get(net, set())
        raw_cands: list[tuple[int, float, int, int, int]] = []

        if not t_pts:
            for y in range(y_lo, y_hi + 1):
                if (px, y) in occ.pin_owner or (px + dx, y) in occ.body_blocked:
                    continue
                raw_cands.append((y, abs(y - po.y) * 0.8, 2, px + dx * 10, y))
        else:
            sorted_t = sorted(t_pts, key=lambda pt: (pt[0] if go_right else -pt[0], pt[1]))
            nearest_tx = sorted_t[0][0]
            # Representative facing anchor points (closest vertical trunk(s) + distinct X columns)
            anchor_pts: list[Point] = []
            seen_cols: set[int] = set()
            for tx, ty in sorted_t:
                if abs(tx - nearest_tx) <= 32 or len(seen_cols) < 4:
                    if tx not in seen_cols or len(anchor_pts) < 8:
                        anchor_pts.append((tx, ty))
                        seen_cols.add(tx)
                if len(anchor_pts) >= 10:
                    break
            t_ys = [pt[1] for pt in t_pts]
            t_y_min, t_y_max = min(t_ys), max(t_ys)
            # Find horizontal trunk Ys of `net` so edge ports align collinearly with existing horizontal bars!
            horiz_ys = [pt[1] for pt, owners in occ.wire_h.items() if net in owners]
            t_y_pref = (
                (max(set(horiz_ys), key=horiz_ys.count))
                if horiz_ys
                else (0.5 * (t_y_min + t_y_max))
            )

            for y in range(y_lo, y_hi + 1):
                if (px, y) in occ.pin_owner or (px, y) in occ.body_blocked or (px, y) in occ.label_blocked:
                    continue
                # Penalize plunging outside the net's vertical span `[t_y_min - 2 .. t_y_max + 2]`
                span_over = max(0, (t_y_min - 2) - y, y - (t_y_max + 2))
                out_of_span_pen = (24.0 + 1.6 * span_over) if span_over > 0 else 0.0

                # Cast straight horizontal ray from `(px, y)` inward
                v_cross_at_x: dict[int, int] = {}
                v_cross = 0
                hit_x: int | None = None
                max_reach_x = px
                cx = px
                while (cx < x_limit) if go_right else (cx > x_limit):
                    nx = cx + dx
                    sc = occ.step_cost(
                        net,
                        (cx, y),
                        (nx, y),
                        None if cx == px else dir_h,
                        dir_h,
                        t_pts,
                        allow_tight_parallel=False,
                    )
                    if sc is None:
                        break
                    max_reach_x = nx
                    if occ.wire_v.get((nx, y), set()) - {net}:
                        v_cross += 1
                    v_cross_at_x[nx] = v_cross
                    if (nx, y) in t_pts:
                        if not (
                            (occ.wire_h.get((nx, y), set()) - {net})
                            or (occ.wire_v.get((nx, y), set()) - {net})
                        ):
                            hit_x = nx
                            break
                    cx = nx

                if max_reach_x == px:
                    continue

                if hit_x is not None:
                    # 0-bend straight horizontal hit right onto `net`'s Steiner trunk!
                    # Strongly reward collinear continuation of an existing horizontal trunk segment!
                    collinear_trunk = net in occ.wire_h.get((hit_x + dx, y), set())
                    u_cost = (
                        0.16 * abs(hit_x - px)
                        + 9.5 * v_cross
                        + 0.32 * abs(y - t_y_pref)
                        + out_of_span_pen
                        - (7.0 if collinear_trunk else 0.0)
                    )
                    raw_cands.append((y, u_cost, 0, hit_x, y))
                    continue

                # Check 1-bend L-route `(px, y) -> (tx, y) -> (tx, ty)` to facing anchors
                best_l: tuple[float, int, int] | None = None
                for tx, ty in anchor_pts:
                    if (tx - px) * dx < 1 or abs(tx - px) > abs(max_reach_x - px):
                        continue
                    if ty == y:
                        continue
                    # `(tx, y)` is a bend, so no foreign wire may pass through `(tx, y)`
                    if (occ.wire_h.get((tx, y), set()) - {net}) or (occ.wire_v.get((tx, y), set()) - {net}):
                        continue
                    step_vy = 1 if ty > y else -1
                    dir_v = 2 if step_vy > 0 else 3
                    leg_ok = True
                    leg_cross = 0
                    cy = y
                    while cy != ty:
                        ny = cy + step_vy
                        # Respect pin outward escape if `(tx, ty)` is a pin with escape away from `cy`
                        if ny == ty:
                            esc = occ.pin_escape.get((tx, ty))
                            if esc is not None and esc != (tx, cy) and (tx, cy) in occ.body_blocked:
                                leg_ok = False
                                break
                        sc_v = occ.step_cost(
                            net,
                            (tx, cy),
                            (tx, ny),
                            dir_h if cy == y else dir_v,
                            dir_v,
                            t_pts,
                            allow_tight_parallel=False,
                        )
                        if sc_v is None:
                            leg_ok = False
                            break
                        if occ.wire_h.get((tx, ny), set()) - {net}:
                            leg_cross += 1
                        cy = ny
                    if leg_ok:
                        l_cost = (
                            9.0
                            + 0.20 * abs(tx - px)
                            + 0.55 * abs(ty - y)
                            + 16.0 * (v_cross_at_x.get(tx, 0) + leg_cross)
                            + out_of_span_pen
                        )
                        if best_l is None or l_cost < best_l[0]:
                            best_l = (l_cost, tx, ty)

                if best_l is not None:
                    raw_cands.append((y, best_l[0], 1, best_l[1], best_l[2]))
                elif abs(max_reach_x - px) >= 2:
                    tx0, ty0 = min(anchor_pts, key=lambda pt: abs(pt[1] - y) + 0.25 * abs(pt[0] - px))
                    u_cost = 36.0 + 1.15 * abs(y - ty0) + 0.20 * abs(tx0 - px) + out_of_span_pen
                    raw_cands.append((y, u_cost, 2, tx0, ty0))

        if not raw_cands:
            for y in range(y_lo, y_hi + 1, 2):
                if (px, y) not in occ.pin_owner:
                    raw_cands.append((y, 50.0 + abs(y - po.y), 2, px + dx * 8, po.y))

        # Prune to top 20 by cost + 12 evenly spaced across range for fast exact DP
        by_cost = sorted(raw_cands, key=lambda c: (c[1], c[0]))
        keep_map: dict[int, tuple[int, float, int, int, int]] = {c[0]: c for c in by_cost[:20]}
        stride = max(1, len(raw_cands) // 12)
        for c in raw_cands[::stride]:
            keep_map.setdefault(c[0], c)
        pruned = sorted(keep_map.values(), key=lambda c: c[0])
        cands_per_port.append(pruned)

        best_c = by_cost[0]
        init_sort_keys.append((float(best_c[0]), float(best_c[3] if go_right else -best_c[3]), idx))

    def _solve_for_order(order: tuple[int, ...]) -> tuple[float, list[tuple[int, float, int, int, int]]] | None:
        dp: list[list[float]] = []
        parent_idx: list[list[int]] = []
        for r, p_idx in enumerate(order):
            c_list = cands_per_port[p_idx]
            row_dp = [float("inf")] * len(c_list)
            row_par = [-1] * len(c_list)
            if r == 0:
                for m, c in enumerate(c_list):
                    row_dp[m] = c[1]
            else:
                prev_list = cands_per_port[order[r - 1]]
                prev_dp = dp[r - 1]
                for m, (yc, uc, _, txc, tyc) in enumerate(c_list):
                    best_v = float("inf")
                    best_p = -1
                    for mp, (yp, _, _, txp, typ) in enumerate(prev_list):
                        dy_ports = yc - yp
                        if dy_ports < 2:
                            break
                        if prev_dp[mp] == float("inf"):
                            continue
                        tight_port_pen = 3.0 if dy_ports == 2 else (0.8 if dy_ports == 3 else 0.0)
                        cand_v = (
                            prev_dp[mp]
                            + uc
                            + tight_port_pen
                            + _pair_port_cross_cost(px, yp, txp, typ, yc, txc, tyc)
                        )
                        if cand_v < best_v:
                            best_v = cand_v
                            best_p = mp
                    row_dp[m] = best_v
                    row_par[m] = best_p
            dp.append(row_dp)
            parent_idx.append(row_par)

        last_dp = dp[-1]
        best_last = min(range(len(last_dp)), key=lambda m: last_dp[m])
        if last_dp[best_last] == float("inf"):
            return None

        chosen: list[tuple[int, float, int, int, int]] = [cands_per_port[order[-1]][best_last]]
        cur_m = best_last
        for r in range(k - 1, 0, -1):
            cur_m = parent_idx[r][cur_m]
            chosen.append(cands_per_port[order[r - 1]][cur_m])
        chosen.reverse()

        # Evaluate exact objective including non-adjacent pairwise crossings
        exact_cost = sum(c[1] for c in chosen)
        for a in range(k):
            ya, _, _, txa, tya = chosen[a]
            for b in range(a + 1, k):
                yb, _, _, txb, tyb = chosen[b]
                exact_cost += _pair_port_cross_cost(px, ya, txa, tya, yb, txb, tyb)
        return exact_cost, chosen

    best_overall_cost = float("inf")
    best_perm: tuple[int, ...] = tuple(x[2] for x in sorted(init_sort_keys))
    best_chosen: list[tuple[int, float, int, int, int]] | None = None

    if k <= 5:
        for perm in permutations(range(k)):
            res_p = _solve_for_order(perm)
            if res_p is not None and res_p[0] < best_overall_cost:
                best_overall_cost, best_chosen = res_p
                best_perm = perm
    else:
        cur_perm = list(best_perm)
        res_init = _solve_for_order(tuple(cur_perm))
        if res_init is not None:
            best_overall_cost, best_chosen = res_init
        improved = True
        while improved:
            improved = False
            for a in range(k):
                for b in range(a + 1, k):
                    trial = list(cur_perm)
                    trial[a], trial[b] = trial[b], trial[a]
                    res_t = _solve_for_order(tuple(trial))
                    if res_t is not None and res_t[0] + 1e-6 < best_overall_cost:
                        best_overall_cost, best_chosen = res_t
                        cur_perm = trial
                        best_perm = tuple(trial)
                        improved = True

    out_assigned: list[tuple[PortOccurrencePlaced, int, int, int]] = []
    if best_chosen is not None:
        for p_idx, (yc, _, krank, rtx, rty) in zip(best_perm, best_chosen):
            po = side_ports[p_idx]
            po.y = yc
            out_assigned.append((po, krank, rtx, rty))
    else:
        for rank_i, p_idx in enumerate(best_perm):
            po = side_ports[p_idx]
            po.y = y_lo + 4 + rank_i * 4
            out_assigned.append((po, 2, px + dx * 8, po.y))
    return out_assigned


def route_placed_schematic(
    circuit: NetlistCircuit,
    intent: SchematicIntent,
    pdk: PdkRegistry,
    placements: dict[str, PlacedInstance],
    ports: list[PortOccurrencePlaced],
    local_bulk_labels: list[dict[str, str]],
    local_bulk_shorts: list[dict[str, str]],
    *,
    allow_wire_labels: bool = False,
) -> tuple[dict[str, JunctionPlaced], list[RoutePlaced], list[str], list[WireLabelStubPlaced], list[BusRipperPlaced]]:
    """Route all nets into exact orthogonal Steiner trees with post-trunk edge-port alignment."""
    supplies = set(circuit.supply_nets)
    grounds = set(circuit.ground_nets)
    rails = supplies | grounds

    bulk_handled: set[tuple[str, str]] = set()
    for item in local_bulk_labels:
        bulk_handled.add((item["reference"], "B"))
    for item in local_bulk_shorts:
        bulk_handled.add((item["reference"], item["bulkPinName"]))

    repeated_port_nets = set(rails) | {
        po.net_name
        for po in ports
        if intent.port_sides.get(po.port_name) == "local_repeat"
    }
    label_stub_nets_pre: set[str] = (
        {
            net
            for net in circuit.nets
            if net not in repeated_port_nets and intent.net_policies.get(net) == "label_stub"
        }
        if allow_wire_labels
        else set()
    )

    min_inst_x = min(p.full_envelope[0] for p in placements.values())
    max_inst_x = max(p.full_envelope[2] for p in placements.values())
    min_inst_y = min(p.full_envelope[1] for p in placements.values())
    max_inst_y = max(p.full_envelope[3] for p in placements.values())
    mid_inst_y = 0.5 * (min_inst_y + max_inst_y)

    edge_left_ports = [
        po
        for po in ports
        if po.symbol_id == "port"
        and po.rotation == 0
        and po.mirror == "none"
        and po.x <= min_inst_x - 4
        and po.net_name not in repeated_port_nets
        and po.net_name not in label_stub_nets_pre
    ]
    edge_right_ports = [
        po
        for po in ports
        if po.symbol_id == "port"
        and po.rotation == 0
        and po.mirror == "horizontal"
        and po.x >= max_inst_x + 4
        and po.net_name not in repeated_port_nets
        and po.net_name not in label_stub_nets_pre
    ]
    edge_occ_ids = {po.occurrence_id for po in edge_left_ports + edge_right_ports}
    fixed_ports = [po for po in ports if po.occurrence_id not in edge_occ_ids]

    occ = _GridOccupancy(
        placements=placements,
        ports=fixed_ports,
        local_bulk_labels=local_bulk_labels,
        local_bulk_shorts=local_bulk_shorts,
        pdk=pdk,
        track_pitch=intent.spacing.wire_track_pitch,
    )
    occ.register_instance_pins(circuit, bulk_handled)
    if edge_left_ports:
        occ.edge_wall_xs.add(min(po.x for po in edge_left_ports))
        occ.left_port_nets = {po.net_name for po in edge_left_ports}
    if edge_right_ports:
        occ.edge_wall_xs.add(max(po.x for po in edge_right_ports))
        occ.right_port_nets = {po.net_name for po in edge_right_ports}

    all_xs = [min_inst_x, max_inst_x] + [pt.x for pt in ports]
    all_ys = [min_inst_y, max_inst_y] + [pt.y for pt in fixed_ports]
    min_x = min(po.x for po in edge_left_ports) if edge_left_ports else (min(all_xs) - 14)
    max_x = max(po.x for po in edge_right_ports) if edge_right_ports else (max(all_xs) + 14)
    min_y, max_y = min(all_ys) - 14, max(all_ys) + 14
    search_bounds: BBox = (min_x, min_y, max_x, max_y)
    top_bus_y = min_inst_y - 5
    bottom_bus_y = max_inst_y + 5

    # Map every routable endpoint per net: point -> RouteStepPlaced
    endpoints_by_net: dict[str, list[RouteStepPlaced]] = defaultdict(list)
    port_occs_by_net: dict[str, list[PortOccurrencePlaced]] = defaultdict(list)
    fixed_port_occs_by_net: dict[str, list[PortOccurrencePlaced]] = defaultdict(list)
    for pt_occ in ports:
        port_occs_by_net[pt_occ.net_name].append(pt_occ)
        if pt_occ.occurrence_id not in edge_occ_ids:
            fixed_port_occs_by_net[pt_occ.net_name].append(pt_occ)

    for inst in circuit.instances:
        pl = placements[inst.ref]
        for pin, net in inst.terminals.items():
            if (inst.ref, pin) in bulk_handled:
                continue
            px, py = pl.pin_points[pin]
            endpoints_by_net[net].append(
                RouteStepPlaced(
                    kind="instance",
                    x=px,
                    y=py,
                    instance_id=pl.instance_id,
                    pin_name=pin,
                )
            )

    routes: list[RoutePlaced] = []
    junctions: dict[str, JunctionPlaced] = {}
    wire_label_stubs: list[WireLabelStubPlaced] = []
    bus_rippers: list[BusRipperPlaced] = []
    tight_nets: list[str] = []
    route_counter = 0
    junc_counter = 0
    stub_counter = 0

    # Optional Bus Ripper synthesis for `parallel_bank` motifs with `bottomTapStyle == "bus_ripper"`
    inst_map = circuit.instance_map()
    bus_handled_nets: set[str] = set()
    for m_idx, m in enumerate(intent.motifs):
        if m.get("type") == "parallel_bank" and m.get("bottomTapStyle") == "bus_ripper":
            b_refs = [str(x) for x in m.get("instances", []) if str(x) in placements]
            cnet = str(m.get("commonTopNet", ""))
            taps: list[tuple[str, int, int]] = []
            for r in b_refs:
                inst = inst_map[r]
                pl = placements[r]
                for pin, pnet in inst.terminals.items():
                    if pnet != cnet and pin != "B":
                        px, py = pl.pin_points[pin]
                        taps.append((pnet, px, py))
            if len(taps) >= 2:
                trunk_y = max(t[2] for t in taps) + 5
                x_s = min(t[1] for t in taps) - 3
                x_e = max(t[1] for t in taps) + 5
                bus_rippers.append(
                    BusRipperPlaced(
                        id=str(m.get("id", f"bus_{m_idx + 1}")),
                        bus_name=str(m.get("busName", "BUS")),
                        trunk_y=trunk_y,
                        x_start=x_s,
                        x_end=x_e,
                        taps=taps,
                    )
                )
                if allow_wire_labels:
                    for t_net, tx, ty in taps:
                        stub_counter += 1
                        wire_label_stubs.append(
                            WireLabelStubPlaced(
                                id=f"WL_BUS_{stub_counter}",
                                net_name=t_net,
                                ref="BUS",
                                pin_name=t_net,
                                x1=tx,
                                y1=ty,
                                x2=tx,
                                y2=trunk_y - 1,
                            )
                        )
                        occ.add_polyline(t_net, [(tx, ty), (tx, trunk_y - 1)])
                        # If t_net only connects to this tap + port, mark handled
                        if len(endpoints_by_net.get(t_net, [])) <= 1:
                            bus_handled_nets.add(t_net)

    # 1. Route local supply / ground / repeated-port taps first (short straight stubs!)
    repeated_port_nets = set(rails) | {
        po.net_name
        for po in ports
        if intent.port_sides.get(po.port_name) == "local_repeat"
    }
    # Handle explicit `label_stub` nets when `allow_wire_labels=True`
    label_stub_nets: set[str] = set()
    if allow_wire_labels:
        for net in circuit.nets:
            if net not in repeated_port_nets and intent.net_policies.get(net) == "label_stub":
                label_stub_nets.add(net)
                for ep in endpoints_by_net.get(net, []):
                    esc = occ.pin_escape.get((ep.x, ep.y), (ep.x - 1, ep.y))
                    dx = esc[0] - ep.x
                    dy = esc[1] - ep.y
                    ex = ep.x + dx * 4
                    ey = ep.y + dy * 4
                    ref_name = str(ep.instance_id or "").replace("source-", "")
                    stub_counter += 1
                    wire_label_stubs.append(
                        WireLabelStubPlaced(
                            id=f"WL_{net}_{stub_counter}",
                            net_name=net,
                            ref=ref_name,
                            pin_name=str(ep.pin_name or ""),
                            x1=ep.x,
                            y1=ep.y,
                            x2=ex,
                            y2=ey,
                        )
                    )
                    occ.add_polyline(net, [(ep.x, ep.y), (ex, ey)])
                    for step_k in range(1, 5):
                        occ.label_blocked.add((ep.x + dx * (4 + step_k), ep.y + dy * (4 + step_k)))
    for rail_net in [n for n in circuit.nets if n in repeated_port_nets]:
        rail_ports = port_occs_by_net.get(rail_net, [])
        if not rail_ports:
            continue
        port_step_by_pt = {
            (po.x, po.y): RouteStepPlaced(
                kind="port",
                x=po.x,
                y=po.y,
                instance_id=po.occurrence_id,
                pin_name=po.port_name,
            )
            for po in rail_ports
        }
        rail_goals = set(port_step_by_pt)
        for ep in endpoints_by_net.get(rail_net, []):
            path_r = _astar_connect_to_tree(
                rail_net,
                (ep.x, ep.y),
                rail_goals,
                occ,
                search_bounds,
                allow_tight_parallel=True,
            )
            if path_r is None:
                nearest = min(rail_ports, key=lambda po: abs(po.x - ep.x) + abs(po.y - ep.y))
                p_step = port_step_by_pt[(nearest.x, nearest.y)]
                path_r = (
                    [(ep.x, ep.y), (p_step.x, p_step.y)]
                    if (ep.x == p_step.x or ep.y == p_step.y)
                    else [(ep.x, ep.y), (ep.x, p_step.y), (p_step.x, p_step.y)]
                )
            p_step = port_step_by_pt[path_r[-1]]
            steps = [RouteStepPlaced(kind="bend", x=bx, y=by) for (bx, by) in path_r[1:-1]] + [p_step]
            route_counter += 1
            routes.append(
                RoutePlaced(
                    id=f"R_{rail_net}_{route_counter}",
                    net_name=rail_net,
                    start=ep,
                    steps=steps,
                )
            )
            occ.add_polyline(rail_net, path_r)

    # 2. Order non-rail nets: critical short local nets first, then general full_wire, then perimeter_bus!
    non_rail_nets = [
        n
        for n in circuit.nets
        if n not in repeated_port_nets and n not in label_stub_nets and n not in bus_handled_nets
    ]
    crit_set = set(intent.critical_nets)

    def _net_priority(n: str) -> tuple[int, int, int, int]:
        pol = intent.net_policies.get(n, "full_wire")
        if pol == "label_stub" and not allow_wire_labels:
            pol = "perimeter_bus"
        inst_eps_n = endpoints_by_net.get(n, [])
        eps = inst_eps_n + [
            RouteStepPlaced(kind="port", x=po.x, y=po.y, instance_id=po.occurrence_id, pin_name=po.port_name)
            for po in port_occs_by_net.get(n, [])
        ]
        xs = [e.x for e in (inst_eps_n or eps)] or [0]
        ys = [e.y for e in (inst_eps_n or eps)] or [0]
        span = (max(xs) - min(xs)) + (max(ys) - min(ys))
        # Coarse span band + left-to-right `min(xs)` ordering prevents braided cross-group diagonals!
        if pol == "perimeter_bus":
            return (2, len(eps), min(xs), span)
        if n in crit_set:
            return (0, span // 56, min(xs), span)
        return (1, span // 56, min(xs), span)

    non_rail_nets.sort(key=_net_priority)

    ep_at_point_by_net: dict[str, dict[Point, RouteStepPlaced]] = defaultdict(dict)
    junction_at_point_by_net: dict[str, dict[Point, str]] = defaultdict(dict)
    tree_points_by_net: dict[str, set[Point]] = defaultdict(set)

    def _bus_target_for_net(net: str) -> int | None:
        pol = intent.net_policies.get(net, "full_wire")
        if pol == "label_stub" and not allow_wire_labels:
            pol = "perimeter_bus"
        if pol != "perimeter_bus":
            return None
        inst_ys = [e.y for e in endpoints_by_net.get(net, [])]
        avg_y = (sum(inst_ys) / len(inst_ys)) if inst_ys else mid_inst_y
        return top_bus_y if avg_y <= mid_inst_y else bottom_bus_y

    def _connect_ep_to_net_tree(net: str, target_ep: RouteStepPlaced) -> None:
        nonlocal junc_counter, route_counter
        tree_points = tree_points_by_net[net]
        ep_at_point = ep_at_point_by_net[net]
        junction_at_point = junction_at_point_by_net[net]

        start_pt = (target_ep.x, target_ep.y)
        if not tree_points:
            tree_points.add(start_pt)
            return
        if start_pt in tree_points:
            return

        bus_y = _bus_target_for_net(net)
        path = _astar_connect_to_tree(
            net,
            start_pt,
            tree_points,
            occ,
            search_bounds,
            allow_tight_parallel=False,
            bus_y_target=bus_y,
        )
        if path is None:
            # Fallback 1: relax parallel pitch to 1 grid while keeping strict edge-port walls intact!
            tight_nets.append(net)
            path = _astar_connect_to_tree(
                net,
                start_pt,
                tree_points,
                occ,
                (min_x, min_y - 10, max_x, max_y + 10),
                allow_tight_parallel=True,
                bus_y_target=bus_y,
            )
        if path is None:
            # Fallback 2: emergency outer relaxation (triggers congestion corridor expansion on retry)
            saved_walls = set(occ.edge_wall_xs)
            occ.edge_wall_xs.clear()
            try:
                path = _astar_connect_to_tree(
                    net,
                    start_pt,
                    tree_points,
                    occ,
                    (min_x - 10, min_y - 10, max_x + 10, max_y + 10),
                    allow_tight_parallel=True,
                    bus_y_target=bus_y,
                )
            finally:
                occ.edge_wall_xs.update(saved_walls)
        if path is None:
            raise RuntimeError(f"Orthogonal router could not find obstacle-free route for net {net!r}")

        join_pt = path[-1]
        if join_pt in ep_at_point:
            end_step = ep_at_point[join_pt]
        elif join_pt in junction_at_point:
            end_step = RouteStepPlaced(
                kind="junction",
                x=join_pt[0],
                y=join_pt[1],
                junction_id=junction_at_point[join_pt],
            )
        else:
            junc_counter += 1
            jid = f"J_{net}_{junc_counter}"
            junctions[jid] = JunctionPlaced(
                id=jid, net_name=net, x=join_pt[0], y=join_pt[1], role="steiner"
            )
            junction_at_point[join_pt] = jid
            _split_segment_at_point(routes, net, join_pt, jid)
            end_step = RouteStepPlaced(kind="junction", x=join_pt[0], y=join_pt[1], junction_id=jid)

        steps: list[RouteStepPlaced] = [
            RouteStepPlaced(kind="bend", x=bx, y=by) for (bx, by) in path[1:-1]
        ]
        steps.append(end_step)

        route_counter += 1
        routes.append(
            RoutePlaced(
                id=f"R_{net}_{route_counter}",
                net_name=net,
                start=target_ep,
                steps=steps,
            )
        )
        occ.add_polyline(net, path)

        for a, b in zip(path, path[1:]):
            if a[0] == b[0]:
                for y in range(min(a[1], b[1]), max(a[1], b[1]) + 1):
                    tree_points.add((a[0], y))
            else:
                for x in range(min(a[0], b[0]), max(a[0], b[0]) + 1):
                    tree_points.add((x, a[1]))

    # 2a. Route all internal device-to-device trunks FIRST using Prim's nearest-collinear-first order!
    for net in non_rail_nets:
        internal_eps: list[RouteStepPlaced] = list(endpoints_by_net.get(net, []))
        for po in fixed_port_occs_by_net.get(net, []):
            internal_eps.append(
                RouteStepPlaced(
                    kind="port",
                    x=po.x,
                    y=po.y,
                    instance_id=po.occurrence_id,
                    pin_name=po.port_name,
                )
            )
        if not internal_eps:
            continue

        ep_map = ep_at_point_by_net[net]
        for e in internal_eps:
            ep_map[(e.x, e.y)] = e

        unique_eps = list(ep_map.values())
        if len(unique_eps) == 1:
            tree_points_by_net[net].add((unique_eps[0].x, unique_eps[0].y))
            continue

        def _is_unobstructed_collinear(ea: RouteStepPlaced, eb: RouteStepPlaced) -> bool:
            if ea.x == eb.x:
                return all(
                    (ea.x, cy) not in occ.body_blocked and (ea.x, cy) not in occ.label_blocked
                    for cy in range(min(ea.y, eb.y) + 1, max(ea.y, eb.y))
                )
            if ea.y == eb.y:
                return all(
                    (cx, ea.y) not in occ.body_blocked and (cx, ea.y) not in occ.label_blocked
                    for cx in range(min(ea.x, eb.x) + 1, max(ea.x, eb.x))
                )
            return False

        collinear_pairs = [
            (abs(ea.x - eb.x) + abs(ea.y - eb.y), ea, eb)
            for i_a, ea in enumerate(unique_eps)
            for eb in unique_eps[i_a + 1 :]
            if _is_unobstructed_collinear(ea, eb)
        ]

        ep_xs = [e.x for e in unique_eps]
        ep_ys = [e.y for e in unique_eps]
        if collinear_pairs:
            # 1) Lock unobstructed collinear stacks/mirrors (e.g. APR.D - INR.D) as straight 0-bend trunks first!
            _, seed_ep, second_ep = min(collinear_pairs, key=lambda item: item[0])
            tree_points_by_net[net].add((seed_ep.x, seed_ep.y))
            _connect_ep_to_net_tree(net, second_ep)
            remaining = [
                e
                for e in unique_eps
                if (e.x, e.y) not in {(seed_ep.x, seed_ep.y), (second_ep.x, second_ep.y)}
            ]
        elif (max(ep_xs) - min(ep_xs) > 24) and (max(ep_ys) - min(ep_ys) > 12):
            # 2) Cross-group & cross-tier net: establish main cross-group horizontal trunk FIRST
            # so local diode/mirror cluster pins drop cleanly onto one shared horizontal bar!
            seed_ep = min(unique_eps, key=lambda e: (e.x, e.y))
            far_cands = [
                e
                for e in unique_eps
                if (e.x, e.y) != (seed_ep.x, seed_ep.y)
                and (abs(e.x - seed_ep.x) > 16 or abs(e.y - seed_ep.y) > 10)
            ] or [e for e in unique_eps if (e.x, e.y) != (seed_ep.x, seed_ep.y)]
            second_ep = min(
                far_cands,
                key=lambda e: abs(e.x - seed_ep.x) + abs(e.y - seed_ep.y),
            )
            tree_points_by_net[net].add((seed_ep.x, seed_ep.y))
            _connect_ep_to_net_tree(net, second_ep)
            remaining = [
                e
                for e in unique_eps
                if (e.x, e.y) not in {(seed_ep.x, seed_ep.y), (second_ep.x, second_ep.y)}
            ]
        else:
            # 3) Compact local net: seed from closest pair
            best_pair_score = float("inf")
            seed_ep = unique_eps[0]
            for i_a, ea in enumerate(unique_eps):
                for eb in unique_eps[i_a + 1 :]:
                    sc_p = abs(ea.x - eb.x) + abs(ea.y - eb.y)
                    if sc_p < best_pair_score:
                        best_pair_score = sc_p
                        seed_ep = ea
            tree_points_by_net[net].add((seed_ep.x, seed_ep.y))
            remaining = [e for e in unique_eps if (e.x, e.y) != (seed_ep.x, seed_ep.y)]

        while remaining:
            t_cur = tree_points_by_net[net]
            nxt_ep = min(
                remaining,
                key=lambda e: min(
                    abs(e.x - tx) + abs(e.y - ty) - (6 if (e.x == tx or e.y == ty) else 0)
                    for tx, ty in t_cur
                ),
            )
            remaining.remove(nxt_ep)
            _connect_ep_to_net_tree(net, nxt_ep)

    # 2b. Optimally order & snap left/right edge ports onto straight horizontal rays / nested L-routes!
    for side_ports, go_right, x_lim in (
        (edge_left_ports, True, max_x - 1),
        (edge_right_ports, False, min_x + 1),
    ):
        if not side_ports:
            continue
        assigned = _optimize_side_edge_ports(
            side_ports,
            go_right=go_right,
            tree_points_by_net=tree_points_by_net,
            occ=occ,
            y_lo=min_inst_y - 6,
            y_hi=max_inst_y + 6,
            x_limit=x_lim,
        )
        for po, _, _, _ in assigned:
            occ.register_port(po)
            ep_at_point_by_net[po.net_name][(po.x, po.y)] = RouteStepPlaced(
                kind="port",
                x=po.x,
                y=po.y,
                instance_id=po.occurrence_id,
                pin_name=po.port_name,
            )

        # Route 0-bend direct hits first, then inner nested L-routes before outer nested L-routes!
        for po, krank, rtx, _ in sorted(
            assigned, key=lambda item: (item[1], abs(item[2] - item[0].x), item[0].y)
        ):
            p_step = ep_at_point_by_net[po.net_name][(po.x, po.y)]
            _connect_ep_to_net_tree(po.net_name, p_step)

    return junctions, routes, tight_nets, wire_label_stubs, bus_rippers


def compile_schematic(
    circuit: NetlistCircuit,
    intent: SchematicIntent,
    pdk: PdkRegistry | None = None,
    *,
    max_congestion_iters: int = 3,
    allow_wire_labels: bool = False,
) -> SolvedSchematic:
    """Run closed-loop placement + routing + corridor congestion expansion."""
    pdk = pdk or PdkRegistry()
    extra_col_gaps: dict[tuple[int, int], int] = defaultdict(int)
    extra_tier_gaps: dict[tuple[int, int], int] = defaultdict(int)
    expansions: list[str] = []

    placements: dict[str, PlacedInstance] = {}
    ports: list[PortOccurrencePlaced] = []
    bulk_labels: list[dict[str, str]] = []
    bulk_shorts: list[dict[str, str]] = []
    junctions: dict[str, JunctionPlaced] = {}
    routes: list[RoutePlaced] = []
    wl_stubs: list[WireLabelStubPlaced] = []
    bus_rippers: list[BusRipperPlaced] = []
    used_iters = 1

    for it in range(1, max_congestion_iters + 1):
        used_iters = it
        placements, ports, bulk_labels, bulk_shorts = place_schematic(
            circuit,
            intent,
            pdk,
            extra_col_gaps=extra_col_gaps,
            extra_tier_gaps=extra_tier_gaps,
            allow_wire_labels=allow_wire_labels,
        )
        try:
            junctions, routes, tight_nets, wl_stubs, bus_rippers = route_placed_schematic(
                circuit,
                intent,
                pdk,
                placements,
                ports,
                bulk_labels,
                bulk_shorts,
                allow_wire_labels=allow_wire_labels,
            )
        except RuntimeError as exc:
            if it == max_congestion_iters:
                raise
            tiers_present = sorted({p.tier for p in placements.values()})
            for ta in tiers_present:
                extra_tier_gaps[(ta, ta)] += 6
            for ta, tb in zip(tiers_present, tiers_present[1:]):
                extra_tier_gaps[(ta, tb)] += 6
            num_cols = max(1, len({p.x for p in placements.values()}))
            for ci in range(num_cols + 2):
                extra_col_gaps[(ci, ci + 1)] += 6
            expansions.append(f"iter {it}: expanded corridors by +6 grids after routing block ({exc})")
            continue

        if not tight_nets or it == max_congestion_iters:
            break

        # Congestion feedback: widen inter-column and inter-tier channels by +4 grids and re-solve!
        tiers_present = sorted({p.tier for p in placements.values()})
        for ta in tiers_present:
            extra_tier_gaps[(ta, ta)] += 4
        for ta, tb in zip(tiers_present, tiers_present[1:]):
            extra_tier_gaps[(ta, tb)] += 4
        num_cols = max(1, len({p.x for p in placements.values()}))
        for ci in range(num_cols):
            extra_col_gaps[(ci, ci + 1)] += 4
        expansions.append(
            f"iter {it}: expanded corridors by +4 grids due to congested nets {sorted(set(tight_nets))}"
        )

    # Compute tight overall schematic bounds
    xs: list[int] = []
    ys: list[int] = []
    for pl in placements.values():
        xs.extend([pl.full_envelope[0], pl.full_envelope[2]])
        ys.extend([pl.full_envelope[1], pl.full_envelope[3]])
    for po in ports:
        xs.append(po.x)
        ys.append(po.y)
    for rt in routes:
        for px, py in rt.points():
            xs.append(px)
            ys.append(py)
    for wl in wl_stubs:
        xs.extend([wl.x1, wl.x2])
        ys.extend([wl.y1, wl.y2])
    for br in bus_rippers:
        xs.extend([br.x_start, br.x_end])
        ys.append(br.trunk_y + 2)

    bounds: BBox = (min(xs) - 6, min(ys) - 6, max(xs) + 6, max(ys) + 6)
    return SolvedSchematic(
        circuit=circuit,
        intent=intent,
        placements=placements,
        ports=ports,
        junctions=junctions,
        routes=routes,
        local_bulk_labels=bulk_labels,
        local_bulk_shorts=bulk_shorts,
        bounds=bounds,
        congestion_iterations=used_iters,
        corridor_expansions=expansions,
        wire_label_stubs=wl_stubs,
        bus_rippers=bus_rippers,
    )
