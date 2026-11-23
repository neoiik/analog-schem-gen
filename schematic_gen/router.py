"""Orthogonal Steiner router, corridor parallel-segment nudger, and closed-loop congestion detector."""

from __future__ import annotations

from collections import defaultdict
import heapq
import math
from typing import Any

import numpy as np
from scipy.optimize import minimize

from schematic_gen.models import (
    BBox,
    JunctionPlaced,
    NetlistCircuit,
    PlacedInstance,
    Point,
    PortOccurrencePlaced,
    RoutePlaced,
    RouteStepPlaced,
    SchematicIntent,
    SolvedSchematic,
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
            self.pin_owner[(pt_port.x, pt_port.y)] = pt_port.net_name
            self.wire_corners[(pt_port.x, pt_port.y)].add(pt_port.net_name)
            # Reserve port text area behind the port pin so wires don't overwrite port names!
            if pt_port.symbol_id == "port":
                if pt_port.mirror == "horizontal":
                    # Right-side port: text extends right of pt_port.x
                    for tx in range(pt_port.x + 1, pt_port.x + 8):
                        for ty in range(pt_port.y - 1, pt_port.y + 2):
                            self.label_blocked.add((tx, ty))
                else:
                    # Left-side port: text extends left of pt_port.x
                    for tx in range(pt_port.x - 7, pt_port.x):
                        for ty in range(pt_port.y - 1, pt_port.y + 2):
                            self.label_blocked.add((tx, ty))
            elif pt_port.symbol_id == "vdd-port":
                for ty in range(pt_port.y - 3, pt_port.y):
                    self.body_blocked.add((pt_port.x, ty))
            elif pt_port.symbol_id == "ground":
                for ty in range(pt_port.y + 1, pt_port.y + 4):
                    self.body_blocked.add((pt_port.x, ty))

    def register_instance_pins(self, circuit: NetlistCircuit) -> None:
        for inst in circuit.instances:
            pl = self.placements[inst.ref]
            bx0, by0, bx1, by1 = pl.body_box
            for pin, net in inst.terminals.items():
                px, py = pl.pin_points[pin]
                self.pin_owner[(px, py)] = net
                self.wire_corners[(px, py)].add(net)
                # Outward escape step from symbol boundary
                if py <= by0:
                    self.pin_escape[(px, py)] = (px, py - 1)
                elif py >= by1:
                    self.pin_escape[(px, py)] = (px, py + 1)
                elif px <= bx0:
                    self.pin_escape[(px, py)] = (px - 1, py)
                else:
                    self.pin_escape[(px, py)] = (px + 1, py)

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
        # 1. Never enter symbol bodies or text labels (except exact start/goal pin itself)
        if v not in goal_set:
            if v in self.body_blocked or v in self.label_blocked:
                return None
        owner = self.pin_owner.get(v)
        if owner is not None and owner != net:
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

        # 6. Compute cost (reward reusing same-net trunk -> Steiner tree!)
        reusing_same_net = (net in same_dir_map.get(v, set())) and (net in same_dir_map.get(u, set()))
        cost = 0.08 if reusing_same_net else 1.0

        if prev_dir is not None and prev_dir != new_dir:
            cost += 7.0

        # Perpendicular crossing over foreign wire: legal at interior point, high penalty
        if perp_dir_map.get(v, set()) - {net}:
            if v in goal_set:
                return None
            cost += 55.0
        if u_perp_foreign and False:
            pass

        # Soft guide toward perimeter bus Y for global bias nets
        if bus_y_target is not None and is_horiz:
            cost += min(1.8, abs(v[1] - bus_y_target) * 0.06)

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


def route_placed_schematic(
    circuit: NetlistCircuit,
    intent: SchematicIntent,
    pdk: PdkRegistry,
    placements: dict[str, PlacedInstance],
    ports: list[PortOccurrencePlaced],
    local_bulk_labels: list[dict[str, str]],
    local_bulk_shorts: list[dict[str, str]],
) -> tuple[dict[str, JunctionPlaced], list[RoutePlaced], list[str]]:
    """Route all nets into exact orthogonal Steiner trees with corridor track separation."""
    supplies = set(circuit.supply_nets)
    grounds = set(circuit.ground_nets)
    rails = supplies | grounds

    occ = _GridOccupancy(
        placements=placements,
        ports=ports,
        local_bulk_labels=local_bulk_labels,
        local_bulk_shorts=local_bulk_shorts,
        pdk=pdk,
        track_pitch=intent.spacing.wire_track_pitch,
    )
    occ.register_instance_pins(circuit)

    all_xs = [p.full_envelope[0] for p in placements.values()] + [
        p.full_envelope[2] for p in placements.values()
    ] + [pt.x for pt in ports]
    all_ys = [p.full_envelope[1] for p in placements.values()] + [
        p.full_envelope[3] for p in placements.values()
    ] + [pt.y for pt in ports]
    min_x, max_x = min(all_xs) - 14, max(all_xs) + 14
    min_y, max_y = min(all_ys) - 12, max(all_ys) + 14
    search_bounds: BBox = (min_x, min_y, max_x, max_y)
    bottom_bus_y = max(p.full_envelope[3] for p in placements.values()) + 6

    bulk_handled: set[tuple[str, str]] = set()
    for item in local_bulk_labels:
        bulk_handled.add((item["reference"], "B"))
    for item in local_bulk_shorts:
        bulk_handled.add((item["reference"], item["bulkPinName"]))

    # Map every routable endpoint per net: point -> RouteStepPlaced
    endpoints_by_net: dict[str, list[RouteStepPlaced]] = defaultdict(list)
    port_occs_by_net: dict[str, list[PortOccurrencePlaced]] = defaultdict(list)
    for pt_occ in ports:
        port_occs_by_net[pt_occ.net_name].append(pt_occ)

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
    tight_nets: list[str] = []
    route_counter = 0
    junc_counter = 0

    # 1. Route local supply / ground / repeated-port taps first (short straight stubs!)
    repeated_port_nets = set(rails) | {
        po.net_name
        for po in ports
        if intent.port_sides.get(po.port_name) == "local_repeat"
    }
    for rail_net in [n for n in circuit.nets if n in repeated_port_nets]:
        rail_ports = port_occs_by_net.get(rail_net, [])
        if not rail_ports:
            continue
        for ep in endpoints_by_net.get(rail_net, []):
            nearest = min(rail_ports, key=lambda po: abs(po.x - ep.x) + abs(po.y - ep.y))
            p_step = RouteStepPlaced(
                kind="port",
                x=nearest.x,
                y=nearest.y,
                instance_id=nearest.occurrence_id,
                pin_name=nearest.port_name,
            )
            pts: list[Point]
            steps: list[RouteStepPlaced]
            if ep.x == p_step.x or ep.y == p_step.y:
                pts = [(ep.x, ep.y), (p_step.x, p_step.y)]
                steps = [p_step]
            else:
                mid = (ep.x, p_step.y)
                pts = [(ep.x, ep.y), mid, (p_step.x, p_step.y)]
                steps = [RouteStepPlaced(kind="bend", x=mid[0], y=mid[1]), p_step]
            route_counter += 1
            routes.append(
                RoutePlaced(
                    id=f"R_{rail_net}_{route_counter}",
                    net_name=rail_net,
                    start=ep,
                    steps=steps,
                )
            )
            occ.add_polyline(rail_net, pts)

    # 2. Order non-rail nets: critical short local nets first, then general full_wire, then perimeter_bus!
    non_rail_nets = [n for n in circuit.nets if n not in repeated_port_nets]
    crit_set = set(intent.critical_nets)

    def _net_priority(n: str) -> tuple[int, int, int]:
        pol = intent.net_policies.get(n, "full_wire")
        eps = endpoints_by_net.get(n, []) + [
            RouteStepPlaced(kind="port", x=po.x, y=po.y, instance_id=po.occurrence_id, pin_name=po.port_name)
            for po in port_occs_by_net.get(n, [])
        ]
        xs = [e.x for e in eps] or [0]
        ys = [e.y for e in eps] or [0]
        span = (max(xs) - min(xs)) + (max(ys) - min(ys))
        if pol == "perimeter_bus":
            return (2, len(eps), span)
        if n in crit_set:
            return (0, span, len(eps))
        return (1, span, len(eps))

    non_rail_nets.sort(key=_net_priority)

    for net in non_rail_nets:
        all_eps: list[RouteStepPlaced] = list(endpoints_by_net.get(net, []))
        for po in port_occs_by_net.get(net, []):
            all_eps.append(
                RouteStepPlaced(
                    kind="port",
                    x=po.x,
                    y=po.y,
                    instance_id=po.occurrence_id,
                    pin_name=po.port_name,
                )
            )
        if len(all_eps) < 2:
            continue

        # Sort terminals left-to-right, top-to-bottom for clean Steiner trunk growth
        all_eps.sort(key=lambda e: (e.x, e.y))
        ep_at_point: dict[Point, RouteStepPlaced] = {(e.x, e.y): e for e in all_eps}
        junction_at_point: dict[Point, str] = {}
        tree_points: set[Point] = {(all_eps[0].x, all_eps[0].y)}

        pol = intent.net_policies.get(net, "full_wire")
        bus_y = bottom_bus_y if pol == "perimeter_bus" else None

        for target_ep in all_eps[1:]:
            start_pt = (target_ep.x, target_ep.y)
            if start_pt in tree_points:
                continue

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
                # Fallback 1: relax parallel pitch to 1 grid and mark net as congested for feedback loop!
                tight_nets.append(net)
                path = _astar_connect_to_tree(
                    net,
                    start_pt,
                    tree_points,
                    occ,
                    (min_x - 10, min_y - 10, max_x + 10, max_y + 10),
                    allow_tight_parallel=True,
                    bus_y_target=bus_y,
                )
            if path is None:
                raise RuntimeError(f"Orthogonal router could not find obstacle-free route for net {net!r}")

            join_pt = path[-1]
            # Determine endpoint representation at `join_pt`
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

            # Every grid point along `path` is now part of `net`'s Steiner tree!
            for a, b in zip(path, path[1:]):
                if a[0] == b[0]:
                    for y in range(min(a[1], b[1]), max(a[1], b[1]) + 1):
                        tree_points.add((a[0], y))
                else:
                    for x in range(min(a[0], b[0]), max(a[0], b[0]) + 1):
                        tree_points.add((x, a[1]))

    return junctions, routes, tight_nets


def compile_schematic(
    circuit: NetlistCircuit,
    intent: SchematicIntent,
    pdk: PdkRegistry | None = None,
    *,
    max_congestion_iters: int = 3,
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
    used_iters = 1

    for it in range(1, max_congestion_iters + 1):
        used_iters = it
        placements, ports, bulk_labels, bulk_shorts = place_schematic(
            circuit,
            intent,
            pdk,
            extra_col_gaps=extra_col_gaps,
            extra_tier_gaps=extra_tier_gaps,
        )
        junctions, routes, tight_nets = route_placed_schematic(
            circuit,
            intent,
            pdk,
            placements,
            ports,
            bulk_labels,
            bulk_shorts,
        )
        if not tight_nets or it == max_congestion_iters:
            break

        # Congestion feedback: widen inter-column and inter-tier channels by +4 grids and re-solve!
        tiers_present = sorted({p.tier for p in placements.values()})
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
    )
