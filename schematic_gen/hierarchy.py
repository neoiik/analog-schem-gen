"""Hierarchical partitioning, bottom-up multi-sheet compilation, flatten-back LVS, and Bridge 3-step handshake."""

from __future__ import annotations

from collections import Counter, defaultdict
import copy
from dataclasses import asdict, dataclass, field
from html import escape
import re
from typing import Any

from schematic_gen.analyzer import analyze_circuit
from schematic_gen.auditor import audit_solved_schematic
from schematic_gen.manifest_builder import (
    build_bridge_manifest,
    ensure_virtuoso_bridge_on_path,
    verify_with_virtuoso_bridge_offline,
)
from schematic_gen.models import (
    AuditReport,
    InstanceIR,
    NetlistCircuit,
    NetlistDesign,
    PortIR,
    SchematicIntent,
    SolvedSchematic,
)
from schematic_gen.optimizer import optimize_schematic_crossings
from schematic_gen.quality import solved_quality_cost
from schematic_gen.pdk import PdkRegistry
from schematic_gen.hierarchy_lvs import verify_hierarchy_flatten_lvs


@dataclass
class PagePartitionSpec:
    """Specification for one child functional schematic sheet."""

    page_id: str
    cell_name: str
    title: str
    role: str
    stage_order: int
    instances: list[str]

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class HierarchyIntent:
    """Coordinate-free multi-sheet partitioning contract for large flat netlists."""

    top_cell: str
    pages: list[PagePartitionSpec]
    global_bias_nets: list[str] = field(default_factory=list)
    authored_by: str = "hierarchy_partitioner"
    notes: str = ""

    @classmethod
    def from_dict(cls, raw: dict[str, Any]) -> "HierarchyIntent":
        pages = [
            PagePartitionSpec(
                page_id=str(p["page_id"]),
                cell_name=str(p["cell_name"]),
                title=str(p.get("title", p["page_id"])),
                role=str(p.get("role", "subblock")),
                stage_order=int(p.get("stage_order", idx * 10)),
                instances=[str(x) for x in p.get("instances", [])],
            )
            for idx, p in enumerate(raw.get("pages", []))
        ]
        return cls(
            top_cell=str(raw.get("topCell") or raw.get("top_cell") or "top_core"),
            pages=pages,
            global_bias_nets=[str(x) for x in raw.get("globalBiasNets", raw.get("global_bias_nets", []))],
            authored_by=str(raw.get("authoredBy", raw.get("authored_by", "user_or_llm"))),
            notes=str(raw.get("notes", "")),
        )

    def to_dict(self) -> dict[str, Any]:
        return {
            "schema": "schem_gen.hierarchy_intent.v1",
            "topCell": self.top_cell,
            "authoredBy": self.authored_by,
            "notes": self.notes,
            "globalBiasNets": list(self.global_bias_nets),
            "pages": [p.to_dict() for p in self.pages],
        }


@dataclass
class HierarchyBuildResult:
    """Complete multi-sheet compilation output with flatten-back LVS & per-sheet DRC."""

    design: NetlistDesign
    hierarchy_intent: HierarchyIntent | None
    solved_pages: dict[str, SolvedSchematic]   # ordered leaf cells + top_cell last
    audits: dict[str, AuditReport]
    combined_manifest: dict[str, Any]
    augmented_process_map: dict[str, Any]
    flatten_lvs_passed: bool
    flatten_lvs_Diffs: list[str]

    @property
    def overall_passed(self) -> bool:
        return self.flatten_lvs_passed and all(r.overall_passed for r in self.audits.values())


def _slugify(text: str, idx: int) -> str:
    words = re.findall(r"[A-Za-z0-9]+", text.lower())
    short = "_".join(words[:4]) if words else f"page_{idx}"
    return f"p{idx:02d}_{short[:28]}"


def _infer_page_ranks(
    buckets: list[tuple[str, list[str]]],
    circuit: NetlistCircuit,
    global_bias_nets: set[str],
) -> list[int]:
    """Topology-only left-to-right page ranking (no page-title keywords, no demo-specific vocabulary).

    Buckets are classified as bias source (contains an independent source or drives a global bias rail through
    a diode-connected device) -> input (touches non-rail input ports) -> intermediate (ordered by longest
    signal path from input pages on the page-level channel->gate graph) -> output (drives output ports).
    """
    inst_map = circuit.instance_map()
    rails = set(circuit.supply_nets) | set(circuit.ground_nets)
    in_ports = {p.net for p in circuit.ports if p.direction == "input"} - rails
    out_ports = {p.net for p in circuit.ports if p.direction == "output"} - rails
    ctrl_pins = {"G", "BASE"}

    page_of: dict[str, int] = {r: bi for bi, (_t, mem) in enumerate(buckets) for r in mem}
    drives: list[set[str]] = [set() for _ in buckets]
    ctrls: list[set[str]] = [set() for _ in buckets]
    is_bias_src = [False] * len(buckets)
    for bi, (_t, mem) in enumerate(buckets):
        for r in mem:
            inst = inst_map[r]
            if inst.device_class in {"isource", "vsource"}:
                is_bias_src[bi] = True
            for pin, net in inst.terminals.items():
                if pin == "B" or net in rails:
                    continue
                if pin in ctrl_pins:
                    ctrls[bi].add(net)
                else:
                    drives[bi].add(net)
            # Diode-connected master feeding a global bias rail
            g = inst.terminals.get("G", inst.terminals.get("BASE", ""))
            d = inst.terminals.get("D", inst.terminals.get("C", ""))
            if g and g == d and g in global_bias_nets:
                is_bias_src[bi] = True

    n = len(buckets)
    succ: list[set[int]] = [set() for _ in range(n)]
    for a in range(n):
        for b in range(n):
            if a != b and (drives[a] & ctrls[b]) - global_bias_nets:
                succ[a].add(b)

    touches_in = [bool((drives[i] | ctrls[i]) & in_ports) for i in range(n)]
    touches_out = [bool(drives[i] & out_ports) for i in range(n)]
    depth = [0] * n
    frontier = [i for i in range(n) if touches_in[i]] or [i for i in range(n) if is_bias_src[i]]
    seen = set(frontier)
    level = 0
    while frontier and level < n:
        nxt: list[int] = []
        for u in frontier:
            for v in succ[u]:
                if v not in seen:
                    seen.add(v)
                    depth[v] = level + 1
                    nxt.append(v)
        frontier = nxt
        level += 1

    ranks: list[int] = []
    for i in range(n):
        if is_bias_src[i] and not touches_out[i]:
            ranks.append(5)
        elif touches_out[i]:
            ranks.append(85)
        elif touches_in[i]:
            ranks.append(25)
        else:
            ranks.append(min(80, 40 + 5 * depth[i]))
    return ranks


def propose_flat_partition(
    circuit: NetlistCircuit,
    *,
    max_page_size: int = 24,
    min_page_size: int = 8,
) -> HierarchyIntent:
    """Partition a large flat analog netlist into cohesive child sheets while protecting Atomic Clusters."""
    inst_map = circuit.instance_map()
    supplies = set(circuit.supply_nets)
    grounds = set(circuit.ground_nets)
    rails = supplies | grounds
    analysis = analyze_circuit(circuit)

    # 1. Build Atomic Clusters (diff pairs, cascode pairs, series RC, local 2-pin passive-MOS ties)
    parent: dict[str, str] = {i.ref: i.ref for i in circuit.instances}

    def _find(x: str) -> str:
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    def _union(a: str, b: str) -> None:
        ra, rb = _find(a), _find(b)
        if ra != rb:
            parent[rb] = ra

    for m in analysis.motifs:
        if m.motif_type in {"diff_pair", "cascode_pair", "rc_series"}:
            base = m.instances[0]
            for other in m.instances[1:]:
                if inst_map[base].section_hint == inst_map[other].section_hint or not inst_map[base].section_hint:
                    _union(base, other)
        elif m.motif_type == "current_mirror" and len(m.instances) <= 3:
            base = m.instances[0]
            for other in m.instances[1:]:
                if inst_map[base].section_hint == inst_map[other].section_hint:
                    _union(base, other)

    # Also keep 2-terminal internal-node degeneration/RC pairs together
    by_net: dict[str, list[str]] = defaultdict(list)
    for inst in circuit.instances:
        for pin, net in inst.terminals.items():
            if net not in rails and pin != "B":
                by_net[net].append(inst.ref)
    for net, refs in by_net.items():
        uniq = list(dict.fromkeys(refs))
        if len(uniq) == 2:
            a, b = uniq
            if inst_map[a].section_hint == inst_map[b].section_hint:
                _union(a, b)

    # HARD atomic segments (private pure-series nodes / same Super-MOS stack) are never split across sheets,
    # even when netlist section comments disagree — same rule as `validate-modules`.
    for br in analysis.branch_abstraction.get("branches", []):
        for seg in br.get("hardSegments", []):
            for other in seg[1:]:
                _union(seg[0], other)

    clusters_by_root: dict[str, list[str]] = defaultdict(list)
    for inst in circuit.instances:
        clusters_by_root[_find(inst.ref)].append(inst.ref)

    # 2. Group atomic clusters into pages (respecting section cohesion + max_page_size!)
    has_sections = any(bool(i.section_hint) for i in circuit.instances)
    raw_buckets: list[tuple[str, list[str]]] = []

    if has_sections:
        sec_order: list[str] = []
        sec_clusters: dict[str, list[list[str]]] = defaultdict(list)
        for root, members in clusters_by_root.items():
            # Majority section hint among cluster members
            sec_counts = Counter(inst_map[r].section_hint for r in members)
            sec = sec_counts.most_common(1)[0][0] or "Core Block"
            if sec not in sec_clusters:
                sec_order.append(sec)
            sec_clusters[sec].append(members)

        for sec in sec_order:
            flat_members = [r for cl in sec_clusters[sec] for r in cl]
            if len(flat_members) <= max_page_size:
                raw_buckets.append((sec, flat_members))
            else:
                # Split oversized section cleanly along cluster boundaries
                cur_chunk: list[str] = []
                part_no = 1
                for cl in sec_clusters[sec]:
                    if cur_chunk and len(cur_chunk) + len(cl) > max_page_size:
                        raw_buckets.append((f"{sec} ({part_no})", cur_chunk))
                        cur_chunk = list(cl)
                        part_no += 1
                    else:
                        cur_chunk.extend(cl)
                if cur_chunk:
                    raw_buckets.append((f"{sec} ({part_no})" if part_no > 1 else sec, cur_chunk))

        # Merge tiny adjacent sections (< min_page_size) if combined size <= max_page_size
        merged_buckets: list[tuple[str, list[str]]] = []
        for title, members in raw_buckets:
            if (
                merged_buckets
                and (len(merged_buckets[-1][1]) < min_page_size or len(members) < min_page_size)
                and len(merged_buckets[-1][1]) + len(members) <= max_page_size
            ):
                prev_title, prev_m = merged_buckets[-1]
                comb_title = f"{prev_title} & {title}" if len(prev_title) < 28 else prev_title
                merged_buckets[-1] = (comb_title, prev_m + members)
            else:
                merged_buckets.append((title, list(members)))
        raw_buckets = merged_buckets
    else:
        # Pure connectivity greedy clustering when no section comments exist
        unassigned = list(clusters_by_root.values())
        page_idx = 1
        while unassigned:
            seed = unassigned.pop(0)
            cur_page = list(seed)
            while unassigned and len(cur_page) < max_page_size:
                cur_nets = {
                    n
                    for r in cur_page
                    for pin, n in inst_map[r].terminals.items()
                    if n not in rails and pin != "B" and len(by_net[n]) <= 8
                }
                best_i = -1
                best_gain = -1
                for idx_c, cand in enumerate(unassigned):
                    if len(cur_page) + len(cand) > max_page_size:
                        continue
                    cand_nets = {
                        n
                        for r in cand
                        for pin, n in inst_map[r].terminals.items()
                        if n not in rails and pin != "B"
                    }
                    gain = len(cur_nets & cand_nets)
                    if gain > best_gain:
                        best_gain = gain
                        best_i = idx_c
                if best_i < 0 or (best_gain == 0 and len(cur_page) >= min_page_size):
                    break
                cur_page.extend(unassigned.pop(best_i))
            raw_buckets.append((f"Functional Subblock {page_idx}", cur_page))
            page_idx += 1

    # 3. Build PagePartitionSpec list & identify cross-sheet global bias nets (>= 3 pages)
    pages: list[PagePartitionSpec] = []
    page_touch_count: Counter[str] = Counter()
    page_ranks = _infer_page_ranks(
        raw_buckets, circuit, set(analysis.branch_abstraction.get("globalBiasNets", []))
    )

    for idx, (title, members) in enumerate(raw_buckets, start=1):
        pid = _slugify(title, idx)
        cname = f"{circuit.cell_name}_{pid}"
        s_rank = page_ranks[idx - 1]
        pages.append(
            PagePartitionSpec(
                page_id=pid,
                cell_name=cname,
                title=title,
                role="subsheet",
                stage_order=s_rank * 10 + idx,
                instances=members,
            )
        )
        nets_in_p = {
            n
            for r in members
            for n in inst_map[r].terminals.values()
            if n not in rails
        }
        for n in nets_in_p:
            page_touch_count[n] += 1

    top_io_nets = {p.net for p in circuit.ports if p.net not in rails}
    global_bias_nets = sorted(
        n for n, cnt in page_touch_count.items() if cnt >= 3 and n not in top_io_nets
    )

    return HierarchyIntent(
        top_cell=circuit.cell_name,
        pages=pages,
        global_bias_nets=global_bias_nets,
        authored_by="atomic_cluster_partitioner",
        notes=f"自动保护差分对/串叠/RC 原子簇，将 {len(circuit.instances)} 个器件切分为 {len(pages)} 张高可读子原理图。",
    )


def outline_flat_circuit_to_design(
    circuit: NetlistCircuit,
    h_intent: HierarchyIntent,
) -> tuple[NetlistDesign, dict[str, dict[str, list[str]]]]:
    """Transform a flat `NetlistCircuit` + `HierarchyIntent` into a multi-cell `NetlistDesign` + block pin sides."""
    inst_map = circuit.instance_map()
    supplies = set(circuit.supply_nets)
    grounds = set(circuit.ground_nets)
    rails = supplies | grounds
    top_ports_set = {p.net for p in circuit.ports}
    top_port_dirs = {p.net: p.direction for p in circuit.ports}

    # Verify complete & disjoint partition of instances
    covered: list[str] = [r for p in h_intent.pages for r in p.instances]
    if len(covered) != len(set(covered)) or set(covered) != set(inst_map):
        missing = sorted(set(inst_map) - set(covered))
        dup = [r for r, c in Counter(covered).items() if c > 1]
        raise ValueError(f"Invalid HierarchyIntent partition: missing={missing}, duplicates={dup}")

    # Count which pages touch each net, and which page drives each net (diode/drain/output)
    pages_of_net: dict[str, set[str]] = defaultdict(set)
    driver_pages_of_net: dict[str, set[str]] = defaultdict(set)
    for page in h_intent.pages:
        for r in page.instances:
            inst = inst_map[r]
            for pin, net in inst.terminals.items():
                pages_of_net[net].add(page.cell_name)
                if pin in {"D", "P", "M"} and inst.device_class != "mos":
                    driver_pages_of_net[net].add(page.cell_name)
                elif inst.device_class == "mos" and pin == "D":
                    driver_pages_of_net[net].add(page.cell_name)

    cells: dict[str, NetlistCircuit] = {}
    pin_sides_by_cell: dict[str, dict[str, list[str]]] = {}
    top_instances: list[InstanceIR] = []
    global_bias_set = set(h_intent.global_bias_nets)

    ordered_pages = sorted(h_intent.pages, key=lambda p: p.stage_order)
    for idx, page in enumerate(ordered_pages, start=1):
        sub_insts = [copy.deepcopy(inst_map[r]) for r in page.instances]
        sub_nets: list[str] = []
        seen_n: set[str] = set()
        for si in sub_insts:
            for n in si.terminals.values():
                if n not in seen_n:
                    seen_n.add(n)
                    sub_nets.append(n)

        # Interface nets = nets shared with another page OR top-level ports
        interface_nets = [
            n for n in sub_nets if (len(pages_of_net[n]) > 1 or n in top_ports_set)
        ]

        left_p: list[str] = []
        right_p: list[str] = []
        top_p: list[str] = []
        bot_p: list[str] = []
        sub_ports: list[PortIR] = []

        for n in interface_nets:
            if n in supplies:
                top_p.append(n)
                sub_ports.append(PortIR(name=n, net=n, direction="inout"))
            elif n in grounds:
                bot_p.append(n)
                sub_ports.append(PortIR(name=n, net=n, direction="inout"))
            elif n in global_bias_set:
                # If this page generates the global bias (diode/drain inside, no gate-only consumer), put on right; else bottom/left
                is_source_page = (page.cell_name in driver_pages_of_net[n]) and (page.stage_order <= 200)
                if is_source_page:
                    right_p.append(n)
                    sub_ports.append(PortIR(name=n, net=n, direction="output"))
                else:
                    bot_p.append(n)
                    sub_ports.append(PortIR(name=n, net=n, direction="input"))
            elif n in top_port_dirs:
                tdir = top_port_dirs[n]
                if tdir == "output":
                    right_p.append(n)
                else:
                    left_p.append(n)
                sub_ports.append(PortIR(name=n, net=n, direction=tdir))
            else:
                # Inter-page signal: if driven here and consumed by a later-stage page -> right output; else left input!
                driven_here = page.cell_name in driver_pages_of_net[n]
                later_consumers = any(
                    other_p.stage_order > page.stage_order and other_p.cell_name in pages_of_net[n]
                    for other_p in ordered_pages
                )
                if driven_here and later_consumers:
                    right_p.append(n)
                    sub_ports.append(PortIR(name=n, net=n, direction="output"))
                else:
                    left_p.append(n)
                    sub_ports.append(PortIR(name=n, net=n, direction="input"))

        cells[page.cell_name] = NetlistCircuit(
            cell_name=page.cell_name,
            ports=sub_ports,
            instances=sub_insts,
            nets=sub_nets,
            supply_nets=[n for n in sub_nets if n in supplies],
            ground_nets=[n for n in sub_nets if n in grounds],
        )
        pin_sides_by_cell[page.cell_name] = {
            "left": left_p,
            "right": right_p,
            "top": top_p,
            "bottom": bot_p,
        }
        top_instances.append(
            InstanceIR(
                ref=f"U{idx}_{page.page_id.upper()}",
                raw_name=f"XU{idx}_{page.page_id}",
                device_class="subckt",
                kind=None,
                polarity="block",
                source_model=page.cell_name,
                terminals={p.name: p.net for p in sub_ports},
                params={},
                section_hint=page.title,
                subckt_cell=page.cell_name,
            )
        )

    top_nets: list[str] = []
    seen_top: set[str] = set()
    for p in circuit.ports:
        if p.net not in seen_top:
            seen_top.add(p.net)
            top_nets.append(p.net)
    for ti in top_instances:
        for n in ti.terminals.values():
            if n not in seen_top:
                seen_top.add(n)
                top_nets.append(n)

    cells[h_intent.top_cell] = NetlistCircuit(
        cell_name=h_intent.top_cell,
        ports=list(circuit.ports),
        instances=top_instances,
        nets=top_nets,
        supply_nets=[n for n in top_nets if n in supplies],
        ground_nets=[n for n in top_nets if n in grounds],
    )
    return NetlistDesign(cells=cells, top_cell=h_intent.top_cell), pin_sides_by_cell


def compile_hierarchical_circuit(
    circuit_or_design: NetlistCircuit | NetlistDesign,
    *,
    pdk: PdkRegistry | None = None,
    hierarchy_intent: HierarchyIntent | None = None,
    max_page_size: int = 24,
    allow_wire_labels: bool = True,
    optimize_crossings: bool = True,
) -> HierarchyBuildResult:
    """Compile a large flat netlist (or multi-subckt `NetlistDesign`) bottom-up into verified multi-sheet schematics."""
    pdk_work = copy.deepcopy(pdk or PdkRegistry())

    if isinstance(circuit_or_design, NetlistCircuit):
        original_flat: NetlistCircuit | None = circuit_or_design
        h_intent = hierarchy_intent or propose_flat_partition(
            circuit_or_design, max_page_size=max_page_size
        )
        design, pin_sides_by_cell = outline_flat_circuit_to_design(circuit_or_design, h_intent)
    else:
        original_flat = None
        h_intent = hierarchy_intent
        design = circuit_or_design
        pin_sides_by_cell = {}
        for cname, c_cell in design.cells.items():
            if cname == design.top_cell:
                continue
            sups = set(c_cell.supply_nets)
            gnds = set(c_cell.ground_nets)
            pin_sides_by_cell[cname] = {
                "left": [p.name for p in c_cell.ports if p.net not in (sups | gnds) and p.direction != "output"],
                "right": [p.name for p in c_cell.ports if p.net not in (sups | gnds) and p.direction == "output"],
                "top": [p.name for p in c_cell.ports if p.net in sups],
                "bottom": [p.name for p in c_cell.ports if p.net in gnds],
            }

    # Register child block symbols into `pdk_work` before compiling `top_cell`
    for cname, sides in pin_sides_by_cell.items():
        pdk_work.register_block_symbol(
            cname,
            left_pins=sides["left"],
            right_pins=sides["right"],
            top_pins=sides["top"],
            bottom_pins=sides["bottom"],
        )
    for cell_obj in design.cells.values():
        pdk_work.ensure_subckt_symbols(cell_obj)

    # True topological bottom-up post-order DFS: leaf cells first, intermediate parents next, `design.top_cell` last
    visited_order: list[str] = []
    seen_cells: set[str] = set()

    def _visit_cell(cn: str) -> None:
        if cn in seen_cells or cn not in design.cells:
            return
        seen_cells.add(cn)
        for inst in design.cells[cn].instances:
            child_cn = inst.subckt_cell or inst.source_model
            if child_cn in design.cells and child_cn != cn:
                _visit_cell(child_cn)
        visited_order.append(cn)

    for cn in design.cells:
        if cn != design.top_cell:
            _visit_cell(cn)
    _visit_cell(design.top_cell)
    ordered_cells = visited_order
    solved_pages: dict[str, SolvedSchematic] = {}
    audits: dict[str, AuditReport] = {}
    manifest_circuits: list[dict[str, Any]] = []
    global_bias_set = set(h_intent.global_bias_nets) if h_intent else set()

    for cname in ordered_cells:
        cell = design.cells[cname]
        analysis = analyze_circuit(cell)
        intent = SchematicIntent.from_dict(analysis.draft_intent)
        rails = set(cell.supply_nets) | set(cell.ground_nets)

        if cname != design.top_cell:
            # Inside child sheets, pure-gate bias input ports use `local_repeat` so child sheets stay ultra-clean!
            for p in cell.ports:
                if p.net in global_bias_set and p.direction == "input":
                    conn_pins = [
                        pin
                        for inst in cell.instances
                        for pin, n_net in inst.terminals.items()
                        if n_net == p.net and pin != "B"
                    ]
                    if conn_pins and all(pin == "G" for pin in conn_pins):
                        intent.port_sides[p.name] = "local_repeat"
                        intent.net_policies[p.net] = "supply_local"
            # Multi-group internal bias nets inside a child sheet use continuous `perimeter_bus` wires
            # (never sever child-sheet devices into floating `label_stub` islands!)
            for n, fan in analysis.net_fanout.items():
                if n not in rails and fan >= 5 and n not in {p.net for p in cell.ports}:
                    intent.net_policies[n] = "perimeter_bus"
        if cname == design.top_cell:
            # On pure-block `TOP` diagram: arrange child blocks cleanly in multi-row grid (e.g. 3~4 blocks per row).
            # On mixed-level `TOP` sheets (child blocks + R/C/MOS primitives, e.g. SCDAC or Flash ADC), preserve
            # analyzer's 2D vertical stacking & array motifs!
            if cell.instances and all(i.device_class == "subckt" for i in cell.instances):
                blocks = [i.ref for i in cell.instances]
                cols_per_row = min(4, max(2, (len(blocks) + 1) // 2))
                intent.column_order_hints = []
                for idx_b, bref in enumerate(blocks):
                    col_slot = idx_b % cols_per_row
                    row_slot = idx_b // cols_per_row
                    intent.tier_overrides[bref] = 2 + row_slot * 2
                    while len(intent.column_order_hints) <= col_slot:
                        intent.column_order_hints.append([])
                    intent.column_order_hints[col_slot].append(bref)
                intent.spacing.inter_group_gap_x = 18
                intent.spacing.intra_group_gap_x = 16
                intent.spacing.inter_tier_gap_y = max(14, 10 + 2 * cols_per_row)
                port_nets_top = {p.net for p in cell.ports}
                for n, fan in analysis.net_fanout.items():
                    if n not in rails and fan >= 6:
                        intent.net_policies[n] = (
                            "perimeter_bus" if (n in port_nets_top or not allow_wire_labels) else "label_stub"
                        )
            for bnet in global_bias_set:
                if bnet in cell.nets:
                    intent.net_policies[bnet] = "label_stub" if allow_wire_labels else "perimeter_bus"

        if any(i.device_class == "subckt" for i in cell.instances) and not all(i.device_class == "subckt" for i in cell.instances):
            intent.spacing.inter_group_gap_x = max(intent.spacing.inter_group_gap_x, 14)
            intent.spacing.intra_group_gap_x = max(intent.spacing.intra_group_gap_x, 12)
            intent.spacing.inter_tier_gap_y = max(intent.spacing.inter_tier_gap_y, 14)

        page_allow_wl = allow_wire_labels if cname == design.top_cell else False
        if optimize_crossings and len(cell.instances) <= 22:
            solved = optimize_schematic_crossings(
                cell, intent, pdk=pdk_work, allow_wire_labels=page_allow_wl, max_evals=10,
                cost_fn=solved_quality_cost,
            )
        else:
            from schematic_gen.router import compile_schematic
            solved = compile_schematic(
                cell, intent, pdk=pdk_work, allow_wire_labels=page_allow_wl
            )

        solved_pages[cname] = solved
        audits[cname] = audit_solved_schematic(solved, pdk_work)

        # Immediately sync this cell's block symbol left/right pin order with its solved port Y coordinates
        # BEFORE any parent cell (at 2-level or N-level hierarchy depth) instantiates and routes to it!
        if cname in pin_sides_by_cell:
            sides = pin_sides_by_cell[cname]
            y_by_port: dict[str, float] = {}
            for po in solved.ports:
                if po.port_name not in y_by_port:
                    y_by_port[po.port_name] = float(po.y)
            sides["left"] = sorted(sides["left"], key=lambda pn: (y_by_port.get(pn, 1e6), pn))
            sides["right"] = sorted(sides["right"], key=lambda pn: (y_by_port.get(pn, 1e6), pn))
            pdk_work.register_block_symbol(
                cname,
                left_pins=sides["left"],
                right_pins=sides["right"],
                top_pins=sides["top"],
                bottom_pins=sides["bottom"],
            )
        single_m = build_bridge_manifest(solved, pdk_work)
        manifest_circuits.extend(single_m["circuits"])

    combined_manifest = {
        "schema": "virtuoso-bridge-exact-schematic-v1",
        "circuits": manifest_circuits,
    }
    # Verify combined multi-circuit manifest against Virtuoso Bridge / preflight
    bridge_check = verify_with_virtuoso_bridge_offline(
        combined_manifest, pdk_work.raw_map, pdk_work.process_name
    )
    if not bridge_check.get("passed", False):
        raise RuntimeError(f"Combined hierarchy manifest failed preflight: {bridge_check}")

    if original_flat is not None:
        flat_ok, flat_diffs = verify_hierarchy_flatten_lvs(
            original_flat, combined_manifest, design.top_cell
        )
    else:
        flat_ok, flat_diffs = True, []

    return HierarchyBuildResult(
        design=design,
        hierarchy_intent=h_intent,
        solved_pages=solved_pages,
        audits=audits,
        combined_manifest=combined_manifest,
        augmented_process_map=pdk_work.raw_map,
        flatten_lvs_passed=flat_ok,
        flatten_lvs_Diffs=flat_diffs,
    )


def import_hierarchy_to_virtuoso_live(
    result: HierarchyBuildResult,
    *,
    process_name: str = "sky130",
    profile: str | None = None,
    overwrite: bool = False,
) -> dict[str, Any]:
    """Execute the 3-step live Virtuoso bottom-up handshake:
    1) Import leaf child schematics;
    2) Run native `symbol_generate_from_schematic_skill(..., sort_pins='geometric')` and read back live `pinOffsets`;
    3) Re-compile `TOP` against real Virtuoso symbol offsets (<0.1s) and import `TOP/schematic`.
    """
    ensure_virtuoso_bridge_on_path()
    from virtuoso_bridge import VirtuosoClient
    from virtuoso_bridge.virtuoso.schematic.manifest import _read_master_offsets
    from virtuoso_bridge.virtuoso.symbol.generator import symbol_generate_from_schematic_skill

    client = VirtuosoClient.from_env(profile=profile)
    top_cell = result.design.top_cell
    leaf_cells = [c for c in result.solved_pages if c != top_cell]
    pdk_live = PdkRegistry(process_name=process_name)
    pdk_live.raw_map = copy.deepcopy(result.augmented_process_map)
    pdk_live.devices.update(pdk_live.raw_map["processes"][process_name].get("devices", {}))

    # Step 1: Import all leaf schematics
    leaf_import = client.schematic.import_manifest(
        result.combined_manifest,
        pdk_live.raw_map,
        processes=[process_name],
        cells=leaf_cells,
        overwrite=overwrite,
    )

    # Step 2: Generate native Virtuoso symbol for each leaf cell & read back exact live pinOffsets!
    out_lib = str(pdk_live.raw_map["processes"][process_name]["outputLibrary"])
    grid = float(pdk_live.raw_map["gridUnit"])
    synced_symbols: dict[str, Any] = {}
    for cname in leaf_cells:
        sym_skill = symbol_generate_from_schematic_skill(
            out_lib, cname, sort_pins="geometric", overwrite=overwrite
        )
        client.execute_skill(sym_skill, timeout=60)
        dev_key = f"subckt:{cname}"
        dev_entry = pdk_live.raw_map["processes"][process_name]["devices"][dev_key]
        live_offsets = _read_master_offsets(client, dev_entry, grid)
        dev_entry["pinOffsets"] = live_offsets
        pdk_live.devices[dev_key]["pinOffsets"] = live_offsets
        synced_symbols[cname] = live_offsets

    # Step 3: Re-compile ONLY `top_cell` with exact live symbol pinOffsets, then import `top_cell`!
    from schematic_gen.router import compile_schematic
    top_solved_prev = result.solved_pages[top_cell]
    top_recompiled = compile_schematic(
        result.design.cells[top_cell],
        top_solved_prev.intent,
        pdk=pdk_live,
        allow_wire_labels=False,
    )
    top_manifest = build_bridge_manifest(top_recompiled, pdk_live)
    top_import = client.schematic.import_manifest(
        top_manifest,
        pdk_live.raw_map,
        processes=[process_name],
        cells=[top_cell],
        overwrite=overwrite,
    )
    return {
        "leafImport": leaf_import,
        "syncedSymbols": synced_symbols,
        "topImport": top_import,
    }

# Split modules (re-exported for backward compatibility)
from schematic_gen.hierarchy_book import render_hierarchy_book_html  # noqa: E402,F401
from schematic_gen.hierarchy_lvs import verify_hierarchy_flatten_lvs  # noqa: E402,F401
