"""Shared, layer-neutral topology primitives, device families, and centralized thresholds.

Every layer (semantic analysis, branch abstraction, validation, placement, paging) imports its
port-role helpers and tunable thresholds from here instead of re-implementing them, so a rule changes in
exactly one place. This module depends only on `models` (no analyzer / placer / auditor imports).

Thresholds and device families can be overridden per PDK in `<pdk>_netlist_rules.json`:

    "topologyThresholds": {"localFanoutMax": 6, "hyperBusMinBranches": 5, ...},
    "deviceFamilies": {"resistor": ["res", "rpoly"], "capacitor": ["cap", "mim"]}
"""

from __future__ import annotations

from collections import defaultdict
from dataclasses import asdict, dataclass, fields
from functools import lru_cache
import json
from pathlib import Path
import re
from typing import Any

from schematic_gen.models import InstanceIR, NetlistCircuit

_DEFAULT_RULES_PATH = Path(__file__).resolve().parent.parent / "pdk" / "sky130_netlist_rules.json"


# ---------------------------------------------------------------------------------------------------------
# Centralized thresholds
# ---------------------------------------------------------------------------------------------------------
@dataclass(frozen=True)
class TopologyThresholds:
    """Single home for the tunable numbers previously scattered across analyzer / abstraction / validator."""

    # A net is "local" (eligible for cohesion / cut accounting) when its fanout is at most this.
    local_fanout_max: int = 6
    # A signal net touching at least this many folded branches is a shared bus (hyper-edge), not a pairwise link.
    hyper_bus_min_branches: int = 5
    # A diode/port bias net driving at least this many distinct branches is a GLOBAL bias rail.
    global_bias_min_branches: int = 3
    # Vertical folding only continues through tapped nodes with at most this many channel connections.
    fold_max_channel_degree: int = 5
    # Structural similarity (LCS ratio) for single-branch quasi-symmetric pairs / coupled mirror pairs.
    quasi_symmetry_min_similarity: float = 0.72
    coupled_mirror_min_similarity: float = 0.85
    # Declared mirror skeleton pairs below this similarity are rejected by validate-modules.
    skeleton_min_similarity: float = 0.5
    # Inter-module private-cut width that triggers a HIGH_INTER_MODULE_CUT warning.
    inter_module_cut_warn: int = 4
    # Pass 3 backbone is "not applicable" when the largest stage holds this fraction of devices (>= 2 stages)
    # or when a single stage covers a circuit of at least `backbone_single_stage_min_devices` devices.
    backbone_degenerate_fraction: float = 0.8
    backbone_single_stage_min_devices: int = 20
    # Digest: full branch table up to this many branches, compact listing beyond.
    digest_full_table_max_branches: int = 30
    # Digest: inline signal-edge map up to this many branches (otherwise point to `inspect`).
    digest_signal_edges_max_branches: int = 20
    # Naming (reference numbers) may only break exact topological ties; weight is kept below every
    # topology increment used by the column/macro scorers.
    naming_tiebreak_weight: float = 0.5
    # Legacy naming-order layout moves (e.g. insert singleton M5 between stacks M1-4 / M6-9 because 4 < 5 < 6).
    # Disabled by default: layout must follow topology; enable only to reproduce old drawings.
    allow_naming_layout_rules: bool = False

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


_CAMEL_RE = re.compile(r"(?<!^)(?=[A-Z])")


def _to_snake(key: str) -> str:
    return _CAMEL_RE.sub("_", key).lower()


@lru_cache(maxsize=8)
def _load_rules(rules_path: str) -> dict[str, Any]:
    p = Path(rules_path)
    if not p.is_file():
        return {}
    return json.loads(p.read_text(encoding="utf-8"))


def load_thresholds(rules_path: Path | str | None = None) -> TopologyThresholds:
    """Defaults overridden by `topologyThresholds` (camelCase or snake_case) in the netlist rules file."""
    raw = _load_rules(str(rules_path or _DEFAULT_RULES_PATH)).get("topologyThresholds", {}) or {}
    allowed = {f.name: f.type for f in fields(TopologyThresholds)}
    kwargs: dict[str, Any] = {}
    for k, v in raw.items():
        sk = k if k in allowed else _to_snake(k)
        if sk in allowed:
            default_v = getattr(TopologyThresholds(), sk)
            if isinstance(default_v, bool):
                kwargs[sk] = v if isinstance(v, bool) else str(v).strip().lower() in {"1", "true", "yes", "on"}
            else:
                kwargs[sk] = type(default_v)(v)
    return TopologyThresholds(**kwargs)


THRESHOLDS = load_thresholds()


# ---------------------------------------------------------------------------------------------------------
# Device families (PDK-agnostic; extendable per PDK via `deviceFamilies`)
# ---------------------------------------------------------------------------------------------------------
_DEFAULT_FAMILIES: dict[str, tuple[str, ...]] = {
    "resistor": ("res",),
    "capacitor": ("cap",),
}


@lru_cache(maxsize=8)
def _family_prefixes(rules_path: str) -> dict[str, tuple[str, ...]]:
    fam = dict(_DEFAULT_FAMILIES)
    for k, v in (_load_rules(rules_path).get("deviceFamilies", {}) or {}).items():
        fam[str(k)] = tuple(dict.fromkeys([*fam.get(str(k), ()), *[str(x).lower() for x in v]]))
    return fam


def device_family(device_class: str, rules_path: Path | str | None = None) -> str:
    """Map a parsed `device_class` (e.g. `res_poly_3t`, `cap_mim_m4`, `resistor`) to a family name."""
    dc = (device_class or "").lower()
    if dc in {"mos", "bjt", "diode", "isource", "vsource", "subckt"}:
        return dc
    for fam, prefixes in _family_prefixes(str(rules_path or _DEFAULT_RULES_PATH)).items():
        if any(dc.startswith(p) for p in prefixes):
            return fam
    return dc


def is_resistor_class(device_class: str) -> bool:
    return device_family(device_class) == "resistor"


def is_capacitor_class(device_class: str) -> bool:
    return device_family(device_class) == "capacitor"


def is_nonpolar_2pin_class(device_class: str) -> bool:
    """Resistors and (non-electrolytic) capacitors of any PDK naming scheme."""
    return device_family(device_class) in {"resistor", "capacitor"}


# ---------------------------------------------------------------------------------------------------------
# Naming tie-break helpers (ONLY for exact topological ties)
# ---------------------------------------------------------------------------------------------------------
def ref_num(ref: str) -> int:
    m = re.search(r"(\d+)$", ref)
    return int(m.group(1)) if m else 10**9


def nat_key(text: str) -> tuple[Any, ...]:
    return tuple(int(p) if p.isdigit() else p.lower() for p in re.split(r"(\d+)", text))


def lcs_len(seq_a: list[str], seq_b: list[str]) -> int:
    m, n = len(seq_a), len(seq_b)
    if m == 0 or n == 0:
        return 0
    dp = [0] * (n + 1)
    for a in seq_a:
        prev = 0
        for j, b in enumerate(seq_b, start=1):
            cur = dp[j]
            if a == b:
                dp[j] = prev + 1
            elif dp[j - 1] > dp[j]:
                dp[j] = dp[j - 1]
            prev = cur
    return dp[n]


# ---------------------------------------------------------------------------------------------------------
# Port-role primitives
# ---------------------------------------------------------------------------------------------------------
def is_passive_device(inst: InstanceIR) -> bool:
    return is_nonpolar_2pin_class(inst.device_class) or inst.device_class == "diode"


def is_cap_device(inst: InstanceIR, device_roles: dict[str, dict[str, Any]]) -> bool:
    return is_capacitor_class(inst.device_class) or device_roles.get(inst.ref, {}).get("role") == "mos_cap"


def ctrl_pin_of(inst: InstanceIR, device_roles: dict[str, dict[str, Any]]) -> str | None:
    role = device_roles.get(inst.ref, {}).get("role", "")
    if inst.device_class == "mos" and role != "mos_cap":
        return "G"
    if inst.device_class == "bjt":
        return "BASE"
    return None


def channel_pins_of(inst: InstanceIR, device_roles: dict[str, dict[str, Any]]) -> tuple[str, ...]:
    role = device_roles.get(inst.ref, {}).get("role", "")
    if inst.device_class == "mos":
        if role == "mos_cap":
            return ("G", "D")
        return tuple(p for p in ("D", "S") if p in inst.terminals)
    if inst.device_class == "bjt":
        return tuple(p for p in ("C", "E") if p in inst.terminals)
    if inst.device_class == "subckt":
        return ()
    return tuple(p for p in ("P", "M") if p in inst.terminals)


def top_bot_pins_of(inst: InstanceIR) -> tuple[str | None, str | None]:
    if inst.device_class == "mos":
        return ("S", "D") if inst.polarity == "pmos" else ("D", "S")
    if inst.device_class == "bjt":
        return ("E", "C") if inst.polarity == "pnp" else ("C", "E")
    if inst.device_class in {"isource", "vsource", "diode"}:
        return ("P", "M")
    return (None, None)


def is_diode_connected(
    inst: InstanceIR,
    rails: set[str],
    device_roles: dict[str, dict[str, Any]],
    stack_by_ref: dict[str, dict[str, Any]],
) -> bool:
    role = device_roles.get(inst.ref, {}).get("role", "")
    if role in {"mos_diode", "bjt_diode"}:
        return True
    if inst.device_class == "mos":
        d = inst.terminals.get("D", "")
        g = inst.terminals.get("G", "")
        s = inst.terminals.get("S", "")
        if d and d == g and d != s and d not in rails:
            return True
        if stack_by_ref.get(inst.ref, {}).get("is_diode_master"):
            return True
    elif inst.device_class == "bjt":
        c = inst.terminals.get("C", "")
        b = inst.terminals.get("BASE", "")
        e = inst.terminals.get("E", "")
        if c and c == b and c != e and c not in rails:
            return True
    return False


# ---------------------------------------------------------------------------------------------------------
# Net-policy provenance & priority (who may overwrite whom)
# ---------------------------------------------------------------------------------------------------------
# Higher wins. The LLM / designer always has the last word; skeleton expansion next; the crossing optimizer
# may only touch policies set by the analyzer defaults (or itself).
NET_POLICY_PRIORITY: dict[str, int] = {"llm": 4, "skeleton": 3, "optimizer": 2, "analyzer": 1}


def net_policy_may_override(current_source: str, writer: str) -> bool:
    """True when `writer` is allowed to replace a policy currently owned by `current_source`."""
    return NET_POLICY_PRIORITY.get(writer, 0) >= NET_POLICY_PRIORITY.get(current_source or "analyzer", 1)


# ---------------------------------------------------------------------------------------------------------
# Default potential tiers & co-gated Super-MOS stacks (moved from analyzer; used by analyzer AND placer)
# ---------------------------------------------------------------------------------------------------------
def assign_default_tiers(circuit: NetlistCircuit) -> dict[str, int]:
    """Assign vertical potential tiers (1 = top PMOS near VDD .. 5 = bottom NMOS near VSS)."""
    supplies = set(circuit.supply_nets)
    grounds = set(circuit.ground_nets)
    tiers: dict[str, int] = {}

    for inst in circuit.instances:
        if inst.device_class == "subckt":
            tiers[inst.ref] = 3
        elif inst.polarity == "pmos":
            # PMOS with Source directly on VDD -> Tier 1 (PMOS_TOP); cascode PMOS -> Tier 2 (PMOS_CAS)
            tiers[inst.ref] = 1 if inst.terminals.get("S") in supplies else 2
        elif inst.polarity == "pnp":
            tiers[inst.ref] = 1 if inst.terminals.get("E") in supplies else 2
        elif inst.polarity == "npn":
            if inst.terminals.get("E") in grounds:
                tiers[inst.ref] = 5
            elif inst.terminals.get("C") in supplies:
                tiers[inst.ref] = 2
            else:
                tiers[inst.ref] = 4
        elif inst.polarity == "nmos":
            # NMOS with Source directly on VSS -> Tier 5 (NMOS_BOT);
            # NMOS source follower / LDO pass FET with Drain directly on VDD (and S != VSS) -> Tier 2 (above mid passives);
            # otherwise diff-pair/cascode NMOS -> Tier 4 (NMOS_MID)
            if inst.terminals.get("S") in grounds:
                tiers[inst.ref] = 5
            elif inst.terminals.get("D") in supplies:
                tiers[inst.ref] = 2
            else:
                tiers[inst.ref] = 4
        elif inst.device_class in {"isource", "vsource"}:
            p_net = inst.terminals.get("P", "")
            m_net = inst.terminals.get("M", "")
            if p_net in supplies and m_net not in grounds:
                tiers[inst.ref] = 1
            elif m_net in grounds and p_net not in supplies:
                tiers[inst.ref] = 5
            else:
                tiers[inst.ref] = 3
        else:
            # Passives (resistors, capacitors, diodes)
            p_net = inst.terminals.get("P", "")
            m_net = inst.terminals.get("M", "")
            if p_net in supplies or m_net in supplies:
                tiers[inst.ref] = 1
            elif p_net in grounds or m_net in grounds:
                tiers[inst.ref] = 5
            else:
                # Floating / inter-stage signal network (e.g. Miller Rz/Cc) -> Mid-rail Tier 3
                tiers[inst.ref] = 3

    # Pin-Facing Vertical Flow Propagation:
    # Even when a supply/ground rail has a custom name (e.g. VDDH, GNDA), if MOS `u`'s
    # bottom vertical pin (`D` for PMOS, `S` for NMOS) drives MOS `v`'s top vertical pin
    # (`S` for PMOS, `D` for NMOS) on a low-fanout non-rail net, `u` must sit above `v`!
    rails = supplies | grounds
    net_fanout: dict[str, int] = defaultdict(int)
    for inst in circuit.instances:
        for p, n in inst.terminals.items():
            if p != "B" and n not in rails:
                net_fanout[n] += 1

    mos_list = [inst for inst in circuit.instances if inst.device_class == "mos"]
    for u in mos_list:
        u_bot_pin = "D" if u.polarity == "pmos" else "S"
        u_bot_net = u.terminals.get(u_bot_pin, "")
        if not u_bot_net or u_bot_net in rails or net_fanout.get(u_bot_net, 99) > 4:
            continue
        for v in mos_list:
            if v.ref == u.ref:
                continue
            v_top_pin = "S" if v.polarity == "pmos" else "D"
            if v.terminals.get(v_top_pin) != u_bot_net:
                continue
            # Same-polarity vertical series/tail connection (e.g. PMOS tail M19 -> PMOS diff pair M20/M21,
            # or PMOS M6 -> PMOS M5, or NMOS diff pair -> NMOS tail)
            if u.polarity == v.polarity == "pmos" and tiers[u.ref] >= tiers[v.ref]:
                # Promote upper PMOS to Tier 1 when its source is NOT driven by another PMOS above it
                u_top_net = u.terminals.get("S", "")
                has_pmos_above_u = any(
                    w.ref != u.ref
                    and w.polarity == "pmos"
                    and w.terminals.get("D") == u_top_net
                    and u_top_net not in rails
                    for w in mos_list
                )
                if not has_pmos_above_u:
                    tiers[u.ref] = 1
            elif u.polarity == v.polarity == "nmos" and tiers[u.ref] >= tiers[v.ref]:
                # Demote lower NMOS to Tier 5 when its source is NOT driving another NMOS below it
                v_bot_net = v.terminals.get("S", "")
                has_nmos_below_v = any(
                    w.ref != v.ref
                    and w.polarity == "nmos"
                    and w.terminals.get("D") == v_bot_net
                    and v_bot_net not in rails
                    for w in mos_list
                )
                if not has_nmos_below_v:
                    tiers[v.ref] = 5

    return tiers


def detect_co_gated_mos_stacks(circuit: NetlistCircuit) -> list[dict[str, Any]]:
    """Detect maximal co-gated series-stacked composite MOS chains (`k >= 2`).

    In analog circuits (e.g. composite long-L current mirrors `M10-M13` ↔ `M14-M17`),
    series-connected same-polarity MOS devices sharing one Gate net act as an atomic
    multi-device group that must stay together vertically, pair by group priority
    (longest matched stack & master-slave first), and align row-by-row horizontally.
    """
    rails = set(circuit.supply_nets) | set(circuit.ground_nets)
    supplies = set(circuit.supply_nets)
    grounds = set(circuit.ground_nets)
    mos_list = [i for i in circuit.instances if i.device_class == "mos"]
    if len(mos_list) < 2:
        return []

    # Count non-bulk terminal incidence per net so only true exclusive series links chain
    net_terms: dict[str, list[tuple[str, str]]] = defaultdict(list)
    for inst in circuit.instances:
        for pin, net in inst.terminals.items():
            if pin != "B":
                net_terms[net].append((inst.ref, pin))

    by_pol_gate: dict[tuple[str, str], list[InstanceIR]] = defaultdict(list)
    for m in mos_list:
        g_net = m.terminals.get("G", "")
        if g_net and g_net not in rails:
            by_pol_gate[(m.polarity, g_net)].append(m)

    stacks: list[dict[str, Any]] = []
    for (pol, g_net), members in sorted(by_pol_gate.items()):
        if len(members) < 2:
            continue
        top_pin, bot_pin = ("S", "D") if pol == "pmos" else ("D", "S")
        by_top: dict[str, list[InstanceIR]] = defaultdict(list)
        by_bot: dict[str, list[InstanceIR]] = defaultdict(list)
        for m in members:
            by_top[m.terminals[top_pin]].append(m)
            by_bot[m.terminals[bot_pin]].append(m)

        # Build strict 1-to-1 series edges `u -> v` (u above v) across exclusive internal nodes
        nxt_of: dict[str, str] = {}
        prv_of: dict[str, str] = {}
        for u in members:
            mid = u.terminals[bot_pin]
            if mid in rails:
                continue
            # Internal series node must exclusively link u.bot_pin and one v.top_pin (ignore bulk)
            inc = net_terms.get(mid, [])
            if len(inc) != 2:
                continue
            cands = [v for v in by_top.get(mid, []) if v.ref != u.ref]
            if len(cands) == 1 and len(by_bot.get(mid, [])) == 1:
                v = cands[0]
                if u.ref not in nxt_of and v.ref not in prv_of:
                    nxt_of[u.ref] = v.ref
                    prv_of[v.ref] = u.ref

        mem_map = {m.ref: m for m in members}
        visited: set[str] = set()
        for m in members:
            if m.ref in visited or m.ref in prv_of:
                continue
            chain: list[str] = []
            cur: str | None = m.ref
            while cur is not None and cur not in visited:
                visited.add(cur)
                chain.append(cur)
                cur = nxt_of.get(cur)
            if len(chain) >= 2:
                top_inst = mem_map[chain[0]]
                bot_inst = mem_map[chain[-1]]
                anchor_rail = ""
                if pol == "pmos" and top_inst.terminals.get("S") in supplies:
                    anchor_rail = top_inst.terminals["S"]
                elif pol == "nmos" and bot_inst.terminals.get("S") in grounds:
                    anchor_rail = bot_inst.terminals["S"]
                is_master = any(
                    mem_map[r].terminals.get("D") == g_net for r in chain
                )
                stacks.append(
                    {
                        "refs": chain,
                        "polarity": pol,
                        "gate_net": g_net,
                        "anchor_rail": anchor_rail,
                        "anchor_side": "top" if pol == "pmos" else "bot",
                        "is_diode_master": is_master,
                        "length": len(chain),
                    }
                )
    return stacks
