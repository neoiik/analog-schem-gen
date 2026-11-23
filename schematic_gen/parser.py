"""Netlist parser supporting SKY130/SPICE/CDL/Spectre, `.param` algebraic expressions, and multi-subckt designs."""

from __future__ import annotations

import ast
import json
import math
from pathlib import Path
import re
from typing import Any

from schematic_gen.models import InstanceIR, NetlistCircuit, NetlistDesign, PortIR
from schematic_gen.pdk import DEFAULT_NETLIST_RULES_PATH


_SPECTRE_INST_RE = re.compile(
    r"^([A-Za-z_][A-Za-z0-9_]*)\s*\(([^)]+)\)\s*([A-Za-z_][A-Za-z0-9_.]*)(.*)$"
)
_SPICE_NUM_RE = re.compile(
    r"^([+-]?(?:\d+\.?\d*|\.\d+)(?:[eE][+-]?\d+)?)(meg|mil|t|g|k|m|u|n|p|f)?([A-Za-z]*)$",
    re.IGNORECASE,
)
_SUFFIX_SCALE: dict[str, float] = {
    "t": 1e12,
    "g": 1e9,
    "meg": 1e6,
    "k": 1e3,
    "m": 1e-3,
    "mil": 25.4e-6,
    "u": 1e-6,
    "n": 1e-9,
    "p": 1e-12,
    "f": 1e-15,
}


def parse_spice_float(token: str) -> float | None:
    """Parse a SPICE numeric literal with optional suffix (`9.1meg`, `0.45p`, `4.7k`)."""
    s = token.strip().strip("'\"{}")
    if not s:
        return None
    try:
        return float(s)
    except ValueError:
        pass
    m = _SPICE_NUM_RE.match(s)
    if not m:
        return None
    base = float(m.group(1))
    suf = (m.group(2) or "").lower()
    scale = _SUFFIX_SCALE.get(suf, 1.0)
    return base * scale


def eval_spice_expr(expr: str, env: dict[str, float]) -> float | None:
    """Safely evaluate a SPICE algebraic parameter expression (`{48*a2_tail}`, `{0.8*a2_cc}`)."""
    raw = expr.strip().strip("'\"{}")
    direct = parse_spice_float(raw)
    if direct is not None:
        return direct
    # Replace inline SPICE suffixed literals inside algebraic formulas (e.g. `2*1.5k`)
    def _sub_lit(match: re.Match[str]) -> str:
        val = parse_spice_float(match.group(0))
        return repr(val) if val is not None else match.group(0)

    cleaned = re.sub(
        r"(?<![A-Za-z0-9_])(\d+\.?\d*|\.\d+)(meg|t|g|k|m|u|n|p|f)\b",
        _sub_lit,
        raw,
        flags=re.IGNORECASE,
    )
    try:
        tree = ast.parse(cleaned, mode="eval")
    except SyntaxError:
        return None

    def _eval(node: ast.AST) -> float:
        if isinstance(node, ast.Expression):
            return _eval(node.body)
        if isinstance(node, ast.Constant) and isinstance(node.value, (int, float)):
            return float(node.value)
        if isinstance(node, ast.Name):
            key = node.id.lower()
            if key in env:
                return float(env[key])
            raise KeyError(node.id)
        if isinstance(node, ast.UnaryOp) and isinstance(node.op, (ast.UAdd, ast.USub)):
            v = _eval(node.operand)
            return v if isinstance(node.op, ast.UAdd) else -v
        if isinstance(node, ast.BinOp):
            left = _eval(node.left)
            right = _eval(node.right)
            if isinstance(node.op, ast.Add):
                return left + right
            if isinstance(node.op, ast.Sub):
                return left - right
            if isinstance(node.op, ast.Mult):
                return left * right
            if isinstance(node.op, ast.Div):
                return left / right
            if isinstance(node.op, ast.Pow):
                return left ** right
        raise ValueError("unsupported expression node")

    try:
        res = _eval(tree)
        return res if math.isfinite(res) else None
    except Exception:
        return None


def format_eng(value: float, unit_suffix: str = "") -> str:
    """Format a float into compact engineering notation (`21.5k`, `8.16p`, `9.1M`)."""
    if value == 0:
        return f"0{unit_suffix}"
    abs_v = abs(value)
    tiers = [
        (1e9, "G"),
        (1e6, "M"),
        (1e3, "k"),
        (1.0, ""),
        (1e-3, "m"),
        (1e-6, "u"),
        (1e-9, "n"),
        (1e-12, "p"),
        (1e-15, "f"),
    ]
    for scale, prefix in tiers:
        if abs_v >= scale * 0.999999:
            scaled = value / scale
            return f"{scaled:.4g}{prefix}{unit_suffix}"
    return f"{value:.4g}{unit_suffix}"


def _join_continuations(lines: list[str]) -> list[tuple[int, str, str]]:
    """Join `+` / `\\` continuations while tracking section comments `(lineno, text, section_hint)`."""
    logical: list[tuple[int, str, str]] = []
    pininfo_lines: list[tuple[int, str, str]] = []
    current_section = ""

    for lineno, raw in enumerate(lines, start=1):
        stripped = raw.strip()
        if not stripped:
            continue
        if stripped.upper().startswith("*.PININFO"):
            pininfo_lines.append((lineno, stripped, current_section))
            continue
        if stripped.startswith("*") or stripped.startswith("//") or stripped.startswith(";"):
            comment_body = stripped.lstrip("*/; ").strip()
            # Keep concise human section titles (skip separator bars or copyright boilerplate)
            if (
                comment_body
                and len(comment_body) <= 72
                and not comment_body.lower().startswith(("copyright", "licensed", "unless", "see the", "http", "===", "---"))
            ):
                current_section = comment_body
            continue
        for marker in (" //", " $ ", " ;"):
            if marker in stripped:
                stripped = stripped.split(marker, 1)[0].strip()
        if not stripped:
            continue
        if stripped.startswith("+") and logical:
            prev_no, prev_txt, prev_sec = logical[-1]
            logical[-1] = (prev_no, prev_txt + " " + stripped[1:].strip(), prev_sec)
            continue
        if logical and logical[-1][1].endswith("\\"):
            prev_no, prev_txt, prev_sec = logical[-1]
            logical[-1] = (prev_no, prev_txt[:-1].rstrip() + " " + stripped, prev_sec)
            continue
        logical.append((lineno, stripped, current_section))

    return pininfo_lines + logical


def _tokenize_preserving_braces(text: str) -> list[str]:
    """Split whitespace tokens while keeping `{a + b}` brace expressions together."""
    out: list[str] = []
    cur: list[str] = []
    depth = 0
    for ch in text:
        if ch == "{":
            depth += 1
            cur.append(ch)
        elif ch == "}":
            depth = max(0, depth - 1)
            cur.append(ch)
        elif ch.isspace() and depth == 0:
            if cur:
                out.append("".join(cur))
                cur = []
        else:
            cur.append(ch)
    if cur:
        out.append("".join(cur))
    return out


def _parse_params(tokens: list[str]) -> tuple[list[str], dict[str, str]]:
    """Split positional tokens from `key=value` tokens (preserving `{...}` expressions)."""
    positional: list[str] = []
    params: dict[str, str] = {}
    joined = " ".join(tokens)
    normalized = re.sub(r"\s*=\s*", "=", joined)
    for tok in _tokenize_preserving_braces(normalized):
        if "=" in tok and not tok.startswith("="):
            k, v = tok.split("=", 1)
            params[k.lower()] = v.strip("'\"")
        else:
            positional.append(tok)
    return positional, params


def _normalize_instance_params(
    params: dict[str, str],
    env: dict[str, float],
    default_unit: str = "um",
) -> tuple[dict[str, str], dict[str, str]]:
    """Normalize & evaluate SPICE/SKY130 parameters, returning `(params, param_expressions)`."""
    out: dict[str, str] = {}
    exprs: dict[str, str] = {}

    def _eval_or_raw(k: str) -> tuple[str | None, float | None]:
        raw = params.get(k)
        if raw is None or raw == "":
            return None, None
        clean = raw.strip()
        if "{" in clean or any(ch.isalpha() for ch in clean if ch.lower() not in "tgmkunpf."):
            exprs[k] = clean
        val = eval_spice_expr(clean, env)
        return clean.strip("{}"), val

    for dim_k in ("w", "l"):
        raw_s, val = _eval_or_raw(dim_k)
        if raw_s is None:
            continue
        if val is not None and val > 0:
            if val < 1e-3:
                out[dim_k] = f"{val * 1e6:.4g}u"
            elif default_unit == "um":
                out[dim_k] = f"{val:.4g}u"
            else:
                out[dim_k] = f"{val:.4g}"
        else:
            out[dim_k] = raw_s

    nf_raw, nf_val = _eval_or_raw("nf")
    out["nf"] = str(int(round(nf_val))) if nf_val is not None else str(nf_raw or "1")

    if "mf" in params:
        mf_raw, mf_val = _eval_or_raw("mf")
        out["mf"] = str(int(round(mf_val))) if mf_val is not None else str(mf_raw)

    m_key = "m" if "m" in params else ("mult" if "mult" in params else ("mf" if "mf" in params else None))
    if m_key:
        m_raw, m_val = _eval_or_raw(m_key)
        out["m"] = str(int(round(m_val))) if m_val is not None else str(m_raw)
    else:
        out["m"] = "1"

    for val_k in ("r", "c", "dc", "idc", "vdc", "area", "perim"):
        if val_k in params:
            raw_s, fval = _eval_or_raw(val_k)
            if fval is not None and val_k in {"r", "c", "dc", "idc", "vdc"}:
                out[val_k] = format_eng(fval)
            elif raw_s is not None:
                out[val_k] = raw_s

    return out, exprs


def _infer_port_direction(port_name: str, is_supply: bool, is_ground: bool) -> str:
    if is_supply or is_ground:
        return "inout"
    up = port_name.upper()
    if any(
        up.startswith(p)
        for p in ("VIN", "IN", "VIP", "VIM", "INN", "INP", "IBIAS", "VBIAS", "VB", "EN", "CLK", "RST", "VREF", "SW")
    ):
        return "input"
    if any(up.startswith(p) for p in ("VOUT", "OUT", "VOP", "VOM", "IOUT", "Q")):
        return "output"
    return "inout"


def _clean_ref_name(raw_name: str, device_class: str, is_matched_pdk: bool, existing_refs: set[str]) -> str:
    """Strip redundant leading `X` on PDK primitive subcircuits (`XM1 -> M1`, `XPOUT -> POUT`)."""
    candidate = raw_name
    if len(raw_name) >= 3 and raw_name[0].upper() == "X" and device_class != "subckt":
        second = raw_name[1].upper()
        if is_matched_pdk or (device_class == "mos" and second == "M") or (
            device_class.startswith("res") and second == "R"
        ) or (
            device_class.startswith("cap") and second == "C"
        ) or (
            device_class == "diode" and second == "D"
        ):
            stripped = raw_name[1:]
            if stripped not in existing_refs:
                candidate = stripped
    if candidate in existing_refs:
        idx = 1
        while f"{candidate}_{idx}" in existing_refs:
            idx += 1
        candidate = f"{candidate}_{idx}"
    return candidate


def parse_netlist_design_text(
    text: str,
    *,
    rules_path: Path | str = DEFAULT_NETLIST_RULES_PATH,
    top_cell_override: str | None = None,
) -> NetlistDesign:
    """Parse single-cell or multi-subcircuit hierarchical netlist into `NetlistDesign`."""
    rules_data: dict[str, Any] = json.loads(Path(rules_path).read_text(encoding="utf-8"))
    default_unit = str(rules_data.get("defaultDimensionUnit", "um"))
    supply_res = [re.compile(p, re.IGNORECASE) for p in rules_data.get("supplyNetPatterns", [])]
    ground_res = [re.compile(p, re.IGNORECASE) for p in rules_data.get("groundNetPatterns", [])]
    model_rules = [
        (re.compile(r["pattern"], re.IGNORECASE), r)
        for r in rules_data.get("modelRules", [])
    ]

    logical_lines = _join_continuations(text.splitlines())

    # First pass: collect `.param` / `parameters` definitions & `.subckt` port signatures
    global_params: dict[str, float] = {}
    subckt_headers: dict[str, list[str]] = {}
    pin_directions_global: dict[str, str] = {}

    for _lineno, line, _sec in logical_lines:
        upper = line.upper()
        if upper.startswith("*.PININFO"):
            for item in line.split()[1:]:
                if ":" in item:
                    pname, pdir = item.split(":", 1)
                    dir_map = {"I": "input", "O": "output", "B": "inout", "IO": "inout"}
                    pin_directions_global[pname] = dir_map.get(pdir.upper(), "inout")
        elif upper.startswith(".PARAM ") or upper.startswith("PARAMETERS "):
            _, pmap = _parse_params(_tokenize_preserving_braces(line)[1:])
            for pk, pv in pmap.items():
                fval = eval_spice_expr(pv, global_params)
                if fval is not None:
                    global_params[pk.lower()] = fval
        elif upper.startswith(".SUBCKT ") or upper.startswith("SUBCKT "):
            toks, _ = _parse_params(_tokenize_preserving_braces(line)[1:])
            if toks:
                subckt_headers[toks[0]] = list(toks[1:])

    # Partition lines into subckt bodies (or one top-level flat body if no `.subckt`)
    raw_blocks: list[tuple[str, list[str], list[tuple[int, str, str]]]] = []
    cur_name: str | None = None
    cur_ports: list[str] = []
    cur_body: list[tuple[int, str, str]] = []
    top_loose_body: list[tuple[int, str, str]] = []

    for lineno, line, sec in logical_lines:
        upper = line.upper()
        if upper.startswith("*.PININFO") or upper.startswith(".PARAM ") or upper.startswith("PARAMETERS "):
            continue
        if upper.startswith(".SUBCKT ") or upper.startswith("SUBCKT "):
            toks, _ = _parse_params(_tokenize_preserving_braces(line)[1:])
            cur_name = toks[0]
            cur_ports = list(toks[1:])
            cur_body = []
            continue
        if upper.startswith(".ENDS") or upper == "ENDS" or upper.startswith("ENDS "):
            if cur_name is not None:
                raw_blocks.append((cur_name, cur_ports, cur_body))
                cur_name = None
            continue
        if line.startswith(".") or upper.startswith(("SIMULATOR ", "GLOBAL ", "INCLUDE ", "MODEL ")):
            continue
        if cur_name is not None:
            cur_body.append((lineno, line, sec))
        else:
            top_loose_body.append((lineno, line, sec))

    if top_loose_body and not raw_blocks:
        raw_blocks.append((top_cell_override or "top_schematic", [], top_loose_body))
    elif top_loose_body and raw_blocks:
        # Loose top-level testbench/top instances instantiating defined subckts
        raw_blocks.append((top_cell_override or "top_schematic", [], top_loose_body))

    cells: dict[str, NetlistCircuit] = {}
    for b_name, declared_ports, body_lines in raw_blocks:
        instances: list[InstanceIR] = []
        existing_refs: set[str] = set()

        for lineno, line, sec in body_lines:
            spectre_match = _SPECTRE_INST_RE.match(line)
            if spectre_match:
                raw_name = spectre_match.group(1)
                pin_nets = spectre_match.group(2).split()
                model_name = spectre_match.group(3)
                _, raw_params = _parse_params(_tokenize_preserving_braces(spectre_match.group(4)))
            else:
                tokens, raw_params = _parse_params(_tokenize_preserving_braces(line))
                if len(tokens) < 3:
                    continue
                raw_name = tokens[0]
                prefix = raw_name[0].upper()

                if (
                    prefix in {"R", "C", "I", "V"}
                    and len(tokens) == 4
                    and tokens[-1] not in subckt_headers
                    and not any(rx.match(tokens[-1]) for rx, _ in model_rules)
                ):
                    pin_nets = tokens[1:3]
                    val_token = tokens[3]
                    model_name = prefix
                    if val_token.upper() != "DC":
                        val_key = {"R": "r", "C": "c", "I": "dc", "V": "dc"}[prefix]
                        raw_params.setdefault(val_key, val_token)
                elif prefix in {"I", "V"} and len(tokens) >= 5 and tokens[3].upper() == "DC":
                    pin_nets = tokens[1:3]
                    model_name = prefix
                    raw_params.setdefault("dc", tokens[4])
                else:
                    model_name = tokens[-1]
                    pin_nets = tokens[1:-1]

            # Check if `model_name` is a hierarchical child `.subckt` in this design!
            if model_name in subckt_headers:
                expected_ports = subckt_headers[model_name]
                if len(pin_nets) != len(expected_ports):
                    raise ValueError(
                        f"Line {lineno} ({raw_name} : subckt {model_name}): expected {len(expected_ports)} "
                        f"pins {expected_ports}, got {len(pin_nets)}"
                    )
                ref = _clean_ref_name(raw_name, "subckt", False, existing_refs)
                existing_refs.add(ref)
                norm_params, exprs = _normalize_instance_params(raw_params, global_params, default_unit)
                instances.append(
                    InstanceIR(
                        ref=ref,
                        raw_name=raw_name,
                        device_class="subckt",
                        kind=None,
                        polarity="block",
                        source_model=model_name,
                        terminals=dict(zip(expected_ports, pin_nets)),
                        params=norm_params,
                        param_expressions=exprs,
                        section_hint=sec,
                        subckt_cell=model_name,
                    )
                )
                continue

            matched_rule: dict[str, Any] | None = None
            for rx, rule in model_rules:
                if rx.match(model_name):
                    matched_rule = rule
                    break

            if matched_rule is not None:
                device_class = str(matched_rule["deviceClass"])
                kind = matched_rule["kind"]
                polarity = str(matched_rule["polarity"])
                expected_terms = list(matched_rule["terminals"])
                if len(pin_nets) != len(expected_terms):
                    if device_class == "mos" and len(pin_nets) == 3:
                        pin_nets = [*pin_nets, pin_nets[2]]
                    elif device_class in {"res_poly_3t", "res_xhigh_po_3t"} and len(pin_nets) == 2:
                        device_class = "resistor"
                        expected_terms = ["P", "M"]
                    else:
                        raise ValueError(
                            f"Line {lineno} ({raw_name} : {model_name}): expected {len(expected_terms)} "
                            f"terminals {expected_terms}, got {len(pin_nets)}: {pin_nets}"
                        )
                terminals = dict(zip(expected_terms, pin_nets))
            else:
                prefix = raw_name[0].upper()
                if prefix == "M":
                    is_pmos = "p" in model_name.lower()
                    device_class = "mos"
                    kind = "pmos" if is_pmos else "nmos"
                    polarity = kind
                    if len(pin_nets) == 3:
                        pin_nets = [*pin_nets, pin_nets[2]]
                    if len(pin_nets) != 4:
                        raise ValueError(f"Line {lineno} ({raw_name}): MOS needs 4 terminals, got {pin_nets}")
                    terminals = dict(zip(["D", "G", "S", "B"], pin_nets))
                elif prefix == "R":
                    device_class = "res_poly_3t" if len(pin_nets) == 3 else "resistor"
                    kind = None
                    polarity = "passive"
                    expected_r = ["P", "M", "B"] if len(pin_nets) == 3 else ["P", "M"]
                    if len(pin_nets) not in (2, 3):
                        raise ValueError(f"Line {lineno} ({raw_name}): resistor needs 2 or 3 pins")
                    terminals = dict(zip(expected_r, pin_nets))
                elif prefix == "C":
                    device_class = "capacitor"
                    kind = None
                    polarity = "passive"
                    terminals = {"P": pin_nets[0], "M": pin_nets[1]}
                elif prefix == "D":
                    device_class = "diode"
                    kind = None
                    polarity = "passive"
                    terminals = {"P": pin_nets[0], "M": pin_nets[1]}
                elif prefix == "I":
                    device_class = "isource"
                    kind = None
                    polarity = "source"
                    terminals = {"P": pin_nets[0], "M": pin_nets[1]}
                elif prefix == "V":
                    device_class = "vsource"
                    kind = None
                    polarity = "source"
                    terminals = {"P": pin_nets[0], "M": pin_nets[1]}
                else:
                    raise ValueError(
                        f"Line {lineno}: unrecognized subcircuit/model {model_name!r} on instance {raw_name!r}"
                    )

            ref = _clean_ref_name(raw_name, device_class, matched_rule is not None, existing_refs)
            existing_refs.add(ref)
            norm_params, exprs = _normalize_instance_params(raw_params, global_params, default_unit)
            instances.append(
                InstanceIR(
                    ref=ref,
                    raw_name=raw_name,
                    device_class=device_class,
                    kind=kind,
                    polarity=polarity,
                    source_model=model_name,
                    terminals=terminals,
                    params=norm_params,
                    param_expressions=exprs,
                    section_hint=sec,
                )
            )

        if not instances:
            continue

        nets: list[str] = []
        seen_nets: set[str] = set()
        for p in declared_ports:
            if p not in seen_nets:
                seen_nets.add(p)
                nets.append(p)
        for inst in instances:
            for net in inst.terminals.values():
                if net not in seen_nets:
                    seen_nets.add(net)
                    nets.append(net)

        supply_nets = [n for n in nets if any(rx.match(n) for rx in supply_res)]
        ground_nets = [n for n in nets if any(rx.match(n) for rx in ground_res)]

        eff_ports = list(declared_ports)
        if not eff_ports:
            inferred: list[str] = []
            for n in nets:
                up = n.upper()
                if n in supply_nets or n in ground_nets or any(
                    up.startswith(pfx)
                    for pfx in ("VIN", "IN", "VIP", "VIM", "VOUT", "OUT", "IBIAS", "VBIAS_EXT", "VREF", "EN")
                ):
                    inferred.append(n)
            eff_ports = inferred or (list(supply_nets) + list(ground_nets))

        ports = [
            PortIR(
                name=pname,
                net=pname,
                direction=pin_directions_global.get(
                    pname,
                    _infer_port_direction(pname, pname in supply_nets, pname in ground_nets),
                ),
            )
            for pname in eff_ports
            if pname in seen_nets
        ]

        final_cell_name = top_cell_override if (top_cell_override and len(raw_blocks) == 1) else b_name
        cells[final_cell_name] = NetlistCircuit(
            cell_name=final_cell_name,
            ports=ports,
            instances=instances,
            nets=nets,
            supply_nets=supply_nets,
            ground_nets=ground_nets,
        )

    if not cells:
        raise ValueError("Netlist contains no recognizable circuit instances")

    # Determine top cell (override, or the cell not instantiated by any other cell, or last block)
    referenced_cells = {
        inst.subckt_cell
        for c in cells.values()
        for inst in c.instances
        if inst.subckt_cell
    }
    unreferenced = [cname for cname in cells if cname not in referenced_cells]
    top_cell = (
        top_cell_override
        if (top_cell_override and top_cell_override in cells)
        else (unreferenced[-1] if unreferenced else list(cells.keys())[-1])
    )
    return NetlistDesign(cells=cells, top_cell=top_cell, global_params=global_params)


def parse_netlist_text(
    text: str,
    *,
    rules_path: Path | str = DEFAULT_NETLIST_RULES_PATH,
    cell_name_override: str | None = None,
) -> NetlistCircuit:
    """Parse a single leaf cell (or top cell of a netlist) into `NetlistCircuit`."""
    design = parse_netlist_design_text(
        text, rules_path=rules_path, top_cell_override=cell_name_override
    )
    circuit = design.cells[design.top_cell]
    if cell_name_override:
        circuit.cell_name = cell_name_override
    return circuit


def parse_netlist_file(
    path: Path | str,
    *,
    rules_path: Path | str = DEFAULT_NETLIST_RULES_PATH,
    cell_name_override: str | None = None,
) -> NetlistCircuit:
    p = Path(path)
    return parse_netlist_text(
        p.read_text(encoding="utf-8"),
        rules_path=rules_path,
        cell_name_override=cell_name_override,
    )


def parse_netlist_design_file(
    path: Path | str,
    *,
    rules_path: Path | str = DEFAULT_NETLIST_RULES_PATH,
    top_cell_override: str | None = None,
) -> NetlistDesign:
    p = Path(path)
    return parse_netlist_design_text(
        p.read_text(encoding="utf-8"),
        rules_path=rules_path,
        top_cell_override=top_cell_override,
    )
