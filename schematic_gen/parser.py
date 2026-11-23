"""Netlist parser supporting SKY130 SPICE/CDL (`X... sky130_fd_pr__*`), standard SPICE, and Spectre leaf syntax."""

from __future__ import annotations

import json
from pathlib import Path
import re
from typing import Any

from schematic_gen.models import InstanceIR, NetlistCircuit, PortIR
from schematic_gen.pdk import DEFAULT_NETLIST_RULES_PATH


_PARAM_RE = re.compile(r"([A-Za-z_][A-Za-z0-9_]*)\s*=\s*([^\s]+)")
_SPECTRE_INST_RE = re.compile(
    r"^([A-Za-z_][A-Za-z0-9_]*)\s*\(([^)]+)\)\s*([A-Za-z_][A-Za-z0-9_.]*)(.*)$"
)


def _join_continuations(lines: list[str]) -> list[tuple[int, str]]:
    """Join SPICE `+` continuation lines and Spectre `\\` continuations."""
    logical: list[tuple[int, str]] = []
    pininfo_lines: list[tuple[int, str]] = []

    for lineno, raw in enumerate(lines, start=1):
        stripped = raw.strip()
        if not stripped:
            continue
        if stripped.upper().startswith("*.PININFO"):
            pininfo_lines.append((lineno, stripped))
            continue
        if stripped.startswith("*") or stripped.startswith("//") or stripped.startswith(";"):
            continue
        # Strip trailing inline comments
        for marker in (" //", " $ ", " ;"):
            if marker in stripped:
                stripped = stripped.split(marker, 1)[0].strip()
        if not stripped:
            continue
        if stripped.startswith("+") and logical:
            prev_no, prev_txt = logical[-1]
            logical[-1] = (prev_no, prev_txt + " " + stripped[1:].strip())
            continue
        if logical and logical[-1][1].endswith("\\"):
            prev_no, prev_txt = logical[-1]
            logical[-1] = (prev_no, prev_txt[:-1].rstrip() + " " + stripped)
            continue
        logical.append((lineno, stripped))

    return pininfo_lines + logical


def _parse_params(tokens: list[str]) -> tuple[list[str], dict[str, str]]:
    """Split positional tokens from `key=value` parameter tokens."""
    positional: list[str] = []
    params: dict[str, str] = {}
    joined = " ".join(tokens)
    # Handle `key = value` with spaces around `=` first
    normalized = re.sub(r"\s*=\s*", "=", joined)
    for tok in normalized.split():
        if "=" in tok and not tok.startswith("="):
            k, v = tok.split("=", 1)
            v = v.strip("'\"{}")
            params[k.lower()] = v
        else:
            positional.append(tok)
    return positional, params


def _normalize_mos_params(params: dict[str, str], default_unit: str = "um") -> dict[str, str]:
    """Normalize SKY130 / SPICE device parameters cleanly without losing precision."""
    out: dict[str, str] = {}
    w = params.get("w")
    l = params.get("l")
    nf = params.get("nf", "1")
    m = params.get("m") or params.get("mult") or params.get("mf") or "1"

    def _fmt_dim(val: str | None) -> str | None:
        if val is None or val == "":
            return None
        # Convert scientific e-6/e-9 or bare SKY130 micron numbers into clean `u` / `n` strings
        try:
            fval = float(val)
            if fval <= 0:
                return val
            if fval < 1e-3:
                # SI meters (e.g. 1.5e-7 -> 0.15u)
                um = fval * 1e6
                return f"{um:g}u"
            if default_unit == "um":
                return f"{fval:g}u"
            return val
        except ValueError:
            return val

    if w is not None:
        out["w"] = _fmt_dim(w) or w
    if l is not None:
        out["l"] = _fmt_dim(l) or l
    if nf is not None:
        out["nf"] = str(nf)
    if "mf" in params:
        out["mf"] = str(params["mf"])
    out["m"] = str(m)

    for extra_key in ("r", "c", "dc", "idc", "vdc", "area", "perim"):
        if extra_key in params:
            out[extra_key] = params[extra_key]
    return out


def _infer_port_direction(port_name: str, is_supply: bool, is_ground: bool) -> str:
    if is_supply or is_ground:
        return "inout"
    up = port_name.upper()
    if any(up.startswith(p) for p in ("VIN", "IN", "VIP", "VIM", "INN", "INP", "IBIAS", "VBIAS", "EN", "CLK", "RST", "VREF")):
        return "input"
    if any(up.startswith(p) for p in ("VOUT", "OUT", "VOP", "VOM", "IOUT", "Q")):
        return "output"
    return "inout"


def _clean_ref_name(raw_name: str, device_class: str, existing_refs: set[str]) -> str:
    """Strip redundant leading `X` on SKY130 primitive subcircuits (`XM1` -> `M1`, `XRZ` -> `RZ`)."""
    candidate = raw_name
    if len(raw_name) >= 3 and raw_name[0].upper() == "X":
        second = raw_name[1].upper()
        if (device_class == "mos" and second == "M") or (
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


def parse_netlist_text(
    text: str,
    *,
    rules_path: Path | str = DEFAULT_NETLIST_RULES_PATH,
    cell_name_override: str | None = None,
) -> NetlistCircuit:
    """Parse a SKY130 SPICE, CDL, or Spectre leaf cell netlist into `NetlistCircuit`."""
    rules_data: dict[str, Any] = json.loads(
        Path(rules_path).read_text(encoding="utf-8")
    )
    default_unit = str(rules_data.get("defaultDimensionUnit", "um"))
    supply_res = [re.compile(p, re.IGNORECASE) for p in rules_data.get("supplyNetPatterns", [])]
    ground_res = [re.compile(p, re.IGNORECASE) for p in rules_data.get("groundNetPatterns", [])]
    model_rules = [
        (re.compile(r["pattern"], re.IGNORECASE), r)
        for r in rules_data.get("modelRules", [])
    ]

    logical_lines = _join_continuations(text.splitlines())

    cell_name = cell_name_override or "top_schematic"
    declared_ports: list[str] = []
    pin_directions: dict[str, str] = {}
    instances: list[InstanceIR] = []
    existing_refs: set[str] = set()
    in_target_subckt = False
    saw_subckt = False

    for lineno, line in logical_lines:
        upper = line.upper()
        if upper.startswith("*.PININFO"):
            for item in line.split()[1:]:
                if ":" in item:
                    pname, pdir = item.split(":", 1)
                    dir_map = {"I": "input", "O": "output", "B": "inout", "IO": "inout"}
                    pin_directions[pname] = dir_map.get(pdir.upper(), "inout")
            continue

        if upper.startswith(".SUBCKT ") or upper.startswith("SUBCKT "):
            if saw_subckt and in_target_subckt:
                break
            tokens, _ = _parse_params(line.split()[1:])
            if not tokens:
                raise ValueError(f"Line {lineno}: malformed .subckt header")
            if not cell_name_override:
                cell_name = tokens[0]
            declared_ports = list(tokens[1:])
            in_target_subckt = True
            saw_subckt = True
            continue

        if upper.startswith(".ENDS") or upper == "ENDS" or upper.startswith("ENDS "):
            if in_target_subckt:
                in_target_subckt = False
                break
            continue

        if line.startswith(".") or upper.startswith(("SIMULATOR ", "GLOBAL ", "PARAMETERS ", "INCLUDE ", "MODEL ")):
            continue

        # Check Spectre style: `M1 (d g s b) nfet_01v8 w=10u l=0.15u`
        spectre_match = _SPECTRE_INST_RE.match(line)
        if spectre_match:
            raw_name = spectre_match.group(1)
            pin_nets = spectre_match.group(2).split()
            model_name = spectre_match.group(3)
            _, raw_params = _parse_params(spectre_match.group(4).split())
        else:
            tokens, raw_params = _parse_params(line.split())
            if len(tokens) < 3:
                continue
            raw_name = tokens[0]
            prefix = raw_name[0].upper()

            if prefix in {"R", "C", "I", "V"} and len(tokens) == 4 and not any(
                rx.match(tokens[-1]) for rx, _ in model_rules
            ):
                # Standard SPICE 2-terminal element with value: `R1 n1 n2 10k` or `I0 VDD vbias 10u`
                pin_nets = tokens[1:3]
                val_token = tokens[3]
                if val_token.upper() == "DC" and "dc" in raw_params:
                    model_name = prefix
                else:
                    model_name = prefix
                    val_key = {"R": "r", "C": "c", "I": "dc", "V": "dc"}[prefix]
                    raw_params.setdefault(val_key, val_token)
            elif prefix in {"I", "V"} and len(tokens) >= 5 and tokens[3].upper() == "DC":
                pin_nets = tokens[1:3]
                model_name = prefix
                raw_params.setdefault("dc", tokens[4])
            else:
                model_name = tokens[-1]
                pin_nets = tokens[1:-1]

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
                # Allow 3-terminal MOS fallback (bulk defaults to S) only if 3 pins supplied
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
                device_class = "resistor"
                kind = None
                polarity = "passive"
                if len(pin_nets) == 3:
                    device_class = "res_poly_3t"
                    terminals = dict(zip(["P", "M", "B"], pin_nets))
                elif len(pin_nets) == 2:
                    terminals = dict(zip(["P", "M"], pin_nets))
                else:
                    raise ValueError(f"Line {lineno} ({raw_name}): resistor needs 2 or 3 pins, got {pin_nets}")
            elif prefix == "C":
                device_class = "capacitor"
                kind = None
                polarity = "passive"
                if len(pin_nets) < 2:
                    raise ValueError(f"Line {lineno} ({raw_name}): capacitor needs 2 pins, got {pin_nets}")
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

        ref = _clean_ref_name(raw_name, device_class, existing_refs)
        existing_refs.add(ref)
        norm_params = _normalize_mos_params(raw_params, default_unit=default_unit)
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
            )
        )

    if not instances:
        raise ValueError("Netlist contains no recognizable circuit instances")

    # Collect unique nets in deterministic appearance order
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

    # If flat netlist had no `.subckt` header, infer ports from supply/ground + single-ended / named IO nets
    if not declared_ports:
        inferred: list[str] = []
        for n in nets:
            up = n.upper()
            if n in supply_nets or n in ground_nets or any(
                up.startswith(prefix)
                for prefix in ("VIN", "IN", "VIP", "VIM", "VOUT", "OUT", "IBIAS", "VBIAS_EXT", "VREF", "EN")
            ):
                inferred.append(n)
        declared_ports = inferred or list(supply_nets) + list(ground_nets)

    ports = [
        PortIR(
            name=pname,
            net=pname,
            direction=pin_directions.get(
                pname,
                _infer_port_direction(pname, pname in supply_nets, pname in ground_nets),
            ),
        )
        for pname in declared_ports
        if pname in seen_nets
    ]

    return NetlistCircuit(
        cell_name=cell_name,
        ports=ports,
        instances=instances,
        nets=nets,
        supply_nets=supply_nets,
        ground_nets=ground_nets,
    )


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
