---
name: analog-schem-gen
description: PDK-agnostic high-readability analog circuit schematic generator and existing-draft refiner. Separates LLM circuit-semantic judgment (IntentSpec: stage order, motif orientations, net routing policies) from deterministic constrained placement (Convex QP), Steiner trunk synthesis, grid-aligned orthogonal track routing + nudging, and LVS/readability DRC verification.
---

# Analog Schematic Generator (`analog-schem-gen`) · LLM × Algorithm SOP

When asked to generate a clean, human-readable analog schematic from a SPICE/CDL/Spectre netlist, refine a cramped/stretched draft schematic, or prepare an exact-coordinate manifest for Cadence Virtuoso, follow this strict division of labor. **Never hand-author raw `(x, y)` coordinates or wire bend arrays.**

## 1. Division of Labor (Execution Granularity)

- **LLM Responsibilities**: Inspect `analysis.json` (`candidateMotifs` + `ambiguitiesForLLM`) and refine `intent.json` using discrete semantic enums:
  1. Left-to-right functional reading order (`groups[].stage_order`);
  2. Symbol semantic orientations (`gate_left`, `gate_right`, `vertical`, `horizontal_l2r`);
  3. Net visual decluttering policy (`supply_local`, `full_wire`, `perimeter_bus`);
  4. Top-level port placement (`near_pin`, `left`, `right`, `local_repeat`);
  5. Multimodal visual review of `preview.svg` + quantitative review of `audit.md`.
- **Algorithm Responsibilities**: Run `schem_gen.py` to compute symbol + parameter text bounding envelopes, solve Convex QP expansion/compaction with exact integer differential symmetry, synthesize Steiner trunks, route obstacle-free orthogonal tracks with parallel nudging, auto-expand congested corridors, and verify LVS + readability DRC.

## 2. Four-Step Co-Execution Workflow (Netlist -> Clean Schematic)

Run from the repository root (or invoke `analog-schem-gen` after `pip install -e .`):

### Step 1: Deterministic Topology Analysis (Algorithm)
```bash
python -B schem_gen.py analyze <netlist.spice> --out <work_dir>/01_analyze --rules pdk/<pdk>_netlist_rules.json
```
Read `<work_dir>/01_analyze/analysis.json` and `<work_dir>/01_analyze/intent.draft.json`.

### Step 2: Refine Semantic Intent `intent.json` (LLM Judgment)
Copy `intent.draft.json` to `<work_dir>/intent.json` and verify four analog schematic conventions:
1. **Stage Reading Order (`groups[].stage_order`)**: `Bias -> Input Diff Stage -> Cascode / Mid Stage -> Compensation / Feedback -> Output Stage & Load`.
2. **Symmetry & Orientations (`motifs` & `orientations`)**:
   - Differential pair: left transistor `gate_left`, right transistor `gate_right` (`back_to_back`), with tail current source declared in `motifs`;
   - Active mirror load above diff pair: left transistor `gate_right`, right transistor `gate_left` (face-to-face) for shortest common-gate trunk;
   - Inter-stage series RC Miller compensation (`RZ + CC`): set `horizontal_l2r` on mid-tier `tierOverrides = 3`.
3. **Net Decluttering (`netPolicies`)**:
   - Power/ground nets: `supply_local`;
   - Cross-chip bias nets spanning >= 3 groups: `perimeter_bus` to route along the outer bus channel instead of cutting across differential signal paths.
4. **Port Placement (`portSides`)**:
   - Set differential input ports (`VINP`, `VINN`) to `near_pin` when a bias column precedes the input pair;
   - Set multi-branch bias ports to `local_repeat`.

### Step 3: Compile Geometry, SVG & Audit Report (Algorithm)
```bash
python -B schem_gen.py compile <netlist.spice> --intent <work_dir>/intent.json --pdk-map pdk/<pdk>_process_map.json --process <pdk> --out <work_dir>/02_compile
```

### Step 4: Visual & DRC Critique Loop (LLM Review)
1. Check `<work_dir>/02_compile/audit.md`: ensure `overallPassed = PASS` (0 symbol/label overlaps, 0 body/pin collisions, 0 collinear foreign wires, 0 split internal nets) and inspect `wireCrossingCount` + `llmRefinementSuggestions`.
2. Inspect `<work_dir>/02_compile/preview.svg` visually. If an orientation flip or `perimeter_bus` toggle can eliminate remaining crossings, update `intent.json` and re-run Step 3.

## 3. Structure-Preserving Draft Cleanup (`refine`)

To clean up an existing cramped or over-stretched `virtuoso-bridge-exact-schematic-v1` manifest while keeping its row/column structure:
```bash
python -B schem_gen.py refine <draft_manifest.json> --pdk-map pdk/<pdk>_process_map.json --process <pdk> --out <work_dir>/refined
```

## 4. Target PDK Plug-in & Live Virtuoso Gate

- Switch target PDKs by supplying `--pdk-map pdk/<name>_process_map.json --process <name> --rules pdk/<name>_netlist_rules.json` (`sky130` is bundled as an open-source Apache-2.0 reference profile).
- Before importing into a live Cadence Virtuoso OA library, run `python -B schem_gen.py probe-pdk --pdk-map <map.json> --process <name> --live` in an authorized session to verify live symbol `pinOffsets` and CDF parameter names.
