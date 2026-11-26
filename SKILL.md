---
name: analog-schem-gen
description: PDK-agnostic high-readability analog circuit schematic generator and existing-draft refiner. Separates LLM coarse-to-fine circuit-semantic extraction (Core Anchors -> Functional Modules -> Meso Skeletons -> Fine IntentSpec) from deterministic vertical branch folding, bias-tree separation, pre-layout semantic auditing, constrained placement (Convex QP), Steiner trunk synthesis, grid-aligned orthogonal track routing + nudging, and LVS/readability DRC verification.
---

# Analog Schematic Generator (`analog-schem-gen`) · Coarse-to-Fine LLM × Algorithm SOP

When asked to generate a clean, human-readable analog schematic from a SPICE/CDL/Spectre netlist, refine a cramped/stretched draft schematic, or prepare an exact-coordinate manifest for Cadence Virtuoso, follow this strict **Coarse → Meso → Fine** division of labor. **Never hand-author raw `(x, y)` coordinates or wire bend arrays, and never group devices blindly by reference numbering (`M1..M10`).**

## 1. Division of Labor (Execution Granularity)

- **Algorithm Front-End (`analyze` / `inspect` / `validate-modules`)**:
  - Losslessly folds 40–160+ device flat netlists (MOS, BJT, R, C, diodes, sources) into ~10–30 top-to-bottom vertical current branches (`BR01..BRk`) without tearing `Super-MOS` or degree-2 series chains;
  - Separates global diode/port **Bias Distribution Trees** from **Signal-Only Inter-Branch Edges**, surfaces **Feedback Loops (SCCs)**, **Cross-Device-Type Quasi-Symmetric Pairs** (e.g., BGR cores, coupled BJT+MOS+R mirror amps), **Regular Arrays (`>= 3` cols)**, and ranked **Core Anchor Candidates** in `semantic_digest.md`;
  - Provides interactive read-only tracing (`inspect --around` / `inspect --module`) and millisecond pre-layout semantic auditing (`validate-modules`).
- **LLM Responsibilities (Strict Coarse → Meso → Fine)**:
  1. **Coarse (Macro Anchors & Modules)**: Anchor the 1–2 true core circuits first (`isCore: true`), then partition folded branches (`BRxx`) into cohesive functional modules with left-to-right `stage_order` and one-sentence topological `evidence` (ignoring global bias rails when drawing boundaries);
  2. **Meso (Per-Module Skeleton)**: Use `inspect --module <BRxx,...>` to focus on one complex module at a time (6–20 devices) and declare its `skeleton` (`mirror_pair`, `diff_stage`, `bias_bank`, `parallel_array`, `horizontal_bridge`);
  3. **Fine (Overrides & Visual Review)**: Let `compile --module-plan` auto-derive routine `orientations`, `tierOverrides`, and `columnOrderHints`; inspect `preview.svg` + `audit.md` and only tweak exceptions (`netPolicies`, `portSides`, or a few `overrides.orientations`).
- **Algorithm Back-End (`compile` / `plan-to-intent` / `audit`)**:
  - Deterministically expands `module_plan.v1` into `intent.v1`, solves Convex QP placement with exact integer symmetry and 2D stacking, synthesizes collinear `VDD`/`VSS` rails and orthogonal Steiner tracks with corridor nudging, and verifies LVS + Bridge exact geometry + Readability DRC.

## 2. Coarse-to-Fine Co-Execution Workflow (Recommended for Complex Netlists)

Run from the repository root (or invoke `analog-schem-gen` after `pip install -e .`):

### Step 1: Deterministic Branch Folding & Semantic Digest (Algorithm)
```bash
python -B schem_gen.py analyze <netlist.spice> --out <work_dir>/01_analyze --rules pdk/<pdk>_netlist_rules.json
```
Read `<work_dir>/01_analyze/semantic_digest.md` **first** (summary tier; on large circuits it lists only core-anchor branches in full and points to `semantic_digest_full.md` / `inspect` for the rest):
- **Section 2 (`Vertical Branch Map`)**: grasp the entire circuit as ~10–30 folded vertical branches (`BR01..BRk`) instead of 60+ flat SPICE lines;
- **Section 3 (`Bias Trees, Core Anchors, Quasi-Symmetric Pairs, Arrays & Feedback Loops`)**: mentally strip `[global]` bias trees first, then inspect ranked **Core Anchors** and **Signal-Only Inter-Branch Edges**;
- **Section 4–7**: check Pass 3 bottleneck cuts (for feedforward multi-stage op-amps), special device roles (`mos_cap`, `mos_active_res`, `bjt_diode`, `bjt_follower`), and targeted `ambiguitiesForLLM`.

### Step 2: Coarse Module Extraction + Local Tracing + Semantic Gate (LLM × Algorithm)
Copy `<work_dir>/01_analyze/module_plan.draft.json` to `<work_dir>/module_plan.json` and refine it **Anchor-First**:
1. **Lock Core Anchors First**: Designate the 1–2 primary core modules (`"isCore": true`) using the Analog Module Pattern Playbook in Section 3 below.
2. **Trace Ambiguous Boundaries On Demand** (do not guess multi-hop connections in raw SPICE):
   ```bash
   # k-hop local signal neighborhood (automatically hides VDD/VSS & global bias trees)
   python -B schem_gen.py inspect <netlist.spice> --around Q15 --hops 1
   ```
3. **Assign Whole Branches (`branches: ["BR05", ...]`) + Write Topological `evidence`**:
   - Keep every folded branch intact unless you have a strong circuit reason to split it;
   - Write a concise one-sentence topological `evidence` per module (citing shared signal nets, mirror symmetry, or feedback divider role — never ref number proximity).
4. **Run Pre-Layout Semantic Validation Gate** (mandatory before layout):
   ```bash
   python -B schem_gen.py validate-modules <netlist.spice> --plan <work_dir>/module_plan.json --out <work_dir>/02_validate
   ```
   Fix all `ERROR` findings (`MISSING_INSTANCES`, `DUPLICATE_INSTANCE`, `TORN_ATOMIC_BRANCH` = a HARD pure-series / Super-MOS segment split across modules, `TORN_DIFF_PAIR`) and review every `SPLIT_AT_SOFT_JOINT`, `ORPHAN_BRANCH_IN_MODULE` or `HIGH_INTER_MODULE_CUT` warning until module boundaries are crisp. The untouched `module_plan.draft.json` always passes this gate and compiles identically to `intent.draft.json`.

### Step 3: Meso Skeleton Refinement (Per-Module Focus Window)
For each complex core or bias module, open a clean **Meso Focus Window** showing only that module's devices, internal private nets, and external signal interfaces:
```bash
python -B schem_gen.py inspect <netlist.spice> --module BR05,BR06,BR07,BR08
```
Add a concise `"skeleton"` inside that module in `module_plan.json` (the draft may already carry a `"suggestedSkeleton"` for uncovered quasi-symmetric pairs — it is inactive until you rename it to `"skeleton"` after checking the focus window):
- `{"type": "mirror_pair", "leftBranches": [...], "rightBranches": [...], "centerBranches": [...]}` (supports both strict mirrors and quasi-symmetric cores like BGR with extra emitter resistor!);
- `{"type": "bias_bank", "referenceBranches": [...], "mirrorBranches": [...]}` (places diode reference column(s) left facing right);
- `{"type": "parallel_array", "branches": [...], "commonNet": "..."}` (preserves uniform column array order);
- `{"type": "horizontal_bridge", "instances": [...]}` (horizontal Miller/RC bridge).

Re-run `validate-modules` to confirm `symmetryPassed = PASS`.

### Step 4: Compile Geometry, SVG & Visual Critique Loop (Algorithm × LLM)
Compile directly from the validated `module_plan.json` (automatically expands into `intent.expanded.json` with derived orientations/tiers, runs QP placement, orthogonal routing, and full LVS/DRC audit):
```bash
python -B schem_gen.py compile <netlist.spice> --module-plan <work_dir>/module_plan.json --allow-wire-labels --pdk-map pdk/<pdk>_process_map.json --process <pdk> --out <work_dir>/03_compile
```
*(For small `<= 20` device circuits, you may alternatively edit `intent.draft.json` directly and run `compile --intent <work_dir>/intent.json`.)*
1. Check `<work_dir>/03_compile/audit.md`: ensure `overallPassed = PASS` (`0` overlaps, `0` body/pin collisions, `0` collinear foreign wires, `0` split internal nets).
2. Inspect `<work_dir>/03_compile/preview.svg` visually. If a few net policies (`perimeter_bus` / `label_stub`), port sides (`near_pin` / `right`), or single-device orientations can eliminate crossings, update `netPolicies`, `portSides`, or `overrides` in `module_plan.json` (or `intent.json`) and re-run Step 4.
3. Policy authority is fixed: your `netPolicies` / `overrides` (`llm`) > skeleton expansion > crossing optimizer > analyzer defaults; the optimizer never overrides your choices. If `audit.md` reports `LABEL_STUB_DEGRADED`, re-run with `--allow-wire-labels` to get real label stubs.
4. Reference numbers and net names are only tie-breakers inside the tool (Section 5 of the digest lists any diff pair whose left/right was decided by naming). Never justify a module boundary by numbering.

## 3. Analog Functional Module Pattern Playbook (Topology-Driven, Name-Agnostic)

Use these structural fingerprints when extracting core & auxiliary modules from anonymous netlists:

| Module Class | Topology Fingerprint (Ignore Ref Numbers & Net Names) | Typical `role` / `skeleton` |
| :--- | :--- | :--- |
| **Bandgap / PTAT Core** | Two quasi-symmetric active branches (`QSYM_*`, similarity `0.75–1.0`) sharing 2–3 local control nets (`VREF`/base/cascode), often with BJT area ratio `m>1` or asymmetric emitter degeneration (`R7` vs `R8+R9`) | `isCore: true`, `role: "input_diff"`, `skeleton: mirror_pair` |
| **Mirrored Error Amp / Diff Core** | Coupled 2-column mirror pair (`QSYM_COUPLED_*`), composite `diff_pair`, or `sandwich_mirror_core` (tolerating single-ended output tap `asymmetricTaps`) | `isCore: true`, `role: "gain_stage"` / `"input_diff"`, `skeleton: mirror_pair` |
| **Global Bias Root & Mirror Bank** | `isource` + diode-connected MOS/BJT (`D=G`, `C=B`, or `Super-MOS` stack) generating `scope=global` bias trees (`net2`, `net11`) + adjacent mirror legs | `role: "bias"`, `skeleton: bias_bank` |
| **Weighted Current / Cap Array** | `>= 3` identical-signature columns (`ARR_*`, e.g., IDAC `PMOS×2` / `NMOS×3`, CDAC caps, R-string ladder) sharing a summing or control bus | `isCore: true`, `role: "cascode_core"`, `skeleton: parallel_array` |
| **Feedback Divider & Output Stage** | High-drive output pass FET/BJT or complementary source follower (`M38/M39`) driving output/reference port + vertical resistor divider (`R10→R11→VSS`) participating in `LOOP*` | `role: "output_stage"` |
| **Compensation, Startup & Clamp** | Floating Miller/RC bridges (`bridge_passive`), grounded decoupling caps (`shunt_passive`), rail-collector BJT clamps (`bjt_follower` / `bjt_diode`), or low-current startup branches tied into a single core node | `role: "compensation"` / `"bias"` / `"output_stage"` |

## 4. Structure-Preserving Draft Cleanup & Interactive Browser Micro-Editing

- Clean up an existing cramped or over-stretched `virtuoso-bridge-exact-schematic-v1` manifest while keeping its row/column structure:
  ```bash
  python -B schem_gen.py refine <draft_manifest.json> --pdk-map pdk/<pdk>_process_map.json --process <pdk> --out <work_dir>/refined
  ```
- Export a standalone zero-dependency browser drag/flip editor (`editor.html`), adjust device positions/orientations visually, download `edited_manifest.json`, and run `refine` to re-route obstacle-free Steiner wires:
  ```bash
  python -B schem_gen.py export-editor <netlist.spice> --intent <work_dir>/intent.json --out <work_dir>/editor.html
  ```

## 5. Multi-Sheet Hierarchical Compilation for Large Netlists (`>= 40` Instances)

Never cram 50–200+ transistors onto a single flat sheet when clean page boundaries exist. Use atomic-cluster-protected hierarchical outlining and bottom-up compilation:
```bash
# Step A: Propose & review/edit coordinate-free page partition intent
python -B schem_gen.py partition <large_netlist.spice> --out <work_dir>/01_partition

# Step B: Compile all child sheets + TOP block diagram, verify Flatten-Back LVS, and emit index.html Schematic Book
python -B schem_gen.py hierarchy <large_netlist.spice> --hierarchy-intent <work_dir>/01_partition/hierarchy_intent.draft.json --out <work_dir>/02_hierarchy
```
Always check that `flattenLvsPassed = PASS` and every sheet in `hierarchy_summary.json` passes readability DRC (`0` violations).

## 6. Target PDK Plug-in, Profiler & Live Virtuoso Gate

- Switch target PDKs by supplying `--pdk-map pdk/<name>_process_map.json --process <name> --rules pdk/<name>_netlist_rules.json` (`sky130` is bundled as an open-source Apache-2.0 reference profile).
- Extract a new PDK profile read-only from Xschem `.sym` files or an authorized live Virtuoso session via `python -B schem_gen.py extract-pdk ...`.
- Before importing into a live Cadence Virtuoso OA library, run `python -B schem_gen.py probe-pdk --pdk-map <map.json> --process <name> --live` in an authorized session to verify live symbol `pinOffsets` and CDF parameter names.
