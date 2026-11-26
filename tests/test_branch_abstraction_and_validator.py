"""Known-solution & counterexample tests for universal branch folding, bias-tree separation,
pre-layout semantic module validation, Meso focus inspection, and Coarse-to-Fine `--module-plan` compilation.
"""

from __future__ import annotations

import json
from pathlib import Path
import tempfile
import unittest

from schem_gen import main as cli_main
from schematic_gen.analyzer import analyze_circuit
from schematic_gen.parser import parse_netlist_text
from schematic_gen.semantic_validator import (
    expand_module_plan_to_intent,
    format_inspect_markdown,
    inspect_netlist_neighborhood,
    validate_module_plan,
)


SYNTHETIC_SKY130_BGR_SPICE = """
.subckt sky130_bgr_ref VDD VSS NODE_B NODE_A NODE_C NODE_E VREF
C1 NODE_C NODE_E 1p
C2 NODE_E VSS 1p
I1 VDD n_bn DC 10u
XM1 n_p1 n_bp VDD VDD sky130_fd_pr__pfet_01v8 l=0.5 w=2 nf=1 m=1
XM2 n_bp n_bp n_p1 VDD sky130_fd_pr__pfet_01v8 l=0.5 w=2 nf=1 m=1
XM10 n_bn n_bn n_n1 VSS sky130_fd_pr__nfet_01v8 l=0.5 w=2 nf=1 m=1
XM11 n_n1 n_bn VSS VSS sky130_fd_pr__nfet_01v8 l=0.5 w=2 nf=1 m=1
XM14 n_bp n_bn n_n2 VSS sky130_fd_pr__nfet_01v8 l=0.5 w=2 nf=1 m=1
XM15 n_n2 n_bn VSS VSS sky130_fd_pr__nfet_01v8 l=0.5 w=2 nf=1 m=1
XM5 n_s1 n_bp VDD VDD sky130_fd_pr__pfet_01v8 l=0.5 w=2 nf=1 m=1
XQ1 n_d1 n_d1 n_s1 sky130_fd_pr__pnp_05v5_W0p68L0p68 m=1
XQ2 n_d2 n_d2 n_d1 sky130_fd_pr__pnp_05v5_W0p68L0p68 m=1
R1 n_d2 VSS 10k
XM6 n_m1 n_bp VDD VDD sky130_fd_pr__pfet_01v8 l=0.5 w=2 nf=1 m=1
XM18 n_m1 NODE_E VSS VSS sky130_fd_pr__nfet_01v8 l=0.5 w=2 nf=1 m=1
R2 NODE_C n_c2 10k
XM20 n_c2 n_m1 n_c3 VSS sky130_fd_pr__nfet_01v8 l=0.5 w=2 nf=1 m=1
XM19 n_c3 n_bn VSS VSS sky130_fd_pr__nfet_01v8 l=0.5 w=2 nf=1 m=1
XQ3 VDD NODE_C n_e1 VSS sky130_fd_pr__npn_05v5_W1p00L1p00 m=1
R3 n_e1 n_e2 5k
XM21 n_e2 n_bn n_e3 VSS sky130_fd_pr__nfet_01v8 l=0.5 w=2 nf=1 m=1
XM22 n_e3 n_bn VSS VSS sky130_fd_pr__nfet_01v8 l=0.5 w=2 nf=1 m=1
XM25 n_ea1 n_bp VDD VDD sky130_fd_pr__pfet_01v8 l=0.5 w=2 nf=1 m=1
XQ4 n_ea2 n_e1 n_ea1 sky130_fd_pr__pnp_05v5_W0p68L0p68 m=1
R4 n_ea2 VSS 5k
XQ5 VDD NODE_B n_f1 VSS sky130_fd_pr__npn_05v5_W1p00L1p00 m=1
R5 n_f1 n_f2 5k
XM23 n_f2 n_bn n_f3 VSS sky130_fd_pr__nfet_01v8 l=0.5 w=2 nf=1 m=1
XM24 n_f3 n_bn VSS VSS sky130_fd_pr__nfet_01v8 l=0.5 w=2 nf=1 m=1
XM26 NODE_A n_bp VDD VDD sky130_fd_pr__pfet_01v8 l=0.5 w=2 nf=1 m=1
XQ6 n_fa2 n_f1 NODE_A sky130_fd_pr__pnp_05v5_W0p68L0p68 m=1
R6 n_fa2 VSS 5k
XM34 n_cas n_bp VDD VDD sky130_fd_pr__pfet_01v8 l=0.5 w=2 nf=1 m=1
XM31 n_cas n_cas NODE_E VSS sky130_fd_pr__nfet_01v8 l=0.5 w=2 nf=1 m=1
XM27 NODE_E n_bn VSS VSS sky130_fd_pr__nfet_01v8 l=0.5 w=2 nf=1 m=1
XQ9 NODE_C NODE_A VDD sky130_fd_pr__pnp_05v5_W0p68L0p68 m=1
XM32 NODE_C n_cas n_b1 VSS sky130_fd_pr__nfet_01v8 l=0.5 w=2 nf=1 m=1
XQ15 n_b1 VREF n_b2 VSS sky130_fd_pr__npn_05v5_W1p00L1p00 m=8
R7 n_b2 n_tail 2k
XQ10 NODE_B NODE_A VDD sky130_fd_pr__pnp_05v5_W0p68L0p68 m=1
XM33 NODE_B n_cas n_b3 VSS sky130_fd_pr__nfet_01v8 l=0.5 w=2 nf=1 m=1
XQ8 n_b3 VREF n_tail VSS sky130_fd_pr__npn_05v5_W1p00L1p00 m=1
R8 n_tail n_rbot 4k
R9 n_rbot VSS 2k
XQ11 n_ea1 n_s1 VREF VSS sky130_fd_pr__npn_05v5_W1p00L1p00 m=1
XQ12 NODE_E n_ea1 VDD sky130_fd_pr__pnp_05v5_W0p68L0p68 m=1
R10 NODE_E VREF 10k
R11 VREF VSS 10k
.ends sky130_bgr_ref
"""


SYNTHETIC_SKY130_IDAC_ARRAY_SPICE = """
.subckt sky130_idac_mini VDD VSS EN N1 N2 N3 VB1 VB2 VB3 VB4 OUT
I1 VDD nb_n DC 10u
XM2 nb_n nb_n VSS VSS sky130_fd_pr__nfet_01v8 w=2 l=0.5 nf=1 m=1
XM1 nb_n EN VSS VSS sky130_fd_pr__nfet_01v8 w=1 l=0.5 nf=1 m=1
XM8 nb_p nb_p VDD VDD sky130_fd_pr__pfet_01v8 w=4 l=0.5 nf=1 m=1
XM3 nb_p nb_n VSS VSS sky130_fd_pr__nfet_01v8 w=2 l=0.5 nf=1 m=1
XM10 n_sum N1 n_s1 VSS sky130_fd_pr__nfet_01v8 w=2 l=0.5 nf=1 m=1
XM11 n_s1 nb_n VSS VSS sky130_fd_pr__nfet_01v8 w=2 l=0.5 nf=1 m=1
XM12 n_sum N2 n_s2 VSS sky130_fd_pr__nfet_01v8 w=2 l=0.5 nf=1 m=1
XM13 n_s2 nb_n VSS VSS sky130_fd_pr__nfet_01v8 w=2 l=0.5 nf=1 m=1
XM14 n_sum N3 n_s3 VSS sky130_fd_pr__nfet_01v8 w=2 l=0.5 nf=1 m=1
XM15 n_s3 nb_n VSS VSS sky130_fd_pr__nfet_01v8 w=2 l=0.5 nf=1 m=1
XM20 n_p1 n_sum VDD VDD sky130_fd_pr__pfet_01v8 w=4 l=0.5 nf=1 m=1
XM21 n_sum VB1 n_p1 VDD sky130_fd_pr__pfet_01v8 w=4 l=0.5 nf=1 m=1
XM22 n_p2 n_sum VDD VDD sky130_fd_pr__pfet_01v8 w=4 l=0.5 nf=1 m=1
XM23 n_sum VB2 n_p2 VDD sky130_fd_pr__pfet_01v8 w=4 l=0.5 nf=1 m=1
XM24 n_p3 n_sum VDD VDD sky130_fd_pr__pfet_01v8 w=4 l=0.5 nf=1 m=1
XM25 n_sum VB3 n_p3 VDD sky130_fd_pr__pfet_01v8 w=4 l=0.5 nf=1 m=1
XM26 n_p4 n_sum VDD VDD sky130_fd_pr__pfet_01v8 w=4 l=0.5 nf=1 m=1
XM27 OUT VB4 n_p4 VDD sky130_fd_pr__pfet_01v8 w=4 l=0.5 nf=1 m=1
.ends sky130_idac_mini
"""


class TestBranchAbstractionAndSemanticValidator(unittest.TestCase):
    def test_known_solution_mixed_bjt_mos_bandgap_branch_folding_bias_separation_and_anchors(self) -> None:
        """Universal branch abstraction must cleanly fold a mixed BJT+MOS+R+C Bandgap Reference,
        separate global bias rails (`n_bn`, `n_bp`), preserve atomic series stacks (incl. BJT diodes
        and resistive feedback divider without mid-divider clamp hijacking), and detect both the
        coupled mirrored error amp AND the asymmetric-in-detail Bandgap core + feedback loop.
        """
        circuit = parse_netlist_text(SYNTHETIC_SKY130_BGR_SPICE)
        analysis = analyze_circuit(circuit)
        ba = analysis.branch_abstraction

        # 1. Global bias trees separated from feedback reference
        self.assertEqual(set(ba["globalBiasNets"]), {"n_bn", "n_bp"})
        bt_by_net = {bt["net"]: bt for bt in ba["biasTrees"]}
        self.assertEqual(bt_by_net["n_bn"]["scope"], "global")
        self.assertEqual(bt_by_net["n_bp"]["scope"], "global")
        self.assertEqual(bt_by_net["VREF"]["scope"], "feedback_ref")

        # 2. Exact top-to-bottom vertical branch folding (zero torn series chains, zero mid-divider hijack)
        branch_by_id = {b["id"]: b for b in ba["branches"]}
        bof = ba["branchOfRef"]

        self.assertEqual(branch_by_id[bof["Q1"]]["instances"], ["M5", "Q1", "Q2", "R1"])
        self.assertEqual(branch_by_id[bof["Q9"]]["instances"], ["Q9", "M32", "Q15", "R7"])
        self.assertEqual(branch_by_id[bof["Q10"]]["instances"], ["Q10", "M33", "Q8", "R8", "R9"])
        self.assertEqual(branch_by_id[bof["R10"]]["instances"], ["Q12", "R10", "R11"])
        # Clamp Q11 taps VREF midpoint and must remain separate from Q12->R10->R11
        self.assertEqual(branch_by_id[bof["Q11"]]["instances"], ["Q11"])
        self.assertEqual(branch_by_id[bof["C1"]]["kind"], "bridge_passive")
        self.assertEqual(branch_by_id[bof["C2"]]["kind"], "shunt_passive")
        # Sizing clue `m=8` surfaced on BGR PTAT branch!
        self.assertIn("Q15:m=8", branch_by_id[bof["Q9"]]["sizeHighlights"])

        # 3. Cross-device-type coupled mirror error amp + quasi-symmetric BGR core + feedback loop
        qpairs = ba["quasiSymmetricPairs"]
        coupled = [q for q in qpairs if q["kind"] == "coupled_mirror_pair"]
        self.assertEqual(len(coupled), 1)
        self.assertEqual(
            set(coupled[0]["leftInstances"] + coupled[0]["rightInstances"]),
            {"Q3", "R3", "M21", "M22", "M25", "Q4", "R4", "Q5", "R5", "M23", "M24", "M26", "Q6", "R6"},
        )
        bgr_qsym = [
            q for q in qpairs
            if q["kind"] == "quasi_symmetric_core"
            and {q["leftBranches"][0], q["rightBranches"][0]} == {bof["Q9"], bof["Q10"]}
        ]
        self.assertEqual(len(bgr_qsym), 1)
        self.assertAlmostEqual(bgr_qsym[0]["similarity"], 8.0 / 9.0, places=2)
        self.assertGreaterEqual(len(ba["feedbackLoops"]), 1)

        # 4. Dual-view semantic_digest.md contains Coarse-to-Fine sections
        md = analysis.semantic_digest_md
        self.assertIn("Vertical Branch Map", md)
        self.assertIn("Core Anchors", md)
        self.assertIn("Feedback Loops", md)
        self.assertIn("Q15:m=8", md)

    def test_known_solution_repeated_branch_array_and_no_false_loop(self) -> None:
        """Regular repeated arrays (>= 3 cols) must be detected cleanly without emitting noisy
        redundant pairwise quasi-symmetric pairs or a false feedback loop on a summing node."""
        circuit = parse_netlist_text(SYNTHETIC_SKY130_IDAC_ARRAY_SPICE)
        analysis = analyze_circuit(circuit)
        ba = analysis.branch_abstraction

        arrs = ba["repeatedArrays"]
        arr_sigs = {(a["signature"], a["columnCount"]) for a in arrs}
        self.assertIn(("PMOS×2", 4), arr_sigs)
        self.assertIn(("NMOS×2", 3), arr_sigs)
        # No redundant pairwise quasi-symmetric pairs emitted for array members
        arr_brs = {bid for a in arrs for bid in a["branches"]}
        for qp in ba["quasiSymmetricPairs"]:
            self.assertFalse(set(qp["leftBranches"] + qp["rightBranches"]) <= arr_brs)
        # Single summing node `n_sum` is NOT a multi-net feedback loop
        self.assertEqual(ba["feedbackLoops"], [])

    def test_counterexamples_module_validator_catches_coverage_torn_orphan_and_symmetry(self) -> None:
        """Pre-layout `validate_module_plan` must catch missing/duplicate instances, torn atomic
        series branches, misplaced orphan branches with exact target hints, and mismatched mirror pairs."""
        circuit = parse_netlist_text(SYNTHETIC_SKY130_BGR_SPICE)
        analysis = analyze_circuit(circuit)
        ba = analysis.branch_abstraction
        bof = ba["branchOfRef"]

        # Counterexample 1: Missing device (omit C2), duplicate device (M5 in 2 mods),
        # torn atomic chain (Q1 separated from Q2/R1), misplaced orphan (R2/M20/M19 placed into output_mod
        # while only connecting via C/net12 elsewhere — wait put Q11 into bias, put C2 missing),
        # and invalid mirror pair (I1-M10-M11 paired with C1)!
        broken_plan = {
            "schema": "schem_gen.module_plan.v1",
            "cellName": "sky130_bgr_ref",
            "modules": [
                {
                    "id": "bias_mod",
                    "label": "Bias",
                    "role": "bias",
                    "isCore": False,
                    "stage_order": 0,
                    "instances": ["I1", "M10", "M11", "M1", "M2", "M14", "M15", "M5", "Q1", "M6", "M18", "C1"],
                    "skeleton": {
                        "type": "mirror_pair",
                        "leftBranches": [bof["I1"]],
                        "rightBranches": [bof["C1"]],
                    },
                },
                {
                    "id": "rest_mod",
                    "label": "Rest",
                    "role": "gain_stage",
                    "isCore": False,
                    "stage_order": 10,
                    "instances": [
                        "M5", "Q2", "R1",  # duplicate M5 + torn Q1 vs Q2/R1!
                        "R2", "M20", "M19",
                        "Q3", "R3", "M21", "M22", "M25", "Q4", "R4",
                        "Q5", "R5", "M23", "M24", "M26", "Q6", "R6",
                        "M34", "M31", "M27",
                        "Q9", "M32", "Q15", "R7",
                        "Q10", "M33", "Q8", "R8", "R9",
                        "Q11", "Q12", "R10", "R11",
                        # Missing C2!
                    ],
                },
            ],
        }
        rep = validate_module_plan(circuit, broken_plan, analysis)
        codes = {f.code for f in rep.findings}
        self.assertFalse(rep.overall_passed)
        self.assertFalse(rep.coverage_passed)
        self.assertFalse(rep.atomic_chains_passed)
        self.assertFalse(rep.symmetry_passed)
        self.assertIn("MISSING_INSTANCES", codes)
        self.assertIn("DUPLICATE_INSTANCE", codes)
        self.assertIn("TORN_ATOMIC_BRANCH", codes)
        self.assertIn("SYMMETRY_TYPE_MISMATCH", codes)
        self.assertIn("NO_CORE_ANCHOR_MARKED", codes)

        # Counterexample 2: Orphan branch detection — put `R2->M20->M19` (connected only to `M6/M18` via `n_m1`
        # and `C` via `NODE_C`) inside a standalone module paired ONLY with `Q11` (which connects to `n_ea1/n_s1/VREF`)
        orphan_plan = {
            "schema": "schem_gen.module_plan.v1",
            "cellName": "sky130_bgr_ref",
            "modules": [
                {
                    "id": "bias_and_startup",
                    "label": "Bias",
                    "role": "bias",
                    "isCore": False,
                    "stage_order": 0,
                    "branches": [bof["I1"], bof["M1"], bof["M5"], bof["M6"]],
                    "evidence": "Global bias mirrors",
                },
                {
                    "id": "error_amp",
                    "label": "Error Amp",
                    "role": "gain_stage",
                    "isCore": True,
                    "stage_order": 10,
                    "branches": [bof["Q3"], bof["M25"], bof["Q5"], bof["M26"]],
                    "evidence": "Mirrored error amplifier",
                },
                {
                    "id": "bgr_core_with_misplaced_branch",
                    "label": "BGR Core + misplaced Q11",
                    "role": "input_diff",
                    "isCore": True,
                    "stage_order": 20,
                    "branches": [bof["Q9"], bof["Q10"], bof["M34"], bof["R2"], bof["C1"], bof["C2"]],
                    "evidence": "BGR core",
                },
                {
                    "id": "output_mod",
                    "label": "Output",
                    "role": "output_stage",
                    "isCore": False,
                    "stage_order": 30,
                    "branches": [bof["Q12"], bof["Q11"]],
                    "evidence": "Output divider and clamp",
                },
            ],
        }
        # Deliberately move `Q11` into a separate mixed group with `M5` where `Q11` connects only to `output_mod/error_amp`
        # Actually put `C2` (touches NODE_E) into `error_amp` where nothing touches NODE_E!
        orphan_plan["modules"][2]["branches"].remove(bof["C2"])
        orphan_plan["modules"][1]["branches"].append(bof["C2"])
        rep_orphan = validate_module_plan(circuit, orphan_plan, analysis)
        self.assertTrue(rep_orphan.overall_passed)  # warnings only, no hard coverage/torn errors
        self.assertGreaterEqual(rep_orphan.metrics["orphanBranchCount"], 1)
        self.assertIn("ORPHAN_BRANCH_IN_MODULE", {f.code for f in rep_orphan.findings})

    def test_coarse_to_fine_cli_inspect_validate_expand_and_compile_pass(self) -> None:
        """Full Coarse-to-Fine CLI loop (`analyze` -> `inspect` -> `validate-modules` ->
        `plan-to-intent` -> `compile --module-plan`) must pass all semantic, LVS, Bridge & DRC gates
        with ZERO hand-authored per-device orientations or tier overrides."""
        circuit = parse_netlist_text(SYNTHETIC_SKY130_BGR_SPICE)
        analysis = analyze_circuit(circuit)
        bof = analysis.branch_abstraction["branchOfRef"]

        # Verify `inspect` neighborhood tracer hides global bias rails by default
        trace_default = inspect_netlist_neighborhood(circuit, analysis, around="Q15", hops=1, include_bias=False)
        self.assertIn("n_bn", trace_default["hiddenGlobalBiasNets"])
        self.assertIn(bof["Q10"], [b["id"] for b in trace_default["reachedBranches"]])
        self.assertIn("Q15", format_inspect_markdown(trace_default))

        # Verify Stage-2 Meso Focus Window isolates internal private nets vs external interfaces
        focus = inspect_netlist_neighborhood(
            circuit,
            analysis,
            module_targets=[bof["Q3"], bof["M25"], bof["Q5"], bof["M26"]],
        )
        ext_sig_names = {e["net"] for e in focus["externalSignalNets"]}
        ext_bias_names = {e["net"] for e in focus["externalBiasNets"]}
        self.assertEqual(ext_sig_names, {"NODE_A", "NODE_B", "NODE_C", "n_ea1"})
        self.assertEqual(ext_bias_names, {"n_bn", "n_bp"})
        self.assertEqual(len(focus["localQuasiSymmetricPairs"]), 1)

        clean_plan = {
            "schema": "schem_gen.module_plan.v1",
            "cellName": "sky130_bgr_ref",
            "circuitType": "bandgap_reference",
            "modules": [
                {
                    "id": "bias_bank",
                    "label": "偏置源与镜像镜",
                    "role": "bias",
                    "isCore": False,
                    "stage_order": 0,
                    "branches": [bof["I1"], bof["M1"], bof["M5"], bof["M6"]],
                    "evidence": "全局偏置主干生成 n_bn/n_bp 及本地偏置镜像",
                    "skeleton": {
                        "type": "bias_bank",
                        "referenceBranches": [bof["I1"], bof["M1"]],
                        "mirrorBranches": [bof["M5"], bof["M6"]],
                    },
                },
                {
                    "id": "comp_aux",
                    "label": "启动与补偿",
                    "role": "compensation",
                    "isCore": False,
                    "stage_order": 10,
                    "branches": [bof["R2"], bof["C1"], bof["C2"]],
                    "evidence": "连接 NODE_C/NODE_E 的阻容补偿与启动支路",
                },
                {
                    "id": "error_amp",
                    "label": "镜像误差放大器",
                    "role": "gain_stage",
                    "isCore": True,
                    "stage_order": 20,
                    "branches": [bof["Q3"], bof["M25"], bof["Q5"], bof["M26"]],
                    "evidence": "耦合双列镜像放大级，采 NODE_C/NODE_B 驱动 NODE_A/n_ea1",
                    "skeleton": {
                        "type": "mirror_pair",
                        "leftBranches": [bof["Q3"], bof["M25"]],
                        "rightBranches": [bof["Q5"], bof["M26"]],
                    },
                },
                {
                    "id": "cascode_bias",
                    "label": "共栅局部偏置柱",
                    "role": "bias",
                    "isCore": False,
                    "stage_order": 30,
                    "branches": [bof["M34"]],
                    "evidence": "M34/M31/M27 生成局部共栅偏置 n_cas 供带隙核心",
                },
                {
                    "id": "bgr_core",
                    "label": "带隙基准核心",
                    "role": "input_diff",
                    "isCore": True,
                    "stage_order": 40,
                    "branches": [bof["Q9"], bof["Q10"]],
                    "evidence": "8:1 双极准对称带隙核心，共控 NODE_A/n_cas/VREF",
                    "skeleton": {
                        "type": "mirror_pair",
                        "leftBranches": [bof["Q9"]],
                        "rightBranches": [bof["Q10"]],
                    },
                },
                {
                    "id": "output_feedback",
                    "label": "输出分压与钳位",
                    "role": "output_stage",
                    "isCore": False,
                    "stage_order": 50,
                    "branches": [bof["Q11"], bof["Q12"]],
                    "evidence": "Q12/R10/R11 分压输出 VREF 及 Q11 钳位",
                },
            ],
            "netPolicies": {
                "n_bn": "label_stub",
                "n_bp": "label_stub",
                "NODE_E": "perimeter_bus",
                "VREF": "perimeter_bus",
            },
            "portSides": {"VREF": "right", "NODE_E": "right"},
        }

        with tempfile.TemporaryDirectory() as tmp:
            tpath = Path(tmp)
            net_file = tpath / "bgr.spice"
            net_file.write_text(SYNTHETIC_SKY130_BGR_SPICE, encoding="utf-8")

            # 1. analyze emits module_plan.draft.json + upgraded semantic_digest.md
            rc_an = cli_main(["analyze", str(net_file), "--out", str(tpath / "01_analyze")])
            self.assertEqual(rc_an, 0)
            self.assertTrue((tpath / "01_analyze" / "module_plan.draft.json").is_file())
            self.assertTrue((tpath / "01_analyze" / "semantic_digest.md").is_file())

            # 2. inspect CLI (both --around and --module)
            rc_in1 = cli_main(["inspect", str(net_file), "--around", "Q15", "--hops", "1", "--out", str(tpath / "tr.md")])
            rc_in2 = cli_main(
                ["inspect", str(net_file), "--module", f"{bof['Q3']},{bof['M25']},{bof['Q5']},{bof['M26']}", "--out", str(tpath / "mod.md")]
            )
            self.assertEqual(rc_in1, 0)
            self.assertEqual(rc_in2, 0)

            # 3. validate-modules + plan-to-intent + compile --module-plan
            plan_file = tpath / "module_plan.json"
            plan_file.write_text(json.dumps(clean_plan, ensure_ascii=False, indent=2), encoding="utf-8")

            rc_vm = cli_main(["validate-modules", str(net_file), "--plan", str(plan_file), "--out", str(tpath / "02_val")])
            self.assertEqual(rc_vm, 0)
            val_json = json.loads((tpath / "02_val" / "module_audit.json").read_text(encoding="utf-8"))
            self.assertTrue(val_json["overallPassed"])
            self.assertEqual(val_json["metrics"]["tornChainCount"], 0)
            self.assertEqual(val_json["metrics"]["orphanBranchCount"], 0)

            intent_out = tpath / "expanded_intent.json"
            rc_pi = cli_main(["plan-to-intent", str(net_file), "--plan", str(plan_file), "--out", str(intent_out)])
            self.assertEqual(rc_pi, 0)
            self.assertTrue(intent_out.is_file())

            rc_co = cli_main(
                [
                    "compile",
                    str(net_file),
                    "--module-plan",
                    str(plan_file),
                    "--allow-wire-labels",
                    "--out",
                    str(tpath / "03_compile"),
                ]
            )
            self.assertEqual(rc_co, 0)
            audit_data = json.loads((tpath / "03_compile" / "audit.json").read_text(encoding="utf-8"))
            self.assertTrue(audit_data["overallPassed"])
            self.assertTrue(audit_data["bridgeManifestValid"])
            self.assertTrue((tpath / "03_compile" / "intent.expanded.json").is_file())
            self.assertTrue((tpath / "03_compile" / "preview.svg").is_file())

            # 4. Counterexample: compile --module-plan rejects invalid plan BEFORE layout
            bad_plan = dict(clean_plan)
            bad_plan["modules"] = clean_plan["modules"][:-1]  # missing output_feedback devices!
            bad_file = tpath / "bad_plan.json"
            bad_file.write_text(json.dumps(bad_plan, ensure_ascii=False, indent=2), encoding="utf-8")
            rc_bad = cli_main(["compile", str(net_file), "--module-plan", str(bad_file), "--out", str(tpath / "04_bad")])
            self.assertEqual(rc_bad, 2)
            self.assertTrue((tpath / "04_bad" / "module_audit.md").is_file())
            self.assertFalse((tpath / "04_bad" / "manifest.json").exists())


if __name__ == "__main__":
    unittest.main()
