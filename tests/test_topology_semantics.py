"""Known-solution & counterexample tests for Pass 0-3 semantic topology analysis and semantic_digest.md."""

from __future__ import annotations

import tempfile
from pathlib import Path
import unittest

from schem_gen import main as cli_main
from schematic_gen.analyzer import analyze_circuit, detect_co_gated_mos_stacks
from schematic_gen.auditor import audit_solved_schematic
from schematic_gen.models import SchematicIntent
from schematic_gen.parser import parse_netlist_text
from schematic_gen.pdk import PdkRegistry
from schematic_gen.router import compile_schematic
from schematic_gen.topology_semantics import (
    classify_device_roles,
    detect_complementary_sf,
    detect_miller_bridges,
)


DUAL_PATH_GAIN_BOOSTED_OPAMP_SPICE = """
.subckt dut VDD VSS VB5 Vb6 Vb7 VB8 VB2 VB3 VB4 VB1 Vin- Vin+ VOUT
C1 VOUT VSS 1p
XM1 net9 net2 VDD VDD sky130_fd_pr__pfet_01v8 l=0.15 w=1 nf=1 m=1
XM10 net0 net13 VDD VDD sky130_fd_pr__pfet_01v8 l=0.15 w=1 nf=1 m=1
XM11 net10 net13 net10 VSS sky130_fd_pr__nfet_01v8 l=0.15 w=1 nf=1 m=1
XM12 Vb6 Vb7 net10 VDD sky130_fd_pr__pfet_01v8 l=0.15 w=1 nf=1 m=1
XM13 net0 Vb7 Vb6 VDD sky130_fd_pr__pfet_01v8 l=0.15 w=1 nf=1 m=1
XM14 net0 VB8 VSS VSS sky130_fd_pr__nfet_01v8 l=0.15 w=1 nf=1 m=1
XM15 net12 net0 VDD VDD sky130_fd_pr__pfet_01v8 l=0.15 w=1 nf=1 m=1
XM16 net12 net12 VSS VSS sky130_fd_pr__nfet_01v8 l=0.15 w=1 nf=1 m=1
XM17 net14 net15 VDD VDD sky130_fd_pr__pfet_01v8 l=0.15 w=1 nf=1 m=1
XM18 net15 VB2 VDD VDD sky130_fd_pr__pfet_01v8 l=0.15 w=1 nf=1 m=1
XM19 net15 VB3 net14 VSS sky130_fd_pr__nfet_01v8 l=0.15 w=1 nf=1 m=1
XM2 net11 net2 VDD VDD sky130_fd_pr__pfet_01v8 l=0.15 w=1 nf=1 m=1
XM20 net14 VB4 VSS VSS sky130_fd_pr__nfet_01v8 l=0.15 w=1 nf=1 m=1
XM21 net15 net16 net15 VSS sky130_fd_pr__nfet_01v8 l=0.15 w=1 nf=1 m=1
XM22 net17 net16 VDD VDD sky130_fd_pr__pfet_01v8 l=0.15 w=1 nf=1 m=1
XM23 net16 net16 net17 VDD sky130_fd_pr__pfet_01v8 l=0.15 w=1 nf=1 m=1
XM24 net18 net16 VDD VDD sky130_fd_pr__pfet_01v8 l=0.15 w=1 nf=1 m=1
XM25 net19 net16 net18 VDD sky130_fd_pr__pfet_01v8 l=0.15 w=1 nf=1 m=1
XM26 net16 Vin- net20 VSS sky130_fd_pr__nfet_01v8 l=0.15 w=1 nf=1 m=1
XM27 net20 Vin- net1 VSS sky130_fd_pr__nfet_01v8 l=0.15 w=1 nf=1 m=1
XM28 net19 Vin+ net21 VSS sky130_fd_pr__nfet_01v8 l=0.15 w=1 nf=1 m=1
XM29 net21 Vin+ net1 VSS sky130_fd_pr__nfet_01v8 l=0.15 w=1 nf=1 m=1
XM3 net2 net2 net9 VDD sky130_fd_pr__pfet_01v8 l=0.15 w=1 nf=1 m=1
XM30 net1 VB1 VSS VSS sky130_fd_pr__nfet_01v8 l=0.15 w=1 nf=1 m=1
XM31 net4 net19 net4 VSS sky130_fd_pr__nfet_01v8 l=0.15 w=1 nf=1 m=1
XM32 net4 VB2 VDD VDD sky130_fd_pr__pfet_01v8 l=0.15 w=1 nf=1 m=1
XM33 net4 VB3 net3 VSS sky130_fd_pr__nfet_01v8 l=0.15 w=1 nf=1 m=1
XM34 net3 VB4 VSS VSS sky130_fd_pr__nfet_01v8 l=0.15 w=1 nf=1 m=1
XM35 net3 net4 VDD VDD sky130_fd_pr__pfet_01v8 l=0.15 w=1 nf=1 m=1
XM36 net6 net4 VDD VDD sky130_fd_pr__pfet_01v8 l=0.15 w=1 nf=1 m=1
XM37 net6 net12 VSS VSS sky130_fd_pr__nfet_01v8 l=0.15 w=1 nf=1 m=1
XM38 VDD net6 VOUT VSS sky130_fd_pr__nfet_01v8 l=0.15 w=1 nf=1 m=1
XM39 VSS net6 VOUT VDD sky130_fd_pr__pfet_01v8 l=0.15 w=1 nf=1 m=1
XM4 net13 net2 net11 VDD sky130_fd_pr__pfet_01v8 l=0.15 w=1 nf=1 m=1
XM5 net2 Vin- net5 VSS sky130_fd_pr__nfet_01v8 l=0.15 w=1 nf=1 m=1
XM6 net5 Vin- net8 VSS sky130_fd_pr__nfet_01v8 l=0.15 w=1 nf=1 m=1
XM7 net13 Vin+ net7 VSS sky130_fd_pr__nfet_01v8 l=0.15 w=1 nf=1 m=1
XM8 net7 Vin+ net8 VSS sky130_fd_pr__nfet_01v8 l=0.15 w=1 nf=1 m=1
XM9 net8 VB5 VSS VSS sky130_fd_pr__nfet_01v8 l=0.15 w=1 nf=1 m=1
R1 net3 net14 1k
.ends dut
"""


class TestTopologySemanticsPass0To3(unittest.TestCase):
    def test_pass0_to_pass3_known_solution_dual_path_gain_boosted_opamp(self) -> None:
        """Pass 0-3 must automatically recover:
        - Pass 0: MOSCAPs (M11,M21,M31), active MOS resistors (M12,M13, NOT cascode M19/M33),
                  source followers (M38,M39), and two complete 9-FET composite diff pairs;
        - Pass 1: Generalized Miller bridge (M11-M12-M13 across M10) & complementary SF (M38/M39+C1);
        - Pass 2: Quasi-mirror sandwich core (M17-M21 <- M22-M30 + R1 -> M31-M35) tolerating M36.G@net4;
        - Pass 3: Exact 3-stage macro backbone (Gain Booster | Main Amplifier | Output Buffer)
                  separated by bottleneck cuts `net0` and `net6`, plus zero-shot `auto` PASS with low crossings.
        """
        circuit = parse_netlist_text(DUAL_PATH_GAIN_BOOSTED_OPAMP_SPICE)
        analysis = analyze_circuit(circuit)

        # Pass 0 device roles
        roles = analysis.device_roles
        for cap_r in ("M11", "M21", "M31"):
            self.assertEqual(roles[cap_r]["role"], "mos_cap")
        for res_r in ("M12", "M13"):
            self.assertEqual(roles[res_r]["role"], "mos_active_res")
        # Counterexample inside the same circuit: DC cascode devices M19/M33 touching MOSCAPs M21/M31
        # must NEVER be misclassified as active MOS resistors!
        self.assertNotEqual(roles["M19"]["role"], "mos_active_res")
        self.assertNotEqual(roles["M33"]["role"], "mos_active_res")
        self.assertEqual(roles["M38"]["role"], "source_follower")
        self.assertEqual(roles["M39"]["role"], "source_follower")

        # Pass 0 composite diff pairs: both 9-FET blocks complete & diode reference leg on LEFT
        diff_motifs = [m for m in analysis.motifs if m.motif_type == "diff_pair"]
        self.assertEqual(len(diff_motifs), 2)
        diff_sets = [set(m.instances) for m in diff_motifs]
        self.assertIn({"M1", "M2", "M3", "M4", "M5", "M6", "M7", "M8", "M9"}, diff_sets)
        self.assertIn({"M22", "M23", "M24", "M25", "M26", "M27", "M28", "M29", "M30"}, diff_sets)
        for dm in diff_motifs:
            # Diode reference load (M3 or M23) must sit on the left so output feeds rightward
            self.assertIn(dm.roles["active_load_left"], {"M3", "M23"})

        # Pass 1 generalized Miller bridge & complementary SF
        millers = [m for m in analysis.motifs if m.motif_type == "miller_bridge"]
        self.assertEqual(len(millers), 1)
        self.assertEqual(millers[0].roles["host_amp"], "M10")
        self.assertEqual(millers[0].instances, ["M11", "M12", "M13"])
        self.assertEqual(millers[0].roles["host_loads"], ["M14"])

        sfs = [m for m in analysis.motifs if m.motif_type == "complementary_sf"]
        self.assertEqual(len(sfs), 1)
        self.assertEqual(set(sfs[0].instances), {"M38", "M39", "C1"})

        # Pass 2 sandwich quasi-mirror core tolerating single-ended tap M36.G@net4
        sandwiches = [m for m in analysis.motifs if m.motif_type == "sandwich_mirror_core"]
        self.assertEqual(len(sandwiches), 1)
        sc = sandwiches[0]
        self.assertEqual(sc.roles["bridgeDevices"], ["R1"])
        self.assertEqual(
            {tuple(p) for p in sc.roles["pairs"]},
            {("M17", "M35"), ("M18", "M32"), ("M19", "M33"), ("M20", "M34"), ("M21", "M31")},
        )
        tap_refs = {(t["ref"], t["pin"], t["net"]) for t in sc.roles["asymmetricTaps"]}
        self.assertEqual(tap_refs, {("M36", "G", "net4")})

        # Pass 3 directed signal-flow backbone & exact 3-macro-stage bottleneck cut partition
        bb = analysis.signal_backbone
        self.assertEqual(set(bb["selectedCutNets"]), {"net0", "net6"})
        stages = bb["macroStages"]
        self.assertEqual(len(stages), 3)
        self.assertEqual(
            set(stages[0]["instances"]),
            {"M1", "M2", "M3", "M4", "M5", "M6", "M7", "M8", "M9", "M10", "M11", "M12", "M13", "M14"},
        )
        self.assertEqual(
            set(stages[1]["instances"]),
            {
                "M15", "M16", "M17", "M18", "M19", "M20", "M21",
                "M22", "M23", "M24", "M25", "M26", "M27", "M28", "M29", "M30", "R1",
                "M31", "M32", "M33", "M34", "M35", "M36", "M37",
            },
        )
        self.assertEqual(set(stages[2]["instances"]), {"M38", "M39", "C1"})

        # Boundary ambiguity on M15/M16 between net0 and net12 must be surfaced to LLM
        amb_cats = {a.category for a in analysis.ambiguities_for_llm}
        self.assertIn("macro_boundary", amb_cats)
        self.assertIn("quasi_symmetry", amb_cats)
        self.assertIn("bridge_style", amb_cats)

        # Compact LLM semantic digest must include all three macro stages & special device highlights
        digest = analysis.semantic_digest_md
        self.assertIn("Gain Booster", digest)
        self.assertIn("Main Amplifier", digest)
        self.assertIn("Output Buffer", digest)
        self.assertIn("M11", digest)
        self.assertIn("M36.G@net4", digest)

        # Zero-shot compile (both strict physical wires and native wire-labels) must pass all DRC/LVS gates
        pdk = PdkRegistry()
        intent = SchematicIntent.from_dict(analysis.draft_intent)
        solved_strict = compile_schematic(circuit, intent, pdk=pdk, allow_wire_labels=False)
        rep_strict = audit_solved_schematic(solved_strict, pdk)
        self.assertTrue(rep_strict.overall_passed, [f.message for f in rep_strict.findings])
        self.assertTrue(rep_strict.bridge_manifest_valid)
        self.assertLessEqual(rep_strict.metrics["wireCrossingCount"], 28)

        solved_labels = compile_schematic(circuit, intent, pdk=pdk, allow_wire_labels=True)
        rep_labels = audit_solved_schematic(solved_labels, pdk)
        self.assertTrue(rep_labels.overall_passed, [f.message for f in rep_labels.findings])
        self.assertTrue(rep_labels.bridge_manifest_valid)
        self.assertLessEqual(rep_labels.metrics["wireCrossingCount"], 8)

        # Verify Phase 2 horizontal MOS Miller bridge (`R90`), vertical sandwich (`M10 < M12 < M14`),
        # and compact non-bloated symmetric diff pair (`M27->M29 == M6->M8 <= 26`)!
        pls = solved_labels.placements
        self.assertEqual(pls["M12"].semantic_orient, "horizontal_gate_down")
        self.assertEqual(pls["M13"].semantic_orient, "horizontal_gate_down")
        self.assertEqual(pls["M12"].cadence_orient, "R90")
        self.assertEqual(pls["M11"].y, pls["M12"].y)
        self.assertEqual(pls["M12"].y, pls["M13"].y)
        self.assertTrue(
            pls["M10"].y < pls["M12"].y < pls["M14"].y,
            f"Expected M10 above horizontal Miller bridge M12 above M14, got y={(pls['M10'].y, pls['M12'].y, pls['M14'].y)}",
        )
        span_diff_b = pls["M8"].x - pls["M6"].x
        span_diff_a = pls["M29"].x - pls["M27"].x
        self.assertLessEqual(span_diff_a, 26, f"Sandwich diff pair M27->M29 must stay compact (not bloated), got {span_diff_a}")
        self.assertEqual(span_diff_a, span_diff_b, f"Both cascoded diff pairs should have matching compact span: {span_diff_a} vs {span_diff_b}")
        # Under-tail bridge R1 must stay centered under tail M30 without bloating diff pair width
        self.assertEqual(pls["R1"].x, pls["M30"].x)
        self.assertGreater(pls["R1"].y, pls["M30"].y)

        # Every supply/ground port occurrence must be wired (zero orphan floating ground symbols)
        # and bottom VSS ground taps must sit tightly above lower under-tail bridge R1.
        route_pts_by_net: dict[str, set[tuple[int, int]]] = {}
        for rt in solved_labels.routes:
            route_pts_by_net.setdefault(rt.net_name, set()).update(rt.points())
        vss_ports = [po for po in solved_labels.ports if po.net_name == "VSS"]
        self.assertGreaterEqual(len(vss_ports), 1)
        for po in solved_labels.ports:
            if po.net_name in {"VDD", "VSS"}:
                self.assertIn(
                    (po.x, po.y),
                    route_pts_by_net.get(po.net_name, set()),
                    f"Orphan unconnected rail port {po.occurrence_id} at {(po.x, po.y)}",
                )
        for po in vss_ports:
            self.assertLess(po.y, pls["R1"].y, f"VSS port {po.occurrence_id} drifted below R1: y={po.y} vs R1.y={pls['R1'].y}")

        # Verify macro left-to-right monotonic progression on solved coordinates:
        # M6 (Gain Booster) < M15 (Main Amp entry) < M30 (Sandwich Center) < M36 (Combiner) < M38 (Output Buffer)
        self.assertTrue(
            pls["M6"].x < pls["M10"].x < pls["M15"].x < pls["M18"].x < pls["M30"].x < pls["M32"].x < pls["M36"].x < pls["M38"].x,
            f"Expected exact left-to-right macro order, got {[pls[r].x for r in ('M6','M10','M15','M18','M30','M32','M36','M38')]}",
        )

    def test_counterexample_grounded_decoupling_moscap_not_miller_bridge(self) -> None:
        """A grounded decoupling MOSCAP (D=S=VSS) must NOT be falsely classified as a Miller bridge
        or trigger false `mos_active_res` labeling on adjacent rail-anchored transistors."""
        spice = """
        .subckt decouple_ce VDD VSS VIN VOUT VB
        XM1 VOUT VIN VSS VSS sky130_fd_pr__nfet_01v8 w=2 l=0.5 nf=1 m=1
        XM2 VOUT VB VDD VDD sky130_fd_pr__pfet_01v8 w=2 l=0.5 nf=1 m=1
        XMCAP VSS VOUT VSS VSS sky130_fd_pr__nfet_01v8 w=4 l=2 nf=1 m=1
        .ends decouple_ce
        """
        circuit = parse_netlist_text(spice)
        stks = detect_co_gated_mos_stacks(circuit)
        roles = classify_device_roles(circuit, stks)
        self.assertEqual(roles["MCAP"]["role"], "mos_cap")
        self.assertNotEqual(roles["M1"]["role"], "mos_active_res")
        self.assertNotEqual(roles["M2"]["role"], "mos_active_res")
        self.assertEqual(detect_miller_bridges(circuit, roles), [])
        self.assertEqual(detect_complementary_sf(circuit, roles), [])

    def test_horizontal_mos_orientation_pins_bulk_stub_and_bridge_compat(self) -> None:
        """Horizontal MOS orientations (`horizontal_gate_down` = R90, `horizontal_gate_up` = R270)
        must produce exact rotated pin coordinates, vertical 2D bulk stubs matching virtuoso-bridge-lite,
        and 0-violation SVG/Bridge/DRC compilation."""
        from schematic_gen.pdk import semantic_to_transform

        pdk = PdkRegistry()
        rot, mir, cad = semantic_to_transform("horizontal_gate_down")
        self.assertEqual(cad, "R90")
        pins_p = pdk.source_pin_offsets("pmos", rot, mir)
        # PMOS R90: S on left (-3, -1), D on right (+3, -1), G facing down (0, +3), B facing up (0, -1)
        self.assertEqual(pins_p["S"], (-3, -1))
        self.assertEqual(pins_p["D"], (3, -1))
        self.assertEqual(pins_p["G"], (0, 3))
        self.assertEqual(pins_p["B"], (0, -1))

        rot_u, mir_u, cad_u = semantic_to_transform("horizontal_gate_up")
        self.assertEqual(cad_u, "R270")
        pins_pu = pdk.source_pin_offsets("pmos", rot_u, mir_u)
        self.assertEqual(pins_pu["G"], (0, -3))
        self.assertEqual(pins_pu["B"], (0, 1))

    def test_cli_analyze_emits_semantic_digest_md(self) -> None:
        """`schem_gen.py analyze` must emit `semantic_digest.md` alongside `analysis.json` and `intent.draft.json`."""
        with tempfile.TemporaryDirectory() as tmp:
            tpath = Path(tmp)
            net_file = tpath / "dut.spice"
            net_file.write_text(DUAL_PATH_GAIN_BOOSTED_OPAMP_SPICE, encoding="utf-8")
            rc = cli_main(["analyze", str(net_file), "--out", str(tpath / "out")])
            self.assertEqual(rc, 0)
            digest_file = tpath / "out" / "semantic_digest.md"
            self.assertTrue(digest_file.is_file())
            txt = digest_file.read_text(encoding="utf-8")
            self.assertIn("Signal-Flow Backbone", txt)
            self.assertIn("BOTTLENECK CUT", txt)


if __name__ == "__main__":
    unittest.main()
