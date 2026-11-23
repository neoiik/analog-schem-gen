from __future__ import annotations

import copy
import json
from pathlib import Path
import tempfile
import unittest

from schematic_gen.analyzer import analyze_circuit
from schematic_gen.auditor import audit_solved_schematic
from schematic_gen.hierarchy import (
    compile_hierarchical_circuit,
    render_hierarchy_book_html,
    verify_hierarchy_flatten_lvs,
)
from schematic_gen.interactive_editor import render_interactive_editor_html
from schematic_gen.models import SchematicIntent
from schematic_gen.optimizer import optimize_schematic_crossings
from schematic_gen.parser import (
    eval_spice_expr,
    parse_netlist_design_text,
    parse_netlist_file,
    parse_netlist_text,
)
from schematic_gen.pdk import PdkRegistry
from schematic_gen.pdk_extractor import (
    extract_process_map_from_xschem_dir,
    parse_xschem_sym_file,
)
from schematic_gen.router import compile_schematic


ROOT = Path(__file__).resolve().parent.parent


class TestV02EvolutionFeatures(unittest.TestCase):
    def test_spice_param_algebraic_evaluation_and_suffixes(self) -> None:
        env = {"a2_tail": 23.0, "a2_rdeg_pu": 13500.0, "a2_fold_ratio": 5.5}
        self.assertAlmostEqual(eval_spice_expr("{48*a2_tail}", env) or 0.0, 1104.0)
        self.assertAlmostEqual(
            eval_spice_expr("{a2_rdeg_pu/a2_fold_ratio}", env) or 0.0, 13500.0 / 5.5
        )
        self.assertAlmostEqual(eval_spice_expr("9.1meg", env) or 0.0, 9.1e6)
        self.assertAlmostEqual(eval_spice_expr("0.45p", env) or 0.0, 0.45e-12)

        txt = """
        .param unit_w=12
        .param r_val=15k
        .subckt param_cell IN OUT VDD VSS
        XM1 OUT IN VSS VSS sky130_fd_pr__nfet_01v8 w={2*unit_w} l=0.5 nf=2
        R1 VDD OUT {2*r_val}
        .ends param_cell
        """
        circuit = parse_netlist_text(txt)
        m1 = circuit.instance_map()["M1"]
        r1 = circuit.instance_map()["R1"]
        self.assertEqual(m1.params["w"], "24u")
        self.assertEqual(m1.param_expressions["w"], "{2*unit_w}")
        self.assertEqual(r1.params["r"], "30k")

    def test_regular_cdac_parallel_bank_and_serpentine_ladder_uniform_pitch(self) -> None:
        pdk = PdkRegistry()
        circuit = parse_netlist_file(ROOT / "examples" / "sky130_cdac_and_ladder.spice")
        analysis = analyze_circuit(circuit)
        m_types = {m.motif_type for m in analysis.motifs}
        self.assertIn("parallel_bank", m_types)
        self.assertIn("serpentine_chain", m_types)

        intent = SchematicIntent.from_dict(analysis.draft_intent)
        solved = compile_schematic(circuit, intent, pdk=pdk)
        rep = audit_solved_schematic(solved, pdk)
        self.assertTrue(rep.overall_passed, msg=str([f.to_dict() for f in rep.findings]))

        # Verify CDAC bank capacitors have strictly identical Y and exact uniform X pitch!
        cap_refs = ["CDUM", "C0", "C1", "C2", "C3", "C4"]
        cap_xs = [solved.placements[r].x for r in cap_refs]
        cap_ys = {solved.placements[r].y for r in cap_refs}
        self.assertEqual(len(cap_ys), 1)
        pitches = {b - a for a, b in zip(cap_xs, cap_xs[1:])}
        self.assertEqual(len(pitches), 1, f"Expected uniform CDAC pitch, got {pitches}")

    def test_bounded_local_flip_optimizer_reduces_crossings_without_drc_error(self) -> None:
        pdk = PdkRegistry()
        circuit = parse_netlist_file(ROOT / "examples" / "sky130_two_stage_ota.spice")
        intent = SchematicIntent.from_dict(
            json.loads((ROOT / "examples" / "sky130_two_stage_intent.json").read_text(encoding="utf-8"))
        )
        # Deliberately degrade Stage 2 & port/bias settings to induce avoidable crossings (`vbias_n` crossing `VINP`/`VINN`)
        bad_intent = copy.deepcopy(intent)
        bad_intent.port_sides["VINP"] = "left"
        bad_intent.port_sides["VINN"] = "left"
        bad_intent.tier_overrides["IB0"] = 2
        bad_intent.net_policies["vbias_n"] = "full_wire"

        before_solved = compile_schematic(circuit, bad_intent, pdk=pdk)
        before_rep = audit_solved_schematic(before_solved, pdk)

        opt_solved = optimize_schematic_crossings(circuit, bad_intent, pdk=pdk, max_evals=16)
        opt_rep = audit_solved_schematic(opt_solved, pdk)

        self.assertTrue(opt_rep.overall_passed)
        self.assertLessEqual(
            opt_rep.metrics["wireCrossingCount"], before_rep.metrics["wireCrossingCount"]
        )
        if before_rep.metrics["wireCrossingCount"] > 0:
            self.assertLess(
                opt_rep.metrics["wireCrossingCount"], before_rep.metrics["wireCrossingCount"]
            )

    def test_wire_label_stubs_and_interactive_editor_export(self) -> None:
        pdk = PdkRegistry()
        circuit = parse_netlist_file(ROOT / "examples" / "sky130_two_stage_ota.spice")
        intent = SchematicIntent.from_dict(
            json.loads((ROOT / "examples" / "sky130_two_stage_intent.json").read_text(encoding="utf-8"))
        )
        intent.net_policies["vbias_n"] = "label_stub"
        solved = compile_schematic(circuit, intent, pdk=pdk, allow_wire_labels=True)
        self.assertGreaterEqual(len(solved.wire_label_stubs), 2)
        rep = audit_solved_schematic(solved, pdk)
        self.assertTrue(rep.overall_passed, msg=str([f.to_dict() for f in rep.findings]))

        editor_html = render_interactive_editor_html(solved, pdk)
        self.assertIn("draft_manifest.json", editor_html)
        self.assertIn("sky130_two_stage_ota", editor_html)

    def test_xschem_sym_extractor_known_pin_offsets(self) -> None:
        sym_content = """v {xschem version=3.4.5 file_version=1.2}
K {type=nmos}
L 4 -20 0 10 0 {}
L 4 10 -30 10 30 {}
B 5 7.5 -32.5 12.5 -27.5 {name=d dir=inout}
B 5 -22.5 -2.5 -17.5 2.5 {name=g dir=in}
B 5 7.5 27.5 12.5 32.5 {name=s dir=inout}
B 5 7.5 -2.5 12.5 2.5 {name=b dir=in}
"""
        with tempfile.TemporaryDirectory() as tmp:
            tdir = Path(tmp)
            (tdir / "nfet_01v8.sym").write_text(sym_content, encoding="utf-8")
            parsed = parse_xschem_sym_file(tdir / "nfet_01v8.sym", xschem_unit_per_grid=10.0)
            # Xschem y=-30 (top) -> Virtuoso y=+3; Xschem y=+30 (bottom) -> Virtuoso y=-3
            self.assertEqual(parsed["pinOffsets"]["d"], [1, 3])
            self.assertEqual(parsed["pinOffsets"]["g"], [-2, 0])
            self.assertEqual(parsed["pinOffsets"]["s"], [1, -3])

            pmap = extract_process_map_from_xschem_dir(tdir, process_name="open_test")
            self.assertIn("nmos", pmap["processes"]["open_test"]["devices"])

    def test_hierarchy_flatten_back_lvs_and_counterexample(self) -> None:
        pdk = PdkRegistry()
        # Multi-subckt design check + flat-to-hierarchy check on two-stage OTA (max_page_size=5)
        circuit = parse_netlist_file(ROOT / "examples" / "sky130_two_stage_ota.spice")
        res = compile_hierarchical_circuit(
            circuit, pdk=pdk, max_page_size=5, allow_wire_labels=True, optimize_crossings=False
        )
        self.assertTrue(res.flatten_lvs_passed, msg=str(res.flatten_lvs_Diffs))
        self.assertTrue(res.overall_passed)

        # Counterexample: corrupting one top-level block pin connection MUST fail flatten-back LVS!
        bad_manifest = copy.deepcopy(res.combined_manifest)
        top_entry = next(
            c for c in bad_manifest["circuits"] if c["cellName"] == circuit.cell_name
        )
        top_entry["instances"][0]["nodes"][0]["netName"] = "VSS"
        ok_bad, diffs_bad = verify_hierarchy_flatten_lvs(
            circuit, bad_manifest, circuit.cell_name
        )
        self.assertFalse(ok_bad)
        self.assertGreater(len(diffs_bad), 0)

        # Also test native multi-subckt SPICE parsing
        multi_txt = """
        .subckt bias_blk VBN VDD VSS
        IB0 VDD VBN 10u
        XM0 VBN VBN VSS VSS sky130_fd_pr__nfet_01v8 w=4 l=0.5
        .ends bias_blk

        .subckt amp_top VIN VOUT VDD VSS
        XB1 vbn_int VDD VSS bias_blk
        XM1 VOUT VIN VSS VSS sky130_fd_pr__nfet_01v8 w=8 l=0.5
        XM2 VOUT vbn_int VDD VDD sky130_fd_pr__pfet_01v8 w=16 l=0.5
        .ends amp_top
        """
        design = parse_netlist_design_text(multi_txt)
        self.assertEqual(design.top_cell, "amp_top")
        res_multi = compile_hierarchical_circuit(design, pdk=pdk, optimize_crossings=False)
        self.assertTrue(res_multi.overall_passed)

    def test_07_opa316_162_instance_hierarchy_and_flatten_lvs(self) -> None:
        opa_path = ROOT / "examples" / "sky130_rcopa316_core.spice"
        circuit = parse_netlist_file(opa_path)
        self.assertEqual(len(circuit.instances), 162)
        pdk = PdkRegistry()
        res = compile_hierarchical_circuit(circuit, pdk=pdk, optimize_crossings=False)
        self.assertTrue(res.flatten_lvs_passed, f"Flatten LVS diffs: {res.flatten_lvs_Diffs}")
        self.assertTrue(res.overall_passed)
        self.assertEqual(len(res.solved_pages), 11)
        book = render_hierarchy_book_html(res, pdk)
        self.assertIn("opamp_core", book)


if __name__ == "__main__":
    unittest.main()
