from __future__ import annotations

import copy
import json
from pathlib import Path
import unittest

from schematic_gen.analyzer import analyze_circuit
from schematic_gen.auditor import audit_manifest_and_solved, audit_solved_schematic
from schematic_gen.manifest_builder import (
    build_bridge_manifest,
    verify_with_virtuoso_bridge_offline,
)
from schematic_gen.models import SchematicIntent
from schematic_gen.parser import parse_netlist_file
from schematic_gen.pdk import PdkRegistry
from schematic_gen.refiner import circuit_and_intent_from_manifest, refine_existing_manifest
from schematic_gen.router import compile_schematic


ROOT = Path(__file__).resolve().parent.parent


class TestAuditorCounterexamplesAndBridgeCompat(unittest.TestCase):
    def setUp(self) -> None:
        self.pdk = PdkRegistry()
        self.ota_circuit = parse_netlist_file(ROOT / "examples" / "sky130_two_stage_ota.spice")
        self.ota_intent = SchematicIntent.from_dict(
            json.loads((ROOT / "examples" / "sky130_two_stage_intent.json").read_text(encoding="utf-8"))
        )
        self.ota_solved = compile_schematic(self.ota_circuit, self.ota_intent, pdk=self.pdk)
        self.ota_manifest = build_bridge_manifest(self.ota_solved, self.pdk)

    def test_bridge_exact_geometry_compatibility_zero_violations(self) -> None:
        res = verify_with_virtuoso_bridge_offline(
            self.ota_manifest, self.pdk.raw_map, "sky130"
        )
        self.assertTrue(res["passed"], msg=str(res))
        if res["circuits"]:
            self.assertEqual(res["circuits"][0]["geometryAudit"]["afterViolationCount"], 0)

        report = audit_solved_schematic(self.ota_solved, self.pdk)
        self.assertTrue(report.overall_passed)
        self.assertEqual(report.metrics["wireCrossingCount"], 0)

    def test_counterexample_lvs_wrong_net_caught(self) -> None:
        bad = copy.deepcopy(self.ota_manifest)
        bad["circuits"][0]["instances"][0]["nodes"][0]["netName"] = "VOUT"
        report = audit_manifest_and_solved(bad, self.ota_circuit, self.pdk)
        self.assertFalse(report.electrical_passed)
        codes = {f.code for f in report.findings}
        self.assertTrue("LVS_TERMINAL_NET_MISMATCH" in codes or "BRIDGE_MANIFEST_INVALID" in codes)

    def test_counterexample_split_internal_net_caught(self) -> None:
        """Removing routes on internal net `vout1` must trigger `SPLIT_INTERNAL_NET`."""
        bad = copy.deepcopy(self.ota_manifest)
        geom = bad["circuits"][0]["sourceGeometry"]
        geom["routes"] = [r for r in geom["routes"] if r["netName"] != "vout1"]
        report = audit_manifest_and_solved(bad, self.ota_circuit, self.pdk)
        self.assertFalse(report.electrical_passed)
        codes = {f.code for f in report.findings}
        self.assertIn("SPLIT_INTERNAL_NET", codes)

    def test_counterexample_collinear_foreign_wire_and_ambiguous_corner_caught(self) -> None:
        """Injecting a foreign wire bend/segment collinear with an existing route must fail DRC."""
        bad = copy.deepcopy(self.ota_manifest)
        geom = bad["circuits"][0]["sourceGeometry"]
        # Pick an existing horizontal route segment on `vtail` and overlap a fake bend/segment from `VINP`
        vtail_rt = next(r for r in geom["routes"] if r["netName"] == "vtail")
        p_start = vtail_rt["start"]["sourcePoint"]
        vinp_rt = next(r for r in geom["routes"] if r["netName"] == "VINP")
        vinp_end = vinp_rt["steps"][-1]
        vinp_rt["steps"] = [
            {"kind": "bend", "position": {"x": p_start["x"], "y": vinp_rt["start"]["sourcePoint"]["y"]}},
            {"kind": "bend", "position": {"x": p_start["x"], "y": p_start["y"]}},
            {"kind": "bend", "position": {"x": vinp_end["sourcePoint"]["x"], "y": p_start["y"]}},
            vinp_end,
        ]
        report = audit_manifest_and_solved(bad, self.ota_circuit, self.pdk)
        self.assertFalse(report.readability_passed)
        codes = {f.code for f in report.findings}
        self.assertTrue(
            codes & {"AMBIGUOUS_JUNCTION_CROSSING", "COLLINEAR_WIRE_OVERLAP", "WIRE_OVER_FOREIGN_PIN"}
        )

    def test_cramped_draft_fails_before_and_passes_after_refine(self) -> None:
        draft = json.loads((ROOT / "examples" / "cramped_draft_manifest.json").read_text(encoding="utf-8"))
        c_draft, _ = circuit_and_intent_from_manifest(draft)
        before = audit_manifest_and_solved(draft, c_draft, self.pdk)
        self.assertFalse(before.readability_passed)
        self.assertGreater(before.metrics["symbolOverlapCount"] + before.metrics["labelOverlapCount"], 0)

        solved_after = refine_existing_manifest(draft, pdk=self.pdk)
        after = audit_solved_schematic(solved_after, self.pdk)
        self.assertTrue(after.overall_passed, msg=str([f.to_dict() for f in after.findings]))

    def test_analyzer_detects_motifs_and_zero_shot_auto_passes(self) -> None:
        analysis = analyze_circuit(self.ota_circuit)
        m_types = {m.motif_type for m in analysis.motifs}
        self.assertIn("diff_pair", m_types)
        self.assertIn("current_mirror", m_types)
        self.assertIn("rc_series", m_types)
        self.assertGreaterEqual(len(analysis.ambiguities_for_llm), 1)

        draft_intent = SchematicIntent.from_dict(analysis.draft_intent)
        solved_auto = compile_schematic(self.ota_circuit, draft_intent, pdk=self.pdk)
        rep_auto = audit_solved_schematic(solved_auto, self.pdk)
        self.assertTrue(rep_auto.overall_passed, msg=str([f.to_dict() for f in rep_auto.findings]))


if __name__ == "__main__":
    unittest.main()
