from __future__ import annotations

import json
from pathlib import Path
import tempfile
import unittest

from schem_gen import main as cli_main


ROOT = Path(__file__).resolve().parent.parent


class TestEndToEndCli(unittest.TestCase):
    def test_analyze_then_compile_and_audit_workflow(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            netlist = ROOT / "examples" / "sky130_two_stage_ota.spice"

            rc1 = cli_main(["analyze", str(netlist), "--out", str(tmp_path / "step1")])
            self.assertEqual(rc1, 0)
            draft_intent = tmp_path / "step1" / "intent.draft.json"
            self.assertTrue(draft_intent.is_file())

            rc2 = cli_main(
                [
                    "compile",
                    str(netlist),
                    "--intent",
                    str(draft_intent),
                    "--out",
                    str(tmp_path / "step2"),
                ]
            )
            self.assertEqual(rc2, 0)
            self.assertTrue((tmp_path / "step2" / "manifest.json").is_file())
            self.assertTrue((tmp_path / "step2" / "preview.svg").is_file())

            rc3 = cli_main(
                [
                    "audit",
                    str(tmp_path / "step2" / "manifest.json"),
                    "--netlist",
                    str(netlist),
                    "--out",
                    str(tmp_path / "step3"),
                ]
            )
            self.assertEqual(rc3, 0)
            audit_data = json.loads((tmp_path / "step3" / "audit.json").read_text(encoding="utf-8"))
            self.assertTrue(audit_data["overallPassed"])

    def test_folded_cascode_auto_and_safe_probe_pdk(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            tmp_path = Path(tmp)
            netlist = ROOT / "examples" / "sky130_folded_cascode.spice"
            rc = cli_main(["auto", str(netlist), "--out", str(tmp_path / "fc_auto")])
            self.assertEqual(rc, 0)

            rc_probe = cli_main(["probe-pdk"])
            self.assertEqual(rc_probe, 0)


if __name__ == "__main__":
    unittest.main()
