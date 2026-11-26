"""Architecture-consistency regression tests (v0.3.2).

Locks in the guarantees from the architecture review:
  1. Single source of atomicity: every benchmark draft (intent + module plan) passes `validate-modules`
     with zero HARD tears, and automatic paging never splits a HARD segment.
  2. Consistent compile paths: `compile --module-plan` on the untouched draft is never worse than `auto`.
  3. Contract fixes: camelCase spacing honored, unknown keys warn; rails from the shared rules file.
  4. Optimizer never moves devices of a declared symmetry skeleton.
  5. No demo vocabulary (OPA316 section titles / names) inside tool rules.
  6. Pass 3 backbone flagged as not applicable only on degenerate partitions.
"""

from __future__ import annotations

from pathlib import Path
import re
import sys
import unittest
import warnings

sys.path.insert(0, str(Path(__file__).resolve().parent))

from schematic_gen.analyzer import analyze_circuit
from schematic_gen.auditor import audit_solved_schematic
from schematic_gen.hierarchy import propose_flat_partition
from schematic_gen.models import SchematicIntent, SpacingConfig
from schematic_gen.optimizer import optimize_schematic_crossings
from schematic_gen.parser import classify_rail_nets, parse_netlist_file, parse_netlist_text
from schematic_gen.pdk import PdkRegistry
from schematic_gen.router import compile_schematic
from schematic_gen.semantic_validator import (
    expand_module_plan_to_intent,
    propose_module_plan_draft,
    validate_module_plan,
)
from test_branch_abstraction_and_validator import SYNTHETIC_SKY130_BGR_SPICE, SYNTHETIC_SKY130_IDAC_ARRAY_SPICE
from test_topology_semantics import DUAL_PATH_GAIN_BOOSTED_OPAMP_SPICE


ROOT = Path(__file__).resolve().parent.parent
EXAMPLES = ROOT / "examples"


def _benchmark_circuits() -> dict[str, object]:
    out: dict[str, object] = {
        "synthetic_bgr": parse_netlist_text(SYNTHETIC_SKY130_BGR_SPICE),
        "synthetic_idac": parse_netlist_text(SYNTHETIC_SKY130_IDAC_ARRAY_SPICE),
        "dual_path_opamp": parse_netlist_text(DUAL_PATH_GAIN_BOOSTED_OPAMP_SPICE),
    }
    for f in sorted(EXAMPLES.glob("*.spice")):
        out[f.stem] = parse_netlist_file(f)
    return out


class TestArchitectureConsistency(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.circuits = _benchmark_circuits()
        cls.analyses = {k: analyze_circuit(c) for k, c in cls.circuits.items()}
        cls.pdk = PdkRegistry()

    def test_drafts_pass_own_semantic_gate_with_zero_hard_tears(self) -> None:
        for name, c in self.circuits.items():
            a = self.analyses[name]
            rep_i = validate_module_plan(c, a.draft_intent, a)
            self.assertEqual(rep_i.metrics["tornChainCount"], 0, f"{name} intent draft: {[f.message for f in rep_i.findings if f.severity == 'ERROR']}")
            self.assertTrue(rep_i.overall_passed, f"{name} intent draft failed own gate")
            plan = propose_module_plan_draft(c, a)
            rep_m = validate_module_plan(c, plan, a)
            self.assertTrue(rep_m.overall_passed, f"{name} module_plan draft failed own gate")
            self.assertFalse(any("skeleton" in m for m in plan["modules"]), "draft must not activate skeletons")

    def test_module_plan_draft_reproduces_intent_draft_membership(self) -> None:
        for name, c in self.circuits.items():
            a = self.analyses[name]
            plan = propose_module_plan_draft(c, a)
            it = expand_module_plan_to_intent(c, plan, a)
            g_draft = {r: g["id"] for g in a.draft_intent["groups"] for r in g["instances"]}
            g_exp = {r: g.id for g in it.groups for r in g.instances}
            moved = sorted(r for r in g_draft if g_exp.get(r) != g_draft[r])
            self.assertEqual(moved, [], f"{name}: module draft moved {moved}")

    def test_hierarchy_paging_never_splits_hard_segments(self) -> None:
        for name, c in self.circuits.items():
            if len(c.instances) <= 32:
                continue
            a = self.analyses[name]
            h = propose_flat_partition(c)
            plan = {"groups": [{"id": p.page_id, "instances": p.instances} for p in h.pages]}
            rep = validate_module_plan(c, plan, a)
            self.assertEqual(rep.metrics["tornChainCount"], 0, f"{name} paging tore HARD segments")

    def test_module_plan_path_never_worse_than_auto(self) -> None:
        for name in ("synthetic_bgr", "dual_path_opamp", "sky130_two_stage_ota", "sky130_folded_cascode"):
            c = self.circuits[name]
            a = self.analyses[name]
            s_auto = compile_schematic(c, SchematicIntent.from_dict(a.draft_intent), pdk=self.pdk, allow_wire_labels=True)
            r_auto = audit_solved_schematic(s_auto, self.pdk)
            it = expand_module_plan_to_intent(c, propose_module_plan_draft(c, a), a)
            s_mod = compile_schematic(c, it, pdk=self.pdk, allow_wire_labels=True)
            r_mod = audit_solved_schematic(s_mod, self.pdk)
            self.assertTrue(r_auto.overall_passed and r_mod.overall_passed, name)
            self.assertLessEqual(
                r_mod.metrics["wireCrossingCount"],
                r_auto.metrics["wireCrossingCount"],
                f"{name}: module_plan path {r_mod.metrics['wireCrossingCount']} > auto {r_auto.metrics['wireCrossingCount']}",
            )

    def test_spacing_camelcase_alias_and_unknown_key_warning(self) -> None:
        sc = SpacingConfig.from_dict({"minClearanceX": 9, "interGroupGapX": 21, "intra_stack_gap_y": 5})
        self.assertEqual((sc.min_clearance_x, sc.inter_group_gap_x, sc.intra_stack_gap_y), (9, 21, 5))
        with warnings.catch_warnings(record=True) as caught:
            warnings.simplefilter("always")
            SpacingConfig.from_dict({"minClearenceX": 3})
        self.assertTrue(any("unknown keys" in str(w.message) for w in caught))
        intent = SchematicIntent.from_dict({"groups": [{"id": "g", "stageOrder": 30, "instances": []}]})
        self.assertEqual(intent.groups[0].stage_order, 30)

    def test_rail_classification_shared_with_parser_rules(self) -> None:
        sup, gnd = classify_rail_nets(["AVDD", "VCCA", "net1", "DVSS", "VSSA", "0", "VDDX"])
        self.assertEqual(sup, ["AVDD", "VCCA"])
        self.assertEqual(gnd, ["DVSS", "VSSA", "0"])

    def test_optimizer_never_moves_declared_skeleton_devices(self) -> None:
        # Small benchmark keeps the bounded search fast; lock a detected quasi-symmetric pair as a mirror skeleton.
        name = "sky130_folded_cascode"
        c = self.circuits[name]
        a = self.analyses[name]
        qp = a.branch_abstraction["quasiSymmetricPairs"][0]
        intent = SchematicIntent.from_dict(a.draft_intent)
        intent.motifs.append(
            {
                "type": "mirror_block_pair",
                "id": "llm_declared_mirror",
                "leftColumns": [list(qp["leftInstances"])],
                "rightColumns": [list(qp["rightInstances"])],
            }
        )
        locked = set(qp["leftInstances"]) | set(qp["rightInstances"])
        before = {r: intent.orientations.get(r) for r in locked}
        solved = optimize_schematic_crossings(c, intent, pdk=self.pdk, allow_wire_labels=True, max_evals=6)
        for mv in solved.optimization_moves:
            refs_in_move = set(re.findall(r"[A-Z]+\d+", mv))
            self.assertFalse(refs_in_move & locked, f"optimizer moved skeleton device: {mv}")
        self.assertEqual({r: solved.intent.orientations.get(r) for r in locked}, before)

    def test_no_demo_vocabulary_in_tool_rules(self) -> None:
        banned = re.compile(
            r"opa316|rcopa|pedestal|overdrive|unit copies|handoff|rail-fraction|shield ref|beta multiplier|replica",
            re.IGNORECASE,
        )
        for py in sorted((ROOT / "schematic_gen").glob("*.py")):
            for ln, line in enumerate(py.read_text(encoding="utf-8").splitlines(), start=1):
                self.assertIsNone(banned.search(line), f"demo vocabulary in {py.name}:{ln}: {line.strip()}")

    def test_pass3_backbone_flag_only_on_degenerate_partitions(self) -> None:
        self.assertNotIn("Pass 3 主干对本电路不适用", self.analyses["dual_path_opamp"].semantic_digest_md)
        self.assertNotIn("Pass 3 主干对本电路不适用", self.analyses["sky130_two_stage_ota"].semantic_digest_md)
        bgr_md = self.analyses["synthetic_bgr"].semantic_digest_md
        bb = self.analyses["synthetic_bgr"].signal_backbone.get("macroStages", [])
        largest = max((len(s["instances"]) for s in bb), default=0)
        n = len(self.circuits["synthetic_bgr"].instances)
        degenerate = bool(bb) and ((largest / n >= 0.8 and len(bb) >= 2) or (len(bb) == 1 and n >= 20))
        self.assertEqual("Pass 3 主干对本电路不适用" in bgr_md, degenerate)

    # ---------------------------------------------------------------- v0.3.3 (P2/P3) guarantees
    def test_layering_rules_no_upward_or_private_imports(self) -> None:
        import ast

        layout_modules = {"placer.py", "router.py", "optimizer.py"}
        forbidden_for_layout = {"analyzer", "auditor", "semantic_validator", "semantic_digest", "branch_abstraction"}
        for py in sorted((ROOT / "schematic_gen").glob("*.py")):
            tree = ast.parse(py.read_text(encoding="utf-8"))
            for node in tree.body:  # module-level imports only (lazy imports inside functions are allowed)
                if isinstance(node, ast.ImportFrom) and (node.module or "").startswith("schematic_gen."):
                    target = node.module.split(".", 1)[1]
                    for alias in node.names:
                        self.assertFalse(
                            alias.name.startswith("_"),
                            f"{py.name} imports private `{alias.name}` from {target}",
                        )
                    if py.name in layout_modules:
                        self.assertNotIn(target, forbidden_for_layout, f"layout module {py.name} imports {target}")

    def test_thresholds_and_device_families_overridable_per_pdk(self) -> None:
        import json
        import tempfile

        from schematic_gen.topology_core import THRESHOLDS, device_family, load_thresholds

        self.assertFalse(THRESHOLDS.allow_naming_layout_rules)
        with tempfile.TemporaryDirectory() as tmp:
            rp = Path(tmp) / "x_netlist_rules.json"
            rp.write_text(
                json.dumps(
                    {
                        "topologyThresholds": {"localFanoutMax": 9, "allowNamingLayoutRules": "true", "quasi_symmetry_min_similarity": 0.8},
                        "deviceFamilies": {"resistor": ["rpoly"], "capacitor": ["mom"]},
                    }
                ),
                encoding="utf-8",
            )
            th = load_thresholds(rp)
            self.assertEqual((th.local_fanout_max, th.allow_naming_layout_rules, th.quasi_symmetry_min_similarity), (9, True, 0.8))
            self.assertEqual(device_family("rpoly_hv", rp), "resistor")
            self.assertEqual(device_family("mom_5x", rp), "capacitor")
        self.assertEqual(device_family("res_xhigh_po_3t"), "resistor")
        self.assertEqual(device_family("cap_mim_m4"), "capacitor")
        # No SKY130 class names hardcoded in tool rules (topology_core docstring is the only mention)
        for py in sorted((ROOT / "schematic_gen").glob("*.py")):
            if py.name == "topology_core.py":
                continue
            self.assertNotRegex(py.read_text(encoding="utf-8"), r"res_xhigh_po_3t|cap_mim_m4", py.name)

    def test_net_policy_priority_optimizer_never_overrides_llm(self) -> None:
        from schematic_gen.topology_core import net_policy_may_override

        self.assertFalse(net_policy_may_override("llm", "optimizer"))
        self.assertFalse(net_policy_may_override("skeleton", "optimizer"))
        self.assertTrue(net_policy_may_override("analyzer", "optimizer"))
        self.assertTrue(net_policy_may_override("optimizer", "llm"))
        name = "sky130_folded_cascode"
        c = self.circuits[name]
        intent = SchematicIntent.from_dict(self.analyses[name].draft_intent)
        rails = set(c.supply_nets) | set(c.ground_nets)
        for net in c.nets:
            if net not in rails:
                intent.net_policies[net] = "full_wire"
                intent.net_policy_sources[net] = "llm"
        intent.critical_nets = []
        solved = optimize_schematic_crossings(c, intent, pdk=self.pdk, max_evals=6)
        self.assertFalse(any(mv.startswith("netPolicy") for mv in solved.optimization_moves), solved.optimization_moves)
        # Hand-edited intent without explicit provenance counts as `llm`
        hand = SchematicIntent.from_dict({**self.analyses[name].draft_intent, "authoredBy": "llm_agent"})
        some_net = next(n for n in hand.net_policies if n not in rails)
        self.assertEqual(hand.net_policy_source(some_net), "llm")

    def test_label_stub_degradation_is_reported(self) -> None:
        name = "sky130_two_stage_ota"
        c = self.circuits[name]
        intent = SchematicIntent.from_dict(self.analyses[name].draft_intent)
        rails = set(c.supply_nets) | set(c.ground_nets)
        net = next(n for n in c.nets if n not in rails and intent.net_policies.get(n) != "supply_local")
        intent.net_policies[net] = "label_stub"
        solved = compile_schematic(c, intent, pdk=self.pdk, allow_wire_labels=False)
        rep = audit_solved_schematic(solved, self.pdk)
        self.assertIn("LABEL_STUB_DEGRADED", {f.code for f in rep.findings})
        self.assertIn(net, rep.metrics["degradedLabelStubNets"])

    def test_tiered_digest_summary_is_bounded(self) -> None:
        from schematic_gen.semantic_digest import render_semantic_digest_md

        a = self.analyses["sky130_rcopa316_core"]
        summary = a.semantic_digest_md
        full_md = render_semantic_digest_md(a, detail="full")
        n_sum, n_full = summary.count("\n"), full_md.count("\n")
        self.assertLess(n_sum, 0.6 * n_full, (n_sum, n_full))
        self.assertLessEqual(n_sum, 220)
        self.assertIn("semantic_digest_full.md", summary)
        for b in a.branch_abstraction["branches"]:
            self.assertIn(b["id"], full_md)
            self.assertIn(b["id"], summary)  # every branch ID stays referencable in the summary
        # Small circuits keep the full detail in the summary tier
        small = self.analyses["synthetic_bgr"].semantic_digest_md
        self.assertNotIn("摘要模式", small)

    def test_naming_only_as_tiebreak_metadata(self) -> None:
        for name in ("dual_path_opamp", "sky130_two_stage_ota", "sky130_folded_cascode"):
            for m in self.analyses[name].motifs:
                if m.motif_type == "diff_pair":
                    self.assertIn(m.roles.get("lr_tiebreak"), {"port_order", "diode_load", "naming"})
        self.assertIn("命名使用声明", self.analyses["dual_path_opamp"].semantic_digest_md)


if __name__ == "__main__":
    unittest.main()
