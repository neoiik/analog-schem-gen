from __future__ import annotations

import unittest

from schematic_gen.router import nudge_parallel_corridor_tracks


class TestRouterAndCorridorNudging(unittest.TestCase):
    def test_known_solution_four_wires_corridor_nudging(self) -> None:
        """Exact numerical benchmark from design note Section 四.2:
        Four horizontal segments of distinct nets A, B, C, D at initial y = [100, 103, 105, 130]
        in corridor [100, 130] with min_pitch = 10 -> must solve to [100, 110, 120, 130]!
        """
        segs = [
            {"id": "segA", "net": "A", "span_start": 10, "span_end": 80, "pos": 100},
            {"id": "segB", "net": "B", "span_start": 15, "span_end": 75, "pos": 103},
            {"id": "segC", "net": "C", "span_start": 20, "span_end": 85, "pos": 105},
            {"id": "segD", "net": "D", "span_start": 10, "span_end": 90, "pos": 130},
        ]
        res = nudge_parallel_corridor_tracks(
            segs, corridor_low=100.0, corridor_high=130.0, min_pitch=10.0, snap_step=1.0
        )
        self.assertTrue(res["feasible"])
        self.assertEqual(res["overflow"], 0.0)
        self.assertAlmostEqual(res["tracks"]["segA"], 100.0, places=4)
        self.assertAlmostEqual(res["tracks"]["segB"], 110.0, places=4)
        self.assertAlmostEqual(res["tracks"]["segC"], 120.0, places=4)
        self.assertAlmostEqual(res["tracks"]["segD"], 130.0, places=4)

    def test_same_net_shares_trunk_while_foreign_net_splits_track(self) -> None:
        """Verify '同网共干、异网分轨': two overlapping segments on `vbias` share one track,
        while foreign net `vout1` is pushed `min_pitch` away.
        """
        segs = [
            {"id": "s1", "net": "vbias", "span_start": 0, "span_end": 50, "pos": 20},
            {"id": "s2", "net": "vbias", "span_start": 30, "span_end": 90, "pos": 21},
            {"id": "s3", "net": "vout1", "span_start": 25, "span_end": 70, "pos": 21},
        ]
        res = nudge_parallel_corridor_tracks(
            segs, corridor_low=18.0, corridor_high=30.0, min_pitch=4.0, snap_step=1.0
        )
        self.assertTrue(res["feasible"])
        self.assertEqual(res["tracks"]["s1"], res["tracks"]["s2"])
        self.assertGreaterEqual(abs(res["tracks"]["s3"] - res["tracks"]["s1"]), 4.0)

    def test_corridor_overflow_reported_when_channel_too_narrow(self) -> None:
        """Five overlapping foreign wires with pitch=10 need span 40; inside [100, 130] (span 30),
        overflow must equal 10.0 so layout congestion loop widens the device gap.
        """
        segs = [
            {"id": f"w{i}", "net": f"N{i}", "span_start": 0, "span_end": 50, "pos": 100 + i * 2}
            for i in range(5)
        ]
        res = nudge_parallel_corridor_tracks(
            segs, corridor_low=100.0, corridor_high=130.0, min_pitch=10.0
        )
        self.assertFalse(res["feasible"])
        self.assertAlmostEqual(res["overflow"], 10.0, places=4)

    def test_flat_top_perimeter_bus_and_straight_output_spine_no_upit(self) -> None:
        """Multi-drop top perimeter bus (VDDH) must route on ONE single flat horizontal rail
        with zero stair-steps, and output net (VOUT) over shunt loads (C6, I1) must stay on
        ONE straight horizontal line from driver to output port without dipping into a U-pit.
        """
        from schematic_gen.analyzer import analyze_circuit
        from schematic_gen.auditor import audit_solved_schematic
        from schematic_gen.models import SchematicIntent
        from schematic_gen.parser import parse_netlist_text
        from schematic_gen.pdk import PdkRegistry
        from schematic_gen.router import compile_schematic

        spice = """
        .subckt spine_demo VDDH VDD VSS Vb VG VOUT betaVOUT
        XM1 n1 Vb VDDH VDD sky130_fd_pr__pfet_01v8 l=0.15 w=1 nf=1 m=1
        R1 n1 VSS 10k
        XM2 n2 Vb VDDH VDD sky130_fd_pr__pfet_01v8 l=0.15 w=1 nf=1 m=1
        XM3 n3 Vb VDDH VDD sky130_fd_pr__pfet_01v8 l=0.15 w=1 nf=1 m=1
        R2_top n2 n3 10k
        XM35 VDD VG VOUT VSS sky130_fd_pr__nfet_01v8 l=0.15 w=1 nf=1 m=1
        R2 VOUT betaVOUT 1k
        R3 betaVOUT VSS 1k
        C6 VOUT VSS 1p
        I1 VOUT VSS DC 1m
        .ends spine_demo
        """
        pdk = PdkRegistry()
        circuit = parse_netlist_text(spice)
        intent = SchematicIntent.from_dict(analyze_circuit(circuit).draft_intent)
        for g in intent.groups:
            g.instances = [r for r in g.instances if r not in {"M35", "R2", "R3", "C6", "I1"}]
        from schematic_gen.models import FunctionalGroupSpec
        intent.groups = [g for g in intent.groups if g.instances] + [
            FunctionalGroupSpec(id="out_stage", label="输出级", role="output_stage", stage_order=40, instances=["M35", "R2", "R3", "C6", "I1"])
        ]
        intent.column_order_hints = [["M1", "R1"], ["M2"], ["M3", "R2_top"], ["M35", "R2", "R3"], ["C6"], ["I1"]]
        intent.net_policies["VDDH"] = "perimeter_bus"
        intent.port_sides["VOUT"] = "right"
        intent.port_sides["betaVOUT"] = "local_repeat"
        solved = compile_schematic(circuit, intent, pdk=pdk, allow_wire_labels=True)
        rep = audit_solved_schematic(solved, pdk)
        self.assertTrue(rep.overall_passed, [f.message for f in rep.findings])

        # 1. All horizontal segments on VDDH must share ONE exact Y coordinate (zero stair-steps)
        vddh_horiz_ys = {
            y0
            for rt in solved.routes
            if rt.net_name == "VDDH"
            for (x0, y0), (x1, y1) in zip(rt.points(), rt.points()[1:])
            if y0 == y1 and x0 != x1
        }
        self.assertEqual(len(vddh_horiz_ys), 1, f"VDDH top bus should be 1 flat horizontal line, got ys={vddh_horiz_ys}")

        # 2. All horizontal segments on VOUT must share ONE exact Y coordinate (zero U-pit dip to C6/I1)
        vout_horiz_ys = {
            y0
            for rt in solved.routes
            if rt.net_name == "VOUT"
            for (x0, y0), (x1, y1) in zip(rt.points(), rt.points()[1:])
            if y0 == y1 and x0 != x1
        }
        self.assertEqual(len(vout_horiz_ys), 1, f"VOUT output spine should be 1 straight horizontal line, got ys={vout_horiz_ys}")

    def test_blocked_horizontal_interior_rail_port_never_hijacks_distant_block(self) -> None:
        """An interior local VDD symbol placed behind intervening device columns must not
        hijack a distant block's unobstructed VDD pin across the schematic."""
        from schematic_gen.analyzer import analyze_circuit
        from schematic_gen.auditor import audit_solved_schematic
        from schematic_gen.models import SchematicIntent
        from schematic_gen.parser import parse_netlist_text
        from schematic_gen.pdk import PdkRegistry
        from schematic_gen.router import compile_schematic

        spice = """
        .subckt blk_a VDD VSS VIP VIN VOUT params: gain=1e6
        B1 VOUT 0 V={gain*(V(VIP)-V(VIN))}
        .ends blk_a

        .subckt rail_match_demo VDD VSS VOUT
        XM1 n1 n1 VDD VDD sky130_fd_pr__pfet_01v8 l=0.15 w=1 nf=1 m=1
        XM2 n1 VSS VSS VSS sky130_fd_pr__nfet_01v8 l=0.15 w=1 nf=1 m=1
        X1 VDD VSS n1 VSS n2 blk_a
        XM3 n3 n2 n2 VDD sky130_fd_pr__pfet_01v8 l=0.15 w=1 nf=1 m=1
        XM4 n4 n3 n3 VDD sky130_fd_pr__pfet_01v8 l=0.15 w=1 nf=1 m=1
        XM5 n5 n4 VSS VSS sky130_fd_pr__nfet_01v8 l=0.15 w=1 nf=1 m=1
        R1 n2 VOUT 1k
        X2 VDD VSS n5 VSS VOUT blk_a
        .ends rail_match_demo
        """
        pdk = PdkRegistry()
        circuit = parse_netlist_text(spice)
        pdk.ensure_subckt_symbols(circuit)
        intent = SchematicIntent.from_dict(analyze_circuit(circuit).draft_intent)
        intent.column_order_hints = [
            ["M1", "M2"],
            ["X1"],
            ["M3", "M4", "M5"],
            ["R1", "X2"],
        ]
        intent.orientations["R1"] = "horizontal_l2r"
        intent.tier_overrides.update({"M1": 1, "M2": 5, "X1": 5, "M3": 1, "M4": 2, "M5": 5, "R1": 2, "X2": 3})
        solved = compile_schematic(circuit, intent, pdk=pdk)
        rep = audit_solved_schematic(solved, pdk)
        self.assertTrue(rep.overall_passed, [f.message for f in rep.findings])
        # X1.VDD must NOT route horizontally across M3/M4/M5 to reach X2's local interior VDD!
        x_mid = solved.placements["M4"].x
        vdd_crosses_mid = any(
            min(a[0], b[0]) < x_mid < max(a[0], b[0]) and a[1] == b[1] and a[1] > solved.placements["M3"].body_box[1]
            for rt in solved.routes
            if rt.net_name == "VDD"
            for a, b in zip(rt.points(), rt.points()[1:])
        )
        self.assertFalse(vdd_crosses_mid, "Interior VDD symbol must not hijack distant X1.VDD across middle columns")


if __name__ == "__main__":
    unittest.main()
