from __future__ import annotations

import unittest

from schematic_gen.placer import solve_1d_compaction_qp


class TestPlacerAndQpKnownSolutions(unittest.TestCase):
    def test_known_solution_expand_cramped_and_compact_stretched(self) -> None:
        """Exact benchmark from design note Section 三.2:
        Three equal-width (W=4 -> extents=(2,2)) devices at initial x = [0, 5, 20].
        Initial net gaps are (5-2)-(0+2) = 1 grid (too cramped) and (20-2)-(5+2) = 11 grids (too loose).
        With min_gap = 4 and target_gap = 4, solver MUST expand left gap to 4 AND compact right gap to 4!
        """
        nodes = ["A", "B", "C"]
        init_x = {"A": 0.0, "B": 5.0, "C": 20.0}
        extents = {"A": (2, 2), "B": (2, 2), "C": (2, 2)}
        separations = [("A", "B", 4), ("B", "C", 4)]
        springs = [("A", "B", 4, 20.0), ("B", "C", 4, 20.0)]

        solved = solve_1d_compaction_qp(
            node_ids=nodes,
            initial_pos=init_x,
            extents=extents,
            separations=separations,
            target_springs=springs,
            anchor_weight=0.02,
        )
        gap_ab = (solved["B"] - 2) - (solved["A"] + 2)
        gap_bc = (solved["C"] - 2) - (solved["B"] + 2)
        self.assertEqual(gap_ab, 4)
        self.assertEqual(gap_bc, 4)

    def test_exact_integer_differential_symmetry_under_asymmetric_push(self) -> None:
        """Differential pair (M1, M2) around tail M5 must retain exact integer pin symmetry
        `(x_M1 + d_l) + (x_M2 + d_r) == 2 * (x_M5 + d_c)` even when pushed from the left by Bias B0.
        """
        nodes = ["B0", "M1", "M5", "M2"]
        init_x = {"B0": 10.0, "M1": 15.0, "M5": 24.0, "M2": 34.0}
        extents = {"B0": (4, 8), "M1": (5, 4), "M5": (3, 6), "M2": (4, 5)}
        separations = [("B0", "M1", 9), ("M1", "M5", 5), ("M5", "M2", 5), ("M1", "M2", 14)]
        springs = [("B0", "M1", 12, 3.0), ("M1", "M2", 14, 6.0)]
        # Suppose M1.S is at dx=+1, M2.S is at dx=-1, M5.D is at dx=+1
        sym = [("M1", "M5", "M2", 1, 1, -1)]

        solved = solve_1d_compaction_qp(
            node_ids=nodes,
            initial_pos=init_x,
            extents=extents,
            separations=separations,
            target_springs=springs,
            symmetric_triples=sym,
        )
        m1_s = solved["M1"] + 1
        m2_s = solved["M2"] - 1
        m5_d = solved["M5"] + 1

        self.assertEqual(m1_s + m2_s, 2 * m5_d)
        # Verify all separation constraints are strictly satisfied in integers
        for u, v, min_g in separations:
            actual_gap = (solved[v] - extents[v][0]) - (solved[u] + extents[u][1])
            self.assertGreaterEqual(actual_gap, min_g, f"violated gap ({u}, {v})")

    def test_intra_group_compact_and_inter_group_breathing_room(self) -> None:
        nodes = ["G1_A", "G1_B", "G2_A", "G2_B"]
        init_x = {"G1_A": 0.0, "G1_B": 40.0, "G2_A": 45.0, "G2_B": 100.0}
        extents = {n: (3, 3) for n in nodes}
        separations = [("G1_A", "G1_B", 6), ("G1_B", "G2_A", 14), ("G2_A", "G2_B", 6)]
        springs = [
            ("G1_A", "G1_B", 6, 15.0),
            ("G1_B", "G2_A", 16, 15.0),
            ("G2_A", "G2_B", 6, 15.0),
        ]
        solved = solve_1d_compaction_qp(nodes, init_x, extents, separations, springs, anchor_weight=0.01)
        g1_internal = (solved["G1_B"] - 3) - (solved["G1_A"] + 3)
        g2_internal = (solved["G2_B"] - 3) - (solved["G2_A"] + 3)
        between_groups = (solved["G2_A"] - 3) - (solved["G1_B"] + 3)

        self.assertEqual(g1_internal, 6)
        self.assertEqual(g2_internal, 6)
        self.assertGreaterEqual(between_groups, 15)

    def test_known_solution_foam_equidistance_qp(self) -> None:
        """Exact known solution for 1D Foam/Bubble equidistance:
        4 nodes pinned at endpoints y=0 and y=90 with bunched initial positions [0, 10, 80, 90]
        must relax to exact uniform spacing [0, 30, 60, 90].
        """
        nodes = ["N0", "N1", "N2", "N3"]
        solved = solve_1d_compaction_qp(
            node_ids=nodes,
            initial_pos={"N0": 0.0, "N1": 10.0, "N2": 80.0, "N3": 90.0},
            extents={n: (2, 2) for n in nodes},
            separations=[("N0", "N1", 4), ("N1", "N2", 4), ("N2", "N3", 4)],
            target_springs=[],
            equalities=[({"N0": 1.0}, 0.0), ({"N3": 1.0}, 90.0)],
            equidistant_triples=[("N0", "N1", "N2", 10.0), ("N1", "N2", "N3", 10.0)],
            anchor_weight=0.001,
        )
        self.assertEqual([solved[n] for n in nodes], [0, 30, 60, 90])

    def test_known_solution_2d_lattice_tension_field_no_boundary_shrinkage(self) -> None:
        """2D abstract lattice short-range tension field with adaptive shared rest length:
        - Asymmetric columns [0, 12, 39, 70] with min_dx=[12, 27, 31] must relax to exact
          uniform integer pitch [0, 31, 62, 93] with zero boundary shrinkage;
        - Staggered 4x4 Y coordinates with min_dy in [11..13] must align all rows horizontally
          and equalize vertical row pitch to exact [13, 13, 13].
        """
        from schematic_gen.placer import relax_lattice_tension_field

        init_x = [0.0, 12.0, 39.0, 70.0]
        init_y = [
            [50.0, 61.0, 74.0, 85.0],
            [50.0, 63.0, 75.0, 87.0],
            [46.0, 59.0, 72.0, 85.0],
            [46.0, 57.0, 68.0, 78.0],
        ]
        min_dx = [12, 27, 31]
        min_dy = [
            [11, 13, 11],
            [13, 12, 12],
            [13, 13, 13],
            [11, 11, 10],
        ]
        out_x, out_y = relax_lattice_tension_field(
            init_col_x=init_x,
            init_grid_y=init_y,
            min_dx=min_dx,
            min_dy=min_dy,
            relax_x=True,
            relax_y=True,
            anchor_bot_y=87.0,
        )
        self.assertEqual([out_x[i + 1] - out_x[i] for i in range(3)], [31, 31, 31])
        for r in range(4):
            self.assertEqual(len({out_y[c][r] for c in range(4)}), 1, f"Row {r} must align horizontally")
        for c in range(4):
            self.assertEqual([out_y[c][r + 1] - out_y[c][r] for r in range(3)], [13, 13, 13])
            self.assertEqual(out_y[c][3], 87)

    def test_pin_facing_pmos_and_nmos_diff_pair_vertical_order_custom_rails(self) -> None:
        """Pin-facing vertical flow must place PMOS tail ABOVE PMOS diff pair even when
        top supply has a custom non-default name (e.g. VDDH), and NMOS tail BELOW NMOS diff pair
        even with a custom ground name (e.g. GNDA), eliminating U-turn tail routing.
        """
        from schematic_gen.analyzer import analyze_circuit
        from schematic_gen.auditor import audit_solved_schematic
        from schematic_gen.models import SchematicIntent
        from schematic_gen.parser import parse_netlist_text
        from schematic_gen.pdk import PdkRegistry
        from schematic_gen.router import compile_schematic

        pdk = PdkRegistry()

        pmos_diff_spice = """
        .subckt pmos_diff_demo VDDH VDD VSS Vb Vref Vfb OutL OutR
        XM19 net8 Vb VDDH VDD sky130_fd_pr__pfet_01v8 l=0.15 w=1 nf=1 m=1
        XM20 OutL Vref net8 VDD sky130_fd_pr__pfet_01v8 l=0.15 w=1 nf=1 m=1
        XM21 OutR Vfb net8 VDD sky130_fd_pr__pfet_01v8 l=0.15 w=1 nf=1 m=1
        R1 OutL VSS 10k
        R2 OutR VSS 10k
        .ends pmos_diff_demo
        """
        c_p = parse_netlist_text(pmos_diff_spice)
        intent_p = SchematicIntent.from_dict(analyze_circuit(c_p).draft_intent)
        solved_p = compile_schematic(c_p, intent_p, pdk=pdk)
        rep_p = audit_solved_schematic(solved_p, pdk)
        self.assertTrue(rep_p.overall_passed, [f.message for f in rep_p.findings])
        self.assertLess(solved_p.placements["M19"].y, solved_p.placements["M20"].y)
        self.assertEqual(solved_p.placements["M20"].y, solved_p.placements["M21"].y)

        nmos_diff_spice = """
        .subckt nmos_diff_demo VDD GNDA VSS Vb Vinp Vinn OutL OutR
        XM5 tail Vb GNDA VSS sky130_fd_pr__nfet_01v8 l=0.15 w=1 nf=1 m=1
        XM1 OutL Vinp tail VSS sky130_fd_pr__nfet_01v8 l=0.15 w=1 nf=1 m=1
        XM2 OutR Vinn tail VSS sky130_fd_pr__nfet_01v8 l=0.15 w=1 nf=1 m=1
        R1 VDD OutL 10k
        R2 VDD OutR 10k
        .ends nmos_diff_demo
        """
        c_n = parse_netlist_text(nmos_diff_spice)
        intent_n = SchematicIntent.from_dict(analyze_circuit(c_n).draft_intent)
        solved_n = compile_schematic(c_n, intent_n, pdk=pdk)
        rep_n = audit_solved_schematic(solved_n, pdk)
        self.assertTrue(rep_n.overall_passed, [f.message for f in rep_n.findings])
        self.assertGreater(solved_n.placements["M5"].y, solved_n.placements["M1"].y)
        self.assertEqual(solved_n.placements["M1"].y, solved_n.placements["M2"].y)

    def test_nonpolar_passive_180_auto_orient_horizontal(self) -> None:
        """Horizontal 2-pin capacitor/resistor must auto-rotate 180 deg regardless of SPICE
        pin order (`C1 right left` vs `C1 left right`) so each pin faces its connected stage.
        """
        from schematic_gen.analyzer import analyze_circuit
        from schematic_gen.models import SchematicIntent
        from schematic_gen.parser import parse_netlist_text
        from schematic_gen.pdk import PdkRegistry
        from schematic_gen.placer import place_schematic

        pdk = PdkRegistry()
        for cap_line in ("C1 n_right n_left 1p", "C1 n_left n_right 1p"):
            spice = f"""
            .subckt cap_orient_demo VDD VSS Vin Vout
            XM1 n_left Vin VSS VSS sky130_fd_pr__nfet_01v8 l=0.15 w=1 nf=1 m=1
            R1 VDD n_left 10k
            {cap_line}
            XM2 Vout n_right VSS VSS sky130_fd_pr__nfet_01v8 l=0.15 w=1 nf=1 m=1
            R2 VDD n_right 10k
            .ends cap_orient_demo
            """
            circuit = parse_netlist_text(spice)
            intent = SchematicIntent.from_dict(analyze_circuit(circuit).draft_intent)
            intent.column_order_hints = [["R1", "M1"], ["C1"], ["R2", "M2"]]
            intent.orientations["C1"] = "horizontal_l2r"
            pls, _, _, _ = place_schematic(circuit, intent, pdk)
            c_inst = circuit.instance_map()["C1"]
            c_pins = pls["C1"].pin_points
            pin_for_left = "P" if c_inst.terminals["P"] == "n_left" else "M"
            pin_for_right = "M" if pin_for_left == "P" else "P"
            self.assertLess(
                c_pins[pin_for_left][0],
                c_pins[pin_for_right][0],
                f"{cap_line}: pin connected to left stage must be on the left side",
            )

    def test_vertical_series_chain_source_follower_divider_order(self) -> None:
        """NMOS source-follower pass transistor (D=VDD) stacked with resistive divider R2-R3
        must stay ordered top-to-bottom as M35 -> R2 -> R3 -> VSS with zero bypass loops.
        """
        from schematic_gen.analyzer import analyze_circuit
        from schematic_gen.auditor import audit_solved_schematic
        from schematic_gen.models import SchematicIntent
        from schematic_gen.parser import parse_netlist_text
        from schematic_gen.pdk import PdkRegistry
        from schematic_gen.router import compile_schematic

        spice = """
        .subckt ldo_pass_demo VDD VSS VG VOUT betaVOUT
        XM35 VDD VG VOUT VSS sky130_fd_pr__nfet_01v8 l=0.15 w=1 nf=1 m=1
        R2 VOUT betaVOUT 1k
        R3 betaVOUT VSS 1k
        C6 VOUT VSS 1p
        .ends ldo_pass_demo
        """
        pdk = PdkRegistry()
        circuit = parse_netlist_text(spice)
        intent = SchematicIntent.from_dict(analyze_circuit(circuit).draft_intent)
        intent.column_order_hints = [["M35", "R2", "R3"], ["C6"]]
        intent.port_sides["betaVOUT"] = "local_repeat"
        solved = compile_schematic(circuit, intent, pdk=pdk, allow_wire_labels=True)
        rep = audit_solved_schematic(solved, pdk)
        self.assertTrue(rep.overall_passed, [f.message for f in rep.findings])
        self.assertLess(solved.placements["M35"].y, solved.placements["R2"].y)
        self.assertLess(solved.placements["R2"].y, solved.placements["R3"].y)

    def test_mirror_straight_horizontal_gate_bar_with_custom_vdd_and_cap_tap(self) -> None:
        """Top PMOS mirror on custom VDDH (M30-M32) and bottom NMOS mirror (M31-M33) separated
        by a floating C3-C4 column must stay aligned in Y and route straight 0-bend G-G bars.
        """
        from schematic_gen.analyzer import analyze_circuit
        from schematic_gen.auditor import audit_solved_schematic
        from schematic_gen.models import SchematicIntent
        from schematic_gen.parser import parse_netlist_text
        from schematic_gen.pdk import PdkRegistry
        from schematic_gen.router import compile_schematic

        spice = """
        .subckt te_demo VDDH VDD VSS Vmid Vout
        XM30 Vout net5 VDDH VDD sky130_fd_pr__pfet_01v8 l=0.15 w=1 nf=1 m=1
        XM31 Vout net6 VSS VSS sky130_fd_pr__nfet_01v8 l=0.15 w=1 nf=1 m=1
        C3 Vmid net5 1p
        C4 net6 Vmid 1p
        XM32 net5 net5 VDDH VDD sky130_fd_pr__pfet_01v8 l=0.15 w=1 nf=1 m=1
        XM36 net5 net5 net1 VSS sky130_fd_pr__nfet_01v8 l=0.15 w=1 nf=1 m=1
        XM37 net1 net1 net6 VSS sky130_fd_pr__nfet_01v8 l=0.15 w=1 nf=1 m=1
        XM33 net6 net6 VSS VSS sky130_fd_pr__nfet_01v8 l=0.15 w=1 nf=1 m=1
        .ends te_demo
        """
        pdk = PdkRegistry()
        circuit = parse_netlist_text(spice)
        intent = SchematicIntent.from_dict(analyze_circuit(circuit).draft_intent)
        intent.column_order_hints = [["M30", "M31"], ["C3", "C4"], ["M32", "M36", "M37", "M33"]]
        intent.orientations.update({"M30": "gate_right", "M31": "gate_right", "M32": "gate_left", "M33": "gate_left"})
        solved = compile_schematic(circuit, intent, pdk=pdk)
        rep = audit_solved_schematic(solved, pdk)
        self.assertTrue(rep.overall_passed, [f.message for f in rep.findings])
        self.assertEqual(solved.placements["M30"].y, solved.placements["M32"].y)
        self.assertEqual(solved.placements["M31"].y, solved.placements["M33"].y)
        # Verify straight 0-bend horizontal G-G bar covers every grid step from gl to gr at y == gl[1]
        for net_n, r_l, r_r in (("net5", "M30", "M32"), ("net6", "M31", "M33")):
            gl = solved.placements[r_l].pin_points["G"]
            gr = solved.placements[r_r].pin_points["G"]
            self.assertEqual(gl[1], gr[1])
            covered_xs: set[int] = set()
            for rt in solved.routes:
                if rt.net_name != net_n:
                    continue
                pts = rt.points()
                for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
                    if y0 == y1 == gl[1]:
                        covered_xs.update(range(min(x0, x1), max(x0, x1) + 1))
            req_xs = set(range(min(gl[0], gr[0]), max(gl[0], gr[0]) + 1))
            self.assertTrue(
                req_xs <= covered_xs,
                f"{net_n} should connect {r_l}.G and {r_r}.G with a straight horizontal line at y={gl[1]}",
            )

    def test_matrix_array_uniform_pitch_expands_for_dual_flythrough_tracks(self) -> None:
        """When two differential inputs (net12, net13) share one inter-row gap (M27-M26) of a 4x2
        folded-cascode matrix array, the ENTIRE matrix must uniformly relax its vertical pitch
        (equal dY across all 4 rows), and net13 must nest above net12 straight through (M27, M26)
        without diving below VSS or detouring across Vnc/M24-M27.
        """
        from schematic_gen.analyzer import analyze_circuit
        from schematic_gen.auditor import audit_solved_schematic
        from schematic_gen.models import FunctionalGroupSpec, SchematicIntent
        from schematic_gen.parser import parse_netlist_text
        from schematic_gen.pdk import PdkRegistry
        from schematic_gen.router import compile_schematic

        spice = """
        .subckt fc_dualtrack VDDH VDD VSS Vb Vref Vfb Vpc Vnc Vn VG
        XM19 tail Vb VDDH VDD sky130_fd_pr__pfet_01v8 l=0.15 w=1 nf=1 m=1
        XM20 net12 Vref tail VDD sky130_fd_pr__pfet_01v8 l=0.15 w=1 nf=1 m=1
        XM21 net13 Vfb tail VDD sky130_fd_pr__pfet_01v8 l=0.15 w=1 nf=1 m=1
        XM22 p1 net9 VDDH VDD sky130_fd_pr__pfet_01v8 l=0.15 w=1 nf=1 m=1
        XM23 p2 net9 VDDH VDD sky130_fd_pr__pfet_01v8 l=0.15 w=1 nf=1 m=1
        XM24 net9 Vpc p1 VDD sky130_fd_pr__pfet_01v8 l=0.15 w=1 nf=1 m=1
        XM25 VG Vpc p2 VDD sky130_fd_pr__pfet_01v8 l=0.15 w=1 nf=1 m=1
        XM27 net9 Vnc net12 VSS sky130_fd_pr__nfet_01v8 l=0.15 w=1 nf=1 m=1
        XM29 VG Vnc net13 VSS sky130_fd_pr__nfet_01v8 l=0.15 w=1 nf=1 m=1
        XM26 net12 Vn VSS VSS sky130_fd_pr__nfet_01v8 l=0.15 w=1 nf=1 m=1
        XM28 net13 Vn VSS VSS sky130_fd_pr__nfet_01v8 l=0.15 w=1 nf=1 m=1
        .ends fc_dualtrack
        """
        pdk = PdkRegistry()
        circuit = parse_netlist_text(spice)
        intent = SchematicIntent.from_dict(analyze_circuit(circuit).draft_intent)
        intent.groups = [
            FunctionalGroupSpec(id="diff", label="差分级", role="diff_pair", stage_order=10, instances=["M19", "M20", "M21"]),
            FunctionalGroupSpec(
                id="fc",
                label="折叠级",
                role="cascode_load",
                stage_order=20,
                instances=["M22", "M24", "M27", "M26", "M23", "M25", "M29", "M28"],
            ),
        ]
        intent.column_order_hints = [
            ["M20"],
            ["M19"],
            ["M21"],
            ["M22", "M24", "M27", "M26"],
            ["M23", "M25", "M29", "M28"],
        ]
        intent.orientations.update(
            {
                "M20": "gate_left",
                "M21": "gate_right",
                "M22": "gate_right",
                "M24": "gate_right",
                "M27": "gate_right",
                "M26": "gate_right",
                "M23": "gate_left",
                "M25": "gate_left",
                "M29": "gate_left",
                "M28": "gate_left",
            }
        )
        intent.net_policies.update({"VDDH": "perimeter_bus", "Vpc": "label_stub", "Vnc": "label_stub", "Vn": "label_stub"})
        for pn in ("Vb", "Vref", "Vfb", "Vpc", "Vnc", "Vn"):
            intent.port_sides[pn] = "local_repeat"
        intent.port_sides["VG"] = "right"
        solved = compile_schematic(circuit, intent, pdk=pdk, allow_wire_labels=True)
        rep = audit_solved_schematic(solved, pdk)
        self.assertTrue(rep.overall_passed, [f.message for f in rep.findings])

        # 1. Uniform equidistant vertical pitch across ALL 4 rows of BOTH folded-cascode columns
        for col_refs in (["M22", "M24", "M27", "M26"], ["M23", "M25", "M29", "M28"]):
            ys = [solved.placements[r].y for r in col_refs]
            dys = [ys[i + 1] - ys[i] for i in range(len(ys) - 1)]
            self.assertEqual(len(set(dys)), 1, f"Expected uniform dY in {col_refs}, got {dys}")
            self.assertGreaterEqual(dys[0], 15, f"Expected uniformly widened dY>=15 for 2 tracks, got {dys}")

        # 2. Both net13 and net12 must cross horizontally strictly inside the (M27, M26) gap,
        #    with net13 nested ABOVE net12 (y_net13 < y_net12) and 1 single horizontal segment each!
        y_m27_bot = solved.placements["M27"].body_box[3]
        y_m26_top = solved.placements["M26"].body_box[1]
        net13_hys = {
            y0
            for rt in solved.routes
            if rt.net_name == "net13"
            for (x0, y0), (x1, y1) in zip(rt.points(), rt.points()[1:])
            if y0 == y1 and x0 != x1
        }
        net12_hys = {
            y0
            for rt in solved.routes
            if rt.net_name == "net12"
            for (x0, y0), (x1, y1) in zip(rt.points(), rt.points()[1:])
            if y0 == y1 and x0 != x1
        }
        self.assertEqual(len(net13_hys), 1, f"net13 should fly straight across with 1 horizontal Y, got {net13_hys}")
        self.assertEqual(len(net12_hys), 1, f"net12 should fly straight across with 1 horizontal Y, got {net12_hys}")
        y13 = next(iter(net13_hys))
        y12 = next(iter(net12_hys))
        self.assertTrue(y_m27_bot < y13 < y12 < y_m26_top, f"Expected {y_m27_bot} < y13({y13}) < y12({y12}) < {y_m26_top}")

    def test_co_gated_series_stack_pairing_priority_and_row_alignment(self) -> None:
        """Matched 4-device co-gated series MOS stacks (M10-M13 <-> M14-M17) must stay atomic,
        pair adjacently with higher priority than single-device distractor branches, orient
        face-to-face, and align layer-by-layer to identical integer Y coordinates.
        """
        from schematic_gen.analyzer import analyze_circuit, detect_co_gated_mos_stacks
        from schematic_gen.auditor import audit_solved_schematic
        from schematic_gen.models import SchematicIntent
        from schematic_gen.parser import parse_netlist_text
        from schematic_gen.pdk import PdkRegistry
        from schematic_gen.router import compile_schematic

        spice = """
        .subckt cogated_stack_demo VDD VSS OUT1 OUT2 OUT3 OUT4
        I1 VDD net11 DC 1m
        XM10 net11 net11 net30 VSS sky130_fd_pr__nfet_01v8 l=0.15 w=1 nf=1 m=1
        XM11 net30 net11 net33 VSS sky130_fd_pr__nfet_01v8 l=0.15 w=1 nf=1 m=1
        XM12 net33 net11 net36 VSS sky130_fd_pr__nfet_01v8 l=0.15 w=1 nf=1 m=1
        XM13 net36 net11 VSS VSS sky130_fd_pr__nfet_01v8 l=0.15 w=1 nf=1 m=1
        XM19 OUT1 net11 VSS VSS sky130_fd_pr__nfet_01v8 l=0.15 w=1 nf=1 m=1
        R1 VDD OUT1 10k
        XM1 net16 net2 VDD VDD sky130_fd_pr__pfet_01v8 l=0.15 w=1 nf=1 m=1
        XM2 net25 net2 net16 VDD sky130_fd_pr__pfet_01v8 l=0.15 w=1 nf=1 m=1
        XM3 net29 net2 net25 VDD sky130_fd_pr__pfet_01v8 l=0.15 w=1 nf=1 m=1
        XM4 net2 net2 net29 VDD sky130_fd_pr__pfet_01v8 l=0.15 w=1 nf=1 m=1
        XM14 net2 net11 net32 VSS sky130_fd_pr__nfet_01v8 l=0.15 w=1 nf=1 m=1
        XM15 net32 net11 net35 VSS sky130_fd_pr__nfet_01v8 l=0.15 w=1 nf=1 m=1
        XM16 net35 net11 net38 VSS sky130_fd_pr__nfet_01v8 l=0.15 w=1 nf=1 m=1
        XM17 net38 net11 VSS VSS sky130_fd_pr__nfet_01v8 l=0.15 w=1 nf=1 m=1
        XM5 OUT3 net2 VDD VDD sky130_fd_pr__pfet_01v8 l=0.15 w=1 nf=1 m=1
        R3 OUT3 VSS 10k
        XM34 net24 net2 VDD VDD sky130_fd_pr__pfet_01v8 l=0.15 w=1 nf=1 m=1
        XM35 net27 net2 net24 VDD sky130_fd_pr__pfet_01v8 l=0.15 w=1 nf=1 m=1
        XM36 net28 net2 net27 VDD sky130_fd_pr__pfet_01v8 l=0.15 w=1 nf=1 m=1
        XM37 OUT4 net2 net28 VDD sky130_fd_pr__pfet_01v8 l=0.15 w=1 nf=1 m=1
        R4 OUT4 VSS 10k
        XM6 net23 net2 VDD VDD sky130_fd_pr__pfet_01v8 l=0.15 w=1 nf=1 m=1
        XM7 net6 net2 net23 VDD sky130_fd_pr__pfet_01v8 l=0.15 w=1 nf=1 m=1
        XM8 net0 net2 net6 VDD sky130_fd_pr__pfet_01v8 l=0.15 w=1 nf=1 m=1
        XM9 OUT2 net2 net0 VDD sky130_fd_pr__pfet_01v8 l=0.15 w=1 nf=1 m=1
        R2 OUT2 VSS 10k
        .ends cogated_stack_demo
        """
        pdk = PdkRegistry()
        circuit = parse_netlist_text(spice)

        # 1. All co-gated stacks detected intact in top-to-bottom order
        stacks = {tuple(s["refs"]): s for s in detect_co_gated_mos_stacks(circuit)}
        self.assertIn(("M10", "M11", "M12", "M13"), stacks)
        self.assertIn(("M14", "M15", "M16", "M17"), stacks)
        self.assertIn(("M1", "M2", "M3", "M4"), stacks)
        self.assertIn(("M6", "M7", "M8", "M9"), stacks)
        self.assertIn(("M34", "M35", "M36", "M37"), stacks)
        self.assertTrue(stacks[("M10", "M11", "M12", "M13")]["is_diode_master"])
        self.assertTrue(stacks[("M1", "M2", "M3", "M4")]["is_diode_master"])

        # 2. Topology first, naming only as tie-break: M10-13 adjacent to M14-17; among the topologically
        #    identical slave stacks M6-9 / M34-37 the reference-number tie-break pairs M6-9 next to M1-4.
        #    The co-gated singleton slave M5 sits on the slave side (right) of its diode master M1-4; the old
        #    naming-order rule "M5 between M1-4 and M6-9 because 4 < 5 < 6" is disabled by default (v0.3.3).
        analysis = analyze_circuit(circuit)
        intent = SchematicIntent.from_dict(analysis.draft_intent)
        solved = compile_schematic(circuit, intent, pdk=pdk)
        rep = audit_solved_schematic(solved, pdk)
        self.assertTrue(rep.overall_passed, [f.message for f in rep.findings])

        x_m10 = solved.placements["M10"].x
        x_m14 = solved.placements["M14"].x
        x_m1 = solved.placements["M1"].x
        x_m5 = solved.placements["M5"].x
        x_m6 = solved.placements["M6"].x
        x_m34 = solved.placements["M34"].x
        self.assertGreater(x_m5, x_m1, f"Slave M5 must sit right of diode master M1-4, got x(M1)={x_m1}, x(M5)={x_m5}")
        self.assertLess(abs(x_m6 - x_m1), abs(x_m34 - x_m1), "Tie-break: M1-4 pairs closer to M6-9 than M34-37")

        # 3. Exact layer-by-layer horizontal row alignment for BOTH 4-stack NMOS and 4-stack PMOS pairs!
        for u, v in (("M10", "M14"), ("M11", "M15"), ("M12", "M16"), ("M13", "M17")):
            self.assertEqual(solved.placements[u].y, solved.placements[v].y, f"{u} and {v} must share exact Y")
        for u, v in (("M1", "M6"), ("M2", "M7"), ("M3", "M8"), ("M4", "M9")):
            self.assertEqual(solved.placements[u].y, solved.placements[v].y, f"{u} and {v} must share exact Y")

        # 4. Single horizontal cross-bridge between paired co-gated stacks (M10-13 <-> M14-17):
        #    NOT 4 parallel horizontal rungs — only 1 horizontal bridge row crosses the corridor!
        gx_l = solved.placements["M10"].pin_points["G"][0]
        gx_r = solved.placements["M14"].pin_points["G"][0]
        lo_x, hi_x = min(gx_l, gx_r), max(gx_l, gx_r)
        mid_x = (lo_x + hi_x) / 2.0
        bridge_ys = {
            y0
            for rt in solved.routes
            if rt.net_name == "net11"
            for (x0, y0), (x1, y1) in zip(rt.points(), rt.points()[1:])
            if y0 == y1 and min(x0, x1) < mid_x < max(x0, x1)
        }
        self.assertEqual(
            len(bridge_ys),
            1,
            f"Paired co-gated stacks M10-13 <-> M14-17 should use 1 single horizontal bridge, got rows {bridge_ys}",
        )

    def test_wl_mirror_subcircuit_pair_auto_detection_and_symmetry(self) -> None:
        """Pure netlist WL coloring + dual-BFS must automatically detect the 7x2 mirrored
        folded emitter-follower error amplifier ({Q3,R3,M21,M22,M25,Q4,R4} <-> {Q5,R5,M23,M24,M26,Q6,R6}),
        fold columns symmetrically around a central axis with complementary orientations,
        and enforce exact integer pair Y-alignment & equal outer-inner X pitch.
        """
        from schematic_gen.analyzer import analyze_circuit, detect_mirror_subcircuit_pairs
        from schematic_gen.auditor import audit_solved_schematic
        from schematic_gen.models import SchematicIntent
        from schematic_gen.parser import parse_netlist_text
        from schematic_gen.pdk import PdkRegistry
        from schematic_gen.router import compile_schematic

        spice = """
        .subckt mirror_amp_demo VDD VSS C B net3 A net2 net11
        XM25 net3 net2 VDD VDD sky130_fd_pr__pfet_01v8 l=0.15 w=1 nf=1 m=1
        XM26 A net2 VDD VDD sky130_fd_pr__pfet_01v8 l=0.15 w=1 nf=1 m=1
        XQ3 VDD C net7 VSS sky130_fd_pr__npn_05v5_W1p00L1p00 m=1
        XQ4 net21 net7 net3 sky130_fd_pr__pnp_05v5_W0p68L0p68 m=1
        XQ5 VDD B net8 VSS sky130_fd_pr__npn_05v5_W1p00L1p00 m=1
        XQ6 net22 net8 A sky130_fd_pr__pnp_05v5_W0p68L0p68 m=1
        R3 net7 net19 1k
        R4 net21 VSS 1k
        R5 net8 net20 1k
        R6 net22 VSS 1k
        XM21 net19 net11 net13 VSS sky130_fd_pr__nfet_01v8 l=0.15 w=1 nf=1 m=1
        XM22 net13 net11 VSS VSS sky130_fd_pr__nfet_01v8 l=0.15 w=1 nf=1 m=1
        XM23 net20 net11 net14 VSS sky130_fd_pr__nfet_01v8 l=0.15 w=1 nf=1 m=1
        XM24 net14 net11 VSS VSS sky130_fd_pr__nfet_01v8 l=0.15 w=1 nf=1 m=1
        .ends mirror_amp_demo
        """
        pdk = PdkRegistry()
        circuit = parse_netlist_text(spice)

        # 1. Pure topology detection without naming assumptions
        mp_list = detect_mirror_subcircuit_pairs(circuit)
        self.assertEqual(len(mp_list), 1)
        pairs_map = dict(mp_list[0]["pairs"])
        expected_pairs = {
            "Q3": "Q5",
            "Q4": "Q6",
            "R3": "R5",
            "R4": "R6",
            "M21": "M23",
            "M22": "M24",
            "M25": "M26",
        }
        self.assertEqual(pairs_map, expected_pairs)

        # 2. Zero-shot analyze + compile produces exact mirrored schematic passing all DRC/LVS gates
        analysis = analyze_circuit(circuit)
        intent = SchematicIntent.from_dict(analysis.draft_intent)
        solved = compile_schematic(circuit, intent, pdk=pdk)
        rep = audit_solved_schematic(solved, pdk)
        self.assertTrue(rep.overall_passed, [f.message for f in rep.findings])

        # 3. Columns folded symmetrically: Q3(outer-L) < Q4(inner-L) < Q6(inner-R) < Q5(outer-R)
        pls = solved.placements
        self.assertTrue(
            pls["Q3"].x < pls["Q4"].x < pls["Q6"].x < pls["Q5"].x,
            f"Expected symmetric column fold Q3 < Q4 < Q6 < Q5, got {[pls[r].x for r in ('Q3','Q4','Q6','Q5')]}",
        )
        # Exact complementary mirror orientations
        self.assertEqual(pls["Q3"].semantic_orient, "gate_left")
        self.assertEqual(pls["Q5"].semantic_orient, "gate_right")
        self.assertEqual(pls["Q4"].semantic_orient, "gate_left")
        self.assertEqual(pls["Q6"].semantic_orient, "gate_right")
        self.assertEqual(pls["M21"].semantic_orient, "gate_left")
        self.assertEqual(pls["M23"].semantic_orient, "gate_right")
        self.assertEqual(pls["M25"].semantic_orient, "gate_right")
        self.assertEqual(pls["M26"].semantic_orient, "gate_left")

        # 4. Exact integer Y alignment for EVERY mirrored pair & exact equal X spacing on left vs right
        for u, v in expected_pairs.items():
            self.assertEqual(pls[u].y, pls[v].y, f"Mirrored pair ({u}, {v}) must share exact Y")
        span_left = pls["Q4"].pin_points["C"][0] - pls["Q3"].pin_points["C"][0]
        span_right = pls["Q5"].pin_points["C"][0] - pls["Q6"].pin_points["C"][0]
        self.assertEqual(span_left, span_right, f"Expected equal outer-inner X pitch, got {span_left} vs {span_right}")

    def test_large_clustered_qp_converges_without_fallback(self) -> None:
        """64-node QP with 56 rigid 2-var column equalities and 448 redundant pairwise separations
        must algebraically eliminate clusters and converge cleanly to the exact compacted solution
        (never failing SLSQP with status=8 and falling back to stretched x0)."""
        n_cols, n_rows = 8, 8
        nodes = [f"C{c}_R{r}" for c in range(n_cols) for r in range(n_rows)]
        extents = {u: (4, 4) for u in nodes}
        # Deliberately stretched initial guess (pitch=40 instead of ideal 8+8=16)
        init_pos = {f"C{c}_R{r}": float(20 + c * 40) for c in range(n_cols) for r in range(n_rows)}
        equalities = [
            ({f"C{c}_R{r}": 1.0, f"C{c}_R0": -1.0}, 0.0)
            for c in range(n_cols)
            for r in range(1, n_rows)
        ]
        # 7 * 64 = 448 redundant pairwise separations between adjacent columns
        separations = [
            (f"C{c}_R{ra}", f"C{c + 1}_R{rb}", 4)
            for c in range(n_cols - 1)
            for ra in range(n_rows)
            for rb in range(n_rows)
        ]
        target_springs = [(f"C{c}_R0", f"C{c + 1}_R0", 8, 6.0) for c in range(n_cols - 1)]
        foam = [(f"C{c}_R0", f"C{c + 1}_R0", f"C{c + 2}_R0", 6.0) for c in range(n_cols - 2)]

        sol = solve_1d_compaction_qp(
            node_ids=nodes,
            initial_pos=init_pos,
            extents=extents,
            separations=separations,
            target_springs=target_springs,
            equalities=equalities,
            equidistant_triples=foam,
        )
        col_xs = [sol[f"C{c}_R0"] for c in range(n_cols)]
        gaps = [col_xs[c + 1] - col_xs[c] - 8 for c in range(n_cols - 1)]
        # Must compact tightly near ideal gap=8 (never remain near stretched x0 gap=32)
        self.assertTrue(all(7 <= g <= 10 for g in gaps), f"Expected compacted gaps near 8, got {gaps}")
        # Rigid column equalities must be exact
        for c in range(n_cols):
            for r in range(1, n_rows):
                self.assertEqual(sol[f"C{c}_R{r}"], sol[f"C{c}_R0"])

    def test_grounded_base_pnp_horizontal_rail_pin_no_hang_and_passes_drc(self) -> None:
        """Horizontal side pins tied to a rail (e.g. grounded-base PNP C=BASE=VSS or tied MOS gate)
        must never hang `_vert_clear_to_rail` and must route cleanly with 0 DRC/LVS violations."""
        from schematic_gen.analyzer import analyze_circuit
        from schematic_gen.auditor import audit_solved_schematic
        from schematic_gen.models import SchematicIntent
        from schematic_gen.parser import parse_netlist_text
        from schematic_gen.pdk import PdkRegistry
        from schematic_gen.router import compile_schematic

        spice = """
        .subckt pnp_tie_check VDD VSS VOUT
        XM1 VOUT VOUT VDD VDD sky130_fd_pr__pfet_01v8 l=0.15 w=1 nf=1 m=1
        R1 VOUT n_e 1k
        XQ1 VSS VSS n_e sky130_fd_pr__pnp_05v5_W0p68L0p68
        XM2 VOUT VDD VSS VSS sky130_fd_pr__nfet_01v8 l=0.15 w=1 nf=1 m=1
        .ends pnp_tie_check
        """
        pdk = PdkRegistry()
        circuit = parse_netlist_text(spice)
        analysis = analyze_circuit(circuit)
        intent = SchematicIntent.from_dict(analysis.draft_intent)
        solved = compile_schematic(circuit, intent, pdk=pdk)
        rep = audit_solved_schematic(solved, pdk)
        self.assertTrue(rep.overall_passed, [f.message for f in rep.findings])


if __name__ == "__main__":
    unittest.main()
