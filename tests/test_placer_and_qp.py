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


if __name__ == "__main__":
    unittest.main()
