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


if __name__ == "__main__":
    unittest.main()
