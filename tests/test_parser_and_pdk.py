from __future__ import annotations

import unittest

from schematic_gen.parser import parse_netlist_text
from schematic_gen.pdk import PdkRegistry, semantic_to_transform


class TestParserAndPdk(unittest.TestCase):
    def test_sky130_spice_primitives_and_pin_orders(self) -> None:
        netlist = """
        * SKY130 primitive check
        *.PININFO VIN:I VOUT:O VDD:B VSS:B
        .subckt test_cell VIN VOUT VDD VSS
        XM1 VOUT VIN VSS VSS sky130_fd_pr__nfet_01v8 W=4 L=0.15 nf=2 mult=1
        XM2 VOUT VIN VDD VDD sky130_fd_pr__pfet_01v8_hvt
        + W=8 L=0.15 nf=2 mult=1
        XR1 VOUT n_mid VSS sky130_fd_pr__res_high_po_0p35 W=0.35 L=10 mult=1
        XC1 n_mid VSS sky130_fd_pr__cap_mim_m3_1 W=5 L=5 MF=2
        .ends test_cell
        """
        circuit = parse_netlist_text(netlist)
        self.assertEqual(circuit.cell_name, "test_cell")
        self.assertEqual(len(circuit.instances), 4)
        insts = circuit.instance_map()

        self.assertEqual(insts["M1"].kind, "nmos")
        self.assertEqual(insts["M1"].terminals, {"D": "VOUT", "G": "VIN", "S": "VSS", "B": "VSS"})
        self.assertEqual(insts["M1"].params["w"], "4u")
        self.assertEqual(insts["M1"].params["l"], "0.15u")
        self.assertEqual(insts["M1"].params["nf"], "2")

        self.assertEqual(insts["M2"].kind, "pmos_hvt")
        self.assertEqual(insts["R1"].device_class, "res_poly_3t")
        self.assertEqual(insts["R1"].terminals, {"P": "VOUT", "M": "n_mid", "B": "VSS"})
        self.assertEqual(insts["C1"].device_class, "cap_mim")
        self.assertEqual(insts["C1"].terminals, {"P": "n_mid", "M": "VSS"})

    def test_spectre_leaf_syntax_supported(self) -> None:
        spectre_txt = """
        subckt inv_leaf IN OUT VDD VSS
        M0 (OUT IN VSS VSS) nfet_01v8 w=2u l=0.15u nf=1 m=1
        M1 (OUT IN VDD VDD) pfet_01v8 w=4u l=0.15u nf=1 m=1
        ends inv_leaf
        """
        circuit = parse_netlist_text(spectre_txt)
        self.assertEqual(circuit.cell_name, "inv_leaf")
        self.assertEqual(len(circuit.instances), 2)
        self.assertEqual(circuit.instance_map()["M0"].polarity, "nmos")
        self.assertEqual(circuit.instance_map()["M1"].polarity, "pmos")

    def test_pdk_pmos_vs_nmos_vertical_polarity_and_horizontal_bridge(self) -> None:
        pdk = PdkRegistry()
        rot0, mir0, _ = semantic_to_transform("gate_left")
        nmos_pins = pdk.source_pin_offsets("nmos", rot0, mir0)
        pmos_pins = pdk.source_pin_offsets("pmos", rot0, mir0)

        # Source y-down: smaller y is HIGHER on schematic!
        # NMOS R0: Drain on top (y=-3), Source on bottom (y=+3)
        self.assertLess(nmos_pins["D"][1], nmos_pins["S"][1])
        self.assertLess(nmos_pins["G"][0], 0)

        # PMOS R0: Source on top toward VDD (y=-3), Drain on bottom toward signal (y=+3)
        self.assertLess(pmos_pins["S"][1], pmos_pins["D"][1])
        self.assertLess(pmos_pins["G"][0], 0)

        # gate_right (MY) mirrors Gate to +x while keeping vertical S/D order intact
        rot_my, mir_my, cad_my = semantic_to_transform("gate_right")
        self.assertEqual(cad_my, "MY")
        nmos_my = pdk.source_pin_offsets("nmos", rot_my, mir_my)
        self.assertGreater(nmos_my["G"][0], 0)
        self.assertEqual(nmos_my["D"][1], nmos_pins["D"][1])

        # Horizontal passive bridge (`horizontal_l2r` -> P on left, M on right)
        rot_h, mir_h, cad_h = semantic_to_transform("horizontal_l2r")
        self.assertEqual(cad_h, "R90")
        cap_h = pdk.source_pin_offsets("cap_mim", rot_h, mir_h)
        self.assertEqual(cap_h["P"], (-3, 0))
        self.assertEqual(cap_h["M"], (3, 0))


if __name__ == "__main__":
    unittest.main()
