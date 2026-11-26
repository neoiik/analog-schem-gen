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

    def test_sky130_bjt_npn_and_pnp_parsing_and_polarity(self) -> None:
        netlist = """
        .subckt bjt_leaf VDD VSS VIN VOUT
        XQ1 VOUT VIN VDD sky130_fd_pr__pnp_05v5_W0p68L0p68 m=2
        XQ2 VOUT VIN VSS VSS sky130_fd_pr__npn_05v5_W1p00L1p00 m=1
        .ends bjt_leaf
        """
        circuit = parse_netlist_text(netlist)
        insts = circuit.instance_map()
        self.assertEqual(insts["Q1"].device_class, "bjt")
        self.assertEqual(insts["Q1"].kind, "pnp")
        self.assertEqual(insts["Q1"].polarity, "pnp")
        self.assertEqual(insts["Q1"].terminals, {"C": "VOUT", "BASE": "VIN", "E": "VDD"})
        self.assertEqual(insts["Q2"].device_class, "bjt")
        self.assertEqual(insts["Q2"].kind, "npn4")
        self.assertEqual(insts["Q2"].polarity, "npn")
        self.assertEqual(insts["Q2"].terminals, {"C": "VOUT", "BASE": "VIN", "E": "VSS", "B": "VSS"})

        pdk = PdkRegistry()
        rot0, mir0, _ = semantic_to_transform("gate_left")
        pnp_pins = pdk.source_pin_offsets("pnp", rot0, mir0)
        npn4_pins = pdk.source_pin_offsets("npn4", rot0, mir0)
        # PNP: Emitter on top (y=-3), Collector on bottom (y=+3), Base on left (x=-3)
        self.assertLess(pnp_pins["E"][1], pnp_pins["C"][1])
        self.assertLess(pnp_pins["BASE"][0], 0)
        # NPN4: Collector on top (y=-3), Emitter on bottom (y=+3), Base on left (x=-3), Substrate on right (x=+1)
        self.assertLess(npn4_pins["C"][1], npn4_pins["E"][1])
        self.assertLess(npn4_pins["BASE"][0], 0)
        self.assertGreater(npn4_pins["B"][0], 0)

    def test_behavioral_subckt_with_params_keyword_and_block_symbol_registration(self) -> None:
        netlist = """
        .subckt ideal_amp VDD VSS VIP VIN VOP VON params: gain=1e6 vocm=0
        VCM ncm 0 DC {vocm}
        EPLUS VOP ncm VIP VIN {gain/2}
        EMINUS VON ncm VIN VIP {gain/2}
        .ends ideal_amp

        .subckt ideal_buf VDD VSS VIP VIN VOUT params: gain=1e6
        B1 VOUT 0 V={gain*(V(VIP)-V(VIN))}
        .ends ideal_buf

        .subckt top_blk VDD VSS Vop Von
        XM1 n1 n1 VDD VDD sky130_fd_pr__pfet_01v8 l=0.15 w=1 nf=1 m=1
        X1 VDD VSS n1 VSS Von Vop ideal_amp
        X2 VDD VSS n1 VSS n2 ideal_buf
        R1 n2 VSS 1k
        .ends top_blk
        """
        circuit = parse_netlist_text(netlist)
        self.assertEqual(circuit.cell_name, "top_blk")
        ports = circuit.port_map()
        self.assertEqual(ports["Vop"].direction, "output")
        self.assertEqual(ports["Von"].direction, "output")

        insts = circuit.instance_map()
        self.assertEqual(insts["X1"].device_class, "subckt")
        self.assertEqual(insts["X1"].subckt_cell, "ideal_amp")
        self.assertEqual(
            insts["X1"].terminals,
            {"VDD": "VDD", "VSS": "VSS", "VIP": "n1", "VIN": "VSS", "VOP": "Von", "VON": "Vop"},
        )
        self.assertEqual(insts["X2"].subckt_cell, "ideal_buf")
        self.assertEqual(
            insts["X2"].terminals,
            {"VDD": "VDD", "VSS": "VSS", "VIP": "n1", "VIN": "VSS", "VOUT": "n2"},
        )

        pdk = PdkRegistry()
        registered = pdk.ensure_subckt_symbols(circuit)
        self.assertEqual(registered, ["ideal_amp", "ideal_buf"])
        amp_pins = pdk.source_pin_offsets("subckt:ideal_amp", 0, "none")
        # VDD top (y<0), VSS bottom (y>0), VIP/VIN left (x<0), VOP/VON right (x>0)
        self.assertLess(amp_pins["VDD"][1], 0)
        self.assertGreater(amp_pins["VSS"][1], 0)
        self.assertLess(amp_pins["VIP"][0], 0)
        self.assertLess(amp_pins["VIN"][0], 0)
        self.assertGreater(amp_pins["VOP"][0], 0)
        self.assertGreater(amp_pins["VON"][0], 0)


if __name__ == "__main__":
    unittest.main()
