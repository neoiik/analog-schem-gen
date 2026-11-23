"""Zero-dependency single-file HTML interactive schematic nudger/drag editor (`export-editor`)."""

from __future__ import annotations

from html import escape
import json

from schematic_gen.manifest_builder import build_bridge_manifest
from schematic_gen.models import SolvedSchematic
from schematic_gen.pdk import PdkRegistry


def render_interactive_editor_html(
    solved: SolvedSchematic,
    pdk: PdkRegistry | None = None,
) -> str:
    """Export a standalone browser canvas allowing human drag/flip micro-tuning -> `refine` round-trip."""
    pdk = pdk or PdkRegistry()
    manifest = build_bridge_manifest(solved, pdk)
    manifest_json = json.dumps(manifest, ensure_ascii=False)
    cell_title = escape(solved.circuit.cell_name)

    return f"""<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<title>{cell_title} · Interactive Schematic Refiner</title>
<style>
  body {{ margin:0; font:14px/1.5 "Segoe UI","Microsoft YaHei",sans-serif; background:#eef3f1; color:#162d31; display:flex; flex-direction:column; height:100vh; }}
  header {{ padding:10px 20px; background:#162d31; color:#fff; display:flex; align-items:center; justify-content:space-between; }}
  header .hint {{ font-size:12px; color:#b8d0cd; }}
  button {{ background:#00897b; color:#fff; border:none; padding:7px 14px; border-radius:4px; cursor:pointer; font-weight:600; }}
  button:hover {{ background:#00695c; }}
  #canvas-wrap {{ flex:1; overflow:auto; padding:16px; }}
  svg {{ background:#fcfdfb; border:1px solid #c8d8d4; border-radius:6px; user-select:none; }}
  .inst-box {{ fill:#e8f4f1; stroke:#00695c; stroke-width:1.6; cursor:grab; }}
  .inst-box.selected {{ stroke:#d84315; stroke-width:2.4; fill:#fbe9e7; }}
  .rubber {{ stroke:#546e7a; stroke-width:1.3; stroke-dasharray:4,3; fill:none; pointer-events:none; }}
</style>
</head>
<body>
<header>
  <div>
    <strong>{cell_title}</strong> · 交互微调画布
    <span class="hint">（鼠标拖拽器件调整行列相对位置 · 选中按 M 水平镜像翻转 · 导出 JSON 后执行 <code>python schem_gen.py refine draft.json --out refined</code> 自动重算净空与正交分轨实线）</span>
  </div>
  <div>
    <button id="btn-export">导出微调清单 (draft_manifest.json)</button>
  </div>
</header>
<div id="canvas-wrap"><svg id="stage" width="1600" height="920"></svg></div>
<script>
const MANIFEST = {manifest_json};
const SCALE = 12;
const svg = document.getElementById("stage");
const circuit = MANIFEST.circuits[0];
let selectedIdx = -1;
let dragIdx = -1;
let dragOffset = {{x: 0, y: 0}};

function render() {{
  svg.innerHTML = "";
  const posByRef = {{}};
  circuit.instances.forEach((inst) => {{
    posByRef[inst.id] = inst.sourcePosition;
  }});
  circuit.sourceGeometry.portOccurrences.forEach((po) => {{
    posByRef[po.occurrenceId] = po.sourcePosition;
  }});

  // Rubber-band net connections
  const netPts = {{}};
  circuit.instances.forEach((inst) => {{
    (inst.nodes || []).forEach((n) => {{
      if (n.netName === "VDD" || n.netName === "VSS" || n.netName === "vdd" || n.netName === "vss") return;
      (netPts[n.netName] = netPts[n.netName] || []).push(inst.sourcePosition);
    }});
  }});
  Object.values(netPts).forEach((pts) => {{
    for (let i = 1; i < pts.length; i++) {{
      const line = document.createElementNS("http://www.w3.org/2000/svg", "line");
      line.setAttribute("x1", pts[0].x * SCALE);
      line.setAttribute("y1", pts[0].y * SCALE);
      line.setAttribute("x2", pts[i].x * SCALE);
      line.setAttribute("y2", pts[i].y * SCALE);
      line.setAttribute("class", "rubber");
      svg.appendChild(line);
    }}
  }});

  // Instances
  circuit.instances.forEach((inst, idx) => {{
    const g = document.createElementNS("http://www.w3.org/2000/svg", "g");
    const x = inst.sourcePosition.x * SCALE;
    const y = inst.sourcePosition.y * SCALE;
    const rect = document.createElementNS("http://www.w3.org/2000/svg", "rect");
    rect.setAttribute("x", x - 30);
    rect.setAttribute("y", y - 34);
    rect.setAttribute("width", 60);
    rect.setAttribute("height", 68);
    rect.setAttribute("rx", 5);
    rect.setAttribute("class", "inst-box" + (idx === selectedIdx ? " selected" : ""));
    rect.addEventListener("mousedown", (ev) => {{
      selectedIdx = idx;
      dragIdx = idx;
      dragOffset = {{x: ev.offsetX - x, y: ev.offsetY - y}};
      render();
    }});
    g.appendChild(rect);

    const label = document.createElementNS("http://www.w3.org/2000/svg", "text");
    label.setAttribute("x", x);
    label.setAttribute("y", y - 4);
    label.setAttribute("text-anchor", "middle");
    label.setAttribute("font-size", "11");
    label.setAttribute("font-weight", "600");
    label.setAttribute("pointer-events", "none");
    label.textContent = inst.reference;
    g.appendChild(label);

    const sub = document.createElementNS("http://www.w3.org/2000/svg", "text");
    sub.setAttribute("x", x);
    sub.setAttribute("y", y + 14);
    sub.setAttribute("text-anchor", "middle");
    sub.setAttribute("font-size", "9.5");
    sub.setAttribute("fill", "#455a64");
    sub.setAttribute("pointer-events", "none");
    const mir = (inst.sourceTransform && inst.sourceTransform.mirror) || "none";
    sub.textContent = mir === "horizontal" ? "MY" : "R0";
    g.appendChild(sub);
    svg.appendChild(g);
  }});
}}

svg.addEventListener("mousemove", (ev) => {{
  if (dragIdx < 0) return;
  const gx = Math.round((ev.offsetX - dragOffset.x) / SCALE);
  const gy = Math.round((ev.offsetY - dragOffset.y) / SCALE);
  circuit.instances[dragIdx].sourcePosition = {{x: Math.max(8, gx), y: Math.max(8, gy)}};
  render();
}});
window.addEventListener("mouseup", () => {{ dragIdx = -1; }});
window.addEventListener("keydown", (ev) => {{
  if (selectedIdx >= 0 && (ev.key === "m" || ev.key === "M")) {{
    const tr = circuit.instances[selectedIdx].sourceTransform || {{rotation: 0, mirror: "none"}};
    tr.mirror = tr.mirror === "horizontal" ? "none" : "horizontal";
    circuit.instances[selectedIdx].sourceTransform = tr;
    render();
  }}
}});

document.getElementById("btn-export").addEventListener("click", () => {{
  const blob = new Blob([JSON.stringify(MANIFEST, null, 2)], {{type: "application/json"}});
  const a = document.createElement("a");
  a.href = URL.createObjectURL(blob);
  a.download = circuit.cellName + "_draft_manifest.json";
  a.click();
}});
render();
</script>
</body></html>
"""
