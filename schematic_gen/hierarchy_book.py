"""Single-file interactive HTML schematic book renderer for hierarchical compilation results."""

from __future__ import annotations

from html import escape
from typing import TYPE_CHECKING

from schematic_gen.pdk import PdkRegistry
from schematic_gen.svg_renderer import render_schematic_svg

if TYPE_CHECKING:  # pragma: no cover
    from schematic_gen.hierarchy import HierarchyBuildResult


def render_hierarchy_book_html(result: "HierarchyBuildResult", pdk: PdkRegistry) -> str:
    """Render a standalone single-file HTML Schematic Book containing TOP + all child sheets."""
    top_name = result.design.top_cell
    sheet_order = [top_name] + [c for c in result.solved_pages if c != top_name]
    title_by_cell = {top_name: f"TOP · {top_name} (顶层架构框图)"}
    if result.hierarchy_intent:
        for p in result.hierarchy_intent.pages:
            title_by_cell[p.cell_name] = f"{p.page_id} · {p.title}"

    rows_html: list[str] = []
    sections_html: list[str] = []
    for cname in sheet_order:
        solved = result.solved_pages[cname]
        rep = result.audits[cname]
        m = rep.metrics
        badge = "PASS" if rep.overall_passed else "FAIL"
        rows_html.append(
            f'<tr><td><a href="#{escape(cname)}"><code>{escape(cname)}</code></a></td>'
            f"<td>{escape(title_by_cell.get(cname, cname))}</td>"
            f"<td>{m['instanceCount']}</td><td>{m['netCount']}</td>"
            f"<td>{m['wireCrossingCount']}</td><td><strong>{badge}</strong></td></tr>"
        )
        svg_inline = render_schematic_svg(solved, pdk, show_groups=True)
        sections_html.append(
            f'<section id="{escape(cname)}" class="sheet">'
            f"<h2>{escape(title_by_cell.get(cname, cname))}</h2>"
            f'<p class="meta">Instances: <code>{m["instanceCount"]}</code> · Nets: <code>{m["netCount"]}</code> · '
            f'Crossings: <code>{m["wireCrossingCount"]}</code> · DRC: <code>{badge}</code></p>'
            f'<div class="svg-wrap">{svg_inline}</div></section>'
        )

    lvs_badge = "PASS (100% 晶体管与网络双射同构)" if result.flatten_lvs_passed else f"FAIL ({result.flatten_lvs_Diffs})"
    return f"""<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<title>{escape(top_name)} · Hierarchical Schematic Book</title>
<style>
  body {{ margin:0; background:#eef3f1; color:#162d31; font:15px/1.6 "Segoe UI","Microsoft YaHei",sans-serif; }}
  main {{ max-width:1360px; margin:24px auto; padding:28px 36px; background:#fff; border-top:5px solid #00695c; }}
  h1 {{ margin:0 0 8px; font-size:1.65rem; }}
  h2 {{ margin:24px 0 6px; font-size:1.15rem; border-left:4px solid #00695c; padding-left:10px; }}
  .summary {{ background:#f3f8f6; padding:12px 16px; border-radius:6px; margin:12px 0 20px; }}
  table {{ width:100%; border-collapse:collapse; margin-bottom:24px; font-size:.92rem; }}
  th, td {{ text-align:left; padding:8px 10px; border-bottom:1px solid #d5e2df; }}
  th {{ background:#f1f6f4; color:#3d5a5e; }}
  .sheet {{ margin-top:28px; padding-top:16px; border-top:1px solid #d5e2df; }}
  .meta {{ color:#4f6b6e; font-size:.88rem; margin:0 0 10px; }}
  .svg-wrap {{ overflow-x:auto; border:1px solid #dce7e4; border-radius:6px; background:#fcfdfb; padding:8px; }}
  a {{ color:#00695c; text-decoration:none; }}
</style>
</head>
<body><main>
  <h1>{escape(top_name)} · 多页层次化原理图册</h1>
  <div class="summary">
    <strong>展平还原 LVS 核对（Flatten-Back Equivalence）</strong>：{escape(lvs_badge)}<br>
    <strong>总页数</strong>：{len(sheet_order)} 页（1 张顶层信号流框图 + {len(sheet_order) - 1} 张功能子原理图）
  </div>
  <table>
    <thead><tr><th>单元名</th><th>功能页说明</th><th>实例数</th><th>网络数</th><th>正交跨线</th><th>DRC 门禁</th></tr></thead>
    <tbody>{''.join(rows_html)}</tbody>
  </table>
  {''.join(sections_html)}
</main></body></html>
"""
