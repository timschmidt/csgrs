#!/usr/bin/env python3
"""Render perf-atlas JSON documents into one self-contained HTML report.

Usage: render.py OUTPUT_DIR ATLAS.json [ATLAS.json ...]

The report needs no network access: styles, scripts and SVG charts are inline.
For each operation it draws time against magnitude (one line per input kind,
at most four per panel), the spread of fuzzed inputs, and magnitude grids for
two-argument operations. Cliffs (a time rise of at least 4x between adjacent
magnitudes), valleys (a point at least 2x faster than both neighbours), failed
calls and series stopped by the single-call ceiling are flagged on the charts
and listed in the highlights table. Every chart has a data-table view.
"""
import html
import json
import math
import sys
from pathlib import Path

CLIFF = 4.0
VALLEY = 2.0
W, H = 680, 300
ML, MR, MT, MB = 70, 120, 18, 46
PW, PH = W - ML - MR, H - MT - MB
SHAPES = ["circle", "square", "triangle", "diamond"]
RAMP = ["#cde2fb", "#b7d3f6", "#9ec5f4", "#86b6ef", "#6da7ec", "#5598e7", "#3987e5",
        "#2a78d6", "#256abf", "#1c5cab", "#184f95", "#104281", "#0d366b"]

CSS = """
.viz-root{color-scheme:light;--surface-1:#fcfcfb;--page:#f9f9f7;--text-primary:#0b0b0b;--text-secondary:#52514e;
--muted:#898781;--grid:#e1e0d9;--axis:#c3c2b7;--border:rgba(11,11,11,.10);
--series-1:#2a78d6;--series-2:#eb6834;--series-3:#1baf7a;--series-4:#eda100;--critical:#d03b3b;--warning:#fab219}
@media (prefers-color-scheme:dark){:root:where(:not([data-theme="light"])) .viz-root{color-scheme:dark;--surface-1:#1a1a19;--page:#0d0d0d;
--text-primary:#f0efec;--text-secondary:#c3c2b7;--grid:#2c2c2a;--axis:#383835;--border:rgba(255,255,255,.10);
--series-1:#3987e5;--series-2:#d95926;--series-3:#199e70;--series-4:#c98500}}
:root[data-theme="dark"] .viz-root{color-scheme:dark;--surface-1:#1a1a19;--page:#0d0d0d;--text-primary:#f0efec;--text-secondary:#c3c2b7;
--grid:#2c2c2a;--axis:#383835;--border:rgba(255,255,255,.10);--series-1:#3987e5;--series-2:#d95926;--series-3:#199e70;--series-4:#c98500}
body{margin:0;background:var(--page);}
.viz-root{font-family:system-ui,-apple-system,"Segoe UI",sans-serif;color:var(--text-primary);background:var(--page);
padding:16px;max-width:1500px;margin:0 auto;line-height:1.45}
h1{font-size:22px;margin:8px 0}h2{font-size:18px;margin:28px 0 8px}h3{font-size:15px;margin:0 0 4px}
p,li{color:var(--text-secondary);font-size:14px}a{color:var(--series-1)}
.card{background:var(--surface-1);border:1px solid var(--border);border-radius:8px;padding:14px 16px;margin:12px 0}
.charts{display:flex;flex-wrap:wrap;gap:12px}.chart{flex:1 1 560px;max-width:700px}
.chart svg{width:100%;height:auto;display:block;touch-action:none}
.chart .caption{font-size:12px;color:var(--text-secondary);margin:2px 0 4px}
.legend{display:flex;flex-wrap:wrap;gap:12px;font-size:12px;color:var(--text-secondary);margin:2px 0 4px}
.legend span{display:inline-flex;align-items:center;gap:6px}
.legend svg{width:20px;height:10px;display:inline}
table{border-collapse:collapse;font-size:12px;font-variant-numeric:tabular-nums}
td,th{padding:3px 8px;border-bottom:1px solid var(--grid);text-align:right}th:first-child,td:first-child{text-align:left}
details{margin-top:6px}summary{cursor:pointer;font-size:13px;color:var(--text-secondary)}
.tbl{overflow-x:auto;max-height:420px}
.gridline{stroke:var(--grid);stroke-width:1}.axisline{stroke:var(--axis);stroke-width:1}
.ticklabel{fill:var(--muted);font-size:11px;font-variant-numeric:tabular-nums}
.axistitle{fill:var(--text-secondary);font-size:12px}
.direct{font-size:11px;fill:var(--text-secondary)}.note{font-size:11px;fill:var(--text-secondary)}
.cell{fill:var(--l)}:root[data-theme="dark"] .cell{fill:var(--d)}
@media (prefers-color-scheme:dark){:root:where(:not([data-theme="light"])) .cell{fill:var(--d)}}
.cell:hover{stroke:var(--text-primary);stroke-width:1.5}
#tip{position:fixed;pointer-events:none;background:var(--surface-1);color:var(--text-primary);border:1px solid var(--border);
border-radius:6px;padding:6px 8px;font-size:12px;box-shadow:0 2px 8px rgba(0,0,0,.15);display:none;z-index:10;max-width:340px}
#tip .row{display:flex;align-items:center;gap:6px}#tip b{font-variant-numeric:tabular-nums}#tip .name{color:var(--text-secondary)}
#tip .key{display:inline-block;width:12px;height:0;border-top:2px solid}
.toolbar{display:flex;gap:12px;align-items:center;flex-wrap:wrap;margin:8px 0}
button,input{font:inherit;font-size:13px;padding:4px 10px;border:1px solid var(--border);border-radius:6px;background:var(--surface-1);color:var(--text-primary)}
.status{font-size:12px;color:var(--text-secondary)}.toc{columns:3 260px;font-size:13px}.toc a{display:block}
.op[hidden]{display:none}
"""

JS = r"""
const tip=document.getElementById('tip');
function showTip(e,rows,title){tip.textContent='';if(title){const t=document.createElement('div');t.className='name';t.textContent=title;tip.appendChild(t);}
for(const r of rows){const d=document.createElement('div');d.className='row';if(r.color){const k=document.createElement('span');k.className='key';k.style.borderColor=r.color;d.appendChild(k);}
const b=document.createElement('b');b.textContent=r.value;d.appendChild(b);const n=document.createElement('span');n.className='name';n.textContent=r.name;d.appendChild(n);tip.appendChild(d);}
tip.style.display='block';const x=Math.min(e.clientX+14,window.innerWidth-tip.offsetWidth-8),y=Math.min(e.clientY+14,window.innerHeight-tip.offsetHeight-8);
tip.style.left=x+'px';tip.style.top=y+'px';}
function hideTip(){tip.style.display='none';}
function fmt(ns){if(ns==null||!isFinite(ns))return '—';const u=[[1e9,'s'],[1e6,'ms'],[1e3,'µs'],[1,'ns']];for(const [d,s] of u){if(ns>=d||s==='ns'){const v=ns/d;return (v>=100?v.toFixed(0):v>=10?v.toFixed(1):v.toFixed(2))+' '+s;}}}
function col(v){return getComputedStyle(document.querySelector('.viz-root')).getPropertyValue(v).trim();}
document.querySelectorAll('svg[data-line]').forEach(svg=>{const d=JSON.parse(svg.dataset.d);const cross=svg.querySelector('.cross');
const hit=svg.querySelector('.hit');hit.addEventListener('pointermove',e=>{const pt=svg.createSVGPoint();pt.x=e.clientX;pt.y=e.clientY;const p=pt.matrixTransform(svg.getScreenCTM().inverse());
let best=0,bd=1e9;d.px.forEach((x,i)=>{const dd=Math.abs(x-p.x);if(dd<bd){bd=dd;best=i;}});cross.setAttribute('x1',d.px[best]);cross.setAttribute('x2',d.px[best]);cross.style.display='';
showTip(e,d.series.map(s=>({color:col(s.c),value:fmt(s.ys[best])+(s.st[best]&&s.st[best]!=='ok'&&s.st[best]!=='single'?' ('+s.st[best]+')':''),name:s.name})),d.xl+' = '+d.xs[best]);});
hit.addEventListener('pointerleave',()=>{cross.style.display='none';hideTip();});});
document.addEventListener('pointerover',e=>{const t=e.target.closest('[data-tip]');if(!t)return;const v=JSON.parse(t.dataset.tip);
const move=ev=>showTip(ev,v.rows,v.title);move(e);t.addEventListener('pointermove',move);t.addEventListener('pointerleave',()=>{hideTip();t.removeEventListener('pointermove',move);},{once:true});});
document.getElementById('theme').addEventListener('click',()=>{const r=document.documentElement;const dark=r.dataset.theme?r.dataset.theme==='dark':matchMedia('(prefers-color-scheme: dark)').matches;r.dataset.theme=dark?'light':'dark';});
const q=document.getElementById('filter');q.addEventListener('input',()=>{const s=q.value.toLowerCase();document.querySelectorAll('.op').forEach(o=>{o.hidden=s&&!o.dataset.name.includes(s);});});
"""


def esc(text):
    return html.escape(str(text), quote=True)


def fmt_ns(ns):
    if ns is None or not math.isfinite(ns):
        return "—"
    for div, unit in ((1e9, "s"), (1e6, "ms"), (1e3, "µs"), (1, "ns")):
        if ns >= div or unit == "ns":
            v = ns / div
            return (f"{v:.0f}" if v >= 100 else f"{v:.1f}" if v >= 10 else f"{v:.2f}") + " " + unit


def fmt_mag(m):
    if m is None:
        return "—"
    if m == int(m):
        m = int(m)
        for div, suffix in ((1 << 30, "G"), (1 << 20, "M"), (1 << 10, "K")):
            if abs(m) >= div and m % div == 0:
                return f"{m // div}{suffix}"
        return str(m)
    return f"{m:g}"


class Scale:
    def __init__(self, values, lo_px, hi_px, log=None):
        vals = [v for v in values if v is not None and math.isfinite(v)]
        if not vals:
            vals = [1.0]
        positive = all(v > 0 for v in vals)
        self.log = (positive and max(vals) / min(vals) >= 16) if log is None else log
        if self.log:
            self.lo, self.hi = math.log(min(vals)), math.log(max(vals))
        else:
            self.lo, self.hi = min(vals), max(vals)
        if self.hi == self.lo:
            self.lo -= 1
            self.hi += 1
        self.lo_px, self.hi_px = lo_px, hi_px

    def __call__(self, v):
        t = ((math.log(v) if self.log else v) - self.lo) / (self.hi - self.lo)
        return self.lo_px + t * (self.hi_px - self.lo_px)


def time_scale(values):
    vals = [v for v in values if v is not None and math.isfinite(v) and v > 0]
    lo, hi = (min(vals), max(vals)) if vals else (1, 10)
    lo = 10 ** math.floor(math.log10(lo))
    hi = 10 ** math.ceil(math.log10(hi))
    if hi <= lo:
        hi = lo * 10
    s = Scale([lo, hi], MT + PH, MT, log=True)
    ticks = [10 ** e for e in range(int(round(math.log10(lo))), int(round(math.log10(hi))) + 1)]
    return s, ticks


def x_ticks(mags, scale):
    mags = sorted(set(m for m in mags if m is not None))
    if len(mags) <= 9:
        return mags
    step = math.ceil(len(mags) / 8)
    return mags[::step] + ([mags[-1]] if mags[-1] not in mags[::step] else [])


def marker(shape, x, y, var, hollow=False, size=4.5):
    fill = "var(--surface-1)" if hollow else f"var({var})"
    common = f'fill="{fill}" stroke="{"var(" + var + ")" if hollow else "var(--surface-1)"}" stroke-width="{2 if hollow else 1.5}"'
    if shape == "square":
        return f'<rect x="{x - size:.1f}" y="{y - size:.1f}" width="{2 * size:.1f}" height="{2 * size:.1f}" rx="1" {common}/>'
    if shape == "triangle":
        return f'<path d="M{x:.1f},{y - size * 1.2:.1f}L{x + size * 1.1:.1f},{y + size * 0.8:.1f}L{x - size * 1.1:.1f},{y + size * 0.8:.1f}Z" {common}/>'
    if shape == "diamond":
        return f'<path d="M{x:.1f},{y - size * 1.2:.1f}L{x + size * 1.2:.1f},{y:.1f}L{x:.1f},{y + size * 1.2:.1f}L{x - size * 1.2:.1f},{y:.1f}Z" {common}/>'
    return f'<circle cx="{x:.1f}" cy="{y:.1f}" r="{size}" {common}/>'


def axes(xs, ys, xticks, yticks, xlabel):
    out = []
    for t in yticks:
        y = ys(t)
        out.append(f'<line class="gridline" x1="{ML}" x2="{ML + PW}" y1="{y:.1f}" y2="{y:.1f}"/>')
        out.append(f'<text class="ticklabel" x="{ML - 8}" y="{y + 4:.1f}" text-anchor="end">{esc(fmt_ns(t))}</text>')
    out.append(f'<line class="axisline" x1="{ML}" x2="{ML + PW}" y1="{MT + PH}" y2="{MT + PH}"/>')
    for t in xticks:
        x = xs(t)
        out.append(f'<line class="axisline" x1="{x:.1f}" x2="{x:.1f}" y1="{MT + PH}" y2="{MT + PH + 4}"/>')
        out.append(f'<text class="ticklabel" x="{x:.1f}" y="{MT + PH + 17}" text-anchor="middle">{esc(fmt_mag(t))}</text>')
    out.append(f'<text class="axistitle" x="{ML + PW / 2}" y="{H - 8}" text-anchor="middle">{esc(xlabel)}{" (log scale)" if xs.log else ""}</text>')
    out.append(f'<text class="axistitle" x="14" y="{MT + PH / 2}" transform="rotate(-90 14 {MT + PH / 2})" text-anchor="middle">time per call (log scale)</text>')
    return out


def anomalies(points):
    """Cliffs, valleys and failures in one series of [m, median, min, max, iters, status]."""
    found = []
    ok = [(i, p) for i, p in enumerate(points) if p[1] is not None and p[5] in ("ok", "single")]
    for (i, a), (j, b) in zip(ok, ok[1:]):
        if a[1] > 0 and b[1] / a[1] >= CLIFF:
            found.append(("cliff", b[1] / a[1], a, b))
    for (_, a), (_, b), (_, c) in zip(ok, ok[1:], ok[2:]):
        if b[1] > 0 and a[1] / b[1] >= VALLEY and c[1] / b[1] >= VALLEY:
            found.append(("valley", min(a[1], c[1]) / b[1], b, a if a[1] <= c[1] else c))
    for p in points:
        if p[5] == "error":
            found.append(("error", 0, p, p))
        if p[5] == "ceiling":
            found.append(("ceiling", 0, p, p))
    return found


def line_chart(op, series, xlabel, slots):
    allx = [p[0] for s in series for p in s["points"]]
    ally = [p[1] for s in series for p in s["points"]]
    xs = Scale(allx, ML, ML + PW)
    ys, yt = time_scale(ally)
    out = [f'<svg viewBox="0 0 {W} {H}" data-line role="img" aria-label="{esc(op["name"])}: time per call by {esc(xlabel)}" data-d="__DATA__">']
    out += axes(xs, ys, x_ticks(allx, xs), yt, xlabel)
    xvals = sorted(set(p[0] for s in series for p in s["points"] if p[0] is not None))
    data = {"xl": xlabel, "xs": [fmt_mag(x) for x in xvals], "px": [round(xs(x), 1) for x in xvals], "series": []}
    labels = []
    notes = []
    for s in series:
        slot = slots[s["kind"]]
        var = f"--series-{slot + 1}"
        shape = SHAPES[slot % 4]
        pts = [p for p in s["points"] if p[1] is not None]
        good = [p for p in pts if p[5] != "error"]
        if len(good) > 1:
            d = "M" + "L".join(f"{xs(p[0]):.1f},{ys(p[1]):.1f}" for p in good)
            out.append(f'<path d="{d}" fill="none" stroke="var({var})" stroke-width="2" stroke-linejoin="round"/>')
        for p in pts:
            out.append(marker(shape, xs(p[0]), ys(p[1]), var, hollow=p[5] == "error"))
            if p[5] == "ceiling":
                out.append(f'<text class="note" x="{xs(p[0]) + 6:.1f}" y="{ys(p[1]) - 6:.1f}">ceiling</text>')
        if good:
            labels.append([ys(good[-1][1]), xs(good[-1][0]), s["kind"]])
        by_x = {p[0]: p for p in s["points"]}
        data["series"].append({"name": s["kind"], "c": var,
                               "ys": [by_x[x][1] if x in by_x else None for x in xvals],
                               "st": [by_x[x][5] if x in by_x else None for x in xvals]})
        for kind, ratio, a, b in anomalies(s["points"]):
            if kind == "cliff":
                notes.append((ratio, xs(b[0]), ys(b[1])))
    for ratio, x, y in sorted(notes, reverse=True)[:2]:
        # Keep notes clear of the direct labels at the right edge.
        anchor_end = x > ML + PW * 0.7
        out.append(f'<text class="note" x="{x - 8 if anchor_end else x + 6:.1f}" y="{y + 14:.1f}" '
                   f'text-anchor="{"end" if anchor_end else "start"}">×{ratio:.0f} cliff</text>')
    if len(series) <= 4:
        labels.sort()
        last = -1e9
        for label in labels:
            label[0] = max(label[0], last + 13)
            last = label[0]
        for y, x, name in labels:
            out.append(f'<text class="direct" x="{ML + PW + 8}" y="{y + 4:.1f}">{esc(name)}</text>')
    out.append(f'<line class="cross axisline" x1="0" x2="0" y1="{MT}" y2="{MT + PH}" style="display:none"/>')
    out.append(f'<rect class="hit" x="{ML}" y="{MT}" width="{PW}" height="{PH}" fill="transparent"/>')
    out.append("</svg>")
    return "".join(out).replace("__DATA__", esc(json.dumps(data)), 1)


def legend(kinds, slots):
    if len(kinds) < 2:
        return ""
    items = []
    for k in kinds:
        slot = slots[k]
        items.append(f'<span><svg viewBox="0 0 20 10"><line x1="0" x2="20" y1="5" y2="5" stroke="var(--series-{slot + 1})" stroke-width="2"/>'
                     f'{marker(SHAPES[slot % 4], 10, 5, f"--series-{slot + 1}", size=3.2)}</svg>{esc(k)}</span>')
    return f'<div class="legend">{"".join(items)}</div>'


def fuzz_chart(op, series, xlabel):
    pts = [p for p in series["points"] if p[1] is not None]
    xs = Scale([p[0] for p in pts], ML, ML + PW)
    ys, yt = time_scale([p[1] for p in pts])
    out = [f'<svg viewBox="0 0 {W} {H}" role="img" aria-label="{esc(op["name"])}: fuzzed inputs">']
    out += axes(xs, ys, x_ticks([p[0] for p in pts], xs), yt, xlabel)
    groups = {}
    for p in pts:
        groups.setdefault(p[0], []).append(p)
    mags = sorted(groups)
    spacing = (PW / max(len(mags) - 1, 1)) * 0.25
    for m in mags:
        group = groups[m]
        for i, p in enumerate(group):
            jitter = (i / max(len(group) - 1, 1) - 0.5) * spacing if len(group) > 1 else 0
            x, y = xs(m) + jitter, ys(p[1])
            tipd = {"title": f"{xlabel} = {fmt_mag(m)}, fuzzed input {i + 1}",
                    "rows": [{"value": fmt_ns(p[1]), "name": "error" if p[5] == "error" else ("ceiling" if p[5] == "ceiling" else "time per call")}]}
            out.append(f'<g data-tip="{esc(json.dumps(tipd))}">{marker("circle", x, y, "--series-1", hollow=p[5] == "error", size=4)}'
                       f'<circle cx="{x:.1f}" cy="{y:.1f}" r="12" fill="transparent"/></g>')
    med = []
    for m in mags:
        v = sorted(p[1] for p in groups[m])
        med.append((m, v[len(v) // 2]))
    if len(med) > 1:
        d = "M" + "L".join(f"{xs(m):.1f},{ys(v):.1f}" for m, v in med)
        out.append(f'<path d="{d}" fill="none" stroke="var(--text-secondary)" stroke-width="1.5" stroke-dasharray="4 3"/>')
        out.append(f'<text class="direct" x="{ML + PW + 8}" y="{ys(med[-1][1]) + 4:.1f}">median</text>')
    out.append("</svg>")
    return "".join(out)


def ramp_color(t, dark=False):
    t = min(max(t, 0.0), 1.0)
    if dark:
        t = 1 - t
    return RAMP[round(t * (len(RAMP) - 1))]


def heatmap(op, grid):
    a, b, ns = grid["a"], grid["b"], grid["ns"]
    errors = grid.get("errors") or [[False] * len(a) for _ in b]
    vals = [v for row in ns for v in row if v is not None]
    if not vals:
        return ""
    lo, hi = math.log(min(vals)), math.log(max(vals))
    span = (hi - lo) or 1.0
    cw, ch = PW / len(a), PH / len(b)
    out = [f'<svg viewBox="0 0 {W} {H}" role="img" aria-label="{esc(op["name"])}: {esc(grid["kind"])} grid">']
    for j, y in enumerate(b):
        cy = MT + PH - (j + 1) * ch
        for i, x in enumerate(a):
            v = ns[j][i]
            cx = ML + i * cw
            if v is None:
                out.append(f'<rect x="{cx + 1:.1f}" y="{cy + 1:.1f}" width="{cw - 2:.1f}" height="{ch - 2:.1f}" fill="none" stroke="var(--grid)"/>')
                continue
            t = (math.log(v) - lo) / span
            tipd = {"title": f'{grid["a_label"]} = {fmt_mag(x)}, {grid["b_label"]} = {fmt_mag(y)}',
                    "rows": [{"value": fmt_ns(v), "name": "error" if errors[j][i] else "time per call"}]}
            out.append(f'<rect class="cell" data-tip="{esc(json.dumps(tipd))}" style="--l:{ramp_color(t)};--d:{ramp_color(t, True)}" '
                       f'x="{cx + 1:.1f}" y="{cy + 1:.1f}" width="{cw - 2:.1f}" height="{ch - 2:.1f}" rx="2"/>')
            if errors[j][i]:
                out.append(f'<text class="note" x="{cx + cw / 2:.1f}" y="{cy + ch / 2 + 4:.1f}" text-anchor="middle">×</text>')
    step_a = max(1, math.ceil(len(a) / 10))
    for i, x in enumerate(a):
        if i % step_a == 0:
            out.append(f'<text class="ticklabel" x="{ML + (i + 0.5) * cw:.1f}" y="{MT + PH + 16}" text-anchor="middle">{esc(fmt_mag(x))}</text>')
    step_b = max(1, math.ceil(len(b) / 10))
    for j, y in enumerate(b):
        if j % step_b == 0:
            out.append(f'<text class="ticklabel" x="{ML - 8}" y="{MT + PH - (j + 0.5) * ch + 4:.1f}" text-anchor="end">{esc(fmt_mag(y))}</text>')
    out.append(f'<text class="axistitle" x="{ML + PW / 2}" y="{H - 8}" text-anchor="middle">{esc(grid["a_label"])}</text>')
    out.append(f'<text class="axistitle" x="14" y="{MT + PH / 2}" transform="rotate(-90 14 {MT + PH / 2})" text-anchor="middle">{esc(grid["b_label"])}</text>')
    # Colour key.
    kx = ML + PW + 22
    for k in range(len(RAMP)):
        t = k / (len(RAMP) - 1)
        yk = MT + PH - (k + 1) * PH / len(RAMP)
        out.append(f'<rect class="cell" style="--l:{ramp_color(t)};--d:{ramp_color(t, True)}" x="{kx}" y="{yk:.1f}" width="14" height="{PH / len(RAMP):.1f}"/>')
    out.append(f'<text class="ticklabel" x="{kx + 18}" y="{MT + 10}">{esc(fmt_ns(max(vals)))}</text>')
    out.append(f'<text class="ticklabel" x="{kx + 18}" y="{MT + PH}">{esc(fmt_ns(min(vals)))}</text>')
    out.append("</svg>")
    return "".join(out)


def table(op):
    parts = []
    if op["series"]:
        mags = sorted(set(p[0] for s in op["series"] for p in s["points"] if p[0] is not None))
        head = "".join(f"<th>{esc(s['kind'])}</th>" for s in op["series"])
        rows = []
        for m in mags:
            cells = []
            for s in op["series"]:
                p = next((p for p in s["points"] if p[0] == m), None)
                text = "—" if p is None else (p[5] if p[1] is None else fmt_ns(p[1]) + ("" if p[5] in ("ok", "single") else f" ({p[5]})"))
                cells.append(f"<td>{esc(text)}</td>")
            rows.append(f"<tr><td>{esc(fmt_mag(m))}</td>{''.join(cells)}</tr>")
        parts.append(f"<table><tr><th>{esc(op['magnitude'])}</th>{head}</tr>{''.join(rows)}</table>")
    for f in op["fuzz"]:
        groups = {}
        for p in f["points"]:
            if p[1] is not None:
                groups.setdefault(p[0], []).append(p[1])
        rows = []
        for m in sorted(groups):
            v = sorted(groups[m])
            rows.append(f"<tr><td>{esc(fmt_mag(m))}</td><td>{len(v)}</td><td>{esc(fmt_ns(v[0]))}</td><td>{esc(fmt_ns(v[len(v) // 2]))}</td><td>{esc(fmt_ns(v[-1]))}</td></tr>")
        parts.append(f"<p>Fuzzed inputs ({esc(f['kind'])})</p><table><tr><th>{esc(op['magnitude'])}</th><th>inputs</th><th>fastest</th><th>median</th><th>slowest</th></tr>{''.join(rows)}</table>")
    for g in op["grids"]:
        head = "".join(f"<th>{esc(fmt_mag(x))}</th>" for x in g["a"])
        rows = "".join(f"<tr><td>{esc(fmt_mag(y))}</td>" + "".join(f"<td>{esc(fmt_ns(v))}</td>" for v in g["ns"][j]) + "</tr>" for j, y in enumerate(g["b"]))
        parts.append(f"<p>{esc(g['kind'])}: rows {esc(g['b_label'])}, columns {esc(g['a_label'])}</p><table><tr><th></th>{head}</tr>{rows}</table>")
    return f'<details><summary>Data table</summary><div class="tbl">{"".join(parts)}</div></details>'


def anchor(crate, op):
    return "op-" + "".join(c if c.isalnum() else "-" for c in f"{crate}-{op['group']}-{op['name']}").lower()


def render(docs):
    highlights = []
    body = []
    toc = []
    for doc in docs:
        crate = doc["crate"]
        toc.append(f'<a href="#crate-{esc(crate)}"><b>{esc(crate)}</b> — {len(doc["operations"])} operations</a>')
        body.append(f'<h2 id="crate-{esc(crate)}">{esc(crate)}</h2>')
        body.append(f'<p>{"Quick" if doc["quick"] else "Full"} sweep, {doc["os"]}/{doc["arch"]}, seed {doc["seed"]}, '
                    f'{doc["elapsed_s"]:.0f} s. Batches of at least {fmt_ns(doc["settings"]["target_batch_ns"])}, '
                    f'{doc["settings"]["samples"]} samples per point, single-call ceiling {fmt_ns(doc["settings"]["single_ceiling_ns"])}, '
                    f'{doc["settings"]["fuzz_per_magnitude"]} fuzzed inputs per magnitude.</p>')
        group = None
        for op in doc["operations"]:
            if op["group"] != group:
                group = op["group"]
                body.append(f"<h3 style='margin-top:18px'>{esc(crate)} · {esc(group)}</h3>")
            kinds = [s["kind"] for s in op["series"]]
            slots = {k: i % 8 for i, k in enumerate(kinds)}
            for s in op["series"]:
                for kind, ratio, a, b in anomalies(s["points"]):
                    highlights.append((kind, ratio, crate, op, s["kind"], a, b))
            charts = []
            for start in range(0, len(op["series"]), 4):
                panel = op["series"][start:start + 4]
                charts.append(f'<div class="chart"><div class="caption">Median time per call by {esc(op["magnitude"])}</div>'
                              f'{legend([s["kind"] for s in panel], slots)}{line_chart(op, panel, op["magnitude"], slots)}</div>')
            for f in op["fuzz"]:
                if any(p[1] is not None for p in f["points"]):
                    charts.append(f'<div class="chart"><div class="caption">Fuzzed {esc(f["kind"])} inputs: each dot is one input; dashed line is the median</div>{fuzz_chart(op, f, op["magnitude"])}</div>')
            for g in op["grids"]:
                h = heatmap(op, g)
                if h:
                    charts.append(f'<div class="chart"><div class="caption">{esc(g["kind"])}: median time per call over both operand sizes (darker is slower)</div>{h}</div>')
            body.append(f'<div class="card op" id="{anchor(crate, op)}" data-name="{esc((crate + " " + op["group"] + " " + op["name"]).lower())}">'
                        f'<h3>{esc(op["name"])}</h3><p>{esc(op["description"])}</p><div class="charts">{"".join(charts)}</div>{table(op)}</div>')
    cliffs = sorted((h for h in highlights if h[0] == "cliff"), key=lambda h: -h[1])
    valleys = sorted((h for h in highlights if h[0] == "valley"), key=lambda h: -h[1])
    failures = [h for h in highlights if h[0] in ("error", "ceiling")]

    def rows(items, limit, what):
        out = []
        for kind, ratio, crate, op, series, a, b in items[:limit]:
            where = f"{fmt_mag(a[0])} → {fmt_mag(b[0])}" if kind == "cliff" else fmt_mag(a[0])
            detail = f"×{ratio:.1f}" if kind in ("cliff", "valley") else ("call failed" if kind == "error" else "single call over ceiling")
            out.append(f'<tr><td><a href="#{anchor(crate, op)}">{esc(crate)} · {esc(op["name"])}</a></td><td>{esc(series)}</td>'
                       f'<td>{esc(where)}</td><td>{esc(fmt_ns(a[1]))}</td><td>{esc(fmt_ns(b[1]))}</td><td>{esc(detail)}</td></tr>')
        if not out:
            return f"<p>No {what} found.</p>"
        return ("<table><tr><th>Operation</th><th>Input kind</th><th>Magnitude</th><th>Time</th><th>Compared with</th><th>Size</th></tr>"
                + "".join(out) + "</table>" + (f"<p>{len(items) - limit} more not shown.</p>" if len(items) > limit else ""))

    head = f"""<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Performance atlas</title><style>{CSS}</style></head><body><div class="viz-root"><div id="tip"></div>
<h1>Performance atlas</h1>
<p>Wall-clock time per call for each operation, swept across input kinds, magnitudes and fuzzed inputs. Both axes of the line
charts are logarithmic where the data spans a wide range, so equal slopes mean equal growth rates and a straight line is a power law.
A <b>cliff</b> is a rise of at least {CLIFF:.0f}× between adjacent magnitudes; a <b>valley</b> is a point at least {VALLEY:.0f}× faster than
both neighbours. Hollow markers are calls that returned an error, which are not fast successes. "Ceiling" marks a single call longer than the
sweep's ceiling; larger magnitudes of that series were skipped. These are focused probes on one host; compare reports only from the
same host, toolchain and settings.</p>
<div class="toolbar"><input id="filter" placeholder="Filter operations" aria-label="Filter operations"><button id="theme">Toggle dark mode</button>
<span class="status">{sum(len(d["operations"]) for d in docs)} operations in {len(docs)} crates, {len(cliffs)} cliffs, {len(valleys)} valleys, {len(failures)} failures or ceilings</span></div>
<div class="card"><h3>Crates</h3><div class="toc">{"".join(toc)}</div></div>
<div class="card"><h3>Largest cliffs</h3>{rows(cliffs, 40, "cliffs")}</div>
<div class="card"><h3>Valleys</h3>{rows(valleys, 20, "valleys")}</div>
<div class="card"><h3>Failures and ceilings</h3>{rows(failures, 40, "failures or ceilings")}</div>
"""
    return head + "".join(body) + f"<script>{JS}</script></div></body></html>\n"


def main():
    if len(sys.argv) < 3:
        sys.exit(__doc__)
    out = Path(sys.argv[1])
    docs = []
    for path in sys.argv[2:]:
        doc = json.loads(Path(path).read_text())
        assert doc.get("format") == "perf-atlas/1", f"{path}: not a perf-atlas document"
        docs.append(doc)
    docs.sort(key=lambda d: d["crate"])
    out.mkdir(parents=True, exist_ok=True)
    (out / "index.html").write_text(render(docs))
    print(f"wrote {out / 'index.html'}")


if __name__ == "__main__":
    main()
