#!/usr/bin/env python3
"""Render trusted perf-atlas/1 records as a standalone HTML/SVG report.

Usage: render.py OUTPUT_DIR ATLAS.json [ATLAS.json ...]

Charts and interactions are embedded; no network fetch is needed to view them.
The producer's callback statuses are displayed without checking mathematical
correctness, certainty, or equivalent workloads. Ordinary-series highlights use
retained successful records, which may straddle failures or missing timings.
Fuzz/grid errors do not enter those highlight tables.

Inputs must provide the expected nested keys, finite plotted magnitudes, and
positive durations where present. The CLI asserts the format tag but performs no
complete schema, matrix-shape, or HTML-context validation. It is intended for
trusted harness output, not arbitrary untrusted JSON. Invalid/extreme values can
raise exceptions. Output construction retains strings and plot data in memory.

Tooltips round timings to three significant figures before browser formatting;
data tables summarize records and are not lossless exports. Keep the source JSON
and workload/environment evidence when interpreting measurements."""
import html
import json
import math
import sys
from pathlib import Path

# Minimum successful-row duration ratio highlighted as a cliff.
CLIFF = 4.0
# Minimum ratio of both retained neighbours to the faster valley row.
VALLEY = 2.0
# SVG viewBox size shared by line, scatter, and heatmap panels.
W, H = 680, 300
# Plot margins leave room for ticks, titles, and right-side direct labels.
ML, MR, MT, MB = 70, 120, 18, 46
# Drawable plot dimensions after subtracting margins.
PW, PH = W - ML - MR, H - MT - MB
# Marker shapes cycle every four colour slots.
SHAPES = ["circle", "square", "triangle", "diamond"]
# Thirteen-bin palette reference; rendering uses its length, while CSS owns actual fills.
RAMP = ["#cde2fb", "#b7d3f6", "#9ec5f4", "#86b6ef", "#6da7ec", "#5598e7", "#3987e5",
        "#2a78d6", "#256abf", "#1c5cab", "#184f95", "#104281", "#0d366b"]

# Embedded layout, light/dark palettes, marker classes, and tooltip styles.
CSS = """
:root{color-scheme:light;--surface-1:#fcfcfb;--page:#f9f9f7;--text-primary:#0b0b0b;--text-secondary:#52514e;
--muted:#898781;--grid:#e1e0d9;--axis:#c3c2b7;--border:rgba(11,11,11,.10);
--series-1:#2a78d6;--series-2:#eb6834;--series-3:#1baf7a;--series-4:#eda100;--series-5:#e87ba4;--series-6:#008300;--series-7:#6250d6;--series-8:#e34948;--critical:#d03b3b;--warning:#fab219}
@media (prefers-color-scheme:dark){:root:not([data-theme="light"]){color-scheme:dark;--surface-1:#1a1a19;--page:#0d0d0d;
--text-primary:#f0efec;--text-secondary:#c3c2b7;--grid:#2c2c2a;--axis:#383835;--border:rgba(255,255,255,.10);
--series-1:#3987e5;--series-2:#d95926;--series-3:#199e70;--series-4:#c98500;--series-5:#d55181;--series-6:#0ca30c;--series-7:#9085e9;--series-8:#e66767}}
:root[data-theme="dark"]{color-scheme:dark;--surface-1:#1a1a19;--page:#0d0d0d;--text-primary:#f0efec;--text-secondary:#c3c2b7;
--grid:#2c2c2a;--axis:#383835;--border:rgba(255,255,255,.10);--series-1:#3987e5;--series-2:#d95926;--series-3:#199e70;--series-4:#c98500;--series-5:#d55181;--series-6:#0ca30c;--series-7:#9085e9;--series-8:#e66767}
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
.s1,.s2,.s3,.s4,.s5,.s6,.s7,.s8{stroke:var(--surface-1);stroke-width:1.5}.s5{fill:var(--series-5)}.s6{fill:var(--series-6)}.s7{fill:var(--series-7)}.s8{fill:var(--series-8)}.h5{stroke:var(--series-5)}.h6{stroke:var(--series-6)}.h7{stroke:var(--series-7)}.h8{stroke:var(--series-8)}.s1{fill:var(--series-1)}.s2{fill:var(--series-2)}.s3{fill:var(--series-3)}.s4{fill:var(--series-4)}
.h1,.h2,.h3,.h4,.h5,.h6,.h7,.h8{fill:var(--surface-1);stroke-width:2}.h1{stroke:var(--series-1)}.h2{stroke:var(--series-2)}.h3{stroke:var(--series-3)}.h4{stroke:var(--series-4)}
.ring{fill:none;stroke:var(--text-primary);stroke-width:1.5;pointer-events:none}
.r0{fill:#cde2fb}.r1{fill:#b7d3f6}.r2{fill:#9ec5f4}.r3{fill:#86b6ef}.r4{fill:#6da7ec}.r5{fill:#5598e7}.r6{fill:#3987e5}.r7{fill:#2a78d6}.r8{fill:#256abf}.r9{fill:#1c5cab}.r10{fill:#184f95}.r11{fill:#104281}.r12{fill:#0d366b}
@media (prefers-color-scheme:dark){:root:where(:not([data-theme="light"])) .viz-root .r0{fill:#0d366b}:root:where(:not([data-theme="light"])) .viz-root .r1{fill:#104281}:root:where(:not([data-theme="light"])) .viz-root .r2{fill:#184f95}:root:where(:not([data-theme="light"])) .viz-root .r3{fill:#1c5cab}:root:where(:not([data-theme="light"])) .viz-root .r4{fill:#256abf}:root:where(:not([data-theme="light"])) .viz-root .r5{fill:#2a78d6}:root:where(:not([data-theme="light"])) .viz-root .r6{fill:#3987e5}:root:where(:not([data-theme="light"])) .viz-root .r7{fill:#5598e7}:root:where(:not([data-theme="light"])) .viz-root .r8{fill:#6da7ec}:root:where(:not([data-theme="light"])) .viz-root .r9{fill:#86b6ef}:root:where(:not([data-theme="light"])) .viz-root .r10{fill:#9ec5f4}:root:where(:not([data-theme="light"])) .viz-root .r11{fill:#b7d3f6}:root:where(:not([data-theme="light"])) .viz-root .r12{fill:#cde2fb}}
:root[data-theme="dark"] .viz-root .r0{fill:#0d366b}:root[data-theme="dark"] .viz-root .r1{fill:#104281}:root[data-theme="dark"] .viz-root .r2{fill:#184f95}:root[data-theme="dark"] .viz-root .r3{fill:#1c5cab}:root[data-theme="dark"] .viz-root .r4{fill:#256abf}:root[data-theme="dark"] .viz-root .r5{fill:#2a78d6}:root[data-theme="dark"] .viz-root .r6{fill:#3987e5}:root[data-theme="dark"] .viz-root .r7{fill:#5598e7}:root[data-theme="dark"] .viz-root .r8{fill:#6da7ec}:root[data-theme="dark"] .viz-root .r9{fill:#86b6ef}:root[data-theme="dark"] .viz-root .r10{fill:#9ec5f4}:root[data-theme="dark"] .viz-root .r11{fill:#b7d3f6}:root[data-theme="dark"] .viz-root .r12{fill:#cde2fb}
#tip{position:fixed;pointer-events:none;background:var(--surface-1);color:var(--text-primary);border:1px solid var(--border);
border-radius:6px;padding:6px 8px;font-size:12px;box-shadow:0 2px 8px rgba(0,0,0,.15);display:none;z-index:10;max-width:340px}
#tip .row{display:flex;align-items:center;gap:6px}#tip b{font-variant-numeric:tabular-nums}#tip .name{color:var(--text-secondary)}
#tip .key{display:inline-block;width:12px;height:0;border-top:2px solid}
.toolbar{display:flex;gap:12px;align-items:center;flex-wrap:wrap;margin:8px 0}
button,input{font:inherit;font-size:13px;padding:4px 10px;border:1px solid var(--border);border-radius:6px;background:var(--surface-1);color:var(--text-primary)}
.status{font-size:12px;color:var(--text-secondary)}.toc{columns:3 260px;font-size:13px}.toc a{display:block}
.op[hidden]{display:none}
"""

# Embedded hover, crosshair, theme, and filter behavior; payloads are display summaries.
JS = r"""
// Shared floating tooltip element, populated with text nodes rather than markup.
const tip=document.getElementById('tip');
// Position a tooltip near the pointer, limiting its right/bottom edges to the viewport.
function showTip(e,rows,title){tip.textContent='';if(title){const t=document.createElement('div');t.className='name';t.textContent=title;tip.appendChild(t);}
for(const r of rows){const d=document.createElement('div');d.className='row';if(r.color){const k=document.createElement('span');k.className='key';k.style.borderColor=r.color;d.appendChild(k);}
const b=document.createElement('b');b.textContent=r.value;d.appendChild(b);const n=document.createElement('span');n.className='name';n.textContent=r.name;d.appendChild(n);tip.appendChild(d);}
tip.style.display='block';const x=Math.min(e.clientX+14,window.innerWidth-tip.offsetWidth-8),y=Math.min(e.clientY+14,window.innerHeight-tip.offsetHeight-8);
tip.style.left=x+'px';tip.style.top=y+'px';}
// Hide the shared tooltip without clearing its retained child nodes.
function hideTip(){tip.style.display='none';}
// Round already-quantized nanoseconds to display units; null/nonfinite values show an em dash.
function fmt(ns){if(ns==null||!isFinite(ns))return '—';const u=[[1e9,'s'],[1e6,'ms'],[1e3,'µs'],[1,'ns']];for(const [d,s] of u){if(ns>=d||s==='ns'){const v=ns/d;return (v>=100?v.toFixed(0):v>=10?v.toFixed(1):v.toFixed(2))+' '+s;}}}
// Resolve a series custom property against the current visualization theme.
function col(v){return getComputedStyle(document.querySelector('.viz-root')).getPropertyValue(v).trim();}
// Hover chooses the nearest x coordinate, including rows with missing timings/statuses.
document.querySelectorAll('svg[data-line]').forEach(svg=>{const d=JSON.parse(svg.dataset.d);const cross=svg.querySelector('.cross');
const hit=svg.querySelector('.hit');hit.addEventListener('pointermove',e=>{const pt=svg.createSVGPoint();pt.x=e.clientX;pt.y=e.clientY;const p=pt.matrixTransform(svg.getScreenCTM().inverse());
let best=0,bd=1e9;d.px.forEach((x,i)=>{const dd=Math.abs(x-p.x);if(dd<bd){bd=dd;best=i;}});cross.setAttribute('x1',d.px[best]);cross.setAttribute('x2',d.px[best]);cross.style.display='';
showTip(e,d.series.map(s=>({color:col(s.c),value:fmt(s.ys[best])+(s.st[best]&&s.st[best]!=='ok'&&s.st[best]!=='single'?' ('+s.st[best]+')':''),name:s.name})),d.xl+' = '+d.xs[best]);});
hit.addEventListener('pointerleave',()=>{cross.style.display='none';hideTip();});});
// Convert client coordinates through the inverse SVG screen transform; a valid matrix is assumed.
function local(svg,e){const pt=svg.createSVGPoint();pt.x=e.clientX;pt.y=e.clientY;return pt.matrixTransform(svg.getScreenCTM().inverse());}
// Fuzz hover chooses the nearest drawn point within 24 SVG units; ties keep the earlier point.
document.querySelectorAll('svg[data-f]').forEach(svg=>{const d=JSON.parse(svg.dataset.f);const ring=svg.querySelector('.ring');const hit=svg.querySelector('.hit');
hit.addEventListener('pointermove',e=>{const p=local(svg,e);let best=null,bd=24*24;for(const q of d.p){const dx=q[0]-p.x,dy=q[1]-p.y,dd=dx*dx+dy*dy;if(dd<bd){bd=dd;best=q;}}
if(!best){ring.style.display='none';hideTip();return;}ring.setAttribute('cx',best[0]);ring.setAttribute('cy',best[1]);ring.style.display='';
showTip(e,[{value:fmt(best[2]),name:best[4]||'time per call'}],d.xl+' = '+best[3]+', fuzzed input');});
hit.addEventListener('pointerleave',()=>{ring.style.display='none';hideTip();});});
// Grid hover selects equal-width cells; None displays skipped even when its error flag records a panic.
document.querySelectorAll('svg[data-h]').forEach(svg=>{const d=JSON.parse(svg.dataset.h);const hit=svg.querySelector('.hit');
hit.addEventListener('pointermove',e=>{const p=local(svg,e);const i=Math.floor((p.x-d.x0)/d.w*d.a.length),j=d.b.length-1-Math.floor((p.y-d.y0)/d.h*d.b.length);
if(i<0||j<0||i>=d.a.length||j>=d.b.length){hideTip();return;}const v=d.ns[j][i];
showTip(e,[{value:v==null?'skipped':fmt(v),name:d.e[j]&&d.e[j][i]?'error':'time per call'}],d.al+' = '+d.a[i]+', '+d.bl+' = '+d.b[j]);});
hit.addEventListener('pointerleave',hideTip);});
// Toggle the root theme attribute from its explicit or system-selected initial theme.
document.getElementById('theme').addEventListener('click',()=>{const r=document.documentElement;const dark=r.dataset.theme?r.dataset.theme==='dark':matchMedia('(prefers-color-scheme: dark)').matches;r.dataset.theme=dark?'light':'dark';});
// Filter operation cards by lowercase crate/group/name; headings and the table of contents remain visible.
const q=document.getElementById('filter');q.addEventListener('input',()=>{const s=q.value.toLowerCase();document.querySelectorAll('.op').forEach(o=>{o.hidden=s&&!o.dataset.name.includes(s);});});
"""


def esc(text):
    """Convert a value to text and escape it for an HTML text or quoted-attribute context.

    Escapes ampersands, angle brackets, and both quote characters. It does not
    validate the source value or supply a JavaScript/CSS escaping contract."""
    return html.escape(str(text), quote=True)


def fmt_ns(ns):
    """Format a duration with seconds/ms/µs/ns units and two, one, or zero decimals.

    None and nonfinite values display an em dash. Negative finite values use ns;
    the helper does not reject them. Formatting rounds, so displayed text is not
    a lossless representation of the recorded binary64 duration."""
    if ns is None or not math.isfinite(ns):
        return "—"
    for div, unit in ((1e9, "s"), (1e6, "ms"), (1e3, "µs"), (1, "ns")):
        if ns >= div or unit == "ns":
            v = ns / div
            return (f"{v:.0f}" if v >= 100 else f"{v:.1f}" if v >= 10 else f"{v:.2f}") + " " + unit


def fmt_mag(m):
    """Format a magnitude, abbreviating exact integer multiples of binary K/M/G units.

    None displays an em dash. Other integers print directly; nonintegers use
    Python's general floating format. Requires a finite numeric value: NaN/infinity
    can fail during int conversion. Abbreviations do not change the axis data."""
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
    """Map numeric values to pixels in linear or natural-log coordinates.

    Finite non-None construction values choose the domain; an empty set defaults
    to one. Automatic log selection requires positivity and a max/min ratio of at
    least sixteen. Calling the object neither filters nor clamps a query, so its
    domain requirements still apply to every plotted value."""
    def __init__(self, values, lo_px, hi_px, log=None):
        """Choose transformed data bounds and retain the supplied pixel endpoints.

        An explicit log flag overrides automatic selection and can fail on nonpositive
        bounds. Equal transformed bounds expand by one on each side. Pixel endpoints
        can descend, as for vertical time axes; they are not validated."""
        vals = [v for v in values if v is not None and math.isfinite(v)]
        if not vals:
            vals = [1.0]
        positive = all(v > 0 for v in vals)
        # Explicit flag or positivity/span-based choice of transformed coordinates.
        self.log = (positive and max(vals) / min(vals) >= 16) if log is None else log
        # lo/hi store transformed data bounds, expanded below for a constant domain.
        if self.log:
            self.lo, self.hi = math.log(min(vals)), math.log(max(vals))
        else:
            self.lo, self.hi = min(vals), max(vals)
        if self.hi == self.lo:
            self.lo -= 1
            self.hi += 1
        # Pixel endpoints remain in caller order, including reversed vertical axes.
        self.lo_px, self.hi_px = lo_px, hi_px

    def __call__(self, v):
        """Interpolate or extrapolate the query in the selected coordinates without clipping.

        Log mode requires a positive numeric query. None and incompatible types raise;
        nonfinite values can propagate to coordinates. This is a display mapping,
        not an enclosure or a numeric-certification step."""
        t = ((math.log(v) if self.log else v) - self.lo) / (self.hi - self.lo)
        return self.lo_px + t * (self.hi_px - self.lo_px)


def time_scale(values):
    """Create an always-logarithmic time axis padded to whole powers of ten.

    Uses finite positive values or a default 1..10 ns domain, and returns every
    integer-decade tick. Values near floating limits can underflow the padded lower
    bound or overflow integer-to-float conversion at the upper bound. Filtering here
    does not validate the values later passed to the returned Scale."""
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
    """Sort/deduplicate non-None magnitudes and retain at most nine representative ticks.

    Keeps all values up to nine, otherwise strides by ceil(count/8) and adds the
    last value if needed. The scale argument is unused; tick density is based on
    count, not pixel spacing. Numeric finiteness/comparability is assumed."""
    mags = sorted(set(m for m in mags if m is not None))
    if len(mags) <= 9:
        return mags
    step = math.ceil(len(mags) / 8)
    return mags[::step] + ([mags[-1]] if mags[-1] not in mags[::step] else [])


def marker(shape, x, y, var, hollow=False, size=4.5):
    """Return a filled or hollow SVG marker using a trusted --series-N CSS token.

    Supports square, triangle, and diamond; every other shape selects a circle.
    Coordinates are formatted to one decimal, while size/radius formatting varies
    by shape. The token suffix is interpolated into a class without escaping and
    size/coordinate validity is not checked."""
    slot = var.rsplit("-", 1)[-1]
    cls = f'class="{"h" if hollow else "s"}{slot}"'
    if shape == "square":
        return f'<rect {cls} x="{x - size:.1f}" y="{y - size:.1f}" width="{2 * size:.1f}" height="{2 * size:.1f}" rx="1"/>'
    if shape == "triangle":
        return f'<path {cls} d="M{x:.1f},{y - size * 1.2:.1f}l{size * 1.1:.1f},{size * 2:.1f}h{-size * 2.2:.1f}z"/>'
    if shape == "diamond":
        return (f'<path {cls} d="M{x:.1f},{y - size * 1.2:.1f}l{size * 1.2:.1f},{size * 1.2:.1f}'
                f'l{-size * 1.2:.1f},{size * 1.2:.1f}l{-size * 1.2:.1f},{-size * 1.2:.1f}z"/>')
    return f'<circle {cls} cx="{x:.1f}" cy="{y:.1f}" r="{size}"/>'


def sig(v):
    """Round finite values to three significant decimal figures for embedded tooltip data.

    None and nonfinite values become None. This deliberately loses precision before
    browser formatting and must not be described as an exact-value display."""
    return None if v is None or not math.isfinite(v) else float(f"{v:.3g}")


def axes(xs, ys, xticks, yticks, xlabel):
    """Build tick/grid SVG fragments using the supplied scales and an escaped x-axis label.

    The time-axis title always says log scale; callers provide a logarithmic y scale.
    Tick values are formatted for display. Coordinates are not clipped, and no
    independent validation of tick domains or scale consistency occurs."""
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


def why(p):
    """Return a truthy optional seventh point element, otherwise None.

    Accepts older six-element records but does not validate the reason's type.
    Callers that concatenate it as text assume a string."""
    return p[6] if len(p) > 6 and p[6] else None


def anomalies(points):
    """Find ratio changes among retained successful rows and collect ordinary error/stop statuses.

    Cliffs compare successive non-None ok/single rows in input order, even across
    intervening failed or missing rows; valleys use successive triples of that same
    filtered list. Positive-denominator checks guard the ratio tests. Error rows
    are collected separately; ceiling and predicted share the ceiling category.
    Skipped rows are ignored. No sorting, finiteness, workload-equivalence, or
    statistical-significance check is performed."""
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
        if p[5] in ("ceiling", "predicted"):
            found.append(("ceiling", 0, p, p))
    return found


def line_chart(op, series, xlabel, slots):
    """Draw one panel's ordinary series and embed rounded hover data in an escaped attribute.

    Time is logarithmic; magnitude uses Scale's automatic rule. Lines retain source
    order and connect non-error measured rows across gaps, including ceiling rows.
    Measured errors get hollow markers; errors with no duration have no marker.
    Only the two largest detected cliffs get inline annotations. Direct labels can
    be shifted downward for spacing without clipping to the plot bounds.

    Hover magnitudes are sorted/unique and duplicate rows resolve to the last row,
    although table uses the first. slots supplies trusted colour indices, and op/name
    and user-facing labels are escaped. Finite magnitudes and positive timings are
    caller preconditions; partial filtering here does not validate the whole panel."""
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
                               "ys": [sig(by_x[x][1]) if x in by_x else None for x in xvals],
                               "st": [(by_x[x][5] + (": " + why(by_x[x]) if why(by_x[x]) else "")) if x in by_x else None for x in xvals]})
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
            short_name = name if len(name) <= 17 else name[:16] + "…"
            out.append(f'<text class="direct" x="{ML + PW + 8}" y="{y + 4:.1f}">{esc(short_name)}</text>')
    out.append(f'<line class="cross axisline" x1="0" x2="0" y1="{MT}" y2="{MT + PH}" style="display:none"/>')
    out.append(f'<rect class="hit" x="{ML}" y="{MT}" width="{PW}" height="{PH}" fill="transparent"/>')
    out.append("</svg>")
    return "".join(out).replace("__DATA__", esc(json.dumps(data)), 1)


def legend(kinds, slots):
    """Build escaped labels and CSS markers for at least two kinds, otherwise return empty text.

    Uses the supplied kind-to-slot mapping; repeated labels reuse its final slot.
    Shape cycles every four slots and the caller normally cycles colours every eight."""
    if len(kinds) < 2:
        return ""
    items = []
    for k in kinds:
        slot = slots[k]
        items.append(f'<span><svg viewBox="0 0 20 10"><line x1="0" x2="20" y1="5" y2="5" stroke="var(--series-{slot + 1})" stroke-width="2"/>'
                     f'{marker(SHAPES[slot % 4], 10, 5, f"--series-{slot + 1}", size=3.2)}</svg>{esc(k)}</span>')
    return f'<div class="legend">{"".join(items)}</div>'


def fuzz_chart(op, series, xlabel):
    """Plot all non-None sample durations with deterministic horizontal jitter per magnitude.

    Failed and ceiling timings remain in the plotted data and in each group's upper
    median; unmeasured panic rows disappear. Errors are hollow. The dashed median
    path is drawn only with multiple magnitudes. Tooltip timings are rounded and
    carry coarse error/ceiling labels. No correctness or statistical inference is made."""
    pts = [p for p in series["points"] if p[1] is not None]
    xs = Scale([p[0] for p in pts], ML, ML + PW)
    ys, yt = time_scale([p[1] for p in pts])
    out = [f'<svg viewBox="0 0 {W} {H}" role="img" aria-label="{esc(op["name"])}: fuzzed inputs" data-f="__DATA__">']
    out += axes(xs, ys, x_ticks([p[0] for p in pts], xs), yt, xlabel)
    groups = {}
    for p in pts:
        groups.setdefault(p[0], []).append(p)
    mags = sorted(groups)
    spacing = (PW / max(len(mags) - 1, 1)) * 0.25
    pdata = []
    for m in mags:
        group = groups[m]
        for i, p in enumerate(group):
            jitter = (i / max(len(group) - 1, 1) - 0.5) * spacing if len(group) > 1 else 0
            x, y = xs(m) + jitter, ys(p[1])
            out.append(marker("circle", x, y, "--series-1", hollow=p[5] == "error", size=4))
            label = ("error: " + (why(p) or "failed")) if p[5] == "error" else ("ceiling" if p[5] == "ceiling" else "")
            pdata.append([round(x, 1), round(y, 1), sig(p[1]), fmt_mag(m), label])
    med = []
    for m in mags:
        v = sorted(p[1] for p in groups[m])
        med.append((m, v[len(v) // 2]))
    if len(med) > 1:
        d = "M" + "L".join(f"{xs(m):.1f},{ys(v):.1f}" for m, v in med)
        out.append(f'<path d="{d}" fill="none" stroke="var(--text-secondary)" stroke-width="1.5" stroke-dasharray="4 3"/>')
        out.append(f'<text class="direct" x="{ML + PW + 8}" y="{ys(med[-1][1]) + 4:.1f}">median</text>')
    out.append('<circle class="ring" r="8" style="display:none"/>')
    out.append(f'<rect class="hit" x="{ML - 12}" y="{MT - 12}" width="{PW + 24}" height="{PH + 24}" fill="transparent"/></svg>')
    return "".join(out).replace("__DATA__", esc(json.dumps({"xl": xlabel, "p": pdata}, separators=(",", ":"))), 1)


def heatmap(op, grid):
    """Draw row-major durations with equal-sized cells and logarithmic colour bins.

    Returns empty text if every duration is None, even if error flags are set.
    Requires nonempty axes, matching matrix shapes, and positive finite durations
    where present; these preconditions are not validated. Missing/empty error data
    defaults to false. Non-None errors get crosses, but a None/error cell only gets
    an empty outline, so missing cells are not necessarily skipped calls.

    Rows increase upward. Colour classes use the count of RAMP entries, not their
    stored colour strings; CSS reverses the palette in dark mode. Hover data rounds
    durations, and the data table separately omits error flags."""
    a, b, ns = grid["a"], grid["b"], grid["ns"]
    errors = grid.get("errors") or [[False] * len(a) for _ in b]
    vals = [v for row in ns for v in row if v is not None]
    if not vals:
        return ""
    lo, hi = math.log(min(vals)), math.log(max(vals))
    span = (hi - lo) or 1.0
    cw, ch = PW / len(a), PH / len(b)
    hdata = {"al": grid["a_label"], "bl": grid["b_label"], "a": [fmt_mag(x) for x in a], "b": [fmt_mag(y) for y in b],
             "ns": [[sig(v) for v in row] for row in ns], "e": errors, "x0": ML, "y0": MT, "w": PW, "h": PH}
    out = [f'<svg viewBox="0 0 {W} {H}" role="img" aria-label="{esc(op["name"])}: {esc(grid["kind"])} grid" '
           f'data-h="{esc(json.dumps(hdata, separators=(",", ":")))}">']
    for j, y in enumerate(b):
        cy = MT + PH - (j + 1) * ch
        for i, x in enumerate(a):
            v = ns[j][i]
            cx = ML + i * cw
            if v is None:
                out.append(f'<rect x="{cx + 1:.1f}" y="{cy + 1:.1f}" width="{cw - 2:.1f}" height="{ch - 2:.1f}" fill="none" stroke="var(--grid)"/>')
                continue
            t = (math.log(v) - lo) / span
            step = round(min(max(t, 0.0), 1.0) * (len(RAMP) - 1))
            out.append(f'<rect class="r{step}" x="{cx + 1:.1f}" y="{cy + 1:.1f}" width="{cw - 2:.1f}" height="{ch - 2:.1f}" rx="2"/>')
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
        out.append(f'<rect class="r{k}" x="{kx}" y="{yk:.1f}" width="14" height="{PH / len(RAMP):.1f}"/>')
    out.append(f'<text class="ticklabel" x="{kx + 18}" y="{MT + 10}">{esc(fmt_ns(max(vals)))}</text>')
    out.append(f'<text class="ticklabel" x="{kx + 18}" y="{MT + PH}">{esc(fmt_ns(min(vals)))}</text>')
    out.append(f'<rect class="hit" x="{ML}" y="{MT}" width="{PW}" height="{PH}" fill="transparent"/>')
    out.append("</svg>")
    return "".join(out)


def table(op):
    """Build one operation's collapsible summary table collection.

    Ordinary series use sorted unique magnitudes and the first matching row, unlike
    hover's last-row selection. They show formatted median/status, not min/max or
    iteration counts; a missing-duration error omits its reason. Fuzz tables aggregate
    all measured durations, including errors/ceilings, without their statuses. Grid
    tables show durations and omit error flags. These summaries are not raw-data exports."""
    parts = []
    if op["series"]:
        mags = sorted(set(p[0] for s in op["series"] for p in s["points"] if p[0] is not None))
        head = "".join(f"<th>{esc(s['kind'])}</th>" for s in op["series"])
        rows = []
        for m in mags:
            cells = []
            for s in op["series"]:
                p = next((p for p in s["points"] if p[0] == m), None)
                text = "—" if p is None else (p[5] if p[1] is None else fmt_ns(p[1]) + ("" if p[5] in ("ok", "single") else f" ({p[5]}{': ' + why(p) if why(p) else ''})"))
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
    """Make a lowercase operation id from crate, group, and name, replacing nonalphanumerics with hyphens.

    Unicode alphanumerics survive. Distinct inputs can collide after replacement or
    case folding; no uniqueness check or disambiguating suffix is added."""
    return "op-" + "".join(c if c.isalnum() else "-" for c in f"{crate}-{op['group']}-{op['name']}").lower()


def render(docs):
    """Assemble a complete report from trusted document mappings in their supplied order.

    Uses four ordinary kinds per chart panel and cycles eight colour slots. Only
    ordinary series contribute highlights: the largest forty cliffs, twenty valleys,
    and first forty errors/ceilings are listed, with counts for omitted rows. Fuzz and
    grid summaries are rendered separately. Duplicate names can generate duplicate ids.

    Most labels are escaped, but some top-level harness metadata is interpolated
    without escaping; this is not a safe renderer for arbitrary untrusted documents.
    No cross-document hardware, settings, schema, or semantic-equivalence validation
    occurs. Assembly retains the complete report and intermediate strings in memory."""
    highlights = []
    body = []
    toc = []
    for doc in docs:
        crate = doc["crate"]
        toc.append(f'<a href="#crate-{esc(crate)}"><b>{esc(crate)}</b> — {len(doc["operations"])} operations</a>')
        body.append(f'<h2 id="crate-{esc(crate)}">{esc(crate)}</h2>')
        body.append(f'<p>{"Quick" if doc["quick"] else "Full"} sweep, {doc["os"]}/{doc["arch"]}, seed {doc["seed"]}, '
                    f'{doc["elapsed_s"]:.0f} s. Target batch duration {fmt_ns(doc["settings"]["target_batch_ns"])}, '
                    f'{doc["settings"]["samples"]} maximum samples per ordinary point, initial-call ceiling {fmt_ns(doc["settings"]["single_ceiling_ns"])}, '
                    f'{doc["settings"]["fuzz_per_magnitude"]} maximum fuzzed inputs per magnitude.</p>')
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
                    charts.append(f'<div class="chart"><div class="caption">{esc(g["kind"])}: median time per call over both operand sizes (colour shows relative cost)</div>{h}</div>')
            body.append(f'<div class="card op" id="{anchor(crate, op)}" data-name="{esc((crate + " " + op["group"] + " " + op["name"]).lower())}">'
                        f'<h3>{esc(op["name"])}</h3><p>{esc(op["description"])}</p><div class="charts">{"".join(charts)}</div>{table(op)}</div>')
    cliffs = sorted((h for h in highlights if h[0] == "cliff"), key=lambda h: -h[1])
    valleys = sorted((h for h in highlights if h[0] == "valley"), key=lambda h: -h[1])
    failures = [h for h in highlights if h[0] in ("error", "ceiling")]

    def rows(items, limit, what):
        """Format at most limit supplied highlight records, retaining their caller-selected order.

        Uses anomaly-specific magnitude/detail wording and escaped text, with a summary
        when further entries are omitted. Empty input gets a no-findings paragraph.
        Does not independently rank or verify the anomaly records."""
        out = []
        for kind, ratio, crate, op, series, a, b in items[:limit]:
            where = f"{fmt_mag(a[0])} → {fmt_mag(b[0])}" if kind == "cliff" else fmt_mag(a[0])
            detail = (f"×{ratio:.1f}" if kind in ("cliff", "valley") else ("call failed" + (f": {why(a)}" if why(a) else "")) if kind == "error"
                      else "predicted over ceiling; not run" if a[5] == "predicted" else "single call over ceiling")
            out.append(f'<tr><td><a href="#{anchor(crate, op)}">{esc(crate)} · {esc(op["name"])}</a></td><td>{esc(series)}</td>'
                       f'<td>{esc(where)}</td><td>{esc(fmt_ns(a[1]))}</td><td>{esc(fmt_ns(b[1]))}</td><td>{esc(detail)}</td></tr>')
        if not out:
            return f"<p>No {what} found.</p>"
        return ("<table><tr><th>Operation</th><th>Input kind</th><th>Magnitude</th><th>Time</th><th>Compared with</th><th>Size</th></tr>"
                + "".join(out) + "</table>" + (f"<p>{len(items) - limit} more not shown.</p>" if len(items) > limit else ""))

    head = f"""<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Performance atlas</title><style>{CSS}</style></head><body><div class="viz-root"><div id="tip"></div>
<h1>Performance atlas</h1>
<p>Wall-clock time per call across input recipes and magnitudes. Line charts always use logarithmic time;
magnitude is logarithmic when all finite values are positive and span at least 16×. A straight line on log-log axes can
suggest a power law over the measured range, but does not prove an algorithm's complexity.</p>
<p>A <b>cliff</b> is a rise of at least {CLIFF:.0f}× between successive retained successful rows; a <b>valley</b>
is at least {VALLEY:.0f}× faster than both such neighbours. These comparisons can cross failed or missing rows.
Highlight tables cover ordinary series only. Hollow markers show measured error rows; missing-duration failures have no marker.
"Ceiling" records an initial call that finished over its threshold; "predicted" means the magnitude was not run.</p>
<p>Statuses describe the initial probe, not every repeated call or an independent correctness check.
Input factories, cache reuse, and required output materialization depend on the producing workload.
Hover timings are rounded; tables are summaries. Retain the JSON and compare equivalent completed work
under matched host, toolchain, and settings.</p>
<div class="toolbar"><input id="filter" placeholder="Filter operations" aria-label="Filter operations"><button id="theme">Toggle dark mode</button>
<span class="status">{sum(len(d["operations"]) for d in docs)} operations in {len(docs)} crates, {len(cliffs)} cliffs, {len(valleys)} valleys, {len(failures)} failures or ceilings</span></div>
<div class="card"><h3>Crates</h3><div class="toc">{"".join(toc)}</div></div>
<div class="card"><h3>Largest cliffs</h3>{rows(cliffs, 40, "cliffs")}</div>
<div class="card"><h3>Valleys</h3>{rows(valleys, 20, "valleys")}</div>
<div class="card"><h3>Failures and ceilings</h3>{rows(failures, 40, "failures or ceilings")}</div>
"""
    return head + "".join(body) + f"<script>{JS}</script></div></body></html>\n"


def main():
    """Load command-line JSON files, sort by crate label, and write OUTPUT_DIR/index.html.

    Requires an output directory plus at least one input; otherwise exits with the
    module usage text. Asserts perf-atlas/1 tags (assertions can be disabled), but
    other fields are trusted. File/JSON/render failures propagate. Creates parent
    directories and writes directly after rendering, without atomic replacement;
    write failure can leave partial output and an earlier render failure leaves any
    existing index untouched. Duplicate input files are not rejected."""
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
