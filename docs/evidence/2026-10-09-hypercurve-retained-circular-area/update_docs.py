"""Publish scoped evidence after the validation/cost/report admission record exists."""
import json
from pathlib import Path
record=Path(__file__).resolve().parent
m=json.loads((record/'manifest.json').read_bytes())
link='docs/evidence/2026-10-09-hypercurve-retained-circular-area/manifest.json'
v=m['validation']['tests'];cost=m['cost'];cap=m['capability_cost']
note=(f"Retained circular fragments and their adjacent algebraic chords now share exact native line/arc lowering for area and contour views. The original full-scale CPArti A5/A6 overlap has a certified integral and native view. Quadratic endpoint projection strictly replays the selected root and leaves the general scalar-witness cache untouched; a broader implicit promotion was rejected after real chord/offset regressions. Similarity transforms preserve the retained carriers with cold and warmed native views; the complete failing run and source archive are retained. Eight new regressions cover branch selection, negative scales, repeated/nested roots, cache-independent field queries, closed-form caps, reversal, degree-five sources, major arcs and typed blockers. Validation passes {v['curve-tests']['passed']} curve tests ({v['curve-tests']['ignored']} explicitly ignored), {v['drc-tests']['passed']} DRC tests and 14 selected consumers, plus strict lints/docs and the WASM demo build. The demo has 35 passing tests and two existing restricted-fragment editability failures, independently reproduced with identical diagnostics on the committed baseline; those gates remain failed. All 231 previous cost witnesses and complete two-board report bytes are preserved apart from the documented version fields. General integration, higher moments, arbitrary exterior traversal, total resource bounds and all remaining plan/future-work scope stay open. See {link}.")
p=Path('docs/properties-status.json');d=json.loads(p.read_bytes())
for item in d['properties']:
    if item['id'] in ['P4','P12','P14','P16','P17','P18','P19','P20','P27','P35','P36','P39']:
        assert link not in item['evidence']
        item['evidence'].append(link)
        item['remaining']+=' '+note
        item['status']='in-progress'
for item in d['targets']:
    if item['id'] in ['T6','T9','T11','T13']:
        item['evidence'].append(link);item['remaining']+=' '+note
for item in d['future_work']['sources']:
    if item['path']=='crates/hypercurve/FUTURE_WORK.md':
        item['remaining']='Entire document read on 2026-10-09. All seven remaining directions (radical signs, generated representation, selected-fiber root counts, S6 promotion, S5/S7 consolidation, public restricted fragments, independent-radical contacts), ignored compositions, and measured-negative constraints remain in scope. '+note
for i,action in enumerate(d['next_actions']):
    if 'Resolve retained circular algebraic-fragment area integration using the original CPArti' in action:
        d['next_actions'][i]=action.replace('Resolve retained circular algebraic-fragment area integration using the original CPArti A5/A6 pair; keep the optional exact duplicate-overlap cap and the unavailable-bound fallback.', 'Preserve the now-certified original CPArti A5/A6 area/native-view replay and the optional duplicate-overlap cap with its unavailable-bound fallback. Continue general retained integration, higher moments, composition and resource bounds; the native endpoint bridge must remain isolated from general selected-field projection.')
p.write_text(json.dumps(d,indent=2)+'\n')

entry=f'''\n## Retained circular endpoint projection (2026-10-09)\n\nThe [admission record](../../{link}) adds exact signed-area and native-contour
views for retained circular fragments with representable endpoints. The source
curve, selected polynomial, isolator and traversal direction remain authoritative.
The original complete CPArti board's A5/A6 overlap now has an exact integral.
Independent cap, semicircle and major-arc identities cover reversal and degree-five
elevation. Exterior traversal without a certificate remains unsupported.
\n**Rejected:** making every optional root projection reconstruct quadratic surds
broke independent chord/offset field operations. The failed suite and exact
source archive are retained. The accepted bridge is local to native line/arc
endpoints and never fills the general scalar-witness cache. A dedicated regression
requests the projection before an independent-field chord query. The existing
native contour/area caches retain completed views. A complete curve run then
exposed one similarity-transform regression: rebuilding from the new native view
discarded retained chord fields and changed later round-offset carriers. Similarity
transforms now preserve retained algebraic carriers directly. The existing test
covers cold and warmed native views under both policies without weakening its
family or point-classification assertions; the rejected run is archived.
\nThe curve library suite passes {v['curve-tests']['passed']} tests, with
{v['curve-tests']['ignored']} existing ignored cases, strict all-feature/all-target
Clippy and docs. The demo has {v['demo-tests']['passed']} passing tests and two
existing restricted-fragment editability failures, reproduced with identical
diagnostics on the isolated committed baseline. Strict demo Clippy and the
release WASM build pass. The two demo tests remain failed pre-push checks.
Downstream validation passes {v['drc-tests']['passed']} DRC tests and 14 selected
consumer tests. Six snapshots and the complete full-board report replays change
only their documented implementation-version fields (18 to 19).
\nFive sequential CPU-0-pinned capability trials use the unchanged full-board pair:
\n| Query | Median, µs |
| --- | ---: |
'''
for name,value in cap['medians_ns'].items():entry+=f'| `{name}` | {value/1000:.3f} |\n'
entry+=f'''
All 231 previous operation rows, inputs, iteration counts and exact witnesses
remain unchanged ({cost['rows']['bytes']} bytes, SHA-256
`{cost['rows']['sha256']}`). The full fixture takes {cost['fixture_seconds']:.6f}
seconds against its unchanged 120-second gate, with {cost['fixture_peak_rss_kib']:,}
KiB peak RSS. The new capability executable is {cap['binary']['bytes']:,} bytes;
process RSS spans {cap['peak_rss_kib'][0]:,}–{cap['peak_rss_kib'][1]:,} KiB and includes
board parsing outside the timers. These are capability costs, with no matched
speedup, allocation-count, or library-size reduction claim.
\nGeneral retained integration and higher moments, complete composition coverage,
all ignored workloads, and total resource bounds remain open.
'''
p=Path('crates/hypercurve/PERFORMANCE.md');p.write_text(p.read_text()+entry)
p=Path('crates/hypercurve/FUTURE_WORK.md');p.write_text(p.read_text()+f'''
## Native circular endpoint projection follow-up (2026-10-09)

The [retained circular-area record](../../{link}) certifies the original full
CPArti A5/A6 overlap and adds shared native arc lowering for retained endpoints.
The narrow quadratic view strictly replays its selected root and does not enter
the general scalar-witness cache. General implicit projection was rejected after
chord/offset field regressions; the failing sources and logs remain available.
Similarity transforms also preserve original retained carriers after native-view
projection, with the existing round-offset regression extended to cold/warm views.
The full demo still has two restricted-fragment editability failures; an isolated
committed baseline reproduces both with identical diagnostics. They remain failed
validation gates and open work. This does not close sections 1–7, public
control-point access for general restricted fragments, retained higher moments,
exterior traversal, or the ignored compositions.
''')
p=Path('crates/hyperdrc/PERFORMANCE.md');p.write_text(p.read_text()+f'''
## Original rounded-pad overlap integral (2026-10-09)

The shared Hypercurve [admission record](../../{link}) now evaluates the exact
area of the full-board CPArti A5/A6 intersection (copper indices 1723/1724) and
exposes its native contour view. The independent rounded-pad area, overlap-box
cap and certified non-duplicate result remain unchanged. The optional duplicate
cap and unavailable-bound fallback remain in place.

Validation passes {v['drc-tests']['passed']} DRC tests and 14 selected consumers,
with strict lints/docs and all six version-only snapshot updates. Five trials
preserve all 231 previous cost rows. The unchanged fixture passes in
{cost['fixture_seconds']:.6f} seconds. Two fresh processes repeat each complete
full-board report exactly; only twelve version fields per board change from 18
to 19 against the prior record. Broader parser/native-source conversion, remaining
readiness consumers and general integration/resource closure remain open.
''')
print('Updated scoped ledgers; no property or broad target marked complete.')
