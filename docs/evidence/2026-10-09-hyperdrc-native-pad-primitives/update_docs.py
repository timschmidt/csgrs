"""Publish the verified checkpoint in the existing repository ledgers."""
import json
from pathlib import Path
import re

record = Path(__file__).resolve().parent
load = lambda p: json.loads(p.read_bytes())
cost = load(record/'cost/manifest.json')
assert cost['source_unchanged'] and cost['fixture_exit_code'] == 0
assert load(record/'cost/row-audit.json')['status'] == 'passed'
assert load(record/'report-replay/baseline-delta-audit.json')['status'] == 'passed'
assert load(record/'area-bound/manifest.json')['status'] == 'passed'
assert all(s['exit_code'] == 0 for s in load(record/'validation/status.json'))
fixture = (record/'cost/fixture.log').read_text()
seconds = float(re.search(r'fixture_smoke_total ms=([\d.]+)',fixture)[1])/1000
peak = int(re.search(r'Maximum resident set size \(kbytes\): (\d+)',fixture)[1])
rss = [row['peak_rss_kib'] for row in cost['observations']]
table = '\n'.join(f"| {label} | {cost['medians_ns']['operation_pad_'+name]/1000:.3f} |" for label,name in [
    ('Rounded construction, four samples/corner','round_construct_samples_4'),
    ('Rounded construction, 128 samples/corner','round_construct_samples_128'),
    ('Rounded construction, original x=1e20 scale','round_construct_original_large'),
    ('Rounded construction, subnormal dimensions','round_construct_subnormal'),
    ('Rounded construction, unavailable finite corners','round_construct_beyond_float'),
    ('Chamfer construction, selected corners','chamfer_construct_selected'),
    ('Chamfer construction, original x=1e20 scale','chamfer_construct_original_large'),
    ('Trapezoid construction, half-delta convention','trapezoid_construct_half_delta'),
    ('Trapezoid construction, limiting triangle','trapezoid_construct_triangle'),
    ('Trapezoid construction, rejected excessive delta','trapezoid_invalid_delta'),
    ('Rounded warm area, four samples/corner','round_warm_area_samples_4'),
    ('Rounded warm bounds, four samples/corner','round_warm_bounds_samples_4'),
])
section = f'''## Native pad primitive construction, 2026-10-09

Rounded and chamfered rectangles and trapezoids now import finite source
parameters before half-size arithmetic, clamping, corner construction, rotation
and translation. Rounded corners remain native circular arcs independently of
report sample count. Native clamping preserves limiting capsules, circles,
chamfer diamonds and subnormal dimensions. Invalid inputs and excessive
trapezoid deltas retain failures; valid zero dimensions denote empty geometry.
Construction and warm queries retain STRICT and shared-budget guards.

Trapezoids use half the opposing-edge delta and the corrected lower-edge signs,
following the [primary KiCad source references](../../docs/evidence/2026-10-09-hyperdrc-native-pad-primitives/reference-sources.json).
The finite API imports binary64 values exactly; it does not imitate KiCad's
integer-unit truncation. Report drawings stay separate from native geometry,
including when finite corners collapse or cannot be represented. The original
x=1e20, size 16384 case remains unchanged for all three shape families.

The [checkpoint record](../../docs/evidence/2026-10-09-hyperdrc-native-pad-primitives/manifest.json)
retains 16 failing baseline tests and four passing controls against `7f922aca6`,
with all 138 baseline source hashes reconstructed. Thirty-three new regressions
cover 28 primitive cases, three KiCad handoffs and two overlap-bound cases.
The final release gate passes 1,561 tests: 1,556 library, two README and three
doctests. Strict all-feature/all-target Clippy, strict rustdoc, formatting and
fixture build pass. HyperCircuit passes strict Clippy and 14 selected tests.
Each of six Easyduino snapshots changes only 160 version fields from 17 to 18.
All 141 DRC Rust sources remain fixed through the final validation and replay.

The initial 1,559-test green suite missed a completeness regression on the full
CPArti board. The J1 A5/A6 rounded-pad intersection retains exact circular
algebraic fragments whose area integral is unavailable. Duplicate detection now
uses the intersection of native bounding boxes as an optional area upper bound:
if it cannot contain enough area to meet either existing duplicate tolerance,
the pair is certified non-duplicate. Unavailable bounds or comparisons retain
the required intersection path. Making this bound mandatory initially failed an
existing optional-marker regression (1,555 passed, one failed); that run and its
exact source bytes are preserved. The corrected focused gate includes all
24 bounds regressions.

The [original-pair replay](../../docs/evidence/2026-10-09-hyperdrc-native-pad-primitives/area-bound/manifest.json)
links both verified libraries against the same full board. Its native geometry
and unavailable integral remain unchanged while the duplicate decision becomes
certified. An independent rational reconstruction of the input boxes, together
with the strict lower bound pi > 3 for rounded area, proves the cap is below the
existing required overlap. General retained circular-fragment integration is
still open; the check no longer requires it for this decidable pair.

Five fresh sequential release processes pinned to CPU 0 produce 231 identical
correctness rows. All 205 previous inputs, names, iteration counts and complete
witnesses remain unchanged. Twenty new construction rows and six warm queries
carry independent exact area/bounds assertions and input/report fingerprints.
The identity artifact contains {cost['rows']['bytes']:,} bytes, SHA-256
`{cost['rows']['sha256']}`.

| Query | Median µs/query |
| --- | ---: |
{table}

First/last assertions are outside timers; construction rows include native
construction. Process peak RSS spans {min(rss):,}–{max(rss):,} KiB and includes
earlier million-sample geometry. These are capability costs without a matched
speedup, allocation-count or library-size claim. The unchanged full two-board
and Gerber fixture passes in {seconds:.3f} seconds against its 120-second gate,
with {peak:,} KiB peak RSS. Its nine ordered check counts and rendered findings
remain identical after the recorded temporary-path/ID normalization.

Complete reports for twelve selected checks repeat byte-for-byte across two
fresh processes on both full boards at the existing fixed paths and with the
same synthetic seven-record IPC sidecar. CPArti retains 2,610 findings and
HVP109A retains 658. Each report changes only twelve version fields from 17 to
18: every finding, marker, coordinate, ID and other serialized byte remains
unchanged. The rejected initial reports and their failed comparison remain in
`pre-area-fix`; they are not accepted as final evidence. This does not establish
universal or path-independent report identity.

Parser ratio products, malformed optional tokens, combined chamfer/rounding
intent, finite placement, circle/oval size arithmetic, line normalization and
sampled arc/Bezier strokes remain open. Forty legacy copper-index sites,
remaining shape/drill/distribution consumers, scalar thresholds, report-chord
widths and density proxies still require audit. General scalar identities,
retained circular integration and total resource/report-allocation bounds are
not established. All 39 properties, 14 targets and crate future work remain in
scope. No push or complete repository pre-push workflow matrix was run.

'''
p = Path('crates/hyperdrc/PERFORMANCE.md'); text = p.read_text()
heading = section.splitlines()[0]
if heading in text:
    start = text.index(heading); end = text.index('## Retained optimization',start)
    text = text[:start]+text[end:]
assert text.count('## Retained optimization') == 1
p.write_text(text.replace('## Retained optimization',section+'## Retained optimization'))

p = Path('docs/properties-status.json'); status = load(p)
evidence = 'docs/evidence/2026-10-09-hyperdrc-native-pad-primitives/manifest.json'
summary = ('Native rounded/chamfered rectangle and trapezoid APIs now construct exact boundaries before finite corner arithmetic. Rounded report samples cannot decide geometry; trapezoids use the KiCad half-delta convention. An optional exact overlap cap certifies the original CPArti rounded-pad pair while retaining required intersection fallback for unavailable bounds. The record preserves 16 baseline failures, the initial full-board integral regression and the mandatory-bound regression. Final validation passes 1,561 DRC and 14 selected consumer tests; six snapshots change only version fields. Five cost trials preserve 231 rows, including all 205 prior witnesses. Both complete full-board reports repeat exactly and change only twelve version fields each. Parser/source conversion, retained circular integrals and broader resource/completeness work remain open. See '+evidence+'.')
for group,ids in [('properties',{'P1','P4','P6','P8','P11','P16','P26','P27','P29','P30','P35','P36','P39'}),('targets',{'T6','T9','T11','T13'})]:
    for row in status[group]:
        if row['id'] in ids and evidence not in row['evidence']:
            row['evidence'].append(evidence)
            row['remaining'] += ' '+summary
for row in status['future_work']['sources']:
    if row['path'] == 'crates/hyperdrc/docs/design-readiness-plan.md' and evidence not in row['remaining']:
        row['remaining'] += ' '+summary
    if row['path'] == 'crates/hypercurve/FUTURE_WORK.md' and evidence not in row['remaining']:
        row['remaining'] += ' The full CPArti rounded-pad overlap retains unsupported circular algebraic-fragment area integration. Its exact original-pair replay and independent duplicate-rejection cap are retained at '+evidence+'; general native circular lowering and integration remain open.'
for i,action in enumerate(status['next_actions']):
    if action.startswith('Continue the HyperDRC native-source audit'):
        status['next_actions'][i] = ('Continue the HyperDRC native-source audit after the finite rectangle and pad-primitive repairs. Preserve the original x=1e20/16384 hard cases and full boards. Resolve retained circular algebraic-fragment area integration using the original CPArti A5/A6 pair; keep the optional exact duplicate-overlap cap and the unavailable-bound fallback. Migrate parser ratio/token/placement arithmetic, circle/oval sizing, line/stroke construction and malformed optional-token handling. Continue 40 legacy copper-index sites, shape/drill/distribution consumers, scalar thresholds, report-chord widths and density proxies. Preserve all prior workloads, failed experiments, complete reports, IPC sidecar and six-board snapshots, with explicit correctness-witness audits. Complete report identity and total resource bounds. All 39 properties, 14 targets and crate future work remain in scope.')
assert len(status['properties']) == 39 and len(status['targets']) == 14
p.write_text(json.dumps(status,indent=2)+'\n')
print('Updated existing performance and full-scope status ledgers from verified evidence.')
