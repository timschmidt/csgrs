"""Write the admission narrative from complete retained evidence."""
import json
from pathlib import Path

E = Path(__file__).resolve().parent
s = json.loads((E / 'qualification-summary-v1.json').read_text())
assert s['passed'] and not (E / 'progress-v1.md').exists()
lines = [
    '# Distribution and concatenation progress', '',
    'Baseline `00e612307`. This milestone restores explicit public failure and',
    'validates sources before distribution shortcuts, with checked native',
    'concatenation and reusable qualified PWN facts. It is focused acceptance,',
    'not completion of properties.md, full CI, or a shipped release.', '',
    'The owning-layer/domain contract is in [contract-v1.md](contract-v1.md);',
    'the deliberate Result-returning API and invalid-source migrations are in',
    '[implementation-contract-v1.md](implementation-contract-v1.md). Reproduction',
    'established empty-on-error wrappers, erased native causes, unchecked huge',
    'copy counts, invalid source shortcuts, and release-mode index rebasing that',
    'could wrap an invalid index into another input.', '',
    'Kept: typed strict wrappers and native causes; native checked concatenation;',
    'shared closed-PWN validation with combined face/edge certainty; checked',
    'copy capacities; exact full-workload semantic and preservation evidence.',
    'Already satisfied and preserved: canonical-empty support and the successful',
    'original 8/16/12 distributions and 22-copy/duplicate concatenations.', '',
    'Independent acceptance covers exact membership and per-face volume proofs,',
    'strict topology, empty accumulation, self-subtraction, touching/overlapping',
    'layouts, signed spacing, arc midpoint placement and winding multiplicity.',
    'Every rational-box component also passes an independent exact six-face',
    'coverage oracle, including the full 22-copy workload. Structural copying',
    'of an open mesh makes no closed-topology claim.', '',
    'The original expanded arc-volume scalar query declines on both revisions.',
    'The [volume witness](volume-oracle-v1.md) proves every face contribution',
    'before adding exact rationals; all copies and downstream tests remain.',
    'This does not extend general scalar expression recognition. All failed',
    'builds and diagnostics remain in [diagnostic-findings-v1.md](diagnostic-findings-v1.md)',
    'and the [complete summary](qualification-summary-v1.json).', '',
    '## Validation', '',
    f"The final `{s['qualification_phase']}` phase passes all {s['phases'][s['qualification_phase']]['commands']} commands on frozen source.",
    'Counts below are per feature configuration and overlap; they are not a',
    'count of distinct workflows or coverage.', '',
    '| Command/configuration | Passed | Ignored |', '| --- | ---: | ---: |',
]
for name, totals in s['test_totals'].items():
    lines.append(f"| {name} | {totals['passed']} | {totals['ignored']} |")
lines += [
    '', 'Formatting, native/root/minimal/FFI Clippy with warnings denied, public',
    'documentation, both benchmark builds and all four original benchmark',
    'consumers pass. Rebuilt C, C++, Python and Go consumers pass, including',
    'Boolean failures and empty composition. JavaScript/WASM certainty, query,',
    'geometry and invalid-input checks plus strict TypeScript declarations pass.',
    'The additional direct all-feature release library build passes. A separate',
    'benchmark build supplies matching baseline/candidate profiles and features',
    'for the fixed cost phase; the preparation record verifies native dependencies.',
    '', '## Preserved work and measured cost', '',
    'All 60 timing and 20 allocation children complete. Every replay of each of',
    'the five fixtures is byte-identical to its original exact coordinate/index',
    'artifact; numerical semantics are established separately above. The fixed',
    'three-pair cold/warm timing matrix includes input construction, and allocation',
    'uses separate binaries. Serialization is an untimed replay after the',
    'post-drop retention snapshot. These short single-host probes are not an',
    'optimization or competitive speedup claim; all unfavorable samples remain.',
    '', '| Fixture | State | Baseline median ns | Candidate median ns | Candidate / baseline |',
    '| --- | --- | ---: | ---: | ---: |',
]
for row in s['cost']['timing_pairs']:
    m = row['median_nanos']
    lines.append(f"| {row['case']} | {row['temperature']} | {m['baseline']} | {m['candidate']} | {row['candidate_over_baseline']:.3f} |")
lines += [
    '', 'The complete timing/allocation rows, requested-byte peaks, post-drop',
    'retention and driver/library sizes are retained in the summary and',
    '`resources-v1/measurements/`. Toolchain, hardware, feature fingerprints and',
    'source byte/line deltas are in `cost-host-v1.json`. Requested allocation',
    'bytes exclude allocator overhead and do not bound total heap use.',
    '', '## Remaining boundaries', '',
    'Capacity checks precede copy construction, and overflow/failed reservations',
    'are typed. They do not bound scalar/shared-buffer allocations, expression',
    'growth, total work, retained heap bytes, or cancellation. Dangerous baseline',
    'children used external containment; Node/WASM retains wall containment but',
    'has no address-space cap. No internal resource-bound claim is made.', '',
    'Local certainty and native PWN evidence do not reconstruct inherited',
    'construction intent. General provenance, wider symbolic completion, clean',
    'external package/release validation, full CI, formal/fuzz/coverage targets',
    'and the 35 separately unadmitted native files remain open.', '',
    'Closure requires all currently available inherited evidence and every new',
    'build/overlay artifact to retain its hash. The 27,093 historical missing',
    'artifact expectations remain recorded without recovery claims. Only the',
    'protected ledger’s raw `current_work` value is updated; every other byte',
    'and decoded key must match revision `9025de62`. No push is performed.',
]
(E / 'progress-v1.md').write_text('\n'.join(lines) + '\n')
