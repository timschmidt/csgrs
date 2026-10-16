"""Record the narrow OBJ correctness decision and every fixed cost sample."""
import json
from pathlib import Path
import re

E = Path(__file__).resolve().parent
assert not (E / 'decision-v1.json').exists()
assert not (E / 'progress-v1.md').exists()


def read(name):
    return json.loads((E / name).read_text())


qualification = read('candidate-validation-v1/result.json')
cost = read('cost-v1/result.json')
application = read('application-v1.json')
assert qualification['complete'] and not qualification['unexpected_exits']
assert cost['complete'] and application['format_exit_code'] == 0
commands = read('candidate-validation-v1/commands.json')
summary = []
table = []
for row in commands:
    text = (E / 'candidate-validation-v1' / (row['name'] + '.stdout')).read_text()
    groups = [tuple(map(int, values)) for values in re.findall(
        r'test result: .*? (\d+) passed; (\d+) failed; (\d+) ignored;', text)]
    counts = dict(zip(['passed', 'failed', 'ignored'], map(sum, zip(*groups)))) if groups else {}
    summary.append({k: row[k] for k in ['name', 'command', 'exit_code', 'expected_exit_code', 'elapsed_seconds']} | {'reported_test_counts': counts})
    count_text = ', '.join(f'{value} {name}' for name, value in counts.items()) or 'command status only'
    table.append(f"| {row['name']} | {row['exit_code']} | {count_text} |")
assert next(r for r in summary if r['name'] == 'minimal-regression')['reported_test_counts'] == {'passed': 7, 'failed': 0, 'ignored': 0}
assert next(r for r in summary if r['name'] == 'format-probe')['reported_test_counts'] == {'passed': 7, 'failed': 5, 'ignored': 0}
cost_table = []
for row in cost['summary']:
    cost_table.append(f"| {row['case']} | {row['median_paired_ratio']:.6f} | {row['slower_candidate_pairs']}/5 | {row['output_bytes']} | {row['retained_string_capacity_bytes']} |")
decision = {
    'status': 'kept: OBJ index validation with preserved valid serialization',
    'main_parent_commit': application['main_parent_commit'],
    'source_snapshot_commit': '3b69af324a9642d1f71b9cc38de58b59a236f740',
    'owning_layer': 'CSGRS OBJ boundary reusing Hypermesh position-index resolution',
    'properties': ['P6', 'P18', 'P26', 'P27', 'P35', 'P36', 'P37'],
    'implementation_sha256': application['implementation_sha256'],
    'qualification_commands': summary, 'fixed_cost_comparison': cost,
    'original_format_diagnostic': {'passed': 7, 'failed': 5, 'known_remaining_failures': qualification['known_unfixed_format_failures']},
    'obj_index_fix_admitted': True, 'native_runtime_admitted': False,
    'pending_native_files_unchanged': 35,
    'tests_or_policies_weakened': False, 'valid_output_serialization_changed': False,
    'optimization_claimed': False, 'application_speedup_claimed': False,
    'published_package_pipeline_executed': False, 'full_workflow_matrix_executed': False,
    'full_plan_complete': False,
    'next': 'Correct STL import topology through native validation, retaining small-scale, malformed-edge and balanced-winding regressions. Keep text projection, binary STL collapse, native timing gates and full distribution/resource work open.',
}
(E / 'decision-v1.json').write_text(json.dumps(decision, indent=2) + '\n')
text = f'''# OBJ export rejects invalid vertex references

The OBJ exporter now returns its existing `IoError::Geometry` diagnostic when a
triangle refers to a missing position. The stream wrapper uses the same check
and writes no bytes on validation failure. Hypermesh's `triangle_points` owns
index resolution; no separate index formula or topology kernel is introduced.
Valid object text retains its existing byte representation.

The [contract](contract-v1.md) defines the finite domain and remaining limits.
The [decision](decision-v1.json) admits this narrow source correction. It does
not admit the 35 pending native files or claim a completed format/release plan.

## Reproduction and correction

The seven public regressions first produced **three failures and four passes**
against the fresh physical baseline. Missing indices, `usize::MAX` and the stream
writer all failed their acceptance assertions. Metadata priority, finite-value
failure, valid open-triangle text and empty/solid composition controls passed.
The [baseline logs](baseline-tests-v1/test.stdout) remain unchanged.

All seven pass with the correction. The unchanged twelve-test format diagnostic
now reports **seven passes and five failures**: its three OBJ index failures
are corrected. Its remaining text projection and STL failures are preserved,
including their failing process exit 101. An expected diagnostic failure is not
a passing format suite. The separately reproduced STL edge-balance failures in
the parent checkpoint also remain open.

Canonical empty, a valid open triangle and a unit cube remain exportable. The
solid round trip checks strict PWN validity, manifoldness, box boundary planes,
exact oriented volume and another Boolean operation. Fixed serialized bytes are
checked separately from those semantic claims. The change does not certify
topology of every exported mesh or the intent of an approximate source.

## Validation of the final source

The candidate uses an initially empty physical Cargo target, with no nested
overlay or inherited Cargo artifacts. Fixed probes bind explicit physical
libraries; their source/library/binary hashes and retained copies are recorded.
The source snapshot is `3b69af324a9642d1f71b9cc38de58b59a236f740` plus unchanged
pending native files. Main parent `{application['main_parent_commit']}` adds
the completed physical-build evidence without changing those production inputs.

| Command | Exit | Reported test results |
| --- | ---: | --- |
{chr(10).join(table)}

Full root tests use all features and the separate OBJ-only configuration.
Clippy covers all targets with warnings denied for both configurations; docs
also deny warnings. Counts overlap across configurations. Original ignored
tests are retained. The exact tested production and regression bytes were then
applied to the main worktree, all other pinned build inputs were compared, and
`cargo fmt --all -- --check` passed there after the final edit; see
[application evidence](application-v1.json).

## Valid-export cost and preserved output

Five alternating pairs run the identical fixed probe in ten fresh processes.
All 30 rows complete successfully. Timed work materializes every output String
and retains every String until timing ends. Construction, strict fixture
validation, one warmup, disk writes and exact output comparison are outside
timing. Every full output byte and retained String capacity agrees across all
processes. The [protocol](cost-v1/protocol.json), [raw rows](cost-v1/rows.json)
and [result](cost-v1/result.json) retain all favorable and unfavorable samples.

| Case | Median paired candidate/baseline time | Slower candidate pairs | OBJ bytes | Retained String capacity bytes |
| --- | ---: | ---: | ---: | ---: |
{chr(10).join(cost_table)}

The directly compiled probe executables contain {cost['binary_bytes']['baseline']}
baseline bytes and {cost['binary_bytes']['candidate']} candidate bytes. These
are probe sizes, not shipped library or binding sizes. String capacities cover
payload buffers only; total allocations and cold exports were not measured.
The new successful validation pass is linear in triangle count and uses a
fixed-size array of borrowed references. It adds no heap buffer. Overall export
retains its existing unbounded input-controlled allocation/work behavior;
external time/address-space containment is not an internal resource guarantee.
This is a correctness correction, with no optimization, statistical significance
or application-speed claim. Faster rejection of malformed input is not credited
as faster successful modeling.

## Remaining boundaries

Direct consumers are Rust exporters, examples and benchmark helpers. No OBJ
export entry point was found in the inspected C ABI or JavaScript wrappers.
The complete pre-push matrix and clean published-package pipeline were not run.
This is source-level acceptance, not a shipped-distribution claim.

The native full-board and copper-neck gates remain failed as recorded by the
parent physical-build checkpoint. Main source guards leave all 35 pending
native files unchanged. Historical nested-overlay identity failures and 27,093
inherited missing artifact expectations remain unqualified; only the new
physical OBJ namespace is inventoried here. There is no whole-history audit or
zero-loss claim. Only `current_work` changes in the status ledger; the remaining
raw bytes and decoded keys stay bound to revision `9025de62`.

Next, correct STL import through native topology validation while retaining
the small valid cubes, malformed edge multiplicity and balanced winding cases.
Text projection, binary STL face collapse, broader provenance/resources,
native admission, release and the remaining properties plan stay open.
'''
(E / 'progress-v1.md').write_text(text)
print('Recorded narrow OBJ acceptance, all diagnostic failures and all fixed cost samples.')
