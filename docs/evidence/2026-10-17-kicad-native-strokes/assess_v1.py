"""Retain rejected ingress experiments and verify the unchanged production tree."""
import ast
import difflib
import gzip
import hashlib
import json
import re
import shutil
import subprocess
from pathlib import Path

W = Path(__file__).resolve().parent
ROOT = W.parents[2]
E = ROOT / 'docs/evidence/2026-10-17-kicad-native-strokes'
PARENT = ROOT / 'docs/evidence/2026-10-17-native-stroke-completion'
BASE = 'a843bf79dfe329fb51ababaaf803752bb74a0f64'

def sha(p):
    with Path(p).open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()

def read(p):
    return json.loads(Path(p).read_text())

def gz(p):
    with gzip.open(p, 'rt') as f:
        return json.load(f)

def dump(p, value):
    Path(p).write_text(json.dumps(value, indent=2, sort_keys=True) + '\n')

assert not E.exists()
assert subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip() == BASE
subprocess.run(['git', 'diff', '--quiet'], cwd=ROOT, check=True)
subprocess.run(['git', 'diff', '--cached', '--quiet'], cwd=ROOT, check=True)
parent = read(PARENT / 'checkpoint-v1.json')
assert all(sha(ROOT / p) == h for p, h in parent['evidence_files_sha256'].items())
assert sha(ROOT / 'docs/properties-status.json') == parent['ledger_after_sha256']
baseline = gz(W / 'reproduction-v1/source-hashes.json.gz')
assert len(baseline) == 2835
assert all(sha(ROOT / p) == sha(W / 'baseline-source-v1' / p) == h for p, h in baseline.items())
manifests = {n: gz(W / f'candidate-preparation-v{n}/source-hashes.json.gz') for n in [1, 3, 4]}
for n, inputs in manifests.items():
    assert all(sha(W / f'candidate-source-v{n}' / p) == h for p, h in inputs.items())

validation = read(W / 'candidate-validation-v4/result.json')
assert validation['complete'] and validation['unexpected_exits'] == ['full-board', 'copper-neck', 'clippy']
rows = read(W / 'candidate-validation-v4/commands.json')
assert len(rows) == 13
assert read(W / 'candidate-v3-cancellation-v1/result.json')['cargo_target_closed']
assert not (W / 'candidate-validation-v3/result.json').exists()

verified_commands = []
for commands in sorted(W.glob('*/commands.json')):
    for row in read(commands):
        for suffix in ['stdout', 'stderr', 'resources']:
            assert sha(commands.parent / (row['name'] + '.' + suffix)) == row[suffix + '_sha256']
        for entry in row.get('binaries', {}).values():
            assert sha(entry['retained']) == entry['sha256']
        verified_commands.append({'folder': commands.parent.name, 'name': row['name'], 'exit_code': row['exit_code']})
for version in [1, 3, 4]:
    protocol = read(W / f'candidate-validation-v{version}/protocol.json')
    assert sha(W / f'candidate_v{version}.py') == protocol['runner_sha256']
    assert sha(W / f'validation_common_v{version}.py') == protocol['common_sha256']
    assert sha(W / f'candidate-preparation-v{version}/source-hashes.json.gz') == protocol['source_hashes_sha256']

def counts(folder, name):
    matches = re.findall(r'test result: (?:ok|FAILED)\. (\d+) passed; (\d+) failed; (\d+) ignored;',
                         (W / folder / (name + '.stdout')).read_text())
    return dict(zip(['passed', 'failed', 'ignored'], [sum(int(row[i]) for row in matches) for i in range(3)]))

assert counts('reproduction-v1', 'archived-frontier') == {'passed': 5, 'failed': 21, 'ignored': 0}
assert counts('reproduction-v1', 'current-source-failures') == {'passed': 39, 'failed': 0, 'ignored': 0}
expected = {'cold-zero-budget': (1, 0), 'native-strokes': (10, 0), 'source-failures': (39, 0),
            'native-aggregation': (12, 0), 'full-board': (0, 1), 'copper-neck': (0, 1),
            'gerber': (1, 0), 'native-tests': (1675, 0)}
results = {}
for name, (passed, failed) in expected.items():
    results[name] = counts('candidate-validation-v4', name)
    assert results[name] == {'passed': passed, 'failed': failed, 'ignored': 0}
full_error = (W / 'candidate-validation-v4/full-board.stderr').read_text()
neck_error = (W / 'candidate-validation-v4/copper-neck.stderr').read_text()
assert '396.946659071s' in full_error and '133.165601323s' in neck_error
assert 'map_clone' in (W / 'candidate-validation-v4/clippy.stderr').read_text()
report = (W / 'candidate-validation-v4/full-board.stdout').read_text()
assert re.findall(r'(\d+) violation\(s\) found, (\d+) waived:', report) == [('12', '0'), ('197', '0')]
assert 'skipping complex project fixture' not in report
assert all(sha(W / 'candidate-source-v4' / p) == baseline[p] for p in [
    'crates/hyperdrc/src/app.rs', 'crates/hyperdrc/src/checks/layer.rs'])

E.mkdir()
retained = {}
physical_only = []
for p in sorted(W.rglob('*')):
    if not p.is_file() or '__pycache__' in p.parts:
        continue
    relative = p.relative_to(W)
    top = relative.parts[0]
    if any(top.startswith(prefix) for prefix in [
        'baseline-source-', 'candidate-source-', 'candidate-target-', 'candidate-scratch-', 'candidate-executables-']):
        continue
    with p.open('rb') as stream:
        magic = stream.read(4)
    if magic == b'\x7fELF' or p.name == 'perf.data':
        physical_only.append({'path': str(p), 'sha256': sha(p), 'bytes': p.stat().st_size})
        continue
    assert p.stat().st_size < 50_000_000, p
    dest = E / relative
    dest.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(p, dest)
    assert sha(dest) == sha(p)
    retained[str(relative)] = {'original': str(p), 'sha256': sha(p)}

changed_sources = {}
for version in [1, 2, 3, 4]:
    source = W / f'candidate-source-v{version}'
    changed = {}
    patch = []
    for p in sorted(source.rglob('*')):
        if not p.is_file():
            continue
        rel = str(p.relative_to(source))
        digest = sha(p)
        if baseline.get(rel) == digest:
            continue
        assert p.suffix in ['.md', '.rs'], p
        dest = E / f'candidate-changes-v{version}' / rel
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(p, dest)
        changed[rel] = digest
        before = (W / 'baseline-source-v1' / rel).read_text() if rel in baseline else ''
        patch.extend(difflib.unified_diff(before.splitlines(keepends=True), p.read_text().splitlines(keepends=True),
                                        fromfile='a/' + rel if rel in baseline else '/dev/null', tofile='b/' + rel))
    (E / f'candidate-v{version}.patch').write_text(''.join(patch))
    changed_sources[str(version)] = changed
assert len(changed_sources['4']) == 13
for p in E.rglob('*.py'):
    ast.parse(p.read_text(), filename=str(p))

decision = {
    'status': 'rejected: exact track ingress exceeds preserved full-board and copper-neck resource contracts',
    'main_parent_commit': BASE, 'source_changes_applied': False, 'implementation_sha256': {},
    'candidate_source_changes_sha256': changed_sources,
    'original_archived_regressions': counts('reproduction-v1', 'archived-frontier'),
    'current_source_failure_regressions': counts('reproduction-v1', 'current-source-failures'),
    'final_candidate_test_results': results,
    'final_candidate_commands': rows,
    'verified_command_records': verified_commands,
    'completed_cargo_targets_closed': [str(W / 'candidate-target-v1'), str(W / 'candidate-target-v3'), '/tmp/csgrs-kicad-native-target-v4'],
    'full_board_resource_gate': {'worker_cpu_seconds': 396.946659071, 'limit_seconds': 120, 'exit_code': 101,
        'worker_cpu_seconds_before_profiling': 204.33, 'peak_rss_kib': 1671044,
        'instrumentation': 'Perf and gdb attached after the worker had already exceeded its unchanged CPU limit. Final time is diagnostic, not an uninstrumented benchmark.'},
    'copper_neck_resource_gate': {'check_body_wall_seconds': 133.165601323, 'limit_seconds': 60, 'exit_code': 101,
        'instrumentation': 'No profiler attached to this cold gate. Follow-up observer raced into the subsequent native mechanical suite.'},
    'candidate_v3_cancelled': {'worker_cpu_seconds_observed': 150.11, 'full_board_assertion_executed': False,
        'exit_code': 130, 'later_checks_attempted': False},
    'report_counts': {'baseline': [12, 201], 'candidate': [12, 197], 'semantic_report_audit_complete': False},
    'retained_completed_evidence': retained, 'physical_artifacts_not_duplicated_in_git': physical_only,
    'original_workload_gates_unchanged': True, 'original_fixture_resolution_preserved': True,
    'native_disposition_preserved': True, 'pending_native_paths_remaining': 0,
    'full_archived_integration_admitted': False, 'native_runtime_admitted': False,
    'general_exact_kicad_stroke_fidelity_claimed': False, 'full_plan_complete': False,
    'full_pre_push_matrix_executed': False, 'downstream_hypercircuit_validation_complete': False,
    'published_package_pipeline_executed': False,
    'limitations': [
        'Focused exact area, bounds and point-membership regressions pass in the prototype. They do not establish admission when unchanged workload gates fail.',
        'The candidate intentionally changes finite rectangles to exact round-capped source sets and readiness version 21 to 22. That migration remains unadmitted; changed findings have not received an independent full-report semantic audit.',
        'Native aggregation avoids obligatory finite projection and preserves authored report sources and non-finite input history in v4. This component has not been independently admitted on the unchanged main source adapter.',
        'Strict Clippy fails on the new explicit clone iterator. No warning suppression or passing lint claim is made.',
        'The 1675 broad native results exclude three separately executed gates; the two failed gates remain failures. Focused results also occur in the broad suite and must not be summed as unique coverage.',
        'Cost preflights verify fixture/oracle execution only. They ran concurrently with validation compilation and use rejected v3 libraries; no final matched benchmark or performance retention is claimed.',
        'A full-board stack sampled neck morphology repeatedly computing retained-loop bounds. Another stack sampled mechanical native aggregation. Neither proves sole causation or supports an unmeasured optimization.',
        'No new internal work, retained-heap, expression-growth or cancellation bound is implemented. External timeout/address-space limits and RSS are distinct contracts.',
        'Cancelled v3 and all harness preparation/compile failures remain preserved. Failed v4 cancellation sent no signals; the v4 runner completed normally with exit 101.',
        'Finite line_polygon fidelity, general strict completion, end-to-end provenance, downstream consumers, shipped external packages, minimum compiler and complete CI validation remain open.',
        'Historical physical artifact audit remains failed/incomplete with 27093 inherited missing paths. New path hashing does not establish whole-history recovery.'
    ],
    'next': [
        'Use unchanged original full projects to isolate source load, native aggregation and individual readiness-check costs against read-only completed v4 libraries.',
        'Repair the owning kernel or composition invariant with independent semantic and cache/budget tests; preserve every original workload gate before reconsidering exact track ingress.',
        'Continue the full properties plan, including independently justified components of the rejected archive, ordinary strict completion, provenance and complete external user-path validation.'
    ]
}
dump(E / 'decision-v1.json', decision)
(E / 'progress-v1.md').write_text('''# E36: exact KiCad track ingress remains unadmitted

The prototype repairs exact source strokes and native board aggregation, but
fails the unchanged full-board and copper-neck resource contracts. No production
source change is applied. The original 35-file bundle remains rejected; earlier
independently admitted components remain in place.

On parent `a843bf79dfe329fb51ababaaf803752bb74a0f64`, the original archived
regressions still produce 5 passes and 21 failures. Current typed-source tests
pass all 39 cases. The new source acceptance tests fail 10/10 on that parent;
their candidate v4 counterparts pass, together with 39 source-failure tests and
12 native-aggregation tests. The latter cover exact out-of-f64-range geometry,
empty composition, malformed operands, holes, mixed curve families, budgets,
source diagnostic history, authored report order, ownership and concurrent use.

Candidate v4 records 1675 passing native results with the three workload gates
executed separately. Gerber passes. Full-board fails at 396.946659071 worker CPU
seconds against 120; copper-neck fails at 133.165601323 check-body wall seconds
against 60. Both original complete projects and assertions are unchanged.
The full-board worker had already used 204.33 CPU seconds before profiling;
its final time is a diagnostic observation, not a clean benchmark. Copper-neck
was not profiled. Documentation and CLI builds pass. Strict Clippy fails on a
new explicit clone iterator. Complete commands and raw outcomes are retained.

The earlier v3 run was cancelled after observing 150.11 worker CPU seconds;
its final assertion did not run. The attempted v4 cancellation found that its
target process had naturally exited and sent no signals. The subsequent stack
snapshot observes a mechanical test in the broad suite, despite stale neck
attribution in its preparation metadata. These are distinct observations.

The candidate migrates finite rectangles to exact capsules and changes report
counts from 12/201 to 12/197. An independent audit of those report changes is
still required. Cost probes are oracle preflights, not admission benchmarks.
The failed attempts, partial preparations and immutable source snapshots are
preserved; completed Cargo targets must not be reopened.

Next, isolate native aggregation and readiness-check costs on the original
projects before choosing a kernel repair. No input reduction, weaker policy,
warning suppression or relaxed runtime limit is accepted. The shipped user
path and full properties plan remain open. The historical artifact audit still
has 27,093 missing paths; new evidence cannot certify that lost history.
''')
dump(E / 'retention-v1.json', {'complete': True, 'source_changes_applied': False,
    'main_inputs_verified': len(baseline), 'retained_evidence_files': len(retained),
    'runner_sha256': sha(__file__), 'decision_sha256': sha(E / 'decision-v1.json')})
print(json.dumps({'retained_evidence_files': len(retained), 'candidate_changes': len(changed_sources['4']),
                  'source_changes_applied': False, 'status': decision['status']}), flush=True)
