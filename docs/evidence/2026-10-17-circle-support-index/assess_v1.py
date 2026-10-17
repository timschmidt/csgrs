"""Retain useful progress without admitting a storage-interrupted candidate."""
import gzip
import hashlib
import json
import re
import subprocess
from pathlib import Path

W = Path(__file__).resolve().parent
ROOT = W.parents[2]

def sha(p):
    with Path(p).open('rb') as f:
        return hashlib.file_digest(f, 'sha256').hexdigest()

def read(p):
    return json.loads(Path(p).read_text())

def dump(p, v):
    Path(p).write_text(json.dumps(v, indent=2) + '\n')

base = read(W / 'baseline-preparation-v1/result.json')['base_commit']
assert subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip() == base
subprocess.run(['git', 'diff', '--quiet'], cwd=ROOT, check=True)
subprocess.run(['git', 'diff', '--cached', '--quiet'], cwd=ROOT, check=True)
with gzip.open(W / 'baseline-preparation-v1/main-input-hashes.json.gz', 'rt') as f:
    main = json.load(f)
assert all(sha(ROOT / p) == h for p, h in main.items())
for label in ['baseline', 'candidate']:
    assert read(W / (label + '-validation-v1/result.json'))['command_failures'] == []
    assert '7 passed; 0 failed' in (W / (label + '-validation-v1/contracts.stdout')).read_text()
for p in W.glob('*/commands.json'):
    for row in read(p):
        for suffix in ['stdout', 'stderr', 'resources']:
            assert sha(p.parent / (row['name'] + '.' + suffix)) == row[suffix + '_sha256']
q = W / 'candidate-qualification-v2'
qr = read(q / 'result.json')
assert qr['complete'] and qr['cargo_target_closed']
expected = ['native-tests', 'curve-all-build', 'curve-all', 'root-all-build', 'root-all', 'root-minimal', 'circuit-build', 'circuit-all', 'docs', 'native-cli']
assert qr['command_failures'] == expected
commands = {r['name']: r for r in read(q / 'commands.json')}
assert all(commands[n]['exit_code'] == 0 for n in ['format', 'clippy', 'native-build', 'full-board', 'copper-neck', 'gerber'])
text = (q / 'native-tests.stdout').read_text()
counts = [tuple(map(int, m)) for m in re.findall(r'test result: (?:ok|FAILED)\. (\d+) passed; (\d+) failed; (\d+) ignored', text)]
assert tuple(map(sum, zip(*counts))) == (1648, 5, 0)
failures = re.findall(r'^test (\S+) \.\.\. FAILED$', text, re.M)
assert len(failures) == 5 and 'QuotaExceeded' in text and 'disk I/O error' in text
assert all('Disk quota exceeded' in (q / (n + '.stderr')).read_text() for n in expected if n != 'native-tests')
b = W / 'boundary-validation-v1'
bc = read(b / 'commands.json')
assert [(r['name'], r['exit_code']) for r in bc] == [('format', 0), ('scalar-value-hash', 0), ('ffi-build', 101), ('ffi-tests', 101), ('ffi-clippy', 101)]
assert '1 passed; 0 failed' in (b / 'scalar-value-hash.stdout').read_text()
assert all('Disk quota exceeded' in (b / (n + '.stderr')).read_text() for n in ['ffi-build', 'ffi-tests', 'ffi-clippy'])
assert not (b / 'result.json').exists()
pre = read(W / 'cost-preflight-v4/result.json')
fallback = read(W / 'cost-fallback-v5/result.json')
heap = read(W / 'heap-preflight-v4/result.json')
assert all(v['complete'] and v['command_failures'] == [] for v in [pre, fallback, heap])
assert pre['all_ordered_exact_rows_equal'] and pre['all_independent_oracles_passed'] and pre['all_receipts_certified']
assert fallback['all_ordered_exact_rows_equal'] and fallback['all_receipts_certified']
assert heap['all_instrumented_witnesses_identical']
assert pre['process_samples'] == 24 and fallback['process_samples'] == 56
for folder in ['cost-preflight-v2', 'cost-preflight-v3']:
    r = read(W / folder / 'commands.json')
    assert [(x['name'], x['exit_code']) for x in r] == [('baseline-contracts', 0), ('candidate-contracts', 0), ('mixed-512-0-baseline', 101)]
    assert not (W / folder / 'result.json').exists()
assert not (W / 'project-reports-v1').exists() and not (W / 'cost-matched-v4').exists()
prototype = ['crates/hypercurve/src/curve_intersection.rs', 'crates/hypercurve/src/circle_support_cache_tests.rs', 'crates/hypercurve/README.md']
with gzip.open(W / 'candidate-preparation-v2/source-hashes.json.gz', 'rt') as f:
    final = json.load(f)
assert all(sha(W / 'candidate-source-v2' / p) == h for p, h in final.items())
changes = {p: sha(W / 'candidate-source-v2' / p) for p in prototype}
assert {p for p, h in final.items() if p not in main or main[p] != h} == set(prototype)
quota = subprocess.run(['quota', '-s'], capture_output=True, text=True)
dump(W / 'quota-observation-v1.json', {
    'command': ['quota', '-s'], 'exit_code': quota.returncode, 'stdout': quota.stdout, 'stderr': quota.stderr,
    'observed_failure': 'Per-user /tmp hard quota reached 51,490 MiB while df reported roughly 13 GiB filesystem free; repository filesystem had roughly 4.4 GiB free.',
    'sandbox_failure': 'Default sandbox mount setup also failed with Quota exceeded while registering /tmp/.git; subsequent evidence reads used approved host execution.',
    'orchestrator_hold_attempt': 'Guarded process lookup found no running qualification orchestrator and failed before sending a signal. The runner had already finished its failed tail. No process was stopped.',
    'user_input_pending': 'Increase quota or provide a build directory with at least 6 GiB; preserve existing csgrs-* physical artifacts.',
    'evidence_loss_detected_in_this_work': False,
})
status = 'unadmitted: exact circle-support index passes focused semantics and original workload gates; full qualification interrupted by storage quota'
decision = {
    'status': status, 'progress_classification': 'progress', 'main_parent_commit': base,
    'owning_layer': 'Hypercurve temporary native intersection batch',
    'prototype_source': 'candidate-source-v2', 'prototype_sha256': changes, 'implementation_sha256': {}, 'source_changes_applied': False,
    'focused_semantic_tests_per_variant': 7, 'existing_batch_reuse_test_per_variant': 1, 'scalar_numeric_hash_contract_passed': True,
    'passed_qualification_commands': [n for n, r in commands.items() if r['exit_code'] == 0],
    'original_workload_gates': {n: {'exit_code': commands[n]['exit_code'], 'whole_supervised_command_seconds': commands[n]['elapsed_seconds']} for n in ['full-board', 'copper-neck', 'gerber']},
    'native_test_passed': 1648, 'native_test_failed': 5, 'native_failure_names': failures,
    'native_failure_diagnosis': 'Four failures explicitly report quota exhaustion; SQLite reports disk I/O error during the same incident. Reproduce all five after storage recovery; no passing result is inferred.',
    'uncompleted_qualification_commands': expected,
    'boundary_commands': [{k: r[k] for k in ['name', 'exit_code', 'elapsed_seconds']} for r in bc],
    'diagnostics': {
        'v1_integer_preflight': 'preflight-comparison-v1/result.json',
        'v4_expanded_exact_preflight': 'cost-preflight-v4/result.json',
        'repeated_fallback_controls': 'cost-fallback-v5/result.json',
        'whole_process_allocation_and_size_preflight': 'heap-preflight-v4/result.json',
    },
    'expanded_preflight_process_samples': 24, 'repeated_fallback_process_samples': 56, 'instrumented_resource_observations': 14,
    'all_expanded_ordered_exact_rows_equal': True, 'all_expanded_receipts_certified': True,
    'discarded_probe_assumptions': [
        'v2 assumed subtracting pi exposed a rational scalar carrier',
        'v3 made the same structural assumption for y; v4 adds strict sign and exact equality proofs before value encoding',
    ],
    'prepared_but_unexecuted': [
        'Full matched timing under a completely qualified candidate', 'Native full-project report byte comparison',
        'Actual C/C++/Python consumers', 'Actual JavaScript/TypeScript consumers', 'WASM Clippy',
    ],
    'limitations': [
        'No production patch admitted; no full native or boundary pass claimed.',
        'Cost observations are focused diagnostics, not end-to-end speedup or final optimization acceptance.',
        'Mixed and superrange heap peaks increase slightly; symbolic allocation count increases by six; all adverse rows retained.',
        'No new total-work, heap-byte, expression-growth or cancellation bound.',
        'Exact-track E36 integration remains rejected and the original 35-file disposition remains intact.',
        'Inherited artifact audit still has 27,093 missing paths; no whole-history or zero-loss claim.',
        'Full pre-push, published-package, MSRV, formal-proof, coverage and directed-campaign targets remain open.',
    ],
    'next': [
        'After storage recovery, use fresh targets for remaining full Hypercurve, CSGRS, HyperDRC, Hypercircuit and boundary qualification; do not reopen completed targets.',
        'Reproduce the five native file-output failures without quota exhaustion, preserving the original assertions.',
        'Complete full-project byte comparisons and matched cost measurements, then decide whether to apply the three-file candidate.',
        'If retained, reevaluate exact KiCad ingress independently against its unchanged workloads; preserve all broader properties targets.',
    ],
    'native_runtime_admitted': False, 'full_archived_integration_admitted': False, 'full_plan_complete': False,
}
dump(W / 'decision-v1.json', decision)
(W / 'progress-v1.md').write_text("""# Circle-support index: unadmitted prototype

The temporary Hypercurve intersection batch has an isolated candidate that
indexes ordered rational support values and preserves the existing symbolic
comparison order. No production source has been changed.

Seven independent semantic contracts and the existing batch-reuse test pass on
both baseline and candidate. The scalar canonical-value hashing test passes.
Strict Clippy passes for Hypercurve, CSGRS, HyperDRC and Hypercircuit. The original
full-board, copper-neck and Gerber gates pass with their assertions and complete
fixtures unchanged. Whole supervised command times were about 55.45, 44.39 and
0.46 seconds; these are not the gates' internal CPU/check-body measurements.

Broader native execution completed 1,648 tests and failed five file-output tests.
Four explicitly report quota exhaustion; SQLite reports disk I/O error during the
same incident. The /tmp per-user hard quota reached 51,490 MiB despite about
13 GiB free on the filesystem. Later builds, documentation and binding builds
failed on quota writes. Neither those checks nor the five failed tests are
credited as passing. A guarded attempt to hold the orchestrator found it had
already exited; no signal was sent. A fresh build location or increased quota
is needed before requalification. Existing physical artifacts must be preserved.

The expanded diagnostic completes 24 fresh-process samples across rational,
symbolic, mixed, superrange and subrange inputs. Every ordered witness agrees,
both circle equations and independent 3-4-5 coordinates hold, and every operation
has a strict certified receipt. Two earlier encoders incorrectly assumed proved
rational values always exposed a structural rational carrier; their baseline
failures are retained. The final encoder requires strict value proofs for both
coordinates and preserves y ordering.

A separate seven-repeat fallback diagnostic retains 56 process samples. At 512
symbolic pairs the candidate/baseline median ratios are about 0.986 cold and
1.002 warm; the earlier single-sample 6% slowdown was not consistent. At 128
symbolic pairs the ratios are about 1.017 and 1.023. Mixed controls range from
about 0.942 to 0.987. These focused diagnostics do not complete performance
admission or establish an end-to-end improvement.

Whole-process allocation observations verify the same exact instrumented output.
At 2,048 rational pairs allocation calls fall from 12,706,548 to 158,425, with
reported heap peaks 9.96 MB and 9.59 MB. Symbolic controls add six allocations;
mixed and superrange peaks increase slightly. Those costs remain visible. The
matched diagnostic binary grows 15,472 bytes, including 12,720 text bytes.
No isolated live-cache heap bound or new internal resource guarantee is claimed.

Remaining work includes full affected-crate and real binding consumers,
full-project report byte comparison, and final matched timing. The candidate is
unadmitted until those checks pass. The exact-track importer remains rejected;
the original 35-file disposition and all broader plan obligations remain intact.
The inherited artifact audit still contains 27,093 missing paths.
""")
print(json.dumps({'status': status, 'native_passed': 1648, 'native_failed': 5, 'source_changes_applied': False}), flush=True)
