"""Close the stored-bound shortcut hypothesis on the fixed native observation."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tomllib

from diagnose_v2 import verify

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    assert not (E / 'decision-v1.json').exists()
    a = json.loads((E / 'assessment-v2.json').read_text())
    prep = json.loads((E / 'preparation-v2/protocol.json').read_text())
    protocol = json.loads((E / 'diagnostic-v2/protocol.json').read_text())
    binary = json.loads((E / 'diagnostic-v2/binary.json').read_text())
    verify(prep, protocol, binary)
    assert sha(E / 'assess_v2.py') == a['assessor_sha256']
    for name, digest in a['artifacts_sha256'].items():
        assert sha(E / name) == digest, name
    c = a['counts']
    assert c['started'] == c['completed'] == 11075136 and c['unfinished'] == 0
    assert c['rows'][0]['starts'] == c['started']
    assert all(row['starts'] == 0 for row in c['rows'][1:])
    assert c['eligible_known_sign'] == c['eligible_contradictory'] == c['anomalous_nonzero_zero_sign'] == 0
    assert not a['original_gate_passed'] and a['instrumented_worker_cpu_seconds'] == 143.517755723
    source = Path(prep['parent_source'])
    profile = ROOT / 'docs/evidence/2026-10-13-native-normal-form-attribution/profile-v1/observations.json'
    observed = json.loads(profile.read_text())
    allocation_weight = next(r for r in observed['intersections'] if r['symbol'] == 'allocator_leaf')
    lock = source / 'Cargo.lock'
    packages = tomllib.loads(lock.read_text())['package']
    pinned = [p for p in packages if p['name'] in ['mimalloc', 'libmimalloc-sys']]
    assert [(p['name'], p['version']) for p in pinned] == [('libmimalloc-sys', '0.1.49'), ('mimalloc', '0.1.52')]
    cache = Path('/tmp/csgrs-properties-cargo/registry/src/index.crates.io-1949cf8c6b5b557f')
    local_inputs = {}
    for package in pinned:
        for name in ['Cargo.toml', 'src/lib.rs']:
            path = cache / (package['name'] + '-' + package['version']) / name
            assert path.is_file()
            local_inputs[str(path)] = sha(path)
    drc = [source / 'crates/hyperdrc/Cargo.toml', source / 'crates/hyperdrc/src/main.rs', source / 'crates/hyperdrc/src/lib.rs']
    assert all('global_allocator' not in p.read_text() for p in drc)
    next_step = {
        'action': 'Review allocator choice at the native DRC process boundary using the existing profile and already-pinned local dependency sources.',
        'reason': 'The stored-bound shortcut has zero native coverage. The existing unchanged-parent profile attributes 29.580% of sampled cycle weight to a declared allocator-leaf subset; this is a different cost question and does not require another scalar recognition pass or helper/storage variant.',
        'available_inputs': {'profile': str(profile.relative_to(ROOT)), 'profile_sha256': sha(profile),
                             'allocator_leaf_sampled_weight': allocation_weight,
                             'cargo_lock_sha256': sha(lock), 'pinned_packages': pinned,
                             'local_package_input_sha256': local_inputs,
                             'drc_entrypoint_input_sha256': {str(p.relative_to(source)): sha(p) for p in drc}},
        'review_required_before_prototype': [
            'Read the pinned allocator wrapper, features, build integration and target support. Inspect DRC executable and test-binary ownership and existing allocation instrumentation. Availability in Cargo.lock is not a retention recommendation.',
            'Distinguish Rust global allocations from allocations performed directly by native dependencies. The sampled allocator subset is not a prediction of what a Rust allocator replacement could remove.',
            'If a separately frozen process-boundary trial is warranted, preserve mathematical/runtime source, proof caches, policy/budget receipts, all original fixtures and both native assertion metrics. Do not impose a global allocator from the reusable library.',
            'Require matched whole-application timing and memory measurements, broad qualification, original gate checks, complete report-byte replay and executable-size evidence before admission. Keep all closed samples and rejected candidates unchanged.'
        ],
        'allocator_source_review_complete': False,
        'prototype_prepared': False, 'dependency_changed': False, 'executed': False,
    }
    decision = {
        'status': 'close-stored-bound-sign-bypass-for-observed-native-workload',
        'decision_runner_sha256': sha(Path(__file__)),
        'assessment_sha256': sha(E / 'assessment-v2.json'),
        'preparation_sha256': sha(E / 'preparation-v2/protocol.json'),
        'closed_observation': {'full_board_runs': 1, 'started': c['started'], 'completed': c['completed'],
                               'unfinished': 0, 'eligible_known_sign': 0, 'eligible_contradictory': 0,
                               'anomalous_nonzero_zero_sign': 0},
        'counts': c,
        'finding': 'Every observed exact-sign quadratic fallback had BoundCache::Invalid immediately before the original call. There was no existing root-bound sign to reuse at this site in these complete-board runs.',
        'interpretation': [
            'Do not prepare a production stored-bound check from this observation: it would add work at the measured site without skipping any observed normalization.',
            'Calibrations cover every representable bound tag and original outcome, including mismatches and unfinished observations. Site controls observe valid positive/negative/zero metadata and a known-sign/declining-normalizer case without bypassing computation.',
            'The observation is specific to this call site and workload. It does not claim that bound certificates are absent elsewhere, that computing new bounds would be free, or that other workloads have zero coverage.',
            'No arithmetic, certificate transfer, normalization retry, memo, or budget change is admitted. The direct/outlined/borrowed multiplication-helper series remains rejected and closed.'
        ],
        'qualification': {'tests': a['tests'], 'command_count': 14,
                          'format_strict_four_crate_clippy_and_warning_denied_docs': True,
                          'focused_counts_overlap_library': True,
                          'shared_reference_test_bytes_identical': True},
        'original_gate': {'worker_cpu_seconds': a['instrumented_worker_cpu_seconds'], 'limit_seconds': 120,
                          'passed': False, 'finding_counts': [12, 197],
                          'separate_copper_neck_gate_executed': False, 'complete_report_byte_identity_established': False},
        'preserved_failures': {'preparation-v1': 'One included reference-test assert needed crate-specific formatting; no compilation or measurement occurred in V1. V2 changes only that formatting. Both failed source trees and all V1 runners remain hash-checked.',
                               'generated_diff_commit_check': 'A blank final unified-diff context line triggered the generic whitespace verifier. Independent byte-identical diff reconstruction is recorded; the exception is restricted to that patch hash.'},
        'next_step': next_step,
        'limits': a['limits'] + ['The 143.517755723-second instrumented result is not a matched speedup or slowdown. The parent release failure remains 133.31248412 / 120 seconds.'],
        'main_and_35_pending_native_paths_unchanged': True,
        'runtime_admitted': False, 'plan_complete': False,
    }
    reference_name = 'crates/hyperreal/src/computable/node/bound_sign_reference_tests.rs'
    assert (Path(prep['reference_source']) / reference_name).read_bytes() == (Path(prep['source']) / reference_name).read_bytes()
    (E / 'decision-v1.json').write_text(json.dumps(decision, indent=2) + '\n')
    status_path = ROOT / 'docs/properties-status.json'
    status = json.loads(status_path.read_text())
    original = json.loads(subprocess.check_output(['git', 'show', '9025de62:docs/properties-status.json'], cwd=ROOT))
    assert {k: v for k, v in status.items() if k != 'current_work'} == {k: v for k, v in original.items() if k != 'current_work'}
    status['current_work']['bound_sign_fallback_coverage'].update(
        status=decision['status'], evidence=str((E / 'decision-v1.json').relative_to(ROOT)),
        result='All 11075136 fallbacks completed with an invalid stored root bound; zero eligible signs, disagreements or anomalous tags. Fourteen commands verified: qualification passes, original instrumented full-board gate fails at 143.517755723 worker-CPU seconds / 120 with 12/197 findings. No bypass prototype is warranted for the observed workload.',
        counts=c, qualification=decision['qualification'], original_gate=decision['original_gate'],
        next_step=next_step)
    status_path.write_text(json.dumps(status, indent=2) + '\n')
    print(json.dumps({'status': decision['status'], 'fallbacks': c['started'], 'eligible': 0, 'runtime_admitted': False}))


if __name__ == '__main__':
    main()
