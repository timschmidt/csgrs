#!/usr/bin/env python3
"""Freeze shared-DAG failures before changing the inverse-trig parsers."""
import difflib
import gzip
import hashlib
import json
from pathlib import Path
import subprocess

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT/'target/scalar-evaluation-candidate/source'
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()


def main():
    out = E/'inverse-dag-baseline-v941.json'
    assert not out.exists()
    previous = read(E/'typed-api-prototype-v927-state.json')
    for name, value in previous['artifacts_sha256'].items():
        assert sha(E/name) == value, name
    old_record = E/previous['source_record']
    before = json.loads(gzip.decompress(old_record.read_bytes()))
    record = E/'typed-api-prototype-v940-inverse-dag-allocation-build/source-hashes.json.gz'
    source = json.loads(gzip.decompress(record.read_bytes()))
    for name, value in source.items():
        assert sha(SOURCE/name) == value, name
    changed = {n: {'before': before.get(n), 'after': v}
               for n, v in source.items() if before.get(n) != v}
    prefix = 'crates/hyperreal/src/computable/'
    assert set(changed) == {prefix+n for n in
                           ['node.rs', 'node/algebraic_leaf_tests.rs', 'node/inverse_trig_dag_tests.rs']}
    node = prefix+'node.rs'
    assert (record.parent/'inputs'/node).read_text().replace(
        'include!("node/inverse_trig_dag_tests.rs");\n', '') == (old_record.parent/'inputs'/node).read_text()
    allocator = prefix+'node/algebraic_leaf_tests.rs'
    new_allocator = (record.parent/'inputs'/allocator).read_text()
    old_allocator = (old_record.parent/'inputs'/allocator).read_text()
    assert new_allocator.count('pub(super) ') == 4
    assert new_allocator.replace('pub(super) ', '') == old_allocator
    patch = E/'inverse-dag-test-helper-visibility-v941.patch'
    assert not patch.exists()
    patch.write_text(''.join(difflib.unified_diff(old_allocator.splitlines(True),
        new_allocator.splitlines(True), fromfile='before/'+allocator, tofile='after/'+allocator)))
    test = prefix+'node/inverse_trig_dag_tests.rs'
    initial = (E/'typed-api-prototype-v928-inverse-dag-regression-build/inputs'/test).read_text()
    current = (record.parent/'inputs'/test).read_text()
    assert initial.rstrip()[:-1].rstrip() == current.split(
        '    #[test]\n    fn inverse_trig_dag_allocation_growth_tracks_shared_nodes')[0].rstrip()
    rejected = (E/'typed-api-prototype-v938-inverse-dag-allocation-build/inputs'/test).read_text()
    assert rejected.replace('                assert!(allocations.calls > 0 && allocations.bytes > 0);\n', '') == current
    guard = json.loads(gzip.decompress((E/'candidate-preparation/main-hashes.json.gz').read_bytes()))
    for name, value in guard.items():
        assert sha(ROOT/name) == value, name
    expected = read(E.parent/'2026-10-10-artifact-numeric-keys/retention.json')['staged_verification']['preexisting_status']
    status = subprocess.check_output(['git', 'status', '--porcelain'], cwd=ROOT, text=True).splitlines()
    assert [s for s in status if not s[3:].startswith(str(E.relative_to(ROOT)))] == expected
    caps = read(E/'inverse-dag-baseline-caps-v937.json')
    assert len(caps['tests']) == 9
    allocation_phase = E/'typed-api-prototype-v941-inverse-dag-allocation-cap'
    result = read(allocation_phase/'result.json')
    assert result['exit_code'] == 101 and result['source_unchanged'] and result['main_unchanged']
    assert sha(allocation_phase/'source-hashes.json.gz') == sha(record)
    log = (allocation_phase/'stdout.log').read_text()
    cap = json.loads(log[log.rfind('{"command":'):])
    assert cap['address_space_bytes'] == 192*1024*1024 and cap['cpu_seconds_soft_hard'] == [2,3]
    assert cap['wall_seconds_limit'] == 4 and cap['helper_sha256'] == sha(E/'run_limited_unit_test.py')
    observations = [s for s in (allocation_phase/'stderr.log').read_text().splitlines()
                    if s.startswith('inverse-trig DAG allocation ')]
    assert len(observations) == 6
    costs = {}
    for family, p in read(E/'algebraic-leaf-paired-cost-v897/summary.json')['provenance'].items():
        binary = Path(p['binaries']['candidate'])
        assert sha(binary) == p['binary_sha256_at_paired_run']['candidate']
        assert binary.stat().st_size == p['executable_bytes']['candidate']
        assert sha(E/p['phases']['candidate']/'summary.json') == p['original_summary_sha256']['candidate']
        costs[family] = {'phase': p['phases']['candidate'], 'binary': str(binary),
                         'sha256': sha(binary), 'bytes': binary.stat().st_size}
    out.write_text(json.dumps({
        'status': 'Frozen ordinary inverse-trig DAG regressions before runtime repair; no runtime admission',
        'previous_checkpoint': 'typed-api-prototype-v927-state.json',
        'previous_verification': {'artifacts': len(previous['artifacts_sha256']),
            'candidate_sources': len(before), 'main_guarded': len(guard), 'original_worktree_entries': len(expected)},
        'source_record': str(record.relative_to(E)), 'source_record_sha256': sha(record),
        'test_only_changes': changed, 'prior_named_test_bodies_and_assertions_unchanged': True,
        'allocator_helper_visibility_patch': patch.name,
        'baseline_caps': 'inverse-dag-baseline-caps-v937.json',
        'outcomes': {'cpu_limit': 4, 'budget_assertion': 1, 'allocation_growth_assertion': 1, 'passing_guards': 4},
        'allocation_result': result, 'allocation_cap': cap, 'allocation_observations': observations,
        'allocation_scope': 'Opt-in thread-local Rust allocation requests during the synchronous parser call and coefficient extraction; excludes fixture setup, assertions, printing, native allocation and other threads. Not RSS or peak live bytes.',
        'collector_correction': 'The first collector stopped after preserving v929 because SIGXCPU left a partial test-output line before the JSON receipt. Its v2 fork reused that terminal phase after source/binary/helper/command/cap checks and located the receipt by its command key. No resource fixture was rerun or reduced.',
        'allocation_test_correction': 'v939 stopped on an invalid assertion that every query must allocate. Coefficient interning permits zero allocation for the linear acos parser. v940 removed only that positivity assertion; v941 preserves all six observations and fails the unchanged eightfold growth limit at depths 6 and 12.',
        'retained_cost_baselines': costs,
        'limits': ['No production source changed in this baseline.',
            'The main runtime, all 49 original worktree entries, all 39 properties, 14 targets and crate future work retain their previous scope and status.']
    }, indent=2)+'\n')
    print(json.dumps({'record': out.name, 'source_sha256': sha(record), 'artifacts_verified': len(previous['artifacts_sha256']), 'failing_regressions': 6, 'passing_guards': 4}))


if __name__ == '__main__':
    main()
