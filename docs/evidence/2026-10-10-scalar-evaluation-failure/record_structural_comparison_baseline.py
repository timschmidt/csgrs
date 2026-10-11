#!/usr/bin/env python3
"""Freeze payload-cost failures and the unchanged comparison oracle."""
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
    out = E/'structural-comparison-baseline-v1024.json'
    assert not out.exists()
    previous = read(E/'typed-api-prototype-v1011-state.json')
    for name, value in previous['artifacts_sha256'].items():
        assert sha(E/name) == value, name
    old_record = E/previous['source_record']
    before = json.loads(gzip.decompress(old_record.read_bytes()))
    record = E/'typed-api-prototype-v1012-structural-comparison-baseline-build/source-hashes.json.gz'
    source = json.loads(gzip.decompress(record.read_bytes()))
    for name, value in source.items():
        assert sha(SOURCE/name) == value, name
    changed = {n: {'before': before.get(n), 'after': v} for n, v in source.items() if before.get(n) != v}
    prefix = 'crates/hyperreal/src/computable/'
    assert set(changed) == {prefix+'node.rs', prefix+'node/structural_comparison_budget_tests.rs'}
    node = prefix+'node.rs'
    assert (SOURCE/node).read_text().replace('include!("node/structural_comparison_budget_tests.rs");\n', '') == (old_record.parent/'inputs'/node).read_text()
    guard = json.loads(gzip.decompress((E/'candidate-preparation/main-hashes.json.gz').read_bytes()))
    for name, value in guard.items():
        assert sha(ROOT/name) == value, name
    expected = read(E.parent/'2026-10-10-artifact-numeric-keys/retention.json')['staged_verification']['preexisting_status']
    status = subprocess.check_output(['git', 'status', '--porcelain'], cwd=ROOT, text=True).splitlines()
    assert [s for s in status if not s[3:].startswith(str(E.relative_to(ROOT)))] == expected
    caps = read(E/'structural-comparison-baseline-caps-v1024.json')
    assert len(caps['tests']) == 12
    assert caps['source_record_sha256'] == sha(record)
    observations = {}
    codes = []
    for phase, row in caps['tests'].items():
        result = read(E/phase/'result.json')
        assert result == row['result'] and result['source_unchanged'] and result['main_unchanged']
        assert sha(E/phase/'source-hashes.json.gz') == sha(record)
        assert row['cap']['address_space_bytes'] == 192*1024*1024
        assert row['cap']['cpu_seconds_soft_hard'] == [2, 3] and row['cap']['wall_seconds_limit'] == 4
        assert row['cap']['helper_sha256'] == sha(E/'run_limited_unit_test.py')
        codes.append(result['exit_code'])
        observations[row['test']] = [s for s in (E/phase/'stderr.log').read_text().splitlines()
            if s.startswith(('wide ', 'lazy ', 'cross product ', 'shift rejection ', 'comparison cutpoints ', 'public comparison '))]
    assert codes == [101, 101, 101, 101, 0, 101, 101, 101, 101, 0, 101, 0]
    assert len(observations['wide_structural_integer_comparison_requires_payload_work']) == 32
    assert len(observations['wide_structural_rational_variants_require_payload_work']) == 136
    costs = {}
    for family, p in read(E/'inverse-dag-paired-cost-completion-v980/summary.json')['provenance'].items():
        binary = Path(p['binaries']['candidate'])
        assert sha(binary) == p['binary_sha256_at_paired_run']['candidate']
        assert binary.stat().st_size == p['executable_bytes']['candidate']
        assert sha(E/p['phases']['candidate']/'summary.json') == p['original_summary_sha256']['candidate']
        costs[family] = {'phase': p['phases']['candidate'], 'binary': str(binary),
            'sha256': sha(binary), 'bytes': binary.stat().st_size}
    out.write_text(json.dumps({
        'status': 'Frozen structural-comparison and lazy rational preflight regressions before runtime repair; no runtime admission',
        'previous_checkpoint': 'typed-api-prototype-v1011-state.json',
        'previous_verification': {'artifacts': len(previous['artifacts_sha256']), 'candidate_sources': len(before),
            'main_guarded': len(guard), 'original_worktree_entries': len(expected)},
        'source_record': str(record.relative_to(E)), 'source_record_sha256': sha(record),
        'test_only_changes': changed, 'prior_named_test_bodies_and_assertions_unchanged': True,
        'baseline_caps': 'structural-comparison-baseline-caps-v1024.json',
        'outcomes': {'assertion_failures': 9, 'passing_guards': 3},
        'observations': observations,
        'findings': [
            'All 32 integer and 136 rational-variant observations return a definite comparison with one node unit, regardless of payload width.',
            'Fresh lazy rational preflight reduces a million-bit stored numerator and denominator before charging one unit: 25 Rust allocation requests totaling 2500392 bytes in the measured call.',
            'The million-bit unequal rational comparison requests 6086 Rust allocations totaling 10817536 bytes at an allowance of one unit.',
            'Different logarithm shifts still compare rational payloads first; the 4097-bit case requests 5248 bytes despite the cheap metadata rejection.',
            'A public cold sign query cancels independent 4097-bit atan arguments in 23 units. The regression requires payload-aware interruption, successful retry, and preservation of the completed zero proof at zero work.',
            'Identity/sign/bit-width guards, every existing comparison cutpoint with retries, and shared-DAG equality already pass.'
        ],
        'allocation_scope': 'Opt-in thread-local Rust allocation requests only during the named synchronous call. Fixture setup, assertions, printing, native allocation and other threads are excluded. This is neither RSS nor peak-live memory.',
        'coverage_limit': 'Tests that assert in a loop stop at the first failure. Retained logs state the observations actually reached; later widths remain in the frozen tests for the repair run.',
        'retained_cost_baselines': costs,
        'limits': ['No production runtime source changed in this baseline.',
            'Cooperative work units are not instruction, elapsed-time or byte bounds.',
            'The main runtime, all 49 original worktree entries, all 39 properties, 14 targets and crate future work retain their previous scope and status.']
    }, indent=2)+'\n')
    print(json.dumps({'record': out.name, 'source_sha256': sha(record),
        'artifacts_verified': len(previous['artifacts_sha256']), 'failing_regressions': 9, 'passing_guards': 3}))


if __name__ == '__main__':
    main()
