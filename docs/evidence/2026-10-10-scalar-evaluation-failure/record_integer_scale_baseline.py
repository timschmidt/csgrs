#!/usr/bin/env python3
"""Preserve the integer-copy baseline, corrected new test and unchanged runtime."""
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
    out = E/'integer-scale-baseline-v1298.json'
    assert not out.exists()
    previous = read(E/'typed-api-prototype-v1283-state.json')
    for name, digest in previous['artifacts_sha256'].items():
        assert sha(E/name) == digest, name
    prior = E/previous['source_record']
    before = json.loads(gzip.decompress(prior.read_bytes()))
    record = E/'typed-api-prototype-v1297-integer-scale-baseline-release/source-hashes.json.gz'
    source = json.loads(gzip.decompress(record.read_bytes()))
    for name, digest in source.items():
        assert sha(SOURCE/name) == digest, name
    changed = {n: {'before': before.get(n), 'after': v} for n, v in source.items() if before.get(n) != v}
    node = 'crates/hyperreal/src/computable/node.rs'
    tests = 'crates/hyperreal/src/computable/node/integer_scale_tests.rs'
    assert set(changed) == {node, tests}
    assert (SOURCE/node).read_text().replace('include!("node/integer_scale_tests.rs");\n', '') == (prior.parent/'inputs'/node).read_text()
    assert all(source[n] == v for n, v in before.items() if 'test' in n)
    initial = E/'typed-api-prototype-v1285-integer-scale-baseline-build/inputs'/tests
    patch = E/'integer-scale-test-correction-v1298.patch'
    assert not patch.exists()
    patch.write_text(''.join(difflib.unified_diff(initial.read_text().splitlines(True),
        (SOURCE/tests).read_text().splitlines(True), fromfile='initial/'+tests, tofile='corrected/'+tests)))
    caps = read(E/'integer-scale-baseline-caps-v1296.json')
    assert caps['source_record_sha256'] == sha(record)
    codes, observations = [], []
    for phase, row in caps['tests'].items():
        result = read(E/phase/'result.json')
        assert result == row['result'] and result['source_unchanged'] and result['main_unchanged']
        assert sha(E/phase/'source-hashes.json.gz') == sha(record)
        cap = row['cap']
        assert cap['address_space_bytes'] == 192*1024*1024
        assert cap['cpu_seconds_soft_hard'] == [2, 3] and cap['wall_seconds_limit'] == 4
        assert not cap['wall_timeout'] and cap['exit_code'] == result['exit_code']
        codes.append(result['exit_code'])
        observations += [s for s in (E/phase/'stderr.log').read_text().splitlines() if s.startswith('integer scale allocation ')]
    assert codes == [101, 101, 0, 0, 0] and len(observations) == 64
    cost = E/'typed-api-prototype-v1298-integer-scale-baseline-cost'
    summary = read(cost/'summary.json')
    assert summary['source_hashes_sha256'] == sha(record) and summary['guards_unchanged'] and summary['processes'] == 72
    binary = ROOT/'target'/cost.name/'probe'
    assert sha(binary) == summary['binary_sha256']
    retained = {}
    for family, provenance in read(E/'inverse-constructor-paired-cost-v1248/summary.json')['provenance'].items():
        path = Path(provenance['binaries']['candidate'])
        assert sha(path) == provenance['binary_sha256_at_paired_run']['candidate']
        retained[family] = {'phase': provenance['phases']['candidate'], 'binary': str(path),
            'sha256': sha(path), 'bytes': path.stat().st_size}
    guard = json.loads(gzip.decompress((E/'candidate-preparation/main-hashes.json.gz').read_bytes()))
    for name, digest in guard.items():
        assert sha(ROOT/name) == digest, name
    expected = read(E.parent/'2026-10-10-artifact-numeric-keys/retention.json')['staged_verification']['preexisting_status']
    status = subprocess.check_output(['git', 'status', '--porcelain'], cwd=ROOT, text=True).splitlines()
    assert [s for s in status if not s[3:].startswith(str(E.relative_to(ROOT)))] == expected
    out.write_text(json.dumps({
        'status': 'Frozen avoidable-copy baseline; no production edits or admission',
        'previous_checkpoint': 'typed-api-prototype-v1283-state.json',
        'verified': {'artifacts': len(previous['artifacts_sha256']), 'prior_sources': len(before),
            'current_sources': len(source), 'main_guarded': len(guard), 'original_worktree_entries': len(expected)},
        'source_record': str(record.relative_to(E)), 'source_record_sha256': sha(record),
        'test_only_changes': changed, 'previous_tests_unchanged': True,
        'baseline_caps': 'integer-scale-baseline-caps-v1296.json', 'exit_codes_in_test_order': codes,
        'allocation_observations': observations,
        'preparation': {
            'v1284': 'Initial new-test compilation needed two explicit BigInt annotations; failed phase retained.',
            'v1285_through_v1290': 'Two allocation failures and two behavior passes; the new sign-reuse guard incorrectly required a cold integer to have a retained sign proof.',
            'correction': 'Before requiring zero-work sign reuse, query the unlimited exact sign. A cold zero-work fact may be Unknown, and every definite answer must agree. This follows the accepted P4 contract. The failed test and exact correction are retained; no allocation assertion or old test changed.',
            'correction_patch': patch.name},
        'finding': 'Positive and negative million-bit inputs request at least 125008 Rust allocation bytes in direct dispatch and at least 125048 through the one-unit public wrapper, despite retaining only 0 or +/-1. Public and raw exact rounding agree with the independent division/remainder oracle.',
        'classification': 'Avoidable copying of discarded integer limbs. EvaluationBudget explicitly excludes individual big-integer operation, byte and time bounds; this finding does not establish an allowance-contract violation.',
        'allocation_scope': 'Synchronous thread-local Rust allocation requests, excluding fixtures and oracle creation; not native allocations, RSS or peak-live memory.',
        'retained_integer_cost': {'phase': cost.name, 'binary': str(binary), 'sha256': sha(binary), 'bytes': binary.stat().st_size, 'processes': 72},
        'retained_other_costs': retained,
        'limits': ['All prior artifacts and failures remain immutable.', 'Full 39-property, 14-target plan and crate future work remain open.']
    }, indent=2)+'\n')
    print(json.dumps({'record': out.name, 'verified_artifacts': len(previous['artifacts_sha256']), 'codes': codes}))


if __name__ == '__main__':
    main()
