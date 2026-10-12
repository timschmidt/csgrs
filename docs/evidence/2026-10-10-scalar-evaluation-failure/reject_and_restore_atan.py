"""Preserve both atan experiments and restore the retained source exactly."""
import gzip
import hashlib
import json
from pathlib import Path

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT/'target/scalar-evaluation-candidate/source'
read = lambda p: json.loads(p.read_text())
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
hashes = lambda p: json.loads(gzip.decompress(p.read_bytes()))


def main():
    complete = read(E/'atan-quotient-zero-preconditioned-cost-v1563/summary.json')
    focused = read(E/'atan-quotient-focused-cost-v1569/summary.json')
    expected = 'c789297f8e3463ae9b76b28e48ad3106bb59426c9d96783fcdb57b9d154fe23d'
    assert complete['source_record_sha256'] == focused['source_record_sha256'] == expected
    assert complete['processes'] == complete['cpu_preparation']['prelude_processes'] == 936
    assert focused['processes'] == focused['cpu_preparation']['prelude_processes'] == 216
    assert complete['guards_unchanged'] and focused['guards_unchanged']
    for family in focused['provenance']:
        assert focused['provenance'][family] == complete['provenance'][family]
    old = next(r for r in complete['cases']['integer'] if r['arguments'] == ['wide_coarse', '10000'])
    new = focused['cases']['integer'][0]
    assert new['arguments'] == old['arguments']
    assert old['median_percent_change'] > 5 and new['median_percent_change'] > 5
    for directory, data in [('atan-quotient-zero-preconditioned-cost-v1563', complete),
                            ('atan-quotient-focused-cost-v1569', focused)]:
        assert all(sha(E/directory/n) == h for n, h in data['artifacts_sha256'].items())
    rejected = E/'atan-quotient-rejection-v1571.json'
    assert not rejected.exists()
    rejected.write_text(json.dumps({
        'status': 'Rejected isolated quotient guard; no runtime admission',
        'candidate_commit': '8c70eab73', 'source_record_sha256': expected,
        'candidate_record': 'atan-quotient-implementation-v1568.json',
        'complete_controlled_record': 'atan-quotient-zero-preconditioned-cost-v1563/summary.json',
        'focused_protocol': 'atan-quotient-focused-protocol-v1570.json',
        'focused_record': 'atan-quotient-focused-cost-v1569/summary.json',
        'persistent_integer_wide_coarse_increase': {
            'complete_nine_repetition_percent': old['median_percent_change'],
            'focused_27_repetition_percent': new['median_percent_change']},
        'complete_cases': complete['cases'], 'focused_cases': focused['cases'],
        'decision': 'The quotient guard preserves frozen correctness, budget and cache observations and removes the unused wide square. The wide-coarse integer control remains slower in the complete and focused fixed-CPU comparisons, so the variant is rejected under the no-significant-regression gate. The wide-pi increase does not persist in the focused run. No causal attribution is established.',
        'limits': ['All original, complete and focused timings remain retained. The focused run covers only its four predeclared fixtures.',
                   'The prepared unconditioned v1562 comparison was never executed. No result is claimed for it.',
                   'No full-stack/replay/proof rerun or admission is claimed for either rejected atan variant.']
    }, indent=2)+'\n')

    candidate_record = E/'typed-api-prototype-v1554-atan-quotient-zero-release-build/source-hashes.json.gz'
    retained_record = E/'typed-api-prototype-v1497-restored-build/source-hashes.json.gz'
    assert sha(candidate_record) == expected
    assert sha(retained_record) == '32b816463eacd2d2cef2f905c0455557c015117690a8a8dd635229a57726d2b2'
    candidate, retained = hashes(candidate_record), hashes(retained_record)
    assert all(sha(SOURCE/n) == h for n, h in candidate.items())
    main_guard = hashes(E/'candidate-preparation/main-hashes.json.gz')
    assert all(sha(ROOT/n) == h for n, h in main_guard.items())
    changed = sorted(n for n in candidate.keys() | retained.keys() if candidate.get(n) != retained.get(n))
    assert changed == ['crates/hyperreal/src/computable/approximation/inverse_trig.rs',
                       'crates/hyperreal/src/computable/node.rs',
                       'crates/hyperreal/src/computable/node/atan_kernel_tests.rs',
                       'crates/hyperreal/src/computable/node/atan_numeric_probe_tests.rs']
    actions = []
    for name in changed:
        assert sha(candidate_record.parent/'inputs'/name) == candidate[name]
        if name in retained:
            archived = retained_record.parent/'inputs'/name
            assert sha(archived) == retained[name]
            (SOURCE/name).write_bytes(archived.read_bytes())
            action = 'restore exact retained bytes'
        else:
            (SOURCE/name).unlink()
            action = 'remove only the newly added, archived test source from the isolated candidate'
        actions.append({'path': name, 'before_sha256': candidate[name],
                        'after_sha256': retained.get(name), 'action': action})
    assert all(sha(SOURCE/n) == h for n, h in retained.items())
    assert all(not (SOURCE/n).exists() for n in candidate.keys()-retained.keys())
    out = E/'atan-source-restoration-v1572.json'
    assert not out.exists()
    out.write_text(json.dumps({'status': 'Both rejected variants remain archived; retained v1404/v1500 source restored',
        'rejection_records': ['atan-magnitude-guard-rejection-v1541.json', rejected.name],
        'candidate_source_sha256': expected, 'retained_source_sha256': sha(retained_record),
        'retained_source_record': str(retained_record.relative_to(E)), 'restoration_actions': actions,
        'retained_source_files_verified': len(retained), 'main_guarded_files': len(main_guard),
        'new_test_functions_archived': 9, 'older_retained_test_sources_unchanged': True,
        'next_step': 'Refresh the restored scalar tests, release libraries and formatting; no broader fresh-validation claim.'}, indent=2)+'\n')
    print(json.dumps({'rejection': rejected.name, 'restoration': out.name,
        'persistent_percent_increases': [old['median_percent_change'], new['median_percent_change']]}))


if __name__ == '__main__':
    main()
