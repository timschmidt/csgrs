"""Check every direct allocation observation and the closed signed-word context."""
import gzip
import json
import re

from common_v1 import E, ROOT, sha, dump, verify


def counts(row):
    result = row['counts']
    histogram = {int(k): v for k, v in result['request_size_histogram'].items()}
    assert 512 not in histogram
    assert sum(histogram.values()) == result['calls']
    assert sum(k * v for k, v in histogram.items()) == result['requested_bytes']
    assert result['peak_bytes'] >= max(0, result['retained_bytes'])
    return result


def main():
    destination = E / 'assessment-v1.json'
    assert not destination.exists()
    out = E / 'run-v1'
    result = json.loads((out / 'result.json').read_text())
    assert result['complete'] and result['commands'] == 5 and result['fixed_processes'] == 2
    assert all(sha(out / p) == h for p, h in result['artifact_sha256'].items())
    protocol = json.loads((out / 'protocol.json').read_text())
    verify(protocol, full=True)
    assert protocol['process_orders'] == ['forward', 'reverse']
    assert (out / 'probe.rs').read_bytes() == (E / 'probe_v1.rs').read_bytes()
    fixture = ROOT / 'docs/evidence/2026-10-13-combined-native-admission/probes-v1/integer_probe.rs'
    source = fixture.read_text()
    body = source[source.index('fn integer('):source.index('fn fixture(')]
    values = re.findall(r'"(-?\d+)"', body)
    assert values == protocol['original_fixture_values'] and len(values) == 19
    signed = values[7:13]
    assert signed == protocol['original_signed_word_values']
    fits = sum(-(1 << 63) <= int(signed[i % 6]) < (1 << 63) for i in range(1000))
    assert fits == 501
    binary = json.loads((out / 'binary.json').read_text())
    assert sha(binary['path']) == binary['sha256']
    commands = json.loads((out / 'commands.json').read_text())
    assert [r['name'] for r in commands] == ['format', 'clippy', 'build', 'allocation-forward', 'allocation-reverse']
    assert all(r['exit_code'] == 0 for r in commands)
    raw = {}
    for command in commands:
        for stream in ['stdout', 'stderr']:
            path = out / (command['name'] + '.' + stream + '.gz')
            assert sha(path) == command[stream + '_sha256']
            raw[command['name'] + '.' + stream] = gzip.decompress(path.read_bytes())
    samples = json.loads((out / 'samples.json').read_text())
    assert [s['order'] for s in samples] == ['forward', 'reverse']
    methods = ['clone-magnitude', 'primitive-magnitude', 'original-leaf', 'signed-word-leaf']
    indexes, batches = [], []
    for sample in samples:
        rows = [json.loads(line) for line in raw['allocation-' + sample['order'] + '.stdout'].splitlines()
                if line.startswith(b'{')]
        assert rows == sample['rows'] and len(rows) == 74
        control = rows[0]
        assert control['passed'] and control['unwind_disabled']
        assert control['empty']['calls'] == 0 and control['allocated']['requested_bytes'] == 17
        order = methods if sample['order'] == 'forward' else methods[::-1]
        expected = [(value, method) for value in values for method in order
                    if method != 'primitive-magnitude' or abs(int(value)) < (1 << 128)]
        cases = [r for r in rows if 'case' in r]
        assert [(r['case'], r['method']) for r in cases] == expected and len(cases) == 71
        assert all(r['exact_value_verified'] for r in cases)
        indexes.append({(r['case'], r['method']): counts(r) for r in cases})
        batch = [r for r in rows if 'batch' in r]
        batch_order = ['original-leaf', 'signed-word-leaf']
        if sample['order'] == 'reverse':
            batch_order.reverse()
        assert [r['method'] for r in batch] == batch_order
        assert all(r['iterations'] == 1000 and r['signed_word_fits'] == fits for r in batch)
        batches.append({r['method']: counts(r) for r in batch})
    repeated = indexes[0] == indexes[1] and batches[0] == batches[1]
    direct = []
    for value in values:
        own = []
        for index in indexes:
            a, b = index[value, 'original-leaf'], index[value, 'signed-word-leaf']
            own.append({'original_leaf': a, 'signed_word_leaf': b,
                        'requested_byte_delta': b['requested_bytes'] - a['requested_bytes'],
                        'call_delta': b['calls'] - a['calls'],
                        'clone_magnitude': index[value, 'clone-magnitude'],
                        'primitive_magnitude': index.get((value, 'primitive-magnitude'))})
        direct.append({'value': value, 'fits_signed_word': -(1 << 63) <= int(value) < (1 << 63),
                       'processes': own})
    dump(E / 'direct-constructor-comparisons-v1.json', direct)
    deltas = [{field: batch['signed-word-leaf'][field] - batch['original-leaf'][field]
               for field in ['calls', 'requested_bytes', 'retained_bytes', 'peak_bytes']} for batch in batches]
    historical = []
    for name in ['2026-10-13-combined-native-admission', '2026-10-13-native-without-persistent-certificates']:
        path = ROOT / 'docs/evidence' / name / 'library-assessment-v1.json'
        assessment = json.loads(path.read_text())
        row = next(r for r in assessment['memory_rows'] if r['suite'] == 'integer'
                   and r['name'] == 'signed_word_boundary_construction')
        assert row['iterations'] == 1000
        historical.append({'assessment': str(path.relative_to(ROOT)), 'sha256': sha(path), 'row': row})
    predicted = fits * 24
    confirmed = repeated and all(d['requested_bytes'] == predicted and d['calls'] == 0
                                and d['retained_bytes'] == 0 and d['peak_bytes'] == 24 for d in deltas)
    for row in direct:
        if row['value'] not in signed:
            continue
        expected_delta = 24 if row['fits_signed_word'] else 0
        confirmed &= all(r['requested_byte_delta'] == expected_delta and r['call_delta'] == 0
                         and r['clone_magnitude']['request_size_histogram'] == {'8': 1}
                         and r['primitive_magnitude']['request_size_histogram'] == {'32': 1}
                         for r in row['processes'])
    assessment = {
        'status': 'signed-word-reconstruction-allocation-mechanism-confirmed' if confirmed else 'allocation-mechanism-not-confirmed',
        'assessor_sha256': sha(__file__), 'result_sha256': sha(out / 'result.json'),
        'comparison_sha256': sha(E / 'direct-constructor-comparisons-v1.json'),
        'fixed_processes': 2, 'case_method_rows': 142, 'batch_rows': 4,
        'all_counts_repeat_with_reversed_method_order': repeated,
        'all_exact_case_values_verified': True, 'signed_word_fits_in_1000': fits,
        'predicted_extra_requested_bytes': predicted, 'observed_batch_deltas': deltas,
        'historical_allocation_context': historical,
        'mechanism': 'On this 64-bit backend the signed-word magnitudes clone with one 8-byte request, whereas rebuilding the same magnitude through BigUint::from(u128) requests 32 bytes. Both Rational leaf paths use the same unmodified constructor implementation. The signed-word dispatch takes the rebuilding path for exactly 501 of the original 1000 boundary values.',
        'limits': ['This direct constructor diagnostic is not the whole affine fixture or a benchmark of a new runtime implementation.',
                   'The exact 12024-byte and 24-byte peak differences account numerically for the closed fixture excess. End-to-end repair still requires testing a new isolated source change against the unchanged original fixture.',
                   'No timing inference, prior-sample replacement, native admission or improvement claim follows from these allocation observations.'],
        'next_step': 'Prototype a crate-private borrowed-integer constructor that reuses only existing bounded canonical values and otherwise preserves the original BigInt-clone path; keep the cache boundary owned by Rational and exactness unbounded. Evaluate the whole existing cost matrix before retention.',
        'runtime_candidate': False, 'runtime_admitted': False, 'plan_complete': False,
    }
    dump(destination, assessment)
    print(json.dumps({k: assessment[k] for k in ['status', 'fixed_processes', 'case_method_rows',
        'all_counts_repeat_with_reversed_method_order', 'observed_batch_deltas', 'runtime_admitted']}))


if __name__ == '__main__':
    main()
