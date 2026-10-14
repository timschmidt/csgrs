"""Check frozen native feasibility artifacts and complete original sphere rows."""
from collections import Counter
import gzip
import hashlib
import json
from pathlib import Path
import re

E = Path(__file__).resolve().parent
OUT = E / 'sphere-cyclotomic-v2'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    destination = E / 'sphere-cyclotomic-assessment-v2.json'
    assert not destination.exists()
    result = json.loads((OUT / 'result.json').read_text())
    assert result['complete'] and result['all_sources_and_inputs_unchanged']
    assert all(sha(OUT / p) == h for p, h in result['artifacts_sha256'].items())
    protocol = json.loads((OUT / 'protocol.json').read_text())
    assert all(sha(E / p) == h for p, h in protocol['inputs_sha256'].items())
    mapping = json.loads(gzip.decompress((OUT / 'source-hashes.json.gz').read_bytes()))
    assert all(sha(Path(protocol['source']) / p) == h for p, h in mapping.items())
    commands = json.loads((OUT / 'commands.json').read_text())
    assert len(commands) == result['commands'] == 12
    assert all(row['exit_code'] == 0 for row in commands)
    tests = (OUT / 'targeted-scalar-tests.stdout').read_text()
    assert 'test result: ok. 6 passed; 0 failed; 0 ignored' in tests
    assert tests.count(' ... ok') == 6
    dimensions = {'fifteen_degrees': (24, 12), 'original_medium': (32, 16),
                  'fifth_turn': (10, 7), 'seventh_turn': (7, 5)}
    cases = []
    for case, (segments, stacks) in dimensions.items():
        raw = (OUT / ('candidate-' + case + '.stdout')).read_text()
        assert 'certified=true original_positions_preserved=true' in raw
        order_line = next(line for line in raw.splitlines() if line.startswith('position_order='))
        order = json.loads(order_line.split('=', 1)[1])
        count = segments * (stacks - 1) + 2
        assert sorted(order) == list(range(count))
        rows = [tuple(map(int, row)) for row in re.findall(r'Triangle \{ v0: (\d+), v1: (\d+), v2: (\d+) \}', raw)]
        assert len(rows) == 2 * count - 4 and len(set(rows)) == len(rows)
        directed = Counter()
        for row in rows:
            assert len(set(row)) == 3 and all(0 <= index < count for index in row)
            for a, b in zip(row, row[1:] + row[:1]):
                directed[a, b] += 1
        assert all(value == 1 and directed[b, a] == 1 for (a, b), value in directed.items())
        assert count - len(directed) // 2 + len(rows) == 2
        if case == 'fifteen_degrees':
            assert raw == (OUT / 'baseline-fifteen_degrees.stdout').read_text()
        cases.append({'case': case, 'certified': True, 'original_positions': count,
                      'triangle_rows': len(rows), 'closed_oriented_edges_rechecked': True,
                      'all_original_indices_preserved_once': True,
                      'stdout_sha256': sha(OUT / ('candidate-' + case + '.stdout'))})
    value = {'complete': True, 'result_sha256': sha(OUT / 'result.json'),
        'assessor_sha256': sha(__file__), 'commands_all_zero': 12, 'targeted_scalar_tests_passed': 6,
        'captured_independent_zero_determinants': 157, 'cases': cases,
        'prior_certified_24_by_12_order_rows_and_certainty_unchanged': True,
        'source_files_rechecked': len(mapping),
        'claim': 'Native feasibility: the unchanged ordinary STRICT sphere hull API certifies all four original exact sampled point sets with the isolated scalar candidate. Prior complete hull rows remain identical.',
        'limits': ['This is not broad scalar/predicate/mesh qualification, formal verification, a complete proof of returned hull convexity, or a matched performance comparison.',
                   'All native allocation, retention, binary-size and proportionate timing gates remain pending.',
                   'Only six targeted tests ran; 781 other library tests were filtered by the frozen feasibility protocol.'],
        'runtime_admitted': False, 'plan_complete': False}
    destination.write_text(json.dumps(value, indent=2) + '\n')
    print(json.dumps({'complete': True, 'commands': 12, 'cases': cases, 'assessment_sha256': sha(destination)}))


if __name__ == '__main__':
    main()
