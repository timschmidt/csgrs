"""Bounded maximal-plane diagnostic with an independent nondegenerate anchor proof."""
import gzip
import hashlib
import importlib.util
import json
from pathlib import Path
import re
import subprocess
import time

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
OUT = E / 'sphere-zero-witnesses-v2'
WORK = ROOT / 'target/sphere-zero-witnesses-v2'
CASES = [('fifteen_degrees', 24, 12), ('original_medium', 32, 16), ('fifth_turn', 10, 7), ('seventh_turn', 7, 5)]
MAX_GROUPS = 128
CASE_SECONDS = 600


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def dump(path, value):
    path.write_text(json.dumps(value, indent=2) + '\n')


def main():
    assert not OUT.exists() and not WORK.exists()
    prior_dir = E / 'sphere-zero-witnesses-v1'
    prior = json.loads((prior_dir / 'result.json').read_text())
    assert prior['complete'] and len(prior['cases']) == 4
    assert all(sha(prior_dir / p) == h for p, h in prior['artifacts_sha256'].items())
    traced = E / 'sphere-orientation-trace-v1'
    result = json.loads((traced / 'result.json').read_text())
    assert result['complete'] and result['commands_passed'] == 7
    assert all(sha(traced / p) == h for p, h in result['artifacts_sha256'].items())
    proof = json.loads((E / 'sphere-orientation-exact-oracle-v1.json').read_text())
    assert proof['complete'] and all(row['exactly_zero'] for row in proof['checks'])
    assert proof['oracle_source_sha256'] == sha(E / 'check_sphere_trace_cyclotomic_v1.py')
    prep = json.loads((traced / 'protocol.json').read_text())
    source = Path(prep['source'])
    mapping = json.loads(gzip.decompress((traced / 'source-hashes.json.gz').read_bytes()))
    main_mapping = json.loads(gzip.decompress((E / 'mesh-identity-preparation-v1/source-hashes.json.gz').read_bytes()))
    root_sources = {p: sha(ROOT / p) for p in main_mapping}
    assert all(root_sources[p] == h for p, h in main_mapping.items() if p != 'docs/properties-status.json')
    libraries = json.loads((traced / 'libraries.json').read_text())
    inputs = {p: sha(E / p) for p in ['collect_sphere_zero_witnesses_v2.py', 'sphere_grid_oracle_probe_v2.rs',
        'check_sphere_trace_cyclotomic_v1.py', 'sphere-cyclotomic-oracle-audit-v1.json']}
    spec = importlib.util.spec_from_file_location('sphere_exact_oracle_v1', E / 'check_sphere_trace_cyclotomic_v1.py')
    oracle = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(oracle)

    def guard(full=False):
        assert all(sha(E / p) == h for p, h in inputs.items())
        assert all(sha(r['path']) == r['sha256'] for r in libraries.values())
        if full:
            assert all(sha(source / p) == h for p, h in mapping.items())
            assert all(sha(ROOT / p) == h for p, h in root_sources.items())

    guard(full=True)
    OUT.mkdir()
    WORK.mkdir()
    binary = WORK / 'probe'
    dump(OUT / 'protocol.json', {'inputs_sha256': inputs, 'source': str(source), 'libraries': libraries,
        'cases': CASES, 'prior_closed_trial_sha256': sha(prior_dir / 'result.json'),
        'mode': 'coordinate_symmetries', 'max_added_groups_per_case': MAX_GROUPS,
        'case_wall_budget_seconds': CASE_SECONDS, 'individual_query_wall_stop_seconds': 180,
        'algorithm': 'Run unchanged original input; if native orientation is unknown, independently reduce its exact cyclotomic determinant. For zero remainder, prove a nonzero anchor normal, collect every original point on that exact plane, and retry with this maximal group. Preserve every attempt and group file.',
        'not_a_performance_trial': True, 'not_a_native_certificate_carrier': True,
        'runtime_admitted': False, 'plan_complete': False})
    commands = []

    def run(name, command, timeout=180):
        guard()
        started = time.monotonic()
        with (OUT / (name + '.stdout')).open('w') as stdout, (OUT / (name + '.stderr')).open('w') as stderr:
            code = subprocess.run(['timeout', '--signal=TERM', '--kill-after=10s', str(timeout), *command],
                                  cwd=source, stdout=stdout, stderr=stderr).returncode
        row = {'name': name, 'command': command, 'exit_code': code, 'timeout_seconds': timeout,
            'process_wall_seconds': time.monotonic() - started,
            'stdout_sha256': sha(OUT / (name + '.stdout')), 'stderr_sha256': sha(OUT / (name + '.stderr'))}
        commands.append(row)
        dump(OUT / 'commands.json', commands)
        print(json.dumps(row), flush=True)
        guard()
        return code

    assert run('format', ['rustfmt', '--edition', '2024', '--check', str(E / 'sphere_grid_oracle_probe_v2.rs')]) == 0
    assert run('link', ['rustc', '--edition=2024', '-O', '-D', 'warnings', str(E / 'sphere_grid_oracle_probe_v2.rs'),
        '--extern', 'csgrs=' + libraries['csgrs']['path'], '--extern', 'hypermesh=' + libraries['hypermesh']['path'],
        '-L', 'dependency=' + str(Path(prep['target']) / 'release/deps'), '-o', str(binary)]) == 0
    binary_hash = sha(binary)
    cases = []
    for case, segments, stacks in CASES:
        field = oracle.Field(segments, stacks)
        count = segments * (stacks - 1) + 2
        points = [field.point(index) for index in range(count)]
        groups, records = [], []
        start = time.monotonic()
        outcome = 'group_budget_exhausted'
        for attempt in range(MAX_GROUPS + 1):
            remaining = CASE_SECONDS - (time.monotonic() - start)
            if remaining <= 0:
                outcome = 'case_wall_budget_exhausted'
                break
            assert sha(binary) == binary_hash
            name = f'{case}-{attempt:03d}'
            path = OUT / (name + '.groups')
            path.write_text(''.join(' '.join(map(str, group)) + '\n' for group in groups))
            code = run(name, [str(binary), case, 'coordinate_symmetries', str(path)], min(180, remaining))
            assert sha(path) == hashlib.sha256((''.join(' '.join(map(str, group)) + '\n' for group in groups)).encode()).hexdigest()
            if code:
                outcome = 'query_stop_or_failure'
                records.append({'attempt': attempt, 'exit_code': code, 'groups_sha256': sha(path)})
                break
            raw = (OUT / (name + '.stdout')).read_text()
            if 'certified=true' in raw:
                assert 'original_positions_preserved=true' in raw
                if case == 'fifteen_degrees':
                    assert 'prior_certified_rows_check=passed' in raw
                outcome = 'certified_with_independently_proven_groups'
                records.append({'attempt': attempt, 'certified': True, 'groups_sha256': sha(path)})
                break
            traces = re.findall(r'sphere-hull-undecided face=\[(\d+), (\d+), (\d+)\] point=(\d+) point_count=(\d+)',
                                (OUT / (name + '.stderr')).read_text())
            if len(traces) != 1:
                outcome = 'unhandled_native_failure'
                records.append({'attempt': attempt, 'native_trace_rows': traces, 'groups_sha256': sha(path)})
                break
            a, b, c, d, size = map(int, traces[0])
            assert size == count and len({a, b, c, d}) == 4
            assert all(0 <= i < count for i in [a, b, c, d])
            remainder = field.orient(*(points[i] for i in [a, b, c, d]))
            ab = [field.sub(x, y) for x, y in zip(points[b], points[a])]
            ac = [field.sub(x, y) for x, y in zip(points[c], points[a])]
            normal = [field.sub(field.mul(ab[i], ac[j]), field.mul(ab[j], ac[i]))
                      for i, j in [(1, 2), (2, 0), (0, 1)]]
            assert any(normal), 'A degenerate anchor cannot certify a common plane.'
            group = ([i for i, point in enumerate(points)
                      if not field.orient(points[a], points[b], points[c], point)]
                     if not remainder else sorted([a, b, c, d]))
            if not remainder:
                assert set([a, b, c, d]).issubset(group) and len(group) >= 4
            records.append({'attempt': attempt, 'indices': [a, b, c, d], 'group': group,
                'anchor_normal_polynomials': normal,
                'root_of_unity_order': field.order, 'determinant_remainder': remainder,
                'groups_sha256': sha(path), 'new_group_added': not remainder and attempt < MAX_GROUPS})
            if remainder:
                outcome = 'undecided_nonzero_determinant'
                break
            assert group not in groups
            if attempt < MAX_GROUPS:
                groups.append(group)
            dump(OUT / (case + '.witnesses.json'), records)
        row = {'case': case, 'outcome': outcome, 'added_groups': groups, 'records': records,
               'process_plus_oracle_wall_seconds': time.monotonic() - start}
        cases.append(row)
        dump(OUT / (case + '.result.json'), row)
        print(json.dumps({'case': case, 'outcome': outcome, 'groups': len(groups)}), flush=True)
    guard(full=True)
    assert sha(binary) == binary_hash
    dump(OUT / 'result.json', {'complete': True, 'commands': len(commands), 'cases': cases,
        'binary': {'path': str(binary), 'sha256': binary_hash, 'bytes': binary.stat().st_size},
        'all_sources_and_inputs_unchanged': True,
        'artifacts_sha256': {p.name: sha(p) for p in sorted(OUT.iterdir()) if p.is_file()},
        'runtime_admitted': False, 'sphere_certificate_pending': True, 'plan_complete': False})
    print(json.dumps({'complete': True, 'commands': len(commands),
        'outcomes': {c['case']: c['outcome'] for c in cases}}), flush=True)


if __name__ == '__main__':
    main()
