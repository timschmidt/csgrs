#!/usr/bin/env python3
"""Compare retained cost executables in alternating order with identical inputs."""
import gzip
import hashlib
import json
from pathlib import Path
import resource
import statistics
import subprocess
import time

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / 'target/scalar-evaluation-candidate/source'
OUT = E / 'atan-quotient-zero-paired-cost-v1562'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
read = lambda p: json.loads(p.read_text())


def limits():
    resource.setrlimit(resource.RLIMIT_AS, (192 * 1024 * 1024, 192 * 1024 * 1024))
    resource.setrlimit(resource.RLIMIT_CPU, (2, 3))
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))


def main():
    assert not OUT.exists()
    OUT.mkdir()
    (OUT / 'reproduce.py').write_bytes(Path(__file__).read_bytes())
    source_record = E / 'typed-api-prototype-v1554-atan-quotient-zero-release-build/source-hashes.json.gz'
    source = json.loads(gzip.decompress(source_record.read_bytes()))
    main_guard = json.loads(gzip.decompress((E / 'candidate-preparation/main-hashes.json.gz').read_bytes()))
    assert sha(source_record) == read(E/'atan-quotient-zero-caps-v1551.json')['source_record_sha256']
    def guard():
        assert all(sha(SOURCE / n) == v for n, v in source.items())
        assert all(sha(ROOT / n) == v for n, v in main_guard.items())
        assert sha(OUT / 'reproduce.py') == sha(Path(__file__))
    guard()
    rows = []
    provenance = {}
    results = {}
    for family, old, new in [
        ('atan', 'typed-api-prototype-v1514-atan-zero-term-baseline-cost', 'typed-api-prototype-v1555-atan-quotient-zero-cost'),
        ('acos', 'typed-api-prototype-v1418-acos-warm-baseline-cost', 'typed-api-prototype-v1561-atan-quotient-acos-cost'),
        ('integer', 'typed-api-prototype-v1357-integer-scale-cost', 'typed-api-prototype-v1556-atan-quotient-integer-cost'),
        ('inverse_trig', 'typed-api-prototype-v1358-integer-scale-inverse-cost', 'typed-api-prototype-v1557-atan-quotient-inverse-cost'),
        ('normal_form', 'typed-api-prototype-v1359-integer-scale-normal-form-cost', 'typed-api-prototype-v1558-atan-quotient-normal-form-cost'),
        ('default', 'typed-api-prototype-v1360-integer-scale-default-cost', 'typed-api-prototype-v1559-atan-quotient-default-cost'),
        ('algebraic', 'typed-api-prototype-v1361-integer-scale-algebraic-cost', 'typed-api-prototype-v1560-atan-quotient-algebraic-cost'),
    ]:
        phases = {'baseline': old, 'candidate': new}
        commands = {which: [r for r in read(E / phase / 'commands.json') if r['name'] not in ['format', 'build']]
                    for which, phase in phases.items()}
        assert len(commands['baseline']) == len(commands['candidate']) == (108 if family == 'atan' else 72 if family in {'integer', 'acos'} else 54)
        assert all(sha(E / old / n) == sha(E / new / n) for n in ['probe.rs', 'reproduce.py'])
        binaries = {which: Path(items[0]['command'][0]) for which, items in commands.items()}
        hashes = {which: sha(binary) for which, binary in binaries.items()}
        if family == 'atan':
            baseline = read(E/'atan-zero-term-baseline-v1515.json')
            retained = baseline['cost_binary_sha256']
            committed = subprocess.check_output(['git', 'show', '71c245225:'+str((E/old/'summary.json').relative_to(ROOT))], cwd=ROOT)
            assert committed == (E/old/'summary.json').read_bytes()
        elif family == 'acos':
            baseline = read(E/'acos-warm-baseline-v1422.json')
            retained = baseline['cost_binary_sha256']
            committed = subprocess.check_output(['git', 'show', '782329f0c:'+str((E/old/'summary.json').relative_to(ROOT))], cwd=ROOT)
            assert committed == (E/old/'summary.json').read_bytes()
        else:
            checkpoint = read(E/'typed-api-prototype-v1404-state.json')
            assert sha(E/old/'summary.json') == checkpoint['artifacts_sha256'][old+'/summary.json']
            prior = read(E/'integer-scale-paired-cost-v1367/summary.json')['provenance'][family]
            assert prior['phases']['candidate'] == old
            retained = prior['binary_sha256_at_paired_run']['candidate']
        assert hashes['baseline'] == retained
        candidate_summary = read(E/new/'summary.json')
        if 'binary_sha256' in candidate_summary:
            assert hashes['candidate'] == candidate_summary['binary_sha256']
        assert all(sha(Path(n)) == v for n, v in candidate_summary['libraries_sha256'].items())
        provenance[family] = {'phases': phases, 'binaries': {k: str(v) for k, v in binaries.items()},
                              'binary_sha256_at_paired_run': hashes, 'executable_bytes': {k: v.stat().st_size for k, v in binaries.items()},
                              'original_summary_sha256': {k: sha(E / p / 'summary.json') for k, p in phases.items()}}
        samples = {}
        for index, (a, b) in enumerate(zip(commands['baseline'], commands['candidate'])):
            assert a['name'] == b['name'] and a['command'][1:] == b['command'][1:]
            prior_output = read(E/old/(a['name']+'.stdout'))
            new_output = read(E/new/(b['name']+'.stdout'))
            assert {k: v for k, v in prior_output.items() if k != 'elapsed_ns'} == {
                k: v for k, v in new_output.items() if k != 'elapsed_ns'}
            assert a['timeout_seconds'] == b['timeout_seconds'] == 4
            assert a['address_space_cap_mib'] == b['address_space_cap_mib'] == 192
            assert a['cpu_seconds'] == b['cpu_seconds'] == [2, 3]
            fixture = tuple(a['command'][1:])
            bucket = samples.setdefault(fixture, {'baseline': [], 'candidate': []})
            for which in (['baseline', 'candidate'] if len(bucket['baseline']) % 2 == 0 else ['candidate', 'baseline']):
                command = commands[which][index]
                name = family + '-' + str(index) + '-' + which
                assert sha(binaries[which]) == hashes[which]
                started = time.monotonic()
                with (OUT / (name + '.stdout')).open('w') as stdout, (OUT / (name + '.stderr')).open('w') as stderr:
                    try:
                        result = subprocess.run(command['command'], cwd=ROOT, stdout=stdout, stderr=stderr,
                                                preexec_fn=limits, timeout=4)
                    except subprocess.TimeoutExpired:
                        (OUT/'failed-attempt.json').write_text(json.dumps({
                            'name': name, 'command': command['command'], 'exit_code': None,
                            'wall_timeout': True, 'wall_seconds': time.monotonic()-started,
                            'binary_sha256': hashes[which], 'address_space_bytes': 192*1024*1024,
                            'cpu_seconds_soft_hard': [2, 3], 'wall_seconds_limit': 4}, indent=2)+'\n')
                        raise
                if result.returncode != 0:
                    (OUT/'failed-attempt.json').write_text(json.dumps({
                        'name': name, 'command': command['command'], 'exit_code': result.returncode,
                        'wall_timeout': False, 'wall_seconds': time.monotonic()-started,
                        'binary_sha256': hashes[which], 'address_space_bytes': 192*1024*1024,
                        'cpu_seconds_soft_hard': [2, 3], 'wall_seconds_limit': 4}, indent=2)+'\n')
                assert result.returncode == 0, name
                actual = read(OUT / (name + '.stdout'))
                expected = read(E / phases[which] / (command['name'] + '.stdout'))
                assert {k: v for k, v in actual.items() if k != 'elapsed_ns'} == {k: v for k, v in expected.items() if k != 'elapsed_ns'}
                assert sha(binaries[which]) == hashes[which]
                bucket[which].append(int(actual['elapsed_ns']))
                rows.append({'name': name, 'command': command['command'], 'exit_code': result.returncode,
                             'address_space_bytes': 192 * 1024 * 1024, 'cpu_seconds_soft_hard': [2, 3],
                             'wall_seconds_limit': 4, 'wall_seconds': time.monotonic() - started,
                             'receipt': actual, 'binary_sha256': hashes[which]})
                (OUT / 'receipts.json').write_text(json.dumps(rows, indent=2) + '\n')
        results[family] = []
        for fixture, values in samples.items():
            assert all(len(v) == 9 for v in values.values())
            medians = {k: statistics.median(v) for k, v in values.items()}
            results[family].append({'arguments': list(fixture), 'elapsed_ns': values, 'median_ns': medians,
                                    'median_percent_change': 100 * (medians['candidate'] / medians['baseline'] - 1)})
        print(family, [round(v['median_percent_change'], 2) for v in results[family]], flush=True)
    guard()
    assert len(rows) == 936
    (OUT / 'summary.json').write_text(json.dumps({
        'source_record_sha256': sha(source_record), 'processes': len(rows), 'guards_unchanged': True,
        'provenance': provenance, 'cases': results,
        'scope': 'Supplemental alternating-order timing of retained, hash-matched executables with the same harnesses, inputs, order, iteration counts, caps and complete observed outputs. Baseline identity is checked against committed evidence. Twelve atan workloads (including dense/sparse input and one/64-root cache controls), eight coarse-warmed acos/cached controls, eight integer and 24 ordinary scalar fixtures are distinct workloads. All original unpaired trials remain retained. This is local timing and executable-size evidence, not a full-board or uniform-speedup claim.',
        'artifacts_sha256': {str(p.relative_to(OUT)): sha(p) for p in OUT.iterdir() if p.is_file()},
    }, indent=2) + '\n')


if __name__ == '__main__':
    main()
