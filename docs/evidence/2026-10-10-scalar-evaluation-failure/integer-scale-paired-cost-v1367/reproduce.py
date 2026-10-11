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
OUT = E / 'integer-scale-paired-cost-v1367'
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
    source_record = E / 'typed-api-prototype-v1356-integer-scale-build/source-hashes.json.gz'
    source = json.loads(gzip.decompress(source_record.read_bytes()))
    main_guard = json.loads(gzip.decompress((E / 'candidate-preparation/main-hashes.json.gz').read_bytes()))
    assert sha(source_record) == '32b816463eacd2d2cef2f905c0455557c015117690a8a8dd635229a57726d2b2'
    def guard():
        assert all(sha(SOURCE / n) == v for n, v in source.items())
        assert all(sha(ROOT / n) == v for n, v in main_guard.items())
        assert sha(OUT / 'reproduce.py') == sha(Path(__file__))
    guard()
    rows = []
    provenance = {}
    results = {}
    for family, old, new in [
        ('integer', 'typed-api-prototype-v1298-integer-scale-baseline-cost', 'typed-api-prototype-v1357-integer-scale-cost'),
        ('inverse_trig', 'typed-api-prototype-v1240-inverse-constructor-inverse-cost', 'typed-api-prototype-v1358-integer-scale-inverse-cost'),
        ('normal_form', 'typed-api-prototype-v1241-inverse-constructor-normal-form-cost', 'typed-api-prototype-v1359-integer-scale-normal-form-cost'),
        ('default', 'typed-api-prototype-v1242-inverse-constructor-default-cost', 'typed-api-prototype-v1360-integer-scale-default-cost'),
        ('algebraic', 'typed-api-prototype-v1243-inverse-constructor-algebraic-cost', 'typed-api-prototype-v1361-integer-scale-algebraic-cost'),
    ]:
        phases = {'baseline': old, 'candidate': new}
        commands = {which: [r for r in read(E / phase / 'commands.json') if r['name'] not in ['format', 'build']]
                    for which, phase in phases.items()}
        assert len(commands['baseline']) == len(commands['candidate']) == (72 if family == 'integer' else 54)
        assert all(sha(E / old / n) == sha(E / new / n) for n in ['probe.rs', 'reproduce.py'])
        binaries = {which: Path(items[0]['command'][0]) for which, items in commands.items()}
        hashes = {which: sha(binary) for which, binary in binaries.items()}
        baseline = read(E/'integer-scale-baseline-v1298.json')
        retained = (baseline['retained_integer_cost'] if family == 'integer' else baseline['retained_other_costs'][family])['sha256']
        assert hashes['baseline'] == retained
        provenance[family] = {'phases': phases, 'binaries': {k: str(v) for k, v in binaries.items()},
                              'binary_sha256_at_paired_run': hashes, 'executable_bytes': {k: v.stat().st_size for k, v in binaries.items()},
                              'original_summary_sha256': {k: sha(E / p / 'summary.json') for k, p in phases.items()}}
        samples = {}
        for index, (a, b) in enumerate(zip(commands['baseline'], commands['candidate'])):
            assert a['name'] == b['name'] and a['command'][1:] == b['command'][1:]
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
    assert len(rows) == 576
    (OUT / 'summary.json').write_text(json.dumps({
        'source_record_sha256': sha(source_record), 'processes': len(rows), 'guards_unchanged': True,
        'provenance': provenance, 'cases': results,
        'scope': 'Supplemental alternating-order timing of retained executables produced by the frozen baseline/candidate harnesses. Executable bytes are hashed before and after every process. Baseline hashes additionally match the committed v1298 baseline record. Frozen harnesses bind source and library identities. The eight integer fixtures are distinct from the 24 ordinary scalar fixtures; no inputs or iteration counts change between baseline and candidate. This is an additional local timing observation, not a replacement for original trials or an allocation/binary-size/whole-program claim.',
        'artifacts_sha256': {str(p.relative_to(OUT)): sha(p) for p in OUT.iterdir() if p.is_file()},
    }, indent=2) + '\n')


if __name__ == '__main__':
    main()
