#!/usr/bin/env python3
"""Compare retained cost executables in alternating order with identical inputs."""
import gzip
import hashlib
import json
import os
from pathlib import Path
import resource
import statistics
import subprocess
import time

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
SOURCE = ROOT / 'target/scalar-evaluation-candidate/source'
OUT = E / 'integer-coarse-zero-preconditioned-cost-v2100'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
read = lambda p: json.loads(p.read_text())


CPU = min(os.sched_getaffinity(0))


def limits():
    os.sched_setaffinity(0, {CPU})
    resource.setrlimit(resource.RLIMIT_AS, (192 * 1024 * 1024, 192 * 1024 * 1024))
    resource.setrlimit(resource.RLIMIT_CPU, (2, 3))
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))


def main():
    assert not OUT.exists()
    OUT.mkdir()
    (OUT / 'reproduce.py').write_bytes(Path(__file__).read_bytes())
    prelude_source = E/'acos_cost_cpu_prelude.rs'
    prelude = ROOT/'target/integer-coarse-zero-cost-cpu-prelude-v2100'
    assert not prelude.exists()
    (OUT/'prelude.rs').write_bytes(prelude_source.read_bytes())
    build = ['rustc', '--edition=2024', '-O', '-D', 'warnings', str(prelude_source), '-o', str(prelude)]
    compiled = subprocess.run(build, cwd=ROOT, capture_output=True, text=True, check=True)
    (OUT/'prelude-build.stdout').write_text(compiled.stdout)
    (OUT/'prelude-build.stderr').write_text(compiled.stderr)
    prelude_hash = sha(prelude)
    (OUT/'preparation.json').write_text(json.dumps({'cpu':CPU,'available_cpus':sorted(os.sched_getaffinity(0)),
        'build_command':build,'prelude_sha256':prelude_hash,'prelude_source_sha256':sha(prelude_source),
        'iterations':50000000,'timed_query_inputs_and_caps_unchanged':True},indent=2)+'\n')
    prelude_receipts = []
    source_record = E / 'typed-api-prototype-v2086-integer-coarse-zero-release-build/source-hashes.json.gz'
    source = json.loads(gzip.decompress(source_record.read_bytes()))
    main_guard = json.loads(gzip.decompress((E / 'candidate-preparation/main-hashes.json.gz').read_bytes()))
    assert sha(source_record) == sha(E/'typed-api-prototype-v2082-integer-coarse-zero-verus/source-hashes.json.gz')
    def guard():
        assert all(sha(SOURCE / n) == v for n, v in source.items())
        assert all(sha(ROOT / n) == v for n, v in main_guard.items())
        assert sha(OUT / 'reproduce.py') == sha(Path(__file__))
    guard()
    rows = []
    provenance = {}
    results = {}
    retained_artifacts = read(E/'typed-api-prototype-v1836-state.json')['artifacts_sha256']
    retained_cost_record = E/'domain-predicate-cost-review-v1780.json'
    assert sha(retained_cost_record) == retained_artifacts[retained_cost_record.name]
    retained_costs = read(retained_cost_record)
    for family, old, new in [
        ('atan', 'typed-api-prototype-v1773-domain-predicate-atan-cost', 'typed-api-prototype-v2093-integer-coarse-zero-atan-cost'),
        ('acos', 'typed-api-prototype-v1779-domain-predicate-acos-cost', 'typed-api-prototype-v2099-integer-coarse-zero-acos-cost'),
        ('integer', 'typed-api-prototype-v1774-domain-predicate-integer-cost', 'typed-api-prototype-v2094-integer-coarse-zero-integer-cost'),
        ('inverse_trig', 'typed-api-prototype-v1775-domain-predicate-inverse-cost', 'typed-api-prototype-v2095-integer-coarse-zero-inverse-cost'),
        ('normal_form', 'typed-api-prototype-v1776-domain-predicate-normal-form-cost', 'typed-api-prototype-v2096-integer-coarse-zero-normal-form-cost'),
        ('default', 'typed-api-prototype-v1777-domain-predicate-default-cost', 'typed-api-prototype-v2097-integer-coarse-zero-default-cost'),
        ('algebraic', 'typed-api-prototype-v1778-domain-predicate-algebraic-cost', 'typed-api-prototype-v2098-integer-coarse-zero-algebraic-cost'),
    ]:
        phases = {'baseline': old, 'candidate': new}
        commands = {which: [r for r in read(E / phase / 'commands.json') if r['name'] not in ['format', 'build']]
                    for which, phase in phases.items()}
        assert len(commands['baseline']) == len(commands['candidate']) == (108 if family == 'atan' else 72 if family in {'integer', 'acos'} else 54)
        assert all(sha(E / old / n) == sha(E / new / n) for n in ['probe.rs', 'reproduce.py'])
        binaries = {which: Path(items[0]['command'][0]) for which, items in commands.items()}
        hashes = {which: sha(binary) for which, binary in binaries.items()}
        prior = retained_costs['families'][family]
        assert prior['phases']['candidate'] == old
        assert sha(E/old/'summary.json') == retained_artifacts[old+'/summary.json']
        retained = prior['binary_sha256']['candidate']
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
                assert sha(prelude) == prelude_hash
                prepared = subprocess.run([str(prelude)], cwd=ROOT, capture_output=True, text=True,
                    preexec_fn=limits, timeout=4, check=True)
                prelude_receipts.append({'before':name,'stdout':prepared.stdout,'stderr':prepared.stderr,'exit_code':prepared.returncode})
                (OUT/'prelude-receipts.json').write_text(json.dumps(prelude_receipts,indent=2)+'\n')
                assert prepared.stderr == '' and len({row['stdout'] for row in prelude_receipts}) == 1
                assert sha(prelude) == prelude_hash
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
        'cpu_preparation': {'cpu':CPU,'prelude_sha256':prelude_hash,'prelude_processes':len(prelude_receipts)},
        'scope': 'Predeclared fixed-CPU comparison with an identical 50-million-step dependent integer prelude before each process. All earlier observations remain retained. Alternating-order timing of retained, hash-matched executables with the same harnesses, inputs, order, iteration counts, caps and complete observed outputs. Baseline is the retained v1836 domain-predicate candidate, checked against its committed checkpoint and binary provenance. Earlier original-cap failures at v1638 and v1710 and the narrow integer cost tradeoff remain separate retained evidence. This proof-only extraction has no new completeness benefit to justify a runtime regression. Twelve atan workloads (including dense/sparse input and one/64-root cache controls), eight coarse-warmed acos/cached controls, eight integer and 24 ordinary scalar fixtures are distinct workloads. All original unpaired trials remain retained. This is local timing and executable-size evidence, not a full-board or uniform-speedup claim.',
        'artifacts_sha256': {str(p.relative_to(OUT)): sha(p) for p in OUT.iterdir() if p.is_file()},
    }, indent=2) + '\n')


if __name__ == '__main__':
    main()
