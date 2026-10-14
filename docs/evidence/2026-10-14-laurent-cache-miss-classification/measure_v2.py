"""Qualify one diagnostic overlay, then collect two fixed reversed count passes."""
import gzip
import json
import os
from pathlib import Path
import re
import subprocess

from miss_classification_common_v2 import E, ROOT, PARENT, WORK, PREP, ENV, GROUPS, ORDERS, SUITES, sha, dump, verify


def rows(raw):
    return [json.loads(line) for line in raw.splitlines() if line.startswith(b'{')]


def identity(raw):
    return [r for r in rows(raw) if 'fixture' in r or 'witness' in r]


def test_names(raw):
    return set(re.findall(rb'^test ([^ ]+) \.\.\. ok$', raw, re.MULTILINE))


def main():
    prep = json.loads((PREP / 'protocol.json').read_text())
    verify(prep, full=True)
    review = json.loads((E / 'source-review-v2.json').read_text())
    assert review['preparation_sha256'] == sha(PREP / 'protocol.json')
    assert review['all_five_source_paths_reviewed'] and review['all_three_drivers_reviewed']
    assert review['source_patch_sha256'] == prep['source_patch_sha256']
    assert prep['compiler'] == subprocess.check_output(['rustc', '-vV'], text=True)
    assert prep['inherited_rustflags'] == os.environ.get('RUSTFLAGS')
    source, target = Path(prep['source']), Path(prep['target'])
    out = E / 'diagnostic-v2'
    out.mkdir(exist_ok=False)
    bindir = WORK / 'binaries'
    bindir.mkdir(exist_ok=False)
    protocol = {'scope': __doc__, 'runner_sha256': sha(__file__), 'assessor_sha256': sha(E / 'assess_v2.py'),
        'preparation_sha256': sha(PREP / 'protocol.json'), 'review_sha256': sha(E / 'source-review-v2.json'),
        'groups': GROUPS, 'suites': SUITES, 'orders': ORDERS, 'inputs': 4096,
        'commands': 53, 'fixed_count_processes': 24, 'trace_windows': 544, 'classified_windows': 136,
        'environment': ENV | {'CARGO_TARGET_DIR': str(target)}, 'timeout_seconds': 1800,
        'compiler': prep['compiler'], 'inherited_rustflags': os.environ.get('RUSTFLAGS'),
        'new_library_builds': 1, 'new_runtime_sources': 0, 'new_diagnostic_sources': 1,
        'no_timing_or_allocation_comparison': True, 'runtime_admitted': False, 'plan_complete': False}
    dump(out / 'protocol.json', protocol)
    commands, binaries, samples = [], {}, []
    libraries = dict(prep['reused_libraries'])

    def guard(full=False):
        verify(prep, full=full)
        assert sha(E / 'source-review-v2.json') == protocol['review_sha256']
        assert all(sha(row['path']) == row['sha256'] for row in binaries.values())
        assert all(sha(row['path']) == row['sha256'] for group in libraries.values() for row in group.values())

    def run(name, command, cwd=source):
        guard()
        print(json.dumps({'start': name}), flush=True)
        done = subprocess.run(['timeout', '--signal=TERM', '--kill-after=10s', '1800', *command],
            cwd=cwd, env=os.environ | protocol['environment'], capture_output=True)
        row = {'name': name, 'command': command, 'cwd': str(cwd), 'exit_code': done.returncode}
        for stream in ['stdout', 'stderr']:
            path = out / (name + '.' + stream + '.gz')
            path.write_bytes(gzip.compress(getattr(done, stream), mtime=0))
            row[stream + '_sha256'] = sha(path)
        commands.append(row)
        dump(out / 'commands.json', commands)
        print(json.dumps({'finished': name, 'exit_code': done.returncode}), flush=True)
        assert done.returncode == 0, name
        guard()
        return done.stdout

    run('format-header', ['rustfmt', '--check', '--edition', '2024', '--config-path',
        str(source / 'crates/.rustfmt.toml'), str(E / 'trace_header_v1.rs')])
    run('clippy-library', ['cargo', 'clippy', '--locked', '--release', '-p', 'hyperreal',
        '--all-features', '--all-targets', '--', '-D', 'warnings'])
    raw = run('build-tests', ['cargo', 'test', '--locked', '--release', '-p', 'hyperreal',
        '--all-features', '--lib', '--no-run', '--message-format=json'])
    artifacts = [r for r in rows(raw) if r.get('reason') == 'compiler-artifact' and r.get('executable')
        and r['target']['name'] == 'hyperreal' and r['target']['kind'] == ['lib'] and r['profile']['test']]
    assert len(artifacts) == 1
    artifact = artifacts[0]
    path = Path(artifact['executable'])
    assert path.is_relative_to(target) and Path(artifact['manifest_path']) == source / 'crates/hyperreal/Cargo.toml'
    assert artifact['features'] == libraries['borrowed']['hyperreal']['features']
    binaries['library-tests'] = {'path': str(path), 'sha256': sha(path), 'bytes': path.stat().st_size,
        'features': artifact['features'], 'manifest_path': artifact['manifest_path']}
    dump(out / 'binaries.json', binaries)
    raw = run('build-library', ['cargo', 'build', '--locked', '--release', '-p', 'hyperreal',
        '--all-features', '--lib', '--message-format=json'])
    libraries['classified'] = {}
    for name in ['hyperreal', 'serde_json']:
        artifacts = [r for r in rows(raw) if r.get('reason') == 'compiler-artifact'
            and r['target']['name'] == name and any(p.endswith('.rlib') for p in r['filenames'])]
        assert len(artifacts) == 1
        artifact = artifacts[0]
        path = Path(next(p for p in artifact['filenames'] if p.endswith('.rlib')))
        assert path.is_relative_to(target)
        if name == 'hyperreal':
            assert Path(artifact['manifest_path']) == source / 'crates/hyperreal/Cargo.toml'
        assert artifact['features'] == libraries['borrowed'][name]['features']
        libraries['classified'][name] = {'path': str(path), 'sha256': sha(path), 'bytes': path.stat().st_size,
            'features': artifact['features'], 'manifest_path': artifact['manifest_path']}
    assert all({name: row['features'] for name, row in libraries[group].items()} ==
               {name: row['features'] for name, row in libraries['classified'].items()} for group in GROUPS)
    dump(out / 'libraries.json', libraries)
    raw = run('scalar-library', [binaries['library-tests']['path'], '--test-threads=2'])
    old = test_names(gzip.decompress((PARENT / 'qualification-v1/candidate-scalar-all.stdout.gz').read_bytes()))
    expected = {b'computable::node::persistent_laurent_cache_tests::diagnostic_cache_counters_observe_publication_and_zero_work_replay',
                b'computable::node::persistent_laurent_cache_tests::diagnostic_lookup_classes_observe_actual_cache_states'}
    actual = test_names(raw)
    assert len(old) == 879 and actual == old | expected and len(actual) == 881
    assert b'test result: ok. 881 passed; 0 failed; 0 ignored;' in raw
    for fragment, count in [(b'quadratic_tower_sign_retention_tests::', 11), (b'persistent_laurent_cache_tests::', 8),
                            (b'laurent_cache_eligibility_tests::', 3), (b'borrowed_integer_tests::', 2)]:
        assert sum(fragment in name for name in actual) == count
    dump(out / 'scalar-tests.json', {'passed': 881, 'prior_879_test_names_retained': True,
        'counter_controls': [name.decode() for name in sorted(expected)],
        'P4': 11, 'persistent_cache': 8, 'eligibility': 3, 'borrowed_integer': 2})
    for suite in SUITES:
        for group in GROUPS:
            own = libraries[group]
            dependency = Path(own['hyperreal']['path']).parent
            if dependency.name != 'deps':
                dependency /= 'deps'
            assert dependency.is_dir()
            flags = ['--edition=2024', '-O', '-D', 'warnings', str(PREP / (suite + '_probe.rs')),
                     '-L', 'dependency=' + str(dependency)]
            for name, row in own.items():
                flags += ['--extern', name + '=' + row['path']]
            if group == 'classified':
                flags += ['--cfg', 'feature="cache-events"']
            key = suite + '-' + group
            run('clippy-' + key, ['clippy-driver', *flags, '--emit=metadata', '-o', str(bindir / (key + '.rmeta'))], ROOT)
            binary = bindir / ('probe-' + key)
            run('link-' + key, ['rustc', *flags, '-o', str(binary)], ROOT)
            binaries[key] = {'path': str(binary), 'sha256': sha(binary), 'bytes': binary.stat().st_size}
            dump(out / 'binaries.json', binaries)
    assert len(commands) == 29 and len(binaries) == 13
    counts = {'rational': (48, 24), 'quadratic': (64, 32), 'integer': (24, 12)}
    for suite in SUITES:
        reference = gzip.decompress((PARENT / 'scalar-costs-v1' / ('sample-' + suite + '-time-1-main.stdout.gz')).read_bytes())
        phases = [r['name'] for r in rows(reference) if 'name' in r]
        for repetition, order in enumerate(ORDERS, 1):
            for position, group in enumerate(order, 1):
                name = f'count-{suite}-{repetition}-{group}'
                raw = run(name, ['taskset', '-c', '0', binaries[suite + '-' + group]['path'], '4096'], ROOT)
                parsed = rows(raw)
                assert len(raw.splitlines()) == len(parsed) == counts[suite][0]
                assert identity(raw) == identity(reference)
                trace = [r for r in parsed if 'trace_phase' in r]
                assert len(trace) == counts[suite][1] and [r['trace_phase'] for r in trace] == phases
                assert all(r['iterations'] == 4096 for r in trace)
                assert all((r['cache_events'] is not None) == (group == 'classified')
                           and (r['lookup_classes'] is not None) == (group == 'classified') for r in trace)
                samples.append({'suite': suite, 'repetition': repetition, 'position': position, 'group': group,
                    'command_name': name, 'trace': trace, 'identity_matches_closed_cost_trial': True})
                dump(out / 'samples.json', samples)
    assert len(commands) == 53 and len(samples) == 24
    guard(full=True)
    result = {'complete': True, 'commands': 53, 'scalar_tests': 881, 'fixed_count_processes': 24,
        'trace_windows': 544, 'classified_windows': 136, 'new_driver_binaries': 12,
        'all_witnesses_and_representation_inventories_match': True,
        'all_builds_and_tests_precede_count_samples': True, 'all_prior_sources_and_artifacts_unchanged': True,
        'artifact_sha256': {p.name: sha(p) for p in sorted(out.iterdir()) if p.is_file()},
        'no_timing_or_allocation_comparison': True, 'new_native_gates_run': False,
        'runtime_candidate': False, 'runtime_admitted': False, 'plan_complete': False}
    dump(out / 'result.json', result)
    print(json.dumps({k: v for k, v in result.items() if k != 'artifact_sha256'}), flush=True)


if __name__ == '__main__':
    main()
