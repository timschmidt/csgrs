"""Qualify counter instrumentation, then run two fixed diagnostic processes."""
import gzip
import json
import os
from pathlib import Path
import re
import subprocess

from common_v1 import E, ROOT, OLD, PREP, WORK, ENV, sha, dump, verify


def parse(data):
    return [json.loads(line) for line in data.splitlines() if line.startswith(b'{')]


def main():
    prep = json.loads((PREP / 'protocol.json').read_text())
    verify(prep, full=True)
    source, target = Path(prep['source']), Path(prep['target'])
    review = E / 'source-review-v1.json'
    review_data = json.loads(review.read_text())
    assert review_data['all_five_paths_reviewed']
    assert review_data['preparation_sha256'] == sha(PREP / 'protocol.json')
    assert prep['compiler'] == subprocess.check_output(['rustc', '-vV'], text=True)
    assert prep['inherited_rustflags'] == os.environ.get('RUSTFLAGS')
    out = E / 'diagnostic-v1'
    out.mkdir(exist_ok=False)
    protocol = {'scope': __doc__, 'runner_sha256': sha(__file__),
                'preparation_sha256': sha(PREP / 'protocol.json'), 'review_sha256': sha(review),
                'scope_sha256': sha(E / 'scope-v1.json'), 'source': str(source), 'target': str(target),
                'environment': ENV | {'CARGO_TARGET_DIR': str(target)},
                'compiler': prep['compiler'], 'inherited_rustflags': os.environ.get('RUSTFLAGS'),
                'fixed_processes': 2, 'inputs_each': 4096, 'diagnostic_only': True,
                'runtime_admitted': False}
    dump(out / 'protocol.json', protocol)
    commands, binaries, libraries, samples = [], {}, {}, []

    def guard(full=False):
        verify(prep, full=full)
        assert sha(PREP / 'protocol.json') == protocol['preparation_sha256']
        assert sha(review) == protocol['review_sha256']
        assert sha(PREP / 'probe.rs') == prep['probe_sha256']
        for row in [*binaries.values(), *libraries.values()]:
            assert sha(row['path']) == row['sha256']

    def run(name, command):
        guard()
        print(json.dumps({'start': name}), flush=True)
        completed = subprocess.run(['timeout', '--signal=TERM', '--kill-after=10s', '1800', *command],
                                   cwd=source, env=os.environ | protocol['environment'], capture_output=True)
        row = {'name': name, 'command': command, 'cwd': str(source), 'exit_code': completed.returncode}
        for stream in ['stdout', 'stderr']:
            path = out / (name + '.' + stream + '.gz')
            path.write_bytes(gzip.compress(getattr(completed, stream), mtime=0))
            row[stream + '_sha256'] = sha(path)
        commands.append(row)
        dump(out / 'commands.json', commands)
        print(json.dumps({'finished': name, 'exit_code': completed.returncode}), flush=True)
        assert completed.returncode == 0, name
        guard()
        return completed.stdout

    run('clippy', ['cargo', 'clippy', '--locked', '--release', '-p', 'hyperreal',
                  '--all-features', '--all-targets', '--', '-D', 'warnings'])
    raw = run('build-library', ['cargo', 'build', '--locked', '--release', '-p', 'hyperreal',
                                '--all-features', '--lib', '--message-format=json'])
    events = parse(raw)
    for name in ['hyperreal', 'serde_json']:
        artifacts = [r for r in events if r.get('reason') == 'compiler-artifact' and r['target']['name'] == name
                     and any(p.endswith('.rlib') for p in r['filenames'])]
        assert len(artifacts) == 1
        artifact = artifacts[0]
        path = Path(next(p for p in artifact['filenames'] if p.endswith('.rlib')))
        assert path.is_relative_to(target)
        if name == 'hyperreal':
            assert Path(artifact['manifest_path']) == source / 'crates/hyperreal/Cargo.toml'
            assert artifact['features'] == prep['prior_libraries'][name]['features']
        libraries[name] = {'path': str(path), 'sha256': sha(path), 'bytes': path.stat().st_size,
                           'manifest_path': artifact['manifest_path'], 'features': artifact['features']}
    dump(out / 'libraries.json', libraries)
    raw = run('build-tests', ['cargo', 'test', '--locked', '--release', '-p', 'hyperreal',
                              '--all-features', '--lib', '--no-run', '--message-format=json'])
    artifacts = [r for r in parse(raw) if r.get('reason') == 'compiler-artifact' and r.get('executable')
                 and r['target']['name'] == 'hyperreal' and r['target']['kind'] == ['lib'] and r['profile']['test']]
    assert len(artifacts) == 1
    artifact = artifacts[0]
    path = Path(artifact['executable'])
    assert path.is_relative_to(target)
    assert Path(artifact['manifest_path']) == source / 'crates/hyperreal/Cargo.toml'
    binaries['tests'] = {'path': str(path), 'sha256': sha(path), 'bytes': path.stat().st_size}
    dump(out / 'binaries.json', binaries)
    raw = run('library-tests', [str(path), '--test-threads=1'])
    assert b'test result: ok. 875 passed; 0 failed; 0 ignored;' in raw
    names = set(re.findall(rb'^test (\S+) \.\.\. ok$', raw, re.M))
    old_raw = gzip.decompress((OLD / 'library-costs-v1/candidate-scalar-library.stdout.gz').read_bytes())
    old_names = set(re.findall(rb'^test (\S+) \.\.\. ok$', old_raw, re.M))
    assert len(old_names) == 868 and len(names) == 875 and old_names <= names
    extra = names - old_names
    assert len(extra) == 7 and all(b'persistent_laurent_cache_tests::' in n for n in extra)
    assert sum(b'diagnostic_cache_counters_observe_publication_and_zero_work_replay' in n for n in extra) == 1
    dump(out / 'qualification.json', {'library_tests_passed': 875, 'all_prior_868_scalar_tests_preserved': True,
                                     'existing_cache_tests_passed': 6, 'new_counter_sanity_tests_passed': 1,
                                     'extra_test_names': sorted(n.decode() for n in extra),
                                     'clippy_all_features_all_targets_warnings_denied': True})
    binary = WORK / 'cache-coverage-probe'
    command = ['rustc', '--edition=2024', '-O', '-D', 'warnings', str(PREP / 'probe.rs'),
               '-L', 'dependency=' + str(target / 'release/deps')]
    for name, row in libraries.items():
        command += ['--extern', name + '=' + row['path']]
    run('link-probe', command + ['-o', str(binary)])
    binaries['probe'] = {'path': str(binary), 'sha256': sha(binary), 'bytes': binary.stat().st_size}
    dump(out / 'binaries.json', binaries)
    reference = json.loads((OLD / 'construction-trace-v1/samples.json').read_text())
    reference = [r['trace'] for r in reference if r['variant'] == 'full-cache']
    assert len(reference) == 2 and reference[0] == reference[1]
    gold = gzip.decompress((OLD / 'construction-trace-v1/trace-1-full-cache.stdout.gz').read_bytes())
    gold_identity = [r for r in parse(gold) if 'fixture' in r or 'witness' in r]
    assert len(gold_identity) == 32
    for repetition in [1, 2]:
        raw = run('trace-' + str(repetition), ['taskset', '-c', '0', str(binary), '4096'])
        rows = parse(raw)
        assert [r for r in rows if 'fixture' in r or 'witness' in r] == gold_identity
        trace = [r for r in rows if 'trace_phase' in r]
        assert len(trace) == 32
        assert [{k: v for k, v in r.items() if k != 'cache_events'} for r in trace] == reference[0]
        assert all(len(r['cache_events']) == 27 for r in trace)
        samples.append({'repetition': repetition, 'rows': trace})
        dump(out / 'samples.json', samples)
    assert samples[0]['rows'] == samples[1]['rows']
    total = {name: sum(r['cache_events'][name] for r in samples[0]['rows'])
             for name in samples[0]['rows'][0]['cache_events']}
    assert all(total[name] > 0 for name in ['project_entry', 'retained_visit', 'retained_lookup',
                                          'retained_hit', 'closed_copy_success', 'store_published',
                                          'evaluation_cell_allocated'])
    guard(full=True)
    result = {'complete': True, 'commands': len(commands), 'library_tests_passed': 875,
              'fixed_processes': 2, 'trace_rows': 64, 'event_fields_per_row': 27,
              'all_prior_public_counts_witnesses_and_inventories_identical': True,
              'all_event_counts_repeat_exactly': True, 'positive_cache_event_controls': True,
              'all_sources_and_prior_artifacts_unchanged': True,
              'timing_or_allocation_comparison': False,
              'artifact_sha256': {p.name: sha(p) for p in sorted(out.iterdir()) if p.is_file()},
              'runtime_candidate': False, 'runtime_admitted': False, 'plan_complete': False}
    dump(out / 'result.json', result)
    print(json.dumps({k: v for k, v in result.items() if k != 'artifact_sha256'}), flush=True)


if __name__ == '__main__':
    main()
