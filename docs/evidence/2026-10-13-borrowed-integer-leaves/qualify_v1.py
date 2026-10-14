"""Qualify exact scalar behavior and original integer allocations; no timing admission."""
import gzip
import json
import os
from pathlib import Path
import re
import subprocess

from common_v1 import E, ROOT, PARENT, PREP, WORK, ENV, sha, dump, verify


def records(raw):
    return [json.loads(line) for line in raw.splitlines() if line.startswith(b'{')]


def test_names(raw):
    return set(re.findall(rb'^test ([^ ]+) \.\.\. ok$', raw, flags=re.MULTILINE))


def main():
    prep = json.loads((PREP / 'protocol.json').read_text())
    verify(prep, full=True)
    review = json.loads((E / 'source-review-v1.json').read_text())
    assert review['preparation_sha256'] == sha(PREP / 'protocol.json')
    assert review['source_patch_sha256'] == prep['source_patch_sha256']
    assert review['all_four_paths_reviewed']
    assert prep['compiler'] == subprocess.check_output(['rustc', '-vV'], text=True)
    assert prep['inherited_rustflags'] == os.environ.get('RUSTFLAGS')
    out = E / 'qualification-v1'
    out.mkdir(exist_ok=False)
    source, target = Path(prep['source']), Path(prep['target'])
    old_bins = json.loads((PARENT / 'library-costs-v1/binaries.json').read_text())
    binaries = {'main': old_bins['baseline-integer-memory'],
                'full-cache': old_bins['candidate-integer-memory']}
    parent_lib = json.loads((PARENT / 'library-costs-v1/libraries.json').read_text())['candidate']['hyperreal']
    parent_variants = json.loads((PARENT / 'preparation-v1/protocol.json').read_text())['variants']
    protocol = {'scope': __doc__, 'preparation_sha256': sha(PREP / 'protocol.json'),
                'review_sha256': sha(E / 'source-review-v1.json'), 'runner_sha256': sha(__file__),
                'environment': ENV | {'CARGO_TARGET_DIR': str(target)},
                'source': str(source), 'target': str(target),
                'orders': [['main', 'full-cache', 'candidate'], ['candidate', 'full-cache', 'main']],
                'inputs_each': 1000, 'fixed_processes': 6, 'all_12_original_integer_phases': True,
                'allocation_only': True, 'printed_timings_not_analyzed': True,
                'original_binaries_reused_not_samples': True,
                'full_library_cost_matrix_pending': True, 'runtime_admitted': False, 'plan_complete': False}
    dump(out / 'protocol.json', protocol)
    commands, samples, library = [], [], None

    def guard(full=False):
        verify(prep, full=full)
        assert sha(E / 'source-review-v1.json') == protocol['review_sha256']
        assert all(sha(r['path']) == r['sha256'] for r in binaries.values())
        if library:
            assert sha(library['path']) == library['sha256']

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

    run('clippy', ['cargo', 'clippy', '--locked', '--release', '-p', 'hyperreal',
                   '--all-features', '--all-targets', '--', '-D', 'warnings'])
    raw = run('build-tests', ['cargo', 'test', '--locked', '--release', '-p', 'hyperreal',
                             '--all-features', '--lib', '--no-run', '--message-format=json'])
    artifacts = [r for r in records(raw) if r.get('reason') == 'compiler-artifact' and r.get('executable')
                 and r['target']['name'] == 'hyperreal' and r['target']['kind'] == ['lib'] and r['profile']['test']]
    assert len(artifacts) == 1
    test_artifact = artifacts[0]
    test_binary = Path(test_artifact['executable'])
    assert test_binary.is_relative_to(target)
    binaries['tests'] = {'path': str(test_binary), 'sha256': sha(test_binary), 'bytes': test_binary.stat().st_size}
    raw = run('build-library', ['cargo', 'build', '--locked', '--release', '-p', 'hyperreal',
                               '--all-features', '--lib', '--message-format=json'])
    artifacts = [r for r in records(raw) if r.get('reason') == 'compiler-artifact'
                 and r['target']['name'] == 'hyperreal' and any(p.endswith('.rlib') for p in r['filenames'])]
    assert len(artifacts) == 1
    artifact = artifacts[0]
    path = Path(next(p for p in artifact['filenames'] if p.endswith('.rlib')))
    assert path.is_relative_to(target) and Path(artifact['manifest_path']) == source / 'crates/hyperreal/Cargo.toml'
    assert artifact['features'] == parent_lib['features']
    library = {'path': str(path), 'sha256': sha(path), 'bytes': path.stat().st_size,
               'features': artifact['features'], 'manifest_path': artifact['manifest_path']}
    dump(out / 'library.json', library)
    raw = run('scalar-library', [str(test_binary), '--test-threads=2'])
    old_raw = gzip.decompress((ROOT / 'docs/evidence/2026-10-13-construction-cache-coverage/diagnostic-v1/library-tests.stdout.gz').read_bytes())
    recorded = test_names(old_raw)
    diagnostic_only = b'computable::node::persistent_laurent_cache_tests::diagnostic_cache_counters_observe_publication_and_zero_work_replay'
    assert len(recorded) == 875 and diagnostic_only in recorded
    before, after = recorded - {diagnostic_only}, test_names(raw)
    expected = {b'rational::arithmetic::borrowed_integer_tests::borrowed_integer_preserves_full_width_sign_and_magnitude',
                b'rational::arithmetic::borrowed_integer_tests::borrowed_integer_reuses_only_bounded_canonical_storage'}
    assert len(before) == 874 and before <= after and after - before == expected
    assert re.search(rb'test result: ok\. 876 passed; 0 failed; 0 ignored;', raw)
    dump(out / 'scalar-tests.json', {'passed': 876, 'original_874_retained': True,
        'new_tests': [x.decode() for x in sorted(expected)], 'P4_tests': sum(b'quadratic_tower_sign_retention_tests::' in x for x in after),
        'persistent_cache_tests': sum(b'persistent_laurent_cache_tests::' in x for x in after)})
    probe = PARENT / 'probes-v1/integer_probe.rs'
    binary = WORK / 'integer-memory'
    run('link-integer-memory', ['rustc', '--edition=2024', '-O', '-D', 'warnings', str(probe),
        '-L', 'dependency=' + str(target / 'release/deps'), '--extern', 'hyperreal=' + library['path'],
        '--cfg', 'feature="count-alloc"', '-o', str(binary)])
    binaries['candidate'] = {'path': str(binary), 'sha256': sha(binary), 'bytes': binary.stat().st_size}
    assert len({binaries[label]['sha256'] for label in ['main', 'full-cache', 'candidate']}) == 3
    dump(out / 'binaries.json', binaries)
    old_rows = records(gzip.decompress((PARENT / 'library-costs-v1/sample-integer-memory-1-baseline.stdout.gz').read_bytes()))
    identity = [r for r in old_rows if 'witness' in r or 'fixture' in r]
    phases = [r['name'] for r in old_rows if 'name' in r]
    assert len(identity) == len(phases) == 12
    for round_index, order in enumerate(protocol['orders'], 1):
        for label in order:
            cwd = source if label == 'candidate' else Path(parent_variants['baseline' if label == 'main' else 'candidate']['source'])
            raw = run('allocations-' + str(round_index) + '-' + label,
                      ['taskset', '-c', '0', binaries[label]['path'], '1000'], cwd)
            rows = records(raw)
            assert [r for r in rows if 'witness' in r or 'fixture' in r] == identity
            costs = [r for r in rows if 'name' in r]
            assert [r['name'] for r in costs] == phases
            assert all(r['iterations'] == 1000 and r['allocation_accounting'] for r in costs)
            samples.append({'round': round_index, 'variant': label, 'rows': costs})
            dump(out / 'samples.json', samples)
    guard(full=True)
    result = {'complete': True, 'commands': len(commands), 'scalar_tests': 876,
              'fixed_allocation_processes': 6, 'allocation_rows': 72,
              'all_original_witnesses_and_representation_inventories_match': True,
              'all_prior_sources_and_artifacts_unchanged': True,
              'all_builds_and_tests_precede_allocation_samples': True,
              'printed_timings_not_analyzed': True, 'full_library_cost_matrix_pending': True,
              'artifact_sha256': {p.name: sha(p) for p in sorted(out.iterdir()) if p.is_file()},
              'runtime_candidate': True, 'runtime_admitted': False, 'plan_complete': False}
    dump(out / 'result.json', result)
    print(json.dumps({k: v for k, v in result.items() if k != 'artifact_sha256'}), flush=True)


if __name__ == '__main__':
    main()
