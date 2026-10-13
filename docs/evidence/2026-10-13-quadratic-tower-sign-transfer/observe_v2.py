"""Observe the fixed P4 compaction cases on current main and endpoint baseline.

Both immutable sources get fresh, separate all-feature scalar build targets.
Every build/link is terminal before the 24 fresh observation processes start.
Unknown, uncovered compaction, contradictions and process failures are retained;
this collects facts and never infers a performance result or runtime admission.
"""
import gzip
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time

E = Path(__file__).resolve().parent
ROOT = E.parents[2]
PRIOR = ROOT / 'docs/evidence/2026-10-13-coefficient-zero-radicals'
WORK = ROOT / 'target/quadratic-tower-sign-transfer-v2'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
sys.path.insert(0, str(ROOT / 'docs/evidence/2026-10-12-existing-worktree'))
from validate import sources


def main():
    out = E / 'observation-v2'
    out.mkdir(exist_ok=False)
    WORK.mkdir(exist_ok=False)
    (WORK / 'bins').mkdir()
    preparation = json.loads((PRIOR / 'preparation-v1/protocol.json').read_text())
    rejection = json.loads((PRIOR / 'decision-v1.json').read_text())
    assert rejection['decision'] == 'reject-zero-radical-guards-on-broad-native-costs-and-original-board-failure'
    assert json.loads((PRIOR / 'native-validation-v1/result.json').read_text())['complete']
    assert json.loads((PRIOR / 'native-controls-v1/result.json').read_text())['complete']
    assert not (PRIOR / 'report-replay-v1').exists()
    parent = json.loads(gzip.decompress((PRIOR / 'preparation-v1/parent-source-hashes.json.gz').read_bytes()))
    main_guard = sources()
    assert main_guard == preparation['main_source_sha256']
    pending = preparation['pending_native_sha256']
    endpoint = preparation['variants']['baseline']
    endpoint_guard = parent | endpoint['source_changes_sha256']
    assert len(parent) == 10130 and len(endpoint_guard) == 10131 and len(pending) == 35
    variants = {
        'main': {'source': str(ROOT), 'target': str(WORK / 'main-target'), 'source_count': len(main_guard)},
        'endpoint': {'source': endpoint['source'], 'target': str(WORK / 'endpoint-target'), 'source_count': len(endpoint_guard)},
    }
    algebra = 'crates/hyperreal/src/computable/node/algebra.rs'
    assert (Path(endpoint['source']) / algebra).read_text().removeprefix(
        '#[cfg(test)]\nmod coefficient_zero_radical_tests;\n\n') == (Path(preparation['parent_source']) / algebra).read_text()
    cases = [{'family': family, 'shape': shape, 'scale': scale}
             for family in ['quadratic', 'nested', 'biquadratic']
             for shape in ['linear', 'square'] for scale in [1, -7]]
    env = {'CARGO_HOME': '/tmp/csgrs-properties-cargo', 'CCACHE_DIR': '/tmp/csgrs-properties-ccache',
           'CARGO_NET_OFFLINE': 'true', 'CARGO_TERM_COLOR': 'never'}
    for name, entries in [('main', main_guard), ('endpoint', endpoint_guard), ('parent', parent)]:
        (out / (name + '-source-hashes.json.gz')).write_bytes(gzip.compress(
            (json.dumps(entries, sort_keys=True) + '\n').encode(), mtime=0))
    protocol = {
        'scope': __doc__, 'runner_sha256': sha(Path(__file__)), 'assessor_sha256': sha(E / 'assess_v2.py'),
        'probe_sha256': sha(E / 'probe_v2.rs'), 'prior_compile_failure_sha256': sha(E / 'failure-v1.json'), 'previous_goal_turn_classification': 'progress',
        'previous_goal_turn_evidence': 'Ten commits through 9025de62a close diagnostics, fixed scalar/native costs and rejection with original failures preserved.',
        'base_commit': subprocess.check_output(['git', 'rev-parse', 'HEAD'], text=True).strip(),
        'prior_preparation_sha256': sha(PRIOR / 'preparation-v1/protocol.json'),
        'prior_rejection_sha256': sha(PRIOR / 'decision-v1.json'),
        'prior_terminal_results_sha256': {name: sha(PRIOR / name) for name in
                                         ['native-validation-v1/result.json', 'native-controls-v1/result.json']},
        'variants': variants,
        'variant_order': ['main', 'endpoint'], 'cases': cases, 'fresh_processes': 24,
        'parent_source': preparation['parent_source'], 'pending_native_sha256': pending,
        'environment': env, 'inherited_rustflags': os.environ.get('RUSTFLAGS'),
        'compiler': subprocess.check_output(['rustc', '-vV'], text=True),
        'build_timeout_seconds': 1800, 'observation_timeout_seconds': 120,
        'precision_scope': {'lower_interval': -1024, 'delta_warming': -1100, 'subsequent_queries': [0, -512, 0]},
        'limits': 'The endpoint baseline runtime equals the unadmitted endpoint parent and adds only test code. Main and endpoint observations are separate scopes. The prepared probe does not establish universal cache monotonicity, abort/domain behavior, numerical error bounds, native gates or performance.',
        'runtime_admitted': False,
    }
    (out / 'protocol.json').write_text(json.dumps(protocol, indent=2) + '\n')
    commands, libraries, artifacts, binaries, observations = [], {}, {}, {}, []

    def verify():
        assert sha(Path(__file__)) == protocol['runner_sha256']
        assert sha(E / 'assess_v2.py') == protocol['assessor_sha256']
        assert sha(E / 'probe_v2.rs') == protocol['probe_sha256']
        assert sha(E / 'failure-v1.json') == protocol['prior_compile_failure_sha256']
        assert sha(PRIOR / 'preparation-v1/protocol.json') == protocol['prior_preparation_sha256']
        assert sha(PRIOR / 'decision-v1.json') == protocol['prior_rejection_sha256']
        assert all(sha(PRIOR / name) == digest for name, digest in protocol['prior_terminal_results_sha256'].items())
        assert sources() == main_guard
        assert all(sha(ROOT / name) == digest for name, digest in pending.items())
        assert all(sha(Path(preparation['parent_source']) / name) == digest for name, digest in parent.items())
        for row in preparation['variants'].values():
            assert all(sha(Path(row['source']) / name) == digest
                       for name, digest in (parent | row['source_changes_sha256']).items())
        assert all(sha(Path(row['path'])) == row['sha256'] for row in libraries.values())
        assert all(sha(Path(path)) == digest for entries in artifacts.values() for path, digest in entries.items())
        assert all(sha(Path(row['path'])) == row['sha256'] for row in binaries.values())

    def run(name, command, cwd=ROOT, extra_env=None, observation=False):
        verify()
        limit = 120 if observation else 1800
        wrapped = ['timeout', '--signal=TERM', '--kill-after=10s', str(limit), *command]
        print(json.dumps({'start': name}), flush=True)
        started = time.monotonic()
        with (out / (name + '.stdout')).open('w') as stdout, (out / (name + '.stderr')).open('w') as stderr:
            code = subprocess.run(wrapped, cwd=cwd, env=os.environ | env | (extra_env or {}),
                                  stdout=stdout, stderr=stderr).returncode
        row = {'name': name, 'command': wrapped, 'cwd': str(cwd), 'extra_environment': extra_env or {},
               'exit_code': code, 'seconds': time.monotonic() - started,
               'stdout_sha256': sha(out / (name + '.stdout')), 'stderr_sha256': sha(out / (name + '.stderr'))}
        commands.append(row)
        (out / 'commands.json').write_text(json.dumps(commands, indent=2) + '\n')
        print(json.dumps({key: row[key] for key in ['name', 'exit_code', 'seconds']}), flush=True)
        verify()
        if not observation:
            assert code == 0, name
        return code, (out / (name + '.stdout')).read_text()

    for variant, item in variants.items():
        directory, target = Path(item['source']), Path(item['target'])
        _, raw = run('build-' + variant, ['cargo', 'build', '--locked', '--release', '-p', 'hyperreal',
                                         '--all-features', '--lib', '--message-format=json'], directory,
                     {'CARGO_TARGET_DIR': str(target)})
        events = [json.loads(line) for line in raw.splitlines() if line.startswith('{')]
        all_artifacts = [event for event in events if event.get('reason') == 'compiler-artifact']
        matches = [event for event in all_artifacts if event['target']['name'] == 'hyperreal'
                   and any(path.endswith('.rlib') for path in event['filenames'])]
        assert len(matches) == 1
        artifact = matches[0]
        library = Path(next(path for path in artifact['filenames'] if path.endswith('.rlib')))
        assert library.is_relative_to(target)
        assert Path(artifact['manifest_path']) == directory / 'crates/hyperreal/Cargo.toml'
        assert artifact['target']['kind'] == ['lib'] and not artifact['profile']['test']
        libraries[variant] = {'path': str(library), 'sha256': sha(library), 'bytes': library.stat().st_size,
                              'manifest_path': artifact['manifest_path'], 'features': artifact['features'],
                              'package_id': artifact['package_id'], 'profile': artifact['profile']}
        artifacts[variant] = {path: sha(Path(path)) for event in all_artifacts for path in event['filenames']
                              if path.endswith(('.rlib', '.so'))}
        (out / 'libraries.json').write_text(json.dumps(libraries, indent=2) + '\n')
        (out / 'build-artifacts.json').write_text(json.dumps(artifacts, indent=2) + '\n')
    assert libraries['main']['features'] == libraries['endpoint']['features']
    for variant, item in variants.items():
        binary = WORK / 'bins' / variant
        run('link-' + variant, ['rustc', '--edition=2024', '-O', '-D', 'warnings', str(E / 'probe_v2.rs'),
                                '-L', 'dependency=' + str(Path(item['target']) / 'release/deps'),
                                '--extern', 'hyperreal=' + libraries[variant]['path'], '-o', str(binary)])
        binaries[variant] = {'path': str(binary), 'sha256': sha(binary), 'bytes': binary.stat().st_size}
        (out / 'binaries.json').write_text(json.dumps(binaries, indent=2) + '\n')
    # No builds overlap any observation; every case has fresh process-global caches.
    for variant in protocol['variant_order']:
        for index, case in enumerate(cases, 1):
            name = f'{variant}-{index:02d}-{case["family"]}-{case["shape"]}-{case["scale"]}'
            code, raw = run(name, [binaries[variant]['path'], case['family'], case['shape'], str(case['scale'])], observation=True)
            data, error = None, None
            try:
                parsed = [json.loads(line) for line in raw.splitlines()]
                assert len(parsed) == 1 and isinstance(parsed[0], dict)
                data = parsed[0]
                assert {key: data[key] for key in case} == case
            except (ValueError, AssertionError, KeyError) as failure:
                error = type(failure).__name__ + ': ' + str(failure)
            observations.append({'name': name, 'variant': variant, 'case': case, 'exit_code': code,
                                 'row': data, 'parse_error': error})
            (out / 'observations.json').write_text(json.dumps(observations, indent=2) + '\n')
    verify()
    assert len(observations) == 24
    result = {'collection_complete': True,
              'complete': all(row['exit_code'] == 0 and row['parse_error'] is None for row in observations),
              'observation_processes': len(observations),
              'process_failures': [row['name'] for row in observations if row['exit_code'] != 0],
              'invalid_output': [row['name'] for row in observations if row['parse_error'] is not None],
              'sources_main_libraries_and_binaries_unchanged': True, 'runtime_admitted': False}
    (out / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result), flush=True)


if __name__ == '__main__':
    main()
